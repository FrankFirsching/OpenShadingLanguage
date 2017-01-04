/*
Copyright (c) 2009-2010 Sony Pictures Imageworks Inc., et al.
All Rights Reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
* Neither the name of Sony Pictures Imageworks nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <OpenImageIO/thread.h>
#include <boost/thread/tss.hpp>   /* for thread_specific_ptr */

#include "OSL/oslconfig.h"
#include "OSL/llvm_util.h"

#if OSL_LLVM_VERSION < 37
#error "LLVM minimum version required for OSL is 3.7"
#endif

#define USE_ORC_JIT (OSL_LLVM_VERSION >= 37)

#include <llvm/ADT/STLExtras.h>
#include <llvm/Bitcode/ReaderWriter.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/RuntimeDyld.h>
#include <llvm/IR/DataLayout.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Mangler.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Support/DynamicLibrary.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Target/TargetMachine.h>
#include <llvm/Transforms/IPO.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/Transforms/Scalar.h>
#include <llvm/Transforms/Scalar/GVN.h>
#include <llvm/Transforms/Utils/UnifyFunctionExitNodes.h>

#if USE_ORC_JIT
#include <llvm/ExecutionEngine/Orc/CompileUtils.h>
#include <llvm/ExecutionEngine/Orc/IRCompileLayer.h>
#include <llvm/ExecutionEngine/Orc/LambdaResolver.h>
#include <llvm/ExecutionEngine/Orc/ObjectLinkingLayer.h>
#endif


OSL_NAMESPACE_ENTER

namespace {
static OIIO::spin_mutex llvm_global_mutex;
static bool llvm_setup_done = false;


// Make a globally unique name given a prefix.
std::string
make_unique_name (string_view prefix = "___")
{
    static OIIO::atomic_int counter (0);
    int c = counter++;
    return OIIO::Strutil::format ("%s%d", prefix, c);
}


template <typename T>
static std::vector<T> singletonSet (T t)
{
    std::vector<T> Vec;
    Vec.push_back(std::move(t));
    return Vec;
}


// Quick conversion of string_view to llvm::StringRef.
inline llvm::StringRef SR (string_view s) {
    return llvm::StringRef (s.data(), s.size());
}

// Quick conversion of array_view to llvm::ArrayRef.
template <typename T>
inline llvm::ArrayRef<T> AR (OIIO::array_view<T> t) {
    return llvm::ArrayRef<T> (t.data(), t.size());
}

// Quick conversion of array_view of const objects to llvm::ArrayRef.
template <typename T>
inline llvm::ArrayRef<T> AR (OIIO::array_view<const T> t) {
    return llvm::ArrayRef<T> (t.data(), t.size());
}

};




// LLVM_Util::Impl holds all the LLVM data structures we didn't want to have
// to expose to user code in llvm_util.h, and the methods that need intimate
// access to those data.
//
// Methods that are simply "pass-through" from LLVM_Util (i.e., the
// LLVM_Util::foo() just calls impl->foo()) are not prefixed with a comment,
// just read the commends in llvm_util.h for details.
//
class LLVM_Util::Impl {
public:
    typedef llvm::orc::ObjectLinkingLayer<> ObjLayerT;
    typedef llvm::orc::IRCompileLayer<ObjLayerT> CompileLayerT;
    typedef CompileLayerT::ModuleSetHandleT ModuleHandleT;
    // friend class LLVM_Util;

    Impl () {
        initialize_llvm ();
        m_llvm_context.reset (new llvm::LLVMContext());
        // FIXME: Can we use LLVMGetGlobalContext()? What are the
        // implications of sharing a context across threads or LLVM_Util
        // instances?
        setup_llvm_datatype_aliases ();
        m_llvm_target_machine.reset (llvm::EngineBuilder().selectTarget());
        m_llvm_data_layout.reset (
            new llvm::DataLayout (m_llvm_target_machine->createDataLayout()));

        // Set up ORC JIT. Can be used for many modules.
        m_orc_compilelayer.reset (
            new CompileLayerT(m_orc_objlayer,
                              llvm::orc::SimpleCompiler(*m_llvm_target_machine)));
    }

    ~Impl () {}

    typedef LLVM_Util::IRBuilder IRBuilder;

    /// Set debug level
    void debug (int d) { m_debug = d; }
    int debug () const { return m_debug; }

    void orc_jit (bool enable) { m_use_orc_jit = enable; }
    bool orc_jit () const { return m_use_orc_jit; }

    /// Return a reference to the current context.
    llvm::LLVMContext &context () const { return *m_llvm_context; }

    llvm::Module *module () { return m_llvm_module.get(); }
    std::unique_ptr<llvm::Module> take_module () { return std::move(m_llvm_module); }

    /// Create a new empty module, make it the current module, return a
    /// pointer to it.
    llvm::Module *new_module (string_view id) {
        m_llvm_module.reset (new llvm::Module(make_unique_name(id), context()));
        m_llvm_module->setDataLayout (*m_llvm_data_layout);
        return m_llvm_module.get();
    }

    /// Create a new module, populated with functions from the buffer
    /// bitcode[0..size-1], make it the current module.  The name identifies
    /// the buffer.  If err is not NULL, error messages will be stored
    /// there.
    llvm::Module *module_from_bitcode (string_view bitcode, string_view name,
                                       std::string *err=NULL) {
        if (err)
            err->clear();
        llvm::MemoryBufferRef buf = llvm::MemoryBufferRef(SR(bitcode), SR(name));
        llvm::ErrorOr<std::unique_ptr<llvm::Module> > ModuleOrErr = llvm::parseBitcodeFile (buf, context());
        if (std::error_code EC = ModuleOrErr.getError())
            if (err)
              *err = EC.message();
        m_llvm_module = std::move (ModuleOrErr.get());
        return m_llvm_module.get();
        // Debugging: print all functions in the module
        // for (llvm::Module::iterator i = m->begin(); i != m->end(); ++i)
        //     std::cout << "  found " << i->getName().data() << "\n";
        //    return m;
    }


    /// Set up a new current function that subsequent basic blocks will
    /// be added to.
    void current_function (llvm::Function *func) { m_current_function = func; }

    /// Return a ptr to the current function we're generating.
    llvm::Function *current_function () const { return m_current_function; }

    /// Return the value ptr for the a-th argument of the current function.
    llvm::Value *current_function_arg (int a) {
        DASSERT (m_current_function);
        llvm::Function::arg_iterator arg_it = m_current_function->arg_begin();
        for (int i = 0;  i < a;  ++i)
            ++arg_it;
        return llvm::cast<llvm::Value>(arg_it);
    }


    /// Create a new IR builder with the given block as entry point. If
    /// block is NULL, a new basic block for the current function will be
    /// created.
    void new_builder (llvm::BasicBlock *block=NULL) {
        if (! block)
            block = llvm::BasicBlock::Create (context(), "" /*name*/, current_function());
        m_builder.reset (new IRBuilder (block));
    }

    /// Return the current IR builder.
    IRBuilder &builder () {
        DASSERT (m_builder);
        return *m_builder;
    }

    /// Return a pointer to the current ExecutionEngine.  Create a JITing
    /// ExecutionEngine if one isn't already set up.
    llvm::ExecutionEngine *execengine () {
        return m_llvm_exec.get();
    }

    /// Setup LLVM optimization passes.
    void setup_optimization_passes (int optlevel);

    /// Run the optimization passes.
    void do_optimize () {
        if (! m_llvm_module_passes)        // If caller didn't set up,
            setup_optimization_passes (0); //   use basic default opt.
        m_llvm_module_passes->run (*module());
    }

    /// Wrap ExecutionEngine::InstallLazyFunctionCreator.
    void InstallLazyFunctionCreator (LLVM_Util::FunctionResolver P) {
        m_lazy_func_resolver = P;
    }

    std::string mangle (string_view name) {
        std::string MangledName;
        llvm::raw_string_ostream MangledNameStream (MangledName);
        llvm::Mangler::getNameWithPrefix (MangledNameStream, SR(name),
                                          *m_llvm_data_layout);
        return MangledName;
    }

    ModuleHandleT addModule (std::unique_ptr<llvm::Module> M) {
        // We need a memory manager to allocate memory and resolve symbols
        // for this new module. Create one that resolves symbols by looking
        // back into the JIT. (Stolen from LLVM Kaleidescope example.)
        auto Resolver = llvm::orc::createLambdaResolver(
                // External lookup functor
                [&](const std::string &name) {
                    std::cout << "Resolver '" << name << "'\n";
                    if (auto Sym = findMangledSymbol(name)) {
                        std::cout << " resolver returned a sym\n";
                        return llvm::RuntimeDyld::SymbolInfo(Sym.getAddress(),
                                                             Sym.getFlags());
                    }
                    if (auto Sym = findUnmangledSymbol(name))
                        return llvm::RuntimeDyld::SymbolInfo(Sym.getAddress(),
                                                             Sym.getFlags());
                    if (auto Sym = findDemangledSymbol(name))
                        return llvm::RuntimeDyld::SymbolInfo(Sym.getAddress(),
                                                             Sym.getFlags());
                    if (m_lazy_func_resolver) {
                        void* addr = (*m_lazy_func_resolver)(name);
                        std::cout << "  used func_resolver, addr=" << addr << "\n";
                        if (addr)
                            return llvm::RuntimeDyld::SymbolInfo(uint64_t(addr),
                                                                 llvm::JITSymbolFlags::None);
                    }
                    std::cout << "   checking process for " << name << "\n";
                    if (auto Addr = llvm::RTDyldMemoryManager::getSymbolAddressInProcess(name)) {
                        std::cout << "   FOUND " << name << "\n";
                        return llvm::RuntimeDyld::SymbolInfo(Addr, llvm::JITSymbolFlags::Exported);
                    }
                        std::cout << "   nnot found " << name << "\n";
                    return llvm::RuntimeDyld::SymbolInfo(nullptr);
                },
                // Dylib lookup functor
                [&](const std::string &name) {
                    std::cout << "Dylib Resolver '" << name << "'\n";
                    return nullptr;
                }
            );
        auto H = m_orc_compilelayer->addModuleSet (singletonSet(std::move(M)),
                                                   llvm::make_unique<llvm::SectionMemoryManager>(),
                                                   std::move(Resolver));
        m_module_handles.push_back (H);
        return H;
    }

    void removeModule (ModuleHandleT H) {
        m_module_handles.erase (std::find(m_module_handles.begin(), m_module_handles.end(), H));
        m_orc_compilelayer->removeModuleSet(H);
    }

    llvm::orc::JITSymbol findMangledSymbol (string_view name) {
        // for (auto H : llvm::make_range(m_module_handles.rbegin(), m_module_handles.rend()))
        //     if (auto Sym = m_orc_compilelayer->findSymbolIn(H, name, true))
        //         return Sym;
        if (auto sym = m_orc_compilelayer->findSymbol (name, true))
            return sym;
        // std::cout << "Second try on " << name << "\n";
        // If we can't find the symbol in the JIT, try looking in the host process.
        if (auto symAddr = llvm::RTDyldMemoryManager::getSymbolAddressInProcess(name)) {
            // std::cout << " FOUND mangled in process " << name << "\n";
            return llvm::orc::JITSymbol (symAddr, llvm::JITSymbolFlags::Exported);
        }
        // std::cout << "  whoa, failed second try\n";
        return llvm::orc::JITSymbol(nullptr);
    }

    llvm::orc::JITSymbol findUnmangledSymbol (string_view name) {
        return findMangledSymbol (mangle (name));
    }

    llvm::orc::JITSymbol findDemangledSymbol (string_view name) {
        while (name.size() && name[0] == '_') {
            name.remove_prefix (1);
            if (auto s = findMangledSymbol (name))
                return s;
        }
        return nullptr;
    }

    void module_done () {
        if (orc_jit()) {
            /*auto modulehandle =*/ addModule (take_module());
        } else {
            std::string engine_errors;
            llvm::EngineBuilder engine_builder (take_module());
            engine_builder.setEngineKind (llvm::EngineKind::JIT)
                          .setOptLevel (llvm::CodeGenOpt::Default) // Aggressive?
                          .setErrorStr (&engine_errors);
            m_llvm_exec.reset (engine_builder.create());
            // FIXME: check for errors here, report
            m_llvm_exec->InstallLazyFunctionCreator (m_lazy_func_resolver);
            m_llvm_exec->finalizeObject ();
        }
    }

    void * get_compiled_function (string_view name) {
        if (orc_jit()) {
            auto ExprSymbol = findUnmangledSymbol (name);
            return (void *) ExprSymbol.getAddress ();
        } else {
            ASSERT (m_llvm_exec);
            return (void *) m_llvm_exec->getFunctionAddress (name);
        }
    }

    /// Create a new LLVM basic block (for the current function) and return
    /// its handle. If an IRBuilder hasn't yet been created, do that also.
    /// If insert is true, set the insertion point for new IR ops to be the
    /// new BB.
    llvm::BasicBlock *new_basic_block (string_view name="",
                                       bool insert=false) {
        // Create an IRBuilder if it hasn't already been done, since if the
        // caller creates a block, surely they're about to start adding
        // code.
        if (! m_builder)
            m_builder.reset (new IRBuilder(context()));
        ASSERT (m_current_function);
        auto block = llvm::BasicBlock::Create (context(), SR(name), current_function());
        if (insert)
            m_builder->SetInsertPoint (block);
        return block;
    }

    /// Save the return block pointer when entering a function. If
    /// after==NULL, generate a new basic block for where to go after the
    /// function return.  Return the after BB.
    llvm::BasicBlock *push_function (llvm::BasicBlock *after=NULL) {
        if (! after)
            after = new_basic_block ();
        m_return_block.push_back (after);
        return after;
    }

    /// Pop basic return destination when exiting a function.  This includes
    /// resetting the IR insertion point to the block following the
    /// corresponding function call.
    void pop_function () {
        ASSERT (! m_return_block.empty());
        builder().SetInsertPoint (m_return_block.back());
        m_return_block.pop_back ();
    }

    /// Return the basic block where we go after returning from the current
    /// function.
    llvm::BasicBlock *return_block () const {
        ASSERT (! m_return_block.empty());
        return m_return_block.back();
    }

    /// Save the basic block pointers when entering a loop.
    void push_loop (llvm::BasicBlock *step, llvm::BasicBlock *after) {
        m_loop_step_block.push_back (step);
        m_loop_after_block.push_back (after);
    }

    /// Pop basic block pointers when exiting a loop.
    void pop_loop () {
        ASSERT (! m_loop_step_block.empty() && ! m_loop_after_block.empty());
        m_loop_step_block.pop_back ();
        m_loop_after_block.pop_back ();
    }

    /// Return the basic block of the current loop's 'step' instructions.
    llvm::BasicBlock *loop_step_block () const {
        ASSERT (! m_loop_step_block.empty());
        return m_loop_step_block.back();
    }

    /// Return the basic block of the current loop's exit point.
    llvm::BasicBlock *loop_after_block () const {
        ASSERT (! m_loop_after_block.empty());
        return m_loop_after_block.back();
    }

    llvm::Type *type_float() const { return m_llvm_type_float; }
    llvm::Type *type_int() const { return m_llvm_type_int; }
    llvm::Type *type_addrint() const { return m_llvm_type_addrint; }
    llvm::Type *type_bool() const { return m_llvm_type_bool; }
    llvm::Type *type_char() const { return m_llvm_type_char; }
    llvm::Type *type_longlong() const { return m_llvm_type_longlong; }
    llvm::Type *type_void() const { return m_llvm_type_void; }
    llvm::Type *type_triple() const { return m_llvm_type_triple; }
    llvm::Type *type_matrix() const { return m_llvm_type_matrix; }
    llvm::Type *type_typedesc() const { return m_llvm_type_longlong; }
    llvm::PointerType *type_void_ptr() const { return m_llvm_type_void_ptr; }
    llvm::PointerType *type_string() { return m_llvm_type_char_ptr; }
    llvm::PointerType *type_ustring_ptr() const { return m_llvm_type_ustring_ptr; }
    llvm::PointerType *type_char_ptr() const { return m_llvm_type_char_ptr; }
    llvm::PointerType *type_int_ptr() const { return m_llvm_type_int_ptr; }
    llvm::PointerType *type_float_ptr() const { return m_llvm_type_float_ptr; }
    llvm::PointerType *type_triple_ptr() const { return m_llvm_type_triple_ptr; }
    llvm::PointerType *type_matrix_ptr() const { return m_llvm_type_matrix_ptr; }

    static size_t total_jit_memory_held ();

private:

    void initialize_llvm ();
    void setup_llvm_datatype_aliases ();

    std::unique_ptr<llvm::LLVMContext> m_llvm_context;
    std::unique_ptr<llvm::Module> m_llvm_module;
    std::unique_ptr<llvm::TargetMachine> m_llvm_target_machine;
    std::unique_ptr<llvm::DataLayout> m_llvm_data_layout;
    std::unique_ptr<IRBuilder> m_builder;
    std::unique_ptr<llvm::ExecutionEngine> m_llvm_exec;
    llvm::Function *m_current_function = NULL;
    std::unique_ptr <llvm::legacy::PassManager> m_llvm_module_passes;
    std::unique_ptr <llvm::legacy::FunctionPassManager> m_llvm_func_passes;
    ObjLayerT m_orc_objlayer;
    std::unique_ptr<CompileLayerT> m_orc_compilelayer;
    std::vector<ModuleHandleT> m_module_handles;
    FunctionResolver m_lazy_func_resolver = NULL;
    std::vector<llvm::BasicBlock *> m_return_block;     // stack for func call
    std::vector<llvm::BasicBlock *> m_loop_after_block; // stack for break
    std::vector<llvm::BasicBlock *> m_loop_step_block;  // stack for continue

    llvm::Type *m_llvm_type_float = NULL;
    llvm::Type *m_llvm_type_int = NULL;
    llvm::Type *m_llvm_type_addrint = NULL;
    llvm::Type *m_llvm_type_bool = NULL;
    llvm::Type *m_llvm_type_char = NULL;
    llvm::Type *m_llvm_type_longlong = NULL;
    llvm::Type *m_llvm_type_void = NULL;
    llvm::Type *m_llvm_type_triple = NULL;
    llvm::Type *m_llvm_type_matrix = NULL;
    llvm::PointerType *m_llvm_type_void_ptr = NULL;
    llvm::PointerType *m_llvm_type_ustring_ptr = NULL;
    llvm::PointerType *m_llvm_type_char_ptr = NULL;
    llvm::PointerType *m_llvm_type_int_ptr = NULL;
    llvm::PointerType *m_llvm_type_float_ptr = NULL;
    llvm::PointerType *m_llvm_type_triple_ptr = NULL;
    llvm::PointerType *m_llvm_type_matrix_ptr = NULL;
    int m_debug = 0;
    bool m_use_orc_jit = false;
};



size_t
LLVM_Util::total_jit_memory_held ()
{
    size_t jitmem = 0;
    OIIO::spin_lock lock (llvm_global_mutex);
#if USE_OLD_JIT
    for (size_t i = 0;  i < jitmm_hold.size();  ++i) {
        llvm::JITMemoryManager *mm = jitmm_hold[i].get();
        if (mm)
            jitmem += mm->GetDefaultCodeSlabSize() * mm->GetNumCodeSlabs()
                    + mm->GetDefaultDataSlabSize() * mm->GetNumDataSlabs()
                    + mm->GetDefaultStubSlabSize() * mm->GetNumStubSlabs();
    }
#endif
    return jitmem;
}



void
LLVM_Util::Impl::initialize_llvm ()
{
    OIIO::spin_lock lock (llvm_global_mutex);
    if (llvm_setup_done)
        return;
    // Some global LLVM initialization for the first thread that
    // gets here.

    llvm::InitializeAllTargets();
    // llvm::InitializeAllTargetInfos();
    llvm::InitializeAllTargetMCs();
    llvm::InitializeAllAsmPrinters();
    llvm::InitializeAllAsmParsers();
    // llvm::InitializeAllDisassemblers();

    // This is necessary to make sure that we can find symbols in our
    // own executable.
    llvm::sys::DynamicLibrary::LoadLibraryPermanently (nullptr);

#if 0
    if (debug()) {
        for (llvm::TargetRegistry::iterator t = llvm::TargetRegistry::begin();
             t != llvm::TargetRegistry::end();  ++t) {
            std::cout << "Target: '" << t->getName() << "' "
                      << t->getShortDescription() << "\n";
        }
        std::cout << "\n";
    }
#endif

    llvm_setup_done = true;
}



void
LLVM_Util::Impl::setup_llvm_datatype_aliases ()
{
    // Set up aliases for types we use over and over
    llvm::LLVMContext &ctx (*m_llvm_context);
    m_llvm_type_float = (llvm::Type *) llvm::Type::getFloatTy (ctx);
    m_llvm_type_int = (llvm::Type *) llvm::Type::getInt32Ty (ctx);
    if (sizeof(char *) == 4)
        m_llvm_type_addrint = (llvm::Type *) llvm::Type::getInt32Ty (ctx);
    else
        m_llvm_type_addrint = (llvm::Type *) llvm::Type::getInt64Ty (ctx);
    m_llvm_type_int_ptr = (llvm::PointerType *) llvm::Type::getInt32PtrTy (ctx);
    m_llvm_type_bool = (llvm::Type *) llvm::Type::getInt1Ty (ctx);
    m_llvm_type_char = (llvm::Type *) llvm::Type::getInt8Ty (ctx);
    m_llvm_type_longlong = (llvm::Type *) llvm::Type::getInt64Ty (ctx);
    m_llvm_type_void = (llvm::Type *) llvm::Type::getVoidTy (ctx);
    m_llvm_type_char_ptr = (llvm::PointerType *) llvm::Type::getInt8PtrTy (ctx);
    m_llvm_type_float_ptr = (llvm::PointerType *) llvm::Type::getFloatPtrTy (ctx);
    m_llvm_type_ustring_ptr = (llvm::PointerType *) llvm::PointerType::get (m_llvm_type_char_ptr, 0);
    m_llvm_type_void_ptr = m_llvm_type_char_ptr;

    // A triple is a struct composed of 3 floats
    std::vector<llvm::Type*> triplefields(3, m_llvm_type_float);
    m_llvm_type_triple = llvm::StructType::create(context(), triplefields, "Vec3");
    m_llvm_type_triple_ptr = (llvm::PointerType *) llvm::PointerType::get (m_llvm_type_triple, 0);

    // A matrix is a struct composed 16 floats
    std::vector<llvm::Type*> matrixfields(16, m_llvm_type_float);
    m_llvm_type_matrix = llvm::StructType::create(context(), matrixfields, "Matrix4");
    m_llvm_type_matrix_ptr = (llvm::PointerType *) llvm::PointerType::get (m_llvm_type_matrix, 0);
}




LLVM_Util::LLVM_Util (int debuglevel)
    : m_impl(new Impl())
{
    m_impl->debug (debuglevel);
}



LLVM_Util::~LLVM_Util ()
{
}



void LLVM_Util::debug (int d)
{
    m_impl->debug (d);
}

int LLVM_Util::debug () const
{
    return m_impl->debug();
}



void LLVM_Util::orc_jit (bool enable)
{
    m_impl->orc_jit (enable);
}

bool LLVM_Util::orc_jit () const
{
    return m_impl->orc_jit();
}



llvm::LLVMContext &
LLVM_Util::context () const
{
    return m_impl->context();
}



llvm::Module *
LLVM_Util::module ()
{
    if (! impl()->module())
        new_module ();
    return impl()->module();
}



// void
// LLVM_Util::module (llvm::Module *m)
// {
//     impl()->m_llvm_module.reset (m);
// }



llvm::Module *
LLVM_Util::new_module (string_view id)
{
    return m_impl->new_module (id);
}



llvm::Module *
LLVM_Util::module_from_bitcode (string_view bitcode, string_view name,
                                std::string *err)
{
    return m_impl->module_from_bitcode (bitcode, name, err);
}



LLVM_Util::IRBuilder &
LLVM_Util::builder ()
{
    return m_impl->builder();
}



void *
LLVM_Util::get_compiled_function (string_view name)
{
    return m_impl->get_compiled_function (name);
}



void
LLVM_Util::InstallLazyFunctionCreator (void* (*P)(const std::string &))
{
    m_impl->InstallLazyFunctionCreator (P);
}



void
LLVM_Util::setup_optimization_passes (int optlevel)
{
    m_impl->setup_optimization_passes (optlevel);
}



void
LLVM_Util::Impl::setup_optimization_passes (int optlevel)
{
    // Construct the per-function passes and module-wide (interprocedural
    // optimization) passes.
    //
    // LLVM keeps changing names and call sequence. This part is easier to
    // understand if we explicitly break it into individual LLVM versions.

    m_llvm_func_passes.reset (new llvm::legacy::FunctionPassManager(module()));
    // llvm::legacy::FunctionPassManager &fpm (*m_llvm_func_passes);
//    fpm.add (new llvm::DataLayoutPass());

    m_llvm_module_passes.reset (new llvm::legacy::PassManager);
    llvm::legacy::PassManager &mpm (*m_llvm_module_passes);
    // mpm.add (new llvm::DataLayoutPass());

    if (optlevel >= 1 && optlevel <= 3) {
        // optimize level 1-3 means to use the same set of optimizations as
        // clang -O1, -O2, -O3
        llvm::PassManagerBuilder builder;
        builder.OptLevel = optlevel;
        builder.Inliner = llvm::createFunctionInliningPass();
        // builder.DisableUnrollLoops = true;
        // builder.populateFunctionPassManager (fpm);
        builder.populateModulePassManager (mpm);
    } else {
        // Unknown choices for llvm_optimize: use a basic set of passes that
        // gets the low-hanging fruit for low optimization cost.

        // Always add verifier?
        mpm.add (llvm::createVerifierPass());
        // Simplify the call graph if possible (deleting unreachable blocks, etc.)
        mpm.add (llvm::createCFGSimplificationPass());
        // Change memory references to registers
        //  mpm.add (llvm::createPromoteMemoryToRegisterPass());
        mpm.add (llvm::createSROAPass());
        // Combine instructions where possible -- peephole opts & bit-twiddling
        mpm.add (llvm::createInstructionCombiningPass());
        // Inline small functions
        mpm.add (llvm::createFunctionInliningPass());  // 250?
        // Eliminate early returns
        mpm.add (llvm::createUnifyFunctionExitNodesPass());
        // resassociate exprssions (a = x + (3 + y) -> a = x + y + 3)
        mpm.add (llvm::createReassociatePass());
        // Eliminate common sub-expressions
        mpm.add (llvm::createGVNPass());
        // Constant propagation with SCCP
        mpm.add (llvm::createSCCPPass());
        // More dead code elimination
        mpm.add (llvm::createAggressiveDCEPass());
        // Combine instructions where possible -- peephole opts & bit-twiddling
        mpm.add (llvm::createInstructionCombiningPass());
        // Simplify the call graph if possible (deleting unreachable blocks, etc.)
        mpm.add (llvm::createCFGSimplificationPass());
        // Try to make stuff into registers one last time.
        mpm.add (llvm::createPromoteMemoryToRegisterPass());
    }
}



void
LLVM_Util::do_optimize ()
{
    m_impl->do_optimize ();
}



void
LLVM_Util::module_done ()
{
    m_impl->module_done ();
}



void
LLVM_Util::internalize_module_functions (const std::string &prefix,
                                         const std::vector<std::string> &exceptions,
                                         const std::vector<std::string> &moreexceptions)
{
    for (llvm::Module::iterator iter = module()->begin(); iter != module()->end(); iter++) {
        llvm::Function *sym = (llvm::Function *)(iter);
        std::string symname = sym->getName();
        if (prefix.size() && ! OIIO::Strutil::starts_with(symname, prefix))
            continue;
        bool needed = false;
        for (size_t i = 0, e = exceptions.size(); i < e; ++i)
            if (sym->getName() == exceptions[i]) {
                needed = true;
                // std::cout << "    necessary LLVM module function "
                //           << sym->getName().str() << "\n";
                break;
            }
        for (size_t i = 0, e = moreexceptions.size(); i < e; ++i)
            if (sym->getName() == moreexceptions[i]) {
                needed = true;
                // std::cout << "    necessary LLVM module function "
                //           << sym->getName().str() << "\n";
                break;
            }
        if (!needed) {
            llvm::GlobalValue::LinkageTypes linkage = sym->getLinkage();
            // std::cout << "    unnecessary LLVM module function "
            //           << sym->getName().str() << " linkage " << int(linkage) << "\n";
            if (linkage == llvm::GlobalValue::ExternalLinkage)
                sym->setLinkage (llvm::GlobalValue::LinkOnceODRLinkage);
            // ExternalLinkage means it's potentially externally callable,
            // and so will definitely have code generated.
            // LinkOnceODRLinkage keeps one copy so it can be inlined or
            // called internally to the module, but allows it to be
            // discarded otherwise.
        }
    }
#if 0
    // I don't think we need to worry about linkage of global symbols, but
    // here is an example of how to iterate over the globals anyway.
    for (llvm::Module::global_iterator iter = module()->global_begin(); iter != module()->global_end(); iter++) {
        llvm::GlobalValue *sym = llvm::dyn_cast<llvm::GlobalValue>(iter);
        if (!sym)
            continue;
        std::string symname = sym->getName();
        if (prefix.size() && ! OIIO::Strutil::starts_with(symname, prefix))
            continue;
        bool needed = false;
        for (size_t i = 0, e = exceptions.size(); i < e; ++i)
            if (sym->getName() == exceptions[i]) {
                needed = true;
                break;
            }
        if (! needed) {
            llvm::GlobalValue::LinkageTypes linkage = sym->getLinkage();
            // std::cout << "    unnecessary LLVM global " << sym->getName().str()
            //           << " linkage " << int(linkage) << "\n";
            if (linkage == llvm::GlobalValue::ExternalLinkage)
                f->setLinkage (llvm::GlobalValue::LinkOnceODRLinkage);
        }
    }
#endif
}



llvm::Function *
LLVM_Util::make_function (string_view name, bool fastcall,
                          llvm::Type *rettype,
                          llvm::Type *arg1,
                          llvm::Type *arg2,
                          llvm::Type *arg3,
                          llvm::Type *arg4)
{
    std::vector<llvm::Type*> args;
    if (arg1) args.push_back (arg1);
    if (arg2) args.push_back (arg2);
    if (arg3) args.push_back (arg3);
    if (arg4) args.push_back (arg4);
    return make_function (name, fastcall, rettype, args);
}



llvm::Function *
LLVM_Util::make_function (string_view name, bool fastcall,
                          llvm::Type *rettype,
                          array_view<llvm::Type*> params,
                          HasVarArgs varargs)
{
    if (! module())
        new_module ();
    llvm::FunctionType *functype = type_function (rettype, params, varargs);
    llvm::Function *func = llvm::Function::Create (functype, llvm::Function::ExternalLinkage,
                                                   SR(name), module());
    // llvm::Constant *c = module()->getOrInsertFunction (SR(name), functype);
    // ASSERT (c && "getOrInsertFunction returned NULL");
    // ASSERT_MSG (llvm::isa<llvm::Function>(c),
                // "Declaration for %s is wrong, LLVM had to make a cast", name.c_str());
    // llvm::Function *func = llvm::cast<llvm::Function>(c);
    if (fastcall)
        func->setCallingConv(llvm::CallingConv::Fast);
    current_function (func);
    set_insert_point ();
    return func;
}



llvm::Function *
LLVM_Util::declare_extern_function (string_view name, llvm::Type *rettype,
                                    array_view<llvm::Type*> params,
                                    HasVarArgs varargs)
{
    if (! module())
        new_module ();
    llvm::FunctionType *functype = type_function (rettype, params, varargs);
    llvm::Function *func = llvm::Function::Create (functype, llvm::Function::ExternalLinkage,
                                                   SR(name), module());
    return func;
}



void
LLVM_Util::current_function (llvm::Function *func)
{
    m_impl->current_function (func);
}


llvm::Function *
LLVM_Util::current_function () const
{
    return m_impl->current_function();
}


llvm::Value *
LLVM_Util::current_function_arg (int a)
{
    return m_impl->current_function_arg (a);
}



llvm::BasicBlock *
LLVM_Util::new_basic_block (string_view name, bool insert)
{
    return m_impl->new_basic_block (name, insert);
}



llvm::BasicBlock *
LLVM_Util::push_function (llvm::BasicBlock *after)
{
    return m_impl->push_function (after);
}



void
LLVM_Util::pop_function ()
{
    m_impl->pop_function ();
}



llvm::BasicBlock *
LLVM_Util::return_block () const
{
    return m_impl->return_block ();
}



void
LLVM_Util::push_loop (llvm::BasicBlock *step, llvm::BasicBlock *after)
{
    m_impl->push_loop (step, after);
}



void
LLVM_Util::pop_loop ()
{
    m_impl->pop_loop ();
}



llvm::BasicBlock *
LLVM_Util::loop_step_block () const
{
    return m_impl->loop_step_block ();
}



llvm::BasicBlock *
LLVM_Util::loop_after_block () const
{
    return m_impl->loop_after_block ();
}




llvm::Type *
LLVM_Util::type_union(const std::vector<llvm::Type *> &types)
{
    llvm::DataLayout target(module());
    size_t max_size = 0;
    size_t max_align = 1;
    for (size_t i = 0; i < types.size(); ++i) {
        size_t size = target.getTypeStoreSize(types[i]);
        size_t align = target.getABITypeAlignment(types[i]);
        max_size  = size  > max_size  ? size  : max_size;
        max_align = align > max_align ? align : max_align;
    }
    size_t padding = (max_size % max_align) ? max_align - (max_size % max_align) : 0;
    size_t union_size = max_size + padding;

    llvm::Type * base_type = NULL;
    // to ensure the alignment when included in a struct use
    // an appropiate type for the array
    if (max_align == sizeof(void*))
        base_type = type_void_ptr();
    else if (max_align == 4)
        base_type = (llvm::Type *) llvm::Type::getInt32Ty (context());
    else if (max_align == 2)
        base_type = (llvm::Type *) llvm::Type::getInt16Ty (context());
    else
        base_type = (llvm::Type *) llvm::Type::getInt8Ty (context());

    size_t array_len = union_size / target.getTypeStoreSize(base_type);
    return (llvm::Type *) llvm::ArrayType::get (base_type, array_len);
}



llvm::Type *LLVM_Util::type_float() const {return m_impl->type_float(); }
llvm::Type *LLVM_Util::type_int() const {return m_impl->type_int(); }
llvm::Type *LLVM_Util::type_addrint() const {return m_impl->type_addrint(); }
llvm::Type *LLVM_Util::type_bool() const {return m_impl->type_bool(); }
llvm::Type *LLVM_Util::type_char() const {return m_impl->type_char(); }
llvm::Type *LLVM_Util::type_longlong() const {return m_impl->type_longlong(); }
llvm::Type *LLVM_Util::type_void() const {return m_impl->type_void(); }
llvm::Type *LLVM_Util::type_triple() const {return m_impl->type_triple(); }
llvm::Type *LLVM_Util::type_matrix() const {return m_impl->type_matrix(); }
llvm::Type *LLVM_Util::type_typedesc() const {return m_impl->type_longlong(); }
llvm::PointerType *LLVM_Util::type_void_ptr() const {return m_impl->type_void_ptr(); }
llvm::PointerType *LLVM_Util::type_string() const { return m_impl->type_char_ptr(); }
llvm::PointerType *LLVM_Util::type_ustring_ptr() const {return m_impl->type_ustring_ptr(); }
llvm::PointerType *LLVM_Util::type_char_ptr() const {return m_impl->type_char_ptr(); }
llvm::PointerType *LLVM_Util::type_int_ptr() const {return m_impl->type_int_ptr(); }
llvm::PointerType *LLVM_Util::type_float_ptr() const {return m_impl->type_float_ptr(); }
llvm::PointerType *LLVM_Util::type_triple_ptr() const {return m_impl->type_triple_ptr(); }
llvm::PointerType *LLVM_Util::type_matrix_ptr() const {return m_impl->type_matrix_ptr(); }



llvm::Type *
LLVM_Util::type_struct (const std::vector<llvm::Type *> &types,
                        string_view name)
{
    return llvm::StructType::create(context(), types, SR(name));
}



llvm::Type *
LLVM_Util::type_ptr (llvm::Type *type)
{
    return llvm::PointerType::get (type, 0);
}



llvm::Type *
LLVM_Util::type_array (llvm::Type *type, int n)
{
    return llvm::ArrayType::get (type, n);
}



llvm::FunctionType *
LLVM_Util::type_function (llvm::Type *rettype,
                          array_view<llvm::Type*> params,
                          HasVarArgs varargs)
{
    return llvm::FunctionType::get (rettype, AR(params), varargs==VarArgs);
}



llvm::PointerType *
LLVM_Util::type_function_ptr (llvm::Type *rettype,
                              array_view<llvm::Type*> params,
                              HasVarArgs varargs)
{
    llvm::FunctionType *functype = type_function (rettype, params, varargs);
    return llvm::PointerType::getUnqual (functype);
}



std::string
LLVM_Util::llvm_typename (llvm::Type *type) const
{
    std::string s;
    llvm::raw_string_ostream stream (s);
    stream << (*type);
    return stream.str();
}



llvm::Type *
LLVM_Util::llvm_typeof (llvm::Value *val) const
{
    return val->getType();
}



std::string
LLVM_Util::llvm_typenameof (llvm::Value *val) const
{
    return llvm_typename (llvm_typeof (val));
}



llvm::Value *
LLVM_Util::constant (float f)
{
    return llvm::ConstantFP::get (context(), llvm::APFloat(f));
}



llvm::Value *
LLVM_Util::constant (int i)
{
    return llvm::ConstantInt::get (context(), llvm::APInt(32,i));
}



llvm::Value *
LLVM_Util::constant (size_t i)
{
    int bits = sizeof(size_t)*8;
    return llvm::ConstantInt::get (context(), llvm::APInt(bits,i));
}



llvm::Value *
LLVM_Util::constant_bool (bool i)
{
    return llvm::ConstantInt::get (context(), llvm::APInt(1,i));
}



llvm::Value *
LLVM_Util::constant_ptr (void *p, llvm::PointerType *type)
{
    if (! type)
        type = type_void_ptr();
    return builder().CreateIntToPtr (constant (size_t (p)), type, "const pointer");
}



llvm::Value *
LLVM_Util::constant (ustring s)
{
    // Create a const size_t with the ustring contents
    size_t bits = sizeof(size_t)*8;
    llvm::Value *str = llvm::ConstantInt::get (context(),
                               llvm::APInt(bits,size_t(s.c_str()), true));
    // Then cast the int to a char*.
    return builder().CreateIntToPtr (str, type_string(), "ustring constant");
}



llvm::Value *
LLVM_Util::constant (const TypeDesc &type)
{
    long long *i = (long long *)&type;
    return llvm::ConstantInt::get (context(), llvm::APInt(64,*i));
}



llvm::Value *
LLVM_Util::void_ptr_null ()
{
    return llvm::ConstantPointerNull::get (type_void_ptr());
}



llvm::Value *
LLVM_Util::ptr_to_cast (llvm::Value* val, llvm::Type *type)
{
    return builder().CreatePointerCast(val,llvm::PointerType::get(type, 0));
}



llvm::Value *
LLVM_Util::ptr_cast (llvm::Value* val, llvm::Type *type)
{
    return builder().CreatePointerCast(val,type);
}



llvm::Value *
LLVM_Util::ptr_cast (llvm::Value* val, const TypeDesc &type)
{
    return ptr_cast (val, llvm::PointerType::get (llvm_type(type), 0));
}



llvm::Value *
LLVM_Util::void_ptr (llvm::Value* val)
{
    return builder().CreatePointerCast(val,type_void_ptr());
}




llvm::Type *
LLVM_Util::llvm_type (const TypeDesc &typedesc)
{
    TypeDesc t = typedesc.elementtype();
    llvm::Type *lt = NULL;
    if (t == TypeDesc::FLOAT)
        lt = type_float();
    else if (t == TypeDesc::INT)
        lt = type_int();
    else if (t == TypeDesc::STRING)
        lt = type_string();
    else if (t.aggregate == TypeDesc::VEC3)
        lt = type_triple();
    else if (t.aggregate == TypeDesc::MATRIX44)
        lt = type_matrix();
    else if (t == TypeDesc::NONE)
        lt = type_void();
    else if (t == TypeDesc::UINT8)
        lt = type_char();
    else if (t == TypeDesc::PTR)
        lt = type_void_ptr();
    else {
        std::cerr << "Bad llvm_type(" << typedesc << ")\n";
        ASSERT (0 && "not handling this type yet");
    }
    if (typedesc.arraylen)
        lt = llvm::ArrayType::get (lt, typedesc.arraylen);
    DASSERT (lt);
    return lt;
}



llvm::Value *
LLVM_Util::offset_ptr (llvm::Value *ptr, int offset, llvm::Type *ptrtype)
{
    llvm::Value *i = builder().CreatePtrToInt (ptr, type_addrint());
    i = builder().CreateAdd (i, constant ((size_t)offset));
    ptr = builder().CreateIntToPtr (i, type_void_ptr());
    if (ptrtype)
        ptr = ptr_cast (ptr, ptrtype);
    return ptr;
}



llvm::Value *
LLVM_Util::op_alloca (llvm::Type *llvmtype, int n, string_view name)
{
    llvm::ConstantInt* numalloc = (llvm::ConstantInt*)constant(n);
    return builder().CreateAlloca (llvmtype, numalloc, SR(name));
}



llvm::Value *
LLVM_Util::op_alloca (const TypeDesc &type, int n, string_view name)
{
    return op_alloca (llvm_type(type.elementtype()), n*type.numelements(), name);
}



llvm::Value *
LLVM_Util::call_function (llvm::Value *func, OIIO::array_view<llvm::Value *> args)
{
    ASSERT (func);
#if 0
    llvm::outs() << "llvm_call_function " << *func << "\n";
    llvm::outs() << nargs << " args:\n";
    for (int i = 0;  i < nargs;  ++i)
        llvm::outs() << "\t" << *(args[i]) << "\n";
#endif
    //llvm_gen_debug_printf (std::string("start ") + std::string(name));
    llvm::Value *r = builder().CreateCall (func, AR(args));
    //llvm_gen_debug_printf (std::string(" end  ") + std::string(name));
    return r;
}



llvm::Value *
LLVM_Util::call_function (string_view name, OIIO::array_view<llvm::Value *> args)
{
    llvm::Function *func = module()->getFunction (SR(name));
    if (! func)
        std::cerr << "Couldn't find function " << name << "\n";
    return call_function (func, args);
}



void
LLVM_Util::mark_fast_func_call (llvm::Value *funccall)
{
    llvm::CallInst* call_inst = llvm::cast<llvm::CallInst>(funccall);
    call_inst->setCallingConv (llvm::CallingConv::Fast);
}



void
LLVM_Util::op_branch (llvm::BasicBlock *block)
{
    builder().CreateBr (block);
    set_insert_point (block);
}



void
LLVM_Util::op_branch (llvm::Value *cond, llvm::BasicBlock *trueblock,
                      llvm::BasicBlock *falseblock)
{
    builder().CreateCondBr (cond, trueblock, falseblock);
    set_insert_point (trueblock);
}



void
LLVM_Util::set_insert_point (llvm::BasicBlock *block)
{
    if (! block)
        block = new_basic_block ();
    builder().SetInsertPoint (block);
}



void
LLVM_Util::op_return (llvm::Value *retval)
{
    if (retval)
        builder().CreateRet (retval);
    else
        builder().CreateRetVoid ();
}



void
LLVM_Util::op_memset (llvm::Value *ptr, int val, int len, int align)
{
    op_memset(ptr, val, constant(len), align);
}



void
LLVM_Util::op_memset (llvm::Value *ptr, int val, llvm::Value *len, int align)
{
    // memset with i32 len
    // and with an i8 pointer (dst) for LLVM-2.8
    llvm::Type* types[] = {
        (llvm::Type *) llvm::PointerType::get(llvm::Type::getInt8Ty(context()), 0),
        (llvm::Type *) llvm::Type::getInt32Ty(context())
    };

    llvm::Function* func = llvm::Intrinsic::getDeclaration (module(),
        llvm::Intrinsic::memset, types);

    // NOTE(boulos): constant(0) would return an i32
    // version of 0, but we need the i8 version. If we make an
    // ::constant(char val) though then we'll get ambiguity
    // everywhere.
    llvm::Value* fill_val = llvm::ConstantInt::get (context(),
                                                    llvm::APInt(8, val));
    // Non-volatile (allow optimizer to move it around as it wishes
    // and even remove it if it can prove it's useless)
    llvm::Value *args[] = { ptr, fill_val, len, constant(align),
                            constant_bool(false) };
    builder().CreateCall (func, args);
}



void
LLVM_Util::op_memcpy (llvm::Value *dst, llvm::Value *src, int len, int align)
{
    // i32 len
    // and with i8 pointers (dst and src) for LLVM-2.8
    llvm::Type* types[] = {
        (llvm::Type *) llvm::PointerType::get(llvm::Type::getInt8Ty(context()), 0),
        (llvm::Type *) llvm::PointerType::get(llvm::Type::getInt8Ty(context()), 0),
        (llvm::Type *) llvm::Type::getInt32Ty(context())
    };

    llvm::Function* func = llvm::Intrinsic::getDeclaration (module(),
        llvm::Intrinsic::memcpy, types);

    // Non-volatile (allow optimizer to move it around as it wishes
    // and even remove it if it can prove it's useless)
    llvm::Value *args[] = { dst, src, constant(len), constant(align),
                            constant_bool(false) };
    builder().CreateCall (func, args);
}



llvm::Value *
LLVM_Util::op_load (llvm::Value *ptr)
{
    return builder().CreateLoad (ptr);
}



void
LLVM_Util::op_store (llvm::Value *val, llvm::Value *ptr)
{
    builder().CreateStore (val, ptr);
}



llvm::Value *
LLVM_Util::GEP (llvm::Value *ptr, llvm::Value *elem)
{
    return builder().CreateGEP (ptr, elem);
}



llvm::Value *
LLVM_Util::GEP (llvm::Value *ptr, int elem)
{
    return builder().CreateConstGEP1_32 (ptr, elem);
}



llvm::Value *
LLVM_Util::GEP (llvm::Value *ptr, int elem1, int elem2)
{
    return builder().CreateConstGEP2_32 (nullptr, ptr, elem1, elem2);
}



llvm::Value *
LLVM_Util::op_add (llvm::Value *a, llvm::Value *b)
{
    if (a->getType() == type_float() && b->getType() == type_float())
        return builder().CreateFAdd (a, b);
    if (a->getType() == type_int() && b->getType() == type_int())
        return builder().CreateAdd (a, b);
    ASSERT (0 && "Op has bad value type combination");
}



llvm::Value *
LLVM_Util::op_sub (llvm::Value *a, llvm::Value *b)
{
    if (a->getType() == type_float() && b->getType() == type_float())
        return builder().CreateFSub (a, b);
    if (a->getType() == type_int() && b->getType() == type_int())
        return builder().CreateSub (a, b);
    ASSERT (0 && "Op has bad value type combination");
}



llvm::Value *
LLVM_Util::op_neg (llvm::Value *a)
{
    if (a->getType() == type_float())
        return builder().CreateFNeg (a);
    if (a->getType() == type_int())
        return builder().CreateNeg (a);
    ASSERT (0 && "Op has bad value type combination");
}



llvm::Value *
LLVM_Util::op_mul (llvm::Value *a, llvm::Value *b)
{
    if (a->getType() == type_float() && b->getType() == type_float())
        return builder().CreateFMul (a, b);
    if (a->getType() == type_int() && b->getType() == type_int())
        return builder().CreateMul (a, b);
    ASSERT (0 && "Op has bad value type combination");
}



llvm::Value *
LLVM_Util::op_div (llvm::Value *a, llvm::Value *b)
{
    if (a->getType() == type_float() && b->getType() == type_float())
        return builder().CreateFDiv (a, b);
    if (a->getType() == type_int() && b->getType() == type_int())
        return builder().CreateSDiv (a, b);
    ASSERT (0 && "Op has bad value type combination");
}



llvm::Value *
LLVM_Util::op_mod (llvm::Value *a, llvm::Value *b)
{
    if (a->getType() == type_float() && b->getType() == type_float())
        return builder().CreateFRem (a, b);
    if (a->getType() == type_int() && b->getType() == type_int())
        return builder().CreateSRem (a, b);
    ASSERT (0 && "Op has bad value type combination");
}



llvm::Value *
LLVM_Util::op_float_to_int (llvm::Value* a)
{
    if (a->getType() == type_float())
        return builder().CreateFPToSI(a, type_int());
    if (a->getType() == type_int())
        return a;
    ASSERT (0 && "Op has bad value type combination");
}



llvm::Value *
LLVM_Util::op_float_to_double (llvm::Value* a)
{
    ASSERT (a->getType() == type_float());
    return builder().CreateFPExt(a, llvm::Type::getDoubleTy(context()));
}



llvm::Value *
LLVM_Util::op_int_to_float (llvm::Value* a)
{
    if (a->getType() == type_int())
        return builder().CreateSIToFP(a, type_float());
    if (a->getType() == type_float())
        return a;
    ASSERT (0 && "Op has bad value type combination");
}



llvm::Value *
LLVM_Util::op_bool_to_int (llvm::Value* a)
{
    if (a->getType() == type_bool())
        return builder().CreateZExt (a, type_int());
    if (a->getType() == type_int())
        return a;
    ASSERT (0 && "Op has bad value type combination");
}



llvm::Value *
LLVM_Util::op_and (llvm::Value *a, llvm::Value *b)
{
    return builder().CreateAnd (a, b);
}


llvm::Value *
LLVM_Util::op_or (llvm::Value *a, llvm::Value *b)
{
    return builder().CreateOr (a, b);
}


llvm::Value *
LLVM_Util::op_xor (llvm::Value *a, llvm::Value *b)
{
    return builder().CreateXor (a, b);
}


llvm::Value *
LLVM_Util::op_shl (llvm::Value *a, llvm::Value *b)
{
    return builder().CreateShl (a, b);
}


llvm::Value *
LLVM_Util::op_shr (llvm::Value *a, llvm::Value *b)
{
    if (a->getType() == type_int() && b->getType() == type_int())
        return builder().CreateAShr (a, b);  // signed int -> arithmetic shift
    ASSERT (0 && "Op has bad value type combination");
}



llvm::Value *
LLVM_Util::op_not (llvm::Value *a)
{
    return builder().CreateNot (a);
}



llvm::Value *
LLVM_Util::op_select (llvm::Value *cond, llvm::Value *a, llvm::Value *b)
{
    return builder().CreateSelect (cond, a, b);
}



llvm::Value *
LLVM_Util::op_eq (llvm::Value *a, llvm::Value *b, bool ordered)
{
    ASSERT (a->getType() == b->getType());
    if (a->getType() == type_float())
        return ordered ? builder().CreateFCmpOEQ (a, b) : builder().CreateFCmpUEQ (a, b);
    else
        return builder().CreateICmpEQ (a, b);
}



llvm::Value *
LLVM_Util::op_ne (llvm::Value *a, llvm::Value *b, bool ordered)
{
    ASSERT (a->getType() == b->getType());
    if (a->getType() == type_float())
        return ordered ? builder().CreateFCmpONE (a, b) : builder().CreateFCmpUNE (a, b);
    else
        return builder().CreateICmpNE (a, b);
}



llvm::Value *
LLVM_Util::op_gt (llvm::Value *a, llvm::Value *b, bool ordered)
{
    ASSERT (a->getType() == b->getType());
    if (a->getType() == type_float())
        return ordered ? builder().CreateFCmpOGT (a, b) : builder().CreateFCmpUGT (a, b);
    else
        return builder().CreateICmpSGT (a, b);
}



llvm::Value *
LLVM_Util::op_lt (llvm::Value *a, llvm::Value *b, bool ordered)
{
    ASSERT (a->getType() == b->getType());
    if (a->getType() == type_float())
        return ordered ? builder().CreateFCmpOLT (a, b) : builder().CreateFCmpULT (a, b);
    else
        return builder().CreateICmpSLT (a, b);
}



llvm::Value *
LLVM_Util::op_ge (llvm::Value *a, llvm::Value *b, bool ordered)
{
    ASSERT (a->getType() == b->getType());
    if (a->getType() == type_float())
        return ordered ? builder().CreateFCmpOGE (a, b) : builder().CreateFCmpUGE (a, b);
    else
        return builder().CreateICmpSGE (a, b);
}



llvm::Value *
LLVM_Util::op_le (llvm::Value *a, llvm::Value *b, bool ordered)
{
    ASSERT (a->getType() == b->getType());
    if (a->getType() == type_float())
        return ordered ? builder().CreateFCmpOLE (a, b) : builder().CreateFCmpULE (a, b);
    else
        return builder().CreateICmpSLE (a, b);
}



void
LLVM_Util::write_bitcode_file (const char *filename, std::string *err)
{
#if OSL_LLVM_VERSION >= 36
    std::error_code local_error;
    llvm::raw_fd_ostream out (filename, local_error, llvm::sys::fs::F_None);
#elif OSL_LLVM_VERSION >= 35
    std::string local_error;
    llvm::raw_fd_ostream out (filename, err ? *err : local_error, llvm::sys::fs::F_None);
#else
    std::string local_error;
    llvm::raw_fd_ostream out (filename, err ? *err : local_error);
#endif
    llvm::WriteBitcodeToFile (module(), out);

#if OSL_LLVM_VERSION >= 36
    if (err && local_error)
        *err = local_error.message ();
#endif
}



std::string
LLVM_Util::bitcode_string (llvm::Function *func)
{
    std::string s;
    llvm::raw_string_ostream stream (s);
    stream << (*func);
    return stream.str();
}



void
LLVM_Util::delete_func_body (llvm::Function *func)
{
    func->deleteBody ();
}



bool
LLVM_Util::func_is_empty (llvm::Function *func)
{
    return func->size() == 1 // func has just one basic block
        && func->front().size() == 1;  // the block has one instruction,
                                       ///   presumably the ret
}


std::string
LLVM_Util::func_name (llvm::Function *func)
{
    return func->getName().str();
}


OSL_NAMESPACE_EXIT
