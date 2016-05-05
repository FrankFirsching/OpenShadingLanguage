/*
Copyright (c) 2009-2013 Sony Pictures Imageworks Inc., et al.
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

#include <OpenImageIO/typedesc.h>
#include <OpenImageIO/ustring.h>
#include <OpenImageIO/dassert.h>

#include <OpenImageIO/argparse.h>
#include <OpenImageIO/strutil.h>
#include <OpenImageIO/sysutil.h>


#if 1

#include <llvm/ADT/STLExtras.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/Orc/CompileUtils.h>
#include <llvm/ExecutionEngine/Orc/IRCompileLayer.h>
#include <llvm/ExecutionEngine/Orc/LambdaResolver.h>
#include <llvm/ExecutionEngine/Orc/ObjectLinkingLayer.h>
#include <llvm/IR/DataLayout.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Mangler.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Target/TargetMachine.h>

#include <OpenImageIO/thread.h>

std::string
make_unique_name (OIIO::string_view prefix = "___")
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

#define USE_ORC 1


class SimpleJIT {
public:
    typedef llvm::orc::ObjectLinkingLayer<> ObjLayerT;
    typedef llvm::orc::IRCompileLayer<ObjLayerT> CompileLayerT;
    typedef CompileLayerT::ModuleSetHandleT ModuleHandleT;


    SimpleJIT () {
        initialize_llvm ();

        // Grab an LLVM context, target machine, and data layout. These can be
        // reused for many modules.
        llvm_context.reset (new llvm::LLVMContext()); /* S */
        llvm_target_machine.reset (llvm::EngineBuilder().selectTarget()); /* TM */
        llvm_data_layout.reset (new llvm::DataLayout (llvm_target_machine->createDataLayout()));
        inttype = llvm::Type::getInt32Ty (*llvm_context);

        // Set up ORC JIT. Can be used for many modules.
        orc_compilelayer.reset (new CompileLayerT(orc_objlayer,
                                                  llvm::orc::SimpleCompiler(*llvm_target_machine)));
    }

    // Initialize LLVM generally
    void initialize_llvm () {
        llvm::InitializeAllTargets();
        // llvm::InitializeAllTargetInfos();
        llvm::InitializeAllTargetMCs();
        llvm::InitializeAllAsmPrinters();
        llvm::InitializeAllAsmParsers();
        // llvm::InitializeAllDisassemblers();
    }

    // Create a module. We can put as many functions in here as we want,
    // but we JIT it all at once and then we don't add to it again.
    llvm::Module *new_module () {
        llvm_module.reset (new llvm::Module (make_unique_name("jit_module_"),
                                             *llvm_context));
        llvm_module->setDataLayout (*llvm_data_layout);
        return llvm_module.get ();
    }

    // Create a function. (Empty for now)
    llvm::Function *make_function (const std::string &name) {
        llvm::Type *params[] = { inttype, inttype };
        llvm::FunctionType *functype = llvm::FunctionType::get (inttype, params, false/*varargs*/);
        llvm::Constant *func_const = llvm_module->getOrInsertFunction (name, functype);
        return current_func = llvm::cast<llvm::Function>(func_const);
    }

    llvm::BasicBlock *new_basic_block (OIIO::string_view basename="") {
        if (! builder)
            builder.reset (new llvm::IRBuilder<>(*llvm_context));
        if (basename.empty())
            basename = OIIO::string_view("block_");
        llvm::BasicBlock *block = llvm::BasicBlock::Create (*llvm_context,
                                                            std::string(basename),
                                                            current_func);
        builder->SetInsertPoint (block);
        return current_block = block;
    }

    llvm::Value * current_function_arg (int a) {
        llvm::Function::arg_iterator arg_it = current_func->arg_begin();
        while (a-- > 0)
            ++arg_it;
        return llvm::cast<llvm::Value>(arg_it);
    }

    std::string mangle (const std::string &name) {
        std::string MangledName;
        llvm::raw_string_ostream MangledNameStream (MangledName);
        llvm::Mangler::getNameWithPrefix (MangledNameStream, name, *llvm_data_layout);
        return MangledName;
    }

    ModuleHandleT addModule (std::unique_ptr<llvm::Module> M) {
        // We need a memory manager to allocate memory and resolve symbols
        // for this new module. Create one that resolves symbols by looking
        // back into the JIT. (Stolen from LLVM Kaleidescope example.)
        auto Resolver = llvm::orc::createLambdaResolver(
                          [&](const std::string &Name) {
                            if (auto Sym = findSymbol(Name))
                              return llvm::RuntimeDyld::SymbolInfo(Sym.getAddress(),
                                                             Sym.getFlags());
                            return llvm::RuntimeDyld::SymbolInfo(nullptr);
                          },
                          [](const std::string &S) { return nullptr; }
                        );
        // std::vector<std::unique_ptr<llvm::Module>> moduleset;// { std::move(M) };
        // moduleset.push_back (std::move(M));
        return orc_compilelayer->addModuleSet (singletonSet(std::move(M)),
                                               llvm::make_unique<llvm::SectionMemoryManager>(),
                                               std::move(Resolver));
    }

    void removeModule (ModuleHandleT H) {
        orc_compilelayer->removeModuleSet(H);
    }

    llvm::orc::JITSymbol findSymbol (const std::string &name) {
        return orc_compilelayer->findSymbol (name, true);
    }

    llvm::orc::JITSymbol findUnmangledSymbol (const std::string &name) {
        return findSymbol (mangle (name));
    }

    void module_done () {
        if (use_orc_jit) {
            /*auto modulehandle =*/ addModule (std::move(llvm_module));
        } else {
            std::string engine_errors;
            llvm::EngineBuilder engine_builder (std::move(llvm_module));
            engine_builder.setEngineKind (llvm::EngineKind::JIT)
                          .setOptLevel (llvm::CodeGenOpt::Default) // Aggressive?
                          .setErrorStr (&engine_errors);
            llvm_exec.reset (engine_builder.create());
            llvm_exec->finalizeObject ();   // Necessary?
        }
    }

    void * get_compiled_function (OIIO::string_view name) {
        if (use_orc_jit) {
            auto ExprSymbol = findUnmangledSymbol ("myadd");
            return (void *) ExprSymbol.getAddress ();
        } else {
            return (void *) llvm_exec->getFunctionAddress (name);
        }
    }

    void * get_compiled_function (llvm::Function *func) {
        return llvm_exec->getPointerToFunction (func);
    }

    void orc_jit (bool enable) { use_orc_jit = enable; }
    bool orc_jit () const { return use_orc_jit; }

    std::unique_ptr<llvm::LLVMContext> llvm_context;
    std::unique_ptr<llvm::TargetMachine> llvm_target_machine;
    std::unique_ptr<llvm::DataLayout> llvm_data_layout;
    ObjLayerT orc_objlayer;
    std::unique_ptr<CompileLayerT> orc_compilelayer;
    std::unique_ptr<llvm::Module> llvm_module;  // current module
    llvm::IntegerType *inttype = NULL;
    std::unique_ptr<llvm::IRBuilder<> > builder;
    llvm::Function *current_func = NULL;
    llvm::BasicBlock *current_block = NULL;
    std::unique_ptr<llvm::ExecutionEngine> llvm_exec;
    bool use_orc_jit = true;
};



int
main (int argc, char *argv[])
{
    {
    std::cout << "running llvmutil_test...\n";

    SimpleJIT jit;



    std::cout << "did setup, orc=" << jit.orc_jit() << "\n";

    jit.new_module ();
    /*llvm::Function *func =*/ jit.make_function ("myadd");

    jit.new_basic_block ();

    llvm::Value *p0 = jit.current_function_arg (0);
    llvm::Value *p1 = jit.current_function_arg (1);
    llvm::Value *r = jit.builder->CreateAdd (p0, p1);
    jit.builder->CreateRet (r);

    std::cout << "Created function\n";

    jit.module_done ();
    typedef int (*IntFuncOfTwoInts)(int,int);
    IntFuncOfTwoInts callable_func = (IntFuncOfTwoInts) jit.get_compiled_function ("myadd");

    std::cout << "Generated code?\n";

    std::cout << "Result 40+2 = " << callable_func(40, 2) << "\n";

#if 0
    getargs (argc, argv);

    // Test simple functions
    test_int_func();
    test_triple_func();

    if (memtest) {
        for (int i = 0; i < memtest; ++i) {
            IntFuncOfTwoInts f = test_big_func (i==0);
            int r = f (42, 42);
            ASSERT (r == 84);
        }
        std::cout << "After " << memtest << " stupid functions compiled:\n";
        std::cout << "   RSS memory = "
                  << OIIO::Strutil::memformat(OIIO::Sysutil::memory_used()) << "\n";
    }
#endif

    }
    std::cout << "did teardown\n";

    return 0;
}




#else


#include "OSL/llvm_util.h"


typedef int (*IntFuncOfTwoInts)(int,int);

static bool verbose = false;
static bool debug = false;
static int memtest = 0;



static void
getargs (int argc, char *argv[])
{
    bool help = false;
    OIIO::ArgParse ap;
    ap.options ("llvmutil_test\n"
                OIIO_INTRO_STRING "\n"
                "Usage:  llvmutil_test [options]",
                "--help", &help, "Print help message",
                "-v", &verbose, "Verbose mode",
                "--debug", &debug, "Debug mode",
                "--memtest %d", &memtest, "Memory test mode (arg: iterations)",
                // "--iters %d", &iterations,
                //     Strutil::format("Number of iterations (default: %d)", iterations).c_str(),
                // "--trials %d", &ntrials, "Number of trials",
                NULL);
    if (ap.parse (argc, (const char**)argv) < 0) {
        std::cerr << ap.geterror() << std::endl;
        ap.usage ();
        exit (EXIT_FAILURE);
    }
    if (help) {
        ap.usage ();
        exit (EXIT_FAILURE);
    }
}



// This demonstrates the use of LLVM_Util to generate the following
// function on the fly, JIT it, and call it:
//      int myadd (int arg1, int arg2)
//      {
//          return arg1 + arg2;    
//      }
//
void
test_int_func ()
{
    // Setup
    OSL::pvt::LLVM_Util ll;

    // Make a function with prototype:   int myadd (int arg1, int arg2)
    // and make it the current function.
    llvm::Function *func = ll.make_function ("myadd",         // name
                                             false,           // fastcall
                                             ll.type_int(),   // return
                                             ll.type_int(),   // arg1
                                             ll.type_int());  // arg2
    ll.current_function (func);

    // Generate the ops for this function:  return arg1 + arg2
    llvm::Value *arg1 = ll.current_function_arg (0);
    llvm::Value *arg2 = ll.current_function_arg (1);
    llvm::Value *sum = ll.op_add (arg1, arg2);
    ll.op_return (sum);

    // Optimize it
    ll.setup_optimization_passes (0);
    ll.do_optimize ();

    // Print the optimized bitcode
    std::cout << "Generated the following bitcode:\n"
              << ll.bitcode_string(func) << "\n";

    // Ask for a callable function (will JIT on demand)
    IntFuncOfTwoInts myadd = (IntFuncOfTwoInts) ll.getPointerToFunction (func);

    // Call it:
    int result = myadd (13, 29);
    std::cout << "The result is " << result << "\n";
    ASSERT (result == 42);
}



// This demonstrates the use of LLVM_Util to generate the following
// function on the fly, JIT it, and call it:
//      void myaddv (Vec3 *result, Vec3 *a, float b)
//      {
//          *result = (*a) * b;
//      }
//
void
test_triple_func ()
{
    // Setup
    OSL::pvt::LLVM_Util ll;

    // Make a function with prototype:   int myadd (int arg1, int arg2)
    // and make it the current function.
    llvm::Function *func = ll.make_function ("myaddv",        // name
                                             false,           // fastcall
                                             ll.type_void(),  // return
                                             (llvm::Type *)ll.type_triple_ptr(), // result
                                             (llvm::Type *)ll.type_triple_ptr(), // arg1
                                             ll.type_float());  // arg2
    ll.current_function (func);

    // Generate the ops for this function:  r = a*b
    llvm::Value *rptr = ll.current_function_arg (0);
    llvm::Value *aptr = ll.current_function_arg (1);
    llvm::Value *b = ll.current_function_arg (2);
    for (int i = 0; i < 3; ++i) {
        llvm::Value *r_elptr = ll.GEP (rptr, 0, i);
        llvm::Value *a_elptr = ll.GEP (aptr, 0, i);
        llvm::Value *product = ll.op_mul (ll.op_load(a_elptr), b);
        ll.op_store (product, r_elptr);
    }
    ll.op_return ();

    // Optimize it
    ll.setup_optimization_passes (0);
    ll.do_optimize ();

    // Print the optimized bitcode
    std::cout << "Generated the following bitcode:\n"
              << ll.bitcode_string(func) << "\n";

    // Ask for a callable function (will JIT on demand)
    typedef void (*FuncVecVecFloat)(void*, void*, float);
    FuncVecVecFloat f = (FuncVecVecFloat) ll.getPointerToFunction (func);

    // Call it:
    {
    float r[3], a[3] = { 1.0, 2.0, 3.0 }, b = 42.0;
    f (r, a, b);
    std::cout << "The result is " << r[0] << ' ' << r[1] << ' ' << r[2] << "\n";
    ASSERT (r[0] == 42.0 && r[1] == 84.0 && r[2] == 126.0);
    }
}




// Make a crazy big function with lots of IR, having prototype:
//      int mybig (int arg1, int arg2);
//
IntFuncOfTwoInts
test_big_func (bool do_print=false)
{
    // Setup
    OSL::pvt::LLVM_Util ll;

    // Make a function with prototype:  int myadd (int arg1, int arg2)
    // and make it the current function in the current module.
    llvm::Function *func = ll.make_function ("myadd",         // name
                                             false,           // fastcall
                                             ll.type_int(),   // return
                                             ll.type_int(),   // arg1
                                             ll.type_int());  // arg2
    // Make it the current function and get it ready to accept IR.
    ll.current_function (func);

    // Generate the ops for this function:  return arg1 + arg2
    llvm::Value *arg1 = ll.current_function_arg (0);
    llvm::Value *arg2 = ll.current_function_arg (1);
    llvm::Value *sum = ll.op_add (arg1, arg2);
    // Additional useless assignments, to bloat code
    for (int i = 0; i < 1000; ++i) {
        sum = ll.op_add (arg1, arg2);
    }
    ll.op_return (sum);

    // Print the optimized bitcode
    // if (do_print)
    //     std::cout << "Generated the following bitcode:\n"
    //               << ll.bitcode_string(func) << "\n";

    ll.setup_optimization_passes (0);
    ll.do_optimize ();

    if (do_print)
        std::cout << "After optimizing:\n"
                  << ll.bitcode_string(func) << "\n";

    // Ask for a callable function (will JIT on demand)
    IntFuncOfTwoInts myadd = (IntFuncOfTwoInts) ll.getPointerToFunction (func);

    // We're done with the module now
    // ll.remove_module (module);

    // Return the function. The callable code should survive the destruction
    // of the LLVM_Util and its resources!
    return myadd;
}




int
main (int argc, char *argv[])
{
    getargs (argc, argv);

    // Test simple functions
    test_int_func();
    test_triple_func();

    if (memtest) {
        for (int i = 0; i < memtest; ++i) {
            IntFuncOfTwoInts f = test_big_func (i==0);
            int r = f (42, 42);
            ASSERT (r == 84);
        }
        std::cout << "After " << memtest << " stupid functions compiled:\n";
        std::cout << "   RSS memory = "
                  << OIIO::Strutil::memformat(OIIO::Sysutil::memory_used()) << "\n";
    }

    return 0;
}

#endif
