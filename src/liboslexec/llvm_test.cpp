/*
Copyright (c) 2009-2016 Sony Pictures Imageworks Inc., et al.
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

#include <OpenImageIO/dassert.h>
#include <OpenImageIO/argparse.h>

#include <llvm/ADT/STLExtras.h>
#include <llvm/Bitcode/ReaderWriter.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/Orc/CompileUtils.h>
#include <llvm/ExecutionEngine/RuntimeDyld.h>
#include <llvm/ExecutionEngine/Orc/IRCompileLayer.h>
#include <llvm/ExecutionEngine/Orc/LambdaResolver.h>
#include <llvm/ExecutionEngine/Orc/ObjectLinkingLayer.h>
#include <llvm/IR/DataLayout.h>
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
#include <llvm/Transforms/Utils/UnifyFunctionExitNodes.h>



static bool verbose = false;
static bool debug = false;
static int opt = -1;
static bool orc = false;


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
                "--opt %d", &opt, "Opt level (0=basic, 1-3=like clang)",
                "--orc", &orc, "Use ORC JIT",
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



template <typename T>
inline std::vector<T> singletonSet (T t)
{
    std::vector<T> Vec;
    Vec.push_back(std::move(t));
    return Vec;
}


std::string
mangle (const std::string &Name, llvm::DataLayout &DL)
{
    std::string MangledName;
    llvm::raw_string_ostream MangledNameStream (MangledName);
    llvm::Mangler::getNameWithPrefix (MangledNameStream, Name, DL);
    return MangledName;
}



extern "C" {
// __attribute__ ((visibility ("default")))
float sqr (float x) { return x*x; }
}



int
main (int argc, char *argv[])
{
    getargs (argc, argv);

    llvm::InitializeAllTargets();
    llvm::InitializeAllTargetMCs();
    llvm::InitializeAllAsmPrinters();
    llvm::InitializeAllAsmParsers();
    llvm::sys::DynamicLibrary::LoadLibraryPermanently (nullptr);
    llvm::LLVMContext Context;
    std::unique_ptr<llvm::TargetMachine> TM (llvm::EngineBuilder().selectTarget());
    std::unique_ptr<llvm::DataLayout> DL;
    DL.reset (new llvm::DataLayout (TM->createDataLayout()));
    std::unique_ptr<llvm::ExecutionEngine> EE;
    typedef llvm::orc::ObjectLinkingLayer<> ObjLayerT;
    typedef llvm::orc::IRCompileLayer<ObjLayerT> CompileLayerT;
    typedef CompileLayerT::ModuleSetHandleT ModuleHandleT;
    ObjLayerT Objlayer;
    CompileLayerT Compilelayer (Objlayer, llvm::orc::SimpleCompiler(*TM));
    std::unique_ptr<llvm::Module> M (new llvm::Module("module", Context));
    M->setDataLayout (*DL);

    // Declare stub for external function sqr
    auto type_float = llvm::Type::getFloatTy (Context);
    llvm::Type* one_float[] = { type_float };
    llvm::FunctionType *functype_ff = llvm::FunctionType::get (type_float, one_float, false);
    llvm::Function::Create (functype_ff, llvm::Function::ExternalLinkage,
                            "sqr", M.get());


    // Create myfunc and generate its IR, which just calls sqr on its argument
    llvm::Function *myfunc = llvm::Function::Create (functype_ff,
                                                     llvm::Function::ExternalLinkage,
                                                     "myfunc", M.get());
    llvm::IRBuilder<> builder (Context);
    auto block = llvm::BasicBlock::Create (Context, "", myfunc);
    builder.SetInsertPoint (block);
    llvm::Value *a = llvm::cast<llvm::Value>(myfunc->arg_begin());
    llvm::Value *asq = builder.CreateCall (M->getFunction ("sqr"), a);
    builder.CreateRet (asq);

    // Set up compilation
    if (orc) {
        auto Resolver = llvm::orc::createLambdaResolver(
            // External lookup functor
            [&](const std::string &name) {
                if (auto Sym = Compilelayer.findSymbol(name, true))
                    return llvm::RuntimeDyld::SymbolInfo(Sym.getAddress(), Sym.getFlags());
                // If not found as a symbol, look up in current process
                if (auto Addr = llvm::RTDyldMemoryManager::getSymbolAddressInProcess(name))
                    return llvm::RuntimeDyld::SymbolInfo(Addr, llvm::JITSymbolFlags::Exported);
                return llvm::RuntimeDyld::SymbolInfo(nullptr);
            },
            // Dylib lookup functor
            [&](const std::string &name) { return nullptr; }
        );
        Compilelayer.addModuleSet (singletonSet(std::move(M)),
                                          llvm::make_unique<llvm::SectionMemoryManager>(),
                                          std::move(Resolver));
    } else {
        // MCJIT
        std::string engine_errors;
        llvm::EngineBuilder engine_builder (std::move(M));
        engine_builder.setEngineKind (llvm::EngineKind::JIT)
                      .setOptLevel (llvm::CodeGenOpt::Default) // Aggressive?
                      .setErrorStr (&engine_errors);
        EE.reset (engine_builder.create());
        if (! EE)
            std::cout << engine_errors << "\n";
        ASSERT (EE);
        EE->finalizeObject ();
    }

    // Ask for a callable function (will JIT on demand)
    typedef float (*FuncFloatFloat)(float);
    FuncFloatFloat my_executable_function = NULL;
    if (orc) {
        auto ExprSymbol = Compilelayer.findSymbol (mangle("myfunc", *DL), true);
        my_executable_function = (FuncFloatFloat) ExprSymbol.getAddress ();
    } else {
        my_executable_function = (FuncFloatFloat) EE->getFunctionAddress ("myfunc");
    }

    ASSERT (my_executable_function);
    printf ("myfunc(42.0f) = %g\n",
            (*my_executable_function)(42.0f));
    ASSERT ((*my_executable_function)(42.0f) == sqr(42.0));
}
