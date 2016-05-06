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

#include "OSL/llvm_util.h"
#include <llvm/ExecutionEngine/RTDyldMemoryManager.h>


typedef int (*IntFuncOfTwoInts)(int,int);

static bool verbose = false;
static bool debug = false;
static int memtest = 0;
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
                "--memtest %d", &memtest, "Memory test mode (arg: iterations)",
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
    OSL::LLVM_Util ll;
    ll.orc_jit (orc);

    // Make a function with prototype:   int myadd (int arg1, int arg2)
    // and make it the current function. A module will automatically be
    // created to hold the function.
    const char funcname[] = "myadd";
    llvm::Function *func = ll.make_function (funcname,         // name
                                             false,           // fastcall
                                             ll.type_int(),   // return
                                             ll.type_int(),   // arg1
                                             ll.type_int());  // arg2

    // Generate the ops for this function:  return arg1 + arg2
    llvm::Value *arg1 = ll.current_function_arg (0);
    llvm::Value *arg2 = ll.current_function_arg (1);
    llvm::Value *sum = ll.op_add (arg1, arg2);
    ll.op_return (sum);

    // Optimize it and compile
    if (opt >= 0) {
        ll.setup_optimization_passes (opt);
        ll.do_optimize ();
    }
    // Print the optimized bitcode
    std::cout << "Generated the following bitcode:\n"
              << ll.bitcode_string(func) << "\n";

    ll.module_done ();


    // Ask for a callable function (will JIT on demand)
    IntFuncOfTwoInts myadd = (IntFuncOfTwoInts) ll.get_compiled_function (funcname);

    // Call it:
    int result = myadd (13, 29);
    std::cout << "The result is " << result << "\n";
    ASSERT (result == 42);
}




extern "C" {
__attribute__ ((visibility ("default"))) 
float sqr (float x) { return x*x; }
}


// This demonstrates the use of LLVM_Util to generate the following
// function on the fly, JIT it, and call it:
//      void myfunc (Vec3 *result, Vec3 *a, float b)
//      {
//          *result = (*a) * sqr(b);
//      }
//
void
test_triple_func ()
{
    // Setup
    OSL::LLVM_Util ll;
    ll.orc_jit (orc);

std::cout << "Trying to resolve sqr, result is "
<< llvm::RTDyldMemoryManager::getSymbolAddressInProcess("sqr") << "\n";
std::cout << "Trying to resolve _sqr, result is "
<< llvm::RTDyldMemoryManager::getSymbolAddressInProcess("_sqr") << "\n";

    // Declare stub for sqr
    llvm::Type* sqr_args[] = { ll.type_float() };
    ll.declare_extern_function ("sqr", ll.type_float(), sqr_args);

    // Make a function with prototype:   int myadd (int arg1, int arg2)
    // and make it the current function. A module will automatically be
    // created to hold the function.
    const char funcname[] = "myfunc";
    llvm::Function *func = ll.make_function (funcname,        // name
                                             false,           // fastcall
                                             ll.type_void(),  // return
                                             (llvm::Type *)ll.type_triple_ptr(), // result
                                             (llvm::Type *)ll.type_triple_ptr(), // arg1
                                             ll.type_float());  // arg2

    // Generate the ops for this function:  r = a*b
    llvm::Value *rptr = ll.current_function_arg (0);
    llvm::Value *aptr = ll.current_function_arg (1);
    llvm::Value *b = ll.current_function_arg (2);
    llvm::Value *bsq = ll.call_function ("sqr", b);
    for (int i = 0; i < 3; ++i) {
        llvm::Value *r_elptr = ll.GEP (rptr, 0, i);
        llvm::Value *a_elptr = ll.GEP (aptr, 0, i);
        llvm::Value *product = ll.op_mul (ll.op_load(a_elptr), bsq);
        ll.op_store (product, r_elptr);
    }
    ll.op_return ();

    // Optimize it and compile
    if (opt >= 0) {
        ll.setup_optimization_passes (opt);
        ll.do_optimize ();
    }

    // ll.write_bitcode_file ("out.bc");

    // Print the optimized bitcode
    std::cout << "Generated the following bitcode:\n"
              << ll.bitcode_string(func) << "\n";

    ll.module_done ();

    // Ask for a callable function (will JIT on demand)
    typedef void (*FuncVecVecFloat)(void*, void*, float);
    FuncVecVecFloat f = (FuncVecVecFloat) ll.get_compiled_function (funcname);

    // Call it:
    {
    float r[3], a[3] = { 1.0, 2.0, 3.0 }, b = 42.0;
    f (r, a, b);
    std::cout << "The result is " << r[0] << ' ' << r[1] << ' ' << r[2] << "\n";
    ASSERT (r[0] == 1764.0 && r[1] == 3528.0 && r[2] == 5292.0);
    }
}




// Make a crazy big function with lots of IR, having prototype:
//      int mybig (int arg1, int arg2);
//
IntFuncOfTwoInts
test_big_func (OSL::LLVM_Util &ll, bool do_print=false)
{
    // Start a new module
    ll.new_module ();

    // Make a function with prototype:  int myadd (int arg1, int arg2)
    // and make it the current function in the current module.
    const char funcname[] = "myadd";
    llvm::Function *func = ll.make_function (funcname,        // name
                                             false,           // fastcall
                                             ll.type_int(),   // return
                                             ll.type_int(),   // arg1
                                             ll.type_int());  // arg2

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
    if (do_print)
        std::cout << "Generated the following bitcode:\n"
                  << ll.bitcode_string(func) << "\n";

    if (opt >= 0) {
        ll.setup_optimization_passes (opt);
        ll.do_optimize ();
        if (do_print)
            std::cout << "After optimizing:\n"
                      << ll.bitcode_string(func) << "\n";
    }

    ll.module_done ();

    // Ask for a callable function (will JIT on demand)
    IntFuncOfTwoInts myadd = (IntFuncOfTwoInts) ll.get_compiled_function (funcname);

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
        OSL::LLVM_Util ll;
        ll.orc_jit (orc);
        for (int i = 0; i < memtest; ++i) {
            IntFuncOfTwoInts f = test_big_func (ll, i==0);
            int r = f (42, 42);
            ASSERT (r == 84);
        }
        std::cout << "After " << memtest << " stupid functions compiled:\n";
        std::cout << "   RSS memory = "
                  << OIIO::Strutil::memformat(OIIO::Sysutil::memory_used()) << "\n";
    }

    return 0;
}
