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

#pragma once

#include "export.h"
#include "oslconfig.h"

#include <memory>
#include <vector>

#ifndef OSL_LLVM_VERSION
#  include <llvm/Config/config.h>
#  define OSL_LLVM_VERSION (LLVM_VERSION_MAJOR*10+LLVM_VERSION_MINOR)
#endif

#if OSL_LLVM_VERSION < 36
#  include "llvm_util_old.h"
#else


#ifdef LLVM_NAMESPACE
namespace llvm = LLVM_NAMESPACE;
#endif

namespace llvm {
  // Opaque type definitions
  class BasicBlock;
  class ConstantFolder;
  class ExecutionEngine;
  class Function;
  class FunctionType;
  class LLVMContext;
  class Module;
  class PointerType;
  class Type;
  class Value;
  template<typename T, typename Inserter> class IRBuilder;
  class IRBuilderDefaultInserter;
}



OSL_NAMESPACE_ENTER


/// Wrapper class around LLVM functionality.  This handles all the
/// gory details of actually dealing with LLVM.  It should be sufficiently
/// generic that it would be useful for any LLVM-JITing app, and is not
/// tied to OSL internals at all.
class OSLEXECPUBLIC LLVM_Util {
public:
    LLVM_Util (int debuglevel=0);
    ~LLVM_Util ();

    struct PerThreadInfo;
    typedef llvm::IRBuilder<llvm::ConstantFolder,
                            llvm::IRBuilderDefaultInserter> IRBuilder;

    /// Set debug level
    void debug (int d);
    int debug () const;

    /// Set whether or not we are using ORC JIT (default: MCJIT)
    void orc_jit (bool enable);
    bool orc_jit () const;

    /// Return a reference to the current context.
    llvm::LLVMContext &context () const;

    /// Return a pointer to the current module.  Make a new one if
    /// necessary.
    llvm::Module *module ();

    /// Create a new empty module, make it the current module.
    llvm::Module *new_module (string_view id = "default");

    /// Create a new module, populated with functions from the buffer
    /// bitcode[0..size-1], make it the current module.  The name identifies
    /// the buffer.  If err is not NULL, error messages will be stored
    /// there.
    llvm::Module *module_from_bitcode (string_view bitcode,
                                       string_view name = "",
                                       std::string *err=NULL);

    enum HasVarArgs { NoVarArgs=0, VarArgs=1 };

    /// Create a new function (that will later be populated with
    /// instructions). If varargs is VarArgs, it will be marked as having
    /// var args (default: no). If stub is StubOnly, the function will be a
    /// stub only, but if it is FunctionWithCode (the default), the new
    /// function will be the "current function" with a new basic block and
    /// the insertion point set (so any subsequent ops will start filling in
    /// the function's code. If no module has yet been created, a
    /// new_module() will be created to house the function.
    llvm::Function *make_function (string_view, bool fastcall,
                                   llvm::Type *rettype,
                                   array_view<llvm::Type*> paramtypes,
                                   HasVarArgs varargs=NoVarArgs);
    /// make_function with individually-passed parameter types (for up to 5
    /// parameters).
    llvm::Function *make_function (string_view, bool fastcall,
                                   llvm::Type *rettype,
                                   llvm::Type *arg1=NULL,
                                   llvm::Type *arg2=NULL,
                                   llvm::Type *arg3=NULL,
                                   llvm::Type *arg4=NULL);

    /// Create the declaration for a function that will be resolved
    /// externally. It does not change the "current" function or the
    /// IR insertion point.
    llvm::Function *declare_extern_function (string_view, llvm::Type *rettype,
                                             array_view<llvm::Type*> paramtypes,
                                             HasVarArgs varargs=NoVarArgs);

    /// Set up a new current function that subsequent basic blocks will
    /// be added to.
    void current_function (llvm::Function *func);

    /// Return a ptr to the current function we're generating.
    llvm::Function *current_function () const;

    /// Return the value ptr for the a-th argument of the current function.
    llvm::Value *current_function_arg (int a);

    /// Return the current IR builder, create a new one (for the current
    /// function) if necessary.
    IRBuilder &builder ();

    /// Change symbols in the module that are marked as having external
    /// linkage to an alternate linkage that allows them to be discarded if
    /// not used within the module. Only do this for functions that start
    /// with prefix, and that DON'T match anything in the two exceptions
    /// lists.
    void internalize_module_functions (const std::string &prefix,
                                       const std::vector<std::string> &exceptions,
                                       const std::vector<std::string> &moreexceptions);

    /// Setup LLVM optimization passes.
    void setup_optimization_passes (int optlevel);

    /// Run the optimization passes.
    void do_optimize ();

    /// Call when we've finished adding functions to the module and are
    /// ready to optimize and codegen it.
    void module_done ();

    /// Retrieve a callable pointer to the JITed version of a function.
    /// This will JIT the function if it hasn't already done so. Be sure
    /// you have already called do_optimize() if you want optimization.
    void *get_compiled_function (string_view name);
    // void *get_compiled_function (llvm::Function *func);

    typedef void* (*FunctionResolver)(const std::string &);

    /// Wrap ExecutionEngine::InstallLazyFunctionCreator.
    void InstallLazyFunctionCreator (void* (*P)(const std::string &));


    /// Create a new LLVM basic block (for the current function) and return
    /// its handle. If insert is true, set the insertion point for new
    /// IR ops to be the new BB.
    llvm::BasicBlock *new_basic_block (string_view name="", bool insert=false);

    /// Save the return block pointer when entering a function. If
    /// after==NULL, generate a new basic block for where to go after the
    /// function return.  Return the after BB.
    llvm::BasicBlock *push_function (llvm::BasicBlock *after=NULL);

    /// Pop basic return destination when exiting a function.  This includes
    /// resetting the IR insertion point to the block following the
    /// corresponding function call.
    void pop_function ();

    /// Return the basic block where we go after returning from the current
    /// function.
    llvm::BasicBlock *return_block () const;

    /// Save the basic block pointers when entering a loop.
    void push_loop (llvm::BasicBlock *step, llvm::BasicBlock *after);

    /// Pop basic block pointers when exiting a loop.
    void pop_loop ();

    /// Return the basic block of the current loop's 'step' instructions.
    llvm::BasicBlock *loop_step_block () const;

    /// Return the basic block of the current loop's exit point.
    llvm::BasicBlock *loop_after_block () const;


    llvm::Type *type_float() const;
    llvm::Type *type_int() const;
    llvm::Type *type_addrint() const;
    llvm::Type *type_bool() const;
    llvm::Type *type_char() const;
    llvm::Type *type_longlong() const;
    llvm::Type *type_void() const;
    llvm::Type *type_triple() const;
    llvm::Type *type_matrix() const;
    llvm::Type *type_typedesc() const;
    llvm::PointerType *type_void_ptr() const;
    llvm::PointerType *type_string() const;
    llvm::PointerType *type_ustring_ptr() const;
    llvm::PointerType *type_char_ptr() const;
    llvm::PointerType *type_int_ptr() const;
    llvm::PointerType *type_float_ptr() const;
    llvm::PointerType *type_triple_ptr() const;
    llvm::PointerType *type_matrix_ptr() const;

    /// Generate the appropriate llvm type definition for a TypeDesc
    /// (this is the actual type, for example when we allocate it).
    llvm::Type *llvm_type (const OIIO::TypeDesc &typedesc);

    /// This will return a llvm::Type that is the same as a C union of
    /// the given types[].
    llvm::Type *type_union (const std::vector<llvm::Type *> &types);

    /// This will return a llvm::Type that is the same as a C struct
    /// comprised fields of the given types[], in order.
    llvm::Type *type_struct (const std::vector<llvm::Type *> &types,
                             string_view name="");

    /// Return the llvm::Type that is a pointer to the given llvm type.
    llvm::Type *type_ptr (llvm::Type *type);

    /// Return the llvm::Type that is an array of n elements of the given
    /// llvm type.
    llvm::Type *type_array (llvm::Type *type, int n);

    /// Return an llvm::FunctionType that describes a function with the
    /// given return types, parameter types (in a vector), and whether it
    /// uses varargs conventions.
    llvm::FunctionType *type_function (llvm::Type *rettype,
                                       array_view<llvm::Type*> params,
                                       HasVarArgs varargs=NoVarArgs);

    /// Return a llvm::PointerType that's a pointer to the described
    /// kind of function.
    llvm::PointerType *type_function_ptr (llvm::Type *rettype,
                                          array_view<llvm::Type*> params,
                                          HasVarArgs varargs=NoVarArgs);

    /// Return the human-readable name of the type of the llvm type.
    std::string llvm_typename (llvm::Type *type) const;

    /// Return the llvm::Type of the llvm value.
    llvm::Type *llvm_typeof (llvm::Value *val) const;

    /// Return the human-readable name of the type of the llvm value.
    std::string llvm_typenameof (llvm::Value *val) const;

    /// Return an llvm::Value holding the given floating point constant.
    llvm::Value *constant (float f);

    /// Return an llvm::Value holding the given integer constant.
    llvm::Value *constant (int i);

    /// Return an llvm::Value holding the given size_t constant.
    llvm::Value *constant (size_t i);

    /// Return an llvm::Value holding the given bool constant.
    /// Change the name so it doesn't get mixed up with int.
    llvm::Value *constant_bool (bool b);

    /// Return a constant void pointer to the given constant address.
    /// If the type specified is NULL, it will make a 'void *'.
    llvm::Value *constant_ptr (void *p, llvm::PointerType *type=NULL);

    /// Return an llvm::Value holding the given string constant.
    llvm::Value *constant (OIIO::ustring s);
    llvm::Value *constant (string_view s) {
        return constant(OIIO::ustring(s));
    }

    /// Return an llvm::Value for a long long that is a packed
    /// representation of a TypeDesc.
    llvm::Value *constant (const OIIO::TypeDesc &type);

    /// Return an llvm::Value for a void* variable with value NULL.
    llvm::Value *void_ptr_null ();

    /// Cast the pointer variable specified by val to the kind of pointer
    /// described by type (as an llvm pointer type).
    llvm::Value *ptr_cast (llvm::Value* val, llvm::Type *type);
    llvm::Value *ptr_cast (llvm::Value* val, llvm::PointerType *type) {
        return ptr_cast (val, (llvm::Type *)type);
    }

    /// Cast the pointer variable specified by val to a pointer to the type
    /// described by type (as an llvm data type).
    llvm::Value *ptr_to_cast (llvm::Value* val, llvm::Type *type);

    /// Cast the pointer variable specified by val to a pointer to the given
    /// data type, return the llvm::Value of the new pointer.
    llvm::Value *ptr_cast (llvm::Value* val, const OIIO::TypeDesc &type);

    /// Cast the pointer variable specified by val to a pointer of type
    /// void* return the llvm::Value of the new pointer.
    llvm::Value *void_ptr (llvm::Value* val);

    /// Generate a pointer that is (ptrtype)((char *)ptr + offset).
    /// If ptrtype is NULL, just return a void*.
    llvm::Value *offset_ptr (llvm::Value *ptr, int offset,
                             llvm::Type *ptrtype=NULL);

    /// Generate an alloca instruction to allocate space for n copies of the
    /// given llvm type, and return its pointer.
    llvm::Value *op_alloca (llvm::Type *llvmtype, int n=1,
                            string_view name="");
    llvm::Value *op_alloca (llvm::PointerType *llvmtype, int n=1,
                            string_view name="") {
        return op_alloca ((llvm::Type *)llvmtype, n, name);
    }

    /// Generate an alloca instruction to allocate space for n copies of the
    /// given type, and return its pointer.
    llvm::Value *op_alloca (const OIIO::TypeDesc &type, int n=1,
                            string_view name="");

    /// Generate code for a call to the function pointer, with the given
    /// arg list.  Return an llvm::Value* corresponding to the return
    /// value of the function, if any.
    llvm::Value *call_function (llvm::Value *func,
                                OIIO::array_view<llvm::Value *> args);
    /// Generate code for a call to the named function with the given arg
    /// list.  Return an llvm::Value* corresponding to the return value of
    /// the function, if any.
    llvm::Value *call_function (string_view name,
                                OIIO::array_view<llvm::Value *> args);

    llvm::Value *call_function (string_view name, llvm::Value *arg0) {
        llvm::Value *args[1] = { arg0 };
        return call_function (name, args);
    }
    llvm::Value *call_function (string_view name, llvm::Value *arg0,
                                llvm::Value *arg1) {
        llvm::Value *args[2] = { arg0, arg1 };
        return call_function (name, args);
    }
    llvm::Value *call_function (string_view name, llvm::Value *arg0,
                                llvm::Value *arg1, llvm::Value *arg2) {
        llvm::Value *args[3] = { arg0, arg1, arg2 };
        return call_function (name, args);
    }
    llvm::Value *call_function (string_view name, llvm::Value *arg0,
                                llvm::Value *arg1, llvm::Value *arg2,
                                llvm::Value *arg3) {
        llvm::Value *args[4] = { arg0, arg1, arg2, arg3 };
        return call_function (name, args);
    }

    /// Mark the function call (which MUST be the value returned by a
    /// call_function()) as using the 'fast' calling convention.
    void mark_fast_func_call (llvm::Value *funccall);

    /// Set the code insertion point for subsequent ops to block.
    /// If block is NULL, create a new basic block for the current function.
    void set_insert_point (llvm::BasicBlock *block=NULL);

    /// Return op from a void function.  If retval is NULL, we are returning
    /// from a void function.
    void op_return (llvm::Value *retval=NULL);

    /// Create a branch instruction to block and establish that as the as
    /// the new code insertion point.
    void op_branch (llvm::BasicBlock *block);

    /// Create a conditional branch instruction to trueblock if cond is
    /// true, to falseblock if cond is false, and establish trueblock as the
    /// new insertion point).
    void op_branch (llvm::Value *cond, llvm::BasicBlock *trueblock,
                    llvm::BasicBlock *falseblock);

    /// Generate code for a memset.
    void op_memset (llvm::Value *ptr, int val, int len, int align=1);

    /// Generate code for variable size memset
    void op_memset (llvm::Value *ptr, int val, llvm::Value *len, int align=1);

    /// Generate code for a memcpy.
    void op_memcpy (llvm::Value *dst, llvm::Value *src, int len, int align=1);

    /// Dereference a pointer:  return *ptr
    llvm::Value *op_load (llvm::Value *ptr);

    /// Store to a dereferenced pointer:   *ptr = val
    void op_store (llvm::Value *val, llvm::Value *ptr);

    // N.B. "GEP" -- GetElementPointer -- is a particular LLVM-ism that is
    // the means for retrieving elements from some kind of aggregate: the
    // i-th field in a struct, the i-th element of an array.  They can be
    // chained together, to get at items in a recursive hierarchy.

    /// Generate a GEP (get element pointer) where the element index is an
    /// llvm::Value, which can be generated from either a constant or a
    /// runtime-computed integer element index.
    llvm::Value *GEP (llvm::Value *ptr, llvm::Value *elem);

    /// Generate a GEP (get element pointer) with an integer element
    /// offset.
    llvm::Value *GEP (llvm::Value *ptr, int elem);

    /// Generate a GEP (get element pointer) with two integer element
    /// offsets.  This is just a special (and common) case of GEP where
    /// we have a 2-level hierarchy and we have fixed element indices
    /// that are known at compile time.
    llvm::Value *GEP (llvm::Value *ptr, int elem1, int elem2);

    // Arithmetic ops.  It auto-detects the type (int vs float).
    // ...
    llvm::Value *op_add (llvm::Value *a, llvm::Value *b);
    llvm::Value *op_sub (llvm::Value *a, llvm::Value *b);
    llvm::Value *op_neg (llvm::Value *a);
    llvm::Value *op_mul (llvm::Value *a, llvm::Value *b);
    llvm::Value *op_div (llvm::Value *a, llvm::Value *b);
    llvm::Value *op_mod (llvm::Value *a, llvm::Value *b);
    llvm::Value *op_float_to_int (llvm::Value *a);
    llvm::Value *op_int_to_float (llvm::Value *a);
    llvm::Value *op_bool_to_int (llvm::Value *a);
    llvm::Value *op_float_to_double (llvm::Value *a);

    llvm::Value *op_and (llvm::Value *a, llvm::Value *b);
    llvm::Value *op_or (llvm::Value *a, llvm::Value *b);
    llvm::Value *op_xor (llvm::Value *a, llvm::Value *b);
    llvm::Value *op_shl (llvm::Value *a, llvm::Value *b);
    llvm::Value *op_shr (llvm::Value *a, llvm::Value *b);
    llvm::Value *op_not (llvm::Value *a);

    /// Generate IR for (cond ? a : b).  Cond should be a bool.
    llvm::Value *op_select (llvm::Value *cond, llvm::Value *a, llvm::Value *b);

    // Comparison ops.  It auto-detects the type (int vs float).
    // ordered only applies to float comparisons -- ordered means the
    // comparison will succeed only if neither arg is NaN.
    // ...
    llvm::Value *op_eq (llvm::Value *a, llvm::Value *b, bool ordered=false);
    llvm::Value *op_ne (llvm::Value *a, llvm::Value *b, bool ordered=false);
    llvm::Value *op_gt (llvm::Value *a, llvm::Value *b, bool ordered=false);
    llvm::Value *op_lt (llvm::Value *a, llvm::Value *b, bool ordered=false);
    llvm::Value *op_ge (llvm::Value *a, llvm::Value *b, bool ordered=false);
    llvm::Value *op_le (llvm::Value *a, llvm::Value *b, bool ordered=false);

    /// Write the module's bitcode (after compilation/optimization) to a
    /// file.  If err is not NULL, errors will be deposited there.
    void write_bitcode_file (const char *filename, std::string *err=NULL);

    /// Convert a function's bitcode to a string.
    std::string bitcode_string (llvm::Function *func);

    /// Delete the IR for the body of the given function to reclaim its
    /// memory (only helpful if we know we won't use it again).
    void delete_func_body (llvm::Function *func);

    /// Is the function empty, except for simply a ret statement?
    bool func_is_empty (llvm::Function *func);

    std::string func_name (llvm::Function *f);

    static size_t total_jit_memory_held ();

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
    Impl *impl() { return m_impl.get(); }
};



OSL_NAMESPACE_EXIT


#endif
