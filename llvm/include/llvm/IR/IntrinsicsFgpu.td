//===- IntrinsicsFgpu.td - Defines Fgpu intrinsics ---------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines all of the FGPU-specific intrinsics.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// FGPU DSP data types
def fgpu_v32f32_ty: LLVMType<v32f32>;

let TargetPrefix = "fgpu" in {  // All intrinsics start with "llvm.fgpu.".

def int_fgpu_lid : GCCBuiltin<"__builtin_fgpu_lid">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrNoMem]>;

def int_fgpu_size : GCCBuiltin<"__builtin_fgpu_size">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrNoMem]>;

def int_fgpu_wgid : GCCBuiltin<"__builtin_fgpu_wgid">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrNoMem]>;

def int_fgpu_wgsize : GCCBuiltin<"__builtin_fgpu_wgsize">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrNoMem]>;

def int_fgpu_wgoff : GCCBuiltin<"__builtin_fgpu_wgoff">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrNoMem]>;

def int_fgpu_dot : GCCBuiltin<"__builtin_fgpu_dot">,
  Intrinsic<[fgpu_v32f32_ty], [fgpu_v32f32_ty, llvm_ptr_ty, llvm_i32_ty], [IntrNoMem]>;

def int_fgpu_lsync : GCCBuiltin<"__builtin_fgpu_lsync">,
  Intrinsic<[], [], []>;

def int_fgpu_gsync : GCCBuiltin<"__builtin_fgpu_gsync">,
  Intrinsic<[], [], []>;

//def int_fgpu_xori_b : GCCBuiltin<"__builtin_msa_xori_b">,
//  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

}
