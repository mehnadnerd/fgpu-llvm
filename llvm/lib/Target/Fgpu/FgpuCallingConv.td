//===-- FgpuCallingConv.td - Calling Conventions for Fgpu --*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
// This describes the calling conventions for Fgpu architecture.
//===----------------------------------------------------------------------===//

def RetCC_FgpuEABI : CallingConv<[
  CCIfType<[i32,f32], CCAssignToReg<[R2, R3, R4, R5]>>
]>;

def RetCC_Fgpu : CallingConv<[
  CCDelegateTo<RetCC_FgpuEABI>
]>;

def CSR_CC_Fgpu  : CalleeSavedRegs<(sequence "R%u", 6, 29)>;

