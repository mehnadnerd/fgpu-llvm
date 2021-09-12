//===-- FgpuTargetInfo.cpp - Fgpu Target Implementation -------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "TargetInfo/FgpuTargetInfo.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target &llvm::getTheFgpuTarget() {
  static Target TheFgpuTarget;
  return TheFgpuTarget;
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeFgpuTargetInfo() {
  RegisterTarget<Triple::fgpu,
                 /*HasJIT=*/true>
      X(getTheFgpuTarget(), "fgpu", "FGPU (32-bit little endian)", "Fgpu");
}
