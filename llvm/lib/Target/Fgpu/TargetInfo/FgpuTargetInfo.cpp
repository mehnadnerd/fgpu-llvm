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
Target &llvm::getTheFgpuelTarget() {
  static Target TheFgpuelTarget;
  return TheFgpuelTarget;
}
Target &llvm::getTheFgpu64Target() {
  static Target TheFgpu64Target;
  return TheFgpu64Target;
}
Target &llvm::getTheFgpu64elTarget() {
  static Target TheFgpu64elTarget;
  return TheFgpu64elTarget;
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeFgpuTargetInfo() {
  RegisterTarget<Triple::fgpu,
                 /*HasJIT=*/true>
      X(getTheFgpuTarget(), "fgpu", "FGPU (32-bit little endian)", "Fgpu");

  RegisterTarget<Triple::fgpu,
                 /*HasJIT=*/true>
      Y(getTheFgpuelTarget(), "fgpuel", "FGPU (32-bit little endian)", "Fgpu");

  RegisterTarget<Triple::fgpu,
                 /*HasJIT=*/true>
      A(getTheFgpu64Target(), "fgpu64", "FGPU (64-bit big endian)", "Fgpu");

  RegisterTarget<Triple::fgpu,
                 /*HasJIT=*/true>
      B(getTheFgpu64elTarget(), "fgpu64el", "FGPU (64-bit little endian)",
        "Fgpu");
}
