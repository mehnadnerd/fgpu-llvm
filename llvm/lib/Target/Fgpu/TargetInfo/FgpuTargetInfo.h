//===-- FgpuTargetInfo.h - Fgpu Target Implementation -----------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_FGPU_TARGETINFO_FGPUTARGETINFO_H
#define LLVM_LIB_TARGET_FGPU_TARGETINFO_FGPUTARGETINFO_H

namespace llvm {

class Target;

Target &getTheFgpuTarget();
Target &getTheFgpuelTarget();
Target &getTheFgpu64Target();
Target &getTheFgpu64elTarget();

} // namespace llvm

#endif // LLVM_LIB_TARGET_FGPU_TARGETINFO_FGPUTARGETINFO_H
