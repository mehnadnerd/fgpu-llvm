//===--- Fgpu.h - Fgpu-specific Tool Helpers ----------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_LIB_DRIVER_TOOLCHAINS_ARCH_FGPU_H
#define LLVM_CLANG_LIB_DRIVER_TOOLCHAINS_ARCH_FGPU_H

#include "clang/Driver/Driver.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/Triple.h"
#include "llvm/Option/Option.h"
#include <string>
#include <vector>

namespace clang {
namespace driver {
namespace tools {

namespace fgpu {
typedef enum { Legacy = 1, Std2008 = 2 } IEEE754Standard;

enum class FloatABI {
  Invalid,
  Soft,
  Hard,
};

IEEE754Standard getIEEE754Standard(StringRef &CPU);
bool hasCompactBranches(StringRef &CPU);
void getFgpuCPUAndABI(const llvm::opt::ArgList &Args,
                      const llvm::Triple &Triple, StringRef &CPUName,
                      StringRef &ABIName);
void getFGPUTargetFeatures(const Driver &D, const llvm::Triple &Triple,
                           const llvm::opt::ArgList &Args,
                           std::vector<StringRef> &Features);
StringRef getGnuCompatibleFgpuABIName(StringRef ABI);
fgpu::FloatABI getFgpuFloatABI(const Driver &D, const llvm::opt::ArgList &Args,
                               const llvm::Triple &Triple);
std::string getFgpuABILibSuffix(const llvm::opt::ArgList &Args,
                                const llvm::Triple &Triple);
bool hasFgpuAbiArg(const llvm::opt::ArgList &Args, const char *Value);
bool isUCLibc(const llvm::opt::ArgList &Args);
bool isNaN2008(const llvm::opt::ArgList &Args, const llvm::Triple &Triple);
bool isFP64ADefault(const llvm::Triple &Triple, StringRef CPUName);
bool isFPXXDefault(const llvm::Triple &Triple, StringRef CPUName,
                   StringRef ABIName, fgpu::FloatABI FloatABI);
bool shouldUseFPXX(const llvm::opt::ArgList &Args, const llvm::Triple &Triple,
                   StringRef CPUName, StringRef ABIName,
                   fgpu::FloatABI FloatABI);
bool supportsIndirectJumpHazardBarrier(StringRef &CPU);

} // end namespace fgpu
} // end namespace target
} // end namespace driver
} // end namespace clang

#endif // LLVM_CLANG_LIB_DRIVER_TOOLCHAINS_ARCH_FGPU_H
