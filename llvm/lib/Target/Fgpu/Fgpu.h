//===-- Fgpu.h - Top-level interface for Fgpu representation ----*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in
// the LLVM Fgpu back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_FGPU_FGPU_H
#define LLVM_LIB_TARGET_FGPU_FGPU_H

#include "MCTargetDesc/FgpuMCTargetDesc.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
  class FgpuTargetMachine;
  class ModulePass;
  class FunctionPass;
  class FgpuRegisterBankInfo;
  class FgpuSubtarget;
  class FgpuTargetMachine;
  class InstructionSelector;
  class PassRegistry;


  FunctionPass *createFgpuModuleISelDagPass();
  FunctionPass *createFgpuOptimizePICCallPass();
  FunctionPass *createFgpuBranchExpansion();
  FunctionPass *createFgpuExpandPseudoPass();
  FunctionPass *createFgpuPreLegalizeCombiner();

  InstructionSelector *createFgpuInstructionSelector(const FgpuTargetMachine &,
                                                     FgpuSubtarget &,
                                                     FgpuRegisterBankInfo &);

  void initializeFgpuBranchExpansionPass(PassRegistry &);
  void initializeFgpuPreLegalizerCombinerPass(PassRegistry&);
} // end namespace llvm;

#endif
