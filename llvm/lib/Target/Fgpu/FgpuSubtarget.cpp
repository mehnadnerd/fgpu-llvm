//===-- FgpuSubtarget.cpp - Fgpu Subtarget Information --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the Fgpu specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "FgpuSubtarget.h"
#include "Fgpu.h"
#include "FgpuMachineFunction.h"
#include "FgpuRegisterInfo.h"
#include "FgpuTargetMachine.h"
#include "FgpuCallLowering.h"
#include "FgpuLegalizerInfo.h"
#include "FgpuRegisterBankInfo.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "fgpu-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "FgpuGenSubtargetInfo.inc"

static cl::opt<bool>
    GPOpt("mgpopt", cl::Hidden,
          cl::desc("Enable gp-relative addressing of fgpu small data items"));

//bool FgpuSubtarget::GINVWarningPrinted = false;

void FgpuSubtarget::anchor() {}

FgpuSubtarget::FgpuSubtarget(const Triple &TT, StringRef CPU, StringRef FS,
                             const FgpuTargetMachine &TM,
                             MaybeAlign StackAlignOverride)
    : FgpuGenSubtargetInfo(TT, CPU, /*TuneCPU*/ CPU, FS),
      FgpuArchVersion(FgpuDefault),
      StackAlignOverride(StackAlignOverride), TM(TM), TargetTriple(TT),
      TSInfo(), InstrInfo(FgpuInstrInfo::create(
                    initializeSubtargetDependencies(CPU, FS, TM))),
      FrameLowering(FgpuFrameLowering::create(*this)),
      TLInfo(FgpuTargetLowering::create(TM, *this)) {

  if (FgpuArchVersion == FgpuDefault)  {
    ;;//don't do anything, only support this for now
  }

  // Set UseSmallSection.
  UseSmallSection = GPOpt;

  CallLoweringInfo.reset(new FgpuCallLowering(*getTargetLowering()));
  Legalizer.reset(new FgpuLegalizerInfo(*this));

  auto *RBI = new FgpuRegisterBankInfo(*getRegisterInfo());
  RegBankInfo.reset(RBI);
  InstSelector.reset(createFgpuInstructionSelector(
      *static_cast<const FgpuTargetMachine *>(&TM), *this, *RBI));
}

bool FgpuSubtarget::isPositionIndependent() const {
  return TM.isPositionIndependent();
}

/// This overrides the PostRAScheduler bit in the SchedModel for any CPU.
bool FgpuSubtarget::enablePostRAScheduler() const { return true; }

void FgpuSubtarget::getCriticalPathRCs(RegClassVector &CriticalPathRCs) const {
  CriticalPathRCs.clear();
  CriticalPathRCs.push_back(isGP64bit() ? &Fgpu::GPR64RegClass
                                        : &Fgpu::GPR32RegClass);
}

CodeGenOpt::Level FgpuSubtarget::getOptLevelToEnablePostRAScheduler() const {
  return CodeGenOpt::Aggressive;
}

FgpuSubtarget &
FgpuSubtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                               const TargetMachine &TM) {
  StringRef CPUName = FGPU_MC::selectFgpuCPU(TM.getTargetTriple(), CPU);

  // Parse features string.
  ParseSubtargetFeatures(CPUName, /*TuneCPU*/ CPUName, FS);
  // Initialize scheduling itinerary for the specified CPU.
  InstrItins = getInstrItineraryForCPU(CPUName);

  if (StackAlignOverride) {
    stackAlignment = *StackAlignOverride;
  } else {
    stackAlignment = Align(8);
  }

  return *this;
}

Reloc::Model FgpuSubtarget::getRelocationModel() const {
  return TM.getRelocationModel();
}

const FgpuABIInfo &FgpuSubtarget::getABI() const { return TM.getABI(); }

const CallLowering *FgpuSubtarget::getCallLowering() const {
  return CallLoweringInfo.get();
}

const LegalizerInfo *FgpuSubtarget::getLegalizerInfo() const {
  return Legalizer.get();
}

const RegisterBankInfo *FgpuSubtarget::getRegBankInfo() const {
  return RegBankInfo.get();
}

InstructionSelector *FgpuSubtarget::getInstructionSelector() const {
  return InstSelector.get();
}
