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

// FIXME: Maybe this should be on by default when Fgpu16 is specified
//
static cl::opt<bool>
    Mixed16_32("fgpu-mixed-16-32", cl::init(false),
               cl::desc("Allow for a mixture of Fgpu16 "
                        "and Fgpu32 code in a single output file"),
               cl::Hidden);

static cl::opt<bool> Fgpu_Os16("fgpu-os16", cl::init(false),
                               cl::desc("Compile all functions that don't use "
                                        "floating point as Fgpu 16"),
                               cl::Hidden);

static cl::opt<bool> Fgpu16HardFloat("fgpu16-hard-float", cl::NotHidden,
                                     cl::desc("Enable fgpu16 hard float."),
                                     cl::init(false));

static cl::opt<bool>
    Fgpu16ConstantIslands("fgpu16-constant-islands", cl::NotHidden,
                          cl::desc("Enable fgpu16 constant islands."),
                          cl::init(true));

static cl::opt<bool>
    GPOpt("mgpopt", cl::Hidden,
          cl::desc("Enable gp-relative addressing of fgpu small data items"));

bool FgpuSubtarget::DspWarningPrinted = false;
bool FgpuSubtarget::MSAWarningPrinted = false;
bool FgpuSubtarget::VirtWarningPrinted = false;
bool FgpuSubtarget::CRCWarningPrinted = false;
bool FgpuSubtarget::GINVWarningPrinted = false;

void FgpuSubtarget::anchor() {}

FgpuSubtarget::FgpuSubtarget(const Triple &TT, StringRef CPU, StringRef FS,
                             bool little, const FgpuTargetMachine &TM,
                             MaybeAlign StackAlignOverride)
    : FgpuGenSubtargetInfo(TT, CPU, /*TuneCPU*/ CPU, FS),
      FgpuArchVersion(FgpuDefault), IsLittle(little), IsSoftFloat(false),
      IsSingleFloat(false), IsFPXX(false), NoABICalls(false), Abs2008(false),
      IsFP64bit(false), UseOddSPReg(true), IsNaN2008bit(false),
      IsGP64bit(false), HasVFPU(false), HasCnFgpu(false), HasCnFgpuP(false),
      HasFgpu3_32(false), HasFgpu3_32r2(false), HasFgpu4_32(false),
      HasFgpu4_32r2(false), HasFgpu5_32r2(false), InFgpu16Mode(false),
      InFgpu16HardFloat(Fgpu16HardFloat), InMicroFgpuMode(false), HasDSP(false),
      HasDSPR2(false), HasDSPR3(false), AllowMixed16_32(Mixed16_32 | Fgpu_Os16),
      Os16(Fgpu_Os16), HasMSA(false), UseTCCInDIV(false), HasSym32(false),
      HasEVA(false), DisableMadd4(false), HasMT(false), HasCRC(false),
      HasVirt(false), HasGINV(false), UseIndirectJumpsHazard(false),
      StackAlignOverride(StackAlignOverride), TM(TM), TargetTriple(TT),
      TSInfo(), InstrInfo(FgpuInstrInfo::create(
                    initializeSubtargetDependencies(CPU, FS, TM))),
      FrameLowering(FgpuFrameLowering::create(*this)),
      TLInfo(FgpuTargetLowering::create(TM, *this)) {

  if (FgpuArchVersion == FgpuDefault)
    FgpuArchVersion = Fgpu32;

  // Don't even attempt to generate code for FGPU-I and FGPU-V. They have not
  // been tested and currently exist for the integrated assembler only.
  if (FgpuArchVersion == Fgpu1)
    report_fatal_error("Code generation for FGPU-I is not implemented", false);
  if (FgpuArchVersion == Fgpu5)
    report_fatal_error("Code generation for FGPU-V is not implemented", false);

  // Check if Architecture and ABI are compatible.
  assert(((!isGP64bit() && isABI_O32()) ||
          (isGP64bit() && (isABI_N32() || isABI_N64()))) &&
         "Invalid  Arch & ABI pair.");

  if (hasMSA() && !isFP64bit())
    report_fatal_error("MSA requires a 64-bit FPU register file (FR=1 mode). "
                       "See -mattr=+fp64.",
                       false);

  if (isFP64bit() && !hasFgpu64() && hasFgpu32() && !hasFgpu32r2())
    report_fatal_error(
        "FPU with 64-bit registers is not available on FGPU32 pre revision 2. "
        "Use -mcpu=fgpu32r2 or greater.");

  if (!isABI_O32() && !useOddSPReg())
    report_fatal_error("-mattr=+nooddspreg requires the O32 ABI.", false);

  if (IsFPXX && (isABI_N32() || isABI_N64()))
    report_fatal_error("FPXX is not permitted for the N32/N64 ABI's.", false);

  if (hasFgpu64r6() && InMicroFgpuMode)
    report_fatal_error("microFGPU64R6 is not supported", false);

  if (!isABI_O32() && InMicroFgpuMode)
    report_fatal_error("microFGPU64 is not supported.", false);

  if (UseIndirectJumpsHazard) {
    if (InMicroFgpuMode)
      report_fatal_error(
          "cannot combine indirect jumps with hazard barriers and microFGPU");
    if (!hasFgpu32r2())
      report_fatal_error(
          "indirect jumps with hazard barriers requires FGPU32R2 or later");
  }
  if (inAbs2008Mode() && hasFgpu32() && !hasFgpu32r2()) {
    report_fatal_error("IEEE 754-2008 abs.fmt is not supported for the given "
                       "architecture.",
                       false);
  }

  if (hasFgpu32r6()) {
    StringRef ISA = hasFgpu64r6() ? "FGPU64r6" : "FGPU32r6";

    assert(isFP64bit());
    assert(isNaN2008());
    assert(inAbs2008Mode());
    if (hasDSP())
      report_fatal_error(ISA + " is not compatible with the DSP ASE", false);
  }

  if (NoABICalls && TM.isPositionIndependent())
    report_fatal_error("position-independent code requires '-mabicalls'");

  if (isABI_N64() && !TM.isPositionIndependent() && !hasSym32())
    NoABICalls = true;

  // Set UseSmallSection.
  UseSmallSection = GPOpt;
  if (!NoABICalls && GPOpt) {
    errs() << "warning: cannot use small-data accesses for '-mabicalls'"
           << "\n";
    UseSmallSection = false;
  }

  if (hasDSPR2() && !DspWarningPrinted) {
    if (hasFgpu64() && !hasFgpu64r2()) {
      errs() << "warning: the 'dspr2' ASE requires FGPU64 revision 2 or "
             << "greater\n";
      DspWarningPrinted = true;
    } else if (hasFgpu32() && !hasFgpu32r2()) {
      errs() << "warning: the 'dspr2' ASE requires FGPU32 revision 2 or "
             << "greater\n";
      DspWarningPrinted = true;
    }
  } else if (hasDSP() && !DspWarningPrinted) {
    if (hasFgpu64() && !hasFgpu64r2()) {
      errs() << "warning: the 'dsp' ASE requires FGPU64 revision 2 or "
             << "greater\n";
      DspWarningPrinted = true;
    } else if (hasFgpu32() && !hasFgpu32r2()) {
      errs() << "warning: the 'dsp' ASE requires FGPU32 revision 2 or "
             << "greater\n";
      DspWarningPrinted = true;
    }
  }

  StringRef ArchName = hasFgpu64() ? "FGPU64" : "FGPU32";

  if (!hasFgpu32r5() && hasMSA() && !MSAWarningPrinted) {
    errs() << "warning: the 'msa' ASE requires " << ArchName
           << " revision 5 or greater\n";
    MSAWarningPrinted = true;
  }
  if (!hasFgpu32r5() && hasVirt() && !VirtWarningPrinted) {
    errs() << "warning: the 'virt' ASE requires " << ArchName
           << " revision 5 or greater\n";
    VirtWarningPrinted = true;
  }
  if (!hasFgpu32r6() && hasCRC() && !CRCWarningPrinted) {
    errs() << "warning: the 'crc' ASE requires " << ArchName
           << " revision 6 or greater\n";
    CRCWarningPrinted = true;
  }
  if (!hasFgpu32r6() && hasGINV() && !GINVWarningPrinted) {
    errs() << "warning: the 'ginv' ASE requires " << ArchName
           << " revision 6 or greater\n";
    GINVWarningPrinted = true;
  }

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

  if (InFgpu16Mode && !IsSoftFloat)
    InFgpu16HardFloat = true;

  if (StackAlignOverride)
    stackAlignment = *StackAlignOverride;
  else if (isABI_N32() || isABI_N64())
    stackAlignment = Align(16);
  else {
    assert(isABI_O32() && "Unknown ABI for stack alignment!");
    stackAlignment = Align(8);
  }

  if ((isABI_N32() || isABI_N64()) && !isGP64bit())
    report_fatal_error("64-bit code requested on a subtarget that doesn't "
                       "support it!");

  return *this;
}

bool FgpuSubtarget::useConstantIslands() {
  LLVM_DEBUG(dbgs() << "use constant islands " << Fgpu16ConstantIslands
                    << "\n");
  return Fgpu16ConstantIslands;
}

Reloc::Model FgpuSubtarget::getRelocationModel() const {
  return TM.getRelocationModel();
}

bool FgpuSubtarget::isABI_N64() const { return getABI().IsN64(); }
bool FgpuSubtarget::isABI_N32() const { return getABI().IsN32(); }
bool FgpuSubtarget::isABI_O32() const { return getABI().IsO32(); }
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
