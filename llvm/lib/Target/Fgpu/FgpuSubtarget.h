//===-- FgpuSubtarget.h - Define Subtarget for the Fgpu ---------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the Fgpu specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Fgpu_FgpuSUBTARGET_H
#define LLVM_LIB_TARGET_Fgpu_FgpuSUBTARGET_H

#include "MCTargetDesc/FgpuABIInfo.h"
#include "FgpuFrameLowering.h"
#include "FgpuISelLowering.h"
#include "FgpuInstrInfo.h"
#include "llvm/CodeGen/SelectionDAGTargetInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/CodeGen/GlobalISel/CallLowering.h"
#include "llvm/CodeGen/GlobalISel/LegalizerInfo.h"
#include "llvm/CodeGen/GlobalISel/RegisterBankInfo.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelector.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/MC/MCInstrItineraries.h"
#include "llvm/Support/ErrorHandling.h"
#include <string>

#define GET_SUBTARGETINFO_HEADER
#include "FgpuGenSubtargetInfo.inc"

namespace llvm {
class StringRef;

class FgpuTargetMachine;

class FgpuSubtarget : public FgpuGenSubtargetInfo {
  virtual void anchor();

  enum FgpuArchEnum {
    FgpuDefault,
    Fgpu1, Fgpu2, Fgpu32, Fgpu32r2, Fgpu32r3, Fgpu32r5, Fgpu32r6, Fgpu32Max,
    Fgpu3, Fgpu4, Fgpu5, Fgpu64, Fgpu64r2, Fgpu64r3, Fgpu64r5, Fgpu64r6
  };

  enum class CPU { P5600 };

  // Used to avoid printing dsp warnings multiple times.
  static bool DspWarningPrinted;

  // Used to avoid printing msa warnings multiple times.
  static bool MSAWarningPrinted;

  // Used to avoid printing crc warnings multiple times.
  static bool CRCWarningPrinted;

  // Used to avoid printing ginv warnings multiple times.
  static bool GINVWarningPrinted;

  // Used to avoid printing virt warnings multiple times.
  static bool VirtWarningPrinted;

  // Fgpu architecture version
  FgpuArchEnum FgpuArchVersion;

  // Processor implementation (unused but required to exist by
  // tablegen-erated code).
  CPU ProcImpl;

  // IsLittle - The target is Little Endian
  bool IsLittle;

  // IsSoftFloat - The target does not support any floating point instructions.
  bool IsSoftFloat;

  // IsSingleFloat - The target only supports single precision float
  // point operations. This enable the target to use all 32 32-bit
  // floating point registers instead of only using even ones.
  bool IsSingleFloat;

  // IsFPXX - Fgpu O32 modeless ABI.
  bool IsFPXX;

  // NoABICalls - Disable SVR4-style position-independent code.
  bool NoABICalls;

  // Abs2008 - Use IEEE 754-2008 abs.fmt instruction.
  bool Abs2008;

  // IsFP64bit - The target processor has 64-bit floating point registers.
  bool IsFP64bit;

  /// Are odd single-precision registers permitted?
  /// This corresponds to -modd-spreg and -mno-odd-spreg
  bool UseOddSPReg;

  // IsNan2008 - IEEE 754-2008 NaN encoding.
  bool IsNaN2008bit;

  // IsGP64bit - General-purpose registers are 64 bits wide
  bool IsGP64bit;

  // IsPTR64bit - Pointers are 64 bit wide
  bool IsPTR64bit;

  // HasVFPU - Processor has a vector floating point unit.
  bool HasVFPU;

  // CPU supports cnFgpu (Cavium Networks Octeon CPU).
  bool HasCnFgpu;

  // CPU supports cnFgpuP (Cavium Networks Octeon+ CPU).
  bool HasCnFgpuP;

  // isLinux - Target system is Linux. Is false we consider ELFOS for now.
  bool IsLinux;

  // UseSmallSection - Small section is used.
  bool UseSmallSection;

  /// Features related to the presence of specific instructions.

  // HasFgpu3_32 - The subset of Fgpu-III instructions added to Fgpu32
  bool HasFgpu3_32;

  // HasFgpu3_32r2 - The subset of Fgpu-III instructions added to Fgpu32r2
  bool HasFgpu3_32r2;

  // HasFgpu4_32 - Has the subset of Fgpu-IV present in Fgpu32
  bool HasFgpu4_32;

  // HasFgpu4_32r2 - Has the subset of Fgpu-IV present in Fgpu32r2
  bool HasFgpu4_32r2;

  // HasFgpu5_32r2 - Has the subset of Fgpu-V present in Fgpu32r2
  bool HasFgpu5_32r2;

  // InFgpu16 -- can process Fgpu16 instructions
  bool InFgpu16Mode;

  // Fgpu16 hard float
  bool InFgpu16HardFloat;

  // InMicroFgpu -- can process MicroFgpu instructions
  bool InMicroFgpuMode;

  // HasDSP, HasDSPR2, HasDSPR3 -- supports DSP ASE.
  bool HasDSP, HasDSPR2, HasDSPR3;

  // Has3D -- Supports Fgpu3D ASE.
  bool Has3D;

  // Allow mixed Fgpu16 and Fgpu32 in one source file
  bool AllowMixed16_32;

  // Optimize for space by compiling all functions as Fgpu 16 unless
  // it needs floating point. Functions needing floating point are
  // compiled as Fgpu32
  bool Os16;

  // HasMSA -- supports MSA ASE.
  bool HasMSA;

  // UseTCCInDIV -- Enables the use of trapping in the assembler.
  bool UseTCCInDIV;

  // Sym32 -- On Fgpu64 symbols are 32 bits.
  bool HasSym32;

  // HasEVA -- supports EVA ASE.
  bool HasEVA;

  // nomadd4 - disables generation of 4-operand madd.s, madd.d and
  // related instructions.
  bool DisableMadd4;

  // HasMT -- support MT ASE.
  bool HasMT;

  // HasCRC -- supports R6 CRC ASE
  bool HasCRC;

  // HasVirt -- supports Virtualization ASE
  bool HasVirt;

  // HasGINV -- supports R6 Global INValidate ASE
  bool HasGINV;

  // Use hazard variants of the jump register instructions for indirect
  // function calls and jump tables.
  bool UseIndirectJumpsHazard;

  // Disable use of the `jal` instruction.
  bool UseLongCalls = false;

  // Assume 32-bit GOT.
  bool UseXGOT = false;

  /// The minimum alignment known to hold of the stack frame on
  /// entry to the function and which must be maintained by every function.
  Align stackAlignment;

  /// The overridden stack alignment.
  MaybeAlign StackAlignOverride;

  InstrItineraryData InstrItins;

  // We can override the determination of whether we are in Fgpu16 mode
  // as from the command line
  enum {NoOverride, Fgpu16Override, NoFgpu16Override} OverrideMode;

  const FgpuTargetMachine &TM;

  Triple TargetTriple;

  const SelectionDAGTargetInfo TSInfo;
  std::unique_ptr<const FgpuInstrInfo> InstrInfo;
  std::unique_ptr<const FgpuFrameLowering> FrameLowering;
  std::unique_ptr<const FgpuTargetLowering> TLInfo;

public:
  bool isPositionIndependent() const;
  /// This overrides the PostRAScheduler bit in the SchedModel for each CPU.
  bool enablePostRAScheduler() const override;
  void getCriticalPathRCs(RegClassVector &CriticalPathRCs) const override;
  CodeGenOpt::Level getOptLevelToEnablePostRAScheduler() const override;

  bool isABI_N64() const;
  bool isABI_N32() const;
  bool isABI_O32() const;
  const FgpuABIInfo &getABI() const;
  bool isABI_FPXX() const { return isABI_O32() && IsFPXX; }

  /// This constructor initializes the data members to match that
  /// of the specified triple.
  FgpuSubtarget(const Triple &TT, StringRef CPU, StringRef FS, bool little,
                const FgpuTargetMachine &TM, MaybeAlign StackAlignOverride);

  /// ParseSubtargetFeatures - Parses features string setting specified
  /// subtarget options.  Definition of function is auto generated by tblgen.
  void ParseSubtargetFeatures(StringRef CPU, StringRef TuneCPU, StringRef FS);

  bool hasFgpu1() const { return FgpuArchVersion >= Fgpu1; }
  bool hasFgpu2() const { return FgpuArchVersion >= Fgpu2; }
  bool hasFgpu3() const { return FgpuArchVersion >= Fgpu3; }
  bool hasFgpu4() const { return FgpuArchVersion >= Fgpu4; }
  bool hasFgpu5() const { return FgpuArchVersion >= Fgpu5; }
  bool hasFgpu4_32() const { return HasFgpu4_32; }
  bool hasFgpu4_32r2() const { return HasFgpu4_32r2; }
  bool hasFgpu32() const {
    return (FgpuArchVersion >= Fgpu32 && FgpuArchVersion < Fgpu32Max) ||
           hasFgpu64();
  }
  bool hasFgpu32r2() const {
    return (FgpuArchVersion >= Fgpu32r2 && FgpuArchVersion < Fgpu32Max) ||
           hasFgpu64r2();
  }
  bool hasFgpu32r3() const {
    return (FgpuArchVersion >= Fgpu32r3 && FgpuArchVersion < Fgpu32Max) ||
           hasFgpu64r2();
  }
  bool hasFgpu32r5() const {
    return (FgpuArchVersion >= Fgpu32r5 && FgpuArchVersion < Fgpu32Max) ||
           hasFgpu64r5();
  }
  bool hasFgpu32r6() const {
    return (FgpuArchVersion >= Fgpu32r6 && FgpuArchVersion < Fgpu32Max) ||
           hasFgpu64r6();
  }
  bool hasFgpu64() const { return FgpuArchVersion >= Fgpu64; }
  bool hasFgpu64r2() const { return FgpuArchVersion >= Fgpu64r2; }
  bool hasFgpu64r3() const { return FgpuArchVersion >= Fgpu64r3; }
  bool hasFgpu64r5() const { return FgpuArchVersion >= Fgpu64r5; }
  bool hasFgpu64r6() const { return FgpuArchVersion >= Fgpu64r6; }

  bool hasCnFgpu() const { return HasCnFgpu; }
  bool hasCnFgpuP() const { return HasCnFgpuP; }

  bool isLittle() const { return IsLittle; }
  bool isABICalls() const { return !NoABICalls; }
  bool isFPXX() const { return IsFPXX; }
  bool isFP64bit() const { return IsFP64bit; }
  bool useOddSPReg() const { return UseOddSPReg; }
  bool noOddSPReg() const { return !UseOddSPReg; }
  bool isNaN2008() const { return IsNaN2008bit; }
  bool inAbs2008Mode() const { return Abs2008; }
  bool isGP64bit() const { return IsGP64bit; }
  bool isGP32bit() const { return !IsGP64bit; }
  unsigned getGPRSizeInBytes() const { return isGP64bit() ? 8 : 4; }
  bool isPTR64bit() const { return IsPTR64bit; }
  bool isPTR32bit() const { return !IsPTR64bit; }
  bool hasSym32() const {
    return (HasSym32 && isABI_N64()) || isABI_N32() || isABI_O32();
  }
  bool isSingleFloat() const { return IsSingleFloat; }
  bool isTargetELF() const { return TargetTriple.isOSBinFormatELF(); }
  bool hasVFPU() const { return HasVFPU; }
  bool inFgpu16Mode() const { return InFgpu16Mode; }
  bool inFgpu16ModeDefault() const {
    return InFgpu16Mode;
  }
  // Hard float for Fgpu16 means essentially to compile as soft float
  // but to use a runtime library for soft float that is written with
  // native Fgpu32 floating point instructions (those runtime routines
  // run in Fgpu32 hard float mode).
  bool inFgpu16HardFloat() const {
    return inFgpu16Mode() && InFgpu16HardFloat;
  }
  bool inMicroFgpuMode() const { return InMicroFgpuMode && !InFgpu16Mode; }
  bool inMicroFgpu32r6Mode() const {
    return inMicroFgpuMode() && hasFgpu32r6();
  }
  bool hasDSP() const { return HasDSP; }
  bool hasDSPR2() const { return HasDSPR2; }
  bool hasDSPR3() const { return HasDSPR3; }
  bool has3D() const { return Has3D; }
  bool hasMSA() const { return HasMSA; }
  bool disableMadd4() const { return DisableMadd4; }
  bool hasEVA() const { return HasEVA; }
  bool hasMT() const { return HasMT; }
  bool hasCRC() const { return HasCRC; }
  bool hasVirt() const { return HasVirt; }
  bool hasGINV() const { return HasGINV; }
  bool useIndirectJumpsHazard() const {
    return UseIndirectJumpsHazard && hasFgpu32r2();
  }
  bool useSmallSection() const { return UseSmallSection; }

  bool hasStandardEncoding() const { return !InFgpu16Mode && !InMicroFgpuMode; }

  bool useSoftFloat() const { return IsSoftFloat; }

  bool useLongCalls() const { return UseLongCalls; }

  bool useXGOT() const { return UseXGOT; }

  bool enableLongBranchPass() const {
    return hasStandardEncoding() || inMicroFgpuMode() || allowMixed16_32();
  }

  /// Features related to the presence of specific instructions.
  bool hasExtractInsert() const { return !inFgpu16Mode() && hasFgpu32r2(); }
  bool hasMTHC1() const { return hasFgpu32r2(); }

  bool allowMixed16_32() const { return inFgpu16ModeDefault() |
                                        AllowMixed16_32; }

  bool os16() const { return Os16; }

  bool isTargetNaCl() const { return TargetTriple.isOSNaCl(); }

  bool isXRaySupported() const override { return true; }

  // for now constant islands are on for the whole compilation unit but we only
  // really use them if in addition we are in Fgpu16 mode
  static bool useConstantIslands();

  Align getStackAlignment() const { return stackAlignment; }

  // Grab relocation model
  Reloc::Model getRelocationModel() const;

  FgpuSubtarget &initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                                 const TargetMachine &TM);

  /// Does the system support unaligned memory access.
  ///
  /// Fgpu32r6/Fgpu64r6 require full unaligned access support but does not
  /// specify which component of the system provides it. Hardware, software, and
  /// hybrid implementations are all valid.
  bool systemSupportsUnalignedAccess() const { return hasFgpu32r6(); }

  // Set helper classes
  void setHelperClassesFgpu16();
  void setHelperClassesFgpuSE();

  const SelectionDAGTargetInfo *getSelectionDAGInfo() const override {
    return &TSInfo;
  }
  const FgpuInstrInfo *getInstrInfo() const override { return InstrInfo.get(); }
  const TargetFrameLowering *getFrameLowering() const override {
    return FrameLowering.get();
  }
  const FgpuRegisterInfo *getRegisterInfo() const override {
    return &InstrInfo->getRegisterInfo();
  }
  const FgpuTargetLowering *getTargetLowering() const override {
    return TLInfo.get();
  }
  const InstrItineraryData *getInstrItineraryData() const override {
    return &InstrItins;
  }

protected:
  // GlobalISel related APIs.
  std::unique_ptr<CallLowering> CallLoweringInfo;
  std::unique_ptr<LegalizerInfo> Legalizer;
  std::unique_ptr<RegisterBankInfo> RegBankInfo;
  std::unique_ptr<InstructionSelector> InstSelector;

public:
  const CallLowering *getCallLowering() const override;
  const LegalizerInfo *getLegalizerInfo() const override;
  const RegisterBankInfo *getRegBankInfo() const override;
  InstructionSelector *getInstructionSelector() const override;
};
} // End llvm namespace

#endif
