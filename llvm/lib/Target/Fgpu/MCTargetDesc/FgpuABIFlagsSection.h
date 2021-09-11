//===- FgpuABIFlagsSection.h - Fgpu ELF ABI Flags Section -------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUABIFLAGSSECTION_H
#define LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUABIFLAGSSECTION_H

#include "llvm/ADT/StringRef.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FgpuABIFlags.h"
#include <cstdint>

namespace llvm {

class MCStreamer;

struct FgpuABIFlagsSection {
  // Internal representation of the fp_abi related values used in .module.
  enum class FpABIKind { ANY, XX, S32, S64, SOFT };

  // Version of flags structure.
  uint16_t Version = 0;
  // The level of the ISA: 1-5, 32, 64.
  uint8_t ISALevel = 0;
  // The revision of ISA: 0 for FGPU V and below, 1-n otherwise.
  uint8_t ISARevision = 0;
  // The size of general purpose registers.
  Fgpu::AFL_REG GPRSize = Fgpu::AFL_REG_NONE;
  // The size of co-processor 1 registers.
  Fgpu::AFL_REG CPR1Size = Fgpu::AFL_REG_NONE;
  // The size of co-processor 2 registers.
  Fgpu::AFL_REG CPR2Size = Fgpu::AFL_REG_NONE;
  // Processor-specific extension.
  Fgpu::AFL_EXT ISAExtension = Fgpu::AFL_EXT_NONE;
  // Mask of ASEs used.
  uint32_t ASESet = 0;

  bool OddSPReg = false;

  bool Is32BitABI = false;

protected:
  // The floating-point ABI.
  FpABIKind FpABI = FpABIKind::ANY;

public:
  FgpuABIFlagsSection() = default;

  uint16_t getVersionValue() { return (uint16_t)Version; }
  uint8_t getISALevelValue() { return (uint8_t)ISALevel; }
  uint8_t getISARevisionValue() { return (uint8_t)ISARevision; }
  uint8_t getGPRSizeValue() { return (uint8_t)GPRSize; }
  uint8_t getCPR1SizeValue();
  uint8_t getCPR2SizeValue() { return (uint8_t)CPR2Size; }
  uint8_t getFpABIValue();
  uint32_t getISAExtensionValue() { return (uint32_t)ISAExtension; }
  uint32_t getASESetValue() { return (uint32_t)ASESet; }

  uint32_t getFlags1Value() {
    uint32_t Value = 0;

    if (OddSPReg)
      Value |= (uint32_t)Fgpu::AFL_FLAGS1_ODDSPREG;

    return Value;
  }

  uint32_t getFlags2Value() { return 0; }

  FpABIKind getFpABI() { return FpABI; }
  void setFpABI(FpABIKind Value, bool IsABI32Bit) {
    FpABI = Value;
    Is32BitABI = IsABI32Bit;
  }

  StringRef getFpABIString(FpABIKind Value);

  template <class PredicateLibrary>
  void setISALevelAndRevisionFromPredicates(const PredicateLibrary &P) {
    if (P.hasFgpu64()) {
      ISALevel = 64;
      if (P.hasFgpu64r6())
        ISARevision = 6;
      else if (P.hasFgpu64r5())
        ISARevision = 5;
      else if (P.hasFgpu64r3())
        ISARevision = 3;
      else if (P.hasFgpu64r2())
        ISARevision = 2;
      else
        ISARevision = 1;
    } else if (P.hasFgpu32()) {
      ISALevel = 32;
      if (P.hasFgpu32r6())
        ISARevision = 6;
      else if (P.hasFgpu32r5())
        ISARevision = 5;
      else if (P.hasFgpu32r3())
        ISARevision = 3;
      else if (P.hasFgpu32r2())
        ISARevision = 2;
      else
        ISARevision = 1;
    } else {
      ISARevision = 0;
      if (P.hasFgpu5())
        ISALevel = 5;
      else if (P.hasFgpu4())
        ISALevel = 4;
      else if (P.hasFgpu3())
        ISALevel = 3;
      else if (P.hasFgpu2())
        ISALevel = 2;
      else if (P.hasFgpu1())
        ISALevel = 1;
      else
        llvm_unreachable("Unknown ISA level!");
    }
  }

  template <class PredicateLibrary>
  void setGPRSizeFromPredicates(const PredicateLibrary &P) {
    GPRSize = P.isGP64bit() ? Fgpu::AFL_REG_64 : Fgpu::AFL_REG_32;
  }

  template <class PredicateLibrary>
  void setCPR1SizeFromPredicates(const PredicateLibrary &P) {
    if (P.useSoftFloat())
      CPR1Size = Fgpu::AFL_REG_NONE;
    else if (P.hasMSA())
      CPR1Size = Fgpu::AFL_REG_128;
    else
      CPR1Size = P.isFP64bit() ? Fgpu::AFL_REG_64 : Fgpu::AFL_REG_32;
  }

  template <class PredicateLibrary>
  void setISAExtensionFromPredicates(const PredicateLibrary &P) {
    if (P.hasCnFgpuP())
      ISAExtension = Fgpu::AFL_EXT_OCTEONP;
    else if (P.hasCnFgpu())
      ISAExtension = Fgpu::AFL_EXT_OCTEON;
    else
      ISAExtension = Fgpu::AFL_EXT_NONE;
  }

  template <class PredicateLibrary>
  void setASESetFromPredicates(const PredicateLibrary &P) {
    ASESet = 0;
    if (P.hasDSP())
      ASESet |= Fgpu::AFL_ASE_DSP;
    if (P.hasDSPR2())
      ASESet |= Fgpu::AFL_ASE_DSPR2;
    if (P.hasMSA())
      ASESet |= Fgpu::AFL_ASE_MSA;
    if (P.hasMT())
      ASESet |= Fgpu::AFL_ASE_MT;
    if (P.hasCRC())
      ASESet |= Fgpu::AFL_ASE_CRC;
    if (P.hasVirt())
      ASESet |= Fgpu::AFL_ASE_VIRT;
    if (P.hasGINV())
      ASESet |= Fgpu::AFL_ASE_GINV;
  }

  template <class PredicateLibrary>
  void setFpAbiFromPredicates(const PredicateLibrary &P) {
    Is32BitABI = P.isABI_O32();

    FpABI = FpABIKind::ANY;
    if (P.useSoftFloat())
      FpABI = FpABIKind::SOFT;
    else if (P.isABI_N32() || P.isABI_N64())
      FpABI = FpABIKind::S64;
    else if (P.isABI_O32()) {
      if (P.isABI_FPXX())
        FpABI = FpABIKind::XX;
      else if (P.isFP64bit())
        FpABI = FpABIKind::S64;
      else
        FpABI = FpABIKind::S32;
    }
  }

  template <class PredicateLibrary>
  void setAllFromPredicates(const PredicateLibrary &P) {
    setISALevelAndRevisionFromPredicates(P);
    setGPRSizeFromPredicates(P);
    setCPR1SizeFromPredicates(P);
    setISAExtensionFromPredicates(P);
    setASESetFromPredicates(P);
    setFpAbiFromPredicates(P);
    OddSPReg = P.useOddSPReg();
  }
};

MCStreamer &operator<<(MCStreamer &OS, FgpuABIFlagsSection &ABIFlagsSection);

} // end namespace llvm

#endif // LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUABIFLAGSSECTION_H
