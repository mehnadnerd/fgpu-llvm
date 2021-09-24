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
  enum class FpABIKind { ANY };

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
      ISALevel = 32;
      ISARevision = 1;
  }

  template <class PredicateLibrary>
  void setGPRSizeFromPredicates(const PredicateLibrary &P) {
    GPRSize = Fgpu::AFL_REG_32;
  }

  template <class PredicateLibrary>
  void setCPR1SizeFromPredicates(const PredicateLibrary &P) {
    CPR1Size = Fgpu::AFL_REG_32;
  }

  template <class PredicateLibrary>
  void setISAExtensionFromPredicates(const PredicateLibrary &P) {
    ISAExtension = Fgpu::AFL_EXT_NONE;
  }

  template <class PredicateLibrary>
  void setASESetFromPredicates(const PredicateLibrary &P) {
    ASESet = 0;
//    if (P.hasGINV())
//      ASESet |= Fgpu::AFL_ASE_GINV;
  }

  template <class PredicateLibrary>
  void setFpAbiFromPredicates(const PredicateLibrary &P) {
    Is32BitABI = true;

    FpABI = FpABIKind::ANY;
  }

  template <class PredicateLibrary>
  void setAllFromPredicates(const PredicateLibrary &P) {
    setISALevelAndRevisionFromPredicates(P);
    setGPRSizeFromPredicates(P);
    setCPR1SizeFromPredicates(P);
    setISAExtensionFromPredicates(P);
    setASESetFromPredicates(P);
    setFpAbiFromPredicates(P);
    OddSPReg = false;
  }
};

MCStreamer &operator<<(MCStreamer &OS, FgpuABIFlagsSection &ABIFlagsSection);

} // end namespace llvm

#endif // LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUABIFLAGSSECTION_H
