//===- FgpuABIFlagsSection.cpp - Fgpu ELF ABI Flags Section ---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/FgpuABIFlagsSection.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FgpuABIFlags.h"

using namespace llvm;

uint8_t FgpuABIFlagsSection::getFpABIValue() {
  switch (FpABI) {
  case FpABIKind::ANY:
    return Fgpu::Val_GNU_FGPU_ABI_FP_ANY;
  }

  llvm_unreachable("unexpected fp abi value");
}

StringRef FgpuABIFlagsSection::getFpABIString(FpABIKind Value) {
  switch (Value) {
  default:
    llvm_unreachable("unsupported fp abi value");
  }
}

uint8_t FgpuABIFlagsSection::getCPR1SizeValue() {
  return (uint8_t)CPR1Size;
}

namespace llvm {

MCStreamer &operator<<(MCStreamer &OS, FgpuABIFlagsSection &ABIFlagsSection) {
  // Write out a Elf_Internal_ABIFlags_v0 struct
  OS.emitIntValue(ABIFlagsSection.getVersionValue(), 2);      // version
  OS.emitIntValue(ABIFlagsSection.getISALevelValue(), 1);     // isa_level
  OS.emitIntValue(ABIFlagsSection.getISARevisionValue(), 1);  // isa_rev
  OS.emitIntValue(ABIFlagsSection.getGPRSizeValue(), 1);      // gpr_size
  OS.emitIntValue(ABIFlagsSection.getCPR1SizeValue(), 1);     // cpr1_size
  OS.emitIntValue(ABIFlagsSection.getCPR2SizeValue(), 1);     // cpr2_size
  OS.emitIntValue(ABIFlagsSection.getFpABIValue(), 1);        // fp_abi
  OS.emitIntValue(ABIFlagsSection.getISAExtensionValue(), 4); // isa_ext
  OS.emitIntValue(ABIFlagsSection.getASESetValue(), 4);       // ases
  OS.emitIntValue(ABIFlagsSection.getFlags1Value(), 4);       // flags1
  OS.emitIntValue(ABIFlagsSection.getFlags2Value(), 4);       // flags2
  return OS;
}

} // end namespace llvm
