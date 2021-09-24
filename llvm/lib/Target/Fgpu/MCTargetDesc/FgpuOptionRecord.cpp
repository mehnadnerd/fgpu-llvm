//===- FgpuOptionRecord.cpp - Abstraction for storing information ---------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "FgpuOptionRecord.h"
#include "FgpuABIInfo.h"
#include "FgpuELFStreamer.h"
#include "FgpuTargetStreamer.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSectionELF.h"
#include <cassert>

using namespace llvm;

void FgpuRegInfoRecord::EmitFgpuOptionRecord() {
  MCAssembler &MCA = Streamer->getAssembler();
  FgpuTargetStreamer *MTS =
      static_cast<FgpuTargetStreamer *>(Streamer->getTargetStreamer());

  Streamer->PushSection();
  {
    MCSectionELF *Sec = Context.getELFSection(".reginfo", ELF::SHT_MIPS_REGINFO,
                                              ELF::SHF_ALLOC, 24);
    MCA.registerSection(*Sec);
    Sec->setAlignment(Align(8));
    Streamer->SwitchSection(Sec);

    Streamer->emitInt32(ri_gprmask);
    Streamer->emitInt32(ri_cprmask[0]);
    Streamer->emitInt32(ri_cprmask[1]);
    Streamer->emitInt32(ri_cprmask[2]);
    Streamer->emitInt32(ri_cprmask[3]);
    assert((ri_gp_value & 0xffffffff) == ri_gp_value);
    Streamer->emitInt32(ri_gp_value);
  }

  Streamer->PopSection();
}

void FgpuRegInfoRecord::SetPhysRegUsed(unsigned Reg,
                                       const MCRegisterInfo *MCRegInfo) {
  unsigned Value = 0;

  for (const MCPhysReg &SubReg : MCRegInfo->subregs_inclusive(Reg)) {
    unsigned EncVal = MCRegInfo->getEncodingValue(SubReg);
    Value |= 1 << EncVal;

    if (GPRRegClass->contains(SubReg))
      ri_gprmask |= Value;
    // FGPU COP1 is the FPU.
    else if (VFPRegClass->contains(SubReg))
      ri_cprmask[1] |= Value;
  }
}
