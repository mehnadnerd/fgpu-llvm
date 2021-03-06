//===- FgpuOptionRecord.h - Abstraction for storing information -*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// FgpuOptionRecord - Abstraction for storing arbitrary information in
// ELF files. Arbitrary information (e.g. register usage) can be stored in Fgpu
// specific ELF sections like .Fgpu.options. Specific records should subclass
// FgpuOptionRecord and provide an implementation to EmitFgpuOptionRecord which
// basically just dumps the information into an ELF section. More information
// about .Fgpu.option can be found in the SysV ABI and the 64-bit ELF Object
// specification.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_FGPU_FGPUOPTIONRECORD_H
#define LLVM_LIB_TARGET_FGPU_FGPUOPTIONRECORD_H

#include "MCTargetDesc/FgpuMCTargetDesc.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCRegisterInfo.h"
#include <cstdint>

namespace llvm {

class FgpuELFStreamer;

class FgpuOptionRecord {
public:
  virtual ~FgpuOptionRecord() = default;

  virtual void EmitFgpuOptionRecord() = 0;
};

class FgpuRegInfoRecord : public FgpuOptionRecord {
public:
  FgpuRegInfoRecord(FgpuELFStreamer *S, MCContext &Context)
      : Streamer(S), Context(Context) {
    ri_gprmask = 0;
    ri_cprmask[0] = ri_cprmask[1] = ri_cprmask[2] = ri_cprmask[3] = 0;
    ri_gp_value = 0;

    const MCRegisterInfo *TRI = Context.getRegisterInfo();
    GPRRegClass = &(TRI->getRegClass(Fgpu::GPROutRegClassID));
    VFPRegClass = &(TRI->getRegClass(Fgpu::VecRegsRegClassID));
  }

  ~FgpuRegInfoRecord() override = default;

  void EmitFgpuOptionRecord() override;
  void SetPhysRegUsed(unsigned Reg, const MCRegisterInfo *MCRegInfo);

private:
  FgpuELFStreamer *Streamer;
  MCContext &Context;
  const MCRegisterClass *GPRRegClass;
  const MCRegisterClass *VFPRegClass;
  uint32_t ri_gprmask;
  uint32_t ri_cprmask[4];
  int64_t ri_gp_value;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_FGPU_FGPUOPTIONRECORD_H
