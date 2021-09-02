//===-- FgpuFrameLowering.h - Define frame lowering for Fgpu ----*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
//
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Fgpu_FgpuFRAMELOWERING_H
#define LLVM_LIB_TARGET_Fgpu_FgpuFRAMELOWERING_H

#include "Fgpu.h"
#include "llvm/CodeGen/TargetFrameLowering.h"

namespace llvm {
  class FgpuSubtarget;

class FgpuFrameLowering : public TargetFrameLowering {
protected:
  const FgpuSubtarget &STI;

public:
  explicit FgpuFrameLowering(const FgpuSubtarget &sti, Align Alignment)
      : TargetFrameLowering(StackGrowsDown, Alignment, 0, Alignment), STI(sti) {
  }

  static const FgpuFrameLowering *create(const FgpuSubtarget &ST);

  bool hasFP(const MachineFunction &MF) const override;

  bool hasBP(const MachineFunction &MF) const;

  bool isFPCloseToIncomingSP() const override { return false; }

  bool enableShrinkWrapping(const MachineFunction &MF) const override {
    return true;
  }

  MachineBasicBlock::iterator
  eliminateCallFramePseudoInstr(MachineFunction &MF,
                                MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator I) const override;

protected:
  uint64_t estimateStackSize(const MachineFunction &MF) const;
};

/// Create FgpuFrameLowering objects.
const FgpuFrameLowering *createFgpu16FrameLowering(const FgpuSubtarget &ST);
const FgpuFrameLowering *createFgpuSEFrameLowering(const FgpuSubtarget &ST);

} // End llvm namespace

#endif
