//===- FgpuRegisterInfo.cpp - FGPU Register Information -------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the FGPU implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "FgpuRegisterInfo.h"
#include "MCTargetDesc/FgpuABIInfo.h"
#include "Fgpu.h"
#include "FgpuMachineFunction.h"
#include "FgpuSubtarget.h"
#include "FgpuTargetMachine.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <cstdint>

using namespace llvm;

#define DEBUG_TYPE "fgpu-reg-info"

#define GET_REGINFO_TARGET_DESC
#include "FgpuGenRegisterInfo.inc"

FgpuRegisterInfo::FgpuRegisterInfo() : FgpuGenRegisterInfo(Fgpu::LR) {}

unsigned FgpuRegisterInfo::getPICCallReg() { return Fgpu::R25; }

const TargetRegisterClass *
FgpuRegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                     unsigned Kind) const {
  FgpuABIInfo ABI = MF.getSubtarget<FgpuSubtarget>().getABI();
  FgpuPtrClass PtrClassKind = static_cast<FgpuPtrClass>(Kind);
  return &Fgpu::FgpuGPRRegClass; // only kind on this system
  llvm_unreachable("Unknown pointer kind");
}

unsigned
FgpuRegisterInfo::getRegPressureLimit(const TargetRegisterClass *RC,
                                      MachineFunction &MF) const {
  switch (RC->getID()) {
  default:
    return 0;
  case Fgpu::FgpuGPRRegClassID: {
    return 27; // TODO: don't make up
  }
  case Fgpu::FgpuFPRRegClassID: {
    return 16;
  }
  }
}

//===----------------------------------------------------------------------===//
// Callee Saved Registers methods
//===----------------------------------------------------------------------===//

/// Fgpu Callee Saved Registers
const MCPhysReg *
FgpuRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  return Fgpu::CSR_CC_Fgpu;
}

const uint32_t *
FgpuRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                       CallingConv::ID) const {
  return CSR_CC_Fgpu_RegMask;
}

BitVector FgpuRegisterInfo::
getReservedRegs(const MachineFunction &MF) const {
  static const MCPhysReg ReservedCPURegs[] = {
      Fgpu::ZERO, Fgpu::SP, Fgpu::LR
  };
  BitVector Reserved(getNumRegs());
  const FgpuSubtarget &Subtarget = MF.getSubtarget<FgpuSubtarget>();

  for (unsigned I = 0; I < array_lengthof(ReservedCPURegs); ++I)
    Reserved.set(ReservedCPURegs[I]);



  // Reserve FP if this function should have a dedicated frame pointer register.
  if (Subtarget.getFrameLowering()->hasFP(MF)) {
    Reserved.set(Fgpu::FP);

    // Reserve the base register if we need to both realign the stack and
    // allocate variable-sized objects at runtime. This should test the
    // same conditions as FgpuFrameLowering::hasBP().
    if (hasStackRealignment(MF) && MF.getFrameInfo().hasVarSizedObjects()) {
      Reserved.set(Fgpu::S7); // Base PTR
    }
  }

  // Reserve GP if small section is used.
  if (Subtarget.useSmallSection()) {
    Reserved.set(Fgpu::GP);
  }

  return Reserved;
}

bool
FgpuRegisterInfo::requiresRegisterScavenging(const MachineFunction &MF) const {
  return true;
}

// FrameIndex represent objects inside a abstract stack.
// We must replace FrameIndex with an stack/frame pointer
// direct reference.
void FgpuRegisterInfo::
eliminateFrameIndex(MachineBasicBlock::iterator II, int SPAdj,
                    unsigned FIOperandNum, RegScavenger *RS) const {
  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();

  LLVM_DEBUG(errs() << "\nFunction : " << MF.getName() << "\n";
             errs() << "<--------->\n"
                    << MI);

  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  uint64_t stackSize = MF.getFrameInfo().getStackSize();
  int64_t spOffset = MF.getFrameInfo().getObjectOffset(FrameIndex);

  LLVM_DEBUG(errs() << "FrameIndex : " << FrameIndex << "\n"
                    << "spOffset   : " << spOffset << "\n"
                    << "stackSize  : " << stackSize << "\n"
                    << "alignment  : "
                    << DebugStr(MF.getFrameInfo().getObjectAlign(FrameIndex))
                    << "\n");

  eliminateFI(MI, FIOperandNum, FrameIndex, stackSize, spOffset);
}

Register FgpuRegisterInfo::
getFrameRegister(const MachineFunction &MF) const {
  const FgpuSubtarget &Subtarget = MF.getSubtarget<FgpuSubtarget>();
  const TargetFrameLowering *TFI = Subtarget.getFrameLowering();

  return TFI->hasFP(MF) ? (Fgpu::FP) :
                          (Fgpu::SP);
}

bool FgpuRegisterInfo::canRealignStack(const MachineFunction &MF) const {
  // Avoid realigning functions that explicitly do not want to be realigned.
  // Normally, we should report an error when a function should be dynamically
  // realigned but also has the attribute no-realign-stack. Unfortunately,
  // with this attribute, MachineFrameInfo clamps each new object's alignment
  // to that of the stack's alignment as specified by the ABI. As a result,
  // the information of whether we have objects with larger alignment
  // requirement than the stack's alignment is already lost at this point.
  if (!TargetRegisterInfo::canRealignStack(MF))
    return false;

  const FgpuSubtarget &Subtarget = MF.getSubtarget<FgpuSubtarget>();
  unsigned FP = Fgpu::FP;
  unsigned BP = Fgpu::S7;

  // We can't perform dynamic stack realignment if we can't reserve the
  // frame pointer register.
  if (!MF.getRegInfo().canReserveReg(FP))
    return false;

  // We can realign the stack if we know the maximum call frame size and we
  // don't have variable sized objects.
  if (Subtarget.getFrameLowering()->hasReservedCallFrame(MF))
    return true;

  // We have to reserve the base pointer register in the presence of variable
  // sized objects.
  return MF.getRegInfo().canReserveReg(BP);
}
