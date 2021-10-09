//===-- FgpuExpandPseudoInsts.cpp - Expand pseudo instructions ------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a pass that expands pseudo instructions into target
// instructions to allow proper scheduling, if-conversion, and other late
// optimizations. This pass should be run after register allocation but before
// the post-regalloc scheduling pass.
//
// This is currently only used for expanding atomic pseudos after register
// allocation. We do this to avoid the fast register allocator introducing
// spills between ll and sc. These stores cause some FGPU implementations to
// abort the atomic RMW sequence.
//
//===----------------------------------------------------------------------===//

#include "Fgpu.h"
#include "FgpuInstrInfo.h"
#include "FgpuSubtarget.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"

using namespace llvm;

#define DEBUG_TYPE "fgpu-pseudo"

namespace {
  class FgpuExpandPseudo : public MachineFunctionPass {
  public:
    static char ID;
    FgpuExpandPseudo() : MachineFunctionPass(ID) {}

    const FgpuInstrInfo *TII;
    const FgpuSubtarget *STI;

    bool runOnMachineFunction(MachineFunction &Fn) override;

    MachineFunctionProperties getRequiredProperties() const override {
      return MachineFunctionProperties().set(
          MachineFunctionProperties::Property::NoVRegs);
    }

    StringRef getPassName() const override {
      return "Fgpu pseudo instruction expansion pass";
    }

  private:
    bool expandMI(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
                  MachineBasicBlock::iterator &NMBB);
    bool expandMBB(MachineBasicBlock &MBB);
    bool expandB(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
                 MachineBasicBlock::iterator &NMBB);
   };
  char FgpuExpandPseudo::ID = 0;
}

bool FgpuExpandPseudo::expandB(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MBBI,
                                MachineBasicBlock::iterator &NMBB) {
  const MCInstrDesc &NewDesc = TII->get(Fgpu::BEQ);
  DebugLoc dl = MBB.findDebugLoc(MBBI);

  MachineInstrBuilder MIB = BuildMI(MBB, MBBI, dl, NewDesc);
  MIB.addReg(Fgpu::ZERO).addReg(Fgpu::ZERO);

  LLVM_DEBUG(dbgs() << "Num operands" << MBBI->getNumOperands());
  MBBI->getOperand(0).dump();
  MIB->addOperand(MBBI->getOperand(0));
  MBBI->eraseFromParent();
}

bool FgpuExpandPseudo::expandMI(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MBBI,
                                MachineBasicBlock::iterator &NMBB) {

  bool Modified = false;
  LLVM_DEBUG(dbgs() << "Trying to expand MI with opcode " << MBBI->getOpcode() << "\n");

  switch (MBBI->getOpcode()) {
    case Fgpu::B:
      expandB(MBB, MBBI, NMBB);
      Modified = true;

      break;
  }

  return Modified;
}

bool FgpuExpandPseudo::expandMBB(MachineBasicBlock &MBB) {
  bool Modified = false;

  MachineBasicBlock::iterator MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    MachineBasicBlock::iterator NMBBI = std::next(MBBI);
    Modified |= expandMI(MBB, MBBI, NMBBI);
    MBBI = NMBBI;
  }

  return Modified;
}

bool FgpuExpandPseudo::runOnMachineFunction(MachineFunction &MF) {
  STI = &static_cast<const FgpuSubtarget &>(MF.getSubtarget());
  TII = STI->getInstrInfo();

  bool Modified = false;
  for (MachineFunction::iterator MFI = MF.begin(), E = MF.end(); MFI != E;
       ++MFI)
    Modified |= expandMBB(*MFI);

  if (Modified)
    MF.RenumberBlocks();

  return Modified;
}

/// createFgpuExpandPseudoPass - returns an instance of the pseudo instruction
/// expansion pass.
FunctionPass *llvm::createFgpuExpandPseudoPass() {
  return new FgpuExpandPseudo();
}
