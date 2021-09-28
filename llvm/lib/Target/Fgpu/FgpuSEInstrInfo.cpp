//===-- FgpuSEInstrInfo.cpp - Fgpu32/64 Instruction Information -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the Fgpu32/64 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "FgpuSEInstrInfo.h"
#include "MCTargetDesc/FgpuInstPrinter.h"
#include "FgpuAnalyzeImmediate.h"
#include "FgpuMachineFunction.h"
#include "FgpuTargetMachine.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

static unsigned getUnconditionalBranch(const FgpuSubtarget &STI) {
  return Fgpu::BNE; // TODO>: make unconditioanl
  //return STI.isPositionIndependent() ? Fgpu::B : Fgpu::J;
}

FgpuSEInstrInfo::FgpuSEInstrInfo(const FgpuSubtarget &STI)
    : FgpuInstrInfo(STI, getUnconditionalBranch(STI)), RI() {}

const FgpuRegisterInfo &FgpuSEInstrInfo::getRegisterInfo() const {
  return RI;
}

/// isLoadFromStackSlot - If the specified machine instruction is a direct
/// load from a stack slot, return the virtual or physical register number of
/// the destination along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than loading from the stack slot.
unsigned FgpuSEInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                              int &FrameIndex) const {
  unsigned Opc = MI.getOpcode();

  if ((Opc == Fgpu::LW)   || (Opc == Fgpu::LWC)) {
    if ((MI.getOperand(1).isFI()) &&  // is a stack slot
        (MI.getOperand(2).isImm()) && // the imm is zero
        (isZeroImm(MI.getOperand(2)))) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
  }

  return 0;
}

/// isStoreToStackSlot - If the specified machine instruction is a direct
/// store to a stack slot, return the virtual or physical register number of
/// the source reg along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than storing to the stack slot.
unsigned FgpuSEInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                             int &FrameIndex) const {
  unsigned Opc = MI.getOpcode();

  if ((Opc == Fgpu::SW)   || (Opc == Fgpu::SWC)) {
    if ((MI.getOperand(1).isFI()) &&  // is a stack slot
        (MI.getOperand(2).isImm()) && // the imm is zero
        (isZeroImm(MI.getOperand(2)))) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
  }
  return 0;
}

void FgpuSEInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I,
                                  const DebugLoc &DL, MCRegister DestReg,
                                  MCRegister SrcReg, bool KillSrc) const {
  unsigned Opc = 0, ZeroReg = 0;

  if (Fgpu::GPROutRegClass.contains(DestReg, SrcReg)) {
      Opc = Fgpu::OR;
  }
//  else if (Fgpu::VecRegsRegClass.contains(DestReg, SrcReg))
//    Opc = Fgpu::FMOV_S;

  assert(Opc && "Cannot copy registers");

  MachineInstrBuilder MIB = BuildMI(MBB, I, DL, get(Opc));

  if (DestReg)
    MIB.addReg(DestReg, RegState::Define);

  if (SrcReg)
    MIB.addReg(SrcReg, getKillRegState(KillSrc));

  if (ZeroReg)
    MIB.addReg(ZeroReg);
}

static bool isORCopyInst(const MachineInstr &MI) {
  switch (MI.getOpcode()) {
  default:
    break;
  case Fgpu::OR:
    if (MI.getOperand(2).getReg() == Fgpu::ZERO)
      return true;
    break;
  }
  return false;
}

/// We check for the common case of 'or', as it's FGPU' preferred instruction
/// for GPRs but we have to check the operands to ensure that is the case.
/// Other move instructions for FGPU are directly identifiable.
Optional<DestSourcePair>
FgpuSEInstrInfo::isCopyInstrImpl(const MachineInstr &MI) const {
  bool isDSPControlWrite = false;
  // Condition is made to match the creation of WRDSP/RDDSP copy instruction
  // from copyPhysReg function.
  if (MI.isMoveReg() || isORCopyInst(MI)) {
    return DestSourcePair{MI.getOperand(0), MI.getOperand(1)};
  }
  return None;
}

void FgpuSEInstrInfo::
storeRegToStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                Register SrcReg, bool isKill, int FI,
                const TargetRegisterClass *RC, const TargetRegisterInfo *TRI,
                int64_t Offset) const {
  DebugLoc DL;
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOStore);

  unsigned Opc = 0;

  if (Fgpu::GPROutRegClass.hasSubClassEq(RC))
    Opc = Fgpu::SW;
  else if (Fgpu::VecRegsRegClass.hasSubClassEq(RC))
    Opc = Fgpu::SWC;

  assert(Opc && "Register class not handled!");
  BuildMI(MBB, I, DL, get(Opc)).addReg(SrcReg, getKillRegState(isKill))
    .addFrameIndex(FI).addImm(Offset).addMemOperand(MMO);
}

void FgpuSEInstrInfo::
loadRegFromStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                 Register DestReg, int FI, const TargetRegisterClass *RC,
                 const TargetRegisterInfo *TRI, int64_t Offset) const {
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOLoad);
  unsigned Opc = 0;

  const Function &Func = MBB.getParent()->getFunction();

  if (Fgpu::GPROutRegClass.hasSubClassEq(RC))
    Opc = Fgpu::LW;
  else if (Fgpu::VecRegsRegClass.hasSubClassEq(RC))
    Opc = Fgpu::LWC;

  assert(Opc && "Register class not handled!");

    BuildMI(MBB, I, DL, get(Opc), DestReg)
        .addFrameIndex(FI)
        .addImm(Offset)
        .addMemOperand(MMO);
}

bool FgpuSEInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineBasicBlock &MBB = *MI.getParent();
  // DEBUG(errs() << "soubhi: entering expandPostRAPseudo\n");

  switch(MI.getDesc().getOpcode()) {
  default:
    return false;
  case Fgpu::RetLR:
    expandRetLR(MBB, MI, Fgpu::RET);
    break;
  case Fgpu::Li32:
    ExpandLi32(MBB, MI);
    break;
  }

  MBB.erase(MI);
  return true;
}

/// Adjust SP by Amount bytes.
void FgpuSEInstrInfo::adjustStackPtr(unsigned SP, int64_t Amount,
                                     MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I) const {
  FgpuABIInfo ABI = Subtarget.getABI();
  DebugLoc DL;
  unsigned ADDiu = ABI.GetPtrAddiuOp();

  if (Amount == 0)
    return;

  if (isInt<14>(Amount)) {
    // addi sp, sp, amount
    BuildMI(MBB, I, DL, get(Fgpu::ADDi), SP).addReg(SP).addImm(Amount);
  } else {
    // For numbers which are not 16bit integers we synthesize Amount inline
    // then add or subtract it from sp.
    unsigned Opc = ABI.GetPtrAdduOp();
    if (Amount < 0) {
      Opc = ABI.GetPtrSubuOp();
      Amount = -Amount;
    }
    unsigned Reg = loadImmediate(Amount, MBB, I, DL, nullptr);
    BuildMI(MBB, I, DL, get(Opc), SP).addReg(SP).addReg(Reg, RegState::Kill);
  }
}

/// This function generates the sequence of instructions needed to get the
/// result of adding register REG and immediate IMM.
unsigned FgpuSEInstrInfo::loadImmediate(int64_t Imm, MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator II,
                                        const DebugLoc &DL,
                                        unsigned *NewImm) const {
  FgpuAnalyzeImmediate AnalyzeImm;
  const FgpuSubtarget &STI = Subtarget;
  MachineRegisterInfo &RegInfo = MBB.getParent()->getRegInfo();
  unsigned Size = 32;
  unsigned LUi = Fgpu::LUi;
  unsigned ZEROReg = Fgpu::ZERO;
  const TargetRegisterClass *RC = &Fgpu::GPROutRegClass;
  bool LastInstrIsADDiu = NewImm;

  const FgpuAnalyzeImmediate::InstSeq &Seq =
    AnalyzeImm.Analyze(Imm, Size, LastInstrIsADDiu);
  FgpuAnalyzeImmediate::InstSeq::const_iterator Inst = Seq.begin();

  assert(Seq.size() && (!LastInstrIsADDiu || (Seq.size() > 1)));

  // The first instruction can be a LUi, which is different from other
  // instructions (ADDiu, ORI and SLL) in that it does not have a register
  // operand.
  Register Reg = RegInfo.createVirtualRegister(RC);

  if (Inst->Opc == LUi)
    BuildMI(MBB, II, DL, get(LUi), Reg).addImm(SignExtend64<16>(Inst->ImmOpnd));
  else
    BuildMI(MBB, II, DL, get(Inst->Opc), Reg).addReg(ZEROReg)
      .addImm(SignExtend64<16>(Inst->ImmOpnd));

  // Build the remaining instructions in Seq.
  for (++Inst; Inst != Seq.end() - LastInstrIsADDiu; ++Inst)
    BuildMI(MBB, II, DL, get(Inst->Opc), Reg).addReg(Reg, RegState::Kill)
      .addImm(SignExtend64<16>(Inst->ImmOpnd));

  if (LastInstrIsADDiu)
    *NewImm = Inst->ImmOpnd;

  return Reg;
}

void FgpuSEInstrInfo::expandRetLR(MachineBasicBlock &MBB,
                                  MachineInstr &I,
                                  unsigned Opc) const {
  BuildMI(MBB, I, I.getDebugLoc(), get(Opc));
}

void FgpuSEInstrInfo::ExpandLi32(MachineBasicBlock &MBB, MachineInstr &I) const {
  unsigned DstReg = I.getOperand(0).getReg();
  const MachineOperand &MO = I.getOperand(1);
  unsigned ImmVal = (unsigned)MO.getImm();
  BuildMI(MBB, I, I.getDebugLoc(), get(Fgpu::Li), DstReg).addImm(ImmVal);
  BuildMI(MBB, I, I.getDebugLoc(), get(Fgpu::LUi), DstReg).addImm(ImmVal>>16);
}

std::pair<bool, bool>
FgpuSEInstrInfo::compareOpndSize(unsigned Opc,
                                 const MachineFunction &MF) const {
  const MCInstrDesc &Desc = get(Opc);
  assert(Desc.NumOperands == 2 && "Unary instruction expected.");
  const FgpuRegisterInfo *RI = &getRegisterInfo();
  unsigned DstRegSize = RI->getRegSizeInBits(*getRegClass(Desc, 0, RI, MF));
  unsigned SrcRegSize = RI->getRegSizeInBits(*getRegClass(Desc, 1, RI, MF));

  return std::make_pair(DstRegSize > SrcRegSize, DstRegSize < SrcRegSize);
}

const FgpuInstrInfo *llvm::createFgpuSEInstrInfo(const FgpuSubtarget &STI) {
  return new FgpuSEInstrInfo(STI);
}
