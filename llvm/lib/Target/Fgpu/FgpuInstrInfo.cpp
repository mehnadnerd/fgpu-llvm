//===- FgpuInstrInfo.cpp - Fgpu Instruction Information -------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the Fgpu implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "FgpuInstrInfo.h"
#include "MCTargetDesc/FgpuBaseInfo.h"
#include "MCTargetDesc/FgpuMCTargetDesc.h"
#include "FgpuSubtarget.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/Target/TargetMachine.h"
#include <cassert>

using namespace llvm;

#define GET_INSTRINFO_CTOR_DTOR
#include "FgpuGenInstrInfo.inc"

// Pin the vtable to this file.
void FgpuInstrInfo::anchor() {}

FgpuInstrInfo::FgpuInstrInfo(const FgpuSubtarget &STI, unsigned UncondBr)
    : FgpuGenInstrInfo(Fgpu::ADJCALLSTACKDOWN, Fgpu::ADJCALLSTACKUP),
      Subtarget(STI), UncondBrOpc(UncondBr) {}

const FgpuInstrInfo *FgpuInstrInfo::create(FgpuSubtarget &STI) {

  return createFgpuSEInstrInfo(STI);
}

bool FgpuInstrInfo::isZeroImm(const MachineOperand &op) const {
  return op.isImm() && op.getImm() == 0;
}

/// insertNoop - If data hazard condition is found insert the target nop
/// instruction.
// FIXME: This appears to be dead code.
void FgpuInstrInfo::
insertNoop(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI) const
{
  DebugLoc DL;
  BuildMI(MBB, MI, DL, get(Fgpu::NOP));
}

MachineMemOperand *
FgpuInstrInfo::GetMemOperand(MachineBasicBlock &MBB, int FI,
                             MachineMemOperand::Flags Flags) const {
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();

  return MF.getMachineMemOperand(MachinePointerInfo::getFixedStack(MF, FI),
                                 Flags, MFI.getObjectSize(FI),
                                 MFI.getObjectAlign(FI));
}

//===----------------------------------------------------------------------===//
// Branch Analysis
//===----------------------------------------------------------------------===//

void FgpuInstrInfo::AnalyzeCondBr(const MachineInstr *Inst, unsigned Opc,
                                  MachineBasicBlock *&BB,
                                  SmallVectorImpl<MachineOperand> &Cond) const {
  assert(getAnalyzableBrOpc(Opc) && "Not an analyzable branch");
  int NumOp = Inst->getNumExplicitOperands();

  // for both int and fp branches, the last explicit operand is the
  // MBB.
  BB = Inst->getOperand(NumOp-1).getMBB();
  Cond.push_back(MachineOperand::CreateImm(Opc));

  for (int i = 0; i < NumOp-1; i++)
    Cond.push_back(Inst->getOperand(i));
}

bool FgpuInstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                  MachineBasicBlock *&TBB,
                                  MachineBasicBlock *&FBB,
                                  SmallVectorImpl<MachineOperand> &Cond,
                                  bool AllowModify) const {
  SmallVector<MachineInstr*, 2> BranchInstrs;
  BranchType BT = analyzeBranch(MBB, TBB, FBB, Cond, AllowModify, BranchInstrs);

  return (BT == BT_None) || (BT == BT_Indirect);
}

void FgpuInstrInfo::BuildCondBr(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                                const DebugLoc &DL,
                                ArrayRef<MachineOperand> Cond) const {
  unsigned Opc = Cond[0].getImm();
  const MCInstrDesc &MCID = get(Opc);
  MachineInstrBuilder MIB = BuildMI(&MBB, DL, MCID);

  for (unsigned i = 1; i < Cond.size(); ++i) {
    assert((Cond[i].isImm() || Cond[i].isReg()) &&
           "Cannot copy operand for conditional branch!");
    MIB.add(Cond[i]);
  }
  MIB.addMBB(TBB);
}

unsigned FgpuInstrInfo::insertBranch(MachineBasicBlock &MBB,
                                     MachineBasicBlock *TBB,
                                     MachineBasicBlock *FBB,
                                     ArrayRef<MachineOperand> Cond,
                                     const DebugLoc &DL,
                                     int *BytesAdded) const {
  // Shouldn't be a fall through.
  assert(TBB && "insertBranch must not be told to insert a fallthrough");
  assert(!BytesAdded && "code size not handled");

  // # of condition operands:
  //  Unconditional branches: 0
  //  Floating point branches: 1 (opc)
  //  Int BranchZero: 2 (opc, reg)
  //  Int Branch: 3 (opc, reg0, reg1)
  assert((Cond.size() <= 3) &&
         "# of Fgpu branch conditions must be <= 3!");

  // Two-way Conditional branch.
  if (FBB) {
    BuildCondBr(MBB, TBB, DL, Cond);
    BuildMI(&MBB, DL, get(UncondBrOpc)).addMBB(FBB);
    return 2;
  }

  // One way branch.
  // Unconditional branch.
  if (Cond.empty())
    BuildMI(&MBB, DL, get(UncondBrOpc)).addMBB(TBB);
  else // Conditional branch.
    BuildCondBr(MBB, TBB, DL, Cond);
  return 1;
}

unsigned FgpuInstrInfo::removeBranch(MachineBasicBlock &MBB,
                                     int *BytesRemoved) const {
  assert(!BytesRemoved && "code size not handled");

  MachineBasicBlock::reverse_iterator I = MBB.rbegin(), REnd = MBB.rend();
  unsigned removed = 0;

  // Up to 2 branches are removed.
  // Note that indirect branches are not removed.
  while (I != REnd && removed < 2) {
    // Skip past debug instructions.
    if (I->isDebugInstr()) {
      ++I;
      continue;
    }
    if (!getAnalyzableBrOpc(I->getOpcode()))
      break;
    // Remove the branch.
    I->eraseFromParent();
    I = MBB.rbegin();
    ++removed;
  }

  return removed;
}

/// reverseBranchCondition - Return the inverse opcode of the
/// specified Branch instruction.
bool FgpuInstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert( (Cond.size() && Cond.size() <= 3) &&
          "Invalid Fgpu branch condition!");
  Cond[0].setImm(getOppositeBranchOpc(Cond[0].getImm()));
  return false;
}

FgpuInstrInfo::BranchType FgpuInstrInfo::analyzeBranch(
    MachineBasicBlock &MBB, MachineBasicBlock *&TBB, MachineBasicBlock *&FBB,
    SmallVectorImpl<MachineOperand> &Cond, bool AllowModify,
    SmallVectorImpl<MachineInstr *> &BranchInstrs) const {
  MachineBasicBlock::reverse_iterator I = MBB.rbegin(), REnd = MBB.rend();

  // Skip all the debug instructions.
  while (I != REnd && I->isDebugInstr())
    ++I;

  if (I == REnd || !isUnpredicatedTerminator(*I)) {
    // This block ends with no branches (it just falls through to its succ).
    // Leave TBB/FBB null.
    TBB = FBB = nullptr;
    return BT_NoBranch;
  }

  MachineInstr *LastInst = &*I;
  unsigned LastOpc = LastInst->getOpcode();
  BranchInstrs.push_back(LastInst);

  // Not an analyzable branch (e.g., indirect jump).
  if (!getAnalyzableBrOpc(LastOpc))
    return LastInst->isIndirectBranch() ? BT_Indirect : BT_None;

  // Get the second to last instruction in the block.
  unsigned SecondLastOpc = 0;
  MachineInstr *SecondLastInst = nullptr;

  // Skip past any debug instruction to see if the second last actual
  // is a branch.
  ++I;
  while (I != REnd && I->isDebugInstr())
    ++I;

  if (I != REnd) {
    SecondLastInst = &*I;
    SecondLastOpc = getAnalyzableBrOpc(SecondLastInst->getOpcode());

    // Not an analyzable branch (must be an indirect jump).
    if (isUnpredicatedTerminator(*SecondLastInst) && !SecondLastOpc)
      return BT_None;
  }

  // If there is only one terminator instruction, process it.
  if (!SecondLastOpc) {
    // Unconditional branch.
    if (LastInst->isUnconditionalBranch()) {
      TBB = LastInst->getOperand(0).getMBB();
      return BT_Uncond;
    }

    // Conditional branch
    AnalyzeCondBr(LastInst, LastOpc, TBB, Cond);
    return BT_Cond;
  }

  // If we reached here, there are two branches.
  // If there are three terminators, we don't know what sort of block this is.
  if (++I != REnd && isUnpredicatedTerminator(*I))
    return BT_None;

  BranchInstrs.insert(BranchInstrs.begin(), SecondLastInst);

  // If second to last instruction is an unconditional branch,
  // analyze it and remove the last instruction.
  if (SecondLastInst->isUnconditionalBranch()) {
    // Return if the last instruction cannot be removed.
    if (!AllowModify)
      return BT_None;

    TBB = SecondLastInst->getOperand(0).getMBB();
    LastInst->eraseFromParent();
    BranchInstrs.pop_back();
    return BT_Uncond;
  }

  // Conditional branch followed by an unconditional branch.
  // The last one must be unconditional.
  if (!LastInst->isUnconditionalBranch())
    return BT_None;

  AnalyzeCondBr(SecondLastInst, SecondLastOpc, TBB, Cond);
  FBB = LastInst->getOperand(0).getMBB();

  return BT_CondUncond;
}

bool FgpuInstrInfo::isBranchOffsetInRange(unsigned BranchOpc,
                                          int64_t BrOffset) const {
  switch (BranchOpc) {
  case Fgpu::B:
  case Fgpu::BAL:
  case Fgpu::BAL_BR:
  case Fgpu::BAL_BR_MM:
  case Fgpu::BC1F:
  case Fgpu::BC1FL:
  case Fgpu::BC1T:
  case Fgpu::BC1TL:
  case Fgpu::BEQ:     case Fgpu::BEQ64:
  case Fgpu::BEQL:
  case Fgpu::BGEZ:    case Fgpu::BGEZ64:
  case Fgpu::BGEZL:
  case Fgpu::BGEZAL:
  case Fgpu::BGEZALL:
  case Fgpu::BGTZ:    case Fgpu::BGTZ64:
  case Fgpu::BGTZL:
  case Fgpu::BLEZ:    case Fgpu::BLEZ64:
  case Fgpu::BLEZL:
  case Fgpu::BLTZ:    case Fgpu::BLTZ64:
  case Fgpu::BLTZL:
  case Fgpu::BLTZAL:
  case Fgpu::BLTZALL:
  case Fgpu::BNE:     case Fgpu::BNE64:
  case Fgpu::BNEL:
    return isInt<18>(BrOffset);

  // microFgpur3 branches
  case Fgpu::B_MM:
  case Fgpu::BC1F_MM:
  case Fgpu::BC1T_MM:
  case Fgpu::BEQ_MM:
  case Fgpu::BGEZ_MM:
  case Fgpu::BGEZAL_MM:
  case Fgpu::BGTZ_MM:
  case Fgpu::BLEZ_MM:
  case Fgpu::BLTZ_MM:
  case Fgpu::BLTZAL_MM:
  case Fgpu::BNE_MM:
  case Fgpu::BEQZC_MM:
  case Fgpu::BNEZC_MM:
    return isInt<17>(BrOffset);

  // microFgpuR3 short branches.
  case Fgpu::B16_MM:
    return isInt<11>(BrOffset);

  case Fgpu::BEQZ16_MM:
  case Fgpu::BNEZ16_MM:
    return isInt<8>(BrOffset);

  // FgpuR6 branches.
  case Fgpu::BALC:
  case Fgpu::BC:
    return isInt<28>(BrOffset);

  case Fgpu::BC1EQZ:
  case Fgpu::BC1NEZ:
  case Fgpu::BC2EQZ:
  case Fgpu::BC2NEZ:
  case Fgpu::BEQC:   case Fgpu::BEQC64:
  case Fgpu::BNEC:   case Fgpu::BNEC64:
  case Fgpu::BGEC:   case Fgpu::BGEC64:
  case Fgpu::BGEUC:  case Fgpu::BGEUC64:
  case Fgpu::BGEZC:  case Fgpu::BGEZC64:
  case Fgpu::BGTZC:  case Fgpu::BGTZC64:
  case Fgpu::BLEZC:  case Fgpu::BLEZC64:
  case Fgpu::BLTC:   case Fgpu::BLTC64:
  case Fgpu::BLTUC:  case Fgpu::BLTUC64:
  case Fgpu::BLTZC:  case Fgpu::BLTZC64:
  case Fgpu::BNVC:
  case Fgpu::BOVC:
  case Fgpu::BGEZALC:
  case Fgpu::BEQZALC:
  case Fgpu::BGTZALC:
  case Fgpu::BLEZALC:
  case Fgpu::BLTZALC:
  case Fgpu::BNEZALC:
    return isInt<18>(BrOffset);

  case Fgpu::BEQZC:  case Fgpu::BEQZC64:
  case Fgpu::BNEZC:  case Fgpu::BNEZC64:
    return isInt<23>(BrOffset);

  // microFgpuR6 branches
  case Fgpu::BC16_MMR6:
    return isInt<11>(BrOffset);

  case Fgpu::BEQZC16_MMR6:
  case Fgpu::BNEZC16_MMR6:
    return isInt<8>(BrOffset);

  case Fgpu::BALC_MMR6:
  case Fgpu::BC_MMR6:
    return isInt<27>(BrOffset);

  case Fgpu::BC1EQZC_MMR6:
  case Fgpu::BC1NEZC_MMR6:
  case Fgpu::BC2EQZC_MMR6:
  case Fgpu::BC2NEZC_MMR6:
  case Fgpu::BGEZALC_MMR6:
  case Fgpu::BEQZALC_MMR6:
  case Fgpu::BGTZALC_MMR6:
  case Fgpu::BLEZALC_MMR6:
  case Fgpu::BLTZALC_MMR6:
  case Fgpu::BNEZALC_MMR6:
  case Fgpu::BNVC_MMR6:
  case Fgpu::BOVC_MMR6:
    return isInt<17>(BrOffset);

  case Fgpu::BEQC_MMR6:
  case Fgpu::BNEC_MMR6:
  case Fgpu::BGEC_MMR6:
  case Fgpu::BGEUC_MMR6:
  case Fgpu::BGEZC_MMR6:
  case Fgpu::BGTZC_MMR6:
  case Fgpu::BLEZC_MMR6:
  case Fgpu::BLTC_MMR6:
  case Fgpu::BLTUC_MMR6:
  case Fgpu::BLTZC_MMR6:
    return isInt<18>(BrOffset);

  case Fgpu::BEQZC_MMR6:
  case Fgpu::BNEZC_MMR6:
    return isInt<23>(BrOffset);

  // DSP branches.
  case Fgpu::BPOSGE32:
    return isInt<18>(BrOffset);
  case Fgpu::BPOSGE32_MM:
  case Fgpu::BPOSGE32C_MMR3:
    return isInt<17>(BrOffset);

  // cnFgpu branches.
  case Fgpu::BBIT0:
  case Fgpu::BBIT032:
  case Fgpu::BBIT1:
  case Fgpu::BBIT132:
    return isInt<18>(BrOffset);

  // MSA branches.
  case Fgpu::BZ_B:
  case Fgpu::BZ_H:
  case Fgpu::BZ_W:
  case Fgpu::BZ_D:
  case Fgpu::BZ_V:
  case Fgpu::BNZ_B:
  case Fgpu::BNZ_H:
  case Fgpu::BNZ_W:
  case Fgpu::BNZ_D:
  case Fgpu::BNZ_V:
    return isInt<18>(BrOffset);
  }

  llvm_unreachable("Unknown branch instruction!");
}

/// Return the corresponding compact (no delay slot) form of a branch.
unsigned FgpuInstrInfo::getEquivalentCompactForm(
    const MachineBasicBlock::iterator I) const {
  unsigned Opcode = I->getOpcode();
  bool canUseShortMicroFgpuCTI = false;

  if (Subtarget.inMicroFgpuMode()) {
    switch (Opcode) {
    case Fgpu::BNE:
    case Fgpu::BNE_MM:
    case Fgpu::BEQ:
    case Fgpu::BEQ_MM:
    // microFgpu has NE,EQ branches that do not have delay slots provided one
    // of the operands is zero.
      if (I->getOperand(1).getReg() == Subtarget.getABI().GetZeroReg())
        canUseShortMicroFgpuCTI = true;
      break;
    // For microFgpu the PseudoReturn and PseudoIndirectBranch are always
    // expanded to JR_MM, so they can be replaced with JRC16_MM.
    case Fgpu::JR:
    case Fgpu::PseudoReturn:
    case Fgpu::PseudoIndirectBranch:
      canUseShortMicroFgpuCTI = true;
      break;
    }
  }

  // FgpuR6 forbids both operands being the zero register.
  if (Subtarget.hasFgpu32r6() && (I->getNumOperands() > 1) &&
      (I->getOperand(0).isReg() &&
       (I->getOperand(0).getReg() == Fgpu::ZERO ||
        I->getOperand(0).getReg() == Fgpu::ZERO_64)) &&
      (I->getOperand(1).isReg() &&
       (I->getOperand(1).getReg() == Fgpu::ZERO ||
        I->getOperand(1).getReg() == Fgpu::ZERO_64)))
    return 0;

  if (Subtarget.hasFgpu32r6() || canUseShortMicroFgpuCTI) {
    switch (Opcode) {
    case Fgpu::B:
      return Fgpu::BC;
    case Fgpu::BAL:
      return Fgpu::BALC;
    case Fgpu::BEQ:
    case Fgpu::BEQ_MM:
      if (canUseShortMicroFgpuCTI)
        return Fgpu::BEQZC_MM;
      else if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Fgpu::BEQC;
    case Fgpu::BNE:
    case Fgpu::BNE_MM:
      if (canUseShortMicroFgpuCTI)
        return Fgpu::BNEZC_MM;
      else if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Fgpu::BNEC;
    case Fgpu::BGE:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Fgpu::BGEC;
    case Fgpu::BGEU:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Fgpu::BGEUC;
    case Fgpu::BGEZ:
      return Fgpu::BGEZC;
    case Fgpu::BGTZ:
      return Fgpu::BGTZC;
    case Fgpu::BLEZ:
      return Fgpu::BLEZC;
    case Fgpu::BLT:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Fgpu::BLTC;
    case Fgpu::BLTU:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Fgpu::BLTUC;
    case Fgpu::BLTZ:
      return Fgpu::BLTZC;
    case Fgpu::BEQ64:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Fgpu::BEQC64;
    case Fgpu::BNE64:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Fgpu::BNEC64;
    case Fgpu::BGTZ64:
      return Fgpu::BGTZC64;
    case Fgpu::BGEZ64:
      return Fgpu::BGEZC64;
    case Fgpu::BLTZ64:
      return Fgpu::BLTZC64;
    case Fgpu::BLEZ64:
      return Fgpu::BLEZC64;
    // For FgpuR6, the instruction 'jic' can be used for these cases. Some
    // tools will accept 'jrc reg' as an alias for 'jic 0, $reg'.
    case Fgpu::JR:
    case Fgpu::PseudoIndirectBranchR6:
    case Fgpu::PseudoReturn:
    case Fgpu::TAILCALLR6REG:
      if (canUseShortMicroFgpuCTI)
        return Fgpu::JRC16_MM;
      return Fgpu::JIC;
    case Fgpu::JALRPseudo:
      return Fgpu::JIALC;
    case Fgpu::JR64:
    case Fgpu::PseudoIndirectBranch64R6:
    case Fgpu::PseudoReturn64:
    case Fgpu::TAILCALL64R6REG:
      return Fgpu::JIC64;
    case Fgpu::JALR64Pseudo:
      return Fgpu::JIALC64;
    default:
      return 0;
    }
  }

  return 0;
}

/// Predicate for distingushing between control transfer instructions and all
/// other instructions for handling forbidden slots. Consider inline assembly
/// as unsafe as well.
bool FgpuInstrInfo::SafeInForbiddenSlot(const MachineInstr &MI) const {
  if (MI.isInlineAsm())
    return false;

  return (MI.getDesc().TSFlags & FgpuII::IsCTI) == 0;
}

/// Predicate for distingushing instructions that have forbidden slots.
bool FgpuInstrInfo::HasForbiddenSlot(const MachineInstr &MI) const {
  return (MI.getDesc().TSFlags & FgpuII::HasForbiddenSlot) != 0;
}

/// Return the number of bytes of code the specified instruction may be.
unsigned FgpuInstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    return MI.getDesc().getSize();
  case  TargetOpcode::INLINEASM:
  case  TargetOpcode::INLINEASM_BR: {       // Inline Asm: Variable size.
    const MachineFunction *MF = MI.getParent()->getParent();
    const char *AsmStr = MI.getOperand(0).getSymbolName();
    return getInlineAsmLength(AsmStr, *MF->getTarget().getMCAsmInfo());
  }
  case Fgpu::CONSTPOOL_ENTRY:
    // If this machine instr is a constant pool entry, its size is recorded as
    // operand #2.
    return MI.getOperand(2).getImm();
  }
}

MachineInstrBuilder
FgpuInstrInfo::genInstrWithNewOpc(unsigned NewOpc,
                                  MachineBasicBlock::iterator I) const {
  MachineInstrBuilder MIB;

  // Certain branches have two forms: e.g beq $1, $zero, dest vs beqz $1, dest
  // Pick the zero form of the branch for readable assembly and for greater
  // branch distance in non-microFgpu mode.
  // Additional FgpuR6 does not permit the use of register $zero for compact
  // branches.
  // FIXME: Certain atomic sequences on Fgpu64 generate 32bit references to
  // Fgpu::ZERO, which is incorrect. This test should be updated to use
  // Subtarget.getABI().GetZeroReg() when those atomic sequences and others
  // are fixed.
  int ZeroOperandPosition = -1;
  bool BranchWithZeroOperand = false;
  if (I->isBranch() && !I->isPseudo()) {
    auto TRI = I->getParent()->getParent()->getSubtarget().getRegisterInfo();
    ZeroOperandPosition = I->findRegisterUseOperandIdx(Fgpu::ZERO, false, TRI);
    BranchWithZeroOperand = ZeroOperandPosition != -1;
  }

  if (BranchWithZeroOperand) {
    switch (NewOpc) {
    case Fgpu::BEQC:
      NewOpc = Fgpu::BEQZC;
      break;
    case Fgpu::BNEC:
      NewOpc = Fgpu::BNEZC;
      break;
    case Fgpu::BGEC:
      NewOpc = Fgpu::BGEZC;
      break;
    case Fgpu::BLTC:
      NewOpc = Fgpu::BLTZC;
      break;
    case Fgpu::BEQC64:
      NewOpc = Fgpu::BEQZC64;
      break;
    case Fgpu::BNEC64:
      NewOpc = Fgpu::BNEZC64;
      break;
    }
  }

  MIB = BuildMI(*I->getParent(), I, I->getDebugLoc(), get(NewOpc));

  // For FgpuR6 JI*C requires an immediate 0 as an operand, JIALC(64) an
  // immediate 0 as an operand and requires the removal of it's implicit-def %ra
  // implicit operand as copying the implicit operations of the instructio we're
  // looking at will give us the correct flags.
  if (NewOpc == Fgpu::JIC || NewOpc == Fgpu::JIALC || NewOpc == Fgpu::JIC64 ||
      NewOpc == Fgpu::JIALC64) {

    if (NewOpc == Fgpu::JIALC || NewOpc == Fgpu::JIALC64)
      MIB->RemoveOperand(0);

    for (unsigned J = 0, E = I->getDesc().getNumOperands(); J < E; ++J) {
      MIB.add(I->getOperand(J));
    }

    MIB.addImm(0);

    // If I has an MCSymbol operand (used by asm printer, to emit R_Fgpu_JALR),
    // add it to the new instruction.
    for (unsigned J = I->getDesc().getNumOperands(), E = I->getNumOperands();
         J < E; ++J) {
      const MachineOperand &MO = I->getOperand(J);
      if (MO.isMCSymbol() && (MO.getTargetFlags() & FgpuII::MO_JALR))
        MIB.addSym(MO.getMCSymbol(), FgpuII::MO_JALR);
    }


  } else {
    for (unsigned J = 0, E = I->getDesc().getNumOperands(); J < E; ++J) {
      if (BranchWithZeroOperand && (unsigned)ZeroOperandPosition == J)
        continue;

      MIB.add(I->getOperand(J));
    }
  }

  MIB.copyImplicitOps(*I);
  MIB.cloneMemRefs(*I);
  return MIB;
}

bool FgpuInstrInfo::findCommutedOpIndices(const MachineInstr &MI,
                                          unsigned &SrcOpIdx1,
                                          unsigned &SrcOpIdx2) const {
  assert(!MI.isBundle() &&
         "TargetInstrInfo::findCommutedOpIndices() can't handle bundles");

  const MCInstrDesc &MCID = MI.getDesc();
  if (!MCID.isCommutable())
    return false;

  switch (MI.getOpcode()) {
  case Fgpu::DPADD_U_H:
  case Fgpu::DPADD_U_W:
  case Fgpu::DPADD_U_D:
  case Fgpu::DPADD_S_H:
  case Fgpu::DPADD_S_W:
  case Fgpu::DPADD_S_D:
    // The first operand is both input and output, so it should not commute
    if (!fixCommutedOpIndices(SrcOpIdx1, SrcOpIdx2, 2, 3))
      return false;

    if (!MI.getOperand(SrcOpIdx1).isReg() || !MI.getOperand(SrcOpIdx2).isReg())
      return false;
    return true;
  }
  return TargetInstrInfo::findCommutedOpIndices(MI, SrcOpIdx1, SrcOpIdx2);
}

// ins, ext, dext*, dins have the following constraints:
// X <= pos      <  Y
// X <  size     <= Y
// X <  pos+size <= Y
//
// dinsm and dinsu have the following constraints:
// X <= pos      <  Y
// X <= size     <= Y
// X <  pos+size <= Y
//
// The callee of verifyInsExtInstruction however gives the bounds of
// dins[um] like the other (d)ins (d)ext(um) instructions, so that this
// function doesn't have to vary it's behaviour based on the instruction
// being checked.
static bool verifyInsExtInstruction(const MachineInstr &MI, StringRef &ErrInfo,
                                    const int64_t PosLow, const int64_t PosHigh,
                                    const int64_t SizeLow,
                                    const int64_t SizeHigh,
                                    const int64_t BothLow,
                                    const int64_t BothHigh) {
  MachineOperand MOPos = MI.getOperand(2);
  if (!MOPos.isImm()) {
    ErrInfo = "Position is not an immediate!";
    return false;
  }
  int64_t Pos = MOPos.getImm();
  if (!((PosLow <= Pos) && (Pos < PosHigh))) {
    ErrInfo = "Position operand is out of range!";
    return false;
  }

  MachineOperand MOSize = MI.getOperand(3);
  if (!MOSize.isImm()) {
    ErrInfo = "Size operand is not an immediate!";
    return false;
  }
  int64_t Size = MOSize.getImm();
  if (!((SizeLow < Size) && (Size <= SizeHigh))) {
    ErrInfo = "Size operand is out of range!";
    return false;
  }

  if (!((BothLow < (Pos + Size)) && ((Pos + Size) <= BothHigh))) {
    ErrInfo = "Position + Size is out of range!";
    return false;
  }

  return true;
}

//  Perform target specific instruction verification.
bool FgpuInstrInfo::verifyInstruction(const MachineInstr &MI,
                                      StringRef &ErrInfo) const {
  // Verify that ins and ext instructions are well formed.
  switch (MI.getOpcode()) {
    case Fgpu::EXT:
    case Fgpu::EXT_MM:
    case Fgpu::INS:
    case Fgpu::INS_MM:
    case Fgpu::DINS:
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 0, 32, 0, 32);
    case Fgpu::DINSM:
      // The ISA spec has a subtle difference between dinsm and dextm
      // in that it says:
      // 2 <= size <= 64 for 'dinsm' but 'dextm' has 32 < size <= 64.
      // To make the bounds checks similar, the range 1 < size <= 64 is checked
      // for 'dinsm'.
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 1, 64, 32, 64);
    case Fgpu::DINSU:
      // The ISA spec has a subtle difference between dinsu and dextu in that
      // the size range of dinsu is specified as 1 <= size <= 32 whereas size
      // for dextu is 0 < size <= 32. The range checked for dinsu here is
      // 0 < size <= 32, which is equivalent and similar to dextu.
      return verifyInsExtInstruction(MI, ErrInfo, 32, 64, 0, 32, 32, 64);
    case Fgpu::DEXT:
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 0, 32, 0, 63);
    case Fgpu::DEXTM:
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 32, 64, 32, 64);
    case Fgpu::DEXTU:
      return verifyInsExtInstruction(MI, ErrInfo, 32, 64, 0, 32, 32, 64);
    case Fgpu::TAILCALLREG:
    case Fgpu::PseudoIndirectBranch:
    case Fgpu::JR:
    case Fgpu::JR64:
    case Fgpu::JALR:
    case Fgpu::JALR64:
    case Fgpu::JALRPseudo:
      if (!Subtarget.useIndirectJumpsHazard())
        return true;

      ErrInfo = "invalid instruction when using jump guards!";
      return false;
    default:
      return true;
  }

  return true;
}

std::pair<unsigned, unsigned>
FgpuInstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const {
  return std::make_pair(TF, 0u);
}

ArrayRef<std::pair<unsigned, const char*>>
FgpuInstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
 using namespace FgpuII;

 static const std::pair<unsigned, const char*> Flags[] = {
    {MO_GOT,          "Fgpu-got"},
    {MO_GOT_CALL,     "Fgpu-got-call"},
    {MO_GPREL,        "Fgpu-gprel"},
    {MO_ABS_HI,       "Fgpu-abs-hi"},
    {MO_ABS_LO,       "Fgpu-abs-lo"},
    {MO_TLSGD,        "Fgpu-tlsgd"},
    {MO_TLSLDM,       "Fgpu-tlsldm"},
    {MO_DTPREL_HI,    "Fgpu-dtprel-hi"},
    {MO_DTPREL_LO,    "Fgpu-dtprel-lo"},
    {MO_GOTTPREL,     "Fgpu-gottprel"},
    {MO_TPREL_HI,     "Fgpu-tprel-hi"},
    {MO_TPREL_LO,     "Fgpu-tprel-lo"},
    {MO_GPOFF_HI,     "Fgpu-gpoff-hi"},
    {MO_GPOFF_LO,     "Fgpu-gpoff-lo"},
    {MO_GOT_DISP,     "Fgpu-got-disp"},
    {MO_GOT_PAGE,     "Fgpu-got-page"},
    {MO_GOT_OFST,     "Fgpu-got-ofst"},
    {MO_HIGHER,       "Fgpu-higher"},
    {MO_HIGHEST,      "Fgpu-highest"},
    {MO_GOT_HI16,     "Fgpu-got-hi16"},
    {MO_GOT_LO16,     "Fgpu-got-lo16"},
    {MO_CALL_HI16,    "Fgpu-call-hi16"},
    {MO_CALL_LO16,    "Fgpu-call-lo16"},
    {MO_JALR,         "Fgpu-jalr"}
  };
  return makeArrayRef(Flags);
}

Optional<ParamLoadedValue>
FgpuInstrInfo::describeLoadedValue(const MachineInstr &MI, Register Reg) const {
  DIExpression *Expr =
      DIExpression::get(MI.getMF()->getFunction().getContext(), {});

  // TODO: Special Fgpu instructions that need to be described separately.
  if (auto RegImm = isAddImmediate(MI, Reg)) {
    Register SrcReg = RegImm->Reg;
    int64_t Offset = RegImm->Imm;
    // When SrcReg is $zero, treat loaded value as immediate only.
    // Ex. $a2 = ADDiu $zero, 10
    if (SrcReg == Fgpu::ZERO || SrcReg == Fgpu::ZERO_64) {
      return ParamLoadedValue(MI.getOperand(2), Expr);
    }
    Expr = DIExpression::prepend(Expr, DIExpression::ApplyOffset, Offset);
    return ParamLoadedValue(MachineOperand::CreateReg(SrcReg, false), Expr);
  } else if (auto DestSrc = isCopyInstr(MI)) {
    const MachineFunction *MF = MI.getMF();
    const TargetRegisterInfo *TRI = MF->getSubtarget().getRegisterInfo();
    Register DestReg = DestSrc->Destination->getReg();
    // TODO: Handle cases where the Reg is sub- or super-register of the
    // DestReg.
    if (TRI->isSuperRegister(Reg, DestReg) || TRI->isSubRegister(Reg, DestReg))
      return None;
  }

  return TargetInstrInfo::describeLoadedValue(MI, Reg);
}

Optional<RegImmPair> FgpuInstrInfo::isAddImmediate(const MachineInstr &MI,
                                                   Register Reg) const {
  // TODO: Handle cases where Reg is a super- or sub-register of the
  // destination register.
  const MachineOperand &Op0 = MI.getOperand(0);
  if (!Op0.isReg() || Reg != Op0.getReg())
    return None;

  switch (MI.getOpcode()) {
  case Fgpu::ADDiu:
  case Fgpu::DADDiu: {
    const MachineOperand &Dop = MI.getOperand(0);
    const MachineOperand &Sop1 = MI.getOperand(1);
    const MachineOperand &Sop2 = MI.getOperand(2);
    // Value is sum of register and immediate. Immediate value could be
    // global string address which is not supported.
    if (Dop.isReg() && Sop1.isReg() && Sop2.isImm())
      return RegImmPair{Sop1.getReg(), Sop2.getImm()};
    // TODO: Handle case where Sop1 is a frame-index.
  }
  }
  return None;
}
