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
      Subtarget(STI)/*, UncondBrOpc(UncondBr)*/ {}

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

  return (BT == BT_CouldntAnalyse) || (BT == BT_Indirect); // true return means couldn't analyse
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

void FgpuInstrInfo::BuildUnCondBr(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                                const DebugLoc &DL) const {
  unsigned Opc = Fgpu::BEQ;
  const MCInstrDesc &MCID = get(Opc);
  MachineInstrBuilder MIB = BuildMI(&MBB, DL, MCID);
  MIB.addReg(Fgpu::ZERO);
  MIB.addReg(Fgpu::ZERO);
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
    BuildUnCondBr(MBB, FBB, DL);
    //BuildMI(&MBB, DL, get(UncondBrOpc)).addMBB(FBB);
    return 2;
  }

  // One way branch.
  // Unconditional branch.
  if (Cond.empty())
    BuildUnCondBr(MBB, TBB, DL);
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

unsigned FgpuInstrInfo::getAnalyzableBrOpc(unsigned Opc) const {
  return (Opc == Fgpu::BEQ || Opc == Fgpu::BNE) ? Opc : 0;
}

/// GetOppositeBranchOpc - Return the inverse of the specified
/// opcode, e.g. turning BEQ to BNE.
unsigned FgpuInstrInfo::getOppositeBranchOpc(unsigned Opc) const {
  switch (Opc) {
  case Fgpu::BEQ: return Fgpu::BNE;
  case Fgpu::BNE: return Fgpu::BEQ;
  }
  llvm_unreachable("Illegal opcode!");
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
    return LastInst->isIndirectBranch() ? BT_Indirect : BT_CouldntAnalyse;

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
      return BT_CouldntAnalyse;
  }

  // If there is only one terminator instruction, process it.
  if (!SecondLastOpc) {
    // Unconditional branch.
    if (LastInst->isUnconditionalBranch()) {
      //TBB = LastInst->getOperand(0).getMBB();
      TBB = LastInst->getOperand(LastInst->getNumExplicitOperands() - 1).getMBB();
      return BT_Uncond;
    }

    // Conditional branch
    AnalyzeCondBr(LastInst, LastOpc, TBB, Cond);
    return BT_Cond;
  }

  // If we reached here, there are two branches.
  // If there are three terminators, we don't know what sort of block this is.
  if (++I != REnd && isUnpredicatedTerminator(*I))
    return BT_CouldntAnalyse;

  BranchInstrs.insert(BranchInstrs.begin(), SecondLastInst);

  // If second to last instruction is an unconditional branch,
  // analyze it and remove the last instruction.
  if (SecondLastInst->isUnconditionalBranch()) {
    // Return if the last instruction cannot be removed.
    if (!AllowModify)
      return BT_CouldntAnalyse;

    TBB = SecondLastInst->getOperand(SecondLastInst->getNumExplicitOperands() - 1).getMBB();
    LastInst->eraseFromParent();
    BranchInstrs.pop_back();
    return BT_Uncond;
  }

  // Conditional branch followed by an unconditional branch.
  // The last one must be unconditional.
  bool lastB = LastOpc == Fgpu::BEQ;
  lastB &= LastInst->getNumExplicitOperands() >= 2;
  lastB &= (LastInst->getOperand(0).isReg() && LastInst->getOperand(1).isReg());
  lastB &= (LastInst->getOperand(0).getReg() == LastInst->getOperand(1).getReg());
  // lastB is an ersatz isUnconditionalbranch, since there isn't one in the ISA
  if (!LastInst->isUnconditionalBranch() && !lastB)
    return BT_CouldntAnalyse;

  AnalyzeCondBr(SecondLastInst, SecondLastOpc, TBB, Cond);
  FBB = LastInst->getOperand(LastInst->getNumExplicitOperands() - 1).getMBB();

  return BT_CondUncond;
}

bool FgpuInstrInfo::isBranchOffsetInRange(unsigned BranchOpc,
                                          int64_t BrOffset) const {
  switch (BranchOpc) {
  case Fgpu::BEQ:
  case Fgpu::BNE:
    return isInt<14>(BrOffset);
  }

  llvm_unreachable("Unknown branch instruction!");
}

/// Return the corresponding compact (no delay slot) form of a branch.
unsigned FgpuInstrInfo::getEquivalentCompactForm(
    const MachineBasicBlock::iterator I) const {
  unsigned Opcode = I->getOpcode();
  return 0;
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
  }
}

MachineInstrBuilder
FgpuInstrInfo::genInstrWithNewOpc(unsigned NewOpc,
                                  MachineBasicBlock::iterator I) const {
  MachineInstrBuilder MIB;
  assert(false); // this won't work

  // Certain branches have two forms: e.g beq $1, $zero, dest vs beqz $1, dest
  // Pick the zero form of the branch for readable assembly and for greater
  // branch distance in non-microFGPU mode.
  // Additional FGPUR6 does not permit the use of register $zero for compact
  // branches.
  // FIXME: Certain atomic sequences on fgpu64 generate 32bit references to
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

  MIB = BuildMI(*I->getParent(), I, I->getDebugLoc(), get(NewOpc));

  // For FGPUR6 JI*C requires an immediate 0 as an operand, JIALC(64) an
  // immediate 0 as an operand and requires the removal of it's implicit-def %ra
  // implicit operand as copying the implicit operations of the instructio we're
  // looking at will give us the correct flags.
  {
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

//  switch (MI.getOpcode()) {
//  case Fgpu::DPADD_U_H:
//  case Fgpu::DPADD_U_W:
//  case Fgpu::DPADD_U_D:
//  case Fgpu::DPADD_S_H:
//  case Fgpu::DPADD_S_W:
//  case Fgpu::DPADD_S_D:
//    // The first operand is both input and output, so it should not commute
//    if (!fixCommutedOpIndices(SrcOpIdx1, SrcOpIdx2, 2, 3))
//      return false;
//
//    if (!MI.getOperand(SrcOpIdx1).isReg() || !MI.getOperand(SrcOpIdx2).isReg())
//      return false;
//    return true;
//  }
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
//  switch (MI.getOpcode()) {
//    case Fgpu::EXT:
//    case Fgpu::INS:
//    case Fgpu::DINS:
//      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 0, 32, 0, 32);
//    case Fgpu::DINSM:
//      // The ISA spec has a subtle difference between dinsm and dextm
//      // in that it says:
//      // 2 <= size <= 64 for 'dinsm' but 'dextm' has 32 < size <= 64.
//      // To make the bounds checks similar, the range 1 < size <= 64 is checked
//      // for 'dinsm'.
//      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 1, 64, 32, 64);
//    case Fgpu::DINSU:
//      // The ISA spec has a subtle difference between dinsu and dextu in that
//      // the size range of dinsu is specified as 1 <= size <= 32 whereas size
//      // for dextu is 0 < size <= 32. The range checked for dinsu here is
//      // 0 < size <= 32, which is equivalent and similar to dextu.
//      return verifyInsExtInstruction(MI, ErrInfo, 32, 64, 0, 32, 32, 64);
//    case Fgpu::DEXT:
//      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 0, 32, 0, 63);
//    case Fgpu::DEXTM:
//      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 32, 64, 32, 64);
//    case Fgpu::DEXTU:
//      return verifyInsExtInstruction(MI, ErrInfo, 32, 64, 0, 32, 32, 64);
//    case Fgpu::TAILCALLREG:
//    case Fgpu::PseudoIndirectBranch:
//    case Fgpu::JR:
//    case Fgpu::JR64:
//    case Fgpu::JALR:
//    case Fgpu::JALR64:
//    case Fgpu::JALRPseudo:
//      if (!Subtarget.useIndirectJumpsHazard())
//        return true;
//
//      ErrInfo = "invalid instruction when using jump guards!";
//      return false;
//    default:
//      return true;
//  }
//
//  return true;
    return false; // these arent implemeted
}

std::pair<unsigned, unsigned>
FgpuInstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const {
  return std::make_pair(TF, 0u);
}

ArrayRef<std::pair<unsigned, const char*>>
FgpuInstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
 using namespace FgpuII;

 static const std::pair<unsigned, const char*> Flags[] = {
    {MO_GOT,          "fgpu-got"},
    {MO_GOT_CALL,     "fgpu-got-call"},
    {MO_GPREL,        "fgpu-gprel"},
    {MO_ABS_HI,       "fgpu-abs-hi"},
    {MO_ABS_LO,       "fgpu-abs-lo"},
    {MO_TLSGD,        "fgpu-tlsgd"},
    {MO_TLSLDM,       "fgpu-tlsldm"},
    {MO_DTPREL_HI,    "fgpu-dtprel-hi"},
    {MO_DTPREL_LO,    "fgpu-dtprel-lo"},
    {MO_GOTTPREL,     "fgpu-gottprel"},
    {MO_TPREL_HI,     "fgpu-tprel-hi"},
    {MO_TPREL_LO,     "fgpu-tprel-lo"},
    {MO_GPOFF_HI,     "fgpu-gpoff-hi"},
    {MO_GPOFF_LO,     "fgpu-gpoff-lo"},
    {MO_GOT_DISP,     "fgpu-got-disp"},
    {MO_GOT_PAGE,     "fgpu-got-page"},
    {MO_GOT_OFST,     "fgpu-got-ofst"},
    {MO_GOT_HI16,     "fgpu-got-hi16"},
    {MO_GOT_LO16,     "fgpu-got-lo16"},
    {MO_CALL_HI16,    "fgpu-call-hi16"},
    {MO_CALL_LO16,    "fgpu-call-lo16"},
    {MO_JALR,         "fgpu-jalr"}
  };
  return makeArrayRef(Flags);
}

Optional<ParamLoadedValue>
FgpuInstrInfo::describeLoadedValue(const MachineInstr &MI, Register Reg) const {
  DIExpression *Expr =
      DIExpression::get(MI.getMF()->getFunction().getContext(), {});

  // TODO: Special FGPU instructions that need to be described separately.
  if (auto RegImm = isAddImmediate(MI, Reg)) {
    Register SrcReg = RegImm->Reg;
    int64_t Offset = RegImm->Imm;
    // When SrcReg is $zero, treat loaded value as immediate only.
    // Ex. $a2 = ADDiu $zero, 10
    if (SrcReg == Fgpu::ZERO) {
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
  case Fgpu::ADDi: {
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
