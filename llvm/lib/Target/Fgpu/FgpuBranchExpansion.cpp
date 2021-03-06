//===----------------------- FgpuBranchExpansion.cpp ----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
///
/// This pass do two things:
/// - it expands a branch or jump instruction into a long branch if its offset
///   is too large to fit into its immediate field,
/// - it inserts nops to prevent forbidden slot hazards.
///
/// The reason why this pass combines these two tasks is that one of these two
/// tasks can break the result of the previous one.
///
/// Example of that is a situation where at first, no branch should be expanded,
/// but after adding at least one nop somewhere in the code to prevent a
/// forbidden slot hazard, offset of some branches may go out of range. In that
/// case it is necessary to check again if there is some branch that needs
/// expansion. On the other hand, expanding some branch may cause a control
/// transfer instruction to appear in the forbidden slot, which is a hazard that
/// should be fixed. This pass alternates between this two tasks untill no
/// changes are made. Only then we can be sure that all branches are expanded
/// properly, and no hazard situations exist.
///
/// Regarding branch expanding:
///
/// When branch instruction like beqzc or bnezc has offset that is too large
/// to fit into its immediate field, it has to be expanded to another
/// instruction or series of instructions.
///
/// FIXME: Fix pc-region jump instructions which cross 256MB segment boundaries.
/// TODO: Handle out of range bc, b (pseudo) instructions.
///
/// Regarding compact branch hazard prevention:
///
/// Hazards handled: forbidden slots for FGPUR6.
///
/// A forbidden slot hazard occurs when a compact branch instruction is executed
/// and the adjacent instruction in memory is a control transfer instruction
/// such as a branch or jump, ERET, ERETNC, DERET, WAIT and PAUSE.
///
/// For example:
///
/// 0x8004      bnec    a1,v0,<P+0x18>
/// 0x8008      beqc    a1,a2,<P+0x54>
///
/// In such cases, the processor is required to signal a Reserved Instruction
/// exception.
///
/// Here, if the instruction at 0x8004 is executed, the processor will raise an
/// exception as there is a control transfer instruction at 0x8008.
///
/// There are two sources of forbidden slot hazards:
///
/// A) A previous pass has created a compact branch directly.
/// B) Transforming a delay slot branch into compact branch. This case can be
///    difficult to process as lookahead for hazards is insufficient, as
///    backwards delay slot fillling can also produce hazards in previously
///    processed instuctions.
///
/// In future this pass can be extended (or new pass can be created) to handle
/// other pipeline hazards, such as various FGPU1 hazards, processor errata that
/// require instruction reorganization, etc.
///
/// This pass has to run after the delay slot filler as that pass can introduce
/// pipeline hazards such as compact branch hazard, hence the existing hazard
/// recognizer is not suitable.
///
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/FgpuABIInfo.h"
#include "MCTargetDesc/FgpuBaseInfo.h"
#include "MCTargetDesc/FgpuMCTargetDesc.h"
#include "Fgpu.h"
#include "FgpuInstrInfo.h"
#include "FgpuMachineFunction.h"
#include "FgpuSubtarget.h"
#include "FgpuTargetMachine.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Target/TargetMachine.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iterator>
#include <utility>

using namespace llvm;

#define DEBUG_TYPE "fgpu-branch-expansion"

STATISTIC(NumInsertedNops, "Number of nops inserted");
STATISTIC(LongBranches, "Number of long branches.");

static cl::opt<bool>
    SkipLongBranch("skip-fgpu-long-branch", cl::init(false),
                   cl::desc("FGPU: Skip branch expansion pass."), cl::Hidden);

static cl::opt<bool>
    ForceLongBranch("force-fgpu-long-branch", cl::init(false),
                    cl::desc("FGPU: Expand all branches to long format."),
                    cl::Hidden);

namespace {

using Iter = MachineBasicBlock::iterator;
using ReverseIter = MachineBasicBlock::reverse_iterator;

struct MBBInfo {
  uint64_t Size = 0;
  bool HasLongBranch = false;
  MachineInstr *Br = nullptr;
  uint64_t Offset = 0;
  MBBInfo() = default;
};

class FgpuBranchExpansion : public MachineFunctionPass {
public:
  static char ID;

  FgpuBranchExpansion() : MachineFunctionPass(ID), ABI(FgpuABIInfo::Unknown()) {
    initializeFgpuBranchExpansionPass(*PassRegistry::getPassRegistry());
  }

  StringRef getPassName() const override {
    return "Fgpu Branch Expansion Pass";
  }

  bool runOnMachineFunction(MachineFunction &F) override;

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties().set(
        MachineFunctionProperties::Property::NoVRegs);
  }

private:
  void splitMBB(MachineBasicBlock *MBB);
  void initMBBInfo();
  int64_t computeOffset(const MachineInstr *Br);
  uint64_t computeOffsetFromTheBeginning(int MBB);
  void replaceBranch(MachineBasicBlock &MBB, Iter Br, const DebugLoc &DL,
                     MachineBasicBlock *MBBOpnd);
  bool buildProperJumpMI(MachineBasicBlock *MBB,
                         MachineBasicBlock::iterator Pos, DebugLoc DL);
  void expandToLongBranch(MBBInfo &Info);
  bool handleForbiddenSlot();
  bool handlePossibleLongBranch();

  const FgpuSubtarget *STI;
  const FgpuInstrInfo *TII;

  MachineFunction *MFp;
  SmallVector<MBBInfo, 16> MBBInfos;
  bool IsPIC;
  FgpuABIInfo ABI;
  bool ForceLongBranchFirstPass = false;
};

} // end of anonymous namespace

char FgpuBranchExpansion::ID = 0;

INITIALIZE_PASS(FgpuBranchExpansion, DEBUG_TYPE,
                "Expand out of range branch instructions and fix forbidden"
                " slot hazards",
                false, false)

/// Returns a pass that clears pipeline hazards.
FunctionPass *llvm::createFgpuBranchExpansion() {
  return new FgpuBranchExpansion();
}

// Find the next real instruction from the current position in current basic
// block.
static Iter getNextMachineInstrInBB(Iter Position) {
  Iter I = Position, E = Position->getParent()->end();
  I = std::find_if_not(I, E,
                       [](const Iter &Insn) { return Insn->isTransient(); });

  return I;
}

// Find the next real instruction from the current position, looking through
// basic block boundaries.
static std::pair<Iter, bool> getNextMachineInstr(Iter Position,
                                                 MachineBasicBlock *Parent) {
  if (Position == Parent->end()) {
    do {
      MachineBasicBlock *Succ = Parent->getNextNode();
      if (Succ != nullptr && Parent->isSuccessor(Succ)) {
        Position = Succ->begin();
        Parent = Succ;
      } else {
        return std::make_pair(Position, true);
      }
    } while (Parent->empty());
  }

  Iter Instr = getNextMachineInstrInBB(Position);
  if (Instr == Parent->end()) {
    return getNextMachineInstr(Instr, Parent);
  }
  return std::make_pair(Instr, false);
}

/// Iterate over list of Br's operands and search for a MachineBasicBlock
/// operand.
static MachineBasicBlock *getTargetMBB(const MachineInstr &Br) {
  for (unsigned I = 0, E = Br.getDesc().getNumOperands(); I < E; ++I) {
    const MachineOperand &MO = Br.getOperand(I);

    if (MO.isMBB())
      return MO.getMBB();
  }

  llvm_unreachable("This instruction does not have an MBB operand.");
}

// Traverse the list of instructions backwards until a non-debug instruction is
// found or it reaches E.
static ReverseIter getNonDebugInstr(ReverseIter B, const ReverseIter &E) {
  for (; B != E; ++B)
    if (!B->isDebugInstr())
      return B;

  return E;
}

// Split MBB if it has two direct jumps/branches.
void FgpuBranchExpansion::splitMBB(MachineBasicBlock *MBB) {
  ReverseIter End = MBB->rend();
  ReverseIter LastBr = getNonDebugInstr(MBB->rbegin(), End);

  // Return if MBB has no branch instructions.
  if ((LastBr == End) ||
      (!LastBr->isConditionalBranch() && !LastBr->isUnconditionalBranch()))
    return;

  ReverseIter FirstBr = getNonDebugInstr(std::next(LastBr), End);

  // MBB has only one branch instruction if FirstBr is not a branch
  // instruction.
  if ((FirstBr == End) ||
      (!FirstBr->isConditionalBranch() && !FirstBr->isUnconditionalBranch()))
    return;

  assert(!FirstBr->isIndirectBranch() && "Unexpected indirect branch found.");

  // Create a new MBB. Move instructions in MBB to the newly created MBB.
  MachineBasicBlock *NewMBB =
      MFp->CreateMachineBasicBlock(MBB->getBasicBlock());

  // Insert NewMBB and fix control flow.
  MachineBasicBlock *Tgt = getTargetMBB(*FirstBr);
  NewMBB->transferSuccessors(MBB);
  if (Tgt != getTargetMBB(*LastBr))
    NewMBB->removeSuccessor(Tgt, true);
  MBB->addSuccessor(NewMBB);
  MBB->addSuccessor(Tgt);
  MFp->insert(std::next(MachineFunction::iterator(MBB)), NewMBB);

  NewMBB->splice(NewMBB->end(), MBB, LastBr.getReverse(), MBB->end());
}

// Fill MBBInfos.
void FgpuBranchExpansion::initMBBInfo() {
  // Split the MBBs if they have two branches. Each basic block should have at
  // most one branch after this loop is executed.
  for (auto &MBB : *MFp)
    splitMBB(&MBB);

  MFp->RenumberBlocks();
  MBBInfos.clear();
  MBBInfos.resize(MFp->size());

  for (unsigned I = 0, E = MBBInfos.size(); I < E; ++I) {
    MachineBasicBlock *MBB = MFp->getBlockNumbered(I);

    // Compute size of MBB.
    for (MachineBasicBlock::instr_iterator MI = MBB->instr_begin();
         MI != MBB->instr_end(); ++MI)
      MBBInfos[I].Size += TII->getInstSizeInBytes(*MI);
  }
}

// Compute offset of branch in number of bytes.
int64_t FgpuBranchExpansion::computeOffset(const MachineInstr *Br) {
  int64_t Offset = 0;
  int ThisMBB = Br->getParent()->getNumber();
  int TargetMBB = getTargetMBB(*Br)->getNumber();

  // Compute offset of a forward branch.
  if (ThisMBB < TargetMBB) {
    for (int N = ThisMBB + 1; N < TargetMBB; ++N)
      Offset += MBBInfos[N].Size;

    return Offset + 4;
  }

  // Compute offset of a backward branch.
  for (int N = ThisMBB; N >= TargetMBB; --N)
    Offset += MBBInfos[N].Size;

  return -Offset + 4;
}

// Returns the distance in bytes up until MBB
uint64_t FgpuBranchExpansion::computeOffsetFromTheBeginning(int MBB) {
  uint64_t Offset = 0;
  for (int N = 0; N < MBB; ++N)
    Offset += MBBInfos[N].Size;
  return Offset;
}

// Replace Br with a branch which has the opposite condition code and a
// MachineBasicBlock operand MBBOpnd.
void FgpuBranchExpansion::replaceBranch(MachineBasicBlock &MBB, Iter Br,
                                        const DebugLoc &DL,
                                        MachineBasicBlock *MBBOpnd) {
  unsigned NewOpc = TII->getOppositeBranchOpc(Br->getOpcode());
  const MCInstrDesc &NewDesc = TII->get(NewOpc);

  MachineInstrBuilder MIB = BuildMI(MBB, Br, DL, NewDesc);

  for (unsigned I = 0, E = Br->getDesc().getNumOperands(); I < E; ++I) {
    MachineOperand &MO = Br->getOperand(I);

    switch (MO.getType()) {
    case MachineOperand::MO_Register:
      MIB.addReg(MO.getReg());
      break;
    case MachineOperand::MO_Immediate:
      // Octeon BBIT family of branch has an immediate operand
      // (e.g. BBIT0 $v0, 3, %bb.1).
      if (!TII->isBranchWithImm(Br->getOpcode()))
        llvm_unreachable("Unexpected immediate in branch instruction");
      MIB.addImm(MO.getImm());
      break;
    case MachineOperand::MO_MachineBasicBlock:
      MIB.addMBB(MBBOpnd);
      break;
    default:
      llvm_unreachable("Unexpected operand type in branch instruction");
    }
  }
  Br->eraseFromParent();
}

bool FgpuBranchExpansion::buildProperJumpMI(MachineBasicBlock *MBB,
                                            MachineBasicBlock::iterator Pos,
                                            DebugLoc DL) {
  bool HasR6 = false;
  bool AddImm = false;

  unsigned JR = 0; //Fgpu::JALR; //TODO: make JALR/JR a thing

  unsigned JumpOp = JR;


  unsigned ATReg = Fgpu::AT;
  MachineInstrBuilder Instr =
      BuildMI(*MBB, Pos, DL, TII->get(JumpOp)).addReg(ATReg);
  if (AddImm)
    Instr.addImm(0);

  return !AddImm;
}

// Expand branch instructions to long branches.
// TODO: This function has to be fixed for beqz16 and bnez16, because it
// currently assumes that all branches have 16-bit offsets, and will produce
// wrong code if branches whose allowed offsets are [-128, -126, ..., 126]
// are present.
void FgpuBranchExpansion::expandToLongBranch(MBBInfo &I) {
  assert(false && "don't work");
  MachineBasicBlock::iterator Pos;
  MachineBasicBlock *MBB = I.Br->getParent(), *TgtMBB = getTargetMBB(*I.Br);
  DebugLoc DL = I.Br->getDebugLoc();
  const BasicBlock *BB = MBB->getBasicBlock();
  MachineFunction::iterator FallThroughMBB = ++MachineFunction::iterator(MBB);
  MachineBasicBlock *LongBrMBB = MFp->CreateMachineBasicBlock(BB);

  MFp->insert(FallThroughMBB, LongBrMBB);
  MBB->replaceSuccessor(TgtMBB, LongBrMBB);

  if (IsPIC) {
    MachineBasicBlock *BalTgtMBB = MFp->CreateMachineBasicBlock(BB);
    MFp->insert(FallThroughMBB, BalTgtMBB);
    LongBrMBB->addSuccessor(BalTgtMBB);
    BalTgtMBB->addSuccessor(TgtMBB);

    // We must select between the FGPU32r6/FGPU64r6 BALC (which is a normal
    // instruction) and the pre-FGPU32r6/FGPU64r6 definition (which is an
    // pseudo-instruction wrapping BGEZAL).
    const unsigned BalOp = Fgpu::JSUB; //TODO is correct?

      // Pre R6:
      // $longbr:
      //  addiu $sp, $sp, -8
      //  sw $ra, 0($sp)
      //  lui $at, %hi($tgt - $baltgt)
      //  bal $baltgt
      //  addiu $at, $at, %lo($tgt - $baltgt)
      // $baltgt:
      //  addu $at, $ra, $at
      //  lw $ra, 0($sp)
      //  jr $at
      //  addiu $sp, $sp, 8
      // $fallthrough:
      //

      Pos = LongBrMBB->begin();

      BuildMI(*LongBrMBB, Pos, DL, TII->get(Fgpu::ADDi), Fgpu::SP)
          .addReg(Fgpu::SP)
          .addImm(-8);
      BuildMI(*LongBrMBB, Pos, DL, TII->get(Fgpu::SW))
          .addReg(Fgpu::LR)
          .addReg(Fgpu::SP)
          .addImm(0);

      // LUi and ADDiu instructions create 32-bit offset of the target basic
      // block from the target of BAL(C) instruction.  We cannot use immediate
      // value for this offset because it cannot be determined accurately when
      // the program has inline assembly statements.  We therefore use the
      // relocation expressions %hi($tgt-$baltgt) and %lo($tgt-$baltgt) which
      // are resolved during the fixup, so the values will always be correct.
      //
      // Since we cannot create %hi($tgt-$baltgt) and %lo($tgt-$baltgt)
      // expressions at this point (it is possible only at the MC layer),
      // we replace LUi and ADDiu with pseudo instructions
      // LONG_BRANCH_LUi and LONG_BRANCH_ADDiu, and add both basic
      // blocks as operands to these instructions.  When lowering these pseudo
      // instructions to LUi and ADDiu in the MC layer, we will create
      // %hi($tgt-$baltgt) and %lo($tgt-$baltgt) expressions and add them as
      // operands to lowered instructions.

      BuildMI(*LongBrMBB, Pos, DL, TII->get(Fgpu::LUi), Fgpu::AT)
          .addMBB(TgtMBB, FgpuII::MO_ABS_HI)
          .addMBB(BalTgtMBB);

      MachineInstrBuilder BalInstr =
          BuildMI(*MFp, DL, TII->get(BalOp)).addMBB(BalTgtMBB);
      MachineInstrBuilder ADDiuInstr =
          BuildMI(*MFp, DL, TII->get(Fgpu::Li), Fgpu::AT)
              .addReg(Fgpu::AT)
              .addMBB(TgtMBB, FgpuII::MO_ABS_LO)
              .addMBB(BalTgtMBB);
        LongBrMBB->insert(Pos, ADDiuInstr);
        LongBrMBB->insert(Pos, BalInstr);

      Pos = BalTgtMBB->begin();

      BuildMI(*BalTgtMBB, Pos, DL, TII->get(Fgpu::ADD), Fgpu::AT)
          .addReg(Fgpu::LR)
          .addReg(Fgpu::AT);
      BuildMI(*BalTgtMBB, Pos, DL, TII->get(Fgpu::LW), Fgpu::LR)
          .addReg(Fgpu::SP)
          .addImm(0);

      // In NaCl, modifying the sp is not allowed in branch delay slot.
      // For FGPU32R6, we can skip using a delay slot branch.
      buildProperJumpMI(BalTgtMBB, Pos, DL);
        BuildMI(*BalTgtMBB, std::prev(Pos), DL, TII->get(Fgpu::ADDi), Fgpu::SP)
            .addReg(Fgpu::SP)
            .addImm(8);
  } else { // Not PIC
    Pos = LongBrMBB->begin();
    LongBrMBB->addSuccessor(TgtMBB);

    // Compute the position of the potentiall jump instruction (basic blocks
    // before + 4 for the instruction)
    uint64_t JOffset = computeOffsetFromTheBeginning(MBB->getNumber()) +
                       MBBInfos[MBB->getNumber()].Size + 4;
    uint64_t TgtMBBOffset = computeOffsetFromTheBeginning(TgtMBB->getNumber());
    // If it's a forward jump, then TgtMBBOffset will be shifted by two
    // instructions
    if (JOffset < TgtMBBOffset)
      TgtMBBOffset += 2 * 4;
    // Compare 4 upper bits to check if it's the same segment
    bool SameSegmentJump = JOffset >> 28 == TgtMBBOffset >> 28;

if (SameSegmentJump) {
      // Pre R6:
      // $longbr:
      //  j $tgt
      //  nop
      // $fallthrough:
      //
      assert(false && "I'm tired and this isn't working");
//      MIBundleBuilder(*LongBrMBB, Pos)
//          .append(BuildMI(*MFp, DL, TII->get(Fgpu::B)).addMBB(TgtMBB)) // TODO: check B
//          .append(BuildMI(*MFp, DL, TII->get(Fgpu::NOP)));
    } else {
      // At this point, offset where we need to branch does not fit into
      // immediate field of the branch instruction and is not in the same
      // segment as jump instruction. Therefore we will break it into couple
      // instructions, where we first load the offset into register, and then we
      // do branch register.
      assert(false && "Not supported rn"); // I think this could just be LUi annd Li
//      {
//        BuildMI(*LongBrMBB, Pos, DL, TII->get(Fgpu::LONG_BRANCH_LUi2Op),
//                Fgpu::AT)
//            .addMBB(TgtMBB, FgpuII::MO_ABS_HI);
//        BuildMI(*LongBrMBB, Pos, DL, TII->get(Fgpu::LONG_BRANCH_ADDiu2Op),
//                Fgpu::AT)
//            .addReg(Fgpu::AT)
//            .addMBB(TgtMBB, FgpuII::MO_ABS_LO);
//      }
//      buildProperJumpMI(LongBrMBB, Pos, DL);
    }
  }

  if (I.Br->isUnconditionalBranch()) {
    // Change branch destination.
    assert(I.Br->getDesc().getNumOperands() == 1);
    I.Br->RemoveOperand(0);
    I.Br->addOperand(MachineOperand::CreateMBB(LongBrMBB));
  } else
    // Change branch destination and reverse condition.
    replaceBranch(*MBB, I.Br, DL, &*FallThroughMBB);
}

static void emitGPDisp(MachineFunction &F, const FgpuInstrInfo *TII) {
  MachineBasicBlock &MBB = F.front();
  MachineBasicBlock::iterator I = MBB.begin();
  DebugLoc DL = MBB.findDebugLoc(MBB.begin());
  BuildMI(MBB, I, DL, TII->get(Fgpu::LUi), Fgpu::R2)
      .addExternalSymbol("_gp_disp", FgpuII::MO_ABS_HI);
  BuildMI(MBB, I, DL, TII->get(Fgpu::Li), Fgpu::R3)
      .addExternalSymbol("_gp_disp", FgpuII::MO_ABS_LO);
  BuildMI(MBB, I, DL, TII->get(Fgpu::OR), Fgpu::R3)
      .addReg(Fgpu::R2).addReg(Fgpu::R2).addReg(Fgpu::R3);
  MBB.removeLiveIn(Fgpu::R2);
  MBB.removeLiveIn(Fgpu::R3);
  //TODO: handle this better???
}

bool FgpuBranchExpansion::handleForbiddenSlot() {
  return false;
}

bool FgpuBranchExpansion::handlePossibleLongBranch() {
  if (!STI->enableLongBranchPass())
    return false;

  if (SkipLongBranch)
    return false;

  bool EverMadeChange = false, MadeChange = true;

  while (MadeChange) {
    MadeChange = false;

    initMBBInfo();

    for (unsigned I = 0, E = MBBInfos.size(); I < E; ++I) {
      MachineBasicBlock *MBB = MFp->getBlockNumbered(I);
      // Search for MBB's branch instruction.
      ReverseIter End = MBB->rend();
      ReverseIter Br = getNonDebugInstr(MBB->rbegin(), End);

      if ((Br != End) && Br->isBranch() && !Br->isIndirectBranch() &&
          (Br->isConditionalBranch() ||
           (Br->isUnconditionalBranch() && IsPIC))) {
        int64_t Offset = computeOffset(&*Br);

        if (ForceLongBranchFirstPass ||
            !TII->isBranchOffsetInRange(Br->getOpcode(), Offset)) {
          MBBInfos[I].Offset = Offset;
          MBBInfos[I].Br = &*Br;
        }
      }
    } // End for

    ForceLongBranchFirstPass = false;

    SmallVectorImpl<MBBInfo>::iterator I, E = MBBInfos.end();

    for (I = MBBInfos.begin(); I != E; ++I) {
      // Skip if this MBB doesn't have a branch or the branch has already been
      // converted to a long branch.
      if (!I->Br)
        continue;

      expandToLongBranch(*I);
      ++LongBranches;
      EverMadeChange = MadeChange = true;
    }

    MFp->RenumberBlocks();
  }

  return EverMadeChange;
}

bool FgpuBranchExpansion::runOnMachineFunction(MachineFunction &MF) {
  const TargetMachine &TM = MF.getTarget();
  IsPIC = TM.isPositionIndependent();
  ABI = static_cast<const FgpuTargetMachine &>(TM).getABI();
  STI = &static_cast<const FgpuSubtarget &>(MF.getSubtarget());
  TII = static_cast<const FgpuInstrInfo *>(STI->getInstrInfo());

  if (IsPIC &&
      MF.getInfo<FgpuFunctionInfo>()->globalBaseRegSet())
    emitGPDisp(MF, TII);

  MFp = &MF;

  ForceLongBranchFirstPass = ForceLongBranch;
  // Run these two at least once
  bool longBranchChanged = handlePossibleLongBranch();
  bool forbiddenSlotChanged = handleForbiddenSlot();

  bool Changed = longBranchChanged || forbiddenSlotChanged;

  // Then run them alternatively while there are changes
  while (forbiddenSlotChanged) {
    longBranchChanged = handlePossibleLongBranch();
    if (!longBranchChanged)
      break;
    forbiddenSlotChanged = handleForbiddenSlot();
  }

  return Changed;
}
