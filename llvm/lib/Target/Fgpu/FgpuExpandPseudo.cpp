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
    bool expandAtomicCmpSwap(MachineBasicBlock &MBB,
                             MachineBasicBlock::iterator MBBI,
                             MachineBasicBlock::iterator &NextMBBI);
    bool expandAtomicCmpSwapSubword(MachineBasicBlock &MBB,
                                    MachineBasicBlock::iterator MBBI,
                                    MachineBasicBlock::iterator &NextMBBI);

    bool expandAtomicBinOp(MachineBasicBlock &BB,
                           MachineBasicBlock::iterator I,
                           MachineBasicBlock::iterator &NMBBI, unsigned Size);
    bool expandAtomicBinOpSubword(MachineBasicBlock &BB,
                                  MachineBasicBlock::iterator I,
                                  MachineBasicBlock::iterator &NMBBI);

    bool expandMI(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
                  MachineBasicBlock::iterator &NMBB);
    bool expandMBB(MachineBasicBlock &MBB);
   };
  char FgpuExpandPseudo::ID = 0;
}

bool FgpuExpandPseudo::expandAtomicCmpSwapSubword(
    MachineBasicBlock &BB, MachineBasicBlock::iterator I,
    MachineBasicBlock::iterator &NMBBI) {

  MachineFunction *MF = BB.getParent();

  const bool ArePtrs64bit = STI->getABI().ArePtrs64bit();
  DebugLoc DL = I->getDebugLoc();
  unsigned LL, SC;

  unsigned ZERO = Fgpu::ZERO;
  unsigned BNE = Fgpu::BNE;
  unsigned BEQ = Fgpu::BEQ;
  unsigned SEOp =
      I->getOpcode() == Fgpu::ATOMIC_CMP_SWAP_I8_POSTRA ? Fgpu::SEB : Fgpu::SEH;

  LL = STI->hasFgpu32r6() ? (ArePtrs64bit ? Fgpu::LL64_R6 : Fgpu::LL_R6)
                          : (ArePtrs64bit ? Fgpu::LL64 : Fgpu::LL);
  SC = STI->hasFgpu32r6() ? (ArePtrs64bit ? Fgpu::SC64_R6 : Fgpu::SC_R6)
                          : (ArePtrs64bit ? Fgpu::SC64 : Fgpu::SC);

  Register Dest = I->getOperand(0).getReg();
  Register Ptr = I->getOperand(1).getReg();
  Register Mask = I->getOperand(2).getReg();
  Register ShiftCmpVal = I->getOperand(3).getReg();
  Register Mask2 = I->getOperand(4).getReg();
  Register ShiftNewVal = I->getOperand(5).getReg();
  Register ShiftAmnt = I->getOperand(6).getReg();
  Register Scratch = I->getOperand(7).getReg();
  Register Scratch2 = I->getOperand(8).getReg();

  // insert new blocks after the current block
  const BasicBlock *LLVM_BB = BB.getBasicBlock();
  MachineBasicBlock *loop1MBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *loop2MBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *exitMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineFunction::iterator It = ++BB.getIterator();
  MF->insert(It, loop1MBB);
  MF->insert(It, loop2MBB);
  MF->insert(It, sinkMBB);
  MF->insert(It, exitMBB);

  // Transfer the remainder of BB and its successor edges to exitMBB.
  exitMBB->splice(exitMBB->begin(), &BB,
                  std::next(MachineBasicBlock::iterator(I)), BB.end());
  exitMBB->transferSuccessorsAndUpdatePHIs(&BB);

  //  thisMBB:
  //    ...
  //    fallthrough --> loop1MBB
  BB.addSuccessor(loop1MBB, BranchProbability::getOne());
  loop1MBB->addSuccessor(sinkMBB);
  loop1MBB->addSuccessor(loop2MBB);
  loop1MBB->normalizeSuccProbs();
  loop2MBB->addSuccessor(loop1MBB);
  loop2MBB->addSuccessor(sinkMBB);
  loop2MBB->normalizeSuccProbs();
  sinkMBB->addSuccessor(exitMBB, BranchProbability::getOne());

  // loop1MBB:
  //   ll dest, 0(ptr)
  //   and Mask', dest, Mask
  //   bne Mask', ShiftCmpVal, exitMBB
  BuildMI(loop1MBB, DL, TII->get(LL), Scratch).addReg(Ptr).addImm(0);
  BuildMI(loop1MBB, DL, TII->get(Fgpu::AND), Scratch2)
      .addReg(Scratch)
      .addReg(Mask);
  BuildMI(loop1MBB, DL, TII->get(BNE))
    .addReg(Scratch2).addReg(ShiftCmpVal).addMBB(sinkMBB);

  // loop2MBB:
  //   and dest, dest, mask2
  //   or dest, dest, ShiftNewVal
  //   sc dest, dest, 0(ptr)
  //   beq dest, $0, loop1MBB
  BuildMI(loop2MBB, DL, TII->get(Fgpu::AND), Scratch)
      .addReg(Scratch, RegState::Kill)
      .addReg(Mask2);
  BuildMI(loop2MBB, DL, TII->get(Fgpu::OR), Scratch)
      .addReg(Scratch, RegState::Kill)
      .addReg(ShiftNewVal);
  BuildMI(loop2MBB, DL, TII->get(SC), Scratch)
      .addReg(Scratch, RegState::Kill)
      .addReg(Ptr)
      .addImm(0);
  BuildMI(loop2MBB, DL, TII->get(BEQ))
      .addReg(Scratch, RegState::Kill)
      .addReg(ZERO)
      .addMBB(loop1MBB);

  //  sinkMBB:
  //    srl     srlres, Mask', shiftamt
  //    sign_extend dest,srlres
  BuildMI(sinkMBB, DL, TII->get(Fgpu::SRLV), Dest)
      .addReg(Scratch2)
      .addReg(ShiftAmnt);
  if (STI->hasFgpu32r2()) {
    BuildMI(sinkMBB, DL, TII->get(SEOp), Dest).addReg(Dest);
  } else {
    const unsigned ShiftImm =
        I->getOpcode() == Fgpu::ATOMIC_CMP_SWAP_I16_POSTRA ? 16 : 24;
    BuildMI(sinkMBB, DL, TII->get(Fgpu::SLL), Dest)
        .addReg(Dest, RegState::Kill)
        .addImm(ShiftImm);
    BuildMI(sinkMBB, DL, TII->get(Fgpu::SRA), Dest)
        .addReg(Dest, RegState::Kill)
        .addImm(ShiftImm);
  }

  LivePhysRegs LiveRegs;
  computeAndAddLiveIns(LiveRegs, *loop1MBB);
  computeAndAddLiveIns(LiveRegs, *loop2MBB);
  computeAndAddLiveIns(LiveRegs, *sinkMBB);
  computeAndAddLiveIns(LiveRegs, *exitMBB);

  NMBBI = BB.end();
  I->eraseFromParent();
  return true;
}

bool FgpuExpandPseudo::expandAtomicCmpSwap(MachineBasicBlock &BB,
                                           MachineBasicBlock::iterator I,
                                           MachineBasicBlock::iterator &NMBBI) {

  const unsigned Size =
      I->getOpcode() == Fgpu::ATOMIC_CMP_SWAP_I32_POSTRA ? 4 : 8;
  MachineFunction *MF = BB.getParent();

  const bool ArePtrs64bit = STI->getABI().ArePtrs64bit();
  DebugLoc DL = I->getDebugLoc();

  unsigned LL, SC, ZERO, BNE, BEQ, MOVE;

  if (Size == 4) {
    LL = STI->hasFgpu32r6()
             ? (ArePtrs64bit ? Fgpu::LL64_R6 : Fgpu::LL_R6)
             : (ArePtrs64bit ? Fgpu::LL64 : Fgpu::LL);
    SC = STI->hasFgpu32r6()
             ? (ArePtrs64bit ? Fgpu::SC64_R6 : Fgpu::SC_R6)
             : (ArePtrs64bit ? Fgpu::SC64 : Fgpu::SC);
    BNE = Fgpu::BNE;
    BEQ = Fgpu::BEQ;

    ZERO = Fgpu::ZERO;
    MOVE = Fgpu::OR;
  } else {
    LL = STI->hasFgpu64r6() ? Fgpu::LLD_R6 : Fgpu::LLD;
    SC = STI->hasFgpu64r6() ? Fgpu::SCD_R6 : Fgpu::SCD;
    ZERO = Fgpu::ZERO_64;
    BNE = Fgpu::BNE64;
    BEQ = Fgpu::BEQ64;
    MOVE = Fgpu::OR64;
  }

  Register Dest = I->getOperand(0).getReg();
  Register Ptr = I->getOperand(1).getReg();
  Register OldVal = I->getOperand(2).getReg();
  Register NewVal = I->getOperand(3).getReg();
  Register Scratch = I->getOperand(4).getReg();

  // insert new blocks after the current block
  const BasicBlock *LLVM_BB = BB.getBasicBlock();
  MachineBasicBlock *loop1MBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *loop2MBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *exitMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineFunction::iterator It = ++BB.getIterator();
  MF->insert(It, loop1MBB);
  MF->insert(It, loop2MBB);
  MF->insert(It, exitMBB);

  // Transfer the remainder of BB and its successor edges to exitMBB.
  exitMBB->splice(exitMBB->begin(), &BB,
                  std::next(MachineBasicBlock::iterator(I)), BB.end());
  exitMBB->transferSuccessorsAndUpdatePHIs(&BB);

  //  thisMBB:
  //    ...
  //    fallthrough --> loop1MBB
  BB.addSuccessor(loop1MBB, BranchProbability::getOne());
  loop1MBB->addSuccessor(exitMBB);
  loop1MBB->addSuccessor(loop2MBB);
  loop1MBB->normalizeSuccProbs();
  loop2MBB->addSuccessor(loop1MBB);
  loop2MBB->addSuccessor(exitMBB);
  loop2MBB->normalizeSuccProbs();

  // loop1MBB:
  //   ll dest, 0(ptr)
  //   bne dest, oldval, exitMBB
  BuildMI(loop1MBB, DL, TII->get(LL), Dest).addReg(Ptr).addImm(0);
  BuildMI(loop1MBB, DL, TII->get(BNE))
    .addReg(Dest, RegState::Kill).addReg(OldVal).addMBB(exitMBB);

  // loop2MBB:
  //   move scratch, NewVal
  //   sc Scratch, Scratch, 0(ptr)
  //   beq Scratch, $0, loop1MBB
  BuildMI(loop2MBB, DL, TII->get(MOVE), Scratch).addReg(NewVal).addReg(ZERO);
  BuildMI(loop2MBB, DL, TII->get(SC), Scratch)
    .addReg(Scratch).addReg(Ptr).addImm(0);
  BuildMI(loop2MBB, DL, TII->get(BEQ))
    .addReg(Scratch, RegState::Kill).addReg(ZERO).addMBB(loop1MBB);

  LivePhysRegs LiveRegs;
  computeAndAddLiveIns(LiveRegs, *loop1MBB);
  computeAndAddLiveIns(LiveRegs, *loop2MBB);
  computeAndAddLiveIns(LiveRegs, *exitMBB);

  NMBBI = BB.end();
  I->eraseFromParent();
  return true;
}

bool FgpuExpandPseudo::expandAtomicBinOpSubword(
    MachineBasicBlock &BB, MachineBasicBlock::iterator I,
    MachineBasicBlock::iterator &NMBBI) {

  MachineFunction *MF = BB.getParent();

  const bool ArePtrs64bit = STI->getABI().ArePtrs64bit();
  DebugLoc DL = I->getDebugLoc();

  unsigned LL, SC, SLT, SLTu, OR, MOVN, MOVZ, SELNEZ, SELEQZ;
  unsigned BEQ = Fgpu::BEQ;
  unsigned SEOp = Fgpu::SEH;

  LL = STI->hasFgpu32r6() ? (ArePtrs64bit ? Fgpu::LL64_R6 : Fgpu::LL_R6)
                          : (ArePtrs64bit ? Fgpu::LL64 : Fgpu::LL);
  SC = STI->hasFgpu32r6() ? (ArePtrs64bit ? Fgpu::SC64_R6 : Fgpu::SC_R6)
                          : (ArePtrs64bit ? Fgpu::SC64 : Fgpu::SC);
  SLT = Fgpu::SLT;
  SLTu = Fgpu::SLTu;
  OR = Fgpu::OR;
  MOVN = Fgpu::MOVN_I_I;
  MOVZ = Fgpu::MOVZ_I_I;
  SELNEZ = Fgpu::SELNEZ;
  SELEQZ = Fgpu::SELEQZ;

  bool IsSwap = false;
  bool IsNand = false;
  bool IsMin = false;
  bool IsMax = false;
  bool IsUnsigned = false;

  unsigned Opcode = 0;
  switch (I->getOpcode()) {
  case Fgpu::ATOMIC_LOAD_NAND_I8_POSTRA:
    SEOp = Fgpu::SEB;
    LLVM_FALLTHROUGH;
  case Fgpu::ATOMIC_LOAD_NAND_I16_POSTRA:
    IsNand = true;
    break;
  case Fgpu::ATOMIC_SWAP_I8_POSTRA:
    SEOp = Fgpu::SEB;
    LLVM_FALLTHROUGH;
  case Fgpu::ATOMIC_SWAP_I16_POSTRA:
    IsSwap = true;
    break;
  case Fgpu::ATOMIC_LOAD_ADD_I8_POSTRA:
    SEOp = Fgpu::SEB;
    LLVM_FALLTHROUGH;
  case Fgpu::ATOMIC_LOAD_ADD_I16_POSTRA:
    Opcode = Fgpu::ADDu;
    break;
  case Fgpu::ATOMIC_LOAD_SUB_I8_POSTRA:
    SEOp = Fgpu::SEB;
    LLVM_FALLTHROUGH;
  case Fgpu::ATOMIC_LOAD_SUB_I16_POSTRA:
    Opcode = Fgpu::SUBu;
    break;
  case Fgpu::ATOMIC_LOAD_AND_I8_POSTRA:
    SEOp = Fgpu::SEB;
    LLVM_FALLTHROUGH;
  case Fgpu::ATOMIC_LOAD_AND_I16_POSTRA:
    Opcode = Fgpu::AND;
    break;
  case Fgpu::ATOMIC_LOAD_OR_I8_POSTRA:
    SEOp = Fgpu::SEB;
    LLVM_FALLTHROUGH;
  case Fgpu::ATOMIC_LOAD_OR_I16_POSTRA:
    Opcode = Fgpu::OR;
    break;
  case Fgpu::ATOMIC_LOAD_XOR_I8_POSTRA:
    SEOp = Fgpu::SEB;
    LLVM_FALLTHROUGH;
  case Fgpu::ATOMIC_LOAD_XOR_I16_POSTRA:
    Opcode = Fgpu::XOR;
    break;
  case Fgpu::ATOMIC_LOAD_UMIN_I8_POSTRA:
  case Fgpu::ATOMIC_LOAD_UMIN_I16_POSTRA:
    IsUnsigned = true;
    LLVM_FALLTHROUGH;
  case Fgpu::ATOMIC_LOAD_MIN_I8_POSTRA:
  case Fgpu::ATOMIC_LOAD_MIN_I16_POSTRA:
    IsMin = true;
    break;
  case Fgpu::ATOMIC_LOAD_UMAX_I8_POSTRA:
  case Fgpu::ATOMIC_LOAD_UMAX_I16_POSTRA:
    IsUnsigned = true;
    LLVM_FALLTHROUGH;
  case Fgpu::ATOMIC_LOAD_MAX_I8_POSTRA:
  case Fgpu::ATOMIC_LOAD_MAX_I16_POSTRA:
    IsMax = true;
    break;
  default:
    llvm_unreachable("Unknown subword atomic pseudo for expansion!");
  }

  Register Dest = I->getOperand(0).getReg();
  Register Ptr = I->getOperand(1).getReg();
  Register Incr = I->getOperand(2).getReg();
  Register Mask = I->getOperand(3).getReg();
  Register Mask2 = I->getOperand(4).getReg();
  Register ShiftAmnt = I->getOperand(5).getReg();
  Register OldVal = I->getOperand(6).getReg();
  Register BinOpRes = I->getOperand(7).getReg();
  Register StoreVal = I->getOperand(8).getReg();

  const BasicBlock *LLVM_BB = BB.getBasicBlock();
  MachineBasicBlock *loopMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *exitMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineFunction::iterator It = ++BB.getIterator();
  MF->insert(It, loopMBB);
  MF->insert(It, sinkMBB);
  MF->insert(It, exitMBB);

  exitMBB->splice(exitMBB->begin(), &BB, std::next(I), BB.end());
  exitMBB->transferSuccessorsAndUpdatePHIs(&BB);

  BB.addSuccessor(loopMBB, BranchProbability::getOne());
  loopMBB->addSuccessor(sinkMBB);
  loopMBB->addSuccessor(loopMBB);
  loopMBB->normalizeSuccProbs();

  BuildMI(loopMBB, DL, TII->get(LL), OldVal).addReg(Ptr).addImm(0);
  if (IsNand) {
    //  and andres, oldval, incr2
    //  nor binopres, $0, andres
    //  and newval, binopres, mask
    BuildMI(loopMBB, DL, TII->get(Fgpu::AND), BinOpRes)
        .addReg(OldVal)
        .addReg(Incr);
    BuildMI(loopMBB, DL, TII->get(Fgpu::NOR), BinOpRes)
        .addReg(Fgpu::ZERO)
        .addReg(BinOpRes);
    BuildMI(loopMBB, DL, TII->get(Fgpu::AND), BinOpRes)
        .addReg(BinOpRes)
        .addReg(Mask);
  } else if (IsMin || IsMax) {

    assert(I->getNumOperands() == 10 &&
           "Atomics min|max|umin|umax use an additional register");
    Register Scratch4 = I->getOperand(9).getReg();

    unsigned SLTScratch4 = IsUnsigned ? SLTu : SLT;
    unsigned SELIncr = IsMax ? SELNEZ : SELEQZ;
    unsigned SELOldVal = IsMax ? SELEQZ : SELNEZ;
    unsigned MOVIncr = IsMax ? MOVN : MOVZ;

    // For little endian we need to clear uninterested bits.
    if (STI->isLittle()) {
      // and OldVal, OldVal, Mask
      // and Incr, Incr, Mask
      BuildMI(loopMBB, DL, TII->get(Fgpu::AND), OldVal)
          .addReg(OldVal)
          .addReg(Mask);
      BuildMI(loopMBB, DL, TII->get(Fgpu::AND), Incr).addReg(Incr).addReg(Mask);
    }

    // unsigned: sltu Scratch4, oldVal, Incr
    // signed:   slt Scratch4, oldVal, Incr
    BuildMI(loopMBB, DL, TII->get(SLTScratch4), Scratch4)
        .addReg(OldVal)
        .addReg(Incr);

    if (STI->hasFgpu64r6() || STI->hasFgpu32r6()) {
      // max: seleqz BinOpRes, OldVal, Scratch4
      //      selnez Scratch4, Incr, Scratch4
      //      or BinOpRes, BinOpRes, Scratch4
      // min: selnqz BinOpRes, OldVal, Scratch4
      //      seleqz Scratch4, Incr, Scratch4
      //      or BinOpRes, BinOpRes, Scratch4
      BuildMI(loopMBB, DL, TII->get(SELOldVal), BinOpRes)
          .addReg(OldVal)
          .addReg(Scratch4);
      BuildMI(loopMBB, DL, TII->get(SELIncr), Scratch4)
          .addReg(Incr)
          .addReg(Scratch4);
      BuildMI(loopMBB, DL, TII->get(OR), BinOpRes)
          .addReg(BinOpRes)
          .addReg(Scratch4);
    } else {
      // max: move BinOpRes, OldVal
      //      movn BinOpRes, Incr, Scratch4, BinOpRes
      // min: move BinOpRes, OldVal
      //      movz BinOpRes, Incr, Scratch4, BinOpRes
      BuildMI(loopMBB, DL, TII->get(OR), BinOpRes)
          .addReg(OldVal)
          .addReg(Fgpu::ZERO);
      BuildMI(loopMBB, DL, TII->get(MOVIncr), BinOpRes)
          .addReg(Incr)
          .addReg(Scratch4)
          .addReg(BinOpRes);
    }

    //  and BinOpRes, BinOpRes, Mask
    BuildMI(loopMBB, DL, TII->get(Fgpu::AND), BinOpRes)
        .addReg(BinOpRes)
        .addReg(Mask);

  } else if (!IsSwap) {
    //  <binop> binopres, oldval, incr2
    //  and newval, binopres, mask
    BuildMI(loopMBB, DL, TII->get(Opcode), BinOpRes)
        .addReg(OldVal)
        .addReg(Incr);
    BuildMI(loopMBB, DL, TII->get(Fgpu::AND), BinOpRes)
        .addReg(BinOpRes)
        .addReg(Mask);
  } else { // atomic.swap
    //  and newval, incr2, mask
    BuildMI(loopMBB, DL, TII->get(Fgpu::AND), BinOpRes)
        .addReg(Incr)
        .addReg(Mask);
  }

  // and StoreVal, OlddVal, Mask2
  // or StoreVal, StoreVal, BinOpRes
  // StoreVal<tied1> = sc StoreVal, 0(Ptr)
  // beq StoreVal, zero, loopMBB
  BuildMI(loopMBB, DL, TII->get(Fgpu::AND), StoreVal)
    .addReg(OldVal).addReg(Mask2);
  BuildMI(loopMBB, DL, TII->get(Fgpu::OR), StoreVal)
    .addReg(StoreVal).addReg(BinOpRes);
  BuildMI(loopMBB, DL, TII->get(SC), StoreVal)
    .addReg(StoreVal).addReg(Ptr).addImm(0);
  BuildMI(loopMBB, DL, TII->get(BEQ))
    .addReg(StoreVal).addReg(Fgpu::ZERO).addMBB(loopMBB);

  //  sinkMBB:
  //    and     maskedoldval1,oldval,mask
  //    srl     srlres,maskedoldval1,shiftamt
  //    sign_extend dest,srlres

  sinkMBB->addSuccessor(exitMBB, BranchProbability::getOne());

  BuildMI(sinkMBB, DL, TII->get(Fgpu::AND), Dest)
    .addReg(OldVal).addReg(Mask);
  BuildMI(sinkMBB, DL, TII->get(Fgpu::SRLV), Dest)
      .addReg(Dest).addReg(ShiftAmnt);

  if (STI->hasFgpu32r2()) {
    BuildMI(sinkMBB, DL, TII->get(SEOp), Dest).addReg(Dest);
  } else {
    const unsigned ShiftImm = SEOp == Fgpu::SEH ? 16 : 24;
    BuildMI(sinkMBB, DL, TII->get(Fgpu::SLL), Dest)
        .addReg(Dest, RegState::Kill)
        .addImm(ShiftImm);
    BuildMI(sinkMBB, DL, TII->get(Fgpu::SRA), Dest)
        .addReg(Dest, RegState::Kill)
        .addImm(ShiftImm);
  }

  LivePhysRegs LiveRegs;
  computeAndAddLiveIns(LiveRegs, *loopMBB);
  computeAndAddLiveIns(LiveRegs, *sinkMBB);
  computeAndAddLiveIns(LiveRegs, *exitMBB);

  NMBBI = BB.end();
  I->eraseFromParent();

  return true;
}

bool FgpuExpandPseudo::expandAtomicBinOp(MachineBasicBlock &BB,
                                         MachineBasicBlock::iterator I,
                                         MachineBasicBlock::iterator &NMBBI,
                                         unsigned Size) {
  MachineFunction *MF = BB.getParent();

  const bool ArePtrs64bit = STI->getABI().ArePtrs64bit();
  DebugLoc DL = I->getDebugLoc();

  unsigned LL, SC, ZERO, BEQ, SLT, SLTu, OR, MOVN, MOVZ, SELNEZ, SELEQZ;

  if (Size == 4) {
    LL = STI->hasFgpu32r6()
             ? (ArePtrs64bit ? Fgpu::LL64_R6 : Fgpu::LL_R6)
             : (ArePtrs64bit ? Fgpu::LL64 : Fgpu::LL);
    SC = STI->hasFgpu32r6()
             ? (ArePtrs64bit ? Fgpu::SC64_R6 : Fgpu::SC_R6)
             : (ArePtrs64bit ? Fgpu::SC64 : Fgpu::SC);
    BEQ = Fgpu::BEQ;
    SLT = Fgpu::SLT;
    SLTu = Fgpu::SLTu;
    OR = Fgpu::OR;
    MOVN = Fgpu::MOVN_I_I;
    MOVZ = Fgpu::MOVZ_I_I;
    SELNEZ = Fgpu::SELNEZ;
    SELEQZ = Fgpu::SELEQZ;

    ZERO = Fgpu::ZERO;
  } else {
    LL = STI->hasFgpu64r6() ? Fgpu::LLD_R6 : Fgpu::LLD;
    SC = STI->hasFgpu64r6() ? Fgpu::SCD_R6 : Fgpu::SCD;
    ZERO = Fgpu::ZERO_64;
    BEQ = Fgpu::BEQ64;
    SLT = Fgpu::SLT64;
    SLTu = Fgpu::SLTu64;
    OR = Fgpu::OR64;
    MOVN = Fgpu::MOVN_I64_I64;
    MOVZ = Fgpu::MOVZ_I64_I64;
    SELNEZ = Fgpu::SELNEZ64;
    SELEQZ = Fgpu::SELEQZ64;
  }

  Register OldVal = I->getOperand(0).getReg();
  Register Ptr = I->getOperand(1).getReg();
  Register Incr = I->getOperand(2).getReg();
  Register Scratch = I->getOperand(3).getReg();

  unsigned Opcode = 0;
  unsigned AND = 0;
  unsigned NOR = 0;

  bool IsOr = false;
  bool IsNand = false;
  bool IsMin = false;
  bool IsMax = false;
  bool IsUnsigned = false;

  switch (I->getOpcode()) {
  case Fgpu::ATOMIC_LOAD_ADD_I32_POSTRA:
    Opcode = Fgpu::ADDu;
    break;
  case Fgpu::ATOMIC_LOAD_SUB_I32_POSTRA:
    Opcode = Fgpu::SUBu;
    break;
  case Fgpu::ATOMIC_LOAD_AND_I32_POSTRA:
    Opcode = Fgpu::AND;
    break;
  case Fgpu::ATOMIC_LOAD_OR_I32_POSTRA:
    Opcode = Fgpu::OR;
    break;
  case Fgpu::ATOMIC_LOAD_XOR_I32_POSTRA:
    Opcode = Fgpu::XOR;
    break;
  case Fgpu::ATOMIC_LOAD_NAND_I32_POSTRA:
    IsNand = true;
    AND = Fgpu::AND;
    NOR = Fgpu::NOR;
    break;
  case Fgpu::ATOMIC_SWAP_I32_POSTRA:
    IsOr = true;
    break;
  case Fgpu::ATOMIC_LOAD_ADD_I64_POSTRA:
    Opcode = Fgpu::DADDu;
    break;
  case Fgpu::ATOMIC_LOAD_SUB_I64_POSTRA:
    Opcode = Fgpu::DSUBu;
    break;
  case Fgpu::ATOMIC_LOAD_AND_I64_POSTRA:
    Opcode = Fgpu::AND64;
    break;
  case Fgpu::ATOMIC_LOAD_OR_I64_POSTRA:
    Opcode = Fgpu::OR64;
    break;
  case Fgpu::ATOMIC_LOAD_XOR_I64_POSTRA:
    Opcode = Fgpu::XOR64;
    break;
  case Fgpu::ATOMIC_LOAD_NAND_I64_POSTRA:
    IsNand = true;
    AND = Fgpu::AND64;
    NOR = Fgpu::NOR64;
    break;
  case Fgpu::ATOMIC_SWAP_I64_POSTRA:
    IsOr = true;
    break;
  case Fgpu::ATOMIC_LOAD_UMIN_I32_POSTRA:
  case Fgpu::ATOMIC_LOAD_UMIN_I64_POSTRA:
    IsUnsigned = true;
    LLVM_FALLTHROUGH;
  case Fgpu::ATOMIC_LOAD_MIN_I32_POSTRA:
  case Fgpu::ATOMIC_LOAD_MIN_I64_POSTRA:
    IsMin = true;
    break;
  case Fgpu::ATOMIC_LOAD_UMAX_I32_POSTRA:
  case Fgpu::ATOMIC_LOAD_UMAX_I64_POSTRA:
    IsUnsigned = true;
    LLVM_FALLTHROUGH;
  case Fgpu::ATOMIC_LOAD_MAX_I32_POSTRA:
  case Fgpu::ATOMIC_LOAD_MAX_I64_POSTRA:
    IsMax = true;
    break;
  default:
    llvm_unreachable("Unknown pseudo atomic!");
  }

  const BasicBlock *LLVM_BB = BB.getBasicBlock();
  MachineBasicBlock *loopMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *exitMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineFunction::iterator It = ++BB.getIterator();
  MF->insert(It, loopMBB);
  MF->insert(It, exitMBB);

  exitMBB->splice(exitMBB->begin(), &BB, std::next(I), BB.end());
  exitMBB->transferSuccessorsAndUpdatePHIs(&BB);

  BB.addSuccessor(loopMBB, BranchProbability::getOne());
  loopMBB->addSuccessor(exitMBB);
  loopMBB->addSuccessor(loopMBB);
  loopMBB->normalizeSuccProbs();

  BuildMI(loopMBB, DL, TII->get(LL), OldVal).addReg(Ptr).addImm(0);
  assert((OldVal != Ptr) && "Clobbered the wrong ptr reg!");
  assert((OldVal != Incr) && "Clobbered the wrong reg!");
  if (IsMin || IsMax) {

    assert(I->getNumOperands() == 5 &&
           "Atomics min|max|umin|umax use an additional register");
    MCRegister Scratch2 = I->getOperand(4).getReg().asMCReg();

    // On Fgpu64 result of slt is GPR32.
    MCRegister Scratch2_32 =
        (Size == 8) ? STI->getRegisterInfo()->getSubReg(Scratch2, Fgpu::sub_32)
                    : Scratch2;

    unsigned SLTScratch2 = IsUnsigned ? SLTu : SLT;
    unsigned SELIncr = IsMax ? SELNEZ : SELEQZ;
    unsigned SELOldVal = IsMax ? SELEQZ : SELNEZ;
    unsigned MOVIncr = IsMax ? MOVN : MOVZ;

    // unsigned: sltu Scratch2, oldVal, Incr
    // signed:   slt Scratch2, oldVal, Incr
    BuildMI(loopMBB, DL, TII->get(SLTScratch2), Scratch2_32)
        .addReg(OldVal)
        .addReg(Incr);

    if (STI->hasFgpu64r6() || STI->hasFgpu32r6()) {
      // max: seleqz Scratch, OldVal, Scratch2
      //      selnez Scratch2, Incr, Scratch2
      //      or Scratch, Scratch, Scratch2
      // min: selnez Scratch, OldVal, Scratch2
      //      seleqz Scratch2, Incr, Scratch2
      //      or Scratch, Scratch, Scratch2
      BuildMI(loopMBB, DL, TII->get(SELOldVal), Scratch)
          .addReg(OldVal)
          .addReg(Scratch2);
      BuildMI(loopMBB, DL, TII->get(SELIncr), Scratch2)
          .addReg(Incr)
          .addReg(Scratch2);
      BuildMI(loopMBB, DL, TII->get(OR), Scratch)
          .addReg(Scratch)
          .addReg(Scratch2);
    } else {
      // max: move Scratch, OldVal
      //      movn Scratch, Incr, Scratch2, Scratch
      // min: move Scratch, OldVal
      //      movz Scratch, Incr, Scratch2, Scratch
      BuildMI(loopMBB, DL, TII->get(OR), Scratch)
          .addReg(OldVal)
          .addReg(ZERO);
      BuildMI(loopMBB, DL, TII->get(MOVIncr), Scratch)
          .addReg(Incr)
          .addReg(Scratch2)
          .addReg(Scratch);
    }

  } else if (Opcode) {
    BuildMI(loopMBB, DL, TII->get(Opcode), Scratch).addReg(OldVal).addReg(Incr);
  } else if (IsNand) {
    assert(AND && NOR &&
           "Unknown nand instruction for atomic pseudo expansion");
    BuildMI(loopMBB, DL, TII->get(AND), Scratch).addReg(OldVal).addReg(Incr);
    BuildMI(loopMBB, DL, TII->get(NOR), Scratch).addReg(ZERO).addReg(Scratch);
  } else {
    assert(IsOr && OR && "Unknown instruction for atomic pseudo expansion!");
    (void)IsOr;
    BuildMI(loopMBB, DL, TII->get(OR), Scratch).addReg(Incr).addReg(ZERO);
  }

  BuildMI(loopMBB, DL, TII->get(SC), Scratch)
      .addReg(Scratch)
      .addReg(Ptr)
      .addImm(0);
  BuildMI(loopMBB, DL, TII->get(BEQ))
      .addReg(Scratch)
      .addReg(ZERO)
      .addMBB(loopMBB);

  NMBBI = BB.end();
  I->eraseFromParent();

  LivePhysRegs LiveRegs;
  computeAndAddLiveIns(LiveRegs, *loopMBB);
  computeAndAddLiveIns(LiveRegs, *exitMBB);

  return true;
}

bool FgpuExpandPseudo::expandMI(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MBBI,
                                MachineBasicBlock::iterator &NMBB) {

  bool Modified = false;

  switch (MBBI->getOpcode()) {
  case Fgpu::ATOMIC_CMP_SWAP_I32_POSTRA:
  case Fgpu::ATOMIC_CMP_SWAP_I64_POSTRA:
    return expandAtomicCmpSwap(MBB, MBBI, NMBB);
  case Fgpu::ATOMIC_CMP_SWAP_I8_POSTRA:
  case Fgpu::ATOMIC_CMP_SWAP_I16_POSTRA:
    return expandAtomicCmpSwapSubword(MBB, MBBI, NMBB);
  case Fgpu::ATOMIC_SWAP_I8_POSTRA:
  case Fgpu::ATOMIC_SWAP_I16_POSTRA:
  case Fgpu::ATOMIC_LOAD_NAND_I8_POSTRA:
  case Fgpu::ATOMIC_LOAD_NAND_I16_POSTRA:
  case Fgpu::ATOMIC_LOAD_ADD_I8_POSTRA:
  case Fgpu::ATOMIC_LOAD_ADD_I16_POSTRA:
  case Fgpu::ATOMIC_LOAD_SUB_I8_POSTRA:
  case Fgpu::ATOMIC_LOAD_SUB_I16_POSTRA:
  case Fgpu::ATOMIC_LOAD_AND_I8_POSTRA:
  case Fgpu::ATOMIC_LOAD_AND_I16_POSTRA:
  case Fgpu::ATOMIC_LOAD_OR_I8_POSTRA:
  case Fgpu::ATOMIC_LOAD_OR_I16_POSTRA:
  case Fgpu::ATOMIC_LOAD_XOR_I8_POSTRA:
  case Fgpu::ATOMIC_LOAD_XOR_I16_POSTRA:
  case Fgpu::ATOMIC_LOAD_MIN_I8_POSTRA:
  case Fgpu::ATOMIC_LOAD_MIN_I16_POSTRA:
  case Fgpu::ATOMIC_LOAD_MAX_I8_POSTRA:
  case Fgpu::ATOMIC_LOAD_MAX_I16_POSTRA:
  case Fgpu::ATOMIC_LOAD_UMIN_I8_POSTRA:
  case Fgpu::ATOMIC_LOAD_UMIN_I16_POSTRA:
  case Fgpu::ATOMIC_LOAD_UMAX_I8_POSTRA:
  case Fgpu::ATOMIC_LOAD_UMAX_I16_POSTRA:
    return expandAtomicBinOpSubword(MBB, MBBI, NMBB);
  case Fgpu::ATOMIC_LOAD_ADD_I32_POSTRA:
  case Fgpu::ATOMIC_LOAD_SUB_I32_POSTRA:
  case Fgpu::ATOMIC_LOAD_AND_I32_POSTRA:
  case Fgpu::ATOMIC_LOAD_OR_I32_POSTRA:
  case Fgpu::ATOMIC_LOAD_XOR_I32_POSTRA:
  case Fgpu::ATOMIC_LOAD_NAND_I32_POSTRA:
  case Fgpu::ATOMIC_SWAP_I32_POSTRA:
  case Fgpu::ATOMIC_LOAD_MIN_I32_POSTRA:
  case Fgpu::ATOMIC_LOAD_MAX_I32_POSTRA:
  case Fgpu::ATOMIC_LOAD_UMIN_I32_POSTRA:
  case Fgpu::ATOMIC_LOAD_UMAX_I32_POSTRA:
    return expandAtomicBinOp(MBB, MBBI, NMBB, 4);
  case Fgpu::ATOMIC_LOAD_ADD_I64_POSTRA:
  case Fgpu::ATOMIC_LOAD_SUB_I64_POSTRA:
  case Fgpu::ATOMIC_LOAD_AND_I64_POSTRA:
  case Fgpu::ATOMIC_LOAD_OR_I64_POSTRA:
  case Fgpu::ATOMIC_LOAD_XOR_I64_POSTRA:
  case Fgpu::ATOMIC_LOAD_NAND_I64_POSTRA:
  case Fgpu::ATOMIC_SWAP_I64_POSTRA:
  case Fgpu::ATOMIC_LOAD_MIN_I64_POSTRA:
  case Fgpu::ATOMIC_LOAD_MAX_I64_POSTRA:
  case Fgpu::ATOMIC_LOAD_UMIN_I64_POSTRA:
  case Fgpu::ATOMIC_LOAD_UMAX_I64_POSTRA:
    return expandAtomicBinOp(MBB, MBBI, NMBB, 8);
  default:
    return Modified;
  }
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
