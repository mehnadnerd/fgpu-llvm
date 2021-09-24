//===-- FgpuISelDAGToDAG.cpp - A Dag to Dag Inst Selector for Fgpu --------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines an instruction selector for the FGPU target.
//
//===----------------------------------------------------------------------===//

#include "FgpuISelDAGToDAG.h"
#include "MCTargetDesc/FgpuBaseInfo.h"
#include "Fgpu.h"
#include "FgpuMachineFunction.h"
#include "FgpuRegisterInfo.h"
#include "FgpuSEISelDAGToDAG.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/CodeGen/StackProtector.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
using namespace llvm;

#define DEBUG_TYPE "fgpu-isel"

//===----------------------------------------------------------------------===//
// Instruction Selector Implementation
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// FgpuDAGToDAGISel - FGPU specific code to select FGPU machine
// instructions for SelectionDAG operations.
//===----------------------------------------------------------------------===//

void FgpuDAGToDAGISel::getAnalysisUsage(AnalysisUsage &AU) const {
  // There are multiple FgpuDAGToDAGISel instances added to the pass pipeline.
  // We need to preserve StackProtector for the next one.
  AU.addPreserved<StackProtector>();
  SelectionDAGISel::getAnalysisUsage(AU);
}

bool FgpuDAGToDAGISel::runOnMachineFunction(MachineFunction &MF) {
  Subtarget = &static_cast<const FgpuSubtarget &>(MF.getSubtarget());
  bool Ret = SelectionDAGISel::runOnMachineFunction(MF);

  processFunctionAfterISel(MF);

  return Ret;
}

/// getGlobalBaseReg - Output the instructions required to put the
/// GOT address into a register.
SDNode *FgpuDAGToDAGISel::getGlobalBaseReg() {
  Register GlobalBaseReg = MF->getInfo<FgpuFunctionInfo>()->getGlobalBaseReg(*MF);
  return CurDAG->getRegister(GlobalBaseReg, getTargetLowering()->getPointerTy(
                                                CurDAG->getDataLayout()))
      .getNode();
}

/// ComplexPattern used on FgpuInstrInfo
/// Used on Fgpu Load/Store instructions
bool FgpuDAGToDAGISel::selectAddrRegImm(SDValue Addr, SDValue &Base,
                                        SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectAddrDefault(SDValue Addr, SDValue &Base,
                                         SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectIntAddr(SDValue Addr, SDValue &Base,
                                     SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectIntAddr11MM(SDValue Addr, SDValue &Base,
                                       SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectIntAddr12MM(SDValue Addr, SDValue &Base,
                                       SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectIntAddr16MM(SDValue Addr, SDValue &Base,
                                       SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectIntAddrLSL2MM(SDValue Addr, SDValue &Base,
                                           SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectIntAddrSImm10(SDValue Addr, SDValue &Base,
                                           SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectIntAddrSImm10Lsl1(SDValue Addr, SDValue &Base,
                                               SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectIntAddrSImm10Lsl2(SDValue Addr, SDValue &Base,
                                               SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectIntAddrSImm10Lsl3(SDValue Addr, SDValue &Base,
                                               SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectAddr16(SDValue Addr, SDValue &Base,
                                    SDValue &Offset) {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectAddr16SP(SDValue Addr, SDValue &Base,
                                      SDValue &Offset) {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectVSplat(SDNode *N, APInt &Imm,
                                    unsigned MinSizeInBits) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectVSplatUimm1(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectVSplatUimm2(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectVSplatUimm3(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectVSplatUimm4(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectVSplatUimm5(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectVSplatUimm6(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectVSplatUimm8(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectVSplatSimm5(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectVSplatUimmPow2(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectVSplatUimmInvPow2(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectVSplatMaskL(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool FgpuDAGToDAGISel::selectVSplatMaskR(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

/// ComplexPattern used on FgpuInstrInfo used on Fgpu Load/Store instructions
bool FgpuDAGToDAGISel::selectFrameAddr(SDNode *Parent, SDValue Addr, SDValue &Base, SDValue &Offset) {
  LLVM_DEBUG(dbgs() << "soubhi: selectFrameAddr entered\n");
  EVT ValTy = Addr.getValueType();
  SDLoc DL(Addr);

  // If Parent is an unaligned f32 load or store, select a (base + index)
  // floating point load/store instruction (luxc1 or suxc1).
  const LSBaseSDNode* LS = 0;

  if (Parent && (LS = dyn_cast<LSBaseSDNode>(Parent))) {
    EVT VT = LS->getMemoryVT();

    if (VT.getSizeInBits() / 8 > LS->getAlignment()) {
      assert(false && "Unaligned loads/stores not supported for this type.");
      if (VT == MVT::f32)
        return false;
    }
  }
  // return false;
  // if Address is FI, get the TargetFrameIndex.
  if (FrameIndexSDNode *FIN = dyn_cast<FrameIndexSDNode>(Addr)) {
    LLVM_DEBUG(dbgs() << "soubhi: FrameIndexSDNode\n");
    Base   = CurDAG->getTargetFrameIndex(FIN->getIndex(), ValTy);
    Offset = CurDAG->getTargetConstant(0, DL, ValTy);
    return true;
  }
  // return false;

  // Addresses of the form FI+const or FI|const
  if (CurDAG->isBaseWithConstantOffset(Addr)) {
    LLVM_DEBUG(dbgs() << "soubhi: isBaseWithConstantOffset\n");
    ConstantSDNode *CN = dyn_cast<ConstantSDNode>(Addr.getOperand(1));
    if (isInt<16>(CN->getSExtValue())) {
      LLVM_DEBUG(dbgs() << "soubhi: is 16bit offset\n");

      // If the first operand is a FI, get the TargetFI Node
      if (FrameIndexSDNode *FIN = dyn_cast<FrameIndexSDNode> (Addr.getOperand(0))) {
        LLVM_DEBUG(dbgs() << "soubhi: operand 0 is a FrameIndex\n");
        Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), ValTy);
        Offset = CurDAG->getTargetConstant(CN->getZExtValue(), DL, ValTy);
        return true;
      }
      // else
      //   Base = Addr.getOperand(0);

    }
  }
  return false;

  Base   = Addr;
  Offset = CurDAG->getTargetConstant(0, DL, ValTy);
  return true;
}

/// Convert vector addition with vector subtraction if that allows to encode
/// constant as an immediate and thus avoid extra 'ldi' instruction.
/// add X, <-1, -1...> --> sub X, <1, 1...>
bool FgpuDAGToDAGISel::selectVecAddAsVecSubIfProfitable(SDNode *Node) {
  assert(Node->getOpcode() == ISD::ADD && "Should only get 'add' here.");

  EVT VT = Node->getValueType(0);
  assert(VT.isVector() && "Should only be called for vectors.");

  SDValue X = Node->getOperand(0);
  SDValue C = Node->getOperand(1);

  auto *BVN = dyn_cast<BuildVectorSDNode>(C);
  if (!BVN)
    return false;

  APInt SplatValue, SplatUndef;
  unsigned SplatBitSize;
  bool HasAnyUndefs;

  if (!BVN->isConstantSplat(SplatValue, SplatUndef, SplatBitSize, HasAnyUndefs,
                            8, false))
    return false;

  auto IsInlineConstant = [](const APInt &Imm) { return Imm.isIntN(5); };

  if (IsInlineConstant(SplatValue))
    return false; // Can already be encoded as an immediate.

  APInt NegSplatValue = 0 - SplatValue;
  if (!IsInlineConstant(NegSplatValue))
    return false; // Even if we negate it it won't help.

  SDLoc DL(Node);

  SDValue NegC = CurDAG->FoldConstantArithmetic(
      ISD::SUB, DL, VT, {CurDAG->getConstant(0, DL, VT), C});
  assert(NegC && "Constant-folding failed!");
  SDValue NewNode = CurDAG->getNode(ISD::SUB, DL, VT, X, NegC);

  ReplaceNode(Node, NewNode.getNode());
  SelectCode(NewNode.getNode());
  return true;
}

/// Select instructions not customized! Used for
/// expanded, promoted and normal instructions
void FgpuDAGToDAGISel::Select(SDNode *Node) {
  unsigned Opcode = Node->getOpcode();

  // If we have a custom node, we already have selected!
  if (Node->isMachineOpcode()) {
    LLVM_DEBUG(errs() << "== "; Node->dump(CurDAG); errs() << "\n");
    Node->setNodeId(-1);
    return;
  }

  // See if subclasses can handle this node.
  if (trySelect(Node))
    return;

  switch(Opcode) {
  default: break;

  case ISD::ADD:
    if (Node->getSimpleValueType(0).isVector() &&
        selectVecAddAsVecSubIfProfitable(Node))
      return;
    break;

  // Get target GOT address.
  case ISD::GLOBAL_OFFSET_TABLE:
    ReplaceNode(Node, getGlobalBaseReg());
    return;

#ifndef NDEBUG
  case ISD::LOAD:
  case ISD::STORE:
    assert((Subtarget->systemSupportsUnalignedAccess() ||
            cast<MemSDNode>(Node)->getMemoryVT().getSizeInBits() / 8 <=
            cast<MemSDNode>(Node)->getAlignment()) &&
           "Unexpected unaligned loads/stores.");
    break;
#endif
  }

  // Select the default instruction
  SelectCode(Node);
}

bool FgpuDAGToDAGISel::
SelectInlineAsmMemoryOperand(const SDValue &Op, unsigned ConstraintID,
                             std::vector<SDValue> &OutOps) {
  // All memory constraints can at least accept raw pointers.
  switch(ConstraintID) {
  default:
    llvm_unreachable("Unexpected asm memory constraint");
  case InlineAsm::Constraint_m:
  case InlineAsm::Constraint_R:
  case InlineAsm::Constraint_ZC:
    OutOps.push_back(Op);
    return false;
  }
  return true;
}
