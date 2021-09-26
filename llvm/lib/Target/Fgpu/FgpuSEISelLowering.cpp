//===- FgpuSEISelLowering.cpp - FgpuSE DAG Lowering Interface -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Subclass of FgpuTargetLowering specialized for fgpu32/64.
//
//===----------------------------------------------------------------------===//

#include "FgpuSEISelLowering.h"
#include "FgpuMachineFunction.h"
#include "FgpuRegisterInfo.h"
#include "FgpuSubtarget.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Triple.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/ISDOpcodes.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/IR/IntrinsicsFgpu.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MachineValueType.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iterator>
#include <utility>

using namespace llvm;

#define DEBUG_TYPE "fgpu-isel"

static cl::opt<bool>
UseFgpuTailCalls("fgpu-tail-calls", cl::Hidden,
                    cl::desc("FGPU: permit tail calls."), cl::init(false));

static cl::opt<bool> NoDPLoadStore("mno-ldc1-sdc1", cl::init(false),
                                   cl::desc("Expand double precision loads and "
                                            "stores to their single precision "
                                            "counterparts"));

FgpuSETargetLowering::FgpuSETargetLowering(const FgpuTargetMachine &TM,
                                           const FgpuSubtarget &STI)
    : FgpuTargetLowering(TM, STI) {
  // Set up the register classes
  addRegisterClass(MVT::i32, &Fgpu::GPROutRegClass);

//  if (!Subtarget.useSoftFloat()) { // TODO: vector support
//    addRegisterClass(MVT::f32, &Fgpu::FGR32RegClass);
//
//    // When dealing with single precision only, use libcalls
//    if (!Subtarget.isSingleFloat()) {
//      if (Subtarget.isFP64bit())
//        addRegisterClass(MVT::f64, &Fgpu::FGR64RegClass);
//      else
//        addRegisterClass(MVT::f64, &Fgpu::AFGR64RegClass);
//    }
//  }

  setOperationAction(ISD::SMUL_LOHI,          MVT::i32, Custom);
  setOperationAction(ISD::UMUL_LOHI,          MVT::i32, Custom);
  setOperationAction(ISD::MULHS,              MVT::i32, Custom);
  setOperationAction(ISD::MULHU,              MVT::i32, Custom);


  setOperationAction(ISD::INTRINSIC_WO_CHAIN, MVT::i64, Custom);
  setOperationAction(ISD::INTRINSIC_W_CHAIN,  MVT::i64, Custom);

  setOperationAction(ISD::SDIVREM, MVT::i32, Custom);
  setOperationAction(ISD::UDIVREM, MVT::i32, Custom);
  setOperationAction(ISD::ATOMIC_FENCE,       MVT::Other, Custom);
  setOperationAction(ISD::LOAD,               MVT::i32, Custom);
  setOperationAction(ISD::STORE,              MVT::i32, Custom);

  setTargetDAGCombine(ISD::MUL);

  setOperationAction(ISD::INTRINSIC_WO_CHAIN, MVT::Other, Custom);
  setOperationAction(ISD::INTRINSIC_W_CHAIN, MVT::Other, Custom);
  setOperationAction(ISD::INTRINSIC_VOID, MVT::Other, Custom);

  if (NoDPLoadStore) {
    setOperationAction(ISD::LOAD, MVT::f64, Custom);
    setOperationAction(ISD::STORE, MVT::f64, Custom);
  }

  setOperationAction(ISD::MUL, MVT::i32, Legal);
  // TODO: MADD

  computeRegisterProperties(Subtarget.getRegisterInfo());
}

const FgpuTargetLowering *
llvm::createFgpuSETargetLowering(const FgpuTargetMachine &TM,
                                 const FgpuSubtarget &STI) {
  return new FgpuSETargetLowering(TM, STI);
}

const TargetRegisterClass *
FgpuSETargetLowering::getRepRegClassFor(MVT VT) const {
  return TargetLowering::getRepRegClassFor(VT);
}

SDValue FgpuSETargetLowering::lowerSELECT(SDValue Op, SelectionDAG &DAG) const {
  return FgpuTargetLowering::LowerOperation(Op, DAG);

//  EVT ResTy = Op->getValueType(0); // TODO: add MOVN/MOVZ support
//  SDLoc DL(Op);
//
//  // Although MTC1_D64 takes an i32 and writes an f64, the upper 32 bits of the
//  // floating point register are undefined. Not really an issue as sel.d, which
//  // is produced from an FSELECT node, only looks at bit 0.
//  SDValue Tmp = DAG.getNode(FgpuISD::MTC1_D64, DL, MVT::f64, Op->getOperand(0));
//  return DAG.getNode(FgpuISD::FSELECT, DL, ResTy, Tmp, Op->getOperand(1),
//                     Op->getOperand(2));
}

bool FgpuSETargetLowering::allowsMisalignedMemoryAccesses(
    EVT VT, unsigned, Align, MachineMemOperand::Flags, bool *Fast) const {
  MVT::SimpleValueType SVT = VT.getSimpleVT().SimpleTy;

  if (Subtarget.systemSupportsUnalignedAccess()) {
    // FGPU32r6/FGPU64r6 is required to support unaligned access. It's
    // implementation defined whether this is handled by hardware, software, or
    // a hybrid of the two but it's expected that most implementations will
    // handle the majority of cases in hardware.
    if (Fast)
      *Fast = false;
    return false;
  }

  switch (SVT) {
  case MVT::i64:
  case MVT::i32:
    if (Fast)
      *Fast = true;
    return false;
  default:
    return false;
  }
}

SDValue FgpuSETargetLowering::LowerOperation(SDValue Op,
                                             SelectionDAG &DAG) const {
  switch(Op.getOpcode()) {
  case ISD::LOAD:  return lowerLOAD(Op, DAG);
  case ISD::STORE: return lowerSTORE(Op, DAG);
//  case ISD::MUL:       return lowerMulDiv(Op, FgpuISD::Mult, true, false, DAG);
  case ISD::INTRINSIC_WO_CHAIN: return lowerINTRINSIC_WO_CHAIN(Op, DAG);
  case ISD::INTRINSIC_W_CHAIN:  return lowerINTRINSIC_W_CHAIN(Op, DAG);
  case ISD::INTRINSIC_VOID:     return lowerINTRINSIC_VOID(Op, DAG);
  case ISD::EXTRACT_VECTOR_ELT: return lowerEXTRACT_VECTOR_ELT(Op, DAG);
  case ISD::BUILD_VECTOR:       return lowerBUILD_VECTOR(Op, DAG);
  case ISD::VECTOR_SHUFFLE:     return lowerVECTOR_SHUFFLE(Op, DAG);
  case ISD::SELECT:             return lowerSELECT(Op, DAG);
  case ISD::BITCAST:            return lowerBITCAST(Op, DAG);
  }

  return FgpuTargetLowering::LowerOperation(Op, DAG);
}

// Determine if the specified node is a constant vector splat.
//
// Returns true and sets Imm if:
// * N is a ISD::BUILD_VECTOR representing a constant splat
//
// This function is quite similar to FgpuSEDAGToDAGISel::selectVSplat. The
// differences are that it assumes the MSA has already been checked and the
// arbitrary requirement for a maximum of 32-bit integers isn't applied (and
// must not be in order for binsri.d to be selectable).
static bool isVSplat(SDValue N, APInt &Imm, bool IsLittleEndian) {
  BuildVectorSDNode *Node = dyn_cast<BuildVectorSDNode>(N.getNode());

  if (!Node)
    return false;

  APInt SplatValue, SplatUndef;
  unsigned SplatBitSize;
  bool HasAnyUndefs;

  if (!Node->isConstantSplat(SplatValue, SplatUndef, SplatBitSize, HasAnyUndefs,
                             8, !IsLittleEndian))
    return false;

  Imm = SplatValue;

  return true;
}

// Test whether the given node is an all-ones build_vector.
static bool isVectorAllOnes(SDValue N) {
  // Look through bitcasts. Endianness doesn't matter because we are looking
  // for an all-ones value.
  if (N->getOpcode() == ISD::BITCAST)
    N = N->getOperand(0);

  BuildVectorSDNode *BVN = dyn_cast<BuildVectorSDNode>(N);

  if (!BVN)
    return false;

  APInt SplatValue, SplatUndef;
  unsigned SplatBitSize;
  bool HasAnyUndefs;

  // Endianness doesn't matter in this context because we are looking for
  // an all-ones value.
  if (BVN->isConstantSplat(SplatValue, SplatUndef, SplatBitSize, HasAnyUndefs))
    return SplatValue.isAllOnesValue();

  return false;
}

// Test whether N is the bitwise inverse of OfNode.
static bool isBitwiseInverse(SDValue N, SDValue OfNode) {
  if (N->getOpcode() != ISD::XOR)
    return false;

  if (isVectorAllOnes(N->getOperand(0)))
    return N->getOperand(1) == OfNode;

  if (isVectorAllOnes(N->getOperand(1)))
    return N->getOperand(0) == OfNode;

  return false;
}

static bool shouldTransformMulToShiftsAddsSubs(APInt C, EVT VT,
                                               SelectionDAG &DAG,
                                               const FgpuSubtarget &Subtarget) {
  // Estimate the number of operations the below transform will turn a
  // constant multiply into. The number is approximately equal to the minimal
  // number of powers of two that constant can be broken down to by adding
  // or subtracting them.
  //
  // If we have taken more than 12[1] / 8[2] steps to attempt the
  // optimization for a native sized value, it is more than likely that this
  // optimization will make things worse.
  //
  // [1] FGPU64 requires 6 instructions at most to materialize any constant,
  //     multiplication requires at least 4 cycles, but another cycle (or two)
  //     to retrieve the result from the HI/LO registers.
  //
  // [2] For FGPU32, more than 8 steps is expensive as the constant could be
  //     materialized in 2 instructions, multiplication requires at least 4
  //     cycles, but another cycle (or two) to retrieve the result from the
  //     HI/LO registers.
  //
  // TODO:
  // - MaxSteps needs to consider the `VT` of the constant for the current
  //   target.
  // - Consider to perform this optimization after type legalization.
  //   That allows to remove a workaround for types not supported natively.
  // - Take in account `-Os, -Oz` flags because this optimization
  //   increases code size.
  unsigned MaxSteps = 8;

  SmallVector<APInt, 16> WorkStack(1, C);
  unsigned Steps = 0;
  unsigned BitWidth = C.getBitWidth();

  while (!WorkStack.empty()) {
    APInt Val = WorkStack.pop_back_val();

    if (Val == 0 || Val == 1)
      continue;

    if (Steps >= MaxSteps)
      return false;

    if (Val.isPowerOf2()) {
      ++Steps;
      continue;
    }

    APInt Floor = APInt(BitWidth, 1) << Val.logBase2();
    APInt Ceil = Val.isNegative() ? APInt(BitWidth, 0)
                                  : APInt(BitWidth, 1) << C.ceilLogBase2();
    if ((Val - Floor).ule(Ceil - Val)) {
      WorkStack.push_back(Floor);
      WorkStack.push_back(Val - Floor);
    } else {
      WorkStack.push_back(Ceil);
      WorkStack.push_back(Ceil - Val);
    }

    ++Steps;
  }

  // If the value being multiplied is not supported natively, we have to pay
  // an additional legalization cost, conservatively assume an increase in the
  // cost of 3 instructions per step. This values for this heuristic were
  // determined experimentally.
  unsigned RegisterSize = DAG.getTargetLoweringInfo()
                              .getRegisterType(*DAG.getContext(), VT)
                              .getSizeInBits();
  Steps *= (VT.getSizeInBits() != RegisterSize) * 3;
  if (Steps > 27)
    return false;

  return true;
}

static SDValue genConstMult(SDValue X, APInt C, const SDLoc &DL, EVT VT,
                            EVT ShiftTy, SelectionDAG &DAG) {
  // Return 0.
  if (C == 0)
    return DAG.getConstant(0, DL, VT);

  // Return x.
  if (C == 1)
    return X;

  // If c is power of 2, return (shl x, log2(c)).
  if (C.isPowerOf2())
    return DAG.getNode(ISD::SHL, DL, VT, X,
                       DAG.getConstant(C.logBase2(), DL, ShiftTy));

  unsigned BitWidth = C.getBitWidth();
  APInt Floor = APInt(BitWidth, 1) << C.logBase2();
  APInt Ceil = C.isNegative() ? APInt(BitWidth, 0) :
                                APInt(BitWidth, 1) << C.ceilLogBase2();

  // If |c - floor_c| <= |c - ceil_c|,
  // where floor_c = pow(2, floor(log2(c))) and ceil_c = pow(2, ceil(log2(c))),
  // return (add constMult(x, floor_c), constMult(x, c - floor_c)).
  if ((C - Floor).ule(Ceil - C)) {
    SDValue Op0 = genConstMult(X, Floor, DL, VT, ShiftTy, DAG);
    SDValue Op1 = genConstMult(X, C - Floor, DL, VT, ShiftTy, DAG);
    return DAG.getNode(ISD::ADD, DL, VT, Op0, Op1);
  }

  // If |c - floor_c| > |c - ceil_c|,
  // return (sub constMult(x, ceil_c), constMult(x, ceil_c - c)).
  SDValue Op0 = genConstMult(X, Ceil, DL, VT, ShiftTy, DAG);
  SDValue Op1 = genConstMult(X, Ceil - C, DL, VT, ShiftTy, DAG);
  return DAG.getNode(ISD::SUB, DL, VT, Op0, Op1);
}

static SDValue performMULCombine(SDNode *N, SelectionDAG &DAG,
                                 const TargetLowering::DAGCombinerInfo &DCI,
                                 const FgpuSETargetLowering *TL,
                                 const FgpuSubtarget &Subtarget) {
  EVT VT = N->getValueType(0);

  if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(N->getOperand(1)))
    if (!VT.isVector() && shouldTransformMulToShiftsAddsSubs(
                              C->getAPIntValue(), VT, DAG, Subtarget))
      return genConstMult(N->getOperand(0), C->getAPIntValue(), SDLoc(N), VT,
                          TL->getScalarShiftAmountTy(DAG.getDataLayout(), VT),
                          DAG);

  return SDValue(N, 0);
}

SDValue
FgpuSETargetLowering::PerformDAGCombine(SDNode *N, DAGCombinerInfo &DCI) const {
  SelectionDAG &DAG = DCI.DAG;
  SDValue Val;

  switch (N->getOpcode()) {
  case ISD::MUL:
    return performMULCombine(N, DAG, DCI, this, Subtarget);
//  case ISD::SHL:
//    Val = performSHLCombine(N, DAG, DCI, Subtarget);
//    break;
  }

  if (Val.getNode()) {
    LLVM_DEBUG(dbgs() << "\nFgpuSE DAG Combine:\n";
               N->printrWithDepth(dbgs(), &DAG); dbgs() << "\n=> \n";
               Val.getNode()->printrWithDepth(dbgs(), &DAG); dbgs() << "\n");
    return Val;
  }

  return FgpuTargetLowering::PerformDAGCombine(N, DCI);
}

MachineBasicBlock *
FgpuSETargetLowering::EmitInstrWithCustomInserter(MachineInstr &MI,
                                                  MachineBasicBlock *BB) const {
  switch (MI.getOpcode()) {
  default:
    return FgpuTargetLowering::EmitInstrWithCustomInserter(MI, BB);
//  case Fgpu::MSA_FP_ROUND_D_PSEUDO:
//    return emitFPROUND_PSEUDO(MI, BB, true);
  }
}

bool FgpuSETargetLowering::isEligibleForTailCallOptimization(
    const CCState &CCInfo, unsigned NextStackOffset,
    const FgpuFunctionInfo &FI) const {
  if (!UseFgpuTailCalls)
    return false;

  // Exception has to be cleared with eret.
  if (FI.isISR())
    return false;

  // Return false if either the callee or caller has a byval argument.
  if (CCInfo.getInRegsParamsCount() > 0 || FI.hasByvalArg())
    return false;

  // Return true if the callee's argument area is no larger than the
  // caller's.
  return NextStackOffset <= FI.getIncomingArgSize();
}

void FgpuSETargetLowering::
getOpndList(SmallVectorImpl<SDValue> &Ops,
            std::deque<std::pair<unsigned, SDValue>> &RegsToPass,
            bool IsPICCall, bool GlobalOrExternal, bool InternalLinkage,
            bool IsCallReloc, CallLoweringInfo &CLI, SDValue Callee,
            SDValue Chain) const {
  Ops.push_back(Callee);
  FgpuTargetLowering::getOpndList(Ops, RegsToPass, IsPICCall, GlobalOrExternal,
                                  InternalLinkage, IsCallReloc, CLI, Callee,
                                  Chain);
}

SDValue FgpuSETargetLowering::lowerLOAD(SDValue Op, SelectionDAG &DAG) const {
  LoadSDNode &Nd = *cast<LoadSDNode>(Op);
  return FgpuTargetLowering::lowerLOAD(Op, DAG);
}

SDValue FgpuSETargetLowering::lowerSTORE(SDValue Op, SelectionDAG &DAG) const {
  StoreSDNode &Nd = *cast<StoreSDNode>(Op);
  return FgpuTargetLowering::lowerSTORE(Op, DAG);
}

SDValue FgpuSETargetLowering::lowerBITCAST(SDValue Op,
                                           SelectionDAG &DAG) const {
  SDLoc DL(Op);
  MVT Src = Op.getOperand(0).getValueType().getSimpleVT();
  MVT Dest = Op.getValueType().getSimpleVT();
  // Skip other cases of bitcast and use default lowering.
  return SDValue();
}

SDValue FgpuSETargetLowering::lowerINTRINSIC_WO_CHAIN(SDValue Op,
                                                      SelectionDAG &DAG) const {
  SDLoc DL(Op);
  unsigned Intrinsic = cast<ConstantSDNode>(Op->getOperand(0))->getZExtValue();
  switch (Intrinsic) {
  default:
    return SDValue();
//  case Intrinsic::fgpu_fdiv_w:
//  case Intrinsic::fgpu_fdiv_d:
//    // TODO: If intrinsics have fast-math-flags, propagate them.
//    return DAG.getNode(ISD::FDIV, DL, Op->getValueType(0), Op->getOperand(1),
//                       Op->getOperand(2));
  case Intrinsic::thread_pointer: {
    EVT PtrVT = getPointerTy(DAG.getDataLayout());
    return DAG.getNode(FgpuISD::ThreadPointer, DL, PtrVT);
  }
  }
}

SDValue FgpuSETargetLowering::lowerINTRINSIC_W_CHAIN(SDValue Op,
                                                     SelectionDAG &DAG) const {
  unsigned Intr = cast<ConstantSDNode>(Op->getOperand(1))->getZExtValue();
  switch (Intr) {
  default:
    return SDValue();
//  case Intrinsic::fgpu_dpsqx_sa_w_ph:
//    return lowerDSPIntr(Op, DAG, FgpuISD::DPSQX_SA_W_PH);
//  case Intrinsic::fgpu_ld_b:
//  case Intrinsic::fgpu_ld_h:
//  case Intrinsic::fgpu_ld_w:
//  case Intrinsic::fgpu_ld_d:
//   return lowerMSALoadIntr(Op, DAG, Intr, Subtarget);
  }
}

SDValue FgpuSETargetLowering::lowerINTRINSIC_VOID(SDValue Op,
                                                  SelectionDAG &DAG) const {
  unsigned Intr = cast<ConstantSDNode>(Op->getOperand(1))->getZExtValue();
  switch (Intr) {
  default:
    return SDValue();
//  case Intrinsic::fgpu_st_b:
//  case Intrinsic::fgpu_st_h:
//  case Intrinsic::fgpu_st_w:
//  case Intrinsic::fgpu_st_d:
//    return lowerMSAStoreIntr(Op, DAG, Intr, Subtarget);
  }
}

// Lower ISD::EXTRACT_VECTOR_ELT into FgpuISD::VEXTRACT_SEXT_ELT.
//
// The non-value bits resulting from ISD::EXTRACT_VECTOR_ELT are undefined. We
// choose to sign-extend but we could have equally chosen zero-extend. The
// DAGCombiner will fold any sign/zero extension of the ISD::EXTRACT_VECTOR_ELT
// result into this node later (possibly changing it to a zero-extend in the
// process).
SDValue FgpuSETargetLowering::
lowerEXTRACT_VECTOR_ELT(SDValue Op, SelectionDAG &DAG) const {
  SDLoc DL(Op);
  EVT ResTy = Op->getValueType(0);
  SDValue Op0 = Op->getOperand(0);
  EVT VecTy = Op0->getValueType(0);
  return SDValue();

//  if (!VecTy.is128BitVector())
//    return SDValue();
//
//  if (ResTy.isInteger()) {
//    SDValue Op1 = Op->getOperand(1);
//    EVT EltTy = VecTy.getVectorElementType();
//    return DAG.getNode(FgpuISD::VEXTRACT_SEXT_ELT, DL, ResTy, Op0, Op1,
//                       DAG.getValueType(EltTy));
//  }
//
//  return Op;
}

static bool isConstantOrUndef(const SDValue Op) {
  if (Op->isUndef())
    return true;
  if (isa<ConstantSDNode>(Op))
    return true;
  if (isa<ConstantFPSDNode>(Op))
    return true;
  return false;
}

static bool isConstantOrUndefBUILD_VECTOR(const BuildVectorSDNode *Op) {
  for (unsigned i = 0; i < Op->getNumOperands(); ++i)
    if (isConstantOrUndef(Op->getOperand(i)))
      return true;
  return false;
}

// Lowers ISD::BUILD_VECTOR into appropriate SelectionDAG nodes for the
// backend.
//
// Lowers according to the following rules:
// - Constant splats are legal as-is as long as the SplatBitSize is a power of
//   2 less than or equal to 64 and the value fits into a signed 10-bit
//   immediate
// - Constant splats are lowered to bitconverted BUILD_VECTORs if SplatBitSize
//   is a power of 2 less than or equal to 64 and the value does not fit into a
//   signed 10-bit immediate
// - Non-constant splats are legal as-is.
// - Non-constant non-splats are lowered to sequences of INSERT_VECTOR_ELT.
// - All others are illegal and must be expanded.
SDValue FgpuSETargetLowering::lowerBUILD_VECTOR(SDValue Op,
                                                SelectionDAG &DAG) const {
  BuildVectorSDNode *Node = cast<BuildVectorSDNode>(Op);
  EVT ResTy = Op->getValueType(0);
  SDLoc DL(Op);
  APInt SplatValue, SplatUndef;
  unsigned SplatBitSize;
  bool HasAnyUndefs;
  return SDValue();
}

/// Determine whether a range fits a regular pattern of values.
/// This function accounts for the possibility of jumping over the End iterator.
template <typename ValType>
static bool
fitsRegularPattern(typename SmallVectorImpl<ValType>::const_iterator Begin,
                   unsigned CheckStride,
                   typename SmallVectorImpl<ValType>::const_iterator End,
                   ValType ExpectedIndex, unsigned ExpectedIndexStride) {
  auto &I = Begin;

  while (I != End) {
    if (*I != -1 && *I != ExpectedIndex)
      return false;
    ExpectedIndex += ExpectedIndexStride;

    // Incrementing past End is undefined behaviour so we must increment one
    // step at a time and check for End at each step.
    for (unsigned n = 0; n < CheckStride && I != End; ++n, ++I)
      ; // Empty loop body.
  }
  return true;
}
