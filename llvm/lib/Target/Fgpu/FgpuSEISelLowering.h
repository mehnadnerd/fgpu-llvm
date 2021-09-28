//===- FgpuSEISelLowering.h - FgpuSE DAG Lowering Interface -----*- C++ -*-===//
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

#ifndef LLVM_LIB_TARGET_FGPU_FGPUSEISELLOWERING_H
#define LLVM_LIB_TARGET_FGPU_FGPUSEISELLOWERING_H

#include "FgpuISelLowering.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/Support/MachineValueType.h"

namespace llvm {

class MachineBasicBlock;
class MachineInstr;
class FgpuSubtarget;
class FgpuTargetMachine;
class SelectionDAG;
class TargetRegisterClass;

  class FgpuSETargetLowering : public FgpuTargetLowering  {
  public:
    explicit FgpuSETargetLowering(const FgpuTargetMachine &TM,
                                  const FgpuSubtarget &STI);

    bool allowsMisalignedMemoryAccesses(
        EVT VT, unsigned AS = 0, Align Alignment = Align(1),
        MachineMemOperand::Flags Flags = MachineMemOperand::MONone,
        bool *Fast = nullptr) const override;

    SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const override;

    SDValue PerformDAGCombine(SDNode *N, DAGCombinerInfo &DCI) const override;

    MachineBasicBlock *
    EmitInstrWithCustomInserter(MachineInstr &MI,
                                MachineBasicBlock *MBB) const override;

    bool isShuffleMaskLegal(ArrayRef<int> Mask, EVT VT) const override {
      return false;
    }

    const TargetRegisterClass *getRepRegClassFor(MVT VT) const override;

  private:
    bool isEligibleForTailCallOptimization(
        const CCState &CCInfo, unsigned NextStackOffset,
        const FgpuFunctionInfo &FI) const override;

    void
    getOpndList(SmallVectorImpl<SDValue> &Ops,
                std::deque<std::pair<unsigned, SDValue>> &RegsToPass,
                bool IsPICCall, bool GlobalOrExternal, bool InternalLinkage,
                bool IsCallReloc, CallLoweringInfo &CLI, SDValue Callee,
                SDValue Chain) const override;

    SDValue lowerLOAD(SDValue Op, SelectionDAG &DAG) const;
    SDValue lowerSTORE(SDValue Op, SelectionDAG &DAG) const;
    SDValue lowerBITCAST(SDValue Op, SelectionDAG &DAG) const;

//    SDValue lowerMulDiv(SDValue Op, unsigned NewOpc, bool HasLo, bool HasHi,
//                        SelectionDAG &DAG) const;

    SDValue lowerINTRINSIC_WO_CHAIN(SDValue Op, SelectionDAG &DAG) const;
    SDValue lowerINTRINSIC_W_CHAIN(SDValue Op, SelectionDAG &DAG) const;
    SDValue lowerINTRINSIC_VOID(SDValue Op, SelectionDAG &DAG) const;
    SDValue lowerEXTRACT_VECTOR_ELT(SDValue Op, SelectionDAG &DAG) const;
    SDValue lowerBUILD_VECTOR(SDValue Op, SelectionDAG &DAG) const;
//    /// Lower VECTOR_SHUFFLE into one of a number of instructions
//    /// depending on the indices in the shuffle.
//    SDValue lowerVECTOR_SHUFFLE(SDValue Op, SelectionDAG &DAG) const;
    SDValue lowerSELECT(SDValue Op, SelectionDAG &DAG) const;

//    MachineBasicBlock *emitBPOSGE32(MachineInstr &MI,
//                                    MachineBasicBlock *BB) const;
  };

} // end namespace llvm

#endif // LLVM_LIB_TARGET_FGPU_FGPUSEISELLOWERING_H
