//===- FgpuInstrInfo.h - Fgpu Instruction Information -----------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the Fgpu implementation of the TargetInstrInfo class.
//
// FIXME: We need to override TargetInstrInfo::getInlineAsmLength method in
// order for FgpuLongBranch pass to work correctly when the code has inline
// assembly.  The returned value doesn't have to be the asm instruction's exact
// size in bytes; FgpuLongBranch only expects it to be the correct upper bound.
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_FGPU_FGPUINSTRINFO_H
#define LLVM_LIB_TARGET_FGPU_FGPUINSTRINFO_H

#include "MCTargetDesc/FgpuMCTargetDesc.h"
#include "Fgpu.h"
#include "FgpuRegisterInfo.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include <cstdint>

#define GET_INSTRINFO_HEADER
#include "FgpuGenInstrInfo.inc"

namespace llvm {

class MachineInstr;
class MachineOperand;
class FgpuSubtarget;
class TargetRegisterClass;
class TargetRegisterInfo;

class FgpuInstrInfo : public FgpuGenInstrInfo {
  virtual void anchor();

protected:
  const FgpuSubtarget &Subtarget;
  //unsigned UncondBrOpc;

public:
  enum BranchType {
    BT_None,       // Couldn't analyze branch.
    BT_NoBranch,   // No branches found.
    BT_Uncond,     // One unconditional branch.
    BT_Cond,       // One conditional branch.
    BT_CondUncond, // A conditional branch followed by an unconditional branch.
    BT_Indirect    // One indirct branch.
  };

  explicit FgpuInstrInfo(const FgpuSubtarget &STI, unsigned UncondBrOpc);

  static const FgpuInstrInfo *create(FgpuSubtarget &STI);

  /// Branch Analysis
  bool analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                     MachineBasicBlock *&FBB,
                     SmallVectorImpl<MachineOperand> &Cond,
                     bool AllowModify) const override;

  unsigned removeBranch(MachineBasicBlock &MBB,
                        int *BytesRemoved = nullptr) const override;

  unsigned insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                        MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond,
                        const DebugLoc &DL,
                        int *BytesAdded = nullptr) const override;

  BranchType analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                           MachineBasicBlock *&FBB,
                           SmallVectorImpl<MachineOperand> &Cond,
                           bool AllowModify,
                           SmallVectorImpl<MachineInstr *> &BranchInstrs) const;

  /// Determine the opcode of a non-delay slot form for a branch if one exists.
  unsigned getEquivalentCompactForm(const MachineBasicBlock::iterator I) const;

  /// Determine if the branch target is in range.
  bool isBranchOffsetInRange(unsigned BranchOpc,
                             int64_t BrOffset) const override;

  /// Insert nop instruction when hazard condition is found
  void insertNoop(MachineBasicBlock &MBB,
                  MachineBasicBlock::iterator MI) const override;

  /// getRegisterInfo - TargetInstrInfo is a superset of MRegister info.  As
  /// such, whenever a client has an instance of instruction info, it should
  /// always be able to get register info as well (through this method).
  virtual const FgpuRegisterInfo &getRegisterInfo() const = 0;

  unsigned getOppositeBranchOpc(unsigned Opc) const;

  virtual bool isBranchWithImm(unsigned Opc) const {
    return false;
  }

  /// Return the number of bytes of code the specified instruction may be.
  unsigned getInstSizeInBytes(const MachineInstr &MI) const override;

  void storeRegToStackSlot(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator MBBI,
                           Register SrcReg, bool isKill, int FrameIndex,
                           const TargetRegisterClass *RC,
                           const TargetRegisterInfo *TRI) const override {
    storeRegToStack(MBB, MBBI, SrcReg, isKill, FrameIndex, RC, TRI, 0);
  }

  void loadRegFromStackSlot(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MBBI,
                            Register DestReg, int FrameIndex,
                            const TargetRegisterClass *RC,
                            const TargetRegisterInfo *TRI) const override {
    loadRegFromStack(MBB, MBBI, DestReg, FrameIndex, RC, TRI, 0);
  }

  virtual void storeRegToStack(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MI,
                               Register SrcReg, bool isKill, int FrameIndex,
                               const TargetRegisterClass *RC,
                               const TargetRegisterInfo *TRI,
                               int64_t Offset) const = 0;

  virtual void loadRegFromStack(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MI,
                                Register DestReg, int FrameIndex,
                                const TargetRegisterClass *RC,
                                const TargetRegisterInfo *TRI,
                                int64_t Offset) const = 0;

  virtual void adjustStackPtr(unsigned SP, int64_t Amount,
                              MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator I) const = 0;

  /// Create an instruction which has the same operands and memory operands
  /// as MI but has a new opcode.
  MachineInstrBuilder genInstrWithNewOpc(unsigned NewOpc,
                                         MachineBasicBlock::iterator I) const;

  bool findCommutedOpIndices(const MachineInstr &MI, unsigned &SrcOpIdx1,
                             unsigned &SrcOpIdx2) const override;

  /// Perform target specific instruction verification.
  bool verifyInstruction(const MachineInstr &MI,
                         StringRef &ErrInfo) const override;

  std::pair<unsigned, unsigned>
  decomposeMachineOperandsTargetFlags(unsigned TF) const override;

  ArrayRef<std::pair<unsigned, const char *>>
  getSerializableDirectMachineOperandTargetFlags() const override;

  Optional<RegImmPair> isAddImmediate(const MachineInstr &MI,
                                      Register Reg) const override;

  Optional<ParamLoadedValue> describeLoadedValue(const MachineInstr &MI,
                                                 Register Reg) const override;

protected:
  bool isZeroImm(const MachineOperand &op) const;

  MachineMemOperand *GetMemOperand(MachineBasicBlock &MBB, int FI,
                                   MachineMemOperand::Flags Flags) const;

private:
  unsigned getAnalyzableBrOpc(unsigned Opc) const;

  void AnalyzeCondBr(const MachineInstr *Inst, unsigned Opc,
                     MachineBasicBlock *&BB,
                     SmallVectorImpl<MachineOperand> &Cond) const;

  void BuildCondBr(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                   const DebugLoc &DL, ArrayRef<MachineOperand> Cond) const;
  void BuildUnCondBr(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                   const DebugLoc &DL) const;
};

/// Create FgpuInstrInfo objects.
const FgpuInstrInfo *createFgpuSEInstrInfo(const FgpuSubtarget &STI);

} // end namespace llvm

#endif // LLVM_LIB_TARGET_FGPU_FGPUINSTRINFO_H
