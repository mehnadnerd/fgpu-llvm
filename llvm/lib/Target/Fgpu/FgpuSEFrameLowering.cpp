//===- FgpuSEFrameLowering.cpp - Fgpu32/64 Frame Information --------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the Fgpu32/64 implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "FgpuSEFrameLowering.h"
#include "MCTargetDesc/FgpuABIInfo.h"
#include "FgpuMachineFunction.h"
#include "FgpuRegisterInfo.h"
#include "FgpuSEInstrInfo.h"
#include "FgpuSubtarget.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCDwarf.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MachineLocation.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include <cassert>
#include <cstdint>
#include <utility>
#include <vector>

using namespace llvm;

static std::pair<unsigned, unsigned> getMFHiLoOpc(unsigned Src) {
  return std::make_pair(0, 0);
}

namespace {

/// Helper class to expand pseudos.
class ExpandPseudo {
public:
  ExpandPseudo(MachineFunction &MF);
  bool expand();

private:
  using Iter = MachineBasicBlock::iterator;

  bool expandInstr(MachineBasicBlock &MBB, Iter I);
  void expandLoadCCond(MachineBasicBlock &MBB, Iter I);
  void expandStoreCCond(MachineBasicBlock &MBB, Iter I);
  void expandLoadACC(MachineBasicBlock &MBB, Iter I, unsigned RegSize);
  void expandStoreACC(MachineBasicBlock &MBB, Iter I, unsigned MFHiOpc,
                      unsigned MFLoOpc, unsigned RegSize);
  bool expandCopy(MachineBasicBlock &MBB, Iter I);
  bool expandCopyACC(MachineBasicBlock &MBB, Iter I, unsigned MFHiOpc,
                     unsigned MFLoOpc);
  bool expandBuildPairF64(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator I, bool FP64) const;
  bool expandExtractElementF64(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator I, bool FP64) const;

  MachineFunction &MF;
  MachineRegisterInfo &MRI;
  const FgpuSubtarget &Subtarget;
  const FgpuSEInstrInfo &TII;
  const FgpuRegisterInfo &RegInfo;
};

} // end anonymous namespace

ExpandPseudo::ExpandPseudo(MachineFunction &MF_)
    : MF(MF_), MRI(MF.getRegInfo()),
      Subtarget(static_cast<const FgpuSubtarget &>(MF.getSubtarget())),
      TII(*static_cast<const FgpuSEInstrInfo *>(Subtarget.getInstrInfo())),
      RegInfo(*Subtarget.getRegisterInfo()) {}

bool ExpandPseudo::expand() {
  bool Expanded = false;

  for (auto &MBB : MF) {
    for (Iter I = MBB.begin(), End = MBB.end(); I != End;)
      Expanded |= expandInstr(MBB, I++);
  }

  return Expanded;
}

bool ExpandPseudo::expandInstr(MachineBasicBlock &MBB, Iter I) {
  switch(I->getOpcode()) {
  case TargetOpcode::COPY:
    if (!expandCopy(MBB, I))
      return false;
    break;
  default:
    return false;
  }

  MBB.erase(I);
  return true;
}

void ExpandPseudo::expandLoadCCond(MachineBasicBlock &MBB, Iter I) {
  //  load $vr, FI
  //  copy ccond, $vr

  assert(I->getOperand(0).isReg() && I->getOperand(1).isFI());

  const TargetRegisterClass *RC = RegInfo.intRegClass(4);
  Register VR = MRI.createVirtualRegister(RC);
  Register Dst = I->getOperand(0).getReg(), FI = I->getOperand(1).getIndex();

  TII.loadRegFromStack(MBB, I, VR, FI, RC, &RegInfo, 0);
  BuildMI(MBB, I, I->getDebugLoc(), TII.get(TargetOpcode::COPY), Dst)
    .addReg(VR, RegState::Kill);
}

void ExpandPseudo::expandStoreCCond(MachineBasicBlock &MBB, Iter I) {
  //  copy $vr, ccond
  //  store $vr, FI

  assert(I->getOperand(0).isReg() && I->getOperand(1).isFI());

  const TargetRegisterClass *RC = RegInfo.intRegClass(4);
  Register VR = MRI.createVirtualRegister(RC);
  Register Src = I->getOperand(0).getReg(), FI = I->getOperand(1).getIndex();

  BuildMI(MBB, I, I->getDebugLoc(), TII.get(TargetOpcode::COPY), VR)
    .addReg(Src, getKillRegState(I->getOperand(0).isKill()));
  TII.storeRegToStack(MBB, I, VR, true, FI, RC, &RegInfo, 0);
}

void ExpandPseudo::expandLoadACC(MachineBasicBlock &MBB, Iter I,
                                 unsigned RegSize) {
  //  load $vr0, FI
  //  copy lo, $vr0
  //  load $vr1, FI + 4
  //  copy hi, $vr1
  // I think this can never be called
//
//  assert(I->getOperand(0).isReg() && I->getOperand(1).isFI());
//
//  const TargetRegisterClass *RC = RegInfo.intRegClass(RegSize);
//  Register VR0 = MRI.createVirtualRegister(RC);
//  Register VR1 = MRI.createVirtualRegister(RC);
//  Register Dst = I->getOperand(0).getReg(), FI = I->getOperand(1).getIndex();
//  Register Lo = RegInfo.getSubReg(Dst, Fgpu::sub_lo);
//  Register Hi = RegInfo.getSubReg(Dst, Fgpu::sub_hi);
//  DebugLoc DL = I->getDebugLoc();
//  const MCInstrDesc &Desc = TII.get(TargetOpcode::COPY);
//
//  TII.loadRegFromStack(MBB, I, VR0, FI, RC, &RegInfo, 0);
//  BuildMI(MBB, I, DL, Desc, Lo).addReg(VR0, RegState::Kill);
//  TII.loadRegFromStack(MBB, I, VR1, FI, RC, &RegInfo, RegSize);
//  BuildMI(MBB, I, DL, Desc, Hi).addReg(VR1, RegState::Kill);
}

void ExpandPseudo::expandStoreACC(MachineBasicBlock &MBB, Iter I,
                                  unsigned MFHiOpc, unsigned MFLoOpc,
                                  unsigned RegSize) {
  //  mflo $vr0, src
  //  store $vr0, FI
  //  mfhi $vr1, src
  //  store $vr1, FI + 4

  assert(I->getOperand(0).isReg() && I->getOperand(1).isFI());

  const TargetRegisterClass *RC = RegInfo.intRegClass(RegSize);
  Register VR0 = MRI.createVirtualRegister(RC);
  Register VR1 = MRI.createVirtualRegister(RC);
  Register Src = I->getOperand(0).getReg(), FI = I->getOperand(1).getIndex();
  unsigned SrcKill = getKillRegState(I->getOperand(0).isKill());
  DebugLoc DL = I->getDebugLoc();

  BuildMI(MBB, I, DL, TII.get(MFLoOpc), VR0).addReg(Src);
  TII.storeRegToStack(MBB, I, VR0, true, FI, RC, &RegInfo, 0);
  BuildMI(MBB, I, DL, TII.get(MFHiOpc), VR1).addReg(Src, SrcKill);
  TII.storeRegToStack(MBB, I, VR1, true, FI, RC, &RegInfo, RegSize);
}

bool ExpandPseudo::expandCopy(MachineBasicBlock &MBB, Iter I) {
  Register Src = I->getOperand(1).getReg();
  std::pair<unsigned, unsigned> Opcodes = getMFHiLoOpc(Src);

  if (!Opcodes.first)
    return false;

  return expandCopyACC(MBB, I, Opcodes.first, Opcodes.second);
}

bool ExpandPseudo::expandCopyACC(MachineBasicBlock &MBB, Iter I,
                                 unsigned MFHiOpc, unsigned MFLoOpc) {
  //  mflo $vr0, src
  //  copy dst_lo, $vr0
  //  mfhi $vr1, src
  //  copy dst_hi, $vr1
//
//  unsigned Dst = I->getOperand(0).getReg(), Src = I->getOperand(1).getReg();
//  const TargetRegisterClass *DstRC = RegInfo.getMinimalPhysRegClass(Dst);
//  unsigned VRegSize = RegInfo.getRegSizeInBits(*DstRC) / 16;
//  const TargetRegisterClass *RC = RegInfo.intRegClass(VRegSize);
//  Register VR0 = MRI.createVirtualRegister(RC);
//  Register VR1 = MRI.createVirtualRegister(RC);
//  unsigned SrcKill = getKillRegState(I->getOperand(1).isKill());
//  Register DstLo = RegInfo.getSubReg(Dst, Fgpu::sub_lo);
//  Register DstHi = RegInfo.getSubReg(Dst, Fgpu::sub_hi);
//  DebugLoc DL = I->getDebugLoc();
//
//  BuildMI(MBB, I, DL, TII.get(MFLoOpc), VR0).addReg(Src);
//  BuildMI(MBB, I, DL, TII.get(TargetOpcode::COPY), DstLo)
//    .addReg(VR0, RegState::Kill);
//  BuildMI(MBB, I, DL, TII.get(MFHiOpc), VR1).addReg(Src, SrcKill);
//  BuildMI(MBB, I, DL, TII.get(TargetOpcode::COPY), DstHi)
//    .addReg(VR1, RegState::Kill);
  return true;
}

/// This method expands the same instruction that FgpuSEInstrInfo::
/// expandBuildPairF64 does, for the case when ABI is fpxx and mthc1 is not
/// available and the case where the ABI is FP64A. It is implemented here
/// because frame indexes are eliminated before FgpuSEInstrInfo::
/// expandBuildPairF64 is called.
bool ExpandPseudo::expandBuildPairF64(MachineBasicBlock &MBB,
                                      MachineBasicBlock::iterator I,
                                      bool FP64) const {
  // For fpxx and when mthc1 is not available, use:
  //   spill + reload via ldc1
  //
  // The case where dmtc1 is available doesn't need to be handled here
  // because it never creates a BuildPairF64 node.
  //
  // The FP64A ABI (fp64 with nooddspreg) must also use a spill/reload sequence
  // for odd-numbered double precision values (because the lower 32-bits is
  // transferred with mtc1 which is redirected to the upper half of the even
  // register). Unfortunately, we have to make this decision before register
  // allocation so for now we use a spill/reload sequence for all
  // double-precision values in regardless of being an odd/even register.
  //
  // For the cases that should be covered here FgpuSEISelDAGToDAG adds $sp as
  // implicit operand, so other passes (like ShrinkWrapping) are aware that
  // stack is used.
//  if (I->getNumOperands() == 4 && I->getOperand(3).isReg()
//      && I->getOperand(3).getReg() == Fgpu::SP) {
//    Register DstReg = I->getOperand(0).getReg();
//    Register LoReg = I->getOperand(1).getReg();
//    Register HiReg = I->getOperand(2).getReg();
//
//    // It should be impossible to have FGR64 on FGPU-II or FGPU32r1 (which are
//    // the cases where mthc1 is not available). 64-bit architectures and
//    // FGPU32r2 or later can use FGR64 though.
//    assert(Subtarget.isGP64bit() || Subtarget.hasMTHC1() ||
//           !Subtarget.isFP64bit());
//
//    const TargetRegisterClass *RC = &Fgpu::GPR32RegClass;
//    const TargetRegisterClass *RC2 =
//        FP64 ? &Fgpu::FGR64RegClass : &Fgpu::AFGR64RegClass;
//
//    // We re-use the same spill slot each time so that the stack frame doesn't
//    // grow too much in functions with a large number of moves.
//    int FI = MF.getInfo<FgpuFunctionInfo>()->getMoveF64ViaSpillFI(MF, RC2);
//    if (!Subtarget.isLittle())
//      std::swap(LoReg, HiReg);
//    TII.storeRegToStack(MBB, I, LoReg, I->getOperand(1).isKill(), FI, RC,
//                        &RegInfo, 0);
//    TII.storeRegToStack(MBB, I, HiReg, I->getOperand(2).isKill(), FI, RC,
//                        &RegInfo, 4);
//    TII.loadRegFromStack(MBB, I, DstReg, FI, RC2, &RegInfo, 0);
//    return true;
//  }

  return false;
}

/// This method expands the same instruction that FgpuSEInstrInfo::
/// expandExtractElementF64 does, for the case when ABI is fpxx and mfhc1 is not
/// available and the case where the ABI is FP64A. It is implemented here
/// because frame indexes are eliminated before FgpuSEInstrInfo::
/// expandExtractElementF64 is called.
bool ExpandPseudo::expandExtractElementF64(MachineBasicBlock &MBB,
                                           MachineBasicBlock::iterator I,
                                           bool FP64) const {
//  const MachineOperand &Op1 = I->getOperand(1);
//  const MachineOperand &Op2 = I->getOperand(2);
//
//  if ((Op1.isReg() && Op1.isUndef()) || (Op2.isReg() && Op2.isUndef())) {
//    Register DstReg = I->getOperand(0).getReg();
//    BuildMI(MBB, I, I->getDebugLoc(), TII.get(Fgpu::IMPLICIT_DEF), DstReg);
//    return true;
//  }
//
//  // For fpxx and when mfhc1 is not available, use:
//  //   spill + reload via ldc1
//  //
//  // The case where dmfc1 is available doesn't need to be handled here
//  // because it never creates a ExtractElementF64 node.
//  //
//  // The FP64A ABI (fp64 with nooddspreg) must also use a spill/reload sequence
//  // for odd-numbered double precision values (because the lower 32-bits is
//  // transferred with mfc1 which is redirected to the upper half of the even
//  // register). Unfortunately, we have to make this decision before register
//  // allocation so for now we use a spill/reload sequence for all
//  // double-precision values in regardless of being an odd/even register.
//  //
//  // For the cases that should be covered here FgpuSEISelDAGToDAG adds $sp as
//  // implicit operand, so other passes (like ShrinkWrapping) are aware that
//  // stack is used.
//  if (I->getNumOperands() == 4 && I->getOperand(3).isReg()
//      && I->getOperand(3).getReg() == Fgpu::SP) {
//    Register DstReg = I->getOperand(0).getReg();
//    Register SrcReg = Op1.getReg();
//    unsigned N = Op2.getImm();
//    int64_t Offset = 4 * (Subtarget.isLittle() ? N : (1 - N));
//
//    // It should be impossible to have FGR64 on FGPU-II or FGPU32r1 (which are
//    // the cases where mfhc1 is not available). 64-bit architectures and
//    // FGPU32r2 or later can use FGR64 though.
//    assert(Subtarget.isGP64bit() || Subtarget.hasMTHC1() ||
//           !Subtarget.isFP64bit());
//
//    const TargetRegisterClass *RC =
//        FP64 ? &Fgpu::FGR64RegClass : &Fgpu::AFGR64RegClass;
//    const TargetRegisterClass *RC2 = &Fgpu::GPR32RegClass;
//
//    // We re-use the same spill slot each time so that the stack frame doesn't
//    // grow too much in functions with a large number of moves.
//    int FI = MF.getInfo<FgpuFunctionInfo>()->getMoveF64ViaSpillFI(MF, RC);
//    TII.storeRegToStack(MBB, I, SrcReg, Op1.isKill(), FI, RC, &RegInfo, 0);
//    TII.loadRegFromStack(MBB, I, DstReg, FI, RC2, &RegInfo, Offset);
//    return true;
//  }

  return false;
}

FgpuSEFrameLowering::FgpuSEFrameLowering(const FgpuSubtarget &STI)
    : FgpuFrameLowering(STI, STI.getStackAlignment()) {}

void FgpuSEFrameLowering::emitPrologue(MachineFunction &MF,
                                       MachineBasicBlock &MBB) const {
  MachineFrameInfo &MFI    = MF.getFrameInfo();
  FgpuFunctionInfo *FgpuFI = MF.getInfo<FgpuFunctionInfo>();

  const FgpuSEInstrInfo &TII =
      *static_cast<const FgpuSEInstrInfo *>(STI.getInstrInfo());
  const FgpuRegisterInfo &RegInfo =
      *static_cast<const FgpuRegisterInfo *>(STI.getRegisterInfo());

  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc dl;
  FgpuABIInfo ABI = STI.getABI();
  unsigned SP = ABI.GetStackPtr();
  unsigned FP = ABI.GetFramePtr();
  unsigned ZERO = ABI.GetNullPtr();
  unsigned MOVE = ABI.GetGPRMoveOp();
  unsigned ADDiu = ABI.GetPtrAddiuOp();
  unsigned AND = Fgpu::AND;

  const TargetRegisterClass *RC = &Fgpu::GPROutRegClass;

  // First, compute final stack size.
  uint64_t StackSize = MFI.getStackSize();

  // No need to allocate space on the stack.
  if (StackSize == 0 && !MFI.adjustsStack()) return;

  MachineModuleInfo &MMI = MF.getMMI();
  const MCRegisterInfo *MRI = MMI.getContext().getRegisterInfo();

  // Adjust stack.
  TII.adjustStackPtr(SP, -StackSize, MBB, MBBI);

  // emit ".cfi_def_cfa_offset StackSize"
  unsigned CFIIndex =
      MF.addFrameInst(MCCFIInstruction::cfiDefCfaOffset(nullptr, StackSize));
  BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex);

  if (MF.getFunction().hasFnAttribute("interrupt"))
    emitInterruptPrologueStub(MF, MBB);

  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();

  if (!CSI.empty()) {
    // Find the instruction past the last instruction that saves a callee-saved
    // register to the stack.
    for (unsigned i = 0; i < CSI.size(); ++i)
      ++MBBI;

    // Iterate over list of callee-saved registers and emit .cfi_offset
    // directives.
    for (std::vector<CalleeSavedInfo>::const_iterator I = CSI.begin(),
           E = CSI.end(); I != E; ++I) {
      int64_t Offset = MFI.getObjectOffset(I->getFrameIdx());
      unsigned Reg = I->getReg();
      { // TODO: may need to add one for the VFP
        // Reg is either in GPR32 or FGR32.
        unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::createOffset(
            nullptr, MRI->getDwarfRegNum(Reg, true), Offset));
        BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
            .addCFIIndex(CFIIndex);
      }
    }
  }

  if (FgpuFI->callsEhReturn()) {
    // Insert instructions that spill eh data registers.
    for (int I = 0; I < 4; ++I) {
      if (!MBB.isLiveIn(ABI.GetEhDataReg(I)))
        MBB.addLiveIn(ABI.GetEhDataReg(I));
      TII.storeRegToStackSlot(MBB, MBBI, ABI.GetEhDataReg(I), false,
                              FgpuFI->getEhDataRegFI(I), RC, &RegInfo);
    }

    // Emit .cfi_offset directives for eh data registers.
    for (int I = 0; I < 4; ++I) {
      int64_t Offset = MFI.getObjectOffset(FgpuFI->getEhDataRegFI(I));
      unsigned Reg = MRI->getDwarfRegNum(ABI.GetEhDataReg(I), true);
      unsigned CFIIndex = MF.addFrameInst(
          MCCFIInstruction::createOffset(nullptr, Reg, Offset));
      BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
          .addCFIIndex(CFIIndex);
    }
  }

  // if framepointer enabled, set it to point to the stack pointer.
  if (hasFP(MF)) {
    // Insert instruction "move $fp, $sp" at this location.
    BuildMI(MBB, MBBI, dl, TII.get(MOVE), FP).addReg(SP).addReg(ZERO)
      .setMIFlag(MachineInstr::FrameSetup);

    // emit ".cfi_def_cfa_register $fp"
    unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::createDefCfaRegister(
        nullptr, MRI->getDwarfRegNum(FP, true)));
    BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
        .addCFIIndex(CFIIndex);

    if (RegInfo.hasStackRealignment(MF)) {
      // addiu $Reg, $zero, -MaxAlignment
      // andi $sp, $sp, $Reg
      Register VR = MF.getRegInfo().createVirtualRegister(RC);
      assert((Log2(MFI.getMaxAlign()) < 16) &&
             "Function's alignment size requirement is not supported.");
      int64_t MaxAlign = -(int64_t)MFI.getMaxAlign().value();

      BuildMI(MBB, MBBI, dl, TII.get(ADDiu), VR).addReg(ZERO).addImm(MaxAlign);
      BuildMI(MBB, MBBI, dl, TII.get(AND), SP).addReg(SP).addReg(VR);

      if (hasBP(MF)) {
        // move $s7, $sp
        unsigned BP = Fgpu::R27;
        BuildMI(MBB, MBBI, dl, TII.get(MOVE), BP)
          .addReg(SP)
          .addReg(ZERO);
      }
    }
  }
}

void FgpuSEFrameLowering::emitInterruptPrologueStub(
    MachineFunction &MF, MachineBasicBlock &MBB) const {
  FgpuFunctionInfo *FgpuFI = MF.getInfo<FgpuFunctionInfo>();
  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();


    report_fatal_error(
        "\"interrupt\" attribute is not supported on fgpu");
}

void FgpuSEFrameLowering::emitEpilogue(MachineFunction &MF,
                                       MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator MBBI = MBB.getFirstTerminator();
  MachineFrameInfo &MFI            = MF.getFrameInfo();
  FgpuFunctionInfo *FgpuFI = MF.getInfo<FgpuFunctionInfo>();

  const FgpuSEInstrInfo &TII =
      *static_cast<const FgpuSEInstrInfo *>(STI.getInstrInfo());
  const FgpuRegisterInfo &RegInfo =
      *static_cast<const FgpuRegisterInfo *>(STI.getRegisterInfo());

  DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
  FgpuABIInfo ABI = STI.getABI();
  unsigned SP = ABI.GetStackPtr();
  unsigned FP = ABI.GetFramePtr();
  unsigned ZERO = ABI.GetNullPtr();
  unsigned MOVE = ABI.GetGPRMoveOp();

  // if framepointer enabled, restore the stack pointer.
  if (hasFP(MF)) {
    // Find the first instruction that restores a callee-saved register.
    MachineBasicBlock::iterator I = MBBI;

    for (unsigned i = 0; i < MFI.getCalleeSavedInfo().size(); ++i)
      --I;

    // Insert instruction "move $sp, $fp" at this location.
    BuildMI(MBB, I, DL, TII.get(MOVE), SP).addReg(FP).addReg(ZERO);
  }

  if (FgpuFI->callsEhReturn()) {
    const TargetRegisterClass *RC =  &Fgpu::GPROutRegClass;

    // Find first instruction that restores a callee-saved register.
    MachineBasicBlock::iterator I = MBBI;
    for (unsigned i = 0; i < MFI.getCalleeSavedInfo().size(); ++i)
      --I;

    // Insert instructions that restore eh data registers.
    for (int J = 0; J < 4; ++J) {
      TII.loadRegFromStackSlot(MBB, I, ABI.GetEhDataReg(J),
                               FgpuFI->getEhDataRegFI(J), RC, &RegInfo);
    }
  }

  if (MF.getFunction().hasFnAttribute("interrupt"))
    emitInterruptEpilogueStub(MF, MBB);

  // Get the number of bytes from FrameInfo
  uint64_t StackSize = MFI.getStackSize();

  if (!StackSize)
    return;

  // Adjust stack.
  TII.adjustStackPtr(SP, StackSize, MBB, MBBI);
}

void FgpuSEFrameLowering::emitInterruptEpilogueStub(
    MachineFunction &MF, MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  FgpuFunctionInfo *FgpuFI = MF.getInfo<FgpuFunctionInfo>();
  DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();

  assert(false && "Exceptiosn not supported");
}

StackOffset
FgpuSEFrameLowering::getFrameIndexReference(const MachineFunction &MF, int FI,
                                            Register &FrameReg) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  FgpuABIInfo ABI = STI.getABI();

  if (MFI.isFixedObjectIndex(FI))
    FrameReg = hasFP(MF) ? ABI.GetFramePtr() : ABI.GetStackPtr();
  else
    FrameReg = hasBP(MF) ? ABI.GetBasePtr() : ABI.GetStackPtr();

  return StackOffset::getFixed(MFI.getObjectOffset(FI) + MFI.getStackSize() -
                               getOffsetOfLocalArea() +
                               MFI.getOffsetAdjustment());
}

bool FgpuSEFrameLowering::spillCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    ArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  MachineFunction *MF = MBB.getParent();
  const TargetInstrInfo &TII = *STI.getInstrInfo();

  for (unsigned i = 0, e = CSI.size(); i != e; ++i) {
    // Add the callee-saved register as live-in. Do not add if the register is
    // RA and return address is taken, because it has already been added in
    // method FgpuTargetLowering::lowerRETURNADDR.
    // It's killed at the spill, unless the register is RA and return address
    // is taken.
    unsigned Reg = CSI[i].getReg();
    bool IsRAAndRetAddrIsTaken = (Reg == Fgpu::LR)
        && MF->getFrameInfo().isReturnAddressTaken();
    if (!IsRAAndRetAddrIsTaken)
      MBB.addLiveIn(Reg);

    // Insert the spill to the stack frame.
    bool IsKill = !IsRAAndRetAddrIsTaken;
    const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(Reg);
    TII.storeRegToStackSlot(MBB, MI, Reg, IsKill,
                            CSI[i].getFrameIdx(), RC, TRI);
  }

  return true;
}

bool
FgpuSEFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  // Reserve call frame if the size of the maximum call frame fits into 16-bit
  // immediate field and there are no variable sized objects on the stack.
  // Make sure the second register scavenger spill slot can be accessed with one
  // instruction.
  return isInt<16>(MFI.getMaxCallFrameSize() + getStackAlignment()) &&
    !MFI.hasVarSizedObjects();
}

/// Mark \p Reg and all registers aliasing it in the bitset.
static void setAliasRegs(MachineFunction &MF, BitVector &SavedRegs,
                         unsigned Reg) {
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();
  for (MCRegAliasIterator AI(Reg, TRI, true); AI.isValid(); ++AI)
    SavedRegs.set(*AI);
}

void FgpuSEFrameLowering::determineCalleeSaves(MachineFunction &MF,
                                               BitVector &SavedRegs,
                                               RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();
  FgpuFunctionInfo *FgpuFI = MF.getInfo<FgpuFunctionInfo>();
  FgpuABIInfo ABI = STI.getABI();
  unsigned RA = Fgpu::LR;
  unsigned FP = ABI.GetFramePtr();
  unsigned BP = Fgpu::R27;

  // Mark $ra and $fp as used if function has dedicated frame pointer.
  if (hasFP(MF)) {
    setAliasRegs(MF, SavedRegs, RA);
    setAliasRegs(MF, SavedRegs, FP);
  }
  // Mark $s7 as used if function has dedicated base pointer.
  if (hasBP(MF))
    setAliasRegs(MF, SavedRegs, BP);

  // Create spill slots for eh data registers if function calls eh_return.
  if (FgpuFI->callsEhReturn())
    FgpuFI->createEhDataRegsFI(MF);

  // Create spill slots for Coprocessor 0 registers if function is an ISR.
  if (FgpuFI->isISR())
    FgpuFI->createISRRegFI(MF);

  // Expand pseudo instructions which load, store or copy accumulators.
  // Add an emergency spill slot if a pseudo was expanded.
  if (ExpandPseudo(MF).expand()) {
    // The spill slot should be half the size of the accumulator. If target have
    // general-purpose registers 64 bits wide, it should be 64-bit, otherwise
    // it should be 32-bit.
    const TargetRegisterClass &RC = Fgpu::GPROutRegClass;
    int FI = MF.getFrameInfo().CreateStackObject(TRI->getSpillSize(RC),
                                                 TRI->getSpillAlign(RC), false);
    RS->addScavengingFrameIndex(FI);
  }

  // Set scavenging frame index if necessary.
  uint64_t MaxSPOffset = estimateStackSize(MF);

  // MSA has a minimum offset of 10 bits signed. If there is a variable
  // sized object on the stack, the estimation cannot account for it.
  if (isIntN(16, MaxSPOffset) &&
      !MF.getFrameInfo().hasVarSizedObjects())
    return;

  const TargetRegisterClass &RC = Fgpu::GPROutRegClass;
  int FI = MF.getFrameInfo().CreateStackObject(TRI->getSpillSize(RC),
                                               TRI->getSpillAlign(RC), false);
  RS->addScavengingFrameIndex(FI);
}

const FgpuFrameLowering *
llvm::createFgpuSEFrameLowering(const FgpuSubtarget &ST) {
  return new FgpuSEFrameLowering(ST);
}
