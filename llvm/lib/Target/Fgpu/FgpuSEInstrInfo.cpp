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
  if (STI.inMicroFgpuMode())
    return STI.isPositionIndependent() ? Fgpu::B_MM : Fgpu::J_MM;
  return STI.isPositionIndependent() ? Fgpu::B : Fgpu::J;
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

  if ((Opc == Fgpu::LW)   || (Opc == Fgpu::LD)   ||
      (Opc == Fgpu::LWC1) || (Opc == Fgpu::LDC1) || (Opc == Fgpu::LDC164)) {
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

  if ((Opc == Fgpu::SW)   || (Opc == Fgpu::SD)   ||
      (Opc == Fgpu::SWC1) || (Opc == Fgpu::SDC1) || (Opc == Fgpu::SDC164)) {
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
  bool isMicroFgpu = Subtarget.inMicroFgpuMode();

  if (Fgpu::GPR32RegClass.contains(DestReg)) { // Copy to CPU Reg.
    if (Fgpu::GPR32RegClass.contains(SrcReg)) {
      if (isMicroFgpu)
        Opc = Fgpu::MOVE16_MM;
      else
        Opc = Fgpu::OR, ZeroReg = Fgpu::ZERO;
    } else if (Fgpu::CCRRegClass.contains(SrcReg))
      Opc = Fgpu::CFC1;
    else if (Fgpu::FGR32RegClass.contains(SrcReg))
      Opc = Fgpu::MFC1;
    else if (Fgpu::HI32RegClass.contains(SrcReg)) {
      Opc = isMicroFgpu ? Fgpu::MFHI16_MM : Fgpu::MFHI;
      SrcReg = 0;
    } else if (Fgpu::LO32RegClass.contains(SrcReg)) {
      Opc = isMicroFgpu ? Fgpu::MFLO16_MM : Fgpu::MFLO;
      SrcReg = 0;
    } else if (Fgpu::HI32DSPRegClass.contains(SrcReg))
      Opc = Fgpu::MFHI_DSP;
    else if (Fgpu::LO32DSPRegClass.contains(SrcReg))
      Opc = Fgpu::MFLO_DSP;
    else if (Fgpu::DSPCCRegClass.contains(SrcReg)) {
      BuildMI(MBB, I, DL, get(Fgpu::RDDSP), DestReg).addImm(1 << 4)
        .addReg(SrcReg, RegState::Implicit | getKillRegState(KillSrc));
      return;
    }
    else if (Fgpu::MSACtrlRegClass.contains(SrcReg))
      Opc = Fgpu::CFCMSA;
  }
  else if (Fgpu::GPR32RegClass.contains(SrcReg)) { // Copy from CPU Reg.
    if (Fgpu::CCRRegClass.contains(DestReg))
      Opc = Fgpu::CTC1;
    else if (Fgpu::FGR32RegClass.contains(DestReg))
      Opc = Fgpu::MTC1;
    else if (Fgpu::HI32RegClass.contains(DestReg))
      Opc = Fgpu::MTHI, DestReg = 0;
    else if (Fgpu::LO32RegClass.contains(DestReg))
      Opc = Fgpu::MTLO, DestReg = 0;
    else if (Fgpu::HI32DSPRegClass.contains(DestReg))
      Opc = Fgpu::MTHI_DSP;
    else if (Fgpu::LO32DSPRegClass.contains(DestReg))
      Opc = Fgpu::MTLO_DSP;
    else if (Fgpu::DSPCCRegClass.contains(DestReg)) {
      BuildMI(MBB, I, DL, get(Fgpu::WRDSP))
        .addReg(SrcReg, getKillRegState(KillSrc)).addImm(1 << 4)
        .addReg(DestReg, RegState::ImplicitDefine);
      return;
    } else if (Fgpu::MSACtrlRegClass.contains(DestReg)) {
      BuildMI(MBB, I, DL, get(Fgpu::CTCMSA))
          .addReg(DestReg)
          .addReg(SrcReg, getKillRegState(KillSrc));
      return;
    }
  }
  else if (Fgpu::FGR32RegClass.contains(DestReg, SrcReg))
    Opc = Fgpu::FMOV_S;
  else if (Fgpu::AFGR64RegClass.contains(DestReg, SrcReg))
    Opc = Fgpu::FMOV_D32;
  else if (Fgpu::FGR64RegClass.contains(DestReg, SrcReg))
    Opc = Fgpu::FMOV_D64;
  else if (Fgpu::GPR64RegClass.contains(DestReg)) { // Copy to CPU64 Reg.
    if (Fgpu::GPR64RegClass.contains(SrcReg))
      Opc = Fgpu::OR64, ZeroReg = Fgpu::ZERO_64;
    else if (Fgpu::HI64RegClass.contains(SrcReg))
      Opc = Fgpu::MFHI64, SrcReg = 0;
    else if (Fgpu::LO64RegClass.contains(SrcReg))
      Opc = Fgpu::MFLO64, SrcReg = 0;
    else if (Fgpu::FGR64RegClass.contains(SrcReg))
      Opc = Fgpu::DMFC1;
  }
  else if (Fgpu::GPR64RegClass.contains(SrcReg)) { // Copy from CPU64 Reg.
    if (Fgpu::HI64RegClass.contains(DestReg))
      Opc = Fgpu::MTHI64, DestReg = 0;
    else if (Fgpu::LO64RegClass.contains(DestReg))
      Opc = Fgpu::MTLO64, DestReg = 0;
    else if (Fgpu::FGR64RegClass.contains(DestReg))
      Opc = Fgpu::DMTC1;
  }
  else if (Fgpu::MSA128BRegClass.contains(DestReg)) { // Copy to MSA reg
    if (Fgpu::MSA128BRegClass.contains(SrcReg))
      Opc = Fgpu::MOVE_V;
  }

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
  case Fgpu::OR_MM:
  case Fgpu::OR:
    if (MI.getOperand(2).getReg() == Fgpu::ZERO)
      return true;
    break;
  case Fgpu::OR64:
    if (MI.getOperand(2).getReg() == Fgpu::ZERO_64)
      return true;
    break;
  }
  return false;
}

/// If @MI is WRDSP/RRDSP instruction return true with @isWrite set to true
/// if it is WRDSP instruction.
static bool isReadOrWriteToDSPReg(const MachineInstr &MI, bool &isWrite) {
  switch (MI.getOpcode()) {
  default:
   return false;
  case Fgpu::WRDSP:
  case Fgpu::WRDSP_MM:
    isWrite = true;
    break;
  case Fgpu::RDDSP:
  case Fgpu::RDDSP_MM:
    isWrite = false;
    break;
  }
  return true;
}

/// We check for the common case of 'or', as it's Fgpu' preferred instruction
/// for GPRs but we have to check the operands to ensure that is the case.
/// Other move instructions for Fgpu are directly identifiable.
Optional<DestSourcePair>
FgpuSEInstrInfo::isCopyInstrImpl(const MachineInstr &MI) const {
  bool isDSPControlWrite = false;
  // Condition is made to match the creation of WRDSP/RDDSP copy instruction
  // from copyPhysReg function.
  if (isReadOrWriteToDSPReg(MI, isDSPControlWrite)) {
    if (!MI.getOperand(1).isImm() || MI.getOperand(1).getImm() != (1 << 4))
      return None;
    else if (isDSPControlWrite) {
      return DestSourcePair{MI.getOperand(2), MI.getOperand(0)};

    } else {
      return DestSourcePair{MI.getOperand(0), MI.getOperand(2)};
    }
  } else if (MI.isMoveReg() || isORCopyInst(MI)) {
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

  if (Fgpu::GPR32RegClass.hasSubClassEq(RC))
    Opc = Fgpu::SW;
  else if (Fgpu::GPR64RegClass.hasSubClassEq(RC))
    Opc = Fgpu::SD;
  else if (Fgpu::ACC64RegClass.hasSubClassEq(RC))
    Opc = Fgpu::STORE_ACC64;
  else if (Fgpu::ACC64DSPRegClass.hasSubClassEq(RC))
    Opc = Fgpu::STORE_ACC64DSP;
  else if (Fgpu::ACC128RegClass.hasSubClassEq(RC))
    Opc = Fgpu::STORE_ACC128;
  else if (Fgpu::DSPCCRegClass.hasSubClassEq(RC))
    Opc = Fgpu::STORE_CCOND_DSP;
  else if (Fgpu::FGR32RegClass.hasSubClassEq(RC))
    Opc = Fgpu::SWC1;
  else if (Fgpu::AFGR64RegClass.hasSubClassEq(RC))
    Opc = Fgpu::SDC1;
  else if (Fgpu::FGR64RegClass.hasSubClassEq(RC))
    Opc = Fgpu::SDC164;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v16i8))
    Opc = Fgpu::ST_B;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v8i16) ||
           TRI->isTypeLegalForClass(*RC, MVT::v8f16))
    Opc = Fgpu::ST_H;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v4i32) ||
           TRI->isTypeLegalForClass(*RC, MVT::v4f32))
    Opc = Fgpu::ST_W;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v2i64) ||
           TRI->isTypeLegalForClass(*RC, MVT::v2f64))
    Opc = Fgpu::ST_D;
  else if (Fgpu::LO32RegClass.hasSubClassEq(RC))
    Opc = Fgpu::SW;
  else if (Fgpu::LO64RegClass.hasSubClassEq(RC))
    Opc = Fgpu::SD;
  else if (Fgpu::HI32RegClass.hasSubClassEq(RC))
    Opc = Fgpu::SW;
  else if (Fgpu::HI64RegClass.hasSubClassEq(RC))
    Opc = Fgpu::SD;
  else if (Fgpu::DSPRRegClass.hasSubClassEq(RC))
    Opc = Fgpu::SWDSP;

  // Hi, Lo are normally caller save but they are callee save
  // for interrupt handling.
  const Function &Func = MBB.getParent()->getFunction();
  if (Func.hasFnAttribute("interrupt")) {
    if (Fgpu::HI32RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(Fgpu::MFHI), Fgpu::K0);
      SrcReg = Fgpu::K0;
    } else if (Fgpu::HI64RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(Fgpu::MFHI64), Fgpu::K0_64);
      SrcReg = Fgpu::K0_64;
    } else if (Fgpu::LO32RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(Fgpu::MFLO), Fgpu::K0);
      SrcReg = Fgpu::K0;
    } else if (Fgpu::LO64RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(Fgpu::MFLO64), Fgpu::K0_64);
      SrcReg = Fgpu::K0_64;
    }
  }

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
  bool ReqIndirectLoad = Func.hasFnAttribute("interrupt") &&
                         (DestReg == Fgpu::LO0 || DestReg == Fgpu::LO0_64 ||
                          DestReg == Fgpu::HI0 || DestReg == Fgpu::HI0_64);

  if (Fgpu::GPR32RegClass.hasSubClassEq(RC))
    Opc = Fgpu::LW;
  else if (Fgpu::GPR64RegClass.hasSubClassEq(RC))
    Opc = Fgpu::LD;
  else if (Fgpu::ACC64RegClass.hasSubClassEq(RC))
    Opc = Fgpu::LOAD_ACC64;
  else if (Fgpu::ACC64DSPRegClass.hasSubClassEq(RC))
    Opc = Fgpu::LOAD_ACC64DSP;
  else if (Fgpu::ACC128RegClass.hasSubClassEq(RC))
    Opc = Fgpu::LOAD_ACC128;
  else if (Fgpu::DSPCCRegClass.hasSubClassEq(RC))
    Opc = Fgpu::LOAD_CCOND_DSP;
  else if (Fgpu::FGR32RegClass.hasSubClassEq(RC))
    Opc = Fgpu::LWC1;
  else if (Fgpu::AFGR64RegClass.hasSubClassEq(RC))
    Opc = Fgpu::LDC1;
  else if (Fgpu::FGR64RegClass.hasSubClassEq(RC))
    Opc = Fgpu::LDC164;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v16i8))
    Opc = Fgpu::LD_B;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v8i16) ||
           TRI->isTypeLegalForClass(*RC, MVT::v8f16))
    Opc = Fgpu::LD_H;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v4i32) ||
           TRI->isTypeLegalForClass(*RC, MVT::v4f32))
    Opc = Fgpu::LD_W;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v2i64) ||
           TRI->isTypeLegalForClass(*RC, MVT::v2f64))
    Opc = Fgpu::LD_D;
  else if (Fgpu::HI32RegClass.hasSubClassEq(RC))
    Opc = Fgpu::LW;
  else if (Fgpu::HI64RegClass.hasSubClassEq(RC))
    Opc = Fgpu::LD;
  else if (Fgpu::LO32RegClass.hasSubClassEq(RC))
    Opc = Fgpu::LW;
  else if (Fgpu::LO64RegClass.hasSubClassEq(RC))
    Opc = Fgpu::LD;
  else if (Fgpu::DSPRRegClass.hasSubClassEq(RC))
    Opc = Fgpu::LWDSP;

  assert(Opc && "Register class not handled!");

  if (!ReqIndirectLoad)
    BuildMI(MBB, I, DL, get(Opc), DestReg)
        .addFrameIndex(FI)
        .addImm(Offset)
        .addMemOperand(MMO);
  else {
    // Load HI/LO through K0. Notably the DestReg is encoded into the
    // instruction itself.
    unsigned Reg = Fgpu::K0;
    unsigned LdOp = Fgpu::MTLO;
    if (DestReg == Fgpu::HI0)
      LdOp = Fgpu::MTHI;

    if (Subtarget.getABI().ArePtrs64bit()) {
      Reg = Fgpu::K0_64;
      if (DestReg == Fgpu::HI0_64)
        LdOp = Fgpu::MTHI64;
      else
        LdOp = Fgpu::MTLO64;
    }

    BuildMI(MBB, I, DL, get(Opc), Reg)
        .addFrameIndex(FI)
        .addImm(Offset)
        .addMemOperand(MMO);
    BuildMI(MBB, I, DL, get(LdOp)).addReg(Reg);
  }
}

bool FgpuSEInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineBasicBlock &MBB = *MI.getParent();
  bool isMicroFgpu = Subtarget.inMicroFgpuMode();
  unsigned Opc;

  switch (MI.getDesc().getOpcode()) {
  default:
    return false;
  case Fgpu::RetRA:
    expandRetRA(MBB, MI);
    break;
  case Fgpu::ERet:
    expandERet(MBB, MI);
    break;
  case Fgpu::PseudoMFHI:
    expandPseudoMFHiLo(MBB, MI, Fgpu::MFHI);
    break;
  case Fgpu::PseudoMFHI_MM:
    expandPseudoMFHiLo(MBB, MI, Fgpu::MFHI16_MM);
    break;
  case Fgpu::PseudoMFLO:
    expandPseudoMFHiLo(MBB, MI, Fgpu::MFLO);
    break;
  case Fgpu::PseudoMFLO_MM:
    expandPseudoMFHiLo(MBB, MI, Fgpu::MFLO16_MM);
    break;
  case Fgpu::PseudoMFHI64:
    expandPseudoMFHiLo(MBB, MI, Fgpu::MFHI64);
    break;
  case Fgpu::PseudoMFLO64:
    expandPseudoMFHiLo(MBB, MI, Fgpu::MFLO64);
    break;
  case Fgpu::PseudoMTLOHI:
    expandPseudoMTLoHi(MBB, MI, Fgpu::MTLO, Fgpu::MTHI, false);
    break;
  case Fgpu::PseudoMTLOHI64:
    expandPseudoMTLoHi(MBB, MI, Fgpu::MTLO64, Fgpu::MTHI64, false);
    break;
  case Fgpu::PseudoMTLOHI_DSP:
    expandPseudoMTLoHi(MBB, MI, Fgpu::MTLO_DSP, Fgpu::MTHI_DSP, true);
    break;
  case Fgpu::PseudoMTLOHI_MM:
    expandPseudoMTLoHi(MBB, MI, Fgpu::MTLO_MM, Fgpu::MTHI_MM, false);
    break;
  case Fgpu::PseudoCVT_S_W:
    expandCvtFPInt(MBB, MI, Fgpu::CVT_S_W, Fgpu::MTC1, false);
    break;
  case Fgpu::PseudoCVT_D32_W:
    Opc = isMicroFgpu ? Fgpu::CVT_D32_W_MM : Fgpu::CVT_D32_W;
    expandCvtFPInt(MBB, MI, Opc, Fgpu::MTC1, false);
    break;
  case Fgpu::PseudoCVT_S_L:
    expandCvtFPInt(MBB, MI, Fgpu::CVT_S_L, Fgpu::DMTC1, true);
    break;
  case Fgpu::PseudoCVT_D64_W:
    Opc = isMicroFgpu ? Fgpu::CVT_D64_W_MM : Fgpu::CVT_D64_W;
    expandCvtFPInt(MBB, MI, Opc, Fgpu::MTC1, true);
    break;
  case Fgpu::PseudoCVT_D64_L:
    expandCvtFPInt(MBB, MI, Fgpu::CVT_D64_L, Fgpu::DMTC1, true);
    break;
  case Fgpu::BuildPairF64:
    expandBuildPairF64(MBB, MI, isMicroFgpu, false);
    break;
  case Fgpu::BuildPairF64_64:
    expandBuildPairF64(MBB, MI, isMicroFgpu, true);
    break;
  case Fgpu::ExtractElementF64:
    expandExtractElementF64(MBB, MI, isMicroFgpu, false);
    break;
  case Fgpu::ExtractElementF64_64:
    expandExtractElementF64(MBB, MI, isMicroFgpu, true);
    break;
  case Fgpu::Fgpueh_return32:
  case Fgpu::Fgpueh_return64:
    expandEhReturn(MBB, MI);
    break;
  }

  MBB.erase(MI);
  return true;
}

/// isBranchWithImm - Return true if the branch contains an immediate
/// operand (\see lib/Target/Fgpu/FgpuBranchExpansion.cpp).
bool FgpuSEInstrInfo::isBranchWithImm(unsigned Opc) const {
  switch (Opc) {
  default:
    return false;
  case Fgpu::BBIT0:
  case Fgpu::BBIT1:
  case Fgpu::BBIT032:
  case Fgpu::BBIT132:
    return true;
  }
}

/// getOppositeBranchOpc - Return the inverse of the specified
/// opcode, e.g. turning BEQ to BNE.
unsigned FgpuSEInstrInfo::getOppositeBranchOpc(unsigned Opc) const {
  switch (Opc) {
  default:           llvm_unreachable("Illegal opcode!");
  case Fgpu::BEQ:    return Fgpu::BNE;
  case Fgpu::BEQ_MM: return Fgpu::BNE_MM;
  case Fgpu::BNE:    return Fgpu::BEQ;
  case Fgpu::BNE_MM: return Fgpu::BEQ_MM;
  case Fgpu::BGTZ:   return Fgpu::BLEZ;
  case Fgpu::BGEZ:   return Fgpu::BLTZ;
  case Fgpu::BLTZ:   return Fgpu::BGEZ;
  case Fgpu::BLEZ:   return Fgpu::BGTZ;
  case Fgpu::BGTZ_MM:   return Fgpu::BLEZ_MM;
  case Fgpu::BGEZ_MM:   return Fgpu::BLTZ_MM;
  case Fgpu::BLTZ_MM:   return Fgpu::BGEZ_MM;
  case Fgpu::BLEZ_MM:   return Fgpu::BGTZ_MM;
  case Fgpu::BEQ64:  return Fgpu::BNE64;
  case Fgpu::BNE64:  return Fgpu::BEQ64;
  case Fgpu::BGTZ64: return Fgpu::BLEZ64;
  case Fgpu::BGEZ64: return Fgpu::BLTZ64;
  case Fgpu::BLTZ64: return Fgpu::BGEZ64;
  case Fgpu::BLEZ64: return Fgpu::BGTZ64;
  case Fgpu::BC1T:   return Fgpu::BC1F;
  case Fgpu::BC1F:   return Fgpu::BC1T;
  case Fgpu::BC1T_MM:   return Fgpu::BC1F_MM;
  case Fgpu::BC1F_MM:   return Fgpu::BC1T_MM;
  case Fgpu::BEQZ16_MM: return Fgpu::BNEZ16_MM;
  case Fgpu::BNEZ16_MM: return Fgpu::BEQZ16_MM;
  case Fgpu::BEQZC_MM:  return Fgpu::BNEZC_MM;
  case Fgpu::BNEZC_MM:  return Fgpu::BEQZC_MM;
  case Fgpu::BEQZC:  return Fgpu::BNEZC;
  case Fgpu::BNEZC:  return Fgpu::BEQZC;
  case Fgpu::BLEZC:  return Fgpu::BGTZC;
  case Fgpu::BGEZC:  return Fgpu::BLTZC;
  case Fgpu::BGEC:   return Fgpu::BLTC;
  case Fgpu::BGTZC:  return Fgpu::BLEZC;
  case Fgpu::BLTZC:  return Fgpu::BGEZC;
  case Fgpu::BLTC:   return Fgpu::BGEC;
  case Fgpu::BGEUC:  return Fgpu::BLTUC;
  case Fgpu::BLTUC:  return Fgpu::BGEUC;
  case Fgpu::BEQC:   return Fgpu::BNEC;
  case Fgpu::BNEC:   return Fgpu::BEQC;
  case Fgpu::BC1EQZ: return Fgpu::BC1NEZ;
  case Fgpu::BC1NEZ: return Fgpu::BC1EQZ;
  case Fgpu::BEQZC_MMR6:  return Fgpu::BNEZC_MMR6;
  case Fgpu::BNEZC_MMR6:  return Fgpu::BEQZC_MMR6;
  case Fgpu::BLEZC_MMR6:  return Fgpu::BGTZC_MMR6;
  case Fgpu::BGEZC_MMR6:  return Fgpu::BLTZC_MMR6;
  case Fgpu::BGEC_MMR6:   return Fgpu::BLTC_MMR6;
  case Fgpu::BGTZC_MMR6:  return Fgpu::BLEZC_MMR6;
  case Fgpu::BLTZC_MMR6:  return Fgpu::BGEZC_MMR6;
  case Fgpu::BLTC_MMR6:   return Fgpu::BGEC_MMR6;
  case Fgpu::BGEUC_MMR6:  return Fgpu::BLTUC_MMR6;
  case Fgpu::BLTUC_MMR6:  return Fgpu::BGEUC_MMR6;
  case Fgpu::BEQC_MMR6:   return Fgpu::BNEC_MMR6;
  case Fgpu::BNEC_MMR6:   return Fgpu::BEQC_MMR6;
  case Fgpu::BC1EQZC_MMR6: return Fgpu::BC1NEZC_MMR6;
  case Fgpu::BC1NEZC_MMR6: return Fgpu::BC1EQZC_MMR6;
  case Fgpu::BEQZC64:  return Fgpu::BNEZC64;
  case Fgpu::BNEZC64:  return Fgpu::BEQZC64;
  case Fgpu::BEQC64:   return Fgpu::BNEC64;
  case Fgpu::BNEC64:   return Fgpu::BEQC64;
  case Fgpu::BGEC64:   return Fgpu::BLTC64;
  case Fgpu::BGEUC64:  return Fgpu::BLTUC64;
  case Fgpu::BLTC64:   return Fgpu::BGEC64;
  case Fgpu::BLTUC64:  return Fgpu::BGEUC64;
  case Fgpu::BGTZC64:  return Fgpu::BLEZC64;
  case Fgpu::BGEZC64:  return Fgpu::BLTZC64;
  case Fgpu::BLTZC64:  return Fgpu::BGEZC64;
  case Fgpu::BLEZC64:  return Fgpu::BGTZC64;
  case Fgpu::BBIT0:  return Fgpu::BBIT1;
  case Fgpu::BBIT1:  return Fgpu::BBIT0;
  case Fgpu::BBIT032:  return Fgpu::BBIT132;
  case Fgpu::BBIT132:  return Fgpu::BBIT032;
  case Fgpu::BZ_B:   return Fgpu::BNZ_B;
  case Fgpu::BZ_H:   return Fgpu::BNZ_H;
  case Fgpu::BZ_W:   return Fgpu::BNZ_W;
  case Fgpu::BZ_D:   return Fgpu::BNZ_D;
  case Fgpu::BZ_V:   return Fgpu::BNZ_V;
  case Fgpu::BNZ_B:  return Fgpu::BZ_B;
  case Fgpu::BNZ_H:  return Fgpu::BZ_H;
  case Fgpu::BNZ_W:  return Fgpu::BZ_W;
  case Fgpu::BNZ_D:  return Fgpu::BZ_D;
  case Fgpu::BNZ_V:  return Fgpu::BZ_V;
  }
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

  if (isInt<16>(Amount)) {
    // addi sp, sp, amount
    BuildMI(MBB, I, DL, get(ADDiu), SP).addReg(SP).addImm(Amount);
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
  unsigned Size = STI.isABI_N64() ? 64 : 32;
  unsigned LUi = STI.isABI_N64() ? Fgpu::LUi64 : Fgpu::LUi;
  unsigned ZEROReg = STI.isABI_N64() ? Fgpu::ZERO_64 : Fgpu::ZERO;
  const TargetRegisterClass *RC = STI.isABI_N64() ?
    &Fgpu::GPR64RegClass : &Fgpu::GPR32RegClass;
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

unsigned FgpuSEInstrInfo::getAnalyzableBrOpc(unsigned Opc) const {
  return (Opc == Fgpu::BEQ    || Opc == Fgpu::BEQ_MM || Opc == Fgpu::BNE    ||
          Opc == Fgpu::BNE_MM || Opc == Fgpu::BGTZ   || Opc == Fgpu::BGEZ   ||
          Opc == Fgpu::BLTZ   || Opc == Fgpu::BLEZ   || Opc == Fgpu::BEQ64  ||
          Opc == Fgpu::BNE64  || Opc == Fgpu::BGTZ64 || Opc == Fgpu::BGEZ64 ||
          Opc == Fgpu::BLTZ64 || Opc == Fgpu::BLEZ64 || Opc == Fgpu::BC1T   ||
          Opc == Fgpu::BC1F   || Opc == Fgpu::B      || Opc == Fgpu::J      ||
          Opc == Fgpu::J_MM   || Opc == Fgpu::B_MM   || Opc == Fgpu::BEQZC_MM ||
          Opc == Fgpu::BNEZC_MM || Opc == Fgpu::BEQC || Opc == Fgpu::BNEC   ||
          Opc == Fgpu::BLTC   || Opc == Fgpu::BGEC   || Opc == Fgpu::BLTUC  ||
          Opc == Fgpu::BGEUC  || Opc == Fgpu::BGTZC  || Opc == Fgpu::BLEZC  ||
          Opc == Fgpu::BGEZC  || Opc == Fgpu::BLTZC  || Opc == Fgpu::BEQZC  ||
          Opc == Fgpu::BNEZC  || Opc == Fgpu::BEQZC64 || Opc == Fgpu::BNEZC64 ||
          Opc == Fgpu::BEQC64 || Opc == Fgpu::BNEC64 || Opc == Fgpu::BGEC64 ||
          Opc == Fgpu::BGEUC64 || Opc == Fgpu::BLTC64 || Opc == Fgpu::BLTUC64 ||
          Opc == Fgpu::BGTZC64 || Opc == Fgpu::BGEZC64 ||
          Opc == Fgpu::BLTZC64 || Opc == Fgpu::BLEZC64 || Opc == Fgpu::BC ||
          Opc == Fgpu::BBIT0 || Opc == Fgpu::BBIT1 || Opc == Fgpu::BBIT032 ||
          Opc == Fgpu::BBIT132 ||  Opc == Fgpu::BC_MMR6 ||
          Opc == Fgpu::BEQC_MMR6 || Opc == Fgpu::BNEC_MMR6 ||
          Opc == Fgpu::BLTC_MMR6 || Opc == Fgpu::BGEC_MMR6 ||
          Opc == Fgpu::BLTUC_MMR6 || Opc == Fgpu::BGEUC_MMR6 ||
          Opc == Fgpu::BGTZC_MMR6 || Opc == Fgpu::BLEZC_MMR6 ||
          Opc == Fgpu::BGEZC_MMR6 || Opc == Fgpu::BLTZC_MMR6 ||
          Opc == Fgpu::BEQZC_MMR6 || Opc == Fgpu::BNEZC_MMR6) ? Opc : 0;
}

void FgpuSEInstrInfo::expandRetRA(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I) const {

  MachineInstrBuilder MIB;
  if (Subtarget.isGP64bit())
    MIB = BuildMI(MBB, I, I->getDebugLoc(), get(Fgpu::PseudoReturn64))
              .addReg(Fgpu::RA_64, RegState::Undef);
  else
    MIB = BuildMI(MBB, I, I->getDebugLoc(), get(Fgpu::PseudoReturn))
              .addReg(Fgpu::RA, RegState::Undef);

  // Retain any imp-use flags.
  for (auto & MO : I->operands()) {
    if (MO.isImplicit())
      MIB.add(MO);
  }
}

void FgpuSEInstrInfo::expandERet(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator I) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(Fgpu::ERET));
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

void FgpuSEInstrInfo::expandPseudoMFHiLo(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         unsigned NewOpc) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(NewOpc), I->getOperand(0).getReg());
}

void FgpuSEInstrInfo::expandPseudoMTLoHi(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         unsigned LoOpc,
                                         unsigned HiOpc,
                                         bool HasExplicitDef) const {
  // Expand
  //  lo_hi pseudomtlohi $gpr0, $gpr1
  // to these two instructions:
  //  mtlo $gpr0
  //  mthi $gpr1

  DebugLoc DL = I->getDebugLoc();
  const MachineOperand &SrcLo = I->getOperand(1), &SrcHi = I->getOperand(2);
  MachineInstrBuilder LoInst = BuildMI(MBB, I, DL, get(LoOpc));
  MachineInstrBuilder HiInst = BuildMI(MBB, I, DL, get(HiOpc));

  // Add lo/hi registers if the mtlo/hi instructions created have explicit
  // def registers.
  if (HasExplicitDef) {
    Register DstReg = I->getOperand(0).getReg();
    Register DstLo = getRegisterInfo().getSubReg(DstReg, Fgpu::sub_lo);
    Register DstHi = getRegisterInfo().getSubReg(DstReg, Fgpu::sub_hi);
    LoInst.addReg(DstLo, RegState::Define);
    HiInst.addReg(DstHi, RegState::Define);
  }

  LoInst.addReg(SrcLo.getReg(), getKillRegState(SrcLo.isKill()));
  HiInst.addReg(SrcHi.getReg(), getKillRegState(SrcHi.isKill()));
}

void FgpuSEInstrInfo::expandCvtFPInt(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I,
                                     unsigned CvtOpc, unsigned MovOpc,
                                     bool IsI64) const {
  const MCInstrDesc &CvtDesc = get(CvtOpc), &MovDesc = get(MovOpc);
  const MachineOperand &Dst = I->getOperand(0), &Src = I->getOperand(1);
  unsigned DstReg = Dst.getReg(), SrcReg = Src.getReg(), TmpReg = DstReg;
  unsigned KillSrc =  getKillRegState(Src.isKill());
  DebugLoc DL = I->getDebugLoc();
  bool DstIsLarger, SrcIsLarger;

  std::tie(DstIsLarger, SrcIsLarger) =
      compareOpndSize(CvtOpc, *MBB.getParent());

  if (DstIsLarger)
    TmpReg = getRegisterInfo().getSubReg(DstReg, Fgpu::sub_lo);

  if (SrcIsLarger)
    DstReg = getRegisterInfo().getSubReg(DstReg, Fgpu::sub_lo);

  BuildMI(MBB, I, DL, MovDesc, TmpReg).addReg(SrcReg, KillSrc);
  BuildMI(MBB, I, DL, CvtDesc, DstReg).addReg(TmpReg, RegState::Kill);
}

void FgpuSEInstrInfo::expandExtractElementF64(MachineBasicBlock &MBB,
                                              MachineBasicBlock::iterator I,
                                              bool isMicroFgpu,
                                              bool FP64) const {
  Register DstReg = I->getOperand(0).getReg();
  Register SrcReg = I->getOperand(1).getReg();
  unsigned N = I->getOperand(2).getImm();
  DebugLoc dl = I->getDebugLoc();

  assert(N < 2 && "Invalid immediate");
  unsigned SubIdx = N ? Fgpu::sub_hi : Fgpu::sub_lo;
  Register SubReg = getRegisterInfo().getSubReg(SrcReg, SubIdx);

  // FPXX on Fgpu-II or Fgpu32r1 should have been handled with a spill/reload
  // in FgpuSEFrameLowering.cpp.
  assert(!(Subtarget.isABI_FPXX() && !Subtarget.hasFgpu32r2()));

  // FP64A (FP64 with nooddspreg) should have been handled with a spill/reload
  // in FgpuSEFrameLowering.cpp.
  assert(!(Subtarget.isFP64bit() && !Subtarget.useOddSPReg()));

  if (SubIdx == Fgpu::sub_hi && Subtarget.hasMTHC1()) {
    // FIXME: Strictly speaking MFHC1 only reads the top 32-bits however, we
    //        claim to read the whole 64-bits as part of a white lie used to
    //        temporarily work around a widespread bug in the -mfp64 support.
    //        The problem is that none of the 32-bit fpu ops mention the fact
    //        that they clobber the upper 32-bits of the 64-bit FPR. Fixing that
    //        requires a major overhaul of the FPU implementation which can't
    //        be done right now due to time constraints.
    //        MFHC1 is one of two instructions that are affected since they are
    //        the only instructions that don't read the lower 32-bits.
    //        We therefore pretend that it reads the bottom 32-bits to
    //        artificially create a dependency and prevent the scheduler
    //        changing the behaviour of the code.
    BuildMI(MBB, I, dl,
            get(isMicroFgpu ? (FP64 ? Fgpu::MFHC1_D64_MM : Fgpu::MFHC1_D32_MM)
                            : (FP64 ? Fgpu::MFHC1_D64 : Fgpu::MFHC1_D32)),
            DstReg)
        .addReg(SrcReg);
  } else
    BuildMI(MBB, I, dl, get(Fgpu::MFC1), DstReg).addReg(SubReg);
}

void FgpuSEInstrInfo::expandBuildPairF64(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         bool isMicroFgpu, bool FP64) const {
  Register DstReg = I->getOperand(0).getReg();
  unsigned LoReg = I->getOperand(1).getReg(), HiReg = I->getOperand(2).getReg();
  const MCInstrDesc& Mtc1Tdd = get(Fgpu::MTC1);
  DebugLoc dl = I->getDebugLoc();
  const TargetRegisterInfo &TRI = getRegisterInfo();

  // When mthc1 is available, use:
  //   mtc1 Lo, $fp
  //   mthc1 Hi, $fp
  //
  // Otherwise, for O32 FPXX ABI:
  //   spill + reload via ldc1
  // This case is handled by the frame lowering code.
  //
  // Otherwise, for FP32:
  //   mtc1 Lo, $fp
  //   mtc1 Hi, $fp + 1
  //
  // The case where dmtc1 is available doesn't need to be handled here
  // because it never creates a BuildPairF64 node.

  // FPXX on Fgpu-II or Fgpu32r1 should have been handled with a spill/reload
  // in FgpuSEFrameLowering.cpp.
  assert(!(Subtarget.isABI_FPXX() && !Subtarget.hasFgpu32r2()));

  // FP64A (FP64 with nooddspreg) should have been handled with a spill/reload
  // in FgpuSEFrameLowering.cpp.
  assert(!(Subtarget.isFP64bit() && !Subtarget.useOddSPReg()));

  BuildMI(MBB, I, dl, Mtc1Tdd, TRI.getSubReg(DstReg, Fgpu::sub_lo))
    .addReg(LoReg);

  if (Subtarget.hasMTHC1()) {
    // FIXME: The .addReg(DstReg) is a white lie used to temporarily work
    //        around a widespread bug in the -mfp64 support.
    //        The problem is that none of the 32-bit fpu ops mention the fact
    //        that they clobber the upper 32-bits of the 64-bit FPR. Fixing that
    //        requires a major overhaul of the FPU implementation which can't
    //        be done right now due to time constraints.
    //        MTHC1 is one of two instructions that are affected since they are
    //        the only instructions that don't read the lower 32-bits.
    //        We therefore pretend that it reads the bottom 32-bits to
    //        artificially create a dependency and prevent the scheduler
    //        changing the behaviour of the code.
    BuildMI(MBB, I, dl,
            get(isMicroFgpu ? (FP64 ? Fgpu::MTHC1_D64_MM : Fgpu::MTHC1_D32_MM)
                            : (FP64 ? Fgpu::MTHC1_D64 : Fgpu::MTHC1_D32)),
            DstReg)
        .addReg(DstReg)
        .addReg(HiReg);
  } else if (Subtarget.isABI_FPXX())
    llvm_unreachable("BuildPairF64 not expanded in frame lowering code!");
  else
    BuildMI(MBB, I, dl, Mtc1Tdd, TRI.getSubReg(DstReg, Fgpu::sub_hi))
      .addReg(HiReg);
}

void FgpuSEInstrInfo::expandEhReturn(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I) const {
  // This pseudo instruction is generated as part of the lowering of
  // ISD::EH_RETURN. We convert it to a stack increment by OffsetReg, and
  // indirect jump to TargetReg
  FgpuABIInfo ABI = Subtarget.getABI();
  unsigned ADDU = ABI.GetPtrAdduOp();
  unsigned SP = Subtarget.isGP64bit() ? Fgpu::SP_64 : Fgpu::SP;
  unsigned RA = Subtarget.isGP64bit() ? Fgpu::RA_64 : Fgpu::RA;
  unsigned T9 = Subtarget.isGP64bit() ? Fgpu::T9_64 : Fgpu::T9;
  unsigned ZERO = Subtarget.isGP64bit() ? Fgpu::ZERO_64 : Fgpu::ZERO;
  Register OffsetReg = I->getOperand(0).getReg();
  Register TargetReg = I->getOperand(1).getReg();

  // addu $ra, $v0, $zero
  // addu $sp, $sp, $v1
  // jr   $ra (via RetRA)
  const TargetMachine &TM = MBB.getParent()->getTarget();
  if (TM.isPositionIndependent())
    BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), T9)
        .addReg(TargetReg)
        .addReg(ZERO);
  BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), RA)
      .addReg(TargetReg)
      .addReg(ZERO);
  BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), SP).addReg(SP).addReg(OffsetReg);
  expandRetRA(MBB, I);
}

const FgpuInstrInfo *llvm::createFgpuSEInstrInfo(const FgpuSubtarget &STI) {
  return new FgpuSEInstrInfo(STI);
}
