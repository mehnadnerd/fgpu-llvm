//===-- FgpuMachineFunctionInfo.cpp - Private data used for Fgpu ----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "FgpuMachineFunction.h"
#include "MCTargetDesc/FgpuABIInfo.h"
#include "FgpuSubtarget.h"
#include "FgpuTargetMachine.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/CommandLine.h"

using namespace llvm;

static cl::opt<bool>
FixGlobalBaseReg("fgpu-fix-global-base-reg", cl::Hidden, cl::init(true),
                 cl::desc("Always use $gp as the global base register."));

FgpuFunctionInfo::~FgpuFunctionInfo() = default;

bool FgpuFunctionInfo::globalBaseRegSet() const {
  return GlobalBaseReg;
}

static const TargetRegisterClass &getGlobalBaseRegClass(MachineFunction &MF) {
  //auto &STI = static_cast<const FgpuSubtarget &>(MF.getSubtarget());
  auto &TM = static_cast<const FgpuTargetMachine &>(MF.getTarget());

  return Fgpu::GPROutRegClass;
}

Register FgpuFunctionInfo::getGlobalBaseReg(MachineFunction &MF) {
  if (!GlobalBaseReg)
    GlobalBaseReg =
        MF.getRegInfo().createVirtualRegister(&getGlobalBaseRegClass(MF));
  return GlobalBaseReg;
}

Register FgpuFunctionInfo::getGlobalBaseRegForGlobalISel(MachineFunction &MF) {
  if (!GlobalBaseReg) {
    getGlobalBaseReg(MF);
    initGlobalBaseReg(MF);
  }
  return GlobalBaseReg;
}

void FgpuFunctionInfo::initGlobalBaseReg(MachineFunction &MF) {
  if (!GlobalBaseReg)
    return;

  assert(false && "not supporte");

  MachineBasicBlock &MBB = MF.front();
  MachineBasicBlock::iterator I = MBB.begin();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  DebugLoc DL;
  const TargetRegisterClass *RC;
  const FgpuABIInfo &ABI =
      static_cast<const FgpuTargetMachine &>(MF.getTarget()).getABI();
  RC = &Fgpu::GPROutRegClass;

  Register V0 = RegInfo.createVirtualRegister(RC);
  Register V1 = RegInfo.createVirtualRegister(RC);

  if (!MF.getTarget().isPositionIndependent()) {
    // Set global register to __gnu_local_gp.
    //
    // lui   $v0, %hi(__gnu_local_gp)
    // addiu $globalbasereg, $v0, %lo(__gnu_local_gp)
    BuildMI(MBB, I, DL, TII.get(Fgpu::LUi), V0)
        .addExternalSymbol("__gnu_local_gp", FgpuII::MO_ABS_HI);
    BuildMI(MBB, I, DL, TII.get(Fgpu::Li), GlobalBaseReg).addReg(V0)
        .addExternalSymbol("__gnu_local_gp", FgpuII::MO_ABS_LO);
    return;
  }

  MF.getRegInfo().addLiveIn(Fgpu::R27);
  MBB.addLiveIn(Fgpu::R27);

  // For O32 ABI, the following instruction sequence is emitted to initialize
  // the global base register:
  //
  //  0. lui   $2, %hi(_gp_disp)
  //  1. addiu $2, $2, %lo(_gp_disp)
  //  2. addu  $globalbasereg, $2, $t9
  //
  // We emit only the last instruction here.
  //
  // GNU linker requires that the first two instructions appear at the beginning
  // of a function and no instructions be inserted before or between them.
  // The two instructions are emitted during lowering to MC layer in order to
  // avoid any reordering.
  //
  // Register $2 (Fgpu::V0) is added to the list of live-in registers to ensure
  // the value instruction 1 (addiu) defines is valid when instruction 2 (addu)
  // reads it.
  MF.getRegInfo().addLiveIn(Fgpu::R2);
  MBB.addLiveIn(Fgpu::R2);
  BuildMI(MBB, I, DL, TII.get(Fgpu::ADD), GlobalBaseReg)
      .addReg(Fgpu::R2).addReg(Fgpu::R27);
}

void FgpuFunctionInfo::createEhDataRegsFI(MachineFunction &MF) {
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  for (int I = 0; I < 4; ++I) {
    const TargetRegisterClass &RC = Fgpu::GPROutRegClass;

    EhDataRegFI[I] = MF.getFrameInfo().CreateStackObject(
        TRI.getSpillSize(RC), TRI.getSpillAlign(RC), false);
  }
}

void FgpuFunctionInfo::createISRRegFI(MachineFunction &MF) {
  // ISRs require spill slots for Status & ErrorPC Coprocessor 0 registers.
  // The current implementation only supports Fgpu32r2+ not Fgpu64rX. Status
  // is always 32 bits, ErrorPC is 32 or 64 bits dependent on architecture,
  // however Fgpu32r2+ is the supported architecture.
  const TargetRegisterClass &RC = Fgpu::GPROutRegClass;
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();

  for (int I = 0; I < 2; ++I)
    ISRDataRegFI[I] = MF.getFrameInfo().CreateStackObject(
        TRI.getSpillSize(RC), TRI.getSpillAlign(RC), false);
}

bool FgpuFunctionInfo::isEhDataRegFI(int FI) const {
  return CallsEhReturn && (FI == EhDataRegFI[0] || FI == EhDataRegFI[1]
                        || FI == EhDataRegFI[2] || FI == EhDataRegFI[3]);
}

bool FgpuFunctionInfo::isISRRegFI(int FI) const {
  return IsISR && (FI == ISRDataRegFI[0] || FI == ISRDataRegFI[1]);
}
MachinePointerInfo FgpuFunctionInfo::callPtrInfo(MachineFunction &MF,
                                                 const char *ES) {
  return MachinePointerInfo(MF.getPSVManager().getExternalSymbolCallEntry(ES));
}

MachinePointerInfo FgpuFunctionInfo::callPtrInfo(MachineFunction &MF,
                                                 const GlobalValue *GV) {
  return MachinePointerInfo(MF.getPSVManager().getGlobalValueCallEntry(GV));
}

int FgpuFunctionInfo::getMoveF64ViaSpillFI(MachineFunction &MF,
                                           const TargetRegisterClass *RC) {
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  if (MoveF64ViaSpillFI == -1) {
    MoveF64ViaSpillFI = MF.getFrameInfo().CreateStackObject(
        TRI.getSpillSize(*RC), TRI.getSpillAlign(*RC), false);
  }
  return MoveF64ViaSpillFI;
}

void FgpuFunctionInfo::anchor() {}
