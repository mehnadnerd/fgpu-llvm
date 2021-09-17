//===---- FgpuABIInfo.cpp - Information about FGPU ABI's ------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "FgpuABIInfo.h"
#include "FgpuRegisterInfo.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/MC/MCTargetOptions.h"
#include "llvm/Support/CommandLine.h"

using namespace llvm;

// Note: this option is defined here to be visible from libLLVMFgpuAsmParser
//       and libLLVMFgpuCodeGen
cl::opt<bool>
EmitJalrReloc("fgpu-jalr-reloc", cl::Hidden,
              cl::desc("FGPU: Emit R_{MICRO}FGPU_JALR relocation with jalr"),
              cl::init(true));

namespace {
 static const MCPhysReg O32IntRegs[4] = {Fgpu::R4, Fgpu::R5, Fgpu::R6, Fgpu::R7};
}

ArrayRef<MCPhysReg> FgpuABIInfo::GetByValArgRegs() const {
  return makeArrayRef(O32IntRegs);
  llvm_unreachable("Unhandled ABI");
}

ArrayRef<MCPhysReg> FgpuABIInfo::GetVarArgRegs() const {
  return makeArrayRef(O32IntRegs);
  llvm_unreachable("Unhandled ABI");
}

unsigned FgpuABIInfo::GetCalleeAllocdArgSizeInBytes(CallingConv::ID CC) const {
  return 0;
  llvm_unreachable("Unhandled ABI");
}

FgpuABIInfo FgpuABIInfo::computeTargetABI(const Triple &TT, StringRef CPU,
                                          const MCTargetOptions &Options) {
  FgpuABIInfo abi(ABI::Unknown);

  abi = ABI::CC_Fgpu;
  // Assert exactly one ABI was chosen.
  assert(abi.ThisABI != ABI::Unknown);

  return abi;
}

unsigned FgpuABIInfo::GetStackPtr() const {
  return Fgpu::SP;
}

unsigned FgpuABIInfo::GetFramePtr() const {
  return Fgpu::R28;
}

unsigned FgpuABIInfo::GetBasePtr() const {
  return Fgpu::R27;
}

unsigned FgpuABIInfo::GetGlobalPtr() const {
  return Fgpu::R29;
}

unsigned FgpuABIInfo::GetNullPtr() const {
  return Fgpu::ZERO;
}

unsigned FgpuABIInfo::GetZeroReg() const {
  return Fgpu::ZERO;
}

unsigned FgpuABIInfo::GetPtrAdduOp() const {
  return Fgpu::ADDu;
}

unsigned FgpuABIInfo::GetPtrAddiuOp() const {
  return Fgpu::ADDiu;
}

unsigned FgpuABIInfo::GetPtrSubuOp() const {
  return Fgpu::SUBu;
}

unsigned FgpuABIInfo::GetPtrAndOp() const {
  return Fgpu::AND;
}

unsigned FgpuABIInfo::GetGPRMoveOp() const {
  return Fgpu::OR;
}

unsigned FgpuABIInfo::GetEhDataReg(unsigned I) const {
  static const unsigned EhDataReg[] = {
      Fgpu::R4, Fgpu::R5, Fgpu::R6, Fgpu::R7
  };

  return EhDataReg[I];
}

