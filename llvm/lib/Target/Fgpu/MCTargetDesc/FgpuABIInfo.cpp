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
static const MCPhysReg O32IntRegs[4] = {Fgpu::A0, Fgpu::A1, Fgpu::A2, Fgpu::A3};

static const MCPhysReg Fgpu64IntRegs[8] = {
    Fgpu::A0_64, Fgpu::A1_64, Fgpu::A2_64, Fgpu::A3_64,
    Fgpu::T0_64, Fgpu::T1_64, Fgpu::T2_64, Fgpu::T3_64};
}

ArrayRef<MCPhysReg> FgpuABIInfo::GetByValArgRegs() const {
  if (IsO32())
    return makeArrayRef(O32IntRegs);
  if (IsN32() || IsN64())
    return makeArrayRef(Fgpu64IntRegs);
  llvm_unreachable("Unhandled ABI");
}

ArrayRef<MCPhysReg> FgpuABIInfo::GetVarArgRegs() const {
  if (IsO32())
    return makeArrayRef(O32IntRegs);
  if (IsN32() || IsN64())
    return makeArrayRef(Fgpu64IntRegs);
  llvm_unreachable("Unhandled ABI");
}

unsigned FgpuABIInfo::GetCalleeAllocdArgSizeInBytes(CallingConv::ID CC) const {
  if (IsO32())
    return CC != CallingConv::Fast ? 16 : 0;
  if (IsN32() || IsN64())
    return 0;
  llvm_unreachable("Unhandled ABI");
}

FgpuABIInfo FgpuABIInfo::computeTargetABI(const Triple &TT, StringRef CPU,
                                          const MCTargetOptions &Options) {
  if (Options.getABIName().startswith("o32"))
    return FgpuABIInfo::O32();
  if (Options.getABIName().startswith("n32"))
    return FgpuABIInfo::N32();
  if (Options.getABIName().startswith("n64"))
    return FgpuABIInfo::N64();
  if (TT.getEnvironment() == llvm::Triple::GNUABIN32)
    return FgpuABIInfo::N32();
  assert(Options.getABIName().empty() && "Unknown ABI option for FGPU");

  if (TT.isFGPU64())
    return FgpuABIInfo::N64();
  return FgpuABIInfo::O32();
}

unsigned FgpuABIInfo::GetStackPtr() const {
  return ArePtrs64bit() ? Fgpu::SP_64 : Fgpu::SP;
}

unsigned FgpuABIInfo::GetFramePtr() const {
  return ArePtrs64bit() ? Fgpu::FP_64 : Fgpu::FP;
}

unsigned FgpuABIInfo::GetBasePtr() const {
  return ArePtrs64bit() ? Fgpu::S7_64 : Fgpu::S7;
}

unsigned FgpuABIInfo::GetGlobalPtr() const {
  return ArePtrs64bit() ? Fgpu::GP_64 : Fgpu::GP;
}

unsigned FgpuABIInfo::GetNullPtr() const {
  return ArePtrs64bit() ? Fgpu::ZERO_64 : Fgpu::ZERO;
}

unsigned FgpuABIInfo::GetZeroReg() const {
  return AreGprs64bit() ? Fgpu::ZERO_64 : Fgpu::ZERO;
}

unsigned FgpuABIInfo::GetPtrAdduOp() const {
  return ArePtrs64bit() ? Fgpu::DADDu : Fgpu::ADDu;
}

unsigned FgpuABIInfo::GetPtrAddiuOp() const {
  return ArePtrs64bit() ? Fgpu::DADDiu : Fgpu::ADDiu;
}

unsigned FgpuABIInfo::GetPtrSubuOp() const {
  return ArePtrs64bit() ? Fgpu::DSUBu : Fgpu::SUBu;
}

unsigned FgpuABIInfo::GetPtrAndOp() const {
  return ArePtrs64bit() ? Fgpu::AND64 : Fgpu::AND;
}

unsigned FgpuABIInfo::GetGPRMoveOp() const {
  return ArePtrs64bit() ? Fgpu::OR64 : Fgpu::OR;
}

unsigned FgpuABIInfo::GetEhDataReg(unsigned I) const {
  static const unsigned EhDataReg[] = {
    Fgpu::A0, Fgpu::A1, Fgpu::A2, Fgpu::A3
  };
  static const unsigned EhDataReg64[] = {
    Fgpu::A0_64, Fgpu::A1_64, Fgpu::A2_64, Fgpu::A3_64
  };

  return IsN64() ? EhDataReg64[I] : EhDataReg[I];
}

