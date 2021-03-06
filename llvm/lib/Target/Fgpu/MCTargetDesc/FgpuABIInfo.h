//===---- FgpuABIInfo.h - Information about FGPU ABI's --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUABIINFO_H
#define LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUABIINFO_H

#include "llvm/ADT/Triple.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/MC/MCRegisterInfo.h"

namespace llvm {

template <typename T> class ArrayRef;
class MCTargetOptions;
class StringRef;

class FgpuABIInfo {
public:
  enum class ABI { Unknown, CC_Fgpu };

protected:
  ABI ThisABI;

public:
  FgpuABIInfo(ABI ThisABI) : ThisABI(ThisABI) {}

  static FgpuABIInfo Unknown() { return FgpuABIInfo(ABI::Unknown); }
  static FgpuABIInfo CC_Fgpu() { return FgpuABIInfo(ABI::CC_Fgpu); }
  static FgpuABIInfo computeTargetABI(const Triple &TT, StringRef CPU,
                                      const MCTargetOptions &Options);

  bool IsKnown() const { return ThisABI != ABI::Unknown; }
  ABI GetEnumValue() const { return ThisABI; }

  /// The registers to use for byval arguments.
  ArrayRef<MCPhysReg> GetByValArgRegs() const;

  /// The registers to use for the variable argument list.
  ArrayRef<MCPhysReg> GetVarArgRegs() const;

  /// Obtain the size of the area allocated by the callee for arguments.
  /// CallingConv::FastCall affects the value for O32.
  unsigned GetCalleeAllocdArgSizeInBytes(CallingConv::ID CC) const;

  /// Ordering of ABI's
  /// FgpuGenSubtargetInfo.inc will use this to resolve conflicts when given
  /// multiple ABI options.
  bool operator<(const FgpuABIInfo Other) const {
    return ThisABI < Other.GetEnumValue();
  }

  unsigned GetStackPtr() const;
  unsigned GetFramePtr() const;
  unsigned GetBasePtr() const;
  unsigned GetGlobalPtr() const;
  unsigned GetNullPtr() const;
  unsigned GetZeroReg() const;
  unsigned GetPtrAdduOp() const;
  unsigned GetPtrAddiuOp() const;
  unsigned GetPtrSubuOp() const;
  unsigned GetPtrAndOp() const;
  unsigned GetGPRMoveOp() const;

  unsigned GetEhDataReg(unsigned I) const;
};
}

#endif
