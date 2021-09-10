//===-- FgpuMCTargetDesc.h - Fgpu Target Descriptions -----------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides Fgpu specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUMCTARGETDESC_H
#define LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUMCTARGETDESC_H

#include "llvm/Support/DataTypes.h"

#include <memory>

namespace llvm {
class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectTargetWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class MCTargetOptions;
class StringRef;
class Target;
class Triple;

MCCodeEmitter *createFgpuMCCodeEmitterEB(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx);
MCCodeEmitter *createFgpuMCCodeEmitterEL(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx);

MCAsmBackend *createFgpuAsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                   const MCRegisterInfo &MRI,
                                   const MCTargetOptions &Options);

std::unique_ptr<MCObjectTargetWriter>
createFgpuELFObjectWriter(const Triple &TT, bool IsN32);

namespace FGPU_MC {
StringRef selectFgpuCPU(const Triple &TT, StringRef CPU);
}

} // End llvm namespace

// Defines symbolic names for Fgpu registers.  This defines a mapping from
// register name to register number.
#define GET_REGINFO_ENUM
#include "FgpuGenRegisterInfo.inc"

// Defines symbolic names for the Fgpu instructions.
#define GET_INSTRINFO_ENUM
#include "FgpuGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "FgpuGenSubtargetInfo.inc"

#endif
