//===-- FgpuMCTargetDesc.h - Fgpu Target Descriptions -----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Fgpu specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef FGPUMCTARGETDESC_H
#define FGPUMCTARGETDESC_H

#include "llvm/Support/DataTypes.h"
#include "llvm/MC/MCTargetOptions.h"
#include <memory>


namespace llvm {
class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectTargetWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class StringRef;
class Target;
class Triple;
class raw_ostream;
class raw_pwrite_stream;

extern Target TheFgpuTarget;

MCCodeEmitter *createFgpuMCCodeEmitter(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx);

MCAsmBackend *createFgpuAsmBackend(const Target &T,
                                   const MCSubtargetInfo &STI,
                                   const MCRegisterInfo &MRI,
                                   const MCTargetOptions &Options);

std::unique_ptr<MCObjectTargetWriter> createFgpuELFObjectWriter(uint8_t OSABI,
                                          bool IsLittleEndian);
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
