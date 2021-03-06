//===-- FgpuAsmBackend.cpp - Fgpu Asm Backend  ----------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the FgpuAsmBackend class.
//
//===----------------------------------------------------------------------===//
//

#include "MCTargetDesc/FgpuAsmBackend.h"
#include "MCTargetDesc/FgpuABIInfo.h"
#include "MCTargetDesc/FgpuFixupKinds.h"
#include "MCTargetDesc/FgpuMCExpr.h"
#include "MCTargetDesc/FgpuMCTargetDesc.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCTargetOptions.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/Format.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

// Prepare value for the target space for it
static unsigned adjustFixupValue(const MCFixup &Fixup, uint64_t Value,
                                 MCContext &Ctx) {

  unsigned Kind = Fixup.getKind();

  // Add/subtract and shift
  switch (Kind) {
  default:
    return 0;
  case FK_Data_2:
  case Fgpu::fixup_Fgpu_LO16:
  case Fgpu::fixup_Fgpu_GPREL16:
  case Fgpu::fixup_Fgpu_GPOFF_HI:
  case Fgpu::fixup_Fgpu_GPOFF_LO:
  case Fgpu::fixup_Fgpu_GOT_PAGE:
  case Fgpu::fixup_Fgpu_GOT_OFST:
  case Fgpu::fixup_Fgpu_GOT_DISP:
  case Fgpu::fixup_Fgpu_GOT_LO16:
  case Fgpu::fixup_Fgpu_CALL_LO16:
  case Fgpu::fixup_MICROFGPU_GPOFF_HI:
  case Fgpu::fixup_MICROFGPU_GPOFF_LO:
  case Fgpu::fixup_MICROFGPU_LO16:
  case Fgpu::fixup_MICROFGPU_GOT_PAGE:
  case Fgpu::fixup_MICROFGPU_GOT_OFST:
  case Fgpu::fixup_MICROFGPU_GOT_DISP:
  case Fgpu::fixup_FGPU_PCLO16:
    Value &= 0xffff;
    break;
  case FK_DTPRel_4:
  case FK_DTPRel_8:
  case FK_TPRel_4:
  case FK_TPRel_8:
  case FK_GPRel_4:
  case FK_Data_4:
  case FK_Data_8:
  case Fgpu::fixup_Fgpu_SUB:
  case Fgpu::fixup_MICROFGPU_SUB:
    break;
  case Fgpu::fixup_Fgpu_PC16:
    // The displacement is then divided by 4 to give us an 18 bit
    // address range. Forcing a signed division because Value can be negative.
    Value = (int64_t)Value / 4;
    // We now check if Value can be encoded as a 16-bit signed immediate.
    if (!isInt<16>(Value)) {
      Ctx.reportError(Fixup.getLoc(), "out of range PC16 fixup");
      return 0;
    }
    break;
  case Fgpu::fixup_Fgpu_JSUB:
  case Fgpu::fixup_Fgpu_PC14:
    Value = (int64_t)Value / 4;
    // We now check if Value can be encoded as a 14-bit signed immediate.
    if (!isInt<14>(Value)) {
      Ctx.reportError(Fixup.getLoc(), "out of range PC14 fixup");
      return 0;
    }
    break;
  case Fgpu::fixup_Fgpu_PC7:
    Value = (int64_t)Value / 4;
    // We now check if Value can be encoded as a 7-bit signed immediate.
    if (!isInt<7>(Value)) {
      Ctx.reportError(Fixup.getLoc(), "out of range PC7 fixup");
      return 0;
    }
    break;
  case Fgpu::fixup_FGPU_PC19_S2:
  case Fgpu::fixup_MICROFGPU_PC19_S2:
    // Forcing a signed division because Value can be negative.
    Value = (int64_t)Value / 4;
    // We now check if Value can be encoded as a 19-bit signed immediate.
    if (!isInt<19>(Value)) {
      Ctx.reportError(Fixup.getLoc(), "out of range PC19 fixup");
      return 0;
    }
    break;
  case Fgpu::fixup_Fgpu_26:
    // So far we are only using this type for jumps.
    // The displacement is then divided by 4 to give us an 28 bit
    // address range.
    Value >>= 2;
    break;
  case Fgpu::fixup_Fgpu_HI16:
  case Fgpu::fixup_Fgpu_GOT:
  case Fgpu::fixup_MICROFGPU_GOT16:
  case Fgpu::fixup_Fgpu_GOT_HI16:
  case Fgpu::fixup_Fgpu_CALL_HI16:
  case Fgpu::fixup_MICROFGPU_HI16:
  case Fgpu::fixup_FGPU_PCHI16:
    // Get the 2nd 16-bits. Also add 1 if bit 15 is 1.
    Value = ((Value + 0x8000) >> 16) & 0xffff;
    break;
  case Fgpu::fixup_Fgpu_HIGHER:
  case Fgpu::fixup_MICROFGPU_HIGHER:
    // Get the 3rd 16-bits.
    Value = ((Value + 0x80008000LL) >> 32) & 0xffff;
    break;
  case Fgpu::fixup_Fgpu_HIGHEST:
  case Fgpu::fixup_MICROFGPU_HIGHEST:
    // Get the 4th 16-bits.
    Value = ((Value + 0x800080008000LL) >> 48) & 0xffff;
    break;
  case Fgpu::fixup_MICROFGPU_26_S1:
    Value >>= 1;
    break;
  case Fgpu::fixup_MICROFGPU_PC7_S1:
    Value -= 4;
    // Forcing a signed division because Value can be negative.
    Value = (int64_t) Value / 2;
    // We now check if Value can be encoded as a 7-bit signed immediate.
    if (!isInt<7>(Value)) {
      Ctx.reportError(Fixup.getLoc(), "out of range PC7 fixup");
      return 0;
    }
    break;
  case Fgpu::fixup_MICROFGPU_PC10_S1:
    Value -= 2;
    // Forcing a signed division because Value can be negative.
    Value = (int64_t) Value / 2;
    // We now check if Value can be encoded as a 10-bit signed immediate.
    if (!isInt<10>(Value)) {
      Ctx.reportError(Fixup.getLoc(), "out of range PC10 fixup");
      return 0;
    }
    break;
  case Fgpu::fixup_MICROFGPU_PC16_S1:
    Value -= 4;
    // Forcing a signed division because Value can be negative.
    Value = (int64_t)Value / 2;
    // We now check if Value can be encoded as a 16-bit signed immediate.
    if (!isInt<16>(Value)) {
      Ctx.reportError(Fixup.getLoc(), "out of range PC16 fixup");
      return 0;
    }
    break;
  case Fgpu::fixup_FGPU_PC18_S3:
    // Forcing a signed division because Value can be negative.
    Value = (int64_t)Value / 8;
    // We now check if Value can be encoded as a 18-bit signed immediate.
    if (!isInt<18>(Value)) {
      Ctx.reportError(Fixup.getLoc(), "out of range PC18 fixup");
      return 0;
    }
    break;
  case Fgpu::fixup_MICROFGPU_PC18_S3:
    // Check alignment.
    if ((Value & 7)) {
      Ctx.reportError(Fixup.getLoc(), "out of range PC18 fixup");
    }
    // Forcing a signed division because Value can be negative.
    Value = (int64_t)Value / 8;
    // We now check if Value can be encoded as a 18-bit signed immediate.
    if (!isInt<18>(Value)) {
      Ctx.reportError(Fixup.getLoc(), "out of range PC18 fixup");
      return 0;
    }
    break;
  case Fgpu::fixup_FGPU_PC21_S2:
    // Forcing a signed division because Value can be negative.
    Value = (int64_t) Value / 4;
    // We now check if Value can be encoded as a 21-bit signed immediate.
    if (!isInt<21>(Value)) {
      Ctx.reportError(Fixup.getLoc(), "out of range PC21 fixup");
      return 0;
    }
    break;
  case Fgpu::fixup_FGPU_PC26_S2:
    // Forcing a signed division because Value can be negative.
    Value = (int64_t) Value / 4;
    // We now check if Value can be encoded as a 26-bit signed immediate.
    if (!isInt<26>(Value)) {
      Ctx.reportError(Fixup.getLoc(), "out of range PC26 fixup");
      return 0;
    }
    break;
  case Fgpu::fixup_MICROFGPU_PC26_S1:
    // Forcing a signed division because Value can be negative.
    Value = (int64_t)Value / 2;
    // We now check if Value can be encoded as a 26-bit signed immediate.
    if (!isInt<26>(Value)) {
      Ctx.reportFatalError(Fixup.getLoc(), "out of range PC26 fixup");
      return 0;
    }
    break;
  case Fgpu::fixup_MICROFGPU_PC21_S1:
    // Forcing a signed division because Value can be negative.
    Value = (int64_t)Value / 2;
    // We now check if Value can be encoded as a 21-bit signed immediate.
    if (!isInt<21>(Value)) {
      Ctx.reportError(Fixup.getLoc(), "out of range PC21 fixup");
      return 0;
    }
    break;
  }

  return Value;
}

std::unique_ptr<MCObjectTargetWriter>
FgpuAsmBackend::createObjectTargetWriter() const {
  return createFgpuELFObjectWriter(TheTriple, IsN32);
}

// Little-endian fixup data byte ordering:
//   fgpu32r2:   a | b | x | x
//   microFGPU:  x | x | a | b

static bool needsMMLEByteOrder(unsigned Kind) {
  return Kind != Fgpu::fixup_MICROFGPU_PC10_S1 &&
         Kind >= Fgpu::fixup_MICROFGPU_26_S1 &&
         Kind < Fgpu::LastTargetFixupKind;
}

// Calculate index for microFGPU specific little endian byte order
static unsigned calculateMMLEIndex(unsigned i) {
  assert(i <= 3 && "Index out of range!");

  return (1 - i / 2) * 2 + i % 2;
}

/// ApplyFixup - Apply the \p Value for given \p Fixup into the provided
/// data fragment, at the offset specified by the fixup and following the
/// fixup kind as appropriate.
void FgpuAsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                                const MCValue &Target,
                                MutableArrayRef<char> Data, uint64_t Value,
                                bool IsResolved,
                                const MCSubtargetInfo *STI) const {
  MCFixupKind Kind = Fixup.getKind();
  MCContext &Ctx = Asm.getContext();
  Value = adjustFixupValue(Fixup, Value, Ctx);

  if (!Value)
    return; // Doesn't change encoding.

  // Where do we start in the object
  unsigned Offset = Fixup.getOffset();
  // Number of bytes we need to fixup
  unsigned NumBytes = (getFixupKindInfo(Kind).TargetSize + 7) / 8;
  // Used to point to big endian bytes
  unsigned FullSize;

  switch ((unsigned)Kind) {
  case FK_Data_2:
  case Fgpu::fixup_Fgpu_16:
  case Fgpu::fixup_MICROFGPU_PC10_S1:
    FullSize = 2;
    break;
  case FK_Data_8:
  case Fgpu::fixup_Fgpu_64:
    FullSize = 8;
    break;
  case FK_Data_4:
  default:
    FullSize = 4;
    break;
  }

  // Grab current value, if any, from bits.
  uint64_t CurVal = 0;

  bool microFgpuLEByteOrder = needsMMLEByteOrder((unsigned) Kind);

  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned Idx = Endian == support::little
                       ? (microFgpuLEByteOrder ? calculateMMLEIndex(i) : i)
                       : (FullSize - 1 - i);
    CurVal |= (uint64_t)((uint8_t)Data[Offset + Idx]) << (i*8);
  }

  uint64_t Mask = ((uint64_t)(-1) >>
                    (64 - getFixupKindInfo(Kind).TargetSize));
  CurVal |= Value & Mask;

  // Write out the fixed up bytes back to the code/data bits.
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned Idx = Endian == support::little
                       ? (microFgpuLEByteOrder ? calculateMMLEIndex(i) : i)
                       : (FullSize - 1 - i);
    Data[Offset + Idx] = (uint8_t)((CurVal >> (i*8)) & 0xff);
  }
}

Optional<MCFixupKind> FgpuAsmBackend::getFixupKind(StringRef Name) const {
  return StringSwitch<Optional<MCFixupKind>>(Name)
      .Case("R_FGPU_NONE", FK_NONE)
      .Case("R_FGPU_32", FK_Data_4)
      .Case("R_FGPU_CALL_HI16", (MCFixupKind)Fgpu::fixup_Fgpu_CALL_HI16)
      .Case("R_FGPU_CALL_LO16", (MCFixupKind)Fgpu::fixup_Fgpu_CALL_LO16)
      .Case("R_FGPU_CALL16", (MCFixupKind)Fgpu::fixup_Fgpu_CALL16)
      .Case("R_FGPU_GOT16", (MCFixupKind)Fgpu::fixup_Fgpu_GOT)
      .Case("R_FGPU_GOT_PAGE", (MCFixupKind)Fgpu::fixup_Fgpu_GOT_PAGE)
      .Case("R_FGPU_GOT_OFST", (MCFixupKind)Fgpu::fixup_Fgpu_GOT_OFST)
      .Case("R_FGPU_GOT_DISP", (MCFixupKind)Fgpu::fixup_Fgpu_GOT_DISP)
      .Case("R_FGPU_GOT_HI16", (MCFixupKind)Fgpu::fixup_Fgpu_GOT_HI16)
      .Case("R_FGPU_GOT_LO16", (MCFixupKind)Fgpu::fixup_Fgpu_GOT_LO16)
      .Case("R_FGPU_TLS_GOTTPREL", (MCFixupKind)Fgpu::fixup_Fgpu_GOTTPREL)
      .Case("R_FGPU_TLS_DTPREL_HI16", (MCFixupKind)Fgpu::fixup_Fgpu_DTPREL_HI)
      .Case("R_FGPU_TLS_DTPREL_LO16", (MCFixupKind)Fgpu::fixup_Fgpu_DTPREL_LO)
      .Case("R_FGPU_TLS_GD", (MCFixupKind)Fgpu::fixup_Fgpu_TLSGD)
      .Case("R_FGPU_TLS_LDM", (MCFixupKind)Fgpu::fixup_Fgpu_TLSLDM)
      .Case("R_FGPU_TLS_TPREL_HI16", (MCFixupKind)Fgpu::fixup_Fgpu_TPREL_HI)
      .Case("R_FGPU_TLS_TPREL_LO16", (MCFixupKind)Fgpu::fixup_Fgpu_TPREL_LO)
      .Case("R_MICROFGPU_CALL16", (MCFixupKind)Fgpu::fixup_MICROFGPU_CALL16)
      .Case("R_MICROFGPU_GOT_DISP", (MCFixupKind)Fgpu::fixup_MICROFGPU_GOT_DISP)
      .Case("R_MICROFGPU_GOT_PAGE", (MCFixupKind)Fgpu::fixup_MICROFGPU_GOT_PAGE)
      .Case("R_MICROFGPU_GOT_OFST", (MCFixupKind)Fgpu::fixup_MICROFGPU_GOT_OFST)
      .Case("R_MICROFGPU_GOT16", (MCFixupKind)Fgpu::fixup_MICROFGPU_GOT16)
      .Case("R_MICROFGPU_TLS_GOTTPREL",
            (MCFixupKind)Fgpu::fixup_MICROFGPU_GOTTPREL)
      .Case("R_MICROFGPU_TLS_DTPREL_HI16",
            (MCFixupKind)Fgpu::fixup_MICROFGPU_TLS_DTPREL_HI16)
      .Case("R_MICROFGPU_TLS_DTPREL_LO16",
            (MCFixupKind)Fgpu::fixup_MICROFGPU_TLS_DTPREL_LO16)
      .Case("R_MICROFGPU_TLS_GD", (MCFixupKind)Fgpu::fixup_MICROFGPU_TLS_GD)
      .Case("R_MICROFGPU_TLS_LDM", (MCFixupKind)Fgpu::fixup_MICROFGPU_TLS_LDM)
      .Case("R_MICROFGPU_TLS_TPREL_HI16",
            (MCFixupKind)Fgpu::fixup_MICROFGPU_TLS_TPREL_HI16)
      .Case("R_MICROFGPU_TLS_TPREL_LO16",
            (MCFixupKind)Fgpu::fixup_MICROFGPU_TLS_TPREL_LO16)
      .Case("R_FGPU_JALR", (MCFixupKind)Fgpu::fixup_Fgpu_JALR)
      .Case("R_MICROFGPU_JALR", (MCFixupKind)Fgpu::fixup_MICROFGPU_JALR)
      .Default(MCAsmBackend::getFixupKind(Name));
}

const MCFixupKindInfo &FgpuAsmBackend::
getFixupKindInfo(MCFixupKind Kind) const {
  const static MCFixupKindInfo LittleEndianInfos[] = {
    // This table *must* be in same the order of fixup_* kinds in
    // FgpuFixupKinds.h.
    //
    // name                    offset  bits  flags
    { "fixup_Fgpu_16",           0,     16,   0 },
    { "fixup_Fgpu_PC14",         0,     14,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_Fgpu_PC7",          0,     7,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_Fgpu_JSUB",         0,     14,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_Fgpu_32",           0,     32,   0 },
    { "fixup_Fgpu_REL32",        0,     32,   0 },
    { "fixup_Fgpu_26",           0,     26,   0 },
    { "fixup_Fgpu_HI16",         0,     16,   0 },
    { "fixup_Fgpu_LO16",         0,     16,   0 },
    { "fixup_Fgpu_GPREL16",      0,     16,   0 },
    { "fixup_Fgpu_LITERAL",      0,     16,   0 },
    { "fixup_Fgpu_GOT",          0,     16,   0 },
    { "fixup_Fgpu_PC16",         0,     16,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_Fgpu_CALL16",       0,     16,   0 },
    { "fixup_Fgpu_GPREL32",      0,     32,   0 },
    { "fixup_Fgpu_SHIFT5",       6,      5,   0 },
    { "fixup_Fgpu_SHIFT6",       6,      5,   0 },
    { "fixup_Fgpu_64",           0,     64,   0 },
    { "fixup_Fgpu_TLSGD",        0,     16,   0 },
    { "fixup_Fgpu_GOTTPREL",     0,     16,   0 },
    { "fixup_Fgpu_TPREL_HI",     0,     16,   0 },
    { "fixup_Fgpu_TPREL_LO",     0,     16,   0 },
    { "fixup_Fgpu_TLSLDM",       0,     16,   0 },
    { "fixup_Fgpu_DTPREL_HI",    0,     16,   0 },
    { "fixup_Fgpu_DTPREL_LO",    0,     16,   0 },
    { "fixup_Fgpu_Branch_PCRel", 0,     16,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_Fgpu_GPOFF_HI",     0,     16,   0 },
    { "fixup_MICROFGPU_GPOFF_HI",0,     16,   0 },
    { "fixup_Fgpu_GPOFF_LO",     0,     16,   0 },
    { "fixup_MICROFGPU_GPOFF_LO",0,     16,   0 },
    { "fixup_Fgpu_GOT_PAGE",     0,     16,   0 },
    { "fixup_Fgpu_GOT_OFST",     0,     16,   0 },
    { "fixup_Fgpu_GOT_DISP",     0,     16,   0 },
    { "fixup_Fgpu_HIGHER",       0,     16,   0 },
    { "fixup_MICROFGPU_HIGHER",  0,     16,   0 },
    { "fixup_Fgpu_HIGHEST",      0,     16,   0 },
    { "fixup_MICROFGPU_HIGHEST", 0,     16,   0 },
    { "fixup_Fgpu_GOT_HI16",     0,     16,   0 },
    { "fixup_Fgpu_GOT_LO16",     0,     16,   0 },
    { "fixup_Fgpu_CALL_HI16",    0,     16,   0 },
    { "fixup_Fgpu_CALL_LO16",    0,     16,   0 },
    { "fixup_Fgpu_PC18_S3",      0,     18,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_FGPU_PC19_S2",      0,     19,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_FGPU_PC21_S2",      0,     21,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_FGPU_PC26_S2",      0,     26,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_FGPU_PCHI16",       0,     16,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_FGPU_PCLO16",       0,     16,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_26_S1",   0,     26,   0 },
    { "fixup_MICROFGPU_HI16",    0,     16,   0 },
    { "fixup_MICROFGPU_LO16",    0,     16,   0 },
    { "fixup_MICROFGPU_GOT16",   0,     16,   0 },
    { "fixup_MICROFGPU_PC7_S1",  0,      7,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_PC10_S1", 0,     10,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_PC16_S1", 0,     16,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_PC26_S1", 0,     26,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_PC19_S2", 0,     19,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_PC18_S3", 0,     18,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_PC21_S1", 0,     21,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_CALL16",  0,     16,   0 },
    { "fixup_MICROFGPU_GOT_DISP",        0,     16,   0 },
    { "fixup_MICROFGPU_GOT_PAGE",        0,     16,   0 },
    { "fixup_MICROFGPU_GOT_OFST",        0,     16,   0 },
    { "fixup_MICROFGPU_TLS_GD",          0,     16,   0 },
    { "fixup_MICROFGPU_TLS_LDM",         0,     16,   0 },
    { "fixup_MICROFGPU_TLS_DTPREL_HI16", 0,     16,   0 },
    { "fixup_MICROFGPU_TLS_DTPREL_LO16", 0,     16,   0 },
    { "fixup_MICROFGPU_GOTTPREL",        0,     16,   0 },
    { "fixup_MICROFGPU_TLS_TPREL_HI16",  0,     16,   0 },
    { "fixup_MICROFGPU_TLS_TPREL_LO16",  0,     16,   0 },
    { "fixup_Fgpu_SUB",                  0,     64,   0 },
    { "fixup_MICROFGPU_SUB",             0,     64,   0 },
    { "fixup_Fgpu_JALR",                 0,     32,   0 },
    { "fixup_MICROFGPU_JALR",            0,     32,   0 }
  };
  static_assert(array_lengthof(LittleEndianInfos) == Fgpu::NumTargetFixupKinds,
                "Not all FGPU little endian fixup kinds added!");

  const static MCFixupKindInfo BigEndianInfos[] = {
    // This table *must* be in same the order of fixup_* kinds in
    // FgpuFixupKinds.h.
    //
    // name                    offset  bits  flags
    { "fixup_Fgpu_16",          16,     16,   0 },
    { "fixup_Fgpu_PC14",         0,     14,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_Fgpu_PC7",          0,     7,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_Fgpu_JSUB",         0,     14,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_Fgpu_32",           0,     32,   0 },
    { "fixup_Fgpu_REL32",        0,     32,   0 },
    { "fixup_Fgpu_26",           6,     26,   0 },
    { "fixup_Fgpu_HI16",        16,     16,   0 },
    { "fixup_Fgpu_LO16",        16,     16,   0 },
    { "fixup_Fgpu_GPREL16",     16,     16,   0 },
    { "fixup_Fgpu_LITERAL",     16,     16,   0 },
    { "fixup_Fgpu_GOT",         16,     16,   0 },
    { "fixup_Fgpu_PC16",        16,     16,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_Fgpu_CALL16",      16,     16,   0 },
    { "fixup_Fgpu_GPREL32",      0,     32,   0 },
    { "fixup_Fgpu_SHIFT5",      21,      5,   0 },
    { "fixup_Fgpu_SHIFT6",      21,      5,   0 },
    { "fixup_Fgpu_64",           0,     64,   0 },
    { "fixup_Fgpu_TLSGD",       16,     16,   0 },
    { "fixup_Fgpu_GOTTPREL",    16,     16,   0 },
    { "fixup_Fgpu_TPREL_HI",    16,     16,   0 },
    { "fixup_Fgpu_TPREL_LO",    16,     16,   0 },
    { "fixup_Fgpu_TLSLDM",      16,     16,   0 },
    { "fixup_Fgpu_DTPREL_HI",   16,     16,   0 },
    { "fixup_Fgpu_DTPREL_LO",   16,     16,   0 },
    { "fixup_Fgpu_Branch_PCRel",16,     16,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_Fgpu_GPOFF_HI",    16,     16,   0 },
    { "fixup_MICROFGPU_GPOFF_HI", 16,     16,   0 },
    { "fixup_Fgpu_GPOFF_LO",    16,     16,   0 },
    { "fixup_MICROFGPU_GPOFF_LO", 16,     16,   0 },
    { "fixup_Fgpu_GOT_PAGE",    16,     16,   0 },
    { "fixup_Fgpu_GOT_OFST",    16,     16,   0 },
    { "fixup_Fgpu_GOT_DISP",    16,     16,   0 },
    { "fixup_Fgpu_HIGHER",      16,     16,   0 },
    { "fixup_MICROFGPU_HIGHER", 16,     16,   0 },
    { "fixup_Fgpu_HIGHEST",     16,     16,   0 },
    { "fixup_MICROFGPU_HIGHEST",16,     16,   0 },
    { "fixup_Fgpu_GOT_HI16",    16,     16,   0 },
    { "fixup_Fgpu_GOT_LO16",    16,     16,   0 },
    { "fixup_Fgpu_CALL_HI16",   16,     16,   0 },
    { "fixup_Fgpu_CALL_LO16",   16,     16,   0 },
    { "fixup_Fgpu_PC18_S3",     14,     18,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_FGPU_PC19_S2",     13,     19,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_FGPU_PC21_S2",     11,     21,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_FGPU_PC26_S2",      6,     26,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_FGPU_PCHI16",      16,     16,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_FGPU_PCLO16",      16,     16,  MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_26_S1",   6,     26,   0 },
    { "fixup_MICROFGPU_HI16",   16,     16,   0 },
    { "fixup_MICROFGPU_LO16",   16,     16,   0 },
    { "fixup_MICROFGPU_GOT16",  16,     16,   0 },
    { "fixup_MICROFGPU_PC7_S1",  9,      7,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_PC10_S1", 6,     10,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_PC16_S1",16,     16,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_PC26_S1", 6,     26,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_PC19_S2",13,     19,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_PC18_S3",14,     18,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_PC21_S1",11,     21,   MCFixupKindInfo::FKF_IsPCRel },
    { "fixup_MICROFGPU_CALL16", 16,     16,   0 },
    { "fixup_MICROFGPU_GOT_DISP",        16,     16,   0 },
    { "fixup_MICROFGPU_GOT_PAGE",        16,     16,   0 },
    { "fixup_MICROFGPU_GOT_OFST",        16,     16,   0 },
    { "fixup_MICROFGPU_TLS_GD",          16,     16,   0 },
    { "fixup_MICROFGPU_TLS_LDM",         16,     16,   0 },
    { "fixup_MICROFGPU_TLS_DTPREL_HI16", 16,     16,   0 },
    { "fixup_MICROFGPU_TLS_DTPREL_LO16", 16,     16,   0 },
    { "fixup_MICROFGPU_GOTTPREL",        16,     16,   0 },
    { "fixup_MICROFGPU_TLS_TPREL_HI16",  16,     16,   0 },
    { "fixup_MICROFGPU_TLS_TPREL_LO16",  16,     16,   0 },
    { "fixup_Fgpu_SUB",                   0,     64,   0 },
    { "fixup_MICROFGPU_SUB",              0,     64,   0 },
    { "fixup_Fgpu_JALR",                  0,     32,   0 },
    { "fixup_MICROFGPU_JALR",             0,     32,   0 }
  };
  static_assert(array_lengthof(BigEndianInfos) == Fgpu::NumTargetFixupKinds,
                "Not all FGPU big endian fixup kinds added!");

  if (Kind < FirstTargetFixupKind)
    return MCAsmBackend::getFixupKindInfo(Kind);

  assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
          "Invalid kind!");

  if (Endian == support::little)
    return LittleEndianInfos[Kind - FirstTargetFixupKind];
  return BigEndianInfos[Kind - FirstTargetFixupKind];
}

/// WriteNopData - Write an (optimal) nop sequence of Count bytes
/// to the given output. If the target cannot generate such a sequence,
/// it should return an error.
///
/// \return - True on success.
bool FgpuAsmBackend::writeNopData(raw_ostream &OS, uint64_t Count) const {
  // Check for a less than instruction size number of bytes
  // FIXME: 16 bit instructions are not handled yet here.
  // We shouldn't be using a hard coded number for instruction size.

  // If the count is not 4-byte aligned, we must be writing data into the text
  // section (otherwise we have unaligned instructions, and thus have far
  // bigger problems), so just write zeros instead.
  OS.write_zeros(Count);
  return true;
}

bool FgpuAsmBackend::shouldForceRelocation(const MCAssembler &Asm,
                                           const MCFixup &Fixup,
                                           const MCValue &Target) {
  const unsigned FixupKind = Fixup.getKind();
  switch (FixupKind) {
  default:
    return false;
  // All these relocations require special processing
  // at linking time. Delegate this work to a linker.
  case Fgpu::fixup_Fgpu_CALL_HI16:
  case Fgpu::fixup_Fgpu_CALL_LO16:
  case Fgpu::fixup_Fgpu_CALL16:
  case Fgpu::fixup_Fgpu_GOT:
  case Fgpu::fixup_Fgpu_GOT_PAGE:
  case Fgpu::fixup_Fgpu_GOT_OFST:
  case Fgpu::fixup_Fgpu_GOT_DISP:
  case Fgpu::fixup_Fgpu_GOT_HI16:
  case Fgpu::fixup_Fgpu_GOT_LO16:
  case Fgpu::fixup_Fgpu_GOTTPREL:
  case Fgpu::fixup_Fgpu_DTPREL_HI:
  case Fgpu::fixup_Fgpu_DTPREL_LO:
  case Fgpu::fixup_Fgpu_TLSGD:
  case Fgpu::fixup_Fgpu_TLSLDM:
  case Fgpu::fixup_Fgpu_TPREL_HI:
  case Fgpu::fixup_Fgpu_TPREL_LO:
  case Fgpu::fixup_Fgpu_JALR:
  case Fgpu::fixup_MICROFGPU_CALL16:
  case Fgpu::fixup_MICROFGPU_GOT_DISP:
  case Fgpu::fixup_MICROFGPU_GOT_PAGE:
  case Fgpu::fixup_MICROFGPU_GOT_OFST:
  case Fgpu::fixup_MICROFGPU_GOT16:
  case Fgpu::fixup_MICROFGPU_GOTTPREL:
  case Fgpu::fixup_MICROFGPU_TLS_DTPREL_HI16:
  case Fgpu::fixup_MICROFGPU_TLS_DTPREL_LO16:
  case Fgpu::fixup_MICROFGPU_TLS_GD:
  case Fgpu::fixup_MICROFGPU_TLS_LDM:
  case Fgpu::fixup_MICROFGPU_TLS_TPREL_HI16:
  case Fgpu::fixup_MICROFGPU_TLS_TPREL_LO16:
  case Fgpu::fixup_MICROFGPU_JALR:
    return true;
  }
}

MCAsmBackend *llvm::createFgpuAsmBackend(const Target &T,
                                         const MCSubtargetInfo &STI,
                                         const MCRegisterInfo &MRI,
                                         const MCTargetOptions &Options) {
  FgpuABIInfo ABI = FgpuABIInfo::computeTargetABI(STI.getTargetTriple(),
                                                  STI.getCPU(), Options);
  return new FgpuAsmBackend(T, MRI, STI.getTargetTriple(), STI.getCPU(),
                            true);
}
