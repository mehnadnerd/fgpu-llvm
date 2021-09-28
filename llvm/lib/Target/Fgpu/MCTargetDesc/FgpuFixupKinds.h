//===-- FgpuFixupKinds.h - Fgpu Specific Fixup Entries ----------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUFIXUPKINDS_H
#define LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUFIXUPKINDS_H

#include "llvm/MC/MCFixup.h"

namespace llvm {
namespace Fgpu {
  // Although most of the current fixup types reflect a unique relocation
  // one can have multiple fixup types for a given relocation and thus need
  // to be uniquely named.
  //
  // This table *must* be in the same order of
  // MCFixupKindInfo Infos[Fgpu::NumTargetFixupKinds]
  // in FgpuAsmBackend.cpp.
  //
  enum Fixups {
    // Branch fixups resulting in R_FGPU_16.
    fixup_Fgpu_16 = FirstTargetFixupKind,

    fixup_Fgpu_PC14,
    fixup_Fgpu_PC7,
    fixup_Fgpu_JSUB,

    // Pure 32 bit data fixup resulting in - R_FGPU_32.
    fixup_Fgpu_32,

    // Full 32 bit data relative data fixup resulting in - R_FGPU_REL32.
    fixup_Fgpu_REL32,

    // Jump 26 bit fixup resulting in - R_FGPU_26.
    fixup_Fgpu_26,

    // Pure upper 16 bit fixup resulting in - R_FGPU_HI16.
    fixup_Fgpu_HI16,

    // Pure lower 16 bit fixup resulting in - R_FGPU_LO16.
    fixup_Fgpu_LO16,

    // 16 bit fixup for GP offest resulting in - R_FGPU_GPREL16.
    fixup_Fgpu_GPREL16,

    // 16 bit literal fixup resulting in - R_FGPU_LITERAL.
    fixup_Fgpu_LITERAL,

    // Symbol fixup resulting in - R_FGPU_GOT16.
    fixup_Fgpu_GOT,

    // PC relative branch fixup resulting in - R_FGPU_PC16.
    fixup_Fgpu_PC16,

    // resulting in - R_FGPU_CALL16.
    fixup_Fgpu_CALL16,

    // resulting in - R_FGPU_GPREL32.
    fixup_Fgpu_GPREL32,

    // resulting in - R_FGPU_SHIFT5.
    fixup_Fgpu_SHIFT5,

    // resulting in - R_FGPU_SHIFT6.
    fixup_Fgpu_SHIFT6,

    // Pure 64 bit data fixup resulting in - R_FGPU_64.
    fixup_Fgpu_64,

    // resulting in - R_FGPU_TLS_GD.
    fixup_Fgpu_TLSGD,

    // resulting in - R_FGPU_TLS_GOTTPREL.
    fixup_Fgpu_GOTTPREL,

    // resulting in - R_FGPU_TLS_TPREL_HI16.
    fixup_Fgpu_TPREL_HI,

    // resulting in - R_FGPU_TLS_TPREL_LO16.
    fixup_Fgpu_TPREL_LO,

    // resulting in - R_FGPU_TLS_LDM.
    fixup_Fgpu_TLSLDM,

    // resulting in - R_FGPU_TLS_DTPREL_HI16.
    fixup_Fgpu_DTPREL_HI,

    // resulting in - R_FGPU_TLS_DTPREL_LO16.
    fixup_Fgpu_DTPREL_LO,

    // PC relative branch fixup resulting in - R_FGPU_PC16
    fixup_Fgpu_Branch_PCRel,

    // resulting in - R_FGPU_GPREL16/R_FGPU_SUB/R_FGPU_HI16
    //                R_MICROFGPU_GPREL16/R_MICROFGPU_SUB/R_MICROFGPU_HI16
    fixup_Fgpu_GPOFF_HI,
    fixup_MICROFGPU_GPOFF_HI,

    // resulting in - R_FGPU_GPREL16/R_FGPU_SUB/R_FGPU_LO16
    //                R_MICROFGPU_GPREL16/R_MICROFGPU_SUB/R_MICROFGPU_LO16
    fixup_Fgpu_GPOFF_LO,
    fixup_MICROFGPU_GPOFF_LO,

    // resulting in - R_FGPU_PAGE
    fixup_Fgpu_GOT_PAGE,

    // resulting in - R_FGPU_GOT_OFST
    fixup_Fgpu_GOT_OFST,

    // resulting in - R_FGPU_GOT_DISP
    fixup_Fgpu_GOT_DISP,

    // resulting in - R_FGPU_HIGHER/R_MICROFGPU_HIGHER
    fixup_Fgpu_HIGHER,
    fixup_MICROFGPU_HIGHER,

    // resulting in - R_FGPU_HIGHEST/R_MICROFGPU_HIGHEST
    fixup_Fgpu_HIGHEST,
    fixup_MICROFGPU_HIGHEST,

    // resulting in - R_FGPU_GOT_HI16
    fixup_Fgpu_GOT_HI16,

    // resulting in - R_FGPU_GOT_LO16
    fixup_Fgpu_GOT_LO16,

    // resulting in - R_FGPU_CALL_HI16
    fixup_Fgpu_CALL_HI16,

    // resulting in - R_FGPU_CALL_LO16
    fixup_Fgpu_CALL_LO16,

    // resulting in - R_FGPU_PC18_S3
    fixup_FGPU_PC18_S3,

    // resulting in - R_FGPU_PC19_S2
    fixup_FGPU_PC19_S2,

    // resulting in - R_FGPU_PC21_S2
    fixup_FGPU_PC21_S2,

    // resulting in - R_FGPU_PC26_S2
    fixup_FGPU_PC26_S2,

    // resulting in - R_FGPU_PCHI16
    fixup_FGPU_PCHI16,

    // resulting in - R_FGPU_PCLO16
    fixup_FGPU_PCLO16,

    // resulting in - R_MICROFGPU_26_S1
    fixup_MICROFGPU_26_S1,

    // resulting in - R_MICROFGPU_HI16
    fixup_MICROFGPU_HI16,

    // resulting in - R_MICROFGPU_LO16
    fixup_MICROFGPU_LO16,

    // resulting in - R_MICROFGPU_GOT16
    fixup_MICROFGPU_GOT16,

    // resulting in - R_MICROFGPU_PC7_S1
    fixup_MICROFGPU_PC7_S1,

    // resulting in - R_MICROFGPU_PC10_S1
    fixup_MICROFGPU_PC10_S1,

    // resulting in - R_MICROFGPU_PC16_S1
    fixup_MICROFGPU_PC16_S1,

    // resulting in - R_MICROFGPU_PC26_S1
    fixup_MICROFGPU_PC26_S1,

    // resulting in - R_MICROFGPU_PC19_S2
    fixup_MICROFGPU_PC19_S2,

    // resulting in - R_MICROFGPU_PC18_S3
    fixup_MICROFGPU_PC18_S3,

    // resulting in - R_MICROFGPU_PC21_S1
    fixup_MICROFGPU_PC21_S1,

    // resulting in - R_MICROFGPU_CALL16
    fixup_MICROFGPU_CALL16,

    // resulting in - R_MICROFGPU_GOT_DISP
    fixup_MICROFGPU_GOT_DISP,

    // resulting in - R_MICROFGPU_GOT_PAGE
    fixup_MICROFGPU_GOT_PAGE,

    // resulting in - R_MICROFGPU_GOT_OFST
    fixup_MICROFGPU_GOT_OFST,

    // resulting in - R_MICROFGPU_TLS_GD
    fixup_MICROFGPU_TLS_GD,

    // resulting in - R_MICROFGPU_TLS_LDM
    fixup_MICROFGPU_TLS_LDM,

    // resulting in - R_MICROFGPU_TLS_DTPREL_HI16
    fixup_MICROFGPU_TLS_DTPREL_HI16,

    // resulting in - R_MICROFGPU_TLS_DTPREL_LO16
    fixup_MICROFGPU_TLS_DTPREL_LO16,

    // resulting in - R_MICROFGPU_TLS_GOTTPREL.
    fixup_MICROFGPU_GOTTPREL,

    // resulting in - R_MICROFGPU_TLS_TPREL_HI16
    fixup_MICROFGPU_TLS_TPREL_HI16,

    // resulting in - R_MICROFGPU_TLS_TPREL_LO16
    fixup_MICROFGPU_TLS_TPREL_LO16,

    // resulting in - R_FGPU_SUB/R_MICROFGPU_SUB
    fixup_Fgpu_SUB,
    fixup_MICROFGPU_SUB,

    // resulting in - R_FGPU_JALR/R_MICROFGPU_JALR
    fixup_Fgpu_JALR,
    fixup_MICROFGPU_JALR,

    // Marker
    LastTargetFixupKind,
    NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
  };
} // namespace Fgpu
} // namespace llvm


#endif
