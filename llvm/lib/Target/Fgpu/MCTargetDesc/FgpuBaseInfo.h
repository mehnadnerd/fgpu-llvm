//===-- FgpuBaseInfo.h - Top level definitions for FGPU MC ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains small standalone helper functions and enum definitions for
// the Fgpu target useful for the compiler back-end and the MC libraries.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUBASEINFO_H
#define LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUBASEINFO_H

#include "FgpuFixupKinds.h"
#include "FgpuMCTargetDesc.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/Support/DataTypes.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// FgpuII - This namespace holds all of the target specific flags that
/// instruction info tracks.
///
namespace FgpuII {
  /// Target Operand Flag enum.
  enum TOF {
    //===------------------------------------------------------------------===//
    // Fgpu Specific MachineOperand flags.

    MO_NO_FLAG,

    /// MO_GOT - Represents the offset into the global offset table at which
    /// the address the relocation entry symbol resides during execution.
    MO_GOT,

    /// MO_GOT_CALL - Represents the offset into the global offset table at
    /// which the address of a call site relocation entry symbol resides
    /// during execution. This is different from the above since this flag
    /// can only be present in call instructions.
    MO_GOT_CALL,

    /// MO_GPREL - Represents the offset from the current gp value to be used
    /// for the relocatable object file being produced.
    MO_GPREL,

    /// MO_ABS_HI/LO - Represents the hi or low part of an absolute symbol
    /// address.
    MO_ABS_HI,
    MO_ABS_LO,

    /// MO_TLSGD - Represents the offset into the global offset table at which
    // the module ID and TSL block offset reside during execution (General
    // Dynamic TLS).
    MO_TLSGD,

    /// MO_TLSLDM - Represents the offset into the global offset table at which
    // the module ID and TSL block offset reside during execution (Local
    // Dynamic TLS).
    MO_TLSLDM,
    MO_DTPREL_HI,
    MO_DTPREL_LO,

    /// MO_GOTTPREL - Represents the offset from the thread pointer (Initial
    // Exec TLS).
    MO_GOTTPREL,

    /// MO_TPREL_HI/LO - Represents the hi and low part of the offset from
    // the thread pointer (Local Exec TLS).
    MO_TPREL_HI,
    MO_TPREL_LO,

    // N32/64 Flags.
    MO_GPOFF_HI,
    MO_GPOFF_LO,
    MO_GOT_DISP,
    MO_GOT_PAGE,
    MO_GOT_OFST,

    ///// MO_HIGHER/HIGHEST - Represents the highest or higher half word of a
    ///// 64-bit symbol address.
    //MO_HIGHER,
    //MO_HIGHEST,

    /// MO_GOT_HI16/LO16, MO_CALL_HI16/LO16 - Relocations used for large GOTs.
    MO_GOT_HI16,
    MO_GOT_LO16,
    MO_CALL_HI16,
    MO_CALL_LO16,

    /// Helper operand used to generate R_FGPU_JALR
    MO_JALR
  };

  enum {
    //===------------------------------------------------------------------===//
    // Instruction encodings.  These are the standard/most common forms for
    // Fgpu instructions.
    //

    // Pseudo - This represents an instruction that is a pseudo instruction
    // or one that has not been implemented yet.  It is illegal to code generate
    // it, but tolerated for intermediate implementation stages.
    FrmPseudo = 0,

    /// This form is for instructions of the format RRR.
    FrmRRR  = 1,
    /// This form is for instructions of the format RRI.
    FrmRRI  = 2,
    /// FrmOther - This form is for instructions that have no specific format.
    FrmRI = 3,
    // ret operation
    FrmCtrl = 4,
    FrmMask = 15

  };

  enum OperandType : unsigned {
    OPERAND_FIRST_FGPU_MEM_IMM = MCOI::OPERAND_FIRST_TARGET,
    OPERAND_MEM_SIMM9 = OPERAND_FIRST_FGPU_MEM_IMM,
    OPERAND_LAST_FGPU_MEM_IMM = OPERAND_MEM_SIMM9
  };
} // namespace FgpuII
} // namespace llvm

#endif
