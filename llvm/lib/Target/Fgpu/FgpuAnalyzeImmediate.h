//===- FgpuAnalyzeImmediate.h - Analyze Immediates -------------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_FGPU_FGPUANALYZEIMMEDIATE_H
#define LLVM_LIB_TARGET_FGPU_FGPUANALYZEIMMEDIATE_H

#include "llvm/ADT/SmallVector.h"
#include <cstdint>

namespace llvm {

  class FgpuAnalyzeImmediate {
  public:
    struct Inst {
      unsigned Opc, ImmOpnd;

      Inst(unsigned Opc, unsigned ImmOpnd);
    };
    using InstSeq = SmallVector<Inst, 7>;

    /// Analyze - Get an instruction sequence to load immediate Imm. The last
    /// instruction in the sequence must be an ADDiu if LastInstrIsADDiu is
    /// true;
    const InstSeq &Analyze(uint64_t Imm, unsigned Size, bool LastInstrIsADDiu);

  private:
    using InstSeqLs = SmallVector<InstSeq, 5>;

    /// AddInstr - Add I to all instruction sequences in SeqLs.
    void AddInstr(InstSeqLs &SeqLs, const Inst &I);

    /// GetInstSeqLs - Get instrucion sequences to load immediate Imm.
    void GetInstSeqLs(uint64_t Imm, unsigned RemSize, InstSeqLs &SeqLs);

    unsigned Size;
    unsigned Li, LUi;
    InstSeq Insts;
  };

} // end namespace llvm

#endif // LLVM_LIB_TARGET_FGPU_FGPUANALYZEIMMEDIATE_H
