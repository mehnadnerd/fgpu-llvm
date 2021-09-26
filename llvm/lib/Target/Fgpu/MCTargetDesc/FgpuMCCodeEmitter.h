//===- FgpuMCCodeEmitter.h - Convert Fgpu Code to Machine Code --*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the FgpuMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUMCCODEEMITTER_H
#define LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUMCCODEEMITTER_H

#include "llvm/MC/MCCodeEmitter.h"
#include <cstdint>

namespace llvm {

class MCContext;
class MCExpr;
class MCFixup;
class MCInst;
class MCInstrInfo;
class MCOperand;
class MCSubtargetInfo;
class raw_ostream;

class FgpuMCCodeEmitter : public MCCodeEmitter {
  const MCInstrInfo &MCII;
  MCContext &Ctx;
  bool IsLittleEndian;

public:
  FgpuMCCodeEmitter(const MCInstrInfo &mcii, MCContext &Ctx_, bool IsLittle)
      : MCII(mcii), Ctx(Ctx_), IsLittleEndian(IsLittle) {}
  FgpuMCCodeEmitter(const FgpuMCCodeEmitter &) = delete;
  FgpuMCCodeEmitter &operator=(const FgpuMCCodeEmitter &) = delete;
  ~FgpuMCCodeEmitter() override = default;

  void EmitByte(unsigned char C, raw_ostream &OS) const;

  void emitInstruction(uint64_t Val, unsigned Size, const MCSubtargetInfo &STI,
                       raw_ostream &OS) const;

  void encodeInstruction(const MCInst &MI, raw_ostream &OS,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const override;

  // getBinaryCodeForInstr - TableGen'erated function for getting the
  // binary encoding for an instruction.
  uint64_t getBinaryCodeForInstr(const MCInst &MI,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;

  // getJumpTargetOpValue - Return binary encoding of the jump
  // target operand. If the machine operand requires relocation,
  // record the relocation and return zero.
  unsigned getJumpTargetOpValue(const MCInst &MI, unsigned OpNo,
                                SmallVectorImpl<MCFixup> &Fixups,
                                const MCSubtargetInfo &STI) const;


  // getUImm5Lsl2Encoding - Return binary encoding of the microFGPU jump
  // target operand.
  unsigned getUImm5Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                                SmallVectorImpl<MCFixup> &Fixups,
                                const MCSubtargetInfo &STI) const;

  unsigned getSImm3Lsa2Value(const MCInst &MI, unsigned OpNo,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

  unsigned getUImm6Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                                SmallVectorImpl<MCFixup> &Fixups,
                                const MCSubtargetInfo &STI) const;

  // getSImm9AddiuspValue - Return binary encoding of the microFGPU addiusp
  // instruction immediate operand.
  unsigned getSImm9AddiuspValue(const MCInst &MI, unsigned OpNo,
                                SmallVectorImpl<MCFixup> &Fixups,
                                const MCSubtargetInfo &STI) const;

  // getBranchTargetOpValue - Return binary encoding of the branch
  // target operand. If the machine operand requires relocation,
  // record the relocation and return zero.
  unsigned getBranchTargetOpValue(const MCInst &MI, unsigned OpNo,
                                  SmallVectorImpl<MCFixup> &Fixups,
                                  const MCSubtargetInfo &STI) const;

  // getBranchTargetOpValue1SImm16 - Return binary encoding of the branch
  // target operand. If the machine operand requires relocation,
  // record the relocation and return zero.
  unsigned getBranchTargetOpValue1SImm16(const MCInst &MI, unsigned OpNo,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const;

  // getBranchTargetOpValueMMR6 - Return binary encoding of the branch
  // target operand. If the machine operand requires relocation,
  // record the relocation and return zero.
  unsigned getBranchTargetOpValueMMR6(const MCInst &MI, unsigned OpNo,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const;

  // getBranchTarget21OpValue - Return binary encoding of the branch
  // offset operand. If the machine operand requires relocation,
  // record the relocation and return zero.
  unsigned getBranchTarget21OpValue(const MCInst &MI, unsigned OpNo,
                                   SmallVectorImpl<MCFixup> &Fixups,
                                   const MCSubtargetInfo &STI) const;

  // getBranchTarget21OpValueMM - Return binary encoding of the branch
  // offset operand for microFGPU. If the machine operand requires
  // relocation,record the relocation and return zero.
  unsigned getBranchTarget21OpValueMM(const MCInst &MI, unsigned OpNo,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const;

  // getBranchTarget26OpValue - Return binary encoding of the branch
  // offset operand. If the machine operand requires relocation,
  // record the relocation and return zero.
  unsigned getBranchTarget26OpValue(const MCInst &MI, unsigned OpNo,
                                    SmallVectorImpl<MCFixup> &Fixups,
                                    const MCSubtargetInfo &STI) const;

  // getBranchTarget26OpValueMM - Return binary encoding of the branch
  // offset operand. If the machine operand requires relocation,
  // record the relocation and return zero.
  unsigned getBranchTarget26OpValueMM(const MCInst &MI, unsigned OpNo,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const;

  // getJumpOffset16OpValue - Return binary encoding of the jump
  // offset operand. If the machine operand requires relocation,
  // record the relocation and return zero.
  unsigned getJumpOffset16OpValue(const MCInst &MI, unsigned OpNo,
                                  SmallVectorImpl<MCFixup> &Fixups,
                                  const MCSubtargetInfo &STI) const;

  // getMachineOpValue - Return binary encoding of operand. If the machin
  // operand requires relocation, record the relocation and return zero.
  unsigned getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;


  template <unsigned ShiftAmount = 0>
  unsigned getMemEncoding(const MCInst &MI, unsigned OpNo,
                          SmallVectorImpl<MCFixup> &Fixups,
                          const MCSubtargetInfo &STI) const;
  unsigned getSizeInsEncoding(const MCInst &MI, unsigned OpNo,
                              SmallVectorImpl<MCFixup> &Fixups,
                              const MCSubtargetInfo &STI) const;

  /// Subtract Offset then encode as a N-bit unsigned integer.
  template <unsigned Bits, int Offset>
  unsigned getUImmWithOffsetEncoding(const MCInst &MI, unsigned OpNo,
                                     SmallVectorImpl<MCFixup> &Fixups,
                                     const MCSubtargetInfo &STI) const;

  unsigned getSimm19Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;

  unsigned getSimm18Lsl3Encoding(const MCInst &MI, unsigned OpNo,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;

  unsigned getUImm3Mod8Encoding(const MCInst &MI, unsigned OpNo,
                                SmallVectorImpl<MCFixup> &Fixups,
                                const MCSubtargetInfo &STI) const;
  unsigned getUImm4AndValue(const MCInst &MI, unsigned OpNo,
                            SmallVectorImpl<MCFixup> &Fixups,
                            const MCSubtargetInfo &STI) const;

  unsigned getMovePRegPairOpValue(const MCInst &MI, unsigned OpNo,
                                  SmallVectorImpl<MCFixup> &Fixups,
                                  const MCSubtargetInfo &STI) const;
  unsigned getMovePRegSingleOpValue(const MCInst &MI, unsigned OpNo,
                                    SmallVectorImpl<MCFixup> &Fixups,
                                    const MCSubtargetInfo &STI) const;

  unsigned getSimm23Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;

  unsigned getExprOpValue(const MCExpr *Expr, SmallVectorImpl<MCFixup> &Fixups,
                          const MCSubtargetInfo &STI) const;

  unsigned getRegisterListOpValue(const MCInst &MI, unsigned OpNo,
                                  SmallVectorImpl<MCFixup> &Fixups,
                                  const MCSubtargetInfo &STI) const;

  unsigned getRegisterListOpValue16(const MCInst &MI, unsigned OpNo,
                                    SmallVectorImpl<MCFixup> &Fixups,
                                    const MCSubtargetInfo &STI) const;

  // getBranch14TargetOpValue - Return binary encoding of the branch
  // target operand, such as BEQ, BNE. If the machine operand
  // requires relocation, record the relocation and return zero.
  unsigned getBranch14TargetOpValue(const MCInst &MI, unsigned OpNo,
                                    SmallVectorImpl<MCFixup> &Fixups,
                                    const MCSubtargetInfo &STI) const;

  /* 7 bit version of previous code */
  unsigned getBranch7TargetOpValue(const MCInst &MI, unsigned OpNo,
                                   SmallVectorImpl<MCFixup> &Fixups,
                                   const MCSubtargetInfo &STI) const;

  unsigned getMemEncoding(const MCInst &MI, unsigned OpNo,
                          SmallVectorImpl<MCFixup> &Fixups,
                          const MCSubtargetInfo &STI) const;

private:
  void LowerCompactBranch(MCInst& Inst) const;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUMCCODEEMITTER_H
