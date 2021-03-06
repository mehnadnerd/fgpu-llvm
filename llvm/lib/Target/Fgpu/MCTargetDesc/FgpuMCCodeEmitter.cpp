//===-- FgpuMCCodeEmitter.cpp - Convert Fgpu Code to Machine Code ---------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the FgpuMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "FgpuMCCodeEmitter.h"
#include "MCTargetDesc/FgpuFixupKinds.h"
#include "MCTargetDesc/FgpuMCExpr.h"
#include "MCTargetDesc/FgpuMCTargetDesc.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <cassert>
#include <cstdint>

using namespace llvm;

#define DEBUG_TYPE "mccodeemitter"

#define GET_INSTRMAP_INFO
#include "FgpuGenInstrInfo.inc"
#undef GET_INSTRMAP_INFO

namespace llvm {

MCCodeEmitter *createFgpuMCCodeEmitterEB(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx) {
  return new FgpuMCCodeEmitter(MCII, Ctx, false);
}

MCCodeEmitter *createFgpuMCCodeEmitterEL(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx) {
  return new FgpuMCCodeEmitter(MCII, Ctx, true);
}

} // end namespace llvm

// If the D<shift> instruction has a shift amount that is greater
// than 31 (checked in calling routine), lower it to a D<shift>32 instruction
static void LowerLargeShift(MCInst& Inst) {
  assert(Inst.getNumOperands() == 3 && "Invalid no. of operands for shift!");
  assert(Inst.getOperand(2).isImm());

  int64_t Shift = Inst.getOperand(2).getImm();
  if (Shift <= 31)
    return; // Do nothing
  Shift -= 32;

  // saminus32
  Inst.getOperand(2).setImm(Shift);

  switch (Inst.getOpcode()) {
  default:
    // Calling function is not synchronized
    llvm_unreachable("Unexpected shift instruction");
//  case Fgpu::DSLL:
//    Inst.setOpcode(Fgpu::DSLL32);
//    return;
//  case Fgpu::DSRL:
//    Inst.setOpcode(Fgpu::DSRL32);
//    return;
//  case Fgpu::DSRA:
//    Inst.setOpcode(Fgpu::DSRA32);
//    return;
//  case Fgpu::DROTR:
//    Inst.setOpcode(Fgpu::DROTR32);
//    return;
  }
}

// Fix a bad compact branch encoding for beqc/bnec.
void FgpuMCCodeEmitter::LowerCompactBranch(MCInst& Inst) const {
  // Encoding may be illegal !(rs < rt), but this situation is
  // easily fixed.
  unsigned RegOp0 = Inst.getOperand(0).getReg();
  unsigned RegOp1 = Inst.getOperand(1).getReg();

  unsigned Reg0 =  Ctx.getRegisterInfo()->getEncodingValue(RegOp0);
  unsigned Reg1 =  Ctx.getRegisterInfo()->getEncodingValue(RegOp1);

//  if (Inst.getOpcode() == Fgpu::BNEC || Inst.getOpcode() == Fgpu::BEQC ||
//      Inst.getOpcode() == Fgpu::BNEC64 || Inst.getOpcode() == Fgpu::BEQC64) {
//    assert(Reg0 != Reg1 && "Instruction has bad operands ($rs == $rt)!");
//    if (Reg0 < Reg1)
//      return;
//  } else if (Inst.getOpcode() == Fgpu::BNVC || Inst.getOpcode() == Fgpu::BOVC) {
//    if (Reg0 >= Reg1)
//      return;
//  } else
//    llvm_unreachable("Cannot rewrite unknown branch!");
//
//  Inst.getOperand(0).setReg(RegOp1);
//  Inst.getOperand(1).setReg(RegOp0);
}

void FgpuMCCodeEmitter::EmitByte(unsigned char C, raw_ostream &OS) const {
  OS << (char)C;
}

void FgpuMCCodeEmitter::emitInstruction(uint64_t Val, unsigned Size,
                                        const MCSubtargetInfo &STI,
                                        raw_ostream &OS) const {
  // Output the instruction encoding in little endian byte order.
  // Little-endian byte ordering:
  //   fgpu32r2:   4 | 3 | 2 | 1
  //   microFGPU:  2 | 1 | 4 | 3
  {
    for (unsigned i = 0; i < Size; ++i) {
      unsigned Shift = IsLittleEndian ? i * 8 : (Size - 1 - i) * 8;
      EmitByte((Val >> Shift) & 0xff, OS);
    }
  }
}

/// encodeInstruction - Emit the instruction.
/// Size the instruction with Desc.getSize().
void FgpuMCCodeEmitter::
encodeInstruction(const MCInst &MI, raw_ostream &OS,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const
{
  // Non-pseudo instructions that get changed for direct object
  // only based on operand values.
  // If this list of instructions get much longer we will move
  // the check to a function call. Until then, this is more efficient.
  MCInst TmpInst = MI;
  switch (MI.getOpcode()) {
  // If shift amount is >= 32 it the inst needs to be lowered further
//  case Fgpu::DSLL:
//  case Fgpu::DSRL:
//  case Fgpu::DSRA:
//  case Fgpu::DROTR:
//    LowerLargeShift(TmpInst);
//    break;
//  // Compact branches, enforce encoding restrictions.
//  case Fgpu::BEQC:
//  case Fgpu::BNEC:
//  case Fgpu::BEQC64:
//  case Fgpu::BNEC64:
//  case Fgpu::BOVC:
//  case Fgpu::BNVC:
//    LowerCompactBranch(TmpInst);
  }

  unsigned long N = Fixups.size();
  uint32_t Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);

  // Check for unimplemented opcodes.
  // Unfortunately in FGPU both NOP and SLL will come in with Binary == 0
  // so we have to special check for them.
  const unsigned Opcode = TmpInst.getOpcode();
  if ((Opcode != Fgpu::NOP) && (Opcode != Fgpu::SLL) &&
      !Binary)
    llvm_unreachable("unimplemented opcode in encodeInstruction()");

  int NewOpcode = -1;

  const MCInstrDesc &Desc = MCII.get(TmpInst.getOpcode());

  // Get byte count of instruction
  unsigned Size = Desc.getSize();
  if (!Size)
    llvm_unreachable("Desc.getSize() returns 0");

  emitInstruction(Binary, Size, STI, OS);
}

/// getBranchTargetOpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned FgpuMCCodeEmitter::
getBranchTargetOpValue(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTargetOpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Fgpu::fixup_Fgpu_PC16)));
  return 0;
}

/// getBranchTargetOpValue1SImm16 - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned FgpuMCCodeEmitter::
getBranchTargetOpValue1SImm16(const MCInst &MI, unsigned OpNo,
                              SmallVectorImpl<MCFixup> &Fixups,
                              const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Fgpu::fixup_Fgpu_PC16)));
  return 0;
}

/// getBranchTargetOpValueMMR6 - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned FgpuMCCodeEmitter::
getBranchTargetOpValueMMR6(const MCInst &MI, unsigned OpNo,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm())
    return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValueMMR6 expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-2, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Fgpu::fixup_Fgpu_PC16)));
  return 0;
}

/// getBranchTarget21OpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned FgpuMCCodeEmitter::
getBranchTarget21OpValue(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTarget21OpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Fgpu::fixup_FGPU_PC21_S2)));
  return 0;
}

/// getBranchTarget21OpValueMM - Return binary encoding of the branch
/// target operand for microFGPU. If the machine operand requires
/// relocation, record the relocation and return zero.
unsigned FgpuMCCodeEmitter::
getBranchTarget21OpValueMM(const MCInst &MI, unsigned OpNo,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
    "getBranchTarget21OpValueMM expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Fgpu::fixup_MICROFGPU_PC21_S1)));
  return 0;
}

/// getBranchTarget26OpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned FgpuMCCodeEmitter::
getBranchTarget26OpValue(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTarget26OpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Fgpu::fixup_FGPU_PC26_S2)));
  return 0;
}

/// getBranchTarget26OpValueMM - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned FgpuMCCodeEmitter::getBranchTarget26OpValueMM(
    const MCInst &MI, unsigned OpNo, SmallVectorImpl<MCFixup> &Fixups,
    const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm())
    return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTarget26OpValueMM expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Fgpu::fixup_MICROFGPU_PC26_S1)));
  return 0;
}

/// getJumpOffset16OpValue - Return binary encoding of the jump
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned FgpuMCCodeEmitter::
getJumpOffset16OpValue(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  if (MO.isImm()) return MO.getImm();

  assert(MO.isExpr() &&
         "getJumpOffset16OpValue expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  Fgpu::Fixups FixupKind = Fgpu::fixup_Fgpu_LO16;
  Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind(FixupKind)));
  return 0;
}

unsigned FgpuMCCodeEmitter::
getUImm5Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    // The immediate is encoded as 'immediate << 2'.
    unsigned Res = getMachineOpValue(MI, MO, Fixups, STI);
    assert((Res & 3) == 0);
    return Res >> 2;
  }

  assert(MO.isExpr() &&
         "getUImm5Lsl2Encoding expects only expressions or an immediate");

  return 0;
}

unsigned FgpuMCCodeEmitter::
getSImm3Lsa2Value(const MCInst &MI, unsigned OpNo,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    int Value = MO.getImm();
    return Value >> 2;
  }

  return 0;
}

unsigned FgpuMCCodeEmitter::
getUImm6Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    unsigned Value = MO.getImm();
    return Value >> 2;
  }

  return 0;
}

unsigned FgpuMCCodeEmitter::
getSImm9AddiuspValue(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    unsigned Binary = (MO.getImm() >> 2) & 0x0000ffff;
    return (((Binary & 0x8000) >> 7) | (Binary & 0x00ff));
  }

  return 0;
}

unsigned FgpuMCCodeEmitter::
getExprOpValue(const MCExpr *Expr, SmallVectorImpl<MCFixup> &Fixups,
               const MCSubtargetInfo &STI) const {
  int64_t Res;

  if (Expr->evaluateAsAbsolute(Res))
    return Res;

  MCExpr::ExprKind Kind = Expr->getKind();
  if (Kind == MCExpr::Constant) {
    return cast<MCConstantExpr>(Expr)->getValue();
  }

  if (Kind == MCExpr::Binary) {
    unsigned Res =
        getExprOpValue(cast<MCBinaryExpr>(Expr)->getLHS(), Fixups, STI);
    Res += getExprOpValue(cast<MCBinaryExpr>(Expr)->getRHS(), Fixups, STI);
    return Res;
  }

  if (Kind == MCExpr::Target) {
    const FgpuMCExpr *FgpuExpr = cast<FgpuMCExpr>(Expr);

    Fgpu::Fixups FixupKind = Fgpu::Fixups(0);
    switch (FgpuExpr->getKind()) {
    case FgpuMCExpr::MEK_None:
    case FgpuMCExpr::MEK_Special:
      llvm_unreachable("Unhandled fixup kind!");
      break;
    case FgpuMCExpr::MEK_DTPREL:
      // MEK_DTPREL is used for marking TLS DIEExpr only
      // and contains a regular sub-expression.
      return getExprOpValue(FgpuExpr->getSubExpr(), Fixups, STI);
    case FgpuMCExpr::MEK_CALL_HI16:
      FixupKind = Fgpu::fixup_Fgpu_CALL_HI16;
      break;
    case FgpuMCExpr::MEK_CALL_LO16:
      FixupKind = Fgpu::fixup_Fgpu_CALL_LO16;
      break;
    case FgpuMCExpr::MEK_DTPREL_HI:
      FixupKind =
                                    Fgpu::fixup_Fgpu_DTPREL_HI;
      break;
    case FgpuMCExpr::MEK_DTPREL_LO:
      FixupKind = Fgpu::fixup_Fgpu_DTPREL_LO;
      break;
    case FgpuMCExpr::MEK_GOTTPREL:
      FixupKind = Fgpu::fixup_Fgpu_GOTTPREL;
      break;
    case FgpuMCExpr::MEK_GOT:
      FixupKind = Fgpu::fixup_Fgpu_GOT;
      break;
    case FgpuMCExpr::MEK_GOT_CALL:
      FixupKind = Fgpu::fixup_Fgpu_CALL16;
      break;
    case FgpuMCExpr::MEK_GOT_DISP:
      FixupKind = Fgpu::fixup_Fgpu_GOT_DISP;
      break;
    case FgpuMCExpr::MEK_GOT_HI16:
      FixupKind = Fgpu::fixup_Fgpu_GOT_HI16;
      break;
    case FgpuMCExpr::MEK_GOT_LO16:
      FixupKind = Fgpu::fixup_Fgpu_GOT_LO16;
      break;
    case FgpuMCExpr::MEK_GOT_PAGE:
      FixupKind = Fgpu::fixup_Fgpu_GOT_PAGE;
      break;
    case FgpuMCExpr::MEK_GOT_OFST:
      FixupKind = Fgpu::fixup_Fgpu_GOT_OFST;
      break;
    case FgpuMCExpr::MEK_GPREL:
      FixupKind = Fgpu::fixup_Fgpu_GPREL16;
      break;
    case FgpuMCExpr::MEK_LO:
      // Check for %lo(%neg(%gp_rel(X)))
      if (FgpuExpr->isGpOff())
        FixupKind = Fgpu::fixup_Fgpu_GPOFF_LO;
      else
        FixupKind = Fgpu::fixup_Fgpu_LO16;
      break;
    case FgpuMCExpr::MEK_HIGHEST:
      FixupKind = Fgpu::fixup_Fgpu_HIGHEST;
      break;
    case FgpuMCExpr::MEK_HIGHER:
      FixupKind = Fgpu::fixup_Fgpu_HIGHER;
      break;
    case FgpuMCExpr::MEK_HI:
      // Check for %hi(%neg(%gp_rel(X)))
      if (FgpuExpr->isGpOff())
        FixupKind = Fgpu::fixup_Fgpu_GPOFF_HI;
      else
        FixupKind = Fgpu::fixup_Fgpu_HI16;
      break;
    case FgpuMCExpr::MEK_PCREL_HI16:
      FixupKind = Fgpu::fixup_FGPU_PCHI16;
      break;
    case FgpuMCExpr::MEK_PCREL_LO16:
      FixupKind = Fgpu::fixup_FGPU_PCLO16;
      break;
    case FgpuMCExpr::MEK_TLSGD:
      FixupKind = Fgpu::fixup_Fgpu_TLSGD;
      break;
    case FgpuMCExpr::MEK_TLSLDM:
      FixupKind = Fgpu::fixup_Fgpu_TLSLDM;
      break;
    case FgpuMCExpr::MEK_TPREL_HI:
      FixupKind = Fgpu::fixup_Fgpu_TPREL_HI;
      break;
    case FgpuMCExpr::MEK_TPREL_LO:
      FixupKind = Fgpu::fixup_Fgpu_TPREL_LO;
      break;
    case FgpuMCExpr::MEK_NEG:
      FixupKind = Fgpu::fixup_Fgpu_SUB;
      break;
    }
    Fixups.push_back(MCFixup::create(0, FgpuExpr, MCFixupKind(FixupKind)));
    return 0;
  }

  if (Kind == MCExpr::SymbolRef)
    Ctx.reportError(Expr->getLoc(), "expected an immediate");
  return 0;
}

//
///// Return binary encoding of memory related operand.
///// If the offset operand requires relocation, record the relocation.
//template <unsigned ShiftAmount>
//unsigned FgpuMCCodeEmitter::getMemEncoding(const MCInst &MI, unsigned OpNo,
//                                           SmallVectorImpl<MCFixup> &Fixups,
//                                           const MCSubtargetInfo &STI) const {
//  // Base register is encoded in bits 20-16, offset is encoded in bits 15-0.
//  assert(MI.getOperand(OpNo).isReg());
//  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI)
//                     << 16;
//  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);
//
//  // Apply the scale factor if there is one.
//  OffBits >>= ShiftAmount;
//
//  return (OffBits & 0xFFFF) | RegBits;
//}

// FIXME: should be called getMSBEncoding
//
unsigned
FgpuMCCodeEmitter::getSizeInsEncoding(const MCInst &MI, unsigned OpNo,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo-1).isImm());
  assert(MI.getOperand(OpNo).isImm());
  unsigned Position = getMachineOpValue(MI, MI.getOperand(OpNo-1), Fixups, STI);
  unsigned Size = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);

  return Position + Size - 1;
}

template <unsigned Bits, int Offset>
unsigned
FgpuMCCodeEmitter::getUImmWithOffsetEncoding(const MCInst &MI, unsigned OpNo,
                                             SmallVectorImpl<MCFixup> &Fixups,
                                             const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo).isImm());
  unsigned Value = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);
  Value -= Offset;
  return Value;
}

unsigned
FgpuMCCodeEmitter::getSimm19Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    // The immediate is encoded as 'immediate << 2'.
    unsigned Res = getMachineOpValue(MI, MO, Fixups, STI);
    assert((Res & 3) == 0);
    return Res >> 2;
  }

  assert(MO.isExpr() &&
         "getSimm19Lsl2Encoding expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  Fgpu::Fixups FixupKind = Fgpu::fixup_FGPU_PC19_S2;
  Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind(FixupKind)));
  return 0;
}

unsigned
FgpuMCCodeEmitter::getSimm18Lsl3Encoding(const MCInst &MI, unsigned OpNo,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    // The immediate is encoded as 'immediate << 3'.
    unsigned Res = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);
    assert((Res & 7) == 0);
    return Res >> 3;
  }

  assert(MO.isExpr() &&
         "getSimm18Lsl2Encoding expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  Fgpu::Fixups FixupKind = Fgpu::fixup_FGPU_PC18_S3;
  Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind(FixupKind)));
  return 0;
}

unsigned
FgpuMCCodeEmitter::getUImm3Mod8Encoding(const MCInst &MI, unsigned OpNo,
                                        SmallVectorImpl<MCFixup> &Fixups,
                                        const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo).isImm());
  const MCOperand &MO = MI.getOperand(OpNo);
  return MO.getImm() % 8;
}

unsigned
FgpuMCCodeEmitter::getUImm4AndValue(const MCInst &MI, unsigned OpNo,
                                    SmallVectorImpl<MCFixup> &Fixups,
                                    const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo).isImm());
  const MCOperand &MO = MI.getOperand(OpNo);
  unsigned Value = MO.getImm();
  switch (Value) {
    case 128:   return 0x0;
    case 1:     return 0x1;
    case 2:     return 0x2;
    case 3:     return 0x3;
    case 4:     return 0x4;
    case 7:     return 0x5;
    case 8:     return 0x6;
    case 15:    return 0x7;
    case 16:    return 0x8;
    case 31:    return 0x9;
    case 32:    return 0xa;
    case 63:    return 0xb;
    case 64:    return 0xc;
    case 255:   return 0xd;
    case 32768: return 0xe;
    case 65535: return 0xf;
  }
  llvm_unreachable("Unexpected value");
}

unsigned
FgpuMCCodeEmitter::getRegisterListOpValue(const MCInst &MI, unsigned OpNo,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
  unsigned res = 0;

  // Register list operand is always first operand of instruction and it is
  // placed before memory operand (register + imm).

  for (unsigned I = OpNo, E = MI.getNumOperands() - 2; I < E; ++I) {
    unsigned Reg = MI.getOperand(I).getReg();
    unsigned RegNo = Ctx.getRegisterInfo()->getEncodingValue(Reg);
    if (RegNo != 31)
      res++;
    else
      res |= 0x10;
  }
  return res;
}

unsigned
FgpuMCCodeEmitter::getRegisterListOpValue16(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  return (MI.getNumOperands() - 4);
}

unsigned
FgpuMCCodeEmitter::getMovePRegPairOpValue(const MCInst &MI, unsigned OpNo,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
 assert(false && "naT in this");
  return -1;
//  unsigned res = 0;
//
//  if (MI.getOperand(0).getReg() == Fgpu::A1 &&
//      MI.getOperand(1).getReg() == Fgpu::A2)
//    res = 0;
//  else if (MI.getOperand(0).getReg() == Fgpu::A1 &&
//           MI.getOperand(1).getReg() == Fgpu::A3)
//    res = 1;
//  else if (MI.getOperand(0).getReg() == Fgpu::A2 &&
//           MI.getOperand(1).getReg() == Fgpu::A3)
//    res = 2;
//  else if (MI.getOperand(0).getReg() == Fgpu::A0 &&
//           MI.getOperand(1).getReg() == Fgpu::S5)
//    res = 3;
//  else if (MI.getOperand(0).getReg() == Fgpu::A0 &&
//           MI.getOperand(1).getReg() == Fgpu::S6)
//    res = 4;
//  else if (MI.getOperand(0).getReg() == Fgpu::A0 &&
//           MI.getOperand(1).getReg() == Fgpu::A1)
//    res = 5;
//  else if (MI.getOperand(0).getReg() == Fgpu::A0 &&
//           MI.getOperand(1).getReg() == Fgpu::A2)
//    res = 6;
//  else if (MI.getOperand(0).getReg() == Fgpu::A0 &&
//           MI.getOperand(1).getReg() == Fgpu::A3)
//    res = 7;
//
//  return res;
}

unsigned
FgpuMCCodeEmitter::getMovePRegSingleOpValue(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  assert(false && "What is this");
//  assert(((OpNo == 2) || (OpNo == 3)) &&
//         "Unexpected OpNo for movep operand encoding!");
//
//  MCOperand Op = MI.getOperand(OpNo);
//  assert(Op.isReg() && "Operand of movep is not a register!");
//  switch (Op.getReg()) {
//  default:
//    llvm_unreachable("Unknown register for movep!");
//  case Fgpu::ZERO:  return 0;
//  case Fgpu::S1:    return 1;
//  case Fgpu::V0:    return 2;
//  case Fgpu::V1:    return 3;
//  case Fgpu::S0:    return 4;
//  case Fgpu::S2:    return 5;
//  case Fgpu::S3:    return 6;
//  case Fgpu::S4:    return 7;
//  }
}

unsigned
FgpuMCCodeEmitter::getSimm23Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  assert(MO.isImm() && "getSimm23Lsl2Encoding expects only an immediate");
  // The immediate is encoded as 'immediate >> 2'.
  unsigned Res = static_cast<unsigned>(MO.getImm());
  assert((Res & 3) == 0);
  return Res >> 2;
}



/// getBranch14TargetOpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned FgpuMCCodeEmitter::getBranch14TargetOpValue(const MCInst &MI, unsigned OpNo, SmallVectorImpl<MCFixup> &Fixups, const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm())
    LLVM_DEBUG(dbgs() << "getBranch14TargetOpValue is immediate = " << MO.getImm() << "\n");

  // If the destination is an immediate, we have nothing to do.
  if (MO.isImm()) return MO.getImm();
  assert(MO.isExpr() && "getBranch14TargetOpValue expects only expressions");

  const MCExpr *Expr = MO.getExpr();
  LLVM_DEBUG(dbgs() << "getBranch14TargetOpValue  = " << *Expr << "\n");
  Fixups.push_back(MCFixup::create(0, Expr,
                                   MCFixupKind(Fgpu::fixup_Fgpu_PC14)));
  return 0;
}

/// getBranch14TargetOpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned FgpuMCCodeEmitter::getBranch7TargetOpValue(const MCInst &MI, unsigned OpNo, SmallVectorImpl<MCFixup> &Fixups, const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm())
    LLVM_DEBUG(dbgs() << "getBranch7TargetOpValue is immediate = " << MO.getImm() << "\n");

  // If the destination is an immediate, we have nothing to do.
  if (MO.isImm()) return MO.getImm();
  assert(MO.isExpr() && "getBranch7TargetOpValue expects only expressions");

  const MCExpr *Expr = MO.getExpr();
  LLVM_DEBUG(dbgs() << "getBranch7TargetOpValue  = " << *Expr << "\n");
  Fixups.push_back(MCFixup::create(0, Expr,
                                   MCFixupKind(Fgpu::fixup_Fgpu_PC7)));
  return 0;
}


/// getJumpTargetOpValue - Return binary encoding of the jump
/// target operand, such as JSUB.
/// If the machine operand requires relocation,
/// record the relocation and return zero.
//@getJumpTargetOpValue {
unsigned FgpuMCCodeEmitter::
    getJumpTargetOpValue(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  LLVM_DEBUG(dbgs() << "soubhi: getJumpTargetOpValue entered!\n");
  unsigned Opcode = MI.getOpcode();
  const MCOperand &MO = MI.getOperand(OpNo);
  // dbgs() << "soubhi: MO.getImm()
  // If the destination is an immediate, we have nothing to do.
  if (MO.isImm()) return MO.getImm();
  assert(MO.isExpr() && "getJumpTargetOpValue expects only expressions");

  // const MCExpr *Expr = MO.getExpr();
  assert(Opcode == Fgpu::JSUB or Opcode == Fgpu::SET_SYNC);
  const MCExpr *Expr = MO.getExpr();
  if (Opcode == Fgpu::JSUB or Opcode == Fgpu::SET_SYNC)
    Fixups.push_back(MCFixup::create(0, Expr,
                                     MCFixupKind(Fgpu::fixup_Fgpu_JSUB)));
  else
    llvm_unreachable("unexpect opcode in getJumpAbsoluteTargetOpValue()");
  return 0;
}
//@CH8_1 }

/// getMachineOpValue - Return binary encoding of operand. If the machine
/// operand requires relocation, record the relocation and return zero.
unsigned FgpuMCCodeEmitter::
    getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                      SmallVectorImpl<MCFixup> &Fixups,
                      const MCSubtargetInfo &STI) const {
  if (MO.isReg()) {
    unsigned Reg = MO.getReg();
    //unsigned RegNo = getFgpuRegisterNumbering(Reg);
    unsigned RegNo = Ctx.getRegisterInfo()->getEncodingValue(Reg);
    return RegNo;
  } else if (MO.isImm()) {
    return static_cast<unsigned>(MO.getImm());
  } else if (MO.isSFPImm()) {
    return static_cast<unsigned>(MO.getSFPImm()); // TODO: 90% sure this is wrong
  }
  // MO must be an Expr.
  assert(MO.isExpr());
  return getExprOpValue(MO.getExpr(),Fixups, STI);
}

/// getMemEncoding - Return binary encoding of memory related operand.
/// If the offset operand requires relocation, record the relocation.
unsigned
FgpuMCCodeEmitter::getMemEncoding(const MCInst &MI, unsigned OpNo,
                                  SmallVectorImpl<MCFixup> &Fixups,
                                  const MCSubtargetInfo &STI) const {
  // assert(MI.getOperand(OpNo).isReg());
  // Base register is encoded in bits 9-5, offset is encoded in bits 23-10.
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI)/4;
  LLVM_DEBUG(dbgs() << "soubhi : RegBits = " << RegBits << "\n");
  LLVM_DEBUG(dbgs() << "soubhi : OffBits = " << OffBits << "\n");
  LLVM_DEBUG(dbgs() << "soubhi : res = " << (((OffBits & 0x3FFF)<<5) | (RegBits & 0x1F)) << "\n");

  return ((OffBits & 0x3FFF)<<5) | (RegBits & 0x1F);
}
#include "FgpuGenMCCodeEmitter.inc"
