//===-- FgpuInstPrinter.cpp - Convert Fgpu MCInst to assembly syntax ------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This class prints an Fgpu MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#include "FgpuInstPrinter.h"
#include "FgpuInstrInfo.h"
#include "FgpuMCExpr.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "asm-printer"

#define PRINT_ALIAS_INSTR
#include "FgpuGenAsmWriter.inc"

template<unsigned R>
static bool isReg(const MCInst &MI, unsigned OpNo) {
  assert(MI.getOperand(OpNo).isReg() && "Register operand expected.");
  return MI.getOperand(OpNo).getReg() == R;
}

const char* Fgpu::FgpuFCCToString(Fgpu::CondCode CC) {
  switch (CC) {
//  case FCOND_F:
//  case FCOND_T:   return "f";
//  case FCOND_UN:
//  case FCOND_OR:  return "un";
//  case FCOND_OEQ:
//  case FCOND_UNE: return "eq";
//  case FCOND_UEQ:
//  case FCOND_ONE: return "ueq";
//  case FCOND_OLT:
//  case FCOND_UGE: return "olt";
//  case FCOND_ULT:
//  case FCOND_OGE: return "ult";
//  case FCOND_OLE:
//  case FCOND_UGT: return "ole";
//  case FCOND_ULE:
//  case FCOND_OGT: return "ule";
//  case FCOND_SF:
//  case FCOND_ST:  return "sf";
//  case FCOND_NGLE:
//  case FCOND_GLE: return "ngle";
//  case FCOND_SEQ:
//  case FCOND_SNE: return "seq";
//  case FCOND_NGL:
//  case FCOND_GL:  return "ngl";
//  case FCOND_LT:
//  case FCOND_NLT: return "lt";
//  case FCOND_NGE:
//  case FCOND_GE:  return "nge";
//  case FCOND_LE:
//  case FCOND_NLE: return "le";
//  case FCOND_NGT:
//  case FCOND_GT:  return "ngt";
  }
  llvm_unreachable("Impossible condition code!");
}

void FgpuInstPrinter::printRegName(raw_ostream &OS, unsigned RegNo) const {
  OS << '$' << StringRef(getRegisterName(RegNo)).lower();
}

void FgpuInstPrinter::printInst(const MCInst *MI, uint64_t Address,
                                StringRef Annot, const MCSubtargetInfo &STI,
                                raw_ostream &O) {
  switch (MI->getOpcode()) {
  default:
    break;
//  case Fgpu::RDHWR:
//  case Fgpu::RDHWR64:
//    O << "\t.set\tpush\n";
//    O << "\t.set\tfgpu32r2\n";
//    break;
  }

  // Try to print any aliases first.
  if (!printAliasInstr(MI, Address, O) && !printAlias(*MI, O))
    printInstruction(MI, Address, O);
  printAnnotation(O, Annot);

  switch (MI->getOpcode()) {
  default:
    break;
//  case Fgpu::RDHWR:
//  case Fgpu::RDHWR64:
//    O << "\n\t.set\tpop";
  }
}

void FgpuInstPrinter::printOperand(const MCInst *MI, unsigned OpNo,
                                   raw_ostream &O) {
  const MCOperand &Op = MI->getOperand(OpNo);
  if (Op.isReg()) {
    printRegName(O, Op.getReg());
    return;
  }

  if (Op.isImm()) {
    O << formatImm(Op.getImm());
    return;
  }

  assert(Op.isExpr() && "unknown operand kind in printOperand");
  Op.getExpr()->print(O, &MAI, true);
}

template <unsigned Bits, unsigned Offset>
void FgpuInstPrinter::printUImm(const MCInst *MI, int opNum, raw_ostream &O) {
  const MCOperand &MO = MI->getOperand(opNum);
  if (MO.isImm()) {
    uint64_t Imm = MO.getImm();
    Imm -= Offset;
    Imm &= (1 << Bits) - 1;
    Imm += Offset;
    O << formatImm(Imm);
    return;
  }

  printOperand(MI, opNum, O);
}

void FgpuInstPrinter::printUnsignedShortImm(const MCInst *MI, int opNum, raw_ostream &O) {
  // errs() << "printUnsignedShortImm entered: Imm= " << opNum << "\n";
  const MCOperand &MO = MI->getOperand(opNum);
  if (MO.isImm())
    O << (unsigned short int)MO.getImm();
  else
    printOperand(MI, opNum, O);
}
void FgpuInstPrinter::printUnsignedImm(const MCInst *MI, int opNum, raw_ostream &O) {
  // errs() << "printUnsignedImm entered: Imm= " << opNum << "\n";
  const MCOperand &MO = MI->getOperand(opNum);
  if (MO.isImm())
    O << (unsigned int)MO.getImm();
  else
    printOperand(MI, opNum, O);
}

void FgpuInstPrinter::
printMemOperand(const MCInst *MI, int opNum, raw_ostream &O) {
  // Load/Store memory operands -- imm($reg)
  // If PIC target the target is loaded as the
  // pattern lw $25,%call16($28)

  printOperand(MI, opNum+1, O);
  O << "(";
  printOperand(MI, opNum, O);
  O << ")";
}

void FgpuInstPrinter::
printMemOperandEA(const MCInst *MI, int opNum, raw_ostream &O) {
  // when using stack locations for not load/store instructions
  // print the same way as all normal 3 operand instructions.
  printOperand(MI, opNum, O);
  O << ", ";
  printOperand(MI, opNum+1, O);
}

void FgpuInstPrinter::
printFCCOperand(const MCInst *MI, int opNum, raw_ostream &O) {
  const MCOperand& MO = MI->getOperand(opNum);
  O << FgpuFCCToString((Fgpu::CondCode)MO.getImm());
}

void FgpuInstPrinter::
printSHFMask(const MCInst *MI, int opNum, raw_ostream &O) {
  llvm_unreachable("TODO");
}

bool FgpuInstPrinter::printAlias(const char *Str, const MCInst &MI,
                                 unsigned OpNo, raw_ostream &OS) {
  OS << "\t" << Str << "\t";
  printOperand(&MI, OpNo, OS);
  return true;
}

bool FgpuInstPrinter::printAlias(const char *Str, const MCInst &MI,
                                 unsigned OpNo0, unsigned OpNo1,
                                 raw_ostream &OS) {
  printAlias(Str, MI, OpNo0, OS);
  OS << ", ";
  printOperand(&MI, OpNo1, OS);
  return true;
}

bool FgpuInstPrinter::printAlias(const MCInst &MI, raw_ostream &OS) {
  switch (MI.getOpcode()) {
  case Fgpu::BEQ:
    // beq $zero, $zero, $L2 => b $L2
    // beq $r0, $zero, $L2 => beqz $r0, $L2
    return (isReg<Fgpu::ZERO>(MI, 0) && isReg<Fgpu::ZERO>(MI, 1) &&
            printAlias("b", MI, 2, OS)) ||
           (isReg<Fgpu::ZERO>(MI, 1) && printAlias("beqz", MI, 0, 2, OS));
  case Fgpu::BNE:
    // bne $r0, $zero, $L2 => bnez $r0, $L2
    return isReg<Fgpu::ZERO>(MI, 1) && printAlias("bnez", MI, 0, 2, OS);
//  case Fgpu::JALR:
//    // jalr $ra, $r1 => jalr $r1
//    return isReg<Fgpu::RA>(MI, 0) && printAlias("jalr", MI, 1, OS);
  case Fgpu::NOR:
    // nor $r0, $r1, $zero => not $r0, $r1
    return isReg<Fgpu::ZERO>(MI, 2) && printAlias("not", MI, 0, 1, OS);
  case Fgpu::OR:
    // or $r0, $r1, $zero => move $r0, $r1
    return isReg<Fgpu::ZERO>(MI, 2) && printAlias("move", MI, 0, 1, OS);
  default: return false;
  }
}

void FgpuInstPrinter::printSaveRestore(const MCInst *MI, raw_ostream &O) {
  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    if (i != 0) O << ", ";
    if (MI->getOperand(i).isReg())
      printRegName(O, MI->getOperand(i).getReg());
    else
      printUImm<16>(MI, i, O);
  }
}

void FgpuInstPrinter::
printRegisterList(const MCInst *MI, int opNum, raw_ostream &O) {
  // - 2 because register List is always first operand of instruction and it is
  // always followed by memory operand (base + offset).
  for (int i = opNum, e = MI->getNumOperands() - 2; i != e; ++i) {
    if (i != opNum)
      O << ", ";
    printRegName(O, MI->getOperand(i).getReg());
  }
}
