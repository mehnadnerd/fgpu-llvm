//===- FgpuMCInstLower.cpp - Convert Fgpu MachineInstr to MCInst ----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower Fgpu MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "FgpuMCInstLower.h"
#include "MCTargetDesc/FgpuBaseInfo.h"
#include "MCTargetDesc/FgpuMCExpr.h"
#include "FgpuAsmPrinter.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/ErrorHandling.h"
#include <cassert>

using namespace llvm;

FgpuMCInstLower::FgpuMCInstLower(FgpuAsmPrinter &asmprinter)
  : AsmPrinter(asmprinter) {}

void FgpuMCInstLower::Initialize(MCContext *C) {
  Ctx = C;
}

MCOperand FgpuMCInstLower::LowerSymbolOperand(const MachineOperand &MO,
                                              MachineOperandType MOTy,
                                              int64_t Offset) const {
  MCSymbolRefExpr::VariantKind Kind = MCSymbolRefExpr::VK_None;
  FgpuMCExpr::FgpuExprKind TargetKind = FgpuMCExpr::MEK_None;
  bool IsGpOff = false;
  const MCSymbol *Symbol;

  switch(MO.getTargetFlags()) {
  default:
    llvm_unreachable("Invalid target flag!");
  case FgpuII::MO_NO_FLAG:
    break;
  case FgpuII::MO_GPREL:
    TargetKind = FgpuMCExpr::MEK_GPREL;
    break;
  case FgpuII::MO_GOT_CALL:
    TargetKind = FgpuMCExpr::MEK_GOT_CALL;
    break;
  case FgpuII::MO_GOT:
    TargetKind = FgpuMCExpr::MEK_GOT;
    break;
  case FgpuII::MO_ABS_HI:
    TargetKind = FgpuMCExpr::MEK_HI;
    break;
  case FgpuII::MO_ABS_LO:
    TargetKind = FgpuMCExpr::MEK_LO;
    break;
  case FgpuII::MO_TLSGD:
    TargetKind = FgpuMCExpr::MEK_TLSGD;
    break;
  case FgpuII::MO_TLSLDM:
    TargetKind = FgpuMCExpr::MEK_TLSLDM;
    break;
  case FgpuII::MO_DTPREL_HI:
    TargetKind = FgpuMCExpr::MEK_DTPREL_HI;
    break;
  case FgpuII::MO_DTPREL_LO:
    TargetKind = FgpuMCExpr::MEK_DTPREL_LO;
    break;
  case FgpuII::MO_GOTTPREL:
    TargetKind = FgpuMCExpr::MEK_GOTTPREL;
    break;
  case FgpuII::MO_TPREL_HI:
    TargetKind = FgpuMCExpr::MEK_TPREL_HI;
    break;
  case FgpuII::MO_TPREL_LO:
    TargetKind = FgpuMCExpr::MEK_TPREL_LO;
    break;
  case FgpuII::MO_GPOFF_HI:
    TargetKind = FgpuMCExpr::MEK_HI;
    IsGpOff = true;
    break;
  case FgpuII::MO_GPOFF_LO:
    TargetKind = FgpuMCExpr::MEK_LO;
    IsGpOff = true;
    break;
  case FgpuII::MO_GOT_DISP:
    TargetKind = FgpuMCExpr::MEK_GOT_DISP;
    break;
  case FgpuII::MO_GOT_HI16:
    TargetKind = FgpuMCExpr::MEK_GOT_HI16;
    break;
  case FgpuII::MO_GOT_LO16:
    TargetKind = FgpuMCExpr::MEK_GOT_LO16;
    break;
  case FgpuII::MO_GOT_PAGE:
    TargetKind = FgpuMCExpr::MEK_GOT_PAGE;
    break;
  case FgpuII::MO_GOT_OFST:
    TargetKind = FgpuMCExpr::MEK_GOT_OFST;
    break;
  case FgpuII::MO_HIGHER:
    TargetKind = FgpuMCExpr::MEK_HIGHER;
    break;
  case FgpuII::MO_HIGHEST:
    TargetKind = FgpuMCExpr::MEK_HIGHEST;
    break;
  case FgpuII::MO_CALL_HI16:
    TargetKind = FgpuMCExpr::MEK_CALL_HI16;
    break;
  case FgpuII::MO_CALL_LO16:
    TargetKind = FgpuMCExpr::MEK_CALL_LO16;
    break;
  case FgpuII::MO_JALR:
    return MCOperand();
  }

  switch (MOTy) {
  case MachineOperand::MO_MachineBasicBlock:
    Symbol = MO.getMBB()->getSymbol();
    break;

  case MachineOperand::MO_GlobalAddress:
    Symbol = AsmPrinter.getSymbol(MO.getGlobal());
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_BlockAddress:
    Symbol = AsmPrinter.GetBlockAddressSymbol(MO.getBlockAddress());
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_ExternalSymbol:
    Symbol = AsmPrinter.GetExternalSymbolSymbol(MO.getSymbolName());
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_MCSymbol:
    Symbol = MO.getMCSymbol();
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_JumpTableIndex:
    Symbol = AsmPrinter.GetJTISymbol(MO.getIndex());
    break;

  case MachineOperand::MO_ConstantPoolIndex:
    Symbol = AsmPrinter.GetCPISymbol(MO.getIndex());
    Offset += MO.getOffset();
    break;

  default:
    llvm_unreachable("<unknown operand type>");
  }

  const MCExpr *Expr = MCSymbolRefExpr::create(Symbol, Kind, *Ctx);

  if (Offset) {
    // Note: Offset can also be negative
    Expr = MCBinaryExpr::createAdd(Expr, MCConstantExpr::create(Offset, *Ctx),
                                   *Ctx);
  }

  if (IsGpOff)
    Expr = FgpuMCExpr::createGpOff(TargetKind, Expr, *Ctx);
  else if (TargetKind != FgpuMCExpr::MEK_None)
    Expr = FgpuMCExpr::create(TargetKind, Expr, *Ctx);

  return MCOperand::createExpr(Expr);
}

MCOperand FgpuMCInstLower::LowerOperand(const MachineOperand &MO,
                                        int64_t offset) const {
  MachineOperandType MOTy = MO.getType();

  switch (MOTy) {
  default: llvm_unreachable("unknown operand type");
  case MachineOperand::MO_Register:
    // Ignore all implicit register operands.
    if (MO.isImplicit()) break;
    return MCOperand::createReg(MO.getReg());
  case MachineOperand::MO_Immediate:
    return MCOperand::createImm(MO.getImm() + offset);
  case MachineOperand::MO_MachineBasicBlock:
  case MachineOperand::MO_GlobalAddress:
  case MachineOperand::MO_ExternalSymbol:
  case MachineOperand::MO_MCSymbol:
  case MachineOperand::MO_JumpTableIndex:
  case MachineOperand::MO_ConstantPoolIndex:
  case MachineOperand::MO_BlockAddress:
    return LowerSymbolOperand(MO, MOTy, offset);
  case MachineOperand::MO_RegisterMask:
    break;
 }

  return MCOperand();
}

MCOperand FgpuMCInstLower::createSub(MachineBasicBlock *BB1,
                                     MachineBasicBlock *BB2,
                                     FgpuMCExpr::FgpuExprKind Kind) const {
  const MCSymbolRefExpr *Sym1 = MCSymbolRefExpr::create(BB1->getSymbol(), *Ctx);
  const MCSymbolRefExpr *Sym2 = MCSymbolRefExpr::create(BB2->getSymbol(), *Ctx);
  const MCBinaryExpr *Sub = MCBinaryExpr::createSub(Sym1, Sym2, *Ctx);

  return MCOperand::createExpr(FgpuMCExpr::create(Kind, Sub, *Ctx));
}

void FgpuMCInstLower::
lowerLongBranchLUi(const MachineInstr *MI, MCInst &OutMI) const {
  OutMI.setOpcode(Fgpu::LUi);

  // Lower register operand.
  OutMI.addOperand(LowerOperand(MI->getOperand(0)));

  FgpuMCExpr::FgpuExprKind Kind;
  unsigned TargetFlags = MI->getOperand(1).getTargetFlags();
  switch (TargetFlags) {
  case FgpuII::MO_HIGHEST:
    Kind = FgpuMCExpr::MEK_HIGHEST;
    break;
  case FgpuII::MO_HIGHER:
    Kind = FgpuMCExpr::MEK_HIGHER;
    break;
  case FgpuII::MO_ABS_HI:
    Kind = FgpuMCExpr::MEK_HI;
    break;
  case FgpuII::MO_ABS_LO:
    Kind = FgpuMCExpr::MEK_LO;
    break;
  default:
    report_fatal_error("Unexpected flags for lowerLongBranchLUi");
  }

  if (MI->getNumOperands() == 2) {
    const MCExpr *Expr =
        MCSymbolRefExpr::create(MI->getOperand(1).getMBB()->getSymbol(), *Ctx);
    const FgpuMCExpr *FgpuExpr = FgpuMCExpr::create(Kind, Expr, *Ctx);
    OutMI.addOperand(MCOperand::createExpr(FgpuExpr));
  } else if (MI->getNumOperands() == 3) {
    // Create %hi($tgt-$baltgt).
    OutMI.addOperand(createSub(MI->getOperand(1).getMBB(),
                               MI->getOperand(2).getMBB(), Kind));
  }
}

void FgpuMCInstLower::lowerLongBranchADDiu(const MachineInstr *MI,
                                           MCInst &OutMI, int Opcode) const {
  OutMI.setOpcode(Opcode);

  FgpuMCExpr::FgpuExprKind Kind;
  unsigned TargetFlags = MI->getOperand(2).getTargetFlags();
  switch (TargetFlags) {
  case FgpuII::MO_HIGHEST:
    Kind = FgpuMCExpr::MEK_HIGHEST;
    break;
  case FgpuII::MO_HIGHER:
    Kind = FgpuMCExpr::MEK_HIGHER;
    break;
  case FgpuII::MO_ABS_HI:
    Kind = FgpuMCExpr::MEK_HI;
    break;
  case FgpuII::MO_ABS_LO:
    Kind = FgpuMCExpr::MEK_LO;
    break;
  default:
    report_fatal_error("Unexpected flags for lowerLongBranchADDiu");
  }

  // Lower two register operands.
  for (unsigned I = 0, E = 2; I != E; ++I) {
    const MachineOperand &MO = MI->getOperand(I);
    OutMI.addOperand(LowerOperand(MO));
  }

  if (MI->getNumOperands() == 3) {
    // Lower register operand.
    const MCExpr *Expr =
        MCSymbolRefExpr::create(MI->getOperand(2).getMBB()->getSymbol(), *Ctx);
    const FgpuMCExpr *FgpuExpr = FgpuMCExpr::create(Kind, Expr, *Ctx);
    OutMI.addOperand(MCOperand::createExpr(FgpuExpr));
  } else if (MI->getNumOperands() == 4) {
    // Create %lo($tgt-$baltgt) or %hi($tgt-$baltgt).
    OutMI.addOperand(createSub(MI->getOperand(2).getMBB(),
                               MI->getOperand(3).getMBB(), Kind));
  }
}

bool FgpuMCInstLower::lowerLongBranch(const MachineInstr *MI,
                                      MCInst &OutMI) const {
  switch (MI->getOpcode()) {
  default:
    return false;
  case Fgpu::LONG_BRANCH_LUi:
  case Fgpu::LONG_BRANCH_LUi2Op:
  case Fgpu::LONG_BRANCH_LUi2Op_64:
    lowerLongBranchLUi(MI, OutMI);
    return true;
  case Fgpu::LONG_BRANCH_ADDiu:
  case Fgpu::LONG_BRANCH_ADDiu2Op:
    lowerLongBranchADDiu(MI, OutMI, Fgpu::ADDiu);
    return true;
  case Fgpu::LONG_BRANCH_DADDiu:
  case Fgpu::LONG_BRANCH_DADDiu2Op:
    lowerLongBranchADDiu(MI, OutMI, Fgpu::DADDiu);
    return true;
  }
}

void FgpuMCInstLower::Lower(const MachineInstr *MI, MCInst &OutMI) const {
  if (lowerLongBranch(MI, OutMI))
    return;

  OutMI.setOpcode(MI->getOpcode());

  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);
    MCOperand MCOp = LowerOperand(MO);

    if (MCOp.isValid())
      OutMI.addOperand(MCOp);
  }
}
