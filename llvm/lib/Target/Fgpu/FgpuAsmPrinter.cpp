//===- FgpuAsmPrinter.cpp - Fgpu LLVM Assembly Printer --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to GAS-format FGPU assembly language.
//
//===----------------------------------------------------------------------===//

#include "FgpuAsmPrinter.h"
#include "MCTargetDesc/FgpuABIInfo.h"
#include "MCTargetDesc/FgpuBaseInfo.h"
#include "MCTargetDesc/FgpuInstPrinter.h"
#include "MCTargetDesc/FgpuMCTargetDesc.h"
#include "Fgpu.h"
#include "FgpuMCInstLower.h"
#include "FgpuMachineFunction.h"
#include "FgpuSubtarget.h"
#include "FgpuTargetMachine.h"
#include "FgpuTargetStreamer.h"
#include "TargetInfo/FgpuTargetInfo.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/Triple.h"
#include "llvm/ADT/Twine.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/InlineAsm.h"
#include "llvm/IR/Instructions.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetLoweringObjectFile.h"
#include "llvm/Target/TargetMachine.h"
#include <cassert>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace llvm;

#define DEBUG_TYPE "fgpu-asm-printer"

extern cl::opt<bool> EmitJalrReloc;

FgpuTargetStreamer &FgpuAsmPrinter::getTargetStreamer() const {
  return static_cast<FgpuTargetStreamer &>(*OutStreamer->getTargetStreamer());
}

bool FgpuAsmPrinter::runOnMachineFunction(MachineFunction &MF) {
  Subtarget = &MF.getSubtarget<FgpuSubtarget>();

  FgpuFI = MF.getInfo<FgpuFunctionInfo>();
  MCP = MF.getConstantPool();

  AsmPrinter::runOnMachineFunction(MF);

  emitXRayTable();

  return true;
}

bool FgpuAsmPrinter::lowerOperand(const MachineOperand &MO, MCOperand &MCOp) {
  MCOp = MCInstLowering.LowerOperand(MO);
  return MCOp.isValid();
}

#include "FgpuGenMCPseudoLowering.inc"

// Lower PseudoReturn/PseudoIndirectBranch/PseudoIndirectBranch64 to JR, JR_MM,
// JALR, or JALR64 as appropriate for the target.
void FgpuAsmPrinter::emitPseudoIndirectBranch(MCStreamer &OutStreamer,
                                              const MachineInstr *MI) {
  bool HasLinkReg = false;
  MCInst TmpInst0;
  assert(false && "I haven't implemented this");
  return;
  //TODO: make work
  //if (Subtarget->hasFgpu64r6()) {
  //  // FGPU64r6 should use (JALR64 ZERO_64, $rs)
  //  TmpInst0.setOpcode(Fgpu::JALR64);
  //  HasLinkReg = true;
  //} else if (Subtarget->hasFgpu32r6()) {
  //  // FGPU32r6 should use (JALR ZERO, $rs)
  //  TmpInst0.setOpcode(Fgpu::JALR);
  //  HasLinkReg = true;
  //} else {
  //  // Everything else should use (JR $rs)
  //  TmpInst0.setOpcode(Fgpu::JR);
  //}

  MCOperand MCOp;

  if (HasLinkReg) {
    unsigned ZeroReg = Fgpu::ZERO;
    TmpInst0.addOperand(MCOperand::createReg(ZeroReg));
  }

  lowerOperand(MI->getOperand(0), MCOp);
  TmpInst0.addOperand(MCOp);

  EmitToStreamer(OutStreamer, TmpInst0);
}

// If there is an MO_JALR operand, insert:
//
// .reloc tmplabel, R_{MICRO}FGPU_JALR, symbol
// tmplabel:
//
// This is an optimization hint for the linker which may then replace
// an indirect call with a direct branch.
static void emitDirectiveRelocJalr(const MachineInstr &MI,
                                   MCContext &OutContext,
                                   TargetMachine &TM,
                                   MCStreamer &OutStreamer,
                                   const FgpuSubtarget &Subtarget) {
  for (unsigned int I = MI.getDesc().getNumOperands(), E = MI.getNumOperands();
       I < E; ++I) {
    MachineOperand MO = MI.getOperand(I);
    if (MO.isMCSymbol() && (MO.getTargetFlags() & FgpuII::MO_JALR)) {
      MCSymbol *Callee = MO.getMCSymbol();
      if (Callee && !Callee->getName().empty()) {
        MCSymbol *OffsetLabel = OutContext.createTempSymbol();
        const MCExpr *OffsetExpr =
            MCSymbolRefExpr::create(OffsetLabel, OutContext);
        const MCExpr *CaleeExpr =
            MCSymbolRefExpr::create(Callee, OutContext);
        OutStreamer.emitRelocDirective(
            *OffsetExpr,
            "R_FGPU_JALR",
            CaleeExpr, SMLoc(), *TM.getMCSubtargetInfo());
        OutStreamer.emitLabel(OffsetLabel);
        return;
      }
    }
  }
}

void FgpuAsmPrinter::emitInstruction(const MachineInstr *MI) {
  FgpuTargetStreamer &TS = getTargetStreamer();
  unsigned Opc = MI->getOpcode();
  TS.forbidModuleDirective();

  if (MI->isDebugValue()) {
    SmallString<128> Str;
    raw_svector_ostream OS(Str);

    PrintDebugValueComment(MI, OS);
    return;
  }
  if (MI->isDebugLabel())
    return;

  switch (Opc) {
  case Fgpu::PATCHABLE_FUNCTION_ENTER:
    LowerPATCHABLE_FUNCTION_ENTER(*MI);
    return;
  case Fgpu::PATCHABLE_FUNCTION_EXIT:
    LowerPATCHABLE_FUNCTION_EXIT(*MI);
    return;
  case Fgpu::PATCHABLE_TAIL_CALL:
    LowerPATCHABLE_TAIL_CALL(*MI);
    return;
  }

  if (EmitJalrReloc &&
      (MI->isReturn() || MI->isCall() || MI->isIndirectBranch())) {
    emitDirectiveRelocJalr(*MI, OutContext, TM, *OutStreamer, *Subtarget);
  }

  MachineBasicBlock::const_instr_iterator I = MI->getIterator();
  MachineBasicBlock::const_instr_iterator E = MI->getParent()->instr_end();

  do {
    // Do any auto-generated pseudo lowerings.
    if (emitPseudoExpansionLowering(*OutStreamer, &*I))
      continue;

    // Skip the BUNDLE pseudo instruction and lower the contents
    if (I->isBundle())
      continue;

    //TODO: add these back if should
    //if (I->getOpcode() == Fgpu::PseudoReturn ||
    //    I->getOpcode() == Fgpu::PseudoReturn64 ||
    //    I->getOpcode() == Fgpu::PseudoIndirectBranch ||
    //    I->getOpcode() == Fgpu::PseudoIndirectBranch64 ||
    //    I->getOpcode() == Fgpu::TAILCALLREG ||
    //    I->getOpcode() == Fgpu::TAILCALLREG64) {
    //  emitPseudoIndirectBranch(*OutStreamer, &*I);
    //  continue;
    //}

    // The inFgpu16Mode() test is not permanent.
    // Some instructions are marked as pseudo right now which
    // would make the test fail for the wrong reason but
    // that will be fixed soon. We need this here because we are
    // removing another test for this situation downstream in the
    // callchain.
    //
    if (I->isPseudo()
        && !isLongBranchPseudo(I->getOpcode()))
      llvm_unreachable("Pseudo opcode found in emitInstruction()");

    MCInst TmpInst0;
    MCInstLowering.Lower(&*I, TmpInst0);
    EmitToStreamer(*OutStreamer, TmpInst0);
  } while ((++I != E) && I->isInsideBundle()); // Delay slot check
}

//===----------------------------------------------------------------------===//
//
//  Fgpu Asm Directives
//
//  -- Frame directive "frame Stackpointer, Stacksize, RARegister"
//  Describe the stack frame.
//
//  -- Mask directives "(f)mask  bitmask, offset"
//  Tells the assembler which registers are saved and where.
//  bitmask - contain a little endian bitset indicating which registers are
//            saved on function prologue (e.g. with a 0x80000000 mask, the
//            assembler knows the register 31 (RA) is saved at prologue.
//  offset  - the position before stack pointer subtraction indicating where
//            the first saved register on prologue is located. (e.g. with a
//
//  Consider the following function prologue:
//
//    .frame  $fp,48,$ra
//    .mask   0xc0000000,-8
//       addiu $sp, $sp, -48
//       sw $ra, 40($sp)
//       sw $fp, 36($sp)
//
//    With a 0xc0000000 mask, the assembler knows the register 31 (RA) and
//    30 (FP) are saved at prologue. As the save order on prologue is from
//    left to right, RA is saved first. A -8 offset means that after the
//    stack pointer subtration, the first register in the mask (RA) will be
//    saved at address 48-8=40.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Mask directives
//===----------------------------------------------------------------------===//

// Create a bitmask with all callee saved registers for CPU or Floating Point
// registers. For CPU registers consider RA, GP and FP for saving if necessary.
void FgpuAsmPrinter::printSavedRegsBitmask() {
  // CPU and FPU Saved Registers Bitmasks
  unsigned CPUBitmask = 0, FPUBitmask = 0;
  int CPUTopSavedRegOff, FPUTopSavedRegOff;

  // Set the CPU and FPU Bitmasks
  const MachineFrameInfo &MFI = MF->getFrameInfo();
  const TargetRegisterInfo *TRI = MF->getSubtarget().getRegisterInfo();
  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  // size of stack area to which FP callee-saved regs are saved.
  unsigned CPURegSize = TRI->getRegSizeInBits(Fgpu::GPROutRegClass) / 8;
  unsigned FPURegSize = TRI->getRegSizeInBits(Fgpu::VecRegsRegClass) / 8;
  unsigned CSFPRegsSize = 0;

  for (const auto &I : CSI) {
    Register Reg = I.getReg();
    unsigned RegNum = TRI->getEncodingValue(Reg);

    // If it's a floating point register, set the FPU Bitmask.
    // If it's a general purpose register, set the CPU Bitmask.
    if (Fgpu::VecRegsRegClass.contains(Reg)) {
      FPUBitmask |= (1 << RegNum);
      CSFPRegsSize += FPURegSize;
    } else if (Fgpu::GPROutRegClass.contains(Reg))
      CPUBitmask |= (1 << RegNum);
  }

  // FP Regs are saved right below where the virtual frame pointer points to.
  FPUTopSavedRegOff = FPUBitmask ? -FPURegSize : 0;

  // CPU Regs are saved below FP Regs.
  CPUTopSavedRegOff = CPUBitmask ? -CSFPRegsSize - CPURegSize : 0;

  FgpuTargetStreamer &TS = getTargetStreamer();
  // Print CPUBitmask
  TS.emitMask(CPUBitmask, CPUTopSavedRegOff);

  // Print FPUBitmask
  TS.emitFMask(FPUBitmask, FPUTopSavedRegOff);
}

//===----------------------------------------------------------------------===//
// Frame and Set directives
//===----------------------------------------------------------------------===//

/// Frame Directive
void FgpuAsmPrinter::emitFrameDirective() {
  const TargetRegisterInfo &RI = *MF->getSubtarget().getRegisterInfo();

  Register stackReg = RI.getFrameRegister(*MF);
  unsigned returnReg = RI.getRARegister();
  unsigned stackSize = MF->getFrameInfo().getStackSize();

  getTargetStreamer().emitFrame(stackReg, stackSize, returnReg);
}

/// Emit Set directives.
const char *FgpuAsmPrinter::getCurrentABIString() const {
  switch (static_cast<FgpuTargetMachine &>(TM).getABI().GetEnumValue()) {
  case FgpuABIInfo::ABI::CC_Fgpu:  return "cc_fgpu";
  default: llvm_unreachable("Unknown Fgpu ABI");
  }
}

void FgpuAsmPrinter::emitFunctionEntryLabel() {
  FgpuTargetStreamer &TS = getTargetStreamer();

  TS.emitDirectiveEnt(*CurrentFnSym);
  OutStreamer->emitLabel(CurrentFnSym);
}

/// EmitFunctionBodyStart - Targets can override this to emit stuff before
/// the first basic block in the function.
void FgpuAsmPrinter::emitFunctionBodyStart() {
  FgpuTargetStreamer &TS = getTargetStreamer();

  MCInstLowering.Initialize(&MF->getContext());

  bool IsNakedFunction = MF->getFunction().hasFnAttribute(Attribute::Naked);
  if (!IsNakedFunction)
    emitFrameDirective();

  if (!IsNakedFunction)
    printSavedRegsBitmask();

  TS.emitDirectiveSetNoReorder();
  TS.emitDirectiveSetNoMacro();
  TS.emitDirectiveSetNoAt();
}

/// EmitFunctionBodyEnd - Targets can override this to emit stuff after
/// the last basic block in the function.
void FgpuAsmPrinter::emitFunctionBodyEnd() {
  FgpuTargetStreamer &TS = getTargetStreamer();

  // There are instruction for this macros, but they must
  // always be at the function end, and we can't emit and
  // break with BB logic.
  TS.emitDirectiveSetAt();
  TS.emitDirectiveSetMacro();
  TS.emitDirectiveSetReorder();
  TS.emitDirectiveEnd(CurrentFnSym->getName());
  // Make sure to terminate any constant pools that were at the end
  // of the function.
  if (!InConstantPool)
    return;
  InConstantPool = false;
  OutStreamer->emitDataRegion(MCDR_DataRegionEnd);
}

void FgpuAsmPrinter::emitBasicBlockEnd(const MachineBasicBlock &MBB) {
  AsmPrinter::emitBasicBlockEnd(MBB);
  FgpuTargetStreamer &TS = getTargetStreamer();
  if (MBB.empty())
    TS.emitDirectiveInsn();
}

/// isBlockOnlyReachableByFallthough - Return true if the basic block has
/// exactly one predecessor and the control transfer mechanism between
/// the predecessor and this block is a fall-through.
bool FgpuAsmPrinter::isBlockOnlyReachableByFallthrough(const MachineBasicBlock*
                                                       MBB) const {
  // The predecessor has to be immediately before this block.
  const MachineBasicBlock *Pred = *MBB->pred_begin();

  // If the predecessor is a switch statement, assume a jump table
  // implementation, so it is not a fall through.
  if (const BasicBlock *bb = Pred->getBasicBlock())
    if (isa<SwitchInst>(bb->getTerminator()))
      return false;

  // If this is a landing pad, it isn't a fall through.  If it has no preds,
  // then nothing falls through to it.
  if (MBB->isEHPad() || MBB->pred_empty())
    return false;

  // If there isn't exactly one predecessor, it can't be a fall through.
  MachineBasicBlock::const_pred_iterator PI = MBB->pred_begin(), PI2 = PI;
  ++PI2;

  if (PI2 != MBB->pred_end())
    return false;

  // The predecessor has to be immediately before this block.
  if (!Pred->isLayoutSuccessor(MBB))
    return false;

  // If the block is completely empty, then it definitely does fall through.
  if (Pred->empty())
    return true;

  // Otherwise, check the last instruction.
  // Check if the last terminator is an unconditional branch.
  MachineBasicBlock::const_iterator I = Pred->end();
  while (I != Pred->begin() && !(--I)->isTerminator()) ;

  return !I->isBarrier();
}

// Print out an operand for an inline asm expression.
bool FgpuAsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNum,
                                     const char *ExtraCode, raw_ostream &O) {
  // Does this asm operand have a single letter operand modifier?
  if (ExtraCode && ExtraCode[0]) {
    if (ExtraCode[1] != 0) return true; // Unknown modifier.

    const MachineOperand &MO = MI->getOperand(OpNum);
    switch (ExtraCode[0]) {
    default:
      // See if this is a generic print operand
      return AsmPrinter::PrintAsmOperand(MI, OpNum, ExtraCode, O);
    case 'X': // hex const int
      if ((MO.getType()) != MachineOperand::MO_Immediate)
        return true;
      O << "0x" << Twine::utohexstr(MO.getImm());
      return false;
    case 'x': // hex const int (low 16 bits)
      if ((MO.getType()) != MachineOperand::MO_Immediate)
        return true;
      O << "0x" << Twine::utohexstr(MO.getImm() & 0xffff);
      return false;
    case 'd': // decimal const int
      if ((MO.getType()) != MachineOperand::MO_Immediate)
        return true;
      O << MO.getImm();
      return false;
    case 'm': // decimal const int minus 1
      if ((MO.getType()) != MachineOperand::MO_Immediate)
        return true;
      O << MO.getImm() - 1;
      return false;
    case 'y': // exact log2
      if ((MO.getType()) != MachineOperand::MO_Immediate)
        return true;
      if (!isPowerOf2_64(MO.getImm()))
        return true;
      O << Log2_64(MO.getImm());
      return false;
    case 'z':
      // $0 if zero, regular printing otherwise
      if (MO.getType() == MachineOperand::MO_Immediate && MO.getImm() == 0) {
        O << "$0";
        return false;
      }
      // If not, call printOperand as normal.
      break;
    case 'D': // Second part of a double word register operand
    case 'L': // Low order register of a double word register operand
    case 'M': // High order register of a double word register operand
    {
      if (OpNum == 0)
        return true;
      const MachineOperand &FlagsOP = MI->getOperand(OpNum - 1);
      if (!FlagsOP.isImm())
        return true;
      unsigned Flags = FlagsOP.getImm();
      unsigned NumVals = InlineAsm::getNumOperandRegisters(Flags);
      // Number of registers represented by this operand. We are looking
      // for 2 for 32 bit mode and 1 for 64 bit mode.
      if (NumVals != 2) {
        return true;
      }

      unsigned RegOp = OpNum;
      if (true){
        // Endianness reverses which register holds the high or low value
        // between M and L.
        switch(ExtraCode[0]) {
        case 'M':
          RegOp = OpNum + 1;
          break;
        case 'L':
          RegOp = OpNum;
          break;
        case 'D': // Always the second part
          RegOp = OpNum + 1;
        }
        if (RegOp >= MI->getNumOperands())
          return true;
        const MachineOperand &MO = MI->getOperand(RegOp);
        if (!MO.isReg())
          return true;
        Register Reg = MO.getReg();
        O << '$' << FgpuInstPrinter::getRegisterName(Reg);
        return false;
      }
      break;
    }
    case 'w':
      // Print MSA registers for the 'f' constraint
      // In LLVM, the 'w' modifier doesn't need to do anything.
      // We can just call printOperand as normal.
      break;
    }
  }

  printOperand(MI, OpNum, O);
  return false;
}

bool FgpuAsmPrinter::PrintAsmMemoryOperand(const MachineInstr *MI,
                                           unsigned OpNum,
                                           const char *ExtraCode,
                                           raw_ostream &O) {
  assert(OpNum + 1 < MI->getNumOperands() && "Insufficient operands");
  const MachineOperand &BaseMO = MI->getOperand(OpNum);
  const MachineOperand &OffsetMO = MI->getOperand(OpNum + 1);
  assert(BaseMO.isReg() &&
         "Unexpected base pointer for inline asm memory operand.");
  assert(OffsetMO.isImm() &&
         "Unexpected offset for inline asm memory operand.");
  int Offset = OffsetMO.getImm();

  // Currently we are expecting either no ExtraCode or 'D','M','L'.
  if (ExtraCode) {
    switch (ExtraCode[0]) {
    case 'D':
      Offset += 4;
      break;
    case 'M':
      Offset += 4;
      break;
    case 'L':
      break;
    default:
      return true; // Unknown modifier.
    }
  }

  O << Offset << "($" << FgpuInstPrinter::getRegisterName(BaseMO.getReg())
    << ")";

  return false;
}

void FgpuAsmPrinter::printOperand(const MachineInstr *MI, int opNum,
                                  raw_ostream &O) {
  const MachineOperand &MO = MI->getOperand(opNum);
  bool closeP = false;

  if (MO.getTargetFlags())
    closeP = true;

  switch(MO.getTargetFlags()) {
  case FgpuII::MO_GPREL:    O << "%gp_rel("; break;
  case FgpuII::MO_GOT_CALL: O << "%call16("; break;
  case FgpuII::MO_GOT:      O << "%got(";    break;
  case FgpuII::MO_ABS_HI:   O << "%hi(";     break;
  case FgpuII::MO_ABS_LO:   O << "%lo(";     break;
  case FgpuII::MO_TLSGD:    O << "%tlsgd(";  break;
  case FgpuII::MO_GOTTPREL: O << "%gottprel("; break;
  case FgpuII::MO_TPREL_HI: O << "%tprel_hi("; break;
  case FgpuII::MO_TPREL_LO: O << "%tprel_lo("; break;
  case FgpuII::MO_GPOFF_HI: O << "%hi(%neg(%gp_rel("; break;
  case FgpuII::MO_GPOFF_LO: O << "%lo(%neg(%gp_rel("; break;
  case FgpuII::MO_GOT_DISP: O << "%got_disp("; break;
  case FgpuII::MO_GOT_PAGE: O << "%got_page("; break;
  case FgpuII::MO_GOT_OFST: O << "%got_ofst("; break;
  }

  switch (MO.getType()) {
    case MachineOperand::MO_Register:
      O << '$'
        << StringRef(FgpuInstPrinter::getRegisterName(MO.getReg())).lower();
      break;

    case MachineOperand::MO_Immediate:
      O << MO.getImm();
      break;

    case MachineOperand::MO_MachineBasicBlock:
      MO.getMBB()->getSymbol()->print(O, MAI);
      return;

    case MachineOperand::MO_GlobalAddress:
      PrintSymbolOperand(MO, O);
      break;

    case MachineOperand::MO_BlockAddress: {
      MCSymbol *BA = GetBlockAddressSymbol(MO.getBlockAddress());
      O << BA->getName();
      break;
    }

    case MachineOperand::MO_ConstantPoolIndex:
      O << getDataLayout().getPrivateGlobalPrefix() << "CPI"
        << getFunctionNumber() << "_" << MO.getIndex();
      if (MO.getOffset())
        O << "+" << MO.getOffset();
      break;

    default:
      llvm_unreachable("<unknown operand type>");
  }

  if (closeP) O << ")";
}

void FgpuAsmPrinter::
printMemOperand(const MachineInstr *MI, int opNum, raw_ostream &O) {
  // Load/Store memory operands -- imm($reg)
  // If PIC target the target is loaded as the
  // pattern lw $25,%call16($28)

  // opNum can be invalid if instruction has reglist as operand.
  // MemOperand is always last operand of instruction (base + offset).
  switch (MI->getOpcode()) {
  default:
    break;
  }

  printOperand(MI, opNum+1, O);
  O << "(";
  printOperand(MI, opNum, O);
  O << ")";
}

void FgpuAsmPrinter::
printMemOperandEA(const MachineInstr *MI, int opNum, raw_ostream &O) {
  // when using stack locations for not load/store instructions
  // print the same way as all normal 3 operand instructions.
  printOperand(MI, opNum, O);
  O << ", ";
  printOperand(MI, opNum+1, O);
}

void FgpuAsmPrinter::
printFCCOperand(const MachineInstr *MI, int opNum, raw_ostream &O,
                const char *Modifier) {
  const MachineOperand &MO = MI->getOperand(opNum);
  O << Fgpu::FgpuFCCToString((Fgpu::CondCode)MO.getImm());
}

void FgpuAsmPrinter::
printRegisterList(const MachineInstr *MI, int opNum, raw_ostream &O) {
  for (int i = opNum, e = MI->getNumOperands(); i != e; ++i) {
    if (i != opNum) O << ", ";
    printOperand(MI, i, O);
  }
}

void FgpuAsmPrinter::emitStartOfAsmFile(Module &M) {
  FgpuTargetStreamer &TS = getTargetStreamer();

  // FgpuTargetStreamer has an initialization order problem when emitting an
  // object file directly (see FgpuTargetELFStreamer for full details). Work
  // around it by re-initializing the PIC state here.
  TS.setPic(OutContext.getObjectFileInfo()->isPositionIndependent());

  // Compute FGPU architecture attributes based on the default subtarget
  // that we'd have constructed. Module level directives aren't LTO
  // clean anyhow.
  // FIXME: For ifunc related functions we could iterate over and look
  // for a feature string that doesn't match the default one.
  const Triple &TT = TM.getTargetTriple();
  StringRef CPU = FGPU_MC::selectFgpuCPU(TT, TM.getTargetCPU());
  StringRef FS = TM.getTargetFeatureString();
  const FgpuTargetMachine &MTM = static_cast<const FgpuTargetMachine &>(TM);
  const FgpuSubtarget STI(TT, CPU, FS, MTM, None);

  bool IsABICalls = false;//STI.isABICalls();
  const FgpuABIInfo &ABI = MTM.getABI();
  if (IsABICalls) {
    //TS.emitDirectiveAbiCalls(); // TODO: what is this supposed to do?
    // FIXME: This condition should be a lot more complicated that it is here.
    //        Ideally it should test for properties of the ABI and not the ABI
    //        itself.
    //        For the moment, I'm only correcting enough to make FGPU-IV work.
    if (!isPositionIndependent())
      TS.emitDirectiveOptionPic0();
  }

  // Tell the assembler which ABI we are using
  std::string SectionName = std::string(".mdebug.") + getCurrentABIString();
  OutStreamer->SwitchSection(
      OutContext.getELFSection(SectionName, ELF::SHT_PROGBITS, 0));

  // TODO: handle O64 ABI

  TS.updateABIInfo(STI);

  // Switch to the .text section.
  OutStreamer->SwitchSection(getObjFileLowering().getTextSection());
}

void FgpuAsmPrinter::emitInlineAsmStart() const {
  FgpuTargetStreamer &TS = getTargetStreamer();

  // GCC's choice of assembler options for inline assembly code ('at', 'macro'
  // and 'reorder') is different from LLVM's choice for generated code ('noat',
  // 'nomacro' and 'noreorder').
  // In order to maintain compatibility with inline assembly code which depends
  // on GCC's assembler options being used, we have to switch to those options
  // for the duration of the inline assembly block and then switch back.
  TS.emitDirectiveSetPush();
  TS.emitDirectiveSetAt();
  TS.emitDirectiveSetMacro();
  TS.emitDirectiveSetReorder();
  OutStreamer->AddBlankLine();
}

void FgpuAsmPrinter::emitInlineAsmEnd(const MCSubtargetInfo &StartInfo,
                                      const MCSubtargetInfo *EndInfo) const {
  OutStreamer->AddBlankLine();
  getTargetStreamer().emitDirectiveSetPop();
}

void FgpuAsmPrinter::EmitJal(const MCSubtargetInfo &STI, MCSymbol *Symbol) {
  MCInst I;
  I.setOpcode(Fgpu::JSUB);
  I.addOperand(
      MCOperand::createExpr(MCSymbolRefExpr::create(Symbol, OutContext)));
  OutStreamer->emitInstruction(I, STI);
}

void FgpuAsmPrinter::EmitInstrReg(const MCSubtargetInfo &STI, unsigned Opcode,
                                  unsigned Reg) {
  MCInst I;
  I.setOpcode(Opcode);
  I.addOperand(MCOperand::createReg(Reg));
  OutStreamer->emitInstruction(I, STI);
}

void FgpuAsmPrinter::EmitInstrRegReg(const MCSubtargetInfo &STI,
                                     unsigned Opcode, unsigned Reg1,
                                     unsigned Reg2) {
  MCInst I;
  //
  // Because of the current td files for Fgpu32, the operands for MTC1
  // appear backwards from their normal assembly order. It's not a trivial
  // change to fix this in the td file so we adjust for it here.
  //
  //if (Opcode == Fgpu::MTC1) {
  //  unsigned Temp = Reg1;
  //  Reg1 = Reg2;
  //  Reg2 = Temp;
  //}
  I.setOpcode(Opcode);
  I.addOperand(MCOperand::createReg(Reg1));
  I.addOperand(MCOperand::createReg(Reg2));
  OutStreamer->emitInstruction(I, STI);
}

void FgpuAsmPrinter::EmitInstrRegRegReg(const MCSubtargetInfo &STI,
                                        unsigned Opcode, unsigned Reg1,
                                        unsigned Reg2, unsigned Reg3) {
  MCInst I;
  I.setOpcode(Opcode);
  I.addOperand(MCOperand::createReg(Reg1));
  I.addOperand(MCOperand::createReg(Reg2));
  I.addOperand(MCOperand::createReg(Reg3));
  OutStreamer->emitInstruction(I, STI);
}

void FgpuAsmPrinter::EmitMovFPIntPair(const MCSubtargetInfo &STI,
                                      unsigned MovOpc, unsigned Reg1,
                                      unsigned Reg2, unsigned FPReg1,
                                      unsigned FPReg2, bool LE) {
  if (!LE) {
    unsigned temp = Reg1;
    Reg1 = Reg2;
    Reg2 = temp;
  }
  EmitInstrRegReg(STI, MovOpc, Reg1, FPReg1);
  EmitInstrRegReg(STI, MovOpc, Reg2, FPReg2);
}

void FgpuAsmPrinter::emitEndOfAsmFile(Module &M) {
  // return to the text section
  OutStreamer->SwitchSection(OutContext.getObjectFileInfo()->getTextSection());
}

void FgpuAsmPrinter::EmitSled(const MachineInstr &MI, SledKind Kind) {
  const uint8_t NoopsInSledCount = 11;
  // For fgpu32 we want to emit the following pattern:
  //
  // .Lxray_sled_N:
  //   ALIGN
  //   B .tmpN
  //   11 NOP instructions (44 bytes)
  //   ADDIU T9, T9, 52
  // .tmpN
  //
  // We need the 44 bytes (11 instructions) because at runtime, we'd
  // be patching over the full 48 bytes (12 instructions) with the following
  // pattern:
  //
  //   ADDIU    SP, SP, -8
  //   NOP
  //   SW       RA, 4(SP)
  //   SW       T9, 0(SP)
  //   LUI      T9, %hi(__xray_FunctionEntry/Exit)
  //   ORI      T9, T9, %lo(__xray_FunctionEntry/Exit)
  //   LUI      T0, %hi(function_id)
  //   JALR     T9
  //   ORI      T0, T0, %lo(function_id)
  //   LW       T9, 0(SP)
  //   LW       RA, 4(SP)
  //   ADDIU    SP, SP, 8
  //
  // We add 52 bytes to t9 because we want to adjust the function pointer to
  // the actual start of function i.e. the address just after the noop sled.
  // We do this because gp displacement relocation is emitted at the start of
  // of the function i.e after the nop sled and to correctly calculate the
  // global offset table address, t9 must hold the address of the instruction
  // containing the gp displacement relocation.
  // FIXME: Is this correct for the static relocation model?
  //
  // For fgpu64 we want to emit the following pattern:
  //
  // .Lxray_sled_N:
  //   ALIGN
  //   B .tmpN
  //   15 NOP instructions (60 bytes)
  // .tmpN
  //
  // We need the 60 bytes (15 instructions) because at runtime, we'd
  // be patching over the full 64 bytes (16 instructions) with the following
  // pattern:
  //
  //   DADDIU   SP, SP, -16
  //   NOP
  //   SD       RA, 8(SP)
  //   SD       T9, 0(SP)
  //   LUI      T9, %highest(__xray_FunctionEntry/Exit)
  //   ORI      T9, T9, %higher(__xray_FunctionEntry/Exit)
  //   DSLL     T9, T9, 16
  //   ORI      T9, T9, %hi(__xray_FunctionEntry/Exit)
  //   DSLL     T9, T9, 16
  //   ORI      T9, T9, %lo(__xray_FunctionEntry/Exit)
  //   LUI      T0, %hi(function_id)
  //   JALR     T9
  //   ADDIU    T0, T0, %lo(function_id)
  //   LD       T9, 0(SP)
  //   LD       RA, 8(SP)
  //   DADDIU   SP, SP, 16
  //
  OutStreamer->emitCodeAlignment(4);
  auto CurSled = OutContext.createTempSymbol("xray_sled_", true);
  OutStreamer->emitLabel(CurSled);
  auto Target = OutContext.createTempSymbol();

  // Emit "B .tmpN" instruction, which jumps over the nop sled to the actual
  // start of function
  const MCExpr *TargetExpr = MCSymbolRefExpr::create(
      Target, MCSymbolRefExpr::VariantKind::VK_None, OutContext);
  EmitToStreamer(*OutStreamer, MCInstBuilder(Fgpu::BEQ)
                                   .addReg(Fgpu::ZERO)
                                   .addReg(Fgpu::ZERO)
                                   .addExpr(TargetExpr));

  for (int8_t I = 0; I < NoopsInSledCount; I++)
    EmitToStreamer(*OutStreamer, MCInstBuilder(Fgpu::SLL)
                                     .addReg(Fgpu::ZERO)
                                     .addReg(Fgpu::ZERO)
                                     .addImm(0));

  OutStreamer->emitLabel(Target);

    EmitToStreamer(*OutStreamer,
                   MCInstBuilder(Fgpu::ADDi)
                       .addReg(Fgpu::R21)
                       .addReg(Fgpu::R21)
                       .addImm(0x34));

  recordSled(CurSled, MI, Kind, 2);
}

void FgpuAsmPrinter::LowerPATCHABLE_FUNCTION_ENTER(const MachineInstr &MI) {
  EmitSled(MI, SledKind::FUNCTION_ENTER);
}

void FgpuAsmPrinter::LowerPATCHABLE_FUNCTION_EXIT(const MachineInstr &MI) {
  EmitSled(MI, SledKind::FUNCTION_EXIT);
}

void FgpuAsmPrinter::LowerPATCHABLE_TAIL_CALL(const MachineInstr &MI) {
  EmitSled(MI, SledKind::TAIL_CALL);
}

void FgpuAsmPrinter::PrintDebugValueComment(const MachineInstr *MI,
                                           raw_ostream &OS) {
  // TODO: implement
}

// Emit .dtprelword or .dtpreldword directive
// and value for debug thread local expression.
void FgpuAsmPrinter::emitDebugValue(const MCExpr *Value, unsigned Size) const {
  if (auto *FgpuExpr = dyn_cast<FgpuMCExpr>(Value)) {
    if (FgpuExpr && FgpuExpr->getKind() == FgpuMCExpr::MEK_DTPREL) {
      switch (Size) {
      case 4:
        OutStreamer->emitDTPRel32Value(FgpuExpr->getSubExpr());
        break;
      case 8:
        OutStreamer->emitDTPRel64Value(FgpuExpr->getSubExpr());
        break;
      default:
        llvm_unreachable("Unexpected size of expression value.");
      }
      return;
    }
  }
  AsmPrinter::emitDebugValue(Value, Size);
}

bool FgpuAsmPrinter::isLongBranchPseudo(int Opcode) const {
  return false;
//  return (Opcode == Fgpu::LONG_BRANCH_LUi
//          || Opcode == Fgpu::LONG_BRANCH_LUi2Op
//          || Opcode == Fgpu::LONG_BRANCH_LUi2Op_64
//          || Opcode == Fgpu::LONG_BRANCH_ADDiu
//          || Opcode == Fgpu::LONG_BRANCH_ADDiu2Op
//          || Opcode == Fgpu::LONG_BRANCH_DADDiu
//          || Opcode == Fgpu::LONG_BRANCH_DADDiu2Op);
}

// Force static initialization.
extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeFgpuAsmPrinter() {
  RegisterAsmPrinter<FgpuAsmPrinter> X(getTheFgpuTarget());
}
