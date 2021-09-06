//===- FgpuAsmPrinter.h - Fgpu LLVM Assembly Printer -----------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Fgpu Assembly printer class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Fgpu_FgpuASMPRINTER_H
#define LLVM_LIB_TARGET_Fgpu_FgpuASMPRINTER_H

#include "FgpuMCInstLower.h"
#include "FgpuSubtarget.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/Support/Compiler.h"
#include <algorithm>
#include <map>
#include <memory>

namespace llvm {

class MCOperand;
class MCSubtargetInfo;
class MCSymbol;
class MachineBasicBlock;
class MachineConstantPool;
class MachineFunction;
class MachineInstr;
class MachineOperand;
class FgpuFunctionInfo;
class FgpuTargetStreamer;
class Module;
class raw_ostream;
class TargetMachine;

class LLVM_LIBRARY_VISIBILITY FgpuAsmPrinter : public AsmPrinter {
  FgpuTargetStreamer &getTargetStreamer() const;

  void EmitInstrWithMacroNoAT(const MachineInstr *MI);

private:
  /// MCP - Keep a pointer to constantpool entries of the current
  /// MachineFunction.
  const MachineConstantPool *MCP = nullptr;

  /// InConstantPool - Maintain state when emitting a sequence of constant
  /// pool entries so we can properly mark them as data regions.
  bool InConstantPool = false;

  // tblgen'erated function.
  bool emitPseudoExpansionLowering(MCStreamer &OutStreamer,
                                   const MachineInstr *MI);

  // lowerOperand - Convert a MachineOperand into the equivalent MCOperand.
  bool lowerOperand(const MachineOperand &MO, MCOperand &MCOp);

  void emitInlineAsmStart() const override;

  void emitInlineAsmEnd(const MCSubtargetInfo &StartInfo,
                        const MCSubtargetInfo *EndInfo) const override;


public:
  const FgpuSubtarget *Subtarget;
  const FgpuFunctionInfo *FgpuFI;
  FgpuMCInstLower MCInstLowering;

  explicit FgpuAsmPrinter(TargetMachine &TM,
                          std::unique_ptr<MCStreamer> Streamer)
      : AsmPrinter(TM, std::move(Streamer)), MCInstLowering(*this) {}

  StringRef getPassName() const override { return "Fgpu Assembly Printer"; }

  bool runOnMachineFunction(MachineFunction &MF) override;

  void emitConstantPool() override {
    bool UsingConstantPools =
      (Subtarget->inFgpu16Mode() && Subtarget->useConstantIslands());
    if (!UsingConstantPools)
      AsmPrinter::emitConstantPool();
    // we emit constant pools customly!
  }

  void emitInstruction(const MachineInstr *MI) override;
  void printSavedRegsBitmask();
  void emitFrameDirective();
  const char *getCurrentABIString() const;
  void emitFunctionEntryLabel() override;
  void emitFunctionBodyStart() override;
  void emitFunctionBodyEnd() override;
  void emitBasicBlockEnd(const MachineBasicBlock &MBB) override;
  bool isBlockOnlyReachableByFallthrough(
                                   const MachineBasicBlock* MBB) const override;
  bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                       const char *ExtraCode, raw_ostream &O) override;
  bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNum,
                             const char *ExtraCode, raw_ostream &O) override;
  void printOperand(const MachineInstr *MI, int opNum, raw_ostream &O);
  void printMemOperand(const MachineInstr *MI, int opNum, raw_ostream &O);
  void printMemOperandEA(const MachineInstr *MI, int opNum, raw_ostream &O);
  void printFCCOperand(const MachineInstr *MI, int opNum, raw_ostream &O,
                       const char *Modifier = nullptr);
  void printRegisterList(const MachineInstr *MI, int opNum, raw_ostream &O);
  void emitStartOfAsmFile(Module &M) override;
  void emitEndOfAsmFile(Module &M) override;
  void PrintDebugValueComment(const MachineInstr *MI, raw_ostream &OS);
  void emitDebugValue(const MCExpr *Value, unsigned Size) const override;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_Fgpu_FgpuASMPRINTER_H
