//===-------- FgpuELFStreamer.cpp - ELF Object Output ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "FgpuELFStreamer.h"
#include "FgpuOptionRecord.h"
#include "FgpuTargetStreamer.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDwarf.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/Casting.h"

using namespace llvm;

FgpuELFStreamer::FgpuELFStreamer(MCContext &Context,
                                 std::unique_ptr<MCAsmBackend> MAB,
                                 std::unique_ptr<MCObjectWriter> OW,
                                 std::unique_ptr<MCCodeEmitter> Emitter)
    : MCELFStreamer(Context, std::move(MAB), std::move(OW),
                    std::move(Emitter)) {
  RegInfoRecord = new FgpuRegInfoRecord(this, Context);
  FgpuOptionRecords.push_back(
      std::unique_ptr<FgpuRegInfoRecord>(RegInfoRecord));
}

void FgpuELFStreamer::emitInstruction(const MCInst &Inst,
                                      const MCSubtargetInfo &STI) {
  MCELFStreamer::emitInstruction(Inst, STI);

  MCContext &Context = getContext();
  const MCRegisterInfo *MCRegInfo = Context.getRegisterInfo();

  for (unsigned OpIndex = 0; OpIndex < Inst.getNumOperands(); ++OpIndex) {
    const MCOperand &Op = Inst.getOperand(OpIndex);

    if (!Op.isReg())
      continue;

    unsigned Reg = Op.getReg();
    RegInfoRecord->SetPhysRegUsed(Reg, MCRegInfo);
  }

  createPendingLabelRelocs();
}

void FgpuELFStreamer::emitCFIStartProcImpl(MCDwarfFrameInfo &Frame) {
  Frame.Begin = getContext().createTempSymbol();
  MCELFStreamer::emitLabel(Frame.Begin);
}

MCSymbol *FgpuELFStreamer::emitCFILabel() {
  MCSymbol *Label = getContext().createTempSymbol("cfi", true);
  MCELFStreamer::emitLabel(Label);
  return Label;
}

void FgpuELFStreamer::emitCFIEndProcImpl(MCDwarfFrameInfo &Frame) {
  Frame.End = getContext().createTempSymbol();
  MCELFStreamer::emitLabel(Frame.End);
}

void FgpuELFStreamer::createPendingLabelRelocs() {
  FgpuTargetELFStreamer *ELFTargetStreamer =
      static_cast<FgpuTargetELFStreamer *>(getTargetStreamer());

  // FIXME: Also mark labels when in FGPU16 mode.
  if (ELFTargetStreamer->isMicroFgpuEnabled()) {
    for (auto *L : Labels) {
      auto *Label = cast<MCSymbolELF>(L);
      getAssembler().registerSymbol(*Label);
      Label->setOther(ELF::STO_MIPS_MICROMIPS);
    }
  }

  Labels.clear();
}

void FgpuELFStreamer::emitLabel(MCSymbol *Symbol, SMLoc Loc) {
  MCELFStreamer::emitLabel(Symbol);
  Labels.push_back(Symbol);
}

void FgpuELFStreamer::SwitchSection(MCSection *Section,
                                    const MCExpr *Subsection) {
  MCELFStreamer::SwitchSection(Section, Subsection);
  Labels.clear();
}

void FgpuELFStreamer::emitValueImpl(const MCExpr *Value, unsigned Size,
                                    SMLoc Loc) {
  MCELFStreamer::emitValueImpl(Value, Size, Loc);
  Labels.clear();
}

void FgpuELFStreamer::emitIntValue(uint64_t Value, unsigned Size) {
  MCELFStreamer::emitIntValue(Value, Size);
  Labels.clear();
}

void FgpuELFStreamer::EmitFgpuOptionRecords() {
  for (const auto &I : FgpuOptionRecords)
    I->EmitFgpuOptionRecord();
}

MCELFStreamer *llvm::createFgpuELFStreamer(
    MCContext &Context, std::unique_ptr<MCAsmBackend> MAB,
    std::unique_ptr<MCObjectWriter> OW, std::unique_ptr<MCCodeEmitter> Emitter,
    bool RelaxAll) {
  return new FgpuELFStreamer(Context, std::move(MAB), std::move(OW),
                             std::move(Emitter));
}
