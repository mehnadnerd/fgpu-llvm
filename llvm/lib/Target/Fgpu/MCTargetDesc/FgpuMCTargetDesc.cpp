//===-- FgpuMCTargetDesc.cpp - Fgpu Target Descriptions -------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides Fgpu specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "FgpuMCTargetDesc.h"
#include "FgpuAsmBackend.h"
#include "FgpuBaseInfo.h"
#include "FgpuELFStreamer.h"
#include "FgpuInstPrinter.h"
#include "FgpuMCAsmInfo.h"
#include "FgpuMCNaCl.h"
#include "FgpuTargetStreamer.h"
#include "TargetInfo/FgpuTargetInfo.h"
#include "llvm/ADT/Triple.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstrAnalysis.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MachineLocation.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define GET_INSTRINFO_MC_DESC
#include "FgpuGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "FgpuGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "FgpuGenRegisterInfo.inc"

/// Select the Fgpu CPU for the given triple and cpu name.
StringRef FGPU_MC::selectFgpuCPU(const Triple &TT, StringRef CPU) {
  if (CPU.empty() || CPU == "generic") {
    if (TT.isFGPU32())
      CPU = "fgpu32";
    else
      CPU = "fgpu64";
  }
  return CPU;
}

static MCInstrInfo *createFgpuMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitFgpuMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createFgpuMCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitFgpuMCRegisterInfo(X, Fgpu::RA);
  return X;
}

static MCSubtargetInfo *createFgpuMCSubtargetInfo(const Triple &TT,
                                                  StringRef CPU, StringRef FS) {
  CPU = FGPU_MC::selectFgpuCPU(TT, CPU);
  return createFgpuMCSubtargetInfoImpl(TT, CPU, /*TuneCPU*/ CPU, FS);
}

static MCAsmInfo *createFgpuMCAsmInfo(const MCRegisterInfo &MRI,
                                      const Triple &TT,
                                      const MCTargetOptions &Options) {
  MCAsmInfo *MAI = new FgpuMCAsmInfo(TT, Options);

  unsigned SP = MRI.getDwarfRegNum(Fgpu::SP, true);
  MCCFIInstruction Inst = MCCFIInstruction::createDefCfaRegister(nullptr, SP);
  MAI->addInitialFrameState(Inst);

  return MAI;
}

static MCInstPrinter *createFgpuMCInstPrinter(const Triple &T,
                                              unsigned SyntaxVariant,
                                              const MCAsmInfo &MAI,
                                              const MCInstrInfo &MII,
                                              const MCRegisterInfo &MRI) {
  return new FgpuInstPrinter(MAI, MII, MRI);
}

static MCStreamer *createMCStreamer(const Triple &T, MCContext &Context,
                                    std::unique_ptr<MCAsmBackend> &&MAB,
                                    std::unique_ptr<MCObjectWriter> &&OW,
                                    std::unique_ptr<MCCodeEmitter> &&Emitter,
                                    bool RelaxAll) {
  MCStreamer *S;
  if (!T.isOSNaCl())
    S = createFgpuELFStreamer(Context, std::move(MAB), std::move(OW),
                              std::move(Emitter), RelaxAll);
  else
    S = createFgpuNaClELFStreamer(Context, std::move(MAB), std::move(OW),
                                  std::move(Emitter), RelaxAll);
  return S;
}

static MCTargetStreamer *createFgpuAsmTargetStreamer(MCStreamer &S,
                                                     formatted_raw_ostream &OS,
                                                     MCInstPrinter *InstPrint,
                                                     bool isVerboseAsm) {
  return new FgpuTargetAsmStreamer(S, OS);
}

static MCTargetStreamer *createFgpuNullTargetStreamer(MCStreamer &S) {
  return new FgpuTargetStreamer(S);
}

static MCTargetStreamer *
createFgpuObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  return new FgpuTargetELFStreamer(S, STI);
}

namespace {

class FgpuMCInstrAnalysis : public MCInstrAnalysis {
public:
  FgpuMCInstrAnalysis(const MCInstrInfo *Info) : MCInstrAnalysis(Info) {}

  bool evaluateBranch(const MCInst &Inst, uint64_t Addr, uint64_t Size,
                      uint64_t &Target) const override {
    unsigned NumOps = Inst.getNumOperands();
    if (NumOps == 0)
      return false;
    switch (Info->get(Inst.getOpcode()).OpInfo[NumOps - 1].OperandType) {
    case MCOI::OPERAND_UNKNOWN:
    case MCOI::OPERAND_IMMEDIATE: {
      // j, jal, jalx, jals
      // Absolute branch within the current 256 MB-aligned region
      uint64_t Region = Addr & ~uint64_t(0xfffffff);
      Target = Region + Inst.getOperand(NumOps - 1).getImm();
      return true;
    }
    case MCOI::OPERAND_PCREL:
      // b, beq ...
      Target = Addr + Inst.getOperand(NumOps - 1).getImm();
      return true;
    default:
      return false;
    }
  }
};
}

static MCInstrAnalysis *createFgpuMCInstrAnalysis(const MCInstrInfo *Info) {
  return new FgpuMCInstrAnalysis(Info);
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeFgpuTargetMC() {
  for (Target *T : {&getTheFgpuTarget(), &getTheFgpuelTarget(),
                    &getTheFgpu64Target(), &getTheFgpu64elTarget()}) {
    // Register the MC asm info.
    RegisterMCAsmInfoFn X(*T, createFgpuMCAsmInfo);

    // Register the MC instruction info.
    TargetRegistry::RegisterMCInstrInfo(*T, createFgpuMCInstrInfo);

    // Register the MC register info.
    TargetRegistry::RegisterMCRegInfo(*T, createFgpuMCRegisterInfo);

    // Register the elf streamer.
    TargetRegistry::RegisterELFStreamer(*T, createMCStreamer);

    // Register the asm target streamer.
    TargetRegistry::RegisterAsmTargetStreamer(*T, createFgpuAsmTargetStreamer);

    TargetRegistry::RegisterNullTargetStreamer(*T,
                                               createFgpuNullTargetStreamer);

    // Register the MC subtarget info.
    TargetRegistry::RegisterMCSubtargetInfo(*T, createFgpuMCSubtargetInfo);

    // Register the MC instruction analyzer.
    TargetRegistry::RegisterMCInstrAnalysis(*T, createFgpuMCInstrAnalysis);

    // Register the MCInstPrinter.
    TargetRegistry::RegisterMCInstPrinter(*T, createFgpuMCInstPrinter);

    TargetRegistry::RegisterObjectTargetStreamer(
        *T, createFgpuObjectTargetStreamer);

    // Register the asm backend.
    TargetRegistry::RegisterMCAsmBackend(*T, createFgpuAsmBackend);
  }

  // Register the MC Code Emitter
  for (Target *T : {&getTheFgpuTarget(), &getTheFgpu64Target()})
    TargetRegistry::RegisterMCCodeEmitter(*T, createFgpuMCCodeEmitterEB);

  for (Target *T : {&getTheFgpuelTarget(), &getTheFgpu64elTarget()})
    TargetRegistry::RegisterMCCodeEmitter(*T, createFgpuMCCodeEmitterEL);
}
