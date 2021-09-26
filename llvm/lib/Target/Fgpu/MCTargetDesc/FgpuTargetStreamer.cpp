//===-- FgpuTargetStreamer.cpp - Fgpu Target Streamer Methods -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides Fgpu specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "FgpuTargetStreamer.h"
#include "FgpuInstPrinter.h"
#include "MCTargetDesc/FgpuABIInfo.h"
#include "FgpuELFStreamer.h"
#include "FgpuMCExpr.h"
#include "FgpuMCTargetDesc.h"
#include "FgpuTargetObjectFile.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"

using namespace llvm;

namespace {
static cl::opt<bool> RoundSectionSizes(
    "fgpu-round-section-sizes", cl::init(false),
    cl::desc("Round section sizes up to the section alignment"), cl::Hidden);
} // end anonymous namespace

FgpuTargetStreamer::FgpuTargetStreamer(MCStreamer &S)
    : MCTargetStreamer(S), GPReg(Fgpu::R29), ModuleDirectiveAllowed(true) {
  GPRInfoSet = FPRInfoSet = FrameInfoSet = false;
}

void FgpuTargetStreamer::emitDirectiveSetReorder() { forbidModuleDirective(); }
void FgpuTargetStreamer::emitDirectiveSetNoReorder() {}
void FgpuTargetStreamer::emitDirectiveSetMacro() { forbidModuleDirective(); }
void FgpuTargetStreamer::emitDirectiveSetNoMacro() { forbidModuleDirective(); }
//void FgpuTargetStreamer::emitDirectiveSetGINV() {}
//void FgpuTargetStreamer::emitDirectiveSetNoGINV() {}
void FgpuTargetStreamer::emitDirectiveSetAt() { forbidModuleDirective(); }
void FgpuTargetStreamer::emitDirectiveSetAtWithArg(unsigned RegNo) {
  forbidModuleDirective();
}
void FgpuTargetStreamer::emitDirectiveSetNoAt() { forbidModuleDirective(); }
void FgpuTargetStreamer::emitDirectiveEnd(StringRef Name) {}
void FgpuTargetStreamer::emitDirectiveEnt(const MCSymbol &Symbol) {}
void FgpuTargetStreamer::emitDirectiveOptionPic0() {}
void FgpuTargetStreamer::emitDirectiveOptionPic2() {}
void FgpuTargetStreamer::emitDirectiveInsn() { forbidModuleDirective(); }
void FgpuTargetStreamer::emitFrame(unsigned StackReg, unsigned StackSize,
                                   unsigned ReturnReg) {}
void FgpuTargetStreamer::emitMask(unsigned CPUBitmask, int CPUTopSavedRegOff) {}
void FgpuTargetStreamer::emitFMask(unsigned FPUBitmask, int FPUTopSavedRegOff) {
}
void FgpuTargetStreamer::emitDirectiveSetArch(StringRef Arch) {
  forbidModuleDirective();
}
void FgpuTargetStreamer::emitDirectiveSetPop() { forbidModuleDirective(); }
void FgpuTargetStreamer::emitDirectiveSetPush() { forbidModuleDirective(); }
void FgpuTargetStreamer::emitDirectiveCpAdd(unsigned RegNo) {}
void FgpuTargetStreamer::emitDirectiveCpLoad(unsigned RegNo) {}
void FgpuTargetStreamer::emitDirectiveCpLocal(unsigned RegNo) {
  // .cplocal $reg
  // This directive forces to use the alternate register for context pointer.
  // For example
  //   .cplocal $4
  //   jal foo
  // expands to
  //   ld    $25, %call16(foo)($4)
  //   jalr  $25

  GPReg = RegNo;

  forbidModuleDirective();
}
bool FgpuTargetStreamer::emitDirectiveCpRestore(
    int Offset, function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  forbidModuleDirective();
  return true;
}
void FgpuTargetStreamer::emitDirectiveCpsetup(unsigned RegNo, int RegOrOffset,
                                              const MCSymbol &Sym, bool IsReg) {
}
void FgpuTargetStreamer::emitDirectiveCpreturn(unsigned SaveLocation,
                                               bool SaveLocationIsRegister) {}

void FgpuTargetStreamer::emitR(unsigned Opcode, unsigned Reg0, SMLoc IDLoc,
                               const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.setLoc(IDLoc);
  getStreamer().emitInstruction(TmpInst, *STI);
}

void FgpuTargetStreamer::emitRX(unsigned Opcode, unsigned Reg0, MCOperand Op1,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.addOperand(Op1);
  TmpInst.setLoc(IDLoc);
  getStreamer().emitInstruction(TmpInst, *STI);
}

void FgpuTargetStreamer::emitRI(unsigned Opcode, unsigned Reg0, int32_t Imm,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  emitRX(Opcode, Reg0, MCOperand::createImm(Imm), IDLoc, STI);
}

void FgpuTargetStreamer::emitRR(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  emitRX(Opcode, Reg0, MCOperand::createReg(Reg1), IDLoc, STI);
}

void FgpuTargetStreamer::emitII(unsigned Opcode, int16_t Imm1, int16_t Imm2,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createImm(Imm1));
  TmpInst.addOperand(MCOperand::createImm(Imm2));
  TmpInst.setLoc(IDLoc);
  getStreamer().emitInstruction(TmpInst, *STI);
}

void FgpuTargetStreamer::emitRRX(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                 MCOperand Op2, SMLoc IDLoc,
                                 const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.addOperand(MCOperand::createReg(Reg1));
  TmpInst.addOperand(Op2);
  TmpInst.setLoc(IDLoc);
  getStreamer().emitInstruction(TmpInst, *STI);
}

void FgpuTargetStreamer::emitRRR(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                 unsigned Reg2, SMLoc IDLoc,
                                 const MCSubtargetInfo *STI) {
  emitRRX(Opcode, Reg0, Reg1, MCOperand::createReg(Reg2), IDLoc, STI);
}

void FgpuTargetStreamer::emitRRRX(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                  unsigned Reg2, MCOperand Op3, SMLoc IDLoc,
                                  const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.addOperand(MCOperand::createReg(Reg1));
  TmpInst.addOperand(MCOperand::createReg(Reg2));
  TmpInst.addOperand(Op3);
  TmpInst.setLoc(IDLoc);
  getStreamer().emitInstruction(TmpInst, *STI);
}

void FgpuTargetStreamer::emitRRI(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                 int16_t Imm, SMLoc IDLoc,
                                 const MCSubtargetInfo *STI) {
  emitRRX(Opcode, Reg0, Reg1, MCOperand::createImm(Imm), IDLoc, STI);
}

void FgpuTargetStreamer::emitRRIII(unsigned Opcode, unsigned Reg0,
                                   unsigned Reg1, int16_t Imm0, int16_t Imm1,
                                   int16_t Imm2, SMLoc IDLoc,
                                   const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.addOperand(MCOperand::createReg(Reg1));
  TmpInst.addOperand(MCOperand::createImm(Imm0));
  TmpInst.addOperand(MCOperand::createImm(Imm1));
  TmpInst.addOperand(MCOperand::createImm(Imm2));
  TmpInst.setLoc(IDLoc);
  getStreamer().emitInstruction(TmpInst, *STI);
}

void FgpuTargetStreamer::emitAdd(unsigned DstReg, unsigned SrcReg,
                                  unsigned TrgReg, bool Is64Bit,
                                  const MCSubtargetInfo *STI) {
  assert(!Is64Bit);
  emitRRR(Fgpu::ADD, DstReg, SrcReg, TrgReg, SMLoc(),
          STI);
}


void FgpuTargetStreamer::emitEmptyDelaySlot(bool hasShortDelaySlot, SMLoc IDLoc,
                                            const MCSubtargetInfo *STI) {
  emitRRI(Fgpu::SLL, Fgpu::ZERO, Fgpu::ZERO, 0, IDLoc, STI);
}

void FgpuTargetStreamer::emitNop(SMLoc IDLoc, const MCSubtargetInfo *STI) {
  emitRRI(Fgpu::SLL, Fgpu::ZERO, Fgpu::ZERO, 0, IDLoc, STI);
}

/// Emit the $gp restore operation for .cprestore.
void FgpuTargetStreamer::emitGPRestore(int Offset, SMLoc IDLoc,
                                       const MCSubtargetInfo *STI) {
  emitLoadWithImmOffset(Fgpu::LW, GPReg, Fgpu::SP, Offset, GPReg, IDLoc, STI);
}

/// Emit a store instruction with an immediate offset.
void FgpuTargetStreamer::emitStoreWithImmOffset(
    unsigned Opcode, unsigned SrcReg, unsigned BaseReg, int64_t Offset,
    function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  if (isInt<16>(Offset)) {
    emitRRI(Opcode, SrcReg, BaseReg, Offset, IDLoc, STI);
    return;
  }

  // sw $8, offset($8) => lui $at, %hi(offset)
  //                      add $at, $at, $8
  //                      sw $8, %lo(offset)($at)

  unsigned ATReg = GetATReg();
  if (!ATReg)
    return;

  unsigned LoOffset = Offset & 0x0000ffff;
  unsigned HiOffset = (Offset & 0xffff0000) >> 16;

  // If msb of LoOffset is 1(negative number) we must increment HiOffset
  // to account for the sign-extension of the low part.
  if (LoOffset & 0x8000)
    HiOffset++;

  // Generate the base address in ATReg.
  emitRI(Fgpu::LUi, ATReg, HiOffset, IDLoc, STI);
  if (BaseReg != Fgpu::ZERO)
    emitRRR(Fgpu::ADD, ATReg, ATReg, BaseReg, IDLoc, STI);
  // Emit the store with the adjusted base and offset.
  emitRRI(Opcode, SrcReg, ATReg, LoOffset, IDLoc, STI);
}

/// Emit a load instruction with an immediate offset. DstReg and TmpReg are
/// permitted to be the same register iff DstReg is distinct from BaseReg and
/// DstReg is a GPR. It is the callers responsibility to identify such cases
/// and pass the appropriate register in TmpReg.
void FgpuTargetStreamer::emitLoadWithImmOffset(unsigned Opcode, unsigned DstReg,
                                               unsigned BaseReg, int64_t Offset,
                                               unsigned TmpReg, SMLoc IDLoc,
                                               const MCSubtargetInfo *STI) {
  if (isInt<16>(Offset)) {
    emitRRI(Opcode, DstReg, BaseReg, Offset, IDLoc, STI);
    return;
  }

  // 1) lw $8, offset($9) => lui $8, %hi(offset)
  //                         add $8, $8, $9
  //                         lw $8, %lo(offset)($9)
  // 2) lw $8, offset($8) => lui $at, %hi(offset)
  //                         add $at, $at, $8
  //                         lw $8, %lo(offset)($at)

  unsigned LoOffset = Offset & 0x0000ffff;
  unsigned HiOffset = (Offset & 0xffff0000) >> 16;

  // If msb of LoOffset is 1(negative number) we must increment HiOffset
  // to account for the sign-extension of the low part.
  if (LoOffset & 0x8000)
    HiOffset++;

  // Generate the base address in TmpReg.
  emitRI(Fgpu::LUi, TmpReg, HiOffset, IDLoc, STI);
  if (BaseReg != Fgpu::ZERO)
    emitRRR(Fgpu::ADD, TmpReg, TmpReg, BaseReg, IDLoc, STI);
  // Emit the load with the adjusted base and offset.
  emitRRI(Opcode, DstReg, TmpReg, LoOffset, IDLoc, STI);
}

FgpuTargetAsmStreamer::FgpuTargetAsmStreamer(MCStreamer &S,
                                             formatted_raw_ostream &OS)
    : FgpuTargetStreamer(S), OS(OS) {}

void FgpuTargetAsmStreamer::emitDirectiveSetReorder() {
  OS << "\t.set\treorder\n";
  FgpuTargetStreamer::emitDirectiveSetReorder();
}

void FgpuTargetAsmStreamer::emitDirectiveSetNoReorder() {
  OS << "\t.set\tnoreorder\n";
  forbidModuleDirective();
}

void FgpuTargetAsmStreamer::emitDirectiveSetMacro() {
  OS << "\t.set\tmacro\n";
  FgpuTargetStreamer::emitDirectiveSetMacro();
}

void FgpuTargetAsmStreamer::emitDirectiveSetNoMacro() {
  OS << "\t.set\tnomacro\n";
  FgpuTargetStreamer::emitDirectiveSetNoMacro();
}

void FgpuTargetAsmStreamer::emitDirectiveSetAt() {
  OS << "\t.set\tat\n";
  FgpuTargetStreamer::emitDirectiveSetAt();
}

void FgpuTargetAsmStreamer::emitDirectiveSetAtWithArg(unsigned RegNo) {
  OS << "\t.set\tat=$" << Twine(RegNo) << "\n";
  FgpuTargetStreamer::emitDirectiveSetAtWithArg(RegNo);
}

void FgpuTargetAsmStreamer::emitDirectiveSetNoAt() {
  OS << "\t.set\tnoat\n";
  FgpuTargetStreamer::emitDirectiveSetNoAt();
}

void FgpuTargetAsmStreamer::emitDirectiveEnd(StringRef Name) {
  OS << "\t.end\t" << Name << '\n';
}

void FgpuTargetAsmStreamer::emitDirectiveEnt(const MCSymbol &Symbol) {
  OS << "\t.ent\t" << Symbol.getName() << '\n';
}

void FgpuTargetAsmStreamer::emitDirectiveOptionPic0() {
  OS << "\t.option\tpic0\n";
}

void FgpuTargetAsmStreamer::emitDirectiveOptionPic2() {
  OS << "\t.option\tpic2\n";
}

void FgpuTargetAsmStreamer::emitDirectiveInsn() {
  FgpuTargetStreamer::emitDirectiveInsn();
  OS << "\t.insn\n";
}

void FgpuTargetAsmStreamer::emitFrame(unsigned StackReg, unsigned StackSize,
                                      unsigned ReturnReg) {
  OS << "\t.frame\t$"
     << StringRef(FgpuInstPrinter::getRegisterName(StackReg)).lower() << ","
     << StackSize << ",$"
     << StringRef(FgpuInstPrinter::getRegisterName(ReturnReg)).lower() << '\n';
}

void FgpuTargetAsmStreamer::emitDirectiveSetArch(StringRef Arch) {
  OS << "\t.set arch=" << Arch << "\n";
  FgpuTargetStreamer::emitDirectiveSetArch(Arch);
}

void FgpuTargetAsmStreamer::emitDirectiveSetPop() {
  OS << "\t.set\tpop\n";
  FgpuTargetStreamer::emitDirectiveSetPop();
}

void FgpuTargetAsmStreamer::emitDirectiveSetPush() {
 OS << "\t.set\tpush\n";
 FgpuTargetStreamer::emitDirectiveSetPush();
}

// Print a 32 bit hex number with all numbers.
static void printHex32(unsigned Value, raw_ostream &OS) {
  OS << "0x";
  for (int i = 7; i >= 0; i--)
    OS.write_hex((Value & (0xF << (i * 4))) >> (i * 4));
}

void FgpuTargetAsmStreamer::emitMask(unsigned CPUBitmask,
                                     int CPUTopSavedRegOff) {
  OS << "\t.mask \t";
  printHex32(CPUBitmask, OS);
  OS << ',' << CPUTopSavedRegOff << '\n';
}

void FgpuTargetAsmStreamer::emitFMask(unsigned FPUBitmask,
                                      int FPUTopSavedRegOff) {
  OS << "\t.fmask\t";
  printHex32(FPUBitmask, OS);
  OS << "," << FPUTopSavedRegOff << '\n';
}

void FgpuTargetAsmStreamer::emitDirectiveCpAdd(unsigned RegNo) {
  OS << "\t.cpadd\t$"
     << StringRef(FgpuInstPrinter::getRegisterName(RegNo)).lower() << "\n";
  forbidModuleDirective();
}

void FgpuTargetAsmStreamer::emitDirectiveCpLoad(unsigned RegNo) {
  OS << "\t.cpload\t$"
     << StringRef(FgpuInstPrinter::getRegisterName(RegNo)).lower() << "\n";
  forbidModuleDirective();
}

void FgpuTargetAsmStreamer::emitDirectiveCpLocal(unsigned RegNo) {
  OS << "\t.cplocal\t$"
     << StringRef(FgpuInstPrinter::getRegisterName(RegNo)).lower() << "\n";
  FgpuTargetStreamer::emitDirectiveCpLocal(RegNo);
}

bool FgpuTargetAsmStreamer::emitDirectiveCpRestore(
    int Offset, function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  FgpuTargetStreamer::emitDirectiveCpRestore(Offset, GetATReg, IDLoc, STI);
  OS << "\t.cprestore\t" << Offset << "\n";
  return true;
}

void FgpuTargetAsmStreamer::emitDirectiveCpsetup(unsigned RegNo,
                                                 int RegOrOffset,
                                                 const MCSymbol &Sym,
                                                 bool IsReg) {
  OS << "\t.cpsetup\t$"
     << StringRef(FgpuInstPrinter::getRegisterName(RegNo)).lower() << ", ";

  if (IsReg)
    OS << "$"
       << StringRef(FgpuInstPrinter::getRegisterName(RegOrOffset)).lower();
  else
    OS << RegOrOffset;

  OS << ", ";

  OS << Sym.getName();
  forbidModuleDirective();
}

void FgpuTargetAsmStreamer::emitDirectiveCpreturn(unsigned SaveLocation,
                                                  bool SaveLocationIsRegister) {
  OS << "\t.cpreturn";
  forbidModuleDirective();
}

// This part is for ELF object output.
FgpuTargetELFStreamer::FgpuTargetELFStreamer(MCStreamer &S,
                                             const MCSubtargetInfo &STI)
    : FgpuTargetStreamer(S), STI(STI) {
  MCAssembler &MCA = getStreamer().getAssembler();

  // It's possible that MCObjectFileInfo isn't fully initialized at this point
  // due to an initialization order problem where LLVMTargetMachine creates the
  // target streamer before TargetLoweringObjectFile calls
  // InitializeMCObjectFileInfo. There doesn't seem to be a single place that
  // covers all cases so this statement covers most cases and direct object
  // emission must call setPic() once MCObjectFileInfo has been initialized. The
  // cases we don't handle here are covered by FgpuAsmPrinter.
  Pic = MCA.getContext().getObjectFileInfo()->isPositionIndependent();

  const FeatureBitset &Features = STI.getFeatureBits();

  // Set the header flags that we can in the constructor.
  // FIXME: This is a fairly terrible hack. We set the rest
  // of these in the destructor. The problem here is two-fold:
  //
  // a: Some of the eflags can be set/reset by directives.
  // b: There aren't any usage paths that initialize the ABI
  //    pointer until after we initialize either an assembler
  //    or the target machine.
  // We can fix this by making the target streamer construct
  // the ABI, but this is fraught with wide ranging dependency
  // issues as well.
  unsigned EFlags = MCA.getELFHeaderEFlags();

  // FIXME: Fix a dependency issue by instantiating the ABI object to some
  // default based off the triple. The triple doesn't describe the target
  // fully, but any external user of the API that uses the MCTargetStreamer
  // would otherwise crash on assertion failure.

  ABI = FgpuABIInfo(FgpuABIInfo::CC_Fgpu());

    EFlags |= ELF::EF_FGPU_ARCH_1;


  MCA.setELFHeaderEFlags(EFlags);
}

void FgpuTargetELFStreamer::emitLabel(MCSymbol *S) {
  auto *Symbol = cast<MCSymbolELF>(S);
  getStreamer().getAssembler().registerSymbol(*Symbol);
  uint8_t Type = Symbol->getType();
  if (Type != ELF::STT_FUNC)
    return;

}

void FgpuTargetELFStreamer::finish() {
  MCAssembler &MCA = getStreamer().getAssembler();
  const MCObjectFileInfo &OFI = *MCA.getContext().getObjectFileInfo();

  // .bss, .text and .data are always at least 16-byte aligned.
  MCSection &TextSection = *OFI.getTextSection();
  MCA.registerSection(TextSection);
  MCSection &DataSection = *OFI.getDataSection();
  MCA.registerSection(DataSection);
  MCSection &BSSSection = *OFI.getBSSSection();
  MCA.registerSection(BSSSection);

  TextSection.setAlignment(Align(std::max(16u, TextSection.getAlignment())));
  DataSection.setAlignment(Align(std::max(16u, DataSection.getAlignment())));
  BSSSection.setAlignment(Align(std::max(16u, BSSSection.getAlignment())));

  if (RoundSectionSizes) {
    // Make sections sizes a multiple of the alignment. This is useful for
    // verifying the output of IAS against the output of other assemblers but
    // it's not necessary to produce a correct object and increases section
    // size.
    MCStreamer &OS = getStreamer();
    for (MCSection &S : MCA) {
      MCSectionELF &Section = static_cast<MCSectionELF &>(S);

      unsigned Alignment = Section.getAlignment();
      if (Alignment) {
        OS.SwitchSection(&Section);
        if (Section.UseCodeAlign())
          OS.emitCodeAlignment(Alignment, Alignment);
        else
          OS.emitValueToAlignment(Alignment, 0, 1, Alignment);
      }
    }
  }

  const FeatureBitset &Features = STI.getFeatureBits();

  // Update e_header flags. See the FIXME and comment above in
  // the constructor for a full rundown on this.
  unsigned EFlags = MCA.getELFHeaderEFlags();

  if (Pic)
    EFlags |= ELF::EF_FGPU_PIC | ELF::EF_FGPU_CPIC;

  MCA.setELFHeaderEFlags(EFlags);

  // Emit all the option records.
  // At the moment we are only emitting .Fgpu.options (ODK_REGINFO) and
  // .reginfo.
  FgpuELFStreamer &MEF = static_cast<FgpuELFStreamer &>(Streamer);
  MEF.EmitFgpuOptionRecords();

  emitFgpuAbiFlags();
}

void FgpuTargetELFStreamer::emitAssignment(MCSymbol *S, const MCExpr *Value) {
  auto *Symbol = cast<MCSymbolELF>(S);
  // If on rhs is microfgpu symbol then mark Symbol as microFgpu.
  if (Value->getKind() != MCExpr::SymbolRef)
    return;
  const auto &RhsSym = cast<MCSymbolELF>(
      static_cast<const MCSymbolRefExpr *>(Value)->getSymbol());

  if (!(RhsSym.getOther() & ELF::STO_FGPU_MICROFGPU))
    return;

  Symbol->setOther(ELF::STO_FGPU_MICROFGPU);
}

MCELFStreamer &FgpuTargetELFStreamer::getStreamer() {
  return static_cast<MCELFStreamer &>(Streamer);
}

void FgpuTargetELFStreamer::emitDirectiveSetNoReorder() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_FGPU_NOREORDER;
  MCA.setELFHeaderEFlags(Flags);
  forbidModuleDirective();
}

void FgpuTargetELFStreamer::emitDirectiveEnd(StringRef Name) {
  MCAssembler &MCA = getStreamer().getAssembler();
  MCContext &Context = MCA.getContext();
  MCStreamer &OS = getStreamer();

  MCSectionELF *Sec = Context.getELFSection(".pdr", ELF::SHT_PROGBITS, 0);

  MCSymbol *Sym = Context.getOrCreateSymbol(Name);
  const MCSymbolRefExpr *ExprRef =
      MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, Context);

  MCA.registerSection(*Sec);
  Sec->setAlignment(Align(4));

  OS.PushSection();

  OS.SwitchSection(Sec);

  OS.emitValueImpl(ExprRef, 4);

  OS.emitIntValue(GPRInfoSet ? GPRBitMask : 0, 4); // reg_mask
  OS.emitIntValue(GPRInfoSet ? GPROffset : 0, 4);  // reg_offset

  OS.emitIntValue(FPRInfoSet ? FPRBitMask : 0, 4); // fpreg_mask
  OS.emitIntValue(FPRInfoSet ? FPROffset : 0, 4);  // fpreg_offset

  OS.emitIntValue(FrameInfoSet ? FrameOffset : 0, 4); // frame_offset
  OS.emitIntValue(FrameInfoSet ? FrameReg : 0, 4);    // frame_reg
  OS.emitIntValue(FrameInfoSet ? ReturnReg : 0, 4);   // return_reg

  // The .end directive marks the end of a procedure. Invalidate
  // the information gathered up until this point.
  GPRInfoSet = FPRInfoSet = FrameInfoSet = false;

  OS.PopSection();

  // .end also implicitly sets the size.
  MCSymbol *CurPCSym = Context.createTempSymbol();
  OS.emitLabel(CurPCSym);
  const MCExpr *Size = MCBinaryExpr::createSub(
      MCSymbolRefExpr::create(CurPCSym, MCSymbolRefExpr::VK_None, Context),
      ExprRef, Context);

  // The ELFObjectWriter can determine the absolute size as it has access to
  // the layout information of the assembly file, so a size expression rather
  // than an absolute value is ok here.
  static_cast<MCSymbolELF *>(Sym)->setSize(Size);
}

void FgpuTargetELFStreamer::emitDirectiveEnt(const MCSymbol &Symbol) {
  GPRInfoSet = FPRInfoSet = FrameInfoSet = false;

  // .ent also acts like an implicit '.type symbol, STT_FUNC'
  static_cast<const MCSymbolELF &>(Symbol).setType(ELF::STT_FUNC);
}

void FgpuTargetELFStreamer::emitDirectiveOptionPic0() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  // This option overrides other PIC options like -KPIC.
  Pic = false;
  Flags &= ~ELF::EF_FGPU_PIC;
  MCA.setELFHeaderEFlags(Flags);
}

void FgpuTargetELFStreamer::emitDirectiveOptionPic2() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Pic = true;
  // NOTE: We are following the GAS behaviour here which means the directive
  // 'pic2' also sets the CPIC bit in the ELF header. This is different from
  // what is stated in the SYSV ABI which consider the bits EF_FGPU_PIC and
  // EF_FGPU_CPIC to be mutually exclusive.
  Flags |= ELF::EF_FGPU_PIC | ELF::EF_FGPU_CPIC;
  MCA.setELFHeaderEFlags(Flags);
}

void FgpuTargetELFStreamer::emitDirectiveInsn() {
  FgpuTargetStreamer::emitDirectiveInsn();
  FgpuELFStreamer &MEF = static_cast<FgpuELFStreamer &>(Streamer);
  MEF.createPendingLabelRelocs();
}

void FgpuTargetELFStreamer::emitFrame(unsigned StackReg, unsigned StackSize,
                                      unsigned ReturnReg_) {
  MCContext &Context = getStreamer().getAssembler().getContext();
  const MCRegisterInfo *RegInfo = Context.getRegisterInfo();

  FrameInfoSet = true;
  FrameReg = RegInfo->getEncodingValue(StackReg);
  FrameOffset = StackSize;
  ReturnReg = RegInfo->getEncodingValue(ReturnReg_);
}

void FgpuTargetELFStreamer::emitMask(unsigned CPUBitmask,
                                     int CPUTopSavedRegOff) {
  GPRInfoSet = true;
  GPRBitMask = CPUBitmask;
  GPROffset = CPUTopSavedRegOff;
}

void FgpuTargetELFStreamer::emitFMask(unsigned FPUBitmask,
                                      int FPUTopSavedRegOff) {
  FPRInfoSet = true;
  FPRBitMask = FPUBitmask;
  FPROffset = FPUTopSavedRegOff;
}

void FgpuTargetELFStreamer::emitDirectiveCpAdd(unsigned RegNo) {
  // .cpadd $reg
  // This directive inserts code to add $gp to the argument's register
  // when support for position independent code is enabled.
  if (!Pic)
    return;

  emitAdd(RegNo, RegNo, GPReg, false, &STI);
  forbidModuleDirective();
}

void FgpuTargetELFStreamer::emitDirectiveCpLoad(unsigned RegNo) {
  // .cpload $reg
  // This directive expands to:
  // lui   $gp, %hi(_gp_disp)
  // addui $gp, $gp, %lo(_gp_disp)
  // addu  $gp, $gp, $reg
  // when support for position independent code is enabled.
  if (!Pic)
    return;

  // There's a GNU extension controlled by -mno-shared that allows
  // locally-binding symbols to be accessed using absolute addresses.
  // This is currently not supported. When supported -mno-shared makes
  // .cpload expand to:
  //   lui     $gp, %hi(__gnu_local_gp)
  //   addiu   $gp, $gp, %lo(__gnu_local_gp)

  StringRef SymName("_gp_disp");
  MCAssembler &MCA = getStreamer().getAssembler();
  MCSymbol *GP_Disp = MCA.getContext().getOrCreateSymbol(SymName);
  MCA.registerSymbol(*GP_Disp);

  MCInst TmpInst;
  TmpInst.setOpcode(Fgpu::LUi);
  TmpInst.addOperand(MCOperand::createReg(GPReg));
  const MCExpr *HiSym = FgpuMCExpr::create(
      FgpuMCExpr::MEK_HI,
      MCSymbolRefExpr::create("_gp_disp", MCSymbolRefExpr::VK_None,
                              MCA.getContext()),
      MCA.getContext());
  TmpInst.addOperand(MCOperand::createExpr(HiSym));
  getStreamer().emitInstruction(TmpInst, STI);

  TmpInst.clear();

  TmpInst.setOpcode(Fgpu::ADDi);
  TmpInst.addOperand(MCOperand::createReg(GPReg));
  TmpInst.addOperand(MCOperand::createReg(GPReg));
  const MCExpr *LoSym = FgpuMCExpr::create(
      FgpuMCExpr::MEK_LO,
      MCSymbolRefExpr::create("_gp_disp", MCSymbolRefExpr::VK_None,
                              MCA.getContext()),
      MCA.getContext());
  TmpInst.addOperand(MCOperand::createExpr(LoSym));
  getStreamer().emitInstruction(TmpInst, STI);

  TmpInst.clear();

  TmpInst.setOpcode(Fgpu::ADD);
  TmpInst.addOperand(MCOperand::createReg(GPReg));
  TmpInst.addOperand(MCOperand::createReg(GPReg));
  TmpInst.addOperand(MCOperand::createReg(RegNo));
  getStreamer().emitInstruction(TmpInst, STI);

  forbidModuleDirective();
}

void FgpuTargetELFStreamer::emitDirectiveCpLocal(unsigned RegNo) {
  if (Pic)
    FgpuTargetStreamer::emitDirectiveCpLocal(RegNo);
}

bool FgpuTargetELFStreamer::emitDirectiveCpRestore(
    int Offset, function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  FgpuTargetStreamer::emitDirectiveCpRestore(Offset, GetATReg, IDLoc, STI);
  // .cprestore offset
  // When PIC mode is enabled and the O32 ABI is used, this directive expands
  // to:
  //    sw $gp, offset($sp)
  // and adds a corresponding LW after every JAL.

  // Note that .cprestore is ignored if used with the N32 and N64 ABIs or if it
  // is used in non-PIC mode.
  if (!Pic)
    return true;

  // Store the $gp on the stack.
  emitStoreWithImmOffset(Fgpu::SW, GPReg, Fgpu::SP, Offset, GetATReg, IDLoc,
                         STI);
  return true;
}

void FgpuTargetELFStreamer::emitDirectiveCpsetup(unsigned RegNo,
                                                 int RegOrOffset,
                                                 const MCSymbol &Sym,
                                                 bool IsReg) {
  // Only N32 and N64 emit anything for .cpsetup iff PIC is set.

  //if (!Pic || !(getABI().IsN32() || getABI().IsN64()))
    return;
//
//  forbidModuleDirective();
//
//  MCAssembler &MCA = getStreamer().getAssembler();
//  MCInst Inst;
//
//  // Either store the old $gp in a register or on the stack
//  if (IsReg) {
//    // move $save, $gpreg
//    emitRRR(Fgpu::OR64, RegOrOffset, GPReg, Fgpu::ZERO, SMLoc(), &STI);
//  } else {
//    // sd $gpreg, offset($sp)
//    emitRRI(Fgpu::SD, GPReg, Fgpu::SP, RegOrOffset, SMLoc(), &STI);
//  }
//
//  if (getABI().IsN32()) {
//    MCSymbol *GPSym = MCA.getContext().getOrCreateSymbol("__gnu_local_gp");
//    const FgpuMCExpr *HiExpr = FgpuMCExpr::create(
//        FgpuMCExpr::MEK_HI, MCSymbolRefExpr::create(GPSym, MCA.getContext()),
//        MCA.getContext());
//    const FgpuMCExpr *LoExpr = FgpuMCExpr::create(
//        FgpuMCExpr::MEK_LO, MCSymbolRefExpr::create(GPSym, MCA.getContext()),
//        MCA.getContext());
//
//    // lui $gp, %hi(__gnu_local_gp)
//    emitRX(Fgpu::LUi, GPReg, MCOperand::createExpr(HiExpr), SMLoc(), &STI);
//
//    // addiu  $gp, $gp, %lo(__gnu_local_gp)
//    emitRRX(Fgpu::ADDiu, GPReg, GPReg, MCOperand::createExpr(LoExpr), SMLoc(),
//            &STI);
//
//    return;
//  }
//
//  const FgpuMCExpr *HiExpr = FgpuMCExpr::createGpOff(
//      FgpuMCExpr::MEK_HI, MCSymbolRefExpr::create(&Sym, MCA.getContext()),
//      MCA.getContext());
//  const FgpuMCExpr *LoExpr = FgpuMCExpr::createGpOff(
//      FgpuMCExpr::MEK_LO, MCSymbolRefExpr::create(&Sym, MCA.getContext()),
//      MCA.getContext());
//
//  // lui $gp, %hi(%neg(%gp_rel(funcSym)))
//  emitRX(Fgpu::LUi, GPReg, MCOperand::createExpr(HiExpr), SMLoc(), &STI);
//
//  // addiu  $gp, $gp, %lo(%neg(%gp_rel(funcSym)))
//  emitRRX(Fgpu::ADDiu, GPReg, GPReg, MCOperand::createExpr(LoExpr), SMLoc(),
//          &STI);
//
//  // daddu  $gp, $gp, $funcreg
//  emitRRR(Fgpu::DADDu, GPReg, GPReg, RegNo, SMLoc(), &STI);
}

void FgpuTargetELFStreamer::emitDirectiveCpreturn(unsigned SaveLocation,
                                                  bool SaveLocationIsRegister) {
  // Only N32 and N64 emit anything for .cpreturn iff PIC is set.
//  if (!Pic || !(getABI().IsN32() || getABI().IsN64()))
    return;

//  MCInst Inst;
//  // Either restore the old $gp from a register or on the stack
//  if (SaveLocationIsRegister) {
//    Inst.setOpcode(Fgpu::OR);
//    Inst.addOperand(MCOperand::createReg(GPReg));
//    Inst.addOperand(MCOperand::createReg(SaveLocation));
//    Inst.addOperand(MCOperand::createReg(Fgpu::ZERO));
//  } else {
//    Inst.setOpcode(Fgpu::LD);
//    Inst.addOperand(MCOperand::createReg(GPReg));
//    Inst.addOperand(MCOperand::createReg(Fgpu::SP));
//    Inst.addOperand(MCOperand::createImm(SaveLocation));
//  }
//  getStreamer().emitInstruction(Inst, STI);
//
//  forbidModuleDirective();
}

void FgpuTargetELFStreamer::emitFgpuAbiFlags() {
  MCAssembler &MCA = getStreamer().getAssembler();
  MCContext &Context = MCA.getContext();
  MCStreamer &OS = getStreamer();
  MCSectionELF *Sec = Context.getELFSection(
      ".FGPU.abiflags", ELF::SHT_MIPS_ABIFLAGS, ELF::SHF_ALLOC, 24);
  MCA.registerSection(*Sec);
  Sec->setAlignment(Align(8));
  OS.SwitchSection(Sec);

  OS << ABIFlagsSection;
}
