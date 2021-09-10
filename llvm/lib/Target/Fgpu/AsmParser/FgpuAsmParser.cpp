//===-- FgpuAsmParser.cpp - Parse Fgpu assembly to MCInst instructions ----===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/FgpuABIFlagsSection.h"
#include "MCTargetDesc/FgpuABIInfo.h"
#include "MCTargetDesc/FgpuBaseInfo.h"
#include "MCTargetDesc/FgpuMCExpr.h"
#include "MCTargetDesc/FgpuMCTargetDesc.h"
#include "FgpuTargetStreamer.h"
#include "TargetInfo/FgpuTargetInfo.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/ADT/Triple.h"
#include "llvm/ADT/Twine.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCAsmParser.h"
#include "llvm/MC/MCParser/MCAsmParserExtension.h"
#include "llvm/MC/MCParser/MCAsmParserUtils.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/MC/MCValue.h"
#include "llvm/MC/SubtargetFeature.h"
#include "llvm/Support/Alignment.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/SMLoc.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

using namespace llvm;

#define DEBUG_TYPE "fgpu-asm-parser"

namespace llvm {

class MCInstrInfo;

} // end namespace llvm

extern cl::opt<bool> EmitJalrReloc;

namespace {

class FgpuAssemblerOptions {
public:
  FgpuAssemblerOptions(const FeatureBitset &Features_) : Features(Features_) {}

  FgpuAssemblerOptions(const FgpuAssemblerOptions *Opts) {
    ATReg = Opts->getATRegIndex();
    Reorder = Opts->isReorder();
    Macro = Opts->isMacro();
    Features = Opts->getFeatures();
  }

  unsigned getATRegIndex() const { return ATReg; }
  bool setATRegIndex(unsigned Reg) {
    if (Reg > 31)
      return false;

    ATReg = Reg;
    return true;
  }

  bool isReorder() const { return Reorder; }
  void setReorder() { Reorder = true; }
  void setNoReorder() { Reorder = false; }

  bool isMacro() const { return Macro; }
  void setMacro() { Macro = true; }
  void setNoMacro() { Macro = false; }

  const FeatureBitset &getFeatures() const { return Features; }
  void setFeatures(const FeatureBitset &Features_) { Features = Features_; }

  // Set of features that are either architecture features or referenced
  // by them (e.g.: FeatureNaN2008 implied by FeatureFgpu32r6).
  // The full table can be found in FgpuGenSubtargetInfo.inc (FgpuFeatureKV[]).
  // The reason we need this mask is explained in the selectArch function.
  // FIXME: Ideally we would like TableGen to generate this information.
  static const FeatureBitset AllArchRelatedMask;

private:
  unsigned ATReg = 1;
  bool Reorder = true;
  bool Macro = true;
  FeatureBitset Features;
};

} // end anonymous namespace

const FeatureBitset FgpuAssemblerOptions::AllArchRelatedMask = {
    Fgpu::FeatureFgpu1, Fgpu::FeatureFgpu2, Fgpu::FeatureFgpu3,
    Fgpu::FeatureFgpu3_32, Fgpu::FeatureFgpu3_32r2, Fgpu::FeatureFgpu4,
    Fgpu::FeatureFgpu4_32, Fgpu::FeatureFgpu4_32r2, Fgpu::FeatureFgpu5,
    Fgpu::FeatureFgpu5_32r2, Fgpu::FeatureFgpu32, Fgpu::FeatureFgpu32r2,
    Fgpu::FeatureFgpu32r3, Fgpu::FeatureFgpu32r5, Fgpu::FeatureFgpu32r6,
    Fgpu::FeatureFgpu64, Fgpu::FeatureFgpu64r2, Fgpu::FeatureFgpu64r3,
    Fgpu::FeatureFgpu64r5, Fgpu::FeatureFgpu64r6, Fgpu::FeatureCnFgpu,
    Fgpu::FeatureCnFgpuP, Fgpu::FeatureFP64Bit, Fgpu::FeatureGP64Bit,
    Fgpu::FeatureNaN2008
};

namespace {

class FgpuAsmParser : public MCTargetAsmParser {
  FgpuTargetStreamer &getTargetStreamer() {
    assert(getParser().getStreamer().getTargetStreamer() &&
           "do not have a target streamer");
    MCTargetStreamer &TS = *getParser().getStreamer().getTargetStreamer();
    return static_cast<FgpuTargetStreamer &>(TS);
  }

  FgpuABIInfo ABI;
  SmallVector<std::unique_ptr<FgpuAssemblerOptions>, 2> AssemblerOptions;
  MCSymbol *CurrentFn; // Pointer to the function being parsed. It may be a
                       // nullptr, which indicates that no function is currently
                       // selected. This usually happens after an '.end func'
                       // directive.
  bool IsLittleEndian;
  bool IsPicEnabled;
  bool IsCpRestoreSet;
  int CpRestoreOffset;
  unsigned GPReg;
  unsigned CpSaveLocation;
  /// If true, then CpSaveLocation is a register, otherwise it's an offset.
  bool     CpSaveLocationIsRegister;

  // Map of register aliases created via the .set directive.
  StringMap<AsmToken> RegisterSets;

  // Print a warning along with its fix-it message at the given range.
  void printWarningWithFixIt(const Twine &Msg, const Twine &FixMsg,
                             SMRange Range, bool ShowColors = true);

  void ConvertXWPOperands(MCInst &Inst, const OperandVector &Operands);

#define GET_ASSEMBLER_HEADER
#include "FgpuGenAsmMatcher.inc"

  unsigned
  checkEarlyTargetMatchPredicate(MCInst &Inst,
                                 const OperandVector &Operands) override;
  unsigned checkTargetMatchPredicate(MCInst &Inst) override;

  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  /// Parse a register as used in CFI directives
  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc) override;
  OperandMatchResultTy tryParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                        SMLoc &EndLoc) override;

  bool parseParenSuffix(StringRef Name, OperandVector &Operands);

  bool parseBracketSuffix(StringRef Name, OperandVector &Operands);

  bool mnemonicIsValid(StringRef Mnemonic, unsigned VariantID);

  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;

  bool ParseDirective(AsmToken DirectiveID) override;

  OperandMatchResultTy parseMemOperand(OperandVector &Operands);
  OperandMatchResultTy
  matchAnyRegisterNameWithoutDollar(OperandVector &Operands,
                                    StringRef Identifier, SMLoc S);
  OperandMatchResultTy matchAnyRegisterWithoutDollar(OperandVector &Operands,
                                                     const AsmToken &Token,
                                                     SMLoc S);
  OperandMatchResultTy matchAnyRegisterWithoutDollar(OperandVector &Operands,
                                                     SMLoc S);
  OperandMatchResultTy parseAnyRegister(OperandVector &Operands);
  OperandMatchResultTy parseImm(OperandVector &Operands);
  OperandMatchResultTy parseJumpTarget(OperandVector &Operands);
  OperandMatchResultTy parseInvNum(OperandVector &Operands);
  OperandMatchResultTy parseRegisterList(OperandVector &Operands);

  bool searchSymbolAlias(OperandVector &Operands);

  bool parseOperand(OperandVector &, StringRef Mnemonic);

  enum MacroExpanderResultTy {
    MER_NotAMacro,
    MER_Success,
    MER_Fail,
  };

  // Expands assembly pseudo instructions.
  MacroExpanderResultTy tryExpandInstruction(MCInst &Inst, SMLoc IDLoc,
                                             MCStreamer &Out,
                                             const MCSubtargetInfo *STI);

  bool expandJalWithRegs(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                         const MCSubtargetInfo *STI);

  bool loadImmediate(int64_t ImmValue, unsigned DstReg, unsigned SrcReg,
                     bool Is32BitImm, bool IsAddress, SMLoc IDLoc,
                     MCStreamer &Out, const MCSubtargetInfo *STI);

  bool loadAndAddSymbolAddress(const MCExpr *SymExpr, unsigned DstReg,
                               unsigned SrcReg, bool Is32BitSym, SMLoc IDLoc,
                               MCStreamer &Out, const MCSubtargetInfo *STI);

  bool emitPartialAddress(FgpuTargetStreamer &TOut, SMLoc IDLoc, MCSymbol *Sym);

  bool expandLoadImm(MCInst &Inst, bool Is32BitImm, SMLoc IDLoc,
                     MCStreamer &Out, const MCSubtargetInfo *STI);

  bool expandLoadSingleImmToGPR(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                const MCSubtargetInfo *STI);
  bool expandLoadSingleImmToFPR(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                const MCSubtargetInfo *STI);
  bool expandLoadDoubleImmToGPR(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                const MCSubtargetInfo *STI);
  bool expandLoadDoubleImmToFPR(MCInst &Inst, bool Is64FPU, SMLoc IDLoc,
                                MCStreamer &Out, const MCSubtargetInfo *STI);

  bool expandLoadAddress(unsigned DstReg, unsigned BaseReg,
                         const MCOperand &Offset, bool Is32BitAddress,
                         SMLoc IDLoc, MCStreamer &Out,
                         const MCSubtargetInfo *STI);

  bool expandUncondBranchMMPseudo(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                  const MCSubtargetInfo *STI);

  void expandMem16Inst(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                       const MCSubtargetInfo *STI, bool IsLoad);
  void expandMem9Inst(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                      const MCSubtargetInfo *STI, bool IsLoad);

  bool expandLoadStoreMultiple(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                               const MCSubtargetInfo *STI);

  bool expandAliasImmediate(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                            const MCSubtargetInfo *STI);

  bool expandBranchImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                       const MCSubtargetInfo *STI);

  bool expandCondBranches(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                          const MCSubtargetInfo *STI);

  bool expandDivRem(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                    const MCSubtargetInfo *STI, const bool IsFgpu64,
                    const bool Signed);

  bool expandTrunc(MCInst &Inst, bool IsDouble, bool Is64FPU, SMLoc IDLoc,
                   MCStreamer &Out, const MCSubtargetInfo *STI);

  bool expandUlh(MCInst &Inst, bool Signed, SMLoc IDLoc, MCStreamer &Out,
                 const MCSubtargetInfo *STI);

  bool expandUsh(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                 const MCSubtargetInfo *STI);

  bool expandUxw(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                 const MCSubtargetInfo *STI);

  bool expandSge(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                 const MCSubtargetInfo *STI);

  bool expandSgeImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                    const MCSubtargetInfo *STI);

  bool expandSgtImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                    const MCSubtargetInfo *STI);

  bool expandSle(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                 const MCSubtargetInfo *STI);

  bool expandSleImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                    const MCSubtargetInfo *STI);

  bool expandRotation(MCInst &Inst, SMLoc IDLoc,
                      MCStreamer &Out, const MCSubtargetInfo *STI);
  bool expandRotationImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                         const MCSubtargetInfo *STI);
  bool expandDRotation(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                       const MCSubtargetInfo *STI);
  bool expandDRotationImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                          const MCSubtargetInfo *STI);

  bool expandAbs(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                 const MCSubtargetInfo *STI);

  bool expandMulImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                    const MCSubtargetInfo *STI);

  bool expandMulO(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                  const MCSubtargetInfo *STI);

  bool expandMulOU(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                   const MCSubtargetInfo *STI);

  bool expandDMULMacro(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                       const MCSubtargetInfo *STI);

  bool expandLoadStoreDMacro(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                             const MCSubtargetInfo *STI, bool IsLoad);

  bool expandStoreDM1Macro(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                           const MCSubtargetInfo *STI);

  bool expandSeq(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                 const MCSubtargetInfo *STI);

  bool expandSeqI(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                  const MCSubtargetInfo *STI);

  bool expandSne(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                 const MCSubtargetInfo *STI);

  bool expandSneI(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                  const MCSubtargetInfo *STI);

  bool expandMXTRAlias(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                       const MCSubtargetInfo *STI);

  bool expandSaaAddr(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                     const MCSubtargetInfo *STI);

  bool reportParseError(const Twine &ErrorMsg);
  bool reportParseError(SMLoc Loc, const Twine &ErrorMsg);

  bool parseMemOffset(const MCExpr *&Res, bool isParenExpr);

  bool parseSetFgpu0Directive();
  bool parseSetArchDirective();
  bool parseSetFeature(uint64_t Feature);
  bool isPicAndNotNxxAbi(); // Used by .cpload, .cprestore, and .cpsetup.
  bool parseDirectiveCpAdd(SMLoc Loc);
  bool parseDirectiveCpLoad(SMLoc Loc);
  bool parseDirectiveCpLocal(SMLoc Loc);
  bool parseDirectiveCpRestore(SMLoc Loc);
  bool parseDirectiveCPSetup();
  bool parseDirectiveCPReturn();
  bool parseDirectiveNaN();
  bool parseDirectiveSet();
  bool parseDirectiveOption();
  bool parseInsnDirective();
  bool parseRSectionDirective(StringRef Section);
  bool parseSSectionDirective(StringRef Section, unsigned Type);

  bool parseSetAtDirective();
  bool parseSetNoAtDirective();
  bool parseSetMacroDirective();
  bool parseSetNoMacroDirective();
  bool parseSetMsaDirective();
  bool parseSetNoMsaDirective();
  bool parseSetNoDspDirective();
  bool parseSetNoFgpu3DDirective();
  bool parseSetReorderDirective();
  bool parseSetNoReorderDirective();
  bool parseSetFgpu16Directive();
  bool parseSetNoFgpu16Directive();
  bool parseSetFpDirective();
  bool parseSetOddSPRegDirective();
  bool parseSetNoOddSPRegDirective();
  bool parseSetPopDirective();
  bool parseSetPushDirective();
  bool parseSetSoftFloatDirective();
  bool parseSetHardFloatDirective();
  bool parseSetMtDirective();
  bool parseSetNoMtDirective();
  bool parseSetNoCRCDirective();
  bool parseSetNoVirtDirective();
  bool parseSetNoGINVDirective();

  bool parseSetAssignment();

  bool parseDirectiveGpWord();
  bool parseDirectiveGpDWord();
  bool parseDirectiveDtpRelWord();
  bool parseDirectiveDtpRelDWord();
  bool parseDirectiveTpRelWord();
  bool parseDirectiveTpRelDWord();
  bool parseDirectiveModule();
  bool parseDirectiveModuleFP();
  bool parseFpABIValue(FgpuABIFlagsSection::FpABIKind &FpABI,
                       StringRef Directive);

  bool parseInternalDirectiveReallowModule();

  bool eatComma(StringRef ErrorStr);

  int matchCPURegisterName(StringRef Symbol);

  int matchHWRegsRegisterName(StringRef Symbol);

  int matchFPURegisterName(StringRef Name);

  int matchFCCRegisterName(StringRef Name);

  int matchACRegisterName(StringRef Name);

  int matchMSA128RegisterName(StringRef Name);

  int matchMSA128CtrlRegisterName(StringRef Name);

  unsigned getReg(int RC, int RegNo);

  /// Returns the internal register number for the current AT. Also checks if
  /// the current AT is unavailable (set to $0) and gives an error if it is.
  /// This should be used in pseudo-instruction expansions which need AT.
  unsigned getATReg(SMLoc Loc);

  bool canUseATReg();

  bool processInstruction(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                          const MCSubtargetInfo *STI);

  // Helper function that checks if the value of a vector index is within the
  // boundaries of accepted values for each RegisterKind
  // Example: INSERT.B $w0[n], $1 => 16 > n >= 0
  bool validateMSAIndex(int Val, int RegKind);

  // Selects a new architecture by updating the FeatureBits with the necessary
  // info including implied dependencies.
  // Internally, it clears all the feature bits related to *any* architecture
  // and selects the new one using the ToggleFeature functionality of the
  // MCSubtargetInfo object that handles implied dependencies. The reason we
  // clear all the arch related bits manually is because ToggleFeature only
  // clears the features that imply the feature being cleared and not the
  // features implied by the feature being cleared. This is easier to see
  // with an example:
  //  --------------------------------------------------
  // | Feature         | Implies                        |
  // | -------------------------------------------------|
  // | FeatureFgpu1    | None                           |
  // | FeatureFgpu2    | FeatureFgpu1                   |
  // | FeatureFgpu3    | FeatureFgpu2 | FeatureFgpuGP64 |
  // | FeatureFgpu4    | FeatureFgpu3                   |
  // | ...             |                                |
  //  --------------------------------------------------
  //
  // Setting Fgpu3 is equivalent to set: (FeatureFgpu3 | FeatureFgpu2 |
  // FeatureFgpuGP64 | FeatureFgpu1)
  // Clearing Fgpu3 is equivalent to clear (FeatureFgpu3 | FeatureFgpu4).
  void selectArch(StringRef ArchFeature) {
    MCSubtargetInfo &STI = copySTI();
    FeatureBitset FeatureBits = STI.getFeatureBits();
    FeatureBits &= ~FgpuAssemblerOptions::AllArchRelatedMask;
    STI.setFeatureBits(FeatureBits);
    setAvailableFeatures(
        ComputeAvailableFeatures(STI.ToggleFeature(ArchFeature)));
    AssemblerOptions.back()->setFeatures(STI.getFeatureBits());
  }

  void setFeatureBits(uint64_t Feature, StringRef FeatureString) {
    if (!(getSTI().getFeatureBits()[Feature])) {
      MCSubtargetInfo &STI = copySTI();
      setAvailableFeatures(
          ComputeAvailableFeatures(STI.ToggleFeature(FeatureString)));
      AssemblerOptions.back()->setFeatures(STI.getFeatureBits());
    }
  }

  void clearFeatureBits(uint64_t Feature, StringRef FeatureString) {
    if (getSTI().getFeatureBits()[Feature]) {
      MCSubtargetInfo &STI = copySTI();
      setAvailableFeatures(
          ComputeAvailableFeatures(STI.ToggleFeature(FeatureString)));
      AssemblerOptions.back()->setFeatures(STI.getFeatureBits());
    }
  }

  void setModuleFeatureBits(uint64_t Feature, StringRef FeatureString) {
    setFeatureBits(Feature, FeatureString);
    AssemblerOptions.front()->setFeatures(getSTI().getFeatureBits());
  }

  void clearModuleFeatureBits(uint64_t Feature, StringRef FeatureString) {
    clearFeatureBits(Feature, FeatureString);
    AssemblerOptions.front()->setFeatures(getSTI().getFeatureBits());
  }

public:
  enum FgpuMatchResultTy {
    Match_RequiresDifferentSrcAndDst = FIRST_TARGET_MATCH_RESULT_TY,
    Match_RequiresDifferentOperands,
    Match_RequiresNoZeroRegister,
    Match_RequiresSameSrcAndDst,
    Match_NoFCCRegisterForCurrentISA,
    Match_NonZeroOperandForSync,
    Match_NonZeroOperandForMTCX,
    Match_RequiresPosSizeRange0_32,
    Match_RequiresPosSizeRange33_64,
    Match_RequiresPosSizeUImm6,
#define GET_OPERAND_DIAGNOSTIC_TYPES
#include "FgpuGenAsmMatcher.inc"
#undef GET_OPERAND_DIAGNOSTIC_TYPES
  };

  FgpuAsmParser(const MCSubtargetInfo &sti, MCAsmParser &parser,
                const MCInstrInfo &MII, const MCTargetOptions &Options)
    : MCTargetAsmParser(Options, sti, MII),
        ABI(FgpuABIInfo::computeTargetABI(Triple(sti.getTargetTriple()),
                                          sti.getCPU(), Options)) {
    MCAsmParserExtension::Initialize(parser);

    parser.addAliasForDirective(".asciiz", ".asciz");
    parser.addAliasForDirective(".hword", ".2byte");
    parser.addAliasForDirective(".word", ".4byte");
    parser.addAliasForDirective(".dword", ".8byte");

    // Initialize the set of available features.
    setAvailableFeatures(ComputeAvailableFeatures(getSTI().getFeatureBits()));

    // Remember the initial assembler options. The user can not modify these.
    AssemblerOptions.push_back(
        std::make_unique<FgpuAssemblerOptions>(getSTI().getFeatureBits()));

    // Create an assembler options environment for the user to modify.
    AssemblerOptions.push_back(
        std::make_unique<FgpuAssemblerOptions>(getSTI().getFeatureBits()));

    getTargetStreamer().updateABIInfo(*this);

    if (!isABI_O32() && !useOddSPReg() != 0)
      report_fatal_error("-mno-odd-spreg requires the O32 ABI");

    CurrentFn = nullptr;

    IsPicEnabled = getContext().getObjectFileInfo()->isPositionIndependent();

    IsCpRestoreSet = false;
    CpRestoreOffset = -1;
    GPReg = ABI.GetGlobalPtr();

    const Triple &TheTriple = sti.getTargetTriple();
    IsLittleEndian = TheTriple.isLittleEndian();

    if (getSTI().getCPU() == "fgpu64r6" && inMicroFgpuMode())
      report_fatal_error("microFGPU64R6 is not supported", false);

    if (!isABI_O32() && inMicroFgpuMode())
      report_fatal_error("microFGPU64 is not supported", false);
  }

  /// True if all of $fcc0 - $fcc7 exist for the current ISA.
  bool hasEightFccRegisters() const { return hasFgpu4() || hasFgpu32(); }

  bool isGP64bit() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureGP64Bit];
  }

  bool isFP64bit() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureFP64Bit];
  }

  bool isJalrRelocAvailable(const MCExpr *JalExpr) {
    if (!EmitJalrReloc)
      return false;
    MCValue Res;
    if (!JalExpr->evaluateAsRelocatable(Res, nullptr, nullptr))
      return false;
    if (Res.getSymB() != nullptr)
      return false;
    if (Res.getConstant() != 0)
      return ABI.IsN32() || ABI.IsN64();
    return true;
  }

  const FgpuABIInfo &getABI() const { return ABI; }
  bool isABI_N32() const { return ABI.IsN32(); }
  bool isABI_N64() const { return ABI.IsN64(); }
  bool isABI_O32() const { return ABI.IsO32(); }
  bool isABI_FPXX() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureFPXX];
  }

  bool useOddSPReg() const {
    return !(getSTI().getFeatureBits()[Fgpu::FeatureNoOddSPReg]);
  }

  bool inMicroFgpuMode() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureMicroFgpu];
  }

  bool hasFgpu1() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureFgpu1];
  }

  bool hasFgpu2() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureFgpu2];
  }

  bool hasFgpu3() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureFgpu3];
  }

  bool hasFgpu4() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureFgpu4];
  }

  bool hasFgpu5() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureFgpu5];
  }

  bool hasFgpu32() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureFgpu32];
  }

  bool hasFgpu64() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureFgpu64];
  }

  bool hasFgpu32r2() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureFgpu32r2];
  }

  bool hasFgpu64r2() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureFgpu64r2];
  }

  bool hasFgpu32r3() const {
    return (getSTI().getFeatureBits()[Fgpu::FeatureFgpu32r3]);
  }

  bool hasFgpu64r3() const {
    return (getSTI().getFeatureBits()[Fgpu::FeatureFgpu64r3]);
  }

  bool hasFgpu32r5() const {
    return (getSTI().getFeatureBits()[Fgpu::FeatureFgpu32r5]);
  }

  bool hasFgpu64r5() const {
    return (getSTI().getFeatureBits()[Fgpu::FeatureFgpu64r5]);
  }

  bool hasFgpu32r6() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureFgpu32r6];
  }

  bool hasFgpu64r6() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureFgpu64r6];
  }

  bool hasDSP() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureDSP];
  }

  bool hasDSPR2() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureDSPR2];
  }

  bool hasDSPR3() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureDSPR3];
  }

  bool hasMSA() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureMSA];
  }

  bool hasCnFgpu() const {
    return (getSTI().getFeatureBits()[Fgpu::FeatureCnFgpu]);
  }

  bool hasCnFgpuP() const {
    return (getSTI().getFeatureBits()[Fgpu::FeatureCnFgpuP]);
  }

  bool inPicMode() {
    return IsPicEnabled;
  }

  bool inFgpu16Mode() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureFgpu16];
  }

  bool useTraps() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureUseTCCInDIV];
  }

  bool useSoftFloat() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureSoftFloat];
  }
  bool hasMT() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureMT];
  }

  bool hasCRC() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureCRC];
  }

  bool hasVirt() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureVirt];
  }

  bool hasGINV() const {
    return getSTI().getFeatureBits()[Fgpu::FeatureGINV];
  }

  /// Warn if RegIndex is the same as the current AT.
  void warnIfRegIndexIsAT(unsigned RegIndex, SMLoc Loc);

  void warnIfNoMacro(SMLoc Loc);

  bool isLittle() const { return IsLittleEndian; }

  const MCExpr *createTargetUnaryExpr(const MCExpr *E,
                                      AsmToken::TokenKind OperatorToken,
                                      MCContext &Ctx) override {
    switch(OperatorToken) {
    default:
      llvm_unreachable("Unknown token");
      return nullptr;
    case AsmToken::PercentCall16:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_GOT_CALL, E, Ctx);
    case AsmToken::PercentCall_Hi:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_CALL_HI16, E, Ctx);
    case AsmToken::PercentCall_Lo:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_CALL_LO16, E, Ctx);
    case AsmToken::PercentDtprel_Hi:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_DTPREL_HI, E, Ctx);
    case AsmToken::PercentDtprel_Lo:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_DTPREL_LO, E, Ctx);
    case AsmToken::PercentGot:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_GOT, E, Ctx);
    case AsmToken::PercentGot_Disp:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_GOT_DISP, E, Ctx);
    case AsmToken::PercentGot_Hi:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_GOT_HI16, E, Ctx);
    case AsmToken::PercentGot_Lo:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_GOT_LO16, E, Ctx);
    case AsmToken::PercentGot_Ofst:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_GOT_OFST, E, Ctx);
    case AsmToken::PercentGot_Page:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_GOT_PAGE, E, Ctx);
    case AsmToken::PercentGottprel:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_GOTTPREL, E, Ctx);
    case AsmToken::PercentGp_Rel:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_GPREL, E, Ctx);
    case AsmToken::PercentHi:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_HI, E, Ctx);
    case AsmToken::PercentHigher:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_HIGHER, E, Ctx);
    case AsmToken::PercentHighest:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_HIGHEST, E, Ctx);
    case AsmToken::PercentLo:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_LO, E, Ctx);
    case AsmToken::PercentNeg:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_NEG, E, Ctx);
    case AsmToken::PercentPcrel_Hi:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_PCREL_HI16, E, Ctx);
    case AsmToken::PercentPcrel_Lo:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_PCREL_LO16, E, Ctx);
    case AsmToken::PercentTlsgd:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_TLSGD, E, Ctx);
    case AsmToken::PercentTlsldm:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_TLSLDM, E, Ctx);
    case AsmToken::PercentTprel_Hi:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_TPREL_HI, E, Ctx);
    case AsmToken::PercentTprel_Lo:
      return FgpuMCExpr::create(FgpuMCExpr::MEK_TPREL_LO, E, Ctx);
    }
  }
};

/// FgpuOperand - Instances of this class represent a parsed Fgpu machine
/// instruction.
class FgpuOperand : public MCParsedAsmOperand {
public:
  /// Broad categories of register classes
  /// The exact class is finalized by the render method.
  enum RegKind {
    RegKind_GPR = 1,      /// GPR32 and GPR64 (depending on isGP64bit())
    RegKind_FGR = 2,      /// FGR32, FGR64, AFGR64 (depending on context and
                          /// isFP64bit())
    RegKind_FCC = 4,      /// FCC
    RegKind_MSA128 = 8,   /// MSA128[BHWD] (makes no difference which)
    RegKind_MSACtrl = 16, /// MSA control registers
    RegKind_COP2 = 32,    /// COP2
    RegKind_ACC = 64,     /// HI32DSP, LO32DSP, and ACC64DSP (depending on
                          /// context).
    RegKind_CCR = 128,    /// CCR
    RegKind_HWRegs = 256, /// HWRegs
    RegKind_COP3 = 512,   /// COP3
    RegKind_COP0 = 1024,  /// COP0
    /// Potentially any (e.g. $1)
    RegKind_Numeric = RegKind_GPR | RegKind_FGR | RegKind_FCC | RegKind_MSA128 |
                      RegKind_MSACtrl | RegKind_COP2 | RegKind_ACC |
                      RegKind_CCR | RegKind_HWRegs | RegKind_COP3 | RegKind_COP0
  };

private:
  enum KindTy {
    k_Immediate,     /// An immediate (possibly involving symbol references)
    k_Memory,        /// Base + Offset Memory Address
    k_RegisterIndex, /// A register index in one or more RegKind.
    k_Token,         /// A simple token
    k_RegList,       /// A physical register list
  } Kind;

public:
  FgpuOperand(KindTy K, FgpuAsmParser &Parser)
      : MCParsedAsmOperand(), Kind(K), AsmParser(Parser) {}

  ~FgpuOperand() override {
    switch (Kind) {
    case k_Memory:
      delete Mem.Base;
      break;
    case k_RegList:
      delete RegList.List;
      break;
    case k_Immediate:
    case k_RegisterIndex:
    case k_Token:
      break;
    }
  }

private:
  /// For diagnostics, and checking the assembler temporary
  FgpuAsmParser &AsmParser;

  struct Token {
    const char *Data;
    unsigned Length;
  };

  struct RegIdxOp {
    unsigned Index; /// Index into the register class
    RegKind Kind;   /// Bitfield of the kinds it could possibly be
    struct Token Tok; /// The input token this operand originated from.
    const MCRegisterInfo *RegInfo;
  };

  struct ImmOp {
    const MCExpr *Val;
  };

  struct MemOp {
    FgpuOperand *Base;
    const MCExpr *Off;
  };

  struct RegListOp {
    SmallVector<unsigned, 10> *List;
  };

  union {
    struct Token Tok;
    struct RegIdxOp RegIdx;
    struct ImmOp Imm;
    struct MemOp Mem;
    struct RegListOp RegList;
  };

  SMLoc StartLoc, EndLoc;

  /// Internal constructor for register kinds
  static std::unique_ptr<FgpuOperand> CreateReg(unsigned Index, StringRef Str,
                                                RegKind RegKind,
                                                const MCRegisterInfo *RegInfo,
                                                SMLoc S, SMLoc E,
                                                FgpuAsmParser &Parser) {
    auto Op = std::make_unique<FgpuOperand>(k_RegisterIndex, Parser);
    Op->RegIdx.Index = Index;
    Op->RegIdx.RegInfo = RegInfo;
    Op->RegIdx.Kind = RegKind;
    Op->RegIdx.Tok.Data = Str.data();
    Op->RegIdx.Tok.Length = Str.size();
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

public:
  /// Coerce the register to GPR32 and return the real register for the current
  /// target.
  unsigned getGPR32Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_GPR) && "Invalid access!");
    AsmParser.warnIfRegIndexIsAT(RegIdx.Index, StartLoc);
    unsigned ClassID = Fgpu::GPR32RegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to GPR32 and return the real register for the current
  /// target.
  unsigned getGPRMM16Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_GPR) && "Invalid access!");
    unsigned ClassID = Fgpu::GPR32RegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to GPR64 and return the real register for the current
  /// target.
  unsigned getGPR64Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_GPR) && "Invalid access!");
    unsigned ClassID = Fgpu::GPR64RegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

private:
  /// Coerce the register to AFGR64 and return the real register for the current
  /// target.
  unsigned getAFGR64Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_FGR) && "Invalid access!");
    if (RegIdx.Index % 2 != 0)
      AsmParser.Warning(StartLoc, "Float register should be even.");
    return RegIdx.RegInfo->getRegClass(Fgpu::AFGR64RegClassID)
        .getRegister(RegIdx.Index / 2);
  }

  /// Coerce the register to FGR64 and return the real register for the current
  /// target.
  unsigned getFGR64Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_FGR) && "Invalid access!");
    return RegIdx.RegInfo->getRegClass(Fgpu::FGR64RegClassID)
        .getRegister(RegIdx.Index);
  }

  /// Coerce the register to FGR32 and return the real register for the current
  /// target.
  unsigned getFGR32Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_FGR) && "Invalid access!");
    return RegIdx.RegInfo->getRegClass(Fgpu::FGR32RegClassID)
        .getRegister(RegIdx.Index);
  }

  /// Coerce the register to FCC and return the real register for the current
  /// target.
  unsigned getFCCReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_FCC) && "Invalid access!");
    return RegIdx.RegInfo->getRegClass(Fgpu::FCCRegClassID)
        .getRegister(RegIdx.Index);
  }

  /// Coerce the register to MSA128 and return the real register for the current
  /// target.
  unsigned getMSA128Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_MSA128) && "Invalid access!");
    // It doesn't matter which of the MSA128[BHWD] classes we use. They are all
    // identical
    unsigned ClassID = Fgpu::MSA128BRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to MSACtrl and return the real register for the
  /// current target.
  unsigned getMSACtrlReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_MSACtrl) && "Invalid access!");
    unsigned ClassID = Fgpu::MSACtrlRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to COP0 and return the real register for the
  /// current target.
  unsigned getCOP0Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_COP0) && "Invalid access!");
    unsigned ClassID = Fgpu::COP0RegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to COP2 and return the real register for the
  /// current target.
  unsigned getCOP2Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_COP2) && "Invalid access!");
    unsigned ClassID = Fgpu::COP2RegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to COP3 and return the real register for the
  /// current target.
  unsigned getCOP3Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_COP3) && "Invalid access!");
    unsigned ClassID = Fgpu::COP3RegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to ACC64DSP and return the real register for the
  /// current target.
  unsigned getACC64DSPReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_ACC) && "Invalid access!");
    unsigned ClassID = Fgpu::ACC64DSPRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to HI32DSP and return the real register for the
  /// current target.
  unsigned getHI32DSPReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_ACC) && "Invalid access!");
    unsigned ClassID = Fgpu::HI32DSPRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to LO32DSP and return the real register for the
  /// current target.
  unsigned getLO32DSPReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_ACC) && "Invalid access!");
    unsigned ClassID = Fgpu::LO32DSPRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to CCR and return the real register for the
  /// current target.
  unsigned getCCRReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_CCR) && "Invalid access!");
    unsigned ClassID = Fgpu::CCRRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to HWRegs and return the real register for the
  /// current target.
  unsigned getHWRegsReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_HWRegs) && "Invalid access!");
    unsigned ClassID = Fgpu::HWRegsRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

public:
  void addExpr(MCInst &Inst, const MCExpr *Expr) const {
    // Add as immediate when possible.  Null MCExpr = 0.
    if (!Expr)
      Inst.addOperand(MCOperand::createImm(0));
    else if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Expr))
      Inst.addOperand(MCOperand::createImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
  }

  void addRegOperands(MCInst &Inst, unsigned N) const {
    llvm_unreachable("Use a custom parser instead");
  }

  /// Render the operand to an MCInst as a GPR32
  /// Asserts if the wrong number of operands are requested, or the operand
  /// is not a k_RegisterIndex compatible with RegKind_GPR
  void addGPR32ZeroAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPR32Reg()));
  }

  void addGPR32NonZeroAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPR32Reg()));
  }

  void addGPR32AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPR32Reg()));
  }

  void addGPRMM16AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPRMM16Reg()));
  }

  void addGPRMM16AsmRegZeroOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPRMM16Reg()));
  }

  void addGPRMM16AsmRegMovePOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPRMM16Reg()));
  }

  void addGPRMM16AsmRegMovePPairFirstOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPRMM16Reg()));
  }

  void addGPRMM16AsmRegMovePPairSecondOperands(MCInst &Inst,
                                               unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPRMM16Reg()));
  }

  /// Render the operand to an MCInst as a GPR64
  /// Asserts if the wrong number of operands are requested, or the operand
  /// is not a k_RegisterIndex compatible with RegKind_GPR
  void addGPR64AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPR64Reg()));
  }

  void addAFGR64AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getAFGR64Reg()));
  }

  void addStrictlyAFGR64AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getAFGR64Reg()));
  }

  void addStrictlyFGR64AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getFGR64Reg()));
  }

  void addFGR64AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getFGR64Reg()));
  }

  void addFGR32AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getFGR32Reg()));
    // FIXME: We ought to do this for -integrated-as without -via-file-asm too.
    // FIXME: This should propagate failure up to parseStatement.
    if (!AsmParser.useOddSPReg() && RegIdx.Index & 1)
      AsmParser.getParser().printError(
          StartLoc, "-mno-odd-spreg prohibits the use of odd FPU "
                    "registers");
  }

  void addStrictlyFGR32AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getFGR32Reg()));
    // FIXME: We ought to do this for -integrated-as without -via-file-asm too.
    if (!AsmParser.useOddSPReg() && RegIdx.Index & 1)
      AsmParser.Error(StartLoc, "-mno-odd-spreg prohibits the use of odd FPU "
                                "registers");
  }

  void addFCCAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getFCCReg()));
  }

  void addMSA128AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getMSA128Reg()));
  }

  void addMSACtrlAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getMSACtrlReg()));
  }

  void addCOP0AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getCOP0Reg()));
  }

  void addCOP2AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getCOP2Reg()));
  }

  void addCOP3AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getCOP3Reg()));
  }

  void addACC64DSPAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getACC64DSPReg()));
  }

  void addHI32DSPAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getHI32DSPReg()));
  }

  void addLO32DSPAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getLO32DSPReg()));
  }

  void addCCRAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getCCRReg()));
  }

  void addHWRegsAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getHWRegsReg()));
  }

  template <unsigned Bits, int Offset = 0, int AdjustOffset = 0>
  void addConstantUImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    uint64_t Imm = getConstantImm() - Offset;
    Imm &= (1ULL << Bits) - 1;
    Imm += Offset;
    Imm += AdjustOffset;
    Inst.addOperand(MCOperand::createImm(Imm));
  }

  template <unsigned Bits>
  void addSImmOperands(MCInst &Inst, unsigned N) const {
    if (isImm() && !isConstantImm()) {
      addExpr(Inst, getImm());
      return;
    }
    addConstantSImmOperands<Bits, 0, 0>(Inst, N);
  }

  template <unsigned Bits>
  void addUImmOperands(MCInst &Inst, unsigned N) const {
    if (isImm() && !isConstantImm()) {
      addExpr(Inst, getImm());
      return;
    }
    addConstantUImmOperands<Bits, 0, 0>(Inst, N);
  }

  template <unsigned Bits, int Offset = 0, int AdjustOffset = 0>
  void addConstantSImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    int64_t Imm = getConstantImm() - Offset;
    Imm = SignExtend64<Bits>(Imm);
    Imm += Offset;
    Imm += AdjustOffset;
    Inst.addOperand(MCOperand::createImm(Imm));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    const MCExpr *Expr = getImm();
    addExpr(Inst, Expr);
  }

  void addMemOperands(MCInst &Inst, unsigned N) const {
    assert(N == 2 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::createReg(AsmParser.getABI().ArePtrs64bit()
                                             ? getMemBase()->getGPR64Reg()
                                             : getMemBase()->getGPR32Reg()));

    const MCExpr *Expr = getMemOff();
    addExpr(Inst, Expr);
  }

  void addMicroFgpuMemOperands(MCInst &Inst, unsigned N) const {
    assert(N == 2 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::createReg(getMemBase()->getGPRMM16Reg()));

    const MCExpr *Expr = getMemOff();
    addExpr(Inst, Expr);
  }

  void addRegListOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");

    for (auto RegNo : getRegList())
      Inst.addOperand(MCOperand::createReg(RegNo));
  }

  bool isReg() const override {
    // As a special case until we sort out the definition of div/divu, accept
    // $0/$zero here so that MCK_ZERO works correctly.
    return isGPRAsmReg() && RegIdx.Index == 0;
  }

  bool isRegIdx() const { return Kind == k_RegisterIndex; }
  bool isImm() const override { return Kind == k_Immediate; }

  bool isConstantImm() const {
    int64_t Res;
    return isImm() && getImm()->evaluateAsAbsolute(Res);
  }

  bool isConstantImmz() const {
    return isConstantImm() && getConstantImm() == 0;
  }

  template <unsigned Bits, int Offset = 0> bool isConstantUImm() const {
    return isConstantImm() && isUInt<Bits>(getConstantImm() - Offset);
  }

  template <unsigned Bits> bool isSImm() const {
    return isConstantImm() ? isInt<Bits>(getConstantImm()) : isImm();
  }

  template <unsigned Bits> bool isUImm() const {
    return isConstantImm() ? isUInt<Bits>(getConstantImm()) : isImm();
  }

  template <unsigned Bits> bool isAnyImm() const {
    return isConstantImm() ? (isInt<Bits>(getConstantImm()) ||
                              isUInt<Bits>(getConstantImm()))
                           : isImm();
  }

  template <unsigned Bits, int Offset = 0> bool isConstantSImm() const {
    return isConstantImm() && isInt<Bits>(getConstantImm() - Offset);
  }

  template <unsigned Bottom, unsigned Top> bool isConstantUImmRange() const {
    return isConstantImm() && getConstantImm() >= Bottom &&
           getConstantImm() <= Top;
  }

  bool isToken() const override {
    // Note: It's not possible to pretend that other operand kinds are tokens.
    // The matcher emitter checks tokens first.
    return Kind == k_Token;
  }

  bool isMem() const override { return Kind == k_Memory; }

  bool isConstantMemOff() const {
    return isMem() && isa<MCConstantExpr>(getMemOff());
  }

  // Allow relocation operators.
  template <unsigned Bits, unsigned ShiftAmount = 0>
  bool isMemWithSimmOffset() const {
    if (!isMem())
      return false;
    if (!getMemBase()->isGPRAsmReg())
      return false;
    if (isa<MCTargetExpr>(getMemOff()) ||
        (isConstantMemOff() &&
         isShiftedInt<Bits, ShiftAmount>(getConstantMemOff())))
      return true;
    MCValue Res;
    bool IsReloc = getMemOff()->evaluateAsRelocatable(Res, nullptr, nullptr);
    return IsReloc && isShiftedInt<Bits, ShiftAmount>(Res.getConstant());
  }

  bool isMemWithPtrSizeOffset() const {
    if (!isMem())
      return false;
    if (!getMemBase()->isGPRAsmReg())
      return false;
    const unsigned PtrBits = AsmParser.getABI().ArePtrs64bit() ? 64 : 32;
    if (isa<MCTargetExpr>(getMemOff()) ||
        (isConstantMemOff() && isIntN(PtrBits, getConstantMemOff())))
      return true;
    MCValue Res;
    bool IsReloc = getMemOff()->evaluateAsRelocatable(Res, nullptr, nullptr);
    return IsReloc && isIntN(PtrBits, Res.getConstant());
  }

  bool isMemWithGRPMM16Base() const {
    return isMem() && getMemBase()->isMM16AsmReg();
  }

  template <unsigned Bits> bool isMemWithUimmOffsetSP() const {
    return isMem() && isConstantMemOff() && isUInt<Bits>(getConstantMemOff())
      && getMemBase()->isRegIdx() && (getMemBase()->getGPR32Reg() == Fgpu::SP);
  }

  template <unsigned Bits> bool isMemWithUimmWordAlignedOffsetSP() const {
    return isMem() && isConstantMemOff() && isUInt<Bits>(getConstantMemOff())
      && (getConstantMemOff() % 4 == 0) && getMemBase()->isRegIdx()
      && (getMemBase()->getGPR32Reg() == Fgpu::SP);
  }

  template <unsigned Bits> bool isMemWithSimmWordAlignedOffsetGP() const {
    return isMem() && isConstantMemOff() && isInt<Bits>(getConstantMemOff())
      && (getConstantMemOff() % 4 == 0) && getMemBase()->isRegIdx()
      && (getMemBase()->getGPR32Reg() == Fgpu::GP);
  }

  template <unsigned Bits, unsigned ShiftLeftAmount>
  bool isScaledUImm() const {
    return isConstantImm() &&
           isShiftedUInt<Bits, ShiftLeftAmount>(getConstantImm());
  }

  template <unsigned Bits, unsigned ShiftLeftAmount>
  bool isScaledSImm() const {
    if (isConstantImm() &&
        isShiftedInt<Bits, ShiftLeftAmount>(getConstantImm()))
      return true;
    // Operand can also be a symbol or symbol plus
    // offset in case of relocations.
    if (Kind != k_Immediate)
      return false;
    MCValue Res;
    bool Success = getImm()->evaluateAsRelocatable(Res, nullptr, nullptr);
    return Success && isShiftedInt<Bits, ShiftLeftAmount>(Res.getConstant());
  }

  bool isRegList16() const {
    if (!isRegList())
      return false;

    int Size = RegList.List->size();
    if (Size < 2 || Size > 5)
      return false;

    unsigned R0 = RegList.List->front();
    unsigned R1 = RegList.List->back();
    if (!((R0 == Fgpu::S0 && R1 == Fgpu::RA) ||
          (R0 == Fgpu::S0_64 && R1 == Fgpu::RA_64)))
      return false;

    int PrevReg = *RegList.List->begin();
    for (int i = 1; i < Size - 1; i++) {
      int Reg = (*(RegList.List))[i];
      if ( Reg != PrevReg + 1)
        return false;
      PrevReg = Reg;
    }

    return true;
  }

  bool isInvNum() const { return Kind == k_Immediate; }

  bool isLSAImm() const {
    if (!isConstantImm())
      return false;
    int64_t Val = getConstantImm();
    return 1 <= Val && Val <= 4;
  }

  bool isRegList() const { return Kind == k_RegList; }

  StringRef getToken() const {
    assert(Kind == k_Token && "Invalid access!");
    return StringRef(Tok.Data, Tok.Length);
  }

  unsigned getReg() const override {
    // As a special case until we sort out the definition of div/divu, accept
    // $0/$zero here so that MCK_ZERO works correctly.
    if (Kind == k_RegisterIndex && RegIdx.Index == 0 &&
        RegIdx.Kind & RegKind_GPR)
      return getGPR32Reg(); // FIXME: GPR64 too

    llvm_unreachable("Invalid access!");
    return 0;
  }

  const MCExpr *getImm() const {
    assert((Kind == k_Immediate) && "Invalid access!");
    return Imm.Val;
  }

  int64_t getConstantImm() const {
    const MCExpr *Val = getImm();
    int64_t Value = 0;
    (void)Val->evaluateAsAbsolute(Value);
    return Value;
  }

  FgpuOperand *getMemBase() const {
    assert((Kind == k_Memory) && "Invalid access!");
    return Mem.Base;
  }

  const MCExpr *getMemOff() const {
    assert((Kind == k_Memory) && "Invalid access!");
    return Mem.Off;
  }

  int64_t getConstantMemOff() const {
    return static_cast<const MCConstantExpr *>(getMemOff())->getValue();
  }

  const SmallVectorImpl<unsigned> &getRegList() const {
    assert((Kind == k_RegList) && "Invalid access!");
    return *(RegList.List);
  }

  static std::unique_ptr<FgpuOperand> CreateToken(StringRef Str, SMLoc S,
                                                  FgpuAsmParser &Parser) {
    auto Op = std::make_unique<FgpuOperand>(k_Token, Parser);
    Op->Tok.Data = Str.data();
    Op->Tok.Length = Str.size();
    Op->StartLoc = S;
    Op->EndLoc = S;
    return Op;
  }

  /// Create a numeric register (e.g. $1). The exact register remains
  /// unresolved until an instruction successfully matches
  static std::unique_ptr<FgpuOperand>
  createNumericReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
                   SMLoc S, SMLoc E, FgpuAsmParser &Parser) {
    LLVM_DEBUG(dbgs() << "createNumericReg(" << Index << ", ...)\n");
    return CreateReg(Index, Str, RegKind_Numeric, RegInfo, S, E, Parser);
  }

  /// Create a register that is definitely a GPR.
  /// This is typically only used for named registers such as $gp.
  static std::unique_ptr<FgpuOperand>
  createGPRReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
               SMLoc S, SMLoc E, FgpuAsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_GPR, RegInfo, S, E, Parser);
  }

  /// Create a register that is definitely a FGR.
  /// This is typically only used for named registers such as $f0.
  static std::unique_ptr<FgpuOperand>
  createFGRReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
               SMLoc S, SMLoc E, FgpuAsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_FGR, RegInfo, S, E, Parser);
  }

  /// Create a register that is definitely a HWReg.
  /// This is typically only used for named registers such as $hwr_cpunum.
  static std::unique_ptr<FgpuOperand>
  createHWRegsReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
                  SMLoc S, SMLoc E, FgpuAsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_HWRegs, RegInfo, S, E, Parser);
  }

  /// Create a register that is definitely an FCC.
  /// This is typically only used for named registers such as $fcc0.
  static std::unique_ptr<FgpuOperand>
  createFCCReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
               SMLoc S, SMLoc E, FgpuAsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_FCC, RegInfo, S, E, Parser);
  }

  /// Create a register that is definitely an ACC.
  /// This is typically only used for named registers such as $ac0.
  static std::unique_ptr<FgpuOperand>
  createACCReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
               SMLoc S, SMLoc E, FgpuAsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_ACC, RegInfo, S, E, Parser);
  }

  /// Create a register that is definitely an MSA128.
  /// This is typically only used for named registers such as $w0.
  static std::unique_ptr<FgpuOperand>
  createMSA128Reg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
                  SMLoc S, SMLoc E, FgpuAsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_MSA128, RegInfo, S, E, Parser);
  }

  /// Create a register that is definitely an MSACtrl.
  /// This is typically only used for named registers such as $msaaccess.
  static std::unique_ptr<FgpuOperand>
  createMSACtrlReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
                   SMLoc S, SMLoc E, FgpuAsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_MSACtrl, RegInfo, S, E, Parser);
  }

  static std::unique_ptr<FgpuOperand>
  CreateImm(const MCExpr *Val, SMLoc S, SMLoc E, FgpuAsmParser &Parser) {
    auto Op = std::make_unique<FgpuOperand>(k_Immediate, Parser);
    Op->Imm.Val = Val;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static std::unique_ptr<FgpuOperand>
  CreateMem(std::unique_ptr<FgpuOperand> Base, const MCExpr *Off, SMLoc S,
            SMLoc E, FgpuAsmParser &Parser) {
    auto Op = std::make_unique<FgpuOperand>(k_Memory, Parser);
    Op->Mem.Base = Base.release();
    Op->Mem.Off = Off;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static std::unique_ptr<FgpuOperand>
  CreateRegList(SmallVectorImpl<unsigned> &Regs, SMLoc StartLoc, SMLoc EndLoc,
                FgpuAsmParser &Parser) {
    assert(Regs.size() > 0 && "Empty list not allowed");

    auto Op = std::make_unique<FgpuOperand>(k_RegList, Parser);
    Op->RegList.List = new SmallVector<unsigned, 10>(Regs.begin(), Regs.end());
    Op->StartLoc = StartLoc;
    Op->EndLoc = EndLoc;
    return Op;
  }

 bool isGPRZeroAsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_GPR && RegIdx.Index == 0;
  }

 bool isGPRNonZeroAsmReg() const {
   return isRegIdx() && RegIdx.Kind & RegKind_GPR && RegIdx.Index > 0 &&
          RegIdx.Index <= 31;
  }

  bool isGPRAsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_GPR && RegIdx.Index <= 31;
  }

  bool isMM16AsmReg() const {
    if (!(isRegIdx() && RegIdx.Kind))
      return false;
    return ((RegIdx.Index >= 2 && RegIdx.Index <= 7)
            || RegIdx.Index == 16 || RegIdx.Index == 17);

  }
  bool isMM16AsmRegZero() const {
    if (!(isRegIdx() && RegIdx.Kind))
      return false;
    return (RegIdx.Index == 0 ||
            (RegIdx.Index >= 2 && RegIdx.Index <= 7) ||
            RegIdx.Index == 17);
  }

  bool isMM16AsmRegMoveP() const {
    if (!(isRegIdx() && RegIdx.Kind))
      return false;
    return (RegIdx.Index == 0 || (RegIdx.Index >= 2 && RegIdx.Index <= 3) ||
      (RegIdx.Index >= 16 && RegIdx.Index <= 20));
  }

  bool isMM16AsmRegMovePPairFirst() const {
    if (!(isRegIdx() && RegIdx.Kind))
      return false;
    return RegIdx.Index >= 4 && RegIdx.Index <= 6;
  }

  bool isMM16AsmRegMovePPairSecond() const {
    if (!(isRegIdx() && RegIdx.Kind))
      return false;
    return (RegIdx.Index == 21 || RegIdx.Index == 22 ||
      (RegIdx.Index >= 5 && RegIdx.Index <= 7));
  }

  bool isFGRAsmReg() const {
    // AFGR64 is $0-$15 but we handle this in getAFGR64()
    return isRegIdx() && RegIdx.Kind & RegKind_FGR && RegIdx.Index <= 31;
  }

  bool isStrictlyFGRAsmReg() const {
    // AFGR64 is $0-$15 but we handle this in getAFGR64()
    return isRegIdx() && RegIdx.Kind == RegKind_FGR && RegIdx.Index <= 31;
  }

  bool isHWRegsAsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_HWRegs && RegIdx.Index <= 31;
  }

  bool isCCRAsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_CCR && RegIdx.Index <= 31;
  }

  bool isFCCAsmReg() const {
    if (!(isRegIdx() && RegIdx.Kind & RegKind_FCC))
      return false;
    return RegIdx.Index <= 7;
  }

  bool isACCAsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_ACC && RegIdx.Index <= 3;
  }

  bool isCOP0AsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_COP0 && RegIdx.Index <= 31;
  }

  bool isCOP2AsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_COP2 && RegIdx.Index <= 31;
  }

  bool isCOP3AsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_COP3 && RegIdx.Index <= 31;
  }

  bool isMSA128AsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_MSA128 && RegIdx.Index <= 31;
  }

  bool isMSACtrlAsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_MSACtrl && RegIdx.Index <= 7;
  }

  /// getStartLoc - Get the location of the first token of this operand.
  SMLoc getStartLoc() const override { return StartLoc; }
  /// getEndLoc - Get the location of the last token of this operand.
  SMLoc getEndLoc() const override { return EndLoc; }

  void print(raw_ostream &OS) const override {
    switch (Kind) {
    case k_Immediate:
      OS << "Imm<";
      OS << *Imm.Val;
      OS << ">";
      break;
    case k_Memory:
      OS << "Mem<";
      Mem.Base->print(OS);
      OS << ", ";
      OS << *Mem.Off;
      OS << ">";
      break;
    case k_RegisterIndex:
      OS << "RegIdx<" << RegIdx.Index << ":" << RegIdx.Kind << ", "
         << StringRef(RegIdx.Tok.Data, RegIdx.Tok.Length) << ">";
      break;
    case k_Token:
      OS << getToken();
      break;
    case k_RegList:
      OS << "RegList< ";
      for (auto Reg : (*RegList.List))
        OS << Reg << " ";
      OS <<  ">";
      break;
    }
  }

  bool isValidForTie(const FgpuOperand &Other) const {
    if (Kind != Other.Kind)
      return false;

    switch (Kind) {
    default:
      llvm_unreachable("Unexpected kind");
      return false;
    case k_RegisterIndex: {
      StringRef Token(RegIdx.Tok.Data, RegIdx.Tok.Length);
      StringRef OtherToken(Other.RegIdx.Tok.Data, Other.RegIdx.Tok.Length);
      return Token == OtherToken;
    }
    }
  }
}; // class FgpuOperand

} // end anonymous namespace

namespace llvm {

extern const MCInstrDesc FgpuInsts[];

} // end namespace llvm

static const MCInstrDesc &getInstDesc(unsigned Opcode) {
  return FgpuInsts[Opcode];
}

static bool hasShortDelaySlot(MCInst &Inst) {
  switch (Inst.getOpcode()) {
    case Fgpu::BEQ_MM:
    case Fgpu::BNE_MM:
    case Fgpu::BLTZ_MM:
    case Fgpu::BGEZ_MM:
    case Fgpu::BLEZ_MM:
    case Fgpu::BGTZ_MM:
    case Fgpu::JRC16_MM:
    case Fgpu::JALS_MM:
    case Fgpu::JALRS_MM:
    case Fgpu::JALRS16_MM:
    case Fgpu::BGEZALS_MM:
    case Fgpu::BLTZALS_MM:
      return true;
    case Fgpu::J_MM:
      return !Inst.getOperand(0).isReg();
    default:
      return false;
  }
}

static const MCSymbol *getSingleMCSymbol(const MCExpr *Expr) {
  if (const MCSymbolRefExpr *SRExpr = dyn_cast<MCSymbolRefExpr>(Expr)) {
    return &SRExpr->getSymbol();
  }

  if (const MCBinaryExpr *BExpr = dyn_cast<MCBinaryExpr>(Expr)) {
    const MCSymbol *LHSSym = getSingleMCSymbol(BExpr->getLHS());
    const MCSymbol *RHSSym = getSingleMCSymbol(BExpr->getRHS());

    if (LHSSym)
      return LHSSym;

    if (RHSSym)
      return RHSSym;

    return nullptr;
  }

  if (const MCUnaryExpr *UExpr = dyn_cast<MCUnaryExpr>(Expr))
    return getSingleMCSymbol(UExpr->getSubExpr());

  return nullptr;
}

static unsigned countMCSymbolRefExpr(const MCExpr *Expr) {
  if (isa<MCSymbolRefExpr>(Expr))
    return 1;

  if (const MCBinaryExpr *BExpr = dyn_cast<MCBinaryExpr>(Expr))
    return countMCSymbolRefExpr(BExpr->getLHS()) +
           countMCSymbolRefExpr(BExpr->getRHS());

  if (const MCUnaryExpr *UExpr = dyn_cast<MCUnaryExpr>(Expr))
    return countMCSymbolRefExpr(UExpr->getSubExpr());

  return 0;
}

static bool isEvaluated(const MCExpr *Expr) {
  switch (Expr->getKind()) {
  case MCExpr::Constant:
    return true;
  case MCExpr::SymbolRef:
    return (cast<MCSymbolRefExpr>(Expr)->getKind() != MCSymbolRefExpr::VK_None);
  case MCExpr::Binary: {
    const MCBinaryExpr *BE = cast<MCBinaryExpr>(Expr);
    if (!isEvaluated(BE->getLHS()))
      return false;
    return isEvaluated(BE->getRHS());
  }
  case MCExpr::Unary:
    return isEvaluated(cast<MCUnaryExpr>(Expr)->getSubExpr());
  case MCExpr::Target:
    return true;
  }
  return false;
}

static bool needsExpandMemInst(MCInst &Inst) {
  const MCInstrDesc &MCID = getInstDesc(Inst.getOpcode());

  unsigned NumOp = MCID.getNumOperands();
  if (NumOp != 3 && NumOp != 4)
    return false;

  const MCOperandInfo &OpInfo = MCID.OpInfo[NumOp - 1];
  if (OpInfo.OperandType != MCOI::OPERAND_MEMORY &&
      OpInfo.OperandType != MCOI::OPERAND_UNKNOWN &&
      OpInfo.OperandType != FgpuII::OPERAND_MEM_SIMM9)
    return false;

  MCOperand &Op = Inst.getOperand(NumOp - 1);
  if (Op.isImm()) {
    if (OpInfo.OperandType == FgpuII::OPERAND_MEM_SIMM9)
      return !isInt<9>(Op.getImm());
    // Offset can't exceed 16bit value.
    return !isInt<16>(Op.getImm());
  }

  if (Op.isExpr()) {
    const MCExpr *Expr = Op.getExpr();
    if (Expr->getKind() != MCExpr::SymbolRef)
      return !isEvaluated(Expr);

    // Expand symbol.
    const MCSymbolRefExpr *SR = static_cast<const MCSymbolRefExpr *>(Expr);
    return SR->getKind() == MCSymbolRefExpr::VK_None;
  }

  return false;
}

bool FgpuAsmParser::processInstruction(MCInst &Inst, SMLoc IDLoc,
                                       MCStreamer &Out,
                                       const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  const unsigned Opcode = Inst.getOpcode();
  const MCInstrDesc &MCID = getInstDesc(Opcode);
  bool ExpandedJalSym = false;

  Inst.setLoc(IDLoc);

  if (MCID.isBranch() || MCID.isCall()) {
    MCOperand Offset;

    switch (Opcode) {
    default:
      break;
    case Fgpu::BBIT0:
    case Fgpu::BBIT032:
    case Fgpu::BBIT1:
    case Fgpu::BBIT132:
      assert(hasCnFgpu() && "instruction only valid for octeon cpus");
      LLVM_FALLTHROUGH;

    case Fgpu::BEQ:
    case Fgpu::BNE:
    case Fgpu::BEQ_MM:
    case Fgpu::BNE_MM:
      assert(MCID.getNumOperands() == 3 && "unexpected number of operands");
      Offset = Inst.getOperand(2);
      if (!Offset.isImm())
        break; // We'll deal with this situation later on when applying fixups.
      if (!isIntN(inMicroFgpuMode() ? 17 : 18, Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (offsetToAlignment(Offset.getImm(),
                            (inMicroFgpuMode() ? Align(2) : Align(4))))
        return Error(IDLoc, "branch to misaligned address");
      break;
    case Fgpu::BGEZ:
    case Fgpu::BGTZ:
    case Fgpu::BLEZ:
    case Fgpu::BLTZ:
    case Fgpu::BGEZAL:
    case Fgpu::BLTZAL:
    case Fgpu::BC1F:
    case Fgpu::BC1T:
    case Fgpu::BGEZ_MM:
    case Fgpu::BGTZ_MM:
    case Fgpu::BLEZ_MM:
    case Fgpu::BLTZ_MM:
    case Fgpu::BGEZAL_MM:
    case Fgpu::BLTZAL_MM:
    case Fgpu::BC1F_MM:
    case Fgpu::BC1T_MM:
    case Fgpu::BC1EQZC_MMR6:
    case Fgpu::BC1NEZC_MMR6:
    case Fgpu::BC2EQZC_MMR6:
    case Fgpu::BC2NEZC_MMR6:
      assert(MCID.getNumOperands() == 2 && "unexpected number of operands");
      Offset = Inst.getOperand(1);
      if (!Offset.isImm())
        break; // We'll deal with this situation later on when applying fixups.
      if (!isIntN(inMicroFgpuMode() ? 17 : 18, Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (offsetToAlignment(Offset.getImm(),
                            (inMicroFgpuMode() ? Align(2) : Align(4))))
        return Error(IDLoc, "branch to misaligned address");
      break;
    case Fgpu::BGEC:    case Fgpu::BGEC_MMR6:
    case Fgpu::BLTC:    case Fgpu::BLTC_MMR6:
    case Fgpu::BGEUC:   case Fgpu::BGEUC_MMR6:
    case Fgpu::BLTUC:   case Fgpu::BLTUC_MMR6:
    case Fgpu::BEQC:    case Fgpu::BEQC_MMR6:
    case Fgpu::BNEC:    case Fgpu::BNEC_MMR6:
      assert(MCID.getNumOperands() == 3 && "unexpected number of operands");
      Offset = Inst.getOperand(2);
      if (!Offset.isImm())
        break; // We'll deal with this situation later on when applying fixups.
      if (!isIntN(18, Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (offsetToAlignment(Offset.getImm(), Align(4)))
        return Error(IDLoc, "branch to misaligned address");
      break;
    case Fgpu::BLEZC:   case Fgpu::BLEZC_MMR6:
    case Fgpu::BGEZC:   case Fgpu::BGEZC_MMR6:
    case Fgpu::BGTZC:   case Fgpu::BGTZC_MMR6:
    case Fgpu::BLTZC:   case Fgpu::BLTZC_MMR6:
      assert(MCID.getNumOperands() == 2 && "unexpected number of operands");
      Offset = Inst.getOperand(1);
      if (!Offset.isImm())
        break; // We'll deal with this situation later on when applying fixups.
      if (!isIntN(18, Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (offsetToAlignment(Offset.getImm(), Align(4)))
        return Error(IDLoc, "branch to misaligned address");
      break;
    case Fgpu::BEQZC:   case Fgpu::BEQZC_MMR6:
    case Fgpu::BNEZC:   case Fgpu::BNEZC_MMR6:
      assert(MCID.getNumOperands() == 2 && "unexpected number of operands");
      Offset = Inst.getOperand(1);
      if (!Offset.isImm())
        break; // We'll deal with this situation later on when applying fixups.
      if (!isIntN(23, Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (offsetToAlignment(Offset.getImm(), Align(4)))
        return Error(IDLoc, "branch to misaligned address");
      break;
    case Fgpu::BEQZ16_MM:
    case Fgpu::BEQZC16_MMR6:
    case Fgpu::BNEZ16_MM:
    case Fgpu::BNEZC16_MMR6:
      assert(MCID.getNumOperands() == 2 && "unexpected number of operands");
      Offset = Inst.getOperand(1);
      if (!Offset.isImm())
        break; // We'll deal with this situation later on when applying fixups.
      if (!isInt<8>(Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (offsetToAlignment(Offset.getImm(), Align(2)))
        return Error(IDLoc, "branch to misaligned address");
      break;
    }
  }

  // SSNOP is deprecated on FGPU32r6/FGPU64r6
  // We still accept it but it is a normal nop.
  if (hasFgpu32r6() && Opcode == Fgpu::SSNOP) {
    std::string ISA = hasFgpu64r6() ? "FGPU64r6" : "FGPU32r6";
    Warning(IDLoc, "ssnop is deprecated for " + ISA + " and is equivalent to a "
                                                      "nop instruction");
  }

  if (hasCnFgpu()) {
    MCOperand Opnd;
    int Imm;

    switch (Opcode) {
      default:
        break;

      case Fgpu::BBIT0:
      case Fgpu::BBIT032:
      case Fgpu::BBIT1:
      case Fgpu::BBIT132:
        assert(MCID.getNumOperands() == 3 && "unexpected number of operands");
        // The offset is handled above
        Opnd = Inst.getOperand(1);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < 0 || Imm > (Opcode == Fgpu::BBIT0 ||
                              Opcode == Fgpu::BBIT1 ? 63 : 31))
          return Error(IDLoc, "immediate operand value out of range");
        if (Imm > 31) {
          Inst.setOpcode(Opcode == Fgpu::BBIT0 ? Fgpu::BBIT032
                                               : Fgpu::BBIT132);
          Inst.getOperand(1).setImm(Imm - 32);
        }
        break;

      case Fgpu::SEQi:
      case Fgpu::SNEi:
        assert(MCID.getNumOperands() == 3 && "unexpected number of operands");
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (!isInt<10>(Imm))
          return Error(IDLoc, "immediate operand value out of range");
        break;
    }
  }

  // Warn on division by zero. We're checking here as all instructions get
  // processed here, not just the macros that need expansion.
  //
  // The FGPU backend models most of the divison instructions and macros as
  // three operand instructions. The pre-R6 divide instructions however have
  // two operands and explicitly define HI/LO as part of the instruction,
  // not in the operands.
  unsigned FirstOp = 1;
  unsigned SecondOp = 2;
  switch (Opcode) {
  default:
    break;
  case Fgpu::SDivIMacro:
  case Fgpu::UDivIMacro:
  case Fgpu::DSDivIMacro:
  case Fgpu::DUDivIMacro:
    if (Inst.getOperand(2).getImm() == 0) {
      if (Inst.getOperand(1).getReg() == Fgpu::ZERO ||
          Inst.getOperand(1).getReg() == Fgpu::ZERO_64)
        Warning(IDLoc, "dividing zero by zero");
      else
        Warning(IDLoc, "division by zero");
    }
    break;
  case Fgpu::DSDIV:
  case Fgpu::SDIV:
  case Fgpu::UDIV:
  case Fgpu::DUDIV:
  case Fgpu::UDIV_MM:
  case Fgpu::SDIV_MM:
    FirstOp = 0;
    SecondOp = 1;
    LLVM_FALLTHROUGH;
  case Fgpu::SDivMacro:
  case Fgpu::DSDivMacro:
  case Fgpu::UDivMacro:
  case Fgpu::DUDivMacro:
  case Fgpu::DIV:
  case Fgpu::DIVU:
  case Fgpu::DDIV:
  case Fgpu::DDIVU:
  case Fgpu::DIVU_MMR6:
  case Fgpu::DIV_MMR6:
    if (Inst.getOperand(SecondOp).getReg() == Fgpu::ZERO ||
        Inst.getOperand(SecondOp).getReg() == Fgpu::ZERO_64) {
      if (Inst.getOperand(FirstOp).getReg() == Fgpu::ZERO ||
          Inst.getOperand(FirstOp).getReg() == Fgpu::ZERO_64)
        Warning(IDLoc, "dividing zero by zero");
      else
        Warning(IDLoc, "division by zero");
    }
    break;
  }

  // For PIC code convert unconditional jump to unconditional branch.
  if ((Opcode == Fgpu::J || Opcode == Fgpu::J_MM) && inPicMode()) {
    MCInst BInst;
    BInst.setOpcode(inMicroFgpuMode() ? Fgpu::BEQ_MM : Fgpu::BEQ);
    BInst.addOperand(MCOperand::createReg(Fgpu::ZERO));
    BInst.addOperand(MCOperand::createReg(Fgpu::ZERO));
    BInst.addOperand(Inst.getOperand(0));
    Inst = BInst;
  }

  // This expansion is not in a function called by tryExpandInstruction()
  // because the pseudo-instruction doesn't have a distinct opcode.
  if ((Opcode == Fgpu::JAL || Opcode == Fgpu::JAL_MM) && inPicMode()) {
    warnIfNoMacro(IDLoc);

    const MCExpr *JalExpr = Inst.getOperand(0).getExpr();

    // We can do this expansion if there's only 1 symbol in the argument
    // expression.
    if (countMCSymbolRefExpr(JalExpr) > 1)
      return Error(IDLoc, "jal doesn't support multiple symbols in PIC mode");

    // FIXME: This is checking the expression can be handled by the later stages
    //        of the assembler. We ought to leave it to those later stages.
    const MCSymbol *JalSym = getSingleMCSymbol(JalExpr);

    if (expandLoadAddress(Fgpu::T9, Fgpu::NoRegister, Inst.getOperand(0),
                          !isGP64bit(), IDLoc, Out, STI))
      return true;

    MCInst JalrInst;
    if (inMicroFgpuMode())
      JalrInst.setOpcode(IsCpRestoreSet ? Fgpu::JALRS_MM : Fgpu::JALR_MM);
    else
      JalrInst.setOpcode(Fgpu::JALR);
    JalrInst.addOperand(MCOperand::createReg(Fgpu::RA));
    JalrInst.addOperand(MCOperand::createReg(Fgpu::T9));

    if (isJalrRelocAvailable(JalExpr)) {
      // As an optimization hint for the linker, before the JALR we add:
      // .reloc tmplabel, R_{MICRO}FGPU_JALR, symbol
      // tmplabel:
      MCSymbol *TmpLabel = getContext().createTempSymbol();
      const MCExpr *TmpExpr = MCSymbolRefExpr::create(TmpLabel, getContext());
      const MCExpr *RelocJalrExpr =
          MCSymbolRefExpr::create(JalSym, MCSymbolRefExpr::VK_None,
                                  getContext(), IDLoc);

      TOut.getStreamer().emitRelocDirective(
          *TmpExpr, inMicroFgpuMode() ? "R_MICROFGPU_JALR" : "R_FGPU_JALR",
          RelocJalrExpr, IDLoc, *STI);
      TOut.getStreamer().emitLabel(TmpLabel);
    }

    Inst = JalrInst;
    ExpandedJalSym = true;
  }

  if (MCID.mayLoad() || MCID.mayStore()) {
    // Check the offset of memory operand, if it is a symbol
    // reference or immediate we may have to expand instructions.
    if (needsExpandMemInst(Inst)) {
      const MCInstrDesc &MCID = getInstDesc(Inst.getOpcode());
      switch (MCID.OpInfo[MCID.getNumOperands() - 1].OperandType) {
      case FgpuII::OPERAND_MEM_SIMM9:
        expandMem9Inst(Inst, IDLoc, Out, STI, MCID.mayLoad());
        break;
      default:
        expandMem16Inst(Inst, IDLoc, Out, STI, MCID.mayLoad());
        break;
      }
      return getParser().hasPendingError();
    }
  }

  if (inMicroFgpuMode()) {
    if (MCID.mayLoad() && Opcode != Fgpu::LWP_MM) {
      // Try to create 16-bit GP relative load instruction.
      for (unsigned i = 0; i < MCID.getNumOperands(); i++) {
        const MCOperandInfo &OpInfo = MCID.OpInfo[i];
        if ((OpInfo.OperandType == MCOI::OPERAND_MEMORY) ||
            (OpInfo.OperandType == MCOI::OPERAND_UNKNOWN)) {
          MCOperand &Op = Inst.getOperand(i);
          if (Op.isImm()) {
            int MemOffset = Op.getImm();
            MCOperand &DstReg = Inst.getOperand(0);
            MCOperand &BaseReg = Inst.getOperand(1);
            if (isInt<9>(MemOffset) && (MemOffset % 4 == 0) &&
                getContext().getRegisterInfo()->getRegClass(
                  Fgpu::GPRMM16RegClassID).contains(DstReg.getReg()) &&
                (BaseReg.getReg() == Fgpu::GP ||
                BaseReg.getReg() == Fgpu::GP_64)) {

              TOut.emitRRI(Fgpu::LWGP_MM, DstReg.getReg(), Fgpu::GP, MemOffset,
                           IDLoc, STI);
              return false;
            }
          }
        }
      } // for
    }   // if load

    // TODO: Handle this with the AsmOperandClass.PredicateMethod.

    MCOperand Opnd;
    int Imm;

    switch (Opcode) {
      default:
        break;
      case Fgpu::ADDIUSP_MM:
        Opnd = Inst.getOperand(0);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < -1032 || Imm > 1028 || (Imm < 8 && Imm > -12) ||
            Imm % 4 != 0)
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case Fgpu::SLL16_MM:
      case Fgpu::SRL16_MM:
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < 1 || Imm > 8)
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case Fgpu::LI16_MM:
        Opnd = Inst.getOperand(1);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < -1 || Imm > 126)
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case Fgpu::ADDIUR2_MM:
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (!(Imm == 1 || Imm == -1 ||
              ((Imm % 4 == 0) && Imm < 28 && Imm > 0)))
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case Fgpu::ANDI16_MM:
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (!(Imm == 128 || (Imm >= 1 && Imm <= 4) || Imm == 7 || Imm == 8 ||
              Imm == 15 || Imm == 16 || Imm == 31 || Imm == 32 || Imm == 63 ||
              Imm == 64 || Imm == 255 || Imm == 32768 || Imm == 65535))
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case Fgpu::LBU16_MM:
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < -1 || Imm > 14)
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case Fgpu::SB16_MM:
      case Fgpu::SB16_MMR6:
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < 0 || Imm > 15)
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case Fgpu::LHU16_MM:
      case Fgpu::SH16_MM:
      case Fgpu::SH16_MMR6:
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < 0 || Imm > 30 || (Imm % 2 != 0))
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case Fgpu::LW16_MM:
      case Fgpu::SW16_MM:
      case Fgpu::SW16_MMR6:
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < 0 || Imm > 60 || (Imm % 4 != 0))
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case Fgpu::ADDIUPC_MM:
        Opnd = Inst.getOperand(1);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if ((Imm % 4 != 0) || !isInt<25>(Imm))
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case Fgpu::LWP_MM:
      case Fgpu::SWP_MM:
        if (Inst.getOperand(0).getReg() == Fgpu::RA)
          return Error(IDLoc, "invalid operand for instruction");
        break;
      case Fgpu::MOVEP_MM:
      case Fgpu::MOVEP_MMR6: {
        unsigned R0 = Inst.getOperand(0).getReg();
        unsigned R1 = Inst.getOperand(1).getReg();
        bool RegPair = ((R0 == Fgpu::A1 && R1 == Fgpu::A2) ||
                        (R0 == Fgpu::A1 && R1 == Fgpu::A3) ||
                        (R0 == Fgpu::A2 && R1 == Fgpu::A3) ||
                        (R0 == Fgpu::A0 && R1 == Fgpu::S5) ||
                        (R0 == Fgpu::A0 && R1 == Fgpu::S6) ||
                        (R0 == Fgpu::A0 && R1 == Fgpu::A1) ||
                        (R0 == Fgpu::A0 && R1 == Fgpu::A2) ||
                        (R0 == Fgpu::A0 && R1 == Fgpu::A3));
        if (!RegPair)
          return Error(IDLoc, "invalid operand for instruction");
        break;
      }
    }
  }

  bool FillDelaySlot =
      MCID.hasDelaySlot() && AssemblerOptions.back()->isReorder();
  if (FillDelaySlot)
    TOut.emitDirectiveSetNoReorder();

  MacroExpanderResultTy ExpandResult =
      tryExpandInstruction(Inst, IDLoc, Out, STI);
  switch (ExpandResult) {
  case MER_NotAMacro:
    Out.emitInstruction(Inst, *STI);
    break;
  case MER_Success:
    break;
  case MER_Fail:
    return true;
  }

  // We know we emitted an instruction on the MER_NotAMacro or MER_Success path.
  // If we're in microFGPU mode then we must also set EF_FGPU_MICROFGPU.
  if (inMicroFgpuMode()) {
    TOut.setUsesMicroFgpu();
    TOut.updateABIInfo(*this);
  }

  // If this instruction has a delay slot and .set reorder is active,
  // emit a NOP after it.
  if (FillDelaySlot) {
    TOut.emitEmptyDelaySlot(hasShortDelaySlot(Inst), IDLoc, STI);
    TOut.emitDirectiveSetReorder();
  }

  if ((Opcode == Fgpu::JalOneReg || Opcode == Fgpu::JalTwoReg ||
       ExpandedJalSym) &&
      isPicAndNotNxxAbi()) {
    if (IsCpRestoreSet) {
      // We need a NOP between the JALR and the LW:
      // If .set reorder has been used, we've already emitted a NOP.
      // If .set noreorder has been used, we need to emit a NOP at this point.
      if (!AssemblerOptions.back()->isReorder())
        TOut.emitEmptyDelaySlot(hasShortDelaySlot(Inst), IDLoc,
                                STI);

      // Load the $gp from the stack.
      TOut.emitGPRestore(CpRestoreOffset, IDLoc, STI);
    } else
      Warning(IDLoc, "no .cprestore used in PIC mode");
  }

  return false;
}

FgpuAsmParser::MacroExpanderResultTy
FgpuAsmParser::tryExpandInstruction(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                    const MCSubtargetInfo *STI) {
  switch (Inst.getOpcode()) {
  default:
    return MER_NotAMacro;
  case Fgpu::LoadImm32:
    return expandLoadImm(Inst, true, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::LoadImm64:
    return expandLoadImm(Inst, false, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::LoadAddrImm32:
  case Fgpu::LoadAddrImm64:
    assert(Inst.getOperand(0).isReg() && "expected register operand kind");
    assert((Inst.getOperand(1).isImm() || Inst.getOperand(1).isExpr()) &&
           "expected immediate operand kind");

    return expandLoadAddress(Inst.getOperand(0).getReg(), Fgpu::NoRegister,
                             Inst.getOperand(1),
                             Inst.getOpcode() == Fgpu::LoadAddrImm32, IDLoc,
                             Out, STI)
               ? MER_Fail
               : MER_Success;
  case Fgpu::LoadAddrReg32:
  case Fgpu::LoadAddrReg64:
    assert(Inst.getOperand(0).isReg() && "expected register operand kind");
    assert(Inst.getOperand(1).isReg() && "expected register operand kind");
    assert((Inst.getOperand(2).isImm() || Inst.getOperand(2).isExpr()) &&
           "expected immediate operand kind");

    return expandLoadAddress(Inst.getOperand(0).getReg(),
                             Inst.getOperand(1).getReg(), Inst.getOperand(2),
                             Inst.getOpcode() == Fgpu::LoadAddrReg32, IDLoc,
                             Out, STI)
               ? MER_Fail
               : MER_Success;
  case Fgpu::B_MM_Pseudo:
  case Fgpu::B_MMR6_Pseudo:
    return expandUncondBranchMMPseudo(Inst, IDLoc, Out, STI) ? MER_Fail
                                                             : MER_Success;
  case Fgpu::SWM_MM:
  case Fgpu::LWM_MM:
    return expandLoadStoreMultiple(Inst, IDLoc, Out, STI) ? MER_Fail
                                                          : MER_Success;
  case Fgpu::JalOneReg:
  case Fgpu::JalTwoReg:
    return expandJalWithRegs(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::BneImm:
  case Fgpu::BeqImm:
  case Fgpu::BEQLImmMacro:
  case Fgpu::BNELImmMacro:
    return expandBranchImm(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::BLT:
  case Fgpu::BLE:
  case Fgpu::BGE:
  case Fgpu::BGT:
  case Fgpu::BLTU:
  case Fgpu::BLEU:
  case Fgpu::BGEU:
  case Fgpu::BGTU:
  case Fgpu::BLTL:
  case Fgpu::BLEL:
  case Fgpu::BGEL:
  case Fgpu::BGTL:
  case Fgpu::BLTUL:
  case Fgpu::BLEUL:
  case Fgpu::BGEUL:
  case Fgpu::BGTUL:
  case Fgpu::BLTImmMacro:
  case Fgpu::BLEImmMacro:
  case Fgpu::BGEImmMacro:
  case Fgpu::BGTImmMacro:
  case Fgpu::BLTUImmMacro:
  case Fgpu::BLEUImmMacro:
  case Fgpu::BGEUImmMacro:
  case Fgpu::BGTUImmMacro:
  case Fgpu::BLTLImmMacro:
  case Fgpu::BLELImmMacro:
  case Fgpu::BGELImmMacro:
  case Fgpu::BGTLImmMacro:
  case Fgpu::BLTULImmMacro:
  case Fgpu::BLEULImmMacro:
  case Fgpu::BGEULImmMacro:
  case Fgpu::BGTULImmMacro:
    return expandCondBranches(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::SDivMacro:
  case Fgpu::SDivIMacro:
  case Fgpu::SRemMacro:
  case Fgpu::SRemIMacro:
    return expandDivRem(Inst, IDLoc, Out, STI, false, true) ? MER_Fail
                                                            : MER_Success;
  case Fgpu::DSDivMacro:
  case Fgpu::DSDivIMacro:
  case Fgpu::DSRemMacro:
  case Fgpu::DSRemIMacro:
    return expandDivRem(Inst, IDLoc, Out, STI, true, true) ? MER_Fail
                                                           : MER_Success;
  case Fgpu::UDivMacro:
  case Fgpu::UDivIMacro:
  case Fgpu::URemMacro:
  case Fgpu::URemIMacro:
    return expandDivRem(Inst, IDLoc, Out, STI, false, false) ? MER_Fail
                                                             : MER_Success;
  case Fgpu::DUDivMacro:
  case Fgpu::DUDivIMacro:
  case Fgpu::DURemMacro:
  case Fgpu::DURemIMacro:
    return expandDivRem(Inst, IDLoc, Out, STI, true, false) ? MER_Fail
                                                            : MER_Success;
  case Fgpu::PseudoTRUNC_W_S:
    return expandTrunc(Inst, false, false, IDLoc, Out, STI) ? MER_Fail
                                                            : MER_Success;
  case Fgpu::PseudoTRUNC_W_D32:
    return expandTrunc(Inst, true, false, IDLoc, Out, STI) ? MER_Fail
                                                           : MER_Success;
  case Fgpu::PseudoTRUNC_W_D:
    return expandTrunc(Inst, true, true, IDLoc, Out, STI) ? MER_Fail
                                                          : MER_Success;

  case Fgpu::LoadImmSingleGPR:
    return expandLoadSingleImmToGPR(Inst, IDLoc, Out, STI) ? MER_Fail
                                                           : MER_Success;
  case Fgpu::LoadImmSingleFGR:
    return expandLoadSingleImmToFPR(Inst, IDLoc, Out, STI) ? MER_Fail
                                                           : MER_Success;
  case Fgpu::LoadImmDoubleGPR:
    return expandLoadDoubleImmToGPR(Inst, IDLoc, Out, STI) ? MER_Fail
                                                           : MER_Success;
  case Fgpu::LoadImmDoubleFGR:
    return expandLoadDoubleImmToFPR(Inst, true, IDLoc, Out, STI) ? MER_Fail
                                                                 : MER_Success;
  case Fgpu::LoadImmDoubleFGR_32:
    return expandLoadDoubleImmToFPR(Inst, false, IDLoc, Out, STI) ? MER_Fail
                                                                  : MER_Success;

  case Fgpu::Ulh:
    return expandUlh(Inst, true, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::Ulhu:
    return expandUlh(Inst, false, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::Ush:
    return expandUsh(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::Ulw:
  case Fgpu::Usw:
    return expandUxw(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::NORImm:
  case Fgpu::NORImm64:
    return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::SGE:
  case Fgpu::SGEU:
    return expandSge(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::SGEImm:
  case Fgpu::SGEUImm:
  case Fgpu::SGEImm64:
  case Fgpu::SGEUImm64:
    return expandSgeImm(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::SGTImm:
  case Fgpu::SGTUImm:
  case Fgpu::SGTImm64:
  case Fgpu::SGTUImm64:
    return expandSgtImm(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::SLE:
  case Fgpu::SLEU:
    return expandSle(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::SLEImm:
  case Fgpu::SLEUImm:
  case Fgpu::SLEImm64:
  case Fgpu::SLEUImm64:
    return expandSleImm(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::SLTImm64:
    if (isInt<16>(Inst.getOperand(2).getImm())) {
      Inst.setOpcode(Fgpu::SLTi64);
      return MER_NotAMacro;
    }
    return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::SLTUImm64:
    if (isInt<16>(Inst.getOperand(2).getImm())) {
      Inst.setOpcode(Fgpu::SLTiu64);
      return MER_NotAMacro;
    }
    return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::ADDi:   case Fgpu::ADDi_MM:
  case Fgpu::ADDiu:  case Fgpu::ADDiu_MM:
  case Fgpu::SLTi:   case Fgpu::SLTi_MM:
  case Fgpu::SLTiu:  case Fgpu::SLTiu_MM:
    if ((Inst.getNumOperands() == 3) && Inst.getOperand(0).isReg() &&
        Inst.getOperand(1).isReg() && Inst.getOperand(2).isImm()) {
      int64_t ImmValue = Inst.getOperand(2).getImm();
      if (isInt<16>(ImmValue))
        return MER_NotAMacro;
      return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail
                                                         : MER_Success;
    }
    return MER_NotAMacro;
  case Fgpu::ANDi:  case Fgpu::ANDi_MM:  case Fgpu::ANDi64:
  case Fgpu::ORi:   case Fgpu::ORi_MM:   case Fgpu::ORi64:
  case Fgpu::XORi:  case Fgpu::XORi_MM:  case Fgpu::XORi64:
    if ((Inst.getNumOperands() == 3) && Inst.getOperand(0).isReg() &&
        Inst.getOperand(1).isReg() && Inst.getOperand(2).isImm()) {
      int64_t ImmValue = Inst.getOperand(2).getImm();
      if (isUInt<16>(ImmValue))
        return MER_NotAMacro;
      return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail
                                                         : MER_Success;
    }
    return MER_NotAMacro;
  case Fgpu::ROL:
  case Fgpu::ROR:
    return expandRotation(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::ROLImm:
  case Fgpu::RORImm:
    return expandRotationImm(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::DROL:
  case Fgpu::DROR:
    return expandDRotation(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::DROLImm:
  case Fgpu::DRORImm:
    return expandDRotationImm(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::ABSMacro:
    return expandAbs(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::MULImmMacro:
  case Fgpu::DMULImmMacro:
    return expandMulImm(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::MULOMacro:
  case Fgpu::DMULOMacro:
    return expandMulO(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::MULOUMacro:
  case Fgpu::DMULOUMacro:
    return expandMulOU(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::DMULMacro:
    return expandDMULMacro(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::LDMacro:
  case Fgpu::SDMacro:
    return expandLoadStoreDMacro(Inst, IDLoc, Out, STI,
                                 Inst.getOpcode() == Fgpu::LDMacro)
               ? MER_Fail
               : MER_Success;
  case Fgpu::SDC1_M1:
    return expandStoreDM1Macro(Inst, IDLoc, Out, STI)
               ? MER_Fail
               : MER_Success;
  case Fgpu::SEQMacro:
    return expandSeq(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::SEQIMacro:
    return expandSeqI(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::SNEMacro:
    return expandSne(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::SNEIMacro:
    return expandSneI(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::MFTC0:   case Fgpu::MTTC0:
  case Fgpu::MFTGPR:  case Fgpu::MTTGPR:
  case Fgpu::MFTLO:   case Fgpu::MTTLO:
  case Fgpu::MFTHI:   case Fgpu::MTTHI:
  case Fgpu::MFTACX:  case Fgpu::MTTACX:
  case Fgpu::MFTDSP:  case Fgpu::MTTDSP:
  case Fgpu::MFTC1:   case Fgpu::MTTC1:
  case Fgpu::MFTHC1:  case Fgpu::MTTHC1:
  case Fgpu::CFTC1:   case Fgpu::CTTC1:
    return expandMXTRAlias(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::SaaAddr:
  case Fgpu::SaadAddr:
    return expandSaaAddr(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  }
}

bool FgpuAsmParser::expandJalWithRegs(MCInst &Inst, SMLoc IDLoc,
                                      MCStreamer &Out,
                                      const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();

  // Create a JALR instruction which is going to replace the pseudo-JAL.
  MCInst JalrInst;
  JalrInst.setLoc(IDLoc);
  const MCOperand FirstRegOp = Inst.getOperand(0);
  const unsigned Opcode = Inst.getOpcode();

  if (Opcode == Fgpu::JalOneReg) {
    // jal $rs => jalr $rs
    if (IsCpRestoreSet && inMicroFgpuMode()) {
      JalrInst.setOpcode(Fgpu::JALRS16_MM);
      JalrInst.addOperand(FirstRegOp);
    } else if (inMicroFgpuMode()) {
      JalrInst.setOpcode(hasFgpu32r6() ? Fgpu::JALRC16_MMR6 : Fgpu::JALR16_MM);
      JalrInst.addOperand(FirstRegOp);
    } else {
      JalrInst.setOpcode(Fgpu::JALR);
      JalrInst.addOperand(MCOperand::createReg(Fgpu::RA));
      JalrInst.addOperand(FirstRegOp);
    }
  } else if (Opcode == Fgpu::JalTwoReg) {
    // jal $rd, $rs => jalr $rd, $rs
    if (IsCpRestoreSet && inMicroFgpuMode())
      JalrInst.setOpcode(Fgpu::JALRS_MM);
    else
      JalrInst.setOpcode(inMicroFgpuMode() ? Fgpu::JALR_MM : Fgpu::JALR);
    JalrInst.addOperand(FirstRegOp);
    const MCOperand SecondRegOp = Inst.getOperand(1);
    JalrInst.addOperand(SecondRegOp);
  }
  Out.emitInstruction(JalrInst, *STI);

  // If .set reorder is active and branch instruction has a delay slot,
  // emit a NOP after it.
  const MCInstrDesc &MCID = getInstDesc(JalrInst.getOpcode());
  if (MCID.hasDelaySlot() && AssemblerOptions.back()->isReorder())
    TOut.emitEmptyDelaySlot(hasShortDelaySlot(JalrInst), IDLoc,
                            STI);

  return false;
}

/// Can the value be represented by a unsigned N-bit value and a shift left?
template <unsigned N> static bool isShiftedUIntAtAnyPosition(uint64_t x) {
  unsigned BitNum = findFirstSet(x);

  return (x == x >> BitNum << BitNum) && isUInt<N>(x >> BitNum);
}

/// Load (or add) an immediate into a register.
///
/// @param ImmValue     The immediate to load.
/// @param DstReg       The register that will hold the immediate.
/// @param SrcReg       A register to add to the immediate or Fgpu::NoRegister
///                     for a simple initialization.
/// @param Is32BitImm   Is ImmValue 32-bit or 64-bit?
/// @param IsAddress    True if the immediate represents an address. False if it
///                     is an integer.
/// @param IDLoc        Location of the immediate in the source file.
bool FgpuAsmParser::loadImmediate(int64_t ImmValue, unsigned DstReg,
                                  unsigned SrcReg, bool Is32BitImm,
                                  bool IsAddress, SMLoc IDLoc, MCStreamer &Out,
                                  const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();

  if (!Is32BitImm && !isGP64bit()) {
    Error(IDLoc, "instruction requires a 64-bit architecture");
    return true;
  }

  if (Is32BitImm) {
    if (isInt<32>(ImmValue) || isUInt<32>(ImmValue)) {
      // Sign extend up to 64-bit so that the predicates match the hardware
      // behaviour. In particular, isInt<16>(0xffff8000) and similar should be
      // true.
      ImmValue = SignExtend64<32>(ImmValue);
    } else {
      Error(IDLoc, "instruction requires a 32-bit immediate");
      return true;
    }
  }

  unsigned ZeroReg = IsAddress ? ABI.GetNullPtr() : ABI.GetZeroReg();
  unsigned AdduOp = !Is32BitImm ? Fgpu::DADDu : Fgpu::ADDu;

  bool UseSrcReg = false;
  if (SrcReg != Fgpu::NoRegister)
    UseSrcReg = true;

  unsigned TmpReg = DstReg;
  if (UseSrcReg &&
      getContext().getRegisterInfo()->isSuperOrSubRegisterEq(DstReg, SrcReg)) {
    // At this point we need AT to perform the expansions and we exit if it is
    // not available.
    unsigned ATReg = getATReg(IDLoc);
    if (!ATReg)
      return true;
    TmpReg = ATReg;
  }

  if (isInt<16>(ImmValue)) {
    if (!UseSrcReg)
      SrcReg = ZeroReg;

    // This doesn't quite follow the usual ABI expectations for N32 but matches
    // traditional assembler behaviour. N32 would normally use addiu for both
    // integers and addresses.
    if (IsAddress && !Is32BitImm) {
      TOut.emitRRI(Fgpu::DADDiu, DstReg, SrcReg, ImmValue, IDLoc, STI);
      return false;
    }

    TOut.emitRRI(Fgpu::ADDiu, DstReg, SrcReg, ImmValue, IDLoc, STI);
    return false;
  }

  if (isUInt<16>(ImmValue)) {
    unsigned TmpReg = DstReg;
    if (SrcReg == DstReg) {
      TmpReg = getATReg(IDLoc);
      if (!TmpReg)
        return true;
    }

    TOut.emitRRI(Fgpu::ORi, TmpReg, ZeroReg, ImmValue, IDLoc, STI);
    if (UseSrcReg)
      TOut.emitRRR(ABI.GetPtrAdduOp(), DstReg, TmpReg, SrcReg, IDLoc, STI);
    return false;
  }

  if (isInt<32>(ImmValue) || isUInt<32>(ImmValue)) {
    warnIfNoMacro(IDLoc);

    uint16_t Bits31To16 = (ImmValue >> 16) & 0xffff;
    uint16_t Bits15To0 = ImmValue & 0xffff;
    if (!Is32BitImm && !isInt<32>(ImmValue)) {
      // Traditional behaviour seems to special case this particular value. It's
      // not clear why other masks are handled differently.
      if (ImmValue == 0xffffffff) {
        TOut.emitRI(Fgpu::LUi, TmpReg, 0xffff, IDLoc, STI);
        TOut.emitRRI(Fgpu::DSRL32, TmpReg, TmpReg, 0, IDLoc, STI);
        if (UseSrcReg)
          TOut.emitRRR(AdduOp, DstReg, TmpReg, SrcReg, IDLoc, STI);
        return false;
      }

      // Expand to an ORi instead of a LUi to avoid sign-extending into the
      // upper 32 bits.
      TOut.emitRRI(Fgpu::ORi, TmpReg, ZeroReg, Bits31To16, IDLoc, STI);
      TOut.emitRRI(Fgpu::DSLL, TmpReg, TmpReg, 16, IDLoc, STI);
      if (Bits15To0)
        TOut.emitRRI(Fgpu::ORi, TmpReg, TmpReg, Bits15To0, IDLoc, STI);
      if (UseSrcReg)
        TOut.emitRRR(AdduOp, DstReg, TmpReg, SrcReg, IDLoc, STI);
      return false;
    }

    TOut.emitRI(Fgpu::LUi, TmpReg, Bits31To16, IDLoc, STI);
    if (Bits15To0)
      TOut.emitRRI(Fgpu::ORi, TmpReg, TmpReg, Bits15To0, IDLoc, STI);
    if (UseSrcReg)
      TOut.emitRRR(AdduOp, DstReg, TmpReg, SrcReg, IDLoc, STI);
    return false;
  }

  if (isShiftedUIntAtAnyPosition<16>(ImmValue)) {
    if (Is32BitImm) {
      Error(IDLoc, "instruction requires a 32-bit immediate");
      return true;
    }

    // Traditionally, these immediates are shifted as little as possible and as
    // such we align the most significant bit to bit 15 of our temporary.
    unsigned FirstSet = findFirstSet((uint64_t)ImmValue);
    unsigned LastSet = findLastSet((uint64_t)ImmValue);
    unsigned ShiftAmount = FirstSet - (15 - (LastSet - FirstSet));
    uint16_t Bits = (ImmValue >> ShiftAmount) & 0xffff;
    TOut.emitRRI(Fgpu::ORi, TmpReg, ZeroReg, Bits, IDLoc, STI);
    TOut.emitRRI(Fgpu::DSLL, TmpReg, TmpReg, ShiftAmount, IDLoc, STI);

    if (UseSrcReg)
      TOut.emitRRR(AdduOp, DstReg, TmpReg, SrcReg, IDLoc, STI);

    return false;
  }

  warnIfNoMacro(IDLoc);

  // The remaining case is packed with a sequence of dsll and ori with zeros
  // being omitted and any neighbouring dsll's being coalesced.
  // The highest 32-bit's are equivalent to a 32-bit immediate load.

  // Load bits 32-63 of ImmValue into bits 0-31 of the temporary register.
  if (loadImmediate(ImmValue >> 32, TmpReg, Fgpu::NoRegister, true, false,
                    IDLoc, Out, STI))
    return false;

  // Shift and accumulate into the register. If a 16-bit chunk is zero, then
  // skip it and defer the shift to the next chunk.
  unsigned ShiftCarriedForwards = 16;
  for (int BitNum = 16; BitNum >= 0; BitNum -= 16) {
    uint16_t ImmChunk = (ImmValue >> BitNum) & 0xffff;

    if (ImmChunk != 0) {
      TOut.emitDSLL(TmpReg, TmpReg, ShiftCarriedForwards, IDLoc, STI);
      TOut.emitRRI(Fgpu::ORi, TmpReg, TmpReg, ImmChunk, IDLoc, STI);
      ShiftCarriedForwards = 0;
    }

    ShiftCarriedForwards += 16;
  }
  ShiftCarriedForwards -= 16;

  // Finish any remaining shifts left by trailing zeros.
  if (ShiftCarriedForwards)
    TOut.emitDSLL(TmpReg, TmpReg, ShiftCarriedForwards, IDLoc, STI);

  if (UseSrcReg)
    TOut.emitRRR(AdduOp, DstReg, TmpReg, SrcReg, IDLoc, STI);

  return false;
}

bool FgpuAsmParser::expandLoadImm(MCInst &Inst, bool Is32BitImm, SMLoc IDLoc,
                                  MCStreamer &Out, const MCSubtargetInfo *STI) {
  const MCOperand &ImmOp = Inst.getOperand(1);
  assert(ImmOp.isImm() && "expected immediate operand kind");
  const MCOperand &DstRegOp = Inst.getOperand(0);
  assert(DstRegOp.isReg() && "expected register operand kind");

  if (loadImmediate(ImmOp.getImm(), DstRegOp.getReg(), Fgpu::NoRegister,
                    Is32BitImm, false, IDLoc, Out, STI))
    return true;

  return false;
}

bool FgpuAsmParser::expandLoadAddress(unsigned DstReg, unsigned BaseReg,
                                      const MCOperand &Offset,
                                      bool Is32BitAddress, SMLoc IDLoc,
                                      MCStreamer &Out,
                                      const MCSubtargetInfo *STI) {
  // la can't produce a usable address when addresses are 64-bit.
  if (Is32BitAddress && ABI.ArePtrs64bit()) {
    Warning(IDLoc, "la used to load 64-bit address");
    // Continue as if we had 'dla' instead.
    Is32BitAddress = false;
  }

  // dla requires 64-bit addresses.
  if (!Is32BitAddress && !hasFgpu3()) {
    Error(IDLoc, "instruction requires a 64-bit architecture");
    return true;
  }

  if (!Offset.isImm())
    return loadAndAddSymbolAddress(Offset.getExpr(), DstReg, BaseReg,
                                   Is32BitAddress, IDLoc, Out, STI);

  if (!ABI.ArePtrs64bit()) {
    // Continue as if we had 'la' whether we had 'la' or 'dla'.
    Is32BitAddress = true;
  }

  return loadImmediate(Offset.getImm(), DstReg, BaseReg, Is32BitAddress, true,
                       IDLoc, Out, STI);
}

bool FgpuAsmParser::loadAndAddSymbolAddress(const MCExpr *SymExpr,
                                            unsigned DstReg, unsigned SrcReg,
                                            bool Is32BitSym, SMLoc IDLoc,
                                            MCStreamer &Out,
                                            const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  bool UseSrcReg = SrcReg != Fgpu::NoRegister && SrcReg != Fgpu::ZERO &&
                   SrcReg != Fgpu::ZERO_64;
  warnIfNoMacro(IDLoc);

  if (inPicMode()) {
    MCValue Res;
    if (!SymExpr->evaluateAsRelocatable(Res, nullptr, nullptr)) {
      Error(IDLoc, "expected relocatable expression");
      return true;
    }
    if (Res.getSymB() != nullptr) {
      Error(IDLoc, "expected relocatable expression with only one symbol");
      return true;
    }

    bool IsPtr64 = ABI.ArePtrs64bit();
    bool IsLocalSym =
        Res.getSymA()->getSymbol().isInSection() ||
        Res.getSymA()->getSymbol().isTemporary() ||
        (Res.getSymA()->getSymbol().isELF() &&
         cast<MCSymbolELF>(Res.getSymA()->getSymbol()).getBinding() ==
             ELF::STB_LOCAL);
    bool UseXGOT = STI->getFeatureBits()[Fgpu::FeatureXGOT] && !IsLocalSym;

    // The case where the result register is $25 is somewhat special. If the
    // symbol in the final relocation is external and not modified with a
    // constant then we must use R_FGPU_CALL16 instead of R_FGPU_GOT16
    // or R_FGPU_CALL16 instead of R_FGPU_GOT_DISP in 64-bit case.
    if ((DstReg == Fgpu::T9 || DstReg == Fgpu::T9_64) && !UseSrcReg &&
        Res.getConstant() == 0 && !IsLocalSym) {
      if (UseXGOT) {
        const MCExpr *CallHiExpr = FgpuMCExpr::create(FgpuMCExpr::MEK_CALL_HI16,
                                                      SymExpr, getContext());
        const MCExpr *CallLoExpr = FgpuMCExpr::create(FgpuMCExpr::MEK_CALL_LO16,
                                                      SymExpr, getContext());
        TOut.emitRX(Fgpu::LUi, DstReg, MCOperand::createExpr(CallHiExpr), IDLoc,
                    STI);
        TOut.emitRRR(IsPtr64 ? Fgpu::DADDu : Fgpu::ADDu, DstReg, DstReg, GPReg,
                     IDLoc, STI);
        TOut.emitRRX(IsPtr64 ? Fgpu::LD : Fgpu::LW, DstReg, DstReg,
                     MCOperand::createExpr(CallLoExpr), IDLoc, STI);
      } else {
        const MCExpr *CallExpr =
            FgpuMCExpr::create(FgpuMCExpr::MEK_GOT_CALL, SymExpr, getContext());
        TOut.emitRRX(IsPtr64 ? Fgpu::LD : Fgpu::LW, DstReg, GPReg,
                     MCOperand::createExpr(CallExpr), IDLoc, STI);
      }
      return false;
    }

    unsigned TmpReg = DstReg;
    if (UseSrcReg &&
        getContext().getRegisterInfo()->isSuperOrSubRegisterEq(DstReg,
                                                               SrcReg)) {
      // If $rs is the same as $rd, we need to use AT.
      // If it is not available we exit.
      unsigned ATReg = getATReg(IDLoc);
      if (!ATReg)
        return true;
      TmpReg = ATReg;
    }

    // FIXME: In case of N32 / N64 ABI and emabled XGOT, local addresses
    // loaded using R_FGPU_GOT_PAGE / R_FGPU_GOT_OFST pair of relocations.
    // FIXME: Implement XGOT for microFGPU.
    if (UseXGOT) {
      // Loading address from XGOT
      //   External GOT: lui $tmp, %got_hi(symbol)($gp)
      //                 addu $tmp, $tmp, $gp
      //                 lw $tmp, %got_lo(symbol)($tmp)
      //                >addiu $tmp, $tmp, offset
      //                >addiu $rd, $tmp, $rs
      // The addiu's marked with a '>' may be omitted if they are redundant. If
      // this happens then the last instruction must use $rd as the result
      // register.
      const MCExpr *CallHiExpr =
          FgpuMCExpr::create(FgpuMCExpr::MEK_GOT_HI16, SymExpr, getContext());
      const MCExpr *CallLoExpr = FgpuMCExpr::create(
          FgpuMCExpr::MEK_GOT_LO16, Res.getSymA(), getContext());

      TOut.emitRX(Fgpu::LUi, TmpReg, MCOperand::createExpr(CallHiExpr), IDLoc,
                  STI);
      TOut.emitRRR(IsPtr64 ? Fgpu::DADDu : Fgpu::ADDu, TmpReg, TmpReg, GPReg,
                   IDLoc, STI);
      TOut.emitRRX(IsPtr64 ? Fgpu::LD : Fgpu::LW, TmpReg, TmpReg,
                   MCOperand::createExpr(CallLoExpr), IDLoc, STI);

      if (Res.getConstant() != 0)
        TOut.emitRRX(IsPtr64 ? Fgpu::DADDiu : Fgpu::ADDiu, TmpReg, TmpReg,
                     MCOperand::createExpr(MCConstantExpr::create(
                         Res.getConstant(), getContext())),
                     IDLoc, STI);

      if (UseSrcReg)
        TOut.emitRRR(IsPtr64 ? Fgpu::DADDu : Fgpu::ADDu, DstReg, TmpReg, SrcReg,
                     IDLoc, STI);
      return false;
    }

    const FgpuMCExpr *GotExpr = nullptr;
    const MCExpr *LoExpr = nullptr;
    if (ABI.IsN32() || ABI.IsN64()) {
      // The remaining cases are:
      //   Small offset: ld $tmp, %got_disp(symbol)($gp)
      //                >daddiu $tmp, $tmp, offset
      //                >daddu $rd, $tmp, $rs
      // The daddiu's marked with a '>' may be omitted if they are redundant. If
      // this happens then the last instruction must use $rd as the result
      // register.
      GotExpr = FgpuMCExpr::create(FgpuMCExpr::MEK_GOT_DISP, Res.getSymA(),
                                   getContext());
      if (Res.getConstant() != 0) {
        // Symbols fully resolve with just the %got_disp(symbol) but we
        // must still account for any offset to the symbol for
        // expressions like symbol+8.
        LoExpr = MCConstantExpr::create(Res.getConstant(), getContext());

        // FIXME: Offsets greater than 16 bits are not yet implemented.
        // FIXME: The correct range is a 32-bit sign-extended number.
        if (Res.getConstant() < -0x8000 || Res.getConstant() > 0x7fff) {
          Error(IDLoc, "macro instruction uses large offset, which is not "
                       "currently supported");
          return true;
        }
      }
    } else {
      // The remaining cases are:
      //   External GOT: lw $tmp, %got(symbol)($gp)
      //                >addiu $tmp, $tmp, offset
      //                >addiu $rd, $tmp, $rs
      //   Local GOT:    lw $tmp, %got(symbol+offset)($gp)
      //                 addiu $tmp, $tmp, %lo(symbol+offset)($gp)
      //                >addiu $rd, $tmp, $rs
      // The addiu's marked with a '>' may be omitted if they are redundant. If
      // this happens then the last instruction must use $rd as the result
      // register.
      if (IsLocalSym) {
        GotExpr =
            FgpuMCExpr::create(FgpuMCExpr::MEK_GOT, SymExpr, getContext());
        LoExpr = FgpuMCExpr::create(FgpuMCExpr::MEK_LO, SymExpr, getContext());
      } else {
        // External symbols fully resolve the symbol with just the %got(symbol)
        // but we must still account for any offset to the symbol for
        // expressions like symbol+8.
        GotExpr = FgpuMCExpr::create(FgpuMCExpr::MEK_GOT, Res.getSymA(),
                                     getContext());
        if (Res.getConstant() != 0)
          LoExpr = MCConstantExpr::create(Res.getConstant(), getContext());
      }
    }

    TOut.emitRRX(IsPtr64 ? Fgpu::LD : Fgpu::LW, TmpReg, GPReg,
                 MCOperand::createExpr(GotExpr), IDLoc, STI);

    if (LoExpr)
      TOut.emitRRX(IsPtr64 ? Fgpu::DADDiu : Fgpu::ADDiu, TmpReg, TmpReg,
                   MCOperand::createExpr(LoExpr), IDLoc, STI);

    if (UseSrcReg)
      TOut.emitRRR(IsPtr64 ? Fgpu::DADDu : Fgpu::ADDu, DstReg, TmpReg, SrcReg,
                   IDLoc, STI);

    return false;
  }

  const FgpuMCExpr *HiExpr =
      FgpuMCExpr::create(FgpuMCExpr::MEK_HI, SymExpr, getContext());
  const FgpuMCExpr *LoExpr =
      FgpuMCExpr::create(FgpuMCExpr::MEK_LO, SymExpr, getContext());

  // This is the 64-bit symbol address expansion.
  if (ABI.ArePtrs64bit() && isGP64bit()) {
    // We need AT for the 64-bit expansion in the cases where the optional
    // source register is the destination register and for the superscalar
    // scheduled form.
    //
    // If it is not available we exit if the destination is the same as the
    // source register.

    const FgpuMCExpr *HighestExpr =
        FgpuMCExpr::create(FgpuMCExpr::MEK_HIGHEST, SymExpr, getContext());
    const FgpuMCExpr *HigherExpr =
        FgpuMCExpr::create(FgpuMCExpr::MEK_HIGHER, SymExpr, getContext());

    bool RdRegIsRsReg =
        getContext().getRegisterInfo()->isSuperOrSubRegisterEq(DstReg, SrcReg);

    if (canUseATReg() && UseSrcReg && RdRegIsRsReg) {
      unsigned ATReg = getATReg(IDLoc);

      // If $rs is the same as $rd:
      // (d)la $rd, sym($rd) => lui    $at, %highest(sym)
      //                        daddiu $at, $at, %higher(sym)
      //                        dsll   $at, $at, 16
      //                        daddiu $at, $at, %hi(sym)
      //                        dsll   $at, $at, 16
      //                        daddiu $at, $at, %lo(sym)
      //                        daddu  $rd, $at, $rd
      TOut.emitRX(Fgpu::LUi, ATReg, MCOperand::createExpr(HighestExpr), IDLoc,
                  STI);
      TOut.emitRRX(Fgpu::DADDiu, ATReg, ATReg,
                   MCOperand::createExpr(HigherExpr), IDLoc, STI);
      TOut.emitRRI(Fgpu::DSLL, ATReg, ATReg, 16, IDLoc, STI);
      TOut.emitRRX(Fgpu::DADDiu, ATReg, ATReg, MCOperand::createExpr(HiExpr),
                   IDLoc, STI);
      TOut.emitRRI(Fgpu::DSLL, ATReg, ATReg, 16, IDLoc, STI);
      TOut.emitRRX(Fgpu::DADDiu, ATReg, ATReg, MCOperand::createExpr(LoExpr),
                   IDLoc, STI);
      TOut.emitRRR(Fgpu::DADDu, DstReg, ATReg, SrcReg, IDLoc, STI);

      return false;
    } else if (canUseATReg() && !RdRegIsRsReg && DstReg != getATReg(IDLoc)) {
      unsigned ATReg = getATReg(IDLoc);

      // If the $rs is different from $rd or if $rs isn't specified and we
      // have $at available:
      // (d)la $rd, sym/sym($rs) => lui    $rd, %highest(sym)
      //                            lui    $at, %hi(sym)
      //                            daddiu $rd, $rd, %higher(sym)
      //                            daddiu $at, $at, %lo(sym)
      //                            dsll32 $rd, $rd, 0
      //                            daddu  $rd, $rd, $at
      //                            (daddu  $rd, $rd, $rs)
      //
      // Which is preferred for superscalar issue.
      TOut.emitRX(Fgpu::LUi, DstReg, MCOperand::createExpr(HighestExpr), IDLoc,
                  STI);
      TOut.emitRX(Fgpu::LUi, ATReg, MCOperand::createExpr(HiExpr), IDLoc, STI);
      TOut.emitRRX(Fgpu::DADDiu, DstReg, DstReg,
                   MCOperand::createExpr(HigherExpr), IDLoc, STI);
      TOut.emitRRX(Fgpu::DADDiu, ATReg, ATReg, MCOperand::createExpr(LoExpr),
                   IDLoc, STI);
      TOut.emitRRI(Fgpu::DSLL32, DstReg, DstReg, 0, IDLoc, STI);
      TOut.emitRRR(Fgpu::DADDu, DstReg, DstReg, ATReg, IDLoc, STI);
      if (UseSrcReg)
        TOut.emitRRR(Fgpu::DADDu, DstReg, DstReg, SrcReg, IDLoc, STI);

      return false;
    } else if ((!canUseATReg() && !RdRegIsRsReg) ||
               (canUseATReg() && DstReg == getATReg(IDLoc))) {
      // Otherwise, synthesize the address in the destination register
      // serially:
      // (d)la $rd, sym/sym($rs) => lui    $rd, %highest(sym)
      //                            daddiu $rd, $rd, %higher(sym)
      //                            dsll   $rd, $rd, 16
      //                            daddiu $rd, $rd, %hi(sym)
      //                            dsll   $rd, $rd, 16
      //                            daddiu $rd, $rd, %lo(sym)
      TOut.emitRX(Fgpu::LUi, DstReg, MCOperand::createExpr(HighestExpr), IDLoc,
                  STI);
      TOut.emitRRX(Fgpu::DADDiu, DstReg, DstReg,
                   MCOperand::createExpr(HigherExpr), IDLoc, STI);
      TOut.emitRRI(Fgpu::DSLL, DstReg, DstReg, 16, IDLoc, STI);
      TOut.emitRRX(Fgpu::DADDiu, DstReg, DstReg,
                   MCOperand::createExpr(HiExpr), IDLoc, STI);
      TOut.emitRRI(Fgpu::DSLL, DstReg, DstReg, 16, IDLoc, STI);
      TOut.emitRRX(Fgpu::DADDiu, DstReg, DstReg,
                   MCOperand::createExpr(LoExpr), IDLoc, STI);
      if (UseSrcReg)
        TOut.emitRRR(Fgpu::DADDu, DstReg, DstReg, SrcReg, IDLoc, STI);

      return false;
    } else {
      // We have a case where SrcReg == DstReg and we don't have $at
      // available. We can't expand this case, so error out appropriately.
      assert(SrcReg == DstReg && !canUseATReg() &&
             "Could have expanded dla but didn't?");
      reportParseError(IDLoc,
                     "pseudo-instruction requires $at, which is not available");
      return true;
    }
  }

  // And now, the 32-bit symbol address expansion:
  // If $rs is the same as $rd:
  // (d)la $rd, sym($rd)     => lui   $at, %hi(sym)
  //                            ori   $at, $at, %lo(sym)
  //                            addu  $rd, $at, $rd
  // Otherwise, if the $rs is different from $rd or if $rs isn't specified:
  // (d)la $rd, sym/sym($rs) => lui   $rd, %hi(sym)
  //                            ori   $rd, $rd, %lo(sym)
  //                            (addu $rd, $rd, $rs)
  unsigned TmpReg = DstReg;
  if (UseSrcReg &&
      getContext().getRegisterInfo()->isSuperOrSubRegisterEq(DstReg, SrcReg)) {
    // If $rs is the same as $rd, we need to use AT.
    // If it is not available we exit.
    unsigned ATReg = getATReg(IDLoc);
    if (!ATReg)
      return true;
    TmpReg = ATReg;
  }

  TOut.emitRX(Fgpu::LUi, TmpReg, MCOperand::createExpr(HiExpr), IDLoc, STI);
  TOut.emitRRX(Fgpu::ADDiu, TmpReg, TmpReg, MCOperand::createExpr(LoExpr),
               IDLoc, STI);

  if (UseSrcReg)
    TOut.emitRRR(Fgpu::ADDu, DstReg, TmpReg, SrcReg, IDLoc, STI);
  else
    assert(
        getContext().getRegisterInfo()->isSuperOrSubRegisterEq(DstReg, TmpReg));

  return false;
}

// Each double-precision register DO-D15 overlaps with two of the single
// precision registers F0-F31. As an example, all of the following hold true:
// D0 + 1 == F1, F1 + 1 == D1, F1 + 1 == F2, depending on the context.
static unsigned nextReg(unsigned Reg) {
  if (FgpuMCRegisterClasses[Fgpu::FGR32RegClassID].contains(Reg))
    return Reg == (unsigned)Fgpu::F31 ? (unsigned)Fgpu::F0 : Reg + 1;
  switch (Reg) {
  default: llvm_unreachable("Unknown register in assembly macro expansion!");
  case Fgpu::ZERO: return Fgpu::AT;
  case Fgpu::AT:   return Fgpu::V0;
  case Fgpu::V0:   return Fgpu::V1;
  case Fgpu::V1:   return Fgpu::A0;
  case Fgpu::A0:   return Fgpu::A1;
  case Fgpu::A1:   return Fgpu::A2;
  case Fgpu::A2:   return Fgpu::A3;
  case Fgpu::A3:   return Fgpu::T0;
  case Fgpu::T0:   return Fgpu::T1;
  case Fgpu::T1:   return Fgpu::T2;
  case Fgpu::T2:   return Fgpu::T3;
  case Fgpu::T3:   return Fgpu::T4;
  case Fgpu::T4:   return Fgpu::T5;
  case Fgpu::T5:   return Fgpu::T6;
  case Fgpu::T6:   return Fgpu::T7;
  case Fgpu::T7:   return Fgpu::S0;
  case Fgpu::S0:   return Fgpu::S1;
  case Fgpu::S1:   return Fgpu::S2;
  case Fgpu::S2:   return Fgpu::S3;
  case Fgpu::S3:   return Fgpu::S4;
  case Fgpu::S4:   return Fgpu::S5;
  case Fgpu::S5:   return Fgpu::S6;
  case Fgpu::S6:   return Fgpu::S7;
  case Fgpu::S7:   return Fgpu::T8;
  case Fgpu::T8:   return Fgpu::T9;
  case Fgpu::T9:   return Fgpu::K0;
  case Fgpu::K0:   return Fgpu::K1;
  case Fgpu::K1:   return Fgpu::GP;
  case Fgpu::GP:   return Fgpu::SP;
  case Fgpu::SP:   return Fgpu::FP;
  case Fgpu::FP:   return Fgpu::RA;
  case Fgpu::RA:   return Fgpu::ZERO;
  case Fgpu::D0:   return Fgpu::F1;
  case Fgpu::D1:   return Fgpu::F3;
  case Fgpu::D2:   return Fgpu::F5;
  case Fgpu::D3:   return Fgpu::F7;
  case Fgpu::D4:   return Fgpu::F9;
  case Fgpu::D5:   return Fgpu::F11;
  case Fgpu::D6:   return Fgpu::F13;
  case Fgpu::D7:   return Fgpu::F15;
  case Fgpu::D8:   return Fgpu::F17;
  case Fgpu::D9:   return Fgpu::F19;
  case Fgpu::D10:   return Fgpu::F21;
  case Fgpu::D11:   return Fgpu::F23;
  case Fgpu::D12:   return Fgpu::F25;
  case Fgpu::D13:   return Fgpu::F27;
  case Fgpu::D14:   return Fgpu::F29;
  case Fgpu::D15:   return Fgpu::F31;
  }
}

// FIXME: This method is too general. In principle we should compute the number
// of instructions required to synthesize the immediate inline compared to
// synthesizing the address inline and relying on non .text sections.
// For static O32 and N32 this may yield a small benefit, for static N64 this is
// likely to yield a much larger benefit as we have to synthesize a 64bit
// address to load a 64 bit value.
bool FgpuAsmParser::emitPartialAddress(FgpuTargetStreamer &TOut, SMLoc IDLoc,
                                       MCSymbol *Sym) {
  unsigned ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  if(IsPicEnabled) {
    const MCExpr *GotSym =
        MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
    const FgpuMCExpr *GotExpr =
        FgpuMCExpr::create(FgpuMCExpr::MEK_GOT, GotSym, getContext());

    if(isABI_O32() || isABI_N32()) {
      TOut.emitRRX(Fgpu::LW, ATReg, GPReg, MCOperand::createExpr(GotExpr),
                   IDLoc, STI);
    } else { //isABI_N64()
      TOut.emitRRX(Fgpu::LD, ATReg, GPReg, MCOperand::createExpr(GotExpr),
                   IDLoc, STI);
    }
  } else { //!IsPicEnabled
    const MCExpr *HiSym =
        MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
    const FgpuMCExpr *HiExpr =
        FgpuMCExpr::create(FgpuMCExpr::MEK_HI, HiSym, getContext());

    // FIXME: This is technically correct but gives a different result to gas,
    // but gas is incomplete there (it has a fixme noting it doesn't work with
    // 64-bit addresses).
    // FIXME: With -msym32 option, the address expansion for N64 should probably
    // use the O32 / N32 case. It's safe to use the 64 address expansion as the
    // symbol's value is considered sign extended.
    if(isABI_O32() || isABI_N32()) {
      TOut.emitRX(Fgpu::LUi, ATReg, MCOperand::createExpr(HiExpr), IDLoc, STI);
    } else { //isABI_N64()
      const MCExpr *HighestSym =
          MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
      const FgpuMCExpr *HighestExpr =
          FgpuMCExpr::create(FgpuMCExpr::MEK_HIGHEST, HighestSym, getContext());
      const MCExpr *HigherSym =
          MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
      const FgpuMCExpr *HigherExpr =
          FgpuMCExpr::create(FgpuMCExpr::MEK_HIGHER, HigherSym, getContext());

      TOut.emitRX(Fgpu::LUi, ATReg, MCOperand::createExpr(HighestExpr), IDLoc,
                  STI);
      TOut.emitRRX(Fgpu::DADDiu, ATReg, ATReg,
                   MCOperand::createExpr(HigherExpr), IDLoc, STI);
      TOut.emitRRI(Fgpu::DSLL, ATReg, ATReg, 16, IDLoc, STI);
      TOut.emitRRX(Fgpu::DADDiu, ATReg, ATReg, MCOperand::createExpr(HiExpr),
                   IDLoc, STI);
      TOut.emitRRI(Fgpu::DSLL, ATReg, ATReg, 16, IDLoc, STI);
    }
  }
  return false;
}

static uint64_t convertIntToDoubleImm(uint64_t ImmOp64) {
  // If ImmOp64 is AsmToken::Integer type (all bits set to zero in the
  // exponent field), convert it to double (e.g. 1 to 1.0)
  if ((Hi_32(ImmOp64) & 0x7ff00000) == 0) {
    APFloat RealVal(APFloat::IEEEdouble(), ImmOp64);
    ImmOp64 = RealVal.bitcastToAPInt().getZExtValue();
  }
  return ImmOp64;
}

static uint32_t covertDoubleImmToSingleImm(uint64_t ImmOp64) {
  // Conversion of a double in an uint64_t to a float in a uint32_t,
  // retaining the bit pattern of a float.
  double DoubleImm = BitsToDouble(ImmOp64);
  float TmpFloat = static_cast<float>(DoubleImm);
  return FloatToBits(TmpFloat);
}

bool FgpuAsmParser::expandLoadSingleImmToGPR(MCInst &Inst, SMLoc IDLoc,
                                             MCStreamer &Out,
                                             const MCSubtargetInfo *STI) {
  assert(Inst.getNumOperands() == 2 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() && Inst.getOperand(1).isImm() &&
         "Invalid instruction operand.");

  unsigned FirstReg = Inst.getOperand(0).getReg();
  uint64_t ImmOp64 = Inst.getOperand(1).getImm();

  uint32_t ImmOp32 = covertDoubleImmToSingleImm(convertIntToDoubleImm(ImmOp64));

  return loadImmediate(ImmOp32, FirstReg, Fgpu::NoRegister, true, false, IDLoc,
                       Out, STI);
}

bool FgpuAsmParser::expandLoadSingleImmToFPR(MCInst &Inst, SMLoc IDLoc,
                                             MCStreamer &Out,
                                             const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  assert(Inst.getNumOperands() == 2 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() && Inst.getOperand(1).isImm() &&
         "Invalid instruction operand.");

  unsigned FirstReg = Inst.getOperand(0).getReg();
  uint64_t ImmOp64 = Inst.getOperand(1).getImm();

  ImmOp64 = convertIntToDoubleImm(ImmOp64);

  uint32_t ImmOp32 = covertDoubleImmToSingleImm(ImmOp64);

  unsigned TmpReg = Fgpu::ZERO;
  if (ImmOp32 != 0) {
    TmpReg = getATReg(IDLoc);
    if (!TmpReg)
      return true;
  }

  if (Lo_32(ImmOp64) == 0) {
    if (TmpReg != Fgpu::ZERO && loadImmediate(ImmOp32, TmpReg, Fgpu::NoRegister,
                                              true, false, IDLoc, Out, STI))
      return true;
    TOut.emitRR(Fgpu::MTC1, FirstReg, TmpReg, IDLoc, STI);
    return false;
  }

  MCSection *CS = getStreamer().getCurrentSectionOnly();
  // FIXME: Enhance this expansion to use the .lit4 & .lit8 sections
  // where appropriate.
  MCSection *ReadOnlySection =
      getContext().getELFSection(".rodata", ELF::SHT_PROGBITS, ELF::SHF_ALLOC);

  MCSymbol *Sym = getContext().createTempSymbol();
  const MCExpr *LoSym =
      MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
  const FgpuMCExpr *LoExpr =
      FgpuMCExpr::create(FgpuMCExpr::MEK_LO, LoSym, getContext());

  getStreamer().SwitchSection(ReadOnlySection);
  getStreamer().emitLabel(Sym, IDLoc);
  getStreamer().emitInt32(ImmOp32);
  getStreamer().SwitchSection(CS);

  if (emitPartialAddress(TOut, IDLoc, Sym))
    return true;
  TOut.emitRRX(Fgpu::LWC1, FirstReg, TmpReg, MCOperand::createExpr(LoExpr),
               IDLoc, STI);
  return false;
}

bool FgpuAsmParser::expandLoadDoubleImmToGPR(MCInst &Inst, SMLoc IDLoc,
                                             MCStreamer &Out,
                                             const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  assert(Inst.getNumOperands() == 2 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() && Inst.getOperand(1).isImm() &&
         "Invalid instruction operand.");

  unsigned FirstReg = Inst.getOperand(0).getReg();
  uint64_t ImmOp64 = Inst.getOperand(1).getImm();

  ImmOp64 = convertIntToDoubleImm(ImmOp64);

  if (Lo_32(ImmOp64) == 0) {
    if (isGP64bit()) {
      if (loadImmediate(ImmOp64, FirstReg, Fgpu::NoRegister, false, false,
                        IDLoc, Out, STI))
        return true;
    } else {
      if (loadImmediate(Hi_32(ImmOp64), FirstReg, Fgpu::NoRegister, true, false,
                        IDLoc, Out, STI))
        return true;

      if (loadImmediate(0, nextReg(FirstReg), Fgpu::NoRegister, true, false,
                        IDLoc, Out, STI))
        return true;
    }
    return false;
  }

  MCSection *CS = getStreamer().getCurrentSectionOnly();
  MCSection *ReadOnlySection =
      getContext().getELFSection(".rodata", ELF::SHT_PROGBITS, ELF::SHF_ALLOC);

  MCSymbol *Sym = getContext().createTempSymbol();
  const MCExpr *LoSym =
      MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
  const FgpuMCExpr *LoExpr =
      FgpuMCExpr::create(FgpuMCExpr::MEK_LO, LoSym, getContext());

  getStreamer().SwitchSection(ReadOnlySection);
  getStreamer().emitLabel(Sym, IDLoc);
  getStreamer().emitValueToAlignment(8);
  getStreamer().emitIntValue(ImmOp64, 8);
  getStreamer().SwitchSection(CS);

  unsigned TmpReg = getATReg(IDLoc);
  if (!TmpReg)
    return true;

  if (emitPartialAddress(TOut, IDLoc, Sym))
    return true;

  TOut.emitRRX(isABI_N64() ? Fgpu::DADDiu : Fgpu::ADDiu, TmpReg, TmpReg,
               MCOperand::createExpr(LoExpr), IDLoc, STI);

  if (isGP64bit())
    TOut.emitRRI(Fgpu::LD, FirstReg, TmpReg, 0, IDLoc, STI);
  else {
    TOut.emitRRI(Fgpu::LW, FirstReg, TmpReg, 0, IDLoc, STI);
    TOut.emitRRI(Fgpu::LW, nextReg(FirstReg), TmpReg, 4, IDLoc, STI);
  }
  return false;
}

bool FgpuAsmParser::expandLoadDoubleImmToFPR(MCInst &Inst, bool Is64FPU,
                                             SMLoc IDLoc, MCStreamer &Out,
                                             const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  assert(Inst.getNumOperands() == 2 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() && Inst.getOperand(1).isImm() &&
         "Invalid instruction operand.");

  unsigned FirstReg = Inst.getOperand(0).getReg();
  uint64_t ImmOp64 = Inst.getOperand(1).getImm();

  ImmOp64 = convertIntToDoubleImm(ImmOp64);

  unsigned TmpReg = Fgpu::ZERO;
  if (ImmOp64 != 0) {
    TmpReg = getATReg(IDLoc);
    if (!TmpReg)
      return true;
  }

  if ((Lo_32(ImmOp64) == 0) &&
      !((Hi_32(ImmOp64) & 0xffff0000) && (Hi_32(ImmOp64) & 0x0000ffff))) {
    if (isGP64bit()) {
      if (TmpReg != Fgpu::ZERO &&
          loadImmediate(ImmOp64, TmpReg, Fgpu::NoRegister, false, false, IDLoc,
                        Out, STI))
        return true;
      TOut.emitRR(Fgpu::DMTC1, FirstReg, TmpReg, IDLoc, STI);
      return false;
    }

    if (TmpReg != Fgpu::ZERO &&
        loadImmediate(Hi_32(ImmOp64), TmpReg, Fgpu::NoRegister, true, false,
                      IDLoc, Out, STI))
      return true;

    if (hasFgpu32r2()) {
      TOut.emitRR(Fgpu::MTC1, FirstReg, Fgpu::ZERO, IDLoc, STI);
      TOut.emitRRR(Fgpu::MTHC1_D32, FirstReg, FirstReg, TmpReg, IDLoc, STI);
    } else {
      TOut.emitRR(Fgpu::MTC1, nextReg(FirstReg), TmpReg, IDLoc, STI);
      TOut.emitRR(Fgpu::MTC1, FirstReg, Fgpu::ZERO, IDLoc, STI);
    }
    return false;
  }

  MCSection *CS = getStreamer().getCurrentSectionOnly();
  // FIXME: Enhance this expansion to use the .lit4 & .lit8 sections
  // where appropriate.
  MCSection *ReadOnlySection =
      getContext().getELFSection(".rodata", ELF::SHT_PROGBITS, ELF::SHF_ALLOC);

  MCSymbol *Sym = getContext().createTempSymbol();
  const MCExpr *LoSym =
      MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
  const FgpuMCExpr *LoExpr =
      FgpuMCExpr::create(FgpuMCExpr::MEK_LO, LoSym, getContext());

  getStreamer().SwitchSection(ReadOnlySection);
  getStreamer().emitLabel(Sym, IDLoc);
  getStreamer().emitValueToAlignment(8);
  getStreamer().emitIntValue(ImmOp64, 8);
  getStreamer().SwitchSection(CS);

  if (emitPartialAddress(TOut, IDLoc, Sym))
    return true;

  TOut.emitRRX(Is64FPU ? Fgpu::LDC164 : Fgpu::LDC1, FirstReg, TmpReg,
               MCOperand::createExpr(LoExpr), IDLoc, STI);

  return false;
}

bool FgpuAsmParser::expandUncondBranchMMPseudo(MCInst &Inst, SMLoc IDLoc,
                                               MCStreamer &Out,
                                               const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();

  assert(getInstDesc(Inst.getOpcode()).getNumOperands() == 1 &&
         "unexpected number of operands");

  MCOperand Offset = Inst.getOperand(0);
  if (Offset.isExpr()) {
    Inst.clear();
    Inst.setOpcode(Fgpu::BEQ_MM);
    Inst.addOperand(MCOperand::createReg(Fgpu::ZERO));
    Inst.addOperand(MCOperand::createReg(Fgpu::ZERO));
    Inst.addOperand(MCOperand::createExpr(Offset.getExpr()));
  } else {
    assert(Offset.isImm() && "expected immediate operand kind");
    if (isInt<11>(Offset.getImm())) {
      // If offset fits into 11 bits then this instruction becomes microFGPU
      // 16-bit unconditional branch instruction.
      if (inMicroFgpuMode())
        Inst.setOpcode(hasFgpu32r6() ? Fgpu::BC16_MMR6 : Fgpu::B16_MM);
    } else {
      if (!isInt<17>(Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (offsetToAlignment(Offset.getImm(), Align(2)))
        return Error(IDLoc, "branch to misaligned address");
      Inst.clear();
      Inst.setOpcode(Fgpu::BEQ_MM);
      Inst.addOperand(MCOperand::createReg(Fgpu::ZERO));
      Inst.addOperand(MCOperand::createReg(Fgpu::ZERO));
      Inst.addOperand(MCOperand::createImm(Offset.getImm()));
    }
  }
  Out.emitInstruction(Inst, *STI);

  // If .set reorder is active and branch instruction has a delay slot,
  // emit a NOP after it.
  const MCInstrDesc &MCID = getInstDesc(Inst.getOpcode());
  if (MCID.hasDelaySlot() && AssemblerOptions.back()->isReorder())
    TOut.emitEmptyDelaySlot(true, IDLoc, STI);

  return false;
}

bool FgpuAsmParser::expandBranchImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                    const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  const MCOperand &DstRegOp = Inst.getOperand(0);
  assert(DstRegOp.isReg() && "expected register operand kind");

  const MCOperand &ImmOp = Inst.getOperand(1);
  assert(ImmOp.isImm() && "expected immediate operand kind");

  const MCOperand &MemOffsetOp = Inst.getOperand(2);
  assert((MemOffsetOp.isImm() || MemOffsetOp.isExpr()) &&
         "expected immediate or expression operand");

  bool IsLikely = false;

  unsigned OpCode = 0;
  switch(Inst.getOpcode()) {
    case Fgpu::BneImm:
      OpCode = Fgpu::BNE;
      break;
    case Fgpu::BeqImm:
      OpCode = Fgpu::BEQ;
      break;
    case Fgpu::BEQLImmMacro:
      OpCode = Fgpu::BEQL;
      IsLikely = true;
      break;
    case Fgpu::BNELImmMacro:
      OpCode = Fgpu::BNEL;
      IsLikely = true;
      break;
    default:
      llvm_unreachable("Unknown immediate branch pseudo-instruction.");
      break;
  }

  int64_t ImmValue = ImmOp.getImm();
  if (ImmValue == 0) {
    if (IsLikely) {
      TOut.emitRRX(OpCode, DstRegOp.getReg(), Fgpu::ZERO,
                   MCOperand::createExpr(MemOffsetOp.getExpr()), IDLoc, STI);
      TOut.emitRRI(Fgpu::SLL, Fgpu::ZERO, Fgpu::ZERO, 0, IDLoc, STI);
    } else
      TOut.emitRRX(OpCode, DstRegOp.getReg(), Fgpu::ZERO, MemOffsetOp, IDLoc,
              STI);
  } else {
    warnIfNoMacro(IDLoc);

    unsigned ATReg = getATReg(IDLoc);
    if (!ATReg)
      return true;

    if (loadImmediate(ImmValue, ATReg, Fgpu::NoRegister, !isGP64bit(), true,
                      IDLoc, Out, STI))
      return true;

    if (IsLikely) {
      TOut.emitRRX(OpCode, DstRegOp.getReg(), ATReg,
              MCOperand::createExpr(MemOffsetOp.getExpr()), IDLoc, STI);
      TOut.emitRRI(Fgpu::SLL, Fgpu::ZERO, Fgpu::ZERO, 0, IDLoc, STI);
    } else
      TOut.emitRRX(OpCode, DstRegOp.getReg(), ATReg, MemOffsetOp, IDLoc, STI);
  }
  return false;
}

void FgpuAsmParser::expandMem16Inst(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                    const MCSubtargetInfo *STI, bool IsLoad) {
  unsigned NumOp = Inst.getNumOperands();
  assert((NumOp == 3 || NumOp == 4) && "unexpected operands number");
  unsigned StartOp = NumOp == 3 ? 0 : 1;

  const MCOperand &DstRegOp = Inst.getOperand(StartOp);
  assert(DstRegOp.isReg() && "expected register operand kind");
  const MCOperand &BaseRegOp = Inst.getOperand(StartOp + 1);
  assert(BaseRegOp.isReg() && "expected register operand kind");
  const MCOperand &OffsetOp = Inst.getOperand(StartOp + 2);

  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned OpCode = Inst.getOpcode();
  unsigned DstReg = DstRegOp.getReg();
  unsigned BaseReg = BaseRegOp.getReg();
  unsigned TmpReg = DstReg;

  const MCInstrDesc &Desc = getInstDesc(OpCode);
  int16_t DstRegClass = Desc.OpInfo[StartOp].RegClass;
  unsigned DstRegClassID =
      getContext().getRegisterInfo()->getRegClass(DstRegClass).getID();
  bool IsGPR = (DstRegClassID == Fgpu::GPR32RegClassID) ||
               (DstRegClassID == Fgpu::GPR64RegClassID);

  if (!IsLoad || !IsGPR || (BaseReg == DstReg)) {
    // At this point we need AT to perform the expansions
    // and we exit if it is not available.
    TmpReg = getATReg(IDLoc);
    if (!TmpReg)
      return;
  }

  auto emitInstWithOffset = [&](const MCOperand &Off) {
    if (NumOp == 3)
      TOut.emitRRX(OpCode, DstReg, TmpReg, Off, IDLoc, STI);
    else
      TOut.emitRRRX(OpCode, DstReg, DstReg, TmpReg, Off, IDLoc, STI);
  };

  if (OffsetOp.isImm()) {
    int64_t LoOffset = OffsetOp.getImm() & 0xffff;
    int64_t HiOffset = OffsetOp.getImm() & ~0xffff;

    // If msb of LoOffset is 1(negative number) we must increment
    // HiOffset to account for the sign-extension of the low part.
    if (LoOffset & 0x8000)
      HiOffset += 0x10000;

    bool IsLargeOffset = HiOffset != 0;

    if (IsLargeOffset) {
      bool Is32BitImm = isInt<32>(OffsetOp.getImm());
      if (loadImmediate(HiOffset, TmpReg, Fgpu::NoRegister, Is32BitImm, true,
                        IDLoc, Out, STI))
        return;
    }

    if (BaseReg != Fgpu::ZERO && BaseReg != Fgpu::ZERO_64)
      TOut.emitRRR(ABI.ArePtrs64bit() ? Fgpu::DADDu : Fgpu::ADDu, TmpReg,
                   TmpReg, BaseReg, IDLoc, STI);
    emitInstWithOffset(MCOperand::createImm(int16_t(LoOffset)));
    return;
  }

  if (OffsetOp.isExpr()) {
    if (inPicMode()) {
      // FIXME:
      // c) Check that immediates of R_FGPU_GOT16/R_FGPU_LO16 relocations
      //    do not exceed 16-bit.
      // d) Use R_FGPU_GOT_PAGE/R_FGPU_GOT_OFST relocations instead
      //    of R_FGPU_GOT_DISP in appropriate cases to reduce number
      //    of GOT entries.
      MCValue Res;
      if (!OffsetOp.getExpr()->evaluateAsRelocatable(Res, nullptr, nullptr)) {
        Error(IDLoc, "expected relocatable expression");
        return;
      }
      if (Res.getSymB() != nullptr) {
        Error(IDLoc, "expected relocatable expression with only one symbol");
        return;
      }

      loadAndAddSymbolAddress(Res.getSymA(), TmpReg, BaseReg,
                              !ABI.ArePtrs64bit(), IDLoc, Out, STI);
      emitInstWithOffset(MCOperand::createImm(int16_t(Res.getConstant())));
    } else {
      // FIXME: Implement 64-bit case.
      // 1) lw $8, sym => lui $8,  %hi(sym)
      //                  lw  $8,  %lo(sym)($8)
      // 2) sw $8, sym => lui $at, %hi(sym)
      //                  sw  $8,  %lo(sym)($at)
      const MCExpr *OffExpr = OffsetOp.getExpr();
      MCOperand LoOperand = MCOperand::createExpr(
          FgpuMCExpr::create(FgpuMCExpr::MEK_LO, OffExpr, getContext()));
      MCOperand HiOperand = MCOperand::createExpr(
          FgpuMCExpr::create(FgpuMCExpr::MEK_HI, OffExpr, getContext()));

      if (ABI.IsN64()) {
        MCOperand HighestOperand = MCOperand::createExpr(
            FgpuMCExpr::create(FgpuMCExpr::MEK_HIGHEST, OffExpr, getContext()));
        MCOperand HigherOperand = MCOperand::createExpr(
            FgpuMCExpr::create(FgpuMCExpr::MEK_HIGHER, OffExpr, getContext()));

        TOut.emitRX(Fgpu::LUi, TmpReg, HighestOperand, IDLoc, STI);
        TOut.emitRRX(Fgpu::DADDiu, TmpReg, TmpReg, HigherOperand, IDLoc, STI);
        TOut.emitRRI(Fgpu::DSLL, TmpReg, TmpReg, 16, IDLoc, STI);
        TOut.emitRRX(Fgpu::DADDiu, TmpReg, TmpReg, HiOperand, IDLoc, STI);
        TOut.emitRRI(Fgpu::DSLL, TmpReg, TmpReg, 16, IDLoc, STI);
        if (BaseReg != Fgpu::ZERO && BaseReg != Fgpu::ZERO_64)
          TOut.emitRRR(Fgpu::DADDu, TmpReg, TmpReg, BaseReg, IDLoc, STI);
        emitInstWithOffset(LoOperand);
      } else {
        // Generate the base address in TmpReg.
        TOut.emitRX(Fgpu::LUi, TmpReg, HiOperand, IDLoc, STI);
        if (BaseReg != Fgpu::ZERO)
          TOut.emitRRR(Fgpu::ADDu, TmpReg, TmpReg, BaseReg, IDLoc, STI);
        // Emit the load or store with the adjusted base and offset.
        emitInstWithOffset(LoOperand);
      }
    }
    return;
  }

  llvm_unreachable("unexpected operand type");
}

void FgpuAsmParser::expandMem9Inst(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                   const MCSubtargetInfo *STI, bool IsLoad) {
  unsigned NumOp = Inst.getNumOperands();
  assert((NumOp == 3 || NumOp == 4) && "unexpected operands number");
  unsigned StartOp = NumOp == 3 ? 0 : 1;

  const MCOperand &DstRegOp = Inst.getOperand(StartOp);
  assert(DstRegOp.isReg() && "expected register operand kind");
  const MCOperand &BaseRegOp = Inst.getOperand(StartOp + 1);
  assert(BaseRegOp.isReg() && "expected register operand kind");
  const MCOperand &OffsetOp = Inst.getOperand(StartOp + 2);

  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned OpCode = Inst.getOpcode();
  unsigned DstReg = DstRegOp.getReg();
  unsigned BaseReg = BaseRegOp.getReg();
  unsigned TmpReg = DstReg;

  const MCInstrDesc &Desc = getInstDesc(OpCode);
  int16_t DstRegClass = Desc.OpInfo[StartOp].RegClass;
  unsigned DstRegClassID =
      getContext().getRegisterInfo()->getRegClass(DstRegClass).getID();
  bool IsGPR = (DstRegClassID == Fgpu::GPR32RegClassID) ||
               (DstRegClassID == Fgpu::GPR64RegClassID);

  if (!IsLoad || !IsGPR || (BaseReg == DstReg)) {
    // At this point we need AT to perform the expansions
    // and we exit if it is not available.
    TmpReg = getATReg(IDLoc);
    if (!TmpReg)
      return;
  }

  auto emitInst = [&]() {
    if (NumOp == 3)
      TOut.emitRRX(OpCode, DstReg, TmpReg, MCOperand::createImm(0), IDLoc, STI);
    else
      TOut.emitRRRX(OpCode, DstReg, DstReg, TmpReg, MCOperand::createImm(0),
                    IDLoc, STI);
  };

  if (OffsetOp.isImm()) {
    loadImmediate(OffsetOp.getImm(), TmpReg, BaseReg, !ABI.ArePtrs64bit(), true,
                  IDLoc, Out, STI);
    emitInst();
    return;
  }

  if (OffsetOp.isExpr()) {
    loadAndAddSymbolAddress(OffsetOp.getExpr(), TmpReg, BaseReg,
                            !ABI.ArePtrs64bit(), IDLoc, Out, STI);
    emitInst();
    return;
  }

  llvm_unreachable("unexpected operand type");
}

bool FgpuAsmParser::expandLoadStoreMultiple(MCInst &Inst, SMLoc IDLoc,
                                            MCStreamer &Out,
                                            const MCSubtargetInfo *STI) {
  unsigned OpNum = Inst.getNumOperands();
  unsigned Opcode = Inst.getOpcode();
  unsigned NewOpcode = Opcode == Fgpu::SWM_MM ? Fgpu::SWM32_MM : Fgpu::LWM32_MM;

  assert(Inst.getOperand(OpNum - 1).isImm() &&
         Inst.getOperand(OpNum - 2).isReg() &&
         Inst.getOperand(OpNum - 3).isReg() && "Invalid instruction operand.");

  if (OpNum < 8 && Inst.getOperand(OpNum - 1).getImm() <= 60 &&
      Inst.getOperand(OpNum - 1).getImm() >= 0 &&
      (Inst.getOperand(OpNum - 2).getReg() == Fgpu::SP ||
       Inst.getOperand(OpNum - 2).getReg() == Fgpu::SP_64) &&
      (Inst.getOperand(OpNum - 3).getReg() == Fgpu::RA ||
       Inst.getOperand(OpNum - 3).getReg() == Fgpu::RA_64)) {
    // It can be implemented as SWM16 or LWM16 instruction.
    if (inMicroFgpuMode() && hasFgpu32r6())
      NewOpcode = Opcode == Fgpu::SWM_MM ? Fgpu::SWM16_MMR6 : Fgpu::LWM16_MMR6;
    else
      NewOpcode = Opcode == Fgpu::SWM_MM ? Fgpu::SWM16_MM : Fgpu::LWM16_MM;
  }

  Inst.setOpcode(NewOpcode);
  Out.emitInstruction(Inst, *STI);
  return false;
}

bool FgpuAsmParser::expandCondBranches(MCInst &Inst, SMLoc IDLoc,
                                       MCStreamer &Out,
                                       const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  bool EmittedNoMacroWarning = false;
  unsigned PseudoOpcode = Inst.getOpcode();
  unsigned SrcReg = Inst.getOperand(0).getReg();
  const MCOperand &TrgOp = Inst.getOperand(1);
  const MCExpr *OffsetExpr = Inst.getOperand(2).getExpr();

  unsigned ZeroSrcOpcode, ZeroTrgOpcode;
  bool ReverseOrderSLT, IsUnsigned, IsLikely, AcceptsEquality;

  unsigned TrgReg;
  if (TrgOp.isReg())
    TrgReg = TrgOp.getReg();
  else if (TrgOp.isImm()) {
    warnIfNoMacro(IDLoc);
    EmittedNoMacroWarning = true;

    TrgReg = getATReg(IDLoc);
    if (!TrgReg)
      return true;

    switch(PseudoOpcode) {
    default:
      llvm_unreachable("unknown opcode for branch pseudo-instruction");
    case Fgpu::BLTImmMacro:
      PseudoOpcode = Fgpu::BLT;
      break;
    case Fgpu::BLEImmMacro:
      PseudoOpcode = Fgpu::BLE;
      break;
    case Fgpu::BGEImmMacro:
      PseudoOpcode = Fgpu::BGE;
      break;
    case Fgpu::BGTImmMacro:
      PseudoOpcode = Fgpu::BGT;
      break;
    case Fgpu::BLTUImmMacro:
      PseudoOpcode = Fgpu::BLTU;
      break;
    case Fgpu::BLEUImmMacro:
      PseudoOpcode = Fgpu::BLEU;
      break;
    case Fgpu::BGEUImmMacro:
      PseudoOpcode = Fgpu::BGEU;
      break;
    case Fgpu::BGTUImmMacro:
      PseudoOpcode = Fgpu::BGTU;
      break;
    case Fgpu::BLTLImmMacro:
      PseudoOpcode = Fgpu::BLTL;
      break;
    case Fgpu::BLELImmMacro:
      PseudoOpcode = Fgpu::BLEL;
      break;
    case Fgpu::BGELImmMacro:
      PseudoOpcode = Fgpu::BGEL;
      break;
    case Fgpu::BGTLImmMacro:
      PseudoOpcode = Fgpu::BGTL;
      break;
    case Fgpu::BLTULImmMacro:
      PseudoOpcode = Fgpu::BLTUL;
      break;
    case Fgpu::BLEULImmMacro:
      PseudoOpcode = Fgpu::BLEUL;
      break;
    case Fgpu::BGEULImmMacro:
      PseudoOpcode = Fgpu::BGEUL;
      break;
    case Fgpu::BGTULImmMacro:
      PseudoOpcode = Fgpu::BGTUL;
      break;
    }

    if (loadImmediate(TrgOp.getImm(), TrgReg, Fgpu::NoRegister, !isGP64bit(),
                      false, IDLoc, Out, STI))
      return true;
  }

  switch (PseudoOpcode) {
  case Fgpu::BLT:
  case Fgpu::BLTU:
  case Fgpu::BLTL:
  case Fgpu::BLTUL:
    AcceptsEquality = false;
    ReverseOrderSLT = false;
    IsUnsigned =
        ((PseudoOpcode == Fgpu::BLTU) || (PseudoOpcode == Fgpu::BLTUL));
    IsLikely = ((PseudoOpcode == Fgpu::BLTL) || (PseudoOpcode == Fgpu::BLTUL));
    ZeroSrcOpcode = Fgpu::BGTZ;
    ZeroTrgOpcode = Fgpu::BLTZ;
    break;
  case Fgpu::BLE:
  case Fgpu::BLEU:
  case Fgpu::BLEL:
  case Fgpu::BLEUL:
    AcceptsEquality = true;
    ReverseOrderSLT = true;
    IsUnsigned =
        ((PseudoOpcode == Fgpu::BLEU) || (PseudoOpcode == Fgpu::BLEUL));
    IsLikely = ((PseudoOpcode == Fgpu::BLEL) || (PseudoOpcode == Fgpu::BLEUL));
    ZeroSrcOpcode = Fgpu::BGEZ;
    ZeroTrgOpcode = Fgpu::BLEZ;
    break;
  case Fgpu::BGE:
  case Fgpu::BGEU:
  case Fgpu::BGEL:
  case Fgpu::BGEUL:
    AcceptsEquality = true;
    ReverseOrderSLT = false;
    IsUnsigned =
        ((PseudoOpcode == Fgpu::BGEU) || (PseudoOpcode == Fgpu::BGEUL));
    IsLikely = ((PseudoOpcode == Fgpu::BGEL) || (PseudoOpcode == Fgpu::BGEUL));
    ZeroSrcOpcode = Fgpu::BLEZ;
    ZeroTrgOpcode = Fgpu::BGEZ;
    break;
  case Fgpu::BGT:
  case Fgpu::BGTU:
  case Fgpu::BGTL:
  case Fgpu::BGTUL:
    AcceptsEquality = false;
    ReverseOrderSLT = true;
    IsUnsigned =
        ((PseudoOpcode == Fgpu::BGTU) || (PseudoOpcode == Fgpu::BGTUL));
    IsLikely = ((PseudoOpcode == Fgpu::BGTL) || (PseudoOpcode == Fgpu::BGTUL));
    ZeroSrcOpcode = Fgpu::BLTZ;
    ZeroTrgOpcode = Fgpu::BGTZ;
    break;
  default:
    llvm_unreachable("unknown opcode for branch pseudo-instruction");
  }

  bool IsTrgRegZero = (TrgReg == Fgpu::ZERO);
  bool IsSrcRegZero = (SrcReg == Fgpu::ZERO);
  if (IsSrcRegZero && IsTrgRegZero) {
    // FIXME: All of these Opcode-specific if's are needed for compatibility
    // with GAS' behaviour. However, they may not generate the most efficient
    // code in some circumstances.
    if (PseudoOpcode == Fgpu::BLT) {
      TOut.emitRX(Fgpu::BLTZ, Fgpu::ZERO, MCOperand::createExpr(OffsetExpr),
                  IDLoc, STI);
      return false;
    }
    if (PseudoOpcode == Fgpu::BLE) {
      TOut.emitRX(Fgpu::BLEZ, Fgpu::ZERO, MCOperand::createExpr(OffsetExpr),
                  IDLoc, STI);
      Warning(IDLoc, "branch is always taken");
      return false;
    }
    if (PseudoOpcode == Fgpu::BGE) {
      TOut.emitRX(Fgpu::BGEZ, Fgpu::ZERO, MCOperand::createExpr(OffsetExpr),
                  IDLoc, STI);
      Warning(IDLoc, "branch is always taken");
      return false;
    }
    if (PseudoOpcode == Fgpu::BGT) {
      TOut.emitRX(Fgpu::BGTZ, Fgpu::ZERO, MCOperand::createExpr(OffsetExpr),
                  IDLoc, STI);
      return false;
    }
    if (PseudoOpcode == Fgpu::BGTU) {
      TOut.emitRRX(Fgpu::BNE, Fgpu::ZERO, Fgpu::ZERO,
                   MCOperand::createExpr(OffsetExpr), IDLoc, STI);
      return false;
    }
    if (AcceptsEquality) {
      // If both registers are $0 and the pseudo-branch accepts equality, it
      // will always be taken, so we emit an unconditional branch.
      TOut.emitRRX(Fgpu::BEQ, Fgpu::ZERO, Fgpu::ZERO,
                   MCOperand::createExpr(OffsetExpr), IDLoc, STI);
      Warning(IDLoc, "branch is always taken");
      return false;
    }
    // If both registers are $0 and the pseudo-branch does not accept
    // equality, it will never be taken, so we don't have to emit anything.
    return false;
  }
  if (IsSrcRegZero || IsTrgRegZero) {
    if ((IsSrcRegZero && PseudoOpcode == Fgpu::BGTU) ||
        (IsTrgRegZero && PseudoOpcode == Fgpu::BLTU)) {
      // If the $rs is $0 and the pseudo-branch is BGTU (0 > x) or
      // if the $rt is $0 and the pseudo-branch is BLTU (x < 0),
      // the pseudo-branch will never be taken, so we don't emit anything.
      // This only applies to unsigned pseudo-branches.
      return false;
    }
    if ((IsSrcRegZero && PseudoOpcode == Fgpu::BLEU) ||
        (IsTrgRegZero && PseudoOpcode == Fgpu::BGEU)) {
      // If the $rs is $0 and the pseudo-branch is BLEU (0 <= x) or
      // if the $rt is $0 and the pseudo-branch is BGEU (x >= 0),
      // the pseudo-branch will always be taken, so we emit an unconditional
      // branch.
      // This only applies to unsigned pseudo-branches.
      TOut.emitRRX(Fgpu::BEQ, Fgpu::ZERO, Fgpu::ZERO,
                   MCOperand::createExpr(OffsetExpr), IDLoc, STI);
      Warning(IDLoc, "branch is always taken");
      return false;
    }
    if (IsUnsigned) {
      // If the $rs is $0 and the pseudo-branch is BLTU (0 < x) or
      // if the $rt is $0 and the pseudo-branch is BGTU (x > 0),
      // the pseudo-branch will be taken only when the non-zero register is
      // different from 0, so we emit a BNEZ.
      //
      // If the $rs is $0 and the pseudo-branch is BGEU (0 >= x) or
      // if the $rt is $0 and the pseudo-branch is BLEU (x <= 0),
      // the pseudo-branch will be taken only when the non-zero register is
      // equal to 0, so we emit a BEQZ.
      //
      // Because only BLEU and BGEU branch on equality, we can use the
      // AcceptsEquality variable to decide when to emit the BEQZ.
      TOut.emitRRX(AcceptsEquality ? Fgpu::BEQ : Fgpu::BNE,
                   IsSrcRegZero ? TrgReg : SrcReg, Fgpu::ZERO,
                   MCOperand::createExpr(OffsetExpr), IDLoc, STI);
      return false;
    }
    // If we have a signed pseudo-branch and one of the registers is $0,
    // we can use an appropriate compare-to-zero branch. We select which one
    // to use in the switch statement above.
    TOut.emitRX(IsSrcRegZero ? ZeroSrcOpcode : ZeroTrgOpcode,
                IsSrcRegZero ? TrgReg : SrcReg,
                MCOperand::createExpr(OffsetExpr), IDLoc, STI);
    return false;
  }

  // If neither the SrcReg nor the TrgReg are $0, we need AT to perform the
  // expansions. If it is not available, we return.
  unsigned ATRegNum = getATReg(IDLoc);
  if (!ATRegNum)
    return true;

  if (!EmittedNoMacroWarning)
    warnIfNoMacro(IDLoc);

  // SLT fits well with 2 of our 4 pseudo-branches:
  //   BLT, where $rs < $rt, translates into "slt $at, $rs, $rt" and
  //   BGT, where $rs > $rt, translates into "slt $at, $rt, $rs".
  // If the result of the SLT is 1, we branch, and if it's 0, we don't.
  // This is accomplished by using a BNEZ with the result of the SLT.
  //
  // The other 2 pseudo-branches are opposites of the above 2 (BGE with BLT
  // and BLE with BGT), so we change the BNEZ into a BEQZ.
  // Because only BGE and BLE branch on equality, we can use the
  // AcceptsEquality variable to decide when to emit the BEQZ.
  // Note that the order of the SLT arguments doesn't change between
  // opposites.
  //
  // The same applies to the unsigned variants, except that SLTu is used
  // instead of SLT.
  TOut.emitRRR(IsUnsigned ? Fgpu::SLTu : Fgpu::SLT, ATRegNum,
               ReverseOrderSLT ? TrgReg : SrcReg,
               ReverseOrderSLT ? SrcReg : TrgReg, IDLoc, STI);

  TOut.emitRRX(IsLikely ? (AcceptsEquality ? Fgpu::BEQL : Fgpu::BNEL)
                        : (AcceptsEquality ? Fgpu::BEQ : Fgpu::BNE),
               ATRegNum, Fgpu::ZERO, MCOperand::createExpr(OffsetExpr), IDLoc,
               STI);
  return false;
}

// Expand a integer division macro.
//
// Notably we don't have to emit a warning when encountering $rt as the $zero
// register, or 0 as an immediate. processInstruction() has already done that.
//
// The destination register can only be $zero when expanding (S)DivIMacro or
// D(S)DivMacro.

bool FgpuAsmParser::expandDivRem(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                 const MCSubtargetInfo *STI,
                                 const bool IsFgpu64, const bool Signed) {
  FgpuTargetStreamer &TOut = getTargetStreamer();

  warnIfNoMacro(IDLoc);

  const MCOperand &RdRegOp = Inst.getOperand(0);
  assert(RdRegOp.isReg() && "expected register operand kind");
  unsigned RdReg = RdRegOp.getReg();

  const MCOperand &RsRegOp = Inst.getOperand(1);
  assert(RsRegOp.isReg() && "expected register operand kind");
  unsigned RsReg = RsRegOp.getReg();

  unsigned RtReg;
  int64_t ImmValue;

  const MCOperand &RtOp = Inst.getOperand(2);
  assert((RtOp.isReg() || RtOp.isImm()) &&
         "expected register or immediate operand kind");
  if (RtOp.isReg())
    RtReg = RtOp.getReg();
  else
    ImmValue = RtOp.getImm();

  unsigned DivOp;
  unsigned ZeroReg;
  unsigned SubOp;

  if (IsFgpu64) {
    DivOp = Signed ? Fgpu::DSDIV : Fgpu::DUDIV;
    ZeroReg = Fgpu::ZERO_64;
    SubOp = Fgpu::DSUB;
  } else {
    DivOp = Signed ? Fgpu::SDIV : Fgpu::UDIV;
    ZeroReg = Fgpu::ZERO;
    SubOp = Fgpu::SUB;
  }

  bool UseTraps = useTraps();

  unsigned Opcode = Inst.getOpcode();
  bool isDiv = Opcode == Fgpu::SDivMacro || Opcode == Fgpu::SDivIMacro ||
               Opcode == Fgpu::UDivMacro || Opcode == Fgpu::UDivIMacro ||
               Opcode == Fgpu::DSDivMacro || Opcode == Fgpu::DSDivIMacro ||
               Opcode == Fgpu::DUDivMacro || Opcode == Fgpu::DUDivIMacro;

  bool isRem = Opcode == Fgpu::SRemMacro || Opcode == Fgpu::SRemIMacro ||
               Opcode == Fgpu::URemMacro || Opcode == Fgpu::URemIMacro ||
               Opcode == Fgpu::DSRemMacro || Opcode == Fgpu::DSRemIMacro ||
               Opcode == Fgpu::DURemMacro || Opcode == Fgpu::DURemIMacro;

  if (RtOp.isImm()) {
    unsigned ATReg = getATReg(IDLoc);
    if (!ATReg)
      return true;

    if (ImmValue == 0) {
      if (UseTraps)
        TOut.emitRRI(Fgpu::TEQ, ZeroReg, ZeroReg, 0x7, IDLoc, STI);
      else
        TOut.emitII(Fgpu::BREAK, 0x7, 0, IDLoc, STI);
      return false;
    }

    if (isRem && (ImmValue == 1 || (Signed && (ImmValue == -1)))) {
      TOut.emitRRR(Fgpu::OR, RdReg, ZeroReg, ZeroReg, IDLoc, STI);
      return false;
    } else if (isDiv && ImmValue == 1) {
      TOut.emitRRR(Fgpu::OR, RdReg, RsReg, Fgpu::ZERO, IDLoc, STI);
      return false;
    } else if (isDiv && Signed && ImmValue == -1) {
      TOut.emitRRR(SubOp, RdReg, ZeroReg, RsReg, IDLoc, STI);
      return false;
    } else {
      if (loadImmediate(ImmValue, ATReg, Fgpu::NoRegister, isInt<32>(ImmValue),
                        false, Inst.getLoc(), Out, STI))
        return true;
      TOut.emitRR(DivOp, RsReg, ATReg, IDLoc, STI);
      TOut.emitR(isDiv ? Fgpu::MFLO : Fgpu::MFHI, RdReg, IDLoc, STI);
      return false;
    }
    return true;
  }

  // If the macro expansion of (d)div(u) or (d)rem(u) would always trap or
  // break, insert the trap/break and exit. This gives a different result to
  // GAS. GAS has an inconsistency/missed optimization in that not all cases
  // are handled equivalently. As the observed behaviour is the same, we're ok.
  if (RtReg == Fgpu::ZERO || RtReg == Fgpu::ZERO_64) {
    if (UseTraps) {
      TOut.emitRRI(Fgpu::TEQ, ZeroReg, ZeroReg, 0x7, IDLoc, STI);
      return false;
    }
    TOut.emitII(Fgpu::BREAK, 0x7, 0, IDLoc, STI);
    return false;
  }

  // (d)rem(u) $0, $X, $Y is a special case. Like div $zero, $X, $Y, it does
  // not expand to macro sequence.
  if (isRem && (RdReg == Fgpu::ZERO || RdReg == Fgpu::ZERO_64)) {
    TOut.emitRR(DivOp, RsReg, RtReg, IDLoc, STI);
    return false;
  }

  // Temporary label for first branch traget
  MCContext &Context = TOut.getStreamer().getContext();
  MCSymbol *BrTarget;
  MCOperand LabelOp;

  if (UseTraps) {
    TOut.emitRRI(Fgpu::TEQ, RtReg, ZeroReg, 0x7, IDLoc, STI);
  } else {
    // Branch to the li instruction.
    BrTarget = Context.createTempSymbol();
    LabelOp = MCOperand::createExpr(MCSymbolRefExpr::create(BrTarget, Context));
    TOut.emitRRX(Fgpu::BNE, RtReg, ZeroReg, LabelOp, IDLoc, STI);
  }

  TOut.emitRR(DivOp, RsReg, RtReg, IDLoc, STI);

  if (!UseTraps)
    TOut.emitII(Fgpu::BREAK, 0x7, 0, IDLoc, STI);

  if (!Signed) {
    if (!UseTraps)
      TOut.getStreamer().emitLabel(BrTarget);

    TOut.emitR(isDiv ? Fgpu::MFLO : Fgpu::MFHI, RdReg, IDLoc, STI);
    return false;
  }

  unsigned ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  if (!UseTraps)
    TOut.getStreamer().emitLabel(BrTarget);

  TOut.emitRRI(Fgpu::ADDiu, ATReg, ZeroReg, -1, IDLoc, STI);

  // Temporary label for the second branch target.
  MCSymbol *BrTargetEnd = Context.createTempSymbol();
  MCOperand LabelOpEnd =
      MCOperand::createExpr(MCSymbolRefExpr::create(BrTargetEnd, Context));

  // Branch to the mflo instruction.
  TOut.emitRRX(Fgpu::BNE, RtReg, ATReg, LabelOpEnd, IDLoc, STI);

  if (IsFgpu64) {
    TOut.emitRRI(Fgpu::ADDiu, ATReg, ZeroReg, 1, IDLoc, STI);
    TOut.emitDSLL(ATReg, ATReg, 63, IDLoc, STI);
  } else {
    TOut.emitRI(Fgpu::LUi, ATReg, (uint16_t)0x8000, IDLoc, STI);
  }

  if (UseTraps)
    TOut.emitRRI(Fgpu::TEQ, RsReg, ATReg, 0x6, IDLoc, STI);
  else {
    // Branch to the mflo instruction.
    TOut.emitRRX(Fgpu::BNE, RsReg, ATReg, LabelOpEnd, IDLoc, STI);
    TOut.emitNop(IDLoc, STI);
    TOut.emitII(Fgpu::BREAK, 0x6, 0, IDLoc, STI);
  }

  TOut.getStreamer().emitLabel(BrTargetEnd);
  TOut.emitR(isDiv ? Fgpu::MFLO : Fgpu::MFHI, RdReg, IDLoc, STI);
  return false;
}

bool FgpuAsmParser::expandTrunc(MCInst &Inst, bool IsDouble, bool Is64FPU,
                                SMLoc IDLoc, MCStreamer &Out,
                                const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();

  assert(Inst.getNumOperands() == 3 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() && Inst.getOperand(1).isReg() &&
         Inst.getOperand(2).isReg() && "Invalid instruction operand.");

  unsigned FirstReg = Inst.getOperand(0).getReg();
  unsigned SecondReg = Inst.getOperand(1).getReg();
  unsigned ThirdReg = Inst.getOperand(2).getReg();

  if (hasFgpu1() && !hasFgpu2()) {
    unsigned ATReg = getATReg(IDLoc);
    if (!ATReg)
      return true;
    TOut.emitRR(Fgpu::CFC1, ThirdReg, Fgpu::RA, IDLoc, STI);
    TOut.emitRR(Fgpu::CFC1, ThirdReg, Fgpu::RA, IDLoc, STI);
    TOut.emitNop(IDLoc, STI);
    TOut.emitRRI(Fgpu::ORi, ATReg, ThirdReg, 0x3, IDLoc, STI);
    TOut.emitRRI(Fgpu::XORi, ATReg, ATReg, 0x2, IDLoc, STI);
    TOut.emitRR(Fgpu::CTC1, Fgpu::RA, ATReg, IDLoc, STI);
    TOut.emitNop(IDLoc, STI);
    TOut.emitRR(IsDouble ? (Is64FPU ? Fgpu::CVT_W_D64 : Fgpu::CVT_W_D32)
                         : Fgpu::CVT_W_S,
                FirstReg, SecondReg, IDLoc, STI);
    TOut.emitRR(Fgpu::CTC1, Fgpu::RA, ThirdReg, IDLoc, STI);
    TOut.emitNop(IDLoc, STI);
    return false;
  }

  TOut.emitRR(IsDouble ? (Is64FPU ? Fgpu::TRUNC_W_D64 : Fgpu::TRUNC_W_D32)
                       : Fgpu::TRUNC_W_S,
              FirstReg, SecondReg, IDLoc, STI);

  return false;
}

bool FgpuAsmParser::expandUlh(MCInst &Inst, bool Signed, SMLoc IDLoc,
                              MCStreamer &Out, const MCSubtargetInfo *STI) {
  if (hasFgpu32r6() || hasFgpu64r6()) {
    return Error(IDLoc, "instruction not supported on fgpu32r6 or fgpu64r6");
  }

  const MCOperand &DstRegOp = Inst.getOperand(0);
  assert(DstRegOp.isReg() && "expected register operand kind");
  const MCOperand &SrcRegOp = Inst.getOperand(1);
  assert(SrcRegOp.isReg() && "expected register operand kind");
  const MCOperand &OffsetImmOp = Inst.getOperand(2);
  assert(OffsetImmOp.isImm() && "expected immediate operand kind");

  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned DstReg = DstRegOp.getReg();
  unsigned SrcReg = SrcRegOp.getReg();
  int64_t OffsetValue = OffsetImmOp.getImm();

  // NOTE: We always need AT for ULHU, as it is always used as the source
  // register for one of the LBu's.
  warnIfNoMacro(IDLoc);
  unsigned ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  bool IsLargeOffset = !(isInt<16>(OffsetValue + 1) && isInt<16>(OffsetValue));
  if (IsLargeOffset) {
    if (loadImmediate(OffsetValue, ATReg, SrcReg, !ABI.ArePtrs64bit(), true,
                      IDLoc, Out, STI))
      return true;
  }

  int64_t FirstOffset = IsLargeOffset ? 0 : OffsetValue;
  int64_t SecondOffset = IsLargeOffset ? 1 : (OffsetValue + 1);
  if (isLittle())
    std::swap(FirstOffset, SecondOffset);

  unsigned FirstLbuDstReg = IsLargeOffset ? DstReg : ATReg;
  unsigned SecondLbuDstReg = IsLargeOffset ? ATReg : DstReg;

  unsigned LbuSrcReg = IsLargeOffset ? ATReg : SrcReg;
  unsigned SllReg = IsLargeOffset ? DstReg : ATReg;

  TOut.emitRRI(Signed ? Fgpu::LB : Fgpu::LBu, FirstLbuDstReg, LbuSrcReg,
               FirstOffset, IDLoc, STI);
  TOut.emitRRI(Fgpu::LBu, SecondLbuDstReg, LbuSrcReg, SecondOffset, IDLoc, STI);
  TOut.emitRRI(Fgpu::SLL, SllReg, SllReg, 8, IDLoc, STI);
  TOut.emitRRR(Fgpu::OR, DstReg, DstReg, ATReg, IDLoc, STI);

  return false;
}

bool FgpuAsmParser::expandUsh(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                              const MCSubtargetInfo *STI) {
  if (hasFgpu32r6() || hasFgpu64r6()) {
    return Error(IDLoc, "instruction not supported on fgpu32r6 or fgpu64r6");
  }

  const MCOperand &DstRegOp = Inst.getOperand(0);
  assert(DstRegOp.isReg() && "expected register operand kind");
  const MCOperand &SrcRegOp = Inst.getOperand(1);
  assert(SrcRegOp.isReg() && "expected register operand kind");
  const MCOperand &OffsetImmOp = Inst.getOperand(2);
  assert(OffsetImmOp.isImm() && "expected immediate operand kind");

  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned DstReg = DstRegOp.getReg();
  unsigned SrcReg = SrcRegOp.getReg();
  int64_t OffsetValue = OffsetImmOp.getImm();

  warnIfNoMacro(IDLoc);
  unsigned ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  bool IsLargeOffset = !(isInt<16>(OffsetValue + 1) && isInt<16>(OffsetValue));
  if (IsLargeOffset) {
    if (loadImmediate(OffsetValue, ATReg, SrcReg, !ABI.ArePtrs64bit(), true,
                      IDLoc, Out, STI))
      return true;
  }

  int64_t FirstOffset = IsLargeOffset ? 1 : (OffsetValue + 1);
  int64_t SecondOffset = IsLargeOffset ? 0 : OffsetValue;
  if (isLittle())
    std::swap(FirstOffset, SecondOffset);

  if (IsLargeOffset) {
    TOut.emitRRI(Fgpu::SB, DstReg, ATReg, FirstOffset, IDLoc, STI);
    TOut.emitRRI(Fgpu::SRL, DstReg, DstReg, 8, IDLoc, STI);
    TOut.emitRRI(Fgpu::SB, DstReg, ATReg, SecondOffset, IDLoc, STI);
    TOut.emitRRI(Fgpu::LBu, ATReg, ATReg, 0, IDLoc, STI);
    TOut.emitRRI(Fgpu::SLL, DstReg, DstReg, 8, IDLoc, STI);
    TOut.emitRRR(Fgpu::OR, DstReg, DstReg, ATReg, IDLoc, STI);
  } else {
    TOut.emitRRI(Fgpu::SB, DstReg, SrcReg, FirstOffset, IDLoc, STI);
    TOut.emitRRI(Fgpu::SRL, ATReg, DstReg, 8, IDLoc, STI);
    TOut.emitRRI(Fgpu::SB, ATReg, SrcReg, SecondOffset, IDLoc, STI);
  }

  return false;
}

bool FgpuAsmParser::expandUxw(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                              const MCSubtargetInfo *STI) {
  if (hasFgpu32r6() || hasFgpu64r6()) {
    return Error(IDLoc, "instruction not supported on fgpu32r6 or fgpu64r6");
  }

  const MCOperand &DstRegOp = Inst.getOperand(0);
  assert(DstRegOp.isReg() && "expected register operand kind");
  const MCOperand &SrcRegOp = Inst.getOperand(1);
  assert(SrcRegOp.isReg() && "expected register operand kind");
  const MCOperand &OffsetImmOp = Inst.getOperand(2);
  assert(OffsetImmOp.isImm() && "expected immediate operand kind");

  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned DstReg = DstRegOp.getReg();
  unsigned SrcReg = SrcRegOp.getReg();
  int64_t OffsetValue = OffsetImmOp.getImm();

  // Compute left/right load/store offsets.
  bool IsLargeOffset = !(isInt<16>(OffsetValue + 3) && isInt<16>(OffsetValue));
  int64_t LxlOffset = IsLargeOffset ? 0 : OffsetValue;
  int64_t LxrOffset = IsLargeOffset ? 3 : (OffsetValue + 3);
  if (isLittle())
    std::swap(LxlOffset, LxrOffset);

  bool IsLoadInst = (Inst.getOpcode() == Fgpu::Ulw);
  bool DoMove = IsLoadInst && (SrcReg == DstReg) && !IsLargeOffset;
  unsigned TmpReg = SrcReg;
  if (IsLargeOffset || DoMove) {
    warnIfNoMacro(IDLoc);
    TmpReg = getATReg(IDLoc);
    if (!TmpReg)
      return true;
  }

  if (IsLargeOffset) {
    if (loadImmediate(OffsetValue, TmpReg, SrcReg, !ABI.ArePtrs64bit(), true,
                      IDLoc, Out, STI))
      return true;
  }

  if (DoMove)
    std::swap(DstReg, TmpReg);

  unsigned XWL = IsLoadInst ? Fgpu::LWL : Fgpu::SWL;
  unsigned XWR = IsLoadInst ? Fgpu::LWR : Fgpu::SWR;
  TOut.emitRRI(XWL, DstReg, TmpReg, LxlOffset, IDLoc, STI);
  TOut.emitRRI(XWR, DstReg, TmpReg, LxrOffset, IDLoc, STI);

  if (DoMove)
    TOut.emitRRR(Fgpu::OR, TmpReg, DstReg, Fgpu::ZERO, IDLoc, STI);

  return false;
}

bool FgpuAsmParser::expandSge(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                              const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();

  assert(Inst.getNumOperands() == 3 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() &&
         Inst.getOperand(1).isReg() &&
         Inst.getOperand(2).isReg() && "Invalid instruction operand.");

  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  unsigned OpReg = Inst.getOperand(2).getReg();
  unsigned OpCode;

  warnIfNoMacro(IDLoc);

  switch (Inst.getOpcode()) {
  case Fgpu::SGE:
    OpCode = Fgpu::SLT;
    break;
  case Fgpu::SGEU:
    OpCode = Fgpu::SLTu;
    break;
  default:
    llvm_unreachable("unexpected 'sge' opcode");
  }

  // $SrcReg >= $OpReg is equal to (not ($SrcReg < $OpReg))
  TOut.emitRRR(OpCode, DstReg, SrcReg, OpReg, IDLoc, STI);
  TOut.emitRRI(Fgpu::XORi, DstReg, DstReg, 1, IDLoc, STI);

  return false;
}

bool FgpuAsmParser::expandSgeImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                 const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();

  assert(Inst.getNumOperands() == 3 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() &&
         Inst.getOperand(1).isReg() &&
         Inst.getOperand(2).isImm() && "Invalid instruction operand.");

  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  int64_t ImmValue = Inst.getOperand(2).getImm();
  unsigned OpRegCode, OpImmCode;

  warnIfNoMacro(IDLoc);

  switch (Inst.getOpcode()) {
  case Fgpu::SGEImm:
  case Fgpu::SGEImm64:
    OpRegCode = Fgpu::SLT;
    OpImmCode = Fgpu::SLTi;
    break;
  case Fgpu::SGEUImm:
  case Fgpu::SGEUImm64:
    OpRegCode = Fgpu::SLTu;
    OpImmCode = Fgpu::SLTiu;
    break;
  default:
    llvm_unreachable("unexpected 'sge' opcode with immediate");
  }

  // $SrcReg >= Imm is equal to (not ($SrcReg < Imm))
  if (isInt<16>(ImmValue)) {
    // Use immediate version of STL.
    TOut.emitRRI(OpImmCode, DstReg, SrcReg, ImmValue, IDLoc, STI);
    TOut.emitRRI(Fgpu::XORi, DstReg, DstReg, 1, IDLoc, STI);
  } else {
    unsigned ImmReg = DstReg;
    if (DstReg == SrcReg) {
      unsigned ATReg = getATReg(Inst.getLoc());
      if (!ATReg)
        return true;
      ImmReg = ATReg;
    }

    if (loadImmediate(ImmValue, ImmReg, Fgpu::NoRegister, isInt<32>(ImmValue),
                      false, IDLoc, Out, STI))
      return true;

    TOut.emitRRR(OpRegCode, DstReg, SrcReg, ImmReg, IDLoc, STI);
    TOut.emitRRI(Fgpu::XORi, DstReg, DstReg, 1, IDLoc, STI);
  }

  return false;
}

bool FgpuAsmParser::expandSgtImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                 const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();

  assert(Inst.getNumOperands() == 3 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() &&
         Inst.getOperand(1).isReg() &&
         Inst.getOperand(2).isImm() && "Invalid instruction operand.");

  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  unsigned ImmReg = DstReg;
  int64_t ImmValue = Inst.getOperand(2).getImm();
  unsigned OpCode;

  warnIfNoMacro(IDLoc);

  switch (Inst.getOpcode()) {
  case Fgpu::SGTImm:
  case Fgpu::SGTImm64:
    OpCode = Fgpu::SLT;
    break;
  case Fgpu::SGTUImm:
  case Fgpu::SGTUImm64:
    OpCode = Fgpu::SLTu;
    break;
  default:
    llvm_unreachable("unexpected 'sgt' opcode with immediate");
  }

  if (DstReg == SrcReg) {
    unsigned ATReg = getATReg(Inst.getLoc());
    if (!ATReg)
      return true;
    ImmReg = ATReg;
  }

  if (loadImmediate(ImmValue, ImmReg, Fgpu::NoRegister, isInt<32>(ImmValue),
                    false, IDLoc, Out, STI))
    return true;

  // $SrcReg > $ImmReg is equal to $ImmReg < $SrcReg
  TOut.emitRRR(OpCode, DstReg, ImmReg, SrcReg, IDLoc, STI);

  return false;
}

bool FgpuAsmParser::expandSle(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                              const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();

  assert(Inst.getNumOperands() == 3 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() &&
         Inst.getOperand(1).isReg() &&
         Inst.getOperand(2).isReg() && "Invalid instruction operand.");

  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  unsigned OpReg = Inst.getOperand(2).getReg();
  unsigned OpCode;

  warnIfNoMacro(IDLoc);

  switch (Inst.getOpcode()) {
  case Fgpu::SLE:
    OpCode = Fgpu::SLT;
    break;
  case Fgpu::SLEU:
    OpCode = Fgpu::SLTu;
    break;
  default:
    llvm_unreachable("unexpected 'sge' opcode");
  }

  // $SrcReg <= $OpReg is equal to (not ($OpReg < $SrcReg))
  TOut.emitRRR(OpCode, DstReg, OpReg, SrcReg, IDLoc, STI);
  TOut.emitRRI(Fgpu::XORi, DstReg, DstReg, 1, IDLoc, STI);

  return false;
}

bool FgpuAsmParser::expandSleImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                 const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();

  assert(Inst.getNumOperands() == 3 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() &&
         Inst.getOperand(1).isReg() &&
         Inst.getOperand(2).isImm() && "Invalid instruction operand.");

  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  int64_t ImmValue = Inst.getOperand(2).getImm();
  unsigned OpRegCode;

  warnIfNoMacro(IDLoc);

  switch (Inst.getOpcode()) {
  case Fgpu::SLEImm:
  case Fgpu::SLEImm64:
    OpRegCode = Fgpu::SLT;
    break;
  case Fgpu::SLEUImm:
  case Fgpu::SLEUImm64:
    OpRegCode = Fgpu::SLTu;
    break;
  default:
    llvm_unreachable("unexpected 'sge' opcode with immediate");
  }

  // $SrcReg <= Imm is equal to (not (Imm < $SrcReg))
  unsigned ImmReg = DstReg;
  if (DstReg == SrcReg) {
    unsigned ATReg = getATReg(Inst.getLoc());
    if (!ATReg)
      return true;
    ImmReg = ATReg;
  }

  if (loadImmediate(ImmValue, ImmReg, Fgpu::NoRegister, isInt<32>(ImmValue),
                    false, IDLoc, Out, STI))
    return true;

  TOut.emitRRR(OpRegCode, DstReg, ImmReg, SrcReg, IDLoc, STI);
  TOut.emitRRI(Fgpu::XORi, DstReg, DstReg, 1, IDLoc, STI);

  return false;
}

bool FgpuAsmParser::expandAliasImmediate(MCInst &Inst, SMLoc IDLoc,
                                         MCStreamer &Out,
                                         const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();

  assert(Inst.getNumOperands() == 3 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() &&
         Inst.getOperand(1).isReg() &&
         Inst.getOperand(2).isImm() && "Invalid instruction operand.");

  unsigned ATReg = Fgpu::NoRegister;
  unsigned FinalDstReg = Fgpu::NoRegister;
  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  int64_t ImmValue = Inst.getOperand(2).getImm();

  bool Is32Bit = isInt<32>(ImmValue) || (!isGP64bit() && isUInt<32>(ImmValue));

  unsigned FinalOpcode = Inst.getOpcode();

  if (DstReg == SrcReg) {
    ATReg = getATReg(Inst.getLoc());
    if (!ATReg)
      return true;
    FinalDstReg = DstReg;
    DstReg = ATReg;
  }

  if (!loadImmediate(ImmValue, DstReg, Fgpu::NoRegister, Is32Bit, false,
                     Inst.getLoc(), Out, STI)) {
    switch (FinalOpcode) {
    default:
      llvm_unreachable("unimplemented expansion");
    case Fgpu::ADDi:
      FinalOpcode = Fgpu::ADD;
      break;
    case Fgpu::ADDiu:
      FinalOpcode = Fgpu::ADDu;
      break;
    case Fgpu::ANDi:
      FinalOpcode = Fgpu::AND;
      break;
    case Fgpu::NORImm:
      FinalOpcode = Fgpu::NOR;
      break;
    case Fgpu::ORi:
      FinalOpcode = Fgpu::OR;
      break;
    case Fgpu::SLTi:
      FinalOpcode = Fgpu::SLT;
      break;
    case Fgpu::SLTiu:
      FinalOpcode = Fgpu::SLTu;
      break;
    case Fgpu::XORi:
      FinalOpcode = Fgpu::XOR;
      break;
    case Fgpu::ADDi_MM:
      FinalOpcode = Fgpu::ADD_MM;
      break;
    case Fgpu::ADDiu_MM:
      FinalOpcode = Fgpu::ADDu_MM;
      break;
    case Fgpu::ANDi_MM:
      FinalOpcode = Fgpu::AND_MM;
      break;
    case Fgpu::ORi_MM:
      FinalOpcode = Fgpu::OR_MM;
      break;
    case Fgpu::SLTi_MM:
      FinalOpcode = Fgpu::SLT_MM;
      break;
    case Fgpu::SLTiu_MM:
      FinalOpcode = Fgpu::SLTu_MM;
      break;
    case Fgpu::XORi_MM:
      FinalOpcode = Fgpu::XOR_MM;
      break;
    case Fgpu::ANDi64:
      FinalOpcode = Fgpu::AND64;
      break;
    case Fgpu::NORImm64:
      FinalOpcode = Fgpu::NOR64;
      break;
    case Fgpu::ORi64:
      FinalOpcode = Fgpu::OR64;
      break;
    case Fgpu::SLTImm64:
      FinalOpcode = Fgpu::SLT64;
      break;
    case Fgpu::SLTUImm64:
      FinalOpcode = Fgpu::SLTu64;
      break;
    case Fgpu::XORi64:
      FinalOpcode = Fgpu::XOR64;
      break;
    }

    if (FinalDstReg == Fgpu::NoRegister)
      TOut.emitRRR(FinalOpcode, DstReg, DstReg, SrcReg, IDLoc, STI);
    else
      TOut.emitRRR(FinalOpcode, FinalDstReg, FinalDstReg, DstReg, IDLoc, STI);
    return false;
  }
  return true;
}

bool FgpuAsmParser::expandRotation(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                   const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned ATReg = Fgpu::NoRegister;
  unsigned DReg = Inst.getOperand(0).getReg();
  unsigned SReg = Inst.getOperand(1).getReg();
  unsigned TReg = Inst.getOperand(2).getReg();
  unsigned TmpReg = DReg;

  unsigned FirstShift = Fgpu::NOP;
  unsigned SecondShift = Fgpu::NOP;

  if (hasFgpu32r2()) {
    if (DReg == SReg) {
      TmpReg = getATReg(Inst.getLoc());
      if (!TmpReg)
        return true;
    }

    if (Inst.getOpcode() == Fgpu::ROL) {
      TOut.emitRRR(Fgpu::SUBu, TmpReg, Fgpu::ZERO, TReg, Inst.getLoc(), STI);
      TOut.emitRRR(Fgpu::ROTRV, DReg, SReg, TmpReg, Inst.getLoc(), STI);
      return false;
    }

    if (Inst.getOpcode() == Fgpu::ROR) {
      TOut.emitRRR(Fgpu::ROTRV, DReg, SReg, TReg, Inst.getLoc(), STI);
      return false;
    }

    return true;
  }

  if (hasFgpu32()) {
    switch (Inst.getOpcode()) {
    default:
      llvm_unreachable("unexpected instruction opcode");
    case Fgpu::ROL:
      FirstShift = Fgpu::SRLV;
      SecondShift = Fgpu::SLLV;
      break;
    case Fgpu::ROR:
      FirstShift = Fgpu::SLLV;
      SecondShift = Fgpu::SRLV;
      break;
    }

    ATReg = getATReg(Inst.getLoc());
    if (!ATReg)
      return true;

    TOut.emitRRR(Fgpu::SUBu, ATReg, Fgpu::ZERO, TReg, Inst.getLoc(), STI);
    TOut.emitRRR(FirstShift, ATReg, SReg, ATReg, Inst.getLoc(), STI);
    TOut.emitRRR(SecondShift, DReg, SReg, TReg, Inst.getLoc(), STI);
    TOut.emitRRR(Fgpu::OR, DReg, DReg, ATReg, Inst.getLoc(), STI);

    return false;
  }

  return true;
}

bool FgpuAsmParser::expandRotationImm(MCInst &Inst, SMLoc IDLoc,
                                      MCStreamer &Out,
                                      const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned ATReg = Fgpu::NoRegister;
  unsigned DReg = Inst.getOperand(0).getReg();
  unsigned SReg = Inst.getOperand(1).getReg();
  int64_t ImmValue = Inst.getOperand(2).getImm();

  unsigned FirstShift = Fgpu::NOP;
  unsigned SecondShift = Fgpu::NOP;

  if (hasFgpu32r2()) {
    if (Inst.getOpcode() == Fgpu::ROLImm) {
      uint64_t MaxShift = 32;
      uint64_t ShiftValue = ImmValue;
      if (ImmValue != 0)
        ShiftValue = MaxShift - ImmValue;
      TOut.emitRRI(Fgpu::ROTR, DReg, SReg, ShiftValue, Inst.getLoc(), STI);
      return false;
    }

    if (Inst.getOpcode() == Fgpu::RORImm) {
      TOut.emitRRI(Fgpu::ROTR, DReg, SReg, ImmValue, Inst.getLoc(), STI);
      return false;
    }

    return true;
  }

  if (hasFgpu32()) {
    if (ImmValue == 0) {
      TOut.emitRRI(Fgpu::SRL, DReg, SReg, 0, Inst.getLoc(), STI);
      return false;
    }

    switch (Inst.getOpcode()) {
    default:
      llvm_unreachable("unexpected instruction opcode");
    case Fgpu::ROLImm:
      FirstShift = Fgpu::SLL;
      SecondShift = Fgpu::SRL;
      break;
    case Fgpu::RORImm:
      FirstShift = Fgpu::SRL;
      SecondShift = Fgpu::SLL;
      break;
    }

    ATReg = getATReg(Inst.getLoc());
    if (!ATReg)
      return true;

    TOut.emitRRI(FirstShift, ATReg, SReg, ImmValue, Inst.getLoc(), STI);
    TOut.emitRRI(SecondShift, DReg, SReg, 32 - ImmValue, Inst.getLoc(), STI);
    TOut.emitRRR(Fgpu::OR, DReg, DReg, ATReg, Inst.getLoc(), STI);

    return false;
  }

  return true;
}

bool FgpuAsmParser::expandDRotation(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                    const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned ATReg = Fgpu::NoRegister;
  unsigned DReg = Inst.getOperand(0).getReg();
  unsigned SReg = Inst.getOperand(1).getReg();
  unsigned TReg = Inst.getOperand(2).getReg();
  unsigned TmpReg = DReg;

  unsigned FirstShift = Fgpu::NOP;
  unsigned SecondShift = Fgpu::NOP;

  if (hasFgpu64r2()) {
    if (TmpReg == SReg) {
      TmpReg = getATReg(Inst.getLoc());
      if (!TmpReg)
        return true;
    }

    if (Inst.getOpcode() == Fgpu::DROL) {
      TOut.emitRRR(Fgpu::DSUBu, TmpReg, Fgpu::ZERO, TReg, Inst.getLoc(), STI);
      TOut.emitRRR(Fgpu::DROTRV, DReg, SReg, TmpReg, Inst.getLoc(), STI);
      return false;
    }

    if (Inst.getOpcode() == Fgpu::DROR) {
      TOut.emitRRR(Fgpu::DROTRV, DReg, SReg, TReg, Inst.getLoc(), STI);
      return false;
    }

    return true;
  }

  if (hasFgpu64()) {
    switch (Inst.getOpcode()) {
    default:
      llvm_unreachable("unexpected instruction opcode");
    case Fgpu::DROL:
      FirstShift = Fgpu::DSRLV;
      SecondShift = Fgpu::DSLLV;
      break;
    case Fgpu::DROR:
      FirstShift = Fgpu::DSLLV;
      SecondShift = Fgpu::DSRLV;
      break;
    }

    ATReg = getATReg(Inst.getLoc());
    if (!ATReg)
      return true;

    TOut.emitRRR(Fgpu::DSUBu, ATReg, Fgpu::ZERO, TReg, Inst.getLoc(), STI);
    TOut.emitRRR(FirstShift, ATReg, SReg, ATReg, Inst.getLoc(), STI);
    TOut.emitRRR(SecondShift, DReg, SReg, TReg, Inst.getLoc(), STI);
    TOut.emitRRR(Fgpu::OR, DReg, DReg, ATReg, Inst.getLoc(), STI);

    return false;
  }

  return true;
}

bool FgpuAsmParser::expandDRotationImm(MCInst &Inst, SMLoc IDLoc,
                                       MCStreamer &Out,
                                       const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned ATReg = Fgpu::NoRegister;
  unsigned DReg = Inst.getOperand(0).getReg();
  unsigned SReg = Inst.getOperand(1).getReg();
  int64_t ImmValue = Inst.getOperand(2).getImm() % 64;

  unsigned FirstShift = Fgpu::NOP;
  unsigned SecondShift = Fgpu::NOP;

  MCInst TmpInst;

  if (hasFgpu64r2()) {
    unsigned FinalOpcode = Fgpu::NOP;
    if (ImmValue == 0)
      FinalOpcode = Fgpu::DROTR;
    else if (ImmValue % 32 == 0)
      FinalOpcode = Fgpu::DROTR32;
    else if ((ImmValue >= 1) && (ImmValue <= 32)) {
      if (Inst.getOpcode() == Fgpu::DROLImm)
        FinalOpcode = Fgpu::DROTR32;
      else
        FinalOpcode = Fgpu::DROTR;
    } else if (ImmValue >= 33) {
      if (Inst.getOpcode() == Fgpu::DROLImm)
        FinalOpcode = Fgpu::DROTR;
      else
        FinalOpcode = Fgpu::DROTR32;
    }

    uint64_t ShiftValue = ImmValue % 32;
    if (Inst.getOpcode() == Fgpu::DROLImm)
      ShiftValue = (32 - ImmValue % 32) % 32;

    TOut.emitRRI(FinalOpcode, DReg, SReg, ShiftValue, Inst.getLoc(), STI);

    return false;
  }

  if (hasFgpu64()) {
    if (ImmValue == 0) {
      TOut.emitRRI(Fgpu::DSRL, DReg, SReg, 0, Inst.getLoc(), STI);
      return false;
    }

    switch (Inst.getOpcode()) {
    default:
      llvm_unreachable("unexpected instruction opcode");
    case Fgpu::DROLImm:
      if ((ImmValue >= 1) && (ImmValue <= 31)) {
        FirstShift = Fgpu::DSLL;
        SecondShift = Fgpu::DSRL32;
      }
      if (ImmValue == 32) {
        FirstShift = Fgpu::DSLL32;
        SecondShift = Fgpu::DSRL32;
      }
      if ((ImmValue >= 33) && (ImmValue <= 63)) {
        FirstShift = Fgpu::DSLL32;
        SecondShift = Fgpu::DSRL;
      }
      break;
    case Fgpu::DRORImm:
      if ((ImmValue >= 1) && (ImmValue <= 31)) {
        FirstShift = Fgpu::DSRL;
        SecondShift = Fgpu::DSLL32;
      }
      if (ImmValue == 32) {
        FirstShift = Fgpu::DSRL32;
        SecondShift = Fgpu::DSLL32;
      }
      if ((ImmValue >= 33) && (ImmValue <= 63)) {
        FirstShift = Fgpu::DSRL32;
        SecondShift = Fgpu::DSLL;
      }
      break;
    }

    ATReg = getATReg(Inst.getLoc());
    if (!ATReg)
      return true;

    TOut.emitRRI(FirstShift, ATReg, SReg, ImmValue % 32, Inst.getLoc(), STI);
    TOut.emitRRI(SecondShift, DReg, SReg, (32 - ImmValue % 32) % 32,
                 Inst.getLoc(), STI);
    TOut.emitRRR(Fgpu::OR, DReg, DReg, ATReg, Inst.getLoc(), STI);

    return false;
  }

  return true;
}

bool FgpuAsmParser::expandAbs(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                              const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned FirstRegOp = Inst.getOperand(0).getReg();
  unsigned SecondRegOp = Inst.getOperand(1).getReg();

  TOut.emitRI(Fgpu::BGEZ, SecondRegOp, 8, IDLoc, STI);
  if (FirstRegOp != SecondRegOp)
    TOut.emitRRR(Fgpu::ADDu, FirstRegOp, SecondRegOp, Fgpu::ZERO, IDLoc, STI);
  else
    TOut.emitEmptyDelaySlot(false, IDLoc, STI);
  TOut.emitRRR(Fgpu::SUB, FirstRegOp, Fgpu::ZERO, SecondRegOp, IDLoc, STI);

  return false;
}

bool FgpuAsmParser::expandMulImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                 const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned ATReg = Fgpu::NoRegister;
  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  int32_t ImmValue = Inst.getOperand(2).getImm();

  ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  loadImmediate(ImmValue, ATReg, Fgpu::NoRegister, true, false, IDLoc, Out,
                STI);

  TOut.emitRR(Inst.getOpcode() == Fgpu::MULImmMacro ? Fgpu::MULT : Fgpu::DMULT,
              SrcReg, ATReg, IDLoc, STI);

  TOut.emitR(Fgpu::MFLO, DstReg, IDLoc, STI);

  return false;
}

bool FgpuAsmParser::expandMulO(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                               const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned ATReg = Fgpu::NoRegister;
  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  unsigned TmpReg = Inst.getOperand(2).getReg();

  ATReg = getATReg(Inst.getLoc());
  if (!ATReg)
    return true;

  TOut.emitRR(Inst.getOpcode() == Fgpu::MULOMacro ? Fgpu::MULT : Fgpu::DMULT,
              SrcReg, TmpReg, IDLoc, STI);

  TOut.emitR(Fgpu::MFLO, DstReg, IDLoc, STI);

  TOut.emitRRI(Inst.getOpcode() == Fgpu::MULOMacro ? Fgpu::SRA : Fgpu::DSRA32,
               DstReg, DstReg, 0x1F, IDLoc, STI);

  TOut.emitR(Fgpu::MFHI, ATReg, IDLoc, STI);

  if (useTraps()) {
    TOut.emitRRI(Fgpu::TNE, DstReg, ATReg, 6, IDLoc, STI);
  } else {
    MCContext & Context = TOut.getStreamer().getContext();
    MCSymbol * BrTarget = Context.createTempSymbol();
    MCOperand LabelOp =
        MCOperand::createExpr(MCSymbolRefExpr::create(BrTarget, Context));

    TOut.emitRRX(Fgpu::BEQ, DstReg, ATReg, LabelOp, IDLoc, STI);
    if (AssemblerOptions.back()->isReorder())
      TOut.emitNop(IDLoc, STI);
    TOut.emitII(Fgpu::BREAK, 6, 0, IDLoc, STI);

    TOut.getStreamer().emitLabel(BrTarget);
  }
  TOut.emitR(Fgpu::MFLO, DstReg, IDLoc, STI);

  return false;
}

bool FgpuAsmParser::expandMulOU(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned ATReg = Fgpu::NoRegister;
  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  unsigned TmpReg = Inst.getOperand(2).getReg();

  ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  TOut.emitRR(Inst.getOpcode() == Fgpu::MULOUMacro ? Fgpu::MULTu : Fgpu::DMULTu,
              SrcReg, TmpReg, IDLoc, STI);

  TOut.emitR(Fgpu::MFHI, ATReg, IDLoc, STI);
  TOut.emitR(Fgpu::MFLO, DstReg, IDLoc, STI);
  if (useTraps()) {
    TOut.emitRRI(Fgpu::TNE, ATReg, Fgpu::ZERO, 6, IDLoc, STI);
  } else {
    MCContext & Context = TOut.getStreamer().getContext();
    MCSymbol * BrTarget = Context.createTempSymbol();
    MCOperand LabelOp =
        MCOperand::createExpr(MCSymbolRefExpr::create(BrTarget, Context));

    TOut.emitRRX(Fgpu::BEQ, ATReg, Fgpu::ZERO, LabelOp, IDLoc, STI);
    if (AssemblerOptions.back()->isReorder())
      TOut.emitNop(IDLoc, STI);
    TOut.emitII(Fgpu::BREAK, 6, 0, IDLoc, STI);

    TOut.getStreamer().emitLabel(BrTarget);
  }

  return false;
}

bool FgpuAsmParser::expandDMULMacro(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                    const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  unsigned TmpReg = Inst.getOperand(2).getReg();

  TOut.emitRR(Fgpu::DMULTu, SrcReg, TmpReg, IDLoc, STI);
  TOut.emitR(Fgpu::MFLO, DstReg, IDLoc, STI);

  return false;
}

// Expand 'ld $<reg> offset($reg2)' to 'lw $<reg>, offset($reg2);
//                                      lw $<reg+1>>, offset+4($reg2)'
// or expand 'sd $<reg> offset($reg2)' to 'sw $<reg>, offset($reg2);
//                                         sw $<reg+1>>, offset+4($reg2)'
// for O32.
bool FgpuAsmParser::expandLoadStoreDMacro(MCInst &Inst, SMLoc IDLoc,
                                          MCStreamer &Out,
                                          const MCSubtargetInfo *STI,
                                          bool IsLoad) {
  if (!isABI_O32())
    return true;

  warnIfNoMacro(IDLoc);

  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned Opcode = IsLoad ? Fgpu::LW : Fgpu::SW;
  unsigned FirstReg = Inst.getOperand(0).getReg();
  unsigned SecondReg = nextReg(FirstReg);
  unsigned BaseReg = Inst.getOperand(1).getReg();
  if (!SecondReg)
    return true;

  warnIfRegIndexIsAT(FirstReg, IDLoc);

  assert(Inst.getOperand(2).isImm() &&
         "Offset for load macro is not immediate!");

  MCOperand &FirstOffset = Inst.getOperand(2);
  signed NextOffset = FirstOffset.getImm() + 4;
  MCOperand SecondOffset = MCOperand::createImm(NextOffset);

  if (!isInt<16>(FirstOffset.getImm()) || !isInt<16>(NextOffset))
    return true;

  // For loads, clobber the base register with the second load instead of the
  // first if the BaseReg == FirstReg.
  if (FirstReg != BaseReg || !IsLoad) {
    TOut.emitRRX(Opcode, FirstReg, BaseReg, FirstOffset, IDLoc, STI);
    TOut.emitRRX(Opcode, SecondReg, BaseReg, SecondOffset, IDLoc, STI);
  } else {
    TOut.emitRRX(Opcode, SecondReg, BaseReg, SecondOffset, IDLoc, STI);
    TOut.emitRRX(Opcode, FirstReg, BaseReg, FirstOffset, IDLoc, STI);
  }

  return false;
}


// Expand 's.d $<reg> offset($reg2)' to 'swc1 $<reg+1>, offset($reg2);
//                                       swc1 $<reg>, offset+4($reg2)'
// or if little endian to 'swc1 $<reg>, offset($reg2);
//                         swc1 $<reg+1>, offset+4($reg2)'
// for Fgpu1.
bool FgpuAsmParser::expandStoreDM1Macro(MCInst &Inst, SMLoc IDLoc,
                                        MCStreamer &Out,
                                        const MCSubtargetInfo *STI) {
  if (!isABI_O32())
    return true;

  warnIfNoMacro(IDLoc);

  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned Opcode = Fgpu::SWC1;
  unsigned FirstReg = Inst.getOperand(0).getReg();
  unsigned SecondReg = nextReg(FirstReg);
  unsigned BaseReg = Inst.getOperand(1).getReg();
  if (!SecondReg)
    return true;

  warnIfRegIndexIsAT(FirstReg, IDLoc);

  assert(Inst.getOperand(2).isImm() &&
         "Offset for macro is not immediate!");

  MCOperand &FirstOffset = Inst.getOperand(2);
  signed NextOffset = FirstOffset.getImm() + 4;
  MCOperand SecondOffset = MCOperand::createImm(NextOffset);

  if (!isInt<16>(FirstOffset.getImm()) || !isInt<16>(NextOffset))
    return true;

  if (!IsLittleEndian)
    std::swap(FirstReg, SecondReg);

  TOut.emitRRX(Opcode, FirstReg, BaseReg, FirstOffset, IDLoc, STI);
  TOut.emitRRX(Opcode, SecondReg, BaseReg, SecondOffset, IDLoc, STI);

  return false;
}

bool FgpuAsmParser::expandSeq(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                              const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();

  assert(Inst.getNumOperands() == 3 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() &&
         Inst.getOperand(1).isReg() &&
         Inst.getOperand(2).isReg() && "Invalid instruction operand.");

  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  unsigned OpReg = Inst.getOperand(2).getReg();

  warnIfNoMacro(IDLoc);

  if (SrcReg != Fgpu::ZERO && OpReg != Fgpu::ZERO) {
    TOut.emitRRR(Fgpu::XOR, DstReg, SrcReg, OpReg, IDLoc, STI);
    TOut.emitRRI(Fgpu::SLTiu, DstReg, DstReg, 1, IDLoc, STI);
    return false;
  }

  unsigned Reg = SrcReg == Fgpu::ZERO ? OpReg : SrcReg;
  TOut.emitRRI(Fgpu::SLTiu, DstReg, Reg, 1, IDLoc, STI);
  return false;
}

bool FgpuAsmParser::expandSeqI(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                               const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();

  assert(Inst.getNumOperands() == 3 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() &&
         Inst.getOperand(1).isReg() &&
         Inst.getOperand(2).isImm() && "Invalid instruction operand.");

  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  int64_t Imm = Inst.getOperand(2).getImm();

  warnIfNoMacro(IDLoc);

  if (Imm == 0) {
    TOut.emitRRI(Fgpu::SLTiu, DstReg, SrcReg, 1, IDLoc, STI);
    return false;
  }

  if (SrcReg == Fgpu::ZERO) {
    Warning(IDLoc, "comparison is always false");
    TOut.emitRRR(isGP64bit() ? Fgpu::DADDu : Fgpu::ADDu,
                 DstReg, SrcReg, SrcReg, IDLoc, STI);
    return false;
  }

  unsigned Opc;
  if (Imm > -0x8000 && Imm < 0) {
    Imm = -Imm;
    Opc = isGP64bit() ? Fgpu::DADDiu : Fgpu::ADDiu;
  } else {
    Opc = Fgpu::XORi;
  }

  if (!isUInt<16>(Imm)) {
    unsigned ATReg = getATReg(IDLoc);
    if (!ATReg)
      return true;

    if (loadImmediate(Imm, ATReg, Fgpu::NoRegister, true, isGP64bit(), IDLoc,
                      Out, STI))
      return true;

    TOut.emitRRR(Fgpu::XOR, DstReg, SrcReg, ATReg, IDLoc, STI);
    TOut.emitRRI(Fgpu::SLTiu, DstReg, DstReg, 1, IDLoc, STI);
    return false;
  }

  TOut.emitRRI(Opc, DstReg, SrcReg, Imm, IDLoc, STI);
  TOut.emitRRI(Fgpu::SLTiu, DstReg, DstReg, 1, IDLoc, STI);
  return false;
}

bool FgpuAsmParser::expandSne(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                              const MCSubtargetInfo *STI) {

  FgpuTargetStreamer &TOut = getTargetStreamer();

  assert(Inst.getNumOperands() == 3 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() &&
         Inst.getOperand(1).isReg() &&
         Inst.getOperand(2).isReg() && "Invalid instruction operand.");

  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  unsigned OpReg = Inst.getOperand(2).getReg();

  warnIfNoMacro(IDLoc);

  if (SrcReg != Fgpu::ZERO && OpReg != Fgpu::ZERO) {
    TOut.emitRRR(Fgpu::XOR, DstReg, SrcReg, OpReg, IDLoc, STI);
    TOut.emitRRR(Fgpu::SLTu, DstReg, Fgpu::ZERO, DstReg, IDLoc, STI);
    return false;
  }

  unsigned Reg = SrcReg == Fgpu::ZERO ? OpReg : SrcReg;
  TOut.emitRRR(Fgpu::SLTu, DstReg, Fgpu::ZERO, Reg, IDLoc, STI);
  return false;
}

bool FgpuAsmParser::expandSneI(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                               const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();

  assert(Inst.getNumOperands() == 3 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() &&
         Inst.getOperand(1).isReg() &&
         Inst.getOperand(2).isImm() && "Invalid instruction operand.");

  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  int64_t ImmValue = Inst.getOperand(2).getImm();

  warnIfNoMacro(IDLoc);

  if (ImmValue == 0) {
    TOut.emitRRR(Fgpu::SLTu, DstReg, Fgpu::ZERO, SrcReg, IDLoc, STI);
    return false;
  }

  if (SrcReg == Fgpu::ZERO) {
    Warning(IDLoc, "comparison is always true");
    if (loadImmediate(1, DstReg, Fgpu::NoRegister, true, false, IDLoc, Out,
                      STI))
      return true;
    return false;
  }

  unsigned Opc;
  if (ImmValue > -0x8000 && ImmValue < 0) {
    ImmValue = -ImmValue;
    Opc = isGP64bit() ? Fgpu::DADDiu : Fgpu::ADDiu;
  } else {
    Opc = Fgpu::XORi;
  }

  if (isUInt<16>(ImmValue)) {
    TOut.emitRRI(Opc, DstReg, SrcReg, ImmValue, IDLoc, STI);
    TOut.emitRRR(Fgpu::SLTu, DstReg, Fgpu::ZERO, DstReg, IDLoc, STI);
    return false;
  }

  unsigned ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  if (loadImmediate(ImmValue, ATReg, Fgpu::NoRegister, isInt<32>(ImmValue),
                    false, IDLoc, Out, STI))
    return true;

  TOut.emitRRR(Fgpu::XOR, DstReg, SrcReg, ATReg, IDLoc, STI);
  TOut.emitRRR(Fgpu::SLTu, DstReg, Fgpu::ZERO, DstReg, IDLoc, STI);
  return false;
}

// Map the DSP accumulator and control register to the corresponding gpr
// operand. Unlike the other alias, the m(f|t)t(lo|hi|acx) instructions
// do not map the DSP registers contigously to gpr registers.
static unsigned getRegisterForMxtrDSP(MCInst &Inst, bool IsMFDSP) {
  switch (Inst.getOpcode()) {
    case Fgpu::MFTLO:
    case Fgpu::MTTLO:
      switch (Inst.getOperand(IsMFDSP ? 1 : 0).getReg()) {
        case Fgpu::AC0:
          return Fgpu::ZERO;
        case Fgpu::AC1:
          return Fgpu::A0;
        case Fgpu::AC2:
          return Fgpu::T0;
        case Fgpu::AC3:
          return Fgpu::T4;
        default:
          llvm_unreachable("Unknown register for 'mttr' alias!");
    }
    case Fgpu::MFTHI:
    case Fgpu::MTTHI:
      switch (Inst.getOperand(IsMFDSP ? 1 : 0).getReg()) {
        case Fgpu::AC0:
          return Fgpu::AT;
        case Fgpu::AC1:
          return Fgpu::A1;
        case Fgpu::AC2:
          return Fgpu::T1;
        case Fgpu::AC3:
          return Fgpu::T5;
        default:
          llvm_unreachable("Unknown register for 'mttr' alias!");
    }
    case Fgpu::MFTACX:
    case Fgpu::MTTACX:
      switch (Inst.getOperand(IsMFDSP ? 1 : 0).getReg()) {
        case Fgpu::AC0:
          return Fgpu::V0;
        case Fgpu::AC1:
          return Fgpu::A2;
        case Fgpu::AC2:
          return Fgpu::T2;
        case Fgpu::AC3:
          return Fgpu::T6;
        default:
          llvm_unreachable("Unknown register for 'mttr' alias!");
    }
    case Fgpu::MFTDSP:
    case Fgpu::MTTDSP:
      return Fgpu::S0;
    default:
      llvm_unreachable("Unknown instruction for 'mttr' dsp alias!");
  }
}

// Map the floating point register operand to the corresponding register
// operand.
static unsigned getRegisterForMxtrFP(MCInst &Inst, bool IsMFTC1) {
  switch (Inst.getOperand(IsMFTC1 ? 1 : 0).getReg()) {
    case Fgpu::F0:  return Fgpu::ZERO;
    case Fgpu::F1:  return Fgpu::AT;
    case Fgpu::F2:  return Fgpu::V0;
    case Fgpu::F3:  return Fgpu::V1;
    case Fgpu::F4:  return Fgpu::A0;
    case Fgpu::F5:  return Fgpu::A1;
    case Fgpu::F6:  return Fgpu::A2;
    case Fgpu::F7:  return Fgpu::A3;
    case Fgpu::F8:  return Fgpu::T0;
    case Fgpu::F9:  return Fgpu::T1;
    case Fgpu::F10: return Fgpu::T2;
    case Fgpu::F11: return Fgpu::T3;
    case Fgpu::F12: return Fgpu::T4;
    case Fgpu::F13: return Fgpu::T5;
    case Fgpu::F14: return Fgpu::T6;
    case Fgpu::F15: return Fgpu::T7;
    case Fgpu::F16: return Fgpu::S0;
    case Fgpu::F17: return Fgpu::S1;
    case Fgpu::F18: return Fgpu::S2;
    case Fgpu::F19: return Fgpu::S3;
    case Fgpu::F20: return Fgpu::S4;
    case Fgpu::F21: return Fgpu::S5;
    case Fgpu::F22: return Fgpu::S6;
    case Fgpu::F23: return Fgpu::S7;
    case Fgpu::F24: return Fgpu::T8;
    case Fgpu::F25: return Fgpu::T9;
    case Fgpu::F26: return Fgpu::K0;
    case Fgpu::F27: return Fgpu::K1;
    case Fgpu::F28: return Fgpu::GP;
    case Fgpu::F29: return Fgpu::SP;
    case Fgpu::F30: return Fgpu::FP;
    case Fgpu::F31: return Fgpu::RA;
    default: llvm_unreachable("Unknown register for mttc1 alias!");
  }
}

// Map the coprocessor operand the corresponding gpr register operand.
static unsigned getRegisterForMxtrC0(MCInst &Inst, bool IsMFTC0) {
  switch (Inst.getOperand(IsMFTC0 ? 1 : 0).getReg()) {
    case Fgpu::COP00:  return Fgpu::ZERO;
    case Fgpu::COP01:  return Fgpu::AT;
    case Fgpu::COP02:  return Fgpu::V0;
    case Fgpu::COP03:  return Fgpu::V1;
    case Fgpu::COP04:  return Fgpu::A0;
    case Fgpu::COP05:  return Fgpu::A1;
    case Fgpu::COP06:  return Fgpu::A2;
    case Fgpu::COP07:  return Fgpu::A3;
    case Fgpu::COP08:  return Fgpu::T0;
    case Fgpu::COP09:  return Fgpu::T1;
    case Fgpu::COP010: return Fgpu::T2;
    case Fgpu::COP011: return Fgpu::T3;
    case Fgpu::COP012: return Fgpu::T4;
    case Fgpu::COP013: return Fgpu::T5;
    case Fgpu::COP014: return Fgpu::T6;
    case Fgpu::COP015: return Fgpu::T7;
    case Fgpu::COP016: return Fgpu::S0;
    case Fgpu::COP017: return Fgpu::S1;
    case Fgpu::COP018: return Fgpu::S2;
    case Fgpu::COP019: return Fgpu::S3;
    case Fgpu::COP020: return Fgpu::S4;
    case Fgpu::COP021: return Fgpu::S5;
    case Fgpu::COP022: return Fgpu::S6;
    case Fgpu::COP023: return Fgpu::S7;
    case Fgpu::COP024: return Fgpu::T8;
    case Fgpu::COP025: return Fgpu::T9;
    case Fgpu::COP026: return Fgpu::K0;
    case Fgpu::COP027: return Fgpu::K1;
    case Fgpu::COP028: return Fgpu::GP;
    case Fgpu::COP029: return Fgpu::SP;
    case Fgpu::COP030: return Fgpu::FP;
    case Fgpu::COP031: return Fgpu::RA;
    default: llvm_unreachable("Unknown register for mttc0 alias!");
  }
}

/// Expand an alias of 'mftr' or 'mttr' into the full instruction, by producing
/// an mftr or mttr with the correctly mapped gpr register, u, sel and h bits.
bool FgpuAsmParser::expandMXTRAlias(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                    const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned rd = 0;
  unsigned u = 1;
  unsigned sel = 0;
  unsigned h = 0;
  bool IsMFTR = false;
  switch (Inst.getOpcode()) {
    case Fgpu::MFTC0:
      IsMFTR = true;
      LLVM_FALLTHROUGH;
    case Fgpu::MTTC0:
      u = 0;
      rd = getRegisterForMxtrC0(Inst, IsMFTR);
      sel = Inst.getOperand(2).getImm();
      break;
    case Fgpu::MFTGPR:
      IsMFTR = true;
      LLVM_FALLTHROUGH;
    case Fgpu::MTTGPR:
      rd = Inst.getOperand(IsMFTR ? 1 : 0).getReg();
      break;
    case Fgpu::MFTLO:
    case Fgpu::MFTHI:
    case Fgpu::MFTACX:
    case Fgpu::MFTDSP:
      IsMFTR = true;
      LLVM_FALLTHROUGH;
    case Fgpu::MTTLO:
    case Fgpu::MTTHI:
    case Fgpu::MTTACX:
    case Fgpu::MTTDSP:
      rd = getRegisterForMxtrDSP(Inst, IsMFTR);
      sel = 1;
      break;
    case Fgpu::MFTHC1:
      h = 1;
      LLVM_FALLTHROUGH;
    case Fgpu::MFTC1:
      IsMFTR = true;
      rd = getRegisterForMxtrFP(Inst, IsMFTR);
      sel = 2;
      break;
    case Fgpu::MTTHC1:
      h = 1;
      LLVM_FALLTHROUGH;
    case Fgpu::MTTC1:
      rd = getRegisterForMxtrFP(Inst, IsMFTR);
      sel = 2;
      break;
    case Fgpu::CFTC1:
      IsMFTR = true;
      LLVM_FALLTHROUGH;
    case Fgpu::CTTC1:
      rd = getRegisterForMxtrFP(Inst, IsMFTR);
      sel = 3;
      break;
  }
  unsigned Op0 = IsMFTR ? Inst.getOperand(0).getReg() : rd;
  unsigned Op1 =
      IsMFTR ? rd
             : (Inst.getOpcode() != Fgpu::MTTDSP ? Inst.getOperand(1).getReg()
                                                 : Inst.getOperand(0).getReg());

  TOut.emitRRIII(IsMFTR ? Fgpu::MFTR : Fgpu::MTTR, Op0, Op1, u, sel, h, IDLoc,
                 STI);
  return false;
}

bool FgpuAsmParser::expandSaaAddr(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                  const MCSubtargetInfo *STI) {
  assert(Inst.getNumOperands() == 3 && "expected three operands");
  assert(Inst.getOperand(0).isReg() && "expected register operand kind");
  assert(Inst.getOperand(1).isReg() && "expected register operand kind");

  warnIfNoMacro(IDLoc);

  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned Opcode = Inst.getOpcode() == Fgpu::SaaAddr ? Fgpu::SAA : Fgpu::SAAD;
  unsigned RtReg = Inst.getOperand(0).getReg();
  unsigned BaseReg = Inst.getOperand(1).getReg();
  const MCOperand &BaseOp = Inst.getOperand(2);

  if (BaseOp.isImm()) {
    int64_t ImmValue = BaseOp.getImm();
    if (ImmValue == 0) {
      TOut.emitRR(Opcode, RtReg, BaseReg, IDLoc, STI);
      return false;
    }
  }

  unsigned ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  if (expandLoadAddress(ATReg, BaseReg, BaseOp, !isGP64bit(), IDLoc, Out, STI))
    return true;

  TOut.emitRR(Opcode, RtReg, ATReg, IDLoc, STI);
  return false;
}

unsigned
FgpuAsmParser::checkEarlyTargetMatchPredicate(MCInst &Inst,
                                              const OperandVector &Operands) {
  switch (Inst.getOpcode()) {
  default:
    return Match_Success;
  case Fgpu::DATI:
  case Fgpu::DAHI:
    if (static_cast<FgpuOperand &>(*Operands[1])
            .isValidForTie(static_cast<FgpuOperand &>(*Operands[2])))
      return Match_Success;
    return Match_RequiresSameSrcAndDst;
  }
}

unsigned FgpuAsmParser::checkTargetMatchPredicate(MCInst &Inst) {
  switch (Inst.getOpcode()) {
  // As described by the FGPUR6 spec, daui must not use the zero operand for
  // its source operand.
  case Fgpu::DAUI:
    if (Inst.getOperand(1).getReg() == Fgpu::ZERO ||
        Inst.getOperand(1).getReg() == Fgpu::ZERO_64)
      return Match_RequiresNoZeroRegister;
    return Match_Success;
  // As described by the Fgpu32r2 spec, the registers Rd and Rs for
  // jalr.hb must be different.
  // It also applies for registers Rt and Rs of microFGPUr6 jalrc.hb instruction
  // and registers Rd and Base for microFGPU lwp instruction
  case Fgpu::JALR_HB:
  case Fgpu::JALR_HB64:
  case Fgpu::JALRC_HB_MMR6:
  case Fgpu::JALRC_MMR6:
    if (Inst.getOperand(0).getReg() == Inst.getOperand(1).getReg())
      return Match_RequiresDifferentSrcAndDst;
    return Match_Success;
  case Fgpu::LWP_MM:
    if (Inst.getOperand(0).getReg() == Inst.getOperand(2).getReg())
      return Match_RequiresDifferentSrcAndDst;
    return Match_Success;
  case Fgpu::SYNC:
    if (Inst.getOperand(0).getImm() != 0 && !hasFgpu32())
      return Match_NonZeroOperandForSync;
    return Match_Success;
  case Fgpu::MFC0:
  case Fgpu::MTC0:
  case Fgpu::MTC2:
  case Fgpu::MFC2:
    if (Inst.getOperand(2).getImm() != 0 && !hasFgpu32())
      return Match_NonZeroOperandForMTCX;
    return Match_Success;
  // As described the FGPUR6 spec, the compact branches that compare registers
  // must:
  // a) Not use the zero register.
  // b) Not use the same register twice.
  // c) rs < rt for bnec, beqc.
  //    NB: For this case, the encoding will swap the operands as their
  //    ordering doesn't matter. GAS performs this transformation  too.
  //    Hence, that constraint does not have to be enforced.
  //
  // The compact branches that branch iff the signed addition of two registers
  // would overflow must have rs >= rt. That can be handled like beqc/bnec with
  // operand swapping. They do not have restriction of using the zero register.
  case Fgpu::BLEZC:   case Fgpu::BLEZC_MMR6:
  case Fgpu::BGEZC:   case Fgpu::BGEZC_MMR6:
  case Fgpu::BGTZC:   case Fgpu::BGTZC_MMR6:
  case Fgpu::BLTZC:   case Fgpu::BLTZC_MMR6:
  case Fgpu::BEQZC:   case Fgpu::BEQZC_MMR6:
  case Fgpu::BNEZC:   case Fgpu::BNEZC_MMR6:
  case Fgpu::BLEZC64:
  case Fgpu::BGEZC64:
  case Fgpu::BGTZC64:
  case Fgpu::BLTZC64:
  case Fgpu::BEQZC64:
  case Fgpu::BNEZC64:
    if (Inst.getOperand(0).getReg() == Fgpu::ZERO ||
        Inst.getOperand(0).getReg() == Fgpu::ZERO_64)
      return Match_RequiresNoZeroRegister;
    return Match_Success;
  case Fgpu::BGEC:    case Fgpu::BGEC_MMR6:
  case Fgpu::BLTC:    case Fgpu::BLTC_MMR6:
  case Fgpu::BGEUC:   case Fgpu::BGEUC_MMR6:
  case Fgpu::BLTUC:   case Fgpu::BLTUC_MMR6:
  case Fgpu::BEQC:    case Fgpu::BEQC_MMR6:
  case Fgpu::BNEC:    case Fgpu::BNEC_MMR6:
  case Fgpu::BGEC64:
  case Fgpu::BLTC64:
  case Fgpu::BGEUC64:
  case Fgpu::BLTUC64:
  case Fgpu::BEQC64:
  case Fgpu::BNEC64:
    if (Inst.getOperand(0).getReg() == Fgpu::ZERO ||
        Inst.getOperand(0).getReg() == Fgpu::ZERO_64)
      return Match_RequiresNoZeroRegister;
    if (Inst.getOperand(1).getReg() == Fgpu::ZERO ||
        Inst.getOperand(1).getReg() == Fgpu::ZERO_64)
      return Match_RequiresNoZeroRegister;
    if (Inst.getOperand(0).getReg() == Inst.getOperand(1).getReg())
      return Match_RequiresDifferentOperands;
    return Match_Success;
  case Fgpu::DINS: {
    assert(Inst.getOperand(2).isImm() && Inst.getOperand(3).isImm() &&
           "Operands must be immediates for dins!");
    const signed Pos = Inst.getOperand(2).getImm();
    const signed Size = Inst.getOperand(3).getImm();
    if ((0 > (Pos + Size)) || ((Pos + Size) > 32))
      return Match_RequiresPosSizeRange0_32;
    return Match_Success;
  }
  case Fgpu::DINSM:
  case Fgpu::DINSU: {
    assert(Inst.getOperand(2).isImm() && Inst.getOperand(3).isImm() &&
           "Operands must be immediates for dinsm/dinsu!");
    const signed Pos = Inst.getOperand(2).getImm();
    const signed Size = Inst.getOperand(3).getImm();
    if ((32 >= (Pos + Size)) || ((Pos + Size) > 64))
      return Match_RequiresPosSizeRange33_64;
    return Match_Success;
  }
  case Fgpu::DEXT: {
    assert(Inst.getOperand(2).isImm() && Inst.getOperand(3).isImm() &&
           "Operands must be immediates for DEXTM!");
    const signed Pos = Inst.getOperand(2).getImm();
    const signed Size = Inst.getOperand(3).getImm();
    if ((1 > (Pos + Size)) || ((Pos + Size) > 63))
      return Match_RequiresPosSizeUImm6;
    return Match_Success;
  }
  case Fgpu::DEXTM:
  case Fgpu::DEXTU: {
    assert(Inst.getOperand(2).isImm() && Inst.getOperand(3).isImm() &&
           "Operands must be immediates for dextm/dextu!");
    const signed Pos = Inst.getOperand(2).getImm();
    const signed Size = Inst.getOperand(3).getImm();
    if ((32 > (Pos + Size)) || ((Pos + Size) > 64))
      return Match_RequiresPosSizeRange33_64;
    return Match_Success;
  }
  case Fgpu::CRC32B: case Fgpu::CRC32CB:
  case Fgpu::CRC32H: case Fgpu::CRC32CH:
  case Fgpu::CRC32W: case Fgpu::CRC32CW:
  case Fgpu::CRC32D: case Fgpu::CRC32CD:
    if (Inst.getOperand(0).getReg() != Inst.getOperand(2).getReg())
      return Match_RequiresSameSrcAndDst;
    return Match_Success;
  }

  uint64_t TSFlags = getInstDesc(Inst.getOpcode()).TSFlags;
  if ((TSFlags & FgpuII::HasFCCRegOperand) &&
      (Inst.getOperand(0).getReg() != Fgpu::FCC0) && !hasEightFccRegisters())
    return Match_NoFCCRegisterForCurrentISA;

  return Match_Success;

}

static SMLoc RefineErrorLoc(const SMLoc Loc, const OperandVector &Operands,
                            uint64_t ErrorInfo) {
  if (ErrorInfo != ~0ULL && ErrorInfo < Operands.size()) {
    SMLoc ErrorLoc = Operands[ErrorInfo]->getStartLoc();
    if (ErrorLoc == SMLoc())
      return Loc;
    return ErrorLoc;
  }
  return Loc;
}

bool FgpuAsmParser::MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                                            OperandVector &Operands,
                                            MCStreamer &Out,
                                            uint64_t &ErrorInfo,
                                            bool MatchingInlineAsm) {
  MCInst Inst;
  unsigned MatchResult =
      MatchInstructionImpl(Operands, Inst, ErrorInfo, MatchingInlineAsm);

  switch (MatchResult) {
  case Match_Success:
    if (processInstruction(Inst, IDLoc, Out, STI))
      return true;
    return false;
  case Match_MissingFeature:
    Error(IDLoc, "instruction requires a CPU feature not currently enabled");
    return true;
  case Match_InvalidOperand: {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0ULL) {
      if (ErrorInfo >= Operands.size())
        return Error(IDLoc, "too few operands for instruction");

      ErrorLoc = Operands[ErrorInfo]->getStartLoc();
      if (ErrorLoc == SMLoc())
        ErrorLoc = IDLoc;
    }

    return Error(ErrorLoc, "invalid operand for instruction");
  }
  case Match_NonZeroOperandForSync:
    return Error(IDLoc,
                 "s-type must be zero or unspecified for pre-FGPU32 ISAs");
  case Match_NonZeroOperandForMTCX:
    return Error(IDLoc, "selector must be zero for pre-FGPU32 ISAs");
  case Match_MnemonicFail:
    return Error(IDLoc, "invalid instruction");
  case Match_RequiresDifferentSrcAndDst:
    return Error(IDLoc, "source and destination must be different");
  case Match_RequiresDifferentOperands:
    return Error(IDLoc, "registers must be different");
  case Match_RequiresNoZeroRegister:
    return Error(IDLoc, "invalid operand ($zero) for instruction");
  case Match_RequiresSameSrcAndDst:
    return Error(IDLoc, "source and destination must match");
  case Match_NoFCCRegisterForCurrentISA:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "non-zero fcc register doesn't exist in current ISA level");
  case Match_Immz:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo), "expected '0'");
  case Match_UImm1_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 1-bit unsigned immediate");
  case Match_UImm2_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 2-bit unsigned immediate");
  case Match_UImm2_1:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected immediate in range 1 .. 4");
  case Match_UImm3_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 3-bit unsigned immediate");
  case Match_UImm4_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 4-bit unsigned immediate");
  case Match_SImm4_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 4-bit signed immediate");
  case Match_UImm5_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 5-bit unsigned immediate");
  case Match_SImm5_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 5-bit signed immediate");
  case Match_UImm5_1:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected immediate in range 1 .. 32");
  case Match_UImm5_32:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected immediate in range 32 .. 63");
  case Match_UImm5_33:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected immediate in range 33 .. 64");
  case Match_UImm5_0_Report_UImm6:
    // This is used on UImm5 operands that have a corresponding UImm5_32
    // operand to avoid confusing the user.
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 6-bit unsigned immediate");
  case Match_UImm5_Lsl2:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected both 7-bit unsigned immediate and multiple of 4");
  case Match_UImmRange2_64:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected immediate in range 2 .. 64");
  case Match_UImm6_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 6-bit unsigned immediate");
  case Match_UImm6_Lsl2:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected both 8-bit unsigned immediate and multiple of 4");
  case Match_SImm6_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 6-bit signed immediate");
  case Match_UImm7_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 7-bit unsigned immediate");
  case Match_UImm7_N1:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected immediate in range -1 .. 126");
  case Match_SImm7_Lsl2:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected both 9-bit signed immediate and multiple of 4");
  case Match_UImm8_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 8-bit unsigned immediate");
  case Match_UImm10_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 10-bit unsigned immediate");
  case Match_SImm10_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 10-bit signed immediate");
  case Match_SImm11_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 11-bit signed immediate");
  case Match_UImm16:
  case Match_UImm16_Relaxed:
  case Match_UImm16_AltRelaxed:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 16-bit unsigned immediate");
  case Match_SImm16:
  case Match_SImm16_Relaxed:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 16-bit signed immediate");
  case Match_SImm19_Lsl2:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected both 19-bit signed immediate and multiple of 4");
  case Match_UImm20_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 20-bit unsigned immediate");
  case Match_UImm26_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 26-bit unsigned immediate");
  case Match_SImm32:
  case Match_SImm32_Relaxed:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 32-bit signed immediate");
  case Match_UImm32_Coerced:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 32-bit immediate");
  case Match_MemSImm9:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 9-bit signed offset");
  case Match_MemSImm10:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 10-bit signed offset");
  case Match_MemSImm10Lsl1:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 11-bit signed offset and multiple of 2");
  case Match_MemSImm10Lsl2:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 12-bit signed offset and multiple of 4");
  case Match_MemSImm10Lsl3:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 13-bit signed offset and multiple of 8");
  case Match_MemSImm11:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 11-bit signed offset");
  case Match_MemSImm12:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 12-bit signed offset");
  case Match_MemSImm16:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 16-bit signed offset");
  case Match_MemSImmPtr:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 32-bit signed offset");
  case Match_RequiresPosSizeRange0_32: {
    SMLoc ErrorStart = Operands[3]->getStartLoc();
    SMLoc ErrorEnd = Operands[4]->getEndLoc();
    return Error(ErrorStart, "size plus position are not in the range 0 .. 32",
                 SMRange(ErrorStart, ErrorEnd));
    }
  case Match_RequiresPosSizeUImm6: {
    SMLoc ErrorStart = Operands[3]->getStartLoc();
    SMLoc ErrorEnd = Operands[4]->getEndLoc();
    return Error(ErrorStart, "size plus position are not in the range 1 .. 63",
                 SMRange(ErrorStart, ErrorEnd));
    }
  case Match_RequiresPosSizeRange33_64: {
    SMLoc ErrorStart = Operands[3]->getStartLoc();
    SMLoc ErrorEnd = Operands[4]->getEndLoc();
    return Error(ErrorStart, "size plus position are not in the range 33 .. 64",
                 SMRange(ErrorStart, ErrorEnd));
    }
  }

  llvm_unreachable("Implement any new match types added!");
}

void FgpuAsmParser::warnIfRegIndexIsAT(unsigned RegIndex, SMLoc Loc) {
  if (RegIndex != 0 && AssemblerOptions.back()->getATRegIndex() == RegIndex)
    Warning(Loc, "used $at (currently $" + Twine(RegIndex) +
                     ") without \".set noat\"");
}

void FgpuAsmParser::warnIfNoMacro(SMLoc Loc) {
  if (!AssemblerOptions.back()->isMacro())
    Warning(Loc, "macro instruction expanded into multiple instructions");
}

void FgpuAsmParser::ConvertXWPOperands(MCInst &Inst,
                                       const OperandVector &Operands) {
  assert(
      (Inst.getOpcode() == Fgpu::LWP_MM || Inst.getOpcode() == Fgpu::SWP_MM) &&
      "Unexpected instruction!");
  ((FgpuOperand &)*Operands[1]).addGPR32ZeroAsmRegOperands(Inst, 1);
  int NextReg = nextReg(((FgpuOperand &)*Operands[1]).getGPR32Reg());
  Inst.addOperand(MCOperand::createReg(NextReg));
  ((FgpuOperand &)*Operands[2]).addMemOperands(Inst, 2);
}

void
FgpuAsmParser::printWarningWithFixIt(const Twine &Msg, const Twine &FixMsg,
                                     SMRange Range, bool ShowColors) {
  getSourceManager().PrintMessage(Range.Start, SourceMgr::DK_Warning, Msg,
                                  Range, SMFixIt(Range, FixMsg),
                                  ShowColors);
}

int FgpuAsmParser::matchCPURegisterName(StringRef Name) {
  int CC;

  CC = StringSwitch<unsigned>(Name)
           .Case("zero", 0)
           .Cases("at", "AT", 1)
           .Case("a0", 4)
           .Case("a1", 5)
           .Case("a2", 6)
           .Case("a3", 7)
           .Case("v0", 2)
           .Case("v1", 3)
           .Case("s0", 16)
           .Case("s1", 17)
           .Case("s2", 18)
           .Case("s3", 19)
           .Case("s4", 20)
           .Case("s5", 21)
           .Case("s6", 22)
           .Case("s7", 23)
           .Case("k0", 26)
           .Case("k1", 27)
           .Case("gp", 28)
           .Case("sp", 29)
           .Case("fp", 30)
           .Case("s8", 30)
           .Case("ra", 31)
           .Case("t0", 8)
           .Case("t1", 9)
           .Case("t2", 10)
           .Case("t3", 11)
           .Case("t4", 12)
           .Case("t5", 13)
           .Case("t6", 14)
           .Case("t7", 15)
           .Case("t8", 24)
           .Case("t9", 25)
           .Default(-1);

  if (!(isABI_N32() || isABI_N64()))
    return CC;

  if (12 <= CC && CC <= 15) {
    // Name is one of t4-t7
    AsmToken RegTok = getLexer().peekTok();
    SMRange RegRange = RegTok.getLocRange();

    StringRef FixedName = StringSwitch<StringRef>(Name)
                              .Case("t4", "t0")
                              .Case("t5", "t1")
                              .Case("t6", "t2")
                              .Case("t7", "t3")
                              .Default("");
    assert(FixedName != "" &&  "Register name is not one of t4-t7.");

    printWarningWithFixIt("register names $t4-$t7 are only available in O32.",
                          "Did you mean $" + FixedName + "?", RegRange);
  }

  // Although SGI documentation just cuts out t0-t3 for n32/n64,
  // GNU pushes the values of t0-t3 to override the o32/o64 values for t4-t7
  // We are supporting both cases, so for t0-t3 we'll just push them to t4-t7.
  if (8 <= CC && CC <= 11)
    CC += 4;

  if (CC == -1)
    CC = StringSwitch<unsigned>(Name)
             .Case("a4", 8)
             .Case("a5", 9)
             .Case("a6", 10)
             .Case("a7", 11)
             .Case("kt0", 26)
             .Case("kt1", 27)
             .Default(-1);

  return CC;
}

int FgpuAsmParser::matchHWRegsRegisterName(StringRef Name) {
  int CC;

  CC = StringSwitch<unsigned>(Name)
            .Case("hwr_cpunum", 0)
            .Case("hwr_synci_step", 1)
            .Case("hwr_cc", 2)
            .Case("hwr_ccres", 3)
            .Case("hwr_ulr", 29)
            .Default(-1);

  return CC;
}

int FgpuAsmParser::matchFPURegisterName(StringRef Name) {
  if (Name[0] == 'f') {
    StringRef NumString = Name.substr(1);
    unsigned IntVal;
    if (NumString.getAsInteger(10, IntVal))
      return -1;     // This is not an integer.
    if (IntVal > 31) // Maximum index for fpu register.
      return -1;
    return IntVal;
  }
  return -1;
}

int FgpuAsmParser::matchFCCRegisterName(StringRef Name) {
  if (Name.startswith("fcc")) {
    StringRef NumString = Name.substr(3);
    unsigned IntVal;
    if (NumString.getAsInteger(10, IntVal))
      return -1;    // This is not an integer.
    if (IntVal > 7) // There are only 8 fcc registers.
      return -1;
    return IntVal;
  }
  return -1;
}

int FgpuAsmParser::matchACRegisterName(StringRef Name) {
  if (Name.startswith("ac")) {
    StringRef NumString = Name.substr(2);
    unsigned IntVal;
    if (NumString.getAsInteger(10, IntVal))
      return -1;    // This is not an integer.
    if (IntVal > 3) // There are only 3 acc registers.
      return -1;
    return IntVal;
  }
  return -1;
}

int FgpuAsmParser::matchMSA128RegisterName(StringRef Name) {
  unsigned IntVal;

  if (Name.front() != 'w' || Name.drop_front(1).getAsInteger(10, IntVal))
    return -1;

  if (IntVal > 31)
    return -1;

  return IntVal;
}

int FgpuAsmParser::matchMSA128CtrlRegisterName(StringRef Name) {
  int CC;

  CC = StringSwitch<unsigned>(Name)
           .Case("msair", 0)
           .Case("msacsr", 1)
           .Case("msaaccess", 2)
           .Case("msasave", 3)
           .Case("msamodify", 4)
           .Case("msarequest", 5)
           .Case("msamap", 6)
           .Case("msaunmap", 7)
           .Default(-1);

  return CC;
}

bool FgpuAsmParser::canUseATReg() {
  return AssemblerOptions.back()->getATRegIndex() != 0;
}

unsigned FgpuAsmParser::getATReg(SMLoc Loc) {
  unsigned ATIndex = AssemblerOptions.back()->getATRegIndex();
  if (ATIndex == 0) {
    reportParseError(Loc,
                     "pseudo-instruction requires $at, which is not available");
    return 0;
  }
  unsigned AT = getReg(
      (isGP64bit()) ? Fgpu::GPR64RegClassID : Fgpu::GPR32RegClassID, ATIndex);
  return AT;
}

unsigned FgpuAsmParser::getReg(int RC, int RegNo) {
  return *(getContext().getRegisterInfo()->getRegClass(RC).begin() + RegNo);
}

bool FgpuAsmParser::parseOperand(OperandVector &Operands, StringRef Mnemonic) {
  MCAsmParser &Parser = getParser();
  LLVM_DEBUG(dbgs() << "parseOperand\n");

  // Check if the current operand has a custom associated parser, if so, try to
  // custom parse the operand, or fallback to the general approach.
  OperandMatchResultTy ResTy = MatchOperandParserImpl(Operands, Mnemonic);
  if (ResTy == MatchOperand_Success)
    return false;
  // If there wasn't a custom match, try the generic matcher below. Otherwise,
  // there was a match, but an error occurred, in which case, just return that
  // the operand parsing failed.
  if (ResTy == MatchOperand_ParseFail)
    return true;

  LLVM_DEBUG(dbgs() << ".. Generic Parser\n");

  switch (getLexer().getKind()) {
  case AsmToken::Dollar: {
    // Parse the register.
    SMLoc S = Parser.getTok().getLoc();

    // Almost all registers have been parsed by custom parsers. There is only
    // one exception to this. $zero (and it's alias $0) will reach this point
    // for div, divu, and similar instructions because it is not an operand
    // to the instruction definition but an explicit register. Special case
    // this situation for now.
    if (parseAnyRegister(Operands) != MatchOperand_NoMatch)
      return false;

    // Maybe it is a symbol reference.
    StringRef Identifier;
    if (Parser.parseIdentifier(Identifier))
      return true;

    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
    MCSymbol *Sym = getContext().getOrCreateSymbol("$" + Identifier);
    // Otherwise create a symbol reference.
    const MCExpr *Res =
        MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());

    Operands.push_back(FgpuOperand::CreateImm(Res, S, E, *this));
    return false;
  }
  default: {
    LLVM_DEBUG(dbgs() << ".. generic integer expression\n");

    const MCExpr *Expr;
    SMLoc S = Parser.getTok().getLoc(); // Start location of the operand.
    if (getParser().parseExpression(Expr))
      return true;

    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

    Operands.push_back(FgpuOperand::CreateImm(Expr, S, E, *this));
    return false;
  }
  } // switch(getLexer().getKind())
  return true;
}

bool FgpuAsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                  SMLoc &EndLoc) {
  return tryParseRegister(RegNo, StartLoc, EndLoc) != MatchOperand_Success;
}

OperandMatchResultTy FgpuAsmParser::tryParseRegister(unsigned &RegNo,
                                                     SMLoc &StartLoc,
                                                     SMLoc &EndLoc) {
  SmallVector<std::unique_ptr<MCParsedAsmOperand>, 1> Operands;
  OperandMatchResultTy ResTy = parseAnyRegister(Operands);
  if (ResTy == MatchOperand_Success) {
    assert(Operands.size() == 1);
    FgpuOperand &Operand = static_cast<FgpuOperand &>(*Operands.front());
    StartLoc = Operand.getStartLoc();
    EndLoc = Operand.getEndLoc();

    // AFAIK, we only support numeric registers and named GPR's in CFI
    // directives.
    // Don't worry about eating tokens before failing. Using an unrecognised
    // register is a parse error.
    if (Operand.isGPRAsmReg()) {
      // Resolve to GPR32 or GPR64 appropriately.
      RegNo = isGP64bit() ? Operand.getGPR64Reg() : Operand.getGPR32Reg();
    }

    return (RegNo == (unsigned)-1) ? MatchOperand_NoMatch
                                   : MatchOperand_Success;
  }

  assert(Operands.size() == 0);
  return (RegNo == (unsigned)-1) ? MatchOperand_NoMatch : MatchOperand_Success;
}

bool FgpuAsmParser::parseMemOffset(const MCExpr *&Res, bool isParenExpr) {
  SMLoc S;

  if (isParenExpr)
    return getParser().parseParenExprOfDepth(0, Res, S);
  return getParser().parseExpression(Res);
}

OperandMatchResultTy
FgpuAsmParser::parseMemOperand(OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  LLVM_DEBUG(dbgs() << "parseMemOperand\n");
  const MCExpr *IdVal = nullptr;
  SMLoc S;
  bool isParenExpr = false;
  OperandMatchResultTy Res = MatchOperand_NoMatch;
  // First operand is the offset.
  S = Parser.getTok().getLoc();

  if (getLexer().getKind() == AsmToken::LParen) {
    Parser.Lex();
    isParenExpr = true;
  }

  if (getLexer().getKind() != AsmToken::Dollar) {
    if (parseMemOffset(IdVal, isParenExpr))
      return MatchOperand_ParseFail;

    const AsmToken &Tok = Parser.getTok(); // Get the next token.
    if (Tok.isNot(AsmToken::LParen)) {
      FgpuOperand &Mnemonic = static_cast<FgpuOperand &>(*Operands[0]);
      if (Mnemonic.getToken() == "la" || Mnemonic.getToken() == "dla") {
        SMLoc E =
            SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
        Operands.push_back(FgpuOperand::CreateImm(IdVal, S, E, *this));
        return MatchOperand_Success;
      }
      if (Tok.is(AsmToken::EndOfStatement)) {
        SMLoc E =
            SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

        // Zero register assumed, add a memory operand with ZERO as its base.
        // "Base" will be managed by k_Memory.
        auto Base = FgpuOperand::createGPRReg(
            0, "0", getContext().getRegisterInfo(), S, E, *this);
        Operands.push_back(
            FgpuOperand::CreateMem(std::move(Base), IdVal, S, E, *this));
        return MatchOperand_Success;
      }
      MCBinaryExpr::Opcode Opcode;
      // GAS and LLVM treat comparison operators different. GAS will generate -1
      // or 0, while LLVM will generate 0 or 1. Since a comparsion operator is
      // highly unlikely to be found in a memory offset expression, we don't
      // handle them.
      switch (Tok.getKind()) {
      case AsmToken::Plus:
        Opcode = MCBinaryExpr::Add;
        Parser.Lex();
        break;
      case AsmToken::Minus:
        Opcode = MCBinaryExpr::Sub;
        Parser.Lex();
        break;
      case AsmToken::Star:
        Opcode = MCBinaryExpr::Mul;
        Parser.Lex();
        break;
      case AsmToken::Pipe:
        Opcode = MCBinaryExpr::Or;
        Parser.Lex();
        break;
      case AsmToken::Amp:
        Opcode = MCBinaryExpr::And;
        Parser.Lex();
        break;
      case AsmToken::LessLess:
        Opcode = MCBinaryExpr::Shl;
        Parser.Lex();
        break;
      case AsmToken::GreaterGreater:
        Opcode = MCBinaryExpr::LShr;
        Parser.Lex();
        break;
      case AsmToken::Caret:
        Opcode = MCBinaryExpr::Xor;
        Parser.Lex();
        break;
      case AsmToken::Slash:
        Opcode = MCBinaryExpr::Div;
        Parser.Lex();
        break;
      case AsmToken::Percent:
        Opcode = MCBinaryExpr::Mod;
        Parser.Lex();
        break;
      default:
        Error(Parser.getTok().getLoc(), "'(' or expression expected");
        return MatchOperand_ParseFail;
      }
      const MCExpr * NextExpr;
      if (getParser().parseExpression(NextExpr))
        return MatchOperand_ParseFail;
      IdVal = MCBinaryExpr::create(Opcode, IdVal, NextExpr, getContext());
    }

    Parser.Lex(); // Eat the '(' token.
  }

  Res = parseAnyRegister(Operands);
  if (Res != MatchOperand_Success)
    return Res;

  if (Parser.getTok().isNot(AsmToken::RParen)) {
    Error(Parser.getTok().getLoc(), "')' expected");
    return MatchOperand_ParseFail;
  }

  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

  Parser.Lex(); // Eat the ')' token.

  if (!IdVal)
    IdVal = MCConstantExpr::create(0, getContext());

  // Replace the register operand with the memory operand.
  std::unique_ptr<FgpuOperand> op(
      static_cast<FgpuOperand *>(Operands.back().release()));
  // Remove the register from the operands.
  // "op" will be managed by k_Memory.
  Operands.pop_back();
  // Add the memory operand.
  if (const MCBinaryExpr *BE = dyn_cast<MCBinaryExpr>(IdVal)) {
    int64_t Imm;
    if (IdVal->evaluateAsAbsolute(Imm))
      IdVal = MCConstantExpr::create(Imm, getContext());
    else if (BE->getLHS()->getKind() != MCExpr::SymbolRef)
      IdVal = MCBinaryExpr::create(BE->getOpcode(), BE->getRHS(), BE->getLHS(),
                                   getContext());
  }

  Operands.push_back(FgpuOperand::CreateMem(std::move(op), IdVal, S, E, *this));
  return MatchOperand_Success;
}

bool FgpuAsmParser::searchSymbolAlias(OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  MCSymbol *Sym = getContext().lookupSymbol(Parser.getTok().getIdentifier());
  if (!Sym)
    return false;

  SMLoc S = Parser.getTok().getLoc();
  if (Sym->isVariable()) {
    const MCExpr *Expr = Sym->getVariableValue();
    if (Expr->getKind() == MCExpr::SymbolRef) {
      const MCSymbolRefExpr *Ref = static_cast<const MCSymbolRefExpr *>(Expr);
      StringRef DefSymbol = Ref->getSymbol().getName();
      if (DefSymbol.startswith("$")) {
        OperandMatchResultTy ResTy =
            matchAnyRegisterNameWithoutDollar(Operands, DefSymbol.substr(1), S);
        if (ResTy == MatchOperand_Success) {
          Parser.Lex();
          return true;
        }
        if (ResTy == MatchOperand_ParseFail)
          llvm_unreachable("Should never ParseFail");
      }
    }
  } else if (Sym->isUnset()) {
    // If symbol is unset, it might be created in the `parseSetAssignment`
    // routine as an alias for a numeric register name.
    // Lookup in the aliases list.
    auto Entry = RegisterSets.find(Sym->getName());
    if (Entry != RegisterSets.end()) {
      OperandMatchResultTy ResTy =
          matchAnyRegisterWithoutDollar(Operands, Entry->getValue(), S);
      if (ResTy == MatchOperand_Success) {
        Parser.Lex();
        return true;
      }
    }
  }

  return false;
}

OperandMatchResultTy
FgpuAsmParser::matchAnyRegisterNameWithoutDollar(OperandVector &Operands,
                                                 StringRef Identifier,
                                                 SMLoc S) {
  int Index = matchCPURegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(FgpuOperand::createGPRReg(
        Index, Identifier, getContext().getRegisterInfo(), S,
        getLexer().getLoc(), *this));
    return MatchOperand_Success;
  }

  Index = matchHWRegsRegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(FgpuOperand::createHWRegsReg(
        Index, Identifier, getContext().getRegisterInfo(), S,
        getLexer().getLoc(), *this));
    return MatchOperand_Success;
  }

  Index = matchFPURegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(FgpuOperand::createFGRReg(
        Index, Identifier, getContext().getRegisterInfo(), S,
        getLexer().getLoc(), *this));
    return MatchOperand_Success;
  }

  Index = matchFCCRegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(FgpuOperand::createFCCReg(
        Index, Identifier, getContext().getRegisterInfo(), S,
        getLexer().getLoc(), *this));
    return MatchOperand_Success;
  }

  Index = matchACRegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(FgpuOperand::createACCReg(
        Index, Identifier, getContext().getRegisterInfo(), S,
        getLexer().getLoc(), *this));
    return MatchOperand_Success;
  }

  Index = matchMSA128RegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(FgpuOperand::createMSA128Reg(
        Index, Identifier, getContext().getRegisterInfo(), S,
        getLexer().getLoc(), *this));
    return MatchOperand_Success;
  }

  Index = matchMSA128CtrlRegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(FgpuOperand::createMSACtrlReg(
        Index, Identifier, getContext().getRegisterInfo(), S,
        getLexer().getLoc(), *this));
    return MatchOperand_Success;
  }

  return MatchOperand_NoMatch;
}

OperandMatchResultTy
FgpuAsmParser::matchAnyRegisterWithoutDollar(OperandVector &Operands,
                                             const AsmToken &Token, SMLoc S) {
  if (Token.is(AsmToken::Identifier)) {
    LLVM_DEBUG(dbgs() << ".. identifier\n");
    StringRef Identifier = Token.getIdentifier();
    OperandMatchResultTy ResTy =
        matchAnyRegisterNameWithoutDollar(Operands, Identifier, S);
    return ResTy;
  } else if (Token.is(AsmToken::Integer)) {
    LLVM_DEBUG(dbgs() << ".. integer\n");
    int64_t RegNum = Token.getIntVal();
    if (RegNum < 0 || RegNum > 31) {
      // Show the error, but treat invalid register
      // number as a normal one to continue parsing
      // and catch other possible errors.
      Error(getLexer().getLoc(), "invalid register number");
    }
    Operands.push_back(FgpuOperand::createNumericReg(
        RegNum, Token.getString(), getContext().getRegisterInfo(), S,
        Token.getLoc(), *this));
    return MatchOperand_Success;
  }

  LLVM_DEBUG(dbgs() << Token.getKind() << "\n");

  return MatchOperand_NoMatch;
}

OperandMatchResultTy
FgpuAsmParser::matchAnyRegisterWithoutDollar(OperandVector &Operands, SMLoc S) {
  auto Token = getLexer().peekTok(false);
  return matchAnyRegisterWithoutDollar(Operands, Token, S);
}

OperandMatchResultTy
FgpuAsmParser::parseAnyRegister(OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  LLVM_DEBUG(dbgs() << "parseAnyRegister\n");

  auto Token = Parser.getTok();

  SMLoc S = Token.getLoc();

  if (Token.isNot(AsmToken::Dollar)) {
    LLVM_DEBUG(dbgs() << ".. !$ -> try sym aliasing\n");
    if (Token.is(AsmToken::Identifier)) {
      if (searchSymbolAlias(Operands))
        return MatchOperand_Success;
    }
    LLVM_DEBUG(dbgs() << ".. !symalias -> NoMatch\n");
    return MatchOperand_NoMatch;
  }
  LLVM_DEBUG(dbgs() << ".. $\n");

  OperandMatchResultTy ResTy = matchAnyRegisterWithoutDollar(Operands, S);
  if (ResTy == MatchOperand_Success) {
    Parser.Lex(); // $
    Parser.Lex(); // identifier
  }
  return ResTy;
}

OperandMatchResultTy
FgpuAsmParser::parseJumpTarget(OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  LLVM_DEBUG(dbgs() << "parseJumpTarget\n");

  SMLoc S = getLexer().getLoc();

  // Registers are a valid target and have priority over symbols.
  OperandMatchResultTy ResTy = parseAnyRegister(Operands);
  if (ResTy != MatchOperand_NoMatch)
    return ResTy;

  // Integers and expressions are acceptable
  const MCExpr *Expr = nullptr;
  if (Parser.parseExpression(Expr)) {
    // We have no way of knowing if a symbol was consumed so we must ParseFail
    return MatchOperand_ParseFail;
  }
  Operands.push_back(
      FgpuOperand::CreateImm(Expr, S, getLexer().getLoc(), *this));
  return MatchOperand_Success;
}

OperandMatchResultTy
FgpuAsmParser::parseInvNum(OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  const MCExpr *IdVal;
  // If the first token is '$' we may have register operand. We have to reject
  // cases where it is not a register. Complicating the matter is that
  // register names are not reserved across all ABIs.
  // Peek past the dollar to see if it's a register name for this ABI.
  SMLoc S = Parser.getTok().getLoc();
  if (Parser.getTok().is(AsmToken::Dollar)) {
    return matchCPURegisterName(Parser.getLexer().peekTok().getString()) == -1
               ? MatchOperand_ParseFail
               : MatchOperand_NoMatch;
  }
  if (getParser().parseExpression(IdVal))
    return MatchOperand_ParseFail;
  const MCConstantExpr *MCE = dyn_cast<MCConstantExpr>(IdVal);
  if (!MCE)
    return MatchOperand_NoMatch;
  int64_t Val = MCE->getValue();
  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  Operands.push_back(FgpuOperand::CreateImm(
      MCConstantExpr::create(0 - Val, getContext()), S, E, *this));
  return MatchOperand_Success;
}

OperandMatchResultTy
FgpuAsmParser::parseRegisterList(OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  SmallVector<unsigned, 10> Regs;
  unsigned RegNo;
  unsigned PrevReg = Fgpu::NoRegister;
  bool RegRange = false;
  SmallVector<std::unique_ptr<MCParsedAsmOperand>, 8> TmpOperands;

  if (Parser.getTok().isNot(AsmToken::Dollar))
    return MatchOperand_ParseFail;

  SMLoc S = Parser.getTok().getLoc();
  while (parseAnyRegister(TmpOperands) == MatchOperand_Success) {
    SMLoc E = getLexer().getLoc();
    FgpuOperand &Reg = static_cast<FgpuOperand &>(*TmpOperands.back());
    RegNo = isGP64bit() ? Reg.getGPR64Reg() : Reg.getGPR32Reg();
    if (RegRange) {
      // Remove last register operand because registers from register range
      // should be inserted first.
      if ((isGP64bit() && RegNo == Fgpu::RA_64) ||
          (!isGP64bit() && RegNo == Fgpu::RA)) {
        Regs.push_back(RegNo);
      } else {
        unsigned TmpReg = PrevReg + 1;
        while (TmpReg <= RegNo) {
          if ((((TmpReg < Fgpu::S0) || (TmpReg > Fgpu::S7)) && !isGP64bit()) ||
              (((TmpReg < Fgpu::S0_64) || (TmpReg > Fgpu::S7_64)) &&
               isGP64bit())) {
            Error(E, "invalid register operand");
            return MatchOperand_ParseFail;
          }

          PrevReg = TmpReg;
          Regs.push_back(TmpReg++);
        }
      }

      RegRange = false;
    } else {
      if ((PrevReg == Fgpu::NoRegister) &&
          ((isGP64bit() && (RegNo != Fgpu::S0_64) && (RegNo != Fgpu::RA_64)) ||
          (!isGP64bit() && (RegNo != Fgpu::S0) && (RegNo != Fgpu::RA)))) {
        Error(E, "$16 or $31 expected");
        return MatchOperand_ParseFail;
      } else if (!(((RegNo == Fgpu::FP || RegNo == Fgpu::RA ||
                    (RegNo >= Fgpu::S0 && RegNo <= Fgpu::S7)) &&
                    !isGP64bit()) ||
                   ((RegNo == Fgpu::FP_64 || RegNo == Fgpu::RA_64 ||
                    (RegNo >= Fgpu::S0_64 && RegNo <= Fgpu::S7_64)) &&
                    isGP64bit()))) {
        Error(E, "invalid register operand");
        return MatchOperand_ParseFail;
      } else if ((PrevReg != Fgpu::NoRegister) && (RegNo != PrevReg + 1) &&
                 ((RegNo != Fgpu::FP && RegNo != Fgpu::RA && !isGP64bit()) ||
                  (RegNo != Fgpu::FP_64 && RegNo != Fgpu::RA_64 &&
                   isGP64bit()))) {
        Error(E, "consecutive register numbers expected");
        return MatchOperand_ParseFail;
      }

      Regs.push_back(RegNo);
    }

    if (Parser.getTok().is(AsmToken::Minus))
      RegRange = true;

    if (!Parser.getTok().isNot(AsmToken::Minus) &&
        !Parser.getTok().isNot(AsmToken::Comma)) {
      Error(E, "',' or '-' expected");
      return MatchOperand_ParseFail;
    }

    Lex(); // Consume comma or minus
    if (Parser.getTok().isNot(AsmToken::Dollar))
      break;

    PrevReg = RegNo;
  }

  SMLoc E = Parser.getTok().getLoc();
  Operands.push_back(FgpuOperand::CreateRegList(Regs, S, E, *this));
  parseMemOperand(Operands);
  return MatchOperand_Success;
}

/// Sometimes (i.e. load/stores) the operand may be followed immediately by
/// either this.
/// ::= '(', register, ')'
/// handle it before we iterate so we don't get tripped up by the lack of
/// a comma.
bool FgpuAsmParser::parseParenSuffix(StringRef Name, OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  if (getLexer().is(AsmToken::LParen)) {
    Operands.push_back(
        FgpuOperand::CreateToken("(", getLexer().getLoc(), *this));
    Parser.Lex();
    if (parseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      return Error(Loc, "unexpected token in argument list");
    }
    if (Parser.getTok().isNot(AsmToken::RParen)) {
      SMLoc Loc = getLexer().getLoc();
      return Error(Loc, "unexpected token, expected ')'");
    }
    Operands.push_back(
        FgpuOperand::CreateToken(")", getLexer().getLoc(), *this));
    Parser.Lex();
  }
  return false;
}

/// Sometimes (i.e. in MSA) the operand may be followed immediately by
/// either one of these.
/// ::= '[', register, ']'
/// ::= '[', integer, ']'
/// handle it before we iterate so we don't get tripped up by the lack of
/// a comma.
bool FgpuAsmParser::parseBracketSuffix(StringRef Name,
                                       OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  if (getLexer().is(AsmToken::LBrac)) {
    Operands.push_back(
        FgpuOperand::CreateToken("[", getLexer().getLoc(), *this));
    Parser.Lex();
    if (parseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      return Error(Loc, "unexpected token in argument list");
    }
    if (Parser.getTok().isNot(AsmToken::RBrac)) {
      SMLoc Loc = getLexer().getLoc();
      return Error(Loc, "unexpected token, expected ']'");
    }
    Operands.push_back(
        FgpuOperand::CreateToken("]", getLexer().getLoc(), *this));
    Parser.Lex();
  }
  return false;
}

static std::string FgpuMnemonicSpellCheck(StringRef S, const FeatureBitset &FBS,
                                          unsigned VariantID = 0);

bool FgpuAsmParser::ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                                     SMLoc NameLoc, OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  LLVM_DEBUG(dbgs() << "ParseInstruction\n");

  // We have reached first instruction, module directive are now forbidden.
  getTargetStreamer().forbidModuleDirective();

  // Check if we have valid mnemonic
  if (!mnemonicIsValid(Name, 0)) {
    FeatureBitset FBS = ComputeAvailableFeatures(getSTI().getFeatureBits());
    std::string Suggestion = FgpuMnemonicSpellCheck(Name, FBS);
    return Error(NameLoc, "unknown instruction" + Suggestion);
  }
  // First operand in MCInst is instruction mnemonic.
  Operands.push_back(FgpuOperand::CreateToken(Name, NameLoc, *this));

  // Read the remaining operands.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    // Read the first operand.
    if (parseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      return Error(Loc, "unexpected token in argument list");
    }
    if (getLexer().is(AsmToken::LBrac) && parseBracketSuffix(Name, Operands))
      return true;
    // AFAIK, parenthesis suffixes are never on the first operand

    while (getLexer().is(AsmToken::Comma)) {
      Parser.Lex(); // Eat the comma.
      // Parse and remember the operand.
      if (parseOperand(Operands, Name)) {
        SMLoc Loc = getLexer().getLoc();
        return Error(Loc, "unexpected token in argument list");
      }
      // Parse bracket and parenthesis suffixes before we iterate
      if (getLexer().is(AsmToken::LBrac)) {
        if (parseBracketSuffix(Name, Operands))
          return true;
      } else if (getLexer().is(AsmToken::LParen) &&
                 parseParenSuffix(Name, Operands))
        return true;
    }
  }
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    return Error(Loc, "unexpected token in argument list");
  }
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

// FIXME: Given that these have the same name, these should both be
// consistent on affecting the Parser.
bool FgpuAsmParser::reportParseError(const Twine &ErrorMsg) {
  SMLoc Loc = getLexer().getLoc();
  return Error(Loc, ErrorMsg);
}

bool FgpuAsmParser::reportParseError(SMLoc Loc, const Twine &ErrorMsg) {
  return Error(Loc, ErrorMsg);
}

bool FgpuAsmParser::parseSetNoAtDirective() {
  MCAsmParser &Parser = getParser();
  // Line should look like: ".set noat".

  // Set the $at register to $0.
  AssemblerOptions.back()->setATRegIndex(0);

  Parser.Lex(); // Eat "noat".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  getTargetStreamer().emitDirectiveSetNoAt();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseSetAtDirective() {
  // Line can be: ".set at", which sets $at to $1
  //          or  ".set at=$reg", which sets $at to $reg.
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "at".

  if (getLexer().is(AsmToken::EndOfStatement)) {
    // No register was specified, so we set $at to $1.
    AssemblerOptions.back()->setATRegIndex(1);

    getTargetStreamer().emitDirectiveSetAt();
    Parser.Lex(); // Consume the EndOfStatement.
    return false;
  }

  if (getLexer().isNot(AsmToken::Equal)) {
    reportParseError("unexpected token, expected equals sign");
    return false;
  }
  Parser.Lex(); // Eat "=".

  if (getLexer().isNot(AsmToken::Dollar)) {
    if (getLexer().is(AsmToken::EndOfStatement)) {
      reportParseError("no register specified");
      return false;
    } else {
      reportParseError("unexpected token, expected dollar sign '$'");
      return false;
    }
  }
  Parser.Lex(); // Eat "$".

  // Find out what "reg" is.
  unsigned AtRegNo;
  const AsmToken &Reg = Parser.getTok();
  if (Reg.is(AsmToken::Identifier)) {
    AtRegNo = matchCPURegisterName(Reg.getIdentifier());
  } else if (Reg.is(AsmToken::Integer)) {
    AtRegNo = Reg.getIntVal();
  } else {
    reportParseError("unexpected token, expected identifier or integer");
    return false;
  }

  // Check if $reg is a valid register. If it is, set $at to $reg.
  if (!AssemblerOptions.back()->setATRegIndex(AtRegNo)) {
    reportParseError("invalid register");
    return false;
  }
  Parser.Lex(); // Eat "reg".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  getTargetStreamer().emitDirectiveSetAtWithArg(AtRegNo);

  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseSetReorderDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }
  AssemblerOptions.back()->setReorder();
  getTargetStreamer().emitDirectiveSetReorder();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseSetNoReorderDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }
  AssemblerOptions.back()->setNoReorder();
  getTargetStreamer().emitDirectiveSetNoReorder();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseSetMacroDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }
  AssemblerOptions.back()->setMacro();
  getTargetStreamer().emitDirectiveSetMacro();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseSetNoMacroDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }
  if (AssemblerOptions.back()->isReorder()) {
    reportParseError("`noreorder' must be set before `nomacro'");
    return false;
  }
  AssemblerOptions.back()->setNoMacro();
  getTargetStreamer().emitDirectiveSetNoMacro();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseSetMsaDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  setFeatureBits(Fgpu::FeatureMSA, "msa");
  getTargetStreamer().emitDirectiveSetMsa();
  return false;
}

bool FgpuAsmParser::parseSetNoMsaDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  clearFeatureBits(Fgpu::FeatureMSA, "msa");
  getTargetStreamer().emitDirectiveSetNoMsa();
  return false;
}

bool FgpuAsmParser::parseSetNoDspDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "nodsp".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  clearFeatureBits(Fgpu::FeatureDSP, "dsp");
  getTargetStreamer().emitDirectiveSetNoDsp();
  return false;
}

bool FgpuAsmParser::parseSetNoFgpu3DDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "nofgpu3d".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  clearFeatureBits(Fgpu::FeatureFgpu3D, "fgpu3d");
  getTargetStreamer().emitDirectiveSetNoFgpu3D();
  return false;
}

bool FgpuAsmParser::parseSetFgpu16Directive() {
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "fgpu16".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  setFeatureBits(Fgpu::FeatureFgpu16, "fgpu16");
  getTargetStreamer().emitDirectiveSetFgpu16();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseSetNoFgpu16Directive() {
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "nofgpu16".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  clearFeatureBits(Fgpu::FeatureFgpu16, "fgpu16");
  getTargetStreamer().emitDirectiveSetNoFgpu16();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseSetFpDirective() {
  MCAsmParser &Parser = getParser();
  FgpuABIFlagsSection::FpABIKind FpAbiVal;
  // Line can be: .set fp=32
  //              .set fp=xx
  //              .set fp=64
  Parser.Lex(); // Eat fp token
  AsmToken Tok = Parser.getTok();
  if (Tok.isNot(AsmToken::Equal)) {
    reportParseError("unexpected token, expected equals sign '='");
    return false;
  }
  Parser.Lex(); // Eat '=' token.
  Tok = Parser.getTok();

  if (!parseFpABIValue(FpAbiVal, ".set"))
    return false;

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }
  getTargetStreamer().emitDirectiveSetFp(FpAbiVal);
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseSetOddSPRegDirective() {
  MCAsmParser &Parser = getParser();

  Parser.Lex(); // Eat "oddspreg".
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  clearFeatureBits(Fgpu::FeatureNoOddSPReg, "nooddspreg");
  getTargetStreamer().emitDirectiveSetOddSPReg();
  return false;
}

bool FgpuAsmParser::parseSetNoOddSPRegDirective() {
  MCAsmParser &Parser = getParser();

  Parser.Lex(); // Eat "nooddspreg".
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  setFeatureBits(Fgpu::FeatureNoOddSPReg, "nooddspreg");
  getTargetStreamer().emitDirectiveSetNoOddSPReg();
  return false;
}

bool FgpuAsmParser::parseSetMtDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "mt".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  setFeatureBits(Fgpu::FeatureMT, "mt");
  getTargetStreamer().emitDirectiveSetMt();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseSetNoMtDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "nomt".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  clearFeatureBits(Fgpu::FeatureMT, "mt");

  getTargetStreamer().emitDirectiveSetNoMt();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseSetNoCRCDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "nocrc".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  clearFeatureBits(Fgpu::FeatureCRC, "crc");

  getTargetStreamer().emitDirectiveSetNoCRC();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseSetNoVirtDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "novirt".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  clearFeatureBits(Fgpu::FeatureVirt, "virt");

  getTargetStreamer().emitDirectiveSetNoVirt();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseSetNoGINVDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "noginv".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  clearFeatureBits(Fgpu::FeatureGINV, "ginv");

  getTargetStreamer().emitDirectiveSetNoGINV();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseSetPopDirective() {
  MCAsmParser &Parser = getParser();
  SMLoc Loc = getLexer().getLoc();

  Parser.Lex();
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  // Always keep an element on the options "stack" to prevent the user
  // from changing the initial options. This is how we remember them.
  if (AssemblerOptions.size() == 2)
    return reportParseError(Loc, ".set pop with no .set push");

  MCSubtargetInfo &STI = copySTI();
  AssemblerOptions.pop_back();
  setAvailableFeatures(
      ComputeAvailableFeatures(AssemblerOptions.back()->getFeatures()));
  STI.setFeatureBits(AssemblerOptions.back()->getFeatures());

  getTargetStreamer().emitDirectiveSetPop();
  return false;
}

bool FgpuAsmParser::parseSetPushDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  // Create a copy of the current assembler options environment and push it.
  AssemblerOptions.push_back(
        std::make_unique<FgpuAssemblerOptions>(AssemblerOptions.back().get()));

  getTargetStreamer().emitDirectiveSetPush();
  return false;
}

bool FgpuAsmParser::parseSetSoftFloatDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  setFeatureBits(Fgpu::FeatureSoftFloat, "soft-float");
  getTargetStreamer().emitDirectiveSetSoftFloat();
  return false;
}

bool FgpuAsmParser::parseSetHardFloatDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  clearFeatureBits(Fgpu::FeatureSoftFloat, "soft-float");
  getTargetStreamer().emitDirectiveSetHardFloat();
  return false;
}

bool FgpuAsmParser::parseSetAssignment() {
  StringRef Name;
  MCAsmParser &Parser = getParser();

  if (Parser.parseIdentifier(Name))
    return reportParseError("expected identifier after .set");

  if (getLexer().isNot(AsmToken::Comma))
    return reportParseError("unexpected token, expected comma");
  Lex(); // Eat comma

  if (getLexer().is(AsmToken::Dollar) &&
      getLexer().peekTok().is(AsmToken::Integer)) {
    // Parse assignment of a numeric register:
    //   .set r1,$1
    Parser.Lex(); // Eat $.
    RegisterSets[Name] = Parser.getTok();
    Parser.Lex(); // Eat identifier.
    getContext().getOrCreateSymbol(Name);
    return false;
  }

  MCSymbol *Sym;
  const MCExpr *Value;
  if (MCParserUtils::parseAssignmentExpression(Name, /* allow_redef */ true,
                                               Parser, Sym, Value))
    return true;
  Sym->setVariableValue(Value);

  return false;
}

bool FgpuAsmParser::parseSetFgpu0Directive() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  // Reset assembler options to their initial values.
  MCSubtargetInfo &STI = copySTI();
  setAvailableFeatures(
      ComputeAvailableFeatures(AssemblerOptions.front()->getFeatures()));
  STI.setFeatureBits(AssemblerOptions.front()->getFeatures());
  AssemblerOptions.back()->setFeatures(AssemblerOptions.front()->getFeatures());

  getTargetStreamer().emitDirectiveSetFgpu0();
  return false;
}

bool FgpuAsmParser::parseSetArchDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  if (getLexer().isNot(AsmToken::Equal))
    return reportParseError("unexpected token, expected equals sign");

  Parser.Lex();
  StringRef Arch = getParser().parseStringToEndOfStatement().trim();
  if (Arch.empty())
    return reportParseError("expected arch identifier");

  StringRef ArchFeatureName =
      StringSwitch<StringRef>(Arch)
          .Case("fgpu1", "fgpu1")
          .Case("fgpu2", "fgpu2")
          .Case("fgpu3", "fgpu3")
          .Case("fgpu4", "fgpu4")
          .Case("fgpu5", "fgpu5")
          .Case("fgpu32", "fgpu32")
          .Case("fgpu32r2", "fgpu32r2")
          .Case("fgpu32r3", "fgpu32r3")
          .Case("fgpu32r5", "fgpu32r5")
          .Case("fgpu32r6", "fgpu32r6")
          .Case("fgpu64", "fgpu64")
          .Case("fgpu64r2", "fgpu64r2")
          .Case("fgpu64r3", "fgpu64r3")
          .Case("fgpu64r5", "fgpu64r5")
          .Case("fgpu64r6", "fgpu64r6")
          .Case("octeon", "cnfgpu")
          .Case("octeon+", "cnfgpup")
          .Case("r4000", "fgpu3") // This is an implementation of Fgpu3.
          .Default("");

  if (ArchFeatureName.empty())
    return reportParseError("unsupported architecture");

  if (ArchFeatureName == "fgpu64r6" && inMicroFgpuMode())
    return reportParseError("fgpu64r6 does not support microFGPU");

  selectArch(ArchFeatureName);
  getTargetStreamer().emitDirectiveSetArch(Arch);
  return false;
}

bool FgpuAsmParser::parseSetFeature(uint64_t Feature) {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  switch (Feature) {
  default:
    llvm_unreachable("Unimplemented feature");
  case Fgpu::FeatureFgpu3D:
    setFeatureBits(Fgpu::FeatureFgpu3D, "fgpu3d");
    getTargetStreamer().emitDirectiveSetFgpu3D();
    break;
  case Fgpu::FeatureDSP:
    setFeatureBits(Fgpu::FeatureDSP, "dsp");
    getTargetStreamer().emitDirectiveSetDsp();
    break;
  case Fgpu::FeatureDSPR2:
    setFeatureBits(Fgpu::FeatureDSPR2, "dspr2");
    getTargetStreamer().emitDirectiveSetDspr2();
    break;
  case Fgpu::FeatureMicroFgpu:
    setFeatureBits(Fgpu::FeatureMicroFgpu, "microfgpu");
    getTargetStreamer().emitDirectiveSetMicroFgpu();
    break;
  case Fgpu::FeatureFgpu1:
    selectArch("fgpu1");
    getTargetStreamer().emitDirectiveSetFgpu1();
    break;
  case Fgpu::FeatureFgpu2:
    selectArch("fgpu2");
    getTargetStreamer().emitDirectiveSetFgpu2();
    break;
  case Fgpu::FeatureFgpu3:
    selectArch("fgpu3");
    getTargetStreamer().emitDirectiveSetFgpu3();
    break;
  case Fgpu::FeatureFgpu4:
    selectArch("fgpu4");
    getTargetStreamer().emitDirectiveSetFgpu4();
    break;
  case Fgpu::FeatureFgpu5:
    selectArch("fgpu5");
    getTargetStreamer().emitDirectiveSetFgpu5();
    break;
  case Fgpu::FeatureFgpu32:
    selectArch("fgpu32");
    getTargetStreamer().emitDirectiveSetFgpu32();
    break;
  case Fgpu::FeatureFgpu32r2:
    selectArch("fgpu32r2");
    getTargetStreamer().emitDirectiveSetFgpu32R2();
    break;
  case Fgpu::FeatureFgpu32r3:
    selectArch("fgpu32r3");
    getTargetStreamer().emitDirectiveSetFgpu32R3();
    break;
  case Fgpu::FeatureFgpu32r5:
    selectArch("fgpu32r5");
    getTargetStreamer().emitDirectiveSetFgpu32R5();
    break;
  case Fgpu::FeatureFgpu32r6:
    selectArch("fgpu32r6");
    getTargetStreamer().emitDirectiveSetFgpu32R6();
    break;
  case Fgpu::FeatureFgpu64:
    selectArch("fgpu64");
    getTargetStreamer().emitDirectiveSetFgpu64();
    break;
  case Fgpu::FeatureFgpu64r2:
    selectArch("fgpu64r2");
    getTargetStreamer().emitDirectiveSetFgpu64R2();
    break;
  case Fgpu::FeatureFgpu64r3:
    selectArch("fgpu64r3");
    getTargetStreamer().emitDirectiveSetFgpu64R3();
    break;
  case Fgpu::FeatureFgpu64r5:
    selectArch("fgpu64r5");
    getTargetStreamer().emitDirectiveSetFgpu64R5();
    break;
  case Fgpu::FeatureFgpu64r6:
    selectArch("fgpu64r6");
    getTargetStreamer().emitDirectiveSetFgpu64R6();
    break;
  case Fgpu::FeatureCRC:
    setFeatureBits(Fgpu::FeatureCRC, "crc");
    getTargetStreamer().emitDirectiveSetCRC();
    break;
  case Fgpu::FeatureVirt:
    setFeatureBits(Fgpu::FeatureVirt, "virt");
    getTargetStreamer().emitDirectiveSetVirt();
    break;
  case Fgpu::FeatureGINV:
    setFeatureBits(Fgpu::FeatureGINV, "ginv");
    getTargetStreamer().emitDirectiveSetGINV();
    break;
  }
  return false;
}

bool FgpuAsmParser::eatComma(StringRef ErrorStr) {
  MCAsmParser &Parser = getParser();
  if (getLexer().isNot(AsmToken::Comma)) {
    SMLoc Loc = getLexer().getLoc();
    return Error(Loc, ErrorStr);
  }

  Parser.Lex(); // Eat the comma.
  return true;
}

// Used to determine if .cpload, .cprestore, and .cpsetup have any effect.
// In this class, it is only used for .cprestore.
// FIXME: Only keep track of IsPicEnabled in one place, instead of in both
// FgpuTargetELFStreamer and FgpuAsmParser.
bool FgpuAsmParser::isPicAndNotNxxAbi() {
  return inPicMode() && !(isABI_N32() || isABI_N64());
}

bool FgpuAsmParser::parseDirectiveCpAdd(SMLoc Loc) {
  SmallVector<std::unique_ptr<MCParsedAsmOperand>, 1> Reg;
  OperandMatchResultTy ResTy = parseAnyRegister(Reg);
  if (ResTy == MatchOperand_NoMatch || ResTy == MatchOperand_ParseFail) {
    reportParseError("expected register");
    return false;
  }

  FgpuOperand &RegOpnd = static_cast<FgpuOperand &>(*Reg[0]);
  if (!RegOpnd.isGPRAsmReg()) {
    reportParseError(RegOpnd.getStartLoc(), "invalid register");
    return false;
  }

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }
  getParser().Lex(); // Consume the EndOfStatement.

  getTargetStreamer().emitDirectiveCpAdd(RegOpnd.getGPR32Reg());
  return false;
}

bool FgpuAsmParser::parseDirectiveCpLoad(SMLoc Loc) {
  if (AssemblerOptions.back()->isReorder())
    Warning(Loc, ".cpload should be inside a noreorder section");

  if (inFgpu16Mode()) {
    reportParseError(".cpload is not supported in Fgpu16 mode");
    return false;
  }

  SmallVector<std::unique_ptr<MCParsedAsmOperand>, 1> Reg;
  OperandMatchResultTy ResTy = parseAnyRegister(Reg);
  if (ResTy == MatchOperand_NoMatch || ResTy == MatchOperand_ParseFail) {
    reportParseError("expected register containing function address");
    return false;
  }

  FgpuOperand &RegOpnd = static_cast<FgpuOperand &>(*Reg[0]);
  if (!RegOpnd.isGPRAsmReg()) {
    reportParseError(RegOpnd.getStartLoc(), "invalid register");
    return false;
  }

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  getTargetStreamer().emitDirectiveCpLoad(RegOpnd.getGPR32Reg());
  return false;
}

bool FgpuAsmParser::parseDirectiveCpLocal(SMLoc Loc) {
  if (!isABI_N32() && !isABI_N64()) {
    reportParseError(".cplocal is allowed only in N32 or N64 mode");
    return false;
  }

  SmallVector<std::unique_ptr<MCParsedAsmOperand>, 1> Reg;
  OperandMatchResultTy ResTy = parseAnyRegister(Reg);
  if (ResTy == MatchOperand_NoMatch || ResTy == MatchOperand_ParseFail) {
    reportParseError("expected register containing global pointer");
    return false;
  }

  FgpuOperand &RegOpnd = static_cast<FgpuOperand &>(*Reg[0]);
  if (!RegOpnd.isGPRAsmReg()) {
    reportParseError(RegOpnd.getStartLoc(), "invalid register");
    return false;
  }

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }
  getParser().Lex(); // Consume the EndOfStatement.

  unsigned NewReg = RegOpnd.getGPR32Reg();
  if (IsPicEnabled)
    GPReg = NewReg;

  getTargetStreamer().emitDirectiveCpLocal(NewReg);
  return false;
}

bool FgpuAsmParser::parseDirectiveCpRestore(SMLoc Loc) {
  MCAsmParser &Parser = getParser();

  // Note that .cprestore is ignored if used with the N32 and N64 ABIs or if it
  // is used in non-PIC mode.

  if (inFgpu16Mode()) {
    reportParseError(".cprestore is not supported in Fgpu16 mode");
    return false;
  }

  // Get the stack offset value.
  const MCExpr *StackOffset;
  int64_t StackOffsetVal;
  if (Parser.parseExpression(StackOffset)) {
    reportParseError("expected stack offset value");
    return false;
  }

  if (!StackOffset->evaluateAsAbsolute(StackOffsetVal)) {
    reportParseError("stack offset is not an absolute expression");
    return false;
  }

  if (StackOffsetVal < 0) {
    Warning(Loc, ".cprestore with negative stack offset has no effect");
    IsCpRestoreSet = false;
  } else {
    IsCpRestoreSet = true;
    CpRestoreOffset = StackOffsetVal;
  }

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  if (!getTargetStreamer().emitDirectiveCpRestore(
          CpRestoreOffset, [&]() { return getATReg(Loc); }, Loc, STI))
    return true;
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseDirectiveCPSetup() {
  MCAsmParser &Parser = getParser();
  unsigned FuncReg;
  unsigned Save;
  bool SaveIsReg = true;

  SmallVector<std::unique_ptr<MCParsedAsmOperand>, 1> TmpReg;
  OperandMatchResultTy ResTy = parseAnyRegister(TmpReg);
  if (ResTy == MatchOperand_NoMatch) {
    reportParseError("expected register containing function address");
    return false;
  }

  FgpuOperand &FuncRegOpnd = static_cast<FgpuOperand &>(*TmpReg[0]);
  if (!FuncRegOpnd.isGPRAsmReg()) {
    reportParseError(FuncRegOpnd.getStartLoc(), "invalid register");
    return false;
  }

  FuncReg = FuncRegOpnd.getGPR32Reg();
  TmpReg.clear();

  if (!eatComma("unexpected token, expected comma"))
    return true;

  ResTy = parseAnyRegister(TmpReg);
  if (ResTy == MatchOperand_NoMatch) {
    const MCExpr *OffsetExpr;
    int64_t OffsetVal;
    SMLoc ExprLoc = getLexer().getLoc();

    if (Parser.parseExpression(OffsetExpr) ||
        !OffsetExpr->evaluateAsAbsolute(OffsetVal)) {
      reportParseError(ExprLoc, "expected save register or stack offset");
      return false;
    }

    Save = OffsetVal;
    SaveIsReg = false;
  } else {
    FgpuOperand &SaveOpnd = static_cast<FgpuOperand &>(*TmpReg[0]);
    if (!SaveOpnd.isGPRAsmReg()) {
      reportParseError(SaveOpnd.getStartLoc(), "invalid register");
      return false;
    }
    Save = SaveOpnd.getGPR32Reg();
  }

  if (!eatComma("unexpected token, expected comma"))
    return true;

  const MCExpr *Expr;
  if (Parser.parseExpression(Expr)) {
    reportParseError("expected expression");
    return false;
  }

  if (Expr->getKind() != MCExpr::SymbolRef) {
    reportParseError("expected symbol");
    return false;
  }
  const MCSymbolRefExpr *Ref = static_cast<const MCSymbolRefExpr *>(Expr);

  CpSaveLocation = Save;
  CpSaveLocationIsRegister = SaveIsReg;

  getTargetStreamer().emitDirectiveCpsetup(FuncReg, Save, Ref->getSymbol(),
                                           SaveIsReg);
  return false;
}

bool FgpuAsmParser::parseDirectiveCPReturn() {
  getTargetStreamer().emitDirectiveCpreturn(CpSaveLocation,
                                            CpSaveLocationIsRegister);
  return false;
}

bool FgpuAsmParser::parseDirectiveNaN() {
  MCAsmParser &Parser = getParser();
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    const AsmToken &Tok = Parser.getTok();

    if (Tok.getString() == "2008") {
      Parser.Lex();
      getTargetStreamer().emitDirectiveNaN2008();
      return false;
    } else if (Tok.getString() == "legacy") {
      Parser.Lex();
      getTargetStreamer().emitDirectiveNaNLegacy();
      return false;
    }
  }
  // If we don't recognize the option passed to the .nan
  // directive (e.g. no option or unknown option), emit an error.
  reportParseError("invalid option in .nan directive");
  return false;
}

bool FgpuAsmParser::parseDirectiveSet() {
  const AsmToken &Tok = getParser().getTok();
  StringRef IdVal = Tok.getString();
  SMLoc Loc = Tok.getLoc();

  if (IdVal == "noat")
    return parseSetNoAtDirective();
  if (IdVal == "at")
    return parseSetAtDirective();
  if (IdVal == "arch")
    return parseSetArchDirective();
  if (IdVal == "bopt") {
    Warning(Loc, "'bopt' feature is unsupported");
    getParser().Lex();
    return false;
  }
  if (IdVal == "nobopt") {
    // We're already running in nobopt mode, so nothing to do.
    getParser().Lex();
    return false;
  }
  if (IdVal == "fp")
    return parseSetFpDirective();
  if (IdVal == "oddspreg")
    return parseSetOddSPRegDirective();
  if (IdVal == "nooddspreg")
    return parseSetNoOddSPRegDirective();
  if (IdVal == "pop")
    return parseSetPopDirective();
  if (IdVal == "push")
    return parseSetPushDirective();
  if (IdVal == "reorder")
    return parseSetReorderDirective();
  if (IdVal == "noreorder")
    return parseSetNoReorderDirective();
  if (IdVal == "macro")
    return parseSetMacroDirective();
  if (IdVal == "nomacro")
    return parseSetNoMacroDirective();
  if (IdVal == "fgpu16")
    return parseSetFgpu16Directive();
  if (IdVal == "nofgpu16")
    return parseSetNoFgpu16Directive();
  if (IdVal == "nomicrofgpu") {
    clearFeatureBits(Fgpu::FeatureMicroFgpu, "microfgpu");
    getTargetStreamer().emitDirectiveSetNoMicroFgpu();
    getParser().eatToEndOfStatement();
    return false;
  }
  if (IdVal == "microfgpu") {
    if (hasFgpu64r6()) {
      Error(Loc, ".set microfgpu directive is not supported with FGPU64R6");
      return false;
    }
    return parseSetFeature(Fgpu::FeatureMicroFgpu);
  }
  if (IdVal == "fgpu0")
    return parseSetFgpu0Directive();
  if (IdVal == "fgpu1")
    return parseSetFeature(Fgpu::FeatureFgpu1);
  if (IdVal == "fgpu2")
    return parseSetFeature(Fgpu::FeatureFgpu2);
  if (IdVal == "fgpu3")
    return parseSetFeature(Fgpu::FeatureFgpu3);
  if (IdVal == "fgpu4")
    return parseSetFeature(Fgpu::FeatureFgpu4);
  if (IdVal == "fgpu5")
    return parseSetFeature(Fgpu::FeatureFgpu5);
  if (IdVal == "fgpu32")
    return parseSetFeature(Fgpu::FeatureFgpu32);
  if (IdVal == "fgpu32r2")
    return parseSetFeature(Fgpu::FeatureFgpu32r2);
  if (IdVal == "fgpu32r3")
    return parseSetFeature(Fgpu::FeatureFgpu32r3);
  if (IdVal == "fgpu32r5")
    return parseSetFeature(Fgpu::FeatureFgpu32r5);
  if (IdVal == "fgpu32r6")
    return parseSetFeature(Fgpu::FeatureFgpu32r6);
  if (IdVal == "fgpu64")
    return parseSetFeature(Fgpu::FeatureFgpu64);
  if (IdVal == "fgpu64r2")
    return parseSetFeature(Fgpu::FeatureFgpu64r2);
  if (IdVal == "fgpu64r3")
    return parseSetFeature(Fgpu::FeatureFgpu64r3);
  if (IdVal == "fgpu64r5")
    return parseSetFeature(Fgpu::FeatureFgpu64r5);
  if (IdVal == "fgpu64r6") {
    if (inMicroFgpuMode()) {
      Error(Loc, "FGPU64R6 is not supported with microFGPU");
      return false;
    }
    return parseSetFeature(Fgpu::FeatureFgpu64r6);
  }
  if (IdVal == "dsp")
    return parseSetFeature(Fgpu::FeatureDSP);
  if (IdVal == "dspr2")
    return parseSetFeature(Fgpu::FeatureDSPR2);
  if (IdVal == "nodsp")
    return parseSetNoDspDirective();
  if (IdVal == "fgpu3d")
    return parseSetFeature(Fgpu::FeatureFgpu3D);
  if (IdVal == "nofgpu3d")
    return parseSetNoFgpu3DDirective();
  if (IdVal == "msa")
    return parseSetMsaDirective();
  if (IdVal == "nomsa")
    return parseSetNoMsaDirective();
  if (IdVal == "mt")
    return parseSetMtDirective();
  if (IdVal == "nomt")
    return parseSetNoMtDirective();
  if (IdVal == "softfloat")
    return parseSetSoftFloatDirective();
  if (IdVal == "hardfloat")
    return parseSetHardFloatDirective();
  if (IdVal == "crc")
    return parseSetFeature(Fgpu::FeatureCRC);
  if (IdVal == "nocrc")
    return parseSetNoCRCDirective();
  if (IdVal == "virt")
    return parseSetFeature(Fgpu::FeatureVirt);
  if (IdVal == "novirt")
    return parseSetNoVirtDirective();
  if (IdVal == "ginv")
    return parseSetFeature(Fgpu::FeatureGINV);
  if (IdVal == "noginv")
    return parseSetNoGINVDirective();

  // It is just an identifier, look for an assignment.
  return parseSetAssignment();
}

/// parseDirectiveGpWord
///  ::= .gpword local_sym
bool FgpuAsmParser::parseDirectiveGpWord() {
  MCAsmParser &Parser = getParser();
  const MCExpr *Value;
  // EmitGPRel32Value requires an expression, so we are using base class
  // method to evaluate the expression.
  if (getParser().parseExpression(Value))
    return true;
  getParser().getStreamer().emitGPRel32Value(Value);

  if (getLexer().isNot(AsmToken::EndOfStatement))
    return Error(getLexer().getLoc(),
                "unexpected token, expected end of statement");
  Parser.Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseDirectiveGpDWord
///  ::= .gpdword local_sym
bool FgpuAsmParser::parseDirectiveGpDWord() {
  MCAsmParser &Parser = getParser();
  const MCExpr *Value;
  // EmitGPRel64Value requires an expression, so we are using base class
  // method to evaluate the expression.
  if (getParser().parseExpression(Value))
    return true;
  getParser().getStreamer().emitGPRel64Value(Value);

  if (getLexer().isNot(AsmToken::EndOfStatement))
    return Error(getLexer().getLoc(),
                "unexpected token, expected end of statement");
  Parser.Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseDirectiveDtpRelWord
///  ::= .dtprelword tls_sym
bool FgpuAsmParser::parseDirectiveDtpRelWord() {
  MCAsmParser &Parser = getParser();
  const MCExpr *Value;
  // EmitDTPRel32Value requires an expression, so we are using base class
  // method to evaluate the expression.
  if (getParser().parseExpression(Value))
    return true;
  getParser().getStreamer().emitDTPRel32Value(Value);

  if (getLexer().isNot(AsmToken::EndOfStatement))
    return Error(getLexer().getLoc(),
                "unexpected token, expected end of statement");
  Parser.Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseDirectiveDtpRelDWord
///  ::= .dtpreldword tls_sym
bool FgpuAsmParser::parseDirectiveDtpRelDWord() {
  MCAsmParser &Parser = getParser();
  const MCExpr *Value;
  // EmitDTPRel64Value requires an expression, so we are using base class
  // method to evaluate the expression.
  if (getParser().parseExpression(Value))
    return true;
  getParser().getStreamer().emitDTPRel64Value(Value);

  if (getLexer().isNot(AsmToken::EndOfStatement))
    return Error(getLexer().getLoc(),
                "unexpected token, expected end of statement");
  Parser.Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseDirectiveTpRelWord
///  ::= .tprelword tls_sym
bool FgpuAsmParser::parseDirectiveTpRelWord() {
  MCAsmParser &Parser = getParser();
  const MCExpr *Value;
  // EmitTPRel32Value requires an expression, so we are using base class
  // method to evaluate the expression.
  if (getParser().parseExpression(Value))
    return true;
  getParser().getStreamer().emitTPRel32Value(Value);

  if (getLexer().isNot(AsmToken::EndOfStatement))
    return Error(getLexer().getLoc(),
                "unexpected token, expected end of statement");
  Parser.Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseDirectiveTpRelDWord
///  ::= .tpreldword tls_sym
bool FgpuAsmParser::parseDirectiveTpRelDWord() {
  MCAsmParser &Parser = getParser();
  const MCExpr *Value;
  // EmitTPRel64Value requires an expression, so we are using base class
  // method to evaluate the expression.
  if (getParser().parseExpression(Value))
    return true;
  getParser().getStreamer().emitTPRel64Value(Value);

  if (getLexer().isNot(AsmToken::EndOfStatement))
    return Error(getLexer().getLoc(),
                "unexpected token, expected end of statement");
  Parser.Lex(); // Eat EndOfStatement token.
  return false;
}

bool FgpuAsmParser::parseDirectiveOption() {
  MCAsmParser &Parser = getParser();
  // Get the option token.
  AsmToken Tok = Parser.getTok();
  // At the moment only identifiers are supported.
  if (Tok.isNot(AsmToken::Identifier)) {
    return Error(Parser.getTok().getLoc(),
                 "unexpected token, expected identifier");
  }

  StringRef Option = Tok.getIdentifier();

  if (Option == "pic0") {
    // FgpuAsmParser needs to know if the current PIC mode changes.
    IsPicEnabled = false;

    getTargetStreamer().emitDirectiveOptionPic0();
    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement)) {
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");
    }
    return false;
  }

  if (Option == "pic2") {
    // FgpuAsmParser needs to know if the current PIC mode changes.
    IsPicEnabled = true;

    getTargetStreamer().emitDirectiveOptionPic2();
    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement)) {
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");
    }
    return false;
  }

  // Unknown option.
  Warning(Parser.getTok().getLoc(),
          "unknown option, expected 'pic0' or 'pic2'");
  Parser.eatToEndOfStatement();
  return false;
}

/// parseInsnDirective
///  ::= .insn
bool FgpuAsmParser::parseInsnDirective() {
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  // The actual label marking happens in
  // FgpuELFStreamer::createPendingLabelRelocs().
  getTargetStreamer().emitDirectiveInsn();

  getParser().Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseRSectionDirective
///  ::= .rdata
bool FgpuAsmParser::parseRSectionDirective(StringRef Section) {
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  MCSection *ELFSection = getContext().getELFSection(
      Section, ELF::SHT_PROGBITS, ELF::SHF_ALLOC);
  getParser().getStreamer().SwitchSection(ELFSection);

  getParser().Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseSSectionDirective
///  ::= .sbss
///  ::= .sdata
bool FgpuAsmParser::parseSSectionDirective(StringRef Section, unsigned Type) {
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  MCSection *ELFSection = getContext().getELFSection(
      Section, Type, ELF::SHF_WRITE | ELF::SHF_ALLOC | ELF::SHF_MIPS_GPREL);
  getParser().getStreamer().SwitchSection(ELFSection);

  getParser().Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseDirectiveModule
///  ::= .module oddspreg
///  ::= .module nooddspreg
///  ::= .module fp=value
///  ::= .module softfloat
///  ::= .module hardfloat
///  ::= .module mt
///  ::= .module crc
///  ::= .module nocrc
///  ::= .module virt
///  ::= .module novirt
///  ::= .module ginv
///  ::= .module noginv
bool FgpuAsmParser::parseDirectiveModule() {
  MCAsmParser &Parser = getParser();
  MCAsmLexer &Lexer = getLexer();
  SMLoc L = Lexer.getLoc();

  if (!getTargetStreamer().isModuleDirectiveAllowed()) {
    // TODO : get a better message.
    reportParseError(".module directive must appear before any code");
    return false;
  }

  StringRef Option;
  if (Parser.parseIdentifier(Option)) {
    reportParseError("expected .module option identifier");
    return false;
  }

  if (Option == "oddspreg") {
    clearModuleFeatureBits(Fgpu::FeatureNoOddSPReg, "nooddspreg");

    // Synchronize the abiflags information with the FeatureBits information we
    // changed above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated abiflags information.
    // If generating ELF, don't do anything (the .FGPU.abiflags section gets
    // emitted at the end).
    getTargetStreamer().emitDirectiveModuleOddSPReg();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else if (Option == "nooddspreg") {
    if (!isABI_O32()) {
      return Error(L, "'.module nooddspreg' requires the O32 ABI");
    }

    setModuleFeatureBits(Fgpu::FeatureNoOddSPReg, "nooddspreg");

    // Synchronize the abiflags information with the FeatureBits information we
    // changed above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated abiflags information.
    // If generating ELF, don't do anything (the .FGPU.abiflags section gets
    // emitted at the end).
    getTargetStreamer().emitDirectiveModuleOddSPReg();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else if (Option == "fp") {
    return parseDirectiveModuleFP();
  } else if (Option == "softfloat") {
    setModuleFeatureBits(Fgpu::FeatureSoftFloat, "soft-float");

    // Synchronize the ABI Flags information with the FeatureBits information we
    // updated above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated ABI Flags information.
    // If generating ELF, don't do anything (the .FGPU.abiflags section gets
    // emitted later).
    getTargetStreamer().emitDirectiveModuleSoftFloat();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else if (Option == "hardfloat") {
    clearModuleFeatureBits(Fgpu::FeatureSoftFloat, "soft-float");

    // Synchronize the ABI Flags information with the FeatureBits information we
    // updated above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated ABI Flags information.
    // If generating ELF, don't do anything (the .FGPU.abiflags section gets
    // emitted later).
    getTargetStreamer().emitDirectiveModuleHardFloat();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else if (Option == "mt") {
    setModuleFeatureBits(Fgpu::FeatureMT, "mt");

    // Synchronize the ABI Flags information with the FeatureBits information we
    // updated above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated ABI Flags information.
    // If generating ELF, don't do anything (the .FGPU.abiflags section gets
    // emitted later).
    getTargetStreamer().emitDirectiveModuleMT();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else if (Option == "crc") {
    setModuleFeatureBits(Fgpu::FeatureCRC, "crc");

    // Synchronize the ABI Flags information with the FeatureBits information we
    // updated above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated ABI Flags information.
    // If generating ELF, don't do anything (the .FGPU.abiflags section gets
    // emitted later).
    getTargetStreamer().emitDirectiveModuleCRC();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else if (Option == "nocrc") {
    clearModuleFeatureBits(Fgpu::FeatureCRC, "crc");

    // Synchronize the ABI Flags information with the FeatureBits information we
    // updated above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated ABI Flags information.
    // If generating ELF, don't do anything (the .FGPU.abiflags section gets
    // emitted later).
    getTargetStreamer().emitDirectiveModuleNoCRC();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else if (Option == "virt") {
    setModuleFeatureBits(Fgpu::FeatureVirt, "virt");

    // Synchronize the ABI Flags information with the FeatureBits information we
    // updated above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated ABI Flags information.
    // If generating ELF, don't do anything (the .FGPU.abiflags section gets
    // emitted later).
    getTargetStreamer().emitDirectiveModuleVirt();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else if (Option == "novirt") {
    clearModuleFeatureBits(Fgpu::FeatureVirt, "virt");

    // Synchronize the ABI Flags information with the FeatureBits information we
    // updated above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated ABI Flags information.
    // If generating ELF, don't do anything (the .FGPU.abiflags section gets
    // emitted later).
    getTargetStreamer().emitDirectiveModuleNoVirt();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else if (Option == "ginv") {
    setModuleFeatureBits(Fgpu::FeatureGINV, "ginv");

    // Synchronize the ABI Flags information with the FeatureBits information we
    // updated above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated ABI Flags information.
    // If generating ELF, don't do anything (the .FGPU.abiflags section gets
    // emitted later).
    getTargetStreamer().emitDirectiveModuleGINV();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else if (Option == "noginv") {
    clearModuleFeatureBits(Fgpu::FeatureGINV, "ginv");

    // Synchronize the ABI Flags information with the FeatureBits information we
    // updated above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated ABI Flags information.
    // If generating ELF, don't do anything (the .FGPU.abiflags section gets
    // emitted later).
    getTargetStreamer().emitDirectiveModuleNoGINV();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else {
    return Error(L, "'" + Twine(Option) + "' is not a valid .module option.");
  }
}

/// parseDirectiveModuleFP
///  ::= =32
///  ::= =xx
///  ::= =64
bool FgpuAsmParser::parseDirectiveModuleFP() {
  MCAsmParser &Parser = getParser();
  MCAsmLexer &Lexer = getLexer();

  if (Lexer.isNot(AsmToken::Equal)) {
    reportParseError("unexpected token, expected equals sign '='");
    return false;
  }
  Parser.Lex(); // Eat '=' token.

  FgpuABIFlagsSection::FpABIKind FpABI;
  if (!parseFpABIValue(FpABI, ".module"))
    return false;

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  // Synchronize the abiflags information with the FeatureBits information we
  // changed above.
  getTargetStreamer().updateABIInfo(*this);

  // If printing assembly, use the recently updated abiflags information.
  // If generating ELF, don't do anything (the .FGPU.abiflags section gets
  // emitted at the end).
  getTargetStreamer().emitDirectiveModuleFP();

  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool FgpuAsmParser::parseFpABIValue(FgpuABIFlagsSection::FpABIKind &FpABI,
                                    StringRef Directive) {
  MCAsmParser &Parser = getParser();
  MCAsmLexer &Lexer = getLexer();
  bool ModuleLevelOptions = Directive == ".module";

  if (Lexer.is(AsmToken::Identifier)) {
    StringRef Value = Parser.getTok().getString();
    Parser.Lex();

    if (Value != "xx") {
      reportParseError("unsupported value, expected 'xx', '32' or '64'");
      return false;
    }

    if (!isABI_O32()) {
      reportParseError("'" + Directive + " fp=xx' requires the O32 ABI");
      return false;
    }

    FpABI = FgpuABIFlagsSection::FpABIKind::XX;
    if (ModuleLevelOptions) {
      setModuleFeatureBits(Fgpu::FeatureFPXX, "fpxx");
      clearModuleFeatureBits(Fgpu::FeatureFP64Bit, "fp64");
    } else {
      setFeatureBits(Fgpu::FeatureFPXX, "fpxx");
      clearFeatureBits(Fgpu::FeatureFP64Bit, "fp64");
    }
    return true;
  }

  if (Lexer.is(AsmToken::Integer)) {
    unsigned Value = Parser.getTok().getIntVal();
    Parser.Lex();

    if (Value != 32 && Value != 64) {
      reportParseError("unsupported value, expected 'xx', '32' or '64'");
      return false;
    }

    if (Value == 32) {
      if (!isABI_O32()) {
        reportParseError("'" + Directive + " fp=32' requires the O32 ABI");
        return false;
      }

      FpABI = FgpuABIFlagsSection::FpABIKind::S32;
      if (ModuleLevelOptions) {
        clearModuleFeatureBits(Fgpu::FeatureFPXX, "fpxx");
        clearModuleFeatureBits(Fgpu::FeatureFP64Bit, "fp64");
      } else {
        clearFeatureBits(Fgpu::FeatureFPXX, "fpxx");
        clearFeatureBits(Fgpu::FeatureFP64Bit, "fp64");
      }
    } else {
      FpABI = FgpuABIFlagsSection::FpABIKind::S64;
      if (ModuleLevelOptions) {
        clearModuleFeatureBits(Fgpu::FeatureFPXX, "fpxx");
        setModuleFeatureBits(Fgpu::FeatureFP64Bit, "fp64");
      } else {
        clearFeatureBits(Fgpu::FeatureFPXX, "fpxx");
        setFeatureBits(Fgpu::FeatureFP64Bit, "fp64");
      }
    }

    return true;
  }

  return false;
}

bool FgpuAsmParser::ParseDirective(AsmToken DirectiveID) {
  // This returns false if this function recognizes the directive
  // regardless of whether it is successfully handles or reports an
  // error. Otherwise it returns true to give the generic parser a
  // chance at recognizing it.

  MCAsmParser &Parser = getParser();
  StringRef IDVal = DirectiveID.getString();

  if (IDVal == ".cpadd") {
    parseDirectiveCpAdd(DirectiveID.getLoc());
    return false;
  }
  if (IDVal == ".cpload") {
    parseDirectiveCpLoad(DirectiveID.getLoc());
    return false;
  }
  if (IDVal == ".cprestore") {
    parseDirectiveCpRestore(DirectiveID.getLoc());
    return false;
  }
  if (IDVal == ".cplocal") {
    parseDirectiveCpLocal(DirectiveID.getLoc());
    return false;
  }
  if (IDVal == ".ent") {
    StringRef SymbolName;

    if (Parser.parseIdentifier(SymbolName)) {
      reportParseError("expected identifier after .ent");
      return false;
    }

    // There's an undocumented extension that allows an integer to
    // follow the name of the procedure which AFAICS is ignored by GAS.
    // Example: .ent foo,2
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      if (getLexer().isNot(AsmToken::Comma)) {
        // Even though we accept this undocumented extension for compatibility
        // reasons, the additional integer argument does not actually change
        // the behaviour of the '.ent' directive, so we would like to discourage
        // its use. We do this by not referring to the extended version in
        // error messages which are not directly related to its use.
        reportParseError("unexpected token, expected end of statement");
        return false;
      }
      Parser.Lex(); // Eat the comma.
      const MCExpr *DummyNumber;
      int64_t DummyNumberVal;
      // If the user was explicitly trying to use the extended version,
      // we still give helpful extension-related error messages.
      if (Parser.parseExpression(DummyNumber)) {
        reportParseError("expected number after comma");
        return false;
      }
      if (!DummyNumber->evaluateAsAbsolute(DummyNumberVal)) {
        reportParseError("expected an absolute expression after comma");
        return false;
      }
    }

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    MCSymbol *Sym = getContext().getOrCreateSymbol(SymbolName);

    getTargetStreamer().emitDirectiveEnt(*Sym);
    CurrentFn = Sym;
    IsCpRestoreSet = false;
    return false;
  }

  if (IDVal == ".end") {
    StringRef SymbolName;

    if (Parser.parseIdentifier(SymbolName)) {
      reportParseError("expected identifier after .end");
      return false;
    }

    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    if (CurrentFn == nullptr) {
      reportParseError(".end used without .ent");
      return false;
    }

    if ((SymbolName != CurrentFn->getName())) {
      reportParseError(".end symbol does not match .ent symbol");
      return false;
    }

    getTargetStreamer().emitDirectiveEnd(SymbolName);
    CurrentFn = nullptr;
    IsCpRestoreSet = false;
    return false;
  }

  if (IDVal == ".frame") {
    // .frame $stack_reg, frame_size_in_bytes, $return_reg
    SmallVector<std::unique_ptr<MCParsedAsmOperand>, 1> TmpReg;
    OperandMatchResultTy ResTy = parseAnyRegister(TmpReg);
    if (ResTy == MatchOperand_NoMatch || ResTy == MatchOperand_ParseFail) {
      reportParseError("expected stack register");
      return false;
    }

    FgpuOperand &StackRegOpnd = static_cast<FgpuOperand &>(*TmpReg[0]);
    if (!StackRegOpnd.isGPRAsmReg()) {
      reportParseError(StackRegOpnd.getStartLoc(),
                       "expected general purpose register");
      return false;
    }
    unsigned StackReg = StackRegOpnd.getGPR32Reg();

    if (Parser.getTok().is(AsmToken::Comma))
      Parser.Lex();
    else {
      reportParseError("unexpected token, expected comma");
      return false;
    }

    // Parse the frame size.
    const MCExpr *FrameSize;
    int64_t FrameSizeVal;

    if (Parser.parseExpression(FrameSize)) {
      reportParseError("expected frame size value");
      return false;
    }

    if (!FrameSize->evaluateAsAbsolute(FrameSizeVal)) {
      reportParseError("frame size not an absolute expression");
      return false;
    }

    if (Parser.getTok().is(AsmToken::Comma))
      Parser.Lex();
    else {
      reportParseError("unexpected token, expected comma");
      return false;
    }

    // Parse the return register.
    TmpReg.clear();
    ResTy = parseAnyRegister(TmpReg);
    if (ResTy == MatchOperand_NoMatch || ResTy == MatchOperand_ParseFail) {
      reportParseError("expected return register");
      return false;
    }

    FgpuOperand &ReturnRegOpnd = static_cast<FgpuOperand &>(*TmpReg[0]);
    if (!ReturnRegOpnd.isGPRAsmReg()) {
      reportParseError(ReturnRegOpnd.getStartLoc(),
                       "expected general purpose register");
      return false;
    }

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    getTargetStreamer().emitFrame(StackReg, FrameSizeVal,
                                  ReturnRegOpnd.getGPR32Reg());
    IsCpRestoreSet = false;
    return false;
  }

  if (IDVal == ".set") {
    parseDirectiveSet();
    return false;
  }

  if (IDVal == ".mask" || IDVal == ".fmask") {
    // .mask bitmask, frame_offset
    // bitmask: One bit for each register used.
    // frame_offset: Offset from Canonical Frame Address ($sp on entry) where
    //               first register is expected to be saved.
    // Examples:
    //   .mask 0x80000000, -4
    //   .fmask 0x80000000, -4
    //

    // Parse the bitmask
    const MCExpr *BitMask;
    int64_t BitMaskVal;

    if (Parser.parseExpression(BitMask)) {
      reportParseError("expected bitmask value");
      return false;
    }

    if (!BitMask->evaluateAsAbsolute(BitMaskVal)) {
      reportParseError("bitmask not an absolute expression");
      return false;
    }

    if (Parser.getTok().is(AsmToken::Comma))
      Parser.Lex();
    else {
      reportParseError("unexpected token, expected comma");
      return false;
    }

    // Parse the frame_offset
    const MCExpr *FrameOffset;
    int64_t FrameOffsetVal;

    if (Parser.parseExpression(FrameOffset)) {
      reportParseError("expected frame offset value");
      return false;
    }

    if (!FrameOffset->evaluateAsAbsolute(FrameOffsetVal)) {
      reportParseError("frame offset not an absolute expression");
      return false;
    }

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    if (IDVal == ".mask")
      getTargetStreamer().emitMask(BitMaskVal, FrameOffsetVal);
    else
      getTargetStreamer().emitFMask(BitMaskVal, FrameOffsetVal);
    return false;
  }

  if (IDVal == ".nan")
    return parseDirectiveNaN();

  if (IDVal == ".gpword") {
    parseDirectiveGpWord();
    return false;
  }

  if (IDVal == ".gpdword") {
    parseDirectiveGpDWord();
    return false;
  }

  if (IDVal == ".dtprelword") {
    parseDirectiveDtpRelWord();
    return false;
  }

  if (IDVal == ".dtpreldword") {
    parseDirectiveDtpRelDWord();
    return false;
  }

  if (IDVal == ".tprelword") {
    parseDirectiveTpRelWord();
    return false;
  }

  if (IDVal == ".tpreldword") {
    parseDirectiveTpRelDWord();
    return false;
  }

  if (IDVal == ".option") {
    parseDirectiveOption();
    return false;
  }

  if (IDVal == ".abicalls") {
    getTargetStreamer().emitDirectiveAbiCalls();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement)) {
      Error(Parser.getTok().getLoc(),
            "unexpected token, expected end of statement");
    }
    return false;
  }

  if (IDVal == ".cpsetup") {
    parseDirectiveCPSetup();
    return false;
  }
  if (IDVal == ".cpreturn") {
    parseDirectiveCPReturn();
    return false;
  }
  if (IDVal == ".module") {
    parseDirectiveModule();
    return false;
  }
  if (IDVal == ".llvm_internal_fgpu_reallow_module_directive") {
    parseInternalDirectiveReallowModule();
    return false;
  }
  if (IDVal == ".insn") {
    parseInsnDirective();
    return false;
  }
  if (IDVal == ".rdata") {
    parseRSectionDirective(".rodata");
    return false;
  }
  if (IDVal == ".sbss") {
    parseSSectionDirective(IDVal, ELF::SHT_NOBITS);
    return false;
  }
  if (IDVal == ".sdata") {
    parseSSectionDirective(IDVal, ELF::SHT_PROGBITS);
    return false;
  }

  return true;
}

bool FgpuAsmParser::parseInternalDirectiveReallowModule() {
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  getTargetStreamer().reallowModuleDirective();

  getParser().Lex(); // Eat EndOfStatement token.
  return false;
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeFgpuAsmParser() {
  RegisterMCAsmParser<FgpuAsmParser> X(getTheFgpuTarget());
  RegisterMCAsmParser<FgpuAsmParser> Y(getTheFgpuelTarget());
  RegisterMCAsmParser<FgpuAsmParser> A(getTheFgpu64Target());
  RegisterMCAsmParser<FgpuAsmParser> B(getTheFgpu64elTarget());
}

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#define GET_MNEMONIC_SPELL_CHECKER
#include "FgpuGenAsmMatcher.inc"

bool FgpuAsmParser::mnemonicIsValid(StringRef Mnemonic, unsigned VariantID) {
  // Find the appropriate table for this asm variant.
  const MatchEntry *Start, *End;
  switch (VariantID) {
  default: llvm_unreachable("invalid variant!");
  case 0: Start = std::begin(MatchTable0); End = std::end(MatchTable0); break;
  }
  // Search the table.
  auto MnemonicRange = std::equal_range(Start, End, Mnemonic, LessOpcode());
  return MnemonicRange.first != MnemonicRange.second;
}
