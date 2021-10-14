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
//    Fgpu::FeatureFgpu1
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

//  void ConvertXWPOperands(MCInst &Inst, const OperandVector &Operands);

#define GET_ASSEMBLER_HEADER
#include "FgpuGenAsmMatcher.inc"

//  unsigned
//  checkEarlyTargetMatchPredicate(MCInst &Inst,
//                                 const OperandVector &Operands) override;
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

//  bool expandJalWithRegs(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
//                         const MCSubtargetInfo *STI);

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

  //bool parseSetFgpu0Directive();
  bool parseSetArchDirective();
  bool parseSetFeature(uint64_t Feature);
  bool isPicAndNotNxxAbi(); // Used by .cpload, .cprestore, and .cpsetup.
  bool parseDirectiveCpAdd(SMLoc Loc);
  bool parseDirectiveCpLoad(SMLoc Loc);
//  bool parseDirectiveCpLocal(SMLoc Loc);
  bool parseDirectiveCpRestore(SMLoc Loc);
  bool parseDirectiveCPSetup();
  bool parseDirectiveCPReturn();
  bool parseDirectiveSet();
  bool parseDirectiveOption();
  bool parseInsnDirective();
  bool parseRSectionDirective(StringRef Section);
  bool parseSSectionDirective(StringRef Section, unsigned Type);

  bool parseSetAtDirective();
  bool parseSetNoAtDirective();
  bool parseSetMacroDirective();
  bool parseSetNoMacroDirective();
  bool parseSetReorderDirective();
  bool parseSetNoReorderDirective();
  bool parseSetPopDirective();
  bool parseSetPushDirective();

  bool parseSetAssignment();

  bool parseDirectiveGpWord();
  bool parseDirectiveGpDWord();
  bool parseDirectiveDtpRelWord();
  bool parseDirectiveDtpRelDWord();
  bool parseDirectiveTpRelWord();
  bool parseDirectiveTpRelDWord();
  bool parseDirectiveModule();
  bool parseFpABIValue(FgpuABIFlagsSection::FpABIKind &FpABI,
                       StringRef Directive);

  bool parseInternalDirectiveReallowModule();

  bool eatComma(StringRef ErrorStr);

  int matchCPURegisterName(StringRef Symbol);

  int matchVFPRegisterName(StringRef Name);

  unsigned getReg(int RC, int RegNo);

  /// Returns the internal register number for the current AT. Also checks if
  /// the current AT is unavailable (set to $0) and gives an error if it is.
  /// This should be used in pseudo-instruction expansions which need AT.
  unsigned getATReg(SMLoc Loc);

  bool canUseATReg();

  bool processInstruction(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                          const MCSubtargetInfo *STI);

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


    CurrentFn = nullptr;

    IsPicEnabled = getContext().getObjectFileInfo()->isPositionIndependent();

    IsCpRestoreSet = false;
    CpRestoreOffset = -1;
    GPReg = ABI.GetGlobalPtr();

    const Triple &TheTriple = sti.getTargetTriple();
    IsLittleEndian = TheTriple.isLittleEndian();
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
      return false;
    return true;
  }

  const FgpuABIInfo &getABI() const { return ABI; }
  bool isABI_N32() const { return false; }
  bool isABI_N64() const { return false; }
  bool isABI_O32() const { return true; }
//  bool isABI_FPXX() const {
//    return getSTI().getFeatureBits()[Fgpu::FeatureFPXX];
//  }

  bool inPicMode() {
    return IsPicEnabled;
  }

//  bool hasGINV() const {
//    return getSTI().getFeatureBits()[Fgpu::FeatureGINV];
//  }

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
    RegKind_VFP = 2,      /// Vector registers
    /// Potentially any (e.g. $1)
    RegKind_Numeric = RegKind_GPR | RegKind_VFP
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
  unsigned getGPRReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_GPR) && "Invalid access!");
    AsmParser.warnIfRegIndexIsAT(RegIdx.Index, StartLoc);
    unsigned ClassID = Fgpu::GPROutRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

private:
  /// Coerce the register to FGR32 and return the real register for the current
  /// target.
  unsigned getVFPReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_VFP) && "Invalid access!");
    return RegIdx.RegInfo->getRegClass(Fgpu::VecRegsRegClassID)
        .getRegister(RegIdx.Index);
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
    Inst.addOperand(MCOperand::createReg(getGPRReg()));
  }

  void addGPR32NonZeroAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPRReg()));
  }

  void addGPR32AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPRReg()));
  }

  void addVFPAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getVFPReg()));
  }

  void addStrictlyVFPAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getVFPReg()));
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

    Inst.addOperand(MCOperand::createReg(getMemBase()->getGPRReg()));

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
    const unsigned PtrBits = 32;
    if (isa<MCTargetExpr>(getMemOff()) ||
        (isConstantMemOff() && isIntN(PtrBits, getConstantMemOff())))
      return true;
    MCValue Res;
    bool IsReloc = getMemOff()->evaluateAsRelocatable(Res, nullptr, nullptr);
    return IsReloc && isIntN(PtrBits, Res.getConstant());
  }

  template <unsigned Bits> bool isMemWithUimmOffsetSP() const {
    return isMem() && isConstantMemOff() && isUInt<Bits>(getConstantMemOff())
      && getMemBase()->isRegIdx() && (getMemBase()->getGPRReg() == Fgpu::SP);
  }

  template <unsigned Bits> bool isMemWithUimmWordAlignedOffsetSP() const {
    return isMem() && isConstantMemOff() && isUInt<Bits>(getConstantMemOff())
      && (getConstantMemOff() % 4 == 0) && getMemBase()->isRegIdx()
      && (getMemBase()->getGPRReg() == Fgpu::SP);
  }

  template <unsigned Bits> bool isMemWithSimmWordAlignedOffsetGP() const {
    return isMem() && isConstantMemOff() && isInt<Bits>(getConstantMemOff())
      && (getConstantMemOff() % 4 == 0) && getMemBase()->isRegIdx()
      && (getMemBase()->getGPRReg() == Fgpu::R29);
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
      return getGPRReg(); // FIXME: GPR64 too

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

  /// Create a register that is definitely a VFP.
  /// This is typically only used for named registers such as $f0.
  static std::unique_ptr<FgpuOperand>
  createVFPReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
               SMLoc S, SMLoc E, FgpuAsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_VFP, RegInfo, S, E, Parser);
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

  bool isVFPAsmReg() const {
    // AFGR64 is $0-$15 but we handle this in getAFGR64()
    return isRegIdx() && RegIdx.Kind & RegKind_VFP && RegIdx.Index <= 31;
  }

  bool isStrictlyVFPAsmReg() const {
    // AFGR64 is $0-$15 but we handle this in getAFGR64()
    return isRegIdx() && RegIdx.Kind == RegKind_VFP && RegIdx.Index <= 31;
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
    case Fgpu::BEQ:
    case Fgpu::BNE:
      assert(MCID.getNumOperands() == 3 && "unexpected number of operands");
      Offset = Inst.getOperand(2);
      if (!Offset.isImm())
        break; // We'll deal with this situation later on when applying fixups.
      if (!isIntN(18, Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (offsetToAlignment(Offset.getImm(),
                            (Align(4))))
        return Error(IDLoc, "branch to misaligned address");
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


  // This expansion is not in a function called by tryExpandInstruction()
  // because the pseudo-instruction doesn't have a distinct opcode.
  if ((Opcode == Fgpu::JSUB) && inPicMode()) {
    warnIfNoMacro(IDLoc);

    const MCExpr *JalExpr = Inst.getOperand(0).getExpr();

    // We can do this expansion if there's only 1 symbol in the argument
    // expression.
    if (countMCSymbolRefExpr(JalExpr) > 1)
      return Error(IDLoc, "jal doesn't support multiple symbols in PIC mode");

    // FIXME: This is checking the expression can be handled by the later stages
    //        of the assembler. We ought to leave it to those later stages.
    const MCSymbol *JalSym = getSingleMCSymbol(JalExpr);

    if (expandLoadAddress(Fgpu::R25, Fgpu::NoRegister, Inst.getOperand(0),
                          true, IDLoc, Out, STI))
      return true;

    MCInst JalrInst;
    JalrInst.setOpcode(Fgpu::JSUB); // TODO add JALR
    JalrInst.addOperand(MCOperand::createReg(Fgpu::LR));
    JalrInst.addOperand(MCOperand::createReg(Fgpu::R25));

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
          *TmpExpr, "R_FGPU_JALR",
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

  return false;
}

FgpuAsmParser::MacroExpanderResultTy
FgpuAsmParser::tryExpandInstruction(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                    const MCSubtargetInfo *STI) {
  switch (Inst.getOpcode()) {
  default:
    return MER_NotAMacro;
  case Fgpu::Li32:
    return expandLoadImm(Inst, true, IDLoc, Out, STI) ? MER_Fail : MER_Success;
//  case Fgpu::LoadImm64:
//    return expandLoadImm(Inst, false, IDLoc, Out, STI) ? MER_Fail : MER_Success;
//  case Fgpu::LoadAddrImm32:
//  case Fgpu::LoadAddrImm64:
//    assert(Inst.getOperand(0).isReg() && "expected register operand kind");
//    assert((Inst.getOperand(1).isImm() || Inst.getOperand(1).isExpr()) &&
//           "expected immediate operand kind");
//
//    return expandLoadAddress(Inst.getOperand(0).getReg(), Fgpu::NoRegister,
//                             Inst.getOperand(1),
//                             Inst.getOpcode() == Fgpu::LoadAddrImm32, IDLoc,
//                             Out, STI)
//               ? MER_Fail
//               : MER_Success;
//  case Fgpu::LoadAddrReg32:
//  case Fgpu::LoadAddrReg64:
//    assert(Inst.getOperand(0).isReg() && "expected register operand kind");
//    assert(Inst.getOperand(1).isReg() && "expected register operand kind");
//    assert((Inst.getOperand(2).isImm() || Inst.getOperand(2).isExpr()) &&
//           "expected immediate operand kind");
//
//    return expandLoadAddress(Inst.getOperand(0).getReg(),
//                             Inst.getOperand(1).getReg(), Inst.getOperand(2),
//                             Inst.getOpcode() == Fgpu::LoadAddrReg32, IDLoc,
//                             Out, STI)
//               ? MER_Fail
//               : MER_Success;
//  case Fgpu::JalOneReg:
//  case Fgpu::JalTwoReg:
//    return expandJalWithRegs(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
//  case Fgpu::BLT:
//  case Fgpu::BLE:
//  case Fgpu::BGE:
//  case Fgpu::BGT:
//  case Fgpu::BLTU:
//  case Fgpu::BLEU:
//  case Fgpu::BGEU:
//  case Fgpu::BGTU:
//  case Fgpu::BLTL:
//  case Fgpu::BLEL:
//  case Fgpu::BGEL:
//  case Fgpu::BGTL:
//  case Fgpu::BLTUL:
//  case Fgpu::BLEUL:
//  case Fgpu::BGEUL:
//  case Fgpu::BGTUL:
//  case Fgpu::BLTImmMacro:
//  case Fgpu::BLEImmMacro:
//  case Fgpu::BGEImmMacro:
//  case Fgpu::BGTImmMacro:
//  case Fgpu::BLTUImmMacro:
//  case Fgpu::BLEUImmMacro:
//  case Fgpu::BGEUImmMacro:
//  case Fgpu::BGTUImmMacro:
//  case Fgpu::BLTLImmMacro:
//  case Fgpu::BLELImmMacro:
//  case Fgpu::BGELImmMacro:
//  case Fgpu::BGTLImmMacro:
//  case Fgpu::BLTULImmMacro:
//  case Fgpu::BLEULImmMacro:
//  case Fgpu::BGEULImmMacro:
//  case Fgpu::BGTULImmMacro:
//    return expandCondBranches(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
//  case Fgpu::LoadImmSingleGPR:
//    return expandLoadSingleImmToGPR(Inst, IDLoc, Out, STI) ? MER_Fail
//                                                           : MER_Success;
//  case Fgpu::LoadImmSingleFGR:
//    return expandLoadSingleImmToFPR(Inst, IDLoc, Out, STI) ? MER_Fail
//                                                           : MER_Success;
//  case Fgpu::LoadImmDoubleGPR:
//    return expandLoadDoubleImmToGPR(Inst, IDLoc, Out, STI) ? MER_Fail
//                                                           : MER_Success;
//  case Fgpu::LoadImmDoubleFGR:
//    return expandLoadDoubleImmToFPR(Inst, true, IDLoc, Out, STI) ? MER_Fail
//                                                                 : MER_Success;
//  case Fgpu::LoadImmDoubleFGR_32:
//    return expandLoadDoubleImmToFPR(Inst, false, IDLoc, Out, STI) ? MER_Fail
//                                                                  : MER_Success;
//  case Fgpu::NORImm:
//  case Fgpu::NORImm64:
//    return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
//  case Fgpu::SGE:
//  case Fgpu::SGEU:
//    return expandSge(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
//  case Fgpu::SGEImm:
//  case Fgpu::SGEUImm:
//  case Fgpu::SGEImm64:
//  case Fgpu::SGEUImm64:
//    return expandSgeImm(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
//  case Fgpu::SGTImm:
//  case Fgpu::SGTUImm:
//  case Fgpu::SGTImm64:
//  case Fgpu::SGTUImm64:
//    return expandSgtImm(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
//  case Fgpu::SLE:
//  case Fgpu::SLEU:
//    return expandSle(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
//  case Fgpu::SLEImm:
//  case Fgpu::SLEUImm:
//  case Fgpu::SLEImm64:
//  case Fgpu::SLEUImm64:
//    return expandSleImm(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
//  case Fgpu::SLTImm64:
//    if (isInt<16>(Inst.getOperand(2).getImm())) {
//      Inst.setOpcode(Fgpu::SLTi64);
//      return MER_NotAMacro;
//    }
//    return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
//  case Fgpu::SLTUImm64:
//    if (isInt<16>(Inst.getOperand(2).getImm())) {
//      Inst.setOpcode(Fgpu::SLTiu64);
//      return MER_NotAMacro;
//    }
//    return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case Fgpu::ADDi:
  case Fgpu::SLTi:
  case Fgpu::SLTiu:
    if ((Inst.getNumOperands() == 3) && Inst.getOperand(0).isReg() &&
        Inst.getOperand(1).isReg() && Inst.getOperand(2).isImm()) {
      int64_t ImmValue = Inst.getOperand(2).getImm();
      if (isInt<16>(ImmValue))
        return MER_NotAMacro;
      return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail
                                                         : MER_Success;
    }
    return MER_NotAMacro;
  case Fgpu::ANDi:
  case Fgpu::ORi:
  case Fgpu::XORi:
    if ((Inst.getNumOperands() == 3) && Inst.getOperand(0).isReg() &&
        Inst.getOperand(1).isReg() && Inst.getOperand(2).isImm()) {
      int64_t ImmValue = Inst.getOperand(2).getImm();
      if (isUInt<16>(ImmValue))
        return MER_NotAMacro;
      return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail
                                                         : MER_Success;
    }
    return MER_NotAMacro;
//  case Fgpu::ABSMacro:
//    return expandAbs(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  }
}
//
//bool FgpuAsmParser::expandJalWithRegs(MCInst &Inst, SMLoc IDLoc,
//                                      MCStreamer &Out,
//                                      const MCSubtargetInfo *STI) {
//  FgpuTargetStreamer &TOut = getTargetStreamer();
//
//  // Create a JALR instruction which is going to replace the pseudo-JAL.
//  MCInst JalrInst;
//  JalrInst.setLoc(IDLoc);
//  const MCOperand FirstRegOp = Inst.getOperand(0);
//  const unsigned Opcode = Inst.getOpcode();
//
//  if (Opcode == Fgpu::JalOneReg) {
//    // jal $rs => jalr $rs
//    JalrInst.setOpcode(Fgpu::JALR);
//    JalrInst.addOperand(MCOperand::createReg(Fgpu::LR));
//    JalrInst.addOperand(FirstRegOp);
//  } else if (Opcode == Fgpu::JalTwoReg) {
//    // jal $rd, $rs => jalr $rd, $rs
//    JalrInst.setOpcode(Fgpu::JALR);
//    JalrInst.addOperand(FirstRegOp);
//    const MCOperand SecondRegOp = Inst.getOperand(1);
//    JalrInst.addOperand(SecondRegOp);
//  }
//  Out.emitInstruction(JalrInst, *STI);
//
//  // If .set reorder is active and branch instruction has a delay slot,
//  // emit a NOP after it.
//  const MCInstrDesc &MCID = getInstDesc(JalrInst.getOpcode());
//  if (MCID.hasDelaySlot() && AssemblerOptions.back()->isReorder())
//    TOut.emitEmptyDelaySlot(false, IDLoc,
//                            STI);
//
//  return false;
//}

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

  if (!Is32BitImm) {
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
  unsigned AdduOp = Fgpu::ADD;

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

  // TODO: these won't work b/c of how LUi/Li are implemented
//  if (isInt<16>(ImmValue)) {
//    if (!UseSrcReg)
//      SrcReg = ZeroReg;
//    //TODO: this won't work because Li doesn't sign-extennd i think
//    TOut.emitRRI(Fgpu::Li, DstReg, SrcReg, ImmValue, IDLoc, STI);
//    return false;
//  }
//
//  if (isUInt<16>(ImmValue)) {
//    unsigned TmpReg = DstReg;
//    if (SrcReg == DstReg) {
//      TmpReg = getATReg(IDLoc);
//      if (!TmpReg)
//        return true;
//    }
//
//    TOut.emitRI(Fgpu::Li, TmpReg, ImmValue, IDLoc, STI); // TODO: should be Li??
//    if (UseSrcReg)
//      TOut.emitRRR(ABI.GetPtrAdduOp(), DstReg, TmpReg, SrcReg, IDLoc, STI);
//    return false;
//  }

  if (isInt<32>(ImmValue) || isUInt<32>(ImmValue)) {
    warnIfNoMacro(IDLoc);

    uint16_t Bits31To16 = (ImmValue >> 16) & 0xffff;
    uint16_t Bits15To0 = ImmValue & 0xffff;

    TOut.emitRI(Fgpu::LUi, TmpReg, Bits31To16, IDLoc, STI);
    if (Bits15To0)
      TOut.emitRI(Fgpu::Li, TmpReg, Bits15To0, IDLoc, STI);
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
    TOut.emitRI(Fgpu::Li, TmpReg, Bits, IDLoc, STI);
    TOut.emitRRI(Fgpu::SLLi, TmpReg, TmpReg, ShiftAmount, IDLoc, STI);

    if (UseSrcReg)
      TOut.emitRRR(AdduOp, DstReg, TmpReg, SrcReg, IDLoc, STI);

    return false;
  }

  warnIfNoMacro(IDLoc);
  assert(false && "Not supported size of immediate");

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
      TOut.emitRRI(Fgpu::SLLi,TmpReg, TmpReg, ShiftCarriedForwards, IDLoc, STI);
      TOut.emitRRI(Fgpu::ORi, TmpReg, TmpReg, ImmChunk, IDLoc, STI);
      ShiftCarriedForwards = 0;
    }

    ShiftCarriedForwards += 16;
  }
  ShiftCarriedForwards -= 16;

  // Finish any remaining shifts left by trailing zeros.
  if (ShiftCarriedForwards)
    TOut.emitRRI(Fgpu::SLLi, TmpReg, TmpReg, ShiftCarriedForwards, IDLoc, STI);

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

  if (!Offset.isImm())
    return loadAndAddSymbolAddress(Offset.getExpr(), DstReg, BaseReg,
                                   Is32BitAddress, IDLoc, Out, STI);

  return loadImmediate(Offset.getImm(), DstReg, BaseReg, Is32BitAddress, true,
                       IDLoc, Out, STI);
}

bool FgpuAsmParser::loadAndAddSymbolAddress(const MCExpr *SymExpr,
                                            unsigned DstReg, unsigned SrcReg,
                                            bool Is32BitSym, SMLoc IDLoc,
                                            MCStreamer &Out,
                                            const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  bool UseSrcReg = SrcReg != Fgpu::NoRegister && SrcReg != Fgpu::ZERO;
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

    bool IsPtr64 = false;
    bool IsLocalSym =
        Res.getSymA()->getSymbol().isInSection() ||
        Res.getSymA()->getSymbol().isTemporary() ||
        (Res.getSymA()->getSymbol().isELF() &&
         cast<MCSymbolELF>(Res.getSymA()->getSymbol()).getBinding() ==
             ELF::STB_LOCAL);
    bool UseXGOT = false && !IsLocalSym;

    // The case where the result register is $25 is somewhat special. If the
    // symbol in the final relocation is external and not modified with a
    // constant then we must use R_FGPU_CALL16 instead of R_FGPU_GOT16
    // or R_FGPU_CALL16 instead of R_FGPU_GOT_DISP in 64-bit case.
    if ((DstReg == Fgpu::R25) && !UseSrcReg &&
        Res.getConstant() == 0 && !IsLocalSym) {
      {
        const MCExpr *CallExpr =
            FgpuMCExpr::create(FgpuMCExpr::MEK_GOT_CALL, SymExpr, getContext());
        TOut.emitRRX(Fgpu::LW, DstReg, GPReg,
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


    const FgpuMCExpr *GotExpr = nullptr;
    const MCExpr *LoExpr = nullptr;
    {
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

    TOut.emitRRX(Fgpu::LW, TmpReg, GPReg,
                 MCOperand::createExpr(GotExpr), IDLoc, STI);

    if (LoExpr)
      TOut.emitRRX(Fgpu::ADDi, TmpReg, TmpReg, // TODO: should be Li??
                   MCOperand::createExpr(LoExpr), IDLoc, STI);

    if (UseSrcReg)
      TOut.emitRRR(Fgpu::ADD, DstReg, TmpReg, SrcReg,
                   IDLoc, STI);

    return false;
  }

  const FgpuMCExpr *HiExpr =
      FgpuMCExpr::create(FgpuMCExpr::MEK_HI, SymExpr, getContext());
  const FgpuMCExpr *LoExpr =
      FgpuMCExpr::create(FgpuMCExpr::MEK_LO, SymExpr, getContext());

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
  TOut.emitRX(Fgpu::Li, TmpReg, MCOperand::createExpr(LoExpr), // TODO: should be Li??
               IDLoc, STI);

  if (UseSrcReg)
    TOut.emitRRR(Fgpu::ADD, DstReg, TmpReg, SrcReg, IDLoc, STI);
  else
    assert(
        getContext().getRegisterInfo()->isSuperOrSubRegisterEq(DstReg, TmpReg));

  return false;
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

      TOut.emitRRX(Fgpu::LW, ATReg, GPReg, MCOperand::createExpr(GotExpr),
                   IDLoc, STI);
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
    assert(false && "I have no confience in this working"); // not going to bother to TODO this
      TOut.emitRX(Fgpu::LUi, ATReg, MCOperand::createExpr(HiExpr), IDLoc, STI);
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

    if (loadImmediate(ImmValue, ATReg, Fgpu::NoRegister, true, true,
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
  bool IsGPR = (DstRegClassID == Fgpu::GPROutRegClassID);

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

    if (BaseReg != Fgpu::ZERO)
      TOut.emitRRR(Fgpu::ADD, TmpReg,
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
                              true, IDLoc, Out, STI);
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
      {
        // Generate the base address in TmpReg.
        TOut.emitRX(Fgpu::LUi, TmpReg, HiOperand, IDLoc, STI);
        if (BaseReg != Fgpu::ZERO)
          TOut.emitRRR(Fgpu::ADD, TmpReg, TmpReg, BaseReg, IDLoc, STI);
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
  bool IsGPR = (DstRegClassID == Fgpu::GPROutRegClassID);

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
    loadImmediate(OffsetOp.getImm(), TmpReg, BaseReg, true, true,
                  IDLoc, Out, STI);
    emitInst();
    return;
  }

  if (OffsetOp.isExpr()) {
    loadAndAddSymbolAddress(OffsetOp.getExpr(), TmpReg, BaseReg,
                            true, IDLoc, Out, STI);
    emitInst();
    return;
  }

  llvm_unreachable("unexpected operand type");
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
//    case Fgpu::BLTImmMacro:
//      PseudoOpcode = Fgpu::BLT;
//      break;
    }

    if (loadImmediate(TrgOp.getImm(), TrgReg, Fgpu::NoRegister, true,
                      false, IDLoc, Out, STI))
      return true;
  }

  switch (PseudoOpcode) {
//  case Fgpu::BLT:
//  case Fgpu::BLTU:
//  case Fgpu::BLTL:
//  case Fgpu::BLTUL:
//    AcceptsEquality = false;
//    ReverseOrderSLT = false;
//    IsUnsigned =
//        ((PseudoOpcode == Fgpu::BLTU) || (PseudoOpcode == Fgpu::BLTUL));
//    IsLikely = ((PseudoOpcode == Fgpu::BLTL) || (PseudoOpcode == Fgpu::BLTUL));
//    ZeroSrcOpcode = Fgpu::BGTZ;
//    ZeroTrgOpcode = Fgpu::BLTZ;
//    break;
//  case Fgpu::BLE:
//  case Fgpu::BLEU:
//  case Fgpu::BLEL:
//  case Fgpu::BLEUL:
//    AcceptsEquality = true;
//    ReverseOrderSLT = true;
//    IsUnsigned =
//        ((PseudoOpcode == Fgpu::BLEU) || (PseudoOpcode == Fgpu::BLEUL));
//    IsLikely = ((PseudoOpcode == Fgpu::BLEL) || (PseudoOpcode == Fgpu::BLEUL));
//    ZeroSrcOpcode = Fgpu::BGEZ;
//    ZeroTrgOpcode = Fgpu::BLEZ;
//    break;
//  case Fgpu::BGE:
//  case Fgpu::BGEU:
//  case Fgpu::BGEL:
//  case Fgpu::BGEUL:
//    AcceptsEquality = true;
//    ReverseOrderSLT = false;
//    IsUnsigned =
//        ((PseudoOpcode == Fgpu::BGEU) || (PseudoOpcode == Fgpu::BGEUL));
//    IsLikely = ((PseudoOpcode == Fgpu::BGEL) || (PseudoOpcode == Fgpu::BGEUL));
//    ZeroSrcOpcode = Fgpu::BLEZ;
//    ZeroTrgOpcode = Fgpu::BGEZ;
//    break;
//  case Fgpu::BGT:
//  case Fgpu::BGTU:
//  case Fgpu::BGTL:
//  case Fgpu::BGTUL:
//    AcceptsEquality = false;
//    ReverseOrderSLT = true;
//    IsUnsigned =
//        ((PseudoOpcode == Fgpu::BGTU) || (PseudoOpcode == Fgpu::BGTUL));
//    IsLikely = ((PseudoOpcode == Fgpu::BGTL) || (PseudoOpcode == Fgpu::BGTUL));
//    ZeroSrcOpcode = Fgpu::BLTZ;
//    ZeroTrgOpcode = Fgpu::BGTZ;
//    break;
  default:
    llvm_unreachable("unknown opcode for branch pseudo-instruction");
  }

//  bool IsTrgRegZero = (TrgReg == Fgpu::ZERO);
//  bool IsSrcRegZero = (SrcReg == Fgpu::ZERO);
//  if (IsSrcRegZero && IsTrgRegZero) {
//    // FIXME: All of these Opcode-specific if's are needed for compatibility
//    // with GAS' behaviour. However, they may not generate the most efficient
//    // code in some circumstances.
//    if (PseudoOpcode == Fgpu::BLT) {
//      TOut.emitRX(Fgpu::BLTZ, Fgpu::ZERO, MCOperand::createExpr(OffsetExpr),
//                  IDLoc, STI);
//      return false;
//    }
//    if (PseudoOpcode == Fgpu::BLE) {
//      TOut.emitRX(Fgpu::BLEZ, Fgpu::ZERO, MCOperand::createExpr(OffsetExpr),
//                  IDLoc, STI);
//      Warning(IDLoc, "branch is always taken");
//      return false;
//    }
//    if (PseudoOpcode == Fgpu::BGE) {
//      TOut.emitRX(Fgpu::BGEZ, Fgpu::ZERO, MCOperand::createExpr(OffsetExpr),
//                  IDLoc, STI);
//      Warning(IDLoc, "branch is always taken");
//      return false;
//    }
//    if (PseudoOpcode == Fgpu::BGT) {
//      TOut.emitRX(Fgpu::BGTZ, Fgpu::ZERO, MCOperand::createExpr(OffsetExpr),
//                  IDLoc, STI);
//      return false;
//    }
//    if (PseudoOpcode == Fgpu::BGTU) {
//      TOut.emitRRX(Fgpu::BNE, Fgpu::ZERO, Fgpu::ZERO,
//                   MCOperand::createExpr(OffsetExpr), IDLoc, STI);
//      return false;
//    }
//    if (AcceptsEquality) {
//      // If both registers are $0 and the pseudo-branch accepts equality, it
//      // will always be taken, so we emit an unconditional branch.
//      TOut.emitRRX(Fgpu::BEQ, Fgpu::ZERO, Fgpu::ZERO,
//                   MCOperand::createExpr(OffsetExpr), IDLoc, STI);
//      Warning(IDLoc, "branch is always taken");
//      return false;
//    }
//    // If both registers are $0 and the pseudo-branch does not accept
//    // equality, it will never be taken, so we don't have to emit anything.
//    return false;
//  }
//  if (IsSrcRegZero || IsTrgRegZero) {
//    if ((IsSrcRegZero && PseudoOpcode == Fgpu::BGTU) ||
//        (IsTrgRegZero && PseudoOpcode == Fgpu::BLTU)) {
//      // If the $rs is $0 and the pseudo-branch is BGTU (0 > x) or
//      // if the $rt is $0 and the pseudo-branch is BLTU (x < 0),
//      // the pseudo-branch will never be taken, so we don't emit anything.
//      // This only applies to unsigned pseudo-branches.
//      return false;
//    }
//    if ((IsSrcRegZero && PseudoOpcode == Fgpu::BLEU) ||
//        (IsTrgRegZero && PseudoOpcode == Fgpu::BGEU)) {
//      // If the $rs is $0 and the pseudo-branch is BLEU (0 <= x) or
//      // if the $rt is $0 and the pseudo-branch is BGEU (x >= 0),
//      // the pseudo-branch will always be taken, so we emit an unconditional
//      // branch.
//      // This only applies to unsigned pseudo-branches.
//      TOut.emitRRX(Fgpu::BEQ, Fgpu::ZERO, Fgpu::ZERO,
//                   MCOperand::createExpr(OffsetExpr), IDLoc, STI);
//      Warning(IDLoc, "branch is always taken");
//      return false;
//    }
//    if (IsUnsigned) {
//      // If the $rs is $0 and the pseudo-branch is BLTU (0 < x) or
//      // if the $rt is $0 and the pseudo-branch is BGTU (x > 0),
//      // the pseudo-branch will be taken only when the non-zero register is
//      // different from 0, so we emit a BNEZ.
//      //
//      // If the $rs is $0 and the pseudo-branch is BGEU (0 >= x) or
//      // if the $rt is $0 and the pseudo-branch is BLEU (x <= 0),
//      // the pseudo-branch will be taken only when the non-zero register is
//      // equal to 0, so we emit a BEQZ.
//      //
//      // Because only BLEU and BGEU branch on equality, we can use the
//      // AcceptsEquality variable to decide when to emit the BEQZ.
//      TOut.emitRRX(AcceptsEquality ? Fgpu::BEQ : Fgpu::BNE,
//                   IsSrcRegZero ? TrgReg : SrcReg, Fgpu::ZERO,
//                   MCOperand::createExpr(OffsetExpr), IDLoc, STI);
//      return false;
//    }
//    // If we have a signed pseudo-branch and one of the registers is $0,
//    // we can use an appropriate compare-to-zero branch. We select which one
//    // to use in the switch statement above.
//    TOut.emitRX(IsSrcRegZero ? ZeroSrcOpcode : ZeroTrgOpcode,
//                IsSrcRegZero ? TrgReg : SrcReg,
//                MCOperand::createExpr(OffsetExpr), IDLoc, STI);
//    return false;
//  }

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

  TOut.emitRRX((AcceptsEquality ? Fgpu::BEQ : Fgpu::BNE),
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
  return true;
}

bool FgpuAsmParser::expandTrunc(MCInst &Inst, bool IsDouble, bool Is64FPU,
                                SMLoc IDLoc, MCStreamer &Out,
                                const MCSubtargetInfo *STI) {
  return true;
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
//  case Fgpu::SGE:
//    OpCode = Fgpu::SLT;
//    break;
//  case Fgpu::SGEU:
//    OpCode = Fgpu::SLTu;
//    break;
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
//  case Fgpu::SGEImm:
//    OpRegCode = Fgpu::SLT;
//    OpImmCode = Fgpu::SLTi;
//    break;
//  case Fgpu::SGEUImm:
//    OpRegCode = Fgpu::SLTu;
//    OpImmCode = Fgpu::SLTiu;
//    break;
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
//  case Fgpu::SGTImm:
//  case Fgpu::SGTImm64:
//    OpCode = Fgpu::SLT;
//    break;
//  case Fgpu::SGTUImm:
//  case Fgpu::SGTUImm64:
//    OpCode = Fgpu::SLTu;
//    break;
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
//  case Fgpu::SLE:
//    OpCode = Fgpu::SLT;
//    break;
//  case Fgpu::SLEU:
//    OpCode = Fgpu::SLTu;
//    break;
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
//  case Fgpu::SLEImm:
//    OpRegCode = Fgpu::SLT;
//    break;
//  case Fgpu::SLEUImm:
//    OpRegCode = Fgpu::SLTu;
//    break;
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

  bool Is32Bit = isInt<32>(ImmValue) || (isUInt<32>(ImmValue));

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
    case Fgpu::ANDi:
      FinalOpcode = Fgpu::AND;
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

//  if (hasFgpu32()) {
//    switch (Inst.getOpcode()) {
//    default:
//      llvm_unreachable("unexpected instruction opcode");
//    case Fgpu::ROL:
//      FirstShift = Fgpu::SRLV;
//      SecondShift = Fgpu::SLLV;
//      break;
//    case Fgpu::ROR:
//      FirstShift = Fgpu::SLLV;
//      SecondShift = Fgpu::SRLV;
//      break;
//    }
//
//    ATReg = getATReg(Inst.getLoc());
//    if (!ATReg)
//      return true;
//
//    TOut.emitRRR(Fgpu::SUBu, ATReg, Fgpu::ZERO, TReg, Inst.getLoc(), STI);
//    TOut.emitRRR(FirstShift, ATReg, SReg, ATReg, Inst.getLoc(), STI);
//    TOut.emitRRR(SecondShift, DReg, SReg, TReg, Inst.getLoc(), STI);
//    TOut.emitRRR(Fgpu::OR, DReg, DReg, ATReg, Inst.getLoc(), STI);
//
//    return false;
//  }

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

//  if (hasFgpu32()) {
//    if (ImmValue == 0) {
//      TOut.emitRRI(Fgpu::SRL, DReg, SReg, 0, Inst.getLoc(), STI);
//      return false;
//    }
//
//    switch (Inst.getOpcode()) {
//    default:
//      llvm_unreachable("unexpected instruction opcode");
//    case Fgpu::ROLImm:
//      FirstShift = Fgpu::SLL;
//      SecondShift = Fgpu::SRL;
//      break;
//    case Fgpu::RORImm:
//      FirstShift = Fgpu::SRL;
//      SecondShift = Fgpu::SLL;
//      break;
//    }
//
//    ATReg = getATReg(Inst.getLoc());
//    if (!ATReg)
//      return true;
//
//    TOut.emitRRI(FirstShift, ATReg, SReg, ImmValue, Inst.getLoc(), STI);
//    TOut.emitRRI(SecondShift, DReg, SReg, 32 - ImmValue, Inst.getLoc(), STI);
//    TOut.emitRRR(Fgpu::OR, DReg, DReg, ATReg, Inst.getLoc(), STI);
//
//    return false;
//  }

  return true;
}

bool FgpuAsmParser::expandAbs(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                              const MCSubtargetInfo *STI) {
  FgpuTargetStreamer &TOut = getTargetStreamer();
  unsigned FirstRegOp = Inst.getOperand(0).getReg();
  unsigned SecondRegOp = Inst.getOperand(1).getReg();

  assert(false && "Not workign rn");
//  TOut.emitRI(Fgpu::BGEZ, SecondRegOp, 8, IDLoc, STI);
  if (FirstRegOp != SecondRegOp)
    TOut.emitRRR(Fgpu::ADD, FirstRegOp, SecondRegOp, Fgpu::ZERO, IDLoc, STI);
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

  TOut.emitRRR(Fgpu::MUL,
              DstReg, SrcReg, ATReg, IDLoc, STI);

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
    TOut.emitRRR(Fgpu::ADD,
                 DstReg, SrcReg, SrcReg, IDLoc, STI);
    return false;
  }

  unsigned Opc;
  if (Imm > -0x8000 && Imm < 0) {
    Imm = -Imm;
    Opc = Fgpu::ADDi;
  } else {
    Opc = Fgpu::XORi;
  }

  if (!isUInt<16>(Imm)) {
    unsigned ATReg = getATReg(IDLoc);
    if (!ATReg)
      return true;

    if (loadImmediate(Imm, ATReg, Fgpu::NoRegister, true, false, IDLoc,
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
    Opc = Fgpu::ADDi;
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

unsigned FgpuAsmParser::checkTargetMatchPredicate(MCInst &Inst) {
  switch (Inst.getOpcode()) {
  case Fgpu::SET_SYNC:
    if (Inst.getOperand(0).getImm() != 0)
      return Match_NonZeroOperandForSync;
    return Match_Success;
  }

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
           .Cases("zero", "r0", 0)
           .Cases("at", "AT", "r1", 1)
           .Cases("a0", "r4", 4)
           .Cases("a1", "r5", 5)
           .Cases("a2", "r6", 6)
           .Cases("a3", "r7", 7)
           .Cases("v0", "r2", 2)
           .Cases("v1", "r3", 3)
           .Cases("s0", "r16", 16)
           .Cases("s1", "r17", 17)
           .Cases("s2", "r18", 18)
           .Cases("s3", "r19", 19)
           .Cases("s4", "r20", 20)
           .Cases("s5", "r21", 21)
           .Cases("s6", "r22", 22)
           .Cases("s7", "r23", 23)
           .Cases("k0", "r26", 26)
           .Cases("bp", "r27", 27)
           .Cases("fp", "r28", 28)
           .Cases("gp", "r29", 29)
           .Cases("ra", "lr", "r30", 30)
           .Cases("sp", "r31", 31)
           .Cases("t0", "r8", 8)
           .Cases("t1", "r9", 9)
           .Cases("t2", "r10", 10)
           .Cases("t3", "r11", 11)
           .Cases("t4", "r12", 12)
           .Cases("t5", "r13", 13)
           .Cases("t6", "r14", 14)
           .Cases("t7", "r15", 15)
           .Cases("t8", "r16", 24)
           .Cases("t9", "r25", 25)
           .Default(-1);

    return CC;
}

int FgpuAsmParser::matchVFPRegisterName(StringRef Name) {
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
  unsigned AT = getReg(Fgpu::GPROutRegClassID, ATIndex);
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
  OperandMatchResultTy ResTy = llvm::MatchOperand_NoMatch;
  // MatchOperandParserImpl(Operands, Mnemonic);
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
      RegNo = Operand.getGPRReg();
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

  Index = matchVFPRegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(FgpuOperand::createVFPReg(
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
  //TODO: I don't think should be called""
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
    RegNo =  Reg.getGPRReg();
    if (RegRange) {
      // Remove last register operand because registers from register range
      // should be inserted first.
      if (
          (RegNo == Fgpu::LR)) {
        Regs.push_back(RegNo);
      } else {
        unsigned TmpReg = PrevReg + 1;
        while (TmpReg <= RegNo) {

          PrevReg = TmpReg;
          Regs.push_back(TmpReg++);
        }
      }

      RegRange = false;
    } else {

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

//bool FgpuAsmParser::parseSetNoGINVDirective() {
//  MCAsmParser &Parser = getParser();
//  Parser.Lex(); // Eat "noginv".
//
//  // If this is not the end of the statement, report an error.
//  if (getLexer().isNot(AsmToken::EndOfStatement)) {
//    reportParseError("unexpected token, expected end of statement");
//    return false;
//  }
//
//  clearFeatureBits(Fgpu::FeatureGINV, "ginv");
//
//  getTargetStreamer().emitDirectiveSetNoGINV();
//  Parser.Lex(); // Consume the EndOfStatement.
//  return false;
//}

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
//  case Fgpu::FeatureGINV:
//    setFeatureBits(Fgpu::FeatureGINV, "ginv");
//    getTargetStreamer().emitDirectiveSetGINV();
//    break;
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
  return inPicMode();
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

  getTargetStreamer().emitDirectiveCpAdd(RegOpnd.getGPRReg());
  return false;
}

bool FgpuAsmParser::parseDirectiveCpLoad(SMLoc Loc) {
  if (AssemblerOptions.back()->isReorder())
    Warning(Loc, ".cpload should be inside a noreorder section");

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

  getTargetStreamer().emitDirectiveCpLoad(RegOpnd.getGPRReg());
  return false;
}

bool FgpuAsmParser::parseDirectiveCpRestore(SMLoc Loc) {
  MCAsmParser &Parser = getParser();

  // Note that .cprestore is ignored if used with the N32 and N64 ABIs or if it
  // is used in non-PIC mode.

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

  FuncReg = FuncRegOpnd.getGPRReg();
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
    Save = SaveOpnd.getGPRReg();
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
//  if (IdVal == "fgpu0")
//    return parseSetFgpu0Directive();
//  if (IdVal == "ginv")
//    return parseSetFeature(Fgpu::FeatureGINV);
//  if (IdVal == "noginv")
//    return parseSetNoGINVDirective();

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
  {
    return Error(L, "'" + Twine(Option) + "' is not a valid .module option.");
  }
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
//  if (IDVal == ".cplocal") {
//    parseDirectiveCpLocal(DirectiveID.getLoc());
//    return false;
//  }
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
    unsigned StackReg = StackRegOpnd.getGPRReg();

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
                                  ReturnRegOpnd.getGPRReg());
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
    assert(false && "non workg");
    //getTargetStreamer().emitDirectiveAbiCalls();
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
