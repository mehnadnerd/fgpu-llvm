//===-- FgpuELFObjectWriter.cpp - Fgpu ELF Writer -------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/FgpuFixupKinds.h"
#include "MCTargetDesc/FgpuMCTargetDesc.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iterator>
#include <list>
#include <utility>

#define DEBUG_TYPE "fgpu-elf-object-writer"

using namespace llvm;

namespace {

/// Holds additional information needed by the relocation ordering algorithm.
struct FgpuRelocationEntry {
  const ELFRelocationEntry R; ///< The relocation.
  bool Matched = false;       ///< Is this relocation part of a match.

  FgpuRelocationEntry(const ELFRelocationEntry &R) : R(R) {}

  void print(raw_ostream &Out) const {
    R.print(Out);
    Out << ", Matched=" << Matched;
  }
};

#ifndef NDEBUG
raw_ostream &operator<<(raw_ostream &OS, const FgpuRelocationEntry &RHS) {
  RHS.print(OS);
  return OS;
}
#endif

class FgpuELFObjectWriter : public MCELFObjectTargetWriter {
public:
  FgpuELFObjectWriter(uint8_t OSABI, bool HasRelocationAddend, bool Is64);

  ~FgpuELFObjectWriter() override = default;

  unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                        const MCFixup &Fixup, bool IsPCRel) const override;
  bool needsRelocateWithSymbol(const MCSymbol &Sym,
                               unsigned Type) const override;
  void sortRelocs(const MCAssembler &Asm,
                  std::vector<ELFRelocationEntry> &Relocs) override;
};

/// The possible results of the Predicate function used by find_best.
enum FindBestPredicateResult {
  FindBest_NoMatch = 0,  ///< The current element is not a match.
  FindBest_Match,        ///< The current element is a match but better ones are
                         ///  possible.
  FindBest_PerfectMatch, ///< The current element is an unbeatable match.
};

} // end anonymous namespace

/// Copy elements in the range [First, Last) to d1 when the predicate is true or
/// d2 when the predicate is false. This is essentially both std::copy_if and
/// std::remove_copy_if combined into a single pass.
template <class InputIt, class OutputIt1, class OutputIt2, class UnaryPredicate>
static std::pair<OutputIt1, OutputIt2> copy_if_else(InputIt First, InputIt Last,
                                                    OutputIt1 d1, OutputIt2 d2,
                                                    UnaryPredicate Predicate) {
  for (InputIt I = First; I != Last; ++I) {
    if (Predicate(*I)) {
      *d1 = *I;
      d1++;
    } else {
      *d2 = *I;
      d2++;
    }
  }

  return std::make_pair(d1, d2);
}

/// Find the best match in the range [First, Last).
///
/// An element matches when Predicate(X) returns FindBest_Match or
/// FindBest_PerfectMatch. A value of FindBest_PerfectMatch also terminates
/// the search. BetterThan(A, B) is a comparator that returns true when A is a
/// better match than B. The return value is the position of the best match.
///
/// This is similar to std::find_if but finds the best of multiple possible
/// matches.
template <class InputIt, class UnaryPredicate, class Comparator>
static InputIt find_best(InputIt First, InputIt Last, UnaryPredicate Predicate,
                         Comparator BetterThan) {
  InputIt Best = Last;

  for (InputIt I = First; I != Last; ++I) {
    unsigned Matched = Predicate(*I);
    if (Matched != FindBest_NoMatch) {
      LLVM_DEBUG(dbgs() << std::distance(First, I) << " is a match (";
                 I->print(dbgs()); dbgs() << ")\n");
      if (Best == Last || BetterThan(*I, *Best)) {
        LLVM_DEBUG(dbgs() << ".. and it beats the last one\n");
        Best = I;
      }
    }
    if (Matched == FindBest_PerfectMatch) {
      LLVM_DEBUG(dbgs() << ".. and it is unbeatable\n");
      break;
    }
  }

  return Best;
}

/// Determine the low relocation that matches the given relocation.
/// If the relocation does not need a low relocation then the return value
/// is ELF::R_FGPU_NONE.
///
/// The relocations that need a matching low part are
/// R_(FGPU|MICROFGPU|FGPU16)_HI16 for all symbols and
/// R_(FGPU|MICROFGPU|FGPU16)_GOT16 for local symbols only.
static unsigned getMatchingLoType(const ELFRelocationEntry &Reloc) {
  unsigned Type = Reloc.Type;
  if (Type == ELF::R_FGPU_HI16)
    return ELF::R_FGPU_LO16;
  if (Type == ELF::R_MICROFGPU_HI16)
    return ELF::R_MICROFGPU_LO16;
  if (Type == ELF::R_FGPU16_HI16)
    return ELF::R_FGPU16_LO16;

  if (Reloc.OriginalSymbol &&
      Reloc.OriginalSymbol->getBinding() != ELF::STB_LOCAL)
    return ELF::R_FGPU_NONE;

  if (Type == ELF::R_FGPU_GOT16)
    return ELF::R_FGPU_LO16;
  if (Type == ELF::R_MICROFGPU_GOT16)
    return ELF::R_MICROFGPU_LO16;
  if (Type == ELF::R_FGPU16_GOT16)
    return ELF::R_FGPU16_LO16;

  return ELF::R_FGPU_NONE;
}

/// Determine whether a relocation (X) matches the one given in R.
///
/// A relocation matches if:
/// - It's type matches that of a corresponding low part. This is provided in
///   MatchingType for efficiency.
/// - It's based on the same symbol.
/// - It's offset of greater or equal to that of the one given in R.
///   It should be noted that this rule assumes the programmer does not use
///   offsets that exceed the alignment of the symbol. The carry-bit will be
///   incorrect if this is not true.
///
/// A matching relocation is unbeatable if:
/// - It is not already involved in a match.
/// - It's offset is exactly that of the one given in R.
static FindBestPredicateResult isMatchingReloc(const FgpuRelocationEntry &X,
                                               const ELFRelocationEntry &R,
                                               unsigned MatchingType) {
  if (X.R.Type == MatchingType && X.R.OriginalSymbol == R.OriginalSymbol) {
    if (!X.Matched &&
        X.R.OriginalAddend == R.OriginalAddend)
      return FindBest_PerfectMatch;
    else if (X.R.OriginalAddend >= R.OriginalAddend)
      return FindBest_Match;
  }
  return FindBest_NoMatch;
}

/// Determine whether Candidate or PreviousBest is the better match.
/// The return value is true if Candidate is the better match.
///
/// A matching relocation is a better match if:
/// - It has a smaller addend.
/// - It is not already involved in a match.
static bool compareMatchingRelocs(const FgpuRelocationEntry &Candidate,
                                  const FgpuRelocationEntry &PreviousBest) {
  if (Candidate.R.OriginalAddend != PreviousBest.R.OriginalAddend)
    return Candidate.R.OriginalAddend < PreviousBest.R.OriginalAddend;
  return PreviousBest.Matched && !Candidate.Matched;
}

#ifndef NDEBUG
/// Print all the relocations.
template <class Container>
static void dumpRelocs(const char *Prefix, const Container &Relocs) {
  for (const auto &R : Relocs)
    dbgs() << Prefix << R << "\n";
}
#endif

FgpuELFObjectWriter::FgpuELFObjectWriter(uint8_t OSABI,
                                         bool HasRelocationAddend, bool Is64)
    : MCELFObjectTargetWriter(Is64, OSABI, ELF::EM_FGPU, HasRelocationAddend) {}

unsigned FgpuELFObjectWriter::getRelocType(MCContext &Ctx,
                                           const MCValue &Target,
                                           const MCFixup &Fixup,
                                           bool IsPCRel) const {
  // Determine the type of the relocation.
  unsigned Kind = Fixup.getTargetKind();

  switch (Kind) {
  case FK_NONE:
    return ELF::R_FGPU_NONE;
  case FK_Data_1:
    Ctx.reportError(Fixup.getLoc(),
                    "FGPU does not support one byte relocations");
    return ELF::R_FGPU_NONE;
  case Fgpu::fixup_Fgpu_16:
  case FK_Data_2:
    return IsPCRel ? ELF::R_FGPU_PC16 : ELF::R_FGPU_16;
  case Fgpu::fixup_Fgpu_32:
  case FK_Data_4:
    return IsPCRel ? ELF::R_FGPU_PC32 : ELF::R_FGPU_32;
  case Fgpu::fixup_Fgpu_64:
  case FK_Data_8:
    return IsPCRel
               ? setRTypes(ELF::R_FGPU_PC32, ELF::R_FGPU_64, ELF::R_FGPU_NONE)
               : (unsigned)ELF::R_FGPU_64;
  }

  if (IsPCRel) {
    switch (Kind) {
    case Fgpu::fixup_Fgpu_Branch_PCRel:
    case Fgpu::fixup_Fgpu_PC16:
      return ELF::R_FGPU_PC16;
    case Fgpu::fixup_MICROFGPU_PC7_S1:
      return ELF::R_MICROFGPU_PC7_S1;
    case Fgpu::fixup_MICROFGPU_PC10_S1:
      return ELF::R_MICROFGPU_PC10_S1;
    case Fgpu::fixup_MICROFGPU_PC16_S1:
      return ELF::R_MICROFGPU_PC16_S1;
    case Fgpu::fixup_MICROFGPU_PC26_S1:
      return ELF::R_MICROFGPU_PC26_S1;
    case Fgpu::fixup_MICROFGPU_PC19_S2:
      return ELF::R_MICROFGPU_PC19_S2;
    case Fgpu::fixup_MICROFGPU_PC18_S3:
      return ELF::R_MICROFGPU_PC18_S3;
    case Fgpu::fixup_MICROFGPU_PC21_S1:
      return ELF::R_MICROFGPU_PC21_S1;
    case Fgpu::fixup_FGPU_PC19_S2:
      return ELF::R_FGPU_PC19_S2;
    case Fgpu::fixup_FGPU_PC18_S3:
      return ELF::R_FGPU_PC18_S3;
    case Fgpu::fixup_FGPU_PC21_S2:
      return ELF::R_FGPU_PC21_S2;
    case Fgpu::fixup_FGPU_PC26_S2:
      return ELF::R_FGPU_PC26_S2;
    case Fgpu::fixup_FGPU_PCHI16:
      return ELF::R_FGPU_PCHI16;
    case Fgpu::fixup_FGPU_PCLO16:
      return ELF::R_FGPU_PCLO16;
    case Fgpu::fixup_Fgpu_PC14:
      return ELF::R_FGPU_PC14;
    case Fgpu::fixup_Fgpu_PC7:
      return ELF::R_FGPU_PC7;
    case Fgpu::fixup_Fgpu_JSUB:
      return ELF::R_FGPU_JSUB;
    }

    llvm_unreachable("invalid PC-relative fixup kind!");
  }

  switch (Kind) {
  case FK_DTPRel_4:
    return ELF::R_FGPU_TLS_DTPREL32;
  case FK_DTPRel_8:
    return ELF::R_FGPU_TLS_DTPREL64;
  case FK_TPRel_4:
    return ELF::R_FGPU_TLS_TPREL32;
  case FK_TPRel_8:
    return ELF::R_FGPU_TLS_TPREL64;
  case FK_GPRel_4:
    return setRTypes(ELF::R_FGPU_GPREL32,
                     is64Bit() ? ELF::R_FGPU_64 : ELF::R_FGPU_NONE,
                     ELF::R_FGPU_NONE);
  case Fgpu::fixup_Fgpu_GPREL16:
    return ELF::R_FGPU_GPREL16;
  case Fgpu::fixup_Fgpu_26:
    return ELF::R_FGPU_26;
  case Fgpu::fixup_Fgpu_CALL16:
    return ELF::R_FGPU_CALL16;
  case Fgpu::fixup_Fgpu_GOT:
    return ELF::R_FGPU_GOT16;
  case Fgpu::fixup_Fgpu_HI16:
    return ELF::R_FGPU_HI16;
  case Fgpu::fixup_Fgpu_LO16:
    return ELF::R_FGPU_LO16;
  case Fgpu::fixup_Fgpu_TLSGD:
    return ELF::R_FGPU_TLS_GD;
  case Fgpu::fixup_Fgpu_GOTTPREL:
    return ELF::R_FGPU_TLS_GOTTPREL;
  case Fgpu::fixup_Fgpu_TPREL_HI:
    return ELF::R_FGPU_TLS_TPREL_HI16;
  case Fgpu::fixup_Fgpu_TPREL_LO:
    return ELF::R_FGPU_TLS_TPREL_LO16;
  case Fgpu::fixup_Fgpu_TLSLDM:
    return ELF::R_FGPU_TLS_LDM;
  case Fgpu::fixup_Fgpu_DTPREL_HI:
    return ELF::R_FGPU_TLS_DTPREL_HI16;
  case Fgpu::fixup_Fgpu_DTPREL_LO:
    return ELF::R_FGPU_TLS_DTPREL_LO16;
  case Fgpu::fixup_Fgpu_GOT_PAGE:
    return ELF::R_FGPU_GOT_PAGE;
  case Fgpu::fixup_Fgpu_GOT_OFST:
    return ELF::R_FGPU_GOT_OFST;
  case Fgpu::fixup_Fgpu_GOT_DISP:
    return ELF::R_FGPU_GOT_DISP;
  case Fgpu::fixup_Fgpu_GPOFF_HI:
    return setRTypes(ELF::R_FGPU_GPREL16, ELF::R_FGPU_SUB, ELF::R_FGPU_HI16);
  case Fgpu::fixup_MICROFGPU_GPOFF_HI:
    return setRTypes(ELF::R_MICROFGPU_GPREL16, ELF::R_MICROFGPU_SUB,
                     ELF::R_MICROFGPU_HI16);
  case Fgpu::fixup_Fgpu_GPOFF_LO:
    return setRTypes(ELF::R_FGPU_GPREL16, ELF::R_FGPU_SUB, ELF::R_FGPU_LO16);
  case Fgpu::fixup_MICROFGPU_GPOFF_LO:
    return setRTypes(ELF::R_MICROFGPU_GPREL16, ELF::R_MICROFGPU_SUB,
                     ELF::R_MICROFGPU_LO16);
  case Fgpu::fixup_Fgpu_HIGHER:
    return ELF::R_FGPU_HIGHER;
  case Fgpu::fixup_Fgpu_HIGHEST:
    return ELF::R_FGPU_HIGHEST;
  case Fgpu::fixup_Fgpu_SUB:
    return ELF::R_FGPU_SUB;
  case Fgpu::fixup_Fgpu_GOT_HI16:
    return ELF::R_FGPU_GOT_HI16;
  case Fgpu::fixup_Fgpu_GOT_LO16:
    return ELF::R_FGPU_GOT_LO16;
  case Fgpu::fixup_Fgpu_CALL_HI16:
    return ELF::R_FGPU_CALL_HI16;
  case Fgpu::fixup_Fgpu_CALL_LO16:
    return ELF::R_FGPU_CALL_LO16;
  case Fgpu::fixup_MICROFGPU_26_S1:
    return ELF::R_MICROFGPU_26_S1;
  case Fgpu::fixup_MICROFGPU_HI16:
    return ELF::R_MICROFGPU_HI16;
  case Fgpu::fixup_MICROFGPU_LO16:
    return ELF::R_MICROFGPU_LO16;
  case Fgpu::fixup_MICROFGPU_GOT16:
    return ELF::R_MICROFGPU_GOT16;
  case Fgpu::fixup_MICROFGPU_CALL16:
    return ELF::R_MICROFGPU_CALL16;
  case Fgpu::fixup_MICROFGPU_GOT_DISP:
    return ELF::R_MICROFGPU_GOT_DISP;
  case Fgpu::fixup_MICROFGPU_GOT_PAGE:
    return ELF::R_MICROFGPU_GOT_PAGE;
  case Fgpu::fixup_MICROFGPU_GOT_OFST:
    return ELF::R_MICROFGPU_GOT_OFST;
  case Fgpu::fixup_MICROFGPU_TLS_GD:
    return ELF::R_MICROFGPU_TLS_GD;
  case Fgpu::fixup_MICROFGPU_TLS_LDM:
    return ELF::R_MICROFGPU_TLS_LDM;
  case Fgpu::fixup_MICROFGPU_TLS_DTPREL_HI16:
    return ELF::R_MICROFGPU_TLS_DTPREL_HI16;
  case Fgpu::fixup_MICROFGPU_TLS_DTPREL_LO16:
    return ELF::R_MICROFGPU_TLS_DTPREL_LO16;
  case Fgpu::fixup_MICROFGPU_GOTTPREL:
    return ELF::R_MICROFGPU_TLS_GOTTPREL;
  case Fgpu::fixup_MICROFGPU_TLS_TPREL_HI16:
    return ELF::R_MICROFGPU_TLS_TPREL_HI16;
  case Fgpu::fixup_MICROFGPU_TLS_TPREL_LO16:
    return ELF::R_MICROFGPU_TLS_TPREL_LO16;
  case Fgpu::fixup_MICROFGPU_SUB:
    return ELF::R_MICROFGPU_SUB;
  case Fgpu::fixup_MICROFGPU_HIGHER:
    return ELF::R_MICROFGPU_HIGHER;
  case Fgpu::fixup_MICROFGPU_HIGHEST:
    return ELF::R_MICROFGPU_HIGHEST;
  case Fgpu::fixup_Fgpu_JALR:
    return ELF::R_FGPU_JALR;
  case Fgpu::fixup_MICROFGPU_JALR:
    return ELF::R_MICROFGPU_JALR;
  }

  llvm_unreachable("invalid fixup kind!");
}

/// Sort relocation table entries by offset except where another order is
/// required by the FGPU ABI.
///
/// FGPU has a few relocations that have an AHL component in the expression used
/// to evaluate them. This AHL component is an addend with the same number of
/// bits as a symbol value but not all of our ABI's are able to supply a
/// sufficiently sized addend in a single relocation.
///
/// The O32 ABI for example, uses REL relocations which store the addend in the
/// section data. All the relocations with AHL components affect 16-bit fields
/// so the addend for a single relocation is limited to 16-bit. This ABI
/// resolves the limitation by linking relocations (e.g. R_FGPU_HI16 and
/// R_FGPU_LO16) and distributing the addend between the linked relocations. The
/// ABI mandates that such relocations must be next to each other in a
/// particular order (e.g. R_FGPU_HI16 must be immediately followed by a
/// matching R_FGPU_LO16) but the rule is less strict in practice.
///
/// The de facto standard is lenient in the following ways:
/// - 'Immediately following' does not refer to the next relocation entry but
///   the next matching relocation.
/// - There may be multiple high parts relocations for one low part relocation.
/// - There may be multiple low part relocations for one high part relocation.
/// - The AHL addend in each part does not have to be exactly equal as long as
///   the difference does not affect the carry bit from bit 15 into 16. This is
///   to allow, for example, the use of %lo(foo) and %lo(foo+4) when loading
///   both halves of a long long.
///
/// See getMatchingLoType() for a description of which high part relocations
/// match which low part relocations. One particular thing to note is that
/// R_FGPU_GOT16 and similar only have AHL addends if they refer to local
/// symbols.
///
/// It should also be noted that this function is not affected by whether
/// the symbol was kept or rewritten into a section-relative equivalent. We
/// always match using the expressions from the source.
void FgpuELFObjectWriter::sortRelocs(const MCAssembler &Asm,
                                     std::vector<ELFRelocationEntry> &Relocs) {
  // We do not need to sort the relocation table for RELA relocations which
  // N32/N64 uses as the relocation addend contains the value we require,
  // rather than it being split across a pair of relocations.
  if (hasRelocationAddend())
    return;

  if (Relocs.size() < 2)
    return;

  // Sort relocations by the address they are applied to.
  llvm::sort(Relocs,
             [](const ELFRelocationEntry &A, const ELFRelocationEntry &B) {
               return A.Offset < B.Offset;
             });

  std::list<FgpuRelocationEntry> Sorted;
  std::list<ELFRelocationEntry> Remainder;

  LLVM_DEBUG(dumpRelocs("R: ", Relocs));

  // Separate the movable relocations (AHL relocations using the high bits) from
  // the immobile relocations (everything else). This does not preserve high/low
  // matches that already existed in the input.
  copy_if_else(Relocs.begin(), Relocs.end(), std::back_inserter(Remainder),
               std::back_inserter(Sorted), [](const ELFRelocationEntry &Reloc) {
                 return getMatchingLoType(Reloc) != ELF::R_FGPU_NONE;
               });

  for (auto &R : Remainder) {
    LLVM_DEBUG(dbgs() << "Matching: " << R << "\n");

    unsigned MatchingType = getMatchingLoType(R);
    assert(MatchingType != ELF::R_FGPU_NONE &&
           "Wrong list for reloc that doesn't need a match");

    // Find the best matching relocation for the current high part.
    // See isMatchingReloc for a description of a matching relocation and
    // compareMatchingRelocs for a description of what 'best' means.
    auto InsertionPoint =
        find_best(Sorted.begin(), Sorted.end(),
                  [&R, &MatchingType](const FgpuRelocationEntry &X) {
                    return isMatchingReloc(X, R, MatchingType);
                  },
                  compareMatchingRelocs);

    // If we matched then insert the high part in front of the match and mark
    // both relocations as being involved in a match. We only mark the high
    // part for cosmetic reasons in the debug output.
    //
    // If we failed to find a match then the high part is orphaned. This is not
    // permitted since the relocation cannot be evaluated without knowing the
    // carry-in. We can sometimes handle this using a matching low part that is
    // already used in a match but we already cover that case in
    // isMatchingReloc and compareMatchingRelocs. For the remaining cases we
    // should insert the high part at the end of the list. This will cause the
    // linker to fail but the alternative is to cause the linker to bind the
    // high part to a semi-matching low part and silently calculate the wrong
    // value. Unfortunately we have no means to warn the user that we did this
    // so leave it up to the linker to complain about it.
    if (InsertionPoint != Sorted.end())
      InsertionPoint->Matched = true;
    Sorted.insert(InsertionPoint, R)->Matched = true;
  }

  LLVM_DEBUG(dumpRelocs("S: ", Sorted));

  assert(Relocs.size() == Sorted.size() && "Some relocs were not consumed");

  // Overwrite the original vector with the sorted elements. The caller expects
  // them in reverse order.
  unsigned CopyTo = 0;
  for (const auto &R : reverse(Sorted))
    Relocs[CopyTo++] = R.R;
}

bool FgpuELFObjectWriter::needsRelocateWithSymbol(const MCSymbol &Sym,
                                                  unsigned Type) const {
  // If it's a compound relocation for N64 then we need the relocation if any
  // sub-relocation needs it.
  if (!isUInt<8>(Type))
    return needsRelocateWithSymbol(Sym, Type & 0xff) ||
           needsRelocateWithSymbol(Sym, (Type >> 8) & 0xff) ||
           needsRelocateWithSymbol(Sym, (Type >> 16) & 0xff);

  switch (Type) {
  default:
    errs() << Type << "\n";
    llvm_unreachable("Unexpected relocation");
    return true;

  // This relocation doesn't affect the section data.
  case ELF::R_FGPU_NONE:
    return false;

  // On REL ABI's (e.g. O32), these relocations form pairs. The pairing is done
  // by the static linker by matching the symbol and offset.
  // We only see one relocation at a time but it's still safe to relocate with
  // the section so long as both relocations make the same decision.
  //
  // Some older linkers may require the symbol for particular cases. Such cases
  // are not supported yet but can be added as required.
  case ELF::R_FGPU_GOT16:
  case ELF::R_FGPU16_GOT16:
  case ELF::R_MICROFGPU_GOT16:
  case ELF::R_FGPU_HIGHER:
  case ELF::R_FGPU_HIGHEST:
  case ELF::R_FGPU_HI16:
  case ELF::R_FGPU16_HI16:
  case ELF::R_MICROFGPU_HI16:
  case ELF::R_FGPU_LO16:
  case ELF::R_FGPU16_LO16:
  case ELF::R_MICROFGPU_LO16:
    // FIXME: It should be safe to return false for the STO_FGPU_MICROFGPU but
    //        we neglect to handle the adjustment to the LSB of the addend that
    //        it causes in applyFixup() and similar.
    if (cast<MCSymbolELF>(Sym).getOther() & ELF::STO_FGPU_MICROFGPU)
      return true;
    return false;

  case ELF::R_FGPU_GOT_PAGE:
  case ELF::R_MICROFGPU_GOT_PAGE:
  case ELF::R_FGPU_GOT_OFST:
  case ELF::R_MICROFGPU_GOT_OFST:
  case ELF::R_FGPU_16:
  case ELF::R_FGPU_32:
  case ELF::R_FGPU_GPREL32:
    if (cast<MCSymbolELF>(Sym).getOther() & ELF::STO_FGPU_MICROFGPU)
      return true;
    LLVM_FALLTHROUGH;
  case ELF::R_FGPU_26:
  case ELF::R_FGPU_64:
  case ELF::R_FGPU_GPREL16:
  case ELF::R_FGPU_PC16:
  case ELF::R_FGPU_SUB:
    return false;

  // FIXME: Many of these relocations should probably return false but this
  //        hasn't been confirmed to be safe yet.
  case ELF::R_FGPU_REL32:
  case ELF::R_FGPU_LITERAL:
  case ELF::R_FGPU_CALL16:
  case ELF::R_FGPU_SHIFT5:
  case ELF::R_FGPU_SHIFT6:
  case ELF::R_FGPU_GOT_DISP:
  case ELF::R_FGPU_GOT_HI16:
  case ELF::R_FGPU_GOT_LO16:
  case ELF::R_FGPU_INSERT_A:
  case ELF::R_FGPU_INSERT_B:
  case ELF::R_FGPU_DELETE:
  case ELF::R_FGPU_CALL_HI16:
  case ELF::R_FGPU_CALL_LO16:
  case ELF::R_FGPU_SCN_DISP:
  case ELF::R_FGPU_REL16:
  case ELF::R_FGPU_ADD_IMMEDIATE:
  case ELF::R_FGPU_PJUMP:
  case ELF::R_FGPU_RELGOT:
  case ELF::R_FGPU_JALR:
  case ELF::R_FGPU_TLS_DTPMOD32:
  case ELF::R_FGPU_TLS_DTPREL32:
  case ELF::R_FGPU_TLS_DTPMOD64:
  case ELF::R_FGPU_TLS_DTPREL64:
  case ELF::R_FGPU_TLS_GD:
  case ELF::R_FGPU_TLS_LDM:
  case ELF::R_FGPU_TLS_DTPREL_HI16:
  case ELF::R_FGPU_TLS_DTPREL_LO16:
  case ELF::R_FGPU_TLS_GOTTPREL:
  case ELF::R_FGPU_TLS_TPREL32:
  case ELF::R_FGPU_TLS_TPREL64:
  case ELF::R_FGPU_TLS_TPREL_HI16:
  case ELF::R_FGPU_TLS_TPREL_LO16:
  case ELF::R_FGPU_GLOB_DAT:
  case ELF::R_FGPU_PC21_S2:
  case ELF::R_FGPU_PC26_S2:
  case ELF::R_FGPU_PC18_S3:
  case ELF::R_FGPU_PC19_S2:
  case ELF::R_FGPU_PCHI16:
  case ELF::R_FGPU_PCLO16:
  case ELF::R_FGPU_COPY:
  case ELF::R_FGPU_JUMP_SLOT:
  case ELF::R_FGPU_NUM:
  case ELF::R_FGPU_PC32:
  case ELF::R_FGPU_EH:
  case ELF::R_MICROFGPU_26_S1:
  case ELF::R_MICROFGPU_GPREL16:
  case ELF::R_MICROFGPU_LITERAL:
  case ELF::R_MICROFGPU_PC7_S1:
  case ELF::R_MICROFGPU_PC10_S1:
  case ELF::R_MICROFGPU_PC16_S1:
  case ELF::R_MICROFGPU_CALL16:
  case ELF::R_MICROFGPU_GOT_DISP:
  case ELF::R_MICROFGPU_GOT_HI16:
  case ELF::R_MICROFGPU_GOT_LO16:
  case ELF::R_MICROFGPU_SUB:
  case ELF::R_MICROFGPU_HIGHER:
  case ELF::R_MICROFGPU_HIGHEST:
  case ELF::R_MICROFGPU_CALL_HI16:
  case ELF::R_MICROFGPU_CALL_LO16:
  case ELF::R_MICROFGPU_SCN_DISP:
  case ELF::R_MICROFGPU_JALR:
  case ELF::R_MICROFGPU_HI0_LO16:
  case ELF::R_MICROFGPU_TLS_GD:
  case ELF::R_MICROFGPU_TLS_LDM:
  case ELF::R_MICROFGPU_TLS_DTPREL_HI16:
  case ELF::R_MICROFGPU_TLS_DTPREL_LO16:
  case ELF::R_MICROFGPU_TLS_GOTTPREL:
  case ELF::R_MICROFGPU_TLS_TPREL_HI16:
  case ELF::R_MICROFGPU_TLS_TPREL_LO16:
  case ELF::R_MICROFGPU_GPREL7_S2:
  case ELF::R_MICROFGPU_PC23_S2:
  case ELF::R_MICROFGPU_PC21_S1:
  case ELF::R_MICROFGPU_PC26_S1:
  case ELF::R_MICROFGPU_PC18_S3:
  case ELF::R_MICROFGPU_PC19_S2:
    return true;

  // FIXME: Many of these should probably return false but FGPU16 isn't
  //        supported by the integrated assembler.
  case ELF::R_FGPU16_26:
  case ELF::R_FGPU16_GPREL:
  case ELF::R_FGPU16_CALL16:
  case ELF::R_FGPU16_TLS_GD:
  case ELF::R_FGPU16_TLS_LDM:
  case ELF::R_FGPU16_TLS_DTPREL_HI16:
  case ELF::R_FGPU16_TLS_DTPREL_LO16:
  case ELF::R_FGPU16_TLS_GOTTPREL:
  case ELF::R_FGPU16_TLS_TPREL_HI16:
  case ELF::R_FGPU16_TLS_TPREL_LO16:
    llvm_unreachable("Unsupported FGPU16 relocation");
    return true;
  }
}

std::unique_ptr<MCObjectTargetWriter>
llvm::createFgpuELFObjectWriter(const Triple &TT, bool IsN32) {
  uint8_t OSABI = MCELFObjectTargetWriter::getOSABI(TT.getOS());
  bool IsN64 = TT.isArch64Bit() && !IsN32;
  bool HasRelocationAddend = TT.isArch64Bit();
  return std::make_unique<FgpuELFObjectWriter>(OSABI, HasRelocationAddend,
                                                IsN64);
}
