//===-- Fgpu.td - Describe the Fgpu Target Machine ---------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
// This is the top level entry point for the Fgpu target.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Target-independent interfaces
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

// The overall idea of the PredicateControl class is to chop the Predicates list
// into subsets that are usually overridden independently. This allows
// subclasses to partially override the predicates of their superclasses without
// having to re-add all the existing predicates.
class PredicateControl {
  // Predicates for the encoding scheme in use such as HasStdEnc
  list<Predicate> EncodingPredicates = [];
  // Predicates for the GPR size such as IsGP64bit
  list<Predicate> GPRPredicates = [];
  // Predicates for the PTR size such as IsPTR64bit
  list<Predicate> PTRPredicates = [];
  // Predicates for a symbol's size such as hasSym32.
  list<Predicate> SYMPredicates = [];
  // Predicates for the FGR size and layout such as IsFP64bit
  list<Predicate> FGRPredicates = [];
  // Predicates for the instruction group membership such as ISA's.
  list<Predicate> InsnPredicates = [];
  // Predicate for the ASE that an instruction belongs to.
  list<Predicate> ASEPredicate = [];
  // Predicate for marking the instruction as usable in hard-float mode only.
  list<Predicate> HardFloatPredicate = [];
  // Predicates for anything else
  list<Predicate> AdditionalPredicates = [];
  list<Predicate> Predicates = !listconcat(EncodingPredicates,
                                           GPRPredicates,
                                           PTRPredicates,
                                           SYMPredicates,
                                           FGRPredicates,
                                           InsnPredicates,
                                           HardFloatPredicate,
                                           ASEPredicate,
                                           AdditionalPredicates);
}

// Like Requires<> but for the AdditionalPredicates list
class AdditionalRequires<list<Predicate> preds> {
  list<Predicate> AdditionalPredicates = preds;
}

//===----------------------------------------------------------------------===//
// Fgpu Subtarget features                                                    //
//===----------------------------------------------------------------------===//

//def FeatureGINV : SubtargetFeature<"ginv", "HasGINV", "true",
//                                   "Fgpu Global Invalidate ASE">;

//===----------------------------------------------------------------------===//
// Register File, Calling Conv, Instruction Descriptions
//===----------------------------------------------------------------------===//

include "FgpuRegisterInfo.td"
include "FgpuSchedule.td"
include "FgpuInstrInfo.td"
include "FgpuCallingConv.td"
include "FgpuRegisterBanks.td"

// Avoid forward declaration issues.
include "FgpuScheduleGeneric.td"

def FgpuInstrInfo : InstrInfo;

//===----------------------------------------------------------------------===//
// Fgpu processors supported.
//===----------------------------------------------------------------------===//


class Proc<string Name, list<SubtargetFeature> Features>
 : ProcessorModel<Name, FgpuGenericModel, Features>;

def : Proc<"generic", []>;

def FgpuAsmParser : AsmParser {
  let ShouldEmitMatchRegisterName = 0;
}

def FgpuAsmParserVariant : AsmParserVariant {
  int Variant = 0;

  // Recognize hard coded registers.
  string RegisterPrefix = "$";
}

def Fgpu : Target {
  let InstructionSet = FgpuInstrInfo;
  let AssemblyParsers = [FgpuAsmParser];
  let AssemblyParserVariants = [FgpuAsmParserVariant];
  let AllowRegisterRenaming = 1;
}

