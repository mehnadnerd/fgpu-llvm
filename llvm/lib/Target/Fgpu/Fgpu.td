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

def FeatureNoABICalls  : SubtargetFeature<"noabicalls", "NoABICalls", "true",
                                "Disable SVR4-style position-independent code">;
def FeaturePTR64Bit    : SubtargetFeature<"ptr64", "IsPTR64bit", "true",
                                "Pointers are 64-bit wide">;
def FeatureGP64Bit     : SubtargetFeature<"gp64", "IsGP64bit", "true",
                                "General Purpose Registers are 64-bit wide">;
def FeatureFP64Bit     : SubtargetFeature<"fp64", "IsFP64bit", "true",
                                "Support 64-bit FP registers">;
def FeatureFPXX        : SubtargetFeature<"fpxx", "IsFPXX", "true",
                                "Support for FPXX">;
def FeatureNaN2008     : SubtargetFeature<"nan2008", "IsNaN2008bit", "true",
                                "IEEE 754-2008 NaN encoding">;
def FeatureAbs2008     : SubtargetFeature<"abs2008", "Abs2008", "true",
                                          "Disable IEEE 754-2008 abs.fmt mode">;
def FeatureSingleFloat : SubtargetFeature<"single-float", "IsSingleFloat",
                                "true", "Only supports single precision float">;
def FeatureSoftFloat   : SubtargetFeature<"soft-float", "IsSoftFloat", "true",
                                "Does not support floating point instructions">;
def FeatureNoOddSPReg  : SubtargetFeature<"nooddspreg", "UseOddSPReg", "false",
                              "Disable odd numbered single-precision "
                              "registers">;
def FeatureVFPU        : SubtargetFeature<"vfpu", "HasVFPU",
                                "true", "Enable vector FPU instructions">;
def FeatureFgpu1       : SubtargetFeature<"fgpu1", "FgpuArchVersion", "Fgpu1",
                                "Fgpu I ISA Support [highly experimental]">;
def FeatureFgpu2       : SubtargetFeature<"fgpu2", "FgpuArchVersion", "Fgpu2",
                                "Fgpu II ISA Support [highly experimental]",
                                [FeatureFgpu1]>;
def FeatureFgpu3_32    : SubtargetFeature<"fgpu3_32", "HasFgpu3_32", "true",
                                "Subset of FGPU-III that is also in FGPU32 "
                                "[highly experimental]">;
def FeatureFgpu3_32r2  : SubtargetFeature<"fgpu3_32r2", "HasFgpu3_32r2", "true",
                                "Subset of FGPU-III that is also in FGPU32r2 "
                                "[highly experimental]">;
def FeatureFgpu3       : SubtargetFeature<"fgpu3", "FgpuArchVersion", "Fgpu3",
                                "FGPU III ISA Support [highly experimental]",
                                [FeatureFgpu2, FeatureFgpu3_32,
                                 FeatureFgpu3_32r2, FeatureGP64Bit,
                                 FeatureFP64Bit]>;
def FeatureFgpu4_32    : SubtargetFeature<"fgpu4_32", "HasFgpu4_32", "true",
                                "Subset of FGPU-IV that is also in FGPU32 "
                                "[highly experimental]">;
def FeatureFgpu4_32r2  : SubtargetFeature<"fgpu4_32r2", "HasFgpu4_32r2", "true",
                                "Subset of FGPU-IV that is also in FGPU32r2 "
                                "[highly experimental]">;
def FeatureFgpu4       : SubtargetFeature<"fgpu4", "FgpuArchVersion",
                                "Fgpu4", "FGPU IV ISA Support",
                                [FeatureFgpu3, FeatureFgpu4_32,
                                 FeatureFgpu4_32r2]>;
def FeatureFgpu5_32r2  : SubtargetFeature<"fgpu5_32r2", "HasFgpu5_32r2", "true",
                                "Subset of FGPU-V that is also in FGPU32r2 "
                                "[highly experimental]">;
def FeatureFgpu5       : SubtargetFeature<"fgpu5", "FgpuArchVersion", "Fgpu5",
                                "FGPU V ISA Support [highly experimental]",
                                [FeatureFgpu4, FeatureFgpu5_32r2]>;
def FeatureFgpu32      : SubtargetFeature<"fgpu32", "FgpuArchVersion", "Fgpu32",
                                "Fgpu32 ISA Support",
                                [FeatureFgpu2, FeatureFgpu3_32,
                                 FeatureFgpu4_32]>;
def FeatureFgpu32r2    : SubtargetFeature<"fgpu32r2", "FgpuArchVersion",
                                "Fgpu32r2", "Fgpu32r2 ISA Support",
                                [FeatureFgpu3_32r2, FeatureFgpu4_32r2,
                                 FeatureFgpu5_32r2, FeatureFgpu32]>;
def FeatureFgpu32r3    : SubtargetFeature<"fgpu32r3", "FgpuArchVersion",
                                "Fgpu32r3", "Fgpu32r3 ISA Support",
                                [FeatureFgpu32r2]>;
def FeatureFgpu32r5    : SubtargetFeature<"fgpu32r5", "FgpuArchVersion",
                                "Fgpu32r5", "Fgpu32r5 ISA Support",
                                [FeatureFgpu32r3]>;
def FeatureFgpu32r6    : SubtargetFeature<"fgpu32r6", "FgpuArchVersion",
                                "Fgpu32r6",
                                "Fgpu32r6 ISA Support [experimental]",
                                [FeatureFgpu32r5, FeatureFP64Bit,
                                 FeatureNaN2008, FeatureAbs2008]>;
def FeatureFgpu64      : SubtargetFeature<"fgpu64", "FgpuArchVersion",
                                "Fgpu64", "Fgpu64 ISA Support",
                                [FeatureFgpu5, FeatureFgpu32]>;
def FeatureFgpu64r2    : SubtargetFeature<"fgpu64r2", "FgpuArchVersion",
                                "Fgpu64r2", "Fgpu64r2 ISA Support",
                                [FeatureFgpu64, FeatureFgpu32r2]>;
def FeatureFgpu64r3    : SubtargetFeature<"fgpu64r3", "FgpuArchVersion",
                                "Fgpu64r3", "Fgpu64r3 ISA Support",
                                [FeatureFgpu64r2, FeatureFgpu32r3]>;
def FeatureFgpu64r5    : SubtargetFeature<"fgpu64r5", "FgpuArchVersion",
                                "Fgpu64r5", "Fgpu64r5 ISA Support",
                                [FeatureFgpu64r3, FeatureFgpu32r5]>;
def FeatureFgpu64r6    : SubtargetFeature<"fgpu64r6", "FgpuArchVersion",
                                "Fgpu64r6",
                                "Fgpu64r6 ISA Support [experimental]",
                                [FeatureFgpu32r6, FeatureFgpu64r5,
                                 FeatureNaN2008, FeatureAbs2008]>;
def FeatureSym32       : SubtargetFeature<"sym32", "HasSym32", "true",
                                          "Symbols are 32 bit on Fgpu64">;

def FeatureFgpu16  : SubtargetFeature<"fgpu16", "InFgpu16Mode", "true",
                                      "Fgpu16 mode">;

def FeatureDSP : SubtargetFeature<"dsp", "HasDSP", "true", "Fgpu DSP ASE">;
def FeatureDSPR2 : SubtargetFeature<"dspr2", "HasDSPR2", "true",
                                    "Fgpu DSP-R2 ASE", [FeatureDSP]>;
def FeatureDSPR3
    : SubtargetFeature<"dspr3", "HasDSPR3", "true", "Fgpu DSP-R3 ASE",
                       [ FeatureDSP, FeatureDSPR2 ]>;

def FeatureFgpu3D : SubtargetFeature<"fgpu3d", "Has3D", "true", "Fgpu 3D ASE">;

def FeatureMSA : SubtargetFeature<"msa", "HasMSA", "true", "Fgpu MSA ASE">;

def FeatureEVA : SubtargetFeature<"eva", "HasEVA", "true", "Fgpu EVA ASE">;

def FeatureCRC : SubtargetFeature<"crc", "HasCRC", "true", "Fgpu R6 CRC ASE">;

def FeatureVirt : SubtargetFeature<"virt", "HasVirt", "true",
                                   "Fgpu Virtualization ASE">;

def FeatureGINV : SubtargetFeature<"ginv", "HasGINV", "true",
                                   "Fgpu Global Invalidate ASE">;

def FeatureMicroFgpu  : SubtargetFeature<"microfgpu", "InMicroFgpuMode", "true",
                                         "microFgpu mode">;

def FeatureCnFgpu : SubtargetFeature<"cnfgpu", "HasCnFgpu",
                                     "true", "Octeon cnFGPU Support",
                                     [FeatureFgpu64r2]>;

def FeatureCnFgpuP : SubtargetFeature<"cnfgpup", "HasCnFgpuP",
                                      "true", "Octeon+ cnFGPU Support",
                                      [FeatureCnFgpu]>;

def FeatureUseTCCInDIV : SubtargetFeature<
                               "use-tcc-in-div",
                               "UseTCCInDIV", "false",
                               "Force the assembler to use trapping">;

def FeatureNoMadd4
    : SubtargetFeature<"nomadd4", "DisableMadd4", "true",
                       "Disable 4-operand madd.fmt and related instructions">;

def FeatureMT : SubtargetFeature<"mt", "HasMT", "true", "Fgpu MT ASE">;

def FeatureLongCalls : SubtargetFeature<"long-calls", "UseLongCalls", "true",
                                        "Disable use of the jal instruction">;

def FeatureXGOT
    : SubtargetFeature<"xgot", "UseXGOT", "true", "Assume 32-bit GOT">;

def FeatureUseIndirectJumpsHazard : SubtargetFeature<"use-indirect-jump-hazard",
                                                    "UseIndirectJumpsHazard",
                                                    "true", "Use indirect jump"
                        " guards to prevent certain speculation based attacks">;

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

def : Proc<"generic", [FeatureFgpu32]>;
def : Proc<"fgpu1", [FeatureFgpu1]>;
def : Proc<"fgpu2", [FeatureFgpu2]>;
def : Proc<"fgpu32", [FeatureFgpu32]>;
def : Proc<"fgpu32r2", [FeatureFgpu32r2]>;
def : Proc<"fgpu32r3", [FeatureFgpu32r3]>;
def : Proc<"fgpu32r5", [FeatureFgpu32r5]>;
def : Proc<"fgpu32r6", [FeatureFgpu32r6]>;

def : Proc<"fgpu3", [FeatureFgpu3]>;
def : Proc<"fgpu4", [FeatureFgpu4]>;
def : Proc<"fgpu5", [FeatureFgpu5]>;
def : Proc<"fgpu64", [FeatureFgpu64]>;
def : Proc<"fgpu64r2", [FeatureFgpu64r2]>;
def : Proc<"fgpu64r3", [FeatureFgpu64r3]>;
def : Proc<"fgpu64r5", [FeatureFgpu64r5]>;
def : Proc<"fgpu64r6", [FeatureFgpu64r6]>;
def : Proc<"octeon", [FeatureFgpu64r2, FeatureCnFgpu]>;
def : Proc<"octeon+", [FeatureFgpu64r2, FeatureCnFgpu, FeatureCnFgpuP]>;

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

//===----------------------------------------------------------------------===//
// Pfm Counters
//===----------------------------------------------------------------------===//

include "FgpuPfmCounters.td"
