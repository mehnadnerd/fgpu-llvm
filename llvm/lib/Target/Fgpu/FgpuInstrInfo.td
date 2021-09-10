//===- FgpuInstrInfo.td - Target Description for Fgpu Target -*- tablegen -*-=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the Fgpu implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//


//===----------------------------------------------------------------------===//
// Fgpu profiles and nodes
//===----------------------------------------------------------------------===//

def SDT_FgpuJmpLink      : SDTypeProfile<0, 1, [SDTCisVT<0, iPTR>]>;
def SDT_FgpuCMov         : SDTypeProfile<1, 4, [SDTCisSameAs<0, 1>,
                                                SDTCisSameAs<1, 2>,
                                                SDTCisSameAs<3, 4>,
                                                SDTCisInt<4>]>;
def SDT_FgpuCallSeqStart : SDCallSeqStart<[SDTCisVT<0, i32>, SDTCisVT<1, i32>]>;
def SDT_FgpuCallSeqEnd   : SDCallSeqEnd<[SDTCisVT<0, i32>, SDTCisVT<1, i32>]>;
def SDT_MFLOHI : SDTypeProfile<1, 1, [SDTCisInt<0>, SDTCisVT<1, untyped>]>;
def SDT_MTLOHI : SDTypeProfile<1, 2, [SDTCisVT<0, untyped>,
                                      SDTCisInt<1>, SDTCisSameAs<1, 2>]>;
def SDT_FgpuMultDiv : SDTypeProfile<1, 2, [SDTCisVT<0, untyped>, SDTCisInt<1>,
                                    SDTCisSameAs<1, 2>]>;
def SDT_FgpuMAddMSub : SDTypeProfile<1, 3,
                                     [SDTCisVT<0, untyped>, SDTCisSameAs<0, 3>,
                                      SDTCisVT<1, i32>, SDTCisSameAs<1, 2>]>;
def SDT_FgpuDivRem16 : SDTypeProfile<0, 2, [SDTCisInt<0>, SDTCisSameAs<0, 1>]>;

def SDT_FgpuThreadPointer : SDTypeProfile<1, 0, [SDTCisPtrTy<0>]>;

def SDT_Sync             : SDTypeProfile<0, 1, [SDTCisVT<0, i32>]>;

def SDT_Ext : SDTypeProfile<1, 3, [SDTCisInt<0>, SDTCisSameAs<0, 1>,
                                   SDTCisVT<2, i32>, SDTCisSameAs<2, 3>]>;
def SDT_Ins : SDTypeProfile<1, 4, [SDTCisInt<0>, SDTCisSameAs<0, 1>,
                                   SDTCisVT<2, i32>, SDTCisSameAs<2, 3>,
                                   SDTCisSameAs<0, 4>]>;

def SDTFgpuLoadLR  : SDTypeProfile<1, 2,
                                   [SDTCisInt<0>, SDTCisPtrTy<1>,
                                    SDTCisSameAs<0, 2>]>;

// Call
def FgpuJmpLink : SDNode<"FgpuISD::JmpLink",SDT_FgpuJmpLink,
                         [SDNPHasChain, SDNPOutGlue, SDNPOptInGlue,
                          SDNPVariadic]>;

// Tail call
def FgpuTailCall : SDNode<"FgpuISD::TailCall", SDT_FgpuJmpLink,
                          [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

// Hi and Lo nodes are used to handle global addresses. Used on
// FgpuISelLowering to lower stuff like GlobalAddress, ExternalSymbol
// static model. (nothing to do with Fgpu Registers Hi and Lo)

// Hi is the odd node out, on FGPU64 it can expand to either daddiu when
// using static relocations with 64 bit symbols, or lui when using 32 bit
// symbols.
def FgpuHigher : SDNode<"FgpuISD::Higher", SDTIntUnaryOp>;
def FgpuHighest : SDNode<"FgpuISD::Highest", SDTIntUnaryOp>;
def FgpuHi    : SDNode<"FgpuISD::Hi", SDTIntUnaryOp>;
def FgpuLo    : SDNode<"FgpuISD::Lo", SDTIntUnaryOp>;

def FgpuGPRel : SDNode<"FgpuISD::GPRel", SDTIntUnaryOp>;

// Hi node for accessing the GOT.
def FgpuGotHi : SDNode<"FgpuISD::GotHi", SDTIntUnaryOp>;

// Hi node for handling TLS offsets
def FgpuTlsHi   : SDNode<"FgpuISD::TlsHi", SDTIntUnaryOp>;

// Thread pointer
def FgpuThreadPointer: SDNode<"FgpuISD::ThreadPointer", SDT_FgpuThreadPointer>;

// Return
def FgpuRet : SDNode<"FgpuISD::Ret", SDTNone,
                     [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

def FgpuERet : SDNode<"FgpuISD::ERet", SDTNone,
                      [SDNPHasChain, SDNPOptInGlue, SDNPSideEffect]>;

// These are target-independent nodes, but have target-specific formats.
def callseq_start : SDNode<"ISD::CALLSEQ_START", SDT_FgpuCallSeqStart,
                           [SDNPHasChain, SDNPSideEffect, SDNPOutGlue]>;
def callseq_end   : SDNode<"ISD::CALLSEQ_END", SDT_FgpuCallSeqEnd,
                           [SDNPHasChain, SDNPSideEffect,
                            SDNPOptInGlue, SDNPOutGlue]>;

// Nodes used to extract LO/HI registers.
def FgpuMFHI : SDNode<"FgpuISD::MFHI", SDT_MFLOHI>;
def FgpuMFLO : SDNode<"FgpuISD::MFLO", SDT_MFLOHI>;

// Node used to insert 32-bit integers to LOHI register pair.
def FgpuMTLOHI : SDNode<"FgpuISD::MTLOHI", SDT_MTLOHI>;

// Mult nodes.
def FgpuMult  : SDNode<"FgpuISD::Mult", SDT_FgpuMultDiv>;
def FgpuMultu : SDNode<"FgpuISD::Multu", SDT_FgpuMultDiv>;

// MAdd*/MSub* nodes
def FgpuMAdd  : SDNode<"FgpuISD::MAdd", SDT_FgpuMAddMSub>;
def FgpuMAddu : SDNode<"FgpuISD::MAddu", SDT_FgpuMAddMSub>;
def FgpuMSub  : SDNode<"FgpuISD::MSub", SDT_FgpuMAddMSub>;
def FgpuMSubu : SDNode<"FgpuISD::MSubu", SDT_FgpuMAddMSub>;

// DivRem(u) nodes
def FgpuDivRem    : SDNode<"FgpuISD::DivRem", SDT_FgpuMultDiv>;
def FgpuDivRemU   : SDNode<"FgpuISD::DivRemU", SDT_FgpuMultDiv>;
def FgpuDivRem16  : SDNode<"FgpuISD::DivRem16", SDT_FgpuDivRem16,
                           [SDNPOutGlue]>;
def FgpuDivRemU16 : SDNode<"FgpuISD::DivRemU16", SDT_FgpuDivRem16,
                           [SDNPOutGlue]>;

// Target constant nodes that are not part of any isel patterns and remain
// unchanged can cause instructions with illegal operands to be emitted.
// Wrapper node patterns give the instruction selector a chance to replace
// target constant nodes that would otherwise remain unchanged with ADDiu
// nodes. Without these wrapper node patterns, the following conditional move
// instruction is emitted when function cmov2 in test/CodeGen/Fgpu/cmov.ll is
// compiled:
//  movn  %got(d)($gp), %got(c)($gp), $4
// This instruction is illegal since movn can take only register operands.

def FgpuWrapper    : SDNode<"FgpuISD::Wrapper", SDTIntBinOp>;

def FgpuSync : SDNode<"FgpuISD::Sync", SDT_Sync, [SDNPHasChain,SDNPSideEffect]>;

def FgpuExt :  SDNode<"FgpuISD::Ext", SDT_Ext>;
def FgpuIns :  SDNode<"FgpuISD::Ins", SDT_Ins>;
def FgpuCIns : SDNode<"FgpuISD::CIns", SDT_Ext>;

def FgpuLWL : SDNode<"FgpuISD::LWL", SDTFgpuLoadLR,
                     [SDNPHasChain, SDNPMayLoad, SDNPMemOperand]>;
def FgpuLWR : SDNode<"FgpuISD::LWR", SDTFgpuLoadLR,
                     [SDNPHasChain, SDNPMayLoad, SDNPMemOperand]>;
def FgpuSWL : SDNode<"FgpuISD::SWL", SDTStore,
                     [SDNPHasChain, SDNPMayStore, SDNPMemOperand]>;
def FgpuSWR : SDNode<"FgpuISD::SWR", SDTStore,
                     [SDNPHasChain, SDNPMayStore, SDNPMemOperand]>;
def FgpuLDL : SDNode<"FgpuISD::LDL", SDTFgpuLoadLR,
                     [SDNPHasChain, SDNPMayLoad, SDNPMemOperand]>;
def FgpuLDR : SDNode<"FgpuISD::LDR", SDTFgpuLoadLR,
                     [SDNPHasChain, SDNPMayLoad, SDNPMemOperand]>;
def FgpuSDL : SDNode<"FgpuISD::SDL", SDTStore,
                     [SDNPHasChain, SDNPMayStore, SDNPMemOperand]>;
def FgpuSDR : SDNode<"FgpuISD::SDR", SDTStore,
                     [SDNPHasChain, SDNPMayStore, SDNPMemOperand]>;

//===----------------------------------------------------------------------===//
// Fgpu Instruction Predicate Definitions.
//===----------------------------------------------------------------------===//
def HasFgpu2     :    Predicate<"Subtarget->hasFgpu2()">,
                      AssemblerPredicate<(all_of FeatureFgpu2)>;
def HasFgpu3_32  :    Predicate<"Subtarget->hasFgpu3_32()">,
                      AssemblerPredicate<(all_of FeatureFgpu3_32)>;
def HasFgpu3_32r2 :   Predicate<"Subtarget->hasFgpu3_32r2()">,
                      AssemblerPredicate<(all_of FeatureFgpu3_32r2)>;
def HasFgpu3     :    Predicate<"Subtarget->hasFgpu3()">,
                      AssemblerPredicate<(all_of FeatureFgpu3)>;
def NotFgpu3     :    Predicate<"!Subtarget->hasFgpu3()">,
                      AssemblerPredicate<(all_of (not FeatureFgpu3))>;
def HasFgpu4_32  :    Predicate<"Subtarget->hasFgpu4_32()">,
                      AssemblerPredicate<(all_of FeatureFgpu4_32)>;
def NotFgpu4_32  :    Predicate<"!Subtarget->hasFgpu4_32()">,
                      AssemblerPredicate<(all_of (not FeatureFgpu4_32))>;
def HasFgpu4_32r2 :   Predicate<"Subtarget->hasFgpu4_32r2()">,
                      AssemblerPredicate<(all_of FeatureFgpu4_32r2)>;
def HasFgpu5_32r2 :   Predicate<"Subtarget->hasFgpu5_32r2()">,
                      AssemblerPredicate<(all_of FeatureFgpu5_32r2)>;
def HasFgpu32    :    Predicate<"Subtarget->hasFgpu32()">,
                      AssemblerPredicate<(all_of FeatureFgpu32)>;
def HasFgpu32r2  :    Predicate<"Subtarget->hasFgpu32r2()">,
                      AssemblerPredicate<(all_of FeatureFgpu32r2)>;
def HasFgpu32r5  :    Predicate<"Subtarget->hasFgpu32r5()">,
                      AssemblerPredicate<(all_of FeatureFgpu32r5)>;
def HasFgpu32r6  :    Predicate<"Subtarget->hasFgpu32r6()">,
                      AssemblerPredicate<(all_of FeatureFgpu32r6)>;
def NotFgpu32r6  :    Predicate<"!Subtarget->hasFgpu32r6()">,
                      AssemblerPredicate<(all_of (not FeatureFgpu32r6))>;
def IsGP64bit    :    Predicate<"Subtarget->isGP64bit()">,
                      AssemblerPredicate<(all_of FeatureGP64Bit)>;
def IsGP32bit    :    Predicate<"!Subtarget->isGP64bit()">,
                      AssemblerPredicate<(all_of (not FeatureGP64Bit))>;
def IsPTR64bit    :   Predicate<"Subtarget->isABI_N64()">,
                      AssemblerPredicate<(all_of FeaturePTR64Bit)>;
def IsPTR32bit    :   Predicate<"!Subtarget->isABI_N64()">,
                      AssemblerPredicate<(all_of (not FeaturePTR64Bit))>;
def HasFgpu64    :    Predicate<"Subtarget->hasFgpu64()">,
                      AssemblerPredicate<(all_of FeatureFgpu64)>;
def NotFgpu64    :    Predicate<"!Subtarget->hasFgpu64()">,
                      AssemblerPredicate<(all_of (not FeatureFgpu64))>;
def HasFgpu64r2  :    Predicate<"Subtarget->hasFgpu64r2()">,
                      AssemblerPredicate<(all_of FeatureFgpu64r2)>;
def HasFgpu64r5  :    Predicate<"Subtarget->hasFgpu64r5()">,
                      AssemblerPredicate<(all_of FeatureFgpu64r5)>;
def HasFgpu64r6  :    Predicate<"Subtarget->hasFgpu64r6()">,
                      AssemblerPredicate<(all_of FeatureFgpu64r6)>;
def NotFgpu64r6  :    Predicate<"!Subtarget->hasFgpu64r6()">,
                      AssemblerPredicate<(all_of (not FeatureFgpu64r6))>;
def InFgpu16Mode :    Predicate<"Subtarget->inFgpu16Mode()">,
                      AssemblerPredicate<(all_of FeatureFgpu16)>;
def NotInFgpu16Mode : Predicate<"!Subtarget->inFgpu16Mode()">,
                      AssemblerPredicate<(all_of (not FeatureFgpu16))>;
def HasCnFgpu    :    Predicate<"Subtarget->hasCnFgpu()">,
                      AssemblerPredicate<(all_of FeatureCnFgpu)>;
def NotCnFgpu    :    Predicate<"!Subtarget->hasCnFgpu()">,
                      AssemblerPredicate<(all_of (not FeatureCnFgpu))>;
def HasCnFgpuP   :    Predicate<"Subtarget->hasCnFgpuP()">,
                      AssemblerPredicate<(all_of FeatureCnFgpuP)>;
def NotCnFgpuP   :    Predicate<"!Subtarget->hasCnFgpuP()">,
                      AssemblerPredicate<(all_of (not FeatureCnFgpuP))>;
def IsSym32     :     Predicate<"Subtarget->hasSym32()">,
                      AssemblerPredicate<(all_of FeatureSym32)>;
def IsSym64     :     Predicate<"!Subtarget->hasSym32()">,
                      AssemblerPredicate<(all_of (not FeatureSym32))>;
def IsN64       :     Predicate<"Subtarget->isABI_N64()">;
def IsNotN64    :     Predicate<"!Subtarget->isABI_N64()">;
def RelocNotPIC :     Predicate<"!TM.isPositionIndependent()">;
def RelocPIC    :     Predicate<"TM.isPositionIndependent()">;
def NoNaNsFPMath :    Predicate<"TM.Options.NoNaNsFPMath">;
def UseAbs :          Predicate<"Subtarget->inAbs2008Mode() ||"
                                "TM.Options.NoNaNsFPMath">;
def HasStdEnc :       Predicate<"Subtarget->hasStandardEncoding()">,
                      AssemblerPredicate<(all_of (not FeatureFgpu16))>;
def NotDSP :          Predicate<"!Subtarget->hasDSP()">;
def InMicroFgpu    :  Predicate<"Subtarget->inMicroFgpuMode()">,
                      AssemblerPredicate<(all_of FeatureMicroFgpu)>;
def NotInMicroFgpu :  Predicate<"!Subtarget->inMicroFgpuMode()">,
                      AssemblerPredicate<(all_of (not FeatureMicroFgpu))>;
def IsLE           :  Predicate<"Subtarget->isLittle()">;
def IsBE           :  Predicate<"!Subtarget->isLittle()">;
def IsNotNaCl    :    Predicate<"!Subtarget->isTargetNaCl()">;
def UseTCCInDIV    :  AssemblerPredicate<(all_of FeatureUseTCCInDIV)>;
def HasEVA       :    Predicate<"Subtarget->hasEVA()">,
                      AssemblerPredicate<(all_of FeatureEVA)>;
def HasMSA : Predicate<"Subtarget->hasMSA()">,
             AssemblerPredicate<(all_of FeatureMSA)>;
def HasMadd4 : Predicate<"!Subtarget->disableMadd4()">,
               AssemblerPredicate<(all_of (not FeatureNoMadd4))>;
def HasMT  : Predicate<"Subtarget->hasMT()">,
             AssemblerPredicate<(all_of FeatureMT)>;
def UseIndirectJumpsHazard : Predicate<"Subtarget->useIndirectJumpsHazard()">,
                            AssemblerPredicate<(all_of FeatureUseIndirectJumpsHazard)>;
def NoIndirectJumpGuards : Predicate<"!Subtarget->useIndirectJumpsHazard()">,
                           AssemblerPredicate<(all_of (not FeatureUseIndirectJumpsHazard))>;
def HasCRC   : Predicate<"Subtarget->hasCRC()">,
               AssemblerPredicate<(all_of FeatureCRC)>;
def HasVirt  : Predicate<"Subtarget->hasVirt()">,
               AssemblerPredicate<(all_of FeatureVirt)>;
def HasGINV  : Predicate<"Subtarget->hasGINV()">,
               AssemblerPredicate<(all_of FeatureGINV)>;
// TODO: Add support for FPOpFusion::Standard
def AllowFPOpFusion : Predicate<"TM.Options.AllowFPOpFusion =="
                                " FPOpFusion::Fast">;
//===----------------------------------------------------------------------===//
// Fgpu GPR size adjectives.
// They are mutually exclusive.
//===----------------------------------------------------------------------===//

class GPR_32 { list<Predicate> GPRPredicates = [IsGP32bit]; }
class GPR_64 { list<Predicate> GPRPredicates = [IsGP64bit]; }

class PTR_32 { list<Predicate> PTRPredicates = [IsPTR32bit]; }
class PTR_64 { list<Predicate> PTRPredicates = [IsPTR64bit]; }

//===----------------------------------------------------------------------===//
// Fgpu Symbol size adjectives.
// They are mutally exculsive.
//===----------------------------------------------------------------------===//

class SYM_32 { list<Predicate> SYMPredicates = [IsSym32]; }
class SYM_64 { list<Predicate> SYMPredicates = [IsSym64]; }

//===----------------------------------------------------------------------===//
// Fgpu ISA/ASE membership and instruction group membership adjectives.
// They are mutually exclusive.
//===----------------------------------------------------------------------===//

// FIXME: I'd prefer to use additive predicates to build the instruction sets
//        but we are short on assembler feature bits at the moment. Using a
//        subtractive predicate will hopefully keep us under the 32 predicate
//        limit long enough to develop an alternative way to handle P1||P2
//        predicates.
class ISA_FGPU1 {
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU1_NOT_FGPU3 {
  list<Predicate> InsnPredicates = [NotFgpu3];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU1_NOT_4_32 {
  list<Predicate> InsnPredicates = [NotFgpu4_32];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU1_NOT_32R6_64R6 {
  list<Predicate> InsnPredicates = [NotFgpu32r6, NotFgpu64r6];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU2 {
  list<Predicate> InsnPredicates = [HasFgpu2];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU2_NOT_32R6_64R6 {
  list<Predicate> InsnPredicates = [HasFgpu2, NotFgpu32r6, NotFgpu64r6];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU3 {
  list<Predicate> InsnPredicates = [HasFgpu3];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU3_NOT_32R6_64R6 {
  list<Predicate> InsnPredicates = [HasFgpu3, NotFgpu32r6, NotFgpu64r6];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU32 {
  list<Predicate> InsnPredicates = [HasFgpu32];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU32_NOT_32R6_64R6 {
  list<Predicate> InsnPredicates = [HasFgpu32, NotFgpu32r6, NotFgpu64r6];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU32R2 {
  list<Predicate> InsnPredicates = [HasFgpu32r2];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU32R2_NOT_32R6_64R6 {
  list<Predicate> InsnPredicates = [HasFgpu32r2, NotFgpu32r6, NotFgpu64r6];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU32R5 {
  list<Predicate> InsnPredicates = [HasFgpu32r5];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU64 {
  list<Predicate> InsnPredicates = [HasFgpu64];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU64_NOT_64R6 {
  list<Predicate> InsnPredicates = [HasFgpu64, NotFgpu64r6];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU64R2 {
  list<Predicate> InsnPredicates = [HasFgpu64r2];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU64R5 {
  list<Predicate> InsnPredicates = [HasFgpu64r5];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU32R6 {
  list<Predicate> InsnPredicates = [HasFgpu32r6];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_FGPU64R6 {
  list<Predicate> InsnPredicates = [HasFgpu64r6];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}
class ISA_MICROFGPU {
  list<Predicate> EncodingPredicates = [InMicroFgpu];
}
class ISA_MICROFGPU32R5 {
  list<Predicate> InsnPredicates = [HasFgpu32r5];
  list<Predicate> EncodingPredicates = [InMicroFgpu];
}
class ISA_MICROFGPU32R6 {
  list<Predicate> InsnPredicates = [HasFgpu32r6];
  list<Predicate> EncodingPredicates = [InMicroFgpu];
}
class ISA_MICROFGPU64R6 {
  list<Predicate> InsnPredicates = [HasFgpu64r6];
  list<Predicate> EncodingPredicates = [InMicroFgpu];
}
class ISA_MICROFGPU32_NOT_FGPU32R6 {
  list<Predicate> InsnPredicates = [NotFgpu32r6];
  list<Predicate> EncodingPredicates = [InMicroFgpu];
}
class ASE_EVA { list<Predicate> ASEPredicate = [HasEVA]; }

// The portions of FGPU-III that were also added to FGPU32
class INSN_FGPU3_32 {
  list<Predicate> InsnPredicates = [HasFgpu3_32];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}

// The portions of FGPU-III that were also added to FGPU32 but were removed in
// FGPU32r6 and FGPU64r6.
class INSN_FGPU3_32_NOT_32R6_64R6 {
  list<Predicate> InsnPredicates = [HasFgpu3_32, NotFgpu32r6, NotFgpu64r6];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}

// The portions of FGPU-III that were also added to FGPU32
class INSN_FGPU3_32R2 {
  list<Predicate> InsnPredicates = [HasFgpu3_32r2];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}

// The portions of FGPU-IV that were also added to FGPU32.
class INSN_FGPU4_32 {
  list <Predicate> InsnPredicates = [HasFgpu4_32];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}

// The portions of FGPU-IV that were also added to FGPU32 but were removed in
// FGPU32r6 and FGPU64r6.
class INSN_FGPU4_32_NOT_32R6_64R6 {
  list<Predicate> InsnPredicates = [HasFgpu4_32, NotFgpu32r6, NotFgpu64r6];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}

// The portions of FGPU-IV that were also added to FGPU32r2 but were removed in
// FGPU32r6 and FGPU64r6.
class INSN_FGPU4_32R2_NOT_32R6_64R6 {
  list<Predicate> InsnPredicates = [HasFgpu4_32r2, NotFgpu32r6, NotFgpu64r6];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}

// The portions of FGPU-IV that were also added to FGPU32r2.
class INSN_FGPU4_32R2 {
  list<Predicate> InsnPredicates = [HasFgpu4_32r2];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}

// The portions of FGPU-V that were also added to FGPU32r2 but were removed in
// FGPU32r6 and FGPU64r6.
class INSN_FGPU5_32R2_NOT_32R6_64R6 {
  list<Predicate> InsnPredicates = [HasFgpu5_32r2, NotFgpu32r6, NotFgpu64r6];
  list<Predicate> EncodingPredicates = [HasStdEnc];
}

class ASE_CNFGPU {
  list<Predicate> ASEPredicate = [HasCnFgpu];
}

class NOT_ASE_CNFGPU {
  list<Predicate> ASEPredicate = [NotCnFgpu];
}

class ASE_CNFGPUP {
  list<Predicate> ASEPredicate = [HasCnFgpuP];
}

class NOT_ASE_CNFGPUP {
  list<Predicate> ASEPredicate = [NotCnFgpuP];
}

class ASE_FGPU64_CNFGPU {
  list<Predicate> ASEPredicate = [HasFgpu64, HasCnFgpu];
}

class ASE_MSA {
  list<Predicate> ASEPredicate = [HasMSA];
}

class ASE_MSA_NOT_MSA64 {
  list<Predicate> ASEPredicate = [HasMSA, NotFgpu64];
}

class ASE_MSA64 {
  list<Predicate> ASEPredicate = [HasMSA, HasFgpu64];
}

class ASE_MT {
  list <Predicate> ASEPredicate = [HasMT];
}

class ASE_CRC {
  list <Predicate> ASEPredicate = [HasCRC];
}

class ASE_VIRT {
  list <Predicate> ASEPredicate = [HasVirt];
}

class ASE_GINV {
  list <Predicate> ASEPredicate = [HasGINV];
}

// Class used for separating microFGPUr6 and microFGPU (r3) instruction.
// It can be used only on instructions that doesn't inherit PredicateControl.
class ISA_MICROFGPU_NOT_32R6 : PredicateControl {
  let InsnPredicates = [NotFgpu32r6];
  let EncodingPredicates = [InMicroFgpu];
}

class ASE_NOT_DSP {
  list<Predicate> ASEPredicate = [NotDSP];
}

class MADD4 {
  list<Predicate> AdditionalPredicates = [HasMadd4];
}

// Classes used for separating expansions that differ based on the ABI in
// use.
class ABI_N64 {
  list<Predicate> AdditionalPredicates = [IsN64];
}

class ABI_NOT_N64 {
  list<Predicate> AdditionalPredicates = [IsNotN64];
}

class FPOP_FUSION_FAST {
  list <Predicate> AdditionalPredicates = [AllowFPOpFusion];
}

//===----------------------------------------------------------------------===//

class FgpuPat<dag pattern, dag result> : Pat<pattern, result>, PredicateControl;

class FgpuInstAlias<string Asm, dag Result, bit Emit = 0b1> :
  InstAlias<Asm, Result, Emit>, PredicateControl;

class IsCommutable {
  bit isCommutable = 1;
}

class IsBranch {
  bit isBranch = 1;
  bit isCTI = 1;
}

class IsReturn {
  bit isReturn = 1;
  bit isCTI = 1;
}

class IsCall {
  bit isCall = 1;
  bit isCTI = 1;
}

class IsTailCall {
  bit isCall = 1;
  bit isTerminator = 1;
  bit isReturn = 1;
  bit isBarrier = 1;
  bit hasExtraSrcRegAllocReq = 1;
  bit isCodeGenOnly = 1;
  bit isCTI = 1;
}

class IsAsCheapAsAMove {
  bit isAsCheapAsAMove = 1;
}

class NeverHasSideEffects {
  bit hasSideEffects = 0;
}

//===----------------------------------------------------------------------===//
// Instruction format superclass
//===----------------------------------------------------------------------===//

include "FgpuInstrFormats.td"

//===----------------------------------------------------------------------===//
// Fgpu Operand, Complex Patterns and Transformations Definitions.
//===----------------------------------------------------------------------===//

class ConstantSImmAsmOperandClass<int Bits, list<AsmOperandClass> Supers = [],
                                  int Offset = 0> : AsmOperandClass {
  let Name = "ConstantSImm" # Bits # "_" # Offset;
  let RenderMethod = "addConstantSImmOperands<" # Bits # ", " # Offset # ">";
  let PredicateMethod = "isConstantSImm<" # Bits # ", " # Offset # ">";
  let SuperClasses = Supers;
  let DiagnosticType = "SImm" # Bits # "_" # Offset;
}

class SimmLslAsmOperandClass<int Bits, list<AsmOperandClass> Supers = [],
                                  int Shift = 0> : AsmOperandClass {
  let Name = "Simm" # Bits # "_Lsl" # Shift;
  let RenderMethod = "addImmOperands";
  let PredicateMethod = "isScaledSImm<" # Bits # ", " # Shift # ">";
  let SuperClasses = Supers;
  let DiagnosticType = "SImm" # Bits # "_Lsl" # Shift;
}

class ConstantUImmAsmOperandClass<int Bits, list<AsmOperandClass> Supers = [],
                                  int Offset = 0> : AsmOperandClass {
  let Name = "ConstantUImm" # Bits # "_" # Offset;
  let RenderMethod = "addConstantUImmOperands<" # Bits # ", " # Offset # ">";
  let PredicateMethod = "isConstantUImm<" # Bits # ", " # Offset # ">";
  let SuperClasses = Supers;
  let DiagnosticType = "UImm" # Bits # "_" # Offset;
}

class ConstantUImmRangeAsmOperandClass<int Bottom, int Top,
                                       list<AsmOperandClass> Supers = []>
    : AsmOperandClass {
  let Name = "ConstantUImmRange" # Bottom # "_" # Top;
  let RenderMethod = "addImmOperands";
  let PredicateMethod = "isConstantUImmRange<" # Bottom # ", " # Top # ">";
  let SuperClasses = Supers;
  let DiagnosticType = "UImmRange" # Bottom # "_" # Top;
}

class SImmAsmOperandClass<int Bits, list<AsmOperandClass> Supers = []>
    : AsmOperandClass {
  let Name = "SImm" # Bits;
  let RenderMethod = "addSImmOperands<" # Bits # ">";
  let PredicateMethod = "isSImm<" # Bits # ">";
  let SuperClasses = Supers;
  let DiagnosticType = "SImm" # Bits;
}

class UImmAsmOperandClass<int Bits, list<AsmOperandClass> Supers = []>
    : AsmOperandClass {
  let Name = "UImm" # Bits;
  let RenderMethod = "addUImmOperands<" # Bits # ">";
  let PredicateMethod = "isUImm<" # Bits # ">";
  let SuperClasses = Supers;
  let DiagnosticType = "UImm" # Bits;
}

// Generic case - only to support certain assembly pseudo instructions.
class UImmAnyAsmOperandClass<int Bits, list<AsmOperandClass> Supers = []>
    : AsmOperandClass {
  let Name = "ImmAny";
  let RenderMethod = "addConstantUImmOperands<32>";
  let PredicateMethod = "isSImm<" # Bits # ">";
  let SuperClasses = Supers;
  let DiagnosticType = "ImmAny";
}

// AsmOperandClasses require a strict ordering which is difficult to manage
// as a hierarchy. Instead, we use a linear ordering and impose an order that
// is in some places arbitrary.
//
// Here the rules that are in use:
// * Wider immediates are a superset of narrower immediates:
//     uimm4 < uimm5 < uimm6
// * For the same bit-width, unsigned immediates are a superset of signed
//   immediates::
//     simm4 < uimm4 < simm5 < uimm5
// * For the same upper-bound, signed immediates are a superset of unsigned
//   immediates:
//     uimm3 < simm4 < uimm4 < simm4
// * Modified immediates are a superset of ordinary immediates:
//     uimm5 < uimm5_plus1 (1..32) < uimm5_plus32 (32..63) < uimm6
//   The term 'superset' starts to break down here since the uimm5_plus* classes
//   are not true supersets of uimm5 (but they are still subsets of uimm6).
// * 'Relaxed' immediates are supersets of the corresponding unsigned immediate.
//     uimm16 < uimm16_relaxed
// * The codeGen pattern type is arbitrarily ordered.
//     uimm5 < uimm5_64, and uimm5 < vsplat_uimm5
//   This is entirely arbitrary. We need an ordering and what we pick is
//   unimportant since only one is possible for a given mnemonic.

def UImm32CoercedAsmOperandClass : UImmAnyAsmOperandClass<33, []> {
  let Name = "UImm32_Coerced";
  let DiagnosticType = "UImm32_Coerced";
}
def SImm32RelaxedAsmOperandClass
    : SImmAsmOperandClass<32, [UImm32CoercedAsmOperandClass]> {
  let Name = "SImm32_Relaxed";
  let PredicateMethod = "isAnyImm<33>";
  let DiagnosticType = "SImm32_Relaxed";
}
def SImm32AsmOperandClass
    : SImmAsmOperandClass<32, [SImm32RelaxedAsmOperandClass]>;
def ConstantUImm26AsmOperandClass
    : ConstantUImmAsmOperandClass<26, [SImm32AsmOperandClass]>;
def ConstantUImm20AsmOperandClass
    : ConstantUImmAsmOperandClass<20, [ConstantUImm26AsmOperandClass]>;
def ConstantSImm19Lsl2AsmOperandClass : AsmOperandClass {
  let Name = "SImm19Lsl2";
  let RenderMethod = "addImmOperands";
  let PredicateMethod = "isScaledSImm<19, 2>";
  let SuperClasses = [ConstantUImm20AsmOperandClass];
  let DiagnosticType = "SImm19_Lsl2";
}
def UImm16RelaxedAsmOperandClass
    : UImmAsmOperandClass<16, [ConstantUImm20AsmOperandClass]> {
  let Name = "UImm16_Relaxed";
  let PredicateMethod = "isAnyImm<16>";
  let DiagnosticType = "UImm16_Relaxed";
}
// Similar to the relaxed classes which take an SImm and render it as
// an UImm, this takes a UImm and renders it as an SImm.
def UImm16AltRelaxedAsmOperandClass
    : SImmAsmOperandClass<16, [UImm16RelaxedAsmOperandClass]> {
  let Name = "UImm16_AltRelaxed";
  let PredicateMethod = "isUImm<16>";
  let DiagnosticType = "UImm16_AltRelaxed";
}
// FIXME: One of these should probably have UImm16AsmOperandClass as the
//        superclass instead of UImm16RelaxedasmOPerandClass.
def UImm16AsmOperandClass
    : UImmAsmOperandClass<16, [UImm16RelaxedAsmOperandClass]>;
def SImm16RelaxedAsmOperandClass
    : SImmAsmOperandClass<16, [UImm16RelaxedAsmOperandClass]> {
  let Name = "SImm16_Relaxed";
  let PredicateMethod = "isAnyImm<16>";
  let DiagnosticType = "SImm16_Relaxed";
}
def SImm16AsmOperandClass
    : SImmAsmOperandClass<16, [SImm16RelaxedAsmOperandClass]>;
def ConstantSImm10Lsl3AsmOperandClass : AsmOperandClass {
  let Name = "SImm10Lsl3";
  let RenderMethod = "addImmOperands";
  let PredicateMethod = "isScaledSImm<10, 3>";
  let SuperClasses = [SImm16AsmOperandClass];
  let DiagnosticType = "SImm10_Lsl3";
}
def ConstantSImm10Lsl2AsmOperandClass : AsmOperandClass {
  let Name = "SImm10Lsl2";
  let RenderMethod = "addImmOperands";
  let PredicateMethod = "isScaledSImm<10, 2>";
  let SuperClasses = [ConstantSImm10Lsl3AsmOperandClass];
  let DiagnosticType = "SImm10_Lsl2";
}
def ConstantSImm11AsmOperandClass
    : ConstantSImmAsmOperandClass<11, [ConstantSImm10Lsl2AsmOperandClass]>;
def ConstantSImm10Lsl1AsmOperandClass : AsmOperandClass {
  let Name = "SImm10Lsl1";
  let RenderMethod = "addImmOperands";
  let PredicateMethod = "isScaledSImm<10, 1>";
  let SuperClasses = [ConstantSImm11AsmOperandClass];
  let DiagnosticType = "SImm10_Lsl1";
}
def ConstantUImm10AsmOperandClass
    : ConstantUImmAsmOperandClass<10, [ConstantSImm10Lsl1AsmOperandClass]>;
def ConstantSImm10AsmOperandClass
    : ConstantSImmAsmOperandClass<10, [ConstantUImm10AsmOperandClass]>;
def ConstantSImm9AsmOperandClass
    : ConstantSImmAsmOperandClass<9, [ConstantSImm10AsmOperandClass]>;
def ConstantSImm7Lsl2AsmOperandClass : AsmOperandClass {
  let Name = "SImm7Lsl2";
  let RenderMethod = "addImmOperands";
  let PredicateMethod = "isScaledSImm<7, 2>";
  let SuperClasses = [ConstantSImm9AsmOperandClass];
  let DiagnosticType = "SImm7_Lsl2";
}
def ConstantUImm8AsmOperandClass
    : ConstantUImmAsmOperandClass<8, [ConstantSImm7Lsl2AsmOperandClass]>;
def ConstantUImm7Sub1AsmOperandClass
    : ConstantUImmAsmOperandClass<7, [ConstantUImm8AsmOperandClass], -1> {
  // Specify the names since the -1 offset causes invalid identifiers otherwise.
  let Name = "UImm7_N1";
  let DiagnosticType = "UImm7_N1";
}
def ConstantUImm7AsmOperandClass
    : ConstantUImmAsmOperandClass<7, [ConstantUImm7Sub1AsmOperandClass]>;
def ConstantUImm6Lsl2AsmOperandClass : AsmOperandClass {
  let Name = "UImm6Lsl2";
  let RenderMethod = "addImmOperands";
  let PredicateMethod = "isScaledUImm<6, 2>";
  let SuperClasses = [ConstantUImm7AsmOperandClass];
  let DiagnosticType = "UImm6_Lsl2";
}
def ConstantUImm6AsmOperandClass
    : ConstantUImmAsmOperandClass<6, [ConstantUImm6Lsl2AsmOperandClass]>;
def ConstantSImm6AsmOperandClass
    : ConstantSImmAsmOperandClass<6, [ConstantUImm6AsmOperandClass]>;
def ConstantUImm5Lsl2AsmOperandClass : AsmOperandClass {
  let Name = "UImm5Lsl2";
  let RenderMethod = "addImmOperands";
  let PredicateMethod = "isScaledUImm<5, 2>";
  let SuperClasses = [ConstantSImm6AsmOperandClass];
  let DiagnosticType = "UImm5_Lsl2";
}
def ConstantUImm5_Range2_64AsmOperandClass
    : ConstantUImmRangeAsmOperandClass<2, 64, [ConstantUImm5Lsl2AsmOperandClass]>;
def ConstantUImm5Plus33AsmOperandClass
    : ConstantUImmAsmOperandClass<5, [ConstantUImm5_Range2_64AsmOperandClass],
                                  33>;
def ConstantUImm5ReportUImm6AsmOperandClass
    : ConstantUImmAsmOperandClass<5, [ConstantUImm5Plus33AsmOperandClass]> {
  let Name = "ConstantUImm5_0_Report_UImm6";
  let DiagnosticType = "UImm5_0_Report_UImm6";
}
def ConstantUImm5Plus32AsmOperandClass
    : ConstantUImmAsmOperandClass<
          5, [ConstantUImm5ReportUImm6AsmOperandClass], 32>;
def ConstantUImm5Plus32NormalizeAsmOperandClass
    : ConstantUImmAsmOperandClass<5, [ConstantUImm5Plus32AsmOperandClass], 32> {
  let Name = "ConstantUImm5_32_Norm";
  // We must also subtract 32 when we render the operand.
  let RenderMethod = "addConstantUImmOperands<5, 32, -32>";
}
def ConstantUImm5Plus1ReportUImm6AsmOperandClass
    : ConstantUImmAsmOperandClass<
          5, [ConstantUImm5Plus32NormalizeAsmOperandClass], 1>{
  let Name = "ConstantUImm5_Plus1_Report_UImm6";
}
def ConstantUImm5Plus1AsmOperandClass
    : ConstantUImmAsmOperandClass<
          5, [ConstantUImm5Plus1ReportUImm6AsmOperandClass], 1>;
def ConstantUImm5AsmOperandClass
    : ConstantUImmAsmOperandClass<5, [ConstantUImm5Plus1AsmOperandClass]>;
def ConstantSImm5AsmOperandClass
    : ConstantSImmAsmOperandClass<5, [ConstantUImm5AsmOperandClass]>;
def ConstantUImm4AsmOperandClass
    : ConstantUImmAsmOperandClass<4, [ConstantSImm5AsmOperandClass]>;
def ConstantSImm4AsmOperandClass
    : ConstantSImmAsmOperandClass<4, [ConstantUImm4AsmOperandClass]>;
def ConstantUImm3AsmOperandClass
    : ConstantUImmAsmOperandClass<3, [ConstantSImm4AsmOperandClass]>;
def ConstantUImm2Plus1AsmOperandClass
    : ConstantUImmAsmOperandClass<2, [ConstantUImm3AsmOperandClass], 1>;
def ConstantUImm2AsmOperandClass
    : ConstantUImmAsmOperandClass<2, [ConstantUImm3AsmOperandClass]>;
def ConstantUImm1AsmOperandClass
    : ConstantUImmAsmOperandClass<1, [ConstantUImm2AsmOperandClass]>;
def ConstantImmzAsmOperandClass : AsmOperandClass {
  let Name = "ConstantImmz";
  let RenderMethod = "addConstantUImmOperands<1>";
  let PredicateMethod = "isConstantImmz";
  let SuperClasses = [ConstantUImm1AsmOperandClass];
  let DiagnosticType = "Immz";
}

def Simm19Lsl2AsmOperand
    : SimmLslAsmOperandClass<19, [], 2>;

def FgpuJumpTargetAsmOperand : AsmOperandClass {
  let Name = "JumpTarget";
  let ParserMethod = "parseJumpTarget";
  let PredicateMethod = "isImm";
  let RenderMethod = "addImmOperands";
}

// Instruction operand types
def jmptarget   : Operand<OtherVT> {
  let EncoderMethod = "getJumpTargetOpValue";
  let ParserMatchClass = FgpuJumpTargetAsmOperand;
}
def brtarget    : Operand<OtherVT> {
  let EncoderMethod = "getBranchTargetOpValue";
  let OperandType = "OPERAND_PCREL";
  let DecoderMethod = "DecodeBranchTarget";
  let ParserMatchClass = FgpuJumpTargetAsmOperand;
}
def brtarget1SImm16 : Operand<OtherVT> {
  let EncoderMethod = "getBranchTargetOpValue1SImm16";
  let OperandType = "OPERAND_PCREL";
  let DecoderMethod = "DecodeBranchTarget1SImm16";
  let ParserMatchClass = FgpuJumpTargetAsmOperand;
}
def calltarget  : Operand<iPTR> {
  let EncoderMethod = "getJumpTargetOpValue";
  let ParserMatchClass = FgpuJumpTargetAsmOperand;
}

def imm64: Operand<i64>;

def simm19_lsl2 : Operand<i32> {
  let EncoderMethod = "getSimm19Lsl2Encoding";
  let DecoderMethod = "DecodeSimm19Lsl2";
  let ParserMatchClass = Simm19Lsl2AsmOperand;
}

def simm18_lsl3 : Operand<i32> {
  let EncoderMethod = "getSimm18Lsl3Encoding";
  let DecoderMethod = "DecodeSimm18Lsl3";
  let ParserMatchClass = FgpuJumpTargetAsmOperand;
}

// Zero
def uimmz       : Operand<i32> {
  let PrintMethod = "printUImm<0>";
  let ParserMatchClass = ConstantImmzAsmOperandClass;
}

// size operand of ins instruction
def uimm_range_2_64 : Operand<i32> {
  let PrintMethod = "printUImm<6, 2>";
  let EncoderMethod = "getSizeInsEncoding";
  let DecoderMethod = "DecodeInsSize";
  let ParserMatchClass = ConstantUImm5_Range2_64AsmOperandClass;
}

// Unsigned Operands
foreach I = {1, 2, 3, 4, 5, 6, 7, 8, 10, 20, 26} in
  def uimm # I : Operand<i32> {
    let PrintMethod = "printUImm<" # I # ">";
    let ParserMatchClass =
        !cast<AsmOperandClass>("ConstantUImm" # I # "AsmOperandClass");
  }

def uimm2_plus1 : Operand<i32> {
  let PrintMethod = "printUImm<2, 1>";
  let EncoderMethod = "getUImmWithOffsetEncoding<2, 1>";
  let DecoderMethod = "DecodeUImmWithOffset<2, 1>";
  let ParserMatchClass = ConstantUImm2Plus1AsmOperandClass;
}

def uimm5_plus1 : Operand<i32> {
  let PrintMethod = "printUImm<5, 1>";
  let EncoderMethod = "getUImmWithOffsetEncoding<5, 1>";
  let DecoderMethod = "DecodeUImmWithOffset<5, 1>";
  let ParserMatchClass = ConstantUImm5Plus1AsmOperandClass;
}

def uimm5_plus1_report_uimm6 : Operand<i32> {
  let PrintMethod = "printUImm<6, 1>";
  let EncoderMethod = "getUImmWithOffsetEncoding<5, 1>";
  let DecoderMethod = "DecodeUImmWithOffset<5, 1>";
  let ParserMatchClass = ConstantUImm5Plus1ReportUImm6AsmOperandClass;
}

def uimm5_plus32 : Operand<i32> {
  let PrintMethod = "printUImm<5, 32>";
  let ParserMatchClass = ConstantUImm5Plus32AsmOperandClass;
}

def uimm5_plus33 : Operand<i32> {
  let PrintMethod = "printUImm<5, 33>";
  let EncoderMethod = "getUImmWithOffsetEncoding<5, 1>";
  let DecoderMethod = "DecodeUImmWithOffset<5, 1>";
  let ParserMatchClass = ConstantUImm5Plus33AsmOperandClass;
}

def uimm5_inssize_plus1 : Operand<i32> {
  let PrintMethod = "printUImm<6>";
  let ParserMatchClass = ConstantUImm5Plus1AsmOperandClass;
  let EncoderMethod = "getSizeInsEncoding";
  let DecoderMethod = "DecodeInsSize";
}

def uimm5_plus32_normalize : Operand<i32> {
  let PrintMethod = "printUImm<5>";
  let ParserMatchClass = ConstantUImm5Plus32NormalizeAsmOperandClass;
}

def uimm5_lsl2 : Operand<OtherVT> {
  let EncoderMethod = "getUImm5Lsl2Encoding";
  let DecoderMethod = "DecodeUImmWithOffsetAndScale<5, 0, 4>";
  let ParserMatchClass = ConstantUImm5Lsl2AsmOperandClass;
}

def uimm5_plus32_normalize_64 : Operand<i64> {
  let PrintMethod = "printUImm<5>";
  let ParserMatchClass = ConstantUImm5Plus32NormalizeAsmOperandClass;
}

def uimm6_lsl2 : Operand<OtherVT> {
  let EncoderMethod = "getUImm6Lsl2Encoding";
  let DecoderMethod = "DecodeUImmWithOffsetAndScale<6, 0, 4>";
  let ParserMatchClass = ConstantUImm6Lsl2AsmOperandClass;
}

foreach I = {16} in
  def uimm # I : Operand<i32> {
    let PrintMethod = "printUImm<" # I # ">";
    let ParserMatchClass =
        !cast<AsmOperandClass>("UImm" # I # "AsmOperandClass");
  }

// Like uimm16_64 but coerces simm16 to uimm16.
def uimm16_relaxed : Operand<i32> {
  let PrintMethod = "printUImm<16>";
  let ParserMatchClass = UImm16RelaxedAsmOperandClass;
}

foreach I = {5} in
  def uimm # I # _64 : Operand<i64> {
    let PrintMethod = "printUImm<" # I # ">";
    let ParserMatchClass =
        !cast<AsmOperandClass>("ConstantUImm" # I # "AsmOperandClass");
  }

foreach I = {16} in
  def uimm # I # _64 : Operand<i64> {
    let PrintMethod = "printUImm<" # I # ">";
    let ParserMatchClass =
        !cast<AsmOperandClass>("UImm" # I # "AsmOperandClass");
  }

// Like uimm16_64 but coerces simm16 to uimm16.
def uimm16_64_relaxed : Operand<i64> {
  let PrintMethod = "printUImm<16>";
  let ParserMatchClass = UImm16RelaxedAsmOperandClass;
}

def uimm16_altrelaxed : Operand<i32> {
  let PrintMethod = "printUImm<16>";
  let ParserMatchClass = UImm16AltRelaxedAsmOperandClass;
}
// Like uimm5 but reports a less confusing error for 32-63 when
// an instruction alias permits that.
def uimm5_report_uimm6 : Operand<i32> {
  let PrintMethod = "printUImm<6>";
  let ParserMatchClass = ConstantUImm5ReportUImm6AsmOperandClass;
}

// Like uimm5_64 but reports a less confusing error for 32-63 when
// an instruction alias permits that.
def uimm5_64_report_uimm6 : Operand<i64> {
  let PrintMethod = "printUImm<5>";
  let ParserMatchClass = ConstantUImm5ReportUImm6AsmOperandClass;
}

foreach I = {1, 2, 3, 4} in
  def uimm # I # _ptr : Operand<iPTR> {
    let PrintMethod = "printUImm<" # I # ">";
    let ParserMatchClass =
        !cast<AsmOperandClass>("ConstantUImm" # I # "AsmOperandClass");
  }

foreach I = {1, 2, 3, 4, 5, 6, 8} in
  def vsplat_uimm # I : Operand<vAny> {
    let PrintMethod = "printUImm<" # I # ">";
    let ParserMatchClass =
        !cast<AsmOperandClass>("ConstantUImm" # I # "AsmOperandClass");
  }

// Signed operands
foreach I = {4, 5, 6, 9, 10, 11} in
  def simm # I : Operand<i32> {
    let DecoderMethod = "DecodeSImmWithOffsetAndScale<" # I # ">";
    let ParserMatchClass =
        !cast<AsmOperandClass>("ConstantSImm" # I # "AsmOperandClass");
  }

foreach I = {1, 2, 3} in
  def simm10_lsl # I : Operand<i32> {
    let DecoderMethod = "DecodeSImmWithOffsetAndScale<10, " # I # ">";
    let ParserMatchClass =
        !cast<AsmOperandClass>("ConstantSImm10Lsl" # I # "AsmOperandClass");
  }

foreach I = {10} in
  def simm # I # _64 : Operand<i64> {
    let DecoderMethod = "DecodeSImmWithOffsetAndScale<" # I # ">";
    let ParserMatchClass =
        !cast<AsmOperandClass>("ConstantSImm" # I # "AsmOperandClass");
  }

foreach I = {5, 10} in
  def vsplat_simm # I : Operand<vAny> {
    let ParserMatchClass =
        !cast<AsmOperandClass>("ConstantSImm" # I # "AsmOperandClass");
  }

def simm7_lsl2 : Operand<OtherVT> {
  let EncoderMethod = "getSImm7Lsl2Encoding";
  let DecoderMethod = "DecodeSImmWithOffsetAndScale<" # I # ", 0, 4>";
  let ParserMatchClass = ConstantSImm7Lsl2AsmOperandClass;
}

foreach I = {16, 32} in
  def simm # I : Operand<i32> {
    let DecoderMethod = "DecodeSImmWithOffsetAndScale<" # I # ">";
    let ParserMatchClass = !cast<AsmOperandClass>("SImm" # I # "AsmOperandClass");
  }

// Like simm16 but coerces uimm16 to simm16.
def simm16_relaxed : Operand<i32> {
  let DecoderMethod = "DecodeSImmWithOffsetAndScale<16>";
  let ParserMatchClass = SImm16RelaxedAsmOperandClass;
}

def simm16_64 : Operand<i64> {
  let DecoderMethod = "DecodeSImmWithOffsetAndScale<16>";
  let ParserMatchClass = SImm16AsmOperandClass;
}

// like simm32 but coerces simm32 to uimm32.
def uimm32_coerced : Operand<i32> {
  let ParserMatchClass = UImm32CoercedAsmOperandClass;
}
// Like simm32 but coerces uimm32 to simm32.
def simm32_relaxed : Operand<i32> {
  let DecoderMethod = "DecodeSImmWithOffsetAndScale<32>";
  let ParserMatchClass = SImm32RelaxedAsmOperandClass;
}

// This is almost the same as a uimm7 but 0x7f is interpreted as -1.
def li16_imm : Operand<i32> {
  let DecoderMethod = "DecodeLi16Imm";
  let ParserMatchClass = ConstantUImm7Sub1AsmOperandClass;
}

def FgpuMemAsmOperand : AsmOperandClass {
  let Name = "Mem";
  let ParserMethod = "parseMemOperand";
}

class FgpuMemSimmAsmOperand<int Width, int Shift = 0> : AsmOperandClass {
  let Name = "MemOffsetSimm" # Width # "_" # Shift;
  let SuperClasses = [FgpuMemAsmOperand];
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseMemOperand";
  let PredicateMethod = "isMemWithSimmOffset<" # Width # ", " # Shift # ">";
  let DiagnosticType = !if(!eq(Shift, 0), "MemSImm" # Width,
                                          "MemSImm" # Width # "Lsl" # Shift);
}

def FgpuMemSimmPtrAsmOperand : AsmOperandClass {
  let Name = "MemOffsetSimmPtr";
  let SuperClasses = [FgpuMemAsmOperand];
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseMemOperand";
  let PredicateMethod = "isMemWithPtrSizeOffset";
  let DiagnosticType = "MemSImmPtr";
}

def FgpuInvertedImmoperand : AsmOperandClass {
  let Name = "InvNum";
  let RenderMethod = "addImmOperands";
  let ParserMethod = "parseInvNum";
}

def InvertedImOperand : Operand<i32> {
  let ParserMatchClass = FgpuInvertedImmoperand;
}

def InvertedImOperand64 : Operand<i64> {
  let ParserMatchClass = FgpuInvertedImmoperand;
}

class mem_generic : Operand<iPTR> {
  let PrintMethod = "printMemOperand";
  let MIOperandInfo = (ops ptr_rc, simm16);
  let EncoderMethod = "getMemEncoding";
  let ParserMatchClass = FgpuMemAsmOperand;
  let OperandType = "OPERAND_MEMORY";
}

// Address operand
def mem : mem_generic;

// MSA specific address operand
def mem_msa : mem_generic {
  let MIOperandInfo = (ops ptr_rc, simm10);
  let EncoderMethod = "getMSAMemEncoding";
}

def simm12 : Operand<i32> {
  let DecoderMethod = "DecodeSimm12";
}

def mem_simm9_exp : mem_generic {
  let MIOperandInfo = (ops ptr_rc, simm9);
  let ParserMatchClass = FgpuMemSimmPtrAsmOperand;
  let OperandNamespace = "FgpuII";
  let OperandType = "OPERAND_MEM_SIMM9";
}

foreach I = {9, 10, 11, 12, 16} in
  def mem_simm # I : mem_generic {
    let MIOperandInfo = (ops ptr_rc, !cast<Operand>("simm" # I));
    let ParserMatchClass = FgpuMemSimmAsmOperand<I>;
  }

foreach I = {1, 2, 3} in
  def mem_simm10_lsl # I : mem_generic {
    let MIOperandInfo = (ops ptr_rc, !cast<Operand>("simm10_lsl" # I));
    let EncoderMethod = "getMemEncoding<" # I  # ">";
    let ParserMatchClass = FgpuMemSimmAsmOperand<10, I>;
  }

def mem_simmptr : mem_generic {
  let ParserMatchClass = FgpuMemSimmPtrAsmOperand;
}

def mem_ea : Operand<iPTR> {
  let PrintMethod = "printMemOperandEA";
  let MIOperandInfo = (ops ptr_rc, simm16);
  let EncoderMethod = "getMemEncoding";
  let OperandType = "OPERAND_MEMORY";
}

def PtrRC : Operand<iPTR> {
  let MIOperandInfo = (ops ptr_rc);
  let DecoderMethod = "DecodePtrRegisterClass";
  let ParserMatchClass = GPR32AsmOperand;
}

// size operand of ins instruction
def size_ins : Operand<i32> {
  let EncoderMethod = "getSizeInsEncoding";
  let DecoderMethod = "DecodeInsSize";
}

// Transformation Function - get the lower 16 bits.
def LO16 : SDNodeXForm<imm, [{
  return getImm(N, N->getZExtValue() & 0xFFFF);
}]>;

// Transformation Function - get the higher 16 bits.
def HI16 : SDNodeXForm<imm, [{
  return getImm(N, (N->getZExtValue() >> 16) & 0xFFFF);
}]>;

// Plus 1.
def Plus1 : SDNodeXForm<imm, [{ return getImm(N, N->getSExtValue() + 1); }]>;

// Node immediate is zero (e.g. insve.d)
def immz : PatLeaf<(imm), [{ return N->getSExtValue() == 0; }]>;

// Node immediate fits as 16-bit sign extended on target immediate.
// e.g. addi, andi
def immSExt8  : PatLeaf<(imm), [{ return isInt<8>(N->getSExtValue()); }]>;

// Node immediate fits as 16-bit sign extended on target immediate.
// e.g. addi, andi
def immSExt16  : PatLeaf<(imm), [{ return isInt<16>(N->getSExtValue()); }]>;
def imm32SExt16  : IntImmLeaf<i32, [{ return isInt<16>(Imm.getSExtValue()); }]>;

// Node immediate fits as 7-bit zero extended on target immediate.
def immZExt7 : PatLeaf<(imm), [{ return isUInt<7>(N->getZExtValue()); }]>;
def timmZExt7 : PatLeaf<(timm), [{ return isUInt<7>(N->getZExtValue()); }]>;

// Node immediate fits as 16-bit zero extended on target immediate.
// The LO16 param means that only the lower 16 bits of the node
// immediate are caught.
// e.g. addiu, sltiu
def immZExt16  : PatLeaf<(imm), [{
  if (N->getValueType(0) == MVT::i32)
    return (uint32_t)N->getZExtValue() == (unsigned short)N->getZExtValue();
  else
    return (uint64_t)N->getZExtValue() == (unsigned short)N->getZExtValue();
}], LO16>;
def imm32ZExt16  : IntImmLeaf<i32, [{
  return (uint32_t)Imm.getZExtValue() == (unsigned short)Imm.getZExtValue();
}]>;

// Immediate can be loaded with LUi (32-bit int with lower 16-bit cleared).
def immSExt32Low16Zero : PatLeaf<(imm), [{
  int64_t Val = N->getSExtValue();
  return isInt<32>(Val) && !(Val & 0xffff);
}]>;

// Zero-extended 32-bit unsigned int with lower 16-bit cleared.
def immZExt32Low16Zero : PatLeaf<(imm), [{
  uint64_t Val = N->getZExtValue();
  return isUInt<32>(Val) && !(Val & 0xffff);
}]>;

// Note immediate fits as a 32 bit signed extended on target immediate.
def immSExt32  : PatLeaf<(imm), [{ return isInt<32>(N->getSExtValue()); }]>;

// Note immediate fits as a 32 bit zero extended on target immediate.
def immZExt32  : PatLeaf<(imm), [{ return isUInt<32>(N->getZExtValue()); }]>;

// shamt field must fit in 5 bits.
def immZExt5 : ImmLeaf<i32, [{return Imm == (Imm & 0x1f);}]>;
def timmZExt5 : TImmLeaf<i32, [{return Imm == (Imm & 0x1f);}]>;

def immZExt5Plus1 : PatLeaf<(imm), [{
  return isUInt<5>(N->getZExtValue() - 1);
}]>;
def immZExt5Plus32 : PatLeaf<(imm), [{
  return isUInt<5>(N->getZExtValue() - 32);
}]>;
def immZExt5Plus33 : PatLeaf<(imm), [{
  return isUInt<5>(N->getZExtValue() - 33);
}]>;

def immZExt5To31 : SDNodeXForm<imm, [{
  return getImm(N, 31 - N->getZExtValue());
}]>;

// True if (N + 1) fits in 16-bit field.
def immSExt16Plus1 : PatLeaf<(imm), [{
  return isInt<17>(N->getSExtValue()) && isInt<16>(N->getSExtValue() + 1);
}]>;

def immZExtRange2To64 : PatLeaf<(imm), [{
  return isUInt<7>(N->getZExtValue()) && (N->getZExtValue() >= 2) &&
         (N->getZExtValue() <= 64);
}]>;

def ORiPred  : PatLeaf<(imm), [{
  return isUInt<16>(N->getZExtValue()) && !isInt<16>(N->getSExtValue());
}], LO16>;

def LUiPred : PatLeaf<(imm), [{
  int64_t Val = N->getSExtValue();
  return !isInt<16>(Val) && isInt<32>(Val) && !(Val & 0xffff);
}]>;

def LUiORiPred  : PatLeaf<(imm), [{
  int64_t SVal = N->getSExtValue();
  return isInt<32>(SVal) && (SVal & 0xffff);
}]>;

// Fgpu Address Mode! SDNode frameindex could possibly be a match
// since load and store instructions from stack used it.
def addr :
  ComplexPattern<iPTR, 2, "selectIntAddr", [frameindex]>;

def addrRegImm :
  ComplexPattern<iPTR, 2, "selectAddrRegImm", [frameindex]>;

def addrDefault :
  ComplexPattern<iPTR, 2, "selectAddrDefault", [frameindex]>;

def addrimm10 : ComplexPattern<iPTR, 2, "selectIntAddrSImm10", [frameindex]>;
def addrimm10lsl1 : ComplexPattern<iPTR, 2, "selectIntAddrSImm10Lsl1",
                                   [frameindex]>;
def addrimm10lsl2 : ComplexPattern<iPTR, 2, "selectIntAddrSImm10Lsl2",
                                   [frameindex]>;
def addrimm10lsl3 : ComplexPattern<iPTR, 2, "selectIntAddrSImm10Lsl3",
                                   [frameindex]>;

//===----------------------------------------------------------------------===//
// Instructions specific format
//===----------------------------------------------------------------------===//

// Arithmetic and logical instructions with 3 register operands.
class ArithLogicR<string opstr, RegisterOperand RO, bit isComm = 0,
                  InstrItinClass Itin = NoItinerary,
                  SDPatternOperator OpNode = null_frag>:
  InstSE<(outs RO:$rd), (ins RO:$rs, RO:$rt),
         !strconcat(opstr, "\t$rd, $rs, $rt"),
         [(set RO:$rd, (OpNode RO:$rs, RO:$rt))], Itin, FrmR, opstr> {
  let isCommutable = isComm;
  let isReMaterializable = 1;
  let TwoOperandAliasConstraint = "$rd = $rs";
}

// Arithmetic and logical instructions with 2 register operands.
class ArithLogicI<string opstr, Operand Od, RegisterOperand RO,
                  InstrItinClass Itin = NoItinerary,
                  SDPatternOperator imm_type = null_frag,
                  SDPatternOperator OpNode = null_frag> :
  InstSE<(outs RO:$rt), (ins RO:$rs, Od:$imm16),
         !strconcat(opstr, "\t$rt, $rs, $imm16"),
         [(set RO:$rt, (OpNode RO:$rs, imm_type:$imm16))],
         Itin, FrmI, opstr> {
  let isReMaterializable = 1;
  let TwoOperandAliasConstraint = "$rs = $rt";
}

// Arithmetic Multiply ADD/SUB
class MArithR<string opstr, InstrItinClass itin, bit isComm = 0> :
  InstSE<(outs), (ins GPR32Opnd:$rs, GPR32Opnd:$rt),
         !strconcat(opstr, "\t$rs, $rt"), [], itin, FrmR, opstr> {
  let Defs = [HI0, LO0];
  let Uses = [HI0, LO0];
  let isCommutable = isComm;
}

//  Logical
class LogicNOR<string opstr, RegisterOperand RO>:
  InstSE<(outs RO:$rd), (ins RO:$rs, RO:$rt),
         !strconcat(opstr, "\t$rd, $rs, $rt"),
         [(set RO:$rd, (not (or RO:$rs, RO:$rt)))], II_NOR, FrmR, opstr> {
  let isCommutable = 1;
}

// Shifts
class shift_rotate_imm<string opstr, Operand ImmOpnd,
                       RegisterOperand RO, InstrItinClass itin,
                       SDPatternOperator OpNode = null_frag,
                       SDPatternOperator PF = null_frag> :
  InstSE<(outs RO:$rd), (ins RO:$rt, ImmOpnd:$shamt),
         !strconcat(opstr, "\t$rd, $rt, $shamt"),
         [(set RO:$rd, (OpNode RO:$rt, PF:$shamt))], itin, FrmR, opstr> {
  let TwoOperandAliasConstraint = "$rt = $rd";
}

class shift_rotate_reg<string opstr, RegisterOperand RO, InstrItinClass itin,
                       SDPatternOperator OpNode = null_frag>:
  InstSE<(outs RO:$rd), (ins RO:$rt, GPR32Opnd:$rs),
         !strconcat(opstr, "\t$rd, $rt, $rs"),
         [(set RO:$rd, (OpNode RO:$rt, GPR32Opnd:$rs))], itin, FrmR,
         opstr>;

// Load Upper Immediate
class LoadUpper<string opstr, RegisterOperand RO, Operand Imm>:
  InstSE<(outs RO:$rt), (ins Imm:$imm16), !strconcat(opstr, "\t$rt, $imm16"),
         [], II_LUI, FrmI, opstr>, IsAsCheapAsAMove {
  let hasSideEffects = 0;
  let isReMaterializable = 1;
}

// Memory Load/Store
class LoadMemory<string opstr, DAGOperand RO, DAGOperand MO,
                 SDPatternOperator OpNode = null_frag,
                 InstrItinClass Itin = NoItinerary,
                 ComplexPattern Addr = addr> :
  InstSE<(outs RO:$rt), (ins MO:$addr), !strconcat(opstr, "\t$rt, $addr"),
         [(set RO:$rt, (OpNode Addr:$addr))], Itin, FrmI, opstr> {
  let DecoderMethod = "DecodeMem";
  let canFoldAsLoad = 1;
  string BaseOpcode = opstr;
  let mayLoad = 1;
}

class Load<string opstr, DAGOperand RO, SDPatternOperator OpNode = null_frag,
           InstrItinClass Itin = NoItinerary, ComplexPattern Addr = addr> :
  LoadMemory<opstr, RO, mem, OpNode, Itin, Addr>;

class StoreMemory<string opstr, DAGOperand RO, DAGOperand MO,
            SDPatternOperator OpNode = null_frag,
            InstrItinClass Itin = NoItinerary, ComplexPattern Addr = addr> :
  InstSE<(outs), (ins RO:$rt, MO:$addr), !strconcat(opstr, "\t$rt, $addr"),
         [(OpNode RO:$rt, Addr:$addr)], Itin, FrmI, opstr> {
  let DecoderMethod = "DecodeMem";
  string BaseOpcode = opstr;
  let mayStore = 1;
}

class Store<string opstr, DAGOperand RO, SDPatternOperator OpNode = null_frag,
            InstrItinClass Itin = NoItinerary, ComplexPattern Addr = addr,
            DAGOperand MO = mem> :
  StoreMemory<opstr, RO, MO, OpNode, Itin, Addr>;

// Load/Store Left/Right
let canFoldAsLoad = 1 in
class LoadLeftRight<string opstr, SDNode OpNode, RegisterOperand RO,
                    InstrItinClass Itin> :
  InstSE<(outs RO:$rt), (ins mem:$addr, RO:$src),
         !strconcat(opstr, "\t$rt, $addr"),
         [(set RO:$rt, (OpNode addr:$addr, RO:$src))], Itin, FrmI> {
  let DecoderMethod = "DecodeMem";
  string Constraints = "$src = $rt";
  let BaseOpcode = opstr;
}

class StoreLeftRight<string opstr, SDNode OpNode, RegisterOperand RO,
                     InstrItinClass Itin> :
  InstSE<(outs), (ins RO:$rt, mem:$addr), !strconcat(opstr, "\t$rt, $addr"),
         [(OpNode RO:$rt, addr:$addr)], Itin, FrmI> {
  let DecoderMethod = "DecodeMem";
  let BaseOpcode = opstr;
}

// COP2 Load/Store
class LW_FT2<string opstr, RegisterOperand RC, InstrItinClass Itin,
             SDPatternOperator OpNode= null_frag> :
  InstSE<(outs RC:$rt), (ins mem_simm16:$addr),
         !strconcat(opstr, "\t$rt, $addr"),
         [(set RC:$rt, (OpNode addrDefault:$addr))], Itin, FrmFI, opstr> {
  let DecoderMethod = "DecodeFMem2";
  let mayLoad = 1;
}

class SW_FT2<string opstr, RegisterOperand RC, InstrItinClass Itin,
             SDPatternOperator OpNode= null_frag> :
  InstSE<(outs), (ins RC:$rt, mem_simm16:$addr),
         !strconcat(opstr, "\t$rt, $addr"),
         [(OpNode RC:$rt, addrDefault:$addr)], Itin, FrmFI, opstr> {
  let DecoderMethod = "DecodeFMem2";
  let mayStore = 1;
}

// COP3 Load/Store
class LW_FT3<string opstr, RegisterOperand RC, InstrItinClass Itin,
             SDPatternOperator OpNode= null_frag> :
  InstSE<(outs RC:$rt), (ins mem:$addr), !strconcat(opstr, "\t$rt, $addr"),
         [(set RC:$rt, (OpNode addrDefault:$addr))], Itin, FrmFI, opstr> {
  let DecoderMethod = "DecodeFMem3";
  let mayLoad = 1;
}

class SW_FT3<string opstr, RegisterOperand RC, InstrItinClass Itin,
             SDPatternOperator OpNode= null_frag> :
  InstSE<(outs), (ins RC:$rt, mem:$addr), !strconcat(opstr, "\t$rt, $addr"),
         [(OpNode RC:$rt, addrDefault:$addr)], Itin, FrmFI, opstr> {
  let DecoderMethod = "DecodeFMem3";
  let mayStore = 1;
}

// Conditional Branch
class CBranch<string opstr, DAGOperand opnd, PatFrag cond_op,
              RegisterOperand RO> :
  InstSE<(outs), (ins RO:$rs, RO:$rt, opnd:$offset),
         !strconcat(opstr, "\t$rs, $rt, $offset"),
         [(brcond (i32 (cond_op RO:$rs, RO:$rt)), bb:$offset)], II_BCC,
         FrmI, opstr> {
  let isBranch = 1;
  let isTerminator = 1;
  let hasDelaySlot = 1;
  let Defs = [AT];
  bit isCTI = 1;
}

class CBranchLikely<string opstr, DAGOperand opnd, RegisterOperand RO> :
  InstSE<(outs), (ins RO:$rs, RO:$rt, opnd:$offset),
         !strconcat(opstr, "\t$rs, $rt, $offset"), [], II_BCC, FrmI, opstr> {
  let isBranch = 1;
  let isTerminator = 1;
  let hasDelaySlot = 1;
  let Defs = [AT];
  bit isCTI = 1;
}

class CBranchZero<string opstr, DAGOperand opnd, PatFrag cond_op,
                  RegisterOperand RO> :
  InstSE<(outs), (ins RO:$rs, opnd:$offset),
         !strconcat(opstr, "\t$rs, $offset"),
         [(brcond (i32 (cond_op RO:$rs, 0)), bb:$offset)], II_BCCZ,
         FrmI, opstr> {
  let isBranch = 1;
  let isTerminator = 1;
  let hasDelaySlot = 1;
  let Defs = [AT];
  bit isCTI = 1;
}

class CBranchZeroLikely<string opstr, DAGOperand opnd, RegisterOperand RO> :
  InstSE<(outs), (ins RO:$rs, opnd:$offset),
         !strconcat(opstr, "\t$rs, $offset"), [], II_BCCZ, FrmI, opstr> {
  let isBranch = 1;
  let isTerminator = 1;
  let hasDelaySlot = 1;
  let Defs = [AT];
  bit isCTI = 1;
}

// SetCC
class SetCC_R<string opstr, PatFrag cond_op, RegisterOperand RO> :
  InstSE<(outs GPR32Opnd:$rd), (ins RO:$rs, RO:$rt),
         !strconcat(opstr, "\t$rd, $rs, $rt"),
         [(set GPR32Opnd:$rd, (cond_op RO:$rs, RO:$rt))],
         II_SLT_SLTU, FrmR, opstr>;

class SetCC_I<string opstr, PatFrag cond_op, Operand Od, PatLeaf imm_type,
              RegisterOperand RO>:
  InstSE<(outs GPR32Opnd:$rt), (ins RO:$rs, Od:$imm16),
         !strconcat(opstr, "\t$rt, $rs, $imm16"),
         [(set GPR32Opnd:$rt, (cond_op RO:$rs, imm_type:$imm16))],
         II_SLTI_SLTIU, FrmI, opstr>;

// Jump
class JumpFJ<DAGOperand opnd, string opstr, SDPatternOperator operator,
             SDPatternOperator targetoperator, string bopstr> :
  InstSE<(outs), (ins opnd:$target), !strconcat(opstr, "\t$target"),
         [(operator targetoperator:$target)], II_J, FrmJ, bopstr> {
  let isTerminator=1;
  let isBarrier=1;
  let hasDelaySlot = 1;
  let DecoderMethod = "DecodeJumpTarget";
  let Defs = [AT];
  bit isCTI = 1;
}

// Unconditional branch
class UncondBranch<Instruction BEQInst, DAGOperand opnd> :
  PseudoSE<(outs), (ins brtarget:$offset), [(br bb:$offset)], II_B>,
  PseudoInstExpansion<(BEQInst ZERO, ZERO, opnd:$offset)> {
  let isBranch = 1;
  let isTerminator = 1;
  let isBarrier = 1;
  let hasDelaySlot = 1;
  let AdditionalPredicates = [RelocPIC];
  let Defs = [AT];
  bit isCTI = 1;
}

// Base class for indirect branch and return instruction classes.
let isTerminator=1, isBarrier=1, hasDelaySlot = 1, isCTI = 1 in
class JumpFR<string opstr, RegisterOperand RO,
             SDPatternOperator operator = null_frag>:
  InstSE<(outs), (ins RO:$rs), "jr\t$rs", [(operator RO:$rs)], II_JR,
         FrmR, opstr>;

// Indirect branch
class IndirectBranch<string opstr, RegisterOperand RO> : JumpFR<opstr, RO> {
  let isBranch = 1;
  let isIndirectBranch = 1;
}

// Jump and Link (Call)
let isCall=1, hasDelaySlot=1, isCTI=1, Defs = [RA] in {
  class JumpLink<string opstr, DAGOperand opnd> :
    InstSE<(outs), (ins opnd:$target), !strconcat(opstr, "\t$target"),
           [(FgpuJmpLink tglobaladdr:$target)], II_JAL, FrmJ, opstr> {
    let DecoderMethod = "DecodeJumpTarget";
  }

  class JumpLinkRegPseudo<RegisterOperand RO, Instruction JALRInst,
                          Register RetReg, RegisterOperand ResRO = RO>:
    PseudoSE<(outs), (ins RO:$rs), [(FgpuJmpLink RO:$rs)], II_JALR>,
    PseudoInstExpansion<(JALRInst RetReg, ResRO:$rs)> {
    let hasPostISelHook = 1;
  }

  class JumpLinkReg<string opstr, RegisterOperand RO>:
    InstSE<(outs RO:$rd), (ins RO:$rs), !strconcat(opstr, "\t$rd, $rs"),
           [], II_JALR, FrmR, opstr> {
    let hasPostISelHook = 1;
  }

  class BGEZAL_FT<string opstr, DAGOperand opnd,
                  RegisterOperand RO> :
    InstSE<(outs), (ins RO:$rs, opnd:$offset),
           !strconcat(opstr, "\t$rs, $offset"), [], II_BCCZAL, FrmI, opstr> {
    let hasDelaySlot = 1;
  }

}

let isCall = 1, isTerminator = 1, isReturn = 1, isBarrier = 1, hasDelaySlot = 1,
    hasExtraSrcRegAllocReq = 1, isCTI = 1, Defs = [AT] in {
  class TailCall<Instruction JumpInst, DAGOperand Opnd> :
    PseudoSE<(outs), (ins calltarget:$target), [], II_J>,
    PseudoInstExpansion<(JumpInst Opnd:$target)>;

  class TailCallReg<Instruction JumpInst, RegisterOperand RO> :
    PseudoSE<(outs), (ins RO:$rs), [(FgpuTailCall RO:$rs)], II_JR>,
    PseudoInstExpansion<(JumpInst RO:$rs)> {
    let hasPostISelHook = 1;
  }
}

class BAL_BR_Pseudo<Instruction RealInst, DAGOperand opnd> :
  PseudoSE<(outs), (ins opnd:$offset), [], II_BCCZAL>,
  PseudoInstExpansion<(RealInst ZERO, opnd:$offset)> {
  let isBranch = 1;
  let isTerminator = 1;
  let isBarrier = 1;
  let hasDelaySlot = 1;
  let Defs = [RA];
  bit isCTI = 1;
}

let isCTI = 1 in {
// Syscall
class SYS_FT<string opstr, Operand ImmOp, InstrItinClass itin = NoItinerary> :
  InstSE<(outs), (ins ImmOp:$code_),
         !strconcat(opstr, "\t$code_"), [], itin, FrmI, opstr>;
// Break
class BRK_FT<string opstr> :
  InstSE<(outs), (ins uimm10:$code_1, uimm10:$code_2),
         !strconcat(opstr, "\t$code_1, $code_2"), [], II_BREAK,
         FrmOther, opstr>;

// (D)Eret
class ER_FT<string opstr, InstrItinClass itin = NoItinerary> :
  InstSE<(outs), (ins),
         opstr, [], itin, FrmOther, opstr>;

// Wait
class WAIT_FT<string opstr> :
  InstSE<(outs), (ins), opstr, [], II_WAIT, FrmOther, opstr>;
}

// Interrupts
class DEI_FT<string opstr, RegisterOperand RO,
             InstrItinClass itin = NoItinerary> :
  InstSE<(outs RO:$rt), (ins),
         !strconcat(opstr, "\t$rt"), [], itin, FrmOther, opstr>;

// Sync
let hasSideEffects = 1 in
class SYNC_FT<string opstr> :
  InstSE<(outs), (ins uimm5:$stype), "sync $stype",
         [(FgpuSync immZExt5:$stype)], II_SYNC, FrmOther, opstr>;

class SYNCI_FT<string opstr, DAGOperand MO> :
  InstSE<(outs), (ins MO:$addr), !strconcat(opstr, "\t$addr"), [],
         II_SYNCI, FrmOther, opstr> {
  let hasSideEffects = 1;
  let DecoderMethod = "DecodeSyncI";
}

let hasSideEffects = 1, isCTI = 1 in {
class TEQ_FT<string opstr, RegisterOperand RO, Operand ImmOp,
             InstrItinClass itin = NoItinerary> :
  InstSE<(outs), (ins RO:$rs, RO:$rt, ImmOp:$code_),
         !strconcat(opstr, "\t$rs, $rt, $code_"), [], itin, FrmI, opstr>;

class TEQI_FT<string opstr, RegisterOperand RO,
              InstrItinClass itin = NoItinerary> :
  InstSE<(outs), (ins RO:$rs, simm16:$imm16),
         !strconcat(opstr, "\t$rs, $imm16"), [], itin, FrmOther, opstr>;
}

// Mul, Div
class Mult<string opstr, InstrItinClass itin, RegisterOperand RO,
           list<Register> DefRegs> :
  InstSE<(outs), (ins RO:$rs, RO:$rt), !strconcat(opstr, "\t$rs, $rt"), [],
         itin, FrmR, opstr> {
  let isCommutable = 1;
  let Defs = DefRegs;
  let hasSideEffects = 0;
}

// Pseudo multiply/divide instruction with explicit accumulator register
// operands.
class MultDivPseudo<Instruction RealInst, RegisterClass R0, RegisterOperand R1,
                    SDPatternOperator OpNode, InstrItinClass Itin,
                    bit IsComm = 1, bit HasSideEffects = 0,
                    bit UsesCustomInserter = 0> :
  PseudoSE<(outs R0:$ac), (ins R1:$rs, R1:$rt),
           [(set R0:$ac, (OpNode R1:$rs, R1:$rt))], Itin>,
  PseudoInstExpansion<(RealInst R1:$rs, R1:$rt)> {
  let isCommutable = IsComm;
  let hasSideEffects = HasSideEffects;
  let usesCustomInserter = UsesCustomInserter;
}

// Pseudo multiply add/sub instruction with explicit accumulator register
// operands.
class MAddSubPseudo<Instruction RealInst, SDPatternOperator OpNode,
                    InstrItinClass itin>
  : PseudoSE<(outs ACC64:$ac),
             (ins GPR32Opnd:$rs, GPR32Opnd:$rt, ACC64:$acin),
             [(set ACC64:$ac,
              (OpNode GPR32Opnd:$rs, GPR32Opnd:$rt, ACC64:$acin))],
             itin>,
    PseudoInstExpansion<(RealInst GPR32Opnd:$rs, GPR32Opnd:$rt)> {
  string Constraints = "$acin = $ac";
}

class Div<string opstr, InstrItinClass itin, RegisterOperand RO,
          list<Register> DefRegs> :
  InstSE<(outs), (ins RO:$rs, RO:$rt), !strconcat(opstr, "\t$$zero, $rs, $rt"),
         [], itin, FrmR, opstr> {
  let Defs = DefRegs;
}

// Move from Hi/Lo
class PseudoMFLOHI<RegisterClass DstRC, RegisterClass SrcRC, SDNode OpNode>
  : PseudoSE<(outs DstRC:$rd), (ins SrcRC:$hilo),
             [(set DstRC:$rd, (OpNode SrcRC:$hilo))], II_MFHI_MFLO>;

class MoveFromLOHI<string opstr, RegisterOperand RO, Register UseReg>:
  InstSE<(outs RO:$rd), (ins), !strconcat(opstr, "\t$rd"), [], II_MFHI_MFLO,
         FrmR, opstr> {
  let Uses = [UseReg];
  let hasSideEffects = 0;
  let isMoveReg = 1;
}

class PseudoMTLOHI<RegisterClass DstRC, RegisterClass SrcRC>
  : PseudoSE<(outs DstRC:$lohi), (ins SrcRC:$lo, SrcRC:$hi),
             [(set DstRC:$lohi, (FgpuMTLOHI SrcRC:$lo, SrcRC:$hi))],
             II_MTHI_MTLO>;

class MoveToLOHI<string opstr, RegisterOperand RO, list<Register> DefRegs>:
  InstSE<(outs), (ins RO:$rs), !strconcat(opstr, "\t$rs"), [], II_MTHI_MTLO,
  FrmR, opstr> {
  let Defs = DefRegs;
  let hasSideEffects = 0;
  let isMoveReg = 1;
}

class EffectiveAddress<string opstr, RegisterOperand RO> :
  InstSE<(outs RO:$rt), (ins mem_ea:$addr), !strconcat(opstr, "\t$rt, $addr"),
         [(set RO:$rt, addr:$addr)], II_ADDIU, FrmI,
         !strconcat(opstr, "_lea")> {
  let isCodeGenOnly = 1;
  let hasNoSchedulingInfo = 1;
  let DecoderMethod = "DecodeMem";
}

// Count Leading Ones/Zeros in Word
class CountLeading0<string opstr, RegisterOperand RO,
                  InstrItinClass itin = NoItinerary>:
  InstSE<(outs RO:$rd), (ins RO:$rs), !strconcat(opstr, "\t$rd, $rs"),
         [(set RO:$rd, (ctlz RO:$rs))], itin, FrmR, opstr>;

class CountLeading1<string opstr, RegisterOperand RO,
                  InstrItinClass itin = NoItinerary>:
  InstSE<(outs RO:$rd), (ins RO:$rs), !strconcat(opstr, "\t$rd, $rs"),
         [(set RO:$rd, (ctlz (not RO:$rs)))], itin, FrmR, opstr>;

// Sign Extend in Register.
class SignExtInReg<string opstr, ValueType vt, RegisterOperand RO,
                   InstrItinClass itin> :
  InstSE<(outs RO:$rd), (ins RO:$rt), !strconcat(opstr, "\t$rd, $rt"),
         [(set RO:$rd, (sext_inreg RO:$rt, vt))], itin, FrmR, opstr>;

// Subword Swap
class SubwordSwap<string opstr, RegisterOperand RO,
                  InstrItinClass itin = NoItinerary>:
  InstSE<(outs RO:$rd), (ins RO:$rt), !strconcat(opstr, "\t$rd, $rt"), [], itin,
         FrmR, opstr> {
  let hasSideEffects = 0;
}

// Read Hardware
class ReadHardware<RegisterOperand CPURegOperand, RegisterOperand RO> :
  InstSE<(outs CPURegOperand:$rt), (ins RO:$rd, uimm8:$sel),
         "rdhwr\t$rt, $rd, $sel", [], II_RDHWR, FrmR, "rdhwr">;

// Ext and Ins
class ExtBase<string opstr, RegisterOperand RO, Operand PosOpnd,
              Operand SizeOpnd, PatFrag PosImm, PatFrag SizeImm,
              SDPatternOperator Op = null_frag> :
  InstSE<(outs RO:$rt), (ins RO:$rs, PosOpnd:$pos, SizeOpnd:$size),
         !strconcat(opstr, "\t$rt, $rs, $pos, $size"),
         [(set RO:$rt, (Op RO:$rs, PosImm:$pos, SizeImm:$size))], II_EXT,
         FrmR, opstr>;

// 'ins' and its' 64 bit variants are matched by C++ code.
class InsBase<string opstr, RegisterOperand RO, Operand PosOpnd,
              Operand SizeOpnd, PatFrag PosImm, PatFrag SizeImm>:
  InstSE<(outs RO:$rt), (ins RO:$rs, PosOpnd:$pos, SizeOpnd:$size, RO:$src),
         !strconcat(opstr, "\t$rt, $rs, $pos, $size"),
         [(set RO:$rt, (null_frag RO:$rs, PosImm:$pos, SizeImm:$size,
                                  RO:$src))],
         II_INS, FrmR, opstr> {
  let Constraints = "$src = $rt";
}

// Atomic instructions with 2 source operands (ATOMIC_SWAP & ATOMIC_LOAD_*).
class Atomic2Ops<PatFrag Op, RegisterClass DRC> :
  PseudoSE<(outs DRC:$dst), (ins PtrRC:$ptr, DRC:$incr),
           [(set DRC:$dst, (Op iPTR:$ptr, DRC:$incr))]> {
  let hasNoSchedulingInfo = 1;
}

class Atomic2OpsPostRA<RegisterClass RC> :
  PseudoSE<(outs RC:$dst), (ins PtrRC:$ptr, RC:$incr), []> {
  let mayLoad = 1;
  let mayStore = 1;
}

class Atomic2OpsSubwordPostRA<RegisterClass RC> :
  PseudoSE<(outs RC:$dst), (ins PtrRC:$ptr, RC:$incr, RC:$mask, RC:$mask2,
                                RC:$shiftamnt), []>;

// Atomic Compare & Swap.
// Atomic compare and swap is lowered into two stages. The first stage happens
// during ISelLowering, which produces the PostRA version of this instruction.
class AtomicCmpSwap<PatFrag Op, RegisterClass DRC> :
  PseudoSE<(outs DRC:$dst), (ins PtrRC:$ptr, DRC:$cmp, DRC:$swap),
           [(set DRC:$dst, (Op iPTR:$ptr, DRC:$cmp, DRC:$swap))]> {
  let hasNoSchedulingInfo = 1;
}

class AtomicCmpSwapPostRA<RegisterClass RC> :
  PseudoSE<(outs RC:$dst), (ins PtrRC:$ptr, RC:$cmp, RC:$swap), []> {
  let mayLoad = 1;
  let mayStore = 1;
}

class AtomicCmpSwapSubwordPostRA<RegisterClass RC> :
  PseudoSE<(outs RC:$dst), (ins PtrRC:$ptr, RC:$mask, RC:$ShiftCmpVal,
                                RC:$mask2, RC:$ShiftNewVal, RC:$ShiftAmt), []> {
  let mayLoad = 1;
  let mayStore = 1;
}

class LLBase<string opstr, RegisterOperand RO, DAGOperand MO = mem> :
  InstSE<(outs RO:$rt), (ins MO:$addr), !strconcat(opstr, "\t$rt, $addr"),
         [], II_LL, FrmI, opstr> {
  let DecoderMethod = "DecodeMem";
  let mayLoad = 1;
}

class SCBase<string opstr, RegisterOperand RO> :
  InstSE<(outs RO:$dst), (ins RO:$rt, mem:$addr),
         !strconcat(opstr, "\t$rt, $addr"), [], II_SC, FrmI> {
  let DecoderMethod = "DecodeMem";
  let mayStore = 1;
  let Constraints = "$rt = $dst";
}

class MFC3OP<string asmstr, RegisterOperand RO, RegisterOperand RD,
             InstrItinClass itin> :
  InstSE<(outs RO:$rt), (ins RD:$rd, uimm3:$sel),
         !strconcat(asmstr, "\t$rt, $rd, $sel"), [], itin, FrmFR> {
  let BaseOpcode = asmstr;
}

class MTC3OP<string asmstr, RegisterOperand RO, RegisterOperand RD,
             InstrItinClass itin> :
  InstSE<(outs RO:$rd), (ins RD:$rt, uimm3:$sel),
         !strconcat(asmstr, "\t$rt, $rd, $sel"), [], itin, FrmFR> {
  let BaseOpcode = asmstr;
}

class TrapBase<Instruction RealInst>
  : PseudoSE<(outs), (ins), [(trap)], II_TRAP>,
    PseudoInstExpansion<(RealInst 0, 0)> {
  let mayStore = 0;
  let mayLoad = 0;
  let hasSideEffects = 1;
  let isTrap = 1;
  let isCodeGenOnly = 1;
}

//===----------------------------------------------------------------------===//
// Pseudo instructions
//===----------------------------------------------------------------------===//

// Return RA.
let isReturn=1, isTerminator=1, isBarrier=1, hasCtrlDep=1, isCTI=1 in {
  let hasDelaySlot=1 in
  def RetRA : PseudoSE<(outs), (ins), [(FgpuRet)]>;

  let hasSideEffects=1 in
  def ERet : PseudoSE<(outs), (ins), [(FgpuERet)]>;
}

let Defs = [SP], Uses = [SP], hasSideEffects = 1, hasNoSchedulingInfo = 1 in {
def ADJCALLSTACKDOWN : FgpuPseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                                  [(callseq_start timm:$amt1, timm:$amt2)]>;
def ADJCALLSTACKUP   : FgpuPseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                                  [(callseq_end timm:$amt1, timm:$amt2)]>;
}

let usesCustomInserter = 1 in {
  def ATOMIC_LOAD_ADD_I8   : Atomic2Ops<atomic_load_add_8, GPR32>;
  def ATOMIC_LOAD_ADD_I16  : Atomic2Ops<atomic_load_add_16, GPR32>;
  def ATOMIC_LOAD_ADD_I32  : Atomic2Ops<atomic_load_add_32, GPR32>;
  def ATOMIC_LOAD_SUB_I8   : Atomic2Ops<atomic_load_sub_8, GPR32>;
  def ATOMIC_LOAD_SUB_I16  : Atomic2Ops<atomic_load_sub_16, GPR32>;
  def ATOMIC_LOAD_SUB_I32  : Atomic2Ops<atomic_load_sub_32, GPR32>;
  def ATOMIC_LOAD_AND_I8   : Atomic2Ops<atomic_load_and_8, GPR32>;
  def ATOMIC_LOAD_AND_I16  : Atomic2Ops<atomic_load_and_16, GPR32>;
  def ATOMIC_LOAD_AND_I32  : Atomic2Ops<atomic_load_and_32, GPR32>;
  def ATOMIC_LOAD_OR_I8    : Atomic2Ops<atomic_load_or_8, GPR32>;
  def ATOMIC_LOAD_OR_I16   : Atomic2Ops<atomic_load_or_16, GPR32>;
  def ATOMIC_LOAD_OR_I32   : Atomic2Ops<atomic_load_or_32, GPR32>;
  def ATOMIC_LOAD_XOR_I8   : Atomic2Ops<atomic_load_xor_8, GPR32>;
  def ATOMIC_LOAD_XOR_I16  : Atomic2Ops<atomic_load_xor_16, GPR32>;
  def ATOMIC_LOAD_XOR_I32  : Atomic2Ops<atomic_load_xor_32, GPR32>;
  def ATOMIC_LOAD_NAND_I8  : Atomic2Ops<atomic_load_nand_8, GPR32>;
  def ATOMIC_LOAD_NAND_I16 : Atomic2Ops<atomic_load_nand_16, GPR32>;
  def ATOMIC_LOAD_NAND_I32 : Atomic2Ops<atomic_load_nand_32, GPR32>;

  def ATOMIC_SWAP_I8       : Atomic2Ops<atomic_swap_8, GPR32>;
  def ATOMIC_SWAP_I16      : Atomic2Ops<atomic_swap_16, GPR32>;
  def ATOMIC_SWAP_I32      : Atomic2Ops<atomic_swap_32, GPR32>;

  def ATOMIC_CMP_SWAP_I8   : AtomicCmpSwap<atomic_cmp_swap_8, GPR32>;
  def ATOMIC_CMP_SWAP_I16  : AtomicCmpSwap<atomic_cmp_swap_16, GPR32>;
  def ATOMIC_CMP_SWAP_I32  : AtomicCmpSwap<atomic_cmp_swap_32, GPR32>;

  def ATOMIC_LOAD_MIN_I8   : Atomic2Ops<atomic_load_min_8, GPR32>;
  def ATOMIC_LOAD_MIN_I16  : Atomic2Ops<atomic_load_min_16, GPR32>;
  def ATOMIC_LOAD_MIN_I32  : Atomic2Ops<atomic_load_min_32, GPR32>;
  def ATOMIC_LOAD_MAX_I8   : Atomic2Ops<atomic_load_max_8, GPR32>;
  def ATOMIC_LOAD_MAX_I16  : Atomic2Ops<atomic_load_max_16, GPR32>;
  def ATOMIC_LOAD_MAX_I32  : Atomic2Ops<atomic_load_max_32, GPR32>;
  def ATOMIC_LOAD_UMIN_I8  : Atomic2Ops<atomic_load_umin_8, GPR32>;
  def ATOMIC_LOAD_UMIN_I16 : Atomic2Ops<atomic_load_umin_16, GPR32>;
  def ATOMIC_LOAD_UMIN_I32 : Atomic2Ops<atomic_load_umin_32, GPR32>;
  def ATOMIC_LOAD_UMAX_I8  : Atomic2Ops<atomic_load_umax_8, GPR32>;
  def ATOMIC_LOAD_UMAX_I16 : Atomic2Ops<atomic_load_umax_16, GPR32>;
  def ATOMIC_LOAD_UMAX_I32 : Atomic2Ops<atomic_load_umax_32, GPR32>;
}

def ATOMIC_LOAD_ADD_I8_POSTRA   : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_ADD_I16_POSTRA  : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_ADD_I32_POSTRA  : Atomic2OpsPostRA<GPR32>;
def ATOMIC_LOAD_SUB_I8_POSTRA   : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_SUB_I16_POSTRA  : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_SUB_I32_POSTRA  : Atomic2OpsPostRA<GPR32>;
def ATOMIC_LOAD_AND_I8_POSTRA   : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_AND_I16_POSTRA  : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_AND_I32_POSTRA  : Atomic2OpsPostRA<GPR32>;
def ATOMIC_LOAD_OR_I8_POSTRA    : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_OR_I16_POSTRA   : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_OR_I32_POSTRA   : Atomic2OpsPostRA<GPR32>;
def ATOMIC_LOAD_XOR_I8_POSTRA   : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_XOR_I16_POSTRA  : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_XOR_I32_POSTRA  : Atomic2OpsPostRA<GPR32>;
def ATOMIC_LOAD_NAND_I8_POSTRA  : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_NAND_I16_POSTRA : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_NAND_I32_POSTRA : Atomic2OpsPostRA<GPR32>;

def ATOMIC_SWAP_I8_POSTRA  : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_SWAP_I16_POSTRA : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_SWAP_I32_POSTRA : Atomic2OpsPostRA<GPR32>;

def ATOMIC_CMP_SWAP_I8_POSTRA : AtomicCmpSwapSubwordPostRA<GPR32>;
def ATOMIC_CMP_SWAP_I16_POSTRA : AtomicCmpSwapSubwordPostRA<GPR32>;
def ATOMIC_CMP_SWAP_I32_POSTRA : AtomicCmpSwapPostRA<GPR32>;

def ATOMIC_LOAD_MIN_I8_POSTRA   : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_MIN_I16_POSTRA  : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_MIN_I32_POSTRA  : Atomic2OpsPostRA<GPR32>;
def ATOMIC_LOAD_MAX_I8_POSTRA   : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_MAX_I16_POSTRA  : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_MAX_I32_POSTRA  : Atomic2OpsPostRA<GPR32>;
def ATOMIC_LOAD_UMIN_I8_POSTRA  : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_UMIN_I16_POSTRA : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_UMIN_I32_POSTRA : Atomic2OpsPostRA<GPR32>;
def ATOMIC_LOAD_UMAX_I8_POSTRA  : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_UMAX_I16_POSTRA : Atomic2OpsSubwordPostRA<GPR32>;
def ATOMIC_LOAD_UMAX_I32_POSTRA : Atomic2OpsPostRA<GPR32>;

/// Pseudo instructions for loading and storing accumulator registers.
let isPseudo = 1, isCodeGenOnly = 1, hasNoSchedulingInfo = 1 in {
  def LOAD_ACC64  : Load<"", ACC64>;
  def STORE_ACC64 : Store<"", ACC64>;
}

// We need these two pseudo instructions to avoid offset calculation for long
// branches.  See the comment in file FgpuLongBranch.cpp for detailed
// explanation.

// Expands to: lui $dst, %highest/%higher/%hi/%lo($tgt - $baltgt)
def LONG_BRANCH_LUi : PseudoSE<(outs GPR32Opnd:$dst),
  (ins brtarget:$tgt, brtarget:$baltgt), []> {
  bit hasNoSchedulingInfo = 1;
}
// Expands to: lui $dst, highest/%higher/%hi/%lo($tgt)
def LONG_BRANCH_LUi2Op : PseudoSE<(outs GPR32Opnd:$dst),
  (ins brtarget:$tgt), []> {
  bit hasNoSchedulingInfo = 1;
}

// Expands to: addiu $dst, $src, %highest/%higher/%hi/%lo($tgt - $baltgt)
def LONG_BRANCH_ADDiu : PseudoSE<(outs GPR32Opnd:$dst),
  (ins GPR32Opnd:$src, brtarget:$tgt, brtarget:$baltgt), []> {
  bit hasNoSchedulingInfo = 1;
}
// Expands to: addiu $dst, $src, %highest/%higher/%hi/%lo($tgt)
def LONG_BRANCH_ADDiu2Op : PseudoSE<(outs GPR32Opnd:$dst),
  (ins GPR32Opnd:$src, brtarget:$tgt), []> {
  bit hasNoSchedulingInfo = 1;
}

//===----------------------------------------------------------------------===//
// Instruction definition
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// FgpuI Instructions
//===----------------------------------------------------------------------===//

/// Arithmetic Instructions (ALU Immediate)
let AdditionalPredicates = [NotInMicroFgpu] in {
  def ADDiu : MMRel, StdMMR6Rel, ArithLogicI<"addiu", simm16_relaxed, GPR32Opnd,
                                             II_ADDIU, imm32SExt16, add>,
              ADDI_FM<0x9>, IsAsCheapAsAMove, ISA_FGPU1;

  def ANDi : MMRel, StdMMR6Rel,
             ArithLogicI<"andi", uimm16, GPR32Opnd, II_ANDI, imm32ZExt16, and>,
             ADDI_FM<0xc>, ISA_FGPU1;
  def ORi  : MMRel, StdMMR6Rel,
             ArithLogicI<"ori", uimm16, GPR32Opnd, II_ORI, imm32ZExt16, or>,
             ADDI_FM<0xd>, ISA_FGPU1;
  def XORi : MMRel, StdMMR6Rel,
             ArithLogicI<"xori", uimm16, GPR32Opnd, II_XORI, imm32ZExt16, xor>,
             ADDI_FM<0xe>, ISA_FGPU1;
  def ADDi  : MMRel, ArithLogicI<"addi", simm16_relaxed, GPR32Opnd, II_ADDI>,
              ADDI_FM<0x8>, ISA_FGPU1_NOT_32R6_64R6;
  def SLTi  : MMRel, SetCC_I<"slti", setlt, simm16, immSExt16, GPR32Opnd>,
              SLTI_FM<0xa>, ISA_FGPU1;
  def SLTiu : MMRel, SetCC_I<"sltiu", setult, simm16, immSExt16, GPR32Opnd>,
              SLTI_FM<0xb>, ISA_FGPU1;

  def LUi   : MMRel, LoadUpper<"lui", GPR32Opnd, uimm16_relaxed>, LUI_FM,
              ISA_FGPU1;

  /// Arithmetic Instructions (3-Operand, R-Type)
  def ADDu : MMRel, StdMMR6Rel, ArithLogicR<"addu", GPR32Opnd, 1, II_ADDU, add>,
             ADD_FM<0, 0x21>, ISA_FGPU1;
  def SUBu : MMRel, StdMMR6Rel, ArithLogicR<"subu", GPR32Opnd, 0, II_SUBU, sub>,
             ADD_FM<0, 0x23>, ISA_FGPU1;

  let Defs = [HI0, LO0] in
    def MUL   : MMRel, ArithLogicR<"mul", GPR32Opnd, 1, II_MUL, mul>,
                ADD_FM<0x1c, 2>, ISA_FGPU32_NOT_32R6_64R6;

  def ADD   : MMRel, StdMMR6Rel, ArithLogicR<"add", GPR32Opnd, 1, II_ADD>,
              ADD_FM<0, 0x20>, ISA_FGPU1;
  def SUB   : MMRel, StdMMR6Rel, ArithLogicR<"sub", GPR32Opnd, 0, II_SUB>,
              ADD_FM<0, 0x22>, ISA_FGPU1;

  def SLT   : MMRel, SetCC_R<"slt", setlt, GPR32Opnd>, ADD_FM<0, 0x2a>,
              ISA_FGPU1;
  def SLTu  : MMRel, SetCC_R<"sltu", setult, GPR32Opnd>, ADD_FM<0, 0x2b>,
              ISA_FGPU1;
  def AND   : MMRel, StdMMR6Rel, ArithLogicR<"and", GPR32Opnd, 1, II_AND, and>,
              ADD_FM<0, 0x24>, ISA_FGPU1;
  def OR    : MMRel, StdMMR6Rel, ArithLogicR<"or", GPR32Opnd, 1, II_OR, or>,
              ADD_FM<0, 0x25>, ISA_FGPU1;
  def XOR   : MMRel, StdMMR6Rel, ArithLogicR<"xor", GPR32Opnd, 1, II_XOR, xor>,
              ADD_FM<0, 0x26>, ISA_FGPU1;
  def NOR   : MMRel, StdMMR6Rel, LogicNOR<"nor", GPR32Opnd>, ADD_FM<0, 0x27>,
              ISA_FGPU1;
}

let AdditionalPredicates = [NotInMicroFgpu] in {
  /// Shift Instructions
  def SLL  : MMRel, shift_rotate_imm<"sll", uimm5, GPR32Opnd, II_SLL, shl,
                                     immZExt5>, SRA_FM<0, 0>, ISA_FGPU1;
  def SRL  : MMRel, shift_rotate_imm<"srl", uimm5, GPR32Opnd, II_SRL, srl,
                                     immZExt5>, SRA_FM<2, 0>, ISA_FGPU1;
  def SRA  : MMRel, shift_rotate_imm<"sra", uimm5, GPR32Opnd, II_SRA, sra,
                                     immZExt5>, SRA_FM<3, 0>, ISA_FGPU1;
  def SLLV : MMRel, shift_rotate_reg<"sllv", GPR32Opnd, II_SLLV, shl>,
             SRLV_FM<4, 0>, ISA_FGPU1;
  def SRLV : MMRel, shift_rotate_reg<"srlv", GPR32Opnd, II_SRLV, srl>,
             SRLV_FM<6, 0>, ISA_FGPU1;
  def SRAV : MMRel, shift_rotate_reg<"srav", GPR32Opnd, II_SRAV, sra>,
             SRLV_FM<7, 0>, ISA_FGPU1;

  // Rotate Instructions
  def ROTR  : MMRel, shift_rotate_imm<"rotr", uimm5, GPR32Opnd, II_ROTR, rotr,
                                      immZExt5>,
              SRA_FM<2, 1>, ISA_FGPU32R2;
  def ROTRV : MMRel, shift_rotate_reg<"rotrv", GPR32Opnd, II_ROTRV, rotr>,
              SRLV_FM<6, 1>, ISA_FGPU32R2;
}

/// Load and Store Instructions
///  aligned
let AdditionalPredicates = [NotInMicroFgpu] in {
  def LB  : LoadMemory<"lb", GPR32Opnd, mem_simmptr, sextloadi8, II_LB>, MMRel,
            LW_FM<0x20>, ISA_FGPU1;
  def LBu : LoadMemory<"lbu", GPR32Opnd, mem_simmptr, zextloadi8, II_LBU,
                       addrDefault>, MMRel, LW_FM<0x24>, ISA_FGPU1;
  def LH  : LoadMemory<"lh", GPR32Opnd, mem_simmptr, sextloadi16, II_LH,
                       addrDefault>, MMRel, LW_FM<0x21>, ISA_FGPU1;
  def LHu : LoadMemory<"lhu", GPR32Opnd, mem_simmptr, zextloadi16, II_LHU>,
            MMRel, LW_FM<0x25>, ISA_FGPU1;
  def LW  : StdMMR6Rel, Load<"lw", GPR32Opnd, load, II_LW, addrDefault>, MMRel,
            LW_FM<0x23>, ISA_FGPU1;
  def SB  : StdMMR6Rel, Store<"sb", GPR32Opnd, truncstorei8, II_SB>, MMRel,
            LW_FM<0x28>, ISA_FGPU1;
  def SH  : Store<"sh", GPR32Opnd, truncstorei16, II_SH>, MMRel, LW_FM<0x29>,
            ISA_FGPU1;
  def SW  : StdMMR6Rel, Store<"sw", GPR32Opnd, store, II_SW>,
            MMRel, LW_FM<0x2b>, ISA_FGPU1;
}

/// load/store left/right
let AdditionalPredicates = [NotInMicroFgpu] in {
def LWL : MMRel, LoadLeftRight<"lwl", FgpuLWL, GPR32Opnd, II_LWL>, LW_FM<0x22>,
          ISA_FGPU1_NOT_32R6_64R6;
def LWR : MMRel, LoadLeftRight<"lwr", FgpuLWR, GPR32Opnd, II_LWR>, LW_FM<0x26>,
          ISA_FGPU1_NOT_32R6_64R6;
def SWL : MMRel, StoreLeftRight<"swl", FgpuSWL, GPR32Opnd, II_SWL>, LW_FM<0x2a>,
          ISA_FGPU1_NOT_32R6_64R6;
def SWR : MMRel, StoreLeftRight<"swr", FgpuSWR, GPR32Opnd, II_SWR>, LW_FM<0x2e>,
          ISA_FGPU1_NOT_32R6_64R6;

// COP2 Memory Instructions
def LWC2 : StdMMR6Rel, LW_FT2<"lwc2", COP2Opnd, II_LWC2, load>, LW_FM<0x32>,
           ISA_FGPU1_NOT_32R6_64R6;
def SWC2 : StdMMR6Rel, SW_FT2<"swc2", COP2Opnd, II_SWC2, store>,
           LW_FM<0x3a>, ISA_FGPU1_NOT_32R6_64R6;
def LDC2 : StdMMR6Rel, LW_FT2<"ldc2", COP2Opnd, II_LDC2, load>, LW_FM<0x36>,
           ISA_FGPU2_NOT_32R6_64R6;
def SDC2 : StdMMR6Rel, SW_FT2<"sdc2", COP2Opnd, II_SDC2, store>,
           LW_FM<0x3e>, ISA_FGPU2_NOT_32R6_64R6;

// COP3 Memory Instructions
let DecoderNamespace = "COP3_" in {
  def LWC3 : LW_FT3<"lwc3", COP3Opnd, II_LWC3, load>, LW_FM<0x33>,
             ISA_FGPU1_NOT_32R6_64R6, NOT_ASE_CNFGPU;
  def SWC3 : SW_FT3<"swc3", COP3Opnd, II_SWC3, store>, LW_FM<0x3b>,
             ISA_FGPU1_NOT_32R6_64R6, NOT_ASE_CNFGPU;
  def LDC3 : LW_FT3<"ldc3", COP3Opnd, II_LDC3, load>, LW_FM<0x37>,
             ISA_FGPU2, NOT_ASE_CNFGPU;
  def SDC3 : SW_FT3<"sdc3", COP3Opnd, II_SDC3, store>, LW_FM<0x3f>,
             ISA_FGPU2, NOT_ASE_CNFGPU;
}

  def SYNC : MMRel, StdMMR6Rel, SYNC_FT<"sync">, SYNC_FM, ISA_FGPU2;
  def SYNCI : MMRel, StdMMR6Rel, SYNCI_FT<"synci", mem_simm16>, SYNCI_FM,
              ISA_FGPU32R2;
}

let AdditionalPredicates = [NotInMicroFgpu] in {
  def TEQ : MMRel, TEQ_FT<"teq", GPR32Opnd, uimm10, II_TEQ>, TEQ_FM<0x34>,
            ISA_FGPU2;
  def TGE : MMRel, TEQ_FT<"tge", GPR32Opnd, uimm10, II_TGE>, TEQ_FM<0x30>,
            ISA_FGPU2;
  def TGEU : MMRel, TEQ_FT<"tgeu", GPR32Opnd, uimm10, II_TGEU>, TEQ_FM<0x31>,
             ISA_FGPU2;
  def TLT : MMRel, TEQ_FT<"tlt", GPR32Opnd, uimm10, II_TLT>, TEQ_FM<0x32>,
            ISA_FGPU2;
  def TLTU : MMRel, TEQ_FT<"tltu", GPR32Opnd, uimm10, II_TLTU>, TEQ_FM<0x33>,
            ISA_FGPU2;
  def TNE : MMRel, TEQ_FT<"tne", GPR32Opnd, uimm10, II_TNE>, TEQ_FM<0x36>,
            ISA_FGPU2;

  def TEQI : MMRel, TEQI_FT<"teqi", GPR32Opnd, II_TEQI>, TEQI_FM<0xc>,
             ISA_FGPU2_NOT_32R6_64R6;
  def TGEI : MMRel, TEQI_FT<"tgei", GPR32Opnd, II_TGEI>, TEQI_FM<0x8>,
             ISA_FGPU2_NOT_32R6_64R6;
  def TGEIU : MMRel, TEQI_FT<"tgeiu", GPR32Opnd, II_TGEIU>, TEQI_FM<0x9>,
              ISA_FGPU2_NOT_32R6_64R6;
  def TLTI : MMRel, TEQI_FT<"tlti", GPR32Opnd, II_TLTI>, TEQI_FM<0xa>,
             ISA_FGPU2_NOT_32R6_64R6;
  def TTLTIU : MMRel, TEQI_FT<"tltiu", GPR32Opnd, II_TTLTIU>, TEQI_FM<0xb>,
               ISA_FGPU2_NOT_32R6_64R6;
  def TNEI : MMRel, TEQI_FT<"tnei", GPR32Opnd, II_TNEI>, TEQI_FM<0xe>,
             ISA_FGPU2_NOT_32R6_64R6;
}

let AdditionalPredicates = [NotInMicroFgpu] in {
  def BREAK : MMRel, StdMMR6Rel, BRK_FT<"break">, BRK_FM<0xd>, ISA_FGPU1;
  def SYSCALL : MMRel, SYS_FT<"syscall", uimm20, II_SYSCALL>, SYS_FM<0xc>,
                ISA_FGPU1;
  def TRAP : TrapBase<BREAK>, ISA_FGPU1;
  def SDBBP : MMRel, SYS_FT<"sdbbp", uimm20, II_SDBBP>, SDBBP_FM,
              ISA_FGPU32_NOT_32R6_64R6;

  def ERET : MMRel, ER_FT<"eret", II_ERET>, ER_FM<0x18, 0x0>, INSN_FGPU3_32;
  def ERETNC : MMRel, ER_FT<"eretnc", II_ERETNC>, ER_FM<0x18, 0x1>,
               ISA_FGPU32R5;
  def DERET : MMRel, ER_FT<"deret", II_DERET>, ER_FM<0x1f, 0x0>, ISA_FGPU32;

  def EI : MMRel, StdMMR6Rel, DEI_FT<"ei", GPR32Opnd, II_EI>, EI_FM<1>,
           ISA_FGPU32R2;
  def DI : MMRel, StdMMR6Rel, DEI_FT<"di", GPR32Opnd, II_DI>, EI_FM<0>,
           ISA_FGPU32R2;

  def WAIT : MMRel, StdMMR6Rel, WAIT_FT<"wait">, WAIT_FM, INSN_FGPU3_32;
}

let AdditionalPredicates = [NotInMicroFgpu] in {
/// Load-linked, Store-conditional
def LL : LLBase<"ll", GPR32Opnd>, LW_FM<0x30>, PTR_32, ISA_FGPU2_NOT_32R6_64R6;
def SC : SCBase<"sc", GPR32Opnd>, LW_FM<0x38>, PTR_32, ISA_FGPU2_NOT_32R6_64R6;
}
/// Jump and Branch Instructions
let AdditionalPredicates = [NotInMicroFgpu, RelocNotPIC] in
def J       : MMRel, JumpFJ<jmptarget, "j", br, bb, "j">, FJ<2>,
              IsBranch, ISA_FGPU1;

let AdditionalPredicates = [NotInMicroFgpu] in {
def JR      : MMRel, IndirectBranch<"jr", GPR32Opnd>, MTLO_FM<8>,
              ISA_FGPU1_NOT_32R6_64R6;
def BEQ     : MMRel, CBranch<"beq", brtarget, seteq, GPR32Opnd>, BEQ_FM<4>,
              ISA_FGPU1;
def BEQL    : MMRel, CBranchLikely<"beql", brtarget, GPR32Opnd>,
              BEQ_FM<20>, ISA_FGPU2_NOT_32R6_64R6;
def BNE     : MMRel, CBranch<"bne", brtarget, setne, GPR32Opnd>, BEQ_FM<5>,
              ISA_FGPU1;
def BNEL    : MMRel, CBranchLikely<"bnel", brtarget, GPR32Opnd>,
              BEQ_FM<21>, ISA_FGPU2_NOT_32R6_64R6;
def BGEZ    : MMRel, CBranchZero<"bgez", brtarget, setge, GPR32Opnd>,
              BGEZ_FM<1, 1>, ISA_FGPU1;
def BGEZL   : MMRel, CBranchZeroLikely<"bgezl", brtarget, GPR32Opnd>,
              BGEZ_FM<1, 3>, ISA_FGPU2_NOT_32R6_64R6;
def BGTZ    : MMRel, CBranchZero<"bgtz", brtarget, setgt, GPR32Opnd>,
              BGEZ_FM<7, 0>, ISA_FGPU1;
def BGTZL   : MMRel, CBranchZeroLikely<"bgtzl", brtarget, GPR32Opnd>,
              BGEZ_FM<23, 0>, ISA_FGPU2_NOT_32R6_64R6;
def BLEZ    : MMRel, CBranchZero<"blez", brtarget, setle, GPR32Opnd>,
              BGEZ_FM<6, 0>, ISA_FGPU1;
def BLEZL   : MMRel, CBranchZeroLikely<"blezl", brtarget, GPR32Opnd>,
              BGEZ_FM<22, 0>, ISA_FGPU2_NOT_32R6_64R6;
def BLTZ    : MMRel, CBranchZero<"bltz", brtarget, setlt, GPR32Opnd>,
              BGEZ_FM<1, 0>, ISA_FGPU1;
def BLTZL   : MMRel, CBranchZeroLikely<"bltzl", brtarget, GPR32Opnd>,
              BGEZ_FM<1, 2>, ISA_FGPU2_NOT_32R6_64R6;
def B       : UncondBranch<BEQ, brtarget>, ISA_FGPU1;

def JAL  : MMRel, JumpLink<"jal", calltarget>, FJ<3>, ISA_FGPU1;

}

let AdditionalPredicates = [NotInMicroFgpu, NoIndirectJumpGuards] in {
  def JALR : JumpLinkReg<"jalr", GPR32Opnd>, JALR_FM, ISA_FGPU1;
  def JALRPseudo : JumpLinkRegPseudo<GPR32Opnd, JALR, RA>, ISA_FGPU1;
}

let AdditionalPredicates = [NotInMicroFgpu] in {
  def JALX : MMRel, JumpLink<"jalx", calltarget>, FJ<0x1D>,
             ISA_FGPU32_NOT_32R6_64R6;
  def BGEZAL : MMRel, BGEZAL_FT<"bgezal", brtarget, GPR32Opnd>, BGEZAL_FM<0x11>,
               ISA_FGPU1_NOT_32R6_64R6;
  def BGEZALL : MMRel, BGEZAL_FT<"bgezall", brtarget, GPR32Opnd>,
                BGEZAL_FM<0x13>, ISA_FGPU2_NOT_32R6_64R6;
  def BLTZAL : MMRel, BGEZAL_FT<"bltzal", brtarget, GPR32Opnd>, BGEZAL_FM<0x10>,
               ISA_FGPU1_NOT_32R6_64R6;
  def BLTZALL : MMRel, BGEZAL_FT<"bltzall", brtarget, GPR32Opnd>,
                BGEZAL_FM<0x12>, ISA_FGPU2_NOT_32R6_64R6;
  def BAL_BR : BAL_BR_Pseudo<BGEZAL, brtarget>, ISA_FGPU1;
}
let AdditionalPredicates = [NotInFgpu16Mode, NotInMicroFgpu] in {
  def TAILCALL : TailCall<J, jmptarget>, ISA_FGPU1;
}
let AdditionalPredicates = [NotInFgpu16Mode, NotInMicroFgpu,
                            NoIndirectJumpGuards] in
  def TAILCALLREG : TailCallReg<JR, GPR32Opnd>, ISA_FGPU1_NOT_32R6_64R6;

// Indirect branches are matched as PseudoIndirectBranch/PseudoIndirectBranch64
// then are expanded to JR, JR64, JALR, or JALR64 depending on the ISA.
class PseudoIndirectBranchBase<Instruction JumpInst, RegisterOperand RO> :
    FgpuPseudo<(outs), (ins RO:$rs), [(brind RO:$rs)],
               II_IndirectBranchPseudo>,
    PseudoInstExpansion<(JumpInst RO:$rs)> {
  let isTerminator=1;
  let isBarrier=1;
  let hasDelaySlot = 1;
  let isBranch = 1;
  let isIndirectBranch = 1;
  bit isCTI = 1;
}

let AdditionalPredicates = [NotInFgpu16Mode, NotInMicroFgpu,
                            NoIndirectJumpGuards] in
  def PseudoIndirectBranch : PseudoIndirectBranchBase<JR, GPR32Opnd>,
                             ISA_FGPU1_NOT_32R6_64R6;

// Return instructions are matched as a RetRA instruction, then are expanded
// into PseudoReturn/PseudoReturn64 after register allocation. Finally,
// FgpuAsmPrinter expands this into JR, JR64, JALR, or JALR64 depending on the
// ISA.
class PseudoReturnBase<RegisterOperand RO> : FgpuPseudo<(outs), (ins RO:$rs),
                                                        [], II_ReturnPseudo> {
  let isTerminator = 1;
  let isBarrier = 1;
  let hasDelaySlot = 1;
  let isReturn = 1;
  let isCodeGenOnly = 1;
  let hasCtrlDep = 1;
  let hasExtraSrcRegAllocReq = 1;
  bit isCTI = 1;
}

def PseudoReturn : PseudoReturnBase<GPR32Opnd>;

// Exception handling related node and instructions.
// The conversion sequence is:
// ISD::EH_RETURN -> FgpuISD::EH_RETURN ->
// FGPUeh_return -> (stack change + indirect branch)
//
// FGPUeh_return takes the place of regular return instruction
// but takes two arguments (V1, V0) which are used for storing
// the offset and return address respectively.
def SDT_FgpuEHRET : SDTypeProfile<0, 2, [SDTCisInt<0>, SDTCisPtrTy<1>]>;

def FGPUehret : SDNode<"FgpuISD::EH_RETURN", SDT_FgpuEHRET,
                      [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

let Uses = [V0, V1], isTerminator = 1, isReturn = 1,
           isBarrier = 1, isCTI = 1, hasNoSchedulingInfo = 1 in {
  def FGPUeh_return32 : FgpuPseudo<(outs), (ins GPR32:$spoff, GPR32:$dst),
                                   [(FGPUehret GPR32:$spoff, GPR32:$dst)]>;
  def FGPUeh_return64 : FgpuPseudo<(outs), (ins GPR64:$spoff, GPR64:$dst),
                                   [(FGPUehret GPR64:$spoff, GPR64:$dst)]>;
}

/// Multiply and Divide Instructions.
let AdditionalPredicates = [NotInMicroFgpu] in {
  def MULT  : MMRel, Mult<"mult", II_MULT, GPR32Opnd, [HI0, LO0]>,
              MULT_FM<0, 0x18>, ISA_FGPU1_NOT_32R6_64R6;
  def MULTu : MMRel, Mult<"multu", II_MULTU, GPR32Opnd, [HI0, LO0]>,
              MULT_FM<0, 0x19>, ISA_FGPU1_NOT_32R6_64R6;
  def SDIV  : MMRel, Div<"div", II_DIV, GPR32Opnd, [HI0, LO0]>,
              MULT_FM<0, 0x1a>, ISA_FGPU1_NOT_32R6_64R6;
  def UDIV  : MMRel, Div<"divu", II_DIVU, GPR32Opnd, [HI0, LO0]>,
              MULT_FM<0, 0x1b>, ISA_FGPU1_NOT_32R6_64R6;
  def MTHI : MMRel, MoveToLOHI<"mthi", GPR32Opnd, [HI0]>, MTLO_FM<0x11>,
             ISA_FGPU1_NOT_32R6_64R6;
  def MTLO : MMRel, MoveToLOHI<"mtlo", GPR32Opnd, [LO0]>, MTLO_FM<0x13>,
             ISA_FGPU1_NOT_32R6_64R6;
  def MFHI : MMRel, MoveFromLOHI<"mfhi", GPR32Opnd, AC0>, MFLO_FM<0x10>,
             ISA_FGPU1_NOT_32R6_64R6;
  def MFLO : MMRel, MoveFromLOHI<"mflo", GPR32Opnd, AC0>, MFLO_FM<0x12>,
             ISA_FGPU1_NOT_32R6_64R6;

  /// Sign Ext In Register Instructions.
  def SEB : MMRel, StdMMR6Rel, SignExtInReg<"seb", i8, GPR32Opnd, II_SEB>,
            SEB_FM<0x10, 0x20>, ISA_FGPU32R2;
  def SEH : MMRel, StdMMR6Rel, SignExtInReg<"seh", i16, GPR32Opnd, II_SEH>,
            SEB_FM<0x18, 0x20>, ISA_FGPU32R2;

  /// Count Leading
  def CLZ : MMRel, CountLeading0<"clz", GPR32Opnd, II_CLZ>, CLO_FM<0x20>,
            ISA_FGPU32_NOT_32R6_64R6;
  def CLO : MMRel, CountLeading1<"clo", GPR32Opnd, II_CLO>, CLO_FM<0x21>,
            ISA_FGPU32_NOT_32R6_64R6;

  /// Word Swap Bytes Within Halfwords
  def WSBH : MMRel, SubwordSwap<"wsbh", GPR32Opnd, II_WSBH>, SEB_FM<2, 0x20>,
             ISA_FGPU32R2;

  /// No operation.
  def NOP : PseudoSE<(outs), (ins), []>,
                     PseudoInstExpansion<(SLL ZERO, ZERO, 0)>, ISA_FGPU1;

  // FrameIndexes are legalized when they are operands from load/store
  // instructions. The same not happens for stack address copies, so an
  // add op with mem ComplexPattern is used and the stack address copy
  // can be matched. It's similar to Sparc LEA_ADDRi
  let AdditionalPredicates = [NotInMicroFgpu] in
    def LEA_ADDiu : MMRel, EffectiveAddress<"addiu", GPR32Opnd>, LW_FM<9>,
                    ISA_FGPU1;

  // MADD*/MSUB*
  def MADD  : MMRel, MArithR<"madd", II_MADD, 1>, MULT_FM<0x1c, 0>,
              ISA_FGPU32_NOT_32R6_64R6;
  def MADDU : MMRel, MArithR<"maddu", II_MADDU, 1>, MULT_FM<0x1c, 1>,
              ISA_FGPU32_NOT_32R6_64R6;
  def MSUB  : MMRel, MArithR<"msub", II_MSUB>, MULT_FM<0x1c, 4>,
              ISA_FGPU32_NOT_32R6_64R6;
  def MSUBU : MMRel, MArithR<"msubu", II_MSUBU>, MULT_FM<0x1c, 5>,
              ISA_FGPU32_NOT_32R6_64R6;
}

let AdditionalPredicates = [NotDSP] in {
def PseudoMULT  : MultDivPseudo<MULT, ACC64, GPR32Opnd, FgpuMult, II_MULT>,
                  ISA_FGPU1_NOT_32R6_64R6;
def PseudoMULTu : MultDivPseudo<MULTu, ACC64, GPR32Opnd, FgpuMultu, II_MULTU>,
                  ISA_FGPU1_NOT_32R6_64R6;
def PseudoMFHI : PseudoMFLOHI<GPR32, ACC64, FgpuMFHI>, ISA_FGPU1_NOT_32R6_64R6;
def PseudoMFLO : PseudoMFLOHI<GPR32, ACC64, FgpuMFLO>, ISA_FGPU1_NOT_32R6_64R6;
def PseudoMTLOHI : PseudoMTLOHI<ACC64, GPR32>, ISA_FGPU1_NOT_32R6_64R6;
def PseudoMADD  : MAddSubPseudo<MADD, FgpuMAdd, II_MADD>,
                  ISA_FGPU32_NOT_32R6_64R6;
def PseudoMADDU : MAddSubPseudo<MADDU, FgpuMAddu, II_MADDU>,
                  ISA_FGPU32_NOT_32R6_64R6;
def PseudoMSUB  : MAddSubPseudo<MSUB, FgpuMSub, II_MSUB>,
                  ISA_FGPU32_NOT_32R6_64R6;
def PseudoMSUBU : MAddSubPseudo<MSUBU, FgpuMSubu, II_MSUBU>,
                  ISA_FGPU32_NOT_32R6_64R6;
}

let AdditionalPredicates = [NotInMicroFgpu] in {
  def PseudoSDIV : MultDivPseudo<SDIV, ACC64, GPR32Opnd, FgpuDivRem, II_DIV,
                                 0, 1, 1>, ISA_FGPU1_NOT_32R6_64R6;
  def PseudoUDIV : MultDivPseudo<UDIV, ACC64, GPR32Opnd, FgpuDivRemU, II_DIVU,
                                 0, 1, 1>, ISA_FGPU1_NOT_32R6_64R6;
  def RDHWR : MMRel, ReadHardware<GPR32Opnd, HWRegsOpnd>, RDHWR_FM, ISA_FGPU1;
  // TODO: Add '0 < pos+size <= 32' constraint check to ext instruction
  def EXT : MMRel, StdMMR6Rel, ExtBase<"ext", GPR32Opnd, uimm5, uimm5_plus1,
                                       immZExt5, immZExt5Plus1, FgpuExt>,
            EXT_FM<0>, ISA_FGPU32R2;
  def INS : MMRel, StdMMR6Rel, InsBase<"ins", GPR32Opnd, uimm5,
                                       uimm5_inssize_plus1, immZExt5,
                                       immZExt5Plus1>,
            EXT_FM<4>, ISA_FGPU32R2;
}
/// Move Control Registers From/To CPU Registers
let AdditionalPredicates = [NotInMicroFgpu] in {
  def MTC0 : MTC3OP<"mtc0", COP0Opnd, GPR32Opnd, II_MTC0>,
             MFC3OP_FM<0x10, 4, 0>, ISA_FGPU1;
  def MFC0 : MFC3OP<"mfc0", GPR32Opnd, COP0Opnd, II_MFC0>,
             MFC3OP_FM<0x10, 0, 0>, ISA_FGPU1;
  def MFC2 : MFC3OP<"mfc2", GPR32Opnd, COP2Opnd, II_MFC2>,
             MFC3OP_FM<0x12, 0, 0>, ISA_FGPU1;
  def MTC2 : MTC3OP<"mtc2", COP2Opnd, GPR32Opnd, II_MTC2>,
             MFC3OP_FM<0x12, 4, 0>, ISA_FGPU1;
}

class Barrier<string asmstr, InstrItinClass itin = NoItinerary> :
  InstSE<(outs), (ins), asmstr, [], itin, FrmOther, asmstr>;
let AdditionalPredicates = [NotInMicroFgpu] in {
  def SSNOP : MMRel, StdMMR6Rel, Barrier<"ssnop", II_SSNOP>, BARRIER_FM<1>,
              ISA_FGPU1;
  def EHB : MMRel, Barrier<"ehb", II_EHB>, BARRIER_FM<3>, ISA_FGPU1;

  let isCTI = 1 in
  def PAUSE : MMRel, StdMMR6Rel, Barrier<"pause", II_PAUSE>, BARRIER_FM<5>,
              ISA_FGPU32R2;
}

// JR_HB and JALR_HB are defined here using the new style naming
// scheme because some of this code is shared with Fgpu32r6InstrInfo.td
// and because of that it doesn't follow the naming convention of the
// rest of the file. To avoid a mixture of old vs new style, the new
// style was chosen.
class JR_HB_DESC_BASE<string instr_asm, RegisterOperand GPROpnd> {
  dag OutOperandList = (outs);
  dag InOperandList = (ins GPROpnd:$rs);
  string AsmString = !strconcat(instr_asm, "\t$rs");
  list<dag> Pattern = [];
}

class JALR_HB_DESC_BASE<string instr_asm, RegisterOperand GPROpnd> {
  dag OutOperandList = (outs GPROpnd:$rd);
  dag InOperandList = (ins GPROpnd:$rs);
  string AsmString = !strconcat(instr_asm, "\t$rd, $rs");
  list<dag> Pattern = [];
}

class JR_HB_DESC<RegisterOperand RO> :
  InstSE<(outs), (ins), "", [], II_JR_HB, FrmJ>, JR_HB_DESC_BASE<"jr.hb", RO> {
  let isBranch=1;
  let isIndirectBranch=1;
  let hasDelaySlot=1;
  let isTerminator=1;
  let isBarrier=1;
  bit isCTI = 1;
}

class JALR_HB_DESC<RegisterOperand RO> :
  InstSE<(outs), (ins), "", [], II_JALR_HB, FrmJ>, JALR_HB_DESC_BASE<"jalr.hb",
                                                                     RO> {
  let isIndirectBranch=1;
  let hasDelaySlot=1;
  bit isCTI = 1;
}

class JR_HB_ENC : JR_HB_FM<8>;
class JALR_HB_ENC : JALR_HB_FM<9>;

def JR_HB : JR_HB_DESC<GPR32Opnd>, JR_HB_ENC, ISA_FGPU32R2_NOT_32R6_64R6;
def JALR_HB : JALR_HB_DESC<GPR32Opnd>, JALR_HB_ENC, ISA_FGPU32;

let AdditionalPredicates = [NotInMicroFgpu, UseIndirectJumpsHazard] in
  def JALRHBPseudo : JumpLinkRegPseudo<GPR32Opnd, JALR_HB, RA>;


let AdditionalPredicates = [NotInFgpu16Mode, NotInMicroFgpu,
                            UseIndirectJumpsHazard] in {
  def TAILCALLREGHB : TailCallReg<JR_HB, GPR32Opnd>, ISA_FGPU32_NOT_32R6_64R6;
  def PseudoIndirectHazardBranch : PseudoIndirectBranchBase<JR_HB, GPR32Opnd>,
                                   ISA_FGPU32R2_NOT_32R6_64R6;
}

class TLB<string asmstr, InstrItinClass itin = NoItinerary> :
  InstSE<(outs), (ins), asmstr, [], itin, FrmOther, asmstr>;
let AdditionalPredicates = [NotInMicroFgpu] in {
  def TLBP : MMRel, TLB<"tlbp", II_TLBP>, COP0_TLB_FM<0x08>, ISA_FGPU1;
  def TLBR : MMRel, TLB<"tlbr", II_TLBR>, COP0_TLB_FM<0x01>, ISA_FGPU1;
  def TLBWI : MMRel, TLB<"tlbwi", II_TLBWI>, COP0_TLB_FM<0x02>, ISA_FGPU1;
  def TLBWR : MMRel, TLB<"tlbwr", II_TLBWR>, COP0_TLB_FM<0x06>, ISA_FGPU1;
}
class CacheOp<string instr_asm, Operand MemOpnd,
              InstrItinClass itin = NoItinerary> :
    InstSE<(outs), (ins  MemOpnd:$addr, uimm5:$hint),
           !strconcat(instr_asm, "\t$hint, $addr"), [], itin, FrmOther,
           instr_asm> {
  let DecoderMethod = "DecodeCacheOp";
}

let AdditionalPredicates = [NotInMicroFgpu] in {
  def CACHE : MMRel, CacheOp<"cache", mem, II_CACHE>, CACHEOP_FM<0b101111>,
              INSN_FGPU3_32_NOT_32R6_64R6;
  def PREF :  MMRel, CacheOp<"pref", mem, II_PREF>, CACHEOP_FM<0b110011>,
              INSN_FGPU3_32_NOT_32R6_64R6;
}
// FIXME: We are missing the prefx instruction.
def ROL : FgpuAsmPseudoInst<(outs),
                            (ins GPR32Opnd:$rs, GPR32Opnd:$rt, GPR32Opnd:$rd),
                            "rol\t$rs, $rt, $rd">;
def ROLImm : FgpuAsmPseudoInst<(outs),
                               (ins GPR32Opnd:$rs, GPR32Opnd:$rt, simm16:$imm),
                               "rol\t$rs, $rt, $imm">;
def : FgpuInstAlias<"rol $rd, $rs",
                    (ROL GPR32Opnd:$rd, GPR32Opnd:$rd, GPR32Opnd:$rs), 0>;
def : FgpuInstAlias<"rol $rd, $imm",
                    (ROLImm GPR32Opnd:$rd, GPR32Opnd:$rd, simm16:$imm), 0>;

def ROR : FgpuAsmPseudoInst<(outs),
                            (ins GPR32Opnd:$rs, GPR32Opnd:$rt, GPR32Opnd:$rd),
                            "ror\t$rs, $rt, $rd">;
def RORImm : FgpuAsmPseudoInst<(outs),
                               (ins GPR32Opnd:$rs, GPR32Opnd:$rt, simm16:$imm),
                               "ror\t$rs, $rt, $imm">;
def : FgpuInstAlias<"ror $rd, $rs",
                    (ROR GPR32Opnd:$rd, GPR32Opnd:$rd, GPR32Opnd:$rs), 0>;
def : FgpuInstAlias<"ror $rd, $imm",
                    (RORImm GPR32Opnd:$rd, GPR32Opnd:$rd, simm16:$imm), 0>;

def DROL : FgpuAsmPseudoInst<(outs),
                             (ins GPR32Opnd:$rs, GPR32Opnd:$rt, GPR32Opnd:$rd),
                             "drol\t$rs, $rt, $rd">, ISA_FGPU64;
def DROLImm : FgpuAsmPseudoInst<(outs),
                                (ins GPR32Opnd:$rs, GPR32Opnd:$rt, simm16:$imm),
                                "drol\t$rs, $rt, $imm">, ISA_FGPU64;
def : FgpuInstAlias<"drol $rd, $rs",
                    (DROL GPR32Opnd:$rd, GPR32Opnd:$rd, GPR32Opnd:$rs), 0>,
      ISA_FGPU64;
def : FgpuInstAlias<"drol $rd, $imm",
                    (DROLImm GPR32Opnd:$rd, GPR32Opnd:$rd, simm16:$imm), 0>,
      ISA_FGPU64;

def DROR : FgpuAsmPseudoInst<(outs),
                             (ins GPR32Opnd:$rs, GPR32Opnd:$rt, GPR32Opnd:$rd),
                             "dror\t$rs, $rt, $rd">, ISA_FGPU64;
def DRORImm : FgpuAsmPseudoInst<(outs),
                                (ins GPR32Opnd:$rs, GPR32Opnd:$rt, simm16:$imm),
                                "dror\t$rs, $rt, $imm">, ISA_FGPU64;
def : FgpuInstAlias<"dror $rd, $rs",
                    (DROR GPR32Opnd:$rd, GPR32Opnd:$rd, GPR32Opnd:$rs), 0>,
      ISA_FGPU64;
def : FgpuInstAlias<"dror $rd, $imm",
                    (DRORImm GPR32Opnd:$rd, GPR32Opnd:$rd, simm16:$imm), 0>,
      ISA_FGPU64;

def ABSMacro : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd), (ins GPR32Opnd:$rs),
                                 "abs\t$rd, $rs">;

def SEQMacro : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                 (ins GPR32Opnd:$rs, GPR32Opnd:$rt),
                                 "seq $rd, $rs, $rt">, NOT_ASE_CNFGPU;

def : FgpuInstAlias<"seq $rd, $rs",
                    (SEQMacro GPR32Opnd:$rd, GPR32Opnd:$rd, GPR32Opnd:$rs), 0>,
                    NOT_ASE_CNFGPU;

def SEQIMacro : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                  (ins GPR32Opnd:$rs, simm32_relaxed:$imm),
                                  "seq $rd, $rs, $imm">, NOT_ASE_CNFGPU;

def : FgpuInstAlias<"seq $rd, $imm",
                    (SEQIMacro GPR32Opnd:$rd, GPR32Opnd:$rd, simm32:$imm), 0>,
                    NOT_ASE_CNFGPU;

def SNEMacro : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                 (ins GPR32Opnd:$rs, GPR32Opnd:$rt),
                                 "sne $rd, $rs, $rt">, NOT_ASE_CNFGPU;

def : FgpuInstAlias<"sne $rd, $rs",
                    (SNEMacro GPR32Opnd:$rd, GPR32Opnd:$rd, GPR32Opnd:$rs), 0>,
                    NOT_ASE_CNFGPU;

def SNEIMacro : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                  (ins GPR32Opnd:$rs, simm32_relaxed:$imm),
                                  "sne $rd, $rs, $imm">, NOT_ASE_CNFGPU;

def : FgpuInstAlias<"sne $rd, $imm",
                    (SNEIMacro GPR32Opnd:$rd, GPR32Opnd:$rd, simm32:$imm), 0>,
                    NOT_ASE_CNFGPU;

def MULImmMacro : FgpuAsmPseudoInst<(outs), (ins GPR32Opnd:$rd, GPR32Opnd:$rs,
                                                 simm32_relaxed:$imm),
                                    "mul\t$rd, $rs, $imm">,
                  ISA_FGPU1_NOT_32R6_64R6;
def MULOMacro : FgpuAsmPseudoInst<(outs), (ins GPR32Opnd:$rd, GPR32Opnd:$rs,
                                               GPR32Opnd:$rt),
                                  "mulo\t$rd, $rs, $rt">,
                ISA_FGPU1_NOT_32R6_64R6;
def MULOUMacro : FgpuAsmPseudoInst<(outs), (ins GPR32Opnd:$rd, GPR32Opnd:$rs,
                                                GPR32Opnd:$rt),
                                   "mulou\t$rd, $rs, $rt">,
                 ISA_FGPU1_NOT_32R6_64R6;

// Virtualization ASE
class HYPCALL_FT<string opstr> :
  InstSE<(outs), (ins uimm10:$code_),
         !strconcat(opstr, "\t$code_"), [], II_HYPCALL, FrmOther, opstr> {
  let BaseOpcode = opstr;
}

let AdditionalPredicates = [NotInMicroFgpu] in {
  def MFGC0    : MMRel, MFC3OP<"mfgc0", GPR32Opnd, COP0Opnd, II_MFGC0>,
                 MFC3OP_FM<0x10, 3, 0>, ISA_FGPU32R5, ASE_VIRT;
  def MTGC0    : MMRel, MTC3OP<"mtgc0", COP0Opnd, GPR32Opnd, II_MTGC0>,
                 MFC3OP_FM<0x10, 3, 2>, ISA_FGPU32R5, ASE_VIRT;
  def MFHGC0   : MMRel, MFC3OP<"mfhgc0", GPR32Opnd, COP0Opnd, II_MFHGC0>,
                 MFC3OP_FM<0x10, 3, 4>, ISA_FGPU32R5, ASE_VIRT;
  def MTHGC0   : MMRel, MTC3OP<"mthgc0", COP0Opnd, GPR32Opnd, II_MTHGC0>,
                 MFC3OP_FM<0x10, 3, 6>, ISA_FGPU32R5, ASE_VIRT;
  def TLBGINV  : MMRel, TLB<"tlbginv", II_TLBGINV>, COP0_TLB_FM<0b001011>,
                 ISA_FGPU32R5, ASE_VIRT;
  def TLBGINVF : MMRel, TLB<"tlbginvf", II_TLBGINVF>, COP0_TLB_FM<0b001100>,
                 ISA_FGPU32R5, ASE_VIRT;
  def TLBGP    : MMRel, TLB<"tlbgp", II_TLBGP>, COP0_TLB_FM<0b010000>,
                 ISA_FGPU32R5, ASE_VIRT;
  def TLBGR    : MMRel, TLB<"tlbgr", II_TLBGR>, COP0_TLB_FM<0b001001>,
                 ISA_FGPU32R5, ASE_VIRT;
  def TLBGWI   : MMRel, TLB<"tlbgwi", II_TLBGWI>, COP0_TLB_FM<0b001010>,
                 ISA_FGPU32R5, ASE_VIRT;
  def TLBGWR   : MMRel, TLB<"tlbgwr", II_TLBGWR>, COP0_TLB_FM<0b001110>,
                 ISA_FGPU32R5, ASE_VIRT;
  def HYPCALL  : MMRel, HYPCALL_FT<"hypcall">,
                 HYPCALL_FM<0b101000>, ISA_FGPU32R5, ASE_VIRT;
}

//===----------------------------------------------------------------------===//
// Instruction aliases
//===----------------------------------------------------------------------===//

multiclass OneOrTwoOperandMacroImmediateAlias<string Memnomic,
                                              Instruction Opcode,
                                              RegisterOperand RO = GPR32Opnd,
                                              Operand Imm = simm32_relaxed> {
  def : FgpuInstAlias<!strconcat(Memnomic, " $rs, $rt, $imm"),
                                (Opcode RO:$rs,
                                        RO:$rt,
                                        Imm:$imm), 0>;
  def : FgpuInstAlias<!strconcat(Memnomic, " $rs, $imm"),
                                (Opcode RO:$rs,
                                        RO:$rs,
                                        Imm:$imm), 0>;
}

let AdditionalPredicates = [NotInMicroFgpu] in {
  def : FgpuInstAlias<"move $dst, $src",
                      (OR GPR32Opnd:$dst, GPR32Opnd:$src, ZERO), 1>,
        GPR_32, ISA_FGPU1;
  def : FgpuInstAlias<"move $dst, $src",
                      (ADDu GPR32Opnd:$dst, GPR32Opnd:$src, ZERO), 1>,
        GPR_32, ISA_FGPU1;

  def : FgpuInstAlias<"bal $offset", (BGEZAL ZERO, brtarget:$offset), 1>,
        ISA_FGPU1_NOT_32R6_64R6;

  def : FgpuInstAlias<"j $rs", (JR GPR32Opnd:$rs), 0>, ISA_FGPU1;

  def : FgpuInstAlias<"jalr $rs", (JALR RA, GPR32Opnd:$rs), 0>;

  def : FgpuInstAlias<"jalr.hb $rs", (JALR_HB RA, GPR32Opnd:$rs), 1>,
        ISA_FGPU32;

  def : FgpuInstAlias<"neg $rt, $rs",
                      (SUB GPR32Opnd:$rt, ZERO, GPR32Opnd:$rs), 1>, ISA_FGPU1;
  def : FgpuInstAlias<"neg $rt",
                      (SUB GPR32Opnd:$rt, ZERO, GPR32Opnd:$rt), 1>, ISA_FGPU1;
  def : FgpuInstAlias<"negu $rt, $rs",
                      (SUBu GPR32Opnd:$rt, ZERO, GPR32Opnd:$rs), 1>, ISA_FGPU1;
  def : FgpuInstAlias<"negu $rt",
                      (SUBu GPR32Opnd:$rt, ZERO, GPR32Opnd:$rt), 1>, ISA_FGPU1;

  def SGE : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                              (ins GPR32Opnd:$rs, GPR32Opnd:$rt),
                              "sge\t$rd, $rs, $rt">, ISA_FGPU1;
  def : FgpuInstAlias<"sge $rs, $rt",
                      (SGE GPR32Opnd:$rs, GPR32Opnd:$rs, GPR32Opnd:$rt), 0>,
        ISA_FGPU1;
  def SGEImm : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                 (ins GPR32Opnd:$rs, simm32:$imm),
                                 "sge\t$rd, $rs, $imm">, GPR_32;
  def : FgpuInstAlias<"sge $rs, $imm", (SGEImm GPR32Opnd:$rs,
                                               GPR32Opnd:$rs,
                                               simm32:$imm), 0>,
        GPR_32;

  def SGEU : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                               (ins GPR32Opnd:$rs, GPR32Opnd:$rt),
                               "sgeu\t$rd, $rs, $rt">, ISA_FGPU1;
  def : FgpuInstAlias<"sgeu $rs, $rt",
                      (SGEU GPR32Opnd:$rs, GPR32Opnd:$rs, GPR32Opnd:$rt), 0>,
        ISA_FGPU1;
  def SGEUImm : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                  (ins GPR32Opnd:$rs, uimm32_coerced:$imm),
                                  "sgeu\t$rd, $rs, $imm">, GPR_32;
  def : FgpuInstAlias<"sgeu $rs, $imm", (SGEUImm GPR32Opnd:$rs,
                                                 GPR32Opnd:$rs,
                                                 uimm32_coerced:$imm), 0>,
        GPR_32;

  def : FgpuInstAlias<
          "sgt $rd, $rs, $rt",
          (SLT GPR32Opnd:$rd, GPR32Opnd:$rt, GPR32Opnd:$rs), 0>, ISA_FGPU1;
  def : FgpuInstAlias<
          "sgt $rs, $rt",
          (SLT GPR32Opnd:$rs, GPR32Opnd:$rt, GPR32Opnd:$rs), 0>, ISA_FGPU1;

  def SGTImm : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                 (ins GPR32Opnd:$rs, simm32:$imm),
                                 "sgt\t$rd, $rs, $imm">, GPR_32;
  def : FgpuInstAlias<"sgt $rs, $imm", (SGTImm GPR32Opnd:$rs,
                                               GPR32Opnd:$rs,
                                               simm32:$imm), 0>,
        GPR_32;
  def : FgpuInstAlias<
          "sgtu $rd, $rs, $rt",
          (SLTu GPR32Opnd:$rd, GPR32Opnd:$rt, GPR32Opnd:$rs), 0>, ISA_FGPU1;
  def : FgpuInstAlias<
          "sgtu $$rs, $rt",
          (SLTu GPR32Opnd:$rs, GPR32Opnd:$rt, GPR32Opnd:$rs), 0>, ISA_FGPU1;

  def SGTUImm : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                  (ins GPR32Opnd:$rs, uimm32_coerced:$imm),
                                  "sgtu\t$rd, $rs, $imm">, GPR_32;
  def : FgpuInstAlias<"sgtu $rs, $imm", (SGTUImm GPR32Opnd:$rs,
                                                 GPR32Opnd:$rs,
                                                 uimm32_coerced:$imm), 0>,
        GPR_32;

  def SLE : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                              (ins GPR32Opnd:$rs, GPR32Opnd:$rt),
                              "sle\t$rd, $rs, $rt">, ISA_FGPU1;
  def : FgpuInstAlias<"sle $rs, $rt",
                      (SLE GPR32Opnd:$rs, GPR32Opnd:$rs, GPR32Opnd:$rt), 0>,
        ISA_FGPU1;
  def SLEImm : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                 (ins GPR32Opnd:$rs, simm32:$imm),
                                 "sle\t$rd, $rs, $imm">, GPR_32;
  def : FgpuInstAlias<"sle $rs, $imm", (SLEImm GPR32Opnd:$rs,
                                               GPR32Opnd:$rs,
                                               simm32:$imm), 0>,
        GPR_32;

  def SLEU : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                               (ins GPR32Opnd:$rs, GPR32Opnd:$rt),
                               "sleu\t$rd, $rs, $rt">, ISA_FGPU1;
  def : FgpuInstAlias<"sleu $rs, $rt",
                      (SLEU GPR32Opnd:$rs, GPR32Opnd:$rs, GPR32Opnd:$rt), 0>,
        ISA_FGPU1;
  def SLEUImm : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                  (ins GPR32Opnd:$rs, uimm32_coerced:$imm),
                                  "sleu\t$rd, $rs, $imm">, GPR_32;
  def : FgpuInstAlias<"sleu $rs, $imm", (SLEUImm GPR32Opnd:$rs,
                                                 GPR32Opnd:$rs,
                                                 uimm32_coerced:$imm), 0>,
        GPR_32;

  def : FgpuInstAlias<
          "not $rt, $rs",
          (NOR GPR32Opnd:$rt, GPR32Opnd:$rs, ZERO), 0>, ISA_FGPU1;
  def : FgpuInstAlias<
          "not $rt",
          (NOR GPR32Opnd:$rt, GPR32Opnd:$rt, ZERO), 0>, ISA_FGPU1;

  def : FgpuInstAlias<"nop", (SLL ZERO, ZERO, 0), 1>, ISA_FGPU1;

  defm : OneOrTwoOperandMacroImmediateAlias<"add", ADDi>,
         ISA_FGPU1_NOT_32R6_64R6;

  defm : OneOrTwoOperandMacroImmediateAlias<"addu", ADDiu>, ISA_FGPU1;

  defm : OneOrTwoOperandMacroImmediateAlias<"and", ANDi>, ISA_FGPU1, GPR_32;

  defm : OneOrTwoOperandMacroImmediateAlias<"or", ORi>, ISA_FGPU1, GPR_32;

  defm : OneOrTwoOperandMacroImmediateAlias<"xor", XORi>, ISA_FGPU1, GPR_32;

  defm : OneOrTwoOperandMacroImmediateAlias<"slt", SLTi>, ISA_FGPU1, GPR_32;

  defm : OneOrTwoOperandMacroImmediateAlias<"sltu", SLTiu>, ISA_FGPU1, GPR_32;

  def : FgpuInstAlias<"mfgc0 $rt, $rd",
                      (MFGC0 GPR32Opnd:$rt, COP0Opnd:$rd, 0), 0>,
                      ISA_FGPU32R5, ASE_VIRT;
  def : FgpuInstAlias<"mtgc0 $rt, $rd",
                      (MTGC0 COP0Opnd:$rd, GPR32Opnd:$rt, 0), 0>,
                      ISA_FGPU32R5, ASE_VIRT;
  def : FgpuInstAlias<"mfhgc0 $rt, $rd",
                      (MFHGC0 GPR32Opnd:$rt, COP0Opnd:$rd, 0), 0>,
                      ISA_FGPU32R5, ASE_VIRT;
  def : FgpuInstAlias<"mthgc0 $rt, $rd",
                      (MTHGC0 COP0Opnd:$rd, GPR32Opnd:$rt, 0), 0>,
                      ISA_FGPU32R5, ASE_VIRT;
  def : FgpuInstAlias<"mfc0 $rt, $rd", (MFC0 GPR32Opnd:$rt, COP0Opnd:$rd, 0), 0>,
        ISA_FGPU1;
  def : FgpuInstAlias<"mtc0 $rt, $rd", (MTC0 COP0Opnd:$rd, GPR32Opnd:$rt, 0), 0>,
        ISA_FGPU1;
  def : FgpuInstAlias<"mfc2 $rt, $rd", (MFC2 GPR32Opnd:$rt, COP2Opnd:$rd, 0), 0>,
        ISA_FGPU1;
  def : FgpuInstAlias<"mtc2 $rt, $rd", (MTC2 COP2Opnd:$rd, GPR32Opnd:$rt, 0), 0>,
        ISA_FGPU1;

  def : FgpuInstAlias<"b $offset", (BEQ ZERO, ZERO, brtarget:$offset), 0>,
        ISA_FGPU1;

  def : FgpuInstAlias<"bnez $rs,$offset",
                      (BNE GPR32Opnd:$rs, ZERO, brtarget:$offset), 0>,
        ISA_FGPU1;
  def : FgpuInstAlias<"bnezl $rs, $offset",
                      (BNEL GPR32Opnd:$rs, ZERO, brtarget:$offset), 1>,
        ISA_FGPU2;
  def : FgpuInstAlias<"beqz $rs,$offset",
                      (BEQ GPR32Opnd:$rs, ZERO, brtarget:$offset), 0>,
        ISA_FGPU1;
  def : FgpuInstAlias<"beqzl $rs, $offset",
                      (BEQL GPR32Opnd:$rs, ZERO, brtarget:$offset), 1>,
        ISA_FGPU2;

  def : FgpuInstAlias<"syscall", (SYSCALL 0), 1>, ISA_FGPU1;

  def : FgpuInstAlias<"break", (BREAK 0, 0), 1>, ISA_FGPU1;
  def : FgpuInstAlias<"break $imm", (BREAK uimm10:$imm, 0), 1>, ISA_FGPU1;
  def : FgpuInstAlias<"ei", (EI ZERO), 1>, ISA_FGPU32R2;
  def : FgpuInstAlias<"di", (DI ZERO), 1>, ISA_FGPU32R2;

  def : FgpuInstAlias<"teq $rs, $rt",
                      (TEQ GPR32Opnd:$rs, GPR32Opnd:$rt, 0), 1>, ISA_FGPU2;
  def : FgpuInstAlias<"tge $rs, $rt",
                      (TGE GPR32Opnd:$rs, GPR32Opnd:$rt, 0), 1>, ISA_FGPU2;
  def : FgpuInstAlias<"tgeu $rs, $rt",
                      (TGEU GPR32Opnd:$rs, GPR32Opnd:$rt, 0), 1>, ISA_FGPU2;
  def : FgpuInstAlias<"tlt $rs, $rt",
                      (TLT GPR32Opnd:$rs, GPR32Opnd:$rt, 0), 1>, ISA_FGPU2;
  def : FgpuInstAlias<"tltu $rs, $rt",
                      (TLTU GPR32Opnd:$rs, GPR32Opnd:$rt, 0), 1>, ISA_FGPU2;
  def : FgpuInstAlias<"tne $rs, $rt",
                      (TNE GPR32Opnd:$rs, GPR32Opnd:$rt, 0), 1>, ISA_FGPU2;
  def : FgpuInstAlias<"rdhwr $rt, $rs",
                      (RDHWR GPR32Opnd:$rt, HWRegsOpnd:$rs, 0), 1>, ISA_FGPU1;

}
def : FgpuInstAlias<"sub, $rd, $rs, $imm",
                    (ADDi GPR32Opnd:$rd, GPR32Opnd:$rs,
                          InvertedImOperand:$imm), 0>, ISA_FGPU1_NOT_32R6_64R6;
def : FgpuInstAlias<"sub $rs, $imm",
                    (ADDi GPR32Opnd:$rs, GPR32Opnd:$rs, InvertedImOperand:$imm),
                    0>, ISA_FGPU1_NOT_32R6_64R6;
def : FgpuInstAlias<"subu, $rd, $rs, $imm",
                    (ADDiu GPR32Opnd:$rd, GPR32Opnd:$rs,
                           InvertedImOperand:$imm), 0>;
def : FgpuInstAlias<"subu $rs, $imm", (ADDiu GPR32Opnd:$rs, GPR32Opnd:$rs,
                                             InvertedImOperand:$imm), 0>;
let AdditionalPredicates = [NotInMicroFgpu] in {
  def : FgpuInstAlias<"sll $rd, $rt, $rs",
                      (SLLV GPR32Opnd:$rd, GPR32Opnd:$rt, GPR32Opnd:$rs), 0>;
  def : FgpuInstAlias<"sra $rd, $rt, $rs",
                      (SRAV GPR32Opnd:$rd, GPR32Opnd:$rt, GPR32Opnd:$rs), 0>;
  def : FgpuInstAlias<"srl $rd, $rt, $rs",
                      (SRLV GPR32Opnd:$rd, GPR32Opnd:$rt, GPR32Opnd:$rs), 0>;
  def : FgpuInstAlias<"sll $rd, $rt",
                      (SLLV GPR32Opnd:$rd, GPR32Opnd:$rd, GPR32Opnd:$rt), 0>;
  def : FgpuInstAlias<"sra $rd, $rt",
                      (SRAV GPR32Opnd:$rd, GPR32Opnd:$rd, GPR32Opnd:$rt), 0>;
  def : FgpuInstAlias<"srl $rd, $rt",
                      (SRLV GPR32Opnd:$rd, GPR32Opnd:$rd, GPR32Opnd:$rt), 0>;
  def : FgpuInstAlias<"seh $rd", (SEH GPR32Opnd:$rd, GPR32Opnd:$rd), 0>,
                     ISA_FGPU32R2;
  def : FgpuInstAlias<"seb $rd", (SEB GPR32Opnd:$rd, GPR32Opnd:$rd), 0>,
                     ISA_FGPU32R2;
}
def : FgpuInstAlias<"sdbbp", (SDBBP 0)>, ISA_FGPU32_NOT_32R6_64R6;
let AdditionalPredicates = [NotInMicroFgpu] in
  def : FgpuInstAlias<"sync", (SYNC 0), 1>, ISA_FGPU2;

def : FgpuInstAlias<"mulo $rs, $rt",
                    (MULOMacro GPR32Opnd:$rs, GPR32Opnd:$rs, GPR32Opnd:$rt), 0>,
                    ISA_FGPU1_NOT_32R6_64R6;
def : FgpuInstAlias<"mulou $rs, $rt",
                    (MULOUMacro GPR32Opnd:$rs, GPR32Opnd:$rs, GPR32Opnd:$rt), 0>,
                    ISA_FGPU1_NOT_32R6_64R6;

let AdditionalPredicates = [NotInMicroFgpu] in
  def : FgpuInstAlias<"hypcall", (HYPCALL 0), 1>, ISA_FGPU32R5, ASE_VIRT;

//===----------------------------------------------------------------------===//
// Assembler Pseudo Instructions
//===----------------------------------------------------------------------===//

// We use uimm32_coerced to accept a 33 bit signed number that is rendered into
// a 32 bit number.
class LoadImmediate32<string instr_asm, Operand Od, RegisterOperand RO> :
  FgpuAsmPseudoInst<(outs RO:$rt), (ins Od:$imm32),
                     !strconcat(instr_asm, "\t$rt, $imm32")> ;
def LoadImm32 : LoadImmediate32<"li", uimm32_coerced, GPR32Opnd>;

class LoadAddressFromReg32<string instr_asm, Operand MemOpnd,
                           RegisterOperand RO> :
  FgpuAsmPseudoInst<(outs RO:$rt), (ins MemOpnd:$addr),
                     !strconcat(instr_asm, "\t$rt, $addr")> ;
def LoadAddrReg32 : LoadAddressFromReg32<"la", mem, GPR32Opnd>;

class LoadAddressFromImm32<string instr_asm, Operand Od, RegisterOperand RO> :
  FgpuAsmPseudoInst<(outs RO:$rt), (ins Od:$imm32),
                     !strconcat(instr_asm, "\t$rt, $imm32")> ;
def LoadAddrImm32 : LoadAddressFromImm32<"la", i32imm, GPR32Opnd>;

def JalTwoReg : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd), (ins GPR32Opnd:$rs),
                      "jal\t$rd, $rs"> ;
def JalOneReg : FgpuAsmPseudoInst<(outs), (ins GPR32Opnd:$rs),
                      "jal\t$rs"> ;

class NORIMM_DESC_BASE<RegisterOperand RO, DAGOperand Imm> :
   FgpuAsmPseudoInst<(outs RO:$rs), (ins RO:$rt, Imm:$imm),
                      "nor\t$rs, $rt, $imm">;
def NORImm : NORIMM_DESC_BASE<GPR32Opnd, simm32_relaxed>, GPR_32;
def : FgpuInstAlias<"nor\t$rs, $imm", (NORImm GPR32Opnd:$rs, GPR32Opnd:$rs,
                                              simm32_relaxed:$imm)>, GPR_32;

let hasDelaySlot = 1, isCTI = 1 in {
def BneImm : FgpuAsmPseudoInst<(outs GPR32Opnd:$rt),
                               (ins imm64:$imm64, brtarget:$offset),
                               "bne\t$rt, $imm64, $offset">;
def BeqImm : FgpuAsmPseudoInst<(outs GPR32Opnd:$rt),
                               (ins imm64:$imm64, brtarget:$offset),
                               "beq\t$rt, $imm64, $offset">;

class CondBranchPseudo<string instr_asm> :
  FgpuAsmPseudoInst<(outs), (ins GPR32Opnd:$rs, GPR32Opnd:$rt,
                                 brtarget:$offset),
                    !strconcat(instr_asm, "\t$rs, $rt, $offset")>;
}

def BLT : CondBranchPseudo<"blt">;
def BLE : CondBranchPseudo<"ble">;
def BGE : CondBranchPseudo<"bge">;
def BGT : CondBranchPseudo<"bgt">;
def BLTU : CondBranchPseudo<"bltu">;
def BLEU : CondBranchPseudo<"bleu">;
def BGEU : CondBranchPseudo<"bgeu">;
def BGTU : CondBranchPseudo<"bgtu">;
def BLTL : CondBranchPseudo<"bltl">, ISA_FGPU2_NOT_32R6_64R6;
def BLEL : CondBranchPseudo<"blel">, ISA_FGPU2_NOT_32R6_64R6;
def BGEL : CondBranchPseudo<"bgel">, ISA_FGPU2_NOT_32R6_64R6;
def BGTL : CondBranchPseudo<"bgtl">, ISA_FGPU2_NOT_32R6_64R6;
def BLTUL: CondBranchPseudo<"bltul">, ISA_FGPU2_NOT_32R6_64R6;
def BLEUL: CondBranchPseudo<"bleul">, ISA_FGPU2_NOT_32R6_64R6;
def BGEUL: CondBranchPseudo<"bgeul">, ISA_FGPU2_NOT_32R6_64R6;
def BGTUL: CondBranchPseudo<"bgtul">, ISA_FGPU2_NOT_32R6_64R6;

let isCTI = 1 in
class CondBranchImmPseudo<string instr_asm> :
  FgpuAsmPseudoInst<(outs), (ins GPR32Opnd:$rs, imm64:$imm, brtarget:$offset),
                    !strconcat(instr_asm, "\t$rs, $imm, $offset")>;

def BEQLImmMacro : CondBranchImmPseudo<"beql">, ISA_FGPU2_NOT_32R6_64R6;
def BNELImmMacro : CondBranchImmPseudo<"bnel">, ISA_FGPU2_NOT_32R6_64R6;

def BLTImmMacro  : CondBranchImmPseudo<"blt">;
def BLEImmMacro  : CondBranchImmPseudo<"ble">;
def BGEImmMacro  : CondBranchImmPseudo<"bge">;
def BGTImmMacro  : CondBranchImmPseudo<"bgt">;
def BLTUImmMacro : CondBranchImmPseudo<"bltu">;
def BLEUImmMacro : CondBranchImmPseudo<"bleu">;
def BGEUImmMacro : CondBranchImmPseudo<"bgeu">;
def BGTUImmMacro : CondBranchImmPseudo<"bgtu">;
def BLTLImmMacro : CondBranchImmPseudo<"bltl">, ISA_FGPU2_NOT_32R6_64R6;
def BLELImmMacro : CondBranchImmPseudo<"blel">, ISA_FGPU2_NOT_32R6_64R6;
def BGELImmMacro : CondBranchImmPseudo<"bgel">, ISA_FGPU2_NOT_32R6_64R6;
def BGTLImmMacro : CondBranchImmPseudo<"bgtl">, ISA_FGPU2_NOT_32R6_64R6;
def BLTULImmMacro : CondBranchImmPseudo<"bltul">, ISA_FGPU2_NOT_32R6_64R6;
def BLEULImmMacro : CondBranchImmPseudo<"bleul">, ISA_FGPU2_NOT_32R6_64R6;
def BGEULImmMacro : CondBranchImmPseudo<"bgeul">, ISA_FGPU2_NOT_32R6_64R6;
def BGTULImmMacro : CondBranchImmPseudo<"bgtul">, ISA_FGPU2_NOT_32R6_64R6;

// FIXME: Predicates are removed because instructions are matched regardless of
// predicates, because PredicateControl was not in the hierarchy. This was
// done to emit more precise error message from expansion function.
// Once the tablegen-erated errors are made better, this needs to be fixed and
// predicates needs to be restored.

def SDivMacro : FgpuAsmPseudoInst<(outs GPR32NonZeroOpnd:$rd),
                                  (ins GPR32Opnd:$rs, GPR32Opnd:$rt),
                                  "div\t$rd, $rs, $rt">,
                ISA_FGPU1_NOT_32R6_64R6;
def SDivIMacro : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                   (ins GPR32Opnd:$rs, simm32:$imm),
                                   "div\t$rd, $rs, $imm">,
                 ISA_FGPU1_NOT_32R6_64R6;
def UDivMacro : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                  (ins GPR32Opnd:$rs, GPR32Opnd:$rt),
                                  "divu\t$rd, $rs, $rt">,
                ISA_FGPU1_NOT_32R6_64R6;
def UDivIMacro : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                   (ins GPR32Opnd:$rs, simm32:$imm),
                                   "divu\t$rd, $rs, $imm">,
                 ISA_FGPU1_NOT_32R6_64R6;


def : FgpuInstAlias<"div $rs, $rt", (SDIV GPR32ZeroOpnd:$rs,
                                          GPR32Opnd:$rt), 0>,
     ISA_FGPU1_NOT_32R6_64R6;
def : FgpuInstAlias<"div $rs, $rt", (SDivMacro GPR32NonZeroOpnd:$rs,
                                               GPR32NonZeroOpnd:$rs,
                                               GPR32Opnd:$rt), 0>,
     ISA_FGPU1_NOT_32R6_64R6;
def : FgpuInstAlias<"div $rd, $imm", (SDivIMacro GPR32Opnd:$rd, GPR32Opnd:$rd,
                                                 simm32:$imm), 0>,
      ISA_FGPU1_NOT_32R6_64R6;

def : FgpuInstAlias<"divu $rt, $rs", (UDIV GPR32ZeroOpnd:$rt,
                                           GPR32Opnd:$rs), 0>,
      ISA_FGPU1_NOT_32R6_64R6;
def : FgpuInstAlias<"divu $rt, $rs", (UDivMacro GPR32NonZeroOpnd:$rt,
                                                GPR32NonZeroOpnd:$rt,
                                                GPR32Opnd:$rs), 0>,
      ISA_FGPU1_NOT_32R6_64R6;

def : FgpuInstAlias<"divu $rd, $imm", (UDivIMacro GPR32Opnd:$rd, GPR32Opnd:$rd,
                                                  simm32:$imm), 0>,
      ISA_FGPU1_NOT_32R6_64R6;

def SRemMacro : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                  (ins GPR32Opnd:$rs, GPR32Opnd:$rt),
                                  "rem\t$rd, $rs, $rt">,
                ISA_FGPU1_NOT_32R6_64R6;
def SRemIMacro : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                   (ins GPR32Opnd:$rs, simm32_relaxed:$imm),
                                   "rem\t$rd, $rs, $imm">,
                 ISA_FGPU1_NOT_32R6_64R6;
def URemMacro : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                  (ins GPR32Opnd:$rs, GPR32Opnd:$rt),
                                  "remu\t$rd, $rs, $rt">,
                ISA_FGPU1_NOT_32R6_64R6;
def URemIMacro : FgpuAsmPseudoInst<(outs GPR32Opnd:$rd),
                                   (ins GPR32Opnd:$rs, simm32_relaxed:$imm),
                                   "remu\t$rd, $rs, $imm">,
                 ISA_FGPU1_NOT_32R6_64R6;

def : FgpuInstAlias<"rem $rt, $rs", (SRemMacro GPR32Opnd:$rt, GPR32Opnd:$rt,
                                               GPR32Opnd:$rs), 0>,
      ISA_FGPU1_NOT_32R6_64R6;
def : FgpuInstAlias<"rem $rd, $imm", (SRemIMacro GPR32Opnd:$rd, GPR32Opnd:$rd,
                                      simm32_relaxed:$imm), 0>,
      ISA_FGPU1_NOT_32R6_64R6;
def : FgpuInstAlias<"remu $rt, $rs", (URemMacro GPR32Opnd:$rt, GPR32Opnd:$rt,
                                                GPR32Opnd:$rs), 0>,
      ISA_FGPU1_NOT_32R6_64R6;
def : FgpuInstAlias<"remu $rd, $imm", (URemIMacro GPR32Opnd:$rd, GPR32Opnd:$rd,
                                       simm32_relaxed:$imm), 0>,
      ISA_FGPU1_NOT_32R6_64R6;

def Ulh : FgpuAsmPseudoInst<(outs GPR32Opnd:$rt), (ins mem:$addr),
                            "ulh\t$rt, $addr">; //, ISA_FGPU1_NOT_32R6_64R6;

def Ulhu : FgpuAsmPseudoInst<(outs GPR32Opnd:$rt), (ins mem:$addr),
                             "ulhu\t$rt, $addr">; //, ISA_FGPU1_NOT_32R6_64R6;

def Ulw : FgpuAsmPseudoInst<(outs GPR32Opnd:$rt), (ins mem:$addr),
                            "ulw\t$rt, $addr">; //, ISA_FGPU1_NOT_32R6_64R6;

def Ush : FgpuAsmPseudoInst<(outs GPR32Opnd:$rt), (ins mem:$addr),
                            "ush\t$rt, $addr">; //, ISA_FGPU1_NOT_32R6_64R6;

def Usw : FgpuAsmPseudoInst<(outs GPR32Opnd:$rt), (ins mem:$addr),
                            "usw\t$rt, $addr">; //, ISA_FGPU1_NOT_32R6_64R6;

def LDMacro : FgpuAsmPseudoInst<(outs GPR32Opnd:$rt),
                                (ins mem_simm16:$addr), "ld $rt, $addr">,
                                ISA_FGPU1_NOT_FGPU3;
def SDMacro : FgpuAsmPseudoInst<(outs GPR32Opnd:$rt),
                                (ins mem_simm16:$addr), "sd $rt, $addr">,
                                ISA_FGPU1_NOT_FGPU3;
//===----------------------------------------------------------------------===//
//  Arbitrary patterns that map to one or more instructions
//===----------------------------------------------------------------------===//

// Load/store pattern templates.
class LoadRegImmPat<Instruction LoadInst, ValueType ValTy, PatFrag Node> :
  FgpuPat<(ValTy (Node addrRegImm:$a)), (LoadInst addrRegImm:$a)>;

class StoreRegImmPat<Instruction StoreInst, ValueType ValTy> :
  FgpuPat<(store ValTy:$v, addrRegImm:$a), (StoreInst ValTy:$v, addrRegImm:$a)>;

// Materialize constants.
multiclass MaterializeImms<ValueType VT, Register ZEROReg,
                           Instruction ADDiuOp, Instruction LUiOp,
                           Instruction ORiOp> {

// Constant synthesis previously relied on the ordering of the patterns below.
// By making the predicates they use non-overlapping, the patterns were
// reordered so that the effect of the newly introduced predicates can be
// observed.

// Arbitrary immediates
def : FgpuPat<(VT LUiORiPred:$imm),
              (ORiOp (LUiOp (HI16 imm:$imm)), (LO16 imm:$imm))>;

// Bits 32-16 set, sign/zero extended.
def : FgpuPat<(VT LUiPred:$imm), (LUiOp (HI16 imm:$imm))>;

// Small immediates
def : FgpuPat<(VT ORiPred:$imm), (ORiOp ZEROReg, imm:$imm)>;
def : FgpuPat<(VT immSExt16:$imm), (ADDiuOp ZEROReg, imm:$imm)>;
}

let AdditionalPredicates = [NotInMicroFgpu] in
  defm : MaterializeImms<i32, ZERO, ADDiu, LUi, ORi>, ISA_FGPU1;

// Carry FgpuPatterns
let AdditionalPredicates = [NotInMicroFgpu] in {
  def : FgpuPat<(subc GPR32:$lhs, GPR32:$rhs),
                (SUBu GPR32:$lhs, GPR32:$rhs)>, ISA_FGPU1;
}
def : FgpuPat<(addc GPR32:$lhs, GPR32:$rhs),
              (ADDu GPR32:$lhs, GPR32:$rhs)>, ISA_FGPU1, ASE_NOT_DSP;
def : FgpuPat<(addc  GPR32:$src, immSExt16:$imm),
              (ADDiu GPR32:$src, imm:$imm)>, ISA_FGPU1, ASE_NOT_DSP;

// Support multiplication for pre-Fgpu32 targets that don't have
// the MUL instruction.
def : FgpuPat<(mul GPR32:$lhs, GPR32:$rhs),
              (PseudoMFLO (PseudoMULT GPR32:$lhs, GPR32:$rhs))>,
      ISA_FGPU1_NOT_32R6_64R6;

// SYNC
def : FgpuPat<(FgpuSync (i32 immz)),
              (SYNC 0)>, ISA_FGPU2;

// Call
def : FgpuPat<(FgpuJmpLink (i32 texternalsym:$dst)),
              (JAL texternalsym:$dst)>, ISA_FGPU1;
//def : FgpuPat<(FgpuJmpLink GPR32:$dst),
//              (JALR GPR32:$dst)>;

// Tail call
let AdditionalPredicates = [NotInMicroFgpu] in {
  def : FgpuPat<(FgpuTailCall (iPTR tglobaladdr:$dst)),
                (TAILCALL tglobaladdr:$dst)>, ISA_FGPU1;
  def : FgpuPat<(FgpuTailCall (iPTR texternalsym:$dst)),
                (TAILCALL texternalsym:$dst)>, ISA_FGPU1;
}
// hi/lo relocs
multiclass FgpuHiLoRelocs<Instruction Lui, Instruction Addiu,
                          Register ZeroReg, RegisterOperand GPROpnd> {
  def : FgpuPat<(FgpuHi tglobaladdr:$in), (Lui tglobaladdr:$in)>;
  def : FgpuPat<(FgpuHi tblockaddress:$in), (Lui tblockaddress:$in)>;
  def : FgpuPat<(FgpuHi tjumptable:$in), (Lui tjumptable:$in)>;
  def : FgpuPat<(FgpuHi tconstpool:$in), (Lui tconstpool:$in)>;
  def : FgpuPat<(FgpuHi texternalsym:$in), (Lui texternalsym:$in)>;

  def : FgpuPat<(FgpuLo tglobaladdr:$in),
                (Addiu ZeroReg, tglobaladdr:$in)>;
  def : FgpuPat<(FgpuLo tblockaddress:$in),
                (Addiu ZeroReg, tblockaddress:$in)>;
  def : FgpuPat<(FgpuLo tjumptable:$in),
                (Addiu ZeroReg, tjumptable:$in)>;
  def : FgpuPat<(FgpuLo tconstpool:$in),
                (Addiu ZeroReg, tconstpool:$in)>;
  def : FgpuPat<(FgpuLo tglobaltlsaddr:$in),
                (Addiu ZeroReg, tglobaltlsaddr:$in)>;
  def : FgpuPat<(FgpuLo texternalsym:$in),
                (Addiu ZeroReg, texternalsym:$in)>;

  def : FgpuPat<(add GPROpnd:$hi, (FgpuLo tglobaladdr:$lo)),
                (Addiu GPROpnd:$hi, tglobaladdr:$lo)>;
  def : FgpuPat<(add GPROpnd:$hi, (FgpuLo tblockaddress:$lo)),
                (Addiu GPROpnd:$hi, tblockaddress:$lo)>;
  def : FgpuPat<(add GPROpnd:$hi, (FgpuLo tjumptable:$lo)),
                (Addiu GPROpnd:$hi, tjumptable:$lo)>;
  def : FgpuPat<(add GPROpnd:$hi, (FgpuLo tconstpool:$lo)),
                (Addiu GPROpnd:$hi, tconstpool:$lo)>;
  def : FgpuPat<(add GPROpnd:$hi, (FgpuLo tglobaltlsaddr:$lo)),
                (Addiu GPROpnd:$hi, tglobaltlsaddr:$lo)>;
  def : FgpuPat<(add GPROpnd:$hi, (FgpuLo texternalsym:$lo)),
                (Addiu GPROpnd:$hi, texternalsym:$lo)>;
}

// wrapper_pic
class WrapperPat<SDNode node, Instruction ADDiuOp, RegisterClass RC>:
      FgpuPat<(FgpuWrapper RC:$gp, node:$in), (ADDiuOp RC:$gp, node:$in)>;

let AdditionalPredicates = [NotInMicroFgpu] in {
  defm : FgpuHiLoRelocs<LUi, ADDiu, ZERO, GPR32Opnd>, ISA_FGPU1;

  def : FgpuPat<(FgpuGotHi tglobaladdr:$in), (LUi tglobaladdr:$in)>, ISA_FGPU1;
  def : FgpuPat<(FgpuGotHi texternalsym:$in), (LUi texternalsym:$in)>,
        ISA_FGPU1;

  def : FgpuPat<(FgpuTlsHi tglobaltlsaddr:$in), (LUi tglobaltlsaddr:$in)>,
        ISA_FGPU1;

  // gp_rel relocs
  def : FgpuPat<(add GPR32:$gp, (FgpuGPRel tglobaladdr:$in)),
                (ADDiu GPR32:$gp, tglobaladdr:$in)>, ISA_FGPU1, ABI_NOT_N64;
  def : FgpuPat<(add GPR32:$gp, (FgpuGPRel tconstpool:$in)),
                (ADDiu GPR32:$gp, tconstpool:$in)>, ISA_FGPU1, ABI_NOT_N64;

  def : WrapperPat<tglobaladdr, ADDiu, GPR32>, ISA_FGPU1;
  def : WrapperPat<tconstpool, ADDiu, GPR32>, ISA_FGPU1;
  def : WrapperPat<texternalsym, ADDiu, GPR32>, ISA_FGPU1;
  def : WrapperPat<tblockaddress, ADDiu, GPR32>, ISA_FGPU1;
  def : WrapperPat<tjumptable, ADDiu, GPR32>, ISA_FGPU1;
  def : WrapperPat<tglobaltlsaddr, ADDiu, GPR32>, ISA_FGPU1;

  // Fgpu does not have "not", so we expand our way
  def : FgpuPat<(not GPR32:$in),
                (NOR GPR32Opnd:$in, ZERO)>, ISA_FGPU1;
}

// extended loads
let AdditionalPredicates = [NotInMicroFgpu] in {
  def : FgpuPat<(i32 (extloadi1  addr:$src)), (LBu addr:$src)>, ISA_FGPU1;
  def : FgpuPat<(i32 (extloadi8  addr:$src)), (LBu addr:$src)>, ISA_FGPU1;
  def : FgpuPat<(i32 (extloadi16 addr:$src)), (LHu addr:$src)>, ISA_FGPU1;

  // peepholes
  def : FgpuPat<(store (i32 0), addr:$dst), (SW ZERO, addr:$dst)>, ISA_FGPU1;
}

// brcond patterns
multiclass BrcondPats<RegisterClass RC, Instruction BEQOp, Instruction BEQOp1,
                      Instruction BNEOp, Instruction SLTOp, Instruction SLTuOp,
                      Instruction SLTiOp, Instruction SLTiuOp,
                      Register ZEROReg> {
def : FgpuPat<(brcond (i32 (setne RC:$lhs, 0)), bb:$dst),
              (BNEOp RC:$lhs, ZEROReg, bb:$dst)>;
def : FgpuPat<(brcond (i32 (seteq RC:$lhs, 0)), bb:$dst),
              (BEQOp RC:$lhs, ZEROReg, bb:$dst)>;

def : FgpuPat<(brcond (i32 (setge RC:$lhs, RC:$rhs)), bb:$dst),
              (BEQOp1 (SLTOp RC:$lhs, RC:$rhs), ZERO, bb:$dst)>;
def : FgpuPat<(brcond (i32 (setuge RC:$lhs, RC:$rhs)), bb:$dst),
              (BEQOp1 (SLTuOp RC:$lhs, RC:$rhs), ZERO, bb:$dst)>;
def : FgpuPat<(brcond (i32 (setge RC:$lhs, immSExt16:$rhs)), bb:$dst),
              (BEQOp1 (SLTiOp RC:$lhs, immSExt16:$rhs), ZERO, bb:$dst)>;
def : FgpuPat<(brcond (i32 (setuge RC:$lhs, immSExt16:$rhs)), bb:$dst),
              (BEQOp1 (SLTiuOp RC:$lhs, immSExt16:$rhs), ZERO, bb:$dst)>;
def : FgpuPat<(brcond (i32 (setgt RC:$lhs, immSExt16Plus1:$rhs)), bb:$dst),
              (BEQOp1 (SLTiOp RC:$lhs, (Plus1 imm:$rhs)), ZERO, bb:$dst)>;
def : FgpuPat<(brcond (i32 (setugt RC:$lhs, immSExt16Plus1:$rhs)), bb:$dst),
              (BEQOp1 (SLTiuOp RC:$lhs, (Plus1 imm:$rhs)), ZERO, bb:$dst)>;

def : FgpuPat<(brcond (i32 (setle RC:$lhs, RC:$rhs)), bb:$dst),
              (BEQOp1 (SLTOp RC:$rhs, RC:$lhs), ZERO, bb:$dst)>;
def : FgpuPat<(brcond (i32 (setule RC:$lhs, RC:$rhs)), bb:$dst),
              (BEQOp1 (SLTuOp RC:$rhs, RC:$lhs), ZERO, bb:$dst)>;

def : FgpuPat<(brcond RC:$cond, bb:$dst),
              (BNEOp RC:$cond, ZEROReg, bb:$dst)>;
}
let AdditionalPredicates = [NotInMicroFgpu] in {
  defm : BrcondPats<GPR32, BEQ, BEQ, BNE, SLT, SLTu, SLTi, SLTiu, ZERO>,
         ISA_FGPU1;
  def : FgpuPat<(brcond (i32 (setlt i32:$lhs, 1)), bb:$dst),
                (BLEZ i32:$lhs, bb:$dst)>, ISA_FGPU1;
  def : FgpuPat<(brcond (i32 (setgt i32:$lhs, -1)), bb:$dst),
                (BGEZ i32:$lhs, bb:$dst)>, ISA_FGPU1;
}

// setcc patterns
multiclass SeteqPats<RegisterClass RC, Instruction SLTiuOp, Instruction XOROp,
                     Instruction SLTuOp, Register ZEROReg> {
  def : FgpuPat<(seteq RC:$lhs, 0),
                (SLTiuOp RC:$lhs, 1)>;
  def : FgpuPat<(setne RC:$lhs, 0),
                (SLTuOp ZEROReg, RC:$lhs)>;
  def : FgpuPat<(seteq RC:$lhs, RC:$rhs),
                (SLTiuOp (XOROp RC:$lhs, RC:$rhs), 1)>;
  def : FgpuPat<(setne RC:$lhs, RC:$rhs),
                (SLTuOp ZEROReg, (XOROp RC:$lhs, RC:$rhs))>;
}

multiclass SetlePats<RegisterClass RC, Instruction XORiOp, Instruction SLTOp,
                     Instruction SLTuOp> {
  def : FgpuPat<(setle RC:$lhs, RC:$rhs),
                (XORiOp (SLTOp RC:$rhs, RC:$lhs), 1)>;
  def : FgpuPat<(setule RC:$lhs, RC:$rhs),
                (XORiOp (SLTuOp RC:$rhs, RC:$lhs), 1)>;
}

multiclass SetgtPats<RegisterClass RC, Instruction SLTOp, Instruction SLTuOp> {
  def : FgpuPat<(setgt RC:$lhs, RC:$rhs),
                (SLTOp RC:$rhs, RC:$lhs)>;
  def : FgpuPat<(setugt RC:$lhs, RC:$rhs),
                (SLTuOp RC:$rhs, RC:$lhs)>;
}

multiclass SetgePats<RegisterClass RC, Instruction XORiOp, Instruction SLTOp,
                     Instruction SLTuOp> {
  def : FgpuPat<(setge RC:$lhs, RC:$rhs),
                (XORiOp (SLTOp RC:$lhs, RC:$rhs), 1)>;
  def : FgpuPat<(setuge RC:$lhs, RC:$rhs),
                (XORiOp (SLTuOp RC:$lhs, RC:$rhs), 1)>;
}

multiclass SetgeImmPats<RegisterClass RC, Instruction XORiOp,
                        Instruction SLTiOp, Instruction SLTiuOp> {
  def : FgpuPat<(setge RC:$lhs, immSExt16:$rhs),
                (XORiOp (SLTiOp RC:$lhs, immSExt16:$rhs), 1)>;
  def : FgpuPat<(setuge RC:$lhs, immSExt16:$rhs),
                (XORiOp (SLTiuOp RC:$lhs, immSExt16:$rhs), 1)>;
}

let AdditionalPredicates = [NotInMicroFgpu] in {
  defm : SeteqPats<GPR32, SLTiu, XOR, SLTu, ZERO>, ISA_FGPU1;
  defm : SetlePats<GPR32, XORi, SLT, SLTu>, ISA_FGPU1;
  defm : SetgtPats<GPR32, SLT, SLTu>, ISA_FGPU1;
  defm : SetgePats<GPR32, XORi, SLT, SLTu>, ISA_FGPU1;
  defm : SetgeImmPats<GPR32, XORi, SLTi, SLTiu>, ISA_FGPU1;

  // bswap pattern
  def : FgpuPat<(bswap GPR32:$rt), (ROTR (WSBH GPR32:$rt), 16)>, ISA_FGPU32R2;
}

// Load halfword/word patterns.
let AdditionalPredicates = [NotInMicroFgpu] in {
  let AddedComplexity = 40 in {
    def : LoadRegImmPat<LBu, i32, zextloadi8>, ISA_FGPU1;
    def : LoadRegImmPat<LHu, i32, zextloadi16>, ISA_FGPU1;
    def : LoadRegImmPat<LB, i32, sextloadi8>, ISA_FGPU1;
    def : LoadRegImmPat<LH, i32, sextloadi16>, ISA_FGPU1;
    def : LoadRegImmPat<LW, i32, load>, ISA_FGPU1;
  }

  // Atomic load patterns.
  def : FgpuPat<(atomic_load_8 addr:$a), (LB addr:$a)>, ISA_FGPU1;
  def : FgpuPat<(atomic_load_16 addr:$a), (LH addr:$a)>, ISA_FGPU1;
  def : FgpuPat<(atomic_load_32 addr:$a), (LW addr:$a)>, ISA_FGPU1;

  // Atomic store patterns.
  def : FgpuPat<(atomic_store_8 addr:$a, GPR32:$v), (SB GPR32:$v, addr:$a)>,
        ISA_FGPU1;
  def : FgpuPat<(atomic_store_16 addr:$a, GPR32:$v), (SH GPR32:$v, addr:$a)>,
        ISA_FGPU1;
  def : FgpuPat<(atomic_store_32 addr:$a, GPR32:$v), (SW GPR32:$v, addr:$a)>,
        ISA_FGPU1;
}

//===----------------------------------------------------------------------===//
// Floating Point Support
//===----------------------------------------------------------------------===//

include "FgpuInstrFPU.td"
include "Fgpu64InstrInfo.td"
include "FgpuCondMov.td"

include "Fgpu32r6InstrInfo.td"
include "Fgpu64r6InstrInfo.td"

// DSP
include "FgpuDSPInstrFormats.td"
include "FgpuDSPInstrInfo.td"

// MSA
include "FgpuMSAInstrFormats.td"
include "FgpuMSAInstrInfo.td"

// EVA
include "FgpuEVAInstrFormats.td"
include "FgpuEVAInstrInfo.td"

// MT
include "FgpuMTInstrFormats.td"
include "FgpuMTInstrInfo.td"
