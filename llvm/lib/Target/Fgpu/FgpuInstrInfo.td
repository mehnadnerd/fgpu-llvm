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
//include "FgpuRegisterInfo.td"

//===----------------------------------------------------------------------===//
// Fgpu profiles and nodes
//===----------------------------------------------------------------------===//

// Node immediate fits as 14-bit sign extended on target immediate.
// e.g. addi
def imm_eq_31  : PatLeaf<(i32 imm), [{ return (N->getZExtValue() == 31) ; }]>;
def imm_eq_16  : PatLeaf<(i32 imm), [{ return (N->getZExtValue() == 16) ; }]>;
def imm_eq_7   : PatLeaf<(i32 imm), [{ return (N->getZExtValue() == 7) ; }]>;
def imm_eq_4   : PatLeaf<(i32 imm), [{ return (N->getZExtValue() == 4) ; }]>;
def imm_eq_3   : PatLeaf<(i32 imm), [{ return (N->getZExtValue() == 3) ; }]>;
def imm_eq_2   : PatLeaf<(i32 imm), [{ return (N->getZExtValue() == 2) ; }]>;
def imm_eq_1   : PatLeaf<(i32 imm), [{ return (N->getZExtValue() == 1) ; }]>;

def immSExt7  : PatLeaf<(i32 imm), [{ return isInt<7>(N->getSExtValue()); }]>;
def immZExt7  : PatLeaf<(i32 imm), [{ return isUInt<7>(N->getSExtValue()); }]>;

def immSExt9  : PatLeaf<(i32 imm), [{ return isInt<9>(N->getSExtValue()); }]>;
def immZExt9  : PatLeaf<(i32 imm), [{ return isUInt<9>(N->getSExtValue()); }]>;

def immSExt10  : PatLeaf<(i32 imm), [{ return isInt<10>(N->getSExtValue()); }]>;
def immZExt10  : PatLeaf<(i32 imm), [{ return isUInt<10>(N->getSExtValue()); }]>;

def immSExt14  : PatLeaf<(i32 imm), [{ return isInt<14>(N->getSExtValue()); }]>;
def immZExt14  : PatLeaf<(i32 imm), [{ return isUInt<14>(N->getSExtValue()); }]>;

def immSExt16  : PatLeaf<(i32 imm), [{ return isInt<16>(N->getSExtValue()); }]>;
def immZExt16  : PatLeaf<(i32 imm), [{ return isUInt<16>(N->getSExtValue()); }]>;

def imm0       : PatLeaf<(i32 imm), [{ return (N->getZExtValue() == 0); }]>;
def uimm4       : PatLeaf<(i32 imm), [{ return isUInt<4>(N->getZExtValue()); }]>;
def uuimm16      : PatLeaf<(i32 imm), [{ return isUInt<16>(N->getZExtValue()); }]>;
def uimm5       : PatLeaf<(i32 imm), [{ return isUInt<5>(N->getZExtValue()); }]>;
def simm5       : PatLeaf<(i32 imm), [{ return isInt<5>(N->getSExtValue()); }]>;

def fpimm0      : PatLeaf<(f32 fpimm), [{ return N->isZero(); }]>;

// Return
def FgpuRET   : SDNode<"FgpuISD::RET", SDTNone, [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

//===----------------------------------------------------------------------===//
// Fgpu Instruction Predicate Definitions.
//===----------------------------------------------------------------------===//
//def HasFgpu2     :    Predicate<"Subtarget->hasFgpu2()">,
//                      AssemblerPredicate<(all_of FeatureFgpu2)>;
def RelocNotPIC :     Predicate<"!TM.isPositionIndependent()">;
def RelocPIC    :     Predicate<"TM.isPositionIndependent()">;
// TODO: Add support for FPOpFusion::Standard
def AllowFPOpFusion : Predicate<"TM.Options.AllowFPOpFusion =="
                                " FPOpFusion::Fast">;
//===----------------------------------------------------------------------===//
// Fgpu GPR size adjectives.
// They are mutually exclusive.
//===----------------------------------------------------------------------===//


//===----------------------------------------------------------------------===//
// Fgpu Symbol size adjectives.
// They are mutally exculsive.
//===----------------------------------------------------------------------===//



//===----------------------------------------------------------------------===//
// Fgpu ISA/ASE membership and instruction group membership adjectives.
// They are mutually exclusive.
//===----------------------------------------------------------------------===//


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

// Instruction operand types

// BEQ, BNE
def brtarget14    : Operand<OtherVT> {
 let EncoderMethod = "getBranch14TargetOpValue";
 let OperandType = "OPERAND_PCREL";
 let DecoderMethod = "DecodeSimm14";
}

def brtarget7    : Operand<OtherVT> {
 let EncoderMethod = "getBranch7TargetOpValue";
 let OperandType = "OPERAND_PCREL";
 let DecoderMethod = "DecodeSimm7";
}


def simm14      : Operand<i32> {
  let DecoderMethod= "DecodeSimm14";
}

def uimm14      : Operand<i32> {
  let DecoderMethod= "DecodeUimm14";
}

def simm7      : Operand<i32> {
  let DecoderMethod= "DecodeSimm7";
}

def uimm7      : Operand<i32> {
  let DecoderMethod= "DecodeUimm7";
}

def simm9      : Operand<i32> {
  let DecoderMethod= "DecodeSimm9";
}

def uimm9      : Operand<i32> {
  let DecoderMethod= "DecodeUimm9";
}


def simm10      : Operand<i32> {
  let DecoderMethod= "DecodeSimm10";
}

def uimm10      : Operand<i32> {
  let DecoderMethod= "DecodeUimm10";
}

def vecoffset9   : Operand<i32> {
  let DecoderMethod= "DecodeVecOffset9";
}


def calltarget  : Operand<iPTR> {
  let EncoderMethod = "getJumpTargetOpValue";
 let DecoderMethod = "DecodeSimm14";
}


// Unsigned Operand
def uimm16      : Operand<i32> {
  let PrintMethod = "printUnsignedShortImm";
  let DecoderMethod= "DecodeUimm16";
}

def imm32      : Operand<i32> {
  let PrintMethod = "printUnsignedImm";
}

def frameAddr:
  ComplexPattern<iPTR, 2, "selectFrameAddr", [frameindex], [SDNPWantParent]>;

def frameAddr_LEA:
  ComplexPattern<iPTR, 2, "selectFrameAddr_LEA", [frameindex], [SDNPWantParent]>;

def FgpuMemAsmOperand : AsmOperandClass {
  let Name = "Mem";
}

// Address operand
def mem : Operand<i32> {
  let PrintMethod = "printMemOperand";
  let MIOperandInfo = (ops ALURegs, immZExt14);
  let EncoderMethod = "getMemEncoding";
  let ParserMatchClass = FgpuMemAsmOperand;
  let DecoderMethod = "DecodeMem";
}


//===----------------------------------------------------------------------===//
// Pattern fragment for load/store
//===----------------------------------------------------------------------===//

class AlignedLoad<PatFrag Node> :
  PatFrag<(ops node:$ptr), (Node node:$ptr), [{
  LoadSDNode *LD = cast<LoadSDNode>(N);
  return (LD->getMemoryVT().getSizeInBits()/8 <= LD->getAlignment()) && (LD->getAddressSpace() == 0);
}]>;


class AlignedStore<PatFrag Node> :
  PatFrag<(ops node:$val, node:$ptr), (Node node:$val, node:$ptr), [{
  StoreSDNode *SD = cast<StoreSDNode>(N);
  return (SD->getMemoryVT().getSizeInBits()/8 <= SD->getAlignment()) && (SD->getAddressSpace() == 0);
}]>;

def load_a          : AlignedLoad<load>;
def loadi16_a       : AlignedLoad<extloadi16>;
def loadi8_a        : AlignedLoad<extloadi8>;
def store_a         : AlignedStore<store>;
def shr_load : PatFrag<(ops node:$ptr), (load node:$ptr), [{
  return cast<LoadSDNode>(N)->getAddressSpace() == 1;
}]>;
def shr_store : PatFrag<(ops node:$val, node:$ptr), (store node:$val, node:$ptr), [{
  return cast<StoreSDNode>(N)->getAddressSpace() == 1;
}]>;

//===----------------------------------------------------------------------===//
// Instructions specific format
//===----------------------------------------------------------------------===//
class CondR<bits<8> op, string instr_asm, PatFrag cond_op, RegisterClass RC, bit isComm = 0>:
  FRRR<op, (outs ALURegs:$rd), (ins RC:$rs, RC:$rt),
     !strconcat(instr_asm, "\t$rd, $rs, $rt"),
     [(set ALURegs:$rd, (cond_op RC:$rs, RC:$rt))],
     IIAlu> {
  let isCommutable = isComm;	// e.g. add rb rc =  add rc rb
}

class CondI<bits<8> op, string instr_asm, PatFrag cond_op, Operand Od, PatLeaf imm_type>:
  FRRI<op, (outs ALURegs:$rd), (ins ALURegs:$rs, Od:$imm14),
     !strconcat(instr_asm, "\t$rd, $rs, $imm14"),
     [(set ALURegs:$rd, (cond_op ALURegs:$rs, imm_type:$imm14))],
     IIAlu>;

// Arithmetic and logical instructions with 3 register operands.
class ArithLogicR<bits<8> op, string instr_asm, SDNode OpNode,
                  InstrItinClass itin, RegisterClass RC, bit isComm = 0>:
  FRRR<op, (outs RC:$rd), (ins RC:$rs, RC:$rt),
     !strconcat(instr_asm, "\t$rd, $rs, $rt"),
     [(set RC:$rd, (OpNode RC:$rs, RC:$rt))], itin> {
  let isCommutable = isComm;	// e.g. add rb rc =  add rc rb
  let isReMaterializable = 1;
}

//Floating-point operation
class FPInsn<bits<8> op, string instr_asm, SDNode OpNode, InstrItinClass itin, RegisterClass RC, bit isComm = 0>:
  FRRR<op, (outs RC:$rd), (ins RC:$rs, RC:$rt), !strconcat(instr_asm, "\t$rd, $rs, $rt"),
     [(set RC:$rd, (OpNode RC:$rs, RC:$rt))], itin> {
  let isCommutable = isComm;	// e.g. add rb rc =  add rc rb
  let isReMaterializable = 1;
}

class FPInsnUnary<bits<8> op, string instr_asm, SDNode OpNode, InstrItinClass itin, RegisterClass RC, bit isComm = 0>:
  FRR<op, (outs RC:$rd), (ins RC:$rs), !strconcat(instr_asm, "\t$rd, $rs"),
     [(set RC:$rd, (OpNode RC:$rs))], itin> {
  let isReMaterializable = 1;
}

class FFMA<bits<8> op, string instr_asm, InstrItinClass itin, RegisterClass RC, bit isComm = 0>:
  FRRR<op, (outs RC:$rd_out), (ins RC:$rd, RC:$rs, RC:$rt), !strconcat(instr_asm, "\t$rd, $rs, $rt"),
     [(set RC:$rd_out, (fadd RC:$rd, (fmul RC:$rs, RC:$rt)))], itin> {
  let isCommutable = isComm;
  let isReMaterializable = 1;
  let Constraints = "$rd_out = $rd";
}

// class VRRR<bits<8> op, string instr_asm, InstrItinClass itin, bit isComm = 0>:
//   FPU_VVV<op, (outs FloatRegs:$rd_out), (ins FloatRegs:$rd, FloatRegs:$rs, FloatRegs:$rt),
// 	  !strconcat(instr_asm, "\t$rd, $rs, $rt"),
// 	  [(set FloatRegs:$rd_out, (add FloatRegs:$rd, (mul FloatRegs:$rs, FloatRegs:$rt)))], itin> {
//   let isCommutable = isComm;
//   let isReMaterializable = 1;
//   let Constraints = "$rd_out = $rd";
// }

class VRVI<bits<8> op, string instr_asm>:
  FPU_VRVI<op,(outs VecRegs:$rd), (ins VecRegs:$rs, ALURegs:$rt, uimm9:$imm9),
	  !strconcat(instr_asm, "\t$rd, $rs, $rt, $imm9"),
	  [], IIAlu> {
  let isCommutable = 0;
}

class VRVV<bits<8> op, string instr_asm>:
  FPU_VRVI<op,(outs VecRegs:$rd_out), (ins VecRegs:$rd, VecRegs:$rs, ALURegs:$rt, uimm9:$imm9),
	  !strconcat(instr_asm, "\t$rd, $rs, $rt, $imm9"),
	  [], IIAlu> {
  let isCommutable = 0;
  let Constraints = "$rd_out = $rd";
}

class RVI<bits<8> op, string instr_asm>:
  FPU_RVI<op,(outs VecRegs:$rd), (ins ALURegs:$rs, ALURegs:$rt, uimm9:$imm9),
	  !strconcat(instr_asm, "\t$rd, $rs, $rt, $imm9"),
	  [], IIAlu> {
  let isCommutable = 0;
}

class ACTC<bits<8> op, string instr_asm>:
    FRRI<op, (outs ALURegs:$rd), (ins ALURegs:$rs, uimm14:$imm14),
     !strconcat(instr_asm, "\t$rd, $rs, $imm14"),
     [], IIAlu> {}

class CFG<bits<8> op, string instr_asm>:
    FRI<op, (outs), (ins ALURegs:$rd, uimm14:$imm16),
     !strconcat(instr_asm, "\t$rd, $imm16"),
     [], IIAlu> {}

//class BUT16<bits<8> op, string instr_asm>:
//    FPU_VVV<op, (outs VecRegs:$rd_out, VecRegs:$rs_out), (ins VecRegs:$rd, VecRegs:$rs, VecRegs:$rt),
//    !strconcat(instr_asm, "\t$rd, $rs, $rt"),
//    [], IIAlu> {
//  let isCommutable = 0;
//  let Constraints = "$rd_out = $rd, $rs_out = $rs";
//}

//class BUT0<bits<8> op, string instr_asm>:
//    FPU_VVVR<op, (outs VecRegs:$rd_out, VecRegs:$rs_out), (ins VecRegs:$rd, VecRegs:$rs, VecRegs:$rt, ALURegs:$rx),
//    !strconcat(instr_asm, "\t$rd, $rs, $rt, $rx"),
//    [], IIAlu> {
//  let isCommutable = 0;
//  let Constraints = "$rd_out = $rd, $rs_out = $rs";
//}

//class GTWID<bits<8> op, string instr_asm>:
//  FPU_VRRR<op,(outs VecRegs:$rd), (ins ALURegs:$rs, ALURegs:$rt, ALURegs:$rx),
//	  !strconcat(instr_asm, "\t$rd, $rs, $rt, $rx"),
//	  [], IIAlu> {
//  let isCommutable = 0;
//}

class VVVI<bits<8> op, string instr_asm>:
    FPU_VVVI<op, (outs VecRegs:$rd), (ins VecRegs:$rs, VecRegs:$rt, uimm9:$imm9),
    !strconcat(instr_asm, "\t$rd, $rs, $rt, $imm9"),
    [], IIAlu> {
  let isCommutable = 0;
}

class VVI<bits<8> op, string instr_asm>:
    FPU_VVVI<op, (outs VecRegs:$rd), (ins VecRegs:$rs, uimm9:$imm9),
    !strconcat(instr_asm, "\t$rd, $rs, $imm9"),
    [], IIAlu> {
  let isCommutable = 0;
}

class VVRV<bits<8> op, string instr_asm>:
    FPU_VVRI<op, (outs ALURegs:$rd, VecRegs:$rs_out), (ins VecRegs:$rs, VecRegs:$rt, uimm9:$imm9),
    !strconcat(instr_asm, "\t$rd, $rs, $rt, $imm9"),
    [], IIAlu> {
  let isCommutable = 0;
  let isReMaterializable = 1;
  let Constraints = "$rs_out = $rs";
}

class VRRI<bits<8> op, string instr_asm>:
  FPU_RVRI<op,(outs ALURegs:$rd), (ins VecRegs:$rs, ALURegs:$rt, uimm9:$imm9),
	  !strconcat(instr_asm, "\t$rd, $rs, $rt, $imm9"),
	  [], IIAlu> {
  let isCommutable = 0;
}

class VDOTC<bits<8> op, string instr_asm>:
  FPU_RVRI<op, (outs ALURegs:$rd_out), (ins ALURegs:$rd, VecRegs:$rs, ALURegs:$rt, simm9:$imm9),
	  !strconcat(instr_asm, "\t$rd, $rs, [$rt + $imm9]"),
	  [], IIAlu> {
  let isCommutable = 0;
  let isReMaterializable = 1;
  let mayLoad = 1;
  let Constraints = "$rd_out = $rd";
}

///* x86-style operation with a memory operand */
//class VVRI<bits<8> op, string instr_asm, InstrItinClass itin, bit isComm = 0>:
//  FPU_VVRI<op, (outs FloatRegs:$rd_out), (ins FloatRegs:$rd, FloatRegs:$rs, ALURegs:$rt, immSExt9:$imm9),
//	  !strconcat(instr_asm, "\t$rd, $rs, [$rt + $imm9]"),
//	  [(set FloatRegs:$rd_out, (add FloatRegs:$rd, (mul FloatRegs:$rs,  (load_a (add ALURegs:$rt, immSExt9:$imm9)))))], itin> {
//  let isCommutable = isComm;
//  let isReMaterializable = 1;
//  let mayLoad = 1;
//  let Constraints = "$rd_out = $rd";
//}


class LogicNOR<bits<8> op, string instr_asm,
                InstrItinClass itin, RegisterClass RC, bit isComm = 0>:
  FRRR<op, (outs RC:$rd), (ins RC:$rs, RC:$rt),
         !strconcat(instr_asm, "\t$rd, $rs, $rt"),
         [(set RC:$rd, (not (or RC:$rs, RC:$rt)))], itin> {
  let isCommutable = 1;
}

class ArithOneARGTwoClasses<bits<8> op, string instr_asm, SDNode OpNode,
                  InstrItinClass itin, RegisterClass RCin, RegisterClass RCout, bit isComm = 0>:
  FRRR<op, (outs RCout:$rd), (ins RCin:$rs),
     !strconcat(instr_asm, "\t$rd, $rs"),
     [(set RCout:$rd, (OpNode RCin:$rs))], itin> {
  let isCommutable = isComm;	// e.g. add rb rc =  add rc rb
  let isReMaterializable = 1;
}

class ArithOneARG<bits<8> op, string instr_asm, SDNode OpNode,
                  InstrItinClass itin, RegisterClass RC, bit isComm = 0>:
  FRRR<op, (outs RC:$rd), (ins RC:$rs),
     !strconcat(instr_asm, "\t$rd, $rs"),
     [(set RC:$rd, (OpNode RC:$rs))], itin> {
  let isCommutable = isComm;	// e.g. add rb rc =  add rc rb
  let isReMaterializable = 1;
}

//def ADDi    : ArithLogicI<0x11, "addi", add, simm14, immSExt14>;
class ArithLogicI<bits<8> op, string instr_asm, SDNode OpNode, Operand Od, PatLeaf imm_type> :
  FRRI<op, (outs ALURegs:$rd), (ins ALURegs:$rs, Od:$imm14),
     !strconcat(instr_asm, "\t$rd, $rs, $imm14"),
     [(set ALURegs:$rd, (OpNode ALURegs:$rs, imm_type:$imm14))], IIAlu> {
  let isReMaterializable = 1;
}
class Macc<bits<8> op, string instr_asm, InstrItinClass itin, RegisterClass RC, bit isComm = 0>:
  FRRR<op, (outs RC:$rd_out), (ins RC:$rd, RC:$rs, RC:$rt),
     !strconcat(instr_asm, "\t$rd, $rs, $rt"),
     [(set RC:$rd_out, (add RC:$rd, (mul RC:$rs, RC:$rt)))], itin> {
  let isCommutable = isComm;
  let isReMaterializable = 1;
  let Constraints = "$rd_out = $rd";
}


//class FPDOT<bits<8> op, string instr_asm, SDNode OpNode, RegisterClass RC, bit isComm = 0>:
//  FPU_VVV<op, (outs RC:$rd_out), (ins RC:$rd, RC:$rs, RC:$rt),
//     !strconcat(instr_asm, "\t$rd, $rs, $rt"),
//     [(set RC:$rd_out, (OpNode RC:$rd, RC:$rs, RC:$rt))], IIAlu> {
//  let isCommutable = isComm;
//  let isReMaterializable = 1;
//  let Constraints = "$rd_out = $rd";
//}


//Runtime Memory instructions
class RTMInstr<bits<8> op, string instr_asm, SDNode OpNode, Operand Imm, PatLeaf imm_type>:
  FRI_RTM<op, (outs ALURegs:$rd), (ins Imm:$uimm4),
     !strconcat(instr_asm, "\t$rd, $uimm4"),
     [(set ALURegs:$rd, (OpNode imm_type:$uimm4))], IIAlu> {
  let isReMaterializable = 1;
}

class LoadUpper<bits<8> op, string instr_asm, SDNode OpNode, RegisterClass RC, Operand Imm>:
  FRI<op, (outs RC:$rd), (ins Imm:$imm16),
     !strconcat(instr_asm, "\t$rd, $imm16"),
     [(set ALURegs:$rd, uuimm16:$imm16)], IIAlu> { // I think correvt
  let isReMaterializable = 1;
  let AddedComplexity = 20;
}

class LoadLower<bits<8> op, string instr_asm, SDNode OpNode, RegisterClass RC, Operand Imm>:
  FRI<op, (outs RC:$rd_out), (ins RC:$rd, Imm:$imm16),
     !strconcat(instr_asm, "\t$rd, $imm16"),
     [(set ALURegs:$rd_out, (or (shl (srl ALURegs:$rd, imm_eq_16), imm_eq_16 ), uuimm16:$imm16))], IIAlu> { // I think is correct
  let isReMaterializable = 1;
  let AddedComplexity = 20;
  let Constraints = "$rd_out = $rd";
}

class Control<bits<8> op, string instr_asm>:
  FCTRL<op, (outs), (ins),
     instr_asm, [], IIBranch> {
  let isTerminator=1;
  let isBarrier=1;
  let Uses=[LR];
}


class Sync_<bits<8> op, string instr_asm>:
  FCTRL<op, (outs), (ins), instr_asm, [], IIBranch> {
  let isBarrier=1;
}

def mem_ea : Operand<iPTR> {
  let PrintMethod = "printMemOperandEA";
  let MIOperandInfo = (ops ALURegs, immZExt14);
  let EncoderMethod = "getMemEncoding";
}
class EffectiveAddress<string instr_asm, RegisterClass RC, Operand Mem> :
  FRRI_Stack<0x11, (outs RC:$rd), (ins Mem:$addr),
  instr_asm, [(set RC:$rd, frameAddr:$addr)], IIAlu>;

def LEA_ADDiu : EffectiveAddress<"addi\t$rd, $addr", ALURegs, mem_ea> {
let isCodeGenOnly = 1;
}

class Load_Frame<bits<8> op, string instr_asm, PatFrag OpNode, RegisterClass RC>
            : FRRI_Stack<op, (outs RC:$rd), (ins mem:$addr),
            !strconcat(instr_asm, "\t$rd, $addr"),
            [(set RC:$rd,(OpNode frameAddr:$addr))], IILoad>{
  let isPseudo = 0;
  let mayLoad = 1;
}
class Load_Byte<bits<8> op, string instr_asm, PatFrag OpNode, RegisterClass RC>
            : FRRR<op, (outs RC:$rd), (ins RC:$rs, RC:$rt),
            !strconcat(instr_asm, "\t$rd, [$rt + $rs]"),
            [(set RC:$rd,(OpNode (add RC:$rt,  RC:$rs)))], IILoad>{
  let isPseudo = 0;
  let mayLoad = 1;
}
class Load_Half<bits<8> op, string instr_asm, PatFrag OpNode, RegisterClass RC>
            : FRRR<op, (outs RC:$rd), (ins RC:$rs, RC:$rt),
            !strconcat(instr_asm, "\t$rd, [$rt + $rs]"),
            [(set RC:$rd, (OpNode (add RC:$rt, (shl RC:$rs, imm_eq_1))))], IILoad>{
  let isPseudo = 0;
  let mayLoad = 1;
}
class Load_Word<bits<8> op, string instr_asm, PatFrag OpNode, RegisterClass RC>
            : FRRR<op, (outs RC:$rd), (ins ALURegs:$rs, ALURegs:$rt),
            !strconcat(instr_asm, "\t$rd, [$rt + $rs]"),
            [(set RC:$rd, (OpNode (add ALURegs:$rt, (shl ALURegs:$rs, imm_eq_2))))], IILoad>{
  let isPseudo = 0;
  let mayLoad = 1;
}


class Load_LWC<bits<8> op, string instr_asm, PatFrag OpNode, RegisterClass RC>
  : FPU_MEMC<op, (outs RC:$rd), (ins ALURegs:$rt, ALURegs:$rs, vecoffset9:$imm9),
            !strconcat(instr_asm, "\t$rd, [$rt + $rs + $imm9]"),
    [(set RC:$rd, (OpNode (add ALURegs:$rt, (add (shl ALURegs:$rs, imm_eq_7), immSExt16:$imm9))))], IILoad>{
  let isPseudo = 0;
  let mayLoad = 1;
  let AddedComplexity = 1000;
}


class Store_SWC<bits<8> op, string instr_asm, PatFrag OpNode, RegisterClass RC>
  : FPU_MEMC<op, (outs), (ins RC:$rd, ALURegs:$rt, ALURegs:$rs, simm9:$imm9),
            !strconcat(instr_asm, "\t$rd, [$rt + $rs + $imm9]"),
    [(OpNode RC:$rd, (add ALURegs:$rt, (add (shl ALURegs:$rs, imm_eq_7), immSExt16:$imm9)))], IIStore>{
  let isPseudo = 0;
  let mayStore = 1;
  let AddedComplexity = 1000;
}

class Load_SLWC<bits<8> op, string instr_asm, PatFrag OpNode, RegisterClass RC> :
  FPU_SMEMC<op, (outs RC:$rd), (ins ALURegs:$rt, ALURegs:$rs, vecoffset9:$imm9),
            !strconcat(instr_asm, "\t$rd, [$rt + $rs + $imm9]"),
    [], IILoad>{
  let isPseudo = 0;
  let mayLoad = 1;
}

class Store_SSWC<bits<8> op, string instr_asm, PatFrag OpNode, RegisterClass RC>
  : FPU_SMEMC<op, (outs), (ins RC:$rd, ALURegs:$rt, ALURegs:$rs, simm9:$imm9),
            !strconcat(instr_asm, "\t$rd, [$rt + $rs + $imm9]"),
    [(set RC:$rd, (OpNode (add ALURegs:$rt, (add (shl ALURegs:$rs, imm_eq_7), immSExt16:$imm9))))], IILoad>{
  let isPseudo = 0;
  let mayStore = 1;
  let AddedComplexity = 1000;
}

class Load_SLW<bits<8> op, string instr_asm, PatFrag OpNode, RegisterClass RC> :
  FPU_SMEM<op, (outs RC:$rd), (ins ALURegs:$rt, ALURegs:$rs, simm9:$imm9),
            !strconcat(instr_asm, "\t$rd, [$rt + $rs + $imm9]"),
    [(set RC:$rd, (OpNode (add ALURegs:$rt, (add ALURegs:$rs, immSExt9:$imm9))))], IILoad>{
  let isPseudo = 0;
  let mayLoad = 1;
}

class Store_SSW<bits<8> op, string instr_asm, PatFrag OpNode, RegisterClass RC>
  : FPU_SMEM<op, (outs), (ins RC:$rd, ALURegs:$rt, ALURegs:$rs, simm9:$imm9),
            !strconcat(instr_asm, "\t$rd, [$rt + $rs + $imm9]"),
    [(OpNode RC:$rd, (add ALURegs:$rt, (add ALURegs:$rs, immSExt9:$imm9)))], IIStore>{
  let isPseudo = 0;
  let mayStore = 1;
}

class Store_Frame<bits<8> op, string instr_asm, PatFrag OpNode, RegisterClass RC>
            : FRRI_Stack<op, (outs), (ins RC:$rd, mem:$addr),
            !strconcat(instr_asm, "\t$rd, $addr"),
            [(OpNode RC:$rd, frameAddr:$addr)], IIStore>{
  let isPseudo = 0;
  let mayStore = 1;
}
class Store_Byte<bits<8> op, string instr_asm, PatFrag OpNode, RegisterClass RC>
            : FRRR<op, (outs), (ins RC:$rd, RC:$rs, RC:$rt),
            !strconcat(instr_asm, "\t$rd, [$rt + $rs]"),
            [(OpNode RC:$rd, (add RC:$rt,  RC:$rs))], IIStore>{
  let isPseudo = 0;
  let mayStore = 1;
}
class Store_Half<bits<8> op, string instr_asm, PatFrag OpNode, RegisterClass RC>
            : FRRR<op, (outs), (ins RC:$rd, RC:$rs, RC:$rt),
            !strconcat(instr_asm, "\t$rd, [$rt + $rs]"),
            [(OpNode RC:$rd, (add RC:$rt, (shl RC:$rs, imm_eq_1)))], IIStore>{
  let isPseudo = 0;
  let mayStore = 1;
}
class Store_Word<bits<8> op, string instr_asm, PatFrag OpNode, RegisterClass RC>
            : FRRR<op, (outs), (ins RC:$rd, ALURegs:$rs, ALURegs:$rt),
            !strconcat(instr_asm, "\t$rd, [$rt + $rs]"),
            [(OpNode RC:$rd, (add ALURegs:$rt, (shl ALURegs:$rs, imm_eq_2)))], IIStore>{
  let isPseudo = 0;
  let mayStore = 1;
}

class LBZ_<bits<8> op, string instr_asm>
  : FRRRI<op, (outs ALURegs:$rd), (ins ALURegs:$rs, ALURegs:$rt, simm9:$imm9),
            !strconcat(instr_asm, "\t$rd, $rs, [$rt + $imm9]"),
            [], IIStore>{
  let isPseudo = 0;
  let mayLoad = 1;
}

class SBZ_<bits<8> op, string instr_asm>
  : FRRRI<op, (outs), (ins ALURegs:$rd, ALURegs:$rs, ALURegs:$rt, simm9:$imm9),
            !strconcat(instr_asm, "\t$rd, $rs, [$rt + $imm9]"),
            [], IIStore>{
  let isPseudo = 0;
  let mayStore = 1;
}

class LWZ_<bits<8> op, string instr_asm>
  : FRRRI<op, (outs ALURegs:$rd), (ins ALURegs:$rs, ALURegs:$rt, simm9:$imm9),
            !strconcat(instr_asm, "\t$rd, $rs, [$rt + $imm9]"),
            [], IIStore>{
  let isPseudo = 0;
  let mayLoad = 1;
}

class SWZ_<bits<8> op, string instr_asm>
  : FRRRI<op, (outs), (ins ALURegs:$rd, ALURegs:$rs, ALURegs:$rt, simm9:$imm9),
            !strconcat(instr_asm, "\t$rd, $rs, [$rt + $imm9]"),
            [], IIStore>{
  let isPseudo = 0;
  let mayStore = 1;
}

class LWCZ_<bits<8> op, string instr_asm>
  : FVRRI<op, (outs VecRegs:$rd), (ins ALURegs:$rs, ALURegs:$rt, simm9:$imm9),
            !strconcat(instr_asm, "\t$rd, $rs, [$rt + $imm9]"),
            [], IILoad>{
  let isPseudo = 0;
  let mayLoad = 1;
}

class SWCZ_<bits<8> op, string instr_asm>
  : FVRRI<op, (outs), (ins VecRegs:$rd, ALURegs:$rs, ALURegs:$rt, simm9:$imm9),
            !strconcat(instr_asm, "\t$rd, $rs, [$rt + $imm9]"),
            [], IIStore>{
  let isPseudo = 0;
  let mayStore = 1;
}

class SWC_<bits<8> op, string instr_asm>
  : FVRRI<op, (outs), (ins VecRegs:$rd, ALURegs:$rs, ALURegs:$rt, simm9:$imm9),
            !strconcat(instr_asm, "\t$rd, [$rt + $rs + $imm9]"),
            [], IIStore>{
  let isPseudo = 0;
  let mayStore = 1;
}

class Branch<bits<8> op, string instr_asm, PatFrag cond_op,
                  Operand Od, RegisterClass RC> :
  FRRI<op, (outs), (ins RC:$rd, RC:$rs, Od:$imm14),
     !strconcat(instr_asm, "\t$rd, $rs, $imm14"),
     [(brcond (i32 (cond_op RC:$rd, RC:$rs)), bb:$imm14)], IIBranch>{
  let isBranch = 1;
  let isTerminator = 1;
// let isBarrier = 1; // WTF was this marked 1???????
}



def SDT_FgpuJmpLink      : SDTypeProfile<0, 1, [SDTCisVT<0, iPTR>]>;
def FgpuJmpLink : SDNode<"FgpuISD::JmpLink",SDT_FgpuJmpLink,
                         [SDNPHasChain, SDNPOutGlue, SDNPOptInGlue,
                          SDNPVariadic]>;
class JmpLink<bits<8> op, string instr_asm>:
  FI<op, (outs), (ins calltarget:$imm14, variable_ops),
     !strconcat(instr_asm, "\t$imm14"),
      [(FgpuJmpLink imm:$imm14)], IIBranch>{
  let isCall=1;
  let Defs=[LR];
}

//def SDT_FgpuJmpLinkRegister      : SDTypeProfile<0, 2, [SDTCisVT<0, iPTR>, SDTCisInt<0>]>;
//def FgpuJmpLinkRegister : SDNode<"FgpuISD::JmpLinkRegister",SDT_FgpuJmpLinkRegister,
//                         [SDNPHasChain, SDNPOutGlue, SDNPOptInGlue,
//                          SDNPVariadic]>;
//class JmpLinkRegister<bits<8> op, string instr_asm>:
//  FRI<op, (outs), (ins ALURegs:$rd, uimm14:$imm14, variable_ops),
//     !strconcat(instr_asm, "\t$rd, $imm14"),
//      [(FgpuJmpLinkRegister ALURegs:$rd, imm:$imm14)], IIBranch>{
//  let isCall=1;
//  let Defs=[LR];
//  let Uses=[LR];
//}

class SetSync_<bits<8> op, string instr_asm>:
  FI<op, (outs), (ins calltarget:$imm14, variable_ops),
     !strconcat(instr_asm, "\t$imm14"),
      [(FgpuJmpLink imm:$imm14)], IIBranch>{
  let isBarrier=1;
}

//===----------------------------------------------------------------------===//
// Pseudo instructions
//===----------------------------------------------------------------------===//

def SDTRTMInst    : SDTypeProfile<1, 1, [ SDTCisInt<1>]>;
def SDTACTInst    : SDTypeProfile<1, 1, [ SDTCisFP<0>]>;
def SDTAto        : SDTypeProfile<1, 2, [SDTCisSameAs<0,2>, SDTCisInt<0>, SDTCisPtrTy<1>]>;

//let hasNoSchedulingInfo = 1 in {
def LUi_Pseudo    : SDNode<"FgpuISD::LUi"      , SDTRTMInst, []>;
def Li_Pseudo     : SDNode<"FgpuISD::Li"      , SDTRTMInst, []>;
def RELU_Pseudo   : SDNode<"FgpuISD::RELU"    , SDTACTInst, []>;
def LP_Pseudo     : SDNode<"FgpuISD::LP"      , SDTRTMInst, [SDNPHasChain]>;
def LID_Pseudo    : SDNode<"FgpuISD::LID"    , SDTRTMInst, [SDNPHasChain]>;
def WGOFF_Pseudo  : SDNode<"FgpuISD::WGOFF"    , SDTRTMInst, [SDNPHasChain]>;
def WGID_Pseudo   : SDNode<"FgpuISD::WGID"    , SDTRTMInst, [SDNPHasChain]>;
def WGSIZE_Pseudo : SDNode<"FgpuISD::WGSIZE"    , SDTRTMInst, [SDNPHasChain]>;
def SIZE_Pseudo   : SDNode<"FgpuISD::SIZE"    , SDTRTMInst, [SDNPHasChain]>;

def DOT_Pseudo    : SDNode<"FgpuISD::DOT" , SDTypeProfile<1, 3, []>, []>;

def NOP_Pseudo    : SDNode<"FgpuISD::NOP"      , SDTRTMInst, []>;


def AADD_Pseudo   : SDNode<"FgpuISD::AADD"  , SDTAto, [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand]>;
def AMAX_Pseudo   : SDNode<"FgpuISD::AMAX"  , SDTAto, [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand]>;

def SDT_FgpuCallSeqStart : SDCallSeqStart<[SDTCisVT<0, i32>, SDTCisVT<1, i32>]>;
def SDT_FgpuCallSeqEnd   : SDCallSeqEnd<[SDTCisVT<0, i32>, SDTCisVT<1, i32>]>;
def callseq_start : SDNode<"ISD::CALLSEQ_START", SDT_FgpuCallSeqStart,
                           [SDNPHasChain, SDNPOutGlue]>;
def callseq_end   : SDNode<"ISD::CALLSEQ_END", SDT_FgpuCallSeqEnd,
                           [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

let Defs = [SP], Uses = [SP], hasSideEffects = 1, hasNoSchedulingInfo = 1 in {
  def ADJCALLSTACKDOWN : FgpuPseudo<(outs), (ins uimm14:$amt1, uimm14:$amt2),
                                    "!ADJCALLSTACKDOWN $amt1",
                                    [(callseq_start timm:$amt1, timm:$amt2)]>;
  def ADJCALLSTACKUP   : FgpuPseudo<(outs), (ins uimm14:$amt1, uimm14:$amt2),
                                    "!ADJCALLSTACKUP $amt1",
                                    [(callseq_end timm:$amt1, timm:$amt2)]>;
}
//}
//===----------------------------------------------------------------------===//
// Instruction definition
//===----------------------------------------------------------------------===//
def LW : Load_Word<0x74, "lw", load_a,    ALURegs>;
def LH : Load_Half<0x72, "lh", loadi16_a, ALURegs>;
def LB : Load_Byte<0x71, "lb", loadi8_a,  ALURegs>;

def LLWI : Load_Frame<0xD4, "llwi", load_a, ALURegs>;


// map load without index, i.e. lw rd, rt[0]
def : Pat<(load_a ALURegs:$in), (LW ZERO,ALURegs:$in )>;
def SW : Store_Word<0x7C, "sw", store_a, ALURegs>;
def SH : Store_Half<0x7A, "sh", truncstorei16, ALURegs>;
def SB : Store_Byte<0x79, "sb", truncstorei8, ALURegs>;


def LSWI : Store_Frame<0xDC, "lswi", store_a, ALURegs>;


// map store without index, i.e. sw rd, rt[0]
def : Pat<(store_a ALURegs:$rd, ALURegs:$addr), (SW ALURegs:$rd, ZERO, ALURegs:$addr)>;
def : Pat<(truncstorei16 ALURegs:$rd, ALURegs:$addr), (SH ALURegs:$rd, ZERO, ALURegs:$addr)>;
def : Pat<(truncstorei8 ALURegs:$rd, ALURegs:$addr), (SB ALURegs:$rd, ZERO, ALURegs:$addr)>;

/// Arithmetic Instructions (ALU Immediate)
def ADDi    : ArithLogicI<0x11, "addi", add, simm14, immSExt14>;
def ADD     : ArithLogicR<0x10, "add", add, IIAlu, ALURegs, 1>;

def VDOT    : VDOTC<0xD1, "vdot">;
def VDOTM0  : VDOTC<0xD2, "vdotm0">;
def VDOTM1  : VDOTC<0xD8, "vdotm1">;
def ACT     : ACTC<0xE4, "act">;
def CFG     : CFG<0xE0, "cfg">;
def FFMA    : FFMA<0xC9, "ffma", IIAlu, FloatRegs, 0>;
//def BUT16   : BUT16<0xE4, "but16">;
//def BUT0    : BUT0<0xE5, "but0">;
//def GTWID   : GTWID<0xE6, "gtwid">;

def VRVI    : VRVI<0xED, "vrvi">;
def VRVV    : VRVV<0xEE, "vrvv">;
def VRRI    : VRRI<0xE5, "vrri">;
def VVVI    : VVVI<0xEA, "vvvi">;
def VVI     : VVI<0xE9, "vvi">;
def VVRV    : VVRV<0xE3, "vvrv">;
def RVI     : RVI<0xEC, "rvi">;

def LBZ  : LBZ_<0xF1, "lbz">;
def SBZ  : SBZ_<0xF9, "sbz">;
def LWZ  : LWZ_<0xF5, "lwz">;
def SWZ  : SWZ_<0xFD, "swz">;
def LWCZ : LWCZ_<0xF3, "lwcz">;
def SWCZ : SWCZ_<0xFB, "swcz">;
def SLWCZ : LWCZ_<0xF7, "slwcz">;
def SSWCZ : SWCZ_<0xFF, "sswcz">;

def FADD    : FPInsn<0xC0, "fadd", fadd, IIAlu, FloatRegs, 1>;
def FMUL    : FPInsn<0xC1, "fmul", fmul, IIAlu, FloatRegs, 1>;
def FSUB    : FPInsn<0xC8, "fsub", fsub, IIAlu, FloatRegs, 0>;
def FDIV    : FPInsn<0xC2, "fdiv", fdiv, IIAlu, FloatRegs, 0>;
def FSQRT   : FPInsnUnary<0xC4, "fqrt", fsqrt, IIAlu, FloatRegs, 0>;


def LWC : Load_LWC<0x77, "lwc", load_a, VecRegs>;
// def : Pat<( v128i8 (load_a (add ALURegs:$rt, (shl ALURegs:$rs, imm_eq_7)))),        (LWC ALURegs:$rt, ALURegs:$rs, 0)>;
// def : Pat<( v128i8 (load_a (add ALURegs:$rt, immSExt16:$imm))),                     (LWC ALURegs:$rt, ZERO, immSExt16:$imm)>;
// def : Pat<( v128i8 (load_a ALURegs:$rt)),                                           (LWC ALURegs:$rt, ZERO, 0)>;
def : Pat<( v32f32 (load_a (add ALURegs:$rt, (shl ALURegs:$rs, imm_eq_7)))),        (LWC ALURegs:$rt, ALURegs:$rs, 0)>;
def : Pat<( v32f32 (load_a (add ALURegs:$rt, immSExt16:$imm))),                     (LWC ALURegs:$rt, ZERO, immSExt16:$imm)>;
def : Pat<( v32f32 (load_a ALURegs:$rt)),                                           (LWC ALURegs:$rt, ZERO, 0)>;

//def LLWC : Load_LLWC<0xD7, "llwc", load_a, FloatRegs>;
//def : Pat<( f32 (load_a (add ALURegs:$rs, ALURegs:$rt))),                       (LLWC ALURegs:$rt, ALURegs:$rs, 0)>;
//def : Pat<( f32 (load_a (add ALURegs:$rs, immSExt6:$imm6))),                    (LLWC ZERO, ALURegs:$rs, immSExt6:$imm6)>;
//def : Pat<( f32 (load_a ALURegs:$rs)),                                          (LLWC ZERO, ALURegs:$rs, 0)>;



def SWC : SWC_<0x7F, "swc">;
//def SWC : Store_SWC<0x7F, "swc", store_a, VecRegs>;
def : Pat<(store_a VecRegs:$rd, (add ALURegs:$rt, (shl ALURegs:$rs, imm_eq_7))),  (SWC VecRegs:$rd, ALURegs:$rt, ALURegs:$rs, 0)>;
def : Pat<(store_a VecRegs:$rd, (add ALURegs:$rt, immSExt16:$imm)),               (SWC VecRegs:$rd, ALURegs:$rt, ZERO, immSExt16:$imm)>;
def : Pat<(store_a VecRegs:$rd, ALURegs:$rt),                                     (SWC VecRegs:$rd, ALURegs:$rt, ZERO, 0)>;

//def LSWC : Store_LSWC<0xDF, "lswc", store_a, FloatRegs>;
//def : Pat<(store_a FloatRegs:$rs, (add ALURegs:$addr, ALURegs:$rt)),                       (LSWC FloatRegs:$rs, ALURegs:$rt, ALURegs:$addr, 0)>;
//def : Pat<(store_a FloatRegs:$rs, (add ALURegs:$addr, immSExt6:$imm6)),                    (LSWC FloatRegs:$rs, ZERO, ALURegs:$addr, immSExt6:$imm6)>;
//def : Pat<(store_a FloatRegs:$rs, ALURegs:$addr),                                          (LSWC FloatRegs:$rs, ZERO, ALURegs:$addr, 0)>;


def SLWC : Load_SLWC<0xD7, "slwc", shr_load, VecRegs>;
// def : Pat<( v128i8 (shr_load (add ALURegs:$rt, (shl ALURegs:$rs, imm_eq_7)))),        (SLWC ALURegs:$rt, ALURegs:$rs, 0)>;
// def : Pat<( v128i8 (shr_load (add ALURegs:$rt, immSExt16:$imm))),                     (SLWC ALURegs:$rt, ZERO, immSExt16:$imm)>;
// def : Pat<( v128i8 (shr_load ALURegs:$rt)),                                           (SLWC ALURegs:$rt, ZERO, 0)>;

def SSWC : SWC_<0xDF, "sswc">;
//def SSWC : Store_SSWC<0xDF, "sswc", store_a, VecRegs>;
//def : Pat<(store_a VecRegs:$rs, (add ALURegs:$addr, ALURegs:$rt)),                (SSWC VecRegs:$rs, ALURegs:$rt, ALURegs:$addr, 0)>;
//def : Pat<(store_a VecRegs:$rs, (add ALURegs:$addr, immSExt9:$imm6)),             (SSWC VecRegs:$rs, ZERO, ALURegs:$addr, immSExt9:$imm6)>;
//def : Pat<(store_a VecRegs:$rs, ALURegs:$addr),                                   (SSWC VecRegs:$rs, ZERO, ALURegs:$addr, 0)>;

def SLW : Load_SLW<0xD5, "slw", shr_load, ALURegs>;
def : Pat<(i32 (shr_load (add ALURegs:$rt, ALURegs:$rs))),                                 (SLW $rt, $rs, 0)>;
def : Pat<(i32 (shr_load ALURegs:$rt)),                                                    (SLW $rt, ZERO, 0)>;
def : Pat<(f32 (shr_load (add ALURegs:$rt, (add ALURegs:$rs, immSExt9:$imm)))),            (COPY_TO_REGCLASS (SLW $rt, $rs, $imm), FloatRegs)>;
def : Pat<(f32 (shr_load (add ALURegs:$rt, ALURegs:$rs))),                                 (COPY_TO_REGCLASS (SLW $rt, $rs, 0), FloatRegs)>;
def : Pat<(f32 (shr_load ALURegs:$rt)),                                                    (COPY_TO_REGCLASS (SLW $rt, ZERO, 0), FloatRegs)>;

def SSW : Store_SSW<0xDD, "ssw", shr_store, ALURegs>;
def : Pat<(shr_store ALURegs:$rd, (add ALURegs:$rt, ALURegs:$rs)),                         (SSW $rd, $rt, $rs, 0)>;
def : Pat<(shr_store ALURegs:$rd, ALURegs:$rt),                                            (SSW $rd, $rt, ZERO, 0)>;
def : Pat<(shr_store FloatRegs:$rd, (add ALURegs:$rt, (add ALURegs:$rs, immSExt9:$imm))),  (SSW (i32 (COPY_TO_REGCLASS $rd, ALURegs)), $rt, $rs, $imm)>;
def : Pat<(shr_store FloatRegs:$rd, (add ALURegs:$rt, ALURegs:$rs)),                       (SSW (i32 (COPY_TO_REGCLASS $rd, ALURegs)), $rt, $rs, 0)>;
def : Pat<(shr_store FloatRegs:$rd, ALURegs:$rt),                                          (SSW (i32 (COPY_TO_REGCLASS $rd, ALURegs)), $rt, ZERO, 0)>;

def SUB     : ArithLogicR<0x12, "sub", sub, IIAlu, ALURegs, 0>;
def MUL     : ArithLogicR<0x51, "mul", mul, IIAlu, ALURegs, 1>;
def MACC    : Macc<0x58, "macc", IIAlu, ALURegs, 0>;
def LUi     : LoadUpper<0x1D, "lui", LUi_Pseudo, ALURegs, uimm16>;
//def Li      : LoadLower<0x19, "li", Li_Pseudo, ALURegs, uimm16>; // TODO: make this
def Li      : LoadUpper<0x19, "li", Li_Pseudo, ALURegs, uimm16>;

def : Pat<(LUi_Pseudo (i32 tglobaladdr:$dst)), (LUi tglobaladdr:$dst)>;
def : Pat<(Li_Pseudo (i32 tglobaladdr:$dst)), (Li tglobaladdr:$dst)>; // TODO: make work

def SLL     : ArithLogicR<0x20, "sll", shl, IIAlu, ALURegs, 0>;
def SLLi    : ArithLogicI<0x21, "slli", shl, uimm14, uimm5>;
def SRL     : ArithLogicR<0x28, "srl", srl, IIAlu, ALURegs, 0>;
def SRLi    : ArithLogicI<0x29, "srli", srl, uimm14, immZExt14>;
def SRA     : ArithLogicR<0x2C, "sra", sra, IIAlu, ALURegs, 0>;
def SRAi    : ArithLogicI<0x2D, "srai", sra, uimm14, uimm5>;
def AND     : ArithLogicR<0x30, "and", and, IIAlu, ALURegs, 1>;
def ANDi    : ArithLogicI<0x31, "andi", and, uimm14, immZExt14>;
def OR      : ArithLogicR<0x32, "or", or, IIAlu, ALURegs, 1>;
def ORi     : ArithLogicI<0x33, "ori", or, uimm14, immZExt14>;
def XOR     : ArithLogicR<0x34, "xor", xor, IIAlu, ALURegs, 1>;
def XORi    : ArithLogicI<0x35, "xori", xor, uimm14, immZExt14>;
def NOR     : LogicNOR<0x38, "nor", IIAlu, ALURegs, 1>;

def LID     : RTMInstr<0xA0, "lid", LID_Pseudo, imm32, uimm4>;
def WGOFF   : RTMInstr<0xA1, "wgoff", WGOFF_Pseudo, imm32, uimm4>;

def AADD    : ArithLogicR<0x81, "aadd", AADD_Pseudo, IIAlu, ALURegs, 1>;
def AMAX    : ArithLogicR<0x82, "amax", AMAX_Pseudo, IIAlu, ALURegs, 1>;
def SIZE    : RTMInstr<0xA2, "size", SIZE_Pseudo, imm32, uimm4>;
def WGID    : RTMInstr<0xA3, "wgid", WGID_Pseudo, imm32, uimm4>;
def WGSIZE  : RTMInstr<0xA4, "wgsize", WGSIZE_Pseudo, imm32, uimm4>;
def LP      : RTMInstr<0xA8, "lp", LP_Pseudo, imm32, uimm4>;

//def RELU    : ArithOneARG<0xE0, "relu", RELU_Pseudo, IIAlu, FloatRegs, 0>;

def : Pat<(i32 immSExt14:$in), (ADDi ZERO, imm:$in)>;
def : Pat<(f32 fpimm0), (COPY_TO_REGCLASS (ADDi ZERO, 0), ALURegs)>;
def Li32    : FgpuPseudo<(outs ALURegs:$rd), (ins imm32:$src), "Li32",
              [(set ALURegs:$rd, (imm:$src))]>;

def NOP     : FgpuPseudo<(outs), (ins), "nop", []>, PseudoInstExpansion<(SLL ZERO, ZERO, 0)>;

let isReturn=1, isTerminator=1, isBarrier=1, hasCtrlDep=1, Uses = [LR] in
  def RetLR : FgpuPseudo<(outs), (ins), "", [(FgpuRET)]>;

def RET     : Control<0x92, "ret">;
def LSYNC    : Sync_<0x93, "lsync">;
def GSYNC    : Sync_<0x94, "gsync">;

def SET_SYNC    : SetSync_<0x95, "set_sync">;

def BEQ   : Branch<0x62, "beq", seteq, brtarget14, ALURegs>;
def BNE   : Branch<0x63, "bne", setne, brtarget14, ALURegs>;
let isBarrier = 1, isBranch = 1, isTerminator = 1 in {
  def B     : FgpuPseudo<(outs), (ins brtarget14:$imm14), "B", [(br bb:$imm14)]>;
}

def PDOM_BEQ   : Branch<0x65, "pdom_beq", seteq, brtarget7, ALURegs>;
def PDOM_BNE   : Branch<0x66, "pdom_bne", setne, brtarget7, ALURegs>;

def JSUB  : JmpLink<0x64, "jsub">;
//def : Pat<(br bb:$addr), (BEQ ZERO, ZERO, bb:$addr)>; // replaced by the pseudo-inst B above

//def JALR  : JmpLinkRegister<0x67, "jalr">;

//def : Pat<(shl (load_a (add ALURegs:$base, (shl ALURegs:$offset, imm_eq_1))), (shl (xor (add ALURegs:$base, (shl ALURegs:$offset, imm_eq_1)), imm_eq_31), imm_eq_4)),
//           (SLL (LH ALURegs:$offset, ALURegs:$base), (SLLi (XORi ALURegs:$offset, 31), 4))>;
// def : Pat<(shl (load_a (add ALURegs:$base, ALURegs:$offset)), (shl (xor (add ALURegs:$base, ALURegs:$offset), imm_eq_31), imm_eq_3)),
//             (SLL (LB ALURegs:$offset, ALURegs:$base), (SLLi (XORi ALURegs:$offset, 31), 3))>;

// def : Pat<(sra (load_a (add ALURegs:$base, ALURegs:$offset)),
//            (shl (add ALURegs:$base, ALURegs:$offset), imm_eq_3)),
//           (SRA (LB ALURegs:$offset, ALURegs:$base), (SLLi ALURegs:$offset, 3))>;
// def : Pat<(srl (load_a (add ALURegs:$base, ALURegs:$offset)),
//            (shl (add ALURegs:$base, ALURegs:$offset), imm_eq_3)),
//           (SRL (LB ALURegs:$offset, ALURegs:$base), (SLLi ALURegs:$offset, 3))>;

def FSLT    : CondR<0xC7, "fslt", setolt, FloatRegs, 0>;
def SLT     : CondR<0xB2, "slt", setlt, ALURegs, 0>;
def SLTi    : CondI<0xB3, "slti", setlt, simm14, immSExt14>;
def SLTu    : CondR<0xB6, "sltu", setult, ALURegs, 0>;
def SLTiu   : CondI<0xB7, "sltiu", setult, uimm14, immSExt14>;

// a == b
def : Pat<(seteq ALURegs:$lhs, ALURegs:$rhs), (SLTiu (XOR ALURegs:$lhs, ALURegs:$rhs), 1)>;
def : Pat<(setoeq FloatRegs:$lhs, FloatRegs:$rhs), (NOR (FSLT FloatRegs:$lhs, FloatRegs:$rhs), (FSLT FloatRegs:$rhs, FloatRegs:$lhs))>;
// a != b
def : Pat<(setne ALURegs:$lhs, ALURegs:$rhs), (SLTu ZERO, (XOR ALURegs:$lhs, ALURegs:$rhs))>;
def : Pat<(setone FloatRegs:$lhs, FloatRegs:$rhs), (OR (FSLT FloatRegs:$lhs, FloatRegs:$rhs), (FSLT FloatRegs:$rhs, FloatRegs:$lhs))>;
def : Pat<(setune FloatRegs:$lhs, FloatRegs:$rhs), (OR (FSLT FloatRegs:$lhs, FloatRegs:$rhs), (FSLT FloatRegs:$rhs, FloatRegs:$lhs))>;
// a <= b
def : Pat<(setle ALURegs:$lhs, ALURegs:$rhs), (XORi (SLT ALURegs:$rhs, ALURegs:$lhs), 1)>;
def : Pat<(setole FloatRegs:$lhs, FloatRegs:$rhs), (XORi (FSLT FloatRegs:$rhs, FloatRegs:$lhs), 1)>;
// a <= b is equal to (XORi (b < a), 1)
def : Pat<(setule ALURegs:$lhs, ALURegs:$rhs), (XORi (SLTu ALURegs:$rhs, ALURegs:$lhs), 1)>;
// a > b
def : Pat<(setgt ALURegs:$lhs, ALURegs:$rhs), (SLT ALURegs:$rhs, ALURegs:$lhs)>;
def : Pat<(setogt FloatRegs:$lhs, FloatRegs:$rhs), (FSLT FloatRegs:$rhs, FloatRegs:$lhs)>;
// a > b is equal to b < a is equal to setlt(b, a)
def : Pat<(setugt ALURegs:$lhs, ALURegs:$rhs), (SLTu ALURegs:$rhs, ALURegs:$lhs)>;
// a >= b
def : Pat<(setge ALURegs:$lhs, ALURegs:$rhs), (XORi (SLT ALURegs:$lhs, ALURegs:$rhs), 1)>;
def : Pat<(setoge FloatRegs:$lhs, FloatRegs:$rhs), (XORi (FSLT FloatRegs:$lhs, FloatRegs:$rhs), 1)>;
def : Pat<(setuge FloatRegs:$lhs, FloatRegs:$rhs), (XORi (FSLT FloatRegs:$lhs, FloatRegs:$rhs), 1)>;
def : Pat<(setge ALURegs:$lhs, immSExt14:$rhs), (XORi (SLTi ALURegs:$lhs, immSExt14:$rhs), 1)>;
// a >= b is equal to b <= a
def : Pat<(setuge ALURegs:$lhs, ALURegs:$rhs), (XORi (SLTu ALURegs:$lhs, ALURegs:$rhs), 1)>;
def : Pat<(setuge ALURegs:$lhs, immSExt14:$rhs), (XORi (SLTiu ALURegs:$lhs, immSExt14:$rhs), 1)>;

// rt is the condition
class CondMov<bits<8> op, string instr_asm> :
  FRRR<op, (outs GPROut:$rd), (ins GPROut:$rs, ALURegs:$rt, GPROut:$F),
     !strconcat(instr_asm, "\t$rd, $rs, $rt"), [], IIAlu> {
  let Constraints = "$F = $rd";
}

// Instantiation of instructions.
def MOVN : CondMov<0x40, "movn">;
def MOVZ : CondMov<0x48, "movz">;


// select patterns

def : Pat<(select (i32 (setge ALURegs:$lhs, ALURegs:$rhs)), ALURegs:$T, ALURegs:$F),
	  (MOVZ $T, (SLT $lhs, $rhs), $F)>;
def : Pat<(select (i32 (setuge ALURegs:$lhs, ALURegs:$rhs)), ALURegs:$T, ALURegs:$F),
	  (MOVZ $T, (SLTu $lhs, $rhs), $F)>;
def : Pat<(select (i32 (setge ALURegs:$lhs, immSExt14:$rhs)), ALURegs:$T, ALURegs:$F),
	  (MOVZ $T, (SLTi $lhs, immSExt14:$rhs), $F)>;
def : Pat<(select (i32 (setuge ALURegs:$lh, immSExt14:$rh)), ALURegs:$T, ALURegs:$F),
	  (MOVZ $T, (SLTiu $lh, immSExt14:$rh), $F)>;
def : Pat<(select (i32 (setle ALURegs:$lhs, ALURegs:$rhs)), ALURegs:$T, ALURegs:$F),
	  (MOVZ $T, (SLT $rhs, $lhs), $F)>;
def : Pat<(select (i32 (setule ALURegs:$lhs, ALURegs:$rhs)), ALURegs:$T, ALURegs:$F),
	  (MOVZ $T, (SLTu $rhs, $lhs), $F)>;
def : Pat<(select (i32 (seteq ALURegs:$lhs, ALURegs:$rhs)), ALURegs:$T, ALURegs:$F),
	  (MOVZ $T, (XOR $lhs, $rhs), $F)>;
def : Pat<(select (i32 (seteq ALURegs:$lhs, 0)), ALURegs:$T, ALURegs:$F),
	  (MOVZ $T, $lhs, $F)>;
def : Pat<(select (i32 (setne ALURegs:$lhs, ALURegs:$rhs)), ALURegs:$T, ALURegs:$F),
	  (MOVN $T, (XOR $lhs, $rhs), $F)>;
def : Pat<(select ALURegs:$cond, ALURegs:$T, ALURegs:$F),
	  (MOVN $T, $cond, $F)>;
def : Pat<(select (i32 (setne ALURegs:$lhs, 0)), ALURegs:$T, ALURegs:$F),
	  (MOVN $T, $lhs, $F)>;

def : Pat<(select (i32 (setge ALURegs:$lhs, ALURegs:$rhs)), FloatRegs:$T, FloatRegs:$F),
	  (MOVZ $T, (SLT $lhs, $rhs), $F)>;
def : Pat<(select (i32 (setuge ALURegs:$lhs, ALURegs:$rhs)), FloatRegs:$T, FloatRegs:$F),
	  (MOVZ $T, (SLTu $lhs, $rhs), $F)>;
def : Pat<(select (i32 (setge ALURegs:$lhs, immSExt14:$rhs)), FloatRegs:$T, FloatRegs:$F),
	  (MOVZ $T, (SLTi $lhs, immSExt14:$rhs), $F)>;
def : Pat<(select (i32 (setuge ALURegs:$lh, immSExt14:$rh)), FloatRegs:$T, FloatRegs:$F),
	  (MOVZ $T, (SLTiu $lh, immSExt14:$rh), $F)>;
def : Pat<(select (i32 (setle ALURegs:$lhs, ALURegs:$rhs)), FloatRegs:$T, FloatRegs:$F),
	  (MOVZ $T, (SLT $rhs, $lhs), $F)>;
def : Pat<(select (i32 (setule ALURegs:$lhs, ALURegs:$rhs)), FloatRegs:$T, FloatRegs:$F),
	  (MOVZ $T, (SLTu $rhs, $lhs), $F)>;
def : Pat<(select (i32 (seteq ALURegs:$lhs, ALURegs:$rhs)), FloatRegs:$T, FloatRegs:$F),
	  (MOVZ $T, (XOR $lhs, $rhs), $F)>;
def : Pat<(select (i32 (seteq ALURegs:$lhs, 0)), FloatRegs:$T, FloatRegs:$F),
	  (MOVZ $T, $lhs, $F)>;
def : Pat<(select (i32 (setne ALURegs:$lhs, ALURegs:$rhs)), FloatRegs:$T, FloatRegs:$F),
	  (MOVN $T, (XOR $lhs, $rhs), $F)>;
def : Pat<(select ALURegs:$cond, FloatRegs:$T, FloatRegs:$F),
	  (MOVN $T, $cond, $F)>;
def : Pat<(select (i32 (setne ALURegs:$lhs, 0)), FloatRegs:$T, FloatRegs:$F),
	  (MOVN $T, $lhs, $F)>;


// brcond patterns

// brcond for slt instruction

def : Pat<(brcond (i32 (setne ALURegs:$lhs, 0)), bb:$dst),
              (BNE ALURegs:$lhs, ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (seteq ALURegs:$lhs, 0)), bb:$dst),
              (BEQ ALURegs:$lhs, ZERO, bb:$dst)>;

def : Pat<(brcond (i32 (seteq ALURegs:$lhs, ALURegs:$rhs)), bb:$dst),
              (BEQ ALURegs:$lhs, ALURegs:$rhs, bb:$dst)>;
def : Pat<(brcond (i32 (setueq ALURegs:$lhs, ALURegs:$rhs)), bb:$dst),
              (BEQ ALURegs:$lhs, ALURegs:$rhs, bb:$dst)>;
def : Pat<(brcond (i32 (setne ALURegs:$lhs, ALURegs:$rhs)), bb:$dst),
              (BNE ALURegs:$lhs, ALURegs:$rhs, bb:$dst)>;
def : Pat<(brcond (i32 (setune ALURegs:$lhs, ALURegs:$rhs)), bb:$dst),
              (BNE ALURegs:$lhs, ALURegs:$rhs, bb:$dst)>;
def : Pat<(brcond (i32 (setlt ALURegs:$lhs, ALURegs:$rhs)), bb:$dst),
              (BNE (SLT ALURegs:$lhs, ALURegs:$rhs), ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (setult ALURegs:$lhs, ALURegs:$rhs)), bb:$dst),
              (BNE (SLTu ALURegs:$lhs, ALURegs:$rhs), ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (setgt ALURegs:$lhs, ALURegs:$rhs)), bb:$dst),
              (BNE (SLT ALURegs:$rhs, ALURegs:$lhs), ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (setugt ALURegs:$lhs, ALURegs:$rhs)), bb:$dst),
              (BNE (SLTu ALURegs:$rhs, ALURegs:$lhs), ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (setle ALURegs:$lhs, ALURegs:$rhs)), bb:$dst),
              (BEQ (SLT ALURegs:$rhs, ALURegs:$lhs), ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (setule ALURegs:$lhs, ALURegs:$rhs)), bb:$dst),
              (BEQ (SLTu ALURegs:$rhs, ALURegs:$lhs), ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (setge ALURegs:$lhs, ALURegs:$rhs)), bb:$dst),
              (BEQ (SLT ALURegs:$lhs, ALURegs:$rhs), ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (setuge ALURegs:$lhs, ALURegs:$rhs)), bb:$dst),
              (BEQ (SLTu ALURegs:$lhs, ALURegs:$rhs), ZERO, bb:$dst)>;

def : Pat<(brcond ALURegs:$cond, bb:$dst),
              (BNE ALURegs:$cond, ZERO, bb:$dst)>;


def : Pat<(FgpuJmpLink (i32 globaladdr:$dst)),
          (JSUB tglobaladdr:$dst)>;
def : Pat<(FgpuJmpLink (i32 tglobaladdr:$dst)),
          (JSUB tglobaladdr:$dst)>;
def : Pat<(FgpuJmpLink (i32 texternalsym:$dst)),
          (JSUB texternalsym:$dst)>;

def bitcast_fpimm_to_i32 : SDNodeXForm<fpimm, [{
return CurDAG->getTargetConstant(
  N->getValueAPF().bitcastToAPInt().getZExtValue(), SDLoc(N), MVT::i32);
}]>;

def : Pat<( f32 fpimm:$in),
      (COPY_TO_REGCLASS (Li32 (bitcast_fpimm_to_i32 f32:$in)), ALURegs)>;
def : Pat<( f32 (load_a ALURegs:$in)), (COPY_TO_REGCLASS (LW ZERO,ALURegs:$in), FloatRegs)>;
def : Pat<( f32 (load_a  (add ALURegs:$rt, (shl ALURegs:$rs, imm_eq_2)))),
          (COPY_TO_REGCLASS (LW $rs,$rt), FloatRegs)>;
def : Pat<( store_a (f32 FloatRegs:$rd), (add ALURegs:$rt, (shl ALURegs:$rs, imm_eq_2))),
          (SW ( i32 (COPY_TO_REGCLASS $rd, ALURegs)), $rs, $rt)>;
def : Pat<( store_a (f32 FloatRegs:$rd), ALURegs:$in),
          (SW ( i32 (COPY_TO_REGCLASS $rd, ALURegs)), ZERO, $in)>;
def : Pat<( f32 (bitconvert (i32 ALURegs:$Xn))), (COPY_TO_REGCLASS ALURegs:$Xn, FloatRegs)>;
def : Pat<( i32 (bitconvert (f32 FloatRegs:$Xn))), (COPY_TO_REGCLASS FloatRegs:$Xn, ALURegs)>;
// def : Pat<(f32 (bitconvert (i32 ALURegs:$Xn))),
//           (COPY_TO_REGCLASS ALURegs:$Xn, FloatRegs)>;
