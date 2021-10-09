//===- FgpuISelLowering.cpp - Fgpu DAG Lowering Implementation ------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that Fgpu uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#include "FgpuISelLowering.h"
#include "MCTargetDesc/FgpuBaseInfo.h"
#include "MCTargetDesc/FgpuInstPrinter.h"
#include "MCTargetDesc/FgpuMCTargetDesc.h"
#include "FgpuCCState.h"
#include "FgpuInstrInfo.h"
#include "FgpuMachineFunction.h"
#include "FgpuRegisterInfo.h"
#include "FgpuSubtarget.h"
#include "FgpuTargetMachine.h"
#include "FgpuTargetObjectFile.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/FunctionLoweringInfo.h"
#include "llvm/CodeGen/ISDOpcodes.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RuntimeLibcalls.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/Type.h"
#include "llvm/IR/Value.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MachineValueType.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"
#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdint>
#include <deque>
#include <iterator>
#include <utility>
#include <vector>

using namespace llvm;

#define DEBUG_TYPE "fgpu-lower"

STATISTIC(NumTailCalls, "Number of tail calls");

static cl::opt<bool>
NoZeroDivCheck("mno-check-zero-division", cl::Hidden,
               cl::desc("FGPU: Don't trap on integer division by zero."),
               cl::init(false));

extern cl::opt<bool> EmitJalrReloc;

// If I is a shifted mask, set the size (Size) and the first bit of the
// mask (Pos), and return true.
// For example, if I is 0x003ff800, (Pos, Size) = (11, 11).
static bool isShiftedMask(uint64_t I, uint64_t &Pos, uint64_t &Size) {
  if (!isShiftedMask_64(I))
    return false;

  Size = countPopulation(I);
  Pos = countTrailingZeros(I);
  return true;
}

//TODO: we don't actually want these to be used

// The FGPU MSA ABI passes vector arguments in the integer register set.
// The number of integer registers used is dependant on the ABI used.
MVT FgpuTargetLowering::getRegisterTypeForCallingConv(LLVMContext &Context,
                                                      CallingConv::ID CC,
                                                      EVT VT) const {
  if (!VT.isVector())
    return getRegisterType(Context, VT);

  return MVT::i32;
}

unsigned FgpuTargetLowering::getNumRegistersForCallingConv(LLVMContext &Context,
                                                           CallingConv::ID CC,
                                                           EVT VT) const {
  if (VT.isVector())
    return divideCeil(VT.getSizeInBits(), 32);
  return FgpuTargetLowering::getNumRegisters(Context, VT);
}

unsigned FgpuTargetLowering::getVectorTypeBreakdownForCallingConv(
    LLVMContext &Context, CallingConv::ID CC, EVT VT, EVT &IntermediateVT,
    unsigned &NumIntermediates, MVT &RegisterVT) const {
  // Break down vector types to either 2 i64s or 4 i32s.
  RegisterVT = getRegisterTypeForCallingConv(Context, CC, VT);
  IntermediateVT = RegisterVT;
  NumIntermediates =
      VT.getFixedSizeInBits() < RegisterVT.getFixedSizeInBits()
          ? VT.getVectorNumElements()
          : divideCeil(VT.getSizeInBits(), RegisterVT.getSizeInBits());
  return NumIntermediates;
}

SDValue FgpuTargetLowering::getGlobalReg(SelectionDAG &DAG, EVT Ty) const {
  MachineFunction &MF = DAG.getMachineFunction();
  FgpuFunctionInfo *FI = MF.getInfo<FgpuFunctionInfo>();
  return DAG.getRegister(FI->getGlobalBaseReg(MF), Ty);
}

SDValue FgpuTargetLowering::getTargetNode(GlobalAddressSDNode *N, EVT Ty,
                                          SelectionDAG &DAG,
                                          unsigned Flag) const {
  return DAG.getTargetGlobalAddress(N->getGlobal(), SDLoc(N), Ty, 0, Flag);
}

SDValue FgpuTargetLowering::getTargetNode(ExternalSymbolSDNode *N, EVT Ty,
                                          SelectionDAG &DAG,
                                          unsigned Flag) const {
  return DAG.getTargetExternalSymbol(N->getSymbol(), Ty, Flag);
}

SDValue FgpuTargetLowering::getTargetNode(BlockAddressSDNode *N, EVT Ty,
                                          SelectionDAG &DAG,
                                          unsigned Flag) const {
  return DAG.getTargetBlockAddress(N->getBlockAddress(), Ty, 0, Flag);
}

SDValue FgpuTargetLowering::getTargetNode(JumpTableSDNode *N, EVT Ty,
                                          SelectionDAG &DAG,
                                          unsigned Flag) const {
  return DAG.getTargetJumpTable(N->getIndex(), Ty, Flag);
}

SDValue FgpuTargetLowering::getTargetNode(ConstantPoolSDNode *N, EVT Ty,
                                          SelectionDAG &DAG,
                                          unsigned Flag) const {
  return DAG.getTargetConstantPool(N->getConstVal(), Ty, N->getAlign(),
                                   N->getOffset(), Flag);
}

const char *FgpuTargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch ((FgpuISD::NodeType)Opcode) {
  case FgpuISD::FIRST_NUMBER:      break;
  case FgpuISD::JmpLink:           return "FgpuISD::JmpLink";
  case FgpuISD::TailCall:          return "FgpuISD::TailCall";
  case FgpuISD::GotHi:             return "FgpuISD::GotHi";
  case FgpuISD::GPRel:             return "FgpuISD::GPRel";
  case FgpuISD::ThreadPointer:     return "FgpuISD::ThreadPointer";
  case FgpuISD::RET:               return "FgpuISD::Ret";
//  case FgpuISD::ERet:              return "FgpuISD::ERet";
//  case FgpuISD::EH_RETURN:         return "FgpuISD::EH_RETURN";
  case FgpuISD::Wrapper:           return "FgpuISD::Wrapper";
  case FgpuISD::DynAlloc:          return "FgpuISD::DynAlloc";
  case FgpuISD::Sync:              return "FgpuISD::Sync";
  case FgpuISD::LUi:               return "FgpuISD::LUi";
  case FgpuISD::Li:                return "FgpuISD::Li";
  case FgpuISD::LP:                return "FgpuISD::LP";
  case FgpuISD::WGOFF:             return "FgpuISD::WGOFF";
  case FgpuISD::WGID:              return "FgpuISD::WGID";
  case FgpuISD::LID:               return "FgpuISD::LID";
  case FgpuISD::WGSIZE:            return "FgpuISD::WGSIZE";
  case FgpuISD::SIZE:              return "FgpuISD::SIZE";
  case FgpuISD::DOT:               return "FgpuISD::DOT";
  case FgpuISD::AADD:              return "FgpuISD::AADD";
  case FgpuISD::AMAX:              return "FgpuISD::AMAX";
  case FgpuISD::RELU:              return "FgpuISD::RELU";
  case FgpuISD::SLW:               return "FgpuISD::SLW";
  case FgpuISD::SLWC:              return "FgpuISD::SLWC";
  case FgpuISD::SSW:               return "FgpuISD::SSW";
  case FgpuISD::SSWC:              return "FgpuISD::SSWC";
  }
  return nullptr;
}

FgpuTargetLowering::FgpuTargetLowering(const FgpuTargetMachine &TM,
                                       const FgpuSubtarget &STI)
    : TargetLowering(TM), Subtarget(STI), ABI(TM.getABI()) {
  // Fgpu does not have i1 type, so use i32 for
  // setcc operations results (slt, sgt, ...).
  setBooleanContents(ZeroOrOneBooleanContent);
  setBooleanVectorContents(ZeroOrNegativeOneBooleanContent);
  // The cmp.cond.fmt instruction in FGPU32r6/FGPU64r6 uses 0 and -1 like MSA
  // does. Integer booleans still use 0 and 1.

  // Load extented operations for i1 types must be promoted
  for (MVT VT : MVT::integer_valuetypes()) {
    setLoadExtAction(ISD::EXTLOAD,  VT, MVT::i1,  Promote);
    setLoadExtAction(ISD::ZEXTLOAD, VT, MVT::i1,  Promote);
    setLoadExtAction(ISD::SEXTLOAD, VT, MVT::i1,  Promote);
  }

  // FGPU doesn't have extending float->double load/store.  Set LoadExtAction
  // for f32, f16
  for (MVT VT : MVT::fp_valuetypes()) {
    setLoadExtAction(ISD::EXTLOAD, VT, MVT::f32, Expand);
    setLoadExtAction(ISD::EXTLOAD, VT, MVT::f16, Expand);
  }

  // Set LoadExtAction for f16 vectors to Expand
  for (MVT VT : MVT::fp_fixedlen_vector_valuetypes()) {
    MVT F16VT = MVT::getVectorVT(MVT::f16, VT.getVectorNumElements());
    if (F16VT.isValid())
      setLoadExtAction(ISD::EXTLOAD, VT, F16VT, Expand);
  }

  setTruncStoreAction(MVT::f32, MVT::f16, Expand);
  setTruncStoreAction(MVT::f64, MVT::f16, Expand);

  setTruncStoreAction(MVT::f64, MVT::f32, Expand);

  // Used by legalize types to correctly generate the setcc result.
  // Without this, every float setcc comes with a AND/OR with the result,
  // we don't want this, since the fpcmp result goes to a flag register,
  // which is used implicitly by brcond and select operations.
  AddPromotedToType(ISD::SETCC, MVT::i1, MVT::i32);

  // Fgpu Custom Operations
  setOperationAction(ISD::BR_JT,              MVT::Other, Expand);
  setOperationAction(ISD::GlobalAddress,      MVT::i32,   Custom);
  setOperationAction(ISD::BlockAddress,       MVT::i32,   Custom);
  setOperationAction(ISD::GlobalTLSAddress,   MVT::i32,   Custom);
  setOperationAction(ISD::JumpTable,          MVT::i32,   Custom);
  setOperationAction(ISD::ConstantPool,       MVT::i32,   Custom);
  setOperationAction(ISD::SELECT,             MVT::f32,   Custom);
  setOperationAction(ISD::SELECT,             MVT::f64,   Custom);
  setOperationAction(ISD::SELECT,             MVT::i32,   Custom);
  setOperationAction(ISD::SETCC,              MVT::f32,   Custom);
  setOperationAction(ISD::SETCC,              MVT::f64,   Custom);
  setOperationAction(ISD::BRCOND,             MVT::Other, Custom);
  setOperationAction(ISD::FCOPYSIGN,          MVT::f32,   Custom);
  setOperationAction(ISD::FCOPYSIGN,          MVT::f64,   Custom);
  //setOperationAction(ISD::FP_TO_SINT,         MVT::i32,   Custom);

    setOperationAction(ISD::FABS, MVT::f32, Custom);
    setOperationAction(ISD::FABS, MVT::f64, Custom);

    setOperationAction(ISD::SHL_PARTS,          MVT::i32,   Custom);
    setOperationAction(ISD::SRA_PARTS,          MVT::i32,   Custom);
    setOperationAction(ISD::SRL_PARTS,          MVT::i32,   Custom);

  setOperationAction(ISD::EH_DWARF_CFA,         MVT::i32,   Custom);

  setOperationAction(ISD::SDIV, MVT::i32, Expand);
  setOperationAction(ISD::SREM, MVT::i32, Expand);
  setOperationAction(ISD::UDIV, MVT::i32, Expand);
  setOperationAction(ISD::UREM, MVT::i32, Expand);
  setOperationAction(ISD::SDIV, MVT::i64, Expand);
  setOperationAction(ISD::SREM, MVT::i64, Expand);
  setOperationAction(ISD::UDIV, MVT::i64, Expand);
  setOperationAction(ISD::UREM, MVT::i64, Expand);

  // Operations not directly supported by Fgpu.
  setOperationAction(ISD::BR_CC,             MVT::f32,   Expand);
  setOperationAction(ISD::BR_CC,             MVT::i32,   Expand);
  setOperationAction(ISD::BR_CC,             MVT::i64,   Expand);
  setOperationAction(ISD::SELECT_CC,         MVT::i32,   Expand);
  setOperationAction(ISD::SELECT_CC,         MVT::i64,   Expand);
  setOperationAction(ISD::SELECT_CC,         MVT::f32,   Expand);
  setOperationAction(ISD::UINT_TO_FP,        MVT::i32,   Expand);
  setOperationAction(ISD::UINT_TO_FP,        MVT::i64,   Expand);
  setOperationAction(ISD::FP_TO_UINT,        MVT::i32,   Expand);
  setOperationAction(ISD::FP_TO_UINT,        MVT::i64,   Expand);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1,    Expand);
    setOperationAction(ISD::CTPOP,           MVT::i32,   Expand);
    setOperationAction(ISD::CTPOP,           MVT::i64,   Expand);
  setOperationAction(ISD::CTTZ,              MVT::i32,   Expand);
  setOperationAction(ISD::CTTZ,              MVT::i64,   Expand);
  setOperationAction(ISD::ROTL,              MVT::i32,   Expand);
  setOperationAction(ISD::ROTL,              MVT::i64,   Expand);
  setOperationAction(ISD::DYNAMIC_STACKALLOC, MVT::i32,  Expand);
  setOperationAction(ISD::DYNAMIC_STACKALLOC, MVT::i64,  Expand);
    setOperationAction(ISD::ROTR, MVT::i32,   Expand);
    setOperationAction(ISD::ROTR, MVT::i64,   Expand);
  setOperationAction(ISD::FSIN,              MVT::f32,   Expand);
  setOperationAction(ISD::FCOS,              MVT::f32,   Expand);
  setOperationAction(ISD::FSINCOS,           MVT::f32,   Expand);
  setOperationAction(ISD::FPOW,              MVT::f32,   Expand);
  setOperationAction(ISD::FLOG,              MVT::f32,   Expand);
  setOperationAction(ISD::FLOG2,             MVT::f32,   Expand);
  setOperationAction(ISD::FLOG10,            MVT::f32,   Expand);
  setOperationAction(ISD::FEXP,              MVT::f32,   Expand);

  setOperationAction(ISD::FP_TO_SINT,        MVT::i32,   Expand);
  setOperationAction(ISD::FP_TO_SINT,        MVT::f32,   Expand);
  setOperationAction(ISD::SINT_TO_FP,        MVT::i32,   Expand);
  setOperationAction(ISD::SINT_TO_FP,        MVT::f32,   Expand);
  //TODO: one of these is legal i think


  setOperationAction(ISD::FMA,               MVT::f32,   Legal); // called FFMA
  setOperationAction(ISD::FREM,              MVT::f32,   Expand);

  // Lower f16 conversion operations into library calls
  setOperationAction(ISD::FP16_TO_FP,        MVT::f32,   Expand);
  setOperationAction(ISD::FP_TO_FP16,        MVT::f32,   Expand);
  setOperationAction(ISD::FP16_TO_FP,        MVT::f64,   Expand);
  setOperationAction(ISD::FP_TO_FP16,        MVT::f64,   Expand);

  setOperationAction(ISD::EH_RETURN, MVT::Other, Custom);

  setOperationAction(ISD::VASTART,           MVT::Other, Custom);
  setOperationAction(ISD::VAARG,             MVT::Other, Custom);
  setOperationAction(ISD::VACOPY,            MVT::Other, Expand);
  setOperationAction(ISD::VAEND,             MVT::Other, Expand);

  // Use the default for now
  setOperationAction(ISD::STACKSAVE,         MVT::Other, Expand);
  setOperationAction(ISD::STACKRESTORE,      MVT::Other, Expand);

    setOperationAction(ISD::ATOMIC_LOAD,     MVT::i64,   Expand);
    setOperationAction(ISD::ATOMIC_STORE,    MVT::i64,   Expand);

    setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i8,  Expand);
    setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i16, Expand);

    setOperationAction(ISD::CTLZ, MVT::i32, Expand);
    setOperationAction(ISD::CTLZ, MVT::i64, Expand);

    setOperationAction(ISD::BSWAP, MVT::i32, Expand);
    setOperationAction(ISD::BSWAP, MVT::i64, Expand);

  setOperationAction(ISD::TRAP, MVT::Other, Legal);

  setTargetDAGCombine(ISD::SDIVREM);
  setTargetDAGCombine(ISD::UDIVREM);
  setTargetDAGCombine(ISD::SELECT);
  setTargetDAGCombine(ISD::AND);
  setTargetDAGCombine(ISD::OR);
  setTargetDAGCombine(ISD::ADD);
  setTargetDAGCombine(ISD::SUB);
  setTargetDAGCombine(ISD::AssertZext);
  setTargetDAGCombine(ISD::SHL);

    // These libcalls are not available in 32-bit.
    setLibcallName(RTLIB::SHL_I128, nullptr);
    setLibcallName(RTLIB::SRL_I128, nullptr);
    setLibcallName(RTLIB::SRA_I128, nullptr);
    setLibcallName(RTLIB::MULO_I64, nullptr);
    setLibcallName(RTLIB::MULO_I128, nullptr);

  setMinFunctionAlignment(Align(4));

  // The arguments on the stack are defined in terms of 4-byte slots on O32
  // and 8-byte slots on N32/N64.
  setMinStackArgumentAlignment(Align(4));

  setStackPointerRegisterToSaveRestore(Fgpu::SP);

  MaxStoresPerMemcpy = 16;
}

const FgpuTargetLowering *
FgpuTargetLowering::create(const FgpuTargetMachine &TM,
                           const FgpuSubtarget &STI) {

  return createFgpuSETargetLowering(TM, STI);
}

// Create a fast isel object.
FastISel *
FgpuTargetLowering::createFastISel(FunctionLoweringInfo &funcInfo,
                                  const TargetLibraryInfo *libInfo) const {
  return nullptr;
}

EVT FgpuTargetLowering::getSetCCResultType(const DataLayout &, LLVMContext &,
                                           EVT VT) const {
  if (!VT.isVector())
    return MVT::i32;
  return VT.changeVectorElementTypeToInteger();
}

static SDValue performDivRemCombine(SDNode *N, SelectionDAG &DAG,
                                    TargetLowering::DAGCombinerInfo &DCI,
                                    const FgpuSubtarget &Subtarget) {
  if (DCI.isBeforeLegalizeOps())
    return SDValue();
  assert(false && "not supported");
  return SDValue();
//  EVT Ty = N->getValueType(0);
//  unsigned LO = (Ty == MVT::i32) ? Fgpu::LO0 : Fgpu::LO0_64;
//  unsigned HI = (Ty == MVT::i32) ? Fgpu::HI0 : Fgpu::HI0_64;
//  unsigned Opc = N->getOpcode() == ISD::SDIVREM ? FgpuISD::DivRem16 :
//                                                  FgpuISD::DivRemU16;
//  SDLoc DL(N);
//
//  SDValue DivRem = DAG.getNode(Opc, DL, MVT::Glue,
//                               N->getOperand(0), N->getOperand(1));
//  SDValue InChain = DAG.getEntryNode();
//  SDValue InGlue = DivRem;
//
//  // insert MFLO
//  if (N->hasAnyUseOfValue(0)) {
//    SDValue CopyFromLo = DAG.getCopyFromReg(InChain, DL, LO, Ty,
//                                            InGlue);
//    DAG.ReplaceAllUsesOfValueWith(SDValue(N, 0), CopyFromLo);
//    InChain = CopyFromLo.getValue(1);
//    InGlue = CopyFromLo.getValue(2);
//  }
//
//  // insert MFHI
//  if (N->hasAnyUseOfValue(1)) {
//    SDValue CopyFromHi = DAG.getCopyFromReg(InChain, DL,
//                                            HI, Ty, InGlue);
//    DAG.ReplaceAllUsesOfValueWith(SDValue(N, 1), CopyFromHi);
//  }
//
//  return SDValue();
}
//
//static Fgpu::CondCode condCodeToFCC(ISD::CondCode CC) {
//  switch (CC) {
//  default: llvm_unreachable("Unknown fp condition code!");
//  case ISD::SETEQ:
//  case ISD::SETOEQ: return Fgpu::FCOND_OEQ;
//  case ISD::SETUNE: return Fgpu::FCOND_UNE;
//  case ISD::SETLT:
//  case ISD::SETOLT: return Fgpu::FCOND_OLT;
//  case ISD::SETGT:
//  case ISD::SETOGT: return Fgpu::FCOND_OGT;
//  case ISD::SETLE:
//  case ISD::SETOLE: return Fgpu::FCOND_OLE;
//  case ISD::SETGE:
//  case ISD::SETOGE: return Fgpu::FCOND_OGE;
//  case ISD::SETULT: return Fgpu::FCOND_ULT;
//  case ISD::SETULE: return Fgpu::FCOND_ULE;
//  case ISD::SETUGT: return Fgpu::FCOND_UGT;
//  case ISD::SETUGE: return Fgpu::FCOND_UGE;
//  case ISD::SETUO:  return Fgpu::FCOND_UN;
//  case ISD::SETO:   return Fgpu::FCOND_OR;
//  case ISD::SETNE:
//  case ISD::SETONE: return Fgpu::FCOND_ONE;
//  case ISD::SETUEQ: return Fgpu::FCOND_UEQ;
//  }
//}
//
///// This function returns true if the floating point conditional branches and
///// conditional moves which use condition code CC should be inverted.
//static bool invertFPCondCodeUser(Fgpu::CondCode CC) {
//  if (CC >= Fgpu::FCOND_F && CC <= Fgpu::FCOND_NGT)
//    return false;
//
//  assert((CC >= Fgpu::FCOND_T && CC <= Fgpu::FCOND_GT) &&
//         "Illegal Condition Code");
//
//  return true;
//}
//
//// Creates and returns an FPCmp node from a setcc node.
//// Returns Op if setcc is not a floating point comparison.
//static SDValue createFPCmp(SelectionDAG &DAG, const SDValue &Op) {
//  // must be a SETCC node
//  if (Op.getOpcode() != ISD::SETCC)
//    return Op;
//
//  SDValue LHS = Op.getOperand(0);
//
//  if (!LHS.getValueType().isFloatingPoint())
//    return Op;
//
//  SDValue RHS = Op.getOperand(1);
//  SDLoc DL(Op);
//
//  // Assume the 3rd operand is a CondCodeSDNode. Add code to check the type of
//  // node if necessary.
//  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(2))->get();
//
//  return DAG.getNode(FgpuISD::FPCmp, DL, MVT::Glue, LHS, RHS,
//                     DAG.getConstant(condCodeToFCC(CC), DL, MVT::i32));
//}
//
//// Creates and returns a CMovFPT/F node.
//static SDValue createCMovFP(SelectionDAG &DAG, SDValue Cond, SDValue True,
//                            SDValue False, const SDLoc &DL) {
//  ConstantSDNode *CC = cast<ConstantSDNode>(Cond.getOperand(2));
//  bool invert = invertFPCondCodeUser((Fgpu::CondCode)CC->getSExtValue());
//  SDValue FCC0 = DAG.getRegister(Fgpu::FCC0, MVT::i32);
//
//  return DAG.getNode((invert ? FgpuISD::CMovFP_F : FgpuISD::CMovFP_T), DL,
//                     True.getValueType(), True, FCC0, False, Cond);
//}

static SDValue performSELECTCombine(SDNode *N, SelectionDAG &DAG,
                                    TargetLowering::DAGCombinerInfo &DCI,
                                    const FgpuSubtarget &Subtarget) {
  if (DCI.isBeforeLegalizeOps())
    return SDValue();

  SDValue SetCC = N->getOperand(0);

  if ((SetCC.getOpcode() != ISD::SETCC) ||
      !SetCC.getOperand(0).getValueType().isInteger())
    return SDValue();

  SDValue False = N->getOperand(2);
  EVT FalseTy = False.getValueType();

  if (!FalseTy.isInteger())
    return SDValue();

  ConstantSDNode *FalseC = dyn_cast<ConstantSDNode>(False);

  // If the RHS (False) is 0, we swap the order of the operands
  // of ISD::SELECT (obviously also inverting the condition) so that we can
  // take advantage of conditional moves using the $0 register.
  // Example:
  //   return (a != 0) ? x : 0;
  //     load $reg, x
  //     movz $reg, $0, a
  if (!FalseC)
    return SDValue();

  const SDLoc DL(N);

  if (!FalseC->getZExtValue()) {
    ISD::CondCode CC = cast<CondCodeSDNode>(SetCC.getOperand(2))->get();
    SDValue True = N->getOperand(1);

    SetCC = DAG.getSetCC(DL, SetCC.getValueType(), SetCC.getOperand(0),
                         SetCC.getOperand(1),
                         ISD::getSetCCInverse(CC, SetCC.getValueType()));

    return DAG.getNode(ISD::SELECT, DL, FalseTy, SetCC, False, True);
  }

  // If both operands are integer constants there's a possibility that we
  // can do some interesting optimizations.
  SDValue True = N->getOperand(1);
  ConstantSDNode *TrueC = dyn_cast<ConstantSDNode>(True);

  if (!TrueC || !True.getValueType().isInteger())
    return SDValue();

  // We'll also ignore MVT::i64 operands as this optimizations proves
  // to be ineffective because of the required sign extensions as the result
  // of a SETCC operator is always MVT::i32 for non-vector types.
  if (True.getValueType() == MVT::i64)
    return SDValue();

  int64_t Diff = TrueC->getSExtValue() - FalseC->getSExtValue();

  // 1)  (a < x) ? y : y-1
  //  slti $reg1, a, x
  //  addiu $reg2, $reg1, y-1
  if (Diff == 1)
    return DAG.getNode(ISD::ADD, DL, SetCC.getValueType(), SetCC, False);

  // 2)  (a < x) ? y-1 : y
  //  slti $reg1, a, x
  //  xor $reg1, $reg1, 1
  //  addiu $reg2, $reg1, y-1
  if (Diff == -1) {
    ISD::CondCode CC = cast<CondCodeSDNode>(SetCC.getOperand(2))->get();
    SetCC = DAG.getSetCC(DL, SetCC.getValueType(), SetCC.getOperand(0),
                         SetCC.getOperand(1),
                         ISD::getSetCCInverse(CC, SetCC.getValueType()));
    return DAG.getNode(ISD::ADD, DL, SetCC.getValueType(), SetCC, True);
  }

  // Could not optimize.
  return SDValue();
}

SDValue  FgpuTargetLowering::PerformDAGCombine(SDNode *N, DAGCombinerInfo &DCI)
  const {
  SelectionDAG &DAG = DCI.DAG;
  unsigned Opc = N->getOpcode();

  switch (Opc) {
  default: break;
  case ISD::SELECT:
    return performSELECTCombine(N, DAG, DCI, Subtarget);
  }

  return SDValue();
}

bool FgpuTargetLowering::shouldFoldConstantShiftPairToMask(
    const SDNode *N, CombineLevel Level) const {
  if (N->getOperand(0).getValueType().isVector())
    return false;
  return true;
}

void
FgpuTargetLowering::ReplaceNodeResults(SDNode *N,
                                       SmallVectorImpl<SDValue> &Results,
                                       SelectionDAG &DAG) const {
  return LowerOperationWrapper(N, Results, DAG);
}

SDValue FgpuTargetLowering::
LowerOperation(SDValue Op, SelectionDAG &DAG) const
{
  switch (Op.getOpcode())
  {
  case ISD::BRCOND:             return lowerBRCOND(Op, DAG);
  case ISD::ConstantPool:       return lowerConstantPool(Op, DAG);
  case ISD::GlobalAddress:      return lowerGlobalAddress(Op, DAG);
  case ISD::BlockAddress:       return lowerBlockAddress(Op, DAG);
  case ISD::GlobalTLSAddress:   return lowerGlobalTLSAddress(Op, DAG);
  case ISD::JumpTable:          return lowerJumpTable(Op, DAG);
  case ISD::SELECT:             return lowerSELECT(Op, DAG);
  case ISD::SETCC:              return lowerSETCC(Op, DAG);
  case ISD::VASTART:            return lowerVASTART(Op, DAG);
  case ISD::VAARG:              return lowerVAARG(Op, DAG);
  case ISD::FCOPYSIGN:          return lowerFCOPYSIGN(Op, DAG);
  case ISD::FABS:               return lowerFABS(Op, DAG);
  case ISD::FRAMEADDR:          return lowerFRAMEADDR(Op, DAG);
  case ISD::RETURNADDR:         return lowerRETURNADDR(Op, DAG);
  case ISD::EH_RETURN:          return lowerEH_RETURN(Op, DAG);
  case ISD::ATOMIC_FENCE:       return lowerATOMIC_FENCE(Op, DAG);
  case ISD::SHL_PARTS:          return lowerShiftLeftParts(Op, DAG);
  case ISD::SRA_PARTS:          return lowerShiftRightParts(Op, DAG, true);
  case ISD::SRL_PARTS:          return lowerShiftRightParts(Op, DAG, false);
  case ISD::LOAD:               return lowerLOAD(Op, DAG);
  case ISD::STORE:              return lowerSTORE(Op, DAG);
  case ISD::EH_DWARF_CFA:       return lowerEH_DWARF_CFA(Op, DAG);
  case ISD::FP_TO_SINT:         return lowerFP_TO_SINT(Op, DAG);
  }
  return SDValue();
}

//===----------------------------------------------------------------------===//
//  Lower helper functions
//===----------------------------------------------------------------------===//

// addLiveIn - This helper function adds the specified physical register to the
// MachineFunction as a live in value.  It also creates a corresponding
// virtual register for it.
static unsigned
addLiveIn(MachineFunction &MF, unsigned PReg, const TargetRegisterClass *RC)
{
  Register VReg = MF.getRegInfo().createVirtualRegister(RC);
  MF.getRegInfo().addLiveIn(PReg, VReg);
  return VReg;
}

static MachineBasicBlock *insertDivByZeroTrap(MachineInstr &MI,
                                              MachineBasicBlock &MBB,
                                              const TargetInstrInfo &TII,
                                              bool Is64Bit) {
//  if (NoZeroDivCheck)
    return &MBB;

//  // Insert instruction "teq $divisor_reg, $zero, 7".
//  MachineBasicBlock::iterator I(MI);
//  MachineInstrBuilder MIB;
//  MachineOperand &Divisor = MI.getOperand(2);
//  MIB = BuildMI(MBB, std::next(I), MI.getDebugLoc(),
//                TII.get(Fgpu::TEQ))
//            .addReg(Divisor.getReg(), getKillRegState(Divisor.isKill()))
//            .addReg(Fgpu::ZERO)
//            .addImm(7);
//
//  // Use the 32-bit sub-register if this is a 64-bit division.
//  if (Is64Bit)
//    MIB->getOperand(0).setSubReg(Fgpu::sub_32);
//
//  // Clear Divisor's kill flag.
//  Divisor.setIsKill(false);
//
//  // We would normally delete the original instruction here but in this case
//  // we only needed to inject an additional instruction rather than replace it.
//
//  return &MBB;
}

MachineBasicBlock *
FgpuTargetLowering::EmitInstrWithCustomInserter(MachineInstr &MI,
                                                MachineBasicBlock *BB) const {
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected instr type to insert");
//  case Fgpu::PseudoSELECT_I:
//  case Fgpu::PseudoSELECT_I64:
//  case Fgpu::PseudoSELECT_S:
//  case Fgpu::PseudoSELECT_D32:
//  case Fgpu::PseudoSELECT_D64:
//    return emitPseudoSELECT(MI, BB, false, Fgpu::BNE);
//  case Fgpu::LDR_W:
//    return emitLDR_W(MI, BB);
//  case Fgpu::LDR_D:
//    return emitLDR_D(MI, BB);
//  case Fgpu::STR_W:
//    return emitSTR_W(MI, BB);
//  case Fgpu::STR_D:
//    return emitSTR_D(MI, BB);
  }
}

MachineBasicBlock *FgpuTargetLowering::emitSignExtendToI32InReg(
    MachineInstr &MI, MachineBasicBlock *BB, unsigned Size, unsigned DstReg,
    unsigned SrcReg) const {
  const TargetInstrInfo *TII = Subtarget.getInstrInfo();
  const DebugLoc &DL = MI.getDebugLoc();

  MachineFunction *MF = BB->getParent();
  MachineRegisterInfo &RegInfo = MF->getRegInfo();
  const TargetRegisterClass *RC = getRegClassFor(MVT::i32);
  Register ScrReg = RegInfo.createVirtualRegister(RC);

  assert(Size < 32);
  int64_t ShiftImm = 32 - (Size * 8);

  BuildMI(BB, DL, TII->get(Fgpu::SLL), ScrReg).addReg(SrcReg).addImm(ShiftImm);
  BuildMI(BB, DL, TII->get(Fgpu::SRA), DstReg).addReg(ScrReg).addImm(ShiftImm);

  return BB;
}

SDValue FgpuTargetLowering::lowerBRCOND(SDValue Op, SelectionDAG &DAG) const {
  // The first operand is the chain, the second is the condition, the third is
  // the block to branch to if the condition is true.
  SDValue Chain = Op.getOperand(0);
  SDValue Dest = Op.getOperand(2);
  SDLoc DL(Op);

  return Op;
//
//  SDValue CondRes = createFPCmp(DAG, Op.getOperand(1));
//
//  // Return if flag is not set by a floating point comparison.
//  if (CondRes.getOpcode() != FgpuISD::FPCmp)
//    return Op;
//
//  SDValue CCNode  = CondRes.getOperand(2);
//  Fgpu::CondCode CC =
//    (Fgpu::CondCode)cast<ConstantSDNode>(CCNode)->getZExtValue();
//  unsigned Opc = invertFPCondCodeUser(CC) ? Fgpu::BRANCH_F : Fgpu::BRANCH_T;
//  SDValue BrCode = DAG.getConstant(Opc, DL, MVT::i32);
//  SDValue FCC0 = DAG.getRegister(Fgpu::FCC0, MVT::i32);
//  return DAG.getNode(FgpuISD::FPBrcond, DL, Op.getValueType(), Chain, BrCode,
//                     FCC0, Dest, CondRes);
}

SDValue FgpuTargetLowering::
lowerSELECT(SDValue Op, SelectionDAG &DAG) const
{
    return Op;
//  SDValue Cond = createFPCmp(DAG, Op.getOperand(0));
//
//  // Return if flag is not set by a floating point comparison.
//  if (Cond.getOpcode() != FgpuISD::FPCmp)
//    return Op;
//
//  return createCMovFP(DAG, Cond, Op.getOperand(1), Op.getOperand(2),
//                      SDLoc(Op));
}

SDValue FgpuTargetLowering::lowerSETCC(SDValue Op, SelectionDAG &DAG) const {
  return Op;
//
//  SDValue Cond = createFPCmp(DAG, Op);
//
//  assert(Cond.getOpcode() == FgpuISD::FPCmp &&
//         "Floating point operand expected.");
//
//  SDLoc DL(Op);
//  SDValue True  = DAG.getConstant(1, DL, MVT::i32);
//  SDValue False = DAG.getConstant(0, DL, MVT::i32);
//
//  return createCMovFP(DAG, Cond, True, False, DL);
}

SDValue FgpuTargetLowering::lowerGlobalAddress(SDValue Op,
                                               SelectionDAG &DAG) const {
  EVT Ty = Op.getValueType();
  GlobalAddressSDNode *N = cast<GlobalAddressSDNode>(Op);
  const GlobalValue *GV = N->getGlobal();

  if (!isPositionIndependent()) {
    const FgpuTargetObjectFile *TLOF =
        static_cast<const FgpuTargetObjectFile *>(
            getTargetMachine().getObjFileLowering());
    const GlobalObject *GO = GV->getBaseObject();
    if (GO && TLOF->IsGlobalInSmallSection(GO, getTargetMachine()))
      // %gp_rel relocation
      return getAddrGPRel(N, SDLoc(N), Ty, DAG, false);

                                // %hi/%lo relocation
    return getAddrNonPIC(N, SDLoc(N), Ty, DAG);
  }

  // Every other architecture would use shouldAssumeDSOLocal in here, but
  // fgpu is special.
  // * In PIC code fgpu requires got loads even for local statics!
  // * To save on got entries, for local statics the got entry contains the
  //   page and an additional add instruction takes care of the low bits.
  // * It is legal to access a hidden symbol with a non hidden undefined,
  //   so one cannot guarantee that all access to a hidden symbol will know
  //   it is hidden.
  // * Fgpu linkers don't support creating a page and a full got entry for
  //   the same symbol.
  // * Given all that, we have to use a full got entry for hidden symbols :-(
  if (GV->hasLocalLinkage())
    return getAddrLocal(N, SDLoc(N), Ty, DAG, false);

  if (Subtarget.useXGOT())
    return getAddrGlobalLargeGOT(
        N, SDLoc(N), Ty, DAG, FgpuII::MO_GOT_HI16, FgpuII::MO_GOT_LO16,
        DAG.getEntryNode(),
        MachinePointerInfo::getGOT(DAG.getMachineFunction()));

  return getAddrGlobal(
      N, SDLoc(N), Ty, DAG,
      (false) ? FgpuII::MO_GOT_DISP : FgpuII::MO_GOT,
      DAG.getEntryNode(), MachinePointerInfo::getGOT(DAG.getMachineFunction()));
}

SDValue FgpuTargetLowering::lowerBlockAddress(SDValue Op,
                                              SelectionDAG &DAG) const {
  BlockAddressSDNode *N = cast<BlockAddressSDNode>(Op);
  EVT Ty = Op.getValueType();

  if (!isPositionIndependent())
    return getAddrNonPIC(N, SDLoc(N), Ty, DAG);

  return getAddrLocal(N, SDLoc(N), Ty, DAG, false);
}

SDValue FgpuTargetLowering::
lowerGlobalTLSAddress(SDValue Op, SelectionDAG &DAG) const
{
  // If the relocation model is PIC, use the General Dynamic TLS Model or
  // Local Dynamic TLS model, otherwise use the Initial Exec or
  // Local Exec TLS Model.

  GlobalAddressSDNode *GA = cast<GlobalAddressSDNode>(Op);
  if (DAG.getTarget().useEmulatedTLS())
    return LowerToTLSEmulatedModel(GA, DAG);

  SDLoc DL(GA);
  const GlobalValue *GV = GA->getGlobal();
  EVT PtrVT = getPointerTy(DAG.getDataLayout());

  TLSModel::Model model = getTargetMachine().getTLSModel(GV);

  if (model == TLSModel::GeneralDynamic || model == TLSModel::LocalDynamic) {
    // General Dynamic and Local Dynamic TLS Model.
    unsigned Flag = (model == TLSModel::LocalDynamic) ? FgpuII::MO_TLSLDM
                                                      : FgpuII::MO_TLSGD;

    SDValue TGA = DAG.getTargetGlobalAddress(GV, DL, PtrVT, 0, Flag);
    SDValue Argument = DAG.getNode(FgpuISD::Wrapper, DL, PtrVT,
                                   getGlobalReg(DAG, PtrVT), TGA);
    unsigned PtrSize = PtrVT.getSizeInBits();
    IntegerType *PtrTy = Type::getIntNTy(*DAG.getContext(), PtrSize);

    SDValue TlsGetAddr = DAG.getExternalSymbol("__tls_get_addr", PtrVT);

    ArgListTy Args;
    ArgListEntry Entry;
    Entry.Node = Argument;
    Entry.Ty = PtrTy;
    Args.push_back(Entry);

    TargetLowering::CallLoweringInfo CLI(DAG);
    CLI.setDebugLoc(DL)
        .setChain(DAG.getEntryNode())
        .setLibCallee(CallingConv::C, PtrTy, TlsGetAddr, std::move(Args));
    std::pair<SDValue, SDValue> CallResult = LowerCallTo(CLI);

    SDValue Ret = CallResult.first;

    if (model != TLSModel::LocalDynamic)
      return Ret;

    SDValue TGAHi = DAG.getTargetGlobalAddress(GV, DL, PtrVT, 0,
                                               FgpuII::MO_DTPREL_HI);
    //SDValue Hi = DAG.getNode(FgpuISD::TlsHi, DL, PtrVT, TGAHi);
    SDValue Hi = DAG.getNode(FgpuISD::LUi, DL, PtrVT, TGAHi);
    SDValue TGALo = DAG.getTargetGlobalAddress(GV, DL, PtrVT, 0,
                                               FgpuII::MO_DTPREL_LO);
    SDValue Lo = DAG.getNode(FgpuISD::Li, DL, PtrVT, TGALo);
    SDValue Add = DAG.getNode(ISD::ADD, DL, PtrVT, Hi, Ret);
    return DAG.getNode(ISD::ADD, DL, PtrVT, Add, Lo);
  }

  SDValue Offset;
  if (model == TLSModel::InitialExec) {
    // Initial Exec TLS Model
    SDValue TGA = DAG.getTargetGlobalAddress(GV, DL, PtrVT, 0,
                                             FgpuII::MO_GOTTPREL);
    TGA = DAG.getNode(FgpuISD::Wrapper, DL, PtrVT, getGlobalReg(DAG, PtrVT),
                      TGA);
    Offset =
        DAG.getLoad(PtrVT, DL, DAG.getEntryNode(), TGA, MachinePointerInfo());
  } else {
    // Local Exec TLS Model
    assert(model == TLSModel::LocalExec);
    SDValue TGAHi = DAG.getTargetGlobalAddress(GV, DL, PtrVT, 0,
                                               FgpuII::MO_TPREL_HI);
    SDValue TGALo = DAG.getTargetGlobalAddress(GV, DL, PtrVT, 0,
                                               FgpuII::MO_TPREL_LO);
    //SDValue Hi = DAG.getNode(FgpuISD::TlsHi, DL, PtrVT, TGAHi);
    SDValue Hi = DAG.getNode(FgpuISD::LUi, DL, PtrVT, TGAHi);
    SDValue Lo = DAG.getNode(FgpuISD::Li, DL, PtrVT, TGALo);
    Offset = DAG.getNode(ISD::ADD, DL, PtrVT, Hi, Lo);
  }

  SDValue ThreadPointer = DAG.getNode(FgpuISD::ThreadPointer, DL, PtrVT);
  return DAG.getNode(ISD::ADD, DL, PtrVT, ThreadPointer, Offset);
}

SDValue FgpuTargetLowering::
lowerJumpTable(SDValue Op, SelectionDAG &DAG) const
{
  JumpTableSDNode *N = cast<JumpTableSDNode>(Op);
  EVT Ty = Op.getValueType();

  if (!isPositionIndependent())
    return getAddrNonPIC(N, SDLoc(N), Ty, DAG);

  return getAddrLocal(N, SDLoc(N), Ty, DAG, false);
}

SDValue FgpuTargetLowering::
lowerConstantPool(SDValue Op, SelectionDAG &DAG) const
{
  ConstantPoolSDNode *N = cast<ConstantPoolSDNode>(Op);
  EVT Ty = Op.getValueType();

  if (!isPositionIndependent()) {
    const FgpuTargetObjectFile *TLOF =
        static_cast<const FgpuTargetObjectFile *>(
            getTargetMachine().getObjFileLowering());

    if (TLOF->IsConstantInSmallSection(DAG.getDataLayout(), N->getConstVal(),
                                       getTargetMachine()))
      // %gp_rel relocation
      return getAddrGPRel(N, SDLoc(N), Ty, DAG, false);

    return getAddrNonPIC(N, SDLoc(N), Ty, DAG);
  }

 return getAddrLocal(N, SDLoc(N), Ty, DAG, false);
}

SDValue FgpuTargetLowering::lowerVASTART(SDValue Op, SelectionDAG &DAG) const {
  MachineFunction &MF = DAG.getMachineFunction();
  FgpuFunctionInfo *FuncInfo = MF.getInfo<FgpuFunctionInfo>();

  SDLoc DL(Op);
  SDValue FI = DAG.getFrameIndex(FuncInfo->getVarArgsFrameIndex(),
                                 getPointerTy(MF.getDataLayout()));

  // vastart just stores the address of the VarArgsFrameIndex slot into the
  // memory location argument.
  const Value *SV = cast<SrcValueSDNode>(Op.getOperand(2))->getValue();
  return DAG.getStore(Op.getOperand(0), DL, FI, Op.getOperand(1),
                      MachinePointerInfo(SV));
}

SDValue FgpuTargetLowering::lowerVAARG(SDValue Op, SelectionDAG &DAG) const {
  SDNode *Node = Op.getNode();
  EVT VT = Node->getValueType(0);
  SDValue Chain = Node->getOperand(0);
  SDValue VAListPtr = Node->getOperand(1);
  const Align Align =
      llvm::MaybeAlign(Node->getConstantOperandVal(3)).valueOrOne();
  const Value *SV = cast<SrcValueSDNode>(Node->getOperand(2))->getValue();
  SDLoc DL(Node);
  unsigned ArgSlotSizeInBytes = 4;

  SDValue VAListLoad = DAG.getLoad(getPointerTy(DAG.getDataLayout()), DL, Chain,
                                   VAListPtr, MachinePointerInfo(SV));
  SDValue VAList = VAListLoad;

  // Re-align the pointer if necessary.
  // It should only ever be necessary for 64-bit types on O32 since the minimum
  // argument alignment is the same as the maximum type alignment for N32/N64.
  //
  // FIXME: We currently align too often. The code generator doesn't notice
  //        when the pointer is still aligned from the last va_arg (or pair of
  //        va_args for the i64 on O32 case).
  if (Align > getMinStackArgumentAlignment()) {
    VAList = DAG.getNode(
        ISD::ADD, DL, VAList.getValueType(), VAList,
        DAG.getConstant(Align.value() - 1, DL, VAList.getValueType()));

    VAList = DAG.getNode(
        ISD::AND, DL, VAList.getValueType(), VAList,
        DAG.getConstant(-(int64_t)Align.value(), DL, VAList.getValueType()));
  }

  // Increment the pointer, VAList, to the next vaarg.
  auto &TD = DAG.getDataLayout();
  unsigned ArgSizeInBytes =
      TD.getTypeAllocSize(VT.getTypeForEVT(*DAG.getContext()));
  SDValue Tmp3 =
      DAG.getNode(ISD::ADD, DL, VAList.getValueType(), VAList,
                  DAG.getConstant(alignTo(ArgSizeInBytes, ArgSlotSizeInBytes),
                                  DL, VAList.getValueType()));
  // Store the incremented VAList to the legalized pointer
  Chain = DAG.getStore(VAListLoad.getValue(1), DL, Tmp3, VAListPtr,
                       MachinePointerInfo(SV));

  // Load the actual argument out of the pointer VAList
  return DAG.getLoad(VT, DL, Chain, VAList, MachinePointerInfo());
}

static SDValue lowerFCOPYSIGN32(SDValue Op, SelectionDAG &DAG,
                                bool HasExtractInsert) {
  assert(false && "FP not supportrd");
  return Op;
//  EVT TyX = Op.getOperand(0).getValueType();
//  EVT TyY = Op.getOperand(1).getValueType();
//  SDLoc DL(Op);
//  SDValue Const1 = DAG.getConstant(1, DL, MVT::i32);
//  SDValue Const31 = DAG.getConstant(31, DL, MVT::i32);
//  SDValue Res;
//
//  // If operand is of type f64, extract the upper 32-bit. Otherwise, bitcast it
//  // to i32.
//  SDValue X = (TyX == MVT::f32) ?
//    DAG.getNode(ISD::BITCAST, DL, MVT::i32, Op.getOperand(0)) :
//    DAG.getNode(FgpuISD::ExtractElementF64, DL, MVT::i32, Op.getOperand(0),
//                Const1);
//  SDValue Y = (TyY == MVT::f32) ?
//    DAG.getNode(ISD::BITCAST, DL, MVT::i32, Op.getOperand(1)) :
//    DAG.getNode(FgpuISD::ExtractElementF64, DL, MVT::i32, Op.getOperand(1),
//                Const1);
//
//  if (HasExtractInsert) {
//    // ext  E, Y, 31, 1  ; extract bit31 of Y
//    // ins  X, E, 31, 1  ; insert extracted bit at bit31 of X
//    SDValue E = DAG.getNode(FgpuISD::Ext, DL, MVT::i32, Y, Const31, Const1);
//    Res = DAG.getNode(FgpuISD::Ins, DL, MVT::i32, E, Const31, Const1, X);
//  } else {
//    // sll SllX, X, 1
//    // srl SrlX, SllX, 1
//    // srl SrlY, Y, 31
//    // sll SllY, SrlX, 31
//    // or  Or, SrlX, SllY
//    SDValue SllX = DAG.getNode(ISD::SHL, DL, MVT::i32, X, Const1);
//    SDValue SrlX = DAG.getNode(ISD::SRL, DL, MVT::i32, SllX, Const1);
//    SDValue SrlY = DAG.getNode(ISD::SRL, DL, MVT::i32, Y, Const31);
//    SDValue SllY = DAG.getNode(ISD::SHL, DL, MVT::i32, SrlY, Const31);
//    Res = DAG.getNode(ISD::OR, DL, MVT::i32, SrlX, SllY);
//  }
//
//  if (TyX == MVT::f32)
//    return DAG.getNode(ISD::BITCAST, DL, Op.getOperand(0).getValueType(), Res);
//
//  SDValue LowX = DAG.getNode(FgpuISD::ExtractElementF64, DL, MVT::i32,
//                             Op.getOperand(0),
//                             DAG.getConstant(0, DL, MVT::i32));
//  return DAG.getNode(FgpuISD::BuildPairF64, DL, MVT::f64, LowX, Res);
}

SDValue
FgpuTargetLowering::lowerFCOPYSIGN(SDValue Op, SelectionDAG &DAG) const {
  return lowerFCOPYSIGN32(Op, DAG, false);
}

static SDValue lowerFABS32(SDValue Op, SelectionDAG &DAG,
                           bool HasExtractInsert) {
  assert(false && "FP not supportrd");
  return Op;
//  SDLoc DL(Op);
//  SDValue Res, Const1 = DAG.getConstant(1, DL, MVT::i32);
//
//  // If operand is of type f64, extract the upper 32-bit. Otherwise, bitcast it
//  // to i32.
//  SDValue X = (Op.getValueType() == MVT::f32)
//                  ? DAG.getNode(ISD::BITCAST, DL, MVT::i32, Op.getOperand(0))
//                  : DAG.getNode(FgpuISD::ExtractElementF64, DL, MVT::i32,
//                                Op.getOperand(0), Const1);
//
//  // Clear MSB.
//  if (HasExtractInsert)
//    Res = DAG.getNode(FgpuISD::Ins, DL, MVT::i32,
//                      DAG.getRegister(Fgpu::ZERO, MVT::i32),
//                      DAG.getConstant(31, DL, MVT::i32), Const1, X);
//  else {
//    // TODO: Provide DAG patterns which transform (and x, cst)
//    // back to a (shl (srl x (clz cst)) (clz cst)) sequence.
//    SDValue SllX = DAG.getNode(ISD::SHL, DL, MVT::i32, X, Const1);
//    Res = DAG.getNode(ISD::SRL, DL, MVT::i32, SllX, Const1);
//  }
//
//  if (Op.getValueType() == MVT::f32)
//    return DAG.getNode(ISD::BITCAST, DL, MVT::f32, Res);
//
//  // FIXME: For fgpu32r2, the sequence of (BuildPairF64 (ins (ExtractElementF64
//  // Op 1), $zero, 31 1) (ExtractElementF64 Op 0)) and the Op has one use, we
//  // should be able to drop the usage of mfc1/mtc1 and rewrite the register in
//  // place.
//  SDValue LowX =
//      DAG.getNode(FgpuISD::ExtractElementF64, DL, MVT::i32, Op.getOperand(0),
//                  DAG.getConstant(0, DL, MVT::i32));
//  return DAG.getNode(FgpuISD::BuildPairF64, DL, MVT::f64, LowX, Res);
}

SDValue FgpuTargetLowering::lowerFABS(SDValue Op, SelectionDAG &DAG) const {
  return lowerFABS32(Op, DAG, false);
}

SDValue FgpuTargetLowering::
lowerFRAMEADDR(SDValue Op, SelectionDAG &DAG) const {
  // check the depth
  if (cast<ConstantSDNode>(Op.getOperand(0))->getZExtValue() != 0) {
    DAG.getContext()->emitError(
        "return address can be determined only for current frame");
    return SDValue();
  }

  MachineFrameInfo &MFI = DAG.getMachineFunction().getFrameInfo();
  MFI.setFrameAddressIsTaken(true);
  EVT VT = Op.getValueType();
  SDLoc DL(Op);
  SDValue FrameAddr = DAG.getCopyFromReg(
      DAG.getEntryNode(), DL, Fgpu::R28, VT);
  return FrameAddr;
}

SDValue FgpuTargetLowering::lowerRETURNADDR(SDValue Op,
                                            SelectionDAG &DAG) const {
  if (verifyReturnAddressArgumentIsConstant(Op, DAG))
    return SDValue();

  // check the depth
  if (cast<ConstantSDNode>(Op.getOperand(0))->getZExtValue() != 0) {
    DAG.getContext()->emitError(
        "return address can be determined only for current frame");
    return SDValue();
  }

  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MVT VT = Op.getSimpleValueType();
  unsigned RA = Fgpu::LR;
  MFI.setReturnAddressIsTaken(true);

  // Return RA, which contains the return address. Mark it an implicit live-in.
  unsigned Reg = MF.addLiveIn(RA, getRegClassFor(VT));
  return DAG.getCopyFromReg(DAG.getEntryNode(), SDLoc(Op), Reg, VT);
}

// An EH_RETURN is the result of lowering llvm.eh.return which in turn is
// generated from __builtin_eh_return (offset, handler)
// The effect of this is to adjust the stack pointer by "offset"
// and then branch to "handler".
SDValue FgpuTargetLowering::lowerEH_RETURN(SDValue Op, SelectionDAG &DAG)
                                                                     const {
  MachineFunction &MF = DAG.getMachineFunction();
  FgpuFunctionInfo *FgpuFI = MF.getInfo<FgpuFunctionInfo>();

  FgpuFI->setCallsEhReturn();
  SDValue Chain     = Op.getOperand(0);
  SDValue Offset    = Op.getOperand(1);
  SDValue Handler   = Op.getOperand(2);
  SDLoc DL(Op);
  EVT Ty = MVT::i32;

  // Store stack offset in V1, store jump target in V0. Glue CopyToReg and
  // EH_RETURN nodes, so that instructions are emitted back-to-back.
  unsigned OffsetReg = Fgpu::R3;
  unsigned AddrReg = Fgpu::R2;
  Chain = DAG.getCopyToReg(Chain, DL, OffsetReg, Offset, SDValue());
  Chain = DAG.getCopyToReg(Chain, DL, AddrReg, Handler, Chain.getValue(1));
  return DAG.getNode(ISD::EH_RETURN, DL, MVT::Other, Chain,
                     DAG.getRegister(OffsetReg, Ty),
                     DAG.getRegister(AddrReg, getPointerTy(MF.getDataLayout())),
                     Chain.getValue(1));
}

SDValue FgpuTargetLowering::lowerATOMIC_FENCE(SDValue Op,
                                              SelectionDAG &DAG) const {
  // FIXME: Need pseudo-fence for 'singlethread' fences
  // FIXME: Set SType for weaker fences where supported/appropriate.
  unsigned SType = 0;
  SDLoc DL(Op);
  return DAG.getNode(FgpuISD::Sync, DL, MVT::Other, Op.getOperand(0),
                     DAG.getConstant(SType, DL, MVT::i32));
}

SDValue FgpuTargetLowering::lowerShiftLeftParts(SDValue Op,
                                                SelectionDAG &DAG) const {
  SDLoc DL(Op);
  MVT VT = MVT::i32;

  SDValue Lo = Op.getOperand(0), Hi = Op.getOperand(1);
  SDValue Shamt = Op.getOperand(2);
  // if shamt < (VT.bits):
  //  lo = (shl lo, shamt)
  //  hi = (or (shl hi, shamt) (srl (srl lo, 1), ~shamt))
  // else:
  //  lo = 0
  //  hi = (shl lo, shamt[4:0])
  SDValue Not = DAG.getNode(ISD::XOR, DL, MVT::i32, Shamt,
                            DAG.getConstant(-1, DL, MVT::i32));
  SDValue ShiftRight1Lo = DAG.getNode(ISD::SRL, DL, VT, Lo,
                                      DAG.getConstant(1, DL, VT));
  SDValue ShiftRightLo = DAG.getNode(ISD::SRL, DL, VT, ShiftRight1Lo, Not);
  SDValue ShiftLeftHi = DAG.getNode(ISD::SHL, DL, VT, Hi, Shamt);
  SDValue Or = DAG.getNode(ISD::OR, DL, VT, ShiftLeftHi, ShiftRightLo);
  SDValue ShiftLeftLo = DAG.getNode(ISD::SHL, DL, VT, Lo, Shamt);
  SDValue Cond = DAG.getNode(ISD::AND, DL, MVT::i32, Shamt,
                             DAG.getConstant(VT.getSizeInBits(), DL, MVT::i32));
  Lo = DAG.getNode(ISD::SELECT, DL, VT, Cond,
                   DAG.getConstant(0, DL, VT), ShiftLeftLo);
  Hi = DAG.getNode(ISD::SELECT, DL, VT, Cond, ShiftLeftLo, Or);

  SDValue Ops[2] = {Lo, Hi};
  return DAG.getMergeValues(Ops, DL);
}

SDValue FgpuTargetLowering::lowerShiftRightParts(SDValue Op, SelectionDAG &DAG,
                                                 bool IsSRA) const {
  SDLoc DL(Op);
  SDValue Lo = Op.getOperand(0), Hi = Op.getOperand(1);
  SDValue Shamt = Op.getOperand(2);
  MVT VT = MVT::i32;

  // if shamt < (VT.bits):
  //  lo = (or (shl (shl hi, 1), ~shamt) (srl lo, shamt))
  //  if isSRA:
  //    hi = (sra hi, shamt)
  //  else:
  //    hi = (srl hi, shamt)
  // else:
  //  if isSRA:
  //   lo = (sra hi, shamt[4:0])
  //   hi = (sra hi, 31)
  //  else:
  //   lo = (srl hi, shamt[4:0])
  //   hi = 0
  SDValue Not = DAG.getNode(ISD::XOR, DL, MVT::i32, Shamt,
                            DAG.getConstant(-1, DL, MVT::i32));
  SDValue ShiftLeft1Hi = DAG.getNode(ISD::SHL, DL, VT, Hi,
                                     DAG.getConstant(1, DL, VT));
  SDValue ShiftLeftHi = DAG.getNode(ISD::SHL, DL, VT, ShiftLeft1Hi, Not);
  SDValue ShiftRightLo = DAG.getNode(ISD::SRL, DL, VT, Lo, Shamt);
  SDValue Or = DAG.getNode(ISD::OR, DL, VT, ShiftLeftHi, ShiftRightLo);
  SDValue ShiftRightHi = DAG.getNode(IsSRA ? ISD::SRA : ISD::SRL,
                                     DL, VT, Hi, Shamt);
  SDValue Cond = DAG.getNode(ISD::AND, DL, MVT::i32, Shamt,
                             DAG.getConstant(VT.getSizeInBits(), DL, MVT::i32));
  SDValue Ext = DAG.getNode(ISD::SRA, DL, VT, Hi,
                            DAG.getConstant(VT.getSizeInBits() - 1, DL, VT));

  Lo = DAG.getNode(ISD::SELECT, DL, VT, Cond, ShiftRightHi, Or);
  Hi = DAG.getNode(ISD::SELECT, DL, VT, Cond,
                   IsSRA ? Ext : DAG.getConstant(0, DL, VT), ShiftRightHi);

  SDValue Ops[2] = {Lo, Hi};
  return DAG.getMergeValues(Ops, DL);
}

static SDValue createLoadLR(unsigned Opc, SelectionDAG &DAG, LoadSDNode *LD,
                            SDValue Chain, SDValue Src, unsigned Offset) {
  SDValue Ptr = LD->getBasePtr();
  EVT VT = LD->getValueType(0), MemVT = LD->getMemoryVT();
  EVT BasePtrVT = Ptr.getValueType();
  SDLoc DL(LD);
  SDVTList VTList = DAG.getVTList(VT, MVT::Other);

  if (Offset)
    Ptr = DAG.getNode(ISD::ADD, DL, BasePtrVT, Ptr,
                      DAG.getConstant(Offset, DL, BasePtrVT));

  SDValue Ops[] = { Chain, Ptr, Src };
  return DAG.getMemIntrinsicNode(Opc, DL, VTList, Ops, MemVT,
                                 LD->getMemOperand());
}

// Expand an unaligned 32 or 64-bit integer load node.
SDValue FgpuTargetLowering::lowerLOAD(SDValue Op, SelectionDAG &DAG) const {
  LoadSDNode *LD = cast<LoadSDNode>(Op);
  EVT MemVT = LD->getMemoryVT();

  if (Subtarget.systemSupportsUnalignedAccess())
    return Op;

  // Return if load is aligned or if MemVT is neither i32 nor i64.
  if ((LD->getAlignment() >= MemVT.getSizeInBits() / 8) ||
      ((MemVT != MVT::i32) && (MemVT != MVT::i64)))
    return SDValue();
  assert(false && "Misaligned");
//  bool IsLittle = true;
//  EVT VT = Op.getValueType();
//  ISD::LoadExtType ExtType = LD->getExtensionType();
//  SDValue Chain = LD->getChain(), Undef = DAG.getUNDEF(VT);
//
//  assert((VT == MVT::i32) || (VT == MVT::i64));
//
//  // Expand
//  //  (set dst, (i64 (load baseptr)))
//  // to
//  //  (set tmp, (ldl (add baseptr, 7), undef))
//  //  (set dst, (ldr baseptr, tmp))
//  if ((VT == MVT::i64) && (ExtType == ISD::NON_EXTLOAD)) {
//    SDValue LDL = createLoadLR(FgpuISD::LDL, DAG, LD, Chain, Undef,
//                               IsLittle ? 7 : 0);
//    return createLoadLR(FgpuISD::LDR, DAG, LD, LDL.getValue(1), LDL,
//                        IsLittle ? 0 : 7);
//  }
//
//  SDValue LWL = createLoadLR(FgpuISD::LWL, DAG, LD, Chain, Undef,
//                             IsLittle ? 3 : 0);
//  SDValue LWR = createLoadLR(FgpuISD::LWR, DAG, LD, LWL.getValue(1), LWL,
//                             IsLittle ? 0 : 3);
//
//  // Expand
//  //  (set dst, (i32 (load baseptr))) or
//  //  (set dst, (i64 (sextload baseptr))) or
//  //  (set dst, (i64 (extload baseptr)))
//  // to
//  //  (set tmp, (lwl (add baseptr, 3), undef))
//  //  (set dst, (lwr baseptr, tmp))
//  if ((VT == MVT::i32) || (ExtType == ISD::SEXTLOAD) ||
//      (ExtType == ISD::EXTLOAD))
//    return LWR;
//
//  assert((VT == MVT::i64) && (ExtType == ISD::ZEXTLOAD));
//
//  // Expand
//  //  (set dst, (i64 (zextload baseptr)))
//  // to
//  //  (set tmp0, (lwl (add baseptr, 3), undef))
//  //  (set tmp1, (lwr baseptr, tmp0))
//  //  (set tmp2, (shl tmp1, 32))
//  //  (set dst, (srl tmp2, 32))
//  SDLoc DL(LD);
//  SDValue Const32 = DAG.getConstant(32, DL, MVT::i32);
//  SDValue SLL = DAG.getNode(ISD::SHL, DL, MVT::i64, LWR, Const32);
//  SDValue SRL = DAG.getNode(ISD::SRL, DL, MVT::i64, SLL, Const32);
//  SDValue Ops[] = { SRL, LWR.getValue(1) };
//  return DAG.getMergeValues(Ops, DL);
}

static SDValue createStoreLR(unsigned Opc, SelectionDAG &DAG, StoreSDNode *SD,
                             SDValue Chain, unsigned Offset) {
  SDValue Ptr = SD->getBasePtr(), Value = SD->getValue();
  EVT MemVT = SD->getMemoryVT(), BasePtrVT = Ptr.getValueType();
  SDLoc DL(SD);
  SDVTList VTList = DAG.getVTList(MVT::Other);

  if (Offset)
    Ptr = DAG.getNode(ISD::ADD, DL, BasePtrVT, Ptr,
                      DAG.getConstant(Offset, DL, BasePtrVT));

  SDValue Ops[] = { Chain, Value, Ptr };
  return DAG.getMemIntrinsicNode(Opc, DL, VTList, Ops, MemVT,
                                 SD->getMemOperand());
}

// Expand an unaligned 32 or 64-bit integer store node.
static SDValue lowerUnalignedIntStore(StoreSDNode *SD, SelectionDAG &DAG,
                                      bool IsLittle) {
  SDValue Value = SD->getValue(), Chain = SD->getChain();
  EVT VT = Value.getValueType();
  assert(false && "Unalinged not supported");

//  // Expand
//  //  (store val, baseptr) or
//  //  (truncstore val, baseptr)
//  // to
//  //  (swl val, (add baseptr, 3))
//  //  (swr val, baseptr)
//  if ((VT == MVT::i32) || SD->isTruncatingStore()) {
//    SDValue SWL = createStoreLR(FgpuISD::SWL, DAG, SD, Chain,
//                                IsLittle ? 3 : 0);
//    return createStoreLR(FgpuISD::SWR, DAG, SD, SWL, IsLittle ? 0 : 3);
//  }
//
//  assert(VT == MVT::i64);
//
//  // Expand
//  //  (store val, baseptr)
//  // to
//  //  (sdl val, (add baseptr, 7))
//  //  (sdr val, baseptr)
//  SDValue SDL = createStoreLR(FgpuISD::SDL, DAG, SD, Chain, IsLittle ? 7 : 0);
//  return createStoreLR(FgpuISD::SDR, DAG, SD, SDL, IsLittle ? 0 : 7);
}

// Lower (store (fp_to_sint $fp) $ptr) to (store (TruncIntFP $fp), $ptr).
static SDValue lowerFP_TO_SINT_STORE(StoreSDNode *SD, SelectionDAG &DAG,
                                     bool SingleFloat) {
  SDValue Val = SD->getValue();

  if (Val.getOpcode() != ISD::FP_TO_SINT ||
      (Val.getValueSizeInBits() > 32 && SingleFloat))
    return SDValue();

  return SDValue(); // TODO: Idk what to do here

//  EVT FPTy = EVT::getFloatingPointVT(Val.getValueSizeInBits());
//  SDValue Tr = DAG.getNode(FgpuISD::TruncIntFP, SDLoc(Val), FPTy,
//                           Val.getOperand(0));
//  return DAG.getStore(SD->getChain(), SDLoc(SD), Tr, SD->getBasePtr(),
//                      SD->getPointerInfo(), SD->getAlignment(),
//                      SD->getMemOperand()->getFlags());
}

SDValue FgpuTargetLowering::lowerSTORE(SDValue Op, SelectionDAG &DAG) const {
  StoreSDNode *SD = cast<StoreSDNode>(Op);
  EVT MemVT = SD->getMemoryVT();

  // Lower unaligned integer stores.
  if (!Subtarget.systemSupportsUnalignedAccess() &&
      (SD->getAlignment() < MemVT.getSizeInBits() / 8) &&
      ((MemVT == MVT::i32) || (MemVT == MVT::i64)))
    return lowerUnalignedIntStore(SD, DAG, true);

  return lowerFP_TO_SINT_STORE(SD, DAG, true);
}

SDValue FgpuTargetLowering::lowerEH_DWARF_CFA(SDValue Op,
                                              SelectionDAG &DAG) const {

  // Return a fixed StackObject with offset 0 which points to the old stack
  // pointer.
  MachineFrameInfo &MFI = DAG.getMachineFunction().getFrameInfo();
  EVT ValTy = Op->getValueType(0);
  int FI = MFI.CreateFixedObject(Op.getValueSizeInBits() / 8, 0, false);
  return DAG.getFrameIndex(FI, ValTy);
}

SDValue FgpuTargetLowering::lowerFP_TO_SINT(SDValue Op,
                                            SelectionDAG &DAG) const {
  if (Op.getValueSizeInBits() > 32)
    return SDValue();

  assert(false && "Not supported fp");
  return SDValue();
//  EVT FPTy = EVT::getFloatingPointVT(Op.getValueSizeInBits());
//  SDValue Trunc = DAG.getNode(FgpuISD::TruncIntFP, SDLoc(Op), FPTy,
//                              Op.getOperand(0));
//  return DAG.getNode(ISD::BITCAST, SDLoc(Op), Op.getValueType(), Trunc);
}

//===----------------------------------------------------------------------===//
//                      Calling Convention Implementation
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// TODO: Implement a generic logic using tblgen that can support this.
// Fgpu O32 ABI rules:
// ---
// i32/f32 - Passed in A0, A1, A2, A3 and stack
// f32 - Only passed in f32 registers if no int reg has been used yet to hold
//       an argument. Otherwise, passed in A1, A2, A3 and stack.
// f64 - not supported
// vXiX - dunno
// vXf32 - no args i think
//
//  For vararg functions, all arguments are passed in A0, A1, A2, A3 and stack.
//===----------------------------------------------------------------------===//

static bool CC_Fgpu(unsigned ValNo, MVT ValVT, MVT LocVT,
                       CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
                       CCState &State) {
  const FgpuSubtarget &Subtarget = static_cast<const FgpuSubtarget &>(
      State.getMachineFunction().getSubtarget());

  static const MCPhysReg IntRegs[] = { Fgpu::R2, Fgpu::R3, Fgpu::R4, Fgpu::R5 };

  const FgpuCCState * FgpuState = static_cast<FgpuCCState *>(&State);

  // Do not process byval args here.
  if (ArgFlags.isByVal())
    return true;

  // Promote i8 and i16
  if (LocVT == MVT::i8 || LocVT == MVT::i16) {
    LocVT = MVT::i32;
    if (ArgFlags.isSExt())
      LocInfo = CCValAssign::SExt;
    else if (ArgFlags.isZExt())
      LocInfo = CCValAssign::ZExt;
    else
      LocInfo = CCValAssign::AExt;
  }

  unsigned Reg;

  Align OrigAlign = ArgFlags.getNonZeroOrigAlign();
  bool isI64 = (ValVT == MVT::i32 && OrigAlign == Align(8));
  bool isVectorFloat = FgpuState->WasOriginalArgVectorFloat(ValNo);

  if (ValVT == MVT::i32 || ValVT == MVT::f32) {
    Reg = State.AllocateReg(IntRegs);
    // If this is the first part of an i64 arg,
    // the allocated register must be either A0 or A2.
    if (isI64 && (Reg == Fgpu::R2 || Reg == Fgpu::R4))
      Reg = State.AllocateReg(IntRegs);
    LocVT = MVT::i32;
  } else if (ValVT == MVT::f64) {
    LocVT = MVT::i32;
    assert(false && "dbl not supported");
  } else
    llvm_unreachable("Cannot handle this ValVT.");

  if (!Reg) {
    unsigned Offset = State.AllocateStack(ValVT.getStoreSize(), OrigAlign);
    State.addLoc(CCValAssign::getMem(ValNo, ValVT, Offset, LocVT, LocInfo));
  } else
    State.addLoc(CCValAssign::getReg(ValNo, ValVT, Reg, LocVT, LocInfo));

  return false;
}

//static bool CC_Fgpu(unsigned ValNo, MVT ValVT, MVT LocVT,
//                       CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
//                       CCState &State) LLVM_ATTRIBUTE_UNUSED;

#include "FgpuGenCallingConv.inc"

 CCAssignFn *FgpuTargetLowering::CCAssignFnForCall() const{
   return CC_Fgpu; // TODO is this right
 }

 CCAssignFn *FgpuTargetLowering::CCAssignFnForReturn() const{
   return RetCC_Fgpu;
 }
//===----------------------------------------------------------------------===//
//                  Call Calling Convention Implementation
//===----------------------------------------------------------------------===//

SDValue FgpuTargetLowering::passArgOnStack(SDValue StackPtr, unsigned Offset,
                                           SDValue Chain, SDValue Arg,
                                           const SDLoc &DL, bool IsTailCall,
                                           SelectionDAG &DAG) const {
  if (!IsTailCall) {
    SDValue PtrOff =
        DAG.getNode(ISD::ADD, DL, getPointerTy(DAG.getDataLayout()), StackPtr,
                    DAG.getIntPtrConstant(Offset, DL));
    return DAG.getStore(Chain, DL, Arg, PtrOff, MachinePointerInfo());
  }

  MachineFrameInfo &MFI = DAG.getMachineFunction().getFrameInfo();
  int FI = MFI.CreateFixedObject(Arg.getValueSizeInBits() / 8, Offset, false);
  SDValue FIN = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
  return DAG.getStore(Chain, DL, Arg, FIN, MachinePointerInfo(), MaybeAlign(),
                      MachineMemOperand::MOVolatile);
}

void FgpuTargetLowering::
getOpndList(SmallVectorImpl<SDValue> &Ops,
            std::deque<std::pair<unsigned, SDValue>> &RegsToPass,
            bool IsPICCall, bool GlobalOrExternal, bool InternalLinkage,
            bool IsCallReloc, CallLoweringInfo &CLI, SDValue Callee,
            SDValue Chain) const {
  // Insert node "GP copy globalreg" before call to function.
  //
  // R_FGPU_CALL* operators (emitted when non-internal functions are called
  // in PIC mode) allow symbols to be resolved via lazy binding.
  // The lazy binding stub requires GP to point to the GOT.
  // Note that we don't need GP to point to the GOT for indirect calls
  // (when R_FGPU_CALL* is not used for the call) because Fgpu linker generates
  // lazy binding stub for a function only when R_FGPU_CALL* are the only relocs
  // used for the function (that is, Fgpu linker doesn't generate lazy binding
  // stub for a function whose address is taken in the program).
  if (IsPICCall && !InternalLinkage && IsCallReloc) {
    unsigned GPReg = Fgpu::R29;
    EVT Ty = MVT::i32;
    RegsToPass.push_back(std::make_pair(GPReg, getGlobalReg(CLI.DAG, Ty)));
  }

  // Build a sequence of copy-to-reg nodes chained together with token
  // chain and flag operands which copy the outgoing args into registers.
  // The InFlag in necessary since all emitted instructions must be
  // stuck together.
  SDValue InFlag;

  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
    Chain = CLI.DAG.getCopyToReg(Chain, CLI.DL, RegsToPass[i].first,
                                 RegsToPass[i].second, InFlag);
    InFlag = Chain.getValue(1);
  }

  // Add argument registers to the end of the list so that they are
  // known live into the call.
  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i)
    Ops.push_back(CLI.DAG.getRegister(RegsToPass[i].first,
                                      RegsToPass[i].second.getValueType()));

  // Add a register mask operand representing the call-preserved registers.
  const TargetRegisterInfo *TRI = Subtarget.getRegisterInfo();
  const uint32_t *Mask =
      TRI->getCallPreservedMask(CLI.DAG.getMachineFunction(), CLI.CallConv);
  assert(Mask && "Missing call preserved mask for calling convention");
  Ops.push_back(CLI.DAG.getRegisterMask(Mask));

  if (InFlag.getNode())
    Ops.push_back(InFlag);
}

void FgpuTargetLowering::AdjustInstrPostInstrSelection(MachineInstr &MI,
                                                       SDNode *Node) const {
  switch (MI.getOpcode()) {
    default:
      return;
//    case Fgpu::JALR:
//    case Fgpu::JALRPseudo:{
//      if (!EmitJalrReloc ||
//          !isPositionIndependent() ||
//          Node->getNumOperands() < 1 ||
//          Node->getOperand(0).getNumOperands() < 2) {
//        return;
//      }
//      // We are after the callee address, set by LowerCall().
//      // If added to MI, asm printer will emit .reloc R_FGPU_JALR for the
//      // symbol.
//      const SDValue TargetAddr = Node->getOperand(0).getOperand(1);
//      StringRef Sym;
//      if (const GlobalAddressSDNode *G =
//              dyn_cast_or_null<const GlobalAddressSDNode>(TargetAddr)) {
//        // We must not emit the R_FGPU_JALR relocation against data symbols
//        // since this will cause run-time crashes if the linker replaces the
//        // call instruction with a relative branch to the data symbol.
//        if (!isa<Function>(G->getGlobal())) {
//          LLVM_DEBUG(dbgs() << "Not adding R_FGPU_JALR against data symbol "
//                            << G->getGlobal()->getName() << "\n");
//          return;
//        }
//        Sym = G->getGlobal()->getName();
//      }
//      else if (const ExternalSymbolSDNode *ES =
//                   dyn_cast_or_null<const ExternalSymbolSDNode>(TargetAddr)) {
//        Sym = ES->getSymbol();
//      }
//
//      if (Sym.empty())
//        return;
//
//      MachineFunction *MF = MI.getParent()->getParent();
//      MCSymbol *S = MF->getContext().getOrCreateSymbol(Sym);
//      LLVM_DEBUG(dbgs() << "Adding R_FGPU_JALR against " << Sym << "\n");
//      MI.addOperand(MachineOperand::CreateMCSymbol(S, FgpuII::MO_JALR));
//    }
  }
}

/// LowerCall - functions arguments are copied from virtual regs to
/// (physical regs)/(stack frame), CALLSEQ_START and CALLSEQ_END are emitted.
SDValue
FgpuTargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
                              SmallVectorImpl<SDValue> &InVals) const {
  SelectionDAG &DAG                     = CLI.DAG;
  SDLoc DL                              = CLI.DL;
  SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
  SmallVectorImpl<SDValue> &OutVals     = CLI.OutVals;
  SmallVectorImpl<ISD::InputArg> &Ins   = CLI.Ins;
  SDValue Chain                         = CLI.Chain;
  SDValue Callee                        = CLI.Callee;
  bool &IsTailCall                      = CLI.IsTailCall;
  CallingConv::ID CallConv              = CLI.CallConv;
  bool IsVarArg                         = CLI.IsVarArg;

  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetFrameLowering *TFL = Subtarget.getFrameLowering();
  FgpuFunctionInfo *FuncInfo = MF.getInfo<FgpuFunctionInfo>();
  bool IsPIC = isPositionIndependent();

  // Analyze operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;
  FgpuCCState CCInfo(
      CallConv, IsVarArg, DAG.getMachineFunction(), ArgLocs, *DAG.getContext(),
      FgpuCCState::getSpecialCallingConvForCallee(Callee.getNode(), Subtarget));

  const ExternalSymbolSDNode *ES =
      dyn_cast_or_null<const ExternalSymbolSDNode>(Callee.getNode());

  // There is one case where CALLSEQ_START..CALLSEQ_END can be nested, which
  // is during the lowering of a call with a byval argument which produces
  // a call to memcpy. For the O32 case, this causes the caller to allocate
  // stack space for the reserved argument area for the callee, then recursively
  // again for the memcpy call. In the NEWABI case, this doesn't occur as those
  // ABIs mandate that the callee allocates the reserved argument area. We do
  // still produce nested CALLSEQ_START..CALLSEQ_END with zero space though.
  //
  // If the callee has a byval argument and memcpy is used, we are mandated
  // to already have produced a reserved argument area for the callee for O32.
  // Therefore, the reserved argument area can be reused for both calls.
  //
  // Other cases of calling memcpy cannot have a chain with a CALLSEQ_START
  // present, as we have yet to hook that node onto the chain.
  //
  // Hence, the CALLSEQ_START and CALLSEQ_END nodes can be eliminated in this
  // case. GCC does a similar trick, in that wherever possible, it calculates
  // the maximum out going argument area (including the reserved area), and
  // preallocates the stack space on entrance to the caller.
  //
  // FIXME: We should do the same for efficiency and space.

  // Note: The check on the calling convention below must match
  //       FgpuABIInfo::GetCalleeAllocdArgSizeInBytes().
  bool MemcpyInByVal = ES &&
                       StringRef(ES->getSymbol()) == StringRef("memcpy") &&
                       CallConv != CallingConv::Fast &&
                       Chain.getOpcode() == ISD::CALLSEQ_START;

  // Allocate the reserved argument area. It seems strange to do this from the
  // caller side but removing it breaks the frame size calculation.
  unsigned ReservedArgArea =
      MemcpyInByVal ? 0 : ABI.GetCalleeAllocdArgSizeInBytes(CallConv);
  CCInfo.AllocateStack(ReservedArgArea, Align(1));

  CCInfo.AnalyzeCallOperands(Outs, CC_Fgpu, CLI.getArgs(),
                             ES ? ES->getSymbol() : nullptr);

  // Get a count of how many bytes are to be pushed on the stack.
  unsigned NextStackOffset = CCInfo.getNextStackOffset();

  // Call site info for function parameters tracking.
  MachineFunction::CallSiteInfo CSInfo;

  // Check if it's really possible to do a tail call. Restrict it to functions
  // that are part of this compilation unit.
  bool InternalLinkage = false;
  if (IsTailCall) {
    IsTailCall = isEligibleForTailCallOptimization(
        CCInfo, NextStackOffset, *MF.getInfo<FgpuFunctionInfo>());
     if (GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee)) {
      InternalLinkage = G->getGlobal()->hasInternalLinkage();
      IsTailCall &= (InternalLinkage || G->getGlobal()->hasLocalLinkage() ||
                     G->getGlobal()->hasPrivateLinkage() ||
                     G->getGlobal()->hasHiddenVisibility() ||
                     G->getGlobal()->hasProtectedVisibility());
     }
  }
  if (!IsTailCall && CLI.CB && CLI.CB->isMustTailCall())
    report_fatal_error("failed to perform tail call elimination on a call "
                       "site marked musttail");

  if (IsTailCall)
    ++NumTailCalls;

  // Chain is the output chain of the last Load/Store or CopyToReg node.
  // ByValChain is the output chain of the last Memcpy node created for copying
  // byval arguments to the stack.
  unsigned StackAlignment = TFL->getStackAlignment();
  NextStackOffset = alignTo(NextStackOffset, StackAlignment);
  SDValue NextStackOffsetVal = DAG.getIntPtrConstant(NextStackOffset, DL, true);

  if (!(IsTailCall || MemcpyInByVal))
    Chain = DAG.getCALLSEQ_START(Chain, NextStackOffset, 0, DL);

  SDValue StackPtr =
      DAG.getCopyFromReg(Chain, DL, Fgpu::SP,
                         getPointerTy(DAG.getDataLayout()));

  std::deque<std::pair<unsigned, SDValue>> RegsToPass;
  SmallVector<SDValue, 8> MemOpChains;

  CCInfo.rewindByValRegsInfo();

  // Walk the register/memloc assignments, inserting copies/loads.
  for (unsigned i = 0, e = ArgLocs.size(), OutIdx = 0; i != e; ++i, ++OutIdx) {
    SDValue Arg = OutVals[OutIdx];
    CCValAssign &VA = ArgLocs[i];
    MVT ValVT = VA.getValVT(), LocVT = VA.getLocVT();
    ISD::ArgFlagsTy Flags = Outs[OutIdx].Flags;
    bool UseUpperBits = false;

    // ByVal Arg.
    if (Flags.isByVal()) {
      unsigned FirstByValReg, LastByValReg;
      unsigned ByValIdx = CCInfo.getInRegsParamsProcessed();
      CCInfo.getInRegsParamInfo(ByValIdx, FirstByValReg, LastByValReg);

      assert(Flags.getByValSize() &&
             "ByVal args of size 0 should have been ignored by front-end.");
      assert(ByValIdx < CCInfo.getInRegsParamsCount());
      assert(!IsTailCall &&
             "Do not tail-call optimize if there is a byval argument.");
      passByValArg(Chain, DL, RegsToPass, MemOpChains, StackPtr, MFI, DAG, Arg,
                   FirstByValReg, LastByValReg, Flags, true,
                   VA);
      CCInfo.nextInRegsParam();
      continue;
    }

    // Promote the value if needed.
    switch (VA.getLocInfo()) {
    default:
      llvm_unreachable("Unknown loc info!");
    case CCValAssign::Full:
      if (VA.isRegLoc()) {
        if ((ValVT == MVT::f32 && LocVT == MVT::i32) ||
            (ValVT == MVT::f64 && LocVT == MVT::i64) ||
            (ValVT == MVT::i64 && LocVT == MVT::f64))
          Arg = DAG.getNode(ISD::BITCAST, DL, LocVT, Arg);
        else if (ValVT == MVT::f64 && LocVT == MVT::i32) {
          assert(false && "dbl nnot supported");
//          SDValue Lo = DAG.getNode(FgpuISD::ExtractElementF64, DL, MVT::i32,
//                                   Arg, DAG.getConstant(0, DL, MVT::i32));
//          SDValue Hi = DAG.getNode(FgpuISD::ExtractElementF64, DL, MVT::i32,
//                                   Arg, DAG.getConstant(1, DL, MVT::i32));
//          if (!Subtarget.isLittle())
//            std::swap(Lo, Hi);
//
//          assert(VA.needsCustom());
//
//          Register LocRegLo = VA.getLocReg();
//          Register LocRegHigh = ArgLocs[++i].getLocReg();
//          RegsToPass.push_back(std::make_pair(LocRegLo, Lo));
//          RegsToPass.push_back(std::make_pair(LocRegHigh, Hi));
//          continue;
        }
      }
      break;
    case CCValAssign::BCvt:
      Arg = DAG.getNode(ISD::BITCAST, DL, LocVT, Arg);
      break;
    case CCValAssign::SExtUpper:
      UseUpperBits = true;
      LLVM_FALLTHROUGH;
    case CCValAssign::SExt:
      Arg = DAG.getNode(ISD::SIGN_EXTEND, DL, LocVT, Arg);
      break;
    case CCValAssign::ZExtUpper:
      UseUpperBits = true;
      LLVM_FALLTHROUGH;
    case CCValAssign::ZExt:
      Arg = DAG.getNode(ISD::ZERO_EXTEND, DL, LocVT, Arg);
      break;
    case CCValAssign::AExtUpper:
      UseUpperBits = true;
      LLVM_FALLTHROUGH;
    case CCValAssign::AExt:
      Arg = DAG.getNode(ISD::ANY_EXTEND, DL, LocVT, Arg);
      break;
    }

    if (UseUpperBits) {
      unsigned ValSizeInBits = Outs[OutIdx].ArgVT.getSizeInBits();
      unsigned LocSizeInBits = VA.getLocVT().getSizeInBits();
      Arg = DAG.getNode(
          ISD::SHL, DL, VA.getLocVT(), Arg,
          DAG.getConstant(LocSizeInBits - ValSizeInBits, DL, VA.getLocVT()));
    }

    // Arguments that can be passed on register must be kept at
    // RegsToPass vector
    if (VA.isRegLoc()) {
      RegsToPass.push_back(std::make_pair(VA.getLocReg(), Arg));

      // Collect CSInfo about which register passes which parameter.
      const TargetOptions &Options = DAG.getTarget().Options;
      if (Options.SupportsDebugEntryValues)
        CSInfo.emplace_back(VA.getLocReg(), i);

      continue;
    }

    // Register can't get to this point...
    assert(VA.isMemLoc());

    // emit ISD::STORE whichs stores the
    // parameter value to a stack Location
    MemOpChains.push_back(passArgOnStack(StackPtr, VA.getLocMemOffset(),
                                         Chain, Arg, DL, IsTailCall, DAG));
  }

  // Transform all store nodes into one single node because all store
  // nodes are independent of each other.
  if (!MemOpChains.empty())
    Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, MemOpChains);

  // If the callee is a GlobalAddress/ExternalSymbol node (quite common, every
  // direct call is) turn it into a TargetGlobalAddress/TargetExternalSymbol
  // node so that legalize doesn't hack it.

  EVT Ty = Callee.getValueType();
  bool GlobalOrExternal = false, IsCallReloc = false;

  // The long-calls feature is ignored in case of PIC.
  // While we do not support -mshared / -mno-shared properly,
  // ignore long-calls in case of -mabicalls too.
  if (!IsPIC) {
    // If the function should be called using "long call",
    // get its address into a register to prevent using
    // of the `jal` instruction for the direct call.
    if (auto *N = dyn_cast<ExternalSymbolSDNode>(Callee)) {
      if (Subtarget.useLongCalls())
        Callee = getAddrNonPIC(N, SDLoc(N), Ty, DAG);
    } else if (auto *N = dyn_cast<GlobalAddressSDNode>(Callee)) {
      bool UseLongCalls = Subtarget.useLongCalls();
      // If the function has long-call/far/near attribute
      // it overrides command line switch pased to the backend.
      if (auto *F = dyn_cast<Function>(N->getGlobal())) {
        if (F->hasFnAttribute("long-call"))
          UseLongCalls = true;
        else if (F->hasFnAttribute("short-call"))
          UseLongCalls = false;
      }
      if (UseLongCalls)
        Callee = getAddrNonPIC(N, SDLoc(N), Ty, DAG);
    }
  }

  if (GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee)) {
    if (IsPIC) {
      const GlobalValue *Val = G->getGlobal();
      InternalLinkage = Val->hasInternalLinkage();

      if (InternalLinkage)
        Callee = getAddrLocal(G, DL, Ty, DAG, false);
      else if (Subtarget.useXGOT()) {
        Callee = getAddrGlobalLargeGOT(G, DL, Ty, DAG, FgpuII::MO_CALL_HI16,
                                       FgpuII::MO_CALL_LO16, Chain,
                                       FuncInfo->callPtrInfo(MF, Val));
        IsCallReloc = true;
      } else {
        Callee = getAddrGlobal(G, DL, Ty, DAG, FgpuII::MO_GOT_CALL, Chain,
                               FuncInfo->callPtrInfo(MF, Val));
        IsCallReloc = true;
      }
    } else
      Callee = DAG.getTargetGlobalAddress(G->getGlobal(), DL,
                                          getPointerTy(DAG.getDataLayout()), 0,
                                          FgpuII::MO_NO_FLAG);
    GlobalOrExternal = true;
  }
  else if (ExternalSymbolSDNode *S = dyn_cast<ExternalSymbolSDNode>(Callee)) {
    const char *Sym = S->getSymbol();

    if (!IsPIC) // static
      Callee = DAG.getTargetExternalSymbol(
          Sym, getPointerTy(DAG.getDataLayout()), FgpuII::MO_NO_FLAG);
    else if (Subtarget.useXGOT()) {
      Callee = getAddrGlobalLargeGOT(S, DL, Ty, DAG, FgpuII::MO_CALL_HI16,
                                     FgpuII::MO_CALL_LO16, Chain,
                                     FuncInfo->callPtrInfo(MF, Sym));
      IsCallReloc = true;
    } else { // PIC
      Callee = getAddrGlobal(S, DL, Ty, DAG, FgpuII::MO_GOT_CALL, Chain,
                             FuncInfo->callPtrInfo(MF, Sym));
      IsCallReloc = true;
    }

    GlobalOrExternal = true;
  }

  SmallVector<SDValue, 8> Ops(1, Chain);
  SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);

  getOpndList(Ops, RegsToPass, IsPIC, GlobalOrExternal, InternalLinkage,
              IsCallReloc, CLI, Callee, Chain);

  if (IsTailCall) {
    MF.getFrameInfo().setHasTailCall();
    SDValue Ret = DAG.getNode(FgpuISD::TailCall, DL, MVT::Other, Ops);
    DAG.addCallSiteInfo(Ret.getNode(), std::move(CSInfo));
    return Ret;
  }

  Chain = DAG.getNode(FgpuISD::JmpLink, DL, NodeTys, Ops);
  SDValue InFlag = Chain.getValue(1);

  DAG.addCallSiteInfo(Chain.getNode(), std::move(CSInfo));

  // Create the CALLSEQ_END node in the case of where it is not a call to
  // memcpy.
  if (!(MemcpyInByVal)) {
    Chain = DAG.getCALLSEQ_END(Chain, NextStackOffsetVal,
                               DAG.getIntPtrConstant(0, DL, true), InFlag, DL);
    InFlag = Chain.getValue(1);
  }

  // Handle result values, copying them out of physregs into vregs that we
  // return.
  return LowerCallResult(Chain, InFlag, CallConv, IsVarArg, Ins, DL, DAG,
                         InVals, CLI);
}

/// LowerCallResult - Lower the result values of a call into the
/// appropriate copies out of appropriate physical registers.
SDValue FgpuTargetLowering::LowerCallResult(
    SDValue Chain, SDValue InFlag, CallingConv::ID CallConv, bool IsVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &DL,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals,
    TargetLowering::CallLoweringInfo &CLI) const {
  // Assign locations to each value returned by this call.
  SmallVector<CCValAssign, 16> RVLocs;
  FgpuCCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(), RVLocs,
                     *DAG.getContext());

  const ExternalSymbolSDNode *ES =
      dyn_cast_or_null<const ExternalSymbolSDNode>(CLI.Callee.getNode());
  CCInfo.AnalyzeCallResult(Ins, RetCC_Fgpu, CLI.RetTy,
                           ES ? ES->getSymbol() : nullptr);

  // Copy all of the result registers out of their specified physreg.
  for (unsigned i = 0; i != RVLocs.size(); ++i) {
    CCValAssign &VA = RVLocs[i];
    assert(VA.isRegLoc() && "Can only return in registers!");

    SDValue Val = DAG.getCopyFromReg(Chain, DL, RVLocs[i].getLocReg(),
                                     RVLocs[i].getLocVT(), InFlag);
    Chain = Val.getValue(1);
    InFlag = Val.getValue(2);

    if (VA.isUpperBitsInLoc()) {
      unsigned ValSizeInBits = Ins[i].ArgVT.getSizeInBits();
      unsigned LocSizeInBits = VA.getLocVT().getSizeInBits();
      unsigned Shift =
          VA.getLocInfo() == CCValAssign::ZExtUpper ? ISD::SRL : ISD::SRA;
      Val = DAG.getNode(
          Shift, DL, VA.getLocVT(), Val,
          DAG.getConstant(LocSizeInBits - ValSizeInBits, DL, VA.getLocVT()));
    }

    switch (VA.getLocInfo()) {
    default:
      llvm_unreachable("Unknown loc info!");
    case CCValAssign::Full:
      break;
    case CCValAssign::BCvt:
      Val = DAG.getNode(ISD::BITCAST, DL, VA.getValVT(), Val);
      break;
    case CCValAssign::AExt:
    case CCValAssign::AExtUpper:
      Val = DAG.getNode(ISD::TRUNCATE, DL, VA.getValVT(), Val);
      break;
    case CCValAssign::ZExt:
    case CCValAssign::ZExtUpper:
      Val = DAG.getNode(ISD::AssertZext, DL, VA.getLocVT(), Val,
                        DAG.getValueType(VA.getValVT()));
      Val = DAG.getNode(ISD::TRUNCATE, DL, VA.getValVT(), Val);
      break;
    case CCValAssign::SExt:
    case CCValAssign::SExtUpper:
      Val = DAG.getNode(ISD::AssertSext, DL, VA.getLocVT(), Val,
                        DAG.getValueType(VA.getValVT()));
      Val = DAG.getNode(ISD::TRUNCATE, DL, VA.getValVT(), Val);
      break;
    }

    InVals.push_back(Val);
  }

  return Chain;
}

static SDValue UnpackFromArgumentSlot(SDValue Val, const CCValAssign &VA,
                                      EVT ArgVT, const SDLoc &DL,
                                      SelectionDAG &DAG) {
  MVT LocVT = VA.getLocVT();
  EVT ValVT = VA.getValVT();

  // Shift into the upper bits if necessary.
  switch (VA.getLocInfo()) {
  default:
    break;
  case CCValAssign::AExtUpper:
  case CCValAssign::SExtUpper:
  case CCValAssign::ZExtUpper: {
    unsigned ValSizeInBits = ArgVT.getSizeInBits();
    unsigned LocSizeInBits = VA.getLocVT().getSizeInBits();
    unsigned Opcode =
        VA.getLocInfo() == CCValAssign::ZExtUpper ? ISD::SRL : ISD::SRA;
    Val = DAG.getNode(
        Opcode, DL, VA.getLocVT(), Val,
        DAG.getConstant(LocSizeInBits - ValSizeInBits, DL, VA.getLocVT()));
    break;
  }
  }

  // If this is an value smaller than the argument slot size (32-bit for O32,
  // 64-bit for N32/N64), it has been promoted in some way to the argument slot
  // size. Extract the value and insert any appropriate assertions regarding
  // sign/zero extension.
  switch (VA.getLocInfo()) {
  default:
    llvm_unreachable("Unknown loc info!");
  case CCValAssign::Full:
    break;
  case CCValAssign::AExtUpper:
  case CCValAssign::AExt:
    Val = DAG.getNode(ISD::TRUNCATE, DL, ValVT, Val);
    break;
  case CCValAssign::SExtUpper:
  case CCValAssign::SExt:
    Val = DAG.getNode(ISD::AssertSext, DL, LocVT, Val, DAG.getValueType(ValVT));
    Val = DAG.getNode(ISD::TRUNCATE, DL, ValVT, Val);
    break;
  case CCValAssign::ZExtUpper:
  case CCValAssign::ZExt:
    Val = DAG.getNode(ISD::AssertZext, DL, LocVT, Val, DAG.getValueType(ValVT));
    Val = DAG.getNode(ISD::TRUNCATE, DL, ValVT, Val);
    break;
  case CCValAssign::BCvt:
    Val = DAG.getNode(ISD::BITCAST, DL, ValVT, Val);
    break;
  }

  return Val;
}

//===----------------------------------------------------------------------===//
//             Formal Arguments Calling Convention Implementation
//===----------------------------------------------------------------------===//
/// LowerFormalArguments - transform physical registers into virtual registers
/// and generate load operations for arguments places on the stack.
SDValue FgpuTargetLowering::LowerFormalArguments(
    SDValue Chain, CallingConv::ID CallConv, bool IsVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &DL,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  FgpuFunctionInfo *FgpuFI = MF.getInfo<FgpuFunctionInfo>();

  FgpuFI->setVarArgsFrameIndex(0);

  // Used with vargs to acumulate store chains.
  std::vector<SDValue> OutChains;

  // Assign locations to all of the incoming arguments.
  SmallVector<CCValAssign, 16> ArgLocs;
  FgpuCCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(), ArgLocs,
                     *DAG.getContext());
  CCInfo.AllocateStack(ABI.GetCalleeAllocdArgSizeInBytes(CallConv), Align(1));
  const Function &Func = DAG.getMachineFunction().getFunction();
  Function::const_arg_iterator FuncArg = Func.arg_begin();

  if (Func.hasFnAttribute("interrupt") && !Func.arg_empty())
    report_fatal_error(
        "Functions with the interrupt attribute cannot have arguments!");

  CCInfo.AnalyzeFormalArguments(Ins, CC_Fgpu);
  FgpuFI->setFormalArgInfo(CCInfo.getNextStackOffset(),
                           CCInfo.getInRegsParamsCount() > 0);

  unsigned CurArgIdx = 0;
  CCInfo.rewindByValRegsInfo();

  for (unsigned i = 0, e = ArgLocs.size(), InsIdx = 0; i != e; ++i, ++InsIdx) {
    CCValAssign &VA = ArgLocs[i];
    if (Ins[InsIdx].isOrigArg()) {
      std::advance(FuncArg, Ins[InsIdx].getOrigArgIndex() - CurArgIdx);
      CurArgIdx = Ins[InsIdx].getOrigArgIndex();
    }
    EVT ValVT = VA.getValVT();
    ISD::ArgFlagsTy Flags = Ins[InsIdx].Flags;
    bool IsRegLoc = VA.isRegLoc();

    if (Flags.isByVal()) {
      assert(Ins[InsIdx].isOrigArg() && "Byval arguments cannot be implicit");
      unsigned FirstByValReg, LastByValReg;
      unsigned ByValIdx = CCInfo.getInRegsParamsProcessed();
      CCInfo.getInRegsParamInfo(ByValIdx, FirstByValReg, LastByValReg);

      assert(Flags.getByValSize() &&
             "ByVal args of size 0 should have been ignored by front-end.");
      assert(ByValIdx < CCInfo.getInRegsParamsCount());
      copyByValRegs(Chain, DL, OutChains, DAG, Flags, InVals, &*FuncArg,
                    FirstByValReg, LastByValReg, VA, CCInfo);
      CCInfo.nextInRegsParam();
      continue;
    }

    // Arguments stored on registers
    if (IsRegLoc) {
      MVT RegVT = VA.getLocVT();
      Register ArgReg = VA.getLocReg();
      const TargetRegisterClass *RC = getRegClassFor(RegVT);

      // Transform the arguments stored on
      // physical registers into virtual ones
      unsigned Reg = addLiveIn(DAG.getMachineFunction(), ArgReg, RC);
      SDValue ArgValue = DAG.getCopyFromReg(Chain, DL, Reg, RegVT);

      ArgValue =
          UnpackFromArgumentSlot(ArgValue, VA, Ins[InsIdx].ArgVT, DL, DAG);

      // Handle floating point arguments passed in integer registers and
      // long double arguments passed in floating point registers.
      if ((RegVT == MVT::i32 && ValVT == MVT::f32) ||
          (RegVT == MVT::i64 && ValVT == MVT::f64) ||
          (RegVT == MVT::f64 && ValVT == MVT::i64))
        ArgValue = DAG.getNode(ISD::BITCAST, DL, ValVT, ArgValue);
      else if (RegVT == MVT::i32 &&
               ValVT == MVT::f64) {
        assert(false && "dbl not supported");
        assert(VA.needsCustom() && "Expected custom argument for f64 split");
//        CCValAssign &NextVA = ArgLocs[++i];
//        unsigned Reg2 =
//            addLiveIn(DAG.getMachineFunction(), NextVA.getLocReg(), RC);
//        SDValue ArgValue2 = DAG.getCopyFromReg(Chain, DL, Reg2, RegVT);
//        ArgValue = DAG.getNode(FgpuISD::BuildPairF64, DL, MVT::f64,
//                               ArgValue, ArgValue2);
      }

      InVals.push_back(ArgValue);
    } else { // VA.isRegLoc()
      MVT LocVT = VA.getLocVT();

      assert(!VA.needsCustom() && "unexpected custom memory argument");

      // We ought to be able to use LocVT directly but O32 sets it to i32
      // when allocating floating point values to integer registers.
      // This shouldn't influence how we load the value into registers unless
      // we are targeting softfloat.
      if (VA.getValVT().isFloatingPoint())
        LocVT = VA.getValVT();

      // sanity check
      assert(VA.isMemLoc());

      // The stack pointer offset is relative to the caller stack frame.
      int FI = MFI.CreateFixedObject(LocVT.getSizeInBits() / 8,
                                     VA.getLocMemOffset(), true);

      // Create load nodes to retrieve arguments from the stack
      SDValue FIN = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
      SDValue ArgValue = DAG.getLoad(
          LocVT, DL, Chain, FIN,
          MachinePointerInfo::getFixedStack(DAG.getMachineFunction(), FI));
      OutChains.push_back(ArgValue.getValue(1));

      ArgValue =
          UnpackFromArgumentSlot(ArgValue, VA, Ins[InsIdx].ArgVT, DL, DAG);

      InVals.push_back(ArgValue);
    }
  }

  for (unsigned i = 0, e = ArgLocs.size(), InsIdx = 0; i != e; ++i, ++InsIdx) {

    if (ArgLocs[i].needsCustom()) {
      ++i;
      continue;
    }

    // The fgpu ABIs for returning structs by value requires that we copy
    // the sret argument into $v0 for the return. Save the argument into
    // a virtual register so that we can access it from the return points.
    if (Ins[InsIdx].Flags.isSRet()) {
      unsigned Reg = FgpuFI->getSRetReturnReg();
      if (!Reg) {
        Reg = MF.getRegInfo().createVirtualRegister(
            getRegClassFor(MVT::i32));
        FgpuFI->setSRetReturnReg(Reg);
      }
      SDValue Copy = DAG.getCopyToReg(DAG.getEntryNode(), DL, Reg, InVals[i]);
      Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, Copy, Chain);
      break;
    }
  }

  if (IsVarArg)
    writeVarArgRegs(OutChains, Chain, DL, DAG, CCInfo);

  // All stores are grouped in one node to allow the matching between
  // the size of Ins and InVals. This only happens when on varg functions
  if (!OutChains.empty()) {
    OutChains.push_back(Chain);
    Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, OutChains);
  }

  return Chain;
}

//===----------------------------------------------------------------------===//
//               Return Value Calling Convention Implementation
//===----------------------------------------------------------------------===//

bool
FgpuTargetLowering::CanLowerReturn(CallingConv::ID CallConv,
                                   MachineFunction &MF, bool IsVarArg,
                                   const SmallVectorImpl<ISD::OutputArg> &Outs,
                                   LLVMContext &Context) const {
  SmallVector<CCValAssign, 16> RVLocs;
  FgpuCCState CCInfo(CallConv, IsVarArg, MF, RVLocs, Context);
  return CCInfo.CheckReturn(Outs, RetCC_Fgpu);
}

bool FgpuTargetLowering::shouldSignExtendTypeInLibCall(EVT Type,
                                                       bool IsSigned) const {
  return IsSigned;
}

SDValue
FgpuTargetLowering::LowerInterruptReturn(SmallVectorImpl<SDValue> &RetOps,
                                         const SDLoc &DL,
                                         SelectionDAG &DAG) const {
  assert(false && "I don't knnow why thsi is a thing");
  MachineFunction &MF = DAG.getMachineFunction();
  FgpuFunctionInfo *FgpuFI = MF.getInfo<FgpuFunctionInfo>();

  FgpuFI->setISR();

  return DAG.getNode(FgpuISD::RET, DL, MVT::Other, RetOps);
}

SDValue
FgpuTargetLowering::LowerReturn(SDValue Chain, CallingConv::ID CallConv,
                                bool IsVarArg,
                                const SmallVectorImpl<ISD::OutputArg> &Outs,
                                const SmallVectorImpl<SDValue> &OutVals,
                                const SDLoc &DL, SelectionDAG &DAG) const {
  // CCValAssign - represent the assignment of
  // the return value to a location
  SmallVector<CCValAssign, 16> RVLocs;
  MachineFunction &MF = DAG.getMachineFunction();

  // CCState - Info about the registers and stack slot.
  FgpuCCState CCInfo(CallConv, IsVarArg, MF, RVLocs, *DAG.getContext());

  // Analyze return values.
  CCInfo.AnalyzeReturn(Outs, RetCC_Fgpu);

  SDValue Flag;
  SmallVector<SDValue, 4> RetOps(1, Chain);

  // Copy the result values into the output registers.
  for (unsigned i = 0; i != RVLocs.size(); ++i) {
    SDValue Val = OutVals[i];
    CCValAssign &VA = RVLocs[i];
    assert(VA.isRegLoc() && "Can only return in registers!");
    bool UseUpperBits = false;

    switch (VA.getLocInfo()) {
    default:
      llvm_unreachable("Unknown loc info!");
    case CCValAssign::Full:
      break;
    case CCValAssign::BCvt:
      Val = DAG.getNode(ISD::BITCAST, DL, VA.getLocVT(), Val);
      break;
    case CCValAssign::AExtUpper:
      UseUpperBits = true;
      LLVM_FALLTHROUGH;
    case CCValAssign::AExt:
      Val = DAG.getNode(ISD::ANY_EXTEND, DL, VA.getLocVT(), Val);
      break;
    case CCValAssign::ZExtUpper:
      UseUpperBits = true;
      LLVM_FALLTHROUGH;
    case CCValAssign::ZExt:
      Val = DAG.getNode(ISD::ZERO_EXTEND, DL, VA.getLocVT(), Val);
      break;
    case CCValAssign::SExtUpper:
      UseUpperBits = true;
      LLVM_FALLTHROUGH;
    case CCValAssign::SExt:
      Val = DAG.getNode(ISD::SIGN_EXTEND, DL, VA.getLocVT(), Val);
      break;
    }

    if (UseUpperBits) {
      unsigned ValSizeInBits = Outs[i].ArgVT.getSizeInBits();
      unsigned LocSizeInBits = VA.getLocVT().getSizeInBits();
      Val = DAG.getNode(
          ISD::SHL, DL, VA.getLocVT(), Val,
          DAG.getConstant(LocSizeInBits - ValSizeInBits, DL, VA.getLocVT()));
    }

    Chain = DAG.getCopyToReg(Chain, DL, VA.getLocReg(), Val, Flag);

    // Guarantee that all emitted copies are stuck together with flags.
    Flag = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
  }

  // The fgpu ABIs for returning structs by value requires that we copy
  // the sret argument into $v0 for the return. We saved the argument into
  // a virtual register in the entry block, so now we copy the value out
  // and into $v0.
  if (MF.getFunction().hasStructRetAttr()) {
    FgpuFunctionInfo *FgpuFI = MF.getInfo<FgpuFunctionInfo>();
    unsigned Reg = FgpuFI->getSRetReturnReg();

    if (!Reg)
      llvm_unreachable("sret virtual register not created in the entry block");
    SDValue Val =
        DAG.getCopyFromReg(Chain, DL, Reg, getPointerTy(DAG.getDataLayout()));
    unsigned V0 = Fgpu::R6;

    Chain = DAG.getCopyToReg(Chain, DL, V0, Val, Flag);
    Flag = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(V0, getPointerTy(DAG.getDataLayout())));
  }

  RetOps[0] = Chain;  // Update chain.

  // Add the flag if we have it.
  if (Flag.getNode())
    RetOps.push_back(Flag);

  // ISRs must use "eret".
  if (DAG.getMachineFunction().getFunction().hasFnAttribute("interrupt"))
    return LowerInterruptReturn(RetOps, DL, DAG);

  // Standard return on Fgpu is a "jr $ra"
  return DAG.getNode(FgpuISD::RET, DL, MVT::Other, RetOps);
}

//===----------------------------------------------------------------------===//
//                           Fgpu Inline Assembly Support
//===----------------------------------------------------------------------===//

/// getConstraintType - Given a constraint letter, return the type of
/// constraint it is for this target.
FgpuTargetLowering::ConstraintType
FgpuTargetLowering::getConstraintType(StringRef Constraint) const {
  // Fgpu specific constraints
  // GCC config/fgpu/constraints.md
  //
  // 'd' : An address register. Equivalent to r
  //       unless generating FGPU16 code.
  // 'y' : Equivalent to r; retained for
  //       backwards compatibility.
  // 'c' : A register suitable for use in an indirect
  //       jump. This will always be $25 for -mabicalls.
  // 'l' : The lo register. 1 word storage.
  // 'x' : The hilo register pair. Double word storage.
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
      default : break;
      case 'd':
      case 'y':
      case 'f':
      case 'c':
      case 'l':
      case 'x':
        return C_RegisterClass;
      case 'R':
        return C_Memory;
    }
  }

  if (Constraint == "ZC")
    return C_Memory;

  return TargetLowering::getConstraintType(Constraint);
}

/// Examine constraint type and operand type and determine a weight value.
/// This object must already have been set up with the operand type
/// and the current alternative constraint selected.
TargetLowering::ConstraintWeight
FgpuTargetLowering::getSingleConstraintMatchWeight(
    AsmOperandInfo &info, const char *constraint) const {
  ConstraintWeight weight = CW_Invalid;
  Value *CallOperandVal = info.CallOperandVal;
    // If we don't have a value, we can't do a match,
    // but allow it at the lowest weight.
  if (!CallOperandVal)
    return CW_Default;
  Type *type = CallOperandVal->getType();
  // Look at the constraint type.
  switch (*constraint) {
  default:
    weight = TargetLowering::getSingleConstraintMatchWeight(info, constraint);
    break;
  case 'd':
  case 'y':
    if (type->isIntegerTy())
      weight = CW_Register;
    break;
  case 'f': // FPU or MSA register
    if (type->isFloatTy() || type->isVectorTy())
      weight = CW_Register;
    break;
  case 'c': // $25 for indirect jumps
  case 'l': // lo register
  case 'x': // hilo register pair
    if (type->isIntegerTy())
      weight = CW_SpecificReg;
    break;
  case 'I': // signed 16 bit immediate
  case 'J': // integer zero
  case 'K': // unsigned 16 bit immediate
  case 'L': // signed 32 bit immediate where lower 16 bits are 0
  case 'N': // immediate in the range of -65535 to -1 (inclusive)
  case 'O': // signed 15 bit immediate (+- 16383)
  case 'P': // immediate in the range of 65535 to 1 (inclusive)
    if (isa<ConstantInt>(CallOperandVal))
      weight = CW_Constant;
    break;
  case 'R':
    weight = CW_Memory;
    break;
  }
  return weight;
}

/// This is a helper function to parse a physical register string and split it
/// into non-numeric and numeric parts (Prefix and Reg). The first boolean flag
/// that is returned indicates whether parsing was successful. The second flag
/// is true if the numeric part exists.
static std::pair<bool, bool> parsePhysicalReg(StringRef C, StringRef &Prefix,
                                              unsigned long long &Reg) {
  if (C.front() != '{' || C.back() != '}')
    return std::make_pair(false, false);

  // Search for the first numeric character.
  StringRef::const_iterator I, B = C.begin() + 1, E = C.end() - 1;
  I = std::find_if(B, E, isdigit);

  Prefix = StringRef(B, I - B);

  // The second flag is set to false if no numeric characters were found.
  if (I == E)
    return std::make_pair(true, false);

  // Parse the numeric characters.
  return std::make_pair(!getAsUnsignedInteger(StringRef(I, E - I), 10, Reg),
                        true);
}

EVT FgpuTargetLowering::getTypeForExtReturn(LLVMContext &Context, EVT VT,
                                            ISD::NodeType) const {
  EVT MinVT = getRegisterType(Context, MVT::i32);
  return VT.bitsLT(MinVT) ? MinVT : VT;
}

std::pair<unsigned, const TargetRegisterClass *> FgpuTargetLowering::
parseRegForInlineAsmConstraint(StringRef C, MVT VT) const {
  const TargetRegisterInfo *TRI =
      Subtarget.getRegisterInfo();
  const TargetRegisterClass *RC;
  StringRef Prefix;
  unsigned long long Reg;

  std::pair<bool, bool> R = parsePhysicalReg(C, Prefix, Reg);

  if (!R.first)
    return std::make_pair(0U, nullptr);

  if (!R.second)
    return std::make_pair(0U, nullptr);

  if (Prefix == "$f") { // Parse $f0-$f31.
    // If the size of FP registers is 64-bit or Reg is an even number, select
    // the 64-bit register class. Otherwise, select the 32-bit register class.
    VT = MVT::v32f32;

    RC = getRegClassFor(VT);
  } else { // Parse $0-$31.
    assert(Prefix == "$");
    RC = getRegClassFor((VT == MVT::Other) ? MVT::i32 : VT);
  }

  assert(Reg < RC->getNumRegs());
  return std::make_pair(*(RC->begin() + Reg), RC);
}

/// Given a register class constraint, like 'r', if this corresponds directly
/// to an LLVM register class, return a register of 0 and the register class
/// pointer.
std::pair<unsigned, const TargetRegisterClass *>
FgpuTargetLowering::getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI,
                                                 StringRef Constraint,
                                                 MVT VT) const {
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    case 'd': // Address register. Same as 'r' unless generating FGPU16 code.
    case 'y': // Same as 'r'. Exists for compatibility.
    case 'r':
      if (VT == MVT::i32 || VT == MVT::i16 || VT == MVT::i8) {
        return std::make_pair(0U, &Fgpu::GPROutRegClass);
      }
      // This will generate an error message
      return std::make_pair(0U, nullptr);
    case 'f': // FPU or MSA register
      if (VT == MVT::v32f32)
        return std::make_pair(0U, &Fgpu::VecRegsRegClass);
      break;
    case 'c': // register suitable for indirect jump
      if (VT == MVT::i32)
        return std::make_pair((unsigned)Fgpu::R25, &Fgpu::GPROutRegClass);
      // This will generate an error message
      return std::make_pair(0U, nullptr);
    }
  }

  if (!Constraint.empty()) {
    std::pair<unsigned, const TargetRegisterClass *> R;
    R = parseRegForInlineAsmConstraint(Constraint, VT);

    if (R.second)
      return R;
  }

  return TargetLowering::getRegForInlineAsmConstraint(TRI, Constraint, VT);
}

/// LowerAsmOperandForConstraint - Lower the specified operand into the Ops
/// vector.  If it is invalid, don't add anything to Ops.
void FgpuTargetLowering::LowerAsmOperandForConstraint(SDValue Op,
                                                     std::string &Constraint,
                                                     std::vector<SDValue>&Ops,
                                                     SelectionDAG &DAG) const {
  SDLoc DL(Op);
  SDValue Result;

  // Only support length 1 constraints for now.
  if (Constraint.length() > 1) return;

  char ConstraintLetter = Constraint[0];
  switch (ConstraintLetter) {
  default: break; // This will fall through to the generic implementation
  case 'I': // Signed 16 bit constant
    // If this fails, the parent routine will give an error
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      EVT Type = Op.getValueType();
      int64_t Val = C->getSExtValue();
      if (isInt<16>(Val)) {
        Result = DAG.getTargetConstant(Val, DL, Type);
        break;
      }
    }
    return;
  case 'J': // integer zero
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      EVT Type = Op.getValueType();
      int64_t Val = C->getZExtValue();
      if (Val == 0) {
        Result = DAG.getTargetConstant(0, DL, Type);
        break;
      }
    }
    return;
  case 'K': // unsigned 16 bit immediate
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      EVT Type = Op.getValueType();
      uint64_t Val = (uint64_t)C->getZExtValue();
      if (isUInt<16>(Val)) {
        Result = DAG.getTargetConstant(Val, DL, Type);
        break;
      }
    }
    return;
  case 'L': // signed 32 bit immediate where lower 16 bits are 0
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      EVT Type = Op.getValueType();
      int64_t Val = C->getSExtValue();
      if ((isInt<32>(Val)) && ((Val & 0xffff) == 0)){
        Result = DAG.getTargetConstant(Val, DL, Type);
        break;
      }
    }
    return;
  case 'N': // immediate in the range of -65535 to -1 (inclusive)
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      EVT Type = Op.getValueType();
      int64_t Val = C->getSExtValue();
      if ((Val >= -65535) && (Val <= -1)) {
        Result = DAG.getTargetConstant(Val, DL, Type);
        break;
      }
    }
    return;
  case 'O': // signed 15 bit immediate
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      EVT Type = Op.getValueType();
      int64_t Val = C->getSExtValue();
      if ((isInt<15>(Val))) {
        Result = DAG.getTargetConstant(Val, DL, Type);
        break;
      }
    }
    return;
  case 'P': // immediate in the range of 1 to 65535 (inclusive)
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      EVT Type = Op.getValueType();
      int64_t Val = C->getSExtValue();
      if ((Val <= 65535) && (Val >= 1)) {
        Result = DAG.getTargetConstant(Val, DL, Type);
        break;
      }
    }
    return;
  }

  if (Result.getNode()) {
    Ops.push_back(Result);
    return;
  }

  TargetLowering::LowerAsmOperandForConstraint(Op, Constraint, Ops, DAG);
}

bool FgpuTargetLowering::isLegalAddressingMode(const DataLayout &DL,
                                               const AddrMode &AM, Type *Ty,
                                               unsigned AS,
                                               Instruction *I) const {
  // No global is ever allowed as a base.
  if (AM.BaseGV)
    return false;

  switch (AM.Scale) {
  case 0: // "r+i" or just "i", depending on HasBaseReg.
    break;
  case 1:
    if (!AM.HasBaseReg) // allow "r+i".
      break;
    return false; // disallow "r+r" or "r+r+i".
  default:
    return false;
  }

  return true;
}

bool
FgpuTargetLowering::isOffsetFoldingLegal(const GlobalAddressSDNode *GA) const {
  // The Fgpu target isn't yet aware of offsets.
  return false;
}

EVT FgpuTargetLowering::getOptimalMemOpType(
    const MemOp &Op, const AttributeList &FuncAttributes) const {

  return MVT::i32;
}

bool FgpuTargetLowering::isFPImmLegal(const APFloat &Imm, EVT VT,
                                      bool ForCodeSize) const {
  if (VT != MVT::f32 && VT != MVT::f64)
    return false;
  return true; // can do with Li32
//  if (Imm.isNegZero())
//    return false;
//  return Imm.isZero();
}

unsigned FgpuTargetLowering::getJumpTableEncoding() const {

  return TargetLowering::getJumpTableEncoding();
}

bool FgpuTargetLowering::useSoftFloat() const {
  return false;
}

void FgpuTargetLowering::copyByValRegs(
    SDValue Chain, const SDLoc &DL, std::vector<SDValue> &OutChains,
    SelectionDAG &DAG, const ISD::ArgFlagsTy &Flags,
    SmallVectorImpl<SDValue> &InVals, const Argument *FuncArg,
    unsigned FirstReg, unsigned LastReg, const CCValAssign &VA,
    FgpuCCState &State) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned GPRSizeInBytes = 4;
  unsigned NumRegs = LastReg - FirstReg;
  unsigned RegAreaSize = NumRegs * GPRSizeInBytes;
  unsigned FrameObjSize = std::max(Flags.getByValSize(), RegAreaSize);
  int FrameObjOffset;
  ArrayRef<MCPhysReg> ByValArgRegs = ABI.GetByValArgRegs();

  if (RegAreaSize)
    FrameObjOffset =
        (int)ABI.GetCalleeAllocdArgSizeInBytes(State.getCallingConv()) -
        (int)((ByValArgRegs.size() - FirstReg) * GPRSizeInBytes);
  else
    FrameObjOffset = VA.getLocMemOffset();

  // Create frame object.
  EVT PtrTy = getPointerTy(DAG.getDataLayout());
  // Make the fixed object stored to mutable so that the load instructions
  // referencing it have their memory dependencies added.
  // Set the frame object as isAliased which clears the underlying objects
  // vector in ScheduleDAGInstrs::buildSchedGraph() resulting in addition of all
  // stores as dependencies for loads referencing this fixed object.
  int FI = MFI.CreateFixedObject(FrameObjSize, FrameObjOffset, false, true);
  SDValue FIN = DAG.getFrameIndex(FI, PtrTy);
  InVals.push_back(FIN);

  if (!NumRegs)
    return;

  // Copy arg registers.
  MVT RegTy = MVT::getIntegerVT(GPRSizeInBytes * 8);
  const TargetRegisterClass *RC = getRegClassFor(RegTy);

  for (unsigned I = 0; I < NumRegs; ++I) {
    unsigned ArgReg = ByValArgRegs[FirstReg + I];
    unsigned VReg = addLiveIn(MF, ArgReg, RC);
    unsigned Offset = I * GPRSizeInBytes;
    SDValue StorePtr = DAG.getNode(ISD::ADD, DL, PtrTy, FIN,
                                   DAG.getConstant(Offset, DL, PtrTy));
    SDValue Store = DAG.getStore(Chain, DL, DAG.getRegister(VReg, RegTy),
                                 StorePtr, MachinePointerInfo(FuncArg, Offset));
    OutChains.push_back(Store);
  }
}

// Copy byVal arg to registers and stack.
void FgpuTargetLowering::passByValArg(
    SDValue Chain, const SDLoc &DL,
    std::deque<std::pair<unsigned, SDValue>> &RegsToPass,
    SmallVectorImpl<SDValue> &MemOpChains, SDValue StackPtr,
    MachineFrameInfo &MFI, SelectionDAG &DAG, SDValue Arg, unsigned FirstReg,
    unsigned LastReg, const ISD::ArgFlagsTy &Flags, bool isLittle,
    const CCValAssign &VA) const {
  unsigned ByValSizeInBytes = Flags.getByValSize();
  unsigned OffsetInBytes = 0; // From beginning of struct
  unsigned RegSizeInBytes = 4;
  Align Alignment =
      std::min(Flags.getNonZeroByValAlign(), Align(RegSizeInBytes));
  EVT PtrTy = getPointerTy(DAG.getDataLayout()),
      RegTy = MVT::getIntegerVT(RegSizeInBytes * 8);
  unsigned NumRegs = LastReg - FirstReg;

  if (NumRegs) {
    ArrayRef<MCPhysReg> ArgRegs = ABI.GetByValArgRegs();
    bool LeftoverBytes = (NumRegs * RegSizeInBytes > ByValSizeInBytes);
    unsigned I = 0;

    // Copy words to registers.
    for (; I < NumRegs - LeftoverBytes; ++I, OffsetInBytes += RegSizeInBytes) {
      SDValue LoadPtr = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                                    DAG.getConstant(OffsetInBytes, DL, PtrTy));
      SDValue LoadVal = DAG.getLoad(RegTy, DL, Chain, LoadPtr,
                                    MachinePointerInfo(), Alignment);
      MemOpChains.push_back(LoadVal.getValue(1));
      unsigned ArgReg = ArgRegs[FirstReg + I];
      RegsToPass.push_back(std::make_pair(ArgReg, LoadVal));
    }

    // Return if the struct has been fully copied.
    if (ByValSizeInBytes == OffsetInBytes)
      return;

    // Copy the remainder of the byval argument with sub-word loads and shifts.
    if (LeftoverBytes) {
      SDValue Val;

      for (unsigned LoadSizeInBytes = RegSizeInBytes / 2, TotalBytesLoaded = 0;
           OffsetInBytes < ByValSizeInBytes; LoadSizeInBytes /= 2) {
        unsigned RemainingSizeInBytes = ByValSizeInBytes - OffsetInBytes;

        if (RemainingSizeInBytes < LoadSizeInBytes)
          continue;

        // Load subword.
        SDValue LoadPtr = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                                      DAG.getConstant(OffsetInBytes, DL,
                                                      PtrTy));
        SDValue LoadVal = DAG.getExtLoad(
            ISD::ZEXTLOAD, DL, RegTy, Chain, LoadPtr, MachinePointerInfo(),
            MVT::getIntegerVT(LoadSizeInBytes * 8), Alignment);
        MemOpChains.push_back(LoadVal.getValue(1));

        // Shift the loaded value.
        unsigned Shamt;

        if (isLittle)
          Shamt = TotalBytesLoaded * 8;
        else
          Shamt = (RegSizeInBytes - (TotalBytesLoaded + LoadSizeInBytes)) * 8;

        SDValue Shift = DAG.getNode(ISD::SHL, DL, RegTy, LoadVal,
                                    DAG.getConstant(Shamt, DL, MVT::i32));

        if (Val.getNode())
          Val = DAG.getNode(ISD::OR, DL, RegTy, Val, Shift);
        else
          Val = Shift;

        OffsetInBytes += LoadSizeInBytes;
        TotalBytesLoaded += LoadSizeInBytes;
        Alignment = std::min(Alignment, Align(LoadSizeInBytes));
      }

      unsigned ArgReg = ArgRegs[FirstReg + I];
      RegsToPass.push_back(std::make_pair(ArgReg, Val));
      return;
    }
  }

  // Copy remainder of byval arg to it with memcpy.
  unsigned MemCpySize = ByValSizeInBytes - OffsetInBytes;
  SDValue Src = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                            DAG.getConstant(OffsetInBytes, DL, PtrTy));
  SDValue Dst = DAG.getNode(ISD::ADD, DL, PtrTy, StackPtr,
                            DAG.getIntPtrConstant(VA.getLocMemOffset(), DL));
  Chain = DAG.getMemcpy(
      Chain, DL, Dst, Src, DAG.getConstant(MemCpySize, DL, PtrTy),
      Align(Alignment), /*isVolatile=*/false, /*AlwaysInline=*/false,
      /*isTailCall=*/false, MachinePointerInfo(), MachinePointerInfo());
  MemOpChains.push_back(Chain);
}

void FgpuTargetLowering::writeVarArgRegs(std::vector<SDValue> &OutChains,
                                         SDValue Chain, const SDLoc &DL,
                                         SelectionDAG &DAG,
                                         CCState &State) const {
  ArrayRef<MCPhysReg> ArgRegs = ABI.GetVarArgRegs();
  unsigned Idx = State.getFirstUnallocated(ArgRegs);
  unsigned RegSizeInBytes = 4;
  MVT RegTy = MVT::getIntegerVT(RegSizeInBytes * 8);
  const TargetRegisterClass *RC = getRegClassFor(RegTy);
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  FgpuFunctionInfo *FgpuFI = MF.getInfo<FgpuFunctionInfo>();

  // Offset of the first variable argument from stack pointer.
  int VaArgOffset;

  if (ArgRegs.size() == Idx)
    VaArgOffset = alignTo(State.getNextStackOffset(), RegSizeInBytes);
  else {
    VaArgOffset =
        (int)ABI.GetCalleeAllocdArgSizeInBytes(State.getCallingConv()) -
        (int)(RegSizeInBytes * (ArgRegs.size() - Idx));
  }

  // Record the frame index of the first variable argument
  // which is a value necessary to VASTART.
  int FI = MFI.CreateFixedObject(RegSizeInBytes, VaArgOffset, true);
  FgpuFI->setVarArgsFrameIndex(FI);

  // Copy the integer registers that have not been used for argument passing
  // to the argument register save area. For O32, the save area is allocated
  // in the caller's stack frame, while for N32/64, it is allocated in the
  // callee's stack frame.
  for (unsigned I = Idx; I < ArgRegs.size();
       ++I, VaArgOffset += RegSizeInBytes) {
    unsigned Reg = addLiveIn(MF, ArgRegs[I], RC);
    SDValue ArgValue = DAG.getCopyFromReg(Chain, DL, Reg, RegTy);
    FI = MFI.CreateFixedObject(RegSizeInBytes, VaArgOffset, true);
    SDValue PtrOff = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
    SDValue Store =
        DAG.getStore(Chain, DL, ArgValue, PtrOff, MachinePointerInfo());
    cast<StoreSDNode>(Store.getNode())->getMemOperand()->setValue(
        (Value *)nullptr);
    OutChains.push_back(Store);
  }
}

void FgpuTargetLowering::HandleByVal(CCState *State, unsigned &Size,
                                     Align Alignment) const {
  const TargetFrameLowering *TFL = Subtarget.getFrameLowering();

  assert(Size && "Byval argument's size shouldn't be 0.");

  Alignment = std::min(Alignment, TFL->getStackAlign());

  unsigned FirstReg = 0;
  unsigned NumRegs = 0;

  if (State->getCallingConv() != CallingConv::Fast) {
    unsigned RegSizeInBytes = 4;
    ArrayRef<MCPhysReg> IntArgRegs = ABI.GetByValArgRegs();
    // FIXME: The O32 case actually describes no shadow registers.
    const MCPhysReg *ShadowRegs =  IntArgRegs.data();

    // We used to check the size as well but we can't do that anymore since
    // CCState::HandleByVal() rounds up the size after calling this function.
    assert(
        Alignment >= Align(RegSizeInBytes) &&
        "Byval argument's alignment should be a multiple of RegSizeInBytes.");

    FirstReg = State->getFirstUnallocated(IntArgRegs);

    // If Alignment > RegSizeInBytes, the first arg register must be even.
    // FIXME: This condition happens to do the right thing but it's not the
    //        right way to test it. We want to check that the stack frame offset
    //        of the register is aligned.
    if ((Alignment > RegSizeInBytes) && (FirstReg % 2)) {
      State->AllocateReg(IntArgRegs[FirstReg], ShadowRegs[FirstReg]);
      ++FirstReg;
    }

    // Mark the registers allocated.
    Size = alignTo(Size, RegSizeInBytes);
    for (unsigned I = FirstReg; Size > 0 && (I < IntArgRegs.size());
         Size -= RegSizeInBytes, ++I, ++NumRegs)
      State->AllocateReg(IntArgRegs[I], ShadowRegs[I]);
  }

  State->addInRegsParamInfo(FirstReg, FirstReg + NumRegs);
}

MachineBasicBlock *FgpuTargetLowering::emitPseudoSELECT(MachineInstr &MI,
                                                        MachineBasicBlock *BB,
                                                        bool isFPCmp,
                                                        unsigned Opc) const {
//  assert(!(Subtarget.hasFgpu4() || Subtarget.hasFgpu32()) &&
//         "Subtarget already supports SELECT nodes with the use of"
//         "conditional-move instructions.");
// TODO: can we use the movz/movn things??

  const TargetInstrInfo *TII =
      Subtarget.getInstrInfo();
  DebugLoc DL = MI.getDebugLoc();

  // To "insert" a SELECT instruction, we actually have to insert the
  // diamond control-flow pattern.  The incoming instruction knows the
  // destination vreg to set, the condition code register to branch on, the
  // true/false values to select between, and a branch opcode to use.
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  MachineFunction::iterator It = ++BB->getIterator();

  //  thisMBB:
  //  ...
  //   TrueVal = ...
  //   setcc r1, r2, r3
  //   bNE   r1, r0, copy1MBB
  //   fallthrough --> copy0MBB
  MachineBasicBlock *thisMBB  = BB;
  MachineFunction *F = BB->getParent();
  MachineBasicBlock *copy0MBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *sinkMBB  = F->CreateMachineBasicBlock(LLVM_BB);
  F->insert(It, copy0MBB);
  F->insert(It, sinkMBB);

  // Transfer the remainder of BB and its successor edges to sinkMBB.
  sinkMBB->splice(sinkMBB->begin(), BB,
                  std::next(MachineBasicBlock::iterator(MI)), BB->end());
  sinkMBB->transferSuccessorsAndUpdatePHIs(BB);

  // Next, add the true and fallthrough blocks as its successors.
  BB->addSuccessor(copy0MBB);
  BB->addSuccessor(sinkMBB);

  if (isFPCmp) {
    // bc1[tf] cc, sinkMBB
    BuildMI(BB, DL, TII->get(Opc))
        .addReg(MI.getOperand(1).getReg())
        .addMBB(sinkMBB);
  } else {
    // bne rs, $0, sinkMBB
    BuildMI(BB, DL, TII->get(Opc))
        .addReg(MI.getOperand(1).getReg())
        .addReg(Fgpu::ZERO)
        .addMBB(sinkMBB);
  }

  //  copy0MBB:
  //   %FalseValue = ...
  //   # fallthrough to sinkMBB
  BB = copy0MBB;

  // Update machine-CFG edges
  BB->addSuccessor(sinkMBB);

  //  sinkMBB:
  //   %Result = phi [ %TrueValue, thisMBB ], [ %FalseValue, copy0MBB ]
  //  ...
  BB = sinkMBB;

  BuildMI(*BB, BB->begin(), DL, TII->get(Fgpu::PHI), MI.getOperand(0).getReg())
      .addReg(MI.getOperand(2).getReg())
      .addMBB(thisMBB)
      .addReg(MI.getOperand(3).getReg())
      .addMBB(copy0MBB);

  MI.eraseFromParent(); // The pseudo instruction is gone now.

  return BB;
}

MachineBasicBlock *
FgpuTargetLowering::emitPseudoD_SELECT(MachineInstr &MI,
                                       MachineBasicBlock *BB) const {
//  assert(!(Subtarget.hasFgpu4() || Subtarget.hasFgpu32()) &&
//         "Subtarget already supports SELECT nodes with the use of"
//         "conditional-move instructions.");

  const TargetInstrInfo *TII = Subtarget.getInstrInfo();
  DebugLoc DL = MI.getDebugLoc();

  // D_SELECT substitutes two SELECT nodes that goes one after another and
  // have the same condition operand. On machines which don't have
  // conditional-move instruction, it reduces unnecessary branch instructions
  // which are result of using two diamond patterns that are result of two
  // SELECT pseudo instructions.
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  MachineFunction::iterator It = ++BB->getIterator();

  //  thisMBB:
  //  ...
  //   TrueVal = ...
  //   setcc r1, r2, r3
  //   bNE   r1, r0, copy1MBB
  //   fallthrough --> copy0MBB
  MachineBasicBlock *thisMBB = BB;
  MachineFunction *F = BB->getParent();
  MachineBasicBlock *copy0MBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *sinkMBB = F->CreateMachineBasicBlock(LLVM_BB);
  F->insert(It, copy0MBB);
  F->insert(It, sinkMBB);

  // Transfer the remainder of BB and its successor edges to sinkMBB.
  sinkMBB->splice(sinkMBB->begin(), BB,
                  std::next(MachineBasicBlock::iterator(MI)), BB->end());
  sinkMBB->transferSuccessorsAndUpdatePHIs(BB);

  // Next, add the true and fallthrough blocks as its successors.
  BB->addSuccessor(copy0MBB);
  BB->addSuccessor(sinkMBB);

  // bne rs, $0, sinkMBB
  BuildMI(BB, DL, TII->get(Fgpu::BNE))
      .addReg(MI.getOperand(2).getReg())
      .addReg(Fgpu::ZERO)
      .addMBB(sinkMBB);

  //  copy0MBB:
  //   %FalseValue = ...
  //   # fallthrough to sinkMBB
  BB = copy0MBB;

  // Update machine-CFG edges
  BB->addSuccessor(sinkMBB);

  //  sinkMBB:
  //   %Result = phi [ %TrueValue, thisMBB ], [ %FalseValue, copy0MBB ]
  //  ...
  BB = sinkMBB;

  // Use two PHI nodes to select two reults
  BuildMI(*BB, BB->begin(), DL, TII->get(Fgpu::PHI), MI.getOperand(0).getReg())
      .addReg(MI.getOperand(3).getReg())
      .addMBB(thisMBB)
      .addReg(MI.getOperand(5).getReg())
      .addMBB(copy0MBB);
  BuildMI(*BB, BB->begin(), DL, TII->get(Fgpu::PHI), MI.getOperand(1).getReg())
      .addReg(MI.getOperand(4).getReg())
      .addMBB(thisMBB)
      .addReg(MI.getOperand(6).getReg())
      .addMBB(copy0MBB);

  MI.eraseFromParent(); // The pseudo instruction is gone now.

  return BB;
}

// FIXME? Maybe this could be a TableGen attribute on some registers and
// this table could be generated automatically from RegInfo.
Register
FgpuTargetLowering::getRegisterByName(const char *RegName, LLT VT,
                                      const MachineFunction &MF) const {
  // Named registers is expected to be fairly rare. For now, just support $28
  // since the linux kernel uses it.
  {
    Register Reg = StringSwitch<Register>(RegName)
                         .Case("$28", Fgpu::R28)
                         .Default(Register());
    if (Reg)
      return Reg;
  }
  report_fatal_error("Invalid register name global variable");
}

MachineBasicBlock *FgpuTargetLowering::emitLDR_W(MachineInstr &MI,
                                                 MachineBasicBlock *BB) const {
  MachineFunction *MF = BB->getParent();
  MachineRegisterInfo &MRI = MF->getRegInfo();
  const TargetInstrInfo *TII = Subtarget.getInstrInfo();
  const bool IsLittle = true;
  DebugLoc DL = MI.getDebugLoc();

  Register Dest = MI.getOperand(0).getReg();
  Register Address = MI.getOperand(1).getReg();
  unsigned Imm = MI.getOperand(2).getImm();

  MachineBasicBlock::iterator I(MI);

  // Fgpu release 6 can load from adress that is not naturally-aligned.
  //Register Temp = MRI.createVirtualRegister(&Fgpu::GPROutRegClass);
  BuildMI(*BB, I, DL, TII->get(Fgpu::LW))
      .addDef(Dest)
      .addUse(Address)
      .addImm(Imm);
  //BuildMI(*BB, I, DL, TII->get(Fgpu::FILL_W)).addDef(Dest).addUse(Temp);
  //TODO: uhh is this correct

  MI.eraseFromParent();
  return BB;
}

MachineBasicBlock *FgpuTargetLowering::emitLDR_D(MachineInstr &MI,
                                                 MachineBasicBlock *BB) const {
  MachineFunction *MF = BB->getParent();
  MachineRegisterInfo &MRI = MF->getRegInfo();
  const TargetInstrInfo *TII = Subtarget.getInstrInfo();
  const bool IsLittle = true;
  DebugLoc DL = MI.getDebugLoc();

  Register Dest = MI.getOperand(0).getReg();
  Register Address = MI.getOperand(1).getReg();
  unsigned Imm = MI.getOperand(2).getImm();

  MachineBasicBlock::iterator I(MI);
  assert(false && "Wide load not supported");

//  if (Subtarget.hasFgpu32r6() || Subtarget.hasFgpu64r6()) {
//    // Fgpu release 6 can load from adress that is not naturally-aligned.
//    if (Subtarget.isGP64bit()) {
//      Register Temp = MRI.createVirtualRegister(&Fgpu::GPR64RegClass);
//      BuildMI(*BB, I, DL, TII->get(Fgpu::LD))
//          .addDef(Temp)
//          .addUse(Address)
//          .addImm(Imm);
//      BuildMI(*BB, I, DL, TII->get(Fgpu::FILL_D)).addDef(Dest).addUse(Temp);
//    } else {
//      Register Wtemp = MRI.createVirtualRegister(&Fgpu::MSA128WRegClass);
//      Register Lo = MRI.createVirtualRegister(&Fgpu::GPROutRegClass);
//      Register Hi = MRI.createVirtualRegister(&Fgpu::GPROutRegClass);
//      BuildMI(*BB, I, DL, TII->get(Fgpu::LW))
//          .addDef(Lo)
//          .addUse(Address)
//          .addImm(Imm + (IsLittle ? 0 : 4));
//      BuildMI(*BB, I, DL, TII->get(Fgpu::LW))
//          .addDef(Hi)
//          .addUse(Address)
//          .addImm(Imm + (IsLittle ? 4 : 0));
//      BuildMI(*BB, I, DL, TII->get(Fgpu::FILL_W)).addDef(Wtemp).addUse(Lo);
//      BuildMI(*BB, I, DL, TII->get(Fgpu::INSERT_W), Dest)
//          .addUse(Wtemp)
//          .addUse(Hi)
//          .addImm(1);
//    }
//  } else {
//    // Fgpu release 5 needs to use instructions that can load from an unaligned
//    // memory address.
//    Register LoHalf = MRI.createVirtualRegister(&Fgpu::GPROutRegClass);
//    Register LoFull = MRI.createVirtualRegister(&Fgpu::GPROutRegClass);
//    Register LoUndef = MRI.createVirtualRegister(&Fgpu::GPROutRegClass);
//    Register HiHalf = MRI.createVirtualRegister(&Fgpu::GPROutRegClass);
//    Register HiFull = MRI.createVirtualRegister(&Fgpu::GPROutRegClass);
//    Register HiUndef = MRI.createVirtualRegister(&Fgpu::GPROutRegClass);
//    Register Wtemp = MRI.createVirtualRegister(&Fgpu::MSA128WRegClass);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::IMPLICIT_DEF)).addDef(LoUndef);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::LWR))
//        .addDef(LoHalf)
//        .addUse(Address)
//        .addImm(Imm + (IsLittle ? 0 : 7))
//        .addUse(LoUndef);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::LWL))
//        .addDef(LoFull)
//        .addUse(Address)
//        .addImm(Imm + (IsLittle ? 3 : 4))
//        .addUse(LoHalf);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::IMPLICIT_DEF)).addDef(HiUndef);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::LWR))
//        .addDef(HiHalf)
//        .addUse(Address)
//        .addImm(Imm + (IsLittle ? 4 : 3))
//        .addUse(HiUndef);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::LWL))
//        .addDef(HiFull)
//        .addUse(Address)
//        .addImm(Imm + (IsLittle ? 7 : 0))
//        .addUse(HiHalf);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::FILL_W)).addDef(Wtemp).addUse(LoFull);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::INSERT_W), Dest)
//        .addUse(Wtemp)
//        .addUse(HiFull)
//        .addImm(1);
//  }

  MI.eraseFromParent();
  return BB;
}

MachineBasicBlock *FgpuTargetLowering::emitSTR_W(MachineInstr &MI,
                                                 MachineBasicBlock *BB) const {
  MachineFunction *MF = BB->getParent();
  MachineRegisterInfo &MRI = MF->getRegInfo();
  const TargetInstrInfo *TII = Subtarget.getInstrInfo();
  const bool IsLittle = true;
  DebugLoc DL = MI.getDebugLoc();

  Register StoreVal = MI.getOperand(0).getReg();
  Register Address = MI.getOperand(1).getReg();
  unsigned Imm = MI.getOperand(2).getImm();

  MachineBasicBlock::iterator I(MI);
  BuildMI(*BB, I, DL, TII->get(Fgpu::SW))
      .addUse(StoreVal)
      .addUse(Address)
      .addImm(Imm);


  MI.eraseFromParent();

  return BB;
}

MachineBasicBlock *FgpuTargetLowering::emitSTR_D(MachineInstr &MI,
                                                 MachineBasicBlock *BB) const {
  MachineFunction *MF = BB->getParent();
  MachineRegisterInfo &MRI = MF->getRegInfo();
  const TargetInstrInfo *TII = Subtarget.getInstrInfo();
  const bool IsLittle = true;
  DebugLoc DL = MI.getDebugLoc();

  Register StoreVal = MI.getOperand(0).getReg();
  Register Address = MI.getOperand(1).getReg();
  unsigned Imm = MI.getOperand(2).getImm();

  MachineBasicBlock::iterator I(MI);
  assert(false && "Wide load not supported");


//  // Fgpu release 6 can store to adress that is not naturally-aligned.
//  if (Subtarget.isGP64bit()) {
//    Register BitcastD = MRI.createVirtualRegister(&Fgpu::MSA128DRegClass);
//    Register Lo = MRI.createVirtualRegister(&Fgpu::GPR64RegClass);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::COPY))
//        .addDef(BitcastD)
//        .addUse(StoreVal);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::COPY_S_D))
//        .addDef(Lo)
//        .addUse(BitcastD)
//        .addImm(0);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::SD))
//        .addUse(Lo)
//        .addUse(Address)
//        .addImm(Imm);
//  } else {
//    Register BitcastW = MRI.createVirtualRegister(&Fgpu::MSA128WRegClass);
//    Register Lo = MRI.createVirtualRegister(&Fgpu::GPROutRegClass);
//    Register Hi = MRI.createVirtualRegister(&Fgpu::GPROutRegClass);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::COPY))
//        .addDef(BitcastW)
//        .addUse(StoreVal);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::COPY_S_W))
//        .addDef(Lo)
//        .addUse(BitcastW)
//        .addImm(0);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::COPY_S_W))
//        .addDef(Hi)
//        .addUse(BitcastW)
//        .addImm(1);
//    BuildMI(*BB, I, DL, TII->get(Fgpu::SW))
//        .addUse(Lo)
//        .addUse(Address)
//        .addImm(Imm + (IsLittle ? 0 : 4));
//    BuildMI(*BB, I, DL, TII->get(Fgpu::SW))
//        .addUse(Hi)
//        .addUse(Address)
//        .addImm(Imm + (IsLittle ? 4 : 0));
//  }

  MI.eraseFromParent();
  return BB;
}
