//=- FgpuScheduleGeneric.td - Generic Scheduling Definitions -*- tablegen -*-=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes the interAptiv processor in a manner of speaking. It
// describes a hypothetical version of the in-order FGPU32R2 interAptiv with all
// branches of the FGPU ISAs, ASEs and ISA variants. The itinerary lists are
// broken down into per ISA lists, so that this file can be used to rapidly
// develop new schedule models.
//
//===----------------------------------------------------------------------===//
def FgpuGenericModel : SchedMachineModel {
  int IssueWidth = 1;
  int MicroOpBufferSize = 0;

  // These figures assume an L1 hit.
  int LoadLatency = 2;
  int MispredictPenalty = 4;

  int HighLatency = 37;
  list<Predicate> UnsupportedFeatures = [];

  let CompleteModel = 1;
  let PostRAScheduler = 1;

  // FIXME: Remove when all errors have been fixed.
  let FullInstRWOverlapCheck = 1;
}

let SchedModel = FgpuGenericModel in {

// ALU Pipeline
// ============

def GenericALU : ProcResource<1> { let BufferSize = 1; }
def GenericIssueALU : ProcResource<1> { let Super = GenericALU; }

def GenericWriteALU : SchedWriteRes<[GenericIssueALU]>;

// add, addi, addiu, addu, and, andi, clo, clz, ext, ins, lui, nor, or, ori,
// rotr, rotrv, seb, seh, sll, sllv, slt, slti, sltiu, sltu, sra, srav, srl,
// srlv, ssnop, sub, subu, wsbh, xor, xori
def : InstRW<[GenericWriteALU], (instrs ADD, ADDi, ADDiu, ADDu, AND, ANDi,
                                 CLO, CLZ, EXT, INS, LEA_ADDiu, LUi, NOP,
                                 NOR, OR, ORi, ROTR, ROTRV, SEB, SEH, SLL,
                                 SLLV, SLT, SLTi, SLTiu, SLTu, SRA, SRAV, SRL,
                                 SRLV, SSNOP, SUB, SUBu, WSBH, XOR, XORi)>;

def : InstRW<[GenericWriteALU], (instrs COPY)>;

// FGPUR6
// ======

// addiupc, align, aluipc, aui, auipc, bitswap, clo, clz, lsa, seleqz, selnez
def : InstRW<[GenericWriteALU], (instrs ADDIUPC, ALIGN, ALUIPC, AUI,
                                 AUIPC, BITSWAP, CLO_R6, CLZ_R6, LSA_R6,
                                 SELEQZ, SELNEZ)>;

// FGPU64
// ======

def : InstRW<[GenericWriteALU], (instrs AND64, ANDi64, DEXT64_32, DSLL64_32,
                                 ORi64, SEB64, SEH64, SLL64_32, SLL64_64,
                                 SLT64, SLTi64, SLTiu64, SLTu64, XOR64,
                                 XORi64)>;

def : InstRW<[GenericWriteALU], (instrs DADD, DADDi, DADDiu, DADDu, DCLO,
                                 DCLZ, DEXT, DEXTM, DEXTU, DINS, DINSM, DINSU,
                                 DROTR, DROTR32, DROTRV, DSBH, DSHD, DSLL,
                                 DSLL32, DSLLV, DSRA, DSRA32, DSRAV, DSRL,
                                 DSRL32, DSRLV, DSUB, DSUBu, LEA_ADDiu64,
                                 LUi64, NOR64, OR64)>;

// FGPU64R6
// ========

def : InstRW<[GenericWriteALU], (instrs DALIGN, DAHI, DATI, DAUI, DCLO_R6,
                                 DCLZ_R6, DBITSWAP, DLSA, DLSA_R6, SELEQZ64,
                                 SELNEZ64)>;


def GenericMDU : ProcResource<1> { let BufferSize = 1; }
def GenericIssueMDU : ProcResource<1> { let Super = GenericALU; }
def GenericIssueDIV : ProcResource<1> { let Super = GenericMDU; }
def GenericWriteHILO : SchedWriteRes<[GenericIssueMDU]>;
def GenericWriteALULong : SchedWriteRes<[GenericIssueALU]> { let Latency = 5; }
def GenericWriteMove : SchedWriteRes<[GenericIssueALU]> { let Latency = 2; }
def GenericWriteMul : SchedWriteRes<[GenericIssueMDU]> { let Latency = 4; }

def : InstRW<[GenericWriteHILO], (instrs MADD, MADDU, MSUB, MSUBU)>;

def : InstRW<[GenericWriteHILO], (instrs PseudoMADD, PseudoMADDU, PseudoMSUB,
                                  PseudoMSUBU, PseudoMULT, PseudoMULTu)>;

def GenericWriteMDUtoGPR : SchedWriteRes<[GenericIssueMDU]> {
  let Latency = 5;
}

def GenericWriteDIV : SchedWriteRes<[GenericIssueDIV]> {
  // Estimated worst case
  let Latency = 33;
  let ResourceCycles = [33];
}
def GenericWriteDIVU : SchedWriteRes<[GenericIssueDIV]> {
  // Estimated worst case
  let Latency = 31;
  let ResourceCycles = [31];
}

// mul
def : InstRW<[GenericWriteMDUtoGPR], (instrs MUL)>;

// mult, multu
def : InstRW<[GenericWriteMul], (instrs MULT, MULTu)>;

// div, sdiv
def : InstRW<[GenericWriteDIV], (instrs PseudoSDIV, SDIV)>;

def : InstRW<[GenericWriteDIVU], (instrs PseudoUDIV, UDIV)>;

// mfhi, mflo, movn, mthi, mtlo, rdwhr
def : InstRW<[GenericWriteALULong], (instrs MFHI, MFLO, PseudoMFHI,
                                     PseudoMFLO)>;

def : InstRW<[GenericWriteMove], (instrs MTHI, MTLO, RDHWR, PseudoMTLOHI)>;

def : InstRW<[GenericWriteALU], (instrs MOVN_I_I, MOVZ_I_I)>;

// FGPUR6
// ======

// muh, muhu, mulu, mul
def : InstRW<[GenericWriteMul], (instrs MUH, MUHU, MULU, MUL_R6)>;

// divu, udiv
def : InstRW<[GenericWriteDIV], (instrs MOD, MODU, DIV, DIVU)>;

// FGPU64
// ======

def : InstRW<[GenericWriteHILO], (instrs DMULU, DMULT, DMULTu, PseudoDMULT,
                                  PseudoDMULTu)>;

def : InstRW<[GenericWriteDIV], (instrs DSDIV, PseudoDSDIV)>;

def : InstRW<[GenericWriteDIVU], (instrs DUDIV, PseudoDUDIV)>;

def : InstRW<[GenericWriteALULong], (instrs MFHI64, MFLO64, PseudoMFHI64,
                                     PseudoMFLO64, PseudoMTLOHI64)>;

def : InstRW<[GenericWriteMove], (instrs MTHI64, MTLO64, RDHWR64)>;

// mov[zn]
def : InstRW<[GenericWriteALU], (instrs MOVN_I_I64, MOVN_I64_I, MOVN_I64_I64,
                                 MOVZ_I_I64, MOVZ_I64_I, MOVZ_I64_I64)>;


// FGPU64R6
// ========

def : InstRW<[GenericWriteMDUtoGPR], (instrs DMUH, DMUHU, DMUL_R6)>;

def : InstRW<[GenericWriteDIV], (instrs DDIV, DMOD)>;

def : InstRW<[GenericWriteDIVU], (instrs DDIVU, DMODU)>;

// CTISTD Pipeline
// ---------------

def GenericIssueCTISTD : ProcResource<1> { let Super = GenericALU; }

def GenericLDST : ProcResource<1> { let BufferSize = 1; }
def GenericIssueLDST : ProcResource<1> { let Super = GenericLDST; }

def GenericWriteJump : SchedWriteRes<[GenericIssueCTISTD]>;
def GenericWriteJumpAndLink : SchedWriteRes<[GenericIssueCTISTD]> {
  let Latency = 2;
}

// b, beq, beql, bg[et]z, bl[et]z, bne, bnel, j, syscall, jal, bltzal, jalx,
// jalr, jr.hb, jr, jalr.hb, jarlc, jialc
def : InstRW<[GenericWriteJump], (instrs B, BAL, BAL_BR, BEQ, BNE, BGTZ, BGEZ,
                                  BLEZ, BLTZ, BLTZAL, J, JALX, JR, JR_HB, ERET,
                                  ERet, ERETNC, DERET)>;

def : InstRW<[GenericWriteJump], (instrs BEQL, BNEL, BGEZL, BGTZL, BLEZL,
                                  BLTZL)>;

def : InstRW<[GenericWriteJump], (instrs TAILCALL, TAILCALLREG,
                                  TAILCALLREGHB, PseudoIndirectBranch,
                                  PseudoIndirectHazardBranch, PseudoReturn,
                                  RetRA)>;

def : InstRW<[GenericWriteJumpAndLink], (instrs BGEZAL, JAL, JALR, JALR_HB,
                                         JALRHBPseudo, JALRPseudo)>;

def : InstRW<[GenericWriteJumpAndLink], (instrs BGEZALL, BLTZALL)>;

def GenericWriteTrap : SchedWriteRes<[GenericIssueCTISTD]>;

def : InstRW<[GenericWriteTrap], (instrs BREAK, SYSCALL, TEQ, TEQI,
                                  TGE, TGEI, TGEIU, TGEU, TNE,
                                  TNEI, TLT, TLTI, TLTU, TTLTIU,
                                  TRAP, SDBBP)>;

// FGPUR6
// ======

def : InstRW<[GenericWriteJumpAndLink], (instrs BALC, BEQZALC, BGEZALC,
                                         BGTZALC, BLEZALC, BLTZALC,
                                         BNEZALC,
                                         JIALC)>;

def : InstRW<[GenericWriteJump], (instrs BC, BC2EQZ, BC2NEZ, BEQC, BEQZC, BGEC,
                                  BGEUC, BGEZC, BGTZC, BLEZC, BLTC, BLTUC,
                                  BLTZC, BNEC, BNEZC, BNVC, BOVC, JIC, JR_HB_R6,
                                  SIGRIE, PseudoIndirectBranchR6,
                                  PseudoIndrectHazardBranchR6)>;

def : InstRW<[GenericWriteJump], (instrs TAILCALLR6REG, TAILCALLHBR6REG)>;

def : InstRW<[GenericWriteTrap], (instrs SDBBP_R6)>;


// FGPU64
// ======

def : InstRW<[GenericWriteJump], (instrs BEQ64, BGEZ64, BGTZ64, BLEZ64,
                                  BLTZ64, BNE64, JR64)>;

def : InstRW<[GenericWriteJumpAndLink], (instrs JALR64, JALR64Pseudo,
                                         JALRHB64Pseudo, JALR_HB64)>;

def : InstRW<[GenericWriteJump], (instrs JR_HB64, TAILCALLREG64,
                                  TAILCALLREGHB64, PseudoReturn64)>;

// FGPU64R6
// ========

def : InstRW<[GenericWriteJump], (instrs BEQC64, BEQZC64, BGEC64, BGEUC64,
                                  BGEZC64, BGTZC64, BLEZC64, BLTC64, BLTUC64,
                                  BLTZC64, BNEC64, BNEZC64, JIC64,
                                  PseudoIndirectBranch64,
                                  PseudoIndirectHazardBranch64)>;

def : InstRW<[GenericWriteJumpAndLink], (instrs JIALC64)>;

def : InstRW<[GenericWriteJump], (instrs JR_HB64_R6, TAILCALL64R6REG,
                                  TAILCALLHB64R6REG, PseudoIndirectBranch64R6,
                                  PseudoIndrectHazardBranch64R6)>;

// COP0 Pipeline
// =============

def GenericCOP0 : ProcResource<1> { let BufferSize = 1; }

def GenericIssueCOP0 : ProcResource<1> { let Super = GenericCOP0; }
def GenericWriteCOP0TLB : SchedWriteRes<[GenericIssueCOP0]> { let Latency = 4; }
def GenericWriteCOP0 : SchedWriteRes<[GenericIssueCOP0]> { let Latency = 3; }
def GenericReadCOP0 : SchedWriteRes<[GenericIssueCOP0]> { let Latency = 2; }
def GenericReadWritePGPR : SchedWriteRes<[GenericIssueCOP0]>;
def GenericReadWriteCOP0Long : SchedWriteRes<[GenericIssueCOP0]> {
  let Latency = 5;
}
def GenericWriteCOP0Short : SchedWriteRes<[GenericIssueCOP0]>;

def : InstRW<[GenericWriteCOP0TLB], (instrs TLBP, TLBR, TLBWI, TLBWR)>;
def : InstRW<[GenericWriteCOP0TLB], (instrs TLBINV, TLBINVF)>;

def : InstRW<[GenericReadCOP0], (instrs MFC0)>;
def : InstRW<[GenericWriteCOP0], (instrs MTC0)>;

def : InstRW<[GenericWriteCOP0], (instrs EVP, DVP)>;

def : InstRW<[GenericWriteCOP0], (instrs DI, EI)>;

def : InstRW<[GenericWriteCOP0], (instrs EHB, PAUSE, WAIT)>;


// FGPU64
// ======

def : InstRW<[GenericReadCOP0], (instrs DMFC0)>;

def : InstRW<[GenericWriteCOP0], (instrs DMTC0)>;


def GenericCOP2 : ProcResource<1> { let BufferSize = 1; }
def GenericWriteCOPOther : SchedWriteRes<[GenericCOP2]>;

def : InstRW<[GenericWriteCOPOther], (instrs MFC2, MTC2)>;

def : InstRW<[GenericWriteCOPOther], (instrs DMFC2, DMTC2)>;



// FGPU MT ASE - hasMT
// ====================

def : InstRW<[GenericWriteMove], (instrs DMT, DVPE, EMT, EVPE, MFTR,
                                  MTTR)>;

def : InstRW<[GenericReadWriteCOP0Long], (instrs YIELD)>;

def : InstRW<[GenericWriteCOP0Short], (instrs FORK)>;

// FGPU Virtualization ASE
// =======================

def : InstRW<[GenericWriteCOP0Short], (instrs HYPCALL, TLBGINV, TLBGINVF, TLBGP,
                                       TLBGR, TLBGWI, TLBGWR, MFGC0, MFHGC0,
                                       MTGC0, MTHGC0)>;

// FGPU64 Virtualization ASE
// =========================

def : InstRW<[GenericWriteCOP0Short], (instrs DMFGC0, DMTGC0)>;

// LDST Pipeline
// -------------

def GenericWriteLoad : SchedWriteRes<[GenericIssueLDST]> {
  let Latency = 2;
}

def GenericWritePref : SchedWriteRes<[GenericIssueLDST]>;
def GenericWriteSync : SchedWriteRes<[GenericIssueLDST]>;
def GenericWriteCache : SchedWriteRes<[GenericIssueLDST]> { let Latency = 5; }

def GenericWriteStore : SchedWriteRes<[GenericIssueLDST]>;
def GenericWriteStoreSC : SchedWriteRes<[GenericIssueLDST]> { let Latency = 2; }

def GenericWriteGPRFromBypass : SchedWriteRes<[GenericIssueLDST]> {
  let Latency = 2;
}

def GenericWriteStoreFromOtherUnits : SchedWriteRes<[GenericIssueLDST]>;
def GenericWriteLoadToOtherUnits : SchedWriteRes<[GenericIssueLDST]> {
  let Latency = 0;
}

// l[bhw], l[bh]u, ll
def : InstRW<[GenericWriteLoad], (instrs LB, LBu, LH, LHu, LW, LL,
                                  LWC2, LWC3, LDC2, LDC3)>;

// lw[lr]
def : InstRW<[GenericWriteLoad], (instrs LWL, LWR)>;

// s[bhw], sc, s[dw]c[23]
def : InstRW<[GenericWriteStore], (instrs SB, SH, SW, SWC2, SWC3,
                                   SDC2, SDC3)>;

// PreFGPUR6 sw[lr]
def : InstRW<[GenericWriteStore], (instrs SWL, SWR)>;

def : InstRW<[GenericWriteStoreSC], (instrs SC)>;

// pref
def : InstRW<[GenericWritePref], (instrs PREF)>;
// cache
def : InstRW<[GenericWriteCache], (instrs CACHE)>;

// sync
def : InstRW<[GenericWriteSync], (instrs SYNC, SYNCI)>;

// FGPUR6
// ======

def : InstRW<[GenericWriteLoad], (instrs LDC2_R6, LL_R6, LWC2_R6, LWPC)>;

def : InstRW<[GenericWriteStore], (instrs SWC2_R6,  SDC2_R6)>;

def : InstRW<[GenericWriteStoreSC], (instrs SC_R6)>;

def : InstRW<[GenericWritePref], (instrs PREF_R6)>;

def : InstRW<[GenericWriteCache], (instrs CACHE_R6)>;

def : InstRW<[GenericWriteSync], (instrs GINVI, GINVT)>;

// FGPU32 EVA
// ==========

def : InstRW<[GenericWriteLoad], (instrs LBE, LBuE, LHE, LHuE, LWE,
                                  LLE)>;

def : InstRW<[GenericWriteStore], (instrs SBE, SHE, SWE, SCE)>;

def : InstRW<[GenericWriteLoad], (instrs LWLE, LWRE)>;

def : InstRW<[GenericWriteStore], (instrs SWLE, SWRE)>;

def : InstRW<[GenericWritePref], (instrs PREFE)>;

def : InstRW<[GenericWriteCache], (instrs CACHEE)>;


// FGPU64
// ======

def : InstRW<[GenericWriteLoad], (instrs LD, LL64, LLD, LWu, LB64, LBu64,
                                  LH64, LHu64, LW64)>;

// l[dw][lr]
def : InstRW<[GenericWriteLoad], (instrs LWL64, LWR64, LDL, LDR)>;

def : InstRW<[GenericWriteStore], (instrs SD, SC64, SCD, SB64, SH64, SW64,
                                   SWL64, SWR64)>;

def : InstRW<[GenericWriteStore], (instrs SDL, SDR)>;

// FGPU64R6
// ========

def : InstRW<[GenericWriteLoad], (instrs LWUPC, LDPC)>;

def : InstRW<[GenericWriteLoad], (instrs LLD_R6, LL64_R6)>;

def : InstRW<[GenericWriteStoreSC], (instrs SC64_R6, SCD_R6)>;

// FGPUR6 CRC ASE - hasCRC
// =======================

def : InstRW<[GenericWriteALU], (instrs CRC32B, CRC32H, CRC32W, CRC32CB,
                                 CRC32CH, CRC32CW)>;

// FGPU64R6 CRC ASE - hasCRC
// -------------------------

def : InstRW<[GenericWriteALU], (instrs CRC32D, CRC32CD)>;


// Cavium Networks FGPU (cnFGPU) - Octeon, HasCnFgpu
// =================================================

def : InstRW<[GenericWriteALU], (instrs BADDu, BBIT0, BBIT032, BBIT1, BBIT132,
                                 CINS, CINS32, CINS64_32, CINS_i32,
                                 DMFC2_OCTEON, DMTC2_OCTEON, DPOP, EXTS,
                                 EXTS32, MTM0, MTM1, MTM2, MTP0, MTP1, MTP2,
                                 POP, SEQ, SEQi, SNE, SNEi,
                                 V3MULU, VMM0, VMULU)>;

def : InstRW<[GenericWriteMDUtoGPR], (instrs DMUL)>;

// Cavium Networks FGPU (cnFGPUP) - Octeon+, HasCnFgpuP
// =================================================

def : InstRW<[GenericWriteALU], (instrs SAA, SAAD)>;

// FPU Pipelines
// =============

def GenericFPQ : ProcResource<1> { let BufferSize = 1; }
def GenericIssueFPUS : ProcResource<1> { let Super = GenericFPQ; }
def GenericIssueFPUL : ProcResource<1> { let Super = GenericFPQ; }
def GenericIssueFPULoad : ProcResource<1> { let Super = GenericFPQ; }
def GenericIssueFPUStore : ProcResource<1> { let Super = GenericFPQ; }
def GenericIssueFPUMove : ProcResource<1> { let Super = GenericFPQ; }
def GenericFPUDivSqrt : ProcResource<1> { let Super = GenericFPQ; }

// The floating point compare of the 24k series including interAptiv has a
// listed latency of 1-2. Using the higher latency here.

def GenericWriteFPUCmp : SchedWriteRes<[GenericIssueFPUS]> { let Latency = 2; }
def GenericWriteFPUS : SchedWriteRes<[GenericIssueFPUS]> { let Latency = 4; }
def GenericWriteFPUL : SchedWriteRes<[GenericIssueFPUL]> { let Latency = 5; }
def GenericWriteFPUStore : SchedWriteRes<[GenericIssueFPUStore]> { let
  Latency = 1;
}
def GenericWriteFPULoad : SchedWriteRes<[GenericIssueFPULoad]> {
  let Latency = 2;
}
def GenericWriteFPUMoveFP : SchedWriteRes<[GenericIssueFPUMove]> {
  let Latency = 4;
}
def GenericWriteFPUMoveGPRFPU : SchedWriteRes<[GenericIssueFPUMove]> {
  let Latency = 2;
}
def GenericWriteFPUDivS : SchedWriteRes<[GenericFPUDivSqrt]> {
  let Latency = 17;
  let ResourceCycles = [ 14 ];
}
def GenericWriteFPUDivD : SchedWriteRes<[GenericFPUDivSqrt]> {
  let Latency = 32;
  let ResourceCycles = [ 29 ];
}
def GenericWriteFPURcpS : SchedWriteRes<[GenericFPUDivSqrt]> {
  let Latency = 13;
  let ResourceCycles = [ 10 ];
}
def GenericWriteFPURcpD : SchedWriteRes<[GenericFPUDivSqrt]> {
  let Latency = 25;
  let ResourceCycles = [ 21 ];
}
def GenericWriteFPURsqrtS : SchedWriteRes<[GenericFPUDivSqrt]> {
  let Latency = 17;
  let ResourceCycles = [ 14 ];
}
def GenericWriteFPURsqrtD : SchedWriteRes<[GenericFPUDivSqrt]> {
  let Latency = 32;
  let ResourceCycles = [ 29 ];
}
def GenericWriteFPUSqrtS : SchedWriteRes<[GenericFPUDivSqrt]> {
  let Latency = 17;
  let ResourceCycles = [ 14 ];
}
def GenericWriteFPUSqrtD : SchedWriteRes<[GenericFPUDivSqrt]> {
  let Latency = 29;
  let ResourceCycles = [ 29 ];
}

// Floating point compare and branch
// ---------------------------------
//
// c.<cc>.[ds], bc1[tf], bc1[tf]l
def : InstRW<[GenericWriteFPUCmp], (instrs FCMP_D32, FCMP_D64, FCMP_S32, BC1F,
                                    BC1T, BC1FL, BC1TL)>;

def : InstRW<[GenericWriteFPUCmp], (instregex "C_[A-Z]+_(S|D32|D64)$")>;

// Short Pipe
// ----------
//
// abs.[ds], abs.ps, add.[ds], neg.[ds], neg.ps, madd.s, msub.s, nmadd,s
// nmsub.s, sub.[ds], mul.s

def : InstRW<[GenericWriteFPUS], (instrs FABS_S, FABS_D32, FABS_D64, FADD_D32,
                                  FADD_D64, FADD_S, MADD_S, MSUB_S, FMUL_S,
                                  FNEG_S, FNEG_D32, FNEG_D64, NMADD_S, NMSUB_S,
                                  FSUB_S, FSUB_D32, FSUB_D64)>;

// Long Pipe
// ----------
//
// nmadd.d, nmsub.d, mul.[ds], mul.ps, ceil.[wl].[sd], cvt.d.[sw], cvt.s.[dw],
// cvt.w.[sd], cvt.[sw].ps, trunc.w.[ds], trunc.w.ps, floor.[ds],
// round.[lw].[ds], floor.[lw].ds

// madd.d, msub.dm mul.d, mul.ps, nmadd.d, nmsub.d, ceil.[wl].[sd], cvt.d.[sw],
// cvt.s.[dw], cvt.w.[sd], cvt.[sw].ps, round.[lw].[ds], floor.[lw].ds,
// trunc.w.[ds], trunc.w.ps,
def : InstRW<[GenericWriteFPUL], (instrs ADDR_PS64,
                                  CEIL_L_D64, CEIL_L_S, CEIL_W_D32,
                                  CEIL_W_D64, CEIL_W_S, CVT_D32_S, CVT_D32_W,
                                  CVT_D64_L, CVT_D64_S, CVT_D64_W, CVT_L_D64,
                                  CVT_L_S, CVT_S_D32, CVT_S_D64, CVT_S_L,
                                  CVT_S_W, CVT_W_D32, CVT_W_D64, CVT_W_S,
                                  CVT_PS_S64, CVT_S_PL64, CVT_S_PU64,
                                  CVT_PS_PW64, CVT_PW_PS64, FADD_PS64,
                                  FLOOR_L_D64, FLOOR_L_S, FLOOR_W_D32,
                                  FLOOR_W_D64, FLOOR_W_S, FMUL_D32, FMUL_D64,
                                  FMUL_PS64, FSUB_PS64, MADD_D32, MADD_D64,
                                  MSUB_D32, MSUB_D64, MULR_PS64,
                                  NMADD_D32, NMADD_D64, NMSUB_D32, NMSUB_D64,
                                  PLL_PS64, PLU_PS64, PUL_PS64, PUU_PS64,
                                  ROUND_L_D64, ROUND_L_S, ROUND_W_D32,
                                  ROUND_W_D64, ROUND_W_S, TRUNC_L_D64,
                                  TRUNC_L_S, TRUNC_W_D32, TRUNC_W_D64,
                                  TRUNC_W_S, PseudoTRUNC_W_D,
                                  PseudoTRUNC_W_D32, PseudoTRUNC_W_S)>;

// Pseudo convert instruction
def : InstRW<[GenericWriteFPUL], (instrs PseudoCVT_D32_W, PseudoCVT_D64_L,
                                  PseudoCVT_D64_W, PseudoCVT_S_L,
                                  PseudoCVT_S_W)>;

// div.[ds], div.ps
def : InstRW<[GenericWriteFPUDivS], (instrs FDIV_S)>;
def : InstRW<[GenericWriteFPUDivD], (instrs FDIV_D32, FDIV_D64)>;

// sqrt.[ds], sqrt.ps
def : InstRW<[GenericWriteFPUSqrtS], (instrs FSQRT_S)>;
def : InstRW<[GenericWriteFPUSqrtD], (instrs FSQRT_D32, FSQRT_D64)>;

// rsqrt.[ds], recip.[ds]
def : InstRW<[GenericWriteFPURcpS], (instrs RECIP_S, RSQRT_S)>;
def : InstRW<[GenericWriteFPURcpD], (instrs RECIP_D32, RECIP_D64,
                                     RSQRT_D32, RSQRT_D64)>;


// Load Pipe
// ---------

// ctc1, mtc1, mthc1, cfc1, mfc1, mfhc1
def : InstRW<[GenericWriteFPUMoveGPRFPU], (instrs BuildPairF64,
                                           BuildPairF64_64, ExtractElementF64,
                                           ExtractElementF64_64, CFC1, CTC1,
                                           MFC1, MFC1_D64, MFHC1_D32,
                                           MFHC1_D64, MTC1, MTC1_D64,
                                           MTHC1_D32, MTHC1_D64)>;

// swc1, swxc1
def : InstRW<[GenericWriteFPUStore], (instrs SDC1, SDC164, SDXC1, SDXC164,
                                      SUXC1, SUXC164, SWC1, SWXC1)>;

def : InstRW<[GenericWriteFPUMoveFP], (instrs FMOV_D32, FMOV_D64, FMOV_S)>;


// movn.[ds], movz.[ds]
def : InstRW<[GenericWriteFPUMoveFP], (instrs MOVF_I, MOVF_D32, MOVF_D64,
                                       MOVF_S, MOVT_I, MOVT_D32, MOVT_D64,
                                       MOVT_S, MOVN_I_D32, MOVN_I_D64,
                                       MOVN_I_S, MOVZ_I_D32, MOVZ_I_D64,
                                       MOVZ_I_S)>;

def : InstRW<[GenericWriteFPUMoveFP], (instrs MOVT_I64, MOVF_I64, MOVZ_I64_S,
                                       MOVN_I64_D64, MOVN_I64_S,
                                       MOVZ_I64_D64)>;

// l[dw]x?c1
def : InstRW<[GenericWriteFPULoad], (instrs LDC1, LDC164, LDXC1, LDXC164,
                                     LUXC1, LUXC164, LWC1, LWXC1)>;

// FGPUR6
// ======

// sel(eq|ne).[ds], max.[ds], maxa.[ds], min.[ds], mina.[ds], class.[ds]
def : InstRW<[GenericWriteFPUS], (instrs SELEQZ_S, SELNEZ_S, SELEQZ_D, SELNEZ_D,
                                  MAX_S, MAX_D, MAXA_S, MAXA_D, MIN_S, MIN_D,
                                  MINA_S, MINA_D, CLASS_S, CLASS_D)>;

def : InstRW<[GenericWriteFPUL], (instrs RINT_S, RINT_D)>;

def : InstRW<[GenericWriteFPUCmp], (instrs BC1EQZ, BC1NEZ, SEL_D, SEL_S)>;

def : InstRW<[GenericWriteFPUS], (instrs MADDF_S, MSUBF_S, MADDF_D, MSUBF_D)>;

// FGPU64
// ======

def : InstRW<[GenericWriteFPUMoveGPRFPU], (instrs DMFC1, DMTC1)>;

// FGPU DSP ASE, HasDSP
// ====================

def : InstRW<[GenericWriteStore], (instrs SWDSP)>;

def : InstRW<[GenericWriteLoad], (instrs LWDSP)>;

def : InstRW<[GenericWriteMove], (instrs PseudoMTLOHI_DSP)>;

def GenericDSP : ProcResource<1> { let BufferSize = 1; }
def GenericDSPShort : SchedWriteRes<[GenericDSP]> { let Latency = 2; }
def GenericDSPLong : SchedWriteRes<[GenericDSP]> { let Latency = 6; }
def GenericDSPBypass : SchedWriteRes<[GenericDSP]> { let Latency = 1; }
def GenericDSPMTHILO : SchedWriteRes<[GenericDSP]> { let Latency = 5; }
def GenericDSPLoad : SchedWriteRes<[GenericDSP]> { let Latency = 4; }
def GenericDSPMTHLIP : SchedWriteRes<[GenericDSP]> { let Latency = 5; }

def : InstRW<[GenericDSPLong], (instregex "^EXTRV_RS_W$")>;
def : InstRW<[GenericDSPLong], (instregex "^EXTRV_R_W$")>;
def : InstRW<[GenericDSPLong], (instregex "^EXTRV_S_H$")>;
def : InstRW<[GenericDSPLong], (instregex "^EXTRV_W$")>;
def : InstRW<[GenericDSPLong], (instregex "^EXTR_RS_W$")>;
def : InstRW<[GenericDSPLong], (instregex "^EXTR_R_W$")>;
def : InstRW<[GenericDSPLong], (instregex "^EXTR_S_H$")>;
def : InstRW<[GenericDSPLong], (instregex "^EXTR_W$")>;
def : InstRW<[GenericDSPLong], (instregex "^INSV$")>;

def : InstRW<[GenericDSPMTHLIP], (instregex "^MTHLIP$")>;
def : InstRW<[GenericDSPMTHILO], (instregex "^MTHI_DSP$")>;
def : InstRW<[GenericDSPMTHILO], (instregex "^MTLO_DSP$")>;

def : InstRW<[GenericDSPShort], (instregex "^ABSQ_S_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^ABSQ_S_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDQ_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDQ_S_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDQ_S_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDSC$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDU_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDU_S_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDWC$")>;
def : InstRW<[GenericDSPShort], (instregex "^BITREV$")>;
def : InstRW<[GenericDSPShort], (instregex "^BPOSGE32$")>;
def : InstRW<[GenericDSPShort], (instregex "^CMPGU_EQ_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^CMPGU_LE_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^CMPGU_LT_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^CMPU_EQ_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^CMPU_LE_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^CMPU_LT_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^CMP_EQ_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^CMP_LE_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^CMP_LT_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPAQ_SA_L_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPAQ_S_W_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPAU_H_QBL$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPAU_H_QBR$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPSQ_SA_L_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPSQ_S_W_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPSU_H_QBL$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPSU_H_QBR$")>;
def : InstRW<[GenericDSPShort], (instregex "^EXTPDPV$")>;
def : InstRW<[GenericDSPShort], (instregex "^EXTPDP$")>;
def : InstRW<[GenericDSPShort], (instregex "^EXTPV$")>;
def : InstRW<[GenericDSPShort], (instregex "^EXTP$")>;
def : InstRW<[GenericDSPShort], (instregex "^LBUX$")>;
def : InstRW<[GenericDSPShort], (instregex "^LHX$")>;
def : InstRW<[GenericDSPShort], (instregex "^LWX$")>;
def : InstRW<[GenericDSPShort], (instregex "^MADDU_DSP$")>;
def : InstRW<[GenericDSPShort], (instregex "^MADD_DSP$")>;
def : InstRW<[GenericDSPShort], (instregex "^MAQ_SA_W_PHL$")>;
def : InstRW<[GenericDSPShort], (instregex "^MAQ_SA_W_PHR$")>;
def : InstRW<[GenericDSPShort], (instregex "^MAQ_S_W_PHL$")>;
def : InstRW<[GenericDSPShort], (instregex "^MAQ_S_W_PHR$")>;
def : InstRW<[GenericDSPShort], (instregex "^MFHI_DSP$")>;
def : InstRW<[GenericDSPShort], (instregex "^MFLO_DSP$")>;
def : InstRW<[GenericDSPShort], (instregex "^MODSUB$")>;
def : InstRW<[GenericDSPShort], (instregex "^MSUBU_DSP$")>;
def : InstRW<[GenericDSPShort], (instregex "^MSUB_DSP$")>;
def : InstRW<[GenericDSPShort], (instregex "^MULEQ_S_W_PHL$")>;
def : InstRW<[GenericDSPShort], (instregex "^MULEQ_S_W_PHR$")>;
def : InstRW<[GenericDSPShort], (instregex "^MULEU_S_PH_QBL$")>;
def : InstRW<[GenericDSPShort], (instregex "^MULEU_S_PH_QBR$")>;
def : InstRW<[GenericDSPShort], (instregex "^MULQ_RS_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^MULSAQ_S_W_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^MULTU_DSP$")>;
def : InstRW<[GenericDSPShort], (instregex "^MULT_DSP$")>;
def : InstRW<[GenericDSPShort], (instregex "^PACKRL_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^PICK_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^PICK_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECEQU_PH_QBLA$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECEQU_PH_QBL$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECEQU_PH_QBRA$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECEQU_PH_QBR$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECEQ_W_PHL$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECEQ_W_PHR$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECEU_PH_QBLA$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECEU_PH_QBL$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECEU_PH_QBRA$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECEU_PH_QBR$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECRQU_S_QB_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECRQ_PH_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECRQ_QB_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECRQ_RS_PH_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^RADDU_W_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^RDDSP$")>;
def : InstRW<[GenericDSPShort], (instregex "^REPLV_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^REPLV_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^REPL_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^REPL_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHILOV$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHILO$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHLLV_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHLLV_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHLLV_S_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHLLV_S_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHLL_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHLL_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHLL_S_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHLL_S_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHRAV_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHRAV_R_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHRAV_R_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHRA_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHRA_R_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHRA_R_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHRLV_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHRL_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^SUBQ_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SUBQ_S_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SUBQ_S_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^SUBU_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^SUBU_S_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^WRDSP$")>;

def : InstRW<[GenericDSPShort],
             (instregex "^Pseudo(CMP|CMPU)_(EQ|LE|LT)_(PH|QB)$")>;
def : InstRW<[GenericDSPShort],
						 (instregex "^PseudoPICK_(PH|QB)$")>;

// FGPU DSP R2 - hasDSP, HasDSPR2, InMicroFgpu
// ===========================================

def : InstRW<[GenericDSPShort], (instregex "^ABSQ_S_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDQH_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDQH_R_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDQH_R_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDQH_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDUH_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDUH_R_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDU_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^ADDU_S_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^APPEND$")>;
def : InstRW<[GenericDSPShort], (instregex "^BALIGN$")>;
def : InstRW<[GenericDSPShort], (instregex "^CMPGDU_EQ_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^CMPGDU_LE_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^CMPGDU_LT_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPA_W_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPAQX_SA_W_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPAQX_S_W_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPAX_W_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPS_W_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPSQX_S_W_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPSQX_SA_W_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^DPSX_W_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^MUL_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^MUL_S_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^MULQ_RS_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^MULQ_S_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^MULQ_S_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^MULSA_W_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECR_QB_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECR_SRA_PH_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^PRECR_SRA_R_PH_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^PREPEND$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHRA_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHRA_R_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHRAV_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHRAV_R_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHRL_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SHRLV_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SUBQH_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SUBQH_R_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SUBQH_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^SUBQH_R_W$")>;
def : InstRW<[GenericDSPShort], (instregex "^SUBU_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SUBU_S_PH$")>;
def : InstRW<[GenericDSPShort], (instregex "^SUBUH_QB$")>;
def : InstRW<[GenericDSPShort], (instregex "^SUBUH_R_QB$")>;

// FGPU MSA ASE - hasMSA
// =====================

def GenericWriteMSAShortLogic : SchedWriteRes<[GenericIssueFPUS]>;
def GenericWriteMSAShortInt : SchedWriteRes<[GenericIssueFPUS]> {
let Latency = 2;
}
def GenericWriteMoveOtherUnitsToFPU : SchedWriteRes<[GenericIssueFPUS]>;
def GenericWriteMSAOther3 : SchedWriteRes<[GenericIssueFPUS]> {
let Latency = 3;
}
def GenericWriteMSALongInt : SchedWriteRes<[GenericIssueFPUS]> {
let Latency = 5;
}
def GenericWriteFPUDivI : SchedWriteRes<[GenericFPQ]> {
  let Latency = 33;
  let ResourceCycles = [ 33 ];
}

// FPUS is also used in moves from floating point and MSA registers to general
// purpose registers.
def GenericWriteMoveFPUSToOtherUnits : SchedWriteRes<[GenericIssueFPUS]> {
  let Latency = 0;
}

// FPUL is also used in moves from floating point and MSA registers to general
// purpose registers.
def GenericWriteMoveFPULToOtherUnits : SchedWriteRes<[GenericIssueFPUL]>;


// adds_a.[bhwd], adds_[asu].[bhwd], addvi?.[bhwd], asub_[us].[bhwd],
// aver?_[us].[bhwd]
def : InstRW<[GenericWriteMSAShortInt], (instregex "^ADD_A_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^ADDS_[ASU]_[BHWD]$")>;

// TODO: ADDVI_[BHW] might be 1 cycle latency rather than 2. Need to confirm it.
// add.[bhwd], addvi.[bhwd], asub_[us].[bhwd], ave.[bhwd], aver.[bhwd]
def : InstRW<[GenericWriteMSAShortInt], (instregex "^ADDVI?_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^ASUB_[US].[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^AVER?_[US].[BHWD]$")>;

// and.v, andi.b, move.v, ldi.[bhwd], xor.v, nor.v, xori.b, nori.b, lsa
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^MOVE_V$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^LDI_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instrs LSA)>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(AND|OR|[XN]OR)_V$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(AND|OR|[XN]OR)I_B$")>;
def : InstRW<[GenericWriteMSAShortLogic],
             (instregex "^(AND|OR|[XN]OR)_V_[DHW]_PSEUDO$")>;

// vshf.[bhwd], binsl.[bhwd], binsr.[bhwd], insert.[bhwd], sld?.[bhwd],
// bset.[bhwd], bclr.[bhwd], bneg.[bhwd], bsel_v, bseli_b
def : InstRW<[GenericWriteMSAShortInt], (instregex "^VSHF_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^(BINSL|BINSLI)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^(BINSR|BINSRI)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^INSERT_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^(SLD|SLDI)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^(BSET|BSETI)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^(BCLR|BCLRI)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^(BNEG|BNEGI)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^(BSEL_V|BSELI_B)$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^BMN*Z.*$")>;
def : InstRW<[GenericWriteMSAShortInt],
             (instregex "^BSEL_(H|W|D|FW|FD)_PSEUDO$")>;

// pcnt.[bhwd], sat_s.[bhwd], sat_u.[bhwd]
def : InstRW<[GenericWriteMSAOther3], (instregex "^PCNT_[BHWD]$")>;
def : InstRW<[GenericWriteMSAOther3], (instregex "^SAT_(S|U)_[BHWD]$")>;

// bnz.[bhwdv], cfcmsa, ctcmsa
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(BNZ|BZ)_[BHWDV]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^C(F|T)CMSA$")>;

// shf.[bhw], fill[bhwd], splat?.[bhwd]
def : InstRW<[GenericWriteMSAShortInt], (instregex "^SHF_[BHW]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^FILL_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^(SPLAT|SPLATI)_[BHWD]$")>;

// fexp2_w, fexp2_d
def : InstRW<[GenericWriteFPUS], (instregex "^FEXP2_(W|D)$")>;

// compare, converts, round to int, floating point truncate.
def : InstRW<[GenericWriteFPUS], (instregex "^(CLT|CLTI)_(S|U)_[BHWD]$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^(CLE|CLEI)_(S|U)_[BHWD]$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^(CEQ|CEQI)_[BHWD]$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_UN_(S|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_UEQ_(S|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_EQ_(S|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_LT_(S|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_ULT_(S|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_LE_(S|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_ULE_(S|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_F_(D|S)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_SAF_(D|S)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_SEQ_(D|S)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_SLE_(D|S)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_SLT_(D|S)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_SUEQ_(D|S)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_SULE_(D|S)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_SULT_(D|S)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^CMP_SUN_(D|S)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FS(AF|EQ|LT|LE|NE|OR)_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FSUEQ_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FSULE_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FSULT_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FSUNE_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FSUN_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FCAF_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FCEQ_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FCLE_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FCLT_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FCNE_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FCOR_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FCUEQ_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FCULE_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FCULT_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FCUNE_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FCUN_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FABS_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FFINT_(U|S)_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FFQL_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FFQR_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FTINT_(U|S)_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FRINT_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FTQ_(H|W)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FTRUNC_(U|S)_(W|D)$")>;

// fexdo.[hw], fexupl.[wd], fexupr.[wd]
def : InstRW<[GenericWriteFPUS], (instregex "^FEXDO_(H|W)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FEXUPL_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FEXUPR_(W|D)$")>;

// fclass.[wd], fmax.[wd], fmax_a.[wd], fmin.[wd], fmin_a.[wd], flog2.[wd]
def : InstRW<[GenericWriteFPUS], (instregex "^FCLASS_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FMAX_A_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FMAX_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FMIN_A_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FMIN_(W|D)$")>;
def : InstRW<[GenericWriteFPUS], (instregex "^FLOG2_(W|D)$")>;

// interleave right/left, interleave even/odd, insert
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(ILVR|ILVL)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(ILVEV|ILVOD)_[BHWD]$")>;

// subs_?.[bhwd], subsus_?.[bhwd], subsuu_?.[bhwd], subvi.[bhwd], subv.[bhwd],
def : InstRW<[GenericWriteMSAShortInt], (instregex "^SUBS_(S|U)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^SUBSUS_(S|U)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^SUBSUU_(S|U)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^SUBVI_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortInt], (instregex "^SUBV_[BHWD]$")>;

// mod_[su].[bhwd], div_[su].[bhwd]
def : InstRW<[GenericWriteFPUDivI], (instregex "^MOD_(S|U)_[BHWD]$")>;
def : InstRW<[GenericWriteFPUDivI], (instregex "^DIV_(S|U)_[BHWD]$")>;

// hadd_[su].[bhwd], hsub_[su].[bhwd], max_[sua].[bhwd], min_[sua].[bhwd],
// maxi_[su].[bhwd], mini_[su].[bhwd], sra?.[bhwd], srar?.[bhwd], srlr.[bhwd],
// sll?.[bhwd], pckev.[bhwd], pckod.[bhwd], nloc.[bhwd], nlzc.[bhwd],
// insve.[bhwd]
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^HADD_(S|U)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^HSUB_(S|U)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(MAX|MIN)_S_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(MAX|MIN)_U_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(MAX|MIN)_A_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic],
             (instregex "^(MAXI|MINI)_(S|U)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(SRA|SRAI)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(SRL|SRLI)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(SRAR|SRARI)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(SRLR|SRLRI)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(SLL|SLLI)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(PCKEV|PCKOD)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^(NLOC|NLZC)_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^INSVE_[BHWD]$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^INSERT_F(D|W)_PSEUDO$")>;
def : InstRW<[GenericWriteMSAShortLogic], (instregex "^FILL_F(D|W)_PSEUDO$")>;

// dpadd_?.[bhwd], dpsub_?.[bhwd], dotp_?.[bhwd], msubv.[bhwd], maddv.[bhwd]
// mulv.[bhwd].
def : InstRW<[GenericWriteMSALongInt], (instregex "^DPADD_(S|U)_[HWD]$")>;
def : InstRW<[GenericWriteMSALongInt], (instregex "^DPSUB_(S|U)_[HWD]$")>;
def : InstRW<[GenericWriteMSALongInt], (instregex "^DOTP_(S|U)_[HWD]$")>;
def : InstRW<[GenericWriteMSALongInt], (instregex "^MSUBV_[BHWD]$")>;
def : InstRW<[GenericWriteMSALongInt], (instregex "^MADDV_[BHWD]$")>;
def : InstRW<[GenericWriteMSALongInt], (instregex "^MULV_[BHWD]$")>;

// madd?.q.[hw], msub?.q.[hw], mul?.q.[hw]
def : InstRW<[GenericWriteMSALongInt], (instregex "^MADDR_Q_[HW]$")>;
def : InstRW<[GenericWriteMSALongInt], (instregex "^MADD_Q_[HW]$")>;
def : InstRW<[GenericWriteMSALongInt], (instregex "^MSUBR_Q_[HW]$")>;
def : InstRW<[GenericWriteMSALongInt], (instregex "^MSUB_Q_[HW]$")>;
def : InstRW<[GenericWriteMSALongInt], (instregex "^MULR_Q_[HW]$")>;
def : InstRW<[GenericWriteMSALongInt], (instregex "^MUL_Q_[HW]$")>;

// fadd.[dw], fmadd.[dw], fmul.[dw], frcp.[dw], frsqrt.[dw], fsqrt.[dw]
// fsub.[dw], fdiv.[dw]
def : InstRW<[GenericWriteFPUL], (instregex "^FADD_[DW]$")>;
def : InstRW<[GenericWriteFPUL], (instregex "^FMADD_[DW]$")>;
def : InstRW<[GenericWriteFPUL], (instregex "^FMSUB_[DW]$")>;
def : InstRW<[GenericWriteFPUL], (instregex "^FMUL_[DW]$")>;
def : InstRW<[GenericWriteFPUL], (instregex "^FRCP_[DW]$")>;
def : InstRW<[GenericWriteFPUL], (instregex "^FRSQRT_[DW]$")>;
def : InstRW<[GenericWriteFPUL], (instregex "^FSQRT_[DW]$")>;
def : InstRW<[GenericWriteFPUL], (instregex "^FSUB_[DW]$")>;
def : InstRW<[GenericWriteFPUL], (instregex "^FDIV_[DW]$")>;

// copy.[su]_[bhwd]
def : InstRW<[GenericWriteFPUMoveGPRFPU], (instregex "^COPY_U_[BHW]$")>;
def : InstRW<[GenericWriteFPUMoveGPRFPU], (instregex "^COPY_S_[BHWD]$")>;

def : InstRW<[GenericWriteFPUStore], (instregex "^ST_[BHWD]$")>;
def : InstRW<[GenericWriteFPUStore], (instrs ST_F16)>;
def : InstRW<[GenericWriteFPULoad], (instregex "^LD_[BHWD]$")>;
def : InstRW<[GenericWriteFPULoad], (instrs LD_F16)>;

// Atomic instructions

// FIXME: Define `WriteAtomic` in the FgpuSchedule.td and
// attach it to the Atomic2OpsPostRA, AtomicCmpSwapPostRA, ...
// classes. Then just define resources for the `WriteAtomic` in each
// machine models.
def GenericAtomic : ProcResource<1> { let BufferSize = 1; }
def GenericWriteAtomic : SchedWriteRes<[GenericAtomic]> { let Latency = 2; }

def : InstRW<[GenericWriteAtomic],
    (instregex "^ATOMIC_SWAP_I(8|16|32|64)_POSTRA$")>;
def : InstRW<[GenericWriteAtomic],
    (instregex "^ATOMIC_CMP_SWAP_I(8|16|32|64)_POSTRA$")>;
def : InstRW<[GenericWriteAtomic],
    (instregex "^ATOMIC_LOAD_(ADD|SUB|AND|OR|XOR|NAND|MIN|MAX|UMIN|UMAX)"
               "_I(8|16|32|64)_POSTRA$")>;
}
