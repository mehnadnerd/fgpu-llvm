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

  let CompleteModel = 0; // TODO: add in the other things so this can be complete, like vector
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

def : InstRW<[GenericWriteALU], (instrs ADD, ADDi, AND, ANDi,
                                 Li, LUi,
                                 NOR, OR, ORi, SLL, SLLi,
                                 SRA, SRAi, SRL, SRLi,
                                 SUB, XOR, XORi, MOVZ, MOVN, SLT, SLTi, SLTiu, SLTu,
                                 COPY, Li32, LEA_ADDiu)>;

def : InstRW<[GenericWriteALU], (instrs AADD, AMAX)>;

def GenericMul : SchedWriteRes<[GenericIssueALU]> {
  let Latency = 4;
}

def : InstRW<[GenericWriteALU], (instrs MUL, MACC)>; // MADD???

// openncl
def : InstRW<[GenericWriteALU], (instrs LID, LP, WGOFF, WGID, SIZE, WGSIZE)>;


// CTISTD Pipeline
// ---------------

def GenericIssueCTISTD : ProcResource<1> { let Super = GenericALU; }



// Load Store Pipeline
// =============
def GenericLDST : ProcResource<1> { let BufferSize = 1; }
def GenericIssueLDST : ProcResource<1> { let Super = GenericLDST; }

def GenericWriteJump : SchedWriteRes<[GenericIssueCTISTD]>;
def GenericWriteJumpAndLink : SchedWriteRes<[GenericIssueCTISTD]> {
  let Latency = 2;
}


def : InstRW<[GenericWriteJump], (instrs BEQ, BNE, JSUB,
                                           PDOM_BEQ, PDOM_BNE)>;

def : InstRW<[GenericWriteJumpAndLink], (instrs RET, RetLR)>;

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

// standard ld/st
def : InstRW<[GenericWriteLoad], (instrs LB, LH, LW, LWC,
                                         LBZ, LWZ, LWCZ)>;

def : InstRW<[GenericWriteStore], (instrs SB, SH, SW, SWC, SBZ, SWZ, SWCZ)>;

// local ld/st, shared ld/st
def : InstRW<[GenericWriteLoad], (instrs LLWI, SLW, SLWC, SLWCZ)>;
def : InstRW<[GenericWriteStore], (instrs LSWI, SSW, SSWC, SSWCZ)>;


// sync
def : InstRW<[GenericWriteSync], (instrs LSYNC, GSYNC)>;


// FPU Pipelines
// =============

def GenericFPQ : ProcResource<1> { let BufferSize = 1; }
def GenericIssueFPUS : ProcResource<1> { let Super = GenericFPQ; }
def GenericFPUDivSqrt : ProcResource<1> { let Super = GenericFPQ; }

// The floating point compare of the 24k series including interAptiv has a
// listed latency of 1-2. Using the higher latency here.

def GenericWriteFPUCmp : SchedWriteRes<[GenericIssueFPUS]> { let Latency = 2; }
def GenericWriteFPUS : SchedWriteRes<[GenericIssueFPUS]> { let Latency = 4; }
def GenericWriteFPUDivS : SchedWriteRes<[GenericFPUDivSqrt]> {
  let Latency = 17;
  let ResourceCycles = [ 14 ];
}
def GenericWriteFPURsqrtS : SchedWriteRes<[GenericFPUDivSqrt]> {
  let Latency = 17;
  let ResourceCycles = [ 14 ];
}

// Floating point compare and branch
// ---------------------------------
//
// c.<cc>.[ds], bc1[tf], bc1[tf]l


def : InstRW<[GenericWriteFPUCmp], (instrs FSLT)>; // just stuffing the uitofp in here, what is UITOFP ??
def : InstRW<[GenericWriteFPUS], (instrs FADD, FSUB, FMUL, FFMA)>;
//def : InstRW<[GenericWriteFPUDivS], (instrs FDIV)>; // uh where are these
//def : InstRW<[GenericWriteFPUsqrtS], (instrs FQRT)>;


}
