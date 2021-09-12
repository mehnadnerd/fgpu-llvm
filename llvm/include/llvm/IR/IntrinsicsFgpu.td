//===- IntrinsicsFgpu.td - Defines Fgpu intrinsics ---------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines all of the FGPU-specific intrinsics.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// FGPU DSP data types
def fgpu_v2q15_ty: LLVMType<v2i16>;
def fgpu_v4q7_ty: LLVMType<v4i8>;
def fgpu_q31_ty: LLVMType<i32>;

let TargetPrefix = "fgpu" in {  // All intrinsics start with "llvm.fgpu.".

//===----------------------------------------------------------------------===//
// FGPU DSP Rev 1

//===----------------------------------------------------------------------===//
// Addition/subtraction

def int_fgpu_addu_qb : GCCBuiltin<"__builtin_fgpu_addu_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty],
            [Commutative, IntrNoMem]>;
def int_fgpu_addu_s_qb : GCCBuiltin<"__builtin_fgpu_addu_s_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty],
            [Commutative, IntrNoMem]>;
def int_fgpu_subu_qb : GCCBuiltin<"__builtin_fgpu_subu_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty], [IntrNoMem]>;
def int_fgpu_subu_s_qb : GCCBuiltin<"__builtin_fgpu_subu_s_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty], [IntrNoMem]>;

def int_fgpu_addq_ph : GCCBuiltin<"__builtin_fgpu_addq_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty],
            [Commutative, IntrNoMem]>;
def int_fgpu_addq_s_ph : GCCBuiltin<"__builtin_fgpu_addq_s_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty],
            [Commutative, IntrNoMem]>;
def int_fgpu_subq_ph : GCCBuiltin<"__builtin_fgpu_subq_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty], [IntrNoMem]>;
def int_fgpu_subq_s_ph : GCCBuiltin<"__builtin_fgpu_subq_s_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty], [IntrNoMem]>;

def int_fgpu_madd: GCCBuiltin<"__builtin_fgpu_madd">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem, Commutative]>;
def int_fgpu_maddu: GCCBuiltin<"__builtin_fgpu_maddu">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem, Commutative]>;

def int_fgpu_msub: GCCBuiltin<"__builtin_fgpu_msub">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem]>;
def int_fgpu_msubu: GCCBuiltin<"__builtin_fgpu_msubu">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem]>;

def int_fgpu_addq_s_w: GCCBuiltin<"__builtin_fgpu_addq_s_w">,
  Intrinsic<[fgpu_q31_ty], [fgpu_q31_ty, fgpu_q31_ty], [Commutative]>;
def int_fgpu_subq_s_w: GCCBuiltin<"__builtin_fgpu_subq_s_w">,
  Intrinsic<[fgpu_q31_ty], [fgpu_q31_ty, fgpu_q31_ty], []>;

def int_fgpu_addsc: GCCBuiltin<"__builtin_fgpu_addsc">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty], [Commutative]>;
def int_fgpu_addwc: GCCBuiltin<"__builtin_fgpu_addwc">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty], [Commutative]>;

def int_fgpu_modsub: GCCBuiltin<"__builtin_fgpu_modsub">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty], [IntrNoMem]>;

def int_fgpu_raddu_w_qb: GCCBuiltin<"__builtin_fgpu_raddu_w_qb">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i8_ty], [IntrNoMem]>;

//===----------------------------------------------------------------------===//
// Absolute value

def int_fgpu_absq_s_ph: GCCBuiltin<"__builtin_fgpu_absq_s_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty], []>;
def int_fgpu_absq_s_w: GCCBuiltin<"__builtin_fgpu_absq_s_w">,
  Intrinsic<[fgpu_q31_ty], [fgpu_q31_ty], []>;

//===----------------------------------------------------------------------===//
// Precision reduce/expand

def int_fgpu_precrq_qb_ph: GCCBuiltin<"__builtin_fgpu_precrq_qb_ph">,
  Intrinsic<[llvm_v4i8_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty], [IntrNoMem]>;
def int_fgpu_precrqu_s_qb_ph: GCCBuiltin<"__builtin_fgpu_precrqu_s_qb_ph">,
  Intrinsic<[llvm_v4i8_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty], []>;
def int_fgpu_precrq_ph_w: GCCBuiltin<"__builtin_fgpu_precrq_ph_w">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_q31_ty, fgpu_q31_ty], [IntrNoMem]>;
def int_fgpu_precrq_rs_ph_w: GCCBuiltin<"__builtin_fgpu_precrq_rs_ph_w">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_q31_ty, fgpu_q31_ty], []>;
def int_fgpu_preceq_w_phl: GCCBuiltin<"__builtin_fgpu_preceq_w_phl">,
  Intrinsic<[fgpu_q31_ty], [fgpu_v2q15_ty], [IntrNoMem]>;
def int_fgpu_preceq_w_phr: GCCBuiltin<"__builtin_fgpu_preceq_w_phr">,
  Intrinsic<[fgpu_q31_ty], [fgpu_v2q15_ty], [IntrNoMem]>;
def int_fgpu_precequ_ph_qbl: GCCBuiltin<"__builtin_fgpu_precequ_ph_qbl">,
  Intrinsic<[fgpu_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;
def int_fgpu_precequ_ph_qbr: GCCBuiltin<"__builtin_fgpu_precequ_ph_qbr">,
  Intrinsic<[fgpu_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;
def int_fgpu_precequ_ph_qbla: GCCBuiltin<"__builtin_fgpu_precequ_ph_qbla">,
  Intrinsic<[fgpu_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;
def int_fgpu_precequ_ph_qbra: GCCBuiltin<"__builtin_fgpu_precequ_ph_qbra">,
  Intrinsic<[fgpu_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;
def int_fgpu_preceu_ph_qbl: GCCBuiltin<"__builtin_fgpu_preceu_ph_qbl">,
  Intrinsic<[fgpu_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;
def int_fgpu_preceu_ph_qbr: GCCBuiltin<"__builtin_fgpu_preceu_ph_qbr">,
  Intrinsic<[fgpu_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;
def int_fgpu_preceu_ph_qbla: GCCBuiltin<"__builtin_fgpu_preceu_ph_qbla">,
  Intrinsic<[fgpu_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;
def int_fgpu_preceu_ph_qbra: GCCBuiltin<"__builtin_fgpu_preceu_ph_qbra">,
  Intrinsic<[fgpu_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;

//===----------------------------------------------------------------------===//
// Shift

def int_fgpu_shll_qb: GCCBuiltin<"__builtin_fgpu_shll_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_i32_ty], []>;
def int_fgpu_shrl_qb: GCCBuiltin<"__builtin_fgpu_shrl_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_shll_ph: GCCBuiltin<"__builtin_fgpu_shll_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, llvm_i32_ty], []>;
def int_fgpu_shll_s_ph: GCCBuiltin<"__builtin_fgpu_shll_s_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, llvm_i32_ty], []>;
def int_fgpu_shra_ph: GCCBuiltin<"__builtin_fgpu_shra_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_shra_r_ph: GCCBuiltin<"__builtin_fgpu_shra_r_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_shll_s_w: GCCBuiltin<"__builtin_fgpu_shll_s_w">,
  Intrinsic<[fgpu_q31_ty], [fgpu_q31_ty, llvm_i32_ty], []>;
def int_fgpu_shra_r_w: GCCBuiltin<"__builtin_fgpu_shra_r_w">,
  Intrinsic<[fgpu_q31_ty], [fgpu_q31_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_shilo: GCCBuiltin<"__builtin_fgpu_shilo">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty], [IntrNoMem]>;

//===----------------------------------------------------------------------===//
// Multiplication

def int_fgpu_muleu_s_ph_qbl: GCCBuiltin<"__builtin_fgpu_muleu_s_ph_qbl">,
  Intrinsic<[fgpu_v2q15_ty], [llvm_v4i8_ty, fgpu_v2q15_ty], []>;
def int_fgpu_muleu_s_ph_qbr: GCCBuiltin<"__builtin_fgpu_muleu_s_ph_qbr">,
  Intrinsic<[fgpu_v2q15_ty], [llvm_v4i8_ty, fgpu_v2q15_ty], []>;
def int_fgpu_mulq_rs_ph: GCCBuiltin<"__builtin_fgpu_mulq_rs_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty], [Commutative]>;
def int_fgpu_muleq_s_w_phl: GCCBuiltin<"__builtin_fgpu_muleq_s_w_phl">,
  Intrinsic<[fgpu_q31_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty], [Commutative]>;
def int_fgpu_muleq_s_w_phr: GCCBuiltin<"__builtin_fgpu_muleq_s_w_phr">,
  Intrinsic<[fgpu_q31_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty], [Commutative]>;
def int_fgpu_mulsaq_s_w_ph: GCCBuiltin<"__builtin_fgpu_mulsaq_s_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, fgpu_v2q15_ty, fgpu_v2q15_ty], []>;
def int_fgpu_maq_s_w_phl: GCCBuiltin<"__builtin_fgpu_maq_s_w_phl">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, fgpu_v2q15_ty, fgpu_v2q15_ty], []>;
def int_fgpu_maq_s_w_phr: GCCBuiltin<"__builtin_fgpu_maq_s_w_phr">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, fgpu_v2q15_ty, fgpu_v2q15_ty], []>;
def int_fgpu_maq_sa_w_phl: GCCBuiltin<"__builtin_fgpu_maq_sa_w_phl">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, fgpu_v2q15_ty, fgpu_v2q15_ty], []>;
def int_fgpu_maq_sa_w_phr: GCCBuiltin<"__builtin_fgpu_maq_sa_w_phr">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, fgpu_v2q15_ty, fgpu_v2q15_ty], []>;
def int_fgpu_mult: GCCBuiltin<"__builtin_fgpu_mult">,
  Intrinsic<[llvm_i64_ty], [llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem, Commutative]>;
def int_fgpu_multu: GCCBuiltin<"__builtin_fgpu_multu">,
  Intrinsic<[llvm_i64_ty], [llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem, Commutative]>;

//===----------------------------------------------------------------------===//
// Dot product with accumulate/subtract

def int_fgpu_dpau_h_qbl: GCCBuiltin<"__builtin_fgpu_dpau_h_qbl">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v4i8_ty, llvm_v4i8_ty],
            [IntrNoMem]>;
def int_fgpu_dpau_h_qbr: GCCBuiltin<"__builtin_fgpu_dpau_h_qbr">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v4i8_ty, llvm_v4i8_ty],
            [IntrNoMem]>;
def int_fgpu_dpsu_h_qbl: GCCBuiltin<"__builtin_fgpu_dpsu_h_qbl">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v4i8_ty, llvm_v4i8_ty],
            [IntrNoMem]>;
def int_fgpu_dpsu_h_qbr: GCCBuiltin<"__builtin_fgpu_dpsu_h_qbr">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v4i8_ty, llvm_v4i8_ty],
            [IntrNoMem]>;
def int_fgpu_dpaq_s_w_ph: GCCBuiltin<"__builtin_fgpu_dpaq_s_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, fgpu_v2q15_ty, fgpu_v2q15_ty], []>;
def int_fgpu_dpsq_s_w_ph: GCCBuiltin<"__builtin_fgpu_dpsq_s_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, fgpu_v2q15_ty, fgpu_v2q15_ty], []>;
def int_fgpu_dpaq_sa_l_w: GCCBuiltin<"__builtin_fgpu_dpaq_sa_l_w">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, fgpu_q31_ty, fgpu_q31_ty], []>;
def int_fgpu_dpsq_sa_l_w: GCCBuiltin<"__builtin_fgpu_dpsq_sa_l_w">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, fgpu_q31_ty, fgpu_q31_ty], []>;

//===----------------------------------------------------------------------===//
// Comparison

def int_fgpu_cmpu_eq_qb: GCCBuiltin<"__builtin_fgpu_cmpu_eq_qb">,
  Intrinsic<[], [llvm_v4i8_ty, llvm_v4i8_ty], [Commutative]>;
def int_fgpu_cmpu_lt_qb: GCCBuiltin<"__builtin_fgpu_cmpu_lt_qb">,
  Intrinsic<[], [llvm_v4i8_ty, llvm_v4i8_ty], []>;
def int_fgpu_cmpu_le_qb: GCCBuiltin<"__builtin_fgpu_cmpu_le_qb">,
  Intrinsic<[], [llvm_v4i8_ty, llvm_v4i8_ty], []>;
def int_fgpu_cmpgu_eq_qb: GCCBuiltin<"__builtin_fgpu_cmpgu_eq_qb">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i8_ty, llvm_v4i8_ty], [Commutative]>;
def int_fgpu_cmpgu_lt_qb: GCCBuiltin<"__builtin_fgpu_cmpgu_lt_qb">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i8_ty, llvm_v4i8_ty], []>;
def int_fgpu_cmpgu_le_qb: GCCBuiltin<"__builtin_fgpu_cmpgu_le_qb">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i8_ty, llvm_v4i8_ty], []>;
def int_fgpu_cmp_eq_ph: GCCBuiltin<"__builtin_fgpu_cmp_eq_ph">,
  Intrinsic<[], [fgpu_v2q15_ty, fgpu_v2q15_ty], [Commutative]>;
def int_fgpu_cmp_lt_ph: GCCBuiltin<"__builtin_fgpu_cmp_lt_ph">,
  Intrinsic<[], [fgpu_v2q15_ty, fgpu_v2q15_ty], []>;
def int_fgpu_cmp_le_ph: GCCBuiltin<"__builtin_fgpu_cmp_le_ph">,
  Intrinsic<[], [fgpu_v2q15_ty, fgpu_v2q15_ty], []>;

//===----------------------------------------------------------------------===//
// Extracting

def int_fgpu_extr_s_h: GCCBuiltin<"__builtin_fgpu_extr_s_h">,
  Intrinsic<[llvm_i32_ty], [llvm_i64_ty, llvm_i32_ty], []>;
def int_fgpu_extr_w: GCCBuiltin<"__builtin_fgpu_extr_w">,
  Intrinsic<[llvm_i32_ty], [llvm_i64_ty, llvm_i32_ty], []>;
def int_fgpu_extr_rs_w: GCCBuiltin<"__builtin_fgpu_extr_rs_w">,
  Intrinsic<[llvm_i32_ty], [llvm_i64_ty, llvm_i32_ty], []>;
def int_fgpu_extr_r_w: GCCBuiltin<"__builtin_fgpu_extr_r_w">,
  Intrinsic<[llvm_i32_ty], [llvm_i64_ty, llvm_i32_ty], []>;
def int_fgpu_extp: GCCBuiltin<"__builtin_fgpu_extp">,
  Intrinsic<[llvm_i32_ty], [llvm_i64_ty, llvm_i32_ty], []>;
def int_fgpu_extpdp: GCCBuiltin<"__builtin_fgpu_extpdp">,
  Intrinsic<[llvm_i32_ty], [llvm_i64_ty, llvm_i32_ty], []>;

//===----------------------------------------------------------------------===//
// Misc

def int_fgpu_wrdsp: GCCBuiltin<"__builtin_fgpu_wrdsp">,
  Intrinsic<[], [llvm_i32_ty, llvm_i32_ty], [ImmArg<ArgIndex<1>>]>;
def int_fgpu_rddsp: GCCBuiltin<"__builtin_fgpu_rddsp">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrReadMem, ImmArg<ArgIndex<0>>]>;

def int_fgpu_insv: GCCBuiltin<"__builtin_fgpu_insv">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty], [IntrReadMem]>;
def int_fgpu_bitrev: GCCBuiltin<"__builtin_fgpu_bitrev">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrNoMem]>;

def int_fgpu_packrl_ph: GCCBuiltin<"__builtin_fgpu_packrl_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty], [IntrNoMem]>;

def int_fgpu_repl_qb: GCCBuiltin<"__builtin_fgpu_repl_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_repl_ph: GCCBuiltin<"__builtin_fgpu_repl_ph">,
  Intrinsic<[fgpu_v2q15_ty], [llvm_i32_ty], [IntrNoMem]>;

def int_fgpu_pick_qb: GCCBuiltin<"__builtin_fgpu_pick_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty], [IntrReadMem]>;
def int_fgpu_pick_ph: GCCBuiltin<"__builtin_fgpu_pick_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty], [IntrReadMem]>;

def int_fgpu_mthlip: GCCBuiltin<"__builtin_fgpu_mthlip">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty], []>;

def int_fgpu_bposge32: GCCBuiltin<"__builtin_fgpu_bposge32">,
  Intrinsic<[llvm_i32_ty], [], [IntrReadMem]>;

def int_fgpu_lbux: GCCBuiltin<"__builtin_fgpu_lbux">,
  Intrinsic<[llvm_i32_ty], [llvm_ptr_ty, llvm_i32_ty], [IntrReadMem, IntrArgMemOnly]>;
def int_fgpu_lhx: GCCBuiltin<"__builtin_fgpu_lhx">,
  Intrinsic<[llvm_i32_ty], [llvm_ptr_ty, llvm_i32_ty], [IntrReadMem, IntrArgMemOnly]>;
def int_fgpu_lwx: GCCBuiltin<"__builtin_fgpu_lwx">,
  Intrinsic<[llvm_i32_ty], [llvm_ptr_ty, llvm_i32_ty], [IntrReadMem, IntrArgMemOnly]>;

//===----------------------------------------------------------------------===//
// FGPU DSP Rev 2

def int_fgpu_absq_s_qb: GCCBuiltin<"__builtin_fgpu_absq_s_qb">,
  Intrinsic<[fgpu_v4q7_ty], [fgpu_v4q7_ty], []>;

def int_fgpu_addqh_ph: GCCBuiltin<"__builtin_fgpu_addqh_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty],
            [IntrNoMem, Commutative]>;
def int_fgpu_addqh_r_ph: GCCBuiltin<"__builtin_fgpu_addqh_r_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty],
            [IntrNoMem, Commutative]>;
def int_fgpu_addqh_w: GCCBuiltin<"__builtin_fgpu_addqh_w">,
  Intrinsic<[fgpu_q31_ty], [fgpu_q31_ty, fgpu_q31_ty],
            [IntrNoMem, Commutative]>;
def int_fgpu_addqh_r_w: GCCBuiltin<"__builtin_fgpu_addqh_r_w">,
  Intrinsic<[fgpu_q31_ty], [fgpu_q31_ty, fgpu_q31_ty],
            [IntrNoMem, Commutative]>;

def int_fgpu_addu_ph: GCCBuiltin<"__builtin_fgpu_addu_ph">,
  Intrinsic<[llvm_v2i16_ty], [llvm_v2i16_ty, llvm_v2i16_ty], [Commutative]>;
def int_fgpu_addu_s_ph: GCCBuiltin<"__builtin_fgpu_addu_s_ph">,
  Intrinsic<[llvm_v2i16_ty], [llvm_v2i16_ty, llvm_v2i16_ty], [Commutative]>;

def int_fgpu_adduh_qb: GCCBuiltin<"__builtin_fgpu_adduh_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty],
            [IntrNoMem, Commutative]>;
def int_fgpu_adduh_r_qb: GCCBuiltin<"__builtin_fgpu_adduh_r_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty],
            [IntrNoMem, Commutative]>;

def int_fgpu_append: GCCBuiltin<"__builtin_fgpu_append">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem, ImmArg<ArgIndex<2>>]>;
def int_fgpu_balign: GCCBuiltin<"__builtin_fgpu_balign">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem, ImmArg<ArgIndex<2>>]>;

def int_fgpu_cmpgdu_eq_qb: GCCBuiltin<"__builtin_fgpu_cmpgdu_eq_qb">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i8_ty, llvm_v4i8_ty], [Commutative]>;
def int_fgpu_cmpgdu_lt_qb: GCCBuiltin<"__builtin_fgpu_cmpgdu_lt_qb">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i8_ty, llvm_v4i8_ty], []>;
def int_fgpu_cmpgdu_le_qb: GCCBuiltin<"__builtin_fgpu_cmpgdu_le_qb">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i8_ty, llvm_v4i8_ty], []>;

def int_fgpu_dpa_w_ph: GCCBuiltin<"__builtin_fgpu_dpa_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v2i16_ty, llvm_v2i16_ty],
            [IntrNoMem]>;
def int_fgpu_dps_w_ph: GCCBuiltin<"__builtin_fgpu_dps_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v2i16_ty, llvm_v2i16_ty],
            [IntrNoMem]>;

def int_fgpu_dpaqx_s_w_ph: GCCBuiltin<"__builtin_fgpu_dpaqx_s_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, fgpu_v2q15_ty, fgpu_v2q15_ty], []>;
def int_fgpu_dpaqx_sa_w_ph: GCCBuiltin<"__builtin_fgpu_dpaqx_sa_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, fgpu_v2q15_ty, fgpu_v2q15_ty], []>;
def int_fgpu_dpax_w_ph: GCCBuiltin<"__builtin_fgpu_dpax_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v2i16_ty, llvm_v2i16_ty],
            [IntrNoMem]>;
def int_fgpu_dpsx_w_ph: GCCBuiltin<"__builtin_fgpu_dpsx_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v2i16_ty, llvm_v2i16_ty],
            [IntrNoMem]>;
def int_fgpu_dpsqx_s_w_ph: GCCBuiltin<"__builtin_fgpu_dpsqx_s_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, fgpu_v2q15_ty, fgpu_v2q15_ty], []>;
def int_fgpu_dpsqx_sa_w_ph: GCCBuiltin<"__builtin_fgpu_dpsqx_sa_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, fgpu_v2q15_ty, fgpu_v2q15_ty], []>;

def int_fgpu_mul_ph: GCCBuiltin<"__builtin_fgpu_mul_ph">,
  Intrinsic<[llvm_v2i16_ty], [llvm_v2i16_ty, llvm_v2i16_ty], [Commutative]>;
def int_fgpu_mul_s_ph: GCCBuiltin<"__builtin_fgpu_mul_s_ph">,
  Intrinsic<[llvm_v2i16_ty], [llvm_v2i16_ty, llvm_v2i16_ty], [Commutative]>;

def int_fgpu_mulq_rs_w: GCCBuiltin<"__builtin_fgpu_mulq_rs_w">,
  Intrinsic<[fgpu_q31_ty], [fgpu_q31_ty, fgpu_q31_ty], [Commutative]>;
def int_fgpu_mulq_s_ph: GCCBuiltin<"__builtin_fgpu_mulq_s_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty], [Commutative]>;
def int_fgpu_mulq_s_w: GCCBuiltin<"__builtin_fgpu_mulq_s_w">,
  Intrinsic<[fgpu_q31_ty], [fgpu_q31_ty, fgpu_q31_ty], [Commutative]>;
def int_fgpu_mulsa_w_ph: GCCBuiltin<"__builtin_fgpu_mulsa_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v2i16_ty, llvm_v2i16_ty],
            [IntrNoMem]>;

def int_fgpu_precr_qb_ph: GCCBuiltin<"__builtin_fgpu_precr_qb_ph">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v2i16_ty, llvm_v2i16_ty], []>;
def int_fgpu_precr_sra_ph_w: GCCBuiltin<"__builtin_fgpu_precr_sra_ph_w">,
  Intrinsic<[llvm_v2i16_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;
def int_fgpu_precr_sra_r_ph_w: GCCBuiltin<"__builtin_fgpu_precr_sra_r_ph_w">,
  Intrinsic<[llvm_v2i16_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;

def int_fgpu_prepend: GCCBuiltin<"__builtin_fgpu_prepend">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem, ImmArg<ArgIndex<2>>]>;

def int_fgpu_shra_qb: GCCBuiltin<"__builtin_fgpu_shra_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_shra_r_qb: GCCBuiltin<"__builtin_fgpu_shra_r_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_shrl_ph: GCCBuiltin<"__builtin_fgpu_shrl_ph">,
  Intrinsic<[llvm_v2i16_ty], [llvm_v2i16_ty, llvm_i32_ty], [IntrNoMem]>;

def int_fgpu_subqh_ph: GCCBuiltin<"__builtin_fgpu_subqh_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty], [IntrNoMem]>;
def int_fgpu_subqh_r_ph: GCCBuiltin<"__builtin_fgpu_subqh_r_ph">,
  Intrinsic<[fgpu_v2q15_ty], [fgpu_v2q15_ty, fgpu_v2q15_ty], [IntrNoMem]>;
def int_fgpu_subqh_w: GCCBuiltin<"__builtin_fgpu_subqh_w">,
  Intrinsic<[fgpu_q31_ty], [fgpu_q31_ty, fgpu_q31_ty], [IntrNoMem]>;
def int_fgpu_subqh_r_w: GCCBuiltin<"__builtin_fgpu_subqh_r_w">,
  Intrinsic<[fgpu_q31_ty], [fgpu_q31_ty, fgpu_q31_ty], [IntrNoMem]>;

def int_fgpu_subu_ph: GCCBuiltin<"__builtin_fgpu_subu_ph">,
  Intrinsic<[llvm_v2i16_ty], [llvm_v2i16_ty, llvm_v2i16_ty], []>;
def int_fgpu_subu_s_ph: GCCBuiltin<"__builtin_fgpu_subu_s_ph">,
  Intrinsic<[llvm_v2i16_ty], [llvm_v2i16_ty, llvm_v2i16_ty], []>;

def int_fgpu_subuh_qb: GCCBuiltin<"__builtin_fgpu_subuh_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty], [IntrNoMem]>;
def int_fgpu_subuh_r_qb: GCCBuiltin<"__builtin_fgpu_subuh_r_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty], [IntrNoMem]>;

//===----------------------------------------------------------------------===//
// FGPU MSA

//===----------------------------------------------------------------------===//
// Addition/subtraction

def int_fgpu_add_a_b : GCCBuiltin<"__builtin_msa_add_a_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_add_a_h : GCCBuiltin<"__builtin_msa_add_a_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_add_a_w : GCCBuiltin<"__builtin_msa_add_a_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_add_a_d : GCCBuiltin<"__builtin_msa_add_a_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_fgpu_adds_a_b : GCCBuiltin<"__builtin_msa_adds_a_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_adds_a_h : GCCBuiltin<"__builtin_msa_adds_a_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_adds_a_w : GCCBuiltin<"__builtin_msa_adds_a_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_adds_a_d : GCCBuiltin<"__builtin_msa_adds_a_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_fgpu_adds_s_b : GCCBuiltin<"__builtin_msa_adds_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_adds_s_h : GCCBuiltin<"__builtin_msa_adds_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_adds_s_w : GCCBuiltin<"__builtin_msa_adds_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_adds_s_d : GCCBuiltin<"__builtin_msa_adds_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_fgpu_adds_u_b : GCCBuiltin<"__builtin_msa_adds_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_adds_u_h : GCCBuiltin<"__builtin_msa_adds_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_adds_u_w : GCCBuiltin<"__builtin_msa_adds_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_adds_u_d : GCCBuiltin<"__builtin_msa_adds_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_fgpu_addv_b : GCCBuiltin<"__builtin_msa_addv_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_addv_h : GCCBuiltin<"__builtin_msa_addv_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_addv_w : GCCBuiltin<"__builtin_msa_addv_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_addv_d : GCCBuiltin<"__builtin_msa_addv_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_fgpu_addvi_b : GCCBuiltin<"__builtin_msa_addvi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty],
  [Commutative, IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_addvi_h : GCCBuiltin<"__builtin_msa_addvi_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty],
  [Commutative, IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_addvi_w : GCCBuiltin<"__builtin_msa_addvi_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty],
  [Commutative, IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_addvi_d : GCCBuiltin<"__builtin_msa_addvi_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty],
  [Commutative, IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_and_v : GCCBuiltin<"__builtin_msa_and_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;

def int_fgpu_andi_b : GCCBuiltin<"__builtin_msa_andi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_asub_s_b : GCCBuiltin<"__builtin_msa_asub_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_asub_s_h : GCCBuiltin<"__builtin_msa_asub_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_asub_s_w : GCCBuiltin<"__builtin_msa_asub_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_asub_s_d : GCCBuiltin<"__builtin_msa_asub_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_asub_u_b : GCCBuiltin<"__builtin_msa_asub_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_asub_u_h : GCCBuiltin<"__builtin_msa_asub_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_asub_u_w : GCCBuiltin<"__builtin_msa_asub_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_asub_u_d : GCCBuiltin<"__builtin_msa_asub_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_ave_s_b : GCCBuiltin<"__builtin_msa_ave_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_ave_s_h : GCCBuiltin<"__builtin_msa_ave_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_ave_s_w : GCCBuiltin<"__builtin_msa_ave_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_ave_s_d : GCCBuiltin<"__builtin_msa_ave_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_fgpu_ave_u_b : GCCBuiltin<"__builtin_msa_ave_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_ave_u_h : GCCBuiltin<"__builtin_msa_ave_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_ave_u_w : GCCBuiltin<"__builtin_msa_ave_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_ave_u_d : GCCBuiltin<"__builtin_msa_ave_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_fgpu_aver_s_b : GCCBuiltin<"__builtin_msa_aver_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_aver_s_h : GCCBuiltin<"__builtin_msa_aver_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_aver_s_w : GCCBuiltin<"__builtin_msa_aver_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_aver_s_d : GCCBuiltin<"__builtin_msa_aver_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_fgpu_aver_u_b : GCCBuiltin<"__builtin_msa_aver_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_aver_u_h : GCCBuiltin<"__builtin_msa_aver_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_aver_u_w : GCCBuiltin<"__builtin_msa_aver_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_fgpu_aver_u_d : GCCBuiltin<"__builtin_msa_aver_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_fgpu_bclr_b : GCCBuiltin<"__builtin_msa_bclr_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_bclr_h : GCCBuiltin<"__builtin_msa_bclr_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_bclr_w : GCCBuiltin<"__builtin_msa_bclr_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_bclr_d : GCCBuiltin<"__builtin_msa_bclr_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_bclri_b : GCCBuiltin<"__builtin_msa_bclri_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_bclri_h : GCCBuiltin<"__builtin_msa_bclri_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_bclri_w : GCCBuiltin<"__builtin_msa_bclri_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_bclri_d : GCCBuiltin<"__builtin_msa_bclri_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_binsl_b : GCCBuiltin<"__builtin_msa_binsl_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
            [IntrNoMem]>;
def int_fgpu_binsl_h : GCCBuiltin<"__builtin_msa_binsl_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
            [IntrNoMem]>;
def int_fgpu_binsl_w : GCCBuiltin<"__builtin_msa_binsl_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
            [IntrNoMem]>;
def int_fgpu_binsl_d : GCCBuiltin<"__builtin_msa_binsl_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_v2i64_ty],
            [IntrNoMem]>;

def int_fgpu_binsli_b : GCCBuiltin<"__builtin_msa_binsli_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;
def int_fgpu_binsli_h : GCCBuiltin<"__builtin_msa_binsli_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;
def int_fgpu_binsli_w : GCCBuiltin<"__builtin_msa_binsli_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;
def int_fgpu_binsli_d : GCCBuiltin<"__builtin_msa_binsli_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;

def int_fgpu_binsr_b : GCCBuiltin<"__builtin_msa_binsr_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
            [IntrNoMem]>;
def int_fgpu_binsr_h : GCCBuiltin<"__builtin_msa_binsr_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
            [IntrNoMem]>;
def int_fgpu_binsr_w : GCCBuiltin<"__builtin_msa_binsr_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
            [IntrNoMem]>;
def int_fgpu_binsr_d : GCCBuiltin<"__builtin_msa_binsr_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_v2i64_ty],
            [IntrNoMem]>;

def int_fgpu_binsri_b : GCCBuiltin<"__builtin_msa_binsri_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;
def int_fgpu_binsri_h : GCCBuiltin<"__builtin_msa_binsri_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;
def int_fgpu_binsri_w : GCCBuiltin<"__builtin_msa_binsri_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;
def int_fgpu_binsri_d : GCCBuiltin<"__builtin_msa_binsri_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;

def int_fgpu_bmnz_v : GCCBuiltin<"__builtin_msa_bmnz_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
            [IntrNoMem]>;

def int_fgpu_bmnzi_b : GCCBuiltin<"__builtin_msa_bmnzi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;

def int_fgpu_bmz_v : GCCBuiltin<"__builtin_msa_bmz_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
            [IntrNoMem]>;

def int_fgpu_bmzi_b : GCCBuiltin<"__builtin_msa_bmzi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;

def int_fgpu_bneg_b : GCCBuiltin<"__builtin_msa_bneg_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_bneg_h : GCCBuiltin<"__builtin_msa_bneg_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_bneg_w : GCCBuiltin<"__builtin_msa_bneg_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_bneg_d : GCCBuiltin<"__builtin_msa_bneg_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_bnegi_b : GCCBuiltin<"__builtin_msa_bnegi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_bnegi_h : GCCBuiltin<"__builtin_msa_bnegi_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_bnegi_w : GCCBuiltin<"__builtin_msa_bnegi_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_bnegi_d : GCCBuiltin<"__builtin_msa_bnegi_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_bnz_b : GCCBuiltin<"__builtin_msa_bnz_b">,
  Intrinsic<[llvm_i32_ty], [llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_bnz_h : GCCBuiltin<"__builtin_msa_bnz_h">,
  Intrinsic<[llvm_i32_ty], [llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_bnz_w : GCCBuiltin<"__builtin_msa_bnz_w">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_bnz_d : GCCBuiltin<"__builtin_msa_bnz_d">,
  Intrinsic<[llvm_i32_ty], [llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_bnz_v : GCCBuiltin<"__builtin_msa_bnz_v">,
  Intrinsic<[llvm_i32_ty], [llvm_v16i8_ty], [IntrNoMem]>;

def int_fgpu_bsel_v : GCCBuiltin<"__builtin_msa_bsel_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
            [IntrNoMem]>;

def int_fgpu_bseli_b : GCCBuiltin<"__builtin_msa_bseli_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;

def int_fgpu_bset_b : GCCBuiltin<"__builtin_msa_bset_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_bset_h : GCCBuiltin<"__builtin_msa_bset_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_bset_w : GCCBuiltin<"__builtin_msa_bset_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_bset_d : GCCBuiltin<"__builtin_msa_bset_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_bseti_b : GCCBuiltin<"__builtin_msa_bseti_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_bseti_h : GCCBuiltin<"__builtin_msa_bseti_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_bseti_w : GCCBuiltin<"__builtin_msa_bseti_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_bseti_d : GCCBuiltin<"__builtin_msa_bseti_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_bz_b : GCCBuiltin<"__builtin_msa_bz_b">,
  Intrinsic<[llvm_i32_ty], [llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_bz_h : GCCBuiltin<"__builtin_msa_bz_h">,
  Intrinsic<[llvm_i32_ty], [llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_bz_w : GCCBuiltin<"__builtin_msa_bz_w">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_bz_d : GCCBuiltin<"__builtin_msa_bz_d">,
  Intrinsic<[llvm_i32_ty], [llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_bz_v : GCCBuiltin<"__builtin_msa_bz_v">,
  Intrinsic<[llvm_i32_ty], [llvm_v16i8_ty], [IntrNoMem]>;

def int_fgpu_ceq_b : GCCBuiltin<"__builtin_msa_ceq_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_ceq_h : GCCBuiltin<"__builtin_msa_ceq_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_ceq_w : GCCBuiltin<"__builtin_msa_ceq_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_ceq_d : GCCBuiltin<"__builtin_msa_ceq_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_ceqi_b : GCCBuiltin<"__builtin_msa_ceqi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_ceqi_h : GCCBuiltin<"__builtin_msa_ceqi_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_ceqi_w : GCCBuiltin<"__builtin_msa_ceqi_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_ceqi_d : GCCBuiltin<"__builtin_msa_ceqi_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_cfcmsa : GCCBuiltin<"__builtin_msa_cfcmsa">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [ImmArg<ArgIndex<0>>]>;

def int_fgpu_cle_s_b : GCCBuiltin<"__builtin_msa_cle_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_cle_s_h : GCCBuiltin<"__builtin_msa_cle_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_cle_s_w : GCCBuiltin<"__builtin_msa_cle_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_cle_s_d : GCCBuiltin<"__builtin_msa_cle_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_cle_u_b : GCCBuiltin<"__builtin_msa_cle_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_cle_u_h : GCCBuiltin<"__builtin_msa_cle_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_cle_u_w : GCCBuiltin<"__builtin_msa_cle_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_cle_u_d : GCCBuiltin<"__builtin_msa_cle_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_clei_s_b : GCCBuiltin<"__builtin_msa_clei_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_clei_s_h : GCCBuiltin<"__builtin_msa_clei_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_clei_s_w : GCCBuiltin<"__builtin_msa_clei_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_clei_s_d : GCCBuiltin<"__builtin_msa_clei_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_clei_u_b : GCCBuiltin<"__builtin_msa_clei_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_clei_u_h : GCCBuiltin<"__builtin_msa_clei_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_clei_u_w : GCCBuiltin<"__builtin_msa_clei_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_clei_u_d : GCCBuiltin<"__builtin_msa_clei_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_clt_s_b : GCCBuiltin<"__builtin_msa_clt_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_clt_s_h : GCCBuiltin<"__builtin_msa_clt_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_clt_s_w : GCCBuiltin<"__builtin_msa_clt_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_clt_s_d : GCCBuiltin<"__builtin_msa_clt_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_clt_u_b : GCCBuiltin<"__builtin_msa_clt_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_clt_u_h : GCCBuiltin<"__builtin_msa_clt_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_clt_u_w : GCCBuiltin<"__builtin_msa_clt_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_clt_u_d : GCCBuiltin<"__builtin_msa_clt_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_clti_s_b : GCCBuiltin<"__builtin_msa_clti_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_clti_s_h : GCCBuiltin<"__builtin_msa_clti_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_clti_s_w : GCCBuiltin<"__builtin_msa_clti_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_clti_s_d : GCCBuiltin<"__builtin_msa_clti_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_clti_u_b : GCCBuiltin<"__builtin_msa_clti_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_clti_u_h : GCCBuiltin<"__builtin_msa_clti_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_clti_u_w : GCCBuiltin<"__builtin_msa_clti_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_clti_u_d : GCCBuiltin<"__builtin_msa_clti_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_copy_s_b : GCCBuiltin<"__builtin_msa_copy_s_b">,
  Intrinsic<[llvm_i32_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_copy_s_h : GCCBuiltin<"__builtin_msa_copy_s_h">,
  Intrinsic<[llvm_i32_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_copy_s_w : GCCBuiltin<"__builtin_msa_copy_s_w">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_copy_s_d : GCCBuiltin<"__builtin_msa_copy_s_d">,
  Intrinsic<[llvm_i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_fgpu_copy_u_b : GCCBuiltin<"__builtin_msa_copy_u_b">,
  Intrinsic<[llvm_i32_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_copy_u_h : GCCBuiltin<"__builtin_msa_copy_u_h">,
  Intrinsic<[llvm_i32_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_copy_u_w : GCCBuiltin<"__builtin_msa_copy_u_w">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_copy_u_d : GCCBuiltin<"__builtin_msa_copy_u_d">,
  Intrinsic<[llvm_i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_fgpu_ctcmsa : GCCBuiltin<"__builtin_msa_ctcmsa">,
  Intrinsic<[], [llvm_i32_ty, llvm_i32_ty], [ImmArg<ArgIndex<0>>]>;

def int_fgpu_div_s_b : GCCBuiltin<"__builtin_msa_div_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_div_s_h : GCCBuiltin<"__builtin_msa_div_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_div_s_w : GCCBuiltin<"__builtin_msa_div_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_div_s_d : GCCBuiltin<"__builtin_msa_div_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_div_u_b : GCCBuiltin<"__builtin_msa_div_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_div_u_h : GCCBuiltin<"__builtin_msa_div_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_div_u_w : GCCBuiltin<"__builtin_msa_div_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_div_u_d : GCCBuiltin<"__builtin_msa_div_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

// This instruction is part of the MSA spec but it does not share the
// __builtin_msa prefix because it operates on GP registers.
def int_fgpu_dlsa : GCCBuiltin<"__builtin_fgpu_dlsa">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i64_ty, llvm_i32_ty],
            [IntrNoMem]>;

def int_fgpu_dotp_s_h : GCCBuiltin<"__builtin_msa_dotp_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_dotp_s_w : GCCBuiltin<"__builtin_msa_dotp_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_dotp_s_d : GCCBuiltin<"__builtin_msa_dotp_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_fgpu_dotp_u_h : GCCBuiltin<"__builtin_msa_dotp_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_dotp_u_w : GCCBuiltin<"__builtin_msa_dotp_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_dotp_u_d : GCCBuiltin<"__builtin_msa_dotp_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_fgpu_dpadd_s_h : GCCBuiltin<"__builtin_msa_dpadd_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v16i8_ty, llvm_v16i8_ty],
  [IntrNoMem]>;
def int_fgpu_dpadd_s_w : GCCBuiltin<"__builtin_msa_dpadd_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_fgpu_dpadd_s_d : GCCBuiltin<"__builtin_msa_dpadd_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_fgpu_dpadd_u_h : GCCBuiltin<"__builtin_msa_dpadd_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v16i8_ty, llvm_v16i8_ty],
  [IntrNoMem]>;
def int_fgpu_dpadd_u_w : GCCBuiltin<"__builtin_msa_dpadd_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_fgpu_dpadd_u_d : GCCBuiltin<"__builtin_msa_dpadd_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_fgpu_dpsub_s_h : GCCBuiltin<"__builtin_msa_dpsub_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v16i8_ty, llvm_v16i8_ty],
  [IntrNoMem]>;
def int_fgpu_dpsub_s_w : GCCBuiltin<"__builtin_msa_dpsub_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_fgpu_dpsub_s_d : GCCBuiltin<"__builtin_msa_dpsub_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_fgpu_dpsub_u_h : GCCBuiltin<"__builtin_msa_dpsub_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v16i8_ty, llvm_v16i8_ty],
  [IntrNoMem]>;
def int_fgpu_dpsub_u_w : GCCBuiltin<"__builtin_msa_dpsub_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_fgpu_dpsub_u_d : GCCBuiltin<"__builtin_msa_dpsub_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_fgpu_fadd_w : GCCBuiltin<"__builtin_msa_fadd_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fadd_d : GCCBuiltin<"__builtin_msa_fadd_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fcaf_w : GCCBuiltin<"__builtin_msa_fcaf_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fcaf_d : GCCBuiltin<"__builtin_msa_fcaf_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fceq_w : GCCBuiltin<"__builtin_msa_fceq_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fceq_d : GCCBuiltin<"__builtin_msa_fceq_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fcle_w : GCCBuiltin<"__builtin_msa_fcle_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fcle_d : GCCBuiltin<"__builtin_msa_fcle_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fclt_w : GCCBuiltin<"__builtin_msa_fclt_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fclt_d : GCCBuiltin<"__builtin_msa_fclt_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fclass_w : GCCBuiltin<"__builtin_msa_fclass_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fclass_d : GCCBuiltin<"__builtin_msa_fclass_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fcne_w : GCCBuiltin<"__builtin_msa_fcne_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fcne_d : GCCBuiltin<"__builtin_msa_fcne_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fcor_w : GCCBuiltin<"__builtin_msa_fcor_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fcor_d : GCCBuiltin<"__builtin_msa_fcor_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fcueq_w : GCCBuiltin<"__builtin_msa_fcueq_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fcueq_d : GCCBuiltin<"__builtin_msa_fcueq_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fcule_w : GCCBuiltin<"__builtin_msa_fcule_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fcule_d : GCCBuiltin<"__builtin_msa_fcule_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fcult_w : GCCBuiltin<"__builtin_msa_fcult_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fcult_d : GCCBuiltin<"__builtin_msa_fcult_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fcun_w : GCCBuiltin<"__builtin_msa_fcun_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fcun_d : GCCBuiltin<"__builtin_msa_fcun_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fcune_w : GCCBuiltin<"__builtin_msa_fcune_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fcune_d : GCCBuiltin<"__builtin_msa_fcune_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fdiv_w : GCCBuiltin<"__builtin_msa_fdiv_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fdiv_d : GCCBuiltin<"__builtin_msa_fdiv_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fexdo_h : GCCBuiltin<"__builtin_msa_fexdo_h">,
  Intrinsic<[llvm_v8f16_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fexdo_w : GCCBuiltin<"__builtin_msa_fexdo_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fexp2_w : GCCBuiltin<"__builtin_msa_fexp2_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_fexp2_d : GCCBuiltin<"__builtin_msa_fexp2_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_fexupl_w : GCCBuiltin<"__builtin_msa_fexupl_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v8f16_ty], [IntrNoMem]>;
def int_fgpu_fexupl_d : GCCBuiltin<"__builtin_msa_fexupl_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v4f32_ty], [IntrNoMem]>;

def int_fgpu_fexupr_w : GCCBuiltin<"__builtin_msa_fexupr_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v8f16_ty], [IntrNoMem]>;
def int_fgpu_fexupr_d : GCCBuiltin<"__builtin_msa_fexupr_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v4f32_ty], [IntrNoMem]>;

def int_fgpu_ffint_s_w : GCCBuiltin<"__builtin_msa_ffint_s_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_ffint_s_d : GCCBuiltin<"__builtin_msa_ffint_s_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_ffint_u_w : GCCBuiltin<"__builtin_msa_ffint_u_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_ffint_u_d : GCCBuiltin<"__builtin_msa_ffint_u_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_ffql_w : GCCBuiltin<"__builtin_msa_ffql_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_ffql_d : GCCBuiltin<"__builtin_msa_ffql_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v4i32_ty], [IntrNoMem]>;

def int_fgpu_ffqr_w : GCCBuiltin<"__builtin_msa_ffqr_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_ffqr_d : GCCBuiltin<"__builtin_msa_ffqr_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v4i32_ty], [IntrNoMem]>;

def int_fgpu_fill_b : GCCBuiltin<"__builtin_msa_fill_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_fill_h : GCCBuiltin<"__builtin_msa_fill_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_fill_w : GCCBuiltin<"__builtin_msa_fill_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_fill_d : GCCBuiltin<"__builtin_msa_fill_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_i64_ty], [IntrNoMem]>;

def int_fgpu_flog2_w : GCCBuiltin<"__builtin_msa_flog2_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_flog2_d : GCCBuiltin<"__builtin_msa_flog2_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fmadd_w : GCCBuiltin<"__builtin_msa_fmadd_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty, llvm_v4f32_ty],
            [IntrNoMem]>;
def int_fgpu_fmadd_d : GCCBuiltin<"__builtin_msa_fmadd_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty, llvm_v2f64_ty],
            [IntrNoMem]>;

def int_fgpu_fmax_w : GCCBuiltin<"__builtin_msa_fmax_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fmax_d : GCCBuiltin<"__builtin_msa_fmax_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fmax_a_w : GCCBuiltin<"__builtin_msa_fmax_a_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fmax_a_d : GCCBuiltin<"__builtin_msa_fmax_a_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fmin_w : GCCBuiltin<"__builtin_msa_fmin_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fmin_d : GCCBuiltin<"__builtin_msa_fmin_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fmin_a_w : GCCBuiltin<"__builtin_msa_fmin_a_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fmin_a_d : GCCBuiltin<"__builtin_msa_fmin_a_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fmsub_w : GCCBuiltin<"__builtin_msa_fmsub_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty, llvm_v4f32_ty],
            [IntrNoMem]>;
def int_fgpu_fmsub_d : GCCBuiltin<"__builtin_msa_fmsub_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty, llvm_v2f64_ty],
            [IntrNoMem]>;

def int_fgpu_fmul_w : GCCBuiltin<"__builtin_msa_fmul_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fmul_d : GCCBuiltin<"__builtin_msa_fmul_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_frint_w : GCCBuiltin<"__builtin_msa_frint_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_frint_d : GCCBuiltin<"__builtin_msa_frint_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_frcp_w : GCCBuiltin<"__builtin_msa_frcp_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_frcp_d : GCCBuiltin<"__builtin_msa_frcp_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_frsqrt_w : GCCBuiltin<"__builtin_msa_frsqrt_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_frsqrt_d : GCCBuiltin<"__builtin_msa_frsqrt_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fsaf_w : GCCBuiltin<"__builtin_msa_fsaf_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fsaf_d : GCCBuiltin<"__builtin_msa_fsaf_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fseq_w : GCCBuiltin<"__builtin_msa_fseq_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fseq_d : GCCBuiltin<"__builtin_msa_fseq_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fsle_w : GCCBuiltin<"__builtin_msa_fsle_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fsle_d : GCCBuiltin<"__builtin_msa_fsle_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fslt_w : GCCBuiltin<"__builtin_msa_fslt_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fslt_d : GCCBuiltin<"__builtin_msa_fslt_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fsne_w : GCCBuiltin<"__builtin_msa_fsne_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fsne_d : GCCBuiltin<"__builtin_msa_fsne_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fsor_w : GCCBuiltin<"__builtin_msa_fsor_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fsor_d : GCCBuiltin<"__builtin_msa_fsor_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fsqrt_w : GCCBuiltin<"__builtin_msa_fsqrt_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fsqrt_d : GCCBuiltin<"__builtin_msa_fsqrt_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fsub_w : GCCBuiltin<"__builtin_msa_fsub_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fsub_d : GCCBuiltin<"__builtin_msa_fsub_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fsueq_w : GCCBuiltin<"__builtin_msa_fsueq_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fsueq_d : GCCBuiltin<"__builtin_msa_fsueq_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fsule_w : GCCBuiltin<"__builtin_msa_fsule_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fsule_d : GCCBuiltin<"__builtin_msa_fsule_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fsult_w : GCCBuiltin<"__builtin_msa_fsult_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fsult_d : GCCBuiltin<"__builtin_msa_fsult_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fsun_w : GCCBuiltin<"__builtin_msa_fsun_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fsun_d : GCCBuiltin<"__builtin_msa_fsun_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_fsune_w : GCCBuiltin<"__builtin_msa_fsune_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_fsune_d : GCCBuiltin<"__builtin_msa_fsune_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_ftint_s_w : GCCBuiltin<"__builtin_msa_ftint_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_ftint_s_d : GCCBuiltin<"__builtin_msa_ftint_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_ftint_u_w : GCCBuiltin<"__builtin_msa_ftint_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_ftint_u_d : GCCBuiltin<"__builtin_msa_ftint_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_ftq_h : GCCBuiltin<"__builtin_msa_ftq_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_ftq_w : GCCBuiltin<"__builtin_msa_ftq_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_ftrunc_s_w : GCCBuiltin<"__builtin_msa_ftrunc_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_ftrunc_s_d : GCCBuiltin<"__builtin_msa_ftrunc_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_ftrunc_u_w : GCCBuiltin<"__builtin_msa_ftrunc_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_fgpu_ftrunc_u_d : GCCBuiltin<"__builtin_msa_ftrunc_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_fgpu_hadd_s_h : GCCBuiltin<"__builtin_msa_hadd_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_hadd_s_w : GCCBuiltin<"__builtin_msa_hadd_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_hadd_s_d : GCCBuiltin<"__builtin_msa_hadd_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_fgpu_hadd_u_h : GCCBuiltin<"__builtin_msa_hadd_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_hadd_u_w : GCCBuiltin<"__builtin_msa_hadd_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_hadd_u_d : GCCBuiltin<"__builtin_msa_hadd_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_fgpu_hsub_s_h : GCCBuiltin<"__builtin_msa_hsub_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_hsub_s_w : GCCBuiltin<"__builtin_msa_hsub_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_hsub_s_d : GCCBuiltin<"__builtin_msa_hsub_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_fgpu_hsub_u_h : GCCBuiltin<"__builtin_msa_hsub_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_hsub_u_w : GCCBuiltin<"__builtin_msa_hsub_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_hsub_u_d : GCCBuiltin<"__builtin_msa_hsub_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_fgpu_ilvev_b : GCCBuiltin<"__builtin_msa_ilvev_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_ilvev_h : GCCBuiltin<"__builtin_msa_ilvev_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_ilvev_w : GCCBuiltin<"__builtin_msa_ilvev_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_ilvev_d : GCCBuiltin<"__builtin_msa_ilvev_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_ilvl_b : GCCBuiltin<"__builtin_msa_ilvl_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_ilvl_h : GCCBuiltin<"__builtin_msa_ilvl_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_ilvl_w : GCCBuiltin<"__builtin_msa_ilvl_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_ilvl_d : GCCBuiltin<"__builtin_msa_ilvl_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_ilvod_b : GCCBuiltin<"__builtin_msa_ilvod_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_ilvod_h : GCCBuiltin<"__builtin_msa_ilvod_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_ilvod_w : GCCBuiltin<"__builtin_msa_ilvod_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_ilvod_d : GCCBuiltin<"__builtin_msa_ilvod_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_ilvr_b : GCCBuiltin<"__builtin_msa_ilvr_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_ilvr_h : GCCBuiltin<"__builtin_msa_ilvr_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_ilvr_w : GCCBuiltin<"__builtin_msa_ilvr_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_ilvr_d : GCCBuiltin<"__builtin_msa_ilvr_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_insert_b : GCCBuiltin<"__builtin_msa_insert_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem]>;
def int_fgpu_insert_h : GCCBuiltin<"__builtin_msa_insert_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem]>;
def int_fgpu_insert_w : GCCBuiltin<"__builtin_msa_insert_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem]>;
def int_fgpu_insert_d : GCCBuiltin<"__builtin_msa_insert_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty, llvm_i64_ty],
  [IntrNoMem]>;

def int_fgpu_insve_b : GCCBuiltin<"__builtin_msa_insve_b">,
  Intrinsic<[llvm_v16i8_ty],
            [llvm_v16i8_ty, llvm_i32_ty, llvm_v16i8_ty],
            [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_insve_h : GCCBuiltin<"__builtin_msa_insve_h">,
  Intrinsic<[llvm_v8i16_ty],
            [llvm_v8i16_ty, llvm_i32_ty, llvm_v8i16_ty],
            [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_insve_w : GCCBuiltin<"__builtin_msa_insve_w">,
  Intrinsic<[llvm_v4i32_ty],
            [llvm_v4i32_ty, llvm_i32_ty, llvm_v4i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_insve_d : GCCBuiltin<"__builtin_msa_insve_d">,
  Intrinsic<[llvm_v2i64_ty],
            [llvm_v2i64_ty, llvm_i32_ty, llvm_v2i64_ty],
            [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_ld_b : GCCBuiltin<"__builtin_msa_ld_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_ptr_ty, llvm_i32_ty],
  [IntrReadMem, IntrArgMemOnly]>;
def int_fgpu_ld_h : GCCBuiltin<"__builtin_msa_ld_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_ptr_ty, llvm_i32_ty],
  [IntrReadMem, IntrArgMemOnly]>;
def int_fgpu_ld_w : GCCBuiltin<"__builtin_msa_ld_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_ptr_ty, llvm_i32_ty],
  [IntrReadMem, IntrArgMemOnly]>;
def int_fgpu_ld_d : GCCBuiltin<"__builtin_msa_ld_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_ptr_ty, llvm_i32_ty],
  [IntrReadMem, IntrArgMemOnly]>;

def int_fgpu_ldr_d : GCCBuiltin<"__builtin_msa_ldr_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_ptr_ty, llvm_i32_ty],
  [IntrReadMem, IntrArgMemOnly]>;
def int_fgpu_ldr_w : GCCBuiltin<"__builtin_msa_ldr_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_ptr_ty, llvm_i32_ty],
  [IntrReadMem, IntrArgMemOnly]>;

def int_fgpu_ldi_b : GCCBuiltin<"__builtin_msa_ldi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<0>>]>;
def int_fgpu_ldi_h : GCCBuiltin<"__builtin_msa_ldi_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<0>>]>;
def int_fgpu_ldi_w : GCCBuiltin<"__builtin_msa_ldi_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<0>>]>;
def int_fgpu_ldi_d : GCCBuiltin<"__builtin_msa_ldi_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<0>>]>;

// This instruction is part of the MSA spec but it does not share the
// __builtin_msa prefix because it operates on the GPR registers.
def int_fgpu_lsa : GCCBuiltin<"__builtin_fgpu_lsa">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem]>;

def int_fgpu_madd_q_h : GCCBuiltin<"__builtin_msa_madd_q_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_fgpu_madd_q_w : GCCBuiltin<"__builtin_msa_madd_q_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_fgpu_maddr_q_h : GCCBuiltin<"__builtin_msa_maddr_q_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_fgpu_maddr_q_w : GCCBuiltin<"__builtin_msa_maddr_q_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_fgpu_maddv_b : GCCBuiltin<"__builtin_msa_maddv_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
  [IntrNoMem]>;
def int_fgpu_maddv_h : GCCBuiltin<"__builtin_msa_maddv_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_fgpu_maddv_w : GCCBuiltin<"__builtin_msa_maddv_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;
def int_fgpu_maddv_d : GCCBuiltin<"__builtin_msa_maddv_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_v2i64_ty],
  [IntrNoMem]>;

def int_fgpu_max_a_b : GCCBuiltin<"__builtin_msa_max_a_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_max_a_h : GCCBuiltin<"__builtin_msa_max_a_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_max_a_w : GCCBuiltin<"__builtin_msa_max_a_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_max_a_d : GCCBuiltin<"__builtin_msa_max_a_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_max_s_b : GCCBuiltin<"__builtin_msa_max_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_max_s_h : GCCBuiltin<"__builtin_msa_max_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_max_s_w : GCCBuiltin<"__builtin_msa_max_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_max_s_d : GCCBuiltin<"__builtin_msa_max_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_max_u_b : GCCBuiltin<"__builtin_msa_max_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_max_u_h : GCCBuiltin<"__builtin_msa_max_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_max_u_w : GCCBuiltin<"__builtin_msa_max_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_max_u_d : GCCBuiltin<"__builtin_msa_max_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_maxi_s_b : GCCBuiltin<"__builtin_msa_maxi_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_maxi_s_h : GCCBuiltin<"__builtin_msa_maxi_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_maxi_s_w : GCCBuiltin<"__builtin_msa_maxi_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_maxi_s_d : GCCBuiltin<"__builtin_msa_maxi_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_maxi_u_b : GCCBuiltin<"__builtin_msa_maxi_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_maxi_u_h : GCCBuiltin<"__builtin_msa_maxi_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_maxi_u_w : GCCBuiltin<"__builtin_msa_maxi_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_maxi_u_d : GCCBuiltin<"__builtin_msa_maxi_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_min_a_b : GCCBuiltin<"__builtin_msa_min_a_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_min_a_h : GCCBuiltin<"__builtin_msa_min_a_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_min_a_w : GCCBuiltin<"__builtin_msa_min_a_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_min_a_d : GCCBuiltin<"__builtin_msa_min_a_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_min_s_b : GCCBuiltin<"__builtin_msa_min_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_min_s_h : GCCBuiltin<"__builtin_msa_min_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_min_s_w : GCCBuiltin<"__builtin_msa_min_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_min_s_d : GCCBuiltin<"__builtin_msa_min_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_min_u_b : GCCBuiltin<"__builtin_msa_min_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_min_u_h : GCCBuiltin<"__builtin_msa_min_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_min_u_w : GCCBuiltin<"__builtin_msa_min_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_min_u_d : GCCBuiltin<"__builtin_msa_min_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_mini_s_b : GCCBuiltin<"__builtin_msa_mini_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_mini_s_h : GCCBuiltin<"__builtin_msa_mini_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_mini_s_w : GCCBuiltin<"__builtin_msa_mini_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_mini_s_d : GCCBuiltin<"__builtin_msa_mini_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_mini_u_b : GCCBuiltin<"__builtin_msa_mini_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_mini_u_h : GCCBuiltin<"__builtin_msa_mini_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_mini_u_w : GCCBuiltin<"__builtin_msa_mini_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_mini_u_d : GCCBuiltin<"__builtin_msa_mini_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_mod_s_b : GCCBuiltin<"__builtin_msa_mod_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_mod_s_h : GCCBuiltin<"__builtin_msa_mod_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_mod_s_w : GCCBuiltin<"__builtin_msa_mod_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_mod_s_d : GCCBuiltin<"__builtin_msa_mod_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_mod_u_b : GCCBuiltin<"__builtin_msa_mod_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_mod_u_h : GCCBuiltin<"__builtin_msa_mod_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_mod_u_w : GCCBuiltin<"__builtin_msa_mod_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_mod_u_d : GCCBuiltin<"__builtin_msa_mod_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_move_v : GCCBuiltin<"__builtin_msa_move_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty], [IntrNoMem]>;

def int_fgpu_msub_q_h : GCCBuiltin<"__builtin_msa_msub_q_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_fgpu_msub_q_w : GCCBuiltin<"__builtin_msa_msub_q_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_fgpu_msubr_q_h : GCCBuiltin<"__builtin_msa_msubr_q_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_fgpu_msubr_q_w : GCCBuiltin<"__builtin_msa_msubr_q_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_fgpu_msubv_b : GCCBuiltin<"__builtin_msa_msubv_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
  [IntrNoMem]>;
def int_fgpu_msubv_h : GCCBuiltin<"__builtin_msa_msubv_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_fgpu_msubv_w : GCCBuiltin<"__builtin_msa_msubv_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;
def int_fgpu_msubv_d : GCCBuiltin<"__builtin_msa_msubv_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_v2i64_ty],
  [IntrNoMem]>;

def int_fgpu_mul_q_h : GCCBuiltin<"__builtin_msa_mul_q_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_mul_q_w : GCCBuiltin<"__builtin_msa_mul_q_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_fgpu_mulr_q_h : GCCBuiltin<"__builtin_msa_mulr_q_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_mulr_q_w : GCCBuiltin<"__builtin_msa_mulr_q_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_fgpu_mulv_b : GCCBuiltin<"__builtin_msa_mulv_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_mulv_h : GCCBuiltin<"__builtin_msa_mulv_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_mulv_w : GCCBuiltin<"__builtin_msa_mulv_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_mulv_d : GCCBuiltin<"__builtin_msa_mulv_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_nloc_b : GCCBuiltin<"__builtin_msa_nloc_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_nloc_h : GCCBuiltin<"__builtin_msa_nloc_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_nloc_w : GCCBuiltin<"__builtin_msa_nloc_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_nloc_d : GCCBuiltin<"__builtin_msa_nloc_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_nlzc_b : GCCBuiltin<"__builtin_msa_nlzc_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_nlzc_h : GCCBuiltin<"__builtin_msa_nlzc_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_nlzc_w : GCCBuiltin<"__builtin_msa_nlzc_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_nlzc_d : GCCBuiltin<"__builtin_msa_nlzc_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_nor_v : GCCBuiltin<"__builtin_msa_nor_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;

def int_fgpu_nori_b : GCCBuiltin<"__builtin_msa_nori_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_or_v : GCCBuiltin<"__builtin_msa_or_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;

def int_fgpu_ori_b : GCCBuiltin<"__builtin_msa_ori_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_pckev_b : GCCBuiltin<"__builtin_msa_pckev_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_pckev_h : GCCBuiltin<"__builtin_msa_pckev_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_pckev_w : GCCBuiltin<"__builtin_msa_pckev_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_pckev_d : GCCBuiltin<"__builtin_msa_pckev_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_pckod_b : GCCBuiltin<"__builtin_msa_pckod_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_pckod_h : GCCBuiltin<"__builtin_msa_pckod_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_pckod_w : GCCBuiltin<"__builtin_msa_pckod_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_pckod_d : GCCBuiltin<"__builtin_msa_pckod_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_pcnt_b : GCCBuiltin<"__builtin_msa_pcnt_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_pcnt_h : GCCBuiltin<"__builtin_msa_pcnt_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_pcnt_w : GCCBuiltin<"__builtin_msa_pcnt_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_pcnt_d : GCCBuiltin<"__builtin_msa_pcnt_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_sat_s_b : GCCBuiltin<"__builtin_msa_sat_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_sat_s_h : GCCBuiltin<"__builtin_msa_sat_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_sat_s_w : GCCBuiltin<"__builtin_msa_sat_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_sat_s_d : GCCBuiltin<"__builtin_msa_sat_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_sat_u_b : GCCBuiltin<"__builtin_msa_sat_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_sat_u_h : GCCBuiltin<"__builtin_msa_sat_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_sat_u_w : GCCBuiltin<"__builtin_msa_sat_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_sat_u_d : GCCBuiltin<"__builtin_msa_sat_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_shf_b : GCCBuiltin<"__builtin_msa_shf_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_shf_h : GCCBuiltin<"__builtin_msa_shf_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_shf_w : GCCBuiltin<"__builtin_msa_shf_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_sld_b : GCCBuiltin<"__builtin_msa_sld_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_sld_h : GCCBuiltin<"__builtin_msa_sld_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_sld_w : GCCBuiltin<"__builtin_msa_sld_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_sld_d : GCCBuiltin<"__builtin_msa_sld_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_fgpu_sldi_b : GCCBuiltin<"__builtin_msa_sldi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;
def int_fgpu_sldi_h : GCCBuiltin<"__builtin_msa_sldi_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;
def int_fgpu_sldi_w : GCCBuiltin<"__builtin_msa_sldi_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;
def int_fgpu_sldi_d : GCCBuiltin<"__builtin_msa_sldi_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_i32_ty],
            [IntrNoMem, ImmArg<ArgIndex<2>>]>;

def int_fgpu_sll_b : GCCBuiltin<"__builtin_msa_sll_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_sll_h : GCCBuiltin<"__builtin_msa_sll_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_sll_w : GCCBuiltin<"__builtin_msa_sll_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_sll_d : GCCBuiltin<"__builtin_msa_sll_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_slli_b : GCCBuiltin<"__builtin_msa_slli_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_slli_h : GCCBuiltin<"__builtin_msa_slli_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_slli_w : GCCBuiltin<"__builtin_msa_slli_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_slli_d : GCCBuiltin<"__builtin_msa_slli_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_splat_b : GCCBuiltin<"__builtin_msa_splat_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_splat_h : GCCBuiltin<"__builtin_msa_splat_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_splat_w : GCCBuiltin<"__builtin_msa_splat_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_fgpu_splat_d : GCCBuiltin<"__builtin_msa_splat_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_fgpu_splati_b : GCCBuiltin<"__builtin_msa_splati_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_splati_h : GCCBuiltin<"__builtin_msa_splati_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_splati_w : GCCBuiltin<"__builtin_msa_splati_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_splati_d : GCCBuiltin<"__builtin_msa_splati_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_sra_b : GCCBuiltin<"__builtin_msa_sra_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_sra_h : GCCBuiltin<"__builtin_msa_sra_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_sra_w : GCCBuiltin<"__builtin_msa_sra_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_sra_d : GCCBuiltin<"__builtin_msa_sra_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_srai_b : GCCBuiltin<"__builtin_msa_srai_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_srai_h : GCCBuiltin<"__builtin_msa_srai_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_srai_w : GCCBuiltin<"__builtin_msa_srai_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_srai_d : GCCBuiltin<"__builtin_msa_srai_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_srar_b : GCCBuiltin<"__builtin_msa_srar_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_srar_h : GCCBuiltin<"__builtin_msa_srar_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_srar_w : GCCBuiltin<"__builtin_msa_srar_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_srar_d : GCCBuiltin<"__builtin_msa_srar_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_srari_b : GCCBuiltin<"__builtin_msa_srari_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_srari_h : GCCBuiltin<"__builtin_msa_srari_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_srari_w : GCCBuiltin<"__builtin_msa_srari_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_srari_d : GCCBuiltin<"__builtin_msa_srari_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_srl_b : GCCBuiltin<"__builtin_msa_srl_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_srl_h : GCCBuiltin<"__builtin_msa_srl_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_srl_w : GCCBuiltin<"__builtin_msa_srl_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_srl_d : GCCBuiltin<"__builtin_msa_srl_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_srli_b : GCCBuiltin<"__builtin_msa_srli_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_srli_h : GCCBuiltin<"__builtin_msa_srli_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_srli_w : GCCBuiltin<"__builtin_msa_srli_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_srli_d : GCCBuiltin<"__builtin_msa_srli_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_srlr_b : GCCBuiltin<"__builtin_msa_srlr_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_srlr_h : GCCBuiltin<"__builtin_msa_srlr_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_srlr_w : GCCBuiltin<"__builtin_msa_srlr_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_srlr_d : GCCBuiltin<"__builtin_msa_srlr_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_srlri_b : GCCBuiltin<"__builtin_msa_srlri_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_srlri_h : GCCBuiltin<"__builtin_msa_srlri_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_srlri_w : GCCBuiltin<"__builtin_msa_srlri_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_srlri_d : GCCBuiltin<"__builtin_msa_srlri_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_st_b : GCCBuiltin<"__builtin_msa_st_b">,
  Intrinsic<[], [llvm_v16i8_ty, llvm_ptr_ty, llvm_i32_ty],
  [IntrArgMemOnly]>;
def int_fgpu_st_h : GCCBuiltin<"__builtin_msa_st_h">,
  Intrinsic<[], [llvm_v8i16_ty, llvm_ptr_ty, llvm_i32_ty],
  [IntrArgMemOnly]>;
def int_fgpu_st_w : GCCBuiltin<"__builtin_msa_st_w">,
  Intrinsic<[], [llvm_v4i32_ty, llvm_ptr_ty, llvm_i32_ty],
  [IntrArgMemOnly]>;
def int_fgpu_st_d : GCCBuiltin<"__builtin_msa_st_d">,
  Intrinsic<[], [llvm_v2i64_ty, llvm_ptr_ty, llvm_i32_ty],
  [IntrArgMemOnly]>;

def int_fgpu_str_d : GCCBuiltin<"__builtin_msa_str_d">,
  Intrinsic<[], [llvm_v2i64_ty, llvm_ptr_ty, llvm_i32_ty],
  [IntrArgMemOnly]>;
def int_fgpu_str_w : GCCBuiltin<"__builtin_msa_str_w">,
  Intrinsic<[], [llvm_v4i32_ty, llvm_ptr_ty, llvm_i32_ty],
  [IntrArgMemOnly]>;

def int_fgpu_subs_s_b : GCCBuiltin<"__builtin_msa_subs_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_subs_s_h : GCCBuiltin<"__builtin_msa_subs_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_subs_s_w : GCCBuiltin<"__builtin_msa_subs_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_subs_s_d : GCCBuiltin<"__builtin_msa_subs_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_subs_u_b : GCCBuiltin<"__builtin_msa_subs_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_subs_u_h : GCCBuiltin<"__builtin_msa_subs_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_subs_u_w : GCCBuiltin<"__builtin_msa_subs_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_subs_u_d : GCCBuiltin<"__builtin_msa_subs_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_subsus_u_b : GCCBuiltin<"__builtin_msa_subsus_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_subsus_u_h : GCCBuiltin<"__builtin_msa_subsus_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_subsus_u_w : GCCBuiltin<"__builtin_msa_subsus_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_subsus_u_d : GCCBuiltin<"__builtin_msa_subsus_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_subsuu_s_b : GCCBuiltin<"__builtin_msa_subsuu_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_subsuu_s_h : GCCBuiltin<"__builtin_msa_subsuu_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_subsuu_s_w : GCCBuiltin<"__builtin_msa_subsuu_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_subsuu_s_d : GCCBuiltin<"__builtin_msa_subsuu_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_subv_b : GCCBuiltin<"__builtin_msa_subv_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_fgpu_subv_h : GCCBuiltin<"__builtin_msa_subv_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_fgpu_subv_w : GCCBuiltin<"__builtin_msa_subv_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_fgpu_subv_d : GCCBuiltin<"__builtin_msa_subv_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_fgpu_subvi_b : GCCBuiltin<"__builtin_msa_subvi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_subvi_h : GCCBuiltin<"__builtin_msa_subvi_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_subvi_w : GCCBuiltin<"__builtin_msa_subvi_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
def int_fgpu_subvi_d : GCCBuiltin<"__builtin_msa_subvi_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;

def int_fgpu_vshf_b : GCCBuiltin<"__builtin_msa_vshf_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
            [IntrNoMem]>;
def int_fgpu_vshf_h : GCCBuiltin<"__builtin_msa_vshf_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
            [IntrNoMem]>;
def int_fgpu_vshf_w : GCCBuiltin<"__builtin_msa_vshf_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
            [IntrNoMem]>;
def int_fgpu_vshf_d : GCCBuiltin<"__builtin_msa_vshf_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_v2i64_ty],
            [IntrNoMem]>;

def int_fgpu_xor_v : GCCBuiltin<"__builtin_msa_xor_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;

def int_fgpu_xori_b : GCCBuiltin<"__builtin_msa_xori_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem, ImmArg<ArgIndex<1>>]>;
}
