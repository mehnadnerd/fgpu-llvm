; NOTE: Assertions have been autogenerated by utils/update_mir_test_checks.py
; RUN: llc -global-isel -amdgpu-fixed-function-abi -stop-after=irtranslator -mtriple=amdgcn-amd-amdhsa -mcpu=gfx900 -verify-machineinstrs -o - %s | FileCheck -enable-var-scope %s

; Test that we don't insert code to pass implicit arguments we know
; the callee does not need.

declare hidden void @extern()

define amdgpu_kernel void @kernel_call_no_workitem_ids() {
  ; CHECK-LABEL: name: kernel_call_no_workitem_ids
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK:   liveins: $sgpr12, $sgpr13, $sgpr14, $vgpr0, $vgpr1, $vgpr2, $sgpr4_sgpr5, $sgpr6_sgpr7, $sgpr8_sgpr9
  ; CHECK:   [[COPY:%[0-9]+]]:vgpr_32(s32) = COPY $vgpr2
  ; CHECK:   [[COPY1:%[0-9]+]]:vgpr_32(s32) = COPY $vgpr1
  ; CHECK:   [[COPY2:%[0-9]+]]:vgpr_32(s32) = COPY $vgpr0
  ; CHECK:   [[COPY3:%[0-9]+]]:sgpr_32 = COPY $sgpr14
  ; CHECK:   [[COPY4:%[0-9]+]]:sgpr_32 = COPY $sgpr13
  ; CHECK:   [[COPY5:%[0-9]+]]:sgpr_32 = COPY $sgpr12
  ; CHECK:   [[COPY6:%[0-9]+]]:sgpr_64 = COPY $sgpr8_sgpr9
  ; CHECK:   [[COPY7:%[0-9]+]]:sgpr_64 = COPY $sgpr6_sgpr7
  ; CHECK:   [[COPY8:%[0-9]+]]:sgpr_64 = COPY $sgpr4_sgpr5
  ; CHECK:   ADJCALLSTACKUP 0, 0, implicit-def $scc
  ; CHECK:   [[GV:%[0-9]+]]:sreg_64(p0) = G_GLOBAL_VALUE @extern
  ; CHECK:   [[COPY9:%[0-9]+]]:_(p4) = COPY [[COPY8]]
  ; CHECK:   [[COPY10:%[0-9]+]]:_(p4) = COPY [[COPY7]]
  ; CHECK:   [[C:%[0-9]+]]:_(p4) = G_CONSTANT i64 0
  ; CHECK:   [[C1:%[0-9]+]]:_(s64) = G_CONSTANT i64 0
  ; CHECK:   [[PTR_ADD:%[0-9]+]]:_(p4) = G_PTR_ADD [[C]], [[C1]](s64)
  ; CHECK:   [[COPY11:%[0-9]+]]:_(s64) = COPY [[COPY6]]
  ; CHECK:   [[COPY12:%[0-9]+]]:_(s32) = COPY [[COPY5]]
  ; CHECK:   [[COPY13:%[0-9]+]]:_(s32) = COPY [[COPY4]]
  ; CHECK:   [[COPY14:%[0-9]+]]:_(s32) = COPY [[COPY3]]
  ; CHECK:   [[COPY15:%[0-9]+]]:_(s32) = COPY [[COPY2]](s32)
  ; CHECK:   [[COPY16:%[0-9]+]]:_(s32) = COPY [[COPY1]](s32)
  ; CHECK:   [[C2:%[0-9]+]]:_(s32) = G_CONSTANT i32 10
  ; CHECK:   [[SHL:%[0-9]+]]:_(s32) = G_SHL [[COPY16]], [[C2]](s32)
  ; CHECK:   [[OR:%[0-9]+]]:_(s32) = G_OR [[COPY15]], [[SHL]]
  ; CHECK:   [[COPY17:%[0-9]+]]:_(s32) = COPY [[COPY]](s32)
  ; CHECK:   [[C3:%[0-9]+]]:_(s32) = G_CONSTANT i32 20
  ; CHECK:   [[SHL1:%[0-9]+]]:_(s32) = G_SHL [[COPY17]], [[C3]](s32)
  ; CHECK:   [[OR1:%[0-9]+]]:_(s32) = G_OR [[OR]], [[SHL1]]
  ; CHECK:   [[COPY18:%[0-9]+]]:_(<4 x s32>) = COPY $private_rsrc_reg
  ; CHECK:   $sgpr0_sgpr1_sgpr2_sgpr3 = COPY [[COPY18]](<4 x s32>)
  ; CHECK:   $sgpr4_sgpr5 = COPY [[COPY9]](p4)
  ; CHECK:   $sgpr6_sgpr7 = COPY [[COPY10]](p4)
  ; CHECK:   $sgpr8_sgpr9 = COPY [[PTR_ADD]](p4)
  ; CHECK:   $sgpr10_sgpr11 = COPY [[COPY11]](s64)
  ; CHECK:   $sgpr12 = COPY [[COPY12]](s32)
  ; CHECK:   $sgpr13 = COPY [[COPY13]](s32)
  ; CHECK:   $sgpr14 = COPY [[COPY14]](s32)
  ; CHECK:   $vgpr31 = COPY [[OR1]](s32)
  ; CHECK:   $sgpr30_sgpr31 = SI_CALL [[GV]](p0), @extern, csr_amdgpu_highregs, implicit $sgpr0_sgpr1_sgpr2_sgpr3, implicit $sgpr4_sgpr5, implicit $sgpr6_sgpr7, implicit $sgpr8_sgpr9, implicit $sgpr10_sgpr11, implicit $sgpr12, implicit $sgpr13, implicit $sgpr14, implicit $vgpr31
  ; CHECK:   ADJCALLSTACKDOWN 0, 0, implicit-def $scc
  ; CHECK:   S_ENDPGM 0
  call void @extern() "amdgpu-no-workitem-id-x" "amdgpu-no-workitem-id-y" "amdgpu-no-workitem-id-z"
  ret void
}

define amdgpu_kernel void @kernel_call_no_workgroup_ids() {
  ; CHECK-LABEL: name: kernel_call_no_workgroup_ids
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK:   liveins: $sgpr12, $sgpr13, $sgpr14, $vgpr0, $vgpr1, $vgpr2, $sgpr4_sgpr5, $sgpr6_sgpr7, $sgpr8_sgpr9
  ; CHECK:   [[COPY:%[0-9]+]]:vgpr_32(s32) = COPY $vgpr2
  ; CHECK:   [[COPY1:%[0-9]+]]:vgpr_32(s32) = COPY $vgpr1
  ; CHECK:   [[COPY2:%[0-9]+]]:vgpr_32(s32) = COPY $vgpr0
  ; CHECK:   [[COPY3:%[0-9]+]]:sgpr_32 = COPY $sgpr14
  ; CHECK:   [[COPY4:%[0-9]+]]:sgpr_32 = COPY $sgpr13
  ; CHECK:   [[COPY5:%[0-9]+]]:sgpr_32 = COPY $sgpr12
  ; CHECK:   [[COPY6:%[0-9]+]]:sgpr_64 = COPY $sgpr8_sgpr9
  ; CHECK:   [[COPY7:%[0-9]+]]:sgpr_64 = COPY $sgpr6_sgpr7
  ; CHECK:   [[COPY8:%[0-9]+]]:sgpr_64 = COPY $sgpr4_sgpr5
  ; CHECK:   ADJCALLSTACKUP 0, 0, implicit-def $scc
  ; CHECK:   [[GV:%[0-9]+]]:sreg_64(p0) = G_GLOBAL_VALUE @extern
  ; CHECK:   [[COPY9:%[0-9]+]]:_(p4) = COPY [[COPY8]]
  ; CHECK:   [[COPY10:%[0-9]+]]:_(p4) = COPY [[COPY7]]
  ; CHECK:   [[C:%[0-9]+]]:_(p4) = G_CONSTANT i64 0
  ; CHECK:   [[C1:%[0-9]+]]:_(s64) = G_CONSTANT i64 0
  ; CHECK:   [[PTR_ADD:%[0-9]+]]:_(p4) = G_PTR_ADD [[C]], [[C1]](s64)
  ; CHECK:   [[COPY11:%[0-9]+]]:_(s64) = COPY [[COPY6]]
  ; CHECK:   [[COPY12:%[0-9]+]]:_(s32) = COPY [[COPY5]]
  ; CHECK:   [[COPY13:%[0-9]+]]:_(s32) = COPY [[COPY4]]
  ; CHECK:   [[COPY14:%[0-9]+]]:_(s32) = COPY [[COPY3]]
  ; CHECK:   [[COPY15:%[0-9]+]]:_(s32) = COPY [[COPY2]](s32)
  ; CHECK:   [[COPY16:%[0-9]+]]:_(s32) = COPY [[COPY1]](s32)
  ; CHECK:   [[C2:%[0-9]+]]:_(s32) = G_CONSTANT i32 10
  ; CHECK:   [[SHL:%[0-9]+]]:_(s32) = G_SHL [[COPY16]], [[C2]](s32)
  ; CHECK:   [[OR:%[0-9]+]]:_(s32) = G_OR [[COPY15]], [[SHL]]
  ; CHECK:   [[COPY17:%[0-9]+]]:_(s32) = COPY [[COPY]](s32)
  ; CHECK:   [[C3:%[0-9]+]]:_(s32) = G_CONSTANT i32 20
  ; CHECK:   [[SHL1:%[0-9]+]]:_(s32) = G_SHL [[COPY17]], [[C3]](s32)
  ; CHECK:   [[OR1:%[0-9]+]]:_(s32) = G_OR [[OR]], [[SHL1]]
  ; CHECK:   [[COPY18:%[0-9]+]]:_(<4 x s32>) = COPY $private_rsrc_reg
  ; CHECK:   $sgpr0_sgpr1_sgpr2_sgpr3 = COPY [[COPY18]](<4 x s32>)
  ; CHECK:   $sgpr4_sgpr5 = COPY [[COPY9]](p4)
  ; CHECK:   $sgpr6_sgpr7 = COPY [[COPY10]](p4)
  ; CHECK:   $sgpr8_sgpr9 = COPY [[PTR_ADD]](p4)
  ; CHECK:   $sgpr10_sgpr11 = COPY [[COPY11]](s64)
  ; CHECK:   $sgpr12 = COPY [[COPY12]](s32)
  ; CHECK:   $sgpr13 = COPY [[COPY13]](s32)
  ; CHECK:   $sgpr14 = COPY [[COPY14]](s32)
  ; CHECK:   $vgpr31 = COPY [[OR1]](s32)
  ; CHECK:   $sgpr30_sgpr31 = SI_CALL [[GV]](p0), @extern, csr_amdgpu_highregs, implicit $sgpr0_sgpr1_sgpr2_sgpr3, implicit $sgpr4_sgpr5, implicit $sgpr6_sgpr7, implicit $sgpr8_sgpr9, implicit $sgpr10_sgpr11, implicit $sgpr12, implicit $sgpr13, implicit $sgpr14, implicit $vgpr31
  ; CHECK:   ADJCALLSTACKDOWN 0, 0, implicit-def $scc
  ; CHECK:   S_ENDPGM 0
  call void @extern() "amdgpu-no-workgroup-id-x" "amdgpu-no-workgroup-id-y" "amdgpu-no-workgroup-id-z"
  ret void
}

define amdgpu_kernel void @kernel_call_no_other_sgprs() {
  ; CHECK-LABEL: name: kernel_call_no_other_sgprs
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK:   liveins: $sgpr12, $sgpr13, $sgpr14, $vgpr0, $vgpr1, $vgpr2, $sgpr4_sgpr5, $sgpr6_sgpr7, $sgpr8_sgpr9
  ; CHECK:   [[COPY:%[0-9]+]]:vgpr_32(s32) = COPY $vgpr2
  ; CHECK:   [[COPY1:%[0-9]+]]:vgpr_32(s32) = COPY $vgpr1
  ; CHECK:   [[COPY2:%[0-9]+]]:vgpr_32(s32) = COPY $vgpr0
  ; CHECK:   [[COPY3:%[0-9]+]]:sgpr_32 = COPY $sgpr14
  ; CHECK:   [[COPY4:%[0-9]+]]:sgpr_32 = COPY $sgpr13
  ; CHECK:   [[COPY5:%[0-9]+]]:sgpr_32 = COPY $sgpr12
  ; CHECK:   [[COPY6:%[0-9]+]]:sgpr_64 = COPY $sgpr8_sgpr9
  ; CHECK:   [[COPY7:%[0-9]+]]:sgpr_64 = COPY $sgpr6_sgpr7
  ; CHECK:   [[COPY8:%[0-9]+]]:sgpr_64 = COPY $sgpr4_sgpr5
  ; CHECK:   ADJCALLSTACKUP 0, 0, implicit-def $scc
  ; CHECK:   [[GV:%[0-9]+]]:sreg_64(p0) = G_GLOBAL_VALUE @extern
  ; CHECK:   [[COPY9:%[0-9]+]]:_(p4) = COPY [[COPY8]]
  ; CHECK:   [[COPY10:%[0-9]+]]:_(p4) = COPY [[COPY7]]
  ; CHECK:   [[C:%[0-9]+]]:_(p4) = G_CONSTANT i64 0
  ; CHECK:   [[C1:%[0-9]+]]:_(s64) = G_CONSTANT i64 0
  ; CHECK:   [[PTR_ADD:%[0-9]+]]:_(p4) = G_PTR_ADD [[C]], [[C1]](s64)
  ; CHECK:   [[COPY11:%[0-9]+]]:_(s64) = COPY [[COPY6]]
  ; CHECK:   [[COPY12:%[0-9]+]]:_(s32) = COPY [[COPY5]]
  ; CHECK:   [[COPY13:%[0-9]+]]:_(s32) = COPY [[COPY4]]
  ; CHECK:   [[COPY14:%[0-9]+]]:_(s32) = COPY [[COPY3]]
  ; CHECK:   [[COPY15:%[0-9]+]]:_(s32) = COPY [[COPY2]](s32)
  ; CHECK:   [[COPY16:%[0-9]+]]:_(s32) = COPY [[COPY1]](s32)
  ; CHECK:   [[C2:%[0-9]+]]:_(s32) = G_CONSTANT i32 10
  ; CHECK:   [[SHL:%[0-9]+]]:_(s32) = G_SHL [[COPY16]], [[C2]](s32)
  ; CHECK:   [[OR:%[0-9]+]]:_(s32) = G_OR [[COPY15]], [[SHL]]
  ; CHECK:   [[COPY17:%[0-9]+]]:_(s32) = COPY [[COPY]](s32)
  ; CHECK:   [[C3:%[0-9]+]]:_(s32) = G_CONSTANT i32 20
  ; CHECK:   [[SHL1:%[0-9]+]]:_(s32) = G_SHL [[COPY17]], [[C3]](s32)
  ; CHECK:   [[OR1:%[0-9]+]]:_(s32) = G_OR [[OR]], [[SHL1]]
  ; CHECK:   [[COPY18:%[0-9]+]]:_(<4 x s32>) = COPY $private_rsrc_reg
  ; CHECK:   $sgpr0_sgpr1_sgpr2_sgpr3 = COPY [[COPY18]](<4 x s32>)
  ; CHECK:   $sgpr4_sgpr5 = COPY [[COPY9]](p4)
  ; CHECK:   $sgpr6_sgpr7 = COPY [[COPY10]](p4)
  ; CHECK:   $sgpr8_sgpr9 = COPY [[PTR_ADD]](p4)
  ; CHECK:   $sgpr10_sgpr11 = COPY [[COPY11]](s64)
  ; CHECK:   $sgpr12 = COPY [[COPY12]](s32)
  ; CHECK:   $sgpr13 = COPY [[COPY13]](s32)
  ; CHECK:   $sgpr14 = COPY [[COPY14]](s32)
  ; CHECK:   $vgpr31 = COPY [[OR1]](s32)
  ; CHECK:   $sgpr30_sgpr31 = SI_CALL [[GV]](p0), @extern, csr_amdgpu_highregs, implicit $sgpr0_sgpr1_sgpr2_sgpr3, implicit $sgpr4_sgpr5, implicit $sgpr6_sgpr7, implicit $sgpr8_sgpr9, implicit $sgpr10_sgpr11, implicit $sgpr12, implicit $sgpr13, implicit $sgpr14, implicit $vgpr31
  ; CHECK:   ADJCALLSTACKDOWN 0, 0, implicit-def $scc
  ; CHECK:   S_ENDPGM 0
  call void @extern() "amdgpu-no-dispatch-id" "amdgpu-no-dispatch-ptr" "amdgpu-no-queue-ptr" "amdgpu-no-workgroup-id-x" "amdgpu-no-workgroup-id-y" "amdgpu-no-workgroup-id-z"
  ret void
}

define void @func_call_no_workitem_ids() {
  ; CHECK-LABEL: name: func_call_no_workitem_ids
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK:   liveins: $sgpr12, $sgpr13, $sgpr14, $vgpr31, $sgpr4_sgpr5, $sgpr6_sgpr7, $sgpr8_sgpr9, $sgpr10_sgpr11, $sgpr30_sgpr31
  ; CHECK:   [[COPY:%[0-9]+]]:vgpr_32(s32) = COPY $vgpr31
  ; CHECK:   [[COPY1:%[0-9]+]]:sgpr_32 = COPY $sgpr14
  ; CHECK:   [[COPY2:%[0-9]+]]:sgpr_32 = COPY $sgpr13
  ; CHECK:   [[COPY3:%[0-9]+]]:sgpr_32 = COPY $sgpr12
  ; CHECK:   [[COPY4:%[0-9]+]]:sgpr_64 = COPY $sgpr10_sgpr11
  ; CHECK:   [[COPY5:%[0-9]+]]:sgpr_64 = COPY $sgpr8_sgpr9
  ; CHECK:   [[COPY6:%[0-9]+]]:sgpr_64 = COPY $sgpr6_sgpr7
  ; CHECK:   [[COPY7:%[0-9]+]]:sgpr_64 = COPY $sgpr4_sgpr5
  ; CHECK:   [[COPY8:%[0-9]+]]:sgpr_64 = COPY $sgpr30_sgpr31
  ; CHECK:   ADJCALLSTACKUP 0, 0, implicit-def $scc
  ; CHECK:   [[GV:%[0-9]+]]:sreg_64(p0) = G_GLOBAL_VALUE @extern
  ; CHECK:   [[COPY9:%[0-9]+]]:_(p4) = COPY [[COPY7]]
  ; CHECK:   [[COPY10:%[0-9]+]]:_(p4) = COPY [[COPY6]]
  ; CHECK:   [[COPY11:%[0-9]+]]:_(p4) = COPY [[COPY5]]
  ; CHECK:   [[COPY12:%[0-9]+]]:_(s64) = COPY [[COPY4]]
  ; CHECK:   [[COPY13:%[0-9]+]]:_(s32) = COPY [[COPY3]]
  ; CHECK:   [[COPY14:%[0-9]+]]:_(s32) = COPY [[COPY2]]
  ; CHECK:   [[COPY15:%[0-9]+]]:_(s32) = COPY [[COPY1]]
  ; CHECK:   [[COPY16:%[0-9]+]]:_(s32) = COPY [[COPY]](s32)
  ; CHECK:   [[COPY17:%[0-9]+]]:_(<4 x s32>) = COPY $sgpr0_sgpr1_sgpr2_sgpr3
  ; CHECK:   $sgpr0_sgpr1_sgpr2_sgpr3 = COPY [[COPY17]](<4 x s32>)
  ; CHECK:   $sgpr4_sgpr5 = COPY [[COPY9]](p4)
  ; CHECK:   $sgpr6_sgpr7 = COPY [[COPY10]](p4)
  ; CHECK:   $sgpr8_sgpr9 = COPY [[COPY11]](p4)
  ; CHECK:   $sgpr10_sgpr11 = COPY [[COPY12]](s64)
  ; CHECK:   $sgpr12 = COPY [[COPY13]](s32)
  ; CHECK:   $sgpr13 = COPY [[COPY14]](s32)
  ; CHECK:   $sgpr14 = COPY [[COPY15]](s32)
  ; CHECK:   $vgpr31 = COPY [[COPY16]](s32)
  ; CHECK:   $sgpr30_sgpr31 = SI_CALL [[GV]](p0), @extern, csr_amdgpu_highregs, implicit $sgpr0_sgpr1_sgpr2_sgpr3, implicit $sgpr4_sgpr5, implicit $sgpr6_sgpr7, implicit $sgpr8_sgpr9, implicit $sgpr10_sgpr11, implicit $sgpr12, implicit $sgpr13, implicit $sgpr14, implicit $vgpr31
  ; CHECK:   ADJCALLSTACKDOWN 0, 0, implicit-def $scc
  ; CHECK:   [[COPY18:%[0-9]+]]:ccr_sgpr_64 = COPY [[COPY8]]
  ; CHECK:   S_SETPC_B64_return [[COPY18]]
  call void @extern() "amdgpu-no-workitem-id-x" "amdgpu-no-workitem-id-y" "amdgpu-no-workitem-id-z"
  ret void
}

define void @func_call_no_workgroup_ids() {
  ; CHECK-LABEL: name: func_call_no_workgroup_ids
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK:   liveins: $sgpr12, $sgpr13, $sgpr14, $vgpr31, $sgpr4_sgpr5, $sgpr6_sgpr7, $sgpr8_sgpr9, $sgpr10_sgpr11, $sgpr30_sgpr31
  ; CHECK:   [[COPY:%[0-9]+]]:vgpr_32(s32) = COPY $vgpr31
  ; CHECK:   [[COPY1:%[0-9]+]]:sgpr_32 = COPY $sgpr14
  ; CHECK:   [[COPY2:%[0-9]+]]:sgpr_32 = COPY $sgpr13
  ; CHECK:   [[COPY3:%[0-9]+]]:sgpr_32 = COPY $sgpr12
  ; CHECK:   [[COPY4:%[0-9]+]]:sgpr_64 = COPY $sgpr10_sgpr11
  ; CHECK:   [[COPY5:%[0-9]+]]:sgpr_64 = COPY $sgpr8_sgpr9
  ; CHECK:   [[COPY6:%[0-9]+]]:sgpr_64 = COPY $sgpr6_sgpr7
  ; CHECK:   [[COPY7:%[0-9]+]]:sgpr_64 = COPY $sgpr4_sgpr5
  ; CHECK:   [[COPY8:%[0-9]+]]:sgpr_64 = COPY $sgpr30_sgpr31
  ; CHECK:   ADJCALLSTACKUP 0, 0, implicit-def $scc
  ; CHECK:   [[GV:%[0-9]+]]:sreg_64(p0) = G_GLOBAL_VALUE @extern
  ; CHECK:   [[COPY9:%[0-9]+]]:_(p4) = COPY [[COPY7]]
  ; CHECK:   [[COPY10:%[0-9]+]]:_(p4) = COPY [[COPY6]]
  ; CHECK:   [[COPY11:%[0-9]+]]:_(p4) = COPY [[COPY5]]
  ; CHECK:   [[COPY12:%[0-9]+]]:_(s64) = COPY [[COPY4]]
  ; CHECK:   [[COPY13:%[0-9]+]]:_(s32) = COPY [[COPY3]]
  ; CHECK:   [[COPY14:%[0-9]+]]:_(s32) = COPY [[COPY2]]
  ; CHECK:   [[COPY15:%[0-9]+]]:_(s32) = COPY [[COPY1]]
  ; CHECK:   [[COPY16:%[0-9]+]]:_(s32) = COPY [[COPY]](s32)
  ; CHECK:   [[COPY17:%[0-9]+]]:_(<4 x s32>) = COPY $sgpr0_sgpr1_sgpr2_sgpr3
  ; CHECK:   $sgpr0_sgpr1_sgpr2_sgpr3 = COPY [[COPY17]](<4 x s32>)
  ; CHECK:   $sgpr4_sgpr5 = COPY [[COPY9]](p4)
  ; CHECK:   $sgpr6_sgpr7 = COPY [[COPY10]](p4)
  ; CHECK:   $sgpr8_sgpr9 = COPY [[COPY11]](p4)
  ; CHECK:   $sgpr10_sgpr11 = COPY [[COPY12]](s64)
  ; CHECK:   $sgpr12 = COPY [[COPY13]](s32)
  ; CHECK:   $sgpr13 = COPY [[COPY14]](s32)
  ; CHECK:   $sgpr14 = COPY [[COPY15]](s32)
  ; CHECK:   $vgpr31 = COPY [[COPY16]](s32)
  ; CHECK:   $sgpr30_sgpr31 = SI_CALL [[GV]](p0), @extern, csr_amdgpu_highregs, implicit $sgpr0_sgpr1_sgpr2_sgpr3, implicit $sgpr4_sgpr5, implicit $sgpr6_sgpr7, implicit $sgpr8_sgpr9, implicit $sgpr10_sgpr11, implicit $sgpr12, implicit $sgpr13, implicit $sgpr14, implicit $vgpr31
  ; CHECK:   ADJCALLSTACKDOWN 0, 0, implicit-def $scc
  ; CHECK:   [[COPY18:%[0-9]+]]:ccr_sgpr_64 = COPY [[COPY8]]
  ; CHECK:   S_SETPC_B64_return [[COPY18]]
  call void @extern() "amdgpu-no-workgroup-id-x" "amdgpu-no-workgroup-id-y" "amdgpu-no-workgroup-id-z"
  ret void
}

define void @func_call_no_other_sgprs() {
  ; CHECK-LABEL: name: func_call_no_other_sgprs
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK:   liveins: $sgpr12, $sgpr13, $sgpr14, $vgpr31, $sgpr4_sgpr5, $sgpr6_sgpr7, $sgpr8_sgpr9, $sgpr10_sgpr11, $sgpr30_sgpr31
  ; CHECK:   [[COPY:%[0-9]+]]:vgpr_32(s32) = COPY $vgpr31
  ; CHECK:   [[COPY1:%[0-9]+]]:sgpr_32 = COPY $sgpr14
  ; CHECK:   [[COPY2:%[0-9]+]]:sgpr_32 = COPY $sgpr13
  ; CHECK:   [[COPY3:%[0-9]+]]:sgpr_32 = COPY $sgpr12
  ; CHECK:   [[COPY4:%[0-9]+]]:sgpr_64 = COPY $sgpr10_sgpr11
  ; CHECK:   [[COPY5:%[0-9]+]]:sgpr_64 = COPY $sgpr8_sgpr9
  ; CHECK:   [[COPY6:%[0-9]+]]:sgpr_64 = COPY $sgpr6_sgpr7
  ; CHECK:   [[COPY7:%[0-9]+]]:sgpr_64 = COPY $sgpr4_sgpr5
  ; CHECK:   [[COPY8:%[0-9]+]]:sgpr_64 = COPY $sgpr30_sgpr31
  ; CHECK:   ADJCALLSTACKUP 0, 0, implicit-def $scc
  ; CHECK:   [[GV:%[0-9]+]]:sreg_64(p0) = G_GLOBAL_VALUE @extern
  ; CHECK:   [[COPY9:%[0-9]+]]:_(p4) = COPY [[COPY7]]
  ; CHECK:   [[COPY10:%[0-9]+]]:_(p4) = COPY [[COPY6]]
  ; CHECK:   [[COPY11:%[0-9]+]]:_(p4) = COPY [[COPY5]]
  ; CHECK:   [[COPY12:%[0-9]+]]:_(s64) = COPY [[COPY4]]
  ; CHECK:   [[COPY13:%[0-9]+]]:_(s32) = COPY [[COPY3]]
  ; CHECK:   [[COPY14:%[0-9]+]]:_(s32) = COPY [[COPY2]]
  ; CHECK:   [[COPY15:%[0-9]+]]:_(s32) = COPY [[COPY1]]
  ; CHECK:   [[COPY16:%[0-9]+]]:_(s32) = COPY [[COPY]](s32)
  ; CHECK:   [[COPY17:%[0-9]+]]:_(<4 x s32>) = COPY $sgpr0_sgpr1_sgpr2_sgpr3
  ; CHECK:   $sgpr0_sgpr1_sgpr2_sgpr3 = COPY [[COPY17]](<4 x s32>)
  ; CHECK:   $sgpr4_sgpr5 = COPY [[COPY9]](p4)
  ; CHECK:   $sgpr6_sgpr7 = COPY [[COPY10]](p4)
  ; CHECK:   $sgpr8_sgpr9 = COPY [[COPY11]](p4)
  ; CHECK:   $sgpr10_sgpr11 = COPY [[COPY12]](s64)
  ; CHECK:   $sgpr12 = COPY [[COPY13]](s32)
  ; CHECK:   $sgpr13 = COPY [[COPY14]](s32)
  ; CHECK:   $sgpr14 = COPY [[COPY15]](s32)
  ; CHECK:   $vgpr31 = COPY [[COPY16]](s32)
  ; CHECK:   $sgpr30_sgpr31 = SI_CALL [[GV]](p0), @extern, csr_amdgpu_highregs, implicit $sgpr0_sgpr1_sgpr2_sgpr3, implicit $sgpr4_sgpr5, implicit $sgpr6_sgpr7, implicit $sgpr8_sgpr9, implicit $sgpr10_sgpr11, implicit $sgpr12, implicit $sgpr13, implicit $sgpr14, implicit $vgpr31
  ; CHECK:   ADJCALLSTACKDOWN 0, 0, implicit-def $scc
  ; CHECK:   [[COPY18:%[0-9]+]]:ccr_sgpr_64 = COPY [[COPY8]]
  ; CHECK:   S_SETPC_B64_return [[COPY18]]
  call void @extern() "amdgpu-no-dispatch-id" "amdgpu-no-dispatch-ptr" "amdgpu-no-queue-ptr" "amdgpu-no-workgroup-id-x" "amdgpu-no-workgroup-id-y" "amdgpu-no-workgroup-id-z"
  ret void
}
