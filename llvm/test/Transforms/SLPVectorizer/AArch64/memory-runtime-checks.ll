; NOTE: Assertions have been autogenerated by utils/update_test_checks.py
; RUN: opt -scoped-noalias-aa -slp-vectorizer -mtriple=arm64-apple-darwin -enable-new-pm=false -S %s | FileCheck %s

define void @needs_versioning_not_profitable(i32* %dst, i32* %src) {
; CHECK-LABEL: @needs_versioning_not_profitable(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[SRC_0:%.*]] = load i32, i32* [[SRC:%.*]], align 4
; CHECK-NEXT:    [[R_0:%.*]] = ashr i32 [[SRC_0]], 16
; CHECK-NEXT:    store i32 [[R_0]], i32* [[DST:%.*]], align 4
; CHECK-NEXT:    [[SRC_GEP_1:%.*]] = getelementptr inbounds i32, i32* [[SRC]], i64 1
; CHECK-NEXT:    [[SRC_1:%.*]] = load i32, i32* [[SRC_GEP_1]], align 4
; CHECK-NEXT:    [[R_1:%.*]] = ashr i32 [[SRC_1]], 16
; CHECK-NEXT:    [[DST_GEP_1:%.*]] = getelementptr inbounds i32, i32* [[DST]], i64 1
; CHECK-NEXT:    store i32 [[R_1]], i32* [[DST_GEP_1]], align 4
; CHECK-NEXT:    ret void
;
entry:
  %src.0 = load i32, i32* %src, align 4
  %r.0 = ashr i32 %src.0, 16
  store i32 %r.0, i32* %dst, align 4
  %src.gep.1 = getelementptr inbounds i32, i32* %src, i64 1
  %src.1 = load i32, i32* %src.gep.1, align 4
  %r.1 = ashr i32 %src.1, 16
  %dst.gep.1 = getelementptr inbounds i32, i32* %dst, i64 1
  store i32 %r.1, i32* %dst.gep.1, align 4
  ret void
}

define void @needs_versioning_profitable(i32* %dst, i32* %src) {
; CHECK-LABEL: @needs_versioning_profitable(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[SRC_0:%.*]] = load i32, i32* [[SRC:%.*]], align 4
; CHECK-NEXT:    [[R_0:%.*]] = ashr i32 [[SRC_0]], 16
; CHECK-NEXT:    store i32 [[R_0]], i32* [[DST:%.*]], align 4
; CHECK-NEXT:    [[SRC_GEP_1:%.*]] = getelementptr inbounds i32, i32* [[SRC]], i64 1
; CHECK-NEXT:    [[SRC_1:%.*]] = load i32, i32* [[SRC_GEP_1]], align 4
; CHECK-NEXT:    [[R_1:%.*]] = ashr i32 [[SRC_1]], 16
; CHECK-NEXT:    [[DST_GEP_1:%.*]] = getelementptr inbounds i32, i32* [[DST]], i64 1
; CHECK-NEXT:    store i32 [[R_1]], i32* [[DST_GEP_1]], align 4
; CHECK-NEXT:    [[SRC_GEP_2:%.*]] = getelementptr inbounds i32, i32* [[SRC]], i64 2
; CHECK-NEXT:    [[SRC_2:%.*]] = load i32, i32* [[SRC_GEP_2]], align 4
; CHECK-NEXT:    [[R_2:%.*]] = ashr i32 [[SRC_2]], 16
; CHECK-NEXT:    [[DST_GEP_2:%.*]] = getelementptr inbounds i32, i32* [[DST]], i64 2
; CHECK-NEXT:    store i32 [[R_2]], i32* [[DST_GEP_2]], align 4
; CHECK-NEXT:    [[SRC_GEP_3:%.*]] = getelementptr inbounds i32, i32* [[SRC]], i64 3
; CHECK-NEXT:    [[SRC_3:%.*]] = load i32, i32* [[SRC_GEP_3]], align 4
; CHECK-NEXT:    [[R_3:%.*]] = ashr i32 [[SRC_3]], 16
; CHECK-NEXT:    [[DST_GEP_3:%.*]] = getelementptr inbounds i32, i32* [[DST]], i64 3
; CHECK-NEXT:    store i32 [[R_3]], i32* [[DST_GEP_3]], align 4
; CHECK-NEXT:    ret void
;
entry:
  %src.0 = load i32, i32* %src, align 4
  %r.0 = ashr i32 %src.0, 16
  store i32 %r.0, i32* %dst, align 4
  %src.gep.1 = getelementptr inbounds i32, i32* %src, i64 1
  %src.1 = load i32, i32* %src.gep.1, align 4
  %r.1 = ashr i32 %src.1, 16
  %dst.gep.1 = getelementptr inbounds i32, i32* %dst, i64 1
  store i32 %r.1, i32* %dst.gep.1, align 4
  %src.gep.2 = getelementptr inbounds i32, i32* %src, i64 2
  %src.2 = load i32, i32* %src.gep.2, align 4
  %r.2 = ashr i32 %src.2, 16
  %dst.gep.2 = getelementptr inbounds i32, i32* %dst, i64 2
  store i32 %r.2, i32* %dst.gep.2, align 4
  %src.gep.3 = getelementptr inbounds i32, i32* %src, i64 3
  %src.3 = load i32, i32* %src.gep.3, align 4
  %r.3 = ashr i32 %src.3, 16
  %dst.gep.3 = getelementptr inbounds i32, i32* %dst, i64 3
  store i32 %r.3, i32* %dst.gep.3, align 4

  ret void
}


define void @no_version(i32* nocapture %dst, i32* nocapture readonly %src) {
; CHECK-LABEL: @no_version(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[SRC_GEP_1:%.*]] = getelementptr inbounds i32, i32* [[SRC:%.*]], i64 1
; CHECK-NEXT:    [[TMP0:%.*]] = bitcast i32* [[SRC]] to <2 x i32>*
; CHECK-NEXT:    [[TMP1:%.*]] = load <2 x i32>, <2 x i32>* [[TMP0]], align 4
; CHECK-NEXT:    [[TMP2:%.*]] = ashr <2 x i32> [[TMP1]], <i32 16, i32 16>
; CHECK-NEXT:    [[DST_GEP_1:%.*]] = getelementptr inbounds i32, i32* [[DST:%.*]], i64 1
; CHECK-NEXT:    [[TMP3:%.*]] = bitcast i32* [[DST]] to <2 x i32>*
; CHECK-NEXT:    store <2 x i32> [[TMP2]], <2 x i32>* [[TMP3]], align 4
; CHECK-NEXT:    ret void
;
entry:
  %src.0 = load i32, i32* %src, align 4
  %src.gep.1 = getelementptr inbounds i32, i32* %src, i64 1
  %src.1 = load i32, i32* %src.gep.1, align 4
  %r.0 = ashr i32 %src.0, 16
  %r.1 = ashr i32 %src.1, 16
  %dst.gep.1 = getelementptr inbounds i32, i32* %dst, i64 1
  store i32 %r.0, i32* %dst, align 4
  store i32 %r.1, i32* %dst.gep.1, align 4
  ret void
}

define void @version_multiple(i32* nocapture %out_block, i32* nocapture readonly %counter) {
; CHECK-LABEL: @version_multiple(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[TMP0:%.*]] = load i32, i32* [[COUNTER:%.*]], align 4
; CHECK-NEXT:    [[TMP1:%.*]] = load i32, i32* [[OUT_BLOCK:%.*]], align 4
; CHECK-NEXT:    [[XOR:%.*]] = xor i32 [[TMP1]], [[TMP0]]
; CHECK-NEXT:    store i32 [[XOR]], i32* [[OUT_BLOCK]], align 4
; CHECK-NEXT:    [[ARRAYIDX_1:%.*]] = getelementptr inbounds i32, i32* [[COUNTER]], i64 1
; CHECK-NEXT:    [[TMP2:%.*]] = load i32, i32* [[ARRAYIDX_1]], align 4
; CHECK-NEXT:    [[ARRAYIDX2_1:%.*]] = getelementptr inbounds i32, i32* [[OUT_BLOCK]], i64 1
; CHECK-NEXT:    [[TMP3:%.*]] = load i32, i32* [[ARRAYIDX2_1]], align 4
; CHECK-NEXT:    [[XOR_1:%.*]] = xor i32 [[TMP3]], [[TMP2]]
; CHECK-NEXT:    store i32 [[XOR_1]], i32* [[ARRAYIDX2_1]], align 4
; CHECK-NEXT:    [[ARRAYIDX_2:%.*]] = getelementptr inbounds i32, i32* [[COUNTER]], i64 2
; CHECK-NEXT:    [[TMP4:%.*]] = load i32, i32* [[ARRAYIDX_2]], align 4
; CHECK-NEXT:    [[ARRAYIDX2_2:%.*]] = getelementptr inbounds i32, i32* [[OUT_BLOCK]], i64 2
; CHECK-NEXT:    [[TMP5:%.*]] = load i32, i32* [[ARRAYIDX2_2]], align 4
; CHECK-NEXT:    [[XOR_2:%.*]] = xor i32 [[TMP5]], [[TMP4]]
; CHECK-NEXT:    store i32 [[XOR_2]], i32* [[ARRAYIDX2_2]], align 4
; CHECK-NEXT:    [[ARRAYIDX_3:%.*]] = getelementptr inbounds i32, i32* [[COUNTER]], i64 3
; CHECK-NEXT:    [[TMP6:%.*]] = load i32, i32* [[ARRAYIDX_3]], align 4
; CHECK-NEXT:    [[ARRAYIDX2_3:%.*]] = getelementptr inbounds i32, i32* [[OUT_BLOCK]], i64 3
; CHECK-NEXT:    [[TMP7:%.*]] = load i32, i32* [[ARRAYIDX2_3]], align 4
; CHECK-NEXT:    [[XOR_3:%.*]] = xor i32 [[TMP7]], [[TMP6]]
; CHECK-NEXT:    store i32 [[XOR_3]], i32* [[ARRAYIDX2_3]], align 4
; CHECK-NEXT:    ret void
;
entry:
  %0 = load i32, i32* %counter, align 4
  %1 = load i32, i32* %out_block, align 4
  %xor = xor i32 %1, %0
  store i32 %xor, i32* %out_block, align 4
  %arrayidx.1 = getelementptr inbounds i32, i32* %counter, i64 1
  %2 = load i32, i32* %arrayidx.1, align 4
  %arrayidx2.1 = getelementptr inbounds i32, i32* %out_block, i64 1
  %3 = load i32, i32* %arrayidx2.1, align 4
  %xor.1 = xor i32 %3, %2
  store i32 %xor.1, i32* %arrayidx2.1, align 4
  %arrayidx.2 = getelementptr inbounds i32, i32* %counter, i64 2
  %4 = load i32, i32* %arrayidx.2, align 4
  %arrayidx2.2 = getelementptr inbounds i32, i32* %out_block, i64 2
  %5 = load i32, i32* %arrayidx2.2, align 4
  %xor.2 = xor i32 %5, %4
  store i32 %xor.2, i32* %arrayidx2.2, align 4
  %arrayidx.3 = getelementptr inbounds i32, i32* %counter, i64 3
  %6 = load i32, i32* %arrayidx.3, align 4
  %arrayidx2.3 = getelementptr inbounds i32, i32* %out_block, i64 3
  %7 = load i32, i32* %arrayidx2.3, align 4
  %xor.3 = xor i32 %7, %6
  store i32 %xor.3, i32* %arrayidx2.3, align 4
  ret void
}

define i32 @use_outside_version_bb(i32* %dst, i32* %src, i1 %c.1) {
; CHECK-LABEL: @use_outside_version_bb(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[SRC_0:%.*]] = load i32, i32* [[SRC:%.*]], align 4
; CHECK-NEXT:    [[R_0:%.*]] = ashr i32 [[SRC_0]], 16
; CHECK-NEXT:    store i32 [[R_0]], i32* [[DST:%.*]], align 4
; CHECK-NEXT:    [[SRC_GEP_1:%.*]] = getelementptr inbounds i32, i32* [[SRC]], i64 1
; CHECK-NEXT:    [[SRC_1:%.*]] = load i32, i32* [[SRC_GEP_1]], align 4
; CHECK-NEXT:    [[R_1:%.*]] = ashr i32 [[SRC_1]], 16
; CHECK-NEXT:    [[DST_GEP_1:%.*]] = getelementptr inbounds i32, i32* [[DST]], i64 1
; CHECK-NEXT:    store i32 [[R_1]], i32* [[DST_GEP_1]], align 4
; CHECK-NEXT:    br label [[EXIT:%.*]]
; CHECK:       exit:
; CHECK-NEXT:    ret i32 [[R_0]]
;
entry:
  %src.0 = load i32, i32* %src, align 4
  %r.0 = ashr i32 %src.0, 16
  store i32 %r.0, i32* %dst, align 4
  %src.gep.1 = getelementptr inbounds i32, i32* %src, i64 1
  %src.1 = load i32, i32* %src.gep.1, align 4
  %r.1 = ashr i32 %src.1, 16
  %dst.gep.1 = getelementptr inbounds i32, i32* %dst, i64 1
  store i32 %r.1, i32* %dst.gep.1, align 4
  br label %exit

exit:
  ret i32 %r.0
}

define i32 @value_used_in_return(i32* %dst, i32* %src, i32 %x) {
; CHECK-LABEL: @value_used_in_return(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[SRC_0:%.*]] = load i32, i32* [[SRC:%.*]], align 4
; CHECK-NEXT:    [[R_0:%.*]] = ashr i32 [[SRC_0]], 16
; CHECK-NEXT:    store i32 [[R_0]], i32* [[DST:%.*]], align 4
; CHECK-NEXT:    [[SRC_GEP_1:%.*]] = getelementptr inbounds i32, i32* [[SRC]], i64 1
; CHECK-NEXT:    [[SRC_1:%.*]] = load i32, i32* [[SRC_GEP_1]], align 4
; CHECK-NEXT:    [[R_1:%.*]] = ashr i32 [[SRC_1]], 16
; CHECK-NEXT:    [[DST_GEP_1:%.*]] = getelementptr inbounds i32, i32* [[DST]], i64 1
; CHECK-NEXT:    store i32 [[R_1]], i32* [[DST_GEP_1]], align 4
; CHECK-NEXT:    [[ADD:%.*]] = add i32 [[X:%.*]], 20
; CHECK-NEXT:    ret i32 [[ADD]]
;
entry:
  %src.0 = load i32, i32* %src, align 4
  %r.0 = ashr i32 %src.0, 16
  store i32 %r.0, i32* %dst, align 4
  %src.gep.1 = getelementptr inbounds i32, i32* %src, i64 1
  %src.1 = load i32, i32* %src.gep.1, align 4
  %r.1 = ashr i32 %src.1, 16
  %dst.gep.1 = getelementptr inbounds i32, i32* %dst, i64 1
  store i32 %r.1, i32* %dst.gep.1, align 4
  %add = add i32 %x, 20
  ret i32 %add
}
define i32 @needs_versioning2_cond_br(i32* %dst, i32* %src, i1 %c.1) {
; CHECK-LABEL: @needs_versioning2_cond_br(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    br i1 [[C_1:%.*]], label [[THEN:%.*]], label [[ELSE:%.*]]
; CHECK:       then:
; CHECK-NEXT:    [[SRC_0:%.*]] = load i32, i32* [[SRC:%.*]], align 4
; CHECK-NEXT:    [[R_0:%.*]] = ashr i32 [[SRC_0]], 16
; CHECK-NEXT:    store i32 [[R_0]], i32* [[DST:%.*]], align 4
; CHECK-NEXT:    [[SRC_GEP_1:%.*]] = getelementptr inbounds i32, i32* [[SRC]], i64 1
; CHECK-NEXT:    [[SRC_1:%.*]] = load i32, i32* [[SRC_GEP_1]], align 4
; CHECK-NEXT:    [[R_1:%.*]] = ashr i32 [[SRC_1]], 16
; CHECK-NEXT:    [[DST_GEP_1:%.*]] = getelementptr inbounds i32, i32* [[DST]], i64 1
; CHECK-NEXT:    store i32 [[R_1]], i32* [[DST_GEP_1]], align 4
; CHECK-NEXT:    ret i32 10
; CHECK:       else:
; CHECK-NEXT:    ret i32 0
;
entry:
  br i1 %c.1, label %then, label %else

then:
  %src.0 = load i32, i32* %src, align 4
  %r.0 = ashr i32 %src.0, 16
  store i32 %r.0, i32* %dst, align 4
  %src.gep.1 = getelementptr inbounds i32, i32* %src, i64 1
  %src.1 = load i32, i32* %src.gep.1, align 4
  %r.1 = ashr i32 %src.1, 16
  %dst.gep.1 = getelementptr inbounds i32, i32* %dst, i64 1
  store i32 %r.1, i32* %dst.gep.1, align 4
  ret i32 10


else:
  ret i32 0
}

define void @pointer_defined_in_bb(i32* %dst, i32** %src.p) {
; CHECK-LABEL: @pointer_defined_in_bb(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[SRC:%.*]] = load i32*, i32** [[SRC_P:%.*]], align 8
; CHECK-NEXT:    [[SRC_0:%.*]] = load i32, i32* [[SRC]], align 4
; CHECK-NEXT:    [[R_0:%.*]] = ashr i32 [[SRC_0]], 16
; CHECK-NEXT:    store i32 [[R_0]], i32* [[DST:%.*]], align 4
; CHECK-NEXT:    [[SRC_GEP_1:%.*]] = getelementptr inbounds i32, i32* [[SRC]], i64 1
; CHECK-NEXT:    [[SRC_1:%.*]] = load i32, i32* [[SRC_GEP_1]], align 4
; CHECK-NEXT:    [[R_1:%.*]] = ashr i32 [[SRC_1]], 16
; CHECK-NEXT:    [[DST_GEP_1:%.*]] = getelementptr inbounds i32, i32* [[DST]], i64 1
; CHECK-NEXT:    store i32 [[R_1]], i32* [[DST_GEP_1]], align 4
; CHECK-NEXT:    ret void
;
entry:
  %src = load i32*, i32** %src.p
  %src.0 = load i32, i32* %src, align 4
  %r.0 = ashr i32 %src.0, 16
  store i32 %r.0, i32* %dst, align 4
  %src.gep.1 = getelementptr inbounds i32, i32* %src, i64 1
  %src.1 = load i32, i32* %src.gep.1, align 4
  %r.1 = ashr i32 %src.1, 16
  %dst.gep.1 = getelementptr inbounds i32, i32* %dst, i64 1
  store i32 %r.1, i32* %dst.gep.1, align 4
  ret void
}

define void @clobber_same_underlying_object(i32* %this) {
; CHECK-LABEL: @clobber_same_underlying_object(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[P_3:%.*]] = getelementptr inbounds i32, i32* [[THIS:%.*]], i32 3
; CHECK-NEXT:    store i32 10, i32* [[P_3]], align 8
; CHECK-NEXT:    tail call void @clobber()
; CHECK-NEXT:    [[P_4:%.*]] = getelementptr inbounds i32, i32* [[THIS]], i32 4
; CHECK-NEXT:    [[L2:%.*]] = load i32, i32* [[P_4]], align 8
; CHECK-NEXT:    store i32 20, i32* [[P_4]], align 8
; CHECK-NEXT:    ret void
;
entry:
  %p.3 = getelementptr inbounds i32, i32* %this, i32 3
  store i32 10, i32* %p.3, align 8
  tail call void @clobber()
  %p.4 = getelementptr inbounds i32, i32* %this, i32 4
  %l2 = load i32, i32* %p.4, align 8
  store i32 20, i32* %p.4, align 8
  ret void
}

declare void @clobber()

define void @slp_not_beneficial(i32* %A, i32* %B) {
; CHECK-LABEL: @slp_not_beneficial(
; CHECK-NEXT:  bb:
; CHECK-NEXT:    [[TMP:%.*]] = getelementptr inbounds i32, i32* [[A:%.*]], i32 4
; CHECK-NEXT:    store i32 0, i32* [[TMP]], align 8
; CHECK-NEXT:    [[TMP3:%.*]] = getelementptr inbounds i32, i32* [[A]], i32 5
; CHECK-NEXT:    [[TMP4:%.*]] = getelementptr inbounds i32, i32* [[B:%.*]], i32 4
; CHECK-NEXT:    [[TMP5:%.*]] = load i32, i32* [[TMP4]], align 8
; CHECK-NEXT:    store i32 [[TMP5]], i32* [[TMP3]], align 8
; CHECK-NEXT:    ret void
;
bb:
  %tmp = getelementptr inbounds i32, i32* %A, i32 4
  store i32 0, i32* %tmp, align 8
  %tmp3 = getelementptr inbounds i32, i32* %A, i32 5
  %tmp4 = getelementptr inbounds i32, i32* %B, i32 4
  %tmp5 = load i32, i32* %tmp4, align 8
  store i32 %tmp5, i32* %tmp3, align 8
  ret void
}

define void @widget(double* %ptr, double* %ptr.2) {
; CHECK-LABEL: @widget(
; CHECK-NEXT:  bb1:
; CHECK-NEXT:    [[TMP3:%.*]] = load double, double* null, align 8
; CHECK-NEXT:    [[TMP4:%.*]] = fmul double undef, [[TMP3]]
; CHECK-NEXT:    [[TMP5:%.*]] = getelementptr inbounds double, double* [[PTR:%.*]], i32 0
; CHECK-NEXT:    [[TMP6:%.*]] = load double, double* [[TMP5]], align 8
; CHECK-NEXT:    [[TMP7:%.*]] = fadd double [[TMP6]], [[TMP4]]
; CHECK-NEXT:    store double [[TMP7]], double* [[TMP5]], align 8
; CHECK-NEXT:    [[TMP8:%.*]] = getelementptr inbounds double, double* [[PTR_2:%.*]], i64 0
; CHECK-NEXT:    [[TMP9:%.*]] = load double, double* [[TMP8]], align 8
; CHECK-NEXT:    [[TMP10:%.*]] = fmul double undef, [[TMP9]]
; CHECK-NEXT:    [[TMP11:%.*]] = getelementptr inbounds double, double* [[PTR]], i32 1
; CHECK-NEXT:    [[TMP12:%.*]] = load double, double* [[TMP11]], align 8
; CHECK-NEXT:    [[TMP13:%.*]] = fadd double [[TMP12]], [[TMP10]]
; CHECK-NEXT:    store double [[TMP13]], double* [[TMP11]], align 8
; CHECK-NEXT:    br label [[BB15:%.*]]
; CHECK:       bb15:
; CHECK-NEXT:    br label [[BB15]]
;
bb1:                                              ; preds = %bb
  %tmp3 = load double, double* null, align 8
  %tmp4 = fmul double undef, %tmp3
  %tmp5 = getelementptr inbounds double, double* %ptr, i32 0
  %tmp6 = load double, double* %tmp5, align 8
  %tmp7 = fadd double %tmp6, %tmp4
  store double %tmp7, double* %tmp5, align 8
  %tmp8 = getelementptr inbounds double, double* %ptr.2, i64 0
  %tmp9 = load double, double* %tmp8, align 8
  %tmp10 = fmul double undef, %tmp9
  %tmp11 = getelementptr inbounds double, double* %ptr, i32 1
  %tmp12 = load double, double* %tmp11, align 8
  %tmp13 = fadd double %tmp12, %tmp10
  store double %tmp13, double* %tmp11, align 8
  br label %bb15

bb15:                                             ; preds = %bb15, %bb14
  br label %bb15
}

%struct = type { i32, i32, float, float }

; Some points we collected as candidates for runtime checks have been removed
; before generating runtime checks. Make sure versioning is skipped.
define void @test_bounds_removed_before_runtime_checks(%struct * %A, i32** %B, i1 %c) {
; CHECK-LABEL: @test_bounds_removed_before_runtime_checks(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[TMP11:%.*]] = getelementptr inbounds [[STRUCT:%.*]], %struct* [[A:%.*]], i64 0, i32 0
; CHECK-NEXT:    [[TMP12:%.*]] = getelementptr inbounds [[STRUCT]], %struct* [[A]], i64 0, i32 1
; CHECK-NEXT:    [[TMP0:%.*]] = bitcast i32* [[TMP11]] to <2 x i32>*
; CHECK-NEXT:    store <2 x i32> <i32 10, i32 300>, <2 x i32>* [[TMP0]], align 8
; CHECK-NEXT:    [[TMP13:%.*]] = load i32*, i32** [[B:%.*]], align 8
; CHECK-NEXT:    br i1 [[C:%.*]], label [[BB23:%.*]], label [[BB14:%.*]]
; CHECK:       bb14:
; CHECK-NEXT:    [[TMP15:%.*]] = sext i32 10 to i64
; CHECK-NEXT:    [[TMP16:%.*]] = add nsw i64 2, [[TMP15]]
; CHECK-NEXT:    [[TMP17:%.*]] = getelementptr inbounds i32, i32* [[TMP13]], i64 [[TMP16]]
; CHECK-NEXT:    [[TMP18:%.*]] = bitcast i32* [[TMP17]] to i8*
; CHECK-NEXT:    [[TMP19:%.*]] = getelementptr inbounds i8, i8* [[TMP18]], i64 3
; CHECK-NEXT:    [[TMP20:%.*]] = getelementptr inbounds [[STRUCT]], %struct* [[A]], i64 0, i32 2
; CHECK-NEXT:    store float 0.000000e+00, float* [[TMP20]], align 8
; CHECK-NEXT:    [[TMP21:%.*]] = load i8, i8* [[TMP19]], align 1
; CHECK-NEXT:    [[TMP22:%.*]] = getelementptr inbounds [[STRUCT]], %struct* [[A]], i64 0, i32 3
; CHECK-NEXT:    store float 0.000000e+00, float* [[TMP22]], align 4
; CHECK-NEXT:    br label [[BB23]]
; CHECK:       bb23:
; CHECK-NEXT:    ret void
;
entry:
  %tmp1 = fmul float 10.0, 20.0
  %tmp2 = fptosi float %tmp1 to i32
  %tmp3 = fmul float 30.0, 20.0
  %tmp4 = fptosi float %tmp3 to i32
  %tmp5 = icmp sgt i32 100, %tmp2
  %tmp6 = select i1 %tmp5, i32 %tmp2, i32 10
  %tmp7 = select i1 false, i32 0, i32 %tmp6
  %tmp8 = icmp sgt i32 200, %tmp4
  %tmp9 = select i1 %tmp8, i32 %tmp4, i32 300
  %tmp10 = select i1 false, i32 0, i32 %tmp9
  %tmp11 = getelementptr inbounds %struct, %struct* %A, i64 0, i32 0
  store i32 %tmp7, i32* %tmp11, align 8
  %tmp12 = getelementptr inbounds %struct, %struct* %A, i64 0, i32 1
  store i32 %tmp10, i32* %tmp12, align 4
  %tmp13 = load i32*, i32** %B, align 8
  br i1 %c, label %bb23, label %bb14

bb14:
  %tmp15 = sext i32 %tmp7 to i64
  %tmp16 = add nsw i64 2, %tmp15
  %tmp17 = getelementptr inbounds i32, i32* %tmp13, i64 %tmp16
  %tmp18 = bitcast i32* %tmp17 to i8*
  %tmp19 = getelementptr inbounds i8, i8* %tmp18, i64 3
  %tmp20 = getelementptr inbounds %struct, %struct* %A, i64 0, i32 2
  store float 0.0, float* %tmp20, align 8
  %tmp21 = load i8, i8* %tmp19, align 1
  %tmp22 = getelementptr inbounds %struct, %struct* %A, i64 0, i32 3
  store float 0.0, float* %tmp22, align 4
  br label %bb23

bb23:
  ret void
}

; In this test there's a single bound, do not generate runtime checks.
define void @single_membound(double* %arg, double* %arg1, double %x) {
; CHECK-LABEL: @single_membound(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[TMP:%.*]] = fsub double [[X:%.*]], 9.900000e+01
; CHECK-NEXT:    [[TMP9:%.*]] = getelementptr inbounds double, double* [[ARG:%.*]], i64 1
; CHECK-NEXT:    store double [[TMP]], double* [[TMP9]], align 8
; CHECK-NEXT:    [[TMP10:%.*]] = getelementptr inbounds double, double* [[ARG1:%.*]], i64 0
; CHECK-NEXT:    [[TMP12:%.*]] = load double, double* [[TMP10]], align 8
; CHECK-NEXT:    [[TMP13:%.*]] = fsub double 1.000000e+00, [[TMP12]]
; CHECK-NEXT:    [[TMP14:%.*]] = getelementptr inbounds double, double* [[ARG]], i64 2
; CHECK-NEXT:    br label [[BB15:%.*]]
; CHECK:       bb15:
; CHECK-NEXT:    [[TMP16:%.*]] = fmul double [[TMP]], 2.000000e+01
; CHECK-NEXT:    store double [[TMP16]], double* [[TMP9]], align 8
; CHECK-NEXT:    [[TMP17:%.*]] = fmul double [[TMP13]], 3.000000e+01
; CHECK-NEXT:    store double [[TMP17]], double* [[TMP14]], align 8
; CHECK-NEXT:    ret void
;
entry:
  %tmp = fsub double %x, 99.0
  %tmp9 = getelementptr inbounds double, double* %arg, i64 1
  store double %tmp, double* %tmp9, align 8
  %tmp10 = getelementptr inbounds double, double* %arg1, i64 0
  %tmp12 = load double, double* %tmp10, align 8
  %tmp13 = fsub double 1.0, %tmp12
  %tmp14 = getelementptr inbounds double, double* %arg, i64 2
  br label %bb15

bb15:
  %tmp16 = fmul double %tmp, 20.0
  store double %tmp16, double* %tmp9, align 8
  %tmp17 = fmul double %tmp13, 30.0
  store double %tmp17, double* %tmp14, align 8
  ret void
}
