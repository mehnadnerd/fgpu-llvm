# NOTE: Assertions have been autogenerated by utils/update_mca_test_checks.py
# RUN: llvm-mca -mtriple=x86_64-unknown-unknown -mcpu=bdver2 -scheduler-stats -dispatch-stats -iterations=100 -timeline -timeline-max-iterations=1 < %s | FileCheck %s

# LLVM-MCA-BEGIN
movb %spl, (%rax)
movb %bpl, (%rcx)
movb %sil, (%rdx)
movb %dil, (%rbx)
# LLVM-MCA-END

# LLVM-MCA-BEGIN
movw %sp, (%rax)
movw %bp, (%rcx)
movw %si, (%rdx)
movw %di, (%rbx)
# LLVM-MCA-END

# LLVM-MCA-BEGIN
movl %esp, (%rax)
movl %ebp, (%rcx)
movl %esi, (%rdx)
movl %edi, (%rbx)
# LLVM-MCA-END

# LLVM-MCA-BEGIN
movq %rsp, (%rax)
movq %rbp, (%rcx)
movq %rsi, (%rdx)
movq %rdi, (%rbx)
# LLVM-MCA-END

# LLVM-MCA-BEGIN
movd %mm0, (%rax)
movd %mm1, (%rcx)
movd %mm2, (%rdx)
movd %mm3, (%rbx)
# LLVM-MCA-END

# LLVM-MCA-BEGIN
movaps %xmm0, (%rax)
movaps %xmm1, (%rcx)
movaps %xmm2, (%rdx)
movaps %xmm3, (%rbx)
# LLVM-MCA-END

# LLVM-MCA-BEGIN
vmovaps %ymm0, (%rax)
vmovaps %ymm1, (%rcx)
vmovaps %ymm2, (%rdx)
vmovaps %ymm3, (%rbx)
# LLVM-MCA-END

# CHECK:      [0] Code Region

# CHECK:      Iterations:        100
# CHECK-NEXT: Instructions:      400
# CHECK-NEXT: Total Cycles:      403
# CHECK-NEXT: Total uOps:        400

# CHECK:      Dispatch Width:    4
# CHECK-NEXT: uOps Per Cycle:    0.99
# CHECK-NEXT: IPC:               0.99
# CHECK-NEXT: Block RThroughput: 4.0

# CHECK:      Instruction Info:
# CHECK-NEXT: [1]: #uOps
# CHECK-NEXT: [2]: Latency
# CHECK-NEXT: [3]: RThroughput
# CHECK-NEXT: [4]: MayLoad
# CHECK-NEXT: [5]: MayStore
# CHECK-NEXT: [6]: HasSideEffects (U)

# CHECK:      [1]    [2]    [3]    [4]    [5]    [6]    Instructions:
# CHECK-NEXT:  1      1     1.00           *            movb	%spl, (%rax)
# CHECK-NEXT:  1      1     1.00           *            movb	%bpl, (%rcx)
# CHECK-NEXT:  1      1     1.00           *            movb	%sil, (%rdx)
# CHECK-NEXT:  1      1     1.00           *            movb	%dil, (%rbx)

# CHECK:      Dynamic Dispatch Stall Cycles:
# CHECK-NEXT: RAT     - Register unavailable:                      0
# CHECK-NEXT: RCU     - Retire tokens unavailable:                 0
# CHECK-NEXT: SCHEDQ  - Scheduler full:                            0
# CHECK-NEXT: LQ      - Load queue full:                           0
# CHECK-NEXT: SQ      - Store queue full:                          371  (92.1%)
# CHECK-NEXT: GROUP   - Static restrictions on the dispatch group: 0
# CHECK-NEXT: USH     - Uncategorised Structural Hazard:           0

# CHECK:      Dispatch Logic - number of cycles where we saw N micro opcodes dispatched:
# CHECK-NEXT: [# dispatched], [# cycles]
# CHECK-NEXT:  0,              24  (6.0%)
# CHECK-NEXT:  1,              372  (92.3%)
# CHECK-NEXT:  4,              7  (1.7%)

# CHECK:      Schedulers - number of cycles where we saw N micro opcodes issued:
# CHECK-NEXT: [# issued], [# cycles]
# CHECK-NEXT:  0,          3  (0.7%)
# CHECK-NEXT:  1,          400  (99.3%)

# CHECK:      Scheduler's queue usage:
# CHECK-NEXT: [1] Resource name.
# CHECK-NEXT: [2] Average number of used buffer entries.
# CHECK-NEXT: [3] Maximum number of used buffer entries.
# CHECK-NEXT: [4] Total number of buffer entries.

# CHECK:       [1]            [2]        [3]        [4]
# CHECK-NEXT: PdEX             21         22         40
# CHECK-NEXT: PdFPU            0          0          64
# CHECK-NEXT: PdLoad           0          0          40
# CHECK-NEXT: PdStore          22         23         24

# CHECK:      Resources:
# CHECK-NEXT: [0.0] - PdAGLU01
# CHECK-NEXT: [0.1] - PdAGLU01
# CHECK-NEXT: [1]   - PdBranch
# CHECK-NEXT: [2]   - PdCount
# CHECK-NEXT: [3]   - PdDiv
# CHECK-NEXT: [4]   - PdEX0
# CHECK-NEXT: [5]   - PdEX1
# CHECK-NEXT: [6]   - PdFPCVT
# CHECK-NEXT: [7.0] - PdFPFMA
# CHECK-NEXT: [7.1] - PdFPFMA
# CHECK-NEXT: [8.0] - PdFPMAL
# CHECK-NEXT: [8.1] - PdFPMAL
# CHECK-NEXT: [9]   - PdFPMMA
# CHECK-NEXT: [10]  - PdFPSTO
# CHECK-NEXT: [11]  - PdFPU0
# CHECK-NEXT: [12]  - PdFPU1
# CHECK-NEXT: [13]  - PdFPU2
# CHECK-NEXT: [14]  - PdFPU3
# CHECK-NEXT: [15]  - PdFPXBR
# CHECK-NEXT: [16.0] - PdLoad
# CHECK-NEXT: [16.1] - PdLoad
# CHECK-NEXT: [17]  - PdMul
# CHECK-NEXT: [18]  - PdStore

# CHECK:      Resource pressure per iteration:
# CHECK-NEXT: [0.0]  [0.1]  [1]    [2]    [3]    [4]    [5]    [6]    [7.0]  [7.1]  [8.0]  [8.1]  [9]    [10]   [11]   [12]   [13]   [14]   [15]   [16.0] [16.1] [17]   [18]
# CHECK-NEXT: 2.00   2.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     4.00

# CHECK:      Resource pressure by instruction:
# CHECK-NEXT: [0.0]  [0.1]  [1]    [2]    [3]    [4]    [5]    [6]    [7.0]  [7.1]  [8.0]  [8.1]  [9]    [10]   [11]   [12]   [13]   [14]   [15]   [16.0] [16.1] [17]   [18]   Instructions:
# CHECK-NEXT:  -     1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movb	%spl, (%rax)
# CHECK-NEXT: 1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movb	%bpl, (%rcx)
# CHECK-NEXT:  -     1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movb	%sil, (%rdx)
# CHECK-NEXT: 1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movb	%dil, (%rbx)

# CHECK:      Timeline view:
# CHECK-NEXT: Index     0123456

# CHECK:      [0,0]     DeER ..   movb	%spl, (%rax)
# CHECK-NEXT: [0,1]     D=eER..   movb	%bpl, (%rcx)
# CHECK-NEXT: [0,2]     D==eER.   movb	%sil, (%rdx)
# CHECK-NEXT: [0,3]     D===eER   movb	%dil, (%rbx)

# CHECK:      Average Wait times (based on the timeline view):
# CHECK-NEXT: [0]: Executions
# CHECK-NEXT: [1]: Average time spent waiting in a scheduler's queue
# CHECK-NEXT: [2]: Average time spent waiting in a scheduler's queue while ready
# CHECK-NEXT: [3]: Average time elapsed from WB until retire stage

# CHECK:            [0]    [1]    [2]    [3]
# CHECK-NEXT: 0.     1     1.0    1.0    0.0       movb	%spl, (%rax)
# CHECK-NEXT: 1.     1     2.0    1.0    0.0       movb	%bpl, (%rcx)
# CHECK-NEXT: 2.     1     3.0    1.0    0.0       movb	%sil, (%rdx)
# CHECK-NEXT: 3.     1     4.0    1.0    0.0       movb	%dil, (%rbx)
# CHECK-NEXT:        1     2.5    1.0    0.0       <total>

# CHECK:      [1] Code Region

# CHECK:      Iterations:        100
# CHECK-NEXT: Instructions:      400
# CHECK-NEXT: Total Cycles:      403
# CHECK-NEXT: Total uOps:        400

# CHECK:      Dispatch Width:    4
# CHECK-NEXT: uOps Per Cycle:    0.99
# CHECK-NEXT: IPC:               0.99
# CHECK-NEXT: Block RThroughput: 4.0

# CHECK:      Instruction Info:
# CHECK-NEXT: [1]: #uOps
# CHECK-NEXT: [2]: Latency
# CHECK-NEXT: [3]: RThroughput
# CHECK-NEXT: [4]: MayLoad
# CHECK-NEXT: [5]: MayStore
# CHECK-NEXT: [6]: HasSideEffects (U)

# CHECK:      [1]    [2]    [3]    [4]    [5]    [6]    Instructions:
# CHECK-NEXT:  1      1     1.00           *            movw	%sp, (%rax)
# CHECK-NEXT:  1      1     1.00           *            movw	%bp, (%rcx)
# CHECK-NEXT:  1      1     1.00           *            movw	%si, (%rdx)
# CHECK-NEXT:  1      1     1.00           *            movw	%di, (%rbx)

# CHECK:      Dynamic Dispatch Stall Cycles:
# CHECK-NEXT: RAT     - Register unavailable:                      0
# CHECK-NEXT: RCU     - Retire tokens unavailable:                 0
# CHECK-NEXT: SCHEDQ  - Scheduler full:                            0
# CHECK-NEXT: LQ      - Load queue full:                           0
# CHECK-NEXT: SQ      - Store queue full:                          371  (92.1%)
# CHECK-NEXT: GROUP   - Static restrictions on the dispatch group: 0
# CHECK-NEXT: USH     - Uncategorised Structural Hazard:           0

# CHECK:      Dispatch Logic - number of cycles where we saw N micro opcodes dispatched:
# CHECK-NEXT: [# dispatched], [# cycles]
# CHECK-NEXT:  0,              24  (6.0%)
# CHECK-NEXT:  1,              372  (92.3%)
# CHECK-NEXT:  4,              7  (1.7%)

# CHECK:      Schedulers - number of cycles where we saw N micro opcodes issued:
# CHECK-NEXT: [# issued], [# cycles]
# CHECK-NEXT:  0,          3  (0.7%)
# CHECK-NEXT:  1,          400  (99.3%)

# CHECK:      Scheduler's queue usage:
# CHECK-NEXT: [1] Resource name.
# CHECK-NEXT: [2] Average number of used buffer entries.
# CHECK-NEXT: [3] Maximum number of used buffer entries.
# CHECK-NEXT: [4] Total number of buffer entries.

# CHECK:       [1]            [2]        [3]        [4]
# CHECK-NEXT: PdEX             21         22         40
# CHECK-NEXT: PdFPU            0          0          64
# CHECK-NEXT: PdLoad           0          0          40
# CHECK-NEXT: PdStore          22         23         24

# CHECK:      Resources:
# CHECK-NEXT: [0.0] - PdAGLU01
# CHECK-NEXT: [0.1] - PdAGLU01
# CHECK-NEXT: [1]   - PdBranch
# CHECK-NEXT: [2]   - PdCount
# CHECK-NEXT: [3]   - PdDiv
# CHECK-NEXT: [4]   - PdEX0
# CHECK-NEXT: [5]   - PdEX1
# CHECK-NEXT: [6]   - PdFPCVT
# CHECK-NEXT: [7.0] - PdFPFMA
# CHECK-NEXT: [7.1] - PdFPFMA
# CHECK-NEXT: [8.0] - PdFPMAL
# CHECK-NEXT: [8.1] - PdFPMAL
# CHECK-NEXT: [9]   - PdFPMMA
# CHECK-NEXT: [10]  - PdFPSTO
# CHECK-NEXT: [11]  - PdFPU0
# CHECK-NEXT: [12]  - PdFPU1
# CHECK-NEXT: [13]  - PdFPU2
# CHECK-NEXT: [14]  - PdFPU3
# CHECK-NEXT: [15]  - PdFPXBR
# CHECK-NEXT: [16.0] - PdLoad
# CHECK-NEXT: [16.1] - PdLoad
# CHECK-NEXT: [17]  - PdMul
# CHECK-NEXT: [18]  - PdStore

# CHECK:      Resource pressure per iteration:
# CHECK-NEXT: [0.0]  [0.1]  [1]    [2]    [3]    [4]    [5]    [6]    [7.0]  [7.1]  [8.0]  [8.1]  [9]    [10]   [11]   [12]   [13]   [14]   [15]   [16.0] [16.1] [17]   [18]
# CHECK-NEXT: 2.00   2.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     4.00

# CHECK:      Resource pressure by instruction:
# CHECK-NEXT: [0.0]  [0.1]  [1]    [2]    [3]    [4]    [5]    [6]    [7.0]  [7.1]  [8.0]  [8.1]  [9]    [10]   [11]   [12]   [13]   [14]   [15]   [16.0] [16.1] [17]   [18]   Instructions:
# CHECK-NEXT:  -     1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movw	%sp, (%rax)
# CHECK-NEXT: 1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movw	%bp, (%rcx)
# CHECK-NEXT:  -     1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movw	%si, (%rdx)
# CHECK-NEXT: 1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movw	%di, (%rbx)

# CHECK:      Timeline view:
# CHECK-NEXT: Index     0123456

# CHECK:      [0,0]     DeER ..   movw	%sp, (%rax)
# CHECK-NEXT: [0,1]     D=eER..   movw	%bp, (%rcx)
# CHECK-NEXT: [0,2]     D==eER.   movw	%si, (%rdx)
# CHECK-NEXT: [0,3]     D===eER   movw	%di, (%rbx)

# CHECK:      Average Wait times (based on the timeline view):
# CHECK-NEXT: [0]: Executions
# CHECK-NEXT: [1]: Average time spent waiting in a scheduler's queue
# CHECK-NEXT: [2]: Average time spent waiting in a scheduler's queue while ready
# CHECK-NEXT: [3]: Average time elapsed from WB until retire stage

# CHECK:            [0]    [1]    [2]    [3]
# CHECK-NEXT: 0.     1     1.0    1.0    0.0       movw	%sp, (%rax)
# CHECK-NEXT: 1.     1     2.0    1.0    0.0       movw	%bp, (%rcx)
# CHECK-NEXT: 2.     1     3.0    1.0    0.0       movw	%si, (%rdx)
# CHECK-NEXT: 3.     1     4.0    1.0    0.0       movw	%di, (%rbx)
# CHECK-NEXT:        1     2.5    1.0    0.0       <total>

# CHECK:      [2] Code Region

# CHECK:      Iterations:        100
# CHECK-NEXT: Instructions:      400
# CHECK-NEXT: Total Cycles:      403
# CHECK-NEXT: Total uOps:        400

# CHECK:      Dispatch Width:    4
# CHECK-NEXT: uOps Per Cycle:    0.99
# CHECK-NEXT: IPC:               0.99
# CHECK-NEXT: Block RThroughput: 4.0

# CHECK:      Instruction Info:
# CHECK-NEXT: [1]: #uOps
# CHECK-NEXT: [2]: Latency
# CHECK-NEXT: [3]: RThroughput
# CHECK-NEXT: [4]: MayLoad
# CHECK-NEXT: [5]: MayStore
# CHECK-NEXT: [6]: HasSideEffects (U)

# CHECK:      [1]    [2]    [3]    [4]    [5]    [6]    Instructions:
# CHECK-NEXT:  1      1     1.00           *            movl	%esp, (%rax)
# CHECK-NEXT:  1      1     1.00           *            movl	%ebp, (%rcx)
# CHECK-NEXT:  1      1     1.00           *            movl	%esi, (%rdx)
# CHECK-NEXT:  1      1     1.00           *            movl	%edi, (%rbx)

# CHECK:      Dynamic Dispatch Stall Cycles:
# CHECK-NEXT: RAT     - Register unavailable:                      0
# CHECK-NEXT: RCU     - Retire tokens unavailable:                 0
# CHECK-NEXT: SCHEDQ  - Scheduler full:                            0
# CHECK-NEXT: LQ      - Load queue full:                           0
# CHECK-NEXT: SQ      - Store queue full:                          371  (92.1%)
# CHECK-NEXT: GROUP   - Static restrictions on the dispatch group: 0
# CHECK-NEXT: USH     - Uncategorised Structural Hazard:           0

# CHECK:      Dispatch Logic - number of cycles where we saw N micro opcodes dispatched:
# CHECK-NEXT: [# dispatched], [# cycles]
# CHECK-NEXT:  0,              24  (6.0%)
# CHECK-NEXT:  1,              372  (92.3%)
# CHECK-NEXT:  4,              7  (1.7%)

# CHECK:      Schedulers - number of cycles where we saw N micro opcodes issued:
# CHECK-NEXT: [# issued], [# cycles]
# CHECK-NEXT:  0,          3  (0.7%)
# CHECK-NEXT:  1,          400  (99.3%)

# CHECK:      Scheduler's queue usage:
# CHECK-NEXT: [1] Resource name.
# CHECK-NEXT: [2] Average number of used buffer entries.
# CHECK-NEXT: [3] Maximum number of used buffer entries.
# CHECK-NEXT: [4] Total number of buffer entries.

# CHECK:       [1]            [2]        [3]        [4]
# CHECK-NEXT: PdEX             21         22         40
# CHECK-NEXT: PdFPU            0          0          64
# CHECK-NEXT: PdLoad           0          0          40
# CHECK-NEXT: PdStore          22         23         24

# CHECK:      Resources:
# CHECK-NEXT: [0.0] - PdAGLU01
# CHECK-NEXT: [0.1] - PdAGLU01
# CHECK-NEXT: [1]   - PdBranch
# CHECK-NEXT: [2]   - PdCount
# CHECK-NEXT: [3]   - PdDiv
# CHECK-NEXT: [4]   - PdEX0
# CHECK-NEXT: [5]   - PdEX1
# CHECK-NEXT: [6]   - PdFPCVT
# CHECK-NEXT: [7.0] - PdFPFMA
# CHECK-NEXT: [7.1] - PdFPFMA
# CHECK-NEXT: [8.0] - PdFPMAL
# CHECK-NEXT: [8.1] - PdFPMAL
# CHECK-NEXT: [9]   - PdFPMMA
# CHECK-NEXT: [10]  - PdFPSTO
# CHECK-NEXT: [11]  - PdFPU0
# CHECK-NEXT: [12]  - PdFPU1
# CHECK-NEXT: [13]  - PdFPU2
# CHECK-NEXT: [14]  - PdFPU3
# CHECK-NEXT: [15]  - PdFPXBR
# CHECK-NEXT: [16.0] - PdLoad
# CHECK-NEXT: [16.1] - PdLoad
# CHECK-NEXT: [17]  - PdMul
# CHECK-NEXT: [18]  - PdStore

# CHECK:      Resource pressure per iteration:
# CHECK-NEXT: [0.0]  [0.1]  [1]    [2]    [3]    [4]    [5]    [6]    [7.0]  [7.1]  [8.0]  [8.1]  [9]    [10]   [11]   [12]   [13]   [14]   [15]   [16.0] [16.1] [17]   [18]
# CHECK-NEXT: 2.00   2.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     4.00

# CHECK:      Resource pressure by instruction:
# CHECK-NEXT: [0.0]  [0.1]  [1]    [2]    [3]    [4]    [5]    [6]    [7.0]  [7.1]  [8.0]  [8.1]  [9]    [10]   [11]   [12]   [13]   [14]   [15]   [16.0] [16.1] [17]   [18]   Instructions:
# CHECK-NEXT:  -     1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movl	%esp, (%rax)
# CHECK-NEXT: 1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movl	%ebp, (%rcx)
# CHECK-NEXT:  -     1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movl	%esi, (%rdx)
# CHECK-NEXT: 1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movl	%edi, (%rbx)

# CHECK:      Timeline view:
# CHECK-NEXT: Index     0123456

# CHECK:      [0,0]     DeER ..   movl	%esp, (%rax)
# CHECK-NEXT: [0,1]     D=eER..   movl	%ebp, (%rcx)
# CHECK-NEXT: [0,2]     D==eER.   movl	%esi, (%rdx)
# CHECK-NEXT: [0,3]     D===eER   movl	%edi, (%rbx)

# CHECK:      Average Wait times (based on the timeline view):
# CHECK-NEXT: [0]: Executions
# CHECK-NEXT: [1]: Average time spent waiting in a scheduler's queue
# CHECK-NEXT: [2]: Average time spent waiting in a scheduler's queue while ready
# CHECK-NEXT: [3]: Average time elapsed from WB until retire stage

# CHECK:            [0]    [1]    [2]    [3]
# CHECK-NEXT: 0.     1     1.0    1.0    0.0       movl	%esp, (%rax)
# CHECK-NEXT: 1.     1     2.0    1.0    0.0       movl	%ebp, (%rcx)
# CHECK-NEXT: 2.     1     3.0    1.0    0.0       movl	%esi, (%rdx)
# CHECK-NEXT: 3.     1     4.0    1.0    0.0       movl	%edi, (%rbx)
# CHECK-NEXT:        1     2.5    1.0    0.0       <total>

# CHECK:      [3] Code Region

# CHECK:      Iterations:        100
# CHECK-NEXT: Instructions:      400
# CHECK-NEXT: Total Cycles:      403
# CHECK-NEXT: Total uOps:        400

# CHECK:      Dispatch Width:    4
# CHECK-NEXT: uOps Per Cycle:    0.99
# CHECK-NEXT: IPC:               0.99
# CHECK-NEXT: Block RThroughput: 4.0

# CHECK:      Instruction Info:
# CHECK-NEXT: [1]: #uOps
# CHECK-NEXT: [2]: Latency
# CHECK-NEXT: [3]: RThroughput
# CHECK-NEXT: [4]: MayLoad
# CHECK-NEXT: [5]: MayStore
# CHECK-NEXT: [6]: HasSideEffects (U)

# CHECK:      [1]    [2]    [3]    [4]    [5]    [6]    Instructions:
# CHECK-NEXT:  1      1     1.00           *            movq	%rsp, (%rax)
# CHECK-NEXT:  1      1     1.00           *            movq	%rbp, (%rcx)
# CHECK-NEXT:  1      1     1.00           *            movq	%rsi, (%rdx)
# CHECK-NEXT:  1      1     1.00           *            movq	%rdi, (%rbx)

# CHECK:      Dynamic Dispatch Stall Cycles:
# CHECK-NEXT: RAT     - Register unavailable:                      0
# CHECK-NEXT: RCU     - Retire tokens unavailable:                 0
# CHECK-NEXT: SCHEDQ  - Scheduler full:                            0
# CHECK-NEXT: LQ      - Load queue full:                           0
# CHECK-NEXT: SQ      - Store queue full:                          371  (92.1%)
# CHECK-NEXT: GROUP   - Static restrictions on the dispatch group: 0
# CHECK-NEXT: USH     - Uncategorised Structural Hazard:           0

# CHECK:      Dispatch Logic - number of cycles where we saw N micro opcodes dispatched:
# CHECK-NEXT: [# dispatched], [# cycles]
# CHECK-NEXT:  0,              24  (6.0%)
# CHECK-NEXT:  1,              372  (92.3%)
# CHECK-NEXT:  4,              7  (1.7%)

# CHECK:      Schedulers - number of cycles where we saw N micro opcodes issued:
# CHECK-NEXT: [# issued], [# cycles]
# CHECK-NEXT:  0,          3  (0.7%)
# CHECK-NEXT:  1,          400  (99.3%)

# CHECK:      Scheduler's queue usage:
# CHECK-NEXT: [1] Resource name.
# CHECK-NEXT: [2] Average number of used buffer entries.
# CHECK-NEXT: [3] Maximum number of used buffer entries.
# CHECK-NEXT: [4] Total number of buffer entries.

# CHECK:       [1]            [2]        [3]        [4]
# CHECK-NEXT: PdEX             21         22         40
# CHECK-NEXT: PdFPU            0          0          64
# CHECK-NEXT: PdLoad           0          0          40
# CHECK-NEXT: PdStore          22         23         24

# CHECK:      Resources:
# CHECK-NEXT: [0.0] - PdAGLU01
# CHECK-NEXT: [0.1] - PdAGLU01
# CHECK-NEXT: [1]   - PdBranch
# CHECK-NEXT: [2]   - PdCount
# CHECK-NEXT: [3]   - PdDiv
# CHECK-NEXT: [4]   - PdEX0
# CHECK-NEXT: [5]   - PdEX1
# CHECK-NEXT: [6]   - PdFPCVT
# CHECK-NEXT: [7.0] - PdFPFMA
# CHECK-NEXT: [7.1] - PdFPFMA
# CHECK-NEXT: [8.0] - PdFPMAL
# CHECK-NEXT: [8.1] - PdFPMAL
# CHECK-NEXT: [9]   - PdFPMMA
# CHECK-NEXT: [10]  - PdFPSTO
# CHECK-NEXT: [11]  - PdFPU0
# CHECK-NEXT: [12]  - PdFPU1
# CHECK-NEXT: [13]  - PdFPU2
# CHECK-NEXT: [14]  - PdFPU3
# CHECK-NEXT: [15]  - PdFPXBR
# CHECK-NEXT: [16.0] - PdLoad
# CHECK-NEXT: [16.1] - PdLoad
# CHECK-NEXT: [17]  - PdMul
# CHECK-NEXT: [18]  - PdStore

# CHECK:      Resource pressure per iteration:
# CHECK-NEXT: [0.0]  [0.1]  [1]    [2]    [3]    [4]    [5]    [6]    [7.0]  [7.1]  [8.0]  [8.1]  [9]    [10]   [11]   [12]   [13]   [14]   [15]   [16.0] [16.1] [17]   [18]
# CHECK-NEXT: 2.00   2.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     4.00

# CHECK:      Resource pressure by instruction:
# CHECK-NEXT: [0.0]  [0.1]  [1]    [2]    [3]    [4]    [5]    [6]    [7.0]  [7.1]  [8.0]  [8.1]  [9]    [10]   [11]   [12]   [13]   [14]   [15]   [16.0] [16.1] [17]   [18]   Instructions:
# CHECK-NEXT:  -     1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movq	%rsp, (%rax)
# CHECK-NEXT: 1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movq	%rbp, (%rcx)
# CHECK-NEXT:  -     1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movq	%rsi, (%rdx)
# CHECK-NEXT: 1.00    -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -      -     1.00   movq	%rdi, (%rbx)

# CHECK:      Timeline view:
# CHECK-NEXT: Index     0123456

# CHECK:      [0,0]     DeER ..   movq	%rsp, (%rax)
# CHECK-NEXT: [0,1]     D=eER..   movq	%rbp, (%rcx)
# CHECK-NEXT: [0,2]     D==eER.   movq	%rsi, (%rdx)
# CHECK-NEXT: [0,3]     D===eER   movq	%rdi, (%rbx)

# CHECK:      Average Wait times (based on the timeline view):
# CHECK-NEXT: [0]: Executions
# CHECK-NEXT: [1]: Average time spent waiting in a scheduler's queue
# CHECK-NEXT: [2]: Average time spent waiting in a scheduler's queue while ready
# CHECK-NEXT: [3]: Average time elapsed from WB until retire stage

# CHECK:            [0]    [1]    [2]    [3]
# CHECK-NEXT: 0.     1     1.0    1.0    0.0       movq	%rsp, (%rax)
# CHECK-NEXT: 1.     1     2.0    1.0    0.0       movq	%rbp, (%rcx)
# CHECK-NEXT: 2.     1     3.0    1.0    0.0       movq	%rsi, (%rdx)
# CHECK-NEXT: 3.     1     4.0    1.0    0.0       movq	%rdi, (%rbx)
# CHECK-NEXT:        1     2.5    1.0    0.0       <total>

# CHECK:      [4] Code Region

# CHECK:      Iterations:        100
# CHECK-NEXT: Instructions:      400
# CHECK-NEXT: Total Cycles:      803
# CHECK-NEXT: Total uOps:        400

# CHECK:      Dispatch Width:    4
# CHECK-NEXT: uOps Per Cycle:    0.50
# CHECK-NEXT: IPC:               0.50
# CHECK-NEXT: Block RThroughput: 6.0

# CHECK:      Instruction Info:
# CHECK-NEXT: [1]: #uOps
# CHECK-NEXT: [2]: Latency
# CHECK-NEXT: [3]: RThroughput
# CHECK-NEXT: [4]: MayLoad
# CHECK-NEXT: [5]: MayStore
# CHECK-NEXT: [6]: HasSideEffects (U)

# CHECK:      [1]    [2]    [3]    [4]    [5]    [6]    Instructions:
# CHECK-NEXT:  1      2     1.50           *      U     movd	%mm0, (%rax)
# CHECK-NEXT:  1      2     1.50           *      U     movd	%mm1, (%rcx)
# CHECK-NEXT:  1      2     1.50           *      U     movd	%mm2, (%rdx)
# CHECK-NEXT:  1      2     1.50           *      U     movd	%mm3, (%rbx)

# CHECK:      Dynamic Dispatch Stall Cycles:
# CHECK-NEXT: RAT     - Register unavailable:                      0
# CHECK-NEXT: RCU     - Retire tokens unavailable:                 0
# CHECK-NEXT: SCHEDQ  - Scheduler full:                            0
# CHECK-NEXT: LQ      - Load queue full:                           0
# CHECK-NEXT: SQ      - Store queue full:                          748  (93.2%)
# CHECK-NEXT: GROUP   - Static restrictions on the dispatch group: 0
# CHECK-NEXT: USH     - Uncategorised Structural Hazard:           0

# CHECK:      Dispatch Logic - number of cycles where we saw N micro opcodes dispatched:
# CHECK-NEXT: [# dispatched], [# cycles]
# CHECK-NEXT:  0,              422  (52.6%)
# CHECK-NEXT:  1,              374  (46.6%)
# CHECK-NEXT:  2,              1  (0.1%)
# CHECK-NEXT:  4,              6  (0.7%)

# CHECK:      Schedulers - number of cycles where we saw N micro opcodes issued:
# CHECK-NEXT: [# issued], [# cycles]
# CHECK-NEXT:  0,          403  (50.2%)
# CHECK-NEXT:  1,          400  (49.8%)

# CHECK:      Scheduler's queue usage:
# CHECK-NEXT: [1] Resource name.
# CHECK-NEXT: [2] Average number of used buffer entries.
# CHECK-NEXT: [3] Maximum number of used buffer entries.
# CHECK-NEXT: [4] Total number of buffer entries.

# CHECK:       [1]            [2]        [3]        [4]
# CHECK-NEXT: PdEX             21         23         40
# CHECK-NEXT: PdFPU            21         23         64
# CHECK-NEXT: PdLoad           0          0          40
# CHECK-NEXT: PdStore          22         24         24

# CHECK:      Resources:
# CHECK-NEXT: [0.0] - PdAGLU01
# CHECK-NEXT: [0.1] - PdAGLU01
# CHECK-NEXT: [1]   - PdBranch
# CHECK-NEXT: [2]   - PdCount
# CHECK-NEXT: [3]   - PdDiv
# CHECK-NEXT: [4]   - PdEX0
# CHECK-NEXT: [5]   - PdEX1
# CHECK-NEXT: [6]   - PdFPCVT
# CHECK-NEXT: [7.0] - PdFPFMA
# CHECK-NEXT: [7.1] - PdFPFMA
# CHECK-NEXT: [8.0] - PdFPMAL
# CHECK-NEXT: [8.1] - PdFPMAL
# CHECK-NEXT: [9]   - PdFPMMA
# CHECK-NEXT: [10]  - PdFPSTO
# CHECK-NEXT: [11]  - PdFPU0
# CHECK-NEXT: [12]  - PdFPU1
# CHECK-NEXT: [13]  - PdFPU2
# CHECK-NEXT: [14]  - PdFPU3
# CHECK-NEXT: [15]  - PdFPXBR
# CHECK-NEXT: [16.0] - PdLoad
# CHECK-NEXT: [16.1] - PdLoad
# CHECK-NEXT: [17]  - PdMul
# CHECK-NEXT: [18]  - PdStore

# CHECK:      Resource pressure per iteration:
# CHECK-NEXT: [0.0]  [0.1]  [1]    [2]    [3]    [4]    [5]    [6]    [7.0]  [7.1]  [8.0]  [8.1]  [9]    [10]   [11]   [12]   [13]   [14]   [15]   [16.0] [16.1] [17]   [18]
# CHECK-NEXT: 2.00   2.00    -      -      -      -      -      -      -      -      -      -      -     4.00    -      -     6.00   6.00    -      -      -      -     4.00

# CHECK:      Resource pressure by instruction:
# CHECK-NEXT: [0.0]  [0.1]  [1]    [2]    [3]    [4]    [5]    [6]    [7.0]  [7.1]  [8.0]  [8.1]  [9]    [10]   [11]   [12]   [13]   [14]   [15]   [16.0] [16.1] [17]   [18]   Instructions:
# CHECK-NEXT:  -     1.00    -      -      -      -      -      -      -      -      -      -      -     1.00    -      -      -     3.00    -      -      -      -     1.00   movd	%mm0, (%rax)
# CHECK-NEXT: 1.00    -      -      -      -      -      -      -      -      -      -      -      -     1.00    -      -     3.00    -      -      -      -      -     1.00   movd	%mm1, (%rcx)
# CHECK-NEXT:  -     1.00    -      -      -      -      -      -      -      -      -      -      -     1.00    -      -      -     3.00    -      -      -      -     1.00   movd	%mm2, (%rdx)
# CHECK-NEXT: 1.00    -      -      -      -      -      -      -      -      -      -      -      -     1.00    -      -     3.00    -      -      -      -      -     1.00   movd	%mm3, (%rbx)

# CHECK:      Timeline view:
# CHECK-NEXT:                     0
# CHECK-NEXT: Index     0123456789

# CHECK:      [0,0]     DeeER.    .   movd	%mm0, (%rax)
# CHECK-NEXT: [0,1]     D==eeER   .   movd	%mm1, (%rcx)
# CHECK-NEXT: [0,2]     D====eeER .   movd	%mm2, (%rdx)
# CHECK-NEXT: [0,3]     D======eeER   movd	%mm3, (%rbx)

# CHECK:      Average Wait times (based on the timeline view):
# CHECK-NEXT: [0]: Executions
# CHECK-NEXT: [1]: Average time spent waiting in a scheduler's queue
# CHECK-NEXT: [2]: Average time spent waiting in a scheduler's queue while ready
# CHECK-NEXT: [3]: Average time elapsed from WB until retire stage

# CHECK:            [0]    [1]    [2]    [3]
# CHECK-NEXT: 0.     1     1.0    1.0    0.0       movd	%mm0, (%rax)
# CHECK-NEXT: 1.     1     3.0    0.0    0.0       movd	%mm1, (%rcx)
# CHECK-NEXT: 2.     1     5.0    0.0    0.0       movd	%mm2, (%rdx)
# CHECK-NEXT: 3.     1     7.0    0.0    0.0       movd	%mm3, (%rbx)
# CHECK-NEXT:        1     4.0    0.3    0.0       <total>

# CHECK:      [5] Code Region

# CHECK:      Iterations:        100
# CHECK-NEXT: Instructions:      400
# CHECK-NEXT: Total Cycles:      602
# CHECK-NEXT: Total uOps:        400

# CHECK:      Dispatch Width:    4
# CHECK-NEXT: uOps Per Cycle:    0.66
# CHECK-NEXT: IPC:               0.66
# CHECK-NEXT: Block RThroughput: 6.0

# CHECK:      Instruction Info:
# CHECK-NEXT: [1]: #uOps
# CHECK-NEXT: [2]: Latency
# CHECK-NEXT: [3]: RThroughput
# CHECK-NEXT: [4]: MayLoad
# CHECK-NEXT: [5]: MayStore
# CHECK-NEXT: [6]: HasSideEffects (U)

# CHECK:      [1]    [2]    [3]    [4]    [5]    [6]    Instructions:
# CHECK-NEXT:  1      1     1.50           *            movaps	%xmm0, (%rax)
# CHECK-NEXT:  1      1     1.50           *            movaps	%xmm1, (%rcx)
# CHECK-NEXT:  1      1     1.50           *            movaps	%xmm2, (%rdx)
# CHECK-NEXT:  1      1     1.50           *            movaps	%xmm3, (%rbx)

# CHECK:      Dynamic Dispatch Stall Cycles:
# CHECK-NEXT: RAT     - Register unavailable:                      0
# CHECK-NEXT: RCU     - Retire tokens unavailable:                 0
# CHECK-NEXT: SCHEDQ  - Scheduler full:                            0
# CHECK-NEXT: LQ      - Load queue full:                           0
# CHECK-NEXT: SQ      - Store queue full:                          559  (92.9%)
# CHECK-NEXT: GROUP   - Static restrictions on the dispatch group: 0
# CHECK-NEXT: USH     - Uncategorised Structural Hazard:           0

# CHECK:      Dispatch Logic - number of cycles where we saw N micro opcodes dispatched:
# CHECK-NEXT: [# dispatched], [# cycles]
# CHECK-NEXT:  0,              222  (36.9%)
# CHECK-NEXT:  1,              373  (62.0%)
# CHECK-NEXT:  3,              1  (0.2%)
# CHECK-NEXT:  4,              6  (1.0%)

# CHECK:      Schedulers - number of cycles where we saw N micro opcodes issued:
# CHECK-NEXT: [# issued], [# cycles]
# CHECK-NEXT:  0,          202  (33.6%)
# CHECK-NEXT:  1,          400  (66.4%)

# CHECK:      Scheduler's queue usage:
# CHECK-NEXT: [1] Resource name.
# CHECK-NEXT: [2] Average number of used buffer entries.
# CHECK-NEXT: [3] Maximum number of used buffer entries.
# CHECK-NEXT: [4] Total number of buffer entries.

# CHECK:       [1]            [2]        [3]        [4]
# CHECK-NEXT: PdEX             21         23         40
# CHECK-NEXT: PdFPU            21         23         64
# CHECK-NEXT: PdLoad           0          0          40
# CHECK-NEXT: PdStore          22         24         24

# CHECK:      Resources:
# CHECK-NEXT: [0.0] - PdAGLU01
# CHECK-NEXT: [0.1] - PdAGLU01
# CHECK-NEXT: [1]   - PdBranch
# CHECK-NEXT: [2]   - PdCount
# CHECK-NEXT: [3]   - PdDiv
# CHECK-NEXT: [4]   - PdEX0
# CHECK-NEXT: [5]   - PdEX1
# CHECK-NEXT: [6]   - PdFPCVT
# CHECK-NEXT: [7.0] - PdFPFMA
# CHECK-NEXT: [7.1] - PdFPFMA
# CHECK-NEXT: [8.0] - PdFPMAL
# CHECK-NEXT: [8.1] - PdFPMAL
# CHECK-NEXT: [9]   - PdFPMMA
# CHECK-NEXT: [10]  - PdFPSTO
# CHECK-NEXT: [11]  - PdFPU0
# CHECK-NEXT: [12]  - PdFPU1
# CHECK-NEXT: [13]  - PdFPU2
# CHECK-NEXT: [14]  - PdFPU3
# CHECK-NEXT: [15]  - PdFPXBR
# CHECK-NEXT: [16.0] - PdLoad
# CHECK-NEXT: [16.1] - PdLoad
# CHECK-NEXT: [17]  - PdMul
# CHECK-NEXT: [18]  - PdStore

# CHECK:      Resource pressure per iteration:
# CHECK-NEXT: [0.0]  [0.1]  [1]    [2]    [3]    [4]    [5]    [6]    [7.0]  [7.1]  [8.0]  [8.1]  [9]    [10]   [11]   [12]   [13]   [14]   [15]   [16.0] [16.1] [17]   [18]
# CHECK-NEXT: 2.00   2.00    -      -      -      -      -      -      -      -      -      -      -     4.00    -      -     6.00   6.00    -      -      -      -     4.00

# CHECK:      Resource pressure by instruction:
# CHECK-NEXT: [0.0]  [0.1]  [1]    [2]    [3]    [4]    [5]    [6]    [7.0]  [7.1]  [8.0]  [8.1]  [9]    [10]   [11]   [12]   [13]   [14]   [15]   [16.0] [16.1] [17]   [18]   Instructions:
# CHECK-NEXT:  -     1.00    -      -      -      -      -      -      -      -      -      -      -     1.00    -      -      -     3.00    -      -      -      -     1.00   movaps	%xmm0, (%rax)
# CHECK-NEXT: 1.00    -      -      -      -      -      -      -      -      -      -      -      -     1.00    -      -     3.00    -      -      -      -      -     1.00   movaps	%xmm1, (%rcx)
# CHECK-NEXT:  -     1.00    -      -      -      -      -      -      -      -      -      -      -     1.00    -      -      -     3.00    -      -      -      -     1.00   movaps	%xmm2, (%rdx)
# CHECK-NEXT: 1.00    -      -      -      -      -      -      -      -      -      -      -      -     1.00    -      -     3.00    -      -      -      -      -     1.00   movaps	%xmm3, (%rbx)

# CHECK:      Timeline view:
# CHECK-NEXT: Index     01234567

# CHECK:      [0,0]     DeER . .   movaps	%xmm0, (%rax)
# CHECK-NEXT: [0,1]     D=eER. .   movaps	%xmm1, (%rcx)
# CHECK-NEXT: [0,2]     D===eER.   movaps	%xmm2, (%rdx)
# CHECK-NEXT: [0,3]     D====eER   movaps	%xmm3, (%rbx)

# CHECK:      Average Wait times (based on the timeline view):
# CHECK-NEXT: [0]: Executions
# CHECK-NEXT: [1]: Average time spent waiting in a scheduler's queue
# CHECK-NEXT: [2]: Average time spent waiting in a scheduler's queue while ready
# CHECK-NEXT: [3]: Average time elapsed from WB until retire stage

# CHECK:            [0]    [1]    [2]    [3]
# CHECK-NEXT: 0.     1     1.0    1.0    0.0       movaps	%xmm0, (%rax)
# CHECK-NEXT: 1.     1     2.0    1.0    0.0       movaps	%xmm1, (%rcx)
# CHECK-NEXT: 2.     1     4.0    2.0    0.0       movaps	%xmm2, (%rdx)
# CHECK-NEXT: 3.     1     5.0    1.0    0.0       movaps	%xmm3, (%rbx)
# CHECK-NEXT:        1     3.0    1.3    0.0       <total>

# CHECK:      [6] Code Region

# CHECK:      Iterations:        100
# CHECK-NEXT: Instructions:      400
# CHECK-NEXT: Total Cycles:      7170
# CHECK-NEXT: Total uOps:        1600

# CHECK:      Dispatch Width:    4
# CHECK-NEXT: uOps Per Cycle:    0.22
# CHECK-NEXT: IPC:               0.06
# CHECK-NEXT: Block RThroughput: 72.0

# CHECK:      Instruction Info:
# CHECK-NEXT: [1]: #uOps
# CHECK-NEXT: [2]: Latency
# CHECK-NEXT: [3]: RThroughput
# CHECK-NEXT: [4]: MayLoad
# CHECK-NEXT: [5]: MayStore
# CHECK-NEXT: [6]: HasSideEffects (U)

# CHECK:      [1]    [2]    [3]    [4]    [5]    [6]    Instructions:
# CHECK-NEXT:  4      1     18.00          *            vmovaps	%ymm0, (%rax)
# CHECK-NEXT:  4      1     18.00          *            vmovaps	%ymm1, (%rcx)
# CHECK-NEXT:  4      1     18.00          *            vmovaps	%ymm2, (%rdx)
# CHECK-NEXT:  4      1     18.00          *            vmovaps	%ymm3, (%rbx)

# CHECK:      Dynamic Dispatch Stall Cycles:
# CHECK-NEXT: RAT     - Register unavailable:                      0
# CHECK-NEXT: RCU     - Retire tokens unavailable:                 0
# CHECK-NEXT: SCHEDQ  - Scheduler full:                            5777  (80.6%)
# CHECK-NEXT: LQ      - Load queue full:                           0
# CHECK-NEXT: SQ      - Store queue full:                          561  (7.8%)
# CHECK-NEXT: GROUP   - Static restrictions on the dispatch group: 0
# CHECK-NEXT: USH     - Uncategorised Structural Hazard:           0

# CHECK:      Dispatch Logic - number of cycles where we saw N micro opcodes dispatched:
# CHECK-NEXT: [# dispatched], [# cycles]
# CHECK-NEXT:  0,              6770  (94.4%)
# CHECK-NEXT:  4,              400  (5.6%)

# CHECK:      Schedulers - number of cycles where we saw N micro opcodes issued:
# CHECK-NEXT: [# issued], [# cycles]
# CHECK-NEXT:  0,          6770  (94.4%)
# CHECK-NEXT:  4,          400  (5.6%)

# CHECK:      Scheduler's queue usage:
# CHECK-NEXT: [1] Resource name.
# CHECK-NEXT: [2] Average number of used buffer entries.
# CHECK-NEXT: [3] Maximum number of used buffer entries.
# CHECK-NEXT: [4] Total number of buffer entries.

# CHECK:       [1]            [2]        [3]        [4]
# CHECK-NEXT: PdEX             23         24         40
# CHECK-NEXT: PdFPU            23         24         64
# CHECK-NEXT: PdLoad           0          0          40
# CHECK-NEXT: PdStore          23         24         24

# CHECK:      Resources:
# CHECK-NEXT: [0.0] - PdAGLU01
# CHECK-NEXT: [0.1] - PdAGLU01
# CHECK-NEXT: [1]   - PdBranch
# CHECK-NEXT: [2]   - PdCount
# CHECK-NEXT: [3]   - PdDiv
# CHECK-NEXT: [4]   - PdEX0
# CHECK-NEXT: [5]   - PdEX1
# CHECK-NEXT: [6]   - PdFPCVT
# CHECK-NEXT: [7.0] - PdFPFMA
# CHECK-NEXT: [7.1] - PdFPFMA
# CHECK-NEXT: [8.0] - PdFPMAL
# CHECK-NEXT: [8.1] - PdFPMAL
# CHECK-NEXT: [9]   - PdFPMMA
# CHECK-NEXT: [10]  - PdFPSTO
# CHECK-NEXT: [11]  - PdFPU0
# CHECK-NEXT: [12]  - PdFPU1
# CHECK-NEXT: [13]  - PdFPU2
# CHECK-NEXT: [14]  - PdFPU3
# CHECK-NEXT: [15]  - PdFPXBR
# CHECK-NEXT: [16.0] - PdLoad
# CHECK-NEXT: [16.1] - PdLoad
# CHECK-NEXT: [17]  - PdMul
# CHECK-NEXT: [18]  - PdStore

# CHECK:      Resource pressure per iteration:
# CHECK-NEXT: [0.0]  [0.1]  [1]    [2]    [3]    [4]    [5]    [6]    [7.0]  [7.1]  [8.0]  [8.1]  [9]    [10]   [11]   [12]   [13]   [14]   [15]   [16.0] [16.1] [17]   [18]
# CHECK-NEXT: 2.00   2.00    -      -      -      -      -      -      -      -      -      -      -     8.00    -      -     72.00  72.00   -      -      -      -     4.00

# CHECK:      Resource pressure by instruction:
# CHECK-NEXT: [0.0]  [0.1]  [1]    [2]    [3]    [4]    [5]    [6]    [7.0]  [7.1]  [8.0]  [8.1]  [9]    [10]   [11]   [12]   [13]   [14]   [15]   [16.0] [16.1] [17]   [18]   Instructions:
# CHECK-NEXT:  -     1.00    -      -      -      -      -      -      -      -      -      -      -     2.00    -      -      -     36.00   -      -      -      -     1.00   vmovaps	%ymm0, (%rax)
# CHECK-NEXT: 1.00    -      -      -      -      -      -      -      -      -      -      -      -     2.00    -      -     36.00   -      -      -      -      -     1.00   vmovaps	%ymm1, (%rcx)
# CHECK-NEXT:  -     1.00    -      -      -      -      -      -      -      -      -      -      -     2.00    -      -      -     36.00   -      -      -      -     1.00   vmovaps	%ymm2, (%rdx)
# CHECK-NEXT: 1.00    -      -      -      -      -      -      -      -      -      -      -      -     2.00    -      -     36.00   -      -      -      -      -     1.00   vmovaps	%ymm3, (%rbx)

# CHECK:      Timeline view:
# CHECK-NEXT:                     0123456789          0123456789
# CHECK-NEXT: Index     0123456789          0123456789          01

# CHECK:      [0,0]     DeER .    .    .    .    .    .    .    ..   vmovaps	%ymm0, (%rax)
# CHECK-NEXT: [0,1]     .D=eER    .    .    .    .    .    .    ..   vmovaps	%ymm1, (%rcx)
# CHECK-NEXT: [0,2]     . D==================================eER..   vmovaps	%ymm2, (%rdx)
# CHECK-NEXT: [0,3]     .  D===================================eER   vmovaps	%ymm3, (%rbx)

# CHECK:      Average Wait times (based on the timeline view):
# CHECK-NEXT: [0]: Executions
# CHECK-NEXT: [1]: Average time spent waiting in a scheduler's queue
# CHECK-NEXT: [2]: Average time spent waiting in a scheduler's queue while ready
# CHECK-NEXT: [3]: Average time elapsed from WB until retire stage

# CHECK:            [0]    [1]    [2]    [3]
# CHECK-NEXT: 0.     1     1.0    1.0    0.0       vmovaps	%ymm0, (%rax)
# CHECK-NEXT: 1.     1     2.0    2.0    0.0       vmovaps	%ymm1, (%rcx)
# CHECK-NEXT: 2.     1     35.0   34.0   0.0       vmovaps	%ymm2, (%rdx)
# CHECK-NEXT: 3.     1     36.0   2.0    0.0       vmovaps	%ymm3, (%rbx)
# CHECK-NEXT:        1     18.5   9.8    0.0       <total>
