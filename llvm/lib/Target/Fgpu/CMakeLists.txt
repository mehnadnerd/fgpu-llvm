add_llvm_component_group(Fgpu HAS_JIT)

set(LLVM_TARGET_DEFINITIONS Fgpu.td)

tablegen(LLVM FgpuGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM FgpuGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM FgpuGenCallingConv.inc -gen-callingconv)
tablegen(LLVM FgpuGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM FgpuGenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM FgpuGenFastISel.inc -gen-fast-isel)
tablegen(LLVM FgpuGenGlobalISel.inc -gen-global-isel)
tablegen(LLVM FgpuGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM FgpuGenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM FgpuGenMCPseudoLowering.inc -gen-pseudo-lowering)
tablegen(LLVM FgpuGenRegisterBank.inc -gen-register-bank)
tablegen(LLVM FgpuGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM FgpuGenSubtargetInfo.inc -gen-subtarget)
#tablegen(LLVM FgpuGenExegesis.inc -gen-exegesis)

add_public_tablegen_target(FgpuCommonTableGen)

add_llvm_target(FgpuCodeGen
  FgpuAnalyzeImmediate.cpp
  FgpuAsmPrinter.cpp
  FgpuCallLowering.cpp
  FgpuCCState.cpp
  FgpuDelaySlotFiller.cpp
  FgpuExpandPseudo.cpp
  FgpuFastISel.cpp
  FgpuInstrInfo.cpp
  FgpuInstructionSelector.cpp
  FgpuISelDAGToDAG.cpp
  FgpuISelLowering.cpp
  FgpuFrameLowering.cpp
  FgpuLegalizerInfo.cpp
  FgpuBranchExpansion.cpp
  FgpuMCInstLower.cpp
  FgpuMachineFunction.cpp
  FgpuModuleISelDAGToDAG.cpp
  FgpuOptimizePICCall.cpp
  FgpuPreLegalizerCombiner.cpp
  FgpuRegisterBankInfo.cpp
  FgpuRegisterInfo.cpp
  FgpuSEFrameLowering.cpp
  FgpuSEInstrInfo.cpp
  FgpuSEISelDAGToDAG.cpp
  FgpuSEISelLowering.cpp
  FgpuSERegisterInfo.cpp
  FgpuSubtarget.cpp
  FgpuTargetMachine.cpp
  FgpuTargetObjectFile.cpp

  LINK_COMPONENTS
  Analysis
  AsmPrinter
  CodeGen
  Core
  MC
  FgpuDesc
  FgpuInfo
  SelectionDAG
  Support
  Target
  GlobalISel

  ADD_TO_COMPONENT
  Fgpu
  )

add_subdirectory(AsmParser)
add_subdirectory(Disassembler)
add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)
