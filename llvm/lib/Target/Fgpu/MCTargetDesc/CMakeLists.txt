add_llvm_component_library(LLVMFgpuDesc
  FgpuABIInfo.cpp
  FgpuABIFlagsSection.cpp
  FgpuAsmBackend.cpp
  FgpuELFObjectWriter.cpp
  FgpuELFStreamer.cpp
  FgpuInstPrinter.cpp
  FgpuMCAsmInfo.cpp
  FgpuMCCodeEmitter.cpp
  FgpuMCExpr.cpp
  FgpuMCTargetDesc.cpp
  FgpuOptionRecord.cpp
  FgpuTargetStreamer.cpp

  LINK_COMPONENTS
  MC
  FgpuInfo
  Support

  ADD_TO_COMPONENT
  Fgpu
  )
