# MCTargetDesc/CMakeLists.txt
add_llvm_component_library(LLVMFgpuDesc
  FgpuMCTargetDesc.cpp
  FgpuABIInfo.cpp
  FgpuMCAsmInfo.cpp
  FgpuAsmBackend.cpp
  FgpuMCCodeEmitter.cpp
  FgpuMCExpr.cpp
  FgpuELFObjectWriter.cpp
  FgpuTargetStreamer.cpp

  LINK_COMPONENTS
  MC
  Support

  ADD_TO_COMPONENT
  Fgpu
  )




