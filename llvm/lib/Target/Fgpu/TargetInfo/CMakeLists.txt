add_llvm_component_library(LLVMFgpuInfo
  FgpuTargetInfo.cpp

  LINK_COMPONENTS
  Support

  ADD_TO_COMPONENT
  Fgpu
  )
