add_llvm_component_library(LLVMFgpuDisassembler
  FgpuDisassembler.cpp

  LINK_COMPONENTS
  MCDisassembler
  FgpuInfo
  Support

  ADD_TO_COMPONENT
  Fgpu
  )
