add_llvm_component_library(LLVMFgpuAsmParser
  FgpuAsmParser.cpp

  LINK_COMPONENTS
  MC
  MCParser
  FgpuDesc
  FgpuInfo
  Support

  ADD_TO_COMPONENT
  Fgpu
  )
