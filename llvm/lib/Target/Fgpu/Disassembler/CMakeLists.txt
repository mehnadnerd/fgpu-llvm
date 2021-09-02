add_llvm_component_library(LLVMFgpuDisassembler
    FgpuDisassembler.cpp

    LINK_COMPONENTS
    MCDisassembler
    FgpuInfo
    FgpuDesc
    Support

    ADD_TO_COMPONENT
    Fgpu
  )
