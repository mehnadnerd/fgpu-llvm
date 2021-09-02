add_llvm_component_library(LLVMFgpuAsmPrinter
	FgpuInstPrinter.cpp

	LINK_COMPONENTS
	MC
	MCParser
	FgpuInfo
	FgpuDesc
	Support

	ADD_TO_COMPONENT
	Fgpu
  )
