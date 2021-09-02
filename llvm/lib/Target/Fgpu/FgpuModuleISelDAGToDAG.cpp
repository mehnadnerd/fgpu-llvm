//===----------------------------------------------------------------------===//
// Instruction Selector Subtarget Control
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// This file defines a pass used to change the subtarget for the
// Fgpu Instruction selector.
//
//===----------------------------------------------------------------------===//

#include "Fgpu.h"
#include "FgpuTargetMachine.h"
#include "llvm/CodeGen/StackProtector.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "Fgpu-isel"

namespace {
  class FgpuModuleDAGToDAGISel : public MachineFunctionPass {
  public:
    static char ID;

    FgpuModuleDAGToDAGISel() : MachineFunctionPass(ID) {}

    // Pass Name
    StringRef getPassName() const override {
      return "Fgpu DAG->DAG Pattern Instruction Selection";
    }

    void getAnalysisUsage(AnalysisUsage &AU) const override {
      AU.addRequired<TargetPassConfig>();
      AU.addPreserved<StackProtector>();
      MachineFunctionPass::getAnalysisUsage(AU);
    }

    bool runOnMachineFunction(MachineFunction &MF) override;
  };

  char FgpuModuleDAGToDAGISel::ID = 0;
}

bool FgpuModuleDAGToDAGISel::runOnMachineFunction(MachineFunction &MF) {
  LLVM_DEBUG(errs() << "In FgpuModuleDAGToDAGISel::runMachineFunction\n");
  auto &TPC = getAnalysis<TargetPassConfig>();
  auto &TM = TPC.getTM<FgpuTargetMachine>();
  TM.resetSubtarget(&MF);
  return false;
}

llvm::FunctionPass *llvm::createFgpuModuleISelDagPass() {
  return new FgpuModuleDAGToDAGISel();
}
