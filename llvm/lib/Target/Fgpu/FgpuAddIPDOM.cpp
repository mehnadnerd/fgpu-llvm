//===-- FgpuDelUselessBranch.cpp - Fgpu DelBranch -------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Simple pass to fills delay slots with useful instructions.
//
//===----------------------------------------------------------------------===//

#include "Fgpu.h"
#include "FgpuTargetMachine.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/Debug.h"

#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionAnalysis.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachinePostDominators.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

using namespace llvm;

#define DEBUG_TYPE "ipdom-pass"

namespace llvm {
  void initializeIPDOMPass(PassRegistry&);
}

static cl::opt<bool> EnableIpdom(
  "enable-fgpu-ipdom",
  cl::init(false),
  cl::desc("Use immediate post-dominator control-flow"),
  cl::Hidden);

namespace {
  class IPDOM : public MachineFunctionPass {
  public:
    static char ID;
  private:
    MachinePostDominatorTree *PDomTree = nullptr;
    void init(MachineFunction &MF) {
      PDomTree = &getAnalysis<MachinePostDominatorTree>();
    }

    MachineBasicBlock *FindImmPDom(MachineBasicBlock &Block) {
      MachineBasicBlock *Idom = &Block;
      for(MachineBasicBlock *BB : Block.successors()) {
	Idom = PDomTree->findNearestCommonDominator(Idom, BB);
	if(Idom == nullptr) {
	  break;
	}
      }
      return Idom;
    }

  public:
    IPDOM() : MachineFunctionPass(ID), PDomTree(nullptr) { 
      initializeIPDOMPass(*PassRegistry::getPassRegistry());
    }
    
    void getAnalysisUsage(AnalysisUsage &AU) const override{
      AU.addPreserved<MachineFunctionAnalysis>();
      AU.addRequired<MachineFunctionAnalysis>();
      AU.addRequired<MachinePostDominatorTree>();
      AU.addPreserved<MachinePostDominatorTree>();
    }
    const llvm::StringRef getPassName() const override {
      return "Fgpu : augment CFG with post-dominator control flow instructions ";
    }
    bool runOnMachineBasicBlock(MachineBasicBlock &MBB);
    bool runOnMachineFunction(MachineFunction &F);
  };
  
} 

char IPDOM::ID = 0;

INITIALIZE_PASS_BEGIN(IPDOM, "IPDOM", "Create IPDOM insns", false, false)
INITIALIZE_PASS_DEPENDENCY(MachinePostDominatorTree)
INITIALIZE_PASS_END(IPDOM, "IPDOM", "Create IPDOM insns", false, false)

bool IPDOM::runOnMachineFunction(MachineFunction &F) {
  bool Changed = false;
  init(F);

  if (EnableIpdom) {
    for (MachineFunction::iterator FI = F.begin(), FE = F.end(); FI != FE; ++FI) {
      Changed |= runOnMachineBasicBlock(*FI);
    }
  }
  return Changed;
}

bool IPDOM::runOnMachineBasicBlock(MachineBasicBlock &MBB) {
  DEBUG(dbgs() << "soubhi: delBranch on basic block pass entered\n");
  bool Changed = false;

  int num_br = 0;
  for(auto it = MBB.begin(), e = MBB.end(); it != e; ++it) {
    if(it->getOpcode()==Fgpu::BEQ or it->getOpcode()==Fgpu::BNE) {
      num_br++;
    }
  }
  errs() << MBB.getName() << " has " << num_br << " branches\n";
  // fall-thru or direct branch 
  if(MBB.succ_size() == 1) {
    return false;
  }
  else if(MBB.succ_size() == 2) {
    MachineBasicBlock::iterator TI  = MBB.end(); TI--;
    if((TI->getOpcode()==Fgpu::BEQ or TI->getOpcode()==Fgpu::BNE) and 
       TI->getOperand(0).getReg() == TI->getOperand(1).getReg()) {
      TI--;
    }
    if(TI->getOpcode()==Fgpu::BEQ or TI->getOpcode()==Fgpu::BNE) {
      MachineBasicBlock *syncBB = FindImmPDom(MBB);
      if(syncBB) {
	auto TII = MBB.getParent()->getSubtarget<FgpuSubtarget>().getInstrInfo();
	unsigned br = TI->getOpcode()==Fgpu::BEQ ? Fgpu::PDOM_BEQ : Fgpu::PDOM_BNE;

	auto MIB = BuildMI(MBB, TI, TI->getDebugLoc(), TII->get(br));
	MIB.addReg(TI->getOperand(0).getReg());
	MIB.addReg(TI->getOperand(1).getReg());
	MIB.addMBB(TI->getOperand(2).getMBB());
	MachineBasicBlock::iterator T = TI;
	TI = std::prev(TI);
	MBB.erase(T);

	T = MBB.end();
	for(auto it = MBB.begin(), e = MBB.end(); it != e; ++it) {
	  if(it->getOpcode() == Fgpu::PDOM_BEQ or it->getOpcode() == Fgpu::PDOM_BNE) {
	    T = it;
	    break;
	  }
	}
	assert(T != MBB.end());
	auto SS_MIB = BuildMI(MBB, T, T->getDebugLoc(), TII->get(Fgpu::SET_SYNC));
	SS_MIB.addMBB(syncBB);


	MachineBasicBlock::iterator I  = syncBB->begin();
	if(not(I->getOpcode() == Fgpu::SET_SYNC)) {
	  BuildMI(*syncBB, I, I->getDebugLoc(), TII->get(Fgpu::SET_SYNC));
	  return true;
	}
      }
    }
  }

  return false;



  return Changed;

}

/// createFgpuDelBranchPass - Returns a pass that DelBranch in Fgpu MachineFunctions
FunctionPass *llvm::createFgpuIPDOMPass(FgpuTargetMachine &tm) {
  return new IPDOM();
}

