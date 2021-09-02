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
#include "llvm/Target/TargetInstrInfo.h"
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

#define DEBUG_TYPE "del-branch"

STATISTIC(NumDelBranch, "Number of useless branch deleted");

static cl::opt<bool> EnableDelBranch(
  "enable-fgpu-del-useless-branch",
  cl::init(true),
  cl::desc("Delete useless branch instructions: beq r0, r0, 0."),
  cl::Hidden);

static cl::opt<bool> EnableReworkBranches(
  "enable-fgpu-rework-branches",
  cl::init(true),
  cl::desc("do intelligent things with backedges and fall-thru"),
  cl::Hidden);


namespace llvm {
  void initializeDelBranchPass(PassRegistry&);
}


namespace {
  class DelBranch : public MachineFunctionPass {
  public:
    static char ID;
  private:
    MachinePostDominatorTree *PDomTree = nullptr;
    bool fixBranchPolarity(MachineBasicBlock &MBB);
    bool runOnMachineBasicBlock(MachineBasicBlock &MBB, MachineBasicBlock &MBBN);
    void init(MachineFunction &MF) {
      PDomTree = &getAnalysis<MachinePostDominatorTree>();
    }
  public:
    DelBranch() :
      MachineFunctionPass(ID), PDomTree(nullptr) {

    }
    void getAnalysisUsage(AnalysisUsage &AU) const override{
      AU.addPreserved<MachineFunctionAnalysis>();
      AU.addRequired<MachineFunctionAnalysis>();
      AU.addRequired<MachinePostDominatorTree>();
      AU.addPreserved<MachinePostDominatorTree>();
    }

    virtual const char *getPassName() const {
      return "Fgpu Del Useless branch";
    }
    bool runOnMachineFunction(MachineFunction &F);

  };
}

char DelBranch::ID = 0;


INITIALIZE_PASS_BEGIN(DelBranch, "DelBranch", "Fix branches", false, false)
INITIALIZE_PASS_DEPENDENCY(MachinePostDominatorTree)
INITIALIZE_PASS_END(DelBranch, "DelBranch", "Fix branches", false, false)


bool DelBranch::runOnMachineFunction(MachineFunction &F) {
  if (not(EnableDelBranch)) {
    return false;
  }
    
  init(F);
  DEBUG(dbgs() << "soubhi: delBranch on Function pass entered\n");
  bool Changed = false, flippedPolarity = true;
  
  while(flippedPolarity and EnableReworkBranches) {
    flippedPolarity = false;
    for(auto I = F.begin(), E = F.end(); I != E; ++I) {
      flippedPolarity |= fixBranchPolarity(*I);
    }
    Changed |= flippedPolarity;
  }
  
  MachineFunction::iterator FJ = F.begin();
  if (FJ != F.end())
    FJ++;
  if (FJ == F.end())
    return Changed;
  for (MachineFunction::iterator FI = F.begin(), FE = F.end(); FJ != FE;
       ++FI, ++FJ) {
    Changed |= runOnMachineBasicBlock(*FI, *FJ);
  }
  return Changed;
}

bool DelBranch::fixBranchPolarity(MachineBasicBlock &MBB) {
  /* dsheffie - step1 : check if there's two branches in a basicblock */
  int n_br = 0;
  SmallVector<MachineInstr*, 2> br;
  for(auto I = MBB.begin(), E = MBB.end(); I != E; ++I) {
    if ( I->getOpcode() == Fgpu::BEQ ) {
      br.push_back(&(*I));
      n_br++;
    }
  }
  if(n_br != 2) {
    return false;
  }

  /* dsheffie - 
   * check second branch is unconditional 
   * and the target dominates the current basicblock (a backedge)
   */
  for(int i = 0; i < 2; i++) {
    if(not(br[i]->getOperand(i).isReg())) {
      return false;
    }
  }
  if(br[1]->getOperand(0).getReg()!=br[1]->getOperand(1).getReg()) {
    return false;
  }
  if(not(PDomTree->dominates(br[1]->getOperand(2).getMBB(), &MBB))) {
    return false;
  }
  /* change first branches target and branch type */


  DEBUG(dbgs() << "dsheffie: found flip candidate\n");
  auto secondBrTarget = br[0]->getOperand(2).getMBB();
  MachineBasicBlock::iterator TI = MBB.end();
  while(&*TI != br[0]) {
    TI--;
    assert(TI != MBB.begin());
  }
  auto TII = MBB.getParent()->getSubtarget<FgpuSubtarget>().getInstrInfo();
  auto MIB = BuildMI(MBB, TI, br[0]->getDebugLoc(), TII->get(Fgpu::BNE));
  MIB.addReg(TI->getOperand(0).getReg());
  MIB.addReg(TI->getOperand(1).getReg());
  MIB.addMBB(br[1]->getOperand(2).getMBB());
  MBB.erase(TI);

  /* create new unconditional branch with other target */
  TI = MBB.end();
  while(&*TI != br[1]) {
    TI--;
    assert(TI != MBB.begin());
  }
  MIB = BuildMI(MBB, TI, br[1]->getDebugLoc(), TII->get(Fgpu::BEQ));
  MIB.addReg(TI->getOperand(0).getReg());
  MIB.addReg(TI->getOperand(1).getReg());
  MIB.addMBB(secondBrTarget);
  MBB.erase(TI);
  
  dbgs() << MBB << "\n";

  return true;
}

bool DelBranch::runOnMachineBasicBlock(MachineBasicBlock &MBB, MachineBasicBlock &MBBN) {
  DEBUG(dbgs() << "soubhi: delBranch on basic block pass entered\n");
  bool Changed = false;


    
  MachineBasicBlock::iterator I = MBB.end();
  if (I != MBB.begin())
    I--;	// set I to the last instruction
  else
    return Changed;

  
  
  if ( (I->getOpcode() == Fgpu::BEQ || I->getOpcode() == Fgpu::BNE) && I->getOperand(2).getMBB() == &MBBN) {
    // I is the instruction of "beq rx, rx, #offset=0", as follows,
    //     beq r0, r0,	$BB0_3
    // $BB0_3:
    //     add	r1, r1, r2
    ++NumDelBranch;
    MBB.erase(I);	// delete the "beq r0, r0, 0" instruction
    Changed = true;	// Notify LLVM kernel Changed
  }
  return Changed;

}

/// createFgpuDelBranchPass - Returns a pass that DelBranch in Fgpu MachineFunctions
FunctionPass *llvm::createFgpuDelBranchPass(FgpuTargetMachine &tm) {
  return new DelBranch();
}

