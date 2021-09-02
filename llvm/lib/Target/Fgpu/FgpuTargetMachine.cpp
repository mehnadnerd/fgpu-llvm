//===-- FgpuTargetMachine.cpp - Define TargetMachine for Fgpu -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Implements the info about Fgpu target spec.
//
//===----------------------------------------------------------------------===//

#include "FgpuTargetMachine.h"
#include "Fgpu.h"
#include "MCTargetDesc/FgpuABIInfo.h"
#include "MCTargetDesc/FgpuMCTargetDesc.h"
#include "Fgpu16ISelDAGToDAG.h"
#include "FgpuSEISelDAGToDAG.h"
#include "FgpuSubtarget.h"
#include "FgpuTargetObjectFile.h"
#include "TargetInfo/FgpuTargetInfo.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/CodeGen/BasicTTIImpl.h"
#include "llvm/CodeGen/GlobalISel/IRTranslator.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelect.h"
#include "llvm/CodeGen/GlobalISel/Legalizer.h"
#include "llvm/CodeGen/GlobalISel/RegBankSelect.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/Function.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetOptions.h"
#include <string>

using namespace llvm;

#define DEBUG_TYPE "Fgpu"

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeFgpuTarget() {
  // Register the target.
  RegisterTargetMachine<FgpuebTargetMachine> X(getTheFgpuTarget());
  RegisterTargetMachine<FgpuelTargetMachine> Y(getTheFgpuelTarget());
  RegisterTargetMachine<FgpuebTargetMachine> A(getTheFgpu64Target());
  RegisterTargetMachine<FgpuelTargetMachine> B(getTheFgpu64elTarget());

  PassRegistry *PR = PassRegistry::getPassRegistry();
  initializeGlobalISel(*PR);
  initializeFgpuDelaySlotFillerPass(*PR);
  initializeFgpuBranchExpansionPass(*PR);
  initializeMicroFgpuSizeReducePass(*PR);
  initializeFgpuPreLegalizerCombinerPass(*PR);
}

static std::string computeDataLayout(const Triple &TT, StringRef CPU,
                                     const TargetOptions &Options,
                                     bool isLittle) {
  std::string Ret;
  FgpuABIInfo ABI = FgpuABIInfo::computeTargetABI(TT, CPU, Options.MCOptions);

  // There are both little and big endian Fgpu.
  if (isLittle)
    Ret += "e";
  else
    Ret += "E";

  if (ABI.IsO32())
    Ret += "-m:m";
  else
    Ret += "-m:e";

  // Pointers are 32 bit on some ABIs.
  if (!ABI.IsN64())
    Ret += "-p:32:32";

  // 8 and 16 bit integers only need to have natural alignment, but try to
  // align them to 32 bits. 64 bit integers have natural alignment.
  Ret += "-i8:8:32-i16:16:32-i64:64";

  // 32 bit registers are always available and the stack is at least 64 bit
  // aligned. On N64 64 bit registers are also available and the stack is
  // 128 bit aligned.
  if (ABI.IsN64() || ABI.IsN32())
    Ret += "-n32:64-S128";
  else
    Ret += "-n32-S64";

  return Ret;
}

static Reloc::Model getEffectiveRelocModel(bool JIT,
                                           Optional<Reloc::Model> RM) {
  if (!RM.hasValue() || JIT)
    return Reloc::Static;
  return *RM;
}

// On function prologue, the stack is created by decrementing
// its pointer. Once decremented, all references are done with positive
// offset from the stack/frame pointer, using StackGrowsUp enables
// an easier handling.
// Using CodeModel::Large enables different CALL behavior.
FgpuTargetMachine::FgpuTargetMachine(const Target &T, const Triple &TT,
                                     StringRef CPU, StringRef FS,
                                     const TargetOptions &Options,
                                     Optional<Reloc::Model> RM,
                                     Optional<CodeModel::Model> CM,
                                     CodeGenOpt::Level OL, bool JIT,
                                     bool isLittle)
    : LLVMTargetMachine(T, computeDataLayout(TT, CPU, Options, isLittle), TT,
                        CPU, FS, Options, getEffectiveRelocModel(JIT, RM),
                        getEffectiveCodeModel(CM, CodeModel::Small), OL),
      isLittle(isLittle), TLOF(std::make_unique<FgpuTargetObjectFile>()),
      ABI(FgpuABIInfo::computeTargetABI(TT, CPU, Options.MCOptions)),
      Subtarget(nullptr), DefaultSubtarget(TT, CPU, FS, isLittle, *this, None),
      NoFgpu16Subtarget(TT, CPU, FS.empty() ? "-Fgpu16" : FS.str() + ",-Fgpu16",
                        isLittle, *this, None),
      Fgpu16Subtarget(TT, CPU, FS.empty() ? "+Fgpu16" : FS.str() + ",+Fgpu16",
                      isLittle, *this, None) {
  Subtarget = &DefaultSubtarget;
  initAsmInfo();

  // Fgpu supports the debug entry values.
  setSupportsDebugEntryValues(true);
}

FgpuTargetMachine::~FgpuTargetMachine() = default;

void FgpuebTargetMachine::anchor() {}

FgpuebTargetMachine::FgpuebTargetMachine(const Target &T, const Triple &TT,
                                         StringRef CPU, StringRef FS,
                                         const TargetOptions &Options,
                                         Optional<Reloc::Model> RM,
                                         Optional<CodeModel::Model> CM,
                                         CodeGenOpt::Level OL, bool JIT)
    : FgpuTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, JIT, false) {}

void FgpuelTargetMachine::anchor() {}

FgpuelTargetMachine::FgpuelTargetMachine(const Target &T, const Triple &TT,
                                         StringRef CPU, StringRef FS,
                                         const TargetOptions &Options,
                                         Optional<Reloc::Model> RM,
                                         Optional<CodeModel::Model> CM,
                                         CodeGenOpt::Level OL, bool JIT)
    : FgpuTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, JIT, true) {}

const FgpuSubtarget *
FgpuTargetMachine::getSubtargetImpl(const Function &F) const {
  Attribute CPUAttr = F.getFnAttribute("target-cpu");
  Attribute FSAttr = F.getFnAttribute("target-features");

  std::string CPU =
      CPUAttr.isValid() ? CPUAttr.getValueAsString().str() : TargetCPU;
  std::string FS =
      FSAttr.isValid() ? FSAttr.getValueAsString().str() : TargetFS;
  bool hasFgpu16Attr = F.getFnAttribute("Fgpu16").isValid();
  bool hasNoFgpu16Attr = F.getFnAttribute("noFgpu16").isValid();

  bool HasMicroFgpuAttr = F.getFnAttribute("microFgpu").isValid();
  bool HasNoMicroFgpuAttr = F.getFnAttribute("nomicroFgpu").isValid();

  // FIXME: This is related to the code below to reset the target options,
  // we need to know whether or not the soft float flag is set on the
  // function, so we can enable it as a subtarget feature.
  bool softFloat = F.getFnAttribute("use-soft-float").getValueAsBool();

  if (hasFgpu16Attr)
    FS += FS.empty() ? "+Fgpu16" : ",+Fgpu16";
  else if (hasNoFgpu16Attr)
    FS += FS.empty() ? "-Fgpu16" : ",-Fgpu16";
  if (HasMicroFgpuAttr)
    FS += FS.empty() ? "+microFgpu" : ",+microFgpu";
  else if (HasNoMicroFgpuAttr)
    FS += FS.empty() ? "-microFgpu" : ",-microFgpu";
  if (softFloat)
    FS += FS.empty() ? "+soft-float" : ",+soft-float";

  auto &I = SubtargetMap[CPU + FS];
  if (!I) {
    // This needs to be done before we create a new subtarget since any
    // creation will depend on the TM and the code generation flags on the
    // function that reside in TargetOptions.
    resetTargetOptions(F);
    I = std::make_unique<FgpuSubtarget>(
        TargetTriple, CPU, FS, isLittle, *this,
        MaybeAlign(F.getParent()->getOverrideStackAlignment()));
  }
  return I.get();
}

void FgpuTargetMachine::resetSubtarget(MachineFunction *MF) {
  LLVM_DEBUG(dbgs() << "resetSubtarget\n");

  Subtarget = &MF->getSubtarget<FgpuSubtarget>();
}

namespace {

/// Fgpu Code Generator Pass Configuration Options.
class FgpuPassConfig : public TargetPassConfig {
public:
  FgpuPassConfig(FgpuTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {
    // The current implementation of long branch pass requires a scratch
    // register ($at) to be available before branch instructions. Tail merging
    // can break this requirement, so disable it when long branch pass is
    // enabled.
    EnableTailMerge = !getFgpuSubtarget().enableLongBranchPass();
  }

  FgpuTargetMachine &getFgpuTargetMachine() const {
    return getTM<FgpuTargetMachine>();
  }

  const FgpuSubtarget &getFgpuSubtarget() const {
    return *getFgpuTargetMachine().getSubtargetImpl();
  }

  void addIRPasses() override;
  bool addInstSelector() override;
  void addPreEmitPass() override;
  void addPreRegAlloc() override;
  bool addIRTranslator() override;
  void addPreLegalizeMachineIR() override;
  bool addLegalizeMachineIR() override;
  bool addRegBankSelect() override;
  bool addGlobalInstructionSelect() override;

  std::unique_ptr<CSEConfigBase> getCSEConfig() const override;
};

} // end anonymous namespace

TargetPassConfig *FgpuTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new FgpuPassConfig(*this, PM);
}

std::unique_ptr<CSEConfigBase> FgpuPassConfig::getCSEConfig() const {
  return getStandardCSEConfigForOpt(TM->getOptLevel());
}

void FgpuPassConfig::addIRPasses() {
  TargetPassConfig::addIRPasses();
  addPass(createAtomicExpandPass());
  if (getFgpuSubtarget().os16())
    addPass(createFgpuOs16Pass());
  if (getFgpuSubtarget().inFgpu16HardFloat())
    addPass(createFgpu16HardFloatPass());
}
// Install an instruction selector pass using
// the ISelDag to gen Fgpu code.
bool FgpuPassConfig::addInstSelector() {
  addPass(createFgpuModuleISelDagPass());
  addPass(createFgpu16ISelDag(getFgpuTargetMachine(), getOptLevel()));
  addPass(createFgpuSEISelDag(getFgpuTargetMachine(), getOptLevel()));
  return false;
}

void FgpuPassConfig::addPreRegAlloc() {
  addPass(createFgpuOptimizePICCallPass());
}

TargetTransformInfo
FgpuTargetMachine::getTargetTransformInfo(const Function &F) {
  if (Subtarget->allowMixed16_32()) {
    LLVM_DEBUG(errs() << "No Target Transform Info Pass Added\n");
    // FIXME: This is no longer necessary as the TTI returned is per-function.
    return TargetTransformInfo(F.getParent()->getDataLayout());
  }

  LLVM_DEBUG(errs() << "Target Transform Info Pass Added\n");
  return TargetTransformInfo(BasicTTIImpl(this, F));
}

// Implemented by targets that want to run passes immediately before
// machine code is emitted.
void FgpuPassConfig::addPreEmitPass() {
  // Expand pseudo instructions that are sensitive to register allocation.
  addPass(createFgpuExpandPseudoPass());

  // The microFgpu size reduction pass performs instruction reselection for
  // instructions which can be remapped to a 16 bit instruction.
  addPass(createMicroFgpuSizeReducePass());

  // The delay slot filler pass can potientially create forbidden slot hazards
  // for FgpuR6 and therefore it should go before FgpuBranchExpansion pass.
  addPass(createFgpuDelaySlotFillerPass());

  // This pass expands branches and takes care about the forbidden slot hazards.
  // Expanding branches may potentially create forbidden slot hazards for
  // FgpuR6, and fixing such hazard may potentially break a branch by extending
  // its offset out of range. That's why this pass combine these two tasks, and
  // runs them alternately until one of them finishes without any changes. Only
  // then we can be sure that all branches are expanded properly and no hazards
  // exists.
  // Any new pass should go before this pass.
  addPass(createFgpuBranchExpansion());

  addPass(createFgpuConstantIslandPass());
}

bool FgpuPassConfig::addIRTranslator() {
  addPass(new IRTranslator(getOptLevel()));
  return false;
}

void FgpuPassConfig::addPreLegalizeMachineIR() {
  addPass(createFgpuPreLegalizeCombiner());
}

bool FgpuPassConfig::addLegalizeMachineIR() {
  addPass(new Legalizer());
  return false;
}

bool FgpuPassConfig::addRegBankSelect() {
  addPass(new RegBankSelect());
  return false;
}

bool FgpuPassConfig::addGlobalInstructionSelect() {
  addPass(new InstructionSelect(getOptLevel()));
  return false;
}
