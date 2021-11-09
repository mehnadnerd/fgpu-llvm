//===- FgpuTargetMachine.h - Define TargetMachine for Fgpu ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the Fgpu specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_FGPU_FGPUTARGETMACHINE_H
#define LLVM_LIB_TARGET_FGPU_FGPUTARGETMACHINE_H

#include "MCTargetDesc/FgpuABIInfo.h"
#include "FgpuSubtarget.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Target/TargetMachine.h"
#include <memory>

namespace llvm {

class FgpuTargetMachine : public LLVMTargetMachine {
  bool isLittle;
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  // Selected ABI
  FgpuABIInfo ABI;
  const FgpuSubtarget *Subtarget;
  FgpuSubtarget DefaultSubtarget;

  mutable StringMap<std::unique_ptr<FgpuSubtarget>> SubtargetMap;

public:
  FgpuTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                    StringRef FS, const TargetOptions &Options,
                    Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                    CodeGenOpt::Level OL, bool JIT, bool isLittle);
  ~FgpuTargetMachine() override;

  TargetTransformInfo getTargetTransformInfo(const Function &F) override;

  const FgpuSubtarget *getSubtargetImpl() const {
    if (Subtarget)
      return Subtarget;
    return &DefaultSubtarget;
  }

  const FgpuSubtarget *getSubtargetImpl(const Function &F) const override;

  /// Reset the subtarget for the Fgpu target.
  void resetSubtarget(MachineFunction *MF);

  // Pass Pipeline Configuration
  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;
  void adjustPassManager(PassManagerBuilder &PMB) override;
  void registerPassBuilderCallbacks(PassBuilder &PB) override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
    return TLOF.get();
  }

  /// Returns true if a cast between SrcAS and DestAS is a noop.
  bool isNoopAddrSpaceCast(unsigned SrcAS, unsigned DestAS) const override {
    // Fgpu doesn't have any special address spaces so we just reserve
    // the first 256 for software use (e.g. OpenCL) and treat casts
    // between them as noops.
    return SrcAS < 256 && DestAS < 256;
  }

  bool isLittleEndian() const { return isLittle; }
  const FgpuABIInfo &getABI() const { return ABI; }
};

/// Fgpu32/64 big endian target machine.
///
class FgpuebTargetMachine : public FgpuTargetMachine {
  virtual void anchor();

public:
  FgpuebTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                      StringRef FS, const TargetOptions &Options,
                      Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                      CodeGenOpt::Level OL, bool JIT);
};

/// Fgpu32/64 little endian target machine.
///
class FgpuelTargetMachine : public FgpuTargetMachine {
  virtual void anchor();

public:
  FgpuelTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                      StringRef FS, const TargetOptions &Options,
                      Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                      CodeGenOpt::Level OL, bool JIT);
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_FGPU_FGPUTARGETMACHINE_H
