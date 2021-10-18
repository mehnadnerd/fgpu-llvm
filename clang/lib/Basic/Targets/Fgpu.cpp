//===--- Fgpu.cpp - Implement Fgpu target feature support -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements Fgpu TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#include "Fgpu.h"
#include "Targets.h"
#include "clang/Basic/Diagnostic.h"
#include "clang/Basic/MacroBuilder.h"
#include "clang/Basic/TargetBuiltins.h"
#include "llvm/ADT/StringSwitch.h"

using namespace clang;
using namespace clang::targets;

const Builtin::Info FgpuTargetInfo::BuiltinInfo[] = {
#define BUILTIN(ID, TYPE, ATTRS)                                               \
  {#ID, TYPE, ATTRS, nullptr, ALL_LANGUAGES, nullptr},
#define LIBBUILTIN(ID, TYPE, ATTRS, HEADER)                                    \
  {#ID, TYPE, ATTRS, HEADER, ALL_LANGUAGES, nullptr},
#include "clang/Basic/BuiltinsFgpu.def"
};

bool FgpuTargetInfo::processorSupportsGPR64() const {
  return llvm::StringSwitch<bool>(CPU)
      .Default(false);
}

static constexpr llvm::StringLiteral ValidCPUNames[] = {
    {"fgpu1"}};

bool FgpuTargetInfo::isValidCPUName(StringRef Name) const {
  return llvm::find(ValidCPUNames, Name) != std::end(ValidCPUNames);
}

void FgpuTargetInfo::fillValidCPUList(
    SmallVectorImpl<StringRef> &Values) const {
  Values.append(std::begin(ValidCPUNames), std::end(ValidCPUNames));
}

unsigned FgpuTargetInfo::getISARev() const {
  return llvm::StringSwitch<unsigned>(getCPU())
             .Cases("fgpu32", "fgpu64", 1)
             .Default(0);
}

void FgpuTargetInfo::getTargetDefines(const LangOptions &Opts,
                                      MacroBuilder &Builder) const {
  {
    DefineStd(Builder, "FGPUEL", Opts);
    Builder.defineMacro("_FGPUEL");
  }

  Builder.defineMacro("__fgpu__");
  Builder.defineMacro("_fgpu");
  if (Opts.GNUMode)
    Builder.defineMacro("fgpu");

  if (ABI == "o32") {
    Builder.defineMacro("__fgpu", "32");
    Builder.defineMacro("_FGPU_ISA", "_FGPU_ISA_FGPU32");
  } else {
    Builder.defineMacro("__fgpu", "64");
    Builder.defineMacro("__fgpu64");
    Builder.defineMacro("__fgpu64__");
    Builder.defineMacro("_FGPU_ISA", "_FGPU_ISA_FGPU64");
  }

  const std::string ISARev = std::to_string(getISARev());

  if (!ISARev.empty())
    Builder.defineMacro("__fgpu_isa_rev", ISARev);

  if (ABI == "o32") {
    Builder.defineMacro("__fgpu_o32");
    Builder.defineMacro("_ABIO32", "1");
    Builder.defineMacro("_FGPU_SIM", "_ABIO32");
  }

  if (!IsNoABICalls) {
    Builder.defineMacro("__fgpu_abicalls");
    if (CanUseBSDABICalls)
      Builder.defineMacro("__ABICALLS__");
  }

  Builder.defineMacro("__REGISTER_PREFIX__", "");

    Builder.defineMacro("__fgpu_hard_float", Twine(1));

  if (IsSingleFloat)
    Builder.defineMacro("__fgpu_single_float", Twine(1));

  switch (FPMode) {
  case FPXX:
    Builder.defineMacro("__fgpu_fpr", Twine(0));
    break;
  case FP32:
    Builder.defineMacro("__fgpu_fpr", Twine(32));
    break;
  case FP64:
    Builder.defineMacro("__fgpu_fpr", Twine(64));
    break;
}

  if (FPMode == FP64 || IsSingleFloat)
    Builder.defineMacro("_FGPU_FPSET", Twine(32));
  else
    Builder.defineMacro("_FGPU_FPSET", Twine(16));


  Builder.defineMacro("_FGPU_SZPTR", Twine(getPointerWidth(0)));
  Builder.defineMacro("_FGPU_SZINT", Twine(getIntWidth()));
  Builder.defineMacro("_FGPU_SZLONG", Twine(getLongWidth()));

  Builder.defineMacro("_FGPU_ARCH", "\"" + CPU + "\"");
  if (CPU == "octeon+")
    Builder.defineMacro("_FGPU_ARCH_OCTEONP");
  else
    Builder.defineMacro("_FGPU_ARCH_" + StringRef(CPU).upper());
}

bool FgpuTargetInfo::hasFeature(StringRef Feature) const {
  return llvm::StringSwitch<bool>(Feature)
      .Case("fgpu", true)
      .Default(false);
}

ArrayRef<Builtin::Info> FgpuTargetInfo::getTargetBuiltins() const {
  return llvm::makeArrayRef(BuiltinInfo, clang::Fgpu::LastTSBuiltin -
                                             Builtin::FirstTSBuiltin);
}

unsigned FgpuTargetInfo::getUnwindWordWidth() const {
  return llvm::StringSwitch<unsigned>(ABI)
      .Case("o32", 32)
      .Default(getPointerWidth(0));
}

bool FgpuTargetInfo::validateTarget(DiagnosticsEngine &Diags) const {

  // FIXME: It's valid to use N32/N64 on a fgpu/fgpuel triple but the backend
  //        can't handle this yet. It's better to fail here than on the
  //        backend assertion.
  if (getTriple().isFGPU32() && (ABI == "n32" || ABI == "n64")) {
    Diags.Report(diag::err_target_unsupported_abi_for_triple)
        << ABI << getTriple().str();
    return false;
  }

  // -fpxx is valid only for the o32 ABI
  if (FPMode == FPXX && (ABI == "n32" || ABI == "n64")) {
    Diags.Report(diag::err_unsupported_abi_for_opt) << "-mfpxx" << "o32";
    return false;
  }


  return true;
}
