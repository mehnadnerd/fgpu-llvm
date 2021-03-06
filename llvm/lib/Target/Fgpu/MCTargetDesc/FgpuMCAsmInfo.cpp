//===-- FgpuMCAsmInfo.cpp - Fgpu Asm Properties ---------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the FgpuMCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "FgpuMCAsmInfo.h"
#include "FgpuABIInfo.h"
#include "llvm/ADT/Triple.h"

using namespace llvm;

void FgpuMCAsmInfo::anchor() { }

FgpuMCAsmInfo::FgpuMCAsmInfo(const Triple &TheTriple,
                             const MCTargetOptions &Options) {
  IsLittleEndian = TheTriple.isLittleEndian();

  FgpuABIInfo ABI = FgpuABIInfo::computeTargetABI(TheTriple, "", Options);

    PrivateGlobalPrefix = "$";
  PrivateLabelPrefix = PrivateGlobalPrefix;

  AlignmentIsInBytes          = false;
  Data16bitsDirective         = "\t.2byte\t";
  Data32bitsDirective         = "\t.4byte\t";
  Data64bitsDirective         = "\t.8byte\t";
  CommentString               = "#";
  ZeroDirective               = "\t.space\t";
  GPRel32Directive            = "\t.gpword\t";
  GPRel64Directive            = "\t.gpdword\t";
  DTPRel32Directive           = "\t.dtprelword\t";
  DTPRel64Directive           = "\t.dtpreldword\t";
  TPRel32Directive            = "\t.tprelword\t";
  TPRel64Directive            = "\t.tpreldword\t";
  UseAssignmentForEHBegin = true;
  SupportsDebugInformation = true;
  ExceptionsType = ExceptionHandling::DwarfCFI;
  DwarfRegNumForCFI = true;
  HasMipsExpressions = true;
}
