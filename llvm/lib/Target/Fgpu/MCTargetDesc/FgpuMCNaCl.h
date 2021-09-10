//===-- FgpuMCNaCl.h - NaCl-related declarations --------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUMCNACL_H
#define LLVM_LIB_TARGET_FGPU_MCTARGETDESC_FGPUMCNACL_H

#include "llvm/MC/MCELFStreamer.h"
#include "llvm/Support/Alignment.h"

namespace llvm {

// NaCl FGPU sandbox's instruction bundle size.
static const Align FGPU_NACL_BUNDLE_ALIGN = Align(16);

bool isBasePlusOffsetMemoryAccess(unsigned Opcode, unsigned *AddrIdx,
                                  bool *IsStore = nullptr);
bool baseRegNeedsLoadStoreMask(unsigned Reg);

// This function creates an MCELFStreamer for Fgpu NaCl.
MCELFStreamer *createFgpuNaClELFStreamer(MCContext &Context,
                                         std::unique_ptr<MCAsmBackend> TAB,
                                         std::unique_ptr<MCObjectWriter> OW,
                                         std::unique_ptr<MCCodeEmitter> Emitter,
                                         bool RelaxAll);
}

#endif
