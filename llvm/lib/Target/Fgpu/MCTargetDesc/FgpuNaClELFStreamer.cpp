//===-- FgpuNaClELFStreamer.cpp - ELF Object Output for Fgpu NaCl ---------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements MCELFStreamer for Fgpu NaCl.  It emits .o object files
// as required by NaCl's SFI sandbox.  It inserts address-masking instructions
// before dangerous control-flow and memory access instructions.  It inserts
// address-masking instructions after instructions that change the stack
// pointer.  It ensures that the mask and the dangerous instruction are always
// emitted in the same bundle.  It aligns call + branch delay to the bundle end,
// so that return address is always aligned to the start of next bundle.
//
//===----------------------------------------------------------------------===//

#include "Fgpu.h"
#include "FgpuELFStreamer.h"
#include "FgpuMCNaCl.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/Support/ErrorHandling.h"
#include <cassert>

using namespace llvm;

#define DEBUG_TYPE "fgpu-mc-nacl"

namespace {

const unsigned IndirectBranchMaskReg = Fgpu::T6;
const unsigned LoadStoreStackMaskReg = Fgpu::T7;

/// Extend the generic MCELFStreamer class so that it can mask dangerous
/// instructions.

class FgpuNaClELFStreamer : public FgpuELFStreamer {
public:
  FgpuNaClELFStreamer(MCContext &Context, std::unique_ptr<MCAsmBackend> TAB,
                      std::unique_ptr<MCObjectWriter> OW,
                      std::unique_ptr<MCCodeEmitter> Emitter)
      : FgpuELFStreamer(Context, std::move(TAB), std::move(OW),
                        std::move(Emitter)) {}

  ~FgpuNaClELFStreamer() override = default;

private:
  // Whether we started the sandboxing sequence for calls.  Calls are bundled
  // with branch delays and aligned to the bundle end.
  bool PendingCall = false;

  bool isIndirectJump(const MCInst &MI) {
    if (MI.getOpcode() == Fgpu::JALR) {
      // FGPU32r6/FGPU64r6 doesn't have a JR instruction and uses JALR instead.
      // JALR is an indirect branch if the link register is $0.
      assert(MI.getOperand(0).isReg());
      return MI.getOperand(0).getReg() == Fgpu::ZERO;
    }
    return MI.getOpcode() == Fgpu::JR;
  }

  bool isStackPointerFirstOperand(const MCInst &MI) {
    return (MI.getNumOperands() > 0 && MI.getOperand(0).isReg()
            && MI.getOperand(0).getReg() == Fgpu::SP);
  }

  bool isCall(const MCInst &MI, bool *IsIndirectCall) {
    unsigned Opcode = MI.getOpcode();

    *IsIndirectCall = false;

    switch (Opcode) {
    default:
      return false;

    case Fgpu::JAL:
    case Fgpu::BAL:
    case Fgpu::BAL_BR:
    case Fgpu::BLTZAL:
    case Fgpu::BGEZAL:
      return true;

    case Fgpu::JALR:
      // JALR is only a call if the link register is not $0. Otherwise it's an
      // indirect branch.
      assert(MI.getOperand(0).isReg());
      if (MI.getOperand(0).getReg() == Fgpu::ZERO)
        return false;

      *IsIndirectCall = true;
      return true;
    }
  }

  void emitMask(unsigned AddrReg, unsigned MaskReg,
                const MCSubtargetInfo &STI) {
    MCInst MaskInst;
    MaskInst.setOpcode(Fgpu::AND);
    MaskInst.addOperand(MCOperand::createReg(AddrReg));
    MaskInst.addOperand(MCOperand::createReg(AddrReg));
    MaskInst.addOperand(MCOperand::createReg(MaskReg));
    FgpuELFStreamer::emitInstruction(MaskInst, STI);
  }

  // Sandbox indirect branch or return instruction by inserting mask operation
  // before it.
  void sandboxIndirectJump(const MCInst &MI, const MCSubtargetInfo &STI) {
    unsigned AddrReg = MI.getOperand(0).getReg();

    emitBundleLock(false);
    emitMask(AddrReg, IndirectBranchMaskReg, STI);
    FgpuELFStreamer::emitInstruction(MI, STI);
    emitBundleUnlock();
  }

  // Sandbox memory access or SP change.  Insert mask operation before and/or
  // after the instruction.
  void sandboxLoadStoreStackChange(const MCInst &MI, unsigned AddrIdx,
                                   const MCSubtargetInfo &STI, bool MaskBefore,
                                   bool MaskAfter) {
    emitBundleLock(false);
    if (MaskBefore) {
      // Sandbox memory access.
      unsigned BaseReg = MI.getOperand(AddrIdx).getReg();
      emitMask(BaseReg, LoadStoreStackMaskReg, STI);
    }
    FgpuELFStreamer::emitInstruction(MI, STI);
    if (MaskAfter) {
      // Sandbox SP change.
      unsigned SPReg = MI.getOperand(0).getReg();
      assert((Fgpu::SP == SPReg) && "Unexpected stack-pointer register.");
      emitMask(SPReg, LoadStoreStackMaskReg, STI);
    }
    emitBundleUnlock();
  }

public:
  /// This function is the one used to emit instruction data into the ELF
  /// streamer.  We override it to mask dangerous instructions.
  void emitInstruction(const MCInst &Inst,
                       const MCSubtargetInfo &STI) override {
    // Sandbox indirect jumps.
    if (isIndirectJump(Inst)) {
      if (PendingCall)
        report_fatal_error("Dangerous instruction in branch delay slot!");
      sandboxIndirectJump(Inst, STI);
      return;
    }

    // Sandbox loads, stores and SP changes.
    unsigned AddrIdx = 0;
    bool IsStore = false;
    bool IsMemAccess = isBasePlusOffsetMemoryAccess(Inst.getOpcode(), &AddrIdx,
                                                    &IsStore);
    bool IsSPFirstOperand = isStackPointerFirstOperand(Inst);
    if (IsMemAccess || IsSPFirstOperand) {
      bool MaskBefore = (IsMemAccess
                         && baseRegNeedsLoadStoreMask(Inst.getOperand(AddrIdx)
                                                          .getReg()));
      bool MaskAfter = IsSPFirstOperand && !IsStore;
      if (MaskBefore || MaskAfter) {
        if (PendingCall)
          report_fatal_error("Dangerous instruction in branch delay slot!");
        sandboxLoadStoreStackChange(Inst, AddrIdx, STI, MaskBefore, MaskAfter);
        return;
      }
      // fallthrough
    }

    // Sandbox calls by aligning call and branch delay to the bundle end.
    // For indirect calls, emit the mask before the call.
    bool IsIndirectCall;
    if (isCall(Inst, &IsIndirectCall)) {
      if (PendingCall)
        report_fatal_error("Dangerous instruction in branch delay slot!");

      // Start the sandboxing sequence by emitting call.
      emitBundleLock(true);
      if (IsIndirectCall) {
        unsigned TargetReg = Inst.getOperand(1).getReg();
        emitMask(TargetReg, IndirectBranchMaskReg, STI);
      }
      FgpuELFStreamer::emitInstruction(Inst, STI);
      PendingCall = true;
      return;
    }
    if (PendingCall) {
      // Finish the sandboxing sequence by emitting branch delay.
      FgpuELFStreamer::emitInstruction(Inst, STI);
      emitBundleUnlock();
      PendingCall = false;
      return;
    }

    // None of the sandboxing applies, just emit the instruction.
    FgpuELFStreamer::emitInstruction(Inst, STI);
  }
};

} // end anonymous namespace

namespace llvm {

bool isBasePlusOffsetMemoryAccess(unsigned Opcode, unsigned *AddrIdx,
                                  bool *IsStore) {
  if (IsStore)
    *IsStore = false;

  switch (Opcode) {
  default:
    return false;

  // Load instructions with base address register in position 1.
  case Fgpu::LB:
  case Fgpu::LBu:
  case Fgpu::LH:
  case Fgpu::LHu:
  case Fgpu::LW:
  case Fgpu::LWC1:
  case Fgpu::LDC1:
  case Fgpu::LL:
  case Fgpu::LL_R6:
  case Fgpu::LWL:
  case Fgpu::LWR:
    *AddrIdx = 1;
    return true;

  // Store instructions with base address register in position 1.
  case Fgpu::SB:
  case Fgpu::SH:
  case Fgpu::SW:
  case Fgpu::SWC1:
  case Fgpu::SDC1:
  case Fgpu::SWL:
  case Fgpu::SWR:
    *AddrIdx = 1;
    if (IsStore)
      *IsStore = true;
    return true;

  // Store instructions with base address register in position 2.
  case Fgpu::SC:
  case Fgpu::SC_R6:
    *AddrIdx = 2;
    if (IsStore)
      *IsStore = true;
    return true;
  }
}

bool baseRegNeedsLoadStoreMask(unsigned Reg) {
  // The contents of SP and thread pointer register do not require masking.
  return Reg != Fgpu::SP && Reg != Fgpu::T8;
}

MCELFStreamer *createFgpuNaClELFStreamer(MCContext &Context,
                                         std::unique_ptr<MCAsmBackend> TAB,
                                         std::unique_ptr<MCObjectWriter> OW,
                                         std::unique_ptr<MCCodeEmitter> Emitter,
                                         bool RelaxAll) {
  FgpuNaClELFStreamer *S = new FgpuNaClELFStreamer(
      Context, std::move(TAB), std::move(OW), std::move(Emitter));
  if (RelaxAll)
    S->getAssembler().setRelaxAll(true);

  // Set bundle-alignment as required by the NaCl ABI for the target.
  S->emitBundleAlignMode(Log2(FGPU_NACL_BUNDLE_ALIGN));

  return S;
}

} // end namespace llvm
