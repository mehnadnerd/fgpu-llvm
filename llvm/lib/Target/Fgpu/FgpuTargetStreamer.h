//===-- FgpuTargetStreamer.h - Fgpu Target Streamer ------------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Fgpu_FgpuTARGETSTREAMER_H
#define LLVM_LIB_TARGET_Fgpu_FgpuTARGETSTREAMER_H

#include "MCTargetDesc/FgpuABIInfo.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"

namespace llvm {

class formatted_raw_ostream;

class FgpuTargetStreamer : public MCTargetStreamer {
public:
  FgpuTargetStreamer(MCStreamer &S);

  virtual void setPic(bool Value) {}

  void emitR(unsigned Opcode, unsigned Reg0, SMLoc IDLoc,
             const MCSubtargetInfo *STI);
  void emitII(unsigned Opcode, int16_t Imm1, int16_t Imm2, SMLoc IDLoc,
              const MCSubtargetInfo *STI);
  void emitRX(unsigned Opcode, unsigned Reg0, MCOperand Op1, SMLoc IDLoc,
              const MCSubtargetInfo *STI);
  void emitRI(unsigned Opcode, unsigned Reg0, int32_t Imm, SMLoc IDLoc,
              const MCSubtargetInfo *STI);
  void emitRR(unsigned Opcode, unsigned Reg0, unsigned Reg1, SMLoc IDLoc,
              const MCSubtargetInfo *STI);
  void emitRRX(unsigned Opcode, unsigned Reg0, unsigned Reg1, MCOperand Op2,
               SMLoc IDLoc, const MCSubtargetInfo *STI);
  void emitRRR(unsigned Opcode, unsigned Reg0, unsigned Reg1, unsigned Reg2,
               SMLoc IDLoc, const MCSubtargetInfo *STI);
  void emitRRRX(unsigned Opcode, unsigned Reg0, unsigned Reg1, unsigned Reg2,
                MCOperand Op3, SMLoc IDLoc, const MCSubtargetInfo *STI);
  void emitRRI(unsigned Opcode, unsigned Reg0, unsigned Reg1, int16_t Imm,
               SMLoc IDLoc, const MCSubtargetInfo *STI);
  void emitRRIII(unsigned Opcode, unsigned Reg0, unsigned Reg1, int16_t Imm0,
                 int16_t Imm1, int16_t Imm2, SMLoc IDLoc,
                 const MCSubtargetInfo *STI);
  void emitAddu(unsigned DstReg, unsigned SrcReg, unsigned TrgReg, bool Is64Bit,
                const MCSubtargetInfo *STI);
  void emitDSLL(unsigned DstReg, unsigned SrcReg, int16_t ShiftAmount,
                SMLoc IDLoc, const MCSubtargetInfo *STI);
  void emitEmptyDelaySlot(bool hasShortDelaySlot, SMLoc IDLoc,
                          const MCSubtargetInfo *STI);
  void emitNop(SMLoc IDLoc, const MCSubtargetInfo *STI);
  void emitMask(unsigned CPUBitmask, int CPUTopSavedRegOff) override;
  void emitFMask(unsigned FPUBitmask, int FPUTopSavedRegOff) override;

  /// Emit a store instruction with an offset. If the offset is out of range
  /// then it will be synthesized using the assembler temporary.
  ///
  /// GetATReg() is a callback that can be used to obtain the current assembler
  /// temporary and is only called when the assembler temporary is required. It
  /// must handle the case where no assembler temporary is available (typically
  /// by reporting an error).
  void emitStoreWithImmOffset(unsigned Opcode, unsigned SrcReg,
                              unsigned BaseReg, int64_t Offset,
                              function_ref<unsigned()> GetATReg, SMLoc IDLoc,
                              const MCSubtargetInfo *STI);
  void emitLoadWithImmOffset(unsigned Opcode, unsigned DstReg, unsigned BaseReg,
                             int64_t Offset, unsigned TmpReg, SMLoc IDLoc,
                             const MCSubtargetInfo *STI);
  void emitGPRestore(int Offset, SMLoc IDLoc, const MCSubtargetInfo *STI);

  void forbidModuleDirective() { ModuleDirectiveAllowed = false; }
  void reallowModuleDirective() { ModuleDirectiveAllowed = true; }
  bool isModuleDirectiveAllowed() { return ModuleDirectiveAllowed; }

  // This method enables template classes to set internal abi flags
  // structure values.
  template <class PredicateLibrary>
  void updateABIInfo(const PredicateLibrary &P) {
    ABI = P.getABI();
  }

  const FgpuABIInfo &getABI() const {
    assert(ABI.hasValue() && "ABI hasn't been set!");
    return *ABI;
  }

protected:
  llvm::Optional<FgpuABIInfo> ABI;

  bool GPRInfoSet;
  unsigned GPRBitMask;
  int GPROffset;

  bool FPRInfoSet;
  unsigned FPRBitMask;
  int FPROffset;

  bool FrameInfoSet;
  int FrameOffset;
  unsigned FrameReg;
  unsigned GPReg;
  unsigned ReturnReg;

private:
  bool ModuleDirectiveAllowed;
};

// This part is for ascii assembly output
class FgpuTargetAsmStreamer : public FgpuTargetStreamer {
  formatted_raw_ostream &OS;

public:
  FgpuTargetAsmStreamer(MCStreamer &S, formatted_raw_ostream &OS);
  void emitMask(unsigned CPUBitmask, int CPUTopSavedRegOff) override;
  void emitFMask(unsigned FPUBitmask, int FPUTopSavedRegOff) override;
};

// This part is for ELF object output
class FgpuTargetELFStreamer : public FgpuTargetStreamer {
  const MCSubtargetInfo &STI;
  bool Pic;

public:
  MCELFStreamer &getStreamer();
  FgpuTargetELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI);

  void setPic(bool Value) override { Pic = Value; }

  void emitLabel(MCSymbol *Symbol) override;
  void emitAssignment(MCSymbol *Symbol, const MCExpr *Value) override;
  void finish() override;

  void emitFrame(unsigned StackReg, unsigned StackSize,
                 unsigned ReturnReg) override;
  void emitMask(unsigned CPUBitmask, int CPUTopSavedRegOff) override;
  void emitFMask(unsigned FPUBitmask, int FPUTopSavedRegOff) override;

};
} // namespace llvm
#endif
