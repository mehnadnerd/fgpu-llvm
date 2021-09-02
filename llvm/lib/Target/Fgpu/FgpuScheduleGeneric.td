//===-- FgpuSchedule.td - Fgpu Scheduling Definitions ------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

def FgpuGenericModel : SchedMachineModel {
  int IssueWidth = 1;
  int MicroOpBufferSize = 0;

  // These figures assume an L1 hit.
  int LoadLatency = 2;
  int MispredictPenalty = 4;

  int HighLatency = 37;
  list<Predicate> UnsupportedFeatures = [];

  let CompleteModel = 1;
  let PostRAScheduler = 1;

  // FIXME: Remove when all errors have been fixed.
  let FullInstRWOverlapCheck = 1;
}