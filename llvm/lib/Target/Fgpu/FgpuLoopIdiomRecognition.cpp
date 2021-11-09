//===- FgpuLoopIdiomRecognition.cpp ------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "FgpuLoopIdiomRecognition.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SetVector.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/Triple.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/InstructionSimplify.h"
#include "llvm/Analysis/LoopAnalysisManager.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/LoopPass.h"
#include "llvm/Analysis/MemoryLocation.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Analysis/TargetLibraryInfo.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Constant.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/IntrinsicInst.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/IR/IntrinsicsFgpu.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/PassManager.h"
#include "llvm/IR/PatternMatch.h"
#include "llvm/IR/Type.h"
#include "llvm/IR/User.h"
#include "llvm/IR/Value.h"
#include "llvm/InitializePasses.h"
#include "llvm/Pass.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/KnownBits.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Utils.h"
#include "llvm/Transforms/Utils/Local.h"
#include "llvm/Transforms/Utils/ScalarEvolutionExpander.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iterator>
#include <map>
#include <set>
#include <utility>
#include <vector>

#define DEBUG_TYPE "fgpu-lir"

using namespace llvm;

static cl::opt<bool>
    DisableLoopIdiom("fgpu-disable-loop-idiom", cl::Hidden, cl::init(false),
                     cl::desc("Disable fgpu in loop idiom recognition"));

namespace llvm {

void initializeFgpuLoopIdiomRecognizeLegacyPassPass(PassRegistry &);
Pass *createFgpuLoopIdiomPass();

} // end namespace llvm

namespace {

class FgpuLoopIdiomRecognize {
public:
  explicit FgpuLoopIdiomRecognize(AliasAnalysis *AA, DominatorTree *DT,
                                  LoopInfo *LF, const TargetLibraryInfo *TLI,
                                  ScalarEvolution *SE)
      : AA(AA), DT(DT), LF(LF), TLI(TLI), SE(SE) {}

  bool run(Loop *L);

private:
  int getSCEVStride(const SCEVAddRecExpr *StoreEv);
  bool recogniseDot(Loop *L, int beCount, bool canChange);
  bool isLegalLoad(Loop *CurLoop, LoadInst *LI);
  void collectLoads(Loop *CurLoop, BasicBlock *BB,
                    SmallVectorImpl<LoadInst *> &Loads);
  bool processCopyingStore(Loop *CurLoop, StoreInst *SI, const SCEV *BECount);
  bool coverLoop(Loop *L, SmallVectorImpl<Instruction *> &Insts) const;
  bool runOnCountableLoop(Loop *L);

  AliasAnalysis *AA;
  const DataLayout *DL;
  DominatorTree *DT;
  LoopInfo *LF;
  const TargetLibraryInfo *TLI;
  ScalarEvolution *SE;
  bool HasMemcpy, HasMemmove;
};

class FgpuLoopIdiomRecognizeLegacyPass : public LoopPass {
public:
  static char ID;

  explicit FgpuLoopIdiomRecognizeLegacyPass() : LoopPass(ID) {
    initializeFgpuLoopIdiomRecognizeLegacyPassPass(
        *PassRegistry::getPassRegistry());
  }

  StringRef getPassName() const override {
    return "Recognize Fgpu-specific loop idioms";
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<LoopInfoWrapperPass>();
    AU.addRequiredID(LoopSimplifyID);
    AU.addRequiredID(LCSSAID);
    AU.addRequired<AAResultsWrapperPass>();
    AU.addRequired<ScalarEvolutionWrapperPass>();
    AU.addRequired<DominatorTreeWrapperPass>();
    AU.addRequired<TargetLibraryInfoWrapperPass>();
    AU.addPreserved<TargetLibraryInfoWrapperPass>();
  }

  bool runOnLoop(Loop *L, LPPassManager &LPM) override;
};

struct Simplifier {
  struct Rule {
    using FuncType = std::function<Value *(Instruction *, LLVMContext &)>;
    Rule(StringRef N, FuncType F) : Name(N), Fn(F) {}
    StringRef Name; // For debugging.
    FuncType Fn;
  };

  void addRule(StringRef N, const Rule::FuncType &F) {
    Rules.push_back(Rule(N, F));
  }

private:
  struct WorkListType {
    WorkListType() = default;

    void push_back(Value *V) {
      // Do not push back duplicates.
      if (!S.count(V)) {
        Q.push_back(V);
        S.insert(V);
      }
    }

    Value *pop_front_val() {
      Value *V = Q.front();
      Q.pop_front();
      S.erase(V);
      return V;
    }

    bool empty() const { return Q.empty(); }

  private:
    std::deque<Value *> Q;
    std::set<Value *> S;
  };

  using ValueSetType = std::set<Value *>;

  std::vector<Rule> Rules;

public:
  struct Context {
    using ValueMapType = DenseMap<Value *, Value *>;

    Value *Root;
    ValueSetType Used;   // The set of all cloned values used by Root.
    ValueSetType Clones; // The set of all cloned values.
    LLVMContext &Ctx;

    Context(Instruction *Exp)
        : Ctx(Exp->getParent()->getParent()->getContext()) {
      initialize(Exp);
    }

    ~Context() { cleanup(); }

    void print(raw_ostream &OS, const Value *V) const;
    Value *materialize(BasicBlock *B, BasicBlock::iterator At);

  private:
    friend struct Simplifier;

    void initialize(Instruction *Exp);
    void cleanup();

    template <typename FuncT> void traverse(Value *V, FuncT F);
    void record(Value *V);
    void use(Value *V);
    void unuse(Value *V);

    bool equal(const Instruction *I, const Instruction *J) const;
    Value *find(Value *Tree, Value *Sub) const;
    Value *subst(Value *Tree, Value *OldV, Value *NewV);
    void replace(Value *OldV, Value *NewV);
    void link(Instruction *I, BasicBlock *B, BasicBlock::iterator At);
  };

  Value *simplify(Context &C);
};

struct PE {
  PE(const Simplifier::Context &c, Value *v = nullptr) : C(c), V(v) {}

  const Simplifier::Context &C;
  const Value *V;
};

LLVM_ATTRIBUTE_USED
raw_ostream &operator<<(raw_ostream &OS, const PE &P) {
  P.C.print(OS, P.V ? P.V : P.C.Root);
  return OS;
}

} // end anonymous namespace

char FgpuLoopIdiomRecognizeLegacyPass::ID = 0;

INITIALIZE_PASS_BEGIN(FgpuLoopIdiomRecognizeLegacyPass, "fgpu-loop-idiom",
                      "Recognize Fgpu-specific loop idioms", false, false)
INITIALIZE_PASS_DEPENDENCY(LoopInfoWrapperPass)
INITIALIZE_PASS_DEPENDENCY(LoopSimplify)
INITIALIZE_PASS_DEPENDENCY(LCSSAWrapperPass)
INITIALIZE_PASS_DEPENDENCY(ScalarEvolutionWrapperPass)
INITIALIZE_PASS_DEPENDENCY(DominatorTreeWrapperPass)
INITIALIZE_PASS_DEPENDENCY(TargetLibraryInfoWrapperPass)
INITIALIZE_PASS_DEPENDENCY(AAResultsWrapperPass)
INITIALIZE_PASS_END(FgpuLoopIdiomRecognizeLegacyPass, "fgpu-loop-idiom",
                    "Recognize Fgpu-specific loop idioms", false, false)

template <typename FuncT>
void Simplifier::Context::traverse(Value *V, FuncT F) {
  WorkListType Q;
  Q.push_back(V);

  while (!Q.empty()) {
    Instruction *U = dyn_cast<Instruction>(Q.pop_front_val());
    if (!U || U->getParent())
      continue;
    if (!F(U))
      continue;
    for (Value *Op : U->operands())
      Q.push_back(Op);
  }
}

void Simplifier::Context::print(raw_ostream &OS, const Value *V) const {
  const auto *U = dyn_cast<const Instruction>(V);
  if (!U) {
    OS << V << '(' << *V << ')';
    return;
  }

  if (U->getParent()) {
    OS << U << '(';
    U->printAsOperand(OS, true);
    OS << ')';
    return;
  }

  unsigned N = U->getNumOperands();
  if (N != 0)
    OS << U << '(';
  OS << U->getOpcodeName();
  for (const Value *Op : U->operands()) {
    OS << ' ';
    print(OS, Op);
  }
  if (N != 0)
    OS << ')';
}

void Simplifier::Context::initialize(Instruction *Exp) {
  // Perform a deep clone of the expression, set Root to the root
  // of the clone, and build a map from the cloned values to the
  // original ones.
  ValueMapType M;
  BasicBlock *Block = Exp->getParent();
  WorkListType Q;
  Q.push_back(Exp);

  while (!Q.empty()) {
    Value *V = Q.pop_front_val();
    if (M.find(V) != M.end())
      continue;
    if (Instruction *U = dyn_cast<Instruction>(V)) {
      if (isa<PHINode>(U) || U->getParent() != Block)
        continue;
      for (Value *Op : U->operands())
        Q.push_back(Op);
      M.insert({U, U->clone()});
    }
  }

  for (std::pair<Value *, Value *> P : M) {
    Instruction *U = cast<Instruction>(P.second);
    for (unsigned i = 0, n = U->getNumOperands(); i != n; ++i) {
      auto F = M.find(U->getOperand(i));
      if (F != M.end())
        U->setOperand(i, F->second);
    }
  }

  auto R = M.find(Exp);
  assert(R != M.end());
  Root = R->second;

  record(Root);
  use(Root);
}

void Simplifier::Context::record(Value *V) {
  auto Record = [this](Instruction *U) -> bool {
    Clones.insert(U);
    return true;
  };
  traverse(V, Record);
}

void Simplifier::Context::use(Value *V) {
  auto Use = [this](Instruction *U) -> bool {
    Used.insert(U);
    return true;
  };
  traverse(V, Use);
}

void Simplifier::Context::unuse(Value *V) {
  if (!isa<Instruction>(V) || cast<Instruction>(V)->getParent() != nullptr)
    return;

  auto Unuse = [this](Instruction *U) -> bool {
    if (!U->use_empty())
      return false;
    Used.erase(U);
    return true;
  };
  traverse(V, Unuse);
}

Value *Simplifier::Context::subst(Value *Tree, Value *OldV, Value *NewV) {
  if (Tree == OldV)
    return NewV;
  if (OldV == NewV)
    return Tree;

  WorkListType Q;
  Q.push_back(Tree);
  while (!Q.empty()) {
    Instruction *U = dyn_cast<Instruction>(Q.pop_front_val());
    // If U is not an instruction, or it's not a clone, skip it.
    if (!U || U->getParent())
      continue;
    for (unsigned i = 0, n = U->getNumOperands(); i != n; ++i) {
      Value *Op = U->getOperand(i);
      if (Op == OldV) {
        U->setOperand(i, NewV);
        unuse(OldV);
      } else {
        Q.push_back(Op);
      }
    }
  }
  return Tree;
}

void Simplifier::Context::replace(Value *OldV, Value *NewV) {
  if (Root == OldV) {
    Root = NewV;
    use(Root);
    return;
  }

  // NewV may be a complex tree that has just been created by one of the
  // transformation rules. We need to make sure that it is commoned with
  // the existing Root to the maximum extent possible.
  // Identify all subtrees of NewV (including NewV itself) that have
  // equivalent counterparts in Root, and replace those subtrees with
  // these counterparts.
  WorkListType Q;
  Q.push_back(NewV);
  while (!Q.empty()) {
    Value *V = Q.pop_front_val();
    Instruction *U = dyn_cast<Instruction>(V);
    if (!U || U->getParent())
      continue;
    if (Value *DupV = find(Root, V)) {
      if (DupV != V)
        NewV = subst(NewV, V, DupV);
    } else {
      for (Value *Op : U->operands())
        Q.push_back(Op);
    }
  }

  // Now, simply replace OldV with NewV in Root.
  Root = subst(Root, OldV, NewV);
  use(Root);
}

void Simplifier::Context::cleanup() {
  for (Value *V : Clones) {
    Instruction *U = cast<Instruction>(V);
    if (!U->getParent())
      U->dropAllReferences();
  }

  for (Value *V : Clones) {
    Instruction *U = cast<Instruction>(V);
    if (!U->getParent())
      U->deleteValue();
  }
}

bool Simplifier::Context::equal(const Instruction *I,
                                const Instruction *J) const {
  if (I == J)
    return true;
  if (!I->isSameOperationAs(J))
    return false;
  if (isa<PHINode>(I))
    return I->isIdenticalTo(J);

  for (unsigned i = 0, n = I->getNumOperands(); i != n; ++i) {
    Value *OpI = I->getOperand(i), *OpJ = J->getOperand(i);
    if (OpI == OpJ)
      continue;
    auto *InI = dyn_cast<const Instruction>(OpI);
    auto *InJ = dyn_cast<const Instruction>(OpJ);
    if (InI && InJ) {
      if (!equal(InI, InJ))
        return false;
    } else if (InI != InJ || !InI)
      return false;
  }
  return true;
}

Value *Simplifier::Context::find(Value *Tree, Value *Sub) const {
  Instruction *SubI = dyn_cast<Instruction>(Sub);
  WorkListType Q;
  Q.push_back(Tree);

  while (!Q.empty()) {
    Value *V = Q.pop_front_val();
    if (V == Sub)
      return V;
    Instruction *U = dyn_cast<Instruction>(V);
    if (!U || U->getParent())
      continue;
    if (SubI && equal(SubI, U))
      return U;
    assert(!isa<PHINode>(U));
    for (Value *Op : U->operands())
      Q.push_back(Op);
  }
  return nullptr;
}

void Simplifier::Context::link(Instruction *I, BasicBlock *B,
                               BasicBlock::iterator At) {
  if (I->getParent())
    return;

  for (Value *Op : I->operands()) {
    if (Instruction *OpI = dyn_cast<Instruction>(Op))
      link(OpI, B, At);
  }

  B->getInstList().insert(At, I);
}

Value *Simplifier::Context::materialize(BasicBlock *B,
                                        BasicBlock::iterator At) {
  if (Instruction *RootI = dyn_cast<Instruction>(Root))
    link(RootI, B, At);
  return Root;
}

Value *Simplifier::simplify(Context &C) {
  WorkListType Q;
  Q.push_back(C.Root);
  unsigned Count = 0;
  const unsigned Limit = 1000;

  while (!Q.empty()) {
    if (Count++ >= Limit)
      break;
    Instruction *U = dyn_cast<Instruction>(Q.pop_front_val());
    if (!U || U->getParent() || !C.Used.count(U))
      continue;
    bool Changed = false;
    for (Rule &R : Rules) {
      Value *W = R.Fn(U, C.Ctx);
      if (!W)
        continue;
      Changed = true;
      C.record(W);
      C.replace(U, W);
      Q.push_back(C.Root);
      break;
    }
    if (!Changed) {
      for (Value *Op : U->operands())
        Q.push_back(Op);
    }
  }
  return Count < Limit ? C.Root : nullptr;
}

static CallInst *createDotIntrinsic(IRBuilder<> &IRBuilder, ArrayRef<Value*> Args,
                                       const DebugLoc &DL) {
  ArrayRef<Value*> Ops = Args;

  Module *M = IRBuilder.GetInsertBlock()->getParent()->getParent();
  Function *Func = Intrinsic::getDeclaration(M, Intrinsic::fgpu_dot);
  CallInst *CI = IRBuilder.CreateCall(Func, Ops);
  CI->setDebugLoc(DL);

  return CI;
}

int FgpuLoopIdiomRecognize::getSCEVStride(const SCEVAddRecExpr *S) {
  if (const SCEVConstant *SC = dyn_cast<SCEVConstant>(S->getOperand(1)))
    return SC->getAPInt().getSExtValue();
  return 0;
}

bool FgpuLoopIdiomRecognize::isLegalLoad(Loop *CurLoop, LoadInst *LI) {
  if (!LI->isSimple())
    return false;
  Value *LoadPtr = LI->getPointerOperand();

  // See if the pointer expression is an AddRec like {base,+,1} on the current
  // loop, which indicates a strided store.  If we have something else, it's a
  // random store we can't handle.
  auto *LoadEv = dyn_cast<SCEVAddRecExpr>(SE->getSCEV(LoadPtr));
  if (!LoadEv || LoadEv->getLoop() != CurLoop || !LoadEv->isAffine())
    return false;

  // Check to see if the stride matches the size of the store.  If so, then we
  // know that every byte is touched in the loop.
  int Stride = getSCEVStride(LoadEv);
  if (Stride == 0 || Stride < 0) // TODO: is it worth to support negative stride?
    return false;
  unsigned LoadSize = DL->getTypeStoreSize(LI->getType());
  if (LoadSize != unsigned(std::abs(Stride)))
    return false;

  // Success.  This store can be converted into a memcpy.
  return true;
}

/// mayLoopAccessLocation - Return true if the specified loop might access the
/// specified pointer location, which is a loop-strided access.  The 'Access'
/// argument specifies what the verboten forms of access are (read or write).
static bool mayLoopAccessLocation(Value *Ptr, ModRefInfo Access, Loop *L,
                                  const SCEV *BECount, unsigned StoreSize,
                                  AliasAnalysis &AA,
                                  SmallPtrSetImpl<Instruction *> &Ignored) {
  // Get the location that may be stored across the loop.  Since the access
  // is strided positively through memory, we say that the modified location
  // starts at the pointer and has infinite size.
  LocationSize AccessSize = LocationSize::afterPointer();

  // If the loop iterates a fixed number of times, we can refine the access
  // size to be exactly the size of the memset, which is (BECount+1)*StoreSize
  if (const SCEVConstant *BECst = dyn_cast<SCEVConstant>(BECount))
    AccessSize = LocationSize::precise((BECst->getValue()->getZExtValue() + 1) *
                                       StoreSize);

  // TODO: For this to be really effective, we have to dive into the pointer
  // operand in the store.  Store to &A[i] of 100 will always return may alias
  // with store of &A[100], we need to StoreLoc to be "A" with size of 100,
  // which will then no-alias a store to &A[100].
  MemoryLocation StoreLoc(Ptr, AccessSize);

  for (auto *B : L->blocks())
    for (auto &I : *B)
      if (Ignored.count(&I) == 0 &&
          isModOrRefSet(
              intersectModRef(AA.getModRefInfo(&I, StoreLoc), Access)))
        return true;

  return false;
}

void FgpuLoopIdiomRecognize::collectLoads(Loop *CurLoop, BasicBlock *BB,
                                          SmallVectorImpl<LoadInst *> &Loads) {
  Loads.clear();
  for (Instruction &I : *BB)
    if (LoadInst *LI = dyn_cast<LoadInst>(&I))
      if (isLegalLoad(CurLoop, LI))
        Loads.push_back(LI);
}

bool FgpuLoopIdiomRecognize::processCopyingStore(Loop *CurLoop, StoreInst *SI,
                                                 const SCEV *BECount) {
  assert((SI->isSimple() || (SI->isVolatile() && false)) &&
         "Expected only non-volatile stores, or Fgpu-specific memcpy"
         "to volatile destination.");

  Value *StorePtr = SI->getPointerOperand();
  auto *StoreEv = cast<SCEVAddRecExpr>(SE->getSCEV(StorePtr));
  unsigned Stride = getSCEVStride(StoreEv);
  unsigned StoreSize = DL->getTypeStoreSize(SI->getValueOperand()->getType());
  if (Stride != StoreSize)
    return false;

  // See if the pointer expression is an AddRec like {base,+,1} on the current
  // loop, which indicates a strided load.  If we have something else, it's a
  // random load we can't handle.
  auto *LI = cast<LoadInst>(SI->getValueOperand());
  auto *LoadEv = cast<SCEVAddRecExpr>(SE->getSCEV(LI->getPointerOperand()));

  // The trip count of the loop and the base pointer of the addrec SCEV is
  // guaranteed to be loop invariant, which means that it should dominate the
  // header.  This allows us to insert code for it in the preheader.
  BasicBlock *Preheader = CurLoop->getLoopPreheader();
  Instruction *ExpPt = Preheader->getTerminator();
  IRBuilder<> Builder(ExpPt);
  SCEVExpander Expander(*SE, *DL, "fgpu-loop-idiom");

  Type *IntPtrTy = Builder.getIntPtrTy(*DL, SI->getPointerAddressSpace());

  // Okay, we have a strided store "p[i]" of a loaded value.  We can turn
  // this into a memcpy/memmove in the loop preheader now if we want.  However,
  // this would be unsafe to do if there is anything else in the loop that may
  // read or write the memory region we're storing to.  For memcpy, this
  // includes the load that feeds the stores.  Check for an alias by generating
  // the base address and checking everything.
  Value *StoreBasePtr = Expander.expandCodeFor(
      StoreEv->getStart(), Builder.getInt8PtrTy(SI->getPointerAddressSpace()),
      ExpPt);
  Value *LoadBasePtr = nullptr;

  bool Overlap = false;
  bool DestVolatile = SI->isVolatile();
  Type *BECountTy = BECount->getType();

  if (DestVolatile) {
    // The trip count must fit in i32, since it is the type of the "num_words"
    // argument to fgpu_memcpy_forward_vp4cp4n2.
    if (StoreSize != 4 || DL->getTypeSizeInBits(BECountTy) > 32) {
    CleanupAndExit:
      // If we generated new code for the base pointer, clean up.
      Expander.clear();
      if (StoreBasePtr && (LoadBasePtr != StoreBasePtr)) {
        RecursivelyDeleteTriviallyDeadInstructions(StoreBasePtr, TLI);
        StoreBasePtr = nullptr;
      }
      if (LoadBasePtr) {
        RecursivelyDeleteTriviallyDeadInstructions(LoadBasePtr, TLI);
        LoadBasePtr = nullptr;
      }
      return false;
    }
  }

  SmallPtrSet<Instruction *, 2> Ignore1;
  Ignore1.insert(SI);
  if (mayLoopAccessLocation(StoreBasePtr, ModRefInfo::ModRef, CurLoop, BECount,
                            StoreSize, *AA, Ignore1)) {
    // Check if the load is the offending instruction.
    Ignore1.insert(LI);
    if (mayLoopAccessLocation(StoreBasePtr, ModRefInfo::ModRef, CurLoop,
                              BECount, StoreSize, *AA, Ignore1)) {
      // Still bad. Nothing we can do.
      goto CleanupAndExit;
    }
    // It worked with the load ignored.
    Overlap = true;
  }

  if (!Overlap) {
    if (false || !HasMemcpy)
      goto CleanupAndExit;
  } else {
    // Don't generate memmove if this function will be inlined. This is
    // because the caller will undergo this transformation after inlining.
    Function *Func = CurLoop->getHeader()->getParent();
    if (Func->hasFnAttribute(Attribute::AlwaysInline))
      goto CleanupAndExit;

    // In case of a memmove, the call to memmove will be executed instead
    // of the loop, so we need to make sure that there is nothing else in
    // the loop than the load, store and instructions that these two depend
    // on.
    SmallVector<Instruction *, 2> Insts;
    Insts.push_back(SI);
    Insts.push_back(LI);
    if (!coverLoop(CurLoop, Insts))
      goto CleanupAndExit;

    if (false || !HasMemmove)
      goto CleanupAndExit;
    bool IsNested = CurLoop->getParentLoop() != nullptr;
    if (IsNested && false)
      goto CleanupAndExit;
  }

  // For a memcpy, we have to make sure that the input array is not being
  // mutated by the loop.
  LoadBasePtr = Expander.expandCodeFor(
      LoadEv->getStart(), Builder.getInt8PtrTy(LI->getPointerAddressSpace()),
      ExpPt);

  SmallPtrSet<Instruction *, 2> Ignore2;
  Ignore2.insert(SI);
  if (mayLoopAccessLocation(LoadBasePtr, ModRefInfo::Mod, CurLoop, BECount,
                            StoreSize, *AA, Ignore2))
    goto CleanupAndExit;

  // Check the stride.
  bool StridePos = getSCEVStride(LoadEv) >= 0;

  // Currently, the volatile memcpy only emulates traversing memory forward.
  if (!StridePos && DestVolatile)
    goto CleanupAndExit;

  bool RuntimeCheck = (Overlap || DestVolatile);

  BasicBlock *ExitB;
  if (RuntimeCheck) {
    // The runtime check needs a single exit block.
    SmallVector<BasicBlock *, 8> ExitBlocks;
    CurLoop->getUniqueExitBlocks(ExitBlocks);
    if (ExitBlocks.size() != 1)
      goto CleanupAndExit;
    ExitB = ExitBlocks[0];
  }

  // The # stored bytes is (BECount+1)*Size.  Expand the trip count out to
  // pointer size if it isn't already.
  LLVMContext &Ctx = SI->getContext();
  BECount = SE->getTruncateOrZeroExtend(BECount, IntPtrTy);
  DebugLoc DLoc = SI->getDebugLoc();

  const SCEV *NumBytesS =
      SE->getAddExpr(BECount, SE->getOne(IntPtrTy), SCEV::FlagNUW);
  if (StoreSize != 1)
    NumBytesS = SE->getMulExpr(NumBytesS, SE->getConstant(IntPtrTy, StoreSize),
                               SCEV::FlagNUW);
  Value *NumBytes = Expander.expandCodeFor(NumBytesS, IntPtrTy, ExpPt);
  if (Instruction *In = dyn_cast<Instruction>(NumBytes))
    if (Value *Simp = SimplifyInstruction(In, {*DL, TLI, DT}))
      NumBytes = Simp;

  CallInst *NewCall;

  if (RuntimeCheck) {
    unsigned Threshold = 1000;
    if (ConstantInt *CI = dyn_cast<ConstantInt>(NumBytes)) {
      uint64_t C = CI->getZExtValue();
      if (Threshold != 0 && C < Threshold)
        goto CleanupAndExit;
      if (C < 1000)
        goto CleanupAndExit;
    }

    BasicBlock *Header = CurLoop->getHeader();
    Function *Func = Header->getParent();
    Loop *ParentL = LF->getLoopFor(Preheader);
    StringRef HeaderName = Header->getName();

    // Create a new (empty) preheader, and update the PHI nodes in the
    // header to use the new preheader.
    BasicBlock *NewPreheader =
        BasicBlock::Create(Ctx, HeaderName + ".rtli.ph", Func, Header);
    if (ParentL)
      ParentL->addBasicBlockToLoop(NewPreheader, *LF);
    IRBuilder<>(NewPreheader).CreateBr(Header);
    for (auto &In : *Header) {
      PHINode *PN = dyn_cast<PHINode>(&In);
      if (!PN)
        break;
      int bx = PN->getBasicBlockIndex(Preheader);
      if (bx >= 0)
        PN->setIncomingBlock(bx, NewPreheader);
    }
    DT->addNewBlock(NewPreheader, Preheader);
    DT->changeImmediateDominator(Header, NewPreheader);

    // Check for safe conditions to execute memmove.
    // If stride is positive, copying things from higher to lower addresses
    // is equivalent to memmove.  For negative stride, it's the other way
    // around.  Copying forward in memory with positive stride may not be
    // same as memmove since we may be copying values that we just stored
    // in some previous iteration.
    Value *LA = Builder.CreatePtrToInt(LoadBasePtr, IntPtrTy);
    Value *SA = Builder.CreatePtrToInt(StoreBasePtr, IntPtrTy);
    Value *LowA = StridePos ? SA : LA;
    Value *HighA = StridePos ? LA : SA;
    Value *CmpA = Builder.CreateICmpULT(LowA, HighA);
    Value *Cond = CmpA;

    // Check for distance between pointers. Since the case LowA < HighA
    // is checked for above, assume LowA >= HighA.
    Value *Dist = Builder.CreateSub(LowA, HighA);
    Value *CmpD = Builder.CreateICmpSLE(NumBytes, Dist);
    Value *CmpEither = Builder.CreateOr(Cond, CmpD);
    Cond = CmpEither;

    if (Threshold != 0) {
      Type *Ty = NumBytes->getType();
      Value *Thr = ConstantInt::get(Ty, Threshold);
      Value *CmpB = Builder.CreateICmpULT(Thr, NumBytes);
      Value *CmpBoth = Builder.CreateAnd(Cond, CmpB);
      Cond = CmpBoth;
    }
    BasicBlock *MemmoveB = BasicBlock::Create(Ctx, Header->getName() + ".rtli",
                                              Func, NewPreheader);
    if (ParentL)
      ParentL->addBasicBlockToLoop(MemmoveB, *LF);
    Instruction *OldT = Preheader->getTerminator();
    Builder.CreateCondBr(Cond, MemmoveB, NewPreheader);
    OldT->eraseFromParent();
    Preheader->setName(Preheader->getName() + ".old");
    DT->addNewBlock(MemmoveB, Preheader);
    // Find the new immediate dominator of the exit block.
    BasicBlock *ExitD = Preheader;
    for (auto PI = pred_begin(ExitB), PE = pred_end(ExitB); PI != PE; ++PI) {
      BasicBlock *PB = *PI;
      ExitD = DT->findNearestCommonDominator(ExitD, PB);
      if (!ExitD)
        break;
    }
    // If the prior immediate dominator of ExitB was dominated by the
    // old preheader, then the old preheader becomes the new immediate
    // dominator.  Otherwise don't change anything (because the newly
    // added blocks are dominated by the old preheader).
    if (ExitD && DT->dominates(Preheader, ExitD)) {
      DomTreeNode *BN = DT->getNode(ExitB);
      DomTreeNode *DN = DT->getNode(ExitD);
      BN->setIDom(DN);
    }

    // Add a call to memmove to the conditional block.
    IRBuilder<> CondBuilder(MemmoveB);
    CondBuilder.CreateBr(ExitB);
    CondBuilder.SetInsertPoint(MemmoveB->getTerminator());

    if (DestVolatile) {
      Type *Int32Ty = Type::getInt32Ty(Ctx);
      Type *Int32PtrTy = Type::getInt32PtrTy(Ctx);
      Type *VoidTy = Type::getVoidTy(Ctx);
      Module *M = Func->getParent();
      FunctionCallee Fn = M->getOrInsertFunction("asdf", VoidTy, Int32PtrTy,
                                                 Int32PtrTy, Int32Ty);

      const SCEV *OneS = SE->getConstant(Int32Ty, 1);
      const SCEV *BECount32 = SE->getTruncateOrZeroExtend(BECount, Int32Ty);
      const SCEV *NumWordsS = SE->getAddExpr(BECount32, OneS, SCEV::FlagNUW);
      Value *NumWords =
          Expander.expandCodeFor(NumWordsS, Int32Ty, MemmoveB->getTerminator());
      if (Instruction *In = dyn_cast<Instruction>(NumWords))
        if (Value *Simp = SimplifyInstruction(In, {*DL, TLI, DT}))
          NumWords = Simp;

      Value *Op0 = (StoreBasePtr->getType() == Int32PtrTy)
                       ? StoreBasePtr
                       : CondBuilder.CreateBitCast(StoreBasePtr, Int32PtrTy);
      Value *Op1 = (LoadBasePtr->getType() == Int32PtrTy)
                       ? LoadBasePtr
                       : CondBuilder.CreateBitCast(LoadBasePtr, Int32PtrTy);
      NewCall = CondBuilder.CreateCall(Fn, {Op0, Op1, NumWords});
    } else {
      NewCall = CondBuilder.CreateMemMove(
          StoreBasePtr, SI->getAlign(), LoadBasePtr, LI->getAlign(), NumBytes);
    }
  } else {
    NewCall = Builder.CreateMemCpy(StoreBasePtr, SI->getAlign(), LoadBasePtr,
                                   LI->getAlign(), NumBytes);
    // Okay, the memcpy has been formed.  Zap the original store and
    // anything that feeds into it.
    RecursivelyDeleteTriviallyDeadInstructions(SI, TLI);
  }

  NewCall->setDebugLoc(DLoc);

  LLVM_DEBUG(dbgs() << "  Formed " << (Overlap ? "memmove: " : "memcpy: ")
                    << *NewCall << "\n"
                    << "    from load ptr=" << *LoadEv << " at: " << *LI << "\n"
                    << "    from store ptr=" << *StoreEv << " at: " << *SI
                    << "\n");

  return true;
}

// Check if the instructions in Insts, together with their dependencies
// cover the loop in the sense that the loop could be safely eliminated once
// the instructions in Insts are removed.
bool FgpuLoopIdiomRecognize::coverLoop(
    Loop *L, SmallVectorImpl<Instruction *> &Insts) const {
  SmallSet<BasicBlock *, 8> LoopBlocks;
  for (auto *B : L->blocks())
    LoopBlocks.insert(B);

  SetVector<Instruction *> Worklist(Insts.begin(), Insts.end());

  // Collect all instructions from the loop that the instructions in Insts
  // depend on (plus their dependencies, etc.).  These instructions will
  // constitute the expression trees that feed those in Insts, but the trees
  // will be limited only to instructions contained in the loop.
  for (unsigned i = 0; i < Worklist.size(); ++i) {
    Instruction *In = Worklist[i];
    for (auto I = In->op_begin(), E = In->op_end(); I != E; ++I) {
      Instruction *OpI = dyn_cast<Instruction>(I);
      if (!OpI)
        continue;
      BasicBlock *PB = OpI->getParent();
      if (!LoopBlocks.count(PB))
        continue;
      Worklist.insert(OpI);
    }
  }

  // Scan all instructions in the loop, if any of them have a user outside
  // of the loop, or outside of the expressions collected above, then either
  // the loop has a side-effect visible outside of it, or there are
  // instructions in it that are not involved in the original set Insts.
  for (auto *B : L->blocks()) {
    for (auto &In : *B) {
      if (isa<BranchInst>(In) || isa<DbgInfoIntrinsic>(In))
        continue;
      if (!Worklist.count(&In) && In.mayHaveSideEffects())
        return false;
      for (auto K : In.users()) {
        Instruction *UseI = dyn_cast<Instruction>(K);
        if (!UseI)
          continue;
        BasicBlock *UseB = UseI->getParent();
        if (LF->getLoopFor(UseB) != L)
          return false;
      }
    }
  }

  return true;
}

bool FgpuLoopIdiomRecognize::recogniseDot(Loop *L, int BECount,
                                          bool canChange) {
  if (BECount != 31) {
    LLVM_DEBUG(dbgs() << "Dot Loop count not doable, give up\n");
    return false; // TODO: multiple of 32, also tail loop
  }
  SmallVector<LoadInst *, 2> Loads;
  if (L->getNumBlocks() != 1) {
    LLVM_DEBUG(dbgs() << "Dot Multiple BB, give up\n");
    return false;
  }

  Instruction *add = nullptr;
  Instruction *mul = nullptr;
  PHINode *phi = nullptr;
  LoadInst *loada = nullptr;
  LoadInst *loadb = nullptr;

  for (auto &B : L->getBlocks()) {
    collectLoads(L, B, Loads);
    for (Instruction &I : *B) {
      if (I.getOpcode() == Instruction::FAdd) {
        if (!add) {
          add = &I;
        } else {
          LLVM_DEBUG(dbgs() << "Dot Multiple FADD, give up\n");
          return false; // multiple fadd, give up
        }
      } else if (I.getOpcode() == Instruction::FMul) {
        if (!mul) {
          mul = &I;
        } else {
          LLVM_DEBUG(dbgs() << "Dot Multiple FMUL, give up\n");
          return false; // multiple fmul, give up
        }
      } // end fmul
    }
  }

  if (Loads.size() != 2) {
    LLVM_DEBUG(dbgs() << "Dot not correct amount/type loads, give up\n");
    return false;
  }
  loada = Loads[0];
  loadb = Loads[1];
  if (!loada || !loadb || !add || !mul) {
    LLVM_DEBUG(dbgs() << "Dot can't find instruction\n");
    return false; // can't find an instruction
  }
  if (!add->isFast() || !mul->isFast()) {
    LLVM_DEBUG(dbgs() << "Dot not fast float math\n");
    return false;
  }
  if (loada->getNumUses() != 1 || loadb->getNumUses() != 1) {
    LLVM_DEBUG(dbgs() << "Dot Loads are re-used/not used\n");
    return false;
  }
  if (mul->getNumUses() != 1) {
    LLVM_DEBUG(dbgs() << "Dot Mul is re-used/not used\n");
    return false;
  }
  Value* addra = getUnderlyingObject(loada->getPointerOperand());
  Value* addrb = getUnderlyingObject(loadb->getPointerOperand());
  if (!addra || !addrb) {
    LLVM_DEBUG(dbgs() << "Dot couldn't discover base addresses\n");
    return false;
  }

  // mul sources are the loads
  if (mul->getNumOperands() != 2) {
    LLVM_DEBUG(dbgs() << "Dot too many mul sources\n");
    return false;
  }
  Instruction *m1 = dyn_cast<Instruction>(mul->getOperand(0));
  Instruction *m2 = dyn_cast<Instruction>(mul->getOperand(1));
  if (!(m1 == loada && m2 == loadb) && !(m1 == loadb && m2 == loada)) {
    LLVM_DEBUG(dbgs() << "Dot couldn't match mul-loads\n");
    return false;
  }
  if (add->getNumOperands() != 2) {
    LLVM_DEBUG(dbgs() << "Dot too many add sources\n");
    return false;
  }
  Instruction *a1 = dyn_cast<Instruction>(add->getOperand(0));
  Instruction *a2 = dyn_cast<Instruction>(add->getOperand(1));
  if (!(a1 == mul || a2 == mul)) {
    LLVM_DEBUG(dbgs() << "Dot couldn't match add-mul\n");
    return false;
  }
  phi = dyn_cast<PHINode>(a1 != mul ? a1 : a2);
  if (!phi || phi->getNumOperands() != 2) {
    LLVM_DEBUG(dbgs() << "Dot too many phi sources\n");
    return false;
  }
  Value *p1 = phi->getOperand(0); // comes from who knows where, not our problem
  Instruction *p2 = dyn_cast<Instruction>(phi->getOperand(1));
  if (p2 != add) {
    LLVM_DEBUG(dbgs() << "Dot phi not as expected\n");
    return false;
  }
  // TODO: p1 and p2 could be flipped
  //TODO: this doesn't work since it can't analyse the floating point value...
//  if (!SE->isLoopInvariant(SE->getSCEV(p1), L)) {
//    LLVM_DEBUG(dbgs() << "Dot phi 1st value not invariant\n");
//    return false;
//  }


  // load stride stuff, should be handled by the collectLoads/isValidLoad
  // TODO: check addrspace of B is 1, also maybe A
  // could also emit fixup if they aren't

  if (!canChange) {
    return true;
  }
  // TODO: do the replacement
  // p1 is destination?? or original
  // and do we make loop dead or what??
  // the destination of add is the "output"
  const DebugLoc &DLoc = add->getDebugLoc();


  // It should have a preheader containing nothing but an unconditional branch.
  BasicBlock *PH = L->getLoopPreheader();
  if (!PH || &PH->front() != PH->getTerminator()) {
    LLVM_DEBUG(dbgs() << "Dot Couldn't get preheader\n");
    return false;
  }
  auto *EntryBI = dyn_cast<BranchInst>(PH->getTerminator());
  if (!EntryBI || EntryBI->isConditional()) {
    LLVM_DEBUG(dbgs() << "Dot Couldn't get entry branch inst\n");
    return false;
  }

  // TODO: why does the cntpop not want anything in the preheader? why does it want it
  // in the precondition????????

  LLVMContext &Ctx = add->getContext();
  Type* f32v32Ty = FixedVectorType::get(Type::getFloatTy(Ctx), 32);
  Type* f32v32ptrTy1 = PointerType::get(f32v32Ty, 1);

  IRBuilder<> Builder(EntryBI);

  Value* aaddrvec = Builder.CreatePointerCast(addra, f32v32ptrTy1);  // TODO: addrspace
  LoadInst* newloada = Builder.CreateLoad(f32v32Ty, aaddrvec);

  Value* baddrvec = Builder.CreatePointerCast(addrb, f32v32ptrTy1);

  Value* zero = ConstantInt::get(Type::getInt32Ty(Ctx), 0);

  ArrayRef<Value*> args = {p1, newloada, baddrvec, zero}; // TODO: deal with multi-nesting, in which case this can be nonzero
  Value* dotIntrin = createDotIntrinsic(Builder, args, DLoc);

  BasicBlock *Body = *(L->block_begin());
  add->replaceUsesOutsideBlock(dotIntrin, Body);
  return true;
}

bool FgpuLoopIdiomRecognize::runOnCountableLoop(Loop *L) {
  //  PolynomialMultiplyRecognize PMR(L, *DL, *DT, *TLI, *SE);
  //  if (PMR.recognize())
  //    return true;

  const SCEV *BECount = SE->getBackedgeTakenCount(L);
  assert(!isa<SCEVCouldNotCompute>(BECount) &&
         "runOnCountableLoop() called on a loop without a predictable"
         "backedge-taken count");
  LLVM_DEBUG(L->dumpVerbose());

  SmallVector<BasicBlock *, 8> ExitBlocks;
  L->getUniqueExitBlocks(ExitBlocks);

  bool Changed = false;
  if (recogniseDot(L,
                   dyn_cast<SCEVConstant>(BECount)->getAPInt().getSExtValue(),
                   false)) {
    LLVM_DEBUG(dbgs() << "dot found\n");
    Changed = recogniseDot(L, dyn_cast<SCEVConstant>(BECount)->getAPInt().getSExtValue(),
                   true);
  } else {
    LLVM_DEBUG(dbgs() << "dot not found\n");
  }

  return Changed;
}

bool FgpuLoopIdiomRecognize::run(Loop *L) {
  const Module &M = *L->getHeader()->getParent()->getParent();
  if (Triple(M.getTargetTriple()).getArch() != Triple::fgpu)
    return false;
  LLVM_DEBUG(dbgs() << "In loop idiom recognise\n");
  if (DisableLoopIdiom.getValue()) {
    LLVM_DEBUG(dbgs() << "Fgpu loop idiom recognise disabled, bailing\n");
    return false;
  }
  // Strategery
  // Form we are looking for
  // iteration count = 32 (for now, could make multiple/have tail)
  // two loads, from two vectors of floats
  // multiply those floats
  // add to an accumulator
  //

  // If the loop could not be converted to canonical form, it must have an
  // indirectbr in it, just give up.
  if (!L->getLoopPreheader())
    return false;

  // Disable loop idiom recognition if the function's name is a common idiom.
  StringRef Name = L->getHeader()->getParent()->getName();
  if (Name == "memset" || Name == "memcpy" || Name == "memmove")
    return false;

  DL = &L->getHeader()->getModule()->getDataLayout();
  if (SE->hasLoopInvariantBackedgeTakenCount(L))
    return runOnCountableLoop(L);
  return false;
}

bool FgpuLoopIdiomRecognizeLegacyPass::runOnLoop(Loop *L, LPPassManager &LPM) {
  if (skipLoop(L))
    return false;

  auto *AA = &getAnalysis<AAResultsWrapperPass>().getAAResults();
  auto *DT = &getAnalysis<DominatorTreeWrapperPass>().getDomTree();
  auto *LF = &getAnalysis<LoopInfoWrapperPass>().getLoopInfo();
  auto *TLI = &getAnalysis<TargetLibraryInfoWrapperPass>().getTLI(
      *L->getHeader()->getParent());
  auto *SE = &getAnalysis<ScalarEvolutionWrapperPass>().getSE();
  return FgpuLoopIdiomRecognize(AA, DT, LF, TLI, SE).run(L);
}

Pass *llvm::createFgpuLoopIdiomPass() {
  return new FgpuLoopIdiomRecognizeLegacyPass();
}

PreservedAnalyses
FgpuLoopIdiomRecognitionPass::run(Loop &L, LoopAnalysisManager &AM,
                                  LoopStandardAnalysisResults &AR,
                                  LPMUpdater &U) {
  return FgpuLoopIdiomRecognize(&AR.AA, &AR.DT, &AR.LI, &AR.TLI, &AR.SE).run(&L)
             ? getLoopPassPreservedAnalyses()
             : PreservedAnalyses::all();
}
