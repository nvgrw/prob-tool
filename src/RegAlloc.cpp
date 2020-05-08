
#include "RegAlloc.hpp"

#include <stack>
#include <unordered_map>
#include <unordered_set>

#include <llvm/Analysis/LoopInfo.h>
#include <llvm/IR/CFG.h>
#include <llvm/IR/Dominators.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Module.h>

namespace pt {

class AllocContext {
  unsigned InstructionCount = 0;
  std::unordered_map<const llvm::Instruction *, unsigned> InstructionID;
  std::unique_ptr<llvm::LoopInfo> LoopInfo;

public:
  AllocContext(const llvm::Function &Func) {
    for (const llvm::BasicBlock &BB : Func) {
      for (const llvm::Instruction &I : BB) {
        InstructionID[&I] = InstructionCount++;
      }
    }

    // Perform loop analysis
    if (!Func.isDeclaration()) {
      llvm::FunctionAnalysisManager FAM;
      FAM.registerPass([&] { return llvm::PassInstrumentationAnalysis(); });
      FAM.registerPass([&] { return llvm::DominatorTreeAnalysis(); });
      FAM.registerPass([&] { return llvm::LoopAnalysis(); });
      LoopInfo = std::make_unique<llvm::LoopInfo>();
      LoopInfo->analyze(FAM.getResult<llvm::DominatorTreeAnalysis>(
          const_cast<llvm::Function &>(Func)));
    }
  }

public:
  unsigned getInstID(const llvm::Instruction *Inst) {
    return InstructionID[Inst];
  }

  unsigned getBlockStart(const llvm::BasicBlock *Block) {
    return InstructionID[&Block->front()];
  }

  unsigned getBlockEnd(const llvm::BasicBlock *Block) {
    return InstructionID[&Block->back()];
  }

  bool isLoopHeader(const llvm::BasicBlock *Block) const {
    if (LoopInfo == nullptr) {
      return false;
    }

    return LoopInfo->isLoopHeader(Block);
  }

  const llvm::BasicBlock *getLoopEnd(const llvm::BasicBlock *Header) {
    // get exit blocks and find the lowest block i guess
    llvm::SmallVector<llvm::BasicBlock *, 8> ExitBlocks;
    LoopInfo->getLoopFor(Header)->getExitBlocks(ExitBlocks);
    llvm::BasicBlock *LastBlock = nullptr;
    for (llvm::BasicBlock *ExitBlock : ExitBlocks) {
      if (LastBlock == nullptr) {
        LastBlock = ExitBlock;
        continue;
      }

      if (getBlockStart(ExitBlock) <= getBlockStart(LastBlock)) {
        continue;
      }

      LastBlock = ExitBlock;
    }

    assert(LastBlock && "No last block found for header");
    return LastBlock;
  }
};

void RegAlloc::allocate() const {
  for (const auto &F : Module.functions()) {
    if (F.isDeclaration())
      continue;

    AllocContext Ctx(F);
    auto Intervals = buildIntervals(Ctx, F);
  }
}

namespace {
using ValSet = std::unordered_set<const llvm::Value *>;
ValSet &getLiveIn(std::unordered_map<const llvm::BasicBlock *, ValSet> &LiveIn,
                  llvm::BasicBlock const *BB) {
  auto It = LiveIn.find(BB);
  if (It != LiveIn.end()) {
    return It->second;
  }

  LiveIn[BB] = ValSet();
  return LiveIn[BB];
}
} // anonymous namespace

std::unique_ptr<std::unordered_map<const llvm::Value *, Interval>>
RegAlloc::buildIntervals(AllocContext &Ctx, const llvm::Function &Func) const {
  std::unordered_map<const llvm::BasicBlock *, ValSet> LiveInMap;
  auto Intervals =
      std::make_unique<std::unordered_map<const llvm::Value *, Interval>>();

  const auto &Blocks = Func.getBasicBlockList();
  for (auto BIt = Blocks.rbegin(), BE = Blocks.rend(); BIt != BE; ++BIt) {
    const llvm::BasicBlock &BB = *BIt;

    // Union of all successor live sets
    ValSet Live;
    for (const llvm::BasicBlock *SuccBB : llvm::successors(&BB)) {
      ValSet &LiveIn = getLiveIn(LiveInMap, SuccBB);
      Live.insert(LiveIn.begin(), LiveIn.end());
    }

    // Collect all successor phi values depending on BB
    std::stack<const llvm::BasicBlock *> BFSBlocks;
    llvm::SmallPtrSet<const llvm::BasicBlock*, 32> BlockSeen;
    BFSBlocks.push(&BB);
    while (!BFSBlocks.empty()) {
      const llvm::BasicBlock *StackBB = BFSBlocks.top();
      BFSBlocks.pop();
      // Ensure we only process a block once
      if (BlockSeen.find(StackBB) != BlockSeen.end()) {
        continue;
      }
      BlockSeen.insert(StackBB);

      for (const llvm::BasicBlock *SuccBB : llvm::successors(StackBB)) {
        for (const llvm::PHINode &PHI : SuccBB->phis()) {
          int Index = PHI.getBasicBlockIndex(&BB);
          if (Index == -1)
            continue;

          llvm::Value *Value = PHI.getIncomingValue(Index);
          if (llvm::isa<llvm::Constant>(Value) ||
              !llvm::isa<llvm::Instruction>(Value)) {
            continue;
          }

          Live.insert(Value);
        }
        BFSBlocks.push(SuccBB);
      }
    }

    // Initialize worst-case intervals
    for (const llvm::Value *V : Live) {
      if (Intervals->find(V) == Intervals->end()) {
        (*Intervals)[V] =
            Interval(Ctx.getBlockStart(&BB), Ctx.getBlockEnd(&BB));
        continue;
      }

      (*Intervals)[V].addRange(Ctx.getBlockStart(&BB), Ctx.getBlockEnd(&BB));
    }

    // Main part
    for (auto IIt = BB.rbegin(), IE = BB.rend(); IIt != IE; IIt++) {
      const llvm::Instruction &Inst = *IIt;
      // Don't process PHIs here
      if (llvm::isa<llvm::PHINode>(&Inst)) {
        continue;
      }

      // Output operands (self)
      if (Inst.getType() != llvm::Type::getVoidTy(Inst.getContext())) {
        (*Intervals)[&Inst].setFrom(Ctx.getInstID(&Inst));
        Live.erase(&Inst);
      }

      // Input operands
      for (const llvm::Value *Operand : Inst.operands()) {
        // Skip constants
        if (llvm::isa<llvm::Constant>(Operand) ||
            !llvm::isa<llvm::Instruction>(Operand)) {
          continue;
        }

        (*Intervals)[Operand].addRange(Ctx.getBlockStart(&BB),
                                       Ctx.getInstID(&Inst));
        Live.insert(Operand);
      }
    }

    // Remove phis from live set
    for (const llvm::PHINode &PHI : BB.phis()) {
      Live.erase(&PHI);
    }

    // Handle loops
    if (Ctx.isLoopHeader(&BB)) {
      const llvm::BasicBlock *LastBlock = Ctx.getLoopEnd(&BB);
      for (const llvm::Value *Value : Live) {
        (*Intervals)[Value].addRange(Ctx.getBlockStart(&BB),
                                     Ctx.getBlockEnd(LastBlock));
      }
    }

    // Update LiveIn for block
    getLiveIn(LiveInMap, &BB) = Live;
  }

  return std::move(Intervals);
}

} // namespace pt
