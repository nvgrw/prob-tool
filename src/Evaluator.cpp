#include "Evaluator.hpp"

#include <llvm/IR/Instructions.h>
#include <llvm/Transforms/Utils/Evaluator.h>
#include <memory>

using namespace pt;

void Evaluator::markSymbolic(const llvm::AllocaInst *Instr) {
  Symbolic.insert(Instr);
}

void Evaluator::unmarkSymbolic(const llvm::AllocaInst *Instr) {
  Symbolic.erase(Instr);
}

bool Evaluator::isSymbolic(const llvm::AllocaInst *Instr) const {
  return Symbolic.find(Instr) != Symbolic.end();
}

bool Evaluator::evaluateOnce(
    llvm::Evaluator *EV, llvm::BasicBlock &BB,
    std::unordered_map<const llvm::Value *, llvm::Constant *> const &Instance)
    const {
  llvm::BasicBlock *NextBB = nullptr;
  llvm::BasicBlock::iterator CurInst = BB.begin();
  llvm::SmallVector<std::unique_ptr<llvm::GlobalVariable>, 10> AllocaTmps;

  // TODO: only do this for variables that actually show up in this block
  for (const llvm::AllocaInst *Variable : Symbolic) {
    const llvm::Value *Value = llvm::cast<llvm::Value>(Variable);
    llvm::Constant *InstanceValue = Instance.at(Value);

    // Create a temporary variable used to refer to the concrete instance of
    // the symbolic variable.
    AllocaTmps.push_back(std::make_unique<llvm::GlobalVariable>(
        Variable->getAllocatedType(), false, llvm::GlobalValue::InternalLinkage,
        InstanceValue, Variable->getName(), llvm::GlobalValue::NotThreadLocal,
        Variable->getType()->getPointerAddressSpace()));
    EV->setVal(const_cast<llvm::Value *>(Value), AllocaTmps.back().get());
  }

  return EV->EvaluateBlock(CurInst, NextBB);
}
