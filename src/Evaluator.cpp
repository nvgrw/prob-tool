#include "Evaluator.hpp"

#include <llvm/IR/Instructions.h>
#include <llvm/Transforms/Utils/Evaluator.h>
#include <memory>

using namespace pt;

void Evaluator::markSymbolic(IntSymVar const *Var) { Symbolic.insert(Var); }

void Evaluator::unmarkSymbolic(IntSymVar const *Var) { Symbolic.erase(Var); }

bool Evaluator::isSymbolic(IntSymVar const *Var) const {
  return Symbolic.find(Var) != Symbolic.end();
}

bool Evaluator::evaluateOnce(
    llvm::Evaluator *EV, llvm::BasicBlock &BB,
    std::unordered_map<const llvm::Value *, llvm::Constant *> const &Instance)
    const {
  llvm::BasicBlock *NextBB = nullptr;
  llvm::BasicBlock::iterator CurInst = BB.begin();
  llvm::SmallVector<std::unique_ptr<llvm::GlobalVariable>, 10> AllocaTmps;

  // TODO: only do this for variables that actually show up in this block
  for (IntSymVar const *Variable : Symbolic) {
    const llvm::AllocaInst *AI = Variable->getAlloca();
    const llvm::Value *Value = llvm::cast<llvm::Value>(AI);
    llvm::Constant *InstanceValue = Instance.at(Value);

    // Create a temporary variable used to refer to the concrete instance of
    // the symbolic variable.
    AllocaTmps.push_back(std::make_unique<llvm::GlobalVariable>(
        AI->getAllocatedType(), false, llvm::GlobalValue::InternalLinkage,
        InstanceValue, AI->getName(), llvm::GlobalValue::NotThreadLocal,
        AI->getType()->getPointerAddressSpace()));
    EV->setVal(const_cast<llvm::Value *>(Value), AllocaTmps.back().get());
  }

  return EV->EvaluateBlock(CurInst, NextBB);
}
