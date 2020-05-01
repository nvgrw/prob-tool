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

void Evaluator::evaluate(llvm::BasicBlock &BB,
                         const llvm ::DataLayout &DL) const {
  llvm::BasicBlock *NextBB = nullptr;
  llvm::BasicBlock::iterator CurInst = BB.begin();
  llvm::Evaluator Evaluator(DL, nullptr);
  llvm::SmallVector<std::unique_ptr<llvm::GlobalVariable>, 10> AllocaTmps;

  for (const llvm::AllocaInst *Variable : Symbolic) {
    // Alloca
    llvm::Type *Ty = Variable->getAllocatedType();

    llvm::Constant *InstanceValue =
        llvm::Constant::getIntegerValue(Ty, llvm::APInt(64, 1, true));
    AllocaTmps.push_back(std::make_unique<llvm::GlobalVariable>(
        Ty, false, llvm::GlobalValue::InternalLinkage, InstanceValue,
        Variable->getName(), llvm::GlobalValue::NotThreadLocal,
        Variable->getType()->getPointerAddressSpace()));
    const llvm::Value *Value = llvm::cast<llvm::Value>(Variable);
    Evaluator.setVal(const_cast<llvm::Value *>(Value), AllocaTmps.back().get());
  }

  __unused bool Status = Evaluator.EvaluateBlock(CurInst, NextBB);
}

/*
std::unique_ptr<Expr> Evaluator::buildExpression(const llvm::Value *Value) {
  // what is this value?
  // if it's a constant, store that
  // if it's a function, then cast & figure out how it's built
  // otherwise, bail.

  if (llvm::isa<llvm::Instruction>(Value)) {
    const llvm::Instruction *Inst = llvm::cast<llvm::Instruction>(Value);
    return nullptr;
  }

  if (llvm::isa<llvm::Constant>(Value)) {
    const llvm::Constant *Const = llvm::cast<llvm::Constant>(Value);
    return std::make_unique<ConstantExpr>(Const);
  }

  return nullptr;
}
 */
