#include "Evaluator.hpp"

#include <llvm/IR/Instructions.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Transforms/Utils/Evaluator.h>

#include <memory>
#include <string>

using namespace pt;

void Evaluator::markSymbolic(IntSymVar *Var) { Symbolic.insert(Var); }

void Evaluator::unmarkSymbolic(IntSymVar *Var) { Symbolic.erase(Var); }

bool Evaluator::isSymbolic(IntSymVar *Var) const {
  return Symbolic.find(Var) != Symbolic.end();
}

void Evaluator::evaluate(llvm::BasicBlock &BB) {
  std::unordered_map<const llvm::Value *, llvm::Constant *> Instance;
  permuteVariablesAndExecute(0, Symbolic.begin(), BB, Instance);
}

void Evaluator::permuteVariablesAndExecute(unsigned VariableIndex,
                                           SymbolicSetT::iterator VarIt,
                                           llvm::BasicBlock &BB,
                                           EvalInstanceT &Instance) {
  if (VarIt == Symbolic.end()) {
    return;
  }

  bool IsFirst = VarIt == Symbolic.begin();
  IntSymVar *Variable = *VarIt;
  SymbolicSetT::iterator VarItNext = ++VarIt;

  unsigned ValueIndex = 0;
  for (IntSymVar::iterator VIt = Variable->begin(), E = Variable->end();
       VIt != E; /*todo: do i want this?*/ ++VIt) {
    llvm::ConstantInt *Value = *VIt;
    Instance[Variable->getAlloca()] = Value;

    if (VarItNext != Symbolic.end()) { // not at bottom level of tree
      permuteVariablesAndExecute(VariableIndex + 1, VarItNext, BB, Instance);
      continue;
    }

    for (auto &P : Instance) {
      std::string Os;
      llvm::raw_string_ostream Ros(Os);
      P.second->print(Ros);
      std::printf("%s ", Os.c_str());
    }
    std::printf("\n");

    // Execute
    //    llvm::Evaluator EV(DL, TLI);
    //    evaluateOnce(&EV, BB, Instance);
    //    std::printf("V%d: I%d\n", 4 - VariableIndex, ValueIndex);

    ValueIndex++;
  }
}

bool Evaluator::evaluateOnce(llvm::Evaluator *EV, llvm::BasicBlock &BB,
                             EvalInstanceT const &Instance) const {
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
