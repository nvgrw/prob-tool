#include "Evaluator.hpp"

#include <llvm/IR/Instructions.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Transforms/Utils/Evaluator.h>

#include <memory>

using namespace pt;

void Evaluator::markSymbolic(IntSymVar *Var) { Symbolic.insert(Var); }

void Evaluator::unmarkSymbolic(IntSymVar *Var) { Symbolic.erase(Var); }

bool Evaluator::isSymbolic(IntSymVar *Var) const {
  return Symbolic.find(Var) != Symbolic.end();
}

void Evaluator::evaluate(llvm::BasicBlock &BB) {
  assert(!States &&
         "evaluate() may only be run once on the same Evaluator instance");

  std::unordered_map<const llvm::Value *, unsigned> IndexMap;
  unsigned NumVariables = 0;
  NumStates = 1;
  for (pt::IntSymVar *SV : Symbolic) {
    IndexMap[SV->getAlloca()] = NumVariables++;
    NumStates *= SV->getRange();
  }

  States = new llvm::Evaluator *[NumStates];
  std::vector<Evaluator::InstanceElem> Instance(NumVariables);
  permuteVariablesAndExecute(Symbolic.begin(), BB, IndexMap, Instance);
}

void Evaluator::permuteVariablesAndExecute(
    SymbolicSetT::iterator VarIt, llvm::BasicBlock &BB,
    std::unordered_map<const llvm::Value *, unsigned> const &IndexMap,
    std::vector<InstanceElem> &Instance) {
  if (VarIt == Symbolic.end()) {
    return;
  }

  IntSymVar *Variable = *VarIt;
  unsigned VariableIndex = IndexMap.at(Variable->getAlloca());
  SymbolicSetT::iterator VarItNext = ++VarIt;

  unsigned ValueIndex = 0;
  for (llvm::ConstantInt *Value : *Variable) {
    Instance[VariableIndex] =
        InstanceElem(ValueIndex, Variable->getRange(), Value);

    if (VarItNext != Symbolic.end()) { // not at bottom level of tree
      permuteVariablesAndExecute(VarItNext, BB, IndexMap, Instance);
      ValueIndex++;
      continue;
    }

    // Execute
    llvm::Evaluator *EV = new llvm::Evaluator(DL, TLI);
    evaluateOnce(EV, BB, IndexMap, Instance);

    unsigned Index = 0;
    unsigned Multiplier = 1;
    for (int I = Instance.size() - 1; I >= 0; I--) {
      Index += Instance[I].Index * Multiplier;
      // ranges don't change but probably cheaper to store range in instance
      Multiplier *= Instance[I].Range;
    }
    States[Index] = EV; // save the execution state

    ValueIndex++;
  }
}

bool Evaluator::evaluateOnce(
    llvm::Evaluator *EV, llvm::BasicBlock &BB,
    std::unordered_map<const llvm::Value *, unsigned> const &IndexMap,
    std::vector<InstanceElem> const &Instance) const {
  llvm::BasicBlock *NextBB = nullptr;
  llvm::BasicBlock::iterator CurInst = BB.begin();
  llvm::SmallVector<std::unique_ptr<llvm::GlobalVariable>, 10> AllocaTmps;

  // TODO: only do this for variables that actually show up in this block
  for (IntSymVar const *Variable : Symbolic) {
    const llvm::AllocaInst *AI = Variable->getAlloca();
    const llvm::Value *Value = llvm::cast<llvm::Value>(AI);
    llvm::Constant *InstanceValue = Instance[IndexMap.at(Value)].Value;

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
