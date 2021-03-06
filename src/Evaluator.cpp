#include <memory>

#include <llvm/IR/IRBuilder.h>

#include "Evaluator.hpp"

using namespace pt;

Evaluator::Evaluator(
    const llvm::DataLayout &DL, const llvm::TargetLibraryInfo *TLI,
    std::unordered_map<const llvm::Value *, unsigned int> IndexMap,
    const std::vector<IntSymVar *> &Symbolic)
    : DL(DL), TLI(TLI), IndexMap(IndexMap), Symbolic(Symbolic),
      NumStates([&Symbolic]() { return Evaluator::getNumStates(Symbolic); }()) {
  assert(!Symbolic.empty() && "Can't evaluate without variables");
}

void Evaluator::evaluate(llvm::BasicBlock &BB) {
  assert(!States &&
         "evaluate() may only be run once on the same Evaluator instance");
  States = new llvm::Evaluator *[NumStates];
  std::vector<Evaluator::InstanceElem> Instance(Symbolic.size());
  permuteVariablesAndExecute(Symbolic.begin(), BB, Instance);
}

llvm::Constant *Evaluator::getValue(const std::vector<StateAddressed> &Location,
                                    llvm::Value *V) const {
  assert(States && "getValue() called on unevaluated Evaluator");
  uint64_t Index = getStateIndex(Location);
  return getValue(Index, V);
}

uint64_t Evaluator::getNumStates(std::vector<IntSymVar *> const &Symbolic) {
  uint64_t V = 1;
  for (pt::IntSymVar *SV : Symbolic) {
    V *= SV->getRange();
  }
  return V;
}

std::vector<Evaluator::StateAddressed>
Evaluator::getLocation(uint64_t StateIndex) const {
  return getLocation(Symbolic, StateIndex);
}

std::vector<Evaluator::StateAddressed>
Evaluator::getLocation(std::vector<IntSymVar *> const &Symbolic,
                       uint64_t StateIndex) {
  std::vector<StateAddressed> Location(Symbolic.size());
  for (int I = Symbolic.size() - 1; I >= 0; I--) {
    uint64_t Range = Symbolic[I]->getRange();
    Location[I].Range = Range;
    if (StateIndex == 0) {
      continue;
    }
    uint64_t Index = StateIndex % Range;
    Location[I].Index = Index;
    StateIndex -= Index;
    StateIndex /= Range;
  }

  return Location;
}

uint64_t
Evaluator::getStateIndex(const std::vector<StateAddressed> &Location) const {
  uint64_t Index = 0;
  uint64_t Multiplier = 1;
  for (int I = Location.size() - 1; I >= 0; I--) {
    Index += Location[I].Index * Multiplier;
    Multiplier *= Location[I].Range;
  }
  return Index;
}

void Evaluator::permuteVariablesAndExecute(
    std::vector<IntSymVar *>::const_iterator VarIt, llvm::BasicBlock &BB,
    std::vector<InstanceElem> &Instance) {
  if (VarIt == Symbolic.end()) {
    return;
  }

  IntSymVar *Variable = *VarIt;
  unsigned VariableIndex = std::distance(Symbolic.begin(), VarIt);
  auto VarItNext = ++VarIt;

  unsigned ValueIndex = 0;
  for (llvm::ConstantInt *Value : *Variable) {
    Instance[VariableIndex] =
        InstanceElem(ValueIndex, Variable->getRange(), Value);

    if (VarItNext != Symbolic.end()) { // not at bottom level of tree
      permuteVariablesAndExecute(VarItNext, BB, Instance);
      ValueIndex++;
      continue;
    }

    // Execute
    llvm::Evaluator *EV = new llvm::Evaluator(DL, TLI);
    assert(evaluateOnce(EV, BB, Instance) && "Evaluation failed");

    uint64_t Index = 0;
    uint64_t Multiplier = 1;
    for (int I = Instance.size() - 1; I >= 0; I--) {
      Index += Instance[I].Index * Multiplier;
      // ranges don't change but probably cheaper to store range in instance
      Multiplier *= Instance[I].Range;
    }
    States[Index] = EV; // save the execution state

    ValueIndex++;
  }
}

bool Evaluator::evaluateOnce(llvm::Evaluator *EV, llvm::BasicBlock &BB,
                             std::vector<InstanceElem> const &Instance) const {
  llvm::BasicBlock *NextBB = nullptr;

  std::vector<std::unique_ptr<llvm::GlobalVariable>> ValueTmps(Symbolic.size());

  // Builder that inserts before start
  llvm::IRBuilder<> Builder(&*BB.begin());

  // Initialize each variable that appears
  unsigned Reg = 0;
  for (IntSymVar *Variable : Symbolic) {
    auto *Alloca = Variable->getAlloca();

    llvm::Constant *InstanceValue = Instance[Reg].Value;
    ValueTmps[Reg] = std::make_unique<llvm::GlobalVariable>(
        Alloca->getAllocatedType(), false, llvm::GlobalValue::InternalLinkage,
        InstanceValue, Alloca->getName(), llvm::GlobalValue::NotThreadLocal,
        Alloca->getType()->getPointerAddressSpace());
    EV->setVal(Alloca, ValueTmps[Reg].get());
    Reg++;
  }

  llvm::BasicBlock::iterator CurInst = BB.begin();
  return EV->EvaluateBlock(CurInst, NextBB, true, true);
}
