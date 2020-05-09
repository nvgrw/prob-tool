#include "Evaluator.hpp"

#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Transforms/Utils/Evaluator.h>

#include <memory>

using namespace pt;

Evaluator::Evaluator(
    const llvm::DataLayout &DL, const llvm::TargetLibraryInfo *TLI,
    std::unordered_map<const llvm::Value *, unsigned int> IndexMap,
    const std::vector<IntSymVar *> &Symbolic)
    : DL(DL), TLI(TLI), IndexMap(IndexMap), Symbolic(Symbolic),
      NumStates([&Symbolic]() {
        unsigned V = 1;
        for (pt::IntSymVar *SV : Symbolic) {
          V *= SV->getRange();
        }
        return V;
      }()) {
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
  unsigned Index = getStateIndex(Location);
  return getValue(Index, V);
}

std::unique_ptr<std::vector<Evaluator::StateAddressed>>
Evaluator::getLocation(unsigned int StateIndex) const {
  auto Location =
      std::make_unique<std::vector<Evaluator::StateAddressed>>(Symbolic.size());
  for (int I = Symbolic.size() - 1; I >= 0; I--) {
    unsigned Range = Symbolic[I]->getRange();
    (*Location)[I].Range = Range;
    if (StateIndex == 0) {
      continue;
    }
    unsigned Index = StateIndex % Range;
    (*Location)[I].Index = Index;
    StateIndex -= Index;
    StateIndex /= Range;
  }

  return std::move(Location);
}

unsigned
Evaluator::getStateIndex(const std::vector<StateAddressed> &Location) const {
  unsigned Index = 0;
  unsigned Multiplier = 1;
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

bool Evaluator::evaluateOnce(llvm::Evaluator *EV, llvm::BasicBlock &BB,
                             std::vector<InstanceElem> const &Instance) const {
  llvm::BasicBlock *NextBB = nullptr;

  struct ValueTmp {
    std::unique_ptr<llvm::GlobalVariable> Global;
    llvm::Instruction *Replaced;
    llvm::LoadInst *Load = nullptr;
  };
  std::vector<ValueTmp> ValueTmps(Symbolic.size());

  // Builder that inserts before start
  llvm::IRBuilder<> Builder(&*BB.begin());

  // ** Initialize each variable that appears ** //
  for (llvm::Instruction &Inst : BB) {
    auto It = IndexMap.find(&Inst);
    if (It == IndexMap.end()) // no variable
      continue;

    unsigned Reg = It->second;
    if (ValueTmps[Reg].Global != nullptr) // already allocated elsewhere
      continue;

    llvm::Constant *InstanceValue = Instance[Reg].Value;
    ValueTmps[Reg] = {.Global = std::make_unique<llvm::GlobalVariable>(
                          Inst.getType(), false,
                          llvm::GlobalValue::InternalLinkage, InstanceValue,
                          Inst.getName(), llvm::GlobalValue::NotThreadLocal, 0),
                      .Replaced = &Inst};
  }

  // ** Replace all instructions with globals ** //
  for (ValueTmp &VT : ValueTmps) {
    if (VT.Global == nullptr)
      continue;
    VT.Load = Builder.CreateLoad(VT.Global.get());
    VT.Replaced->replaceAllUsesWith(VT.Load);
    VT.Replaced->removeFromParent();
  }

  llvm::BasicBlock::iterator CurInst = BB.begin();
  bool Success = EV->EvaluateBlock(CurInst, NextBB);

  // ** Revert changes to BasicBlock ** //
  for (ValueTmp &VT : ValueTmps) {
    if (VT.Global == nullptr)
      continue;
    VT.Replaced->insertAfter(VT.Load);
    VT.Load->replaceAllUsesWith(VT.Replaced);
    VT.Load->eraseFromParent();
  }

  return Success;
}
