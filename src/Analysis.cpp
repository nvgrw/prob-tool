
#include <iostream>
#include <memory>
#include <numeric>

#include <llvm/IR/DebugInfoMetadata.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Module.h>

#include <unsupported/Eigen/KroneckerProduct>

#include "Analysis.hpp"

using namespace llvm;

namespace {
pt::SpMat makeMat(unsigned Dim, std::vector<pt::SpTrp> Triplets) {
  pt::SpMat Mat(Dim, Dim);
  Mat.setFromTriplets(Triplets.begin(), Triplets.end());
  return Mat;
}
} // anonymous namespace

namespace pt {
Analysis::Analysis(std::unique_ptr<llvm::Module> const &Module)
    : Module(Module) {
  prepareModule();
  computeLabels();
  computeVariables();

  dumpLabeled();
}

SpMat Analysis::run() { return computeMatrix(translateTransforms()); }

void Analysis::dumpLabeled() {
  std::cerr << "===== Labeled Program =====" << std::endl;
  for (auto &F : Module->functions()) {
    fprintf(stderr, "(%s)\n", F.getName().data());
    for (auto &BB : F) {
      fprintf(stderr, "%s:\n", BB.getName().data());
      for (auto &I : BB) {
        if (hasLabel(&I)) {
          fprintf(stderr, "%02u | ", Labels[&I]);
        } else {
          fprintf(stderr, "   | ");
        }
        std::cerr.flush();
        I.print(llvm::errs());
        fprintf(stderr, "\n");
      }
    }
  }
}

bool Analysis::hasLabel(const llvm::Value *Inst) const {
  return Labels.find(Inst) != Labels.end();
}

bool Analysis::isLabelable(const llvm::BasicBlock *BB,
                           const llvm::Instruction *Inst) const {
  switch (Inst->getOpcode()) {
  case Instruction::Br:
  case Instruction::Choose:
  case Instruction::Switch:
  case Instruction::Ret:
  case Instruction::Store:
    return true;
  default:
    return false;
  }
}

void Analysis::prepareModule() {
  for (const auto *DU : Module->debug_compile_units()) {
    assert(!DU->isOptimized() && "Analysis requires unoptimized module.");
  }

  // Remove the names from all values + declarations
  for (Module::iterator FI = Module->begin(), E = Module->end(); FI != E;) {
    Function &F = *FI++;

    for (auto &BB : F)
      for (auto &I : BB)
        I.setName("");
  }
}

void Analysis::computeLabels() {
  for (auto &F : Module->functions()) {
    for (auto &BB : F) {
      bool BlockLabeled = false;
      for (auto &I : BB) {
        if (!isLabelable(&BB, &I))
          continue;

        if (!BlockLabeled) {
          BlockLabeled = true;
          BasicBlockLabels[&BB] = MaxLabels;
        }

        Labels[&I] = MaxLabels++;
      }
    }
  }
}

void Analysis::computeVariables() {
  unsigned VariableIndex = 0;
  // TODO: better handling for variables across functions
  for (const auto &F : *Module) {
    for (const auto &BB : F) {
      for (const auto &I : BB) {
        const CallInst *CI = dyn_cast<CallInst>(&I);
        if (!CI || CI->getIntrinsicID() != Intrinsic::dbg_declare)
          continue;

        // TODO: obtain range from user or metadata
        Variables.push_back(new pt::IntSymVar(const_cast<CallInst *>(CI),
                                              APInt(64, 0, false),
                                              APInt(64, 2, false)));
        ValToVarIndex[cast<ValueAsMetadata>(
                          cast<MetadataAsValue>(CI->getOperand(0))
                              ->getMetadata())
                          ->getValue()] = VariableIndex++;
      }
    }
  }
}

std::vector<std::vector<SpMat>> Analysis::translateTransforms() {
  std::vector<std::vector<SpMat>> Matrices;

  // TODO: deal with multiple functions (currently assuming just 1)
  for (auto &F : Module->functions()) {
    for (auto &BB : F) {
      // Evaluate all possible combinations of variables
      if (!Variables.empty()) {
        pt::Evaluator Evaluator(Module->getDataLayout(), nullptr, ValToVarIndex,
                                Variables);
        Evaluator.evaluate(BB);

        for (auto &I : BB) {
          if (!hasLabel(&I)) {
            continue;
          }

          translateInstruction(Evaluator, Matrices, &I);
        }
      }
      // TODO: TRANSLATE INSTRUCTIONS IN FUNCTIONS WITHOUT VARIABLES
    }
  }

  return Matrices;
}

void Analysis::translateInstruction(pt::Evaluator const &Evaluator,
                                    std::vector<std::vector<SpMat>> &Matrices,
                                    llvm::Instruction const *Instruction) {
  unsigned NumStates = Evaluator.getNumStates();
  unsigned FromLabel = Labels[Instruction];
  SpMat StateIdentity(NumStates, NumStates);
  StateIdentity.setIdentity();

  switch (Instruction->getOpcode()) {
  case Instruction::Br: {
    const llvm::BranchInst *BI = llvm::cast<llvm::BranchInst>(Instruction);
    if (BI->isUnconditional()) {
      SpMat TransferMatrix = makeMat(
          MaxLabels,
          {SpTrp(FromLabel, BasicBlockLabels[BI->getSuccessor(0)], 1.0)});
      Matrices.push_back({StateIdentity, TransferMatrix});
      break;
    }

    std::vector<SpTrp> TrueStateTriplets, FalseStateTriplets;
    for (unsigned StateIndex = 0; StateIndex < NumStates; StateIndex++) {
      const llvm::Constant *V = Evaluator.getValue(
          StateIndex, const_cast<llvm::Value *>(BI->getCondition()));
      TrueStateTriplets.push_back(
          SpTrp(StateIndex, StateIndex, V->isZeroValue() ? 0.0 : 1.0));
      FalseStateTriplets.push_back(
          SpTrp(StateIndex, StateIndex, !V->isZeroValue() ? 0.0 : 1.0));
    }

    SpMat TrueStateMatrix = makeMat(NumStates, TrueStateTriplets);
    SpMat FalseStateMatrix = makeMat(NumStates, FalseStateTriplets);

    SpMat TrueTransferMatrix =
        makeMat(MaxLabels,
                {SpTrp(FromLabel, BasicBlockLabels[BI->getSuccessor(0)], 1.0)});
    SpMat FalseTransferMatrix =
        makeMat(MaxLabels,
                {SpTrp(FromLabel, BasicBlockLabels[BI->getSuccessor(1)], 1.0)});

    Matrices.push_back({TrueStateMatrix, TrueTransferMatrix});
    Matrices.push_back({FalseStateMatrix, FalseTransferMatrix});
  } break;
  case Instruction::Choose: {
    const llvm::ChooseInst *CI = llvm::cast<llvm::ChooseInst>(Instruction);

    uint64_t WeightSum = 0;
    for (auto &Choice : CI->choices()) {
      WeightSum += Choice.getChoiceWeight()->getZExtValue();
    }

    std::vector<SpTrp> TransferTriplets;
    for (auto &Choice : CI->choices()) {
      unsigned ToLabel = BasicBlockLabels[Choice.getChoiceSuccessor()];
      TransferTriplets.push_back(
          SpTrp(FromLabel, ToLabel,
                Choice.getChoiceWeight()->getZExtValue() / (double)WeightSum));
    }

    SpMat TransferMatrix = makeMat(MaxLabels, TransferTriplets);
    Matrices.push_back({StateIdentity, TransferMatrix});
  } break;
  case Instruction::Switch:
    llvm_unreachable("Switch not implemented");
  case Instruction::Ret: {
    SpMat TransferMatrix =
        makeMat(MaxLabels, {SpTrp(FromLabel, FromLabel, 1.0)});
    Matrices.push_back({StateIdentity, TransferMatrix});
  } break;
  case Instruction::Store: {
    const auto *Store = cast<llvm::StoreInst>(Instruction);
    unsigned StoreToVarIndex = ValToVarIndex[Store->getPointerOperand()];
    pt::IntSymVar const *Variable = Variables[StoreToVarIndex];

    std::vector<SpTrp> StateTriplets;
    for (unsigned StateIndex = 0; StateIndex < NumStates; StateIndex++) {
      const llvm::Constant *V = Evaluator.getValue(
          StateIndex, const_cast<llvm::Value *>(Store->getValueOperand()));
      // todo: remove const cast if it works

      unsigned ValueIndex = Variable->getIndexOfValue(V);
      auto Location = Evaluator.getLocation(StateIndex);
      Location[StoreToVarIndex].Index = ValueIndex;
      unsigned DestinationStateIndex = Evaluator.getStateIndex(Location);
      StateTriplets.push_back(SpTrp(StateIndex, DestinationStateIndex, 1.0));
    }

    SpMat StateMatrix = makeMat(NumStates, StateTriplets);
    SpMat TransferMatrix =
        makeMat(MaxLabels, {SpTrp(FromLabel, FromLabel + 1, 1.0)});
    Matrices.push_back({StateMatrix, TransferMatrix});
  } break;
  default:
    llvm_unreachable("Unsupported labeled instruction");
  }
}

SpMat Analysis::computeMatrix(
    std::vector<std::vector<SpMat>> const &Matrices) const {
  SpMat Identity1(1, 1);
  Identity1.setIdentity();

  SpMat Result;
  bool ResultSet = false;
  for (const std::vector<SpMat> &Kroneckerize : Matrices) {
    SpMat V =
        std::accumulate(Kroneckerize.begin(), Kroneckerize.end(), Identity1,
                        [&](const SpMat &Acc, const SpMat &Val) -> SpMat {
                          return Eigen::kroneckerProduct(Acc, Val).eval();
                        });
    if (!ResultSet) {
      Result = V;
      ResultSet = true;
    } else {
      Result = Result + V;
    }
  }

  assert(ResultSet && "No matrix generated");
  return Result;
}
} // namespace pt
