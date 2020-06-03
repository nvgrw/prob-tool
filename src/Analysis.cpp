
#include <iostream>
#include <map>
#include <memory>
#include <numeric>

#include <llvm/IR/DebugInfoMetadata.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Module.h>

#include <unsupported/Eigen/KroneckerProduct>

#include "Analysis.hpp"

using namespace llvm;

namespace {
pt::SpMat makeMat(uint64_t Dim, std::vector<pt::SpTrp> Triplets) {
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
}

SpMat Analysis::run() { return computeMatrix(translateTransforms()); }

std::tuple<Eigen::Matrix<std::string, Eigen::Dynamic, 1>, Eigen::MatrixXi>
Analysis::states() {
  Eigen::Matrix<std::string, Eigen::Dynamic, 1> VariableNames(Variables.size());
  for (unsigned I = 0; I < Variables.size(); I++) {
    const auto *Variable = Variables[I];
    const CallInst *CI = Variable->getDecl();
    auto *DIV = cast<DIVariable>(
        cast<MetadataAsValue>(CI->getOperand(1))->getMetadata());
    VariableNames(I) = DIV->getName();
  }

  uint64_t NumStates = Evaluator::getNumStates(Variables);
  std::vector<Eigen::VectorXi> StateVectorsNoLabel;
  for (uint64_t StateIndex = 0; StateIndex < NumStates; StateIndex++) {
    auto State = Evaluator::getLocation(Variables, StateIndex);
    Eigen::VectorXi StateVector(State.size() + 1);
    for (uint64_t ValueIndex = 0; ValueIndex < State.size(); ValueIndex++) {
      StateVector(ValueIndex) = State[ValueIndex].Index;
    }
    StateVectorsNoLabel.push_back(StateVector);
  }

  Eigen::MatrixXi States(NumStates * MaxLabels, Variables.size() + 1);
  uint64_t Insert = 0;
  for (unsigned Label = 0; Label < MaxLabels; Label++) {
    for (auto V : StateVectorsNoLabel) {
      V(V.rows() - 1) = Label;
      States.row(Insert++) = V.transpose();
    }
  }

  return std::make_tuple(VariableNames.transpose(), States);
}

std::vector<SpMat> Analysis::abstractPrints(
    std::unordered_set<std::string> const &Whitelist) const {
  const Function &F = *Module->getFunction("main");

  SmallPtrSet<const BasicBlock *, 8> BlocksToEvaluate;
  SmallVector<const CallInst *, 8> CallInstructions;
  // Find the blocks and expressions that need to be evaluated
  for (const auto &BB : F) {
    for (const auto &I : BB) {
      const CallInst *Inst = dyn_cast<CallInst>(&I);
      if (Inst == nullptr || !Inst->getType()->isVoidTy()) // not a void call
        continue;
      std::string Name(Inst->getCalledFunction()->getName());
      if (Whitelist.find(Name) == Whitelist.end()) // not whitelisted
        continue;

      BlocksToEvaluate.insert(&BB);
      CallInstructions.push_back(Inst);
    }
  }

  // Compute block values
  // Yes this does duplicate work, but there isn't enough time to make this nice
  // right now.
  std::unordered_map<const BasicBlock *, std::unique_ptr<pt::Evaluator>>
      Evaluators;
  for (const BasicBlock *Block : BlocksToEvaluate) {
    auto Evaluator = std::make_unique<pt::Evaluator>(
        Module->getDataLayout(), nullptr, ValToVarIndex, Variables);
    Evaluator->evaluate(*const_cast<BasicBlock *>(Block));
    Evaluators[Block] = std::move(Evaluator);
  }

  std::vector<SpMat> Results;
  for (const CallInst *CI : CallInstructions) {
    const auto &Evaluator = Evaluators[CI->getParent()];
    uint64_t NumStates = Evaluator->getNumStates();

    // dynamically determine the range of values based on what is seen
    std::map<int64_t, std::vector<uint64_t>> ConstantValueToIndices;
    for (uint64_t StateIndex = 0; StateIndex < NumStates; StateIndex++) {
      ConstantInt *C = dyn_cast<ConstantInt>(
          Evaluator->getValue(StateIndex, CI->getArgOperand(0)));
      // TODO: support these down the line
      // This should not be a huge issue as booleans and integers are
      // represented using the LLVM arbitrary precision integral type.
      assert(C && "Non-ConstantInt values are currently unsupported");

      int64_t Value = C->isOne() ? C->getZExtValue() : C->getSExtValue();
      ConstantValueToIndices[Value].push_back(StateIndex);
    }

    // put these values into a matrix, using the fact that std::map is ordered
    // by key
    uint64_t CurrentColumn = 0;
    std::vector<SpTrp> AbstractTriplets;
    for (const auto ValueAndIndices : ConstantValueToIndices) {
      for (uint64_t StateIndex : ValueAndIndices.second) {
        AbstractTriplets.push_back(SpTrp(StateIndex, CurrentColumn, 1.0));
      }
      CurrentColumn++;
    }

    SpMat AbstractMatrix(NumStates, CurrentColumn);
    AbstractMatrix.setFromTriplets(AbstractTriplets.begin(),
                                   AbstractTriplets.end());
    Eigen::MatrixXd TransitionMatrix(MaxLabels, 1);
    TransitionMatrix.setOnes();
    SpMat OutMatrix = Eigen::kroneckerProduct(AbstractMatrix, TransitionMatrix);
    Results.push_back(OutMatrix);
  }

  return Results;
}

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

bool Analysis::isLabelable(const llvm::Instruction *Inst) const {
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
        if (!isLabelable(&I))
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
  // TODO: better handling for multiple functions
  const Function &F = *Module->getFunction("main");
  for (const auto &BB : F) {
    for (const auto &I : BB) {
      const CallInst *CI = dyn_cast<CallInst>(&I);
      if (!CI || CI->getIntrinsicID() != Intrinsic::dbg_declare)
        continue;

      ValToVarIndex[cast<ValueAsMetadata>(
                        cast<MetadataAsValue>(CI->getOperand(0))->getMetadata())
                        ->getValue()] = VariableIndex++;

      auto *VarMeta = cast<DILocalVariable>(
          cast<MetadataAsValue>(CI->getOperand(1))->getMetadata());
      uint64_t SizeInBits = VarMeta->getType()->getSizeInBits();
      int64_t MaxVal = ~(UINT64_MAX << (SizeInBits - 1));
      int64_t MinVal = -MaxVal - 1;

      // TODO: obtain range from user or metadata
      Variables.push_back(new pt::IntSymVar(const_cast<CallInst *>(CI),
                                            APInt(64, MinVal, true),
                                            APInt(64, MaxVal, true)));
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
  uint64_t NumStates = Evaluator.getNumStates();
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
    for (uint64_t StateIndex = 0; StateIndex < NumStates; StateIndex++) {
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
    for (uint64_t StateIndex = 0; StateIndex < NumStates; StateIndex++) {
      const llvm::Constant *V = Evaluator.getValue(
          StateIndex, const_cast<llvm::Value *>(Store->getValueOperand()));
      // todo: remove const cast if it works

      unsigned ValueIndex = Variable->getIndexOfValue(V);
      auto Location = Evaluator.getLocation(StateIndex);
      Location[StoreToVarIndex].Index = ValueIndex;
      uint64_t DestinationStateIndex = Evaluator.getStateIndex(Location);
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
