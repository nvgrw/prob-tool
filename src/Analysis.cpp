
#include <iostream>
#include <memory>
#include <string>

#include <llvm/Bitcode/BitcodeReader.h>
#include <llvm/IR/DebugInfoMetadata.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/MemoryBuffer.h>
#include <llvm/Transforms/Scalar.h>
#include <llvm/Transforms/Utils.h>

#include <Eigen/Dense>
#include <llvm/IR/Intrinsics.h>
#include <unsupported/Eigen/KroneckerProduct>

#include "Analysis.hpp"
#include "Evaluator.hpp"

using namespace llvm;
using Eigen::MatrixXd;

Analysis::Analysis(const std::string &Filename) {
  readAndParse(Filename);
  prepareModule();
  computeLabels();
  computeVariables();

  printLabeled();

  translateTransforms();
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

void Analysis::readAndParse(const std::string &Filename) {
  auto BufferOrError = MemoryBuffer::getFile(Filename);
  if (std::error_code Ec = BufferOrError.getError()) {
    report_fatal_error(errorCodeToError(Ec));
  }
  MemoryBufferRef Buffer(*BufferOrError.get());
  auto ModuleOrError = parseBitcodeFile(Buffer, Context);
  if (Error Error = ModuleOrError.takeError()) {
    report_fatal_error(std::move(Error));
  }
  this->Module = std::move(ModuleOrError.get());
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

void Analysis::translateTransforms() {
  for (auto &F : Module->functions()) {
    for (auto &BB : F) {
      // Evaluate all possible combinations of variables
      std::cout << "BB " << BB.getName().data() << std::endl;
      if (!Variables.empty()) {
        pt::Evaluator Evaluator(Module->getDataLayout(), nullptr, ValToVarIndex,
                                Variables);
        Evaluator.evaluate(BB);

        for (auto &I : BB) {
          if (!hasLabel(&I)) {
            continue;
          }

          translateInstruction(Evaluator, &I);
        }
      }
      // TODO: TRANSLATE INSTRUCTIONS IN FUNCTIONS WITHOUT VARIABLES
    }
  }
}

void Analysis::translateInstruction(pt::Evaluator const &Evaluator,
                                    llvm::Instruction const *Instruction) {
  {
    std::string OutString;
    llvm::raw_string_ostream OutStream(OutString);
    Instruction->print(OutStream);
    std::printf("TR: (L%u) %s\n", Labels[Instruction], OutString.c_str());
  }

  unsigned NumStates = Evaluator.getNumStates();
  unsigned FromLabel = Labels[Instruction];
  MatrixXd StateIdentity(NumStates, NumStates);
  StateIdentity.setIdentity();
  std::vector<std::vector<MatrixXd>> Matrices;

  switch (Instruction->getOpcode()) {
  case Instruction::Br: {
    const llvm::BranchInst *BI = llvm::cast<llvm::BranchInst>(Instruction);
    if (BI->isUnconditional()) {
      MatrixXd TransferMatrix(MaxLabels, MaxLabels);
      TransferMatrix.setZero();
      TransferMatrix(FromLabel, BasicBlockLabels[BI->getSuccessor(0)]) = 1.0;
      std::cout<<TransferMatrix <<std::endl;
      Matrices.push_back({StateIdentity, TransferMatrix});
      break;
    }

    MatrixXd TrueStateMatrix(NumStates, NumStates),
        FalseStateMatrix(NumStates, NumStates);
    TrueStateMatrix.setZero();
    FalseStateMatrix.setZero();
    for (unsigned StateIndex = 0; StateIndex < NumStates; StateIndex++) {
      const llvm::Constant *V = Evaluator.getValue(
          StateIndex, const_cast<llvm::Value *>(BI->getCondition()));
      TrueStateMatrix(StateIndex, StateIndex) = V->isZeroValue() ? 0.0 : 1.0;
      FalseStateMatrix(StateIndex, StateIndex) = !V->isZeroValue() ? 0.0 : 1.0;
    }

    MatrixXd TrueTransferMatrix(MaxLabels, MaxLabels),
        FalseTransferMatrix(MaxLabels, MaxLabels);
    TrueTransferMatrix.setZero();
    FalseTransferMatrix.setZero();

    TrueTransferMatrix(FromLabel, BasicBlockLabels[BI->getSuccessor(0)]) = 1.0;
    FalseTransferMatrix(FromLabel, BasicBlockLabels[BI->getSuccessor(1)]) = 1.0;

    Matrices.push_back({TrueStateMatrix, TrueTransferMatrix});
    Matrices.push_back({FalseStateMatrix, FalseTransferMatrix});
  } break;
  case Instruction::Choose: {
    const llvm::ChooseInst *CI = llvm::cast<llvm::ChooseInst>(Instruction);

    uint64_t WeightSum = 0;
    for (auto &Choice : CI->choices()) {
      WeightSum += Choice.getChoiceWeight()->getZExtValue();
    }

    MatrixXd TransferMatrix(MaxLabels, MaxLabels);
    TransferMatrix.setZero();
    for (auto &Choice : CI->choices()) {
      unsigned ToLabel = BasicBlockLabels[Choice.getChoiceSuccessor()];
      TransferMatrix(FromLabel, ToLabel) =
          Choice.getChoiceWeight()->getZExtValue() / (double)WeightSum;
    }

    Matrices.push_back({StateIdentity, TransferMatrix});
  } break;
  case Instruction::Switch:
    llvm_unreachable("Switch not implemented");
  case Instruction::Ret: {
    MatrixXd TransferMatrix(MaxLabels, MaxLabels);
    TransferMatrix.setZero();
    TransferMatrix(FromLabel, FromLabel) = 1.0;

    Matrices.push_back({StateIdentity, TransferMatrix});
  } break;
  case Instruction::Store: {
    const auto *Store = cast<llvm::StoreInst>(Instruction);
    unsigned StoreToVarIndex = ValToVarIndex[Store->getPointerOperand()];
    pt::IntSymVar const *Variable = Variables[StoreToVarIndex];

    MatrixXd StateMatrix(NumStates, NumStates);
    StateMatrix.setZero();
    for (unsigned StateIndex = 0; StateIndex < NumStates; StateIndex++) {
      const llvm::Constant *V = Evaluator.getValue(
          StateIndex, const_cast<llvm::Value *>(Store->getValueOperand()));
      // todo: remove const cast if it works

      unsigned ValueIndex = Variable->getIndexOfValue(V);
      auto Location = Evaluator.getLocation(StateIndex);
      Location[StoreToVarIndex].Index = ValueIndex;
      unsigned DestinationStateIndex = Evaluator.getStateIndex(Location);
      StateMatrix(StateIndex, DestinationStateIndex) = 1.0;
    }

    MatrixXd TransferMatrix(MaxLabels, MaxLabels);
    TransferMatrix.setZero();
    TransferMatrix(FromLabel, FromLabel + 1) = 1.0;

    Matrices.push_back({StateMatrix, TransferMatrix});
  } break;
  default:
    llvm_unreachable("Unsupported labeled instruction");
  }
}

void Analysis::printLabeled() {
  for (auto &F : Module->functions()) {
    std::printf("(%s)\n", F.getName().data());
    for (auto &BB : F) {
      std::printf("%s:\n", BB.getName().data());
      for (auto &I : BB) {
        if (hasLabel(&I)) {
          std::printf("%02u | ", Labels[&I]);
        } else {
          std::printf("   | ");
        }
        std::cout.flush();
        I.print(llvm::outs());
        printf("\n");
      }
    }
  }
}
