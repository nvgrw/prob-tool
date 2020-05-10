
#include <iostream>
#include <memory>
#include <string>

#include <llvm/Bitcode/BitcodeReader.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/MemoryBuffer.h>
#include <llvm/Transforms/Scalar.h>
#include <llvm/Transforms/Utils.h>

#include <Eigen/Dense>

#include "Analysis.hpp"
#include "Evaluator.hpp"
#include "RegAlloc.hpp"

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
  // Trivial check first
  switch (Inst->getOpcode()) {
  case Instruction::Br:
  case Instruction::Choose:
  case Instruction::Switch:
  case Instruction::Ret:
    return true;
  default:
    break;
  }

  if (Inst->isUsedOutsideOfBlock(BB) || Inst->hasNUses(0)) {
    // TODO: figure out a way to handle this and cases in which this breaks down
    return false;
  }

  bool HasNonVoidUser = false;
  for (const llvm::Value *User : Inst->users()) {
    if (!User->getType()->isVoidTy()) {
      HasNonVoidUser = true;
      break;
    }
  }

  return !HasNonVoidUser;
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
  legacy::PassManager PM;
  PM.add(createConstantPropagationPass());
  PM.add(createPromoteMemoryToRegisterPass());
  PM.add(createCFGSimplificationPass());
  PM.add(createDeadCodeEliminationPass());
  PM.run(*Module);

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
  pt::RegAlloc RA(*Module);
  auto Allocations = RA.allocate();
  // TODO: work on multiple functions

  auto &AllocPair = Allocations[Module->getFunction("main")];
  unsigned NumAllocVars = std::get<1>(AllocPair);
  // map alloc var to label var???

//  ValToVarIndex = *std::get<0>(AllocPair);
  for (unsigned Index = 0; Index < NumVars; Index++) {
    Variables.push_back(new pt::IntSymVar(
        Module->getContext(), APInt(64, 0, false), APInt(64, 2, false)));
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

  switch (Instruction->getOpcode()) {
  case Instruction::Br: {
    const llvm::BranchInst *BI = llvm::cast<llvm::BranchInst>(Instruction);
    if (BI->isUnconditional()) {
      // todo: handle unconditional
      break;
    }

    MatrixXd Matrix(NumStates, NumStates);
    Matrix.setZero();
    for (unsigned StateIndex = 0; StateIndex < NumStates; StateIndex++) {
      const llvm::Constant *V = Evaluator.getValue(
          StateIndex, const_cast<llvm::Value *>(BI->getCondition()));
      Matrix(StateIndex, StateIndex) = V->isZeroValue() ? 0.0 : 1.0;
      // TODO: generate another matrix for !Condition + Edge transfer
    }

    std::cout << Matrix << std::endl;
  } break;
  case Instruction::Choose: {
    const llvm::ChooseInst *CI = llvm::cast<llvm::ChooseInst>(Instruction);

    uint64_t WeightSum = 0;
    for (auto &Choice : CI->choices()) {
      WeightSum += Choice.getChoiceWeight()->getZExtValue();
    }

    MatrixXd Matrix(MaxLabels, MaxLabels);
    Matrix.setZero();
    for (auto &Choice : CI->choices()) {
      unsigned ToLabel = BasicBlockLabels[Choice.getChoiceSuccessor()];
      Matrix(FromLabel, ToLabel) =
          Choice.getChoiceWeight()->getZExtValue() / (double)WeightSum;
    }

    std::cout << Matrix << std::endl;
  } break;
  case Instruction::Switch:
    llvm_unreachable("Switch not implemented");
  case Instruction::Ret:
    llvm_unreachable("Ret not implemented");
    //  case Instruction::Select: {
    //    const llvm::SelectInst *SI =
    //    llvm::cast<llvm::SelectInst>(Instruction);
    //
    //    for (unsigned StateIndex = 0; StateIndex < NumStates; StateIndex++) {
    //      const llvm::Constant *V = Evaluator.getValue(
    //          StateIndex, const_cast<llvm::Value *>(SI->getCondition()));
    //      V->dump();
    //    }
    //  } break;
    //  case Instruction::Switch:
    //  case Instruction::Select:
    //  case Instruction::Ret:
  case Instruction::PHI:
    // TODO: handle phis separately
    break;
  default: {
    unsigned StoreToVarIndex = ValToVarIndex[Instruction];
    pt::IntSymVar const *Variable = Variables[StoreToVarIndex];

    MatrixXd Matrix(NumStates, NumStates);
    Matrix.setZero();
    for (unsigned StateIndex = 0; StateIndex < NumStates; StateIndex++) {
      const llvm::Constant *V = Evaluator.getValue(
          StateIndex, const_cast<llvm::Instruction *>(Instruction));
      // todo: remove const cast if it works

      unsigned ValueIndex = Variable->getIndexOfValue(V);
      auto Location = Evaluator.getLocation(StateIndex);
      (*Location)[StoreToVarIndex].Index = ValueIndex;
      unsigned DestinationStateIndex = Evaluator.getStateIndex(*Location);
      Matrix(StateIndex, DestinationStateIndex) = 1.0;
    }

    std::cout << Matrix << std::endl;
  } break;
  }
}

void Analysis::printLabeled() {
  for (auto &F : Module->functions()) {
    std::printf("(%s)\n", F.getName().data());
    for (auto &BB : F) {
      std::printf("%s:\n", BB.getName().data());
      for (auto &I : BB) {
        auto VarIndexIt = ValToVarIndex.find(&I);
        if (VarIndexIt != ValToVarIndex.end()) {
          std::printf("%02u ", VarIndexIt->second);
        } else {
          std::printf("XX ");
        }

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
