
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
#include <llvm/Transforms/Utils/Evaluator.h>

#include <Eigen/Dense>

#include "Analysis.hpp"
#include "Evaluator.hpp"

using namespace llvm;
using Eigen::MatrixXd;

namespace {
bool isLabelable(const Instruction *Inst) {
  switch (Inst->getOpcode()) {
  case Instruction::Br:
  case Instruction::Choose:
  case Instruction::Switch:
  case Instruction::Store:
  case Instruction::Select:
  case Instruction::Ret:
    return true;
  default:
    return false;
  }
}
} // namespace

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
  PM.add(createCFGSimplificationPass());
  PM.add(createConstantPropagationPass());
  PM.add(createDemoteRegisterToMemoryPass());
  PM.add(createDeadCodeEliminationPass());
  PM.run(*Module);

  // Remove calls, as calls are not supported.
  // TODO(nvgrw): provide better error messages in cases where functions are not
  // TODO(nvgrw): void functions. Could also improve by inlining first?
  for (auto &F : Module->functions()) {
    if (F.isDeclaration())
      continue;

    for (auto &BB : F) {
      for (BasicBlock::iterator II = BB.begin(), E = BB.end(); II != E;) {
        CallInst *CI = dyn_cast<CallInst>(II++);
        if (!CI)
          continue;

        CI->eraseFromParent();
      }
    }
  }

  // Remove the names from all values + declarations
  for (Module::iterator FI = Module->begin(), E = Module->end(); FI != E;) {
    Function &F = *FI++;
    if (F.isDeclaration()) {
      F.eraseFromParent();
      continue;
    }

    for (auto &BB : F)
      for (auto &I : BB)
        I.setName("");
  }
}

void Analysis::computeLabels() {
  for (auto &F : Module->functions()) {
    for (auto &BB : F) {
      for (auto &I : BB) {
        if (!isLabelable(&I))
          continue;

        Labels[&I] = MaxLabels++;
      }
    }
  }
}

void Analysis::computeVariables() {
  for (auto &F : Module->functions()) {
    for (auto &BB : F)
      for (auto &I : BB) {
        AllocaInst *AI = dyn_cast<AllocaInst>(&I);
        if (!AI)
          continue;

        Variables.push_back(AI);
      }
  }
}

void Analysis::translateTransforms() {
  // TODO(nvgrw): change this to accurately reflect # possible values
  //  unsigned Dimension = 3 * Variables.size();

  // Create some kind of system that 'executes' the file from top to bottom,
  // computing the values of expressions as we go. We could replace variables
  // with 'instantiable constants' that are not actually constant but are
  // evaluated as such.

  pt::Evaluator Evaluator;
  for (auto &V : Variables) {
    Evaluator.markSymbolic(V);
  }

  for (auto &F : Module->functions()) {
    for (auto &BB : F) {
      Evaluator.evaluate(BB, Module->getDataLayout());
    }
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
