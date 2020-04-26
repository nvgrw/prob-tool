
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

#include <Eigen/Dense>

#include "Analysis.hpp"

using namespace llvm;
using Eigen::MatrixXd;

namespace {
bool is_labelable(const Instruction *Inst) {
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

  // TODO(nvgrw): change this to accurate reflect # possible values
  //  unsigned Dimension = 3 * Variables.size();

  printLabeled();
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

  // Remove the names from all values
  for (auto &F : Module->functions()) {
    if (F.isDeclaration())
      continue;

    for (auto &BB : F)
      for (auto &I : BB)
        I.setName("");
  }
}

void Analysis::computeLabels() {
  for (auto &F : Module->functions()) {
    if (F.isDeclaration())
      continue;

    for (auto &BB : F) {
      for (auto &I : BB) {
        if (!is_labelable(&I))
          continue;

        Labels[&I] = MaxLabels++;
      }
    }
  }
}

void Analysis::computeVariables() {
  for (auto &F : Module->functions()) {
    if (F.isDeclaration())
      continue;

    for (auto &BB : F)
      for (auto &I : BB) {
        AllocaInst *AI = dyn_cast<AllocaInst>(&I);
        if (!AI)
          continue;

        Variables.push_back(AI);
      }
  }
}

void Analysis::printLabeled() {
  for (auto &F : Module->functions()) {
    if (F.isDeclaration())
      continue;

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
