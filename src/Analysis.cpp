
#include <iostream>
#include <memory>

#include <llvm/Bitcode/BitcodeReader.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Support/MemoryBuffer.h>
#include <llvm/Transforms/Scalar.h>

#include "Analysis.hpp"

using namespace llvm;

Analysis::Analysis(const std::string &Filename) {
  readAndParse(Filename);
  prepareModule();
  computeVariables();
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

void Analysis::computeVariables() {
  for (auto &F : Module->functions()) {
    if (F.isDeclaration())
      continue;

    for (auto &BB : F)
      for (auto &I : BB)
        I.dump();
  }
}
