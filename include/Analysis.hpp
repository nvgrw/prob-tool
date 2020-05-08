#ifndef PT_ANALYSIS_HPP
#define PT_ANALYSIS_HPP

#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include <llvm/ADT/SmallVector.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>

#include "Evaluator.hpp"

namespace llvm {
class AllocaInst;
class Value;
} // namespace llvm

// todo: put in pt namespace
class Analysis {
private:
  llvm::LLVMContext Context;
  std::unique_ptr<llvm::Module> Module;

  llvm::SmallVector<pt::IntSymVar, 10> Variables;
  std::unordered_map<const llvm::Value *, unsigned> ValToVarIndex;

  std::map<const llvm::Value *, unsigned> Labels;
  std::map<const llvm::BasicBlock *, unsigned> BasicBlockLabels;
  unsigned MaxLabels = 0;

public:
  Analysis(const std::string &Filename);

private:
  bool hasLabel(const llvm::Value *Inst) const;

private:
  void readAndParse(const std::string &Filename);
  void prepareModule();
  void computeLabels();
  void computeVariables();
  void translateTransforms();
  void translateInstruction(pt::Evaluator const &Evaluator,
                            llvm::Instruction const *Instruction);

  void printLabeled();
};

#endif // PT_ANALYSIS_HPP
