#ifndef PT_ANALYSIS_HPP
#define PT_ANALYSIS_HPP

#include <memory>
#include <string>
#include <map>

#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/ADT/SmallVector.h>

#include "Evaluator.hpp"

namespace llvm {
  class AllocaInst;
  class Value;
} // namespace llvm

class Analysis {
private:
  llvm::LLVMContext Context;
  std::unique_ptr<llvm::Module> Module;

  llvm::SmallVector<pt::IntSymVar, 10> Variables;
  std::map<const llvm::Value *, unsigned> Labels;
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

  void printLabeled();
};

#endif // PT_ANALYSIS_HPP
