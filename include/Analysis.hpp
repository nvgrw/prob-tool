#ifndef PT_ANALYSIS_HPP
#define PT_ANALYSIS_HPP

#include <memory>
#include <string>

#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>

class Analysis {
private:
  llvm::LLVMContext Context;
  std::unique_ptr<llvm::Module> Module;

public:
  Analysis(const std::string &Filename);

private:
  void readAndParse(const std::string &Filename);

  void prepareModule();

  void computeVariables();
};

#endif // PT_ANALYSIS_HPP
