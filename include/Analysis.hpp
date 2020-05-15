#ifndef PT_ANALYSIS_HPP
#define PT_ANALYSIS_HPP

#include <unordered_map>

#include <Eigen/Dense>

#include "Evaluator.hpp"

namespace llvm {
class AllocaInst;
class Value;
} // namespace llvm

namespace pt {
class Evaluator;
}

// todo: put in pt namespace
class Analysis {
private:
  llvm::LLVMContext Context;
  const std::unique_ptr<llvm::Module> &Module;

  std::vector<pt::IntSymVar *> Variables;
  std::unordered_map<const llvm::Value *, unsigned> ValToVarIndex;

  std::map<const llvm::Value *, unsigned> Labels;
  std::map<const llvm::BasicBlock *, unsigned> BasicBlockLabels;
  unsigned MaxLabels = 0;

public:
  Analysis(std::unique_ptr<llvm::Module> const &Module);
  ~Analysis() {
    for (auto *Variable : Variables) {
      delete Variable;
    }
  }

  Eigen::MatrixXd run();
  void printLabeled();

private:
  bool hasLabel(const llvm::Value *Inst) const;
  bool isLabelable(const llvm::BasicBlock *BB,
                   const llvm::Instruction *Inst) const;

private:
  void prepareModule();
  void computeLabels();
  void computeVariables();
  std::vector<std::vector<Eigen::MatrixXd>> translateTransforms();
  void translateInstruction(pt::Evaluator const &Evaluator,
                            std::vector<std::vector<Eigen::MatrixXd>> &Matrices,
                            llvm::Instruction const *Instruction);
  Eigen::MatrixXd computeMatrix(
      std::vector<std::vector<Eigen::MatrixXd>> const &Matrices) const;
};

#endif // PT_ANALYSIS_HPP
