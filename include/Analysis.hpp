#ifndef PT_ANALYSIS_HPP
#define PT_ANALYSIS_HPP

#include <unordered_map>
#include <unordered_set>

#include <Eigen/Sparse>

#include "Evaluator.hpp"

namespace llvm {
class AllocaInst;
class Value;
} // namespace llvm

namespace pt {
class Evaluator;
} // namespace pt

namespace pt {
using SpMat = Eigen::SparseMatrix<double>;
using SpTrp = Eigen::Triplet<double>;

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

  SpMat run();
  std::tuple<Eigen::Matrix<std::string, Eigen::Dynamic, 1>, Eigen::MatrixXi>
  states();
  std::vector<SpMat>
  abstractPrints(std::unordered_set<std::string> const &Whitelist) const;
  void dumpLabeled();

private:
  bool hasLabel(const llvm::Value *Inst) const;
  bool isLabelable(const llvm::Instruction *Inst) const;

private:
  void prepareModule();
  void computeLabels();
  void computeVariables();
  std::vector<std::vector<SpMat>> translateTransforms();
  void translateInstruction(pt::Evaluator const &Evaluator,
                            std::vector<std::vector<SpMat>> &Matrices,
                            llvm::Instruction const *Instruction);
  SpMat computeMatrix(std::vector<std::vector<SpMat>> const &Matrices) const;
};
} // namespace pt

#endif // PT_ANALYSIS_HPP
