
#ifndef PT_EVALUATOR_HPP
#define PT_EVALUATOR_HPP

#include <llvm/ADT/SmallPtrSet.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constant.h>
//#include <llvm/IR/Instructions.h>
#include <llvm/IR/Value.h>

/*
 * Assumption: expressions are not dependent on control flow, i.e. a reg2mem
 * pass was executed and values that cross basic blocks are stored in alloca'd
 * variables (represented by llvm::Instruction in SymbolicExpr).
 */

namespace llvm {
class AllocaInst;
} // namespace llvm

namespace pt {

/*
enum class ExprType { Binary, Unary, Constant, Symbolic };

class Expr {
  ExprType Type;

public:
  Expr(ExprType Type) : Type(Type) {}
};

class BinaryExpr : public Expr {
  std::unique_ptr<Expr> Lhs;
  std::unique_ptr<Expr> Rhs;

public:
  BinaryExpr(std::unique_ptr<Expr> Lhs, std::unique_ptr<Expr> Rhs)
      : Expr(ExprType::Binary), Lhs(std::move(Lhs)), Rhs(std::move(Rhs)) {
    assert(Lhs != nullptr && Rhs != nullptr && "Lhs and Rhs must be non-null");
  }

  Expr &getLhs() const { return *Lhs; }

  Expr &getRhs() const { return *Rhs; }
};

class UnaryExpr : public Expr {
  std::unique_ptr<Expr> Value;

public:
  UnaryExpr(std::unique_ptr<Expr> Value)
      : Expr(ExprType::Unary), Value(std::move(Value)) {
    assert(Value != nullptr && "Value must be non-null");
  }

  Expr &getValue() const { return *Value; }
};

class ConstantExpr : public Expr {
  const llvm::Constant *Value;

public:
  ConstantExpr(const llvm::Constant *Value)
      : Expr(ExprType::Constant), Value(Value) {
    assert(Value != nullptr && "Value must be non-null");
  }

  const llvm::Constant *getValue() const { return Value; }
};

class SymbolicExpr : public Expr {
  const llvm::Instruction *Value;

public:
  SymbolicExpr(const llvm::Instruction *Value)
      : Expr(ExprType::Symbolic), Value(Value) {
    assert(Value != nullptr && "Value must be non-null");
  }

  const llvm::Instruction *getValue() const { return Value; }
};

*/
class Evaluator {
  llvm::SmallPtrSet<const llvm::AllocaInst *, 10> Symbolic;

public:
  Evaluator() = default;

public:
  void markSymbolic(const llvm::AllocaInst *Instr);
  void unmarkSymbolic(const llvm::AllocaInst *Instr);
  bool isSymbolic(const llvm::AllocaInst *Instr) const;

public:
  void evaluate(llvm::BasicBlock &BB,
                const llvm ::DataLayout &DL) const;
  //  std::unique_ptr<Expr> buildExpression(const llvm::Value *Value);
};
} // namespace pt

#endif // PT_EVALUATOR_HPP
