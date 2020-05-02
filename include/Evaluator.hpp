
#ifndef PT_EVALUATOR_HPP
#define PT_EVALUATOR_HPP

#include <type_traits>
#include <unordered_map>

#include <llvm/ADT/APInt.h>
#include <llvm/ADT/SmallPtrSet.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Instructions.h>

/* TODO remove this note
 * Assumption: expressions are not dependent on control flow, i.e. a reg2mem
 * pass was executed and values that cross basic blocks are stored in alloca'd
 * variables (represented by llvm::Instruction in SymbolicExpr).
 */

namespace llvm {
class BasicBlock;
// class Constant;
// class ConstantInt;
class Evaluator;
class Value;
} // namespace llvm

namespace pt {

class IntSymVar;

// MARK: === SYMBOLIC VARIABLES ===
template <class SymVarT, class ValueT,
          class = typename std::enable_if<
              std::is_base_of<llvm::Constant, ValueT>::value>::type>
class SymVar {
protected:
  const llvm::AllocaInst *Alloca;

public:
  SymVar() = delete;

protected:
  SymVar(const llvm::AllocaInst *Alloca) : Alloca(Alloca) {}

private:
  template <bool IsConst>
  class Iterator : public std::iterator<std::forward_iterator_tag, ValueT> {
    friend IntSymVar;

    const SymVarT *Variable;
    ValueT *Value;

  public:
    // Copy constructor, const -> const, nonconst -> nonconst
    Iterator(const Iterator &) = default;

    // Copy constructor, nonconst -> const
    template <bool IsConst_ = IsConst,
              class = typename std::enable_if<IsConst_>::type>
    Iterator(const Iterator<false> &RHS)
        : Variable(RHS.Variable), Value(Value) {}

  public:
    Iterator &operator++() {
      Variable->next(this);
      return *this;
    }

    ValueT &operator*() const { return *Value; }

  protected:
    Iterator(SymVarT *Variable, ValueT *Value)
        : Variable(Variable), Value(Value) {}
  };

public:
  using iterator = Iterator</*IsConst=*/false>;
  using const_iterator = Iterator</*IsConst=*/true>;

  static_assert(std::is_trivially_copy_constructible<const_iterator>(),
                "const_iterator needs to be trivially copy-constructible");

protected:
  virtual void next(iterator *Curr) = 0;
};

class IntSymVar : public SymVar<IntSymVar, llvm::ConstantInt> {
private:
  llvm::APInt Min;
  llvm::APInt Max;

public:
  IntSymVar(const llvm::AllocaInst *Alloca, llvm::APInt &Min, llvm::APInt &Max)
      : SymVar(Alloca), Min(Min), Max(Max) {
    assert(Min.getBitWidth() == Max.getBitWidth() &&
           "Bitwidth of Min and Max must be the same.");
  }

public:
  llvm::APInt const &getMin() const { return Min; }
  llvm::APInt const &getMax() const { return Max; }
  unsigned int getBitWidth() const { return Min.getBitWidth(); }

  iterator begin() {
    return iterator(this, llvm::ConstantInt::get(Alloca->getContext(), Min));
  }
  iterator end() { return iterator(this, nullptr); }

private:
  void next(iterator *Curr) override {
    if (Curr->Value->getValue().sge(Max)) {
      Curr->Value = nullptr; // Reached End
      return;
    }

    // Increment by 1
    static llvm::APInt One = llvm::APInt(64, 1, false);
    const llvm::APInt NewValue = Curr->Value->getValue().sadd_sat(One);
    Curr->Value = llvm::ConstantInt::get(Curr->Value->getContext(), NewValue);
  }
};

// MARK: === EVALUATOR ===
class Evaluator {
  llvm::SmallPtrSet<const llvm::AllocaInst *, 10> Symbolic;

public:
  Evaluator() = default;

public:
  void markSymbolic(const llvm::AllocaInst *Instr);
  void unmarkSymbolic(const llvm::AllocaInst *Instr);
  bool isSymbolic(const llvm::AllocaInst *Instr) const;

private:
  // there is one evaluator PER RUN. We want to be able to query all the
  // evaluators with specific variable values + the value that we want

  bool evaluateOnce(llvm::Evaluator *EV, llvm::BasicBlock &BB,
                    std::unordered_map<const llvm::Value *,
                                       llvm::Constant *> const &Instance) const;
};
} // namespace pt

#endif // PT_EVALUATOR_HPP
