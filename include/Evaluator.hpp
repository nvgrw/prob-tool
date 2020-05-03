
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
class Evaluator;
class Value;
class TargetLibraryInfo;
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

  const llvm::AllocaInst *getAlloca() const { return Alloca; }

protected:
  SymVar(const llvm::AllocaInst *Alloca) : Alloca(Alloca) {}

private:
  class Iterator : public std::iterator<std::forward_iterator_tag, ValueT> {
    friend IntSymVar;

    const SymVarT *Variable;
    ValueT *Value;
    using ImplT = SymVar<SymVarT, ValueT>;

  public:
    // Copy constructor, const -> const, nonconst -> nonconst
    Iterator(const Iterator &) = default;

  public:
    Iterator &operator++() {
      static_cast<const ImplT *>(Variable)->next(this);
      return *this;
    }
    bool operator==(const Iterator &Rhs) {
      return Value == Rhs.Value && Variable == Rhs.Variable;
    }
    bool operator!=(const Iterator &Rhs) { return !(*this == Rhs); }

    ValueT *operator*() const { return Value; }

  protected:
    Iterator(const SymVarT *Variable, ValueT *Value)
        : Variable(Variable), Value(Value) {}
  };

public:
  using iterator = Iterator;

protected:
  virtual void next(iterator *Curr) const = 0;
};

class IntSymVar : public SymVar<IntSymVar, llvm::ConstantInt> {
private:
  llvm::APInt Min;
  llvm::APInt Max;

public:
  IntSymVar(const llvm::AllocaInst *Alloca, llvm::APInt Min, llvm::APInt Max)
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
  void next(iterator *Curr) const override {
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
public:
  typedef llvm::SmallPtrSet<IntSymVar *, 10> SymbolicSetT;
  typedef std::unordered_map<const llvm::Value *, llvm::Constant *>
      EvalInstanceT;

private:
  const llvm::DataLayout &DL;
  const llvm::TargetLibraryInfo *TLI;
  SymbolicSetT Symbolic;

public:
  Evaluator(const llvm::DataLayout &DL, const llvm::TargetLibraryInfo *TLI)
      : DL(DL), TLI(TLI) {}

public:
  void markSymbolic(IntSymVar *Var);
  void unmarkSymbolic(IntSymVar *Var);
  bool isSymbolic(IntSymVar *Var) const;

  void evaluate(llvm::BasicBlock &BB);

private:
  void permuteVariablesAndExecute(unsigned VariableIndex,
                                  SymbolicSetT::iterator VarIt,
                                  llvm::BasicBlock &BB,
                                  EvalInstanceT &Instance);

  // there is one evaluator PER RUN. We want to be able to query all the
  // evaluators with specific variable values + the value that we want
  bool evaluateOnce(llvm::Evaluator *EV, llvm::BasicBlock &BB,
                    EvalInstanceT const &Instance) const;
};
} // namespace pt

#endif // PT_EVALUATOR_HPP
