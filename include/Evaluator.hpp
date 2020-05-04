
#ifndef PT_EVALUATOR_HPP
#define PT_EVALUATOR_HPP

#include <type_traits>
#include <unordered_map>
#include <vector>

#include <llvm/ADT/APInt.h>
#include <llvm/ADT/SmallPtrSet.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Transforms/Utils/Evaluator.h>

/* TODO remove this note
 * Assumption: expressions are not dependent on control flow, i.e. a reg2mem
 * pass was executed and values that cross basic blocks are stored in alloca'd
 * variables (represented by llvm::Instruction in SymbolicExpr).
 */

namespace llvm {
class BasicBlock;
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
  virtual unsigned getRange() const = 0;
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
  unsigned getRange() const override {
    return static_cast<unsigned>(Max.getSExtValue() - Min.getSExtValue() + 1);
  }
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
  typedef llvm::SmallVector<IntSymVar *, 10> SymbolicSetT;
  struct StateAddressed {
    unsigned Index;
    unsigned Range;
    StateAddressed() = default;
    StateAddressed(unsigned Index, unsigned Range)
        : Index(Index), Range(Range) {}
  };
  struct InstanceElem : public StateAddressed {
    llvm::Constant *Value;
    InstanceElem() = default;
    InstanceElem(unsigned Index, unsigned Range, llvm::Constant *Value)
        : StateAddressed(Index, Range), Value(Value) {}
  };

private:
  const llvm::DataLayout &DL;
  const llvm::TargetLibraryInfo *TLI;
  SymbolicSetT Symbolic;

  llvm::Evaluator **States = nullptr;
  unsigned NumStates = 0;

public:
  Evaluator(const llvm::DataLayout &DL, const llvm::TargetLibraryInfo *TLI)
      : DL(DL), TLI(TLI) {}

  ~Evaluator() {
    if (!States)
      return;

    for (int I = 0; I < NumStates; I++) {
      delete States[I];
    }
    delete[] States;
  }

public:
  void addSymbolic(IntSymVar *Var);
  //  bool isSymbolic(IntSymVar *Var) const;

  void evaluate(llvm::BasicBlock &BB);

  llvm::Constant *getValue(std::vector<StateAddressed> const &Location,
                           llvm::Value *V) const;
  inline llvm::Constant *getValue(unsigned StateIndex, llvm::Value *V) const {
    return States[StateIndex]->getVal(V);
  }
  inline unsigned getNumStates() const { return NumStates; }

  std::unique_ptr<std::vector<StateAddressed>>
  getLocation(unsigned StateIndex) const;

private:
  void permuteVariablesAndExecute(
      SymbolicSetT::iterator VarIt, llvm::BasicBlock &BB,
      std::unordered_map<const llvm::Value *, unsigned> const &IndexMap,
      std::vector<InstanceElem> &Instance);

  // there is one evaluator PER RUN. We want to be able to query all the
  // evaluators with specific variable values + the value that we want
  bool evaluateOnce(
      llvm::Evaluator *EV, llvm::BasicBlock &BB,
      std::unordered_map<const llvm::Value *, unsigned> const &IndexMap,
      std::vector<InstanceElem> const &Instance) const;
};
} // namespace pt

#endif // PT_EVALUATOR_HPP
