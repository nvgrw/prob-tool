
#ifndef PT_EVALUATOR_HPP
#define PT_EVALUATOR_HPP

#include <type_traits>
#include <unordered_map>
#include <vector>

#include <llvm/IR/Constant.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Transforms/Utils/Evaluator.h>

namespace llvm {
class BasicBlock;
class CallInst;
class TargetLibraryInfo;
class Value;
class ValueAsMetadata;
class MetadataAsValue;
} // namespace llvm

namespace pt {

class IntSymVar;

// MARK: === SYMBOLIC VARIABLES ===
template <class SymVarT, class ValueT,
          class = typename std::enable_if<
              std::is_base_of<llvm::Constant, ValueT>::value>::type>
class SymVar {
protected:
  llvm::CallInst *Decl;

public:
  SymVar() = delete;

  llvm::CallInst *getDecl() const { return Decl; }

  llvm::AllocaInst *getAlloca() const {
    return llvm::cast<llvm::AllocaInst>(
        llvm::cast<llvm::ValueAsMetadata>(
            llvm::cast<llvm::MetadataAsValue>(Decl->getOperand(0))
                ->getMetadata())
            ->getValue());
  }

protected:
  SymVar(llvm::CallInst *Decl) : Decl(Decl) {}

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
  virtual unsigned getIndexOfValue(const llvm::Constant *V) const = 0;
};

class IntSymVar : public SymVar<IntSymVar, llvm::ConstantInt> {
private:
  const llvm::APInt Min;
  const llvm::APInt Max;

public:
  IntSymVar(llvm::CallInst *Decl, llvm::APInt Min, llvm::APInt Max)
      : SymVar(Decl), Min(Min), Max(Max) {
    assert(Min.getBitWidth() == Max.getBitWidth() &&
           "Bitwidth of Min and Max must be the same.");
  }

public:
  llvm::APInt const &getMin() const { return Min; }
  llvm::APInt const &getMax() const { return Max; }
  unsigned getRange() const override {
    return static_cast<unsigned>(Max.getSExtValue() - Min.getSExtValue() + 1);
  }
  unsigned getIndexOfValue(const llvm::Constant *V) const override {
    assert(llvm::isa<llvm::ConstantInt>(V) &&
           "IntSymVar value index queried with non-ConstantInt type.");

    const llvm::ConstantInt *CI = llvm::cast<llvm::ConstantInt>(V);
    unsigned Index = (CI->getValue() - Min).getSExtValue();
    assert(Index < getRange() && "Index out of range.");
    return Index;
  }
  unsigned int getBitWidth() const { return Min.getBitWidth(); }

  iterator begin() {
    llvm::ConstantInt *Value = llvm::ConstantInt::get(Decl->getContext(), Min);
    return iterator(this, Value);
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
  const std::unordered_map<const llvm::Value *, unsigned> IndexMap;
  const std::vector<IntSymVar *> Symbolic;

  const unsigned NumStates;
  llvm::Evaluator **States = nullptr;

public:
  Evaluator(const llvm::DataLayout &DL, const llvm::TargetLibraryInfo *TLI,
            std::unordered_map<const llvm::Value *, unsigned> IndexMap,
            const std::vector<IntSymVar *> &Symbolic);

  ~Evaluator() {
    if (!States)
      return;

    for (int I = 0; I < NumStates; I++) {
      delete States[I];
    }
    delete[] States;
  }

public:
  void evaluate(llvm::BasicBlock &BB);

  llvm::Constant *getValue(std::vector<StateAddressed> const &Location,
                           llvm::Value *V) const;
  inline llvm::Constant *getValue(unsigned StateIndex, llvm::Value *V) const {
    return States[StateIndex]->getVal(V);
  }
  inline unsigned getNumStates() const { return NumStates; }
  static unsigned getNumStates(std::vector<IntSymVar *> const &Symbolic);

  std::vector<StateAddressed> getLocation(unsigned StateIndex) const;
  static std::vector<StateAddressed>
  getLocation(std::vector<IntSymVar *> const &Symbolic, unsigned StateIndex);
  unsigned getStateIndex(std::vector<StateAddressed> const &Location) const;

private:
  void
  permuteVariablesAndExecute(std::vector<IntSymVar *>::const_iterator VarIt,
                             llvm::BasicBlock &BB,
                             std::vector<InstanceElem> &Instance);

  bool evaluateOnce(llvm::Evaluator *EV, llvm::BasicBlock &BB,
                    std::vector<InstanceElem> const &Instance) const;
};
} // namespace pt

#endif // PT_EVALUATOR_HPP
