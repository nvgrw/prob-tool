#ifndef PT_REGALLOC_HPP
#define PT_REGALLOC_HPP

#include <unordered_map>
#include <vector>

namespace llvm {
class Function;
class Module;
class Value;
} // namespace llvm

namespace pt {
class AllocContext;
} // namespace pt

namespace pt {
// Overlap not possible so we don't handle this
// TODO: may be a good idea to handle it anyway
class Interval {
public:
  std::vector<unsigned> Ranges;

public:
  Interval() = default;
  Interval(unsigned From, unsigned To) : Ranges{From, To} {}

public:
  unsigned getFrom() const {
    if (Ranges.empty())
      return 0;
    return Ranges.front();
  }
  void setFrom(unsigned From) {
    assert(!Ranges.empty() && "No ranges.");
    assert(Ranges.front() <= From && "New from results in larger range.");
    unsigned Second = *++Ranges.begin();
    assert(Second >= From && "New from falls outside first range.");
    *Ranges.begin() = From;
  }
  unsigned getTo() const {
    if (Ranges.empty())
      return 0;
    return Ranges.back();
  }
  void setTo(unsigned To) {
    assert(!Ranges.empty() && "No ranges.");
    assert(Ranges.back() >= To && "New to results in larger range.");
    unsigned Second = *++Ranges.rbegin();
    assert(Second <= To && "New to falls outside last range.");
    *Ranges.rbegin() = To;
  }

  void addRange(unsigned From, unsigned To) {
    if (Ranges.empty()) {
      Ranges.push_back(From);
      Ranges.push_back(To);
      return;
    }

    // Strict subset
    if (getTo() >= From && To >= getFrom()) {
      return;
    }

    assert((getTo() < From || To < getFrom()) &&
           "Adding range within existing range");

    // Add range to end
    if (getTo() < From) {
      Ranges.push_back(From);
      Ranges.push_back(To);
      return;
    }

    // Add range to start
    if (To < getFrom()) {
      Ranges.insert(Ranges.begin(), To);
      Ranges.insert(Ranges.begin(), From);
      return;
    }
  }
};

class RegAlloc {
  const llvm::Module &Module;

public:
  RegAlloc(const llvm::Module &Module) : Module(Module) {}

public:
  void allocate() const;

private:
  std::unique_ptr<std::unordered_map<const llvm::Value *, Interval>>
  buildIntervals(AllocContext &Ctx, const llvm::Function &Func) const;
};
} // namespace pt

#endif // PT_REGALLOC_HPP
