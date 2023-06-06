// Copyright 2020 Western Digital Corporation or its affiliates.
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <cassert>
#include <optional>
#include <vector>
#include "IntRegNames.hpp"


namespace WdRiscv
{

  template <typename URV>
  class Hart;

  /// Model a RISCV integer register file.
  /// URV (unsigned register value) is the register value type. For
  /// 32-bit registers, URV must be uint32_t. For 64-bit integers,
  /// it must be uint64_t.
  template <typename URV>
  class IntRegs
  {
  public:

    friend class Hart<URV>;  // To access reset and last-written-reg methods.

    /// Constructor: Define a register file with the given number of
    /// registers. Each register is of type URV. All registers initialized
    /// to zero.
    IntRegs(unsigned registerCount);

    /// Destructor.
    ~IntRegs()
    { regs_.clear(); }
    
    /// Return value of ith register. Register zero always yields zero.
    URV read(unsigned i) const
    { return regs_[i]; }

    /// Set value of ith register to the given value. Setting register
    /// zero has no effect.
    void write(unsigned i, URV value)
    {
      originalValue_ = regs_[i];
      if (i != 0)
	regs_[i] = value;
      lastWrittenReg_ = i;
    }

    /// Similar to write but does not record a change.
    void poke(unsigned i, URV value)
    {
      if (i != 0)
	regs_.at(i) = value;
    }

    /// Return the count of registers in this register file.
    size_t size() const
    { return regs_.size(); }

    /// Set ix to the number of the register corresponding to the
    /// given name returning true on success and false if no such
    /// register.  For example, if name is "x2" then ix will be set to
    /// 2. If name is "tp" then ix will be set to 4.
    bool findReg(std::string_view name, unsigned& ix) const;

    /// Return the name of the given register.
    static constexpr std::string_view regName(unsigned i, bool abiNames = false)
    { return IntRegNames::regName(i, abiNames); }

  protected:

    /// Clear all regisers.
    void reset()
    {
      clearLastWrittenReg();
      for (auto& reg : regs_)
	reg = 0;
    }

    /// Clear the number denoting the last written register.
    void clearLastWrittenReg()
    { lastWrittenReg_.reset(); }

    /// Return the number of the last written register or -1 if no register has
    /// been written since the last clearLastWrittenReg.
    int getLastWrittenReg() const
    { return lastWrittenReg_.has_value() ? static_cast<int>(*lastWrittenReg_) : -1; }

    /// Similar to getLastWrittenReg but if successful set regValue to
    /// the prevous value of the last written register.
    int getLastWrittenReg(uint64_t& regValue) const
    {
      if (not lastWrittenReg_.has_value()) return -1;
      regValue = originalValue_;
      return static_cast<int>(*lastWrittenReg_);
    }

  private:

    std::vector<URV> regs_;
    std::optional<unsigned> lastWrittenReg_;  // Register accessed in most recent write.
    URV originalValue_ = 0;                   // Original value of last written reg.
  };
}
