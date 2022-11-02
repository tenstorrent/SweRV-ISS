// Copyright 2022 Tenstorrent Corporation or its affiliates.
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
#include <vector>
#include <string>
#include <unordered_map>
#include <type_traits>
#include <cassert>

namespace WdRiscv
{

    /// Symbolic names of the integer registers.
    enum IntRegNumber
      {
	RegX0 = 0,
	RegX1 = 1,
	RegX2 = 2,
	RegX3 = 3,
	RegX4 = 4,
	RegX5 = 5,
	RegX6 = 6,
	RegX7 = 7,
	RegX8 = 8,
	RegX9 = 9,
	RegX10 = 10,
	RegX11 = 11,
	RegX12 = 12,
	RegX13 = 13,
	RegX14 = 14,
	RegX15 = 15,
	RegX16 = 16,
	RegX17 = 17,
	RegX18 = 18,
	RegX19 = 19,
	RegX20 = 20,
	RegX21 = 21,
	RegX22 = 22,
	RegX23 = 23,
	RegX24 = 24,
	RegX25 = 25,
	RegX26 = 26,
	RegX27 = 27,
	RegX28 = 28,
	RegX29 = 29,
	RegX30 = 30,
	RegX31 = 31,
	RegZero = RegX0,
	RegRa = RegX1, // return address
	RegSp = RegX2, // stack pointer
	RegGp = RegX3, // global pointer
	RegTp = RegX4, // thread pointer
	RegFp = RegX8, // frame pointer
	RegS0 = RegX8, // Callee saved registers
	RegS1 = RegX9, 
	RegA0 = RegX10, // Call arguments (caller save)
	RegA1 = RegX11,
	RegA2 = RegX12,
	RegA3 = RegX13,
	RegA4 = RegX14,
	RegA5 = RegX15,
	RegA6 = RegX16,
	RegA7 = RegX17,
	RegS2 = RegX18,  // Callee saved registers.
	RegS3 = RegX19,
	RegS4 = RegX20,
	RegS5 = RegX21,
	RegS6 = RegX22,
	RegS7 = RegX23,
	RegS8 = RegX24,
	RegS9 = RegX25,
	RegS10 = RegX26,
	RegS11 = RegX27,
	RegT0 = RegX5, // temporary
	RegT1 = RegX6,
	RegT2 = RegX7,
	RegT3 = RegX28,
	RegT4 = RegX29,
	RegT5 = RegX30,
	RegT6 = RegX31
      };


  /// Manage names of interger registers.
  class IntRegNames
  {
  public:

    /// Constructor
    IntRegNames()
    {
      numberToName_.resize(32);

      for (unsigned ix = 0; ix < 32; ++ix)
	{
	  std::string name = "x" + std::to_string(ix);
	  nameToNumber_[name] = IntRegNumber(ix);
	  numberToName_[ix] = name;
	}

      numberToAbiName_ = { "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2",
			   "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
			   "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7",
			   "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6" };

      for (unsigned ix = 0; ix < 32; ++ix)
	{
	  std::string abiName = numberToAbiName_.at(ix);
	  nameToNumber_[abiName] = IntRegNumber(ix);
	}

      nameToNumber_["fp"] = RegX8;   // Fp, s0 and x8 name the same reg.
    }

    /// Destructor.
    ~IntRegNames()
    { }
    
    /// Set ix to the number of the register corresponding to the
    /// given name returning true on success and false if no such
    /// register.  For example, if name is "x2" then ix will be set to
    /// 2. If name is "tp" then ix will be set to 4.
    [[nodiscard]] bool findReg(const std::string& name, unsigned& ix) const
    {
      const auto iter = nameToNumber_.find(name);
      if (iter == nameToNumber_.end())
	return false;
      ix = iter->second;
      return true;
    }

    /// Return the name of the given register.
    const std::string& regName(unsigned i, bool abiNames = false) const
    {
      if (abiNames)
	{
	  if (i < numberToAbiName_.size())
	    return numberToAbiName_[i];
	  return unknown_;
	}
      if (i < numberToName_.size())
	return numberToName_[i];
      return unknown_;
    }

  private:

    const std::string unknown_ = std::string("x?");
    std::unordered_map<std::string, IntRegNumber> nameToNumber_;
    std::vector<std::string> numberToAbiName_;
    std::vector<std::string> numberToName_;
  };
}
