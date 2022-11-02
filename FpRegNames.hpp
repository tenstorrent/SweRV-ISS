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

#include <vector>
#include <unordered_map>
#include <string>


namespace WdRiscv
{

  /// Symbolic names of the integer registers.
  enum FpRegNumber
    {
      RegF0   = 0,
      RegF1   = 1,
      RegF2   = 2,
      RegF3   = 3,
      RegF4   = 4,
      RegF5   = 5,
      RegF6   = 6,
      RegF7   = 7,
      RegF8   = 8,
      RegF9   = 9,
      RegF10  = 10,
      RegF11  = 11,
      RegF12  = 12,
      RegF13  = 13,
      RegF14  = 14,
      RegF15  = 15,
      RegF16  = 16,
      RegF17  = 17,
      RegF18  = 18,
      RegF19  = 19,
      RegF20  = 20,
      RegF21  = 21,
      RegF22  = 22,
      RegF23  = 23,
      RegF24  = 24,
      RegF25  = 25,
      RegF26  = 26,
      RegF27  = 27,
      RegF28  = 28,
      RegF29  = 29,
      RegF30  = 30,
      RegF31  = 31,
      RegFt0  = RegF0,
      RegFt1  = RegF1,
      RegFt2  = RegF2,
      RegFt3  = RegF3,
      RegFt4  = RegF4,
      RegFt5  = RegF5,
      RegFt6  = RegF6,
      RegFt7  = RegF7,
      RegFs0  = RegF8,
      RegFs1  = RegF9,
      RegFa0  = RegF10,
      RegFa1  = RegF11,
      RegFa2  = RegF12,
      RegFa3  = RegF13,
      RegFa4  = RegF14,
      RegFa5  = RegF15,
      RegFa6  = RegF16,
      RegFa7  = RegF17,
      RegFs2  = RegF18,
      RegFs3  = RegF19,
      RegFs4  = RegF20,
      RegFs5  = RegF21,
      RegFs6  = RegF22,
      RegFs7  = RegF23,
      RegFs8  = RegF24,
      RegFs9  = RegF25,
      RegFs10 = RegF26,
      RegFs11 = RegF27,
      RegFt8  = RegF28,
      RegFt9  = RegF29,
      RegFt10 = RegF30,
      RegFt11 = RegF31
    };


  /// Manage floating point register names.
  class FpRegNames
  {
  public:

    /// Constructor.
    FpRegNames()
    {
      numberToName_.resize(32);

      for (unsigned ix = 0; ix < 32; ++ix)
	{
	  std::string name = "f" + std::to_string(ix);
	  nameToNumber_[name] = FpRegNumber(ix);
	  numberToName_.at(ix) = name;
	}

      numberToAbiName_ = { "ft0", "ft1", "ft2", "ft3", "ft4", "ft5", "ft6", "ft7",
			   "fs0", "fs1", "fa0", "fa1", "fa2", "fa3", "fa4", "fa5",
			   "fa6", "fa7", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7",
			   "fs8", "fs9", "fs10", "fs11", "ft8", "ft9", "ft10", "ft11" };

      for (unsigned ix = 0; ix < 32; ++ix)
	{
	  std::string abiName = numberToAbiName_.at(ix);
	  nameToNumber_[abiName] = FpRegNumber(ix);
	}
    }

    /// Destructor.
    ~FpRegNames()
    { }
    
    /// Set ix to the number of the register corresponding to the
    /// given name returning true on success and false if no such
    /// register.  For example, if name is "f2" then ix will be set to
    /// 2. If name is "fa0" then ix will be set to 10.
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

    const std::string unknown_ = std::string("f?");
    std::unordered_map<std::string, FpRegNumber> nameToNumber_;
    std::vector<std::string> numberToAbiName_;
    std::vector<std::string> numberToName_;
  };

}
