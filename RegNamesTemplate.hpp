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
#include <tuple>
#include <unordered_map>

#include "util.hpp"

namespace WdRiscv
{
  template <typename RegNumberEnum,
            std::size_t NUM_REGS,
            char PREFIX_CHAR,
            std::array<std::string_view, NUM_REGS>(*GET_NUMBER_TO_ABI_NAME)(),
            std::size_t NUM_OTHER_NAME_TO_NUMBER_MAPPINGS = 0,
            std::array<std::pair<std::string_view, RegNumberEnum>, NUM_OTHER_NAME_TO_NUMBER_MAPPINGS>(*GET_OTHER_NAME_TO_NUMBER_MAPPINGS)() = nullptr>
  class RegNamesTemplate
  {
  public:
    RegNamesTemplate() = delete;

    /// Set ix to the number of the register corresponding to the
    /// given name returning true on success and false if no such
    /// register.  For example, if name is "x2" then ix will be set to
    /// 2. If name is "tp" then ix will be set to 4.
    [[nodiscard]] static bool findReg(std::string_view name, unsigned& ix)
    {
      const auto iter = nameToNumber_.find(name);
      if (iter == nameToNumber_.end())
        return false;
      ix = iter->second;
      return true;
    }

    /// Return the name of the given register.
    static constexpr std::string_view regName(unsigned i, bool abiNames = false)
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

    static std::unordered_map<std::string_view, RegNumberEnum> buildNameToNumberMap()
    {
      std::unordered_map<std::string_view, RegNumberEnum> result;

      for (unsigned ix = 0; ix < numberToName_.size(); ++ix)
        {
          result[numberToName_.at(ix)] = RegNumberEnum(ix);
        }

      for (unsigned ix = 0; ix < numberToAbiName_.size(); ++ix)
        {
          result[numberToAbiName_.at(ix)] = RegNumberEnum(ix);
        }

      if (GET_OTHER_NAME_TO_NUMBER_MAPPINGS != nullptr)
        {
          for (auto&& [name, number] : GET_OTHER_NAME_TO_NUMBER_MAPPINGS())
            {
              result[name] = number;
            }
        }

      return result;
    }

    static inline constexpr auto unknown_arr_     = std::to_array<char>({PREFIX_CHAR, '?', 0});
    static inline constexpr auto unknown_         = std::string_view(unknown_arr_.begin(), std::prev(unknown_arr_.end()));
    static inline constexpr auto numberToName_    = util::make_reg_name_array<NUM_REGS, PREFIX_CHAR>::value;
    static inline const     auto nameToNumber_    = buildNameToNumberMap();
    static inline constexpr auto numberToAbiName_ = GET_NUMBER_TO_ABI_NAME();
  };

}
