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

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <unordered_map>

// This namespace should just be used within this file.  It is needed to allow
// template specializations which are not allowed directly in a struct/class.
// It is used to build register name constant strings at compile time.
namespace _helper
{
  // Splits an unsigned value into its digits.  The contained type
  // is an integer_sequence containing the digits in printed order.
  template <size_t V, size_t... Vs> struct _digit_sequence_helper
  {
    using type = typename _digit_sequence_helper<V / 10, V % 10, Vs...>::type;
  };
  template <> struct _digit_sequence_helper<0>
  {
    using type = std::index_sequence<0>;
  };
  template <std::size_t V, std::size_t... Vs> struct _digit_sequence_helper<0, V, Vs...>
  {
    using type = std::index_sequence<V, Vs...>;
  };
  template <std::size_t V> using digit_sequence = typename _digit_sequence_helper<V>::type;

  // Creates a compile-time string using an integer_sequence of the digits.
  // the template is only defined for integer_sequence<size_t>.
  template <typename, char...> struct _value_to_string_helper;
  template <std::size_t... Digits, char... PREFIX_CHARS>
  struct _value_to_string_helper<std::index_sequence<Digits...>, PREFIX_CHARS...>
  {
    static constexpr const char value[] = {PREFIX_CHARS..., ('0' + Digits)..., 0};
  };
  template <std::size_t V, char... PREFIX_CHARS> using value_to_string = _value_to_string_helper<digit_sequence<V>, PREFIX_CHARS...>;

  // Creates an array of compile-time register strings.  Specify the number
  // of register names to create and provide the prefix characters at the end
  template <typename, char...> struct _reg_name_array_helper;
  template <std::size_t... Vs, char... PREFIX_CHARS>
  struct _reg_name_array_helper<std::index_sequence<Vs...>, PREFIX_CHARS...>
  {
    static constexpr auto value = std::array{std::string_view(value_to_string<Vs, PREFIX_CHARS...>::value)...};
  };
  template <std::size_t V, char... PREFIX_CHARS> using make_reg_name_array = _reg_name_array_helper<std::make_index_sequence<V>, PREFIX_CHARS...>;
}

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

    static const std::unordered_map<std::string_view, RegNumberEnum> buildNameToNumberMap()
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

      if constexpr (GET_OTHER_NAME_TO_NUMBER_MAPPINGS != nullptr)
        {
          for (auto&& [name, number] : GET_OTHER_NAME_TO_NUMBER_MAPPINGS())
            {
              result[name] = number;
            }
        }

      return result;
    }

    RegNamesTemplate() = delete;

    static inline constexpr const char unknown_[]       = {PREFIX_CHAR, '?', 0};
    static inline constexpr auto       numberToName_    = _helper::make_reg_name_array<NUM_REGS, PREFIX_CHAR>::value;
    static inline const     auto       nameToNumber_    = buildNameToNumberMap();
    static inline constexpr auto       numberToAbiName_ = GET_NUMBER_TO_ABI_NAME();
  };

}
