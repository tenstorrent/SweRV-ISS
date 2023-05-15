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

#include <string_view>

#include "RegNamesTemplate.hpp"

namespace WdRiscv
{

  /// Symbolic names of the floating-point registers.
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


  constexpr auto _getFpRegNumberToAbiNameArr()
  {
    using namespace std::string_view_literals;

    return std::array{
      "ft0"sv, "ft1"sv, "ft2"sv,  "ft3"sv,  "ft4"sv, "ft5"sv, "ft6"sv,  "ft7"sv,
      "fs0"sv, "fs1"sv, "fa0"sv,  "fa1"sv,  "fa2"sv, "fa3"sv, "fa4"sv,  "fa5"sv,
      "fa6"sv, "fa7"sv, "fs2"sv,  "fs3"sv,  "fs4"sv, "fs5"sv, "fs6"sv,  "fs7"sv,
      "fs8"sv, "fs9"sv, "fs10"sv, "fs11"sv, "ft8"sv, "ft9"sv, "ft10"sv, "ft11"sv
    };
  }


  /// Manage floating point register names.
  class FpRegNames : public RegNamesTemplate<FpRegNumber,
                                             32,
                                             'f',
                                             _getFpRegNumberToAbiNameArr> {};

}
