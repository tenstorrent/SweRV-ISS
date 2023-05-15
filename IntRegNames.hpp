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

  // This array can go directly in the template declaration once using C++20
  constexpr auto _getIntRegNumberToAbiNameArr()
  {
    using namespace std::string_view_literals;

    return std::array{
      "zero"sv, "ra"sv, "sp"sv,  "gp"sv,  "tp"sv, "t0"sv, "t1"sv, "t2"sv,
      "s0"sv,   "s1"sv, "a0"sv,  "a1"sv,  "a2"sv, "a3"sv, "a4"sv, "a5"sv,
      "a6"sv,   "a7"sv, "s2"sv,  "s3"sv,  "s4"sv, "s5"sv, "s6"sv, "s7"sv,
      "s8"sv,   "s9"sv, "s10"sv, "s11"sv, "t3"sv, "t4"sv, "t5"sv, "t6"sv
    };
  }


  constexpr auto _getAdditionalIntRegNameToNumberMappings()
  {
    using namespace std::string_view_literals;

    return std::array{ std::pair { "fp"sv, RegX8 } }; // Fp, s0 and x8 name the same reg.
  }


  /// Manage names of integer registers.
  class IntRegNames : public RegNamesTemplate<IntRegNumber,
                                              32,
                                              'x',
                                              _getIntRegNumberToAbiNameArr,
                                              _getAdditionalIntRegNameToNumberMappings().size(),
                                              _getAdditionalIntRegNameToNumberMappings> {};

}
