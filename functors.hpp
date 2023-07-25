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

#include <algorithm>
#include <functional>

namespace WdRiscv
{

/// Function operator to compute bitwise a and not b (a & ~b).
struct MyAndn
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  { return a & ~b; }
};


/// Function operator to shift a left by b bits.
/// Only the least sig n bits of b are used, n is log2(width of T).
struct MySll
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  {
    constexpr unsigned mask = sizeof(T)*8 - 1;
    unsigned amount = b & mask;
    return a << amount;
  }
};


/// Shift right.
struct MySr
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  {
    constexpr unsigned mask = sizeof(T)*8 - 1;
    unsigned amount = b & mask;
    return a >> amount;
  }
};


/// Function operator to perform carry-less multiply of a and b.
struct MyClmul
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  {
    constexpr unsigned width = sizeof(T)*8;  // Bit count of T
    T res{0};
    for (unsigned i = 0; i < width; ++i)
      if ((b >> i) & 1)
	res ^= a << i;
    return res;
  }
};


/// Function operator to perform carry-less multiply of a and b and
/// return the upper w bits of the product. W is the width of T.
struct MyClmulh
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  {
    constexpr unsigned width = sizeof(T)*8;  // Bit count of T
    T res{0};
    for (unsigned i = 0; i < width; ++i)
      if ((b >> i) & 1)
	res ^= (a >> (width - i));
    return res;
  }
};


struct MyRsub
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  { return std::minus{}(b, a); }
};


struct MyMin
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  { return std::min<T>(a, b); }
};


struct MyMax
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  { return std::max<T>(a, b); }
};


struct MyBitNand
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  { return ~ (a & b); }
};


struct
MyBitAndNot
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  { return a & ~b; }
};


struct
MyBitNor
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  { return ~ (a | b); }
};


struct
MyBitOrNot
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  { return a | ~b; }
};


struct
MyBitXnor
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  { return ~ (a ^ b); }
};

}
