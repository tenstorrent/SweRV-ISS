// Copyright 2023 Western Digital Corporation or its affiliates.
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

// If using C++23 or later and std::float16_t is defined, use it.
#if defined(__STDCPP_FLOAT16_T__)
#include <stdfloat>

#define NATIVE_FP16 1

using Float16 = std::float16_t;

template <typename T>
using is_fp = std::is_floating_point<T>;

// Otherwise check for _Float16 by checking for a builtin compiler macro.
#elif defined(__FLT16_MAX__)

#define NATIVE_FP16 1

using Float16 = _Float16;

// Last resort, use a custom Float16 class
#else

#include <bit>
#include <cfenv>
#include <compare>
#include <cstdint>
#include <functional>
#include <type_traits>

// Define these macros for compatibility with older compilers
#ifndef __FLT16_MANT_DIG__
#define __FLT16_MANT_DIG__ 11
#endif

#ifndef __FLT16_DECIMAL_DIG__
#define __FLT16_DECIMAL_DIG__ 5
#endif

#ifndef __FLT16_MAX_EXP__
#define __FLT16_MAX_EXP__ 16
#endif

#ifndef __FLT16_MIN_EXP__
#define __FLT16_MIN_EXP__ -13
#endif

/// Model a half-precision floating point number.
class Float16
{
private:

  static constexpr unsigned NUM_EXPONENT_BITS    = __FLT16_DECIMAL_DIG__;
  static constexpr unsigned NUM_SIGNIFICAND_BITS = __FLT16_MANT_DIG__;
  static constexpr unsigned MAX_EXPONENT         = __FLT16_MAX_EXP__;
  static constexpr int      MIN_EXPONENT         = __FLT16_MIN_EXP__;

  static constexpr unsigned EXP_MASK = ((1 << NUM_EXPONENT_BITS) - 1);
  static constexpr unsigned SIG_MASK = ((1 << (NUM_SIGNIFICAND_BITS - 1)) - 1);

  /// Convert a float to a Float16's internal representation.
  static constexpr uint16_t bitsFromFloat(float val)
  {
    // TODO: this function does not set FP exceptions or handle rounding modes
    // correctly in all cases. This function should only be necessary when not
    // using SoftFloat, when using C++20 or previous, and when _Float is not
    // defined (GCC 11 and previous and Clang 14 and previous), so one of the
    // above options can be used to mitigate the issue.

    uint32_t ui32 = std::bit_cast<uint32_t>(val);
    bool     sign = ui32 >> 31;
    int      exp  = (ui32 >> 23) & 0xFF;
    uint32_t sig  = ui32 & 0x007FFFFF;

    if (exp == 0xFF)
      {
        if (sig)
          {
            // Is SNaN, mark as invalid and convert to QNaN
            if ((sig >> 22) == 0)
              std::feraiseexcept(FE_INVALID);

            return ((sign << 15)                              |
                    ((EXP_MASK << (NUM_SIGNIFICAND_BITS - 1)) | // Exponent should be all 1s
                    (1 << (NUM_SIGNIFICAND_BITS - 2)))        | // Upper-most bits of mantissa should be 1
                    (sig >> (std::numeric_limits<float>::digits - NUM_SIGNIFICAND_BITS)));
          }

        return (uint16_t{sign} << 15) | (uint16_t{EXP_MASK} << (NUM_SIGNIFICAND_BITS - 1));
      }

    exp  = exp - (std::numeric_limits<float>::max_exponent - MAX_EXPONENT);
    if (exp < MIN_EXPONENT)
      return uint16_t{sign} << 15;
    if (exp < 1)
      {
        // In Float16 number would be subnormal. Adjust.
        int shift = -exp;
        sig       = sig | (1 << 23);  // Put back implied most sig digit.
        sig       = sig >> (shift + 1);
        exp       = 0;
      }
    else if (exp >= static_cast<int>(EXP_MASK))
      return (uint16_t{sign} << 15) | (uint16_t{EXP_MASK} << (NUM_SIGNIFICAND_BITS - 1));

    return (uint16_t{sign} << 15) | uint16_t(exp << 10) | uint16_t(sig >> (std::numeric_limits<float>::digits - NUM_SIGNIFICAND_BITS));
  }

  /// Convert this Float16 to a float.
  constexpr float toFloat() const
  {
    bool sign = signBit();
    if (isInf())
      {
        float x = std::numeric_limits<float>::infinity();
        return sign ? -x : x;
      }

    if (isNan())
      {
        if (isSnan())
          std::feraiseexcept(FE_INVALID);
        float x = std::bit_cast<float>(0x7FC00000U | (sigBits() << (std::numeric_limits<float>::digits - NUM_SIGNIFICAND_BITS)));
        return sign ? -x : x;
      }

    uint32_t sig = sigBits();
    uint32_t exp = expBits();

    // Subnormal if exponent bits are zero and significand non-zero.
    if (exp == 0)
      {
        if (sig == 0)
          return sign ? -0.0f : 0.0f;

        // Subnormal in half precision would be normal in float.
        // Renormalize.
        unsigned shift = std::countl_zero(sig) - (std::numeric_limits<decltype(sig)>::digits - NUM_SIGNIFICAND_BITS);
        sig            = sig << shift;
        exp            = -shift;
      }

    // Update exponent for float bias.
    exp = exp + (std::numeric_limits<float>::max_exponent - MAX_EXPONENT);
    sig = sig << (std::numeric_limits<float>::digits - NUM_SIGNIFICAND_BITS);
    return std::bit_cast<float>((uint32_t{sign} << 31) + (exp << 23) + sig);
  }

  // Helper to avoid duplicating for each binary operation.
  // Needs to be defined above the uses.
  template <template <typename Operand> typename Op>
  static constexpr Float16 binaryOp(const Float16& a, const Float16& b)
  {
    Float16 result;
    result.u16 = bitsFromFloat(Op<float>{}(a.toFloat(),
                                           b.toFloat()));
    return result;
  }

public:

  /// Default constructor: value will be zero.
  constexpr Float16() = default;

  /// Convert this Float16 to another arithmetic type.
  template <typename T>
  requires std::is_arithmetic<T>::value
  explicit constexpr operator T() const { return static_cast<T>(toFloat()); }

  /// Returns the whether this Float16 is equal to to another.
  constexpr auto operator==(const Float16& other) const
  {
    return toFloat() == other.toFloat();
  }

  /// Returns the result of the comparison of this Float16 to another.
  /// Allows all other ordered operators.
  constexpr auto operator<=>(const Float16& other) const
  {
    return toFloat() <=> other.toFloat();
  }

  /// Unary minus operator.
  constexpr Float16 operator-() const
  {
    Float16 ret;
    ret.u16 = u16 xor 0x8000;
    return ret;
  }

  // Softfloat provides its own versions of these functions/operators
  // so only allow their use if not using SoftFloat.  This guard
  // prevents accidentally not using a SoftFloat when required.
#ifndef SOFT_FLOAT

  /// Construct a Float16 from an arithmetic type
  template <typename T>
  requires std::is_arithmetic<T>::value
  explicit constexpr Float16(T v) : u16(bitsFromFloat(static_cast<float>(v))) {}

  /// Binary addition operator.
  constexpr Float16 operator+(const Float16& other)
  {
    return binaryOp<std::plus>(*this, other);
  }

  /// Binary subtraction operator.
  constexpr Float16 operator-(const Float16& other)
  {
    return binaryOp<std::minus>(*this, other);
  }

  /// Binary multiplication operator.
  constexpr Float16 operator*(const Float16& other)
  {
    return binaryOp<std::multiplies>(*this, other);
  }

  /// Binary division operator.
  constexpr Float16 operator/(const Float16& other)
  {
    return binaryOp<std::divides>(*this, other);
  }
#endif

protected:
  friend struct _fphelpers;

  /// Return the sign bit of this Float16 in the least significant
  /// bit of the result.
  constexpr unsigned signBit() const
  { return u16 >> 15; }

  /// Return the exponent bits as is (without adjusting for bias).
  constexpr unsigned expBits() const
  { return (u16 >> (NUM_SIGNIFICAND_BITS - 1)) & EXP_MASK; }

  /// Return the significand bits excluding hidden bits.
  constexpr unsigned sigBits() const
  { return u16 & SIG_MASK; }

  /// Return a Float16 with magnitude of this and sign of v.
  constexpr Float16 copySign(Float16 v) const
  { Float16 r;  r.u16 = ((u16 & 0x7fff) | (v.u16 & 0x8000)); return r; }

  /// Return true is this number encodes infinity
  constexpr bool isInf() const
  { return expBits() == EXP_MASK and sigBits() == 0; }

  /// Return true if this number encodes not-a-number.
  constexpr bool isNan() const
  { return expBits() == EXP_MASK and sigBits() != 0; }

  /// Return true if this number encodes a signaling not-a-number.
  constexpr bool isSnan() const
  { return expBits() == EXP_MASK and sigBits() != 0 and ((sigBits() >> (NUM_SIGNIFICAND_BITS - 2)) & 1) == 0; }

  /// Decomposes given floating point value num into a normalized fraction
  /// and an integral power of two.
  constexpr Float16 frexp(int* exp) const
  {
    if ((expBits() == 0 && sigBits() == 0) || isNan() || isInf())
      {
        *exp = 0;
        return *this;
      }

    uint16_t sig = sigBits();
    *exp = static_cast<int>(expBits()) - (MAX_EXPONENT - 2);
    if (expBits() == 0)
    {
      uint16_t shift = std::countl_zero(sig) - (std::numeric_limits<decltype(sig)>::digits - NUM_SIGNIFICAND_BITS);
      *exp = *exp - shift + 1;
      sig  = (sig << shift) & SIG_MASK;
    }

    Float16 ret;
    ret.u16 = (signBit() << 15) | ((MAX_EXPONENT - 2) << (NUM_SIGNIFICAND_BITS - 1)) | sig;
    return ret;
  }


private:
  uint16_t u16 = 0;
} __attribute__((packed));
static_assert(sizeof(Float16) == sizeof(uint16_t));

#endif


// If not using a native float16 type, define a helper class
// that has access to the custom Float16's protected members.
// This helper class prevents accidentally using a Float16
// member function outside of this file.
#if not defined(NATIVE_FP16)

#include <cmath>

struct _fphelpers
{
  /// Return a Float16 with magnitude of x and sign of y.
  template <typename FloatType>
  static constexpr FloatType copySign(const FloatType& x, const FloatType& y)
  { return x.copySign(y); }

  /// Return true is this number encodes infinity
  template <typename FloatType>
  static constexpr bool isInf(const FloatType& f)
  { return f.isInf(); }

  /// Return true if this number encodes not-a-number.
  template <typename FloatType>
  static constexpr bool isNan(const FloatType& f)
  { return f.isNan(); }

  /// Return the sign bit of this Float16 in the least significant
  /// bit of the result.
  template <typename FloatType>
  static constexpr unsigned signBit(const FloatType& f)
  { return f.signBit(); }

  /// Decomposes given floating point value num into a normalized fraction
  /// and an integral power of two.
  template <typename FloatType>
  static constexpr FloatType frexp(const FloatType& f, int* exp)
  { return f.frexp(exp); }

  /// Categorizes a floating point value into the following categories:
  /// zero, subnormal, normal, infinite, NAN.
  template <typename FloatType>
  static constexpr decltype(FP_NORMAL) classify(const FloatType& f)
  {
    if (f.expBits() == 0)
      {
        if (f.sigBits() == 0)
          return FP_ZERO;
        return FP_SUBNORMAL;
      }
    if (f.isInf())
      return FP_INFINITE;
    if (f.isNan())
      return FP_NAN;
    return FP_NORMAL;
  }
};

#endif


// When not using C++23 (or later), create the std::numeric_limits and
// float.h/math.c versions of some functions for float16.  This allows
// common templated behavior for all floating-point types outside of this
// file.
#if not defined(__STDCPP_FLOAT16_T__)

#include <bit>
#include <cfloat>
#include <cmath>
#include <cstdint>

template <typename T>
struct is_fp : std::bool_constant<std::is_floating_point<T>::value or std::is_same<T, Float16>::value> {};

namespace std
{
  template<>
  struct numeric_limits<Float16>
  {
    static inline constexpr auto digits       = __FLT16_MANT_DIG__;
    static inline constexpr auto max_exponent = __FLT16_MAX_EXP__;

    static constexpr auto min() noexcept
    {
      // Use raw hex here to avoid F16 literal
      return std::bit_cast<Float16>(uint16_t{0b0000'0100'0000'0000});
    }

    static constexpr auto max() noexcept
    {
      // Use raw hex here to avoid F16 literal
      return std::bit_cast<Float16>(uint16_t{0b0111'1011'1111'1111});
    }

    static constexpr auto infinity() noexcept
    {
      // Use raw hex here to avoid F16 literal
      return std::bit_cast<Float16>(uint16_t{0b0111'1100'0000'0000});
    }

    static constexpr auto quiet_NaN() noexcept
    {
      // Use raw hex here to avoid F16 literal
      return std::bit_cast<Float16>(uint16_t{0b0111'1110'0000'0000});
    }

    static constexpr auto signaling_NaN() noexcept
    {
      // Use raw hex here to avoid F16 literal
      return std::bit_cast<Float16>(uint16_t{0b0111'1101'0000'0000});
    }
  };

  inline auto copysign(Float16 a, Float16 b)
  {
#ifndef NATIVE_FP16
    return _fphelpers::copySign(a, b);
#else
    return static_cast<Float16>(std::copysignf(static_cast<float>(a),
                                               static_cast<float>(b)));
#endif
  }

  inline auto fmax(Float16 a, Float16 b)
  {
#ifndef NATIVE_FP16
    if (_fphelpers::isNan(a))
      return b;
    if (_fphelpers::isNan(b))
      return a;
    return a >= b ? a : b;
#else
    return static_cast<Float16>(std::fmaxf(static_cast<float>(a),
                                           static_cast<float>(b)));
#endif
  }

  inline auto fmin(Float16 a, Float16 b)
  {
#ifndef NATIVE_FP16
    if (_fphelpers::isNan(a))
      return b;
    if (_fphelpers::isNan(b))
      return a;
    return a <= b ? a : b;
#else
    return static_cast<Float16>(std::fminf(static_cast<float>(a),
                                           static_cast<float>(b)));
#endif
  }

  inline decltype(FP_NORMAL) fpclassify(Float16 v)
  {
#ifndef NATIVE_FP16
    return _fphelpers::classify(v);
#else
    return std::fpclassify(static_cast<float>(v));
#endif
  }

  inline auto frexp(Float16 v, int* p)
  {
#ifndef NATIVE_FP16
    return _fphelpers::frexp(v, p);
#else
    return static_cast<Float16>(std::frexp(static_cast<float>(v), p));
#endif
  }

  inline bool isinf(Float16 v)
  {
#ifndef NATIVE_FP16
    return _fphelpers::isInf(v);
#else
    return std::isinf(static_cast<float>(v));
#endif
  }

  inline bool isnan(Float16 v)
  {
#ifndef NATIVE_FP16
    return _fphelpers::isNan(v);
#else
    return std::isnan(static_cast<float>(v));
#endif
  }

  inline bool signbit(Float16 v)
  {
#ifndef NATIVE_FP16
    return _fphelpers::signBit(v);
#else
    return std::signbit(static_cast<float>(v));
#endif
  }

// SoftFloat has its own versions of these files, so don't allow them to be called
// when using SoftFloat
#ifndef SOFT_FLOAT
  inline auto fma(Float16 a, Float16 b, Float16 c)
  {
    return static_cast<Float16>(std::fmaf(static_cast<float>(a),
                                          static_cast<float>(b),
                                          static_cast<float>(c)));
  }

  inline auto sqrt(Float16 v)
  {
    return static_cast<Float16>(static_cast<float>(v));
  }
#endif
}

#endif
