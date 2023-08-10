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

// Otherwise check for _Float16 by checking for a builtin compiler macro.
#elif defined(__FLT16_MAX__)

#include <cmath>

#define NATIVE_FP16 1

using Float16 = _Float16;

// Floating point utilities are not defined for the internal type, so
// create simple wrappers around float here.
namespace std
{

inline auto copysign(Float16 a, Float16 b)
{
  return __builtin_copysignf16(a, b);
}

inline auto fmax(Float16 a, Float16 b)
{
  return static_cast<Float16>(std::fmaxf(static_cast<float>(a),
                                         static_cast<float>(b)));
}

inline auto fmin(Float16 a, Float16 b)
{
  return static_cast<Float16>(std::fminf(static_cast<float>(a),
                                         static_cast<float>(b)));
}

inline decltype(FP_NORMAL) fpclassify(Float16 v)
{
  // Can't static cast to float here because a subnormal number
  // may become normal upon promotion to float.
  return __builtin_fpclassify(FP_NAN, FP_INFINITE, FP_NORMAL, FP_SUBNORMAL, FP_ZERO, v);
}

inline auto frexp(Float16 v, int* p)
{
  return static_cast<Float16>(std::frexp(static_cast<float>(v), p));
}

inline bool isinf(Float16 v)
{
  return __builtin_isinf(v);
}

inline bool isnan(Float16 v)
{
  return __builtin_isnan(v);
}

inline bool signbit(Float16 v)
{
  return __builtin_signbit(v);
}

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

}

#endif


// If using C++23 or later and std::bfloat16_t is defined, use it.
#if defined(__STDCPP_BFLOAT16_T__)

// Avoid including this file twice to pacify clang-tidy
#if not defined(__STDCPP_FLOAT16_T__)
#include <stdfloat>
#endif

#define NATIVE_BF16 1

using BFloat16 = std::bfloat16_t;

// Otherwise check for _Float16 by checking for a builtin compiler macro.
#elif defined(__BFLT16_MAX__)

#if not defined(__FLT16_MAX__)
#include <cmath>
#endif

#define NATIVE_BF16 1

using BFloat16 = __bf16;

// Floating point utilities are not defined for the internal type, so
// create simple wrappers around float here.
namespace std
{

inline auto copysign(BFloat16 a, BFloat16 b)
{
  return static_cast<BFloat16>(std::copysignf(static_cast<float>(a),
                                              static_cast<float>(b)));
}

inline auto fmax(BFloat16 a, BFloat16 b)
{
  return static_cast<BFloat16>(std::fmaxf(static_cast<float>(a),
                                          static_cast<float>(b)));
}

inline auto fmin(BFloat16 a, BFloat16 b)
{
  return static_cast<BFloat16>(std::fminf(static_cast<float>(a),
                                          static_cast<float>(b)));
}

inline decltype(FP_NORMAL) fpclassify(BFloat16 v)
{
  // Can't static cast to float here because a subnormal number
  // may become normal upon promotion to float.
  return __builtin_fpclassify(FP_NAN, FP_INFINITE, FP_NORMAL, FP_SUBNORMAL, FP_ZERO, v);
}

inline auto frexp(BFloat16 v, int* p)
{
  return static_cast<BFloat16>(std::frexp(static_cast<float>(v), p));
}

inline bool isinf(BFloat16 v)
{
  return __builtin_isinf(v);
}

inline bool isnan(BFloat16 v)
{
  return __builtin_isnan(v);
}

inline bool signbit(BFloat16 v)
{
  return __builtin_signbit(v);
}

inline auto fma(BFloat16 a, BFloat16 b, BFloat16 c)
{
  return static_cast<BFloat16>(std::fmaf(static_cast<float>(a),
                                         static_cast<float>(b),
                                         static_cast<float>(c)));
}

inline auto sqrt(BFloat16 v)
{
  return static_cast<BFloat16>(static_cast<float>(v));
}

}

#endif


// Last resort, use a custom Float16 class
#if not defined(NATIVE_FP16) or not defined(NATIVE_BF16)

#include <bit>
#include <cfenv>
#include <cmath>
#include <compare>
#include <cstdint>
#include <functional>
#include <type_traits>

namespace fp16compat
{
// Forward-declare the template
template <unsigned, unsigned, int>
class Float16Template;
}

// Forward declare float.h/math.h function overloads for the float16 template.
// These forward declarations are necessary to allow the template to mark them
// as "friend" to allow their implementations to access protected members of
// the template.
namespace std
{
template <unsigned x, unsigned y, int z>
constexpr fp16compat::Float16Template<x, y, z> copysign(fp16compat::Float16Template<x, y, z>,
                                                        fp16compat::Float16Template<x, y, z>);

template <unsigned x, unsigned y, int z>
constexpr decltype(FP_NORMAL) fpclassify(fp16compat::Float16Template<x, y, z>);

template <unsigned x, unsigned y, int z>
fp16compat::Float16Template<x, y, z> frexp(fp16compat::Float16Template<x, y, z>, int*);

template <unsigned x, unsigned y, int z>
constexpr bool isinf(fp16compat::Float16Template<x, y, z>);

template <unsigned x, unsigned y, int z>
constexpr bool isnan(fp16compat::Float16Template<x, y, z>);

template <unsigned x, unsigned y, int z>
constexpr bool signbit(fp16compat::Float16Template<x, y, z>);

}

namespace fp16compat
{

/// Model a 16-bit floating point number.
template <unsigned NUM_SIGNIFICAND_BITS_,
          unsigned MAX_EXPONENT_,
          int MIN_EXPONENT_>
class Float16Template
{

// Softfloat provides its own versions of these functions/operators
// so only allow their use if not using SoftFloat.  This guard
// prevents accidentally not using a SoftFloat when required.
// Note that we still want literals/constants to be allowed, so
// mark the functionality as consteval to only allow them to be
// used at compile time.
#ifdef SOFT_FLOAT
#define CONSTEXPR_OR_CONSTEVAL consteval
#define IS_CONSTANT_EVALUATED (MAX_EXPONENT_ > 0) // Avoids warning when using std::is_constant_evaluated in consteval context
#else
#define CONSTEXPR_OR_CONSTEVAL constexpr
#define IS_CONSTANT_EVALUATED std::is_constant_evaluated()
#endif

private:

  static constexpr unsigned NUM_SIGNIFICAND_BITS = NUM_SIGNIFICAND_BITS_;
  static constexpr unsigned NUM_EXPONENT_BITS    = 16 - NUM_SIGNIFICAND_BITS_;
  static constexpr unsigned MAX_EXPONENT         = MAX_EXPONENT_;
  static constexpr int      MIN_EXPONENT         = MIN_EXPONENT_;

  static constexpr unsigned EXP_MASK = ((1 << NUM_EXPONENT_BITS) - 1);
  static constexpr unsigned SIG_MASK = ((1 << (NUM_SIGNIFICAND_BITS - 1)) - 1);

  static constexpr unsigned FLOAT_THIS_EXP_DIFF = std::numeric_limits<float>::max_exponent - MAX_EXPONENT;
  static constexpr unsigned FLOAT_THIS_SIG_DIFF = std::numeric_limits<float>::digits - NUM_SIGNIFICAND_BITS;


  /// Convert a float to a Float16's internal representation.
  static CONSTEXPR_OR_CONSTEVAL uint16_t bitsFromFloat(float val)
  {
    uint32_t ui32 = std::bit_cast<uint32_t>(val);
    bool     sign = ui32 >> 31;
    int      exp  = static_cast<int>((ui32 >> 23) & 0xFF);
    uint32_t sig  = ui32 & 0x007FFFFF;

    if (exp == 0xFF)
      {
        if (sig)
          {
            // Is SNaN, mark as invalid and convert to QNaN
            if (not IS_CONSTANT_EVALUATED and (sig >> 22) == 0)
              std::feraiseexcept(FE_INVALID);

            return ((sign << 15)                              |
                    ((EXP_MASK << (NUM_SIGNIFICAND_BITS - 1)) | // Exponent should be all 1s
                    (1 << (NUM_SIGNIFICAND_BITS - 2)))        | // Upper-most bits of mantissa should be 1
                    (sig >> FLOAT_THIS_SIG_DIFF));
          }

        return (uint16_t{sign} << 15) | (uint16_t{EXP_MASK} << (NUM_SIGNIFICAND_BITS - 1));
      }

    if (exp == 0 and sig == 0)
      return uint16_t{sign} << 15;

    sig |= static_cast<uint32_t>(exp > 0 or FLOAT_THIS_EXP_DIFF != 0) << (std::numeric_limits<float>::digits - 1);
    exp -= FLOAT_THIS_EXP_DIFF + 1;

    int      roundingMode = IS_CONSTANT_EVALUATED ? FE_TOWARDZERO : std::fegetround();
    bool     roundNearest;
    uint32_t roundIncrement;
    if (roundingMode == FE_DOWNWARD or roundingMode == FE_UPWARD or roundingMode == FE_TOWARDZERO)
      {
        roundIncrement = ((1U << FLOAT_THIS_SIG_DIFF) - 1) *
                         ((roundingMode != FE_TOWARDZERO) && (sign == (roundingMode == FE_DOWNWARD)));
        roundNearest   = false;
      }
    else
      {
        roundIncrement = (1U << (FLOAT_THIS_SIG_DIFF - 1));
        roundNearest   = (roundingMode == FE_TONEAREST);
      }
    uint32_t roundBits = sig & ((1U << FLOAT_THIS_SIG_DIFF) - 1);

    if (exp < 0)
      {
        bool isTiny = (exp < -1) or (sig + roundIncrement < (1U << (std::numeric_limits<float>::digits + 1)));
        if (sig & (1U << (std::numeric_limits<float>::digits - 1)))
          sig = (exp >= -31) ? (sig >> -exp) | ((sig << (exp & 31)) != 0) : (sig != 0);
        exp       = 0;
        roundBits = sig & ((1U << FLOAT_THIS_SIG_DIFF) - 1);
        if (not IS_CONSTANT_EVALUATED and isTiny and roundBits)
          std::feraiseexcept(FE_UNDERFLOW);
      }
    else if (exp > static_cast<int>(EXP_MASK - 2) or
             (exp == static_cast<int>(EXP_MASK - 2) and
              (sig + roundIncrement >= (1U << std::numeric_limits<float>::digits))))
      {
        if (not IS_CONSTANT_EVALUATED)
          std::feraiseexcept(FE_OVERFLOW | FE_INEXACT);
        return ((uint16_t{sign} << 15) | (uint16_t{EXP_MASK} << (NUM_SIGNIFICAND_BITS - 1))) - (not roundIncrement);
      }

    sig = (sig + roundIncrement) >> FLOAT_THIS_SIG_DIFF;
    if (not IS_CONSTANT_EVALUATED and roundBits)
      std::feraiseexcept(FE_INEXACT);
    sig &= ~static_cast<uint32_t>(not(roundBits ^ (1U << (FLOAT_THIS_SIG_DIFF - 1))) and roundNearest);
    if (not sig)
      exp = 0;

    return (static_cast<uint16_t>(sign) << 15)                       +
            static_cast<uint16_t>(exp << (NUM_SIGNIFICAND_BITS - 1)) +
            static_cast<uint16_t>(sig);
  }

  /// Convert this Float16 to a float.
  constexpr float toFloat() const
  {
    bool sign = signBit();
    if (std::isinf(*this))
      {
        float x = std::numeric_limits<float>::infinity();
        return sign ? -x : x;
      }

    if (std::isnan(*this))
      {
        if (isSnan())
          std::feraiseexcept(FE_INVALID);
        float x = std::bit_cast<float>(0x7FC00000U | (sigBits() << FLOAT_THIS_SIG_DIFF));
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
        if constexpr (FLOAT_THIS_EXP_DIFF != 0)
          {
            unsigned shift = std::countl_zero(sig) - (std::numeric_limits<decltype(sig)>::digits - NUM_SIGNIFICAND_BITS);
            sig            = sig << shift;
            exp            = -shift;
          }
      }

    // Update exponent for float bias.
    exp +=  FLOAT_THIS_EXP_DIFF;
    sig <<= FLOAT_THIS_SIG_DIFF;
    return std::bit_cast<float>((uint32_t{sign} << 31) + (exp << 23) + sig);
  }

  // Helper to avoid duplicating for each binary operation.
  // Needs to be defined above the uses.
  template <template <typename Operand> typename Op>
  static CONSTEXPR_OR_CONSTEVAL Float16Template binaryOp(const Float16Template& a, const Float16Template& b)
  {
    Float16Template result;
    result.u16 = bitsFromFloat(Op<float>{}(a.toFloat(),
                                           b.toFloat()));
    return result;
  }

public:

  /// Default constructor: value will be zero.
  constexpr Float16Template() = default;

  /// Convert this Float16 to another arithmetic type.
  template <typename T>
  requires std::is_arithmetic<T>::value
  explicit constexpr operator T() const { return static_cast<T>(toFloat()); }

  /// Returns the whether this Float16 is equal to to another.
  constexpr auto operator==(const Float16Template& other) const
  {
    return toFloat() == other.toFloat();
  }

  /// Returns the result of the comparison of this Float16 to another.
  /// Allows all other ordered operators.
  constexpr auto operator<=>(const Float16Template& other) const
  {
    return toFloat() <=> other.toFloat();
  }

  /// Unary minus operator.
  constexpr Float16Template operator-() const
  {
    Float16Template ret;
    ret.u16 = u16;
    // For soft-float, we want a signaling nan to stay the same so that subsequent
    // ops will set the FP flags.
#ifndef SOFT_FLOAT
    if (std::isnan(*this))
      ret.u16 |= ((EXP_MASK << (NUM_SIGNIFICAND_BITS - 1)) | // Exponent should be all 1s
                  (1 << (NUM_SIGNIFICAND_BITS - 2)));
#ifndef SOFT_FLOAT
    ret.u16 = ret.u16 xor 0x8000;
    return ret;
  }

  /// Construct a Float16 from an arithmetic type
  template <typename T>
  requires std::is_arithmetic<T>::value
  explicit CONSTEXPR_OR_CONSTEVAL Float16Template(T v) : u16(bitsFromFloat(static_cast<float>(v))) {}

  /// Binary addition operator.
  CONSTEXPR_OR_CONSTEVAL Float16Template operator+(const Float16Template& other)
  {
    return binaryOp<std::plus>(*this, other);
  }

  /// Binary subtraction operator.
  CONSTEXPR_OR_CONSTEVAL Float16Template operator-(const Float16Template& other)
  {
    return binaryOp<std::minus>(*this, other);
  }

  /// Binary multiplication operator.
  CONSTEXPR_OR_CONSTEVAL Float16Template operator*(const Float16Template& other)
  {
    return binaryOp<std::multiplies>(*this, other);
  }

  /// Binary division operator.
  CONSTEXPR_OR_CONSTEVAL Float16Template operator/(const Float16Template& other)
  {
    return binaryOp<std::divides>(*this, other);
  }

#undef CONSTEXPR_OR_CONSTEVAL
#undef IS_CONSTANT_EVALUATED

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

  /// Return true if this number encodes a signaling not-a-number.
  constexpr bool isSnan() const
  { return expBits() == EXP_MASK and sigBits() != 0 and ((sigBits() >> (NUM_SIGNIFICAND_BITS - 2)) & 1) == 0; }

  friend Float16Template std::copysign<NUM_SIGNIFICAND_BITS_, MAX_EXPONENT_, MIN_EXPONENT_>(Float16Template, Float16Template);

  friend decltype(FP_NORMAL) std::fpclassify<NUM_SIGNIFICAND_BITS_, MAX_EXPONENT_, MIN_EXPONENT_>(Float16Template);

  friend Float16Template std::frexp<NUM_SIGNIFICAND_BITS_, MAX_EXPONENT_, MIN_EXPONENT_>(Float16Template, int*);

  friend bool std::isinf<NUM_SIGNIFICAND_BITS_, MAX_EXPONENT_, MIN_EXPONENT_>(Float16Template);

  friend bool std::isnan<NUM_SIGNIFICAND_BITS_, MAX_EXPONENT_, MIN_EXPONENT_>(Float16Template);

  friend bool std::signbit<NUM_SIGNIFICAND_BITS_, MAX_EXPONENT_, MIN_EXPONENT_>(Float16Template);

private:
  uint16_t u16 = 0;
} __attribute__((packed));

}

namespace std
{

template <unsigned x, unsigned y, int z>
constexpr fp16compat::Float16Template<x, y, z> copysign(fp16compat::Float16Template<x, y, z> man,
                                                        fp16compat::Float16Template<x, y, z> sign)
{
  fp16compat::Float16Template<x, y, z> r;
  r.u16 = ((man.u16 & 0x7fff) | (sign.u16 & 0x8000));
  return r;
}

template <unsigned x, unsigned y, int z>
constexpr fp16compat::Float16Template<x, y, z> fmax(fp16compat::Float16Template<x, y, z> a,
                                                    fp16compat::Float16Template<x, y, z> b)
{
  if (std::isnan(a))
    return b;
  if (std::isnan(b))
    return a;
  return a >= b ? a : b;
}

template <unsigned x, unsigned y, int z>
constexpr fp16compat::Float16Template<x, y, z> fmin(fp16compat::Float16Template<x, y, z> a,
                                                    fp16compat::Float16Template<x, y, z> b)
{
  if (std::isnan(a))
    return b;
  if (std::isnan(b))
    return a;
  return a <= b ? a : b;
}

template <unsigned x, unsigned y, int z>
constexpr decltype(FP_NORMAL) fpclassify(fp16compat::Float16Template<x, y, z> f)
{
  if (f.expBits() == 0)
    {
      if (f.sigBits() == 0)
        return FP_ZERO;
      return FP_SUBNORMAL;
    }
  if (std::isinf(f))
    return FP_INFINITE;
  if (std::isnan(f))
    return FP_NAN;
  return FP_NORMAL;
}

template <unsigned x, unsigned y, int z>
inline fp16compat::Float16Template<x, y, z> frexp(fp16compat::Float16Template<x, y, z> v, int* p)
{
  if ((v.expBits() == 0 and v.sigBits() == 0) or std::isnan(v) or std::isinf(v))
    {
      *p = 0;
      return v;
    }

  uint16_t sig = v.sigBits();
  *p           = static_cast<int>(v.expBits()) - (decltype(v)::MAX_EXPONENT - 2);
  if (v.expBits() == 0)
    {
      uint16_t shift = std::countl_zero(sig) - (std::numeric_limits<decltype(sig)>::digits - decltype(v)::NUM_SIGNIFICAND_BITS);
      *p             = *p - shift + 1;
      sig            = (sig << shift) & decltype(v)::SIG_MASK;
    }

  fp16compat::Float16Template<x, y, z> ret;
  ret.u16 = (v.signBit() << 15) | ((decltype(v)::MAX_EXPONENT - 2) << (decltype(v)::NUM_SIGNIFICAND_BITS - 1)) | sig;
  return ret;
}

template <unsigned x, unsigned y, int z>
constexpr bool isinf(fp16compat::Float16Template<x, y, z> v)
{
  return v.expBits() == decltype(v)::EXP_MASK and v.sigBits() == 0;
}

template <unsigned x, unsigned y, int z>
constexpr bool isnan(fp16compat::Float16Template<x, y, z> v)
{
  return v.expBits() == decltype(v)::EXP_MASK and v.sigBits() != 0;
}

template <unsigned x, unsigned y, int z>
constexpr bool signbit(fp16compat::Float16Template<x, y, z> v)
{
  return v.signBit();
}

// SoftFloat has its own versions of these files, so don't allow them to be called
// when using SoftFloat
#ifndef SOFT_FLOAT
template <unsigned x, unsigned y, int z>
inline auto fma(fp16compat::Float16Template<x, y, z> a,
                fp16compat::Float16Template<x, y, z> b,
                fp16compat::Float16Template<x, y, z> c)
{
  return static_cast<fp16compat::Float16Template<x, y, z>>(std::fmaf(static_cast<float>(a),
                                                                     static_cast<float>(b),
                                                                     static_cast<float>(c)));
}

template <unsigned x, unsigned y, int z>
inline fp16compat::Float16Template<x, y, z> sqrt(fp16compat::Float16Template<x, y, z> v)
{
  return static_cast<fp16compat::Float16Template<x, y, z>>(static_cast<float>(v));
}
#endif

}

#endif


#if not defined(NATIVE_FP16)

// Define these macros for compatibility with older compilers
#ifndef __FLT16_MANT_DIG__
#define __FLT16_MANT_DIG__ (11)  // NOLINT(bugprone-reserved-identifier)
#endif

#ifndef __FLT16_MAX_EXP__
#define __FLT16_MAX_EXP__ (16)  // NOLINT(bugprone-reserved-identifier)
#endif

#ifndef __FLT16_MIN_EXP__
#define __FLT16_MIN_EXP__ (-13)  // NOLINT(bugprone-reserved-identifier)
#endif

/// Model a 16-bit floating point number.
using Float16 = fp16compat::Float16Template<__FLT16_MANT_DIG__,
                                            __FLT16_MAX_EXP__,
                                            __FLT16_MIN_EXP__>;
static_assert(sizeof(Float16) == sizeof(uint16_t));
#endif


#if not defined(NATIVE_BF16)

// Define these macros for compatibility with older compilers
#ifndef __BFLT16_MANT_DIG__
#define __BFLT16_MANT_DIG__ (8)  // NOLINT(bugprone-reserved-identifier)
#endif

#ifndef __BFLT16_MAX_EXP__
#define __BFLT16_MAX_EXP__ (128)  // NOLINT(bugprone-reserved-identifier)
#endif

#ifndef __BFLT16_MIN_EXP__
#define __BFLT16_MIN_EXP__ (-125)  // NOLINT(bugprone-reserved-identifier)
#endif

/// Model a 16-bit brain floating point number.
using BFloat16 = fp16compat::Float16Template<__BFLT16_MANT_DIG__,
                                             __BFLT16_MAX_EXP__,
                                             __BFLT16_MIN_EXP__>;
static_assert(sizeof(BFloat16) == sizeof(uint16_t));

#endif


// When not using C++23 (or later), create the std::numeric_limits and
// float.h/math.c versions of some functions for float16.  This allows
// common templated behavior for all floating-point types outside of this
// file.
#if not defined(__STDCPP_FLOAT16_T__)

// Avoid including these files twice to pacify clang-tidy
#if defined(NATIVE_FP16)
#include <bit>
#include <cstdint>
#endif

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

}

#endif

// When not using C++23 (or later), create the std::numeric_limits and
// float.h/math.c versions of some functions for bfloat16.  This allows
// common templated behavior for all floating-point types outside of this
// file.
#if not defined(__STDCPP_BFLOAT16_T__)

// Avoid including these files twice to pacify clang-tidy
#if defined(__STDCPP_FLOAT16_T__) and defined(NATIVE_BF16)
#include <bit>
#include <cstdint>
#endif

namespace std
{

template<>
struct numeric_limits<BFloat16>
{
  static inline constexpr auto digits       = __BFLT16_MANT_DIG__;
  static inline constexpr auto max_exponent = __BFLT16_MAX_EXP__;

  static constexpr auto min() noexcept
  {
    // Use raw hex here to avoid BF16 literal
    return std::bit_cast<BFloat16>(uint16_t{0b0000'0000'1000'0000});
  }

  static constexpr auto max() noexcept
  {
    // Use raw hex here to avoid BF16 literal
    return std::bit_cast<BFloat16>(uint16_t{0b0111'1111'0111'1111});
  }

  static constexpr auto infinity() noexcept
  {
    // Use raw hex here to avoid BF16 literal
    return std::bit_cast<BFloat16>(uint16_t{0b0111'1111'1000'0000});
  }

  static constexpr auto quiet_NaN() noexcept
  {
    // Use raw hex here to avoid BF16 literal
    return std::bit_cast<BFloat16>(uint16_t{0b0111'1111'1100'0000});
  }

  static constexpr auto signaling_NaN() noexcept
  {
    // Use raw hex here to avoid BF16 literal
    return std::bit_cast<BFloat16>(uint16_t{0b0111'1111'1010'0000});
  }
};

}

#endif

// Create a helper template to determine whether a type is a float type or not.
#if defined(__STDCPP_FLOAT16_T__) and defined(__STDCPP_BFLOAT16_T__)

// Float16 and BFloat16 are C++23 native types.  Just alias std::is_floating_point;
template <typename T>
using is_fp = std::is_floating_point<T>;

#elif defined(__STDCPP_FLOAT16_T__)

// Float16 is a C++23 native type.  Add an explicit check for BFloat16 to std::is_floating_point;
template <typename T>
struct is_fp : std::bool_constant<std::is_floating_point<T>::value or std::is_same<T, BFloat16>::value> {};

#elif defined(__STDCPP_BFLOAT16_T__)

// BFloat16 is a C++23 native type.  Add an explicit check for Float16 to std::is_floating_point;
template <typename T>
struct is_fp : std::bool_constant<std::is_floating_point<T>::value or std::is_same<T, Float16>::value> {};

#else

// Neither Float16 nor BFloat16 are C++23 native types.  Add explicity checks for both to std::is_floating_point
template <typename T>
struct is_fp : std::bool_constant<std::is_floating_point<T>::value or
                                  std::is_same<T, Float16>::value  or
                                  std::is_same<T, BFloat16>::value> {};

#endif
