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


// This file contains the some functions for converting floats and integers
// to and from each other.

#pragma once

#include <array>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "float16-compat.hpp"

#ifdef SOFT_FLOAT
#include "softfloat-util.hpp"
#elif defined(__x86_64__)
#include <emmintrin.h>
#endif


namespace WdRiscv
{

/// RISCV floating point rounding modes.
enum class RoundingMode : uint32_t
  {
    NearestEven,     // Round to nearest, ties to even
    Zero,            // Round towards zero.
    Down,            // Round down (towards negative infinity)
    Up,              // Round up (towards positive infinity)
    NearestMax,      // Round to nearest, ties to max magnitude
    Invalid1,
    Invalid2,
    Dynamic,
    FcsrMask = 0xe0, // Mask of mode-bits in FCSR.
    FcsrShift = 5    // Index of least-significant mode bit in FCSR.
  };


/// RISCV floating point exception flags.
enum class FpFlags : uint32_t
  {
    None = 0,
    Inexact = 1,
    Underflow = 2,
    Overflow = 4,
    DivByZero = 8,
    Invalid = 16,
    FcsrMask = 0x1f   // Mask of flag-bits in the FCSR.
  };


/// Sets the floating point rounding mode in the machine
/// running this simulator. Do nothing in the simulated
/// RISCV machine.
inline int
setSimulatorRoundingMode(RoundingMode mode)
{
  // Arbitrarily use nearest-even for modes with no equivalent in
  // simulator.  The right mode will later be selected by an FP
  // instruction (if dynamic) or an exception will be trigerred (if
  // invalid).
  if (mode == RoundingMode::Dynamic or mode == RoundingMode::Invalid1 or mode == RoundingMode::Invalid2)
    mode = RoundingMode::NearestEven;

#ifdef SOFT_FLOAT
  static constexpr auto riscvToSoftFloat = std::array
    {
      softfloat_round_near_even,    // NearsetEven
      softfloat_round_minMag,       // Zero
      softfloat_round_min,          // Down
      softfloat_round_max,          // Up
      softfloat_round_near_maxMag,  // NearestMax
    };

  int      previous      = softfloat_roundingMode;
  uint32_t ix            = uint32_t(mode);
  softfloat_roundingMode = riscvToSoftFloat.at(ix);
#else
  static constexpr auto riscvToFe = std::array
    {
      FE_TONEAREST,  // NearsetEven
      FE_TOWARDZERO, // Zero
      FE_DOWNWARD,   // Down
      FE_UPWARD,     // Up
      FE_TONEAREST   // NearestMax; TODO: need a more accurate method here
    };

  int      previous = std::fegetround();
  uint32_t ix       = uint32_t(mode);
  int      next     = riscvToFe.at(ix);
  if (next != previous)
    std::fesetround(next);
#endif
  return previous;
}


/// Resets the floating point rounding mode in the machine
/// running this simulator to the value prior to the previous
/// set value (should be provided as the parameter).  Do
/// nothing in the simulated RISCV machine.
inline void undoSetSimulatorRoundingMode(int orig)
{
#ifdef SOFT_FLOAT
  softfloat_roundingMode = orig;
#else
  int previous = std::fegetround();
  if (orig != previous)
    std::fesetround(orig);
#endif
}


/// Clear the floating point flags in the machine running this
/// simulator. Do nothing in the simulated RISCV machine.
inline void
clearSimulatorFpFlags()
{
#ifdef SOFT_FLOAT
  softfloat_exceptionFlags = 0;
#elif defined(__x86_64__)
  uint32_t val = _mm_getcsr();
  val &= ~uint32_t(0x3f);
  _mm_setcsr(val);
#else
  if (fetestexcept(FE_ALL_EXCEPT) != 0)
    std::feclearexcept(FE_ALL_EXCEPT);
#endif
}


/// Gets the active floating point flags in the machine running this
/// simulator. Do nothing in the simulated RISCV machine.
inline uint32_t
activeSimulatorFpFlags()
{
  uint32_t incFlags = 0;
#ifdef SOFT_FLOAT
  int flags = softfloat_exceptionFlags;
  if (flags)
    {
      if (flags & softfloat_flag_inexact)   incFlags |= uint32_t(FpFlags::Inexact);
      if (flags & softfloat_flag_underflow) incFlags |= uint32_t(FpFlags::Underflow);
      if (flags & softfloat_flag_overflow)  incFlags |= uint32_t(FpFlags::Overflow);
      if (flags & softfloat_flag_infinite)  incFlags |= uint32_t(FpFlags::DivByZero);
      if (flags & softfloat_flag_invalid)   incFlags |= uint32_t(FpFlags::Invalid);
    }
#else
#ifdef __x86_64__
  int flags = _mm_getcsr() & 0x3f;
#else
  int flags = fetestexcept(FE_ALL_EXCEPT);
#endif
  if (flags)
    {
      if (flags & FE_INEXACT)    incFlags |= uint32_t(FpFlags::Inexact);
      if (flags & FE_UNDERFLOW)  incFlags |= uint32_t(FpFlags::Underflow);
      if (flags & FE_OVERFLOW)   incFlags |= uint32_t(FpFlags::Overflow);
      if (flags & FE_DIVBYZERO)  incFlags |= uint32_t(FpFlags::DivByZero);
      if (flags & FE_INVALID)    incFlags |= uint32_t(FpFlags::Invalid);
    }
#endif
  return incFlags;
}


/// "Raises" the provided floating point flags in the machine running
/// this simulator by or-ing them with the currently active flags. Do
/// nothing in the simulated RISCV machine.
inline void
raiseSimulatorFpFlags(FpFlags flags)
{
  using underlying_t = typename std::underlying_type<FpFlags>::type;
#ifdef SOFT_FLOAT
  if (static_cast<underlying_t>(flags) & static_cast<underlying_t>(FpFlags::Inexact))
    softfloat_exceptionFlags |= softfloat_flag_inexact;
  if (static_cast<underlying_t>(flags) & static_cast<underlying_t>(FpFlags::Underflow))
    softfloat_exceptionFlags |= softfloat_flag_underflow;
  if (static_cast<underlying_t>(flags) & static_cast<underlying_t>(FpFlags::Overflow))
    softfloat_exceptionFlags |= softfloat_flag_overflow;
  if (static_cast<underlying_t>(flags) & static_cast<underlying_t>(FpFlags::DivByZero))
    softfloat_exceptionFlags |= softfloat_flag_infinite;
  if (static_cast<underlying_t>(flags) & static_cast<underlying_t>(FpFlags::Invalid))
    softfloat_exceptionFlags |= softfloat_flag_invalid;
#else
  decltype(FE_INEXACT) flagsToRaise = 0;
  if (static_cast<underlying_t>(flags) & static_cast<underlying_t>(FpFlags::Inexact))
    flagsToRaise |= FE_INEXACT;
  if (static_cast<underlying_t>(flags) & static_cast<underlying_t>(FpFlags::Underflow))
    flagsToRaise |= FE_UNDERFLOW;
  if (static_cast<underlying_t>(flags) & static_cast<underlying_t>(FpFlags::Overflow))
    flagsToRaise |= FE_OVERFLOW;
  if (static_cast<underlying_t>(flags) & static_cast<underlying_t>(FpFlags::DivByZero))
    flagsToRaise |= FE_DIVBYZERO;
  if (static_cast<underlying_t>(flags) & static_cast<underlying_t>(FpFlags::Invalid))
    flagsToRaise |= FE_INVALID;
  std::feraiseexcept(flagsToRaise);
#endif
}


/// Converts an integer value to a floating point value.  The
/// destination floating point type must be specified in the
/// call, but the source integer type is inferred from the
/// parameter.
template <typename To, typename From>
auto fpConvertTo(From x)
  -> typename std::enable_if<is_fp<To>::value && std::numeric_limits<From>::is_integer, To>::type
{
#ifdef SOFT_FLOAT

  using namespace WdRiscv;

  if constexpr (std::is_same<From, int8_t>::value || std::is_same<From, int16_t>::value)
    return fpConvertTo<To>(static_cast<int32_t>(x));
  else if constexpr (std::is_same<From, uint8_t>::value || std::is_same<From, uint16_t>::value)
    return fpConvertTo<To>(static_cast<uint32_t>(x));
  else if constexpr (std::is_same<From, int32_t>::value)
    {
      if constexpr (std::is_same<To, Float16>::value)
        return softToNative(i32_to_f16(x));
      else if constexpr (std::is_same<To, float>::value)
        return softToNative(i32_to_f32(x));
      else if constexpr (std::is_same<To, double>::value)
        return softToNative(i32_to_f64(x));
      else
        static_assert(!sizeof(To), "Unknown destination float type");
    }
  else if constexpr (std::is_same<From, uint32_t>::value)
    {
      if constexpr (std::is_same<To, Float16>::value)
        return softToNative(ui32_to_f16(x));
      else if constexpr (std::is_same<To, float>::value)
        return softToNative(ui32_to_f32(x));
      else if constexpr (std::is_same<To, double>::value)
        return softToNative(ui32_to_f64(x));
      else
        static_assert(!sizeof(To), "Unknown destination float type");
    }
  else if constexpr (std::is_same<From, int64_t>::value)
    {
      if constexpr (std::is_same<To, Float16>::value)
        return softToNative(i64_to_f16(x));
      else if constexpr (std::is_same<To, float>::value)
        return softToNative(i64_to_f32(x));
      else if constexpr (std::is_same<To, double>::value)
        return softToNative(i64_to_f64(x));
      else
        static_assert(!sizeof(To), "Unknown destination float type");
    }
  else if constexpr (std::is_same<From, uint64_t>::value)
    {
      if constexpr (std::is_same<To, Float16>::value)
        return softToNative(ui64_to_f16(x));
      else if constexpr (std::is_same<To, float>::value)
        return softToNative(ui64_to_f32(x));
      else if constexpr (std::is_same<To, double>::value)
        return softToNative(ui64_to_f64(x));
      else
        static_assert(!sizeof(To), "Unknown destination float type");
    }
  else
    static_assert(!sizeof(From), "Unknown source integer type");

#else

  return static_cast<To>(x);

#endif

}


/// Converts a floating point value to an integer value.  The
/// destination integer type must be specified in the call,
/// but the source floating point type is inferred from the
/// parameter.
template <typename To, typename From>
auto fpConvertTo(From x)
  -> typename std::enable_if<std::numeric_limits<To>::is_integer && is_fp<From>::value, To>::type
{
#ifdef SOFT_FLOAT

  using namespace WdRiscv;

  if constexpr (std::is_same<To, int8_t>::value || std::is_same<To, uint8_t>::value ||
                std::is_same<To, int16_t>::value || std::is_same<To, uint16_t>::value)
    {
      // Softfloat doesn't have conversion functions for int8_t, uint8_t,
      // int16_t, or uint16_t, so wrap them here.
      auto old_flags = softfloat_exceptionFlags;
      auto result    = fpConvertTo<typename std::conditional<std::is_unsigned<To>::value, uint32_t, int32_t>::type>(x);
      if (result > std::numeric_limits<To>::max())
        {
          softfloat_exceptionFlags = old_flags | softfloat_flag_invalid;
          return std::numeric_limits<To>::max();
        }
      if (std::is_signed<To>::value && result < std::numeric_limits<To>::min())
        {
          softfloat_exceptionFlags = old_flags | softfloat_flag_invalid;
          return std::numeric_limits<To>::min();
        }
      return result;
    }
  else if constexpr (std::is_same<To, int32_t>::value)
    {
      if constexpr (std::is_same<From, Float16>::value)
        return f16_to_i32(nativeToSoft(x), softfloat_roundingMode, true);
      else if constexpr (std::is_same<From, float>::value)
        return f32_to_i32(nativeToSoft(x), softfloat_roundingMode, true);
      else if constexpr (std::is_same<From, double>::value)
        return f64_to_i32(nativeToSoft(x), softfloat_roundingMode, true);
      else
        static_assert(!sizeof(From), "Unknown source float type");
    }
  else if constexpr (std::is_same<To, uint32_t>::value)
    {
      if constexpr (std::is_same<From, Float16>::value)
        return f16_to_ui32(nativeToSoft(x), softfloat_roundingMode, true);
      else if constexpr (std::is_same<From, float>::value)
        return f32_to_ui32(nativeToSoft(x), softfloat_roundingMode, true);
      else if constexpr (std::is_same<From, double>::value)
        return f64_to_ui32(nativeToSoft(x), softfloat_roundingMode, true);
      else
        static_assert(!sizeof(From), "Unknown source float type");
    }
  else if constexpr (std::is_same<To, int64_t>::value)
    {
      if constexpr (std::is_same<From, Float16>::value)
        return f16_to_i64(nativeToSoft(x), softfloat_roundingMode, true);
      else if constexpr (std::is_same<From, float>::value)
        return f32_to_i64(nativeToSoft(x), softfloat_roundingMode, true);
      else if constexpr (std::is_same<From, double>::value)
        return f64_to_i64(nativeToSoft(x), softfloat_roundingMode, true);
      else
        static_assert(!sizeof(From), "Unknown source float type");
    }
  else if constexpr (std::is_same<To, uint64_t>::value)
    {
      if constexpr (std::is_same<From, Float16>::value)
        return f16_to_ui64(nativeToSoft(x), softfloat_roundingMode, true);
      else if constexpr (std::is_same<From, float>::value)
        return f32_to_ui64(nativeToSoft(x), softfloat_roundingMode, true);
      else if constexpr (std::is_same<From, double>::value)
        return f64_to_ui64(nativeToSoft(x), softfloat_roundingMode, true);
      else
        static_assert(!sizeof(From), "Unknown source float type");
    }
  else
    static_assert(!sizeof(To), "Unknown destination integer type");

#else

  double working = static_cast<double>(x);

  To   result;
  bool valid = false;
  bool exact = true;

  constexpr To MIN_TO = std::numeric_limits<To>::min();
  constexpr To MAX_TO = std::numeric_limits<To>::max();

  unsigned signBit = std::signbit(working);
  if (std::isinf(working))
    {
      if (signBit)
        result = MIN_TO;
      else
        result = MAX_TO;
    }
  else if (std::isnan(working))
    result = MAX_TO;
  else
    {
      double near = std::nearbyint(working);
      if (near == 0)
        {
          result = 0;
          valid  = true;
          exact  = near == working;
        }
      // Using "near > MAX_TO" may not work beacuse of rounding.
      else if (near >= 2 * double(static_cast<To>(1) << (std::numeric_limits<To>::digits - 1)))
        result = MAX_TO;
      else if (near < MIN_TO)
        result = MIN_TO;
      else
        {
          // std::lprint will produce an overflow if most sig bit
          // of result is 1 (it thinks there's an overflow).  We
          // compensate with the divide multiply by 2.
          if (working < (uint64_t(1) << 63))
            result = std::llrint(working);
          else
            {
              result = std::llrint(working / 2);
              result *= 2;
            }
          valid = true;
          exact = near == working;
        }
    }

  int newFlags = (FE_INVALID * not valid) | (FE_INEXACT * not exact);
  if (newFlags)
    std::feraiseexcept(newFlags);

  return result;

#endif
}


/// Converts a floating point value to an different floating
/// point type.  The destination floating point type must be
/// specified in the call, but the source floating point type
// is inferred from the parameter.
template <typename To, bool CANONICALIZE_NAN, typename From>
auto fpConvertTo(From x)
  -> typename std::enable_if<is_fp<To>::value && is_fp<From>::value, To>::type
{
#ifdef SOFT_FLOAT

  using namespace WdRiscv;

  if constexpr (std::is_same<To, Float16>::value)
    {
      To result;
      if constexpr (std::is_same<From, Float16>::value)
        result = x;
      else if constexpr (std::is_same<From, float>::value)
        result = softToNative(f32_to_f16(nativeToSoft(x)));
      else if constexpr (std::is_same<From, double>::value)
        result = softToNative(f64_to_f16(nativeToSoft(x)));
      else
        static_assert(!sizeof(To), "Unknown destination float type");
      return result;
    }
  else if constexpr (std::is_same<To, BFloat16>::value)
    {
      To result;
      if constexpr (std::is_same<From, BFloat16>::value)
        result = x;
      else if constexpr (std::is_same<From, float>::value)
        result = softToNative(f32_to_bf16(nativeToSoft(x)));
      else
        static_assert(!sizeof(To), "Unknown destination float type");
      return result;
    }
  else if constexpr (std::is_same<To, float>::value)
    {
      To result;
      if constexpr (std::is_same<From, Float16>::value)
        result = softToNative(f16_to_f32(nativeToSoft(x)));
      else if constexpr (std::is_same<From, BFloat16>::value)
        result = softToNative(bf16_to_f32(nativeToSoft(x)));
      else if constexpr (std::is_same<From, float>::value)
        result = x;
      else if constexpr (std::is_same<From, double>::value)
        result = softToNative(f64_to_f32(nativeToSoft(x)));
      else
        static_assert(!sizeof(To), "Unknown destination float type");
      return result;
    }
  else if constexpr (std::is_same<To, double>::value)
    {
      To result;
      if constexpr (std::is_same<From, Float16>::value)
        result = softToNative(f16_to_f64(nativeToSoft(x)));
      else if constexpr (std::is_same<From, float>::value)
        result = softToNative(f32_to_f64(nativeToSoft(x)));
      else if constexpr (std::is_same<From, double>::value)
        result = x;
      else
        static_assert(!sizeof(To), "Unknown destination float type");
      return result;
    }
  else
    static_assert(!sizeof(From), "Unknown source float type");

#else

  To result = static_cast<To>(x);
  if constexpr (CANONICALIZE_NAN)
    if (std::isnan(result))
      result = std::numeric_limits<To>::quiet_NaN();
  return result;

#endif
}


/// Return the integral type that is the same width as the given
/// floating point type. For example:
///    getSameWidthIntegerType<float>::type
/// yields the type
///    int32_t.
template <typename T>
struct getSameWidthIntType;

template <> struct getSameWidthIntType<BFloat16> { using type = int16_t; };
template <> struct getSameWidthIntType<Float16>  { using type = int16_t; };
template <> struct getSameWidthIntType<float>    { using type = int32_t; };
template <> struct getSameWidthIntType<double>   { using type = int64_t; };

/// Return the unsigned integral type that is the same width as the given
/// floating point type. For example:
///    getSameWidthIntegerType<float>::type
/// yields the type
///    uint32_t.
template <typename T>
struct getSameWidthUintType;

template <> struct getSameWidthUintType<BFloat16> { using type = uint16_t; };
template <> struct getSameWidthUintType<Float16>  { using type = uint16_t; };
template <> struct getSameWidthUintType<float>    { using type = uint32_t; };
template <> struct getSameWidthUintType<double>   { using type = uint64_t; };

/// Return the floating point type that is the same width as the given
/// integer type. For example:
///    getSameWidthFloatType<int32_t>::type
/// yields the type
///    float.
template <typename T>
struct getSameWidthFloatType;

template <> struct getSameWidthFloatType<int16_t>   { using type = Float16; };
template <> struct getSameWidthFloatType<int32_t>   { using type = float; };
template <> struct getSameWidthFloatType<int64_t>   { using type = double; };
template <> struct getSameWidthFloatType<uint16_t>  { using type = Float16; };
template <> struct getSameWidthFloatType<uint32_t>  { using type = float; };
template <> struct getSameWidthFloatType<uint64_t>  { using type = double; };


/// Return true if given float is a signaling not-a-number.
template <typename T>
inline auto
isSnan(T f)
  -> typename std::enable_if<is_fp<T>::value, bool>::type
{
  using uint_fsize_t = typename getSameWidthUintType<T>::type;

  if (std::isnan(f))
    {
      uint_fsize_t u = std::bit_cast<uint_fsize_t>(f);
      return ((u >> (std::numeric_limits<T>::digits - 2)) & 1) == 0; // Most sig bit of significand must be zero.
    }
  return false;
}


/// The system's C library may be configured to handle tininess before
/// rounding.  In that case, an underflow exception may have been
/// unnecessarily triggered on a subnormal value that was rounded to a
/// normal value, and if so, that exception should be masked.
/// Also, if the result is a NaN, ensure the value is converted to a
/// quiet NaN.
template <typename T>
inline T
maybeAdjustForTininessBeforeRoundingAndQuietNaN(T res)
{
  // SoftFloat handles tininess after rounding and handles quiet NaN
  // conversions automatically, so below only applies when not using
  // SoftFloat.
#ifndef SOFT_FLOAT
  decltype(FP_SUBNORMAL) classification = std::fpclassify(res);
  if (classification != FP_SUBNORMAL and classification != FP_ZERO)
    {
      if (std::fetestexcept(FE_UNDERFLOW))
        std::feclearexcept(FE_UNDERFLOW);
      if (classification == FP_NAN)
        res = std::numeric_limits<T>::quiet_NaN();
    }
#endif
  return res;
}


/// Floating point add. Return sum of two fp numbers. Return a
/// canonical NAN if either is a NAN.
template <typename FT>
inline
FT
doFadd(FT f1, FT f2)
{
#ifdef SOFT_FLOAT
  FT res = softAdd(f1, f2);
#else
  FT res = f1 + f2;
#endif

  res = maybeAdjustForTininessBeforeRoundingAndQuietNaN(res);
  return res;
}


/// Floating point subtract. Return difference of two fp
/// numbers. Return a canonical NAN if either is a NAN.
template <typename FT>
inline
FT
doFsub(FT f1, FT f2)
{
  return doFadd(f1, -f2);
}


/// Floating point multiply. Return product of two fp
/// numbers. Return a canonical NAN if either is a NAN.
template <typename FT>
inline
FT
doFmul(FT f1, FT f2)
{
#ifdef SOFT_FLOAT
  FT res = softMul(f1, f2);
#else
  FT res = f1 * f2;
#endif

  res = maybeAdjustForTininessBeforeRoundingAndQuietNaN(res);
  return res;
}


/// Floating point divide. Return quotient of two fp numbers. Return
/// a canonical NAN if either is a NAN.
template <typename FT>
inline
FT
doFdiv(FT f1, FT f2)
{
#ifdef SOFT_FLOAT
  FT res = softDiv(f1, f2);
#else
  FT res = f1 / f2;
#endif

  res = maybeAdjustForTininessBeforeRoundingAndQuietNaN(res);
  return res;
}


#ifndef SOFT_FLOAT

/// Floating point fused multiply and add using c++ standard library.
inline
Float16
cppFma(Float16 a, Float16 b, Float16 c)
{
  Float16 res{};
#ifndef FP_FAST_FMAF
  res = a * b + c;
#else
  res = std::fma(a, b, c);
#endif
  return res;
}


/// Floating point fused multiply and add using c++ standard library.
inline
float
cppFma(float a, float b, float c)
{
  float res = 0;
#ifndef FP_FAST_FMAF
  res = a * b + c;
#else
  res = std::fma(a, b, c);
#endif
  return res;
}


/// Floating point fused multiply and add using c++ standard library.
inline
double
cppFma(double a, double b, double c)
{
  double res = 0;
#ifndef FP_FAST_FMA
  res = a * b + c;
#else
  res = std::fma(a, b, c);
#endif
  return res;
}

#endif


/// Floating point fused multiply and add.
template <typename FT>
inline
FT
fusedMultiplyAdd(FT a, FT b, FT c)
{
#ifndef SOFT_FLOAT
  FT res = cppFma(a, b, c);
  if ((std::isinf(a) and b == FT{}) or (a == FT{} and std::isinf(b)))
    std::feraiseexcept(FE_INVALID);
#else
  FT res = softFma(a, b, c);
#endif

  res = maybeAdjustForTininessBeforeRoundingAndQuietNaN(res);
  return res;
}


template <typename FT>
inline
FT
doFsqrt(FT f1)
{
#ifdef SOFT_FLOAT
  FT res = softSqrt(f1);
#else
  FT res = std::sqrt(f1);
#endif

  res = maybeAdjustForTininessBeforeRoundingAndQuietNaN(res);
  return res;
}


template <bool EXACT, typename FT>
inline
FT
doFround(FT f1)
{
#ifdef SOFT_FLOAT
  FT res = softRound(f1, EXACT);
#else
  using int_fsize_t = typename getSameWidthIntType<FT>::type;

  FT res = f1;
  if (std::isnan(f1))
    {
      res = std::numeric_limits<FT>::quiet_NaN();
      if (isSnan(f1))
        raiseSimulatorFpFlags(FpFlags::Invalid);
    }
  else if (f1 == FT{} or std::isinf(f1))  // zero or infinity
    ;
  else
    {
      int exp = 0;
      std::frexp(f1, &exp);
      if (exp < std::numeric_limits<FT>::digits - 1)
        {
          // These conversions may raise FP exceptions, but we don't want
          // to raise them in this instruction, so just clear them afterwards.
          int_fsize_t intVal = fpConvertTo<int_fsize_t>(f1);
          res                = fpConvertTo<FT>(intVal);

          clearSimulatorFpFlags();

          if (intVal == 0 and std::signbit(f1))
            res = std::copysign(res, f1);

          if constexpr (EXACT)
            if (res != f1)
              raiseSimulatorFpFlags(FpFlags::Inexact);
        }
    }
#endif
  return res;
}

}
