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

#include <cfenv>
#include <cstdint>
#include <limits>
#include <type_traits>
#include "float16-compat.hpp"
#include "softfloat-util.hpp"

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
      else if (std::is_signed<To>::value && result < std::numeric_limits<To>::min())
        {
          softfloat_exceptionFlags = old_flags | softfloat_flag_invalid;
          return std::numeric_limits<To>::min();
        }
      else
        {
          return result;
        }
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
  else if constexpr (std::is_same<To, float>::value)
    {
      To result;
      if constexpr (std::is_same<From, Float16>::value)
        result = softToNative(f16_to_f32(nativeToSoft(x)));
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
      double result;
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
