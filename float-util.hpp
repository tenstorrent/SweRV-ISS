#pragma once

#include "softfloat-util.hpp"
#include "float16-compat.hpp"


namespace WdRiscv
{

  /// Quiet NAN for Float16, float and double.
  template <typename T>
  T
  getQuietNan()
  {
    return std::numeric_limits<T>::quiet_NaN();
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

}
