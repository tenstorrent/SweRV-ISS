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


  /// Floating point add. Return sum of two fp numbers. Return a
  /// canonical NAN if either is a NAN.
  template <typename FT>
  FT
  doFadd(FT f1, FT f2)
  {
#ifdef SOFT_FLOAT
    FT res = softAdd(f1, f2);
#else
    FT res = f1 + f2;
#endif
    if (std::isnan(res))
      res = getQuietNan<FT>();
    return res;
  }


  /// Floating point multiply. Return product of two fp
  /// numbers. Return a canonical NAN if either is a NAN.
  template <typename FT>
  FT
  doFmul(FT f1, FT f2)
  {
#ifdef SOFT_FLOAT
    FT res = softMul(f1, f2);
#else
    FT res = f1 * f2;
#endif
    if (std::isnan(res))
      res = getQuietNan<FT>();
    return res;
  }


  /// Floating point divide. Return quotient of two fp numbers. Return
  /// a canonical NAN if either is a NAN.
  template <typename FT>
  FT
  doFdiv(FT f1, FT f2)
  {
#ifdef SOFT_FLOAT
    FT res = softDiv(f1, f2);
#else
    FT res = f1 / f2;
#endif
    if (std::isnan(res))
      res = getQuietNan<FT>();
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
  FT
  doFma(FT a, FT b, FT c)
  {
    FT res{};
#ifndef SOFT_FLOAT
    res = cppFma(a, b, c);
    if ((std::isinf(x) and y == FT{}) or (x == FT{} and std::isinf(y)))
      std::feraiseexcept(FE_INVALID);
    if (std::isnan(res))
      res = std::numeric_limits<FT>::quiet_NaN();
#else
    res = softFma(a, b, c);
#endif

    return res;
  }


  inline
  Float16
  fusedMultiplyAdd(Float16 a, Float16 b, Float16 c)
  {
    return doFma(a, b, c);
  }


  inline
  float
  fusedMultiplyAdd(float a, float b, float c)
  {
    return doFma(a, b, c);
  }


  inline
  double
  fusedMultiplyAdd(double a, double b, double c)
  {
    return doFma(a, b, c);
  }

}
