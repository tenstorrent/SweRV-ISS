#pragma once

#include "softfloat-util.hpp"


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
  static FT
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
  static FT
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
}

