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

#pragma once


/// Provide fixed size integer types wider than 64-bit.
/// The reason we do not use boost is that we want 2's complement representation
/// of signed integers.


#include <type_traits>
#include <concepts>
#include <cstdint>


namespace WdRiscv
{

// If the compiler provides a 128-bit integer type, use it.
#ifdef __SIZEOF_INT128__

  __extension__ using Int128  = __int128;
  __extension__ using Uint128 = unsigned __int128;

#else

  class Int128;

  /// Unsigned 128-bit integer.
  class Uint128
  {
  public:

    using SelfType    = Uint128;
    using SignedType  = Int128;
    using HalfType    = uint64_t;
    using QuarterType = uint32_t;

    static constexpr int width()     { return 8*sizeof(SelfType); }
    static constexpr int halfWidth() { return 8*sizeof(HalfType); }

    /// Default constructor.
    constexpr Uint128() = default;

    /// Copy constructor.
    constexpr Uint128(const Uint128&) = default;

    /// Construct from a 128-bit int: Copy bits.
    constexpr Uint128(const Int128& x);

    /// Construct from a 64-bit unsigned int.
    constexpr Uint128(HalfType x)
      : low_(x)
    { }

    /// Construct from a pair of half-type ints.
    constexpr Uint128(HalfType high, HalfType low)
      : low_(low), high_(high)
    { }

    /// Assignment constructor.
    constexpr SelfType& operator = (const SelfType&) = default;

    /// Return least sig half.
    constexpr HalfType low() const
    { return low_; }

    /// Return most sig half.
    constexpr HalfType high() const
    { return high_; }

    /// Convert to a built-in integral type.
    template <std::integral INT>
    constexpr explicit operator INT() const
    { return low_; }

    constexpr SelfType& operator += (const SelfType& x)
    {
      HalfType prevLow = low_;
      low_ += x.low_;
      high_ += x.high_;
      if (low_ < prevLow)
        high_++;
      return *this;
    }

    constexpr SelfType& operator -= (const SelfType& x)
    {
      HalfType prevLow = low_;
      low_ -= x.low_;
      high_ -= x.high_;
      if (low_ > prevLow)
        high_--;
      return *this;
    }

    constexpr SelfType& operator |= (const SelfType& x)
    { low_ |= x.low_; high_ |= x.high_; return *this; }

    constexpr SelfType& operator &= (const SelfType& x)
    { low_ &= x.low_; high_ &= x.high_; return *this; }

    constexpr SelfType& operator ^= (const SelfType& x)
    { low_ ^= x.low_; high_ ^= x.high_; return *this; }

    constexpr SelfType& operator ++ ()
    { *this += 1; return *this; }

    constexpr SelfType operator ++ (int)
    { SelfType temp = *this; temp += 1; return temp; }

    constexpr SelfType& operator -- ()
    { *this -= 1; return *this; }

    constexpr SelfType operator -- (int)
    { SelfType temp = *this; temp -= 1; return temp; }

    constexpr SelfType operator ~ () const
    { SelfType temp( ~high_, ~low_); return temp; }

    SelfType& operator *= (const SelfType& x);

    SelfType& operator /= (const SelfType& x);

    SelfType& operator %= (const SelfType& x);

    SelfType& operator >>= (int n);

    SelfType& operator <<= (int n);

    constexpr bool operator == (const SelfType& x) const
    { return high_ == x.high_ and low_ == x.low_; }

    constexpr bool operator != (const SelfType& x) const
    { return not (*this == x); }

    constexpr bool operator < (const SelfType& x) const
    { return high_ < x.high_ or (high_ == x.high_ and low_ < x.low_); }

    constexpr bool operator > (const SelfType& x) const
    { return high_ > x.high_ or (high_ == x.high_ and low_ > x.low_); }

    constexpr bool operator <= (const SelfType& x) const
    { return high_ < x.high_ or (high_ == x.high_ and low_ <= x.low_); }

    constexpr bool operator >= (const SelfType& x) const
    { return high_ > x.high_ or (high_ == x.high_ and low_ >= x.low_); }

  protected:

    HalfType low_ = 0;
    HalfType high_ = 0;
  };

  constexpr Uint128 operator + (Uint128 a, Uint128 b);
  constexpr Uint128 operator - (Uint128 a, Uint128 b);


  /// Signed 128-bit integer.
  class Int128
  {
  public:

    using SelfType     = Int128;
    using UnsignedType = Uint128;
    using HalfType     = int64_t;
    using HalfUnsigned = uint64_t;

    static constexpr int width()     { return 8*sizeof(SelfType); }
    static constexpr int halfWidth() { return 8*sizeof(HalfType); }

    /// Default constructor.
    constexpr Int128() = default;

    /// Copy constructor.
    constexpr Int128(const Int128&) = default;

    /// Construct from an unsigned 128-bit int: Copy bits.
    constexpr Int128(Uint128 x)
      : low_(static_cast<HalfType>(x.low())),
        high_(static_cast<HalfType>(x.high()))
    { }

    /// Construct from a smaller signed int.
    template <std::signed_integral T>
    constexpr Int128(T x)
      : low_(x), high_(x < 0? ~HalfType(0) : 0)
    { }

    /// Construct from a smaller unsigned int.
    template <std::unsigned_integral T>
    constexpr Int128(T x)
      : low_(x)
    { }

    /// Construct from a pair of half-type ints.
    constexpr Int128(HalfType high, HalfType low)
      : low_(low), high_(high)
    { }

    /// Assignment constructor.
    constexpr SelfType& operator = (const SelfType&) = default;

    /// Return least sig half.
    constexpr HalfType low() const
    { return low_; }

    /// Return most sig half.
    constexpr HalfType high() const
    { return high_; }

    /// Convert to a built-in integral type.
    template <std::integral INT>
    constexpr explicit operator INT() const
    { return low_; }

    constexpr SelfType& operator += (const SelfType& x)
    {
      HalfUnsigned prevLow = low_;
      low_ = static_cast<HalfType>(HalfUnsigned(low_) + HalfUnsigned(x.low_));
      high_ = static_cast<HalfType>(HalfUnsigned(high_) + HalfUnsigned(x.high_));
      if (HalfUnsigned(low_) < prevLow)
        high_ = static_cast<HalfType>(HalfUnsigned(high_) + 1);
      return *this;
    }

    constexpr SelfType& operator -= (const SelfType& x)
    {
      HalfUnsigned prevLow = low_;
      low_ = static_cast<HalfType>(HalfUnsigned(low_) - HalfUnsigned(x.low_));
      high_ = static_cast<HalfType>(HalfUnsigned(high_) - HalfUnsigned(x.high_));
      if (HalfUnsigned(low_) > prevLow)
        high_ = static_cast<HalfType>(HalfUnsigned(high_) - 1);
      return *this;
    }

    constexpr SelfType& operator |= (const SelfType& x)
    { low_ |= x.low_; high_ |= x.high_; return *this; }

    constexpr SelfType& operator &= (const SelfType& x)
    { low_ &= x.low_; high_ &= x.high_; return *this; }

    constexpr SelfType& operator ^= (const SelfType& x)
    { low_ ^= x.low_; high_ ^= x.high_; return *this; }

    constexpr SelfType& operator ++ ()
    { *this += 1; return *this; }

    constexpr SelfType operator ++ (int)
    { SelfType temp = *this; temp += 1; return temp; }

    constexpr SelfType& operator -- ()
    { *this -= 1; return *this; }

    constexpr SelfType operator -- (int)
    { SelfType temp = *this; temp -= 1; return temp; }

    constexpr SelfType operator ~ () const
    { SelfType temp( ~high_, ~low_); return temp; }

    SelfType& operator *= (const SelfType& x);

    SelfType& operator /= (const SelfType& x);

    SelfType& operator %= (const SelfType& x);

    SelfType& operator >>= (int n);

    SelfType& operator <<= (int n);

    constexpr bool operator == (const SelfType& x) const
    { return high_ == x.high_ and low_ == x.low_; }

    constexpr bool operator != (const SelfType& x) const
    { return not (*this == x); }

    constexpr bool operator < (const SelfType& x) const
    {
      return (high_ < x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) < HalfUnsigned(x.low_)));
    }

    constexpr bool operator > (const SelfType& x) const
    {
      return (high_ > x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) > HalfUnsigned(x.low_)));
    }

    constexpr bool operator <= (const SelfType& x) const
    {
      return (high_ < x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) <= HalfUnsigned(x.low_)));
    }

    constexpr bool operator >= (const SelfType& x) const
    {
      return (high_ > x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) >= HalfUnsigned(x.low_)));
    }

  protected:

    HalfType low_ = 0;
    HalfType high_ = 0;
  };

#endif


  class Int256;

  /// Unsigned 256-bit integer.
  class Uint256
  {
  public:

    using SelfType    = Uint256;
    using SignedType  = Int256;
    using HalfType    = Uint128;
    using QuarterType = uint64_t;

    static constexpr int width()     { return 8*sizeof(SelfType); }
    static constexpr int halfWidth() { return 8*sizeof(HalfType); }

    /// Default constructor.
    constexpr Uint256() = default;

    /// Copy constructor.
    constexpr Uint256(const Uint256&) = default;

    /// Construct from a 256-bit int: Copy bits.
    constexpr Uint256(const Int256& x);

    /// Construct from a smaller unsigned int.
    template <typename T>
    requires std::is_integral<T>::value or
             std::is_same<T, Uint128>::value
    constexpr Uint256(const T& x)
      : low_(x)
    { }

    /// Construct from a pair of half-type ints.
    constexpr Uint256(const HalfType& high, const HalfType& low)
      : low_(low), high_(high)
    { }

    /// Assignment constructor.
    constexpr SelfType& operator = (const SelfType&) = default;

    /// Return least sig half.
    constexpr HalfType low() const
    { return low_; }

    /// Return most sig half.
    constexpr HalfType high() const
    { return high_; }

    /// Convert to a built-in integral type.
    template <typename INT>
    requires std::is_integral<INT>::value or
             std::is_same<INT, Int128>::value or
             std::is_same<INT, Uint128>::value
    constexpr explicit operator INT() const
    { return INT(low_); }

    constexpr SelfType& operator += (const SelfType& x)
    {
      HalfType prevLow = low_;
      low_ += x.low_;
      high_ += x.high_;
      if (low_ < prevLow)
        high_++;
      return *this;
    }

    constexpr SelfType& operator -= (const SelfType& x)
    {
      HalfType prevLow = low_;
      low_ -= x.low_;
      high_ -= x.high_;
      if (low_ > prevLow)
        high_--;
      return *this;
    }

    constexpr SelfType& operator |= (const SelfType& x)
    { low_ |= x.low_; high_ |= x.high_; return *this; }

    constexpr SelfType& operator &= (const SelfType& x)
    { low_ &= x.low_; high_ &= x.high_; return *this; }

    constexpr SelfType& operator ^= (const SelfType& x)
    { low_ ^= x.low_; high_ ^= x.high_; return *this; }

    constexpr SelfType& operator ++ ()
    { *this += 1; return *this; }

    constexpr SelfType operator ++ (int)
    { SelfType temp = *this; temp += 1; return temp; }

    constexpr SelfType& operator -- ()
    { *this -= 1; return *this; }

    constexpr SelfType operator -- (int)
    { SelfType temp = *this; temp -= 1; return temp; }

    constexpr SelfType operator ~ () const
    { SelfType temp( ~high_, ~low_); return temp; }

    SelfType& operator *= (const SelfType& x);

    SelfType& operator /= (const SelfType& x);

    SelfType& operator %= (const SelfType& x);

    SelfType& operator >>= (int n);

    SelfType& operator <<= (int n);

    constexpr bool operator == (const SelfType& x) const
    { return high_ == x.high_ and low_ == x.low_; }

    constexpr bool operator != (const SelfType& x) const
    { return not (*this == x); }

    constexpr bool operator < (const SelfType& x) const
    { return high_ < x.high_ or (high_ == x.high_ and low_ < x.low_); }

    constexpr bool operator > (const SelfType& x) const
    { return high_ > x.high_ or (high_ == x.high_ and low_ > x.low_); }

    constexpr bool operator <= (const SelfType& x) const
    { return high_ < x.high_ or (high_ == x.high_ and low_ <= x.low_); }

    constexpr bool operator >= (const SelfType& x) const
    { return high_ > x.high_ or (high_ == x.high_ and low_ >= x.low_); }

  protected:

    HalfType low_ = 0;
    HalfType high_ = 0;
  };

  constexpr Uint256 operator + (Uint256 a, Uint256 b);
  constexpr Uint256 operator - (Uint256 a, Uint256 b);


  /// Signed 256-bit integer.
  class Int256
  {
  public:

    using SelfType     =  Int256;
    using UnsignedType =  Uint256;
    using HalfType     =  Int128;
    using HalfUnsigned =  Uint128;

    static constexpr int width()     { return 8*sizeof(SelfType); }
    static constexpr int halfWidth() { return 8*sizeof(HalfType); }

    /// Default constructor.
    constexpr Int256() = default;

    /// Copy constructor.
    constexpr Int256(const Int256&) = default;

    /// Construct from an unsigned 256-bit int: Copy bits.
    constexpr Int256(const Uint256& x)
      : low_(static_cast<HalfType>(x.low())),
        high_(static_cast<HalfType>(x.high()))
    { }

    /// Construct from a smaller signed int.
    template <typename T>
    requires std::is_signed<T>::value or
             std::is_same<T, Int128>::value
    constexpr Int256(const T& x)
      : low_(x), high_(x < T{}? ~HalfType(0) : HalfType(0))
    { }

    /// Construct from a smaller unsigned int.
    template <typename T>
    requires std::is_unsigned<T>::value or
             std::is_same<T, Uint128>::value
    constexpr Int256(const T& x)
      : low_(x)
    { }

    /// Construct from a pair of half-type ints.
    constexpr Int256(const HalfType& high, const HalfType& low)
      : low_(low), high_(high)
    { }

    /// Assignment constructor.
    constexpr SelfType& operator = (const SelfType&) = default;

    /// Return least sig half.
    constexpr HalfType low() const
    { return low_; }

    /// Return most sig half.
    constexpr HalfType high() const
    { return high_; }

    /// Convert to a built-in integral type.
    template <typename INT>
    requires std::is_integral<INT>::value or
             std::is_same<INT, Int128>::value or
             std::is_same<INT, Uint128>::value
    constexpr explicit operator INT() const
    { return INT(low_); }

    constexpr SelfType& operator += (const SelfType& x)
    {
      HalfUnsigned prevLow = low_;
      low_ = static_cast<HalfType>(HalfUnsigned(low_) + HalfUnsigned(x.low_));
      high_ = static_cast<HalfType>(HalfUnsigned(high_) + HalfUnsigned(x.high_));
      if (HalfUnsigned(low_) < prevLow)
        high_ = static_cast<HalfType>(HalfUnsigned(high_) + 1);
      return *this;
    }

    constexpr SelfType& operator -= (const SelfType& x)
    {
      HalfUnsigned prevLow = low_;
      low_ = static_cast<HalfType>(HalfUnsigned(low_) - HalfUnsigned(x.low_));
      high_ = static_cast<HalfType>(HalfUnsigned(high_) - HalfUnsigned(x.high_));
      if (HalfUnsigned(low_) > prevLow)
        high_ = static_cast<HalfType>(HalfUnsigned(high_) - 1);
      return *this;
    }

    constexpr SelfType& operator |= (const SelfType& x)
    { low_ |= x.low_; high_ |= x.high_; return *this; }

    constexpr SelfType& operator &= (const SelfType& x)
    { low_ &= x.low_; high_ &= x.high_; return *this; }

    constexpr SelfType& operator ^= (const SelfType& x)
    { low_ ^= x.low_; high_ ^= x.high_; return *this; }

    constexpr SelfType& operator ++ ()
    { *this += 1; return *this; }

    constexpr SelfType operator ++ (int)
    { SelfType temp = *this; temp += 1; return temp; }

    constexpr SelfType& operator -- ()
    { *this -= 1; return *this; }

    constexpr SelfType operator -- (int)
    { SelfType temp = *this; temp -= 1; return temp; }

    constexpr SelfType operator ~ () const
    { SelfType temp( ~high_, ~low_); return temp; }

    SelfType& operator *= (const SelfType& x);

    SelfType& operator /= (const SelfType& x);

    SelfType& operator %= (const SelfType& x);

    SelfType& operator >>= (int n);

    SelfType& operator <<= (int n);

    constexpr bool operator == (const SelfType& x) const
    { return high_ == x.high_ and low_ == x.low_; }

    constexpr bool operator != (const SelfType& x) const
    { return not (*this == x); }

    constexpr bool operator < (const SelfType& x) const
    {
      return (high_ < x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) < HalfUnsigned(x.low_)));
    }

    constexpr bool operator > (const SelfType& x) const
    {
      return (high_ > x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) > HalfUnsigned(x.low_)));
    }

    constexpr bool operator <= (const SelfType& x) const
    {
      return (high_ < x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) <= HalfUnsigned(x.low_)));
    }

    constexpr bool operator >= (const SelfType& x) const
    {
      return (high_ > x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) >= HalfUnsigned(x.low_)));
    }

  protected:

    HalfType low_ = 0;
    HalfType high_ = 0;
  };


  class Int512;

  /// Unsigned 512-bit integer.
  class Uint512
  {
  public:

    using SelfType    = Uint512;
    using SignedType  = Int512;
    using HalfType    = Uint256;
    using QuarterType = Uint128;

    static constexpr int width()     { return 8*sizeof(SelfType); }
    static constexpr int halfWidth() { return 8*sizeof(HalfType); }

    /// Default constructor.
    constexpr Uint512() = default;

    /// Copy constructor.
    constexpr Uint512(const Uint512&) = default;

    /// Construct from a 512-bit int: Copy bits.
    constexpr Uint512(const Int512& x);

    /// Construct from a smaller unsigned int.
    template <typename T>
    requires std::is_integral<T>::value or
             std::is_same<T, Uint128>::value or
             std::is_same<T, Uint256>::value
    constexpr Uint512(const T& x)
      : low_(x)
    { }

    /// Construct from a pair of 256-bit unsigned ints.
    constexpr Uint512(const Uint256& high, const Uint256& low)
      : low_(low), high_(high)
    { }

    /// Assignment constructor.
    constexpr SelfType& operator = (const SelfType&) = default;

    /// Return least sig half.
    constexpr HalfType low() const
    { return low_; }

    /// Return most sig half.
    constexpr HalfType high() const
    { return high_; }

    /// Convert to a half-type.
    constexpr operator HalfType() const
    { return low_; }

    /// Convert to a built-in integral type.
    template <typename INT>
    requires std::is_integral<INT>::value or
             std::is_same<INT, Int128>::value or
             std::is_same<INT, Uint128>::value
    constexpr explicit operator INT() const
    { return INT(low_); }

    constexpr SelfType& operator += (const SelfType& x)
    {
      HalfType prevLow = low_;
      low_ += x.low_;
      high_ += x.high_;
      if (low_ < prevLow)
        high_++;
      return *this;
    }

    constexpr SelfType& operator -= (const SelfType& x)
    {
      HalfType prevLow = low_;
      low_ -= x.low_;
      high_ -= x.high_;
      if (low_ > prevLow)
        high_--;
      return *this;
    }

    constexpr SelfType& operator |= (const SelfType& x)
    { low_ |= x.low_; high_ |= x.high_; return *this; }

    constexpr SelfType& operator &= (const SelfType& x)
    { low_ &= x.low_; high_ &= x.high_; return *this; }

    constexpr SelfType& operator ^= (const SelfType& x)
    { low_ ^= x.low_; high_ ^= x.high_; return *this; }

    constexpr SelfType& operator ++ ()
    { *this += 1; return *this; }

    constexpr SelfType operator ++ (int)
    { SelfType temp = *this; temp += 1; return temp; }

    constexpr SelfType& operator -- ()
    { *this -= 1; return *this; }

    constexpr SelfType operator -- (int)
    { SelfType temp = *this; temp -= 1; return temp; }

    constexpr SelfType operator ~ () const
    { SelfType temp( ~high_, ~low_); return temp; }

    SelfType& operator *= (const SelfType& x);

    SelfType& operator /= (const SelfType& x);

    SelfType& operator %= (const SelfType& x);

    SelfType& operator >>= (int n);

    SelfType& operator <<= (int n);

    constexpr bool operator == (const SelfType& x) const
    { return high_ == x.high_ and low_ == x.low_; }

    constexpr bool operator != (const SelfType& x) const
    { return not (*this == x); }

    constexpr bool operator < (const SelfType& x) const
    { return high_ < x.high_ or (high_ == x.high_ and low_ < x.low_); }

    constexpr bool operator > (const SelfType& x) const
    { return high_ > x.high_ or (high_ == x.high_ and low_ > x.low_); }

    constexpr bool operator <= (const SelfType& x) const
    { return high_ < x.high_ or (high_ == x.high_ and low_ <= x.low_); }

    constexpr bool operator >= (const SelfType& x) const
    { return high_ > x.high_ or (high_ == x.high_ and low_ >= x.low_); }

  protected:

    HalfType low_ = 0;
    HalfType high_ = 0;
  };

  constexpr Uint512 operator + (Uint512 a, Uint512 b);
  constexpr Uint512 operator - (Uint512 a, Uint512 b);


  /// Signed 512-bit integer.
  class Int512
  {
  public:

    using SelfType     = Int512;
    using UnsignedType = Uint512;
    using HalfType     = Int256;
    using HalfUnsigned = Uint256;

    static constexpr int width()     { return 8*sizeof(SelfType); }
    static constexpr int halfWidth() { return 8*sizeof(HalfType); }

    /// Default constructor.
    constexpr Int512() = default;

    /// Copy constructor.
    constexpr Int512(const Int512&) = default;

    /// Construct from an unsigned 512-bit int: Copy bits.
    constexpr Int512(const Uint512& x)
      : low_(static_cast<HalfType>(x.low())),
        high_(static_cast<HalfType>(x.high()))
    { }

    /// Construct from a smaller signed int.
    template <typename T>
    requires std::is_signed<T>::value or
             std::is_same<T, Int128>::value or
             std::is_same<T, Int256>::value
    constexpr Int512(const T& x)
      : low_(x), high_(x < T{}? ~HalfType(0) : HalfType(0))
    { }

    /// Construct from a smaller unsigned int.
    template <typename T>
    requires std::is_unsigned<T>::value or
             std::is_same<T, Uint128>::value or
             std::is_same<T, Uint256>::value
    constexpr Int512(const T& x)
      : low_(x)
    { }

    /// Construct from a pair of 256-bit ints.
    constexpr Int512(const HalfType& high, const HalfType& low)
      : low_(low), high_(high)
    { }

    /// Assignment constructor.
    constexpr SelfType& operator = (const SelfType&) = default;

    /// Return least sig half.
    constexpr HalfType low() const
    { return low_; }

    /// Return most sig half.
    constexpr HalfType high() const
    { return high_; }

    /// Convert to a half.
    constexpr operator HalfType() const
    { return low_; }

    /// Convert to a built-in integral type.
    template <typename INT>
    requires std::is_integral<INT>::value or
             std::is_same<INT, Int128>::value or
             std::is_same<INT, Uint128>::value
    constexpr explicit operator INT() const
    { return INT(low_); }

    constexpr SelfType& operator += (const SelfType& x)
    {
      HalfUnsigned prevLow = low_;
      low_ = static_cast<HalfType>(HalfUnsigned(low_) + HalfUnsigned(x.low_));
      high_ = static_cast<HalfType>(HalfUnsigned(high_) + HalfUnsigned(x.high_));
      if (HalfUnsigned(low_) < prevLow)
        high_ = static_cast<HalfType>(HalfUnsigned(high_) + 1);
      return *this;
    }

    constexpr SelfType& operator -= (const SelfType& x)
    {
      HalfUnsigned prevLow = low_;
      low_ = static_cast<HalfType>(HalfUnsigned(low_) - HalfUnsigned(x.low_));
      high_ = static_cast<HalfType>(HalfUnsigned(high_) - HalfUnsigned(x.high_));
      if (HalfUnsigned(low_) > prevLow)
        high_ = static_cast<HalfType>(HalfUnsigned(high_) - 1);
      return *this;
    }

    constexpr SelfType& operator |= (const SelfType& x)
    { low_ |= x.low_; high_ |= x.high_; return *this; }

    constexpr SelfType& operator &= (const SelfType& x)
    { low_ &= x.low_; high_ &= x.high_; return *this; }

    constexpr SelfType& operator ^= (const SelfType& x)
    { low_ ^= x.low_; high_ ^= x.high_; return *this; }

    constexpr SelfType& operator ++ ()
    { *this += 1; return *this; }

    constexpr SelfType operator ++ (int)
    { SelfType temp = *this; temp += 1; return temp; }

    constexpr SelfType& operator -- ()
    { *this -= 1; return *this; }

    constexpr SelfType operator -- (int)
    { SelfType temp = *this; temp -= 1; return temp; }

    constexpr SelfType operator ~ () const
    { SelfType temp( ~high_, ~low_); return temp; }

    SelfType& operator *= (const SelfType& x);

    SelfType& operator /= (const SelfType& x);

    SelfType& operator %= (const SelfType& x);

    SelfType& operator >>= (int n);

    SelfType& operator <<= (int n);

    constexpr bool operator == (const SelfType& x) const
    { return high_ == x.high_ and low_ == x.low_; }

    constexpr bool operator != (const SelfType& x) const
    { return not (*this == x); }

    constexpr bool operator < (const SelfType& x) const
    {
      return (high_ < x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) < HalfUnsigned(x.low_)));
    }

    constexpr bool operator > (const SelfType& x) const
    {
      return (high_ > x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) > HalfUnsigned(x.low_)));
    }

    constexpr bool operator <= (const SelfType& x) const
    {
      return (high_ < x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) <= HalfUnsigned(x.low_)));
    }

    constexpr bool operator >= (const SelfType& x) const
    {
      return (high_ > x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) >= HalfUnsigned(x.low_)));
    }

  protected:

    HalfType low_ = 0;
    HalfType high_ = 0;
  };


  class Int1024;

  /// Unsigned 1024-bit integer.
  class Uint1024
  {
  public:

    using SelfType    = Uint1024;
    using SignedType  = Int1024;
    using HalfType    = Uint512;
    using QuarterType = Uint256;

    static constexpr int width()     { return 8*sizeof(SelfType); }
    static constexpr int halfWidth() { return 8*sizeof(HalfType); }

    /// Default constructor.
    constexpr Uint1024() = default;

    /// Copy constructor.
    constexpr Uint1024(const Uint1024&) = default;

    /// Construct from a 1024-bit int: Copy bits.
    constexpr Uint1024(const Int1024& x);

    /// Construct from a smaller unsigned int.
    template <typename T>
    requires std::is_integral<T>::value or
             std::is_same<T, Uint128>::value or
             std::is_same<T, Uint256>::value or
             std::is_same<T, Uint512>::value
    constexpr Uint1024(const T& x)
      : low_(x)
    { }

    /// Construct from a pair of 512-bit unsigned ints.
    constexpr Uint1024(const HalfType& high, const HalfType& low)
      : low_(low), high_(high)
    { }

    /// Assignment constructor.
    constexpr SelfType& operator = (const SelfType&) = default;

    /// Return least sig half.
    constexpr HalfType low() const
    { return low_; }

    /// Return most sig half.
    constexpr HalfType high() const
    { return high_; }

    /// Convert to a half-type.
    constexpr operator HalfType() const
    { return low_; }

    /// Convert to a built-in integral type.
    template <typename INT>
    requires std::is_integral<INT>::value or
             std::is_same<INT, Int128>::value or
             std::is_same<INT, Uint128>::value
    constexpr explicit operator INT() const
    { return INT(low_); }

    constexpr SelfType& operator += (const SelfType& x)
    {
      HalfType prevLow = low_;
      low_ += x.low_;
      high_ += x.high_;
      if (low_ < prevLow)
        high_++;
      return *this;
    }

    constexpr SelfType& operator -= (const SelfType& x)
    {
      HalfType prevLow = low_;
      low_ -= x.low_;
      high_ -= x.high_;
      if (low_ > prevLow)
        high_--;
      return *this;
    }

    constexpr SelfType& operator |= (const SelfType& x)
    { low_ |= x.low_; high_ |= x.high_; return *this; }

    constexpr SelfType& operator &= (const SelfType& x)
    { low_ &= x.low_; high_ &= x.high_; return *this; }

    constexpr SelfType& operator ^= (const SelfType& x)
    { low_ ^= x.low_; high_ ^= x.high_; return *this; }

    constexpr SelfType& operator ++ ()
    { *this += 1; return *this; }

    constexpr SelfType operator ++ (int)
    { SelfType temp = *this; temp += 1; return temp; }

    constexpr SelfType& operator -- ()
    { *this -= 1; return *this; }

    constexpr SelfType operator -- (int)
    { SelfType temp = *this; temp -= 1; return temp; }

    constexpr SelfType operator ~ () const
    { SelfType temp( ~high_, ~low_); return temp; }

    SelfType& operator *= (const SelfType& x);

    SelfType& operator /= (const SelfType& x);

    SelfType& operator %= (const SelfType& x);

    SelfType& operator >>= (int n);

    SelfType& operator <<= (int n);

    constexpr bool operator == (const SelfType& x) const
    { return high_ == x.high_ and low_ == x.low_; }

    constexpr bool operator != (const SelfType& x) const
    { return not (*this == x); }

    constexpr bool operator < (const SelfType& x) const
    { return high_ < x.high_ or (high_ == x.high_ and low_ < x.low_); }

    constexpr bool operator > (const SelfType& x) const
    { return high_ > x.high_ or (high_ == x.high_ and low_ > x.low_); }

    constexpr bool operator <= (const SelfType& x) const
    { return high_ < x.high_ or (high_ == x.high_ and low_ <= x.low_); }

    constexpr bool operator >= (const SelfType& x) const
    { return high_ > x.high_ or (high_ == x.high_ and low_ >= x.low_); }

  protected:

    HalfType low_ = 0;
    HalfType high_ = 0;
  };

  constexpr Uint1024 operator + (Uint1024 a, Uint1024 b);
  constexpr Uint1024 operator - (Uint1024 a, Uint1024 b);


  /// Signed 1024-bit integer.
  class Int1024
  {
  public:

    using SelfType     = Int1024;
    using UnsignedType = Uint1024;
    using HalfType     = Int512;
    using HalfUnsigned = Uint512;

    static constexpr int width()     { return 8*sizeof(SelfType); }
    static constexpr int halfWidth() { return 8*sizeof(HalfType); }

    /// Default constructor.
    constexpr Int1024() = default;

    /// Copy constructor.
    constexpr Int1024(const Int1024&) = default;

    /// Construct from an unsigned 1024-bit int: Copy bits.
    constexpr Int1024(const Uint1024& x)
      : low_(static_cast<HalfType>(x.low())),
        high_(static_cast<HalfType>(x.high()))
    { }

    /// Construct from a smaller signed int.
    template <typename T>
    requires std::is_signed<T>::value or
             std::is_same<T, Int128>::value or
             std::is_same<T, Int256>::value or
             std::is_same<T, Int512>::value
    constexpr Int1024(const T& x)
      : low_(x), high_(x < T{}? ~HalfType(0) : HalfType(0))
    { }

    /// Construct from a smaller unsigned int.
    template <typename T>
    requires std::is_unsigned<T>::value or
             std::is_same<T, Uint128>::value or
             std::is_same<T, Uint256>::value or
             std::is_same<T, Uint512>::value
    constexpr Int1024(const T& x)
      : low_(x)
    { }

    /// Construct from a pair of 512-bit ints.
    constexpr Int1024(const HalfType& high, const HalfType& low)
      : low_(low), high_(high)
    { }

    /// Assignment constructor.
    constexpr SelfType& operator = (const SelfType&) = default;

    /// Return least sig half.
    constexpr HalfType low() const
    { return low_; }

    /// Return most sig half.
    constexpr HalfType high() const
    { return high_; }

    /// Convert to a half.
    constexpr operator HalfType() const
    { return low_; }

    /// Convert to a built-in integral type.
    template <typename INT>
    requires std::is_integral<INT>::value or
             std::is_same<INT, Int128>::value or
             std::is_same<INT, Uint128>::value
    constexpr explicit operator INT() const
    { return INT(low_); }

    constexpr SelfType& operator += (const SelfType& x)
    {
      HalfUnsigned prevLow = low_;
      low_ = static_cast<HalfType>(HalfUnsigned(low_) + HalfUnsigned(x.low_));
      high_ = static_cast<HalfType>(HalfUnsigned(high_) + HalfUnsigned(x.high_));
      if (HalfUnsigned(low_) < prevLow)
        high_ = static_cast<HalfType>(HalfUnsigned(high_) + 1);
      return *this;
    }

    constexpr SelfType& operator -= (const SelfType& x)
    {
      HalfUnsigned prevLow = low_;
      low_ = static_cast<HalfType>(HalfUnsigned(low_) - HalfUnsigned(x.low_));
      high_ = static_cast<HalfType>(HalfUnsigned(high_) - HalfUnsigned(x.high_));
      if (HalfUnsigned(low_) > prevLow)
        high_ = static_cast<HalfType>(HalfUnsigned(high_) - 1);
      return *this;
    }

    constexpr SelfType& operator |= (const SelfType& x)
    { low_ |= x.low_; high_ |= x.high_; return *this; }

    constexpr SelfType& operator &= (const SelfType& x)
    { low_ &= x.low_; high_ &= x.high_; return *this; }

    constexpr SelfType& operator ^= (const SelfType& x)
    { low_ ^= x.low_; high_ ^= x.high_; return *this; }

    constexpr SelfType& operator ++ ()
    { *this += 1; return *this; }

    constexpr SelfType operator ++ (int)
    { SelfType temp = *this; temp += 1; return temp; }

    constexpr SelfType& operator -- ()
    { *this -= 1; return *this; }

    constexpr SelfType operator -- (int)
    { SelfType temp = *this; temp -= 1; return temp; }

    constexpr SelfType operator ~ () const
    { SelfType temp( ~high_, ~low_); return temp; }

    SelfType& operator *= (const SelfType& x);

    SelfType& operator /= (const SelfType& x);

    SelfType& operator %= (const SelfType& x);

    SelfType& operator >>= (int n);

    SelfType& operator <<= (int n);

    constexpr bool operator == (const SelfType& x) const
    { return high_ == x.high_ and low_ == x.low_; }

    constexpr bool operator != (const SelfType& x) const
    { return not (*this == x); }

    constexpr bool operator < (const SelfType& x) const
    {
      return (high_ < x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) < HalfUnsigned(x.low_)));
    }

    constexpr bool operator > (const SelfType& x) const
    {
      return (high_ > x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) > HalfUnsigned(x.low_)));
    }

    constexpr bool operator <= (const SelfType& x) const
    {
      return (high_ < x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) <= HalfUnsigned(x.low_)));
    }

    constexpr bool operator >= (const SelfType& x) const
    {
      return (high_ > x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) >= HalfUnsigned(x.low_)));
    }

  protected:

    HalfType low_ = 0;
    HalfType high_ = 0;
  };


#ifndef __SIZEOF_INT128__

  constexpr
  Uint128::Uint128(const Int128& x)
    : low_(x.low()), high_(x.high())
  { }

  constexpr Uint128 operator + (Uint128 a, Uint128 b)
  { a += b; return a; }

  constexpr Uint128 operator - (Uint128 a, Uint128 b)
  { a -= b; return a; }

  inline Uint128 operator * (Uint128 a, Uint128 b)
  { a *= b; return a; }

  inline Uint128 operator / (Uint128 a, Uint128 b)
  { a /= b; return a; }

  inline Uint128 operator % (Uint128 a, Uint128 b)
  { a %= b; return a; }

  constexpr Uint128 operator - (Uint128 a)
  { Uint128 c = 0; c -= a; return c; }

  inline Uint128 operator >> (Uint128 x, int n)
  { x >>= n; return x; }

  inline Uint128 operator << (Uint128 x, int n)
  { x <<= n; return x; }

  constexpr Uint128 operator | (Uint128 a, Uint128 b)
  { a |= b; return a; }

  constexpr Uint128 operator & (Uint128 a, Uint128 b)
  { a &= b; return a; }

  constexpr Uint128 operator ^ (Uint128 a, Uint128 b)
  { a ^= b; return a; }


  constexpr Int128 operator + (Int128 a, Int128 b)
  { a += b; return a; }

  constexpr Int128 operator - (Int128 a, Int128 b)
  { a -= b; return a; }

  inline Int128 operator * (Int128 a, Int128 b)
  { a *= b; return a; }

  inline Int128 operator / (Int128 a, Int128 b)
  { a /= b; return a; }

  inline Int128 operator % (Int128 a, Int128 b)
  { a %= b; return a; }

  constexpr Int128 operator - (Int128 a)
  { Int128 c = 0; c -= a; return c; }

  inline Int128 operator >> (Int128 x, int n)
  { x >>= n; return x; }

  inline Int128 operator << (Int128 x, int n)
  { x <<= n; return x; }

  constexpr Int128 operator | (Int128 a, Int128 b)
  { a |= b; return a; }

  constexpr Int128 operator & (Int128 a, Int128 b)
  { a &= b; return a; }

  constexpr Int128 operator ^ (Int128 a, Int128 b)
  { a ^= b; return a; }

#endif


  constexpr
  Uint256::Uint256(const Int256& x)
    : low_(x.low()), high_(x.high())
  { }

  constexpr Uint256 operator + (Uint256 a, Uint256 b)
  { a += b; return a; }

  constexpr Uint256 operator - (Uint256 a, Uint256 b)
  { a -= b; return a; }

  inline Uint256 operator * (Uint256 a, Uint256 b)
  { a *= b; return a; }

  inline Uint256 operator / (Uint256 a, Uint256 b)
  { a /= b; return a; }

  inline Uint256 operator % (Uint256 a, Uint256 b)
  { a %= b; return a; }

  constexpr Uint256 operator - (Uint256 a)
  { Uint256 c = 0UL; c -= a; return c; }

  inline Uint256 operator >> (Uint256 x, int n)
  { x >>= n; return x; }

  inline Uint256 operator << (Uint256 x, int n)
  { x <<= n; return x; }

  constexpr Uint256 operator | (Uint256 a, Uint256 b)
  { a |= b; return a; }

  constexpr Uint256 operator & (Uint256 a, Uint256 b)
  { a &= b; return a; }

  constexpr Uint256 operator ^ (Uint256 a, Uint256 b)
  { a ^= b; return a; }


  constexpr Int256 operator + (Int256 a, Int256 b)
  { a += b; return a; }

  constexpr Int256 operator - (Int256 a, Int256 b)
  { a -= b; return a; }

  inline Int256 operator * (Int256 a, Int256 b)
  { a *= b; return a; }

  inline Int256 operator / (Int256 a, Int256 b)
  { a /= b; return a; }

  inline Int256 operator % (Int256 a, Int256 b)
  { a %= b; return a; }

  constexpr Int256 operator - (Int256 a)
  { Int256 c = 0L; c -= a; return c; }

  inline Int256 operator >> (Int256 x, int n)
  { x >>= n; return x; }

  inline Int256 operator << (Int256 x, int n)
  { x <<= n; return x; }

  constexpr Int256 operator | (Int256 a, Int256 b)
  { a |= b; return a; }

  constexpr Int256 operator & (Int256 a, Int256 b)
  { a &= b; return a; }

  constexpr Int256 operator ^ (Int256 a, Int256 b)
  { a ^= b; return a; }


  constexpr
  Uint512::Uint512(const Int512& x)
    : low_(x.low()), high_(x.high())
  { }

  constexpr Uint512 operator + (Uint512 a, Uint512 b)
  { a += b; return a; }

  constexpr Uint512 operator - (Uint512 a, Uint512 b)
  { a -= b; return a; }

  inline Uint512 operator * (Uint512 a, Uint512 b)
  { a *= b; return a; }

  inline Uint512 operator / (Uint512 a, Uint512 b)
  { a /= b; return a; }

  inline Uint512 operator % (Uint512 a, Uint512 b)
  { a %= b; return a; }

  constexpr Uint512 operator - (Uint512 a)
  { Uint512 c = 0UL; c -= a; return c; }

  inline Uint512 operator >> (Uint512 x, int n)
  { x >>= n; return x; }

  inline Uint512 operator << (Uint512 x, int n)
  { x <<= n; return x; }

  constexpr Uint512 operator | (Uint512 a, Uint512 b)
  { a |= b; return a; }

  constexpr Uint512 operator & (Uint512 a, Uint512 b)
  { a &= b; return a; }

  constexpr Uint512 operator ^ (Uint512 a, Uint512 b)
  { a ^= b; return a; }


  constexpr Int512 operator + (Int512 a, Int512 b)
  { a += b; return a; }

  constexpr Int512 operator - (Int512 a, Int512 b)
  { a -= b; return a; }

  inline Int512 operator * (Int512 a, Int512 b)
  { a *= b; return a; }

  inline Int512 operator / (Int512 a, Int512 b)
  { a /= b; return a; }

  inline Int512 operator % (Int512 a, Int512 b)
  { a %= b; return a; }

  constexpr Int512 operator - (Int512 a)
  { Int512 c = 0L; c -= a; return c; }

  inline Int512 operator >> (Int512 x, int n)
  { x >>= n; return x; }

  inline Int512 operator << (Int512 x, int n)
  { x <<= n; return x; }

  constexpr Int512 operator | (Int512 a, Int512 b)
  { a |= b; return a; }

  constexpr Int512 operator & (Int512 a, Int512 b)
  { a &= b; return a; }

  constexpr Int512 operator ^ (Int512 a, Int512 b)
  { a ^= b; return a; }


  constexpr
  Uint1024::Uint1024(const Int1024& x)
    : low_(x.low()), high_(x.high())
  { }

  constexpr Uint1024 operator + (Uint1024 a, Uint1024 b)
  { a += b; return a; }

  constexpr Uint1024 operator - (Uint1024 a, Uint1024 b)
  { a -= b; return a; }

  inline Uint1024 operator * (Uint1024 a, Uint1024 b)
  { a *= b; return a; }

  inline Uint1024 operator / (Uint1024 a, Uint1024 b)
  { a /= b; return a; }

  inline Uint1024 operator % (Uint1024 a, Uint1024 b)
  { a %= b; return a; }

  constexpr Uint1024 operator - (Uint1024 a)
  { Uint1024 c = 0UL; c -= a; return c; }

  inline Uint1024 operator >> (Uint1024 x, int n)
  { x >>= n; return x; }

  inline Uint1024 operator << (Uint1024 x, int n)
  { x <<= n; return x; }

  constexpr Uint1024 operator | (Uint1024 a, Uint1024 b)
  { a |= b; return a; }

  constexpr Uint1024 operator & (Uint1024 a, Uint1024 b)
  { a &= b; return a; }

  constexpr Uint1024 operator ^ (Uint1024 a, Uint1024 b)
  { a ^= b; return a; }


  constexpr Int1024 operator + (Int1024 a, Int1024 b)
  { a += b; return a; }

  constexpr Int1024 operator - (Int1024 a, Int1024 b)
  { a -= b; return a; }

  inline Int1024 operator * (Int1024 a, Int1024 b)
  { a *= b; return a; }

  inline Int1024 operator / (Int1024 a, Int1024 b)
  { a /= b; return a; }

  inline Int1024 operator % (Int1024 a, Int1024 b)
  { a %= b; return a; }

  constexpr Int1024 operator - (Int1024 a)
  { Int1024 c = 0L; c -= a; return c; }

  inline Int1024 operator >> (Int1024 x, int n)
  { x >>= n; return x; }

  inline Int1024 operator << (Int1024 x, int n)
  { x <<= n; return x; }

  constexpr Int1024 operator | (Int1024 a, Int1024 b)
  { a |= b; return a; }

  constexpr Int1024 operator & (Int1024 a, Int1024 b)
  { a &= b; return a; }

  constexpr Int1024 operator ^ (Int1024 a, Int1024 b)
  { a ^= b; return a; }


  /// Return the width in bits of the given integer type T. This is
  /// usually 8*sizeof(T).
  template <typename T>
  constexpr unsigned
  integerWidth()
  {
    if constexpr (std::is_same<Int128, T>::value)   return 128;
    if constexpr (std::is_same<Int256, T>::value)   return 256;
    if constexpr (std::is_same<Int512, T>::value)   return 512;
    if constexpr (std::is_same<Int1024, T>::value)  return 1024;
    if constexpr (std::is_same<Uint128, T>::value)  return 128;
    if constexpr (std::is_same<Uint256, T>::value)  return 256;
    if constexpr (std::is_same<Uint512, T>::value)  return 512;
    if constexpr (std::is_same<Uint1024, T>::value) return 1024;

    return 8*sizeof(T);
  }


  /// Return the integral type that is twice as wide as the given
  /// type. For example:
  ///    makeDoubleWide<uint16_t>::type
  /// yields the type
  ///    uint32_t.
  template <typename T>
  struct makeDoubleWide;

  template <> struct makeDoubleWide<uint8_t>    { using type = uint16_t; };
  template <> struct makeDoubleWide<uint16_t>   { using type = uint32_t; };
  template <> struct makeDoubleWide<uint32_t>   { using type = uint64_t; };
  template <> struct makeDoubleWide<uint64_t>   { using type = Uint128; };
  template <> struct makeDoubleWide<Uint128>    { using type = Uint256; };
  template <> struct makeDoubleWide<Uint256>    { using type = Uint512; };
  template <> struct makeDoubleWide<Uint512>    { using type = Uint1024; };

  template <> struct makeDoubleWide<int8_t>     { using type = int16_t; };
  template <> struct makeDoubleWide<int16_t>    { using type = int32_t; };
  template <> struct makeDoubleWide<int32_t>    { using type = int64_t; };
  template <> struct makeDoubleWide<int64_t>    { using type = Int128; };
  template <> struct makeDoubleWide<Int128>     { using type = Int256; };
  template <> struct makeDoubleWide<Int256>     { using type = Int512; };
  template <> struct makeDoubleWide<Int512>     { using type = Int1024; };
}
