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

#include <bit>
#include <cassert>
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>
#include <vector>
#include "float16-compat.hpp"
#include "FpRegNames.hpp"

namespace WdRiscv
{

  /// RISCV values used to synthesize the results of the classify
  /// instructions (e.g. flcass.s).
  enum class FpClassifyMasks : uint32_t
    {
      NegInfinity  = 1,       // bit 0
      NegNormal    = 1 << 1,  // bit 1
      NegSubnormal = 1 << 2,  // bit 2
      NegZero      = 1 << 3,  // bit 3
      PosZero      = 1 << 4,  // bit 4
      PosSubnormal = 1 << 5,  // bit 5
      PosNormal    = 1 << 6,  // bit 6
      PosInfinity  = 1 << 7,  // bit 7
      SignalingNan = 1 << 8,  // bit 8
      QuietNan     = 1 << 9   // bit 9
    };

  /// Values of FS field in mstatus.
  enum class FpStatus : uint32_t
    {
      Off = 0,
      Initial = 1,
      Clean = 2,
      Dirty = 3
    };

  template <typename URV>
  class Hart;


  /// Model a RISCV floating point register file. We use double precision
  /// representation for each register and nan-boxing for single precision
  /// and float16 values.
  class FpRegs
  {
  public:

    friend class Hart<uint32_t>;
    friend class Hart<uint64_t>;

    /// Constructor: Define a register file with the given number of
    /// registers. All registers initialized to zero.
    FpRegs(unsigned registerCount);

    /// Destructor.
    ~FpRegs()
    { regs_.clear(); }

    /// Return value of ith register.
    double readDouble(unsigned i) const
    { assert(flen_ >= 64); return regs_[i].dp; }

    /// Return the bit pattern of the ith register as an unsigned
    /// integer. If the register contains a nan-boxed value, return
    /// that value without the box.
    uint64_t readBitsUnboxed(unsigned i) const
    {
      const FpUnion& u = regs_.at(i);
      if (hasHalf_ and u.isBoxedHalf())
	return (u.i64 << 48) >> 48;

      if (hasSingle_ and u.isBoxedSingle())
	return (u.i64 << 32) >> 32;

      return u.i64;
    }

    /// Return true if given bit pattern represents a nan-boxed
    /// single precision value.
    static bool isBoxedSingle(uint64_t value)
    {
      FpUnion u{value};
      return u.isBoxedSingle();
    }

    /// Return true if given bite pattern represents a nan-boxed
    /// half precision value.
    static bool isBoxedHalf(uint64_t value)
    {
      FpUnion u{value};
      return u.isBoxedHalf();
    }

    /// Return the bit pattern of the ith register as an unsigned
    /// integer. If the register contains a nan-boxed value, do not
    /// unbox it (return the 64-bit NaN).
    uint64_t readBitsRaw(unsigned i) const
    {
      const FpUnion& u  = regs_.at(i);
      return u.i64 & mask_;
    }

    /// Set FP register i to the given value.
    void pokeBits(unsigned i, uint64_t val)
    {
      regs_.at(i) = val;
    }

    /// Set value of ith register to the given value.
    void writeDouble(unsigned i, double value)
    {
      assert(flen_ >= 64);
      originalValue_ = regs_.at(i);
      regs_[i] = value;  // Use [] instead of at for speed.
      lastWrittenReg_ = i;
    }

    /// Disallow write double using a standard conversion from float.
    void writeDouble(unsigned i, float) = delete;

    /// Read a single precision floating point number from the ith
    /// register.  If the register width is greater than 32 bits, this
    /// will recover the least significant 32 bits (it assumes that
    /// the number in the register is NAN-boxed). If the register
    /// width is 32-bit, this will simply recover the number in it.
    float readSingle(unsigned i) const;

    /// Write a single precision number into the ith register. NAN-box
    /// the number if the register is 64-bit wide.
    void writeSingle(unsigned i, float x);

    /// Disallow write double using a standard conversion from double.
    void writeSingle(unsigned i, double) = delete;

    /// Similar to readSingle but for for half precision.
    Float16 readHalf(unsigned i) const;

    /// Similar to writeSingle but for for half precision.
    void writeHalf(unsigned i, Float16 x);

    /// Similar to readSingle but for bfloat16.
    BFloat16 readBFloat16(unsigned i) const;

    /// Similar to writeSingle but for bfloat16.
    void writeBFloat16(unsigned i, BFloat16 x);

    /// Read from register i a value of type FT (Float16, float, or double).
    template <typename FT>
    FT read(unsigned i) const
    {
      if constexpr (std::is_same<FT, Float16>::value)  return readHalf(i);
      if constexpr (std::is_same<FT, float>::value)    return readSingle(i);
      if constexpr (std::is_same<FT, double>::value)   return readDouble(i);
      if constexpr (std::is_same<FT, BFloat16>::value) return readBFloat16(i);
      assert(0);
      return FT{};
    }

    /// Return the count of registers in this register file.
    size_t size() const
    { return regs_.size(); }

    /// Set ix to the number of the register corresponding to the
    /// given name returning true on success and false if no such
    /// register.  For example, if name is "f2" then ix will be set to
    /// 2. If name is "fa0" then ix will be set to 10.
    bool findReg(std::string_view name, unsigned& ix) const;

    /// Return the name of the given register.
    static constexpr std::string_view regName(unsigned i, bool abiNames = false)
    { return FpRegNames::regName(i, abiNames); }

  protected:

    void reset(bool hasHalf, bool hasSingle, bool hasDouble);

    /// Clear the number denoting the last written register.
    void clearLastWrittenReg()
    { lastWrittenReg_.reset(); lastFpFlags_ = 0; }

    /// Return the number of the last written register or -1 if no register has
    /// been written since the last clearLastWrittenReg.
    int getLastWrittenReg() const
    { return lastWrittenReg_.has_value() ? static_cast<int>(*lastWrittenReg_) : -1; }

    /// Similar to getLastWrittenReg but if successful set regValue to
    /// the prevous value of the last written register.
    int getLastWrittenReg(uint64_t& regValue) const
    {
      if (not lastWrittenReg_.has_value()) return -1;

      // Copy bits of last written value inot regValue
      regValue = originalValue_.i64;

      return static_cast<int>(*lastWrittenReg_);
    }

    /// Return the incremental floating point flag values resulting from
    /// the execution of the last instruction. Return 0 if last instructions
    /// is not an FP instruction or if it does not set any of the FP flags.
    unsigned getLastFpFlags() const
    { return lastFpFlags_; }

    /// Set the incremental FP flags produced by the last executed FP
    /// instruction.
    void setLastFpFlags(unsigned flags)
    { lastFpFlags_ = flags; }

    /// Set width of floating point register (flen). Internal
    /// representation always uses 64-bits. If flen is set to 32 then
    /// nan-boxing is not done. Return true on success and false
    /// on failure (fail if length is neither 32 or 64).
    /// Flen should not be set to 32 if D extension is enabled.
    bool setFlen(unsigned length)
    {
      if (length != 32 and length != 64)
        return false;
      flen_ = length;
      mask_ = ~uint64_t(0) >> (64 - length);
      return true;
    }

  private:

    // Union of double, single, and half precision numbers used for
    // NAN boxing.
    union FpUnion
    {
      constexpr FpUnion(double x)   : dp(x)  { }
      constexpr FpUnion(uint64_t x) : i64(x) { }
      constexpr FpUnion(float x)    : sp(x)  { i64 |= ~uint64_t(0) << 32; }
      constexpr FpUnion(Float16 x)  : hp(x)  { i64 |= ~uint64_t(0) << 16; }

      /// Return true if bit pattern corresponds to a nan-boxed single
      /// precision float.
      constexpr bool isBoxedSingle() const
      { return (i64 >> 32) == ~uint32_t(0); }

      /// Return true if bit pattern corresponds to a nan-boxed half
      /// precision (16-bit) float.
      constexpr bool isBoxedHalf() const
      { return (i64 >> 16) == (~uint64_t(0) >> 16); }

      float    sp;
      Float16  hp;
      double   dp;
      uint64_t i64;
    };

    std::vector<FpUnion> regs_;
    bool hasHalf_ = false;                    // True if half (16-bit) precision enabled.
    bool hasSingle_ = false;                  // True if F extension enabled.
    bool hasDouble_ = false;                  // True if D extension enabled.
    std::optional<unsigned> lastWrittenReg_;  // Register accessed in most recent write.
    unsigned lastFpFlags_ = 0;
    FpUnion originalValue_ = UINT64_C(0);     // Original value of last written reg.
    unsigned flen_ = 64;                      // Floating point register width.
    uint64_t mask_ = ~uint64_t(0);
  };


  inline
  float
  FpRegs::readSingle(unsigned i) const
  {
    assert(flen_ >= 32);

    const FpUnion& u = regs_.at(i);
    if (flen_ == 32 or u.isBoxedSingle())
      return u.sp;

    // Not properly boxed single, replace with NaN.
    return std::numeric_limits<float>::quiet_NaN();
  }


  inline
  void
  FpRegs::writeSingle(unsigned i, float x)
  {
    assert(flen_ >= 32);
    originalValue_ = regs_.at(i);

    regs_[i] = x;  // Use [] instead of at for speed.
    lastWrittenReg_ = i;
  }


  inline
  Float16
  FpRegs::readHalf(unsigned i) const
  {
    assert(flen_ >= 16);

    const FpUnion& u = regs_.at(i);
    if (flen_ == 16 or u.isBoxedHalf())
      return u.hp;

    return std::numeric_limits<Float16>::quiet_NaN();
  }


  inline
  void
  FpRegs::writeHalf(unsigned i, Float16 x)
  {
    assert(flen_ >= 16);
    originalValue_ = regs_.at(i);

    regs_[i] = x;  // Use [] instead of at for speed.
    lastWrittenReg_ = i;
  }


  inline
  BFloat16
  FpRegs::readBFloat16(unsigned i) const
  {
    assert(flen_ >= 16);

    const FpUnion& u = regs_.at(i);
    if (flen_ == 16 or u.isBoxedHalf())
      return std::bit_cast<BFloat16>(u.hp);

    return std::numeric_limits<BFloat16>::quiet_NaN();
  }


  inline
  void
  FpRegs::writeBFloat16(unsigned i, BFloat16 x)
  {
    writeHalf(i, std::bit_cast<Float16>(x));
  }


  /// Classify given floating point value (see std::fpclassify)
  /// returning the classifications in the least significant 10 bits
  /// of the result according to the RISCV spec. FT must be one of
  /// float, double, or Float16.
  template <typename FT>
  unsigned
  fpClassifyRiscv(FT val);

}
