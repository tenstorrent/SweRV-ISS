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

#include "FpRegs.hpp"
#include "softfloat-util.hpp"

using namespace WdRiscv;


float
Float16::toFloat() const
{
#ifdef SOFT_FLOAT

  auto sf16 = nativeToSoft(*this);
  auto sf32 = f16_to_f32(sf16);
  return softToNative(sf32);

#else

  bool sign = (i16 >> 15) & 1;
  if (isInf())
    {
      float x = std::numeric_limits<float>::infinity();
      return sign? -x : x;
    }

  if (isSnan())
    {
      float x = std::numeric_limits<float>::signaling_NaN();
      return sign? -x : x;
    }

  if (isNan())
    {
      float x = std::numeric_limits<float>::quiet_NaN();
      return sign? -x : x;
    }

  if (isZero())
    return sign? -0.0f : 0.0f;

  if (isSubnormal())
    {
      // Subnormal in half precision would be normal in float.
      // Renormalize.
      uint32_t sig = sigBits();
      assert(sig != 0);
      uint32_t exp = expBits();
      unsigned mssb = 8*sizeof(sig) - 1 - __builtin_clz(sig);  // Most sig set bit
      assert(mssb <= 9);
      unsigned shift = 10 - mssb;
      sig = sig & ~(uint32_t(1) << shift);  // Clear most sig bit
      sig = sig << shift;
      exp = exp - shift;
      exp = exp - 15 + 127;  // Update bias
      uint32_t val = sign? 1 : 0;
      val = (val << 31) | (exp << 23) | sig;
      Uint32FloatUnion uf{val};
      return uf.f;
    }

  // Normalized number. Update exponent for float bias.
  uint32_t sig = sigBits();
  uint32_t exp = expBits();
  exp = exp - 15 + 127;
  uint32_t val = sign? 1 : 0;
  val = (val << 31) | (exp << 23) | sig;
  Uint32FloatUnion uf{val};
  return uf.f;

#endif
}


Float16
Float16::fromFloat(float val)
{
#ifdef SOFT_FLOAT

  auto sf32 = nativeToSoft(val);
  auto sf16 = f32_to_f16(sf32);
  return softToNative(sf16);

#else

  bool sign = std::signbit(val);
  if (std::isinf(val))
    {
      Float16 x = Float16::infinity();
      return sign? -x : x;
    }

  if (std::isnan(val))
    {
      Float16 x = WdRiscv::isSnan(val)? Float16::signalingNan() : Float16::quietNan();
      return sign? -x : x;
    }

  if (val == 0 or not std::isnormal(val))
    return sign? -Float16{} : Float16{};

  // Normalized float. Update exponent for float16 bias.
  Uint32FloatUnion uf{val};

  uint32_t sig = (uf.u << 9) >> 9;
  int exp = ((uf.u) >> 23) & 0xff;
  exp = exp - 127 + 15;
  if (exp < -10)
    return sign? -Float16{} : Float16{};
  if (exp < 0)
    {
      // In Float16 number would be subnormal. Adjust.
      int shift = -exp;
      sig = sig | (1 << 23);  // Put back implied most sig digit.
      sig = sig >> shift;
      exp = 0;
    }
  if (exp >= 0x1f)
    return sign? -Float16::infinity() : Float16::infinity();

  uint16_t res = sign? 1 : 0;
  res = (res << 15) | uint16_t(exp << 10) | uint16_t(sig >> 13);

  return Float16::fromBits(res);

#endif
}


FpRegs::FpRegs(unsigned regCount)
  : regs_(regCount, 0)
{
}


bool
FpRegs::findReg(const std::string& name, unsigned& ix) const
{
  unsigned i = 0;
  if (not regNames_.findReg(name, i))
    return false;

  if (i >= regs_.size())
    return false;

  ix = i;
  return true;
}


void
FpRegs::reset(bool hasHalf, bool hasSingle, bool hasDouble)
{
  hasHalf_ = hasHalf;
  hasSingle_ = hasSingle;
  hasDouble_ = hasDouble;

  if (hasDouble)
    {
      for (auto& reg : regs_)
	reg = 0;
    }
  else if (hasSingle)
    {
      // F extension present without D. Reset to NAN-boxed
      // single-precision zeros.
      for (size_t i = 0; i < regs_.size(); ++i)
	writeSingle(i, 0.0f);
    }
  else if (hasHalf)
    {
      // F16 extension present without F or D. Reset to NAN-boxed
      // half-precision zeros.
      for (size_t i = 0; i < regs_.size(); ++i)
	writeHalf(i, Float16());
    }

  clearLastWrittenReg();
}
