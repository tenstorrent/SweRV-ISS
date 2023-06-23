// Copyright 2023 Tenstorrent Corporation.
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

#include <iostream>
#include <cfenv>
#include <cmath>
#include <climits>
#include <cassert>
#include "wideint.hpp"
#include "instforms.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"


using namespace WdRiscv;


/// Function operator to compute bitwise a and not b (a & ~b).
struct MyAndn 
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  { return a & ~b; }
};



template <typename URV>
void
Hart<URV>::execVandn_vv(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked, MyAndn());
      break;
    case EW::Half:
      vop_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked, MyAndn());
      break;
    case EW::Word:
      vop_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked, MyAndn());
      break;
    case EW::Word2:
      vop_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked, MyAndn());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVandn_vx(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<int8_t> (vd, vs1, e2, group, start, elems, masked, MyAndn());
      break;
    case EW::Half:
      vop_vx<int16_t>(vd, vs1, e2, group, start, elems, masked, MyAndn());
      break;
    case EW::Word:
      vop_vx<int32_t>(vd, vs1, e2, group, start, elems, masked, MyAndn());
      break;
    case EW::Word2:
      vop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, MyAndn());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template<typename T>
constexpr T
bitReverse(T x)
{
  T result{0};
  unsigned bitCount = sizeof(T)*8;
  for (unsigned i = 0; i < bitCount; ++i, x >>= 1)
    result |= (x & T{1}) << (bitCount - 1 - i);
  return result;
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vbrev_v(unsigned vd, unsigned vs1, unsigned group,
		   unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  (void)errors;
  ELEM_TYPE e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = bitReverse(e1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVbrev_v(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vbrev_v<uint8_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      vbrev_v<uint16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      vbrev_v<uint32_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word2:
      vbrev_v<uint64_t>(vd, vs1, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename T>
constexpr T brev8(const T& x)
{
  T res{0};
  for (unsigned byteIx = 0; byteIx < sizeof(x); ++byteIx)
    {
      uint8_t byte = static_cast<uint8_t>(x >> 8*byteIx);
      byte = bitReverse(byte);
      res |= T{byte} << 8*byteIx;
    }
  return res;
}



template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vbrev8_v(unsigned vd, unsigned vs1, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  (void)errors;
  ELEM_TYPE e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  dest = brev8(e1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVbrev8_v(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vbrev8_v<uint8_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      vbrev8_v<uint16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      vbrev8_v<uint32_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word2:
      vbrev8_v<uint64_t>(vd, vs1, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vrev8_v(unsigned vd, unsigned vs1, unsigned group,
		   unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  (void)errors;
  ELEM_TYPE e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = util::byteswap(e1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVrev8_v(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vrev8_v<uint8_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      vrev8_v<uint16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      vrev8_v<uint32_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word2:
      vrev8_v<uint64_t>(vd, vs1, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vclz_v(unsigned vd, unsigned vs1, unsigned group,
		  unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  (void)errors;
  ELEM_TYPE e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = std::countl_zero(e1);  // Count leading zeros.
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVclz_v(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vclz_v<uint8_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      vclz_v<uint16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      vclz_v<uint32_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word2:
      vclz_v<uint64_t>(vd, vs1, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vctz_v(unsigned vd, unsigned vs1, unsigned group,
		  unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  (void)errors;
  ELEM_TYPE e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = std::countr_zero(e1);  // Count trailing zeros.
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVctz_v(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vctz_v<uint8_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      vctz_v<uint16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      vctz_v<uint32_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word2:
      vctz_v<uint64_t>(vd, vs1, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vcpop_v(unsigned vd, unsigned vs1, unsigned group,
		   unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  (void)errors;
  ELEM_TYPE e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = std::popcount(e1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVcpop_v(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vcpop_v<uint8_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      vcpop_v<uint16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      vcpop_v<uint32_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word2:
      vcpop_v<uint64_t>(vd, vs1, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


/// Function operator to rotate a left by b bits. Only the least sig
/// n bits of b are used, n is log2(width of T1). For example, if T1 is
/// uint32_t then only least sig 5 bits of b are used.
struct
MyRol
{
  template <typename T1, typename T2>
  constexpr T1 operator() (const T1& a, const T2& b) const
  {
    unsigned width = sizeof(T1)*8;  // Bit count of T1
    unsigned mask = width - 1;
    unsigned amount = unsigned(b & mask);
    return (a << amount) | (a >> ((width - amount) & mask));
  }
};


template <typename URV>
void
Hart<URV>::execVrol_vv(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked, MyRol());
      break;
    case EW::Half:
      vop_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked, MyRol());
      break;
    case EW::Word:
      vop_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked, MyRol());
      break;
    case EW::Word2:
      vop_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked, MyRol());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVrol_vx(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked, MyRol());
      break;
    case EW::Half:
      vop_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked, MyRol());
      break;
    case EW::Word:
      vop_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked, MyRol());
      break;
    case EW::Word2:
      vop_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked, MyRol());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


/// Function operator to rotate a right by b bits.
/// Only the least sig n bits of b are used, n is log2(width of T1).
struct
MyRor
{
  template <typename T1, typename T2>
  constexpr T1 operator() (const T1& a, const T2& b) const
  {
    unsigned width = sizeof(T1)*8;  // Bit count of T1
    unsigned mask = width - 1;
    unsigned amount = unsigned(b & mask);
    return (a >> amount) | (a << ((width - amount) & mask));
  }
};


template <typename URV>
void
Hart<URV>::execVror_vv(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked, MyRor());
      break;
    case EW::Half:
      vop_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked, MyRor());
      break;
    case EW::Word:
      vop_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked, MyRor());
      break;
    case EW::Word2:
      vop_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked, MyRor());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVror_vx(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked, MyRor());
      break;
    case EW::Half:
      vop_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked, MyRor());
      break;
    case EW::Word:
      vop_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked, MyRor());
      break;
    case EW::Word2:
      vop_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked, MyRor());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVror_vi(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  int32_t imm = di->op2As<int32_t>();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  URV e2 = imm;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked, MyRor());
      break;
    case EW::Half:
      vop_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked, MyRor());
      break;
    case EW::Word:
      vop_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked, MyRor());
      break;
    case EW::Word2:
      vop_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked, MyRor());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


/// Function operator to shift a left by b bits.
/// Only the least sig n bits of b are used, n is log2(width of T).
struct
MySll
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  {
    unsigned width = sizeof(T)*8;  // Bit count of T
    unsigned mask = width - 1;
    unsigned amount = unsigned(b & mask);
    return a >> amount;
  }
};


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwsll_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  using DWT = typename makeDoubleWide<ELEM_TYPE>::type; // Double wide type
  unsigned errors = 0, wideGroup = group*2;
  (void)errors;

  ELEM_TYPE e1 = 0, e2 = 0;
  DWT dest = 0;

  MySll mySll;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
	  dest = mySll(DWT(e1), DWT(e2));
          if (not vecRegs_.write(vd, ix, wideGroup, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVwsll_vv(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vwsll_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Half:
      vwsll_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vwsll_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vwsll_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();

}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwsll_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  using DWT = typename makeDoubleWide<ELEM_TYPE>::type; // Double wide type
  unsigned errors = 0, wideGroup = group*2;
  (void)errors;

  ELEM_TYPE e1 = 0;
  DWT dest = 0;

  MySll mySll;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = mySll(DWT(e1), DWT(e2));
          if (not vecRegs_.write(vd, ix, wideGroup, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVwsll_vx(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  URV e2 = SRV(intRegs_.read(rs2));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vwsll_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked);
      break;
    case EW::Half:
      vwsll_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked);
      break;
    case EW::Word:
      vwsll_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked);
      break;
    case EW::Word2:
      vwsll_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVwsll_vi(const DecodedInst* di)
{
  if (not isRvzvbb())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  uint32_t imm = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  URV e2 = imm;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vwsll_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked);
      break;
    case EW::Half:
      vwsll_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked);
      break;
    case EW::Word:
      vwsll_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked);
      break;
    case EW::Word2:
      vwsll_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


/// Function operator to perform carry-less multiply of a and b.
struct
MyClmul
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  {
    unsigned width = sizeof(T)*8;  // Bit count of T
    T res{0};
    for (unsigned i = 0; i < width; ++i)
      if ((b >> i) & 1)
	res ^= a << i;
    return res;
  }
};


template <typename URV>
void
Hart<URV>::execVclmul_vv(const DecodedInst* di)
{
  if (not isRvzvbc())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  
  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Word2:
      vop_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked, MyClmul());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVclmul_vx(const DecodedInst* di)
{
  if (not isRvzvbc())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Word2:
      vop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, MyClmul());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


/// Function operator to perform carry-less multiply of a and b and
/// return the upper w bits of the product. W is the width of T.
struct
MyClmulh
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  {
    unsigned width = sizeof(T)*8;  // Bit count of T
    T res{0};
    for (unsigned i = 0; i < width; ++i)
      if ((b >> i) & 1)
	res ^= (a >> (width - i));
    return res;
  }
};


template <typename URV>
void
Hart<URV>::execVclmulh_vv(const DecodedInst* di)
{
  if (not isRvzvbc())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  
  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Word2:
      vop_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked, MyClmulh());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVclmulh_vx(const DecodedInst* di)
{
  if (not isRvzvbc())
    {
      illegalInst(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Word2:
      vop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, MyClmulh());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


struct
MyGhsh
{
};  


template <typename URV>
void
Hart<URV>::execVghsh_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkg() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 x{0}, y{0}, h{0}, z{0};
      if (not vecRegs_.read(vd, i, groupx8, y))
	assert(0);
      if (not vecRegs_.read(vs2, i, groupx8, x))
	assert(0);
      if (not vecRegs_.read(vs1, i, groupx8, h))
	assert(0);
      Uint128 s = brev8(y ^ x);

      for (unsigned bit = 0; bit < 128; bit++) {
        if ((s >> static_cast<int>(i)) & 1U)
	  z ^= h;

	bool reduce = ((h >> 127) & 1) != 0;
        h <<= 1;
        if (reduce)
          h ^= 0x87;
      }
      Uint128 res = brev8(z);
      if (not vecRegs_.write(vd, i, groupx8, res))
	assert(0);
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVgmul_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkg() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 y{0}, h{0}, z{0};
      if (not vecRegs_.read(vd, i, groupx8, y))
	assert(0);
      if (not vecRegs_.read(vs1, i, groupx8, h))
	assert(0);
      for (unsigned bit = 0; bit < 128; bit++)
	{
	  if ((y >> static_cast<int>(bit)) & 1U)
	    z ^= h;
	  bool reduce = ((h >> 127) & 1) != 0;
	  h <<= 1;
	  if (reduce)
	    h ^= 0x87;
	}
      Uint128 res = brev8(z);
      vecRegs_.write(vd, i, groupx8, res);
    }

  postVecSuccess();
}


extern Uint128
aes_shift_rows_inv(Uint128 x);

extern Uint128
aes_subbytes_inv(Uint128 x);

extern Uint128
aes_subbytes_fwd(Uint128 x);

extern Uint128
aes_shift_rows_fwd(Uint128 x);

extern Uint128
aes_mixcolumns_fwd(Uint128 x);;

extern uint32_t
aes_subword_fwd(uint32_t x);

extern uint32_t
aes_decode_rcon(uint8_t r);

extern uint32_t
sm4_subword(uint32_t x);


template <typename URV>
void
Hart<URV>::execVaesdf_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 state{0}, rkey{0};
      if (not vecRegs_.read(vd, i, groupx8, state))
	assert(0);
      if (not vecRegs_.read(vs1, i, groupx8, rkey))
	assert(0);
      Uint128 sr = aes_shift_rows_inv(state);
      Uint128 sb = aes_subbytes_inv(sr);
      Uint128 ark = sb ^ rkey;
      if (not vecRegs_.write(vd, i, groupx8, ark))
	assert(0);
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVaesdf_vs(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 state{0}, rkey{0};
      if (not vecRegs_.read(vd, i, groupx8, state))
	assert(0);
      if (not vecRegs_.read(vs1, 0, groupx8, rkey))
	assert(0);
      Uint128 sr = aes_shift_rows_inv(state);
      Uint128 sb = aes_subbytes_inv(sr);
      Uint128 ark = sb ^ rkey;
      if (not vecRegs_.write(vd, i, groupx8, ark))
	assert(0);
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVaesef_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 state{0}, rkey{0};
      if (not vecRegs_.read(vd, i, groupx8, state))
	assert(0);
      if (not vecRegs_.read(vs1, i, groupx8, rkey))
	assert(0);
      Uint128 sb = aes_subbytes_fwd(state);
      Uint128 sr = aes_shift_rows_fwd(sb);
      Uint128 ark = sr ^ rkey;
      if (not vecRegs_.write(vd, i, groupx8, ark))
	assert(0);
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVaesef_vs(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 state{0}, rkey{0};
      if (not vecRegs_.read(vd, i, groupx8, state))
	assert(0);
      if (not vecRegs_.read(vs1, 0, groupx8, rkey))
	assert(0);
      Uint128 sb = aes_subbytes_fwd(state);
      Uint128 sr = aes_shift_rows_fwd(sb);
      Uint128 ark = sr ^ rkey;
      if (not vecRegs_.write(vd, i, groupx8, ark))
	assert(0);
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVaesem_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 state{0}, rkey{0};
      if (not vecRegs_.read(vd, i, groupx8, state))
	assert(0);
      if (not vecRegs_.read(vs1, i, groupx8, rkey))
	assert(0);
      Uint128 sb = aes_subbytes_fwd(state);
      Uint128 sr = aes_shift_rows_fwd(sb);
      Uint128 mix = aes_mixcolumns_fwd(sr);
      Uint128 ark = mix ^ rkey;
      if (not vecRegs_.write(vd, i, groupx8, ark))
	assert(0);
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVaesem_vs(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 state{0}, rkey{0};
      if (not vecRegs_.read(vd, i, groupx8, state))
	assert(0);
      if (not vecRegs_.read(vs1, 0, groupx8, rkey))
	assert(0);
      Uint128 sb = aes_subbytes_fwd(state);
      Uint128 sr = aes_shift_rows_fwd(sb);
      Uint128 mix = aes_mixcolumns_fwd(sr);
      Uint128 ark = mix ^ rkey;
      if (not vecRegs_.write(vd, i, groupx8, ark))
	assert(0);
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVaesdm_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 state{0}, rkey{0};
      if (not vecRegs_.read(vd, i, groupx8, state))
	assert(0);
      if (not vecRegs_.read(vs1, i, groupx8, rkey))
	assert(0);
      Uint128 sr = aes_shift_rows_inv(state);
      Uint128 sb = aes_subbytes_inv(sr);
      Uint128 ark = sb ^ rkey;
      Uint128 mix = aes_mixcolumns_fwd(ark);
      if (not vecRegs_.write(vd, i, groupx8, mix))
	assert(0);
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVaesdm_vs(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 state{0}, rkey{0};
      if (not vecRegs_.read(vd, i, groupx8, state))
	assert(0);
      if (not vecRegs_.read(vs1, 0, groupx8, rkey))
	assert(0);
      Uint128 sr = aes_shift_rows_inv(state);
      Uint128 sb = aes_subbytes_inv(sr);
      Uint128 ark = sb ^ rkey;
      Uint128 mix = aes_mixcolumns_fwd(ark);
      if (not vecRegs_.write(vd, i, groupx8, mix))
	assert(0);
    }

  postVecSuccess();
}


/// Rotate a word right by 8 bits.
uint32_t
aes_rotword(uint32_t x)
{
  return x >> 8 | ((x & 0xff) << 24);
}


template <typename URV>
void
Hart<URV>::execVaeskf1_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1(),  round = di->op2();
  if (round > 10 or round == 0)
    round ^= 0x8; // Flip bit 3.
  unsigned r = round - 1;

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 e1{0};
      if (not vecRegs_.read(vs1, i, groupx8, e1))
	assert(0);
      uint32_t crk0 = static_cast<uint32_t>(e1);
      uint32_t crk1 = static_cast<uint32_t>(e1 >> 32);
      uint32_t crk2 = static_cast<uint32_t>(e1 >> 64);
      uint32_t crk3 = static_cast<uint32_t>(e1 >> 96);
      uint32_t w0 = aes_subword_fwd(aes_rotword(crk3)) ^ aes_decode_rcon(r) ^ crk0;
      uint32_t w1 = w0 ^ crk1;
      uint32_t w2 = w1 ^ crk2;
      uint32_t w3 = w2 ^ crk3;

      Uint128 res = (Uint128(w0) | (Uint128(w1) << 32) |
                     (Uint128(w2) << 64) | (Uint128(w3) << 96));
      if (not vecRegs_.write(vd, i, groupx8, res))
	assert(0);
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVaeskf2_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1(),  round = di->op2();
  if (round > 10 or round == 0)
    round ^= 0x8; // Flip bit 3.

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 e1{0}, d{0};
      if (not vecRegs_.read(vs1, i, groupx8, e1))
	assert(0);
      uint32_t crk3 = static_cast<uint32_t>(e1 >> 96);

      if (not vecRegs_.read(vs1, i, groupx8, d))
	assert(0);
      uint32_t rkb0 = static_cast<uint32_t>(d);
      uint32_t rkb1 = static_cast<uint32_t>(d >> 32);
      uint32_t rkb2 = static_cast<uint32_t>(d >> 64);
      uint32_t rkb3 = static_cast<uint32_t>(d >> 96);

      uint32_t w0 = (round & 1) ? aes_subword_fwd(crk3) & rkb0 :
	aes_subword_fwd(aes_rotword(crk3)) ^ aes_decode_rcon(round >> 1) ^ rkb0;
      uint32_t w1 = w0 ^ rkb1;
      uint32_t w2 = w1 ^ rkb2;
      uint32_t w3 = w2 ^ rkb3;

      Uint128 res = (Uint128(w0) | (Uint128(w1) << 32) |
                     (Uint128(w2) << 64) | (Uint128(w3) << 96));
      if (not vecRegs_.write(vd, i, groupx8, res))
	assert(0);
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVaesz_vs(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 state{0}, rkey{0};
      if (not vecRegs_.read(vd, i, groupx8, state))
	assert(0);
      if (not vecRegs_.read(vs1, i, groupx8, rkey))
	assert(0);

      Uint128 ark = state ^ rkey;
      if (not vecRegs_.write(vd, i, groupx8, ark))
	assert(0);
    }

  postVecSuccess();
}


template <typename T>
constexpr T
rotateRight(T x, unsigned n)
{
  unsigned width = sizeof(x) * 8;
  if (n == width)
    return x;
  assert(n < width);
  return (x >> n) | (x << (width - n));
}


constexpr uint32_t
sig0(uint32_t x)
{
  return rotateRight(x, 7) ^ rotateRight(x, 18) ^ (x >> 3);
}


constexpr uint64_t
sig0(uint64_t x)
{
  return rotateRight(x, 1) ^ rotateRight(x, 8) ^ (x >> 7);
}


constexpr uint32_t
sig1(uint32_t x)
{
  return rotateRight(x, 17) ^ rotateRight(x, 19) ^ (x >> 10);
}


constexpr uint64_t
sig1(uint64_t x)
{
  return rotateRight(x, 19) ^ rotateRight(x, 61) ^ (x >> 6);
}


template <typename ET, typename ET4>
void
vsha2ms(ET4& dd, ET4& e1, ET4& e2)
{
  unsigned etw = sizeof(ET)*8;  // element type width
  ET w0{ET(dd)}, w1{ET(dd >> etw)}, w2{ET(dd >> 2*etw)}, w3{ET(dd >> 3*etw)};
  ET w4{ET(e1)}, w9{ET(e1 >> etw)}, w10 {ET(e1 >> 2*etw)}, w11{ET(e1 >> 3*etw)};
  ET w12{ET(e2)}, /* w13{ET(e2 >> etw)}, */ w14{ET(e2 >> 2*etw)}, w15{ET(e2 >> 3*etw)};
  ET w16 = sig1(w14) + w9  + sig0(w1) + w0;
  ET w17 = sig1(w15) + w10 + sig0(w2) + w1;
  ET w18 = sig1(w16) + w11 + sig0(w3) + w2;
  ET w19 = sig1(w17) + w12 + sig0(w4) + w3;
  dd = (ET4(w16) | (ET4(w17) << etw) | (ET4(w18) << 2*etw) | (ET4(w19) << 3*etw) );
}


template <typename URV>
void
Hart<URV>::execVsha2ms_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 4*vecRegs_.elemWidthInBits(), egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  if (group*vecRegs_.bitsPerRegister() < egw or (elems % egs) or (start % egs) or
      (not isRvzvknha() and not isRvzvknhb()) or
	      (vs1 + group > vd and vd + group > vs1) or
      (vs2 + group > vd and vd + group > vs2) or
      (isRvzvknha() and sew != EW::Word) or
      (isRvzvknhb() and sew != EW::Word and sew != EW::Word2))
    {
      illegalInst(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  if (sew == EW::Word)
    {
      for (unsigned i = egStart; i < egLen; ++i)
	{
	  Uint128 dd{0}, e1{0}, e2{0};
	  if (not vecRegs_.read(vd, i, groupx8, dd)) assert(0);
	  if (not vecRegs_.read(vd, i, groupx8, e1)) assert(0);
	  if (not vecRegs_.read(vd, i, groupx8, e2)) assert(0);
	  vsha2ms<uint32_t, Uint128>(dd, e1, e2);
	  vecRegs_.write(vd, i, groupx8, dd);
	}
    }
  else
    {
      for (unsigned i = egStart; i < egLen; ++i)
	{
	  Uint256 dd{0}, e1{0}, e2{0};
	  if (not vecRegs_.read(vd, i, groupx8, dd)) assert(0);
	  if (not vecRegs_.read(vd, i, groupx8, e1)) assert(0);
	  if (not vecRegs_.read(vd, i, groupx8, e2)) assert(0);
	  vsha2ms<uint64_t, Uint256>(dd, e1, e2);
	  vecRegs_.write(vd, i, groupx8, dd);
	}
    }

  postVecSuccess();
}


static constexpr
uint32_t
sum0(uint32_t x)
{
  MyRor ror;
  return ror(x, 2) ^ ror(x, 13) ^ ror(x, 22);
}

static constexpr
uint64_t
sum0(uint64_t x)
{
  MyRor ror;
  return ror(x, 28) ^ ror(x, 34) ^ ror(x, 39);
}


static constexpr
uint32_t
sum1(uint32_t x)
{
  MyRor ror;
  return ror(x, 6) ^ ror(x, 11) ^ ror(x, 25);
}


static constexpr
uint64_t
sum1(uint64_t x)
{
  MyRor ror;
  return ror(x, 14) ^ ror(x, 18) ^ ror(x, 41);
}


static constexpr
uint32_t
ch(uint32_t x, uint32_t y, uint32_t z)
{
  return (x & y) ^ ((~x) & z);
}


static constexpr
uint64_t
ch(uint64_t x, uint64_t y, uint64_t z)
{
  return (x & y) ^ ((~x) & z);
}


static constexpr
uint32_t
maj(uint32_t x, uint32_t y, uint32_t z)
{
  return (x & y) ^ (x & z) ^ (y & z);
}


static constexpr
uint64_t
maj(uint64_t x, uint64_t y, uint64_t z)
{
  return (x & y) ^ (x & z) ^ (y & z);
}


template <typename ET, typename ET4>
void
vsha2c(ET4& dd, ET4& e1, ET4& e2, bool high)
{
  unsigned etw = sizeof(ET)*8;  // element type width
  ET f{ET(e1)}, e{ET(e1 >> etw)}, b{ET(e1 >> 2*etw)}, a{ET(e1 >> 3*etw)};
  ET h{ET(dd)}, g{ET(dd >> etw)}, d{ET(dd >> 2*etw)}, c{ET(dd >> 3*etw)};

  ET m0{ET(e2)}, m1{ET(e2 >> etw)}, m2{ET(e2 >> 2*etw)}, m3{ET(e2 >> 3*etw)};

  ET w0{}, w1{};
  if (high)
    {
      w0 = m2;
      w1 = m3;
    }
  else
    {
      w0 = m0;
      w1 = m1;
    }
  ET t1 = h + sum1(e) + ch(e, f, g) + w0;
  ET t2 = sum0(a) + maj(a, b, c);
  h  = g;
  g  = f;
  f  = e;
  e  = d + t1;
  d  = c;
  c  = b;
  b  = a;
  a  = t1 + t2;
  t1  = h + sum1(e) + ch(e, f, g) + w1;
  t2  = sum0(a) + maj(a, b, c);
  h = g;
  g = f;
  f = e;
  e = d + t1;
  d = c;
  c = b;
  b = a;
  a = t1 + t2;
  dd = ET4{f} | (ET4{e} << etw) | (ET4{b} << 2*etw) | (ET4{f} << 3*etw);
}

template <typename URV>
void
Hart<URV>::execVsha2ch_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 4*vecRegs_.elemWidthInBits(), egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  if (group*vecRegs_.bitsPerRegister() < egw or (elems % egs) or (start % egs) or
      (not isRvzvknha() and not isRvzvknhb()) or
      (vs1 + group > vd and vd + group > vs1) or
      (vs2 + group > vd and vd + group > vs2) or
      (isRvzvknha() and sew != EW::Word) or
      (isRvzvknhb() and sew != EW::Word and sew != EW::Word2))
    {
      illegalInst(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  switch (sew)
    {
    case EW::Word:
      for (unsigned i = egStart; i < egLen; ++i)
	{
	  Uint128 e1{0}, e2{0}, dd{0};
	  if (not vecRegs_.read(vd, i, groupx8, dd)) assert(0);
	  if (not vecRegs_.read(vs1, i, groupx8, e1)) assert(0);
	  if (not vecRegs_.read(vs2, i, groupx8, e2)) assert(0);
	  vsha2c<uint32_t, Uint128>(dd, e1, e2, true);
	  if (not vecRegs_.write(vd, i, groupx8, dd)) assert(0);
	}
      break;
    case EW::Word2:
      for (unsigned i = egStart; i < egLen; ++i)
	{
	  Uint256 e1{0}, e2{0}, dd{0};
	  if (not vecRegs_.read(vd, i, groupx8, dd)) assert(0);
	  if (not vecRegs_.read(vs1, i, groupx8, e1)) assert(0);
	  if (not vecRegs_.read(vs2, i, groupx8, e2)) assert(0);
	  vsha2c<uint64_t, Uint256>(dd, e1, e2, true);
	  if (not vecRegs_.write(vd, i, groupx8, dd)) assert(0);
	}
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsha2cl_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 4*vecRegs_.elemWidthInBits(), egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  if (group*vecRegs_.bitsPerRegister() < egw or (elems % egs) or (start % egs) or
      (not isRvzvknha() and not isRvzvknhb()) or
      (vs1 + group > vd and vd + group > vs1) or
      (vs2 + group > vd and vd + group > vs2) or
      (isRvzvknha() and sew != EW::Word) or
      (isRvzvknhb() and sew != EW::Word and sew != EW::Word2))
    {
      illegalInst(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  switch (sew)
    {
    case EW::Word:
      for (unsigned i = egStart; i < egLen; ++i)
	{
	  Uint128 e1{0}, e2{0}, dd{0};
	  if (not vecRegs_.read(vd, i, groupx8, dd)) assert(0);
	  if (not vecRegs_.read(vs1, i, groupx8, e1)) assert(0);
	  if (not vecRegs_.read(vs2, i, groupx8, e2)) assert(0);
	  vsha2c<uint32_t, Uint128>(dd, e1, e2, false);
	  if (not vecRegs_.write(vd, i, groupx8, dd)) assert(0);
	}
      break;
    case EW::Word2:
      for (unsigned i = egStart; i < egLen; ++i)
	{
	  Uint256 e1{0}, e2{0}, dd{0};
	  if (not vecRegs_.read(vd, i, groupx8, dd)) assert(0);
	  if (not vecRegs_.read(vs1, i, groupx8, e1)) assert(0);
	  if (not vecRegs_.read(vs2, i, groupx8, e2)) assert(0);
	  vsha2c<uint64_t, Uint256>(dd, e1, e2, false);
	  if (not vecRegs_.write(vd, i, groupx8, dd)) assert(0);
	}
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


static constexpr
uint32_t
round_key(uint32_t x, uint32_t s)
{
  MyRol rol;
  return x ^ s ^ rol(s, 13) ^ rol(s, 23);
}
	  

static constexpr
auto ck = std::to_array<uint32_t>({
  0x00070E15, 0x1C232A31, 0x383F464D, 0x545B6269,
  0x70777E85, 0x8C939AA1, 0xA8AFB6BD, 0xC4CBD2D9,
  0xE0E7EEF5, 0xFC030A11, 0x181F262D, 0x343B4249,
  0x50575E65, 0x6C737A81, 0x888F969D, 0xA4ABB2B9,
  0xC0C7CED5, 0xDCE3EAF1, 0xF8FF060D, 0x141B2229,
  0x30373E45, 0x4C535A61, 0x686F767D, 0x848B9299,
  0xA0A7AEB5, 0xBCC3CAD1, 0xD8DFE6ED, 0xF4FB0209,
  0x10171E25, 0x2C333A41, 0x484F565D, 0x646B7279
});
static_assert(ck.size() == 32);


template <typename URV>
void
Hart<URV>::execVsm4k_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvksed() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1(),  imm = di->op2();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;
  uint32_t rnd = imm & 7; // Lower 3 bits

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 e1{0};
      if (not vecRegs_.read(vs1, i, groupx8, e1)) assert(0);
      uint32_t rk0 = uint32_t(e1), rk1 = uint32_t(e1 >> 32), rk2 = uint32_t(e1 >> 64),
	rk3 = uint32_t(e1 >> 96);

      uint32_t b = rk1 ^ rk2 ^ rk3 ^ ck[std::size_t{4} * rnd];
      uint32_t s = sm4_subword(b);
      uint32_t rk4 = round_key(rk0, s);

      b = rk2 ^ rk3 ^ rk4 ^ ck[std::size_t{4} * rnd + 1];
      s = sm4_subword(b);
      uint32_t rk5 = round_key(rk1, s);

      b = rk3 ^ rk4 ^ rk5 ^ ck[std::size_t{4} * rnd + 2];
      s = sm4_subword(b);
      uint32_t rk6 = round_key(rk2, s);

      b = rk4 ^ rk5 ^ rk6 ^ ck[std::size_t{4} * rnd + 3];
      s = sm4_subword(b);
      uint32_t rk7 = round_key(rk3, s);

      Uint128 dd = (Uint128{rk4} | (Uint128{rk5} << 32) | (Uint128{rk6} << 64) |
		    (Uint128{rk7} << 86));
      if (not vecRegs_.write(vd, i, groupx8, dd))
	assert(0);
    }

  postVecSuccess();
}


static constexpr
uint32_t
sm4_round(uint32_t x, uint32_t s)
{
  MyRol rol;
  return x ^ s ^ rol(s, 2) ^ rol(s, 10) ^ rol(s, 18) ^ rol(s, 24);
}


template <typename URV>
void
Hart<URV>::execVsm4r_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvksed() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 e1{}, dd{};
      if (not vecRegs_.read(vs1, i, groupx8, e1)) assert(0);
      if (not vecRegs_.read(vd, i, groupx8, dd)) assert(0);

      uint32_t rk0 = uint32_t(e1), rk1 = uint32_t(e1 >> 32), rk2 = uint32_t(e1 >> 64),
	rk3 = uint32_t(e1 >> 96);

      uint32_t x0 = uint32_t(dd), x1 = uint32_t(dd >> 32), x2 = uint32_t(dd >> 64),
	x3 = uint32_t(dd >> 96);

      uint32_t b  = x1 ^ x2 ^ x3 ^ rk0;
      uint32_t s = sm4_subword(b);
      uint32_t x4 = sm4_round(x0, s);

      b = x2 ^ x3 ^ x4 ^ rk1;
      s = sm4_subword(b);
      uint32_t x5 = sm4_round(x1, s);

      b = x3 ^ x4 ^ x5 ^ rk2;
      s = sm4_subword(b);
      uint32_t x6 = sm4_round(x2, s);

      b = x4 ^ x5 ^ x6 ^ rk3;
      s = sm4_subword(b);
      uint32_t x7 = sm4_round(x3, s);

      dd = Uint128{x4} | (Uint128{x5} << 32) | (Uint128{x6} << 64) | (Uint128{x7} << 96);
      if (not vecRegs_.write(vd, i, groupx8, dd))
	assert(0);
    }
      
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsm4r_vs(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvksed() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs))
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 e1{}, dd{};
      if (not vecRegs_.read(vs1, 0, groupx8, e1)) assert(0);
      if (not vecRegs_.read(vd, i, groupx8, dd)) assert(0);

      uint32_t rk0 = uint32_t(e1), rk1 = uint32_t(e1 >> 32), rk2 = uint32_t(e1 >> 64),
	rk3 = uint32_t(e1 >> 96);

      uint32_t x0 = uint32_t(dd), x1 = uint32_t(dd >> 32), x2 = uint32_t(dd >> 64),
	x3 = uint32_t(dd >> 96);

      uint32_t b  = x1 ^ x2 ^ x3 ^ rk0;
      uint32_t s = sm4_subword(b);
      uint32_t x4 = sm4_round(x0, s);

      b = x2 ^ x3 ^ x4 ^ rk1;
      s = sm4_subword(b);
      uint32_t x5 = sm4_round(x1, s);

      b = x3 ^ x4 ^ x5 ^ rk2;
      s = sm4_subword(b);
      uint32_t x6 = sm4_round(x2, s);

      b = x4 ^ x5 ^ x6 ^ rk3;
      s = sm4_subword(b);
      uint32_t x7 = sm4_round(x3, s);

      dd = Uint128{x4} | (Uint128{x5} << 32) | (Uint128{x6} << 64) | (Uint128{x7} << 96);
      if (not vecRegs_.write(vd, i, groupx8, dd))
	assert(0);
    }
      
  postVecSuccess();
}


static constexpr
uint32_t p1(uint32_t x)
{
  MyRol rol;
  return x ^ rol(x, 15) ^ rol(x, 23);
}


static constexpr
uint32_t
zvksh_w(uint32_t m16, uint32_t m9, uint32_t m3, uint32_t m13, uint32_t m6)
{
  MyRol rol;
  return p1(m16 ^ m9 ^ rol(m3, 15)) ^ rol(m13, 7) ^ m6;
}

  
template <typename URV>
void
Hart<URV>::execVsm3me_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 256, egs = 8;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  if (not isRvzvksh() or group*vecRegs_.bitsPerRegister() < egw or
	      sew != EW::Word or (elems % egs) or (start % egs) or
	      (vs1 + group > vd and vd + group > vs1) or
      (vs2 + group > vd and vd + group > vs2))
    {
      illegalInst(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint256 e1{0}, e2{0};
      if (not vecRegs_.read(vs1, i, groupx8, e1)) assert(0);
      if (not vecRegs_.read(vs2, i, groupx8, e2)) assert(0);

      uint32_t w0 = uint32_t{e2}, w1 = uint32_t{e2>>32}, w2 = uint32_t{e2>>32*2},
	w3 = uint32_t{e2>>32*3}, w4 = uint32_t{e2>>32*4}, w5 = uint32_t{e2>>32*5},
	w6 = uint32_t{e2>>32*6}, w7 = uint32_t{e2>>32*7};

      uint32_t w8 = uint32_t{e1}, w9 = uint32_t{e1>>32}, w10 = uint32_t{e1>>32*2},
	w11 = uint32_t{e1>>32*3}, w12 = uint32_t{e1>>32*4}, w13 = uint32_t{e1>>32*5},
	w14 = uint32_t{e1>>32*6}, w15 = uint32_t{e1>>32*7};

      // Byte Swap inputs from big-endian to little-endian
      w15 = util::byteswap(w15);
      w14 = util::byteswap(w14);
      w13 = util::byteswap(w13);
      w12 = util::byteswap(w12);
      w11 = util::byteswap(w11);
      w10 = util::byteswap(w10);
      w9  = util::byteswap(w9);
      w8  = util::byteswap(w8);
      w7  = util::byteswap(w7);
      w6  = util::byteswap(w6);
      w5  = util::byteswap(w5);
      w4  = util::byteswap(w4);
      w3  = util::byteswap(w3);
      w2  = util::byteswap(w2);
      w1  = util::byteswap(w1);
      w0  = util::byteswap(w0);

      // Note that some of the newly computed words are used in later invocations.
      uint32_t w16 = zvksh_w(w0 ,  w7 ,  w13 ,  w3  , w10 );
      uint32_t w17 = zvksh_w(w1 ,  w8 ,  w14 ,  w4  , w11 );
      uint32_t w18 = zvksh_w(w2 ,  w9 ,  w15 ,  w5  , w12 );
      uint32_t w19 = zvksh_w(w3 , w10 ,  w16 ,  w6  , w13 );
      uint32_t w20 = zvksh_w(w4 , w11 ,  w17 ,  w7  , w14 );
      uint32_t w21 = zvksh_w(w5 , w12 ,  w18 ,  w8  , w15 );
      uint32_t w22 = zvksh_w(w6 , w13 ,  w19 ,  w9  , w16 );
      uint32_t w23 = zvksh_w(w7 , w14 ,  w20 ,  w10 , w17 );

      // Byte swap outputs from little-endian back to big-endian
      w16 = util::byteswap(w16);
      w17 = util::byteswap(w17);
      w18 = util::byteswap(w18);
      w19 = util::byteswap(w19);
      w20 = util::byteswap(w20);
      w21 = util::byteswap(w21);
      w22 = util::byteswap(w22);
      w23 = util::byteswap(w23);

      using U256 = Uint256;
      U256 dd = ( U256{w16} | (U256{w17} << 32) | (U256{w18} << 32*2) |
		  (U256{w19} << 32*3) | (U256{w20} << 32*4) | (U256{w21} << 32*5) |
		  (U256{w22} << 32*6) | (U256{w23} << 32*7) );
	
      if (not vecRegs_.write(vd, i, groupx8, dd))
	assert(0);
    }
      
  postVecSuccess();
}


static constexpr
uint32_t
FF1(uint32_t x, uint32_t y, uint32_t z)
{
  return x ^ y ^ z;
}


static constexpr
uint32_t
FF2(uint32_t x, uint32_t y, uint32_t z)
{
  return  (x & y) | (x & z) | (y & z);
}


static constexpr
uint32_t
FF_j(uint32_t x, uint32_t y, uint32_t z, uint32_t j)
{
  return j <= 15 ? FF1(x, y, z) : FF2(x, y, z);
}


static constexpr
uint32_t
GG1(uint32_t x, uint32_t y, uint32_t z)
{
  return x ^ y ^ z;
}


static constexpr
uint32_t
GG2(uint32_t x, uint32_t y, uint32_t z)
{
  return (x & y) | (~x & z);
}


static constexpr
uint32_t
GG_j(uint32_t x, uint32_t y, uint32_t z, uint32_t j)
{
  return j <= 15 ? GG1(x, y, z) : GG2(x, y, z);
}


static constexpr
uint32_t
T_j(uint32_t j)
{
  return j <= 15 ? 0x79CC4519 : 0x7A879D8A;
}
    

static constexpr
uint32_t
P_0(uint32_t x)
{
  MyRol rol;
  return x ^ rol(x,  9) ^ rol(x, 17);
}


template <typename URV>
void
Hart<URV>::execVsm3c_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using EW = ElementWidth;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 256, egs = 8;
  unsigned elems = vecRegs_.elemCount();
  EW sew = vecRegs_.elemWidth();
  unsigned vd = di->op0(),  vs1 = di->op1(),  imm = di->op2();

  if (not isRvzvksh() or group*vecRegs_.bitsPerRegister() < egw or
	      sew != EW::Word or (elems % egs) or (start % egs) or
      (vs1 + group > vd and vd + group > vs1))
    {
      illegalInst(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs,  egStart = start / egs,  rnds = imm;
  MyRol rol;  // Rotate left

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint256 el1{0}, dd{0};
      if (not vecRegs_.read(vd, i, groupx8, dd)) assert(0);
      if (not vecRegs_.read(vs1, i, groupx8, el1)) assert(0);

      // load state
      uint32_t ai{uint32_t{dd}}, bi{uint32_t{dd >> 32}}, ci{uint32_t{dd >> 2*32}},
	di{uint32_t{dd >> 3*32}}, ei{uint32_t{dd >> 4*32}}, fi{uint32_t{dd >> 5*32}},
	gi{uint32_t{dd >> 6*32}}, hi{uint32_t{dd >> 7*32}};;

      //load message schedule
      uint32_t w0i{uint32_t{el1}}, w1i{uint32_t{el1 >> 32}}, /* u_w2{uint32_t{el1 >> 2*32}},
								u_w3{uint32_t{el1 >> 3*32}}, */
	w4i{uint32_t{el1 >> 4*32}}, w5i{uint32_t{el1 >> 5*32}}
	/* u_w6{uint32_t{el1 >> 6*32}}, u_w7{uint32_t{el1 >> 7*32}} */;

      // u_w inputs are unused
      // perform endian swap
      uint32_t h = util::byteswap(hi);
      uint32_t g = util::byteswap(gi);
      uint32_t f = util::byteswap(fi);
      uint32_t e = util::byteswap(ei);
      uint32_t d = util::byteswap(di);
      uint32_t c = util::byteswap(ci);
      uint32_t b = util::byteswap(bi);
      uint32_t a = util::byteswap(ai);
      uint32_t w5 = util::byteswap(w5i);
      uint32_t w4 = util::byteswap(w4i);
      uint32_t w1 = util::byteswap(w1i);
      uint32_t w0 = util::byteswap(w0i);

      uint32_t x0 = w0 ^ w4;
      uint32_t x1 = w1 ^ w5;
      uint32_t j = 2 * rnds;
      uint32_t ss1 = rol(rol(a, 12) + e + rol(T_j(j), j % 32), 7);
      uint32_t ss2 = ss1 ^ rol(a, 12);
      uint32_t tt1 = FF_j(a, b, c, j) + d + ss2 + x0;
      uint32_t tt2 = GG_j(e, f, g, j) + h + ss1 + w0;
      d = c;
      uint32_t c1 = rol(b, 9);
      b = a;
      uint32_t a1 = tt1;
      h = g;
      uint32_t g1 = rol(f, 19);
      f = e;
      uint32_t e1 = P_0(tt2);
      j = 2 * rnds + 1;
      ss1 = rol(rol(a1, 12) + e1 + rol(T_j(j), j % 32), 7);
      ss2 = ss1 ^ rol(a1, 12);
      tt1 = FF_j(a1, b, c1, j) + d + ss2 + x1;
      tt2 = GG_j(e1, f, g1, j) + h + ss1 + w1;
      d = c1;
      uint32_t c2 = rol(b, 9);
      b = a1;
      uint32_t a2 = tt1;
      h = g1;
      uint32_t g2 = rol(f, 19);
      f = e1;
      uint32_t e2 = P_0(tt2);

      // Swap back to big endian
      g1 = util::byteswap(g1);
      g2 = util::byteswap(g2);
      e1 = util::byteswap(e1);
      e2 = util::byteswap(e2);
      c1 = util::byteswap(c1);
      c2 = util::byteswap(c2);
      a1 = util::byteswap(a1);
      a2 = util::byteswap(a2);

      using U256 = Uint256;
      dd = ( U256{a2} | (U256{a1} << 32) | (U256{c2} << 32*2) | (U256{c1} << 32*3) |
	     (U256{e2} << 32*4) | (U256{e1} << 32*5) | (U256{g2} << 32*6) |
	     (U256{g1} << 32*7) );
  
      if (not vecRegs_.write(vd, i, groupx8, dd))
	assert(0);
    }
      
  postVecSuccess();
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;


