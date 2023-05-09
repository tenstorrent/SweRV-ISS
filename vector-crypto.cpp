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
#include <boost/multiprecision/cpp_int.hpp>
#include "float-convert-helpers.hpp"
#include "wideint.hpp"
#include "instforms.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"
#include "softfloat-util.hpp"


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

  typedef ElementWidth EW;
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

  typedef ElementWidth EW;
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
T
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

  typedef ElementWidth EW;
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
T brev8(const T& x)
{
  T res{0};
  for (unsigned byteIx = 0; byteIx < sizeof(x); ++byteIx)
    {
      uint8_t byte = (x >> 8*byteIx);
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

  typedef ElementWidth EW;
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

  typedef ElementWidth EW;
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
          dest = util::countLeadingZeros(e1);
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

  typedef ElementWidth EW;
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
          dest = util::countTrailingZeros(e1);
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

  typedef ElementWidth EW;
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
          dest = util::countOnes(e1);
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

  typedef ElementWidth EW;
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
/// n bits of b are used, n is log2(width of T). For example, if T is
/// uint32_t then only least sig 5 bits of b are used.
struct
MyRol
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  {
    unsigned width = sizeof(T)*8;  // Bit count of T
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

  typedef ElementWidth EW;
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

  typedef ElementWidth EW;
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
/// Only the least sig n bits of b are used, n is log2(width of T).
struct
MyRor
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  {
    unsigned width = sizeof(T)*8;  // Bit count of T
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

  typedef ElementWidth EW;
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

  typedef ElementWidth EW;
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

  typedef ElementWidth EW;
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
  typedef typename makeDoubleWide<ELEM_TYPE>::type DWT; // Double wide type
  unsigned errors = 0, wideGroup = group*2;

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

  typedef ElementWidth EW;
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
  typedef typename makeDoubleWide<ELEM_TYPE>::type DWT; // Double wide type
  unsigned errors = 0, wideGroup = group*2;

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

  typedef ElementWidth EW;
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

  typedef ElementWidth EW;
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

  typedef ElementWidth EW;
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

  typedef ElementWidth EW;
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

  typedef ElementWidth EW;
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

  typedef ElementWidth EW;
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

  typedef ElementWidth EW;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkg() or group*vecRegs_.bitsPerRegister() < egw or sew != EW::Word)
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount();

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
        if ((s >> i) & 1)
	  z ^= h;

	bool reduce = (h >> 127) & 1;
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

  typedef ElementWidth EW;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkg() or group*vecRegs_.bitsPerRegister() < egw or sew != EW::Word)
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemCount();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 x{0}, y{0}, h{0}, z{0};
      if (not vecRegs_.read(vd, i, groupx8, y))
	assert(0);
      if (not vecRegs_.read(vs1, i, groupx8, h))
	assert(0);
      for (unsigned bit = 0; bit < 128; bit++)
	{
	  if ((y >> bit) & 1)
	    z ^= h;
	  bool reduce = (h >> 127) & 1;
	  h <<= 1;
	  if (reduce)
	    h ^= 0x87;
	}
      Uint128 res = brev8(z);
      vecRegs_.write(vd, i, groupx8, res);
    }

  postVecSuccess();
}


extern __uint128_t
aes_shift_rows_inv(__uint128_t x);

extern __uint128_t
aes_subbytes_inv(__uint128_t x);


template <typename URV>
void
Hart<URV>::execVaesdf_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  typedef ElementWidth EW;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or sew != EW::Word)
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemCount();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      __uint128_t state{0}, rkey{0};
      if (not vecRegs_.read(vd, i, groupx8, state))
	assert(0);
      if (not vecRegs_.read(vs1, i, groupx8, rkey))
	assert(0);
      __uint128_t sr = aes_shift_rows_inv(state);
      __uint128_t sb = aes_subbytes_inv(sr);
      __uint128_t ark = sb ^ rkey;
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

  typedef ElementWidth EW;

  unsigned groupx8 = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned group = groupx8 > 8 ? groupx8/8 : 1;
  unsigned egw = 128, egs = 4;
  EW sew = vecRegs_.elemWidth();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or sew != EW::Word)
    {
      illegalInst(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemCount();

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      __uint128_t state{0}, rkey{0};
      if (not vecRegs_.read(vd, i, groupx8, state))
	assert(0);
      if (not vecRegs_.read(vs1, 0, groupx8, rkey))
	assert(0);
      __uint128_t sr = aes_shift_rows_inv(state);
      __uint128_t sb = aes_subbytes_inv(sr);
      __uint128_t ark = sb ^ rkey;
      if (not vecRegs_.write(vd, i, groupx8, ark))
	assert(0);
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVaesef_vv(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVaesef_vs(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVaesem_vv(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVaesem_vs(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVaesdm_vv(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVaesdm_vs(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVaeskf1_vi(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVaeskf2_vi(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVaesz_vs(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsha2ms_vv(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsha2ch_vv(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsha2cl_vv(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsm4k_vi(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsm4r_vv(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsm4r_vs(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsm3me_vv(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsm3c_vi(const DecodedInst* di)
{
  postVecFail(di);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;


