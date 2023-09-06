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
#include "crypto-util.hpp"
#include "functors.hpp"
#include "instforms.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"


using namespace WdRiscv;


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
      if (vecRegs_.isDestActive(vd, ix, group, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = bitReverse(e1);
	}
      vecRegs_.write(vd, ix, group, dest);
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
      if (vecRegs_.isDestActive(vd, ix, group, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = brev8(e1);
	}
      vecRegs_.write(vd, ix, group, dest);
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
      if (vecRegs_.isDestActive(vd, ix, group, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = util::byteswap(e1);
	}
      vecRegs_.write(vd, ix, group, dest);
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
      if (vecRegs_.isDestActive(vd, ix, group, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = std::countl_zero(e1);  // Count leading zeros.
	}
      vecRegs_.write(vd, ix, group, dest);
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
      if (vecRegs_.isDestActive(vd, ix, group, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = std::countr_zero(e1);  // Count trailing zeros.
	}
      vecRegs_.write(vd, ix, group, dest);
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
      if (vecRegs_.isDestActive(vd, ix, group, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = std::popcount(e1);
	}
      vecRegs_.write(vd, ix, group, dest);
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
      vop_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked, rol<uint8_t>);
      break;
    case EW::Half:
      vop_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked, rol<uint16_t>);
      break;
    case EW::Word:
      vop_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked, rol<uint32_t>);
      break;
    case EW::Word2:
      vop_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked, rol<uint64_t>);
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
      vop_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked, rol<uint8_t>);
      break;
    case EW::Half:
      vop_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked, rol<uint16_t>);
      break;
    case EW::Word:
      vop_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked, rol<uint32_t>);
      break;
    case EW::Word2:
      vop_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked, rol<uint64_t>);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


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
      vop_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked, ror<uint8_t>);
      break;
    case EW::Half:
      vop_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked, ror<uint16_t>);
      break;
    case EW::Word:
      vop_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked, ror<uint32_t>);
      break;
    case EW::Word2:
      vop_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked, ror<uint64_t>);
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
      vop_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked, ror<uint8_t>);
      break;
    case EW::Half:
      vop_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked, ror<uint16_t>);
      break;
    case EW::Word:
      vop_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked, ror<uint32_t>);
      break;
    case EW::Word2:
      vop_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked, ror<uint64_t>);
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
      vop_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked, ror<uint8_t>);
      break;
    case EW::Half:
      vop_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked, ror<uint16_t>);
      break;
    case EW::Word:
      vop_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked, ror<uint32_t>);
      break;
    case EW::Word2:
      vop_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked, ror<uint64_t>);
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
      if (vecRegs_.isDestActive(vd, ix, group, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = mySll(DWT(e1), DWT(e2));
	}
      vecRegs_.write(vd, ix, wideGroup, dest);
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
      if (vecRegs_.isDestActive(vd, ix, group, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = mySll(DWT(e1), DWT(e2));
	}
      vecRegs_.write(vd, ix, wideGroup, dest);
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
      vecRegs_.read(vd, i, groupx8, y);
      vecRegs_.read(vs2, i, groupx8, x);
      vecRegs_.read(vs1, i, groupx8, h);
      h         = brev8(h);
      Uint128 s = brev8(y ^ x);

      for (unsigned bit = 0; bit < 128; bit++)
        {
          if ((s >> static_cast<int>(bit)) & 1U)
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
      vecRegs_.read(vd, i, groupx8, y);
      vecRegs_.read(vs1, i, groupx8, h);
      y = brev8(y);
      h = brev8(h);

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
      vecRegs_.read(vd, i, groupx8, state);
      vecRegs_.read(vs1, i, groupx8, rkey);

      Uint128 sr = aes_shift_rows_inv(state);
      Uint128 sb = aes_subbytes_inv(sr);
      Uint128 ark = sb ^ rkey;
      vecRegs_.write(vd, i, groupx8, ark);
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
  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs) or vd == vs1)
    {
      illegalInst(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 state{0}, rkey{0};
      vecRegs_.read(vd, i, groupx8, state);
      vecRegs_.read(vs1, 0, groupx8, rkey);

      Uint128 sr = aes_shift_rows_inv(state);
      Uint128 sb = aes_subbytes_inv(sr);
      Uint128 ark = sb ^ rkey;
      vecRegs_.write(vd, i, groupx8, ark);
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
      vecRegs_.read(vd, i, groupx8, state);
      vecRegs_.read(vs1, i, groupx8, rkey);

      Uint128 sb = aes_subbytes_fwd(state);
      Uint128 sr = aes_shift_rows_fwd(sb);
      Uint128 ark = sr ^ rkey;
      vecRegs_.write(vd, i, groupx8, ark);
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
  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs) or vd == vs1)
    {
      illegalInst(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 state{0}, rkey{0};
      vecRegs_.read(vd, i, groupx8, state);
      vecRegs_.read(vs1, 0, groupx8, rkey);

      Uint128 sb = aes_subbytes_fwd(state);
      Uint128 sr = aes_shift_rows_fwd(sb);
      Uint128 ark = sr ^ rkey;
      vecRegs_.write(vd, i, groupx8, ark);
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
      vecRegs_.read(vd, i, groupx8, state);
      vecRegs_.read(vs1, i, groupx8, rkey);

      Uint128 sb = aes_subbytes_fwd(state);
      Uint128 sr = aes_shift_rows_fwd(sb);
      Uint128 mix = aes_mixcolumns_fwd(sr);
      Uint128 ark = mix ^ rkey;
      vecRegs_.write(vd, i, groupx8, ark);
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
  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs) or vd == vs1)
    {
      illegalInst(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 state{0}, rkey{0};
      vecRegs_.read(vd, i, groupx8, state);
      vecRegs_.read(vs1, 0, groupx8, rkey);

      Uint128 sb = aes_subbytes_fwd(state);
      Uint128 sr = aes_shift_rows_fwd(sb);
      Uint128 mix = aes_mixcolumns_fwd(sr);
      Uint128 ark = mix ^ rkey;
      vecRegs_.write(vd, i, groupx8, ark);
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
      vecRegs_.read(vd, i, groupx8, state);
      vecRegs_.read(vs1, i, groupx8, rkey);

      Uint128 sr = aes_shift_rows_inv(state);
      Uint128 sb = aes_subbytes_inv(sr);
      Uint128 ark = sb ^ rkey;
      Uint128 mix = aes_mixcolumns_inv(ark);
      vecRegs_.write(vd, i, groupx8, mix);
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
  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs) or vd == vs1)
    {
      illegalInst(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 state{0}, rkey{0};
      vecRegs_.read(vd, i, groupx8, state);
      vecRegs_.read(vs1, 0, groupx8, rkey);

      Uint128 sr = aes_shift_rows_inv(state);
      Uint128 sb = aes_subbytes_inv(sr);
      Uint128 ark = sb ^ rkey;
      Uint128 mix = aes_mixcolumns_inv(ark);
      vecRegs_.write(vd, i, groupx8, mix);
    }

  postVecSuccess();
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
      vecRegs_.read(vs1, i, groupx8, e1);
      auto [crk0, crk1, crk2, crk3] = toQuarters(e1);
      uint32_t w0 = aes_subword_fwd(aes_rotword(crk3)) ^ aes_decode_rcon(r) ^ crk0;
      uint32_t w1 = w0 ^ crk1;
      uint32_t w2 = w1 ^ crk2;
      uint32_t w3 = w2 ^ crk3;

      Uint128 res = fromQuarters(w0, w1, w2, w3);
      vecRegs_.write(vd, i, groupx8, res);
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
  if (round > 14 or round < 2)
    round ^= 0x8; // Flip bit 3.

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 e1{0}, d{0};
      vecRegs_.read(vs1, i, groupx8, e1);
      uint32_t crk3 = toQuarters(e1)[3];

      vecRegs_.read(vd, i, groupx8, d);
      auto [rkb0, rkb1, rkb2, rkb3] = toQuarters(d);

      uint32_t w0 = (round & 1) ? aes_subword_fwd(crk3) ^ rkb0 :
	aes_subword_fwd(aes_rotword(crk3)) ^ aes_decode_rcon((round >> 1) - 1) ^ rkb0;
      uint32_t w1 = w0 ^ rkb1;
      uint32_t w2 = w1 ^ rkb2;
      uint32_t w3 = w2 ^ rkb3;

      Uint128 res = fromQuarters(w0, w1, w2, w3);
      vecRegs_.write(vd, i, groupx8, res);
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
  unsigned vd = di->op0(),  vs1 = di->op1();

  if (not isRvzvkned() or group*vecRegs_.bitsPerRegister() < egw or
      sew != EW::Word or (elems % egs) or (start % egs) or vd == vs1)
    {
      illegalInst(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned egLen = elems / egs, egStart = start / egs;

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint128 state{0}, rkey{0};
      vecRegs_.read(vd, i, groupx8, state);
      vecRegs_.read(vs1, 0, groupx8, rkey);

      Uint128 ark = state ^ rkey;
      vecRegs_.write(vd, i, groupx8, ark);
    }

  postVecSuccess();
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
	  vecRegs_.read(vd, i, groupx8, dd);
	  vecRegs_.read(vs1, i, groupx8, e1);
	  vecRegs_.read(vs2, i, groupx8, e2);

	  vsha2ms<uint32_t, Uint128>(dd, e1, e2);
	  vecRegs_.write(vd, i, groupx8, dd);
	}
    }
  else
    {
      for (unsigned i = egStart; i < egLen; ++i)
	{
	  Uint256 dd{0}, e1{0}, e2{0};
	  vecRegs_.read(vd, i, groupx8, dd);
	  vecRegs_.read(vs1, i, groupx8, e1);
	  vecRegs_.read(vs2, i, groupx8, e2);

	  vsha2ms<uint64_t, Uint256>(dd, e1, e2);
	  vecRegs_.write(vd, i, groupx8, dd);
	}
    }

  postVecSuccess();
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
	  vecRegs_.read(vd, i, groupx8, dd);
	  vecRegs_.read(vs1, i, groupx8, e1);
	  vecRegs_.read(vs2, i, groupx8, e2);

	  vsha2c<uint32_t, Uint128>(dd, e1, e2, true);
	  vecRegs_.write(vd, i, groupx8, dd);
	}
      break;
    case EW::Word2:
      for (unsigned i = egStart; i < egLen; ++i)
	{
	  Uint256 e1{0}, e2{0}, dd{0};
	  vecRegs_.read(vd, i, groupx8, dd);
	  vecRegs_.read(vs1, i, groupx8, e1);
	  vecRegs_.read(vs2, i, groupx8, e2);

	  vsha2c<uint64_t, Uint256>(dd, e1, e2, true);
	  vecRegs_.write(vd, i, groupx8, dd);
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
	  vecRegs_.read(vd, i, groupx8, dd);
	  vecRegs_.read(vs1, i, groupx8, e1);
	  vecRegs_.read(vs2, i, groupx8, e2);

	  vsha2c<uint32_t, Uint128>(dd, e1, e2, false);
	  vecRegs_.write(vd, i, groupx8, dd);
	}
      break;
    case EW::Word2:
      for (unsigned i = egStart; i < egLen; ++i)
	{
	  Uint256 e1{0}, e2{0}, dd{0};
	  vecRegs_.read(vd, i, groupx8, dd);
	  vecRegs_.read(vs1, i, groupx8, e1);
	  vecRegs_.read(vs2, i, groupx8, e2);

	  vsha2c<uint64_t, Uint256>(dd, e1, e2, false);
	  vecRegs_.write(vd, i, groupx8, dd);
	}
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
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
      vecRegs_.read(vs1, i, groupx8, e1);
      auto [rk0, rk1, rk2, rk3] = toQuarters(e1);

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

      Uint128 dd = fromQuarters(rk4, rk5, rk6, rk7);
      vecRegs_.write(vd, i, groupx8, dd);
    }

  postVecSuccess();
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
      vecRegs_.read(vs1, i, groupx8, e1);
      vecRegs_.read(vd, i, groupx8, dd);

      auto [rk0, rk1, rk2, rk3] = toQuarters(e1);
      auto [x0,  x1,  x2,  x3]  = toQuarters(dd);

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

      dd = fromQuarters(x4, x5, x6, x7);
      vecRegs_.write(vd, i, groupx8, dd);
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
      vecRegs_.read(vs1, 0, groupx8, e1);
      vecRegs_.read(vd, i, groupx8, dd);

      auto [rk0, rk1, rk2, rk3] = toQuarters(e1);
      auto [x0,  x1,  x2,  x3]  = toQuarters(dd);

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

      dd = fromQuarters(x4, x5, x6, x7);
      vecRegs_.write(vd, i, groupx8, dd);
    }

  postVecSuccess();
}


constexpr
std::array<uint32_t, 8> toEighths(Uint256 v)
{
  auto [h0, h1]         = toHalves(v);
  auto [e0, e1, e2, e3] = toQuarters(h0);
  auto [e4, e5, e6, e7] = toQuarters(h1);
  return { e0, e1, e2, e3, e4, e5, e6, e7 };
}

constexpr
Uint256 fromEighths(uint32_t e0, uint32_t e1, uint32_t e2, uint32_t e3,
                    uint32_t e4, uint32_t e5, uint32_t e6, uint32_t e7)
{
  auto h0 = fromQuarters(e0, e1, e2, e3);
  auto h1 = fromQuarters(e4, e5, e6, e7);
  return fromHalves(h0, h1);
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
      vecRegs_.read(vs1, i, groupx8, e1);
      vecRegs_.read(vs2, i, groupx8, e2);

      auto [w0, w1, w2,  w3,  w4,  w5,  w6,  w7]  = toEighths(e2);
      auto [w8, w9, w10, w11, w12, w13, w14, w15] = toEighths(e1);

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

      Uint256 dd = fromEighths(w16, w17, w18, w19, w20, w21, w22, w23);

      vecRegs_.write(vd, i, groupx8, dd);
    }

  postVecSuccess();
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

  for (unsigned i = egStart; i < egLen; ++i)
    {
      Uint256 el1{0}, dd{0};
      vecRegs_.read(vd, i, groupx8, dd);
      vecRegs_.read(vs1, i, groupx8, el1);

      // load state
      auto [ai, bi, ci, di, ei, fi, gi, hi]  = toEighths(dd);

      //load message schedule
      auto [w0i, w1i, w2i, w3i, w4i, w5i, w6i, w7i] = toEighths(el1);

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

      dd = fromEighths(a2, a1, c2, c1, e2, e1, g2, g1);

      vecRegs_.write(vd, i, groupx8, dd);
    }

  postVecSuccess();
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;


