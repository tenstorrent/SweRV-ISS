// Copyright 2024 Tenstorrent Corporation or its affiliates.
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

#include <climits>
#include <cassert>
#include "DecodedInst.hpp"
#include "Hart.hpp"

using namespace WdRiscv;


template <typename URV>
void
Hart<URV>::execVqdot_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  if (not extensionIsEnabled(RvExtension::Zvqdot))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, group))
    return;

  if (sew != ElementWidth::Word)
    {
      postVecFail(di);
      return;
    }

  if (start >= elems)
    return;

  unsigned destGroup = std::max(vecRegs_.groupMultiplierX8(GroupMultiplier::One), group);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      int32_t dest = 0;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  int32_t e1 = 0, e2 = 0;
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  for (unsigned i = 0; i < sizeof(e1); ++i)
	    {
	      int32_t b1 = int8_t(e1 >> i*8);  // Ith byte of e1.
	      int32_t b2 = int8_t(e2 >> i*8);  // Ith byte of e2.
	      dest += b1 * b2;
	    }
	}

      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVqdot_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  if (sew != ElementWidth::Word)
    {
      postVecFail(di);
      return;
    }

  unsigned destGroup = std::max(vecRegs_.groupMultiplierX8(GroupMultiplier::One), group);

  int32_t e2 = intRegs_.read(rs);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      int32_t dest = 0;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  int32_t e1 = 0;
	  vecRegs_.read(vs1, ix, group, e1);

	  for (unsigned i = 0; i < sizeof(e1); ++i)
	    {
	      int32_t b1 = int8_t(e1 >> i*8);  // Ith byte of e1.
	      int32_t b2 = int8_t(e2 >> i*8);  // Ith byte of e2.
	      dest += b1 * b2;
	    }
	}

      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVqdotu_vv(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, group))
    return;

  if (sew != ElementWidth::Word)
    {
      postVecFail(di);
      return;
    }

  if (start >= elems)
    return;

  unsigned destGroup = std::max(vecRegs_.groupMultiplierX8(GroupMultiplier::One), group);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      uint32_t dest = 0;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  uint32_t e1 = 0, e2 = 0;
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  for (unsigned i = 0; i < sizeof(e1); ++i)
	    {
	      uint32_t b1 = uint8_t(e1 >> i*8);  // Ith byte of e1.
	      uint32_t b2 = uint8_t(e2 >> i*8);  // Ith byte of e2.
	      dest += b1 * b2;
	    }
	}

      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVqdotu_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  if (sew != ElementWidth::Word)
    {
      postVecFail(di);
      return;
    }

  unsigned destGroup = std::max(vecRegs_.groupMultiplierX8(GroupMultiplier::One), group);

  uint32_t e2 = intRegs_.read(rs);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      uint32_t dest = 0;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  uint32_t e1 = 0;
	  vecRegs_.read(vs1, ix, group, e1);

	  for (unsigned i = 0; i < sizeof(e1); ++i)
	    {
	      uint32_t b1 = uint8_t(e1 >> i*8);  // Ith byte of e1.
	      uint32_t b2 = uint8_t(e2 >> i*8);  // Ith byte of e2.
	      dest += b1 * b2;
	    }
	}

      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVqdotsu_vv(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, group))
    return;

  if (sew != ElementWidth::Word)
    {
      postVecFail(di);
      return;
    }

  if (start >= elems)
    return;

  unsigned destGroup = std::max(vecRegs_.groupMultiplierX8(GroupMultiplier::One), group);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      int32_t dest = 0;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  int32_t e1 = 0;
	  uint32_t e2 = 0;
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  for (unsigned i = 0; i < sizeof(e1); ++i)
	    {
	      int32_t b1 = int8_t(e1 >> i*8);  // Ith byte of e1.
	      uint32_t b2 = uint8_t(e2 >> i*8);  // Ith byte of e2.
	      dest += b1 * b2;
	    }
	}

      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVqdotsu_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  if (sew != ElementWidth::Word)
    {
      postVecFail(di);
      return;
    }

  unsigned destGroup = std::max(vecRegs_.groupMultiplierX8(GroupMultiplier::One), group);

  uint32_t e2 = intRegs_.read(rs);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      int32_t dest = 0;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  int32_t e1 = 0;
	  vecRegs_.read(vs1, ix, group, e1);

	  for (unsigned i = 0; i < sizeof(e1); ++i)
	    {
	      int32_t b1 = int8_t(e1 >> i*8);  // Ith byte of e1.
	      uint32_t b2 = uint8_t(e2 >> i*8);  // Ith byte of e2.
	      dest += b1 * b2;
	    }
	}

      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVqdotus_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  if (sew != ElementWidth::Word)
    {
      postVecFail(di);
      return;
    }

  unsigned destGroup = std::max(vecRegs_.groupMultiplierX8(GroupMultiplier::One), group);

  int32_t e2 = intRegs_.read(rs);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      int32_t dest = 0;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  uint32_t e1 = 0;
	  vecRegs_.read(vs1, ix, group, e1);

	  for (unsigned i = 0; i < sizeof(e1); ++i)
	    {
	      uint32_t b1 = uint8_t(e1 >> i*8);  // Ith byte of e1.
	      int32_t b2 = int8_t(e2 >> i*8);  // Ith byte of e2.
	      dest += b1 * b2;
	    }
	}

      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
