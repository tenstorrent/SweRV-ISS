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

#include <iostream>
#include <cfenv>
#include <cmath>
#include <climits>
#include <cassert>
#include <optional>
#include <boost/multiprecision/cpp_int.hpp>
#include "wideint.hpp"
#include "instforms.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"
#include "float-util.hpp"


// make_unsigned/make_signed do not work on our types -- compensate.
namespace std
{
  template <>
  struct
  make_unsigned<WdRiscv::Int128>
  {
    typedef WdRiscv::Uint128 type;
  };

  template <>
  struct
  make_unsigned<WdRiscv::Int256>
  {
    typedef WdRiscv::Uint256 type;
  };

  template <>
  struct
  make_unsigned<WdRiscv::Int512>
  {
    typedef WdRiscv::Uint512 type;
  };

  template <>
  struct
  make_unsigned<WdRiscv::Int1024>
  {
    typedef WdRiscv::Uint1024 type;
  };

  template <>
  struct
  make_unsigned<WdRiscv::Uint128>
  {
    typedef WdRiscv::Uint128 type;
  };

  template <>
  struct
  make_unsigned<WdRiscv::Uint256>
  {
    typedef WdRiscv::Uint256 type;
  };

  template <>
  struct
  make_unsigned<WdRiscv::Uint512>
  {
    typedef WdRiscv::Uint512 type;
  };

  template <>
  struct
  make_unsigned<WdRiscv::Uint1024>
  {
    typedef WdRiscv::Uint1024 type;
  };

  template <>
  struct
  make_signed<WdRiscv::Uint128>
  {
    typedef WdRiscv::Int128 type;
  };

  template <>
  struct
  make_signed<WdRiscv::Uint256>
  {
    typedef WdRiscv::Int256 type;
  };

  template <>
  struct
  make_signed<WdRiscv::Uint512>
  {
    typedef WdRiscv::Int512 type;
  };

  template <>
  struct
  make_signed<WdRiscv::Uint1024>
  {
    typedef WdRiscv::Int1024 type;
  };

  template <>
  struct
  make_signed<WdRiscv::Int128>
  {
    typedef WdRiscv::Int128 type;
  };

  template <>
  struct
  make_signed<WdRiscv::Int256>
  {
    typedef WdRiscv::Int256 type;
  };

  template <>
  struct
  make_signed<WdRiscv::Int512>
  {
    typedef WdRiscv::Int512 type;
  };

  template <>
  struct
  make_signed<WdRiscv::Int1024>
  {
    typedef WdRiscv::Int1024 type;
  };
}


namespace WdRiscv
{
  template <> struct makeDoubleWide<Float16>    { typedef float   type; };
  template <> struct makeDoubleWide<BFloat16>   { typedef float   type; };
  template <> struct makeDoubleWide<float>      { typedef double  type; };

  /// Set result to the upper half of a*b computed in double width
  /// intermediate.
  template <typename T>
  void mulh(const T& a, const T& b, T& result)
  {
    typedef typename makeDoubleWide<T>::type T2; // Double wide type

    unsigned tbits = integerWidth<T> (); // Number of bits in T

    T2 temp = a;
    temp *= b;
    temp >>= tbits;
    result = T(temp);
  }


  /// Specialized mulh for 1024-bit unsigned operands.
  template <>
  void mulh(const Uint1024& a, const Uint1024& b, Uint1024& result)
  {
    // Unpack b into 512-bit pieces
    Uint512 a0 = Uint512(a), a1 = Uint512(a >> 512);
    Uint512 b0 = Uint512(b), b1 = Uint512(b >> 512);

    // Multiply the 4 pieces accumulating results. Maintain upper 1024
    // bits of resuts.
    Uint1024 temp = a0;
    temp *= b0;
    result = temp;

    result >>= 512;
    temp = a1;
    temp *= b0;
    result += temp;

    result >>= 512;
    temp = a0;
    temp *= b1;
    result += temp;
    
    result >>= 512;
    temp = a1;
    temp *= b1;
    result += temp;
  }


  /// Specialized mulh for 1024-bit signed operands.
  template <>
  void mulh(const Int1024& a, const Int1024& b, Int1024& result)
  {
    // Unpack b into 512-bit pieces
    Int512 a0 = Int512(a), a1 = Int512(a >> 512);
    Int512 b0 = Int512(b), b1 = Int512(b >> 512);

    // Multiply the 4 pieces accumulating results. Maintain upper 1024
    // bits of resuts.
    Int1024 temp = a0;
    temp *= b0;
    result = temp;

    result >>= 512;
    temp = a1;
    temp *= b0;
    result += temp;

    result >>= 512;
    temp = a0;
    temp *= b1;
    result += temp;
    
    result >>= 512;
    temp = a1;
    temp *= b1;
    result += temp;
  }


  /// Set result to the upper half of a*b computed in double width
  /// intermediate. TS is a signed integer type (e.g. int8_t).
  /// TU is the corresponding unsigned integer type (e.g. uint8_t).
  template <typename TS, typename TU>
  void mulhsu(const TS& a, const TU& b, TS& result)
  {
    typedef typename makeDoubleWide<TS>::type TS2; // Double wide signed type

    unsigned bits = integerWidth<TS> (); // Number of bits in TS and TU

    TS2 temp = a;
    temp *= b;
    temp >>= bits;
    result = TS(temp);
  }


  /// Specialized mulhsu for 1024-bit unsigned operands.
  template <>
  void mulhsu(const Int1024& a, const Uint1024& b, Int1024& result)
  {
    if (a > Int1024(0))
      {
        Uint1024 ua(a);
        Uint1024 temp = 0;
        mulh(ua, b, temp);
        result = temp;
        return;
      }

    Int1024 smallest = Int1024(1) << 1023;  // Smallest 1024-bit integer.
    if (a == smallest)
      {
        // Result consists of bits 1024 to 2047 of -(power(2,1023)*b)
        // computed in unlimited precision.
        Uint1024 temp = b;
        temp >>= 1;
        result = temp;
        result = -result;
        return;
      }

    Uint1024 nega(-a), temp(0);
    mulh(nega, b, temp);
    result = temp;
    result = -result;
  }

  /// Set result to the product of a and b where a is signed and b
  /// is unsigned and where a and b have the same width.
  template <typename TS, typename TU>
  void mulsu(const TS& a, const TU& b, TS& result)
  {
    bool neg = a < 0;
    TU aa = neg? TU(-a) : TU(a);
    aa *= b;
    result = TS(aa);
    if (neg)
      result = - result;
  }
}


using namespace WdRiscv;


// Floating-point widening operations may raise floating-point exceptions.
// In the case of a scalar operand, the widening operation should not
// raise any exceptions when all vector elements are masked off.  As a
// result, this class exists to perform the widening instruction just
// prior to first actual use (i.e. within the vector instruction's
// operation) and cache the result so that successive uses don't have to
// re-perform the widening.
template <typename ELEM_TYPE>
class WidenedFpScalar
{
  using ELEM_TYPE2X = typename makeDoubleWide<ELEM_TYPE>::type;

  public:
    constexpr WidenedFpScalar(ELEM_TYPE value): v_{value} {};

    constexpr operator ELEM_TYPE2X()
    {
      if (not vDw_.has_value())
        vDw_ = fpConvertTo<ELEM_TYPE2X, true>(v_);
      return *vDw_;
    }

  private:
    ELEM_TYPE v_;
    std::optional<ELEM_TYPE2X> vDw_;
};


template <typename URV>
void
Hart<URV>::enableVectorExtension(bool flag)
{
  enableExtension(RvExtension::V, flag);
  csRegs_.enableVectorExtension(flag);

  if (not flag)
    setVecStatus(VecStatus::Off);
}


template <typename URV>
inline
void
Hart<URV>::setVecStatus(FpStatus value)
{
  if (mstatus_.bits_.VS != unsigned(value))
    {
      mstatus_.bits_.VS = unsigned(value);
      writeMstatus();
    }

  if (virtMode_ and vsstatus_.bits_.VS != unsigned(value))
    {
      vsstatus_.bits_.VS = unsigned(value);
      pokeCsr(CsrNumber::VSSTATUS, vsstatus_.value_);
      recordCsrWrite(CsrNumber::VSSTATUS);
      updateCachedVsstatus();
    }
}


template <typename URV>
bool
Hart<URV>::checkVecIntInst(const DecodedInst* di)
{
  return checkVecIntInst(di, vecRegs_.groupMultiplier(), vecRegs_.elemWidth());
}


template <typename URV>
bool
Hart<URV>::checkVecIntInst(const DecodedInst* di, GroupMultiplier /*gm*/,
			     ElementWidth /*eew*/)
{
  if (not checkSewLmulVstart(di))
    return false;

  if (di->isMasked() and di->op0() == 0)  // Dest register cannot overlap mask register v0
    {
      postVecFail(di);
      return false;
    }

#if 0
  // Use of vstart values greater than vlmax is reserved (section 3.7 of spec).
  if (vstart > vecRegs_.vlmax(gm, eew))
    {
      postVecFail(di);
      return false;
    }
#endif

  return true;
}


template <typename URV>
bool
Hart<URV>::checkVecFpInst(const DecodedInst* di, bool wide,
                               bool (Hart::*fp16LegalFn)() const)
{
  if (not checkVecIntInst(di))
    return false;

  return checkFpSewLmulVstart(di, wide, fp16LegalFn);
}



template <typename URV>
bool
Hart<URV>::checkFpSewLmulVstart(const DecodedInst* di, bool wide,
				 bool (Hart::*fp16LegalFn)() const)
{
  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = false;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:   ok = (this->*fp16LegalFn)(); break;
    case EW::Word:   ok = isFpLegal();            break;
    case EW::Word2:  ok = isDpLegal();            break;
    default:         ok = false;                  break;
    }

  if (ok and wide)
    {
      switch (sew)
	{
	case EW::Half:   ok = isFpLegal(); break;
	case EW::Word:   ok = isDpLegal();  break;
	default:         ok = false;        break;
	}
    }

  // Clear soft-float library or x86 exception flags
  clearSimulatorFpFlags();

  // Set soft-float library or x86 rounding mode
  setSimulatorRoundingMode(getFpRoundingMode());

  if (not ok)
    postVecFail(di);

  return ok;
}


template <typename URV>
bool
Hart<URV>::checkSewLmulVstart(const DecodedInst* di)
{
  // vector extension must be enabled, mstatus.fs must not be off, sew/lmul must
  // be legal, vtype.vill must not be set.
  if (not isVecLegal() or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  // Trap on use of non-zero vstart for arithmetic vector ops.
  URV vstart = csRegs_.peekVstart();
  if (trapNonZeroVstart_ and vstart > 0)
    {
      postVecFail(di);
      return false;
    }

  return true;
}  


template <typename URV>
inline
bool
Hart<URV>::checkVecOpsVsEmul(const DecodedInst* di, unsigned op0,
			     unsigned op1, unsigned op2, unsigned groupX8)
{
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask = eg - 1;   // Assumes eg is 1, 2, 4, or 8
  unsigned op = op0 | op1 | op2;
  if ((op & mask) == 0)
    {
      vecRegs_.setOpEmul(eg, eg, eg);  // Track operand group for logging
      return true;
    }

  postVecFail(di);
  return false;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecOpsVsEmul(const DecodedInst* di, unsigned op0,
			     unsigned op1, unsigned groupX8)
{
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask = eg - 1;   // Assumes eg is 1, 2, 4, or 8
  unsigned op = op0 | op1;
  if ((op & mask) == 0)
    {
      vecRegs_.setOpEmul(eg, eg);  // Track operand group for logging
      return true;
    }
  postVecFail(di);
  return false;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecOpsVsEmul(const DecodedInst* di, unsigned op0, unsigned groupX8)
{
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask = eg - 1;   // Assumes eg is 1, 2, 4, or 8
  if ((op0 & mask) == 0)
    {
      vecRegs_.setOpEmul(eg);  // Track operand group for logging
      return true;
    }
  postVecFail(di);
  return false;
}


template <typename URV>
inline
bool
Hart<URV>::checkRedOpVsEmul(const DecodedInst* di, unsigned op1,
			    unsigned groupX8, unsigned vstart)
{
  // Reduction ops must have zero vstart.
  if (vstart > 0)
    {
      postVecFail(di);
      return false;
    }

  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask = eg - 1;   // Assumes eg is 1, 2, 4, or 8

  if ((op1 & mask) == 0)
    {
      vecRegs_.setOpEmul(1, eg, 1);  // Track operand group for logging (1 for mask).
      return true;
    }

  // Vector operand not a multiple of emul or non-zero vstart: illegal.
  postVecFail(di);
  return false;
}


/// Return true if destination/source overlap is allowed.
static
bool
checkDestSourceOverlap(unsigned dest, unsigned destWidth, unsigned destGroupX8,
		       unsigned src, unsigned srcWidth, unsigned srcGroupX8)
{
  if (srcWidth == destWidth)
    return true;   // Source eew == dest eew

  unsigned srcGroup = srcGroupX8 >= 8 ? srcGroupX8/8 : 1;
  unsigned destGroup = destGroupX8 >= 8 ? destGroupX8/8 : 1;

  if (src >= dest + destGroup or dest >= src + srcGroup)
    return true;  // No overlap.

  // Destination eew > source eew: Overlap ok if source group is >=
  // 1 and overlap is at last # of <srcGroup> registers in dest.
  if (destWidth > srcWidth)
    return srcGroupX8 >= 8 and src == dest + destGroup - srcGroup;

  // Destination eew < source eew: Overlap ok if overlap is at
  // first register in source.
  return src == dest;
}


/// Return true if destination and source groups do overlap.
static
bool
hasDestSourceOverlap(unsigned dest, unsigned destGroupX8, unsigned src,
		     unsigned srcGroupX8)
{
  unsigned srcGroup = srcGroupX8 >= 8 ? srcGroupX8/8 : 1;
  unsigned destGroup = destGroupX8 >= 8 ? destGroupX8/8 : 1;

  if (src >= dest + destGroup or dest >= src + srcGroup)
    return false;  // No overlap.
  return true;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecMaskInst(const DecodedInst* di, unsigned dest,
				 unsigned src, unsigned groupX8)
{
  if (not checkSewLmulVstart(di))
    return false;

#if 0
  // Use of vstart values greater than vlmax is reserved (section 3.7 of spec).
  if (vstart > vecRegs_.vlmax())
    {
      postVecFail(di);
      return false;
    }
#endif

  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask = eg - 1;   // Assumes eg is 1, 2, 4, or 8

  // Destination eew is 1 which is less than source, overlap ok only
  // if it is in the 1st source register (dest == src).
  if (hasDestSourceOverlap(dest, 8, src, groupX8) and dest != src)
    {
      postVecFail(di);
      return false;
    }

  if ((src & mask) == 0)
    {
      vecRegs_.setOpEmul(1, eg);  // Track operand group for logging. 1 for mask.
      return true;
    }

  // Vector operand not a multiple of emul: illegal.
  postVecFail(di);
  return false;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecMaskInst(const DecodedInst* di, unsigned op0, unsigned op1,
				 unsigned op2, unsigned groupX8)
{
  if (not checkSewLmulVstart(di))
    return false;

#if 0
  // Use of vstart values greater than vlmax is reserved (section 3.7 of spec).
  if (vstart > vecRegs_.vlmax())
    {
      postVecFail(di);
      return false;
    }
#endif

  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask = eg - 1;   // Assumes eg is 1, 2, 4, or 8

  unsigned destEew = 1, destGroupX8 = 8, srcEew = vecRegs_.elemWidthInBits();
  if (not checkDestSourceOverlap(op0, destEew, destGroupX8, op1, srcEew, groupX8) or
      not checkDestSourceOverlap(op0, destEew, destGroupX8, op2, srcEew, groupX8))
    {
      postVecFail(di);
      return false;
    }

  unsigned op = op1 | op2;
  if ((op & mask) == 0)
    {
      vecRegs_.setOpEmul(1, eg, eg);  // Track operand group for logging. 1 for mask.
      return true;
    }

  // Vector operand not a multiple of emul: illegal.
  postVecFail(di);
  return false;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecOpsVsEmulW0(const DecodedInst* di, unsigned op0,
			       unsigned op1, unsigned op2, unsigned groupX8)
{
  unsigned wideGroupX8 = 2*groupX8;
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask = eg - 1;   // Assumes eg is 1, 2, 4, or 8
  unsigned eg2 = wideGroupX8 >= 8 ? wideGroupX8 / 8 : 1;
  unsigned mask2 = eg2 - 1;

  unsigned sew = vecRegs_.elemWidthInBits();
  unsigned sewx2 = sew * 2;

  // Destination EEW > source EEW, no overlap except in highest destination
  // register and only if source EEW >= 1.
  bool overlapOk = checkDestSourceOverlap(op0, sewx2, wideGroupX8, op1, sew, groupX8);
  if (op1 != op2)
    overlapOk = overlapOk and checkDestSourceOverlap(op0, sewx2, wideGroupX8, op2, sew, groupX8);

  unsigned op = op1 | op2;

  if (overlapOk and (op0 & mask2) == 0 and (op & mask) == 0)
    {
      vecRegs_.setOpEmul(eg2, eg, eg);  // Track operand group for logging
      return true;
    }

  postVecFail(di);
  return false;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecOpsVsEmulW0W1(const DecodedInst* di, unsigned op0,
				 unsigned op1, unsigned op2, unsigned groupX8)
{
  unsigned wideGroupX8 = 2*groupX8;
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask = eg - 1;   // Assumes eg is 1, 2, 4, or 8
  unsigned eg2 = wideGroupX8 >= 8 ? wideGroupX8 / 8 : 1;
  unsigned mask2 = eg2 - 1;

  unsigned sew = vecRegs_.elemWidthInBits();
  unsigned sewx2 = 2*sew;

  bool overlapOk = checkDestSourceOverlap(op0, sewx2, wideGroupX8, op2, sew, groupX8);

  unsigned opw = op0 | op1;

  if (overlapOk and (opw & mask2) == 0 and (op2 & mask) == 0)
    {
      vecRegs_.setOpEmul(eg2, eg2, eg);  // Track operand group for logging
      return true;
    }

  postVecFail(di);
  return false;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecOpsVsEmulW0W1(const DecodedInst* di, unsigned op0,
				 unsigned op1, unsigned groupX8)
{
  unsigned wideGroupX8 = 2*groupX8;
  unsigned eg2 = wideGroupX8 >= 8 ? wideGroupX8 / 8 : 1;
  unsigned mask = eg2 - 1;
  
  unsigned op = op0 | op1;

  if ((op & mask) == 0)
    {
      vecRegs_.setOpEmul(eg2, eg2);  // Track operand group for logging
      return true;
    }

  postVecFail(di);
  return false;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecOpsVsEmulW1(const DecodedInst* di, unsigned op0,
			       unsigned op1, unsigned op2, unsigned groupX8)
{
  unsigned wideGroupX8 = 2*groupX8;
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask = eg - 1;
  unsigned eg2 = wideGroupX8 >= 8 ? wideGroupX8 / 8 : 1;
  unsigned mask2 = eg2 - 1;
  
  unsigned sew = vecRegs_.elemWidthInBits();
  unsigned sewx2 = 2*sew;

  bool overlapOk = checkDestSourceOverlap(op0, sew, groupX8, op1, sewx2, wideGroupX8);

  unsigned op = op0 | op2;

  if (overlapOk and (op & mask) == 0 and (op1 & mask2) == 0)
    {
      vecRegs_.setOpEmul(eg, eg2, eg);  // Track operand group for logging
      return true;
    }

  postVecFail(di);
  return false;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecOpsVsEmulW1(const DecodedInst* di, unsigned op0,
			       unsigned op1, unsigned groupX8)
{
  unsigned wideGroupX8 = 2*groupX8;
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask = eg - 1;
  unsigned eg2 = wideGroupX8 >= 8 ? wideGroupX8 / 8 : 1;
  unsigned mask2 = eg2 - 1;
  
  unsigned sew = vecRegs_.elemWidthInBits();
  unsigned sewx2 = 2*sew;

  bool overlapOk = checkDestSourceOverlap(op0, sew, groupX8, op1, sewx2, wideGroupX8);

  if (overlapOk and (op0 & mask) == 0 and (op1 & mask2) == 0)
    {
      vecRegs_.setOpEmul(eg, eg2);  // Track operand group for logging
      return true;
    }

  postVecFail(di);
  return false;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecFpMaskInst(const DecodedInst* di, unsigned dest,
				   unsigned src, unsigned groupX8)
{
  if (not checkVecMaskInst(di, dest, src, groupX8))
    return false;

  typedef ElementWidth EW;
  EW sew = vecRegs_.elemWidth();
  bool ok = false;
  switch (sew)
    {
    case EW::Half:   ok = isZvfhLegal(); break;
    case EW::Word:   ok = isFpLegal();   break;
    case EW::Word2:  ok = isDpLegal();   break;
    default:         ok = false;         break;
    }

  // Clear soft-float library or x86 exception flags
  clearSimulatorFpFlags();

  if (not ok)
    postVecFail(di);

  return ok;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecFpMaskInst(const DecodedInst* di, unsigned dest,
				   unsigned src1, unsigned src2, unsigned groupX8)
{
  if (not checkVecMaskInst(di, dest, src1, src2, groupX8))
    return false;

  typedef ElementWidth EW;
  EW sew = vecRegs_.elemWidth();
  bool ok = false;
  switch (sew)
    {
    case EW::Half:   ok = isZvfhLegal(); break;
    case EW::Word:   ok = isFpLegal();   break;
    case EW::Word2:  ok = isDpLegal();   break;
    default:         ok = false;         break;
    }

  // Clear soft-float library or x86 exception flags
  clearSimulatorFpFlags();

  if (not ok)
    postVecFail(di);

  return ok;
}


template <typename URV>
void
Hart<URV>::vsetvl(unsigned rd, unsigned rs1, URV vtypeVal)
{
  bool ma = (vtypeVal >> 7) & 1;  // Mask agnostic
  bool ta = (vtypeVal >> 6) & 1;  // Tail agnostic
  GroupMultiplier gm = GroupMultiplier(vtypeVal & 7);
  ElementWidth ew = ElementWidth((vtypeVal >> 3) & 7);

  bool vill = (vtypeVal >> (8*sizeof(URV) - 1)) & 1;
  vill = vill or not vecRegs_.legalConfig(ew, gm);

  // Determine vl
  URV elems = 0;

  if (gm == GroupMultiplier::Reserved)
    vill = true;
  else
    {
      uint32_t gm8 = vecRegs_.groupMultiplierX8(gm);
      unsigned bitsPerElem = vecRegs_.elemWidthInBits(ew);
      unsigned vlmax = (gm8*vecRegs_.bitsPerRegister()/bitsPerElem) / 8;
      if (vlmax == 0)
        vill = true;
      else
        {
          if (rd != 0 and rs1 == 0)
            elems = vlmax;
          else if (rd == 0 and rs1 == 0)
            peekCsr(CsrNumber::VL, elems);  // Keep current value of VL.
          else  // strip mining
            {
              URV avl = intRegs_.read(rs1);  // Application vector length.
              if (avl <= vlmax)
                elems = avl;
              else if (avl >= 2*vlmax)
                elems = vlmax;
              else
		// avl > vlmax and < 2*vlmax, spec allows anything between
		// ceil(avl/2) and vlmax inclusive. We choose vlmax.
                elems = vlmax;
            }
        }

      if (elems > vlmax)
	vill = true;
    }

  if (vill)
    {
      ma = false; ta = false; gm = GroupMultiplier(0); ew = ElementWidth(0);
      elems = 0;
    }

  if (vill or (rd != 0 or rs1 != 0))
    {
      // VL is not writeable: Poke it.
      pokeCsr(CsrNumber::VL, elems);
      recordCsrWrite(CsrNumber::VL);
    }

  csRegs_.peek(CsrNumber::VL, elems);
  intRegs_.write(rd, elems);
  vecRegs_.elemCount(elems);  // Update cached value of VL.

  // Pack vtype values and update vtype
  URV vtype = 0;
  vtype |= URV(gm) | (URV(ew) << 3) | (URV(ta) << 6) | (URV(ma) << 7);
  vtype |= (URV(vill) << (8*sizeof(URV) - 1));
  pokeCsr(CsrNumber::VTYPE, vtype);
  recordCsrWrite(CsrNumber::VTYPE);

  // Update cached vtype fields in vecRegs_.
  vecRegs_.updateConfig(ew, gm, ma, ta, vill);
}


template <typename URV>
void
Hart<URV>::postVecSuccess()
{
  csRegs_.clearVstart();

  // Writing to vstart should mark VS as dirty, so do so
  // regardless of whether any vector operand registers
  // were written.
  markVsDirty();
}


template <typename URV>
void
Hart<URV>::postVecFail(const DecodedInst* di)
{
  illegalInst(di);
  if (vecRegs_.getLastWrittenReg() >= 0)
    markVsDirty();
}


template <typename URV>
void
Hart<URV>::execVsetvli(const DecodedInst* di)
{
  if (not isVecLegal())
    {
      postVecFail(di);
      return;
    }

  unsigned rd = di->op0();
  unsigned rs1 = di->op1();
  unsigned imm = di->op2();
  
  URV vtypeVal = imm;
  vsetvl(rd, rs1, vtypeVal);
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsetivli(const DecodedInst* di)
{
  if (not isVecLegal())
    {
      postVecFail(di);
      return;
    }

  unsigned rd = di->op0();
  unsigned avl = di->op1();
  unsigned imm = di->op2();
  
  bool ma = (imm >> 7) & 1;  // Mask agnostic
  bool ta = (imm >> 6) & 1;  // Tail agnostic
  GroupMultiplier gm = GroupMultiplier(imm & 7);
  ElementWidth ew = ElementWidth((imm >> 3) & 7);

  bool vill = not vecRegs_.legalConfig(ew, gm);

  // Determine vl
  URV elems = avl;
  if (gm == GroupMultiplier::Reserved)
    vill = true;
  else
    {
      uint32_t gm8 = vecRegs_.groupMultiplierX8(gm);
      unsigned bitsPerElem = vecRegs_.elemWidthInBits(ew);
      unsigned vlmax = (gm8*vecRegs_.bitsPerRegister()/bitsPerElem) / 8;
      if (vlmax == 0)
        vill = true;
      else if (elems > vlmax)
	elems = vlmax;
    }

  if (vill)
    {
      ma = false; ta = false; gm = GroupMultiplier(0); ew = ElementWidth(0);
      elems = 0;
    }

  // VL is not writeable: Poke it.
  pokeCsr(CsrNumber::VL, elems);
  recordCsrWrite(CsrNumber::VL);

  vecRegs_.elemCount(elems);  // Update cached value of VL.
  intRegs_.write(rd, elems);

  // Pack vtype values and update vtype
  URV vtype = 0;
  vtype |= URV(gm) | (URV(ew) << 3) | (URV(ta) << 6) | (URV(ma) << 7);
  vtype |= (URV(vill) << (8*sizeof(URV) - 1));
  pokeCsr(CsrNumber::VTYPE, vtype);
  recordCsrWrite(CsrNumber::VTYPE);

  // Update cached vtype fields in vecRegs_.
  vecRegs_.updateConfig(ew, gm, ma, ta, vill);
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsetvl(const DecodedInst* di)
{
  if (not isVecLegal())
    {
      postVecFail(di);
      return;
    }

  unsigned rd = di->op0();
  unsigned rs1 = di->op1();

  URV vtypeVal = intRegs_.read(di->op2());
  vsetvl(rd, rs1, vtypeVal);
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vop_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		  unsigned start, unsigned elems, bool masked,
		  std::function<ELEM_TYPE(ELEM_TYPE, ELEM_TYPE)> op)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, e2{}, dest{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = op(e1, e2);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execVop_vv(const DecodedInst* di, OP op)
{
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
      vop_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Half:
      vop_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Word:
      vop_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Word2:
      vop_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execVopu_vv(const DecodedInst* di, OP op)
{
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
      vop_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Half:
      vop_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Word:
      vop_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Word2:
      vop_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execVop_vx(const DecodedInst* di, OP op)
{
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
      vop_vx<int8_t> (vd, vs1, e2, group, start, elems, masked, op);
      break;
    case EW::Half:
      vop_vx<int16_t>(vd, vs1, e2, group, start, elems, masked, op);
      break;
    case EW::Word:
      vop_vx<int32_t>(vd, vs1, e2, group, start, elems, masked, op);
      break;
    case EW::Word2:
      vop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, op);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execVopu_vx(const DecodedInst* di, OP op)
{
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
      vop_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked, op);
      break;
    case EW::Half:
      vop_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked, op);
      break;
    case EW::Word:
      vop_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked, op);
      break;
    case EW::Word2:
      vop_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked, op);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVadd_vv(const DecodedInst* di)
{
  execVop_vv(di, std::plus());
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vop_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
		  unsigned start, unsigned elems, bool masked,
		  std::function<ELEM_TYPE(ELEM_TYPE, ELEM_TYPE)> op)
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
          dest = op(e1, e2);
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
Hart<URV>::execVadd_vx(const DecodedInst* di)
{
  execVop_vx(di, std::plus());
}


template <typename URV>
void
Hart<URV>::execVadd_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(), vs1 = di->op1();
  int32_t imm = di->op2As<int32_t>();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<int8_t> (vd, vs1, imm, group, start, elems, masked, std::plus());
      break;
    case EW::Half:
      vop_vx<int16_t>(vd, vs1, imm, group, start, elems, masked, std::plus());
      break;
    case EW::Word:
      vop_vx<int32_t>(vd, vs1, imm, group, start, elems, masked, std::plus());
      break;
    case EW::Word2:
      vop_vx<int64_t>(vd, vs1, imm, group, start, elems, masked, std::plus());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsub_vv(const DecodedInst* di)
{
  execVop_vv(di, std::minus());
}


template <typename URV>
void
Hart<URV>::execVsub_vx(const DecodedInst* di)
{
  execVop_vx(di, std::minus());
}


struct MyRsub
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  { return b - a; }
};


template <typename URV>
void
Hart<URV>::execVrsub_vx(const DecodedInst* di)
{
  execVop_vx(di, MyRsub());
}


template <typename URV>
void
Hart<URV>::execVrsub_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(), vs1 = di->op1();
  int32_t imm = di->op2As<int32_t>();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<int8_t> (vd, vs1, imm, group, start, elems, masked, MyRsub());
      break;
    case EW::Half:
      vop_vx<int16_t>(vd, vs1, imm, group, start, elems, masked, MyRsub());
      break;
    case EW::Word:
      vop_vx<int32_t>(vd, vs1, imm, group, start, elems, masked, MyRsub());
      break;
    case EW::Word2:
      vop_vx<int64_t>(vd, vs1, imm, group, start, elems, masked, MyRsub());
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
Hart<URV>::vwadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type DWT; // Double wide type
  unsigned errors = 0, wideGroup = group*2;

  ELEM_TYPE e1 = 0, e2 = 0;
  DWT dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = DWT(e1);
          dest += DWT(e2);
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
Hart<URV>::execVwaddu_vv(const DecodedInst* di)
{
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
    case EW::Byte: vwadd_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vwadd_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vwadd_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwadd_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVwadd_vv(const DecodedInst* di)
{
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
    case EW::Byte: vwadd_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vwadd_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vwadd_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwadd_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwadd_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type DWT; // Double wide type
  unsigned errors = 0, wideGroup = group*2;

  ELEM_TYPE e1 = 0;
  DWT dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = DWT(e1);
          dest += DWT(e2);
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
Hart<URV>::execVwaddu_vx(const DecodedInst* di)
{
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
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  URV e2 = intRegs_.read(di->op2());

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwadd_vx<uint8_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half: vwadd_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word: vwadd_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: vwadd_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVwadd_vx(const DecodedInst* di)
{
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
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(di->op2()));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwadd_vx<int8_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half: vwadd_vx<int16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word: vwadd_vx<int32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: vwadd_vx<int64_t>(vd, vs1, e2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwsub_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type DWT; // Double wide type
  unsigned errors = 0, wideGroup = group*2;

  ELEM_TYPE e1 = 0;
  DWT dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = DWT(e1);
          dest -= DWT(e2);
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
Hart<URV>::execVwsubu_vx(const DecodedInst* di)
{
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
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  URV e2 = intRegs_.read(di->op2()); // FIX: Spec says sign extened. We differ.

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwsub_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half: vwsub_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word: vwsub_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2:  postVecFail(di); return;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVwsub_vx(const DecodedInst* di)
{
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
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(di->op2()));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vwsub_vx<int8_t> (vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half:  vwsub_vx<int16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word:  vwsub_vx<int32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2:  postVecFail(di); return;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template<typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type DWT; // Double wide type
  unsigned errors = 0, wideGroup = group*2;

  ELEM_TYPE e1 = 0, e2 = 0;
  DWT dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = DWT(e1);
          dest -= DWT(e2);
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
Hart<URV>::execVwsubu_vv(const DecodedInst* di)
{
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
    case EW::Byte: vwsub_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vwsub_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vwsub_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwsub_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVwsub_vv(const DecodedInst* di)
{
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
    case EW::Byte: vwsub_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vwsub_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vwsub_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwsub_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwadd_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type DWT; // Double wide type
  unsigned errors = 0, wideGroup = group*2;

  ELEM_TYPE e2 = 0;
  DWT e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, wideGroup, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = e1;
          dest += DWT(e2);
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
Hart<URV>::execVwaddu_wv(const DecodedInst* di)
{
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

  if (not checkVecOpsVsEmulW0W1(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwadd_wv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vwadd_wv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vwadd_wv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwadd_wv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVwadd_wv(const DecodedInst* di)
{
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

  if (not checkVecOpsVsEmulW0W1(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwadd_wv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vwadd_wv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vwadd_wv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwadd_wv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVwaddu_wx(const DecodedInst* di)
{
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
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmulW0W1(di, vd, vs1, group))
    return;

  URV e2 = intRegs_.read(di->op2());

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<uint16_t>(vd, vs1, uint8_t(e2), group*2, start, elems, masked, std::plus());
      break;
    case EW::Half:
      vop_vx<uint32_t>(vd, vs1, uint16_t(e2), group*2, start, elems, masked, std::plus());
      break;
    case EW::Word:
      vop_vx<uint64_t>(vd, vs1, uint32_t(e2), group*2, start, elems, masked, std::plus());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVwadd_wx(const DecodedInst* di)
{
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
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmulW0W1(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(di->op2()));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<int16_t>(vd, vs1, int8_t(e2),  group*2, start, elems, masked, std::plus());
      break;
    case EW::Half:
      vop_vx<int32_t>(vd, vs1, int16_t(e2), group*2, start, elems, masked, std::plus());
      break;
    case EW::Word:
      vop_vx<int64_t>(vd, vs1, int32_t(e2), group*2, start, elems, masked, std::plus());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVwsubu_wx(const DecodedInst* di)
{
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
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmulW0W1(di, vd, vs1, group))
    return;

  URV e2 = intRegs_.read(di->op2()); // FIX: Spec says sign extened. We differ.

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<uint16_t>(vd, vs1, -uint16_t(uint8_t(e2)),  group*2, start, elems, masked, std::plus());
      break;
    case EW::Half:
      vop_vx<uint32_t>(vd, vs1, -uint32_t(uint16_t(e2)), group*2, start, elems, masked, std::plus());
      break;
    case EW::Word:
      vop_vx<uint64_t>(vd, vs1, -uint64_t(uint32_t(e2)), group*2, start, elems, masked, std::plus());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVwsub_wx(const DecodedInst* di)
{
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
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmulW0W1(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(di->op2()));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<int16_t>(vd, vs1, -int16_t(int8_t(e2)),  group*2, start, elems, masked, std::plus());
      break;
    case EW::Half:
      vop_vx<int32_t>(vd, vs1, -int32_t(int16_t(e2)), group*2, start, elems, masked, std::plus());
      break;
    case EW::Word:
      vop_vx<int64_t>(vd, vs1, -int64_t(int32_t(e2)), group*2, start, elems, masked, std::plus());
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
Hart<URV>::vwsub_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type DWT; // Double wide type
  unsigned errors = 0, wideGroup = group*2;

  ELEM_TYPE e2 = 0;
  DWT e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, wideGroup, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = e1;
          dest -= DWT(e2);
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
Hart<URV>::execVwsubu_wv(const DecodedInst* di)
{
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

  if (not checkVecOpsVsEmulW0W1(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwsub_wv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vwsub_wv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vwsub_wv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwsub_wv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVwsub_wv(const DecodedInst* di)
{
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

  if (not checkVecOpsVsEmulW0W1(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwsub_wv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vwsub_wv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vwsub_wv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwsub_wv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmop_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked,
		   std::function<bool(ELEM_TYPE, ELEM_TYPE)> op)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, e2{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchMask(vd);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          bool flag = op(e1, e2);
          if (not vecRegs_.writeMaskRegister(vd, ix, flag))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmop_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
		   unsigned start, unsigned elems, bool masked,
		   std::function<bool(ELEM_TYPE, ELEM_TYPE)> op)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchMask(vd);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          bool flag = op(e1, e2);
          if (not vecRegs_.writeMaskRegister(vd, ix, flag))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVmseq_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecMaskInst(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked, std::equal_to());
      break;
    case EW::Half:
      vmop_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked, std::equal_to());
      break;
    case EW::Word:
      vmop_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked, std::equal_to());
      break;
    case EW::Word2:
      vmop_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked, std::equal_to());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmseq_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, e2, group, start, elems, masked, std::equal_to());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, e2, group, start, elems, masked, std::equal_to());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, e2, group, start, elems, masked, std::equal_to());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, std::equal_to());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmseq_vi(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  int32_t imm = di->op2As<int32_t>();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, imm, group, start, elems, masked, std::equal_to());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, imm, group, start, elems, masked, std::equal_to());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, imm, group, start, elems, masked, std::equal_to());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, imm, group, start, elems, masked, std::equal_to());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsne_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecMaskInst(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Half:
      vmop_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Word:
      vmop_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Word2:
      vmop_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked, std::not_equal_to());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsne_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, e2, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, e2, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, e2, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, std::not_equal_to());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsne_vi(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  int32_t imm = di->op2As<int32_t>();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, imm, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, imm, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, imm, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, imm, group, start, elems, masked, std::not_equal_to());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsltu_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecMaskInst(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    case EW::Half:
      vmop_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    case EW::Word:
      vmop_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    case EW::Word2:
      vmop_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsltu_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  URV e2 = intRegs_.read(rs2);

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    case EW::Half:
      vmop_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    case EW::Word:
      vmop_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    case EW::Word2:
      vmop_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmslt_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecMaskInst(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    case EW::Half:
      vmop_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    case EW::Word:
      vmop_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    case EW::Word2:
      vmop_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmslt_vx(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsleu_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecMaskInst(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Half:
      vmop_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word:
      vmop_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word2:
      vmop_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsleu_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  URV e2 = intRegs_.read(rs2);

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Half:
      vmop_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word:
      vmop_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word2:
      vmop_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsleu_vi(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  // Immediate is sign exended and then treated as unsigned.
  int64_t imm = di->op2As<int32_t>();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<uint8_t> (vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    case EW::Half:
      vmop_vx<uint16_t>(vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word:
      vmop_vx<uint32_t>(vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word2:
      vmop_vx<uint64_t>(vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsle_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecMaskInst(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Half:
      vmop_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word:
      vmop_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word2:
      vmop_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsle_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsle_vi(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  int32_t imm = di->op2As<int32_t>();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsgtu_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  URV e2 = intRegs_.read(rs2);

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    case EW::Half:
      vmop_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    case EW::Word:
      vmop_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    case EW::Word2:
      vmop_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsgtu_vi(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  // Immediate is sign exended and then treated as unsigned.
  int64_t imm = di->op2As<int32_t>();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<uint8_t> (vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    case EW::Half:
      vmop_vx<uint16_t>(vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    case EW::Word:
      vmop_vx<uint32_t>(vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    case EW::Word2:
      vmop_vx<uint64_t>(vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsgt_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsgt_vi(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  int32_t imm = di->op2As<int32_t>();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


struct MyMin
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  { return a < b ? a : b; }
};


template <typename URV>
void
Hart<URV>::execVminu_vv(const DecodedInst* di)
{
  execVopu_vv(di, MyMin());
}


template <typename URV>
void
Hart<URV>::execVminu_vx(const DecodedInst* di)
{
  execVopu_vx(di, MyMin());
}


template <typename URV>
void
Hart<URV>::execVmin_vv(const DecodedInst* di)
{
  execVop_vv(di, MyMin());
}


template <typename URV>
void
Hart<URV>::execVmin_vx(const DecodedInst* di)
{
  execVop_vx(di, MyMin());
}


struct MyMax
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  { return a > b ? a : b; }
};


template <typename URV>
void
Hart<URV>::execVmaxu_vv(const DecodedInst* di)
{
  execVopu_vv(di, MyMax());
}


template <typename URV>
void
Hart<URV>::execVmaxu_vx(const DecodedInst* di)
{
  execVopu_vx(di, MyMax());
}


template <typename URV>
void
Hart<URV>::execVmax_vv(const DecodedInst* di)
{
  execVop_vv(di, MyMax());
}


template <typename URV>
void
Hart<URV>::execVmax_vx(const DecodedInst* di)
{
  execVop_vx(di, MyMax());
}


template <typename URV>
void
Hart<URV>::execVand_vv(const DecodedInst* di)
{
  execVopu_vv(di, std::bit_and());
}


template <typename URV>
void
Hart<URV>::execVand_vx(const DecodedInst* di)
{
  execVop_vx(di, std::bit_and());
}


template <typename URV>
void
Hart<URV>::execVand_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(), vs1 = di->op1();
  int32_t imm = di->op2As<int32_t>();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<int8_t> (vd, vs1, imm, group, start, elems, masked, std::bit_and());
      break;
    case EW::Half:
      vop_vx<int16_t>(vd, vs1, imm, group, start, elems, masked, std::bit_and());
      break;
    case EW::Word:
      vop_vx<int32_t>(vd, vs1, imm, group, start, elems, masked, std::bit_and());
      break;
    case EW::Word2:
      vop_vx<int64_t>(vd, vs1, imm, group, start, elems, masked, std::bit_and());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVor_vv(const DecodedInst* di)
{
  execVopu_vv(di, std::bit_or());
}


template <typename URV>
void
Hart<URV>::execVor_vx(const DecodedInst* di)
{
  execVop_vx(di, std::bit_or());
}


template <typename URV>
void
Hart<URV>::execVor_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(), vs1 = di->op1();
  int32_t imm = di->op2As<int32_t>();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<int8_t> (vd, vs1, imm, group, start, elems, masked, std::bit_or());
      break;
    case EW::Half:
      vop_vx<int16_t>(vd, vs1, imm, group, start, elems, masked, std::bit_or());
      break;
    case EW::Word:
      vop_vx<int32_t>(vd, vs1, imm, group, start, elems, masked, std::bit_or());
      break;
    case EW::Word2:
      vop_vx<int64_t>(vd, vs1, imm, group, start, elems, masked, std::bit_or());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVxor_vv(const DecodedInst* di)
{
  execVopu_vv(di, std::bit_xor());
}


template <typename URV>
void
Hart<URV>::execVxor_vx(const DecodedInst* di)
{
  execVop_vx(di, std::bit_xor());
}


template <typename URV>
void
Hart<URV>::execVxor_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(), vs1 = di->op1();
  int32_t imm = di->op2As<int32_t>();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<int8_t> (vd, vs1, imm, group, start, elems, masked, std::bit_xor());
      break;
    case EW::Half:
      vop_vx<int16_t>(vd, vs1, imm, group, start, elems, masked, std::bit_xor());
      break;
    case EW::Word:
      vop_vx<int32_t>(vd, vs1, imm, group, start, elems, masked, std::bit_xor());
      break;
    case EW::Word2:
      vop_vx<int64_t>(vd, vs1, imm, group, start, elems, masked, std::bit_xor());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess();
}


/// Shift left.
struct MySll
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  {
    unsigned mask = sizeof(T)*8 - 1;
    unsigned amount = b & mask;
    return a << amount;
  }
};


template <typename URV>
void
Hart<URV>::execVsll_vv(const DecodedInst* di)
{
  execVopu_vv(di, MySll());
}


template <typename URV>
void
Hart<URV>::execVsll_vx(const DecodedInst* di)
{
  execVopu_vx(di, MySll());
}


template <typename URV>
void
Hart<URV>::execVsll_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool msk = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  URV imm = di->op2();  // Unsigned -- zero extended.

  unsigned gp = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, gp))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<uint8_t> (vd, vs1, imm, gp, start, elems, msk, MySll());
      break;
    case EW::Half:
      vop_vx<uint16_t>(vd, vs1, imm, gp, start, elems, msk, MySll());
      break;
    case EW::Word:
      vop_vx<uint32_t>(vd, vs1, imm, gp, start, elems, msk, MySll());
      break;
    case EW::Word2:
      vop_vx<uint64_t>(vd, vs1, imm, gp, start, elems, msk, MySll());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


/// Shift right.
struct MySr
{
  template <typename T>
  constexpr T operator() (const T& a, const T& b) const
  {
    unsigned mask = sizeof(T)*8 - 1;
    unsigned amount = b & mask;
    return a >> amount;
  }
};


template <typename URV>
void
Hart<URV>::execVsrl_vv(const DecodedInst* di)
{
  execVopu_vv(di, MySr());
}


template <typename URV>
void
Hart<URV>::execVsrl_vx(const DecodedInst* di)
{
  execVopu_vx(di, MySr());
}


template <typename URV>
void
Hart<URV>::execVsrl_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool msk = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  URV imm = di->op2();  // Unsigned -- zero extended.

  unsigned gp = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, gp))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<uint8_t> (vd, vs1, imm, gp, start, elems, msk, MySr());
      break;
    case EW::Half:
      vop_vx<uint16_t>(vd, vs1, imm, gp, start, elems, msk, MySr());
      break;
    case EW::Word:
      vop_vx<uint32_t>(vd, vs1, imm, gp, start, elems, msk, MySr());
      break;
    case EW::Word2:
      vop_vx<uint64_t>(vd, vs1, imm, gp, start, elems, msk, MySr());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsra_vv(const DecodedInst* di)
{
  execVop_vv(di, MySr());
}


template <typename URV>
void
Hart<URV>::execVsra_vx(const DecodedInst* di)
{
  execVop_vx(di, MySr());
}


template <typename URV>
void
Hart<URV>::execVsra_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool msk = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  URV imm = di->op2();  // Unsigned -- zero extended.

  unsigned gp = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, gp))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<int8_t> (vd, vs1, imm, gp, start, elems, msk, MySr());
      break;
    case EW::Half:
      vop_vx<int16_t>(vd, vs1, imm, gp, start, elems, msk, MySr());
      break;
    case EW::Word:
      vop_vx<int32_t>(vd, vs1, imm, gp, start, elems, msk, MySr());
      break;
    case EW::Word2:
      vop_vx<int64_t>(vd, vs1, imm, gp, start, elems, msk, MySr());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vnsr_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		  unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE2X e1 = 0;
  ELEM_TYPE e2 = 0, dest = 0;

  unsigned elemBits = integerWidth<ELEM_TYPE2X> ();
  unsigned mask = elemBits - 1;
  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group2x, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = ELEM_TYPE(e1 >> (unsigned(e2) & mask));
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
Hart<URV>::execVnsrl_wv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vnsr_wv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vnsr_wv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vnsr_wv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vnsr_wv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vnsr_wx(unsigned vd, unsigned vs1, URV e2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;

  ELEM_TYPE2X e1 = 0;
  ELEM_TYPE dest = 0;

  unsigned elemBits = integerWidth<ELEM_TYPE2X> ();
  unsigned mask = elemBits - 1;
  unsigned amount = unsigned(e2) & mask;
  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group2x, e1))
        {
          dest = ELEM_TYPE(e1 >> amount);
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
Hart<URV>::execVnsrl_wx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  // Spec says sign extend scalar register. We comply.
  URV e2 = SRV(intRegs_.read(rs2));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vnsr_wx<uint8_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half:  vnsr_wx<uint16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word:  vnsr_wx<uint32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: vnsr_wx<uint64_t>(vd, vs1, e2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVnsrl_wi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool msk = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  URV imm = di->op2();   // Unsigned -- zero extended.

  unsigned gp = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, gp))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, gp))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vnsr_wx<uint8_t> (vd, vs1, imm, gp, start, elems, msk); break;
    case EW::Half:  vnsr_wx<uint16_t>(vd, vs1, imm, gp, start, elems, msk); break;
    case EW::Word:  vnsr_wx<uint32_t>(vd, vs1, imm, gp, start, elems, msk); break;
    case EW::Word2: vnsr_wx<uint64_t>(vd, vs1, imm, gp, start, elems, msk); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVnsra_wv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vnsr_wv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vnsr_wv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vnsr_wv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vnsr_wv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVnsra_wx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  // Spec says sign extend scalar register. We comply.
  URV e2 = SRV(intRegs_.read(rs2));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vnsr_wx<int8_t> (vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half:  vnsr_wx<int16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word:  vnsr_wx<int32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: vnsr_wx<int64_t>(vd, vs1, e2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVnsra_wi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool msk = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  URV imm = di->op2();   // Unsigned -- zero extended

  unsigned gp = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, gp))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, gp))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vnsr_wx<int8_t> (vd, vs1, imm, gp, start, elems, msk); break;
    case EW::Half:  vnsr_wx<int16_t>(vd, vs1, imm, gp, start, elems, msk); break;
    case EW::Word:  vnsr_wx<int32_t>(vd, vs1, imm, gp, start, elems, msk); break;
    case EW::Word2: vnsr_wx<int64_t>(vd, vs1, imm, gp, start, elems, msk); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vrgather_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                       unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2))
        {
          dest = 0;
	  if (e2 < vecRegs_.bytesPerRegister() * 8)
	    {
	      unsigned vs1Ix = unsigned(e2);
	      if (vecRegs_.read(vs1, vs1Ix, group, e1))
		dest = e1;
	    }

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
Hart<URV>::execVrgather_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (hasDestSourceOverlap(vd, group, vs1, group) or
      hasDestSourceOverlap(vd, group, vs2, group))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vrgather_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vrgather_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vrgather_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vrgather_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vrgather_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                       unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;

  unsigned bytesPerElem = sizeof(ELEM_TYPE);
  unsigned vlmax = group*vecRegs_.bitsPerRegister()/bytesPerElem;

  URV rv2 = intRegs_.read(rs2);
  URV vs1Ix = rv2 < vlmax ? rv2 : vlmax;

  ELEM_TYPE e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      dest = 0;
      if (vecRegs_.read(vs1, vs1Ix, group, e1))
        dest = e1;

      if (not vecRegs_.write(vd, ix, group, dest))
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVrgather_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (hasDestSourceOverlap(vd, group, vs1, group))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vrgather_vx<uint8_t> (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half:  vrgather_vx<uint16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vrgather_vx<uint32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vrgather_vx<uint64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vrgather_vi(unsigned vd, unsigned vs1, uint32_t imm, unsigned group,
                       unsigned start, unsigned elems, bool masked)
{
  uint32_t vs1Ix = imm;
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      dest = 0;
      if (vecRegs_.read(vs1, vs1Ix, group, e1))
        dest = e1;

      if (not vecRegs_.write(vd, ix, group, dest))
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVrgather_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  uint32_t vd = di->op0(),  vs1 = di->op1(),  imm = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (hasDestSourceOverlap(vd, group, vs1, group))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vrgather_vi<uint8_t> (vd, vs1, imm, group, start, elems, masked); break;
    case EW::Half:  vrgather_vi<uint16_t>(vd, vs1, imm, group, start, elems, masked); break;
    case EW::Word:  vrgather_vi<uint32_t>(vd, vs1, imm, group, start, elems, masked); break;
    case EW::Word2: vrgather_vi<uint64_t>(vd, vs1, imm, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}




template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vrgatherei16_vv(unsigned vd, unsigned vs1, unsigned vs2,
                           unsigned group, unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;
  uint16_t e2 = 0;
  unsigned e2Group = (16*group)/(8*sizeof(ELEM_TYPE));

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, e2Group, e2))
        {
          unsigned vs1Ix = e2;
          dest = 0;
          if (vecRegs_.read(vs1, vs1Ix, group, e1))
            dest = e1;

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
Hart<URV>::execVrgatherei16_vv(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  unsigned widthInBytes = vecRegs_.elemWidthInBytes(sew);
  bool masked = di->isMasked();

  unsigned v2Group = (2*group) / widthInBytes;

  GroupMultiplier v2gm = GroupMultiplier::One;
  if (not vecRegs_.groupNumberX8ToSymbol(v2Group, v2gm) or
      not vecRegs_.legalConfig(ElementWidth::Half, v2gm) or
      (masked and vd == 0))
    {
      postVecFail(di);
      return;
    }

  unsigned eg = group >= 8 ? group / 8 : 1;
  unsigned v2g = v2Group >= 8 ? v2Group / 8 : 1;

  if ((vd % eg) or (vs1 % eg) or (vs2 % v2g))
    {
      postVecFail(di);
      return;
    }

  if (hasDestSourceOverlap(vd, group, vs1, group) or
      hasDestSourceOverlap(vd, group, vs2, v2Group))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  vecRegs_.setOpEmul(eg, eg, v2g);  // Track operand group for logging

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vrgatherei16_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vrgatherei16_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vrgatherei16_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vrgatherei16_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vcompress_vm(unsigned vd, unsigned vs1, unsigned vs2,
                        unsigned group, unsigned start, unsigned elems)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;
  unsigned destIx = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isActive(vs2, ix))
        {
          if (vecRegs_.read(vs1, ix, group, e1))
            {
              dest = e1;
              if (not vecRegs_.write(vd, destIx++, group, dest))
                errors++;
            }
          else
            errors++;
        }
      else
	vecRegs_.touchReg(vd, group);
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVcompress_vm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  // vs2 is a mask register: elmul2 is 1

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  if (hasDestSourceOverlap(vd, group, vs1, group) or
      hasDestSourceOverlap(vd, group, vs2, 1) or di->isMasked() or start > 0)
    {
      postVecFail(di);  // Source/dest cannot overlap, must not be masked, 0 vstart.
      return;
    }

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vcompress_vm<uint8_t>(vd, vs1, vs2, group, start, elems); break;
    case EW::Half: vcompress_vm<uint16_t>(vd, vs1, vs2, group, start, elems); break;
    case EW::Word: vcompress_vm<uint32_t>(vd, vs1, vs2, group, start, elems); break;
    case EW::Word2: vcompress_vm<uint64_t>(vd, vs1, vs2, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vredsum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, result = 0;
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.read(vs1, ix, group, e1))
	result += e1;
      else
	errors++;
    }

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVredsum_vs(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vredsum_vs<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vredsum_vs<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vredsum_vs<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vredsum_vs<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vredand_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, result = 0;
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result))
    errors++;
  
  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;
      
      if (vecRegs_.read(vs1, ix, group, e1))
	result = result & e1;
      else
	errors++;
    }

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVredand_vs(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vredand_vs<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vredand_vs<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vredand_vs<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vredand_vs<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vredor_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, result = 0;
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result))
    errors++;
  
  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.read(vs1, ix, group, e1))
	result = result | e1;
      else
	errors++;
    }

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVredor_vs(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vredor_vs<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vredor_vs<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vredor_vs<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vredor_vs<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vredxor_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, result = 0;
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result))
    errors++;
  
  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.read(vs1, ix, group, e1))
	result = result ^ e1;
      else
	errors++;
    }

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVredxor_vs(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vredxor_vs<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vredxor_vs<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vredxor_vs<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vredxor_vs<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vredminu_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                       unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, result = 0;
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.read(vs1, ix, group, e1))
	result = result < e1 ? result : e1;
      else
	errors++;
    }

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVredminu_vs(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vredminu_vs<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vredminu_vs<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vredminu_vs<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vredminu_vs<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vredmin_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, result = 0;
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;
      
      if (vecRegs_.read(vs1, ix, group, e1))
	result = result < e1 ? result : e1;
      else
	errors++;
    }

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVredmin_vs(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vredmin_vs<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vredmin_vs<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vredmin_vs<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vredmin_vs<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vredmaxu_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                       unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, result = 0;
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;
      
      if (vecRegs_.read(vs1, ix, group, e1))
	result = result > e1 ? result : e1;
      else
	errors++;
    }

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVredmaxu_vs(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vredmaxu_vs<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vredmaxu_vs<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vredmaxu_vs<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vredmaxu_vs<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vredmax_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, result = 0;
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;
      
      if (vecRegs_.read(vs1, ix, group, e1))
	result = result > e1 ? result : e1;
      else
	errors++;
    }

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVredmax_vs(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(), start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vredmax_vs<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vredmax_vs<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vredmax_vs<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vredmax_vs<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwredsum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X;

  unsigned errors = 0;
  ELEM_TYPE2X result = 0;
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result))
    errors++;
  
  ELEM_TYPE e1 = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.read(vs1, ix, group, e1))
	{
	  ELEM_TYPE2X e1dw = e1;
	  result += e1dw;
	}
      else
	errors++;
    }

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVwredsumu_vs(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vwredsum_vs<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwredsum_vs<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwredsum_vs<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVwredsum_vs(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vwredsum_vs<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwredsum_vs<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwredsum_vs<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmand_mm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  if ((elems+7)/8 > vecRegs_.bytesPerRegister())
    assert(0);

  uint8_t* vdData = vecRegs_.getVecData(di->op0());
  uint8_t* vs1Data = vecRegs_.getVecData(di->op1());
  uint8_t* vs2Data = vecRegs_.getVecData(di->op2());
  if (not vs1Data or not vs2Data or not vdData)
    assert(0);

  // If bits indices are byte aligned process bytes
  if ((start & 7) == 0 and (elems & 7) == 0)
    {
      start = start >> 3;
      elems = elems >> 3;
      for (unsigned i = start; i < elems; ++i)
        vdData[i] = vs1Data[i] & vs2Data[i];
    }
  else     // Bit indices are not byte aligned.
    for (unsigned i = start; i < elems; ++i)
      {
        unsigned byteIx = i >> 3;
        unsigned bitIx = i & 7; // Bit index in byte
        uint8_t mask = 1 << bitIx;
        vdData[byteIx] = ( (vdData[byteIx] & ~mask) |
                           (vs1Data[byteIx] & vs2Data[byteIx] & mask) );
      }

  vecRegs_.touchMask(di->op0());
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmnand_mm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  if ((elems+7)/8 > vecRegs_.bytesPerRegister())
    assert(0);

  uint8_t* vdData = vecRegs_.getVecData(di->op0());
  uint8_t* vs1Data = vecRegs_.getVecData(di->op1());
  uint8_t* vs2Data = vecRegs_.getVecData(di->op2());
  if (not vs1Data or not vs2Data or not vdData)
    assert(0);

  // If bits indices are byte aligned process bytes
  if ((start & 7) == 0 and (elems & 7) == 0)
    {
      start = start >> 3;
      elems = elems >> 3;
      for (unsigned i = start; i < elems; ++i)
        vdData[i] = ~ (vs1Data[i] & vs2Data[i]);
    }
  else    // Bit indices are not byte aligned.
    for (unsigned i = start; i < elems; ++i)
      {
        unsigned byteIx = i >> 3;
        unsigned bitIx = i & 7; // Bit index in byte
        uint8_t mask = 1 << bitIx;
        vdData[byteIx] = ( (vdData[byteIx] & ~mask) |
                           (~(vs1Data[byteIx] & vs2Data[byteIx]) & mask) );
      }

  vecRegs_.touchMask(di->op0());
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmandnot_mm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  if ((elems+7)/8 > vecRegs_.bytesPerRegister())
    assert(0);

  uint8_t* vdData = vecRegs_.getVecData(di->op0());
  uint8_t* vs1Data = vecRegs_.getVecData(di->op1());
  uint8_t* vs2Data = vecRegs_.getVecData(di->op2());
  if (not vs1Data or not vs2Data or not vdData)
    assert(0);

  // If bits indices are byte aligned process bytes
  if ((start & 7) == 0 and (elems & 7) == 0)
    {
      start = start >> 3;
      elems = elems >> 3;
      for (unsigned i = start; i < elems; ++i)
        vdData[i] = vs1Data[i] & ~vs2Data[i];
    }
  else    // Bit indices are not byte aligned.
    for (unsigned i = start; i < elems; ++i)
      {
        unsigned byteIx = i >> 3;
        unsigned bitIx = i & 7; // Bit index in byte
        uint8_t mask = 1 << bitIx;
        vdData[byteIx] = ( (vdData[byteIx] & ~mask) |
                           ((vs1Data[byteIx] & ~vs2Data[byteIx]) & mask) );
      }

  vecRegs_.touchMask(di->op0());
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmxor_mm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  if ((elems+7)/8 > vecRegs_.bytesPerRegister())
    assert(0);

  uint8_t* vdData = vecRegs_.getVecData(di->op0());
  uint8_t* vs1Data = vecRegs_.getVecData(di->op1());
  uint8_t* vs2Data = vecRegs_.getVecData(di->op2());
  if (not vs1Data or not vs2Data or not vdData)
    assert(0);

  // If bits indices are byte aligned process bytes
  if ((start & 7) == 0 and (elems & 7) == 0)
    {
      start = start >> 3;
      elems = elems >> 3;
      for (unsigned i = start; i < elems; ++i)
        vdData[i] = vs1Data[i] ^ vs2Data[i];
    }
  else    // Bit indices are not byte aligned.
    for (unsigned i = start; i < elems; ++i)
      {
        unsigned byteIx = i >> 3;
        unsigned bitIx = i & 7; // Bit index in byte
        uint8_t mask = 1 << bitIx;
        vdData[byteIx] = ( (vdData[byteIx] & ~mask) |
                           ((vs1Data[byteIx] ^ vs2Data[byteIx]) & mask) );
      }

  vecRegs_.touchMask(di->op0());
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmor_mm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  if ((elems+7)/8 > vecRegs_.bytesPerRegister())
    assert(0);

  uint8_t* vdData = vecRegs_.getVecData(di->op0());
  uint8_t* vs1Data = vecRegs_.getVecData(di->op1());
  uint8_t* vs2Data = vecRegs_.getVecData(di->op2());
  if (not vs1Data or not vs2Data or not vdData)
    assert(0);

  // If bits indices are byte aligned process bytes
  if ((start & 7) == 0 and (elems & 7) == 0)
    {
      start = start >> 3;
      elems = elems >> 3;
      for (unsigned i = start; i < elems; ++i)
        vdData[i] = vs1Data[i] | vs2Data[i];
    }
  else    // Bit indices are not byte aligned.
    for (unsigned i = start; i < elems; ++i)
      {
        unsigned byteIx = i >> 3;
        unsigned bitIx = i & 7; // Bit index in byte
        uint8_t mask = 1 << bitIx;
        vdData[byteIx] = ( (vdData[byteIx] & ~mask) |
                           ((vs1Data[byteIx] | vs2Data[byteIx]) & mask) );
      }

  vecRegs_.touchMask(di->op0());
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmnor_mm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  if ((elems+7)/8 > vecRegs_.bytesPerRegister())
    assert(0);

  uint8_t* vdData = vecRegs_.getVecData(di->op0());
  uint8_t* vs1Data = vecRegs_.getVecData(di->op1());
  uint8_t* vs2Data = vecRegs_.getVecData(di->op2());
  if (not vs1Data or not vs2Data or not vdData)
    assert(0);

  // If bits indices are byte aligned process bytes
  if ((start & 7) == 0 and (elems & 7) == 0)
    {
      start = start >> 3;
      elems = elems >> 3;
      for (unsigned i = start; i < elems; ++i)
        vdData[i] = ~(vs1Data[i] | vs2Data[i]);
    }
  else    // Bit indices are not byte aligned.
    for (unsigned i = start; i < elems; ++i)
      {
        unsigned byteIx = i >> 3;
        unsigned bitIx = i & 7; // Bit index in byte
        uint8_t mask = 1 << bitIx;
        vdData[byteIx] = ( (vdData[byteIx] & ~mask) |
                           ( ~(vs1Data[byteIx] | vs2Data[byteIx]) & mask) );
      }

  vecRegs_.touchMask(di->op0());
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmornot_mm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  if ((elems+7)/8 > vecRegs_.bytesPerRegister())
    assert(0);

  uint8_t* vdData = vecRegs_.getVecData(di->op0());
  uint8_t* vs1Data = vecRegs_.getVecData(di->op1());
  uint8_t* vs2Data = vecRegs_.getVecData(di->op2());
  if (not vs1Data or not vs2Data or not vdData)
    assert(0);

  // If bits indices are byte aligned process bytes
  if ((start & 7) == 0 and (elems & 7) == 0)
    {
      start = start >> 3;
      elems = elems >> 3;
      for (unsigned i = start; i < elems; ++i)
        vdData[i] = vs1Data[i] | ~vs2Data[i];
    }
  else     // Bit indices are not byte aligned.
    for (unsigned i = start; i < elems; ++i)
      {
        unsigned byteIx = i >> 3;
        unsigned bitIx = i & 7; // Bit index in byte
        uint8_t mask = 1 << bitIx;
        vdData[byteIx] = ( (vdData[byteIx] & ~mask) |
                           ((vs1Data[byteIx] | ~vs2Data[byteIx]) & mask) );
      }

  vecRegs_.touchMask(di->op0());
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmxnor_mm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  if ((elems+7)/8 > vecRegs_.bytesPerRegister())
    assert(0);

  uint8_t* vdData = vecRegs_.getVecData(di->op0());
  uint8_t* vs1Data = vecRegs_.getVecData(di->op1());
  uint8_t* vs2Data = vecRegs_.getVecData(di->op2());
  if (not vs1Data or not vs2Data or not vdData)
    assert(0);

  // If bits indices are byte aligned process bytes
  if ((start & 7) == 0 and (elems & 7) == 0)
    {
      start = start >> 3;
      elems = elems >> 3;
      for (unsigned i = start; i < elems; ++i)
        vdData[i] = vs1Data[i] ^ ~vs2Data[i];
    }
  else     // Bit indices are not byte aligned.
    for (unsigned i = start; i < elems; ++i)
      {
        unsigned byteIx = i >> 3;
        unsigned bitIx = i & 7; // Bit index in byte
        uint8_t mask = 1 << bitIx;
        vdData[byteIx] = ( (vdData[byteIx] & ~mask) |
                           ((vs1Data[byteIx] ^ ~vs2Data[byteIx]) & mask) );
      }

  vecRegs_.touchMask(di->op0());
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVcpop_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not isVecLegal() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned rd = di->op0(),  vs1 = di->op1(),  elems = vecRegs_.elemCount();;

  uint32_t count = 0;
  for (uint32_t ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.isActive(vs1, ix))
        count++;
    }

  intRegs_.write(rd, count);
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfirst_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not isVecLegal() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned rd = di->op0(),  vs1 = di->op1(),  elems = vecRegs_.elemCount();

  SRV first = -1;

  for (uint32_t ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.isActive(vs1, ix))
	{
	  first = ix;
	  break;
	}
    }

  intRegs_.write(rd, first);
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsbf_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not isVecLegal() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  elems = vecRegs_.elemCount();;

  if (vd == vs1 or (masked and vd == 0))
    {
      postVecFail(di);
      return;
    }

  uint8_t* vdData = vecRegs_.getVecData(vd);
  uint8_t* vs1Data = vecRegs_.getVecData(vs1);

  bool found = false;  // true if set bit is found in vs1

  for (uint32_t ix = start; ix < elems; ++ix)
    {
      unsigned byteIx = ix >> 3;
      unsigned bitIx = ix & 7; // Bit index in byte
      uint8_t mask = 1 << bitIx;

      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      found = found or (vs1Data[byteIx] & mask);
      vdData[byteIx] = vdData[byteIx] & ~mask;
      if (not found)
	vdData[byteIx] |= mask;
    }

  vecRegs_.touchMask(vd);
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsif_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not isVecLegal() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  elems = vecRegs_.elemCount();;

  if (vd == vs1 or (masked and vd == 0))
    {
      postVecFail(di);
      return;
    }

  uint8_t* vdData = vecRegs_.getVecData(vd);
  uint8_t* vs1Data = vecRegs_.getVecData(vs1);

  bool found = false;  // true if set bit is found in vs1

  for (uint32_t ix = start; ix < elems; ++ix)
    {
      unsigned byteIx = ix >> 3;
      unsigned bitIx = ix & 7; // Bit index in byte
      uint8_t mask = 1 << bitIx;
      uint8_t inputByte = vs1Data[byteIx];

      if ((not masked) or vecRegs_.isActive(0, ix))
	{
	  vdData[byteIx] = vdData[byteIx] & ~mask;
	  if (not found)
	    vdData[byteIx] |= mask;
	  found = found or (inputByte & mask);
	}
    }

  vecRegs_.touchMask(vd);
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsof_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not isVecLegal() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  elems = vecRegs_.elemCount();;

  if (vd == vs1 or (masked and vd == 0))
    {
      postVecFail(di);
      return;
    }

  uint8_t* vdData = vecRegs_.getVecData(vd);
  uint8_t* vs1Data = vecRegs_.getVecData(vs1);

  bool found = false;  // true if set bit is found in vs1

  for (uint32_t ix = start; ix < elems; ++ix)
    {
      unsigned byteIx = ix >> 3;
      unsigned bitIx = ix & 7; // Bit index in byte
      uint8_t mask = 1 << bitIx;

      bool active = (not masked) or vecRegs_.isActive(0, ix);
      bool inputSet = vs1Data[byteIx] & mask;

      if (active)
	vdData[byteIx] &= ~mask;

      if (found or not inputSet)
	continue;

      if (active)
	{
	  found = true;
	  vdData[byteIx] |= mask;
	}
    }

  vecRegs_.touchMask(vd);
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execViota_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  unsigned vd = di->op0(),  vs1 = di->op1(),  elems = vecRegs_.elemCount();;
  unsigned group = vecRegs_.groupMultiplierX8();

  if (not isVecLegal() or not vecRegs_.legalConfig() or start > 0 or
      (vs1 >= vd and vs1 < vd + group/8))
    {
      postVecFail(di);
      return;
    }

  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (masked and vd == 0)
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, group))
    return;

  unsigned sum = 0;

  for (uint32_t ix = start; ix < elems; ++ix)
    {
      bool sourceSet = vecRegs_.isActive(vs1, ix);

      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      switch (sew)
        {
        case ElementWidth::Byte: vecRegs_.write(vd, ix, group, int8_t(sum)); break;
        case ElementWidth::Half: vecRegs_.write(vd, ix, group, int16_t(sum)); break;
        case ElementWidth::Word: vecRegs_.write(vd, ix, group, int32_t(sum)); break;
        case ElementWidth::Word2: vecRegs_.write(vd, ix, group, int64_t(sum)); break;
	default:  postVecFail(di); return;
        }

      if (sourceSet)
        sum++;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVid_v(const DecodedInst* di)
{
  // Spec does not mention vstart > 0. Got a clarification saying it
  // is ok not to take an exception in that case.
  uint32_t start = csRegs_.peekVstart();
  if (not isVecLegal() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  elems = vecRegs_.elemCount();;

  if (masked and vd == 0)
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, group))
    return;

  for (uint32_t ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      switch (sew)
        {
        case ElementWidth::Byte: vecRegs_.write(vd, ix, group, int8_t(ix)); break;
        case ElementWidth::Half: vecRegs_.write(vd, ix, group, int16_t(ix)); break;
        case ElementWidth::Word: vecRegs_.write(vd, ix, group, int32_t(ix)); break;
        case ElementWidth::Word2: vecRegs_.write(vd, ix, group, int64_t(ix)); break;
	default:  postVecFail(di); return;
        }
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vslideup(unsigned vd, unsigned vs1, URV amount, unsigned group,
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

      if (ix < amount)
        continue;

      unsigned from = ix - amount;

      if (vecRegs_.read(vs1, from, group, e1))
        {
          dest = e1;
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
Hart<URV>::execVslideup_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (hasDestSourceOverlap(vd, group, vs1, group))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  URV amount = intRegs_.read(rs2);

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vslideup<uint8_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Half: vslideup<uint16_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word: vslideup<uint32_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word2: vslideup<uint64_t>(vd, vs1, amount, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVslideup_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), imm = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (hasDestSourceOverlap(vd, group, vs1, group))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  URV amount = imm;

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vslideup<uint8_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Half: vslideup<uint16_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word: vslideup<uint32_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word2: vslideup<uint64_t>(vd, vs1, amount, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVslide1up_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (hasDestSourceOverlap(vd, group, vs1, group))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;
  if (start >= elems)
    return;

  URV amount = 1;

  // Sign extend scalar value
  SRV replacement = SRV(intRegs_.read(rs2));

  switch (sew)
    {
    case ElementWidth::Byte:
      vslideup<uint8_t>(vd, vs1, amount, group, start, elems, masked);
      if (not masked or vecRegs_.isActive(0, 0))
	vecRegs_.write(vd, 0, group, int8_t(replacement));
      break;

    case ElementWidth::Half:
      vslideup<uint16_t>(vd, vs1, amount, group, start, elems, masked);
      if (not masked or vecRegs_.isActive(0, 0))
	vecRegs_.write(vd, 0, group, int16_t(replacement));
      break;

    case ElementWidth::Word:
      vslideup<uint32_t>(vd, vs1, amount, group, start, elems, masked);
      if (not masked or vecRegs_.isActive(0, 0))
	vecRegs_.write(vd, 0, group, int32_t(replacement));
      break;

    case ElementWidth::Word2:
      vslideup<uint64_t>(vd, vs1, amount, group, start, elems, masked);
      if (not masked or vecRegs_.isActive(0, 0))
	vecRegs_.write(vd, 0, group, int64_t(replacement));
      break;

    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vslidedown(unsigned vd, unsigned vs1, URV amount, unsigned group,
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

      e1 = 0;
      if (amount < vecRegs_.bytesInRegisterFile())
	{
	  URV from = ix + amount;
	  vecRegs_.read(vs1, from, group, e1); // no-op if from out of bounds
	}
      dest = e1;
      if (not vecRegs_.write(vd, ix, group, dest))
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVslidedown_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  URV amount = intRegs_.read(rs2);

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   vslidedown<uint8_t> (vd, vs1, amount, group, start, elems, masked); break;
    case EW::Half:   vslidedown<uint16_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word:   vslidedown<uint32_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word2:  vslidedown<uint64_t>(vd, vs1, amount, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVslidedown_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), imm = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  URV amount = imm;

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vslidedown<uint8_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Half: vslidedown<uint16_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word: vslidedown<uint32_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word2: vslidedown<uint64_t>(vd, vs1, amount, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVslide1down_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;
  if (start >= elems)
    return;

  URV amount = 1;

  // Sign extend scalar value
  SRV replacement = SRV(intRegs_.read(rs2));

  switch (sew)
    {
    case ElementWidth::Byte:
      vslidedown<uint8_t>(vd, vs1, amount, group, start, elems, masked);
      if (not masked or vecRegs_.isActive(0, elems-1))
	vecRegs_.write(vd, elems-1, group, int8_t(replacement));
      break;

    case ElementWidth::Half:
      vslidedown<uint16_t>(vd, vs1, amount, group, start, elems, masked);
      if (not masked or vecRegs_.isActive(0, elems-1))
	vecRegs_.write(vd, elems-1, group, int16_t(replacement));
      break;

    case ElementWidth::Word:
      vslidedown<uint32_t>(vd, vs1, amount, group, start, elems, masked);
      if (not masked or vecRegs_.isActive(0, elems-1))
	vecRegs_.write(vd, elems-1, group, int32_t(replacement));
      break;

    case ElementWidth::Word2:
      vslidedown<uint64_t>(vd, vs1, amount, group, start, elems, masked);
      if (not masked or vecRegs_.isActive(0, elems-1))
	vecRegs_.write(vd, elems-1, group, int64_t(replacement));
      break;

    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfslide1up_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (hasDestSourceOverlap(vd, group, vs1, group))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;
  if (elems == 0)
    return;

  URV amount = 1;

  switch (sew)
    {
    case ElementWidth::Byte:
      postVecFail(di);
      return;

    case ElementWidth::Half:
      {
	vslideup<uint16_t>(vd, vs1, amount, group, start, elems, masked);
	if (not masked or vecRegs_.isActive(0, 0))
          {
            Float16 f = fpRegs_.readHalf(rs2);
            vecRegs_.write(vd, 0, group, std::bit_cast<uint16_t>(f));
          }
      }
      break;

    case ElementWidth::Word:
      {
	vslideup<uint32_t>(vd, vs1, amount, group, start, elems, masked);
	if (not masked or vecRegs_.isActive(0, 0))
	  {
            float f = fpRegs_.readSingle(rs2);
            vecRegs_.write(vd, 0, group, std::bit_cast<uint32_t>(f));
	  }
      }
      break;

    case ElementWidth::Word2:
      {
	vslideup<uint64_t>(vd, vs1, amount, group, start, elems, masked);
	if (not masked or vecRegs_.isActive(0, 0))
	  {
            double d = fpRegs_.readDouble(rs2);
            vecRegs_.write(vd, 0, group, std::bit_cast<uint64_t>(d));
	  }
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
Hart<URV>::execVfslide1down_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;
  if (elems == 0)
    return;

  URV amount = 1;

  switch (sew)
    {
    case ElementWidth::Byte:
      postVecFail(di);
      return;

    case ElementWidth::Half:
      {
	vslidedown<uint16_t>(vd, vs1, amount, group, start, elems, masked);
        if (not masked or vecRegs_.isActive(0, elems - 1))
          {
            Float16 f = fpRegs_.readHalf(rs2);
            vecRegs_.write(vd, elems - 1, group, std::bit_cast<uint16_t>(f));
          }
      }
      break;

    case ElementWidth::Word:
      {
	vslidedown<uint32_t>(vd, vs1, amount, group, start, elems, masked);
	if (not masked or vecRegs_.isActive(0, elems-1))
	  {
            float f = fpRegs_.readSingle(rs2);
            vecRegs_.write(vd, elems-1, group, std::bit_cast<uint32_t>(f));
	  }
      }
      break;

    case ElementWidth::Word2:
      {
	vslidedown<uint64_t>(vd, vs1, amount, group, start, elems, masked);
	if (not masked or vecRegs_.isActive(0, elems-1))
	  {
            double d = fpRegs_.readDouble(rs2);
            vecRegs_.write(vd, elems-1, group, std::bit_cast<uint64_t>(d));
	  }
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
Hart<URV>::execVmul_vv(const DecodedInst* di)
{
  execVop_vv(di, std::multiplies());
}


template <typename URV>
void
Hart<URV>::execVmul_vx(const DecodedInst* di)
{
  execVop_vx(di, std::multiplies());
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmulh_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          mulh<ELEM_TYPE>(e1, e2, dest);
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
Hart<URV>::execVmulh_vv(const DecodedInst* di)
{
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
    case EW::Byte: vmulh_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vmulh_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vmulh_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmulh_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmulh_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = SRV(intRegs_.read(rs2)), dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          mulh(e1, e2, dest);
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
Hart<URV>::execVmulh_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vmulh_vx<int8_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half: vmulh_vx<int16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word: vmulh_vx<int32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vmulh_vx<int64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmulhu_vv(const DecodedInst* di)
{
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
    case EW::Byte: vmulh_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vmulh_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vmulh_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmulh_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmulhu_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = intRegs_.read(rs2), dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          mulh(e1, e2, dest);
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
Hart<URV>::execVmulhu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vmulhu_vx<uint8_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half: vmulhu_vx<uint16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word: vmulhu_vx<uint32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vmulhu_vx<uint64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmulhsu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                      unsigned start, unsigned elems, bool masked)
{
  typedef typename std::make_unsigned<ELEM_TYPE>::type  U_ELEM_TYPE;

  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;
  U_ELEM_TYPE e2 = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          mulhsu(e1, e2, dest);
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
Hart<URV>::execVmulhsu_vv(const DecodedInst* di)
{
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
    case EW::Byte: vmulhsu_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vmulhsu_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vmulhsu_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmulhsu_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmulhsu_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                      unsigned start, unsigned elems, bool masked)
{
  typedef typename std::make_unsigned<ELEM_TYPE>::type  U_ELEM_TYPE;

  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;
  U_ELEM_TYPE e2 = intRegs_.read(rs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          mulhsu(e1, e2, dest);
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
Hart<URV>::execVmulhsu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vmulhsu_vx<int8_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half: vmulhsu_vx<int16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word: vmulhsu_vx<int32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vmulhsu_vx<int64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group, dest))
        {
	  dest = (e1 * dest) + e2;
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
Hart<URV>::execVmadd_vv(const DecodedInst* di)
{
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
    case EW::Byte: vmadd_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vmadd_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vmadd_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmadd_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmadd_vx(unsigned vd, unsigned rs1, unsigned v2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2 = 0, dest = 0;
  ELEM_TYPE e1 = SRV(intRegs_.read(rs1));

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(v2, ix, group, e2) and vecRegs_.read(vd, ix, group, dest))
        {
	  dest = (e1 * dest) + e2;
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
Hart<URV>::execVmadd_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs2, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vmadd_vx<int8_t> (vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vmadd_vx<int16_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vmadd_vx<int32_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmadd_vx<int64_t>(vd, rs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vnmsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group, dest))
        {
	  dest = -(e1 * dest) + e2;
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
Hart<URV>::execVnmsub_vv(const DecodedInst* di)
{
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
    case EW::Byte: vnmsub_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vnmsub_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vnmsub_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vnmsub_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vnmsub_vx(unsigned vd, unsigned rs1, unsigned v2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2 = 0, dest = 0;
  ELEM_TYPE e1 = SRV(intRegs_.read(rs1));

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(v2, ix, group, e2) and vecRegs_.read(vd, ix, group, dest))
        {
	  dest = -(e1 * dest) + e2;
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
Hart<URV>::execVnmsub_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs2, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vnmsub_vx<int8_t> (vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vnmsub_vx<int16_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vnmsub_vx<int32_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vnmsub_vx<int64_t>(vd, rs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group, dest))
        {
	  dest = (e1 * e2) + dest;
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
Hart<URV>::execVmacc_vv(const DecodedInst* di)
{
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
    case EW::Byte: vmacc_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vmacc_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vmacc_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmacc_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmacc_vx(unsigned vd, unsigned rs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2 = 0, dest = 0;
  ELEM_TYPE e1 = SRV(intRegs_.read(rs1));

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, group, dest))
        {
	  dest = (e1 * e2) + dest;
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
Hart<URV>::execVmacc_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs2, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vmacc_vx<int8_t> (vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vmacc_vx<int16_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vmacc_vx<int32_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmacc_vx<int64_t>(vd, rs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vnmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group, dest))
        {
	  dest = -(e1 * e2) + dest;
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
Hart<URV>::execVnmsac_vv(const DecodedInst* di)
{
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
    case EW::Byte: vnmsac_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vnmsac_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vnmsac_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vnmsac_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vnmsac_vx(unsigned vd, unsigned rs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2 = 0, dest = 0;
  ELEM_TYPE e1 = SRV(intRegs_.read(rs1));

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, group, dest))
        {
	  dest = -(e1 * e2) + dest;
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
Hart<URV>::execVnmsac_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs2, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vnmsac_vx<int8_t> (vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vnmsac_vx<int16_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vnmsac_vx<int32_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vnmsac_vx<int64_t>(vd, rs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmulu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type  ELEM_TYPE_X2;

  ELEM_TYPE e1 = 0, e2 = 0;
  ELEM_TYPE_X2 dest = 0;

  unsigned errors = 0, wideGroup = group * 2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = ELEM_TYPE_X2(e1);
          dest *= e2;
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
Hart<URV>::execVwmulu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  // Double wide legal.
  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwmulu_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vwmulu_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vwmulu_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmulu_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmulu_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type  ELEM_TYPE_X2;

  ELEM_TYPE e1 = 0;
  ELEM_TYPE_X2 dest = 0;
  unsigned errors = 0, wideGroup = group * 2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = ELEM_TYPE_X2(e1);
          dest *= e2;
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
Hart<URV>::execVwmulu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));  // Spec says sign extend.

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwmulu_vx<uint8_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half: vwmulu_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word: vwmulu_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: vwmulu_vx<uint64_t>(vd, vs1, int64_t(e2), group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmul_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type  ELEM_TYPE_X2;

  ELEM_TYPE e1 = 0, e2 = 0;
  ELEM_TYPE_X2 dest = 0;
  unsigned errors = 0,  wideGroup = group * 2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = ELEM_TYPE_X2(e1);
          dest *= ELEM_TYPE_X2(e2);
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
Hart<URV>::execVwmul_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  // Double wide legal.
  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwmul_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vwmul_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vwmul_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmul_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmul_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type  ELEM_TYPE_X2;

  ELEM_TYPE e1 = 0;
  ELEM_TYPE_X2 dest = 0;
  ELEM_TYPE_X2 e2Wide(e2);
  unsigned errors = 0, wideGroup = group * 2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = ELEM_TYPE_X2(e1);
          dest *= e2Wide;
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
Hart<URV>::execVwmul_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  // Double wide legal.
  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwmul_vx<int8_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half: vwmul_vx<int16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word: vwmul_vx<int32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: vwmul_vx<int64_t>(vd, vs1, int64_t(e2), group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmulsu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type  ELEM_TYPE_X2;
  typedef typename std::make_unsigned<ELEM_TYPE>::type ELEM_TYPE_U;

  ELEM_TYPE e1 = 0;
  ELEM_TYPE_U e2u = 0;
  ELEM_TYPE_X2 dest = 0;
  unsigned errors = 0,  wideGroup = group * 2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2u))
        {
          dest = ELEM_TYPE_X2(e1);
          ELEM_TYPE_X2 tmp2(e2u);
          dest *= tmp2;
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
Hart<URV>::execVwmulsu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwmulsu_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vwmulsu_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vwmulsu_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmulsu_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmulsu_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type  ELEM_TYPE_X2;
  typedef typename std::make_unsigned<ELEM_TYPE>::type ELEM_TYPE_U;

  ELEM_TYPE e1 = 0;
  ELEM_TYPE_X2 dest = 0;
  ELEM_TYPE_U e2u = ELEM_TYPE_U(e2);
  ELEM_TYPE_X2 e2Wide(e2u);

  unsigned errors = 0,  wideGroup = group * 2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = ELEM_TYPE_X2(e1);
          dest *= e2Wide;
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
Hart<URV>::execVwmulsu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));   // Spec says sign extend.

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwmulsu_vx<int8_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half: vwmulsu_vx<int16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word: vwmulsu_vx<int32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: vwmulsu_vx<int64_t>(vd, vs1, int64_t(e2), group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type DWT; // Double wide type
  unsigned errors = 0, wideGroup = group*2;

  ELEM_TYPE e1 = 0, e2 = 0;
  DWT dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2)
          and vecRegs_.read(vd, ix, wideGroup, dest))
        {
          dest += DWT(e1) * DWT(e2);
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
Hart<URV>::execVwmaccu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwmacc_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vwmacc_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vwmacc_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmacc_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmaccu_vx(unsigned vd, ELEM_TYPE e1, unsigned vs2, unsigned group,
                      unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type DWT; // Double wide type
  typedef typename std::make_signed<DWT>::type SDWT; // Signed double wide type
  unsigned errors = 0, wideGroup = group*2;

  ELEM_TYPE e2 = 0;
  DWT dest = 0;
  SDWT sde1 = SDWT(e1);  // sign extend (per spec)
  DWT de1 = sde1;  // And make unsigned

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, wideGroup, dest))
        {
          dest += de1 * DWT(e2);
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
Hart<URV>::execVwmaccu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs2, vs2, group))
    return;

  SRV e1 = SRV(intRegs_.read(rs1));  // Spec says sign extend.

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwmaccu_vx<uint8_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Half: vwmaccu_vx<uint16_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word: vwmaccu_vx<uint32_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmaccu_vx<uint64_t>(vd, int64_t(e1), vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVwmacc_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  // Double wide legal.
  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwmacc_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vwmacc_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vwmacc_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmacc_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmacc_vx(unsigned vd, ELEM_TYPE e1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type DWT; // Double wide type
  unsigned errors = 0, wideGroup = group*2;

  ELEM_TYPE e2 = 0;
  DWT dest = 0;
  DWT de1 = DWT(e1);  // sign extend

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, wideGroup, dest))
        {
          dest += de1 * DWT(e2);
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
Hart<URV>::execVwmacc_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs2, vs2, group))
    return;

  SRV e1 = SRV(intRegs_.read(rs1));  // Sign extend.

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vwmacc_vx<int8_t> (vd, e1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwmacc_vx<int16_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwmacc_vx<int32_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmacc_vx<int64_t>(vd, int64_t(e1), vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmaccsu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                       unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type DWT; // Double wide type
  typedef typename std::make_unsigned<DWT>::type DWTU; // Double wide type unsigned
  typedef typename std::make_unsigned<ELEM_TYPE>::type SWTU; // Single wide type unsigned

  unsigned errors = 0, wideGroup = group*2;

  ELEM_TYPE e1 = 0, e2 = 0;
  DWT dest = 0, temp = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2)
          and vecRegs_.read(vd, ix, wideGroup, dest))
        {
          mulsu(DWT(e1), DWTU(SWTU(e2)), temp);
          dest += temp;
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
Hart<URV>::execVwmaccsu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwmaccsu_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vwmaccsu_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vwmaccsu_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmaccsu_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmaccsu_vx(unsigned vd, ELEM_TYPE e1, unsigned vs2, unsigned group,
                       unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type DWT; // Double wide type
  typedef typename std::make_unsigned<DWT>::type DWTU; // Double wide type unsigned
  typedef typename std::make_unsigned<ELEM_TYPE>::type SWTU; // Single wide type unsigned

  unsigned errors = 0, wideGroup = group*2;

  ELEM_TYPE e2 = 0;
  DWT de1 = DWT(e1);  // Sign extend.
  DWT dest = 0, temp = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, wideGroup, dest))
        {
          mulsu(de1, DWTU(SWTU(e2)), temp);
          dest += temp;
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
Hart<URV>::execVwmaccsu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs2, vs2, group))
    return;

  SRV e1 = SRV(intRegs_.read(rs1));  // Sign extend.

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwmaccsu_vx<int8_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Half: vwmaccsu_vx<int16_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word: vwmaccsu_vx<int32_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmaccsu_vx<int64_t>(vd, int64_t(e1), vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmaccus_vx(unsigned vd, ELEM_TYPE e1, unsigned vs2, unsigned group,
                       unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type DWT; // Double wide type
  typedef typename std::make_unsigned<DWT>::type DWTU; // Double wide type unsigned
  typedef typename std::make_unsigned<ELEM_TYPE>::type SWTU; // Single wide type unsigned

  unsigned errors = 0, wideGroup = group*2;

  ELEM_TYPE e2 = 0;
  DWT de1u = DWTU(SWTU(e1));  // Sign extend.
  DWT dest = 0, temp = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, wideGroup);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, wideGroup, dest))
        {
          mulsu(DWT(e2), de1u, temp);
          dest += temp;
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
Hart<URV>::execVwmaccus_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs2, vs2, group))
    return;

  URV e1 = intRegs_.read(rs1);

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vwmaccus_vx<int8_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Half: vwmaccus_vx<int16_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word: vwmaccus_vx<int32_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmaccus_vx<int64_t>(vd, e1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vdivu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = ~ ELEM_TYPE(0); // divide by zero result
          if (e2 != ELEM_TYPE(0))
            dest = e1 / e2;
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
Hart<URV>::execVdivu_vv(const DecodedInst* di)
{
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
    case EW::Byte: vdivu_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vdivu_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vdivu_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vdivu_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vdivu_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;

  // Spec (sep 24, 2020) says scalar register value should be sign
  // extended.
  ELEM_TYPE e1 = 0, e2 = intRegs_.read(rs2), dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = ~ ELEM_TYPE(0); // divide by zero result
          if (e2 != ELEM_TYPE(0))
            dest = e1 / e2;
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
Hart<URV>::execVdivu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vdivu_vx<uint8_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half:  vdivu_vx<uint16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vdivu_vx<uint32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vdivu_vx<uint64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vdiv_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  int elemBits = integerWidth<ELEM_TYPE> ();
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;
  ELEM_TYPE minInt = ELEM_TYPE(1) << (elemBits - 1);
  ELEM_TYPE negOne = ELEM_TYPE(-1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = negOne; // Divide by zero result
          if (e2 != ELEM_TYPE(0))
            {
              if (e1 == minInt and e2 == negOne)
                dest = e1;
              else
                dest = e1 / e2;
            }
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
Hart<URV>::execVdiv_vv(const DecodedInst* di)
{
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
    case EW::Byte: vdiv_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vdiv_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vdiv_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vdiv_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vdiv_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  int elemBits = integerWidth<ELEM_TYPE> ();
  ELEM_TYPE e1 = 0, e2 = SRV(intRegs_.read(rs2)), dest = 0;
  ELEM_TYPE minInt = ELEM_TYPE(1) << (elemBits - 1);
  ELEM_TYPE negOne = ELEM_TYPE(-1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = negOne; // Divide by zero result
          if (e2 != ELEM_TYPE(0))
            {
              if (e1 == minInt and e2 == negOne)
                dest = e1;
              else
                dest = e1 / e2;
            }
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
Hart<URV>::execVdiv_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vdiv_vx<int8_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half: vdiv_vx<int16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word: vdiv_vx<int32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vdiv_vx<int64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vremu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = e1; // divide by zero result
          if (e2 != ELEM_TYPE(0))
            dest = e1 % e2;
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
Hart<URV>::execVremu_vv(const DecodedInst* di)
{
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
    case EW::Byte: vremu_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vremu_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vremu_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vremu_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vremu_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;

  // Spec (sep 24, 2020) says scalar register value should be sign
  // extended.
  ELEM_TYPE e1 = 0, e2 = intRegs_.read(rs2), dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = e1; // divide by zero result
          if (e2 != ELEM_TYPE(0))
            dest = e1 % e2;
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
Hart<URV>::execVremu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  vremu_vx<uint8_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half:  vremu_vx<uint16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vremu_vx<uint32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vremu_vx<uint64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vrem_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  int elemBits = integerWidth<ELEM_TYPE> ();
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;
  ELEM_TYPE minInt = ELEM_TYPE(1) << (elemBits - 1);
  ELEM_TYPE negOne = ELEM_TYPE(-1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = e1; // Divide by zero remainder
          if (e2 != ELEM_TYPE(0))
            {
              if (e1 == minInt and e2 == negOne)
                dest = 0;
              else
                dest = e1 % e2;
            }
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
Hart<URV>::execVrem_vv(const DecodedInst* di)
{
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
    case EW::Byte: vrem_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half: vrem_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vrem_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vrem_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vrem_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0,  elemBits = integerWidth<ELEM_TYPE> ();
  ELEM_TYPE e1 = 0, e2 = SRV(intRegs_.read(rs2)), dest = 0;
  ELEM_TYPE minInt = ELEM_TYPE(1) << (elemBits - 1);
  ELEM_TYPE negOne = ELEM_TYPE(-1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = e1; // Divide by zero remainder
          if (e2 != ELEM_TYPE(0))
            {
              if (e1 == minInt and e2 == negOne)
                dest = 0;
              else
                dest = e1 % e2;
            }
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
Hart<URV>::execVrem_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vrem_vx<int8_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half: vrem_vx<int16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word: vrem_vx<int32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vrem_vx<int64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE, typename FROM_TYPE>
void
Hart<URV>::vsext(unsigned vd, unsigned vs1, unsigned group, unsigned fromGroup,
                 unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  FROM_TYPE e1 = 0;
  ELEM_TYPE dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, fromGroup, e1))
        {
          dest = e1;
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
Hart<URV>::execVsext_vf2(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned fromGroup = group/2;
  if (fromGroup == 0)
    {
      postVecFail(di);
      return;
    }
  GroupMultiplier emul = GroupMultiplier::One;
  if (not vecRegs_.groupNumberX8ToSymbol(fromGroup, emul))
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned eg = group >= 8 ? group / 8 : 1;
  if (vd % eg)
    {
      postVecFail(di);
      return;
    }
  if (eg > 2 and (vs1 % (eg/2)))
    {
      postVecFail(di);
      return;
    }
  unsigned dw = vecRegs_.elemWidthInBits();  // destination width
  unsigned sw = dw/2; // source width.
  if (not checkDestSourceOverlap(vd, dw, group, vs1, sw, fromGroup))
    {
      postVecFail(di);
      return;
    }

  vecRegs_.setOpEmul(eg, eg > 2? eg/2 : 1);  // Track operand group for logging

  typedef ElementWidth EW;

  EW sew = vecRegs_.elemWidth();
  EW eew = sew;  // Effective elem width of source.
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: eew = EW::Byte; break;
    case EW::Word: eew = EW::Half; break;
    case EW::Word2: eew = EW::Word; break;
    case EW::Word4: eew = EW::Word2; break;
    case EW::Word8: eew = EW::Word4; break;
    case EW::Word16: eew = EW::Word8; break;
    case EW::Word32: eew = EW::Word16; break;
    }
  if (not vecRegs_.legalConfig(eew, emul))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: vsext<int16_t,int8_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Word: vsext<int32_t, int16_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Word2: vsext<int64_t, int32_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsext_vf4(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned fromGroup = group/4;
  if (fromGroup == 0)
    {
      postVecFail(di);
      return;
    }
  GroupMultiplier emul = GroupMultiplier::One;
  if (not vecRegs_.groupNumberX8ToSymbol(fromGroup, emul))
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned eg = group >= 8 ? group / 8 : 1;
  if (vd % eg)
    {
      postVecFail(di);
      return;
    }
  if (eg > 4 and (vs1 % (eg/4)))
    {
      postVecFail(di);
      return;
    }
  unsigned dw = vecRegs_.elemWidthInBits();  // destination width
  unsigned sw = dw/4; // source width.
  if (not checkDestSourceOverlap(vd, dw, group, vs1, sw, fromGroup))
    {
      postVecFail(di);
      return;
    }

  vecRegs_.setOpEmul(eg, eg > 4? eg/4 : 1);  // Track operand group for logging

  typedef ElementWidth EW;

  EW sew = vecRegs_.elemWidth();
  EW eew = sew;  // Effective elem width of source.
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: postVecFail(di); return;
    case EW::Word: eew = EW::Byte; break;
    case EW::Word2: eew = EW::Half; break;
    case EW::Word4: eew = EW::Word; break;
    case EW::Word8: eew = EW::Word2; break;
    case EW::Word16: eew = EW::Word4; break;
    case EW::Word32: eew = EW::Word8; break;
    }
  if (not vecRegs_.legalConfig(eew, emul))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: postVecFail(di); return;
    case EW::Word: vsext<int32_t, int8_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Word2: vsext<int64_t, int16_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsext_vf8(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned fromGroup = group/8;
  if (fromGroup == 0)
    {
      postVecFail(di);
      return;
    }
  GroupMultiplier emul = GroupMultiplier::One;
  if (not vecRegs_.groupNumberX8ToSymbol(fromGroup, emul))
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned eg = group >= 8 ? group / 8 : 1;
  if (vd % eg)
    {
      postVecFail(di);
      return;
    }
  if (eg > 8 and (vs1 % (eg/8)))
    {
      postVecFail(di);
      return;
    }
  unsigned dw = vecRegs_.elemWidthInBits();  // destination width
  unsigned sw = dw/8; // source width.
  if (not checkDestSourceOverlap(vd, dw, group, vs1, sw, fromGroup))
    {
      postVecFail(di);
      return;
    }

  vecRegs_.setOpEmul(eg, eg > 8? eg/8 : 1);  // Track operand group for logging

  typedef ElementWidth EW;

  EW sew = vecRegs_.elemWidth();
  EW eew = sew;  // Effective elem width of source.
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: postVecFail(di); return;
    case EW::Word: postVecFail(di); return;
    case EW::Word2: eew = EW::Byte; break;
    case EW::Word4: eew = EW::Half; break;
    case EW::Word8: eew = EW::Word; break;
    case EW::Word16: eew = EW::Word2; break;
    case EW::Word32: eew = EW::Word4; break;
    }
  if (not vecRegs_.legalConfig(eew, emul))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: postVecFail(di); return;
    case EW::Word: postVecFail(di); return;
    case EW::Word2: vsext<int64_t, int8_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE, typename FROM_TYPE>
void
Hart<URV>::vzext(unsigned vd, unsigned vs1, unsigned group, unsigned fromGroup,
                 unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  FROM_TYPE e1 = 0;
  ELEM_TYPE dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, fromGroup, e1))
        {
          dest = e1;
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
Hart<URV>::execVzext_vf2(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned fromGroup = group/2;
  if (fromGroup == 0)
    {
      postVecFail(di);
      return;
    }
  GroupMultiplier emul = GroupMultiplier::One;
  if (not vecRegs_.groupNumberX8ToSymbol(fromGroup, emul))
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned eg = group >= 8 ? group / 8 : 1;
  if (vd % eg)
    {
      postVecFail(di);
      return;
    }
  if (eg > 2 and (vs1 % (eg/2)))
    {
      postVecFail(di);
      return;
    }
  unsigned dw = vecRegs_.elemWidthInBits();  // destination width
  unsigned sw = dw/2; // source width.
  if (not checkDestSourceOverlap(vd, dw, group, vs1, sw, fromGroup))
    {
      postVecFail(di);
      return;
    }

  vecRegs_.setOpEmul(eg, eg > 2? eg/2 : 1);  // Track operand group for logging

  typedef ElementWidth EW;

  EW sew = vecRegs_.elemWidth();
  EW eew = sew;  // Effective elem width of source.
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: eew = EW::Byte; break;
    case EW::Word: eew = EW::Half; break;
    case EW::Word2: eew = EW::Word; break;
    case EW::Word4: eew = EW::Word2; break;
    case EW::Word8: eew = EW::Word4; break;
    case EW::Word16: eew = EW::Word8; break;
    case EW::Word32: eew = EW::Word16; break;
    }
  if (not vecRegs_.legalConfig(eew, emul))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: vzext<uint16_t,uint8_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Word: vzext<uint32_t, uint16_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Word2: vzext<uint64_t, uint32_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVzext_vf4(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned fromGroup = group/4;
  if (fromGroup == 0)
    {
      postVecFail(di);
      return;
    }
  GroupMultiplier emul = GroupMultiplier::One;
  if (not vecRegs_.groupNumberX8ToSymbol(fromGroup, emul))
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned eg = group >= 8 ? group / 8 : 1;
  if (vd % eg)
    {
      postVecFail(di);
      return;
    }
  if (eg > 4 and (vs1 % (eg/4)))
    {
      postVecFail(di);
      return;
    }
  unsigned dw = vecRegs_.elemWidthInBits();  // destination width
  unsigned sw = dw/4; // source width.
  if (not checkDestSourceOverlap(vd, dw, group, vs1, sw, fromGroup))
    {
      postVecFail(di);
      return;
    }

  vecRegs_.setOpEmul(eg, eg > 4? eg/4 : 1);  // Track operand group for logging

  typedef ElementWidth EW;

  EW sew = vecRegs_.elemWidth();
  EW eew = sew;  // Effective elem width of source.
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: postVecFail(di); return;
    case EW::Word: eew = EW::Byte; break;
    case EW::Word2: eew = EW::Half; break;
    case EW::Word4: eew = EW::Word; break;
    case EW::Word8: eew = EW::Word2; break;
    case EW::Word16: eew = EW::Word4; break;
    case EW::Word32: eew = EW::Word8; break;
    }
  if (not vecRegs_.legalConfig(eew, emul))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: postVecFail(di); return;
    case EW::Word: vzext<uint32_t, uint8_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Word2: vzext<uint64_t, uint16_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVzext_vf8(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned fromGroup = group/8;
  if (fromGroup == 0)
    {
      postVecFail(di);
      return;
    }
  GroupMultiplier emul = GroupMultiplier::One;
  if (not vecRegs_.groupNumberX8ToSymbol(fromGroup, emul))
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned eg = group >= 8 ? group / 8 : 1;
  if (vd % eg)
    {
      postVecFail(di);
      return;
    }
  if (eg > 8 and (vs1 % (eg/8)))
    {
      postVecFail(di);
      return;
    }
  unsigned dw = vecRegs_.elemWidthInBits();  // destination width
  unsigned sw = dw/8; // source width.
  if (not checkDestSourceOverlap(vd, dw, group, vs1, sw, fromGroup))
    {
      postVecFail(di);
      return;
    }

  vecRegs_.setOpEmul(eg, eg > 8? eg/8 : 1);  // Track operand group for logging

  typedef ElementWidth EW;

  EW sew = vecRegs_.elemWidth();
  EW eew = sew;  // Effective elem width of source.
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: postVecFail(di); return;
    case EW::Word: postVecFail(di); return;
    case EW::Word2: eew = EW::Byte; break;
    case EW::Word4: eew = EW::Half; break;
    case EW::Word8: eew = EW::Word; break;
    case EW::Word16: eew = EW::Word2; break;
    case EW::Word32: eew = EW::Word4; break;
    }
  if (not vecRegs_.legalConfig(eew, emul))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: postVecFail(di); return;
    case EW::Word: postVecFail(di); return;
    case EW::Word2: vzext<uint64_t, uint8_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vadc_vvm(unsigned vd, unsigned vs1, unsigned vs2, unsigned vcin,
                    unsigned group, unsigned start, unsigned elems)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = e1 + e2;
          if (vecRegs_.isActive(vcin, ix))
            dest += ELEM_TYPE(1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vadc_vxm(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned vcin,
                    unsigned group, unsigned start, unsigned elems)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = e1 + e2;
          if (vecRegs_.isActive(vcin, ix))
            dest += ELEM_TYPE(1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsbc_vvm(unsigned vd, unsigned vs1, unsigned vs2, unsigned vbin,
                    unsigned group, unsigned start, unsigned elems)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = e1 - e2;
          if (vecRegs_.isActive(vbin, ix))
            dest -= ELEM_TYPE(1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsbc_vxm(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned vbin,
                    unsigned group, unsigned start, unsigned elems)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = e1 - e2;
          if (vecRegs_.isActive(vbin, ix))
            dest -= ELEM_TYPE(1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmadc_vvm(unsigned vcout, unsigned vs1, unsigned vs2, bool carry, unsigned vcin,
                     unsigned group, unsigned start, unsigned elems)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = e1 + e2;
	  bool cin = carry and vecRegs_.isActive(vcin, ix);
	  if (cin)
            dest += ELEM_TYPE(1);

          bool cout = cin? dest <= e1 : dest < e1;
          if (not vecRegs_.writeMaskRegister(vcout, ix, cout))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmadc_vxm(unsigned vcout, unsigned vs1, ELEM_TYPE e2, bool carry, unsigned vcin,
                     unsigned group, unsigned start, unsigned elems)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = e1 + e2;
          bool cin = carry and vecRegs_.isActive(vcin, ix);
	  if (cin)
	    dest += ELEM_TYPE(1);

          bool cout = cin? dest <= e1 : dest < e1;
          if (not vecRegs_.writeMaskRegister(vcout, ix, cout))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmsbc_vvm(unsigned vbout, unsigned vs1, unsigned vs2, bool borrow, unsigned vbin,
                     unsigned group, unsigned start, unsigned elems)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = e1 - e2;
          bool bout = e1 < e2;

          if (borrow and vecRegs_.isActive(vbin, ix))
	    {
	      dest -= ELEM_TYPE(1);
	      bout = e1 <= e2;
	    }

          if (not vecRegs_.writeMaskRegister(vbout, ix, bout))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmsbc_vxm(unsigned vbout, unsigned vs1, ELEM_TYPE e2, bool borrow, unsigned vbin,
                     unsigned group, unsigned start, unsigned elems)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = e1 - e2;
          bool bout = e1 < e2;

          if (borrow and vecRegs_.isActive(vbin, ix))
	    {
	      dest -= ELEM_TYPE(1);
	      bout = e1 <= e2;
	    }

          if (not vecRegs_.writeMaskRegister(vbout, ix, bout))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVadc_vvm(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2(),  vcin = 0;
  if (vd == vcin or not masked)   // cannot overlap vcin, unmasked verion reserved
    {
      postVecFail(di);
      return;
    }

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vadc_vvm<uint8_t>(vd, vs1, vs2, vcin, group, start, elems); break;
    case EW::Half: vadc_vvm<uint16_t>(vd, vs1, vs2, vcin, group, start, elems); break;
    case EW::Word: vadc_vvm<uint32_t>(vd, vs1, vs2, vcin, group, start, elems); break;
    case EW::Word2: vadc_vvm<uint64_t>(vd, vs1, vs2, vcin, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVadc_vxm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vcin = 0;
  if (vd == vcin or not masked)   // cannot overlap vcin, unmasked verion reserved
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(di->op2()));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vadc_vxm<uint8_t>(vd, vs1, e2, vcin, group, start, elems); break;
    case EW::Half: vadc_vxm<uint16_t>(vd, vs1, e2, vcin, group, start, elems); break;
    case EW::Word: vadc_vxm<uint32_t>(vd, vs1, int32_t(e2), vcin, group, start, elems); break;
    case EW::Word2: vadc_vxm<uint64_t>(vd, vs1, int64_t(e2), vcin, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVadc_vim(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vcin = 0;
  if (vd == vcin or not masked)  // cannot overlap vcin, unmasked verion reserved
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  SRV e2 = di->op2As<int32_t>();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vadc_vxm<uint8_t>(vd, vs1, e2, vcin, group, start, elems); break;
    case EW::Half: vadc_vxm<uint16_t>(vd, vs1, e2, vcin, group, start, elems); break;
    case EW::Word: vadc_vxm<uint32_t>(vd, vs1, int32_t(e2), vcin, group, start, elems); break;
    case EW::Word2: vadc_vxm<uint64_t>(vd, vs1, int64_t(e2), vcin, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsbc_vvm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2(),  vbin = 0;
  if (vd == vbin or not masked) // cannot overlap borrow-in, unmasked verion reserved
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vsbc_vvm<uint8_t>(vd, vs1, vs2, vbin, group, start, elems); break;
    case EW::Half: vsbc_vvm<uint16_t>(vd, vs1, vs2, vbin, group, start, elems); break;
    case EW::Word: vsbc_vvm<uint32_t>(vd, vs1, vs2, vbin, group, start, elems); break;
    case EW::Word2: vsbc_vvm<uint64_t>(vd, vs1, vs2, vbin, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsbc_vxm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vbin = 0;
  if (vd == vbin or not masked) // cannot overlap borrow-in, unmasked verion reserved
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(di->op2()));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vsbc_vxm<uint8_t>(vd, vs1, e2, vbin, group, start, elems); break;
    case EW::Half: vsbc_vxm<uint16_t>(vd, vs1, e2, vbin, group, start, elems); break;
    case EW::Word: vsbc_vxm<uint32_t>(vd, vs1, int32_t(e2), vbin, group, start, elems); break;
    case EW::Word2: vsbc_vxm<uint64_t>(vd, vs1, int64_t(e2), vbin, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmadc_vvm(const DecodedInst* di)
{
  bool carry = di->isMasked();
  unsigned vcout = di->op0(),  vs1 = di->op1(),  vs2 = di->op2(),  vcin = 0;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vcout, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vmadc_vvm<uint8_t>(vcout, vs1, vs2, carry, vcin, group, start, elems); break;
    case EW::Half: vmadc_vvm<uint16_t>(vcout, vs1, vs2, carry, vcin, group, start, elems); break;
    case EW::Word: vmadc_vvm<uint32_t>(vcout, vs1, vs2, carry, vcin, group, start, elems); break;
    case EW::Word2: vmadc_vvm<uint64_t>(vcout, vs1, vs2, carry, vcin, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmadc_vxm(const DecodedInst* di)
{
  bool carry = di->isMasked();
  unsigned vcout = di->op0(),  vs1 = di->op1(),  vcin = 0;
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vcout, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(di->op2()));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vmadc_vxm<uint8_t>(vcout, vs1, e2, carry, vcin, group, start, elems); break;
    case EW::Half: vmadc_vxm<uint16_t>(vcout, vs1, e2, carry, vcin, group, start, elems); break;
    case EW::Word: vmadc_vxm<uint32_t>(vcout, vs1, int32_t(e2), carry, vcin, group, start, elems); break;
    case EW::Word2: vmadc_vxm<uint64_t>(vcout, vs1, int64_t(e2), carry, vcin, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmadc_vim(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool carry = di->isMasked();
  unsigned vcout = di->op0(),  vs1 = di->op1(),  vcin = 0;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vcout, vs1, group))
    return;

  SRV e2 = di->op2As<int32_t>();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vmadc_vxm<uint8_t>(vcout, vs1, e2, carry, vcin, group, start, elems); break;
    case EW::Half: vmadc_vxm<uint16_t>(vcout, vs1, e2, carry, vcin, group, start, elems); break;
    case EW::Word: vmadc_vxm<uint32_t>(vcout, vs1, int32_t(e2), carry, vcin, group, start, elems); break;
    case EW::Word2: vmadc_vxm<uint64_t>(vcout, vs1, int64_t(e2), carry, vcin, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsbc_vvm(const DecodedInst* di)
{
  bool borrow = di->isMasked();
  unsigned vbout = di->op0(),  vs1 = di->op1(),  vs2 = di->op2(),  vbin = 0;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vbout, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vmsbc_vvm<uint8_t>(vbout, vs1, vs2, borrow, vbin, group, start, elems); break;
    case EW::Half: vmsbc_vvm<uint16_t>(vbout, vs1, vs2, borrow, vbin, group, start, elems); break;
    case EW::Word: vmsbc_vvm<uint32_t>(vbout, vs1, vs2, borrow, vbin, group, start, elems); break;
    case EW::Word2: vmsbc_vvm<uint64_t>(vbout, vs1, vs2, borrow, vbin, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmsbc_vxm(const DecodedInst* di)
{
  bool borrow = di->isMasked();
  unsigned vbout = di->op0(),  vs1 = di->op1(),  vbin = 0;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vbout, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(di->op2()));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vmsbc_vxm<uint8_t>(vbout, vs1, e2, borrow, vbin, group, start, elems); break;
    case EW::Half: vmsbc_vxm<uint16_t>(vbout, vs1, e2, borrow, vbin, group, start, elems); break;
    case EW::Word: vmsbc_vxm<uint32_t>(vbout, vs1, int32_t(e2), borrow, vbin, group, start, elems); break;
    case EW::Word2: vmsbc_vxm<uint64_t>(vbout, vs1, int64_t(e2), borrow, vbin, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmerge_vvm(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems)
{
  unsigned errors = 0;

  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = vecRegs_.isActive(0, ix) ? e2 : e1;
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
Hart<URV>::execVmerge_vvm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2(),  start = csRegs_.peekVstart();;
  if (not di->isMasked() or di->op0() == 0) // Must be masked, dest must not overlap v0.
    {
      postVecFail(di);
      return;
    }

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vmerge_vvm<int8_t>(vd, vs1, vs2, group, start, elems); break;
    case EW::Half: vmerge_vvm<int16_t>(vd, vs1, vs2, group, start, elems); break;
    case EW::Word: vmerge_vvm<int32_t>(vd, vs1, vs2, group, start, elems); break;
    case EW::Word2: vmerge_vvm<int64_t>(vd, vs1, vs2, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmerge_vxm(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
		      unsigned start, unsigned elems)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = vecRegs_.isActive(0, ix) ? e2 : e1;
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
Hart<URV>::execVmerge_vxm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2(), start = csRegs_.peekVstart();
  if (not di->isMasked() or di->op0() == 0 or // Must be masked, dest must not overlap v0.
      not di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vmerge_vxm<int8_t>(vd, vs1, e2, group, start, elems); break;
    case EW::Half: vmerge_vxm<int16_t>(vd, vs1, e2, group, start, elems); break;
    case EW::Word: vmerge_vxm<int32_t>(vd, vs1, e2, group, start, elems); break;
    case EW::Word2: vmerge_vxm<int64_t>(vd, vs1, e2, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmerge_vim(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(), vs1 = di->op1(), start = csRegs_.peekVstart();
  int32_t imm = di->op2As<int32_t>();
  if (not di->isMasked() or di->op0() == 0) // Must be masked. Dest must not overlap v0.
    {
      postVecFail(di);
      return;
    }

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vmerge_vxm<int8_t>(vd, vs1, imm, group, start, elems); break;
    case EW::Half: vmerge_vxm<int16_t>(vd, vs1, imm, group, start, elems); break;
    case EW::Word: vmerge_vxm<int32_t>(vd, vs1, imm, group, start, elems); break;
    case EW::Word2: vmerge_vxm<int64_t>(vd, vs1, imm, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmv_x_s(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned rd = di->op0(), vs1 = di->op1(), groupX8 = 8;

  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  if ((vs1 % eg) or di->isMasked())
    {
      postVecFail(di);
      return;
    }
  vecRegs_.setOpEmul(1, eg);  // Track operand group for logging

  ElementWidth sew = vecRegs_.elemWidth();

  switch (sew)
    {
    case ElementWidth::Byte:
      {
        int8_t val = 0;
        vecRegs_.read(vs1, 0, groupX8, val);
        intRegs_.write(rd, SRV(val));
      }
      break;

    case ElementWidth::Half:
      {
        int16_t val = 0;
        vecRegs_.read(vs1, 0, groupX8, val);
        intRegs_.write(rd, SRV(val));
      }
      break;

    case ElementWidth::Word:
      {
        int32_t val = 0;
        vecRegs_.read(vs1, 0, groupX8, val);
        intRegs_.write(rd, SRV(val));
      }
      break;

    case ElementWidth::Word2:
      {
        int64_t val = 0;
        vecRegs_.read(vs1, 0, groupX8, val);
        intRegs_.write(rd, SRV(val));
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
Hart<URV>::execVmv_s_x(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(), rs1 = di->op1(), groupX8 = 8, start = csRegs_.peekVstart();
  ElementWidth sew = vecRegs_.elemWidth();
  SRV val = intRegs_.read(rs1);

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      if (start < vecRegs_.elemCount())
	vecRegs_.write(vd, 0, groupX8, int8_t(val));
      break;
    case EW::Half:
      if (start < vecRegs_.elemCount())
	vecRegs_.write(vd, 0, groupX8, int16_t(val));
      break;
    case EW::Word:
      if (start < vecRegs_.elemCount())
	vecRegs_.write(vd, 0, groupX8, int32_t(val));
      break;
    case EW::Word2:
      if (start < vecRegs_.elemCount())
	vecRegs_.write(vd, 0, groupX8, int64_t(val));
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfmv_f_s(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned rd = di->op0(), vs1 = di->op1(), groupX8 = 8;

  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  if (vs1 % eg)
    {
      postVecFail(di);
      return;
    }
  vecRegs_.setOpEmul(1, eg);  // Track operand group for logging

  ElementWidth sew = vecRegs_.elemWidth();

  switch (sew)
    {
    case ElementWidth::Byte:
      postVecFail(di);
      return;

    case ElementWidth::Half:
      if (not isZvfhLegal())
	postVecFail(di);
      else
	{
	  Float16 val{};
	  vecRegs_.read(vs1, 0, groupX8, val);
	  fpRegs_.writeHalf(rd, val);
	}
      break;

    case ElementWidth::Word:
      if (not isFpLegal())
	postVecFail(di);
      else
	{
	  float val{};
	  vecRegs_.read(vs1, 0, groupX8, val);
	  fpRegs_.writeSingle(rd, val);
	}
      break;

    case ElementWidth::Word2:
      if (not isDpLegal())
	postVecFail(di);
      else
	{
	  double val{};
	  vecRegs_.read(vs1, 0, groupX8, val);
	  fpRegs_.writeDouble(rd, val);
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
Hart<URV>::execVfmv_s_f(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(), rs1 = di->op1(), groupX8 = 8, start = csRegs_.peekVstart();
  ElementWidth sew = vecRegs_.elemWidth();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      postVecFail(di);
      return;
    case EW::Half:
      if (not isZvfhLegal())
	postVecFail(di);
      else if (start < vecRegs_.elemCount())
	{
	  Float16 val = fpRegs_.readHalf(rs1);
	  vecRegs_.write(vd, 0, groupX8, val);
	}
      break;
    case EW::Word:
      if (not isFpLegal())
	postVecFail(di);
      else if (start < vecRegs_.elemCount())
	{
	  float val = fpRegs_.readSingle(rs1);
	  vecRegs_.write(vd, 0, groupX8, val);
	}
      break;
    case EW::Word2:
      if (not isDpLegal())
	postVecFail(di);
      else if (start < vecRegs_.elemCount())
	{
	  double val = fpRegs_.readDouble(rs1);
	  vecRegs_.write(vd, 0, groupX8, val);
	}
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmv_v_v(unsigned vd, unsigned vs1, unsigned group,
                   unsigned start, unsigned elems)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = e1;
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
Hart<URV>::execVmv_v_v(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  unsigned vd = di->op0(),  vs1 = di->op1();

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vmv_v_v<int8_t>(vd, vs1, group, start, elems); break;
    case EW::Half: vmv_v_v<int16_t>(vd, vs1, group, start, elems); break;
    case EW::Word: vmv_v_v<int32_t>(vd, vs1, group, start, elems); break;
    case EW::Word2: vmv_v_v<int64_t>(vd, vs1, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmv_v_x(unsigned vd, ELEM_TYPE e1, unsigned group,
                   unsigned start, unsigned elems)
{
  unsigned errors = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    if (not vecRegs_.write(vd, ix, group, e1))
      errors++;

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVmv_v_x(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  unsigned vd = di->op0();
  unsigned rs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, group))
    return;

  SRV e1 = intRegs_.read(rs1);

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vmv_v_x<int8_t>(vd, e1, group, start, elems); break;
    case EW::Half: vmv_v_x<int16_t>(vd, e1, group, start, elems); break;
    case EW::Word: vmv_v_x<int32_t>(vd, e1, group, start, elems); break;
    case EW::Word2: vmv_v_x<int64_t>(vd, e1, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmv_v_i(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  unsigned vd = di->op0();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, group))
    return;

  int32_t e1 = di->op1As<int32_t>();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: vmv_v_x<int8_t>(vd, e1, group, start, elems); break;
    case EW::Half: vmv_v_x<int16_t>(vd, e1, group, start, elems); break;
    case EW::Word: vmv_v_x<int32_t>(vd, e1, group, start, elems); break;
    case EW::Word2: vmv_v_x<int64_t>(vd, e1, group, start, elems); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmv1r_v(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(), vs1 = di->op1();
  if (vd == vs1)
    return;

  unsigned bytes = vecRegs_.bytesPerRegister();

  uint8_t* dest = vecRegs_.getVecData(vd);
  uint8_t* source = vecRegs_.getVecData(vs1);
  assert(dest);
  assert(source);

  memcpy(dest, source, bytes);

  vecRegs_.touchReg(vd, 1*8);  // Grouping of 1.
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmv2r_v(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(), vs1 = di->op1();
  if ((vd & 1) != 0 or (vs1 & 1) != 0)
    {
      postVecFail(di);   // Vec indices must be even
      return;
    }

  if (vd == vs1)
    return;

  unsigned bytes = vecRegs_.bytesPerRegister() * 2;

  uint8_t* dest = vecRegs_.getVecData(vd);
  uint8_t* source = vecRegs_.getVecData(vs1);
  assert(dest);
  assert(source);

  memcpy(dest, source, bytes);

  vecRegs_.touchReg(vd, 2*8);    // Grouping of 2
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmv4r_v(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(), vs1 = di->op1();
  if ((vd & 3) != 0 or (vs1 & 3) != 0)
    {
      postVecFail(di);   // Vec indices must be multiples of 4
      return;
    }

  if (vd == vs1)
    return;

  unsigned bytes = vecRegs_.bytesPerRegister() * 4;

  uint8_t* dest = vecRegs_.getVecData(vd);
  uint8_t* source = vecRegs_.getVecData(vs1);
  assert(dest);
  assert(source);

  memcpy(dest, source, bytes);

  vecRegs_.touchReg(vd, 4*8);  // Grouping of 4.
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVmv8r_v(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(), vs1 = di->op1();
  if ((vd & 7) != 0 or (vs1 & 7) != 0)
    {
      postVecFail(di);   // Vec indices must be multiples of 8
      return;
    }

  if (vd == vs1)
    return;

  unsigned bytes = vecRegs_.bytesPerRegister()*8;

  uint8_t* dest = vecRegs_.getVecData(vd);
  uint8_t* source = vecRegs_.getVecData(vs1);
  assert(dest);
  assert(source);

  memcpy(dest, source, bytes);

  vecRegs_.touchReg(vd, 8*8);  // Grouping of 8.
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsaddu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  ELEM_TYPE maxVal = ~ ELEM_TYPE(0);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = e1 + e2;
          if (dest < e1)
            {
              dest = maxVal;
              csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
            }
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
Hart<URV>::execVsaddu_vv(const DecodedInst* di)
{
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
    case EW::Byte:   vsaddu_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vsaddu_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vsaddu_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vsaddu_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsaddu_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;

  ELEM_TYPE maxVal = ~ ELEM_TYPE(0);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = e1 + e2;
          if (dest < e1)
            {
              dest = maxVal;
              csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
            }
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
Hart<URV>::execVsaddu_vx(const DecodedInst* di)
{
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
    case EW::Byte:   vsaddu_vx<uint8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vsaddu_vx<uint16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vsaddu_vx<uint32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vsaddu_vx<uint64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsaddu_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(), vs1 = di->op1();
  int32_t imm = di->op2As<int32_t>();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   vsaddu_vx<uint8_t> (vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Half:   vsaddu_vx<uint16_t>(vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Word:   vsaddu_vx<uint32_t>(vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Word2:  vsaddu_vx<uint64_t>(vd, vs1, imm,          group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  uint32_t bitCount = sizeof(ELEM_TYPE)*8;
  ELEM_TYPE minVal = ELEM_TYPE(1) << (bitCount - 1);
  ELEM_TYPE maxVal = minVal - 1;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = e1 + e2;
          bool sameSign = (e1 < 0) == (e2 < 0);
          if (sameSign and ((e1 < 0) != (dest < 0)))
            {
              if (e1 < 0)
                dest = minVal;
              else
                dest = maxVal;
              csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
            }
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
Hart<URV>::execVsadd_vv(const DecodedInst* di)
{
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
    case EW::Byte:   vsadd_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vsadd_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vsadd_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vsadd_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsadd_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;

  uint32_t bitCount = sizeof(ELEM_TYPE)*8;
  ELEM_TYPE minVal = ELEM_TYPE(1) << (bitCount - 1);
  ELEM_TYPE maxVal = minVal - 1;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = e1 + e2;
          bool sameSign = (e1 < 0) == (e2 < 0);
          if (sameSign and ((e1 < 0) != (dest < 0)))
            {
              if (e1 < 0)
                dest = minVal;
              else
                dest = maxVal;
              csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
            }
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
Hart<URV>::execVsadd_vx(const DecodedInst* di)
{
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
    case EW::Byte:   vsadd_vx<int8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vsadd_vx<int16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vsadd_vx<int32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vsadd_vx<int64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsadd_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(), vs1 = di->op1();
  int32_t imm = di->op2As<int32_t>();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   vsadd_vx<int8_t> (vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Half:   vsadd_vx<int16_t>(vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Word:   vsadd_vx<int32_t>(vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Word2:  vsadd_vx<int64_t>(vd, vs1, imm,          group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vssubu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  ELEM_TYPE minVal = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = e1 - e2;
          if (dest > e1)
            {
              dest = minVal;
              csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
            }
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
Hart<URV>::execVssubu_vv(const DecodedInst* di)
{
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
    case EW::Byte:   vssubu_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vssubu_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vssubu_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vssubu_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vssubu_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;

  ELEM_TYPE minVal = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = e1 - e2;
          if (dest > e1)
            {
              dest = minVal;
              csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
            }
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
Hart<URV>::execVssubu_vx(const DecodedInst* di)
{
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
    case EW::Byte:   vssubu_vx<uint8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vssubu_vx<uint16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vssubu_vx<uint32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vssubu_vx<uint64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vssub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  uint32_t bitCount = sizeof(ELEM_TYPE)*8;
  ELEM_TYPE minVal = ELEM_TYPE(1) << (bitCount - 1);
  ELEM_TYPE maxVal = minVal - 1;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = e1 - e2;
          bool sameSign = (e1 < 0) == (e2 >= 0);
          if (sameSign and ((e1 < 0) != (dest < 0)))
            {
              if (e1 < 0)
                dest = minVal;
              else
                dest = maxVal;
              csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
            }
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
Hart<URV>::execVssub_vv(const DecodedInst* di)
{
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
    case EW::Byte:   vssub_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vssub_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vssub_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vssub_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vssub_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, dest = 0;

  uint32_t bitCount = sizeof(ELEM_TYPE)*8;
  ELEM_TYPE minVal = ELEM_TYPE(1) << (bitCount - 1);
  ELEM_TYPE maxVal = minVal - 1;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = e1 - e2;
          bool sameSign = (e1 < 0) == (e2 >= 0);
          if (sameSign and ((e1 < 0) != (dest < 0)))
            {
              if (e1 < 0)
                dest = minVal;
              else
                dest = maxVal;
              csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
            }
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
Hart<URV>::execVssub_vx(const DecodedInst* di)
{
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
    case EW::Byte:   vssub_vx<int8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vssub_vx<int16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vssub_vx<int32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vssub_vx<int64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename T>
static
void
roundoff(VecRoundingMode mode, T& value, unsigned d)
{
  if (d == 0)
    return;

  unsigned bit = 0;

  unsigned vd = unsigned((value >> d) & 1);
  unsigned vd_1 = unsigned((value >> (d-1)) & 1);

  switch (mode)
    {
    case VecRoundingMode::NearestUp:
      bit = vd_1;
      break;

    case VecRoundingMode::NearestEven:
      bit = vd_1 & ( ((((T(1) << (d-1)) - 1) & value) != 0)  |  vd );
      break;

    case VecRoundingMode::Down:
      break;

    case VecRoundingMode::Odd:
      bit = (~vd & 1)  & ( (((T(1) << d) - 1) & value) != 0 );
      break;

    default:
      break;
    }


  T extra = bit;
  value = (value >> d) + extra;
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vaadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0;

  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2; // Double wide

  URV rmVal = 0;
  peekCsr(CsrNumber::VXRM, rmVal);
  VecRoundingMode rm = VecRoundingMode(rmVal);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          ELEM_TYPE2 temp = e1;
          temp += e2;
          roundoff(rm, temp, 1);
          ELEM_TYPE dest = ELEM_TYPE(temp);
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
Hart<URV>::execVaadd_vv(const DecodedInst* di)
{
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
    case EW::Byte:   vaadd_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vaadd_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vaadd_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vaadd_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVaaddu_vv(const DecodedInst* di)
{
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
    case EW::Byte:   vaadd_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vaadd_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vaadd_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vaadd_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vaadd_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0;

  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2; // Double wide

  URV rmVal = 0;
  peekCsr(CsrNumber::VXRM, rmVal);
  VecRoundingMode rm = VecRoundingMode(rmVal);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          ELEM_TYPE2 temp = e1;
          temp += e2;
          roundoff(rm, temp, 1);
          ELEM_TYPE dest = ELEM_TYPE(temp);
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
Hart<URV>::execVaadd_vx(const DecodedInst* di)
{
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
    case EW::Byte:   vaadd_vx<int8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vaadd_vx<int16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vaadd_vx<int32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vaadd_vx<int64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVaaddu_vx(const DecodedInst* di)
{
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
    case EW::Byte:   vaadd_vx<uint8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vaadd_vx<uint16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vaadd_vx<uint32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vaadd_vx<uint64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vasub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0;

  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2; // Double wide

  URV rmVal = 0;
  peekCsr(CsrNumber::VXRM, rmVal);
  VecRoundingMode rm = VecRoundingMode(rmVal);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          ELEM_TYPE2 temp = e1;
          temp -= e2;
          roundoff(rm, temp, 1);
          ELEM_TYPE dest = ELEM_TYPE(temp);
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
Hart<URV>::execVasub_vv(const DecodedInst* di)
{
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
    case EW::Byte:   vasub_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vasub_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vasub_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vasub_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVasubu_vv(const DecodedInst* di)
{
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
    case EW::Byte:   vasub_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vasub_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vasub_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vasub_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vasub_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0;

  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2; // Double wide

  URV rmVal = 0;
  peekCsr(CsrNumber::VXRM, rmVal);
  VecRoundingMode rm = VecRoundingMode(rmVal);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          ELEM_TYPE2 temp = e1;
          temp -= e2;
          roundoff(rm, temp, 1);
          ELEM_TYPE dest = ELEM_TYPE(temp);
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
Hart<URV>::execVasub_vx(const DecodedInst* di)
{
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
    case EW::Byte:   vasub_vx<int8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vasub_vx<int16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vasub_vx<int32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vasub_vx<int64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVasubu_vx(const DecodedInst* di)
{
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
    case EW::Byte:   vasub_vx<uint8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vasub_vx<uint16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vasub_vx<uint32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vasub_vx<uint64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsmul_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0;

  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2; // Double wide

  URV rmVal = 0;
  peekCsr(CsrNumber::VXRM, rmVal);
  VecRoundingMode rm = VecRoundingMode(rmVal);

  int elemBits = integerWidth<ELEM_TYPE> ();
  ELEM_TYPE minVal = ELEM_TYPE(1) << (elemBits - 1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          ELEM_TYPE dest = 0;
          if (e1 == minVal and e2 == minVal)
            {
              // Result saturates at max positive value.
              dest = minVal - 1;
              csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
            }
          else
            {
              ELEM_TYPE2 temp = e1;
              temp *= e2;
              roundoff(rm, temp, sizeof(ELEM_TYPE)*8 - 1);
              dest = ELEM_TYPE(temp);
            }

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
Hart<URV>::execVsmul_vv(const DecodedInst* di)
{
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
    case EW::Byte:   vsmul_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vsmul_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vsmul_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vsmul_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsmul_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0;

  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2; // Double wide

  URV rmVal = 0;
  peekCsr(CsrNumber::VXRM, rmVal);
  VecRoundingMode rm = VecRoundingMode(rmVal);

  int elemBits = integerWidth<ELEM_TYPE> ();
  ELEM_TYPE minVal = ELEM_TYPE(1) << (elemBits - 1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          ELEM_TYPE dest = 0;
          if (e1 == minVal and e2 == minVal)
            {
              // Result saturates at max positive value.
              dest = minVal - 1;
              csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
            }
          else
            {
              ELEM_TYPE2 temp = e1;
              temp *= e2;
              roundoff(rm, temp, sizeof(ELEM_TYPE)*8 - 1);
              dest = ELEM_TYPE(temp);
            }

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
Hart<URV>::execVsmul_vx(const DecodedInst* di)
{
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
    case EW::Byte:   vsmul_vx<int8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vsmul_vx<int16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vsmul_vx<int32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vsmul_vx<int64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vssr_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0, e2 = 0;

  URV rmVal = 0;
  peekCsr(CsrNumber::VXRM, rmVal);
  VecRoundingMode rm = VecRoundingMode(rmVal);

  unsigned elemBits = integerWidth<ELEM_TYPE> ();
  unsigned mask = elemBits - 1;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          ELEM_TYPE dest = e1;
          unsigned amount = unsigned(e2) & mask;
          roundoff(rm, dest, amount);

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
Hart<URV>::execVssrl_vv(const DecodedInst* di)
{
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
    case EW::Byte:   vssr_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vssr_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vssr_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vssr_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vssr_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = 0;

  URV rmVal = 0;
  peekCsr(CsrNumber::VXRM, rmVal);
  VecRoundingMode rm = VecRoundingMode(rmVal);

  unsigned elemBits = integerWidth<ELEM_TYPE> ();
  unsigned mask = elemBits - 1;
  unsigned amount = unsigned(e2) & mask;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          ELEM_TYPE dest = e1;
          roundoff(rm, dest, amount);
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
Hart<URV>::execVssrl_vx(const DecodedInst* di)
{
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
    case EW::Byte:   vssr_vx<uint8_t> (vd, vs1, e2,           group, start, elems, masked); break;
    case EW::Half:   vssr_vx<uint16_t>(vd, vs1, e2,           group, start, elems, masked); break;
    case EW::Word:   vssr_vx<uint32_t>(vd, vs1, e2,           group, start, elems, masked); break;
    case EW::Word2:  vssr_vx<uint64_t>(vd, vs1, e2,           group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVssrl_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  imm = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   vssr_vx<uint8_t> (vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Half:   vssr_vx<uint16_t>(vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Word:   vssr_vx<uint32_t>(vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Word2:  vssr_vx<uint64_t>(vd, vs1, imm,           group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVssra_vv(const DecodedInst* di)
{
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
    case EW::Byte:   vssr_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vssr_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vssr_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vssr_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVssra_vx(const DecodedInst* di)
{
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
    case EW::Byte:   vssr_vx<int8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vssr_vx<int16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vssr_vx<int32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vssr_vx<int64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVssra_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), imm = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   vssr_vx<int8_t> (vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Half:   vssr_vx<int16_t>(vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Word:   vssr_vx<int32_t>(vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Word2:  vssr_vx<int64_t>(vd, vs1, imm,          group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vnclip_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  typedef typename std::make_unsigned<ELEM_TYPE>::type  U_ELEM_TYPE;
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE2X e1 = 0;
  ELEM_TYPE e2 = 0;

  URV rmVal = 0;
  peekCsr(CsrNumber::VXRM, rmVal);
  VecRoundingMode rm = VecRoundingMode(rmVal);

  unsigned elemBits = integerWidth<ELEM_TYPE2X> ();
  unsigned mask = elemBits - 1;
  unsigned group2x = group*2;
  bool saturated = false; // True if any of the elements saturate.

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group2x, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          unsigned amount = unsigned(e2) & mask;
          roundoff(rm, e1, amount);

          ELEM_TYPE dest = ELEM_TYPE(e1);
          if (e1 != ELEM_TYPE2X(dest))
            {
              if constexpr (std::is_same<ELEM_TYPE, U_ELEM_TYPE>::value)
                dest = std::numeric_limits<ELEM_TYPE>::max();
              else
                dest = (e1 < 0) ? std::numeric_limits<ELEM_TYPE>::min() : std::numeric_limits<ELEM_TYPE>::max();
	      saturated = true;
            }

          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  if (saturated)
    csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVnclipu_wv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   vnclip_wv<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vnclip_wv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vnclip_wv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vnclip_wv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vnclip_wx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  typedef typename std::make_unsigned<ELEM_TYPE>::type  U_ELEM_TYPE;
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE2X e1 = 0;

  URV rmVal = 0;
  peekCsr(CsrNumber::VXRM, rmVal);
  VecRoundingMode rm = VecRoundingMode(rmVal);

  unsigned elemBits = integerWidth<ELEM_TYPE2X> ();
  unsigned mask = elemBits - 1;
  unsigned amount = unsigned(e2) & mask;
  unsigned group2x = group*2;
  bool saturated = false; // True if any of the elements saturate.

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group2x, e1))
        {
          roundoff(rm, e1, amount);

          ELEM_TYPE dest = ELEM_TYPE(e1);
          if (e1 != ELEM_TYPE2X(dest))
            {
              if constexpr (std::is_same<ELEM_TYPE, U_ELEM_TYPE>::value)
                dest = std::numeric_limits<ELEM_TYPE>::max();
              else
                dest = (e1 < 0) ? std::numeric_limits<ELEM_TYPE>::min() : std::numeric_limits<ELEM_TYPE>::max();
	      saturated = true;
            }

          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  if (saturated)
    csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVnclipu_wx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   vnclip_wx<uint8_t> (vd, vs1, e2,           group, start, elems, masked); break;
    case EW::Half:   vnclip_wx<uint16_t>(vd, vs1, e2,           group, start, elems, masked); break;
    case EW::Word:   vnclip_wx<uint32_t>(vd, vs1, e2,           group, start, elems, masked); break;
    case EW::Word2:  vnclip_wx<uint64_t>(vd, vs1, e2,           group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVnclipu_wi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  imm = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   vnclip_wx<uint8_t> (vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Half:   vnclip_wx<uint16_t>(vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Word:   vnclip_wx<uint32_t>(vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Word2:  vnclip_wx<uint64_t>(vd, vs1, imm,           group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVnclip_wv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   vnclip_wv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vnclip_wv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vnclip_wv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vnclip_wv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVnclip_wx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   vnclip_wx<int8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vnclip_wx<int16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vnclip_wx<int32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vnclip_wx<int64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVnclip_wi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  imm = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   vnclip_wx<int8_t> (vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Half:   vnclip_wx<int16_t>(vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Word:   vnclip_wx<int32_t>(vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Word2:  vnclip_wx<int64_t>(vd, vs1, imm,           group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorLoad(const DecodedInst* di, ElementWidth eew, bool faultFirst)
{
  vecRegs_.ldStAddr_.clear();
  vecRegs_.stData_.clear();
  vecRegs_.maskedAddr_.clear();

  // Compute emul: lmul*eew/sew
  unsigned groupX8 = vecRegs_.groupMultiplierX8();
  groupX8 = groupX8 * vecRegs_.elemWidthInBits(eew) / vecRegs_.elemWidthInBits();
  GroupMultiplier lmul = GroupMultiplier::One;
  bool badConfig = not vecRegs_.groupNumberX8ToSymbol(groupX8, lmul);
  badConfig = badConfig or not vecRegs_.legalConfig(eew, lmul);

  unsigned start = csRegs_.peekVstart();
  if (not isVecLegal() or badConfig or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(), rs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, groupX8))
    return false;

  unsigned elemCount = vecRegs_.elemCount();
  uint64_t addr = intRegs_.read(rs1) + start*sizeof(ELEM_TYPE);

  vecRegs_.ldStSize_ = sizeof(ELEM_TYPE);

  for (unsigned ix = start; ix < elemCount; ++ix, addr += sizeof(ELEM_TYPE))
    {
      bool skip = masked and not vecRegs_.isActive(0, ix);
      vecRegs_.maskedAddr_.push_back(skip);
      vecRegs_.ldStAddr_.push_back(addr);
      if (skip)
	{
	  vecRegs_.touchReg(vd, groupX8);
	  continue;
	}

      auto cause = ExceptionCause::NONE;
      uint64_t pa1 = addr, pa2 = addr; // Phys addrs or faulting virtual address.
      uint64_t gpa1 = addr, gpa2 = addr;

      ELEM_TYPE elem = 0;
      cause = determineLoadException(pa1, pa2, gpa1, gpa2, sizeof(elem),
                                     false /*hyper*/);
      if (cause == ExceptionCause::NONE)
	memRead(pa1, pa2, elem);
      else
        {
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
          if (ix == 0 or not faultFirst)
            initiateLoadException(cause, pa1, gpa1);
          return false;
        }

      if (not vecRegs_.write(vd, ix, groupX8, elem))
	assert(0);
    }
  return true;
}


template <typename URV>
void
Hart<URV>::execVle8_v(const DecodedInst* di)
{
  if (not vectorLoad<uint8_t>(di, ElementWidth::Byte, false))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVle16_v(const DecodedInst* di)
{
  if (not vectorLoad<uint16_t>(di, ElementWidth::Half, false))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVle32_v(const DecodedInst* di)
{
  if (not vectorLoad<uint32_t>(di, ElementWidth::Word, false))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVle64_v(const DecodedInst* di)
{
  if (not vectorLoad<uint64_t>(di, ElementWidth::Word2, false))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVle128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVle256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVle512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVle1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorStore(const DecodedInst* di, ElementWidth eew)
{
  vecRegs_.ldStAddr_.clear();
  vecRegs_.maskedAddr_.clear();
  vecRegs_.stData_.clear();

  // Compute emul: lmul*eew/sew
  unsigned groupX8 = vecRegs_.groupMultiplierX8();
  groupX8 = groupX8 * vecRegs_.elemWidthInBits(eew) / vecRegs_.elemWidthInBits();
  GroupMultiplier lmul = GroupMultiplier::One;
  bool badConfig = false;
  if (not vecRegs_.groupNumberX8ToSymbol(groupX8, lmul))
    badConfig = true;
  else
    badConfig = not vecRegs_.legalConfig(eew, lmul);

  if (not isVecLegal() or badConfig or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  uint32_t vd = di->op0(), rs1 = di->op1(), errors = 0;

  if (not checkVecOpsVsEmul(di, vd, groupX8))
    return false;

  unsigned elemCount = vecRegs_.elemCount();
  unsigned start = csRegs_.peekVstart();
  uint64_t addr = intRegs_.read(rs1) + start*sizeof(ELEM_TYPE);

  vecRegs_.ldStSize_ = sizeof(ELEM_TYPE);

  for (unsigned ix = start; ix < elemCount; ++ix, addr += sizeof(ELEM_TYPE))
    {
      bool skip = masked and not vecRegs_.isActive(0, ix);
      vecRegs_.ldStAddr_.push_back(addr);
      vecRegs_.maskedAddr_.push_back(skip);
      if (skip)
	{
	  vecRegs_.stData_.push_back(0);
	  continue;
	}

      ELEM_TYPE elem = 0;
      if (not vecRegs_.read(vd, ix, groupX8, elem))
        {
          errors++;
          break;
        }

      uint64_t pa1 = addr, pa2 = addr; // Physical addresses or faulting virtual addresses.
      uint64_t gpa1 = addr, gpa2 = addr;

      auto cause = determineStoreException(pa1, pa2, gpa1, gpa2, sizeof(elem),
                                           false /*hyper*/);
      if (cause == ExceptionCause::NONE)
	{
	  memWrite(pa1, pa2, elem);
	  vecRegs_.stData_.push_back(elem);
	}
      else
        {
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
          initiateStoreException(cause, pa1, gpa1);
	  assert(errors == 0);
          return false;
        }
    }

  assert(errors == 0);
  return true;
}


template <typename URV>
void
Hart<URV>::execVse8_v(const DecodedInst* di)
{
  if (not vectorStore<uint8_t>(di, ElementWidth::Byte))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVse16_v(const DecodedInst* di)
{
  if (not vectorStore<uint16_t>(di, ElementWidth::Half))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVse32_v(const DecodedInst* di)
{
  if (not vectorStore<uint32_t>(di, ElementWidth::Word))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVse64_v(const DecodedInst* di)
{
  if (not vectorStore<uint64_t>(di, ElementWidth::Word2))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVse128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVse256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVse512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVse1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlm_v(const DecodedInst* di)
{
  if (not isVecLegal() or not vecRegs_.legalConfig() or di->isMasked())
    {
      postVecFail(di);
      return;
    }

  // Change element count to byte count, elem width to byte, and emul to 1.
  uint32_t elems = vecRegs_.elemCount();
  uint32_t bytes = (elems + 7) / 8;
  ElementWidth ew = vecRegs_.elemWidth();
  GroupMultiplier gm = vecRegs_.groupMultiplier();
  vecRegs_.elemCount(bytes);
  vecRegs_.elemWidth(ElementWidth::Byte);
  vecRegs_.groupMultiplier(GroupMultiplier::One);

  // Do load bytes.
  bool ok = vectorLoad<uint8_t>(di, ElementWidth::Byte, false);

  vecRegs_.elemCount(elems); // Restore elem count.
  vecRegs_.elemWidth(ew); // Restore elem width.
  vecRegs_.groupMultiplier(gm); // Restore group multiplier

  if (not ok)
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsm_v(const DecodedInst* di)
{
  if (not isVecLegal() or not vecRegs_.legalConfig() or di->isMasked())
    {
      postVecFail(di);
      return;
    }

  // Change element count to byte count, element width to byte, and emul to 1.
  uint32_t elems = vecRegs_.elemCount();
  uint32_t bytes = (elems + 7) / 8;
  ElementWidth ew = vecRegs_.elemWidth();
  GroupMultiplier gm = vecRegs_.groupMultiplier();
  vecRegs_.elemCount(bytes);
  vecRegs_.elemWidth(ElementWidth::Byte);
  vecRegs_.groupMultiplier(GroupMultiplier::One);
  
  // Do store bytes.
  bool ok = vectorStore<uint8_t>(di, ElementWidth::Byte);

  vecRegs_.elemCount(elems); // Restore elem count.
  vecRegs_.elemWidth(ew); // Restore elem width.
  vecRegs_.groupMultiplier(gm); // Restore group multiplier

  if (not ok)
    return;
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorLoadWholeReg(const DecodedInst* di, ElementWidth eew)
{
  vecRegs_.ldStAddr_.clear();
  vecRegs_.maskedAddr_.clear();
  vecRegs_.stData_.clear();

  unsigned start = csRegs_.peekVstart();
  unsigned groupX8 = di->vecFieldCount() * 8;
  GroupMultiplier gm = GroupMultiplier::One;
  bool badConfig = not vecRegs_.groupNumberX8ToSymbol(groupX8, gm);
  badConfig = badConfig or not vecRegs_.legalConfig(eew, gm);
  if ((not isVecLegal()) or badConfig or di->isMasked())
    {
      postVecFail(di);
      return false;
    }

  unsigned vd = di->op0(), rs1 = di->op1();

  unsigned elemBytes = vecRegs_.elemWidthInBytes(eew);
  unsigned elemCount = (groupX8*vecRegs_.bytesPerRegister()) / elemBytes / 8;
  URV addr = intRegs_.read(rs1) + start*sizeof(ELEM_TYPE);

  vecRegs_.ldStSize_ = sizeof(ELEM_TYPE);

  for (unsigned ix = start; ix < elemCount; ++ix, addr += sizeof(ELEM_TYPE))
    {
      auto cause = ExceptionCause::NONE;
      uint64_t pa1 = addr, pa2 = addr; // Physical addresses or faulting virtual addresses.
      uint64_t gpa1 = addr, gpa2 = addr;

      ELEM_TYPE elem = 0;
      cause = determineLoadException(pa1, pa2, gpa1, gpa2, sizeof(elem),
                                     false /*hyper*/);
      if (cause == ExceptionCause::NONE)
	memRead(pa1, pa2, elem);

      if (cause != ExceptionCause::NONE)
        {
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
          initiateLoadException(cause, pa1, gpa1);
          return false;
        }

      if (not vecRegs_.write(vd, ix, groupX8, elem))
        {
          assert(0);
          return false;
        }

      vecRegs_.ldStAddr_.push_back(addr);
      vecRegs_.maskedAddr_.push_back(false);
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVlre8_v(const DecodedInst* di)
{
  if (not vectorLoadWholeReg<uint8_t>(di, ElementWidth::Byte))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlre16_v(const DecodedInst* di)
{
  if (not vectorLoadWholeReg<uint16_t>(di, ElementWidth::Half))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlre32_v(const DecodedInst* di)
{
  if (not vectorLoadWholeReg<uint32_t>(di, ElementWidth::Word))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlre64_v(const DecodedInst* di)
{
  if (not vectorLoadWholeReg<uint64_t>(di, ElementWidth::Word2))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlre128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlre256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlre512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlre1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
bool
Hart<URV>::vectorStoreWholeReg(const DecodedInst* di, GroupMultiplier gm)
{
  vecRegs_.ldStAddr_.clear();
  vecRegs_.maskedAddr_.clear();
  vecRegs_.stData_.clear();

  unsigned start = csRegs_.peekVstart();
  unsigned groupX8 = vecRegs_.groupMultiplierX8(gm);
  ElementWidth eew = ElementWidth::Byte;
  if (not isVecLegal()  or  not vecRegs_.legalConfig(eew, gm)  or  di->isMasked())
    {
      postVecFail(di);
      return false;
    }

  unsigned vd = di->op0(), rs1 = di->op1();

  unsigned elemSize = 1;
  unsigned elemCount = (groupX8*vecRegs_.bytesPerRegister()) / elemSize / 8;
  URV addr = intRegs_.read(rs1) + start*elemSize;

  vecRegs_.ldStSize_ = sizeof(uint8_t);

  for (unsigned ix = start; ix < elemCount; ++ix, addr += elemSize)
    {
      uint8_t elem = 0;
      if (not vecRegs_.read(vd, ix, groupX8, elem))
        {
          assert(0);
          return false;
        }

      uint64_t pa1 = addr, pa2 = addr; // Physical addresses or faulting virtual addresses.
      uint64_t gpa1 = addr, gpa2 = addr;
      auto cause = determineStoreException(pa1, pa2, gpa1, gpa2, sizeof(elem),
                                           false /*hyper*/);
      if (cause == ExceptionCause::NONE)
	{
	  memWrite(pa1, pa2, elem);
	  vecRegs_.ldStAddr_.push_back(addr);
	  vecRegs_.maskedAddr_.push_back(false);
	  vecRegs_.stData_.push_back(elem);
	}
      else
        {
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
          initiateStoreException(cause, pa1, gpa1);
          return false;
        }
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVs1r_v(const DecodedInst* di)
{
  if (not vectorStoreWholeReg(di, GroupMultiplier::One))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVs2r_v(const DecodedInst* di)
{
  if (not vectorStoreWholeReg(di, GroupMultiplier::Two))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVs4r_v(const DecodedInst* di)
{
  if (not vectorStoreWholeReg(di, GroupMultiplier::Four))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVs8r_v(const DecodedInst* di)
{
  if (not vectorStoreWholeReg(di, GroupMultiplier::Eight))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVle8ff_v(const DecodedInst* di)
{
  if (not vectorLoad<uint8_t>(di, ElementWidth::Byte, true))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVle16ff_v(const DecodedInst* di)
{
  if (not vectorLoad<uint16_t>(di, ElementWidth::Half, true))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVle32ff_v(const DecodedInst* di)
{
  if (not vectorLoad<uint32_t>(di, ElementWidth::Word, true))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVle64ff_v(const DecodedInst* di)
{
  if (not vectorLoad<uint64_t>(di, ElementWidth::Word2, true))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVle128ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVle256ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVle512ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVle1024ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorLoadStrided(const DecodedInst* di, ElementWidth eew)
{
  vecRegs_.ldStAddr_.clear();
  vecRegs_.maskedAddr_.clear();
  vecRegs_.stData_.clear();

  // Compute emul: lmul*eew/sew
  unsigned groupX8 = vecRegs_.groupMultiplierX8();
  groupX8 = groupX8 * vecRegs_.elemWidthInBits(eew) / vecRegs_.elemWidthInBits();
  GroupMultiplier lmul = GroupMultiplier::One;
  bool badConfig = false;
  if (not vecRegs_.groupNumberX8ToSymbol(groupX8, lmul))
    badConfig = true;
  else
    badConfig = not vecRegs_.legalConfig(eew, lmul);

  if (not isVecLegal() or badConfig or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(), rs1 = di->op1(), rs2 = di->op2();

  if (not checkVecOpsVsEmul(di, vd, groupX8))
    return false;

  uint64_t stride = intRegs_.read(rs2);
  unsigned start = csRegs_.peekVstart();
  unsigned elemCount = vecRegs_.elemCount();
  uint64_t addr = intRegs_.read(rs1) + start*stride;

  vecRegs_.ldStSize_ = sizeof(ELEM_TYPE);

  for (unsigned ix = start; ix < elemCount; ++ix, addr += stride)
    {
      bool skip = masked and not vecRegs_.isActive(0, ix);
      vecRegs_.ldStAddr_.push_back(addr);
      vecRegs_.maskedAddr_.push_back(skip);
      if (skip)
	{
	  vecRegs_.touchReg(vd, groupX8);
	  continue;
	}

      auto cause = ExceptionCause::NONE;
      uint64_t pa1 = addr, pa2 = addr; // Physical addresses or faulting virtual addresses.
      uint64_t gpa1 = addr, gpa2 = addr;

      ELEM_TYPE elem = 0;
      cause = determineLoadException(pa1, pa2, gpa1, gpa2, sizeof(elem),
                                     false /*hyper*/);
      if (cause == ExceptionCause::NONE)
	memRead(pa1, pa2, elem);
      else
        {
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
          initiateLoadException(cause, pa1, gpa1);
          return false;
        }

      if (not vecRegs_.write(vd, ix, groupX8, elem))
        {
          assert(0);
          return false;
        }
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVlse8_v(const DecodedInst* di)
{
  if (not vectorLoadStrided<uint8_t>(di, ElementWidth::Byte))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlse16_v(const DecodedInst* di)
{
  if (not vectorLoadStrided<uint16_t>(di, ElementWidth::Half))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlse32_v(const DecodedInst* di)
{
  if (not vectorLoadStrided<uint32_t>(di, ElementWidth::Word))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlse64_v(const DecodedInst* di)
{
  if (not vectorLoadStrided<uint64_t>(di, ElementWidth::Word2))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlse128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlse256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlse512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlse1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorStoreStrided(const DecodedInst* di, ElementWidth eew)
{
  vecRegs_.ldStAddr_.clear();
  vecRegs_.maskedAddr_.clear();
  vecRegs_.stData_.clear();

  // Compute emul: lmul*eew/sew
  unsigned groupX8 = vecRegs_.groupMultiplierX8();
  groupX8 = groupX8 * vecRegs_.elemWidthInBits(eew) / vecRegs_.elemWidthInBits();
  GroupMultiplier lmul = GroupMultiplier::One;
  bool badConfig = false;
  if (not vecRegs_.groupNumberX8ToSymbol(groupX8, lmul))
    badConfig = true;
  else
    badConfig = not vecRegs_.legalConfig(eew, lmul);

  unsigned start = csRegs_.peekVstart();
  if (not isVecLegal() or badConfig or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(), rs1 = di->op1(), rs2 = di->op2();

  if (not checkVecOpsVsEmul(di, vd, groupX8))
    return false;

  uint64_t stride = intRegs_.read(rs2);
  unsigned elemCount = vecRegs_.elemCount();
  uint64_t addr = intRegs_.read(rs1) + start*stride;

  vecRegs_.ldStSize_ = sizeof(ELEM_TYPE);

  // TODO check permissions, translate, ....
  for (unsigned ix = start; ix < elemCount; ++ix, addr += stride)
    {
      bool skip = masked and not vecRegs_.isActive(0, ix);
      vecRegs_.ldStAddr_.push_back(addr);
      vecRegs_.maskedAddr_.push_back(skip);
      if (skip)
	{
	  vecRegs_.stData_.push_back(0);
	  continue;
	}

      ELEM_TYPE elem = 0;
      if (not vecRegs_.read(vd, ix, groupX8, elem))
        {
          assert(0);
          return false;
        }

      uint64_t pa1 = addr, pa2 = addr; // Physical addresses or faulting virtual addresses.
      uint64_t gpa1 = addr, gpa2 = addr;

      auto cause = determineStoreException(pa1, pa2, gpa1, gpa2, sizeof(elem),
                                           false /*hyper*/);
      if (cause == ExceptionCause::NONE)
	{
	  memWrite(pa1, pa2, elem);
	  vecRegs_.stData_.push_back(elem);
	}
      else
        {
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
          initiateStoreException(cause, pa1, gpa1);
          return false;
        }
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVsse8_v(const DecodedInst* di)
{
  if (not vectorStoreStrided<uint8_t>(di, ElementWidth::Byte))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsse16_v(const DecodedInst* di)
{
  if (not vectorStoreStrided<uint16_t>(di, ElementWidth::Half))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsse32_v(const DecodedInst* di)
{
  if (not vectorStoreStrided<uint32_t>(di, ElementWidth::Word))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsse64_v(const DecodedInst* di)
{
  if (not vectorStoreStrided<uint64_t>(di, ElementWidth::Word2))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsse128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsse256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsse512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsse1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorLoadIndexed(const DecodedInst* di, ElementWidth offsetEew)
{
  vecRegs_.ldStAddr_.clear();
  vecRegs_.maskedAddr_.clear();
  vecRegs_.stData_.clear();

  uint32_t elemWidth = vecRegs_.elemWidthInBits();
  uint32_t offsetWidth = vecRegs_.elemWidthInBits(offsetEew);

  uint32_t groupX8 = vecRegs_.groupMultiplierX8();
  uint32_t offsetGroupX8 = (offsetWidth*groupX8)/elemWidth;

  GroupMultiplier offsetGroup{GroupMultiplier::One};
  bool badConfig = not vecRegs_.groupNumberX8ToSymbol(offsetGroupX8, offsetGroup);
  badConfig = badConfig or not vecRegs_.legalConfig(offsetEew, offsetGroup);
  if (not isVecLegal() or badConfig or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  uint32_t vd = di->op0(), rs1 = di->op1(), vi = di->op2();

  if (not checkVecOpsVsEmul(di, vd, groupX8))
    return false;

  uint64_t addr = intRegs_.read(rs1);

  unsigned start = csRegs_.peekVstart();
  unsigned elemCount = vecRegs_.elemCount(), elemSize = elemWidth / 8;
  vecRegs_.ldStSize_ = elemSize;

  for (unsigned ix = start; ix < elemCount; ++ix)
    {
      uint64_t offset = 0;
      if (not vecRegs_.readStride(vi, ix, offsetEew, offsetGroupX8, offset))
	{
	  assert(0);
	  return false;
	}
      uint64_t vaddr = addr + offset;
      bool skip = masked and not vecRegs_.isActive(0, ix);
      vecRegs_.ldStAddr_.push_back(vaddr);
      vecRegs_.maskedAddr_.push_back(skip);
      if (skip)
	{
	  vecRegs_.touchReg(vd, groupX8);
	  continue;
	}

      uint64_t pa1 = vaddr, pa2 = vaddr; // Physical addresses or faulting virtual addresses.
      uint64_t gpa1 = vaddr, gpa2 = vaddr;

      auto cause = determineLoadException(pa1, pa2, gpa1, gpa2, elemSize,
                                          false /*hyper*/);
      if (cause == ExceptionCause::NONE)
	{
	  if (elemSize == 1)
	    {
	      uint8_t x = 0;
	      memRead(pa1, pa2, x);
	      if (not vecRegs_.write(vd, ix, groupX8, x)) assert(0);
	    }
	  else if (elemSize == 2)
	    {
	      uint16_t x = 0;
	      memRead(pa1, pa2, x);
	      if (not vecRegs_.write(vd, ix, groupX8, x)) assert(0);
	    }
	  else if (elemSize == 4)
	    {
	      uint32_t x = 0;
	      memRead(pa1, pa2, x);
	      if (not vecRegs_.write(vd, ix, groupX8, x)) assert(0);
	    }
	  else if (elemSize == 8)
	    {
	      uint64_t x = 0;
	      memRead(pa1, pa2, x);
	      if (not vecRegs_.write(vd, ix, groupX8, x)) assert(0);
	    }
	  else
	    assert(0);
	}
      else
        {
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
          initiateLoadException(cause, pa1, gpa1);
          return false;
        }
    }
  return true;
}


template <typename URV>
void
Hart<URV>::execVloxei8_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadIndexed<uint8_t>(di,  EW::Byte); break;
    case EW::Half:   ok = vectorLoadIndexed<uint16_t>(di, EW::Byte); break;
    case EW::Word:   ok = vectorLoadIndexed<uint32_t>(di, EW::Byte); break;
    case EW::Word2:  ok = vectorLoadIndexed<uint64_t>(di, EW::Byte); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVloxei16_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadIndexed<uint8_t>(di,  EW::Half); break;
    case EW::Half:   ok = vectorLoadIndexed<uint16_t>(di, EW::Half); break;
    case EW::Word:   ok = vectorLoadIndexed<uint32_t>(di, EW::Half); break;
    case EW::Word2:  ok = vectorLoadIndexed<uint64_t>(di, EW::Half); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVloxei32_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadIndexed<uint8_t>(di,  EW::Word); break;
    case EW::Half:   ok = vectorLoadIndexed<uint16_t>(di, EW::Word); break;
    case EW::Word:   ok = vectorLoadIndexed<uint32_t>(di, EW::Word); break;
    case EW::Word2:  ok = vectorLoadIndexed<uint64_t>(di, EW::Word); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVloxei64_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadIndexed<uint8_t>(di,  EW::Word2); break;
    case EW::Half:   ok = vectorLoadIndexed<uint16_t>(di, EW::Word2); break;
    case EW::Word:   ok = vectorLoadIndexed<uint32_t>(di, EW::Word2); break;
    case EW::Word2:  ok = vectorLoadIndexed<uint64_t>(di, EW::Word2); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVluxei8_v(const DecodedInst* di)
{
  execVloxei8_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxei16_v(const DecodedInst* di)
{
  execVloxei16_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxei32_v(const DecodedInst* di)
{
  execVloxei32_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxei64_v(const DecodedInst* di)
{
  execVloxei64_v(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorStoreIndexed(const DecodedInst* di, ElementWidth offsetEew)
{
  vecRegs_.ldStAddr_.clear();
  vecRegs_.maskedAddr_.clear();
  vecRegs_.stData_.clear();

  uint32_t elemWidth = vecRegs_.elemWidthInBits();
  uint32_t offsetWidth = vecRegs_.elemWidthInBits(offsetEew);

  uint32_t groupX8 = vecRegs_.groupMultiplierX8();
  uint32_t offsetGroupX8 = (offsetWidth*groupX8)/elemWidth;

  GroupMultiplier offsetGroup{GroupMultiplier::One};
  bool badConfig = not vecRegs_.groupNumberX8ToSymbol(offsetGroupX8, offsetGroup);
  badConfig = badConfig or not vecRegs_.legalConfig(offsetEew, offsetGroup);
  if (not isVecLegal() or badConfig or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  uint32_t vd = di->op0(), rs1 = di->op1(), vi = di->op2();

  if (not checkVecOpsVsEmul(di, vd, groupX8))
    return false;

  uint64_t addr = intRegs_.read(rs1);
  unsigned start = csRegs_.peekVstart();
  unsigned elemCount = vecRegs_.elemCount(), elemSize = elemWidth / 8;
  vecRegs_.ldStSize_ = elemSize;

  for (unsigned ix = start; ix < elemCount; ++ix)
    {
      uint64_t offset = 0;
      if (not vecRegs_.readStride(vi, ix, offsetEew, offsetGroupX8, offset))
	assert(0);

      uint64_t vaddr = addr + offset, data = 0;
      bool skip = masked and not vecRegs_.isActive(0, ix);
      vecRegs_.ldStAddr_.push_back(vaddr);
      vecRegs_.maskedAddr_.push_back(skip);
      if (skip)
	{
	  vecRegs_.stData_.push_back(0);
	  continue;
	}

      uint64_t pa1 = vaddr, pa2 = vaddr; // Physical addresses or faulting virtual addresses.
      uint64_t gpa1 = vaddr, gpa2 = vaddr;

      auto cause = ExceptionCause::NONE;
      if (elemSize == 1)
	{
	  uint8_t x = 0;
	  if (not vecRegs_.read(vd, ix, groupX8, x)) assert(0);
	  cause = determineStoreException(pa1, pa2, gpa1, gpa2, sizeof(x),
                                          false /*hyper*/);
	  if (cause == ExceptionCause::NONE)
	    memWrite(pa1, pa2, x);
	  data = x;
	}
      else if (elemSize == 2)
	{
	  uint16_t x = 0;
	  if (not vecRegs_.read(vd, ix, groupX8, x)) assert(0);
	  cause = determineStoreException(pa1, pa2, gpa1, gpa2, sizeof(x),
                                          false /*hyper*/);
	  if (cause == ExceptionCause::NONE)
	    memWrite(pa1, pa2, x);
	  data = x;
	}
      else if (elemSize == 4)
	{
	  uint32_t x = 0;
	  if (not vecRegs_.read(vd, ix, groupX8, x)) assert(0);
	  cause = determineStoreException(pa1, pa2, gpa1, gpa2, sizeof(x),
                                          false /*hyper*/);
	  if (cause == ExceptionCause::NONE)
	    memWrite(pa1, pa2, x);
	  data = x;
	}
      else if (elemSize == 8)
	{
	  uint64_t x = 0;
	  if (not vecRegs_.read(vd, ix, groupX8, x)) assert(0);
	  cause = determineStoreException(pa1, pa2, gpa1, gpa2, sizeof(x),
                                          false /*hyper*/);
	  if (cause == ExceptionCause::NONE)
	    memWrite(pa1, pa2, x);
	  data = x;
	}
      else
	assert(0);

      if (cause != ExceptionCause::NONE)
        {
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
          initiateStoreException(cause, pa1, gpa1);
          return false;
        }

      vecRegs_.stData_.push_back(data);
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVsoxei8_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreIndexed<uint8_t>(di,  EW::Byte); break;
    case EW::Half:   ok = vectorStoreIndexed<uint16_t>(di, EW::Byte); break;
    case EW::Word:   ok = vectorStoreIndexed<uint32_t>(di, EW::Byte); break;
    case EW::Word2:  ok = vectorStoreIndexed<uint64_t>(di, EW::Byte); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsoxei16_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreIndexed<uint8_t>(di,  EW::Half); break;
    case EW::Half:   ok = vectorStoreIndexed<uint16_t>(di, EW::Half); break;
    case EW::Word:   ok = vectorStoreIndexed<uint32_t>(di, EW::Half); break;
    case EW::Word2:  ok = vectorStoreIndexed<uint64_t>(di, EW::Half); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsoxei32_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreIndexed<uint8_t>(di,  EW::Word); break;
    case EW::Half:   ok = vectorStoreIndexed<uint16_t>(di, EW::Word); break;
    case EW::Word:   ok = vectorStoreIndexed<uint32_t>(di, EW::Word); break;
    case EW::Word2:  ok = vectorStoreIndexed<uint64_t>(di, EW::Word); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsoxei64_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreIndexed<uint8_t>(di,  EW::Word2); break;
    case EW::Half:   ok = vectorStoreIndexed<uint16_t>(di, EW::Word2); break;
    case EW::Word:   ok = vectorStoreIndexed<uint32_t>(di, EW::Word2); break;
    case EW::Word2:  ok = vectorStoreIndexed<uint64_t>(di, EW::Word2); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsuxei8_v(const DecodedInst* di)
{
  execVsoxei8_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxei16_v(const DecodedInst* di)
{
  execVsoxei16_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxei32_v(const DecodedInst* di)
{
  execVsoxei32_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxei64_v(const DecodedInst* di)
{
  execVsoxei64_v(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorLoadSeg(const DecodedInst* di, ElementWidth eew,
			 unsigned fieldCount, uint64_t stride, bool faultFirst)
{
  vecRegs_.ldStAddr_.clear();
  vecRegs_.maskedAddr_.clear();
  vecRegs_.stData_.clear();

  // Compute emul: lmul*eew/sew
  unsigned groupX8 = vecRegs_.groupMultiplierX8();
  groupX8 = groupX8 * vecRegs_.elemWidthInBits(eew) / vecRegs_.elemWidthInBits();
  GroupMultiplier lmul = GroupMultiplier::One;
  bool badConfig = not vecRegs_.groupNumberX8ToSymbol(groupX8, lmul);
  badConfig = badConfig or not vecRegs_.legalConfig(eew, lmul);
  badConfig = badConfig or (groupX8*fieldCount > 64);

  if (not isVecLegal() or badConfig or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(), rs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, groupX8))
    return false;

  unsigned start = csRegs_.peekVstart();
  uint64_t addr = intRegs_.read(rs1) + start*stride;
  unsigned elemCount = vecRegs_.elemCount();
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;

  // Used registers must not exceed 32.
  if (vd + fieldCount*eg > 32)
    {
      postVecFail(di);
      return false;
    }

  unsigned elemSize = sizeof(ELEM_TYPE);
  vecRegs_.ldStSize_ = elemSize;

  for (unsigned ix = start; ix < elemCount; ++ix, addr += stride)
    {
      uint64_t faddr = addr;  // Field address

      for (unsigned field = 0; field < fieldCount; ++field, faddr += elemSize)
	{
	  unsigned dvg = vd + field*eg;   // Destination vector gorup.
	  bool skip = masked and not vecRegs_.isActive(0, ix);
	  vecRegs_.ldStAddr_.push_back(faddr);
	  vecRegs_.maskedAddr_.push_back(skip);
	  if (skip)
	    {
	      vecRegs_.touchReg(dvg, groupX8);
	      continue;
	    }

	  ELEM_TYPE elem(0);
	  uint64_t pa1 = faddr, pa2 = faddr; // Physical addresses or faulting virtual addresses.
          uint64_t gpa1 = faddr, gpa2 = faddr;

	  auto cause = determineLoadException(pa1, pa2, gpa1, gpa2, sizeof(elem),
                                              false /*hyper*/);

	  if (cause == ExceptionCause::NONE)
            memRead(pa1, pa2, elem);
	  else
	    {
	      csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
	      if (ix == 0 or not faultFirst)
		initiateLoadException(cause, pa1, gpa1);
	      return false;
	    }

	  if (not vecRegs_.write(dvg, ix, groupX8, elem))
	    assert(0);
	}
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVlsege8_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint8_t);
  if (not vectorLoadSeg<uint8_t>(di, ElementWidth::Byte, fieldCount, stride, false))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlsege16_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint16_t);
  if (not vectorLoadSeg<uint16_t>(di, ElementWidth::Half, fieldCount, stride, false))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlsege32_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint32_t);
  if (not vectorLoadSeg<uint32_t>(di, ElementWidth::Word, fieldCount, stride, false))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlsege64_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint64_t);
  if (not vectorLoadSeg<uint64_t>(di, ElementWidth::Word2, fieldCount, stride, false))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlsege128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlsege256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlsege512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlsege1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorStoreSeg(const DecodedInst* di, ElementWidth eew,
			  unsigned fieldCount, uint64_t stride)
{
  vecRegs_.ldStAddr_.clear();
  vecRegs_.maskedAddr_.clear();
  vecRegs_.stData_.clear();

  // Compute emul: lmul*eew/sew
  unsigned groupX8 = vecRegs_.groupMultiplierX8();
  groupX8 = groupX8 * vecRegs_.elemWidthInBits(eew) / vecRegs_.elemWidthInBits();
  GroupMultiplier lmul = GroupMultiplier::One;
  bool badConfig = not vecRegs_.groupNumberX8ToSymbol(groupX8, lmul);
  badConfig = badConfig or not vecRegs_.legalConfig(eew, lmul);
  badConfig = badConfig or (groupX8*fieldCount > 64);

  unsigned start = csRegs_.peekVstart();
  if (not isVecLegal() or badConfig or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(), rs1 = di->op1();

  if (not checkVecOpsVsEmul(di, vd, groupX8))
    return false;

  uint64_t addr = intRegs_.read(rs1) + start*stride;
  unsigned elemCount = vecRegs_.elemCount(), elemSize = sizeof(ELEM_TYPE);
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;

  // Used registers must not exceed 32.
  if (vd + fieldCount*eg > 32)
    {
      postVecFail(di);
      return false;
    }

  vecRegs_.ldStSize_ = sizeof(ELEM_TYPE);

  for (unsigned ix = start; ix < elemCount; ++ix, addr += stride)
    {
      uint64_t faddr = addr;   // Field address

      for (unsigned field = 0; field < fieldCount; ++field, faddr += elemSize)
	{
	  uint64_t pa1 = faddr, pa2 = faddr; // Physical addresses or faulting virtual addresses.
          uint64_t gpa1 = faddr, gpa2 = faddr;
	  unsigned dvg = vd + field*eg;   // Source vector gorup.

	  bool skip = masked and not vecRegs_.isActive(0, ix);
	  vecRegs_.ldStAddr_.push_back(faddr);
	  vecRegs_.maskedAddr_.push_back(skip);
	  if (skip)
	    {
	      vecRegs_.stData_.push_back(0);
	      continue;
	    }

	  ELEM_TYPE elem = 0;
	  if (not vecRegs_.read(dvg, ix, groupX8, elem))
	    assert(0);

	  auto cause = determineStoreException(pa1, pa2, gpa1, gpa2, sizeof(elem),
                                               false /*hyper*/);
	  if (cause == ExceptionCause::NONE)
	    {
	      memWrite(pa1, pa2, elem);
	      vecRegs_.stData_.push_back(elem);
	    }
	  else
	    {
	      csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
	      initiateStoreException(cause, pa1, gpa1);
	      return false;
	    }
	}
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVssege8_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint8_t);
  if (not vectorStoreSeg<uint8_t>(di, ElementWidth::Byte, fieldCount, stride))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVssege16_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint16_t);
  if (not vectorStoreSeg<uint16_t>(di, ElementWidth::Half, fieldCount, stride))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVssege32_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint32_t);
  if (not vectorStoreSeg<uint32_t>(di, ElementWidth::Word, fieldCount, stride))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVssege64_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint64_t);
  if (not vectorStoreSeg<uint64_t>(di, ElementWidth::Word2, fieldCount, stride))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVssege128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVssege256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVssege512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVssege1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlssege8_v(const DecodedInst* di)
{
  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorLoadSeg<uint8_t>(di, ElementWidth::Byte, fieldCount, stride, false))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlssege16_v(const DecodedInst* di)
{
  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorLoadSeg<uint16_t>(di, ElementWidth::Half, fieldCount, stride, false))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlssege32_v(const DecodedInst* di)
{
  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorLoadSeg<uint32_t>(di, ElementWidth::Word, fieldCount, stride, false))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlssege64_v(const DecodedInst* di)
{
  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorLoadSeg<uint64_t>(di, ElementWidth::Word2, fieldCount, stride, false))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlssege128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlssege256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlssege512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlssege1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsssege8_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di, vecRegs_.groupMultiplier(), ElementWidth::Byte))
    return;

  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorStoreSeg<uint8_t>(di, ElementWidth::Byte, fieldCount, stride))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsssege16_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di, vecRegs_.groupMultiplier(), ElementWidth::Half))
    return;

  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorStoreSeg<uint16_t>(di, ElementWidth::Half, fieldCount, stride))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsssege32_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di, vecRegs_.groupMultiplier(), ElementWidth::Word))
    return;

  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorStoreSeg<uint32_t>(di, ElementWidth::Word, fieldCount, stride))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsssege64_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di, vecRegs_.groupMultiplier(), ElementWidth::Word2))
    return;

  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorStoreSeg<uint64_t>(di, ElementWidth::Word2, fieldCount, stride))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsssege128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsssege256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsssege512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsssege1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorLoadSegIndexed(const DecodedInst* di, ElementWidth offsetEew)
{
  vecRegs_.ldStAddr_.clear();
  vecRegs_.maskedAddr_.clear();
  vecRegs_.stData_.clear();

  uint32_t elemWidth = vecRegs_.elemWidthInBits();
  uint32_t offsetWidth = vecRegs_.elemWidthInBits(offsetEew);

  uint32_t groupX8 = vecRegs_.groupMultiplierX8();
  uint32_t offsetGroupX8 = (offsetWidth*groupX8)/elemWidth;

  GroupMultiplier offsetGroup{GroupMultiplier::One};
  bool badConfig = not vecRegs_.groupNumberX8ToSymbol(offsetGroupX8, offsetGroup);
  badConfig = badConfig or not vecRegs_.legalConfig(offsetEew, offsetGroup);
  if (not isVecLegal() or badConfig or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  uint32_t vd = di->op0(), rs1 = di->op1(), vi = di->op2();

  if (not checkVecOpsVsEmul(di, vd, groupX8))
    return false;

  uint64_t addr = intRegs_.read(rs1);
  unsigned start = csRegs_.peekVstart(), elemSize = elemWidth / 8;
  unsigned elemCount = vecRegs_.elemCount(), fieldCount = di->vecFieldCount();
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;

  // Used registers must not exceed 32.
  if (vd + fieldCount*eg > 32)
    {
      postVecFail(di);
      return false;
    }
  vecRegs_.ldStSize_ = elemSize;

  for (unsigned ix = start; ix < elemCount; ++ix)
    {
      uint64_t offset = 0;
      if (not vecRegs_.readStride(vi, ix, offsetEew, offsetGroupX8, offset))
	assert(0);

      uint64_t faddr = addr + offset;

      for (unsigned field = 0; field < fieldCount; ++field, faddr += elemSize)
	{
	  unsigned dvg = vd + field*eg;  // Destination vector grop.
	  bool skip = masked and not vecRegs_.isActive(0, ix);
	  vecRegs_.ldStAddr_.push_back(faddr);
	  vecRegs_.maskedAddr_.push_back(skip);
	  if (skip)
	    {
	      vecRegs_.touchReg(dvg, groupX8);
	      continue;
	    }

	  uint64_t pa1 = faddr, pa2 = faddr; // Physical addresses or faulting virtual addresses.
          uint64_t gpa1 = faddr, gpa2 = faddr;

          auto cause = determineLoadException(pa1, pa2, gpa1, gpa2, elemSize,
                                              false /*hyper*/);
	  if (cause == ExceptionCause::NONE)
	    {
	      if (elemSize == 1)
		{
		  uint8_t x = 0;
		  memRead(pa1, pa2, x);
		  if (not vecRegs_.write(dvg, ix, groupX8, x)) assert(0);
		}
	      else if (elemSize == 2)
		{
		  uint16_t x = 0;
		  memRead(pa1, pa2, x);
		  if (not vecRegs_.write(dvg, ix, groupX8, x)) assert(0);
		}
	      else if (elemSize == 4)
		{
		  uint32_t x = 0;
		  memRead(pa1, pa2, x);
		  if (not vecRegs_.write(dvg, ix, groupX8, x)) assert(0);
		}
	      else if (elemSize == 8)
		{
		  uint64_t x = 0;
		  memRead(pa1, pa2, x);
		  if (not vecRegs_.write(dvg, ix, groupX8, x)) assert(0);
		}
	      else
		assert(0);
	    }
	  else
	    {
	      csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
	      initiateLoadException(cause, pa1, gpa1);
	      return false;
	    }
	}
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVluxsegei8_v(const DecodedInst* di)
{
  execVloxsegei8_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxsegei16_v(const DecodedInst* di)
{
  execVloxsegei16_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxsegei32_v(const DecodedInst* di)
{
  execVloxsegei32_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxsegei64_v(const DecodedInst* di)
{
  execVloxsegei64_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxsegei128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVluxsegei256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVluxsegei512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVluxsegei1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorStoreSegIndexed(const DecodedInst* di, ElementWidth offsetEew)
{
  vecRegs_.ldStAddr_.clear();
  vecRegs_.maskedAddr_.clear();
  vecRegs_.stData_.clear();

  uint32_t elemWidth = vecRegs_.elemWidthInBits();
  uint32_t offsetWidth = vecRegs_.elemWidthInBits(offsetEew);

  uint32_t groupX8 = vecRegs_.groupMultiplierX8();
  uint32_t offsetGroupX8 = (offsetWidth*groupX8)/elemWidth;

  GroupMultiplier offsetGroup{GroupMultiplier::One};
  bool badConfig = not vecRegs_.groupNumberX8ToSymbol(offsetGroupX8, offsetGroup);
  badConfig = badConfig or not vecRegs_.legalConfig(offsetEew, offsetGroup);
  if (not isVecLegal() or badConfig or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  uint32_t vd = di->op0(), rs1 = di->op1(), vi = di->op2();

  if (not checkVecOpsVsEmul(di, vd, groupX8))
    return false;

  uint64_t addr = intRegs_.read(rs1);
  unsigned start = csRegs_.peekVstart(), elemSize = elemWidth / 8;
  unsigned elemCount = vecRegs_.elemCount(), fieldCount = di->vecFieldCount();
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;

  // Used registers must not exceed 32.
  if (vd + fieldCount*eg > 32)
    {
      postVecFail(di);
      return false;
    }
  vecRegs_.ldStSize_ = elemSize;

  for (unsigned ix = start; ix < elemCount; ++ix)
    {
      uint64_t offset = 0;
      if (not vecRegs_.readStride(vi, ix, offsetEew, offsetGroupX8, offset))
	assert(0);

      uint64_t faddr = addr + offset, data = 0;

      for (unsigned field = 0; field < fieldCount; ++field, faddr += elemSize)
	{
	  unsigned dvg = vd + field*eg;  // Source vector grop.
	  bool skip = masked and not vecRegs_.isActive(0, ix);
	  vecRegs_.ldStAddr_.push_back(faddr);
	  vecRegs_.maskedAddr_.push_back(skip);
	  if (skip)
	    {
	      vecRegs_.stData_.push_back(0);
	      continue;
	    }

	  auto cause = ExceptionCause::NONE;
	  uint64_t pa1 = faddr, pa2 = faddr; // Physical addresses or faulting virtual addresses.
          uint64_t gpa1 = faddr, gpa2 = faddr;

	  if (elemSize == 1)
	    {
	      uint8_t x = 0;
	      if (not vecRegs_.read(dvg, ix, groupX8, x)) assert(0);
	      cause = determineStoreException(pa1, pa2, gpa1, gpa2, sizeof(x),
                                              false /*hyper*/);
	      if (cause == ExceptionCause::NONE)
		memWrite(pa1, pa2, x);
	      data = x;
	    }
	  else if (elemSize == 2)
	    {
	      uint16_t x = 0;
	      if (not vecRegs_.read(dvg, ix, groupX8, x)) assert(0);
	      cause = determineStoreException(pa1, pa2, gpa1, gpa2, sizeof(x),
                                              false /*hyper*/);
	      if (cause == ExceptionCause::NONE)
		memWrite(pa1, pa2, x);
	      data = x;
	    }
	  else if (elemSize == 4)
	    {
	      uint32_t x = 0;
	      if (not vecRegs_.read(dvg, ix, groupX8, x)) assert(0);
	      cause = determineStoreException(pa1, pa2, gpa1, gpa2, sizeof(x),
                                              false /*hyper*/);
	      if (cause == ExceptionCause::NONE)
		memWrite(pa1, pa2, x);
	      data = x;
	    }
	  else if (elemSize == 8)
	    {
	      uint64_t x = 0;
	      if (not vecRegs_.read(dvg, ix, groupX8, x)) assert(0);
	      cause = determineStoreException(pa1, pa2, gpa1, gpa2, sizeof(x),
                                              false /*hyper*/);
	      if (cause == ExceptionCause::NONE)
		memWrite(pa1, pa2, x);
	      data = x;
	    }
	  else
	    assert(0);

	  if (cause != ExceptionCause::NONE)
	    {
	      csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
	      initiateStoreException(cause, pa1, gpa1);
	      return false;
	    }

	  vecRegs_.stData_.push_back(data);
	}
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVsuxsegei8_v(const DecodedInst* di)
{
  execVsoxsegei8_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxsegei16_v(const DecodedInst* di)
{
  execVsoxsegei16_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxsegei32_v(const DecodedInst* di)
{
  execVsoxsegei32_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxsegei64_v(const DecodedInst* di)
{
  execVsoxsegei32_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxsegei128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsuxsegei256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsuxsegei512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsuxsegei1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVloxsegei8_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadSegIndexed<uint8_t>(di,  EW::Byte); break;
    case EW::Half:   ok = vectorLoadSegIndexed<uint16_t>(di, EW::Byte); break;
    case EW::Word:   ok = vectorLoadSegIndexed<uint32_t>(di, EW::Byte); break;
    case EW::Word2:  ok = vectorLoadSegIndexed<uint64_t>(di, EW::Byte); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVloxsegei16_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadSegIndexed<uint8_t>(di,  EW::Half); break;
    case EW::Half:   ok = vectorLoadSegIndexed<uint16_t>(di, EW::Half); break;
    case EW::Word:   ok = vectorLoadSegIndexed<uint32_t>(di, EW::Half); break;
    case EW::Word2:  ok = vectorLoadSegIndexed<uint64_t>(di, EW::Half); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVloxsegei32_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadSegIndexed<uint8_t>(di,  EW::Word); break;
    case EW::Half:   ok = vectorLoadSegIndexed<uint16_t>(di, EW::Word); break;
    case EW::Word:   ok = vectorLoadSegIndexed<uint32_t>(di, EW::Word); break;
    case EW::Word2:  ok = vectorLoadSegIndexed<uint64_t>(di, EW::Word); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVloxsegei64_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadSegIndexed<uint8_t>(di,  EW::Word2); break;
    case EW::Half:   ok = vectorLoadSegIndexed<uint16_t>(di, EW::Word2); break;
    case EW::Word:   ok = vectorLoadSegIndexed<uint32_t>(di, EW::Word2); break;
    case EW::Word2:  ok = vectorLoadSegIndexed<uint64_t>(di, EW::Word2); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVloxsegei128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVloxsegei256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVloxsegei512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVloxsegei1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsoxsegei8_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreSegIndexed<uint8_t>(di,  EW::Byte); break;
    case EW::Half:   ok = vectorStoreSegIndexed<uint16_t>(di, EW::Byte); break;
    case EW::Word:   ok = vectorStoreSegIndexed<uint32_t>(di, EW::Byte); break;
    case EW::Word2:  ok = vectorStoreSegIndexed<uint64_t>(di, EW::Byte); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsoxsegei16_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreSegIndexed<uint8_t>(di,  EW::Half); break;
    case EW::Half:   ok = vectorStoreSegIndexed<uint16_t>(di, EW::Half); break;
    case EW::Word:   ok = vectorStoreSegIndexed<uint32_t>(di, EW::Half); break;
    case EW::Word2:  ok = vectorStoreSegIndexed<uint64_t>(di, EW::Half); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsoxsegei32_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreSegIndexed<uint8_t>(di,  EW::Word); break;
    case EW::Half:   ok = vectorStoreSegIndexed<uint16_t>(di, EW::Word); break;
    case EW::Word:   ok = vectorStoreSegIndexed<uint32_t>(di, EW::Word); break;
    case EW::Word2:  ok = vectorStoreSegIndexed<uint64_t>(di, EW::Word); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsoxsegei64_v(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreSegIndexed<uint8_t>(di,  EW::Word2); break;
    case EW::Half:   ok = vectorStoreSegIndexed<uint16_t>(di, EW::Word2); break;
    case EW::Word:   ok = vectorStoreSegIndexed<uint32_t>(di, EW::Word2); break;
    case EW::Word2:  ok = vectorStoreSegIndexed<uint64_t>(di, EW::Word2); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVsoxsegei128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsoxsegei256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsoxsegei512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsoxsegei1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlsege8ff_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint8_t);
  if (not vectorLoadSeg<uint8_t>(di, ElementWidth::Byte, fieldCount, stride, true))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlsege16ff_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint16_t);
  if (not vectorLoadSeg<uint16_t>(di, ElementWidth::Half, fieldCount, stride, true))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlsege32ff_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint32_t);
  if (not vectorLoadSeg<uint32_t>(di, ElementWidth::Word, fieldCount, stride, true))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlsege64ff_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint64_t);
  if (not vectorLoadSeg<uint64_t>(di, ElementWidth::Word2, fieldCount, stride, true))
    return;
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVlsege128ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlsege256ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlsege512ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlsege1024ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


static
float minfp(float a, float b)
{
  return std::fminf(a, b);
}


static
double minfp(double a, double b)
{
  return std::fmin(a, b);
}


static
Float16 minfp(Float16 a, Float16 b)
{
  return std::fmin(a, b);
}


template <typename FT>
static FT
doFmin(FT f1, FT f2)
{
  FT res{};

  bool isNan1 = std::isnan(f1), isNan2 = std::isnan(f2);
  if (isNan1 and isNan2)
    res = std::numeric_limits<FT>::quiet_NaN();
  else if (isNan1)
    res = f2;
  else if (isNan2)
    res = f1;
  else
    res = minfp(f1, f2);  // std::fminf or std::fmin

  if (isSnan(f1) or isSnan(f2))
    raiseSimulatorFpFlags(FpFlags::Invalid);
  else if (std::signbit(f1) != std::signbit(f2) and f1 == f2)
    res = std::copysign(res, -FT{});  // Make sure min(-0, +0) is -0.

  return res;
}


static
float maxfp(float a, float b)
{
  return std::fmaxf(a, b);
}


static
double maxfp(double a, double b)
{
  return std::fmax(a, b);
}


static
Float16 maxfp(Float16 a, Float16 b)
{
  return std::fmax(a, b);
}


template <typename FT>
static FT
doFmax(FT f1, FT f2)
{
  FT res{};

  bool isNan1 = std::isnan(f1), isNan2 = std::isnan(f2);
  if (isNan1 and isNan2)
    res = std::numeric_limits<FT>::quiet_NaN();
  else if (isNan1)
    res = f2;
  else if (isNan2)
    res = f1;
  else
    res = maxfp(f1, f2);  // std::fmaxf or std::fmax

  if (isSnan(f1) or isSnan(f2))
    raiseSimulatorFpFlags(FpFlags::Invalid);
  else if (std::signbit(f1) != std::signbit(f2) and f1 == f2)
    res = std::copysign(res, FT{});  // Make sure max(-0, +0) is +0.

  return res;
}


static constexpr std::array<uint32_t, 128> frsqrt7Table = {
  52,  51,  50,  48,  47,  46,  44,  43,  42,  41,  40,  39,  38,  36,  35,  34,
  33,  32,  31,  30,  30,  29,  28,  27,  26,  25,  24,  23,  23,  22,  21,  20,
  19,  19,  18,  17,  16,  16,  15,  14,  14,  13,  12,  12,  11,  10,  10,  9,
  9,   8,   7,   7,   6,   6,   5,   4,   4,   3,   3,   2,   2,   1,   1,   0,
  127, 125, 123, 121, 119, 118, 116, 114, 113, 111, 109, 108, 106, 105, 103, 102,
  100, 99,  97,  96,  95,  93,  92,  91,  90,  88,  87,  86,  85,  84,  83,  82,
  80,  79,  78,  77,  76,  75,  74,  73,  72,  71,  70,  70,  69,  68,  67,  66,
  65,  64,  63,  63,  62,  61,  60,  59,  59,  58,  57,  56,  56,  55,  54,  53
};


// Approximate 1 / sqrt(val)
template <typename T>
static T
doFrsqrt7(T val, bool& divByZero, bool& invalid)
{
  static constexpr int bias            = std::numeric_limits<T>::max_exponent - 1;
  static constexpr int bitsOfPrecision = std::numeric_limits<T>::digits - 1;

  using uint_fsize_t = typename getSameWidthUintType<T>::type;

  divByZero = false;
  invalid = false;

  bool signBit = std::signbit(val);
  if (val == T{})
    {
      val = std::numeric_limits<T>::infinity();
      if (signBit)
	val = -val;
      divByZero = true;
    }
  else if (std::isinf(val) and not signBit)
    {
      val = T{};
    }
  else if (std::isnan(val))
    {
      if (isSnan(val))
	invalid = true;
      val = std::numeric_limits<T>::quiet_NaN();
    }
  else if (signBit)
    {
      val = std::numeric_limits<T>::quiet_NaN();
      invalid = true;
    }
  else
    {
      int inExp  = 0;
      T   inFrac = std::frexp(val, &inExp);
      inExp += bias - 1;
      uint_fsize_t u         = std::bit_cast<uint_fsize_t>(inFrac);
      int          sigMs6    = (u >> (bitsOfPrecision - 6)) & 0x3f;  // Most sig 6 bits of significand
      uint_fsize_t outExp    = (3 * bias - 1 - inExp) / 2;
      int          index     = (uint_fsize_t(inExp & 1) << 6) | sigMs6;
      uint_fsize_t outSigMs7 = frsqrt7Table[index];
      u                      = (outSigMs7 << (bitsOfPrecision - 7)) | (outExp << bitsOfPrecision);
      val                    = std::bit_cast<T>(u);
    }

  return val;
}


static constexpr std::array<uint32_t, 128> frec7Table = {
  127, 125, 123, 121, 119, 117, 116, 114, 112, 110, 109, 107, 105, 104, 102, 100, 
  99,  97,  96,  94,  93,  91,  90,  88,  87,  85,  84,  83,  81,  80,  79,  77,  
  76,  75,  74,  72,  71,  70,  69,  68,  66,  65,  64,  63,  62,  61,  60,  59,  
  58,  57,  56,  55,  54,  53,  52,  51,  50,  49,  48,  47,  46,  45,  44,  43,  
  42,  41,  40,  40,  39,  38,  37,  36,  35,  35,  34,  33,  32,  31,  31,  30,  
  29,  28,  28,  27,  26,  25,  25,  24,  23,  23,  22,  21,  21,  20,  19,  19,  
  18,  17,  17,  16,  15,  15,  14,  14,  13,  12,  12,  11,  11,  10,  9,   9,   
  8,   8,   7,   7,   6,   5,   5,   4,   4,   3,   3,   2,   2,   1,   1,   0
};

// Approximate 1 / x
template <typename T>
static T
doFrec7(T val, RoundingMode mode, FpFlags& flags)
{
  static constexpr int bias            = std::numeric_limits<T>::max_exponent - 1;
  static constexpr int bitsOfPrecision = std::numeric_limits<T>::digits - 1;

  using uint_fsize_t = typename getSameWidthUintType<T>::type;

  flags = FpFlags::None;
  bool signBit = std::signbit(val);

  if (val == T{})
    {
      val = std::numeric_limits<T>::infinity();
      if (signBit)
	val = -val;
      flags = FpFlags(unsigned(FpFlags::DivByZero) | unsigned(flags));
    }
  else if (std::isinf(val))
    {
      val = signBit? -T{} : T{};
    }
  else if (std::isnan(val))
    {
      if (isSnan(val))
	flags = FpFlags(unsigned(FpFlags::Invalid) | unsigned(flags));
      val = std::numeric_limits<T>::quiet_NaN();
    }
  else
    {
      int inExp  = 0;
      T   inFrac = std::frexp(val, &inExp);
      inExp += bias - 1;

      if (inExp < -1 or inExp > 2*bias)
	{
	  auto upDown = signBit? RoundingMode::Up : RoundingMode::Down;
	  if (mode == upDown or mode == RoundingMode::Zero)
	    {
	      val = std::numeric_limits<T>::max();
	      if (signBit)
		val = -val;
	      flags = FpFlags(unsigned(FpFlags::Inexact) | unsigned(FpFlags::Overflow) |
			      unsigned(flags));
	    }
	  else
	    {
	      val = std::numeric_limits<T>::infinity();
	      if (signBit)
		val = -val;
	      flags = FpFlags(unsigned(FpFlags::Inexact) | unsigned(FpFlags::Overflow) |
			      unsigned(flags));
	    }
	}
      else
        {
          uint_fsize_t u         = std::bit_cast<uint_fsize_t>(inFrac);
          int          sigMs7    = (u >> (bitsOfPrecision - 7)) & 0x7f;  // Most sig 7 bits of significand
          int          outExp    = (2*bias - 1 - inExp);
          uint_fsize_t outSigMs7 = static_cast<uint_fsize_t>(frec7Table[sigMs7]) << (bitsOfPrecision - 7);

          if (outExp < 1)
            {
              outSigMs7 = ((static_cast<uint_fsize_t>(1) << bitsOfPrecision) | outSigMs7) >> (1 - outExp);
              outExp    = 0;
            }

          u   = outSigMs7                                              |
                (static_cast<uint_fsize_t>(outExp) << bitsOfPrecision) |
                (static_cast<uint_fsize_t>(signBit) << (std::numeric_limits<uint_fsize_t>::digits - 1));
          val = std::bit_cast<T>(u);
        }
    }

  return val;
}


template <typename URV>
void
Hart<URV>::execVfadd_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      vop_vv<Float16>(vd, vs1, vs2, group, start, elems, masked, doFadd<Float16>);
      break;
    case EW::Word:
      vop_vv<float>  (vd, vs1, vs2, group, start, elems, masked, doFadd<float>);
      break;
    case EW::Word2:
      vop_vv<double> (vd, vs1, vs2, group, start, elems, masked, doFadd<double>);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfadd_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = doFadd(e1, e2);
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
Hart<URV>::execVfadd_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfadd_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vfadd_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vfadd_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfsub_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      vop_vv<Float16>(vd, vs1, vs2, group, start, elems, masked, doFsub<Float16>);
      break;
    case EW::Word:
      vop_vv<float>  (vd, vs1, vs2, group, start, elems, masked, doFsub<float>);
      break;
    case EW::Word2:
      vop_vv<double> (vd, vs1, vs2, group, start, elems, masked, doFsub<double>);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsub_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};
  ELEM_TYPE negE2 = - fpRegs_.read<ELEM_TYPE>(fs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = doFadd(e1, negE2);
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
Hart<URV>::execVfsub_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfsub_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vfsub_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vfsub_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfrsub_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = doFadd(e2, -e1);
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
Hart<URV>::execVfrsub_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfrsub_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vfrsub_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vfrsub_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = doFadd(e1dw, e2dw);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwadd_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true /*widen*/))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:   vfwadd_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfwadd_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  postVecFail(di); return;
    default:         postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwadd_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e1{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);
  ELEM_TYPE2X e1dw{}, dest{};
  WidenedFpScalar e2dw{e2};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
          dest = doFadd<ELEM_TYPE2X>(e1dw, e2dw);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwadd_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half: vfwadd_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word: vfwadd_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = doFadd(e1dw, -e2dw);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwsub_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:   vfwsub_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfwsub_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  postVecFail(di); return;
    default:         postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwsub_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e1{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);
  ELEM_TYPE2X e1dw{}, dest{};
  WidenedFpScalar negE2dw{-e2};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
          dest = doFadd<ELEM_TYPE2X>(e1dw, negE2dw);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwsub_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half: vfwsub_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word: vfwsub_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwadd_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group2x, e1dw) and vecRegs_.read(vs2, ix, group, e2))
        {
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = doFadd(e1dw, e2dw);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwadd_wv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0W1(di, vd, vs1, vs2, group))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:   vfwadd_wv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfwadd_wv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  postVecFail(di); return;
    default:         postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwadd_wf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);
  ELEM_TYPE2X e1dw{}, dest{};
  WidenedFpScalar e2dw{e2};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group2x, e1dw))
        {
          dest = doFadd<ELEM_TYPE2X>(e1dw, e2dw);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwadd_wf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0W1(di, vd, vs1, group))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half: vfwadd_wf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word: vfwadd_wf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwsub_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE  e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group2x, e1dw) and vecRegs_.read(vs2, ix, group, e2))
        {
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = doFadd(e1dw, -e2dw);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwsub_wv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0W1(di, vd, vs1, vs2, group))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:   vfwsub_wv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfwsub_wv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwsub_wf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);
  ELEM_TYPE2X e1dw{}, dest{};
  WidenedFpScalar negE2dw{-e2};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group2x, e1dw))
        {
          dest = doFadd<ELEM_TYPE2X>(e1dw, negE2dw);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwsub_wf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0W1(di, vd, vs1, group))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half: vfwsub_wf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word: vfwsub_wf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfmul_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      vop_vv<Float16>(vd, vs1, vs2, group, start, elems, masked, doFmul<Float16>);
      break;
    case EW::Word:
      vop_vv<float>  (vd, vs1, vs2, group, start, elems, masked, doFmul<float>);
      break;
    case EW::Word2:
      vop_vv<double> (vd, vs1, vs2, group, start, elems, masked, doFmul<double>);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmul_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = doFmul(e1, e2);
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
Hart<URV>::execVfmul_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfmul_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vfmul_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vfmul_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfdiv_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      vop_vv<Float16>(vd, vs1, vs2, group, start, elems, masked, doFdiv<Float16>);
      break;
    case EW::Word:
      vop_vv<float>  (vd, vs1, vs2, group, start, elems, masked, doFdiv<float>);
      break;
    case EW::Word2:
      vop_vv<double> (vd, vs1, vs2, group, start, elems, masked, doFdiv<double>);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfdiv_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = doFdiv(e1, e2);
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
Hart<URV>::execVfdiv_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfdiv_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vfdiv_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vfdiv_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfrdiv_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = doFdiv(e2, e1);
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
Hart<URV>::execVfrdiv_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfrdiv_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vfrdiv_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vfrdiv_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwmul_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide
  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = doFmul(e1dw, e2dw);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwmul_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  // Double wide legal. Destination register multiple of emul.
  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:   vfwmul_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfwmul_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwmul_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e1{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);
  ELEM_TYPE2X e1dw{}, dest{};
  WidenedFpScalar e2dw{e2};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
          dest = doFmul<ELEM_TYPE2X>(e1dw, e2dw);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwmul_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  // Double wide legal. Destination register multiple of emul.
  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half: vfwmul_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word: vfwmul_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group, dest))
        {
	  dest = fusedMultiplyAdd(e1, dest, e2);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }


  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfmadd_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:   vfmadd_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfmadd_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vfmadd_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess();
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmadd_vf(unsigned vd, unsigned f1, unsigned vf2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2{}, dest{};
  ELEM_TYPE e1 = fpRegs_.read<ELEM_TYPE>(f1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vf2, ix, group, e2) and vecRegs_.read(vd, ix, group, dest))
        {
          dest = fusedMultiplyAdd(e1, dest, e2);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfmadd_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vd, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfmadd_vf<Float16>(vd, f1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfmadd_vf<float>  (vd, f1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfmadd_vf<double> (vd, f1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group, dest))
        {
	  dest = fusedMultiplyAdd(-e1, dest, -e2);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }


  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfnmadd_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:   vfnmadd_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfnmadd_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vfnmadd_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess();
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmadd_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2{}, dest{};
  ELEM_TYPE e1 = fpRegs_.read<ELEM_TYPE>(f1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, group, dest))
        {
          dest = fusedMultiplyAdd(-e1, dest, -e2);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfnmadd_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vd, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfnmadd_vf<Float16>(vd, f1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfnmadd_vf<float>  (vd, f1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfnmadd_vf<double> (vd, f1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group, dest))
        {
	  dest = fusedMultiplyAdd(e1, dest, -e2);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }


  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfmsub_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:   vfmsub_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfmsub_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vfmsub_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess();
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmsub_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2{}, dest{};
  ELEM_TYPE e1 = fpRegs_.read<ELEM_TYPE>(f1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, group, dest))
        {
          dest = fusedMultiplyAdd(e1, dest, -e2);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfmsub_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vd, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfmsub_vf<Float16>(vd, f1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfmsub_vf<float>  (vd, f1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfmsub_vf<double> (vd, f1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group, dest))
        {
	  dest = fusedMultiplyAdd(-e1, dest, e2);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }


  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfnmsub_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:   vfnmsub_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfnmsub_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vfnmsub_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess();
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmsub_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2{}, dest{};
  ELEM_TYPE e1 = fpRegs_.read<ELEM_TYPE>(f1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, group, dest))
        {
          dest = fusedMultiplyAdd(-e1, dest, e2);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfnmsub_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vd, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfnmsub_vf<Float16>(vd, f1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfnmsub_vf<float>  (vd, f1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfnmsub_vf<double> (vd, f1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group, dest))
        {
	  dest = fusedMultiplyAdd(e1, e2, dest);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }


  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfmacc_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:   vfmacc_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfmacc_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vfmacc_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess();
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmacc_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2{}, dest{};
  ELEM_TYPE e1 = fpRegs_.read<ELEM_TYPE>(f1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, group, dest))
        {
          dest = fusedMultiplyAdd(e1, e2, dest);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfmacc_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  clearSimulatorFpFlags();
  setSimulatorRoundingMode(getFpRoundingMode());

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vd, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:
      if (not isZvfhLegal()) { postVecFail(di); return; }
      vfmacc_vf<Float16>(vd, f1, vs2, group, start, elems, masked);
      break;

    case EW::Word:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfmacc_vf<float>  (vd, f1, vs2, group, start, elems, masked);
      break;

    case EW::Word2:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfmacc_vf<double> (vd, f1, vs2, group, start, elems, masked);
      break;

    default: postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group, dest))
        {
	  dest = fusedMultiplyAdd(-e1, e2, -dest);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }


  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfnmacc_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:   vfnmacc_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfnmacc_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vfnmacc_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess();
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmacc_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2{}, dest{};
  ELEM_TYPE e1 = fpRegs_.read<ELEM_TYPE>(f1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, group, dest))
        {
          dest = fusedMultiplyAdd(-e1, e2, -dest);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfnmacc_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  v2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vd, v2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfnmacc_vf<Float16>(vd, f1, v2, group, start, elems, masked); break;
    case EW::Word:  vfnmacc_vf<float>  (vd, f1, v2, group, start, elems, masked); break;
    case EW::Word2: vfnmacc_vf<double> (vd, f1, v2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group, dest))
        {
	  dest = fusedMultiplyAdd(e1, e2, -dest);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }


  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfmsac_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:   vfmsac_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfmsac_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vfmsac_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess();
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmsac_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2{}, dest{};
  ELEM_TYPE e1 = fpRegs_.read<ELEM_TYPE>(f1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, group, dest))
        {
          dest = fusedMultiplyAdd(e1, e2, -dest);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfmsac_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vd, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfmsac_vf<Float16>(vd, f1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfmsac_vf<float>  (vd, f1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfmsac_vf<double> (vd, f1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group, dest))
        {
	  dest = fusedMultiplyAdd(-e1, e2, dest);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }


  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfnmsac_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:   vfnmsac_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfnmsac_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vfnmsac_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess();
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmsac_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2{}, dest{};
  ELEM_TYPE e1 = fpRegs_.read<ELEM_TYPE>(f1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, group, dest))
        {
          dest = fusedMultiplyAdd(-e1, e2, dest);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfnmsac_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vd, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfnmsac_vf<Float16>(vd, f1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfnmsac_vf<float>  (vd, f1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfnmsac_vf<double> (vd, f1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group2x, dest))
        {
	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = fusedMultiplyAdd(e1dw, e2dw, dest);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }


  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwmacc_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:   vfwmacc_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfwmacc_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess();
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwmacc_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e2{};
  ELEM_TYPE e1 = fpRegs_.read<ELEM_TYPE>(f1);
  ELEM_TYPE2X e2dw{}, dest{};
  WidenedFpScalar e1dw{e1};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, group2x, dest))
        {
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
          dest = fusedMultiplyAdd<ELEM_TYPE2X>(e1dw, e2dw, dest);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwmacc_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  fs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs2, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half: vfwmacc_vf<Float16>(vd, fs1, vs2, group, start, elems, masked); break;
    case EW::Word: vfwmacc_vf<float>  (vd, fs1, vs2, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwnmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group2x, dest))
        {
	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = fusedMultiplyAdd(-e1dw, e2dw, -dest);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }


  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwnmacc_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half: vfwnmacc_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vfwnmacc_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }
  postVecSuccess();
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwnmacc_vf(unsigned vd, unsigned fs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e2{};
  ELEM_TYPE e1 = fpRegs_.read<ELEM_TYPE>(fs1);
  ELEM_TYPE2X e2dw{}, dest{};
  WidenedFpScalar e1dw{-e1};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, group2x, dest))
        {
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
          dest = fusedMultiplyAdd<ELEM_TYPE2X>(e1dw, e2dw, -dest);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwnmacc_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  fs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs2, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half: vfwnmacc_vf<Float16>(vd, fs1, vs2, group, start, elems, masked); break;
    case EW::Word: vfwnmacc_vf<float>  (vd, fs1, vs2, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group2x, dest))
        {
	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = fusedMultiplyAdd(e1dw, e2dw, -dest);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }


  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwmsac_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:   vfwmsac_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfwmsac_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess();
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwmsac_vf(unsigned vd, unsigned fs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e2{};
  ELEM_TYPE e1 = fpRegs_.read<ELEM_TYPE>(fs1);
  ELEM_TYPE2X e2dw{}, dest{};
  WidenedFpScalar e1dw{e1};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, group2x, dest))
        {
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
          dest = fusedMultiplyAdd<ELEM_TYPE2X>(e1dw, e2dw, -dest);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwmsac_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  fs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs2, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half: vfwmsac_vf<Float16>(vd, fs1, vs2, group, start, elems, masked); break;
    case EW::Word: vfwmsac_vf<float>  (vd, fs1, vs2, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwnmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and
	  vecRegs_.read(vs2, ix, group, e2) and
	  vecRegs_.read(vd, ix, group2x, dest))
        {
	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = fusedMultiplyAdd(-e1dw, e2dw, dest);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }


  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwnmsac_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half: vfwnmsac_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word: vfwnmsac_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwnmsac_vf(unsigned vd, unsigned fs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X; // Double wide

  unsigned errors = 0;
  ELEM_TYPE e2{};
  ELEM_TYPE e1 = fpRegs_.read<ELEM_TYPE>(fs1);
  ELEM_TYPE2X e2dw{}, dest{};
  WidenedFpScalar e1dw{-e1};

  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs2, ix, group, e2) and vecRegs_.read(vd, ix, group2x, dest))
        {
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
          dest = fusedMultiplyAdd<ELEM_TYPE2X>(e1dw, e2dw, dest);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwnmsac_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  fs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs2, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half: vfwnmsac_vf<Float16>(vd, fs1, vs2, group, start, elems, masked); break;
    case EW::Word: vfwnmsac_vf<float>  (vd, fs1, vs2, group, start, elems, masked); break;
    default: postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsqrt_v(unsigned vd, unsigned vs1, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = doFsqrt(e1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfsqrt_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:  vfsqrt_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:  vfsqrt_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2: vfsqrt_v<double> (vd, vs1, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmerge(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		   unsigned start, unsigned elems)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(rs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = vecRegs_.isActive(0, ix) ? e2 : e1;
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
Hart<URV>::execVfmerge_vfm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  if (not di->isMasked() or vd == 0)
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:
      if (not isZvfhLegal()) { postVecFail(di); return; }
      vfmerge<Float16>(vd, vs1, rs2, group, start, elems);
      break;

    case EW::Word:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfmerge<float>  (vd, vs1, rs2, group, start, elems);
      break;

    case EW::Word2:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfmerge<double> (vd, vs1, rs2, group, start, elems);
      break;

    default: postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmv_v_f(unsigned vd, unsigned rs1, unsigned group,
		    unsigned start, unsigned elems)
{
  unsigned errors = 0;
  ELEM_TYPE e1{};
  e1 = fpRegs_.read<ELEM_TYPE>(rs1);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (not vecRegs_.write(vd, ix, group, e1))
	errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfmv_v_f(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  unsigned vd = di->op0(),  rs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:
      if (not isZvfhLegal()) { postVecFail(di); return; }
      vfmv_v_f<Float16>(vd, rs1, group, start, elems);
      break;

    case EW::Word:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfmv_v_f<float>  (vd, rs1, group, start, elems);
      break;

    case EW::Word2:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfmv_v_f<double> (vd, rs1, group, start, elems);
      break;

    default: postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfeq_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchMask(vd);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
	  bool flag = false;
	  if (std::isnan(e1) or std::isnan(e2))
	    {
	      if (isSnan(e1) or isSnan(e2))
		orFcsrFlags(FpFlags::Invalid);
	    }
	  else
	    flag = e1 == e2;
          if (not vecRegs_.writeMaskRegister(vd, ix, flag))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVmfeq_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecFpMaskInst(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   postVecFail(di); return;
    case EW::Half:   vmfeq_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vmfeq_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vmfeq_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfeq_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(rs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchMask(vd);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  bool flag = false;
	  if (std::isnan(e1) or std::isnan(e2))
	    {
	      if (isSnan(e1) or isSnan(e2))
		orFcsrFlags(FpFlags::Invalid);
	    }
	  else
	    flag = e1 == e2;
          if (not vecRegs_.writeMaskRegister(vd, ix, flag))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVmfeq_vf(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecFpMaskInst(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vmfeq_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vmfeq_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vmfeq_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfne_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchMask(vd);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
	  bool flag = true;
	  if (std::isnan(e1) or std::isnan(e2))
	    {
	      if (isSnan(e1) or isSnan(e2))
		orFcsrFlags(FpFlags::Invalid);
	    }
	  else
	    flag = e1 != e2;
          if (not vecRegs_.writeMaskRegister(vd, ix, flag))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVmfne_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecFpMaskInst(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   postVecFail(di); return;
    case EW::Half:   vmfne_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vmfne_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vmfne_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfne_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(rs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchMask(vd);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  bool flag = true;
	  if (std::isnan(e1) or std::isnan(e2))
	    {
	      if (isSnan(e1) or isSnan(e2))
		orFcsrFlags(FpFlags::Invalid);
	    }
	  else
	    flag = e1 != e2;
          if (not vecRegs_.writeMaskRegister(vd, ix, flag))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVmfne_vf(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecFpMaskInst(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vmfne_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vmfne_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vmfne_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmflt_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchMask(vd);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
	  bool flag = false;
	  if (std::isnan(e1) or std::isnan(e2))
	    orFcsrFlags(FpFlags::Invalid);
	  else
	    flag = e1 < e2;
          if (not vecRegs_.writeMaskRegister(vd, ix, flag))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVmflt_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecFpMaskInst(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  postVecFail(di); return;
    case EW::Half:  vmflt_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vmflt_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmflt_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmflt_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(rs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchMask(vd);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  bool flag = false;
	  if (std::isnan(e1) or std::isnan(e2))
	    orFcsrFlags(FpFlags::Invalid);
	  else
	    flag = e1 < e2;
          if (not vecRegs_.writeMaskRegister(vd, ix, flag))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVmflt_vf(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecFpMaskInst(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vmflt_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vmflt_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vmflt_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfle_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchMask(vd);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
	  bool flag = false;
	  if (std::isnan(e1) or std::isnan(e2))
	    orFcsrFlags(FpFlags::Invalid);
	  else
	    flag = e1 <= e2;
          if (not vecRegs_.writeMaskRegister(vd, ix, flag))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVmfle_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemCount(), start = csRegs_.peekVstart();

  if (not checkVecFpMaskInst(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  postVecFail(di); return;
    case EW::Half:  vmfle_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vmfle_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmfle_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfle_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(rs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchMask(vd);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  bool flag = false;
	  if (std::isnan(e1) or std::isnan(e2))
	    orFcsrFlags(FpFlags::Invalid);
	  else
	    flag = e1 <= e2;
          if (not vecRegs_.writeMaskRegister(vd, ix, flag))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVmfle_vf(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecFpMaskInst(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vmfle_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vmfle_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vmfle_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfgt_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(rs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchMask(vd);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  bool flag = false;
	  if (std::isnan(e1) or std::isnan(e2))
	    orFcsrFlags(FpFlags::Invalid);
	  else
	    flag = e1 > e2;
          if (not vecRegs_.writeMaskRegister(vd, ix, flag))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVmfgt_vf(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecFpMaskInst(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vmfgt_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vmfgt_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vmfgt_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfge_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(rs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchMask(vd);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  bool flag = false;
	  if (std::isnan(e1) or std::isnan(e2))
	    orFcsrFlags(FpFlags::Invalid);
	  else
	    flag = e1 >= e2;
          if (not vecRegs_.writeMaskRegister(vd, ix, flag))
            errors++;
        }
      else
        errors++;
    }

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVmfge_vf(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecFpMaskInst(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vmfge_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vmfge_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vmfge_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfclass_v(unsigned vd, unsigned vs1, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  typedef typename getSameWidthIntType<ELEM_TYPE>::type INT_TYPE;
	  INT_TYPE dest = fpClassifyRiscv(e1);
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
Hart<URV>::execVfclass_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Byte:   postVecFail(di); return;
    case EW::Half:   vfclass_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   vfclass_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2:  vfclass_v<double> (vd, vs1, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfcvt_xu_f_v(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  typedef typename getSameWidthUintType<ELEM_TYPE>::type UINT_TYPE;
	  UINT_TYPE dest = fpConvertTo<UINT_TYPE>(e1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();
  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfcvt_xu_f_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Byte:   postVecFail(di); return;
    case EW::Half:   vfcvt_xu_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   vfcvt_xu_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2:  vfcvt_xu_f_v<double> (vd, vs1, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfcvt_x_f_v(unsigned vd, unsigned vs1, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  typedef typename getSameWidthIntType<ELEM_TYPE>::type INT_TYPE;
	  INT_TYPE dest = fpConvertTo<INT_TYPE>(e1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();
  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfcvt_x_f_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Byte:   postVecFail(di); return;
    case EW::Half:   vfcvt_x_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   vfcvt_x_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2:  vfcvt_x_f_v<double> (vd, vs1, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfcvt_rtz_xu_f_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;
  setSimulatorRoundingMode(RoundingMode::Zero);

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
    case EW::Byte:   postVecFail(di); return;
    case EW::Half:   vfcvt_xu_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   vfcvt_xu_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2:  vfcvt_xu_f_v<double> (vd, vs1, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfcvt_rtz_x_f_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;
  setSimulatorRoundingMode(RoundingMode::Zero);

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
    case EW::Byte:   postVecFail(di); return;
    case EW::Half:   vfcvt_x_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   vfcvt_x_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2:  vfcvt_x_f_v<double> (vd, vs1, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfcvt_f_xu_v(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  typedef typename getSameWidthUintType<ELEM_TYPE>::type UINT_TYPE;

  unsigned errors = 0;
  UINT_TYPE e1{0};
  ELEM_TYPE dest{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  dest = fpConvertTo<ELEM_TYPE>(e1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();
  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfcvt_f_xu_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Byte:   postVecFail(di); return;
    case EW::Half:   vfcvt_f_xu_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   vfcvt_f_xu_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2:  vfcvt_f_xu_v<double> (vd, vs1, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfcvt_f_x_v(unsigned vd, unsigned vs1, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  typedef typename getSameWidthIntType<ELEM_TYPE>::type INT_TYPE;

  unsigned errors = 0;
  INT_TYPE e1{0};
  ELEM_TYPE dest{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  dest = fpConvertTo<ELEM_TYPE>(e1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();
  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfcvt_f_x_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Byte:   postVecFail(di); return;
    case EW::Half:   vfcvt_f_x_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   vfcvt_f_x_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2:  vfcvt_f_x_v<double> (vd, vs1, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfwcvt_xu_f_v(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{};
  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  typedef typename getSameWidthUintType<ELEM_TYPE>::type UINT_TYPE;
	  typedef typename makeDoubleWide<UINT_TYPE>::type UINT_TYPE2X;
	  UINT_TYPE2X dest = fpConvertTo<UINT_TYPE2X>(e1);

          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();
  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwcvt_xu_f_v(const DecodedInst* di)
{
  // Float to double-wide integer.
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: vfwcvt_xu_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word: vfwcvt_xu_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfwcvt_x_f_v(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{};
  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  typedef typename getSameWidthIntType<ELEM_TYPE>::type INT_TYPE;
	  typedef typename makeDoubleWide<INT_TYPE>::type INT_TYPE2X;
	  INT_TYPE2X dest = fpConvertTo<INT_TYPE2X>(e1);

          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();
  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwcvt_x_f_v(const DecodedInst* di)
{
  // Float to double-wide integer
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: vfwcvt_x_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word: vfwcvt_x_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfwcvt_rtz_xu_f_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  setSimulatorRoundingMode(RoundingMode::Zero);

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: vfwcvt_xu_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word: vfwcvt_xu_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfwcvt_rtz_x_f_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;
  setSimulatorRoundingMode(RoundingMode::Zero);

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: vfwcvt_x_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word: vfwcvt_x_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfwcvt_f_xu_v(unsigned vd, unsigned vs1, unsigned group,
			 unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X;
  typedef typename getSameWidthFloatType<ELEM_TYPE2X>::type FP_TYPE2X;

  unsigned errors = 0;
  ELEM_TYPE e1{};
  FP_TYPE2X dest{};
  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  dest = fpConvertTo<FP_TYPE2X>(e1);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();
  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwcvt_f_xu_v(const DecodedInst* di)
{
  // Unsigned to double-wide fp.
  if (not checkVecIntInst(di))
    return;

  clearSimulatorFpFlags();
  setSimulatorRoundingMode(getFpRoundingMode());

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      if (not isZvfhLegal()) { postVecFail(di); return; }
      vfwcvt_f_xu_v<uint8_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfwcvt_f_xu_v<uint16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfwcvt_f_xu_v<uint32_t>(vd, vs1, group, start, elems, masked);
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
Hart<URV>::vfwcvt_f_x_v(unsigned vd, unsigned vs1, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X;
  typedef typename getSameWidthFloatType<ELEM_TYPE2X>::type FP_TYPE2X;

  unsigned errors = 0;
  ELEM_TYPE e1{};
  FP_TYPE2X dest{};
  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  dest = fpConvertTo<FP_TYPE2X>(e1);
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();
  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwcvt_f_x_v(const DecodedInst* di)
{
  // signed to double-wide fp.
  if (not checkVecIntInst(di))
    return;

  clearSimulatorFpFlags();
  setSimulatorRoundingMode(getFpRoundingMode());

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      if (not isZvfhLegal()) { postVecFail(di); return; }
      vfwcvt_f_x_v<int8_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfwcvt_f_x_v<int16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfwcvt_f_x_v<int32_t>(vd, vs1, group, start, elems, masked);
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
Hart<URV>::vfwcvt_f_f_v(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X;

  unsigned errors = 0;
  ELEM_TYPE e1{};
  ELEM_TYPE2X dest{};
  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group2x);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = fpConvertTo<ELEM_TYPE2X, false>(e1);
          if (isSnan(dest))
            {
              dest = std::numeric_limits<ELEM_TYPE2X>::quiet_NaN();
              raiseSimulatorFpFlags(FpFlags::Invalid);
            }
          if (not vecRegs_.write(vd, ix, group2x, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();
  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwcvt_f_f_v(const DecodedInst* di)
{
  // Float to double-wide float.
  if (not checkVecFpInst(di, true, &Hart::isZvfhminLegal))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: vfwcvt_f_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word: vfwcvt_f_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfncvt_xu_f_w(unsigned vd, unsigned vs1, unsigned group,
			 unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X;
  typedef typename getSameWidthFloatType<ELEM_TYPE2X>::type FLOAT_TYPE2X;

  unsigned errors = 0;
  FLOAT_TYPE2X e1{};
  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group2x, e1))
        {
	  ELEM_TYPE dest = fpConvertTo<ELEM_TYPE>(e1);

          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();
  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfncvt_xu_f_w(const DecodedInst* di)
{
  // Double-wide float to unsigned
  if (not checkVecIntInst(di))
    return;

  clearSimulatorFpFlags();
  setSimulatorRoundingMode(getFpRoundingMode());

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      if (not isZvfhLegal()) { postVecFail(di); return; }
      vfncvt_xu_f_w<uint8_t> (vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfncvt_xu_f_w<uint16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfncvt_xu_f_w<uint32_t>(vd, vs1, group, start, elems, masked);
      break;
    default:       postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfncvt_x_f_w(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X;
  typedef typename getSameWidthFloatType<ELEM_TYPE2X>::type FLOAT_TYPE2X;

  unsigned errors = 0;
  FLOAT_TYPE2X e1{};
  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group2x, e1))
        {
	  ELEM_TYPE dest = fpConvertTo<ELEM_TYPE>(e1);

          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();
  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfncvt_x_f_w(const DecodedInst* di)
{
  // Double-wide float to int.
  if (not checkVecIntInst(di))
    return;

  clearSimulatorFpFlags();
  setSimulatorRoundingMode(getFpRoundingMode());

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      if (not isZvfhLegal()) { postVecFail(di); return; }
      vfncvt_x_f_w<int8_t> (vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfncvt_x_f_w<int16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfncvt_x_f_w<int32_t>(vd, vs1, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfncvt_rtz_xu_f_w(const DecodedInst* di)
{
  // Double-wide float to unsigned
  if (not checkVecIntInst(di))
    return;

  clearSimulatorFpFlags();
  setSimulatorRoundingMode(RoundingMode::Zero);

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      if (not isZvfhLegal()) { postVecFail(di); return; }
      vfncvt_xu_f_w<uint8_t> (vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfncvt_xu_f_w<uint16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfncvt_xu_f_w<uint32_t>(vd, vs1, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfncvt_rtz_x_f_w(const DecodedInst* di)
{
  // double-wide float to int
  if (not checkVecIntInst(di))
    return;

  clearSimulatorFpFlags();
  setSimulatorRoundingMode(RoundingMode::Zero);

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:
      if (not isZvfhLegal()) { postVecFail(di); return; }
      vfncvt_x_f_w<int8_t> (vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfncvt_x_f_w<int16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfncvt_x_f_w<int32_t>(vd, vs1, group, start, elems, masked);
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
Hart<URV>::vfncvt_f_xu_w(unsigned vd, unsigned vs1, unsigned group,
			 unsigned start, unsigned elems, bool masked)
{
  typedef typename getSameWidthFloatType<ELEM_TYPE>::type FLOAT_TYPE;
  typedef typename makeDoubleWide<ELEM_TYPE>::type UINT_TYPE2X;

  unsigned errors = 0;
  UINT_TYPE2X e1{0};
  FLOAT_TYPE dest{};
  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group2x, e1))
        {
	  dest = fpConvertTo<FLOAT_TYPE>(e1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();
  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfncvt_f_xu_w(const DecodedInst* di)
{
  // Double-wide unsigned to float
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: vfncvt_f_xu_w<uint16_t>(vd, vs1, group, start, elems, masked); break;
    case EW::Word: vfncvt_f_xu_w<uint32_t>(vd, vs1, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfncvt_f_x_w(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  typedef typename getSameWidthFloatType<ELEM_TYPE>::type FLOAT_TYPE;
  typedef typename makeDoubleWide<ELEM_TYPE>::type INT_TYPE2X;

  unsigned errors = 0;
  INT_TYPE2X e1{0};
  FLOAT_TYPE dest{};
  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group2x, e1))
        {
	  dest = fpConvertTo<FLOAT_TYPE>(e1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();
  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfncvt_f_x_w(const DecodedInst* di)
{
  // Double-wide int to float
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: vfncvt_f_x_w<int16_t>(vd, vs1, group, start, elems, masked); break;
    case EW::Word: vfncvt_f_x_w<int32_t> (vd, vs1, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfncvt_f_f_w(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X;

  unsigned errors = 0;
  ELEM_TYPE2X e1{};
  ELEM_TYPE dest{};
  unsigned group2x = group*2;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group2x, e1))
        {
	  dest = fpConvertTo<ELEM_TYPE, false>(e1);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();
  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfncvt_f_f_w(const DecodedInst* di)
{
  // Double-wide float to float.
  if (not checkVecFpInst(di, true, &Hart::isZvfhminLegal))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   postVecFail(di); return;
    case EW::Half:   vfncvt_f_f_w<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   vfncvt_f_f_w<float>  (vd, vs1, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfncvt_rod_f_f_w(const DecodedInst* di)
{
  // Double-wide float to float.

  if (not checkVecFpInst(di))
    return;

#ifdef SOFT_FLOAT
  softfloat_roundingMode = softfloat_round_odd;
  // TBD FIX: what if not using SOFT_FLOAT
#endif

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: vfncvt_f_f_w<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word: vfncvt_f_f_w<float>  (vd, vs1, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfredsum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2{};
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, e2))
    errors++;

  ELEM_TYPE e1{}, result{e2};

  bool anyActive = false;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
        continue;

      anyActive = true;
      if (vecRegs_.read(vs1, ix, group, e1))
        result = doFadd(result, e1);
      else
        errors++;
    }

  // Note: NaN canonicalization when there are no active elements
  // is only allowed for vfredusum.vs and NOT for vfredosum.vs,
  // vfredmin.vs, and vfredmax.vs.
  if (not anyActive and std::isnan(result))
    result = std::numeric_limits<decltype(result)>::quiet_NaN();

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfredsum_vs(const DecodedInst* di)
{
  if (not checkFpSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  postVecFail(di); return;
    case EW::Half:  vfredsum_vs<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfredsum_vs<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfredsum_vs<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfredosum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2{};
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, e2))
    errors++;
  
  ELEM_TYPE e1{}, result{e2};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.read(vs1, ix, group, e1))
	result = doFadd(result, e1);
      else
	errors++;
    }

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfredosum_vs(const DecodedInst* di)
{
  if (not checkFpSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  postVecFail(di); return;
    case EW::Half:  vfredosum_vs<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfredosum_vs<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfredosum_vs<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfredmin_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2{};
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, e2))
    errors++;

  ELEM_TYPE e1{}, result{e2};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.read(vs1, ix, group, e1))
        result = doFmin(result, e1);
      else
        errors++;
    }

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfredmin_vs(const DecodedInst* di)
{
  if (not checkFpSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  postVecFail(di); return;
    case EW::Half:  vfredmin_vs<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfredmin_vs<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfredmin_vs<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }

  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfredmax_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e2{};
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, e2))
    errors++;

  ELEM_TYPE e1{}, result{e2};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.read(vs1, ix, group, e1))
        result = doFmax(result, e1);
      else
        errors++;
    }

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfredmax_vs(const DecodedInst* di)
{
  if (not checkFpSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  postVecFail(di); return;
    case EW::Half:  vfredmax_vs<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfredmax_vs<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfredmax_vs<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }

  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfwredsum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X;

  unsigned errors = 0;
  ELEM_TYPE2X result{};
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  ELEM_TYPE e1{};

  bool anyActive = false;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
        continue;

      anyActive = true;
      if (vecRegs_.read(vs1, ix, group, e1))
        {
          ELEM_TYPE2X e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
          result           = doFadd(result, e1dw);
        }
      else
        errors++;
    }

  // Note: NaN canonicalization when there are no active elements
  // is only allowed for vfwredusum.vs and NOT for vfwredosum.vs.
  if (not anyActive and std::isnan(result))
    result = std::numeric_limits<decltype(result)>::quiet_NaN();

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwredsum_vs(const DecodedInst* di)
{
  if (not checkFpSewLmulVstart(di, true))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  postVecFail(di); return;
    case EW::Half:  vfwredsum_vs<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfwredsum_vs<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  postVecFail(di); return;
    default:  postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfwredosum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
			 unsigned start, unsigned elems, bool masked)
{
  typedef typename makeDoubleWide<ELEM_TYPE>::type ELEM_TYPE2X;

  unsigned errors = 0;
  ELEM_TYPE2X result{};
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  if (not vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result))
    errors++;
  
  ELEM_TYPE e1{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.read(vs1, ix, group, e1))
	{
	  ELEM_TYPE2X e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  result = doFadd(result, e1dw);
	}
      else
	errors++;
    }

  if (not vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result))
    errors++;

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfwredosum_vs(const DecodedInst* di)
{
  if (not checkFpSewLmulVstart(di, true))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di, vs1, group, start))
    return;
  if (elems == 0)
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:  postVecFail(di); return;
    case EW::Half:  vfwredosum_vs<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfwredosum_vs<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  postVecFail(di); return;
    default:  postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfrsqrt7_v(unsigned vd, unsigned vs1, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};

  bool inv = false, dbz = false;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  bool edbz = false, einv = false;  // Element divide-by-zero and invalid
	  dest = doFrsqrt7(e1, edbz, einv);
	  dbz = dbz or edbz;
	  inv = inv or einv;
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  clearSimulatorFpFlags();

  if (inv) raiseSimulatorFpFlags(FpFlags::Invalid);
  if (dbz) raiseSimulatorFpFlags(FpFlags::DivByZero);

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfrsqrt7_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:  vfrsqrt7_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:  vfrsqrt7_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2: vfrsqrt7_v<double> (vd, vs1, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfrec7_v(unsigned vd, unsigned vs1, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};

  FpFlags flags = FpFlags::None;
  auto mode = getFpRoundingMode();

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  FpFlags elemFlags = FpFlags::None;
	  dest = doFrec7(e1, mode, elemFlags);
	  flags = FpFlags(unsigned(flags) | unsigned(elemFlags));
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  orFcsrFlags(flags);

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfrec7_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:  vfrec7_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:  vfrec7_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2: vfrec7_v<double> (vd, vs1, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmin_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, e2{}, dest{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = doFmin(e1, e2);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfmin_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Byte:   postVecFail(di); return;
    case EW::Half:   vfmin_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vfmin_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vfmin_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:  postVecFail(di); return;
    }

  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmin_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = doFmin(e1, e2);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfmin_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfmin_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vfmin_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vfmin_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmax_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, e2{}, dest{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = doFmax(e1, e2);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfmax_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Byte:  postVecFail(di); return;
    case EW::Half:  vfmax_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfmax_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfmax_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }

  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmax_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = doFmax(e1, e2);
          if (not vecRegs_.write(vd, ix, group, dest))
            errors++;
        }
      else
        errors++;
    }

  updateAccruedFpBits();

  assert(errors == 0);
}


template <typename URV>
void
Hart<URV>::execVfmax_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfmax_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vfmax_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vfmax_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }

  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsgnj_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, e2{}, dest{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = std::copysign(e1, e2);
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
Hart<URV>::execVfsgnj_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Byte:  postVecFail(di); return;
    case EW::Half:  vfsgnj_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfsgnj_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfsgnj_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }

  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsgnj_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = std::copysign(e1, e2);
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
Hart<URV>::execVfsgnj_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfsgnj_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vfsgnj_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vfsgnj_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }

  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsgnjn_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, e2{}, dest{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
          dest = std::copysign(e1, e2);
	  dest = -dest;
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
Hart<URV>::execVfsgnjn_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Byte:  postVecFail(di); return;
    case EW::Half:  vfsgnjn_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfsgnjn_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfsgnjn_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }

  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsgnjn_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
          dest = std::copysign(e1, e2);
	  dest = -dest;
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
Hart<URV>::execVfsgnjn_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfsgnjn_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vfsgnjn_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vfsgnjn_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }

  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsgnjx_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, e2{}, dest{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1) and vecRegs_.read(vs2, ix, group, e2))
        {
	  int s1 = (std::signbit(e1) == 0) ? 0 : 1;
	  int s2 = (std::signbit(e2) == 0) ? 0 : 1;
	  int sign = s1 ^ s2;
	  ELEM_TYPE x{};
	  if (sign)
	    x = -x;
	  dest = std::copysign(e1, x);
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
Hart<URV>::execVfsgnjx_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Byte:  postVecFail(di); return;
    case EW::Half:  vfsgnjx_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfsgnjx_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfsgnjx_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }

  postVecSuccess();
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsgnjx_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  unsigned errors = 0;
  ELEM_TYPE e1{}, dest{};
  ELEM_TYPE e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	{
	  vecRegs_.touchReg(vd, group);
	  continue;
	}

      if (vecRegs_.read(vs1, ix, group, e1))
        {
	  int s1 = (std::signbit(e1) == 0) ? 0 : 1;
	  int s2 = (std::signbit(e2) == 0) ? 0 : 1;
	  int sign = s1 ^ s2;
	  ELEM_TYPE x{};
	  if (sign)
	    x = -x;
	  dest = std::copysign(e1, x);

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
Hart<URV>::execVfsgnjx_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:  vfsgnjx_vf<Float16>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vfsgnjx_vf<float>  (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vfsgnjx_vf<double> (vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }

  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfncvtbf16_f_f_w(const DecodedInst* di)
{
  // Double-wide float to float.
  if (not checkVecFpInst(di, true, &Hart::isZvfbfminLegal))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW1(di, vd, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte:   postVecFail(di); return;
    case EW::Half:   vfncvt_f_f_w<BFloat16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   postVecFail(di); return;
    default:         postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfwcvtbf16_f_f_v(const DecodedInst* di)
{
  // Float to double-wide float.
  if (not checkVecFpInst(di, true, &Hart::isZvfbfminLegal))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs1, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: vfwcvt_f_f_v<BFloat16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word: postVecFail(di); return;
    default:       postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfwmaccbf16_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true, &Hart::isZvfbfwmaLegal))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half:   vfwmacc_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess();
}


template <typename URV>
void
Hart<URV>::execVfwmaccbf16_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true, &Hart::isZvfbfwmaLegal))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  fs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmulW0(di, vd, vs2, vs2, group))
    return;

  typedef ElementWidth EW;
  switch (sew)
    {
    case EW::Half: vfwmacc_vf<BFloat16>(vd, fs1, vs2, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }
  postVecSuccess();
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
