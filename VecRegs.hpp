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

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstddef>
#include <optional>
#include <vector>
#include <unordered_map>
#include <string_view>
#include <cassert>
#include <stdexcept>
#include "FpRegs.hpp"
#include "float-util.hpp"
#include "VecLdStInfo.hpp"

namespace WdRiscv
{

  /// Values of VS field in mstatus.
  using VecStatus = FpStatus;

  enum class GroupMultiplier : uint32_t
    {
     One      = 0,
     Two      = 1,
     Four     = 2,
     Eight    = 3,
     Reserved = 4,
     Eighth   = 5,
     Quarter  = 6,
     Half     = 7
    };


  /// Selected element width.
  enum class ElementWidth : uint32_t
    {
     Byte      = 0,
     Half      = 1,
     Word      = 2,
     Word2     = 3,
     Word4     = 4,
     Word8     = 5,
     Word16    = 6,
     Word32    = 7
    };


  enum class VecRoundingMode
    {
     NearestUp   = 0,
     NearestEven = 1,
     Down        = 2,
     Odd         = 3,
     VcsrMask    = 6,  // Mask of rounding mode bits in VCSR
     VcsrShift   = 1   // Index of least-sig rounding mode bit in VCSR
    };


  enum class VecEnums : uint32_t
    {
     GroupLimit = 8, // One past largest VecGroupMultiplier value
     WidthLimit = 8  // One past largest ElementWidth value
    };


  /// Symbolic names of the integer registers.
  enum VecRegNumber
    {
     RegV0 = 0,
     RegV1 = 1,
     RegV2 = 2,
     RegV3 = 3,
     RegV4 = 4,
     RegV5 = 5,
     RegV6 = 6,
     RegV7 = 7,
     RegV8 = 8,
     RegV9 = 9,
     RegV10 = 10,
     RegV11 = 11,
     RegV12 = 12,
     RegV13 = 13,
     RegV14 = 14,
     RegV15 = 15,
     RegV16 = 16,
     RegV17 = 17,
     RegV18 = 18,
     RegV19 = 19,
     RegV20 = 20,
     RegV21 = 21,
     RegV22 = 22,
     RegV23 = 23,
     RegV24 = 24,
     RegV25 = 25,
     RegV26 = 26,
     RegV27 = 27,
     RegV28 = 28,
     RegV29 = 29,
     RegV30 = 30,
     RegV31 = 31
    };


  template <typename URV>
  class Hart;

  /// Model a RISCV vector register file.
  class VecRegs
  {
  public:

    friend class Hart<uint32_t>;
    friend class Hart<uint64_t>;

    /// Constructor: Define an empty vector register file which may be
    /// reconfigured later using the config method.
    VecRegs();

    /// Destructor.
    ~VecRegs();

    /// Return count of vector registers. This is independent of group
    /// multiplier.
    uint32_t registerCount() const
    { return regCount_; }

    /// Return the number of bytes per vector register. This is
    /// independent of group multiplier.
    uint32_t bytesPerRegister() const
    { return bytesPerReg_; }

    /// Return the number of bytes in this register file.
    uint32_t bytesInRegisterFile() const
    { return bytesInRegFile_; }

    /// Return true if given elemIx is valid for the given register
    /// number, group multiplier, and element size.
    bool isValidIndex(uint32_t regNum, uint64_t elemIx, unsigned groupX8,
		      size_t elemSize) const
    {
      if (regNum >= regCount_ or elemIx >= bytesInRegFile_)
	return false;
      if (elemIx*elemSize > ((bytesPerReg_*groupX8) >> 3) - elemSize)
        return false;
      std::size_t regOffset = static_cast<std::size_t>(regNum)*bytesPerReg_;
      return regOffset + elemIx*elemSize <= bytesInRegFile_ - elemSize;
    }

    /// Set value to that of the element with given index within the
    /// vector register of the given number. Throw an exception if the
    /// combination of element index, vector number and group
    /// multiplier (pre-scaled by 8) is invalid. We require a
    /// pre-scaled group multiplier to avoid passing a fraction.
    template<typename T>
    void read(uint32_t regNum, uint64_t elemIx, uint32_t groupX8, T& value) const
    {
      if (not isValidIndex(regNum, elemIx, groupX8, sizeof(T)))
        throw std::runtime_error("invalid vector register index");
      std::size_t regOffset = static_cast<std::size_t>(regNum)*bytesPerReg_;
      const T* data = reinterpret_cast<const T*>(data_.data() + regOffset);
      value = data[elemIx];
    }

    /// Set the element with given index within the vector register of
    /// the given number to the given value. Throw an exception
    /// if the combination of element index, vector number and group
    /// multiplier (pre-scaled by 8) is invalid. We require a
    /// pre-scaled multiplier to avoid passing a fraction.
    template<typename T>
    void write(uint32_t regNum, uint64_t elemIx, uint32_t groupX8, const T& value)
    {
      if (not isValidIndex(regNum, elemIx, groupX8, sizeof(T)))
        throw std::runtime_error("invalid vector register index");
      std::size_t regOffset = static_cast<std::size_t>(regNum)*bytesPerReg_;
      T* data = reinterpret_cast<T*>(data_.data() + regOffset);
      data[elemIx] = value;
      lastWrittenReg_ = regNum;
      lastGroupX8_ = groupX8;
    }

    /// Return true if the combination regNum, elemIx, eew, and
    /// groupX8 is valid setting stride to the value of the designated
    /// register element. Return false otherwise leaving stride
    /// unmodified.
    bool readStride(uint32_t regNum, uint32_t elemIx, ElementWidth eew,
		    uint32_t groupX8, uint64_t& stride) const
    {
      switch(eew)
        {
        case ElementWidth::Byte:
          {
            uint8_t temp = 0;
            read(regNum, elemIx, groupX8, temp);
            stride = temp;
	    return true;
          }
        case ElementWidth::Half:
          {
            uint16_t temp = 0;
            read(regNum, elemIx, groupX8, temp);
            stride = temp;
            return true;
          }
        case ElementWidth::Word:
          {
            uint32_t temp = 0;
	    read(regNum, elemIx, groupX8, temp);
            stride = temp;
            return true;
          }
        case ElementWidth::Word2:
          {
            uint64_t temp = 0;
	    read(regNum, elemIx, groupX8, temp);
            stride = temp;
            return true;
          }
        default:
          return false;
        }
      return false;
    }

    /// Return the count of registers in this register file.
    size_t size() const
    { return regCount_; }

    /// Return the number of bits in a register in this register file.
    uint32_t bitsPerRegister() const
    { return 8*bytesPerReg_; }

    /// Return the currently configured element width.
    ElementWidth elemWidth() const
    { return sew_; }

    /// Return the currently configured group multiplier.
    GroupMultiplier groupMultiplier() const
    { return group_; }

    /// Return the currently configured element width in bits (for
    /// example if SEW is Byte, then this returns 8).
    uint32_t elemWidthInBits() const
    { return sewInBits_; }

    /// Return the width in bits corresponding to the given symbolic
    /// element width. Return 0 if symbolic value is out of bounds.
    static uint32_t elemWidthInBits(ElementWidth ew)
    { return ew > ElementWidth::Word32 ? 0 : uint32_t(8) << uint32_t(ew); }

    /// Return the currently configured group multiplier as a unsigned
    /// integer scaled by 8. For example if group multiplier is One,
    /// this returns 8. If group multiplier is Eigth, this returns 1.
    /// We pre-scale by 8 to avoid division when the multiplier is a
    /// fraction.
    uint32_t groupMultiplierX8() const
    { return groupX8_; }

    /// Return true if double the given element-width/group-multiplier
    /// is legal (multiplier is pre-scaled by 8). We check if the
    /// combination sew*2 and groupX8*2 is legal. Sets dsew to double
    /// the element width (eew) if legal.
    bool isDoubleWideLegal(ElementWidth sew, ElementWidth& dsew, uint32_t groupX8) const
    {
      uint32_t wideGroup = groupX8 * 2;
      GroupMultiplier emul = GroupMultiplier::One;
      if (not groupNumberX8ToSymbol(wideGroup, emul))
        return false;

      if (not doubleSew(sew, dsew))
        return false;

      return legalConfig(dsew, emul);
    }

    /// Return true if double the given element-width/group-multiplier
    /// is legal (multiplier is pre-scaled by 8). We check if the
    /// combination sew*2 and groupX8*2 is legal.
    bool isDoubleWideLegal(ElementWidth sew, uint32_t groupX8) const
    {
      ElementWidth eew = sew;
      return isDoubleWideLegal(sew, eew, groupX8);
    }

    /// Set ix to the number of the register corresponding to the
    /// given vector register name returning true on success and false
    /// if no such register. For example, if name is "v2" then ix
    /// will be set to 2.
    static bool findReg(std::string_view name, unsigned& ix);

    /// Return a reference to the vector ld/st element information. If last executed
    /// instruction was not ld/st or if it did not update any element then the returned
    /// reference will be to an empty vector. Set elemSize to the element size of the
    /// last ld/st instruction or 0 if last instruction was not ld/st.
    const std::vector<VecLdStInfo>& getLastMemory(unsigned& elemSize) const;

    /// Return true if the given element width and grouping
    /// combination is legal.
    bool legalConfig(ElementWidth ew, GroupMultiplier mul) const
    {
      if (size_t(ew) >= legalConfigs_.size()) return false;
      const auto& groupFlags = legalConfigs_.at(size_t(ew));
      if (size_t(mul) >= groupFlags.size()) return false;
      return groupFlags.at(size_t(mul));
    }

    /// Return the smallest element size in bytes supported by
    /// this vector register file. This may change from run to
    /// run based on the configuration.
    unsigned minElementSizeInBytes() const
    { return minBytesPerElem_; }

    /// Set symbol to the symbolic value of the given numeric group
    /// multiplier (pre-scaled by 8). Return true on success and
    /// false if groupX8 is out of bounds.
    static constexpr
    bool groupNumberX8ToSymbol(uint32_t groupX8, GroupMultiplier& symbol)
    {
      if (groupX8 == 1)  { symbol = GroupMultiplier::Eighth;   return true; }
      if (groupX8 == 2)  { symbol = GroupMultiplier::Quarter;  return true; }
      if (groupX8 == 4)  { symbol = GroupMultiplier::Half;     return true; }
      if (groupX8 == 8)  { symbol = GroupMultiplier::One;      return true; }
      if (groupX8 == 16) { symbol = GroupMultiplier::Two;      return true; }
      if (groupX8 == 32) { symbol = GroupMultiplier::Four;     return true; }
      if (groupX8 == 64) { symbol = GroupMultiplier::Eight;    return true; }
      return false;
    }

    /// Set dsew to the double of the given sew returning true on
    /// success and false if double the given sew is out of bounds.
    static constexpr
    bool doubleSew(ElementWidth sew, ElementWidth& dsew)
    {
      using EW = ElementWidth;
      if (sew == EW::Byte   ) { dsew = EW:: Half;   return true; }
      if (sew == EW::Half   ) { dsew = EW:: Word;   return true; }
      if (sew == EW::Word   ) { dsew = EW:: Word2;  return true; }
      if (sew == EW::Word2  ) { dsew = EW:: Word4;  return true; }
      if (sew == EW::Word4  ) { dsew = EW:: Word8;  return true; }
      if (sew == EW::Word8  ) { dsew = EW:: Word16; return true; }
      if (sew == EW::Word16 ) { dsew = EW:: Word32; return true; }
      return false;
    }

    /// Convert the given symbolic element width to a byte count.
    static constexpr uint32_t elemWidthInBytes(ElementWidth sew)
    { return uint32_t(1) << uint32_t(sew); }

    /// Convert the given symbolic group multiplier to a number scaled by
    /// eight (e.g. One is converted to 8, and Eigth to 1). Return 0 if
    /// given symbolic multiplier is not valid.
    static constexpr uint32_t groupMultiplierX8(GroupMultiplier gm)
    {
      if (gm < GroupMultiplier::Reserved)
        return 8*(uint32_t(1) << uint32_t(gm));
      if (gm > GroupMultiplier::Reserved and gm <= GroupMultiplier::Half)
        return 8 >> (8 - uint32_t(gm));
      return 0;
    }

    /// Return the element count in a register group defined by the
    /// currently configured group multiplier and element width.
    uint32_t vlmax() const
    {
      return groupX8_*bytesPerReg_/sewInBits_;
    }

    /// Return the element count in a register group defined by the
    /// given group multiplier and element width.
    uint32_t vlmax(GroupMultiplier gm, ElementWidth eew) const
    {
      uint32_t gm8 = groupMultiplierX8(gm);
      uint32_t eewInBits = elemWidthInBits(eew);
      return gm8*bytesPerReg_/eewInBits;
    }

    /// Return the number of elements in a vector register given an EEW.
    uint32_t singleMax(ElementWidth eew) const
    {
      uint32_t eewInBits = elemWidthInBits(eew);
      return 8*bytesPerReg_/eewInBits;
    }

    /// Return the maximum of the VLMAX and VLEN/EEW for tail elements when LMUL < 1.
    uint32_t elemMax(ElementWidth eew) const
    { return std::max(vlmax(), singleMax(eew)); }

    /// Return the maximum of the VLMAX and VLEN/SEW for tail elements when LMUL < 1.
    uint32_t elemMax() const
    { return elemMax(sew_); }

    /// Return true if tail-agnostic is set.
    bool isTailAgnostic() const
    { return tailAgn_; }

    /// Set tail-agnostic to the given flag.
    void setTailAgnostic(bool flag)
    { tailAgn_ = flag; }

    /// Return true if tail-agnostic-policy is set to all-ones.
    bool isTailAgnosticOnes() const
    { return tailAgnOnes_; }

    /// Return true if mask-agnostic is set.
    bool isMaskAgnostic() const
    { return maskAgn_; }

    /// Return true if mask-agnostic-policy is set to all-ones.
    bool isMaskAgnosticOnes() const
    { return maskAgnOnes_; }

    /// Return true if mask-producing instructions should update the whole destination
    /// register.
    bool updateWholeMask() const
    { return updateWholeMask_; }

    /// Return true if mask-producing instructions should update the whole destination
    /// register.
    void configUpdateWholeMask(bool flag)
    { updateWholeMask_ = flag; }

    /// If flag is true, configure vector engine for writing ones in
    /// inactive destination register elements when mask-agnostic is
    /// on. Otherwise, preserve inactive elements.
    void configMaskAgnosticAllOnes(bool flag)
    { maskAgnOnes_ = flag; }

    /// If flag is true, configure vector engine for writing ones in
    /// tail destination register elements when tail-agnostic is
    /// on. Otherwise, preserve tail elements.
    void configTailAgnosticAllOnes(bool flag)
    { tailAgnOnes_ = flag; }

    /// When flag is true, trap on invalid/unsuported vtype configuraions in vsetvl,
    /// vsetvli, vsetivli. When flag is false, set vtype.vill instead.
    void configVectorTrapVtype(bool flag)
    { trapVtype_ = flag; }

    /// If flag is true, unordered fp reduction should use a reduction tree computation,
    /// else uses ordered version.
    void configVectorFpUnorderedSumRed(ElementWidth ew, bool flag)
    { fpUnorderedSumTreeRed_.at(uint32_t(ew)) = flag; }

    /// When flag is true, when VL > VLMAX reduce AVL to match VLMAX and write
    /// to VL. This only applies to vsetvl instructions.
    void configVectorLegalizeVsetvlAvl(bool flag)
    { legalizeVsetvlAvl_ = flag; }

    /// When flag is true, when VL > VLMAX reduce AVL to match VLMAX and write
    /// to VL. This only applies to vsetvli instructions.
    void configVectorLegalizeVsetvliAvl(bool flag)
    { legalizeVsetvliAvl_ = flag; }

    /// If flag is true, make VL/VSTART value a multiple of EGS in vector-crypto
    /// instructions that have EGS. Otherwise, trigger an exceptio if VL/VSTART is not a
    /// mulitple of EGS for such instrucions.
    void configLegalizeForEgs(bool flag)
    { legalizeForEgs_ = flag; }

    /// Return true if elems/vstart is a multiple of EGS or if it is legalized to be a
    /// multiple of egs. Return false if legalization is not enabled and elems/vstart is
    /// not a multiple of EGS. This is for some vector-crypto instructions.
    bool validateForEgs(unsigned egs, unsigned& vl, unsigned& vstart) const
    {
      if (legalizeForEgs_)
	{
	  if ((vl % egs) != 0)
	    vl = vl - (vl % egs);

	  if ((vstart % egs) != 0)
	    vstart = vstart - (vstart % egs);
	  return true;
	}

      return (vl % egs) == 0  and  (vstart % egs) == 0;
    }

    /// Return a string representation of the given group multiplier.
    static constexpr std::string_view to_string(GroupMultiplier group)
    {
      using namespace std::string_view_literals;

      constexpr auto vec =
        std::array{"m1"sv, "m2"sv, "m4"sv, "m8"sv, "m?"sv, "mf8"sv, "mf4"sv, "mf2"sv};
      return size_t(group) < vec.size()? vec.at(size_t(group)) : "m?";
    }

    /// Return a string representation of the given element width.
    static constexpr std::string_view to_string(ElementWidth ew)
    {
      using namespace std::string_view_literals;

      constexpr auto vec =
        std::array{"e8"sv, "e16"sv, "e32"sv, "e64"sv, "e128"sv, "e256"sv, "e512"sv, "e1024"sv};
      return size_t(ew) < vec.size()? vec.at(size_t(ew)) : "e?";
    }

    /// Convert given string to a group multiplier returning true on
    /// success and false if given string does not contain a valid
    /// group multiplier representation.
    static bool to_lmul(std::string_view lmul, GroupMultiplier& group)
    {
      static const std::unordered_map<std::string_view, GroupMultiplier> map(
        { {"m1", GroupMultiplier::One}, {"m2", GroupMultiplier::Two},
          {"m4", GroupMultiplier::Four}, {"m8", GroupMultiplier::Eight},
          {"m?", GroupMultiplier::Reserved}, {"mf8", GroupMultiplier::Eighth},
          {"mf4", GroupMultiplier::Quarter}, {"mf2", GroupMultiplier::Half} });

      auto lmul_it = map.find(lmul);
      if (lmul_it != map.end())
        {
          group = lmul_it->second;
          return true;
        }
      return false;
    }

    /// Convert given string to an element width returning true on
    /// success and false if given string does not contain a valid
    /// element width.  Example: "e32" yields ElementWidth::Word.
    static bool to_sew(std::string_view sew, ElementWidth& ew)
    {
      static const std::unordered_map<std::string_view, ElementWidth> map(
        { {"e8", ElementWidth::Byte}, {"e16", ElementWidth::Half},
          {"e32", ElementWidth::Word}, {"e64", ElementWidth::Word2},
          {"e128", ElementWidth::Word4}, {"e256", ElementWidth::Word8},
          {"e512", ElementWidth::Word16}, {"e1024", ElementWidth::Word32} });

      auto sew_it = map.find(sew);
      if (sew_it != map.end())
        {
          ew = sew_it->second;
          return true;
        }
      return false;
    }

    /// Information about ld/st instruction. Cleared before each instruction is executed.
    const std::vector<VecLdStInfo>& ldStInfo() const
    { return ldStInfo_; }

    struct Step
    {
      enum Operation { CrossRegRed, AdjacRed, StrideRed, ScalarRed, None };

      template <typename ET, typename RT>
      Step(Operation op, ET e1, ET e2, RT res)
        : op_(op)
      {
        using uint_fsize_et = typename getSameWidthUintType<ET>::type;
        using uint_fsize_rt = typename getSameWidthUintType<RT>::type;

        operands_.at(0) = std::bit_cast<uint_fsize_et>(e1);
        operands_.at(1) = std::bit_cast<uint_fsize_et>(e2);
        result_ = std::bit_cast<uint_fsize_rt>(res);
      }

      static constexpr std::string_view opToStr(Operation op)
      {
        using namespace std::string_view_literals;

        constexpr auto vec =
          std::array{"group-wise"sv, "adjacent"sv, "stride"sv, "scalar"sv, "none"sv};

        return vec.at(op);
      }

      Operation op_;
      std::array<uint64_t, 2> operands_;
      uint64_t result_;
    };

    void lastIncVec(std::vector<uint8_t>& fpFlags, std::vector<uint8_t>& vxsat,
                    std::vector<Step>& steps) const
    {
      fpFlags = fpFlags_;
      vxsat = vxsat_;
      steps = steps_;
    }

    /// Ruturn the effective group multiplier of the given operand.
    unsigned getOpEmul(unsigned op) const
    { return op < 3? opsEmul_.at(op) : 0; }

    /// Set size to the elem size in byte and the count used in the last load/store
    /// instruction. Return true on success. Return false if last exectued instruction was
    /// not load/store.
    bool vecLdStElemsUsed(unsigned& size, unsigned& count) const
    {
      size = ldStSize_;
      count = ldStInfo_.size();
      return ldStSize_ != 0;
    }


  protected:

    /// Clear load/address and store data used for logging/tracing.
    void clearTraceData()
    {
      ldStSize_ = 0;
      ldStInfo_.clear();
      fpFlags_.clear();
      vxsat_.clear();
      steps_.clear();
      clearLastWrittenReg();
      setOpEmul(1, 1, 1);
    }

    /// Clear the number denoting the last written register.
    void clearLastWrittenReg()
    { lastWrittenReg_.reset(); }

    /// Return the number of the last written vector regsiter or -1 if no
    /// no register has been written since the last clearLastWrittenReg.
    int getLastWrittenReg(uint32_t& groupX8) const
    {
      if (lastWrittenReg_.has_value())
        {
          groupX8 = lastGroupX8_;
          return static_cast<int>(*lastWrittenReg_);
        }
      return -1;
    }

    /// Return the number of the last written vector register or -1 if no
    /// no register has been written since the last clearLastWrittenReg.
    int getLastWrittenReg() const
    { return lastWrittenReg_.has_value() ? static_cast<int>(*lastWrittenReg_) : -1; }

    /// Set effective group multipliers of the operands of a vector
    /// instruction (this is used to record logging information).
    void setOpEmul(unsigned emul0, unsigned emul1 = 1, unsigned emul2 = 1)
    { opsEmul_.at(0) = emul0; opsEmul_.at(1) = emul1; opsEmul_.at(2) = emul2; }

    /// For instructions that do not use the write method, mark the
    /// last written register and the effective group multiplier.
    void touchReg(uint32_t reg, uint32_t groupX8)
    { lastWrittenReg_ = reg; lastGroupX8_ = groupX8; }

    /// Same as above for mask registers
    void touchMask(uint32_t reg)
    { touchReg(reg, 8); }  // Grouping of of 1

    /// Return true if element of given index is active with respect
    /// to the given mask vector register. Element is active if the
    /// corresponding mask bit is 1.
    bool isActive(uint32_t maskReg, uint32_t ix) const
    {
      if (maskReg >= regCount_)
        return false;

      uint32_t byteIx = ix >> 3;
      uint32_t bitIx = ix & 7;  // bit in byte
      if (byteIx >= bytesPerReg_)
        return false;

      const uint8_t* data = data_.data() + static_cast<std::size_t>(maskReg)*bytesPerReg_;
      return (data[byteIx] >> bitIx) & 1;
    }

    /// Return true if element at index ix of vector destination is active;
    /// otherwise, return false. Set val to the current value of the element if
    /// active or the expected new value if inactive (either current value or
    /// all ones depending on the agnostic policy).
    template<typename T>
    bool isDestActive(unsigned vd, unsigned ix, unsigned emulx8, bool masked, T& val) const
    {
      read(vd, ix, emulx8, val);

      if (ix >= elemCount())
	{
	  if (tailAgn_ and tailAgnOnes_)
	    setAllBits(val);
	  return false;
	}
      if (masked and not isActive(0, ix))
	{
	  if (maskAgn_ and maskAgnOnes_)
	    setAllBits(val);
	  return false;
	}

      return true;
    }

    /// Return true if element at ix of mask destination is active; otherwise,
    /// return false and either read element at ix into val or set val to all
    /// ones depending on the mask-agnostic/tail-agnostic policy.
    bool isMaskDestActive(unsigned vd, unsigned ix, bool masked, unsigned nelems,
			  bool& val) const
    {
      readMaskRegister(vd, ix, val);

      if (ix >= nelems)
	{
	  if (tailAgnOnes_)   // For mask vectors, tail-agnostic is always true.
	    val = true;
	  return false;
	}
      if (masked and not isActive(0, ix))
	{
	  if (maskAgn_ and maskAgnOnes_)
	    val = true;
	  return false;
	}
      return true;
    }

    /// Return true if element at ix of mask destination is active; otherwise,
    /// return false and either read element at ix into val or set val to all
    /// ones depending on the mask-agnostic/tail-agnostic policy.
    bool isMaskDestActive(unsigned vd, unsigned ix, bool masked, bool& val) const
    {
      return isMaskDestActive(vd, ix, masked, elemCount(), val);
    }

    /// Return true if the index ix element group of vector destination is active;
    /// otherwise, return false. Set val to the current value of the element if
    /// active or the expected new value if inactive (either current value or
    /// all ones depending on the agnostic policy).
    template<typename T>
    bool isGroupDestActive(unsigned vd, unsigned ix, unsigned egs, unsigned emulx8, bool masked,
                            T& val) const
    {
      read(vd, ix, emulx8, val);

      if (ix*egs >= elemCount())
	{
	  if (tailAgn_ and tailAgnOnes_)
	    setAllBits(val);
	  return false;
	}
      if (masked and not isActive(0, ix*egs))
	{
	  if (maskAgn_ and maskAgnOnes_)
	    setAllBits(val);
	  return false;
	}

      return true;
    }

    /// Set the ith bit of the given mask register to the given value.
    void writeMaskRegister(uint32_t maskReg, uint32_t i, bool value)
    {
      uint32_t byteIx = i >> 3;
      uint32_t bitIx = i & 7;  // bit in byte
      if (maskReg >= regCount_ or byteIx >= bytesPerReg_)
        throw std::runtime_error("invalid vector register index");

      uint8_t* data = data_.data() + static_cast<std::size_t>(maskReg)*bytesPerReg_;
      uint8_t mask = uint8_t(1) << bitIx;
      if (value)
        data[byteIx] |= mask;
      else
        data[byteIx] &= ~mask;
      lastWrittenReg_ = maskReg;
      lastGroupX8_ = 8;
    }

    /// Set value to the ith bit of the given mask register.
    void readMaskRegister(uint32_t maskReg, uint32_t i, bool& value) const
    {
      uint32_t byteIx = i >> 3;
      uint32_t bitIx = i & 7;  // bit in byte
      if (maskReg >= regCount_ or byteIx >= bytesPerReg_)
        throw std::runtime_error("invalid vector register index");

      const uint8_t* data = data_.data() + static_cast<std::size_t>(maskReg)*bytesPerReg_;
      uint8_t mask = uint8_t(1) << bitIx;
      value = data[byteIx] & mask;
    }

    /// Return the pointers to the 1st byte of the memory area
    /// associated with the given vector. Return nullptr if
    /// vector index is out of bounds.
    uint8_t* getVecData(uint32_t vecIx)
    {
      if (vecIx >= regCount_)
        return nullptr;
      return data_.data() + static_cast<std::size_t>(vecIx)*bytesPerReg_;
    }

    const uint8_t* getVecData(uint32_t vecIx) const
    {
      if (vecIx >= regCount_)
        return nullptr;
      return data_.data() + static_cast<std::size_t>(vecIx)*bytesPerReg_;
    }

    /// It is convenient to construct an empty register file (bytesPerReg = 0)
    /// and configure it later. Old configuration is lost. Register of
    /// newly configured file are initialized to zero.
    void config(uint32_t bytesPerReg, uint32_t minBytesPerElem,
		uint32_t maxBytesPerElem,
                std::unordered_map<GroupMultiplier, unsigned>* minSewPerLmul,
		std::unordered_map<GroupMultiplier, unsigned>* maxSewPerLmul);

    void reset();

    /// Return currently configure element count (cached valye of VL).
    uint32_t elemCount() const
    { return elems_; }

    /// Set currently configure element count (cached valye of VL).
    void elemCount(uint32_t n)
    { elems_ = n; }

    /// Set the currently configured element width.
    void elemWidth(ElementWidth ew)
    { sew_ = ew; }

    /// Set the currently configured group multiplier.
    void groupMultiplier(GroupMultiplier gm)
    { group_ = gm; groupX8_ = groupMultiplierX8(gm); }

    /// Return true if current vtype configuration is legal. This is a cached
    /// value of VTYPE.VILL.
    bool legalConfig() const
    { return not vill_; }

    /// Update cached vtype fields. This is called when Vsetvli is
    /// executed.
    void updateConfig(ElementWidth sew, GroupMultiplier gm,
                      bool maskAgn, bool tailAgn, bool illegal)
    {
      sew_ = sew;
      group_ = gm;
      maskAgn_ = maskAgn;
      tailAgn_ = tailAgn;
      vill_ = illegal;

      groupX8_ = groupMultiplierX8(gm);
      sewInBits_ = elemWidthInBits(sew);
    }

    /// Return the vstart value at the beginning of the last executed
    /// vector instruction.
    unsigned getLastVstart() const
    { return lastVstart_; }

    /// Set the vstart at the beginning of execution of the last executed
    /// vector instruction.
    void setLastVstart(unsigned n)
    { lastVstart_ = n; }

  private:

    /// Map an vector group multiplier to a flag indicating whether given
    /// group is supported.
    using GroupFlags = std::vector<bool>;

    /// Map an element width to a vector of flags indicating supported groups.
    using GroupsForWidth = std::vector<GroupFlags>;

    uint32_t regCount_ = 0;
    uint32_t bytesPerReg_ = 0;
    uint32_t minBytesPerElem_ = 0;
    uint32_t maxBytesPerElem_ = 0;
    uint32_t bytesInRegFile_ = 0;
    std::vector<uint8_t> data_;

    uint32_t elems_ = 0;                           // Cached VL
    ElementWidth sew_ = ElementWidth::Byte;        // Cached VTYPE.SEW
    GroupMultiplier group_ = GroupMultiplier::One; // Cached VTYPE.VLMUL
    bool maskAgn_ = false;                         // Cached VTYPE.ma
    bool tailAgn_ = false;                         // Cached VTYPE.ta
    bool vill_ = false;                            // Cached VTYPE.VILL
    bool maskAgnOnes_ = true; // True if ones written in masked elems when mask agnostic.
    bool tailAgnOnes_ = true; // True if ones written in tail elems when mask agnostic.
    bool updateWholeMask_ = false;  // True if mask instructions update whole mask reg.
    bool trapVtype_ = false; // If true trap invalid vtype; else set VTYPE.VILL.
    std::vector<bool> fpUnorderedSumTreeRed_; // True if unordered fp reduction should use a reduction tree computation
    bool legalizeVsetvlAvl_ = false; // If true legalize VL to VLMAX if vtype is legal (if applicable).
    bool legalizeVsetvliAvl_ = false; // If true legalize VL to VLMAX if vtype is legal (if applicable).
    bool legalizeForEgs_ = false;

    uint32_t groupX8_ = 8;    // Group multiplier as a number scaled by 8.
    uint32_t sewInBits_ = 8;  // SEW expressed in bits (Byte corresponds to 8).

    GroupsForWidth legalConfigs_;

    std::optional<unsigned> lastWrittenReg_;
    uint32_t lastGroupX8_ = 8;   // 8 times last grouping factor
    uint32_t lastVstart_ = 0;    // Vstart at beginning of last vec instruction.

    // Following used for logging/tracing. Cleared before each instruction.
    // Collected by a vector load/store instruction.
    unsigned ldStSize_ = 0;
    std::vector<VecLdStInfo> ldStInfo_;
    std::vector<bool> maskedAddr_;    // True if address is masked off (element skipped).
    std::vector<Step> steps_;         // Incremental steps taken by previous instruction (useful for vector instruction debug).
    std::vector<uint8_t> fpFlags_;    // Incremental fp flags (useful for vector instruction debug).
    std::vector<uint8_t> vxsat_;      // VXSAT per-element operation (useful for vector instruction debug).
    std::vector<unsigned> opsEmul_;   // Effective grouping of vector operands.
  };
}
