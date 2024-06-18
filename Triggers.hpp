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

#include <cstdint>
#include <vector>
#include <string>
#include <bit>
#include "trapEnums.hpp"

namespace WdRiscv
{

  /// Trigger timing control: Before instruction or after.
  enum class TriggerTiming { Before, After };

  /// Trigger type.
  enum class TriggerType : uint32_t {
    None = 0, Legacy = 1, AddrData = 2, Mcontrol = AddrData, Icount = 3, Itrigger = 4,
    Etrigger = 5, Mcontrol6 = 6, Tmext = 7, Reserved0 = 8, Reserved1 = 9, Reserved2 = 10,
    Reserved3 = 11, Custom0 = 12, Custom1 = 13, Custom2 = 14, Disabled = 15
  };

  enum class TriggerOffset { Tdata1 = 0, Tdata2 = 1, Tdata3 = 2, Tinfo = 3 };


  template <typename URV>
  struct Mcontrol;


  template <typename URV>
  struct Mcontrol6;


  /// Bit fields of tinfo trigger register component.
  struct Tinfo
  {
    unsigned info_     : 16;   // Bits 15-0 : Supported types in type field of TDATA1.
    unsigned           : 8;    // Bits 23-16: Reserved -- hardwired to zero.
    unsigned version_  : 8;    // Bits 24-31: Version
  };


  /// Bit fields of mcontrol trigger register component. 32-bit version.
  template <>
  struct Mcontrol<uint32_t>
    {
      unsigned load_    : 1;   // Bit  0    : trigger on load
      unsigned store_   : 1;   // Bit  1    : trigger on store
      unsigned execute_ : 1;   // Bit  2    : trigger on instruction
      unsigned u_       : 1;   // Bit  3    : enable in user mode
      unsigned s_       : 1;   // Bit  4    : enable in supervisor mode
      unsigned          : 1;   // Bit  5    : reserved: hardwired to zero
      unsigned m_       : 1;   // Bit  6    : enable in machine mode
      unsigned match_   : 4;   // Bits 10-7 : controls what is considered to be a match
      unsigned chain_   : 1;   // Bit  11   :
      unsigned action_  : 4;   // Bits 15-12:
      unsigned sizelo_  : 2;   // Bits 17-16:
      unsigned timing_  : 1;   // Bit  18   :
      unsigned select_  : 1;   // Bit  19   :
      unsigned hit_     : 1;   // Bit  20   :
      unsigned maskMax_ : 6;   // Bit  26-21:
      unsigned dmode_   : 1;   // Bit  27   : trigger writable only in debug mode.
      unsigned type_    : 4;   // Bits 31-28:
  } __attribute__((packed));


  /// Bit fields of mcontrol trigger register component. 64-bit version.
  template <>
  struct Mcontrol<uint64_t>
  {
    unsigned load_    : 1;   // Bit  0    : trigger on load
    unsigned store_   : 1;   // Bit  1    : trigger on store
    unsigned execute_ : 1;   // Bit  2    : trigger on instruction
    unsigned u_       : 1;   // Bit  3    : enable in user mode
    unsigned s_       : 1;   // Bit  4    : enable in supervisor mode
    unsigned          : 1;   // Bit  5    : reserved: hardwired to zero
    unsigned m_       : 1;   // Bit  6    : enable in machine mode
    unsigned match_   : 4;   // Bits 10-7 : controls what is considered to be a match
    unsigned chain_   : 1;   // Bit  11   :
    unsigned action_  : 4;   // Bits 15-12:
    unsigned sizelo_  : 2;   // Bits 17-16:
    unsigned timing_  : 1;   // Bit  18   :
    unsigned select_  : 1;   // Bit  19   :
    unsigned hit_     : 1;   // Bit  20   :
    unsigned sizehi_  : 2;   // Bits 22-21:
    unsigned          : 30;  // Bits 52-23:
    unsigned maskMax_ : 6;   // Bits 58-53:
    unsigned dmode_   : 1;   // Bit  59   : trigger writable only in debug mode.
    unsigned type_    : 4;   // Bits 63-60:
  } __attribute__((packed));


  /// Bit fields of mcontrol6 trigger register component. 32-bit version.
  template <>
  struct Mcontrol6<uint32_t>
  {
    unsigned load_    : 1;   // Bit  0    : trigger on load
    unsigned store_   : 1;   // Bit  1    : trigger on store
    unsigned execute_ : 1;   // Bit  2    : trigger on instruction
    unsigned u_       : 1;   // Bit  3    : enable in user mode
    unsigned s_       : 1;   // Bit  4    : enable in supervisor mode
    unsigned unceren_ : 1;   // Bit  5    : uncertain
    unsigned m_       : 1;   // Bit  6    : enable in machine mode
    unsigned match_   : 4;   // Bits 10-7 : controls what is considered to be a match
    unsigned chain_   : 1;   // Bit  11   :
    unsigned action_  : 4;   // Bits 15-12:
    unsigned size_    : 3;   // Bits 18-16:
    unsigned          : 2;   // Bits 20-19: reserved
    unsigned select_  : 1;   // Bit  21   :
    unsigned hit0_    : 1;   // Bit  22   :
    unsigned vu_      : 1;   // Bit  23   :
    unsigned vs_      : 1;   // Bit  24   :
    unsigned hit1_    : 1;   // Bit  25   :
    unsigned uncer_   : 2;   // Bits 26   :
    unsigned dmode_   : 1;   // Bit  27   : trigger writable only in debug mode.
    unsigned type_    : 4;   // Bits 31-28:
  } __attribute__((packed));


  /// Bit fields of mcontrol6 trigger register component. 64-bit version.
  template <>
  struct Mcontrol6<uint64_t>
  {
    unsigned load_    : 1;   // Bit  0    : trigger on load
    unsigned store_   : 1;   // Bit  1    : trigger on store
    unsigned execute_ : 1;   // Bit  2    : trigger on instruction
    unsigned u_       : 1;   // Bit  3    : enable in user mode
    unsigned s_       : 1;   // Bit  4    : enable in supervisor mode
    unsigned unceren_ : 1;   // Bit  5    : uncertain
    unsigned m_       : 1;   // Bit  6    : enable in machine mode
    unsigned match_   : 4;   // Bits 10-7 : controls what is considered to be a match
    unsigned chain_   : 1;   // Bit  11   :
    unsigned action_  : 4;   // Bits 15-12:
    unsigned size_    : 3;   // Bits 18-16:
    unsigned          : 2;   // Bits 20-19: reserved
    unsigned select_  : 1;   // Bit  21   :
    unsigned hit0_    : 1;   // Bit  22   :
    unsigned vu_      : 1;   // Bit  23   :
    unsigned vs_      : 1;   // Bit  24   :
    unsigned hit1_    : 1;   // Bit  25   :
    unsigned uncer_   : 2;   // Bits 26   :
    unsigned          : 32;  // Bits 58-27:
    unsigned dmode_   : 1;   // Bit  59   : trigger writable only in debug mode.
    unsigned type_    : 4;   // Bits 63-60:
  } __attribute__((packed));


  /// Bit fields for Icount trigger register component.
  template <typename URV>
  struct Icount
  {
    unsigned action_  : 6;
    unsigned u_       : 1;
    unsigned s_       : 1;
    unsigned pending_ : 1;
    unsigned m_       : 1;
    unsigned count_   : 14;
    unsigned hit_     : 1;
    unsigned vu_      : 1;
    unsigned vs_      : 1;
    URV               : 8*sizeof(URV) - 32;  // Zero bits
    unsigned dmode_   : 1;   // Trigger writable only in debug mode.
    unsigned type_    : 4;
  } __attribute__((packed));


  /// Bif tields for Etrigger trigger register component.
  template <typename URV>
  struct Etrigger
  {
    unsigned action_  : 6;
    unsigned u_       : 1;
    unsigned s_       : 1;
    unsigned          : 1;                  // Reserved -- zero.
    unsigned m_       : 1;
    unsigned          : 1;                  // Reserved -- zero.
    unsigned vu_      : 1;
    unsigned vs_      : 1;
    uint64_t          : 8*sizeof(URV) - 19; // Reserved -- zero.
    unsigned hit      : 1;
    unsigned dmode    : 1;
    unsigned type     : 4;
  } __attribute((packed));


  /// Bif tields for Itrigger trigger register component.
  template <typename URV>
  struct Itrigger
  {
    unsigned action_  : 6;
    unsigned u_       : 1;
    unsigned s_       : 1;
    unsigned          : 1;                  // Reserved -- zero.
    unsigned m_       : 1;
    unsigned nmi_     : 1;
    unsigned vu_      : 1;
    unsigned vs_      : 1;
    uint64_t          : sizeof(URV)*8 - 19; // Reserved -- zero.
    unsigned hit      : 1;
    unsigned dmode    : 1;
    unsigned type     : 4;
  } __attribute((packed));


  /// Union to pack/unpack TINFO trigger register value
  union TinfoBits
  {
    TinfoBits(uint64_t value) :
      value_(value)
    { }

    unsigned info() const      { return tinfo_.info_; }
    unsigned version() const   { return tinfo_.version_; }

    uint64_t value_ = 0;
    Tinfo tinfo_;
  };


  /// Union to pack/unpack TDATA1 trigger register value
  template <typename URV>
  union Data1Bits
  {
    Data1Bits(URV value) :
      value_(value)
    { }

    /// Return trigger type field of Tdata1.
    TriggerType type() const { return TriggerType(mcontrol_.type_); }

    /// Return true if type is None or Disabled.
    bool isDisabled() const
    { return type() == TriggerType::None or type() == TriggerType::Disabled; }

    bool isMcontrol()  const { return type() == TriggerType::Mcontrol; }
    bool isMcontrol6() const { return type() == TriggerType::Mcontrol6; }
    bool isAddrData()  const { return isMcontrol() or isMcontrol6(); }
    bool isInstCount() const { return type() == TriggerType::Icount; }
    bool isEtrigger()  const { return type() == TriggerType::Etrigger; }
    bool isItrigger()  const { return type() == TriggerType::Etrigger; }

    /// Return true if trigger is writable only in debug mode.
    bool dmodeOnly() const   { return mcontrol_.dmode_; }

    /// Set the type field of tdata1.
    void setType(TriggerType type)
    { mcontrol_.type_ = unsigned(type); }

    template <typename T>
    const T& mcontrol() const
    {
      if constexpr (std::is_same_v<T, decltype(mcontrol_)>)
        return mcontrol_;
      else
        return mcontrol6_;
    }

    URV value_ = 0;
    Mcontrol<URV>  mcontrol_;
    Mcontrol6<URV> mcontrol6_;
    Icount<URV> icount_;
    Etrigger<URV> etrigger_;
    Itrigger<URV> itrigger_;
  };


  template <typename URV>
  class Triggers;

  /// Model a RISCV trigger.
  template <typename URV>
  class Trigger
  {
  public:

    friend class Triggers<URV>;

    enum class Select { MatchAddress, MatchData };

    enum class Action { RaiseBreak, EnterDebug, StartTrace, StopTrace,
			EmitTrace, Reserved, External0, External1 };

    enum class Chain { No, Yes };

    enum class Match { Equal = 0, Masked = 1, GE = 2, LT = 3, MaskHighEqualLow = 4,
		       MaskLowEqualHigh = 5, NotEqual = 8, NotMasked = 9,
                       NotMaskHighEqualLow = 12, NotMaskLowEqualHigh = 13 };

    Trigger(URV data1 = 0, URV data2 = 0, URV /*data3*/ = 0,
	    URV mask1 = ~URV(0), URV mask2 = ~URV(0), URV mask3 = 0)
      : data1_(data1), data2_(data2), data1WriteMask_(mask1),
	data2WriteMask_(mask2), data3WriteMask_(mask3)
    { }

    /// Read the tdata1 register of the trigger. This is typically the control register of
    /// the trigger.
    URV readData1() const
    { return modifiedT1_? prevData1_ : data1_.value_; }

    /// Read the tdata2 register of the trigger. This is typically the
    /// target value of the trigger.
    URV readData2() const
    { return data2_; }

    /// Read the tdata3 register of the trigger (currently unused).
    URV readData3() const
    { return data3_; }

    /// Read the tinfo register ot the trigger.
    URV readInfo() const
    { return info_.value_; }

    /// Write the tdata1 register of the trigger. This is the interface for CSR
    /// instructions.
    bool writeData1(bool debugMode, URV val)
    {
      if (isDebugModeOnly() and not debugMode)
	return false;
      URV mask = data1WriteMask_;
      if (not debugMode)  // dmode bit writable only in debug mode
	mask &= ~(URV(1) << (8*sizeof(URV) - 5));

      if (not modifiedT1_)
        prevData1_ = data1_.value_;

      // Writing 0 (None) into type is changed to 15 (Disabled). Section 5.7.2 of spec.
      Data1Bits<URV> valBits{val};
      if (valBits.type() == TriggerType::None)
	valBits.setType(TriggerType::Disabled);
      val = valBits.value_;

      data1_.value_ = (val & mask) | (data1_.value_ & ~mask);
      modifiedT1_ = true;

      if (data1_.isAddrData())
	{
	  if (not data1_.dmodeOnly())
	    data1_.mcontrol_.action_ = 0;
	}
      else if (data1_.isInstCount())
	{
	  if (not data1_.dmodeOnly())
	    data1_.icount_.action_ = 0;
	}

      return true;
    }

    /// Write the tdata2 register of the trigger. This is the interface for CSR
    /// instructions.
    bool writeData2(bool debugMode, URV value)
    {
      if (isDebugModeOnly() and not debugMode)
	return false;

      data2_ = (value & data2WriteMask_) | (data2_ & ~data2WriteMask_);
      modifiedT2_ = true;

      updateCompareMask();
      return true;
    }

    /// Write the tdata3 register of the trigger. This is the interface for CSR
    /// instructions.
    bool writeData3(bool debugMode, URV value)
    {
      if (isDebugModeOnly() and not debugMode)
	return false;

      data3_ = (value & data3WriteMask_) | (data3_ & ~data3WriteMask_);
      modifiedT3_ = true;
      return true;
    }

    /// Write the tinfo register of the trigger. This is the interface for CSR
    /// instructions.
    bool writeInfo(bool debugMode, URV value)
    {
      if (isDebugModeOnly() and not debugMode)
	return false;

      info_.value_ = (value & infoWriteMask_) | (info_.value_ & ~infoWriteMask_);
      modifiedInfo_ = true;

      return true;
    }

    /// Poke tdata1. This allows writing of modifiable bits that are read-only to the CSR
    /// instructions.
    void pokeData1(URV x)
    {
      URV val = (x & data1PokeMask_) | (data1_.value_ & ~data1PokeMask_);
      data1_.value_ = val;
      // Configuration dmode==0 and action==1 is not allowed.
      if (data1_.mcontrol_.dmode_ == 0 and data1_.mcontrol_.action_ == 1)
	data1_.mcontrol_.action_ = 0;
    }

    /// Poke tdata2. This allows writing of modifiable bits that are read-only to the CSR
    /// instructions.
    void pokeData2(URV x)
    {
      data2_ = (x & data2PokeMask_) | (data2_ & ~data2PokeMask_);
      updateCompareMask();
    }

    /// Poke tdata3. This allows writing of modifiable bits that are read-only to the CSR
    /// instructions.
    void pokeData3(URV x)
    { data3_ = (x & data3PokeMask_) | (data3_ & ~data3PokeMask_); }

    /// Poke tinfo. This allows writing of modifiable bits that are read-only to the CSR
    /// instructions.
    void pokeInfo(URV x)
    { info_.value_ = (x & infoPokeMask_) | (info_.value_ & ~infoPokeMask_); }

    void configData1(URV reset, URV mask, URV pokeMask)
    { data1Reset_ = reset; data1_.value_ = reset; data1WriteMask_ = mask; data1PokeMask_ = pokeMask;}

    void configData2(URV reset, URV mask, URV pokeMask)
    { data2Reset_ = reset; data2_ = reset; data2WriteMask_ = mask; data2PokeMask_ = pokeMask;}

    void configData3(URV reset, URV mask, URV pokeMask)
    { data3Reset_ = reset; data3_ = reset; data3WriteMask_ = mask; data3PokeMask_ = pokeMask;}

    void configInfo(URV reset, URV mask, URV pokeMask)
    { infoReset_ = reset; info_ = reset; infoWriteMask_ = mask; infoPokeMask_ = pokeMask;}

    /// Reset trigger.
    void reset()
    {
      data1_.value_ = data1Reset_; data2_ = data2Reset_; data3_ = data3Reset_;
      writeData2(true, data2Reset_); // Define compare mask
    }

    /// Return true if this trigger is enabled.
    bool isEnabled() const
    {
      if (data1_.isAddrData())
        {
          if (data1_.isMcontrol())
            return data1_.mcontrol_.m_ or data1_.mcontrol_.s_ or data1_.mcontrol_.u_;
          else
            return data1_.mcontrol6_.m_ or data1_.mcontrol6_.s_ or data1_.mcontrol6_.u_ or
                   data1_.mcontrol6_.vs_ or data1_.mcontrol6_.vu_;
        }
      if (data1_.isInstCount())
	return data1_.icount_.m_ or data1_.icount_.s_ or data1_.icount_.u_;
      return false;
    }

    /// Return true if trigger is writable only in debug mode.
    bool isDebugModeOnly() const
    { return data1_.dmodeOnly(); }

    /// Return true if this is an instruction (execute) trigger.
    bool isInst() const
    { return data1_.isAddrData() and data1_.mcontrol_.execute_; }

    /// Return true if this trigger will cause the processor to enter debug mode on a hit.
    bool isEnterDebugOnHit() const
    {
      if (data1_.isAddrData())
	return Action(data1_.mcontrol_.action_) == Action::EnterDebug;
      if (data1_.isInstCount())
	return Action(data1_.icount_.action_) == Action::EnterDebug;
      return false;
    }

    /// Return true if this trigger is enabled for loads (or stores if
    /// isLoad is false), for addresses, for the given timing and if
    /// it matches the given data address.  Return false otherwise.
    bool matchLdStAddr(URV address, unsigned size, TriggerTiming timing, bool isLoad,
                       PrivilegeMode mode, bool virtMode) const;

    /// Return true if this trigger is enabled for loads (or stores if
    /// isLoad is false), for data, for the given timing and if it
    /// matches the given value address.  Return false otherwise.
    bool matchLdStData(URV value, TriggerTiming timing, bool isLoad,
                       PrivilegeMode mode, bool virtMode) const;

    /// Return true if this trigger is enabled for instruction
    /// addresses (execution), for the given timing and if it matches
    /// the given address. Return false otherwise.
    bool matchInstAddr(URV address, unsigned size, TriggerTiming timing,
                       PrivilegeMode mode, bool virtMode) const;

    /// Return true if this trigger is enabled for instruction opcodes
    /// (execution), for the given timing and if it matches the given
    /// opcode.  Return false otherwise.
    bool matchInstOpcode(URV opcode, TriggerTiming timing,
                         PrivilegeMode mode, bool virtMode) const;

    /// Return true if this trigger is enabled for given mode.
    /// Return false otherwise. This is called for both
    /// instruction retire and trap scenarios.
    bool matchInstCount(PrivilegeMode mode, bool virtMode)
    {
      if (not data1_.isInstCount())
	return false;  // Not an icount trigger.
      Icount<URV>& icount = data1_.icount_;

      if (mode == PrivilegeMode::Machine and not icount.m_)
	return false;  // Trigger is not enabled.

      if (mode == PrivilegeMode::Supervisor and not virtMode and not icount.s_)
	return false;  // Trigger is not enabled.

      if (mode == PrivilegeMode::User and not virtMode and not icount.u_)
	return false;  // Trigger is not enabled.

      if (mode == PrivilegeMode::Supervisor and virtMode and not icount.vs_)
	return false;  // Trigger is not enabled.

      if (mode == PrivilegeMode::User and virtMode and not icount.vu_)
	return false;  // Trigger is not enabled.

      if (mode == PrivilegeMode::Reserved)
        return false;

      return true;
    }

    /// If this trigger is enabled and is of type icount, then make it
    /// count down returning true if its value becomes zero. Return
    /// false otherwise.
    bool instCountdown(PrivilegeMode mode, bool virtMode)
    {
      if (not matchInstCount(mode, virtMode))
        return false;
      Icount<URV>& icount = data1_.icount_;

      icount.count_ = icount.count_? icount.count_ - 1 : 0;
      icount.pending_ = not icount.count_;
      return icount.pending_;
    }

    /// Perform a match on the given item (maybe an address or a value) and the data2
    /// component of this trigger (assumed to be of type Address) according to the match
    /// field.
    bool doMatch(URV item) const;

    /// Set the hit bit of this trigger. For a chained trigger, this is to be called only
    /// if all the triggers in the chain have tripped.
    void setHit(bool flag)
    {
      if (data1_.isAddrData())
	{
          if (not modifiedT1_)
            prevData1_ = data1_.value_;
          if (data1_.isMcontrol())
            data1_.mcontrol_.hit_ = flag;
          else
            data1_.mcontrol6_.hit0_ = flag; // only implement before
	  modifiedT1_ = true;
	}
      if (data1_.isInstCount())
	{
          if (not modifiedT1_)
            prevData1_ = data1_.value_;
	  data1_.icount_.hit_ = flag;
	  modifiedT1_ = true;
	}
    }

    /// Return the hit bit of this trigger.
    bool getHit() const
    {
      if (data1_.isAddrData())
        return data1_.isMcontrol()? data1_.mcontrol_.hit_ : data1_.mcontrol6_.hit0_;
      if (data1_.isInstCount())
	return data1_.icount_.hit_;
      return false;
    }

    /// Return the chain bit of this trigger or false if this trigger has
    /// no chain bit.
    bool getChain() const
    {
      if (data1_.isAddrData())
	return data1_.mcontrol_.chain_;
      return false;
    }

    /// Return the timing of this trigger.
    TriggerTiming getTiming() const
    {
      if (data1_.isAddrData())
	return data1_.isMcontrol()? TriggerTiming(data1_.mcontrol_.timing_) : TriggerTiming::Before;
      return TriggerTiming::After;  // icount has "after" timing.
    }

    /// Return true if the chain of this trigger has tripped.
    bool hasTripped() const
    { return chainHit_; }

    /// Mark this trigger as tripped.
    void setTripped(bool flag)
    { chainHit_ = flag; }

    /// Return the action fields of the trigger.
    Action getAction() const
    {
      if (data1_.isAddrData())
	return Action(data1_.mcontrol_.action_);
      if (data1_.isInstCount())
	return Action(data1_.icount_.action_);
      return Action::RaiseBreak;
    }

    /// Enable all ld/st address matching [address, address+size-1].
    void enableAllLdStAddrMatch(bool flag)
    { matchAllLdStAddr_ = flag; }

    /// Enable all inst address matching [address, address+size-1].
    void enableAllInstAddrMatch(bool flag)
    { matchAllInstAddr_ = flag; }

  protected:

    void updateCompareMask()
    {
      // Pre-compute mask for a masked compare (match == 1 in mcontrol).
      data2CompareMask_ = ~URV(0);
      unsigned leastSigZeroBit = std::countr_one(data2_); // Index of least sig zero bit
      if (leastSigZeroBit >= 8*sizeof(URV) - 1)
        data2CompareMask_ = 0;
      else
        data2CompareMask_ = data2CompareMask_ << (leastSigZeroBit + 1);
    }

    bool isModified() const
    { return modifiedT1_ or modifiedT2_ or modifiedT3_ or modifiedInfo_ or modifiedControl_; }

    void clearModified()
    { modifiedT1_ = modifiedT2_ = modifiedT3_ = modifiedInfo_ = modifiedControl_ = false; }

    bool getLocalHit() const
    { return localHit_; }

    void setLocalHit(bool flag)
    { localHit_ = flag; }

    void setChainHit(bool flag)
    { chainHit_ = flag; }

    void setChainBounds(size_t begin, size_t end)
    {
      chainBegin_ = begin;
      chainEnd_ = end;
    }

    void getChainBounds(size_t& begin, size_t& end) const
    {
      begin = chainBegin_;
      end = chainEnd_;
    }

    bool peek(uint64_t& data1, uint64_t& data2, uint64_t& data3) const
    {
      data1 = data1_.value_; data2 = data2_; data3 = data3_;
      return true;
    }

    bool peek(uint64_t& data1, uint64_t& data2, uint64_t& data3,
	      uint64_t& wm1, uint64_t& wm2, uint64_t& wm3,
	      uint64_t& pm1, uint64_t& pm2, uint64_t& pm3) const
    {
      bool ok = peek(data1, data2, data3);
      wm1 = data1WriteMask_; wm2 = data2WriteMask_; wm3 = data3WriteMask_;
      pm1 = data1PokeMask_; pm2 = data2PokeMask_; pm3 = data3PokeMask_;
      return ok;
    }

    template <typename M>
    bool matchLdStAddr(URV address, unsigned size, TriggerTiming timing, bool isLoad,
                       PrivilegeMode mode, bool virtMode) const;

    template <typename M>
    bool matchLdStData(URV value, TriggerTiming timing, bool isLoad,
                       PrivilegeMode mode, bool virtMode) const;

    template <typename M>
    bool matchInstAddr(URV address, unsigned size, TriggerTiming timing,
                       PrivilegeMode mode, bool virtMode) const;

    template <typename M>
    bool matchInstOpcode(URV opcode, TriggerTiming timing,
                         PrivilegeMode mode, bool virtMode) const;

  private:

    Data1Bits<URV> data1_ = Data1Bits<URV>(0);
    URV data2_            = 0;
    URV data3_            = 0;
    TinfoBits info_       = TinfoBits(0);

    URV data1Reset_   = 0;
    URV data2Reset_   = 0;
    URV data3Reset_   = 0;
    URV infoReset_    = 0;

    URV data1WriteMask_   = ~URV(0);
    URV data2WriteMask_   = ~URV(0);
    URV data3WriteMask_   = 0;              // Place holder.
    URV infoWriteMask_    = ~URV(0);

    URV data1PokeMask_   = ~URV(0);
    URV data2PokeMask_   = ~URV(0);
    URV data3PokeMask_   = 0;              // Place holder.
    URV infoPokeMask_    = ~URV(0);

    URV data2CompareMask_ = ~URV(0);

    URV prevData1_ = 0;

    bool localHit_ = false;  // Trigger tripped in isolation.
    bool chainHit_ = false;   // All entries in chain tripped.
    bool modifiedT1_ = false;
    bool modifiedT2_ = false;
    bool modifiedT3_ = false;
    bool modifiedInfo_ = false;
    bool modifiedControl_ = false;

    size_t chainBegin_ = 0, chainEnd_ = 0;

    bool matchAllLdStAddr_ = true; // If enabled, attempt to match address against
                                        // any [address, address+size-1].
    bool matchAllInstAddr_ = true; // Same as above but for instruction fetch.
  };


  template <typename URV>
  class Triggers
  {
  public:

    Triggers(unsigned count = 0);

    /// Return the number of defined triggers.
    size_t size() const
    { return triggers_.size(); }

    /// Set value to the tdata1 register of the given trigger. Return true on success and
    /// false (leaving value unmodified) if trigger is out of bounds.
    bool readData1(URV trigger, URV& value) const;

    /// Set value to the tdata2 register of the given trigger. Return true on success and
    /// false (leaving value unmodified) if trigger is out of bounds or if data2 is not
    /// implemented.
    bool readData2(URV trigger, URV& value) const;

    /// Set value to the tdata3 register of the given trigger. Return true on success and
    /// false (leaving value unmodified) if trigger is out of bounds of if data3 is not
    /// implemented.
    bool readData3(URV trigger, URV& value) const;

    /// Set value to the tinfo register of the given trigger. Return true on success and
    /// false (leaving value unmodified) if trigger is out of bounds of if tinfo is not
    /// implemented.
    bool readInfo(URV trigger, URV& value) const;

    /// Set value to the tcontroinfo of the given trigger. Return true on success and
    /// false (leaving value unmodified) if trigger is out of bounds of if tcontrol is not
    /// implemented.
    bool readControl(URV trigger, URV& value) const;

    /// Set the tdata1 register of the given trigger to the given value. Return true on
    /// success and false (leaving value unmodified) if trigger is out of bounds.
    bool writeData1(URV trigger, bool debugMode, URV value);

    /// Set the tdata2 register of the given trigger to the given value. Return true on
    /// success and false (leaving value unmodified) if trigger is out of bounds or if
    /// tdata2 is not implemented.
    bool writeData2(URV trigger, bool debugMode, URV value);

    /// Set the tdata3 register of the given trigger to the given value. Return true on
    /// success and false (leaving value unmodified) if trigger is out of bounds or if
    /// tdata3 is not implemented.
    bool writeData3(URV trigger, bool debugMode, URV value);

    /// Set the tinfo register of the given trigger to the given value. Return true on
    /// success and false (leaving value unmodified) if trigger is out of bounds or if
    /// tinfo is not implemented.
    bool writeInfo(URV trigger, bool debugMode, URV value);

    /// Set the tcontrol register of the given trigger to the given value. Return true on
    /// success and false (leaving value unmodified) if trigger is out of bounds or if
    /// tcontrol is not implemented.
    bool writeControl(URV trigger, bool debugMode, URV value);

    /// Return true if given trigger is enabled. Return false if trigger is not enabled or
    /// if it is out of bounds.
    bool isEnabled(URV trigger) const
    {
      if (trigger >= triggers_.size())
	return false;
      return triggers_.at(trigger).isEnabled();
    }

    /// Return true if one or more triggers are enabled.
    bool hasActiveTrigger() const
    {
      for (const auto& trigger : triggers_)
	if (trigger.isEnabled())
	  return true;
      return false;
    }

    /// Return true if one or more instruction (execute) triggers are
    /// enabled.
    bool hasActiveInstTrigger() const
    {
      for (const auto& trigger : triggers_)
	if (trigger.isEnabled() and trigger.isInst())
	  return true;
      return false;
    }

    /// Return true if any of the load (store if isLoad is true) triggers trips. A
    /// load/store trigger trips if it matches the given address and timing and if all the
    /// remaining triggers in its chain have tripped. Set the local-hit bit of any
    /// load/store trigger that matches. If the trigger action is contingent on interrupts
    /// being enabled (ie == true), then the trigger will not trip even if its condition
    /// is satisfied.
    bool ldStAddrTriggerHit(URV address, unsigned size, TriggerTiming, bool isLoad,
                            PrivilegeMode mode, bool virtMode, bool ie);

    /// Similar to ldStAddrTriggerHit but for data match.
    bool ldStDataTriggerHit(URV value, TriggerTiming, bool isLoad,
                            PrivilegeMode mode, bool virtMode, bool ie);

    /// Similar to ldStAddrTriggerHit but for instruction address.
    bool instAddrTriggerHit(URV address, unsigned size, TriggerTiming timing,
                            PrivilegeMode mode, bool virtMode, bool ie);

    /// Similar to instAddrTriggerHit but for instruction opcode.
    bool instOpcodeTriggerHit(URV opcode, TriggerTiming timing,
                              PrivilegeMode mode, bool virtMode, bool ie);

    /// Make every active icount trigger count down unless it was written by the current
    /// instruction. If a count-down register becomes zero as a result of the count-down
    /// and the associated actions is not suppressed (e.g. action is ebreak exception and
    /// interrupts are disabled), then consider the trigger as having tripped and set its
    /// hit bit to 1. Return true if any icount trigger trips; otherwise, return false.
    bool icountTriggerHit(PrivilegeMode prevPrivMode, bool prevVirtMode,
                          PrivilegeMode mode, bool virtMode, bool ie);

    /// Return true if any of the exception-triggers (etrigger) trips.
    bool expTriggerHit(URV cause, PrivilegeMode mode, bool virtMode, bool interruptEnabled);

    /// Return true if any of the interrupt-triggers (itrigger) trips.
    bool intTriggerHit(URV cause, PrivilegeMode mode, bool virtMode, bool interruptEnabled);

    /// Reset the given trigger with the given data1, data2, and data3
    /// values and corresponding write and poke masks. Values are applied
    /// without masking. Subsequent writes will be masked.
    bool reset(URV trigger, URV data1, URV data2, URV data3,
	       URV writeMask1, URV writeMask2, URV writeMask3,
	       URV pokeMask1, URV pokeMask2, URV pokeMask3);

    /// Configure given trigger with given reset values, write masks and
    /// and poke masks.
    bool config(unsigned trigger, 
		const std::vector<uint64_t>& resets,
		const std::vector<uint64_t>& masks,
		const std::vector<uint64_t>& pokeMasks);

    /// Get the values of the three components of the given debug
    /// trigger. Return true on success and false if trigger is out of
    /// bounds.
    bool peek(unsigned trigger, uint64_t& data1, uint64_t& data2,
              uint64_t& data3) const;

    /// Get the values of the three components of the given debug
    /// trigger as well as the components write and poke masks. Return
    /// true on success and false if trigger is out of bounds.
    bool peek(unsigned trigger,
              uint64_t& data1, uint64_t& data2, uint64_t& data3,
	      uint64_t& wm1, uint64_t& wm2, uint64_t& wm3,
	      uint64_t& pm1, uint64_t& pm2, uint64_t& pm3) const;

    /// Set the values of the three components of the given debug
    /// trigger. Return true on success and false if trigger is out of
    /// bounds.
    bool poke(URV trigger, URV v1, URV v2, URV v3);

    bool pokeData1(URV trigger, URV val);
    bool pokeData2(URV trigger, URV val);
    bool pokeData3(URV trigger, URV val);
    bool pokeInfo(URV trigger, URV val);

    /// Clear the remembered indices of the triggers written by the
    /// last instruction.
    void clearLastWrittenTriggers()
    {
      for (auto& trig : triggers_)
	{
	  trig.setLocalHit(false);
	  trig.setChainHit(false);
	  trig.clearModified();
	}
    }

    /// Fill the trigs vector with the indices of the triggers written
    /// by the last instruction.
    void getLastWrittenTriggers(std::vector<unsigned>& trigs) const
    {
      trigs.clear();
      for (unsigned i = 0; i < triggers_.size(); ++i)
	if (triggers_.at(i).isModified())
	  trigs.push_back(i);
    }

    /// Set before/after to the count of tripped triggers with
    /// before/after timing.
    void countTrippedTriggers(unsigned& before, unsigned& after) const
    {
      before = after = 0;
      for (const auto& trig : triggers_)
	if (trig.hasTripped())
	  (trig.getTiming() == TriggerTiming::Before)? before++ : after++;
    }

    /// Return true if there is one or more tripped trigger action set
    /// to "enter debug mode".
    bool hasEnterDebugModeTripped() const
    {
      for (const auto& trig : triggers_)
	if (trig.hasTripped())
          {
            // If chained, use action of last trigger in chain.
            size_t start = 0, end = 0;
            trig.getChainBounds(start, end);
            auto& last = triggers_.at(end - 1);
            if (last.getAction() == Trigger<URV>::Action::EnterDebug)
              return true;
          }
      return false;
    }

    /// Enable all ld/st address matching [address, address+size-1].
    void enableAllLdStAddrMatch(bool flag)
    { for ( auto& trig : triggers_) trig.enableAllLdStAddrMatch(flag); }

    /// Enable all inst address matching [address, address+size-1].
    void enableAllInstAddrMatch(bool flag)
    { for ( auto& trig : triggers_) trig.enableAllInstAddrMatch(flag); }

    /// Reset all triggers.
    void reset();

    /// Return true if given trigger has a local hit.
    bool getLocalHit(URV ix) const
    { return ix < triggers_.size()? triggers_[ix].getLocalHit() : false; }

    /// Enable/disable firing of triggers in machine mode. This supports MTE bit of the
    /// TCONTROL CSR.
    void enableMachineMode(bool flag)
    { mmodeEnabled_ = flag; }

    /// Enable/disable firing of triggers in machine mode when interrupts are enabled.
    void enableMmodeWithIe(bool flag)
    { mmodeWithIe_ = flag; }

    /// Return true if given trigger type is supported.
    bool isSupportedType(TriggerType type) const
    {
      unsigned ix = unsigned(type);
      return ix < supportedTypes_.size() ? supportedTypes_.at(ix) : false;
    }

    /// Set the supported trigger types to the types in the given vector. Items not in the
    /// vector are not supported. Return true on success. Return false if
    /// "none"/"disabled" is not in types vector or if a string in the vector does not correspond
    /// to a trigger type. Valid strings:
    ///   "none", "legacy", "mcontrol", "icount", "itrigger", "etrigger", "mcontrol6",
    //    "tmexttrigger", "disabled"
    bool setSupportedTypes(const std::vector<std::string>& types);

    /// Set the supported trigger types to the types in the given vector. Items not in the
    /// vector are not supported. Return true on success. Return false if None/Disabled is
    /// not in types vector.
    bool setSupportedTypes(const std::vector<TriggerType>& types);

    void getTriggerChange(URV ix, std::vector<std::pair<TriggerOffset, uint64_t>>& changes) const
    {
      changes.clear();
      if (ix >= triggers_.size())
        return;

      using TO = TriggerOffset;

      const auto& trig = triggers_.at(ix);
      if (trig.modifiedT1_)
	changes.push_back(std::pair<TO, uint64_t>(TO::Tdata1, trig.data1_.value_));

      if (trig.modifiedT2_)
	changes.push_back(std::pair<TO, uint64_t>(TO::Tdata2, trig.data2_));

      if (trig.modifiedT3_)
	changes.push_back(std::pair<TO, uint64_t>(TO::Tdata3, trig.data3_));

      if (trig.modifiedInfo_)
	changes.push_back(std::pair<TO, uint64_t>(TO::Tinfo, trig.info_.value_));
    }

    bool isTdata3Modified(URV ix) const
    { return ix < triggers_.size()? triggers_[ix].modifiedT3_ : false; }

  protected:

    /// Return pointer to preceeding trigger in chain or nullptr
    /// if no such trigger.
    Trigger<URV>* getPreceedingTrigger(const Trigger<URV>& trig)
    {
      size_t beginChain = 0, endChain = 0;
      trig.getChainBounds(beginChain, endChain);
      for (unsigned i = beginChain; i < endChain; ++i)
        if (&triggers_.at(i) == &trig)
          return i == beginChain? nullptr : &triggers_.at(i-1);
      return nullptr;
    }

    /// If all the triggers in the chain of the given trigger have tripped (in isolation
    /// using local-hit), then return true setting the hit bit of these
    /// triggers. Otherwise, return false.
    bool updateChainHitBit(Trigger<URV>& trigger);

    /// Define the chain bounds of each trigger.
    void defineChainBounds();

  private:

    std::vector<bool> supportedTypes_;   // Indexed by a TriggerMode.

    std::vector< Trigger<URV> > triggers_;
    bool mmodeEnabled_ = true;  // Triggers trip in Machine mode when true.
    bool mmodeWithIe_ = true;  // Triggers trip in Machine mode when interrupt enabled. 
  };
}
