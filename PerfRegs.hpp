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
#include <cstddef>
#include <vector>
#include <string_view>
#include <unordered_map>
#include <type_traits>
#include <cassert>
#include <functional>
#include "trapEnums.hpp"


namespace WdRiscv
{

  /// Symbolic names for performance events.
  enum class EventNumber
    {
      None,
      ClockActive,       // 1:  Cycles clock active
      ICacheHits,        // 2:  Instruction cache hits
      ICacheMisses,      // 3:  Instruction cache misses

      InstCommited,      // 4:  Instructions committed
      Inst16Commited,    // 5:  16-bit instructions committed
      Inst32Commited,    // 6:  32-bit instructions committed
      InstAligned,       // 7   4-byte aligned instructions

      InstDecode,        // 8:  Instructions decoded

      Mult,              // 9:  Multiply instructions committed
      Div,               // 10: Divide  instructions committed
      Load,              // 11: Loads committed
      Store,             // 12: stores committed
      MisalignLoad,      // 13: misaligned loads
      MisalignStore,     // 14: misaligned stores
      Alu,               // 15: alu instructions committed
      CsrRead,           // 16: Csr read instructions committed
      CsrReadWrite,      // 17: Csr read/write instructions committed
      CsrWrite,          // 18: Csr write instructions committed
      Ebreak,            // 19: Ebreak instructions committed
      Ecall,             // 20: Ecall instructions committed
      Fence,             // 21: Fence instructions committed
      Fencei,            // 22: Fence.i instructions committed
      Mret,              // 23: Mret instructions committed
      Branch,            // 24: Branch instructions committed

      BranchMiss,        // 25: Mis-predicted branches

      BranchTaken,       // 26: Taken branches

      BranchUnpredict,   // 27: Unpredictable branches
      FetchStall,        // 28: Fetcher stall cycles
      AlignStall,        // 29: Aligner stall cycles
      DecodeStall,       // 30: Decoder stall cycles
      PostSyncStall,     // 31: Post sync stall cycles
      PreSynchStall,     // 32: Pre sync stall cycles
      PipeFrozen,        // 33: Cycles pipeline is frozen
      StoreStall,        // 34: LSU store stalls cycles
      DmaDccmStall,      // 35: DMA DCCM stall cycles
      DmaIccmStall,      // 36: DMA ICCM stall cycles

      Exception,         // 37: Exception count

      TimerInterrupt,    // 38: Timer interrupts

      ExternalInterrupt, // 39: External interrupts

      TluFlush,          // 40: TLU flushes (flush lower) 
      TluFlushError,     // 41: Branch error flushes
      BusFetch,          // 42: Fetch bus transactions
      BusTransactions,   // 43: Load/store bus transactions
      BusMisalign,       // 44: Misaligned load/store bus transactions
      IbusError,         // 45: I-bus errors
      DbusError,         // 46: D-bus errors
      IbusBusy,          // 47: Cycles stalled due to Ibus busy 
      DbusBusy,          // 48: Cycles stalled due to Dbus busy 
      InterruptDisabled, // 49: Cycles interrupts disabled
      InterruptStall,    // 50: Cycles interrupts stalled while disabled
      Atomic,            // 51: Atomic (amo) instruction
      Lr,                // 52: Load-reserve instruction
      Sc,                // 53: Store-conditional instruction

      Bitmanip,          // 54: Bit-manipulation
      BusLoad,           // 55: Bus load instructions committed
      BusStore,          // 56: Bus store instructions committed

      MultDiv,           // 57: M-extension instruction (Multiply/divide)
      FpHalf,            // 58: Half precision instruction
      FpSingle,          // 59: Single precision instruction
      FpDouble,          // 60: Double precision instruction
      Vector,            // 61: Vector instruction
      Csr,               // 62: Csr instruction
      _End               // 63: Non-event serving as count of events
    };


  template <typename URV>
  class CsRegs;

  template <typename URV>
  class Hart;


  /// Model a set of consecutive performance counters. Theses
  /// correspond to a set of consecutive performance counter CSR.
  class PerfRegs
  {
  public:

    friend class Hart<uint32_t>;
    friend class Hart<uint64_t>;
    friend class CsRegs<uint32_t>;
    friend class CsRegs<uint64_t>;

    /// Define n counters (none if n is zero). Defined counters correspond
    /// consecutively to CSRs mhpcounter3, mhpmcounter4 ...
    PerfRegs(unsigned n = 0);

    /// Chnage to n, number of defined counters. Defined counters correspond
    /// consecutively to CSRs mhpcounter3, mhpmcounter4 ...
    void config(unsigned n);

    /// Update (count-up) all the performance counters currently
    /// associated with the given event, globally enabled in
    /// perfControl, and enabled for the given privilage-mode and the
    /// given virtual-mode.
    bool updateCounters(EventNumber event, uint32_t perfControl,
                        PrivilegeMode mode, bool isVirt)
    {
      if (not activeCounter_)
	return true;   // No counter is assigned a valid event.

      uint32_t mask = privModeToMask(mode, isVirt);

      for (unsigned counterIx = 0; counterIx < eventOfCounter_.size(); ++counterIx)
	{
	  if (event != eventOfCounter_.at(counterIx))
	    continue;
          // Performance counters handeled in here are MHPMCOUNTER3 to
          // MHPMCOUNTER31 and they are indexed 0 to 29.
          if ((perfControl >> (3+counterIx)) & 1)
            {
	      if (enableMask_.at(counterIx) & mask)
		{
		  uint64_t prev = counters_.at(counterIx)++;
		  if (ovfEnabled_ and counters_.at(counterIx) < prev and ovfCallback_)
		    ovfCallback_(counterIx);
		}
            }
	}
      return true;
    }

    /// Associate given event number with given counter.  Subsequent
    /// calls to updatePerofrmanceCounters(en) will cause given
    /// counter to count up by 1 if this counter is enabled for the
    /// hart privilege mode. The mask parameter is a bit-field
    /// corresponding to the privilege modes for which the event is
    /// enabled (see PerfRegs::PrivModeMask and privModeToMask).
    bool assignEventToCounter(uint64_t event, unsigned counter, unsigned mask)
    {
      EventNumber eventId = EventNumber::None;
      if (userNumberToId_.empty())
	eventId = EventNumber(event);
      else
	{
	  const auto iter = userNumberToId_.find(event);
	  if (iter != userNumberToId_.end())
	    eventId = iter->second;
	}
      pendingEvent_ = eventId;
      pendingCounter_ = counter;
      pendingMask_ = mask;
      hasPending_ = true;
      return true;
    }

    /// Return the number of perormance counters.
    size_t size() const
    { return counters_.size(); }

    /// Map the give user event number to the given internal event id.
    /// Wehn the given user number is written to an mphpmevent csr, then
    /// the corresponding event-id is associated with the event counter csr.
    void configEventNumber(uint64_t userNumber, EventNumber eventId)
    { userNumberToId_[userNumber] = eventId; }

    /// Enable/disable counter overflow.
    void enableOverflow(bool flag)
    { ovfEnabled_ = flag; }

    /// Return true if one or more counters is assigned a valid event. Return false
    /// if all counters are assigned no event (event None).
    bool hasActiveCounter() const
    { return activeCounter_; }

    /// Set id to event-id (tag from enum EventNumber) coresponding to the
    /// given event name returning true. Return false leaving id unmodified
    /// if given string is not an event name.
    static bool findEvent(std::string_view name, EventNumber& id)
    {
      auto iter = eventNameToId_.find(name);
      if (iter == eventNameToId_.end())
	return false;
      id = iter->second;
      return true;
    }

  protected:

    /// This is for documentation.
    enum class PrivModeMask : uint32_t { U = 1, S = 2, M = 4, VU = 8, VS = 16 };

    /// Return mask corresponding to given privilege mode and V bit.
    uint32_t privModeToMask(PrivilegeMode mode, bool isVirt)
    {
      uint32_t n = static_cast<uint32_t>(mode);
      n = n + 1;
      if (isVirt)
	n *= 8;
      return n;
    }

    bool applyPerfEventAssign();

    /// Reset all assosiations among events and counters.
    void reset();

  private:

    // Map counter index to event currently associated with counter.
    std::vector<EventNumber> eventOfCounter_;

    // Map counter index to a word containing enable bits (1 bit per privielge mode).
    std::vector<uint32_t> enableMask_;

    std::vector<uint64_t> counters_;

    // Map an event name to an event id.
    static const std::unordered_map<std::string_view, EventNumber> eventNameToId_;

    // Map a user event number to an internal event id.
    std::unordered_map<uint64_t, EventNumber> userNumberToId_;

    // Pending event assignment to counter.
    EventNumber pendingEvent_ = EventNumber::None;
    unsigned pendingCounter_ = 0;
    uint32_t pendingMask_ = 0;
    bool hasPending_ = false;

    bool activeCounter_ = false; // True if any counter is assigned a vlid event.

    // Called with counter index (0 corresponds to mhpmcounter3) on overflow.
    std::function<void(unsigned)> ovfCallback_ = nullptr;
    bool ovfEnabled_ = false;   // True if counter overflow enabled.
  };
}
