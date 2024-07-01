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
#include <string_view>
#include <vector>
#include <algorithm>
#include <unordered_map>

namespace WdRiscv
{

  /// Physical memory attribute. An instance of this is typically
  /// associated with a word-aligned section of the address space.
  class Pma
  {
  public:

    friend class PmaManager;

    enum Attrib
      {
       None = 0, Read = 1, Write = 2, Exec = 4, Idempotent = 8,
       AmoOther = 0x10,  // for amo add/min/max
       AmoSwap = 0x20, AmoLogical = 0x40,
       MemMapped = 0x200, Rsrv = 0x400,
       Io = 0x800, Cacheable = 0x1000,
       MisalOk = 0x2000, // True if misaligned access supported.
       MisalAccFault = 0x4000, // Set if misaligned generates access fault.
       Mapped = Exec | Read | Write,
       AmoArith = AmoSwap | AmoOther | AmoLogical,
       Amo = AmoArith,
       Default = Read | Write | Exec | Idempotent | Amo | Rsrv | MisalOk
      };

    /// Default constructor: No access allowed, no-mmr, no-atomic.
    Pma(Attrib a = None)
      : attrib_(a)
    { }

    /// Return true if associated address region is mapped (accessible
    /// for read, write, or execute).
    bool isMapped() const
    { return attrib_ & Mapped; }

    /// Return true if region has memory mapped register(s).
    bool hasMemMappedReg() const
    { return attrib_ & MemMapped; }

    /// Return true if idempotent region (non-IO region).
    bool isIdempotent() const
    { return attrib_ & Idempotent; }

    /// Return true if cacheable region.
    bool isCacheable() const
    { return attrib_ & Cacheable; }

    /// Return true if readable (load instructions allowed) region.
    bool isRead() const
    { return attrib_ & Read; }

    /// Return true if writeable (store instructions allowed) region.
    bool isWrite() const
    { return attrib_ & Write; }

    /// Return true if executable (fetch allowed) region.
    bool isExec() const
    { return attrib_ & Exec; }

    /// Return true if atomic instructions are allowed.
    bool isAmo() const
    { return attrib_ & Amo; }

    /// Return true if lr/sc instructions are allowed.
    bool isRsrv() const
    { return attrib_ & Rsrv; }

    /// Return true if IO region.
    bool isIo() const
    { return attrib_ & Io; }

    /// Return true if misaligned data access is supported in this region.
    bool isMisalignedOk() const
    { return attrib_& MisalOk; }

    /// Return true if misaligned access generates a misaligned
    /// exception in this region. Returns false if misaligned access
    /// is supported.
    bool misalOnMisal() const
    { return isMisalignedOk() and not (attrib_ & MisalAccFault); }

    /// Return true if misaligned access generates an access fault
    /// exception in this region. Returns false if misaligned access
    /// is supported.
    bool accessFaultOnMisal() const
    { return isMisalignedOk() and (attrib_ & MisalAccFault); }

    /// Return true if this object has the same attributes as the
    /// given object.
    bool operator== (const Pma& other) const
    { return attrib_ == other.attrib_; }

    /// Return true if this object has different attributes from those
    /// of the given object.
    bool operator!= (const Pma& other) const
    { return attrib_ != other.attrib_; }

    /// Enable given attribute in this PMA. Enabling None has no effect.
    void enable(Attrib a)
    { attrib_ |= a; }

    /// Disable given attribute in this PMA. Disabling None has no effect.
    void disable(Attrib a)
    { attrib_ &= ~a; }

    /// Return true if this PMA has the given attribute. If given value
    /// is the or of multiple attributes, then all attributes must be
    /// present in this PMA.
    bool hasAttrib(Attrib a) const
    { return (attrib_ & a) == a; }

    /// Return an integer represenation of the attributes. For now,
    /// just return as-is, could modify later.
    uint32_t attributesToInt()
    { return attrib_; }

    /// Convert given string to a Pma object. Return true on success return false if
    /// string does not contain a valid attribute names.  Valid names: none, read, write,
    /// execute, idempotent, amo, mem_mapped, rsrv, io.
    static bool stringToAttrib(std::string_view str, Attrib& attrib);

    static std::string attributesToString(uint32_t attrib);

  private:

    uint32_t attrib_ = 0;
  };


  /// Physical memory attribute manager. One per memory. Shared
  /// among cores and harts. Physical memory attributes apply to
  /// word-aligned regions as small as 1 word (but are expected to be
  /// applied to a few number of large regions).
  class PmaManager
  {
  public:

    friend class Memory;

    enum AccessReason { None, Fetch, LdSt };

    PmaManager(uint64_t memorySize);

    ~PmaManager() = default;

    /// Return the physical memory attribute associated with the
    /// word-aligned address covering the given address. Return
    /// an unmapped attribute if the given address is out of memory
    /// range.
    inline Pma getPma(uint64_t addr) const
    {
      addr = (addr >> 2) << 2; // Make word aligned.

      // Search regions in order. Return first matching.
      for (const auto& region : regions_)
	if (region.valid_ and addr >= region.firstAddr_ and addr <= region.lastAddr_)
	  return region.pma_;

      if (addr >= memSize_)
	return noAccessPma_;
      return defaultPma_;  // rwx amo rsrv idempotent misalok
    }

    struct PmaTrace
    {
      unsigned ix_;
      uint64_t addr_;
      uint64_t baseAddr_;
      uint64_t lastAddr_;
      AccessReason reason_;
    };

    /// Similar to getPma but updates trace associated with each PMA entry
    inline Pma accessPma(uint64_t addr) const
    { return accessPma(addr, AccessReason::None); }

    /// Similar to getPma but updates trace associated with each PMA entry
    inline Pma accessPma(uint64_t addr, AccessReason reason) const
    {
      addr = (addr >> 2) << 2; // Make word aligned.

      // Search regions in order. Return first matching.
      auto it = std::find_if(regions_.begin(), regions_.end(),
          [addr] (const auto& region) {
              return region.valid_ and addr >= region.firstAddr_ and
                      addr <= region.lastAddr_;
          });

      if (it != regions_.end())
        {
          if (trace_)
            pmaTrace_.push_back({unsigned(std::distance(regions_.begin(), it)),
                                  addr, it->firstAddr_, it->lastAddr_, reason});
          return (*it).pma_;
        }

      if (addr >= memSize_)
	return noAccessPma_;
      return defaultPma_;  // rwx amo rsrv idempotent misalok
    }

    /// Used for tracing to determine if an address matches multiple PMAs.
    bool matchMultiplePma(uint64_t addr) const
    {
      bool hit = false;
      for (const auto& region : regions_)
	if (region.valid_ and addr >= region.firstAddr_ and addr <= region.lastAddr_)
          {
            if (hit)
              return true;
            hit = true;
          }
      return false;
    }

    /// Define a physical memory attribute region at given index ix (indices are 0 to n-1
    /// where n is the region count). Regions are checked in order order (if an address is
    /// covered by multiple regions, then the first defined region applies). The defined
    /// region consists of the word-aligned words with addresses between fistAddr and
    /// lastAddr inclusive. For example, if firstAddr is 5 and lastAddr is 13, then the
    /// defined region consists of the words at 8 and 12 (bytes 8 to 15).
    bool defineRegion(unsigned ix, uint64_t firstAddr, uint64_t lastAddr, Pma pma);

    /// Mark entry at given index as invalid.
    void invalidateEntry(unsigned ix)
    {
      if (ix >= 128)
	return;  // Arbitrary limit.
      if (ix >= regions_.size())
	regions_.resize(ix + 1);
      regions_.at(ix).valid_ = false;
    }

    /// Associate a mask with the word-aligned word at the given
    /// address. Return true on success and flase if given address is
    /// not in a memory mapped region.
    bool setMemMappedMask(uint64_t addr, uint64_t mask, unsigned size);

    /// Return mask associated with the word-aligned word at the given
    /// address.  Return 0xffffffff if no mask was ever associated
    /// with given address.
    uint64_t getMemMappedMask(uint64_t addr) const;

    /// Return true if given address is whitin a memory mapped register.
    bool isMemMappedReg(size_t addr) const
    {
      addr = (addr >> 2) << 2;   // Make a multiple of 4.
      if (memMappedRegs_.find(addr) != memMappedRegs_.end())
	return true;
      addr = (addr >> 3) << 3;   // Make a multiple of 8.
      return memMappedRegs_.find(addr) != memMappedRegs_.end();
    }

    /// Enable misaligned data access in default PMA.
    void enableMisalignedData(bool flag)
    {
      if (flag)
	{
	  defaultPma_.enable(Pma::Attrib::MisalOk);
	  noAccessPma_.enable(Pma::Attrib::MisalOk);
	}
      else
	{
	  defaultPma_.disable(Pma::Attrib::MisalOk);
	  noAccessPma_.disable(Pma::Attrib::MisalOk);
	}
    }

    /// Clear the default PMA (no access).
    void clearDefaultPma()
    { defaultPma_.attrib_ = Pma::Attrib::None; }

    const std::vector<PmaTrace>& getPmaTrace() const
    { return pmaTrace_; }

    void clearPmaTrace()
    { pmaTrace_.clear(); }

    void enableTrace(bool flag)
    { trace_ = flag; }

    /// Print current pma map matching a particular address.
    void printPmas(std::ostream& os, uint64_t address) const;

    /// Print current pma map.
    void printPmas(std::ostream& os) const;

    // Mark region as having memory mapped registers if it overlapps such registers.
    void updateMemMappedAttrib(unsigned ix);

  protected:

    /// Reset (to zero) all memory mapped registers.
    void resetMemMapped()
    { for (auto& kv  : memMappedRegs_) kv.second.value_ = 0; }

    /// Set value to the value of the memory mapped register at addr returning true if
    /// addr is valid. Return false if addr does not fall in a memory-mapped register.
    bool readRegister(uint64_t addr, uint8_t& value) const;

    /// Set value to the value of the memory mapped register at addr returning true if
    /// addr is valid. Return false if addr is not half-word aligned or does not fall in a
    /// memory-mapped register.
    bool readRegister(uint64_t addr, uint16_t& value) const;

    /// Set value to the value of the memory mapped register at addr returning true if
    /// addr is valid. Return false if addr is not word aligned or does not fall in a
    /// memory-mapped register.
    bool readRegister(uint64_t addr, uint32_t& value) const;

    /// Set value to the value of the memory mapped register at addr returning true if
    /// addr is valid. Return false if addr is not double-word aligned or is not that of a
    /// memory-mapped register.
    bool readRegister(uint64_t addr, uint64_t& value) const;

    /// Set the value of the byte of the memory mapped regiser at addr to the given value
    /// returning true if addr is valid. Return false if addr does not fall in a memory
    /// mapped reg.
    bool writeRegister(uint64_t addr, uint8_t value);

    /// Set the value of the half-word of the memory mapped regiser at addr to the given
    /// value returning true if addr is valid. Return false if addr does not fall in a
    /// memory mapped reg or if addr is not half-word aligned.
    bool writeRegister(uint64_t addr, uint16_t value);

    /// Set the value of the word of the memory mapped regiser at addr to the given
    /// value returning true if addr is valid. Return false if addr does not fall in a
    /// memory mapped reg or if addr is not word aligned.
    bool writeRegister(uint64_t addr, uint32_t value);

    /// Set the value of the the memory mapped regiser(s) overlapping addr to the given
    /// value returning true if addr is valid. Return false if addr does not fall in a
    /// memory mapped reg or if addr is not double-word aligned.
    bool writeRegister(uint64_t addr, uint64_t value);

    /// Return true if write is allowed.
    bool checkRegisterWrite(uint64_t addr, unsigned size) const;

    /// Return true if read is allowed.
    bool checkRegisterRead(uint64_t addr, unsigned size) const;

    /// Similar to writeRgister but no masking is applied to value.
    bool pokeRegister(uint64_t addr, uint64_t value);

    /// Similar to writeRgister but no masking is applied to value.
    bool pokeRegisterByte(uint64_t addr, uint8_t value);

  private:

    struct Region
    {
      bool overlaps(uint64_t low, uint64_t high) const
      { return not (high < firstAddr_ or low > lastAddr_); }

      uint64_t firstAddr_ = 0;
      uint64_t lastAddr_ = 0;
      Pma pma_;
      bool valid_ = false;
    };

    struct MemMappedReg
    {
      uint64_t value_ = 0;
      uint64_t mask_ = ~uint64_t(0);
      unsigned size_ = 4;
    };

    /// Return the Region object associated with the
    /// word-aligned word designed by the given address. Return a
    /// no-access object if the givena ddress is out of memory range.
    Region getRegion(uint64_t addr) const
    {
      addr = (addr >> 2) << 2;
      for (const auto& region : regions_)
	if (region.valid_ and addr >= region.firstAddr_ and addr <= region.lastAddr_)
          return region;

      if (addr >= memSize_)
	return { .pma_ = noAccessPma_ };
      return { .pma_ = defaultPma_ };  // rwx amo rsrv idempotent misalok
    }

    /// Print current pmp map matching a particular address.
    void printRegion(std::ostream& os, Region region) const;

    std::vector<Region> regions_;
    std::unordered_map<uint64_t, MemMappedReg> memMappedRegs_;
    std::vector<std::pair<uint64_t, uint64_t>> memMappedRanges_;

    bool trace_ = false;  // Collect stats if true.
    mutable std::vector<PmaTrace> pmaTrace_;

    Pma defaultPma_{Pma::Attrib::Default};
    Pma noAccessPma_{Pma::Attrib::None};
    uint64_t memSize_ = 0;
  };
}
