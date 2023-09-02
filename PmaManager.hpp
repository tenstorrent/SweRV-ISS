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
       None = 0, Read = 1, Write = 2, Exec = 4, Idempotent = 8, Amo = 16,
       Iccm = 32, Dccm = 64, MemMapped = 128, Rsrv = 256, Io = 512,
       Cacheable = 1024,
       MisalOk = 2048, // True if misaligned access supported.
       MisalAccFault = 4096, // Set if misaligned generates access fault.
       Mapped = Exec | Read | Write,
       Default = Read | Write | Exec | Idempotent | Amo | Rsrv | MisalOk
      };

    /// Default constructor: No access allowed. No-dccm, no-iccm,
    /// no-mmr, no-atomic.
    Pma(Attrib a = None)
      : attrib_(a)
    { }

    /// Return true if associated address region is mapped (accessible
    /// for read, write, or execute).
    bool isMapped() const
    { return attrib_ & (Mapped | MemMapped); }

    /// Return true if ICCM region (instruction closely coupled
    /// memory).
    bool isIccm() const
    { return attrib_ & Iccm; }

    /// Return true if DCCM region (data closely coupled memory).
    bool isDccm() const
    { return attrib_ & Dccm; }

    /// Return true if memory-mapped-register region.
    bool isMemMappedReg() const
    { return attrib_ & MemMapped; }

    /// Return true if idempotent region (non-IO region).
    bool isIdempotent() const
    { return attrib_ & Idempotent; }

    /// Return true if cacheable region.
    bool isCacheable() const
    { return attrib_ & Cacheable; }

    /// Return true if readable (load instructions allowed) region.
    bool isRead() const
    { return attrib_ & (Read | MemMapped); }

    /// Return true if writeable (store instructions allowed) region.
    bool isWrite() const
    { return attrib_ & (Write | MemMapped); }

    /// Return true if executable (fetch allowed) region.
    bool isExec() const
    { return attrib_ & Exec; }

    /// Return true if atomic instructions are allowed.
    bool isAmo() const
    { return attrib_ & Amo; }

    /// Return true if lr/sc instructions are allowed.
    bool isRsrv() const
    { return attrib_ & Rsrv; }

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

    /// Convert given string to a Pma object. Return true on success
    /// return false if string does not contain a valid attribute names.
    /// Valid names: none, read, write, execute, idempotent, amo, iccm,
    /// dccm, mem_mapped, rsrv, io.
    static bool stringToAttrib(std::string_view str, Attrib& attrib);

  private:

    uint32_t attrib_;
  };


  /// Physical memory attribute manager. One per memory. Shared
  /// among cores and harts. Physical memory attributes apply to
  /// word-aligned regions as small as 1 word (but are expected to be
  /// applied to a few number of large regions).
  class PmaManager
  {
  public:

    friend class Memory;

    PmaManager(uint64_t memorySize);

    /// Return the physical memory attribute associated with the
    /// word-aligned address covering the given address. Return
    /// an unmapped attribute if the given address is out of memory
    /// range.
    Pma getPma(uint64_t addr) const
    {
      addr = (addr >> 2) << 2; // Make word aligned.

      // Search regions in order. Return first matching.
      for (const auto& region : regions_)
	if (addr >= region.firstAddr_ and addr <= region.lastAddr_)
	  return region.pma_;

      if (addr >= memSize_)
	return noAccessPma_;
      return defaultPma_;  // rwx amo rsrv idempotent misalok
    }

    /// Define a physical memory attribute region at given index ix
    /// (indices are 0 to n-1 where n is the region count). Regions
    /// are checked in order order (if an address is covered by
    /// multiple regions, then the first defined region applies). The
    /// defined region consists of the word-aligned words with
    /// addresses between fistAddr and lastAddr inclusive. For
    /// example, if firstAddr is 5 and lastAddr is 13, then the
    /// defined region consists of the words at 8 and 12 (bytes 8 to
    /// 15).
    bool defineRegion(unsigned ix, uint64_t firstAddr, uint64_t lastAddr, Pma pma)
    {
      Region region{firstAddr, lastAddr, pma};
      if (ix >= 128)
	return false;  // Arbitrary limit.
      if (ix >= regions_.size())
	regions_.resize(ix + 1);
      regions_.at(ix) = region;
      return true;
    }

    /// Associate a mask with the word-aligned word at the given
    /// address. Return true on success and flase if given address is
    /// not in a memory mapped region.
    bool setMemMappedMask(uint64_t addr, uint32_t mask);

    /// Return mask associated with the word-aligned word at the given
    /// address.  Return 0xffffffff if no mask was ever associated
    /// with given address.
    uint32_t getMemMappedMask(uint64_t addr) const;

    /// Return true if the word-algined word containing given address
    /// is in data closed coupled memory.
    bool isAddrInDccm(size_t addr) const
    { Pma pma = getPma(addr); return pma.isDccm(); }

    /// Return true if given address is in memory-mapped register region.
    bool isAddrMemMapped(size_t addr) const
    { Pma pma = getPma(addr); return pma.isMemMappedReg(); }

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

  protected:

    /// Reset (to zero) all memory mapped registers.
    void resetMemMapped()
    { for (auto& kv  : memMappedRegs_) kv.second.value_ = 0; }

    /// Set value to the value of the memory mapped regiser at addr
    /// returning true if addr is valid. Return false if addr is not word
    /// aligned or is outside of the memory-mapped-regiser area.
    bool readRegister(uint64_t addr, uint32_t& value) const;

    /// Set the value of the memory mapped regiser at addr to the
    /// given value returning true if addr is valid. Return false if
    /// addr is not a memory mapped reg leaving vlaue unmodified.
    bool writeRegister(uint64_t addr, uint32_t value);

    /// Similar to writeRgister but no masking is applied to value.
    bool pokeRegister(uint64_t addr, uint32_t value);

    /// Set the value of the memory mapped regiser byte at addr to the
    /// given value applying masking and returning true if addr is
    /// valid. Return false if addr is not a memory mapped reg leaving
    /// vlaue unmodified.
    bool writeRegisterByte(uint64_t addr, uint8_t value);

    /// Similar to writeRgisterByte but no masking is applied to value.
    bool pokeRegisterByte(uint64_t addr, uint8_t value);

  private:

    struct Region
    {
      uint64_t firstAddr_ = 0;
      uint64_t lastAddr_ = 0;
      Pma pma_;
    };
    
    struct MemMappedReg
    {
      uint32_t value_ = 0;
      uint32_t mask_ = ~uint32_t(0);
    };

    std::vector<Region> regions_;
    std::unordered_map<uint64_t, MemMappedReg> memMappedRegs_;

    Pma defaultPma_{Pma::Attrib::Default};
    Pma noAccessPma_{Pma::Attrib::None};
    uint64_t memSize_ = 0;
  };
}
