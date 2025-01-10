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
#include <ostream>
#include <vector>
#include <algorithm>

namespace WdRiscv
{

  /// Translation lookaside buffer entry.
  struct TlbEntry
  {
    uint64_t virtPageNum_ = 0;
    uint64_t physPageNum_ = 0;
    uint64_t counter_ = 0;   // 2-bit counter for replacement.
    uint32_t asid_ = 0;      // Address space identifier.
    uint32_t vmid_ = 0;      // Virtual machine identifier.
    bool valid_ = false;
    bool global_ = false;    //
    bool user_ = false;      // User-mode entry if true.
    bool read_ = false;      // Has read access.
    bool write_ = false;     // Write access.
    bool exec_ = false;      // Execute Access.
    bool accessed_ = false;
    bool dirty_ = false;
    uint8_t level_ = 0;      // Level of corresponding PTE in address translation walk.
    uint8_t pbmt_ = 0;
  };


  /// Translation lookaside buffer.
  class Tlb
  {
  public:

    /// Address translation mode.
    enum class Mode : uint32_t { Bare = 0, Sv32 = 1, Sv39 = 8, Sv48 = 9, Sv57 = 10,
				 Sv64 = 11, Limit_ = 12};

    /// Define a a TLB with the given size (number of entries).
    Tlb(unsigned size);

    /// Return pointer to TLB entry associated with given virtual page
    /// number and address space identifier.
    /// Return nullptr if no such entry.
    TlbEntry* findEntry(uint64_t pageNum, uint32_t asid)
    {
      auto* entry = getEntry(pageNum);

      if (entry and entry->valid_ and entry->virtPageNum_ == pageNum)
        if (entry->global_ or entry->asid_ == asid)
            return entry;
      return nullptr;
    }

    /// Return pointer to TLB entry associated with given virtual page
    /// number, address space identifier, and virtual machine identifier.
    /// Return nullptr if no such entry.
    TlbEntry* findEntry(uint64_t pageNum, uint32_t asid, uint32_t vmid)
    {
      auto* entry = getEntry(pageNum);

      if (entry and entry->valid_ and entry->virtPageNum_ == pageNum)
        if (entry->global_ or (entry->asid_ == asid and entry->vmid_ == vmid))
            return entry;
      return nullptr;
    }

    /// Return pointer to TLB entry associated with given virtual page
    /// number and address space identifier. Update entry time of
    /// access and increment time if entry is found. Return nullptr if
    /// no such entry.
    TlbEntry* findEntryUpdateTime(uint64_t pageNum, uint32_t asid)
    {
      auto* entry = findEntry(pageNum, asid);
      if (entry)
        {
          ++entry->counter_;
          entry->counter_ &= 3;
        }
      return entry;
    }

    /// Return pointer to TLB entry associated with given virtual page
    /// number and address space identifier. Update entry time of
    /// access and increment time if entry is found. Return nullptr if
    /// no such entry.
    TlbEntry* findEntryUpdateTime(uint64_t pageNum, uint32_t asid, uint32_t vmid)
    {
      auto* entry = findEntry(pageNum, asid, vmid);
      if (entry)
        {
          ++entry->counter_;
          entry->counter_ &= 3;
        }
      return entry;
    }

    /// Print TLB content
    void printTlb(std::ostream& ost) const;

    /// Print TLB entry
    void printEntry(std::ostream& ost, const TlbEntry& te) const;

    /// Return as a string the page/megapage size corresponding to given translation mode
    /// and page table entry level.
    static constexpr const char* ptePageSize(Mode m, uint32_t level)
    {
      if (m == Mode::Bare)
        return "";

      if (level <= 1) return "4K";

      if (m == Mode::Sv32)
        {
          if (level == 2) return "4M";
        }
      else if (m == Mode::Sv39)
        {
          if (level == 2) return "2M";
          if (level == 3) return "1G";
        }
      else if (m == Mode::Sv48)
        {
          if (level == 2) return "2M";
          if (level == 3) return "1G";
          if (level == 4) return "512G";
        }
      else if (m == Mode::Sv57)
        {
          if (level == 2) return "2M";
          if (level == 3) return "1G";
          if (level == 4) return "512G";
          if (level == 5) return "256T";
        }

      assert(0);
      return "";
    }


    /// Set number of TLB entries.
    void setTlbSize(unsigned size)
    { entries_.resize(size); }

    /// Insert a TLB entry for the given translation parameters. If TLB is full
    /// the contents of the  least recently accessed slot are replaced by the
    /// given parameters. Return true on success and false otherwise.
    bool insertEntry(uint64_t virtPageNum, uint64_t physPageNum,
                     uint32_t asid, bool global, bool isUser, bool read,
                     bool write, bool exec);

    /// Insert copy of given entry. Return true on success and false otherwise.
    bool insertEntry(const TlbEntry& entry);

    /// Invalidate every entry matching given address space identifier
    /// unless it is global.
    void invalidateAsid(uint32_t asid)
    {
      for (auto& entry : entries_)
	if (entry.asid_ == asid and not entry.global_)
          {
            entry.valid_ = false;
            entry.counter_ = 0;
          }
    }

    /// Invalidate every entry matching given virtual-machine identifier.
    void invalidateVmid(uint32_t vmid)
    {
      for (auto& entry : entries_)
	if (entry.vmid_ == vmid)
          {
            entry.valid_ = false;
            entry.counter_ = 0;
          }
    }

    /// Invalidate every entry matching given virtual page number.
    void invalidateVirtualPage(uint64_t vpn)
    {
      for (auto& entry : entries_)
        {
          unsigned size = sizeIn4kBytes(mode_, entry.level_);

          if (entry.virtPageNum_ <= vpn and vpn < entry.virtPageNum_ + size)
            {
              entry.valid_ = false;
              entry.counter_ = 0;
            }
        }
    }

    /// Invalidate every entry matching given virtual page number and address space
    /// identifer except for global entries.
    void invalidateVirtualPageAsid(uint64_t vpn, uint32_t asid)
    {
      for (auto& entry : entries_)
        {
          unsigned size = sizeIn4kBytes(mode_, entry.level_);

          if (entry.virtPageNum_ <= vpn and vpn < entry.virtPageNum_ + size and
              entry.asid_ == asid and not entry.global_)
            {
              entry.valid_ = false;
              entry.counter_ = 0;
            }
        }
    }

    /// Invalidate every entry matching given virtual page number and
    /// virtual machine identifer except for global entries.
    void invalidateVirtualPageVmid(uint64_t vpn, uint32_t vmid)
    {
      for (auto& entry : entries_)
        {
          unsigned size = sizeIn4kBytes(mode_, entry.level_);

          if (entry.virtPageNum_ == vpn and vpn < entry.virtPageNum_ + size and
              entry.vmid_ == vmid and not entry.global_)
            {
              entry.valid_ = false;
              entry.counter_ = 0;
            }
        }
    }

    /// Invalidate all entries.
    void invalidate()
    {
      for (auto& entry : entries_)
        {
          entry.valid_ = false;
          entry.counter_ = 0;
        }
    }

    /// Set the address translation mode.
    void setMode(Mode m)
    {
      mode_ = m;
      invalidate();
    }

    /// Return the size of a page/megapage for the given mode and TLB entry level in units
    /// of 4k-bytes.
    uint64_t sizeIn4kBytes(Mode mode, unsigned level) const
    {
      if (mode == Mode::Bare)
        return 0;

      if (level <= 1) return 1;                 // 4K bytes

      if (mode == Mode::Sv32)
        {
          if (level == 2) return 1024;          // 4M bytes
        }
      else if (mode == Mode::Sv39)
        {
          if (level == 2) return 512;           // 2M bytes
          if (level == 3) return 256*1024;      // 1G bytes
        }
      else if (mode == Mode::Sv48)
        {
          if (level == 2) return 512;           // 2M bytes
          if (level == 3) return 256*1024;      // 1G bytes
          if (level == 4) return 128*1024*1024; // 512G bytes
        }
      else if (mode == Mode::Sv57)
        {
          if (level == 2) return 512;           // 2M bytes
          if (level == 3) return 256*1024;      // 1G bytes
          if (level == 4) return 128*1024*1024; // 512G bytes
          if (level == 5) return uint64_t(64)*1024*1204*1024;  // 256T bytes
        }

      assert(0);
      return 0;
    }

    /// Return the size of the page/megapage corresponding to the given TLB entry.


  private:

    /// Return reference to TLB entry associated with given virtual page
    /// number.
    inline TlbEntry* getEntry(uint64_t pageNum)
    {
      unsigned ix = pageNum & (entries_.size() - 1);
      return (entries_.size())? &entries_.at(ix) : nullptr;
    }

    std::vector<TlbEntry> entries_;

    Mode mode_ = Mode::Bare;
  };
}


