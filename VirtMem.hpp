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

#include <iosfwd>
#include <boost/variant.hpp>
#include "trapEnums.hpp"
#include "Memory.hpp"
#include "Tlb.hpp"
#include "Pte.hpp"


namespace WdRiscv
{

  template <typename URV>
  class Hart;

  class PmpManager;

  class VirtMem
  {
  public:

    friend class Hart<uint32_t>;
    friend class Hart<uint64_t>;

    enum Mode { Bare = 0, Sv32 = 1, Sv39 = 8, Sv48 = 9, Sv57 = 10, Sv64 = 11 };

    VirtMem(unsigned hartIx, Memory& memory, unsigned pageSize,
            PmpManager& pmpMgr, unsigned tlbSize);

    /// Perform virtual to physical memory address translation and
    /// check for read access if the read flag is true (similary also
    /// check for write access is the write flag is true ...).  Return
    /// encoutered exception on failure or ExceptionType::NONE on
    /// success. Does not check for page crossing.
    ExceptionCause translate(uint64_t va, PrivilegeMode pm, bool read,
                             bool write, bool exec, uint64_t& pa);

    /// Same as translate but only check for execute access.
    ExceptionCause translateForFetch(uint64_t va, PrivilegeMode pm, uint64_t& pa);

    /// Same as translate but only check for execute access and page
    /// crossing.  On success pa1 has the physical address and pa2 has
    /// a copy of pa1 or the physical address of the subsequent page
    /// if the access crosses a page boundary. On Fail pa1 has the
    /// virtual faulting address.
    ExceptionCause translateForFetch2(uint64_t va, unsigned size, PrivilegeMode pm,
				      uint64_t& pa1, uint64_t& pa2);

    /// Same as translate but only check for read access.
    ExceptionCause translateForLoad(uint64_t va, PrivilegeMode pm, uint64_t& pa);

    /// Same as translate but only check for read access and page
    /// crossing.  On success pa1 has the physical address and pa2 has
    /// a copy of pa1 or the physical address of the subsequent page
    /// if the access crosses a page boundary. On Fail pa1 has the
    /// virtual faulting address.
    ExceptionCause translateForLoad2(uint64_t va, unsigned size, PrivilegeMode pm,
				     uint64_t& pa1, uint64_t& pa2);

    /// Same as translate but only check for write access.
    ExceptionCause translateForStore(uint64_t va, PrivilegeMode pm, uint64_t& pa);

    /// Same as translate but only check for write access and page
    /// crossing.  On success pa1 has the physical address and pa2 has
    /// a copy of pa1 or the physical address of the subsequent page
    /// if the access crosses a page boundary. On Fail pa1 has the
    /// virtual faulting address.
    ExceptionCause translateForStore2(uint64_t va, unsigned size, PrivilegeMode pm,
				      uint64_t& pa1, uint64_t& pa2);

    /// Set number of TLB entries.
    void setTlbSize(unsigned size)
    { tlb_.setTlbSize(size); }

    /// Return page size.
    unsigned pageSize() const
    { return pageSize_; }

    /// Return the address of the first byte in the page containing
    /// the given address.
    uint64_t pageStartAddress(uint64_t address) const
    { return (address >> pageBits_) << pageBits_; }

    /// Return the page number corresponding to the given address.
    uint64_t pageNumber(uint64_t addr) const
    { return addr >> pageBits_; }

    /// Debug method: Print all the entries in the page table.
    void printPageTable(std::ostream& os) const;

    /// Print all the page table entries at or below the page table
    /// page rooted at the given address. This is a helper to
    /// printPageTable.
    template <typename PTE, typename VA>
    void printEntries(std::ostream& os, uint64_t addr, std::string path) const;

    /// Return the addresses of the instruction page table entries
    /// used in the last page table walk. Return empty vector if the
    /// last executed instruction did not induce an instruction page
    /// table walk.
    const std::vector<uint64_t>& getInstrPteAddrs() const
    { return pteInstrAddr_; }

    /// Return the addresses of the data page table entries used in
    /// the last page table walk. Return empty vector if the last
    /// executed instruction did not induce a data page table walk.
    const std::vector<uint64_t>& getDataPteAddrs() const
    { return pteDataAddr_; }

    /// Clear trace of page table walk
    void clearPageTableWalk()
    {
      pteInstrAddr_.clear();
      pteDataAddr_.clear();
      clearUpdatedPtes();
    }

    static const char* pageSize(Mode m, uint32_t level)
    {
      if (m == Mode::Bare)
        return "";
      if (m == Mode::Sv32)
        {
          if (level == 0)
            return "4K";
          else
            return "4M";
        }

      switch (level)
        {
          case 0: return "4K";
          case 1: return "2M";
          case 2: return "1G";
          case 3: return "512G";
          case 4: return "256T";
          default: return "";
        }
    }

  protected:

    /// Helper to translate method.
    template <typename PTE, typename VA>
    ExceptionCause pageTableWalk(uint64_t va, PrivilegeMode pm, bool read, bool write,
                                 bool exec, uint64_t& pa, TlbEntry& tlbEntry);

    /// Same as above but for version 1.12 of spec.
    template <typename PTE, typename VA>
    ExceptionCause pageTableWalk1p12(uint64_t va, PrivilegeMode pm, bool read, bool write,
				     bool exec, uint64_t& pa, TlbEntry& tlbEntry);

    /// Helper to translate method.
    ExceptionCause pageTableWalkUpdateTlb(uint64_t va, PrivilegeMode pm, bool read,
                                          bool write, bool exec, uint64_t& pa);

    /// Set the page table root page: The root page is placed in
    /// physical memory at address root * page_size
    void setPageTableRootPage(uint64_t root)
    { pageTableRootPage_ = root; }

    // Change the translation mode to m.  Page size is reset to 4096.
    void setMode(Mode m)
    {
      mode_ = m;
      pageSize_ = 4096;
      pageBits_ = 12;
    }

    /// Set the address space id (asid).
    void setAddressSpace(uint32_t asid)
    { asid_ = asid; }

    /// Make executable pages also readable (supports MXR bit in MSTATUS).
    void setExecReadable(bool flag)
    { execReadable_ = flag; }

    /// Allow supervisor-mode code to access user-mode pages (supports SUM
    /// bit in MSTATUS).
    void setSupervisorAccessUser(bool flag)
    { supervisorOk_ = flag; }

    /// Enable address translation trace
    void enableAddrTransTrace(FILE* file)
    { attFile_ = file; }

    /// Return true if successful and false if page size is not supported.
    bool setPageSize(uint64_t size);

    /// Return current paging mode.
    Mode mode() const
    { return mode_; }

    /// Return current address space id.
    uint32_t addressSpace() const
    { return asid_; }

    /// Set behavior if first access to page or store and page not dirty.
    void setFaultOnFirstAccess(bool flag)
    { faultOnFirstAccess_ = flag; }

    /// Clear saved data for updated leaf level PTE.
    void clearUpdatedPtes()
    { updatedPtes_.clear(); }

    /// Remember value of page table entry modified by most recent translation.
    /// This is for reporting intial memory state.
    void saveUpdatedPte(uint64_t addr, unsigned size, uint64_t value)
    { updatedPtes_.emplace(updatedPtes_.end(), addr, size, value); }

    /// Set byte to the previous PTE value if address is within
    /// the PTE entry updated by the last translation. Leave
    /// byte unchanged otherwise.
    void getPrevByte(uint64_t addr, uint8_t& byte)
    {
      for (auto& prev : updatedPtes_)
	if (addr >= prev.addr_ and addr < prev.addr_ + prev.size_)
	  byte = (prev.value_ >> ((addr - prev.addr_)*8)) & 0xff;
    }

  private:

    struct UpdatedPte
    {
      UpdatedPte(uint64_t addr, unsigned size, uint64_t value)
	: addr_(addr), size_(size), value_(value)
      { }

      uint64_t addr_ = 0;
      unsigned size_ = 0;
      uint64_t value_ = 0;
    };

    Memory& memory_;
    uint64_t pageTableRootPage_ = 0;
    Mode mode_ = Bare;
    uint32_t asid_ = 0;
    unsigned pageSize_ = 4096;
    unsigned pageBits_ = 12;
    uint64_t pageMask_ = 0xfff;
    unsigned hartIx_ = 0;

    uint64_t time_ = 0;  //  Access order

    std::vector<UpdatedPte> updatedPtes_;

    // Cached mstatus bits
    bool execReadable_ = false;  // MXR bit
    bool supervisorOk_ = false;  // SUM bit
    bool faultOnFirstAccess_ = true;

    FILE* attFile_ = nullptr;

    // Addresses of PTEs used in most recent insruction an data translations.
    std::vector<uint64_t> pteInstrAddr_;
    std::vector<uint64_t> pteDataAddr_;

    PmpManager& pmpMgr_;
    Tlb tlb_;
  };

}
