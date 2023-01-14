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

    ~VirtMem()
    { }

    /// Perform virtual to physical memory address translation and
    /// check for read/write/fetch access if the read/write/exec flag
    /// is true (one and only one of the flags must be true).  Return
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

    /// Same as translate but only check for read access if load is
    /// true and for write acess if load is false..
    ExceptionCause translateForLdSt(uint64_t va, PrivilegeMode pm, bool load, uint64_t& pa);

    /// Same as translateForLdSt but translate subsequent page address
    /// if access crosses page boundary. On success pa1 has the
    /// physical address and pa2 has a copy of pa1 or the physical
    /// address of the subsequent page if the access crosses a page
    /// boundary. On Fail pa1 has the virtual faulting address.
    ExceptionCause translateForLdSt2(uint64_t va, unsigned size, PrivilegeMode pm,
				     bool load, uint64_t& pa1, uint64_t& pa2);

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

    /// Return the number of instruction page table walks used by the
    /// last instruction address translation (this may be 0, 1, or 2
    /// -- 0 if TLB hit, 2 if instruction crosses page boundary).
    unsigned numFetchWalks() const
    { return fetchWalks_.size(); }

    /// Return the number of walks used by the last data translation.
    unsigned numDataWalks() const
    { return dataWalks_.size(); }

    /// Return the addresses of the instruction page table entries
    /// used in the last page table walk. Return empty vector if the
    /// last executed instruction did not induce an instruction page
    /// table walk.
    const std::vector<uint64_t>& getFetchWalks(unsigned ix) const
    { return ix < fetchWalks_.size() ? fetchWalks_.at(ix) : emptyWalk_; }

    /// Return the addresses of the data page table entries used in
    /// the last page table walk. Return empty vector if the last
    /// executed instruction did not induce a data page table walk.
    const std::vector<uint64_t>& getDataWalks(unsigned ix) const
    { return ix < dataWalks_.size() ? dataWalks_.at(ix) : emptyWalk_; }

    const std::vector<std::vector<uint64_t>>& getFetchWalks() const
    { return fetchWalks_; }

    const std::vector<std::vector<uint64_t>>& getDataWalks() const
    { return dataWalks_; }

    /// Clear trace of page table walk
    void clearPageTableWalk()
    {
      fetchWalks_.clear();
      dataWalks_.clear();
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

    // Similar to translateForFetch but for VS/VU privilege modes.
    ExceptionCause translateForFetchVs(uint64_t va, PrivilegeMode pm, uint64_t& pa);

    // Similar to translateForLoad but for VS/VU privilege modes.
    ExceptionCause translateForLoadVs(uint64_t va, PrivilegeMode pm, uint64_t& pa);

    // Similar to translateForStore but for VS/VU privilege modes.
    ExceptionCause translateForStoreVs(uint64_t va, PrivilegeMode pm, uint64_t& pa);

    /// Heper to transAddrNoUpdate
    ExceptionCause transNoUpdate(uint64_t va, PrivilegeMode priv, bool read,
				 bool write, bool exec, uint64_t& pa);

    /// Translate virtual address without updating TLB or
    /// updating/checking A/D bits of PTE. Return ExceptionCause::NONE
    /// on success or fault/access exception on failure. If succesful
    /// set pa to the physical address.
    ExceptionCause transAddrNoUpdate(uint64_t va, PrivilegeMode pm, bool r,
				     bool w, bool x, uint64_t& pa);

    /// Helper to translate method.
    template <typename PTE, typename VA>
    ExceptionCause pageTableWalk(uint64_t va, PrivilegeMode pm, bool read, bool write,
                                 bool exec, uint64_t& pa, TlbEntry& tlbEntry);

    /// Same as above but for version 1.12 of spec.
    template <typename PTE, typename VA>
    ExceptionCause pageTableWalk1p12(uint64_t va, PrivilegeMode pm, bool read, bool write,
				     bool exec, uint64_t& pa, TlbEntry& tlbEntry);

    /// Page table walk for the G stage of 2-stage address translation.
    template <typename PTE, typename VA>
    ExceptionCause pageTableWalkStage2(uint64_t va, PrivilegeMode pm, bool read, bool write,
				       bool exec, uint64_t& pa, TlbEntry& tlbEntry);

    /// Helper to translate methods. Similar to translate but does not use or
    /// update TLB cache. Given TLB entry is initialized so that caller may
    /// place it in the TLB.
    ExceptionCause translateNoTlb(uint64_t va, PrivilegeMode pm, bool read,
				  bool write, bool exec, uint64_t& pa, TlbEntry& entry);

    /// Helper to translate methods for 2nd stage of guest address translation
    /// (guest physical address to host physical address).
    ExceptionCause doStage2Translate(uint64_t va, PrivilegeMode pm, bool read,
				     bool write, bool exec, uint64_t& pa, TlbEntry& entry);

    /// Set the page table root page: The root page is placed in
    /// physical memory at address root * page_size
    void setRootPage(uint64_t root)
    { rootPage_ = root; }

    /// Set the page table root page for 2nd stage address translation.
    void setStage2RootPage(uint64_t root)
    { rootPageStage2_ = root; }

    // Change the translation mode to m.
    void setMode(Mode m)
    { mode_ = m; }

    // Change the translation mode to m for the 2nd stage of 2-stage
    // (VS) translation.
    void setStage2Mode(Mode m)
    { modeStage2_ = m; }

    /// Set the address space id (asid).
    void setAsid(uint32_t asid)
    { asid_ = asid; }

    /// Set the virtual machine id for 2-stage translation.
    void setVmid(uint32_t vmid)
    { vmid_ = vmid; }

    /// Set V mode (hyerpvisor VS or VU mode) if flag is true.
    void setTwoStage(bool flag)
    { twoStage_ = flag; }

    /// Make executable pages also readable (supports MXR bit in MSTATUS).
    void setExecReadable(bool flag)
    { execReadable_ = flag; }

    /// Allow supervisor-mode code to access user-mode pages (supports SUM
    /// bit in MSTATUS).
    void setSupervisorAccessUser(bool flag)
    { supervisorOk_ = flag; }

    /// Enable/disable address translation logging (disable if file is
    /// null). Return previous value of file.
    FILE* enableAddrTransLog(FILE* file)
    { FILE* prev = attFile_; attFile_ = file; return prev; }

    /// Return true if successful and false if page size is not supported.
    bool setPageSize(uint64_t size);

    /// Return current address translation mode (SV32, SV39 ...)
    Mode mode() const
    { return mode_; }

    /// Return true if two-stage translation is on.
    bool isTwoStage() const
    { return twoStage_; }

    /// Return current address space id.
    uint32_t asid() const
    { return asid_; }

    /// Return current virtual machine id.
    uint32_t vmid() const
    { return vmid_; }

    /// Set behavior if first access to page or store and page not dirty.
    void setFaultOnFirstAccess(bool flag)
    { faultOnFirstAccess_ = flag; }

    /// Clear saved data for updated leaf level PTE.
    void clearUpdatedPtes()
    { updatedPtes_.clear(); }

    /// Remember value of page table entry modified by most recent translation.
    /// This is for reporting intial memory state.
    void saveUpdatedPte(uint64_t addr, unsigned size, uint64_t value)
    {
      if (trace_)
	updatedPtes_.emplace(updatedPtes_.end(), addr, size, value);
    }

    /// Eable/disable tracing of accessed page table entries.
    /// Return prior trace setting.
    bool enableTrace(bool flag)
    { bool prev = trace_; trace_ = flag; return prev; }

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
    uint64_t rootPage_ = 0;
    uint64_t rootPageStage2_ = 0;  // Root page for 2nd stage translation.
    Mode mode_ = Bare;
    Mode modeStage2_ = Bare;       // For 2nd stage translation.
    bool twoStage_ = false;        // True if two-stage translation is on.
    uint32_t asid_ = 0;
    uint32_t vmid_ = 0;
    unsigned pageSize_ = 4096;
    unsigned pageBits_ = 12;
    uint64_t pageMask_ = 0xfff;
    unsigned hartIx_ = 0;

    uint64_t time_ = 0;  //  Access order

    bool trace_ = true;

    std::vector<UpdatedPte> updatedPtes_;

    // Cached mstatus bits
    bool execReadable_ = false;  // MXR bit
    bool supervisorOk_ = false;  // SUM bit
    bool faultOnFirstAccess_ = true;
    bool accessDirtyCheck_ = true;  // To be able to supress AD check

    FILE* attFile_ = nullptr;

    // Addresses of PTEs used in most recent insruction an data translations.
    typedef std::vector<uint64_t> Walk;
    std::vector<Walk> fetchWalks_;       // Instruction fetch walks of last instruction.
    std::vector<Walk> dataWalks_;    // Data access walks of last instruction.
    const Walk emptyWalk_;

    PmpManager& pmpMgr_;
    Tlb tlb_;
  };

}
