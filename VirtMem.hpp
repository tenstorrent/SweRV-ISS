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
#include "trapEnums.hpp"
#include "Memory.hpp"
#include "Tlb.hpp"
#include "Pte.hpp"
#include "util.hpp"


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

    enum Mode { Bare = 0, Sv32 = 1, Sv39 = 8, Sv48 = 9, Sv57 = 10, Sv64 = 11,
		Limit_ = 12};

    enum Pbmt { None = 0, Nc = 1, Io = 2, Reserved = 3 };

    VirtMem(unsigned hartIx, Memory& memory, unsigned pageSize,
            PmpManager& pmpMgr, unsigned tlbSize);

    ~VirtMem() = default;

    /// Perform virtual to physical memory address translation and
    /// check for read/write/fetch access (one and only one of the
    /// read/write/exec flags must be true). Return encoutered
    /// exception on failure or ExceptionType::NONE on success. Does
    /// not check for page crossing. The twoStage flag indicates
    /// that two-stage translation (hypervisor with V=1) is on.
    ExceptionCause translate(uint64_t va, PrivilegeMode pm, bool twoStage,
			     bool read, bool write, bool exec, uint64_t& gpa,
                             uint64_t& pa);

    /// Similar to translate but targeting only execute access.
    ExceptionCause translateForFetch(uint64_t va, PrivilegeMode pm, bool twoStage,
				     uint64_t& gpa, uint64_t& pa)
    {
      forFetch_ = true;
      auto cause = translate(va, pm, twoStage, false, false, true, gpa, pa);
      forFetch_ = false;
      return cause;
    }

    /// Similar to translate but targeting only read access.
    ExceptionCause translateForLoad(uint64_t va, PrivilegeMode pm, bool twoStage,
				    uint64_t& gpa, uint64_t& pa)
    { return translate(va, pm, twoStage, true, false, false, gpa, pa); }

    /// Similar to translate but targeting only write access.
    ExceptionCause translateForStore(uint64_t va, PrivilegeMode pm, bool twoStage,
				     uint64_t& gpa, uint64_t& pa)
    { return translate(va, pm, twoStage, false, true, false, gpa, pa); }

    /// Similar to translateForFetch but also check for page
    /// crossing. On success, gpa1/pa1 will have the physical address and
    /// gpa2/pa2 a copy of pa1 or the physical address of the subsequent
    /// page if the access crosses a page boundary. On failure, either
    /// pa1 or gpa1 will have the virtual faulting address depending on
    /// if there was a two stage translation and which stage the fail occured.
    ExceptionCause translateForFetch2(uint64_t va, unsigned size, PrivilegeMode pm,
				      bool twoStage, uint64_t& gpa1, uint64_t& pa1,
                                      uint64_t& gpa2, uint64_t& pa2);

    /// Same as translate but targeting load or store and checking for
    /// page crossing.  On success, gpa1/pa1 will have the physical address
    /// and gpa2/pa2 a copy of pa1 or the physical address of the
    /// subsequent page if the access crosses a page boundary. On
    /// failuer, either gpa1 or pa1 will have the virtual faulting address
    /// depending on if there was a two stage translation and which stage
    /// the fail occured.
    ExceptionCause translateForLdSt2(uint64_t va, unsigned size, PrivilegeMode pm,
				     bool twoStage, bool load, uint64_t& gpa1,
                                     uint64_t& pa1, uint64_t& gpa2, uint64_t& pa2);

    /// Load version of translateForLdSt2.
    ExceptionCause translateForLoad2(uint64_t va, unsigned size, PrivilegeMode pm,
				     bool twoStage, uint64_t& gpa1, uint64_t& pa1,
                                     uint64_t& gpa2, uint64_t& pa2)
    { return translateForLdSt2(va, size, pm, twoStage, true, gpa1, pa1, gpa2, pa2); }

    /// Store version of translateForLdSt2.
    ExceptionCause translateForStore2(uint64_t va, unsigned size, PrivilegeMode pm,
				      bool twoStage, uint64_t& gpa1, uint64_t& pa1,
                                      uint64_t& gpa2, uint64_t& pa2)
    { return translateForLdSt2(va, size, pm, twoStage, false, gpa1, pa1, gpa2, pa2); }

    /// Set number of TLB entries.
    void setTlbSize(unsigned size)
    {
      tlb_.setTlbSize(size);
      vsTlb_.setTlbSize(size);
      stage2Tlb_.setTlbSize(size);
    }

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
    void printEntries(std::ostream& os, uint64_t addr, const std::string& path) const;

    /// Return the number of instruction page table walks used by the
    /// last instruction address translation (this may be 0, 1, or 2
    /// -- 0 if TLB hit, 2 if instruction crosses page boundary).
    unsigned numFetchWalks() const
    { return fetchWalks_.size(); }

    /// Return the number of walks used by the last data translation.
    unsigned numDataWalks() const
    { return dataWalks_.size(); }

    /// Mark items in the modes array as supported translation modes.
    void setSupportedModes(const std::vector<Mode>& modes)
    {
      std::fill(supportedModes_.begin(), supportedModes_.end(), false);
      for (auto mode : modes)
	{
	  unsigned ix = unsigned(mode);
	  if (ix < supportedModes_.size())
	    supportedModes_.at(ix) = true;
	}
    }

    /// Return true if given mode is supported. On construction all
    /// modes are supported but that can be modified with
    /// setSupportedModes.
    bool isModeSupported(Mode mode)
    {
      unsigned ix = unsigned(mode);
      return ix < supportedModes_.size() ? supportedModes_.at(ix) : false;
    }

    struct WalkEntry
    {
      enum Type { GVA = 0, GPA = 1, PA = 2};

      WalkEntry(uint64_t addr, Type type)
        : addr_(addr), type_(type)
      { assert(type != Type::PA); }

      WalkEntry(uint64_t addr)
        : addr_(addr), type_(Type::PA)
      { }

      uint64_t addr_ = 0;
      Type type_ = Type::PA;

      // Only applicable for leaf entries
      Pbmt pbmt_ = Pbmt::None;
    };

    /// Return the addresses of the instruction page table entries
    /// used in the last page table walk. Return empty vector if the
    /// last executed instruction did not induce an instruction page
    /// table walk.
    const std::vector<WalkEntry>& getFetchWalks(unsigned ix) const
    { return ix < fetchWalks_.size() ? fetchWalks_.at(ix) : emptyWalk_; }

    /// Return the addresses of the data page table entries used in
    /// the last page table walk. Return empty vector if the last
    /// executed instruction did not induce a data page table walk.
    const std::vector<WalkEntry>& getDataWalks(unsigned ix) const
    { return ix < dataWalks_.size() ? dataWalks_.at(ix) : emptyWalk_; }

    const std::vector<std::vector<WalkEntry>>& getFetchWalks() const
    { return fetchWalks_; }

    const std::vector<std::vector<WalkEntry>>& getDataWalks() const
    { return dataWalks_; }

    /// Clear trace of page table walk
    void clearPageTableWalk()
    {
      fetchWalks_.clear();
      dataWalks_.clear();
      clearUpdatedPtes();
      fetchPageCross_ = false;
      dataPageCross_ = false;
      pbmt_ = Pbmt::None;
    }

    /// Clear extra trap information
    void clearTrapInfo()
    {
      stage1Trap_ = false;
      stage1AttemptedADUpdate_ = false;
    }

    /// Return page based memory type of last translation
    Pbmt lastPbmt() const
    { return pbmt_; }

    static constexpr const char* pageSize(Mode m, uint32_t level)
    {
      if (m == Mode::Bare)
        return "";
      if (m == Mode::Sv32)
        {
          if (level == 0)
            return "4K";
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

    /// Return string representing translation mode. Example: Sv32 yields "sv32".
    static constexpr std::string_view to_string(Mode mode)
    {
      using namespace std::string_view_literals;
      constexpr auto vec =
        std::array{"bare"sv, "sv32"sv, "sv?"sv, "sv?"sv, "sv?"sv, "sv?"sv,
		   "sv?"sv, "sv?"sv, "sv39"sv, "sv48"sv, "sv57"sv, "sv64"sv};
      return size_t(mode) < vec.size()? vec.at(size_t(mode)) : "sv?";
    }

    /// Set mode to the translation mode corresponding to modeStr
    /// returning true if successful. Return false leaving mode
    /// unmodified if modeStr does not correspond to a mode.
    static bool to_mode(std::string_view modeStr, Mode& mode)
    {
      static const std::unordered_map<std::string_view, Mode> map(
        { {"bare", Mode::Bare }, {"sv32", Mode::Sv32 }, {"sv39", Mode::Sv39 },
	  {"sv48", Mode::Sv48 }, {"sv57", Mode::Sv57 }, {"sv64", Mode::Sv64 } });
      auto iter = map.find(modeStr);
      if (iter != map.end())
	{
	  mode = iter->second;
	  return true;
	}
      return false;
    }

    static Pma overridePmaWithPbmt(Pma pma, Pbmt pbmt)
    {
      if (pbmt == Pbmt::None or pbmt == Pbmt::Reserved)
        return pma;

      pma.disable(Pma::Attrib::Cacheable);
      pma.disable(Pma::Attrib::Amo);
      pma.disable(Pma::Attrib::Rsrv);

      if (pbmt == Pbmt::Nc)
        {
          pma.enable(Pma::Attrib::Idempotent);
          pma.disable(Pma::Attrib::Io);
        }
      else
        {
          pma.disable(Pma::Attrib::Idempotent);
          pma.enable(Pma::Attrib::Io);
        }
      return pma;
    }

  protected:

    /// Return current big-endian mode of implicit memory read/write
    /// used by translation.
    bool bigEndian() const
    { return bigEnd_; }

    /// Set the big-endian mode of implicit memory read/write ops used
    /// by translation.
    void setBigEndian(bool be)
    { bigEnd_ = be; }

    /// Read a memory word honoring the big-endian flag. Return true
    /// on success and false on failure.
    bool memRead(uint64_t addr, bool bigEnd, uint32_t& data)
    {
      if (not memory_.read(addr, data))
	return false;
      if (bigEnd)
	data = util::byteswap(data);
      return true;
    }

    /// Read a memory double-word honoring the big-endian flag. Return
    /// true on success and false on failure.
    bool memRead(uint64_t addr, bool bigEnd, uint64_t& data)
    {
      if (not memory_.read(addr, data))
	return false;
      if (bigEnd)
	data = util::byteswap(data);
      return true;
    }

    /// Write a memory word honoring the big-endian flag. Return true
    /// on success and false on failure.
    bool memWrite(uint64_t addr, bool bigEnd, uint32_t data)
    {
      if (bigEnd)
	data = util::byteswap(data);
      return memory_.write(hartIx_, addr, data);
    }

    /// Write a memory double-word honoring the big-endian flag. Return
    /// true on success and false on failure.
    bool memWrite(uint64_t addr, bool bigEnd, uint64_t data)
    {
      if (bigEnd)
	data = util::byteswap(data);
      return memory_.write(hartIx_, addr, data);
    }

    /// Use exec access permission for read permission.
    void useExecForRead(bool flag)
    { xForR_ = flag; }

    /// Return true if use-exec-for-read is on.
    bool isExecForRead() const
    { return xForR_; }

    /// Heper to transAddrNoUpdate
    ExceptionCause transNoUpdate(uint64_t va, PrivilegeMode priv, bool twoStage,
				 bool read, bool write, bool exec, uint64_t& pa);

    /// Translate virtual address without updating TLB or
    /// updating/checking A/D bits of PTE. Return ExceptionCause::NONE
    /// on success or fault/access exception on failure. If succesful
    /// set pa to the physical address.
    ExceptionCause transAddrNoUpdate(uint64_t va, PrivilegeMode pm, bool twoStage,
				     bool r, bool w, bool x, uint64_t& pa);

    /// Helper to translate methods: Page table walk version 1.12.
    template <typename PTE, typename VA>
    ExceptionCause pageTableWalk1p12(uint64_t va, PrivilegeMode pm, bool read, bool write,
				     bool exec, uint64_t& pa, TlbEntry& tlbEntry);

    /// Page table walk version 1.12 for the G stage of 2-stage
    /// address translation.
    template <typename PTE, typename VA>
    ExceptionCause stage2PageTableWalk(uint64_t va, PrivilegeMode pm, bool read, bool write,
				       bool exec, bool isPteAddr, uint64_t& pa, TlbEntry& tlbEntry);

    /// Page table walk version 1.12 for the VS stage of 2-stage
    /// address translation.
    template <typename PTE, typename VA>
    ExceptionCause stage1PageTableWalk(uint64_t va, PrivilegeMode pm, bool read, bool write,
				       bool exec, uint64_t& pa, TlbEntry& tlbEntry);

    /// Helper to translate methods for single stage translation. Does not use or
    /// update TLB cache. Given TLB entry is initialized so that caller may
    /// place it in the TLB.
    ExceptionCause translateNoTlb(uint64_t va, PrivilegeMode pm, bool twoStage,
				  bool r, bool w, bool x, uint64_t& pa, TlbEntry& entry);

    /// Heler to translateNoTlb.
    ExceptionCause twoStageTranslateNoTlb(uint64_t va, PrivilegeMode priv, bool read, bool write,
					  bool exec, uint64_t& pa, TlbEntry& entry);

    /// Helper to translate methods for 2nd stage of guest address translation
    /// (guest physical address to host physical address). We distinguish between
    /// final G-stage translation and PTE address translations.
    ExceptionCause stage2TranslateNoTlb(uint64_t va, PrivilegeMode pm, bool r,
					bool w, bool x, bool isPteAddr, uint64_t& pa, TlbEntry& entry);

    ExceptionCause stage2Translate(uint64_t va, PrivilegeMode priv, bool r, bool w,
				   bool x, bool isPteAddr, uint64_t& pa);

    ExceptionCause stage1TranslateNoTlb(uint64_t va, PrivilegeMode priv, bool r, bool w,
					bool x, uint64_t& pa, TlbEntry& entry);

    ExceptionCause twoStageTranslate(uint64_t va, PrivilegeMode priv, bool r, bool w,
				     bool x, uint64_t& gpa, uint64_t& pa);


    /// Set the page table root page: The root page is placed in
    /// physical memory at address root * page_size
    void setRootPage(uint64_t root)
    { rootPage_ = root; }

    /// Set the page table root page for Vs mdoe: The root page is
    /// placed in guest physical memory at address root * page_size
    void setVsRootPage(uint64_t root)
    { vsRootPage_ = root; }

    /// Set the page table root page for 2nd stage address translation after
    /// clearing the least siginficant 2 bits of the given address.
    void setStage2RootPage(uint64_t root)
    { rootPageStage2_ = (root >> 2) << 2; }

    // Change the translation mode to m.
    void setMode(Mode m)
    { mode_ = m; }

    // Change the translation mode of VS (V==1) to m.
    void setVsMode(Mode m)
    { vsMode_ = m; }

    // Change the translation mode to m for the 2nd stage of 2-stage
    // (VS) translation.
    void setStage2Mode(Mode m)
    { modeStage2_ = m; }

    /// Set the address space id (asid).
    void setAsid(uint32_t asid)
    { asid_ = asid; }

    /// Set the address space id (asid) for VS mode.
    void setVsAsid(uint32_t asid)
    { vsAsid_ = asid; }

    /// Set the virtual machine id for 2-stage translation.
    void setVmid(uint32_t vmid)
    { vmid_ = vmid; }

    /// Make executable pages also readable (supports MXR bit in
    /// MSTATUS/SSTATUS).  This affects both stages of translation in
    /// virtual mode.
    void setExecReadable(bool flag)
    { execReadable_ = flag; }

    /// Make executable pages also readable (supports MXR bit in VSSTATUS).
    /// This only affects stage1 translation.
    void setStage1ExecReadable(bool flag)
    { s1ExecReadable_ = flag; }

    /// Return the stage1 executable-readable state (MXR bit in VSSTATUS).
    bool stage1ExecReadable() const
    { return s1ExecReadable_; }

    /// Allow supervisor-mode code to access user-mode pages (supports SUM
    /// bit in MSTATUS).
    void setSum(bool flag)
    { sum_ = flag; }

    /// Allow supervisor-mode code to access user-mode pages (supports SUM
    /// bit in MSTATUS).
    void setVsSum(bool flag)
    { vsSum_ = flag; }

    /// Enable/disable page-based-memory types.
    void enablePbmt(bool flag)
    { pbmtEnabled_ = flag; }

    /// Enable/disable NAPOT page size (naturally aligned power of 2).
    void enableNapot(bool flag)
    { napotEnabled_ = flag; }

    /// Enable/disable pointer masking for user mode.
    void enablePointerMasking(bool flag)
    { pmEnabled_ = flag; }

    /// Return true if successful and false if page size is not supported.
    bool setPageSize(uint64_t size);

    /// Return current address translation mode (SV32, SV39 ...)
    Mode mode() const
    { return mode_; }

    Mode vsMode() const
    { return vsMode_; }

    Mode modeStage2() const
    { return modeStage2_; }

    /// Return current address space id.
    uint32_t asid() const
    { return asid_; }

    /// Return current address space id for VS mode.
    uint32_t vsAsid() const
    { return vsAsid_; }

    /// Return current virtual machine id.
    uint32_t vmid() const
    { return vmid_; }

    /// Return whether previous translation was page crossing.
    bool pageCross(bool flag) const
    { return (flag)? fetchPageCross_ : dataPageCross_; }

    /// Set behavior if first access to page or store and page not dirty.
    void setFaultOnFirstAccess(bool flag)
    { faultOnFirstAccess_ = flag; }

    /// Similar to above but applies to 2nd stage translation.
    void setFaultOnFirstAccessStage2(bool flag)
    { faultOnFirstAccess2_ = flag; }

    /// Return true if last translation had a fault in VS-stage translation and false otherwise.
    /// Sets flag if attempted to update A/D bits on last stage 1 translation.
    /// This is necessary to properly write mtinst/htinst.
    bool stage1TrapInfo(bool& implicitWrite) const
    {
      implicitWrite = stage1AttemptedADUpdate_;
      return stage1Trap_;
    }

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

    /// Check for NAPOT on PTE and apply NAPOT fixup if applicable.
    /// Returns false if PTE would cause a pagefault due to NAPOT, and true otherwise.
    template <typename PTE, typename VA>
    bool napotCheck(PTE& pte, VA va)
    {
      if (napotEnabled_)
        {
          if (pte.hasNapot())
            {
              if ((pte.ppn0() & 0xf) != 0x8)
                return false;
              pte.setPpn0((pte.ppn0() & ~0xf) | (va.vpn0() & 0xf));
            }
        }
      else if (pte.hasNapot())
        return false;
      return true;
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
    uint64_t rootPage_ = 0;        // Root page for S mode (V==0).
    uint64_t vsRootPage_ = 0;  // Root page of VS 1st stage translation (V == 1).
    uint64_t rootPageStage2_ = 0;  // Root page of VS 2nd stage translation (V == 1).
    Mode mode_ = Bare;
    Mode vsMode_ = Bare;
    Mode modeStage2_ = Bare;       // For 2nd stage translation.
    uint32_t asid_ = 0;
    uint32_t vsAsid_ = 0;
    uint32_t vmid_ = 0;
    unsigned pageSize_ = 4096;
    unsigned pageBits_ = 12;
    uint64_t pageMask_ = 0xfff;
    unsigned hartIx_ = 0;

    uint64_t time_ = 0;  //  Access order

    bool trace_ = true;
    bool bigEnd_ = false;
    bool pbmtEnabled_ = false;
    bool napotEnabled_ = false;
    bool pmEnabled_ = false;  // Pointer masking

    std::vector<UpdatedPte> updatedPtes_;

    // Cached mstatus bits
    bool execReadable_ = false;  // MXR bit
    bool s1ExecReadable_ = false;  // MXR bit of vsstatus
    bool sum_ = false;  // Supervisor privilege can access user pages.
    bool vsSum_ = false;  // Supervisor privilege can access user pages for VS mode.
    bool faultOnFirstAccess_ = true;
    bool faultOnFirstAccess2_ = true;
    bool accessDirtyCheck_ = true;  // To be able to supress AD check

    bool xForR_ = false;   // True for hlvx.hu and hlvx.wu instructions: use exec for read

    std::vector<bool> supportedModes_; // Indexed by Mode.

    // Addresses of PTEs used in most recent insruction an data translations.
    using Walk = std::vector<WalkEntry>;
    bool forFetch_ = false;
    std::vector<Walk> fetchWalks_;       // Instruction fetch walks of last instruction.
    std::vector<Walk> dataWalks_;    // Data access walks of last instruction.
    const Walk emptyWalk_;

    // Track page crossing information
    bool fetchPageCross_;
    bool dataPageCross_;

    // Extra trap information
    bool stage1Trap_ = false;
    bool stage1AttemptedADUpdate_ = false;

    Pbmt pbmt_ = Pbmt::None;

    PmpManager& pmpMgr_;
    Tlb tlb_;
    Tlb vsTlb_;
    Tlb stage2Tlb_;
  };

}
