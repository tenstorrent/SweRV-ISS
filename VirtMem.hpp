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


namespace WdRiscv
{


  /// Structure to unpack the fields of a 32-bit page table entry.
  struct Pte32Bits
  {
    unsigned valid_    : 1;
    unsigned read_     : 1;
    unsigned write_    : 1;
    unsigned exec_     : 1;
    unsigned user_     : 1;
    unsigned global_   : 1;
    unsigned accessed_ : 1;
    unsigned dirty_    : 1;
    unsigned rsw_      : 2;   // Reserved for supervisor.
    unsigned ppn0_     : 10;  // Physical page num
    unsigned ppn1_     : 12;  // Physical page num
  } __attribute__((packed));


  /// 32-bit page table entry.
  union Pte32
  {
    Pte32Bits bits_;
    uint32_t data_ = 0;

    /// Constructor: Intialize from the the given data value.
    Pte32(uint32_t word) : data_(word)
    { }

    /// Return true if valid bit is on in this PTE.
    bool valid() const      { return bits_.valid_; }

    /// Return true if read permission bit is on in this PTE.
    bool read() const       { return bits_.read_; }

    /// Return true if write permission bit is on in this PTE.
    bool write() const      { return bits_.write_; }

    /// Return true if execute permission bit is on in this PTE.
    bool exec() const       { return bits_.exec_; }

    /// Return true if this PTE is marked for user privilege.
    bool user() const       { return bits_.user_; }

    /// Return true if this PTE is marked global.
    bool global() const     { return bits_.global_; }

    /// Return true if this PTE is marked accessed.
    bool accessed() const   { return bits_.accessed_; }

    /// Return true if this PTE is marked dirty.
    bool dirty() const      { return bits_.dirty_; }

    /// Return true if this PTE is a leaf.
    bool leaf() const       { return valid() and (read() or exec()); }

    /// Physical page number in this PTE (see Sv32 PTE in the
    /// privileged spec.)
    uint32_t ppn() const    { return ppn0() | (ppn1() << 10); }

    /// Physical page number field 0 in this PTE (see Sv32 PTE in the
    /// privileged spec.)
    uint32_t ppn0() const   { return bits_.ppn0_; }

    /// Physical page number field 1 in this PTE (see Sv32 PTE in the
    /// privileged spec.)
    uint32_t ppn1() const   { return bits_.ppn1_; }

    /// Number of levels of an Sv32 PTE.
    static uint32_t levels() { return 2; }

    /// Size in bytes of this object.
    static uint32_t size()  { return sizeof(data_); }

    /// Page based memory type. NA for Sv32.
    uint32_t pbmt() const   { return 0; }

    /// Naturally aligned power of 2 translation. NA for Sv32.
    bool hasNapot() const  { return false; }

    /// Return the ith physical page number (PPN) field encoded in
    /// this PTE. The index i must be smaller than the number of
    /// levels of Sv32. See the Sv32 PTE in the privileged spec.
    uint32_t ppn(int i) const
    {
      if (i == 0) return ppn0();
      if (i == 1) return ppn1();
      assert(0); return 0;
    }

    /// Return the right shift amount to right justify the ith
    /// physical page number (ppn) in a physical address for Sv32. The
    /// index i must be smaller than the number of levels of Sv32. See
    /// the Sv32 physical address in the privileged spec.
    uint32_t paPpnShift(int i) const
    {
      if (i == 0) return 12;
      if (i == 1) return 22;
      assert(0); return 0;
    }
  };


  /// Struct to unpack the fields of a page table entry for Sv39.
  struct Pte39Bits
  {
    unsigned valid_    : 1;
    unsigned read_     : 1;
    unsigned write_    : 1;
    unsigned exec_     : 1;
    unsigned user_     : 1;
    unsigned global_   : 1;
    unsigned accessed_ : 1;
    unsigned dirty_    : 1;
    unsigned rsw_      : 2;   // Reserved for supervisor.
    unsigned ppn0_     : 9;   // Physical page num
    unsigned ppn1_     : 9;   // Physical page num
    unsigned ppn2_     : 26;  // Physical page num
    unsigned res_      : 7;   // Reserved
    unsigned pbmt_     : 2;   // Page based memory type
    unsigned n_        : 1;
  } __attribute((packed));


  /// Page table entry for Sv39
  union Pte39
  {
    Pte39Bits bits_;
    uint64_t data_ = 0;

    /// Constructor: Intialize from the the given data value.
    Pte39(uint64_t data) : data_(data)
    { }

    /// Return true if valid bit is on in this PTE.
    bool valid() const      { return bits_.valid_; }

    /// Return true if read permission bit is on in this PTE.
    bool read() const       { return bits_.read_; }

    /// Return true if write permission bit is on in this PTE.
    bool write() const      { return bits_.write_; }

    /// Return true if execute permission bit is on in this PTE.
    bool exec() const       { return bits_.exec_; }

    /// Return true if this PTE is marked for user privilege.
    bool user() const       { return bits_.user_; }

    /// Return true if this PTE is marked global.
    bool global() const     { return bits_.global_; }

    /// Return true if this PTE is marked accessed.
    bool accessed() const   { return bits_.accessed_; }

    /// Return true if this PTE is marked dirty.
    bool dirty() const      { return bits_.dirty_; }

    /// Return true if this PTE is a leaf.
    bool leaf() const       { return valid() and (read() or exec()); }

    /// Physical page number in this PTE (see Sv39 PTE in the
    /// privileged spec.)
    uint64_t ppn() const    { return ppn0() | (ppn1() << 9) | (ppn2() << 18); }

    /// Physical page number field 0 in this PTE (see Sv39 PTE in the
    /// privileged spec.)
    uint64_t ppn0() const   { return bits_.ppn0_; }

    /// Physical page number field 1 in this PTE (see Sv39 PTE in the
    /// privileged spec.)
    uint64_t ppn1() const   { return bits_.ppn1_; }

    /// Physical page number field 2 in this PTE (see Sv39 PTE in the
    /// privileged spec.)
    uint64_t ppn2() const   { return bits_.ppn2_; }

    /// Number of levels of an Sv39 PTE.
    static uint32_t levels() { return 3; }

    /// Size in bytes of this object.
    static uint32_t size()   { return sizeof(data_); }

    /// Page based memory type.
    uint32_t pbmt() const   { return bits_.pbmt_; }

    /// Naturally aligned power of 2 translation.
    bool hasNapot() const  { return bits_.n_; }

    /// Return the ith physical page number (PPN) field encoded in
    /// this PTE. The index i must be smaller than the number of
    /// levels of Sv39. See the Sv39 PTE in the privileged spec.
    uint64_t ppn(int i) const
    {
      if (i == 0) { return ppn0(); }
      if (i == 1) { return ppn1(); }
      if (i == 2) { return ppn2(); }
      assert(0);
      return 0;
    }

    /// Return the right shift amount to right justify the ith
    /// physical page number (ppn) in a physical address for Sv39. The
    /// index i must be smaller than the number of levels of Sv39. See
    /// the Sv39 physical address in the privileged spec.
    uint32_t paPpnShift(int i) const
    {
      if (i == 0) { return 12; }
      if (i == 1) { return 21; }
      if (i == 2) { return 30; }
      assert(0);
      return 0;
    }
  };


  /// Struct to unpack the fields of a page table entry for Sv48.
  struct Pte48Bits
  {
    unsigned valid_    : 1;
    unsigned read_     : 1;
    unsigned write_    : 1;
    unsigned exec_     : 1;
    unsigned user_     : 1;
    unsigned global_   : 1;
    unsigned accessed_ : 1;
    unsigned dirty_    : 1;
    unsigned rsw_      : 2;   // Reserved for supervisor.
    unsigned ppn0_     : 9;   // Physical page num
    unsigned ppn1_     : 9;   // Physical page num
    unsigned ppn2_     : 9;   // Physical page num
    unsigned ppn3_     : 17;  // Physical page num
    unsigned res_      : 7;   // Reserved
    unsigned pbmt_     : 2;   // Page based memory type
    unsigned n_        : 1;
  } __attribute__((packed));


  /// Page table entry for Sv48
  union Pte48
  {
    Pte48Bits bits_;
    uint64_t data_ = 0;

    /// Constructor: Intialize from the the given data value.
    Pte48(uint64_t data) : data_(data)
    { }

    /// Return true if valid bit is on in this PTE.
    bool valid() const      { return bits_.valid_; }

    /// Return true if read permission bit is on in this PTE.
    bool read() const       { return bits_.read_; }

    /// Return true if write permission bit is on in this PTE.
    bool write() const      { return bits_.write_; }

    /// Return true if execute permission bit is on in this PTE.
    bool exec() const       { return bits_.exec_; }

    /// Return true if this PTE is marked for user privilege.
    bool user() const       { return bits_.user_; }

    /// Return true if this PTE is marked global.
    bool global() const     { return bits_.global_; }

    /// Return true if this PTE is marked accessed.
    bool accessed() const   { return bits_.accessed_; }

    /// Return true if this PTE is marked dirty.
    bool dirty() const      { return bits_.dirty_; }

    /// Return true if this PTE is a leaf.
    bool leaf() const       { return valid() and (read() or exec()); }

    /// Physical page number in this PTE (see Sv48 PTE in the
    /// privileged spec.)
    uint64_t ppn() const    { return ppn0() | (ppn1() << 9) | (ppn2() << 18) | (ppn3() << 27); }

    /// Physical page number field 0 in this PTE (see Sv48 PTE in the
    /// privileged spec.)
    uint64_t ppn0() const   { return bits_.ppn0_; }

    /// Physical page number field 1 in this PTE (see Sv48 PTE in the
    /// privileged spec.)
    uint64_t ppn1() const   { return bits_.ppn1_; }

    /// Physical page number field 2 in this PTE (see Sv48 PTE in the
    /// privileged spec.)
    uint64_t ppn2() const   { return bits_.ppn2_; }

    /// Physical page number field 3 in this PTE (see Sv48 PTE in the
    /// privileged spec.)
    uint64_t ppn3() const   { return bits_.ppn3_; }

    /// Number of levels of an Sv48 PTE.
    static uint32_t levels() { return 4; }

    /// Size in bytes of this object.
    static uint32_t size()   { return sizeof(data_); }

    /// Page based memory type.
    uint32_t pbmt() const   { return bits_.pbmt_; }

    /// Naturally aligned power of 2 translation.
    bool hasNapot() const  { return bits_.n_; }

    /// Return the ith physical page number (PPN) field encoded in
    /// this PTE. The index i must be smaller than the number of
    /// levels of Sv48. See the Sv48 PTE in the privileged spec.
    uint64_t ppn(int i) const
    {
      if (i == 0) { return ppn0(); }
      if (i == 1) { return ppn1(); }
      if (i == 2) { return ppn2(); }
      if (i == 3) { return ppn3(); }
      assert(0);
      return 0;
    }

    /// Return the right shift amount to right justify the ith
    /// physical page number (ppn) in a physical address for Sv48. The
    /// index i must be smaller than the number of levels of Sv48. See
    /// the Sv48 physical address in the privileged spec.
    uint32_t paPpnShift(int i) const
    {
      if (i == 0) { return 12; }
      if (i == 1) { return 21; }
      if (i == 2) { return 30; }
      if (i == 3) { return 39; }
      assert(0);
      return 0;
    }
  };


  /// Struct to unpack the fields of a page table entry for Sv57.
  struct Pte57Bits
  {
    unsigned valid_    : 1;
    unsigned read_     : 1;
    unsigned write_    : 1;
    unsigned exec_     : 1;
    unsigned user_     : 1;
    unsigned global_   : 1;
    unsigned accessed_ : 1;
    unsigned dirty_    : 1;
    unsigned rsw_      : 2;   // Reserved for supervisor.
    unsigned ppn0_     : 9;   // Physical page num
    unsigned ppn1_     : 9;   // Physical page num
    unsigned ppn2_     : 9;   // Physical page num
    unsigned ppn3_     : 9;   // Physical page num
    unsigned ppn4_     : 8;   // Physical page num
    unsigned res_      : 7;   // Reserved
    unsigned pbmt_     : 2;   // Page based memory type.
    unsigned n_        : 1;
  } __attribute__((packed));


  /// Page table entry for Sv57
  union Pte57
  {
    Pte57Bits bits_;
    uint64_t data_ = 0;

    /// Constructor: Intialize from the the given data value.
    Pte57(uint64_t data) : data_(data)
    { }

    /// Return true if valid bit is on in this PTE.
    bool valid() const      { return bits_.valid_; }

    /// Return true if read permission bit is on in this PTE.
    bool read() const       { return bits_.read_; }

    /// Return true if write permission bit is on in this PTE.
    bool write() const      { return bits_.write_; }

    /// Return true if execute permission bit is on in this PTE.
    bool exec() const       { return bits_.exec_; }

    /// Return true if this PTE is marked for user privilege.
    bool user() const       { return bits_.user_; }

    /// Return true if this PTE is marked global.
    bool global() const     { return bits_.global_; }

    /// Return true if this PTE is marked accessed.
    bool accessed() const   { return bits_.accessed_; }

    /// Return true if this PTE is marked dirty.
    bool dirty() const      { return bits_.dirty_; }

    /// Return true if this PTE is a leaf.
    bool leaf() const       { return valid() and (read() or exec()); }

    /// Physical page number in this PTE (see Sv57 PTE in the
    /// privileged spec.)
    uint64_t ppn() const    { return ppn0() | (ppn1() << 9) | (ppn2() << 18) | (ppn3() << 27); }

    /// Physical page number field 0 in this PTE (see Sv57 PTE in the
    /// privileged spec.)
    uint64_t ppn0() const   { return bits_.ppn0_; }

    /// Physical page number field 1 in this PTE (see Sv57 PTE in the
    /// privileged spec.)
    uint64_t ppn1() const   { return bits_.ppn1_; }

    /// Physical page number field 2 in this PTE (see Sv57 PTE in the
    /// privileged spec.)
    uint64_t ppn2() const   { return bits_.ppn2_; }

    /// Physical page number field 3 in this PTE (see Sv57 PTE in the
    /// privileged spec.)
    uint64_t ppn3() const   { return bits_.ppn3_; }

    /// Physical page number field 4 in this PTE (see Sv57 PTE in the
    /// privileged spec.)
    uint64_t ppn4() const   { return bits_.ppn4_; }

    /// Number of levels of an Sv57 PTE.
    static uint32_t levels() { return 5; }

    /// Size in bytes of this object.
    static uint32_t size()  { return sizeof(data_); }

    /// Page based memory type.
    uint32_t pbmt() const   { return bits_.pbmt_; }

    /// Naturally aligned power of 2 translation.
    bool hasNapot() const  { return bits_.n_; }

    /// Return the ith physical page number (PPN) field encoded in
    /// this PTE. The index i must be smaller than the number of
    /// levels of Sv57. See the Sv57 PTE in the privileged spec.
    uint64_t ppn(int i) const
    {
      if (i == 0) { return ppn0(); }
      if (i == 1) { return ppn1(); }
      if (i == 2) { return ppn2(); }
      if (i == 3) { return ppn3(); }
      if (i == 4) { return ppn4(); }
      assert(0);
      return 0;
    }

    /// Return the right shift amount to right justify the ith
    /// physical page number (ppn) in a physical address for Sv57. The
    /// index i must be smaller than the number of levels of Sv57. See
    /// the Sv57 physical address in the privileged spec.
    uint32_t paPpnShift(int i) const
    {
      if (i == 0) { return 12; }
      if (i == 1) { return 21; }
      if (i == 2) { return 30; }
      if (i == 3) { return 39; }
      if (i == 4) { return 48; }
      assert(0);
      return 0;
    }
  };


  /// Structure to unpack the fields of 32-bit virtual address.
  struct Va32Bits
  {
    unsigned offset_ : 12;
    unsigned vpn0_   : 10;
    unsigned vpn1_   : 10;
  } __attribute__((packed));


  /// 32-bit virtual address.
  union Va32
  {
    Va32Bits bits_;
    uint32_t data_ = 0;

    Va32(uint32_t word) : data_(word)
    { }

    uint32_t offset() const { return bits_.offset_; }

    uint32_t vpn0() const   { return bits_.vpn0_; }

    uint32_t vpn1() const   { return bits_.vpn1_; }

    uint32_t vpn(int i) const
    {
      if (i == 0) return vpn0();
      if (i == 1) return vpn1();
      assert(0);
      return 0;
    }
  };


  /// Structure to unpack the fields of Sv39 virtual address.
  struct Va39Bits
  {
    unsigned offset_ : 12;
    unsigned vpn0_   : 9;
    unsigned vpn1_   : 9;
    unsigned vpn2_   : 9;
  } __attribute__((packed));


  /// 39-bit virtual address.
  union Va39
  {
    Va39Bits bits_;
    uint64_t data_ = 0;

    Va39(uint64_t data) : data_(data)
    { }

    uint64_t offset() const { return bits_.offset_; }

    uint64_t vpn0() const   { return bits_.vpn0_; }

    uint64_t vpn1() const   { return bits_.vpn1_; }

    uint64_t vpn2() const   { return bits_.vpn2_; }

    uint64_t vpn(int i) const
    {
      if (i == 0) return vpn0();
      if (i == 1) return vpn1();
      if (i == 2) return vpn2();
      assert(0);
      return 0;
    }
  };


  /// Structure to unpack the fields of Sv48 virtual address.
  struct Va48Bits
  {
    unsigned offset_ : 12;
    unsigned vpn0_   : 9;
    unsigned vpn1_   : 9;
    unsigned vpn2_   : 9;
    unsigned vpn3_   : 9;
  } __attribute__((packed));


  /// 48-bit virtual address.
  union Va48
  {
    Va48Bits bits_;
    uint64_t data_ = 0;

    Va48(uint64_t data) : data_(data)
    { }

    uint64_t offset() const { return bits_.offset_; }

    uint64_t vpn0() const   { return bits_.vpn0_; }

    uint64_t vpn1() const   { return bits_.vpn1_; }

    uint64_t vpn2() const   { return bits_.vpn2_; }

    uint64_t vpn3() const   { return bits_.vpn3_; }

    uint64_t vpn(int i) const
    {
      if (i == 0) return vpn0();
      if (i == 1) return vpn1();
      if (i == 2) return vpn2();
      if (i == 3) return vpn3();
      assert(0);
      return 0;
    }
  };


  /// Structure to unpack the fields of Sv57 virtual address.
  struct Va57Bits
  {
    unsigned offset_ : 12;
    unsigned vpn0_   : 9;
    unsigned vpn1_   : 9;
    unsigned vpn2_   : 9;
    unsigned vpn3_   : 9;
    unsigned vpn4_   : 9;
  } __attribute__((packed));


  /// 57-bit virtual address.
  union Va57
  {
    Va57Bits bits_;
    uint64_t data_ = 0;

    Va57(uint64_t data) : data_(data)
    { }

    uint64_t offset() const { return bits_.offset_; }

    uint64_t vpn0() const   { return bits_.vpn0_; }

    uint64_t vpn1() const   { return bits_.vpn1_; }

    uint64_t vpn2() const   { return bits_.vpn2_; }

    uint64_t vpn3() const   { return bits_.vpn3_; }

    uint64_t vpn4() const   { return bits_.vpn4_; }

    uint64_t vpn(int i) const
    {
      if (i == 0) return vpn0();
      if (i == 1) return vpn1();
      if (i == 2) return vpn2();
      if (i == 3) return vpn3();
      if (i == 4) return vpn4();
      assert(0);
      return 0;
    }
  };


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
