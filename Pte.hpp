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
#include <cassert>

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

    /// Return rsw value (reserved for supervisor).
    uint64_t rsw() const    { return bits_.rsw_; }

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

    /// Return reserved bits value. NA for Sv32.
    uint64_t res() const    { return 0; }

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

    /// Set the physical phage number (ppn1,ppn0) to the least
    /// significant 22 bits of value.
    void setPpn(uint64_t value)
    { bits_.ppn0_ = value & 0x3ff; bits_.ppn1_ = (value >> 10) & 0xfff; }

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
  } __attribute__((packed));


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

    /// Return rsw value (reserved for supervisor).
    uint64_t rsw() const    { return bits_.rsw_; }

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

    /// Return reserved bits value
    uint64_t res() const    { return bits_.res_; }

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

    /// Set the physical phage number (ppn2,ppn1,ppn0) to the least
    /// significant 44 bits of value.
    void setPpn(uint64_t value)
    {
      bits_.ppn0_ = value & 0x1ff;
      bits_.ppn1_ = (value >> 9) & 0x1ff;
      bits_.ppn2_ = (value >> 18) & 0x3ffffff;
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

    /// Return rsw value (reserved for supervisor).
    uint64_t rsw() const    { return bits_.rsw_; }

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

    /// Return reserved bits value
    uint64_t res() const    { return bits_.res_; }

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

    /// Set the physical phage number (ppn3,ppn2,ppn1,ppn0) to the least
    /// significant 44 bits of value.
    void setPpn(uint64_t value)
    {
      bits_.ppn0_ = value & 0x1ff;
      bits_.ppn1_ = (value >> 9) & 0x1ff;
      bits_.ppn2_ = (value >> 18) & 0x1ff;
      bits_.ppn3_ = (value >> 27) & 0x1ffff;
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

    /// Return rsw value (reserved for supervisor).
    uint64_t rsw() const    { return bits_.rsw_; }

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

    /// Return reserved bits value
    uint64_t res() const    { return bits_.res_; }

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

    /// Set the physical phage number (ppn4,ppn3,ppn2,ppn1,ppn0) to the least
    /// significant 44 bits of value.
    void setPpn(uint64_t value)
    {
      bits_.ppn0_ = value & 0x1ff;
      bits_.ppn1_ = (value >> 9) & 0x1ff;
      bits_.ppn2_ = (value >> 18) & 0x1ff;
      bits_.ppn3_ = (value >> 27) & 0x1ff;
      bits_.ppn4_ = (value >> 36) & 0xff;
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


  /// Structure to unpack the fields of an Sv32x4 virtual address
  /// (guest physical address).
  struct Va32x4Bits
  {
    unsigned offset_ : 12;
    unsigned vpn0_   : 10;
    unsigned vpn1_   : 12;
  } __attribute__((packed));


  /// Sv32x4 virtual address (guest physical address).
  union Va32x4
  {
    Va32x4Bits bits_;
    uint64_t data_ = 0;

    Va32x4(uint32_t word) : data_(word)
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


  /// Structure to unpack the fields of Sv39x4 virtual address.
  struct Va39x4Bits
  {
    unsigned offset_ : 12;
    unsigned vpn0_   : 9;
    unsigned vpn1_   : 9;
    unsigned vpn2_   : 11;
  } __attribute__((packed));


  /// Sv39x4 virtual address.
  union Va39x4
  {
    Va39x4Bits bits_;
    uint64_t data_ = 0;

    Va39x4(uint64_t data) : data_(data)
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


  /// Structure to unpack the fields of Sv48x4 virtual address.
  struct Va48x4Bits
  {
    unsigned offset_ : 12;
    unsigned vpn0_   : 9;
    unsigned vpn1_   : 9;
    unsigned vpn2_   : 9;
    unsigned vpn3_   : 11;
  } __attribute__((packed));


  /// Sv48x4 virtual address.
  union Va48x4
  {
    Va48x4Bits bits_;
    uint64_t data_ = 0;

    Va48x4(uint64_t data) : data_(data)
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


  /// Structure to unpack the fields of Sv57x4 virtual address.
  struct Va57x4Bits
  {
    unsigned offset_ : 12;
    unsigned vpn0_   : 9;
    unsigned vpn1_   : 9;
    unsigned vpn2_   : 9;
    unsigned vpn3_   : 9;
    unsigned vpn4_   : 11;
  } __attribute__((packed));


  /// Sv57x4 virtual address.
  union Va57x4
  {
    Va57x4Bits bits_;
    uint64_t data_ = 0;

    Va57x4(uint64_t data) : data_(data)
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


}
