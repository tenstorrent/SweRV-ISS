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
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <functional>
#include <cassert>
#include "Triggers.hpp"
#include "PerfRegs.hpp"
#include "CsrFields.hpp"
#include "Imsic.hpp"
#include "util.hpp"


namespace WdRiscv
{

  /// Control and status register number.
  enum class CsrNumber : uint32_t
    {
      // Machine mode registers.

      // Machine info.
      MVENDORID = 0xF11,
      MARCHID = 0xF12,
      MIMPID = 0xF13,
      MHARTID = 0xF14,
      MCONFIGPTR = 0xF15,

      // Machine trap setup.
      MSTATUS = 0x300,
      MISA = 0x301,
      MEDELEG = 0x302,
      MIDELEG = 0x303,
      MIE = 0x304,
      MTVEC = 0x305,
      MCOUNTEREN = 0x306,
      MSTATUSH = 0x310,
      MCOUNTINHIBIT = 0x320,

      // Machine trap handling
      MSCRATCH = 0x340,
      MEPC = 0x341,
      MCAUSE = 0x342,
      MTVAL = 0x343,
      MIP = 0x344,
      MTINST = 0x34A,
      MTVAL2 = 0x34B,

      // Machine protection and translation.
      PMPCFG0 = 0x3a0,
      PMPCFG15 = 0x3af,
      PMPADDR0 = 0x3b0,
      PMPADDR63 = 0x3ef,
      MCYCLECFG = 0x321,
      MINSTRETCFG = 0x322,
      MCYCLECFGH = 0x721,
      MINSTRETCFGH = 0x722,

      MENVCFG = 0x30a,
      MENVCFGH = 0x31a,

      // Non maskable interrupts
      MNSCRATCH = 0x740,
      MNEPC = 0x741,
      MNCAUSE = 0x742,
      MNSTATUS = 0x744,

      // Machine Counter/Timers
      MCYCLE = 0xb00,
      MINSTRET = 0xb02,
      MHPMCOUNTER3 = 0xb03,
      MHPMCOUNTER4 = 0xb04,
      MHPMCOUNTER5 = 0xb05,
      MHPMCOUNTER6 = 0xb06,
      MHPMCOUNTER7 = 0xb07,
      MHPMCOUNTER8 = 0xb08,
      MHPMCOUNTER9 = 0xb09,
      MHPMCOUNTER10 = 0xb0a,
      MHPMCOUNTER11 = 0xb0b,
      MHPMCOUNTER12 = 0xb0c,
      MHPMCOUNTER13 = 0xb0d,
      MHPMCOUNTER14 = 0xb0e,
      MHPMCOUNTER15 = 0xb0f,
      MHPMCOUNTER16 = 0xb10,
      MHPMCOUNTER17 = 0xb11,
      MHPMCOUNTER18 = 0xb12,
      MHPMCOUNTER19 = 0xb13,
      MHPMCOUNTER20 = 0xb14,
      MHPMCOUNTER21 = 0xb15,
      MHPMCOUNTER22 = 0xb16,
      MHPMCOUNTER23 = 0xb17,
      MHPMCOUNTER24 = 0xb18,
      MHPMCOUNTER25 = 0xb19,
      MHPMCOUNTER26 = 0xb1a,
      MHPMCOUNTER27 = 0xb1b,
      MHPMCOUNTER28 = 0xb1c,
      MHPMCOUNTER29 = 0xb1d,
      MHPMCOUNTER30 = 0xb1e,
      MHPMCOUNTER31 = 0xb1f,

      MCYCLEH = 0xb80,
      MINSTRETH = 0xb82,
      MHPMCOUNTER3H = 0xb83,
      MHPMCOUNTER4H = 0xb84,
      MHPMCOUNTER5H = 0xb85,
      MHPMCOUNTER6H = 0xb86,
      MHPMCOUNTER7H = 0xb87,
      MHPMCOUNTER8H = 0xb88,
      MHPMCOUNTER9H = 0xb89,
      MHPMCOUNTER10H = 0xb8a,
      MHPMCOUNTER11H = 0xb8b,
      MHPMCOUNTER12H = 0xb8c,
      MHPMCOUNTER13H = 0xb8d,
      MHPMCOUNTER14H = 0xb8e,
      MHPMCOUNTER15H = 0xb8f,
      MHPMCOUNTER16H = 0xb90,
      MHPMCOUNTER17H = 0xb91,
      MHPMCOUNTER18H = 0xb92,
      MHPMCOUNTER19H = 0xb93,
      MHPMCOUNTER20H = 0xb94,
      MHPMCOUNTER21H = 0xb95,
      MHPMCOUNTER22H = 0xb96,
      MHPMCOUNTER23H = 0xb97,
      MHPMCOUNTER24H = 0xb98,
      MHPMCOUNTER25H = 0xb99,
      MHPMCOUNTER26H = 0xb9a,
      MHPMCOUNTER27H = 0xb9b,
      MHPMCOUNTER28H = 0xb9c,
      MHPMCOUNTER29H = 0xb9d,
      MHPMCOUNTER30H = 0xb9e,
      MHPMCOUNTER31H = 0xb9f,

      // Machine counter setup.
      MHPMEVENT3 = 0x323,
      MHPMEVENT4 = 0x324,
      MHPMEVENT5 = 0x325,
      MHPMEVENT6 = 0x326,
      MHPMEVENT7 = 0x327,
      MHPMEVENT8 = 0x328,
      MHPMEVENT9 = 0x329,
      MHPMEVENT10 = 0x32a,
      MHPMEVENT11 = 0x32b,
      MHPMEVENT12 = 0x32c,
      MHPMEVENT13 = 0x32d,
      MHPMEVENT14 = 0x32e,
      MHPMEVENT15 = 0x32f,
      MHPMEVENT16 = 0x330,
      MHPMEVENT17 = 0x331,
      MHPMEVENT18 = 0x332,
      MHPMEVENT19 = 0x333,
      MHPMEVENT20 = 0x334,
      MHPMEVENT21 = 0x335,
      MHPMEVENT22 = 0x336,
      MHPMEVENT23 = 0x337,
      MHPMEVENT24 = 0x338,
      MHPMEVENT25 = 0x339,
      MHPMEVENT26 = 0x33a,
      MHPMEVENT27 = 0x33b,
      MHPMEVENT28 = 0x33c,
      MHPMEVENT29 = 0x33d,
      MHPMEVENT30 = 0x33e,
      MHPMEVENT31 = 0x33f,

      MHPMEVENTH3 = 0x723,
      MHPMEVENTH4 = 0x724,
      MHPMEVENTH5 = 0x725,
      MHPMEVENTH6 = 0x726,
      MHPMEVENTH7 = 0x727,
      MHPMEVENTH8 = 0x728,
      MHPMEVENTH9 = 0x729,
      MHPMEVENTH10 = 0x72a,
      MHPMEVENTH11 = 0x72b,
      MHPMEVENTH12 = 0x72c,
      MHPMEVENTH13 = 0x72d,
      MHPMEVENTH14 = 0x72e,
      MHPMEVENTH15 = 0x72f,
      MHPMEVENTH16 = 0x730,
      MHPMEVENTH17 = 0x731,
      MHPMEVENTH18 = 0x732,
      MHPMEVENTH19 = 0x733,
      MHPMEVENTH20 = 0x734,
      MHPMEVENTH21 = 0x735,
      MHPMEVENTH22 = 0x736,
      MHPMEVENTH23 = 0x737,
      MHPMEVENTH24 = 0x738,
      MHPMEVENTH25 = 0x739,
      MHPMEVENTH26 = 0x73a,
      MHPMEVENTH27 = 0x73b,
      MHPMEVENTH28 = 0x73c,
      MHPMEVENTH29 = 0x73d,
      MHPMEVENTH30 = 0x73e,
      MHPMEVENTH31 = 0x73f,

      // Supervisor mode registers.

      // Supervisor trap setup.
      SSTATUS = 0x100,
      SIE = 0x104,
      STVEC = 0x105,
      SCOUNTEREN = 0x106,

      SENVCFG = 0x10a,
      SCOUNTOVF = 0xda0,

      // Supervisor Trap Handling 
      SSCRATCH = 0x140,
      SEPC = 0x141,
      SCAUSE = 0x142,
      STVAL = 0x143,
      SIP = 0x144,
      STIMECMP = 0x14d,
      STIMECMPH = 0x15d,
      // Supervisor Protection and Translation 
      SATP = 0x180,

      // Hypervisor registers
      HSTATUS = 0x600,
      HEDELEG = 0x602,
      HIDELEG = 0x603,
      HIE = 0x604,
      HCOUNTEREN = 0x606,
      HGEIE = 0x607,
      HTVAL = 0x643,
      HIP = 0x644,
      HVIP = 0x645,
      HTINST = 0x64A,
      HGEIP = 0xE12,
      HENVCFG = 0x60A,
      HENVCFGH = 0x61A,
      HGATP = 0x680,
      HCONTEXT = 0x6A8,
      HTIMEDELTA = 0x605,
      HTIMEDELTAH = 0x615,

      // Virtual supervisor
      VSSTATUS = 0x200,
      VSIE = 0x204,
      VSTVEC = 0x205,
      VSSCRATCH = 0x240,
      VSEPC = 0x241,
      VSCAUSE = 0x242,
      VSTVAL = 0x243,
      VSIP = 0x244,
      VSTIMECMP = 0x24d,
      VSTIMECMPH = 0x25d,
      VSATP = 0x280,

      // User mode registers.

      // User Floating-Point CSRs
      FFLAGS = 0x001,
      FRM = 0x002,
      FCSR = 0x003,

      // User Counter/Timers
      CYCLE = 0xc00,
      TIME = 0xc01,
      INSTRET = 0xc02,
      HPMCOUNTER3 = 0xc03,
      HPMCOUNTER4 = 0xc04,
      HPMCOUNTER5 = 0xc05,
      HPMCOUNTER6 = 0xc06,
      HPMCOUNTER7 = 0xc07,
      HPMCOUNTER8 = 0xc08,
      HPMCOUNTER9 = 0xc09,
      HPMCOUNTER10 = 0xc0a,
      HPMCOUNTER11 = 0xc0b,
      HPMCOUNTER12 = 0xc0c,
      HPMCOUNTER13 = 0xc0d,
      HPMCOUNTER14 = 0xc0e,
      HPMCOUNTER15 = 0xc0f,
      HPMCOUNTER16 = 0xc10,
      HPMCOUNTER17 = 0xc11,
      HPMCOUNTER18 = 0xc12,
      HPMCOUNTER19 = 0xc13,
      HPMCOUNTER20 = 0xc14,
      HPMCOUNTER21 = 0xc15,
      HPMCOUNTER22 = 0xc16,
      HPMCOUNTER23 = 0xc17,
      HPMCOUNTER24 = 0xc18,
      HPMCOUNTER25 = 0xc19,
      HPMCOUNTER26 = 0xc1a,
      HPMCOUNTER27 = 0xc1b,
      HPMCOUNTER28 = 0xc1c,
      HPMCOUNTER29 = 0xc1d,
      HPMCOUNTER30 = 0xc1e,
      HPMCOUNTER31 = 0xc1f,

      CYCLEH = 0xc80,
      TIMEH = 0xc81,
      INSTRETH = 0xc82,
      HPMCOUNTER3H = 0xc83,
      HPMCOUNTER4H = 0xc84,
      HPMCOUNTER5H = 0xc85,
      HPMCOUNTER6H = 0xc86,
      HPMCOUNTER7H = 0xc87,
      HPMCOUNTER8H = 0xc88,
      HPMCOUNTER9H = 0xc89,
      HPMCOUNTER10H = 0xc8a,
      HPMCOUNTER11H = 0xc8b,
      HPMCOUNTER12H = 0xc8c,
      HPMCOUNTER13H = 0xc8d,
      HPMCOUNTER14H = 0xc8e,
      HPMCOUNTER15H = 0xc8f,
      HPMCOUNTER16H = 0xc90,
      HPMCOUNTER17H = 0xc91,
      HPMCOUNTER18H = 0xc92,
      HPMCOUNTER19H = 0xc93,
      HPMCOUNTER20H = 0xc94,
      HPMCOUNTER21H = 0xc95,
      HPMCOUNTER22H = 0xc96,
      HPMCOUNTER23H = 0xc97,
      HPMCOUNTER24H = 0xc98,
      HPMCOUNTER25H = 0xc99,
      HPMCOUNTER26H = 0xc9a,
      HPMCOUNTER27H = 0xc9b,
      HPMCOUNTER28H = 0xc9c,
      HPMCOUNTER29H = 0xc9d,
      HPMCOUNTER30H = 0xc9e,
      HPMCOUNTER31H = 0xc9f,

      // Debug/Trace registers.
      SCONTEXT = 0x5a8,
      TSELECT  = 0x7a0,
      TDATA1   = 0x7a1,
      TDATA2   = 0x7a2,
      TDATA3   = 0x7a3,
      TINFO    = 0x7a4,
      TCONTROL = 0x7a5,
      MCONTEXT = 0x7a8,

      // Debug mode registers.
      DCSR      = 0x7b0,
      DPC       = 0x7b1,
      DSCRATCH0 = 0x7b2,
      DSCRATCH1 = 0x7b3,

      // Vector extension register.
      VSTART   = 0x008,
      VXSAT    = 0x009,
      VXRM     = 0x00a,
      VCSR     = 0x00f,
      VL       = 0xc20,
      VTYPE    = 0xc21,
      VLENB    = 0xc22,

      // Advanced interrupt architecture (AIA)
      MISELECT   = 0x350,
      MIREG      = 0x351,
      MTOPEI     = 0x35c,
      MTOPI      = 0xFB0,
      MVIEN      = 0x308,
      MVIP       = 0x309,
      MIDELEGH   = 0x313,
      MIEH       = 0x314,
      MVIENH     = 0x318,
      MVIPH      = 0x319,
      MIPH       = 0x354,
      SISELECT   = 0x150,
      SIREG      = 0x151,
      STOPEI     = 0x15c,
      STOPI      = 0xdb0,
      SIEH       = 0x114,
      SIPH       = 0x154,
      HVIEN      = 0x608,
      HVICTL     = 0x609,
      HVIPRIO1   = 0x646,
      HVIPRIO2   = 0x647,
      VSISELECT  = 0x250,
      VSIREG     = 0x251,
      VSTOPEI    = 0x25c,
      VSTOPI     = 0xeb0,
      HIDELEGH   = 0x613,
      HVIENH     = 0x618,
      HVIPH      = 0x655,
      HVIPRIO1H  = 0x656,
      HVIPRIO2H  = 0x657,
      VSIEH      = 0x214,
      VSIPH      = 0x254,

      // Stateen CSR
      SSTATEEN0   = 0x10c,
      SSTATEEN1   = 0x10d,
      SSTATEEN2   = 0x10e,
      SSTATEEN3   = 0x10f,
      MSTATEEN0   = 0x30c,
      MSTATEEN1   = 0x30d,
      MSTATEEN2   = 0x30e,
      MSTATEEN3   = 0x30f,
      MSTATEEN0H  = 0x31c,
      MSTATEEN1H  = 0x31d,
      MSTATEEN2H  = 0x31e,
      MSTATEEN3H  = 0x31f,
      HSTATEEN0   = 0x60c,
      HSTATEEN1   = 0x60d,
      HSTATEEN2   = 0x60e,
      HSTATEEN3   = 0x60f,
      HSTATEEN0H  = 0x61c,
      HSTATEEN1H  = 0x61d,
      HSTATEEN2H  = 0x61e,
      HSTATEEN3H  = 0x61f,

      // Tenstorrent Ascalon CSRs
      PMACFG0  = 0x7e0,   // Physical memory protection
      PMACFG31 = 0x7ff,
      PMACFG32 = 0xbe0,
      PMACFG63 = 0xbff,
      MATP     = 0x7c7,   // Machine address translation and protection

      MAX_CSR_ = 0xfff,
      MIN_CSR_ = 0      // csr with smallest number
    };


  /// Return true if first csr number is smaller than the second.
  inline bool operator< (CsrNumber a, CsrNumber b)
  { return static_cast<unsigned>(a) < static_cast<unsigned>(b); }

  /// Return true if first csr number is smaller than or equal the second.
  inline bool operator<= (CsrNumber a, CsrNumber b)
  { return static_cast<unsigned>(a) <= static_cast<unsigned>(b); }

  /// Return true if first csr number is greater than the second.
  inline bool operator> (CsrNumber a, CsrNumber b)
  { return static_cast<unsigned>(a) > static_cast<unsigned>(b); }

  /// Return true if first csr number is greater than or equal the second.
  inline bool operator>= (CsrNumber a, CsrNumber b)
  { return static_cast<unsigned>(a) >= static_cast<unsigned>(b); }


  template <typename URV>
  class CsRegs;

  /// Model a control and status register. The template type URV
  /// (unsigned register value) is the type of the register value. It
  /// must be uint32_t for 32-bit implementations and uint64_t for
  /// 64-bit.
  template <typename URV>
  class Csr
  {
  public:

    /// Default constructor.
    Csr()
    { valuePtr_ = &value_; }

    /// Constructor. The mask indicates which bits are writable: A zero bit
    /// in the mask corresponds to a non-writable (preserved) bit in the
    /// register value. To make the whole register writable, set mask to
    /// all ones.
    Csr(std::string name, CsrNumber number, bool mandatory,
	bool implemented, URV value, URV writeMask = ~URV(0))
      : name_(std::move(name)), number_(unsigned(number)), mandatory_(mandatory),
	implemented_(implemented), initialValue_(value), value_(value),
	writeMask_(writeMask), pokeMask_(writeMask)
    {
      valuePtr_ = &value_;
      privMode_ = PrivilegeMode((number_ & 0x300) >> 8);
    }

    /// Copy constructor is not available.
    Csr(const Csr<URV>& other) = delete;

    void operator=(const Csr<URV>& other) = delete;

    /// Return lowest privilege mode that can access the register.
    /// Bits 9 and 8 of the register number encode the privilege mode.
    /// Privilege of user level performance counter is modified by
    /// mcounteren.
    PrivilegeMode privilegeMode() const
    { return privMode_; }

    /// Return true if register is read-only. Bits ten and eleven of
    /// the register number denote read-only when both one and read-write
    /// otherwise.
    bool isReadOnly() const
    { return (number_ & 0xc00) == 0xc00 or number_ == unsigned(CsrNumber::HGEIP); }

    /// Return true if register is implemented.
    bool isImplemented() const
    { return implemented_; }

    /// Return true if register is mandatory (not optional).
    bool isMandatory() const
    { return mandatory_; }

    /// Return true if this is a hypervisor register.
    bool isHypervisor() const
    { return hyper_; }

    /// Return true if this is a supervisor register that maps to a
    /// virtual supervisor register (e.g. sstatus maps to vsstatus).
    bool mapsToVirtual() const
    { return mapsToVirtual_; }

    /// Return true if this is a high-half of a CSR (e.g. MSTATUSH is
    /// the high half of MSTATUS).
    bool isHighHalf() const
    { return high_; }

    /// Mark this CSR as a high-half (e.g. MSTATUSH is a high half).
    void markAsHighHalf(bool flag)
    { high_ = flag; }

    /// Return true if this register has been marked as a debug-mode
    /// register.
    bool isDebug() const
    { return debug_; }

    /// Return true if this register is shared among harts.
    bool isShared() const
    { return shared_; }

    /// Return the current value of this register.
    URV read() const
    { return *valuePtr_ & readMask_; }

    /// Return the write-mask associated with this register. A
    /// register value bit is writable by the write method if and only
    /// if the corresponding bit in the mask is 1; otherwise, the bit
    /// is preserved.
    URV getWriteMask() const
    { return writeMask_; }

    /// Return the poke mask associated with this register. A register
    /// value bit is modifiable if and only if the corresponding bit
    /// in the mask is 1; otherwise, the bit is preserved. The write
    /// mask is used by the CSR write instructions. The poke mask
    /// allows the caller to change bits that are read only for CSR
    /// instructions but are modifiable by the hardware.
    URV getPokeMask() const
    { return pokeMask_; }

    URV getReadMask() const
    { return readMask_; }

    /// Return the reset value of this CSR.
    URV getResetValue() const
    { return initialValue_; }

    /// Return the number of this register.
    CsrNumber getNumber() const
    { return CsrNumber(number_); }

    /// Return the name of this register.
    std::string_view getName() const
    { return name_; }

    /// Register a pre-poke call back which will get invoked with CSR and
    /// poked value.
    void registerPrePoke(std::function<void(Csr<URV>&, URV&)> func)
    { prePoke_.push_back(func); }

    /// Register a pre-write call back which will get invoked with
    /// CSR and written value.
    void registerPreWrite(std::function<void(Csr<URV>&, URV&)> func)
    { preWrite_.push_back(func); }

    /// Register a post-poke call back which will get invoked with CSR and
    /// poked value.
    void registerPostPoke(std::function<void(Csr<URV>&, URV)> func)
    { postPoke_.push_back(func); }

    /// Register a post-write call back which will get invoked with
    /// CSR and written value.
    void registerPostWrite(std::function<void(Csr<URV>&, URV)> func)
    { postWrite_.push_back(func); }

    /// Register a post-reset call back.
    void registerPostReset(std::function<void(Csr<URV>&)> func)
    { postReset_.push_back(func); }

    /// Get field width of CSR
    unsigned width(std::string_view field) const
    {
      for (auto& f : fields_)
        {
          if (f.field == field)
            return f.width;
        }
    }

    struct Field
    {
      std::string field;
      unsigned width;
    };

    // Returns CSR fields
    const std::vector<Field>& fields() const
    { return fields_; }

  protected:

    friend class CsRegs<URV>;
    friend class Hart<URV>;

    /// Define the privilege mode of this CSR.
    void definePrivilegeMode(PrivilegeMode mode)
    { privMode_ = mode; }

    /// Associate given location with the value of this CSR. The
    /// previous value of the CSR is lost. If given location is null
    /// then the default location defined in this object is restored.
    void tie(URV* location)
    { valuePtr_ = location ? location : &value_; }

    /// Reset to initial (power-on) value.
    void reset()
    {
      *valuePtr_ = initialValue_;
      for (auto func : postReset_)
        func(*this);
    }

    /// Change the privilege required to access the register. This is
    /// used to control access to the user-level performance counters.
    void setPrivilegeMode(PrivilegeMode mode)
    { privMode_ = mode; }

    /// Configure.
    void config(std::string name, CsrNumber num, bool mandatory,
		bool implemented, URV value, URV writeMask, URV pokeMask,
		bool isDebug)
    { name_ = std::move(name); number_ = unsigned(num); mandatory_ = mandatory;
      implemented_ = implemented; initialValue_ = value;
      writeMask_ = writeMask; pokeMask_ = pokeMask;
      debug_ = isDebug; *valuePtr_ = value; }

    /// Define the mask used by the poke method to write this
    /// register. The mask defined the register bits that are
    /// modifiable (even though such bits may not be writable using a
    /// CSR instruction). For example, the meip bit (of the mip CSR)
    /// is not writable using a CSR instruction but is modifiable.
    void setPokeMask(URV mask)
    { pokeMask_ = mask; }

    /// Mark register as a debug-mode register. Accessing a debug-mode
    /// register when the processor is not in debug mode will trigger an
    /// illegal instruction exception.
    void setIsDebug(bool flag)
    { debug_ = flag; }

    /// Mark register as shared among harts.
    void setIsShared(bool flag)
    { shared_ = flag; }

    /// Mark register as implemented.
    void setImplemented(bool flag)
    { implemented_ = flag; }

    /// Set initial value.
    void setInitialValue(URV v)
    { initialValue_ = v; }

    /// Mark register as defined.
    void setDefined(bool flag)
    { defined_ = flag; }

    /// True if this is a hypervisor register. Hypervisor registers
    /// are not available in VS/VU (virtual-supervisor) mode.
    void setHypervisor(bool flag)
    { hyper_ = flag; }

    void setMapsToVirtual(bool flag)
    { mapsToVirtual_ = flag; }

    bool isDefined() const
    { return defined_; }

    void pokeNoMask(URV v)
    { *valuePtr_ = v; }

    void setWriteMask(URV mask)
    { writeMask_ = mask; }

    void setReadMask(URV mask)
    { readMask_ = mask; }

    /// Set the value of this register to the given value x honoring
    /// the write mask (defined at construction): Set the ith bit of
    /// this register to the ith bit of the given value x if the ith
    /// bit of the write mask is 1; otherwise, leave the ith bit
    /// unmodified. This is the interface used by the CSR
    /// instructions.
    void write(URV x)
    {
      if (not hasPrev_)
	{
	  prev_ = *valuePtr_;
	  hasPrev_ = true;
	}
      for (auto func : preWrite_)
        func(*this, x);

      URV newVal = (x & writeMask_) | (*valuePtr_ & ~writeMask_);
      *valuePtr_ = newVal;

      for (auto func : postWrite_)
        func(*this, newVal);
    }

    /// Similar to the write method but using the poke mask instead of
    /// the write mask. This is the interface used by non-csr
    /// instructions to change modifiable (but not writable through
    /// CSR instructions) bits of this register.
    void poke(URV x)
    {
      for (auto func : prePoke_)
        func(*this, x);

      URV newVal = (x & pokeMask_) | (*valuePtr_ & ~pokeMask_);
      *valuePtr_ = newVal;

      for (auto func : postPoke_)
        func(*this, newVal);
    }

    /// Return the value of this register before last sequence of
    /// writes. Return current value if no writes since
    /// clearLastWritten.
    URV prevValue() const
    { return hasPrev_? prev_ : read(); }

    /// Clear previous value recorded by first write since
    /// clearLastWritten.
    void clearLastWritten()
    { hasPrev_ = false; }

    void setFields(const std::vector<Field>& fields)
    { fields_ = fields; }

    std::vector<Field> getFields() const
    { return fields_; }

    bool field(std::string_view field, URV& val) const
    {
      unsigned start = 0;
      for (auto& f : fields_)
        {
          if (f.field == field)
            {
              URV mask = ((1 << f.width) - 1) << start;
              val = (value_ & mask) >> start;
              return true;
            }
          start += f.width;
        }
      return false;
    }

  private:

    std::string name_;
    unsigned number_ = 0;
    bool mandatory_ = false;   // True if mandated by architecture.
    bool implemented_ = false; // True if register is implemented.
    bool hyper_ = false;       // True if hypervisor CSR.
    bool mapsToVirtual_ = false; // True if CSR maps to a virtual supervisor CSR.
    bool defined_ = false;
    bool debug_ = false;       // True if this is a debug-mode register.
    bool shared_ = false;      // True if this is shared among harts.
    URV initialValue_ = 0;
    PrivilegeMode privMode_ = PrivilegeMode::Machine;
    URV value_ = 0;
    URV prev_ = 0;
    bool hasPrev_ = false;
    bool high_ = false;

    // This will point to value_ except when shadowing the value of
    // some other register.
    URV* valuePtr_ = nullptr;

    URV writeMask_ = ~URV(0);
    URV pokeMask_ = ~URV(0);
    URV readMask_ = ~URV(0);  // Used for sstatus.

    std::vector<std::function<void(Csr<URV>&, URV)>> postPoke_;
    std::vector<std::function<void(Csr<URV>&, URV)>> postWrite_;

    std::vector<std::function<void(Csr<URV>&, URV&)>> prePoke_;
    std::vector<std::function<void(Csr<URV>&, URV&)>> preWrite_;

    std::vector<std::function<void(Csr<URV>&)>> postReset_;

    // Optionally define fields within a CSR with name and widths
    std::vector<Field> fields_;
  };


  /// Model the control and status register set.
  template <typename URV>
  class CsRegs
  {
  public:

    friend class Hart<uint32_t>;
    friend class Hart<uint64_t>;

    CsRegs();
    
    ~CsRegs();

    CsRegs(const CsRegs &) = delete;

    /// Return pointer to the control-and-status register
    /// corresponding to the given name or nullptr if no such
    /// register.
    Csr<URV>* findCsr(std::string_view name);

    /// Return pointer to the control-and-status register
    /// corresponding to the given number or nullptr if no such
    /// register.
    Csr<URV>* findCsr(CsrNumber number);

    const Csr<URV>* findCsr(CsrNumber number) const;

    /// Read given CSR on behalf of a CSR instruction (e.g. csrrw)
    /// into value returning true on success.  Return false leaving
    /// value unmodified if there is no CSR with the given number or
    /// if the CSR is not implemented or if it is not accessible by
    /// the given mode.
    bool read(CsrNumber number, PrivilegeMode mode, URV& value) const;

    /// Write given CSR on behalf of a CSR instruction (e.g. csrrw)
    /// returning true on success. Return false writing nothing if
    /// there is no CSR with the given number or if the CSR is not
    /// implemented or if it is not accessible by the given mode.
    bool write(CsrNumber number, PrivilegeMode mode, URV value);

    /// Return true if given register is writable by a CSR instruction
    /// in the given privilege and virtual mode.
    bool isWriteable(CsrNumber number, PrivilegeMode mode, bool virtMode) const;

    /// Return true if this is a high-half of a CSR (e.g. MSTATUSH is
    /// the high half of MSTATUS).
    bool isHighHalf(CsrNumber number) const
    {
      const Csr<URV>* csr = getImplementedCsr(number, virtMode_);
      return csr ? csr->isHighHalf() : false;
    }

    /// Return true if given register is readable by a CSR instruction
    /// in the given privlege and virtual modes.
    bool isReadable(CsrNumber number, PrivilegeMode pm, bool virtMode) const;

    /// Fill the nums vector with the numbers of the CSRs written by
    /// the last instruction.
    void getLastWrittenRegs(std::vector<CsrNumber>& csrNums) const
    {
      csrNums = lastWrittenRegs_;
    }

    /// Fill the nums vector with the numbers of the CSRs and triggers
    /// written by the last instruction.
    void getLastWrittenRegs(std::vector<CsrNumber>& csrNums,
			    std::vector<unsigned>& triggerNums) const
    {
      csrNums = lastWrittenRegs_;
      triggers_.getLastWrittenTriggers(triggerNums);
    }

    /// Associate an IMSIC with this register file.
    void attachImsic(std::shared_ptr<TT_IMSIC::Imsic> imsic)
    { imsic_ = imsic; }

  protected:

    /// Advance a csr number by the given amount (add amount to number).
    static CsrNumber advance(CsrNumber csrn, uint32_t amount)
    { return CsrNumber(uint32_t(csrn) + amount); }

    /// Advance a csr number by the given amount (add amount to number).
    static CsrNumber advance(CsrNumber csrn, int32_t amount)
    { return CsrNumber(uint32_t(csrn) + amount); }

    /// Similar to read but returned value is sign extended: sign bit is bit
    /// corresponding to most significant set bit in write mask of CSR.
    bool readSignExtend(CsrNumber number, PrivilegeMode mode, URV& value) const;

    /// Define csr with given name and number. Return pointer to csr
    /// on success or nullptr if given name is already in use or if the
    /// csr number is out of bounds or if it is associated with an
    /// already defined CSR.
    Csr<URV>* defineCsr(std::string name, CsrNumber number,
			bool mandatory, bool implemented, URV value,
			URV writeMask, URV pokeMask, bool isDebug = false,
			bool quiet = false);

    /// Return pointer to CSR with given number. Return nullptr if
    /// number is out of bounds or if corresponding CSR is not
    /// implemented.
    Csr<URV>* getImplementedCsr(CsrNumber num)
    {
      size_t ix = size_t(num);
      if (ix >= regs_.size()) return nullptr;
      Csr<URV>* csr = &regs_.at(ix);
      return csr->isImplemented() ? csr : nullptr;
    }

    /// Return pointer to CSR with given number. Return nullptr if
    /// number is out of bounds or if corresponding CSR is not
    /// implemented.
    const Csr<URV>* getImplementedCsr(CsrNumber num) const
    {
      size_t ix = size_t(num);
      if (ix >= regs_.size()) return nullptr;
      const Csr<URV>* csr = &regs_.at(ix);
      return csr->isImplemented() ? csr : nullptr;
    }

    /// Similar to getImplementedCsr except that when virtaulMode is true:
    /// Supervisor CSRs are remapped to the virtual supervisor counterparts.
    Csr<URV>* getImplementedCsr(CsrNumber num, bool virtualMode);

    /// Const version.
    const Csr<URV>* getImplementedCsr(CsrNumber num, bool virtualMode) const;

    /// Enable/disable load-data debug triggerring (disabled by default).
    void configLoadDataTrigger(bool flag)
    { triggers_.enableLoadData(flag); }

    /// Enable/disable exec-opcode triggering (disabled by default).
    void configExecOpcodeTrigger(bool flag)
    { triggers_.enableExecOpcode(flag); }

    /// Restrict chaining only to pairs of consecutive (even-numbered followed
    /// by odd) triggers.
    void configEvenOddTriggerChaining(bool flag)
    { triggers_.setEvenOddChaining(flag); }

    /// Return true if one more debug triggers are enabled.
    bool hasActiveTrigger() const
    { return hasActiveTrigger_; }

    /// Return true if one more instruction (execution) debug triggers
    /// are enabled.
    bool hasActiveInstTrigger() const
    { return hasActiveInstTrigger_; }

    /// Get the values of the three components of the given debug
    /// trigger. Return true on success and false if trigger is out of
    /// bounds.
    bool peekTrigger(unsigned trigger, uint64_t& data1, uint64_t& data2,
                     uint64_t& data3) const
    { return triggers_.peek(trigger, data1, data2, data3); }

    /// Get the values of the three components of the given debug
    /// trigger as well as the components write and poke masks. Return
    /// true on success and false if trigger is out of bounds.
    bool peekTrigger(unsigned trigger,
                     uint64_t& data1, uint64_t& data2, uint64_t& data3,
		     uint64_t& wm1, uint64_t& wm2, uint64_t& wm3,
		     uint64_t& pm1, uint64_t& pm2, uint64_t& pm3) const
    { return triggers_.peek(trigger, data1, data2, data3, wm1, wm2, wm3,
			    pm1, pm2, pm3); }

    /// Set the values of the three components of the given debug
    /// trigger. Return true on success and false if trigger is out of
    /// bounds.
    bool pokeTrigger(URV trigger, URV data1, URV data2, URV data3)
    { return triggers_.poke(trigger, data1, data2, data3); }

    /// Return true if any of the load (store if isLoad is false)
    /// triggers trips. A load/store trigger trips if it matches the
    /// given address and timing and if all the remaining triggers in
    /// its chain have tripped. Set the local-hit bit of any
    /// load/store trigger that matches. If a matching load/store
    /// trigger causes its chain to trip, then set the hit bit of all
    /// the triggers in that chain.
    bool ldStAddrTriggerHit(URV addr, TriggerTiming t, bool isLoad,
                            PrivilegeMode mode, bool ie)
    {
      bool chainHit = triggers_.ldStAddrTriggerHit(addr, t, isLoad, mode, ie);
      URV tselect = 0;
      peek(CsrNumber::TSELECT, tselect);
      if (triggers_.getLocalHit(tselect))
	recordWrite(CsrNumber::TDATA1);  // Hit bit in TDATA1 changed.
      return chainHit;
    }

    /// Similar to ldStAddrTriggerHit but for data match.
    bool ldStDataTriggerHit(URV data, TriggerTiming t, bool isLoad,
                            PrivilegeMode mode, bool ie)
    {
      bool chainHit = triggers_.ldStDataTriggerHit(data, t, isLoad, mode, ie);
      URV tselect = 0;
      peek(CsrNumber::TSELECT, tselect);
      if (triggers_.getLocalHit(tselect))
	recordWrite(CsrNumber::TDATA1);  // Hit bit in TDATA1 changed.
      return chainHit;
    }

    /// Similar to ldStAddrTriggerHit but for instruction address.
    bool instAddrTriggerHit(URV addr, TriggerTiming t, PrivilegeMode mode,
                            bool ie)
    {
      bool chainHit = triggers_.instAddrTriggerHit(addr, t, mode, ie);
      URV tselect = 0;
      peek(CsrNumber::TSELECT, tselect);
      if (triggers_.getLocalHit(tselect))
	recordWrite(CsrNumber::TDATA1);  // Hit bit in TDATA1 changed.
      return chainHit;
    }

    /// Similar to instAddrTriggerHit but for instruction opcode.
    bool instOpcodeTriggerHit(URV opcode, TriggerTiming t, PrivilegeMode mode,
                              bool ie)
    {
      bool chainHit = triggers_.instOpcodeTriggerHit(opcode, t, mode, ie);
      URV tselect = 0;
      peek(CsrNumber::TSELECT, tselect);
      if (triggers_.getLocalHit(tselect))
	recordWrite(CsrNumber::TDATA1);  // Hit bit in TDATA1 changed.
      return chainHit;
    }

    /// Make every active icount trigger count down unless it was
    /// written by the current instruction. Set the hit bit of a
    /// counted-down register if its value becomes zero. Return true
    /// if any counted-down register reaches zero; otherwise, return
    /// false.
    bool icountTriggerHit(PrivilegeMode mode, bool ie)
    {
      bool hit = triggers_.icountTriggerHit(mode, ie);
      URV tselect = 0;
      peek(CsrNumber::TSELECT, tselect);
      if (triggers_.getLocalHit(tselect))
	recordWrite(CsrNumber::TDATA1);  // Hit bit in TDATA1 changed.
      return hit;
    }

    /// Set pre and post to the count of "before"/"after" triggers
    /// that tripped by the last executed instruction.
    void countTrippedTriggers(unsigned& pre, unsigned& post) const
    { triggers_.countTrippedTriggers(pre, post); }

    /// Set t1, t2, and t3 to true if corresponding component (tdata1,
    /// tdata2, an tdata3) of given trigger was changed by the current
    /// instruction.
    void getTriggerChange(URV trigger, bool& t1, bool& t2, bool& t3) const
    { triggers_.getTriggerChange(trigger, t1, t2, t3); }

    /// Associate given event number with given counter.  Subsequent
    /// calls to Hart::updatePerofrmanceCounters will cause given
    /// counter to count up by 1 if enabled for the hart privilege
    /// mode.  Return true on success. Return false if counter number
    /// is out of bounds. The mask parameter is a bit-field
    /// corresponding to the privilege modes for which the event is
    /// enabled (see PerfRegs::PrivModeMask and privModeToMask).
    bool assignEventToCounter(URV event, unsigned counter, uint32_t mask)
    { return mPerfRegs_.assignEventToCounter(event, counter, mask); }

    bool applyPerfEventAssign()
    { return mPerfRegs_.applyPerfEventAssign(); }

    /// Return true if there is one or more tripped trigger action set
    /// to "enter debug mode".
    bool hasEnterDebugModeTripped() const
    { return triggers_.hasEnterDebugModeTripped(); }

    /// Set value to the value of the given register returning true on
    /// success and false if number is out of bound. Peeks register assuming
    /// virtMode.
    bool peek(CsrNumber number, URV& value, bool virtMode) const;

    /// Set value to the value of the given register returning true on
    /// success and false if number is out of bound.
    bool peek(CsrNumber number, URV& value) const
    { return peek(number, value, virtMode_); }

    /// Set register to the given value masked by the poke mask. A
    /// read-only register can be changed this way as long as its poke
    /// mask is non-zero. Return true on success and false if number is
    /// out of bounds.
    bool poke(CsrNumber number, URV value);

    /// Reset all CSRs to their initial (power-on) values.
    void reset();

    /// Configure CSR. Return true on success and false on failure.
    bool configCsr(std::string_view name, bool implemented, URV resetValue,
                   URV mask, URV pokeMask, bool debug, bool shared);

    /// Configure CSR. Return true on success and false on failure.
    bool configCsr(CsrNumber csr, bool implemented, URV resetValue,
                   URV mask, URV pokeMask, bool debug, bool shared);

    /// Configure machine mode performance counters returning true on
    /// success and false on failure. N consecutive counters starting
    /// at MHPMCOUNTER3/MHPMCOUNTER3H are made read/write. The
    /// remaining counters are made read only. For each counter that
    /// is made read-write the corresponding MHPMEVENT is made
    /// read-write. The cof flag indicates that counter overflow
    /// extension is enabled.
    bool configMachineModePerfCounters(unsigned numCounters, bool cof);

    /// Configure user mode performance counters returning true on
    /// success and false on failure. N cannot exceed the number of
    /// machine mode performance registers. First N performance
    /// counters are configured as readable, the remaining ones are
    /// made read-zero.
    bool configUserModePerfCounters(unsigned numCounters);

    /// Helper to write method. Update frm/fflags after fscr is written.
    /// Update fcsr after frm/fflags is written.
    void updateFcsrGroupForWrite(CsrNumber number, URV value);

    /// Helper to poke method. Update frm/fflags after fscr is poked.
    /// Update fcsr after frm/fflags is poked.
    void updateFcsrGroupForPoke(CsrNumber number, URV value);

    /// Helper to write method. Update vxrm/vxsat after vscr is written.
    /// Update vcsr after vxrm/vxsat is written.
    void updateVcsrGroupForWrite(CsrNumber number, URV value);

    /// Helper to poke method. Update vxrm/vxsat after vscr is poked.
    /// Update vcsr after vxrm/vxsat is poked.
    void updateVcsrGroupForPoke(CsrNumber number, URV value);

    /// Helper to construtor. Define machine-mode CSRs
    void defineMachineRegs();

    /// Helper to construtor. Define supervisor-mode CSRs
    void defineSupervisorRegs();

    /// Helper to construtor. Define user-mode CSRs
    void defineUserRegs();

    /// Helper to constructor. Define hypervisor CSRs.
    void defineHypervisorRegs();

    /// Helper to construtor. Define debug-mode CSRs
    void defineDebugRegs();

    /// Helper to construtor. Define vector CSRs
    void defineVectorRegs();

    /// Helper to construtor. Define floating point CSRs
    void defineFpRegs();

    /// Helper to construtor. Define advanced interrupt architecture CSRs
    void defineAiaRegs();

    /// Helper to construtor. Define Mstateen extension CSRs
    void defineStateEnableRegs();

    /// Helper to constructor. Define PMA CSRs.
    void definePmaRegs();

    /// Set the store error address capture register. Return true on
    /// success and false if register is not implemented.
    bool setStoreErrorAddrCapture(URV value);

    /// The the supervisor interruppt externa pin.
    void setSeiPin(bool flag)
    { seiPin_ = flag; }

    /// Return true if given number corresponds to an implemented CSR.
    bool isImplemented(CsrNumber num) const
    {
      size_t ix = size_t(num);
      return ix< regs_.size() and regs_.at(ix).isImplemented();
    }

    /// Update the user level counter privilege. This is called after
    /// a write/poke to MCOUNTEREN/SCOUNTEREN/HCOUNTEREN.
    void updateCounterPrivilege();

    /// Update the virtual interrupt register control. This is called
    /// after a write/poke to HVICTL.
    void updateVirtInterruptCtl();

    /// Enter/exit debug mode based given a flag value of true/false.
    void enterDebug(bool flag)
    { debugMode_ = flag; }

    /// Return true if hart (and this register file) are in debug mode.
    bool inDebugMode() const
    { return debugMode_; }

    /// Called after writing given csr to update aliased bits in other
    /// hypervisor related CSRs.
    void hyperWrite(Csr<URV>* csr);

    /// Called after poking given csr to update aliased bits in other
    /// hypervisor related CSRs.
    void hyperPoke(Csr<URV>* csr);

    bool readTdata(CsrNumber number, PrivilegeMode mode, URV& value) const;

    bool writeTdata(CsrNumber number, PrivilegeMode mode, URV value);

    bool pokeTdata(CsrNumber number, URV value);

    bool readTopi(CsrNumber number, URV& value) const;

    bool setCsrFields(CsrNumber number, const std::vector<typename Csr<URV>::Field>& fields)
    {
      auto csr = findCsr(number);
      if (not csr)
        return false;

      csr->setFields(fields);
      return true;
    }

    bool getCsrFields(CsrNumber number, std::vector<typename Csr<URV>::Field>& fields)
    {
      auto csr = findCsr(number);
      if (not csr)
        return false;

      fields = csr->getFields();
      return true;
    }

    /// Clear VSTART CSR. Record write if value changes.
    void clearVstart()
    {
      auto& csr = regs_.at(size_t(CsrNumber::VSTART));
      auto prev = csr.read();
      if (prev != 0)
	{
	  csr.write(0);
	  recordWrite(CsrNumber::VSTART);
	}
    }

    /// Fast peek method for MIP.
    URV peekMip() const
    {
      const auto& csr = regs_.at(size_t(CsrNumber::MIP));
      return csr.read();
    }

    /// Fast peek method for HGEIE.
    URV peekHgeie() const
    {
      const auto& csr = regs_.at(size_t(CsrNumber::HGEIE));
      return csr.read();
    }

    /// Fast peek method for MIE.
    URV peekMie() const
    {
      const auto& csr = regs_.at(size_t(CsrNumber::MIE));
      return csr.read();
    }

    /// Return the effective interrupt enable mask: This is MIE ored
    /// with the virtual interrupt enable implied by MVIEN.
    URV effectiveInterruptEnable() const
    {
      const auto& csr = regs_.at(size_t(CsrNumber::MIE));
      return csr.read() | shadowSie_;
    }

    /// Fast peek method for MSTATUS
    URV peekMstatus() const
    {
      const auto& csr = regs_.at(size_t(CsrNumber::MSTATUS));
      return csr.read();
    }

    /// Fast peek method for SSTATUS or VSSTATUS
    URV peekSstatus(bool virtMode) const
    {
      if (virtMode)
	{
	  const auto& csr = regs_.at(size_t(CsrNumber::VSSTATUS));
	  return csr.read();
	}
      const auto& csr = regs_.at(size_t(CsrNumber::SSTATUS));
      return csr.read();
    }

    /// Fast peek method for VSTART.
    URV peekVstart() const
    {
      const auto& csr = regs_.at(size_t(CsrNumber::VSTART));
      return csr.read();
    }

    /// Fast peek method for MNSTATUS.
    URV peekMnstatus() const
    {
      const auto& csr = regs_.at(size_t(CsrNumber::MNSTATUS));
      return csr.read();
    }

    /// Fast peek method for MIDELEG
    URV peekMideleg() const
    {
      const auto& csr = regs_.at(size_t(CsrNumber::MIDELEG));
      return csr.read();
    }

    /// Fast peek method for HIDELEG
    URV peekHideleg() const
    {
      const auto& csr = regs_.at(size_t(CsrNumber::HIDELEG));
      return csr.read();
    }

    /// Set the current integer-register/CSR width.
    void turnOn32BitMode(bool flag)
    {
      rv32_ = flag;
    }

    /// Return the byte of the PMPCFG register associated with the
    /// given PMPADDR register. Return 0 if given PMPADDR register is
    /// out of bounds or is not implemented or if corresponding PMPCFG
    /// is not implemented.
    unsigned getPmpConfigByteFromPmpAddr(CsrNumber csrn) const;

    /// Record given CSR number as a being written by the current
    /// instruction. Recorded numbers can be later retrieved by the
    /// getLastWrittenRegs method.
    void recordWrite(CsrNumber num);

    void enableRecordWrite(bool flag)
    { recordWrite_ = flag; }

    /// Clear the remembered indices of the CSR register(s) written by
    /// the last instruction.
    void clearLastWrittenRegs()
    {
      for (auto& csrNum : lastWrittenRegs_)
        regs_.at(size_t(csrNum)).clearLastWritten();
      lastWrittenRegs_.clear();
      triggers_.clearLastWrittenTriggers();
    }

    /// Configure given trigger with given reset values, write and
    /// poke masks. Return true on success and false on failure.
    bool configTrigger(unsigned trigger,
                       uint64_t rv1, uint64_t rv2, uint64_t rv3,
		       uint64_t wm1, uint64_t wm2, uint64_t wm3,
		       uint64_t pm1, uint64_t pm2, uint64_t pm3)
    {
      return triggers_.config(trigger, rv1, rv2, rv3,
			      wm1, wm2, wm3, pm1, pm2, pm3);
    }

    bool isInterruptEnabled() const
    { return interruptEnable_; }

    /// Tie the shared CSRs in this file to the corresponding CSRs in
    /// the target CSR file making them share the same location for
    /// their value.
    void tieSharedCsrsTo(CsRegs<URV>& target);

    /// Tie CSR values of machine mode performance counters to the
    /// elements of the given vector so that when a counter in the
    /// vector is changed the corresponding CSR value changes and
    /// vice-versa. This is done to avoid the overhead of CSR checking
    /// when incrementing performance counters.
    void tiePerfCounters(std::vector<uint64_t>& counters);

    /// Set the maximum performance counter event id. Ids larger than
    /// the max value are replaced by that max.
    void setMaxEventId(URV maxId)
    { maxEventId_ = maxId; }

    /// Configure valid event. If this is used then events outside the
    /// given vector are replaced by zero before being assigned to an
    /// MHPMEVENT register. Otherwise, events greater that
    /// max-event-id are clamped to max-event-id before being assigned
    /// to an MHPMEVENT register.
    void configPerfEvents(std::vector<unsigned>& eventVec)
    {
      hasPerfEventSet_ = true;
      perfEventSet_.insert(eventVec.begin(), eventVec.end());
    }

    /// Lock/unlock mdseac. This supports imprecise load/store exceptions.
    void lockMdseac(bool flag)
    { mdseacLocked_ = flag; }

    /// Return true if MDSEAC register is locked (it is unlocked on reset
    /// and after a write to MDEAU).
    bool mdseacLocked() const
    { return mdseacLocked_; }

    /// Adjust the value of the PMPADDR register according to the
    /// grain mask and the A field of the corresponding PMPCFG.
    /// Return adjusted value.
    URV adjustPmpValue(CsrNumber csrn, URV value) const;

    /// Read value of SIP (MIP masked with MIDELEG). This never
    /// maps to VSIP.
    bool readSip(URV& value) const;

    /// Read value of SIE (MIE masked with MIDELEG). This never
    /// maps to VSIP.
    bool readSie(URV& value) const;

    /// Helper to read method.
    bool readMvip(URV& value) const;

    /// Helper to write method.
    bool writeMvip(URV value);

    /// Adjust the value of TIME/TIMEH by adding the time delta in
    /// virtual mode.
    URV adjustTimeValue(CsrNumber csrn, URV value) const;

    /// Adjust the value of SSTATEEN by masking with MSTATEEN and HSTATEEN.
    URV adjustSstateenValue(CsrNumber csrn, URV value) const;

    /// Adjust the value of HSTATEEN by masking with MSTATEEN.
    URV adjustHstateenValue(CsrNumber csrn, URV value) const;

    // Adjust the value of SCOUNTOVF by masking with MCOUNTEREN/HCOUNTEREN
    URV adjustScountovfValue(URV value) const;

    /// Heler to read method.
    bool readMireg(CsrNumber num, URV& value) const;

    /// Heler to read method.
    bool readSireg(CsrNumber num, URV& value) const;

    /// Helper to read method.
    bool readVsireg(CsrNumber num, URV& value) const;

    /// Helper to write method: Mask with MIP/MIDELEG.
    bool writeSip(URV value);

    /// Helper to write method: Mask with SIP/MIDELEG.
    bool writeSie(URV value);

    /// Helper to write method: Mask with MSTATEEN/HSTATEEN.
    bool writeSstateen(CsrNumber num, URV value);

    /// Helper to write method: Mask with MSTATEEN.
    bool writeHstateen(CsrNumber num, URV value);

    /// Helper to write method.
    bool writeMireg(CsrNumber num, URV value);

    /// Helper to write method.
    bool writeSireg(CsrNumber num, URV value);

    /// Helper to write method.
    bool writeVsireg(CsrNumber num, URV value);

    /// Helper to write method.
    bool writeMtopei();

    /// Helper to write method.
    bool writeStopei();

    /// Helper to write method.
    bool writeVstopei();

    /// Helper to read method.
    bool readMtopei();

    /// Legalize the PMPCFG value before updating such a register: If
    /// the grain factor G is greater than or equal to 1, then the NA4
    /// mode is not selectable in the A field. If a field is locked it
    /// is replaced by the current value. Return the legalized value.
    URV legalizePmpcfgValue(URV current, URV value) const;

    /// Legalize a PMACFG value.
    URV legalizePmacfgValue(URV current, URV value) const;

    /// Legalize scountovf, matching OF bit of given mhpmevent.
    void updateScountovfValue(CsrNumber mhpm, uint64_t value);

    /// Return true if given CSR number is a PMPADDR register and if
    /// that register is locked.  Return false otherwise.
    bool isPmpaddrLocked(CsrNumber csrn) const;

    /// Set the physical memory protection G parameter. The grain size
    /// is 2 to the power G+2.  The values returned by a read operation
    /// of the PMPADDR registers are adjusted according to G.
    void setPmpG(unsigned value)
    { pmpG_ = value; }

    /// Return the physical memory protection G parameter. See setPmpG.
    unsigned getPmpG() const
    { return pmpG_; }

    /// Set the max number of guest interrupt count. This should be
    /// done before hypervisor mode is enable.
    void setGuestInterruptCount(unsigned value)
    { geilen_ = value; }

    /// Enable/disable user mode.
    void enableUserMode(bool flag)
    { userEnabled_ = flag; }

    /// Enable/disable supervisor time compare.
    void enableSstc(bool flag)
    { sstcEnabled_ = flag; enableMenvcfgStce(flag); updateSstc(); }

    /// Enable/disable top-of-range mode in pmp configurations.
    void enablePmpTor(bool flag)
    { pmpTor_ = flag; }

    /// Enable/disable NA4 mode in pmp configurations.
    void enablePmpNa4(bool flag)
    { pmpNa4_ = flag; }

    /// Update implementation status of Sstc (supervisor timer)
    /// related CSRs.  This is called when Sstc related configuration
    /// changes.
    void updateSstc();

    /// Enable/disable F extension.
    void enableRvf(bool flag);

    /// Enable/disable C extension.
    void enableRvc(bool flag)
    {
      // Least sig bit reads zero if C. Least sig 2 bits read zero if not C.
      URV mask = ~URV(0) << (flag ? 1 : 2);  // 
      regs_.at(size_t(CsrNumber::MEPC)).setReadMask(mask);
      regs_.at(size_t(CsrNumber::SEPC)).setReadMask(mask);
      regs_.at(size_t(CsrNumber::MNEPC)).setReadMask(mask);
      regs_.at(size_t(CsrNumber::VSEPC)).setReadMask(mask);
    }

    /// Enable/disable counter-overflow extension (sscofpmf)
    void enableSscofpmf(bool flag);

    /// Enable/disable access to certain CSRs from non-machine mode.
    void enableStateen(bool flag);

    /// Enable/disable resubale non maskable interrupt extension.
    void enableSmrnmi(bool flag);

    /// Enable/disable supervisor mode.
    void enableSupervisorMode(bool flag);

    /// Enable/disable hypervisor mode.
    void enableHypervisorMode(bool flag);

    /// Enable/disable vector extension.
    void enableVector(bool flag);

    /// Enable/disable advanced interrupt artchitecture extension.
    void enableAia(bool flag);

    /// Enable/disable virtual supervisor. When enabled, the trap-related
    /// CSRs point to their virtual counterpars (e.g. reading writing sstatus will
    /// actually read/write vsstatus).
    void enableVirtualSupervisor(bool flag);

    /// Return a legal mstatus value (chanign mpp if necessary).
    URV legalizeMstatusValue(URV value) const;

    /// Called after an MHPMEVENT CSR is written/poked to update the
    /// contorl of the underlying counter.
    void updateCounterControl(CsrNumber number);

    /// Update bits of SIE that are distinct for MIE (happens where MVIEN bits are set and
    /// corresponding bits in MIDELEG are clear).
    void updateShadowSie();

    /// Turn on/off virtual mode. When virtual mode is on read/write to
    /// supervisor CSRs get redirected to virtual supervisor CSRs and read/write
    /// of virtual supervisor CSRs become illegal.
    void setVirtualMode(bool flag)
    { virtMode_ = flag; }

    /// helper to add fields of machine CSRs
    void addMachineFields();

    /// helper to add fields of supervisor CSRs
    void addSupervisorFields();

    /// helper to add fields of user CSRs
    void addUserFields();

    /// helper to add fields of vector CSRs
    void addVectorFields();

    /// helper to add fields of fp CSRs
    void addFpFields();

    /// helper to add fields of hypervisor CSRs
    void addHypervisorFields();

    /// Return true if given CSR is a hypervisor CSR.
    bool isHypervisor(CsrNumber csrn) const
    { auto csr = getImplementedCsr(csrn); return csr and csr->isHypervisor(); }

    /// If flag is false, bit HENVCFG.STCE becomes read-only-zero;
    /// otherwise, bit is readable.
    void enableHenvcfgStce(bool flag);

    /// If flag is false, bit MENVCFG.STCE becomes read-only-zero;
    /// otherwise, bit is readable.
    void enableMenvcfgStce(bool flag);

    /// Return the value of the STCE bit of the MENVCFG CSR. Return
    /// false if CSR is not implemented or if SSTC extension is off.
    bool menvcfgStce()
    {
      auto csr = getImplementedCsr(rv32_? CsrNumber::MENVCFGH : CsrNumber::MENVCFG);
      if (not csr)
	return false;
      if (not sstcEnabled_)
	return false;
      URV value = csr->read();
      if (rv32_)
	{
	  MenvcfghFields<uint32_t> fields(value);
	  return fields.bits_.STCE;
	}
      MenvcfgFields<uint64_t> fields(value);
      return fields.bits_.STCE;
    }

    /// Return the value of the PBMTE bit of the MENVCFG CSR. Return
    /// false if CSR is not implemented.
    bool menvcfgPbmte()
    {
      auto csr = getImplementedCsr(rv32_? CsrNumber::MENVCFGH : CsrNumber::MENVCFG);
      if (not csr)
	return false;
      URV value = csr->read();
      if (rv32_)
	{
	  MenvcfghFields<uint32_t> fields(value);
	  return fields.bits_.PBMTE;
	}
      MenvcfgFields<uint64_t> fields(value);
      return fields.bits_.PBMTE;
    }

    /// Return the value of the STCE bit of the HENVCFG CSR. Return
    /// false if CSR is not implemented
    bool henvcfgStce()
    {
      auto csr = getImplementedCsr(rv32_? CsrNumber::HENVCFGH : CsrNumber::HENVCFG);
      if (not csr)
	return false;
      URV value = csr->read();
      if (rv32_)
	{
	  HenvcfghFields<uint32_t> fields(value);
	  return fields.bits_.STCE;
	}
      HenvcfgFields<uint64_t> fields(value);
      return fields.bits_.STCE;
    }

    /// Return the ADUE bit of MENVCFG CSR.
    bool menvcfgAdue()
    {
      auto csr = getImplementedCsr(CsrNumber::MENVCFG);
      if (not csr)
	return false;
      URV value = csr->read();
      MenvcfgFields<uint64_t> fields(value);
      return fields.bits_.ADUE;
    }

    /// Return the ADUE bit of MENVCFG CSR.
    bool henvcfgAdue()
    {
      auto csr = getImplementedCsr(CsrNumber::HENVCFG);
      if (not csr)
	return false;
      URV value = csr->read();
      MenvcfgFields<uint64_t> fields(value);
      return fields.bits_.ADUE;
    }

    /// Set ix to the counter index corresponding to the given
    /// MHPMEVENT CSR number (0 for MHPMEVENT3, 1 for MHPMEVENT4, ...)
    /// Return true on success and false if number does not correspond
    /// to an MHPMEVENT CSR.
    bool getIndexOfMhpmevent(CsrNumber csrn, unsigned& ix) const
    {
      if (csrn >= CsrNumber::MHPMEVENT3 and csrn <= CsrNumber::MHPMEVENT31)
	{
	  ix = unsigned(csrn) - unsigned(CsrNumber::MHPMEVENT3);
	  return true;
	}

      if (rv32_)
	{
	  if (csrn >= CsrNumber::MHPMEVENTH3 and csrn <= CsrNumber::MHPMEVENTH31)
	    {
	      ix = unsigned(csrn) - unsigned(CsrNumber::MHPMEVENTH3);
	      return true;
	    }
	}

      return false;
    }

    /// Set value to the value of the MHPMEVENT register indexed by
    /// the given index (0 corresponds to MHPMEVENT3).  For RV32 and
    /// if counter-overflow is enabled, then use the MHPMEVENTH
    /// registers to get the upper 64-bits of the returned value.
    /// Return true on succes.  Return false leaving value unmodified
    /// if index is out of bounds.
    bool getMhpmeventValue(unsigned ix, uint64_t& value) const
    {
      if (ix >= 29)
	return false;
      
      using CN = CsrNumber;
      CN csrn = CN(unsigned(CN::MHPMEVENT3) + ix);
      auto csr = findCsr(csrn);
      if (not csr)
	return false;

      value = csr->read();

      if (rv32_ and cofEnabled_ and superEnabled_)
	  {
	    CN hcsrn = CN(unsigned(CN::MHPMEVENTH3) + ix);
	    auto hcsr = this->findCsr(hcsrn);
	    if (not hcsr)
	      return false;
	    value |= uint64_t(hcsr->read()) << 32;
	  }

      return true;
    }

    /// Returns true if CSR is defined as part of a STATEEN and enabled, or
    /// not part of STATEEN. Returns false otherwise.
    bool isStateEnabled(CsrNumber num, PrivilegeMode mode, bool virtMode) const;

  private:

    bool rv32_ = sizeof(URV) == 4;
    std::vector< Csr<URV> > regs_;
    std::unordered_map<std::string, CsrNumber, util::string_hash, std::equal_to<>> nameToNumber_;

    Triggers<URV> triggers_;
    std::shared_ptr<TT_IMSIC::Imsic> imsic_;

    // Register written since most recent clearLastWrittenRegs
    std::vector<CsrNumber> lastWrittenRegs_;

    // Counters implementing machine performance counters.
    PerfRegs mPerfRegs_;

    bool interruptEnable_ = false;  // Cached MSTATUS MIE bit.

    // These can be obtained from Triggers. Speed up access by caching
    // them in here.
    bool hasActiveTrigger_ = false;
    bool hasActiveInstTrigger_ = false;

    bool mdseacLocked_ = false; // Once written, MDSEAC persists until
                                // MDEAU is written.
    URV maxEventId_ = ~URV(0);  // Default unlimited.
    bool hasPerfEventSet_ = false;
    std::unordered_set<unsigned> perfEventSet_;

    URV shadowSie_ = 0;     // Used where mideleg is 0 and mvien is 1.

    unsigned pmpG_ = 0;     // PMP G value: ln2(pmpGrain) - 2
    unsigned geilen_ = 0;   // Guest interrupt count.

    bool userEnabled_ = false;    // User mode enabled
    bool superEnabled_ = false;   // Supervisor
    bool hyperEnabled_ = false;   // Hypervisor
    bool sstcEnabled_ = false;    // Supervisor time compare
    bool cofEnabled_ = false;     // Counter overflow
    bool stateenOn_ = false;      // Mstateen extension.
    bool pmpTor_ = true;          // Top-of-range PMP mode enabled
    bool pmpNa4_ = true;          // Na4 PMP mode enabled
    bool aiaEnabled_ = false;     // Aia extension.

    bool recordWrite_ = true;
    bool debugMode_ = false;
    bool virtMode_ = false;       // True if hart virtual (V) mode is on.

    bool seiPin_ = false;
  };
}
