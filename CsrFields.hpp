#pragma once

#include <cstdint>

namespace WdRiscv
{

  /// Struct used to pack/unpack MSTATUSH in RV32.
  struct Mstatush
  {
    unsigned res0     : 4;
    unsigned SBE      : 1;
    unsigned MBE      : 1;
    unsigned GVA      : 1;
    unsigned MPV      : 1;
    unsigned res1     : 24;
  };


  /// Struct used to pack/unpack MSTATUS in RV32.
  struct Mstatus32
    {
      unsigned UIE      : 1;
      unsigned SIE      : 1;
      unsigned res0     : 1;
      unsigned MIE      : 1;
      unsigned UPIE     : 1;
      unsigned SPIE     : 1;
      unsigned UBE      : 1;
      unsigned MPIE     : 1;
      unsigned SPP      : 1;
      unsigned VS       : 2;
      unsigned MPP      : 2;
      unsigned FS       : 2;
      unsigned XS       : 2;
      unsigned MPRV     : 1;
      unsigned SUM      : 1;
      unsigned MXR      : 1;
      unsigned TVM      : 1;
      unsigned TW       : 1;
      unsigned TSR      : 1;
      unsigned res1     : 8;  // Reserved
      unsigned SD       : 1;
    };


  /// Struct used to pack/unpack MSTATUS in RV64.
  struct Mstatus64
    {
      unsigned UIE      : 1;
      unsigned SIE      : 1;
      unsigned res0     : 1;
      unsigned MIE      : 1;
      unsigned UPIE     : 1;
      unsigned SPIE     : 1;
      unsigned UBE      : 1;
      unsigned MPIE     : 1;
      unsigned SPP      : 1;
      unsigned VS       : 2;
      unsigned MPP      : 2;
      unsigned FS       : 2;
      unsigned XS       : 2;
      unsigned MPRV     : 1;
      unsigned SUM      : 1;
      unsigned MXR      : 1;
      unsigned TVM      : 1;
      unsigned TW       : 1;
      unsigned TSR      : 1;
      unsigned res1     : 9;
      unsigned UXL      : 2;
      unsigned SXL      : 2;
      unsigned SBE      : 1;
      unsigned MBE      : 1;
      unsigned GVA      : 1;
      unsigned MPV      : 1;
      unsigned res2     : 23;  // Reserved
      unsigned SD       : 1;
    };


  /// Structure used to unpack/pack the fields of the machine status
  /// register.
  template <typename URV>
  union MstatusFields;

  /// 32-bit version.
  template <>
  union MstatusFields<uint32_t>
  {
    MstatusFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_;   // Machine status register value.
    Mstatus32 bits_;   // Bit fields.
  };

  /// 64-bit version.
  template <>
  union MstatusFields<uint64_t>
  {
    MstatusFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;   // Machine status register value.
    Mstatus64 bits_;   // Bit fields.
  };


  /// Effective mstatus: Cached value of mstatus for RV64 and
  /// mstatush/mstatus for RV32.
  template <typename URV>
  union Emstatus;

  /// RV32 version.
  template <>
  union Emstatus<uint32_t>
  {
    Emstatus(uint32_t low = 0, uint32_t high = 0)
      : value_{low, high}
    { }

    struct
    {
      uint32_t low_;
      uint32_t high_;
    } value_;

    struct
    {
      unsigned UIE      : 1;
      unsigned SIE      : 1;
      unsigned res0     : 1;
      unsigned MIE      : 1;
      unsigned UPIE     : 1;
      unsigned SPIE     : 1;
      unsigned UBE      : 1;
      unsigned MPIE     : 1;
      unsigned SPP      : 1;
      unsigned VS       : 2;
      unsigned MPP      : 2;
      unsigned FS       : 2;
      unsigned XS       : 2;
      unsigned MPRV     : 1;
      unsigned SUM      : 1;
      unsigned MXR      : 1;
      unsigned TVM      : 1;
      unsigned TW       : 1;
      unsigned TSR      : 1;
      unsigned res1     : 8;  // Reserved
      unsigned SD       : 1;

      // mstatush
      unsigned res2     : 4;
      unsigned SBE      : 1;
      unsigned MBE      : 1;
      unsigned GVA      : 1;
      unsigned MPV      : 1;
      unsigned res3     : 24;
    } bits_;
  };

  /// RV64 version.
  template <>
  union Emstatus<uint64_t>
  {
    Emstatus(uint32_t value = 0)
      : value_(value)
    { }

    uint64_t value_;
    Mstatus64 bits_;
  };


  /// Structure used to unpack/pack the fields of the hypervisor
  /// status register.
  template <typename URV>
  union HstatusFields;

  /// 32-bit version.
  template <>
  union HstatusFields<uint32_t>
  {
    HstatusFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_;   // Hypervisor status register value.
    struct
    {
      unsigned res0     : 5;
      unsigned VSBE     : 1;   // Virt supervisor big endian
      unsigned GVA      : 1;   // Guest virtual address
      unsigned SPV      : 1;   // Supervisor previous virtual mode
      unsigned SPVP     : 1;   // Supervisor previous virtual privilege (nominal priv)
      unsigned HU       : 1;   // Hypervisor instructions available in user mode
      unsigned res1     : 2;
      unsigned VGEIN    : 6;   // Virtual guest external interrupt number
      unsigned res2     : 2;
      unsigned VTVM     : 1;   // Trap on access to vsatp or SFENCE.VMA or SINVAL.VMA
      unsigned VTW      : 1;
      unsigned VTSR     : 1;   // Trap on sret
      unsigned res3     : 9;
    } bits_;
  };

  /// 64-bit version.
  template <>
  union HstatusFields<uint64_t>
  {
    HstatusFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;   // Machine status register value.
    struct
    {
      unsigned res0     : 5;
      unsigned VSBE     : 1;   // Virt supervisor big endian
      unsigned GVA      : 1;   // Guest virtual address
      unsigned SPV      : 1;   // Supervisor previous virtual mode
      unsigned SPVP     : 1;   // Supervisor previous virtual privilege (nominal priv)
      unsigned HU       : 1;   // Hypervisor instructions available in user mode
      unsigned res1     : 2;
      unsigned VGEIN    : 6;   // Virtual guest external interrupt number
      unsigned res2     : 2;
      unsigned VTVM     : 1;   // Trap on access to vsatp or SFENCE.VMA or SINVAL.VMA
      unsigned VTW      : 1;
      unsigned VTSR     : 1;   // Trap on sret
      unsigned res3     : 9;
      unsigned VSXL     : 2;
      unsigned res4     : 29;
    } bits_;
  };


  /// Structure used to unpack/pack the fields of the SATP register
  template <typename URV>
  union SatpFields;

  template <>
  union SatpFields<uint32_t>
  {
    SatpFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_;  // SATP register value
    struct
    {
      unsigned PPN : 22;
      unsigned ASID : 9;
      unsigned MODE : 1;
    } bits_;
  };

  template <>
  union SatpFields<uint64_t>
  {
    SatpFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_; // SATP register value
    struct
    {
      uint64_t PPN : 44;
      unsigned ASID : 16;
      unsigned MODE : 4;
    } bits_;
  };

  /// Structure used to unpack/pack the fields of the MENVCFG register
  template <typename URV>
  union MenvcfgFields;

  template <>
  union MenvcfgFields<uint32_t>
  {
    MenvcfgFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_; // MENVCFG register value
    struct
    {
      unsigned FIOM : 1;
      unsigned reserved0: 3;
      unsigned CBIE : 2;
      unsigned CBCFE : 1;
      unsigned CBZE : 1;
      unsigned reserved1 : 24;
    } bits_;
  };

  template <>
  union MenvcfgFields<uint64_t>
  {
    MenvcfgFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_; // MENVCFG register value
    struct
    {
      unsigned FIOM : 1;
      unsigned reserved0: 3;
      unsigned CBIE : 2;
      unsigned CBCFE : 1;
      unsigned CBZE : 1;
      uint64_t reserved1 : 54;
      unsigned PBMTE : 1;
      unsigned STCE : 1;
    } bits_;
  };

  /// Structure used to unpack/pack the fields of the MENVCFGH register (rv32 only)
  template <typename URV>
  union MenvcfghFields;

  template <>
  union MenvcfghFields<uint32_t>
  {
    MenvcfghFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_; // MENVCFGH register value
    struct
    {
      unsigned reserved0 : 30;
      unsigned PBMTE : 1;
      unsigned STCE : 1;
    } bits_;
  };

  /// Structure used to unpack/pack the fields of the SENVCFG register
  template <typename URV>
  union SenvcfgFields;

  template <>
  union SenvcfgFields<uint32_t>
  {
    SenvcfgFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_; // SENVCFG register value
    struct
    {
      unsigned FIOM : 1;
      unsigned reserved0 : 3;
      unsigned CBIE : 2;
      unsigned CBCFE : 1;
      unsigned CBZE : 1;
      unsigned reserved1 : 24;
    } bits_;
  };

  template <>
  union SenvcfgFields<uint64_t>
  {
    SenvcfgFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_; // SENVCFG register value
    struct
    {
      unsigned FIOM : 1;
      unsigned reserved0 : 3;
      unsigned CBIE : 2;
      unsigned CBCFE : 1;
      unsigned CBZE : 1;
      uint64_t reserved1 : 56;
    } bits_;
  };

  /// Structure used to unpack/pack the fields of the HENVCFG register
  template <typename URV>
  union HenvcfgFields;

  template <>
  union HenvcfgFields<uint32_t>
  {
    HenvcfgFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_; // HENVCFG register value
    struct
    {
      unsigned FIOM : 1;
      unsigned reserved0: 3;
      unsigned CBIE : 2;
      unsigned CBCFE : 1;
      unsigned CBZE : 1;
      unsigned reserved1 : 24;
    } bits_;
  };

  template <>
  union HenvcfgFields<uint64_t>
  {
    HenvcfgFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_; // HENVCFG register value
    struct
    {
      unsigned FIOM : 1;
      unsigned reserved0: 3;
      unsigned CBIE : 2;
      unsigned CBCFE : 1;
      unsigned CBZE : 1;
      uint64_t reserved1 : 54;
      unsigned PBMTE : 1;
      unsigned STCE : 1;
    } bits_;
  };

  /// Structure used to unpack/pack the fields of the HENVCFGH register (rv32 only)
  template <typename URV>
  union HenvcfghFields;

  template <>
  union HenvcfghFields<uint32_t>
  {
    HenvcfghFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_; // HENVCFGH register value
    struct
    {
      unsigned reserved0 : 30;
      unsigned PBMTE : 1;
      unsigned STCE : 1;
    } bits_;
  };


  /// Structure used to unpack/pack the fields of the DCSR register
  template <typename URV>
  union DcsrFields;

  template <>
  union DcsrFields<uint32_t>
  {
    DcsrFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_; // DCSR register value
    struct
    {
      unsigned PRV : 2;
      unsigned STEP : 1;
      unsigned NMIP : 1;
      unsigned MPRVEN : 1;
      unsigned V : 1;
      unsigned CAUSE : 3;
      unsigned STOPTIME : 1;
      unsigned STOPCOUNT : 1;
      unsigned STEPIE : 1;
      unsigned EBREAKU : 1;
      unsigned EBREAKS : 1;
      unsigned reserved1 : 1;
      unsigned EBREAKM : 1;
      unsigned EBREAKVU : 1;
      unsigned EBREAKVS : 1;
      unsigned reserved2 : 10;
      unsigned DEBUGVER : 4;
    } bits_;
  };

  template <>
  union DcsrFields<uint64_t>
  {
    DcsrFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_; // DCSR register value
    struct
    {
      unsigned PRV : 2;
      unsigned STEP : 1;
      unsigned NMIP : 1;
      unsigned MPRVEN : 1;
      unsigned V : 1;
      unsigned CAUSE : 3;
      unsigned STOPTIME : 1;
      unsigned STOPCOUNT : 1;
      unsigned STEPIE : 1;
      unsigned EBREAKU : 1;
      unsigned EBREAKS : 1;
      unsigned reserved1 : 1;
      unsigned EBREAKM : 1;
      unsigned EBREAKVU : 1;
      unsigned EBREAKVS : 1;
      unsigned reserved2 : 10;
      unsigned DEBUGVER : 4;
    } bits_;
  };

  /// Structure used to unpack/pack the fields of the VTYPE register
  template<typename URV>
  union VtypeFields;

  template <>
  union VtypeFields<uint32_t>
  {
    VtypeFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_; // VTYPE register value
    struct
    {
      unsigned LMUL : 3;
      unsigned SEW : 3;
      unsigned VTA : 1;
      unsigned VMA : 1;
      unsigned reserved0 : 23;
      unsigned VILL : 1;
    } bits_;
  };

  template <>
  union VtypeFields<uint64_t>
  {
    VtypeFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_; // VTYPE register value
    struct
    {
      unsigned LMUL : 3;
      unsigned SEW : 3;
      unsigned VTA : 1;
      unsigned VMA : 1;
      uint64_t reserved0 : 55;
      unsigned VILL : 1;
    } bits_;
  };


  /// Structure used to unpack/pack the fields of the HGATP register
  template <typename URV>
  union HgatpFields;

  template <>
  union HgatpFields<uint32_t>
  {
    HgatpFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_;  // HGATP register value
    struct
    {
      unsigned PPN  : 22;
      unsigned VMID : 7;
      unsigned res  : 2;
      unsigned MODE : 1;
    } bits_;
  };

  template <>
  union HgatpFields<uint64_t>
  {
    HgatpFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_; // HGATP register value
    struct
    {
      uint64_t PPN  : 44;
      unsigned VMID : 14;
      unsigned res  : 2;
      unsigned MODE : 4;
    } bits_;
  };


  /// Structure used to unpack/pack the fields of the mhpmevent register
  union MhpmeventFields
  {
    MhpmeventFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;  // register value
    struct
    {
      uint64_t EVENT : 56;
      unsigned res   : 2;
      unsigned VUINH : 1;
      unsigned VSINH : 1;
      unsigned UINH  : 1;
      unsigned SINH  : 1;  // Supervisor inhibit
      unsigned MINH  : 1;  // Machine inhibit
      unsigned OF    : 1;
    } bits_;
  };

}
