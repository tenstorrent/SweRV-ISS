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
    unsigned MPELP    : 1;
    unsigned MDT      : 1;
    unsigned res1     : 22;
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
      unsigned SPELP    : 1;
      unsigned SDT      : 1;
      unsigned res1     : 6;  // Reserved
      unsigned SD       : 1;
    };


  /// Struct used to pack/unpack MSTATUS in RV64.
  struct Mstatus64
    {
      unsigned UIE      : 1;  // bit 0
      unsigned SIE      : 1;  // bit 1
      unsigned res0     : 1;  // bit 2
      unsigned MIE      : 1;  // bit 3
      unsigned UPIE     : 1;  // bit 4
      unsigned SPIE     : 1;  // bit 5
      unsigned UBE      : 1;  // bit 6
      unsigned MPIE     : 1;  // bit 7
      unsigned SPP      : 1;  // bit 8
      unsigned VS       : 2;  // bit 9  10
      unsigned MPP      : 2;  // bit 11 12
      unsigned FS       : 2;  // bit 13 14
      unsigned XS       : 2;  // bit 15 16
      unsigned MPRV     : 1;  // bit 17
      unsigned SUM      : 1;  // bit 18
      unsigned MXR      : 1;  // bit 19
      unsigned TVM      : 1;  // bit 20
      unsigned TW       : 1;  // bit 21
      unsigned TSR      : 1;  // bit 22
      unsigned SPELP    : 1;  // bit 23
      unsigned SDT      : 1;  // bit 24
      unsigned res1     : 7;  // bit 25 to 31
      unsigned UXL      : 2;  // bit 32 33
      unsigned SXL      : 2;  // bit 34 35
      unsigned SBE      : 1;  // bit 36
      unsigned MBE      : 1;  // bit 37
      unsigned GVA      : 1;  // bit 38
      unsigned MPV      : 1;  // bit 39
      unsigned res2     : 1;  // bit 40
      unsigned MPELP    : 1;  // bit 41
      unsigned MDT      : 1;  // bit 42
      unsigned res3     : 20; // bit 43 to 62
      unsigned SD       : 1;  // bit 63
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

    uint64_t value()
    { return value64_; }

    uint64_t value64_;

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

    uint64_t value()
    { return value_; }

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

    uint64_t value()
    { return value_; }

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

    uint64_t value()
    { return value_; }

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
      unsigned res4     : 14;
      unsigned HUPMM    : 2;   // Pointer mask mode in VU-mode
      unsigned res5     : 14;
    } bits_;
  };


  /// Union to pack/unpack the fields of the MNSTATUS register
  union MnstatusFields
  {
    MnstatusFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;

    struct Mnstatus
    {
      unsigned res0   : 3;  // Bit 0 to 2
      unsigned NMIE   : 1;  // Bit 3
      unsigned res1   : 3;  // Bit 4 to 6
      unsigned MNPV   : 1;  // Bit 7
      unsigned res2   : 1;  // Bit 8
      unsigned MNPELP : 1;  // Bit 9
      unsigned res3   : 1;  // Bit 19
      unsigned MNPP   : 2;  // Bit 11 to 12
      unsigned res4   : 19; // Bit 13 to 31
    } bits_;
  };


  /// Union to pack/unpack the fields of the FCSR register
  union FcsrFields
  {
    FcsrFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;
    struct
    {
      unsigned FFLAGS : 5;
      unsigned FRM    : 3;
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
      unsigned FIOM      : 1;   // Bit  0
      unsigned reserved0 : 1;   // Bit  1
      unsigned LPE       : 1;   // Bit  2
      unsigned SSE       : 1;   // Bit  3
      unsigned CBIE      : 2;   // Bits 5:4
      unsigned CBCFE     : 1;   // Bit  6
      unsigned CBZE      : 1;   // Bit  7
      unsigned reserved2 : 24;  // Bits 31:8
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
      unsigned FIOM      : 1;   // Bit  0
      unsigned reserved0 : 1;   // Bit  1
      unsigned LPE       : 1;   // Bit  2
      unsigned SSE       : 1;   // Bit  3
      unsigned CBIE      : 2;   // Bits 5:4
      unsigned CBCFE     : 1;   // Bit  6
      unsigned CBZE      : 1;   // Bit  7
      uint64_t reserved1 : 24;  // Bits 31:8
      unsigned PMM       : 2;   // Bits 33:32
      unsigned reserved2 : 21;  // Bits 54:34
      unsigned SRMCFG    : 1;   // Bit  55
      uint64_t reserved3 : 5;   // Bits 60:56
      unsigned ADUE      : 1;   // Bit  61
      unsigned PBMTE     : 1;   // Bit  62
      unsigned STCE      : 1;   // Bit  63
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
      unsigned PMM       : 2;   // Bits 1:0
      unsigned reserved2 : 21;  // Bits 22:2
      unsigned SRMCFG    : 1;   // Bit  23
      uint64_t reserved3 : 5;   // Bits 28:24
      unsigned ADUE      : 1;   // Bit  29
      unsigned PBMTE     : 1;   // Bit  30
      unsigned STCE      : 1;   // Bit  31
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
      unsigned FIOM      : 1;  // Bit  0
      unsigned reserved0 : 1;  // Bit  1
      unsigned LPE       : 1;  // Bit  2
      unsigned reserved1 : 1;  // Bit  3
      unsigned CBIE      : 2;  // Bits 5:4
      unsigned CBCFE     : 1;  // Bit  6
      unsigned CBZE      : 1;  // Bit  7
      unsigned reserved2 : 24; // Bits 31:8
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
      unsigned FIOM      : 1;  // Bit  0
      unsigned reserved0 : 1;  // Bit  1
      unsigned LPE       : 1;  // Bit  2
      unsigned SSE       : 1;  // Bit  3
      unsigned CBIE      : 2;  // Bits 5:4
      unsigned CBCFE     : 1;  // Bit  6
      unsigned CBZE      : 1;  // Bit  7
      unsigned reserved1 : 24; // Bits 31:8
      unsigned PMM       : 2;  // Bits 33:32
      uint64_t reserved2 : 30; // Bits 63:34
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
      unsigned FIOM      : 1;
      unsigned reserved0 : 1;
      unsigned LPE       : 1;
      unsigned SSE       : 1;
      unsigned CBIE      : 2;
      unsigned CBCFE     : 1;
      unsigned CBZE      : 1;
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
      unsigned FIOM      : 1;
      unsigned reserved0 : 1;
      unsigned LPE       : 1;
      unsigned SSE       : 1;
      unsigned CBIE      : 2;
      unsigned CBCFE     : 1;
      unsigned CBZE      : 1;
      unsigned reserved1 : 24;
      unsigned PMM       : 2;
      uint64_t reserved2 : 25;
      unsigned DTE       : 1;
      unsigned reserved3 : 1;
      unsigned ADUE      : 1;
      unsigned PBMTE     : 1;
      unsigned STCE      : 1;
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
      unsigned PMM       : 2;
      uint64_t reserved2 : 25;
      unsigned DTE       : 1;
      unsigned reserved3 : 1;
      unsigned ADUE      : 1;
      unsigned PBMTE     : 1;
      unsigned STCE      : 1;
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
      unsigned PRV         : 2;     // bits 1-0
      unsigned STEP        : 1;     // bit  2 
      unsigned NMIP        : 1;     // bit  3
      unsigned MPRVEN      : 1;     // bit  4
      unsigned V           : 1;     // bit  5
      unsigned CAUSE       : 3;     // bits 8-6
      unsigned STOPTIME    : 1;     // bit  9
      unsigned STOPCOUNT   : 1;     // bit  10
      unsigned STEPIE      : 1;     // bit  11
      unsigned EBREAKU     : 1;     // bit  12
      unsigned EBREAKS     : 1;     // bit  13
      unsigned reserved1   : 1;     // bit  14
      unsigned EBREAKM     : 1;     // bit  15
      unsigned EBREAKVU    : 1;     // bit  16
      unsigned EBREAKVS    : 1;     // bit  17
      unsigned reserved2   : 10;    // bit  27-18
      unsigned DEBUGVER    : 4;     // bit  31-28
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


  union HvictlFields
  {
    HvictlFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;
    struct
    {
      unsigned IPRIO  : 8;
      unsigned IPRIOM : 1;
      unsigned DPR    : 1;
      unsigned res0   : 6;
      unsigned IID    : 12;
      unsigned res1   : 2;
      unsigned VTI    : 1;
      uint64_t res2   : 33;
    } bits_;
  };


  /// Structure used to unpack/pack the fields of the MSECCFG register
  template <typename URV>
  union MseccfgFields;

  template <>
  union MseccfgFields<uint32_t>
  {
    MseccfgFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_;
    struct
    {
      unsigned MML   : 1;   // Bit  0
      unsigned MMWP  : 1;   // Bit  1
      unsigned RLB   : 1;   // Bit  2
      unsigned res0  : 5;   // Bits 7:3
      unsigned USEED : 1;   // Bit  8
      unsigned SSEED : 1;   // Bit  9
      unsigned MLPTE : 1;   // Bit  10
      unsigned res1  : 21;  // Bit  31:11
    } bits_;
  };

  template <>
  union MseccfgFields<uint64_t>
  {
    MseccfgFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;
    struct
    {
      unsigned MML   : 1;   // Bit  0
      unsigned MMWP  : 1;   // Bit  1
      unsigned RLB   : 1;   // Bit  2
      unsigned res0  : 5;   // Bits 7:3
      unsigned USEED : 1;   // Bit  8
      unsigned SSEED : 1;   // Bit  9
      unsigned MLPTE : 1;   // Bit  10
      unsigned res1  : 21;  // Bit  31:11
      unsigned PMM   : 2;   // Bits 33:32
      unsigned res2  : 30;  // Bits 63:34
    } bits_;
  };


  /// Structure used to unpack/pack the fields of the TCONTROL register
  template <typename URV>
  union TcontrolFields;

  template <>
  union TcontrolFields<uint32_t>
  {
    TcontrolFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_;
    struct
    {
      unsigned           : 3;    // Bits 2-0  : Reserved -- hardwired to zero.
      unsigned mte_      : 1;    // Bits 3    : Machine mode trigger enable.
      unsigned           : 3;    // Bits 4-6  : Reserved -- hardwired to zero.
      unsigned mpte_     : 1;    // Bits 7    : Machine mode previous trigger enable.
      unsigned           : 24;   // Bits 31-8 : Rserved -- hardwired to zero.
    } bits_;
  };


  template <>
  union TcontrolFields<uint64_t>
  {
    TcontrolFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;
    struct
    {
      unsigned           : 3;    // Bits 2-0  : Reserved -- hardwired to zero.
      unsigned mte_      : 1;    // Bits 3    : Machine mode trigger enable.
      unsigned           : 3;    // Bits 4-6  : Reserved -- hardwired to zero.
      unsigned mpte_     : 1;    // Bits 7    : Machine mode previous trigger enable.
      unsigned           : 24;   // Bits 31-8 : Reserved -- hardwired to zero.
      unsigned           : 32;   // Bits 63-32: Reserved -- hardwired to zero.
    } bits_;
  };

}
