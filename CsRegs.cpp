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

#include <iostream>
#include <algorithm>
#include <cassert>
#include <cfenv>
#include <array>
#include <bit>
#include <tuple>
#include "CsRegs.hpp"
#include "FpRegs.hpp"
#include "VecRegs.hpp"
#include "float-util.hpp"

using namespace WdRiscv;


template <typename URV>
CsRegs<URV>::CsRegs()
  : regs_(size_t(CsrNumber::MAX_CSR_) + 1)
{
  // Define CSR entries.
  defineMachineRegs();
  defineSupervisorRegs();
  defineUserRegs();
  defineHypervisorRegs();
  defineDebugRegs();
  defineVectorRegs();
  defineFpRegs();
  defineAiaRegs();
  defineStateEnableRegs();
}


template <typename URV>
CsRegs<URV>::~CsRegs()
{
  regs_.clear();
  nameToNumber_.clear();
}


template <typename URV>
Csr<URV>*
CsRegs<URV>::defineCsr(std::string name, CsrNumber csrn, bool mandatory,
		       bool implemented, URV resetValue, URV writeMask,
		       URV pokeMask, bool isDebug, bool quiet)
{
  size_t ix = size_t(csrn);

  if (ix >= regs_.size())
    return nullptr;

  if (nameToNumber_.contains(name))
    {
      if (not quiet)
	std::cerr << "Error: CSR " << name << " already defined\n";
      return nullptr;
    }

  auto& csr = regs_.at(ix);
  if (csr.isDefined())
    {
      if (not quiet)
	std::cerr << "Error: CSR 0x" << std::hex << size_t(csrn) << std::dec
		  << " is already defined as " << csr.getName() << '\n';
      return nullptr;
    }

  using CN = CsrNumber;
  PrivilegeMode priv = PrivilegeMode((ix & 0x300) >> 8);
  if (priv != PrivilegeMode::Reserved)
    csr.definePrivilegeMode(priv);
  else if ((ix >= size_t(CN::HSTATUS) and ix <= size_t(CN::HCONTEXT)) or
           (ix >= size_t(CN::VSSTATUS) and ix <= size_t(CN::VSATP)) or
           (ix == size_t(CN::HGEIP)) or (ix == size_t(CN::VSTOPI)))
    // bits 8 and 9 not sufficient for hypervisor CSRs
    csr.definePrivilegeMode(PrivilegeMode::Supervisor);
  else
    assert(false);

  csr.setDefined(true);

  csr.config(name, csrn, mandatory, implemented, resetValue, writeMask,
	     pokeMask, isDebug);

  nameToNumber_.insert_or_assign(std::move(name), csrn);
  return &csr;
}


template <typename URV>
Csr<URV>*
CsRegs<URV>::findCsr(std::string_view name)
{
  const auto iter = nameToNumber_.find(name);
  if (iter == nameToNumber_.end())
    return nullptr;

  size_t num = size_t(iter->second);
  if (num >= regs_.size())
    return nullptr;

  return &regs_.at(num);
}


template <typename URV>
Csr<URV>*
CsRegs<URV>::findCsr(CsrNumber number)
{
  size_t ix = size_t(number);
  if (ix >= regs_.size())
    return nullptr;
  return &regs_.at(ix);
}


template <typename URV>
const Csr<URV>*
CsRegs<URV>::findCsr(CsrNumber number) const
{
  size_t ix = size_t(number);
  if (ix >= regs_.size())
    return nullptr;
  return &regs_.at(ix);
}


template <typename URV>
Csr<URV>*
CsRegs<URV>::getImplementedCsr(CsrNumber num, bool virtualMode)
{
  auto csr = getImplementedCsr(num);
  if (not csr)
    return csr;
  if (not virtualMode)
    return csr;
  if (not csr->mapsToVirtual())
    return csr;
  num = CsrNumber(URV(num) + 0x100);
  return getImplementedCsr(num);
}


template <typename URV>
const Csr<URV>*
CsRegs<URV>::getImplementedCsr(CsrNumber num, bool virtualMode) const
{
  auto csr = getImplementedCsr(num);
  if (not csr)
    return csr;
  if (not virtualMode)
    return csr;
  if (not csr->mapsToVirtual())
    return csr;
  num = CsrNumber(URV(num) + 0x100);
  return getImplementedCsr(num);
}


template <typename URV>
URV
CsRegs<URV>::adjustSipSieValue(URV value) const
{
  // Read value of MIP/SIP is masked by MIDELG/HIDELEG.
  auto deleg = getImplementedCsr(CsrNumber::MIDELEG);
  if (deleg)
    {
      value &= deleg->read();
      if (virtMode_)
	{
	  auto hdeleg = getImplementedCsr(CsrNumber::HIDELEG);
	  if (hdeleg)
	    value &= (hdeleg->read() >> 1);  // Bit positions in HIDELG are shifted.
	}
    }
  return value;
}


template <typename URV>
URV
CsRegs<URV>::adjustTimeValue(CsrNumber num, URV value) const
{
  if (not virtMode_)
    return value;

  auto delta = getImplementedCsr(CsrNumber::HTIMEDELTA);
  if (num == CsrNumber::TIME)
    {
      if (delta)
	value += delta->read();
    }
  else if (num == CsrNumber::TIMEH)
    {
      auto time = getImplementedCsr(CsrNumber::TIME);
      auto deltah = getImplementedCsr(CsrNumber::HTIMEDELTAH);
      if (time and delta and deltah)
	{
	  uint64_t t64 = (uint64_t(value) << 32) | uint32_t(time->read());
	  uint64_t d64 = (uint64_t(deltah->read()) << 32) | uint32_t(delta->read());
	  t64 += d64;
	  value = t64 >> 32;
	}
    }
  return value;
}


template <typename URV>
URV
CsRegs<URV>::adjustSstateenValue(CsrNumber num, URV value) const
{
  using CN = CsrNumber;

  if (num >= CN::SSTATEEN0 and num <= CN::SSTATEEN3)
    {
      CN base = CN::SSTATEEN0;
      unsigned ix = unsigned(num) - unsigned(base);

      constexpr bool rv32 = sizeof(URV) == 4;

      constexpr unsigned msbIx = sizeof(URV)*8 - 1; // Most sig bit
      auto mcsr0 = getImplementedCsr(rv32? CN::MSTATEEN0H : CN::MSTATEEN0);
      if (ix == 0 and mcsr0 and not ((mcsr0->read() >> msbIx) & 1))
	return false;  // SSTATEN0 not accessible because MSB of MSTATEEN0 is 0.


      // If a bit is zero in MSTATEEN, it becomes zero in SSTATEEN
      CsrNumber mnum = CsrNumber(unsigned(CN::MSTATEEN0) + ix);
      auto mcsr = getImplementedCsr(mnum);
      if (mcsr)
	  value &= mcsr->read();

      if (virtMode_)
	{
	  auto hcsr0 = getImplementedCsr(rv32? CN::HSTATEEN0H : CN::HSTATEEN0);
	  if (ix == 0 and hcsr0 and not ((hcsr0->read() >> msbIx) & 1))
	    return false;  // SSTATEN0 not accessible because bit 63 of HSTATEEN0 is 0.

	  CsrNumber hnum = CsrNumber(unsigned(CN::HSTATEEN0) + ix);
	  auto hcsr = getImplementedCsr(hnum);
	  if (hcsr)
	    value &= hcsr->read();
	}
    }
  return value;
}


template <typename URV>
URV
CsRegs<URV>::adjustHstateenValue(CsrNumber num, URV value) const
{
  using CN = CsrNumber;

  if ((num >= CN::HSTATEEN0 and num <= CN::HSTATEEN3) or
      (num >= CN::HSTATEEN0H and num <= CN::HSTATEEN3H))

    {
      CN base = CN::HSTATEEN0;
      if (num >= CN::HSTATEEN0H and num <= CN::HSTATEEN3H)
	base = CN::HSTATEEN0H;
      unsigned ix = unsigned(num) - unsigned(base);

      constexpr bool rv32 = sizeof(URV) == 4;
      constexpr unsigned msbIx = sizeof(URV)*8 - 1; // Most sig bit
      auto mcsr0 = getImplementedCsr(rv32? CN::MSTATEEN0H : CN::MSTATEEN0);
      if (ix == 0 and mcsr0 and not ((mcsr0->read() >> msbIx) & 1))
	return false;  // SSTATEN0 not accessible because MSB of MSTATEEN0 is 0.

      // If a bit is zero in MSTATEEN, it becomes zero in HSTATEEN
      CsrNumber mnum = CsrNumber(unsigned(CN::MSTATEEN0) + ix);
      auto mcsr = getImplementedCsr(mnum);
      if (mcsr)
	value &= mcsr->read();
    }
  return value;
}


template <typename URV>
bool
CsRegs<URV>::read(CsrNumber num, PrivilegeMode mode, URV& value) const
{
  using CN = CsrNumber;

  auto csr = getImplementedCsr(num, virtMode_);
  if (not csr)
    return false;

  if (mode < csr->privilegeMode())
    return false;

  if (csr->isDebug() and not inDebugMode())
    return false; // Debug-mode register.

  if (num >= CN::TDATA1 and num <= CN::TDATA3)
    return readTdata(num, mode, value);

  if (num == CN::FFLAGS or num == CN::FRM)
    {
      auto fcsr = getImplementedCsr(CN::FCSR);
      if (not fcsr)
        return false;
      value = fcsr->read();
      if (num == CN::FFLAGS)
        value = value & URV(FpFlags::FcsrMask);
      else
        value = (value & URV(RoundingMode::FcsrMask)) >> URV(RoundingMode::FcsrShift);
      return true;
    }

  value = csr->read();

  if (num == CN::SIP or num == CN::SIE)
    value = adjustSipSieValue(value);  // Mask by delegation registers.
  else if (virtMode_ and (num == CN::TIME or num == CN::TIMEH))
    value = adjustTimeValue(num, value);  // In virt mode, time is time + htimedelta.
  else if (num >= CN::PMPADDR0 and num <= CN::PMPADDR63)
    value = adjustPmpValue(num, value);
  else if (num >= CN::SSTATEEN0 and num <= CN::SSTATEEN3)
    value = adjustSstateenValue(num, value);
  else if (num >= CN::HSTATEEN0 and num <= CN::HSTATEEN3)
    value = adjustHstateenValue(num, value);

  return true;
}


template <typename URV>
bool
CsRegs<URV>::readSignExtend(CsrNumber number, PrivilegeMode mode, URV& value) const
{
  if (not read(number, mode, value))
    return false;
  if (value == 0)
    return true;

  auto csr = getImplementedCsr(number, virtMode_);
  URV mask = csr->getWriteMask();
  unsigned lz = std::countl_zero(mask);

  using SRV = typename std::make_signed_t<URV>;
  SRV svalue = value;
  svalue = (svalue << lz) >> lz;
  value = svalue;
  return true;
}


template <typename URV>
void
CsRegs<URV>::enableSupervisorMode(bool flag)
{
  superEnabled_ = flag;

  using CN = CsrNumber;

  for (auto csrn : { CN::SSTATUS, CN::SIE, CN::STVEC, CN::SCOUNTEREN,
		     CN::SSCRATCH, CN::SEPC, CN::SCAUSE, CN::STVAL, CN::SIP,
		     CN::SENVCFG, CN::SATP, CN::MEDELEG, CN::MIDELEG } )
    {
      auto csr = findCsr(csrn);
      if (not csr)
        {
          std::cerr << "Error: enableSupervisorMode: CSR number 0x"
                    << std::hex << URV(csrn) << std::dec << " undefined\n";
          assert(0);
        }
      else
        csr->setImplemented(flag);
    }

  if (hyperEnabled_)
    {
      for (auto csrn : { CN::VSSTATUS, CN::VSIE, CN::VSTVEC, CN::VSSCRATCH,
			 CN::VSEPC, CN::VSCAUSE, CN::VSTVAL, CN::VSIP, CN::VSATP } )
	{
	  auto csr = findCsr(csrn);
	  if (not csr)
	    {
	      std::cerr << "Error: enableSupervisorMode: CSR number 0x"
			<< std::hex << URV(csrn) << std::dec << " undefined\n";
	    }
	  else
	    csr->setImplemented(flag);
	}
    }

  using IC = InterruptCause;

  // In MIP/MIE, make writable/pokable bits corresponding to
  // SEIP/STIP/SSIP (supervisor external/timer/software interrupt
  // pending) when sstc is enabled and read-only-zero when supervisor
  // is disabled.
  URV sbits = ( URV(1) << unsigned(IC::S_EXTERNAL) |
		URV(1) << unsigned(IC::S_TIMER)    |
		URV(1) << unsigned(IC::S_SOFTWARE) );

  for (auto csrn : { CN::MIP, CN::MIE } )
    {
      auto csr = findCsr(csrn);
      if (csr)
	{
	  URV mask = csr->getWriteMask();
	  mask = flag? mask | sbits : mask & ~sbits;
	  csr->setWriteMask(mask);

	  mask = csr->getPokeMask();
	  mask = flag? mask | sbits : mask & ~sbits;
	  csr->setPokeMask(mask);
	}
    }

  updateSstc();  // To activate/deactivate STIMECMP.
  enableSscofpmf(cofEnabled_);  // To activate/deactivate SCOUNTOVF.
}


template <typename URV>
void
CsRegs<URV>::updateSstc()
{
  bool flag = sstcEnabled_;

  flag = flag and superEnabled_;
  auto menv = getImplementedCsr(CsrNumber::MENVCFG);
  if (menv)
    flag = flag and menvcfgStce();
  
  findCsr(CsrNumber::STIMECMP)->setImplemented(flag);

  if (superEnabled_)
    {
      // S_TIMER bit in MIP is read-only if stimecmp is implemented and
      // writeable if it is not.
      auto mip = findCsr(CsrNumber::MIP);
      if (mip)
	{
	  URV mask = mip->getWriteMask();
	  URV stBit = URV(1) << unsigned(InterruptCause::S_TIMER);
	  mask = flag? mask & ~stBit : mask | stBit;
	  mip->setWriteMask(mask);
	}
    }

  flag = flag and hyperEnabled_;
  findCsr(CsrNumber::VSTIMECMP)->setImplemented(flag);
}


template <typename URV>
void
CsRegs<URV>::enableHypervisorMode(bool flag)
{
  hyperEnabled_ = flag;

  using CN = CsrNumber;
  for (auto csrn : { CN::HSTATUS, CN::HEDELEG, CN::HIDELEG, CN::HIE, CN::HCOUNTEREN,
	CN::HGEIE, CN::HTVAL, CN::HIP, CN::HVIP, CN::HTINST, CN::HGEIP, CN::HENVCFG,
	CN::HENVCFGH, CN::HGATP, CN::HCONTEXT, CN::HTIMEDELTA, CN::HTIMEDELTAH,
        CN::MTVAL2, CN::MTINST } )
    {
      auto csr = findCsr(csrn);
      if (not csr)
        {
          std::cerr << "Error: enableHypervisorMode: CSR number 0x"
                    << std::hex << URV(csrn) << std::dec << " undefined\n";
        }
      else
        csr->setImplemented(flag);
    }

  if (superEnabled_)
    {
      for (auto csrn : { CN::VSSTATUS, CN::VSIE, CN::VSTVEC, CN::VSSCRATCH,
			 CN::VSEPC, CN::VSCAUSE, CN::VSTVAL, CN::VSIP, CN::VSATP })
	{
	  auto csr = findCsr(csrn);
	  if (not csr)
	    {
	      std::cerr << "Error: enableHypervisorMode: CSR number 0x"
			<< std::hex << URV(csrn) << std::dec << " undefined\n";
	    }
	  else
	    csr->setImplemented(flag);
	}
    }

  if (flag)
    {
      // enable MPV and GVA bits
      uint64_t hyperBits;
      Csr<URV>* mstatus;
      if (not rv32_)
	{
	  hyperBits = uint64_t(0x3) << 38;
	  mstatus = findCsr(CN::MSTATUS);
	}
      else
	{
	  hyperBits = 0x3 << 6;
	  mstatus = findCsr(CN::MSTATUSH);
	}

      URV mask = mstatus->getWriteMask();
      mask |= hyperBits;
      mstatus->setWriteMask(mask);
      mask = mstatus->getPokeMask();
      mask |= hyperBits;
      mstatus->setPokeMask(mask);
    }

  if (flag)
    {
      // Make VSEIP, VSTIP, and VSSIP read-only one.
      auto csr = findCsr(CN::MIDELEG);
      auto sgeip = geilen_ ? URV(1) << 12 : 0;  // Bit SGEIP
      for (auto&& [getMaskFn, setMaskFn] : { std::pair{ &Csr<URV>::getWriteMask, &Csr<URV>::setWriteMask },
                                             std::pair{ &Csr<URV>::getPokeMask,  &Csr<URV>::setPokeMask } })
        {
          auto mask = (csr->*getMaskFn)();
          mask &= ~URV(0x444) & ~sgeip;
          (csr->*setMaskFn)(mask);
        }

      for (auto&& [getValueFn, setValueFn] : { std::pair{ &Csr<URV>::getResetValue, &Csr<URV>::setInitialValue },
                                               std::pair{ &Csr<URV>::read,          &Csr<URV>::poke } })
        {
          auto value = (csr->*getValueFn)();
          auto newValue = value | sgeip | 0x444; // Bits VSEIP, VSTIP, and VSSIP.
          if (value != newValue)
            (csr->*setValueFn)(newValue);
        }
    }

  updateSstc();  // To activate/deactivate VSTIMECMP.
}


template <typename URV>
void
CsRegs<URV>::enableRvf(bool flag)
{
  for (auto csrn : { CsrNumber::FCSR, CsrNumber::FFLAGS, CsrNumber::FRM } )
    {
      auto csr = findCsr(csrn);
      if (not csr)
        {
          std::cerr << "Error: enableRvf: CSR number 0x"
                    << std::hex << URV(csrn) << std::dec << " undefined\n";
          assert(0);
        }
      else if (not csr->isImplemented())
        csr->setImplemented(flag);
    }
}


template <typename URV>
void
CsRegs<URV>::enableSscofpmf(bool flag)
{
  cofEnabled_ = flag;

  flag &= superEnabled_;

  auto csrn = CsrNumber::SCOUNTOVF;
  auto csr = findCsr(csrn);
  if (not csr)
    {
      std::cerr << "Error: enableSscofpmf: CSR number 0x"
		<< std::hex << URV(csrn) << std::dec << " is not defined\n";
      assert(0);
    }
  else
    csr->setImplemented(flag);

  mPerfRegs_.enableOverflow(flag);
  if (flag and not mPerfRegs_.ovfCallback_)
    {
      // Define callback to be invoked when a counter overflows. The
      // callback sets the LCOF bit of the MIP CSR.
      mPerfRegs_.ovfCallback_ = [this](unsigned ix) {
	assert(ix < 29);

	// Get value of MHPMEVENT CSR corresponding to counter.
	uint64_t mhpmVal = 0;
	if (not this->getMhpmeventValue(ix, mhpmVal))
	  return; // Should not happen.

	MhpmeventFields fields(mhpmVal);
	if (fields.bits_.OF)
	  return;   // Overflow bit already set: No interrupt.

	fields.bits_.OF = 1;

	CsrNumber evnum = CsrNumber(unsigned(CsrNumber::MHPMEVENT3) + ix);
	if (rv32_)
	  evnum = CsrNumber(unsigned(CsrNumber::MHPMEVENTH3) + ix);
	auto event = this->findCsr(evnum);
	if (not event)
	  {
	    assert(0);
	    return;
	  }

	if (rv32_)
	  event->poke(fields.value_ >> 32);
	else
	  event->poke(fields.value_);
	this->recordWrite(evnum);

	auto mip = this->findCsr(CsrNumber::MIP);
	if (mip)
	  {
	    URV newVal = mip->read() | (1 << URV(InterruptCause::LCOF));
	    mip->poke(newVal);
	    recordWrite(CsrNumber::MIP);
	  }
      };
    }
}


template <typename URV>
void
CsRegs<URV>::enableVectorExtension(bool flag)
{
  for (auto csrn : { CsrNumber::VSTART, CsrNumber::VXSAT, CsrNumber::VXRM,
		     CsrNumber::VCSR, CsrNumber::VL, CsrNumber::VTYPE,
		     CsrNumber::VLENB } )
    {
      auto csr = findCsr(csrn);
      if (not csr)
        {
          std::cerr << "Error: enableVectorExtension: CSR number 0x"
                    << std::hex << URV(csrn) << std::dec << " undefined\n";
          assert(0);
        }
      else
        csr->setImplemented(flag);
    }
}


template <typename URV>
URV
CsRegs<URV>::legalizeMstatusValue(URV value) const
{
  MstatusFields<URV> fields(value);
  PrivilegeMode mode = PrivilegeMode(fields.bits_.MPP);

  if (fields.bits_.FS == unsigned(FpStatus::Dirty) or fields.bits_.XS == unsigned(FpStatus::Dirty) or
      fields.bits_.VS == unsigned(VecStatus::Dirty))
    fields.bits_.SD = 1;
  else
    fields.bits_.SD = 0;

  if (mode == PrivilegeMode::Machine)
    return fields.value_;

  if (mode == PrivilegeMode::Supervisor and not superEnabled_)
    mode = PrivilegeMode::User;

  if (mode == PrivilegeMode::Reserved)
    mode = PrivilegeMode::User;

  if (mode == PrivilegeMode::User and not userEnabled_)
    mode = PrivilegeMode::Machine;

  fields.bits_.MPP = unsigned(mode);

  return fields.value_;
}


template <typename URV>
URV
legalizeMisa(Csr<URV>* csr, URV v)
{
  URV wm = csr->getWriteMask();
  if (wm == 0)
    return csr->getResetValue();

  v = (v & wm) | (csr->read() & ~wm) ;

  // E must be complement of I
  bool i = v & (1 << ('I' - 'A'));
  bool e = v & (1 << ('E' - 'A'));
  if (e == i)
    v ^= v & (1 << ('E' - 'A'));  // Flip E bit.
      
  if ((v & (1 << ('F' - 'A'))) == 0)
    v &= ~(URV(1) << ('D' - 'A'));  // D is off if F is off.

  if ((v & (1 << ('F' - 'A'))) == 0 or (v & (1 << ('D' - 'A'))) == 0)
    v &= ~(URV(1) << ('V' - 'A'));  // V is off if F or D is off.

  return v;
}


template <typename URV>
bool
CsRegs<URV>::writeSipSie(CsrNumber num, URV value)
{
  using CN = CsrNumber;

  if (num == CN::SIP or num == CN::SIE)
    {
      Csr<URV>* csr = getImplementedCsr(num, virtMode_);
      if (not csr)
	return false;

      // Get corresponding M register (MIP or MIE).
      auto mcsr = getImplementedCsr(CN(unsigned(num) + 0x200));

      auto mideleg = getImplementedCsr(CN(CN::MIDELEG));

      URV prevMask = csr->getWriteMask();
      URV mask = prevMask;

      if (mcsr)
	mask &= mcsr->getWriteMask();
      mask &= mideleg ? mideleg->read() : 0; // Only delegated bits writeable

      csr->setWriteMask(mask);
      csr->write(value);
      csr->setWriteMask(prevMask);

      recordWrite(num);
      return true;
    }
  return false;
}


template <typename URV>
bool
CsRegs<URV>::writeSstateen(CsrNumber num, URV value)
{
  using CN = CsrNumber;

  if (num >= CN::SSTATEEN0 and num <= CN::SSTATEEN3)
    {
      CN base = CN::SSTATEEN0;
      unsigned ix = unsigned(num) - unsigned(base);

      Csr<URV>* csr = getImplementedCsr(num, virtMode_);
      if (not csr)
	return false;

      constexpr bool rv32 = sizeof(URV) == 4;
      constexpr unsigned msbIx = sizeof(URV)*8 - 1; // Most sig bit
      auto mcsr0 = getImplementedCsr(rv32? CN::MSTATEEN0H : CN::MSTATEEN0);
      if (ix == 0 and mcsr0 and not ((mcsr0->read() >> msbIx) & 1))
	return false;  // SSTATEN0 not accessible because MSB of MSTATEEN0 is 0.

      URV prevMask = csr->getWriteMask();
      URV mask = prevMask;

      CsrNumber mnum = CsrNumber(unsigned(CN::MSTATEEN0) + ix);
      auto mcsr = getImplementedCsr(mnum);
      if (mcsr)
	mask &= mcsr->read();

      if (virtMode_)
	{
	  auto hcsr0 = getImplementedCsr(rv32? CN::HSTATEEN0H : CN::HSTATEEN0);
	  if (ix == 0 and hcsr0 and not ((hcsr0->read() >> msbIx) & 1))
	    return false;  // SSTATEN0 not accessible because bit 63 of HSTATEEN0 is 0.

	  CsrNumber hnum = CsrNumber(unsigned(CN::HSTATEEN0) + ix);
	  auto hcsr = getImplementedCsr(hnum);
	  if (hcsr)
	    mask &= hcsr->read();
	}

      csr->setWriteMask(mask);
      csr->write(value);
      csr->setWriteMask(prevMask);
      recordWrite(num);
      return true;
    }

  return false;
}


template <typename URV>
bool
CsRegs<URV>::writeHstateen(CsrNumber num, URV value)
{
  using CN = CsrNumber;

  if ((num >= CN::HSTATEEN0 and num <= CN::HSTATEEN3) or
      (num >= CN::HSTATEEN0H and num <= CN::HSTATEEN3H))

    {
      CN base = CN::HSTATEEN0;
      if (num >= CN::HSTATEEN0H and num <= CN::HSTATEEN3H)
	base = CN::HSTATEEN0H;
      unsigned ix = unsigned(num) - unsigned(base);

      Csr<URV>* csr = getImplementedCsr(num, virtMode_);
      if (not csr)
	return false;

      constexpr bool rv32 = sizeof(URV) == 4;
      constexpr unsigned msbIx = sizeof(URV)*8 - 1; // Most sig bit
      auto mcsr0 = getImplementedCsr(rv32? CN::MSTATEEN0H : CN::MSTATEEN0);
      if (ix == 0 and mcsr0 and not ((mcsr0->read() >> msbIx) & 1))
	return false;  // HSSTATEN0 not accessible because MSB of MSTATEEN0 is 0.

      URV prevMask = csr->getWriteMask();
      URV mask = prevMask;

      CsrNumber mnum = CsrNumber(unsigned(CN::MSTATEEN0) + ix);
      auto mcsr = getImplementedCsr(mnum);
      if (mcsr)
	mask &= mcsr->read();

      csr->setWriteMask(mask);
      csr->write(value);
      csr->setWriteMask(prevMask);
      recordWrite(num);
      return true;
    }

  return false;
}


template <typename URV>
bool
CsRegs<URV>::write(CsrNumber num, PrivilegeMode mode, URV value)
{
  using CN = CsrNumber;

  Csr<URV>* csr = getImplementedCsr(num, virtMode_);
  if (not csr or mode < csr->privilegeMode() or csr->isReadOnly())
    return false;
  if (csr->isDebug() and not inDebugMode())
    return false; // Debug-mode register.

  if (isPmpaddrLocked(num))
    {
      recordWrite(num);
      return true;  // Writing a locked PMPADDR register has no effect.
    }

  if (num >= CN::TDATA1 and num <= CN::TDATA3)
    {
      if (not writeTdata(num, mode, value))
	return false;
      recordWrite(num);
      return true;
    }

  // Write mask of SIP/SIE is a combined with that of MIP/MIE and
  // delgation register.
  if (num == CN::SIP or num == CN::SIE)
    return writeSipSie(num, value);

  if (num >= CN::SSTATEEN0 and num <= CN::SSTATEEN3)
    return writeSstateen(num, value);

  if (num >= CN::HSTATEEN0 and num <= CN::HSTATEEN3)
    return writeHstateen(num, value);

  if (num == CN::MSTATUS or num == CN::SSTATUS or num == CN::VSSTATUS)
    {
      value &= csr->getWriteMask();
      value = legalizeMstatusValue(value);
      csr->poke(value);   // Write cannot modify SD bit of status: poke it.
      recordWrite(num);

      // Cache interrupt enable from mstatus.mie.
      if (num == CN::MSTATUS)
	{
	  MstatusFields<URV> fields(csr->read());
	  interruptEnable_ = fields.bits_.MIE;
	}
      return true;
    }

  if (num == CN::MISA)
    {
      value = legalizeMisa(csr, value);
      csr->pokeNoMask(value);
      recordWrite(num);
      return true;
    }

  if (num >= CN::PMPCFG0 and num <= CN::PMPCFG15)
    {
      URV prev = 0;
      peek(num, prev);
      value = legalizePmpcfgValue(prev, value);
    }
   
  csr->write(value);
  recordWrite(num);

  if ((num >= CN::MHPMEVENT3 and num <= CN::MHPMEVENT31) or
      (num >= CN::MHPMEVENTH3 and num <= CN::MHPMEVENTH31))
    updateCounterControl(num);
  else if (num == CN::FFLAGS or num == CN::FRM or num == CN::FCSR)
    updateFcsrGroupForWrite(num, value);   // fflags and frm are part of fcsr
  else if (num == CN::VXSAT or num == CN::VXRM or num == CN::VCSR)
    updateVcsrGroupForWrite(num, value);   // vxsat and vrm are part of vcsr
  else if (num == CN::MCOUNTEREN or num == CN::SCOUNTEREN)
    updateCounterPrivilege();  // Reflect counter accessibility in user/supervisor.
  else 
    hyperWrite(csr);   // Update hypervisor CSR aliased bits.

  return true;
}


template <typename URV>
bool
CsRegs<URV>::isWriteable(CsrNumber number, PrivilegeMode mode ) const
{
  const Csr<URV>* csr = getImplementedCsr(number, virtMode_);
  if (not csr)
    return false;

  if (mode < csr->privilegeMode())
    return false;

  if (csr->isReadOnly())
    return false;

  if (csr->isDebug() and not inDebugMode())
    return false;  // Debug-mode register.

  return true;
}


template <typename URV>
bool
CsRegs<URV>::isReadable(CsrNumber number, PrivilegeMode mode ) const
{
  const Csr<URV>* csr = getImplementedCsr(number, virtMode_);
  if (not csr)
    return false;

  if (mode < csr->privilegeMode())
    return false;

  if (csr->isDebug() and not inDebugMode())
    return false;  // Debug-mode register.

  return true;
}



template <typename URV>
void
CsRegs<URV>::reset()
{
  for (auto& csr : regs_)
    if (csr.isImplemented())
      csr.reset();

  triggers_.reset();
  mPerfRegs_.reset();

  // Cache interrupt enable.
  Csr<URV>* mstatus = getImplementedCsr(CsrNumber::MSTATUS);
  if (mstatus)
    {
      MstatusFields<URV> fields(mstatus->read());
      interruptEnable_ = fields.bits_.MIE;
    }

  mdseacLocked_ = false;
}


template <typename URV>
bool
CsRegs<URV>::configCsr(std::string_view name, bool implemented, URV resetValue,
                       URV mask, URV pokeMask, bool isDebug, bool shared)
{
  auto iter = nameToNumber_.find(name);
  if (iter == nameToNumber_.end())
    return false;

  size_t num = size_t(iter->second);
  if (num >= regs_.size())
    return false;

  return configCsr(CsrNumber(num), implemented, resetValue, mask, pokeMask,
		   isDebug, shared);
}


template <typename URV>
bool
CsRegs<URV>::configCsr(CsrNumber csrNum, bool implemented, URV resetValue,
                       URV mask, URV pokeMask, bool isDebug, bool shared)
{
  if (size_t(csrNum) >= regs_.size())
    {
      std::cerr << "ConfigCsr: CSR number " << size_t(csrNum)
		<< " out of bound\n";
      return false;
    }

  auto& csr = regs_.at(size_t(csrNum));
  if (csr.isMandatory() and not implemented)
    {
      std::cerr << "CSR " << csr.getName() << " is mandatory and is being "
		<< "configured as not-implemented -- configuration ignored.\n";
      return false;
    }

  csr.setImplemented(implemented);
  csr.setInitialValue(resetValue);
  csr.setWriteMask(mask);
  csr.setPokeMask(pokeMask);
  csr.pokeNoMask(resetValue);
  csr.setIsDebug(isDebug);
  csr.setIsShared(shared);

  // Cache interrupt enable.
  if (csrNum == CsrNumber::MSTATUS)
    {
      MstatusFields<URV> fields(csr.read());
      interruptEnable_ = fields.bits_.MIE;

      // Update masks of sstatus.
      auto& sstatus = regs_.at(size_t(CsrNumber::SSTATUS));
      sstatus.setWriteMask(sstatus.getWriteMask() & csr.getWriteMask());
      sstatus.setPokeMask(sstatus.getPokeMask() & csr.getPokeMask());
    }

  return true;
}


template <typename URV>
bool
CsRegs<URV>::configMachineModePerfCounters(unsigned numCounters)
{
  if (numCounters > 29)
    {
      std::cerr << "No more than 29 machine mode performance counters "
		<< "can be defined\n";
      return false;
    }

  unsigned errors = 0;
  bool shared = false;

  for (unsigned i = 0; i < 29; ++i)
    {
      URV resetValue = 0, mask = ~URV(0), pokeMask = ~URV(0);
      if (i >= numCounters)
	mask = pokeMask = 0;

      CsrNumber csrNum = CsrNumber(i + unsigned(CsrNumber::MHPMCOUNTER3));
      bool isDebug = false;
      if (not configCsr(csrNum, true, resetValue, mask, pokeMask, isDebug,
                        shared))
	errors++;

      if (rv32_)
         {
	   csrNum = CsrNumber(i + unsigned(CsrNumber::MHPMCOUNTER3H));
	   if (not configCsr(csrNum, true, resetValue, mask, pokeMask,
                             isDebug, shared))
	     errors++;
	 }

      csrNum = CsrNumber(i + unsigned(CsrNumber::MHPMEVENT3));
      if (not configCsr(csrNum, true, resetValue, mask, pokeMask, isDebug,
                        shared))
	errors++;
    }

  if (errors == 0)
    {
      mPerfRegs_.config(numCounters);
      tiePerfCounters(mPerfRegs_.counters_);
    }

  return errors == 0;
}


template <typename URV>
bool
CsRegs<URV>::configUserModePerfCounters(unsigned numCounters)
{
  if (numCounters > mPerfRegs_.size())
    {
      std::cerr << "User mode number of performance counters (" << numCounters
                << ") cannot exceed that of machine mode ("
                << mPerfRegs_.size() << '\n';
      return false;
    }

  unsigned errors = 0;
  bool shared = false;

  for (unsigned i = 0; i < 29; ++i)
    {
      URV resetValue = 0, mask = ~URV(0), pokeMask = ~URV(0);
      if (i >= numCounters)
	mask = pokeMask = 0;

      CsrNumber csrNum = CsrNumber(i + unsigned(CsrNumber::HPMCOUNTER3));
      bool isDebug = false;
      if (not configCsr(csrNum, true, resetValue, mask, pokeMask, isDebug,
                        shared))
	errors++;

      if (rv32_)
         {
	   csrNum = CsrNumber(i + unsigned(CsrNumber::HPMCOUNTER3H));
	   if (not configCsr(csrNum, true, resetValue, mask, pokeMask,
                             isDebug, shared))
	     errors++;
	 }
    }

  return errors == 0;
}


template <typename URV>
void
CsRegs<URV>::updateFcsrGroupForWrite(CsrNumber number, URV value)
{
  if (number == CsrNumber::FFLAGS)
    {
      auto fcsr = getImplementedCsr(CsrNumber::FCSR);
      if (fcsr)
	{
          URV mask = URV(FpFlags::FcsrMask);
	  URV fcsrVal = fcsr->read();
          fcsrVal = (fcsrVal & ~mask) | (value & mask);
	  fcsr->write(fcsrVal);
	  // recordWrite(CsrNumber::FCSR);
	}
      return;
    }

  if (number == CsrNumber::FRM)
    {
      auto fcsr = getImplementedCsr(CsrNumber::FCSR);
      if (fcsr)
	{
	  URV fcsrVal = fcsr->read();
          URV mask = URV(RoundingMode::FcsrMask);
          URV shift = URV(RoundingMode::FcsrShift);
          fcsrVal = (fcsrVal & ~mask) | ((value << shift) & mask);
	  fcsr->write(fcsrVal);
	  // recordWrite(CsrNumber::FCSR);
          setSimulatorRoundingMode(RoundingMode((fcsrVal & mask) >> shift));
	}
      return;
    }

  if (number == CsrNumber::FCSR)
    {
      URV newVal = value & URV(FpFlags::FcsrMask);
      auto fflags = getImplementedCsr(CsrNumber::FFLAGS);
      if (fflags and fflags->read() != newVal)
	{
	  fflags->write(newVal);
	  // recordWrite(CsrNumber::FFLAGS);
	}

      newVal = (value & URV(RoundingMode::FcsrMask)) >> URV(RoundingMode::FcsrShift);
      auto frm = getImplementedCsr(CsrNumber::FRM);
      if (frm and frm->read() != newVal)
	{
	  frm->write(newVal);
	  // recordWrite(CsrNumber::FRM);
	}
      setSimulatorRoundingMode(RoundingMode(newVal));
    }
}


template <typename URV>
void
CsRegs<URV>::updateFcsrGroupForPoke(CsrNumber number, URV value)
{
  if (number == CsrNumber::FFLAGS)
    {
      auto fcsr = getImplementedCsr(CsrNumber::FCSR);
      if (fcsr)
	{
          URV mask = URV(FpFlags::FcsrMask);
	  URV fcsrVal = fcsr->read();
          fcsrVal = (fcsrVal & ~mask) | (value & mask);
	  fcsr->poke(fcsrVal);
	}
      return;
    }

  if (number == CsrNumber::FRM)
    {
      auto fcsr = getImplementedCsr(CsrNumber::FCSR);
      if (fcsr)
	{
	  URV fcsrVal = fcsr->read();
          URV mask = URV(RoundingMode::FcsrMask);
          URV shift = URV(RoundingMode::FcsrShift);
          fcsrVal = (fcsrVal & ~mask) | ((value << shift) & mask);
	  fcsr->poke(fcsrVal);
          setSimulatorRoundingMode(RoundingMode((fcsrVal & mask) >> shift));
	}
      return;
    }

  if (number == CsrNumber::FCSR)
    {
      URV newVal = value & URV(FpFlags::FcsrMask);
      auto fflags = getImplementedCsr(CsrNumber::FFLAGS);
      if (fflags and fflags->read() != newVal)
        fflags->poke(newVal);

      newVal = (value & URV(RoundingMode::FcsrMask)) >> URV(RoundingMode::FcsrShift);
      auto frm = getImplementedCsr(CsrNumber::FRM);
      if (frm and frm->read() != newVal)
        frm->poke(newVal);
      setSimulatorRoundingMode(RoundingMode(newVal));
    }
}


template <typename URV>
void
CsRegs<URV>::updateVcsrGroupForWrite(CsrNumber number, URV value)
{
  if (number == CsrNumber::VXSAT)
    {
      auto vcsr = getImplementedCsr(CsrNumber::VCSR);
      if (vcsr)
	{
          URV mask = 1;
	  URV vcsrVal = vcsr->read();
          vcsrVal = (vcsrVal & ~mask) | (value & mask);
	  vcsr->write(vcsrVal);
	  // recordWrite(CsrNumber::VCSR);
	}
      return;
    }

  if (number == CsrNumber::VXRM)
    {
      auto vcsr = getImplementedCsr(CsrNumber::VCSR);
      if (vcsr)
	{
	  URV vcsrVal = vcsr->read();
          URV mask = URV(VecRoundingMode::VcsrMask);
          URV shift = URV(VecRoundingMode::VcsrShift);
          vcsrVal = (vcsrVal & ~mask) | ((value << shift) & mask);
	  vcsr->write(vcsrVal);
	  // recordWrite(CsrNumber::VCSR);
	}
      return;
    }

  if (number == CsrNumber::VCSR)
    {
      URV newVal = value & 1;
      auto vxsat = getImplementedCsr(CsrNumber::VXSAT);
      if (vxsat and vxsat->read() != newVal)
	{
	  vxsat->write(newVal);
	  // recordWrite(CsrNumber::VXSAT);
	}

      newVal = (value & URV(VecRoundingMode::VcsrMask)) >> URV(VecRoundingMode::VcsrShift);
      auto vxrm = getImplementedCsr(CsrNumber::VXRM);
      if (vxrm and vxrm->read() != newVal)
	{
	  vxrm->write(newVal);
	  // recordWrite(CsrNumber::VXRM);
	}
    }
}


template <typename URV>
void
CsRegs<URV>::updateVcsrGroupForPoke(CsrNumber number, URV value)
{
  if (number == CsrNumber::VXSAT)
    {
      auto vcsr = getImplementedCsr(CsrNumber::VCSR);
      if (vcsr)
	{
          URV mask = 1;
	  URV vcsrVal = vcsr->read();
          vcsrVal = (vcsrVal & ~mask) | (value & mask);
	  vcsr->poke(vcsrVal);
	}
      return;
    }

  if (number == CsrNumber::VXRM)
    {
      auto vcsr = getImplementedCsr(CsrNumber::VCSR);
      if (vcsr)
	{
	  URV vcsrVal = vcsr->read();
          URV mask = URV(VecRoundingMode::VcsrMask);
          URV shift = URV(VecRoundingMode::VcsrShift);
          vcsrVal = (vcsrVal & ~mask) | ((value << shift) & mask);
	  vcsr->poke(vcsrVal);
	}
      return;
    }

  if (number == CsrNumber::VCSR)
    {
      URV newVal = value & 1;
      auto vxsat = getImplementedCsr(CsrNumber::VXSAT);
      if (vxsat and vxsat->read() != newVal)
        vxsat->poke(newVal);

      newVal = (value & URV(VecRoundingMode::VcsrMask)) >> URV(VecRoundingMode::VcsrShift);
      auto vxrm = getImplementedCsr(CsrNumber::VXRM);
      if (vxrm and vxrm->read() != newVal)
        vxrm->poke(newVal);
    }
}


template <typename URV>
void
CsRegs<URV>::recordWrite(CsrNumber num)
{
  if (not recordWrite_)
    return;
  auto& lwr = lastWrittenRegs_;
  if (std::find(lwr.begin(), lwr.end(), num) == lwr.end())
    lwr.push_back(num);
}


template <typename URV>
void
CsRegs<URV>::defineMachineRegs()
{
  URV rom = 0;        // Read-only mask: no bit writeable.
  URV wam = ~URV(0);  // Write-all mask: all bits writeable.

  bool mand = true;  // Mandatory.
  bool imp = true;   // Implemented.

  using Csrn = CsrNumber;

  // Machine info.
  defineCsr("mvendorid",  Csrn::MVENDORID,  mand, imp, 0, rom, rom);
  defineCsr("marchid",    Csrn::MARCHID,    mand, imp, 0, rom, rom);
  defineCsr("mimpid",     Csrn::MIMPID,     mand, imp, 0, rom, rom);
  defineCsr("mhartid",    Csrn::MHARTID,    mand, imp, 0, rom, rom);
  defineCsr("mconfigptr", Csrn::MCONFIGPTR, mand, imp, 0, rom, rom);

  // Machine status setup.

  // mstatus
  //           S R        T T T M S M X  F  M  V  S M U S U M R S U
  //           D E        S W V X U P S  S  P  S  P P B P P I E I I
  //             S        R   M R M R       P     P I E I I E S E E
  //                                V               E   E E
  URV mask = 0b0'00000000'1'1'1'1'1'1'11'11'11'11'1'1'0'1'0'1'0'1'0;
  URV val =  0b0'00000000'0'0'0'0'0'0'00'00'11'00'0'0'0'0'0'0'0'0'0;
  if (not rv32_)
    {
      mask |= uint64_t(0b0000) << 32;  // Mask for SXL and UXL (currently not writable).
      val |= uint64_t(0b1010) << 32;   // Value of SXL and UXL : sxlen=uxlen=64
    }
  URV pokeMask = mask | (URV(1) << (sizeof(URV)*8 - 1));  // Make SD pokable.

  defineCsr("mstatus", Csrn::MSTATUS, mand, imp, val, mask, pokeMask);
  if (rv32_)
    {
      mask = 0;
      auto c = defineCsr("mstatush", Csrn::MSTATUSH, mand, imp, 0, mask, mask);
      c->markAsHighHalf(true);
    }

  val = 0x4000112d;  // MISA: acdfim
  if constexpr (sizeof(URV) == 8)
    val = 0x800000000000112d;  // MISA: acdfim
  defineCsr("misa", Csrn::MISA, mand, imp, val, rom, rom);

  // Bits corresponding to reserved interrupts are hardwired to zero
  // in medeleg.
  URV userBits = ( (URV(1) << unsigned(ExceptionCause::RESERVED0)) |
                   (URV(1) << unsigned(ExceptionCause::RESERVED1)) |
                   (URV(1) << unsigned(ExceptionCause::RESERVED2)) |
                   (URV(1) << unsigned(ExceptionCause::RESERVED3)));
  mask = wam & ~ userBits;
  defineCsr("medeleg", Csrn::MEDELEG, !mand, !imp, 0, mask, mask);

  defineCsr("mideleg", Csrn::MIDELEG, !mand, !imp, 0, wam, wam);

  // Interrupt enable: Least sig 12 bits corresponding to the 12
  // interrupt causes are writable.
  // TODO: SGEIE (bit 12)
  URV mieMask = 0xfff; 
  defineCsr("mie", Csrn::MIE, mand, imp, 0, mieMask, mieMask);

  // Initial value of 0: vectored interrupt. Mask of ~2 to make bit 1
  // non-writable.
  mask = ~URV(2);
  defineCsr("mtvec", Csrn::MTVEC, mand, imp, 0, mask, mask);

  defineCsr("mcounteren", Csrn::MCOUNTEREN, !mand, imp, 0, wam, wam);

  mask = ~URV(2);  // All bits writeable except 1.
  defineCsr("mcountinhibit", Csrn::MCOUNTINHIBIT, !mand, imp, 0, mask, mask);

  // Machine trap handling: mscratch and mepc.
  defineCsr("mscratch", Csrn::MSCRATCH, mand, imp, 0, wam, wam);
  mask = ~URV(1);  // Bit 0 of MEPC is not writable.
  defineCsr("mepc", Csrn::MEPC, mand, imp, 0, mask, mask);

  // All bits of mcause writeable.
  defineCsr("mcause", Csrn::MCAUSE, mand, imp, 0, wam, wam);
  defineCsr("mtval", Csrn::MTVAL, mand, imp, 0, wam, wam);

  // MIP is read-only for CSR instructions but the bits corresponding
  // to defined interrupts are modifiable.
  defineCsr("mip", CsrNumber::MIP, mand, imp, 0, rom, mieMask | 0x3000);

  // Physical memory protection. Odd-numbered PMPCFG are only present
  // in 32-bit implementations.
  uint64_t cfgMask = 0x9f9f9f9f;
  if (not rv32_)
    cfgMask = 0x9f9f9f9f9f9f9f9fL;
  defineCsr("pmpcfg0",   Csrn::PMPCFG0,   !mand, imp, 0, cfgMask, cfgMask);
  defineCsr("pmpcfg2",   Csrn::PMPCFG2,   !mand, imp, 0, cfgMask, cfgMask);
  defineCsr("pmpcfg4",   Csrn::PMPCFG4,   !mand, imp, 0, cfgMask, cfgMask);
  defineCsr("pmpcfg6",   Csrn::PMPCFG6,   !mand, imp, 0, cfgMask, cfgMask);
  defineCsr("pmpcfg8",   Csrn::PMPCFG8,   !mand, imp, 0, cfgMask, cfgMask);
  defineCsr("pmpcfg10",  Csrn::PMPCFG10,  !mand, imp, 0, cfgMask, cfgMask);
  defineCsr("pmpcfg12",  Csrn::PMPCFG12,  !mand, imp, 0, cfgMask, cfgMask);
  defineCsr("pmpcfg14",  Csrn::PMPCFG14,  !mand, imp, 0, cfgMask, cfgMask);
  if (rv32_)
    {
      defineCsr("pmpcfg1",   Csrn::PMPCFG1,   !mand, imp, 0, cfgMask, cfgMask);
      defineCsr("pmpcfg3",   Csrn::PMPCFG3,   !mand, imp, 0, cfgMask, cfgMask);
      defineCsr("pmpcfg5",   Csrn::PMPCFG5,   !mand, imp, 0, cfgMask, cfgMask);
      defineCsr("pmpcfg7",   Csrn::PMPCFG7,   !mand, imp, 0, cfgMask, cfgMask);
      defineCsr("pmpcfg9",   Csrn::PMPCFG9,   !mand, imp, 0, cfgMask, cfgMask);
      defineCsr("pmpcfg11",  Csrn::PMPCFG11,  !mand, imp, 0, cfgMask, cfgMask);
      defineCsr("pmpcfg13",  Csrn::PMPCFG13,  !mand, imp, 0, cfgMask, cfgMask);
      defineCsr("pmpcfg15",  Csrn::PMPCFG15,  !mand, imp, 0, cfgMask, cfgMask);
    }
  else
    {
      defineCsr("pmpcfg1",   Csrn::PMPCFG1,   !mand, !imp, 0, cfgMask, cfgMask);
      defineCsr("pmpcfg3",   Csrn::PMPCFG3,   !mand, !imp, 0, cfgMask, cfgMask);
      defineCsr("pmpcfg5",   Csrn::PMPCFG5,   !mand, !imp, 0, cfgMask, cfgMask);
      defineCsr("pmpcfg7",   Csrn::PMPCFG7,   !mand, !imp, 0, cfgMask, cfgMask);
      defineCsr("pmpcfg9",   Csrn::PMPCFG9,   !mand, !imp, 0, cfgMask, cfgMask);
      defineCsr("pmpcfg11",  Csrn::PMPCFG11,  !mand, !imp, 0, cfgMask, cfgMask);
      defineCsr("pmpcfg13",  Csrn::PMPCFG13,  !mand, !imp, 0, cfgMask, cfgMask);
      defineCsr("pmpcfg15",  Csrn::PMPCFG15,  !mand, !imp, 0, cfgMask, cfgMask);
    }

  uint64_t pmpMask = 0xffffffff;
  if (not rv32_)
    pmpMask = 0x003f'ffff'ffff'ffffL; // Top 10 bits are zeros

  for (unsigned i = 0; i < 64; ++i)
    {
      std::string name = std::string("pmpaddr") + std::to_string(i);
      Csrn num = Csrn{unsigned(Csrn::PMPADDR0) + i};
      defineCsr(std::move(name), num,  !mand, imp, 0, pmpMask, pmpMask);
    }

  uint64_t menvMask = 0xf1;
  if (not rv32_)
    menvMask = 0xc0000000000000f1;
  defineCsr("menvcfg", Csrn::MENVCFG, !mand, imp, 0, menvMask, menvMask);
  if (rv32_)
    {
      menvMask = 0xc0000000;
      auto c = defineCsr("menvcfgh", Csrn::MENVCFGH, !mand, imp, 0, menvMask, menvMask);
      c->markAsHighHalf(true);
    }

  // Machine Counter/Timers.
  defineCsr("mcycle",    Csrn::MCYCLE,    mand, imp, 0, wam, wam);
  defineCsr("minstret",  Csrn::MINSTRET,  mand, imp, 0, wam, wam);
  if (rv32_)
    {
      auto c = defineCsr("mcycleh",   Csrn::MCYCLEH,   mand, imp, 0, wam, wam);
      c->markAsHighHalf(true);
      c = defineCsr("minstreth", Csrn::MINSTRETH, mand, imp, 0, wam, wam);
      c->markAsHighHalf(true);
    }

  // Define mhpmcounter3/mhpmcounter3h to mhpmcounter31/mhpmcounter31h
  // as write-anything/read-zero (user can change that in the config
  // file by setting the number of writeable counters). Same for
  // mhpmevent3/mhpmevent3h to mhpmevent3h/mhpmevent31h.
  for (unsigned i = 3; i <= 31; ++i)
    {
      CsrNumber csrNum = CsrNumber(unsigned(CsrNumber::MHPMCOUNTER3) + i - 3);
      std::string name = "mhpmcounter" + std::to_string(i);
      defineCsr(name, csrNum, mand, imp, 0, rom, rom);

      if (rv32_)
        {
          // High register counterpart of mhpmcounter.
          name += "h";
          csrNum = CsrNumber(unsigned(CsrNumber::MHPMCOUNTER3H) + i - 3);
          bool hmand = rv32_;  // high counters mandatory only in rv32
          auto c = defineCsr(std::move(name), csrNum, hmand, imp, 0, rom, rom);
	  c->markAsHighHalf(true);
        }

      csrNum = CsrNumber(unsigned(CsrNumber::MHPMEVENT3) + i - 3);
      name = "mhpmevent" + std::to_string(i);
      defineCsr(std::move(name), csrNum, mand, imp, 0, rom, rom);
    }

  // add CSR fields
  addMachineFields();
}


template <typename URV>
void
CsRegs<URV>::tieSharedCsrsTo(CsRegs<URV>& target)
{
  if (this == &target)
    return;

  assert(regs_.size() == target.regs_.size());
  for (size_t i = 0; i < regs_.size(); ++i)
    {
      CsrNumber csrn = CsrNumber(i);
      auto csr = getImplementedCsr(csrn);
      auto targetCsr = target.getImplementedCsr(csrn);
      if (csr)
        {
          assert(targetCsr);
          if (csr->isShared())
            {
              assert(targetCsr->isShared());
              csr->tie(targetCsr->valuePtr_);
            }
        }
      else
        assert(not targetCsr);
    }
}


template <typename URV>
void
CsRegs<URV>::tiePerfCounters(std::vector<uint64_t>& counters)
{
  // Since the user-mode counters are a shadow of their machine-mode
  // counterparts, we tie them as well regardless of whether or not
  // they are configured.

  if (rv32_)
    {
      // Tie each mhpmcounter CSR value to the least significant 4
      // bytes of the corresponding counters_ entry. Tie each
      // mhpmcounterh CSR value to the most significan 4 bytes of the
      // corresponding counters_ entry.
      for (unsigned num = 3; num <= 31; ++num)
	{
	  unsigned ix = num - 3;
	  if (ix >= counters.size())
	    break;

	  unsigned highIx = ix +  unsigned(CsrNumber::MHPMCOUNTER3H);
	  Csr<URV>& csrHigh = regs_.at(highIx);
	  URV* low = reinterpret_cast<URV*>(&counters.at(ix));
          URV* high = low + 1;
	  csrHigh.tie(high);

	  unsigned lowIx = ix +  unsigned(CsrNumber::MHPMCOUNTER3);
	  Csr<URV>& csrLow = regs_.at(lowIx);
	  csrLow.tie(low);

          // Tie the user-mode performance counter to their
          // machine-mode counterparts.
          highIx = ix +  unsigned(CsrNumber::HPMCOUNTER3H);
          regs_.at(highIx).tie(high);
          lowIx = ix +  unsigned(CsrNumber::HPMCOUNTER3);
          regs_.at(lowIx).tie(low);
	}
    }
  else
    {
      for (unsigned num = 3; num <= 31; ++num)
	{
	  unsigned ix = num - 3;
	  if (ix >= counters.size())
	    break;
	  unsigned csrIx = ix +  unsigned(CsrNumber::MHPMCOUNTER3);
	  Csr<URV>& csr = regs_.at(csrIx);
	  URV* loc = reinterpret_cast<URV*>(&counters.at(ix));
	  csr.tie(loc);

          // Tie user-mode perf register to corresponding machine mode reg.
          csrIx = ix +  unsigned(CsrNumber::HPMCOUNTER3);
          regs_.at(csrIx).tie(loc);
	}
    }
}


template <typename URV>
void
CsRegs<URV>::defineSupervisorRegs()
{
  bool mand = true;   // Mandatory.
  bool imp = true;    // Implemented.
  URV wam = ~ URV(0); // Write-all mask: all bits writeable.

  // Supervisor trap SETUP_CSR.

  using Csrn = CsrNumber;

  // sstatus
  //           S R        T T T M S M X  F  M  V  S M U S R M R S R
  //           D E        S W V X U P S  S  P  S  P P B P E I E I E
  //             S        R   M R M R       P     P I E I S E S E S
  //                                V               E   E  
  URV mask = 0b0'00000000'0'0'0'1'1'0'11'11'00'11'1'0'0'1'0'0'0'1'0;
  URV pokeMask = mask | (URV(1) << (sizeof(URV)*8 - 1));  // Make SD pokable.
  defineCsr("sstatus",    Csrn::SSTATUS,    !mand, !imp, 0, mask, pokeMask);

  auto sstatus = findCsr(Csrn::SSTATUS);
  if (sstatus)
    {
      // SSTATUS tied to MSTATUS but not all bits are readable.
      sstatus->setReadMask(0x800de762L);
      if constexpr (sizeof(URV) == 8)
	sstatus->setReadMask(0x80000003000de762L);
      sstatus->setMapsToVirtual(true);
    }

  // SSTATUS shadows MSTATUS
  auto mstatus = findCsr(Csrn::MSTATUS);
  if (sstatus and mstatus)
    sstatus->tie(mstatus->valuePtr_);

  defineCsr("stvec",      Csrn::STVEC,      !mand, !imp, 0, wam, wam);
  defineCsr("scounteren", Csrn::SCOUNTEREN, !mand, !imp, 0, wam, wam);

  // Supervisor Trap Handling 
  defineCsr("sscratch",   Csrn::SSCRATCH,   !mand, !imp, 0, wam, wam);
  mask = ~URV(1);  // Bit 0 of SEPC is not writable.
  defineCsr("sepc",       Csrn::SEPC,       !mand, !imp, 0, mask, mask);
  defineCsr("scause",     Csrn::SCAUSE,     !mand, !imp, 0, wam, wam);
  defineCsr("stval",      Csrn::STVAL,      !mand, !imp, 0, wam, wam);

  // Bits of SIE appear hardwired to zreo unless delegated. By default
  // only ssie, stie, and seie are writeable.
  mask = 0x222;
  defineCsr("sie",        Csrn::SIE,        !mand, !imp, 0, mask, mask);
  auto sie = findCsr(Csrn::SIE);
  auto mie = findCsr(Csrn::MIE);
  if (sie and mie)
    sie->tie(mie->valuePtr_);

  // Bits of SIE appear hardwired to zreo unless delegated.
  mask = 0x2;  // Only ssip bit writable (when delegated)
  defineCsr("sip",        Csrn::SIP,        !mand, !imp, 0, mask, mask);

  auto sip = findCsr(Csrn::SIP);
  auto mip = findCsr(Csrn::MIP);
  if (sip and mip)
    sip->tie(mip->valuePtr_); // Sip is a shadow if mip

  mask = 0xf1;
  defineCsr("senvcfg",    Csrn::SENVCFG,    !mand, !imp, 0, mask, mask);

  mask = 0;
  defineCsr("scountovf",  Csrn::SCOUNTOVF,  !mand, !imp, 0, mask, 0xfffffff8);

  // Supervisor Protection and Translation 
  defineCsr("satp",       Csrn::SATP,       !mand, !imp, 0, wam, wam);

  // Supervisor time compare.
  auto stc = defineCsr("stimecmp",   Csrn::STIMECMP,   !mand, !imp, 0, wam, wam);
  stc->setHypervisor(true);
  if (rv32_)
    {
      auto csr = defineCsr("stimecmph",  Csrn::STIMECMPH,  !mand, !imp, 0, wam, wam);
      if (csr)
	csr->setHypervisor(true);
    }


  // Mark supervisor CSR that maps to virtual supervisor counterpart
  for (auto csrn : { Csrn::SSTATUS, Csrn::SIE, Csrn::STVEC,
		     Csrn::SSCRATCH, Csrn::SEPC,
		     Csrn::SCAUSE, Csrn::STVAL, Csrn::SIP,
		     Csrn::SATP, Csrn::STIMECMP })
    {
      auto csr = findCsr(csrn);
      if (csr)
	csr->setMapsToVirtual(true);
    }

  if (rv32_)
    {
      auto csr = findCsr(Csrn::STIMECMPH);
      if (csr)
	csr->setMapsToVirtual(true);
    }

  // add CSR fields
  addSupervisorFields();
}


template <typename URV>
void
CsRegs<URV>::defineUserRegs()
{
  bool mand = true;    // Mandatory.
  bool imp  = true;    // Implemented.
  URV  wam  = ~URV(0); // Write-all mask: all bits writeable.

  using Csrn = CsrNumber;

  // User Counter/Timers
  defineCsr("cycle",    Csrn::CYCLE,    !mand, imp,  0, wam, wam);
  defineCsr("time",     Csrn::TIME,     !mand, imp,  0, wam, wam);
  defineCsr("instret",  Csrn::INSTRET,  !mand, imp,  0, wam, wam);

  auto c = defineCsr("cycleh",   Csrn::CYCLEH,   !mand, !imp, 0, wam, wam);
  c->markAsHighHalf(true);
  c = defineCsr("timeh",    Csrn::TIMEH,    !mand, !imp, 0, wam, wam);
  c->markAsHighHalf(true);
  c = defineCsr("instreth", Csrn::INSTRETH, !mand, !imp, 0, wam, wam);
  c->markAsHighHalf(true);

  // Define hpmcounter3/hpmcounter3h to hpmcounter31/hpmcounter31h
  // as write-anything/read-zero (user can change that in the config
  // file).  Same for mhpmevent3/mhpmevent3h to mhpmevent3h/mhpmevent31h.
  for (unsigned i = 3; i <= 31; ++i)
    {
      CsrNumber csrNum = CsrNumber(unsigned(CsrNumber::HPMCOUNTER3) + i - 3);
      std::string name = "hpmcounter" + std::to_string(i);
      defineCsr(name, csrNum, !mand, !imp, 0, wam, wam);

      // High register counterpart of mhpmcounter.
      name += "h";
      csrNum = CsrNumber(unsigned(CsrNumber::HPMCOUNTER3H) + i - 3);
      c = defineCsr(std::move(name), csrNum, !mand, !imp, 0, wam, wam);
      c->markAsHighHalf(true);
    }

  // add CSR fields
  addUserFields();
}


template <typename URV>
void
CsRegs<URV>::defineHypervisorRegs()
{
  bool mand = true;    // Mandatory.
  bool imp  = true;    // Implemented.
  URV  wam  = ~URV(0); // Write-all mask: all bits writeable.

  using Csrn = CsrNumber;

  Csr<URV>* csr = nullptr;
  csr = defineCsr("hstatus",     Csrn::HSTATUS,     !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);

  using EC = ExceptionCause;
  URV zero = ((1 << unsigned(EC::S_ENV_CALL))               |
              (1 << unsigned(EC::VS_ENV_CALL))              |
              (1 << unsigned(EC::M_ENV_CALL))               |
              (1 << unsigned(EC::INST_GUEST_PAGE_FAULT))    |
              (1 << unsigned(EC::LOAD_GUEST_PAGE_FAULT))    |
              (1 << unsigned(EC::VIRT_INST))                |
              (1 << unsigned(EC::STORE_GUEST_PAGE_FAULT)))  ;

  URV mask = wam & ~zero;
  URV pokeMask = mask;
  csr = defineCsr("hedeleg",     Csrn::HEDELEG,     !mand, !imp, 0, mask, pokeMask);
  csr->setHypervisor(true);

  using IC = InterruptCause;
  zero = ((1 << unsigned(IC::S_SOFTWARE))  |
          (1 << unsigned(IC::S_TIMER))     |
          (1 << unsigned(IC::S_EXTERNAL))  |
          (1 << unsigned(IC::G_EXTERNAL))) ;
  mask = wam & ~zero;
  pokeMask = mask;
  csr = defineCsr("hideleg",     Csrn::HIDELEG,     !mand, !imp, 0, mask, pokeMask);
  csr->setHypervisor(true);

  mask = 0x1444;     // Bits SGEIP, VSEIP, VSTIP, and VSSIP writeable.
  pokeMask = mask;   // Same bits pokeable.
  csr = defineCsr("hie",         Csrn::HIE,         !mand, !imp, 0, mask, pokeMask);
  csr->setHypervisor(true);

  csr = defineCsr("hcounteren",  Csrn::HCOUNTEREN,  !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("hgeie",       Csrn::HGEIE,       !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("htval",       Csrn::HTVAL,       !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);

  mask = 0x4; // Bit VSSIP writeable
  pokeMask = 0x1444; // Bits SGEIP, VSEIP, VSTIP, and VSSIP pokeable.
  csr = defineCsr("hip",         Csrn::HIP,         !mand, !imp, 0, mask, pokeMask);
  csr->setHypervisor(true);

  mask = pokeMask = 0x444; // Bits VSEIP, VSTIP, and VSSIP.
  csr = defineCsr("hvip",        Csrn::HVIP,        !mand, !imp, 0, mask, pokeMask);
  csr->setHypervisor(true);

  csr = defineCsr("htinst",      Csrn::HTINST,      !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("hgeip",       Csrn::HGEIP,       !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("henvcfg",     Csrn::HENVCFG,     !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("henvcfgh",    Csrn::HENVCFGH,    !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true); csr->markAsHighHalf(true);
  csr = defineCsr("hgatp",       Csrn::HGATP,       !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("htimedelta",  Csrn::HTIMEDELTA,  !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("htimedeltah", Csrn::HTIMEDELTAH, !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true); csr->markAsHighHalf(true);

  // This may be already defined with trigger CSRs.
  if (not nameToNumber_.contains("hcontext"))
    csr = defineCsr("hcontext",    Csrn::HCONTEXT,    !mand, !imp, 0, wam, wam);
  else
    csr = findCsr(Csrn::HCONTEXT);
  csr->setHypervisor(true);

  csr = defineCsr("vsstatus",    Csrn::VSSTATUS,    !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);

  mask = pokeMask = 0x222;
  csr = defineCsr("vsie",        Csrn::VSIE,        !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);

  csr = defineCsr("vstvec",      Csrn::VSTVEC,      !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("vssratch",    Csrn::VSSCRATCH,   !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("vsepc",       Csrn::VSEPC,       !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("vscause",     Csrn::VSCAUSE,     !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("vstval",      Csrn::VSTVAL,      !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);

  mask = pokeMask = 0x222;
  csr = defineCsr("vsip",        Csrn::VSIP,        !mand, !imp, 0, mask, pokeMask);
  csr->setHypervisor(true);

  csr = defineCsr("vsatp",       Csrn::VSATP,       !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("vsstimecmp",  Csrn::VSTIMECMP,   !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  if (rv32_)
    {
      csr = defineCsr("vsstimecmph",  Csrn::VSTIMECMPH,   !mand, !imp, 0, wam, wam);
      if (csr)
	{
	  csr->setHypervisor(true);
	  csr->markAsHighHalf(true);
	}
    }
  

  // additional machine CSRs
  csr = defineCsr("mtval2",      Csrn::MTVAL2,      !mand, !imp, 0, wam, wam);
  csr = defineCsr("mtinst",      Csrn::MTINST,      !mand, !imp, 0, wam, wam);

  // In MIE/MIP bits corresponding to VSEIP/VSTIP/VSSIP are writeable pokeable.
  csr = findCsr(Csrn::MIP);
  if (csr)
    {
      csr->setWriteMask(csr->getWriteMask() | 0x444);
      csr->setPokeMask(csr->getPokeMask() | 0x444);
    }
  csr = findCsr(Csrn::MIE);
  if (csr)
    {
      csr->setWriteMask(csr->getWriteMask() | 0x444);
      csr->setPokeMask(csr->getPokeMask() | 0x444);
    }
}


template <typename URV>
void
CsRegs<URV>::defineDebugRegs()
{
  bool mand = true; // Mandatory.
  bool imp = true;  // Implemented.
  URV wam = ~URV(0);  // Write-all mask: all bits writeable.

  using Csrn = CsrNumber;

  // Debug/Trace registers.
  defineCsr("scontext", Csrn::SCONTEXT, !mand, !imp, 0, wam, wam);
  defineCsr("tselect",  Csrn::TSELECT,  !mand, imp,  0, wam, wam);
  defineCsr("tdata1",   Csrn::TDATA1,   !mand, imp,  0, wam, wam);
  defineCsr("tdata2",   Csrn::TDATA2,   !mand, imp,  0, wam, wam);
  defineCsr("tdata3",   Csrn::TDATA3,   !mand, !imp, 0, wam, wam);
  defineCsr("tinfo",    Csrn::TINFO,    !mand, !imp, 0, wam, wam);
  defineCsr("tcontrol", Csrn::TCONTROL, !mand, !imp, 0, wam, wam);
  defineCsr("mcontext", Csrn::MCONTEXT, !mand, !imp, 0, wam, wam);
  if (not nameToNumber_.contains("hcontext"))
    defineCsr("hcontext", Csrn::HCONTEXT, !mand, !imp, 0, wam, wam);

  // Define triggers.
  unsigned triggerCount = 4;
  triggers_ = Triggers<URV>(triggerCount);

  Data1Bits<URV> data1Mask(0), data1Val(0);

  // Set the masks of the read-write fields of data1 to all 1.
  data1Mask.mcontrol_.dmode_   = 1;
  data1Mask.mcontrol_.hit_     = 1;
  data1Mask.mcontrol_.select_  = 1;
  data1Mask.mcontrol_.action_  = 1; // Only least sig bit writeable
  data1Mask.mcontrol_.chain_   = 1;
  data1Mask.mcontrol_.match_   = 1; // Only least sig bit of match is writeable.
  data1Mask.mcontrol_.m_       = 1;
  data1Mask.mcontrol_.execute_ = 1;
  data1Mask.mcontrol_.store_   = 1;
  data1Mask.mcontrol_.load_    = 1;

  // Set intitial values of fields of data1.
  data1Val.mcontrol_.type_ = unsigned(TriggerType::AddrData);
  data1Val.mcontrol_.maskMax_ = rv32_ ? 31 : 63;

  // Values, write-masks, and poke-masks of the three components of
  // the triggres.
  URV val1(data1Val.value_), val2(0), val3(0);
  URV wm1(data1Mask.value_), wm2(~URV(0)), wm3(0);
  URV pm1(wm1), pm2(wm2), pm3(wm3);

  triggers_.config(0, val1, val2, val3, wm1, wm2, wm3, pm1, pm2, pm3);
  triggers_.config(1, val1, val2, val3, wm1, wm2, wm3, pm1, pm2, pm3);
  triggers_.config(2, val1, val2, val3, wm1, wm2, wm3, pm1, pm2, pm3);

  Data1Bits<URV> icountMask(0), icountVal(0);

  icountMask.icount_.dmode_  = 1;
  icountMask.icount_.count_  = (~0) & 0x3fff;
  icountMask.icount_.m_      = 1;
  icountMask.icount_.action_ = 0;
  icountMask.icount_.action_ = (~0) & 0x3f;

  icountVal.icount_.type_ = unsigned(TriggerType::InstCount);
  icountVal.icount_.count_ = 0;

  triggers_.config(3, icountVal.value_, 0, 0, icountMask.value_, 0, 0,
		   icountMask.value_, 0, 0);

  hasActiveTrigger_ = triggers_.hasActiveTrigger();
  hasActiveInstTrigger_ = triggers_.hasActiveInstTrigger();

  // Debug mode registers.
  URV dcsrVal = 0x40000003;
  URV dcsrMask = 0x00008e04;
  URV dcsrPokeMask = dcsrMask | 0x1cf; // Cause field modifiable
  bool isDebug = true;
  defineCsr("dcsr", Csrn::DCSR, !mand, imp, dcsrVal, dcsrMask,
	    dcsrPokeMask, isDebug);

  // Least sig bit of dpc is not writeable.
  URV dpcMask = ~URV(1);
  defineCsr("dpc", CsrNumber::DPC, !mand, imp, 0, dpcMask, dpcMask, isDebug);

  defineCsr("dscratch0", CsrNumber::DSCRATCH0, !mand, !imp, 0, wam, wam,
	    isDebug);
  defineCsr("dscratch1", CsrNumber::DSCRATCH1, !mand, !imp, 0, wam, wam,
	    isDebug);
}


template <typename URV>
void
CsRegs<URV>::defineVectorRegs()
{
  bool mand = true;  // Mndatory
  bool imp = true;   // Implemented

  defineCsr("vstart", CsrNumber::VSTART, !mand, !imp, 0, 0, 0);
  defineCsr("vxsat",  CsrNumber::VXSAT,  !mand, !imp, 0, 1, 1);  // 1 bit
  defineCsr("vxrm",   CsrNumber::VXRM,   !mand, !imp, 0, 3, 3);  // 2 bits
  defineCsr("vcsr",   CsrNumber::VCSR,   !mand, !imp, 0, 7, 7);  // 3 bits
  URV pokeMask = ~URV(0);
  defineCsr("vl",     CsrNumber::VL,     !mand, !imp, 0, 0, pokeMask);

  uint64_t mask = 0x800000ff;
  if (not rv32_)
    mask = 0x80000000000000ffL;
  defineCsr("vtype",  CsrNumber::VTYPE,  !mand, !imp, 0, mask, mask);

  defineCsr("vlenb",  CsrNumber::VLENB,  !mand, !imp, 0, 0, 0);

  // add CSR fields
  addVectorFields();
}


template <typename URV>
void
CsRegs<URV>::defineFpRegs()
{
  bool mand = true;  // Mndatory
  bool imp = true;   // Implemented

  // User Floating-Point CSRs
  URV wam = ~URV(0);  // Write-all mask: all bits writeable.
  defineCsr("fflags",   CsrNumber::FFLAGS,   !mand, !imp, 0, wam, wam);
  defineCsr("frm",      CsrNumber::FRM,      !mand, !imp, 0, wam, wam);
  defineCsr("fcsr",     CsrNumber::FCSR,     !mand, !imp, 0, 0xff, 0xff);

  // add FP fields
  addFpFields();
}


template <typename URV>
void
CsRegs<URV>::defineAiaRegs()
{
  bool mand = true;  // Mndatory
  bool imp = true;   // Implemented
  URV wam = ~URV(0);  // Write-all mask: all bits writeable.

  // Advanced interrupt archtecture CSRs
  defineCsr("miselect",   CsrNumber::MISELECT,   !mand, !imp, 0, wam, wam);
  defineCsr("mireg",      CsrNumber::MIREG,      !mand, !imp, 0, wam, wam);
  defineCsr("mtopei",     CsrNumber::MTOPEI,     !mand, !imp, 0, wam, wam);
  defineCsr("mtopi",      CsrNumber::MTOPI,      !mand, !imp, 0, wam, wam);
  defineCsr("mvien",      CsrNumber::MVIEN,      !mand, !imp, 0, wam, wam);
  defineCsr("mvip",       CsrNumber::MVIP,       !mand, !imp, 0, wam, wam);
  defineCsr("midelegh",   CsrNumber::MIDELEGH,   !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
  defineCsr("mieh",       CsrNumber::MIEH,       !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
  defineCsr("mvienh",     CsrNumber::MVIENH,     !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
  defineCsr("mviph",      CsrNumber::MVIPH,      !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
  defineCsr("miph",       CsrNumber::MIPH,       !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
  defineCsr("siselect",   CsrNumber::SISELECT,   !mand, !imp, 0, wam, wam);
  defineCsr("sireg",      CsrNumber::SIREG,      !mand, !imp, 0, wam, wam);
  defineCsr("stopei",     CsrNumber::STOPEI,     !mand, !imp, 0, wam, wam);
  defineCsr("stopi",      CsrNumber::STOPI,      !mand, !imp, 0, wam, wam);
  defineCsr("sieh",       CsrNumber::SIEH,       !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
  defineCsr("siph",       CsrNumber::SIPH,       !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
  defineCsr("hvien",      CsrNumber::HVIEN,      !mand, !imp, 0, wam, wam);
  defineCsr("hvictl",     CsrNumber::HVICTL,     !mand, !imp, 0, wam, wam);
  defineCsr("hviprio1",   CsrNumber::HVIPRIO1,   !mand, !imp, 0, wam, wam);
  defineCsr("hviprio2",   CsrNumber::HVIPRIO2,   !mand, !imp, 0, wam, wam);
  defineCsr("vsiselect",  CsrNumber::VSISELECT,  !mand, !imp, 0, wam, wam);
  defineCsr("vsireg",     CsrNumber::VSIREG,     !mand, !imp, 0, wam, wam);
  defineCsr("vstopei",    CsrNumber::VSTOPEI,    !mand, !imp, 0, wam, wam);
  defineCsr("vstopi",     CsrNumber::VSTOPI,     !mand, !imp, 0, wam, wam);
  defineCsr("hidelegh",   CsrNumber::HIDELEGH,   !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
  defineCsr("nhienh",     CsrNumber::NHIENH,     !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
  defineCsr("hviph",      CsrNumber::HVIPH,      !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
  defineCsr("hviprio1h",  CsrNumber::HVIPRIO1H,  !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
  defineCsr("hviprio3h",  CsrNumber::HVIPRIO3H,  !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
  defineCsr("vsieh",      CsrNumber::VSIEH,      !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
  defineCsr("vsiph",      CsrNumber::VSIPH,      !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
}


template <typename URV>
void
CsRegs<URV>::defineStateEnableRegs()
{
  bool mand = true;  // Mndatory
  bool imp = true;   // Implemented
  URV wam = ~URV(0);  // Write-all mask: all bits writeable.

  defineCsr("sstateen0", CsrNumber::SSTATEEN0,  !mand, !imp, 0, wam, wam);
  defineCsr("sstateen1", CsrNumber::SSTATEEN1,  !mand, !imp, 0, wam, wam);
  defineCsr("sstateen2", CsrNumber::SSTATEEN2,  !mand, !imp, 0, wam, wam);
  defineCsr("sstateen3", CsrNumber::SSTATEEN3,  !mand, !imp, 0, wam, wam);

  defineCsr("mstateen0", CsrNumber::MSTATEEN0,  !mand, !imp, 0, wam, wam);
  defineCsr("mstateen1", CsrNumber::MSTATEEN1,  !mand, !imp, 0, wam, wam);
  defineCsr("mstateen2", CsrNumber::MSTATEEN2,  !mand, !imp, 0, wam, wam);
  defineCsr("mstateen3", CsrNumber::MSTATEEN3,  !mand, !imp, 0, wam, wam);

  defineCsr("hstateen0", CsrNumber::HSTATEEN0,  !mand, !imp, 0, wam, wam);
  defineCsr("hstateen1", CsrNumber::HSTATEEN1,  !mand, !imp, 0, wam, wam);
  defineCsr("hstateen2", CsrNumber::HSTATEEN2,  !mand, !imp, 0, wam, wam);
  defineCsr("hstateen3", CsrNumber::HSTATEEN3,  !mand, !imp, 0, wam, wam);

  if (sizeof(URV) == 4)
    {
      defineCsr("sstateen0h", CsrNumber::MSTATEEN0H,  !mand, !imp, 0, wam, wam);
      defineCsr("sstateen1h", CsrNumber::MSTATEEN1H,  !mand, !imp, 0, wam, wam);
      defineCsr("sstateen2h", CsrNumber::MSTATEEN2H,  !mand, !imp, 0, wam, wam);
      defineCsr("sstateen3h", CsrNumber::MSTATEEN3H,  !mand, !imp, 0, wam, wam);

      defineCsr("hstateen0h", CsrNumber::HSTATEEN0H,  !mand, !imp, 0, wam, wam);
      defineCsr("hstateen1h", CsrNumber::HSTATEEN1H,  !mand, !imp, 0, wam, wam);
      defineCsr("hstateen2h", CsrNumber::HSTATEEN2H,  !mand, !imp, 0, wam, wam);
      defineCsr("hstateen3h", CsrNumber::HSTATEEN3H,  !mand, !imp, 0, wam, wam);
    }
}


template <typename URV>
bool
CsRegs<URV>::peek(CsrNumber number, URV& value) const
{
  auto csr = getImplementedCsr(number, virtMode_);
  if (not csr)
    return false;

  if (number >= CsrNumber::TDATA1 and number <= CsrNumber::TDATA3)
    return readTdata(number, PrivilegeMode::Machine, value);

  if (number == CsrNumber::FFLAGS or number == CsrNumber::FRM)
    {
      auto fcsr = getImplementedCsr(CsrNumber::FCSR);
      if (not fcsr)
        return false;
      value = fcsr->read();
      if (number == CsrNumber::FFLAGS)
        value = value & URV(FpFlags::FcsrMask);
      else
        value = (value & URV(RoundingMode::FcsrMask)) >> URV(RoundingMode::FcsrShift);
      return true;
    }

  // Value of SIP/SIE is masked by delegation register.
  if (number == CsrNumber::SIP or number == CsrNumber::SIE)
    {
      value = csr->read();
      auto deleg = getImplementedCsr(CsrNumber::MIDELEG);
      if (deleg)
	{
	  value &= deleg->read();
	  if (virtMode_)
	    {
	      auto hdeleg = getImplementedCsr(CsrNumber::HIDELEG);
	      if (hdeleg)
		value &= hdeleg->read();
	    }
	}
      return true;
    }

  // Value of VSIP/VSIE is masked by shifted HIDELG delegation register.
  if (number == CsrNumber::VSIP or number == CsrNumber::VSIE)
    {
      value = csr->read();
      auto hdeleg = getImplementedCsr(CsrNumber::HIDELEG);
      if (hdeleg)
	value &= hdeleg->read() >> 1;
      return true;
    }

  value = csr->read();

  if (number >= CsrNumber::PMPADDR0 and number <= CsrNumber::PMPADDR63)
    value = adjustPmpValue(number, value);

  return true;
}
  

template <typename URV>
bool
CsRegs<URV>::poke(CsrNumber num, URV value)
{
  using CN = CsrNumber;

  Csr<URV>* csr = getImplementedCsr(num, virtMode_);
  if (not csr)
    return false;

  if (isPmpaddrLocked(num))
    return true;  // Writing a locked PMPADDR register has no effect.

  if (num >= CN::TDATA1 and num <= CN::TDATA3)
    return pokeTdata(num, value);

  // Poke mask of SIP/SIE is combined with that of MIE/MIP.
  if (num == CN::SIP or num == CN::SIE)
    {
      // Get MIP/MIE
      auto mcsr = getImplementedCsr(CN(unsigned(num) + 0x200));
      if (mcsr)
        {
          URV prevMask = csr->getPokeMask();
	  URV tmpMask = prevMask & mcsr->getPokeMask();
          csr->setPokeMask(tmpMask);
          csr->poke(value);
          csr->setPokeMask(prevMask);
        }
      else
        csr->poke(value);
      return true;
    }

  if (num == CN::MISA)
    {
      value = legalizeMisa(csr, value);
      csr->pokeNoMask(value);
      return true;
    }

  if (num >= CN::PMPCFG0 and num <= CN::PMPCFG15)
    {
      URV prev = 0;
      peek(num, prev);
      value = legalizePmpcfgValue(prev, value);
    }
  else if (num == CN::MSTATUS or num == CN::SSTATUS or num == CN::VSSTATUS)
    {
      value &= csr->getPokeMask();
      value = legalizeMstatusValue(value);
    }

  csr->poke(value);

  if ((num >= CN::MHPMEVENT3 and num <= CN::MHPMEVENT31) or
      (num >= CN::MHPMEVENTH3 and num <= CN::MHPMEVENTH31))
    updateCounterControl(num);
  else if (num == CN::FFLAGS or num == CN::FRM or num == CN::FCSR)
    updateFcsrGroupForPoke(num, value);   // fflags and frm are parts of fcsr
  else if (num == CN::VXSAT or num == CN::VXRM or num == CN::VCSR)
    updateVcsrGroupForPoke(num, value); // fflags and frm are parts of fcsr

  // Cache interrupt enable.
  if (num == CN::MSTATUS)
    {
      MstatusFields<URV> fields(csr->read());
      interruptEnable_ = fields.bits_.MIE;
    }
  else if (num == CN::MCOUNTEREN or num == CN::SCOUNTEREN)
    updateCounterPrivilege();  // Reflect counter accessibility in user/supervisor.
  else
    hyperPoke(csr);    // Update hypervisor CSR aliased bits.

  return true;
}


template <typename URV>
bool
CsRegs<URV>::readTdata(CsrNumber number, PrivilegeMode mode, URV& value) const
{
  // Determine currently selected trigger.
  URV trigger = 0;
  if (not read(CsrNumber::TSELECT, mode, trigger))
    return false;

  if (number == CsrNumber::TDATA1)
    return triggers_.readData1(trigger, value);

  if (number == CsrNumber::TDATA2)
    return triggers_.readData2(trigger, value);

  if (number == CsrNumber::TDATA3)
    return triggers_.readData3(trigger, value);

  return false;
}


template <typename URV>
bool
CsRegs<URV>::writeTdata(CsrNumber number, PrivilegeMode mode, URV value)
{
  // Determine currently selected trigger.
  URV trigger = 0;
  if (not read(CsrNumber::TSELECT, mode, trigger))
    return false;

  // The CSR instructions never execute in debug mode.
  bool dMode = false;
  if (number == CsrNumber::TDATA1)
    {
      bool ok = triggers_.writeData1(trigger, dMode, value);
      if (ok) 
	{
	  // TDATA1 modified, update cached values
	  hasActiveTrigger_ = triggers_.hasActiveTrigger();
	  hasActiveInstTrigger_ = triggers_.hasActiveInstTrigger();
	}
      return ok;
    }

  if (number == CsrNumber::TDATA2)
    return triggers_.writeData2(trigger, dMode, value);

  if (number == CsrNumber::TDATA3)
    return triggers_.writeData3(trigger, dMode, value);

  return false;
}


template <typename URV>
bool
CsRegs<URV>::pokeTdata(CsrNumber number, URV value)
{
  // Determine currently selected trigger.
  URV trigger = 0;
  if (not read(CsrNumber::TSELECT, PrivilegeMode::Machine, trigger))
    return false;

  if (number == CsrNumber::TDATA1)
    {
      bool ok = triggers_.pokeData1(trigger, value);
      if (ok) 
	{
	  // TDATA1 modified, update cached values
	  hasActiveTrigger_ = triggers_.hasActiveTrigger();
	  hasActiveInstTrigger_ = triggers_.hasActiveInstTrigger();
	}
      return ok;
    }

  if (number == CsrNumber::TDATA2)
    return triggers_.pokeData2(trigger,value);

  if (number == CsrNumber::TDATA3)
    return triggers_.pokeData3(trigger, value);

  return false;
}


template <typename URV>
unsigned
CsRegs<URV>::getPmpConfigByteFromPmpAddr(CsrNumber csrn) const
{
  if (csrn < CsrNumber::PMPADDR0 or csrn > CsrNumber::PMPADDR63)
    return 0;

  unsigned pmpIx = unsigned(csrn) - unsigned(CsrNumber::PMPADDR0);

  // Determine rank of config register corresponding to pmpIx.
  unsigned cfgOffset = pmpIx / 4; // 0, 1, 2, ... or 15.

  // Identify byte within config register.
  unsigned byteIx = pmpIx % 4;

  if (not rv32_)
    {
      cfgOffset = (cfgOffset / 2) * 2;  // 0, 2, 4, ... or 14
      byteIx = pmpIx % 8;
    }

  CsrNumber cfgNum = CsrNumber(unsigned(CsrNumber::PMPCFG0) + cfgOffset);

  URV val = 0;
  if (peek(cfgNum, val))
    return (val >> 8*byteIx) & 0xff;

  return 0;
}


template <typename URV>
URV
CsRegs<URV>::adjustPmpValue(CsrNumber csrn, URV value) const
{
  if (csrn < CsrNumber::PMPADDR0 or csrn > CsrNumber::PMPADDR63)
    return value;   // Not a PMPADDR CSR.

  if (pmpG_ == 0)
    return value;

  unsigned byte = getPmpConfigByteFromPmpAddr(csrn);

  unsigned aField =(byte >> 3) & 3;
  if (aField < 2)
    {
      // A field is OFF or TOR
      if (pmpG_ >= 1)
        value = (value >> pmpG_) << pmpG_; // Clear least sig G bits.
    }
  else
    {
      // A field is NAPOT
      if (pmpG_ >= 2)
        {
          unsigned width = rv32_ ? 32 : 64;
          URV mask = ~URV(0) >> (width - pmpG_ + 1);
          value = value | mask; // Set to 1 least sig G-1 bits
        }
    }

  return value;
}


template <typename URV>
URV
CsRegs<URV>::legalizePmpcfgValue(URV current, URV value) const
{
  URV legal = 0;
  for (unsigned i = 0; i < sizeof(value); ++i)
    {
      uint8_t cb = (current >> (i*8)) & 0xff;  // Current byte.
      uint8_t nb = (value >> (i*8)) & 0xff;    // New byte.

      if (cb >> 7)
        nb = cb; // Field is locked. Use byte from current value.
      else if (pmpG_ != 0)
        {
          // If G is >= 1 then NA4 is not selectable in the A field of
          // the new byte.
          unsigned aField = (nb >> 3) & 3;
          if (aField == 2)
            {
              aField = 3;
              nb = nb | (aField << 3); // Change A field in new byte to 3.
            }
        }

      // w=1 r=0 is not allowed, change to w=0 r=0 
      if ((nb & 3) == 2)
        nb = (nb >> 2) << 2;

      legal = legal | (URV(nb) << i*8);
    }

  return legal;
}  


template <typename URV>
bool
CsRegs<URV>::isPmpaddrLocked(CsrNumber csrn) const
{
  if (csrn < CsrNumber::PMPADDR0 or csrn > CsrNumber::PMPADDR63)
    return false;   // Not a PMPADDR CSR.

  unsigned byte = getPmpConfigByteFromPmpAddr(csrn);
  bool locked = byte & 0x80;
  if (locked)
    return true;

  // If the next PMPADDR is top-of-range and is locked, then the
  // current PMADDR is considered to be locked.
  if (csrn >= CsrNumber::PMPADDR63)
    return false;  // No next PMPADDR register.

  CsrNumber csrn2 = CsrNumber(unsigned(csrn) + 1);
  byte = getPmpConfigByteFromPmpAddr(csrn2);
  locked = byte & 0x80;
  bool tor = ((byte >> 3) & 3) == 1;
  return locked and tor;
}


template <typename URV>
void
CsRegs<URV>::updateCounterPrivilege()
{
  URV mMask = 0;
  if (not peek(CsrNumber::MCOUNTEREN, mMask))
    return;

  URV sMask = 0;
  peek(CsrNumber::SCOUNTEREN, sMask);

  // Bits 0, 1, 2, 3 to 31 of mask correspond to CYCLE, TIME, INSTRET,
  // HPMCOUNTER3 to HPMCOUNTER31
  for (unsigned i = 0; i < 32; ++i)
    {
      bool mFlag = (mMask >> i) & 1;
      PrivilegeMode nextMode = PrivilegeMode::Machine;

      if (mFlag)
        {
          if (superEnabled_)
            {
              nextMode = PrivilegeMode::Supervisor;

              bool sFlag = (sMask >> i) & 1;
              if (sFlag and userEnabled_)
                nextMode = PrivilegeMode::User;
            }
          else if (userEnabled_)
            nextMode = PrivilegeMode::User;
        }

      unsigned num = i + unsigned(CsrNumber::CYCLE);

      CsrNumber csrn = CsrNumber(num);
      auto csr = getImplementedCsr(csrn);
      if (csr)
        csr->setPrivilegeMode(nextMode);

      num = i + unsigned(CsrNumber::CYCLEH);
      csrn = CsrNumber(num);
      csr = getImplementedCsr(csrn);
      if (csr)
        csr->setPrivilegeMode(nextMode);
    }

  auto stimecmp = findCsr(CsrNumber::STIMECMP);
  auto stimecmph = findCsr(CsrNumber::STIMECMP);
  for (auto csr : {stimecmp, stimecmph})
    {
      if ((mMask & 2) == 0)  // TM bit set in mcounteren.
	csr->setPrivilegeMode(PrivilegeMode::Machine);
      else if (superEnabled_)
	csr->setPrivilegeMode(PrivilegeMode::Supervisor);
    }
}



template <typename URV>
void
CsRegs<URV>::updateCounterControl(CsrNumber csrn)
{
  unsigned counterIx = 0;
  if (not getIndexOfMhpmevent(csrn, counterIx))
    {
      assert(0);
      return;
    }

  // This gets 64-bit value (MHPMEVEN and MHPMEVENTH in rv32).
  uint64_t value = 0;
  if (not getMhpmeventValue(counterIx, value))
    {
      assert(0);
      return;
    }

  uint32_t mask = ~uint32_t(0);  // All privilege modes enabled.
  URV event = (value << 8 >> 8);  // Drop top 8 bits of 64-bit value.

  if (hasPerfEventSet_)
    {
      if (not perfEventSet_.contains(event))
        event = 0;
    }
  else
    event = std::min(event, maxEventId_);

  if (cofEnabled_)
    {
      MhpmeventFields fields(value);
      event = fields.bits_.EVENT;
      if (fields.bits_.MINH)
	mask &= ~ mPerfRegs_.privModeToMask(PrivilegeMode::Machine, false);
      if (fields.bits_.SINH)
	mask &= ~ mPerfRegs_.privModeToMask(PrivilegeMode::Supervisor, false);
      if (fields.bits_.UINH)
	mask &= ~ mPerfRegs_.privModeToMask(PrivilegeMode::User, false);
      if (fields.bits_.VSINH)
	mask &= ~ mPerfRegs_.privModeToMask(PrivilegeMode::Supervisor, true);
      if (fields.bits_.VUINH)
	mask &= ~ mPerfRegs_.privModeToMask(PrivilegeMode::User, true);
    }

  assignEventToCounter(event, counterIx, mask);
}


template <typename URV>
void
CsRegs<URV>::addMachineFields()
{
  constexpr unsigned xlen = sizeof(URV)*8;
  setCsrFields(CsrNumber::MVENDORID, {{"OFFSET", 7}, {"BANK", 25}});
  setCsrFields(CsrNumber::MARCHID, {{"marchid", xlen}});
  setCsrFields(CsrNumber::MIMPID, {{"mimpid", xlen}});
  setCsrFields(CsrNumber::MHARTID, {{"mhartid", xlen}});
  setCsrFields(CsrNumber::MCONFIGPTR, {{"mconfigptr", xlen}});
  setCsrFields(CsrNumber::MISA, {{"EXT", 26}, {"zero", xlen - 28}, {"MXL", 2}});
  // TODO: field for each trap?
  setCsrFields(CsrNumber::MEDELEG, {{"medeleg", xlen}});
  setCsrFields(CsrNumber::MIDELEG, {{"mideleg", xlen}});
  setCsrFields(CsrNumber::MIE,
    {{"zero", 1}, {"SSIE", 1}, {"zero", 1}, {"MSIE", 1},
     {"zero", 1}, {"STIE", 1}, {"zero", 1}, {"MTIE", 1},
     {"zero", 1}, {"SEIE", 1}, {"zero", 1}, {"MEIE", 1},
     {"zero", xlen - 12}});
  setCsrFields(CsrNumber::MIP,
    {{"zero", 1}, {"SSIP", 1}, {"zero", 1}, {"MSIP", 1},
     {"zero", 1}, {"STIP", 1}, {"zero", 1}, {"MTIP", 1},
     {"zero", 1}, {"SEIP", 1}, {"zero", 1}, {"MEIP", 1},
     {"zero", xlen - 12}});
  setCsrFields(CsrNumber::MTVEC, {{"MODE", 2}, {"BASE", xlen - 2}});

  std::vector<typename Csr<URV>::Field> mcount = {{"CY", 1}, {"TM", 1}, {"IR", 1}};
  std::vector<typename Csr<URV>::Field> hpm;
  for (unsigned i = 3; i <= 31; ++i)
    hpm.push_back({"HPM" + std::to_string(i), 1});
  mcount.insert(mcount.end(), hpm.begin(), hpm.end());
  setCsrFields(CsrNumber::MCOUNTEREN, mcount);
  mcount.at(1) = {"zero", 1}; // TM cleared for MCOUNTINHIBIT
  setCsrFields(CsrNumber::MCOUNTINHIBIT, mcount);
  setCsrFields(CsrNumber::MSCRATCH, {{"mscratch", xlen}});
  setCsrFields(CsrNumber::MEPC, {{"mepc", xlen}});
  setCsrFields(CsrNumber::MCAUSE, {{"CODE", xlen - 1}, {"INT", 1}});
  setCsrFields(CsrNumber::MTVAL, {{"mtval", xlen}});
  setCsrFields(CsrNumber::MCYCLE, {{"mcycle", xlen}});
  setCsrFields(CsrNumber::MINSTRET, {{"minstret", xlen}});
  // TODO: add GVA/SBE fields
  if (rv32_)
    {
      setCsrFields(CsrNumber::MSTATUS,
        {{"UIE",  1}, {"SIE",  1}, {"res1", 1}, {"MIE",   1},
         {"UPIE", 1}, {"SPIE", 1}, {"UBE",  1}, {"MPIE",  1},
         {"SPP",  1}, {"VS",   2}, {"MPP",  2}, {"FS",    2},
         {"XS",   2}, {"MPRV", 1}, {"SUM",  1}, {"MXR",   1},
         {"TVM",  1}, {"TW",   1}, {"TSR",  1}, {"res0",  8},
         {"SD",   1}});
      setCsrFields(CsrNumber::MSTATUSH,
        {{"res1", 4}, {"SBE", 1}, {"MBE", 1}, {"res0", 26}});
      setCsrFields(CsrNumber::MENVCFG,
        {{"FIOM", 1}, {"res0",  3}, {"CBIE", 2}, {"CBCFE", 1},
         {"CBZE", 1}, {"res1", 24}});
      setCsrFields(CsrNumber::MENVCFGH,
        {{"res0", 30}, {"PBMTE", 1}, {"STCE", 1}});
      setCsrFields(CsrNumber::MCYCLEH, {{"mcycleh", 32}});
      setCsrFields(CsrNumber::MINSTRETH, {{"minstreth", 32}});
    }
  else
    {
      setCsrFields(CsrNumber::MSTATUS,
        {{"UIE",  1},  {"SIE",  1}, {"res2", 1}, {"MIE",   1},
         {"UPIE", 1},  {"SPIE", 1}, {"UBE",  1}, {"MPIE",  1},
         {"SPP",  1},  {"VS",   2}, {"MPP",  2}, {"FS",    2},
         {"XS",   2},  {"MPRV", 1}, {"SUM",  1}, {"MXR",   1},
         {"TVM",  1},  {"TW",   1}, {"TSR",  1}, {"res1",  9},
         {"UXL",  2},  {"SXL",  2}, {"SBE",  1}, {"MBE",   1},
         {"res0", 25}, {"SD",   1}});
      setCsrFields(CsrNumber::MENVCFG,
        {{"FIOM", 1}, {"res0",  3}, {"CBIE",  2}, {"CBCFE", 1},
         {"CBZE", 1}, {"res1", 54}, {"PBMTE", 1}, {"STCE",  1}});
    }

  unsigned pmpIx = 0;
  for (unsigned i = 0; i < 16; i += 2)
    {
      std::vector<typename Csr<URV>::Field> pmps;

      if (rv32_)
        {
          CsrNumber csrNum = CsrNumber(unsigned(CsrNumber::PMPCFG0) + i + 1);
          unsigned end = pmpIx + 4;
          for (; pmpIx < end; pmpIx++)
            {
              std::string name = "pmp" + std::to_string(pmpIx) + "cfg";
              pmps.push_back({name + "R", 1});
              pmps.push_back({name + "W", 1});
              pmps.push_back({name + "X", 1});
              pmps.push_back({name + "A", 2});
              pmps.push_back({name + "zero", 2});
              pmps.push_back({name + "L", 1});
            }
          setCsrFields(csrNum, pmps);
        }
      else
        {
          CsrNumber csrNum = CsrNumber(unsigned(CsrNumber::PMPCFG0) + i);
          unsigned end = pmpIx + 8;
          for (; pmpIx < end; pmpIx++)
            {
              std::string name = "pmp" + std::to_string(pmpIx) + "cfg";
              pmps.push_back({name + "R", 1});
              pmps.push_back({name + "W", 1});
              pmps.push_back({name + "X", 1});
              pmps.push_back({name + "A", 2});
              pmps.push_back({name + "zero", 2});
              pmps.push_back({name + "L", 1});
            }

          setCsrFields(csrNum, pmps);
        }
    }
  for (unsigned i = 0; i < 64; ++i)
    {
      CsrNumber csrNum = CsrNumber(unsigned(CsrNumber::PMPADDR0) + i);
      if (rv32_)
        setCsrFields(csrNum, {{"addr", 32}});
      else
        setCsrFields(csrNum, {{"addr", 54}, {"zero", 10}});
    }

  for (unsigned i = 3; i <= 31; ++i)
    {
      CsrNumber csrNum = CsrNumber(unsigned(CsrNumber::MHPMCOUNTER3) + i - 3);
      std::string name = "mhpmcounter" + std::to_string(i);
      setCsrFields(csrNum, {{name, xlen}});
      if (rv32_)
        {
          // High register counterpart of mhpmcounter.
          csrNum = CsrNumber(unsigned(CsrNumber::MHPMCOUNTER3H) + i - 3);
          name += "h";
          setCsrFields(csrNum, {{name, xlen}});
        }
    }
}


template <typename URV>
void
CsRegs<URV>::addSupervisorFields()
{
  constexpr unsigned xlen = sizeof(URV)*8;
  setCsrFields(CsrNumber::STVEC, {{"MODE", 2}, {"BASE", xlen - 2}});

  std::vector<typename Csr<URV>::Field> scount = {{"CY", 1}, {"TM", 1}, {"IR", 1}};
  std::vector<typename Csr<URV>::Field> hpm;
  for (unsigned i = 3; i <= 31; ++i)
    hpm.push_back({"HPM" + std::to_string(i), 1});
  scount.insert(scount.end(), hpm.begin(), hpm.end());
  setCsrFields(CsrNumber::SCOUNTEREN, scount);

  setCsrFields(CsrNumber::SSCRATCH, {{"sscratch", xlen}});
  setCsrFields(CsrNumber::SEPC, {{"sepc", xlen}});
  setCsrFields(CsrNumber::SCAUSE, {{"CODE", xlen - 1}, {"INT", 1}});
  setCsrFields(CsrNumber::STVAL, {{"stval", xlen}});
  setCsrFields(CsrNumber::SIE,
    {{"zero", 1}, {"SSIE", 1}, {"zero", 3}, {"STIE", 1},
     {"zero", 3}, {"SEIE", 1}, {"zero", xlen - 10}});
  setCsrFields(CsrNumber::SIP,
    {{"zero", 1}, {"SSIP", 1}, {"zero", 3}, {"STIP", 1},
     {"zero", 3}, {"SEIP", 1}, {"zero", xlen - 10}});
  setCsrFields(CsrNumber::SENVCFG,
    {{"FIOM", 1}, {"res0", 3}, {"CBIE", 2}, {"CBCFE", 1},
     {"CBZE", 1}, {"res1", xlen - 8}});

  if (rv32_)
    {
      setCsrFields(CsrNumber::SSTATUS,
        {{"res0", 1}, {"SIE",  1}, {"res1",  3}, {"SPIE", 1},
         {"UBE",  1}, {"res2", 1}, {"SPP",   1}, {"VS",   2},
         {"res3", 2}, {"FS",   2}, {"XS",    2}, {"res4", 1},
         {"SUM",  1}, {"MXR",  1}, {"res5", 11}, {"SD",   1}});
      setCsrFields(CsrNumber::SATP,
        {{"PPN", 22}, {"ASID", 9}, {"MODE", 1}});
    }
  else
    {
      setCsrFields(CsrNumber::SSTATUS,
        {{"res0",  1}, {"SIE",  1}, {"res1",  3}, {"SPIE",  1},
         {"UBE",   1}, {"res2", 1}, {"SPP",   1}, {"VS",    2},
         {"res3",  2}, {"FS",   2}, {"XS",    2}, {"res4",  1},
         {"SUM",   1}, {"MXR",  1}, {"res5", 12}, {"UXL",   2},
         {"res6", 29}, {"SD",   1}});
      setCsrFields(CsrNumber::SATP,
        {{"PPN", 44}, {"ASID", 16}, {"MODE", 4}});
    }
}


template <typename URV>
void
CsRegs<URV>::addUserFields()
{
  constexpr unsigned xlen = sizeof(URV)*8;
  setCsrFields(CsrNumber::CYCLE, {{"cycle", xlen}});
  setCsrFields(CsrNumber::TIME, {{"time", xlen}});
  setCsrFields(CsrNumber::INSTRET, {{"instret", xlen}});
  if (rv32_)
    {
      setCsrFields(CsrNumber::CYCLEH, {{"cycleh", xlen}});
      setCsrFields(CsrNumber::TIMEH, {{"timeh", xlen}});
      setCsrFields(CsrNumber::INSTRETH, {{"instreth", xlen}});
    }

  for (unsigned i = 3; i <= 31; ++i)
    {
      CsrNumber csrNum = CsrNumber(unsigned(CsrNumber::HPMCOUNTER3) + i - 3);
      std::string name = "hpmcounter" + std::to_string(i);
      setCsrFields(csrNum, {{name, xlen}});
      if (rv32_)
        {
          // High register counterpart of hpmcounter.
          csrNum = CsrNumber(unsigned(CsrNumber::HPMCOUNTER3H) + i - 3);
          name += "h";
          setCsrFields(csrNum, {{name, xlen}});
        }
    }
}


template <typename URV>
void
CsRegs<URV>::addVectorFields()
{
  constexpr unsigned xlen = sizeof(URV)*8;
  setCsrFields(CsrNumber::VSTART, {{"vstart", xlen}});
  setCsrFields(CsrNumber::VXSAT, {{"vxsat", 1}, {"zero", xlen - 1}});
  setCsrFields(CsrNumber::VXRM, {{"vxrm", 2}, {"zero", xlen - 2}});
  setCsrFields(CsrNumber::VCSR,
    {{"vxsat", 1}, {"vxrm", 2}, {"zero", xlen - 3}});
  setCsrFields(CsrNumber::VL, {{"vl", xlen}});
  setCsrFields(CsrNumber::VTYPE,
    {{"LMUL",       3}, {"SEW",   3}, {"VTA", 1}, {"VMA", 1},
     {"res", xlen - 9}, {"ILL", 1}});
  setCsrFields(CsrNumber::VLENB, {{"vlenb", xlen}});
}


template <typename URV>
void
CsRegs<URV>::addFpFields()
{
  setCsrFields(CsrNumber::FFLAGS,
    {{"NX", 1}, {"UF", 1}, {"OF", 1}, {"DZ", 1}, {"NV", 1}});
  setCsrFields(CsrNumber::FRM, {{"frm", 3}});
  setCsrFields(CsrNumber::FCSR, {{"fflags", 5}, {"frm", 3}, {"res0", 24}});
}


template <typename URV>
void
CsRegs<URV>::hyperWrite(Csr<URV>* csr)
{
  auto num = csr->getNumber();
  auto value = csr->read();

  auto hip = getImplementedCsr(CsrNumber::HIP);
  auto hvip = getImplementedCsr(CsrNumber::HVIP);
  auto mip = getImplementedCsr(CsrNumber::MIP);

  bool hipUpdated = num == CsrNumber::HIP;
  URV vsMask = 0x444;  // VSEIP, VSTIP and VSSIP.

  if (num == CsrNumber::MIP)
    {
      // Updating MIP is reflected into HIP.
      if (hip)
	{
	  URV newVal = (mip->read() & vsMask) | (hip->read() & ~vsMask);
	  hip->poke(newVal);
	  hipUpdated = true;
	  recordWrite(CsrNumber::HIP);
	}
    }
  else if (num == CsrNumber::HIP)
    {
      // Updating HIP is reflected into MIP.
      if (mip)
	{
	  URV newVal = (mip->read() & vsMask) | (hip->read() & ~vsMask);
	  mip->poke(newVal);
	  recordWrite(CsrNumber::MIP);
	}
    }
  else if (num == CsrNumber::HVIP)
    {
      // Poking HVIP injects values into HIP. FIX : Need to
      // logical-or external values for VSEIP and VSTIP.
      if (hip)
	{
	  hip->poke(value);
	  hipUpdated = true;
	  recordWrite(CsrNumber::HIP);
	}
    }

  if (hipUpdated)
    {
      // Writing HIP changes bit VSSIP in HVIP.
      if (hvip  and num != CsrNumber::HVIP)
	{
	  URV mask = 0x400; // Bit VSSIP
	  URV newVal = (value & mask) | (hvip->read() & ~mask);
	  hvip->poke(newVal);
	  recordWrite(CsrNumber::HVIP);
	}

      // Updating HIP is reflected in VSIP and MIP.
      auto vsip = getImplementedCsr(CsrNumber::VSIP);
      if (vsip)
	{
	  URV val = hip->read() & ~ URV(0x1000);  // Clear bit 12 (SGEIP)
	  vsip->poke(val >> 1);
	  recordWrite(CsrNumber::VSIP);
	}

      // Updating HIP is reflected in MIP.
      if (mip and num != CsrNumber::MIP)
	{
	  URV newVal = (mip->read() & ~vsMask) | (hip->read() & vsMask);
	  mip->poke(newVal);
	  recordWrite(CsrNumber::MIP);
	}
    }

  auto hie = getImplementedCsr(CsrNumber::HIE);
  auto mie = getImplementedCsr(CsrNumber::MIE);
  if (num == CsrNumber::HIE)
    {
      if (mie)
	{
	  mie->poke((mie->read() & ~vsMask) | (hie->read() & vsMask));
	  recordWrite(CsrNumber::MIP);
	}
    }
  else if (num == CsrNumber::MIE)
    {
      if (hie)
	{
	  hie->poke((hie->read() & ~vsMask) | (mie->read() & vsMask));
	  recordWrite(CsrNumber::MIP);
	}
    }
}


template <typename URV>
void
CsRegs<URV>::hyperPoke(Csr<URV>* csr)
{
  auto num = csr->getNumber();
  auto value = csr->read();

  auto hip = getImplementedCsr(CsrNumber::HIP);
  auto hvip = getImplementedCsr(CsrNumber::HVIP);
  auto mip = getImplementedCsr(CsrNumber::MIP);

  bool hipUpdated = num == CsrNumber::HIP;
  URV vsMask = 0x444;  // VSEIP, VSTIP and VSSIP.

  if (num == CsrNumber::MIP)
    {
      // Updating MIP is reflected into HIP.
      if (hip)
	{
	  URV newVal = (mip->read() & vsMask) | (hip->read() & ~vsMask);
	  hip->poke(newVal);
	  hipUpdated = true;
	}
    }
  else if (num == CsrNumber::HIP)
    {
      // Updating HIP is reflected into MIP.
      if (mip)
	{
	  URV newVal = (mip->read() & vsMask) | (hip->read() & ~vsMask);
	  mip->poke(newVal);
	}
    }
  else if (num == CsrNumber::HVIP)
    {
      // Poking HVIP injects values into HIP. FIX : Need to
      // logical-or external values for VSEIP and VSTIP.
      if (hip)
	{
	  hip->poke(value);
	  hipUpdated = true;
	}
    }

  if (hipUpdated)
    {
      // Writing HIP changes bit VSSIP in HVIP.
      if (hvip  and num != CsrNumber::HVIP)
	{
	  URV mask = 0x400; // Bit VSSIP
	  URV newVal = (value & mask) | (hvip->read() & ~mask);
	  hvip->poke(newVal);
	}

      // Updating HIP is reflected in VSIP and MIP.
      auto vsip = getImplementedCsr(CsrNumber::VSIP);
      if (vsip)
	{
	  URV val = hip->read() & ~ URV(0x1000);  // Clear bit 12 (SGEIP)
	  vsip->poke(val >> 1);
	}

      // Updating HIP is reflected in MIP.
      if (mip and num != CsrNumber::MIP)
	{
	  URV newVal = (mip->read() & ~vsMask) | (hip->read() & vsMask);
	  mip->poke(newVal);
	}
    }

  auto hie = getImplementedCsr(CsrNumber::HIE);
  auto mie = getImplementedCsr(CsrNumber::MIE);
  if (num == CsrNumber::HIE)
    {
      if (mie)
	{
	  mie->poke((mie->read() & ~vsMask) | (hie->read() & vsMask));
	}
    }
  else if (num == CsrNumber::MIE)
    {
      if (hie)
	{
	  hie->poke((hie->read() & ~vsMask) | (mie->read() & vsMask));
	}
    }
}


template class WdRiscv::CsRegs<uint32_t>;
template class WdRiscv::CsRegs<uint64_t>;
