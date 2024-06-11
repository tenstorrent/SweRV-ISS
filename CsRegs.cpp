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
  defineEntropyReg();
  definePmaRegs();
  defineSteeRegs();
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
		       URV pokeMask, bool quiet)
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

  csr.config(name, csrn, mandatory, implemented, resetValue, writeMask, pokeMask);

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
  num = advance(num, 0x100);   // Get VCSR corresponding to CSR.
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
  num = advance(num, 0x100);   // Get VCSR corresponding to CSR.
  return getImplementedCsr(num);
}


template <typename URV>
bool
CsRegs<URV>::readSip(URV& value) const
{
  value = 0;
  auto sip = getImplementedCsr(CsrNumber::SIP);
  if (not sip)
    return false;
  value = sip->read();

  // Read value of MIP/SIP is masked by MIDELG.
  auto deleg = getImplementedCsr(CsrNumber::MIDELEG);
  if (deleg)
    value &= deleg->read();

  // Where mideleg is 0 and mvien is 1, sip becomes an alias to mvip.
  auto mvien = getImplementedCsr(CsrNumber::MVIEN);
  auto mvip = getImplementedCsr(CsrNumber::MVIP);
  if (deleg and mvien and mvip)
    {
      URV mask = mvien->read() & ~deleg->read();
      value = (value & ~mask) | (mvip->read() & mask);
    }

  // Bits SGEIP, VSEIP, VSTIP, VSSIP are read-only zero in SIE/SIP.
  value &= ~ URV(0x1444);

  return true;
}


template <typename URV>
bool
CsRegs<URV>::readSie(URV& value) const
{
  value = 0;
  auto sie = getImplementedCsr(CsrNumber::SIE);
  if (not sie)
    return false;
  value = sie->read();

  // Read value of MIP/SIP is masked by MIDELG.
  auto deleg = getImplementedCsr(CsrNumber::MIDELEG);
  if (deleg)
    value &= deleg->read();

  // Where mideleg is 0 and mvien is 1, sie becomes a writable.
  auto mvien = getImplementedCsr(CsrNumber::MVIEN);
  auto mvip = getImplementedCsr(CsrNumber::MVIP);
  if (deleg and mvien and mvip)
    {
      URV mask = mvien->read() & ~deleg->read();
      value = (value & ~mask) | (shadowSie_ & mask);
    }

  // Bits SGEIP, VSEIP, VSTIP, VSSIP are read-only zero in SIE/SIP.
  value &= ~ URV(0x1444);

  return true;
}


template <typename URV>
bool
CsRegs<URV>::readMvip(URV& value) const
{
  value = 0;
  auto mvip = getImplementedCsr(CsrNumber::MVIP);
  if (not mvip)
    return false;
  value = mvip->read();

  // Bit 1/9 of MVIP is an alias to bit 1/9 in MIP if bit 1/9 is not set in MVIEN.
  auto mvien = getImplementedCsr(CsrNumber::MVIEN);
  auto mip = getImplementedCsr(CsrNumber::MIP);
  if (mvien and mip)
    {
      URV mask = mvien->read();
      mask &= URV(0x202);
      mask ^= URV(0x202);
      value = (value & ~mask) | (mip->read() & mask);
    }

  // Bit STIE (5) of MVIP is an alias to bit 5 of MIP if bit 5 of MIP is writable.
  // Othrwise, it is zero.
  if (mip)
    {
      URV mask = URV(0x20);  // Bit 5
      if ((mip->getWriteMask() & mask) != 0)   // Bit 5 writable in mip
	value = (value & ~mask) | (mip->read() & mask);
    }

  return true;
}


template <typename URV>
bool
CsRegs<URV>::writeMvip(URV value)
{
  auto mvip = getImplementedCsr(CsrNumber::MVIP);
  if (not mvip)
    return false;

  auto mvien = getImplementedCsr(CsrNumber::MVIEN);
  auto mip = getImplementedCsr(CsrNumber::MIP);
  if (mvien and mip)
    {
      // Bit 1/9 of MVIP is an alias to bit 1/9 in MIP if bit 1/9 is not set in MVIEN.
      URV mask = mvien->read();
      URV b19 = URV(0x202);
      mask ^= b19;

      // Bit STIE (5) of MVIP is an alias to bit 5 of MIP if bit 5 of MIP is writable.
      // Othrwise, it is zero.
      URV b5 = URV(0x20);  // Bit 5 mask
      if ((mip->getWriteMask() & b5) != 0)   // Bit 5 writable in mip
	mask |= b5;

      mask &= b19 | b5;
      mip->write((mip->read() & ~mask) | (value & mask));
      mvip->write((mvip->read() & mask) | (value & ~mask));
    }
  else
    mvip->write(value);

  recordWrite(CsrNumber::MVIP);
  return true;
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

      // If a bit is zero in MSTATEEN, it becomes zero in SSTATEEN
      CsrNumber mnum = advance(CN::MSTATEEN0, ix);
      auto mcsr = getImplementedCsr(mnum);
      if (mcsr)
	  value &= mcsr->read();

      // If a bit is zero in HSTATEEN, it becomes zero in SSTATEEN
      if (virtMode_)
	{
	  CsrNumber hnum = advance(CN::HSTATEEN0, ix);
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

      // If a bit is zero in MSTATEEN, it becomes zero in HSTATEEN
      CN mnum = CN::MSTATEEN0;
      if (num >= CN::HSTATEEN0H and num <= CN::HSTATEEN3H)
        mnum = CN::MSTATEEN0H;
      mnum = advance(mnum, ix);
      auto mcsr = getImplementedCsr(mnum);
      if (mcsr)
	value &= mcsr->read();
    }
  return value;
}


template <typename URV>
URV
CsRegs<URV>::adjustScountovfValue(URV value) const
{
  auto csr = getImplementedCsr(CsrNumber::MCOUNTEREN);
  assert(csr and "MCOUNTEREN not implemented");
  URV mask = csr->read();
  if (virtMode_)
    {
      csr = getImplementedCsr(CsrNumber::HCOUNTEREN);
      assert(csr and "HCOUNTERN not implemented");
      mask &= csr->read();
    }
  return value & mask;
}


template <typename URV>
bool
CsRegs<URV>::readMireg(CsrNumber num, URV& value) const
{
  URV sel = 0;
  peek(CsrNumber::MISELECT, sel);
  if (imsic_)
    {
      auto csr = getImplementedCsr(num, virtMode_);
      if (not csr)
	return false;

      return imsic_->readMireg(sel, value);
    }
  return false;
}


template <typename URV>
bool
CsRegs<URV>::readSireg(CsrNumber num, URV& value) const
{
  URV sel = 0;
  peek(CsrNumber::SISELECT, sel);
  if (imsic_)
    {
      auto csr = getImplementedCsr(num, virtMode_);
      if (not csr)
	return false;

      unsigned guest = 0;
      if (virtMode_)
	{
	  URV hs = 0;
	  peek(CsrNumber::HSTATUS, hs);
	  HstatusFields<URV> hsf(hs);
	  guest = hsf.bits_.VGEIN;
	}

      return imsic_->readSireg(virtMode_, guest, sel, value);
    }
  return false;
}


template <typename URV>
bool
CsRegs<URV>::readVsireg(CsrNumber num, URV& value) const
{
  URV sel = 0;
  peek(CsrNumber::VSISELECT, sel);
  if (imsic_)
    {
      auto csr = getImplementedCsr(num, virtMode_);
      if (not csr)
	return false;

      URV hs = 0;
      peek(CsrNumber::HSTATUS, hs);
      HstatusFields<URV> hsf(hs);
      unsigned guest = hsf.bits_.VGEIN;

      return imsic_->readSireg(true, guest, sel, value);
    }
  return false;
}


template <typename URV>
bool
CsRegs<URV>::read(CsrNumber num, PrivilegeMode mode, URV& value) const
{
  using CN = CsrNumber;

  auto csr = getImplementedCsr(num, virtMode_);
  if (not csr or mode < csr->privilegeMode() or not isStateEnabled(num, mode, virtMode_))
    return false;
  num = csr->getNumber();  // CSR may have been remapped from S to VS

  if (csr->isDebug() and not inDebugMode())
    return false; // Debug-mode register.

  if (num >= CN::TDATA1 and num <= CN::TINFO)
    return readTrigger(num, mode, value);

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
  else if (num == CN::MIREG)
    return readMireg(num, value);
  else if (num == CN::SIREG)
    return readSireg(num, value);
  else if (num == CN::VSIREG)
    return readVsireg(num, value);
  else if (num == CN::SIP)
    return readSip(value);
  else if (num == CN::SIE)
    return readSie(value);

  if (num == CN::MTOPEI)
    {
      if (not imsic_)
	return false;
      value = imsic_->machineTopId();
      value |= value << 16;  // Bits 26:16 same as bits 10;0 as required by spec.
      return true;
    }
  else if (num == CN::STOPEI)
    {
      if (not imsic_)
	return false;
      value = imsic_->supervisorTopId();
      value |= value << 16;  // Bits 26:16 same as bits 10;0 as required by spec.
      return true;
    }
  else if (num == CN::VSTOPEI)
    {
      if (not imsic_)
	return false;
      const auto& hs = regs_.at(size_t(CsrNumber::HSTATUS));
      URV hsVal = hs.read();
      HstatusFields<URV> hsf(hsVal);
      unsigned vgein = hsf.bits_.VGEIN;
      if (not vgein or vgein >= imsic_->guestCount())
	return false;
      value = imsic_->guestTopId(vgein);
      value |= value << 16;  // Bits 26:16 same as bits 10;0 as required by spec.
      return true;
    }

  if (num == CN::MTOPI or num == CN::STOPI or num == CN::VSTOPI)
    return readTopi(num, value);
  else if (num == CN::SIE)
    return readSie(value);
  else if (num == CN::SIP)
    return readSip(value);
  else if (num == CN::MVIP)
    return readMvip(value);

  value = csr->read();

  if (virtMode_ and (num == CN::TIME or num == CN::TIMEH))
    value = adjustTimeValue(num, value);  // In virt mode, time is time + htimedelta.
  else if (num >= CN::PMPADDR0 and num <= CN::PMPADDR63)
    value = adjustPmpValue(num, value);
  else if (num >= CN::SSTATEEN0 and num <= CN::SSTATEEN3)
    value = adjustSstateenValue(num, value);
  else if ((num >= CN::HSTATEEN0 and num <= CN::HSTATEEN3) or
           (num >= CN::HSTATEEN0H and num <= CN::HSTATEEN3H))
    value = adjustHstateenValue(num, value);
  else if (num == CN::SCOUNTOVF)
    value = adjustScountovfValue(value);

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

  auto enableCsr = [this] (CN csrn, bool flag) {
    auto csr = findCsr(csrn);
    if (csr)
      csr->setImplemented(flag);
  };

  for (auto csrn : { CN::SSTATUS, CN::SIE, CN::STVEC, CN::SCOUNTEREN,
		     CN::SSCRATCH, CN::SEPC, CN::SCAUSE, CN::STVAL, CN::SIP,
		     CN::SENVCFG, CN::SATP, CN::MEDELEG, CN::MIDELEG,
		     CN::SCONTEXT } )
    enableCsr(csrn, flag);

  if (hyperEnabled_)
    for (auto csrn : { CN::VSSTATUS, CN::VSIE, CN::VSTVEC, CN::VSSCRATCH,
		       CN::VSEPC, CN::VSCAUSE, CN::VSTVAL, CN::VSIP, CN::VSATP } )
      enableCsr(csrn, flag);

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

  // Make IR/TM/CY bits read only zero in MCOUNTERNE/SCOUNTEREN/HCOUNTEREN if the
  // RETIRED/TIME/CYCLE CSRs is not implemented. This is not explicitly stated in the spec
  // but it is in the lore.
  URV mask = 0;  // Least sig 3 bits of MCOUNTEREN.
  if (regs_.at(unsigned(CN::CYCLE)).isImplemented())
    mask |= 1;
  if (regs_.at(unsigned(CN::TIME)).isImplemented())
    mask |= 2;
  if (regs_.at(unsigned(CN::INSTRET)).isImplemented())
    mask |= 4;
  auto& mce = regs_.at(unsigned(CN::MCOUNTEREN));
  auto& sce = regs_.at(unsigned(CN::SCOUNTEREN));
  auto& hce = regs_.at(unsigned(CN::SCOUNTEREN));
  mce.setReadMask((mce.getReadMask() & ~URV(7)) | mask);
  sce.setReadMask((sce.getReadMask() & ~URV(7)) | mask);
  hce.setReadMask((hce.getReadMask() & ~URV(7)) | mask);

  updateSstc();  // To activate/deactivate STIMECMP.
  enableSscofpmf(cofEnabled_);  // To activate/deactivate SCOUNTOVF.
  enableSmstateen(stateenOn_);  // To activate/deactivate STATEEN CSRs.
  enableTriggers(triggersOn_);  // To activate/deactivate SCONTEXT.
}


template <typename URV>
void
CsRegs<URV>::updateSstc()
{
  bool stce = menvcfgStce();
  URV mMask = 0;
  if (not peek(CsrNumber::MCOUNTEREN, mMask))
    return;
  bool mTm = (mMask & 2) >> 1;

  PrivilegeMode mode = (stce & mTm)? PrivilegeMode::Supervisor : PrivilegeMode::Machine;

  auto stimecmp = findCsr(CsrNumber::STIMECMP);
  if (sstcEnabled_ and not stimecmp->isImplemented())
    stimecmp->setImplemented(true);
  stimecmp->setPrivilegeMode(mode);
  stimecmp->setHypervisor(stce);
  if (rv32_)
    {
      auto stimecmph = findCsr(CsrNumber::STIMECMPH);
      if (sstcEnabled_ and not stimecmph->isImplemented())
	stimecmph->setImplemented(true);
      stimecmph->setPrivilegeMode(mode);
      stimecmp->setHypervisor(stce);
    }

  if (superEnabled_)
    {
      // S_TIMER bit in MIP is read-only if stimecmp is implemented and
      // writeable if it is not.
      auto mip = findCsr(CsrNumber::MIP);
      if (mip)
	{
	  URV mask = mip->getWriteMask();
	  URV stBit = URV(1) << unsigned(InterruptCause::S_TIMER);
	  bool readOnly = stce;
	  mask = readOnly? mask & ~stBit : mask | stBit;
	  mip->setWriteMask(mask);
	}
    }

  URV hMask = 0;
  peek(CsrNumber::HCOUNTEREN, hMask);
  bool hstce = henvcfgStce();
  bool hTm = (hMask & 2) >> 1;

  auto vstimecmp = findCsr(CsrNumber::VSTIMECMP);
  vstimecmp->setImplemented(sstcEnabled_ and hyperEnabled_);
  vstimecmp->setPrivilegeMode(mode);
  if (rv32_)
    {
      auto vstimecmph = findCsr(CsrNumber::VSTIMECMPH);
      vstimecmph->setImplemented(sstcEnabled_ and hyperEnabled_);
      vstimecmph->setPrivilegeMode(mode);
    }

  if (stce)
    {
      bool noVs = not (hstce and hTm);
      stimecmp->setHypervisor(noVs);
      if (rv32_)
	findCsr(CsrNumber::STIMECMPH)->setHypervisor(noVs);
    }

  auto hip = findCsr(CsrNumber::HIP);
  if (hip)
    {
      // Update VSTIP bit in HIP. See chapter 3 of SSTC spc.
      URV mask = hip->getReadMask();
      URV vstBit = URV(1) << unsigned(InterruptCause::VS_TIMER);
      if (stce and not hstce)
	mask = mask & ~vstBit;  // Make read-only zero
      else
	mask = mask | vstBit;  // Make readable
      hip->setReadMask(mask);
    }
}


template <typename URV>
void
CsRegs<URV>::enableHypervisorMode(bool flag)
{
  hyperEnabled_ = flag;

  using CN = CsrNumber;

  auto enableCsr = [this] (CN csrn, bool flag) {
    auto csr = findCsr(csrn);
    if (csr)
      csr->setImplemented(flag);
  };

  for (auto csrn : { CN::HSTATUS, CN::HEDELEG, CN::HIDELEG, CN::HIE, CN::HCOUNTEREN,
	CN::HGEIE, CN::HTVAL, CN::HIP, CN::HVIP, CN::HTINST, CN::HGEIP, CN::HENVCFG,
	CN::HGATP, CN::HCONTEXT, CN::HTIMEDELTA, CN::MTVAL2, CN::MTINST, CN::HCONTEXT } )
    enableCsr(csrn, flag);

  if (rv32_)
    for (auto csrn : { CN::HENVCFGH, CN::HTIMEDELTAH } )
      enableCsr(csrn, flag);

  if (superEnabled_)
    for (auto csrn : { CN::VSSTATUS, CN::VSIE, CN::VSTVEC, CN::VSSCRATCH,
		       CN::VSEPC, CN::VSCAUSE, CN::VSTVAL, CN::VSIP, CN::VSATP })
      enableCsr(csrn, flag);

  // Enable/disable MPV and GVA bits
  {
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

    if (not flag)
      mstatus->write(mstatus->read() & ~hyperBits);  // Clear MPV and GVA.

    URV mask = mstatus->getWriteMask();
    mask = flag? (mask | hyperBits) : (mask & ~hyperBits);
    mstatus->setWriteMask(mask);

    mask = mstatus->getPokeMask();
    mask = flag? (mask | hyperBits) : (mask & ~hyperBits);
    mstatus->setPokeMask(mask);

    mask = mstatus->getReadMask();
    mask = flag? (mask | hyperBits) : (mask & ~hyperBits);
    mstatus->setReadMask(mask);
  }

  // Bits correspondig to VSEIP, VSTIP, VSSIP, and SGEIP.
  URV vsBits = 0x444;
  URV sgBit = geilen_ ? 0x1000 : 0;  // Bit SGEIP

  auto csr = findCsr(CN::MIDELEG);
  if (flag)
    {
      // Make VSEIP, VSTIP, and VSSIP and posibly SGEIP read-only one.
      URV rooMask = vsBits | sgBit;  // Bits to be made read-only-one.
      csr->setWriteMask(csr->getWriteMask() | rooMask); // Make bits writeable.
      csr->setReadMask(csr->getReadMask() | rooMask);  // Open for reading.
      csr->write(csr->read() | rooMask);  // Set bits to one.
      csr->setWriteMask(csr->getWriteMask() & ~rooMask); // Make bits read-only.
    }
  else
    {
      // Make VSEIP, VSTIP, VSSIP, and SGEIP read only zero.
      auto mask = csr->getReadMask();
      mask &= ~URV(0x1444);
      csr->setReadMask(mask);
    }

  if (flag)
    {
      auto hideleg = getImplementedCsr(CsrNumber::HIDELEG);
      assert(hideleg);

      auto vsip = getImplementedCsr(CsrNumber::VSIP);
      auto vsie = getImplementedCsr(CsrNumber::VSIE);
      URV mask = 0x222;   // Bits VSEIP, VSTIP, and VSSIP of VSIP
      mask &= (hideleg->read() >> 1);
      if (vsip)
	vsip->setReadMask(mask);
      if (vsie)
	vsie->setReadMask(mask);
    }

  // Enable/disable hypervisor related exceptions (bits 23:20 in MEDELEG).
  csr = findCsr(CN::MEDELEG);
  if (csr)
    {
      URV bits = URV(0xf) << 20;  // Bits 23:20
      auto mask = csr->getReadMask();
      csr->setReadMask(flag ? (mask | bits) : (mask & ~bits));
    }

  // Bit MIP.VSSIP is writeable if hypervisor is enabled, otherwise it is not
  csr = findCsr(CN::MIP);
  if (csr)
    {
      URV bit = 4;
      auto mask = csr->getWriteMask();
      csr->setWriteMask(flag ? (mask | bit) : (mask & ~bit));
    }

  // In MIE, bits VSEIE, VSTIE, VSSIE, and SGEIE become read-only zero if no hypervisor.
  csr = findCsr(CN::MIE);
  if (csr)
    {
      URV bits = 0x1444;
      auto mask = csr->getReadMask();
      csr->setReadMask(flag ? (mask | bits) : (mask & ~bits));
    }

  updateSstc();                // To activate/deactivate VSTIMECMP.
  enableSmstateen(stateenOn_); // To activate/deactivate STATEEN CSRs.
  enableAia(aiaEnabled_);      // To activate/deactivate AIA hypervisor CSRs.
  enableTriggers(triggersOn_); // To activate/deactivate HCONTEXT.
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

  // If neither F or S extension is enabled then FS bits in MSTATUS are
  // read only zero; otherwise, they are readable.
  auto mstatus = findCsr(CsrNumber::MSTATUS);
  if (mstatus)
    {
      MstatusFields<URV> fields(mstatus->getReadMask());
      fields.bits_.FS = 0;

      if (flag or superEnabled_)
	fields.bits_.FS = ~ fields.bits_.FS;
      mstatus->setReadMask(fields.value_);
    }
}


template <typename URV>
void
CsRegs<URV>::enableSscofpmf(bool flag)
{
  cofEnabled_ = flag;

  auto csrn = CsrNumber::SCOUNTOVF;
  auto csr = findCsr(csrn);
  if (not csr)
    {
      std::cerr << "Error: enableSscofpmf: CSR number 0x"
		<< std::hex << URV(csrn) << std::dec << " is not defined\n";
      assert(0);
    }
  else
    csr->setImplemented(flag & superEnabled_);

  // Add CSR fields.
  std::vector<typename Csr<URV>::Field> hpm = {{"zero", 3}};
  for (unsigned i = 3; i <= 31; ++i)
    hpm.push_back({"HPM" + std::to_string(i), 1});
  setCsrFields(csrn, hpm);

  // Mask/unmask LCOF bits
  for (auto csrn : {CsrNumber::MIE, CsrNumber::MIP, CsrNumber::SIE, CsrNumber::SIP})
    {
      auto csr = findCsr(csrn);
      if (csr)
	{
	  auto lcof = URV(1) << URV(InterruptCause::LCOF);
	  if (flag)
	    {
	      csr->setWriteMask(csr->getWriteMask() | lcof);
	      csr->setReadMask(csr->getReadMask() | lcof);
	    }
	  else
	    {
	      csr->setWriteMask(csr->getWriteMask() & ~lcof);
	      csr->setReadMask(csr->getReadMask() & ~lcof);
	    }
	}
    }

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

	CsrNumber evnum = advance(CsrNumber::MHPMEVENT3, ix);
	if (rv32_)
	  evnum = advance(CsrNumber::MHPMEVENTH3, ix);
	auto event = this->findCsr(evnum);
	if (not event)
	  {
	    assert(0);
	    return;
	  }

	if (rv32_)
          {
            event->poke(fields.value_ >> 32);
	    if (superEnabled_)
	      updateScountovfValue(evnum);
          }
	else
          {
            event->poke(fields.value_);
	    if (superEnabled_)
	      updateScountovfValue(evnum);
          }
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
CsRegs<URV>::enableZicntr(bool flag)
{
  using CN = CsrNumber;
  for (auto csrn: { CN::CYCLE, CN::TIME, CN::INSTRET })
    {
      auto csr = findCsr(csrn);
      csr->setImplemented(flag);
    }
  if (rv32_)
    for (auto csrn: { CN::CYCLEH, CN::TIMEH, CN::INSTRETH })
      {
	auto csr = findCsr(csrn);
	csr->setImplemented(flag);
      }

  // Make IR/TM/CY bits read only zero in MCOUNTERNE/SCOUNTEREN/HCOUNTEREN if the
  // RETIRED/TIME/CYCLE CSRs are not implemented. This is not explicitly stated in the
  // spec but it is in the lore.
  URV mask = 7;  // Least sig 3 bits of MCOUNTEREN.
  auto& mce = regs_.at(unsigned(CN::MCOUNTEREN));
  auto& sce = regs_.at(unsigned(CN::SCOUNTEREN));
  auto& hce = regs_.at(unsigned(CN::SCOUNTEREN));

  if (flag)
    {
      mce.setReadMask(mce.getReadMask() | mask);
      sce.setReadMask(sce.getReadMask() | mask);
      hce.setReadMask(hce.getReadMask() | mask);
    }
}


template <typename URV>
void
CsRegs<URV>::enableZihpm(bool flag)
{
  using CN = CsrNumber;

  for (unsigned i = 3; i <= 31; ++i)
    {
      auto csrn = advance(CN::HPMCOUNTER3, i - 3);
      auto csr = findCsr(csrn);
      csr->setImplemented(flag);
      if (rv32_)
        {
          auto csrnh = advance(CN::HPMCOUNTER3H, i - 3);
          auto csrh = findCsr(csrnh);
          csrh->setImplemented(flag);
        }
    }

  // If zihmp is disabled make bits corresponding to counters read only zero in
  // MCOUNTEREN/SCOUNTEREN/HCOUNTEREN.
  auto& mce = regs_.at(unsigned(CN::MCOUNTEREN));
  auto& sce = regs_.at(unsigned(CN::SCOUNTEREN));
  auto& hce = regs_.at(unsigned(CN::SCOUNTEREN));
  URV mask = (~URV(0)) << 3;
  if (flag)
    {
      mce.setReadMask(mce.getReadMask() | mask);
      sce.setReadMask(sce.getReadMask() | mask);
      hce.setReadMask(hce.getReadMask() | mask);
    }
  else
    {
      mce.setReadMask(mce.getReadMask() & ~mask);
      sce.setReadMask(sce.getReadMask() & ~mask);
      hce.setReadMask(hce.getReadMask() & ~mask);
    }
}


template <typename URV>
void
CsRegs<URV>::enableSmstateen(bool flag)
{
  using CN = CsrNumber;

  stateenOn_ = flag;

  auto enableCsr = [this] (CN csrn, bool flag) {
    auto csr = findCsr(csrn);
    if (csr)
      csr->setImplemented(flag);
  };

  for (auto csrn : { CN::MSTATEEN0, CN::MSTATEEN1, CN::MSTATEEN2, CN::MSTATEEN3 } )
    enableCsr(csrn, flag);

  if (rv32_)
    for (auto csrn : { CN::MSTATEEN0H, CN::MSTATEEN1H, CN::MSTATEEN2H, CN::MSTATEEN3H } )
      enableCsr(csrn, flag);

  flag &= superEnabled_;
  for (auto csrn : { CN::SSTATEEN0, CN::SSTATEEN1, CN::SSTATEEN2, CN::SSTATEEN3 } )
    enableCsr(csrn, flag);

  flag &= hyperEnabled_;
  for (auto csrn : { CN::HSTATEEN0, CN::HSTATEEN1, CN::HSTATEEN2, CN::HSTATEEN3 } )
    enableCsr(csrn, flag);

  if (rv32_)
    for (auto csrn : { CN::HSTATEEN0H, CN::HSTATEEN1H, CN::HSTATEEN2H, CN::HSTATEEN3H } )
      enableCsr(csrn, flag);
}


template <typename URV>
void
CsRegs<URV>::enableSmrnmi(bool flag)
{
  using CN = CsrNumber; 
  for (auto csrn : { CN::MNSCRATCH, CN::MNEPC, CN::MNCAUSE, CN::MNSTATUS })
    {
      auto csr = findCsr(csrn);
      if (not csr)
	assert(0 && "Undefined CSR in SMRNMI extension");
      else
	csr->setImplemented(flag);
    }
}


template <typename URV>
void
CsRegs<URV>::enableVector(bool flag)
{
  for (auto csrn : { CsrNumber::VSTART, CsrNumber::VXSAT, CsrNumber::VXRM,
		     CsrNumber::VCSR, CsrNumber::VL, CsrNumber::VTYPE,
		     CsrNumber::VLENB } )
    {
      auto csr = findCsr(csrn);
      if (not csr)
        {
          std::cerr << "Error: CsRegs::enableVector: CSR number 0x"
                    << std::hex << URV(csrn) << std::dec << " undefined\n";
          assert(0);
        }
      else
        csr->setImplemented(flag);
    }
}


template <typename URV>
void
CsRegs<URV>::enableAia(bool flag)
{
  using CN = CsrNumber;

  aiaEnabled_ = flag;

  for (auto csrn : { CN::MISELECT, CN::MIREG, CN::MTOPEI, CN::MTOPI, CN::MVIEN,
		     CN::MVIP, CN::SISELECT, CN::SIREG, CN::STOPEI, CN::STOPI })
    {
      auto csr = findCsr(csrn);
      csr->setImplemented(flag);
    }

  bool hflag = hyperEnabled_ and flag;
  for (auto csrn : { CN::HVIEN, CN::HVICTL, CN::HVIPRIO1, CN::HVIPRIO2,   
		     CN::VSISELECT, CN::VSIREG, CN::VSTOPEI, CN::VSTOPI } )
    {
      auto csr = findCsr(csrn);
      csr->setImplemented(hflag);
    }

  if (sizeof(URV) == 4)
    {
      for (auto csrn : { CN::MIDELEGH, CN::MIEH, CN::MVIENH, CN::MVIPH, CN::MIPH,       
			 CN::SIEH, CN::SIPH, CN::HIDELEGH } )
	{
	  auto csr = findCsr(csrn);
	  csr->setImplemented(flag);
	}

      for (auto csrn : { CN::HVIENH, CN::HVIPH, CN::HVIPRIO1H, CN::HVIPRIO2H,
			 CN::VSIEH, CN::VSIPH } )
	{
	  auto csr = findCsr(csrn);
	  csr->setImplemented(hflag);
	}
    }
}


template <typename URV>
void
CsRegs<URV>::enableSsnpm(bool flag)
{
  using CN = CsrNumber;

  if (not rv32_)
    {
      uint8_t mask = flag? 0x3 : 0;
      SenvcfgFields<uint64_t> sf{regs_.at(size_t(CN::SENVCFG)).getReadMask()};
      sf.bits_.PMM = mask;
      regs_.at(size_t(CN::SENVCFG)).setReadMask(sf.value_);

      HenvcfgFields<uint64_t> hf{regs_.at(size_t(CN::HENVCFG)).getReadMask()};
      hf.bits_.PMM = mask;
      regs_.at(size_t(CN::HENVCFG)).setReadMask(hf.value_);

      HstatusFields<uint64_t> hs{regs_.at(size_t(CN::HSTATUS)).getReadMask()};
      hs.bits_.HUPMM = mask;
      regs_.at(size_t(CN::HSTATUS)).setReadMask(hs.value_);
    }
}


template <typename URV>
void
CsRegs<URV>::enableSmnpm(bool flag)
{
  using CN = CsrNumber;

  if (not rv32_)
    {
      MenvcfgFields<uint64_t> hf{regs_.at(size_t(CN::MENVCFG)).getReadMask()};
      uint8_t mask = flag? 0x3 : 0;
      hf.bits_.PMM = mask;
      regs_.at(size_t(CN::MENVCFG)).setReadMask(hf.value_);
    }
}


template <typename URV>
void
CsRegs<URV>::enableZkr(bool flag)
{
  using CN = CsrNumber;

  auto csr = findCsr(CN::SEED);
  if (not csr)
    {
      std::cerr << "Error: enableZkr: CSR number 0x"
		<< std::hex << URV(CN::SEED) << std::dec << " is not defined\n";
      assert(0);
    }
  else
    csr->setImplemented(flag);

  MseccfgFields<URV> mf{regs_.at(size_t(CN::MSECCFG)).getReadMask()};
  mf.bits_.USEED = flag;
  mf.bits_.SSEED = flag;
  regs_.at(size_t(CN::MSECCFG)).setReadMask(mf.value_);
}


template <typename URV>
URV
CsRegs<URV>::legalizeMstatusValue(URV value) const
{
  MstatusFields<URV> fields(value);
  PrivilegeMode mpp = PrivilegeMode(fields.bits_.MPP);
  PrivilegeMode spp = PrivilegeMode(fields.bits_.SPP);

  if (fields.bits_.FS == unsigned(FpStatus::Dirty) or fields.bits_.XS == unsigned(FpStatus::Dirty) or
      fields.bits_.VS == unsigned(VecStatus::Dirty))
    fields.bits_.SD = 1;
  else
    fields.bits_.SD = 0;

  assert(spp == PrivilegeMode(0) or spp == PrivilegeMode(1));

  if (not superEnabled_)
    spp = PrivilegeMode(0);

  if (mpp == PrivilegeMode::Supervisor and not superEnabled_)
    mpp = PrivilegeMode::User;

  if (mpp == PrivilegeMode::Reserved)
    mpp = PrivilegeMode::User;

  if (mpp == PrivilegeMode::User and not userEnabled_)
    mpp = PrivilegeMode::Machine;

  fields.bits_.MPP = unsigned(mpp);
  fields.bits_.SPP = unsigned(spp);

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

  if ((v & (1 << ('U' - 'A'))) == 0)
    v &= ~(URV(1) << ('S' - 'A')); // S is off if U is off.

  return v;
}


template <typename URV>
bool
CsRegs<URV>::writeSip(URV value)
{
  using CN = CsrNumber;

  assert(not virtMode_);

  Csr<URV>* sip = getImplementedCsr(CN::SIP);
  if (not sip)
    return false;

  URV prevSipMask = sip->getWriteMask();
  URV sipMask = prevSipMask;

  auto mip = getImplementedCsr(CN::MIP);
  if (mip)
    sipMask &= mip->getWriteMask(); // In case SIP mask is more permissive than MIP.

  auto mideleg = getImplementedCsr(CN::MIDELEG);
  sipMask &= mideleg ? mideleg->read() : 0; // Only delegated bits writeable

  // Bits SGEIP, VSEIP, VSTIP, VSSIP are not writeable in SIE/SIP.
  sipMask &= ~ URV(0x1444);

  // Where mideleg is 0 and mvien is 1, SIP becomes an alias to mvip.
  // See AIA spec section 5.3.
  auto mvien = getImplementedCsr(CsrNumber::MVIEN);
  auto mvip = getImplementedCsr(CsrNumber::MVIP);
  if (mideleg and mvien and mvip)
    {
      URV mvipMask = mvien->read() & ~mideleg->read();
      sipMask &= ~ mvipMask;  // Don't write SIP where SIP is an alias to mvip.
      mvip->write((mvip->read() & ~mvipMask) | (value & mvipMask)); // Write mvip instead.
    }

  sip->setWriteMask(sipMask);
  sip->write(value);
  sip->setWriteMask(prevSipMask);

  recordWrite(CN::SIP);
  return true;
}


template <typename URV>
void
CsRegs<URV>::updateShadowSie()
{
  using CN = CsrNumber;

  // See AIA spec section 5.3.
  auto sie = getImplementedCsr(CN::SIE);
  auto mideleg = getImplementedCsr(CN::MIDELEG);
  auto mvien = getImplementedCsr(CsrNumber::MVIEN);
  auto mvip = getImplementedCsr(CsrNumber::MVIP);
  if (mideleg and mvien and mvip)
    {
      URV value = sie->read();
      URV smask = mvien->read() & ~mideleg->read();
      shadowSie_ = (shadowSie_ & ~smask) | (value & smask);
    }
}


template <typename URV>
bool
CsRegs<URV>::writeSie(URV value)
{
  using CN = CsrNumber;

  assert(not virtMode_);

  Csr<URV>* sie = getImplementedCsr(CN::SIE);
  if (not sie)
    return false;

  URV prevSieMask = sie->getWriteMask();
  URV sieMask = prevSieMask;

  auto mie = getImplementedCsr(CN::MIE);
  if (mie)
    sieMask &= mie->getWriteMask(); // In case SIE mask is more permissive than MIE.

  auto mideleg = getImplementedCsr(CN::MIDELEG);
  sieMask &= mideleg ? mideleg->read() : 0; // Only delegated bits writeable

  // Bits SGEIP, VSEIP, VSTIP, VSSIP are not writeable in SIE/SIP.
  sieMask &= ~ URV(0x1444);

  // Where mideleg is 0 and mvien is 1, SIE becomes writable independent of MIP.
  // See AIA spec section 5.3.
  auto mvien = getImplementedCsr(CsrNumber::MVIEN);
  auto mvip = getImplementedCsr(CsrNumber::MVIP);
  if (mideleg and mvien and mvip)
    {
      URV smask = mvien->read() & ~mideleg->read();
      sieMask &= ~ smask;  // Don't write SIE where SIE is indepedent of MIP.
      shadowSie_ = (shadowSie_ & ~smask) | (value & smask); // Write shadow instead.
    }

  sie->setWriteMask(sieMask);
  sie->write(value);
  sie->setWriteMask(prevSieMask);

  recordWrite(CN::SIE);
  return true;
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

      auto csr = getImplementedCsr(num, virtMode_);
      if (not csr)
	return false;

      URV prevMask = csr->getWriteMask();
      URV mask = prevMask;

      CN mnum = advance(CN::MSTATEEN0, ix);
      auto mcsr = getImplementedCsr(mnum);
      if (mcsr)
	mask &= mcsr->read();

      if (virtMode_)
	{
	  CN hnum = advance(CN::HSTATEEN0, ix);
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

      auto csr = getImplementedCsr(num, virtMode_);
      if (not csr)
	return false;

      URV prevMask = csr->getWriteMask();
      URV mask = prevMask;

      CN mnum = CN::MSTATEEN0;
      if (num >= CN::HSTATEEN0H and num <= CN::HSTATEEN3H)
        mnum = CN::MSTATEEN0H;
      mnum = advance(mnum, ix);
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
CsRegs<URV>::writeMireg(CsrNumber num, URV value)
{
  URV sel = 0;
  peek(CsrNumber::MISELECT, sel);

  if (imsic_)
    {
      Csr<URV>* csr = getImplementedCsr(num, virtMode_);
      if (not csr)
	return false;

      if (imsic_->writeMireg(sel, value))
	{
	  imsic_->readMireg(sel, value);
	  csr->write(value);
	  recordWrite(num);
	  return true;
	}
    }
  return false;
}


template <typename URV>
bool
CsRegs<URV>::writeSireg(CsrNumber num, URV value)
{
  URV sel = 0;
  peek(CsrNumber::SISELECT, sel);

  if (imsic_)
    {
      Csr<URV>* csr = getImplementedCsr(num, virtMode_);
      if (not csr)
	return false;

      unsigned guest = 0;
      if (virtMode_)
	{
	  URV hs = regs_.at(size_t(CsrNumber::HSTATUS)).read();
	  HstatusFields<URV> hsf(hs);
	  guest = hsf.bits_.VGEIN;
	}

      if (imsic_->writeSireg(virtMode_, guest, sel, value))
	{
	  imsic_->readSireg(virtMode_, guest, sel, value);
	  csr->write(value);
	  recordWrite(num);
	  return true;
	}
    }
  return false;
}


template <typename URV>
bool
CsRegs<URV>::writeVsireg(CsrNumber num, URV value)
{
  URV sel = 0;
  peek(CsrNumber::VSISELECT, sel);

  if (imsic_)
    {
      Csr<URV>* csr = getImplementedCsr(num, virtMode_);
      if (not csr)
	return false;

      unsigned guest = 0;
      URV hs = regs_.at(size_t(CsrNumber::HSTATUS)).read();
      HstatusFields<URV> hsf(hs);
      guest = hsf.bits_.VGEIN;

      if (imsic_->writeSireg(true, guest, sel, value))
	{
	  imsic_->readSireg(true, guest, sel, value);
	  csr->write(value);
	  recordWrite(num);
	  return true;
	}
    }
  return false;
}


template <typename URV>
bool
CsRegs<URV>::writeMtopei()
{
  if (not imsic_)
    return false;

  // Section 3.9 of AIA: Write to STOPEI clears the pending bit
  // corresponding to the topid before the write.
  unsigned id = imsic_->machineTopId();
  if (id)
    imsic_->setMachinePending(id, false);
  return true;
}


template <typename URV>
bool
CsRegs<URV>::writeStopei()
{
  if (not imsic_)
    return false;

  // Section 3.9 of AIA: Write to STOPEI clears the pending bit
  // corresponding to the topid before the write.
  unsigned id = imsic_->supervisorTopId();
  if (id)
    imsic_->setSupervisorPending(id, false);
  return true;
}


template <typename URV>
bool
CsRegs<URV>::writeVstopei()
{
  if (not imsic_)
    return false;

  const auto& hs = regs_.at(size_t(CsrNumber::HSTATUS));
  URV hsVal = hs.read();
  HstatusFields<URV> hsf(hsVal);

  unsigned vgein = hsf.bits_.VGEIN;
  if (not vgein or vgein >= imsic_->guestCount())
    return false;

  unsigned id = imsic_->guestTopId(vgein);
  if (id)
    imsic_->setGuestPending(vgein, id, false);
  return true;
}


template <typename URV>
void
CsRegs<URV>::enableHenvcfgStce(bool flag)
{
  using CN = CsrNumber;

  // If flag is false, HENVCFG.STCE becomes read-only-zero.
  if (rv32_)
    {
      HenvcfghFields<uint32_t> hf{uint32_t(regs_.at(size_t(CN::HENVCFGH)).getReadMask())};
      hf.bits_.STCE = flag;
      regs_.at(size_t(CN::HENVCFGH)).setReadMask(hf.value_);

      hf = uint32_t(regs_.at(size_t(CN::HENVCFGH)).getWriteMask());
      hf.bits_.STCE = flag;
      regs_.at(size_t(CN::HENVCFGH)).setWriteMask(hf.value_);
    }
  else
    {
      HenvcfgFields<uint64_t> hf{regs_.at(size_t(CN::HENVCFG)).getReadMask()};
      hf.bits_.STCE = flag;
      regs_.at(size_t(CN::HENVCFG)).setReadMask(hf.value_);

      hf = regs_.at(size_t(CN::HENVCFG)).getWriteMask();
      hf.bits_.STCE = flag;
      regs_.at(size_t(CN::HENVCFG)).setWriteMask(hf.value_);
    }
}


template <typename URV>
void
CsRegs<URV>::enableMenvcfgStce(bool flag)
{
  using CN = CsrNumber;

  // If flag is false, MENVCFG.STCE becomes read-only-zero.
  if (rv32_)
    {
      MenvcfghFields<uint32_t> hf{uint32_t(regs_.at(size_t(CN::MENVCFGH)).getReadMask())};
      hf.bits_.STCE = flag;
      regs_.at(size_t(CN::MENVCFGH)).setReadMask(hf.value_);

      hf = uint32_t(regs_.at(size_t(CN::MENVCFGH)).getWriteMask());
      hf.bits_.STCE = flag;
      regs_.at(size_t(CN::MENVCFGH)).setWriteMask(hf.value_);
    }
  else
    {
      MenvcfgFields<uint64_t> hf{regs_.at(size_t(CN::MENVCFG)).getReadMask()};
      hf.bits_.STCE = flag;
      regs_.at(size_t(CN::MENVCFG)).setReadMask(hf.value_);

      hf = regs_.at(size_t(CN::MENVCFG)).getWriteMask();
      hf.bits_.STCE = flag;
      regs_.at(size_t(CN::MENVCFG)).setWriteMask(hf.value_);
    }

  bool stce = menvcfgStce();
  enableHenvcfgStce(stce);
}


template <typename URV>
void
CsRegs<URV>::enableHenvcfgPbmte(bool flag)
{
  using CN = CsrNumber;

  if (rv32_)
    {
      HenvcfghFields<uint32_t> hf{uint32_t(regs_.at(size_t(CN::HENVCFGH)).getReadMask())};
      hf.bits_.PBMTE = flag;
      regs_.at(size_t(CN::HENVCFGH)).setReadMask(hf.value_);

      hf = uint32_t(regs_.at(size_t(CN::HENVCFGH)).getWriteMask());
      hf.bits_.PBMTE = flag;
      regs_.at(size_t(CN::HENVCFGH)).setWriteMask(hf.value_);
    }
  else
    {
      HenvcfgFields<uint64_t> hf{regs_.at(size_t(CN::HENVCFG)).getReadMask()};
      hf.bits_.PBMTE = flag;
      regs_.at(size_t(CN::HENVCFG)).setReadMask(hf.value_);

      hf = regs_.at(size_t(CN::HENVCFG)).getWriteMask();
      hf.bits_.PBMTE = flag;
      regs_.at(size_t(CN::HENVCFG)).setWriteMask(hf.value_);
    }
}


template <typename URV>
void
CsRegs<URV>::enableMenvcfgPbmte(bool flag)
{
  using CN = CsrNumber;

  if (rv32_)
    {
      HenvcfghFields<uint32_t> hf{uint32_t(regs_.at(size_t(CN::MENVCFGH)).getReadMask())};
      hf.bits_.PBMTE = flag;
      regs_.at(size_t(CN::MENVCFGH)).setReadMask(hf.value_);

      hf = uint32_t(regs_.at(size_t(CN::MENVCFGH)).getWriteMask());
      hf.bits_.PBMTE = flag;
      regs_.at(size_t(CN::MENVCFGH)).setWriteMask(hf.value_);
    }
  else
    {
      HenvcfgFields<uint64_t> hf{regs_.at(size_t(CN::MENVCFG)).getReadMask()};
      hf.bits_.PBMTE = flag;
      regs_.at(size_t(CN::MENVCFG)).setReadMask(hf.value_);

      hf = regs_.at(size_t(CN::MENVCFG)).getWriteMask();
      hf.bits_.PBMTE = flag;
      regs_.at(size_t(CN::MENVCFG)).setWriteMask(hf.value_);
    }

  bool pbmte = menvcfgPbmte();
  enableHenvcfgPbmte(pbmte);
}


template <typename URV>
void
CsRegs<URV>::enableHenvcfgAdue(bool flag)
{
  using CN = CsrNumber;

  if (not rv32_)
    {
      HenvcfgFields<uint64_t> hf{regs_.at(size_t(CN::HENVCFG)).getReadMask()};
      hf.bits_.ADUE = flag;
      regs_.at(size_t(CN::HENVCFG)).setReadMask(hf.value_);

      hf = regs_.at(size_t(CN::HENVCFG)).getWriteMask();
      hf.bits_.ADUE = flag;
      regs_.at(size_t(CN::HENVCFG)).setWriteMask(hf.value_);
    }
}


template <typename URV>
void
CsRegs<URV>::enableMenvcfgAdue(bool flag)
{
  using CN = CsrNumber;

  if (not rv32_)
    {
      HenvcfgFields<uint64_t> hf{regs_.at(size_t(CN::MENVCFG)).getReadMask()};
      hf.bits_.ADUE = flag;
      regs_.at(size_t(CN::MENVCFG)).setReadMask(hf.value_);

      hf = regs_.at(size_t(CN::MENVCFG)).getWriteMask();
      hf.bits_.ADUE = flag;
      regs_.at(size_t(CN::MENVCFG)).setWriteMask(hf.value_);
    }

  bool adue = menvcfgAdue();
  enableHenvcfgAdue(adue);
}


template <typename URV>
void
CsRegs<URV>::enableTriggers(bool flag)
{
  using CN = CsrNumber;
  triggersOn_ = flag;

  auto enableCsr = [this] (CN csrn, bool flag) {
    auto csr = findCsr(csrn);
    if (csr)
      csr->setImplemented(flag);
  };

  for (auto csrn : { CN::TSELECT, CN::TDATA1, CN::TDATA2, CN::TDATA3, CN::TINFO,
		     CN::TCONTROL, CN::MCONTEXT })
    enableCsr(csrn, flag);

  enableCsr(CN::SCONTEXT, flag and superEnabled_);
  enableCsr(CN::HCONTEXT, flag and superEnabled_ and hyperEnabled_);
}


template <typename URV>
void
CsRegs<URV>::enableStee(bool flag)
{
  auto csr = findCsr(CsrNumber::C_MATP);
  if (csr)
    csr->setImplemented(flag);
}


template <typename URV>
bool
CsRegs<URV>::write(CsrNumber csrn, PrivilegeMode mode, URV value)
{
  using CN = CsrNumber;

  Csr<URV>* csr = getImplementedCsr(csrn, virtMode_);
  if (not csr or mode < csr->privilegeMode() or not isStateEnabled(csrn, mode, virtMode_) or
      csr->isReadOnly())
    return false;
  CN num = csr->getNumber();  // CSR may have been remapped from S to VS

  if (csr->isDebug() and not inDebugMode())
    return false; // Debug-mode register.

  if (isPmpaddrLocked(num))
    {
      recordWrite(num);
      return true;  // Writing a locked PMPADDR register has no effect.
    }

  if (num >= CN::TDATA1 and num <= CN::TINFO)
    {
      if (not writeTrigger(num, mode, value))
	return false;
      recordWrite(num);
      return true;
    }

  // Write mask of SIP/SIE is a combined with that of MIP/MIE and
  // delgation registers.
  if (num == CN::SIP)
    return writeSip(value);
  if (num == CN::SIE)
    return writeSie(value);
  if (num == CN::MVIP)
    return writeMvip(value);

  if (num >= CN::SSTATEEN0 and num <= CN::SSTATEEN3)
    return writeSstateen(num, value);

  if ((num >= CN::HSTATEEN0 and num <= CN::HSTATEEN3) or
      (num >= CN::HSTATEEN0H and num <= CN::HSTATEEN3H))
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

  if (num == CN::MIREG)
    return writeMireg(num, value);
  else if (num == CN::SIREG)
    return writeSireg(num, value);
  else if (num == CN::VSIREG)
    return writeVsireg(num, value);
  else if (num == CN::MTOPEI)
    return writeMtopei();
  else if (num == CN::STOPEI)
    return writeStopei();
  else if (num == CN::VSTOPEI)
    return writeVstopei();

  if (num >= CN::PMPCFG0 and num <= CN::PMPCFG15)
    {
      URV prev = 0;
      peek(num, prev);
      value = legalizePmpcfgValue(prev, value);
    }
  else if (num >= CN::PMACFG0 and num <= CN::PMACFG15)
    {
      URV prev = 0;
      peek(num, prev);
      value = legalizePmacfgValue(prev, value);
    }
  else if (num == CN::MNSTATUS)
    {
      using MNF = MnstatusFields;
      MNF mnf{value};
      if (mnf.bits_.NMIE == 0 and MNF{peekMnstatus()}.bits_.NMIE == 1)
	{
	  mnf.bits_.NMIE = 1;  // Attempt to clear mnstatus.nmie has no effect
	  value = mnf.value_;
	}
    }
  else if (num == CN::TSELECT)
    {
      if (value >= triggers_.size())
	return true; // New value out of bounds. Preserve old.
    }

  csr->write(value);
  recordWrite(csrn);

  if (num == CN::MENVCFG)
    {
      bool stce = menvcfgStce();
      enableHenvcfgStce(stce); // MENVCFG.STCE off makes HENVCFG.STCE read-only zero.

      bool pbmte = menvcfgPbmte();
      enableHenvcfgPbmte(pbmte);

      bool adue = menvcfgAdue();
      enableHenvcfgAdue(adue);
    }
  else if ((num >= CN::MHPMEVENT3 and num <= CN::MHPMEVENT31) or
	   (num >= CN::MHPMEVENTH3 and num <= CN::MHPMEVENTH31))
    {
      updateCounterControl(num);
      if (cofEnabled_ and superEnabled_)
        {
          if (not rv32_ or (rv32_ and num >= CN::MHPMEVENTH3 and num <= CN::MHPMEVENTH31))
            updateScountovfValue(num);
        }
    }
  else if (num == CN::FFLAGS or num == CN::FRM or num == CN::FCSR)
    updateFcsrGroupForWrite(num, value);   // fflags and frm are part of fcsr
  else if (num == CN::VXSAT or num == CN::VXRM or num == CN::VCSR)
    updateVcsrGroupForWrite(num, value);   // vxsat and vrm are part of vcsr
  else if (num == CN::MCOUNTEREN or num == CN::SCOUNTEREN or num == CN::HCOUNTEREN)
    updateCounterPrivilege();  // Reflect counter accessibility in user/supervisor.
  else if (num == CN::HVICTL)
    updateVirtInterruptCtl();
  else if (num == CN::MVIEN or num == CN::MVIP)
    updateShadowSie();
  else if (num == CN::TCONTROL)
    triggers_.enableMachineMode(tcontrolMte());
  else
    hyperWrite(csr);   // Update hypervisor CSR aliased bits.

  if (num == CN::MENVCFG or num == CN::HENVCFG)
    updateSstc();

  return true;
}


template <typename URV>
bool
CsRegs<URV>::isWriteable(CsrNumber num, PrivilegeMode pm, bool vm) const
{
  const Csr<URV>* csr = getImplementedCsr(num, vm);
  if (not csr or pm < csr->privilegeMode() or not isStateEnabled(num, pm, vm) or
      csr->isReadOnly())
    return false;

  if (csr->isDebug() and not inDebugMode())
    return false;  // Debug-mode register.

  return true;
}


template <typename URV>
bool
CsRegs<URV>::isReadable(CsrNumber num, PrivilegeMode pm, bool vm) const
{
  const Csr<URV>* csr = getImplementedCsr(num, vm);
  if (not csr or pm < csr->privilegeMode() or not isStateEnabled(num, pm, vm))
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
      {
	csr.reset();
	if (hyperEnabled_ and csr.getNumber() == CsrNumber::MIDELEG)
	  {
	    // If hypervisor is enabled then VSEIP, VTSIP, VSSIP, and SGEIP bits
	    // of MIDELEG are read-only one.
	    auto sgeip = geilen_ ? URV(1) << 12 : URV(0);  // Bit SGEIP
	    URV vsBits = URV(0x444) | sgeip;
	    csr.pokeNoMask(csr.read() | vsBits);
	  }
      }

  triggers_.reset();
  mPerfRegs_.reset();
  triggers_.enableMachineMode(tcontrolMte());

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
                       URV mask, URV pokeMask, bool shared)
{
  auto iter = nameToNumber_.find(name);
  if (iter == nameToNumber_.end())
    return false;

  size_t num = size_t(iter->second);
  if (num >= regs_.size())
    return false;

  return configCsr(CsrNumber(num), implemented, resetValue, mask, pokeMask, shared);
}


template <typename URV>
bool
CsRegs<URV>::configCsrByUser(std::string_view name, bool implemented, URV resetValue,
			     URV mask, URV pokeMask, bool shared)
{
  auto iter = nameToNumber_.find(name);
  if (iter == nameToNumber_.end())
    return false;

  size_t num = size_t(iter->second);
  if (num >= regs_.size())
    return false;

  bool ok = configCsr(CsrNumber(num), implemented, resetValue, mask, pokeMask, shared);

  // Make user choice to disable a CSR sticky.
  if (not implemented)
    {
      auto csr = findCsr(CsrNumber(num));
      if (csr)
	{
	  if (csr->isMandatory())
	    std::cerr << "Error: Cannot disable mandatory CSR " << csr->getName() << '\n';
	  else
	    csr->setUserDisabled(true);
	}
    }

  return ok;
}


template <typename URV>
bool
CsRegs<URV>::configCsr(CsrNumber csrNum, bool implemented, URV resetValue,
                       URV mask, URV pokeMask, bool shared)
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
  csr.setIsDebug(csrNum >= CsrNumber::_MIN_DBG and csrNum <= CsrNumber::_MAX_DBG);
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
CsRegs<URV>::configMachineModePerfCounters(unsigned numCounters, bool cof)
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
      URV evMask = ~URV(0), evPokeMask = ~URV(0);  // Event regs masks

      if constexpr (sizeof(URV) == 8)
	{
	  // If counter overflow is on, then bits 56 and 57 are reserved.
	  if (cof)
	    {
	      MhpmeventFields fields{0};
	      fields.bits_.res = ~fields.bits_.res;  // Set reserved bits to all ones
	      evMask &= ~fields.value_;        // Clear reserved bits in mask
	      evPokeMask &= ~fields.value_;    // Clear reserved bits in mask
	    }
	}

      if (i >= numCounters)
	mask = pokeMask = evMask = evPokeMask = 0;

      CsrNumber csrNum = advance(CsrNumber::MHPMCOUNTER3, i);
      if (not configCsr(csrNum, true, resetValue, mask, pokeMask, shared))
	errors++;

      if (rv32_)
         {
	   csrNum = advance(CsrNumber::MHPMCOUNTER3H, i);
	   if (not configCsr(csrNum, true, resetValue, mask, pokeMask, shared))
	     errors++;
	 }

      csrNum = advance(CsrNumber::MHPMEVENT3, i);
      if (not configCsr(csrNum, true, resetValue, evMask, evPokeMask, shared))
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

  // Configure numCouters. These will be tied to the corresponding
  // machine perf counters in tiePerfCounters.
  for (unsigned i = 0; i < 29; ++i)
    {
      URV resetValue = 0, mask = ~URV(0), pokeMask = ~URV(0);
      if (i >= numCounters)
	mask = pokeMask = 0;

      CsrNumber csrNum = advance(CsrNumber::HPMCOUNTER3, i);
      if (not configCsr(csrNum, false, resetValue, mask, pokeMask, shared))
	errors++;

      if (rv32_)
         {
	   csrNum = advance(CsrNumber::HPMCOUNTER3H, i);
	   if (not configCsr(csrNum, false, resetValue, mask, pokeMask, shared))
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
  unsigned ix = unsigned(num);

  // When CSR with corresponds virtual CSR is written (e.g. stval and
  // vstval), mark the virtual CSR so that it gets reported as modified.
  if (virtMode_ and ix < regs_.size() and regs_.at(ix).mapsToVirtual())
    {
      CsrNumber vnum = advance(num, 0x100);  // Get VCSR corresponding to CSR
      if (std::find(lwr.begin(), lwr.end(), vnum) == lwr.end())
	lwr.push_back(vnum);
      return;
    }

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
      // SXL and UXL (currently not writable).
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

  val = 0x4020112d;  // MISA: acdfimv
  if constexpr (sizeof(URV) == 8)
    val = 0x800000000020112d;  // MISA: acdfimv
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

  mask = pokeMask = 0xffffffff;  // Only least sig 32 bits writable
  defineCsr("mcounteren", Csrn::MCOUNTEREN, !mand, imp, 0, mask, pokeMask);

  mask = 0xfffffffd;  // Least sig 32 bis writable except for bit 1.
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
  for (unsigned i = 0; i < 16; ++i)
    {
      bool implemented = rv32_ or (i & 1) == 0;  // Only even numbered CSRs in rv64
      std::string name = std::string("pmpcfg") + std::to_string(i);
      Csrn csrn = Csrn(unsigned(Csrn::PMPCFG0) + i);
      defineCsr(name, csrn,  !mand, implemented, 0, cfgMask, cfgMask);
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

  URV menvMask = 0xf1;
  if constexpr (sizeof(URV) == 8)
    menvMask = 0xe0000003000000f1;
  defineCsr("menvcfg", Csrn::MENVCFG, !mand, imp, 0, menvMask, menvMask);
  if (rv32_)
    {
      menvMask = 0xe0000003;
      auto c = defineCsr("menvcfgh", Csrn::MENVCFGH, !mand, imp, 0, menvMask, menvMask);
      c->markAsHighHalf(true);
    }

  uint32_t mseMask = 0x300;
  defineCsr("mseccfg", Csrn::MSECCFG, !mand, imp, 0, mseMask, mseMask);
  if (rv32_)
    {
      auto c = defineCsr("mseccfgh", Csrn::MSECCFGH, !mand, imp, 0, rom, rom);
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

  // Non maskable interrupts.
  defineCsr("mnscratch", Csrn::MNSCRATCH, !mand, !imp, 0, wam, wam);

  mask = ~URV(1); // Bit 0 of MNEPC is not writeable
  defineCsr("mnepc", Csrn::MNEPC, !mand, !imp, 0, mask, mask);

  mask = URV(1) << (sizeof(URV)*8 - 1);  // Most sig bit is read-only 1
  defineCsr("mncause", Csrn::MNCAUSE, !mand, !imp, mask, ~mask, ~mask);

  mask = 0b1100010001000;  // Fields MNPV, MNPP, and NMIE writeable.
  defineCsr("mnstatus", Csrn::MNSTATUS, !mand, !imp, 0, mask, pokeMask);

  // Define mhpmcounter3/mhpmcounter3h to mhpmcounter31/mhpmcounter31h
  // as write-anything/read-zero (user can change that in the config
  // file by setting the number of writeable counters). Same for
  // mhpmevent3/mhpmevent3h to mhpmevent3h/mhpmevent31h.
  for (unsigned i = 3; i <= 31; ++i)
    {
      CsrNumber csrNum = advance(CsrNumber::MHPMCOUNTER3, i - 3);
      std::string name = "mhpmcounter" + std::to_string(i);
      defineCsr(name, csrNum, mand, imp, 0, rom, rom);

      if (rv32_)
        {
          // High register counterpart of mhpmcounter.
          name += "h";
          csrNum = advance(CsrNumber::MHPMCOUNTER3H, i - 3);
          bool hmand = rv32_;  // high counters mandatory only in rv32
          auto c = defineCsr(std::move(name), csrNum, hmand, imp, 0, rom, rom);
	  c->markAsHighHalf(true);
        }

      csrNum = advance(CsrNumber::MHPMEVENT3, i - 3);
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

  mask = pokeMask = 0xffffffff;  // Only least sig 32 bits writable
  defineCsr("scounteren", Csrn::SCOUNTEREN, !mand, !imp, 0, wam, wam);

  // Supervisor Trap Handling 
  defineCsr("sscratch",   Csrn::SSCRATCH,   !mand, !imp, 0, wam, wam);
  mask = ~URV(1);  // Bit 0 of SEPC is not writable.
  defineCsr("sepc",       Csrn::SEPC,       !mand, !imp, 0, mask, mask);
  defineCsr("scause",     Csrn::SCAUSE,     !mand, !imp, 0, wam, wam);
  defineCsr("stval",      Csrn::STVAL,      !mand, !imp, 0, wam, wam);

  // Bits of SIE appear hardwired to zreo unless delegated. By default only bit LOCFIE,
  // SSIE, STIE, and SEIE are writeable when delegated.
  mask = 0x2222;
  defineCsr("sie",        Csrn::SIE,        !mand, !imp, 0, mask, mask);
  auto sie = findCsr(Csrn::SIE);
  auto mie = findCsr(Csrn::MIE);
  if (sie and mie)
    sie->tie(mie->valuePtr_);

  // Bits of SIP appear hardwired to zreo unless delegated.
  mask = 0x2002;  // Only bits LCOFIP and SSIP bit writable (when delegated)
  defineCsr("sip",        Csrn::SIP,        !mand, !imp, 0, mask, mask);

  auto sip = findCsr(Csrn::SIP);
  auto mip = findCsr(Csrn::MIP);
  if (sip and mip)
    sip->tie(mip->valuePtr_); // Sip is a shadow if mip

  mask = 0xf1;
  if constexpr (sizeof(URV) == 8)
    mask = 0x00000003000000f1;  // PMM field writable.
  defineCsr("senvcfg",    Csrn::SENVCFG,    !mand, !imp, 0, mask, mask);

  mask = 0;
  defineCsr("scountovf",  Csrn::SCOUNTOVF,  !mand, !imp, 0, mask, 0xfffffff8);

  // Supervisor Protection and Translation 
  defineCsr("satp",       Csrn::SATP,       !mand, !imp, 0, wam, wam);

  // Supervisor time compare.
  defineCsr("stimecmp",   Csrn::STIMECMP,   !mand, !imp, 0, wam, wam);

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

  using CN = CsrNumber;

  // User Counter/Timers
  auto c = defineCsr("cycle", CN::CYCLE,    !mand, !imp,  0, wam, wam);
  c->setHypervisor(true);
  c = defineCsr("time",       CN::TIME,     !mand, !imp,  0, wam, wam);
  c->setHypervisor(true);
  c = defineCsr("instret",    CN::INSTRET,  !mand, !imp,  0, wam, wam);
  c->setHypervisor(true);

  c = defineCsr("cycleh",     CN::CYCLEH,   !mand, !imp, 0, wam, wam);
  c->setHypervisor(true);
  c->markAsHighHalf(true);
  c = defineCsr("timeh",      CN::TIMEH,    !mand, !imp, 0, wam, wam);
  c->setHypervisor(true);
  c->markAsHighHalf(true);
  c = defineCsr("instreth",   CN::INSTRETH, !mand, !imp, 0, wam, wam);
  c->setHypervisor(true);
  c->markAsHighHalf(true);

  // Define hpmcounter3/hpmcounter3h to hpmcounter31/hpmcounter31h
  // as write-anything/read-zero (user can change that in the config
  // file).  Same for mhpmevent3/mhpmevent3h to mhpmevent3h/mhpmevent31h.
  for (unsigned i = 3; i <= 31; ++i)
    {
      CsrNumber csrNum = advance(CsrNumber::HPMCOUNTER3, i - 3);
      std::string name = "hpmcounter" + std::to_string(i);
      c = defineCsr(name, csrNum, !mand, !imp, 0, wam, wam);
      c->setHypervisor(true);

      // High register counterpart of mhpmcounter.
      name += "h";
      csrNum = advance(CsrNumber::HPMCOUNTER3H, i - 3);
      c = defineCsr(std::move(name), csrNum, !mand, !imp, 0, wam, wam);
      c->setHypervisor(true);
      c->markAsHighHalf(true);
    }

  // Quality of service
  URV mask = 0x0fff0fff;
  c = defineCsr("srmcfg", CN::SRMCFG, !mand, !imp, 0, mask, mask);
  c->setHypervisor(true);

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

  URV reset = 0;
  URV mask = 0b000000000'1'1'1'00'111111'00'1'1'1'1'1'00000;
  if constexpr (sizeof(URV) == 8)
    reset = reset | (URV(2) << 32);  // VSXL = 2 (64-bits).
  URV pokeMask = mask;
    
  csr = defineCsr("hstatus",     Csrn::HSTATUS,     !mand, !imp, reset, mask, pokeMask);
  csr->setHypervisor(true);

  using EC = ExceptionCause;
  mask = ~((URV(1) << unsigned(EC::S_ENV_CALL))               |
	   (URV(1) << unsigned(EC::VS_ENV_CALL))              |
	   (URV(1) << unsigned(EC::M_ENV_CALL))               |
	   (URV(1) << unsigned(EC::INST_GUEST_PAGE_FAULT))    |
	   (URV(1) << unsigned(EC::LOAD_GUEST_PAGE_FAULT))    |
	   (URV(1) << unsigned(EC::VIRT_INST))                |
	   (URV(1) << unsigned(EC::STORE_GUEST_PAGE_FAULT)))  ;

  pokeMask = mask;
  csr = defineCsr("hedeleg",     Csrn::HEDELEG,     !mand, !imp, 0, mask, pokeMask);
  csr->setHypervisor(true);

  using IC = InterruptCause;

  // Bits 10 6 and 2 are writeable.
  mask = ((URV(1) << unsigned(IC::VS_SOFTWARE))  |
	  (URV(1) << unsigned(IC::VS_TIMER))     |
	  (URV(1) << unsigned(IC::VS_EXTERNAL))) ;

  pokeMask = mask;
  csr = defineCsr("hideleg",     Csrn::HIDELEG,     !mand, !imp, 0, mask, pokeMask);
  csr->setHypervisor(true);

  pokeMask = mask = 0x1444;     // Bits SGEIP, VSEIP, VSTIP, and VSSIP writeable.
  csr = defineCsr("hie",         Csrn::HIE,         !mand, !imp, 0, mask, pokeMask);
  csr->setHypervisor(true);

  mask = pokeMask = 0xffffffff;  // Only least sig 32 bits writable
  csr = defineCsr("hcounteren",  Csrn::HCOUNTEREN,  !mand, !imp, 0, mask, pokeMask);
  csr->setHypervisor(true);
  pokeMask = mask = ~URV(1); // All bits writeable except bit 0
  csr = defineCsr("hgeie",       Csrn::HGEIE,       !mand, !imp, 0, mask, pokeMask);
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
  pokeMask = mask = ~URV(1); // All bits writeable except bit 0
  csr = defineCsr("hgeip",       Csrn::HGEIP,       !mand, !imp, 0, mask, pokeMask);
  csr->setHypervisor(true);
  csr = defineCsr("henvcfg",     Csrn::HENVCFG,     !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("henvcfgh",    Csrn::HENVCFGH,    !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true); csr->markAsHighHalf(true);

  mask = ~(URV(0x3) << (rv32_? 29 : 58));
  pokeMask = mask;
  csr = defineCsr("hgatp",       Csrn::HGATP,       !mand, !imp, 0, mask, pokeMask);
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

  // vsstatus
  //           S R        T T T M S M X  F  M  V  S M U S U M R S U
  //           D E        S W V X U P S  S  P  S  P P B P P I E I I
  //             S        R   M R M R       P     P I E I I E S E E
  //                                V               E   E E
  mask     = 0b0'00000000'0'0'0'1'1'0'11'11'00'11'1'0'0'1'0'0'0'1'0;
  URV val  = 0b0'00000000'0'0'0'0'0'0'00'00'00'00'0'0'0'0'0'0'0'0'0;
  pokeMask = mask | (URV(1) << (sizeof(URV)*8 - 1));  // Make SD pokable.
  if (not rv32_)
    {
      val |= uint64_t(0b10) << 32;  // Value of UXL: uxlen=64
    }
  csr = defineCsr("vsstatus",    Csrn::VSSTATUS,    !mand, !imp, val, mask, pokeMask);
  csr->setHypervisor(true);

  mask = pokeMask = 0x222;
  csr = defineCsr("vsie",        Csrn::VSIE,        !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);

  csr = defineCsr("vstvec",      Csrn::VSTVEC,      !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("vssratch",    Csrn::VSSCRATCH,   !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  mask = ~URV(1);  // Bit 0 of VSEPC is not writable.
  csr = defineCsr("vsepc",       Csrn::VSEPC,       !mand, !imp, 0, mask, mask);
  csr->setHypervisor(true);
  csr = defineCsr("vscause",     Csrn::VSCAUSE,     !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);
  csr = defineCsr("vstval",      Csrn::VSTVAL,      !mand, !imp, 0, wam, wam);
  csr->setHypervisor(true);

  mask = 0x2;   // Only bit SSIE is writeable
  pokeMask = 0x222;
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

  // In MIP bits corresponding to SGEIP/VSEIP/VSTIP/VSSIP are pokeable.
  csr = findCsr(Csrn::MIP);
  if (csr)
    {
      csr->setPokeMask(csr->getPokeMask() | 0x1444);
      csr->setWriteMask(csr->getWriteMask() | 0x4);  // Bit VSSIP is writeable.
    }

  // In MIE bits corresponding to SGEIP/VSEIP/VSTIP/VSSIP are pokeable/writeable.
  csr = findCsr(Csrn::MIE);
  if (csr)
    {
      csr->setWriteMask(csr->getWriteMask() | 0x1444);
      csr->setPokeMask(csr->getPokeMask() | 0x1444);
    }

  addHypervisorFields();
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
  defineCsr("scontext",  Csrn::SCONTEXT,  !mand, !imp,  0, wam, wam);
  defineCsr("mscontext", Csrn::MSCONTEXT, !mand, !imp,  0, wam, wam);
  defineCsr("tselect",   Csrn::TSELECT,   !mand, !imp,  0, wam, wam);
  defineCsr("tdata1",    Csrn::TDATA1,    !mand, !imp,  0, wam, wam);
  defineCsr("tdata2",    Csrn::TDATA2,    !mand, !imp,  0, wam, wam);
  defineCsr("tdata3",    Csrn::TDATA3,    !mand, !imp,  0, wam, wam);

  URV mask = 0x100ffff;   // Only least sig bit of version is writeable.
  URV reset = 0x10087d;   // Version 1, Tmext/Legacy/Custom types are not supported.
  defineCsr("tinfo",    Csrn::TINFO,    !mand, !imp,  reset, mask, mask);

  mask = 0x88;   // Only MPTE and MTE bits writable.
  defineCsr("tcontrol", Csrn::TCONTROL, !mand, !imp, 0, mask, mask);

  defineCsr("mcontext", Csrn::MCONTEXT, !mand, !imp, 0, wam, wam);
  if (not nameToNumber_.contains("hcontext"))
    defineCsr("hcontext", Csrn::HCONTEXT, !mand, !imp, 0, wam, wam);

  // Define triggers.
  unsigned triggerCount = 4;  // FIXME: why 4?
  triggers_ = Triggers<URV>(triggerCount);

  // Debug mode registers.
  URV dcsrVal = 0x40000003;
  URV dcsrMask = 0x00008e04;
  URV dcsrPokeMask = dcsrMask | 0x1cf; // Cause field modifiable
  defineCsr("dcsr", Csrn::DCSR, !mand, imp, dcsrVal, dcsrMask, dcsrPokeMask);

  // Least sig bit of dpc is not writeable.
  URV dpcMask = ~URV(1);
  defineCsr("dpc", CsrNumber::DPC, !mand, imp, 0, dpcMask, dpcMask);

  defineCsr("dscratch0", CsrNumber::DSCRATCH0, !mand, !imp, 0, wam, wam);
  defineCsr("dscratch1", CsrNumber::DSCRATCH1, !mand, !imp, 0, wam, wam);

  // Add CSR fields.
  addDebugFields();
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
  defineCsr("vtype",  CsrNumber::VTYPE,  !mand, !imp, mask & ~URV(0xff), mask, mask);

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
  using CN = CsrNumber;

  defineCsr("miselect",   CN::MISELECT,   !mand, !imp, 0, wam, wam);
  defineCsr("mireg",      CN::MIREG,      !mand, !imp, 0, wam, wam);
  defineCsr("mtopei",     CN::MTOPEI,     !mand, !imp, 0, wam, wam);
  defineCsr("mtopi",      CN::MTOPI,      !mand, !imp, 0, wam, wam);
  
  URV mask = 0b10'000'0010;  // Bits 9 and 1 (SEI, SSI).
  defineCsr("mvien",      CN::MVIEN,      !mand, !imp, 0, mask, mask);

  defineCsr("mvip",       CN::MVIP,       !mand, !imp, 0, mask, mask);
  defineCsr("siselect",   CN::SISELECT,   !mand, !imp, 0, wam, wam);
  defineCsr("sireg",      CN::SIREG,      !mand, !imp, 0, wam, wam);
  defineCsr("stopei",     CN::STOPEI,     !mand, !imp, 0, wam, wam);
  defineCsr("stopi",      CN::STOPI,      !mand, !imp, 0, wam, wam);

  defineCsr("hvien",      CN::HVIEN,      !mand, !imp, 0, wam, wam)->setHypervisor(true);
  defineCsr("hvictl",     CN::HVICTL,     !mand, !imp, 0, wam, wam)->setHypervisor(true);
  defineCsr("hviprio1",   CN::HVIPRIO1,   !mand, !imp, 0, wam, wam)->setHypervisor(true);
  defineCsr("hviprio2",   CN::HVIPRIO2,   !mand, !imp, 0, wam, wam)->setHypervisor(true);
  defineCsr("vsiselect",  CN::VSISELECT,  !mand, !imp, 0, wam, wam)->setHypervisor(true);
  defineCsr("vsireg",     CN::VSIREG,     !mand, !imp, 0, wam, wam)->setHypervisor(true);
  defineCsr("vstopei",    CN::VSTOPEI,    !mand, !imp, 0, wam, wam)->setHypervisor(true);
  defineCsr("vstopi",     CN::VSTOPI,     !mand, !imp, 0, wam, wam)->setHypervisor(true);

  for (auto csrn : { CN::SISELECT, CN::SIREG, CN::STOPEI, CN::STOPI } )
    {
      auto csr = findCsr(csrn);
      if (csr)
	csr->setMapsToVirtual(true);
    }

  if (sizeof(URV) == 4)
    {
      defineCsr("midelegh", CN::MIDELEGH, !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
      defineCsr("mieh",     CN::MIEH,     !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
      defineCsr("mvienh",   CN::MVIENH,   !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
      defineCsr("mviph",    CN::MVIPH,    !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
      defineCsr("miph",     CN::MIPH,     !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
      defineCsr("sieh",     CN::SIEH,     !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
      defineCsr("siph",     CN::SIPH,     !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
      defineCsr("hidelegh", CN::HIDELEGH, !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
      defineCsr("hvienh",   CN::HVIENH,   !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
      defineCsr("hviph",    CN::HVIPH,    !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
      defineCsr("hviprio1h",CN::HVIPRIO1H,!mand, !imp, 0, wam, wam)->markAsHighHalf(true);
      defineCsr("hviprio2h",CN::HVIPRIO2H,!mand, !imp, 0, wam, wam)->markAsHighHalf(true);
      defineCsr("vsieh",    CN::VSIEH,    !mand, !imp, 0, wam, wam)->markAsHighHalf(true);
      defineCsr("vsiph",    CN::VSIPH,    !mand, !imp, 0, wam, wam)->markAsHighHalf(true);

      for (auto csrn : { CN::SIEH, CN::SIPH } )
	{
	  auto csr = findCsr(csrn);
	  if (csr)
	    csr->setMapsToVirtual(true);
	}

      for (auto csrn : { CN::HVIENH, CN::HVIPRIO1H, CN::HVIPRIO2H } )
	{
	  auto csr = findCsr(csrn);
	  if (csr)
	    csr->setHypervisor(true);
	}
    }

  addAiaFields();
}


template <typename URV>
void
CsRegs<URV>::defineStateEnableRegs()
{
  bool mand = true;  // Mndatory
  bool imp = true;   // Implemented

  // Default: none of the sstateen CSRs are writable.
  defineCsr("sstateen0", CsrNumber::SSTATEEN0,  !mand, !imp, 0, 0, 0);
  defineCsr("sstateen1", CsrNumber::SSTATEEN1,  !mand, !imp, 0, 0, 0);
  defineCsr("sstateen2", CsrNumber::SSTATEEN2,  !mand, !imp, 0, 0, 0);
  defineCsr("sstateen3", CsrNumber::SSTATEEN3,  !mand, !imp, 0, 0, 0);

  URV mask = 0;  // Default: nothing writable.

  if constexpr (sizeof(URV) == 8)
    mask = uint64_t(0b1101111) << 57;   // Bits 57 to 63

  defineCsr("mstateen0", CsrNumber::MSTATEEN0,  !mand, !imp, 0, mask, mask);
  defineCsr("mstateen1", CsrNumber::MSTATEEN1,  !mand, !imp, 0, 0, 0);
  defineCsr("mstateen2", CsrNumber::MSTATEEN2,  !mand, !imp, 0, 0, 0);
  defineCsr("mstateen3", CsrNumber::MSTATEEN3,  !mand, !imp, 0, 0, 0);

  defineCsr("hstateen0", CsrNumber::HSTATEEN0,  !mand, !imp, 0, mask, mask)->setHypervisor(true);
  defineCsr("hstateen1", CsrNumber::HSTATEEN1,  !mand, !imp, 0, 0, 0)->setHypervisor(true);
  defineCsr("hstateen2", CsrNumber::HSTATEEN2,  !mand, !imp, 0, 0, 0)->setHypervisor(true);
  defineCsr("hstateen3", CsrNumber::HSTATEEN3,  !mand, !imp, 0, 0, 0)->setHypervisor(true);

  if (sizeof(URV) == 4)
    {
      mask = URV(0b1101111) << 25;   // Bits 25 to 31
      defineCsr("sstateen0h", CsrNumber::MSTATEEN0H,  !mand, !imp, 0, mask, mask);
      defineCsr("sstateen1h", CsrNumber::MSTATEEN1H,  !mand, !imp, 0, 0, 0);
      defineCsr("sstateen2h", CsrNumber::MSTATEEN2H,  !mand, !imp, 0, 0, 0);
      defineCsr("sstateen3h", CsrNumber::MSTATEEN3H,  !mand, !imp, 0, 0, 0);

      defineCsr("hstateen0h", CsrNumber::HSTATEEN0H,  !mand, !imp, 0, mask, mask);
      defineCsr("hstateen1h", CsrNumber::HSTATEEN1H,  !mand, !imp, 0, 0, 0);
      defineCsr("hstateen2h", CsrNumber::HSTATEEN2H,  !mand, !imp, 0, 0, 0);
      defineCsr("hstateen3h", CsrNumber::HSTATEEN3H,  !mand, !imp, 0, 0, 0);
    }
}


template <typename URV>
void
CsRegs<URV>::defineEntropyReg()
{
  using CN = CsrNumber;

  bool imp = false;
  bool mand = false;

  uint32_t rom = 0;
  uint32_t pokeMask = 0xc000ffff;

  // Entropy source
  auto csr = defineCsr("seed", CN::SEED, mand, imp, 0, rom, pokeMask);
  csr->setHypervisor(true);
}


template <typename URV>
void
CsRegs<URV>::definePmaRegs()
{
  using CN = CsrNumber;

  bool imp = true;
  bool mand = true;

  uint64_t reset = 0x7, mask = 0xfc0ffffffffff1ff;
  uint64_t pokeMask = ~(uint64_t(0x3f) << 52);   // Bits 52 to 57 are read only zero

  for (unsigned i = 0; i < 16; ++i)
    {
      std::string name = std::string("pmacfg") + std::to_string(i);
      CN num = advance(CN::PMACFG0, i);
      defineCsr(name, num, !mand, !imp, reset, mask, pokeMask);
    }
}


template <typename URV>
void
CsRegs<URV>::defineSteeRegs()
{
  bool imp = false;
  bool mand = false;
  uint64_t reset = 0, mask = 0x1, pokeMask = 0x1;
  defineCsr("c_matp", CsrNumber::C_MATP, !mand, !imp, reset, mask, pokeMask);
}


template <typename URV>
bool
CsRegs<URV>::peek(CsrNumber num, URV& value, bool virtMode) const
{
  using CN = CsrNumber;

  auto csr = getImplementedCsr(num, virtMode);
  if (not csr)
    return false;
  num = csr->getNumber();  // CSR may have been remapped from S to VS

  if (num >= CN::TDATA1 and num <= CN::TINFO)
    return readTrigger(num, PrivilegeMode::Machine, value);

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

  if (num == CN::MTOPEI)
    {
      if (not imsic_)
	return false;
      value = imsic_->machineTopId();
      value |= value << 16;  // Bits 26:16 same as bits 10;0 as required by spec.
      return true;
    }
  else if (num == CN::STOPEI)
    {
      if (not imsic_)
	return false;
      value = imsic_->supervisorTopId();
      value |= value << 16;  // Bits 26:16 same as bits 10;0 as required by spec.
      return true;
    }
  else if (num == CN::VSTOPEI)
    {
      if (not imsic_)
	return false;
      const auto& hs = regs_.at(size_t(CsrNumber::HSTATUS));
      URV hsVal = hs.read();
      HstatusFields<URV> hsf(hsVal);
      unsigned vgein = hsf.bits_.VGEIN;
      if (not vgein or vgein >= imsic_->guestCount())
	return false;
      value = imsic_->guestTopId(vgein);
      value |= value << 16;  // Bits 26:16 same as bits 10;0 as required by spec.
      return true;
    }

  if (num == CsrNumber::MTOPI or num == CsrNumber::STOPI or
      num == CsrNumber::VSTOPI)
    return readTopi(num, value);
  else if (num == CN::SIP)
    return readSip(value);
  else if (num == CN::SIE)
    return readSie(value);
  else if (num == CN::MVIP)
    return readMvip(value);

  value = csr->read();

  if (virtMode and (num == CN::TIME or num == CN::TIMEH))
    value = adjustTimeValue(num, value);  // In virt mode, time is time + htimedelta.
  else if (num >= CN::PMPADDR0 and num <= CN::PMPADDR63)
    value = adjustPmpValue(num, value);
  else if (num >= CN::SSTATEEN0 and num <= CN::SSTATEEN3)
    value = adjustSstateenValue(num, value);
  else if ((num >= CN::HSTATEEN0 and num <= CN::HSTATEEN3) or
           (num >= CN::HSTATEEN0H and num <= CN::HSTATEEN3H))
    value = adjustHstateenValue(num, value);
  else if (num == CN::SCOUNTOVF)
    value = adjustScountovfValue(value);

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

  if (num >= CN::TDATA1 and num <= CN::TINFO)
    return pokeTrigger(num, value);

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
  else if (num >= CN::PMACFG0 and num <= CN::PMACFG15)
    {
      URV prev = 0;
      peek(num, prev);
      value = legalizePmacfgValue(prev, value);
    }
  else if (num == CN::MSTATUS or num == CN::SSTATUS or num == CN::VSSTATUS)
    {
      value &= csr->getPokeMask();
      value = legalizeMstatusValue(value);
    }
  else if (num == CN::TSELECT)
    {
      if (value >= triggers_.size())
	return true; // New value out of bounds. Preserve old.
    }

  csr->poke(value);

  if (num == CN::MENVCFG)
    {
      bool stce = menvcfgStce();
      enableHenvcfgStce(stce); // MENVCFG.STCE off makes HENVCFG.STCE read-only zero.

      bool pbmte = menvcfgPbmte();
      enableHenvcfgPbmte(pbmte);

      bool adue = menvcfgAdue();
      enableHenvcfgAdue(adue);
    }
  if ((num >= CN::MHPMEVENT3 and num <= CN::MHPMEVENT31) or
      (num >= CN::MHPMEVENTH3 and num <= CN::MHPMEVENTH31))
    {
      updateCounterControl(num);
      if (cofEnabled_ and superEnabled_)
        {
          if (rv32_ and num >= CN::MHPMEVENTH3 and num <= CN::MHPMEVENTH31)
            updateScountovfValue(num);
          else if (not rv32_)
            updateScountovfValue(num);
        }
    }
  else if (num == CN::FFLAGS or num == CN::FRM or num == CN::FCSR)
    updateFcsrGroupForPoke(num, value);   // fflags and frm are parts of fcsr
  else if (num == CN::VXSAT or num == CN::VXRM or num == CN::VCSR)
    updateVcsrGroupForPoke(num, value); // fflags and frm are parts of fcsr
  else if (num == CN::MCOUNTEREN or num == CN::SCOUNTEREN or num == CN::HCOUNTEREN)
    updateCounterPrivilege();  // Reflect counter accessibility in user/supervisor.
  else if (num == CN::HVICTL)
    updateVirtInterruptCtl();
  else if (num == CN::MVIEN or num == CN::MVIP)
    updateShadowSie();
  else if (num == CN::TCONTROL)
    triggers_.enableMachineMode(tcontrolMte());

  // Cache interrupt enable.
  if (num == CN::MSTATUS)
    {
      MstatusFields<URV> fields(csr->read());
      interruptEnable_ = fields.bits_.MIE;
    }
  else if (num == CN::MCOUNTEREN or num == CN::SCOUNTEREN or num == CN::HCOUNTEREN)
    updateCounterPrivilege();  // Reflect counter accessibility in user/supervisor.
  else if (num == CN::HVICTL)
    updateVirtInterruptCtl();
  else
    hyperPoke(csr);    // Update hypervisor CSR aliased bits.

  if (num == CN::MENVCFG or num == CN::HENVCFG)
    updateSstc();

  return true;
}


template <typename URV>
bool
CsRegs<URV>::readTrigger(CsrNumber number, PrivilegeMode mode, URV& value) const
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

  if (number == CsrNumber::TINFO)
    return triggers_.readInfo(trigger, value);

  return false;
}


template <typename URV>
bool
CsRegs<URV>::writeTrigger(CsrNumber number, PrivilegeMode mode, URV value)
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

  if (number == CsrNumber::TINFO)
    return triggers_.writeInfo(trigger, dMode, value);

  return false;
}


template <typename URV>
bool
CsRegs<URV>::pokeTrigger(CsrNumber number, URV value)
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

  if (number == CsrNumber::TINFO)
    return triggers_.pokeInfo(trigger, value);

  return false;
}


template <typename URV>
bool
CsRegs<URV>::readTopi(CsrNumber number, URV& value) const
{
  using IC = InterruptCause;

  auto mip = getImplementedCsr(CsrNumber::MIP)->read();
  if (seiPin_)
    mip |= URV(1) << URV(IC::S_EXTERNAL);

  auto mie = getImplementedCsr(CsrNumber::MIE)->read();

  auto mideleg = getImplementedCsr(CsrNumber::MIDELEG);
  URV midelegMask = mideleg? mideleg->read() : 0;

  const uint8_t iidShift = 16;
  auto highest_prio = [](uint64_t bits) -> unsigned {
    using IC = InterruptCause;

    for (InterruptCause ic : { IC::M_EXTERNAL, IC::M_SOFTWARE, IC::M_TIMER,
                               IC::S_EXTERNAL, IC::S_SOFTWARE, IC::S_TIMER,
                               IC::G_EXTERNAL,
                               IC::VS_EXTERNAL, IC::VS_SOFTWARE, IC::VS_TIMER,
                               IC::LCOF } )
	{
          URV mask = URV(1) << unsigned(ic);
          if (bits & mask)
            return unsigned(ic);
        }
    return 0;
  };

  value = 0;
  if (number == CsrNumber::MTOPI)
    {
      // We can set IPRIO=1 for read-only zero iprio arrays
      unsigned iid = highest_prio(mip & mie & ~midelegMask);
      if (iid)
        value = (iid << iidShift) | 1;
      return true;
    }

  if (number == CsrNumber::STOPI or number == CsrNumber::VSTOPI)
    {
      auto hideleg = getImplementedCsr(CsrNumber::HIDELEG);
      URV hidelegMask = hideleg? hideleg->read() : 0;

      if (not virtMode_)
        {
          unsigned iid = highest_prio(mip & mie & midelegMask & ~hidelegMask);
          if (iid)
            value = (iid << iidShift) | 1;
          return true;
        }

      auto vs = mip & mie & midelegMask & hidelegMask & ~(URV(1) << unsigned(IC::G_EXTERNAL));
      bool external = (vs & (URV(1) << unsigned(IC::VS_EXTERNAL))) != 0;

      auto csr = getImplementedCsr(CsrNumber::HVICTL);
      HvictlFields hvictl = csr? csr->read() : 0;
      unsigned iprio = hvictl.bits_.IPRIO;
      unsigned dpr = hvictl.bits_.DPR;
      unsigned iid = hvictl.bits_.IID;
      unsigned sExternal = unsigned(IC::S_EXTERNAL);

      if (external)
        {
          unsigned id = 0;
          if (imsic_)
            {
              URV hsVal = regs_.at(size_t(CsrNumber::HSTATUS)).read();
              HstatusFields<URV> hsf(hsVal);
              unsigned vgein = hsf.bits_.VGEIN;

              if (not vgein or vgein >= imsic_->guestCount())
                return false;
              id = imsic_->guestTopId(vgein);
            }
          if (id != 0)
            value = (sExternal << iidShift) | id;
          else if (iid == sExternal and iprio != 0)
            value = (sExternal << iidShift) | iprio;
          else
            value = (sExternal << iidShift) | 256;
        }

      bool vti = hvictl.bits_.VTI;
      if (not vti and not value)
        {
          if (vs & (URV(1) << unsigned(IC::VS_SOFTWARE)))
            value = unsigned(IC::S_SOFTWARE) << iidShift; // TODO: read-only zero hivprio
          else if (vs & (URV(1) << unsigned(IC::VS_TIMER)))
            value = unsigned(IC::S_TIMER) << iidShift;
        }
      if (vti and iid != sExternal and not (dpr and value))
          // DPR solely determines priority between candidates
          value = (iid << iidShift) | iprio;

      return true;
    }

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

  CsrNumber cfgNum = advance(CsrNumber::PMPCFG0, cfgOffset);

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
	  URV mask = ~URV(0);
	  if (width >= pmpG_ - 1)
	    mask >>= (width - pmpG_ + 1);
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
      else
	{
	  unsigned aField = (nb >> 3) & 3;
	  if (aField == 2)   // NA4
	    {
	      // If G is >= 1 then NA4 is not selectable in the A field.
	      if (not pmpNa4_ or (pmpG_ != 0 and aField == 2))
		nb = (cb & 0x18) | (nb & ~0x18);  // Preserve A field.
	    }
	  else if (aField == 1)  // TOR
	    {
	      if (not pmpTor_)   // TOR not supported
		nb = (cb & 0x18) | (nb & ~0x18);  // Preserve A field.
	    }

	  // w=1 r=0 is not allowed: Preserve the xwr field.
	  if ((nb & 3) == 2)
	    {
	      nb = (cb & 7) | (nb & ~7);   // Preserve xwr field.
	    }
	}

      legal = legal | (URV(nb) << i*8);
    }

  return legal;
}


template <typename URV>
URV
CsRegs<URV>::legalizePmacfgValue(URV prev, URV next) const
{
  // If any of the fields are illegal, keep current value.

  uint64_t val = next;

  uint64_t n = val >> 58;
  if (n > 0 and n < 12)
    return prev;

#if 0
  unsigned reserved = (val >> 52) & 0x3f;  // Bits 57 to 52.
  if (reserved != 0)
    return prev;  // Reserved bits must be zero.
#endif

  bool read = (val & 1);       // bit 0
  bool write = (val & 2);      // bit 1
  bool exec = (val & 4);       // bit 2
  bool cacheable = val & 0x80; // Bit 7
  bool coherent = val & 0x100; // Bit 8, routing for IO.

  unsigned memType = (val >> 3) & 3;   // Bits 4:3
  bool io = memType != 0;

  unsigned amo = (val >> 5) & 3;   // Bits 6:5

  if (io)
    {
      if (write and !read and !exec)
	return prev;
      if (amo != 0)
	return prev;  // IO must be amo-none.
      if (write and not read)
	return prev;  // Cannot have write without read.
      if (coherent)
	return prev;  // IO routing constraint.
    }
  else
    {
      // Either RWX or no access.
      unsigned count = read + write + exec;
      if (count != 0 and count != 3)
	return prev;

      if (cacheable and amo != 3)
	return prev;   // Cacheable must be amo-arithmetic.
      if (not cacheable and amo != 0)
	return prev;   // Non-cacheable must be amo-none.
      if (cacheable and not coherent)
	return prev;
    }

  return next;
}


template <typename URV>
void
CsRegs<URV>::updateScountovfValue(CsrNumber mhpmNum)
{
  using CN = CsrNumber;

  auto scountovf = getImplementedCsr(CN::SCOUNTOVF);
  if (not scountovf)
    {
      assert(0);
      return;
    }

  auto mhpm = getImplementedCsr(mhpmNum);
  if (not mhpm)
    {
      assert(0);
      return;
    }

  URV value = mhpm->read();
  bool of = value >> (8*sizeof(URV) - 1);

  URV ix = 3;
  if (rv32_)
    {
      assert(mhpmNum >= CN::MHPMEVENTH3 and mhpmNum <= CN::MHPMEVENTH31);
      ix += uint32_t(mhpmNum) - uint32_t(CN::MHPMEVENTH3);
    }
  else
    {
      assert(mhpmNum >= CN::MHPMEVENT3 and mhpmNum <= CN::MHPMEVENT31);
      ix += uint32_t(mhpmNum) - uint32_t(CN::MHPMEVENT3);
    }

  URV mask = ~ (1 << ix);
  URV prev = scountovf->read() & mask;
  scountovf->poke(of << ix | prev);
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

  CsrNumber csrn2 = advance(csrn, 1);
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

  URV hMask = 0;
  peek(CsrNumber::HCOUNTEREN, hMask);

  // Bits 0, 1, 2, 3 to 31 of mask correspond to CYCLE, TIME, INSTRET,
  // HPMCOUNTER3 to HPMCOUNTER31
  for (unsigned i = 0; i < 32; ++i)
    {
      bool mFlag = (mMask >> i) & 1;
      PrivilegeMode nextMode = PrivilegeMode::Machine;
      bool virtAccess = false;

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

          // from the spec, if counter is visible from VU, by effect it will also be
          // visible from U i.e. if a counter is visible from U and VS, then it must also
          // be visible to VU.
          if (hyperEnabled_)
            virtAccess = (hMask >> i) & 1;
        }

      unsigned num = i + unsigned(CsrNumber::CYCLE);

      CsrNumber csrn = CsrNumber(num);
      auto csr = getImplementedCsr(csrn);
      if (csr)
        {
          csr->setPrivilegeMode(nextMode);
          csr->setHypervisor(!virtAccess);
        }

      csrn = advance(CsrNumber::CYCLEH, i);
      csr = getImplementedCsr(csrn);
      if (csr)
        {
          csr->setPrivilegeMode(nextMode);
          csr->setHypervisor(!virtAccess);
        }
    }

  // Both STCE and TM control (v)stimecmp accessability.
  bool stce = menvcfgStce(); mMask &= (stce << 1);
  bool hstce = henvcfgStce(); hMask &= (hstce << 1);

  auto stimecmp = getImplementedCsr(CsrNumber::STIMECMP);
  auto stimecmph = getImplementedCsr(CsrNumber::STIMECMPH);
  for (auto csr : {stimecmp, stimecmph})
    {
      if (csr)
        {
          if ((mMask & 2) == 0)  // TM bit clear in mcounteren.
            csr->setPrivilegeMode(PrivilegeMode::Machine);
          else if (superEnabled_)
            csr->setPrivilegeMode(PrivilegeMode::Supervisor);
	  if (hyperEnabled_)
	    {
	      bool noVs = (mMask & 2) == 2 and (hMask & 2) == 0;
	      csr->setHypervisor(noVs);  // Not accessible from VS
	    }
        }
    }

  auto vstimecmp = getImplementedCsr(CsrNumber::VSTIMECMP);
  auto vstimecmph = getImplementedCsr(CsrNumber::VSTIMECMPH);
  for (auto csr : {vstimecmp, vstimecmph})
    {
      if (csr)
        {
          if ((mMask & 2) == 0)  // TM bit clear in mcounteren.
            csr->setPrivilegeMode(PrivilegeMode::Machine);
          else if (superEnabled_)
            csr->setPrivilegeMode(PrivilegeMode::Supervisor);
        }
    }
}


template <typename URV>
void
CsRegs<URV>::updateVirtInterruptCtl()
{
  URV val;
  peek(CsrNumber::HVICTL, val);
  HvictlFields hvictl(val);
  bool vti = hvictl.bits_.VTI;

  auto csr = getImplementedCsr(CsrNumber::VSIP);
  if (csr)
    csr->setHypervisor(not vti);
  csr = getImplementedCsr(CsrNumber::VSIPH);
  if (csr)
    csr->setHypervisor(not vti);
  csr = getImplementedCsr(CsrNumber::VSIE);
  if (csr)
    csr->setHypervisor(not vti);
  csr = getImplementedCsr(CsrNumber::VSIEH);
  if (csr)
    csr->setHypervisor(not vti);
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
     {"zero", 1}, {"LCOFIE", 1}, {"zero", xlen - 14}});
  setCsrFields(CsrNumber::MIP,
    {{"zero", 1}, {"SSIP", 1}, {"zero", 1}, {"MSIP", 1},
     {"zero", 1}, {"STIP", 1}, {"zero", 1}, {"MTIP", 1},
     {"zero", 1}, {"SEIP", 1}, {"zero", 1}, {"MEIP", 1},
     {"zero", 1}, {"LCOFIP", 1}, {"zero", xlen - 14}});
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
         {{"res1", 4}, {"SBE", 1}, {"MBE", 1}, {"GVA",   1},
	  {"MPV",  1}, {"res0", 24}});
      setCsrFields(CsrNumber::MENVCFG,
        {{"FIOM", 1}, {"res0",  3}, {"CBIE", 2}, {"CBCFE", 1},
         {"CBZE", 1}, {"res1", 24}});
      setCsrFields(CsrNumber::MENVCFGH,
        {{"PMM", 2}, {"res0", 28}, {"PBMTE", 1}, {"STCE", 1}});
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
         {"GVA",  1},  {"MPV",  1}, {"res0", 23}, {"SD",   1}});
      setCsrFields(CsrNumber::MENVCFG,
        {{"FIOM", 1}, {"res0",  3}, {"CBIE", 2}, {"CBCFE", 1},
         {"CBZE", 1}, {"res1", 24}, {"PMM",  2}, {"res2", 27},
         {"ADUE", 1}, {"PBMTE", 1}, {"STCE", 1}});
    }

  unsigned pmpIx = 0;
  for (unsigned i = 0; i < 16; i += 2)
    {
      std::vector<typename Csr<URV>::Field> pmps;

      if (rv32_)
        {
          CsrNumber csrNum = advance(CsrNumber::PMPCFG0, i + 1);
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
          CsrNumber csrNum = advance(CsrNumber::PMPCFG0, i);
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
      CsrNumber csrNum = advance(CsrNumber::PMPADDR0, i);
      if (rv32_)
        setCsrFields(csrNum, {{"addr", 32}});
      else
        setCsrFields(csrNum, {{"addr", 54}, {"zero", 10}});
    }

  for (unsigned i = 0; i < 16; ++i)
    {
      CsrNumber csrNum;
      csrNum = advance(CsrNumber::PMACFG0, i);
      setCsrFields(csrNum,
      {{"permission",         3},     {"memtype",   2}, {"amotype", 2},
       {"cache_or_combining", 1},     {"coherency", 1}, {"res1",    3},
       {"pa",                44},     {"res0",      2}, {"size",    6}});
    }

  for (unsigned i = 3; i <= 31; ++i)
    {
      CsrNumber csrNum = advance(CsrNumber::MHPMCOUNTER3, i - 3);
      std::string name = "mhpmcounter" + std::to_string(i);
      setCsrFields(csrNum, {{name, xlen}});
      if (rv32_)
        {
          // High register counterpart of mhpmcounter.
          csrNum = advance(CsrNumber::MHPMCOUNTER3H, i - 3);
          name += "h";
          setCsrFields(csrNum, {{name, xlen}});
        }

      csrNum = advance(CsrNumber::MHPMEVENT3, i - 3);
      name = "mhpmevent" + std::to_string(i);
      if (rv32_)
        {
          setCsrFields(csrNum, {{name, xlen}});

          // High register counterpart of mhpmevent
          csrNum = advance(CsrNumber::MHPMEVENTH3, i - 3);
          name += "h";
        }
      setCsrFields(csrNum,
        {{name, xlen - 8}, {"res", 2}, {"VUINH", 1}, {"VSINH", 1},
        {"UINH", 1}, {"SINH", 1}, {"MINH", 1}, {"OF", 1}});
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
     {"zero", 3}, {"SEIE", 1}, {"zero", 3}, {"LCOFIE", 1},
     {"zero", xlen - 14}});
  setCsrFields(CsrNumber::SIP,
    {{"zero", 1}, {"SSIP", 1}, {"zero", 3}, {"STIP", 1},
     {"zero", 3}, {"SEIP", 1}, {"zero", 3}, {"LCOFIP", 1},
     {"zero", xlen - 14}});

  if (rv32_)
    {
      setCsrFields(CsrNumber::SSTATUS,
        {{"res0", 1}, {"SIE",  1}, {"res1",  3}, {"SPIE", 1},
         {"UBE",  1}, {"res2", 1}, {"SPP",   1}, {"VS",   2},
         {"res3", 2}, {"FS",   2}, {"XS",    2}, {"res4", 1},
         {"SUM",  1}, {"MXR",  1}, {"res5", 11}, {"SD",   1}});
      setCsrFields(CsrNumber::SATP,
        {{"PPN", 22}, {"ASID", 9}, {"MODE", 1}});
      setCsrFields(CsrNumber::SENVCFG,
        {{"FIOM", 1}, {"res0", 3}, {"CBIE", 2}, {"CBCFE", 1},
         {"CBZE", 1}, {"res1", xlen - 8}});
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
      setCsrFields(CsrNumber::SENVCFG,
        {{"FIOM", 1}, {"res0",  3}, {"CBIE", 2}, {"CBCFE", 1},
         {"CBZE", 1}, {"res1", 24}, {"PMM",  2}, {"res2", xlen - 34}});
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
      CsrNumber csrNum = advance(CsrNumber::HPMCOUNTER3, i - 3);
      std::string name = "hpmcounter" + std::to_string(i);
      setCsrFields(csrNum, {{name, xlen}});
      if (rv32_)
        {
          // High register counterpart of hpmcounter.
          csrNum = advance(CsrNumber::HPMCOUNTER3H, i - 3);
          name += "h";
          setCsrFields(csrNum, {{name, xlen}});
        }
    }

  setCsrFields(CsrNumber::SRMCFG,
      {{"RCID", 12}, {"res0", 4}, {"MCID", 12}, {"res1", xlen - 28}});
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
CsRegs<URV>::addHypervisorFields()
{
  using Csrn = CsrNumber;
  constexpr unsigned xlen = sizeof(URV)*8;
  // TODO: fix this to compile-time fields
  setCsrFields(Csrn::HEDELEG, {{"hedeleg", xlen}});
  setCsrFields(Csrn::HIDELEG, {{"hideleg", xlen}});
  setCsrFields(Csrn::HVIP,
      {{"zero", 2}, {"VSSIP", 1}, {"zero", 3}, {"VSTIP", 1},
       {"zero", 3}, {"VSEIP", 1}, {"zero", 5}});
  setCsrFields(Csrn::HIE,
      {{"zero", 2}, {"VSSIE", 1}, {"zero", 3}, {"VSTIE", 1}, {"zero", 3},
       {"VSEIE", 1}, {"zero", 1}, {"SGEIE", 1}, {"zero", xlen - 14}});
  setCsrFields(Csrn::HIP,
      {{"zero", 2}, {"VSSIP", 1}, {"zero", 3}, {"VSTIP", 1}, {"zero", 3},
       {"VSEIP", 1}, {"zero", 1}, {"SGEIP", 1}, {"zero", xlen - 14}});
  setCsrFields(Csrn::HGEIE, {{"zero", 1}, {"hgeie", xlen - 1}});
  setCsrFields(Csrn::HGEIP, {{"zero", 1}, {"hgeip", xlen - 1}});

  std::vector<typename Csr<URV>::Field> hcount = {{"CY", 1}, {"TM", 1}, {"IR", 1}};
  std::vector<typename Csr<URV>::Field> hpm;
  for (unsigned i = 3; i <= 31; ++i)
    hpm.push_back({"HPM" + std::to_string(i), 1});
  hcount.insert(hcount.end(), hpm.begin(), hpm.end());
  setCsrFields(Csrn::HCOUNTEREN, hcount);
  setCsrFields(Csrn::HTIMEDELTA, {{"htimedelta", xlen}});
  setCsrFields(Csrn::HTVAL, {{"htval", xlen}});
  setCsrFields(Csrn::HTINST, {{"htinst", xlen}});

  setCsrFields(Csrn::VSTVEC, {{"MODE", 2}, {"BASE", xlen - 2}});
  setCsrFields(Csrn::VSSCRATCH, {{"sscratch", xlen}});
  setCsrFields(Csrn::VSEPC, {{"sepc", xlen}});
  setCsrFields(Csrn::VSCAUSE, {{"CODE", xlen - 1}, {"INT", 1}});
  setCsrFields(Csrn::VSTVAL, {{"stval", xlen}});
  setCsrFields(Csrn::VSIE,
    {{"zero", 1}, {"SSIE", 1}, {"zero", 3}, {"STIE", 1},
     {"zero", 3}, {"SEIE", 1}, {"zero", xlen - 10}});
  setCsrFields(Csrn::VSIP,
    {{"zero", 1}, {"SSIP", 1}, {"zero", 3}, {"STIP", 1},
     {"zero", 3}, {"SEIP", 1}, {"zero", xlen - 10}});

  setCsrFields(Csrn::MTVAL2, {{"mtval2", xlen}});
  setCsrFields(Csrn::MTINST, {{"mtinst", xlen}});

  std::vector<typename Csr<URV>::Field> fields;
  if (getCsrFields(Csrn::MIP, fields))
    {
      fields.at(2) = {"VSSIP", 1};
      fields.at(6) = {"VSTIP", 1};
      fields.at(10) = {"VSEIP", 1};
      setCsrFields(Csrn::MIP, fields);
    }

  if (getCsrFields(Csrn::MIE, fields))
    {
      fields.at(2) = {"VSSIE", 1};
      fields.at(6) = {"VSTIE", 1};
      fields.at(10) = {"VSEIE", 1};
      setCsrFields(Csrn::MIE, fields);
    }

  if (rv32_)
    {
      setCsrFields(Csrn::HSTATUS,
        {{"res0", 5}, {"VSBE", 1}, {"GVA", 1},   {"SPV", 1},  {"SPVP", 1},
         {"HU", 1},   {"res1", 2}, {"VGEIN", 6}, {"res2", 2}, {"VTVM", 1},
         {"VTW", 1},  {"VTSR", 1}, {"res3", 9}});
      setCsrFields(Csrn::HENVCFG,
        {{"FIOM", 1}, {"res0", 3}, {"CBIE", 2}, {"CBCFE", 1}, {"CBZE", 1},
         {"res1", xlen - 8}});
      setCsrFields(Csrn::HENVCFGH,
        {{"PMM", 2}, {"res0", 28}, {"PBMTE", 1}, {"VSTCE", 1}});
      setCsrFields(Csrn::HTIMEDELTAH, {{"htimedeltah", xlen}});
      setCsrFields(Csrn::HGATP,
        {{"PPN", 22}, {"VMID", 7}, {"zero", 2}, {"MODE", 1}});
      setCsrFields(Csrn::VSSTATUS,
        {{"res0", 1}, {"SIE",  1}, {"res1",  3}, {"SPIE", 1},
         {"UBE",  1}, {"res2", 1}, {"SPP",   1}, {"VS",   2},
         {"res3", 2}, {"FS",   2}, {"XS",    2}, {"res4", 1},
         {"SUM",  1}, {"MXR",  1}, {"res5", 11}, {"SD",   1}});
      setCsrFields(Csrn::VSATP,
        {{"PPN", 22}, {"ASID", 9}, {"MODE", 1}});
    }
  else
    {
      setCsrFields(Csrn::HSTATUS,
        {{"res0", 5}, {"VSBE", 1}, {"GVA", 1},   {"SPV", 1},  {"SPVP", 1},
         {"HU", 1},   {"res1", 2}, {"VGEIN", 6}, {"res2", 2}, {"VTVM", 1},
         {"VTW", 1},  {"VTSR", 1}, {"res3", 9},  {"VSXL", 2}, {"res4", 14},
         {"HUPMM", 2}, {"res5", 14}});
      setCsrFields(Csrn::HENVCFG,
        {{"FIOM",  1}, {"res0", 3}, {"CBIE", 2}, {"CBCFE", 1}, {"CBZE", 1},
         {"res1", 24}, {"PMM", 2}, {"res2", 27}, {"ADUE",  1}, {"PBMTE", 1},
         {"VSTCE", 1}});
      setCsrFields(Csrn::HGATP,
        {{"PPN", 44}, {"VMID", 14}, {"zero", 2}, {"MODE", 4}});
      setCsrFields(Csrn::VSSTATUS,
        {{"res0",  1}, {"SIE",  1}, {"res1",  3}, {"SPIE",  1},
         {"UBE",   1}, {"res2", 1}, {"SPP",   1}, {"VS",    2},
         {"res3",  2}, {"FS",   2}, {"XS",    2}, {"res4",  1},
         {"SUM",   1}, {"MXR",  1}, {"res5", 12}, {"UXL",   2},
         {"res6", 29}, {"SD",   1}});
      setCsrFields(Csrn::VSATP,
        {{"PPN", 44}, {"ASID", 16}, {"MODE", 4}});
    }
}


template <typename URV>
void
CsRegs<URV>::addAiaFields()
{
  using Csrn = CsrNumber;
  constexpr unsigned xlen = sizeof(URV)*8;

  setCsrFields(Csrn::MTOPEI,
      {{"prio", 11}, {"identity", 11}, {"zero", xlen - 22}});
  setCsrFields(Csrn::STOPEI,
      {{"prio", 11}, {"identity", 11}, {"zero", xlen - 22}});
  setCsrFields(Csrn::VSTOPEI,
      {{"prio", 11}, {"identity", 11}, {"zero", xlen - 22}});
}

template <typename URV>
void
CsRegs<URV>::addDebugFields()
{
  using Csrn = CsrNumber;
  constexpr unsigned xlen = sizeof(URV)*8;

  setCsrFields(Csrn::TDATA1,
      {{"data", xlen - 5}, {"dmode", 1}, {"ttype", 4}});
  setCsrFields(Csrn::TCONTROL,
      {{"zero", 3}, {"mte", 1}, {"zero", 3}, {"zero", xlen - 8}});

  if (rv32_)
    {
      setCsrFields(Csrn::TINFO,
          {{"info", 16}, {"zero", 8}, {"version", 8}});
    }
  else
    {
      setCsrFields(Csrn::TINFO,
          {{"info", 16}, {"zero", 8}, {"version", 8}, {"zero", xlen - 32}});
    }
}


template <typename URV>
void
CsRegs<URV>::hyperWrite(Csr<URV>* csr)
{
  if (not hyperEnabled_)
    return;

  auto num = csr->getNumber();
  auto value = csr->read();

  auto hip = getImplementedCsr(CsrNumber::HIP);
  auto hvip = getImplementedCsr(CsrNumber::HVIP);
  auto mip = getImplementedCsr(CsrNumber::MIP);
  auto vsip = getImplementedCsr(CsrNumber::VSIP);
  auto vsie = getImplementedCsr(CsrNumber::VSIE);
  auto hideleg = getImplementedCsr(CsrNumber::HIDELEG);

  URV prevHipVal = hip->prevValue();

  URV hieMask = 0x1444; // SGEIE, VSEIE, VSTIE and VSSIE.

  auto updateCsr = [this](Csr<URV>* csr, URV val) {
    if (csr and csr->read() != val)
      {
	csr->poke(val);
	recordWrite(csr->getNumber());
      }
  };

  if (num == CsrNumber::HIDELEG)
    {
      assert(hideleg);
      URV mask = 0x222;   // Bits VSEIP, VSTIP, and VSSIP of VSIP
      mask &= (hideleg->read() >> 1);
      if (vsip)
	vsip->setReadMask(mask);
      if (vsie)
	vsie->setReadMask(mask);
    }
  else if (num == CsrNumber::MIP)
    {
      // Updating MIP is reflected into HIP/VSIP.
      URV val = mip->read() & hieMask;
      hip->poke(val | (hip->read() & ~hieMask));
      updateCsr(vsip, val >> 1);
    }
  else if (num == CsrNumber::HIP)
    {
      // Updating HIP is reflected into MIP/VSIP.
      URV val = hip->read() & hieMask;
      updateCsr(mip, val | (mip->read() & ~hieMask));
      updateCsr(vsip, val >> 1);
    }
  else if (num == CsrNumber::HVIP)
    {
      // Writing HVIP injects values into HIP. FIX : Need to logical-or external values
      // for VSEIP and VSTIP.
      if (hip)
	{
	  // Bit 10 (VSEIP) of HIP is the or of bit 10 of HVIP and HGEIP bit
	  // selected by GVEIN.
	  URV hsVal = regs_.at(size_t(CsrNumber::HSTATUS)).read();
	  HstatusFields<URV> hsf(hsVal);
	  unsigned vgein = hsf.bits_.VGEIN;
	  URV hgeipVal = regs_.at(size_t(CsrNumber::HGEIP)).read();
	  unsigned bit = (hgeipVal >> vgein) & 1;  // Bit of HGEIP selected by VGEIN
	  value = value | (bit << 10);  // Or HGEIP bit selected by GVEIN.
	  hip->poke(value);
	}

      updateCsr(vsip, value >> 1);
    }
  else if (num == CsrNumber::HGEIP or num == CsrNumber::HSTATUS)
    {
      // Updating HGEIP or HSTATUS.VGEIN is reflected in HIP
      if (hip)
        {
	  URV hsVal = regs_.at(size_t(CsrNumber::HSTATUS)).read();
	  HstatusFields<URV> hsf(hsVal);
	  unsigned vgein = hsf.bits_.VGEIN;
	  URV hgeipVal = regs_.at(size_t(CsrNumber::HGEIP)).read();
	  unsigned bit = (hgeipVal >> vgein) & 1;  // Bit of HGEIP selected by VGEIN
	  // Update bit VSEIP (10) of HIP.
	  hip->poke(hip->read() & ~(URV(1) << 10));  // Clear bit 10 of HIP
	  URV mask = bit << 10;
	  if (hvip)
	    mask |= (hvip->read() & (1 << 10));  // Or bit 10 of HVIP.
          hip->poke(hip->read() | mask);  // Set HIP bit 10 to or of HVIP and HGEIP bits.
        }
    }
  else if (num == CsrNumber::VSIP)
    {
      // Updating VSIP injects values into writeable bits of HIP
      if (hip)
        {
	  value = value << 1;  // Aliased bits shifted in HIP
	  if (hideleg)
	    {
	      URV newVal = (hip->read() & ~hideleg->read()) | (value & hideleg->read());
	      hip->poke(newVal);
	    }
	  else
	    hip->poke(value);
        }
    }

  bool hipUpdated = false;
  if (hip->read() != prevHipVal)
    {
      hipUpdated = true;
      recordWrite(CsrNumber::HIP);
    }

  if (hipUpdated)
    {
      // Writing HIP changes bit VSSIP in HVIP.
      if (hvip and num != CsrNumber::HVIP)
	{
	  URV mask = 0x4; // Bit VSSIP
	  URV newVal = (hip->read() & mask) | (hvip->read() & ~mask);
	  updateCsr(hvip, newVal);
	}

      // Updating HIP is reflected in VSIP.
      if (vsip and num != CsrNumber::VSIP)
	{
	  URV val = hip->read() & ~ URV(0x1000);  // Clear bit 12 (SGEIP)
	  updateCsr(vsip, val >> 1);
	}

      // Updating HIP is reflected in MIP.
      if (mip and num != CsrNumber::MIP)
	{
	  URV newVal = (mip->read() & ~hieMask) | (hip->read() & hieMask);
	  updateCsr(mip, newVal);
	}
    }

  auto hie = getImplementedCsr(CsrNumber::HIE);
  auto mie = getImplementedCsr(CsrNumber::MIE);
  if (num == CsrNumber::HIE)
    {
      // Updating HIE is reflected into MIE/VSIE.
      URV val = hie->read() & hieMask;
      URV mieVal = (mie->read() & ~hieMask) | val;
      updateCsr(mie, mieVal);
      updateCsr(vsie, val >> 1);
    }
  else if (num == CsrNumber::MIE)
    {
      // Updating MIE is reflected into HIE/VSIE.
      URV val = (mie->read() & hieMask);
      URV hieVal = val | (hie->read() & ~hieMask);
      updateCsr(hie, hieVal);
      updateCsr(vsie, val >> 1);
    }
  else if (num == CsrNumber::VSIE)
    {
      URV vsMask = 0x444;  // VSEIP, VSTIP and VSSIP.
      if (hideleg)
	vsMask = vsMask & hideleg->read();

      URV val = ((vsie->read() << 1) & vsMask);
      URV mieVal = (mie->read() & ~vsMask) | val;
      updateCsr(mie, mieVal);

      URV hieVal = (hie->read() & ~vsMask) | val;
      updateCsr(hie, hieVal);
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
  auto vsip = getImplementedCsr(CsrNumber::VSIP);
  auto hideleg = getImplementedCsr(CsrNumber::HIDELEG);

  bool hipUpdated = num == CsrNumber::HIP;
  URV hieMask = 0x1444; // SGEIE, VSEIE, VSTIE and VSSIE.

  if (num == CsrNumber::MIP)
    {
      // Updating MIP is reflected into HIP/VSIP.
      URV val = mip->read() & hieMask;
      if (hip)
	{
	  hip->poke(val | (hip->read() & ~hieMask));
	  hipUpdated = true;
	}
      if (vsip)
	{
	  if (hideleg)
	    val &= hideleg->read();
	  vsip->poke(val >> 1);
	}
    }
  else if (num == CsrNumber::HIP)
    {
      URV val = hip->read() & hieMask;
      // Updating HIP is reflected into MIP/VSIP.
      if (mip)
	{
	  mip->poke(val | (mip->read() & ~hieMask));
	}
      if (vsip)
	{
	  if (hideleg)
	    val &= hideleg->read();
	  vsip->poke(val >> 1);
	}
    }
  else if (num == CsrNumber::HVIP)
    {
      // Poking HVIP injects values into HIP. FIX : Need to
      // logical-or external values for VSEIP and VSTIP.
      if (hip)
        {
	  URV hsVal = regs_.at(size_t(CsrNumber::HSTATUS)).read();
	  HstatusFields<URV> hsf(hsVal);
	  unsigned vgein = hsf.bits_.VGEIN;
	  URV hgeipVal = regs_.at(size_t(CsrNumber::HGEIP)).read();
	  unsigned bit = (hgeipVal >> vgein) & 1;  // Bit of HGEIP selected by VGEIN
	  value = value | (bit << 10);
          hip->poke(value);
          hipUpdated = true;
        }
      if (vsip)
	{
	  if (hideleg)
	    value &= hideleg->read();
	  vsip->poke(value >> 1);
	}
    }
  else if (num == CsrNumber::HGEIP or num == CsrNumber::HSTATUS)
    {
      // Updating HGEIP or HSTATUS.VGEIN is reflected in HIP
      // FIX: only do this when VGEIN is updated
      if (hip)
        {
	  URV hsVal = regs_.at(size_t(CsrNumber::HSTATUS)).read();
	  HstatusFields<URV> hsf(hsVal);
	  unsigned vgein = hsf.bits_.VGEIN;
	  URV hgeipVal = regs_.at(size_t(CsrNumber::HGEIP)).read();
	  unsigned bit = (hgeipVal >> vgein) & 1;  // Bit of HGEIP selected by VGEIN
	  // Update bit VSEIP (10) of HIP.
	  hip->poke(hip->read() & ~(URV(1) << 10));  // Clear bit 10 of HIP
	  URV mask = bit << 10;
          hip->poke(hip->read() | mask);  // Set bit 10 to HGEIP bit selected by VGEIN.
          hipUpdated = true;
        }
    }
  else if (num == CsrNumber::VSIP)
    {
      // Updating VSIP injects values into writeable bits of HIP
      if (hip)
        {
	  value = value << 1;  // Aliased bits shifter in HIP
	  if (hideleg)
	    {
	      URV newVal = (hip->read() & ~hideleg->read()) | (value & hideleg->read());
	      hip->poke(newVal);
	    }
	  else
	    hip->poke(value);
          hipUpdated = true;
        }
    }

  if (hipUpdated)
    {
      // Writing HIP changes bit VSSIP in HVIP.
      if (hvip and num != CsrNumber::HVIP)
	{
	  URV mask = 0x4; // Bit VSSIP
	  URV newVal = (hip->read() & mask) | (hvip->read() & ~mask);
	  hvip->poke(newVal);
	}

      // Updating HIP is reflected in VSIP.
      if (vsip and num != CsrNumber::VSIP)
	{
	  URV val = hip->read() & ~ URV(0x1000);  // Clear bit 12 (SGEIP)
	  vsip->poke(val >> 1);
	}

      // Updating HIP is reflected in MIP.
      if (mip and num != CsrNumber::MIP)
	{
	  URV newVal = (mip->read() & ~hieMask) | (hip->read() & hieMask);
	  mip->poke(newVal);
	}
    }

  auto hie = getImplementedCsr(CsrNumber::HIE);
  auto mie = getImplementedCsr(CsrNumber::MIE);
  auto vsie = getImplementedCsr(CsrNumber::VSIE);
  if (num == CsrNumber::HIE)
    {
      URV val = hie->read() & hieMask;
      if (mie)
        mie->poke((mie->read() & ~hieMask) | val);
      if (vsie)
	{
	  if (hideleg)
	    val &= hideleg->read();
          vsie->poke(val >> 1);
	}
    }
  else if (num == CsrNumber::MIE)
    {
      URV val = (mie->read() & hieMask);
      if (hie)
        hie->poke(val | (hie->read() & ~hieMask));
      if (vsie)
        {
	  if (hideleg)
	    val &= hideleg->read();
          vsie->poke(val >> 1);
        }
    }
  else if (num == CsrNumber::VSIE)
    {
      URV vsMask = 0x444;  // VSEIP, VSTIP and VSSIP.
      if (hideleg)
	vsMask = vsMask & hideleg->read();
      URV val = ((vsie->read() << 1) & vsMask);
      if (mie)
        mie->poke((mie->read() & ~vsMask) | val);
      if (hie)
        hie->poke(val | (hie->read() & ~vsMask));
    }
}


template <typename URV>
bool
CsRegs<URV>::isStateEnabled(CsrNumber num, PrivilegeMode pm, bool vm) const
{
  if (not stateenOn_ or pm == PrivilegeMode::Machine)
    return true;

  using CN = CsrNumber;
  // sstateen not applicable for now
  CN csrn = rv32_? CN::MSTATEEN0H : CN::MSTATEEN0;
  if (vm)
    csrn = rv32_? CN::HSTATEEN0H : CN::HSTATEEN0;

  int enableBit = -1;
  unsigned offset = 0;
  if (num == CN::SRMCFG)
    enableBit = 55;
  if (num == CN::HCONTEXT or num == CN::SCONTEXT)
    enableBit = 57;
  // hgeip hgeie
  else if (num == CN::MISELECT or num == CN::MIREG or num == CN::MTOPEI or num == CN::MTOPI or
	   num == CN::MVIEN or num == CN::MVIP or num == CN::MIDELEGH or num == CN::MIEH or
	   num == CN::MVIENH or num == CN::MVIPH or num == CN::MIPH or num == CN::STOPEI or
	   num == CN::VSTOPEI)
    enableBit = 58;  // IMSIC state.
  if (num == CN::SIPH or num == CN::SIEH or num == CN::STOPI or num == CN::HIDELEGH or
      num == CN::HVIEN or num == CN::HVIENH or num == CN::HVIPH or num == CN::HVICTL or
      num == CN::HVIPRIO1 or num == CN::HVIPRIO1H or num == CN::HVIPRIO2 or
      num == CN::HVIPRIO2H or num == CN::VSIPH or num == CN::VSIEH or num == CN::VSTOPI)
    enableBit = 59;  // AIA state not controlled by bits 58 and 60
  else if (num == CN::SISELECT or num == CN::SIREG or num == CN::VSISELECT or num == CN::VSIREG)
    enableBit = 60;
  else if (num == CN::SIREG)
    {
      URV select = 0;
      if (peek(CN::SISELECT, select))
	{
	  if (select >= 0x30 and select <= 0x3f)
	    enableBit = 59;  // Sections 2.5 and 5.4.1 of AIA
	}
    }
  else if (num == CN::HENVCFG or num == CN::HENVCFGH or num == CN::SENVCFG)
    enableBit = 62;
  else if (num >= CN::HSTATEEN0 and num <= CN::HSTATEEN3)
    {
      enableBit = 63;
      offset = unsigned(num) - unsigned(CN::HSTATEEN0);
    }
  else if (num >= CN::HSTATEEN0H and num <= CN::HSTATEEN3H)
    {
      enableBit = 63;
      offset = unsigned(num) - unsigned(CN::HSTATEEN0H);
    }
  else if (num >= CN::SSTATEEN0 and num <= CN::SSTATEEN3)
    {
      enableBit = 63;
      offset = unsigned(num) - unsigned(CN::SSTATEEN0);
    }

  if (enableBit < 0)
    return true;  // CSR not affected by STATEEN

  csrn = advance(csrn, offset);
  auto csr = getImplementedCsr(csrn);
  if (not csr)
    return true;

  if (rv32_) enableBit -= 8*sizeof(URV);
  URV value = csr->read();
  return (value >> enableBit) & 1;
}


template class WdRiscv::CsRegs<uint32_t>;
template class WdRiscv::CsRegs<uint64_t>;
