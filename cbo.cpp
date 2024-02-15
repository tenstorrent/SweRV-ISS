// Copyright 2022 Tenstorrent Corporation.
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
#include <cassert>

#include "DecodedInst.hpp"
#include "Hart.hpp"

using namespace WdRiscv;


template <typename URV>
ExceptionCause
Hart<URV>::determineCboException(uint64_t addr, uint64_t& gpa, uint64_t& pa, bool isZero)
{
  uint64_t mask = uint64_t(cacheLineSize_) - 1;
  addr &= ~mask;  // Make addr a multiple of cache line size.

  addr = URV(addr);   // Truncate to 32 bits in 32-bit mode.

  using EC = ExceptionCause;

  EC cause = EC::NONE;

  // Address translation
  if (isRvs())
    {
      PrivilegeMode pm = privMode_;
      bool virt = virtMode_;
      if (mstatusMprv() and not nmieOverridesMprv())
	{
	  pm = mstatusMpp();
	  virt = mstatus_.bits_.MPV;
	}

      if (pm != PrivilegeMode::Machine)
        {
          gpa = pa = addr;
	  if (isZero)
	    {
	      bool read = false, write = true, exec = false;
	      cause = virtMem_.translate(addr, pm, virt, read, write, exec, gpa, pa);
	      if (cause != EC::NONE)
		return cause;
	    }
	  else
	    {
	      // If load or store is allowed CBO is allowed.
	      bool read = true, write = false, exec = false;
	      cause = virtMem_.translate(addr, pm, virt, read, write, exec, gpa, pa);
	      if (cause != EC::NONE)
		{
		  if (cause == EC::LOAD_ACC_FAULT)
		    return EC::STORE_ACC_FAULT;
		  if (cause == EC::LOAD_PAGE_FAULT)
		    return EC::STORE_PAGE_FAULT;
		  if (cause == EC::LOAD_GUEST_PAGE_FAULT)
		    return EC::STORE_GUEST_PAGE_FAULT;
		  return cause;
		}
	    }
        }
    }

  // Physical memory protection.
  if (pmpEnabled_)
    {
      assert((cacheLineSize_ % 8) == 0);
      auto ep = effectivePrivilege();

      for (uint64_t i = 0; i < cacheLineSize_; i += 8)
	{
	  uint64_t dwa = pa + i*8;  // Double word address
	  Pmp pmp = pmpManager_.accessPmp(dwa, PmpManager::AccessReason::LdSt);
	  if (isZero)
	    {
	      if (not pmp.isWrite(ep))
		return EC::STORE_ACC_FAULT;
	    }
	  else if (not pmp.isRead(ep) and not pmp.isWrite(ep))
	    return EC::STORE_ACC_FAULT;
	}
    }

  return EC::NONE;
}


template <typename URV>
void
Hart<URV>::execCbo_clean(const DecodedInst* di)
{
  if (not isRvzicbom())
    {
      illegalInst(di);
      return;
    }

  using PM = PrivilegeMode;
  PM pm = privilegeMode();

  MenvcfgFields<URV> menvf(peekCsr(CsrNumber::MENVCFG));
  SenvcfgFields<URV> senvf(isRvs()? peekCsr(CsrNumber::SENVCFG) : 0);
  HenvcfgFields<URV> henvf(isRvh()? peekCsr(CsrNumber::HENVCFG) : 0);

  if ( (pm != PM::Machine and not menvf.bits_.CBCFE) or
       (not virtMode_ and pm == PM::User and not senvf.bits_.CBCFE) )
    {
      illegalInst(di);
      return;
    }

  if (virtMode_)
    {
      if ( (pm == PM::Supervisor and not henvf.bits_.CBCFE) or
	   (pm == PM::User and not (henvf.bits_.CBCFE and senvf.bits_.CBCFE)) )
	{
	  virtualInst(di);
	  return;
	}
    }

  uint64_t virtAddr = intRegs_.read(di->op0());
  uint64_t gPhysAddr = virtAddr;
  uint64_t physAddr = virtAddr;
  bool isZero = false;

  auto cause = determineCboException(virtAddr, gPhysAddr, physAddr, isZero);
  if (cause != ExceptionCause::NONE)
    {
      uint64_t mask = uint64_t(cacheLineSize_) - 1;
      initiateStoreException(di, cause, virtAddr & ~mask, gPhysAddr & ~mask);
    }
}


template <typename URV>
void
Hart<URV>::execCbo_flush(const DecodedInst* di)
{
  if (not isRvzicbom())
    {
      illegalInst(di);
      return;
    }

  using PM = PrivilegeMode;
  PM pm = privilegeMode();

  MenvcfgFields<URV> menvf(peekCsr(CsrNumber::MENVCFG));
  SenvcfgFields<URV> senvf(isRvs()? peekCsr(CsrNumber::SENVCFG) : 0);
  HenvcfgFields<URV> henvf(isRvh()? peekCsr(CsrNumber::HENVCFG) : 0);

  if ( (pm != PM::Machine and not menvf.bits_.CBCFE) or
       (not virtMode_ and pm == PM::User and not senvf.bits_.CBCFE) )
    {
      illegalInst(di);
      return;
    }

  if (virtMode_)
    {
      if ( (pm == PM::Supervisor and not henvf.bits_.CBCFE) or
	   (pm == PM::User and not (henvf.bits_.CBCFE and senvf.bits_.CBCFE)) )
	{
	  virtualInst(di);
	  return;
	}
    }

  uint64_t virtAddr = intRegs_.read(di->op0());
  uint64_t gPhysAddr = virtAddr;
  uint64_t physAddr = virtAddr;
  bool isZero = false;

  auto cause = determineCboException(virtAddr, gPhysAddr, physAddr, isZero);
  if (cause != ExceptionCause::NONE)
    {
      uint64_t mask = uint64_t(cacheLineSize_) - 1;
      initiateStoreException(di, cause, virtAddr & ~mask, gPhysAddr & ~mask);
    }
}


template <typename URV>
void
Hart<URV>::execCbo_inval(const DecodedInst* di)
{
  if (not isRvzicbom())
    {
      illegalInst(di);
      return;
    }

  using PM = PrivilegeMode;
  PM pm = privilegeMode();

  MenvcfgFields<URV> menvf(peekCsr(CsrNumber::MENVCFG));
  SenvcfgFields<URV> senvf(isRvs()? peekCsr(CsrNumber::SENVCFG) : 0);
  HenvcfgFields<URV> henvf(isRvh()? peekCsr(CsrNumber::HENVCFG) : 0);

  if ( (pm != PM::Machine and menvf.bits_.CBIE == 0) or
       (not virtMode_ and pm == PM::User and senvf.bits_.CBIE == 0) )
    {
      illegalInst(di);
      return;
    }

  if (virtMode_)
    {
      if ( (pm == PM::Supervisor and henvf.bits_.CBIE == 0) or
	   (pm == PM::User and (henvf.bits_.CBIE == 0 or senvf.bits_.CBIE == 0)) )
	{
	  virtualInst(di);
	  return;
	}
    }


#if 0
  // If we are doing a flush then we require write access. If we are
  // doing an invalidate then we only require read access.
  bool isFlush = false;
  if (not virtMode_)
    {
      isFlush = ( (pm != PM::Machine and menvf.bits_.CBIE == 1) or
		  (pm == PM::User and senvf.bits_.CBIE == 1) );
    }
  else
    {
      isFlush = ( (pm == PM::Supervisor and henvf.bits_.CBIE == 1) or
		  (pm == PM::User and (henvf.bits_.CBIE == 1 or senvf.bits_.CBIE == 1)) );
    }
#endif

  bool isZero = false;

  uint64_t virtAddr = intRegs_.read(di->op0());
  uint64_t gPhysAddr = virtAddr;
  uint64_t physAddr = virtAddr;

  auto cause = determineCboException(virtAddr, gPhysAddr, physAddr, isZero);
  if (cause != ExceptionCause::NONE)
    {
      uint64_t mask = uint64_t(cacheLineSize_) - 1;
      initiateStoreException(di, cause, virtAddr & ~mask, gPhysAddr & ~mask);
    }
}


template <typename URV>
void
Hart<URV>::execCbo_zero(const DecodedInst* di)
{
  if (not isRvzicboz())
    {
      illegalInst(di);
      return;
    }

  using PM = PrivilegeMode;
  PM pm = privilegeMode();

  MenvcfgFields<URV> menvf(peekCsr(CsrNumber::MENVCFG));
  SenvcfgFields<URV> senvf(isRvs()? peekCsr(CsrNumber::SENVCFG) : 0);
  HenvcfgFields<URV> henvf(isRvh()? peekCsr(CsrNumber::HENVCFG) : 0);

  if ( (pm != PM::Machine and not menvf.bits_.CBZE) or
       (not virtMode_ and pm == PM::User and not senvf.bits_.CBZE) )
    {
      illegalInst(di);
      return;
    }

  if (virtMode_)
    {
      if ( (pm == PM::Supervisor and not henvf.bits_.CBZE) or
	   (pm == PM::User and not (henvf.bits_.CBZE and senvf.bits_.CBZE)) )
	{
	  virtualInst(di);
	  return;
	}
    }

  // Translate virtual addr and check for exception.
  uint64_t virtAddr = intRegs_.read(di->op0());
  uint64_t mask = uint64_t(cacheLineSize_) - 1;
  virtAddr = virtAddr & ~mask;  // Make address cache line aligned.

  uint64_t gPhysAddr = virtAddr;
  uint64_t physAddr = virtAddr;

  bool isZero = true;
  auto cause = determineCboException(virtAddr, gPhysAddr, physAddr, isZero);
  if (cause != ExceptionCause::NONE)
    {
      initiateStoreException(di, cause, virtAddr, gPhysAddr);
      return;
    }

  for (unsigned i = 0; i < cacheLineSize_; i+= 8)
    pokeMemory(physAddr + i, uint64_t(0), true /*usePma*/);
}


template <typename URV>
void
Hart<URV>::execPrefetch_i(const DecodedInst* di)
{
  if (not isRvzicbop())
    {
      illegalInst(di);
      return;
    }
}


template <typename URV>
void
Hart<URV>::execPrefetch_r(const DecodedInst* di)
{
  if (not isRvzicbop())
    {
      illegalInst(di);
      return;
    }
}


template <typename URV>
void
Hart<URV>::execPrefetch_w(const DecodedInst* di)
{
  if (not isRvzicbop())
    {
      illegalInst(di);
      return;
    }
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
