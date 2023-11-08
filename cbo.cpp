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
Hart<URV>::determineCboException(uint64_t addr, uint64_t& gpa, uint64_t& pa, bool isRead)
{
  uint64_t mask = uint64_t(cacheLineSize_) - 1;
  addr &= ~mask;  // Make addr a multiple of cache line size.

  addr = URV(addr);   // Truncate to 32 bits in 32-bit mode.

  ExceptionCause cause = ExceptionCause::NONE;

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
	  bool read = isRead, write = not isRead, exec = false;
          cause = virtMem_.translate(addr, pm, virt, read, write, exec, gpa, pa);
          if (cause != ExceptionCause::NONE)
            return cause;
        }
    }

  // Physical memory protection.
  if (pmpEnabled_)
    {
      assert((cacheLineSize_ % 8) == 0);
      if (isRead)
	{
	  for (uint64_t i = 0; i < cacheLineSize_; i += 8)
	    {
	      uint64_t dwa = pa + i*8;  // Double word address
	      Pmp pmp = pmpManager_.accessPmp(dwa, PmpManager::AccessReason::LdSt);
	      if (not pmp.isRead(effectivePrivilege()))
		return ExceptionCause::LOAD_ACC_FAULT;
	    }
	}
      else
	{
	  for (uint64_t i = 0; i < cacheLineSize_; i += 8)
	    {
	      uint64_t dwa = pa + i*8;  // Double word address
	      Pmp pmp = pmpManager_.accessPmp(dwa, PmpManager::AccessReason::LdSt);
	      if (not pmp.isWrite(effectivePrivilege()))
		return ExceptionCause::STORE_ACC_FAULT;
	    }
	}
    }

  return ExceptionCause::NONE;
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
  SenvcfgFields<URV> senvf(peekCsr(CsrNumber::SENVCFG));

  if ( (pm != PM::Machine and not menvf.bits_.CBCFE) or
       (pm == PM::User and not senvf.bits_.CBCFE) )
    {
      illegalInst(di);
      return;
    }

  uint64_t virtAddr = intRegs_.read(di->op0());
  uint64_t gPhysAddr = virtAddr;
  uint64_t physAddr = virtAddr;
  bool isRead = false;

  auto cause = determineCboException(virtAddr, gPhysAddr, physAddr, isRead);
  if (cause != ExceptionCause::NONE)
    {
      uint64_t mask = uint64_t(cacheLineSize_) - 1;
      initiateStoreException(cause, virtAddr & ~mask, gPhysAddr & ~mask);
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
  SenvcfgFields<URV> senvf(peekCsr(CsrNumber::SENVCFG));

  if ( (pm != PM::Machine and not menvf.bits_.CBCFE) or
       (pm == PM::User and not senvf.bits_.CBCFE) )
    {
      illegalInst(di);
      return;
    }

  uint64_t virtAddr = intRegs_.read(di->op0());
  uint64_t gPhysAddr = virtAddr;
  uint64_t physAddr = virtAddr;
  bool isRead = false;

  auto cause = determineCboException(virtAddr, gPhysAddr, physAddr, isRead);
  if (cause != ExceptionCause::NONE)
    {
      uint64_t mask = uint64_t(cacheLineSize_) - 1;
      initiateStoreException(cause, virtAddr & ~mask, gPhysAddr & ~mask);
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
  SenvcfgFields<URV> senvf(peekCsr(CsrNumber::SENVCFG));

  if ( (pm != PM::Machine and not menvf.bits_.CBIE) or
       (pm == PM::User and not senvf.bits_.CBIE) )
    {
      illegalInst(di);
      return;
    }

  // If we are doing a flush then we require write access. If we are
  // doing an invalidate then we only require read access.
  bool isFlush = ( (pm != PM::Machine and menvf.bits_.CBIE) or
		   (pm == PM::User and senvf.bits_.CBIE) );
  bool isRead = not isFlush;

  uint64_t virtAddr = intRegs_.read(di->op0());
  uint64_t gPhysAddr = virtAddr;
  uint64_t physAddr = virtAddr;

  auto cause = determineCboException(virtAddr, gPhysAddr, physAddr, isRead);
  if (cause != ExceptionCause::NONE)
    {
      uint64_t mask = uint64_t(cacheLineSize_) - 1;
      initiateStoreException(cause, virtAddr & ~mask, gPhysAddr & ~mask);
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
  SenvcfgFields<URV> senvf(peekCsr(CsrNumber::SENVCFG));

  if ( (pm != PM::Machine and not menvf.bits_.CBZE) or
       (pm == PM::User and not senvf.bits_.CBZE) )
    {
      illegalInst(di);
      return;
    }

  // Translate virtual addr and check for exception.
  uint64_t virtAddr = intRegs_.read(di->op0());
  uint64_t mask = uint64_t(cacheLineSize_) - 1;
  virtAddr = virtAddr & ~mask;  // Make address cache line aligned.

  uint64_t gPhysAddr = virtAddr;
  uint64_t physAddr = virtAddr;

  bool isRead = false;
  auto cause = determineCboException(virtAddr, gPhysAddr, physAddr, isRead);
  if (cause != ExceptionCause::NONE)
    {
      initiateStoreException(cause, virtAddr, gPhysAddr);
      return;
    }

  for (unsigned i = 0; i < cacheLineSize_; i+= 8)
    pokeMemory(physAddr + i, uint64_t(0), true /*usePma*/);
}

template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
