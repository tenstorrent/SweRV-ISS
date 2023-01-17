// Copyright 2022 Tenstorretn Corporation.
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
//stributed under the License isstributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <climits>
#include <cassert>

#include "instforms.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"
#include "Mcm.hpp"

using namespace WdRiscv;


template <typename URV>
void
Hart<URV>::execHfence_vvma(const DecodedInst* di)
{
  if (not isRvh() or not isRvs() or privMode_ < PrivilegeMode::Supervisor or virtMode_)
    {
      illegalInst(di);
      return;
    }

  auto tlb = virtMem_.stage1Tlb_;

  // Invalidate whole VS TLB. This is overkill. 
  if (di->op1() == 0 and di->op2() == 0)
       tlb.invalidate();
  else if (di->op1() == 0 and di->op2() != 0)
    {
      URV asid = intRegs_.read(di->op2());
      tlb.invalidateAsid(asid);
    }
  else if (di->op1() != 0 and di->op2() == 0)
    {
      URV addr = intRegs_.read(di->op1());
      uint64_t vpn = virtMem_.pageNumber(addr);
      tlb.invalidateVirtualPage(vpn);
    }
  else
    {
      URV addr = intRegs_.read(di->op1());
      uint64_t vpn = virtMem_.pageNumber(addr);
      URV asid = intRegs_.read(di->op2());
      tlb.invalidateVirtualPage(vpn, asid);
    }

  invalidateDecodeCache();
}


template <typename URV>
void
Hart<URV>::execHfence_gvma(const DecodedInst* di)
{
  typedef PrivilegeMode PM;
  bool valid = isRvh();
  if (privMode_ == PM::User)
    valid = false;
  else if (privMode_ == PM::Supervisor)
    if (mstatus_.bits_.TVM != 0 or virtMode_)
      valid = false;
  if (not valid)
    {
      illegalInst(di);
      return;
    }

  auto tlb = virtMem_.stage2Tlb_;

  // Invalidate whole VS TLB. This is overkill. 
  if (di->op1() == 0 and di->op2() == 0)
       tlb.invalidate();
  else if (di->op1() == 0 and di->op2() != 0)
    {
      URV vmid = intRegs_.read(di->op2());
      tlb.invalidateVmid(vmid);
    }
  else if (di->op1() != 0 and di->op2() == 0)
    {
      URV addr = intRegs_.read(di->op1());
      uint64_t vpn = virtMem_.pageNumber(addr);
      tlb.invalidateVirtualPage(vpn);
    }
  else
    {
      URV addr = intRegs_.read(di->op1());
      uint64_t vpn = virtMem_.pageNumber(addr);
      URV asid = intRegs_.read(di->op2());
      tlb.invalidateVirtualPage(vpn, asid);
    }

  invalidateDecodeCache();
}


template <typename URV>
template <typename LOAD_TYPE>
bool
Hart<URV>::hyperLoad(uint64_t virtAddr, uint64_t& data)
{
  ldStAddr_ = virtAddr;   // For reporting ld/st addr in trace-mode.
  ldStPhysAddr1_ = ldStPhysAddr2_ = virtAddr;
  ldStSize_ = sizeof(LOAD_TYPE);

  if (hasActiveTrigger())
    {
      if (ldStAddrTriggerHit(virtAddr, TriggerTiming::Before, true /*isLoad*/))
	triggerTripped_ = true;
    }

  // Unsigned version of LOAD_TYPE
  typedef typename std::make_unsigned<LOAD_TYPE>::type ULT;

  uint64_t addr1 = virtAddr;
  uint64_t addr2 = addr1;
  auto cause = determineLoadException(addr1, addr2, ldStSize_, true /*hyper*/);
  if (cause != ExceptionCause::NONE)
    {
      if (triggerTripped_)
        return false;
      initiateLoadException(cause, addr1);
      return false;
    }
  ldStPhysAddr1_ = addr1;
  ldStPhysAddr2_ = addr2;

  // Loading from console-io does a standard input read.
  if (conIoValid_ and addr1 == conIo_ and enableConIn_ and not triggerTripped_)
    {
      SRV val = fgetc(stdin);
      data = val;
      return true;
    }

  if (toHostValid_ and addr1 == toHost_)
    {
      data = 0;
      return true;
    }

  ULT narrow = 0;   // Unsigned narrow loaded value
  if (addr1 >= clintStart_ and addr1 < clintLimit_ and addr1 - clintStart_ >= 0xbff8)
    {
      uint64_t tm = instCounter_ >> counterToTimeShift_; // Fake time: instr count
      tm = tm >> (addr1 - 0xbff8) * 8;
      narrow = tm;
    }
  else
    {
      bool hasMcmVal = false;
      if (mcm_)
	{
	  uint64_t mcmVal = 0;
	  if (mcm_->getCurrentLoadValue(*this, addr1, ldStSize_, mcmVal))
	    {
	      narrow = mcmVal;
	      hasMcmVal = true;
	    }
	}
      if (not hasMcmVal)
	memRead(addr1, addr2, narrow);
    }

  data = narrow;
  if (not std::is_same<ULT, LOAD_TYPE>::value)
    data = int64_t(LOAD_TYPE(narrow)); // Loading signed: Sign extend.

  if (initStateFile_)
    {
      dumpInitState("load", virtAddr, addr1);
      if (addr1 != addr2 or memory_.getLineNumber(addr1) != memory_.getLineNumber(addr1 + ldStSize_))
	dumpInitState("load", virtAddr + ldStSize_, addr2 + ldStSize_);
    }

  // Check for load-data-trigger.
  if (hasActiveTrigger())
    {
      TriggerTiming timing = TriggerTiming::Before;
      bool isLoad = true;
      if (ldStDataTriggerHit(narrow, timing, isLoad))
	triggerTripped_ = true;
    }
  if (triggerTripped_)
    return false;

  return true;  // Success.
}


template <typename URV>
void
Hart<URV>::execHlv_b(const DecodedInst* di)
{
  if (not isRvh())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }
      
  if (privMode_ == PrivilegeMode::User and hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();
  uint64_t data = 0;
  if (hyperLoad<int8_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
void
Hart<URV>::execHlv_bu(const DecodedInst* di)
{
  if (not isRvh())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }
      
  if (privMode_ == PrivilegeMode::User and hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();
  uint64_t data = 0;
  if (hyperLoad<uint8_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
void
Hart<URV>::execHlv_h(const DecodedInst* di)
{
  if (not isRvh())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }
      
  if (privMode_ == PrivilegeMode::User and hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();
  uint64_t data = 0;
  if (hyperLoad<int16_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
void
Hart<URV>::execHlv_hu(const DecodedInst* di)
{
  if (not isRvh())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }
      
  if (privMode_ == PrivilegeMode::User and hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();
  uint64_t data = 0;
  if (hyperLoad<uint16_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
void
Hart<URV>::execHlv_w(const DecodedInst* di)
{
  if (not isRvh())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }
      
  if (privMode_ == PrivilegeMode::User and hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();
  uint64_t data = 0;
  if (hyperLoad<int32_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
void
Hart<URV>::execHlv_wu(const DecodedInst* di)
{
  if (not isRvh() or not isRv64())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }
      
  if (privMode_ == PrivilegeMode::User and hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();
  uint64_t data = 0;
  if (hyperLoad<uint32_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
void
Hart<URV>::execHlvx_hu(const DecodedInst* di)
{
  if (not isRvh())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }
      
  if (privMode_ == PrivilegeMode::User and hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  virtMem_.useExecForRead(true);

  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();
  uint64_t data = 0;
  if (hyperLoad<uint16_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);

  virtMem_.useExecForRead(false);
}


template <typename URV>
void
Hart<URV>::execHlvx_wu(const DecodedInst* di)
{
  if (not isRvh())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }
      
  if (privMode_ == PrivilegeMode::User and hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  virtMem_.useExecForRead(true);

  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();
  uint64_t data = 0;
  if (hyperLoad<uint32_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);

  virtMem_.useExecForRead(false);
}


template <typename URV>
void
Hart<URV>::execHlv_d(const DecodedInst* di)
{
  if (not isRvh() or not isRv64())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }
      
  if (privMode_ == PrivilegeMode::User and hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();
  uint64_t data = 0;
  if (hyperLoad<int64_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
template <typename STORE_TYPE>
inline
bool
Hart<URV>::hyperStore(URV virtAddr, STORE_TYPE storeVal)
{
  typedef PrivilegeMode PM;
  PM mode = hstatus_.bits_.SPVP ? PM::Supervisor : PM::User;

  assert(virtAddr);
  assert(mode != PrivilegeMode::Machine);
  return false;
}


template <typename URV>
void
Hart<URV>::execHsv_b(const DecodedInst* di)
{
  if (not isRvh())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }
      
  if (privMode_ == PrivilegeMode::User and hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  uint32_t rs1 = di->op1();
  URV base = intRegs_.read(rs1);
  URV addr = base + di->op2As<SRV>();
  uint8_t value = uint8_t(intRegs_.read(di->op0()));
  hyperStore<uint8_t>(addr, value);
}


template <typename URV>
void
Hart<URV>::execHsv_h(const DecodedInst* di)
{
  if (not isRvh())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }
      
  if (privMode_ == PrivilegeMode::User and hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  uint32_t rs1 = di->op1();
  URV base = intRegs_.read(rs1);
  URV addr = base + di->op2As<SRV>();
  uint16_t value = uint8_t(intRegs_.read(di->op0()));
  hyperStore<uint8_t>(addr, value);
}


template <typename URV>
void
Hart<URV>::execHsv_w(const DecodedInst* di)
{
  if (not isRvh())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }
      
  if (privMode_ == PrivilegeMode::User and hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  uint32_t rs1 = di->op1();
  URV base = intRegs_.read(rs1);
  URV addr = base + di->op2As<SRV>();
  uint32_t value = uint8_t(intRegs_.read(di->op0()));
  hyperStore<uint8_t>(addr, value);
}


template <typename URV>
void
Hart<URV>::execHsv_d(const DecodedInst* di)
{
  if (not isRvh() or not isRv64())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }
      
  if (privMode_ == PrivilegeMode::User and hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  uint32_t rs1 = di->op1();
  URV base = intRegs_.read(rs1);
  URV addr = base + di->op2As<SRV>();
  uint8_t value = uint8_t(intRegs_.read(di->op0()));
  hyperStore<uint64_t>(addr, value);
}


template <typename URV>
void
Hart<URV>::execHinval_vvma(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execHinval_gvma(const DecodedInst*)
{
  assert(0);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
