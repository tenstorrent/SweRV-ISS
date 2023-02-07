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
  typedef PrivilegeMode PM;

  if (not isRvh())
    {
      illegalInst(di);
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);
      return;
    }

  if (privMode_ == PM::User or (privMode_ == PM::Supervisor and mstatus_.bits_.TVM == 1))
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

  if (not isRvh())
    {
      illegalInst(di);
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);
      return;
    }

  if (privMode_ == PM::User or (privMode_ == PM::Supervisor and mstatus_.bits_.TVM == 1))
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
  if (load<int8_t>(virtAddr, true /*hyper*/, data))
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
  if (load<uint8_t>(virtAddr, true /*hyper*/, data))
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
  if (load<int16_t>(virtAddr, true /*hyper*/, data))
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
  if (load<uint16_t>(virtAddr, true /*hyper*/, data))
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
  if (load<int32_t>(virtAddr, true /*hyper*/, data))
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
  if (load<uint32_t>(virtAddr, true /*hyper*/, data))
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
  if (load<uint16_t>(virtAddr, true /*hyper*/, data))
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
  if (load<uint32_t>(virtAddr, true /*hyper*/, data))
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
  if (load<uint64_t>(virtAddr, true /*hyper*/, data))
    intRegs_.write(di->op0(), data);
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
  store<uint8_t>(addr, true /*hyper*/, value);
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
  store<uint16_t>(addr, true /*hyper*/, value);
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
  store<uint32_t>(addr, true /*hyper*/, value);
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
  store<uint64_t>(addr, true /*hyper*/, value);
}


template <typename URV>
void
Hart<URV>::execHinval_vvma(const DecodedInst* di)
{
  execHfence_vvma(di);
}


template <typename URV>
void
Hart<URV>::execHinval_gvma(const DecodedInst* di)
{
  execHfence_gvma(di);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
