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


using namespace WdRiscv;


template <typename URV>
void
Hart<URV>::execHfence_vvma(const DecodedInst* di)
{
  if (not isRvs() or privMode_ < PrivilegeMode::Supervisor or virtMode_)
    {
      illegalInst(di);
      return;
    }

  assert(0);
}


template <typename URV>
void
Hart<URV>::execHfence_gvma(const DecodedInst* di)
{
  bool valid = (privMode_ == PrivilegeMode::Machine or
		(privMode_ == PrivilegeMode::Supervisor and not virtMode_));
  if (not valid)
    {
      illegalInst(di);
      return;
    }

  assert(0);
}


template <typename URV>
template <typename LOAD_TYPE>
bool
Hart<URV>::hyperLoad(uint64_t addr, uint64_t& data)
{
  typedef PrivilegeMode PM;
  PM mode = hstatus_.bits_.SPVP ? PM::Supervisor : PM::User;

  assert(addr);
  assert(mode != PrivilegeMode::Machine);
  data = 0;
  return false;
}


template <typename URV>
void
Hart<URV>::execHlv_b(const DecodedInst* di)
{
  typedef PrivilegeMode PM;

  // Must not be in virtual mode, must have H extension, must not be in User mode
  // unless hstatus.hu is on.
  bool illegal = (virtMode_ or not isRvh() or
		  (privMode_ == PM::User and hstatus_.bits_.HU));
  if (illegal)
    illegalInst(di);

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
  typedef PrivilegeMode PM;

  // Must not be in virtual mode, must have H extension, must not be in User mode
  // unless hstatus.hu is on.
  bool illegal = (virtMode_ or not isRvh() or
		  (privMode_ == PM::User and hstatus_.bits_.HU));
  if (illegal)
    illegalInst(di);

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
  typedef PrivilegeMode PM;

  // Must not be in virtual mode, must have H extension, must not be in User mode
  // unless hstatus.hu is on.
  bool illegal = (virtMode_ or not isRvh() or
		  (privMode_ == PM::User and hstatus_.bits_.HU));
  if (illegal)
    illegalInst(di);

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
  typedef PrivilegeMode PM;

  // Must not be in virtual mode, must have H extension, must not be in User mode
  // unless hstatus.hu is on.
  bool illegal = (virtMode_ or not isRvh() or
		  (privMode_ == PM::User and hstatus_.bits_.HU));
  if (illegal)
    illegalInst(di);

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
  typedef PrivilegeMode PM;

  // Must not be in virtual mode, must have H extension, must not be in User mode
  // unless hstatus.hu is on.
  bool illegal = (virtMode_ or not isRvh() or
		  (privMode_ == PM::User and hstatus_.bits_.HU));
  if (illegal)
    illegalInst(di);

  URV base = intRegs_.read(di->op1());

  uint64_t virtAddr = base + di->op2As<int32_t>();
  uint64_t data = 0;
  if (hyperLoad<int32_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
void
Hart<URV>::execHlvx_hu(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execHlvx_wu(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execHlv_wu(const DecodedInst* di)
{
  typedef PrivilegeMode PM;

  // Must not be in virtual mode, must have H extension, must not be in User mode
  // unless hstatus.hu is on. Must be in 64-bit mode.
  bool illegal = (virtMode_ or not isRvh() or not isRv64() or
		  (privMode_ == PM::User and hstatus_.bits_.HU));
  if (illegal)
    illegalInst(di);

  URV base = intRegs_.read(di->op1());

  uint64_t virtAddr = base + di->op2As<int32_t>();
  uint64_t data = 0;
  if (hyperLoad<uint32_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
void
Hart<URV>::execHlv_d(const DecodedInst* di)
{
  typedef PrivilegeMode PM;

  // Must not be in virtual mode, must have H extension, must not be in User mode
  // unless hstatus.hu is on. Must be in 64-bit mode.
  bool illegal = (virtMode_ or not isRvh() or not isRv64() or
		  (privMode_ == PM::User and hstatus_.bits_.HU));
  if (illegal)
    illegalInst(di);

  URV base = intRegs_.read(di->op1());

  uint64_t virtAddr = base + di->op2As<int32_t>();
  uint64_t data = 0;
  if (hyperLoad<uint64_t>(virtAddr, data))
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
  typedef PrivilegeMode PM;

  // Must not be in virtual mode, must have H extension, must not be in User mode
  // unless hstatus.hu is on.
  bool illegal = (virtMode_ or not isRvh() or
		  (privMode_ == PM::User and hstatus_.bits_.HU));
  if (illegal)
    illegalInst(di);

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
  typedef PrivilegeMode PM;

  // Must not be in virtual mode, must have H extension, must not be in User mode
  // unless hstatus.hu is on.
  bool illegal = (virtMode_ or not isRvh() or
		  (privMode_ == PM::User and hstatus_.bits_.HU));
  if (illegal)
    illegalInst(di);

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
  typedef PrivilegeMode PM;

  // Must not be in virtual mode, must have H extension, must not be in User mode
  // unless hstatus.hu is on.
  bool illegal = (virtMode_ or not isRvh() or
		  (privMode_ == PM::User and hstatus_.bits_.HU));
  if (illegal)
    illegalInst(di);

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
  typedef PrivilegeMode PM;

  // Must not be in virtual mode, must have H extension, must not be in User mode
  // unless hstatus.hu is on. Must be in 64-bit mode.
  bool illegal = (virtMode_ or not isRvh() or not isRv64() or
		  (privMode_ == PM::User and hstatus_.bits_.HU));
  if (illegal)
    illegalInst(di);

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
}


template <typename URV>
void
Hart<URV>::execHinval_gvma(const DecodedInst*)
{
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
