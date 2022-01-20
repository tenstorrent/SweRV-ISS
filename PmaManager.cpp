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
#include <cmath>
#include <cassert>
#include "PmaManager.hpp"

using namespace WdRiscv;


bool
Pma::stringToAttrib(const std::string& str, Pma::Attrib& attrib)
{
  static std::unordered_map<std::string, Attrib> stringToAttrib = {
    { "none", Pma::None },
    { "read", Pma::Read },
    { "write", Pma::Write },
    { "exec", Pma::Exec },
    { "idempotent", Pma::Idempotent },
    { "amo", Pma::Amo },
    { "iccm", Pma::Iccm },
    { "dccm", Pma::Dccm },
    { "mem_mapped", Pma::MemMapped },
    { "rsrv", Pma::Rsrv },
    { "Io", Pma::Io }
  };

  auto iter = stringToAttrib.find(str);
  if (iter != stringToAttrib.end())
    {
      attrib = iter->second;
      return true;
    }

  attrib = Pma::None;
  return false;
}

PmaManager::PmaManager(uint64_t /*memSize*/, uint64_t /*sectionSize*/)
{
}


bool
PmaManager::setMemMappedMask(uint64_t addr, uint32_t mask)
{
  addr = (addr >> 2) << 2;
  if (not isAddrMemMapped(addr))
    return false;
  memMappedRegs_[addr].mask_ = mask;
  return true;
}


uint32_t
PmaManager::getMemMappedMask(uint64_t addr) const
{
  addr = (addr >> 2) << 2;
  auto iter = memMappedRegs_.find(addr);
  if (iter == memMappedRegs_.end())
    return ~uint32_t(0);
  return iter->second.mask_;
}


bool
PmaManager::readRegister(uint64_t addr, uint32_t& value) const
{
  addr = (addr >> 2) << 2;
  auto iter = memMappedRegs_.find(addr);
  if (iter == memMappedRegs_.end())
    return false;
  value = iter->second.value_;
  return true;
}


bool
PmaManager::writeRegister(uint64_t addr, uint32_t value)
{
  addr = (addr >> 2) << 2;
  auto iter = memMappedRegs_.find(addr);
  if (iter == memMappedRegs_.end())
    return false;
  iter->second.value_ = value & iter->second.mask_;
  return true;
}


bool
PmaManager::writeRegisterNoMask(uint64_t addr, uint32_t value)
{
  addr = (addr >> 2) << 2;
  auto iter = memMappedRegs_.find(addr);
  if (iter == memMappedRegs_.end())
    return false;
  iter->second.value_ = value;
  return true;
}


bool
PmaManager::writeRegisterByte(uint64_t addr, uint8_t value)
{
  unsigned byteIx = addr & 3;
  addr = (addr >> 2) << 2;
  auto iter = memMappedRegs_.find(addr);
  if (iter == memMappedRegs_.end())
    return false;

  unsigned shift = byteIx * 8;
  uint32_t byteMask = 0xff << shift;
  uint32_t shiftedByte = (uint32_t(value) << shift) & iter->second.mask_;
  iter->second.value_ = iter->second.value_ & ~byteMask;
  iter->second.value_ = iter->second.value_ | shiftedByte;
  return true;
}
