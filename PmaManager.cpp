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
Pma::stringToAttrib(std::string_view str, Pma::Attrib& attrib)
{
  static const std::unordered_map<std::string_view, Attrib> stringToAttrib = {
    { "none", Pma::None },
    { "read", Pma::Read },
    { "write", Pma::Write },
    { "exec", Pma::Exec },
    { "idempotent", Pma::Idempotent },
    { "amoswap", Pma::AmoSwap },
    { "amological", Pma::AmoLogical },
    { "amoother", Pma::AmoOther },
    { "amoarithmetic", Pma::AmoArith },
    { "amo", Pma::AmoArith },
    { "mem_mapped", Pma::MemMapped },
    { "rsrv", Pma::Rsrv },
    { "io", Pma::Io },
    { "cacheable", Pma::Cacheable },
    { "misal_ok", Pma::MisalOk },
    { "misal_acc_fault", Pma::MisalAccFault }
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


std::string
Pma::attributesToString(uint32_t attrib)
{
  std::string result;

  result += (attrib & Pma::None)? "none," : "";
  result += (attrib & Pma::Read)? "read," : "";
  result += (attrib & Pma::Write)? "write," : "";
  result += (attrib & Pma::Exec)? "exec," : "";
  result += (attrib & Pma::Idempotent)? "idempotent," : "";
  result += (attrib & Pma::AmoOther)? "amoother," : "";
  result += (attrib & Pma::AmoSwap)? "amoswap," : "";
  result += (attrib & Pma::AmoLogical)? "amological," : "";
  result += (attrib & Pma::MemMapped)? "memmapped," : "";
  result += (attrib & Pma::Rsrv)? "rsrv," : "";
  result += (attrib & Pma::Io)? "io," : "";
  result += (attrib & Pma::Cacheable)? "cacheable," : "";
  result += (attrib & Pma::MisalOk)? "misalok," : "";
  result += (attrib & Pma::MisalAccFault)? "misalaccfault," : "";
  return result;
}


PmaManager::PmaManager(uint64_t memSize)
  : memSize_(memSize)
{
  noAccessPma_.enable(Pma::Attrib::MisalOk);
  regions_.reserve(32);
}


bool
PmaManager::setMemMappedMask(uint64_t addr, uint64_t mask, unsigned size)
{
  if (size != 4 and size != 8)
    return false;

  if ((addr & (size - 1)) != 0)
    return false;   // Not aligned.

  if (not isMemMappedReg(addr))
    return false;

  memMappedRegs_[addr].mask_ = mask;
  memMappedRegs_[addr].size_ = size;
  return true;
}


uint64_t
PmaManager::getMemMappedMask(uint64_t addr) const
{
  auto iter = memMappedRegs_.find(addr);
  if (iter == memMappedRegs_.end())
    return ~uint64_t(0);
  return iter->second.mask_;
}


bool
PmaManager::readRegister(uint64_t addr, uint32_t& value) const
{
  if ((addr & 3) != 0)
    return false;  // Not word aligned.

  auto iter = memMappedRegs_.find(addr);
  if (iter == memMappedRegs_.end())
    return false;

  if (iter->second.size_ != 4)
    return false;

  value = iter->second.value_;
  return true;
}


bool
PmaManager::readRegister(uint64_t addr, uint64_t& value) const
{
  if ((addr & 7) != 0)
    return false;   // Not double-word aligned.

  auto iter = memMappedRegs_.find(addr);
  if (iter == memMappedRegs_.end())
    return false;

  if (iter->second.size_ != 8)
    return false;

  value = iter->second.value_;
  return true;
}


bool
PmaManager::writeRegister(uint64_t addr, uint64_t value)
{
  auto iter = memMappedRegs_.find(addr);
  if (iter == memMappedRegs_.end())
    return false;

  iter->second.value_ = value & iter->second.mask_;
  return true;
}


bool
PmaManager::checkRegisterWrite(uint64_t addr, unsigned size) const
{
  auto iter = memMappedRegs_.find(addr);
  if (iter == memMappedRegs_.end())
    return false;

  return iter->second.size_ == size;
}


bool
PmaManager::pokeRegister(uint64_t addr, uint64_t value)
{
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
  auto iter = memMappedRegs_.find(addr);
  if (iter == memMappedRegs_.end())
    return false;

  unsigned shift = byteIx * 8;
  uint32_t byteMask = 0xff << shift;
  uint32_t shiftedByte = (uint64_t(value) << shift) & iter->second.mask_;
  iter->second.value_ = iter->second.value_ & ~byteMask;
  iter->second.value_ = iter->second.value_ | shiftedByte;
  return true;
}


bool
PmaManager::pokeRegisterByte(uint64_t addr, uint8_t value)
{
  unsigned byteIx = addr & 3;
  addr = (addr >> 2) << 2;
  auto iter = memMappedRegs_.find(addr);
  if (iter == memMappedRegs_.end())
    return false;

  unsigned shift = byteIx * 8;
  uint32_t byteMask = 0xff << shift;
  uint32_t shiftedByte = uint64_t(value) << shift;
  iter->second.value_ = iter->second.value_ & ~byteMask;
  iter->second.value_ = iter->second.value_ | shiftedByte;
  return true;
}


void
PmaManager::printRegion(std::ostream& os, Region region) const
{
  const auto& pma = region.pma_;
  os << "valid: " << std::hex << region.valid_ << "\n";

  if (not region.valid_)
    return;

  os << std::hex;
  os << "base addr: 0x" << region.firstAddr_ << "\n";
  os << "last addr: 0x" << region.lastAddr_ << "\n";
  os << std::dec;

  os << "attributes: " << Pma::attributesToString(pma.attrib_) << "\n";
}


void
PmaManager::printPmas(std::ostream& os, uint64_t address) const
{
  auto region = getRegion(address);
  printRegion(os, region);
}


void
PmaManager::printPmas(std::ostream& os) const
{
  for (size_t i = 0; i < regions_.size(); ++i)
    {
      os << "Region " << i << '\n';
      auto& region = regions_.at(i);
      printRegion(os, region);
      os << '\n';
    }
}
