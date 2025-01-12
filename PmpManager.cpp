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
#include <fstream>
#include "PmpManager.hpp"

using namespace WdRiscv;


PmpManager::PmpManager(uint64_t /*memSize*/, uint64_t /*sectionSize*/)
{
}


PmpManager::~PmpManager() = default;


void
PmpManager::reset()
{
  regions_.clear();
  fastRegion_.region_ = nullptr;
}


void
PmpManager::defineRegion(uint64_t a0, uint64_t a1, Pmp::Type type,
			 Pmp::Mode mode, unsigned pmpIx, bool lock)
{
  a0 = (a0 >> 2) << 2;   // Make word aligned.
  a1 = (a1 >> 2) << 2;   // Make word aligned.

  Pmp pmp(mode, pmpIx, lock, type);
  Region region{a0, a1, pmp};
  regions_.push_back(region);
}


void
PmpManager::printRegion(std::ostream& os, Region region) const
{
  const auto& pmp = region.pmp_;
  os << "pmp ix: " << std::dec << pmp.pmpIndex() << "\n";
  os << "base addr: " << std::hex << region.firstAddr_ << "\n";
  os << "last addr: " << std::hex << region.lastAddr_ << "\n";

  os << "rwx: " << Pmp::toString(Pmp::Mode(pmp.mode_)) << "\n";
  os << "matching: " << Pmp::toString(Pmp::Type(pmp.type_)) << "\n";
}


void
PmpManager::printPmps(std::ostream& os, uint64_t addr) const
{
  auto region = getRegion(addr);
  printRegion(os, region);
}


void
PmpManager::printPmps(std::ostream& os) const
{
  for (const auto& region : regions_)
    printRegion(os, region);
}


std::string
Pmp::toString(Pmp::Type type)
{
  switch (type)
    {
    case Pmp::Type::Off:   return "off";
    case Pmp::Type::Tor:   return "tor";
    case Pmp::Type::Na4:   return "na4";
    case Pmp::Type::Napot: return "napot";
    default:               return "?";
    }
  return "";
}


std::string
Pmp::toString(Pmp::Mode mode)
{
  std::string result;

  result += (mode & Mode::Read)  ? "r" : "-";
  result += (mode & Mode::Write) ? "w" : "-";
  result += (mode & Mode::Exec)  ? "x" : "-";

  return result;
}
