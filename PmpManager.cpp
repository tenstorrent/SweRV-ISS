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
  if (pmpIx >= accessCount_.size())
    {
      accessCount_.resize(pmpIx + 1);
      typeCount_.resize(pmpIx + 1);
    }
}


bool
PmpManager::printStats(std::ostream& out) const
{
  out << "PMP entry access count:\n";
  for (size_t i = 0; i < accessCount_.size(); ++i)
    out << "  entry " << i << ": " << accessCount_.at(i) << '\n';

  out << "PMP type count:\n";
  for (size_t i = 0; i < typeCount_.size(); ++i)
    {
      out << "  ";
      switch(i)
        {
        case Pmp::Type::Off:   out << "OFF:   "; break;
        case Pmp::Type::Tor:   out << "TOR:   "; break;
        case Pmp::Type::Na4:   out << "NA4:   "; break;
        case Pmp::Type::Napot: out << "NAPOT: "; break;
        default:               out << "??:    "; break;
        }
      out << typeCount_.at(i) << '\n';
    }

  return true;
}


bool
PmpManager::printStats(const std::string& path) const
{
  std::ofstream out(path);
  if (not out.good())
    {
      std::cerr << "Failed to open file '" << path << "' for output\n";
      return false;
    }

  return printStats(out);
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
