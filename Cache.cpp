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


#include <algorithm>
#include <bit>
#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include "Cache.hpp"

using namespace WdRiscv;


Cache::Cache(uint64_t totalSize, unsigned lineSize, unsigned setSize)
  : size_(totalSize), lineSize_(lineSize), setSize_(setSize)
{
  assert(std::has_single_bit(totalSize));
  assert(std::has_single_bit(setSize));
  assert(std::has_single_bit(lineSize));

  lineNumberShift_ = std::bit_width(lineSize) - 1;

  assert(totalSize >= lineSize);

  uint64_t lineCount = totalSize / lineSize;
  assert(lineCount >= setSize);

  uint64_t count = lineCount / setSize;
  assert(std::has_single_bit(count));

  setIndexMask_ = count - 1;

  linesPerSet_.resize(count);

  for (auto& lines : linesPerSet_)
    lines.resize(setSize_);
}


Cache::~Cache()
{
  std::cerr << "Cache access: " << accesses_ << '\n';
  std::cerr << "Cache hits: " << hits_ << '\n';

  double ratio = accesses_ == 0? 0. : double(hits_)/double(accesses_);
  std::cerr << "Hit ratio: " << ratio << '\n';
}


void
Cache::getLineAddresses(std::vector<uint64_t>& result) const
{
  result.clear();

  std::vector<Entry> entries;

  for (const auto& lines : linesPerSet_)
    for (const auto& entry : lines)
      if (entry.valid())
        entries.push_back(entry);

  std::sort(entries.begin(), entries.end(),
            [](const Entry& a, const Entry& b) {
              return a.time_ < b.time_;
            } );

  result.reserve(entries.size());
  for (const auto& entry : entries)
    result.push_back(entry.tag_ << lineNumberShift_);
}


bool
Cache::saveSnapshot(const std::string& path)
{
  std::ofstream ofs(path, std::ios::trunc);

  if (not ofs)
    {
      std::cerr << "Cache::saveSnapshot failed - cannot open " << path << " for write\n";
      return false;
    }

  std::vector<uint64_t> vec;
  getLineAddresses(vec);

  ofs << std::hex;

  for (auto& addr : vec)
    ofs << "0x" << addr << '\n';

  ofs << std::dec;

  if (not ofs)
    {
      std::cerr << "Cache::saveSnapshot failed to write all line addresses\n";
      return false;
    }

  return true;
}


bool
Cache::loadSnapshot(const std::string& path)
{
  std::ifstream ifs(path);
  if (not ifs)
    {
      std::cerr << "Cache::loadSnapshot failed - cannot open " << path << " for read\n";
      return false;
    }

  uint64_t addr = 0;
  while (ifs)
    {
      if (ifs >> std::hex >> addr)
        insert(addr);
      else if (ifs.eof())
        break;
      else
        {
          std::cerr << "Cache::loadSnapshot failed to load " << path << "\n";
          return false;
        }
    }

  return true;
}
