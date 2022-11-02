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

#include "IntRegs.hpp"


using namespace WdRiscv;


template <typename URV>
IntRegs<URV>::IntRegs(unsigned regCount)
  : regs_(regCount, 0)
{
}


template <typename URV>
bool
IntRegs<URV>::findReg(const std::string& name, unsigned& ix) const
{
  unsigned i = 0;
  if (not regNames_.findReg(name, i))
    return false;

  if (i >= regs_.size())
    return false;

  ix = i;
  return true;
}


template class WdRiscv::IntRegs<uint32_t>;
template class WdRiscv::IntRegs<uint64_t>;
