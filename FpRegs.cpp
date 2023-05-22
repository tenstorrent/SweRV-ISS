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

#include "FpRegs.hpp"
#include "softfloat-util.hpp"

using namespace WdRiscv;


FpRegs::FpRegs(unsigned regCount)
  : regs_(regCount, UINT64_C(0))
{
}


bool
FpRegs::findReg(const std::string_view name, unsigned& ix) const
{
  unsigned i = 0;
  if (not FpRegNames::findReg(name, i))
    return false;

  if (i >= regs_.size())
    return false;

  ix = i;
  return true;
}


void
FpRegs::reset(bool hasHalf, bool hasSingle, bool hasDouble)
{
  hasHalf_ = hasHalf;
  hasSingle_ = hasSingle;
  hasDouble_ = hasDouble;

  if (hasDouble)
    {
      for (auto& reg : regs_)
	reg = UINT64_C(0);
    }
  else if (hasSingle)
    {
      // F extension present without D. Reset to NAN-boxed
      // single-precision zeros.
      for (size_t i = 0; i < regs_.size(); ++i)
	writeSingle(i, 0.0f);
    }
  else if (hasHalf)
    {
      // F16 extension present without F or D. Reset to NAN-boxed
      // half-precision zeros.
      for (size_t i = 0; i < regs_.size(); ++i)
	writeHalf(i, Float16{});
    }

  clearLastWrittenReg();
}
