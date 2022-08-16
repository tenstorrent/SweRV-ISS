// Copyright 2022 Tenstorrent Corporation.
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
#include <cassert>

#include "DecodedInst.hpp"
#include "Hart.hpp"

using namespace WdRiscv;


template <typename URV>
void
Hart<URV>::execCbo_clean(const DecodedInst* di)
{
  if (not isRvzicbom())
    {
      illegalInst(di);
      return;
    }

  typedef PrivilegeMode PM;
  PM pm = privilegeMode();

  URV menv = 0, senv = 0;
  peekCsr(CsrNumber::MENVCFG, menv);
  peekCsr(CsrNumber::SENVCFG, senv);

  unsigned mcbcfe = (menv >> 6) & 1;
  unsigned scbcfe = (senv >> 6) & 1;

  if ( (pm != PM::Machine and not mcbcfe) or
       (pm == PM::User and not scbcfe) )
    {
      illegalInst(di);
      return;
    }

  // Check for exception.
  static bool firstTime = true;
  if (firstTime)
    {
      std::cerr << "execCbo_clean: check for exception" << '\n';
      firstTime = false;
    }
}


template <typename URV>
void
Hart<URV>::execCbo_flush(const DecodedInst* di)
{
  if (not isRvzicbom())
    {
      illegalInst(di);
      return;
    }

  typedef PrivilegeMode PM;
  PM pm = privilegeMode();

  URV menv = 0, senv = 0;
  peekCsr(CsrNumber::MENVCFG, menv);
  peekCsr(CsrNumber::SENVCFG, senv);

  unsigned mcbcfe = (menv >> 6) & 1;
  unsigned scbcfe = (senv >> 6) & 1;

  if ( (pm != PM::Machine and not mcbcfe) or
       (pm == PM::User and not scbcfe) )
    {
      illegalInst(di);
      return;
    }

  // Check for exception.
  static bool firstTime = true;
  if (firstTime)
    {
      std::cerr << "execCbo_flush: check for exception" << '\n';
      firstTime = false;
    }
}


template <typename URV>
void
Hart<URV>::execCbo_inval(const DecodedInst* di)
{
  if (not isRvzicbom())
    {
      illegalInst(di);
      return;
    }

  typedef PrivilegeMode PM;
  PM pm = privilegeMode();

  URV menv = 0, senv = 0;
  peekCsr(CsrNumber::MENVCFG, menv);
  peekCsr(CsrNumber::SENVCFG, senv);

  unsigned mcbie = (menv >> 4) & 3;
  unsigned scbie = (senv >> 4) & 3;

  if ( (pm != PM::Machine and mcbie == 0) or
       (pm == PM::User and scbie == 0) )
    {
      illegalInst(di);
      return;
    }

  // Check for exception.
  static bool firstTime = true;
  if (firstTime)
    {
      std::cerr << "execCbo_inval: check for exception" << '\n';
      firstTime = false;
    }
}


template <typename URV>
void
Hart<URV>::execCbo_zero(const DecodedInst* di)
{
  if (not isRvzicboz())
    {
      illegalInst(di);
      return;
    }

  typedef PrivilegeMode PM;
  PM pm = privilegeMode();

  URV menv = 0, senv = 0;
  peekCsr(CsrNumber::MENVCFG, menv);
  peekCsr(CsrNumber::SENVCFG, senv);

  unsigned mcbze = (menv >> 7) & 1;
  unsigned scbze = (senv >> 7) & 1;

  if ( (pm != PM::Machine and not mcbze) or
       (pm == PM::User and not scbze) )
    {
      illegalInst(di);
      return;
    }

  // Translate virtual addr and check for exception.
  uint64_t virtAddr = intRegs_.read(di->op0());
  uint64_t physAddr = virtAddr;
  static bool firstTime = true;
  if (firstTime)
    {
      std::cerr << "execCbo_zero: check for exception" << '\n';
      firstTime = false;
    }

  assert((cacheLineSize_ % 8) == 0);
  for (unsigned i = 0; i < cacheLineSize_; i+= 8)
    memory_.poke(physAddr + i, uint64_t(0), true /*usePma*/);
}

template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
