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
#include <cfenv>
#include <cmath>
#include <climits>
#include <cassert>
#include <boost/multiprecision/cpp_int.hpp>
#include "float-convert-helpers.hpp"
#include "wideint.hpp"
#include "instforms.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"
#include "softfloat-util.hpp"


using namespace WdRiscv;


template <typename URV>
void
Hart<URV>::execVandn_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVandn_vx(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVbrev_v(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVbrev8_v(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVrev8_v(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVclz_v(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVctz_v(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVcpop_v(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVrol_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVrol_vx(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVror_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVror_vx(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVror_vi(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVwsll_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVwsll_vx(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVwsll_vi(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVclmul_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVclmul_vx(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVclmulh_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVclmulh_vx(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVghsh_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVgmul_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVaesdf_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVaesdf_vs(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVaesef_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVaesef_vs(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVaesem_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVaesem_vs(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVaesdm_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVaesdm_vs(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVaeskf1_vi(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVaeskf2_vi(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVaesz_vs(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVsha2ms_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVsha2ch_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVsha2cl_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVsm4k_vi(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVsm4r_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVsm4r_vs(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVsm3me_vv(const DecodedInst*)
{
  assert(0);
}


template <typename URV>
void
Hart<URV>::execVsm3c_vi(const DecodedInst*)
{
  assert(0);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;


