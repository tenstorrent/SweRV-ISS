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

#include "instforms.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"


using namespace WdRiscv;


template <typename URV>
void
Hart<URV>::execHfence_vvma(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execHfence_gvma(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execHlv_b(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execHlv_bu(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execHlv_h(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execHlv_hu(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execHlv_w(const DecodedInst*)
{
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
Hart<URV>::execHsv_b(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execHsv_h(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execHsv_w(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execHlv_wu(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execHlv_d(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execHsv_d(const DecodedInst*)
{
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
