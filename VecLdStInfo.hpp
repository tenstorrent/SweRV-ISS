// Copyright 2024 Tenstorrent Corporation.
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

#pragma once

#include <cstdint>

namespace WdRiscv
{

  /// Track vector load/store per element information.
  struct VecLdStInfo
  {
    VecLdStInfo(uint64_t va, uint64_t pa, uint64_t pa2, uint64_t stData, unsigned size,
		bool skip, bool isLoad)
      : va_{va}, pa_{pa}, pa2_{pa2}, stData_{stData}, size_{size}, masked_{skip},
	isLoad_{isLoad}
    { }
   
    uint64_t va_ = 0;      // Virtual address of data.
    uint64_t pa_ = 0;      // Physical addres of data.
    uint64_t pa2_ = 0;     // For page crossers: addr on 2nd page, otherwise same as pa_.
    uint64_t stData_ = 0;  // Store data.
    unsigned size_ = 0;    // Element size.
    bool masked_ = false;  // True if element is masked off (element skipped).
    bool isLoad_ = false;  // True if a load instruction.
  };

}
