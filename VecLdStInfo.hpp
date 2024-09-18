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

  /// Vector element information for vector load store instructions.
  struct VecLdStElem
  {
    uint64_t va_ = 0;      // Virtual address of data.
    uint64_t pa_ = 0;      // Physical addres of data.
    uint64_t pa2_ = 0;     // For page crossers: addr on 2nd page, otherwise same as pa_.
    uint64_t stData_ = 0;  // Store data.
    unsigned ix_ = 0;      // Index of element in vector register group.
    bool masked_ = false;  // True if element is masked off (element skipped).

    VecLdStElem(uint64_t va, uint64_t pa, uint64_t pa2, uint64_t data, unsigned ix,
		bool masked)
      : va_{va}, pa_{pa}, pa2_{pa2}, stData_{data}, ix_{ix}, masked_{masked}
    { }
  };


  /// Track vector load/store execution information.
  struct VecLdStInfo
  {
    bool empty() const
    { return elems_.empty(); }

    void clear()
    {
      elemSize_ = 0;
      elems_.clear();
    }

    void init(unsigned elemSize, unsigned vecReg, bool isLoad)
    {
      elemSize_ = elemSize;
      vec_ = vecReg;
      isLoad_ = isLoad;
    }

    void addElem(const VecLdStElem& elem)
    {
      elems_.push_back(elem);
    }

    void setLastElem(uint64_t pa, uint64_t pa2)
    {
      elems_.back().pa_ = pa;
      elems_.back().pa2_ = pa2;
    }

    void setLastElem(uint64_t pa, uint64_t pa2, uint64_t data)
    {
      elems_.back().pa_ = pa;
      elems_.back().pa2_ = pa2;
      elems_.back().stData_ = data;
    }

    unsigned elemSize_ = 0;
    unsigned vec_ = 0;      // Base vector register of vector load/store instruction.
    bool isLoad_ = false;
    std::vector<VecLdStElem> elems_;
  };

}
