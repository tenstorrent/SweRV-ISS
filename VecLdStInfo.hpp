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
    uint64_t pa_ = 0;      // Physical address of data.
    uint64_t pa2_ = 0;     // For page crossers: addr on 2nd page, otherwise same as pa_.
    uint64_t stData_ = 0;  // Store data.
    unsigned ix_ = 0;      // Index of element in vector register group.
    unsigned field_ = 0;   // For segment load store: field of element.
    bool masked_ = false;  // True if element is masked off (element skipped).

    VecLdStElem()
    { }

    VecLdStElem(uint64_t va, uint64_t pa, uint64_t pa2, uint64_t data, unsigned ix,
		bool masked, unsigned field = 0)
      : va_{va}, pa_{pa}, pa2_{pa2}, stData_{data}, ix_{ix}, field_{field},
	masked_{masked}
    { }
  };


  /// Track execution information for a vector load store instruction. This is used for
  /// tracing and by the memory consistency model.
  struct VecLdStInfo
  {
    /// Return true if no element was loaded/stored by vector instruction.
    bool empty() const
    { return elems_.empty(); }

    /// Clear this object.
    void clear()
    {
      elemSize_ = 0;
      elems_.clear();
    }

    /// Set element size (in bytes), data vector register, and type (load or store).
    void init(unsigned elemSize, unsigned vecReg, bool isLoad)
    {
      elemSize_ = elemSize;
      vec_ = vecReg;
      isLoad_ = isLoad;
      
      isIndexed_ = false;
      isSegmented_ = false;

      ixVec_ = 0;
      fields_ = 0;
    }

    /// Set element size (in bytes), data vector register, index vector register, and type
    /// (load or store).
    void initIndexed(unsigned elemSize, unsigned vecReg, unsigned ixReg, bool isLoad)
    {
      init(elemSize, vecReg, isLoad);
      isIndexed_ = true;
      ixVec_ = ixReg;
    }

    /// Set the field count. Used for ld/st segment and ld/st whole regs.
    void setFieldCount(unsigned fields, bool isSeg = false)
    {
      fields_ = fields;
      isSegmented_ = isSeg;
    }

    /// Add element information to this object.
    void addElem(const VecLdStElem& elem)
    {
      elems_.push_back(elem);
    }

    /// Set the physical addresses of the last added element.
    void setLastElem(uint64_t pa, uint64_t pa2)
    {
      elems_.back().pa_ = pa;
      elems_.back().pa2_ = pa2;
    }

    /// Set the physical addresses and the store data value of the last added element.
    void setLastElem(uint64_t pa, uint64_t pa2, uint64_t data)
    {
      elems_.back().pa_ = pa;
      elems_.back().pa2_ = pa2;
      elems_.back().stData_ = data;
    }

    /// Remove last added element.
    void removeLastElem()
    {
      assert(not elems_.empty());
      elems_.resize(elems_.size() - 1);
    }

    unsigned elemSize_ = 0;           // Elem size in bytes.
    unsigned vec_ = 0;                // Base data vector register.
    unsigned ixVec_ = 0;              // Base index vector register.
    unsigned fields_ = 0;             // For load/store segment.
    bool isLoad_ = false;             // True for load instructions.
    bool isIndexed_ = false;          // True for indexed instructions.
    bool isSegmented_ = false;        // True for load/store segment.
    std::vector<VecLdStElem> elems_;  // Element info.
  };

}
