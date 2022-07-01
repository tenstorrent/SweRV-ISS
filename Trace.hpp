// Copyright 2022 Tenstorrent.
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

#include "Hart.hpp"

namespace WdRiscv
{
  template <typename URV>
  struct TraceRecord
  {
  public:

    TraceRecord(Hart<URV>* hart, const DecodedInst& di)
      : hart_(hart), di_(di)
    { }

    /// Hart index of trace record. Every hart in the system is
    /// assigned an unqiue index from a set of consecutive integers
    /// starting with zero.
    unsigned hartIndex() const
    { return hart_->sysHartIndex(); }

    /// Virtual PC of last executed instruction.
    uint64_t virtPc() const
    { return di_.address(); }

    /// Physical PC of last executed instruction.
    uint64_t physPc() const
    { return di_.physAddress(); }

    /// Opcode of last executed instruction.
    uint32_t instruction() const
    { return di_.inst(); }

    /// PC of the next instruction.
    uint64_t nextVirtPc() const
    { return hart_->pc(); }

    /// Privilege mode before last executed instruction.
    PrivilegeMode privMode() const
    { return hart_->lastPrivMode(); }

    /// True if last executed instruction encoutred a trap.
    bool hasTrap() const
    { return hart_->lastInstructionTrapped(); }

    /// Return size of data of last load/store/amo instruction or zero
    /// if last instrcution was not load/store/amo.  Set virtAddr and
    /// physAddr to the corresponding addresses if load/store/amo.
    unsigned loadSoreAddr(uint64_t& virtAddr, uint64_t& physAddr)
    { return hart_->lastLdStAddress(virtAddr, physAddr); }

    /// Return true if record is for a load instruction.
    bool isLoad() const
    { return di_.isLoad(); }

    /// Return true if record is for a store instruction.
    bool isStore() const
    { return di_.isStore(); }

    /// Return true if record is for an AMO instruction.
    bool isAmo() const
    { return di_.isAmo(); }

    /// Return true if record is for a branch instruction.
    bool isBranch() const
    { return di_.isBranch(); }

    /// Return true if record is for a conditional branch instruction.
    bool isConditionalBranch() const
    { return di_.isConditionalBranch(); }

    /// Return true if this is a branch instruction where the target
    /// address is in a register.
    bool isBranchToRegister() const
    { return di_.isBranchToRegister(); }

    /// Return true if record is for a floating point instruction.
    bool isFp() const
    { return di_.isFp(); }

    /// Return true if record is for a vector instruction.
    bool isVector() const
    { return di_.isVector(); }

    /// Return true if record is for a multiply instruction.
    bool isMultiply() const
    { return di_.isVector(); }

    /// Return true if record is for a divide/remainder instruction.
    bool isDivide() const
    { return di_.isDivide(); }

    /// Return the RISCV ISA extension of the instruction of this record.
    RvExtension extension() const
    { return di_.extension(); }

    /// Return the RISCV instruction format of the instruction of this record.
    RvFormat format() const
    { return di_.format(); }

    /// Return the instruction id of the instruction of this record.
    InstId instId() const
    { return di_.instId(); }

    /// Return the instruction name of the instruction of this record.
    std::string name() const
    { return di_.name(); }

    const Hart<URV>* hart_ = nullptr;
    const DecodedInst& di_;
  };
}
