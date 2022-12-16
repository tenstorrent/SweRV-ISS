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

    /// Return the operand type of the ith operand. Returns None if
    /// i is out of bounds.
    OperandType ithOperandType(unsigned i) const
    { return di_.ithOperandType(i); }

    /// Return the mode of the ith operand or None if i is out of
    /// bounds. Object must be valid.
    OperandMode ithOperandMode(unsigned i) const
    { return di_.ithOperandMode(i); }

    /// Return the ith operands or zero if i is out of bounds. For exmaple, if
    /// decode insruction is "addi x3, x4, 10" then the 0th operand would be 3
    /// and the second operands would be 10.
    uint32_t ithOperand(unsigned i) const
    { return di_.ithOperand(i); }

    /// Privilege mode before last executed instruction.
    PrivilegeMode privMode() const
    { return hart_->lastPrivMode(); }

    /// Privilege mode after last executed instruction.
    PrivilegeMode nextPrivMode() const
    { return hart_->privilegeMode(); }

    /// Trap vector mode after last executed instruction
    TrapVectorMode nextTvecMode() const
    {
      URV tvec = 0;
      peekCsr(CsrNumber::MTVEC, tvec);
      auto tvm = TrapVectorMode{tvec&3};
      return tvm;
    }

    /// True if last executed instruction encountered a trap.
    bool hasTrap() const
    { return hart_->lastInstructionTrapped(); }

    /// True if last executed instruction encountered a trap and
    /// set cause to interrupt/exception cause.
    bool hasTrap(URV& cause) const
    {
      bool trapped = hasTrap();
      if (trapped)
        hart_->peekCsr(CsrNumber::MCAUSE, cause);
      return trapped;
    }

    /// True if traget program finished.
    bool hasStop() const
    { return hart_->hasTargetProgramFinished(); }

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

    /// Return currently configured element width.
    ElementWidth elemWidth() const
    { return hart_->elemWidth(); }

    /// Return currently configured group multiplier.
    GroupMultiplier groupMultiplier() const
    { return hart_->groupMultiplier(); }

    /// Return the paging mode before last executed instruction.
    VirtMem::Mode pageMode() const
    { return hart_->lastPageMode(); }

    /// Return the paging mode after last executed instruction.
    VirtMem::Mode nextPageMode() const
    { return hart_->pageMode(); }

    /// Return CSR value after last executed instruction.
    bool peekCsr(CsrNumber csr, URV& val) const
    { return hart_->peekCsr(csr, val); }

    /// Return CSR field value after last executed instruction.
    bool peekCsr(CsrNumber csr, std::string field, URV& val) const
    { return hart_->peekCsr(csr, field, val); }

    /// Return the page table walk for load/store/fetch of last executed instruction.
    /// Will be empty if there was no walk.
    void getPageTableWalkEntries(bool instr, std::vector<uint64_t>& ptes) const
    { hart_->getPageTableWalkEntries(instr, ptes); }

    const Hart<URV>* hart_ = nullptr;
    const DecodedInst& di_;
  };
}
