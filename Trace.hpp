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

#include <unordered_set>

#include "DecodedInst.hpp"
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

    /// Return the rounding mode associated with a floating point instruction
    uint32_t roundingMode() const
    { return di_.roundingMode(); }

    /// Privilege mode before last executed instruction.
    PrivilegeMode privMode() const
    { return hart_->lastPrivMode(); }

    /// Privilege mode after last executed instruction.
    PrivilegeMode nextPrivMode() const
    { return hart_->privilegeMode(); }

    /// Trap vector mode after last executed instruction
    TrapVectorMode nextTvecMode() const
    {
      URV tvec = peekCsr(CsrNumber::MTVEC);
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
        {
          if (nextPrivMode() == PrivilegeMode::Machine)
            cause = hart_->peekCsr(CsrNumber::MCAUSE);
          else if (nextPrivMode() == PrivilegeMode::Supervisor)
            cause = hart_->peekCsr(CsrNumber::SCAUSE);
        }
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

    /// Return true if last instruction was a branch and (if it was)
    /// whether it was taken.
    std::pair<bool, bool> lastBranchTaken() const
    { return std::make_pair(isBranch(), hart_->lastBranchTaken()); }

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
    bool peekCsr(CsrNumber csr, std::string_view field, URV& val) const
    { return hart_->peekCsr(csr, field, val); }

    /// Return the number of page table walks of the last
    /// executed instruction
    unsigned getNumPageTableWalks(bool instr) const
    { return hart_->getNumPageTableWalks(instr); }

    /// Return the page table walk addresses for load/store/fetch of last executed instruction.
    /// Will be empty if there was no walk.
    void getPageTableWalkAddresses(bool instr, unsigned ix,
                                   std::vector<uint64_t>& addrs) const
    { hart_->getPageTableWalkAddresses(instr, ix, addrs); }

    /// Return the page table walk entries for load/store/fetch of last executed instruction.
    /// Will be empty if there was no walk.
    void getPageTableWalkEntries(bool instr, unsigned ix,
                                 std::vector<uint64_t>& ptes) const
    { hart_->getPageTableWalkEntries(instr, ix, ptes); }

    bool peekIntReg(unsigned i, URV& value) const
    { return hart_->peekIntReg(i, value); }

    bool peekFpReg(unsigned i, uint64_t& value) const
    { return hart_->peekFpReg(i, value); }

    bool peekCsReg(CsrNumber number, URV& value) const
    { return hart_->peekCsr(number, value); }

    /// TODO: modified regs
    /// Return the list of CSR address-value pairs modified after last executed instruction.
    using CVP = std::pair<URV, URV>;  // CSR-value pair
    void getModifiedCsrs(std::vector<CVP>& cvps) const
    {
      URV value;
      cvps.clear();

      std::vector<CsrNumber> csrs;
      std::vector<unsigned> triggers;
      hart_->csRegs().getLastWrittenRegs(csrs, triggers);

      cvps.reserve(csrs.size() + triggers.size());

      /// Collect non-trigger CSRs and their values.
      for (CsrNumber csr : csrs)
        {
          if (not hart_->peekCsr(csr, value))
            continue;
          if (csr >= CsrNumber::TDATA1 and csr <= CsrNumber::TDATA3)
            continue; // Debug trigger values collected below.
          cvps.push_back(CVP(URV(csr), value));
        }

      /// Collect trigger CSRs and their values. A synthetic CSR number
      /// is used encoding the trigger number and the trigger component.
      for (unsigned trigger : triggers)
        {
          uint64_t data1(0), data2(0), data3(0);
          if (not hart_->peekTrigger(trigger, data1, data2, data3))
            continue;

          // Components of trigger that changed.
          bool t1 = false, t2 = false, t3 = false;
          hart_->getTriggerChange(trigger, t1, t2, t3);

          if (t1)
            {
              URV ecsr = (trigger << 16) | URV(CsrNumber::TDATA1);
              cvps.push_back(CVP(ecsr, data1));
            }

          if (t2)
            {
              URV ecsr = (trigger << 16) | URV(CsrNumber::TDATA2);
              cvps.push_back(CVP(ecsr, data2));
            }

          if (t3)
            {
              URV ecsr = (trigger << 16) | URV(CsrNumber::TDATA3);
              cvps.push_back(CVP(ecsr, data3));
            }
        }
    }

    using SVP = std::pair<URV, uint64_t>;  // select-value pair
    void getModifiedImsicCsrs(std::vector<SVP>& mcvps,
                              std::vector<SVP>& scvps,
                              std::vector<std::vector<SVP>>& gcvps) const
    {
      if (not hart_->imsic())
        return;

      const auto& imsic = (*hart_->imsic());

      std::vector<std::pair<unsigned, unsigned>> mselects, sselects;
      std::vector<std::vector<std::pair<unsigned, unsigned>>> gselects;

      imsic.fileTraces(mselects, sselects, gselects);

      bool ok; unsigned int value;
      for (auto [select, size] : mselects)
        {
          if (size == 4)
            ok = imsic.template readMireg<uint32_t>(select, value);
          else
            ok = imsic.readMireg(select, value);
          if (ok)
            mcvps.emplace_back(select, value);
        }

      for (auto [select, size] : sselects)
        {
          if (size == 4)
            ok = imsic.template readSireg<uint32_t>(false, 0 /* guest */, select, value);
          else
            ok = imsic.readSireg(false, 0 /* guest */, select, value);
          if (ok)
            scvps.emplace_back(select, value);
        }

      for (unsigned i = 0; i < gselects.size(); ++i)
        {
          std::vector<SVP> tmp;
          for (auto [select, size] : gselects.at(i))
            {
              if (size == 4)
                ok = imsic.template readSireg<uint32_t>(true, i, select, value);
              else
                ok = imsic.readSireg(true, i, select, value);

              if (ok)
                tmp.emplace_back(select, value);
            }
          gcvps.push_back(std::move(tmp));
        }
    }

    /// TODO: add support for vector ld/st
    unsigned lastLdStAddress(uint64_t& virtAddr, uint64_t& physAddr) const
    { return hart_->lastLdStAddress(virtAddr, physAddr); /* this returns the size of the last ld/st*/ }

    unsigned lastStVal(uint64_t& value) const
    {
      uint64_t addr;
      return hart_->lastStore(addr, value); /* this returns the size of the last ld/st */
    }

    bool misalignedLdSt(bool& misal) const
    { return hart_->misalignedLdSt(misal); }

    /// Return true if associated hart is holding a reservation
    /// (obtained thru an LR instruction).
    bool hasLr() const
    { return hart_->hasLr(); }

    /// Return the reason for loss of reservation of the associated hart.
    CancelLrCause cancelLrCause() const
    { return hart_->cancelLrCause(); }

    void getPmpsAccessed(std::vector<PmpManager::PmpTrace>& pmps) const
    { hart_->getPmpsAccessed(pmps); }

    void getPmasAccessed(std::vector<PmaManager::PmaTrace>& pmas) const
    { hart_->getPmasAccessed(pmas); }

    bool matchMultiplePmp(uint64_t addr) const
    { return hart_->pmpManager().matchMultiplePmp(addr); }

    bool matchMultiplePma(uint64_t addr) const
    { return hart_->pmaManager().matchMultiplePma(addr); }

    /// Return the number of pages accessed by a vector ld/st instruction.
    unsigned numVecPagesAccessed() const
    {
      if (not isVector())
        return 0;

      std::unordered_set<uint64_t> pages;
      const auto& addrs = hart_->vecRegs().ldStAddrs();
      const auto& masked = hart_->vecRegs().maskedAddrs();
      assert(addrs.size() == masked.size());

      for (unsigned i = 0; i < addrs.size(); ++i)
        {
          const auto& addr = addrs.at(i);
          const auto& mask = masked.at(i);

          if (not mask)
            {
              unsigned page = hart_->virtMem().pageNumber(addr);
              pages.emplace(page);
            }
        }
      return pages.size();
    }

    bool virtualMode() const
    { return hart_->lastVirtMode();  }

    bool nextVirtualMode() const
    { return hart_->virtMode();  }

    const Hart<URV>* hart_ = nullptr;
    const DecodedInst& di_;
  };
}
