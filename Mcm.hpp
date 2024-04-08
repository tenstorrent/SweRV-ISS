#pragma once

#include <vector>
#include <array>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <set>
#include <unordered_set>
#include "DecodedInst.hpp"


namespace WdRiscv
{
  template <typename URV>
  class Hart;

  using McmInstrIx = uint32_t;
  using MemoryOpIx = uint32_t;

  struct MemoryOp
  {
    uint64_t   time_           = 0;
    uint64_t   physAddr_       = 0;
    uint64_t   data_           = 0;
    uint64_t   rtlData_        = 0;
    McmInstrIx instrTag_       = 0;
    uint64_t   forwardTime_    = 0;  // Time of store instruction forwarding to this op.
    uint8_t    hartIx_    : 8  = 0;
    uint8_t    size_      : 8  = 0;
    bool       isRead_    : 1  = false;
    bool       failRead_  : 1  = false;
    bool       canceled_  : 1  = false;

    bool isCanceled() const { return canceled_; }
    void cancel() { canceled_ = true; }
  };


  struct McmInstr
  {
    // memOps contains indices into an array of MemoryOp items.
    std::vector<MemoryOpIx> memOps_;
    uint64_t virtAddr_ = 0;   // Virtual data address for ld/st instructions.
    uint64_t physAddr_ = 0;   // Phusical data address for ld/st instruction.
    uint64_t physAddr2_ = 0;  // Additional data address for page crossing stores.
    uint64_t data_ = 0;       // Data for load/sore instructions.
    uint64_t addrTime_ = 0;   // Time address register was produced (for ld/st/amo).
    uint64_t dataTime_ = 0;   // Time data register was produced (for st/amo).
    McmInstrIx addrProducer_ = 0;
    McmInstrIx dataProducer_ = 0;
    DecodedInst di_;
    McmInstrIx tag_ = 0;
    uint8_t size_   : 8 = 0;        // Data size for load/store insructions.
    bool retired_   : 1 = false;
    bool canceled_  : 1 = false;
    bool isLoad_    : 1 = false;
    bool isStore_   : 1 = false;
    bool complete_  : 1 = false;
    bool forwarded_ : 1 = false; // True if all bytes of load were forwarded.

    /// Return true if this a load/store isntruction.
    bool isMemory() const { return isLoad_ or isStore_; }

    /// Return true if this instruction is retired.
    bool isRetired() const { return retired_; }

    /// Return true if this instruction is canceled.
    bool isCanceled() const { return canceled_; }

    /// Mark instruction as canceled.
    void cancel() { canceled_ = true; }

    /// Associated given memory operation index with this instruction.
    void addMemOp(MemoryOpIx memOpIx)
    {
      if (std::find(memOps_.begin(), memOps_.end(), memOpIx) != memOps_.end())
	{
	  std::cerr << "McmInstr::addMemOp: Error: Op already added\n";
	  assert(0 && "McmInstr::addMemOp: Op ix already present");
	}
      memOps_.push_back(memOpIx);
    }

    /// Return true if data memory referenced by this instruction overlaps that
    /// of the given other instruction. Both instructions must be memory
    /// instructions.  Both instructions must be retired.
    bool overlaps(const McmInstr& other) const
    {
      // A non-successful store conditional (zero size) does not overlap anything.
      if ((di_.isSc() and size_ == 0) or (other.di_.isSc() and other.size_ == 0))
	return false;
	  
      if (size_ == 0 or other.size_ == 0)
	std::cerr << "McmInstr::overlaps: Error: tag1=" << tag_
		  << " tag2=" << other.tag_ << " zero data size\n";

      if (not retired_ or not other.retired_)
	std::cerr << "McmInstr::overlaps: Error: tag1=" << tag_
		  << " tag2=" << other.tag_ << " non-retired instruction\n";

      if (virtAddr_ == other.virtAddr_)
	return true;
      if (virtAddr_ < other.virtAddr_)
	return other.virtAddr_ - virtAddr_ < size_;
      return virtAddr_ - other.virtAddr_ < other.size_;
    }

    /// Return true if address of the data memory referenced by this
    /// instruction is aligned.
    bool isAligned() const
    { return (physAddr_ & (size_ - 1)) == 0; }

  };


  template <typename URV>
  class Mcm
  {
  public:

    Mcm(unsigned hartCount, unsigned pageSize, unsigned mergeBufferSize);

    ~Mcm();

    /// Initiate an out of order read for a load instruction. If a
    /// preceding overlapping store has not yet left the merge/store
    /// buffer then forward data from that store to the read operation;
    /// otherwise, get the data from glbal memory. Return true on
    /// success and false if global memory is not readable (in the
    /// case where we do not forward).
    bool readOp(Hart<URV>& hart, uint64_t time, uint64_t instrTag,
		uint64_t physAddr, unsigned size, uint64_t rtlData);
    
    /// This is a write operation bypassing the merge buffer.
    bool bypassOp(Hart<URV>& hart, uint64_t time, uint64_t instrTag,
		  uint64_t physAddr, unsigned size, uint64_t rtlData);

    /// Initiate a merge buffer write.  All associated store write
    /// transactions are marked completed. Write instructions where
    /// all writes are complete are marked complete. Return true on
    /// success.  The given physical address must be a multiple of the
    /// merge buffer line size (which is also the cache line
    /// size). The rtlData vector must be of size n or larger where n
    /// is the merge buffer line size. The rtlData bytes will be
    /// placed in memory in consecutive locations starting with
    /// physAddr.
    bool mergeBufferWrite(Hart<URV>& hart, uint64_t time, uint64_t physAddr,
			  const std::vector<uint8_t>& rtlData,
			  const std::vector<bool>& mask);

    /// Insert a write operation for the given instruction into the
    /// merge buffer removing it from the store buffer. Return true on
    /// success. Return false if no such operation is in the store
    /// buffer.
    bool mergeBufferInsert(Hart<URV>& hart, uint64_t time, uint64_t instrTag,
			   uint64_t physAddr, unsigned size,
			   uint64_t rtlData);

    /// Cancel all the memory operations associted with the given tag. This is
    /// done when a speculative instruction is canceled or when an instruction
    /// is trapped.
    void cancelInstruction(Hart<URV>& hart, uint64_t instrTag);

    /// This is called when an instruction is retired.
    bool retire(Hart<URV>& hart, uint64_t time, uint64_t instrTag,
		const DecodedInst& di, bool trapped);

    /// Perform PPO checks (e.g. rule 4) on pending instructions.
    bool finalChecks(Hart<URV>& hart);

    bool setCurrentInstruction(Hart<URV>& hart, uint64_t instrTag);

    /// Return the load value of the current target instruction which
    /// must be a load instruction (set with setCurrentInstruction).
    /// Paddr1 is the physical address of the loaded data. Paddr2 is
    /// the same as paddr1 except for page corssing loads where paddr2
    /// is the physical address of the second page. Vaddr is the virtual
    /// address of the load data.
    bool getCurrentLoadValue(Hart<URV>& hart, uint64_t vaddr, uint64_t paddr1,
			     uint64_t paddr2, unsigned size, uint64_t& value);

    /// Return the merge buffer line size in bytes.
    unsigned mergeBufferLineSize() const
    { return lineSize_; }

    /// Skip checking RTL read-op values against this model. This is used
    /// for items like the CLINT timer where we cannot match the RTL.
    void skipReadCheck(uint64_t addr)
    { skipReadCheck_.insert(addr); }

    /// Enable/disable total-store-order.
    void enableTso(bool flag)
    { isTso_ = flag; }

    /// Return the earliest memory time for the byte at the given
    /// address. Return 0 if address is not covered by given instruction.
    uint64_t earliestByteTime(const McmInstr& instr, uint64_t addr) const;

    /// Return the latest memory time for the byte at the given
    /// address. Return max value if address is not covered by given
    /// instruction.
    uint64_t latestByteTime(const McmInstr& instr, uint64_t addr) const;

    bool ppoRule1(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule2(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule3(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule4(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule5(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule6(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule7(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule8(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule9(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule10(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule11(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule12(Hart<URV>& hart, const McmInstr& instr) const;

    /// Check that previous SINVAL.VMA instructions are executed before subsequent
    /// implicit references to the memory-management data structures. Return true on
    /// success and false if there are translations between the the current
    /// SFENCE.INVAL.IR and the most recent SINVAL.VMA instruction.
    bool checkSfenceInvalIr(Hart<URV>& hart, const McmInstr& instr) const;

    /// If given instruction is a fence add it to the set of pending
    /// fences. If oldest pending fence instruction is within window,
    /// then remove it from pending set and check rule 4 on it.
    bool processFence(Hart<URV>& hart, const McmInstr& instr);

    uint64_t latestOpTime(const McmInstr& instr) const
    {
      if (not instr.complete_)
	{
	  std::cerr << "Mcm::latestOpTime: Called on an incomplete instruction\n";
	  assert(0 && "Mcm::lasestOpTime: Incomplete instr");
	}
      uint64_t time = 0;
      for (auto opIx : instr.memOps_)
	if (opIx < sysMemOps_.size())
	  time = std::max(time, sysMemOps_.at(opIx).time_);
      return time;
    }

    /// Return the smallest time of the memory operations of given instruction.
    uint64_t earliestOpTime(const McmInstr& instr) const
    {
      if (not instr.complete_ and instr.memOps_.empty())
	return time_;

      uint64_t mt = ~uint64_t(0);
      for (auto opIx : instr.memOps_)
	if (opIx < sysMemOps_.size())
	  mt = std::min(mt, sysMemOps_.at(opIx).time_);
      return mt;
    }

    /// Return the smallest time of the memor operations of the given
    /// instruction. Adjust read-operation times to account for
    /// forwarding.
    uint64_t effectiveReadTime(const McmInstr& instr) const;

    /// Return true if instruction a is before b in memory time.
    bool isBeforeInMemoryTime(const McmInstr& a, const McmInstr& b) const
    {
      // if (a.complete_ and not b.complete_)
      // return true;
      if (not a.complete_ and b.complete_)
	return false;
      if (not a.complete_ and not b.complete_)
	{
	  std::cerr << "Mcm::isBeforeInMemoryTime: Both instructions incomplete\n";
	  assert(0 && "Mcm::isBeforeInMemoryTime: Both instructions incomplete");
	  return false;
	}
      uint64_t aTime = latestOpTime(a);
      uint64_t bTime = earliestOpTime(b);
      if (a.isStore_ and b.isStore_ and aTime == bTime)
	return a.tag_ < b.tag_;
      return aTime < bTime;
    }

    /// Configure checking whole merge buffer line (versus checking
    /// bytes covered by stores).
    void setCheckWholeMbLine(bool flag)
    { checkWholeLine_ = flag; }

  protected:

    using MemoryOpVec = std::vector<MemoryOp>;

    void cancelReplayedReads(McmInstr*);

    /// Compute a mask of the instruction data bytes covered by the
    /// given memory operation. Return 0 if the operation does not
    /// overlap the given instruction.
    unsigned determineOpMask(const McmInstr& instr, const MemoryOp& op) const;

    /// If op overlaps instruction then trim high end of op to the
    /// boundary of the instruction.
    void trimMemoryOp(const McmInstr& instr, MemoryOp& op);

    /// Helper to retire method: Capture paramters of store instruction and
    /// commit its data to memory. Return true on success and false on failure.
    /// Return true if instuction is not a a store.
    bool retireStore(Hart<URV>& hart, McmInstr& instr);

    /// Return the page number corresponding to the given address
    uint64_t pageNum(uint64_t addr) const
    { return addr >> 12; }

    /// Return the address of the page with the given page number.
    uint64_t pageAddress(uint64_t pageNum) const
    { return pageNum << 12; }

    /// Remove from hartPendingWrites_ the write ops falling with the given RTL
    /// line and masked by rtlMask (rtlMask bit is on for op bytes) and place
    /// them sorted by instr tag in coveredWrites. Write ops may not straddle
    /// line boundary. Write ops may not be partially masked.
    bool collectCoveredWrites(Hart<URV>& hart, uint64_t time, uint64_t lineBegin,
			      uint64_t lineSize, const std::vector<bool>& rtlMask,
			      MemoryOpVec& coveredWrites);

    /// Forward from a store to a read op. Return true on success.
    /// Return false if instr is not retired, is canceled, is not a
    /// store (amo couts as store), or does not match range address of
    /// op.  Mask is the mask of bits of op to be updated by the
    /// forward (bypass) operartion and is updated (bits cleared) if
    /// some parts of op, covered by the mask, are successfully
    /// updated.
    bool forwardTo(const McmInstr& instr, uint64_t iaddr, uint64_t idata,
		   unsigned isize, MemoryOp& op, uint64_t& mask);

    /// Forward to the given read op from the stores of the retired
    /// instructions ahead of tag.
    bool forwardToRead(Hart<URV>& hart, uint64_t tag, MemoryOp& op);

    /// Determine the source and destination registers of the given
    /// instruction.
    void identifyRegisters(const DecodedInst& di,
			   std::vector<unsigned>& sourceRegs,
			   std::vector<unsigned>& destRegs);

    /// Return true if the data memory referenced by given instruction overlpas
    /// that of the given memory operation.
    bool overlaps(const McmInstr& instr, const MemoryOp& op) const
    {
      if (instr.size_ == 0 or op.size_ == 0)
	std::cerr << "Mcm::overlaps: Error: tag1=" << instr.tag_
		  << " tag2=" << op.instrTag_ << " zero data size\n";

      if (instr.physAddr_ == instr.physAddr2_)   // Non-page-crossing
	return rangesOverlap(instr.physAddr_, instr.size_, op.physAddr_, op.size_);

      // Page crossing.
      unsigned size1 = offsetToNextPage(instr.physAddr_);
      if (rangesOverlap(instr.physAddr_, size1, op.physAddr_, op.size_))
	return true;
      unsigned size2 = instr.size_ - size1;
      return rangesOverlap(instr.physAddr2_, size2, op.physAddr_, op.size_);
    }

    /// Return true if the given address ranges overlap one another.
    bool rangesOverlap(uint64_t addr1, unsigned size1, uint64_t addr2, unsigned size2) const
    {
      if (addr1 <= addr2)
	return addr2 - addr1 < size1;
      return addr1 - addr2 < size2;
    }

    /// Return true if given instruction data addresses overlap the
    /// given address. Return false if instruction is not a memory
    /// instruction. Instruction must be retired.
    bool instrOverlapsPhysAddr(const McmInstr& instr, uint64_t addr) const
    {
      if (not instr.isMemory())
	return false;
      assert(instr.isRetired());
      if (instr.physAddr_ == instr.physAddr2_)
	return instr.physAddr_ <= addr and addr - instr.physAddr_ < instr.size_;

      unsigned size1 = offsetToNextPage(instr.physAddr_);
      if (pageNum(instr.physAddr_) == pageNum(addr))
	return instr.physAddr_ <= addr and addr - instr.physAddr_ < size1;
      unsigned size2 = instr.size_ - size1;
      return instr.physAddr2_ <= addr and addr - instr.physAddr2_ < size2;
    }

    bool instrHasRead(const McmInstr& instr) const;

    bool instrHasWrite(const McmInstr& instr) const;

    bool checkStoreComplete(const McmInstr& instr) const;

    bool checkStoreData(unsigned hartId, const McmInstr& insrt) const;

    bool checkLoadComplete(const McmInstr& instr) const;

    /// Clear in the given mask, bits corresponding to the target
    /// instruction bytes covered by the given store instruction.
    void clearMaskBitsForWrite(const McmInstr& storeInstr, const McmInstr& target,
			       unsigned& mask) const;

    void cancelNonRetired(unsigned hartIx, uint64_t instrTag);

    bool checkRtlWrite(unsigned hartId, const McmInstr& instr,
		       const MemoryOp& op) const;

    bool checkRtlRead(unsigned hartId, const McmInstr& instr,
		      const MemoryOp& op) const;

    bool updateTime(const char* method, uint64_t time);

    McmInstr* findInstr(unsigned hartIx, McmInstrIx tag);

    McmInstr* findOrAddInstr(unsigned hartIx, McmInstrIx tag);

    void cancelInstr(McmInstr& instr)
    {
      if (instr.isCanceled())
	std::cerr << "Instr with tag=" << instr.tag_ << " already canceled\n";
      instr.cancel();
      for (auto memIx : instr.memOps_)
	{
	  if (sysMemOps_.at(memIx).isCanceled())
	    std::cerr << "Mcm::cancelInstr: Error: op already canceled\n";
	  sysMemOps_.at(memIx).cancel();
	}
    }

    void updateDependencies(const Hart<URV>& hart, const McmInstr& instr);

    void setProducerTime(unsigned hartIx, McmInstr& instr);

    /// Map register number of operand opIx to a unique integer by adding
    /// an offset: integer register have 0 offset, fp regs have 32, and
    /// csr regs have 64.
    unsigned effectiveRegIx(const DecodedInst& di, unsigned opIx) const;

    /// Return the difference between the next page boundary and the
    /// current address. Return 0 if address is on a page boundary.
    unsigned offsetToNextPage(uint64_t addr) const
    { return pageSize_ - (addr & (pageSize_ - 1)); }

  private:

    const unsigned intRegOffset_ = 0;
    const unsigned fpRegOffset_ = 32;
    const unsigned csRegOffset_ = 64;
    const unsigned totalRegCount_ = csRegOffset_ + 4096; // 4096: max csr count.

    using McmInstrVec = std::vector<McmInstr>;

    using RegTimeVec = std::vector<uint64_t>; // Map reg index to time.
    using RegProducer = std::vector<uint64_t>; // Map reg index to instr tag.

    MemoryOpVec sysMemOps_;  // Memory ops of all cores.
    std::vector<McmInstrVec> hartInstrVecs_; // One vector per hart.
    std::vector<MemoryOpVec> hartPendingWrites_; // One vector per hart.

    uint64_t time_ = 0;
    uint64_t sinvalVmaTime_ = 0;
    uint64_t sinvalVmaTag_ = 0;
    unsigned pageSize_ = 4096;
    unsigned lineSize_ = 64; // Merge buffer line size.
    unsigned windowSize_ = 1000;

    bool writeOnInsert_ = false;

    // Check whole merge buffer line if true otherwise check bytes
    // covered by store instructions.
    bool checkWholeLine_ = false;

    bool isTso_ = false;  // True if total-store-ordering model.

    std::vector<McmInstrIx> currentInstrTag_;

    std::vector<RegTimeVec> hartRegTimes_;  // One vector per hart.
    std::vector<RegProducer> hartRegProducers_;  // One vector per hart.
    std::vector<std::set<McmInstrIx>> hartPendingFences_;

    // Dependency time of most recent branch in program order or 0 if
    // branch does not depend on a prior memory instruction.
    std::vector<uint64_t> hartBranchTimes_;
    std::vector<uint64_t> hartBranchProducers_;

    std::unordered_set<uint64_t> skipReadCheck_;
  };

}
