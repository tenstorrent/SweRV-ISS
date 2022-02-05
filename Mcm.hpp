#pragma once

#include <vector>
#include <array>
#include <algorithm>
#include <cassert>


namespace WdRiscv
{
  template <typename URV>
  class System;

  template <typename URV>
  class Hart;

  class DecodedInst;

  typedef uint32_t McmInstrIx;
  typedef uint32_t MemoryOpIx;

  struct MemoryOp
  {
    uint64_t time_ = 0;
    uint64_t physAddr_ = 0;
    uint64_t data_ = 0;
    uint64_t rtlData_ = 0;
    McmInstrIx instrTag_ = 0;
    uint8_t hartIx_ = 0;
    uint8_t size_ = 0;
    bool isRead_ = false;
    bool internal_ = false;
    bool failRead_ = false;
    bool canceled_ = false;

    bool isCanceled() const { return canceled_; }
    void cancel() { canceled_ = true; }
  };


  struct McmInstr
  {
    // memOps contains indices into an array of MemoryOp items.
    std::vector<MemoryOpIx> memOps_;
    uint64_t physAddr_ = 0;   // Data address for ld/store instruction.
    uint64_t data_ = 0;       // Data for load/sore instructions.
    McmInstrIx tag_ = 0;
    uint8_t size_ = 0;        // Data size for load/store insructions.
    bool retired_ = false;
    bool canceled_ = false;
    bool isStore_ = false;

    bool isRetired() const { return retired_; }

    bool isCanceled() const { return canceled_; }

    void cancel() { canceled_ = true; }

    void addMemOp(MemoryOpIx memOpIx)
    {
      assert(std::find(memOps_.begin(), memOps_.end(), memOpIx) == memOps_.end());
      memOps_.push_back(memOpIx);
    }
  };


  template <typename URV>
  class Mcm
  {
  public:

    Mcm(WdRiscv::System<URV>&);

    ~Mcm();

    /// Initiate an out of order read for a load instruction. If the
    /// the read is external, we read the data from the global memory
    /// and keep for a later compare to the RTL data (we do not perform
    /// an immediate compare becuase the instruction may later be canceled).
    /// If the the read is internal we defer it till the corresponding
    /// instruction is retired because some of the store instructions
    /// from which to obtain forwarded data may not yet have appeared.
    /// Return true on success.  Return false if load is exernal but
    /// corresponding memory is not readable.
    bool readOp(unsigned HartId, uint64_t time, uint64_t instrTag,
		uint64_t physAddr, unsigned size, uint64_t rtlData,
		bool internal);
    
    /// Initiate a merge buffer write.  All associated store write
    /// transactions are marked completed. Write instructions where all
    /// writes are complete are marked complete. Return true on success.
    bool mergeBufferWrite(unsigned hartId, uint64_t time, uint64_t physAddr,
			  const std::vector<uint8_t>& rtlData);

    /// Insert a write operation for the given instruction into the
    /// merge buffer removing it from the store buffer. Return true on
    /// success. Return false if no such operation is in the store
    /// buffer.
    bool mergeBufferInsert(unsigned hartId, uint64_t time, uint64_t instrTag,
			   uint64_t physAddr, unsigned size,
			   uint64_t rtlData);

    /// Cancel all the early-read transaction associted with the given
    /// tag. This is done when a speculative load instruction is canceled.
    bool cancelRead(unsigned hartId, uint64_t instTag);

    /// This is called when an instruction is retired.
    bool retire(unsigned hartId, uint64_t time, uint64_t instrTag);

    bool setCurrentInstruction(unsigned hartId, uint64_t instrTag);

    /// Return the load value of the current target instruction
    /// (set with setCurrentInstruction).
    bool getCurrentLoadValue(unsigned hartId, uint64_t addr, unsigned size,
			     uint64_t& value);

  protected:

    /// Forward from a store to a read op. Return true on success.
    /// Return false if instr is not retired, is canceled, is not a
    /// store, is amo or does not match range address of op.  Mask is
    /// the mask of bits of op to be updated by the forward (bypass)
    /// operartion and is updated (bits cleared) if some parts of op,
    /// covered by the mask, are successfully updated.
    bool forwardTo(const McmInstr& instr, MemoryOp& op, uint64_t& mask);

    void cancelNonRetired(unsigned hartIx, uint64_t instrTag);

    bool checkRtlWrite(unsigned hartId, const McmInstr& instr,
		       const MemoryOp& op);

    bool checkRtlRead(unsigned hartId, const McmInstr& instr,
		      const MemoryOp& op);

    bool updateTime(const char* method, uint64_t time);

    McmInstr* findInstr(unsigned hartIx, McmInstrIx tag);

    McmInstr* findOrAddInstr(unsigned hartIx, McmInstrIx tag);

    void cancelInstr(McmInstr& instr)
    {
      assert(not instr.isCanceled());
      instr.cancel();
      for (auto memIx : instr.memOps_)
	{
	  assert(not sysMemOps_.at(memIx).isCanceled());
	  sysMemOps_.at(memIx).cancel();
	}
    }

  private:

    typedef std::vector<McmInstr> McmInstrVec;
    typedef std::vector<MemoryOp> MemoryOpVec;

    MemoryOpVec sysMemOps_;  // Memory ops of all cores.
    std::vector<McmInstrVec> hartInstrVecs_; // One vector per hart.
    std::vector<MemoryOpVec> hartPendingWrites_; // One vector per hart.

    WdRiscv::System<URV>& system_;
    uint64_t time_ = 0;
    unsigned lineSize_ = 64; // Cache/merge buffer line size.

    std::vector<McmInstrIx> currentInstrTag_;
  };

}
