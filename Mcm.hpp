#pragma once

#include <vector>
#include <array>


namespace WdRiscv
{
  template <typename URV>
  class System;

  template <typename URV>
  class Hart;

  class DecodedInst;
}


namespace TTMcm
{

  struct MemoryOp
  {
    uint64_t time_ = 0;
    uint64_t physAddr_ = 0;
    uint64_t data_ = 0;
    uint64_t rtlData_ = 0;
    uint32_t instrTag_ = 0;
    uint8_t hartIx_ = 0;
    uint8_t size_ = 0;
    bool read_ = false;
    bool internal_ = false;
    bool failRead_ = false;
  };


  struct McmInstr
  {
    // memOps contains indices into an array of MemoryOp items.
    std::array<uint32_t, 4> memOps_ = {
      0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
    };
    uint64_t physAddr_ = 0;
    uint64_t data_ = 0;
    uint32_t tag_ = 0;
    uint8_t size_ = 0;
    bool retired_ = false;

    unsigned maxMemOpCount() const
    { return sizeof(memOps_) / sizeof(memOps_[0]); }

    unsigned countMemOps() const
    {
      unsigned count = 0;
      for (auto x : memOps_)
	{
	  if (x != ~uint32_t(0))
	    count++;
	  else
	    break;
	}
      return count;
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
			  unsigned size,
			  const std::vector<uint8_t>& referenceData);

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
    bool commit(unsigned hartId, uint64_t time, uint64_t instrTag);

  protected:

    bool checkRtlWrite(unsigned hartId, uint64_t time,
		       const McmInstr& instr, const MemoryOp& op);

    bool updateTime(const char* method, uint64_t time);

    McmInstr* findInstr(unsigned hartIx, uint32_t tag);

    McmInstr* findOrAddInstr(unsigned hartIx, uint32_t tag);

  private:

    typedef std::vector<McmInstr> McmInstrVec;
    typedef std::vector<MemoryOp> MemoryOpVec;

    MemoryOpVec memOps_;  // Memory ops.
    std::vector<McmInstrVec> hartInstrVecs_; // One vector per hart.
    std::vector<MemoryOpVec> hartPendingWrites_; // One vector per hart.

    WdRiscv::System<URV>& system_;
    uint64_t time_ = 0;
    unsigned lineSize_ = 64; // Cache/merge buffer line size.
  };

}
