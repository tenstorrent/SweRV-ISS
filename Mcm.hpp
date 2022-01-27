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
    uint64_t address_ = 0;
    uint64_t data_ = 0;
    uint64_t rtlData_ = 0;
    uint32_t instTag_ = 0;
    uint8_t hart_ = 0;
    uint8_t size_ = 0;
    bool read_ = false;
    bool internal_ = false;
    bool failRead_ = false;
  };


  struct McmInst
  {
    WdRiscv::DecodedInst* di_ = nullptr;
    std::array<MemoryOp*, 4> memOps_ = { nullptr, nullptr, nullptr, nullptr };
    uint32_t tag_ = 0;
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
    bool earlyRead(unsigned HartId, uint64_t instTag, uint64_t physAddr,
		   unsigned size, uint64_t rtlData, bool internal);

    /// Initiate a merge buffer write.  All associated store write
    /// transactions are marked completed. Write instructions where all
    /// write are complete are marked complete. Return true on success.
    bool mergeBufferWrite(uint64_t physAddr, unsigned size,
			  const std::vector<uint8_t>& referenceData);

    /// Insert a write operation for the given instruction into the
    /// merge buffer removing it from the store buffer. Return true on
    /// success. Return false if no such operation is in the store
    /// buffer.
    bool mergeBufferInsert(unsigned hartId, uint64_t instTag,
			   uint64_t physAddr, unsigned size,
			   uint64_t referenceData);

    /// Cancel all the early-read transaction associted with the given
    /// tag. This is done when a speculative load instruction is canceled.
    bool cancelRead(unsigned hartId, uint64_t instTag);

    /// This is called when a load/store instruction is retired.
    bool commit(unsigned HartId, uint64_t instTag);

  protected:

    bool findInst(unsigned hartId, uint32_t tag, McmInst& inst);

  private:
    WdRiscv::System<URV>& system_;
  };

}
