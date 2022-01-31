#include <iostream>
#include "Mcm.hpp"
#include "System.hpp"
#include "Hart.hpp"

using namespace TTMcm;
using namespace WdRiscv;

template <typename URV>
Mcm<URV>::Mcm(System<URV>& system)
  : system_(system)
{
  memOps_.reserve(100000);
  hartInstrVecs_.resize(system.hartCount());
  for (auto& vec : hartInstrVecs_)
    vec.reserve(100000);
}


template <typename URV>
Mcm<URV>::~Mcm()
{
}


template <typename URV>
bool
Mcm<URV>::readOp(unsigned hartId, uint64_t time, uint64_t instrTag,
		 uint64_t physAddr, unsigned size, uint64_t rtlData,
		 bool internal)
{
  if (time < time_)
    {
      std::cerr << "Error: Mcm::readOp: Backward time: "
		<< time << " < " << time_ << "\n";
      return false;
    }
  time_ = time;

  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    {
      std::cerr << "Error: Mcm::readOp: Invalid hart id: " << hartId << '\n';
      return false;
    }

  unsigned hartIx = hartPtr->sysHartIndex();
  assert(hartIx < hartInstrVecs_.size());

  MemoryOp op;
  op.time_ = time;
  op.physAddr_ = physAddr;
  op.rtlData_ = rtlData;
  op.instrTag_ = instrTag;
  op.hartIx_ = hartIx;
  op.size_ = size;
  op.read_ = true;
  op.internal_ = internal;
  if (not internal)
    {
      if (size == 1)
	{
	  uint8_t val = 0;
	  op.failRead_ = not hartPtr->peekMemory(physAddr, size, val);
	  op.data_ = val;
	}
      else if (size == 2)
	{
	  uint16_t val = 0;
	  op.failRead_ = not hartPtr->peekMemory(physAddr, size, val);
	  op.data_ = val;
	}
      else if (size == 4)
	{
	  uint32_t val = 0;
	  op.failRead_ = not hartPtr->peekMemory(physAddr, size, val);
	  op.data_ = val;
	}
      else if (size == 8)
	{
	  uint64_t val = 0;
	  op.failRead_ = not hartPtr->peekMemory(physAddr, size, val);
	  op.data_ = val;
	}
      else
	{
	  op.failRead_ = true;
	  std::cerr << "Error: Mcm::readOp: Invalid read size: " << size
		    << '\n';
	  return false;
	}
    }

  
  McmInstr* instr = findOrAddInstr(hartIx, instrTag);
  if (not instr)
    {
      assert(0);
      return false;
    }

  unsigned opCount = instr->countMemOps();
  if (opCount >= instr->maxMemOpCount())
    {
      assert(0);
      return false;
    }

  instr->memOps_[opCount] = memOps_.size();
  memOps_.push_back(op);
  
  return true;
}


template <typename URV>
McmInstr*
Mcm<URV>::findInstr(unsigned hartIx, uint32_t tag)
{
  auto& vec = hartInstrVecs_.at(hartIx);
  if (tag < vec.size() and vec.at(tag).tag_ == tag)
    return &(vec.at(tag));
  return nullptr;
}


template <typename URV>
McmInstr*
Mcm<URV>::findOrAddInstr(unsigned hartIx, uint32_t tag)
{
  auto ptr = findInstr(hartIx, tag);
  if (ptr)
    return ptr;

  auto& vec = hartInstrVecs_.at(hartIx);
  if (tag >= vec.size())
    {
      McmInstr instr;
      instr.tag_ = tag;
      vec.resize(tag + 1);
      vec.at(tag) = instr;
      return &vec.at(tag);
    }

  assert(vec.at(tag).tag_ == 0);
  vec.at(tag).tag_ = tag;
  return &vec.at(tag);
}


template <typename URV>
bool
Mcm<URV>::mergeBufferInsert(unsigned hartId, uint64_t time, uint64_t instrTag,
			    uint64_t physAddr, unsigned size,
			    uint64_t rtlData)
{
  if (time < time_)
    {
      std::cerr << "Error: Mcm::mergeBufferInsert: Backward time: "
		<< time << " < " << time_ << "\n";
      return false;
    }
  time_ = time;

  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    {
      std::cerr << "Error: Mcm::readOp: Invalid hart id: " << hartId << '\n';
      return false;
    }

  unsigned hartIx = hartPtr->sysHartIndex();
  assert(hartIx < hartInstrVecs_.size());

  MemoryOp op;
  op.time_ = time;
  op.physAddr_ = physAddr;
  op.rtlData_ = rtlData;
  op.instrTag_ = instrTag;
  op.hartIx_ = hartIx;
  op.size_ = size;
  op.read_ = false;
  op.internal_ = false;

  hartPendingWrites_.at(hartIx).push_back(op);

  return true;
}


template <typename URV>
bool
Mcm<URV>::commit(unsigned hartId, uint64_t time, uint64_t tag)
{
  if (time < time_)
    {
      std::cerr << "Error: Mcm::commit: Backward time: "
		<< time << " < " << time_ << "\n";
      return false;
    }
  time_ = time;

  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    {
      std::cerr << "Error: Mcm::commit: Invalid hart id: " << hartId << '\n';
      return false;
    }

  unsigned hartIx = hartPtr->sysHartIndex();
  assert(hartIx < hartInstrVecs_.size());

  McmInstr* instr = findOrAddInstr(hartIx, tag);
  if (not instr)
    {
      assert(0);
      return false;
    }

  // If instruction is a store, save corresponding address and written data.
  uint64_t addr = 0, value = 0;
  unsigned size = hartPtr->lastMemory(addr, value);
  if (size)
    {
      instr->size_ = size;
      instr->physAddr_ = addr;
      instr->data_ = value;
    }
  return true;
}


template <typename URV>
bool
Mcm<URV>::mergeBufferWrite(unsigned hartId, uint64_t time, uint64_t physAddr,
			   unsigned size,
			   const std::vector<uint8_t>& referenceData)
{
  assert(size == lineSize_);
  if (time < time_)
    {
      std::cerr << "Error: Mcm::mergeBufferWrite: Backward time: "
		<< time << " < " << time_ << "\n";
      return false;
    }
  time_ = time;

  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    {
      std::cerr << "Error: Mcm::mergeBufferWrite: Invalid hart id: " << hartId << '\n';
      return false;
    }

  // Read our memory. Apply pending writes. Compare to reference. Commit to
  // our memory.
  assert((physAddr % lineSize_) == 0);
  assert(referenceData.size() == size);
  

  unsigned hartIx = hartPtr->sysHartIndex();
  assert(hartIx < hartPendingWrites_.size());

  auto& writesVec = hartPendingWrites_.at(hartIx);
  for (auto& write : writesVec)
    {
      if (write.physAddr_ >= physAddr and write.physAddr_ < physAddr + lineSize_)
	{
	  write.time_ = time;
	  memOps_.push_back(write);
	  // Add op to instruction. Mark instruciton complete if all bytes covered.
	  // Remove op from writesVec
	}
    }
  
  return false;
}


template class TTMcm::Mcm<uint32_t>;
template class TTMcm::Mcm<uint64_t>;
