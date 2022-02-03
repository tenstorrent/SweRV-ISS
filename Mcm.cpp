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
  memOps_.reserve(200000);
  hartInstrVecs_.resize(system.hartCount());
  for (auto& vec : hartInstrVecs_)
    vec.reserve(200000);
}


template <typename URV>
Mcm<URV>::~Mcm()
{
}


template <typename URV>
inline
bool
Mcm<URV>::updateTime(const char* method, uint64_t time)
{
  if (time < time_)
    {
      std::cerr << "Error: " << method << ": Backward time: "
		<< time << " < " << time_ << "\n";
      return false;
    }
  time_ = time;
  return true;
}


template <typename URV>
bool
Mcm<URV>::readOp(unsigned hartId, uint64_t time, uint64_t instrTag,
		 uint64_t physAddr, unsigned size, uint64_t rtlData,
		 bool internal)
{
  if (not updateTime("Mcm::readOp", time))
    return false;

  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    {
      std::cerr << "Error: Mcm::readOp: Invalid hart id: " << hartId << '\n';
      return false;
    }

  unsigned hartIx = hartPtr->sysHartIndex();
  assert(hartIx < hartInstrVecs_.size());

  MemoryOp op = {};
  op.time_ = time;
  op.physAddr_ = physAddr;
  op.rtlData_ = rtlData;
  op.instrTag_ = instrTag;
  op.hartIx_ = hartIx;
  op.size_ = size;
  op.isRead_ = true;
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
  if (instr)
    {
      if (instr->isCanceled())
	op.cancel();
      instr->addMemOp(memOps_.size());
    }
  else
    assert(0);
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
  if (not updateTime("Mcm::mergeBufferInsert", time))
    return false;

  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    {
      std::cerr << "Error: Mcm::readOp: Invalid hart id: " << hartId << '\n';
      return false;
    }

  unsigned hartIx = hartPtr->sysHartIndex();
  assert(hartIx < hartInstrVecs_.size());

  MemoryOp op = {};
  op.time_ = time;
  op.physAddr_ = physAddr;
  op.rtlData_ = rtlData;
  op.instrTag_ = instrTag;
  op.hartIx_ = hartIx;
  op.size_ = size;
  op.isRead_ = false;
  op.internal_ = false;

  hartPendingWrites_.at(hartIx).push_back(op);

  // If corresponding insruction is retired, compare to its data.
  McmInstr* instr = findOrAddInstr(hartIx, instrTag);
  assert(instr);
  if (not instr)
    return false;
  if (instr->retired_)
    return checkRtlWrite(hartId, time, *instr, op);

  return true;
}


template <typename URV>
bool
Mcm<URV>::retire(unsigned hartId, uint64_t time, uint64_t tag)
{
  if (not updateTime("Mcm::retire", time))
    return false;

  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    {
      std::cerr << "Error: Mcm::retire: Invalid hart id: " << hartId << '\n';
      return false;
    }

  unsigned hartIx = hartPtr->sysHartIndex();
  assert(hartIx < hartInstrVecs_.size());

  cancelNonRetired(hartIx, tag);

  McmInstr* instr = findOrAddInstr(hartIx, tag);
  if (not instr)
    {
      assert(0);
      return false;
    }

  if (instr->retired_)
    {
      std::cerr << "Mcm::retire: Error: Instruction tag " << tag
		<< " retired multiple times\n";
      return false;
    }
  instr->retired_ = true;

  // If instruction is a store, save corresponding address and written data.
  uint64_t addr = 0, value = 0;
  unsigned stSize = hartPtr->lastStore(addr, value);
  if (stSize)
    {
      instr->size_ = stSize;
      instr->physAddr_ = addr;
      instr->data_ = value;
      instr->isStore_ = true;
    }

  // Check read operations of instruction.
  for (auto opIx : instr->memOps_)
    {
      if (opIx >= memOps_.size())
	continue;
      const auto& op = memOps_.at(opIx);
      if (op.isRead_)
	{
	  if (op.internal_)
	    {
	      assert(0 and "Implement forwarding.");
	    }
	  else
	    {
	      if (op.rtlData_ != op.data_)
		{
		  std::cerr << "Error: RTL/whisper read mismatch time=" << time
			    << " hart-id=" << hartId << " instr-tag=0x" << std::hex
			    << op.instrTag_ << " addr=0x" << std::hex << op.physAddr_
			    << " size=" << op.size_ << " rtl=0x" << op.rtlData_
			    << " whisper=0x" << op.data_ << std::dec << '\n';
		  return false;
		}
	    }
	}
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::mergeBufferWrite(unsigned hartId, uint64_t time, uint64_t physAddr,
			   const std::vector<uint8_t>& rtlData)
{
  if (not updateTime("Mcm::mergeBufferWrite", time))
    return false;

  assert(rtlData.size() == lineSize_);

  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    {
      std::cerr << "Error: Mcm::mergeBufferWrite: Invalid hart id: " << hartId << '\n';
      return false;
    }

  // Read our memory. Apply pending writes. Compare to reference. Commit to
  // our memory.
  assert((physAddr % lineSize_) == 0);
  assert(rtlData.size() == lineSize_);

  unsigned hartIx = hartPtr->sysHartIndex();
  assert(hartIx < hartPendingWrites_.size());

  std::vector<MemoryOp> coveredWrites;

  uint64_t lineEnd = physAddr + lineSize_;

  // Collect pending-writes matching merge buffer address in coveredWrites
  // removing them from the pending-write vector.
  size_t pendingSize = 0;  // Pending size after removal of matchign writes
  auto& writesVec = hartPendingWrites_.at(hartIx);
  for (size_t i = 0; i < writesVec.size(); ++i)
    {
      auto& write = writesVec.at(i);
      if (write.physAddr_ >= physAddr and write.physAddr_ < lineEnd)
	{
	  write.time_ = time;
	  assert(write.physAddr_ + write.size_  <= lineEnd);
	  McmInstr* instr = findOrAddInstr(hartIx, write.instrTag_);
	  assert(instr);
	  assert(not instr->isCanceled());
	  instr->addMemOp(memOps_.size());
	  memOps_.push_back(write);
	  coveredWrites.push_back(write);
	}
      else
	{
	  if (i != pendingSize)
	    writesVec.at(pendingSize) = writesVec.at(i);
	  pendingSize++;
	}
      writesVec.resize(pendingSize);
    }

  std::sort(coveredWrites.begin(), coveredWrites.end(),
	    [](const MemoryOp& a, const MemoryOp& b) {
	      return a.instrTag_ < b.instrTag_;
	    });

  std::vector<uint8_t> line(lineSize_, 0);
  for (unsigned i = 0; i < lineSize_; ++i)
    {
      uint8_t byte = 0;
      if (not hartPtr->peekMemory(physAddr + i, byte, true /*usePma*/))
	assert(0);
      line.at(i) = byte;
    }

  for (const auto& write : coveredWrites)
    {
      assert(write.physAddr_ >= physAddr);
      assert(write.physAddr_ + write.size_ <= lineEnd);
      unsigned ix = write.physAddr_ - physAddr;
      for (unsigned i = 0; i < write.size_; ++i)
	line.at(ix+i) = ((uint8_t*) write.rtlData_)[i];
    }

  for (unsigned i = 0; i < lineSize_; ++i)
    hartPtr->pokeMemory(physAddr + i, line.at(i), true);
  
  // Compare our line to RTL line.
  bool result = true;
  assert(line.size() == rtlData.size());
  auto iterPair = std::mismatch(line.begin(), line.end(), rtlData.begin());
  if (iterPair.first != line.end())
    {
      size_t offset = iterPair.first - line.begin();
      std::cerr << "Error: Mismatch on merge buffer write time=" << time
		<< " hart-id=" << hartId << " addr=0x" << std::hex
		<< (physAddr + offset) << " rtl=0x" << rtlData.at(offset)
		<< " whisper=0x" << line.at(offset) << std::dec << '\n';
      result = false;
    }

  return result;
}


template <typename URV>
void
Mcm<URV>::cancelNonRetired(unsigned hartIx, uint64_t instrTag)
{
  auto& vec = hartInstrVecs_.at(hartIx);
  if (vec.empty())
    return;

  instrTag += 1;  // To count backward.

  if (instrTag > vec.size())
    instrTag = vec.size();

  while (instrTag and not vec.at(instrTag-1).retired_)
    cancelInstr(vec.at(--instrTag));
}


template <typename URV>
bool
Mcm<URV>::checkRtlWrite(unsigned hartId, uint64_t time,
			const McmInstr& instr, const MemoryOp& op)
{
  assert(instr.size_ > 0);
  assert(op.size_ <= instr.size_);
  uint64_t data = instr.data_;
  if (op.size_ <= instr.size_)
    {
      unsigned shift = 64 - (op.size_*8);
      data = (data << shift) >> shift;
    }
  if (data == op.rtlData_)
    return true;

  std::cerr << "Error: RTL/whisper write mismatch time=" << time
	    << " hart-id=" << hartId << " instr-tag=0x" << std::hex
	    << instr.tag_ << " addr=0x" << std::hex << op.physAddr_
	    << " size=" << op.size_ << " rtl=0x" << op.rtlData_
	    << " whisper=0x" << data << std::dec << '\n';
  return false;
}


// FIX TODO When a write is seen, check its data against instruction
// if instruction is already retired.

template class TTMcm::Mcm<uint32_t>;
template class TTMcm::Mcm<uint64_t>;
