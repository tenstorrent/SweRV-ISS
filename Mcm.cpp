#include <iostream>
#include "Mcm.hpp"
#include "System.hpp"
#include "Hart.hpp"

using namespace WdRiscv;


template <typename URV>
Mcm<URV>::Mcm(System<URV>& system, unsigned mergeBufferSize)
  : system_(system), lineSize_(mergeBufferSize)
{
  sysMemOps_.reserve(200000);

  hartInstrVecs_.resize(system.hartCount());
  for (auto& vec : hartInstrVecs_)
    vec.reserve(200000);

  hartPendingWrites_.resize(system.hartCount());
  currentInstrTag_.resize(system.hartCount());
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
Mcm<URV>::readOp(Hart<URV>& hart, uint64_t time, uint64_t instrTag,
		 uint64_t physAddr, unsigned size, uint64_t rtlData,
		 bool internal)
{
  if (not updateTime("Mcm::readOp", time))
    return false;

  unsigned hartIx = hart.sysHartIndex();
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
	  op.failRead_ = not hart.peekMemory(physAddr, val, true /*usePma*/);
	  op.data_ = val;
	}
      else if (size == 2)
	{
	  uint16_t val = 0;
	  op.failRead_ = not hart.peekMemory(physAddr, val, true /*usePma*/);
	  op.data_ = val;
	}
      else if (size == 4)
	{
	  uint32_t val = 0;
	  op.failRead_ = not hart.peekMemory(physAddr, val, true /*usePma*/);
	  op.data_ = val;
	}
      else if (size == 8)
	{
	  uint64_t val = 0;
	  op.failRead_ = not hart.peekMemory(physAddr, val, true /*usePma*/);
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
      instr->addMemOp(sysMemOps_.size());
    }
  else
    assert(0);
  sysMemOps_.push_back(op);
  
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
Mcm<URV>::mergeBufferInsert(Hart<URV>& hart, uint64_t time, uint64_t instrTag,
			    uint64_t physAddr, unsigned size,
			    uint64_t rtlData)
{
  if (not updateTime("Mcm::mergeBufferInsert", time))
    return false;

  unsigned hartIx = hart.sysHartIndex();
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
    {
      URV hartId = 0;
      hart.peekCsr(CsrNumber::MHARTID, hartId);
      return checkRtlWrite(hartId, *instr, op);
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::retire(Hart<URV>& hart, uint64_t time, uint64_t tag,
		 const DecodedInst& di)
{
  if (not updateTime("Mcm::retire", time))
    return false;

  unsigned hartIx = hart.sysHartIndex();
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

  if (not di.isValid())
    {
      cancelInstr(*instr);
      return true;
    }

  instr->retired_ = true;
  instr->di_ = di;

  // If instruction is a store, save corresponding address and written data.
  uint64_t addr = 0, value = 0;
  unsigned stSize = hart.lastStore(addr, value);
  if (stSize)
    {
      instr->size_ = stSize;
      instr->physAddr_ = addr;
      instr->data_ = value;
      instr->isStore_ = true;
    }

  URV hartId = 0;
  hart.peekCsr(CsrNumber::MHARTID, hartId);

  // Check read operations of instruction comparing RTL values to
  // meory model (whisper) values.
  for (auto opIx : instr->memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      auto& op = sysMemOps_.at(opIx);
      if (not op.isRead_)
	continue;

      if (not checkRtlRead(hartId, *instr, op))
	return false;
    }

  // Check PPO rule 3.
  if (not ppoRule3(hart, *instr))
    return false;

  return true;
}


template <typename URV>
bool
Mcm<URV>::mergeBufferWrite(Hart<URV>& hart, uint64_t time, uint64_t physAddr,
			   const std::vector<uint8_t>& rtlData)
{
  if (not updateTime("Mcm::mergeBufferWrite", time))
    return false;

  assert(rtlData.size() <= lineSize_);

  // Read our memory. Apply pending writes. Compare to reference. Commit to
  // our memory.
  assert((physAddr % lineSize_) == 0);

  unsigned hartIx = hart.sysHartIndex();
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
	  instr->addMemOp(sysMemOps_.size());
	  sysMemOps_.push_back(write);
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
      if (not hart.peekMemory(physAddr + i, byte, true /*usePma*/))
	assert(0);
      line.at(i) = byte;
    }

  for (const auto& write : coveredWrites)
    {
      assert(write.physAddr_ >= physAddr);
      assert(write.physAddr_ + write.size_ <= lineEnd);
      unsigned ix = write.physAddr_ - physAddr;
      for (unsigned i = 0; i < write.size_; ++i)
	line.at(ix+i) = ((uint8_t*) &(write.rtlData_))[i];
    }

  for (unsigned i = 0; i < lineSize_; ++i)
    hart.pokeMemory(physAddr + i, line.at(i), true);
  
  // Compare our line to RTL line.
  bool result = true;
  assert(rtlData.size() <= line.size());
  auto iterPair = std::mismatch(rtlData.begin(), rtlData.end(), line.begin());
  if (iterPair.first != rtlData.end())
    {
      URV hartId = 0;
      hart.peekCsr(CsrNumber::MHARTID, hartId);

      size_t offset = iterPair.first - rtlData.begin();
      std::cerr << "Error: Mismatch on merge buffer write time=" << time
		<< " hart-id=" << hartId << " addr=0x" << std::hex
		<< (physAddr + offset) << " rtl=0x" << unsigned(rtlData.at(offset))
		<< " whisper=0x" << unsigned(line.at(offset)) << std::dec << '\n';
      result = false;
    }

  for (size_t i = 0; i < coveredWrites.size(); ++i)
    {
      auto tag = coveredWrites.at(i).instrTag_;
      if (i > 0 and tag == coveredWrites.at(i-1).instrTag_)
	continue;
      auto instr = findInstr(hartIx, tag);
      assert(instr != nullptr);
      if (checkStoreComplete(*instr))
	instr->complete_ = true;
      if (not ppoRule1(hart, *instr))
	result = false;
    }

  return result;
}


template <typename URV>
bool
Mcm<URV>::forwardTo(const McmInstr& instr, MemoryOp& readOp, uint64_t& mask)
{
  if (mask == 0)
    return true;  // No bytes left to forward.

  if (instr.isCanceled() or not instr.isRetired() or not instr.isStore_)
    return false;

  uint64_t rol = readOp.physAddr_, roh = readOp.physAddr_ + readOp.size_ - 1;
  uint64_t il = instr.physAddr_, ih = instr.physAddr_ + instr.size_ - 1;
  if (roh < il or rol > ih)
    return false;  // no overlap

  unsigned count = 0; // Count of forwarded bytes
  for (unsigned rix = 0; rix < readOp.size_; ++rix)
    {
      uint64_t byteAddr = rol + rix;
      if (byteAddr < il or byteAddr > ih)
	continue;  // Read-op byte does not overlap instruction.

      uint64_t byteMask = uint64_t(0xff) << (rix * 8);
      if ((byteMask & mask) == 0)
	continue;  // Byte forwarded by another instruction.

      // Check if read-op byte overlaps departed write-op of instruction
      bool departed = false;
      for (const auto wopIx : instr.memOps_)
	{
	  if (wopIx >= sysMemOps_.size())
	    continue;
	  const auto& wop = sysMemOps_.at(wopIx);
	  if (wop.isRead_)
	    continue;  // Should not happen.
	  if (wop.time_ < readOp.time_)  // TBD : <= instead of < ?
	    departed = false; // Write op left core before read.
	}
      if (departed)
	{
	  // Complain
	}
      
      uint8_t byteVal = instr.data_ >> (byteAddr - il)*8;
      uint64_t aligned = uint64_t(byteVal) << 8*rix;
	
      assert((readOp.data_ & mask) == 0);
      readOp.data_ = (readOp.data_ & ~byteMask) | aligned;
      mask = mask & ~byteMask;
      count++;
      if (mask == 0)
	break;
    }

  return count > 0;
}


template <typename URV>
void
Mcm<URV>::cancelNonRetired(unsigned hartIx, uint64_t instrTag)
{
  auto& vec = hartInstrVecs_.at(hartIx);

  if (instrTag > vec.size())
    vec.resize(instrTag);

  while (instrTag and not vec.at(instrTag-1).retired_)
    cancelInstr(vec.at(--instrTag));
}


template <typename URV>
bool
Mcm<URV>::checkRtlRead(unsigned hartId, const McmInstr& instr,
		       const MemoryOp& op)
{
  assert(instr.size_ > 0);
  assert(op.size_ <= instr.size_);
  if (op.rtlData_ != op.data_)
    {
      std::cerr << "Error: RTL/whisper read mismatch time=" << op.time_
		<< " hart-id=" << hartId << " instr-tag=0x" << std::hex
		<< op.instrTag_ << " addr=0x" << std::hex << op.physAddr_
		<< " size=" << unsigned(op.size_) << " rtl=0x" << op.rtlData_
		<< " whisper=0x" << op.data_ << std::dec << '\n';
      return false;
    }
  return true;
}


template <typename URV>
bool
Mcm<URV>::checkRtlWrite(unsigned hartId, const McmInstr& instr,
			const MemoryOp& op)
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

  std::cerr << "Error: RTL/whisper write mismatch time=" << op.time_
	    << " hart-id=" << hartId << " instr-tag=0x" << std::hex
	    << instr.tag_ << " addr=0x" << std::hex << op.physAddr_
	    << " size=" << unsigned(op.size_) << " rtl=0x" << op.rtlData_
	    << " whisper=0x" << data << std::dec << '\n';
  return false;
}


template <typename URV>
void
Mcm<URV>::clearMaskBitsForWrite(const McmInstr& storeInstr,
				const McmInstr& target, uint64_t mask) const
{
  /// Clear in the given mask, bits corresponding to the target instruction
  /// bytes covered by the given store instruction writes.
  uint64_t storeMask = 0; // Mask of bits written by store.
  for (auto opIx : storeInstr.memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      auto& op = sysMemOps_.at(opIx);
      if (op.isRead_)
	continue;

      if (op.time_ > latestOpTime(target))
	continue;  // Write op too late to affect target instruction.

      uint64_t opMask = ~uint64_t(0);
      if (op.physAddr_ <= target.physAddr_)
	{
	  uint64_t offset = target.physAddr_ - op.physAddr_;
	  if (offset > 8)
	    offset = 8;
	  opMask >>= offset*8;
	}
      else
	{
	  uint64_t offset = op.physAddr_ - target.physAddr_;
	  if (offset > 8)
	    offset = 8;
	  opMask <<= offset*8;
	}
      storeMask |= mask;
    }

  unsigned unused = (8 - storeInstr.size_)*8;  // Unwritten upper bits of store.
  storeMask = (storeMask << unused) >> unused;

  mask &= storeMask;
}


template <typename URV>
bool
Mcm<URV>::checkStoreComplete(const McmInstr& instr) const
{
  if (instr.isCanceled() or not instr.isStore_)
    return false;

  uint64_t mergeMask = 0;
  for (auto opIx : instr.memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      auto& op = sysMemOps_.at(opIx);
      if (op.isRead_)
	continue;

      uint64_t mask = ~uint64_t(0);
      if (op.physAddr_ <= instr.physAddr_)
	{
	  uint64_t offset = instr.physAddr_ - op.physAddr_;
	  if (offset > 8)
	    offset = 8;
	  mask >>= offset*8;
	}
      else
	{
	  uint64_t offset = op.physAddr_ - instr.physAddr_;
	  if (offset > 8)
	    offset = 8;
	  mask <<= offset*8;
	}
      mergeMask |= mask;
    }

  unsigned unused = (8 - instr.size_)*8;  // Unused upper bits of value.
  mergeMask = (mergeMask << unused) >> unused;

  uint64_t expectedMask = (~uint64_t(0) << unused) >> unused;
  return mergeMask == expectedMask;
}


template <typename URV>
bool
Mcm<URV>::setCurrentInstruction(Hart<URV>& hart, uint64_t tag)
{
  unsigned hartIx = hart.sysHartIndex();
  currentInstrTag_.at(hartIx) = tag;
  return true;
}


template <typename URV>
bool
Mcm<URV>::getCurrentLoadValue(Hart<URV>& hart, uint64_t addr,
			      unsigned size, uint64_t& value)
{
  value = 0;
  if (size == 0 or size > 8)
    {
      assert(0);
      return false;
    }

  unsigned hartIx = hart.sysHartIndex();
  uint64_t tag = currentInstrTag_.at(hartIx);

  McmInstr* instr = findInstr(hartIx, tag);
  if (not instr or instr->isCanceled())
    return false;

  // We expect Mcm::retire to be called after this method is called.
  if (instr->isRetired())
    {
      assert(0);
      return false;
    }

  instr->size_ = size;
  instr->physAddr_ = addr;

  // Merge read operation values.
  uint64_t mergeMask = 0;
  uint64_t merged = 0;
  for (auto opIx : instr->memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      auto& op = sysMemOps_.at(opIx);
      if (not op.isRead_)
	continue;

      if (op.internal_)
	if (not forwardToRead(hart, tag, op))
	  return false;

      uint64_t opVal = op.data_;
      uint64_t mask = ~uint64_t(0);
      if (op.physAddr_ <= addr)
	{
	  uint64_t offset = addr - op.physAddr_;
	  if (offset > 8)
	    offset = 8;
	  opVal >>= offset*8;
	  mask >>= offset*8;
	}
      else
	{
	  uint64_t offset = op.physAddr_ - addr;
	  if (offset > 8)
	    offset = 8;
	  opVal <<= offset*8;
	  mask <<= offset*8;
	}
      merged |= (opVal & mask);
      mergeMask |= mask;
    }

  unsigned unused = (8 - size)*8;  // Unused upper bits of value.
  value = (merged << unused) >> unused;
  mergeMask = (mergeMask << unused) >> unused;

  uint64_t expectedMask = (~uint64_t(0) << unused) >> unused;
  if (mergeMask != expectedMask)
    {
      std::cerr << "Error: Read ops do not cover all the bytes of load instruction tag=0x" << std::hex << tag << std::dec << '\n';
      return false;
    }
  else
    instr->complete_ = true;

  return true;
}
  

template <typename URV>
bool
Mcm<URV>::forwardToRead(Hart<URV>& hart, uint64_t tag, MemoryOp& op)
{
  if (not op.internal_)
    return false;

  unsigned hartIx = hart.sysHartIndex();

  uint64_t mask = (~uint64_t(0)) >> (8 - op.size_)*8;
  const auto& instrVec = hartInstrVecs_.at(hartIx);
  for (McmInstrIx ix = tag; ix > 0 and mask != 0; --ix)
    forwardTo(instrVec.at(ix-1), op, mask);
  if (mask != 0)
    {
      using std::cerr;
      URV hartId = 0;
      hart.peekCsr(CsrNumber::MHARTID, hartId);
      std::cerr << "Error: Internal read does not forward from preceeding stores"
		<< " time=" << op.time_ << " hart-id=" << hartId
		<< " instr-tag=0x" << std::hex << tag << " addr=0x"
		<< op.physAddr_ << std::dec << '\n';
      return false;
    }
  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule1(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Check ppo rule 1 for all the write operations asociated with
  // the given store instruction B.
  
  if (not instrB.complete_)
    return true;  // We will try again when B is complete.

  unsigned hartIx = hart.sysHartIndex();
  URV hartId = 0;
  hart.peekCsr(CsrNumber::MHARTID, hartId);

  const auto& instrVec = hartInstrVecs_.at(hartIx);

  for (McmInstrIx tag = instrB.tag_; tag > 0; --tag)
    {
      const auto& instrA =  instrVec.at(tag-1);
      if (instrA.isCanceled())
	continue;
      assert(instrA.isRetired());

      if (not instrA.isMemory() or not instrA.overlaps(instrB))
	continue;

      if (isBeforeInMemoryTime(instrA, instrB))
	continue;

      std::cerr << "Error: PPO rule 1 failed: hart-id=" << hartId << " tag1="
		<< instrA.tag_ << " tag2=" << instrB.tag_ << '\n';
      return false;
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule3(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 3: A is a write resulting from an AMO/SC instructions, A and
  // B have overlapping addresses, B loads data from A.
 
  assert(instrB.di_.isValid());

  // Instruction B must be a load/amo instruction.
  const InstEntry* instEntry = instrB.di_.instEntry();
  if (instEntry->isStore())
    return true;  // NA: store instruction.

  if (not instEntry->isLoad() and not instEntry->isAtomic())
    return true;  // NA: must be load/amo.

  if (not instrB.complete_)
    return true;  // We will try again when B is complete.

  unsigned hartIx = hart.sysHartIndex();
  URV hartId = 0;
  hart.peekCsr(CsrNumber::MHARTID, hartId);

  const auto& instrVec = hartInstrVecs_.at(hartIx);

  using std::cerr;

  uint64_t mask = ~uint64_t(0);  // Bites of B not-written by preceeding non-atomic stores.
  unsigned shift = (8 - instrB.size_) * 8;
  mask = (mask << shift) >> shift;

  for (McmInstrIx tag = instrB.tag_; tag > 0; --tag)
    {
      const auto& instrA =  instrVec.at(tag-1);
      if (instrA.isCanceled())
	continue;
      assert(instrA.isRetired());

      if (not instrA.isStore_ or not instrA.overlaps(instrB))
	continue;

      // If A is not atomic remove from mask the bytes of B that are
      // covered by A. Done when all bytes of B are covered.
      assert(instrA.di_.isValid());
      if (not instrA.di_.instEntry()->isAtomic())
	{
	  clearMaskBitsForWrite(instrA, instrB, mask);
	  if (mask == 0)
	    return true;
	}
      else if (not isBeforeInMemoryTime(instrA, instrB))
	{
	  cerr << "Error: PPO rule 3 failed: hart-id=" << hartId << " tag1="
	       << instrA.tag_ << " tag2=" << instrB.tag_ << " time1="
	       << latestOpTime(instrA) << " time2=" << earliestOpTime(instrB)
	       << '\n';
	  return false;
	}
    }

  return true;
}


template class WdRiscv::Mcm<uint32_t>;
template class WdRiscv::Mcm<uint64_t>;
