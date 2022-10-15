#include <iostream>
#include "Mcm.hpp"
#include "System.hpp"
#include "Hart.hpp"

using namespace WdRiscv;
using std::cerr;


template <typename URV>
Mcm<URV>::Mcm(unsigned hartCount, unsigned mergeBufferSize)
  : lineSize_(mergeBufferSize)
{
  sysMemOps_.reserve(200000);

  hartInstrVecs_.resize(hartCount);
  for (auto& vec : hartInstrVecs_)
    vec.reserve(200000);

  hartPendingWrites_.resize(hartCount);
  currentInstrTag_.resize(hartCount);

  hartRegTimes_.resize(hartCount);
  for (auto& vec : hartRegTimes_)
    vec.resize(totalRegCount_);

  hartRegProducers_.resize(hartCount);
  for (auto& vec : hartRegProducers_)
    vec.resize(totalRegCount_);

  hartBranchTimes_.resize(hartCount);
  hartBranchProducers_.resize(hartCount);

  // If no merge buffer, then memory is updated on insert messages.
  writeOnInsert_ = (lineSize_ == 0);
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
      cerr << "Error: " << method << ": Backward time: "
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
  if (hartIx >= hartInstrVecs_.size())
    {
      cerr << "Mcm::readOp: Error: Hart ix out of bound\n";
      assert(0 && "Mcm::readOp: Hart ix out of bound");
    }

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
	  cerr << "Error: Mcm::readOp: Invalid read size: " << size << '\n';
	  return false;
	}
    }

  McmInstr* instr = findOrAddInstr(hartIx, instrTag);

  bool io = false;  // FIX  get io from PMA of address.
  if (instr->isCanceled())
    op.cancel();
  else if (instr->isRetired() and not io)
    cerr << "Warning: Read op time=" << op.time_ << " occurs after "
	 << "instruction retires tag=" << instr->tag_ << '\n';

  instr->addMemOp(sysMemOps_.size());
  sysMemOps_.push_back(op);

  bool result = true;
  if (checkLoadComplete(*instr))
    {
      instr->complete_ = true;
      if (instr->retired_)
	if (not ppoRule3(hart, *instr))
	  result = false;
    }
  
  return result;
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
      if (tag > 100000000)
	{
	  std::cerr << "MCM: Instruction tag way too large: " << tag << '\n';
	  std::cerr << "MCM: Code expects dense consecutive tags starting at 0\n";
	  assert(0);
	}
      McmInstr instr;
      instr.tag_ = tag;
      vec.resize(tag + 1);
      vec.at(tag) = instr;
      return &vec.at(tag);
    }

  if (vec.at(tag).tag_ != 0)
    {
      cerr << "Mcm::findOrAddInstr: Error: Instr tag already in use\n";
      assert(0 && "Mcm::findOrAddInstr: Instr tag already in use");
    }

  vec.at(tag).tag_ = tag;
  return &vec.at(tag);
}


template <typename URV>
void
Mcm<URV>::updateDependencies(const Hart<URV>& hart, const McmInstr& instr)
{
  if (not instr.retired_)
    {
      cerr << "Mcm::updateDependencies: Error: Instruction is not retired\n";
      assert(0 && "Mcm::updateDependencies: Instruction is not retired");
    }

  unsigned hartIx = hart.sysHartIndex();
  auto& regTimeVec = hartRegTimes_.at(hartIx);
  auto& regProducer = hartRegProducers_.at(hartIx);

  const DecodedInst& di = instr.di_;
  if (not di.isValid())
    {
      cerr << "Mcm::updateDependencies: Error: Decoded instr is invalid\n";
      assert(0 && "Mcm::updateDependencies: Decoded instr is invalid");
    }
  if (di.operandCount() == 0)
    return;

  const auto instEntry = di.instEntry();

  if (instEntry->isIthOperandIntRegDest(0) and di.ithOperand(0) == 0)
    return; // Destination is x0.

  uint64_t time = 0, tag = 0;

  bool hasDep = true;
  if (instEntry->isSc())
    {
      URV val = 0;
      hart.peekIntReg(di.op0(), val);
      if (val == 1)
	return;  // store-conditional failed.
      if (instr.memOps_.size() == 0)
	{
	  tag = instr.tag_;
	  time = ~uint64_t(0); // Will be updated when SC drains to memory.
	}
    }
  else if (instEntry->isStore())
    return;   // No destination register.
  else if (di.isLoad() or di.isAmo())
    hasDep = false;
  else if (di.isBranch())
    hasDep = false;

  for (const auto& opIx : instr.memOps_)
    if (opIx < sysMemOps_.size() and sysMemOps_.at(opIx).time_ > time)
      {
	time = sysMemOps_.at(opIx).time_;
	tag = instr.tag_;
      }

  if (di.isBranch())
    hartBranchTimes_.at(hartIx) = 0;

  std::vector<unsigned> sourceRegs, destRegs;
  identifyRegisters(di, sourceRegs, destRegs);

  bool first = true; // first branch source
  for (auto regIx : sourceRegs)
    {
      if (hasDep and regTimeVec.at(regIx) > time)
	{
	  time = regTimeVec.at(regIx);
	  tag = regProducer.at(regIx);
	}
      if (di.isBranch())
	if (first or regTimeVec.at(regIx) > hartBranchTimes_.at(hartIx))
	  {
	    first = false;
	    hartBranchTimes_.at(hartIx) = regTimeVec.at(regIx);
	    hartBranchProducers_.at(hartIx) = regProducer.at(regIx);
	  }
    }

  bool noSource = sourceRegs.empty();
  if (noSource)
    assert(tag == 0 and time == 0);

  for (auto regIx : destRegs)
    {
      bool update = noSource or time > regTimeVec.at(regIx);
      if (update)
	{
	  regTimeVec.at(regIx) = time;
	  regProducer.at(regIx) = tag;
	}
    }
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

  MemoryOp op = {};
  op.time_ = time;
  op.physAddr_ = physAddr;
  op.rtlData_ = rtlData;
  op.instrTag_ = instrTag;
  op.hartIx_ = hartIx;
  op.size_ = size;
  op.isRead_ = false;
  op.internal_ = false;

  if (not writeOnInsert_)
    hartPendingWrites_.at(hartIx).push_back(op);

  // If corresponding insruction is retired, compare to its data.
  McmInstr* instr = findOrAddInstr(hartIx, instrTag);
  if (not instr)
    {
      cerr << "Mcm::MergeBufferInsert: Error: Unknown instr tag\n";
      assert(0 && "Mcm::MergeBufferInsert: Unknown instr tag");
      return false;
    }

  bool result = true;

  if (writeOnInsert_)
    {
      // Associate write op with instruction.
      instr->addMemOp(sysMemOps_.size());
      sysMemOps_.push_back(op);
      if (checkStoreComplete(*instr))
	{
	  instr->complete_ = true;
	  if (instr->retired_)
	    if (not ppoRule1(hart, *instr))
	      result = false;
	}

      // Commit write to memory. 
      if (op.size_ == 1)
	hart.pokeMemory(physAddr, uint8_t(rtlData), true);
      else if (op.size_ == 2)
	hart.pokeMemory(physAddr, uint16_t(rtlData), true);
      else if (op.size_ == 4)
	hart.pokeMemory(physAddr, uint32_t(rtlData), true);
      else if (op.size_ == 8)
	hart.pokeMemory(physAddr, uint64_t(rtlData), true);
      else
	{
	  cerr << "Mcm::MergeBufferInsert: Error: write size not a power of 2\n";
	  assert(0 && "Mcm::MergeBufferInsert: write size not a power of 2");
	}
    }

  if (instr->retired_)
    if (not checkRtlWrite(hart.hartId(), *instr, op))
      result = false;

  return result;
}


template <typename URV>
bool
Mcm<URV>::retire(Hart<URV>& hart, uint64_t time, uint64_t tag,
		 const DecodedInst& di)
{
  if (not updateTime("Mcm::retire", time))
    return false;

  unsigned hartIx = hart.sysHartIndex();

  cancelNonRetired(hartIx, tag);

  McmInstr* instr = findOrAddInstr(hartIx, tag);
  if (instr->retired_)
    {
      cerr << "Mcm::retire: Error: Instruction tag=" << tag
	   << " retired multiple times\n";
      return false;
    }

  if (not di.isValid())
    {
      cancelInstr(*instr);  // Instruction took a trap.
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

  URV hartId = hart.hartId();

  // Check read operations of instruction comparing RTL values to
  // memory model (whisper) values.
  for (auto opIx : instr->memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      auto& op = sysMemOps_.at(opIx);
      if (not op.isRead_)
	continue;

      if (not checkRtlRead(hartId, *instr, op))
	{
	  updateDependencies(hart, *instr);
	  return false;
	}
    }

  // Amo sanity check.
  if (di.instEntry()->isAmo())
    {
      // Must have a read.  Must not have a write.
      if (not instrHasRead(*instr))
	{
	  cerr << "Error: Amo instruction tag=" << tag
	       << " retired before read op.\n";
	  updateDependencies(hart, *instr);
	  return false;
	}
#if 0
      if (instrHasWrite(*instr))
	{
	  cerr << "Error: Amo instruction tag=" << tag
	       << " retired after write op.\n";
	  return false;
	}
#endif
      instr->isStore_ = true;  // AMO is both load and store.
    }

  bool ok = ppoRule2(hart, *instr);
  ok = ppoRule3(hart, *instr) and ok;
  ok = ppoRule4(hart, *instr) and ok;
  ok = ppoRule5(hart, *instr) and ok;
  ok = ppoRule6(hart, *instr) and ok;
  ok = ppoRule8(hart, *instr) and ok;
  ok = ppoRule9(hart, *instr) and ok;
  ok = ppoRule10(hart, *instr) and ok;
  ok = ppoRule11(hart, *instr) and ok;
  ok = ppoRule12(hart, *instr) and ok;

  updateDependencies(hart, *instr);

  return ok;
}


static void
reportMismatch(uint64_t hartId, uint64_t time, std::string tag, uint64_t addr,
	       uint64_t rtlData, uint64_t whisperData)
{
  cerr << "Error: Mismatch on " << tag << " time=" << time
	    << " hart-id=" << hartId << " addr=0x" << std::hex
	    << addr << " rtl=0x" << rtlData
	    << " whisper=0x" << whisperData << std::dec << '\n';
}


static bool
checkBufferWriteParams(unsigned lineSize, unsigned rtlLineSize, uint64_t physAddr)
{
  if (lineSize == 0)
    {
      cerr << "Merge buffer write attempted when merge buffer is disabled\n";
      return false;
    }

  if (rtlLineSize > lineSize)
    {
      cerr << "Merge buffer write: RTL line size (" << rtlLineSize << ") greater than"
	   << " cache line size (" << lineSize << ")\n";
      return false;
    }

  if ((physAddr % lineSize) != 0)
    {
      cerr << "Merge buffer write: address (0x" << std::hex << physAddr << ") "
	   << "not a multiple of line size (" << std::dec << lineSize << ")\n";
      return false;
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::collectCoveredWrites(Hart<URV>& hart, uint64_t time, uint64_t lineBegin,
			       MemoryOpVec& coveredWrites)
{
  unsigned hartIx = hart.sysHartIndex();
  auto& pendingWrites = hartPendingWrites_.at(hartIx);
  size_t pendingSize = 0;  // pendingWrite size after removal of matching writes

  uint64_t lineEnd = lineBegin + lineSize_;

  for (size_t i = 0; i < pendingWrites.size(); ++i)
    {
      auto& write = pendingWrites.at(i);
      if (write.physAddr_ >= lineBegin and write.physAddr_ < lineEnd)
	{
	  write.time_ = time;
	  if (write.physAddr_ + write.size_  > lineEnd)
	    {
	      cerr << "Mcm::mergeBufferWrite: Error: Store address out of line bounds\n";
	      return false;
	    }
	  McmInstr* instr = findOrAddInstr(hartIx, write.instrTag_);
	  if (not instr or instr->isCanceled())
	    {
	      cerr << "Mcm::mergeBufferWrite: Write for an invalid store\n";
	      return false;
	    }
	  instr->addMemOp(sysMemOps_.size());
	  sysMemOps_.push_back(write);
	  coveredWrites.push_back(write);
	}
      else
	{
	  if (i != pendingSize)
	    pendingWrites.at(pendingSize) = pendingWrites.at(i);
	  pendingSize++;
	}
    }
  pendingWrites.resize(pendingSize);

  std::sort(coveredWrites.begin(), coveredWrites.end(),
	    [](const MemoryOp& a, const MemoryOp& b) {
	      return a.instrTag_ < b.instrTag_;
	    });
  return true;
}


template <typename URV>
bool
Mcm<URV>::mergeBufferWrite(Hart<URV>& hart, uint64_t time, uint64_t physAddr,
			   const std::vector<uint8_t>& rtlData,
			   const std::vector<bool>& rtlMask)
{
  if (not updateTime("Mcm::mergeBufferWrite", time))
    return false;
  if (not checkBufferWriteParams(lineSize_, rtlData.size(), physAddr))
    return false;

  unsigned hartIx = hart.sysHartIndex();

  // Remove from hartPendingWrites_ the writes matching the RTL line
  // addresses and place them sorted by instr tag in coveredWrites.
  std::vector<MemoryOp> coveredWrites;
  if (not collectCoveredWrites(hart, time, physAddr, coveredWrites))
    return false;

  // Read our memory corresponding to RTL line addresses.
  std::vector<uint8_t> line(lineSize_, 0);
  for (unsigned i = 0; i < lineSize_; ++i)
    {
      uint8_t byte = 0;
      if (not hart.peekMemory(physAddr + i, byte, true /*usePma*/))
	{
	  cerr << "Mcm::mergeBufferWrite: Failed to query memory\n";
	  return false;
	}
      line.at(i) = byte;
    }

  // Apply pending writes to our line. Compute mask of updated bytes.
  std::vector<bool> mask(lineSize_);
  uint64_t lineEnd = physAddr + lineSize_;
  for (const auto& write : coveredWrites)
    {
      if ((write.physAddr_ < physAddr) or (write.physAddr_ + write.size_ > lineEnd))
	{
	  cerr << "Mcm::mergeBufferWrite: Store address out of line bound\n";
	  return false;
	}
      unsigned ix = write.physAddr_ - physAddr;
      for (unsigned i = 0; i < write.size_; ++i)
	{
	  line.at(ix+i) = ((uint8_t*) &(write.rtlData_))[i];
	  mask.at(ix+i) = true;
	}
    }

  // Put our line back in memory (use poke words to accomodate clint).
  for (unsigned i = 0; i < lineSize_; i += 4)
    {
      uint32_t word = *((uint32_t*) (line.data() + i));
      hart.pokeMemory(physAddr + i, word, true);
    }
  
  // Compare our line to RTL line.
  bool result = true;
  if (checkWholeLine_)
    {
      auto iterPair = std::mismatch(rtlData.begin(), rtlData.end(), line.begin());
      if (iterPair.first != rtlData.end())
	{
	  size_t offset = iterPair.first - rtlData.begin();
	  reportMismatch(hart.hartId(), time, "merge buffer write", physAddr + offset,
			 rtlData.at(offset), line.at(offset));
	  result = false;
	}
    }
  else
    {   // Compare covered writes.
      for (unsigned i = 0; i < lineSize_; ++i)
	{
	  if (mask.at(i) and (line.at(i) != rtlData.at(i)))
	    {
	      reportMismatch(hart.hartId(), time, "merge buffer write", physAddr + i,
			     rtlData.at(i), line.at(i));
	      result = false;
	      break;
	    }
	  if (not rtlMask.empty() and i < rtlMask.size() and mask.at(i) != rtlMask.at(i))
	    {
	      cerr << "Error: Mismatch on merge buffer update time=" << time
		   << " hart-id=" << hart.hartId() << " addr=0x" << std::hex
		   << (physAddr + i) << std::dec << " rtl=" << rtlMask.at(i)
		   << " whisper=" << mask.at(i) << '\n';
	      result = false;
	      break;
	    }
	}
    }

  auto& instrVec = hartInstrVecs_.at(hartIx);

  for (size_t i = 0; i < coveredWrites.size(); ++i)
    {
      auto tag = coveredWrites.at(i).instrTag_;
      if (i > 0 and tag == coveredWrites.at(i-1).instrTag_)
	continue;
      auto instr = findInstr(hartIx, tag);
      if (not instr)
	{
	  cerr << "Mcm::mergeBufferWrite: Covered instruction tag is invalid\n";
	  return false;
	}
      if (checkStoreComplete(*instr))
	{
	  instr->complete_ = true;
	  if (not ppoRule1(hart, *instr))
	    result = false;
	}
      if (instr->retired_ and instr->di_.instEntry()->isSc())
	{
	  if (not instr->complete_)
	    {
	      cerr << "Mcm::mergeBufferWrite: sc instruction written before complete\n";
	      return false;
	    }
	  for (uint64_t tag = instr->tag_ + 1; tag < instrVec.size(); ++tag)
	    if (instrVec.at(tag).retired_)
	      updateDependencies(hart, instrVec.at(tag));
	}
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
      uint64_t wopTime = 0;
      for (const auto wopIx : instr.memOps_)
	{
	  if (wopIx >= sysMemOps_.size())
	    continue;
	  const auto& wop = sysMemOps_.at(wopIx);
	  if (wop.isRead_)
	    continue;  // Should not happen.
	  if (wop.time_ < readOp.time_)  // TBD : <= instead of < ?
	    {
	      wopTime = wop.time_;
	      departed = false; // Write op left core before read.
	    }
	}
      if (departed)
	{
	  cerr << "Internal read op forward source already drained from "
	       << "merge buffer: read-time=" << readOp.time_ << " read-tag="
	       << readOp.instrTag_ << " write-time=" << wopTime
	       << " write-tag=" << instr.tag_ << '\n';
	}
      
      uint8_t byteVal = instr.data_ >> (byteAddr - il)*8;
      uint64_t aligned = uint64_t(byteVal) << 8*rix;
	
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

  while (instrTag)
    {
      if (vec.at(instrTag-1).retired_ or vec.at(instrTag-1).canceled_)
	break;
      cancelInstr(vec.at(--instrTag));
    }
}


template <typename URV>
bool
Mcm<URV>::checkRtlRead(unsigned hartId, const McmInstr& instr,
		       const MemoryOp& op)
{
  if (op.size_ > instr.size_)
    {
      cerr << "Error: Read operation size (" << unsigned(op.size_) << ") larger than "
	   << "instruction data size (" << unsigned(instr.size_) << "): Hart-id="
	   << hartId << " time=" << op.time_ << " tag=" << instr.tag_ << '\n';
      return false;
    }

  if (op.rtlData_ != op.data_)
    {
      cerr << "Error: RTL/whisper read mismatch time=" << op.time_
	   << " hart-id=" << hartId << " instr-tag=" 
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
  if (instr.size_ == 0)
    {
      cerr << "Error: Merge buffer insert for a non-store instruction: "
	   << "Hart-id=" << hartId << " time=" << time_ << " tag=" << instr.tag_
	   << '\n';
      return false;
    }

  if (op.size_ > instr.size_)
    {
      cerr << "Error: Write size exceeds store instruction size: "
	   << "Hart-id=" << hartId << " time=" << time_ << " tag=" << instr.tag_
	   << " write-size=" << op.size_ << " store-size=" << instr.size_ << '\n';
      return false;
    }

  uint64_t data = instr.data_;

  if (op.size_ < instr.size_)
    {
      uint64_t shift = (op.physAddr_ - instr.physAddr_) * 8;
      data = data >> shift;
      shift = 64 - op.size_*8;
      data = (data << shift) >> shift;
    }

  if (data == op.rtlData_)
    return true;

  cerr << "Error: RTL/whisper write mismatch time=" << op.time_
       << " hart-id=" << hartId << " instr-tag="
       << instr.tag_ << " addr=0x" << std::hex << op.physAddr_
       << " size=" << unsigned(op.size_) << " rtl=0x" << op.rtlData_
       << " whisper=0x" << data << std::dec << '\n';
  return false;
}


template <typename URV>
void
Mcm<URV>::clearMaskBitsForWrite(const McmInstr& storeInstr,
				const McmInstr& target, uint64_t& mask) const
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
      storeMask |= opMask;
    }

  unsigned unused = (8 - storeInstr.size_)*8;  // Unwritten upper bits of store.
  storeMask = (storeMask << unused) >> unused;

  mask &= ~storeMask;
}


/// An external read op should not be able to forward
template <typename URV>
bool
Mcm<URV>::checkExternalRead(Hart<URV>& hart, const MemoryOp& op) const
{
  const auto& instrVec = hartInstrVecs_.at(hart.sysHartIndex());

  if (op.isCanceled() or op.failRead_ or op.internal_ or op.instrTag_ >= instrVec.size())
    {
      cerr << "Error: Checking external read for a canceled/failed/internal op: "
	   << "hart-id=" << hart.hartId() << " op-time=" << op.time_ << " op-instr-tag="
	   << op.instrTag_ << '\n';
      return false;
    }

  for (auto tag = op.instrTag_; tag > 0; --tag)
    {
      const auto& prev = instrVec.at(tag);
      if (not prev.isStore_ or not prev.overlaps(op))
	continue;
      bool fail = not prev.complete_ or latestOpTime(prev) >= op.time_;
      if (fail)
	{
	  cerr << "Error: External read op must not forward from store: hart-id="
	       << hart.hartId() << " op-time=" << op.time_ << " op-instr-tag="
	       << op.instrTag_ << " store-tag=" << prev.tag_ << '\n';
	  return false;
	}

      // TBD FIX : Once op is covered by store instructions return true.
    }
  return true;
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
Mcm<URV>::checkLoadComplete(const McmInstr& instr) const
{
  if (instr.isCanceled() or instr.isStore_)
    return false;

  uint64_t mergeMask = 0;
  for (auto opIx : instr.memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      auto& op = sysMemOps_.at(opIx);
      if (not op.isRead_)
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
  currentInstrTag_.at(hart.sysHartIndex()) = tag;
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
      cerr << "Mcm::getCurrentLoadValue: Invalid size: " << size << '\n';
      assert(0 && "Mcm::getCurrentLoadValue: Invalid size");
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
      cerr << "Mcm::getCurrentLoadValue: Instruction already retired\n";
      assert(0 && "Mcm::getCurrentLoadValue: Instruction areadly retired");
      return false;
    }

  instr->size_ = size;
  instr->physAddr_ = addr;

  // Merge read operation values.
  uint64_t mergeMask = 0;
  uint64_t merged = 0;
  bool ok = true;
  for (auto opIx : instr->memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      auto& op = sysMemOps_.at(opIx);
      if (not op.isRead_)
	continue;

      instr->isLoad_ = true;

      if (op.internal_)
	{
	  if (not forwardToRead(hart, tag, op))
	    ok = false;
	}
      else if (not checkExternalRead(hart, op))
	ok = false;

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
      cerr << "Error: Read ops do not cover all the bytes of load instruction"
	   << " tag=" << tag << '\n';
      ok = false;
    }

  instr->complete_ = true;  // FIX : Only for non-io

  return ok;
}
  

template <typename URV>
bool
Mcm<URV>::forwardToRead(Hart<URV>& hart, uint64_t tag, MemoryOp& op)
{
  if (not op.internal_)
    return false;

  uint64_t mask = (~uint64_t(0)) >> (8 - op.size_)*8;
  const auto& instrVec = hartInstrVecs_.at(hart.sysHartIndex());
  for (McmInstrIx ix = tag; ix > 0 and mask != 0; --ix)
    {
      const auto& instr = instrVec.at(ix-1);
      if (not forwardTo(instr, op, mask))
	continue;

      const auto& di = instr.di_;
      if (not di.instEntry()->isAtomic())
	continue;

      cerr << "Error: Internal read forwards from an atomic instruction"
	   << " time=" << op.time_ << " hart-id=" << hart.hartId()
	   << " instr-tag=" << tag << " addr=0x" << std::hex
	   << op.physAddr_ << " amo-tag=" << instr.tag_ << std::dec << '\n';
      return false;
    }

  if (mask != 0)
    {
      cerr << "Error: Internal read does not forward from preceeding stores"
	   << " time=" << op.time_ << " hart-id=" << hart.hartId()
	   << " instr-tag=" << tag << " addr=0x" << std::hex
	   << op.physAddr_ << std::dec << '\n';
      return false;
    }
  return true;
}


template <typename URV>
void
Mcm<URV>::identifyRegisters(const DecodedInst& di,
			    std::vector<unsigned>& sourceRegs,
			    std::vector<unsigned>& destRegs)
{
  sourceRegs.clear();
  destRegs.clear();

  if (not di.isValid())
    return;

  const auto entry = di.instEntry();
  if (not entry)
    {
      cerr << "Mcm::identifyRegisters: Error invalid instr entry\n";
      assert(0 && "Mcm::identifyRegisters: Error invalid instr entry");
    }

  if (entry->hasRoundingMode() and
      RoundingMode(di.roundingMode()) == RoundingMode::Dynamic)
    sourceRegs.push_back(unsigned(CsrNumber::FCSR) + csRegOffset_);

  if (entry->modifiesFflags())
    destRegs.push_back(unsigned(CsrNumber::FCSR) + csRegOffset_);

  auto id = entry->instId();
  bool skipCsr = ((id == InstId::csrrs or id == InstId::csrrc or
		   id == InstId::csrrsi or id == InstId::csrrci)
		  and di.op1() == 0);

  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      bool isDest = entry->isIthOperandWrite(i);
      bool isSource = entry->isIthOperandRead(i);
      if (not isDest and not isSource)
	continue;
	
      size_t regIx = 0;
      switch(di.ithOperandType(i))
	{
	case OperandType::IntReg:
	  regIx = di.ithOperand(i);
	  if (regIx == 0)
	    continue;  // x0
	  break;
	case OperandType::FpReg:
	  regIx = di.ithOperand(i) + fpRegOffset_;
	  break;
	case OperandType::CsReg:
	  if (isSource and skipCsr)
	    continue;
	  else
	    {
	      CsrNumber csr{di.ithOperand(i)};
	      if (csr == CsrNumber::FFLAGS or csr == CsrNumber::FRM)
		csr = CsrNumber::FCSR;
	      regIx = unsigned(csr) + csRegOffset_;
	    }
	  break;
	case OperandType::VecReg:   // FIX: Not yet supported.
	case OperandType::Imm:
	case OperandType::None:
	  continue;
	}

      if (isDest)
	destRegs.push_back(regIx);
      if (isSource)
	sourceRegs.push_back(regIx);
    }
}


template <typename URV>
bool
Mcm<URV>::instrHasRead(const McmInstr& instr) const
{
  for (auto opIx : instr.memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      auto& op = sysMemOps_.at(opIx);
      if (op.isRead_)
	return true;
    }
  return false;
}


template <typename URV>
bool
Mcm<URV>::instrHasWrite(const McmInstr& instr) const
{
  for (auto opIx : instr.memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      auto& op = sysMemOps_.at(opIx);
      if (not op.isRead_)
	return true;
    }
  return false;
}


template <typename URV>
bool
Mcm<URV>::ppoRule1(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Check ppo rule 1 for all the write operations asociated with
  // the given store instruction B.
  
  if (not instrB.complete_)
    return true;  // We will try again when B is complete.

  const auto& instrVec = hartInstrVecs_.at(hart.sysHartIndex());

  for (McmInstrIx tag = instrB.tag_; tag > 0; --tag)
    {
      const auto& instrA =  instrVec.at(tag-1);
      if (instrA.isCanceled())
	continue;

      if (not instrA.isRetired())
	{
	  cerr << "Mcm::ppoRule1: Error: Instruction is not retired\n";
	  assert(0 && "Mcm::ppoRule1: Error: Instruction is not retired");
	  return false;
	}

      if (not instrA.isMemory() or not instrA.overlaps(instrB))
	continue;

      if (isBeforeInMemoryTime(instrA, instrB))
	continue;

      cerr << "Error: PPO rule 1 failed: hart-id=" << hart.hartId() << " tag1="
	   << instrA.tag_ << " tag2=" << instrB.tag_ << '\n';
      return false;
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule2(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 2: a and b are loads, x is a byte read by both a and b,
  // there is no store to x between a and b in program order, and a
  // and b return values for x written by different memory operations.
 
  // Instruction B must be a load/amo instruction.
  if (not instrB.isLoad_)
    return true;  // NA: B is not a load.

  unsigned hartIx = hart.sysHartIndex();
  const auto& instrVec = hartInstrVecs_.at(hartIx);

  uint64_t mask = ~uint64_t(0);  // Bits of B not-written by preceeding stores.
  unsigned shift = (8 - instrB.size_) * 8;
  mask = (mask << shift) >> shift;

  for (McmInstrIx tag = instrB.tag_; tag > 0; --tag)
    {
      const auto& instrA =  instrVec.at(tag-1);
      if (instrA.isCanceled() or not instrA.isLoad_)
	continue;

      if (not instrA.overlaps(instrB))
	continue;

      clearMaskBitsForWrite(instrA, instrB, mask);
      if (mask == 0)
	return true; // All bytes of B written by preceeding stores.

      if (isBeforeInMemoryTime(instrA, instrB))
	continue;

      // Is there a remote write between A and B memory time that
      // covers a byte of B.
      if (instrA.memOps_.size() == 0 or instrB.memOps_.size() == 0)
	{
	  cerr << "Error: PPO Rule 2: Instruction with no memory operation: "
	       << "hart-id=" << hart.hartId() << " tag1=" << instrA.tag_
	       << " tag2=" << instrB.tag_ << '\n';
	  return false;
	}
      uint64_t ix0 = instrB.memOps_.front();
      uint64_t ix1 = instrA.memOps_.back();

      for (uint64_t ix = ix0; ix <= ix1; ++ix)
	{
	  const MemoryOp& op = sysMemOps_.at(ix);
	  if (op.hartIx_ != hartIx and not op.isRead_ and instrB.overlaps(op))
	    {
	      unsigned shift = 8*(8 - op.size_);
	      uint64_t opMask = ~uint64_t(0);
	      opMask = (opMask << shift) >> shift;
	      if (op.physAddr_ <= instrB.physAddr_)
		opMask >>= (instrB.physAddr_ - op.physAddr_)*8;
	      else
		opMask <<= (op.physAddr_- instrB.physAddr_)*8;
	      if ((opMask & mask) != 0)
		{
		  cerr << "Error: PPO Rule 2 failed: hart-id=" << hart.hartId()
		       << " tag1=" << instrA.tag_ << " tag2=" << instrB.tag_
		       << " intermediate store at time=" << op.time_ << '\n';
		  return false;
		}
	    }
	}
    }
  
  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule3(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 3: A is a write resulting from an AMO/SC instructions, A and
  // B have overlapping addresses, B loads data from A.
 
  if (not instrB.di_.isValid())
    {
      cerr << "Mcm::ppoRule3: Error: Instruction B is not decoded.\n";
      assert(0 && "Mcm::ppoRule3: Error: Instruction B is not decoded.");
    }

  // Instruction B must be a load/amo instruction.
  const DecodedInst& bdi = instrB.di_;
  if (bdi.isStore())
    return true;  // NA: store instruction.

  if (not bdi.isLoad() and not bdi.isAtomic())
    return true;  // NA: must be load/amo.

  if (not instrB.complete_)
    return true;  // We will try again when B is complete.

  const auto& instrVec = hartInstrVecs_.at(hart.sysHartIndex());

  uint64_t mask = ~uint64_t(0);  // Bits of B not-written by preceeding non-atomic stores.
  unsigned shift = (8 - instrB.size_) * 8;
  mask = (mask << shift) >> shift;

  for (McmInstrIx tag = instrB.tag_; tag > 0; --tag)
    {
      const auto& instrA =  instrVec.at(tag-1);
      if (instrA.isCanceled())
	continue;
      if (not instrA.isRetired())
	{
	  cerr << "Mcm::ppoRule3: Error: Instruction A is not retired.\n";
	  assert(0 && "Mcm::ppoRule3: Error: Instruction A is not retired.");
	}

      if (not instrA.isStore_ or not instrA.overlaps(instrB))
	continue;

      // If A is not atomic remove from mask the bytes of B that are
      // covered by A. Done when all bytes of B are covered.
      if (not instrA.di_.isValid())
	{
	  cerr << "Mcm::ppoRule3: Error: Instruction A is not decoded.\n";
	  assert(0 && "Mcm::ppoRule3: Instruction A is not decoded.");
	}

      if (not instrA.di_.isAtomic())
	{
	  clearMaskBitsForWrite(instrA, instrB, mask);
	  if (mask == 0)
	    return true;
	}
      else if (not isBeforeInMemoryTime(instrA, instrB))
	{
	  cerr << "Error: PPO rule 3 failed: hart-id=" << hart.hartId() << " tag1="
	       << instrA.tag_ << " tag2=" << instrB.tag_ << " time1="
	       << latestOpTime(instrA) << " time2=" << earliestOpTime(instrB)
	       << '\n';
	  return false;
	}
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule4(Hart<URV>& hart, const McmInstr& instr) const
{
  // Rule 4: There is a fence that orders A before B.

  if (not instr.retired_ or not instr.di_.isValid())
    {
      cerr << "Mcm::ppoRule4: Invalid/undecoded instruction\n";
      assert(0 && "Mcm::ppoRule4: Invalid/undecoded instruction\n");
    }

  if (not instr.di_.isFence())
    return true;

  bool predRead = instr.di_.isFencePredRead();
  bool predWrite = instr.di_.isFencePredWrite();
  bool succRead = instr.di_.isFenceSuccRead();
  bool succWrite = instr.di_.isFenceSuccWrite();

  // In the future we will become smarter. For now, all memory
  // operations of preceeding (in prog order) instructions must be
  // finished. No memory operation of succeeding instruction (in prog
  // order) can exist. In the future we will filter out speculative
  // operations.

  unsigned hartIx = hart.sysHartIndex();

  for (uint64_t i = sysMemOps_.size(); i > 0; --i)
    {
      const auto& op = sysMemOps_.at(i-1);
      if (op.isCanceled() or op.hartIx_ != hartIx)
	continue;

      if ((predRead and op.isRead_) or (predWrite and not op.isRead_))
	if (op.time_ < time_ and op.instrTag_ > instr.tag_)
	  {
	    cerr << "Error: PPO rule 4 failed: hart-id=" << hart.hartId()
		 << " tag1=" << instr.tag_ << " tag2=" << op.instrTag_
		 << " time1=" << time_ << " time2=" << op.time_ << '\n';
	    return false;
	  }

      if ((succRead and op.isRead_) or (succWrite and not op.isRead_))
	if (op.time_ > time_ and op.instrTag_ < instr.tag_)
	  {
	    cerr << "Error: PPO rule 4 failed: hart-id=" << hart.hartId()
		 << " tag1=" << op.instrTag_ << " tag2=" << instr.tag_
		 << " time1=" << op.time_ << " time2=" << time_ << '\n';
	    return false;
	  }
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule5(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 5: A has an acquire annotation

  if (instrB.isCanceled())
    {
      cerr << "Mcm::ppoRule5: B instruction canceled tag=" << instrB.tag_ << '\n';
      return false;
    }

  if (not instrB.isMemory())
    return true;

  const auto& instrVec = hartInstrVecs_.at(hart.sysHartIndex());

  for (McmInstrIx tag = instrB.tag_; tag > 0; --tag)
    {
      const auto& instrA =  instrVec.at(tag-1);
      if (instrA.isCanceled() or not instrA.isMemory())
	continue;
      if (not instrA.isRetired() or not instrA.di_.isValid())
	{
	  cerr << "Mcm::ppoRule5: Instruction A invalid/not-retired\n";
	  assert(0 && "Mcm::ppoRule5: Instruction A invalid/not-retired");
	}
      if (not instrA.di_.isAtomicAcquire())
	continue;

      bool fail = false;
      if (instrA.di_.isAmo())
	fail = instrA.memOps_.size() != 2; // Incomplete amo might finish afrer B
      else if (not instrA.complete_)
	fail = true; // Incomplete store might finish after B
      else if (not instrB.memOps_.empty() and
	       earliestOpTime(instrB) <= latestOpTime(instrA))
	fail = true;  // A finishes after B

      if (fail)
	{
	  cerr << "Error: PPO rule 5 failed: hart-id=" << hart.hartId()
	       << " tag1=" << instrA.tag_ << " tag2=" << instrB.tag_ << '\n';
	  return false;
	}
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule6(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 6: B has a release annotation

  if (instrB.isCanceled() or not instrB.di_.isValid())
    {
      cerr << "Mcm::ppoRule6: Instr B canceled/invalid\n";
      assert(0 && "Mcm::ppoRule6: Instr B canceled/invalid");
    }
  if (not instrB.isMemory() or not instrB.di_.isAtomicRelease())
    return true;

  const auto& instrVec = hartInstrVecs_.at(hart.sysHartIndex());

  for (McmInstrIx tag = instrB.tag_; tag > 0; --tag)
    {
      const auto& instrA =  instrVec.at(tag-1);
      if (instrA.isCanceled() or not instrA.isMemory())
	continue;
      if (not instrA.isRetired() or not instrA.di_.isValid())
	{
	  cerr << "Mcm::ppoRule6: Instr A invalid/not-retired\n";
	  assert(0 && "Mcm::ppoRule6: Instr A invalid/not-retired");
	}

      bool fail = false;
      if (instrA.di_.isAmo())
	fail = instrA.memOps_.size() != 2; // Incomplete amo might finish afrer B
      else if (not instrA.complete_)
	fail = true; // Incomplete store might finish after B
      else if (not instrB.memOps_.empty() and
	       earliestOpTime(instrB) <= latestOpTime(instrA))
	fail = true;  // A finishes after B

      if (fail)
	{
	  cerr << "Error: PPO rule 6 failed: hart-id=" << hart.hartId()
	       << " tag1=" << instrA.tag_ << " tag2=" << instrB.tag_ << '\n';
	  return false;
	}
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule8(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 8: B is a store-conditional, A is a load-reserve paired with B.

  if (instrB.isCanceled())
    {
      cerr << "Mcm::ppoRule8: Instr B canceled\n";
      assert(0 && "Mcm::ppoRule8: Instr B canceled");
    }

  if (not instrB.isMemory())
    return true;
  if (not instrB.di_.isValid())
    {
      cerr << "Mcm::ppoRule8: Instr B undecoded\n";
      assert(0 && "Mcm::ppoRule8: Instr B undecoded");
    }
  if (not instrB.di_.isSc())
    return true;

  uint64_t addr = 0, value = 0;
  if (not hart.lastStore(addr, value))
    return true;  // Score conditional was not successful.

  const auto& instrVec = hartInstrVecs_.at(hart.sysHartIndex());

  for (McmInstrIx tag = instrB.tag_; tag > 0; --tag)
    {
      const auto& instrA =  instrVec.at(tag-1);
      if (instrA.isCanceled())
	continue;
      if (not instrA.isRetired() or not instrA.di_.isValid())
	{
	  cerr << "Mcm::ppoRule8: Instr A undecoded/invalid\n";
	  assert(0 && "Mcm::ppoRule8: Instr B undecoded/invalid");
	}
      if (not instrA.di_.isLr())
	continue;

      bool fail = false;
      if (not instrA.complete_)
	fail = true;
      else if (not instrB.memOps_.empty() and
	       earliestOpTime(instrB) <= latestOpTime(instrA))
	fail = true;  // A finishes after B

      if (fail)
	{
	  cerr << "Error: PPO rule 8 failed: hart-id=" << hart.hartId()
	       << " tag1=" << instrA.tag_ << " tag2=" << instrB.tag_ << '\n';
	  return false;
	}

      return true;
    }

  cerr << "Error: PPO rule 8: Could not find LR instruction paired with SC at tag=" << instrB.tag_ << '\n';

  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule9(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 9: B has a syntactic address dependency on A

  if (instrB.isCanceled())
    {
      cerr << "Mcm::ppoRule12: Instr B canceled: tag=" << instrB.tag_ << "\n";
      return false;
    }

  if (not instrB.isMemory())
    return true;

  const auto& bdi = instrB.di_;
  unsigned addrReg = 0;
  if (bdi.isLoad() or bdi.isStore() or bdi.isAmo())
    addrReg = bdi.op1();

  uint64_t time = hartRegTimes_.at(hart.sysHartIndex()).at(addrReg);

  for (auto opIx : instrB.memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      if (sysMemOps_.at(opIx).time_ > time)
	continue;

      cerr << "Error: PPO rule 9 failed: hart-id=" << hart.hartId() << " tag1="
	   << hartRegProducers_.at(hart.sysHartIndex()).at(addrReg)
	   << " tag2=" << instrB.tag_ << '\n';
      return false;
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule10(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 10: B has a syntactic data dependency on A

  if (instrB.isCanceled())
    {
      cerr << "Mcm::ppoRule10: Instr B canceled: tag=" << instrB.tag_ << "\n";
      return false;
    }

  if (not instrB.isMemory())
    return true;

  unsigned hartIx = hart.sysHartIndex();

  // TBD FIX: this mixes integer and FP data registers -- incorrect.
  const auto& bdi = instrB.di_;
  unsigned dataReg = 0;
  if (bdi.isAmo())
    dataReg = bdi.op2();
  else if (bdi.isStore())
    dataReg = bdi.op0();
  else
    return true;

  uint64_t time = hartRegTimes_.at(hartIx).at(dataReg);

  for (auto opIx : instrB.memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      if (sysMemOps_.at(opIx).time_ > time)
	continue;

      cerr << "Error: PPO rule 10 failed: hart-id=" << hart.hartId() << " tag1="
	   << hartRegProducers_.at(hartIx).at(dataReg)
	   << " tag2=" << instrB.tag_ << '\n';
      return false;
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule11(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 11: B is a store with a control dependency on A

  if (instrB.isCanceled())
    {
      cerr << "Mcm::ppoRule11: Instr B canceled\n";
      assert(0 && "Mcm::ppoRule11: Instr B canceled");
    }

  unsigned hartIx = hart.sysHartIndex();

  const auto& bdi = instrB.di_;
  if (not bdi.isStore() and not bdi.isAmo())
    return true;

  auto producerTag = hartBranchProducers_.at(hartIx);
  if (hartBranchTimes_.at(hartIx) == 0)
    return true;

  const auto& instrVec = hartInstrVecs_.at(hartIx);
  if (producerTag >= instrVec.size())
    return true;
  const auto& producer = instrVec.at(producerTag);
  if (not producer.di_.isValid())
    return true;

  if (not producer.complete_ or isBeforeInMemoryTime(instrB, producer))
    {
      cerr << "Error: PPO rule 11 failed: hart-id=" << hart.hartId() << " tag1="
	   << producerTag << " tag2=" << instrB.tag_ << '\n';
      return false;
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule12(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 12: B is a load, there is a store M between A and B such that
  // 1. B loads a value written by M
  // 2. M has an address or data denpendency on A

  if (instrB.isCanceled())
    {
      cerr << "Mcm::ppoRule12: Instr B canceled: tag=" << instrB.tag_ << "\n";
      return false;
    }

  if (not instrB.isLoad_)
    return true;  // NA: B is not a load.

  unsigned hartIx = hart.sysHartIndex();
  const auto& instrVec = hartInstrVecs_.at(hartIx);
  if (instrVec.empty() or instrB.tag_ == 0)
    return true;  // Nothing before B in instruction order.

  // Check all preceeding instructions for a non-finished store M with
  // address overlapping that of B. This is expensive. We need to keep
  // set of non-finished stores.
  size_t ix = std::min(size_t(instrB.tag_), instrVec.size());
  for ( ; ix ; --ix)
    {
      size_t mtag = ix - 1;
      const auto& instrM = instrVec.at(mtag);
      if (instrM.isCanceled() or not instrM.di_.isValid())
	continue;

      const auto& mdi = instrM.di_;
      if ((not mdi.isStore() and not mdi.isAmo()) or instrM.complete_
	  or not instrM.overlaps(instrB))
	continue;

      unsigned addrReg = mdi.op1();
      auto apTag = hartRegProducers_.at(hartIx).at(addrReg); // address producer tag

      // TBD FIX: this mixes integer and FP data registers -- incorrect.
      unsigned dataReg = mdi.isAmo()? mdi.op2() : mdi.op0();
      auto dpTag = hartRegProducers_.at(hartIx).at(dataReg); // data producer tag

      for (auto aTag : { apTag, dpTag } )
	{
	  const auto& instrA = instrVec.at(aTag);
	  if (instrA.di_.isValid())
	    if (not instrA.complete_ or isBeforeInMemoryTime(instrB, instrA))
	      {
		cerr << "Error: PPO rule 12 failed: hart-id=" << hart.hartId() << " tag1="
		     << aTag << " tag2=" << instrB.tag_ << '\n';
		return false;
	      }
	}
    }

  return true;
}



template class WdRiscv::Mcm<uint32_t>;
template class WdRiscv::Mcm<uint64_t>;
