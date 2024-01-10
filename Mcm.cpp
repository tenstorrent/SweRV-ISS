#include <iostream>
#include <string_view>
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

  hartPendingFences_.resize(hartCount);

 // If no merge buffer, then memory is updated on insert messages.
  writeOnInsert_ = (lineSize_ == 0);
}


template <typename URV>
Mcm<URV>::~Mcm() = default;


template <typename URV>
inline
bool
Mcm<URV>::updateTime(const char* method, uint64_t time)
{
  if (time < time_)
    {
      cerr << "Warning: " << method << ": Backward time: "
	   << time << " < " << time_ << "\n";
      return true;
    }
  time_ = time;
  return true;
}


template <typename URV>
bool
Mcm<URV>::readOp(Hart<URV>& hart, uint64_t time, uint64_t instrTag,
		 uint64_t physAddr, unsigned size, uint64_t rtlData)
{
  if (not updateTime("Mcm::readOp", time))
    return false;

  unsigned hartIx = hart.sysHartIndex();
  if (hartIx >= hartInstrVecs_.size())
    {
      cerr << "Mcm::readOp: Error: Hart ix out of bound\n";
      assert(0 && "Mcm::readOp: Hart ix out of bound");
    }

  McmInstr* instr = findOrAddInstr(hartIx, instrTag);
  if (instr->isCanceled())
    return true;
  bool io = false;  // FIX  get io from PMA of address.
  if (instr->isRetired() and not io)
    cerr << "Warning: Read op time=" << time << " occurs after "
	 << "instruction retires tag=" << instr->tag_ << '\n';

  MemoryOp op = {};
  op.time_ = time;
  op.physAddr_ = physAddr;
  op.rtlData_ = rtlData;
  op.instrTag_ = instrTag;
  op.hartIx_ = hartIx;
  op.size_ = size;
  op.isRead_ = true;

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
  else if (size < 8)
    {
      uint8_t val = 0;
      for (unsigned i = 0; i < size; ++i)
	if (not hart.peekMemory(physAddr + i, val, true))
	  {
	    op.failRead_ = true;
	    break;
	  }
	else
	  op.data_ = op.data_ | (uint64_t(val) << (8*i));
    }
  else
    {
      op.failRead_ = true;
      cerr << "Error: Mcm::readOp: Invalid read size: " << size << '\n';
      return false;
    }

  instr->addMemOp(sysMemOps_.size());
  sysMemOps_.push_back(op);
  instr->isLoad_ = true;

  bool result = true;
  if (checkLoadComplete(*instr))
    instr->complete_ = true;
  
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

  const auto* instEntry = di.instEntry();

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
      if (instr.memOps_.empty())
	{
	  tag = instr.tag_;
	  time = ~uint64_t(0); // Will be updated when SC drains to memory.
	}
    }
  else if (instEntry->isStore())
    return;   // No destination register.
  else if (di.isLoad() or di.isAmo() or di.isBranch())
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
      regTimeVec.at(regIx) = time;
      regProducer.at(regIx) = tag;
    }
}


template <typename URV>
void
Mcm<URV>::setProducerTime(unsigned hartIx, McmInstr& instr)
{
  auto& di = instr.di_;

  // Set producer of address register.
  if (di.isLoad() or di.isAmo() or di.isStore())
    {
      unsigned addrReg = effectiveRegIx(di, 1);  // Addr reg is operand 1 of instr.
      instr.addrProducer_ = hartRegProducers_.at(hartIx).at(addrReg);
      instr.addrTime_ = hartRegTimes_.at(hartIx).at(addrReg);
    }

  // Set producer of data register.
  if (di.isStore() or di.isAmo())
    {
      unsigned doi = di.isAmo()? 2 : 0;  // Data-regiser operand index
      unsigned dataReg = effectiveRegIx(di, doi);  // Data operand may be integer/fp/csr
      instr.dataProducer_ = hartRegProducers_.at(hartIx).at(dataReg);
      instr.dataTime_ = hartRegTimes_.at(hartIx).at(dataReg);
    }
}


template <typename URV>
static bool
pokeHartMemory(Hart<URV>& hart, uint64_t physAddr, uint64_t data, unsigned size)
{
  if (size == 1)
    return hart.pokeMemory(physAddr, uint8_t(data), true);

  if (size == 2)
    return hart.pokeMemory(physAddr, uint16_t(data), true);

  if (size == 4)
    return hart.pokeMemory(physAddr, uint32_t(data), true);

  if (size == 8)
    return hart.pokeMemory(physAddr, uint64_t(data), true);

  if (size < 8)
    {
      for (unsigned i = 0; i < size; ++i)
	if (not hart.pokeMemory(physAddr + i, uint8_t(data >> (8*i)), true))
	  return false;
      return true;
    }

  cerr << "MCM pokeHartMemory: " << "Invalid data size (" << size << ")\n";
  return false;
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

  if (not writeOnInsert_)
    hartPendingWrites_.at(hartIx).push_back(op);

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
      instr->complete_ = checkStoreComplete(*instr);

      if (not instr->retired_)
	{
	  cerr << "Mcm::MergeBufferInsert: Error: Merge buffer write for a non-retired store\n";
	  return false;
	}

      if (not ppoRule1(hart, *instr))
	result = false;

      // We commit the RTL data to memory but we check them against
      // whisper data (checkRtlWrite below). This is simpler than
      // committing part of whisper instruction data.
      if (not pokeHartMemory(hart, physAddr, rtlData, op.size_))
	result = false;
    }

  // If corresponding insruction is retired, compare to its data.
  if (instr->retired_)
    if (not checkRtlWrite(hart.hartId(), *instr, op))
      result = false;

  return result;
}


template <typename URV>
bool
Mcm<URV>::bypassOp(Hart<URV>& hart, uint64_t time, uint64_t instrTag,
		   uint64_t physAddr, unsigned size, uint64_t rtlData)
{
  if (not updateTime("Mcm::writeOp", time))
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

  // If corresponding insruction is retired, compare to its data.
  McmInstr* instr = findOrAddInstr(hartIx, instrTag);
  if (not instr)
    {
      cerr << "Mcm::bypassOp: Error: Unknown instr tag\n";
      assert(0 && "Mcm::BypassOp: Unknown instr tag");
      return false;
    }

  bool result = true;

  // Associate write op with instruction.
  instr->addMemOp(sysMemOps_.size());
  sysMemOps_.push_back(op);

  instr->complete_ = checkStoreComplete(*instr);

  if (instr->retired_)
    {
      result = pokeHartMemory(hart, physAddr, rtlData, size) and result;
      result = checkRtlWrite(hart.hartId(), *instr, op) and result;
      if (instr->complete_)
	result = ppoRule1(hart, *instr) and result;
    }

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
      cerr << "Mcm::retire: Error: Time=" << time << " hart-id=" << hartIx << " tag="
	   << tag << " Instruction retired multiple times\n";
      return false;
    }

  if (not di.isValid())
    {
      cancelInstr(*instr);  // Instruction took a trap.
      return true;
    }

  instr->retired_ = true;
  instr->di_ = di;

  bool ok = true;

  // If instruction is a store, save address, size, and written data.
  uint64_t addr = 0, addr2 = 0, value = 0;
  unsigned stSize = hart.lastStore(addr, addr2, value);
  if (stSize)
    {
      instr->size_ = stSize;
      instr->physAddr_ = addr;
      instr->physAddr2_ = addr2;
      instr->data_ = value;
      instr->isStore_ = true;
      instr->complete_ = checkStoreComplete(*instr);
      if (instr->complete_)   // Write ops already seen. Commit data.
	ok = pokeHartMemory(hart, addr, value, stSize) and ok;
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
	ok = false;
    }

  // Amo sanity check.
  if (di.instEntry()->isAmo())
    {
      // Must have a read.  Must not have a write.
      if (not instrHasRead(*instr))
	{
	  cerr << "Error: Hart-id=" << hart.hartId() << " tag=" << tag
	       << " amo instruction retired before read op.\n";
	  ok = false;
	}

      if (not instrHasWrite(*instr))
	{
	  cerr << "Warning: Hart-id=" << hart.hartId() << " tag=" << tag
	       << " amo instruction retired without a write op.\n";
	  return false;
	}

      instr->isStore_ = true;  // AMO is both load and store.
    }

  setProducerTime(hartIx, *instr);
  updateDependencies(hart, *instr);

  if (instr->isStore_)
    {
      if (instr->complete_)
	ok = checkStoreData(hartIx, *instr) and ok;
      ok = ppoRule1(hart, *instr) and ok;
    }

  ok = ppoRule2(hart, *instr) and ok;
  ok = ppoRule3(hart, *instr) and ok;
  ok = processFence(hart, *instr) and ok;  // ppo rule 4.
  ok = ppoRule5(hart, *instr) and ok;
  ok = ppoRule6(hart, *instr) and ok;
  ok = ppoRule7(hart, *instr) and ok;
  ok = ppoRule8(hart, *instr) and ok;
  ok = ppoRule9(hart, *instr) and ok;
  ok = ppoRule10(hart, *instr) and ok;
  ok = ppoRule11(hart, *instr) and ok;
  ok = ppoRule12(hart, *instr) and ok;

  return ok;
}


static void
reportMismatch(uint64_t hartId, uint64_t time, std::string_view tag, uint64_t addr,
	       uint64_t rtlData, uint64_t whisperData)
{
  cerr << "Error: Mismatch on " << tag << " time=" << time
	    << " hart-id=" << hartId << " addr=0x" << std::hex
	    << addr << " rtl=0x" << rtlData
	    << " whisper=0x" << whisperData << std::dec << '\n';
}


static bool
checkBufferWriteParams(unsigned hartId, uint64_t time, unsigned lineSize,
		       uint64_t& rtlLineSize, uint64_t physAddr)
{
  if (lineSize == 0)
    {
      cerr << "Merge buffer write attempted when merge buffer is disabled\n";
      return false;
    }

  if (rtlLineSize > lineSize)
    {
      cerr << "Error: Hart-id=" << hartId << " time=" << time
	   << "RTL merge buffer write line size (" << rtlLineSize << ") greater than"
	   << " reference line size (" << lineSize << ")\n";
      return false;
    }

  if ((physAddr % lineSize) + rtlLineSize > lineSize)
    {
      cerr << "Warning: Hart-id=" << hartId << " time=" << time
	   << " RTL merge buffer write data at address 0x"
	   << std::hex << physAddr << " crosses buffer boundary" << std::dec
	   << " -- truncating RTL data\n";
	rtlLineSize -= (physAddr % lineSize);
    }

#if 0
  if ((physAddr % lineSize) != 0)
    {
      cerr << "Merge buffer write: address (0x" << std::hex << physAddr << ") "
	   << "not a multiple of line size (" << std::dec << lineSize << ")\n";
      return false;
    }
#endif

  return true;
}


template <typename URV>
bool
Mcm<URV>::collectCoveredWrites(Hart<URV>& hart, uint64_t time, uint64_t rtlAddr,
			       uint64_t rtlLineSize, const std::vector<bool>& rtlMask,
			       MemoryOpVec& coveredWrites)
{
  unsigned hartIx = hart.sysHartIndex();
  auto& pendingWrites = hartPendingWrites_.at(hartIx);
  size_t pendingSize = 0;  // pendingWrite size after removal of matching writes

  uint64_t lineEnd = rtlAddr + rtlLineSize;

  for (size_t i = 0; i < pendingWrites.size(); ++i)
    {
      auto& op = pendingWrites.at(i);  // Write op
      McmInstr* instr = findOrAddInstr(hartIx, op.instrTag_);

      bool written = false;  // True if op is actually written
      if (op.physAddr_ >= rtlAddr and op.physAddr_ < lineEnd)
	{
	  if (op.physAddr_ + op.size_  > lineEnd)
	    {
	      cerr << "Error: Pending store address out of line bounds time=" << time
		   << " hart-id=" << hart.hartId() << " addr=0x" << std::hex
		   << op.physAddr_ << std::dec << "\n";
	      return false;
	    }

	  if (not instr or instr->isCanceled())
	    {
	      cerr << "Error: Write for an invalid/speculated store time=" << time
		   << " hart-id=" << hart.hartId() << " tag=" << op.instrTag_
		   << " addr=0x" << std::hex << op.physAddr_ << std::dec << "\n";
	      return false;
	    }

	  if (rtlMask.empty())
	    written = true;  // No masking
	  else
	    {
	      // Check if op bytes are all masked or all unmaksed.
	      unsigned masked = 0;  // Count of masked bytes of op.
	      for (unsigned opIx = 0; opIx < op.size_; ++opIx)   // Scan op bytes
		{
		  unsigned lineIx = opIx + op.physAddr_ - rtlAddr; // Index in line
		  if (lineIx < rtlMask.size() and rtlMask.at(lineIx))
		    masked++;
		}
	      if (masked != 0)
		{
		  if (masked != op.size_)
		    {
		      cerr << "Error: Write op partially masked time=" << time
			   << " hart-id=" << hart.hartId() << "tag=" << op.instrTag_
			   << "addr=0x" << std::hex << op.physAddr_ << std::dec << "\n";
		      return false;
		    }
		  written = true;
		}
	    }
	}

      if (written)
	{
	  op.time_ = time;
	  instr->addMemOp(sysMemOps_.size());
	  sysMemOps_.push_back(op);
	  coveredWrites.push_back(op);
	}
      else
	{
	  // Op is not written, keep it in pending writes.
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
  uint64_t rtlSize = rtlData.size();
  if (not checkBufferWriteParams(hart.hartId(), time, lineSize_, rtlSize, physAddr))
    return false;

  unsigned hartIx = hart.sysHartIndex();

  // Remove from hartPendingWrites_ the writes matching the RTL line
  // address and place them sorted by instr tag in coveredWrites.
  std::vector<MemoryOp> coveredWrites;
  if (not collectCoveredWrites(hart, time, physAddr, rtlSize, rtlMask, coveredWrites))
    return false;

  // Read our memory corresponding to RTL line addresses.
  uint64_t lineEnd = physAddr + lineSize_ - (physAddr % lineSize_);
  std::vector<uint8_t> line;
  line.reserve(lineSize_);
  for (uint64_t addr = physAddr; addr < lineEnd; ++addr)
    {
      uint8_t byte = 0;
      if (not hart.peekMemory(addr, byte, true /*usePma*/))
	{
	  cerr << "Mcm::mergeBufferWrite: Failed to query memory\n";
	  return false;
	}
      line.push_back(byte);
    }

  // Apply pending writes to our line.
  for (const auto& write : coveredWrites)
    {
      if ((write.physAddr_ < physAddr) or (write.physAddr_ + write.size_ > lineEnd))
	{
	  cerr << "Mcm::mergeBufferWrite: Store address out of line bound\n";
	  return false;
	}
      unsigned ix = write.physAddr_ - physAddr;
      for (unsigned i = 0; i < write.size_; ++i)
	line.at(ix+i) = ((uint8_t*) &(write.rtlData_))[i];
    }

  // Put our line back in memory (use poke words to accomodate clint).
  assert((line.size() % 4) == 0);
  for (unsigned i = 0; i < line.size(); i += 4)
    {
      uint32_t word = *((uint32_t*) (line.data() + i));
      hart.pokeMemory(physAddr + i, word, true);
    }

  // Compare our line to RTL line.
  bool result = true;
  if (checkWholeLine_ or rtlMask.empty())
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
      for (unsigned i = 0; i < line.size(); ++i)
	if (rtlMask.at(i) and (line.at(i) != rtlData.at(i)))
	  {
	    reportMismatch(hart.hartId(), time, "merge buffer write", physAddr + i,
			   rtlData.at(i), line.at(i));
	    result = false;
	    break;
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
Mcm<URV>::forwardTo(const McmInstr& instr, uint64_t iaddr, uint64_t idata,
		    unsigned isize, MemoryOp& readOp, uint64_t& mask)
{
  if (mask == 0)
    return true;  // No bytes left to forward.

  if (instr.isCanceled() or not instr.isRetired() or not instr.isStore_)
    return false;

  uint64_t rol = readOp.physAddr_, roh = readOp.physAddr_ + readOp.size_ - 1;
  uint64_t il = iaddr, ih = il + isize - 1;
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
	    continue;  // May happen for AMO.
	  if (byteAddr < wop.physAddr_ or byteAddr >= wop.physAddr_ + wop.size_)
	    continue;  // Write op does overalp read.
	  if (wop.time_ < readOp.time_)
	    departed = true; // Write op cannot forward.
	}
      if (departed)
	continue;
      
      uint8_t byteVal = idata >> (byteAddr - il)*8;
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
void
Mcm<URV>::cancelInstruction(Hart<URV>& hart, uint64_t instrTag)
{
  McmInstr* instr = findInstr(hart.sysHartIndex(), instrTag);
  if (not instr or instr->isCanceled())
    return;
  cancelInstr(*instr);
}
  

template <typename URV>
bool
Mcm<URV>::checkRtlRead(unsigned hartId, const McmInstr& instr,
		       const MemoryOp& op) const
{
  if (op.size_ > instr.size_)
    {
      cerr << "Warning: Read operation size (" << unsigned(op.size_) << ") larger than "
	   << "instruction data size (" << unsigned(instr.size_) << "): Hart-id="
	   << hartId << " time=" << op.time_ << " tag=" << instr.tag_ << '\n';
    }

  if (op.rtlData_ != op.data_)
    {
      if (skipReadCheck_.find(op.physAddr_) !=  skipReadCheck_.end())
	{
	  cerr << "Error: RTL/whisper read mismatch time=" << op.time_
	       << " hart-id=" << hartId << " instr-tag=" 
	       << op.instrTag_ << " addr=0x" << std::hex << op.physAddr_
	       << " size=" << unsigned(op.size_) << " rtl=0x" << op.rtlData_
	       << " whisper=0x" << op.data_ << std::dec << '\n';
	}
      return false;
    }
  return true;
}


template <typename URV>
bool
Mcm<URV>::checkRtlWrite(unsigned hartId, const McmInstr& instr,
			const MemoryOp& op) const
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

  const char* tag = instr.di_.isAmo()? " AMO " : " ";

  cerr << "Error: RTL/whisper" << tag << "write mismatch time=" << op.time_
       << " hart-id=" << hartId << " instr-tag="
       << instr.tag_ << " addr=0x" << std::hex << op.physAddr_
       << " size=" << unsigned(op.size_) << " rtl=0x" << op.rtlData_
       << " whisper=0x" << data << std::dec << '\n';
  return false;
}


template <typename URV>
bool
Mcm<URV>::checkStoreData(unsigned hartId, const McmInstr& storeInstr) const
{
  if (not storeInstr.complete_)
    return false;

  bool ok = true;

  for (auto opIx : storeInstr.memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;

      const auto& op = sysMemOps_.at(opIx);
      if (op.isRead_)
	continue;

      if (not checkRtlWrite(hartId, storeInstr, op))
	ok = false;
    }

  return ok;
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
      const auto& op = sysMemOps_.at(opIx);
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


template <typename URV>
bool
Mcm<URV>::checkStoreComplete(const McmInstr& instr) const
{
  if (instr.isCanceled() or not instr.isStore_)
    return false;

  unsigned expectedMask = (1 << instr.size_) - 1;  // Mask of bytes covered by instruction.
  unsigned writeMask = 0;   // Mask of bytes covered by write operations.
  uint64_t addr = instr.physAddr_, addr2 = instr.physAddr2_, size = instr.size_;
  for (auto opIx : instr.memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      const auto& op = sysMemOps_.at(opIx);
      if (op.isRead_)
	continue;

      unsigned mask = 0;
      if (pageNum(op.physAddr_) == pageNum(addr))
	{
	  if (op.physAddr_ <= addr)
	    {
	      if (op.physAddr_ + op.size_ > addr)
		{
		  uint64_t overlap = op.physAddr_ + op.size_ - addr;
		  assert(overlap > 0 and overlap <= 8);
		  mask = (1 << overlap) - 1;
		}
	    }
	  else
	    {
	      if (addr + size > op.physAddr_)
		{
		  mask = (1 << op.size_) - 1;
		  mask = mask << (op.physAddr_ - addr);
		}
	    }
	}
      else if (addr != addr2 and pageNum(op.physAddr_) == pageNum(addr2))
	{
	  unsigned size1 = 4096 - (addr % 4096);

	  if (op.physAddr_ == addr2)
	    {
	      uint64_t overlap = op.physAddr_ + op.size_ - addr2;
	      assert(overlap > 0 and overlap <= 8);
	      mask = ((1 << overlap) - 1) << size1;
	    }
	  else
	    {
	      unsigned size2 = size - size1;
	      if (addr2 + size2 > op.physAddr_)
		{
		  mask = (1 << op.size_) - 1;
		  mask = mask << ((op.physAddr_ - addr2) + size1);
		}
	    }
	}

      mask &= expectedMask;
      writeMask |= mask;
    }

  return writeMask == expectedMask;
}


template <typename URV>
bool
Mcm<URV>::checkLoadComplete(const McmInstr& instr) const
{
  if (instr.isCanceled() or not instr.isLoad_ or instr.size_ == 0)
    return false;

  unsigned expectedMask = (1 << instr.size_) - 1;  // Mask of bytes covered by instruction.
  unsigned readMask = 0;   // Mask of bytes covered by read operations.
  uint64_t addr = instr.physAddr_, size = instr.size_;
  for (auto opIx : instr.memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      const auto& op = sysMemOps_.at(opIx);
      if (not op.isRead_)
	continue;

      unsigned mask = 0;
      if (op.physAddr_ <= addr)
	{
	  if (op.physAddr_ + op.size_ > addr)
	    {
	      uint64_t overlap = op.physAddr_ + op.size_ - addr;
	      assert(overlap > 0 and overlap <= 8);
	      mask = (1 << overlap) - 1;
	    }
	}
      else
	{
	  if (addr + size > op.physAddr_)
	    {
	      mask = expectedMask;
	      mask = mask << (op.physAddr_ - addr + 1);
	    }
	}
      mask &= expectedMask;
      readMask |= mask;
    }

  return readMask == expectedMask;
}


template <typename URV>
bool
Mcm<URV>::setCurrentInstruction(Hart<URV>& hart, uint64_t tag)
{
  currentInstrTag_.at(hart.sysHartIndex()) = tag;
  return true;
}


/// Compute a mask of the instruction data bytes covered by the given
/// memory operation. Return 0 if the operation doesnot overlap the
/// given instruction.
template <typename URV>
unsigned
Mcm<URV>::determineOpMask(McmInstr* instr, MemoryOp& op, uint64_t addr1, uint64_t addr2)
{
  unsigned size = instr->size_;
  unsigned mask = 0;
  if (size == 0)
    return mask;

  if (addr1 == addr2)
    {
      if (op.physAddr_ <= addr1)
	{
	  if (op.physAddr_ + op.size_ <= addr1)
	    return mask;  // Read op does not overlap instruction.

	  if (op.physAddr_ + op.size_ > addr1 + size)
	    op.size_ = addr1 + size - op.physAddr_;  // Trim wide ops.
	  uint64_t overlap = op.physAddr_ + op.size_ - addr1;
	  assert(overlap > 0 and overlap <= 8);
	  mask = (1 << overlap) - 1;
	}
      else
	{
	  if (addr1 + size <= op.physAddr_)
	    return mask;  // Read op does no overlap instruction.

	  if (op.physAddr_ + op.size_ > addr1 + size)
	    op.size_ = addr1 + size - op.physAddr_;  // Trim wide ops.
	  mask = (1 << op.size_) - 1;
	  mask = mask << (op.physAddr_ - addr1);
	}
    }
  else
    {
      unsigned size1 = 4096 - (addr1 % 4096);

      if (pageNum(op.physAddr_) == pageNum(addr1))
	{
	  assert(size1 < size);

	  if (op.physAddr_ <= addr1)
	    {
	      if (op.physAddr_ + op.size_ <= addr1)
		return mask;  // Read op does not overlap instruction.
	      if (op.physAddr_ + op.size_ > addr1 + size1)
		op.size_ = addr1 + size1 - op.physAddr_;  // Trim wide ops.
	      uint64_t overlap = op.physAddr_ + op.size_ - addr1;
	      assert(overlap > 0 and overlap <= 8);
	      mask = (1 << overlap) - 1;
	    }
	  else
	    {
	      if (addr1 + size <= op.physAddr_)
		return mask;  // Read op does no overlap instruction.
	      if (op.physAddr_ + op.size_ > addr1 + size1)
		op.size_ = addr1 + size1 - op.physAddr_;  // Trim wide ops.
	      mask = (1 << op.size_) - 1;
	      mask = mask << (op.physAddr_ - addr1);
	    }
	}
      else if (pageNum(op.physAddr_) == pageNum(addr2))
	{
	  unsigned size2 = size - size1;

	  if (op.physAddr_ == addr2)
	    {
	      if (op.physAddr_ + op.size_ > addr2 + size2)
		op.size_ = addr2 + size2 - op.physAddr_;  // Trim wide ops.
	      uint64_t overlap = op.physAddr_ + op.size_ - addr2;
	      assert(overlap > 0 and overlap <= 8);
	      mask = ((1 << overlap) - 1) << size1;
	    }
	  else
	    {
	      if (addr2 + size2 <= op.physAddr_)
		return mask;  // Read op does no overlap instruction.
	      if (op.physAddr_ + op.size_ > addr2 + size2)
		op.size_ = addr2 + size2 - op.physAddr_;  // Trim wide ops.
	      mask = (1 << op.size_) - 1;
	      mask = mask << ((op.physAddr_ - addr2) + size1);
	    }
	}
    }

  return mask;
}


template <typename URV>
void
Mcm<URV>::cancelReplayedReads(McmInstr* instr, uint64_t addr1, uint64_t addr2)
{
  size_t nops = instr->memOps_.size();
  assert(instr->size_ > 0 and instr->size_ <= 8);
  unsigned expectedMask = (1 << instr->size_) - 1;  // Mask of bytes covered by instruction.
  unsigned readMask = 0;    // Mask of bytes covered by read operations.

  auto& ops = instr->memOps_;

  // Process read ops in reverse order so that later reads take precedence.
  for (size_t j = 0; j < nops; ++j)
    {
      auto opIx = ops.at(nops - 1 - j);
      if (opIx >= sysMemOps_.size())
	continue;
      auto& op = sysMemOps_.at(opIx);
      if (not op.isRead_)
	continue;

      instr->isLoad_ = true;

      bool cancel = (readMask == expectedMask);
      if (not cancel)
	{
	  unsigned mask = determineOpMask(instr, op, addr1, addr2);
	  mask &= expectedMask;
	  if ((mask & readMask) == mask)
	    cancel = true;  // Read op already covered by other read ops
	  else
	    readMask |= mask;
	}

      if (cancel)
	op.cancel();
    }

  // Remove canceled ops.
  size_t count = 0;
  for (size_t i = 0; i < ops.size(); ++i)
    {
      auto& op = sysMemOps_.at(ops.at(i));
      if (not op.isCanceled())
	{
	  if (count < i)
	    ops.at(count) = ops.at(i);
	  count++;
	}
    }
  ops.resize(count);
}


template <typename URV>
bool
Mcm<URV>::getCurrentLoadValue(Hart<URV>& hart, uint64_t addr1, uint64_t addr2,
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
  instr->physAddr_ = addr1;

  if (addr2 == addr1 and pageNum(addr1 + size - 1) != pageNum(addr1))
    addr2 = pageAddress(pageNum(addr2) + 1);

  // Cancel early read ops that are covered by later ones. Trim wide reads.
  cancelReplayedReads(instr, addr1, addr2);

  uint64_t mergeMask = 0;
  uint64_t merged = 0;
  bool ok = true;
  size_t nops = instr->memOps_.size();
  for (size_t j = 0; j < nops; ++j)
    {
      auto opIx = instr->memOps_.at(nops - 1 - j);
      if (opIx >= sysMemOps_.size())
	continue;
      auto& op = sysMemOps_.at(opIx);
      if (not op.isRead_)
	continue;

      // Let forwarding override read-op data.
      forwardToRead(hart, tag, op);

      uint64_t opVal = op.data_;
      uint64_t mask = ~uint64_t(0);
      if (pageNum(op.physAddr_) == pageNum(addr1))
	{
	  if (op.physAddr_ <= addr1)
	    {
	      uint64_t offset = addr1 - op.physAddr_;
	      if (offset > 8)
		offset = 8;
	      opVal >>= offset*8;
	      mask >>= offset*8;
	    }
	  else
	    {
	      uint64_t offset = op.physAddr_ - addr1;
	      if (offset > 8)
		offset = 8;
	      opVal <<= offset*8;
	      mask <<= offset*8;
	    }
	}
      else if (pageNum(op.physAddr_) == pageNum(addr2))
	{
	  if (op.physAddr_ == addr2)
	    {
	      uint64_t offset = 4096 - (addr1 % 4096);
	      if (offset > 8)
		offset = 8;
	      opVal <<= offset*8;
	      mask <<= offset*8;
	    }
	  else
	    assert(0);
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
  uint64_t mask = (~uint64_t(0)) >> (8 - op.size_)*8;
  const auto& instrVec = hartInstrVecs_.at(hart.sysHartIndex());
  for (McmInstrIx ix = tag; ix > 0 and mask != 0; --ix)
    {
      const auto& instr = instrVec.at(ix-1);
      if (instr.isCanceled() or not instr.isRetired() or not instr.isStore_)
	continue;

      if (not forwardTo(instr, instr.physAddr_, instr.data_, instr.size_, op, mask))
	{
	  if (instr.physAddr_ == instr.physAddr2_)
	    continue;

	  unsigned size1 = 4096 - (instr.physAddr_ % 4096);
	  unsigned size2 = instr.size_ - size1;
	  assert(size2 > 0 and size2 < 8);
	  uint64_t data2 = instr.data_ >> size1 * 8;
	  if (not forwardTo(instr, instr.physAddr2_, data2, size2, op, mask))
	    continue;
	}

      const auto& di = instr.di_;
      if (di.instEntry()->isAtomic())
	{
	  cerr << "Error: Read op forwards from an atomic instruction"
	       << " time=" << op.time_ << " hart-id=" << hart.hartId()
	       << " instr-tag=" << tag << " addr=0x" << std::hex
	       << op.physAddr_ << " amo-tag=" << instr.tag_ << std::dec << '\n';
	}
    }

  return true;
}


template <typename URV>
unsigned
Mcm<URV>::effectiveRegIx(const DecodedInst& di, unsigned opIx) const
{
  auto type = di.ithOperandType(opIx);
  switch (type)
    {
    case OperandType::IntReg:
      return di.ithOperand(opIx) + intRegOffset_;

    case OperandType::FpReg:
      return di.ithOperand(opIx) + fpRegOffset_;

    case OperandType::CsReg:
      {
	CsrNumber csr{di.ithOperand(opIx)};
	if (csr == CsrNumber::FFLAGS or csr == CsrNumber::FRM)
	  csr = CsrNumber::FCSR;
	return unsigned(csr) + csRegOffset_;
      }

    case OperandType::VecReg:   // FIX: Not yet supported.
    case OperandType::Imm:
    case OperandType::None:
      assert(0);
      return 0;
    }
  return 0;
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

  const auto* entry = di.instEntry();
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
	
      auto type = di.ithOperandType(i);
      // FIX: Support VecReg
      if (type == OperandType::VecReg or type == OperandType::Imm or type == OperandType::None)
	continue;
      if (isSource and type == OperandType::CsReg and skipCsr)
	continue;

      size_t regIx = effectiveRegIx(di, i);
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
  return std::ranges::any_of(instr.memOps_,
                             [this](auto opIx) { return opIx < sysMemOps_.size() &&
                                                        sysMemOps_.at(opIx).isRead_; });
}


template <typename URV>
bool
Mcm<URV>::instrHasWrite(const McmInstr& instr) const
{
  return std::ranges::any_of(instr.memOps_,
                             [this](auto opIx) { return opIx < sysMemOps_.size() &&
                                                        not sysMemOps_.at(opIx).isRead_; });
}


template <typename URV>
uint64_t
Mcm<URV>::earliestByteTime(const McmInstr& instr, uint64_t addr) const
{
  uint64_t time = 0;
  bool found = false;

  for (auto opIx : instr.memOps_)
    if (opIx < sysMemOps_.size())
      {
	const auto& op = sysMemOps_.at(opIx);
	if (op.physAddr_ <= addr and addr < op.physAddr_ + op.size_)
	  {
	    time = found? std::min(time, op.time_) : op.time_;
	    found = true;
	  }
      }

  return time;
}


template <typename URV>
uint64_t
Mcm<URV>::latestByteTime(const McmInstr& instr, uint64_t addr) const
{
  uint64_t time = ~uint64_t(0);
  bool found = false;

  for (auto opIx : instr.memOps_)
    if (opIx < sysMemOps_.size())
      {
	const auto& op = sysMemOps_.at(opIx);
	if (op.physAddr_ <= addr and addr < op.physAddr_ + op.size_)
	  {
	    time = found? std::max(time, op.time_) : op.time_;
	    found = true;
	  }
      }

  return time;
}


template <typename URV>
bool
Mcm<URV>::ppoRule1(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 1: B is a store, A and B have overlapping addresses.
  
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
	  cerr << "Error: ppoRule1: hart-id=" << hart.hartId()
	       << " tag1=" << instrA.tag_ << " tag2=" << instrB.tag_
	       << " instruction of tag1 is not retired\n";
	  return false;
	}

      if (not instrA.isMemory() or not instrA.overlaps(instrB))
	continue;

      if (instrA.isAligned() and instrB.isAligned())
	{
	  if (isBeforeInMemoryTime(instrA, instrB))
	    continue;
	}
      else
	{
	  // Check overlapped bytes.
	  bool ok = true;
	  for (unsigned i = 0; i < instrB.size_ and ok; ++i)
	    {
	      uint64_t byteAddr = instrB.physAddr_ + i;
	      if (instrA.physAddr_ <= byteAddr and byteAddr < instrA.physAddr_ + instrA.size_)
		{
		  uint64_t ta = latestByteTime(instrA, byteAddr);
		  uint64_t tb = earliestByteTime(instrB, byteAddr);
		  ok = ta < tb or (ta == tb and instrA.isStore_);
		}
	    }
	  if (ok)
	    continue;
	}

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
      if (instrA.memOps_.empty() or instrB.memOps_.empty())
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
	  if (op.isCanceled())
	    continue;
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
Mcm<URV>::processFence(Hart<URV>& hart, const McmInstr& instr)
{
  if (not instr.retired_ or not instr.di_.isValid())
    {
      cerr << "Mcm::processFence: Invalid/undecoded instruction\n";
      assert(0 && "Mcm::processFence: Invalid/undecoded instruction\n");
      return false;
    }

  unsigned hartIx = hart.sysHartIndex();

  auto& pendingFences = hartPendingFences_.at(hartIx);
  if (instr.di_.isFence())
    pendingFences.insert(instr.tag_);

  // If head of pending fence instructions is within window, process it.
  auto iter = pendingFences.begin();
  if (iter == pendingFences.end())
    return true;

  auto tag = *iter;
  if (instr.tag_ - tag < windowSize_)
    return true;

  pendingFences.erase(iter);
  auto& instrVec = hartInstrVecs_.at(hartIx);
  if (tag >= instrVec.size())
    {
      assert(0 && "Invalid tag in Mcm::processFence");
      return false;
    }
  const auto& pending = instrVec.at(tag);
  if (not pending.retired_ or not pending.di_.isValid() or not pending.di_.isFence())
    {
      assert(0 && "Invalid fence instruction in Mcm::processFence");
      return false;
    }

  return ppoRule4(hart, pending);
}


template <typename URV>
bool
Mcm<URV>::finalChecks(Hart<URV>& hart)
{
  unsigned hartIx = hart.sysHartIndex();
  auto& pendingFences = hartPendingFences_.at(hartIx);
  const auto& instrVec = hartInstrVecs_.at(hartIx);

  bool ok = true;

  for (auto fenceIx : pendingFences)
    {
      const auto& fence = instrVec.at(fenceIx);
      ok = ppoRule4(hart, fence) and ok;
    }

  pendingFences.clear();
  return ok;
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
      return false;
    }

  if (not instr.di_.isFence())
    return true;

  bool predRead = instr.di_.isFencePredRead();
  bool predWrite = instr.di_.isFencePredWrite();
  bool succRead = instr.di_.isFenceSuccRead();
  bool succWrite = instr.di_.isFenceSuccWrite();
  bool predIn = instr.di_.isFencePredInput();
  bool predOut = instr.di_.isFencePredInput();
  bool succIn = instr.di_.isFencePredInput();
  bool succOut = instr.di_.isFencePredInput();

  unsigned hartIx = hart.sysHartIndex();
  const auto& instrVec = hartInstrVecs_.at(hartIx);

  McmInstrIx begin = instr.tag_ >= windowSize_ ? instr.tag_ - windowSize_ : 0;
  McmInstrIx end = instr.tag_ + windowSize_;
  if (end >= instrVec.size())
    end = instrVec.size();

  bool ok = true;

  for (McmInstrIx predIx = begin; predIx < instr.tag_; ++predIx)
    {
      const McmInstr& pred = instrVec.at(predIx);
      if (instr.canceled_ or not pred.di_.isValid() or not pred.isMemory())
	continue;

      Pma predPma = hart.getPma(pred.physAddr_);

      if (not (predRead and pred.isLoad_)
	  and not (predWrite and pred.isStore_)
	  and not (predIn and pred.isLoad_ and predPma.isIo())
	  and not (predOut and pred.isStore_ and predPma.isIo()))
	continue;

      for (McmInstrIx succIx = instr.tag_ + 1; succIx < end; ++succIx)
	{
	  const McmInstr& succ = instrVec.at(succIx);
	  if (succ.canceled_ or not succ.di_.isValid() or not succ.isMemory())
	    continue;

	  Pma succPma = hart.getPma(pred.physAddr_);

	  if (not (succRead and succ.isLoad_)
	      and not (succWrite and succ.isStore_)
	      and not (succIn and succ.isLoad_ and succPma.isIo())
	      and not (succOut and succ.isStore_ and succPma.isIo()))
	    continue;

	  auto predTime = latestOpTime(pred);
	  auto succTime = earliestOpTime(succ);
	  if (predTime < succTime)
	    continue;

	  // Successor performs before predecessor -- Allow if successor is a load and
	  // there is no store from another core to the same cache line in between the
	  // predecessor and susscessor time.
	  if (not succ.isStore_)
	    {
	      auto low = std::lower_bound(sysMemOps_.begin(), sysMemOps_.end(), succTime,
					  [](const MemoryOp& op, const uint64_t& t) -> bool
					  { return op.time_ < t; });

	      auto high = std::upper_bound(low, sysMemOps_.end(), predTime,
					   [](const uint64_t& t, const MemoryOp& op) -> bool
					   { return t < op.time_; });

	      bool fail = false;
	      for (auto iter = low; iter != high and not fail; ++iter)
		{
		  auto& op = *iter;
		  fail = (not op.isRead_ and op.time_ >= succTime and op.time_ <= predTime
			  and op.hartIx_ != hartIx
			  and (op.physAddr_ / lineSize_) == (succ.physAddr_ / lineSize_));
		}
	      if (not fail)
		continue;
	    }

	  cerr << "Error: PPO rule 4 failed: hart-id=" << hart.hartId()
	       << " tag1=" << pred.tag_ << " tag2=" << succ.tag_
	       << " fence-tag=" << instr.tag_
	       << " time1=" << predTime << " time2=" << succTime << '\n';
	  ok = false;
	}
    }

  return ok;
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

  if (instrB.memOps_.empty())
    return true;

  unsigned hartIx = hart.sysHartIndex();
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

      bool hasAquire = instrA.di_.isAtomicAcquire();
      if (isTso_)
	hasAquire = hasAquire or instrA.di_.isLoad() or instrA.di_.isAmo();

      if (not hasAquire)
	continue;

      bool fail = false;
      if (instrA.di_.isAmo())
	fail = instrA.memOps_.size() != 2; // Incomplete amo might finish afrer B
      else if (not instrA.complete_)
	fail  = true; // Incomplete store might finish after B
      else
	{
	  auto timeB = earliestOpTime(instrB);
	  if (timeB <= latestOpTime(instrA))
	    {
	      // B peforms before A -- Allow if there is no write overlapping B after B
	      // from another core.
	      for (size_t ix = sysMemOps_.size(); ix != 0 and not fail; ix--)
		{
		  const auto& op = sysMemOps_.at(ix-1);
		  if (op.isCanceled())
		    continue;
		  if (op.time_ < timeB)
		    break;
		  if (not op.isRead_ and instrB.overlaps(op) and op.hartIx_ != hartIx)
		    fail = true;
		}
	    }
	}

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

  bool hasRelease = instrB.di_.isAtomicRelease();
  if (isTso_)
    hasRelease = hasRelease or instrB.di_.isStore() or instrB.di_.isAmo();

  if (not instrB.isMemory() or not hasRelease)
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
      else
        fail = (not instrA.complete_ or     // Incomplete store might finish after B
	        (not instrB.memOps_.empty() and
	         earliestOpTime(instrB) <= latestOpTime(instrA)));  // A finishes after B

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
Mcm<URV>::ppoRule7(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 7: A and B have RCsc annotations.
  if (instrB.isCanceled() or not instrB.di_.isValid())
    {
      cerr << "Mcm::ppoRule7: Instr B canceled/invalid\n";
      assert(0 && "Mcm::ppoRule7: Instr B canceled/invalid");
    }

  bool bHasRc = instrB.di_.isAtomicRelease() or instrB.di_.isAtomicAcquire();
  if (isTso_)
    bHasRc = bHasRc or instrB.di_.isLoad() or instrB.di_.isStore() or instrB.di_.isAmo();

  if (not instrB.isMemory() or not bHasRc)
    return true;

  const auto& instrVec = hartInstrVecs_.at(hart.sysHartIndex());

  for (McmInstrIx tag = instrB.tag_; tag > 0; --tag)
    {
      const auto& instrA =  instrVec.at(tag-1);
      if (instrA.isCanceled() or not instrA.isMemory())
	continue;
      if (not instrA.isRetired() or not instrA.di_.isValid())
	{
	  cerr << "Mcm::ppoRule7: Instr A invalid/not-retired\n";
	  assert(0 && "Mcm::ppoRule&: Instr A invalid/not-retired");
	}

      bool aHasRc = instrA.di_.isAtomicRelease() or instrA.di_.isAtomicAcquire();
      if (isTso_)
	aHasRc = bHasRc or instrA.di_.isLoad() or instrA.di_.isStore() or instrA.di_.isAmo();
      if (not aHasRc)
	continue;

      bool incomplete = not instrA.complete_ or 
	(instrA.di_.isAmo() and instrA.memOps_.size() != 2); // Incomplete amo might finish afrer B

      bool fail = (incomplete or     // Incomplete store might finish after B
		   (not instrB.memOps_.empty() and
		    earliestOpTime(instrB) <= latestOpTime(instrA)));  // A finishes after B

      if (fail)
	{
	  cerr << "Error: PPO rule 7 failed: hart-id=" << hart.hartId()
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

      if (not instrA.complete_ or
          (not instrB.memOps_.empty() and
           earliestOpTime(instrB) <= latestOpTime(instrA)))
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
      cerr << "Mcm::ppoRule9: Instr B canceled: tag=" << instrB.tag_ << "\n";
      return false;
    }

  if (not instrB.isMemory())
    return true;

  const auto& bdi = instrB.di_;
  if (bdi.isLoad() or bdi.isStore() or bdi.isAmo())
    {
      uint64_t addrTime = instrB.addrTime_;

      for (auto opIx : instrB.memOps_)
	{
	  if (opIx < sysMemOps_.size() and sysMemOps_.at(opIx).time_ <= addrTime)
	    {
	      cerr << "Error: PPO rule 9 failed: hart-id=" << hart.hartId() << " tag1="
		   << instrB.addrProducer_ << " tag2=" << instrB.tag_ << '\n';
	      return false;
	    }
	}
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

  const auto& bdi = instrB.di_;
  if (bdi.isStore() or bdi.isAmo())
    {
      uint64_t dataTime = instrB.dataTime_;

      for (auto opIx : instrB.memOps_)
	{
	  if (opIx < sysMemOps_.size() and sysMemOps_.at(opIx).time_ <= dataTime)
	    {
	      cerr << "Error: PPO rule 10 failed: hart-id=" << hart.hartId() << " tag1="
		   << instrB.dataProducer_  << " tag2=" << instrB.tag_ << '\n';
	      return false;
	    }
	}
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

  // Check all preceeding instructions for a store M with address
  // overlapping that of B. This is expensive. We need to keep set of
  // non-finished stores.
  size_t ix = std::min(size_t(instrB.tag_), instrVec.size());
  for ( ; ix ; --ix)
    {
      size_t mtag = ix - 1;
      const auto& instrM = instrVec.at(mtag);
      if (instrM.isCanceled() or not instrM.di_.isValid())
	continue;

      const auto& mdi = instrM.di_;
      if ((not mdi.isStore() and not mdi.isAmo()) or not instrM.overlaps(instrB))
	continue;

      auto apTag = instrM.addrProducer_;
      auto dpTag = instrM.dataProducer_;

      for (auto aTag : { apTag, dpTag } )
	{
	  const auto& instrA = instrVec.at(aTag);
	  if (instrA.di_.isValid())
	    if (not instrA.complete_ or isBeforeInMemoryTime(instrB, instrA))
	      {
		cerr << "Error: PPO rule 12 failed: hart-id=" << hart.hartId() << " tag1="
		     << aTag << " tag2=" << instrB.tag_ << " mtag=" << mtag
		     << " time1=" << latestOpTime(instrA)
		     << " time2=" << earliestOpTime(instrB) << '\n';
		return false;
	      }
	}
    }

  return true;
}


template class WdRiscv::Mcm<uint32_t>;
template class WdRiscv::Mcm<uint64_t>;
