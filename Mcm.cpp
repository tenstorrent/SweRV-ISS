#include <iostream>
#include <string_view>
#include "Mcm.hpp"
#include "System.hpp"
#include "Hart.hpp"

using namespace WdRiscv;
using std::cerr;


template <typename URV>
Mcm<URV>::Mcm(unsigned hartCount, unsigned pageSize, unsigned mergeBufferSize)
  : pageSize_(pageSize), lineSize_(mergeBufferSize)
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
  hartUndrainedStores_.resize(hartCount);

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
  assert(di.isValid());

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

  auto& storeSet = hartUndrainedStores_.at(hart.sysHartIndex());
  storeSet.insert(instrTag);

  bool result = true;

  if (writeOnInsert_)
    {
      // Associate write op with instruction.
      instr->addMemOp(sysMemOps_.size());
      sysMemOps_.push_back(op);
      instr->complete_ = checkStoreComplete(*instr);
      if (instr->complete_)
	storeSet.erase(instrTag);

      if (not instr->retired_)
	{
	  cerr << "Mcm::MergeBufferInsert: Error: Merge buffer write for a non-retired store\n";
	  return false;
	}

      if (not ppoRule1(hart, *instr))
	result = false;
      if (not ppoRule3(hart, *instr))
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

  auto& storeSet = hartUndrainedStores_.at(hart.sysHartIndex());

  if (not instr->retired_)
    storeSet.insert(instrTag);

  // Associate write op with instruction.
  instr->addMemOp(sysMemOps_.size());
  sysMemOps_.push_back(op);

  instr->complete_ = checkStoreComplete(*instr);
  if (instr->complete_)
    storeSet.erase(instrTag);

  bool result = pokeHartMemory(hart, physAddr, rtlData, size);

  if (instr->retired_)
    {
      result = checkRtlWrite(hart.hartId(), *instr, op) and result;
      if (instr->complete_)
	{
	  result = ppoRule1(hart, *instr) and result;
	  result = ppoRule3(hart, *instr) and result;
	}
    }
  else
    {
      // TODO: Mark instruction to avoid poking memory in retireStore
    }

  return result;
}


template <typename URV>
bool
Mcm<URV>::retireStore(Hart<URV>& hart, McmInstr& instr)
{
  uint64_t vaddr = 0, paddr = 0, paddr2 = 0, value = 0;
  unsigned stSize = hart.lastStore(vaddr, paddr, paddr2, value);
  if (not stSize)
    return true;   // Not a store.

  instr.size_ = stSize;
  instr.virtAddr_ = vaddr;
  instr.physAddr_ = paddr;
  instr.physAddr2_ = paddr2;
  instr.data_ = value;
  instr.isStore_ = true;
  instr.complete_ = checkStoreComplete(instr);

  auto& storeSet = hartUndrainedStores_.at(hart.sysHartIndex());

  if (not instr.complete_)
    {
      storeSet.insert(instr.tag_);
      return true;
    }

  storeSet.erase(instr.tag_);

  if (paddr == paddr2)
    return pokeHartMemory(hart, paddr, value, stSize);

  unsigned size1 = offsetToNextPage(paddr);
  bool ok = pokeHartMemory(hart, paddr, value, size1);

  unsigned size2 = stSize - size1;
  uint64_t value2 = value >> (size1 * 8);
  ok = pokeHartMemory(hart, paddr2, value2, size2) and ok;

  return ok;
}


template <typename URV>
bool
Mcm<URV>::retire(Hart<URV>& hart, uint64_t time, uint64_t tag,
		 const DecodedInst& di, bool trapped)
{
  unsigned hartIx = hart.sysHartIndex();
  cancelNonRetired(hartIx, tag);

  if (not updateTime("Mcm::retire", time))
    return false;

  McmInstr* instr = findOrAddInstr(hartIx, tag);
  if (instr->retired_)
    {
      cerr << "Mcm::retire: Error: Time=" << time << " hart-id=" << hartIx << " tag="
	   << tag << " Instruction retired multiple times\n";
      return false;
    }

  if (not di.isValid() or trapped)
    {
      cancelInstr(*instr);  // Instruction took a trap.
      return true;
    }

  instr->retired_ = true;
  instr->retireTime_ = time;
  instr->di_ = di;
  if (instr->di_.instId() == InstId::sfence_vma)
    {
      sinvalVmaTime_ = time;
      sinvalVmaTag_ = tag;
    }

  if (instr->di_.instId() == InstId::sfence_inval_ir)
    return checkSfenceInvalIr(hart, *instr);
  if (instr->di_.instId() == InstId::sfence_w_inval)
    return checkSfenceWInval(hart, *instr);


  // If instruction is a store, save address, size, and written data.
  bool ok = retireStore(hart, *instr);

  // Check read operations of instruction comparing RTL values to
  // memory model (whisper) values.
  for (auto opIx : instr->memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      auto& op = sysMemOps_.at(opIx);
      if (not op.isRead_)
	continue;

      if (not checkRtlRead(hart, *instr, op))
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

  if (instr->isStore_ and instr->complete_)
    {
      ok = checkStoreData(hartIx, *instr) and ok;
      ok = ppoRule1(hart, *instr) and ok;
      ok = ppoRule3(hart, *instr) and ok;
    }

  assert(di.isValid());

  ok = ppoRule2(hart, *instr) and ok;
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

  uint64_t lineAddr = physAddr - (physAddr % lineSize_);
  hart.cancelOtherHartsLr(physAddr);

  unsigned hartIx = hart.sysHartIndex();

  // Remove from hartPendingWrites_ the writes matching the RTL line
  // address and place them sorted by instr tag in coveredWrites.
  std::vector<MemoryOp> coveredWrites;
  if (not collectCoveredWrites(hart, time, physAddr, rtlSize, rtlMask, coveredWrites))
    return false;

  // Read our memory corresponding to RTL line addresses.
  uint64_t lineEnd = lineAddr + lineSize_;
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

  // Apply pending writes to our line and to memory.
  for (const auto& write : coveredWrites)
    {
      if ((write.physAddr_ < physAddr) or (write.physAddr_ + write.size_ > lineEnd))
	{
	  cerr << "Mcm::mergeBufferWrite: Store address out of line bound\n";
	  return false;
	}
      
      switch (write.size_)
	{
	case 1:
	  hart.pokeMemory(write.physAddr_, uint8_t(write.rtlData_), true);
	  break;
	case 2:
	  hart.pokeMemory(write.physAddr_, uint16_t(write.rtlData_), true);
	  break;
	case 4:
	  hart.pokeMemory(write.physAddr_, uint32_t(write.rtlData_), true);
	  break;
	case 8:
	  hart.pokeMemory(write.physAddr_, uint64_t(write.rtlData_), true);
	  break;
	default:
	  for (unsigned i = 0; i < write.size_; ++i)
	    hart.pokeMemory(write.physAddr_ + i, uint8_t(write.rtlData_ >> 8*i), true);
	  break;
	}

      unsigned ix = write.physAddr_ - physAddr;
      for (unsigned i = 0; i < write.size_; ++i)
	line.at(ix+i) = ((uint8_t*) &(write.rtlData_))[i];
    }

  // Compare covered writes.
  bool result = true;
  for (unsigned i = 0; i < line.size(); ++i)
    if (rtlMask.at(i) and (line.at(i) != rtlData.at(i)))
      {
	reportMismatch(hart.hartId(), time, "merge buffer write", physAddr + i,
		       rtlData.at(i), line.at(i));
	result = false;
	break;
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
	  auto& storeSet = hartUndrainedStores_.at(hartIx);
	  storeSet.erase(instr->tag_);
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
Mcm<URV>::writeToReadForward(const MemoryOp& writeOp, MemoryOp& readOp, uint64_t& mask)
{
  if (mask == 0)
    return true;  // No bytes left to forward.

  if (not readOp.overlaps(writeOp))
    return false;

  unsigned count = 0; // Count of forwarded bytes
  for (unsigned rix = 0; rix < readOp.size_ and mask != 0; ++rix)
    {
      uint64_t byteAddr = readOp.physAddr_ + rix;
      if (not writeOp.overlaps(byteAddr))
	continue;  // Read-op byte does not overlap write-op.

      uint64_t byteMask = uint64_t(0xff) << (rix * 8);
      if ((byteMask & mask) == 0)
	continue;  // Byte forwarded by another instruction.

      uint8_t byteVal = writeOp.rtlData_ >> (byteAddr - writeOp.physAddr_)*8;
      uint64_t aligned = uint64_t(byteVal) << 8*rix;
	
      readOp.data_ = (readOp.data_ & ~byteMask) | aligned;
      mask = mask & ~byteMask;
      count++;
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
Mcm<URV>::checkRtlRead(Hart<URV>& hart, const McmInstr& instr,
		       const MemoryOp& op) const
{
  // This is disabled until we resolve discrepancy over CLINT between
  // whisper config file and RTL.
  return true;

  if (op.size_ > instr.size_)
    {
      cerr << "Warning: Read operation size (" << unsigned(op.size_) << ") larger than "
	   << "instruction data size (" << unsigned(instr.size_) << "): Hart-id="
	   << hart.hartId() << " time=" << op.time_ << " tag=" << instr.tag_ << '\n';
    }

  uint64_t addr = op.physAddr_;
  bool skip = (hart.isAclintAddr(addr) or hart.isInterruptorAddr(addr, op.size_) or
	       hart.isImsicAddr(addr) or hart.isPciAddr(addr));
  if (skip)
    return true;

  if (op.rtlData_ != op.data_)
    {
      cerr << "Error: RTL/whisper read mismatch time=" << op.time_
	   << " hart-id=" << hart.hartId() << " instr-tag=" 
	   << op.instrTag_ << " addr=0x" << std::hex << addr
	   << " size=" << unsigned(op.size_) << " rtl=0x" << op.rtlData_
	   << " whisper=0x" << op.data_ << std::dec << '\n';
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
Mcm<URV>::clearMaskBitsForWrite(const McmInstr& store, const McmInstr& target,
				unsigned& mask) const
{
  if (not store.isStore_ or not target.isMemory() or not store.overlaps(target))
    return;

  if (not store.isRetired() or not target.isRetired())
    return;

  if (store.virtAddr_ <= target.virtAddr_)
    {
      uint64_t overlap = store.virtAddr_ + store.size_ - target.virtAddr_;
      if (overlap >= target.size_)
	mask = 0;
      else
	{
	  unsigned m = (1 << overlap) - 1;
	  mask = mask & ~m;
	}
      return;
    }
  
  uint64_t end = std::min(target.virtAddr_ + target.size_, store.virtAddr_ + store.size_);
  uint64_t overlap = end - store.virtAddr_;
  unsigned m = (1 << overlap) - 1;
  m = m << (store.virtAddr_ - target.virtAddr_);
  mask = mask & ~m;
}


/// Return a mask where the ith bit is set if addr + i is in the range
/// [cover, cover + coverSize - 1]
unsigned
maskCoveredBytes(uint64_t addr, unsigned size, uint64_t cover, unsigned coverSize)
{
  if (cover <= addr)
    {
      if (cover + coverSize > addr)
	{
	  uint64_t overlap = cover + coverSize - addr;
	  if (overlap > size)
	    overlap = size;
	  assert(overlap > 0 and overlap <= 8);
	  return (1 << overlap) - 1;
	}

      return 0;  // No overlap.
    }

  if (addr + size > cover)
    {
      uint64_t overlap = addr + size - cover;
      if (overlap > coverSize)
	overlap = coverSize;
      assert(overlap > 0 and overlap <= 8);
      unsigned mask = (1 << overlap) - 1;
      mask = mask << (cover - addr);
      return mask;
    }

  return 0;  // No overlap.
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
      if (addr == addr2)
	mask = maskCoveredBytes(addr, size, op.physAddr_, op.size_);
      else
	{
	  unsigned size1 = offsetToNextPage(addr);
	  if (pageNum(op.physAddr_) == pageNum(addr))
	    mask = maskCoveredBytes(addr, size1, op.physAddr_, op.size_);
	  else
	    {
	      unsigned size2 = size - size1;
	      mask = maskCoveredBytes(addr2, size2, op.physAddr_, op.size_);
	      mask = mask << size1;
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
  uint64_t addr = instr.physAddr_, addr2 = instr.physAddr2_, size = instr.size_;
  for (auto opIx : instr.memOps_)
    {
      if (opIx >= sysMemOps_.size())
	continue;
      const auto& op = sysMemOps_.at(opIx);
      if (not op.isRead_)
	continue;

      unsigned mask = 0;
      if (addr == addr2)
	mask = maskCoveredBytes(addr, size, op.physAddr_, op.size_);
      else
	{
	  unsigned size1 = offsetToNextPage(addr);
	  if (pageNum(op.physAddr_) == pageNum(addr))
	    mask = maskCoveredBytes(addr, size1, op.physAddr_, op.size_);
	  else
	    {
	      unsigned size2 = size - size1;
	      mask = maskCoveredBytes(addr2, size2, op.physAddr_, op.size_);
	      mask = mask << size1;
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


/// Return a mask of the bytes of the given address range that are
/// covered by the given memory operation. Bit i of the returned mask
/// will be set if byte at addr+i is covered by op.
unsigned
getMask(uint64_t addr, unsigned size, const MemoryOp& op)
{
  unsigned mask = 0;

  if (op.physAddr_ <= addr)
    {
      if (op.physAddr_ + op.size_ <= addr)
	return mask;  // Op does not overlap address range.
      uint64_t overlap = op.physAddr_ + op.size_ - addr;
      if (overlap > size)
	overlap = size;
      assert(overlap > 0 and overlap <= 8);
      mask = (1 << overlap) - 1;
    }
  else
    {
      if (addr + size <= op.physAddr_)
	return mask;  // Op does not overlap address range.
      uint64_t overlap = addr + size - op.physAddr_;
      if (overlap > op.size_)
	overlap = op.size_;
      mask = (1 << overlap) - 1;
      mask = mask << (op.physAddr_ - addr);
    }

  return mask;
}


template <typename URV>
unsigned
Mcm<URV>::determineOpMask(const McmInstr& instr, const MemoryOp& op) const
{
  unsigned size = instr.size_;
  uint64_t addr1 = instr.physAddr_, addr2 = instr.physAddr2_;

  if (addr1 == addr2)
    return getMask(addr1, size, op);

  unsigned size1 = offsetToNextPage(addr1);

  if (pageNum(op.physAddr_) == pageNum(addr1))
    {
      assert(size1 < size);
      return getMask(addr1, size1, op);
    }

  if (pageNum(op.physAddr_) == pageNum(addr2))
    {
      unsigned size2 = size - size1;
      unsigned mask = getMask(addr2, size2, op);
      mask = mask << size1;
      return mask;
    }

  return 0;  // no overlap.
}


/// If given memory operation overlaps the given address range then
/// set its high end to the end of the address range.
void
trimOp(MemoryOp& op, uint64_t addr, unsigned size)
{
  if (op.physAddr_ <= addr)
    {
      if (op.physAddr_ + op.size_ <= addr)
	return;  // Op does not overlap instruction.
      if (op.physAddr_ + op.size_ > addr + size)
	op.size_ = addr + size - op.physAddr_;  // Trim wide op.
    }
  else
    {
      if (addr + size <= op.physAddr_)
	return;  // Op does no overlap instruction.
      if (op.physAddr_ + op.size_ > addr + size)
	op.size_ = addr + size - op.physAddr_;  // Trim wide op.
    }
}


template <typename URV>
void
Mcm<URV>::trimMemoryOp(const McmInstr& instr, MemoryOp& op)
{
  unsigned size = instr.size_;
  uint64_t addr1 = instr.physAddr_, addr2 = instr.physAddr2_;

  if (addr1 == addr2)
    trimOp(op, addr1, size);
  else
    {
      unsigned size1 = offsetToNextPage(addr1);
      if (pageNum(op.physAddr_) == pageNum(addr1))
	{
	  assert(size1 < size);
	  trimOp(op, addr1, size1);
	}
      else if (pageNum(op.physAddr_) == pageNum(addr2))
	{
	  unsigned size2 = size - size1;
	  trimOp(op, addr2, size2);
	}
    }
}


template <typename URV>
void
Mcm<URV>::cancelReplayedReads(McmInstr* instr)
{
  assert(instr->size_ > 0 and instr->size_ <= 8);
  unsigned expectedMask = (1 << instr->size_) - 1;  // Mask of bytes covered by instruction.
  unsigned readMask = 0;    // Mask of bytes covered by read operations.

  auto& ops = instr->memOps_;
  for (auto& opIx : ops)
    trimMemoryOp(*instr, sysMemOps_.at(opIx));

  // Process read ops in reverse order so that later reads take precedence.
  size_t nops = instr->memOps_.size();
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
	  unsigned mask = determineOpMask(*instr, op);
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
      auto opIx = ops.at(i);
      if (opIx < sysMemOps_.size() and not sysMemOps_.at(opIx).isCanceled())
	{
	  if (count < i)
	    ops.at(count) = opIx;
	  count++;
	}
    }
  ops.resize(count);
}


template <typename URV>
bool
Mcm<URV>::getCurrentLoadValue(Hart<URV>& hart, uint64_t vaddr, uint64_t paddr1,
			      uint64_t paddr2, unsigned size, uint64_t& value)
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
  instr->virtAddr_ = vaddr;
  instr->physAddr_ = paddr1;
  if (paddr2 == paddr1 and pageNum(paddr1 + size - 1) != pageNum(paddr1))
    paddr2 = pageAddress(pageNum(paddr2) + 1);
  instr->physAddr2_ = paddr2;

  // Cancel early read ops that are covered by later ones. Trim wide reads.
  cancelReplayedReads(instr);

  uint64_t mergeMask = 0;
  uint64_t merged = 0;
  bool ok = true;
  size_t nops = instr->memOps_.size();
  for (size_t j = 0; j < nops; ++j)
    {
      auto opIx = instr->memOps_.at(nops - 1 - j);
      auto& op = sysMemOps_.at(opIx);
      if (not op.isRead_)
	continue;

      // Let forwarding override read-op data.
      forwardToRead(hart, op);

      uint64_t opVal = op.data_;
      uint64_t mask = ~uint64_t(0);
      if (pageNum(op.physAddr_) == pageNum(paddr1))
	{
	  if (op.physAddr_ <= paddr1)
	    {
	      uint64_t offset = paddr1 - op.physAddr_;
	      if (offset > 8)
		offset = 8;
	      opVal >>= offset*8;
	      mask >>= offset*8;
	    }
	  else
	    {
	      uint64_t offset = op.physAddr_ - paddr1;
	      if (offset > 8)
		offset = 8;
	      opVal <<= offset*8;
	      mask <<= offset*8;
	    }
	}
      else if (pageNum(op.physAddr_) == pageNum(paddr2))
	{
	  if (op.physAddr_ == paddr2)
	    {
	      uint64_t offset = offsetToNextPage(paddr1);
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

  unsigned forwardCount = 0, readCount = 0;
  for (auto opIx : instr->memOps_)
    {
      auto& op = sysMemOps_.at(opIx);
      if (op.isRead_)
	{
	  readCount++;
	  if (op.forwardTime_)
	    forwardCount++;
	}
    }
  if (readCount and forwardCount == readCount)
    instr->forwarded_ = true;

  instr->complete_ = true;  // FIX : Only for non-io

  return ok;
}
  

template <typename URV>
bool
Mcm<URV>::storeToReadForward(const McmInstr& store, MemoryOp& readOp, uint64_t& mask,
			     uint64_t addr, uint64_t data, unsigned size)
{
  if (mask == 0)
    return true;  // No bytes left to forward.

  if (store.isCanceled() or not store.isRetired() or not store.isStore_)
    return false;

  uint64_t rol = readOp.physAddr_, roh = readOp.physAddr_ + readOp.size_ - 1;
  uint64_t il = addr, ih = il + size - 1;
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
      for (const auto wopIx : store.memOps_)
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
      
      uint8_t byteVal = data >> (byteAddr - il)*8;
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
bool
Mcm<URV>::forwardToRead(Hart<URV>& hart, MemoryOp& readOp)
{
  auto hartIx = hart.sysHartIndex();
  const auto& instrVec = hartInstrVecs_.at(hartIx);

  const auto& undrained = hartUndrainedStores_.at(hartIx);

  std::set<McmInstrIx> stores;

  for (auto iter = undrained.rbegin(); iter != undrained.rend(); ++iter)
    {
      auto storeTag = *iter;
      const auto& store = instrVec.at(storeTag);
      if (store.isCanceled() or store.tag_ > readOp.instrTag_)
	continue;
      if (overlaps(store, readOp))
	stores.insert(store.tag_);
    }

  for (auto iter = sysMemOps_.rbegin(); iter != sysMemOps_.rend(); ++iter)
    {
      const auto& writeOp = *iter;
      if (writeOp.time_ < readOp.time_)
	break;

      if (writeOp.isCanceled()  or  writeOp.isRead_  or writeOp.hartIx_ != readOp.hartIx_  or
	  writeOp.instrTag_ >= readOp.instrTag_)
	continue;

      if (readOp.overlaps(writeOp))
	stores.insert(writeOp.instrTag_);
    }

  uint64_t mask = (~uint64_t(0)) >> (8 - readOp.size_)*8;

  for (auto iter = stores.rbegin(); iter != stores.rend() and mask != 0; ++iter)
    {
      auto storeTag = *iter;
      const auto& store = instrVec.at(storeTag);

      uint64_t prev = mask;

      if (not storeToReadForward(store, readOp, mask, store.physAddr_, store.data_, store.size_))
	{
	  if (store.physAddr_ == store.physAddr2_)
	    continue;
	  unsigned size1 = offsetToNextPage(store.physAddr_);
	  unsigned size2 = store.size_ - size1;
	  assert(size2 > 0 and size2 < 8);
	  uint64_t data2 = store.data_ >> size1 * 8;
	  storeToReadForward(store, readOp, mask, store.physAddr2_, data2, size2);
	}

      if (readOp.forwardTime_ == 0)
	readOp.forwardTime_ = earliestOpTime(store);
      else if (mask != prev)
	readOp.forwardTime_ = std::min(earliestOpTime(store), readOp.forwardTime_);
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

  if (entry->hasRoundingMode() and RoundingMode(di.roundingMode()) == RoundingMode::Dynamic)
    sourceRegs.push_back(unsigned(CsrNumber::FRM) + csRegOffset_);

  if (entry->modifiesFflags())
    destRegs.push_back(unsigned(CsrNumber::FFLAGS) + csRegOffset_);

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
      if (regIx == size_t(CsrNumber::FCSR) + csRegOffset_)
	{
	  if (isDest)
	    {
	      destRegs.push_back(size_t(CsrNumber::FFLAGS) + csRegOffset_);
	      destRegs.push_back(size_t(CsrNumber::FRM) + csRegOffset_);
	    }
	  if (isSource)
	    {
	      sourceRegs.push_back(size_t(CsrNumber::FFLAGS) + csRegOffset_);
	      sourceRegs.push_back(size_t(CsrNumber::FRM) + csRegOffset_);
	    }
	}
      else
	{
	  if (isDest)
	    destRegs.push_back(regIx);
	  if (isSource)
	    sourceRegs.push_back(regIx);
	}
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
Mcm<URV>::ppoRule1(const McmInstr& instrA, const McmInstr& instrB) const
{
  if (instrA.isCanceled())
    return true;

  assert(instrA.isRetired());

  if (not instrA.isMemory() or not instrA.overlaps(instrB))
    return true;

  if (instrA.isAligned() and instrB.isAligned())
    return isBeforeInMemoryTime(instrA, instrB);

  // Check overlapped bytes.
  bool ok = true;
  if (instrB.physAddr_ == instrB.physAddr2_)
    {    // Non-page crossing.
      for (unsigned i = 0; i < instrB.size_ and ok; ++i)
	{
	  uint64_t byteAddr = instrB.physAddr_ + i;
	  if (instrOverlapsPhysAddr(instrA, byteAddr))
	    {
	      uint64_t ta = latestByteTime(instrA, byteAddr);
	      uint64_t tb = earliestByteTime(instrB, byteAddr);
	      ok = ta < tb or (ta == tb and instrA.isStore_);
	    }
	}
    }
  else
    {    // Page crossing.
      unsigned size1 = offsetToNextPage(instrB.physAddr_);
      unsigned size2 = instrB.size_ - size1;
      for (unsigned i = 0; i < size1 and ok; ++i)
	{
	  uint64_t byteAddr = instrB.physAddr_ + i;
	  if (instrOverlapsPhysAddr(instrA, byteAddr))
	    {
	      uint64_t ta = latestByteTime(instrA, byteAddr);
	      uint64_t tb = earliestByteTime(instrB, byteAddr);
	      ok = ta < tb or (ta == tb and instrA.isStore_);
	    }
	}
      for (unsigned i = 0; i < size2 and ok; ++i)
	{
	  uint64_t byteAddr = instrB.physAddr2_ + i;
	  if (instrOverlapsPhysAddr(instrA, byteAddr))
	    {
	      uint64_t ta = latestByteTime(instrA, byteAddr);
	      uint64_t tb = earliestByteTime(instrB, byteAddr);
	      ok = ta < tb or (ta == tb and instrA.isStore_);
	    }
	}
    }

  return ok;
}


template <typename URV>
bool
Mcm<URV>::ppoRule1(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 1: B is a store, A and B have overlapping addresses.
  assert(instrB.di_.isValid());

  if (not instrB.complete_)
    return true;  // We will try again when B is complete.

  auto hartIx = hart.sysHartIndex();
  auto minTag = getSmallerMemTimeInstr(hartIx, instrB);
  const auto& instrVec = hartInstrVecs_.at(hartIx);

  for (McmInstrIx tag = instrB.tag_ - 1; tag > minTag; --tag)
    {
      const auto& instrA =  instrVec.at(tag);
      if (not ppoRule1(instrA, instrB))
	{
	  cerr << "Error: PPO rule 1 failed: hart-id=" << hart.hartId() << " tag1="
	       << instrA.tag_ << " tag2=" << instrB.tag_ << '\n';
	  return false;
	}
    }

  const auto& undrained = hartUndrainedStores_.at(hartIx);
  for (auto& tag : undrained)
    {
      if (tag >= instrB.tag_)
	break;
      const auto& instrA =  instrVec.at(tag);
      if (not ppoRule1(instrA, instrB))
	{
	  cerr << "Error: PPO rule 1 failed: hart-id=" << hart.hartId() << " tag1="
	       << instrA.tag_ << " tag2=" << instrB.tag_ << '\n';
	  return false;
	}
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule2(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 2: a and b are loads, x is a byte read by both a and b, there is no store to x
  // between a and b in program order, and a and b return values for x written by
  // different memory operations.
 
  // Instruction B must be a load/amo instruction.
  if (not instrB.isLoad_)
    return true;  // NA: B is not a load.

  if (instrB.forwarded_)
    return true;  // NA: B's data obtained entirely from local hart.

  unsigned hartIx = hart.sysHartIndex();
  auto minTag = getSmallerMemTimeInstr(hartIx, instrB);
  const auto& instrVec = hartInstrVecs_.at(hartIx);

  // Bit i of mask is 1 if ith byte of data of B is not written by a preceeding store from
  // the same hart.
  unsigned mask = (1 << instrB.size_) - 1;

  for (McmInstrIx tag = instrB.tag_ - 1; tag > minTag; --tag)
    {
      const auto& instrA =  instrVec.at(tag);
      if (instrA.isCanceled() or not instrA.isMemory() or not instrA.overlaps(instrB))
	continue;

      // Clear mask bits corresponding to bytes of B written by a preceeding store (A).
      clearMaskBitsForWrite(instrA, instrB, mask);
      if (mask == 0)
	return true; // All bytes of B written by preceeding stores.

      if (not instrA.isLoad_ or isBeforeInMemoryTime(instrA, instrB))
	continue;

      // Is there a remote write between A and B memory time that covers a byte of B.
      if (instrA.memOps_.empty() or instrB.memOps_.empty())
	{
	  cerr << "Error: PPO Rule 2: Instruction with no memory op: hart-id="
	       << hart.hartId() << " tag1=" << instrA.tag_ << " tag2=" << instrB.tag_ << '\n';
	  return false;
	}
      uint64_t ix0 = instrB.memOps_.front();
      uint64_t ix1 = instrA.memOps_.back();

      for (uint64_t ix = ix0; ix <= ix1; ++ix)
	{
	  const MemoryOp& remoteOp = sysMemOps_.at(ix);
	  if (remoteOp.isCanceled() or remoteOp.hartIx_ == hartIx or remoteOp.isRead_)
	    continue;

	  // Check the of bytes of the remote write. If the address of any of them
	  // overlaps A and B and corresponding time is between times of A and B, we fail.
	  for (unsigned byteIx = 0; byteIx < remoteOp.size_; ++byteIx)
	    {
	      uint64_t addr = remoteOp.physAddr_ + byteIx;

	      if (not overlapsAddr(instrA, addr) or not overlapsAddr(instrB, addr))
		continue;

	      unsigned byteMask = 1 << (addr - instrB.physAddr_);  // Non page crossing
	      if (instrB.physAddr_ != instrB.physAddr2_) // Page crossing
		{
		  unsigned size1 = offsetToNextPage(instrB.physAddr_);
		  if (not (addr > instrB.physAddr_ and addr < instrB.physAddr_ + size1))
		    byteMask = 1 << (size1 + addr - instrB.physAddr2_);
		}

	      if (not (byteMask & mask))
		continue;   // Byte of B covered by local store.

	      auto earlyB = earliestByteTime(instrB, addr);
	      auto lateA = latestByteTime(instrA, addr);

	      auto rot = remoteOp.time_;
	      if (earlyB <= lateA and earlyB <= rot and rot <= lateA)
		{
		  cerr << "Error: PPO Rule 2 failed: hart-id=" << hart.hartId()
		       << " tag1=" << instrA.tag_ << " tag2=" << instrB.tag_
		       << " intermediate remote store from hart-id="
		       << remoteOp.hartIx_ << " store-tag=" << remoteOp.instrTag_
		       << " store-time=" << remoteOp.time_ << '\n';
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
 
  // Instruction B must be a load/amo instruction.
  const DecodedInst& bdi = instrB.di_;
  if (bdi.isStore())
    return true;  // NA: store instruction.

  if (not bdi.isLoad() and not bdi.isAtomic())
    return true;  // NA: must be load/amo.

  if (not instrB.complete_)
    return true;  // We will try again when B is complete.

  // Bit i of mask is 1 if byte i of data of instruction B is not written by a preceeding
  // store.
  unsigned mask = (1 << instrB.size_) - 1;

  unsigned hartIx = hart.sysHartIndex();
  auto minTag = getSmallerMemTimeInstr(hartIx, instrB);
  const auto& instrVec = hartInstrVecs_.at(hartIx);

  for (McmInstrIx tag = instrB.tag_ - 1; tag > minTag; --tag)
    {
      const auto& instrA =  instrVec.at(tag);
      if (instrA.isCanceled())
	continue;
      assert(instrA.isRetired());

      if (not instrA.isStore_ or not instrA.overlaps(instrB))
	continue;

      // If A is not atomic remove from mask the bytes of B that are covered by A. Done
      // when all bytes of B are covered.
      assert(instrA.di_.isValid());
      if (not instrA.di_.isAtomic())
	{
	  clearMaskBitsForWrite(instrA, instrB, mask);
	  if (mask == 0)
	    return true;   // All bytes of B written by non-atomic stores.
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
  assert(instr.isRetired());

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

  auto& pendingWrites = hartPendingWrites_.at(hartIx);
  if (not pendingWrites.empty())
    cerr << "Warning: Merge buffer is not empty at end of run.\n";

  uint64_t toHost = 0;
  bool hasToHost = hart.getToHostAddress(toHost);

  const auto& storeSet = hartUndrainedStores_.at(hartIx);
  for (auto tag : storeSet)
    {
      const auto& instr = instrVec.at(tag);
      if (not hasToHost or toHost != instr.virtAddr_)
	cerr << "Warning: Hart-id=" << hart.hartId() << " tag=" << instr.tag_
	     << " Store instruction is not drained at end of run.\n";
    }

  return ok;
}


template <typename URV>
uint64_t
Mcm<URV>::effectiveReadTime(const McmInstr& instr) const
{

  if (not instr.isLoad_)
    return earliestOpTime(instr);

  if (not instr.complete_ and instr.memOps_.empty())
    return time_;

  uint64_t mint = ~uint64_t(0);
  for (auto opIx : instr.memOps_)
    if (opIx < sysMemOps_.size())
      {
	auto& op = sysMemOps_.at(opIx);
	uint64_t t = op.time_;
	if (op.isRead_ and op.forwardTime_ and op.forwardTime_ > t)
	  t = op.forwardTime_;
	mint = std::min(mint, t);
      }

  return mint;
}


template <typename URV>
bool
Mcm<URV>::ppoRule4(Hart<URV>& hart, const McmInstr& instr) const
{
  // Rule 4: There is a fence that orders A before B.

  assert(instr.isRetired());

  if (not instr.di_.isFence())
    return true;

  bool predRead = instr.di_.isFencePredRead();
  bool predWrite = instr.di_.isFencePredWrite();
  bool succRead = instr.di_.isFenceSuccRead();
  bool succWrite = instr.di_.isFenceSuccWrite();
  bool predIn = instr.di_.isFencePredInput();
  bool predOut = instr.di_.isFencePredOutput();
  bool succIn = instr.di_.isFencePredInput();
  bool succOut = instr.di_.isFencePredOutput();

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

	  if (not pred.complete_ or not pred.retired_)
	    {
	      cerr << "Error: PPO rule 4 failed: hart-id=" << hart.hartId()
		   << " tag1=" << pred.tag_ << " fence-tag=" << instr.tag_
		   << " memory instruction before fence is not retired/complate\n";
	      return false;
	    }

	  auto predTime = latestOpTime(pred);
	  auto succTime = effectiveReadTime(succ);

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
Mcm<URV>::ppoRule5(Hart<URV>& hart, const McmInstr& instrA, const McmInstr& instrB) const
{
  if (instrA.isCanceled() or not instrA.isMemory())
    return true;

  assert(instrA.isRetired());

  bool hasAquire = instrA.di_.isAtomicAcquire();
  if (isTso_)
    hasAquire = hasAquire or instrA.di_.isLoad() or instrA.di_.isAmo();

  if (not hasAquire)
    return true;

  if (instrA.di_.isAmo())
    return instrA.memOps_.size() == 2; // Fail if != 2: Incomplete amo might finish afrer B

  if (not instrA.complete_)
    return false; // Incomplete store might finish after B

  auto timeA = latestOpTime(instrA);
  auto timeB = effectiveReadTime(instrB);

  if (timeB > timeA)
    return true;

  auto hartIx = hart.sysHartIndex();

  // B peforms before A -- Allow if there is no write overlapping B between A and B from
  // another core.
  for (size_t ix = sysMemOps_.size(); ix != 0; ix--)
    {
      const auto& op = sysMemOps_.at(ix-1);
      if (op.isCanceled() or op.time_ > timeA)
	continue;
      if (op.time_ < timeB)
	break;
      if (not op.isRead_ and overlaps(instrB, op) and op.hartIx_ != hartIx)
	return false;
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule5(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 5: A has an acquire annotation

  if (not instrB.isMemory() or instrB.memOps_.empty())
    return true;

  unsigned hartIx = hart.sysHartIndex();
  const auto& instrVec = hartInstrVecs_.at(hartIx);
  auto minTag = getSmallerMemTimeInstr(hartIx, instrB);

  for (McmInstrIx tag = instrB.tag_ - 1; tag > minTag; --tag)
    {
      const auto& instrA =  instrVec.at(tag);
      if (not ppoRule5(hart, instrA, instrB))
	{
	  cerr << "Error: PPO rule 5 failed: hart-id=" << hart.hartId()
	       << " tag1=" << instrA.tag_ << " tag2=" << instrB.tag_ << '\n';
	  return false;
	}
    }

  const auto& undrained = hartUndrainedStores_.at(hartIx);
  for (auto& tag : undrained)
    {
      if (tag >= instrB.tag_)
	break;
      const auto& instrA =  instrVec.at(tag);
      if (not ppoRule5(hart, instrA, instrB))
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
Mcm<URV>::ppoRule6(const McmInstr& instrA, const McmInstr& instrB) const
{
  bool hasRelease = instrB.di_.isAtomicRelease();
  if (isTso_)
    hasRelease = hasRelease or instrB.di_.isStore() or instrB.di_.isAmo();

  if (not instrB.isMemory() or not hasRelease)
    return true;

  if (instrA.isCanceled() or not instrA.isMemory())
    return true;

  assert(instrA.isRetired());

  if (instrA.di_.isAmo())
    return instrA.memOps_.size() == 2; // Fail if incomplete amo (finishes afrer B).

  if (not instrA.complete_)
    return false;       // Fail if incomplete store (finishes after B).

  if (instrB.memOps_.empty())
    return true;   // Undrained store.

  auto btime = earliestOpTime(instrB);
  return latestOpTime(instrA) < btime;  // A finishes brefore B.
}


template <typename URV>
bool
Mcm<URV>::ppoRule6(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 6: B has a release annotation


  auto hartIx = hart.sysHartIndex();
  auto minTag = getSmallerMemTimeInstr(hartIx, instrB);
  const auto& instrVec = hartInstrVecs_.at(hartIx);

  for (McmInstrIx tag = instrB.tag_ - 1; tag > minTag; --tag)
    {
      const auto& instrA =  instrVec.at(tag);
      if (not ppoRule6(instrA, instrB))
	{
	  cerr << "Error: PPO rule 6 failed: hart-id=" << hart.hartId()
	       << " tag1=" << instrA.tag_ << " tag2=" << instrB.tag_ << '\n';
	  return false;
	}
    }

  const auto& undrained = hartUndrainedStores_.at(hartIx);
  for (auto& tag : undrained)
    {
      if (tag >= instrB.tag_)
	break;
      const auto& instrA =  instrVec.at(tag);
      if (not ppoRule6(instrA, instrB))
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
Mcm<URV>::ppoRule7(const McmInstr& instrA, const McmInstr& instrB) const
{
  if (instrA.isCanceled() or not instrA.isMemory())
    return true;

  assert(instrA.isRetired());

  bool bHasRc = instrB.di_.isAtomicRelease() or instrB.di_.isAtomicAcquire();
  if (isTso_)
    bHasRc = bHasRc or instrB.di_.isLoad() or instrB.di_.isStore() or instrB.di_.isAmo();

  bool aHasRc = instrA.di_.isAtomicRelease() or instrA.di_.isAtomicAcquire();
  if (isTso_)
    aHasRc = bHasRc or instrA.di_.isLoad() or instrA.di_.isStore() or instrA.di_.isAmo();

  if (not aHasRc or not bHasRc)
    return true;

  bool incomplete = not instrA.complete_ or (instrA.di_.isAmo() and instrA.memOps_.size() != 2);
  if (incomplete)
    return false;   // Incomplete amo finishes afrer B.

  if (instrB.memOps_.empty())
    return true;    // Undrained store will finish later.

  auto btime = earliestOpTime(instrB);
  return latestOpTime(instrA) < btime;  // A finishes before B
}


template <typename URV>
bool
Mcm<URV>::ppoRule7(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 7: A and B have RCsc annotations.

  bool bHasRc = instrB.di_.isAtomicRelease() or instrB.di_.isAtomicAcquire();
  if (isTso_)
    bHasRc = bHasRc or instrB.di_.isLoad() or instrB.di_.isStore() or instrB.di_.isAmo();

  if (not instrB.isMemory() or not bHasRc)
    return true;

  auto hartIx = hart.sysHartIndex();
  auto minTag = getSmallerMemTimeInstr(hartIx, instrB);
  const auto& instrVec = hartInstrVecs_.at(hartIx);

  for (McmInstrIx tag = instrB.tag_ - 1; tag > minTag; --tag)
    {
      const auto& instrA =  instrVec.at(tag);
      if (not ppoRule7(instrA, instrB))
	{
	  cerr << "Error: PPO rule 7 failed: hart-id=" << hart.hartId()
	       << " tag1=" << instrA.tag_ << " tag2=" << instrB.tag_ << '\n';
	  return false;
	}
    }

  const auto& undrained = hartUndrainedStores_.at(hartIx);
  for (auto& tag : undrained)
    {
      if (tag >= instrB.tag_)
	break;
      const auto& instrA =  instrVec.at(tag);
      if (not ppoRule7(instrA, instrB))
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
  if (not instrB.isMemory() or not instrB.di_.isSc())
    return true;

  uint64_t addr = 0, value = 0;
  if (not hart.lastStore(addr, value))
    return true;  // Score conditional was not successful.

  auto hartIx = hart.sysHartIndex();
  auto minTag = getSmallerMemTimeInstr(hartIx, instrB);
  const auto& instrVec = hartInstrVecs_.at(hartIx);
  auto btime = earliestOpTime(instrB);

  for (McmInstrIx tag = instrB.tag_ - 1; tag > minTag; --tag)
    {
      const auto& instrA =  instrVec.at(tag);
      if (instrA.isCanceled())
	continue;

      assert(instrA.isRetired());

      // Read happen before retire, instrucions retire in program order, if an instruction
      // retires before B performs, no eralier instruction can read after B.
      if (instrA.retireTime_ < btime)
	break;

      if (not instrA.di_.isLr())
	continue;

      if (not instrA.complete_ or
          (not instrB.memOps_.empty() and btime <= latestOpTime(instrA)))
	{
	  cerr << "Error: PPO rule 8 failed: hart-id=" << hart.hartId()
	       << " tag1=" << instrA.tag_ << " tag2=" << instrB.tag_ << '\n';
	  return false;
	}

      return true;
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::ppoRule9(Hart<URV>& hart, const McmInstr& instrB) const
{
  // Rule 9: B has a syntactic address dependency on A

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

  if (not instrB.isLoad_)
    return true;  // NA: B is not a load.

  unsigned hartIx = hart.sysHartIndex();
  const auto& instrVec = hartInstrVecs_.at(hartIx);
  if (instrVec.empty() or instrB.tag_ == 0)
    return true;  // Nothing before B in instruction order.

  auto earlyB = earliestOpTime(instrB);

  // Check all preceeding instructions for a store M with address overlapping that of B.
  size_t ix = std::min(size_t(instrB.tag_), instrVec.size());
  for ( ; ix ; --ix)
    {
      size_t mtag = ix - 1;
      const auto& instrM = instrVec.at(mtag);
      if (instrM.isCanceled() or not instrM.di_.isValid())
	continue;

      // Quit if we find a load with retire time earlier than B load time. Found load
      // would be an A that will not fail (all preceeding As will not fail either). M will
      // only depend on loads and will never depend on a store.
      if (instrM.isLoad_ and instrM.retireTime_ < earlyB)
	break;

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
		     << " time2=" << earlyB << '\n';
		return false;
	      }
	}
    }

  return true;
}


template <typename URV>
bool
Mcm<URV>::checkSfenceInvalIr(Hart<URV>& hart, const McmInstr& instr) const
{
  // This is very crude. We are using retire time of sinval.vma, we should be using
  // execution time. We are using read/write times of ld/store instructions, we should be
  // using translation times. Required times are currently not available. We do not
  // consider instruction address translation.

  if (sinvalVmaTag_ == 0)
    return true;   // No sinval.vma was retired

  unsigned hartIx = hart.sysHartIndex();

  for (size_t ix = sysMemOps_.size(); ix > 0; --ix)
    {
      const auto& op = sysMemOps_.at(ix-1);
      if (op.canceled_ or op.hartIx_ != hartIx)
	continue;
      if (op.instrTag_ < instr.tag_)
	break;
      if (op.time_ < sinvalVmaTime_)
	{
	  cerr << "Error: Hart-id=" << hart.hartId() << "implicit memory access for "
	       << "translation of instruction at tag=" << op.instrTag_ << " is out of order "
	       << "with respect to sinval.vma instruction with tag=" << sinvalVmaTag_
	       << " and sfence.inval.ir with tag=" << instr.tag_ << '\n';
	  return false;
	}
    }

  return true;
}  


template <typename URV>
bool
Mcm<URV>::checkSfenceWInval(Hart<URV>& hart, const McmInstr& instr) const
{
  // This is very crude: Check that there are no pending stores (stores in the
  // store/merge buffer) when the sfence.w.inval is retired.

  unsigned hartIx = hart.sysHartIndex();
  auto& pendingWrites = hartPendingWrites_.at(hartIx);
  if (pendingWrites.empty())
    return true;

  cerr << "Error: Hart-id=" << hart.hartId() << "sfence.w.inval tag=" << instr.tag_
       << " retired while there are pending stores in the store/merge buffer.\n";
  return false;
}


template class WdRiscv::Mcm<uint32_t>;
template class WdRiscv::Mcm<uint64_t>;
