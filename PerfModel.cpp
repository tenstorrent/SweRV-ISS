#include <cinttypes>
#include "PerfModel.hpp"

using namespace TT_PERF;


PerfApi::PerfApi(System64& system)
  : system_(system)
{
  unsigned n = system.hartCount();

  hartPacketMaps_.resize(n);
  hartStoreMaps_.resize(n);
  hartLastRetired_.resize(n);
  hartRegProducers_.resize(n);

  for (auto& producers : hartRegProducers_)
    producers.resize(totalRegCount_);
}


std::shared_ptr<Hart64>
PerfApi::checkHart(const char* caller, unsigned hartIx)
{
  auto hart = getHart(hartIx);
  if (not hart)
    {
      std::cerr << caller << ": Bad hart index: " << hartIx << '\n';
      assert(0);
    }
  return hart;
}


std::shared_ptr<InstrPac>
PerfApi::checkTag(const char* caller, unsigned hartIx, uint64_t tag)
{
  auto& packetMap = hartPacketMaps_.at(hartIx);
  auto iter = packetMap.find(tag);
  if (iter != packetMap.end())
    return iter->second;
  std::cerr << caller << ": Bad tag: " << tag << '\n';
  assert(0);
  return nullptr;
}


bool
PerfApi::checkTime(const char* caller, uint64_t time)
{
  if (time < time_)
    {
      std::cerr << caller << ": Bad time: " << time << '\n';
      assert(0);
      return false;
    }
  time_ = time;
  return true;
}


bool
PerfApi::fetch(unsigned hartIx, uint64_t time, uint64_t tag, uint64_t vpc,
	       bool& trap, ExceptionCause& cause, uint64_t& trapPc)
{
  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_fetch %" PRIu64 " 0x%" PRIx64 "\n",
            hartIx, time, tag, vpc);

  auto hart = checkHart("Fetch", hartIx);
  if (not hart)
    return false;

  if (not checkTime("Fetch", time))
    return false;

  auto& packetMap = hartPacketMaps_.at(hartIx);
  if (not packetMap.empty() and packetMap.rbegin()->first >= tag)
    {
      std::cerr << "Error in PerfApi::fetch: tag " << tag << "is not in increasing order.\n";
      assert(0);
      return false;
    }

  auto packet = getInstructionPacket(hartIx, tag);
  if (packet)
    return false;   // Tag already fetched.

  auto prev = this->prevFetch_;
  if (prev and not prev->predicted() and not prev->trapped() and not prev->executed())
    {
      if (not prev->decodedInst().isBranch())
	if (prev->instrVa() + prev->decodedInst().instSize() != vpc)
	  prev->predictBranch(true, vpc);
    }

  uint64_t ppc = 0, ppc2 = 0;  // Physical pc.
  uint32_t opcode = 0;
  uint64_t gpc = 0; // Guest physical pc.
  cause = hart->fetchInstNoTrap(vpc, ppc, ppc2, gpc, opcode);

  if (cause == ExceptionCause::NONE)
    {
      packet = std::make_shared<InstrPac>(tag, vpc, ppc, ppc2);
      assert(packet);
      packet->fetched_ = true;
      insertPacket(hartIx, tag, packet);
      if (not decode(hartIx, time, tag, opcode))
	assert(0);
      prevFetch_ = packet;
      trap = false;
    }
  else
    {
      prevFetch_ = nullptr;
      trap = true;
      assert(0 && "get trap pc");
      trapPc = 0;
    }

  return true;
}


bool
PerfApi::decode(unsigned hartIx, uint64_t time, uint64_t tag, uint32_t opcode)
{
  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_decode %" PRIu64 " 0x%" PRIx32 "\n",
            hartIx, time, tag, opcode);

  if (not checkTime("Decode", time))
    return false;

  auto hart = checkHart("Decode", hartIx);
  if (not hart)
    return false;

  auto packet = checkTag("Decode", hartIx, tag);
  if (not packet)
    return false;

  if (packet->decoded())
    return true;

  hart->decode(packet->instrVa(), packet->instrPa(), opcode, packet->di_);
  packet->decoded_ = true;

  // Collect producers of operands of this instruction.
  auto& di = packet->decodedInst();
  auto& producers = hartRegProducers_.at(hartIx);
  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      using OM = WdRiscv::OperandMode;

      auto mode = di.ithOperandMode(i);
      if (mode != OM::None)
	{
	  unsigned regNum = di.ithOperand(i);
	  unsigned gri = globalRegIx(di.ithOperandType(i), regNum);
	  packet->opProducers_.at(i) = producers.at(gri);
	}
    }

  // Mark this insruction as the producer of each of its destination registers.
  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      using OM = WdRiscv::OperandMode;

      auto mode = di.ithOperandMode(i);
      if (mode == OM::Write or mode == OM::ReadWrite)
	{
	  unsigned regNum = di.ithOperand(i);
	  unsigned gri = globalRegIx(di.ithOperandType(i), regNum);
	  if (regNum == 0 and di.ithOperandType(i) == WdRiscv::OperandType::IntReg)
	    continue;  // Reg X0 has no producer
	  producers.at(gri) = packet;
	}
    }

  return true;
}


bool
PerfApi::execute(unsigned hartIx, uint64_t time, uint64_t tag)
{
  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_execute %" PRIu64 "\n",
            hartIx, time, tag);

  if (not checkTime("Execute", time))
    return false;

  auto hart = checkHart("Execute", hartIx);
  if (not hart)
    return false;

  auto pacPtr = checkTag("execute", hartIx, tag);
  if (not pacPtr)
    return false;

  auto& packet = *pacPtr;

  auto& di = packet.decodedInst();

  if (packet.executed())
    {
      // Instruction is being re-executed. Must be load/store. Every instruction that
      // depends on it must be re-executed.
      if (not di.isLoad() and not di.isStore())
	assert(0);
      auto& packetMap = hartPacketMaps_.at(hartIx);
      auto iter = packetMap.find(packet.tag());
      assert(iter != packetMap.end());
      for ( ; iter != packetMap.end(); ++iter)
	{
	  auto succ = iter->second;   // Successor packet
	  if (succ->dependsOn(packet))
	    succ->executed_ = false;
	}
    }

  // Collect register operand values.
  assert(di.operandCount() <= packet.opVal_.size());
  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      if (di.ithOperandMode(i) == WdRiscv::OperandMode::None)
	continue;
      unsigned regNum = di.ithOperand(i);
      unsigned gri = globalRegIx(di.ithOperandType(i), regNum);
      uint64_t value = 0;

      auto& producer = packet.opProducers_.at(i);
      if (producer)
	value = getDestValue(*producer, gri);
      else if (not peekRegister(*hart, di.ithOperandType(i), regNum, value))
	assert(0);
      packet.opVal_.at(i) = value;
    }

  // Execute the instruction: Poke source register values, execute, recover destination
  // values.
  if (not execute(hartIx, packet))
    assert(0);

  packet.executed_ = true;
  packet.execTime_ = time;
  if (packet.isBranch() and packet.predicted_)
    packet.mispredicted_ = packet.prTarget_ != packet.nextIva_;

  return true;
}


bool
PerfApi::execute(unsigned hartIx, InstrPac& packet)
{
  assert(packet.decoded());

  auto hartPtr = getHart(hartIx);
  assert(hartPtr);
  auto& hart = *hartPtr;

  using OM = WdRiscv::OperandMode;
  using OT = WdRiscv::OperandType;

  uint64_t prevPc = hart.peekPc();

  // Don't execute if on wrong path to avoid potential exception.
  uint32_t opcode = 0;
  if (not hartPtr->readInst(packet.iva_, opcode) or packet.di_.inst() != opcode)
    {
      packet.shouldFlush_ = true;
      return true;  // Wrong opcode. Must be flushed. Pretend it was executed.
    }

  hart.pokePc(packet.instrVa());

  std::array<uint64_t, 3> prevVal;  // Previous operand values

  // Save prev value of poperands.
  for (unsigned i = 0; i < packet.di_.operandCount(); ++i)
    {
      auto mode = packet.di_.ithOperandMode(i);
      auto type = packet.di_.ithOperandType(i);
      uint32_t operand = packet.di_.ithOperand(i);
      if (mode == OM::None)
	continue;

      switch (type)
	{
	case OT::IntReg:
	  if (not hart.peekIntReg(operand, prevVal.at(i)))
	    assert(0);
	  break;
	case OT::FpReg:
	  if (not hart.peekFpReg(operand, prevVal.at(i)))
	    assert(0);
	  break;
	case OT::CsReg:
	  if (not hart.peekCsr(WdRiscv::CsrNumber(operand), prevVal.at(i)))
	    assert(0);
	  break;
	case OT::VecReg:
	  assert(0);
	  break;
	case OT::Imm:
	  break;
	default:
	  assert(0);
	  break;
	}
    }

  // Poke packet operands into hart.
  for (unsigned i = 0; i < packet.di_.operandCount(); ++i)
    {
      auto mode = packet.di_.ithOperandMode(i);
      auto type = packet.di_.ithOperandType(i);
      uint32_t operand = packet.di_.ithOperand(i);
      uint64_t val = packet.opVal_.at(i);
      if (mode == OM::None)
	continue;

      switch (type)
	{
	case OT::IntReg:
	  if (not hart.pokeIntReg(operand, val))
	    assert(0);
	  break;
	case OT::FpReg:
	  if (not hart.pokeFpReg(operand, val))
	    assert(0);
	  break;
	case OT::CsReg:
	  if (not hart.pokeCsr(WdRiscv::CsrNumber(operand), val))
	    assert(0);
	  break;
	case OT::VecReg:
	  assert(0);
	  break;
	case OT::Imm:
	  break;
	default:
	  assert(0);
	  break;
	}
    }

  hart.singleStep();

  auto& di = packet.decodedInst();
  if (di.isLoad())
    packet.dsize_ = di.loadSize();
  else if (di.isStore())
    {
      uint64_t sva = 0, spa1 = 0, spa2 = 0, sval = 0;
      unsigned ssize = hart.lastStore(sva, spa1, spa2, sval);
      if (ssize == 0)
	assert(0);

      packet.dva_ = sva;
      packet.dpa_ = spa1;  // FIX TODO : handle page corrsing
      packet.dsize_ = di.storeSize();
      assert(ssize = packet.dsize_);

      auto& storeMap =  hartStoreMaps_.at(hartIx);
      storeMap[packet.tag()] = getInstructionPacket(hartIx, packet.tag());
    }
  else if (packet.isBranch())
    packet.nextIva_ = hart.peekPc();

  // Record the values of the dstination register.
  unsigned destIx = 0;
  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      auto mode = di.ithOperandMode(i);
      if (mode == OM::Write or mode == OM::ReadWrite)
	{
	  unsigned regNum = di.ithOperand(i);
	  unsigned gri = globalRegIx(di.ithOperandType(i), regNum);
	  uint64_t destVal = 0;
	  if (not peekRegister(hart, di.ithOperandType(i), regNum, destVal))
	    assert(0);
	  packet.destValues_.at(destIx) = InstrPac::DestValue(gri, destVal);
	  destIx++;
	}
    }

  // Memory should not have changed.

  // Restore additional CSRs modified by the execution (in case of a trap).
  std::vector<WdRiscv::CsrNumber> csrns;
  hart.lastCsr(csrns);
  for (auto csrn : csrns)
    {
      uint64_t value = hart.lastCsrValue(csrn);
      if (not hart.pokeCsr(csrn, value))
	assert(0);
    }

  // Restore hart status.
  for (unsigned i = 0; i < packet.di_.operandCount(); ++i)
    {
      auto mode = packet.di_.ithOperandMode(i);
      auto type = packet.di_.ithOperandType(i);
      uint32_t operand = packet.di_.ithOperand(i);
      uint64_t prev = prevVal.at(i);
      if (mode == OM::None)
	continue;

      switch (type)
	{
	case OT::IntReg:
	  if (not hart.pokeIntReg(operand, prev))
	    assert(0);
	  break;
	case OT::FpReg:
	  if (not hart.pokeFpReg(operand, prev))
	    assert(0);
	  break;
	case OT::CsReg:
	  if (not hart.pokeCsr(WdRiscv::CsrNumber(operand), prev))
	    assert(0);
	  break;
	case OT::VecReg:
	  assert(0);
	  break;
	default:
	  assert(0);
	  break;
	}
    }

  hart.pokePc(prevPc);
  hart.setInstructionCount(hart.getInstructionCount() - 1);

  return true;
}


bool
PerfApi::retire(unsigned hartIx, uint64_t time, uint64_t tag, FILE* traceFile)
{
  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_retire %" PRIu64 "\n",
            hartIx, time, tag);

  if (not checkTime("Retire", time))
    return false;

  auto hart = checkHart("Retire", hartIx);
  if (not hart)
    return false;

  auto pacPtr = checkTag("Retire", hartIx, tag);
  if (not pacPtr)
    return false;
  auto& packet = *pacPtr;

  if (tag <= hartLastRetired_.at(hartIx))
    {
      std::cerr << "Hart=" << hartIx << " time=" << time << " tag=" << tag
		<< " Out of order retire\n";
      return false;
    }

  if (packet.retired())
    {
      std::cerr << "Hart=" << hartIx << " time=" << time << " tag=" << tag
		<< " Tag retired more than once\n";
      return false;
    }

  if (packet.instrVa() != hart->peekPc())
    {
      std::cerr << "Hart=" << hartIx << " time=" << time << " tag=" << tag << std::hex
		<< " Wrong pc: 0x" << packet.instrVa() << " expecting "
		<< hart->peekPc() << '\n' << std::dec;
      return false;
    }

  hart->setInstructionCount(tag - 1);
  hart->singleStep(traceFile);

  // Undo renaming of destination registers.
  auto& producers = hartRegProducers_.at(hartIx);
  for (size_t i = 0; i < packet.destValues_.size(); ++i)
    {
      auto gri = packet.destValues_.at(i).first;
      auto& producer = producers.at(gri);
      if (producer and producer->tag() == packet.tag())
        producer = nullptr;
    }

  packet.retired_ = true;

  // Stores erased at drain time.
  if (not packet.decodedInst().isStore())
    {
      auto& packetMap = hartPacketMaps_.at(hartIx);
      packetMap.erase(packet.tag());
    }

  return true;
}


WdRiscv::ExceptionCause
PerfApi::translateInstrAddr(unsigned hartIx, uint64_t iva, uint64_t& ipa)
{
  auto hart = checkHart("Translate-instr-addr", hartIx);
  bool r = false, w = false, x = true;
  auto pm = hart->privilegeMode();
  return  hart->transAddrNoUpdate(iva, pm, r, w, x, ipa);
}


WdRiscv::ExceptionCause
PerfApi::translateLoadAddr(unsigned hartIx, uint64_t iva, uint64_t& ipa)
{
  auto hart = checkHart("translate-load-addr", hartIx);
  bool r = true, w = false, x = false;
  auto pm = hart->privilegeMode();
  return  hart->transAddrNoUpdate(iva, pm, r, w, x, ipa);
}


WdRiscv::ExceptionCause
PerfApi::translateStoreAddr(unsigned hartIx, uint64_t iva, uint64_t& ipa)
{
  auto hart = checkHart("translate-store-addr", hartIx);
  bool r = false, w = true, x = false;
  auto pm = hart->privilegeMode();
  return  hart->transAddrNoUpdate(iva, pm, r, w, x, ipa);
}


bool
PerfApi::drainStore(unsigned hartIx, uint64_t time, uint64_t tag)
{
  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_drain_store %" PRIu64 "\n",
            hartIx, time, tag);

  if (not checkTime("Drain-store", time))
    return false;

  auto hart = checkHart("Drain-store", hartIx);
  auto pacPtr = checkTag("Drain-store", hartIx, tag);

  if (not hart or not pacPtr or not pacPtr->retired())
    {
      assert(0);
      return false;
    }

  auto& packet = *pacPtr;

  if (packet.drained())
    {
      std::cerr << "Hart=" << hartIx << " time=" << time << " tag=" << tag
		<< " Instruction drained more than once\n";
      return false;
    }

  if (not packet.di_.isStore())
    {
      std::cerr << "Hart=" << hartIx << " time=" << time << " tag=" << tag
		<< " Draining a non-store instruction\n";
      return false;
    }

  uint64_t value = packet.opVal_.at(0);
  uint64_t addr = packet.dpa_;    // FIX TODO : Handle page crossing store.

  switch (packet.dsize_)
    {
    case 1:
      if (not hart->pokeMemory(addr, uint8_t(value), true))
	assert(0);
      break;

    case 2:
      if (not hart->pokeMemory(addr, uint16_t(value), true))
	assert(0);
      break;

    case 4:
      if (not hart->pokeMemory(addr, uint32_t(value), true))
	assert(0);
      break;

    case 8:
      if (not hart->pokeMemory(addr, value, true))
	assert(0);
      break;

    default:
      assert(0);
    }

  packet.drained_ = true;

  auto& packetMap = hartPacketMaps_.at(hartIx);
  packetMap.erase(packet.tag());

  auto& storeMap = hartStoreMaps_.at(hartIx);
  storeMap.erase(packet.tag());

  return true;
}


bool
PerfApi::getLoadData(unsigned hartIx, uint64_t tag, uint64_t vaddr, unsigned size, uint64_t& data)
{
  auto hart = checkHart("Get-load-data", hartIx);
  auto packet = checkTag("Get-load-Data", hartIx, tag);

  if (not hart or not packet or not packet->di_.isLoad() or packet->trapped())
    {
      assert(0);
      return false;
    }

  if (packet->executed())
    {
      assert(size == packet->dataSize());
      data = packet->destValues_.at(0).second;
      return true;
    }

  auto& storeMap =  hartStoreMaps_.at(hartIx);

  unsigned mask = (1 << size) - 1;  // One bit ber byte of load data.
  unsigned forwarded = 0;            // One bit per forwarded byte.

  for (auto& kv : storeMap)
    {
      auto stTag = kv.first;
      auto& stPac = kv.second;

      if (stTag > tag)
	break;

      if (not stPac->executed())
	continue;

      uint64_t stAddr = stPac->dataVa();
      unsigned stSize = stPac->dataSize();
      if (stAddr + stSize < vaddr or vaddr + size < stSize)
	continue;  // No overlap.

      uint64_t stData = stPac->opVal_.at(0);

      for (unsigned i = 0; i < size; ++i)
	{
	  unsigned byteMask = 1 << i;
	  uint64_t byteAddr = vaddr + i;
	  if (byteAddr >= stAddr and byteAddr < stAddr + stSize)
	    {
	      uint64_t stByte = (stData >> (byteAddr - stAddr)*8) & 0xff;
	      data |= stByte << (i*8);
	      forwarded |= byteMask;
	    }
	}
    }

  if (forwarded == mask)
    return true;

  for (unsigned i = 0; i < size; ++i)
    if (not (forwarded & (1 << i)))
      {
	uint8_t byte = 0;
	if (not hart->peekMemory(vaddr + i, byte, true))
	  assert(0);
	data |= uint64_t(byte) << (i*8);
      }

  return true;
}


bool
PerfApi::flush(unsigned hartIx, uint64_t time, uint64_t tag)
{
  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_flush %" PRIu64 "\n",
            hartIx, time, tag);

  if (not checkTime("Flush", time))
    return false;

  if (not checkHart("Flush", hartIx))
    return false;

  auto& packetMap = hartPacketMaps_.at(hartIx);

  // Flush tag and all older packets. Flush in reverse order to undo register renaming.
  for (auto iter = packetMap.rbegin(); iter != packetMap.rend(); ++iter)
    {
      auto pacPtr = iter->second;
      if (pacPtr->tag_ < tag)
	break;

      if (not pacPtr or pacPtr->retired())
        {
          assert(0);
          return false;
        }

      auto& packet = *pacPtr;
      auto& di = packet.di_;
      auto& producers = hartRegProducers_.at(hartIx);
      for (size_t i = 0; i < di.operandCount(); ++i)
        {
          if (di.ithOperandMode(i) == WdRiscv::OperandMode::Write or
              di.ithOperandMode(i) == WdRiscv::OperandMode::ReadWrite)
            {
              unsigned regNum = di.ithOperand(i);
              unsigned gri = globalRegIx(di.ithOperandType(i), regNum);
              producers.at(gri) = packet.opProducers_.at(i);
            }
        }

      packetMap.erase(pacPtr->tag_);
    }

  return true;
}


bool
PerfApi::shouldFlush(unsigned hartIx, uint64_t time, uint64_t tag, bool& flush,
		     uint64_t& addr)
{
  flush = false;
  addr = 0;

  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_should_flush %" PRIu64 "\n",
            hartIx, time, tag);

  if (not checkTime("Flush", time))
    return false;

  if (not checkHart("Flush", hartIx))
    return false;

  auto pacPtr = checkTag("Retire", hartIx, tag);
  if (not pacPtr)
    return false;
  auto& packet = *pacPtr;

  if (packet.shouldFlush())
    {
      flush = true;
      addr = packet.iva_;
    }
  else
    {
      // If on the wrong path after a branch, then we wshould flush.
      auto& packetMap = hartPacketMaps_.at(hartIx);
      for (auto iter = packetMap.find(tag); iter != packetMap.end(); --iter)
	if (iter->second->mispredicted_)
	  {
	    flush = true;
	    addr = iter->second->nextIva_;
	  }
    }

  return true;
}
