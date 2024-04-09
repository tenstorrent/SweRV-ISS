#include "PerfModel.hpp"

using namespace TT_PERF;


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
  auto hart = checkHart("Fetch", hartIx);
  if (not hart)
    return false;

  if (not checkTime("Fetch", time))
    return false;

  auto packet = getInstructionPacket(hartIx, tag);
  if (packet)
    return false;   // Tag already fetched.

  auto prev = this->prevFetch_;
  if (prev and prev->executed())
    {
      if (prev->nextPc() != vpc and not prev->trapped() and not prev->predicted())
	prev->predictBranch(true /*taken*/, vpc);
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

  return true;
}


bool
PerfApi::execute(unsigned hartIx, uint64_t time, uint64_t tag)
{
  if (not checkTime("Execute", time))
    return false;

  auto hart = checkHart("Execute", hartIx);
  if (not hart)
    return false;

  auto packet = checkTag("execute", hartIx, tag);
  if (not packet)
    return false;

  auto& di = packet->decodedInst();

  if (packet->executed())
    {
      // Instruction is being re-executed. Must be load/store. Every instruction that
      // depends on it must be re-executed.
      if (not di.isLoad() and not di.isStore())
	assert(0);
      auto& packetMap = hartPacketMaps_.at(hartIx);
      auto iter = packetMap.find(packet->tag());
      assert(iter != packetMap.end());
      for ( ; iter != packetMap.end(); ++iter)
	{
	  auto succ = iter->second;   // Successor packet
	  if (succ->dependsOn(*packet))
	    succ->executed_ = false;
	}
    }

  // Collect register operand values. Remeber the in-flight instructions producing
  // these values.
  auto& producers = hartRegProducers_.at(hartIx);
  assert(di.operandCount() <= packet->opVal_.size());
  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      if (di.ithOperandMode(i) == WdRiscv::OperandMode::None)
	continue;
      unsigned regNum = di.ithOperand(i);
      unsigned gri = globalRegIx(di.ithOperandType(i), regNum);
      auto& producer = producers.at(gri);
      uint64_t value = 0;
      if (producer)
	value = getDestValue(producer, gri);
      else if (not peekRegister(*hart, di.ithOperandType(i), regNum, value))
	assert(0);
      packet->opVal_.at(i) = value;
    }

  // Execute the instruction: Poke source register values, execute, recover destination
  // values.
  if (not execute(hartIx, packet))
    assert(0);

  packet->executed_ = true;
  packet->execTime_ = time;

  return true;
}

  
bool
PerfApi::execute(unsigned hartIx, std::shared_ptr<InstrPac>& pacPtr)
{
  assert(pacPtr);
  auto& packet = *pacPtr;
  assert(packet.decoded());

  auto hartPtr = getHart(hartIx);
  assert(hartPtr);
  auto& hart = *hartPtr;

  using OM = WdRiscv::OperandMode;
  using OT = WdRiscv::OperandType;

  uint64_t prevPc = hart.peekPc();
  hart.pokePc(packet.instrVa());

  std::array<uint64_t, 3> prevVal;  // Previous operand values

  // Poke packet operands into hart saving previous values of packed registers.
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
	  if (not hart.peekIntReg(operand, prevVal.at(i)))
	    assert(0);
	  if (not hart.pokeIntReg(operand, val))
	    assert(0);
	  break;
	case OT::FpReg:
	  if (not hart.peekFpReg(operand, prevVal.at(i)))
	    assert(0);
	  if (not hart.pokeFpReg(operand, val))
	    assert(0);
	  break;
	case OT::CsReg:
	  if (not hart.peekCsr(WdRiscv::CsrNumber(operand), prevVal.at(i)))
	    assert(0);
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

  // Mark this insruction as the producer of each of its destination registers (at most 2:
  // one register and one csr). Record the values of the dstination register.
  auto& di = packet.decodedInst();
  unsigned destIx = 0;
  auto& producers = hartRegProducers_.at(hart.sysHartIndex());
  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      auto mode = di.ithOperandMode(i);
      if (mode == OM::Write or mode == OM::ReadWrite)
	{
	  unsigned regNum = di.ithOperand(i);
	  unsigned gri = globalRegIx(di.ithOperandType(i), regNum);
	  auto prevProducer = producers.at(gri);
	  producers.at(gri) = pacPtr;
	  uint64_t destVal = 0;
	  if (not peekRegister(hart, di.ithOperandType(i), regNum, destVal))
	    assert(0);
	  packet.destValues_.at(destIx) = InstrPac::DestValue(gri, destVal);
	  packet.prevDestWriter_.at(destIx)= prevProducer;
	  destIx++;
	}
    }

  // Restore changed memory.
  uint64_t maddr = 0, mval = 0;
  unsigned stSize = hart.lastStore(maddr, mval);
  bool usePma = true;
  switch (stSize)
    {
    case 0:
      break;
    case 1:
      if (not hart.pokeMemory(maddr, uint8_t(mval), usePma))
	assert(0);
      break;
    case 2:
      if (not hart.pokeMemory(maddr, uint16_t(mval), usePma))
	assert(0);
      break;
    case 4:
      if (not hart.pokeMemory(maddr, uint32_t(mval), usePma))
	assert(0);
      break;
    case 8:
      if (not hart.pokeMemory(maddr, uint64_t(mval), usePma))
	assert(0);
      break;
    default:
      assert(0);
    }

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
  if (not checkTime("Retire", time))
    return false;

  auto hart = checkHart("Retire", hartIx);
  if (not hart)
    return false;

  auto packet = checkTag("Retire", hartIx, tag);
  if (not packet)
    return false;

  if (tag <= hartLastRetired_.at(hartIx))
    {
      std::cerr << "Hart=" << hartIx << " time=" << time << " tag=" << tag
		<< " Out of order retire\n";
      return false;
    }

  if (packet->retired())
    {
      std::cerr << "Hart=" << hartIx << " time=" << time << " tag=" << tag
		<< " Tag retired more than once\n";
      return false;
    }

  if (packet->instrVa() != hart->peekPc())
    {
      std::cerr << "Hart=" << hartIx << " time=" << time << " tag=" << tag << std::hex
		<< " Wrong pc: 0x" << packet->instrVa() << " expecting "
		<< hart->peekPc() << '\n' << std::dec;
      return false;
    }

  hart->setInstructionCount(tag - 1);
  hart->singleStep(traceFile);

  if (packet->di_.isLoad())
    packet->dsize_ = packet->di_.loadSize();
  else if (packet->di_.isStore())
    packet->dsize_ = packet->di_.storeSize();

  // Undo renaming of destination registers.
  auto& producers = hartRegProducers_.at(hartIx);
  for (size_t i = 0; i < packet->destValues_.size(); ++i)
    {
      auto& prev = packet->prevDestWriter_.at(i);
      auto gri = packet->destValues_.at(i).first;
      if (producers.at(gri).get() == packet.get())
	producers.at(gri) = prev;
    }

  packet->retired_ = true;

  auto& packetMap = hartPacketMaps_.at(hartIx);
  packetMap.erase(packet->tag());

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
  if (not checkTime("Drain-store", time))
    return false;

  auto hart = checkHart("Drain-store", hartIx);
  auto packet = checkTag("Drain-store", hartIx, tag);

  if (not hart or not packet or not packet->retired())
    {
      assert(0);
      return false;
    }

  if (packet->drained())
    {
      std::cerr << "Hart=" << hartIx << " time=" << time << " tag=" << tag
		<< " Instruction drained more than once\n";
      return false;
    }

  if (not packet->di_.isStore())
    {
      std::cerr << "Hart=" << hartIx << " time=" << time << " tag=" << tag
		<< " Draining a non-store instruction\n";
      return false;
    }

  uint64_t value = packet->opVal_.at(0);
  uint64_t addr = packet->dpa_;    // FIX TODO : Handle page crossing store.

  switch (packet->dsize_)
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

  packet->drained_ = true;
  return true;
}


bool
PerfApi::getLoadData(unsigned hartIx, uint64_t tag, uint64_t& data)
{

  auto hart = checkHart("Get-load-data", hartIx);
  auto packet = checkTag("Get-load-Data", hartIx, tag);

  if (not hart or not packet or not packet->executed() or not packet->di_.isLoad()
      or packet->trapped())
    {
      assert(0);
      return false;
    }

  data = packet->destValues_.at(0).second;
  return true;
}
