#include "PerfModel.hpp"

using namespace TT_WPA;


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
  if (prev)
    {
      if (prev->nextPc() != vpc and not prev->trapped() and not prev->predicted())
	prev->predictBranch(true /*taken*/, vpc);
    }

  uint64_t ppc = 0;  // Physical pc.
  uint32_t opcode = 0;
  uint64_t gpc = 0; // Guest physical pc.
  cause = hart->fetchInstNoTrap(vpc, ppc, gpc, opcode);

  if (cause == ExceptionCause::NONE)
    {
      packet = std::make_shared<InstrPac>(tag, vpc, ppc);
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
      // get trap pc.
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

  if (packet->executed())
    {
      // Instruction is being re-executed. Must be load/store. Every instruction that
      // depends on it must be re-executed.
      auto& di = packet->decodedInst();
      assert(di.isLoad() or di.isStore());
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
  using OM = WdRiscv::OperandMode;
  auto& producers = hartRegProducers_.at(hartIx);
  WdRiscv::DecodedInst& di = packet->di_;
  assert(di.operandCount() < packet->opVal_.size());
  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      auto mode = di.ithOperandMode(i);
      if (mode == OM::Read or mode == OM::ReadWrite)
	{
	  unsigned regNum = di.ithOperand(i);
	  unsigned gri = globalRegIx(di.ithOperandType(i), regNum);
	  auto& producer = producers.at(gri);
	  uint64_t value = 0;
	  if (producer)
	    value = getDestValue(producer, gri);
	  else if (not peekRegister(hart, di.ithOperandType(i), regNum, value))
	    assert(0);
	  packet->opVal_.at(i) = value;
	}
    }

  // Execute the instruction: Poke register values, execute, restore poked registers
  // and restination registers.
  assert(0 && "implement execute");
  std::array<uint64_t, 3> opValues;   // Operand values of executed instruction.

  // Mark this insruction as the producer of each of its destination registers (at most 2:
  // one register and one csr). Record the values of the dstination register.
  unsigned destIx = 0;
  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      auto mode = di.ithOperandMode(i);
      if (mode == OM::Write or mode == OM::ReadWrite)
	{
	  unsigned gri = globalRegIx(di.ithOperandType(i), di.ithOperand(i));
	  auto prevProducer = producers.at(gri);
	  producers.at(gri) = packet;
	  packet->destValues_.at(destIx) = InstrPac::DestValue(gri, opValues.at(i));
	  packet->prevDestWriter_.at(destIx)= prevProducer;
	  destIx++;
	}
    }

  packet->executed_ = true;
  return false;
}

  
bool
PerfApi::retire(unsigned hartIx, uint64_t time, uint64_t tag)
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

  assert(0 && "implement retire");

  // Undo renaming of destination registers.
  auto& producers = hartRegProducers_.at(hartIx);
  for (size_t i = 0; i < packet->destValues_.size(); ++i)
    {
      auto& prev = packet->prevDestWriter_.at(i);
      if (prev)
	{
	  auto gri = packet->destValues_.at(i).first;
	  if (producers.at(gri).get() == packet.get())
	    producers.at(gri) = prev;
	}
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
  if (not hart)
    return false;

  auto packet = checkTag("drain-store", hartIx, tag);
  if (not packet)
    return false;

  if (not packet->retired())
    {
      assert(0 && "Draining non-rertired tag");
      return false;
    }

  assert(0 && "Implement retire");

  packet->retired_ = true;
  return true;
}
