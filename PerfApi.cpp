// Copyright 2024 Tenstorrent Corporation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cinttypes>
#include "PerfApi.hpp"

using namespace TT_PERF;


using CSRN = WdRiscv::CsrNumber;


PerfApi::PerfApi(System64& system)
  : system_(system)
{
  unsigned n = system.hartCount();

  traceFiles_.resize(n);

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
  std::cerr << caller << ": Unknown tag (never fetched): " << tag << '\n';
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

  if (tag == 0)
    {
      std::cerr << "Error in PerfApi::fetch: Hart-ix=" << hartIx << "tag=" << tag
		<< " zero tag is reserved.\n";
      assert(0);
      return false;
    }

  auto& packetMap = hartPacketMaps_.at(hartIx);
  if (not packetMap.empty() and packetMap.rbegin()->first >= tag)
    {
      std::cerr << "Error in PerfApi::fetch: Hart-ix=" << hartIx << "tag=" << tag
		<< " tag is not in increasing order.\n";
      assert(0);
      return false;
    }

  auto packet = getInstructionPacket(hartIx, tag);
  if (packet)
    {
      std::cerr << "Error in PerfApi::fetch: Hart-ix=" << hartIx << "tag=" << tag
		<< " tag is already fetched.\n";
      return false;   // Tag already fetched.
    }

  auto prev = this->prevFetch_;
  if (prev and not prev->predicted() and not prev->trapped() and not prev->executed())
    {
      bool sequential = prev->instrVa() + prev->decodedInst().instSize() == vpc;
      if (not prev->decodedInst().isBranch())
	{
	  if (not sequential)
	    prev->predictBranch(true, vpc);
	}
      else
	{
	  if (sequential)
	    prev->predictBranch(false, vpc);
	}
    }

  uint64_t ppc = 0, ppc2 = 0;  // Physical pc.
  uint32_t opcode = 0;
  uint64_t gpc = 0; // Guest physical pc.
  cause = hart->fetchInstNoTrap(vpc, ppc, ppc2, gpc, opcode);

  packet = std::make_shared<InstrPac>(tag, vpc, ppc, ppc2);
  assert(packet);
  packet->fetched_ = true;
  packet->opcode_= opcode;
  insertPacket(hartIx, tag, packet);
  if (not decode(hartIx, time, tag))
    assert(0);
  prevFetch_ = packet;

  trap = cause != ExceptionCause::NONE;

  if (prev and not prev->trapped() and prev->executed() and prev->nextIva_ != vpc)
    {
      packet->shouldFlush_ = true;
      packet->flushVa_ = prev->nextIva_;
    }

  if (trap)
    {
      prevFetch_ = nullptr;
      trapPc = 0;
    }

  return true;
}


bool
PerfApi::decode(unsigned hartIx, uint64_t time, uint64_t tag)
{
  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_decode %" PRIu64 "\n",
            hartIx, time, tag);

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

  hart->decode(packet->instrVa(), packet->instrPa(), packet->opcode_, packet->di_);
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

      auto mode = di.effectiveIthOperandMode(i);
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

  auto hartPtr = checkHart("Execute", hartIx);
  if (not hartPtr)
    return false;

  auto& hart = *hartPtr;

  auto pacPtr = checkTag("execute", hartIx, tag);
  if (not pacPtr)
    return false;

  auto& packet = *pacPtr;
  auto& di = packet.decodedInst();
  if (di.isLr())
    return true;   // LR is executed and retired at PerApi::retire.

  auto& packetMap = hartPacketMaps_.at(hartIx);

  if (packet.executed())
    {
      // Instruction is being re-executed. Must be load/store. Every instruction that
      // depends on it must be re-executed.
      if (not di.isLoad() and not di.isStore())
	assert(0);
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
  bool peekOk = true;
  assert(di.operandCount() <= packet.opVal_.size());
  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      if (di.ithOperandType(i) == WdRiscv::OperandType::Imm)
	{
	  packet.opVal_.at(i) = di.ithOperand(i);
	  continue;
	}
      assert(di.ithOperandMode(i) != WdRiscv::OperandMode::None);
      unsigned regNum = di.ithOperand(i);
      unsigned gri = globalRegIx(di.ithOperandType(i), regNum);
      uint64_t value = 0;

      auto& producer = packet.opProducers_.at(i);
      if (producer)
	{
	  value = getDestValue(*producer, gri);
	  if (not producer->executed())
	    {
	      std::cerr << "Error: PerfApi::execute: Hart-ix=" << hartIx << "tag=" << tag
			<< " depends on tag=" << producer->tag_ << " which is not yet executed.\n";
	      assert(0);
	      return false;
	    }
	}
      else
	peekOk = peekRegister(hart, di.ithOperandType(i), regNum, value) and peekOk;

      packet.opVal_.at(i) = value;
    }

  // Execute the instruction: Poke source register values, execute, recover destination
  // values.
  if (not execute(hartIx, packet))
    assert(0);

  if (not peekOk)
    assert(packet.trap_);

  packet.executed_ = true;
  packet.execTime_ = time;
  if (packet.predicted_)
    packet.mispredicted_ = packet.prTarget_ != packet.nextIva_;

  if (packet.isBranch())
    {
      // Check if the next instruction in program order is at the right PC.
      auto iter = packetMap.find(packet.tag());
      ++iter;
      if (iter != packetMap.end())
	{
	  auto& next = *(iter->second);
	  if (next.iva_ != packet.nextIva_)
	    {
	      next.shouldFlush_ = true;
	      next.flushVa_ = packet.nextIva_;
	    }
	  if (next.executed_)
	    {
	      packet.shouldFlush_ = true;
	      packet.flushVa_ = packet.iva_;
	    }
	}
    }

  return true;
}


bool
PerfApi::execute(unsigned hartIx, InstrPac& packet)
{
  assert(packet.decoded());

  auto hartPtr = getHart(hartIx);
  assert(hartPtr);
  auto& hart = *hartPtr;

  uint64_t prevPc = hart.peekPc();
  uint64_t prevInstrCount = hart.getInstructionCount();

  hart.pokePc(packet.instrVa());
  hart.setInstructionCount(packet.tag_ - 1);

  std::array<uint64_t, 4> prevVal;  // Previous operand values

  // Save hart register values corresponding to packet operands in prevVal.
  bool saveOk = saveHartValues(hart, packet, prevVal);

  // Install packet operand values (some obtained from previous in-flight instructions)
  // into the hart registers.
  bool setOk = setHartValues(hart, packet);

  auto& di = packet.decodedInst();

  unsigned imsicId = 0, imsicGuest = 0;
  if (di.isCsr())
    saveImsicTopei(hart, CSRN(di.ithOperand(2)), imsicId, imsicGuest);

  // Execute
  skipIoLoad_ = true;   // Load from IO space takes effect at retire.
  hart.singleStep();
  skipIoLoad_ = false;

  bool trap = hart.lastInstructionTrapped();
  packet.trap_ = packet.trap_ or trap;

  // If save fails or set fails, there must be a trap.
  if (not saveOk or not setOk)
    assert(trap);

  // Record PC of subsequent packet.
  packet.nextIva_ = hart.peekPc();

  if (not trap)
    recordExecutionResults(hart, packet);

  // Undo changes to the hart.

  hart.untickTime();  // Restore timer value.

  // Restore CSRs modified by the instruction or trap. TODO: For vector ld/st we have to
  // restore partially modified vectors.
  if (di.isXRet() and not trap)
    {   // For an MRET/SRET/... Privilege may have been lowered. Restore it before restoring CSRs.
      hart.setVirtualMode(hart.lastVirtMode());
      hart.setPrivilegeMode(hart.lastPrivMode());
    }

  // Restore changed CSR changes due to a trap or to mret/sret.
  if (trap or di.isXRet())
    {
      std::vector<CSRN> csrns;
      hart.lastCsr(csrns);
      for (auto csrn : csrns)
	{
	  uint64_t value = hart.lastCsrValue(csrn);
	  if (not hart.pokeCsr(csrn, value))
	    assert(0);
	}
    }

  if (trap)
    {  // Privilege raised.  Restore it after restoring CSRs.
      hart.setVirtualMode(hart.lastVirtMode());
      hart.setPrivilegeMode(hart.lastPrivMode());
    }

  // Restore hart registers that we changed before single step.
  restoreHartValues(hart, packet, prevVal);

  if (di.isCsr())
    restoreImsicTopei(hart, CSRN(di.ithOperand(2)), imsicId, imsicGuest);

  hart.setTargetProgramFinished(false);
  hart.pokePc(prevPc);
  hart.setInstructionCount(prevInstrCount);

  hart.clearTraceData();

  return true;
}


bool
PerfApi::retire(unsigned hartIx, uint64_t time, uint64_t tag)
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
		<< " Wrong pc at retire: 0x" << packet.instrVa() << " expecting 0x"
		<< hart->peekPc() << '\n' << std::dec;
      return false;
    }

  hart->setInstructionCount(tag - 1);
  hart->singleStep(traceFiles_.at(hartIx));

  // Undo renaming of destination registers.
  auto& producers = hartRegProducers_.at(hartIx);
  auto& di = packet.decodedInst();
  for (size_t i = 0; i < di.operandCount(); ++i)
    {
      using OM = WdRiscv::OperandMode;
      auto mode = di.ithOperandMode(i);
      if (mode == OM::Write or mode == OM::ReadWrite)
	{
	  unsigned regNum = di.ithOperand(i);
	  unsigned gri = globalRegIx(di.ithOperandType(i), regNum);
	  auto& producer = producers.at(gri);
	  if (producer and producer->tag() == packet.tag())
	    producer = nullptr;
	}
    }

  if (di.isLr())
    {
      bool trap = hart->lastInstructionTrapped();
      packet.trap_ = packet.trap_ or trap;

      // Record PC of subsequent packet.
      packet.nextIva_ = hart->peekPc();

      if (not trap)
        recordExecutionResults(*hart, packet);

      packet.executed_ = true;
    }

  packet.retired_ = true;

  if (packet.isAmo() or packet.isSc())
    {
      uint64_t sva = 0, spa1 = 0, spa2 = 0, sval = 0;
      unsigned size = hart->lastStore(sva, spa1, spa2, sval);
      if (size != 0)   // Could be zero for a failed sc
	if (not commitMemoryWrite(*hart, spa1, size, packet.storeData_))
	  assert(0);
      if (packet.isSc())
	hart->cancelLr(WdRiscv::CancelLrCause::SC);
      auto& storeMap = hartStoreMaps_.at(hartIx);
      packet.drained_ = true;
      storeMap.erase(packet.tag());
    }

  // Clear dependency on other packets to expedite release of packet memory.
  for (auto& producer : packet.opProducers_)
    producer = nullptr;

  // Stores erased at drain time.
  if (not packet.isStore())
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
  return  hart->transAddrNoUpdate(iva, pm, hart->virtMode(), r, w, x, ipa);
}


WdRiscv::ExceptionCause
PerfApi::translateLoadAddr(unsigned hartIx, uint64_t iva, uint64_t& ipa)
{
  auto hart = checkHart("translate-load-addr", hartIx);
  bool r = true, w = false, x = false;
  auto pm = hart->privilegeMode();
  return  hart->transAddrNoUpdate(iva, pm, hart->virtMode(), r, w, x, ipa);
}


WdRiscv::ExceptionCause
PerfApi::translateStoreAddr(unsigned hartIx, uint64_t iva, uint64_t& ipa)
{
  auto hart = checkHart("translate-store-addr", hartIx);
  bool r = false, w = true, x = false;
  auto pm = hart->privilegeMode();
  return  hart->transAddrNoUpdate(iva, pm, hart->virtMode(), r, w, x, ipa);
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

  if (not packet.di_.isStore())
    {
      std::cerr << "Hart=" << hartIx << " time=" << time << " tag=" << tag
		<< " Draining a non-store instruction\n";
      return false;
    }

  if (packet.isAmo() or packet.isSc())
    assert(packet.drained());   // AMO/SC drained at retire.
  else
    {
      if (packet.drained())
	{
	  std::cerr << "Hart=" << hartIx << " time=" << time << " tag=" << tag
		    << " Instruction drained more than once\n";
	  assert(0);
	}

      uint64_t value = packet.storeData_;
      uint64_t addr = packet.dpa_;    // FIX TODO : Handle page crossing store.

      if (packet.dsize_ and not commitMemoryWrite(*hart, addr, packet.dsize_, value))
	assert(0);

      packet.drained_ = true;
    }

  // Clear dependency on other packets to expedite release of packet memory.
  for (auto& producer : packet.opProducers_)
    producer = nullptr;

  auto& packetMap = hartPacketMaps_.at(hartIx);
  packetMap.erase(packet.tag());

  auto& storeMap = hartStoreMaps_.at(hartIx);
  storeMap.erase(packet.tag());

  return true;
}


bool
PerfApi::getLoadData(unsigned hartIx, uint64_t tag, uint64_t va, uint64_t pa1,
		     uint64_t pa2, unsigned size, uint64_t& data)
{
  auto hart = checkHart("Get-load-data", hartIx);
  auto packet = checkTag("Get-load-Data", hartIx, tag);

  bool isLoad = packet->di_.isLoad() or packet->di_.isAmo();

  if (not hart or not packet or not isLoad or packet->trapped())
    {
      assert(0);
      return false;
    }

  // If AMO destination register is x0, we lose the loaded value: redo the read for AMOs
  // to avoid that case. AMOs should not have a discrepancy between early read and read at
  // excecute, so redoing the read is ok.
  if (packet->executed() and not packet->isAmo())
    {
      assert(size == packet->dataSize());
      data = packet->destValues_.at(0).second;
      return true;
    }

  data = 0;
  bool isDev = hart->isAclintMtimeAddr(pa1) or hart->isImsicAddr(pa1) or hart->isPciAddr(pa1);
  if (isDev)
    {
#if 0
      // FIX : enable after coordinating with Arch team
      if (skipIoLoad_)
        return true;  // Load from IO space happens at execute.
#endif
      hart->deviceRead(pa1, size, data);
      return true;
    }

  if (uint64_t toHost = 0; hart->getToHostAddress(toHost) && toHost == pa1)
    return true;  // Reading from toHost yields 0.

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
      if (stAddr + stSize < va or va + size < stSize)
	continue;  // No overlap.

      uint64_t stData = stPac->opVal_.at(0);

      for (unsigned i = 0; i < size; ++i)
	{
	  unsigned byteMask = 1 << i;
	  uint64_t byteAddr = va + i;
	  if (byteAddr >= stAddr and byteAddr < stAddr + stSize)
	    {
	      data &= ~(0xffull << (i * 8));
	      uint64_t stByte = (stData >> (byteAddr - stAddr)*8) & 0xff;
	      data |= stByte << (i*8);
	      forwarded |= byteMask;
	    }
	}
    }

  if (forwarded == mask)
    return true;

  unsigned size1 = size;
  if (pa1 != pa2 and pageNum(pa1) != pageNum(pa2))
    size1 = offsetToNextPage(pa1);

  for (unsigned i = 0; i < size; ++i)
    if (not (forwarded & (1 << i)))
      {
	uint8_t byte = 0;
	if (i < size1)
	  {
	    if (not hart->peekMemory(pa1 + i, byte, true))
	      assert(0);
	  }
	else
	  {
	    if (not hart->peekMemory(pa2 + (i-size1), byte, true))
	      assert(0);
	  }
	data |= uint64_t(byte) << (i*8);
      }

  return true;
}


bool
PerfApi::setStoreData(unsigned hartIx, uint64_t tag, uint64_t value)
{
  auto hart = checkHart("Set-store-data", hartIx);
  auto packetPtr = checkTag("Set-store-Data", hartIx, tag);
  if (not packetPtr)
    {
      assert(0);
      return false;
    }

  auto& packet = *packetPtr;
  if (not hart or not (packet.isStore() or packet.isAmo()))
    {
      assert(0);
      return false;
    }

  packet.storeData_ = value;
  return true;
}


bool
PerfApi::commitMemoryWrite(Hart64& hart, uint64_t addr, unsigned size, uint64_t value)
{
  if (hart.isToHostAddr(addr))
    {
      hart.handleStoreToHost(addr, value);
      return true;
    }

  switch (size)
    {
    case 1:  return hart.pokeMemory(addr, uint8_t(value), true);
    case 2:  return hart.pokeMemory(addr, uint16_t(value), true);
    case 4:  return hart.pokeMemory(addr, uint32_t(value), true);
    case 8:  return hart.pokeMemory(addr, value, true);
    default: assert(0);
    }

  return false;
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
  auto& storeMap =  hartStoreMaps_.at(hartIx);
  auto& producers = hartRegProducers_.at(hartIx);

  // Flush tag and all older packets. Flush in reverse order to undo register renaming.
  for (auto iter = packetMap.rbegin(); iter != packetMap.rend(); )
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
      for (size_t i = 0; i < di.operandCount(); ++i)
        {
	  auto mode = di.effectiveIthOperandMode(i);
          if (mode == WdRiscv::OperandMode::Write or
              mode == WdRiscv::OperandMode::ReadWrite)
            {
              unsigned regNum = di.ithOperand(i);
              unsigned gri = globalRegIx(di.ithOperandType(i), regNum);
	      if (gri != 0)
		assert(producers.at(gri)->tag_ == packet.tag_);
              auto prev = packet.opProducers_.at(i);
	      if (prev and prev->retired_)
                prev = nullptr;
	      producers.at(gri) = prev;
            }
        }

      ++iter;
    }

  // delete input tag and all newer instructions
  std::erase_if(packetMap, [&tag](const auto &packet)
  {
    auto const& [_, pacPtr] = packet;
    return pacPtr->tag_ >= tag;
  });

  std::erase_if(storeMap, [&tag](const auto &packet)
  {
    auto const& [_, pacPtr] = packet;
    return pacPtr->tag_ >= tag;
  });

  if (prevFetch_ && prevFetch_->tag_ > tag)
    prevFetch_ = nullptr;

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
      addr = packet.flushVa_;
    }
  else
    {
      // If on the wrong path after a branch, then we wshould flush.
      auto& packetMap = hartPacketMaps_.at(hartIx);
      auto iter = packetMap.find(tag); --iter;
      for ( ; iter != packetMap.end(); --iter)
	{
	  auto& packet = *iter->second;
	  if (packet.mispredicted_)
	    {
	      flush = true;
	      if (packet.di_.isBranch())
		addr = packet.nextIva_;
	      else
		addr = packet.iva_;
	    }
	}
    }

  return true;
}


unsigned
InstrPac::getSourceOperands(std::array<Operand, 3>& ops)
{
  assert(decoded_);
  if (not decoded_)
    return 0;

  unsigned count = 0;

  using OM = WdRiscv::OperandMode;

  for (unsigned i = 0; i < di_.operandCount(); ++i)
    {
      if (di_.ithOperandMode(i) == OM::Read or di_.ithOperandMode(i) == OM::ReadWrite or di_.ithOperandType(i) == OperandType::Imm)
	{
	  auto& op = ops.at(count);
	  op.type = di_.ithOperandType(i);
	  op.number = (di_.ithOperandType(i) == OperandType::Imm) ? 0 : di_.ithOperand(i);
	  op.value = opVal_.at(i);
	  ++count;
	}
    }

  return count;
}


unsigned
InstrPac::getDestOperands(std::array<Operand, 2>& ops)
{
  assert(decoded_);
  if (not decoded_)
    return 0;

  unsigned count = 0;

  using OM = WdRiscv::OperandMode;

  for (unsigned i = 0; i < di_.operandCount(); ++i)
    {
      auto mode = di_.effectiveIthOperandMode(i);
      if (mode == OM::Write or mode == OM::ReadWrite)
	{
	  auto& op = ops.at(count);
	  op.type = di_.ithOperandType(i);
	  op.number = di_.ithOperand(i);
	  op.value = destValues_.at(count).second;
	  op.prevValue = opVal_.at(i);
	  ++count;
	}
    }

  return count;
}


uint64_t
InstrPac::branchTargetFromDecode() const
{
  if (!isBranch()) return 0;

  using WdRiscv::InstId;
  switch (di_.instEntry()->instId())
    {
    case InstId::jal:
    case InstId::c_jal:
    case InstId::c_j:
      return instrVa() + di_.op1As<int64_t>();

    case InstId::beq:
    case InstId::bne:
    case InstId::blt:
    case InstId::bge:
    case InstId::bltu:
    case InstId::bgeu:
    case InstId::c_beqz:
    case InstId::c_bnez:
      return instrVa() + di_.op2As<int64_t>();

    default:
      return 0;
    }
}


bool
PerfApi::saveHartValues(Hart64& hart, const InstrPac& packet,
			std::array<uint64_t, 4>& prevVal)
{
  using OM = WdRiscv::OperandMode;
  using OT = WdRiscv::OperandType;

  auto& di = packet.decodedInst();
  bool ok = true;

  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      auto mode = di.ithOperandMode(i);
      auto type = di.ithOperandType(i);
      uint32_t operand = di.ithOperand(i);
      if (mode == OM::None)
	continue;

      switch (type)
	{
	case OT::IntReg:
	  if (not hart.peekIntReg(operand, prevVal.at(i)))
	    assert(0);
	  break;
	case OT::FpReg:
	  ok = hart.peekFpReg(operand, prevVal.at(i)) and ok;
	  break;
	case OT::CsReg:
	  ok = hart.peekCsr(CSRN(operand), prevVal.at(i)) and ok;
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

  return ok;
}


void
PerfApi::saveImsicTopei(Hart64& hart, CSRN csrn, unsigned& id, unsigned& guest)
{
  id = 0;
  guest = 0;

  auto imsic = hart.imsic();
  if (not imsic)
    return;

  if (csrn == CSRN::MTOPEI)
    {
      id = imsic->machineTopId();
    }
  else if (csrn == CSRN::STOPEI)
    {
      id = imsic->supervisorTopId();
    }
  else if (csrn == CSRN::VSTOPEI)
    {
      uint64_t hs = 0;
      if (hart.peekCsr(CSRN::HSTATUS, hs))
	{
	  WdRiscv::HstatusFields<uint64_t> hsf(hs);
	  unsigned gg = hsf.bits_.VGEIN;
	  if (gg > 0 and gg < imsic->guestCount())
	    {
	      guest = gg;
	      imsic->guestTopId(gg);
	    }
	}
    }
}



void
PerfApi::restoreImsicTopei(Hart64& hart, CSRN csrn, unsigned id, unsigned guest)
{
  auto imsic = hart.imsic();
  if (not imsic)
    return;

  if (id == 0)
    return;

  if (csrn == CSRN::MTOPEI)
    {
      imsic->setMachinePending(id, true);
    }
  else if (csrn == CSRN::STOPEI)
    {
      imsic->setSupervisorPending(id, true);
    }
  else if (csrn == CSRN::VSTOPEI)
    {
      if (guest > 0 and guest < imsic->guestCount())
	imsic->setGuestPending(guest, id, true);
    }
}


void
PerfApi::restoreHartValues(Hart64& hart, const InstrPac& packet,
			   const std::array<uint64_t, 4>& prevVal)
{
  using OM = WdRiscv::OperandMode;
  using OT = WdRiscv::OperandType;

  auto& di = packet.decodedInst();

  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      auto mode = di.ithOperandMode(i);
      auto type = di.ithOperandType(i);
      uint32_t operand = di.ithOperand(i);
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
          {
            auto csrn = CSRN(operand);
            if (not hart.pokeCsr(csrn, prev))
              assert(0);
          }
	  break;

	case OT::VecReg:
	  assert(0);
	  break;

	default:
	  assert(0);
	  break;
	}
    }
}


bool
PerfApi::setHartValues(Hart64& hart, const InstrPac& packet)
{
  using OM = WdRiscv::OperandMode;
  using OT = WdRiscv::OperandType;

  auto& di = packet.decodedInst();

  bool ok = true;

  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      auto mode = di.ithOperandMode(i);
      auto type = di.ithOperandType(i);
      uint32_t operand = di.ithOperand(i);
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
 	  ok = hart.pokeFpReg(operand, val) and ok;
 	  break;
 	case OT::CsReg:
 	  ok = hart.pokeCsr(CSRN(operand), val) and ok;
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

  return ok;
}


void
PerfApi::recordExecutionResults(Hart64& hart, InstrPac& packet)
{
  auto& di = packet.decodedInst();

  auto hartIx = hart.sysHartIndex();

  if (di.isLoad())
    {
      hart.lastLdStAddress(packet.dva_, packet.dpa_);  // FIX TODO : handle page corrsing
      packet.dsize_ = di.loadSize();
    }
  else if (di.isStore() or di.isAmo())
    {
      uint64_t sva = 0, spa1 = 0, spa2 = 0, sval = 0;
      unsigned ssize = hart.lastStore(sva, spa1, spa2, sval);
      if (ssize == 0 and not di.isSc())
	{
	  std::cerr << "Hart=" << hartIx << " tag=" << packet.tag_
		    << " store/AMO with zero size\n";
	  assert(0);
	}

      packet.dva_ = sva;
      packet.dpa_ = spa1;  // FIX TODO : handle page corrsing
      packet.dsize_ = ssize;
      assert(ssize == packet.dsize_);
      if (di.isStore() and not di.isSc())
	{
	  auto& storeMap =  hartStoreMaps_.at(hartIx);
	  storeMap[packet.tag()] = getInstructionPacket(hartIx, packet.tag());
	}
    }

  if (hart.hasTargetProgramFinished())
    packet.nextIva_ = haltPc;

  if (di.isBranch()) packet.taken_ = hart.lastBranchTaken();

  // Record the values of the destination register.
  unsigned destIx = 0;
  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      using OM = WdRiscv::OperandMode;

      auto mode = di.effectiveIthOperandMode(i);
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
}
