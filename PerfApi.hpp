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

#pragma once

#include <iostream>
#include <vector>
#include "System.hpp"
#include "Hart.hpp"
#include "DecodedInst.hpp"

namespace TT_PERF         // Tenstorrent Whisper Performance Model API
{

  using System64 = WdRiscv::System<uint64_t>;
  using Hart64 = WdRiscv::Hart<uint64_t>;
  using ExceptionCause = WdRiscv::ExceptionCause;
  using OperandType = WdRiscv::OperandType;

  /// Strucuture to recover the source/destination operands of an instruction packet.
  struct Operand
  {
    OperandType type = OperandType::IntReg;
    unsigned number = 0;                  // Register number.
    uint64_t value = 0;
    uint64_t prevValue = 0;               // Used for modified registers.
  };


  /// Instruction packet.
  class InstrPac
  {
  public:

    friend class PerfApi;

    /// Constructor: iva/ipa are the instruction virtual/physical addresses.  For
    /// instruction crossing page boundary, ipa2 is the physical address of the other
    /// page. I not crossing page boundary ipa2 is same as ipa.
    InstrPac(uint64_t tag, uint64_t iva, uint64_t ipa, uint64_t ipa2)
      : tag_(tag), iva_(iva), ipa_(ipa), ipa2_(ipa2)
    { }

    ~InstrPac()
    { }

    /// This supports PerfApi::shouldFlush. It is not meant to be called directly.
    bool shouldFlush() const
    { return shouldFlush_; }

    /// Return the instruction virtual address.
    uint64_t instrVa() const
    { return iva_; }

    /// Return the instruction physical address.
    uint64_t instrPa() const
    { return ipa_; }

    /// For non-page crossing fetch return the same value as instrPa. For page-crossing
    /// return the physical address of the other page.
    uint64_t instrPa2() const
    { return ipa2_; }

    /// Return the data virtual address of a load/store instruction. Return 0 if
    /// instruction is not load/store.
    uint64_t dataVa() const
    { return dva_; }

    /// Return the data physical address of a load/store instruction. Return 0 if
    /// instruction is not load/store.
    uint64_t dataPa() const
    { return dpa_; }

    /// Return the size of the instruction (2 or 4 bytes). Instruction must be fetched.
    uint64_t instrSize() const
    { assert(fetched_); return di_.instSize(); }

    /// Return the data size of a load/store instruction. Return 0 if instruction
    /// is not load/store.
    uint64_t dataSize() const
    { return dsize_; }

    /// Return true if this is a branch instruction. Will return false if instruction is
    /// not decoded.
    uint64_t isBranch() const
    { return di_.isBranch(); }

    uint64_t isBranchToRegister() const
    { return di_.isBranchToRegister(); }

    /// Return true if this branch instruction is taken. Will return false if
    /// instruction has not executed or is not a branch.
    bool isTakenBranch() const
    { return taken_; }

    /// Return branch target as determined by decode, even if the branch is not taken.
    /// Return 0 if the instruction is not decoded, not a branch, or is an indirect branch.
    uint64_t branchTargetFromDecode() const;

    /// Record the branch prediction made by the performance model. Return false if
    /// instruction is not a branch or is not decoded.
    bool predictBranch(bool taken, uint64_t target)
    {
      if (not decoded_ or not isBranch())
	return false;
      predicted_ = true;
      prTaken_ = taken;
      prTarget_ = target;
      return true;
    }

    /// Return true if this instruction depends on the given instruction.
    bool dependsOn(const InstrPac& other) const
    {
      assert(decoded_);
      for (unsigned i = 0; i < di_.operandCount(); ++i)
	{
	  using OM = WdRiscv::OperandMode;

	  auto mode = di_.ithOperandMode(i);
	  if (mode == OM::Read or mode == OM::ReadWrite)
	    {
	      auto producer = opProducers_.at(i);
	      if (producer and producer->tag_ == other.tag_)
		return true;
	    }
	}
      return false;
    }

    /// Return the decoded instruction object associated with this packet.
    const WdRiscv::DecodedInst& decodedInst() const
    { return di_; }

    /// Return the PC of the instruction following this instruction in program order.
    /// Only valid if instruction is executed.
    uint64_t nextPc() const
    { assert(executed_); return nextIva_; }

    /// Return true if instruction fetch or execute encountered a trap.
    bool trapped() const
    { return trap_; }

    /// Return true if a branch prediction was made for this instruction.
    bool predicted() const
    { return predicted_; }

    /// Return true if instruction is decoded.
    bool decoded() const
    { return decoded_; }

    /// Return true if instruction is executed.
    bool executed() const
    { return executed_; }

    /// Return true if instruction is retired.
    bool retired() const
    { return retired_; }

    /// Return true if this is a store instruction and the store is drained.
    bool drained() const
    { return drained_; }

    /// Return the tag of this instruction.
    uint64_t tag() const
    { return tag_; }

    /// Return true if this a load instruction.  Packet must be decoded.
    bool isLoad() const
    { return di_.isLoad(); }

    /// Return true if this a load instruction.  Packet must be decoded.
    bool isStore() const
    { return di_.isStore(); }

    /// Return true if this an AMO instruction (does not include lr/sc).  Packet must be
    /// decoded.
    bool isAmo() const
    { return di_.isAmo(); }

    /// Return true if this a store conditional (sc) instruction.  Packet must be decoded.
    bool isSc() const
    { return di_.isSc(); }

    /// Return true if this is a privileged instruction (ebreak/ecall)
    bool isPrivileged() const
    {
      if (di_.instEntry())
        {
          auto instId = di_.instEntry()->instId();
          if (instId == WdRiscv::InstId::ebreak)
            return true;
          else if (instId == WdRiscv::InstId::ecall)
            return true;
        }
      return false;
    }

    /// Fill the given array with the source operands of this instruction (which must be
    /// decoded). Value of each operand will be zero unless the instruction is executed.
    /// Return the number of operands written into the array.
    unsigned getSourceOperands(std::array<Operand, 3>& ops);

    /// Fill the given array with the destination operands of this instruction (which must
    /// be decoded). Value of each operand will be zero unless the instruction is
    /// executed.  Return the number of operands written into the array.
    unsigned getDestOperands(std::array<Operand, 2>& ops);

  private:

    uint64_t tag_ = 0;
    uint64_t iva_ = 0;        // Instruction virtual address (from performance model)
    uint64_t ipa_ = 0;        // Instruction physical address
    uint64_t ipa2_ = 0;       // Instruction physical address on other page
    uint64_t nextIva_ = 0;    // Virtual address of subsequent instruction in prog order

    uint64_t dva_ = 0;        // ld/st data virtual address
    uint64_t dpa_ = 0;        // ld/st data physical address
    uint64_t dsize_ = 0;      // ld/st data size (total)
    uint64_t storeData_ = 0;  // Store data.
    uint64_t flushVa_ = 0;    // Redirect PC for packets that should be flushed.

    WdRiscv::DecodedInst di_; // decoded instruction.

    uint64_t execTime_ = 0;   // Execution time
    uint64_t prTarget_ = 0;   // Predicted branch target

    std::array<uint64_t, 4> opVal_;       // Operand values (count and types are in di_)

    // Entry i is the in-flight producer of the ith operand.
    std::array<std::shared_ptr<InstrPac>, 4> opProducers_;

    // Global register index of a destination register and its corresponding value.
    typedef std::pair<unsigned, uint64_t> DestValue;
    std::array<DestValue, 2> destValues_;

    uint32_t opcode_ = 0;

    // Following applicable if instruction is a branch
    bool predicted_    : 1 = false;  // true if predicted to be a branch
    bool prTaken_      : 1 = false;  // true if predicted branch/jump is taken
    bool taken_        : 1 = false;  // true if branch/jump is actually taken
    bool mispredicted_ : 1 = false;
    bool shouldFlush_  : 1 = false;

    bool fetched_      : 1 = false;  // true if instruction fetched
    bool decoded_      : 1 = false;  // true if instruction decoded
    bool executed_     : 1 = false;  // true if instruction executed
    bool retired_      : 1 = false;  // true if instruction retired (committed)
    bool drained_      : 1 = false;  // true if a store that has been drained
    bool trap_         : 1 = false;  // true if instruction trapped
  };


  class PerfApi
  {
  public:

    PerfApi(System64& system);

    /// Called by the performance model to affect a fetch in whisper. The
    /// instruction tag must be monotonically increasing for a particular
    /// hart. Time must be monotonically non-decreasing. If a trap is taken, this
    /// will set trap to true and, cause to the trap cause, and trapPc to the trap
    /// handler pc; otherwise, trap will be false, and cause and trapPc will be
    /// left unmodified. If a trap happens, whisper will expect the trapPc as the
    /// the vpc on the subsequent call to fetch.
    /// Return true on success and false if given tag has already been fetched.
    /// The tag is a sequence number. It must be monotonically increasing and
    /// must not be zero. Tags of flushed intructions may be reusede.
    bool fetch(unsigned hartIx, uint64_t time, uint64_t tag, uint64_t vpc,
	       bool& trap, ExceptionCause& cause, uint64_t& trapPC);

    /// Called by the performance model to affect a decode in whisper. Whisper
    /// will return false if the instruction has not been fetched; otherwise, it
    /// will return true after decoding the instruction, updating the di field of
    /// the corresponding packet, and marking the packed as decoded.
    bool decode(unsigned hartIx, uint64_t time, uint64_t tag);

    /// Optionally called by performance model after decode to inform Whisper of branch
    /// prediction. Returns true on success and false on error (tag was never decoded).
    bool predictBranch(unsigned hart, uint64_t tag, bool prTaken, uint64_t prTarget)
    {
      auto packet = getInstructionPacket(hart, tag);
      if (not packet)
	return false;
      return packet->predictBranch(prTaken, prTarget);
    }

    /// Execute instruction with the given tag in the given hart. Return true on success
    /// marking the instruction as executed and collect its operand values. Return false
    /// if the given tag has not yet been fetched or if it has been flushed.
    bool execute(unsigned hart, uint64_t time, uint64_t tag);

    /// Helper to above execute: Execute packet instruction without changing hart state.
    /// Poke packet source register values into hart, execute, collect destination
    /// values. Restore hart state.
    bool execute(unsigned hartIx, InstrPac& packet);

    /// Retire given instruction at the given hart. Commit all related state
    /// changes. SC/AMO instructions are executed at this stage and write memory
    /// without going through the store/merge buffer. Return true on success and
    /// false on failure (instruction was not executed or was flushed).
    bool retire(unsigned hart, uint64_t time, uint64_t tag);

    /// Return a pointer to the instruction packet with the given tag in the given
    /// hart. Return a null pointer if the given tag has not yet been fetched.
    std::shared_ptr<InstrPac> getInstructionPacket(unsigned hartIx, uint64_t tag)
    {
      auto& packetMap = hartPacketMaps_.at(hartIx);
      auto iter = packetMap.find(tag);
      if (iter != packetMap.end())
	return iter->second;
      return nullptr;
    }

    /// Flush an instruction and all older instructions at the given hart. Restore
    /// dependency chain (register renaming) to the way it was before the flushed
    /// instructions were decoded.
    bool flush(unsigned hartIx, uint64_t time, uint64_t tag);

    /// Called by performance model to determine whether or not it should redirect
    /// fetch. Return true on success and false on failure. If successful set flush to
    /// true if flush is required and false otherwise. If flush is required, the new fetch
    /// address will be in addr. Note: This is not being used, we should deprecate it.
    bool shouldFlush(unsigned hartIx, uint64_t time, uint64_t tag, bool& flush,
		     uint64_t& addr);

    /// Translate an instruction virtual address into a physical address. Return
    /// ExceptionCause::NONE on success and actual cause on failure.
    WdRiscv::ExceptionCause translateInstrAddr(unsigned hart, uint64_t iva, uint64_t& ipa);

    /// Translate a load data virtual address into a physical address.  Return
    /// ExceptionCause::NONE on success and actual cause on failure.
    WdRiscv::ExceptionCause translateLoadAddr(unsigned hart, uint64_t va, uint64_t& pa);

    /// Translate a store data virtual address into a physical address.  Return
    /// ExceptionCause::NONE on success and actual cause on failure.
    WdRiscv::ExceptionCause translateStoreAddr(unsigned hart, uint64_t va, uint64_t& pa);

    /// Called by performance model to request that whisper updates its own memory with
    /// the data of a store instruction. Return true on success and false on error. It is
    /// an error for a store to be drained before it is retired.
    bool drainStore(unsigned hart, uint64_t time, uint64_t tag);

    /// Set data to the load value of the given instruction tag and return true on
    /// success. Return false on failure leaving data unmodified: tag is not valid or
    /// corresponding instruction is not executed or is not a load.
    bool getLoadData(unsigned hart, uint64_t tag, uint64_t vaddr, unsigned size, uint64_t& data);

    /// Set the data value of a store/amo instruction to be commited to memory
    /// at drain/retire time.
    bool setStoreData(unsigned hart, uint64_t tag, uint64_t value);

    /// Return a pointer of the hart having the given index or null if no such hart.
    std::shared_ptr<Hart64> getHart(unsigned hartIx)
    { return system_.ithHart(hartIx); }

    /// Return number of harts int the system.
    unsigned hartCount() const
    { return system_.hartCount(); }

    /// Enable command log: Log API calls for replay.
    void enableCommandLog(FILE* log)
    { commandLog_ = log; }

    /// Enable instruction tracing to the log file(s).
    void enableTraceLog(std::vector<FILE*>& files)
    { traceFiles_ = files; }

  protected:

    bool commitMemoryWrite(Hart64& hart, unsigned addr, unsigned size, uint64_t value);

    void insertPacket(unsigned hartIx, uint64_t tag, std::shared_ptr<InstrPac> ptr)
    {
      auto& packetMap = hartPacketMaps_.at(hartIx);
      if (not packetMap.empty() and packetMap.rbegin()->first >= tag)
	assert(0 and "Inserted packet with tag newer than oldest tag.");
      packetMap[tag] = ptr;
    }

    std::shared_ptr<Hart64> checkHart(const char* caller, unsigned hartIx);

    std::shared_ptr<InstrPac> checkTag(const char* caller, unsigned HartIx, uint64_t tag);

    bool checkTime(const char* caller, uint64_t time);

    /// Return the global register index for the local (within regiser file) inxex of the
    /// given type (INT, FP, CSR, ...)  and the given relative register number.
    unsigned globalRegIx(WdRiscv::OperandType type, unsigned regNum)
    {
      using OT = WdRiscv::OperandType;
      switch(type)
	{
	case OT::IntReg: return regNum + intRegOffset_;
	case OT::FpReg:  return regNum + fpRegOffset_;
	case OT::CsReg:  return regNum + csRegOffset_;
	case OT::VecReg:
	case OT::Imm:
	case OT::None:   assert(0); return ~unsigned(0);
	}
      assert(0);
      return ~unsigned(0);
    }

    bool peekRegister(Hart64& hart, WdRiscv::OperandType type, unsigned regNum,
		      uint64_t& value)
    {
      using OT = WdRiscv::OperandType;
      switch(type)
	{
	case OT::IntReg: return hart.peekIntReg(regNum, value);
	case OT::FpReg:  return hart.peekFpReg(regNum, value);
	case OT::CsReg:  return hart.peekCsr(WdRiscv::CsrNumber(regNum), value);
	case OT::VecReg:
	case OT::Imm:
	case OT::None:   assert(0); return ~unsigned(0);
	}

      return false;
    }

    /// Get from the producing packet, the value of the register with the given
    /// global register index.
    uint64_t getDestValue(const InstrPac& producer, unsigned gri) const
    {
      assert(producer.executed());
      for (auto& p : producer.destValues_)
	if (p.first == gri)
	  return p.second;
      assert(0);
      return 0;
    }

  private:

    /// Map an instruction tag to corresponding packet.
    typedef std::map<uint64_t, std::shared_ptr<InstrPac>> PacketMap;

    /// Map a global register index to the in-flight instruction producing that
    /// register. This is register renaming.
    typedef std::vector<std::shared_ptr<InstrPac>> RegProducers;

    System64& system_;
    std::shared_ptr<InstrPac> prevFetch_;

    /// Per-hart map of in flight instruction packets.
    std::vector<PacketMap> hartPacketMaps_;

    /// Per-hart map of in flight executed store packets.
    std::vector<PacketMap> hartStoreMaps_;

    /// Per-hart index tag the last retired instruction.
    std::vector<uint64_t> hartLastRetired_;

    /// Per-hart register renaming table (indexed by global register index).
    std::vector<RegProducers> hartRegProducers_;

    uint64_t time_ = 0;

    FILE* commandLog_ = nullptr;
    std::vector<FILE*> traceFiles_;   // One per hart.

    /// Global indexing for all registers.
    const unsigned intRegOffset_ = 0;
    const unsigned fpRegOffset_ = 32;
    const unsigned csRegOffset_ = 64;
    const unsigned totalRegCount_ = csRegOffset_ + 4096; // 4096: max CSR count.

    static constexpr uint64_t haltPc = ~uint64_t(1);  // value assigned to InstPac->nextIva_ when program termination is encountered
  };

}
