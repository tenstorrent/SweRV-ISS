// Copyright 2022 Tenstorrent Corporation.
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
#include "System.hpp"
#include "Hart.hpp"
#include "DecodedInst.hpp"

namespace TT_WPA         // Tenstorrent Whisper Performance Model API
{

  using System64 = WdRiscv::System<uint64_t>;
  using Hart64 = WdRiscv::Hart<uint64_t>;
  using ExceptionCause = WdRiscv::ExceptionCause;

  /// Instruction packet.
  class InstrPac
  {
  public:

    friend class PerfApi;

    InstrPac(uint64_t tag, uint64_t iva, uint64_t ipa)
      : tag_(tag), iva_(iva), ipa_(ipa)
    { }

    /// This must be called by the performance model after a call to execute. If it
    /// returns true, then the performance model subsequent call must be a flush to cause
    /// whisper to flush.
    bool shouldFlush() const
    {
      if (not executed_ or not predicted_ or not isBranch())
	return false;
      bool taken = nextIva_ != iva_ + di_.instSize();
      return prTaken_ != taken or prTarget_ == nextIva_;
    }

    /// Return the instruction virtual address.
    uint64_t instrVa() const
    { return iva_; }

    /// Return the instruction physical address.
    uint64_t instrPa() const
    { return ipa_; }

    /// Return the data virtual address of a load/store instruction. Return 0 if
    /// instruction is not load/store.
    uint64_t dataVa() const
    { return dva_; }
    
    /// Return the data physical address of a load/store instruction. Return 0 if
    /// instruction is not load/store.
    uint64_t dataPa() const
    { return dpa_; }

    /// Return the data size of a load/store instruction. Return 0 if instruction
    /// is not load/store.
    uint64_t dataSize() const
    { return dsize_; }

    /// Return true if this is a branch instruction. Will return false if instruction is
    /// not decoded.
    uint64_t isBranch() const
    { return di_.isBranch(); }

    /// Record the branch prediction made by the performance model. Return false if
    /// instruction is not a branch or is not decoded.
    bool predictBranch(bool taken, uint64_t target)
    {
      if (not decoded_ or not isBranch())
	return false;
      prTaken_ = taken;
      prTarget_ = target;
      return true;
    }

    uint64_t nextPc() const
    { return nextIva_; }

    bool trapped() const
    { return trap_; }

    bool predicted() const
    { return predicted_; }

    bool decoded() const
    { return decoded_; }

    bool executed() const
    { return executed_; }

    bool retired() const
    { return executed_; }

    uint64_t tag() const
    { return tag_; }

  private:

    uint64_t tag_ = 0;
    uint64_t iva_ = 0;        // instruction virtual address
    uint64_t ipa_ = 0;        // instruction physical address
    uint64_t nextIva_ = 0;    // virtual address of subsequent instruction in prog order

    uint64_t dva_ = 0;        // ld/st data virtual address
    uint64_t dpa_ = 0;        // ld/st data physical address
    uint64_t dsize_ = 0;      // ld/st data size (total)

    WdRiscv::DecodedInst di_;          // decoded instruction.

    uint64_t execTime_ = 0;   // Execution time
    uint64_t prTarget_ = 0;   // Predicted branch target

    uint64_t opVal_[3];       // Operand values (count and types are in di)

    // Following applicable if instruction is a branch
    bool predicted_  : 1 = false;  // true if predicted to be a branch
    bool prTaken_    : 1 = false;  // true if predicted branch/jump is taken

    bool fetched_    : 1 = false;
    bool decoded_    : 1 = false;  // true if instruction decoded
    bool executed_   : 1 = false;  // true if instruction executed
    bool retired_    : 1 = false;  // true if instruction retired (committed)
    bool trap_       : 1 = false;  // true if instruction trapped
  };


  class PerfApi
  {
  public:

    PerfApi(System64& system)
      : system_(system)
    {
      unsigned n = system.hartCount();
      hartPacketMaps_.resize(n);
      hartLastRetired_.resize(n);
    }

    /// Called by the performance model to affect a fetch in whisper. The
    /// instruction tag must be monotonically increasing for a particular
    /// hart. Time must be monotonically non-decreasing. If a trap is taken, this
    /// will set trap to true and, cause to the trap cause, and trapPc to the trap
    /// handler pc; otherwise, trap will be false, and cause and trapPc will be
    /// left unmodified. If a trap happens, whisper will expect the trapPc as the
    /// the vpc on the subsequent call to fetch.
    /// Return true on success and false if given tag has already been fetched.
    bool fetch(unsigned hartIx, uint64_t time, uint64_t tag, uint64_t vpc,
	       bool& trap, ExceptionCause& cause, uint64_t& trapPC);

    /// Called by the performance model to affect a decode in whisper. Whisper
    /// will return false if the instruction has not been fetched; otherwise, it
    /// will return true after decoding the instruction, updating the di field of
    /// the corresponding packet, and marking the packed as decoded.
    bool decode(unsigned hartIx, uint64_t time, uint64_t tag, uint32_t opcode);

    /// Optionally called by performance model after decode to inform Whisper of branch
    /// prediction. Returns true on success and false on error (tag was never decoded).
    bool branchPredict(unsigned hart, uint64_t tag, bool prTaken, uint64_t prTarget)
    {
      auto packet = getInstructionPacket(hart, tag);
      if (not packet)
	return false;
      return packet->predictBranch(prTaken, prTarget);
    }

    /// Execute instruction with the given tag in the given hart. Return true on
    /// success marking the instruction as executed and collect its operand
    /// values.  Return false if the given tag has not yet been fetched or
    /// if it has been flushed.
    bool execute(unsigned hart, uint64_t time, uint64_t tag);

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

    /// Return a pointer of the hart having the given index or null if no such hart.
    std::shared_ptr<Hart64> getHart(unsigned hartIx)
    {
      return system_.ithHart(hartIx);
    }

  protected:

    void insertPacket(unsigned hartIx, uint64_t tag, std::shared_ptr<InstrPac> ptr)
    {
      auto& packetMap = hartPacketMaps_.at(hartIx);
      packetMap[tag] = ptr;
    }

    std::shared_ptr<Hart64> checkHart(const char* caller, unsigned hartIx);

    std::shared_ptr<InstrPac> checkTag(const char* caller, unsigned HartIx, uint64_t tag);

    bool checkTime(const char* caller, uint64_t time);

  private:

    typedef std::map<uint64_t, std::shared_ptr<InstrPac>> PacketMap;

    System64& system_;
    std::shared_ptr<InstrPac> prevFetch_;

    /// Map a hart index to the corresponding instruction packet map.
    std::vector<PacketMap> hartPacketMaps_;

    /// Map a hart index to the tag of the last retired instruction.
    std::vector<uint64_t> hartLastRetired_;

    uint64_t time_ = 0;
  };

}
