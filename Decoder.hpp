// Copyright 2020 Western Digital Corporation or its affiliates.
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

#include <cstdint>
#include "InstId.hpp"
#include "InstEntry.hpp"

namespace WdRiscv
{

  class DecodedInst;

  class Decoder
  {
  public:
    
    /// Constructor.
    Decoder();

    /// Destructor.
    ~Decoder();

    /// Decode given instruction returning a pointer to the
    /// instruction information and filling op0, op1 and op2 with the
    /// corresponding operand specifier values. For example, if inst
    /// is the instruction code for "addi r3, r4, 77", then the
    /// returned value would correspond to addi and op0, op1 and op2
    /// will be set to 3, 4, and 77 respectively. If an instruction
    /// has fewer than 3 operands then only a subset of op0, op1 and
    /// op2 will be set. If inst is not a valid instruction , then we
    /// return a reference to the illegal-instruction info.
    const InstEntry& decode(uint32_t inst, uint32_t& op0, uint32_t& op1,
			    uint32_t& op2, uint32_t& op3) const;

    /// Similar to the preceding decode method but with decoded data
    /// placed in the given DecodedInst object.
    void decode(uint64_t addr, uint64_t physAddr, uint32_t inst,
		DecodedInst& decodedInst) const;

    /// Return the 32-bit instruction corresponding to the given 16-bit
    /// compressed instruction. Return an illegal 32-bit opcode if given
    /// 16-bit code is not a valid compressed instruction.
    uint32_t expandCompressedInst(uint16_t inst) const;

    /// Enable/disabled rv64. Some code points will decode differently if
    /// rv64 is enabled.
    void enableRv64(bool flag)
    { rv64_ = flag; }

    /// Return true if rv64 is enabled.
    bool isRv64() const
    { return rv64_; }

    /// Do not consider lr and sc instructions as load/store events for
    /// performance counter when flag is false. Do consider them when
    /// flag is true.
    void perfCountAtomicLoadStore(bool flag)
    { instTable_.perfCountAtomicLoadStore(flag); }

    /// Do not consider flw,fsw,fld,fsd...c instructions as load/store
    /// events for performance counter when flag is false. Do consider
    /// them when flag is true.
    void perfCountFpLoadStore(bool flag)
    { instTable_.perfCountFpLoadStore(flag); }

    /// Return the instruction table entry associated with the given
    /// instruction id. Return illegal instruction entry id is out of
    /// bounds.
    const InstEntry& getInstructionEntry(InstId id) const
    { return instTable_.getEntry(id); }

    /// Return the instruction table entry associated with the given
    /// instruction id. Return illegal instruction entry id is out of
    /// bounds.
    const InstEntry& getInstructionEntry(std::string_view name) const
    { return instTable_.getEntry(name); }

  protected:

    /// Decode a floating point instruction.
    const InstEntry& decodeFp(uint32_t inst, uint32_t& op0, uint32_t& op1,
			      uint32_t& op2) const;

    /// Decode a vector instruction.
    const InstEntry& decodeVec(uint32_t inst, uint32_t& op0, uint32_t& op1,
			       uint32_t& op2, uint32_t& op3) const;

    /// Decode a vector load instruction.
    const InstEntry& decodeVecLoad(uint32_t f3, uint32_t imm12,
				   uint32_t& fieldCount) const;

    /// Decode a vector store instruction.
    const InstEntry& decodeVecStore(uint32_t f3, uint32_t imm12,
				    uint32_t& fieldCount) const;

    /// Decode some of the vector crypto instructions.
    const InstEntry& decodeVecCrypto(uint32_t inst, uint32_t& op0, uint32_t& op1,
				     uint32_t& op2) const;

    /// Decode a compressed instruction.
    const InstEntry& decode16(uint16_t inst, uint32_t& op0, uint32_t& op1,
			      uint32_t& op2) const;

  private:

    InstTable instTable_;
    bool rv64_ = false;
  };
}

