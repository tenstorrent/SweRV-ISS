// Copyright 2022 Tenstorrent Corporation or its affiliates.
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

#include <iosfwd>
#include <string>
#include <functional>
#include <unordered_map>
#include "IntRegNames.hpp"
#include "FpRegNames.hpp"
#include "DecodedInst.hpp"


namespace WdRiscv
{

  class Decoder;

  /// Disassemble a decoded instruction.
  class Disassembler
  {
  public:

    Disassembler()
    { }

    ~Disassembler()
    { }

    /// Enable/disable use of abi-names when priting register names.
    /// For example: We print "x2" when abi names are disabled and
    /// "sp" when they are enabeld.
    void enableAbiNames(bool flag)
    { abiNames_ = flag; }

    /// Return true if abi-names are enabled.
    bool abiNames() const
    { return abiNames_; }

    /// Disassemble given instruction putting the results in the
    /// given string (cleared on entry).
    void disassembleInst(const DecodedInst& di, std::string& str);

    /// Decode gigen instruction and disassmble it puting the results
    /// in the given sring (cleared on entry).
    void disassembleInst(uint32_t instruction, const Decoder& decoder,
			 std::string& str);

    /// Return the name of the integer register of the given index.
    const std::string& intRegName(unsigned ix) const
    { return intNames_.regName(ix, abiNames_); }

    /// Return the name of the floating point register of the given index.
    const std::string& fpRegName(unsigned ix) const
    { return fpNames_.regName(ix, abiNames_); }

    /// Return the name of the CSR of the given index.
    std::string csRegName(unsigned ix) const
    {
      std::string name;
      if (csrNameCallback_)
	name = csrNameCallback_(ix);
      if (name.empty())
	name = "c" + std::to_string(ix);
      return name;
    }

    /// Set a callback to obtain the abi CSR name.
    void setCsrNameCallback(std::function<std::string(unsigned)> callback)
    { csrNameCallback_ = callback; }

  protected:

    /// Uncached disassembly.
    void disassembleUncached(const DecodedInst& di, std::ostream& out);

    /// Cachsed disassembly.
    void disassemble(const DecodedInst& di, std::string& str);

  private:

    IntRegNames intNames_;
    FpRegNames fpNames_;
    bool abiNames_ = false;
    std::function<std::string(unsigned ix)> csrNameCallback_ = nullptr;

    std::unordered_map<uint32_t, std::string> disasMap_;
  };
}
