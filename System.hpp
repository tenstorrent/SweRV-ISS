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

#include <memory>               // For shared_ptr
#include <functional>
#include <string_view>
#include <unordered_map>
#include "Memory.hpp"
#include "SparseMem.hpp"

namespace WdRiscv
{

  template <typename URV>
  class Hart;

  template <typename URV>
  class Core;

  template <typename URV>
  class Mcm;

  class DecodedInst;
  class IoDevice;

  /// Model a system consisting of n cores with m-harts per core and a
  /// memory. The harts in the system are indexed from 0 to n*m -
  /// 1. The type URV (unsigned register value) is that of the integer
  /// register and is either uint32_t or uint64_t.
  template <typename URV>
  class System
  {
  public:

    typedef Hart<URV> HartClass;
    typedef Core<URV> CoreClass;

    /// Constructor: Construct a system with n (coreCount) cores each
    /// consisting of m (hartsPerCore) harts. The harts in this system
    /// are indexed with 0 to n*m - 1.  Each core is assigned a
    /// hart-id start from the sequence 0, hartIdOffset,
    /// 2*hartIdOffset, ...  Harts in a core are assigned consecutive
    /// hart-ids (values of MHARTID CSRs) starting with the start if
    /// od the core.
    System(unsigned coreCount, unsigned hartsPerCore, unsigned hartIdOffset,
           size_t memSize, size_t pageSize);

    ~System();

    /// Return count of cores in this system.
    unsigned coreCount() const
    { return cores_.size(); }

    /// Return the number of harts per core.
    unsigned hartsPerCore() const
    { return hartsPerCore_; }

    /// Return count of harts (coreCount * hartsPerCore) in this
    /// system.
    unsigned hartCount() const
    { return hartCount_; }

    /// Return pointer to the ith hart in the system or null if i is
    /// out of bounds. A hart index is valid if it is less than the
    /// value returned by the hartCount method.
    std::shared_ptr<HartClass> ithHart(unsigned i) const
    {
      if (i >= sysHarts_.size())
	return std::shared_ptr<HartClass>();
      return sysHarts_.at(i);
    }

    /// Return pointer to this system hart having the given value as
    /// its hart-id (value of MHARTID CSR) or null if no such hart.
    std::shared_ptr<HartClass> findHartByHartId(URV hartId) const
    {
      const auto& iter = hartIdToIndex_.find(hartId);
      if (iter != hartIdToIndex_.end())
        return ithHart(iter->second);
      return std::shared_ptr<HartClass>();
    }

    /// Return pointer to the ith core in the system or null if i is
    /// out of bounds.
    std::shared_ptr<CoreClass> ithCore(unsigned i)
    {
      if (i >= cores_.size())
	return std::shared_ptr<CoreClass>();
      return cores_.at(i);
    }

    /// With a true flag, when loading ELF files, error out if ELF
    /// file refers to unmapped memory. With a false flag, ignore
    /// unmapped memory in the ELF file.
    void checkUnmappedElf(bool flag);

    /// Enable compressed address tracing (last occurrence of each
    /// address is reported) snapping addresses to a multiple of the
    /// line size. For example, if addreses 15, 65, 67, 129, 68
    /// are generated by the program then the line addresses will be
    /// 0, 1, 1, 2, 1 and the compresses addresse reported will be
    /// 0, 2, 1.
    void enableDataLineTrace(const std::string& path)
    { memory_->enableDataLineTrace(path); }

    /// Similar to enableDataLineTrace but for instructions.
    void enableInstructionLineTrace(const std::string& path)
    {
      memory_->enableInstructionLineTrace(path);
      for (auto hart : sysHarts_)  hart->enableInstructionLineTrace();
    }

    /// Define read memory callback. This (along with
    /// defineWriteMemoryCallback) allows the caller to bypass the
    /// memory model with their own.
    void defineReadMemoryCallback(
         std::function<bool(uint64_t, unsigned, uint64_t&)> callback )
    {
      memory_->defineReadMemoryCallback(callback);
    }

    /// Define write memory callback. This (along with
    /// defineReadMemoryCallback) allows the caller to bypass the
    /// memory model with their own.
    void defineWriteMemoryCallback(
         std::function<bool(uint64_t, unsigned, uint64_t)> callback )
    {
      memory_->defineWriteMemoryCallback(callback);
    }

    /// Break a hart-index-in-system into a core-index and a
    /// hart-index in core. Return true if successful and false if
    /// igven hart-index-in-system is out of bounds.
    bool unpackSystemHartIx(unsigned hartIxInSys, unsigned& coreIx,
                            unsigned& hartIxInCore)
    {
      if (hartIxInSys >= sysHarts_.size())
        return false;
      coreIx = hartIxInSys / hartsPerCore_;
      hartIxInCore = hartIxInSys % hartsPerCore_;
      return true;
    }

    /// Print the ELF symbols on the given stream. Output format:
    /// <name> <value>
    void printElfSymbols(std::ostream& out) const
    { memory_->printElfSymbols(out); }

    /// Locate the given ELF symbol (symbols are collected for every
    /// loaded ELF file) returning true if symbol is found and false
    /// otherwise. Set value to the corresponding value if symbol is
    /// found.
    bool findElfSymbol(const std::string& symbol, ElfSymbol& value) const
    { return memory_->findElfSymbol(symbol, value); }

    /// Load the given ELF files into memory. Return true on succes
    /// and false on failure. If not raw, and a hart PC is non-zero
    /// then set that PC to the first non-zero ELF entry point found.
    /// Similary set the global pointer regiser to the value of the
    /// ELF __global_pointer$ symbol.
    bool loadElfFiles(const std::vector<std::string>& files, bool raw, bool verbose);

    /// Load the given hex files and set memory locations accordingly.
    /// Return true on success. Return false if file does not exists,
    /// cannot be opened or contains malformed data.
    /// File format: A line either contains @address where address
    /// is a hexadecimal memory address or one or more space separated
    /// tokens each consisting of two hexadecimal digits.
    bool loadHexFiles(const std::vector<std::string>& files, bool verbose);

    /// Load the binary files and set memory locations accordingly.
    /// Return true on success. Return false if file does not exist,
    /// or cannot be opened. A file is loaded at the given default
    /// offset unless the filename has the form <path>:<value>
    /// where value is an integer in which case the effective file
    /// name will be <path> and the load addres will be <value>.
    bool loadBinaryFiles(const std::vector<std::string>& files,
			 uint64_t defOffset, bool verbose);

    /// Save snapshot (registers, memory etc) into given directory
    /// which should alread exist. Return true on success.
    bool saveSnapshot(Hart<URV>& hart, const std::string& dirPath);

    /// Load snapshot (registers, memory etc) from the given drectory.
    /// Return true on succes.
    bool loadSnapshot(const std::string& dirPath, Hart<URV>& hart);

    /// Write contents of memory accessed by current run in verilog
    /// hex format to the file at the given path. Return true on
    /// success and false on failure. Currently this will write the
    /// contents of accessed pages.
    bool writeAccessedMemory(const std::string& path) const;

    /// Special target program symbol writing to which stops the
    /// simulated program or performs console io.
    void setTohostSymbol(const std::string& sym)
    { toHostSym_ = sym; }

    /// Special target program symbol writing to which provide
    /// host data (console input) to the simulated hart.
    void setFromHostSymbol(const std::string& sym)
    { fromHostSym_ = sym; }

    /// Special target program symbol writing/reading to/from which
    /// writes/reads to/from the console.
    void setConsoleIoSymbol(const std::string& sym)
    { consoleIoSym_ = sym; }

    /// Device a UART device at given address reserving given
    /// size (in bytes) of address space for it.
    void defineUart(uint64_t addr, uint64_t size);

    /// Enable memory consistency model. This is relevant in
    /// server/interactive where RTL monitor or interactive command
    /// may initiate out of order memory transactions. Behavior is
    /// undefined if used in non-server/non-interactive mode or if
    /// used after execution has started. The mergeBuffserSize is
    /// the merge buffer line size in bytes.
    bool enableMcm(unsigned mergeBufferSize, bool mbLineCheckAll);

    /// Return true if memory consistency model is enabled.
    bool isMcmEnabled() const
    { return mcm_ != nullptr; }

    /// Return the merge buffer line size in bytes (see enableMcm).
    unsigned mergeBufferSize() const
    { return mbSize_; }

    bool mcmRead(Hart<URV>& hart, uint64_t time, uint64_t tag, uint64_t addr,
		 unsigned size, uint64_t data, bool internal);

    /// Initiate a merge buffer write.  All associated store write
    /// transactions are marked completed. Write instructions where
    /// all writes are complete are marked complete. Return true on
    /// success.  The given physical address must be a multiple of the
    /// merge buffer line size (which is also the cache line
    /// size). The rtlData vector must be of size n or larger where n
    /// is the merge buffer line size. The rtlData bytes will be
    /// placed in memory in consecutive locations starting with
    /// physAddr. If not-empty, the mask vector contains a flag that
    /// is set for each byte of rtlData that is written by the RTL.
    bool mcmMbWrite(Hart<URV>& hart, uint64_t time, uint64_t pysAddr,
		    const std::vector<uint8_t>& rtlData,
		    const std::vector<bool>& mask);

    bool mcmMbInsert(Hart<URV>& hart, uint64_t time, uint64_t tag,
		     uint64_t addr, unsigned size, uint64_t data);

    bool mcmRetire(Hart<URV>& hart, uint64_t time, uint64_t tag,
		   const DecodedInst& di);

    bool mcmSetCurrentInstruction(Hart<URV>& hart, uint64_t tag);

    /// Produce a signature file used to score tests from the
    /// riscv-arch-tests project.  The file is written to the
    // path specified in the parameter.
    bool produceTestSignatureFile(std::string_view outPath) const;

  private:

    unsigned hartCount_;
    unsigned hartsPerCore_;

    std::vector< std::shared_ptr<CoreClass> > cores_;
    std::vector< std::shared_ptr<HartClass> > sysHarts_; // All harts in system.
    std::unordered_map<URV, unsigned> hartIdToIndex_;
    std::unique_ptr<Memory> memory_;
    std::unique_ptr<SparseMem> sparseMem_;
    std::shared_ptr<Mcm<URV>> mcm_;
    unsigned mbSize_ = 64;  // Merge buffer size.
    std::string toHostSym_ = "tohost";   // ELF symbol to use as "tohost" addr.
    std::string fromHostSym_ = "fromhost";
    std::string consoleIoSym_ = "__whisper_console_io";  // ELF symbol to use as console-io addr.
    std::vector<std::shared_ptr<IoDevice>> ioDevs_;
  };
}
