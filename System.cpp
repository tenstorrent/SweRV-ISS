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

#include <iostream>
#include <fstream>
#include "Filesystem.hpp"
#include "Hart.hpp"
#include "Core.hpp"
#include "System.hpp"
#include "Mcm.hpp"
#include "Uart8250.hpp"
#include "Uartsf.hpp"


using namespace WdRiscv;


inline bool
isPowerOf2(uint64_t x)
{
  return x != 0 and (x & (x-1)) == 0;
}


template <typename URV>
System<URV>::System(unsigned coreCount, unsigned hartsPerCore,
                    unsigned hartIdOffset, size_t memSize,
                    size_t pageSize)
  : hartCount_(coreCount * hartsPerCore), hartsPerCore_(hartsPerCore)
{
  cores_.resize(coreCount);

  memory_ = std::make_shared<Memory>(memSize, pageSize);
  sparseMem_ = nullptr;

  Memory& mem = *(memory_.get());
  mem.setHartCount(hartCount_);

  for (unsigned ix = 0; ix < coreCount; ++ix)
    {
      URV coreHartId = ix * hartIdOffset;
      cores_.at(ix) = std::make_shared<CoreClass>(coreHartId, ix, hartsPerCore, mem);

      // Maintain a vector of all the harts in the system.  Map hart-id to index
      // of hart in system.
      auto core = cores_.at(ix);
      for (unsigned i = 0; i < hartsPerCore; ++i)
        {
          auto hart = core->ithHart(i);
          sysHarts_.push_back(hart);
          URV hartId = coreHartId + i;
          unsigned hartIx = ix*hartsPerCore + i;
          hartIdToIndex_[hartId] = hartIx;
        }
    }

#ifdef MEM_CALLBACKS
  sparseMem_ = new SparseMem();
  auto readf = [this](uint64_t addr, unsigned size, uint64_t& value) -> bool {
                 return sparseMem_->read(addr, size, value); };
  auto writef = [this](uint64_t addr, unsigned size, uint64_t value) -> bool {
                  return sparseMem_->write(addr, size, value); };

  mem.defineReadMemoryCallback(readf);
  mem.defineWriteMemoryCallback(writef);
#endif

}


template <typename URV>
void
System<URV>::defineUart(uint64_t addr, uint64_t size)
{
  auto uart = new Uartsf(addr, size);
  ioDevs_.push_back(uart);
  memory_->registerIoDevice(uart);
}


template <typename URV>
System<URV>::~System()
{
  delete sparseMem_;
  sparseMem_ = nullptr;

  delete mcm_;
  mcm_ = nullptr;

  for (auto dev : ioDevs_)
    delete dev;
}


template <typename URV>
void
System<URV>::checkUnmappedElf(bool flag)
{
  if (memory_)
    memory_->checkUnmappedElf(flag);
}


template <typename URV>
bool
System<URV>::writeAccessedMemory(const std::string& path) const
{
  if (not sparseMem_)
    return false;
  return sparseMem_->writeHexFile(path);
}


template <typename URV>
bool
System<URV>::loadElfFiles(const std::vector<std::string>& files, bool raw, bool verbose)
{
  unsigned registerWidth = sizeof(URV)*8;
  uint64_t end = 0, entry = 0;
  uint64_t gp = 0, tp = 0; // gp: global-pointer, tp: thread-pointer
  unsigned errors = 0;
  ElfSymbol sym;

  for (const auto& file : files)
    {
      if (verbose)
	std::cerr << "Loading ELF file " << file << '\n';
      uint64_t end0 = 0, entry0 = 0;
      if (not memory_->loadElfFile(file, registerWidth, entry0, end0))
	errors++;
      else
	{
	  if (not entry)
	    entry = entry0;

	  if (memory_->findElfSymbol("_end", sym))   // For newlib/linux emulation.
	    end = std::max(end, sym.addr_);
	  else
	    end = std::max(end, end0);

	  if (not gp and memory_->findElfSymbol("__global_pointer$", sym))
	    gp = sym.addr_;

	  if (not tp and memory_->findElfSection(".tdata", sym))
	    tp = sym.addr_;
	}
    }

  for (auto hart : sysHarts_)
    {
      if (not toHostSym_.empty() and memory_->findElfSymbol(toHostSym_, sym))
	hart->setToHostAddress(sym.addr_);
      if (not fromHostSym_.empty() and memory_->findElfSymbol(fromHostSym_, sym))
	hart->setFromHostAddress(sym.addr_);
      if (not consoleIoSym_.empty() and memory_->findElfSymbol(consoleIoSym_, sym))
	hart->setConsoleIo(URV(sym.addr_));
      hart->setTargetProgramBreak(end);

      if (not raw)
	{
	  if (not hart->peekIntReg(RegGp) and gp)
	    hart->pokeIntReg(RegGp, URV(gp));
	  if (not hart->peekIntReg(RegTp) and tp)
	    hart->pokeIntReg(RegTp, URV(tp));
	  if (entry)
	    hart->pokePc(URV(entry));
	}
    }

  return true;
}


template <typename URV>
bool
System<URV>::loadHexFiles(const std::vector<std::string>& files, bool verbose)
{
  unsigned errors = 0;
  for (const auto& file : files)
    {
      if (verbose)
	std::cerr << "Loading HEX file " << file << '\n';
      if (not memory_->loadHexFile(file))
	errors++;
    }
  return errors == 0;
}


template <typename URV>
bool
System<URV>::loadBinaryFiles(const std::vector<std::string>& files,
			     uint64_t defOffset, bool verbose)
{
  unsigned errors = 0;

  for (const auto& binaryFile : files)
    {
      std::string filename = binaryFile;
      uint64_t offset = defOffset;
      auto end = binaryFile.find(":");
      if (end != std::string::npos)
        {
          filename = binaryFile.substr(0, end);
          std::string offsStr = binaryFile.substr(end + 1, binaryFile.length());
          offset = strtoull(offsStr.c_str(), nullptr, 0);
        }
      else
        std::cerr << "Binary file " << binaryFile << " does not have an address, will use address 0x"
		  << std::hex << offset << std::dec << '\n';

      if (verbose)
	std::cerr << "Loading binary " << filename << " at address 0x" << std::hex
		  << offset << std::dec << '\n';

      if (not memory_->loadBinaryFile(filename, offset))
        errors++;
    }

  return errors == 0;
}


bool
saveUsedMemBlocks(const std::string& filename,
		  std::vector<std::pair<uint64_t, uint64_t>>& blocks)
{
  std::ofstream ofs(filename, std::ios::trunc);
  if (not ofs)
    {
      std::cerr << "saveUsedMemBlocks failed - cannot open "
                << filename << " for write\n";
      return false;
    }
  for (auto& it: blocks)
    ofs << it.first << " " << it.second << "\n";
  return true;
}


template <typename URV>
bool
System<URV>::saveSnapshot(Hart<URV>& hart, const std::string& dir)
{
  Filesystem::path dirPath = dir;

  Filesystem::path regPath = dirPath / "registers";
  if (not hart.saveSnapshotRegs(regPath.string()))
    return false;

  auto& syscall = hart.getSyscall();

  Filesystem::path usedBlocksPath = dirPath / "usedblocks";
  std::vector<std::pair<uint64_t,uint64_t>> usedBlocks;
  if (sparseMem_)
    sparseMem_->getUsedBlocks(usedBlocks);
  else
    syscall.getUsedMemBlocks(usedBlocks);
  if (not saveUsedMemBlocks(usedBlocksPath.string(), usedBlocks))
    return false;

  Filesystem::path memPath = dirPath / "memory";
  if (not memory_->saveSnapshot(memPath.string(), usedBlocks))
    return false;

  Filesystem::path fdPath = dirPath / "fd";
  if (not syscall.saveFileDescriptors(fdPath.string()))
    return false;

  Filesystem::path mmapPath = dirPath / "mmap";
  if (not syscall.saveMmap(mmapPath.string()))
    return false;

  Filesystem::path cachePath = dirPath / "cache";
  if (not memory_->saveCacheSnapshot(cachePath))
    return false;

  Filesystem::path dtracePath = dirPath / "data-lines";
  if (not memory_->saveDataAddressTrace(dtracePath))
    return false;

  Filesystem::path itracePath = dirPath / "instr-lines";
  if (not memory_->saveInstructionAddressTrace(itracePath))
    return false;

  return true;
}


static
bool
loadUsedMemBlocks(const std::string& filename,
		  std::vector<std::pair<uint64_t, uint64_t>>& blocks)
{
  blocks.clear();
  std::ifstream ifs(filename);
  if (not ifs)
    {
      std::cerr << "loadUsedMemBlocks failed - cannot open "
                << filename << " for read\n";
      return false;
    }

  typedef std::pair<uint64_t, uint64_t> Pair;

  std::string line;
  while (std::getline(ifs, line))
    {
      std::istringstream iss(line);
      uint64_t addr, length;
      iss >> addr;
      iss >> length;
      blocks.push_back(Pair{addr, length});
    }

  return true;
}


template <typename URV>
bool
System<URV>::loadSnapshot(const std::string& dir, Hart<URV>& hart)
{
  Filesystem::path dirPath = dir;
  std::vector<std::pair<uint64_t,uint64_t>> usedBlocks;

  Filesystem::path regPath = dirPath / "registers";
  if (not hart.loadSnapshotRegs(regPath.string()))
    return false;

  auto& syscall = hart.getSyscall();
  Filesystem::path usedBlocksPath = dirPath / "usedblocks";
  if (not loadUsedMemBlocks(usedBlocksPath.string(), usedBlocks))
    return false;

  Filesystem::path mmapPath = dirPath / "mmap";
  if (not syscall.loadMmap(mmapPath.string()))
    return false;

  Filesystem::path memPath = dirPath / "memory";
  if (not memory_->loadSnapshot(memPath.string(), usedBlocks))
    return false;

  Filesystem::path fdPath = dirPath / "fd";
  if (not syscall.loadFileDescriptors(fdPath.string()))
    return false;

  Filesystem::path cachePath = dirPath / "cache";
  if (Filesystem::is_regular_file(cachePath))
    if (not memory_->loadCacheSnapshot(cachePath.string()))
      return false;

  return true;
}


template <typename URV>
bool
System<URV>::enableMcm(unsigned mbLineSize, bool mbLineCheckAll)
{
  if (mcm_)
    {
      delete mcm_;
      mcm_ = nullptr;
    }

  if (mbLineSize != 0)
    if (not isPowerOf2(mbLineSize) or mbLineSize > 512)
      {
	std::cerr << "Error: Invalid merge buffer line size: "
		  << mbLineSize << '\n';
	return false;
      }

  mcm_ = new Mcm<URV>(this->hartCount(), mbLineSize);
  mbSize_ = mbLineSize;
  mcm_->setCheckWholeMbLine(mbLineCheckAll);

  for (auto hart :  sysHarts_)
    hart->setMcm(mcm_);

  return true;
}


template <typename URV>
bool
System<URV>::mcmRead(Hart<URV>& hart, uint64_t time, uint64_t tag,
		     uint64_t addr, unsigned size, uint64_t data,
		     bool internal)
{
  if (not mcm_)
    return false;
  return mcm_->readOp(hart, time, tag, addr, size, data, internal);
}


template <typename URV>
bool
System<URV>::mcmMbWrite(Hart<URV>& hart, uint64_t time, uint64_t addr,
			const std::vector<uint8_t>& data,
			const std::vector<bool>& mask)
{
  if (not mcm_)
    return false;
  return mcm_->mergeBufferWrite(hart, time, addr, data, mask);
}


template <typename URV>
bool
System<URV>::mcmMbInsert(Hart<URV>& hart, uint64_t time, uint64_t tag,
			 uint64_t addr, unsigned size, uint64_t data)
{
  if (not mcm_)
    return false;
  return mcm_->mergeBufferInsert(hart, time, tag, addr, size, data);
}


template <typename URV>
bool
System<URV>::mcmRetire(Hart<URV>& hart, uint64_t time, uint64_t tag,
		       const DecodedInst& di)
{
  if (not mcm_)
    return false;
  return mcm_->retire(hart, time, tag, di);
}

template <typename URV>
bool
System<URV>::mcmSetCurrentInstruction(Hart<URV>& hart, uint64_t tag)
{
  if (not mcm_)
    return false;
  return mcm_->setCurrentInstruction(hart, tag);
}


template class WdRiscv::System<uint32_t>;
template class WdRiscv::System<uint64_t>;
