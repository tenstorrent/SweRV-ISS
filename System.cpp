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

#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include "Filesystem.hpp"
#include "Hart.hpp"
#include "Core.hpp"
#include "SparseMem.hpp"
#include "System.hpp"
#include "Mcm.hpp"
#include "PerfModel.hpp"
#include "Uart8250.hpp"
#include "Uartsf.hpp"
#include "pci/virtio/Blk.hpp"


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
  : hartCount_(coreCount * hartsPerCore), hartsPerCore_(hartsPerCore),
    imsicMgr_((coreCount * hartsPerCore), pageSize), time_(0)
{
  cores_.resize(coreCount);

  memory_ = std::make_unique<Memory>(memSize, pageSize);
  sparseMem_ = nullptr;

  Memory& mem = *memory_;
  mem.setHartCount(hartCount_);

  for (unsigned ix = 0; ix < coreCount; ++ix)
    {
      URV coreHartId = ix * hartIdOffset;
      cores_.at(ix) = std::make_shared<CoreClass>(coreHartId, ix, hartsPerCore, mem, time_);

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
  sparseMem_ = std::make_unique<SparseMem>();
  auto readf = [this](uint64_t addr, unsigned size, uint64_t& value) -> bool {
                 return sparseMem_->read(addr, size, value); };
  auto writef = [this](uint64_t addr, unsigned size, uint64_t value) -> bool {
                  return sparseMem_->write(addr, size, value); };
  auto mapf = [this](uint64_t addr, size_t size) -> uint8_t* {
                  return sparseMem_->map(addr, size); };

  mem.defineReadMemoryCallback(readf);
  mem.defineWriteMemoryCallback(writef);
  mem.defineMapMemoryCallback(mapf);
#endif
}


template <typename URV>
void
System<URV>::defineUart(uint64_t addr, uint64_t size)
{
  auto uart = std::make_shared<Uartsf>(addr, size);
  memory_->registerIoDevice(uart);
  ioDevs_.push_back(std::move(uart));
}


template <typename URV>
System<URV>::~System()
{
  // Final MCM checks
  if (mcm_)
    for (const auto& hartPtr : sysHarts_)
      mcm_->finalChecks(*hartPtr);

  // Write back binary files were marked for update.
  for (auto bf : binaryFiles_)
    {
      auto path = std::get<0>(bf);
      uint64_t addr = std::get<1>(bf);
      uint64_t size = std::get<2>(bf);
      std::cerr << "Updating " << path << " from addr: 0x" << std::hex << addr
		<< std::dec << " size: " << size << '\n';
      FILE* file = fopen(path.c_str(), "w");
      if (not file)
	{
	  std::cerr << "Failed to open " << path << " for update\n";
	  continue;
	}
      for (uint64_t i = 0; i < size; ++i)
	{
	  uint8_t byte = 0;
	  memory_->peek(addr + i, byte, false);
	  fputc(byte, file);
	}
      fclose(file);
    }
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

  for (const auto& hart : sysHarts_)
    {
      if (not toHostSym_.empty() and memory_->findElfSymbol(toHostSym_, sym))
	hart->setToHostAddress(sym.addr_);
      if (not fromHostSym_.empty() and memory_->findElfSymbol(fromHostSym_, sym))
	hart->setFromHostAddress(sym.addr_);
      if (not consoleIoSym_.empty() and memory_->findElfSymbol(consoleIoSym_, sym))
	hart->setConsoleIo(URV(sym.addr_));

      if (verbose)
	std::cerr << "Setting program break to 0x" << std::hex << end << std::dec << '\n';
      hart->setTargetProgramBreak(end);

      if (not raw)
	{
	  if (not hart->peekIntReg(RegGp) and gp)
	    {
              if (verbose)
                std::cerr << "Setting register gp to 0x" << std::hex << gp << std::dec << '\n';
	      hart->pokeIntReg(RegGp, URV(gp));
	    }
	  if (not hart->peekIntReg(RegTp) and tp)
	    {
              if (verbose)
                std::cerr << "Setting register tp to 0x" << std::hex << tp << std::dec << '\n';
	      hart->pokeIntReg(RegTp, URV(tp));
	    }
	  if (entry)
	    {
	      if (verbose)
                std::cerr << "Setting PC to 0x" << std::hex << entry << std::dec << '\n';
	      hart->pokePc(URV(entry));
	    }
	}
    }

  return errors == 0;
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


static bool
binaryFileParams(std::string spec, uint64_t defOffset, std::string& filename, uint64_t &offset, bool &update)
{
  using std::cerr;
  // Split filespec around colons. Spec format: <file>,
  // <file>:<offset>, or <file>:<offset>:u
  std::vector<std::string> parts;
  boost::split(parts, spec, boost::is_any_of(":"));

  filename = parts.at(0);
  offset = defOffset;
  update = false;

  if (parts.empty())
    {
      std::cerr << "Error: Empty binary file name\n";
      return false;
    }

  filename = parts.at(0);

  if (parts.size() > 1)
    {
      std::string offsStr = parts.at(1);
      if (offsStr.empty())
	cerr << "Warning: Empty binary file offset: " << spec << '\n';
      else
	{
	  char* tail = nullptr;
	  offset = strtoull(offsStr.c_str(), &tail, 0);
	  if (tail and *tail)
	    {
	      cerr << "Error: Invalid binary file offset: " << spec << '\n';
	      return false;
	    }
	}
    }
  else
    cerr << "Binary file " << filename << " does not have an address, will use address 0x"
	 << std::hex << offset << std::dec << '\n';

  if (parts.size() > 2)
    {
      if (parts.at(2) != "u")
	{
	  cerr << "Error: Invalid binary file attribute: " << spec << '\n';
	  return false;
	}
      update = true;
    }

  return true;
}


template <typename URV>
bool
System<URV>::loadBinaryFiles(const std::vector<std::string>& fileSpecs,
			     uint64_t defOffset, bool verbose)
{
  using std::cerr;
  unsigned errors = 0;

  for (const auto& spec : fileSpecs)
    {

      std::string filename;
      uint64_t offset;
      bool update;

      if (!binaryFileParams(spec, defOffset, filename, offset, update))
        {
	  errors++;
	  continue;
        }

      if (verbose)
	cerr << "Loading binary " << filename << " at address 0x" << std::hex
	     << offset << std::dec << '\n';

      if (not memory_->loadBinaryFile(filename, offset))
	{
	  errors++;
	  continue;
	}

      if (update)
	{
	  uint64_t size = Filesystem::file_size(filename);
	  BinaryFile bf = { filename, offset, size };
	  binaryFiles_.push_back(bf);
	}
    }

  return errors == 0;
}


#ifdef LZ4_COMPRESS
template <typename URV>
bool
System<URV>::loadLz4Files(const std::vector<std::string>& fileSpecs,
			  uint64_t defOffset, bool verbose)
{
  using std::cerr;
  unsigned errors = 0;

  for (const auto& spec : fileSpecs)
    {
      std::string filename;
      uint64_t offset;
      bool update;

      if (!binaryFileParams(spec, defOffset, filename, offset, update))
        {
	  errors++;
	  continue;
        }

      if (update)
	{
	  cerr << "Updating not supported on lz4 files, ignoring " << filename << '\n';
	  errors++;
	  continue;
	}

      if (verbose)
	cerr << "Loading lz4 compressed file " << filename << " at address 0x" << std::hex
	     << offset << std::dec << '\n';

      if (not memory_->loadLz4File(filename, offset))
	{
	  errors++;
	  continue;
	}

      if (update)
	{
	  uint64_t size = Filesystem::file_size(filename);
	  BinaryFile bf = { filename, offset, size };
	  binaryFiles_.push_back(bf);
	}
    }

  return errors == 0;
}
#endif


static
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


static
bool
saveTime(const std::string& filename, uint64_t time)
{
  std::ofstream ofs(filename, std::ios::trunc);
  if (not ofs)
    {
      std::cerr << "saveTime failed - cannot open "
                << filename << " for write\n";
      return false;
    }
  ofs << std::dec << time << "\n";
  return true;
}


template <typename URV>
bool
System<URV>::saveSnapshot(const std::string& dir)
{
  Filesystem::path dirPath = dir;
  if (not Filesystem::is_directory(dirPath))
    if (not Filesystem::create_directories(dirPath))
      {
	std::cerr << "Error: Failed to create snapshot directory " << dir << '\n';
	return false;
      }

  uint64_t minSp = ~uint64_t(0);
  for (auto hartPtr : sysHarts_)
    {
      std::string name = "registers";
      if (hartCount_ > 1)
	name += std::to_string(hartPtr->sysHartIndex());
      Filesystem::path regPath = dirPath / name;
      if (not hartPtr->saveSnapshotRegs(regPath.string()))
	return false;

      URV sp = 0;
      if (not hartPtr->peekIntReg(IntRegNumber::RegSp, sp))
	assert(0);
      minSp = std::min(minSp, uint64_t(sp));
    }

  auto& hart0 = *ithHart(0);
  auto& syscall = hart0.getSyscall();

  Filesystem::path usedBlocksPath = dirPath / "usedblocks";
  std::vector<std::pair<uint64_t,uint64_t>> usedBlocks;
  if (sparseMem_)
    sparseMem_->getUsedBlocks(usedBlocks);
  else
    syscall.getUsedMemBlocks(minSp, usedBlocks);

  if (not saveUsedMemBlocks(usedBlocksPath.string(), usedBlocks))
    return false;

  Filesystem::path timePath = dirPath / "time";
  if (not saveTime(timePath.string(), time_))
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

  Filesystem::path dtracePath = dirPath / "data-lines";
  if (not memory_->saveDataAddressTrace(dtracePath))
    return false;

  Filesystem::path itracePath = dirPath / "instr-lines";
  if (not memory_->saveInstructionAddressTrace(itracePath))
    return false;

  Filesystem::path branchPath = dirPath / "branch-trace";
  return hart0.saveBranchTrace(branchPath);
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

  std::string line;
  while (std::getline(ifs, line))
    {
      std::istringstream iss(line);
      uint64_t addr, length;
      iss >> addr;
      iss >> length;
      blocks.emplace_back(addr, length);
    }

  return true;
}


static
bool
loadTime(const std::string& filename, uint64_t& time)
{
  std::ifstream ifs(filename);
  if (not ifs)
    {
      std::cerr << "loadTime failed - cannot open "
                << filename << " for read\n";
      return false;
    }

  std::string line;
  std::getline(ifs, line);
  uint64_t val = strtoull(line.c_str(), nullptr, 0);
  time = val;
  return true;
}


template <typename URV>
bool
System<URV>::configImsic(uint64_t mbase, uint64_t mstride,
			 uint64_t sbase, uint64_t sstride,
			 unsigned guests, unsigned ids,
                         bool trace)
{
  using std::cerr;

  size_t ps = pageSize();

  if ((mbase % ps) != 0)
    {
      cerr << "Error: IMISC mbase (0x" << std::hex << mbase << ") is not"
	   << " a multiple of page size (0x" << ps << ")\n" << std::dec;
      return false;
    }

  if (mstride == 0)
    {
      cerr << "Error: IMSIC mstride must not be zero.\n";
      return false;
    }

  if ((mstride % ps) != 0)
    {
      cerr << "Error: IMISC mstride (0x" << std::hex << mstride << ") is not"
	   << " a multiple of page size (0x" << ps << ")\n" << std::dec;
      return false;
    }

  if (sstride)
    {
      if ((sbase % ps) != 0)
	{
	  cerr << "Error: IMISC sbase (0x" << std::hex << sbase << ") is not"
	       << " a multiple of page size (0x" << ps << ")\n" << std::dec;
	  return false;
	}

      if ((sstride % ps) != 0)
	{
	  cerr << "Error: IMISC sstride (0x" << std::hex << sstride << ") is not"
	       << " a multiple of page size (0x" << ps << ")\n" << std::dec;
	  return false;
	}
    }

  if (guests and sstride < (guests + 1)*ps)
    {
      cerr << "Error: IMISC supervisor stride (0x" << std::hex << sstride << ") is"
	   << " too small for configured guests (" << std::dec << guests << ").\n";
      return false;
    }

  if (mstride and sstride)
    {
      unsigned hc = hartCount();
      uint64_t mend = mbase + hc*mstride, send = sbase + hc*sstride;
      if ((sbase > mbase and sbase < mend) or
	  (send > mbase and send < mend))
	{
	  cerr << "Error: IMSIC machine file address range overlaps that of supervisor.\n";
	  return false;
	}
    }

  if ((ids % 64) != 0)
    {
      cerr << "Error: IMSIC interrupt id limit (" << ids << ") is not a multiple of 64.\n";
      return false;
    }

  if (ids > 2048)
    {
      cerr << "Error: IMSIC interrupt id limit (" << ids << ") is larger than 2048.\n";
      return false;
    }

  bool ok = imsicMgr_.configureMachine(mbase, mstride, ids);
  ok = imsicMgr_.configureSupervisor(sbase, sstride, ids) and ok;
  ok = imsicMgr_.configureGuests(guests, ids) and ok;
  if (not ok)
    {
      cerr << "Error: Failed to configure IMSIC.\n";
      return false;
    }

  uint64_t mend = mbase + mstride * hartCount();
  uint64_t send = sbase + sstride * hartCount();

  auto readFunc = [this](uint64_t addr, unsigned size, uint64_t& data) -> bool {
    return this->imsicMgr_.read(addr, size, data);
  };

  auto writeFunc = [this](uint64_t addr, unsigned size, uint64_t data) -> bool {
    return  this->imsicMgr_.write(addr, size, data);
  };

  for (unsigned i = 0; i < hartCount(); ++i)
    {
      auto hart = ithHart(i);
      auto imsic = imsicMgr_.ithImsic(i);
      hart->attachImsic(imsic, mbase, mend, sbase, send, readFunc, writeFunc, trace);
    }

  return true;
}


template <typename URV>
bool
System<URV>::configPci(uint64_t configBase, uint64_t mmioBase, uint64_t mmioSize, unsigned buses, unsigned slots)
{
  if (mmioBase - configBase < (1ULL << 28))
    {
      std::cerr << "PCI config space typically needs 28bits to fully cover entire region" << std::endl;
      return false;
    }

  pci_ = std::make_shared<Pci>(configBase, mmioBase, mmioSize, buses, slots);

  for (auto& hart : sysHarts_)
    hart->attachPci(pci_, configBase, mmioBase, mmioSize);
  return true;
}


template <typename URV>
std::shared_ptr<PciDev>
System<URV>::defineVirtioBlk(std::string_view filename, bool ro) const
{
  return std::make_shared<Blk>(std::string(filename), ro);
}


template <typename URV>
bool
System<URV>::addPciDevices(const std::vector<std::string>& devs)
{
  if (not pci_)
    {
      std::cerr << "Please specify a PCI region in the json" << std::endl;
      return false;
    }

  auto devmapf = [this](uint64_t addr, size_t size) -> uint8_t* {
    return this->memory_->data(addr, size);
  };

  auto writef = [this](uint64_t addr, unsigned size, uint64_t data) -> bool {
    return this->imsicMgr_.write(addr, size, data);
  };

  for (const auto& devStr : devs)
    {
      std::shared_ptr<PciDev> dev;
      std::vector<std::string> tokens;
      boost::split(tokens, devStr, boost::is_any_of(":"), boost::token_compress_on);

      if (tokens.size() < 3)
        {
          std::cerr << "PCI device string should have at least 3 fields" << std::endl;
          return false;
        }

      std::string name = tokens.at(0);
      unsigned bus = std::stoi(tokens.at(1));
      unsigned slot = std::stoi(tokens.at(2));

      if (name == "virtio-blk")
        {
          if (not (tokens.size() == 4))
            {
              std::cerr << "virtio-blk requires backing input file" << std::endl;
              return false;
            }
          dev = defineVirtioBlk(tokens.at(3), false);
        }

      if (not pci_->register_device(dev, bus, slot, devmapf, writef))
        return false;
    }
  return true;
}


template <typename URV>
bool
System<URV>::enableMcm(unsigned mbLineSize, bool mbLineCheckAll)
{
  if (mbLineSize != 0)
    if (not isPowerOf2(mbLineSize) or mbLineSize > 512)
      {
	std::cerr << "Error: Invalid merge buffer line size: "
		  << mbLineSize << '\n';
	return false;
      }

  mcm_ = std::make_shared<Mcm<URV>>(this->hartCount(), pageSize(), mbLineSize);
  mbSize_ = mbLineSize;
  mcm_->setCheckWholeMbLine(mbLineCheckAll);

  for (auto& hart :  sysHarts_)
    hart->setMcm(mcm_);

  return true;
}


template <typename URV>
bool
System<URV>::enablePerfApi()
{
  if constexpr (sizeof(URV) == 4)
    {
      std::cerr << "Performance model API is not supported for RV32\n";
      return false;
    }
  else
    {
      perfApi_ = std::make_shared<TT_PERF::PerfApi>(*this);
      for (auto& hart : sysHarts_)
	hart->setPerfApi(perfApi_);
    }

  return true;
}


template <typename URV>
void
System<URV>::enableTso(bool flag)
{
  if (mcm_)
    mcm_->enableTso(flag);
}


template <typename URV>
bool
System<URV>::mcmRead(Hart<URV>& hart, uint64_t time, uint64_t tag,
		     uint64_t addr, unsigned size, uint64_t data)
{
  if (not mcm_)
    return false;
  return mcm_->readOp(hart, time, tag, addr, size, data);
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
System<URV>::mcmBypass(Hart<URV>& hart, uint64_t time, uint64_t tag,
		       uint64_t addr, unsigned size, uint64_t data)
{
  if (not mcm_)
    return false;
  return mcm_->bypassOp(hart, time, tag, addr, size, data);
}


template <typename URV>
bool
System<URV>::mcmIFetch(Hart<URV>& hart, uint64_t /*time*/, uint64_t addr)
{
  if (not mcm_)
    return false;
  return hart.mcmIFetch(addr);
}


template <typename URV>
bool
System<URV>::mcmIEvict(Hart<URV>& hart, uint64_t /*time*/, uint64_t addr)
{
  if (not mcm_)
    return false;
  return hart.mcmIEvict(addr);
}


template <typename URV>
bool
System<URV>::mcmRetire(Hart<URV>& hart, uint64_t time, uint64_t tag,
		       const DecodedInst& di, bool trapped)
{
  if (not mcm_)
    return false;
  return mcm_->retire(hart, time, tag, di, trapped);
}

template <typename URV>
bool
System<URV>::mcmSetCurrentInstruction(Hart<URV>& hart, uint64_t tag)
{
  if (not mcm_)
    return false;
  return mcm_->setCurrentInstruction(hart, tag);
}


template <typename URV>
void
System<URV>::perfApiCommandLog(FILE* log)
{
  if (not perfApi_)
    return;
  perfApi_->enableCommandLog(log);
}


template <typename URV>
bool
System<URV>::perfApiFetch(unsigned hart, uint64_t time, uint64_t tag, uint64_t vpc)
{
  if (not perfApi_)
    return false;

  bool trap; ExceptionCause cause; uint64_t trapPc;
  return perfApi_->fetch(hart, time, tag, vpc, trap, cause, trapPc);
}


template <typename URV>
bool
System<URV>::perfApiDecode(unsigned hart, uint64_t time, uint64_t tag, uint32_t opcode)
{
  if (not perfApi_)
    return false;
  return perfApi_->decode(hart, time, tag, opcode);
}


template <typename URV>
bool
System<URV>::perfApiExecute(unsigned hart, uint64_t time, uint64_t tag)
{
  if (not perfApi_)
    return false;
  return perfApi_->execute(hart, time, tag);
}


template <typename URV>
bool
System<URV>::perfApiRetire(unsigned hart, uint64_t time, uint64_t tag, FILE* traceFile)
{
  if (not perfApi_)
    return false;
  return perfApi_->retire(hart, time, tag, traceFile);
}


template <typename URV>
bool
System<URV>::perfApiDrainStore(unsigned hart, uint64_t time, uint64_t tag)
{
  if (not perfApi_)
    return false;
  return perfApi_->drainStore(hart, time, tag);
}


template <typename URV>
bool
System<URV>::perfApiPredictBranch(unsigned hart, uint64_t /*time*/, uint64_t tag,
				  bool taken, uint64_t target)
{
  if (not perfApi_)
    return false;
  return perfApi_->predictBranch(hart, tag, taken, target);
}


template <typename URV>
bool
System<URV>::perfApiFlush(unsigned hart, uint64_t time, uint64_t tag)
{
  if (not perfApi_)
    return false;
  return perfApi_->flush(hart, time, tag);
}


template <typename URV>
bool
System<URV>::perfApiShouldFlush(unsigned hart, uint64_t time, uint64_t tag, bool& flush,
				uint64_t& addr)
{
  flush = false;
  if (not perfApi_)
    return false;
  return perfApi_->shouldFlush(hart, time, tag, flush, addr);
}


template <typename URV>
bool
System<URV>::produceTestSignatureFile(std::string_view outPath) const
{
  // Find the begin_signature and end_signature section addresses
  // from the elf file
  WdRiscv::ElfSymbol beginSignature, endSignature;
  for (auto&& [symbolName, pSymbol] : { std::pair{ "begin_signature", &beginSignature },
                                        std::pair{ "end_signature",   &endSignature } })
    {
      if (not findElfSymbol(symbolName, *pSymbol))
        {
          std::cerr << "Failed to find symbol " << symbolName << " in memory.\n";
          return false;
        }
    }

  if (beginSignature.addr_ > endSignature.addr_)
    {
      std::cerr << "Ending address for signature file is before starting address.\n";
      return false;
    }

  // Get all of the data between the two sections and store it in a vector to ensure
  // it can all be read correctly.
  std::vector<uint32_t> data;
  data.reserve((endSignature.addr_ - beginSignature.addr_) / 4);
  for (std::size_t addr = beginSignature.addr_; addr < endSignature.addr_; addr += 4)
    {
      uint32_t value;
      if (not memory_->peek(addr, value, true))
        {
          std::cerr << "Unable to read data at address 0x" << std::hex << addr << ".\n";
          return false;
        }

      data.push_back(value);
    }

  // If all data has been read correctly, write it as 32-bit hex values with one
  // per line.
  std::ofstream outFile(outPath.data());
  outFile << std::hex << std::setfill('0');
  for (uint32_t value : data)
    {
      outFile << std::setw(8) << value << '\n';
    }

  return true;
}


template <typename URV>
bool
System<URV>::getSparseMemUsedBlocks(std::vector<std::pair<uint64_t, uint64_t>>& usedBlocks) const
{
  if (sparseMem_)
    {
      sparseMem_->getUsedBlocks(usedBlocks);
      return true;
    }
  return false;
}


extern void forceUserStop(int);


template <typename URV>
bool
System<URV>::batchRun(std::vector<FILE*>& traceFiles, bool waitAll, uint64_t stepWindow)
{
#if 0
  uint64_t trapPc = 0, tag = 1, time = 0;
  ExceptionCause cause = ExceptionCause::NONE;
  bool trap = false;

  unsigned hartIx = 0;
  Hart<URV>& hart0 = *ithHart(0);

  //perfApi_->setRetirePc(hart0->peekPc());

  uint64_t pc = hart0.peekPc();
  uint32_t opcode = 0xfebff0ef;
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);
  perfApi_->retire(hartIx, time++, tag, traceFiles.at(0));

  pc = 0x10158;   opcode = 0x80818713;  tag++;  // #2
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);
  perfApi_->retire(hartIx, time++, tag, traceFiles.at(0));

  pc = 0x1015c;   opcode = 0x4781;  tag++;  // #3
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);
  perfApi_->retire(hartIx, time++, tag, traceFiles.at(0));

  pc = 0x1015e;   opcode = 0x46a9;  tag++;  // #4
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);
  perfApi_->retire(hartIx, time++, tag, traceFiles.at(0));

  pc = 0x10160;   opcode = 0xc31c;  tag++; // #5
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);
  perfApi_->retire(hartIx, time++, tag, traceFiles.at(0));
  perfApi_->drainStore(hartIx, time++, tag);

  pc = 0x10162;   opcode = 0x2785;  tag++;  // #6
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);
  perfApi_->retire(hartIx, time++, tag, traceFiles.at(0));

  pc = 0x10164;   opcode = 0x0711;  tag++; // #7
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);
  perfApi_->retire(hartIx, time++, tag, traceFiles.at(0));

  pc = 0x10166;   opcode = 0xfed79de3; tag++; // #8
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);
  perfApi_->retire(hartIx, time++, tag, traceFiles.at(0));

  pc = 0x10160;   opcode = 0xc31c;  tag++; // #9
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);
  perfApi_->retire(hartIx, time++, tag, traceFiles.at(0));
  perfApi_->drainStore(hartIx, time++, tag);

  pc = 0x10162;   opcode = 0x2785;  tag++; // #10
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);

  pc = 0x10164;   opcode = 0x711;  tag++;// #11
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);

  perfApi_->execute(hartIx, time++, 11); // Execute #11 out of order
  perfApi_->execute(hartIx, time++, 10);

  perfApi_->retire(hartIx, time++, 10, traceFiles.at(0));
  perfApi_->retire(hartIx, time++, 11, traceFiles.at(0));

  pc = 0x10166;   opcode = 0xed79de3; tag++;  // #12
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);
  perfApi_->retire(hartIx, time++, tag, traceFiles.at(0));
  
  pc = 0x10160;   opcode = 0xc31c; tag++;  // #13  this will not execute for a while
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);

  pc = 0x10162;   opcode = 0x2785; tag++;  // #14
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);

  pc = 0x10164;   opcode = 0x0711; tag++;  // #15
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);

  pc = 0x10166;   opcode = 0xfed79de3; tag++;  // #16
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);

  pc = 0x10160;   opcode = 0xc31c; tag++;  // #17
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);

  pc = 0x10162;   opcode = 0x2785; tag++;  // #18
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);

  pc = 0x10164;   opcode = 0x0711; tag++;  // #19
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);

  pc = 0x10166;   opcode = 0xfed79de3; tag++; // #20
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);

  pc = 0x10160;   opcode = 0xc31c; tag++;  // #21
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);


  perfApi_->execute(hartIx, time++, 13);  // Execute 13

  // retire 14 to 21
  perfApi_->retire(hartIx, time++, 13, traceFiles.at(0));
  perfApi_->retire(hartIx, time++, 14, traceFiles.at(0));
  perfApi_->retire(hartIx, time++, 15, traceFiles.at(0));
  perfApi_->retire(hartIx, time++, 16, traceFiles.at(0));
  perfApi_->retire(hartIx, time++, 17, traceFiles.at(0));
  perfApi_->retire(hartIx, time++, 18, traceFiles.at(0));
  perfApi_->retire(hartIx, time++, 19, traceFiles.at(0));
  perfApi_->retire(hartIx, time++, 20, traceFiles.at(0));
  perfApi_->retire(hartIx, time++, 21, traceFiles.at(0));

  perfApi_->drainStore(hartIx, time++, 13);
  perfApi_->drainStore(hartIx, time++, 17);
  perfApi_->drainStore(hartIx, time++, 21);

  pc = 0x10162;   opcode = 0x2785; tag++; time++;  // #22
  perfApi_->fetch(hartIx, time, tag, pc, trap, cause, trapPc);
  pc = 0x10164;   opcode = 0x0711; tag++;  // #23
  perfApi_->fetch(hartIx, time, tag, pc, trap, cause, trapPc);

  perfApi_->decode(hartIx, time++, 22, opcode);
  perfApi_->decode(hartIx, time, 23, opcode);
  perfApi_->execute(hartIx, time++, 22);
  perfApi_->execute(hartIx, time, 23);
  perfApi_->retire(hartIx, time++, 22, traceFiles.at(0));
  perfApi_->retire(hartIx, time, 23, traceFiles.at(0));

  pc = 0x10166;   opcode = 0xfed79de3; tag++; // #24
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  pc = 0x1016a;   opcode = 0x4501; tag++; // #25, wrong fetch
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  pc = 0x1016c;   opcode = 0x8082; tag++; // #26, wrong fetch
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  pc = 0x10172;   opcode = 0x67c5; tag++; // #27, wrong fetch
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);

  auto packet = perfApi_->getInstructionPacket(hartIx, 24);
  packet->predictBranch(false, 0x1016a);
  perfApi_->execute(hartIx, time++, 24);

  bool flush = false;
  uint64_t addr = 0;
  if (perfApi_->shouldFlush(hartIx, time++, 24, flush, addr))
    {
      assert(flush);
      perfApi_->flush(hartIx, time++, 24);
    }

  tag = 33;
  pc = 0x10166;   opcode = 0xfed79de3; tag++;  // #34: redoing #24
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  pc = 0x10160;   opcode = 0xc31c; tag++; // #35
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  pc = 0x10162;   opcode = 0x2785; tag++; // #36, wrong fetch
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  pc = 0x10164;   opcode = 0x0711; tag++; // #37, wrong fetch
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);

  perfApi_->execute(hartIx, time++, 34);
  perfApi_->execute(hartIx, time++, 35);
  perfApi_->execute(hartIx, time++, 36);
  perfApi_->execute(hartIx, time++, 37);

  perfApi_->retire(hartIx, time++, 34, traceFiles.at(0));
  perfApi_->retire(hartIx, time++, 35, traceFiles.at(0));
  perfApi_->retire(hartIx, time++, 36, traceFiles.at(0));
  perfApi_->retire(hartIx, time++, 37, traceFiles.at(0));

  pc = 0x10166;   opcode = 0xfed79de3; tag++;  // #38
  perfApi_->fetch(hartIx, time++, tag, pc, trap, cause, trapPc);
  perfApi_->decode(hartIx, time++, tag, opcode);
  perfApi_->execute(hartIx, time++, tag);
  perfApi_->retire(hartIx, time++, tag, traceFiles.at(0));
  exit(0);
#endif

  auto forceSnapshot = [this]() -> void {
      uint64_t tag = ++snapIx_;
      std::string pathStr = snapDir_ + std::to_string(tag);
      Filesystem::path path = pathStr;
      if (not Filesystem::is_directory(path) and
	  not Filesystem::create_directories(path))
        {
          std::cerr << "Error: Failed to create snapshot directory " << pathStr << '\n';
          std::cerr << "Continuing...\n";
        }

      if (not saveSnapshot(pathStr))
        {
          std::cerr << "Error: Failed to save a snapshot\n";
          std::cerr << "Continuining...\n";
        }
  };

  if (hartCount() == 0)
    return true;

  while (true)
    {
      bool progSnapshot = false;
      std::atomic<bool> result = true;

      if (hartCount() == 1)
        {
          auto& hart = *ithHart(0);
          try
            {
              result = hart.run(traceFiles.at(0));
#ifdef FAST_SLOPPY
              hart.reportOpenedFiles(std::cout);
#endif
            }
          catch (const CoreException& ce)
            {
              if (ce.type() == CoreException::Type::Snapshot)
                progSnapshot = true;
            }
        }
      else if (not stepWindow)
        {
          // Run each hart in its own thread.
          std::vector<std::thread> threadVec;
          std::atomic<unsigned> finished = 0;  // Count of finished threads.

          auto threadFunc = [&result, &finished, &progSnapshot] (Hart<URV>* hart, FILE* traceFile) {
                              try
                                {
                                  bool r = hart->run(traceFile);
                                  result = result and r;
                                }
                              catch (const CoreException& ce)
                                {
                                  if (ce.type() == CoreException::Type::Snapshot)
                                    progSnapshot = true;
                                }
                              finished++;
                            };

          for (unsigned i = 0; i < hartCount(); ++i)
            {
              Hart<URV>* hart = ithHart(i).get();
              threadVec.emplace_back(std::thread(threadFunc, hart, traceFiles.at(i)));
            }

          if (not waitAll)
            {
              // First thread to finish terminates run.
              while (finished == 0)
                sleep(1);
              forceUserStop(0);
            }

          for (auto& t : threadVec)
            {
              if (progSnapshot)
                forceUserStop(0);
              t.join();
            }
        }
      else
        {
          // Run all harts in one thread round-robin.
          bool result = true;
          unsigned finished = 0;

          for (auto hptr : sysHarts_)
            finished += hptr->hasTargetProgramFinished();

          while ((waitAll and finished != hartCount()) or
                 (not waitAll and finished == 0))
            {
              for (auto hptr : sysHarts_)
                {
                  if (hptr->hasTargetProgramFinished())
                    continue;
                  unsigned steps = (rand() % stepWindow) + 1;
                  // step N times
                  unsigned ix = hptr->sysHartIndex();
                  try
                    {
                      result = hptr->runSteps(steps, traceFiles.at(ix)) and result;
                    }
                  catch (const CoreException& ce)
                    {
                      if (ce.type() == CoreException::Type::Snapshot)
                        progSnapshot = true;
                    }
                  if (hptr->hasTargetProgramFinished())
                    finished++;
                }

              if (progSnapshot)
                break;
            }
        }

      if (progSnapshot)
        forceSnapshot();
      else
        return result;
    }
}


/// Run producing a snapshot after each snapPeriod instructions. Each
/// snapshot goes into its own directory names <dir><n> where <dir> is
/// the string in snapDir and <n> is a sequential integer starting at
/// 0. Return true on success and false on failure.
template <typename URV>
bool
System<URV>::snapshotRun(std::vector<FILE*>& traceFiles, const std::vector<uint64_t>& periods)
{
  if (hartCount() == 0)
    return true;

  Hart<URV>& hart0 = *ithHart(0);

  uint64_t globalLimit = hart0.getInstructionCountLimit();

  for (size_t ix = 0; true; ++ix)
    {
      uint64_t nextLimit = globalLimit;
      if (not periods.empty())
	{
	  if (periods.size() == 1)
	    nextLimit = hart0.getInstructionCount() + periods.at(0);
	  else
	    nextLimit = ix < periods.size() ? periods.at(ix) : globalLimit;
	}

      nextLimit = std::min(nextLimit, globalLimit);

      uint64_t tag = 0;
      if (periods.size() > 1)
	tag = ix < periods.size() ? periods.at(ix) : nextLimit;
      else
        tag = ++snapIx_;
      std::string pathStr = snapDir_ + std::to_string(tag);
      Filesystem::path path = pathStr;
      if (not Filesystem::is_directory(path) and
	  not Filesystem::create_directories(path))
	{
	  std::cerr << "Error: Failed to create snapshot directory " << pathStr << '\n';
	  return false;
	}

      for (auto hartPtr : sysHarts_)
	hartPtr->setInstructionCountLimit(nextLimit);

      batchRun(traceFiles, true /*waitAll*/, 0 /*stepWindow*/);

      bool done = false;
      for (auto& hartPtr : sysHarts_)
	if (hartPtr->hasTargetProgramFinished() or nextLimit >= globalLimit)
	  {
	    done = true;
	    Filesystem::remove_all(path);
	    break;
	  }
      if (done)
	break;

      if (not saveSnapshot(pathStr))
	{
	  std::cerr << "Error: Failed to save a snapshot\n";
	  return false;
	}
    }

  // Incremental branch traces are in snapshot directories. Turn off
  // branch tracing to prevent top-level branch tracing file from
  // being generated since the data will be for the last snapshot and
  // not for the whole run. Same is done for instruction and data line
  // tracing.
  for (auto hartPtr : sysHarts_)
    {
      hartPtr->traceBranches(std::string(), 0);
      std::string emptyPath;
      memory_->enableDataLineTrace(emptyPath);
      memory_->enableInstructionLineTrace(emptyPath);
    }

  return true;
}


template <typename URV>
bool
System<URV>::loadSnapshot(const std::string& snapDir)
{
  using std::cerr;

  if (not Filesystem::is_directory(snapDir))
    {
      cerr << "Error: Path is not a snapshot directory: " << snapDir << '\n';
      return false;
    }

  if (hartCount_ == 0)
    {
      cerr << "Error: System::loadSnapshot: System with no harts\n";
      return false;
    }

  Filesystem::path dirPath = snapDir;

  // Restore the register values.
  for (auto hartPtr : sysHarts_)
    {
      unsigned ix = hartPtr->sysHartIndex();
      
      std::string name = "registers" + std::to_string(ix);
      
      Filesystem::path regPath = dirPath / name;
      bool missing = not Filesystem::is_regular_file(regPath);
      if (missing and ix == 0 and hartCount_ == 1)
	{
	  // Support legacy snapshots where hart index was not appended to filename.
	  regPath = dirPath / "registers";
	  missing = not Filesystem::is_regular_file(regPath);
	}
      if (missing)
	{
	  cerr << "Error: Snapshot file does not exists: " << regPath << '\n';
	  return false;
	}

      if (not hartPtr->loadSnapshotRegs(regPath.string()))
	return false;
    }


  Filesystem::path usedBlocksPath = dirPath / "usedblocks";
  std::vector<std::pair<uint64_t,uint64_t>> usedBlocks;
  if (not loadUsedMemBlocks(usedBlocksPath.string(), usedBlocks))
    return false;

  auto& hart0 = *ithHart(0);

  Filesystem::path timePath = dirPath / "time";
  if (not loadTime(timePath.string(), time_))
    {
      std::cerr << "Using instruction count for time\n";
      time_ = hart0.getInstructionCount();  // Legacy snapshots.
    }

  auto& syscall = hart0.getSyscall();
  Filesystem::path mmapPath = dirPath / "mmap";
  if (not syscall.loadMmap(mmapPath.string()))
    return false;

  Filesystem::path memPath = dirPath / "memory";
  if (not memory_->loadSnapshot(memPath.string(), usedBlocks))
    return false;

  // Rearm CLINT timers.
  for (auto hartPtr : sysHarts_)
    {
      uint64_t clintAddr = 0;
      if (hartPtr->hasClint(clintAddr))
	{
	  uint64_t timeCmpAddr = clintAddr + 0x4000 + hartPtr->sysHartIndex() * 8;
	  uint64_t timeCmp = 0;
	  memory_->peek(timeCmpAddr, timeCmp, false);
	  hartPtr->setClintAlarm(timeCmp);
	}
    }

  Filesystem::path fdPath = dirPath / "fd";
  return syscall.loadFileDescriptors(fdPath.string());
}


template class WdRiscv::System<uint32_t>;
template class WdRiscv::System<uint64_t>;
