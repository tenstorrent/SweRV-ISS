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

// Copyright 2024 Tenstorrent Corporation or its affiliates.
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


#include <fstream>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <unistd.h>
#include <dlfcn.h>
#include <csignal>

#include "Session.hpp"
#include "HartConfig.hpp"
#include "Hart.hpp"
#include "Server.hpp"
#include "Interactive.hpp"


#if !defined(SOL_TCP) && defined(IPPROTO_TCP)
#define SOL_TCP IPPROTO_TCP
#endif


using namespace WdRiscv;
using StringVec = std::vector<std::string>;

template <typename URV>
Session<URV>::Session()
{
}


template <typename URV>
std::shared_ptr<System<URV>>
Session<URV>::defineSystem(const Args& args, const HartConfig& config)
{
  // Collect primary configuration parameters.
  unsigned hartsPerCore = 1;
  unsigned coreCount = 1;
  size_t pageSize = UINT64_C(4)*1024;
  size_t memorySize = size_t(1) << 32;  // 4 gigs

  if (not getPrimaryConfigParameters(args, config, hartsPerCore, coreCount,
                                     pageSize, memorySize))
    return nullptr;

  checkAndRepairMemoryParams(memorySize, pageSize);

  if (args.hexFiles.empty() and args.expandedTargets.empty()
      and args.binaryFiles.empty() and args.kernelFile.empty()
      and not args.interactive)
    {
      std::cerr << "No program file specified.\n";
      return nullptr;
    }

  // Create cores & harts.
  unsigned hartIdOffset = hartsPerCore;
  config.getHartIdOffset(hartIdOffset);
  if (hartIdOffset < hartsPerCore)
    {
      std::cerr << "Invalid core_hart_id_offset: " << hartIdOffset
                << ",  must be greater than harts_per_core: " << hartsPerCore << '\n';
      return nullptr;
    }

  system_ = std::make_shared<System<URV>> (coreCount, hartsPerCore, hartIdOffset,
					   memorySize, pageSize);
  assert(system_ -> hartCount() == coreCount*hartsPerCore);
  assert(system_ -> hartCount() > 0);

  return system_;
}


template <typename URV>
bool
Session<URV>::configureSystem(const Args& args, const HartConfig& config)
{
  if (not system_)
    return false;

  auto& system = *system_;

  // Configure harts. Define callbacks for non-standard CSRs.
  bool userMode = args.isa.find_first_of("uU") != std::string::npos;
  if (not config.configHarts(system, userMode, args.verbose))
    if (not args.interactive)
      return false;

  // Configure memory.
  if (not config.configMemory(system, args.unmappedElfOk))
    return false;

  if (not args.pciDevs.empty())
    if (not system.addPciDevices(args.pciDevs))
      return false;

  if (not args.dataLines.empty())
    system.enableDataLineTrace(args.dataLines);
  if (not args.instrLines.empty())
    system.enableInstructionLineTrace(args.instrLines);

  bool newlib = false, linux = false;
  checkForNewlibOrLinux(args, newlib, linux);
  bool clib = newlib or linux;
  bool updateMisa = clib and not config.hasCsrConfig("misa");

  std::string isa;
  if (not determineIsa(config, args, clib, isa))
    return false;

  if (not openUserFiles(args))
    return false; 

  for (unsigned i = 0; i < system.hartCount(); ++i)
    {
      auto& hart = *(system_ -> ithHart(i));
      hart.setConsoleOutput(consoleOut_);
      hart.enableBasicBlocks(bblockFile_, args.bblockInsts);
      hart.enableNewlib(newlib);
      hart.enableLinux(linux);
      if (not isa.empty())
	if (not hart.configIsa(isa, updateMisa))
	  return false;
      hart.reset();
    }

  for (unsigned i = 0; i < system.hartCount(); ++i)
    if (not applyCmdLineArgs(args, *system.ithHart(i), config, clib))
      if (not args.interactive)
	return false;

  if (not args.loadFrom.empty())
    if (not system.loadSnapshot(args.loadFrom))
      return false;

  // Set instruction count limit.
  if (args.instCountLim)
    for (unsigned i = 0; i < system.hartCount(); ++i)
      {
	auto& hart = *system.ithHart(i);
	uint64_t count = args.relativeInstCount? hart.getInstructionCount() : 0;
	count += *args.instCountLim;
	hart.setInstructionCountLimit(count);
      }

  if (not args.initStateFile.empty())
    {
      if (system.hartCount() > 1)
	{
	  std::cerr << "Initial line-state report (--initstate) valid only when hart count is 1\n";
	  return false;
	}
      auto& hart0 = *system.ithHart(0);
      hart0.setInitialStateFile(initStateFile_);
    }

  return true;
}



template <typename URV>
bool
Session<URV>::getPrimaryConfigParameters(const Args& args, const HartConfig& config,
					 unsigned& hartsPerCore, unsigned& coreCount,
					 size_t& pageSize, size_t& memorySize)
{
  config.getHartsPerCore(hartsPerCore);
  if (args.hasHarts)
    hartsPerCore = args.harts;
  if (hartsPerCore == 0 or hartsPerCore > 32)
    {
      std::cerr << "Unsupported hart count: " << hartsPerCore;
      std::cerr << " (1 to 32 currently supported)\n";
      return false;
    }

  config.getCoreCount(coreCount);
  if (args.hasCores)
    coreCount = args.cores;
  if (coreCount == 0 or coreCount > 32)
    {
      std::cerr << "Unsupported core count: " << coreCount;
      std::cerr << " (1 to 32 currently supported)\n";
      return false;
    }

  // Determine simulated memory size. Default to 4 gigs.
  // If running a 32-bit machine (pointer size = 32 bits), try 2 gigs.
  if (memorySize == 0)
    memorySize = size_t(1) << 31;  // 2 gigs
  config.getMemorySize(memorySize);
  if (args.memorySize)
    memorySize = *args.memorySize;

  if (not config.getPageSize(pageSize))
    pageSize = args.pageSize;

  return true;
}


template <typename URV>
bool
Session<URV>::checkAndRepairMemoryParams(size_t& memSize, size_t& pageSize)
{
  bool ok = true;

  unsigned logPageSize = static_cast<unsigned>(std::log2(pageSize));
  size_t p2PageSize = size_t(1) << logPageSize;
  if (p2PageSize != pageSize)
    {
      std::cerr << "Memory page size (0x" << std::hex << pageSize << ") "
		<< "is not a power of 2 -- using 0x" << p2PageSize << '\n'
		<< std::dec;
      pageSize = p2PageSize;
      ok = false;
    }

  if (pageSize < 64)
    {
      std::cerr << "Page size (" << pageSize << ") is less than 64. Using 64.\n";
      pageSize = 64;
      ok = false;
    }

  if (memSize < pageSize)
    {
      std::cerr << "Memory size (0x" << std::hex << memSize << ") "
		<< "smaller than page size (0x" << pageSize << ") -- "
                << "using 0x" << pageSize << " as memory size\n" << std::dec;
      memSize = pageSize;
      ok = false;
    }

  size_t pageCount = memSize / pageSize;
  if (pageCount * pageSize != memSize)
    {
      size_t newSize = (pageCount + 1) * pageSize;
      if (newSize == 0)
	newSize = (pageCount - 1) * pageSize;  // Avoid overflow
      std::cerr << "Memory size (0x" << std::hex << memSize << ") is not a "
		<< "multiple of page size (0x" << pageSize << ") -- "
		<< "using 0x" << newSize << '\n' << std::dec;
      memSize = newSize;
      ok = false;
    }

  return ok;
}


template<typename URV>
bool
Session<URV>::openUserFiles(const Args& args)
{
  traceFiles_.resize(system_ -> hartCount());

  unsigned ix = 0;
  for (auto& traceFile : traceFiles_)
    {
      size_t len = args.traceFile.size();
      doGzip_ = len > 3 and args.traceFile.substr(len-3) == ".gz";

      if (not args.traceFile.empty())
        {
          std::string name = args.traceFile;
          if (args.logPerHart)
            {
              if (not doGzip_)
                name.append(std::to_string(ix));
              else
                name.insert(len - 3, std::to_string(ix));
            }

          if ((ix == 0) || args.logPerHart)
	    {
	      if (doGzip_)
		{
		  std::string cmd = "/usr/bin/gzip -c > ";
		  cmd += name;
		  traceFile = popen(cmd.c_str(), "w");
		}
	      else
		traceFile = fopen(name.c_str(), "w");
	    }
          else
	    traceFile = traceFiles_.at(0);   // point the same File pointer to each hart

          if (not traceFile)
            {
              std::cerr << "Failed to open trace file '" << name
                        << "' for output\n";
              return false;
            }
        }

      if (args.trace and traceFile == nullptr)
        traceFile = stdout;
      ++ix;
    }

  if (not args.commandLogFile.empty())
    {
      commandLog_ = fopen(args.commandLogFile.c_str(), "w");
      if (not commandLog_)
	{
	  std::cerr << "Failed to open command log file '"
		    << args.commandLogFile << "' for output\n";
	  return false;
	}
      setlinebuf(commandLog_);  // Make line-buffered.
    }

  if (not args.consoleOutFile.empty())
    {
      consoleOut_ = fopen(args.consoleOutFile.c_str(), "w");
      if (not consoleOut_)
	{
	  std::cerr << "Failed to open console output file '"
		    << args.consoleOutFile << "' for output\n";
	  return false;
	}
    }

  if (not args.bblockFile.empty())
    {
      bblockFile_ = fopen(args.bblockFile.c_str(), "w");
      if (not bblockFile_)
	{
	  std::cerr << "Failed to open basic block file '"
		    << args.bblockFile << "' for output\n";
	  return false;
	}
    }

  if (not args.initStateFile.empty())
    {
      initStateFile_ = fopen(args.initStateFile.c_str(), "w");
      if (not initStateFile_)
	{
	  std::cerr << "Failed to open init state file '"
		    << args.initStateFile << "' for output\n";
	  return false;
	}
    }

  return true;
}


template<typename URV>
void
Session<URV>::closeUserFiles()
{
  if (consoleOut_ and consoleOut_ != stdout)
    fclose(consoleOut_);
  consoleOut_ = nullptr;

  FILE* prev = nullptr;
  for (auto& traceFile : traceFiles_)
    {
      if (traceFile and traceFile != stdout and traceFile != prev)
	{
	  if (doGzip_)
	    pclose(traceFile);
	  else
	    fclose(traceFile);
	}
      prev = traceFile;
      traceFile = nullptr;
    }

  if (commandLog_ and commandLog_ != stdout)
    fclose(commandLog_);
  commandLog_ = nullptr;

  if (bblockFile_ and bblockFile_ != stdout)
    fclose(bblockFile_);
  bblockFile_ = nullptr;

  if (initStateFile_ and initStateFile_ != stdout)
    fclose(initStateFile_);
  initStateFile_ = nullptr;
}


template<typename URV>
void
Session<URV>::checkForNewlibOrLinux(const Args& args, bool& newlib, bool& linux)
{
  if (args.raw)
    {
      if (args.newlib or args.linux)
	std::cerr << "Raw mode not compatible with newlib/linux. Sticking"
		  << " with raw mode.\n";
      return;
    }

  newlib = args.newlib;
  linux = args.linux;

  if (linux or newlib)
    return;  // Emulation preference already set by user.

  for (auto target : args.expandedTargets)
    {
      auto elfPath = target.at(0);
      if (not linux)
	linux = (Memory::isSymbolInElfFile(elfPath, "__libc_early_init") or
		 Memory::isSymbolInElfFile(elfPath, "__dladdr"));
      if (not newlib)
	newlib = Memory::isSymbolInElfFile(elfPath, "__call_exitprocs");

      if (linux and newlib)
	break;
    }

  if (linux and args.verbose)
    std::cerr << "Detected Linux symbol in ELF\n";

  if (newlib and args. verbose)
    std::cerr << "Detected Newlib symbol in ELF\n";

  if (newlib and linux)
    {
      std::cerr << "Fishy: Both Newlib and Linux symbols present in "
		<< "ELF file(s). Doing Linux emulation.\n";
      newlib = false;
    }
}


template<typename URV>
bool
Session<URV>::determineIsa(const HartConfig& config, const Args& args, bool clib,
			   std::string& isa)
{
  isa.clear();

  if (not args.isa.empty() and args.elfisa)
    std::cerr << "Warning: Both --isa and --elfisa present: Using --isa\n";

  isa = args.isa;

  if (isa.empty() and args.elfisa)
    if (not getElfFilesIsaString(args, isa))
      return false;

  if (isa.empty())
    {
      // No command line ISA. Use config file.
      config.getIsa(isa);
    }

  if (isa.empty() and clib)
    {
      if (args.verbose)
	std::cerr << "No ISA specified, using i/m/a/c/f/d/v extensions for newlib/linux\n";
      isa = "imcafdv";
    }

  if (isa.empty() and not args.raw)
    {
      if (args.verbose)
	std::cerr << "No ISA specified: Defaulting to imac\n";
      isa = "imac";
    }

  return true;
}



template<typename URV>
bool
Session<URV>::getElfFilesIsaString(const Args& args, std::string& isaString)
{
  StringVec archTags;

  unsigned errors = 0;
  
  for (const auto& target : args.expandedTargets)
    {
      const auto& elfFile = target.front();
      if (not Memory::collectElfRiscvTags(elfFile, archTags))
        errors++;
    }

  if (archTags.empty())
    return errors == 0;

  const std::string& ref = archTags.front();

  for (const auto& tag : archTags)
    if (tag != ref)
      std::cerr << "Warning different ELF files have different ISA strings: "
		<< tag << " and " << ref << '\n';

  isaString = ref;

  if (args.verbose)
    std::cerr << "ISA string from ELF file(s): " << isaString << '\n';

  return errors == 0;
}


/// Set stack pointer to a reasonable value for Linux/Newlib.
template<typename URV>
static
void
sanitizeStackPointer(Hart<URV>& hart, bool verbose)
{
  // Set stack pointer to the 128 bytes below end of memory.
  size_t memSize = hart.getMemorySize();
  if (memSize > 128)
    {
      size_t spValue = memSize - 128;
      if (verbose)
	std::cerr << "Setting stack pointer to 0x" << std::hex << spValue
		  << std::dec << " for newlib/linux\n";
      hart.pokeIntReg(IntRegNumber::RegSp, spValue);
    }
}


/// Apply register initialization specified on the command line.
template<typename URV>
static
bool
applyCmdLineRegInit(const Args& args, Hart<URV>& hart)
{
  bool ok = true;

  URV hartIx = hart.sysHartIndex();

  for (const auto& regInit : args.regInits)
    {
      // Each register initialization is a string of the form reg=val or hart:reg=val
      std::vector<std::string> tokens;
      boost::split(tokens, regInit, boost::is_any_of("="), boost::token_compress_on);
      if (tokens.size() != 2)
	{
	  std::cerr << "Invalid command line register initialization: " << regInit << '\n';
	  ok = false;
	  continue;
	}

      std::string regName = tokens.at(0);
      const std::string& regVal = tokens.at(1);

      bool specificHart = false;
      unsigned ix = 0;
      size_t colonIx = regName.find(':');
      if (colonIx != std::string::npos)
	{
	  std::string hartStr = regName.substr(0, colonIx);
	  regName = regName.substr(colonIx + 1);
	  if (not Args::parseCmdLineNumber("hart", hartStr, ix))
	    {
	      std::cerr << "Invalid command line register initialization: " << regInit << '\n';
	      ok = false;
	      continue;
	    }
	  specificHart = true;
	}

      URV val = 0;
      if (not Args::parseCmdLineNumber("register", regVal, val))
	{
	  ok = false;
	  continue;
	}

      if (specificHart and ix != hartIx)
	continue;

      unsigned reg = 0;
      Csr<URV>* csr = nullptr;

      if (hart.findIntReg(regName, reg))
	hart.pokeIntReg(reg, val);
      else if (hart.findFpReg(regName, reg))
	hart.pokeFpReg(reg, val);
      else if ((csr = hart.findCsr(regName)) != nullptr)
	hart.pokeCsr(csr->getNumber(), val);
      else
	{
	  std::cerr << "Invalid --setreg register: " << regName << '\n';
	  ok = false;
	  continue;
	}

      if (args.verbose)
	std::cerr << "Setting register " << regName << " to command line "
		  << "value 0x" << std::hex << val << std::dec << '\n';
    }

  return ok;
}


template<typename URV>
bool
Session<URV>::applyCmdLineArgs(const Args& args, Hart<URV>& hart,
			       const HartConfig& config, bool clib)
{
  unsigned errors = 0;

  auto& system = *system_;

  if (clib)  // Linux or Newlib enabled.
    sanitizeStackPointer(hart, args.verbose);

  if (args.toHostSym)
    system.setTohostSymbol(*args.toHostSym);

  if (args.consoleIoSym)
    system.setConsoleIoSymbol(*args.consoleIoSym);

  // Load ELF/HEX/binary files. Entry point of first ELF file sets the start PC unless in
  // raw mode.
  if (hart.sysHartIndex() == 0)
    {
      StringVec paths;
      for (const auto& target : args.expandedTargets)
	paths.push_back(target.at(0));

      if (not system.loadElfFiles(paths, args.raw, args.verbose))
	errors++;

      if (not system.loadHexFiles(args.hexFiles, args.verbose))
	errors++;

      uint64_t offset = 0;
      if (not system.loadBinaryFiles(args.binaryFiles, offset, args.verbose))
	errors++;

      if (not args.kernelFile.empty())
	{
	  // Default kernel file offset. FIX: make a parameter.
	  StringVec files{args.kernelFile};
	  offset = hart.isRv64() ? 0x80200000 : 0x80400000;
	  if (not system.loadBinaryFiles(files, offset, args.verbose))
	    errors++;
	}
    }

  if (not args.instFreqFile.empty())
    hart.enableInstructionFrequency(true);

  if (args.clint)
    {
      uint64_t swAddr = *args.clint;
      uint64_t timerAddr = swAddr + 0x4000;
      uint64_t clintEnd = swAddr + 0xc000;
      config.configClint(system, hart, swAddr, clintEnd, timerAddr);
    }

  uint64_t window = 1000000;
  if (args.branchWindow)
    window = *args.branchWindow;
  if (not args.branchTraceFile.empty())
    hart.traceBranches(args.branchTraceFile, window);

  if (args.logStart)
    hart.setLogStart(*args.logStart);

  if (args.logPerHart or (system.hartCount() == 1))
    hart.setOwnTrace(args.logPerHart or (system.hartCount() == 1));

  if (not args.loadFrom.empty())
    {
      if (not args.stdoutFile.empty() or not args.stderrFile.empty() or
	  not args.stdinFile.empty())
	std::cerr << "Info: Options --stdin/--stdout/--stderr are ignored with --loadfrom\n";
    }
  else
    {
      if (not args.stdoutFile.empty())
	if (not hart.redirectOutputDescriptor(STDOUT_FILENO, args.stdoutFile))
	  errors++;

      if (not args.stderrFile.empty())
	if (not hart.redirectOutputDescriptor(STDERR_FILENO, args.stderrFile))
	  errors++;

      if (not args.stdinFile.empty())
	if (not hart.redirectInputDescriptor(STDIN_FILENO, args.stdinFile))
	  errors++;
    }

  if (args.instCounter)
    hart.setInstructionCount(*args.instCounter);

  // Command line to-host overrides that of ELF and config file.
  if (args.toHost)
    hart.setToHostAddress(*args.toHost);
  if (args.fromHost)
    hart.setFromHostAddress(*args.fromHost);

  // Command-line entry point overrides that of ELF.
  if (args.startPc)
    {
      hart.defineResetPc(*args.startPc);
      hart.pokePc(URV(*args.startPc));
    }

  // Command-line exit point overrides that of ELF.
  if (args.endPc)
    hart.setStopAddress(URV(*args.endPc));

  // Command-line console io address overrides config file.
  if (args.consoleIo)
    hart.setConsoleIo(URV(*args.consoleIo));

  hart.enableConsoleInput(! args.noConInput);

  if (args.interruptor)
    {
      uint64_t addr = *args.interruptor;
      config.configInterruptor(system, hart, addr);
    }

  if (args.syscallSlam)
    hart.defineSyscallSlam(*args.syscallSlam);

  if (args.tracePtw)
    hart.tracePtw(true);

  // Setup periodic external interrupts.
  if (args.alarmInterval)
    {
      // Convert from micro-seconds to processor ticks. Assume a 1
      // ghz-processor.
      uint64_t ticks = (*args.alarmInterval)*1000;
      hart.setupPeriodicTimerInterrupts(ticks);
    }

  if (args.triggers)
    hart.enableTriggers(args.triggers);
  hart.enableGdb(args.gdb);
  if (args.gdbTcpPort.size()>hart.sysHartIndex())
    hart.setGdbTcpPort(args.gdbTcpPort[hart.sysHartIndex()]);
  if (args.counters)
    hart.enablePerformanceCounters(args.counters);
  if (args.abiNames)
    hart.enableAbiNames(args.abiNames);

  // Apply register initialization.
  if (not applyCmdLineRegInit(args, hart))
    errors++;

  // Setup target program arguments.
  if (not args.expandedTargets.empty())
    {
      if (clib)
	{
	  if (args.loadFrom.empty())
	    if (not hart.setTargetProgramArgs(args.expandedTargets.front()))
	      {
		size_t memSize = hart.memorySize();
		size_t suggestedStack = memSize - 4;

		std::cerr << "Failed to setup target program arguments -- stack "
			  << "is not writable\n"
			  << "Try using --setreg sp=<val> to set the stack pointer "
			  << "to a\nwritable region of memory (e.g. --setreg "
			  << "sp=0x" << std::hex << suggestedStack << '\n'
			  << std::dec;
		errors++;
	      }
	}
      else if (args.expandedTargets.front().size() > 1)
	{
	  std::cerr << "Warning: Target program options present which requires\n"
		    << "         the use of --newlib/--linux. Options ignored.\n";
	}
    }

  if (args.csv)
    hart.enableCsvLog(args.csv);

  if (args.logStart)
    hart.setLogStart(*args.logStart);

  if (args.mcm)
    {
      unsigned mcmLineSize = 64;
      config.getMcmLineSize(mcmLineSize);
      if (args.mcmls)
	mcmLineSize = *args.mcmls;
      bool checkAll = false;
      config.getMcmCheckAll(checkAll);
      if (args.mcmca)
	checkAll = true;
      if (not system.enableMcm(mcmLineSize, checkAll))
	errors++;
    }

  if (args.perfApi)
    if (not system.enablePerfApi())
      errors++;

  if (not args.snapshotPeriods.empty())
    {
      auto periods = args.snapshotPeriods;
      std::sort(periods.begin(), periods.end());
      if (std::find(periods.begin(), periods.end(), 0)
                      != periods.end())
        {
          std::cerr << "Snapshot periods of 0 are ignored\n";
          periods.erase(std::remove(periods.begin(), periods.end(), 0), periods.end());
        }

      auto it = std::unique(periods.begin(), periods.end());
      if (it != periods.end())
        {
          periods.erase(it, periods.end());
          std::cerr << "Duplicate snapshot periods not supported, removed duplicates\n";
        }
    }

  if (not args.snapshotDir.empty())
    system.setSnapshotDir(args.snapshotDir);

  if (args.tlbSize)
    {
      size_t size = *args.tlbSize;
      if ((size & (size-1)) != 0)
	{
	  std::cerr << "TLB size must be a power of 2\n";
	  errors++;
	}
      else
	hart.setTlbSize(size);
    }

  return errors == 0;
}


template<typename URV>
bool
Session<URV>::runServer(const std::string& serverFile)
{
  auto& system = *system_;
  auto traceFile = traceFiles_.at(0);
  auto commandLog = commandLog_;

  std::array<char, 1024> hostName = {};
  if (gethostname(hostName.data(), hostName.size()) != 0)
    {
      std::cerr << "Failed to obtain name of this computer\n";
      return false;
    }

  int soc = socket(AF_INET, SOCK_STREAM, 0);
  if (soc < 0)
    {
      std::array<char, 512> buffer;
      char* p = buffer.data();
#ifdef __APPLE__
      strerror_r(errno, buffer.data(), buffer.size());
#else
      p = strerror_r(errno, buffer.data(), buffer.size());
#endif
      std::cerr << "Failed to create socket: " << p << '\n';
      return -1;
    }

  int one = 1;
  setsockopt(soc, SOL_TCP, TCP_NODELAY, &one, sizeof(one));

  sockaddr_in serverAddr;
  memset(&serverAddr, 0, sizeof(serverAddr));
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  serverAddr.sin_port = htons(0);

  if (bind(soc, (sockaddr*) &serverAddr, sizeof(serverAddr)) < 0)
    {
      perror("Socket bind failed");
      return false;
    }

  if (listen(soc, 1) < 0)
    {
      perror("Socket listen failed");
      return false;
    }

  sockaddr_in socAddr;
  socklen_t socAddrSize = sizeof(socAddr);
  socAddr.sin_family = AF_INET;
  socAddr.sin_port = 0;
  if (getsockname(soc, (sockaddr*) &socAddr,  &socAddrSize) == -1)
    {
      perror("Failed to obtain socket information");
      return false;
    }

  {
    std::ofstream out(serverFile);
    if (not out.good())
      {
	std::cerr << "Failed to open file '" << serverFile << "' for output\n";
	return false;
      }
    out << hostName.data() << ' ' << ntohs(socAddr.sin_port) << std::endl;
  }

  sockaddr_in clientAddr;
  socklen_t clientAddrSize = sizeof(clientAddr);
  int newSoc = accept(soc, (sockaddr*) & clientAddr, &clientAddrSize);
  if (newSoc < 0)
    {
      perror("Socket accept failed");
      return false;
    }

  one = 1;
  setsockopt(newSoc, SOL_TCP, TCP_NODELAY, &one, sizeof(one));

  bool ok = true;

  try
    {
      Server<URV> server(system);
      ok = server.interact(newSoc, traceFile, commandLog);
    }
  catch(...)
    {
      ok = false;
    }

  close(newSoc);
  close(soc);

  return ok;
}


template<typename URV>
bool
Session<URV>::runServerShm(const std::string& serverFile)
{
  auto& system = *system_;
  auto traceFile = traceFiles_.at(0);
  auto commandLog = commandLog_;

  std::string path = "/" + serverFile;
  int fd = shm_open(path.c_str(), O_RDWR | O_CREAT, S_IRWXU | S_IRWXG | S_IRWXO);
  if (fd < 0)
    {
      perror("Failed to open shared memory file");
      return false;
    }
  if (ftruncate(fd, 4096) < 0)
    {
      perror("Failed ftruncate on shared memory file");
      return false;
    }

  char* shm = (char*) mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (shm == MAP_FAILED)
    {
      perror("Failed mmap");
      return false;
    }

  bool ok = true;

  try
    {
      Server<URV> server(system);
      ok = server.interact(std::span<char>(shm, 4096), traceFile, commandLog);
    }
  catch(...)
    {
      ok = false;
    }

  if (munmap(shm, 4096) < 0)
    {
      perror("Failed to unmap");
      return false;
    }

  close(fd);

  if (shm_unlink(path.c_str()) < 0)
    {
      perror("Failed shm unlink");
      return false;
    }
  return ok;
}


// In interactive mode, keyboard interrupts (typically control-c) are
// ignored.
static void
kbdInterruptHandler(int)
{
  std::cerr << "keyboard interrupt\n";
}


template<typename URV>
bool
Session<URV>::runInteractive()
{
  // Ignore keyboard interrupt for most commands. Long running
  // commands will enable keyboard interrupts while they run.
  struct sigaction newAction;
  sigemptyset(&newAction.sa_mask);
  newAction.sa_flags = 0;
  newAction.sa_handler = kbdInterruptHandler;
  sigaction(SIGINT, &newAction, nullptr);

  Interactive interactive(*system_);
  return interactive.interact(traceFiles_.at(0), commandLog_);
}


//NOLINTBEGIN(bugprone-reserved-identifier, cppcoreguidelines-avoid-non-const-global-variables)
extern void (*__tracerExtension)(void*);
void (*__tracerExtensionInit)() = nullptr;
extern "C" {
  std::string tracerExtensionArgs;
}
//NOLINTEND(bugprone-reserved-identifier, cppcoreguidelines-avoid-non-const-global-variables)

template <typename URV>
static
bool
loadTracerLibrary(const std::string& tracerLib)
{
  if (tracerLib.empty())
    return true;

  std::vector<std::string> result;
  boost::split(result, tracerLib, boost::is_any_of(":"));
  assert(not result.empty());

  auto* soPtr = dlopen(result[0].c_str(), RTLD_NOW);
  if (not soPtr)
    {
      std::cerr << "Error: Failed to load shared library " << dlerror() << '\n';
      return false;
    }

  if (result.size() == 2)
    tracerExtensionArgs = result[1];

  std::string entry("tracerExtension");
  entry += sizeof(URV) == 4 ? "32" : "64";

  __tracerExtension = reinterpret_cast<void (*)(void*)>(dlsym(soPtr, entry.c_str()));
  if (not __tracerExtension)
    {
      std::cerr << "Error: Could not find symbol tracerExtension in " << tracerLib << '\n';
      return false;
    }

  entry = "tracerExtensionInit";
  entry += sizeof(URV) == 4 ? "32" : "64";

  __tracerExtensionInit = reinterpret_cast<void (*)()>(dlsym(soPtr, entry.c_str()));
  if (__tracerExtensionInit)
    __tracerExtensionInit();

  return true;
}


template<typename URV>
bool
Session<URV>::run(const Args& args)
{
  auto& system = *system_;

  if (not loadTracerLibrary<URV>(args.tracerLib))
    return false;

  bool serverMode = not args.serverFile.empty();
  if (serverMode)
    {
      if (args.shm)
	return runServerShm(args.serverFile);
      return runServer(args.serverFile);
    }

  if (args.interactive)
    return runInteractive();

  if (not args.snapshotPeriods.empty())
    return system.snapshotRun(traceFiles_, args.snapshotPeriods);

  bool waitAll = not args.quitOnAnyHart;
  uint64_t stepWindow = args.deterministic.value_or(0);
  unsigned seed = args.seed.value_or(time(NULL));
  srand(seed);

  if (stepWindow)
    std::cout << "Deterministic multi-hart run with seed: " << seed
	      << " and steps distribution between 1 and " << stepWindow << "\n";

  return system.batchRun(traceFiles_, waitAll, stepWindow);
}


static bool
getXlenFromElfFile(const Args& args, unsigned& xlen)
{
  if (args.expandedTargets.empty())
    return false;

  // Get the length from the first target.
  const auto& elfPath = args.expandedTargets.front().front();
  bool is32 = false, is64 = false, isRiscv = false;
  if (not Memory::checkElfFile(elfPath, is32, is64, isRiscv))
    return false;  // ELF does not exist.

  if (not is32 and not is64)
    return false;

  if (is32 and is64)
    {
      std::cerr << "Error: ELF file '" << elfPath << "' has both 32 and 64-bit class\n";
      return false;
    }

  xlen = is32 ? 32 : 64;

  if (args.verbose)
    std::cerr << "Setting xlen to " << xlen << " based on ELF file " <<  elfPath << '\n';
  return true;
}


template<typename URV>
unsigned
Session<URV>::determineRegisterWidth(const Args& args, const HartConfig& config)
{
  unsigned isaLen = 0;
  if (not args.isa.empty())
    {
      if (args.isa.starts_with("rv32"))
	isaLen = 32;
      else if (args.isa.starts_with("rv64"))
	isaLen = 64;
      else
	std::cerr << "Command line --isa tag does not start with rv32/rv64\n";
    }

  // 1. If --isa specifies xlen, go with that.
  if (isaLen)
    {
      if (args.verbose)
        std::cerr << "Setting xlen from --isa: " << isaLen << "\n";
      return isaLen;
    }

  // 2. If config file has isa tag, go with that.
  unsigned xlen = 32;
  if (config.getXlen(xlen))
    {
      if (args.verbose)
	std::cerr << "Setting xlen from config file: " << xlen << "\n";
      return xlen;
    }

  // 3. Get xlen from ELF file.
  if (getXlenFromElfFile(args, xlen))
    {
      if (args.verbose)
	std::cerr << "Setting xlen from ELF file: " << xlen << "\n";
      return xlen;
    }

  if (args.verbose)
    std::cerr << "Using default for xlen: " << xlen << "\n";
  
  return xlen;
}


template class WdRiscv::Session<uint32_t>;
template class WdRiscv::Session<uint64_t>;
