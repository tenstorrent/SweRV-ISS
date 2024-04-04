#include "Session.hpp"
#include "HartConfig.hpp"

using namespace WdRiscv;


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
  assert(system_ ->hartCount() == coreCount*hartsPerCore);
  assert(system_ ->hartCount() > 0);

  return system_;
}


template <typename URV>
bool
Session<URV>::configureSystem(const Args& args, const HartConfig& config)
{
  if (not system_)
    return false;

  // Configure harts. Define callbacks for non-standard CSRs.
  bool userMode = args.isa.find_first_of("uU") != std::string::npos;
  if (not config.configHarts(*system_, userMode, args.verbose))
    if (not args.interactive)
      return false;

  // Configure memory.
  if (not config.configMemory(*system_, args.unmappedElfOk))
    return false;

  if (not args.pciDevs.empty())
    if (not system_ ->addPciDevices(args.pciDevs))
      return false;

  if (not args.dataLines.empty())
    system_ ->enableDataLineTrace(args.dataLines);
  if (not args.instrLines.empty())
    system_ ->enableInstructionLineTrace(args.instrLines);

  bool newlib = false, linux = false;
  checkForNewlibOrLinux(args, newlib, linux);
  bool clib = newlib or linux;
  bool updateMisa = clib and not config.hasCsrConfig("misa");

  std::string isa;
  if (not determineIsa(config, args, clib, isa))
    return false;

  if (not openUserFiles())
    return false; 

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

  unsigned ix = 0;
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
getElfFilesIsaString(const Args& args, std::string& isaString)
{
  std::vector<std::string> archTags;

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
