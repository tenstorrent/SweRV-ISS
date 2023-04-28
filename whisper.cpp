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
#include <sstream>
#include <thread>
#include <optional>
#include <atomic>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <dlfcn.h>
#include <csignal>
#include "HartConfig.hpp"
#include "WhisperMessage.h"
#include "Hart.hpp"
#include "Core.hpp"
#include "System.hpp"
#include "Server.hpp"
#include "Interactive.hpp"
#include "third_party/nlohmann/json.hpp"
#include "Filesystem.hpp"


using namespace WdRiscv;


/// Return format string suitable for printing an integer of type URV
/// in hexadecimal form.
template <typename URV>
static constexpr
const char*
getHexForm()
{
  if constexpr (sizeof(URV) == 4)
    return "0x%08x";
  if constexpr (sizeof(URV) == 8)
    return "0x%016x";
  if constexpr (sizeof(URV) == 16)
    return "0x%032x";
  return "0x%x";
}


/// Convert the command line string numberStr to a number using
/// strotull and a base of zero (prefixes 0 and 0x are
/// honored). Return true on success and false on failure (string does
/// not represent a number). TYPE is an integer type (e.g
/// uint32_t). Option is the command line option associated with the
/// string and is used for diagnostic messages.
template <typename TYPE>
static
bool
parseCmdLineNumber(const std::string& option,
		   const std::string& numberStr,
		   TYPE& number)
{
  std::string str = numberStr;
  bool good = not str.empty();
  uint64_t scale = 1;
  if (good)
    {
      char suffix = str.back();
      if (suffix == 'k')
        scale = 1024;
      else if (suffix == 'm')
        scale = 1024*1024;
      else if (suffix == 'g')
        scale = 1024*1024*1024;
      if (scale != 1)
        {
          str = str.substr(0, str.length() - 1);
          if (str.empty())
            good = false;
        }
    }

  if (good)
    {
      typedef typename std::make_signed_t<TYPE> STYPE;

      char* end = nullptr;
      
      bool bad = false;

      if (std::is_same<TYPE, STYPE>::value)
        {
          int64_t val = strtoll(str.c_str(), &end, 0) * scale;
          number = static_cast<TYPE>(val);
          bad = val != number;
        }
      else
        {
          uint64_t val = strtoull(str.c_str(), &end, 0) * scale;
          number = static_cast<TYPE>(val);
          bad = val != number;
        }

      if (bad)
	{
	  std::cerr << "parseCmdLineNumber: Number too large: " << numberStr
		    << '\n';
	  return false;
	}
      if (end and *end)
	good = false;  // Part of the string are non parseable.
    }

  if (not good)
    std::cerr << "Invalid command line " << option << " value: " << numberStr
	      << '\n';
  return good;
}


/// Aapter for the parseCmdLineNumber for optionals.
template <typename TYPE>
static
bool
parseCmdLineNumber(const std::string& option,
		   const std::string& numberStr,
		   std::optional<TYPE>& number)
{
  TYPE n;
  if (not parseCmdLineNumber(option, numberStr, n))
    return false;
  number = n;
  return true;
}


typedef std::vector<std::string> StringVec;
typedef std::vector<uint64_t> Uint64Vec;


/// Hold values provided on the command line.
struct Args
{
  StringVec   hexFiles;                  // Hex files to be loaded into simulator memory.
  StringVec   binaryFiles;               // Binary files to be loaded into simulator memory.
  std::string traceFile;                 // Log of state change after each instruction.
  std::string commandLogFile;            // Log of interactive or socket commands.
  std::string consoleOutFile;            // Console io output file.
  std::string serverFile;                // File in which to write server host and port.
  std::string instFreqFile;              // Instruction frequency file.
  std::string configFile;                // Configuration (JSON) file.
  std::string bblockFile;                // Basci block file.
  std::string attFile;                   // Address translation file.
  std::string branchTraceFile;           // Branch trace file.
  std::string tracerLib;                 // Path to tracer extension shared library.
  std::string isa;
  std::string snapshotDir = "snapshot";  // Dir prefix for saving snapshots
  std::string loadFrom;                  // Directory for loading a snapshot
  std::string stdoutFile;                // Redirect target program stdout to this.
  std::string stderrFile;                // Redirect target program stderr to this.
  std::string stdinFile;                 // Redirect target program stdin to this.
  std::string dataLines;                 // Output file for data address line tracing.
  std::string instrLines;                // Output file for instruction address line tracing.
  std::string initStateFile;             // Output file for inital state of memory lines used in run.
  std::string kernelFile;                // Load kernel image at address.
  std::string testSignatureFile;         // Output signature file to score riscv-arch-test tests
  StringVec   regInits;                  // Initial values of regs
  StringVec   targets;                   // Target (ELF file) programs and associated
                                         // program options to be loaded into simulator
                                         // memory. Each target plus args is one string.
  StringVec   isaVec;                    // Isa string split around _ with rv32/rv64 prefix removed.
  std::string targetSep = " ";           // Target program argument separator.

  std::optional<std::string> toHostSym;
  std::optional<std::string> consoleIoSym;

  // Ith item is a vector of strings representing ith target and its args.
  std::vector<StringVec> expandedTargets;

  std::optional<uint64_t> startPc;
  std::optional<uint64_t> endPc;
  std::optional<uint64_t> toHost;
  std::optional<uint64_t> fromHost;
  std::optional<uint64_t> consoleIo;
  std::optional<uint64_t> instCountLim;
  std::optional<uint64_t> memorySize;
  Uint64Vec snapshotPeriods;
  std::optional<uint64_t> alarmInterval;
  std::optional<uint64_t> clint;  // Core-local-interrupt (Clint) mem mapped address
  std::optional<uint64_t> interruptor; // Interrupt generator mem mapped address
  std::optional<uint64_t> syscallSlam;
  std::optional<uint64_t> instCounter;
  std::optional<uint64_t> branchWindow_;
  std::optional<unsigned> mcmls;

  unsigned regWidth = 32;
  unsigned harts = 1;
  unsigned cores = 1;
  unsigned pageSize = 4*1024;
  uint64_t bblockInsts = ~uint64_t(0);

  bool help = false;
  bool hasRegWidth = false;
  bool hasHarts = false;
  bool hasCores = false;
  bool trace = false;
  bool interactive = false;
  bool verbose = false;
  bool version = false;
  bool traceLdSt = false;  // Trace ld/st data address if true.
  bool csv = false;        // Log files in CSV format when true.
  bool triggers = false;   // Enable debug triggers when true.
  bool counters = false;   // Enable performance counters when true.
  bool gdb = false;        // Enable gdb mode when true.
  std::vector<unsigned> gdbTcpPort;        // Enable gdb mode over TCP when port is positive.
  bool abiNames = false;   // Use ABI register names in inst disassembly.
  bool newlib = false;     // True if target program linked with newlib.
  bool linux = false;      // True if target program linked with Linux C-lib.
  bool raw = false;        // True if bare-metal program (no linux no newlib).
  bool elfisa = false;     // Use ELF file RISCV architecture tags to set MISA if true.
  bool fastExt = false;    // True if fast external interrupt dispatch enabled.
  bool unmappedElfOk = false;
  bool mcm = false;        // Memory consistency checks
  bool mcmca = false;      // Memory consistency checks: check all bytes of merge buffer
  bool quitOnAnyHart = false;    // True if run quits when any hart finishes.
  bool noConInput = false;       // If true console io address is not used for input (ld).
  bool relativeInstCount = false;
  bool tracePtw = false;   // Enable printing of page table walk info in log.
  bool shm = false;        // Enable shared memory IPC for server mode (default is socket).

  // Expand each target program string into program name and args.
  void expandTargets();
};


void
Args::expandTargets()
{
  this->expandedTargets.clear();
  for (const auto& target : this->targets)
    {
      StringVec tokens;
      boost::split(tokens, target, boost::is_any_of(this->targetSep),
		   boost::token_compress_on);
      this->expandedTargets.push_back(tokens);
    }
}


static
void
printVersion()
{
  unsigned version = 1;
  unsigned subversion = 801;
  std::cout << "Version " << version << "." << subversion << " compiled on "
	    << __DATE__ << " at " << __TIME__ << '\n';
#ifdef GIT_SHA
  #define xstr(x) str(x)
  #define str(x) #x
  std::cout << "Git SHA: " << xstr(GIT_SHA) << '\n';
  #undef str
  #undef xstr
#endif
}


static
bool
collectCommandLineValues(const boost::program_options::variables_map& varMap,
			 Args& args)
{
  bool ok = true;

  if (varMap.count("startpc"))
    {
      auto numStr = varMap["startpc"].as<std::string>();
      if (not parseCmdLineNumber("startpc", numStr, args.startPc))
	ok = false;
    }

  if (varMap.count("endpc"))
    {
      auto numStr = varMap["endpc"].as<std::string>();
      if (not parseCmdLineNumber("endpc", numStr, args.endPc))
	ok = false;
    }

  if (varMap.count("tohost"))
    {
      auto numStr = varMap["tohost"].as<std::string>();
      if (not parseCmdLineNumber("tohost", numStr, args.toHost))
	ok = false;
    }

  if (varMap.count("fromhost"))
    {
      auto numStr = varMap["fromhost"].as<std::string>();
      if (not parseCmdLineNumber("fromhost", numStr, args.fromHost))
	ok = false;
    }

  if (varMap.count("consoleio"))
    {
      auto numStr = varMap["consoleio"].as<std::string>();
      if (not parseCmdLineNumber("consoleio", numStr, args.consoleIo))
	ok = false;
    }

  if (varMap.count("maxinst"))
    {
      auto numStr = varMap["maxinst"].as<std::string>();
      if (not parseCmdLineNumber("maxinst", numStr, args.instCountLim))
	ok = false;
      args.relativeInstCount = not numStr.empty() and numStr.at(0) == '+';
    }

  if (varMap.count("memorysize"))
    {
      auto numStr = varMap["memorysize"].as<std::string>();
      if (not parseCmdLineNumber("memorysize", numStr, args.memorySize))
        ok = false;
    }

  if (varMap.count("tohostsym"))
    args.toHostSym = varMap["tohostsym"].as<std::string>();

  if (varMap.count("consoleiosym"))
    args.consoleIoSym = varMap["consoleiosym"].as<std::string>();

  if (varMap.count("xlen"))
    args.hasRegWidth = true;

  if (varMap.count("cores"))
    args.hasCores = true;

  if (varMap.count("harts"))
    args.hasHarts = true;

  if (varMap.count("alarm"))
    {
      auto numStr = varMap["alarm"].as<std::string>();
      if (not parseCmdLineNumber("alarm", numStr, args.alarmInterval))
        ok = false;
      else if (*args.alarmInterval == 0)
        std::cerr << "Warning: Zero alarm period ignored.\n";
    }

  if (varMap.count("branchwindow"))
    {
      auto numStr = varMap["branchwindow"].as<std::string>();
      if (not parseCmdLineNumber("branchwindow", numStr, args.branchWindow_))
        ok = false;
    }

  if (varMap.count("clint"))
    {
      auto numStr = varMap["clint"].as<std::string>();
      if (not parseCmdLineNumber("clint", numStr, args.clint))
        ok = false;
      else if ((*args.clint & 7) != 0)
        {
          std::cerr << "Error: clint address must be a multiple of 8\n";
          ok = false;
        }
    }

  if (varMap.count("interruptor"))
    {
      auto numStr = varMap["interruptor"].as<std::string>();
      if (not parseCmdLineNumber("interruptor", numStr, args.interruptor))
        ok = false;
      else if ((*args.interruptor & 7) != 0)
        {
          std::cerr << "Error: interruptor address must be a multiple of 8\n";
          ok = false;
        }
    }

  if (varMap.count("syscallslam"))
    {
      auto numStr = varMap["syscallslam"].as<std::string>();
      if (not parseCmdLineNumber("syscallslam", numStr, args.syscallSlam))
        ok = false;
    }

  if (varMap.count("mcmls"))
    {
      auto numStr = varMap["mcmls"].as<std::string>();
      if (not parseCmdLineNumber("mcmls", numStr, args.mcmls))
        ok = false;
    }

  if (varMap.count("instcounter"))
    {
      auto numStr = varMap["instcounter"].as<std::string>();
      if (not parseCmdLineNumber("instcounter", numStr, args.instCounter))
        ok = false;
    }

  if (args.interactive)
    args.trace = true;  // Enable instruction tracing in interactive mode.

  return ok;
}


/// Parse command line arguments. Place option values in args.
/// Return true on success and false on failure. Exists program
/// if --help is used.
static
bool
parseCmdLineArgs(int argc, char* argv[], Args& args)
{
  try
    {
      // Define command line options.
      namespace po = boost::program_options;
      po::options_description desc("options");
      desc.add_options()
	("help,h", po::bool_switch(&args.help),
	 "Produce this message.")
	("log,l", po::bool_switch(&args.trace),
	 "Enable tracing to standard output of executed instructions.")
	("isa", po::value(&args.isa),
	 "Specify instruction set extensions to enable. Supported extensions "
	 "are a, c, d, f, i, m, s and u. Default is imc.")
	("xlen", po::value(&args.regWidth),
	 "Specify register width (32 or 64), defaults to 32")
	("harts", po::value(&args.harts),
	 "Specify number of hardware threads per core (default=1).")
	("cores", po::value(&args.cores),
	 "Specify number of core per system (default=1).")
	("pagesize", po::value(&args.pageSize),
	 "Specify memory page size.")
	("target,t", po::value(&args.targets)->multitoken(),
	 "Target program (ELF file) to load into simulator memory. In "
	 "newlib/Linux emulation mode, program options may follow program name.")
	("targetsep", po::value(&args.targetSep),
	 "Target program argument separator.")
	("hex,x", po::value(&args.hexFiles)->multitoken(),
	 "HEX file to load into simulator memory.")
	("binary,b", po::value(&args.binaryFiles)->multitoken(),
	 "Binary file to load into simulator memory. File path may be suffixed with a colon followed "
	 "by an address (integer) in which case data will be loaded at address as opposed to zero. "
	 " Example: -b file1  -b file2:0x1040")
        ("kernel", po::value(&args.kernelFile),
         "Kernel binary file to load into simulator memory. File will be loaded at 0x400000 for "
        "rv32 or 0x200000 for rv64 unless an explicit addresss is specified after a colon suffix "
        "to the file path.")
        ("testsignature", po::value(&args.testSignatureFile),
         "Produce a signature file used to score tests provided by the riscv-arch-test project.")
	("logfile,f", po::value(&args.traceFile),
	 "Enable tracing to given file of executed instructions. Output is compressed (with /usr/bin/gzip) if file name ends with \".gz\".")
	("csvlog", po::bool_switch(&args.csv),
	 "Enable CSV format for log file.")
	("consoleoutfile", po::value(&args.consoleOutFile),
	 "Redirect console output to given file.")
	("commandlog", po::value(&args.commandLogFile),
	 "Enable logging of interactive/socket commands to the given file.")
	("server", po::value(&args.serverFile),
	 "Interactive server mode. Put server hostname and port in file. If shared memory "
         "is enabled, file is memory mapped filename")
        ("shm", po::bool_switch(&args.shm),
         "Enable shared memory IPC for server mode (default mode uses socket).")
	("startpc,s", po::value<std::string>(),
	 "Set program entry point. If not specified, use entry point of the "
	 "most recently loaded ELF file.")
	("endpc,e", po::value<std::string>(),
	 "Set stop program counter. Simulator will stop once instruction at "
	 "the stop program counter is executed.")
	("tohost", po::value<std::string>(),
	 "Memory address for host target interface (HTIF).")
	("tohostsym", po::value<std::string>(),
	 "ELF symbol to use for setting tohost from ELF file (in the case "
	 "where tohost is not specified on the command line). Default: "
	 "\"tohost\".")
	("fromhost", po::value<std::string>(),
	 "Memory address for host target interface (HTIF).")
	("consoleio", po::value<std::string>(),
	 "Memory address corresponding to console io. Reading/writing "
	 "(lw/lh/lb sw/sh/sb) from given address reads/writes a byte from the "
         "console.")
	("consoleiosym", po::value<std::string>(),
	 "ELF symbol to use as console-io address (in the case where "
         "consoleio is not specified on the command line). Deafult: "
         "\"__whisper_console_io\".")
	("maxinst,m", po::value<std::string>(),
	 "Limit executed instruction count to arg. With a leading plus sign interpret the count as relative to the loaded (from a snapshot) instruction count.")
	("memorysize", po::value<std::string>(),
	 "Memory size (must be a multiple of 4096).")
	("interactive,i", po::bool_switch(&args.interactive),
	 "Enable interactive mode.")
	("traceload", po::bool_switch(&args.traceLdSt),
	 "Enable tracing of load/store instruction data address (deprecated -- now always on).")
	("traceptw", po::bool_switch(&args.tracePtw),
	 "Enable printing of page table walk information in log.")
	("triggers", po::bool_switch(&args.triggers),
	 "Enable debug triggers (triggers are on in interactive and server modes)")
	("counters", po::bool_switch(&args.counters),
	 "Enable performance counters")
	("gdb", po::bool_switch(&args.gdb),
	 "Run in gdb mode enabling remote debugging from gdb (this requires gdb version"
         "8.2 or higher).")
	("gdb-tcp-port", po::value(&args.gdbTcpPort)->multitoken(),
	 	 "TCP port number for gdb; If port num is negative,"
			" gdb will work with stdio (default -1).")
	("profileinst", po::value(&args.instFreqFile),
	 "Report instruction frequency to file.")
        ("att", po::value(&args.attFile),
         "Dump implicit memory accesses associated with page table walk (PTE entries) to file.")
        ("tracebranch", po::value(&args.branchTraceFile),
         "Trace branch instructions to the given file.")
        ("branchwindow", po::value<std::string>(),
         "Trace branches in the last n instructions.")
        ("tracerlib", po::value(&args.tracerLib),
         "Path to tracer extension shared library which should provide C symbol tracerExtension."
         "Optionally include arguments after a colon to be exposed to the shared library "
         "as C symbol tracerExtensionArgs (ex. tracer.so or tracer.so:hello42).")
	("setreg", po::value(&args.regInits)->multitoken(),
	 "Initialize registers. Apply to all harts unless specific prefix "
	 "present (hart is 1 in 1:x3=0xabc). Example: --setreg x1=4 x2=0xff "
	 "1:x3=0xabc")
	("configfile", po::value(&args.configFile),
	 "Configuration file (JSON file defining system features).")
	("bblockfile", po::value(&args.bblockFile),
	 "Basic blocks output stats file.")
	("bblockinterval", po::value(&args.bblockInsts),
	 "Basic block stats are reported even mulitples of given instruction counts and once at end of run.")
	("snapshotdir", po::value(&args.snapshotDir),
	 "Directory prefix for saving snapshots.")
	("snapshotperiod", po::value(&args.snapshotPeriods)->multitoken(),
	 "Snapshot period: Save snapshot using snapshotdir every so many instructions. "
         "Specifying multiple periods will only save a snapshot on first instance (not periodic).")
	("loadfrom", po::value(&args.loadFrom),
	 "Snapshot directory from which to restore a previously saved (snapshot) state.")
	("stdout", po::value(&args.stdoutFile),
	 "Redirect standard output of newlib/Linux target program to this.")
	("stderr", po::value(&args.stderrFile),
	 "Redirect standard error of newlib/Linux target program to this.")
	("stdin", po::value(&args.stdinFile),
	 "Redirect standard input of newlib/Linux target program to this.")
	("datalines", po::value(&args.dataLines),
	 "Generate data line address trace to the given file.")
	("instrlines", po::value(&args.instrLines),
	 "Generate instruction line address trace to the given file.")
	("initstate", po::value(&args.initStateFile),
	 "Generate to given file the initial state of accessed memory lines.")
	("abinames", po::bool_switch(&args.abiNames),
	 "Use ABI register names (e.g. sp instead of x2) in instruction disassembly.")
	("newlib", po::bool_switch(&args.newlib),
	 "Emulate (some) newlib system calls. Done automatically if newlib "
         "symbols are detected in the target ELF file.")
	("linux", po::bool_switch(&args.linux),
	 "Emulate (some) Linux system calls. Done automatically if Linux "
         "symbols are detected in the target ELF file.")
	("raw", po::bool_switch(&args.raw),
	 "Bare metal mode: Disble emulation of Linux/newlib system call emulation "
         "even if Linux/newlib symbols detected in the target ELF file.")
	("elfisa", po::bool_switch(&args.elfisa),
	 "Configure reset value of MISA according to the RISCV architecture tag(s) "
         "encoded into the laoded ELF file(s) if any.")
	("fastext", po::bool_switch(&args.fastExt),
	 "Enable fast external interrupt dispatch.")
	("unmappedelfok", po::bool_switch(&args.unmappedElfOk),
	 "Do not flag as error ELF file sections targeting unmapped "
         " memory.")
	("alarm", po::value<std::string>(),
	 "External interrupt period in micro-seconds: Convert arg to an "
         "instruction count, n, assuming a 1ghz clock, and force an external "
         " interrupt every n instructions. No-op if arg is zero.")
        ("softinterrupt", po::value<std::string>(),
         "Address of memory mapped word(s) controlling software interrupts. In "
         "an n-hart system, words at addresses a, a+4, ... a+(n-1)*4 "
         "are associated with the n harts (\"a\" being the address "
         "specified by this option and must be a multiple of "
         "4). Writing 0/1 to one of these addresses (using sw) "
         "clear/sets the software interrupt bit in the the MIP (machine "
         "interrupt pending) CSR of the corresponding hart. If a "
         "software interrupt is taken, it is up to interrupt handler to "
         "write zero to the same location to clear the corresponding "
         "bit in MIP. Writing values besides 0/1 will not affect the "
         "MIP bit and neither will writing using sb/sh/sd or writing to "
         "non-multiple-of-4 addresses.")
        ("clint", po::value<std::string>(),
         "Define address, a, of memory mapped area for clint (core local "
         "interruptor). In an n-hart system, words at addresses a, a+4, ... "
         "a+(n-1)*4, are  associated with the n harts. Store a 0/1 to one of "
         "these locations clears/sets the software interrupt bit in the MIP CSR "
         "of the corresponding hart. Similary, addresses b, b+8, ... b+(n-1)*8, "
         "where b is a+0x4000, are associated with the n harts. Writing to one "
         "of these double words sets the timer-limit of the corresponding hart. "
         "A timer interrupt in such a hart becomes pending when the timer value "
         "equals or exceeds the timer limit.")
        ("interruptor", po::value<std::string>(),
         "Define address, z, of a memory mapped interrupt agent. Storing a word in z,"
	 "using a sw instruction, will set/clear a bit in the MIP CSR of a hart"
	 "in the system. The stored word should have the hart index in bits 0 to 11,"
	 "the interrupt id (bit number of MIP) in bits 12 to 19, and the interrupt"
	 "value in bits 20 to 31 (0 to clear, non-zero to set).")
        ("syscallslam", po::value<std::string>(),
         "Define address, a, of a non-cached memory area in which the "
         "memory changes of an emulated system call will be slammed. This "
         "is used in server mode to relay the effects of a system call "
         "to the RTL simulator. The memory area at location a will be filled "
         "with a sequence of pairs of double words designating addresses and "
         "corresponding values. A zero/zero pair will indicate the end of "
         "sequence.")
	("mcm", po::bool_switch(&args.mcm),
	 "Enabe memory consistency checks. This is meaningful in server/interactive "
	 "mode.")
	("mcmca", po::bool_switch(&args.mcmca),
	 "Check all bytes of the memory consistency check merge buffer. If not used "
	 "we only check the bytes inserted into the merge buffer.")
	("mcmls", po::value<std::string>(),
	 "Memory consitency checker merge buffer line size. If set to zero then "
	 "write operations are not buffered and will happen as soon a received.")
	("instcounter", po::value<std::string>(),
	 "Set instruction counter to given value.")
        ("quitany", po::bool_switch(&args.quitOnAnyHart),
         "Terminate multi-threaded run when any hart finishes (default is to wait "
         "for all harts.)")
        ("noconinput", po::bool_switch(&args.noConInput),
         "Do not use console IO address for input. Loads from the cosole io address "
         "simply return last value stored there.")
	("verbose,v", po::bool_switch(&args.verbose),
	 "Be verbose.")
	("version", po::bool_switch(&args.version),
	 "Print version.");

      // Define positional options.
      po::positional_options_description pdesc;
      pdesc.add("target", -1);

      // Parse command line options.
      po::variables_map varMap;
      po::command_line_parser parser(argc, argv);
      auto parsed = parser.options(desc).positional(pdesc).run();
      po::store(parsed, varMap);
      po::notify(varMap);

      // auto unparsed = po::collect_unrecognized(parsed.options, po::include_positional);

      if (args.version)
        printVersion();

      if (args.help)
	{
	  std::cout <<
	    "Simulate a RISCV system running the program specified by the given ELF\n"
	    "and/or HEX file. With --newlib/--linux, the ELF file is a newlib/linux linked\n"
	    "program and may be followed by corresponding command line arguments.\n"
	    "All numeric arguments are interpreted as hexadecimal numbers when prefixed"
	    " with 0x."
	    "Examples:\n"
	    "  whisper --target prog --log\n"
	    "  whisper --target prog --setreg sp=0xffffff00\n"
	    "  whisper --newlib --log --target \"prog -x -y\"\n"
	    "  whisper --linux --log --targetsep ':' --target \"prog:-x:-y\"\n\n";
	  std::cout << desc;
	  return true;
	}

      if (not collectCommandLineValues(varMap, args))
	return false;
    }

  catch (std::exception& exp)
    {
      std::cerr << "Failed to parse command line args: " << exp.what() << '\n';
      return false;
    }

  return true;
}


/// Apply register initializations specified on the command line.
template<typename URV>
static
bool
applyCmdLineRegInit(const Args& args, Hart<URV>& hart)
{
  bool ok = true;

  URV hartIx = hart.sysHartIndex();

  for (const auto& regInit : args.regInits)
    {
      // Each register initialization is a string of the form reg=val
      // or hart:reg=val
      std::vector<std::string> tokens;
      boost::split(tokens, regInit, boost::is_any_of("="),
		   boost::token_compress_on);
      if (tokens.size() != 2)
	{
	  std::cerr << "Invalid command line register initialization: "
		    << regInit << '\n';
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
	  if (not parseCmdLineNumber("hart", hartStr, ix))
	    {
	      std::cerr << "Invalid command line register initialization: "
			<< regInit << '\n';
	      ok = false;
	      continue;
	    }
	  specificHart = true;
	}

      URV val = 0;
      if (not parseCmdLineNumber("register", regVal, val))
	{
	  ok = false;
	  continue;
	}

      if (specificHart and ix != hartIx)
	continue;

      if (unsigned reg = 0; hart.findIntReg(regName, reg))
	{
	  if (args.verbose)
	    std::cerr << "Setting register " << regName << " to command line "
		      << "value 0x" << std::hex << val << std::dec << '\n';
	  hart.pokeIntReg(reg, val);
	  continue;
	}

      if (unsigned reg = 0; hart.findFpReg(regName, reg))
	{
	  if (args.verbose)
	    std::cerr << "Setting register " << regName << " to command line "
		      << "value 0x" << std::hex << val << std::dec << '\n';
	  hart.pokeFpReg(reg, val);
	  continue;
	}

      auto csr = hart.findCsr(regName);
      if (csr)
	{
	  if (args.verbose)
	    std::cerr << "Setting register " << regName << " to command line "
		      << "value 0x" << std::hex << val << std::dec << '\n';
	  hart.pokeCsr(csr->getNumber(), val);
	  continue;
	}

      std::cerr << "No such RISCV register: " << regName << '\n';
      ok = false;
    }

  return ok;
}


static
void
checkForNewlibOrLinux(const Args& args, bool& newlib, bool& linux)
{
  if (args.raw)
    {
      if (args.newlib or args.linux)
	std::cerr << "Raw mode not comptible with newlib/linux. Sticking"
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
	linux = Memory::isSymbolInElfFile(elfPath, "__libc_csu_init");

      if (not newlib)
	newlib = Memory::isSymbolInElfFile(elfPath, "__call_exitprocs");

      if (linux and newlib)
	break;
    }

  if (linux and args.verbose)
    std::cerr << "Detected linux symbol in ELF\n";

  if (newlib and args. verbose)
    std::cerr << "Detected newlib symbol in ELF\n";

  if (newlib and linux)
    {
      std::cerr << "Fishy: Both newlib and linux symbols present in "
		<< "ELF file(s). Doing linux emulation.\n";
      newlib = false;
    }
}


/// Set stack pointer to a reasonable value for linux/newlib.
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


/// Load register and memory state from snapshot previously saved
/// in the given directory. Return true on success and false on
/// failure.
template <typename URV>
static
bool
loadSnapshot(System<URV>& system, Hart<URV>& hart, const std::string& snapDir)
{
  using std::cerr;

  if (not Filesystem::is_directory(snapDir))
    {
      cerr << "Error: Path is not a snapshot directory: " << snapDir << '\n';
      return false;
    }

  Filesystem::path path(snapDir);
  Filesystem::path regPath = path / "registers";
  if (not Filesystem::is_regular_file(regPath))
    {
      cerr << "Error: Snapshot file does not exists: " << regPath << '\n';
      return false;
    }

  Filesystem::path memPath = path / "memory";
  if (not Filesystem::is_regular_file(regPath))
    {
      cerr << "Error: Snapshot file does not exists: " << memPath << '\n';
      return false;
    }

  if (not system.loadSnapshot(path, hart))
    {
      cerr << "Error: Failed to load sanpshot from dir " << snapDir << '\n';
      return false;
    }

  return true;
}


static
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
      std::cerr << "Warning differen ELF files have different ISA strings: "
		<< tag << " and " << ref << '\n';

  isaString = ref;

  if (args.verbose)
    std::cerr << "ISA string from ELF file(s): " << isaString << '\n';

  return errors == 0;
}


/// Return the string representing the current contents of the MISA CSR.
template<typename URV>
static
std::string
getIsaStringFromCsr(const Hart<URV>& hart)
{
  std::string res;

  URV val;
  if (not hart.peekCsr(CsrNumber::MISA, val))
    return res;

  URV mask = 1;
  for (char c = 'a'; c <= 'z'; ++c, mask <<= 1)
    if (val & mask)
      res += c;

  return res;
}

/// Apply command line arguments: Load ELF and HEX files, set
/// start/end/tohost. Return true on success and false on failure.
template<typename URV>
static
bool
applyCmdLineArgs(const Args& args, Hart<URV>& hart, System<URV>& system,
		 const HartConfig& config, bool clib)
{
  unsigned errors = 0;

  if (clib)  // Linux or newlib enabled.
    sanitizeStackPointer(hart, args.verbose);

  if (args.toHostSym)
    system.setTohostSymbol(*args.toHostSym);

  if (args.consoleIoSym)
    system.setConsoleIoSymbol(*args.consoleIoSym);

  // Load ELF files. Entry point of first file sets the start PC
  // unless in raw mode.
  if (hart.sysHartIndex() == 0)
    {
      StringVec paths;
      for (const auto& target : args.expandedTargets)
	paths.push_back(target.at(0));
      if (not system.loadElfFiles(paths, args.raw, args.verbose))
	errors++;
    }

  // Load HEX files.
  if (not system.loadHexFiles(args.hexFiles, args.verbose))
    errors++;

  // Load binary files
  uint64_t offset = 0;
  if (not system.loadBinaryFiles(args.binaryFiles, offset, args.verbose))
    errors++;

  if (not args.kernelFile.empty())
    {
      // Default kernel file offset. FIX: make a parameter.
      std::vector<std::string> files{args.kernelFile};
      offset = hart.isRv64() ? 0x80200000 : 0x80400000;
      if (not system.loadBinaryFiles(files, offset, args.verbose))
	errors++;
    }

  if (not args.instFreqFile.empty())
    hart.enableInstructionFrequency(true);

  if (args.clint)
    {
      uint64_t swAddr = *args.clint;
      uint64_t timerAddr = swAddr + 0x4000;
      uint64_t clintLimit = swAddr + 0xc000 - 1;
      config.configClint(system, hart, swAddr, clintLimit, timerAddr);
    }

  uint64_t window = 1000000;
  if (args.branchWindow_)
    window = *args.branchWindow_;
  if (not args.branchTraceFile.empty())
    hart.traceBranches(args.branchTraceFile, window);

  if (not args.loadFrom.empty())
    {
      if (not loadSnapshot(system, hart, args.loadFrom))
	errors++;

      if (not args.stdoutFile.empty() or not args.stderrFile.empty() or
	  not args.stdinFile.empty())
	std::cerr << "Info: Options --stdin, --stdout, and --stderr are ignored with --loadfrom\n";
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
    hart.pokePc(URV(*args.startPc));

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

  // Set instruction count limit.
  if (args.instCountLim)
    {
      uint64_t count = args.relativeInstCount? hart.getInstructionCount() : 0;
      count += *args.instCountLim;
      hart.setInstructionCountLimit(count);
    }

  // Print load-instruction data-address when tracing instructions.
  // if (args.traceLdSt)  // Deprecated -- now always on.

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
  else if (not args.expandedTargets.empty() and args.expandedTargets.front().size() > 1)
    {
      std::cerr << "Warning: Target program options present which requires\n"
		<< "         the use of --newlib/--linux. Options ignored.\n";
    }

  if (args.csv)
    hart.enableCsvLog(args.csv);

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

  return errors == 0;
}


/// Open a server socket and put opened socket information (hostname
/// and port number) in the given server file. Wait for one
/// connection. Service connection. Return true on success and false
/// on failure.
template <typename URV>
static
bool
runServer(System<URV>& system, const std::string& serverFile,
	  FILE* traceFile, FILE* commandLog)
{
  char hostName[1024];
  if (gethostname(hostName, sizeof(hostName)) != 0)
    {
      std::cerr << "Failed to obtain name of this computer\n";
      return false;
    }

  int soc = socket(AF_INET, SOCK_STREAM, 0);
  if (soc < 0)
    {
      char buffer[512];
      char* p = buffer;
#ifdef __APPLE__
      strerror_r(errno, buffer, 512);
#else
      p = strerror_r(errno, buffer, 512);
#endif
      std::cerr << "Failed to create socket: " << p << '\n';
      return -1;
    }

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
    out << hostName << ' ' << ntohs(socAddr.sin_port) << std::endl;
  }

  sockaddr_in clientAddr;
  socklen_t clientAddrSize = sizeof(clientAddr);
  int newSoc = accept(soc, (sockaddr*) & clientAddr, &clientAddrSize);
  if (newSoc < 0)
    {
      perror("Socket accept failed");
      return false;
    }

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

/// Open a shared memory region and write name to given server file.
/// Return true on success and false on failure
template <typename URV>
static
bool
runServerShm(System<URV>& system, const std::string& serverFile,
	  FILE* traceFile, FILE* commandLog)
{
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

  char* shm = (char*) mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (shm == MAP_FAILED)
    {
      perror("Failed mmap");
      return false;
    }

  bool ok = true;

  try
    {
      Server<URV> server(system);
      ok = server.interact(shm, traceFile, commandLog);
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


template <typename URV>
static
bool
reportInstructionFrequency(Hart<URV>& hart, const std::string& outPath)
{
  FILE* outFile = fopen(outPath.c_str(), "w");
  if (not outFile)
    {
      std::cerr << "Failed to open instruction frequency file '" << outPath
		<< "' for output.\n";
      return false;
    }

  hart.reportInstructionFrequency(outFile);
  hart.reportTrapStat(outFile);
  fprintf(outFile, "\n");
  hart.reportPmpStat(outFile);
  fprintf(outFile, "\n");
  hart.reportLrScStat(outFile);

  fclose(outFile);
  return true;
}


/// Open the trace-file, command-log and console-output files
/// specified on the command line. Return true if successful or false
/// if any specified file fails to open.
static
bool
openUserFiles(const Args& args, FILE*& traceFile, FILE*& commandLog,
	      FILE*& consoleOut, FILE*& bblockFile, FILE*& attFile)
{
  size_t len = args.traceFile.size();
  bool doGzip = len > 3 and args.traceFile.substr(len-3) == ".gz";

  if (not args.traceFile.empty())
    {
      if (doGzip)
	{
	  std::string cmd = "/usr/bin/gzip -c > ";
	  cmd += args.traceFile;
	  traceFile = popen(cmd.c_str(), "w");
	}
      else
	traceFile = fopen(args.traceFile.c_str(), "w");
      if (not traceFile)
	{
	  std::cerr << "Failed to open trace file '" << args.traceFile
		    << "' for output\n";
	  return false;
	}
    }

  if (args.trace and traceFile == nullptr)
    traceFile = stdout;

  if (not args.commandLogFile.empty())
    {
      commandLog = fopen(args.commandLogFile.c_str(), "w");
      if (not commandLog)
	{
	  std::cerr << "Failed to open command log file '"
		    << args.commandLogFile << "' for output\n";
	  return false;
	}
      setlinebuf(commandLog);  // Make line-buffered.
    }

  if (not args.consoleOutFile.empty())
    {
      consoleOut = fopen(args.consoleOutFile.c_str(), "w");
      if (not consoleOut)
	{
	  std::cerr << "Failed to open console output file '"
		    << args.consoleOutFile << "' for output\n";
	  return false;
	}
    }

  if (not args.bblockFile.empty())
    {
      bblockFile = fopen(args.bblockFile.c_str(), "w");
      if (not bblockFile)
	{
	  std::cerr << "Failed to open basic block file '"
		    << args.bblockFile << "' for output\n";
	  return false;
	}
    }

  if (not args.attFile.empty())
    {
      attFile = fopen(args.attFile.c_str(), "w");
      if (not attFile)
        {
          std::cerr << "Failed to open address translation file '"
                    << args.attFile << "' for output\n";
          return false;
        }
    }

  return true;
}


/// Counterpart to openUserFiles: Close any open user file.
static
void
closeUserFiles(const Args& args, FILE*& traceFile, FILE*& commandLog,
	       FILE*& consoleOut, FILE*& bblockFile, FILE*& attFile)
{
  if (consoleOut and consoleOut != stdout)
    fclose(consoleOut);
  consoleOut = nullptr;

  if (traceFile and traceFile != stdout)
    {
      size_t len = args.traceFile.size();
      bool doGzip = len > 3 and args.traceFile.substr(len-3) == ".gz";
      if (doGzip)
	pclose(traceFile);
      else
	fclose(traceFile);
    }
  traceFile = nullptr;

  if (commandLog and commandLog != stdout)
    fclose(commandLog);
  commandLog = nullptr;

  if (bblockFile and bblockFile != stdout)
    fclose(bblockFile);
  bblockFile = nullptr;

  if (attFile and attFile != stdout)
    fclose(attFile);
  attFile = nullptr;
}


// In interactive mode, keyboard interrupts (typically control-c) are
// ignored.
static void
kbdInterruptHandler(int)
{
  std::cerr << "keyboard interrupt\n";
}


template <typename URV>
static bool
batchRun(System<URV>& system, FILE* traceFile, bool waitAll)
{
  if (system.hartCount() == 0)
    return true;

  if (system.hartCount() == 1)
    {
      auto& hart = *system.ithHart(0);
      bool ok = hart.run(traceFile);
#ifdef FAST_SLOPPY
      hart.reportOpenedFiles(std::cout);
#endif
      return ok;
    }

  // Run each hart in its own thread.

  std::vector<std::thread> threadVec;

  std::atomic<bool> result = true;
  std::atomic<unsigned> finished = 0;  // Count of finished threads. 

  auto threadFunc = [&traceFile, &result, &finished] (Hart<URV>* hart) {
		      bool r = hart->run(traceFile);
		      result = result and r;
                      finished++;
		    };

  for (unsigned i = 0; i < system.hartCount(); ++i)
    {
      Hart<URV>* hart = system.ithHart(i).get();
      threadVec.emplace_back(std::thread(threadFunc, hart));
    }                             

  if (waitAll)
    {
      for (auto& t : threadVec)
        t.join();
    }
  else
    {
      // First thread to finish terminates run.
      while (finished == 0)
        ;

      extern void forceUserStop(int);
      forceUserStop(0);
      
      for (auto& t : threadVec)
        t.join();
    }

  return result;
}


/// Run producing a snapshot after each snapPeriod instructions. Each
/// snapshot goes into its own directory names <dir><n> where <dir> is
/// the string in snapDir and <n> is a sequential integer starting at
/// 0. Return true on success and false on failure.
template <typename URV>
static
bool
snapshotRun(System<URV>& system, FILE* traceFile, const std::string& snapDir,
	    const Uint64Vec& periods)
{
  assert(system.hartCount() == 1);
  Hart<URV>& hart = *(system.ithHart(0));

  uint64_t globalLimit = hart.getInstructionCountLimit();

  for (size_t ix = 0; true; ++ix)
    {
      uint64_t nextLimit = globalLimit;
      if (not periods.empty())
	{
	  if (periods.size() == 1)
	    nextLimit = hart.getInstructionCount() + periods.at(0);
	  else
	    nextLimit = ix < periods.size() ? periods.at(ix) : globalLimit;
	}

      nextLimit = std::min(nextLimit, globalLimit);
      hart.setInstructionCountLimit(nextLimit);
      uint64_t tag = ix;
      if (periods.size() > 1)
	tag = ix < periods.size() ? periods.at(ix) : nextLimit;
      std::string pathStr = snapDir + std::to_string(tag);
      Filesystem::path path = pathStr;
      if (not Filesystem::is_directory(path) and not Filesystem::create_directories(path))
	{
	  std::cerr << "Error: Failed to create snapshot directory " << pathStr << '\n';
	  return false;
	}

      hart.run(traceFile);

      if (hart.hasTargetProgramFinished() or nextLimit >= globalLimit)
	{
	  Filesystem::remove_all(path);
	  break;
	}

      if (not system.saveSnapshot(hart, pathStr))
	{
	  std::cerr << "Error: Failed to save a snapshot\n";
	  return false;
	}
    }

  hart.traceBranches(std::string(), 0);  // Turn off branch tracing.

#ifdef FAST_SLOPPY
  hart.reportOpenedFiles(std::cout);
#endif

  return true;
}


static
bool
determineIsa(const HartConfig& config, const Args& args, bool clib, std::string& isa)
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
        std::cerr << "No ISA specfied, using a/c/m/f/d extensions for newlib/linux\n";
      isa = "imcafd";
    }

  if (isa.empty() and not args.raw)
    {
      if (args.verbose)
	std::cerr << "No ISA specified: Defaulting to imac\n";
      isa = "imac";
    }

  return true;
}


void (*tracerExtension)(void*) = nullptr;
void (*tracerExtensionInit)() = nullptr;
extern "C" {
  std::string tracerExtensionArgs = "";
}

template <typename URV>
static
bool
loadTracerLibrary(const std::string& tracerLib)
{
  if (tracerLib.empty())
    return true;

  std::vector<std::string> result;
  boost::split(result, tracerLib, boost::is_any_of(":"));
  assert(result.size() >= 1);

  auto soPtr = dlopen(result[0].c_str(), RTLD_NOW);
  if (not soPtr)
    {
      std::cerr << "Error: Failed to load shared libarary " << dlerror() << '\n';
      return false;
    }

  if (result.size() == 2)
    tracerExtensionArgs = result[1];

  std::string entry("tracerExtension");
  entry += sizeof(URV) == 4 ? "32" : "64";

  tracerExtension = reinterpret_cast<void (*)(void*)>(dlsym(soPtr, entry.c_str()));
  if (not tracerExtension)
    {
      std::cerr << "Error: Could not find symbol tracerExtension in " << tracerLib << '\n';
      return false;
    }

  entry = "tracerExtensionInit";
  entry += sizeof(URV) == 4 ? "32" : "64";

  tracerExtensionInit = reinterpret_cast<void (*)()>(dlsym(soPtr, entry.c_str()));
  if (tracerExtensionInit)
    tracerExtensionInit();

  return true;
}


/// Depending on command line args, start a server, run in interactive
/// mode, or initiate a batch run.
template <typename URV>
static
bool
sessionRun(System<URV>& system, const Args& args, FILE* traceFile, FILE* cmdLog)
{
  if (not loadTracerLibrary<URV>(args.tracerLib))
    return false;

  // In server/interactive modes: enable triggers and performance counters.
  bool serverMode = not args.serverFile.empty();
  if (serverMode or args.interactive)
    {
      for (unsigned i = 0; i < system.hartCount(); ++i)
        {
          auto& hart = *system.ithHart(i);
          hart.enableTriggers(true);
          hart.enablePerformanceCounters(true);
        }
    }

  if (serverMode)
    return (args.shm)? runServerShm(system, args.serverFile, traceFile, cmdLog) :
                       runServer(system, args.serverFile, traceFile, cmdLog);

  if (args.interactive)
    {
      // Ignore keyboard interrupt for most commands. Long running
      // commands will enable keyboard interrupts while they run.
      struct sigaction newAction;
      sigemptyset(&newAction.sa_mask);
      newAction.sa_flags = 0;
      newAction.sa_handler = kbdInterruptHandler;
      sigaction(SIGINT, &newAction, nullptr);

      Interactive interactive(system);
      return interactive.interact(traceFile, cmdLog);
    }

  if (not args.snapshotPeriods.empty())
    {
      if (system.hartCount() == 1)
        return snapshotRun(system, traceFile, args.snapshotDir, args.snapshotPeriods);
      std::cerr << "Warning: Snapshots not supported for multi-thread runs\n";
    }

  bool waitAll = not args.quitOnAnyHart;
  return batchRun(system, traceFile, waitAll);
}


/// Santize memory parameters. Page/region sizes must be greater and
/// equal to 4 and must be powers of 2.
/// Region size must be a multiple of page size.
/// Memory size must be a multiple of region size.
/// Return true if given parameters are good. False if any parameters
/// is changed to meet expectation.
static
bool
checkAndRepairMemoryParams(size_t& memSize, size_t& pageSize)
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


static
bool
getPrimaryConfigParameters(const Args& args, const HartConfig& config,
                           unsigned& hartsPerCore, unsigned& coreCount,
                           size_t& pageSize, size_t& memorySize)
{
  config.getHartsPerCore(hartsPerCore);
  if (args.hasHarts)
    hartsPerCore = args.harts;
  if (hartsPerCore == 0 or hartsPerCore > 16)
    {
      std::cerr << "Unsupported hart count: " << hartsPerCore;
      std::cerr << " (1 to 16 currently suppored)\n";
      return false;
    }

  config.getCoreCount(coreCount);
  if (args.hasCores)
    coreCount = args.cores;
  if (coreCount == 0 or coreCount > 16)
    {
      std::cerr << "Unsupported core count: " << coreCount;
      std::cerr << " (1 to 16 currently suppored)\n";
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
static
bool
session(const Args& args, const HartConfig& config)
{
  // Collect primary configuration paramters.
  unsigned hartsPerCore = 1;
  unsigned coreCount = 1;
  size_t pageSize = 4*1024;
  size_t memorySize = size_t(1) << 32;  // 4 gigs

  if (not getPrimaryConfigParameters(args, config, hartsPerCore, coreCount,
                                     pageSize, memorySize))
    return false;

  checkAndRepairMemoryParams(memorySize, pageSize);

  // Create cores & harts.
  unsigned hartIdOffset = hartsPerCore;
  config.getHartIdOffset(hartIdOffset);
  if (hartIdOffset < hartsPerCore)
    {
      std::cerr << "Invalid core_hart_id_offset: " << hartIdOffset
                << ",  must be greater than harts_per_core: " << hartsPerCore << '\n';
      return false;
    }
  System<URV> system(coreCount, hartsPerCore, hartIdOffset, memorySize, pageSize);
  assert(system.hartCount() == coreCount*hartsPerCore);
  assert(system.hartCount() > 0);

  // Configure harts. Define callbacks for non-standard CSRs.
  bool userMode = args.isa.find_first_of("uU") != std::string::npos;
  if (not config.configHarts(system, userMode, args.verbose))
    if (not args.interactive)
      return false;

  // Configure memory.
  if (not config.configMemory(system, args.unmappedElfOk))
    return false;

  if (not args.dataLines.empty())
    system.enableDataLineTrace(args.dataLines);
  if (not args.instrLines.empty())
    system.enableInstructionLineTrace(args.instrLines);

  if (args.hexFiles.empty() and args.expandedTargets.empty()
      and args.binaryFiles.empty() and args.kernelFile.empty()
      and not args.interactive)
    {
      std::cerr << "No program file specified.\n";
      return false;
    }

  FILE* traceFile = nullptr;
  FILE* commandLog = nullptr;
  FILE* consoleOut = stdout;
  FILE* bblockFile = nullptr;
  FILE* attFile = nullptr;
  if (not openUserFiles(args, traceFile, commandLog, consoleOut, bblockFile, attFile))
    return false;

  bool newlib = false, linux = false;
  checkForNewlibOrLinux(args, newlib, linux);
  bool clib = newlib or linux;
  bool updateMisa = clib and not config.hasCsrConfig("misa");

  std::string isa;
  if (not determineIsa(config, args, clib, isa))
    return false;

  for (unsigned i = 0; i < system.hartCount(); ++i)
    {
      auto& hart = *system.ithHart(i);
      hart.setConsoleOutput(consoleOut);
      hart.enableBasicBlocks(bblockFile, args.bblockInsts);
      hart.enableAddrTransLog(attFile);
      hart.enableNewlib(newlib);
      hart.enableLinux(linux);
      if (not isa.empty())
	if (not hart.configIsa(isa, updateMisa))
	  return false;
      hart.reset();
    }

  for (unsigned i = 0; i < system.hartCount(); ++i)
    if (not applyCmdLineArgs(args, *system.ithHart(i), system, config, clib))
      if (not args.interactive)
	return false;

  if (not args.initStateFile.empty())
    {
      if (system.hartCount() > 1)
	{
	  std::cerr << "Initial line-state report (--initstate) valid only when hart count is 1\n";
	  return false;
	}
      auto& hart0 = *system.ithHart(0);
      if (not hart0.setInitialStateFile(args.initStateFile))
	return false;
    }

  bool result = sessionRun(system, args, traceFile, commandLog);

  auto& hart0 = *system.ithHart(0);
  if (not args.instFreqFile.empty())
    result = reportInstructionFrequency(hart0, args.instFreqFile) and result;

  if (not args.testSignatureFile.empty())
    result = system.produceTestSignatureFile(args.testSignatureFile) and result;

  closeUserFiles(args, traceFile, commandLog, consoleOut, bblockFile, attFile);

  return result;
}


/// Determine regiser width (xlen) from ELF file.  Return true if
/// successful and false otherwise (xlen is left unmodified).
static
bool
getXlenFromElfFile(const Args& args, unsigned& xlen)
{
  if (args.expandedTargets.empty())
    return false;

  // Get the length from the first target.
  auto& elfPath = args.expandedTargets.front().front();
  bool is32 = false, is64 = false, isRiscv = false;
  if (not Memory::checkElfFile(elfPath, is32, is64, isRiscv))
    return false;  // ELF does not exist.

  if (not is32 and not is64)
    return false;

  if (is32 and is64)
    {
      std::cerr << "Error: ELF file '" << elfPath << "' has both"
		<< " 32  and 64-bit calss\n";
      return false;
    }

  if (is32)
    xlen = 32;
  else
    xlen = 64;

  if (args.verbose)
    std::cerr << "Setting xlen to " << xlen << " based on ELF file "
	      <<  elfPath << '\n';
  return true;
}


/// Obtain integer-register width (xlen). Command line has top
/// priority, then config file, then ELF file.
static
unsigned
determineRegisterWidth(const Args& args, const HartConfig& config)
{
  unsigned isaLen = 0;
  if (not args.isa.empty())
    {
      if (boost::starts_with(args.isa, "rv32"))
	isaLen = 32;
      else if (boost::starts_with(args.isa, "rv64"))
	isaLen = 64;
    }

  // 1. If --xlen used, go with that.
  if (args.hasRegWidth)
    {
      unsigned xlen = args.regWidth;
      if (args.verbose)
	std::cerr << "Setting xlen from --xlen: " << xlen << "\n";
      if (isaLen and xlen != isaLen)
	{
	  std::cerr << "Xlen value from --xlen (" << xlen
		    << ") different from --isa (" << args.isa
		    << "), using: " << xlen << "\n";
	}
      return xlen;
    }

  // 2. If --isa specifies xlen, go with that.
  if (isaLen)
    {
      if (args.verbose)
        std::cerr << "Setting xlen from --isa: " << isaLen << "\n";
      return isaLen;
    }

  // 3. If config file specifies xlen, go with that.
  unsigned xlen = 32;
  if (config.getXlen(xlen))
    {
      if (args.verbose)
	std::cerr << "Setting xlen from config file: " << xlen << "\n";
      return xlen;
    }

  // 4. Get xlen from ELF file.
  if (getXlenFromElfFile(args, xlen))
    {
      if (args.verbose)
	std::cerr << "Setting xlen from ELF file: " << xlen << "\n";
    }
  else if (args.verbose)
    std::cerr << "Using default for xlen: " << xlen << "\n";
  
  return xlen;
}


#include <termios.h>


int
main(int argc, char* argv[])
{
  Args args;
  if (not parseCmdLineArgs(argc, argv, args))
    return 1;
  if (args.help)
    return 0;

  // Expand each target program string into program name and args.
  args.expandTargets();

  // Load configuration file.
  HartConfig config;
  if (not args.configFile.empty())
    if (not config.loadConfigFile(args.configFile))
      return 1;

  struct termios term;
  tcgetattr(STDIN_FILENO, &term);  // Save terminal state.

  unsigned regWidth = determineRegisterWidth(args, config);
  bool ok = true;

  try
    {
      if (regWidth == 32)
	ok = session<uint32_t>(args, config);
      else if (regWidth == 64)
	ok = session<uint64_t>(args, config);
      else
	{
	  std::cerr << "Invalid register width: " << regWidth;
	  std::cerr << " -- expecting 32 or 64\n";
	  ok = false;
	}
    }
  catch (std::exception& e)
    {
      std::cerr << e.what() << '\n';
      ok = false;
    }
	
  tcsetattr(STDIN_FILENO, 0, &term);  // Restore terminal state.
  return ok? 0 : 1;
}
