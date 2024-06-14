#pragma once

#include <cstdint>
#include <string>
#include <span>
#include <optional>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>


namespace WdRiscv
{
  class HartConfig;

  /// Parse/maintain arguments provided on the command line.
  struct Args
  {
    using StringVec = std::vector<std::string>;
    using Uint64Vec = std::vector<uint64_t>;

    /// Expand each target program string into program name and args.
    void expandTargets();

    /// Determine register width (xlen) from ELF file. Return true if successful and false
    /// otherwise (xlen is left unmodified).
    bool getXlenFromElfFile(unsigned& xlen) const;

    /// Parse command line arguments and collect option values. Return true on success and
    /// false on failure.
    bool parseCmdLineArgs(std::span<char*> argv);

    /// Parse command line arguments and collect option vlaues. Return true on success and
    /// false on failure.
    bool parseCmdLineArgs(std::vector<std::string>& args)
    {
      std::vector<char*> argv;
      for (auto& arg : args)
	argv.push_back(arg.data());
      return parseCmdLineArgs(std::span(argv.data(), argv.size()));
    }

    /// Helper to parseCmdLineArgs.
    bool collectCommandLineValues(const boost::program_options::variables_map& varMap);

    /// Convert the command line string numberStr to a number using strotull and a base of
    /// zero (prefixes 0 and 0x are honored). Return true on success and false on failure
    /// (string does not represent a number). TYPE is an integer type (e.g
    /// uint32_t). Option is the command line option associated with the string and is
    /// used for diagnostic messages.
    template <typename TYPE>
    static bool
    parseCmdLineNumber(const std::string& option, const std::string& numberStr,
		       TYPE& number);

    /// Adapter for the parseCmdLineNumber for optionals.
    template <typename TYPE>
    static bool
    parseCmdLineNumber(const std::string& option, const std::string& numberStr,
		       std::optional<TYPE>& number);

    StringVec   hexFiles;                  // Hex files to be loaded into simulator memory.
    StringVec   binaryFiles;               // Binary files to be loaded into simulator memory.
#ifdef LZ4_COMPRESS
    StringVec   lz4Files;                  // LZ4 files to be loaded into simulator memory.
#endif
    std::string traceFile;                 // Log of state change after each instruction.
    std::string commandLogFile;            // Log of interactive or socket commands.
    std::string consoleOutFile;            // Console io output file.
    std::string serverFile;                // File in which to write server host and port.
    std::string instFreqFile;              // Instruction frequency file.
    std::string configFile;                // Configuration (JSON) file.
    std::string bblockFile;                // Basci block file.
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
    std::string initStateFile;             // Output: initial state of used memory lines.
    std::string kernelFile;                // Input: Load kernel image at address.
    std::string testSignatureFile;         // Output: signature to score riscv-arch-test tests.
    StringVec   regInits;                  // Initial values of regs.
    StringVec   targets;                   // Target (ELF file) programs and associated
    // program options to be loaded into simulator
    // memory. Each target plus args is one string.
    StringVec   isaVec;                    // Extensions from isa string rv32/rv64 removed.
    std::string targetSep = " ";           // Target program argument separator.
    StringVec   pciDevs;                   // PCI device list.
    StringVec   envVars;                   // Environment variables.

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
    std::optional<uint64_t> tlbSize;
    Uint64Vec snapshotPeriods;
    Uint64Vec steesr;
    std::optional<uint64_t> alarmInterval;
    std::optional<uint64_t> clint;        // Advanced core-local-interrupt (CLINT) mem mapped address
    std::optional<uint64_t> interruptor;  // Interrupt generator mem mapped address
    std::optional<uint64_t> syscallSlam;
    std::optional<uint64_t> instCounter;
    std::optional<uint64_t> branchWindow;
    std::optional<uint64_t> logStart;
    std::optional<unsigned> mcmls;
    std::optional<uint64_t> deterministic;
    std::optional<unsigned> seed;

    unsigned regWidth = 32;
    unsigned harts = 1;
    unsigned cores = 1;
    unsigned pageSize = 4U*1024;
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
    std::vector<unsigned> gdbTcpPort;   // Enable gdb mode over TCP when port is positive.
    bool abiNames = false;   // Use ABI register names in inst dis-assembly.
    bool newlib = false;     // True if target program linked with newlib.
    bool linux = false;      // True if target program linked with Linux C-lib.
    bool raw = false;        // True if bare-metal program (no linux no newlib).
    bool elfisa = false;     // Use ELF file RISCV architecture tags to set MISA if true.
    bool unmappedElfOk = false;
    bool mcm = false;        // Memory consistency checks.
    bool mcmca = false;      // Memory consistency checks: check all bytes of merge buffer.
    bool perfApi = false;    // Performance model API.
    bool reportub = false;         // Report used blocks with sparse memory.
    bool quitOnAnyHart = false;    // True if run quits when any hart finishes.
    bool noConInput = false;       // If true console io address is not used for input.
    bool relativeInstCount = false;
    bool tracePtw = false;   // Enable printing of page table walk info in log.
    bool shm = false;        // Enable shared memory IPC for server mode (default is socket).
    bool logPerHart = false; // Enable separate log files for each hart.
  };
}
