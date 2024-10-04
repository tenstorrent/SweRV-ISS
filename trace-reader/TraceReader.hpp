#pragma once

#include <vector>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <boost/iostreams/filtering_streambuf.hpp>

namespace WhisperUtil  {

  // Operand type: Integer-register, floating-point register, control
  // and status register, vector register, or immediate value.
  enum class OperandType { Int, Fp, Csr, Vec, Imm };

  // Processor privilege mod.
  enum class PrivMode { Machine, Supervisor, User };

  class PageTableMaker;

  // Enum for the major columns of the log file.
  enum class HeaderTag { Pc, Inst, DestRegs, SourceOps,
    Memory, InstType, Priv, Trap, Dis, HartId, Iptw, Dptw, Pmp, _Count
  };

  // Model an instruction operand.
  struct Operand
  {
    OperandType type = OperandType::Int;
    unsigned number = 0;                  // Register number.
    unsigned emul = 1;                    // Effective group multiplier for vector register.
    uint64_t value = 0;
    uint64_t prevValue = 0;               // Used for modified registers.
    std::vector<uint8_t> vecValue;        // Used for vector registers
    std::vector<uint8_t> vecPrevValue;    // Used for modified vector registers
  };

  // Model a record in the log file.
  struct TraceRecord
  {
    uint64_t virtPc = 0, physPc = 0;
    uint64_t takenBranchTarget = 0;
    uint32_t inst = 0;
    uint8_t instSize = 0;
    uint8_t instType = 0;
    uint8_t dataSize = 0;     // For ld/st instructions
    uint8_t fpFlags = 0;
    uint8_t roundingMode = 0;
    std::vector<Operand> modifiedRegs;
    std::vector<Operand> sourceOperands;
    std::vector<std::pair<unsigned, uint64_t>> contextCSRs;
    std::vector<uint64_t> virtAddrs;  // Memory addresses
    std::vector<uint64_t> physAddrs;  // Memory addresses
    std::vector<uint64_t> memVals;    // Correspondign data for store
    std::vector<bool> maskedAddrs;    // Maked addresses (for vector instructions).
    std::vector<uint64_t> dpteAddrs;  // PTE addresses address translation.
    std::vector<uint64_t> ipteAddrs;

    PrivMode priv = PrivMode::Machine;
    bool virt = false;
    bool hasTrap = false;
    uint64_t trap = 0;
    std::string assembly;

    // Clear this record.
    void clear();

    // Print this record to the given stream
    void print(std::ostream& os) const;

    // Return true if this is a floating point instruction.
    bool isFp() const
    { return instType == 'f'; }

    // Return true if this is a vector instruction.
    bool isVector() const
    { return instType == 'v'; }

    // Return true if this is an atomic instruction.
    bool isAtomic() const
    { return instType == 'a'; }

    // Return true if this is a scalar load instruction.
    bool isLoad() const
    { return instType == 'l'; }

    // Return true if this is a scalar store instruction.
    bool isStore() const
    { return instType == 's'; }

    // Return true if this is a vector load instruction.
    bool isVecLoad() const
    { return isVector() and (inst & 0x7f) == 0x7; }

    // Return true if this is a vector store instruction.
    bool isVecStore() const
    { return isVector() and (inst & 0x7f) == 0x27; }

    // Return true if this is a call instruction.
    bool isCall() const
    { return instType == 'c'; }

    // Return true if this is a return instruction.
    bool isReturn() const
    { return instType == 'r'; }

    // Return true if this is a jump instruction (excluding call/return).
    bool isJump() const
    { return instType == 'j'; }

    // Return true if this is a conditional branch instruction.
    bool isConditionalBranch() const
    { return instType == 't' or instType == 'n'; }

    // Return true if this is a conditional branch instruction that is taken.
    bool isTakenConditionalBranch() const
    { return instType == 't'; }

    // Return true if this is a conditional branch instruction that is
    // not taken. This will return false if this is a conditional
    // branch that is taken or if the instruction is not a conditional
    // branch.
    bool isNotTakenConditionalBranch() const
    { return instType == 'n'; }

    // Return true if this an illegal instruction.
    bool isIllegal() const
    { return inst == 0 or ~inst == 0; }
  };

  inline std::ostream & operator<<(std::ostream & os, const TraceRecord & data)
  {
    data.print(os);
    return os;
  }

  // Reader for whisper CSV log file.
  // Samle usage:
  //   TraceRecord rec("log.csv");
  //   TraceReader reader;
  //   while (reader.nextRecord(record))
  //     {
  //        record.print(std::cout);
  //     }
  class TraceReader
  {
  public:

    // Constructor. Open given input file.
    TraceReader(const std::string& inputPath);

    // Constructor with register initialization.
    TraceReader(const std::string& inputPath, const std::string& initPath);

    // Destructor.
    ~TraceReader();

    // Return true if associated input stream if valid (good for
    // input).
    operator bool() const
    { return static_cast<bool> (fileStream_); }

    // Return true if asscoaited input stream is at end of file.
    bool eof() const
    { return fileStream_.eof(); }

    // Read and parse the next record in the input stream opened at
    // construction. Put the read data in the given record. Return
    // true on success and false on failure or end of file.
    bool nextRecord(TraceRecord& record);

    // Read and parse the next record in the input stream opened at
    // construction. Put the read data in the given record. Return
    // true on success and false on failure or end of file. Also store the line
    // that was parsed.
    bool nextRecord(TraceRecord& record, std::string& line);

    // Parse given non-header line putting the collected data in the
    // given record.
    bool parseLine(std::string& line, uint64_t lineNum,
		   TraceRecord& record);

    // Define the parameters of a page table generator. Once defined
    // page table walks can be generated by calling genPageTableWalk.
    // Return true on success and false on failure (we fail if addr
    // is not a multiple of the page size, if arenaSize is not
    // a multiple of the page size or is smaller than 1 page).
    template<class Mode>
    bool definePageTableMaker(uint64_t addr,
			      /* PageTableMaker:: */ Mode mode,
			      uint64_t arenaSize);

    // Generate a page table walk that would be suitable for
    // translating the given virtual address to the given physical
    // address. Page table pages are created as necessary in
    // the arena defined by definePageTableMaker.
    bool genPageTableWalk(uint64_t virAddr, uint64_t physAddr,
			  std::vector<uint64_t>& walk);

    // Parse header line setting up the indices corresponding to the
    // header tags. For example, if the header line consists of
    // "pc,inst", then the Pc index would be 0 and the Inst index would
    // be 1.
    bool extractHeaderIndices(const std::string& line, uint64_t lineNum);

    std::string getHeaderLine();

    // Return the current value of the given integer regiser. The
    // given regiser index must be less than 32.
    uint64_t intRegValue(unsigned ix) const
    { return intRegs_.at(ix); }

    // Return the bits of current value of the given floating point
    // regiser.  The given regiser index must be less than 32.
    uint64_t fpRegValue(unsigned ix) const
    { return fpRegs_.at(ix); }

    // Return the current value of the given CSR. Return 0 if the CSR
    // is not in the trace. The given regiser index must be less than
    // 4096.
    uint64_t csrValue(unsigned ix) const
    { return csRegs_.at(ix); }

    // Return the current value of the given vector regiser.  The
    // given regiser index must be less than 32. Note that we return a
    // reference and the referenced data changes with each invocation
    // of nextRecord.
    const std::vector<uint8_t>& vecRegValue(unsigned ix) const
    { return vecRegs_.at(ix); }

  protected:

    /// Read the file containing initial vlues of registers.
    bool readInitialState(const std::string& path);

    // Extract a pair of addresses from the given field. Return true on
    // success and false on failure.
    bool extractAddressPair(uint64_t lineNum, const char* tag,
			    const char* pairString,
			    uint64_t& virt, uint64_t& phys,
			    bool& masked);

    // Parse the register value in the given value string into the given
    // operand.  Return true on success and false on failure. Update the
    // current values of regisers maintained by this parser.
    bool parseRegValue(uint64_t lineNum, char* regName,
		       char* valStr, Operand& operand);

    // Parse the given operand string into the given operand. Return true on
    // success and false on failure. Assing a value to the operand from the
    // maintained set of values in this parser.
    bool parseOperand(uint64_t lineNum, char* opStr,
		      Operand& operand);

    // Parse the memory string putting the resuts in the given
    // record. Return true on success and false on failure.
    bool parseMem(uint64_t lineNum, char* memStr,
		  TraceRecord& rec);

    bool splitLine(std::string& line, uint64_t lineNum);

  private:

    typedef std::vector<uint8_t> VecReg;

    std::vector<uint64_t> intRegs_;
    std::vector<uint64_t> fpRegs_;
    std::vector<uint64_t> csRegs_;
    std::vector<VecReg>   vecRegs_;

    std::vector<char*> fields_;
    std::vector<char*> subfields_;
    std::vector<char*> keyvals_;

    std::vector<int> indices_; // Map a header tag to an index (field number in line).

    std::string headerLine_; // The original header line
    std::string line_;       // Buffer used to read file records.
    uint64_t lineNum_ = 0;   // Line number in input file.
    unsigned colCount_ = 0;  // Column count.

    PageTableMaker* pageMaker_ = nullptr;

    std::ifstream fileStream_;
    std::istream* input_ = nullptr;
    boost::iostreams::filtering_streambuf<boost::iostreams::input> inStreambuf_;
  };
}

