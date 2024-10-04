#include <TraceReader.hpp>
#include <sstream>
#include <inttypes.h>
#include <cassert>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/hex.hpp>
#include <boost/io/ios_state.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/zstd.hpp>
#include "PageTableMaker.hpp"

using namespace WhisperUtil;


// Log file characters corresponding to the operand types (these
// correspond to the entries in OperandType).
static std::vector<char> operandTypeChar = { 'x', 'f', 'c', 'v', 'i' };

// Log file characters corresponding to privilege mode (thses correspond
// to the entries in PrivMode).
static std::vector<char> privChar = { 'm', 's', 'u' };

// Map a header string to a HeaderTag.
static std::unordered_map<std::string, HeaderTag> headerMap = {
  {"pc",              HeaderTag::Pc },
  {"inst",            HeaderTag::Inst },
  {"modified regs",   HeaderTag::DestRegs },
  {"source operands", HeaderTag::SourceOps },
  {"memory",          HeaderTag::Memory },
  {"inst info",       HeaderTag::InstType },
  {"privilege",       HeaderTag::Priv },
  {"trap",            HeaderTag::Trap },
  {"disassembly",     HeaderTag::Dis },
  {"hartid",          HeaderTag::HartId },
  {"iptw",            HeaderTag::Iptw },
  {"dptw",            HeaderTag::Dptw },
  {"pmp",             HeaderTag::Pmp}
};


TraceReader::TraceReader(const std::string& inputPath)
  : intRegs_(32), fpRegs_(32), csRegs_(4096), vecRegs_(32),
    fileStream_(inputPath.c_str()), input_(nullptr)
{
  for (auto& vecReg : vecRegs_)
    vecReg.push_back(0);
  size_t len = inputPath.size();
  if (len > 3 and inputPath.substr(len - 3) == ".gz")
    {
      inStreambuf_.push(boost::iostreams::gzip_decompressor());
      inStreambuf_.push(fileStream_);
      input_ = new std::istream(&inStreambuf_);
    }
  else if (len > 4 and inputPath.substr(len - 4) == ".bz2")
    {
      inStreambuf_.push(boost::iostreams::bzip2_decompressor());
      inStreambuf_.push(fileStream_);
      input_ = new std::istream(&inStreambuf_);
    }
  else if (len > 4 and inputPath.substr(len - 4) == ".zst")
    {
      inStreambuf_.push(boost::iostreams::zstd_decompressor());
      inStreambuf_.push(fileStream_);
      input_ = new std::istream(&inStreambuf_);
    }
  else
    input_ = new std::istream(fileStream_.rdbuf());
}


TraceReader::TraceReader(const std::string& inputPath, const std::string& initPath)
  : TraceReader(inputPath) {
  readInitialState(initPath);
}


bool
TraceReader::readInitialState(const std::string& path)
{
  using std::cerr;
  std::ifstream in(path);
  if (not in)
    {
      cerr << "Failed to open file '" << path << "' for input\n";
      return false;
    }

  std::string prefix = "File " + path + " line ";

  unsigned lineNum = 0, errors = 0;
  std::string line;
  while (std::getline(in, line))
    {
      lineNum++;
      if (line.empty())
	continue;

      std::istringstream iss(line);
      std::string type, numStr, valStr;
      iss >> type >> numStr >> valStr;

      if (type == "pm" or type == "vm" or type == "po" or type == "pb" or type == "pc")
	continue;

      if (type != "x" and type != "f" and type != "c" and type != "v")
	{
	  cerr << prefix << lineNum << ": Bad register type: " << type << '\n';
	  errors++;
	  continue;
	}
      if (numStr.empty() or valStr.empty())
	{
	  cerr << prefix << lineNum << ": Fewer than 3 tokens in line: " << line << '\n';
	  errors++;
	  continue;
	}

      std::size_t l = 0;
      unsigned num = std::stoul(numStr, &l, 0);
      if (l < numStr.size())
	{
	  cerr << prefix << lineNum << ": Bad register number: " << numStr << '\n';
	  errors++;
	  continue;
	}

      uint64_t val = 0;
      std::vector<uint8_t> vecValue;
      if (type == "v")
	{
	  bool ok = valStr.size() > 2 and boost::starts_with(valStr, "0x");
	  if (ok)
	    try
	      {
		boost::algorithm::unhex(valStr.begin() + 2, valStr.end(), std::back_inserter(vecValue));
	      }
	    catch (...)
	      {
		ok = false;
	      }
	  if (not ok)
	    {
	      cerr << prefix << lineNum << ": Bad vector register value: " << valStr << '\n';
	      errors++;
	      continue;
	    }
	}
      else
	{
	  val = std::stoull(valStr, &l, 0);
	  if (l < valStr.size())
	    {
	      std::cerr << prefix << lineNum << ": Bad value: " << valStr << '\n';
	      errors++;
	      continue;
	    }
	}

      size_t limit = (type == "x" ? intRegs_.size() :
		      (type == "f" ? fpRegs_.size() :
		       (type == "c" ? csRegs_.size() :
			(type == "v" ? vecRegs_.size() : 0))));
      if (num >= limit)
	{
	  std::cerr << prefix << lineNum << ": Reg number out of bounds: " << num << '\n';
	  errors++;
	  continue;
	}

      if (type == "x")
	intRegs_.at(num) = val;
      else if (type == "f")
	fpRegs_.at(num) = val;
      else if (type == "c")
	csRegs_.at(num) = val;
      else if (type == "v")
	vecRegs_.at(num) = vecValue;
    }

  return errors == 0;
}


TraceReader::~TraceReader()
{
  delete input_;
  input_ = nullptr;

  delete pageMaker_;
  pageMaker_ = nullptr;
}


void
TraceRecord::clear()
{
  virtPc = physPc = 0;
  takenBranchTarget = 0;
  inst = 0;
  instSize = 0;
  instType = 0;
  dataSize = 0;
  fpFlags = 0;
  roundingMode = 0;
  modifiedRegs.clear();
  sourceOperands.clear();
  virtAddrs.clear();
  physAddrs.clear();
  memVals.clear();
  maskedAddrs.clear();
  dpteAddrs.clear();
  ipteAddrs.clear();
  contextCSRs.clear();
  priv = PrivMode::Machine;
  virt = false;
  hasTrap = false;
  trap = 0;
  assembly.clear();
}


void
TraceRecord::print(std::ostream& os) const
{
  boost::io::ios_flags_saver restore_flags(os);
  os << "PC=" << std::hex << virtPc;
  if (physPc != virtPc)
    os << ":" << std::hex << physPc;
  os << "  inst=" << std::hex << inst << " size=" << unsigned(instSize);
  if (instType != 0)
    os << " type=" << instType;
  os << " virt=" << virt;
  if (size_t(priv) < privChar.size())
    os << " priv=" << privChar.at(size_t(priv));
  if (hasTrap)
    os << " trap=" << std::hex << trap;
  if (not assembly.empty())
    os << " disas=\"" << assembly << '"';
  os << '\n';

  if (not hasTrap and (instType == 'j' or instType == 'c' or instType == 't'
		       or instType == 'c'))
    os << "  branch_target: " << std::hex << takenBranchTarget << '\n';

  for (const auto& mreg : modifiedRegs)
    {
      os << "  dest: " << operandTypeChar.at(size_t(mreg.type))
	 << std::dec << mreg.number
	 << '=';

      if (mreg.type == OperandType::Vec)
        {
	  auto& vv = mreg.vecValue;
          for (auto it = vv.rbegin(); it != vv.rend(); ++it)
            os << std::hex << std::setw(2) << std::setfill('0') << (unsigned) *it;

          os << " prev=";
	  auto& pv = mreg.vecPrevValue;
          for (auto it = pv.rbegin(); it != pv.rend(); ++it)
            os << std::hex << std::setw(2) << std::setfill('0') << (unsigned) *it;
        }
      else
        os << std::hex << mreg.value << " prev=" << mreg.prevValue;

      os << '\n';
    }

  if (instType == 'f')
    {
      os << "  fp_flags: " << std::hex << unsigned(fpFlags) << '\n';
      os << "  rounding_mode: " << std::hex << unsigned(roundingMode) << '\n';
    }

  for (const auto& src : sourceOperands)
    {
      if (src.type == OperandType::Imm)
	os << "  src: imm=" << std::hex << src.value << '\n';
      else
	{
	  os << "  src: " << operandTypeChar.at(size_t(src.type));
	  os << std::dec << src.number;
          os << '=';
	  if (src.type == OperandType::Vec)
	    {
              for (auto val : src.vecValue)
                os << std::hex << std::setw(2) << std::setfill('0') << (unsigned) val;
	      if (src.emul != 1)
		os << "m" << src.emul;
	    }
	  else
	    os << std::hex << src.value;
	  os << '\n';
	}
    }

  if (not virtAddrs.empty())
    {
      os << (virtAddrs.size() == 1 ? "  mem: "  :  "  mems:\n");
      for (size_t i = 0; i < virtAddrs.size(); ++i)
	{
	  if (virtAddrs.size() > 1)
	    os << "    ";
	  os << std::hex << virtAddrs.at(i);
	  if (i < physAddrs.size() and physAddrs.at(i) != virtAddrs.at(i))
	    os << ":" << std::hex << physAddrs.at(i);
	  if (i < memVals.size())
	    os << "=" << std::hex << memVals.size();
	  if (i < maskedAddrs.size() and maskedAddrs.at(i))
	    os << " masked";
	  os << '\n';
	}
    }

  if (not ipteAddrs.empty())
    {
      const char* sep = "";
      os << "  ipte addrs:";
      for (auto addr : ipteAddrs)
        {
          os << sep << " " << addr;
          sep = ",";
        }
      os << '\n';
    }

  if (not dpteAddrs.empty())
    {
      const char* sep = "";
      os << "  dpte addrs:";
      for (auto addr : dpteAddrs)
        {
          os << sep << " " << addr;
          sep = ",";
        }
      os << '\n';
    }
}


inline
uint64_t
hexStrToNum(const char* x)
{
  uint64_t res = 0, nibble = 0;
  char c;
  while ((c = *x++))
    {
      if (c >= '0' and c <= '9')
	nibble = c - '0';
      else if (c >= 'a' and c <= 'f')
	nibble = 10 + c - 'a';
      else if (c >= 'A' and c <= 'F')
	nibble = 10 + c - 'A';
      else
	break;
      res = (res << 4) | nibble;
    }
  return res;
}


inline
uint64_t
hexStrToNum(const char* x, const char*& rest)
{
  uint64_t res = 0, nibble = 0;
  char c;
  while ((c = *x))
    {
      if (c >= '0' and c <= '9')
	nibble = c - '0';
      else if (c >= 'a' and c <= 'f')
	nibble = 10 + c - 'a';
      else if (c >= 'A' and c <= 'F')
	nibble = 10 + c - 'A';
      else
	break;
      res = (res << 4) | nibble;
      x++;
    }
  rest = x;
  return res;
}


inline
bool
TraceReader::extractAddressPair(uint64_t lineNum, const char* tag,
				const char* pairStr,
				uint64_t& virt, uint64_t& phys,
				bool& masked)
{
  masked = false;

  const char* rest = nullptr;
  virt = hexStrToNum(pairStr, rest);
  phys = virt;
  if (rest == nullptr or *rest == 0)
    return true;

  if (*rest == 'm')
    {
      masked = true;
      rest++;
    }
  if (*rest == 0)
    return true;

  if (*rest == ':')
    {
      rest++;
      phys = hexStrToNum(rest, rest);
      if (rest == nullptr or *rest == 0)
	return true;
    }

  std::cerr << "Line " << lineNum << ": Bad " << tag << " address field: "
	    << pairStr << '\n';
  return false;
}


bool
TraceReader::parseRegValue(uint64_t lineNum, char* regName,
			   char* valStr, Operand& operand)
{
  if (strlen(regName) < 2)
   {
     std::cerr << "Line " << lineNum << ": Empty reg name\n";
     return false;
   }

 bool good = false;

 char rc = regName[0];
 if (rc == 'v')
   boost::algorithm::unhex(valStr, valStr + strlen(valStr), std::back_inserter(operand.vecValue));
 else
   operand.value = hexStrToNum(valStr);

 if ((rc == 'x' or rc == 'f' or rc == 'v' or rc == 'c') and
     std::isdigit(regName[1]))
    {
      char* tail = nullptr;
      unsigned regNum = std::strtoul(regName + 1, &tail, 10);
      good = (rc == 'c') ? regNum < 4096 : regNum < 32;
      if (good)
	{
	  operand.number = regNum;
	  if (rc == 'x')
	    {
	      operand.type = OperandType::Int;
	      operand.prevValue = intRegs_.at(regNum);
	      intRegs_.at(regNum) = operand.value;
	    }
	  else if (rc == 'f')
	    {
	      operand.type = OperandType::Fp;
	      operand.prevValue = fpRegs_.at(regNum);
	      fpRegs_.at(regNum) = operand.value;
	    }
	  else if (rc == 'v')
	    {
	      operand.type = OperandType::Vec;
              operand.vecPrevValue = vecRegs_.at(regNum);
              vecRegs_.at(regNum) = operand.vecValue;
	    }
	  else if (rc == 'c')
	    {
	      operand.type = OperandType::Csr;
	      operand.prevValue = csRegs_.at(regNum);
	      csRegs_.at(regNum) = operand.value;
	    }
	  else
	    good = false;

	  if (tail and *tail != 0)
	    good = false;
	}
    }

 if (good)
   return true;

  std::cerr << "Line " << lineNum << ": Bad reg name: " << regName << '\n';
  return false;
}


bool
TraceReader::parseOperand(uint64_t lineNum, char* opStr, Operand& operand)
{
  if (opStr == nullptr or *opStr == 0)
    return false;

  bool good = false;

  if (strlen(opStr) > 1)
    {
      char rc = opStr[0];
      if (rc == 'i')
	{
	  operand.type = OperandType::Imm;
	  operand.value = hexStrToNum(opStr + 1);
	  return true;
	}

      char* tail = nullptr;
      unsigned regNum = std::strtoul(opStr + 1, &tail, 10);
      good = (rc == 'c') ? regNum < 4096 : regNum < 32;
      if (good)
	{
	  operand.number = regNum;
	  if (rc == 'x')
	    {
	      operand.type = OperandType::Int;
	      operand.value = intRegs_.at(regNum);
	    }
	  else if (rc == 'f')
	    {
	      operand.type = OperandType::Fp;
	      operand.value = fpRegs_.at(regNum);
	    }
	  else if (rc == 'v')
	    {
	      operand.type = OperandType::Vec;
	      if (tail and *tail == 'm')
		{
		  unsigned emul = strtoul(tail+1, &tail, 10);
		  if (emul >= 1 and emul <= 8)
		    operand.emul = emul;
		}
	      operand.vecValue = vecRegs_.at(regNum);
	    }
	  else if (rc == 'c')
	    {
	      operand.type = OperandType::Csr;
	      operand.value = csRegs_.at(regNum);
	    }
	  else
	    good = false;

	  if (tail and *tail != 0)
	    good = false;
	}
    }

  if (good)
    return true;

  std::cerr << "Line " << lineNum << ": Bad reg name: " << opStr << '\n';
  return false;
}


// Split string around given char. Put resutls in given vector.
inline
void
mySplit(std::vector<std::string>& result, const std::string& str, char c)
{
  result.clear();
  size_t prev = 0, pos = 0;
  while ((pos = str.find(c, prev)) != std::string::npos)
    {

      std::string token = str.substr(prev, pos - prev);
      prev = pos + 1;
      result.push_back(token);
    }
  if (prev < str.length())
    result.push_back(str.substr(prev));
}


// Split string around given char. Put resutls in given vector.
inline
void
mySplit(std::vector<char*>& result, char* str, char c)
{
  result.clear();
  if (str == nullptr or *str == 0)
    return;

  size_t prev = 0;
  size_t len = strlen(str);
  for (size_t i = 0; i < len; ++i)
    if (str[i] == c)
      {
	char* token = str + prev;
	str[i] = 0;
	result.push_back(token);
	prev = i+1;
      }
  if (prev < len)
    result.push_back(str + prev);
}


bool
TraceReader::parseMem(uint64_t lineNum, char* memStr, TraceRecord& rec)
{
  if (memStr == nullptr or *memStr == 0)
    return false;

  mySplit(subfields_, memStr, ';');
  for (const auto& entry : subfields_)
    {
      mySplit(keyvals_, entry, '=');
      if (keyvals_.size() == 1 or keyvals_.size() == 2)
	{
	  uint64_t virt = 0, phys = 0;
	  bool masked = false;
	  if (not extractAddressPair(lineNum, "Memory", keyvals_.at(0),
				     virt, phys, masked))
	    return false;
	  rec.virtAddrs.push_back(virt);
	  rec.physAddrs.push_back(phys);
	  rec.maskedAddrs.push_back(masked);
	  if (keyvals_.size() == 2)
	    {
	      uint64_t val = hexStrToNum(keyvals_.at(1));
	      rec.memVals.push_back(val);
	    }
	}
      else
	{
	  std::cerr << "Line " << lineNum << ": Bad memory filed: " << memStr << '\n';
	  return false;
	}
    }

  if (rec.memVals.size() > rec.virtAddrs.size())
    {
      std::cerr << "Line " << lineNum << ": Memory value count greater than address count\n";
      return false;
    }

  return true;
}


bool
TraceReader::splitLine(std::string& line, uint64_t lineNum)
{
  // We avoid boost::split and mySplit for speed.

  fields_.resize(colCount_);

  size_t prev = 0, cc = 0;
  for (size_t i = 0; i < line.size(); ++i)
    if (line[i] == ',')
      {
	if (cc >= colCount_)
	  break;
	fields_.at(cc) = &line.at(prev);
	line[i] = 0;
	prev = i+1;
	cc++;
      }

  if (cc < colCount_)
    {
      if (prev < line.size())
	fields_.at(cc++) = &line.at(prev);
      else if (prev == line.size() and line.at(prev-1) == 0)
	fields_.at(cc++) = &line.at(prev-1);
    }

  if (cc != colCount_)
    {
      std::cerr << "Line " << lineNum << ": Col count (" << cc
		<< ") different from that of header (" << colCount_ << ")\n";
      return false;
    }
  return true;
}


static
void
determineVecDataSize(TraceRecord& record, uint64_t vtype)
{
  unsigned imm = (record.inst >> 20) & 0xfff; // Top 12 bits
  uint32_t f3 = (record.inst >> 12) & 7;
  unsigned lumop =  imm & 0x1f;
  unsigned mop = (imm >> 6) & 3;
  unsigned mew = (imm >> 8) & 1;
  unsigned dataSize = 0;
  unsigned elemWidth = 0;

  unsigned sew = (vtype >> 3) & 7;
  if (sew == 0) elemWidth = 1;
  if (sew == 1) elemWidth = 2;
  if (sew == 2) elemWidth = 4;
  if (sew == 3) elemWidth = 8;

  // Set data size to the vector element size
  if (mop == 0)
    {      // Unit stride
      if (lumop == 0 or lumop == 0x8 or lumop == 0x10)  // 0x10 is load only
        {
	  if (f3 == 0) dataSize = 1;
	  if (f3 == 5) dataSize = 2;
	  if (f3 == 6) dataSize = 4;
	  if (f3 == 7) dataSize = 8;
	  if (mew == 1) dataSize *= 16;
        }
      else if (lumop == 0xb and mew == 0 and f3 == 0)
	{ // vlm.v or vsm.v
	  dataSize = 1;
	}
    }
  else if (mop == 1 or mop == 3)
    {      // indexed unordered or indexed
      if (mew == 0)
        dataSize = elemWidth;
    }
  else if (mop == 2)
    {      // Strided
      if (f3 == 0) dataSize = 1;
      if (f3 == 5) dataSize = 2;
      if (f3 == 6) dataSize = 4;
      if (f3 == 7) dataSize = 8;
      if (mew == 1) dataSize *= 16;
    }

  record.dataSize = dataSize;
}


static
void
determineDataSize(TraceRecord& record, const std::vector<uint64_t>& csRegs)
{
  if (record.instType == 'v')
    determineVecDataSize(record, csRegs.at(0xc21));
  else
    {
      if (record.instSize == 4)
	{
	  unsigned sizeCode = (record.inst >> 12) & 3;
	  if      (sizeCode == 0) record.dataSize = 1;
	  else if (sizeCode == 1) record.dataSize = 2;
	  else if (sizeCode == 2) record.dataSize = 4;
	  else if (sizeCode == 3) record.dataSize = 8;
	}
      else if (record.instSize == 2)
	{
	  unsigned f3 = (record.inst >> 13) & 7;
	  unsigned quad = record.inst & 3; // Opcode quadrant
	  if (quad == 0 or quad == 2)
	    {
	      if (f3 == 1 or f3 == 3 or f3 == 5 or f3 == 7)
		record.dataSize = 8;
	      else if (f3 == 2 or f3 == 6)
		record.dataSize = 4;
              else if (f3 == 4)
                {
                  unsigned f6 = (record.inst >> 10) & 0x3f;
                  if (f6 == 0x20)
                    record.dataSize = 1;
                  else if (f6 == 0x21)
                    record.dataSize = 2;
                  else if (f6 == 0x22)
                    record.dataSize = 1;
                  else if (f6 == 0x23)
                    record.dataSize = 2;
                }
	    }
	}
    }
}


bool
TraceReader::parseLine(std::string& line, uint64_t lineNum, TraceRecord& record)
{
  record.clear();
  if (line.empty())
    return false;

  // Fplit line around commas putting results in fields_.
  if (not splitLine(line, lineNum))
    return false;

  // PC
  int ix = indices_.at(size_t(HeaderTag::Pc));
  if (ix >= 0)
    {
      bool masked = false;
      if (not extractAddressPair(lineNum, "PC", fields_.at(ix),
				 record.virtPc, record.physPc, masked))
	return false;
    }

  // Instruction.
  ix = indices_.at(size_t(HeaderTag::Inst));
  if (ix >= 0)
    {
      const char* instStr = fields_.at(ix);
      record.inst = hexStrToNum(instStr);
      record.instSize = (record.inst & 3) == 3 ? 4 : 2;
    }

  // Source operands. This must be parsed before modified registers to get
  // the correct values for a register used as both source and target.
  ix = indices_.at(size_t(HeaderTag::SourceOps));
  if (ix >= 0)
    {
      char* sourceOps = fields_.at(ix);
      if (*sourceOps)
	{
	  mySplit(subfields_, sourceOps, ';');
	  for (auto source : subfields_)
	    {
	      if (strncmp(source, "rm=", 3) == 0)
		{
		  char* rms = source + 3;
		  record.roundingMode = hexStrToNum(rms);
		  continue;
		}
	      Operand operand;
	      if (not parseOperand(lineNum, source, operand))
		return false;
	      record.sourceOperands.push_back(operand);
	    }
	}
    }

  // Modified regs.
  ix = indices_.at(size_t(HeaderTag::DestRegs));
  if (ix >= 0)
    {
      char* regs = fields_.at(ix);
      if (*regs)
	{
	  mySplit(subfields_, regs, ';');
	  for (const auto& reg : subfields_)
	    {
	      mySplit(keyvals_, reg, '=');
	      if (keyvals_.size() != 2)
		{
		  std::cerr << "Line " << lineNum << ": Bad register change field: " << reg
			    << ", expecting: <reg>=<value>\n";
		  return false;
		}
	      if (strcmp(keyvals_.at(0), "pc") == 0)
		{
		  record.takenBranchTarget = hexStrToNum(keyvals_.at(1));
		  continue;
		}
	      if (strcmp(keyvals_.at(0), "ff") == 0)
		{
		  record.fpFlags = hexStrToNum(keyvals_.at(1));
		  continue;
		}
	      Operand operand;
	      if (not parseRegValue(lineNum, keyvals_.at(0), keyvals_.at(1), operand))
		return false;
	      record.modifiedRegs.push_back(operand);
	    }
	}
    }

  // Instruction type.
  ix = indices_.at(size_t(HeaderTag::InstType));
  if (ix >= 0)
    {
      char* itype = fields_.at(ix);
      if (*itype)
        record.instType = *itype;
    }

  // Memory.
  ix = indices_.at(size_t(HeaderTag::Memory));
  if (ix >= 0)
    {
      char* mem = fields_.at(ix);
      if (mem and *mem)
	{
	  if (not parseMem(lineNum, mem, record))
	    return false;
	  determineDataSize(record, csRegs_);
	}
    }

  // Privilege elvel.
  ix = indices_.at(size_t(HeaderTag::Priv));
  if (ix >= 0)
    {
      char* priv = fields_.at(ix);
      if (*priv)
	{
          record.virt = *priv == 'v';
	  if (*priv == 'm') record.priv = PrivMode::Machine;
	  else if (strchr(priv, 's')) record.priv = PrivMode::Supervisor;
	  else if (strchr(priv, 'u')) record.priv = PrivMode::User;
	}
    }

  // Trap.
  ix = indices_.at(size_t(HeaderTag::Trap));
  if (ix >= 0)
    {
      char* trap = fields_.at(ix);
      if (*trap)
	{
	  record.hasTrap = true;
	  record.trap = hexStrToNum(trap);
	}
    }

  // Disassembly
  ix = indices_.at(size_t(HeaderTag::Dis));
  if (ix >= 0)
    {
      for (char* str = fields_.at(ix); *str; ++str)
	if (*str == ';')
	  *str = ',';
      record.assembly = fields_.at(ix);
    }

  // I-page table walk.
  ix = indices_.at(size_t(HeaderTag::Iptw));
  if (ix >= 0)
    {
      char* ptw = fields_.at(ix);
      if (*ptw)
        {
          mySplit(subfields_, ptw, ';');
          for (const auto& addr : subfields_)
            {
	      mySplit(keyvals_, addr, '=');
              if (keyvals_.size() == 1)
                continue;
              if (strcmp(keyvals_.at(0), "ma") == 0)
                continue;
              if (keyvals_.size() > 2)
                {
		  std::cerr << "Line " << lineNum << ": Bad ptw field: " << addr
			    << ", expecting: <addr>=<pte> or <addr>\n";
                  return false;
                }
              record.ipteAddrs.push_back(hexStrToNum(keyvals_.at(0)));
            }
        }
    }

  // D-page table walk
  ix = indices_.at(size_t(HeaderTag::Dptw));
  if (ix >= 0)
    {
      char* ptw = fields_.at(ix);
      if (*ptw)
        {
          mySplit(subfields_, ptw, ';');
          for (const auto& addr : subfields_)
            {
	      mySplit(keyvals_, addr, '=');
              if (keyvals_.size() == 1)
                continue;
              if (strcmp(keyvals_.at(0), "ma") == 0)
                continue;
              if (keyvals_.size() > 2)
                {
		  std::cerr << "Line " << lineNum << ": Bad ptw field: " << addr
			    << ", expecting: <addr>=<pte> or <addr>\n";
                  return false;
                }
              record.dpteAddrs.push_back(hexStrToNum(keyvals_.at(0)));
            }
        }
    }

  return true;
}


bool
TraceReader::extractHeaderIndices(const std::string& line, uint64_t lineNum)
{
  // Initialize all indices to -1.
  indices_.resize(size_t(HeaderTag::_Count), -1);

  headerLine_ = line;

  std::vector<std::string> cols;
  mySplit(cols, line, ',');

  for (size_t i = 0; i < cols.size(); ++i)
    {
      std::string tag = cols.at(i);
      boost::trim(tag);
      auto iter = headerMap.find(tag);
      if (iter == headerMap.end())
	{
	  std::cerr << "Line " << lineNum << ": Unknown tag: " << tag << '\n';
	  continue;
	}
      size_t ix = size_t(iter->second);
      indices_.at(ix) = i;
    }
  colCount_ = cols.size();

  if (colCount_ == 0)
    {
      std::cerr << "Line " << lineNum << ": Empty header line.\n";
      return false;
    }
  if (colCount_ > 512)
    {
      std::cerr << "Line " << lineNum << ": Too many columns in header line: " << colCount_ << '\n';
      return false;
    }

  return true;
}

std::string
TraceReader::getHeaderLine()
{
  return headerLine_;
}

bool
TraceReader::nextRecord(TraceRecord& record)
{
  if (lineNum_ == 0)
    {
      // Process header line.
      lineNum_++;
      if (not std::getline(*input_, line_))
	return false;
      if (not extractHeaderIndices(line_, lineNum_))
	return false;
    }

  // Process a non-header record.
  lineNum_++;
  if (not std::getline(*input_, line_))
    return false;

  if (not parseLine(line_, lineNum_, record))
    return false;

  return true;
}

bool
TraceReader::nextRecord(TraceRecord& record, std::string& line)
{
  if (lineNum_ == 0)
    {
      // Process header line.
      lineNum_++;
      if (not std::getline(*input_, line_))
	return false;
      if (not extractHeaderIndices(line_, lineNum_))
	return false;
    }

  // Process a non-header record.
  lineNum_++;
  if (not std::getline(*input_, line_))
    return false;

  line = line_;

  if (not parseLine(line_, lineNum_, record))
    return false;

  return true;
}


template<class Mode>
bool
TraceReader::definePageTableMaker(uint64_t addr,
				  Mode mode,
				  uint64_t arenaSize)
{
  delete pageMaker_;
  pageMaker_ = nullptr;

  unsigned pageSize = 4096;

  if ((addr % pageSize) != 0)
    return false;

  if ((arenaSize % pageSize) != 0)
    return false;

  if (arenaSize < pageSize)
    return false;

  pageMaker_ = new PageTableMaker(addr, mode, arenaSize);
  return true;
}


bool
TraceReader::genPageTableWalk(uint64_t vaddr, uint64_t paddr,
			      std::vector<uint64_t>& walk)
{
  if (not pageMaker_)
    return false;
  return pageMaker_->makeWalk(vaddr, paddr, walk);
}
