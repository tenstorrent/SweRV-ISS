#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <unistd.h>
#include <fcntl.h>
#include "Filesystem.hpp"
#include "Hart.hpp"


using namespace WdRiscv;


template <typename URV>
bool
Hart<URV>::saveSnapshotRegs(const std::string & filename)
{
  // open file for write, check success
  std::ofstream ofs(filename, std::ios::trunc);
  if (not ofs)
    {
      std::cerr << "Hart::saveSnapshotRegs failed - cannot open " << filename << " for write\n";
      return false;
    }

  // Write Privilege Mode, Program Order, Program Break, and Program Counter.
  ofs << "pm " << std::dec << unsigned(privilegeMode()) << '\n';
  ofs << "po " << std::dec << getInstructionCount() << '\n';
  ofs << "pb 0x" << std::hex << syscall_.targetProgramBreak() << std::dec << '\n';
  ofs << "pc 0x" << std::hex << peekPc() << std::dec << '\n';

  // write integer registers
  for (unsigned i = 1; i < 32; i++)
    ofs << "x " << std::dec << i << " 0x" << std::hex << peekIntReg(i) << std::dec << "\n";

  // write floating point registers
  for (unsigned i = 0; i < 32; i++)
    {
      uint64_t val = 0;
      peekFpReg(i, val);
      ofs << "f " << std::dec << i << " 0x" << std::hex << val << std::dec << "\n";
    }

  // write control & status registers
  for (unsigned i = unsigned(CsrNumber::MIN_CSR_); i <= unsigned(CsrNumber::MAX_CSR_); i++)
    {
      URV val = 0;
      if (not peekCsr(CsrNumber(i), val))
        continue;
      ofs << "c 0x" << std::hex << i << " 0x" << val << std::dec << "\n";
    }

  // write vector registers.
  std::vector<uint8_t> vecBytes;
  for (unsigned i = 0; i < vecRegCount(); ++i)
    {
      peekVecReg(i, vecBytes);
      ofs << "v " << std::dec << i << " 0x";
      ofs << std::hex;
      for (auto byte : vecBytes)
	ofs << std::setw(2) << std::setfill('0') << unsigned(byte);
      ofs << std::dec;
      ofs << '\n';
    }

  ofs.close();
  return true;
}


/// Read an integer value fromt the given stream. Return true on
/// success and false on failure. Use strtoull to do the string to
/// integer conversion so that numbers starting with 0 or 0x are
/// interpreted as octal or hexadecimal.
static
bool
loadSnapshotValue(std::istream& stream, uint64_t& val)
{
  std::string str;
  if (not (stream >> str))
    return false;
  char* extra = nullptr;
  val = strtoull(str.c_str(), &extra, 0);
  if (extra and *extra)
    return false;
  return true;
}


/// Read from the given stream a register number and a register value.
/// Return true on success and false on failure. Use strtoull to do
/// the string to integer conversions so that numbers starting with 0
/// or 0x are interpreted as octal or hexadecimal.
static
bool
loadRegNumAndValue(std::istream& stream, unsigned& num, uint64_t& val)
{
  std::string str;
  if (not (stream >> str))
    return false;
  char* extra = nullptr;
  num = strtoull(str.c_str(), &extra, 0);
  if (extra and *extra)
    return false;
  return loadSnapshotValue(stream, val);
}


/// Read from the given stream a vector register number and a register value.
/// Return true on success and false on failure. Register value must have
/// with a 0x prefix and must be a hexadecimal number. The most sig digits
/// of the register value are placed in the first entry of vecBytes.
static
bool
loadVecRegNumAndValue(std::istream& stream, unsigned& num,
		      std::vector<uint8_t>& vecBytes)
{
  vecBytes.clear();

  std::string str;
  if (not (stream >> str))
    return false;

  char* extra = nullptr;
  num = strtoull(str.c_str(), &extra, 0);
  if (extra and *extra)
    return false;

  std::string vvs;  // vector value string
  if (not (stream >> vvs))
    return false;

  if (not (vvs.starts_with("0x") or vvs.starts_with("0X")))
    return false;

  vvs = vvs.substr(2);  // Skip leading 0x

  for (auto c : vvs)
    if (not std::isxdigit(c))
      return false;

  if ((vvs.size() & 1) != 0)
    vvs.insert(vvs.begin(), '0');  // Make hex digit count even by prepeindg a '0'.

  vecBytes.resize(vvs.size() / 2);

  for (size_t i = 0; i < vvs.size(); ++i)
    {
      unsigned hd = vvs[i], nibble = 0;
      if      (hd >= '0' and hd <= '9') nibble = hd - '0';
      else if (hd >= 'a' and hd <= 'f') nibble = 10 + hd - 'a';
      else if (hd >= 'A' and hd <= 'F') nibble = 10 + hd - 'A';
      if ((i & 1) == 0)
	vecBytes[i/2] |= (nibble << 4);
      else
	vecBytes[i/2] |= nibble;
    }

  return true;
}


template <typename URV>
bool
Hart<URV>::loadSnapshotRegs(const std::string & filename)
{
  // open file for read, check success
  std::ifstream ifs(filename);
  if (not ifs)
    {
      std::cerr << "Hart::loadSnapshotRegs failed - cannot open " << filename << " for read\n";
      return false;
    }
  // read line by line and set registers
  std::string line;
  unsigned lineNum = 0;
  unsigned num = 0;
  uint64_t val = 0;
  bool error = false;
  auto privMode = PrivilegeMode::Machine;

  while(std::getline(ifs, line))
    {
      lineNum++;
      std::istringstream iss(line);
      std::string type;
      error = true;
      // parse first part (register class)
      if (not (iss >> type))
        break; // error: parse failed

      if (type == "pc")     // PC
        {
          if (not loadSnapshotValue(iss, val))
            break; // error: parse failed
          pokePc(val);
        }
      else if (type == "pm")  // Prrivilege Mode
	{
	  if (not loadSnapshotValue(iss, val))
	    break;
	  if (val == 0)
	    privMode = PrivilegeMode::User;
	  else if (val == 1)
	    privMode = PrivilegeMode::Supervisor;
	  else if (val == 3)
	    privMode = PrivilegeMode::Machine;
	  else 
	    break; // error.
	}
      else if (type == "po")  // Program order
        {
          if (not loadSnapshotValue(iss, val))
            break;
          setInstructionCount(val);
        }
      else if (type == "pb")  // Program break
        {
          if (not loadSnapshotValue(iss, val))
            break;
          setTargetProgramBreak(val);
        }
      else if (type == "c")   // CSR
        {
          if (not loadRegNumAndValue(iss, num, val))
            break;
          if (not pokeCsr(CsrNumber(num), val))
	    std::cerr << "Warning: Register snapshot loader: Line " << lineNum
		      << ": No such CSR: " << line << '\n';
        }
      else if (type.compare("x") == 0)   // Integer register
        {  
          if (not loadRegNumAndValue(iss, num, val))
            break;
          if (not pokeIntReg(num, val))
	    std::cerr << "Warning: Register snapshot loader: Line " << lineNum
		      << ": No such register: " << line << '\n';
        }
      else if (type.compare("f") == 0)   // FP register
        {
          if (isRvf() or isRvd())
            {
              if (not loadRegNumAndValue(iss, num, val))
                break;
              if (not pokeFpReg(num, val))
		std::cerr << "Warning: Register snapshot loader: Line " << lineNum
			  << ": No such FP register: " << line << '\n';
            }
        }
      else if (type.compare("v") == 0) // Vector registers
	{
	  if (isRvv())
	    {
	      std::vector<uint8_t> vecBytes;
	      if (not loadVecRegNumAndValue(iss, num, vecBytes))
		break;
	      if (not pokeVecReg(num, vecBytes))
		std::cerr << "Warning: Register snapshot loader: Line " << lineNum
			  << ": No such vector register: " << line << '\n';
	    }
	}
      else
        break;  // error: parse failed
      error = false;
    }

  setPrivilegeMode(privMode);

  if (error)
    {
      std::cerr << "Hart::loadSnapshotRegs failed to parse " << filename << ":"
                << std::dec << lineNum << "\n";
      std::cerr << "\t" << line << "\n";
    }
  ifs.close();
  return not error;
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
