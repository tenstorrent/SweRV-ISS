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
#include <string>
#include <cmath>
#include <cstdlib>
#include <boost/algorithm/string.hpp>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <elfio/elfio.hpp>
#include <zlib.h>
#ifdef LZ4_COMPRESS
#include <lz4frame.h>
#endif
#include "Memory.hpp"
#include "wideint.hpp"

using namespace WdRiscv;


inline bool
isPowerOf2(uint64_t x)
{
  return x != 0 and (x & (x-1)) == 0;
}


Memory::Memory(uint64_t size, uint64_t pageSize)
  : size_(size), data_(nullptr), pageSize_(pageSize), reservations_(1),
    lastWriteData_(1), pmaMgr_(size)
{ 
  assert(size >= pageSize);
  assert(pageSize >= 64);

  assert(isPowerOf2(pageSize));

  pageShift_ = static_cast<unsigned>(std::log2(pageSize_));

#ifndef MEM_CALLBACKS

  void* mem = mmap(nullptr, size_, PROT_READ | PROT_WRITE,
		   MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
  if (mem == (void*) -1)
    {
      std::cerr << "Failed to map " << size_ << " bytes using mmap.\n";
      throw std::runtime_error("Out of memory");
    }

  data_ = reinterpret_cast<uint8_t*>(mem);

#endif

}


Memory::~Memory()
{
  if (data_)
    {
      munmap(data_, size_);
      data_ = nullptr;
    }

  if (not dataLineFile_.empty())
    saveDataAddressTrace(dataLineFile_);

  if (not instrLineFile_.empty())
    saveInstructionAddressTrace(instrLineFile_);
}


bool
Memory::loadHexFile(const std::string& fileName)
{
  std::ifstream input(fileName);

  if (not input.good())
    {
      std::cerr << "Failed to open hex-file '" << fileName << "' for input\n";
      return false;
    }

  size_t addr = 0, errors = 0, unmappedCount = 0;
  size_t oob = 0; // Out of bounds addresses

  std::string line;

  for (unsigned lineNum = 0; std::getline(input, line); ++lineNum)
    {
      boost::algorithm::trim(line);
      if (line.empty())
	continue;

      if (line[0] == '@')
	{
	  if (line.size() == 1)
	    {
	      std::cerr << "File " << fileName << ", Line " << lineNum << ": "
			<< "Invalid hexadecimal address: " << line << '\n';
	      errors++;
	      continue;
	    }
	  char* end = nullptr;
	  addr = std::strtoull(line.c_str() + 1, &end, 16);
	  if (end and *end and not isspace(*end))
	    {
	      std::cerr << "File " << fileName << ", Line " << lineNum << ": "
			<< "Invalid hexadecimal address: " << line << '\n';
	      errors++;
	    }
	  continue;
	}

      std::istringstream iss(line);
      uint32_t value = 0;
      while (iss)
	{
	  iss >> std::hex >> value;
	  if (iss.fail())
	    {
	      std::cerr << "File " << fileName << ", Line " << lineNum << ": "
			<< "Invalid data: " << line << '\n';
	      errors++;
	      break;
	    }
	  if (value > 0xff)
	    {
	      std::cerr << "File " << fileName << ", Line " << lineNum << ": "
			<< "Invalid value: " << std::hex << value << '\n'
			<< std::dec;
	      errors++;
	    }
	  if (addr < size_)
	    {
	      if (not errors)
		{
                  if (not initializeByte(addr, value & 0xff))
                    {
                      if (unmappedCount == 0)
                        std::cerr << "Failed to copy HEX file byte at address 0x"
                                  << std::hex << addr << std::dec
                                  << ": corresponding location is not mapped\n";
                      unmappedCount++;
                      if (checkUnmappedElf_)
                        return false;
                    }
                  addr++;
		}
	    }
	  else
	    {
              if (not oob)
                std::cerr << "File " << fileName << ", Line " << lineNum << ": "
                          << "Warning: Address out of bounds: "
                          << std::hex << addr << '\n' << std::dec;
	      oob++;
	    }
	  if (iss.eof())
	    break;
	}

      if (iss.bad())
	{
	  std::cerr << "File " << fileName << ", Line " << lineNum << ": "
		    << "Failed to parse data line: " << line << '\n';
	  errors++;
	}
    }

  if (oob > 1)
    std::cerr << "File " << fileName << ": Warning: File contained "
              << oob << " out of bounds addresses.\n";

  // In case writing ELF data modified last-written-data associated
  // with each hart.
  for (unsigned hartId = 0; hartId < reservations_.size(); ++hartId)
    clearLastWriteInfo(hartId);

  return errors == 0;
}


bool
Memory::loadBinaryFile(const std::string& fileName, uint64_t addr)
{
  std::ifstream input(fileName, std::ios::binary);

  if (not input.good())
    {
      std::cerr << "Failed to open binary file '" << fileName << "' for input\n";
      return false;
    }

  // unmapped and out of bounds addresses
  size_t unmappedCount = 0, oob = 0, num = 0;

  char b;
  while (input.get(b))
    {
      if (addr < size_)
        {
          if (not initializeByte(addr, b))
            {
              if (unmappedCount == 0)
                std::cerr << "Failed to copy binary file byte at address 0x"
                          << std::hex << addr << std::dec
                          << ": corresponding location is not mapped\n";
              unmappedCount++;
              if (checkUnmappedElf_)
                return false;
            }
          addr++;
        }
      else
        {
          if (not oob)
            std::cerr << "File " << fileName << ", Byte " << num << ": "
                      << "Warning: Address out of bounds: "
                      << std::hex << addr << '\n' << std::dec;
          oob++;
        }
      num++;
    };


  if (oob > 1)
    std::cerr << "File " << fileName << ": Warning: File contained "
              << oob << " out of bounds addresses.\n";

  // In case writing ELF data modified last-written-data associated
  // with each hart.
  for (unsigned hartId = 0; hartId < reservations_.size(); ++hartId)
    clearLastWriteInfo(hartId);

  return true;
}

#ifdef LZ4_COMPRESS
std::pair<std::unique_ptr<uint8_t[]>, size_t>
Memory::loadFile(const std::string& filename)
{
  std::streampos length;
  std::ifstream f(filename, std::ios::binary);
  if (f.fail())
    throw std::runtime_error("Failed to load LZ4 file");

  f.seekg(0, std::ios::end);
  length = f.tellg();
  f.seekg(0, std::ios::beg);

  auto data = std::make_unique<uint8_t[]>(length);
  f.read((char *)&data[0], length);

  return std::make_pair(std::move(data), std::move(length));
}

#define BLOCK_SIZE (4*1024*1024)

bool
Memory::loadLz4File(const std::string& fileName, uint64_t addr)
{
  LZ4F_dctx *dctx;
  LZ4F_errorCode_t ret = LZ4F_createDecompressionContext(&dctx, LZ4F_VERSION);
  if (LZ4F_isError(ret))
    {
      throw std::runtime_error("Couldn't initialize LZ4 context");
    }

  auto [src, src_size] = loadFile(fileName);
  size_t dst_size = BLOCK_SIZE;
  auto dst = std::make_unique<uint8_t[]>(dst_size);
  size_t src_offset = 0;

  // unmapped and out of bounds addresses
  size_t unmappedCount = 0, oob = 0, num = 0;

  while (src_size)
    {
      size_t src_bytes_read = src_size;
      size_t dst_bytes_written = dst_size;

      size_t ret = LZ4F_decompress(dctx, dst.get(), &dst_bytes_written, &src[src_offset], &src_bytes_read, NULL);
      if (LZ4F_isError(ret))
	{
	  throw std::runtime_error("LZ4F_decompress failed");
	}

      for (size_t n = 0; n < dst_bytes_written; n++)
	{
	  char b = dst[n];

	  if (addr < size_)
	    {
	      // Speed things up but not initalizing zero bytes
	      if (b and not initializeByte(addr, b))
		{
		  if (unmappedCount == 0)
		    std::cerr << "Failed to copy binary file byte at address 0x"
			      << std::hex << addr << std::dec
			      << ": corresponding location is not mapped\n";
		  unmappedCount++;
		  if (checkUnmappedElf_)
		    return false;
		}
	      addr++;
	    }
	  else
	    {
	      if (not oob)
		std::cerr << "File " << fileName << ", Byte " << num << ": "
			  << "Warning: Address out of bounds: "
			  << std::hex << addr << '\n' << std::dec;
	      oob++;
	    }
	  num++;
	}

      src_offset = src_offset + src_bytes_read;
      src_size = src_size - src_bytes_read;
    }

  if (oob > 1)
    std::cerr << "File " << fileName << ": Warning: File contained "
	      << oob << " out of bounds addresses.\n";

  // In case writing ELF data modified last-written-data associated
  // with each hart.
  for (unsigned hartId = 0; hartId < reservations_.size(); ++hartId)
    clearLastWriteInfo(hartId);

  return true;
}
#endif

bool
Memory::loadElfSegment(ELFIO::elfio& reader, int segIx, uint64_t& end)
{
  const ELFIO::segment* seg = reader.segments[segIx];
  ELFIO::Elf64_Addr paddr = seg->get_physical_address();
  ELFIO::Elf_Xword segSize = seg->get_file_size(); // Size in file.
  end = 0;
  if (seg->get_type() != PT_LOAD)
    return true;

  if (paddr + seg->get_memory_size() > size_)
    {
      std::cerr << "End of ELF segment " << segIx << " (0x"
                << std::hex << (paddr+segSize)
                << ") is beyond end of simulated memory (0x"
                << size_ << ")\n" << std::dec;
      if (checkUnmappedElf_)
        return false;
    }

  size_t unmappedCount = 0;

#if 0

  // Load sections of segment. This is not ideal since it fails to load
  // orphaned data (data not belonging to any section).
  auto segSecCount = seg->get_sections_num();
  for (int secOrder = 0; secOrder < segSecCount; ++secOrder)
    {
      auto secIx = seg->get_section_index_at(secOrder);
      auto sec = reader.sections[secIx];
      const char* secData = sec->get_data();
      if (not secData)
        continue;

      size_t size = sec->get_size();
      size_t addr = sec->get_address();

      for (size_t i = 0; i < size; ++i)
        {
          if (not initializeByte(addr + i, secData[i]))
            {
              if (unmappedCount == 0)
                std::cerr << "Failed to copy ELF byte at address 0x"
                          << std::hex << (addr + i) << std::dec
                          << ": corresponding location is not mapped\n";
              unmappedCount++;
              if (checkUnmappedElf_)
                return false;
            }
        }

      #if 0
      // Debug code. Dump on standard output in verilog hex format.
      printf("@%lx\n", addr);
      size_t remain = size;
      while (remain)
        {
          size_t chunk = std::min(remain, size_t(16));
          const char* sep = "";
          for (size_t ii = 0; ii < chunk; ++ii)
            {
              printf("%s%02x", sep, (*secData++) & 0xff);
              sep = " ";
            }
          printf("\n");
          remain -= chunk;
        }
      #endif
    }

#else

  // Load segment directly.
  const char* segData = seg->get_data();
  for (size_t i = 0; i < segSize; ++i)
    {
      if (not initializeByte(paddr + i, segData[i]))
        {
          if (unmappedCount == 0)
            std::cerr << "Failed to copy ELF byte at address 0x"
                      << std::hex << (paddr + i) << std::dec
                      << ": corresponding location is not mapped\n";
          unmappedCount++;
          if (checkUnmappedElf_)
            return false;
        }
    }

#endif

  end = paddr + uint64_t(seg->get_memory_size());
  return true;
}


/// Extract an unsigned little-endian length encoded 128-bit value from given
/// stream.  Return true on success and afalse on failure.
/// See: https://en.wikipedia.org/wiki/LEB128
static
bool
extractUleb128(std::istream& in, Uint128& value)
{
  value = 0;
  uint8_t byte = 0;
  unsigned shift = 0;
  unsigned count = 0;

  while (in.read((char*) &byte, 1) and count < 19)
    {
      uint8_t msb = byte >> 7;  // Most sig bit
      byte = (byte << 1) >> 1;  // Clear most sig bit
      value = value | (Uint128(byte) << shift);
      shift += 8;
      count++;
      if (not msb)
        return true;
    }

  return false;
}


bool
Memory::collectElfRiscvTags(const std::string& fileName,
                            std::vector<std::string>& tags)
{
  ELFIO::elfio reader;

  if (not reader.load(fileName))
    {
      std::cerr << "Error: Failed to load ELF file " << fileName << '\n';
      return false;
    }

  for (const auto* sec : reader.sections)
    {
      if (sec->get_type() != 0x70000003)
        continue;

      const char* secData = sec->get_data();
      size_t size = sec->get_size();
      if (not secData or not size)
        continue;

      // 1st char is format verion. Currently supported version is 'A'.
      std::string dataString(secData, size);
      std::istringstream iss(dataString);
      char version;
      iss.read(&version, 1);
      if (not iss or version != 'A')
        {
          std::cerr << "Unknown ELF RISCV section format: '" << version << "'\n";
          return false;
        }

      // Next is a 4-byte section length.
      uint32_t secLen = 0;
      iss.read((char*) &secLen, sizeof(secLen));

      // Next is a null terminated string containing vendor name.
      std::string vendorName;
      std::getline(iss, vendorName, '\0');

      // Next is tag: file (1), section(2) or symbol(3).
      uint8_t tag = 0;
      iss.read((char*) &tag, sizeof(tag));
      if (not iss or tag != 1)
        {
          std::cerr << "Unexpected ELF RISCV section tag: " << tag << "(expecting 1)\n";
          return false;
        }

      // Next is a 4-byte attributes size including tag and size.
      // https://embarc.org/man-pages/as/RISC_002dV_002dATTRIBUTE.html#RISC_002dV_002dATTRIBUTE
      uint32_t attribsSize = 0;
      iss.read((char*) &attribsSize, sizeof(attribsSize));
      if (not iss)
        {
          std::cerr << "Corrupted ELF RISCV file attributes subsection\n";
          return false;
        }

      if (attribsSize == 0)
        continue;

      if (attribsSize <= sizeof(tag) + sizeof(attribsSize))
        {
          std::cerr << "Corrupted ELF RISCV file attributes subsection: Invalid size\n";
          return true;
        }

      attribsSize -= (sizeof(tag) + sizeof(attribsSize));

      auto attribsStart = iss.tellg();

      while (iss and (iss.tellg() - attribsStart < attribsSize))
        {
          // Next is a unsigned lengh-encoded binary 128 tag.
          Uint128 tag = 0;
          if (not extractUleb128(iss, tag))
            {
              std::cerr << "Empty/corrupted ELF RISCV file attributes subsection: Invalid tag\n";
              return false;
            }

          // If tag is even, value is another uleb128. If odd, value
          // is a null-terminated string.
          if ((tag & 1) == 0)
            {
              Uint128 value = 0;
              if (not extractUleb128(iss, value))
                {
                  std::cerr << "Empty/corrupted ELF RISCV file attributes subsection: Invalid tag value\n";
                  return false;
                }
            }
          else
            {
              std::string value;
              std::getline(iss, value, '\0');
              if (not iss)
                {
                  std::cerr << "Corrupted ELF RISCV file attributes subsection: Missing architeture tag string\n";
                  return false;
                }
              if (tag == 5)
                tags.push_back(value);
              return true;
            }
        }
    }

  return true;
}


void
Memory::collectElfSymbols(ELFIO::elfio& reader)
{
  for (const auto& sec : reader.sections)
    {
      if (sec->get_type() != SHT_SYMTAB)
	continue;

      const ELFIO::symbol_section_accessor symAccesor(reader, sec);
      ELFIO::Elf64_Addr address = 0;
      ELFIO::Elf_Xword size = 0;
      unsigned char bind, type, other;
      ELFIO::Elf_Half index = 0;

      // Finding symbol by name does not work. Walk all the symbols.
      ELFIO::Elf_Xword symCount = symAccesor.get_symbols_num();
      for (ELFIO::Elf_Xword symIx = 0; symIx < symCount; ++symIx)
	{
	  std::string name;
	  if (symAccesor.get_symbol(symIx, name, address, size, bind, type,
				    index, other))
	    {
	      if (name.empty())
		continue;

	      if (type == STT_NOTYPE or type == STT_FUNC or type == STT_OBJECT)
		symbols_[name] = ElfSymbol(address, size);
	    }
	}
    }
}


void
Memory::collectElfSections(ELFIO::elfio& reader)
{
  auto secCount = reader.sections.size();

  for (int secIx = 0; secIx < secCount; ++secIx)
    {
      auto* sec = reader.sections[secIx];
      sections_[sec->get_name()] = ElfSymbol(sec->get_address(), sec->get_size());
    }
}



bool
Memory::loadElfFile(const std::string& fileName, unsigned regWidth,
		    uint64_t& entryPoint, uint64_t& end)
{
  entryPoint = 0;
  end = 0;

  ELFIO::elfio reader;

  if (regWidth != 32 and regWidth != 64)
    {
      std::cerr << "Error: Memory::loadElfFile called with a unsupported "
		<< "register width: " << regWidth << '\n';
      return false;
    }

  if (not reader.load(fileName))
    {
      std::cerr << "Error: Failed to load ELF file " << fileName << '\n';
      return false;
    }

  bool is32 = reader.get_class() == ELFCLASS32;
  bool is64 = reader.get_class() == ELFCLASS64;
  if (not (is32 or is64))
    {
      std::cerr << "Error: ELF file is neither 32 nor 64-bit. Only 32/64-bit ELFs are currently supported\n";
      return false;
    }

  if (regWidth == 32 and not is32)
    {
      if (is64)
	std::cerr << "Error: Loading a 64-bit ELF file in 32-bit mode.\n";
      else
	std::cerr << "Error: Loading non-32-bit ELF file in 32-bit mode.\n";
      return false;
    }

  if (regWidth == 64 and not is64)
    {
      std::cerr << "Error: Loading non-64-bit ELF file in 64-bit mode.\n";
      return false;
    }

  if (reader.get_machine() != EM_RISCV)
    {
      std::cerr << "Warning: non-riscv ELF file\n";
    }

  // Copy loadable ELF segments into memory.
  uint64_t maxEnd = 0;  // Largest end address of a segment.
  unsigned errors = 0;

  for (int segIx = 0; segIx < reader.segments.size(); ++segIx)
    {
      uint64_t end = 0;
      if (loadElfSegment(reader, segIx, end))
        maxEnd = std::max(end, maxEnd);
      else
        errors++;
    }

  if (maxEnd == 0)
    {
      std::cerr << "No loadable segment in ELF file\n";
      errors++;
    }

  // In case writing ELF data modified last-written-data associated
  // with each hart.
  for (unsigned hartId = 0; hartId < reservations_.size(); ++hartId)
    clearLastWriteInfo(hartId);

  // Collect symbols.
  collectElfSymbols(reader);

  // Collect address/size of sections.
  collectElfSections(reader);

  // Get the program entry point.
  if (not errors)
    {
      entryPoint = reader.get_entry();
      end = maxEnd;
    }

  return errors == 0;
}


bool
Memory::findElfSymbol(const std::string& name, ElfSymbol& symbol) const
{
  auto symbol_it = symbols_.find(name);
  if (symbol_it == symbols_.end())
    return false;

  symbol = symbol_it->second;
  return true;
}


bool
Memory::findElfSection(const std::string& name, ElfSymbol& symbol) const
{
  auto section_it = sections_.find(name);
  if (section_it == sections_.end())
    return false;

  symbol = section_it->second;
  return true;
}


bool
Memory::findElfFunction(uint64_t addr, std::string& name, ElfSymbol& value) const
{
  for (const auto& kv : symbols_)
    {
      const auto& sym = kv.second;
      size_t start = sym.addr_, end = sym.addr_ + sym.size_;
      if (addr >= start and addr < end)
	{
	  name = kv.first;
	  value = sym;
	  return true;
	}
    }

  return false;
}


void
Memory::printElfSymbols(std::ostream& out) const
{
  out << std::hex;
  for (const auto& kv : symbols_)
    out << kv.first << ' ' << "0x" << kv.second.addr_ << '\n';
  out << std::dec;
}


bool
Memory::getElfFileAddressBounds(const std::string& fileName, uint64_t& minAddr,
				uint64_t& maxAddr)
{
  ELFIO::elfio reader;

  if (not reader.load(fileName))
    {
      std::cerr << "Failed to load ELF file " << fileName << '\n';
      return false;
    }

  // Get min max bounds of the segments.
  size_t minBound = ~ size_t(0);
  size_t maxBound = 0;
  unsigned validSegs = 0;
  for (const ELFIO::segment* seg : reader.segments)
    {
      if (seg->get_type() != PT_LOAD)
	continue;

      ELFIO::Elf64_Addr vaddr = seg->get_virtual_address();
      ELFIO::Elf_Xword size = seg->get_file_size(); // Size in file.

      minBound = std::min(minBound, size_t(vaddr));
      maxBound = std::max(maxBound, size_t(vaddr + size));
      validSegs++;
    }

  if (validSegs == 0)
    {
      std::cerr << "No loadable segment in ELF file\n";
      return false;
    }

  minAddr = minBound;
  maxAddr = maxBound;
  return true;
}


bool
Memory::checkElfFile(const std::string& path, bool& is32bit,
		     bool& is64bit, bool& isRiscv)
{
  ELFIO::elfio reader;

  if (not reader.load(path))
    return false;

  is32bit = reader.get_class() == ELFCLASS32;
  is64bit = reader.get_class() == ELFCLASS64;
  isRiscv = reader.get_machine() == EM_RISCV;

  return true;
}


bool
Memory::isSymbolInElfFile(const std::string& path, const std::string& target)
{
  ELFIO::elfio reader;

  if (not reader.load(path))
    return false;

  auto secCount = reader.sections.size();
  for (int secIx = 0; secIx < secCount; ++secIx)
    {
      auto* sec = reader.sections[secIx];
      if (sec->get_type() != SHT_SYMTAB)
	continue;

      const ELFIO::symbol_section_accessor symAccesor(reader, sec);
      ELFIO::Elf64_Addr address = 0;
      ELFIO::Elf_Xword size = 0;
      unsigned char bind, type, other;
      ELFIO::Elf_Half index = 0;

      // Finding symbol by name does not work. Walk all the symbols.
      ELFIO::Elf_Xword symCount = symAccesor.get_symbols_num();
      for (ELFIO::Elf_Xword symIx = 0; symIx < symCount; ++symIx)
	{
	  std::string name;
	  if (symAccesor.get_symbol(symIx, name, address, size, bind, type,
				    index, other))
	    {
	      if (name.empty())
		continue;
	      if (type == STT_NOTYPE or type == STT_FUNC or type == STT_OBJECT)
		if (name == target)
		  return true;
	    }
	}
    }
  return false;
}


bool
Memory::saveSnapshot(const std::string& filename,
                     const std::vector<std::pair<uint64_t,uint64_t>>& usedBlocks) const
{
  constexpr size_t maxChunk = size_t(1) << 28;

  // Open binary file for write (compressed) and check success.
  std::cerr << "saveSnapshot starts..\n";
  gzFile gzout = gzopen(filename.c_str(), "wb2");
  if (not gzout)
    {
      std::cerr << "Memory::saveSnapshot failed - cannot open " << filename
                << " for write\n";
      return false;
    }

  std::vector<uint32_t> temp;  // To collect sparse memory data.

  // write the simulated memory into the file and check success
  uint64_t prevAddr = 0;
  (void)prevAddr;
  bool success = true;
  for (const auto& blk: usedBlocks)
    {
#ifndef MEM_CALLBACKS
      if (blk.first >= size_)
	{
	  std::cerr << "Memory::saveSnapshot: Block address (0x" << std::hex << blk.first
		    << ") out of bounds (0x" << size_ << ")\n" << std::dec;
	  success = false;
	  break;
	}
#endif

      size_t remainingSize = blk.second;
#ifndef MEM_CALLBACKS
      if (remainingSize > size_ or size_ - remainingSize < blk.first)
	{
	  std::cerr << "Memory::saveSnapshot: Block at (0x" << std::hex << blk.first
		    << std::dec << ") extends beyond memory bound\n";
	  success = false;
	  break;
	}
#endif

      assert(prevAddr <= blk.first);
      prevAddr = blk.first + blk.second;

#ifdef MEM_CALLBACKS
      temp.resize(remainingSize);
      assert((blk.first & 3) == 0);
      assert((remainingSize & 3) == 0);
      size_t wordCount = remainingSize / 4;
      uint64_t addr = blk.first;
      for (size_t i = 0; i < wordCount; ++i, addr += 4)
	{
	  uint32_t x = 0;
	  peek(addr, x, false);
	  temp.at(i) = x;
	}
      uint8_t* buffer = reinterpret_cast<uint8_t*>(temp.data());
#else
      uint8_t* buffer = data_ + blk.first;
#endif
      std::cerr << "*";
      while (remainingSize)  // write in chunk due to limitation of gzwrite
        {
          std::cerr << "-";
          fflush(stdout);
          size_t currentChunk = std::min(remainingSize, maxChunk);
          int resp = gzwrite(gzout, buffer, currentChunk);
          success = resp > 0 and size_t(resp) == currentChunk;
          if (not success)
            break;
          remainingSize -= currentChunk;
          buffer += currentChunk;
        }
      if (not success)
        break;
    }

  if (not success)
    std::cerr << "Memory::saveSnapshot failed - write into " << filename
              << " failed with errno " << strerror(errno) << "\n";
  gzclose(gzout);
  std::cerr << "\nsaveSnapshot finished\n";
  return success;
}


bool
Memory::loadSnapshot(const std::string & filename,
                     const std::vector<std::pair<uint64_t,uint64_t>>& usedBlocks)
{
  constexpr size_t maxChunk = size_t(1) << 28;  // This must match saveSnapshot
  std::cerr << "loadSnapshot starts..\n";

  // open binary file for read (decompress) and check success
  gzFile gzin = gzopen(filename.c_str(), "rb");
  if (not gzin or gzeof(gzin))
    {
      std::cerr << "Memory::loadSnapshot failed - cannot open "
                << filename << " for read\n";
      return false;
    }

  std::vector<uint32_t> temp;

  // read (decompress) file into simulated memory and check success
  bool success = true;
  uint64_t prevAddr = 0;
  (void)prevAddr;
  size_t remainingSize = 0;
  for (const auto& blk: usedBlocks)
    {
#ifndef MEM_CALLBACKS
      if (blk.first >= size_)
	{
	  std::cerr << "Memory::loadSnapshot: Block address (0x" << std::hex << blk.first
		    << ") out of bounds (0x" << size_ << ")\n" << std::dec;
	  success = false;
	  break;
	}
#endif
      remainingSize = blk.second;
#ifndef MEM_CALLBACKS
      if (remainingSize > size_ or size_ - remainingSize < blk.first)
	{
	  std::cerr << "Memory::loadSnapshot: Block at (0x" << std::hex << blk.first
		    << ") extends beyond memory bound\n" << std::dec;
	  success = false;
	  break;
	}
#endif

      assert((blk.first & 3) == 0);
      assert((remainingSize & 3) == 0);
      assert(prevAddr <= blk.first);
      temp.resize(maxChunk);
      prevAddr = blk.first + blk.second;
      uint64_t addr = blk.first;

      std::cerr << "*";
      while (remainingSize) // read in chunk due to gzread limitation
        {
          std::cerr << "-";
          fflush(stdout);
          size_t currentChunk = std::min(remainingSize, maxChunk);
          int resp = gzread(gzin, temp.data(), currentChunk);
	  int words = resp / 4;
	  for (int i = 0; i < words; ++i, addr += 4)
	    {
	      // Avoid poking zero pages to maintain sparsity.
	      uint32_t prev = 0;
	      peek(addr, prev, false);
	      uint32_t curr = temp.at(i);
	      if (curr != prev)
		poke(addr, curr);
	    }
          if (resp == 0)
            {
              success = gzeof(gzin);
              break;
            }
          remainingSize -= resp;
        }
      if (not success)
        break;
    }

  if (not success)
    std::cerr << "Memory::loadSnapshot failed - read from " << filename
              << " failed: " << gzerror(gzin, nullptr) << "\n";
  else if (remainingSize > 0)
    std::cerr << "Memory::loadSnapshot: Warning: Snapshot data size smaller than memory size\n";

  gzclose(gzin);
  std::cerr << "\nloadSnapshot finished\n";
  return success;
}


bool
Memory::saveAddressTrace(std::string_view tag,
			 const LineMap& lineMap,
			 const std::string& path)
{
  std::ofstream out(path, std::ios::trunc);

  if (not out)
    {
      std::cerr << "Memory::saveAddressTrace failed - cannot open " << path
		<< " for write\n";
      return false;
    }

  std::cerr << "Trace map size for " << tag << ": " << lineMap.size() << '\n';

  std::vector<uint64_t> addrVec;
  addrVec.reserve(lineMap.size());

  for (const auto& kv : lineMap)
    addrVec.push_back(kv.first);

  std::sort(addrVec.begin(), addrVec.end(),
	    [&lineMap](uint64_t a, uint64_t b) {
	      return lineMap.at(a).order < lineMap.at(b).order;
	    });

  out << std::hex;

  for (auto vaddr : addrVec)
    {
      uint64_t paddr = lineMap.at(vaddr).paddr;
      out << vaddr << ':' << paddr;
      out << '\n';
    }

  out << std::dec;

  return true;
}


bool
Memory::saveDataAddressTrace(const std::string& path) const
{
  if (not dataLineTrace_)
    return true;
  return saveAddressTrace("data", dataLineMap_, path);
}


bool
Memory::saveInstructionAddressTrace(const std::string& path) const
{
  if (not instrLineTrace_)
    return true;
  return saveAddressTrace("instruction", instrLineMap_, path);
}


bool
Memory::initializeByte(uint64_t addr, uint8_t value)
{
  if (addr >= size_)
    return false;

  if (pmaMgr_.isMemMappedReg(addr))
    {
      if (not pmaMgr_.pokeRegisterByte(addr, value))
        return false;
    }

  // We initialize both the memory-mapped-register and the external
  // memory to match/simplify the test-bench.
  if (writeCallback_)
    writeCallback_(addr, 1, value);
  else
    data_[addr] = value;
  return true;
}


void
Memory::resetMemoryMappedRegisters()
{
  pmaMgr_.resetMemMapped();
}
