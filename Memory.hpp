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

#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>
#include <span>
#include <unordered_map>
#include <functional>
#include <shared_mutex>
#include <mutex>
#include <type_traits>
#include <cassert>
#include "trapEnums.hpp"
#include "PmaManager.hpp"
#include "IoDevice.hpp"
#include "util.hpp"


namespace ELFIO
{
  class elfio;
}


namespace WdRiscv
{

  template <typename URV>
  class Hart;

  /// Location and size of an ELF file symbol.
  struct ElfSymbol
  {
    ElfSymbol(uint64_t addr = 0, uint64_t size = 0)
      : addr_(addr), size_(size)
    { }

    uint64_t addr_ = 0;
    uint64_t size_ = 0;
  };


  /// Model physical memory of system.
  class Memory
  {
  public:

    friend class Hart<uint32_t>;
    friend class Hart<uint64_t>;

    /// Constructor: define a memory of the given size initialized to
    /// zero. Given memory size (byte count) must be a multiple of 4
    /// otherwise, it is truncated to a multiple of 4.
    Memory(uint64_t size, uint64_t pageSize = UINT64_C(4)*1024);

    /// Destructor.
    ~Memory();

    /// Define number of hardware threads for LR/SC. FIX: put this in
    /// constructor.
    void setHartCount(unsigned count)
    { reservations_.resize(count); lastWriteData_.resize(count); }

    /// Return memory size in bytes.
    uint64_t size() const
    { return size_; }

    /// Read an unsigned integer value of type T from memory at the
    /// given address into value assuming a little-endian memory
    /// organization. Return true on success. Return false if any of
    /// the requested bytes is out of memory bounds, fall in unmapped
    /// memory, are in a region marked non-read, or if is to a
    /// memory-mapped-register and the access size is different than
    /// 4.
    template <typename T>
    bool read(uint64_t address, T& value) const
    {
      if (readIo(address, value))
	return true;
#ifdef FAST_SLOPPY
      if (address + sizeof(T) > size_)
        return false;
#else
      Pma pma1 = pmaMgr_.accessPma(address);
      if (not pma1.isRead())
	return false;

      if (address & (sizeof(T) - 1))  // If address is misaligned
	{
          Pma pma2 = pmaMgr_.accessPma(address + sizeof(T) - 1);
          if (not pma2.isRead())
            return false;
        }

      // Memory mapped region accessible only with word-size read.
      if (pma1.hasMemMappedReg() and pmaMgr_.isMemMappedReg(address))
	return readRegister(address, value);

#endif

#ifdef MEM_CALLBACKS
      uint64_t val = 0;
      if (not readCallback_(address, sizeof(T), val))
        return false;
      value = val;
#else
      value = *(reinterpret_cast<const T*>(data_ + address));
#endif

      return true;
    }

    /// Read an unsigned integer of type T from memory for instruction
    /// fetch. Return true on success and false if address is not
    /// executable.
    template <typename T>
    bool readInst(uint64_t address, T& value) const
    {
      Pma pma = pmaMgr_.accessPma(address);
      if (not pma.isExec())
	return false;

      if (address & (sizeof(T) -1))
	{
	  // Misaligned address: Check next address.
	  Pma pma2 = pmaMgr_.getPma(address + sizeof(T) - 1);
	  if (not pma2.isExec())
	    return false;  // No exec.
	}

#ifdef MEM_CALLBACKS
      uint64_t val = 0;
      if (not readCallback_(address, sizeof(T), val))
	return false;
      value = val;
#else
      value = *(reinterpret_cast<const T*>(data_ + address));
#endif

      return true;
    }

    /// Return true if read will be successful if tried.
    bool checkRead(uint64_t address, unsigned readSize)
    {
      Pma pma1 = pmaMgr_.getPma(address);
      if (not pma1.isRead())
	return false;

      if (address & (readSize - 1))  // If address is misaligned
	{
          Pma pma2 = pmaMgr_.getPma(address + readSize - 1);
          if (not pma2.isRead())
            return false;
	}

      // Memory mapped region accessible only with word-size read.
      if (pma1.hasMemMappedReg() and pmaMgr_.isMemMappedReg(address))
	return pmaMgr_.checkRegisterRead(address, readSize);

      return true;
    }

    /// Return true if write will be successful if tried. Do not
    /// write.
    bool checkWrite(uint64_t address, unsigned writeSize)
    {
      Pma pma1 = pmaMgr_.getPma(address);
      if (not pma1.isWrite())
	return false;

      if (address & (writeSize - 1))  // If address is misaligned
	{
          Pma pma2 = pmaMgr_.getPma(address + writeSize - 1);
          if (not pma2.isWrite())
            return false;
	}

      // Memory mapped region accessible only with word-size write.
      if (pma1.hasMemMappedReg() and pmaMgr_.isMemMappedReg(address))
	return pmaMgr_.checkRegisterWrite(address, writeSize);

      return true;
    }

    /// Write given unsigned integer value of type T into memory
    /// starting at the given address and assuming little-endian
    /// origanization. Return true on success. Return false if any of
    /// the target memory bytes are out of bounds or fall in
    /// inaccessible regions or if the write crosses memory region of
    /// different attributes.
    template <typename T>
    bool write([[maybe_unused]] unsigned sysHartIx, uint64_t address, T value)
    {
      auto& lwd = lastWriteData_.at(sysHartIx);
      lwd.size_ = 0;
      lwd.addr_ = address;
      lwd.value_ = value;

      if (writeIo(address, value))
	{
	  lwd.size_ = sizeof(value);
	  return true;
	}

#ifdef FAST_SLOPPY
      if (address + sizeof(T) > size_)
        return false;
      *(reinterpret_cast<T*>(data_ + address)) = value;
#else

      Pma pma1 = pmaMgr_.accessPma(address);
      if (not pma1.isWrite())
	return false;

      if (address & (sizeof(T) - 1))  // If address is misaligned
	{
          Pma pma2 = pmaMgr_.accessPma(address + sizeof(T) - 1);
          if (pma1 != pma2)
	    return false;
	}

      // Memory mapped region accessible only with word-size write.
      if (pma1.hasMemMappedReg() and pmaMgr_.isMemMappedReg(address))
	return writeRegister(sysHartIx, address, value);

  #ifdef MEM_CALLBACKS
      uint64_t val = value;
      if (not writeCallback_(address, sizeof(T), val))
	return false;
  #else
      *(reinterpret_cast<T*>(data_ + address)) = value;
  #endif

#endif

      lwd.size_ = sizeof(T);
      return true;
    }

    /// Perfrom read from IO devices. Return true if we hit in any IO
    /// device and false otherwise.
    template<typename T>
    bool readIo(uint64_t addr, T& val) const
    {
      return std::ranges::any_of(ioDevs_,
                                 [addr, &val](const auto& dev) {
                                   bool found = dev->isAddressInRange(addr);
                                   if (found)
                                     val = dev->read(addr);
                                   return found;
                                 });
    }

    /// Perfrom write from IO devices. Return true if we hit in any IO
    /// device and false otherwise.
    template<typename T>
    bool writeIo(uint64_t addr, T val)
    {
      return std::ranges::any_of(ioDevs_,
                                 [addr, val](auto& dev) {
                                   bool found = dev->isAddressInRange(addr);
                                   if (found)
                                     dev->write(addr, val);
                                   return found;
                                 });
    }

    /// Write half-word (2 bytes) to given address. Return true on
    /// success. Return false if address is out of bounds or is not
    /// writable.
    bool writeHalfWord(unsigned sysHartIx, uint64_t address, uint16_t value)
    { return write(sysHartIx, address, value); }

    /// Read word (4 bytes) from given address into value. Return true
    /// on success.  Return false if address is out of bounds or is
    /// not writable.
    bool writeWord(unsigned sysHartIx, uint64_t address, uint32_t value)
    { return write(sysHartIx, address, value); }

    /// Read a double-word (8 bytes) from given address into
    /// value. Return true on success. Return false if address is out
    /// of bounds.
    bool writeDoubleWord(unsigned sysHartIx, uint64_t address, uint64_t value)
    { return write(sysHartIx, address, value); }

    /// Similar to read but ignore physical-memory-attributes if
    /// usePma is false.
    template <typename T>
    bool peek(uint64_t address, T& value, bool usePma) const
    {
      if (address + sizeof(T) > size_)
        return false;

#ifdef FAST_SLOPPY
      value = *(reinterpret_cast<const T*>(data_ + address));
      return true;
#endif

      // Memory mapped region accessible only with word-size read.
      Pma pma1 = pmaMgr_.getPma(address);
      if (pma1.hasMemMappedReg() and pmaMgr_.isMemMappedReg(address))
	return readRegister(address, value);

      if (usePma)
        {
          if (not pma1.isRead() and not pma1.isExec())
            return false;

          if (address & (sizeof(T) - 1))  // If address is misaligned
            {
              Pma pma2 = pmaMgr_.getPma(address + sizeof(T) - 1);
              if (not pma2.isRead() and not pma2.isExec())
                return false;
            }
        }

#ifdef MEM_CALLBACKS
      uint64_t val = 0;
      if (not readCallback_(address, sizeof(T), val))
        return false;
      value = val;
      return true;
#endif

      value = *(reinterpret_cast<const T*>(data_ + address));
      return true;
    }

    /// Load the given hex file and set memory locations accordingly.
    /// Return true on success. Return false if file does not exists,
    /// cannot be opened or contains malformed data.
    /// File format: A line either contains @address where address
    /// is a hexadecimal memory address or one or more space separated
    /// tokens each consisting of two hexadecimal digits.
    bool loadHexFile(const std::string& file);

    /// Load the binary file and set memory locations accordingly.
    /// Return true on success. Return false if file does not exist,
    /// or cannot be opened.
    bool loadBinaryFile(const std::string& file, uint64_t addr);

    /// Load the lz4 compressed binary file and set memory locations
    /// accordingly.
    /// Return true on success. Return false if file does not exist,
    /// or cannot be opened.
    bool loadLz4File(const std::string& file, uint64_t addr);

    /// Load the given ELF file and set memory locations accordingly.
    /// Return true on success. Return false if file does not exists,
    /// cannot be opened or contains malformed data, or if it contains
    /// data incompatible with the given register width (32 or 64). If
    /// successful, set entryPoint to the entry point of the loaded
    /// file and end to the address past that of the loaded byte with
    /// the largest address. Extract symbol names and corresponding
    /// addresses and sizes into the memory symbols map.
    bool loadElfFile(const std::string& file, unsigned registerWidth,
		     uint64_t& entryPoint, uint64_t& end);

    /// Locate an ELF symbol by name (symbols are collected for every
    /// loaded ELF file) returning true if symbol is found and false
    /// otherwise. If found set result to the address/size of the ELF
    /// symbol.
    bool findElfSymbol(const std::string& name, ElfSymbol& result) const;

    /// Locate an ELF section by name returning true if section is
    /// found and false otherwise. If found set result to the
    /// address/size of the ELF section.
    bool findElfSection(const std::string& name, ElfSymbol& result) const;

    /// Locate the ELF function cotaining the give address returning true
    /// on success and false on failure.  If successful set name to the
    /// corresponding function name and symbol to the corresponding symbol
    /// value.
    bool findElfFunction(uint64_t addr, std::string& name, ElfSymbol& value) const;

    /// Print the ELF symbols on the given stream. Output format:
    /// <name> <value>
    void printElfSymbols(std::ostream& out) const;

    /// Enable/disable errors on unmapped memory when loading
    /// ELF/hex/binary files.
    void checkUnmappedElf(bool flag)
    { checkUnmappedElf_ = flag; }

    /// Return the min and max addresses corresponding to the segments
    /// in the given ELF file. Return true on success and false if
    /// the ELF file does not exist or cannot be read (in which
    /// case min and max address are left unmodified).
    static bool getElfFileAddressBounds(const std::string& file,
					uint64_t& minAddr, uint64_t& maxAddr);

    /// Collect RISCV architecture attributes from given ELF file.
    /// The toolchain encodes the architecture string used at
    /// compilation (e.g. --march=imac") into an ELF file tag. This
    /// method recovers sutch tag(s) and appends them to the given
    /// tags vector. Return true on success and false on failure. If
    /// no such tag is present, that is considered a success.
    static bool collectElfRiscvTags(const std::string& file,
                                    std::vector<std::string>& tags);

    /// Return true if given path corresponds to an ELF file and set
    /// the given flags according to the contents of the file.  Return
    /// false leaving the flags unmodified if file does not exist,
    /// cannot be read, or is not an ELF file.
    static bool checkElfFile(const std::string& path, bool& is32bit,
			     bool& is64bit, bool& isRiscv);

    /// Return true if given symbol is present in the given ELF file.
    static bool isSymbolInElfFile(const std::string& path,
				  const std::string& target);

    /// This is for performance modeling. Enable a highest level cache
    /// with given size, line size, and set associativity.  Any
    /// previously enabled cache is deleted.  Return true on success
    /// and false if the sizes are not powers of 2 or if any of them
    /// is zero, or if they are too large (more than 64 MB for cache
    /// size, more than 1024 for line size, and more than 64 for
    /// associativity). This has no impact on functionality.
    bool configureCache(uint64_t size, unsigned lineSize,
                        unsigned setAssociativity);

    /// Delete currently configured cache.
    void deleteCache();

    /// Define read memory callback. This (along with defineWriteMemoryCallback) allows
    /// the caller to bypass the memory model with their own.
    void defineReadMemoryCallback(
         std::function<bool(uint64_t, unsigned, uint64_t&)> callback )
    { readCallback_ = std::move(callback); }

    /// Define write memory callback. This (along with defineReadMemoryCallback) allows
    /// the caller to bypass the memory model with their own.
    void defineWriteMemoryCallback(std::function<bool(uint64_t, unsigned, uint64_t)> callback)
    { writeCallback_ = std::move(callback); }

    /// Define page initialization callback. This is used to speed-up memory insitialization
    /// for the sparse-memory mode..
    void defineInitPageCallback(std::function<bool(uint64_t, const std::span<uint8_t>)> callback)
    { initPageCallback_ = std::move(callback); }

    /// Enable tracing of memory data lines referenced by current
    /// run. A memory data line is typically 64-bytes long and corresponds to
    /// a cachable line.
    void enableDataLineTrace(const std::string& path)
    { dataLineTrace_ = true; dataLineFile_ = path; }

    /// Enable tracing of memory instruction fetch lines referenced by
    /// current run. A memory line is typically 64-bytes long and
    /// corresponds to a cachable line.
    void enableInstructionLineTrace(const std::string& path)
    { instrLineTrace_ = true;  instrLineFile_ = path; }

    void registerIoDevice(std::shared_ptr<IoDevice> dev)
    { assert(dev); ioDevs_.push_back(std::move(dev)); }

    /// Take a snapshot of the entire simulated memory into binary
    /// file. Return true on success or false on failure
    bool saveSnapshot(const std::string& filename,
                      const std::vector<std::pair<uint64_t,uint64_t>>& used_blocks) const;

    /// Load the simulated memory from snapshot binary file. Return
    /// true on success or false on failure
    bool loadSnapshot(const std::string& filename,
                      const std::vector<std::pair<uint64_t,uint64_t>>& used_blocks);

    /// If address tracing enabled, then write the accumulated data
    /// addresses into the given file.
    bool saveDataAddressTrace(const std::string& path) const;

    /// If instruction tracing enabled, then write the accumulated
    /// addresses into the given file.
    bool saveInstructionAddressTrace(const std::string& path) const;

    /// If address tracing enabled, then load the accumulated data
    /// addresses from the given file.
    bool loadDataAddressTrace(const std::string& path);

    /// If instruction tracing enabled, then load the accumulated
    /// instruction addresses into the given file.
    bool loadInstructionAddressTrace(const std::string& path);

    /// Return the line number corresponding to the given address.
    uint64_t getLineNumber(uint64_t addr) const
    { return addr >> lineShift_; }

    /// Return the memory/cache line size.
    unsigned lineSize() const
    { return 1 << lineShift_; }

    /// Return the page size.
    uint64_t pageSize() const
    { return pageSize_; }

    /// Return the number of the page containing the given address.
    uint64_t getPageIx(uint64_t addr) const
    { return addr >> pageShift_; }

    /// Return start address of page containing given address.
    uint64_t getPageStartAddr(uint64_t addr) const
    { return (addr >> pageShift_) << pageShift_; }

    /// Same as write but effects not recorded in last-write info and
    /// physical memory attributes are ignored if usePma is false.
    template <typename T>
    bool poke(uint64_t address, T value, bool usePma = true)
    {
      if (address + sizeof(T) > size_)
        return false;

      Pma pma1 = pmaMgr_.getPma(address);
      if (pma1.hasMemMappedReg() and pmaMgr_.isMemMappedReg(address))
        {
	  for (unsigned i = 0; i < sizeof(T); ++i)
	    {
	      uint8_t byte = (value >> (i*8)) & 0xff;
	      if (not pmaMgr_.pokeRegisterByte(address + i, byte))
		return false;
	    }
	  return true;
        }

      if (usePma)
        {
          if (not pma1.isMapped())
            return false;

          if (address & (sizeof(T) - 1))  // If address is misaligned
            {
              Pma pma2 = pmaMgr_.getPma(address + sizeof(T) - 1);
              if (not pma2.isMapped())
                return false;
            }
        }

#ifdef MEM_CALLBACKS
      uint64_t val = value;
      return writeCallback_(address, sizeof(T), val);
#endif

      *(reinterpret_cast<T*>(data_ + address)) = value;
      return true;
    }

    /// Return true if given address has reserve-eventual attribute.
    bool hasReserveAttribute(uint64_t addr) const
    { return pmaMgr_.accessPma(addr).isRsrv(); }

    /// Return true if given address is page aligned.
    bool isPageAligned(uint64_t addr) const
    { return ((addr >> pageShift_) << pageShift_) == addr; }

  protected:

    /// Write byte to given address without write-access check. Return
    /// true on success. Return false if address is not mapped. This
    /// is used to initialize memory. If address is in
    /// memory-mapped-register region, then both mem-mapped-register
    /// and external memory are written.
    bool initializeByte(uint64_t address, uint8_t value);

    /// Write given buffer to the page at the given address. Buffer size
    /// must be >= pageSize_.
    bool initializePage(uint64_t addr, const std::span<uint8_t> buffer);

    /// Clear the information associated with last write.
    void clearLastWriteInfo(unsigned sysHartIx)
    {
      auto& lwd = lastWriteData_.at(sysHartIx);
      lwd.size_ = 0;
    }

    /// Reset (to zero) all memory mapped registers.
    void resetMemoryMappedRegisters();

    /// Read a memory mapped register word.
    bool readRegister(uint64_t addr, auto& value) const
    { return pmaMgr_.readRegister(addr, value); }

    /// Return memory mapped mask associated with the word containing
    /// the given address. Return all 1 if given address is not a
    /// memory mapped register.
    uint32_t getMemoryMappedMask(uint64_t addr) const
    { return pmaMgr_.getMemMappedMask(addr); }

    /// Perform masking for a write to a memory mapped register.
    /// Return masked value.
    uint32_t doRegisterMasking(uint64_t addr, uint32_t value) const
    {
      uint32_t mask = getMemoryMappedMask(addr);
      value = value & mask;
      return value;
    }

    /// Write a memory mapped register.
    bool writeRegister(unsigned sysHartIx, uint64_t addr, auto value)
    {
      value = doRegisterMasking(addr, value);
      auto& lwd = lastWriteData_.at(sysHartIx);

      if (not pmaMgr_.writeRegister(addr, value))
	{
	  lwd.size_ = 0;
	  return false;
	}

      lwd.size_ = sizeof(value);
      lwd.addr_ = addr;
      lwd.value_ = value;
      return true;
    }

    /// Track LR instructin resrvations.
    struct Reservation
    {
      uint64_t addr_ = 0;
      unsigned size_ = 0;
      bool valid_ = false;
      CancelLrCause cause_ = CancelLrCause::NONE;
    };

    /// Invalidate LR reservations matching address of poked/written
    /// bytes and belonging to harts other than the given hart-id. The
    /// memory tracks one reservation per hart indexed by local hart
    /// ids.
    void invalidateOtherHartLr(unsigned sysHartIx, uint64_t addr,
                               unsigned storeSize)
    {
      for (uint64_t i = 0; i < reservations_.size(); ++i)
        {
          if (i == sysHartIx) continue;
          auto& res = reservations_[i];
          if ((addr >= res.addr_ and (addr - res.addr_) < res.size_) or
              (addr < res.addr_ and (res.addr_ - addr) < storeSize))
	    {
	      res.valid_ = false;
	      res.cause_ = CancelLrCause::STORE;
	    }
        }
    }


    /// Invalidate LR reservation corresponding to the given hart.
    void invalidateLr(unsigned sysHartIx, CancelLrCause cause)
    {
      auto& res = reservations_.at(sysHartIx);
      res.valid_ = false;
      res.cause_ = cause;
    }

    /// Make a LR reservation for the given hart.
    void makeLr(unsigned sysHartIx, uint64_t addr, unsigned size)
    {
      auto& res = reservations_.at(sysHartIx);
      res.addr_ = addr;
      res.size_ = size;
      res.valid_ = true;
      res.cause_ = CancelLrCause::NONE;
    }

    // returns true if the given hart has a valid LR reservation
    bool hasLr(unsigned sysHartIx) const
    {
      const auto& res = reservations_.at(sysHartIx);
      return res.valid_;
    }

    /// Return true if given hart has a valid LR reservation and if it
    /// contains the range defined by the given address and size.
    bool hasLr(unsigned sysHartIx, uint64_t addr, unsigned size) const
    {
      const auto& res = reservations_.at(sysHartIx);
      return (res.valid_ and res.addr_ <= addr and
	      addr + size <= res.addr_ + res.size_);
    }

    /// Return the cause for the reservation cancleation in the given
    /// hart. Return NONE if hart index is out of bounds. Return NONE
    /// if given hart has a valid reservation.
    CancelLrCause cancelLrCause(unsigned sysHartIx) const
    {
      if (sysHartIx >= reservations_.size())
	return CancelLrCause::NONE;
      const auto& res = reservations_.at(sysHartIx);
      return res.valid_ ? CancelLrCause::NONE : res.cause_;
    }

    /// Load contents of given ELF segment into memory.
    /// This is a helper to loadElfFile.
    bool loadElfSegment(ELFIO::elfio& reader, int segment, uint64_t& end);

    /// Helper to loadElfFile: Collet ELF symbols.
    void collectElfSymbols(ELFIO::elfio& reader);

    /// Helper to loadElfFile: Collet ELF sections.
    void collectElfSections(ELFIO::elfio& reader);

    /// Structure to track data lines referenced by a run.
    struct LineEntry
    {
      uint64_t paddr = 0;   // Physical line number.
      uint64_t order = 0;   // Reference order.
    };
    typedef std::unordered_map<uint64_t, LineEntry> LineMap;

    static bool saveAddressTrace(std::string_view tag,
                                 const LineMap& lineMap,
                                 const std::string& path);

    static bool loadAddressTrace(LineMap& lineMap,
                                 uint64_t& refCount,
                                 const std::string& path);

    /// Add line of given address to the data line address trace.
    void traceDataLine(uint64_t vaddr, uint64_t paddr)
    { dataLineMap_[vaddr >> lineShift_] = LineEntry{paddr >> lineShift_, memRefCount_++}; }

    /// Add line of given address to the instruction line address trace.
    void traceInstructionLine(uint64_t vaddr, uint64_t paddr)
    { instrLineMap_[vaddr >> lineShift_] = LineEntry{paddr >> lineShift_, memRefCount_++}; }

  private:

    /// Information about last write operation by a hart.
    struct LastWriteData
    {
      unsigned size_ = 0;
      uint64_t addr_ = 0;
      uint64_t value_ = 0;
    };

    uint64_t size_;        // Size of memory in bytes.
    uint8_t* data_;      // Pointer to memory data.

    uint64_t pageSize_    = UINT64_C(4)*1024;   // Must be a power of 2.
    unsigned pageShift_   = 12;                 // Shift address by this to get page no.
    unsigned regionShift_ = 28;                 // Shift address by this to get region no.
    unsigned regionMask_  = 0xf;                // This should depend on mem size.

    std::shared_mutex amoMutex_;
    std::mutex lrMutex_;

    bool checkUnmappedElf_ = true;

    std::unordered_map<std::string, ElfSymbol, util::string_hash, std::equal_to<>> symbols_;
    std::unordered_map<std::string, ElfSymbol, util::string_hash, std::equal_to<>> sections_;

    std::vector<Reservation> reservations_;
    std::vector<LastWriteData> lastWriteData_;

    PmaManager pmaMgr_;

    // Support for line address traces
    bool dataLineTrace_ = false;
    bool instrLineTrace_ = false;
    std::string dataLineFile_;
    std::string instrLineFile_;
    unsigned lineShift_ = 6;   // log2 of line size.
    uint64_t memRefCount_ = 0;
    LineMap dataLineMap_;   // Map virt data line addr to phys line and order.
    LineMap instrLineMap_;  // Map virt instr line line addr to phys line and order.

    std::vector<std::shared_ptr<IoDevice>> ioDevs_;

    /// Callback for read: bool func(uint64_t addr, unsigned size, uint64_t& val);
    std::function<bool(uint64_t, unsigned, uint64_t&)> readCallback_ = nullptr;

    /// Callback for write: bool func(uint64_t addr, unsigned size, uint64_t val);
    std::function<bool(uint64_t, unsigned, uint64_t)> writeCallback_ = nullptr;

    /// Callback to initialize a page of memory.
    std::function<bool(uint64_t, const std::span<uint8_t>)> initPageCallback_ = nullptr;

    std::pair<std::unique_ptr<uint8_t[]>, size_t> loadFile(const std::string& filename);
  };
}
