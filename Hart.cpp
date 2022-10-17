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
#include <sstream>
#include <climits>
#include <map>
#include <mutex>
#include <array>
#include <atomic>
#include <cstring>
#include <ctime>
#include <boost/format.hpp>
#include <sys/time.h>
#include <poll.h>
#include <sys/ioctl.h>

#include <boost/multiprecision/cpp_int.hpp>

// On pure 32-bit machines, use boost for 128-bit integer type.
#if __x86_64__
  typedef __int128_t  Int128;
  typedef __uint128_t Uint128;
#else
  typedef boost::multiprecision::int128_t  Int128;
  typedef boost::multiprecision::uint128_t Uint128;
#endif

#include <fcntl.h>
#include <sys/time.h>
#include <sys/stat.h>

#include <cassert>
#include <csignal>

#define __STDC_FORMAT_MACROS
#include <cinttypes>
#include <sys/socket.h>
#include <netinet/in.h>

#include "instforms.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"
#include "Mcm.hpp"


#ifndef SO_REUSEPORT
#define SO_REUSEPORT SO_REUSEADDR
#endif


using namespace WdRiscv;


template <typename TYPE>
static
bool
parseNumber(const std::string& numberStr, TYPE& number)
{
  bool good = not numberStr.empty();

  if (good)
    {
      char* end = nullptr;
      if (sizeof(TYPE) == 4)
	number = strtoul(numberStr.c_str(), &end, 0);
      else if (sizeof(TYPE) == 8)
	number = strtoull(numberStr.c_str(), &end, 0);
      else
	{
	  std::cerr << "parseNumber: Only 32/64-bit RISCV harts supported\n";
	  return false;
	}
      if (end and *end)
	good = false;  // Part of the string are non parseable.
    }
  return good;
}


template <typename URV>
Hart<URV>::Hart(unsigned hartIx, URV hartId, Memory& memory)
  : hartIx_(hartIx), memory_(memory), intRegs_(32),
    fpRegs_(32), vecRegs_(), syscall_(*this),
    pmpManager_(memory.size(), 1024*1024),
    virtMem_(hartIx, memory, memory.pageSize(), pmpManager_, 16),
    isa_()
{
  decodeCacheSize_ = 128*1024;  // Must be a power of 2.
  decodeCacheMask_ = decodeCacheSize_ - 1;
  decodeCache_.resize(decodeCacheSize_);

  interruptStat_.resize(size_t(InterruptCause::MAX_CAUSE) + 1);
  exceptionStat_.resize(size_t(ExceptionCause::MAX_CAUSE) + 1);

  // Tie the retired instruction and cycle counter CSRs to variables
  // held in the hart.
  if constexpr (sizeof(URV) == 4)
    {
      URV* low = reinterpret_cast<URV*> (&retiredInsts_);
      URV* high = low + 1;

      auto& mirLow = csRegs_.regs_.at(size_t(CsrNumber::MINSTRET));
      auto& irLow = csRegs_.regs_.at(size_t(CsrNumber::INSTRET));
      mirLow.tie(low);
      irLow.tie(low);

      auto& mirHigh = csRegs_.regs_.at(size_t(CsrNumber::MINSTRETH));
      auto& irHigh = csRegs_.regs_.at(size_t(CsrNumber::INSTRETH));
      mirHigh.tie(high);
      irHigh.tie(high);

      low = reinterpret_cast<URV*> (&cycleCount_);
      high = low + 1;

      auto& mcycleLow = csRegs_.regs_.at(size_t(CsrNumber::MCYCLE));
      auto& cycleLow = csRegs_.regs_.at(size_t(CsrNumber::CYCLE));
      mcycleLow.tie(low);
      cycleLow.tie(low);

      auto& mcycleHigh = csRegs_.regs_.at(size_t(CsrNumber::MCYCLEH));
      auto& cycleHigh = csRegs_.regs_.at(size_t(CsrNumber::CYCLEH));
      mcycleHigh.tie(high);
      cycleHigh.tie(high);

      // TIME is a read-only shadow of MCYCLE.
      csRegs_.regs_.at(size_t(CsrNumber::TIME)).tie(low);
      csRegs_.regs_.at(size_t(CsrNumber::TIMEH)).tie(high);
    }
  else
    {
      csRegs_.regs_.at(size_t(CsrNumber::MINSTRET)).tie(&retiredInsts_);
      csRegs_.regs_.at(size_t(CsrNumber::MCYCLE)).tie(&cycleCount_);

      // INSTRET and CYCLE are read-only shadows of MINSTRET and MCYCLE.
      csRegs_.regs_.at(size_t(CsrNumber::INSTRET)).tie(&retiredInsts_);
      csRegs_.regs_.at(size_t(CsrNumber::CYCLE)).tie(&cycleCount_);

      // TIME is a read-only shadow of MCYCLE.
      csRegs_.regs_.at(size_t(CsrNumber::TIME)).tie(&cycleCount_);
    }

  // Tie the FCSR register to variable held in the hart.
  csRegs_.regs_.at(size_t(CsrNumber::FCSR)).tie(&fcsrValue_);

  // Configure MHARTID CSR.
  bool implemented = true, debug = false, shared = false;
  URV mask = 0, pokeMask = 0;

  csRegs_.configCsr(CsrNumber::MHARTID, implemented, hartId, mask, pokeMask,
                    debug, shared);
}


template <typename URV>
Hart<URV>::~Hart()
{
}


template <typename URV>
void
Hart<URV>::getImplementedCsrs(std::vector<CsrNumber>& vec) const
{
  vec.clear();

  for (unsigned i = 0; i <= unsigned(CsrNumber::MAX_CSR_); ++i)
    {
      CsrNumber csrn = CsrNumber(i);
      if (csRegs_.isImplemented(csrn))
	vec.push_back(csrn);
    }
}


template <typename URV>
bool
Hart<URV>::configureCache(uint64_t size, unsigned lineSize,
                          unsigned setSize)
{
  return memory_.configureCache(size, lineSize, setSize);
}


template <typename URV>
void
Hart<URV>::deleteCache()
{
  memory_.deleteCache();
}


template <typename URV>
void
Hart<URV>::getCacheLineAddresses(std::vector<uint64_t>& addresses)
{
  memory_.getCacheLineAddresses(addresses);
}


template <typename URV>
unsigned
Hart<URV>::countImplementedPmpRegisters() const
{
  unsigned count = 0;

  unsigned num = unsigned(CsrNumber::PMPADDR0);
  for (unsigned ix = 0; ix < 64; ++ix, ++num)
    if (csRegs_.isImplemented(CsrNumber(num)))
      count++;

  if (count and count < 64)
    std::cerr << "Warning: Some but not all PMPADDR CSRs are implemented\n";

  unsigned cfgCount = 0;
  if (mxlen_ == 32)
    {
      num = unsigned(CsrNumber::PMPCFG0);
      for (unsigned ix = 0; ix < 16; ++ix, ++num)
        if (csRegs_.isImplemented(CsrNumber(num)))
          cfgCount++;
      if (count and cfgCount != 16)
        std::cerr << "Warning: Physical memory protection enabled but only "
                  << cfgCount << "/16" << " PMPCFG CSRs implemented\n";
    }
  else
    {
      num = unsigned(CsrNumber::PMPCFG0);
      for (unsigned ix = 0; ix < 16; ++ix, ++num)
        if (csRegs_.isImplemented(CsrNumber(num)))
          cfgCount++;
      if (count and cfgCount != 8)  // Only even numbered CFG CSRs implemented.
        std::cerr << "Warning: Physical memory protection enabled but only "
		  << cfgCount << "/8" << " PMPCFG CSRs implemented.\n";
    }

  return count;
}


template <typename URV>
void
Hart<URV>::processExtensions(bool verbose)
{
  URV value = 0;
  peekCsr(CsrNumber::MISA, value);

  rva_ = (value & 1) and isa_.isEnabled(RvExtension::A);   // Atomic

  rvc_ = (value & (URV(1) << ('c' - 'a')));  // Compress option.
  rvc_ = rvc_ and isa_.isEnabled(RvExtension::C);

  bool flag = value & (URV(1) << ('f' - 'a'));  // Single precision FP
  flag = flag and isa_.isEnabled(RvExtension::F);
  enableRvf(flag);

  // D requires F and is enabled only if F is enabled.
  flag = value & (URV(1) << ('d' - 'a'));  // Double precision FP
  if (flag and not rvf_)
    {
      flag = false;
      if (verbose)
	std::cerr << "Bit 3 (d) is set in the MISA register but f "
		  << "extension (bit 5) is not enabled -- ignored\n";
    }
  flag = flag and isa_.isEnabled(RvExtension::D);
  enableRvd(flag);

  rve_ = value & (URV(1) << ('e' - 'a'));
  rve_ = rve_ and isa_.isEnabled(RvExtension::E);
  if (rve_)
    intRegs_.regs_.resize(16);

  flag = value & (URV(1) << ('i' - 'a'));
  if (not flag and not rve_ and verbose)
    std::cerr << "Bit 8 (i extension) is cleared in the MISA register "
	      << " but extension is mandatory -- assuming bit 8 set\n";

  rvm_ = value & (URV(1) << ('m' - 'a'));
  rvm_ = rvm_ and isa_.isEnabled(RvExtension::M);

  flag = value & (URV(1) << ('s' - 'a'));  // Supervisor-mode option.
  enableSupervisorMode(flag);

  flag = value & (URV(1) << ('u' - 'a'));  // User-mode option.
  enableUserMode(flag);

  flag = value & (URV(1) << ('v' - 'a'));  // User-mode option.
  flag = flag and isa_.isEnabled(RvExtension::V);
  enableVectorMode(flag);

  if (verbose)
    for (auto ec : { 'b', 'h', 'j', 'k', 'l', 'n', 'o', 'p',
		     'q', 'r', 't', 'w', 'x', 'y', 'z' } )
      {
	unsigned bit = ec - 'a';
	if (value & (URV(1) << bit))
	  std::cerr << "Bit " << bit << " (" << ec << ") set in the MISA "
		    << "register but extension is not supported "
		    << "-- ignored\n";
      }

  if (isa_.isEnabled(RvExtension::Zba))
    enableRvzba(true);
  if (isa_.isEnabled(RvExtension::Zbb))
    enableRvzbb(true);
  if (isa_.isEnabled(RvExtension::Zbc))
    enableRvzbc(true);
  if (isa_.isEnabled(RvExtension::Zbs))
    enableRvzbs(true);
  if (isa_.isEnabled(RvExtension::Zfh))
    enableRvzfh(true);
  if (isa_.isEnabled(RvExtension::Zfhmin))
    enableRvzfhmin(true);
  if (isa_.isEnabled(RvExtension::Zknd))
    enableRvzknd(true);
  if (isa_.isEnabled(RvExtension::Zkne))
    enableRvzkne(true);
  if (isa_.isEnabled(RvExtension::Zknh))
    enableRvzknh(true);
  if (isa_.isEnabled(RvExtension::Zbkb))
    enableRvzbkb(true);
  if (isa_.isEnabled(RvExtension::Zksed))
    enableRvzksed(true);
  if (isa_.isEnabled(RvExtension::Zksh))
    enableRvzksh(true);
  if (isa_.isEnabled(RvExtension::Svinval))
    enableRvsvinval(true);
  if (isa_.isEnabled(RvExtension::Zicbom))
    enableRvzicbom(true);
  if (isa_.isEnabled(RvExtension::Zicboz))
    enableRvzicboz(true);
}


static
Pmp::Mode
getModeFromPmpconfigByte(uint8_t byte)
{
  unsigned m = 0;

  if (byte & 1) m = Pmp::Read  | m;
  if (byte & 2) m = Pmp::Write | m;
  if (byte & 4) m = Pmp::Exec  | m;

  return Pmp::Mode(m);
}


template <typename URV>
void
Hart<URV>::updateMemoryProtection()
{
  pmpManager_.reset();

  const unsigned count = 64;
  unsigned impCount = 0;  // Count of implemented PMP registers

  for (unsigned ix = 0; ix < count; ++ix)
    {
      uint64_t low = 0, high = 0;
      Pmp::Type type = Pmp::Type::Off;
      Pmp::Mode mode = Pmp::Mode::None;
      bool locked = false;

      if (unpackMemoryProtection(ix, type, mode, locked, low, high))
        {
          impCount++;
          if (type != Pmp::Type::Off)
            pmpManager_.defineRegion(low, high, type, mode, ix, locked);
        }
    }

  pmpEnabled_ = impCount > 0;
  pmpManager_.enable(pmpEnabled_);
}


template <typename URV>
bool
Hart<URV>::unpackMemoryProtection(unsigned entryIx, Pmp::Type& type,
                                  Pmp::Mode& mode, bool& locked,
                                  uint64_t& low, uint64_t& high) const
{
  low = high = 0;
  type = Pmp::Type::Off;
  mode = Pmp::Mode::None;
  locked = false;

  if (entryIx >= 64)
    return false;
  
  CsrNumber csrn = CsrNumber(unsigned(CsrNumber::PMPADDR0) + entryIx);

  unsigned config = csRegs_.getPmpConfigByteFromPmpAddr(csrn);
  type = Pmp::Type((config >> 3) & 3);
  locked = config & 0x80;
  mode = getModeFromPmpconfigByte(config);

  URV pmpVal = 0;
  if (not peekCsr(csrn, pmpVal))
    return false;   // Unimplemented PMPADDR reg.  Should not happen.

  if (type == Pmp::Type::Off)
    return true;   // Entry is off.

  unsigned pmpG = csRegs_.getPmpG();

  if (type == Pmp::Type::Tor)    // Top of range
    {
      if (entryIx > 0)
        {
          URV prevVal = 0;
          CsrNumber lowerCsrn = CsrNumber(unsigned(csrn) - 1);
          peekCsr(lowerCsrn, prevVal);
          low = prevVal;
          low = (low >> pmpG) << pmpG;  // Clear least sig G bits.
          low = low << 2;
        }
              
      high = pmpVal;
      high = (high >> pmpG) << pmpG;
      high = high << 2;
      if (high == 0)
        {
          type = Pmp::Type::Off;  // Empty range.
          return true;
        }

      high = high - 1;
      return true;
    }

  uint64_t sizeM1 = 3;     // Size minus 1
  uint64_t napot = pmpVal;  // Naturally aligned power of 2.
  if (type == Pmp::Type::Napot)  // Naturally algined power of 2.
    {
      unsigned rzi = 0;  // Righmost-zero-bit index in pmpval.
      if (pmpVal == URV(-1))
        {
          // Handle special case where pmpVal is set to maximum value
          napot = 0;
          rzi = mxlen_;
        }
      else
        {
          rzi = __builtin_ctzl(~pmpVal); // rightmost-zero-bit ix.
          napot = (napot >> rzi) << rzi; // Clear bits below rightmost zero bit.
        }

      // Avoid overflow when computing 2 to the power 64 or
      // higher. This is incorrect but should work in practice where
      // the physical address space is 64-bit wide or less.
      if (rzi + 3 >= 64)
        sizeM1 = -1L;
      else
        sizeM1 = (uint64_t(1) << (rzi + 3)) - 1;
    }
  else
    assert(type == Pmp::Type::Na4);

  low = napot;
  low = (low >> pmpG) << pmpG;
  low = low << 2;
  high = low + sizeM1;
  return true;
}


template <typename URV>
void
Hart<URV>::updateAddressTranslation()
{
  if (not isRvs())
    return;

  URV value = 0;
  if (not peekCsr(CsrNumber::SATP, value))
    return;

  uint32_t prevAsid = virtMem_.addressSpace();

  URV mode = 0, asid = 0, ppn = 0;
  if constexpr (sizeof(URV) == 4)
    {
      mode = value >> 31;
      asid = (value >> 22) & 0x1ff;
      ppn = value & 0x3fffff;  // Least sig 22 bits
    }
  else
    {
      mode = value >> 60;
      if ((mode >= 1 and mode <= 7) or mode >= 12)
        mode = 0;  // 1-7 and 12-15 are reserved in version 1.12 of sepc.
      asid = (value >> 44) & 0xffff;
      ppn = value & 0xfffffffffffll;  // Least sig 44 bits
    }

  virtMem_.setMode(VirtMem::Mode(mode));
  virtMem_.setAddressSpace(asid);
  virtMem_.setPageTableRootPage(ppn);

  if (asid != prevAsid)
    invalidateDecodeCache();
}


template <typename URV>
void
Hart<URV>::reset(bool resetMemoryMappedRegs)
{
  privMode_ = PrivilegeMode::Machine;

  intRegs_.reset();
  csRegs_.reset();
  vecRegs_.reset();

  // Suppress resetting memory mapped register on initial resets sent
  // by the test bench. Otherwise, initial resets obliterate memory
  // mapped register data loaded from the ELF/HEX file.
  if (resetMemoryMappedRegs)
    memory_.resetMemoryMappedRegisters();
  cancelLr(); // Clear LR reservation (if any).

  clearPendingNmi();

  setPc(resetPc_);
  currPc_ = pc_;
  bbPc_ = pc_;

  // Enable extensions if corresponding bits are set in the MISA CSR.
  processExtensions();

  perfControl_ = ~uint32_t(0);
  URV value = 0;
  if (peekCsr(CsrNumber::MCOUNTINHIBIT, value))
    perfControl_ = ~value;

  prevPerfControl_ = perfControl_;

  debugMode_ = false;
  debugStepMode_ = false;

  dcsrStepIe_ = false;
  dcsrStep_ = false;

  if (peekCsr(CsrNumber::DCSR, value))
    {
      dcsrStep_ = (value >> 2) & 1;
      dcsrStepIe_ = (value >> 11) & 1;
    }

  resetVector();
  resetFloat();

  // Update cached values of mstatus.mpp and mstatus.mprv and mstatus.fs.
  updateCachedMstatusFields();

  updateAddressTranslation();

  updateMemoryProtection();
  countImplementedPmpRegisters();

  csRegs_.updateCounterPrivilege();

  alarmLimit_ = alarmInterval_? alarmInterval_ + instCounter_ : ~uint64_t(0);
  consecutiveIllegalCount_ = 0;

  // Trigger software interrupt in hart 0 on reset.
  if (clintSiOnReset_ and hartIx_ == 0)
    pokeMemory(clintStart_, uint32_t(1), true);

  clearTraceData();

  decoder_.enableRv64(isRv64());
}


template <typename URV>
void
Hart<URV>::resetVector()
{
  // If vector extension enabled but vectors not configured, then
  // configure for 128-bits per regiser and 32-bits per elemement.
  if (isRvv())
    {
      if (vecRegs_.registerCount() == 0)
	vecRegs_.config(16 /*bytesPerReg*/, 1 /*minBytesPerElem*/,
			4 /*maxBytesPerElem*/, nullptr /*minSewPerLmul*/);
      unsigned bytesPerReg = vecRegs_.bytesPerRegister();
      csRegs_.configCsr("vlenb", true, bytesPerReg, 0, 0, false, false);
    }

  // Make cached vector engine parameters match reset value of the VTYPE CSR.
  URV value = 0;
  if (peekCsr(CsrNumber::VTYPE, value))
    {
      bool vill = (value >> (8*sizeof(URV) - 1)) & 1;
      bool ma = (value >> 7) & 1;
      bool ta = (value >> 6) & 1;
      GroupMultiplier gm = GroupMultiplier(value & 7);
      ElementWidth ew = ElementWidth((value >> 3) & 7);
      vecRegs_.updateConfig(ew, gm, ma, ta, vill);
    }

  // Set VS to initial in MSTATUS if linux/newlib emulation. This
  // allows linux/newlib program to run without startup code.
  if (isRvv() and (newlib_ or linux_))
    {
      URV val = csRegs_.peekMstatus();
      MstatusFields<URV> fields(val);
      fields.bits_.VS = unsigned(FpFs::Initial);
      csRegs_.write(CsrNumber::MSTATUS, PrivilegeMode::Machine, fields.value_);
    }
}


template <typename URV>
void
Hart<URV>::updateCachedMstatusFields()
{
  URV csrVal = csRegs_.peekMstatus();
  MstatusFields<URV> msf(csrVal);
  mstatusMpp_ = PrivilegeMode(msf.bits_.MPP);
  mstatusMprv_ = msf.bits_.MPRV;
  mstatusFs_ = FpFs(msf.bits_.FS);
  mstatusVs_ = FpFs(msf.bits_.VS);

  virtMem_.setExecReadable(msf.bits_.MXR);
  virtMem_.setSupervisorAccessUser(msf.bits_.SUM);
}


template <typename URV>
bool
Hart<URV>::setInitialStateFile(const std::string& path)
{
  initStateFile_ = fopen(path.c_str(), "w");
  if (not initStateFile_)
    {
      std::cerr << "Failed to open '" << path << "' for output\n";
      return false;
    }
  return true;
}


template <typename URV>
bool
Hart<URV>::peekMemory(uint64_t address, uint8_t& val, bool usePma) const
{
  return memory_.peek(address, val, usePma);
}
  

template <typename URV>
bool
Hart<URV>::peekMemory(uint64_t address, uint16_t& val, bool usePma) const
{
  return memory_.peek(address, val, usePma);
}


template <typename URV>
bool
Hart<URV>::peekMemory(uint64_t address, uint32_t& val, bool usePma) const
{
  return memory_.peek(address, val, usePma);
}


template <typename URV>
bool
Hart<URV>::peekMemory(uint64_t address, uint64_t& val, bool usePma) const
{
  uint32_t high = 0, low = 0;

  if (memory_.peek(address, low, usePma) and memory_.peek(address + 4, high, usePma))
    {
      val = (uint64_t(high) << 32) | low;
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::pokeMemory(uint64_t addr, uint8_t val, bool usePma)
{
  std::lock_guard<std::mutex> lock(memory_.lrMutex_);

  memory_.invalidateOtherHartLr(hartIx_, addr, sizeof(val));

  if (memory_.poke(addr, val, usePma))
    {
      invalidateDecodeCache(addr, sizeof(val));
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::pokeMemory(uint64_t addr, uint16_t val, bool usePma)
{
  std::lock_guard<std::mutex> lock(memory_.lrMutex_);

  memory_.invalidateOtherHartLr(hartIx_, addr, sizeof(val));

  if (memory_.poke(addr, val, usePma))
    {
      invalidateDecodeCache(addr, sizeof(val));
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::pokeMemory(uint64_t addr, uint32_t val, bool usePma)
{
  // We allow poke to bypass masking for memory mapped registers
  // otherwise, there is no way for external driver to clear bits that
  // are read-only to this hart.

  std::lock_guard<std::mutex> lock(memory_.lrMutex_);

  memory_.invalidateOtherHartLr(hartIx_, addr, sizeof(val));

  URV adjusted = val;
  if (addr >= clintStart_ and addr < clintLimit_)
    {
      processClintWrite(addr, sizeof(val), adjusted);
      val = adjusted;
    }

  if (memory_.poke(addr, val, usePma))
    {
      invalidateDecodeCache(addr, sizeof(val));
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::pokeMemory(uint64_t addr, uint64_t val, bool usePma)
{
  std::lock_guard<std::mutex> lock(memory_.lrMutex_);

  memory_.invalidateOtherHartLr(hartIx_, addr, sizeof(val));

  if (memory_.poke(addr, val, usePma))
    {
      invalidateDecodeCache(addr, sizeof(val));
      return true;
    }

  return false;
}


template <typename URV>
void
Hart<URV>::setPendingNmi(NmiCause cause)
{
  // First nmi sets the cause. The cause is sticky.
  if (not nmiPending_)
    nmiCause_ = cause;

  nmiPending_ = true;

  // Set the nmi pending bit in the DCSR register.
  URV val = 0;  // DCSR value
  if (peekCsr(CsrNumber::DCSR, val))
    {
      val |= URV(1) << 3;  // nmip bit
      pokeCsr(CsrNumber::DCSR, val);
      recordCsrWrite(CsrNumber::DCSR);
    }
}


template <typename URV>
void
Hart<URV>::clearPendingNmi()
{
  nmiPending_ = false;
  nmiCause_ = NmiCause::UNKNOWN;

  URV val = 0;  // DCSR value
  if (peekCsr(CsrNumber::DCSR, val))
    {
      val &= ~(URV(1) << 3);  // nmip bit
      pokeCsr(CsrNumber::DCSR, val);
      recordCsrWrite(CsrNumber::DCSR);
    }
}


template <typename URV>
void
Hart<URV>::setToHostAddress(uint64_t address)
{
  toHost_ = URV(address);
  toHostValid_ = true;
}


template <typename URV>
void
Hart<URV>::clearToHostAddress()
{
  toHost_ = 0;
  toHostValid_ = false;
}


template <typename URV>
inline
void
Hart<URV>::execBeq(const DecodedInst* di)
{
  URV v1 = intRegs_.read(di->op0()),  v2 = intRegs_.read(di->op1());
  if (v1 != v2)
    return;

  URV nextPc = currPc_ + di->op2As<SRV>();
  if (not isRvc() and (nextPc & 3))
    {
      // Target must be word aligned if C is off.
      initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
    }
  else
    {
      setPc(nextPc);
      lastBranchTaken_ = true;
    }
}


template <typename URV>
inline
void
Hart<URV>::execBne(const DecodedInst* di)
{
  URV v1 = intRegs_.read(di->op0()),  v2 = intRegs_.read(di->op1());
  if (v1 == v2)
    return;

  URV nextPc = currPc_ + di->op2As<SRV>();
  if (not isRvc() and (nextPc & 3))
    {
      // Target must be word aligned if C is off.
      initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
    }
  else
    {
      setPc(nextPc);
      lastBranchTaken_ = true;
    }
}


template <typename URV>
inline
void
Hart<URV>::execAddi(const DecodedInst* di)
{
  SRV imm = di->op2As<SRV>();
  SRV v = intRegs_.read(di->op1()) + imm;
  intRegs_.write(di->op0(), v);
}


template <typename URV>
inline
void
Hart<URV>::execAdd(const DecodedInst* di)
{
  URV v = intRegs_.read(di->op1()) + intRegs_.read(di->op2());
  intRegs_.write(di->op0(), v);
}


template <typename URV>
inline
void
Hart<URV>::execAndi(const DecodedInst* di)
{
  SRV imm = di->op2As<SRV>();
  URV v = intRegs_.read(di->op1()) & imm;
  intRegs_.write(di->op0(), v);
}


static
void
printUnsignedHisto(const char* tag, const std::vector<uintmax_t>& histo,
		   FILE* file)
{
  if (histo.size() < 7)
    return;

  if (histo.at(0))
    fprintf(file, "    %s 0           %jd\n", tag, histo.at(0));
  if (histo.at(1))
    fprintf(file, "    %s 1           %jd\n", tag, histo.at(1));
  if (histo.at(2))
    fprintf(file, "    %s 2           %jd\n", tag, histo.at(2));
  if (histo.at(3))
    fprintf(file, "    %s (2,     16] %jd\n", tag, histo.at(3));
  if (histo.at(4))
    fprintf(file, "    %s (16,    1k] %jd\n", tag, histo.at(4));
  if (histo.at(5))
    fprintf(file, "    %s (1k,   64k] %jd\n", tag, histo.at(5));
  if (histo.at(6))
    fprintf(file, "    %s > 64k       %jd\n", tag, histo.at(6));
}


static
void
printSignedHisto(const char* tag, const std::vector<uintmax_t>& histo,
		 FILE* file)
{
  if (histo.size() < 13)
    return;

  if (histo.at(0))
    fprintf(file, "    %s <= -64k     %jd\n", tag, histo.at(0));
  if (histo.at(1))
    fprintf(file, "    %s (-64k, -1k] %jd\n", tag, histo.at(1));
  if (histo.at(2))
    fprintf(file, "    %s (-1k,  -16] %jd\n", tag, histo.at(2));
  if (histo.at(3))
    fprintf(file, "    %s (-16,   -3] %jd\n", tag, histo.at(3));
  if (histo.at(4))
    fprintf(file, "    %s -2          %jd\n", tag, histo.at(4));
  if (histo.at(5))
    fprintf(file, "    %s -1          %jd\n", tag, histo.at(5));
  if (histo.at(6))
    fprintf(file, "    %s 0           %jd\n", tag, histo.at(6));
  if (histo.at(7))
    fprintf(file, "    %s 1           %jd\n", tag, histo.at(7));
  if (histo.at(8))
    fprintf(file, "    %s 2           %jd\n", tag, histo.at(8));
  if (histo.at(9))
    fprintf(file, "    %s (2,     16] %jd\n", tag, histo.at(9));
  if (histo.at(10))
    fprintf(file, "    %s (16,    1k] %jd\n", tag, histo.at(10));
  if (histo.at(11))	              
    fprintf(file, "    %s (1k,   64k] %jd\n", tag, histo.at(11));
  if (histo.at(12))	              
    fprintf(file, "    %s > 64k       %jd\n", tag, histo.at(12));
}


enum class FpKinds { PosInf, NegInf, PosNormal, NegNormal, PosSubnormal, NegSubnormal,
                     PosZero, NegZero, QuietNan, SignalingNan };


static
void
printFpHisto(const char* tag, const std::vector<uintmax_t>& histo, FILE* file)
{
  for (unsigned i = 0; i <= unsigned(FpKinds::SignalingNan); ++i)
    {
      FpKinds kind = FpKinds(i);
      uintmax_t freq = histo.at(i);
      if (not freq)
        continue;

      switch (kind)
        {
        case FpKinds::PosInf:
          fprintf(file, "    %s pos_inf       %jd\n", tag, freq);
          break;

        case FpKinds::NegInf:
          fprintf(file, "    %s neg_inf       %jd\n", tag, freq);
          break;

        case FpKinds::PosNormal:
          fprintf(file, "    %s pos_normal    %jd\n", tag, freq);
          break;

        case FpKinds::NegNormal:
          fprintf(file, "    %s neg_normal    %jd\n", tag, freq);
          break;

        case FpKinds::PosSubnormal:
          fprintf(file, "    %s pos_subnormal %jd\n", tag, freq);
          break;

        case FpKinds::NegSubnormal:
          fprintf(file, "    %s neg_subnormal %jd\n", tag, freq);
          break;

        case FpKinds::PosZero:
          fprintf(file, "    %s pos_zero      %jd\n", tag, freq);
          break;

        case FpKinds::NegZero:
          fprintf(file, "    %s neg_zero      %jd\n", tag, freq);
          break;

        case FpKinds::QuietNan:
          fprintf(file, "    %s quiet_nan     %jd\n", tag, freq);
          break;

        case FpKinds::SignalingNan:
          fprintf(file, "    %s signaling_nan %jd\n", tag, freq);
          break;
        }
    }
}


template <typename URV>
void
Hart<URV>::reportInstructionFrequency(FILE* file) const
{
  std::vector<size_t> indices;
  instProfs_.sort(indices);

  for (auto profIx : indices)
    {
      const InstProfile* profPtr = instProfs_.ithEntry(profIx);
      if (not profPtr or not profPtr->freq_)
	continue;

      const InstProfile& prof = *profPtr;
      const InstEntry& entry = decoder_.getInstructionEntry(prof.id_);

      std::string instr;
      // Don't collect non-vector repeats
      if (entry.isVector())
        instr = entry.name() + "." + VecRegs::to_string(prof.elemWidth_);
      else if (prof.elemWidth_ == ElementWidth::Byte)
        instr = entry.name();
      else
        continue;

      fprintf(file, "%s %jd\n", instr.c_str(), prof.freq_);

      uint64_t count = 0;
      for (auto n : prof.destRegFreq_) count += n;
      if (count)
	{
          fprintf(file, "  +rd");
	  auto regCount = prof.destRegFreq_.size();
	  for (unsigned i = 0; i < regCount; ++i)
	    if (prof.destRegFreq_.at(i))
              fprintf(file, " %d:%jd", i, prof.destRegFreq_.at(i));
          fprintf(file, "\n");
	}

      unsigned srcIx = 0;

      for (unsigned opIx = 0; opIx < entry.operandCount(); ++opIx)
        {
	  auto mode = entry.ithOperandMode(opIx);
	  auto type = entry.ithOperandType(opIx);
          if ((mode == OperandMode::Read or mode == OperandMode::ReadWrite) and
              type != OperandType::Imm and type != OperandType::None)
            {
              uint64_t count = 0;
              for (auto n : prof.srcRegFreq_.at(srcIx))
                count += n;
              if (count)
                {
                  const auto& regFreq = prof.srcRegFreq_.at(srcIx);
		  auto regCount = regFreq.size();
                  fprintf(file, "  +rs%d", srcIx + 1);
                  for (unsigned i = 0; i < regCount; ++i)
                    if (regFreq.at(i))
                      fprintf(file, " %d:%jd", i, regFreq.at(i));
                  fprintf(file, "\n");

                  const auto& histo = prof.srcHisto_.at(srcIx);
                  std::string tag = std::string("+hist") + std::to_string(srcIx + 1);
                  if (entry.ithOperandType(opIx) == OperandType::FpReg)
                    printFpHisto(tag.c_str(), histo, file);
                  else if (entry.isUnsigned())
                    printUnsignedHisto(tag.c_str(), histo, file);
                  else
                    printSignedHisto(tag.c_str(), histo, file);
                }

              srcIx++;
            }
	}

      if (prof.hasImm_)
	{
	  fprintf(file, "  +imm  min:%d max:%d\n", prof.minImm_, prof.maxImm_);
	  printSignedHisto("+hist ", prof.srcHisto_.back(), file);
	}

      if (prof.user_)
        fprintf(file, "  +user %" PRIu64 "\n", prof.user_);
      if (prof.supervisor_)
        fprintf(file, "  +supervisor %" PRIu64 "\n", prof.supervisor_);
      if (prof.machine_)
        fprintf(file, "  +machine %" PRIu64 "\n", prof.machine_);
    }
}


template <typename URV>
void
Hart<URV>::reportTrapStat(FILE* file) const
{
  fprintf(file, "\n");
  fprintf(file, "Interrupts (incuding NMI): %" PRIu64 "\n", interruptCount_);
  for (unsigned i = 0; i < interruptStat_.size(); ++i)
    {
      InterruptCause cause = InterruptCause(i);
      uint64_t count = interruptStat_.at(i);
      if (not count)
        continue;
      switch(cause)
        {
        case InterruptCause::S_SOFTWARE:
          fprintf(file, "  + S_SOFTWARE  : %" PRIu64 "\n", count);
          break;
        case InterruptCause::M_SOFTWARE:
          fprintf(file, "  + M_SOFTWARE  : %" PRIu64 "\n", count);
          break;
        case InterruptCause::S_TIMER   :
          fprintf(file, "  + S_TIMER     : %" PRIu64 "\n", count);
          break;
        case InterruptCause::M_TIMER   :
          fprintf(file, "  + M_TIMER     : %" PRIu64 "\n", count);
          break;
        case InterruptCause::S_EXTERNAL:
          fprintf(file, "  + S_EXTERNAL  : %" PRIu64 "\n", count);
          break;
        case InterruptCause::M_EXTERNAL:
          fprintf(file, "  + M_EXTERNAL  : %" PRIu64 "\n", count);
          break;
        case InterruptCause::M_INT_TIMER1:
          fprintf(file, "  + M_INT_TIMER1: %" PRIu64 "\n", count);
          break;
        case InterruptCause::M_INT_TIMER0:
          fprintf(file, "  + M_INT_TIMER0: %" PRIu64 "\n", count);
          break;
        case InterruptCause::M_LOCAL :
          fprintf(file, "  + M_LOCAL     : %" PRIu64 "\n", count);
          break;
        default:
          fprintf(file, "  + ????        : %" PRIu64 "\n", count);
        }
    }

  fprintf(file, "\n");
  fprintf(file, "Non maskable interrupts: %" PRIu64 "\n", nmiCount_);

  fprintf(file, "\n");
  fprintf(file, "Exceptions: %" PRIu64 "\n", exceptionCount_);
  for (unsigned i = 0; i < exceptionStat_.size(); ++i)
    {
      ExceptionCause cause = ExceptionCause(i);
      uint64_t count = exceptionStat_.at(i);
      if (not count)
        continue;

      switch(cause)
        {
        case ExceptionCause::INST_ADDR_MISAL :
          fprintf(file, "  + INST_ADDR_MISAL : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::INST_ACC_FAULT  :
          fprintf(file, "  + INST_ACC_FAULT  : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::ILLEGAL_INST    :
          fprintf(file, "  + ILLEGAL_INST    : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::BREAKP          :
          fprintf(file, "  + BREAKP          : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::LOAD_ADDR_MISAL :
          fprintf(file, "  + LOAD_ADDR_MISAL : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::LOAD_ACC_FAULT  :
          fprintf(file, "  + LOAD_ACC_FAULT  : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::STORE_ADDR_MISAL:
          fprintf(file, "  + STORE_ADDR_MISAL: %" PRIu64 "\n", count);
          break;
        case ExceptionCause::STORE_ACC_FAULT :
          fprintf(file, "  + STORE_ACC_FAULT : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::U_ENV_CALL      :
          fprintf(file, "  + U_ENV_CALL      : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::S_ENV_CALL      :
          fprintf(file, "  + S_ENV_CALL      : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::M_ENV_CALL      :
          fprintf(file, "  + M_ENV_CALL      : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::INST_PAGE_FAULT :
          fprintf(file, "  + INST_PAGE_FAULT : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::LOAD_PAGE_FAULT :
          fprintf(file, "  + LOAD_PAGE_FAULT : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::STORE_PAGE_FAULT:
          fprintf(file, "  + STORE_PAGE_FAULT: %" PRIu64 "\n", count);
          break;
        case ExceptionCause::NONE            :
          fprintf(file, "  + NONE            : %" PRIu64 "\n", count);
          break;
        default:
          fprintf(file, "  + ????            : %" PRIu64 "\n", count);
          break;
        }
    }
}


template <typename URV>
void
Hart<URV>::reportPmpStat(FILE* file) const
{
  std::ostringstream oss;
  pmpManager_.printStats(oss);
  fprintf(file, "%s", oss.str().c_str());
}


template <typename URV>
void
Hart<URV>::reportLrScStat(FILE* file) const
{
  fprintf(file, "Load-reserve dispatched: %jd\n", uintmax_t(lrCount_));
  fprintf(file, "Load-reserve successful: %jd\n", uintmax_t(lrSuccess_)); 
  fprintf(file, "Store-conditional dispatched: %jd\n", uintmax_t(scCount_));
  fprintf(file, "Store-conditional successful: %jd\n", uintmax_t(scSuccess_));
}


template <typename URV>
ExceptionCause
Hart<URV>::determineMisalLoadException(URV /*addr*/, unsigned /*accessSize*/) const
{
  if (not misalDataOk_)
    return ExceptionCause::LOAD_ADDR_MISAL;

  return ExceptionCause::NONE;
}


template <typename URV>
ExceptionCause
Hart<URV>::determineMisalStoreException(URV addr, unsigned accessSize) const
{
  if (not misalDataOk_)
    return ExceptionCause::STORE_ADDR_MISAL;

  uint64_t addr2 = addr + accessSize - 1;

  // Misaligned access to a region with side effect.
  if (not isAddrIdempotent(addr) or not isAddrIdempotent(addr2))
    return ExceptionCause::STORE_ADDR_MISAL;

  return ExceptionCause::NONE;
}


template <typename URV>
void
Hart<URV>::initiateLoadException(ExceptionCause cause, URV addr)
{
  initiateException(cause, currPc_, addr);
}


template <typename URV>
void
Hart<URV>::initiateStoreException(ExceptionCause cause, URV addr)
{
  initiateException(cause, currPc_, addr);
}


template <typename URV>
ExceptionCause
Hart<URV>::determineLoadException(uint64_t& addr1, uint64_t& addr2, unsigned ldSize)
{
  uint64_t va1 = URV(addr1);   // Virtual address. Truncate to 32-bits in 32-bit mode.
  uint64_t va2 = va1;
  addr1 = va1;
  addr2 = va2;  // Phys addr of 2nd page when crossing page boundary.

  // Misaligned load from io section triggers an exception. Crossing
  // dccm to non-dccm causes an exception.
  uint64_t alignMask = ldSize - 1;
  bool misal = addr1 & alignMask;
  misalignedLdSt_ = misal;

  ExceptionCause cause = ExceptionCause::NONE;
  if (misal)
    {
      cause = determineMisalLoadException(addr1, ldSize);
      if (cause == ExceptionCause::LOAD_ADDR_MISAL)
        return cause;  // Misaligned resulting in misaligned-adddress-exception
      va2 = (va1 + ldSize) & ~alignMask;
    }

  // Address translation
  if (isRvs())
    {
      PrivilegeMode mode = mstatusMprv_? mstatusMpp_ : privMode_;
      if (mode != PrivilegeMode::Machine)
        {
	  cause = virtMem_.translateForLoad2(va1, ldSize, mode, addr1, addr2);
          if (cause != ExceptionCause::NONE)
	    return cause;
        }
    }

  // Misaligned resulting in access fault exception.
  if (misal and cause != ExceptionCause::NONE)
    return cause;

  // Physical memory protection. Assuming grain size is >= 8.
  if (pmpEnabled_)
    {
      Pmp pmp = pmpManager_.accessPmp(addr1);
      if (not pmp.isRead(privMode_, mstatusMpp_, mstatusMprv_))
	{
	  addr1 = va1;
	  return ExceptionCause::LOAD_ACC_FAULT;
	}
      if (misal or addr1 != addr2)
	{
	  pmp = pmpManager_.accessPmp(addr2);
	  if (not pmp.isRead(privMode_, mstatusMpp_, mstatusMprv_))
	    {
	      addr1 = va2;
	      return ExceptionCause::LOAD_ACC_FAULT;
	    }
	}
    }

  if (not misal)
    {
      if (not memory_.checkRead(addr1, ldSize))
	return ExceptionCause::LOAD_ACC_FAULT;  // Invalid physical memory attribute.
    }
  else
    {
      uint64_t aligned = addr1 & ~alignMask;
      if (not memory_.checkRead(aligned, ldSize))
	{
	  addr1 = va1;
	  return ExceptionCause::LOAD_ACC_FAULT;  // Invalid physical memory attribute.
	}
      uint64_t next = addr1 == addr2? aligned + ldSize : addr2;
      if (not memory_.checkRead(next, ldSize))
	{
	  addr1 = va2;
	  return ExceptionCause::LOAD_ACC_FAULT;  // Invalid physical memory attribute.
	}
    }

  return ExceptionCause::NONE;
}


template <typename URV>
template <typename LOAD_TYPE>
inline
bool
Hart<URV>::fastLoad(uint64_t addr, uint64_t& value)
{
  // Unsigned version of LOAD_TYPE
  typedef typename std::make_unsigned<LOAD_TYPE>::type ULT;

  ULT uval = 0;
  if (memory_.read(addr, uval))
    {
      if constexpr (std::is_same<ULT, LOAD_TYPE>::value)
        value = uval;
      else
        value = SRV(LOAD_TYPE(uval)); // Sign extend.
      return true;  // Success.
    }
  return false;
}


/// Dump initial state of a memory line to the given file.
static void
dumpInitState(FILE* file, const char* tag, std::unordered_set<uint64_t>& dumpedLines,
	      uint64_t vaddr, uint64_t paddr, Memory& mem)
{
  uint64_t pline = mem.getLineNumber(paddr);
  if (dumpedLines.find(pline) != dumpedLines.end())
    return;  // Already dumped

  dumpedLines.insert(pline);

  uint64_t vline = mem.getLineNumber(vaddr);
  unsigned lineSize = mem.lineSize();
  fprintf(file, "%s,%0jx,%0jx,", tag, uintmax_t(vline*lineSize), uintmax_t(pline*lineSize));

  uint64_t byteAddr = pline * lineSize + lineSize - 1;
  for (unsigned i = 0; i < lineSize; ++i)
    {
      uint8_t byte = 0;
      mem.peek(byteAddr--, byte, false);
      fprintf(file, "%02x", unsigned(byte));
    }
  fprintf(file, "\n");
}


/// Shift executed instruction counter by this amount to produce a
/// fake timer value. For example, if shift amout is 3, we are
/// dividing instruction count by 8 (2 to power 3) to produce a timer
/// value.
unsigned counterToTimeShift = 3;

template <typename URV>
template <typename LOAD_TYPE>
inline
bool
Hart<URV>::load(uint64_t virtAddr, uint64_t& data)
{
  ldStAddr_ = virtAddr;   // For reporting ld/st addr in trace-mode.
  ldStPhysAddr1_ = ldStPhysAddr2_ = virtAddr;
  ldStSize_ = sizeof(LOAD_TYPE);

#ifdef FAST_SLOPPY
  return fastLoad<LOAD_TYPE>(virtAddr, data);
#else

  if (hasActiveTrigger())
    {
      if (ldStAddrTriggerHit(virtAddr, TriggerTiming::Before, true /*isLoad*/,
                             privMode_, isInterruptEnabled()))
	triggerTripped_ = true;
    }

  // Unsigned version of LOAD_TYPE
  typedef typename std::make_unsigned<LOAD_TYPE>::type ULT;

  uint64_t addr1 = virtAddr;
  uint64_t addr2 = addr1;
  auto cause = determineLoadException(addr1, addr2, ldStSize_);
  if (cause != ExceptionCause::NONE)
    {
      if (triggerTripped_)
        return false;
      initiateLoadException(cause, addr1);
      return false;
    }
  ldStPhysAddr1_ = addr1;
  ldStPhysAddr2_ = addr2;

  // Loading from console-io does a standard input read.
  if (conIoValid_ and addr1 == conIo_ and enableConIn_ and not triggerTripped_)
    {
      SRV val = fgetc(stdin);
      data = val;
      return true;
    }

  ULT narrow = 0;   // Unsigned narrow loaded value
  if (addr1 >= clintStart_ and addr1 < clintLimit_ and addr1 - clintStart_ >= 0xbff8)
    {
      uint64_t tm = instCounter_ >> counterToTimeShift; // Fake time: instr count
      tm = tm >> (addr1 - 0xbff8) * 8;
      narrow = tm;
    }
  else
    {
      bool hasMcmVal = false;
      if (mcm_)
	{
	  uint64_t mcmVal = 0;
	  if (mcm_->getCurrentLoadValue(*this, addr1, ldStSize_, mcmVal))
	    {
	      narrow = mcmVal;
	      hasMcmVal = true;
	    }
	}
      if (not hasMcmVal)
	memRead(addr1, addr2, narrow);
    }

  data = narrow;
  if (not std::is_same<ULT, LOAD_TYPE>::value)
    data = int64_t(LOAD_TYPE(narrow)); // Loading signed: Sign extend.

  if (initStateFile_)
    {
      dumpInitState(initStateFile_, "load", initStateLines_, virtAddr, addr1, memory_);
      if (addr1 != addr2 or memory_.getLineNumber(addr1) != memory_.getLineNumber(addr1 + ldStSize_))
	dumpInitState(initStateFile_, "load", initStateLines_, virtAddr + ldStSize_, addr2 + ldStSize_, memory_);
    }

  // Check for load-data-trigger.
  if (hasActiveTrigger())
    {
      TriggerTiming timing = TriggerTiming::Before;
      bool isLoad = true;
      if (ldStDataTriggerHit(narrow, timing, isLoad, privMode_,
			     isInterruptEnabled()))
	triggerTripped_ = true;
    }
  if (triggerTripped_)
    return false;

  return true;  // Success.
#endif
}


template <typename URV>
inline
void
Hart<URV>::execLw(const DecodedInst* di)
{
  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();

  uint64_t data = 0;
  if (load<int32_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
inline
void
Hart<URV>::execLh(const DecodedInst* di)
{
  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();

  uint64_t data = 0;
  if (load<int16_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
template <typename STORE_TYPE>
inline
bool
Hart<URV>::fastStore(URV addr, STORE_TYPE storeVal)
{
  ldStAddr_ = addr;   // For reporting ld/st addr in trace-mode.
  ldStPhysAddr1_ = addr;
  ldStSize_ = sizeof(STORE_TYPE);
  ldStData_ = storeVal;

  STORE_TYPE prev = 0;
  memory_.peek(addr, prev, false /*usePma*/);
  ldStPrevData_ = prev;

  if (memory_.write(hartIx_, addr, storeVal))
    {
      ldStWrite_ = true;

      if (toHostValid_ and addr == toHost_ and storeVal != 0)
	{
	  throw CoreException(CoreException::Stop, "write to to-host",
			      toHost_, storeVal);
	}
      return true;
    }

  initiateStoreException(ExceptionCause::STORE_ACC_FAULT, addr);
  return false;
}


#include <termios.h>
#undef VSTART

static bool
hasPendingInput(int fd)
{
  static bool firstTime = true;

  if (firstTime)
    {
      firstTime = false;
      struct termios term;
      tcgetattr(fd, &term);
      cfmakeraw(&term);
      term.c_lflag &= ~ECHO;
      tcsetattr(fd, 0, &term);
    }

  struct pollfd inPollfd;
  inPollfd.fd = fd;
  inPollfd.events = POLLIN;
  int code = poll(&inPollfd, 1, 0);
  return code == 1 and (inPollfd.revents & POLLIN) != 0;
}
  

static int
readCharNonBlocking(int fd)
{
  if (not hasPendingInput(fd))
    return 0;

  char c = 0;
  if (::read(fd, &c, sizeof(c)) == 1)
    return c;

  std::cerr << "readCharNonBlocking: unexpected fail on read\n";
  return -1;
}


template <typename URV>
template <typename STORE_TYPE>
void
Hart<URV>::handleStoreToHost(URV physAddr, STORE_TYPE storeVal)
{
  uint64_t val = storeVal;
  uint64_t data = (val << 16) >> 16;
  unsigned cmd = (val >> 48) & 0xff;
  unsigned dev = (val >> 56) & 0xff;
  if (dev == 1)
    {
      if (cmd == 1)
	{
	  char c = data;
	  if (c)
	    {
	      putchar(c);
	      fflush(stdout);
	    }
	}
      else if (cmd == 0 and fromHostValid_)
	{
	  int ch = readCharNonBlocking(fileno(stdin));
	  if (ch < 0)
	    throw CoreException(CoreException::Stop, "EOF", toHost_, val);
	  if (ch > 0)
	    memory_.poke(fromHost_, ((val >> 48) << 48) | uint64_t(ch));
	}
    }
  else if (dev == 0 and cmd == 0)
    {
      ldStWrite_ = true;
      ldStData_ = storeVal;
      memory_.write(hartIx_, physAddr, storeVal);
      if (storeVal & 1)
	throw CoreException(CoreException::Stop, "write to to-host",
			    toHost_, val);
      else
	assert(0);
    }
  else
    {
      ldStWrite_ = true;
      ldStData_ = storeVal;
      memory_.write(hartIx_, physAddr, storeVal);
      assert(0);
    }
}


template <typename URV>
template <typename STORE_TYPE>
inline
bool
Hart<URV>::store(URV virtAddr, STORE_TYPE storeVal)
{
#ifdef FAST_SLOPPY
  return fastStore(virtAddr, storeVal);
#else

  std::lock_guard<std::mutex> lock(memory_.lrMutex_);

  ldStAddr_ = virtAddr;   // For reporting ld/st addr in trace-mode.
  ldStPhysAddr1_ = ldStAddr_;
  ldStSize_ = sizeof(STORE_TYPE);

  // ld/st-address or instruction-address triggers have priority over
  // ld/st access or misaligned exceptions.
  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = false;  // Not a load.
  if (hasTrig and ldStAddrTriggerHit(virtAddr, timing, isLd, privMode_,
                                     isInterruptEnabled()))
    triggerTripped_ = true;

  // Determine if a store exception is possible.
  uint64_t addr1 = virtAddr, addr2 = virtAddr;
  ExceptionCause cause = determineStoreException(addr1, addr2, ldStSize_);
  ldStPhysAddr1_ = addr1;
  ldStPhysAddr2_ = addr2;

  // Consider store-data trigger if there is no trap or if the trap is
  // due to an external cause.
  if (hasTrig and cause == ExceptionCause::NONE)
    if (ldStDataTriggerHit(storeVal, timing, isLd, privMode_,
                           isInterruptEnabled()))
      triggerTripped_ = true;
  if (triggerTripped_)
    return false;

  if (cause != ExceptionCause::NONE)
    {
      initiateStoreException(cause, addr1);
      return false;
    }

  STORE_TYPE temp = 0;
  memPeek(addr1, addr2, temp, false /*usePma*/);
  ldStPrevData_ = temp;

  // If addr is special location, then write to console.
  if (conIoValid_ and addr1 == conIo_)
    {
      if (consoleOut_)
	{
	  fputc(storeVal, consoleOut_);
	  if (storeVal == '\n')
	    fflush(consoleOut_);
	}
      return true;
    }

  if (initStateFile_)
    {
      dumpInitState(initStateFile_, "store", initStateLines_, virtAddr, addr1, memory_);
      if (addr1 != addr2 or memory_.getLineNumber(addr1) != memory_.getLineNumber(addr1 + ldStSize_))
	dumpInitState(initStateFile_, "store", initStateLines_, virtAddr + ldStSize_, addr2 + ldStSize_, memory_);
    }

  // If we write to special location, end the simulation.
  if (toHostValid_ and addr1 == toHost_)
    {
      handleStoreToHost(addr1, storeVal);
      return true;
    }

  memory_.invalidateOtherHartLr(hartIx_, addr1, ldStSize_);
  if (addr2 != addr1)
    memory_.invalidateOtherHartLr(hartIx_, addr1, ldStSize_);
  invalidateDecodeCache(virtAddr, ldStSize_);

  if (mcm_)
    {
      ldStWrite_ = true;
      ldStData_ = storeVal;
      return true;  // Memory updated when merge buffer is written.
    }

  if (addr1 >= clintStart_ and addr1 < clintLimit_)
    {
      assert(addr1 == addr2);
      URV val = storeVal;
      processClintWrite(addr1, ldStSize_, val);
      storeVal = val;
    }

  if (hasInterruptor_ and addr1 == interruptor_ and ldStSize_ == 4)
    processInterruptorWrite(storeVal);

  memWrite(addr1, addr2, storeVal);
  ldStWrite_ = true;
  memPeek(addr1, addr2, temp, false /*usePma*/);
  ldStData_ = temp;
  return true;

#endif
}


template <typename URV>
void
Hart<URV>::processClintWrite(uint64_t addr, unsigned stSize, URV& storeVal)
{
  if (addr >= clintStart_ and addr < clintStart_ + 0x4000)
    {
      unsigned hartIx = (addr - clintStart_) / 4;
      auto hart = indexToHart_(hartIx);
      if (hart and stSize == 4 and (addr & 3) == 0)
	{
	  storeVal = storeVal & 1;  // Only bit zero is implemented.

	  URV mipVal = csRegs_.peekMip();
	  if (storeVal)
	    mipVal = mipVal | (URV(1) << URV(InterruptCause::M_SOFTWARE));
	  else
	    mipVal = mipVal & ~(URV(1) << URV(InterruptCause::M_SOFTWARE));
	  hart->pokeCsr(CsrNumber::MIP, mipVal);
	  recordCsrWrite(CsrNumber::MIP);
	  return;
	}
    }
  else if (addr >= clintStart_ + 0x4000 and addr < clintStart_ + 0xbff8) 
    {
      unsigned hartIx = (addr - clintStart_ - 0x4000) / 8;
      auto hart = indexToHart_(hartIx);
      if (hart and (stSize == 4 or stSize == 8))
	{
	  if (stSize == 4)
	    {
	      if ((addr & 7) == 0)  // Multiple of 8
		{
		  hart->clintAlarm_ = (hart->clintAlarm_ >> 32) << 32;  // Clear low 32
		  hart->clintAlarm_ |= uint32_t(storeVal);  // Update low 32.
		}
	      else if ((addr & 3) == 0)  // Multiple of 4
		{
		  hart->clintAlarm_ = (hart->clintAlarm_ << 32) >> 32;  // Clear high 32
		  hart->clintAlarm_ |= (uint64_t(storeVal) << 32);  // Update high 32.
		}
	    }
	  else if (stSize == 8)
	    {
	      if ((addr & 7) == 0)
		hart->clintAlarm_ = storeVal;

	      // An htif_getc may be pending, send char back to target.  FIX: keep track of pending getc.
	      if (fromHostValid_ and hasPendingInput(fileno(stdin)))
		{
		  uint64_t v = 0;
		  peekMemory(fromHost_, v, true);
		  if (v == 0)
		    memory_.poke(fromHost_, (uint64_t(1) << 56) | char(readCharNonBlocking(fileno(stdin))));
		}
	    }

	  // URV mipVal = hart->csRegs_.peekMip();
	  // mipVal = mipVal & ~(URV(1) << URV(InterruptCause::M_TIMER));
	  // hart->pokeCsr(CsrNumber::MIP, mipVal);
	  return;
	}
    }
  else if (addr - clintStart_ >= 0xbff8)
    return;  // Timer.

  // Address did not match any hart entry in clint.
  storeVal = 0;
}


template <typename URV>
void
Hart<URV>::processInterruptorWrite(uint32_t storeVal)
{
  if (not indexToHart_)
    return;

  unsigned hartIx = storeVal & 0xfff;
  unsigned interruptId = (storeVal >> 12) & 0xff;
  unsigned val = (storeVal >> 20) == 0 ? 0 : 1;
  auto hart = indexToHart_(hartIx);
  if (not hart)
    return;

  URV mipVal = csRegs_.peekMip();
  if (val)
    mipVal = mipVal | (URV(1) << interruptId);
  else
    mipVal = mipVal & ~(URV(1) << interruptId);

  hart->pokeCsr(CsrNumber::MIP, mipVal);
  recordCsrWrite(CsrNumber::MIP);
}


template <typename URV>
inline
void
Hart<URV>::execSw(const DecodedInst* di)
{
  uint32_t rs1 = di->op1();
  URV base = intRegs_.read(rs1);
  URV addr = base + di->op2As<SRV>();
  uint32_t value = uint32_t(intRegs_.read(di->op0()));

  store<uint32_t>(addr, value);
}


template <typename URV>
bool
Hart<URV>::readInst(uint64_t address, uint32_t& inst)
{
  inst = 0;

  uint16_t low;  // Low 2 bytes of instruction.
  if (not memory_.readInst(address, low))
    return false;

  inst = low;

  if ((inst & 0x3) == 3)  // Non-compressed instruction.
    {
      uint16_t high;
      if (not memory_.readInst(address + 2, high))
	return false;
      inst |= (uint32_t(high) << 16);
    }

  return true;
}


template <typename URV>
bool
Hart<URV>::defineMemoryMappedRegisterWriteMask(uint64_t addr, uint32_t mask)
{
  return memory_.defineMemoryMappedRegisterWriteMask(addr, mask);
}


template <typename URV>
inline
bool
Hart<URV>::fetchInst(URV virtAddr, uint64_t& physAddr, uint32_t& inst)
{
  uint64_t addr = virtAddr;

  // Inst address translation and memory protection is not affected by MPRV.
  bool instMprv = false;

  if (isRvs() and privMode_ != PrivilegeMode::Machine)
    {
      if (triggerTripped_)
        return false;

      auto cause = virtMem_.translateForFetch(virtAddr, privMode_, addr);
      if (cause != ExceptionCause::NONE)
        {
          initiateException(cause, virtAddr, virtAddr);
          return false;
        }
    }
  physAddr = virtAddr;

  if (virtAddr & 1)
    {
      if (triggerTripped_)
        return false;
      initiateException(ExceptionCause::INST_ADDR_MISAL, virtAddr, virtAddr);
      return false;
    }

  if ((addr & 3) == 0)   // Word aligned
    {
      if (not memory_.readInst(addr, inst))
        {
          if (triggerTripped_)
            return false;
          initiateException(ExceptionCause::INST_ACC_FAULT, virtAddr, virtAddr);
          return false;
        }

      if (pmpEnabled_)
        {
          Pmp pmp = pmpManager_.accessPmp(addr);
          if (not pmp.isExec(privMode_, mstatusMpp_, instMprv))
            {
              if (triggerTripped_)
                return false;
              initiateException(ExceptionCause::INST_ACC_FAULT, virtAddr, virtAddr);
              return false;
            }
        }

      if (initStateFile_)
	dumpInitState(initStateFile_, "fetch", initStateLines_, virtAddr, physAddr, memory_);

      return true;
    }

  uint16_t half;
  if (not memory_.readInst(addr, half))
    {
      if (triggerTripped_)
        return false;
      initiateException(ExceptionCause::INST_ACC_FAULT, virtAddr, virtAddr);
      return false;
    }

  if (pmpEnabled_)
    {
      Pmp pmp = pmpManager_.accessPmp(addr);
      if (not pmp.isExec(privMode_, mstatusMpp_, instMprv))
        {
          if (triggerTripped_)
            return false;
          initiateException(ExceptionCause::INST_ACC_FAULT, virtAddr, virtAddr);
          return false;
        }
    }

  if (initStateFile_)
    dumpInitState(initStateFile_, "fetch", initStateLines_, virtAddr, physAddr, memory_);
  inst = half;
  if (isCompressedInst(inst))
    return true;

  if (isRvs() and privMode_ != PrivilegeMode::Machine)
    {
      auto cause = virtMem_.translateForFetch(virtAddr+2, privMode_, addr);
      if (cause != ExceptionCause::NONE)
        {
          if (triggerTripped_)
            return false;
          initiateException(cause, virtAddr, virtAddr+2);
          return false;
        }
    }
  else
    addr += 2;

  uint16_t upperHalf;
  if (not memory_.readInst(addr, upperHalf))
    {
      if (triggerTripped_)
        return false;
      initiateException(ExceptionCause::INST_ACC_FAULT, virtAddr, virtAddr + 2);
      return false;
    }

  if (pmpEnabled_)
    {
      Pmp pmp = pmpManager_.accessPmp(addr);
      if (not pmp.isExec(privMode_, mstatusMpp_, instMprv))
        {
          if (triggerTripped_)
            return false;
          initiateException(ExceptionCause::INST_ACC_FAULT, virtAddr, virtAddr + 2);
          return false;
        }
    }

  if (initStateFile_)
    dumpInitState(initStateFile_, "fetch", initStateLines_, virtAddr, addr, memory_);

  inst = inst | (uint32_t(upperHalf) << 16);
  return true;
}


template <typename URV>
bool
Hart<URV>::fetchInstPostTrigger(URV virtAddr, uint64_t& physAddr,
				uint32_t& inst, FILE* traceFile)
{
  if (fetchInst(virtAddr, physAddr, inst))
    return true;

  // Fetch failed: take pending trigger-exception.
  URV info = virtAddr;
  takeTriggerAction(traceFile, virtAddr, info, instCounter_, true);

  return false;
}


template <typename URV>
void
Hart<URV>::illegalInst(const DecodedInst* di)
{
  if (triggerTripped_)
    return;

  uint32_t inst = di->inst();
  if (isCompressedInst(inst))
    inst = inst & 0xffff;

  initiateException(ExceptionCause::ILLEGAL_INST, currPc_, inst);
}


template <typename URV>
void
Hart<URV>::unimplemented(const DecodedInst* di)
{
  illegalInst(di);
}


// Start an asynchronous exception.
template <typename URV>
void
Hart<URV>::initiateInterrupt(InterruptCause cause, URV pc)
{
  bool interrupt = true;
  URV info = 0;  // This goes into mtval.
  initiateTrap(interrupt, URV(cause), pc, info);

  hasInterrupt_ = true;
  interruptCount_++;

  if (not enableCounters_)
    return;

  PerfRegs& pregs = csRegs_.mPerfRegs_;
  if (cause == InterruptCause::M_EXTERNAL)
    pregs.updateCounters(EventNumber::ExternalInterrupt, prevPerfControl_,
                         lastPriv_);
  else if (cause == InterruptCause::M_TIMER)
    pregs.updateCounters(EventNumber::TimerInterrupt, prevPerfControl_,
                         lastPriv_);
}


// Start a synchronous exception.
template <typename URV>
void
Hart<URV>::initiateException(ExceptionCause cause, URV pc, URV info)
{
  // Check if stuck because of lack of exception handler. Disable if
  // you do want the stuck behavior.
  if (true)
    {
      if (instCounter_ == counterAtLastIllegal_ + 1)
	consecutiveIllegalCount_++;
      else
	consecutiveIllegalCount_ = 0;

      if (consecutiveIllegalCount_ > 64)  // FIX: Make a parameter
	{
	  throw CoreException(CoreException::Stop,
			      "64 consecutive illegal instructions",
			      0, 3);
	}

      counterAtLastIllegal_ = instCounter_;
    }

  bool interrupt = false;
  exceptionCount_++;
  hasException_ = true;
  initiateTrap(interrupt, URV(cause), pc, info);

  PerfRegs& pregs = csRegs_.mPerfRegs_;
  if (enableCounters_)
    pregs.updateCounters(EventNumber::Exception, prevPerfControl_,
                         lastPriv_);
}


template <typename URV>
void
Hart<URV>::initiateTrap(bool interrupt, URV cause, URV pcToSave, URV info)
{
  cancelLr(); // Clear LR reservation (if any).

  PrivilegeMode origMode = privMode_;

  // Exceptions are taken in machine mode.
  privMode_ = PrivilegeMode::Machine;
  PrivilegeMode nextMode = PrivilegeMode::Machine;

  // But they can be delegated.
  if (isRvs() and origMode != PrivilegeMode::Machine)
    {
      CsrNumber csrn = interrupt? CsrNumber::MIDELEG : CsrNumber::MEDELEG;
      URV delegVal = 0;
      peekCsr(csrn, delegVal);
      if (delegVal & (URV(1) << cause))
        nextMode = PrivilegeMode::Supervisor;
    }

  CsrNumber epcNum = CsrNumber::MEPC;
  CsrNumber causeNum = CsrNumber::MCAUSE;
  CsrNumber tvalNum = CsrNumber::MTVAL;
  CsrNumber tvecNum = CsrNumber::MTVEC;

  if (nextMode == PrivilegeMode::Supervisor)
    {
      epcNum = CsrNumber::SEPC;
      causeNum = CsrNumber::SCAUSE;
      tvalNum = CsrNumber::STVAL;
      tvecNum = CsrNumber::STVEC;
    }
  else if (nextMode == PrivilegeMode::User)
    {
      epcNum = CsrNumber::UEPC;
      causeNum = CsrNumber::UCAUSE;
      tvalNum = CsrNumber::UTVAL;
      tvecNum = CsrNumber::UTVEC;
    }

  // Save address of instruction that caused the exception or address
  // of interrupted instruction.
  if (not csRegs_.write(epcNum, privMode_, pcToSave & ~(URV(1))))
    assert(0 and "Failed to write EPC register");

  // Save the exception cause.
  URV causeRegVal = cause;
  if (interrupt)
    causeRegVal |= URV(1) << (mxlen_ - 1);
  if (not csRegs_.write(causeNum, privMode_, causeRegVal))
    assert(0 and "Failed to write CAUSE register");

  // Clear mtval on interrupts. Save synchronous exception info.
  if (not csRegs_.write(tvalNum, privMode_, info))
    assert(0 and "Failed to write TVAL register");

  // Update status register saving xIE in xPIE and previous privilege
  // mode in xPP by getting current value of mstatus ...
  URV status = csRegs_.peekMstatus();

  // ... updating its fields
  MstatusFields<URV> msf(status);

  if (nextMode == PrivilegeMode::Machine)
    {
      msf.bits_.MPP = unsigned(origMode);
      msf.bits_.MPIE = msf.bits_.MIE;
      msf.bits_.MIE = 0;
    }
  else if (nextMode == PrivilegeMode::Supervisor)
    {
      msf.bits_.SPP = unsigned(origMode);
      msf.bits_.SPIE = msf.bits_.SIE;
      msf.bits_.SIE = 0;
    }
  else if (nextMode == PrivilegeMode::User)
    {
      msf.bits_.UPIE = msf.bits_.UIE;
      msf.bits_.UIE = 0;
    }

  // ... and putting it back
  if (not csRegs_.write(CsrNumber::MSTATUS, privMode_, msf.value_))
    assert(0 and "Failed to write MSTATUS register");
  updateCachedMstatusFields();
  
  // Set program counter to trap handler address.
  URV tvec = 0;
  if (not csRegs_.read(tvecNum, privMode_, tvec))
    assert(0 and "Failed to read TVEC register");

  URV base = (tvec >> 2) << 2;  // Clear least sig 2 bits.
  tvecMode_ = TrapVectorMode(tvec & 0x3);

  if (tvecMode_ == TrapVectorMode::Vectored and interrupt)
    base = base + 4*cause;

  setPc(base);

  // Change privilege mode.
  privMode_ = nextMode;

  if (instFreq_)
    accumulateTrapStats(false /*isNmi*/);
}


template <typename URV>
void
Hart<URV>::initiateNmi(URV cause, URV pcToSave)
{
  URV nextPc = nmiPc_;
  undelegatedInterrupt(cause, pcToSave, nextPc);
  nmiCount_++;
  if (instFreq_)
    accumulateTrapStats(true);
}


template <typename URV>
void
Hart<URV>::undelegatedInterrupt(URV cause, URV pcToSave, URV nextPc)
{
  hasInterrupt_ = true;
  interruptCount_++;

  cancelLr();  // Clear LR reservation (if any).

  PrivilegeMode origMode = privMode_;

  // NMI is taken in machine mode.
  privMode_ = PrivilegeMode::Machine;

  // Save address of instruction that caused the exception or address
  // of interrupted instruction.
  pcToSave = (pcToSave >> 1) << 1; // Clear least sig bit.
  if (not csRegs_.write(CsrNumber::MEPC, privMode_, pcToSave))
    assert(0 and "Failed to write EPC register");

  // Save the exception cause.
  if (not csRegs_.write(CsrNumber::MCAUSE, privMode_, cause))
    assert(0 and "Failed to write CAUSE register");

  // Clear mtval
  if (not csRegs_.write(CsrNumber::MTVAL, privMode_, 0))
    assert(0 and "Failed to write MTVAL register");

  // Update status register saving xIE in xPIE and previous privilege
  // mode in xPP by getting current value of mstatus ...
  URV status = csRegs_.peekMstatus();
  MstatusFields<URV> msf(status);

  // ... updating its fields
  msf.bits_.MPP = unsigned(origMode);
  msf.bits_.MPIE = msf.bits_.MIE;
  msf.bits_.MIE = 0;

  // ... and putting it back
  if (not csRegs_.write(CsrNumber::MSTATUS, privMode_, msf.value_))
    assert(0 and "Failed to write MSTATUS register");
  updateCachedMstatusFields();
  
  // Clear pending nmi bit in dcsr
  URV dcsrVal = 0;
  if (peekCsr(CsrNumber::DCSR, dcsrVal))
    {
      dcsrVal &= ~(URV(1) << 3);
      pokeCsr(CsrNumber::DCSR, dcsrVal);
      recordCsrWrite(CsrNumber::DCSR);
    }

  setPc(nextPc);
}


template <typename URV>
bool
Hart<URV>::peekIntReg(unsigned ix, URV& val) const
{ 
  if (ix < intRegs_.size())
    {
      val = intRegs_.read(ix);
      return true;
    }
  return false;
}


template <typename URV>
URV
Hart<URV>::peekIntReg(unsigned ix) const
{ 
  assert(ix < intRegs_.size());
  return intRegs_.read(ix);
}


template <typename URV>
bool
Hart<URV>::peekIntReg(unsigned ix, URV& val, std::string& name) const
{ 
  if (ix < intRegs_.size())
    {
      val = intRegs_.read(ix);
      name = intRegName(ix);
      return true;
    }
  return false;
}


template <typename URV>
bool
Hart<URV>::peekFpReg(unsigned ix, uint64_t& val) const
{ 
  if (not isRvf() and not isRvd())
    return false;

  if (ix < fpRegs_.size())
    {
      val = fpRegs_.readBitsRaw(ix);
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::peekUnboxedFpReg(unsigned ix, uint64_t& val) const
{ 
  if (not isRvf() and not isRvd())
    return false;

  if (ix < fpRegs_.size())
    {
      val = fpRegs_.readBitsUnboxed(ix);
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::pokeFpReg(unsigned ix, uint64_t val)
{ 
  if (not isRvf() and not isRvd())
    return false;

  if (ix < fpRegs_.size())
    {
      fpRegs_.pokeBits(ix, val);
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::pokeIntReg(unsigned ix, URV val)
{ 
  if (ix < intRegs_.size())
    {
      intRegs_.poke(ix, val);
      return true;
    }
  return false;
}


template <typename URV>
bool
Hart<URV>::peekCsr(CsrNumber csrn, URV& val) const
{ 
  return csRegs_.peek(csrn, val);
}


template <typename URV>
bool
Hart<URV>::peekCsr(CsrNumber csrn, URV& val, URV& reset, URV& writeMask,
		   URV& pokeMask) const
{ 
  const Csr<URV>* csr = csRegs_.getImplementedCsr(csrn);
  if (not csr)
    return false;

  if (not peekCsr(csrn, val))
    return false;

  reset = csr->getResetValue();
  writeMask = csr->getWriteMask();
  pokeMask = csr->getPokeMask();
  return true;
}


template <typename URV>
bool
Hart<URV>::peekCsr(CsrNumber csrn, URV& val, std::string& name) const
{ 
  const Csr<URV>* csr = csRegs_.getImplementedCsr(csrn);
  if (not csr)
    return false;

  if (not peekCsr(csrn, val))
    return false;

  name = csr->getName();
  return true;
}


template <typename URV>
void
Hart<URV>::postCsrUpdate(CsrNumber csr, URV val)
{
  // This makes sure that counters stop counting after corresponding
  // event reg is written.
  if (enableCounters_)
    if (csr >= CsrNumber::MHPMEVENT3 and csr <= CsrNumber::MHPMEVENT31)
      if (not csRegs_.applyPerfEventAssign())
        std::cerr << "Unexpected applyPerfAssign fail\n";

  if (csr == CsrNumber::DCSR)
    {
      dcsrStep_ = (val >> 2) & 1;
      dcsrStepIe_ = (val >> 11) & 1;
    }
  else if (csr >= CsrNumber::PMPCFG0 and csr <= CsrNumber::PMPCFG15)
    updateMemoryProtection();
  else if (csr >= CsrNumber::PMPADDR0 and csr <= CsrNumber::PMPADDR63)
    {
      unsigned config = csRegs_.getPmpConfigByteFromPmpAddr(csr);
      auto type = Pmp::Type((config >> 3) & 3);
      if (type != Pmp::Type::Off)
        updateMemoryProtection();
    }
  else if (csr == CsrNumber::SATP)
    updateAddressTranslation();
  else if (csr == CsrNumber::FCSR or csr == CsrNumber::FRM or csr == CsrNumber::FFLAGS)
    markFsDirty(); // Update FS field of MSTATS if FCSR is written

  // Update cached values of MSTATUS MPP and MPRV.
  if (csr == CsrNumber::MSTATUS or csr == CsrNumber::SSTATUS)
    updateCachedMstatusFields();

  // Update cached value of VTYPE
  if (csr == CsrNumber::VTYPE)
    {
      bool vill = (val >> (8*sizeof(URV) - 1)) & 1;
      bool ma = (val >> 7) & 1;
      bool ta = (val >> 6) & 1;
      GroupMultiplier gm = GroupMultiplier(val & 7);
      ElementWidth ew = ElementWidth((val >> 3) & 7);
      vecRegs_.updateConfig(ew, gm, ma, ta, vill);
    }

  if (csr == CsrNumber::VSTART or csr == CsrNumber::VXSAT or csr == CsrNumber::VXRM or
      csr == CsrNumber::VCSR or csr == CsrNumber::VL or csr == CsrNumber::VTYPE or
      csr == CsrNumber::VLENB)
    markVsDirty();
}


template <typename URV>
bool
Hart<URV>::pokeCsr(CsrNumber csr, URV val)
{ 
  // Some/all bits of some CSRs are read only to CSR instructions but
  // are modifiable. Use the poke method (instead of write) to make
  // sure modifiable value are changed.
  if (not csRegs_.poke(csr, val))
    return false;

  postCsrUpdate(csr, val);

  return true;
}


template <typename URV>
bool
Hart<URV>::peekVecReg(unsigned ix, std::vector<uint8_t>& value) const
{
  if (not isRvv())
    return false;

  if (ix > vecRegs_.size())
    return false;

  const uint8_t* data = vecRegs_.getVecData(ix);
  unsigned byteCount = vecRegs_.bytesPerRegister();
  value.resize(byteCount);

  for (unsigned i = 0; i < byteCount; ++i)
    value.at(i) = data[byteCount - 1 - i];

  return true;
}


template <typename URV>
bool
Hart<URV>::pokeVecReg(unsigned ix, const std::vector<uint8_t>& val)
{
  if (not isRvv() or ix > vecRegs_.size() or val.empty())
    return false;

  uint8_t* regData = vecRegs_.getVecData(ix);
  if (not regData)
    return false;

  // Bytes in val are in reverse order (most signficant first).
  std::vector<uint8_t> data = val;
  std::reverse(data.begin(), data.end());

  uint32_t count = vecRegs_.bytesPerRegister();
  for (uint32_t i = 0; i < count; ++i)
    {
      uint8_t byte = i < data.size() ? data.at(i) : 0;
      regData[i] = byte;
    }

  return true;
}


template <typename URV>
URV
Hart<URV>::peekPc() const
{
  return pc_;
}


template <typename URV>
void
Hart<URV>::pokePc(URV address)
{
  setPc(address);
  bbPc_ = pc_;
}


template <typename URV>
bool
Hart<URV>::findIntReg(const std::string& name, unsigned& num) const
{
  if (intRegs_.findReg(name, num))
    return true;

  unsigned n = 0;
  if (parseNumber<unsigned>(name, n) and n < intRegs_.size())
    {
      num = n;
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::findFpReg(const std::string& name, unsigned& num) const
{
  if (not isRvf())
    return false;   // Floating point extension not enabled.

  if (fpRegs_.findReg(name, num))
    return true;

  if (name.empty())
    return false;

  if (name.at(0) == 'f')
    {
      std::string numStr = name.substr(1);
      unsigned n = 0;
      if (parseNumber<unsigned>(numStr, num) and n < fpRegCount())
	return true;
    }

  unsigned n = 0;
  if (parseNumber<unsigned>(name, n) and n < fpRegCount())
    {
      num = n;
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::findVecReg(const std::string& name, unsigned& num) const
{
  if (not isRvv())
    return false;

  return vecRegs_.findReg(name, num);
}


template <typename URV>
Csr<URV>*
Hart<URV>::findCsr(const std::string& name)
{
  Csr<URV>* csr = csRegs_.findCsr(name);

  if (not csr)
    {
      unsigned n = 0;
      if (parseNumber<unsigned>(name, n))
	csr = csRegs_.findCsr(CsrNumber(n));
    }

  return csr;
}


template <typename URV>
bool
Hart<URV>::configCsr(const std::string& name, bool implemented, URV resetValue,
                     URV mask, URV pokeMask, bool debug, bool shared)
{
  return csRegs_.configCsr(name, implemented, resetValue, mask, pokeMask,
			   debug, shared);
}


template <typename URV>
bool
Hart<URV>::defineCsr(const std::string& name, CsrNumber num,
		     bool implemented, URV resetVal, URV mask,
		     URV pokeMask, bool isDebug)
{
  bool mandatory = false, quiet = true;
  auto c = csRegs_.defineCsr(name, num, mandatory, implemented, resetVal,
			     mask, pokeMask, isDebug, quiet);
  return c != nullptr;
}


template <typename URV>
bool
Hart<URV>::configIsa(const std::string& isa, bool updateMisa)
{
  if (not isa_.configIsa(isa))
    return false;

  if (updateMisa)
    {
      Csr<URV>* csr = this->findCsr("misa");
      if (not csr)
	return false;

      URV misaReset = csr->getResetValue();
      if (isa_.isEnabled(RvExtension::A))
	misaReset |= URV(1);
      if (isa_.isEnabled(RvExtension::B))
	misaReset |= URV(2);
      if (isa_.isEnabled(RvExtension::C))
	misaReset |= URV(4);
      if (isa_.isEnabled(RvExtension::D))
	misaReset |= URV(8);
      if (isa_.isEnabled(RvExtension::F))
	misaReset |= URV(32);
      if (isa_.isEnabled(RvExtension::M))
	misaReset |= URV(0x1000);
      if (isa_.isEnabled(RvExtension::V))
	misaReset |= URV(0x20000);
  
      URV mask = 0, pokeMask = 0;
      bool implemented = true, isDebug = false, shared = true;

      if (not this->configCsr("misa", implemented, misaReset, mask, pokeMask,
			      isDebug, shared))
	return false;
    }

  return true;
}


template <typename URV>
bool
Hart<URV>::configMachineModePerfCounters(unsigned numCounters)
{
  return csRegs_.configMachineModePerfCounters(numCounters);
}


template <typename URV>
bool
Hart<URV>::configUserModePerfCounters(unsigned numCounters)
{
  return csRegs_.configUserModePerfCounters(numCounters);
}


template <typename URV>
bool
Hart<URV>::configMemoryProtectionGrain(uint64_t size)
{
  bool ok = true;

  if (size < 4)
    {
      std::cerr << "Memory protection grain size (" << size << ") is "
                << "smaller than 4. Using 4.\n";
      size = 4;
      ok = false;
    }

  uint64_t log2Size = static_cast<uint64_t>(std::log2(size));
  uint64_t powerOf2 = uint64_t(1) << log2Size;
  if (size != powerOf2)
    {
      std::cerr << "Memory protection grain size (0x" << std::hex
                << size << ") is not a power of 2. Using: 0x"
                << powerOf2 << '\n';
      size = powerOf2;
      ok = false;
    }

  if (log2Size > 33)
    {
      std::cerr << "Memory protection grain size (0x" << std::hex
                << size << ") is larger than 2 to the power 33. "
                << "Using 2 to the power 33.\n";
      size = uint64_t(1) << 33;
      powerOf2 = size;
      log2Size = 33;
      ok = false;
    }

  unsigned pmpG = log2Size - 2;
  csRegs_.setPmpG(pmpG);

  return ok;
}


template <typename URV>
void
Hart<URV>::recordDivInst(unsigned rd, URV value)
{
  hasLastDiv_ = true;
  priorDivRdVal_ = value;

  lastDivRd_ = rd;
}


template <typename URV>
bool
Hart<URV>::redirectOutputDescriptor(int fd, const std::string& path)
{
  return syscall_.redirectOutputDescriptor(fd, path);
}


template <typename URV>
bool
Hart<URV>::redirectInputDescriptor(int fd, const std::string& path)
{
  return syscall_.redirectInputDescriptor(fd, path);
}


template <typename URV>
bool
Hart<URV>::cancelLastDiv()
{
  if (not hasLastDiv_)
    return false;

  hasLastDiv_ = false;
  return pokeIntReg(lastDivRd_, priorDivRdVal_);
}



template <typename URV>
void
Hart<URV>::undoForTrigger()
{
  unsigned regIx = 0;
  URV value = 0;
  if (intRegs_.getLastWrittenReg(regIx, value))
    {
      pokeIntReg(regIx, value);
      intRegs_.clearLastWrittenReg();
    }

  uint64_t fpVal = 0;
  if (fpRegs_.getLastWrittenReg(regIx, fpVal))
    {
      pokeFpReg(regIx, fpVal);
      fpRegs_.clearLastWrittenReg();
    }

  setPc(currPc_);
}


void
addToSignedHistogram(std::vector<uint64_t>& histo, int64_t val)
{
  if (histo.size() < 13)
    histo.resize(13);

  if (val < 0)
    {
      if      (val <= -64*1024) histo.at(0)++;
      else if (val <= -1024)    histo.at(1)++;
      else if (val <= -16)      histo.at(2)++;
      else if (val < -2)        histo.at(3)++;
      else if (val == -2)       histo.at(4)++;
      else if (val == -1)       histo.at(5)++;
    }
  else
    {
      if      (val == 0)       histo.at(6)++;
      else if (val == 1)       histo.at(7)++;
      else if (val == 2)       histo.at(8)++;
      else if (val <= 16)      histo.at(9)++;
      else if (val <= 1024)    histo.at(10)++;
      else if (val <= 64*1024) histo.at(11)++;
      else                     histo.at(12)++;
    }
}


void
addToUnsignedHistogram(std::vector<uint64_t>& histo, uint64_t val)
{
  if (histo.size() < 13)
    histo.resize(13);

  if      (val == 0)       histo.at(0)++;
  else if (val == 1)       histo.at(1)++;
  else if (val == 2)       histo.at(2)++;
  else if (val <= 16)      histo.at(3)++;
  else if (val <= 1024)    histo.at(4)++;
  else if (val <= 64*1024) histo.at(5)++;
  else                     histo.at(6)++;
}


template <typename FP_TYPE>
void
addToFpHistogram(std::vector<uint64_t>& histo, FP_TYPE val)
{
  if (histo.size() < 13)
    histo.resize(13);

  unsigned type = fpClassifyRiscv(val);
  FpKinds kind = FpKinds::PosNormal;

  if (type == unsigned(FpClassifyMasks::PosInfinity))
    kind = FpKinds::PosInf;
  else if (type == unsigned(FpClassifyMasks::NegInfinity))
    kind = FpKinds::NegInf;
  else if (type == unsigned(FpClassifyMasks::PosNormal))
    kind = FpKinds::PosNormal;
  else if (type == unsigned(FpClassifyMasks::NegNormal))
    kind = FpKinds::NegNormal;
  else if (type == unsigned(FpClassifyMasks::PosSubnormal))
    kind = FpKinds::PosSubnormal;
  else if (type == unsigned(FpClassifyMasks::NegSubnormal))
    kind = FpKinds::NegSubnormal;
  else if (type == unsigned(FpClassifyMasks::PosZero))
    kind = FpKinds::PosZero;
  else if (type == unsigned(FpClassifyMasks::NegZero))
    kind = FpKinds::NegZero;
  else if (type == unsigned(FpClassifyMasks::SignalingNan))
    kind = FpKinds::SignalingNan;
  else if (type == unsigned(FpClassifyMasks::QuietNan))
    kind = FpKinds::QuietNan;
  histo.at(unsigned(kind))++;
}


/// Return true if given hart is in debug mode and the stop count bit of
/// the DSCR register is set.
template <typename URV>
bool
isDebugModeStopCount(const Hart<URV>& hart)
{
  if (not hart.inDebugMode())
    return false;

  URV dcsrVal = 0;
  if (not hart.peekCsr(CsrNumber::DCSR, dcsrVal))
    return false;

  if ((dcsrVal >> 10) & 1)
    return true;  // stop count bit is set
  return false;
}


template <typename URV>
void
Hart<URV>::updatePerformanceCounters(uint32_t inst, const InstEntry& info,
				     uint32_t op0, uint32_t op1)
{
  InstId id = info.instId();

  if (isDebugModeStopCount(*this))
    return;

  if (hasInterrupt_)
    return;

  // We do not update the performance counters if an instruction
  // causes an exception unless it is an ebreak or an ecall.
  if (hasException_ and id != InstId::ecall and id != InstId::ebreak and
      id != InstId::c_ebreak)
    return;

  PerfRegs& pregs = csRegs_.mPerfRegs_;

  pregs.updateCounters(EventNumber::InstCommited, prevPerfControl_,
                       lastPriv_);

  if (isCompressedInst(inst))
    pregs.updateCounters(EventNumber::Inst16Commited, prevPerfControl_,
                         lastPriv_);
  else
    pregs.updateCounters(EventNumber::Inst32Commited, prevPerfControl_,
                         lastPriv_);

  if (info.extension() == RvExtension::I)
    {
      if (id == InstId::ebreak or id == InstId::c_ebreak)
	pregs.updateCounters(EventNumber::Ebreak, prevPerfControl_,
                             lastPriv_);
      else if (id == InstId::ecall)
	pregs.updateCounters(EventNumber::Ecall, prevPerfControl_,
                             lastPriv_);
      else if (id == InstId::fence)
	pregs.updateCounters(EventNumber::Fence, prevPerfControl_,
                             lastPriv_);
      else if (id == InstId::fencei)
	pregs.updateCounters(EventNumber::Fencei, prevPerfControl_,
                             lastPriv_);
      else if (id == InstId::mret)
	pregs.updateCounters(EventNumber::Mret, prevPerfControl_,
                             lastPriv_);
      else if (id != InstId::illegal)
	pregs.updateCounters(EventNumber::Alu, prevPerfControl_,
                             lastPriv_);
    }
  else if (info.isMultiply())
    {
      pregs.updateCounters(EventNumber::Mult, prevPerfControl_,
                           lastPriv_);
      pregs.updateCounters(EventNumber::MultDiv, prevPerfControl_,
                           lastPriv_);
    }
  else if (info.isDivide())
    {
      pregs.updateCounters(EventNumber::Div, prevPerfControl_,
                           lastPriv_);
      pregs.updateCounters(EventNumber::MultDiv, prevPerfControl_,
                           lastPriv_);
    }
  else if (info.isPerfLoad())
    {
      pregs.updateCounters(EventNumber::Load, prevPerfControl_,
                           lastPriv_);
      if (misalignedLdSt_)
	pregs.updateCounters(EventNumber::MisalignLoad, prevPerfControl_,
                             lastPriv_);
      if (isDataAddressExternal(ldStAddr_))
	pregs.updateCounters(EventNumber::BusLoad, prevPerfControl_,
                             lastPriv_);
    }
  else if (info.isPerfStore())
    {
      pregs.updateCounters(EventNumber::Store, prevPerfControl_,
                           lastPriv_);
      if (misalignedLdSt_)
	pregs.updateCounters(EventNumber::MisalignStore, prevPerfControl_,
                             lastPriv_);
      if (isDataAddressExternal(ldStAddr_))
	pregs.updateCounters(EventNumber::BusStore, prevPerfControl_,
                             lastPriv_);
    }
  else if (info.isBitManipulation())
    {
      pregs.updateCounters(EventNumber::Bitmanip, prevPerfControl_,
                           lastPriv_);
    }
  else if (info.isAtomic())
    {
      if (id == InstId::lr_w or id == InstId::lr_d)
	pregs.updateCounters(EventNumber::Lr, prevPerfControl_,
                             lastPriv_);
      else if (id == InstId::sc_w or id == InstId::sc_d)
	pregs.updateCounters(EventNumber::Sc, prevPerfControl_,
                             lastPriv_);
      else
	pregs.updateCounters(EventNumber::Atomic, prevPerfControl_,
                             lastPriv_);
    }
  else if (info.isCsr() and not hasException_)
    {
      if ((id == InstId::csrrw or id == InstId::csrrwi))
	{
	  if (op0 == 0)
	    pregs.updateCounters(EventNumber::CsrWrite, prevPerfControl_,
                                 lastPriv_);
	  else
	    pregs.updateCounters(EventNumber::CsrReadWrite, prevPerfControl_,
                                 lastPriv_);
	}
      else
	{
	  if (op1 == 0)
	    pregs.updateCounters(EventNumber::CsrRead, prevPerfControl_,
                                 lastPriv_);
	  else
	    pregs.updateCounters(EventNumber::CsrReadWrite, prevPerfControl_,
                                 lastPriv_);
	}
      pregs.updateCounters(EventNumber::Csr, prevPerfControl_,
			   lastPriv_);
    }
  else if (info.isBranch())
    {
      pregs.updateCounters(EventNumber::Branch, prevPerfControl_,
                           lastPriv_);
      if (lastBranchTaken_)
	pregs.updateCounters(EventNumber::BranchTaken, prevPerfControl_,
                             lastPriv_);
    }

  // Some insts (e.g. flw) can be both load/store and FP
  if (info.extension() == RvExtension::F)
    {
      pregs.updateCounters(EventNumber::FpSingle, prevPerfControl_,
                           lastPriv_);
    }
  else if (info.extension() == RvExtension::D)
    {
      pregs.updateCounters(EventNumber::FpDouble, prevPerfControl_,
                           lastPriv_);
    }
  else if (info.extension() == RvExtension::Zfh)
    {
      pregs.updateCounters(EventNumber::FpHalf, prevPerfControl_,
                           lastPriv_);
    }
  else if (info.extension() == RvExtension::V)
    {
      pregs.updateCounters(EventNumber::Vector, prevPerfControl_,
                           lastPriv_);
    }

}


template <typename URV>
void
Hart<URV>::updatePerformanceCountersForCsr(const DecodedInst& di)
{
  const InstEntry& info = *(di.instEntry());

  if (not enableCounters_)
    return;

  if (not info.isCsr())
    return;

  updatePerformanceCounters(di.inst(), info, di.op0(), di.op1());
}


template <typename URV>
void
Hart<URV>::accumulateInstructionStats(const DecodedInst& di)
{
  const InstEntry& info = *(di.instEntry());

  if (enableCounters_)
    {
      // For CSR instruction we need to let the counters count before
      // letting CSR instruction write. Consequently we update the counters
      // from within the code executing the CSR instruction.
      if (not info.isCsr())
        updatePerformanceCounters(di.inst(), info, di.op0(), di.op1());
    }

  prevPerfControl_ = perfControl_;

  // We do not update the instruction stats if an instruction causes
  // an exception unless it is an ebreak or an ecall.
  InstId id = info.instId();
  if (hasException_ and id != InstId::ecall and id != InstId::ebreak and
      id != InstId::c_ebreak)
    return;

  misalignedLdSt_ = false;

  if (not instFreq_)
    return;

  InstProfile* profPtr = nullptr;
  if (info.isVector())
    profPtr = instProfs_.find(id, vecRegs_.elemWidth());
  else
    profPtr = instProfs_.find(id);

  if (not profPtr)
    return;

  InstProfile& prof = *profPtr;

  prof.freq_++;
  if (lastPriv_ == PrivilegeMode::User)
    prof.user_++;
  else if (lastPriv_ == PrivilegeMode::Supervisor)
    prof.supervisor_++;
  else if (lastPriv_ == PrivilegeMode::Machine)
    prof.machine_++;

  unsigned opIx = 0;  // Operand index

  unsigned rd = unsigned(intRegCount() + 1);
  OperandType rdType = OperandType::None;
  URV rdOrigVal = 0;   // Integer destination register value.

  uint64_t frdOrigVal = 0;  // Floating point destination register value.

  if (info.isIthOperandWrite(0))
    {
      rdType = info.ithOperandType(0);
      if (rdType == OperandType::IntReg)
	{
	  prof.destRegFreq_.at(di.op0())++; opIx++;
	  intRegs_.getLastWrittenReg(rd, rdOrigVal);
	  assert(rd == di.op0());
	}
      else if (rdType == OperandType::FpReg)
	{
	  prof.destRegFreq_.at(di.op0())++; opIx++;
	  fpRegs_.getLastWrittenReg(rd, frdOrigVal);
	  assert(rd == di.op0());
	}
      else if (rdType == OperandType::VecReg)
	{
	  prof.destRegFreq_.at(di.op0())++; opIx++;
	  rd = di.op0();
	  // unsigned groupX8 = 8;
	  // rd = vecRegs_.getLastWrittenReg(groupX8); 
	  // assert(rd == di.op0());    // Does not work for load seg.
	}
      else if (rdType == OperandType::CsReg)
	{
	  if (prof.destRegFreq_.size() <= di.op0())
	    prof.destRegFreq_.resize(di.op0() + 1);
	  prof.destRegFreq_.at(di.op0())++; opIx++;
	}
    }

  unsigned maxOperand = 4;  // At most 4 operands (including immediate).
  unsigned srcIx = 0;  // Processed source operand rank.

  for (unsigned i = opIx; i < maxOperand; ++i)
    {
      if (info.ithOperandType(i) == OperandType::IntReg)
        {
	  uint32_t regIx = di.ithOperand(i);
	  prof.srcRegFreq_.at(srcIx).at(regIx)++;

          URV val = intRegs_.read(regIx);
          if (regIx == rd and rdType == OperandType::IntReg)
            val = rdOrigVal;
          if (info.isUnsigned())
            addToUnsignedHistogram(prof.srcHisto_.at(srcIx), val);
          else
            addToSignedHistogram(prof.srcHisto_.at(srcIx), SRV(val));
          srcIx++;
	}
      else if (info.ithOperandType(i) == OperandType::FpReg)
        {
	  uint32_t regIx = di.ithOperand(i);
	  prof.srcRegFreq_.at(srcIx).at(regIx)++;

          uint64_t val = fpRegs_.readBitsRaw(regIx);
          if (regIx == rd and rdType == OperandType::FpReg)
            val = frdOrigVal;

	  FpRegs::FpUnion u{val};
	  bool done = false;
	  if (isRvzfh() and fpRegs_.isBoxedHalf(val))
	    {
	      Float16 hpVal = u.hp;
	      addToFpHistogram(prof.srcHisto_.at(srcIx), hpVal);
	      done = true;
	    }
          else if (isRvf())
            {
	      if (not isRvd() or fpRegs_.isBoxedSingle(val))
		{
		  FpRegs::FpUnion u{val};
		  float spVal = u.sp;
		  addToFpHistogram(prof.srcHisto_.at(srcIx), spVal);
		  done = true;
		}
            }
          if (isRvd() and not done)
            {
              FpRegs::FpUnion u{val};
              double dpVal = u.dp;
              addToFpHistogram(prof.srcHisto_.at(srcIx), dpVal);
            }

          srcIx++;
        }
      else if (info.ithOperandType(i) == OperandType::VecReg)
	{
         uint32_t regIx = di.ithOperand(i);
         prof.srcRegFreq_.at(srcIx).at(regIx)++;

         switch (vecRegs_.elemWidth())
           {
             case ElementWidth::Byte:
               {
                 int8_t val;
                 size_t numElem = ((vecRegs_.bytesPerRegister()*vecRegs_.groupMultiplierX8()) >> 3);
                 for (uint32_t elemIx = 0; elemIx < numElem; elemIx++)
                   {
                     assert(vecRegs_.read(regIx, elemIx, vecRegs_.groupMultiplierX8(), val));
                     addToSignedHistogram(prof.srcHisto_.at(srcIx), val);
                   }
                 break;
               }
             case ElementWidth::Half:
               {
                 int16_t val;
                 size_t numElem = (((vecRegs_.bytesPerRegister()*vecRegs_.groupMultiplierX8()) >> 3) >> 1);
                 for (uint32_t elemIx = 0; elemIx < numElem; elemIx++)
                   {
                     assert(vecRegs_.read(regIx, elemIx, vecRegs_.groupMultiplierX8(), val));
                     addToSignedHistogram(prof.srcHisto_.at(srcIx), val);
                   }
                 break;
               }
             case ElementWidth::Word:
               {
                 int32_t val;
                 size_t numElem = (((vecRegs_.bytesPerRegister()*vecRegs_.groupMultiplierX8()) >> 3) >> 2);
                 for (uint32_t elemIx = 0; elemIx < numElem; elemIx++)
                   {
                     assert(vecRegs_.read(regIx, elemIx, vecRegs_.groupMultiplierX8(), val));
                     addToSignedHistogram(prof.srcHisto_.at(srcIx), val);
                   }
                 break;
               }
             case ElementWidth::Word2:
               {
                 int64_t val;
                 size_t numElem = (((vecRegs_.bytesPerRegister()*vecRegs_.groupMultiplierX8()) >> 3) >> 3);
                 for (uint32_t elemIx = 0; elemIx < numElem; elemIx++)
                   {
                     assert(vecRegs_.read(regIx, elemIx, vecRegs_.groupMultiplierX8(), val));
                     addToSignedHistogram(prof.srcHisto_.at(srcIx), val);
                   }
                 break;
               }
             default:
               break;
           }
         srcIx++;
	}
      else if (info.ithOperandType(i) == OperandType::CsReg)
	{
	  uint32_t regIx = di.ithOperand(i);
	  if (prof.srcRegFreq_.at(srcIx).size() <= regIx)
	    prof.srcRegFreq_.at(srcIx).resize(regIx + 1);
	  prof.srcRegFreq_.at(srcIx).at(regIx)++;
	  srcIx++;
	}
      else if (info.ithOperandType(i) == OperandType::Imm)
        {
          int32_t imm = di.ithOperand(i);
          prof.hasImm_ = true;
          if (prof.freq_ == 1)
            {
              prof.minImm_ = prof.maxImm_ = imm;
            }
          else
            {
              prof.minImm_ = std::min(prof.minImm_, imm);
              prof.maxImm_ = std::max(prof.maxImm_, imm);
            }
          addToSignedHistogram(prof.srcHisto_.back(), imm);
        }
    }

  if (prof.hasImm_)
    assert(srcIx + 1 < maxOperand);
}


template <typename URV>
void
Hart<URV>::accumulateTrapStats(bool isNmi)
{
  URV causeVal = 0;
  peekCsr(CsrNumber::MCAUSE, causeVal);

  // If most sig bit of mcause is 1, we have an interrupt.
  bool isInterrupt = causeVal >> (sizeof(causeVal)*8 - 1);

  if (isNmi)
    ;
  else if (isInterrupt)
    {
      causeVal = (causeVal << 1) >> 1;  // Clear most sig bit.
      if (causeVal < interruptStat_.size())
        interruptStat_.at(causeVal)++;
    }
  else
    {
      if (causeVal < exceptionStat_.size())
	exceptionStat_.at(causeVal)++;
    }
}


template <typename URV>
inline
void
Hart<URV>::clearTraceData()
{
  intRegs_.clearLastWrittenReg();
  fpRegs_.clearLastWrittenReg();
  csRegs_.clearLastWrittenRegs();
  memory_.clearLastWriteInfo(hartIx_);
  syscall_.clearMemoryChanges();
  vecRegs_.clearTraceData();
  virtMem_.clearPageTableWalk();
  lastBranchTaken_ = false;
}


template <typename URV>
inline
void
Hart<URV>::setTargetProgramBreak(URV addr)
{
  uint64_t progBreak = addr;

  uint64_t pageAddr = memory_.getPageStartAddr(addr);
  if (pageAddr != addr)
    progBreak = pageAddr + memory_.pageSize();

  syscall_.setTargetProgramBreak(progBreak);
}


template <typename URV>
inline
bool
pokeString(Hart<URV>& hart, uint64_t addr, const std::string& str)
{
  for (uint8_t c : str)
    if (not hart.pokeMemory(addr++, c, true))
      return false;
  return hart.pokeMemory(addr, uint8_t(0), true);   // null byte at end
}


template <typename URV>
inline
bool
Hart<URV>::setTargetProgramArgs(const std::vector<std::string>& args)
{
  URV sp = 0;

  if (not peekIntReg(RegSp, sp))
    return false;

  // Make sp 16-byte aligned.
  if ((sp & 0xf) != 0)
    sp -= (sp & 0xf);

  // Push the arguments on the stack recording their addresses.
  std::vector<URV> argvAddrs;  // Address of the argv strings.
  for (const auto& arg : args)
    {
      sp -= arg.size() + 1;  // Make room for arg and null char.
      argvAddrs.push_back(sp);
      if (not pokeString(*this, sp, arg))
	return false;
    }
  argvAddrs.push_back(0);  // Null pointer at end of argv.

  // Setup envp on the stack (LANG is needed for clang compiled code).
  std::vector<std::string> envs = { "LANG=C", "LC_ALL=C" };
  std::vector<URV> envpAddrs;  // Addresses of the envp strings.
  for (const auto& env : envs)
    {
      sp -= env.size() + 1;  // Make room for env entry and null char.
      envpAddrs.push_back(sp);
      if (not pokeString(*this, sp, env))
	return false;
    }
  envpAddrs.push_back(0);  // Null pointer at end of envp.

  // Push on stack null for aux vector.
  sp -= sizeof(URV);
  if (not memory_.poke(sp, URV(0)))
    return false;

  // Push argv/envp entries on the stack.
  sp -= URV(envpAddrs.size() + argvAddrs.size() + 1) * sizeof(URV); // Make room for envp, argv, & argc

  if ((sp & 0xf) != 0)
    sp -= (sp & 0xf);  // Make sp 16-byte aligned.

  size_t ix = 1;  // Index 0 is for argc

  // Push argv entries on the stack.
  for (const auto addr : argvAddrs)
    if (not memory_.poke(sp + ix++*sizeof(URV), addr))
      return false;

  // Set environ for newlib. This is superfluous for Linux.
  URV ea = sp + ix*sizeof(URV);  // Address of envp array
  ElfSymbol sym;
  if (memory_.findElfSymbol("environ", sym))
    memory_.poke(URV(sym.addr_), ea);

  // Push envp entries on the stack.
  for (const auto addr : envpAddrs)
    if (not memory_.poke(sp + ix++*sizeof(URV), addr))
      return false;

  // Put argc on the stack.
  if (not memory_.poke(sp, URV(args.size())))
    return false;

  if (not pokeIntReg(RegSp, sp))
    return false;

  return true;
}


template <typename URV>
int
Hart<URV>::lastVecReg(const DecodedInst& di, unsigned& group) const
{
  unsigned groupX8 = 8;
  int vecReg = vecRegs_.getLastWrittenReg(groupX8);
  if (vecReg < 0)
    {
      group = 0;
      return vecReg;
    }

  InstId instId = di.instEntry()->instId();

  // We want to report all the registers in the group.
  group  = (groupX8 >= 8) ? groupX8/8 : 1;
  vecReg = di.op0();  // Make sure we have 1st reg in group.
  if ((instId >= InstId::vlsege8_v and instId <= InstId::vssege1024_v) or
      (instId >= InstId::vlsege8ff_v and instId <= InstId::vlsege1024ff_v))
    group = group*di.vecFieldCount();  // Scale by field count
  else if (instId >= InstId::vlssege8_v and instId <= InstId::vsssege1024_v)
    group = group*di.vecFieldCount();  // Scale by field count
  else if (instId >= InstId::vluxsegei8_v and instId <= InstId::vsoxsegei1024_v)
    group = group*di.vecFieldCount();  // Scale by field count

  return vecReg;
}


template <typename URV>
void
Hart<URV>::lastCsr(std::vector<CsrNumber>& csrs,
		   std::vector<unsigned>& triggers) const
{
  csRegs_.getLastWrittenRegs(csrs, triggers);
}


template <typename URV>
void
handleExceptionForGdb(WdRiscv::Hart<URV>& hart, int fd);


// Return true if debug mode is entered and false otherwise.
template <typename URV>
bool
Hart<URV>::takeTriggerAction(FILE* traceFile, URV pc, URV info,
			     uint64_t& counter, bool beforeTiming)
{
  // Check triggers configuration to determine action: take breakpoint
  // exception or enter debugger.

  bool enteredDebug = false;

  if (csRegs_.hasEnterDebugModeTripped())
    {
      enterDebugMode_(DebugModeCause::TRIGGER, pc);
      enteredDebug = true;
    }
  else
    {
      initiateException(ExceptionCause::BREAKP, pc, info);
      if (dcsrStep_)
	{
	  enterDebugMode_(DebugModeCause::TRIGGER, pc_);
	  enteredDebug = true;
	}
    }

  if (beforeTiming and traceFile)
    {
      uint32_t inst = 0;
      readInst(currPc_, inst);

      std::string instStr;
      printInstTrace(inst, counter, instStr, traceFile);
    }

  return enteredDebug;
}


// True if keyboard interrupt (user hit control-c) pending.
static std::atomic<bool> userStop = false;

// Negation of the preceding variable. Exists for speed (obsessive
// compulsive engineering).
static std::atomic<bool> noUserStop = true;

void
forceUserStop(int)
{
  userStop = true;
  noUserStop = false;
}


static void
clearUserStop()
{
  userStop = false;
  noUserStop = true;
}


/// Install a signal handler for SIGINT (keyboard) interrupts on
/// construction. Restore to previous handlers on destruction. This
/// allows us to catch a control-c typed by the user in the middle of
/// a long-run and return to the top level interactive command
/// processor.
class SignalHandlers
{
public:

  SignalHandlers()
  {
    clearUserStop();

    struct sigaction newKbdAction;
    memset(&newKbdAction, 0, sizeof(newKbdAction));
    newKbdAction.sa_handler = forceUserStop;
    sigaction(SIGINT, &newKbdAction, &prevKbdAction_);
  }

  ~SignalHandlers()
  {
    sigaction(SIGINT, &prevKbdAction_, nullptr);
  }
  
private:

  struct sigaction prevKbdAction_;
};


template <typename URV>
inline
bool
Hart<URV>::fetchInstWithTrigger(URV addr, uint64_t& physAddr, uint32_t& inst,
				FILE* file)
{
  // Process pre-execute address trigger and fetch instruction.
  bool hasTrig = hasActiveInstTrigger();
  triggerTripped_ = (hasTrig and
                     instAddrTriggerHit(addr, TriggerTiming::Before,
                                        privMode_, isInterruptEnabled()));
  // Fetch instruction.
  bool fetchOk = true;
  if (triggerTripped_)
    {
      if (not fetchInstPostTrigger(addr, physAddr, inst, file))
        {
          ++cycleCount_;
          return false;  // Next instruction in trap handler.
        }
    }
  else
    {
      uint32_t ix = (addr >> 1) & decodeCacheMask_;
      DecodedInst* di = &decodeCache_[ix];
      if (not di->isValid() or di->address() != pc_)
        fetchOk = fetchInst(addr, physAddr, inst);
      else
        inst = di->inst();
    }
  if (not fetchOk)
    {
      ++cycleCount_;

      std::string instStr;
      printInstTrace(inst, instCounter_, instStr, file);

      if (dcsrStep_)
        enterDebugMode_(DebugModeCause::STEP, pc_);

      return false;  // Next instruction in trap handler.
    }

  // Process pre-execute opcode trigger.
  if (hasTrig and instOpcodeTriggerHit(inst, TriggerTiming::Before,
                                       privMode_,
                                       isInterruptEnabled()))
    triggerTripped_ = true;

  return true;
}


// We want amo instructions to print in the same order as executed.
static std::mutex execMutex;


template <typename URV>
bool
Hart<URV>::untilAddress(uint64_t address, FILE* traceFile)
{
  std::string instStr;
  instStr.reserve(128);

  uint64_t limit = instCountLim_;
  bool doStats = instFreq_ or enableCounters_;

  // Check for gdb break every 1000000 instructions.
  unsigned gdbCount = 0, gdbLimit = 1000000;

  if (enableGdb_)
    handleExceptionForGdb(*this, gdbInputFd_);

  while (pc_ != address and instCounter_ < limit)
    {
      if (userStop)
        break;
      resetExecInfo();

      if (enableGdb_ and ++gdbCount >= gdbLimit)
        {
          gdbCount = 0;
          if (hasPendingInput(gdbInputFd_))
            {
              handleExceptionForGdb(*this, gdbInputFd_);
              continue;
            }
        }

      if (preInst_)
        {
          bool halt = false, reset = false;
          while (true)
            {
              preInst_(*this, halt, reset);
              if (reset)
                {
                  this->reset();
                  return true;
                }
              if (not halt)
                break;
            }
        }

      try
	{
	  // This avoid interleaving of amo execution and tracing.
	  std::lock_guard<std::mutex> lock(execMutex);
      
          uint32_t inst = 0;
	  currPc_ = pc_;

	  ++instCounter_;

          if (processExternalInterrupt(traceFile, instStr))
	    continue;  // Next instruction in trap handler.
	  uint64_t physPc = 0;
          if (not fetchInstWithTrigger(pc_, physPc, inst, traceFile))
	    continue;  // Next instruction in trap handler.

	  // Decode unless match in decode cache.
	  uint32_t ix = (pc_ >> 1) & decodeCacheMask_;
	  DecodedInst* di = &decodeCache_[ix];
	  if (not di->isValid() or di->address() != pc_)
	    decode(pc_, physPc, inst, *di);

          // Increment pc and execute instruction
	  pc_ += di->instSize();
	  execute(di);

	  ++cycleCount_;

	  if (initStateFile_)
	    {
	      auto& instrPte = virtMem_.getInstrPteAddrs();
	      for (auto addr : instrPte)
		dumpInitState(initStateFile_, "ipt", initStateLines_, addr, addr, memory_);
	      auto& dataPte = virtMem_.getDataPteAddrs();
	      for (auto addr : dataPte)
		dumpInitState(initStateFile_, "dpt", initStateLines_, addr, addr, memory_);
	    }

	  if (hasException_ or hasInterrupt_)
	    {
              if (doStats)
                accumulateInstructionStats(*di);
	      printDecodedInstTrace(*di, instCounter_, instStr, traceFile);
	      continue;
	    }

	  if (triggerTripped_)
	    {
	      undoForTrigger();
	      if (takeTriggerAction(traceFile, currPc_, currPc_, instCounter_, true))
		return true;
	      continue;
	    }

          if (minstretEnabled())
            ++retiredInsts_;

	  if (bbFile_)
	    countBasicBlocks(di);

	  if (instrLineTrace_)
	    memory_.traceInstructionLine(currPc_);

	  if (doStats)
	    accumulateInstructionStats(*di);
	  printDecodedInstTrace(*di, instCounter_, instStr, traceFile);

	  bool icountHit = (enableTriggers_ and
			    icountTriggerHit(privMode_, isInterruptEnabled()));
	  if (icountHit)
	    if (takeTriggerAction(traceFile, pc_, pc_, instCounter_, false))
	      return true;
          prevPerfControl_ = perfControl_;
	}
      catch (const CoreException& ce)
	{
	  return logStop(ce, instCounter_, traceFile);
	}
    }

  return true;
}


template <typename URV>
bool
Hart<URV>::runUntilAddress(uint64_t address, FILE* traceFile)
{
  struct timeval t0;
  gettimeofday(&t0, nullptr);

  uint64_t limit = instCountLim_;
  uint64_t counter0 = instCounter_;

  // Setup signal handlers. Restore on destruction.
  SignalHandlers handlers;

  bool success = untilAddress(address, traceFile);
      
  if (instCounter_ == limit)
    std::cerr << "Stopped -- Reached instruction limit\n";
  else if (pc_ == address)
    std::cerr << "Stopped -- Reached end address\n";

  // Simulator stats.
  struct timeval t1;
  gettimeofday(&t1, nullptr);
  double elapsed = (double(t1.tv_sec - t0.tv_sec) +
		    double(t1.tv_usec - t0.tv_usec)*1e-6);

  uint64_t numInsts = instCounter_ - counter0;

  reportInstsPerSec(numInsts, elapsed, userStop);

  if (bbFile_)
    dumpBasicBlocks();

  return success;
}


template <typename URV>
bool
Hart<URV>::simpleRun()
{
  // For speed: do not record/clear CSR changes.
  enableCsrTrace_ = false;

  bool success = true;

  try
    {
      while (true)
        {
          bool hasLim = (instCountLim_ < ~uint64_t(0));
          if (hasLim or bbFile_ or instrLineTrace_ or isRvs() or hasClint())
            simpleRunWithLimit();
          else
            simpleRunNoLimit();

          if (userStop)
            {
              std::cerr << "Stopped -- interrupted\n";
              break;
            }

          if (hasLim)
            std::cerr << "Stopped -- Reached instruction limit\n";
          break;
        }
    }
  catch (const CoreException& ce)
    {
      success = logStop(ce, 0, nullptr);
    }

  enableCsrTrace_ = true;

  if (bbFile_)
    dumpBasicBlocks();

  return success;
}


template <typename URV>
void
Hart<URV>::dumpBasicBlocks()
{
  if (bbFile_)
    {
      bool first = true;
      for (const auto& kv : basicBlocks_)
	{
	  const BbStat& stat = kv.second;
	  if (stat.count_)
	    {
	      if (first)
		{
		  fprintf(bbFile_, "T");
		  first = false;
		}
	      fprintf(bbFile_, ":%ld:%ld:%ld:%ld ", kv.first, stat.count_,
		      stat.access_, stat.hit_);
	    }
	}
      if (not first)
	fprintf(bbFile_, "\n");
    }
  bbInsts_ = 0;

  // Clear basic block stats.
  for (auto& kv : basicBlocks_)
    {
      auto& stat = kv.second;
      stat.count_ = 0;
      stat.access_ = 0;
      stat.hit_ = 0;
    }
}


template <typename URV>
void
Hart<URV>::countBasicBlocks(const DecodedInst* di)
{
  if (bbInsts_ >= bbLimit_)
    dumpBasicBlocks();

  bbInsts_++;

  if (di->instEntry()->isBranch())
    {
      auto& blockStat = basicBlocks_[pc_];
      blockStat.count_++;

      if (memory_.cache_)
	{
	  uint64_t access = memory_.cache_->accessCount() - bbCacheAccess_;
	  uint64_t hit = memory_.cache_->hitCount() - bbCacheHit_;
	  basicBlocks_[bbPc_].access_ += access;
	  basicBlocks_[bbPc_].hit_ += hit;
	  bbCacheAccess_ = memory_.cache_->accessCount();
	  bbCacheHit_ = memory_.cache_->hitCount();
	}
      bbPc_ = pc_;
    }
  else
    {
      auto iter = basicBlocks_.find(pc_);
      if (iter != basicBlocks_.end())
	{
	  iter->second.count_++;
	  if (memory_.cache_)
	    {
	      uint64_t access = memory_.cache_->accessCount() - bbCacheAccess_;
	      uint64_t hit = memory_.cache_->hitCount() - bbCacheHit_;
	      basicBlocks_[bbPc_].access_ += access;
	      basicBlocks_[bbPc_].hit_ += hit;
	      bbCacheAccess_ = memory_.cache_->accessCount();
	      bbCacheHit_ = memory_.cache_->hitCount();
	    }
	  bbPc_ = pc_;
	}
      else
	basicBlocks_[bbPc_].count_++;
    }
}


template <typename URV>
bool
Hart<URV>::simpleRunWithLimit()
{
  uint64_t limit = instCountLim_;
  bool checkInterrupt = isRvs() or hasClint();
  std::string instStr;

  while (noUserStop and instCounter_ < limit) 
    {
      currPc_ = pc_;
      ++instCounter_;
	  cycleCount_++;

      if (checkInterrupt and processExternalInterrupt(nullptr, instStr))
	continue;

      // Fetch/decode unless match in decode cache.
      uint32_t ix = (pc_ >> 1) & decodeCacheMask_;
      DecodedInst* di = &decodeCache_[ix];
      if (not di->isValid() or di->address() != pc_)
        {
          uint32_t inst = 0;
	  uint64_t physPc = 0;
          if (not fetchInst(pc_, physPc, inst))
            continue;
          decode(pc_, physPc, inst, *di);
        }

      pc_ += di->instSize();
      execute(di);

      if (instrLineTrace_)
	memory_.traceInstructionLine(currPc_);

      if (bbFile_)
	countBasicBlocks(di);
    }

  return true;
}


template <typename URV>
bool
Hart<URV>::simpleRunNoLimit()
{
  while (noUserStop) 
    {
      currPc_ = pc_;
      ++instCounter_;

      // Fetch/decode unless match in decode cache.
      uint32_t ix = (pc_ >> 1) & decodeCacheMask_;
      DecodedInst* di = &decodeCache_[ix];
      if (not di->isValid() or di->address() != pc_)
        {
          uint32_t inst = 0;
	  uint64_t physPc = 0;
          if (not fetchInst(pc_, physPc, inst))
            continue;
          decode(pc_, physPc, inst, *di);
        }

      pc_ += di->instSize();
      execute(di);
    }

  return true;
}


template <typename URV>
bool
Hart<URV>::openTcpForGdb()
{
  struct sockaddr_in address;
  socklen_t addrlen = sizeof(address);

  memset(&address, 0, addrlen);

  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons( gdbTcpPort_ );

  int gdbFd = socket(AF_INET, SOCK_STREAM, 0);
  if (gdbFd < 0)
    {
      std::cerr << "Failed to create gdb socket at port " << gdbTcpPort_ << '\n';
      return false;
    }

#ifndef __APPLE__
  int opt = 1;
  if (setsockopt(gdbFd, SOL_SOCKET,
		 SO_REUSEADDR | SO_REUSEPORT, &opt,
		 sizeof(opt)) != 0)
    {
      std::cerr << "Failed to set socket option for gdb socket\n";
      return false;
    }
#endif

  if (bind(gdbFd, reinterpret_cast<sockaddr*>(&address), addrlen) < 0)
    {
      std::cerr << "Failed to bind gdb socket\n";
      return false;
    }

  if (listen(gdbFd, 3) < 0)
    {
      std::cerr << "Failed to listen to gdb socket\n";
      return false;
    }

  gdbInputFd_ = accept(gdbFd, (sockaddr*) &address, &addrlen);
  if (gdbInputFd_ < 0)
    {
      std::cerr << "Failed to accept from gdb socket\n";
      return false;
    }

  return true;
}


extern void (*tracerExtension)(void*);


/// Run indefinitely.  If the tohost address is defined, then run till
/// a write is attempted to that address.
template <typename URV>
bool
Hart<URV>::run(FILE* file)
{
  if (gdbTcpPort_ >= 0)
    openTcpForGdb();
  else if (enableGdb_)
    gdbInputFd_ = STDIN_FILENO;

  // To run fast, this method does not do much besides
  // straight-forward execution. If any option is turned on, we switch
  // to runUntilAddress which supports all features.
  URV stopAddr = stopAddrValid_? stopAddr_ : ~URV(0); // ~URV(0): No-stop PC.
  bool complex = (stopAddrValid_ or instFreq_ or enableTriggers_ or enableGdb_
                  or enableCounters_ or alarmInterval_ or file
		  or tracerExtension or hasInterruptor_ or initStateFile_);
  if (complex)
    return runUntilAddress(stopAddr, file); 

  uint64_t counter0 = instCounter_;

  struct timeval t0;
  gettimeofday(&t0, nullptr);

  // Setup signal handlers. Restore on destruction.
  SignalHandlers handlers;

  bool success = simpleRun();

  // Simulator stats.
  struct timeval t1;
  gettimeofday(&t1, nullptr);
  double elapsed = (double(t1.tv_sec - t0.tv_sec) +
		    double(t1.tv_usec - t0.tv_usec)*1e-6);

  uint64_t numInsts = instCounter_ - counter0;
  reportInstsPerSec(numInsts, elapsed, userStop);
  return success;
}


template <typename URV>
bool
Hart<URV>::isInterruptPossible(InterruptCause& cause)
{
  if (debugMode_ and not debugStepMode_)
    return false;

  URV mip = csRegs_.peekMip();
  URV mie = csRegs_.peekMie();
  if ((mie & mip) == 0)
    return false;  // Nothing enabled that is also pending.

  typedef InterruptCause IC;

  // Check for machine-level interrupts if MIE enabled or if user/supervisor.
  URV mstatus = csRegs_.peekMstatus();
  MstatusFields<URV> fields(mstatus);
  bool globalEnable = fields.bits_.MIE or privMode_ < PrivilegeMode::Machine;
  URV delegVal = 0;
  peekCsr(CsrNumber::MIDELEG, delegVal);
  for (InterruptCause ic : { IC::M_EXTERNAL, IC::M_LOCAL, IC::M_SOFTWARE,
                              IC::M_TIMER, IC::M_INT_TIMER0,
                              IC::M_INT_TIMER1 } )
    {
      URV mask = URV(1) << unsigned(ic);
      bool delegated = (mask & delegVal) != 0;
      bool enabled = globalEnable;
      if (delegated)
        enabled = ((privMode_ == PrivilegeMode::Supervisor and fields.bits_.SIE) or
                   privMode_ < PrivilegeMode::Supervisor);
      if (enabled)
        if (mie & mask & mip)
          {
            cause = ic;
            return true;
          }
    }

  // Supervisor mode interrupts: SIE enabled and supervior mode, or user-mode.
  if (isRvs())
    {
      bool check = ((fields.bits_.SIE and privMode_ == PrivilegeMode::Supervisor)
                    or privMode_ < PrivilegeMode::Supervisor);
      if (check)
        for (auto ic : { IC::S_EXTERNAL, IC::S_SOFTWARE, IC::S_TIMER } )
          {
            URV mask = URV(1) << unsigned(ic);
            if (mie & mask & mip)
              {
                cause = ic;
                return true;
              }
          }
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::processExternalInterrupt(FILE* traceFile, std::string& instStr)
{
  // If mip poked exernally we avoid over-writing it for 1 instruction.
  if (not mipPoked_)
    {
      URV mipVal = csRegs_.peekMip();
      URV prev = mipVal;

      for (auto dev : memory_.ioDevs_)
	if (dev->isInterruptPending())
	  {
	    mipVal |=  (URV(1) << URV(InterruptCause::M_EXTERNAL)) | (URV(1) << URV(InterruptCause::S_EXTERNAL));
	    dev->setInterruptPending(false);
	  }

      if (hasClint())
	{
	  // TODO: We should issue S_TIMER, M_TIMER or both based on configuration.
	  if ((instCounter_ >> counterToTimeShift) >= clintAlarm_)
	    mipVal = mipVal | (URV(1) << URV(InterruptCause::M_TIMER)) | (URV(1) << URV(InterruptCause::S_TIMER));
	  else
	    mipVal = mipVal & ~(URV(1) << URV(InterruptCause::M_TIMER)) & ~(URV(1) << URV(InterruptCause::S_TIMER));
	}

      bool hasAlarm = alarmLimit_ != ~uint64_t(0);
      if (hasAlarm)
	{
	  if (instCounter_ >= alarmLimit_)
	    {
	      alarmLimit_ += alarmInterval_;
	      mipVal = mipVal | (URV(1) << URV(InterruptCause::M_TIMER));
	    }
	  else
	    mipVal = mipVal & ~(URV(1) << URV(InterruptCause::M_TIMER));
	}

      if (mipVal != prev)
	csRegs_.poke(CsrNumber::MIP, mipVal);
    }
  mipPoked_ = false;

  if (debugStepMode_ and not dcsrStepIe_)
    return false;

  // If a non-maskable interrupt was signaled by the test-bench, take it.
  if (nmiPending_)
    {
      initiateNmi(URV(nmiCause_), pc_);
      nmiPending_ = false;
      nmiCause_ = NmiCause::UNKNOWN;
      uint32_t inst = 0; // Load interrupted inst.
      readInst(currPc_, inst);
      printInstTrace(inst, instCounter_, instStr, traceFile);
      return true;
    }

  // If interrupts enabled and one is pending, take it.
  InterruptCause cause;
  if (isInterruptPossible(cause))
    {
      // Attach changes to interrupted instruction.
      initiateInterrupt(cause, pc_);
      uint32_t inst = 0; // Load interrupted inst.
      readInst(currPc_, inst);
      printInstTrace(inst, instCounter_, instStr, traceFile);
      ++cycleCount_;
      return true;
    }
  return false;
}


template <typename URV>
void
Hart<URV>::invalidateDecodeCache(URV addr, unsigned storeSize)
{
  // Consider putting this in a callback associated with memory
  // write/poke. This way it can be applied only to pages marked
  // execute.

  // We want to check the location before the address just in case it
  // contains a 4-byte instruction that overlaps what was written.
  storeSize += 3;
  addr -= 3;

  for (unsigned i = 0; i < storeSize; i += 2)
    {
      URV instAddr = (addr + i) >> 1;
      uint32_t cacheIx = instAddr & decodeCacheMask_;
      auto& entry = decodeCache_[cacheIx];
      if ((entry.address() >> 1) == instAddr)
	entry.invalidate();
    }
}


template <typename URV>
void
Hart<URV>::invalidateDecodeCache()
{
  for (auto& entry : decodeCache_)
    entry.invalidate();
}


template <typename URV>
void
Hart<URV>::singleStep(FILE* traceFile)
{
  DecodedInst di;
  singleStep(di, traceFile);
}


template <typename URV>
void
Hart<URV>::singleStep(DecodedInst& di, FILE* traceFile)
{
  std::string instStr;

  // Single step is mostly used for follow-me mode where we want to
  // know the changes after the execution of each instruction.
  bool doStats = instFreq_ or enableCounters_;

  try
    {
      uint32_t inst = 0;
      currPc_ = pc_;

      resetExecInfo();

      ++instCounter_;

      if (processExternalInterrupt(traceFile, instStr))
	return;  // Next instruction in interrupt handler.

      uint64_t physPc = 0;
      if (not fetchInstWithTrigger(pc_, physPc, inst, traceFile))
        return;

      decode(pc_, physPc, inst, di);

      // Increment pc and execute instruction
      pc_ += di.instSize();
      execute(&di);

      ++cycleCount_;

      if (hasException_ or hasInterrupt_)
	{
	  if (doStats)
	    accumulateInstructionStats(di);
	  printDecodedInstTrace(di, instCounter_, instStr, traceFile);
	  if (dcsrStep_ and not ebreakInstDebug_)
	    enterDebugMode_(DebugModeCause::STEP, pc_);
	  return;
	}

      if (triggerTripped_)
	{
	  undoForTrigger();
	  takeTriggerAction(traceFile, currPc_, currPc_, instCounter_, true);
	  return;
	}

      if (minstretEnabled() and not ebreakInstDebug_)
        ++retiredInsts_;

      if (doStats)
	accumulateInstructionStats(di);
      printInstTrace(inst, instCounter_, instStr, traceFile);

      bool icountHit = (enableTriggers_ and 
			icountTriggerHit(privMode_, isInterruptEnabled()));
      if (icountHit)
	{
	  takeTriggerAction(traceFile, pc_, pc_, instCounter_, false);
	  return;
	}

      // If step bit set in dcsr then enter debug mode unless already there.
      if (dcsrStep_ and not ebreakInstDebug_)
	enterDebugMode_(DebugModeCause::STEP, pc_);

      prevPerfControl_ = perfControl_;
    }
  catch (const CoreException& ce)
    {
      // If step bit set in dcsr then enter debug mode unless already there.
      // This is for the benefit of the test bench.
      if (dcsrStep_ and not ebreakInstDebug_)
	enterDebugMode_(DebugModeCause::STEP, pc_);

      logStop(ce, instCounter_, traceFile);
    }
}


template <typename URV>
bool
Hart<URV>::whatIfSingleStep(uint32_t inst, ChangeRecord& record)
{
  uint64_t prevExceptionCount = exceptionCount_;
  URV prevPc = pc_;

  clearTraceData();
  triggerTripped_ = false;

  // Note: triggers not yet supported.

  DecodedInst di;
  uint64_t physPc = pc_;
  decode(pc_, physPc, inst, di);

  // Execute instruction
  pc_ += di.instSize();
  execute(&di);

  bool result = exceptionCount_ == prevExceptionCount;

  // If step bit set in dcsr then enter debug mode unless already there.
  if (dcsrStep_ and not ebreakInstDebug_)
    enterDebugMode_(DebugModeCause::STEP, pc_);

  // Collect changes. Undo each collected change.
  exceptionCount_ = prevExceptionCount;

  collectAndUndoWhatIfChanges(prevPc, record);
  
  return result;
}


template <typename URV>
bool
Hart<URV>::whatIfSingleStep(URV whatIfPc, uint32_t inst, ChangeRecord& record)
{
  URV prevPc = pc_;
  setPc(whatIfPc);

  // Note: triggers not yet supported.
  triggerTripped_ = false;

  // Fetch instruction. We don't care about what we fetch. Just checking
  // if there is a fetch exception.
  uint32_t tempInst = 0;
  uint64_t physPc = 0;
  bool fetchOk = fetchInst(pc_, physPc, tempInst);

  if (not fetchOk)
    {
      collectAndUndoWhatIfChanges(prevPc, record);
      return false;
    }

  bool res = whatIfSingleStep(inst, record);

  setPc(prevPc);
  return res;
}


template <typename URV>
bool
Hart<URV>::whatIfSingStep(const DecodedInst& di, ChangeRecord& record)
{
  clearTraceData();
  uint64_t prevExceptionCount = exceptionCount_;
  URV prevPc  = pc_, prevCurrPc = currPc_;

  setPc(di.address());
  currPc_ = pc_;

  // Note: triggers not yet supported.
  triggerTripped_ = false;

  // Save current value of operands.
  uint64_t prevRegValues[4];
  for (unsigned i = 0; i < 4; ++i)
    {
      URV prev = 0;
      prevRegValues[i] = 0;
      uint32_t operand = di.ithOperand(i);

      switch (di.ithOperandType(i))
	{
	case OperandType::None:
	case OperandType::Imm:
	  break;
	case OperandType::IntReg:
	  peekIntReg(operand, prev);
	  prevRegValues[i] = prev;
	  break;
	case OperandType::FpReg:
	  peekFpReg(operand, prevRegValues[i]);
	  break;
	case OperandType::CsReg:
	  peekCsr(CsrNumber(operand), prev);
	  prevRegValues[i] = prev;
	  break;
        case OperandType::VecReg:
          assert(0);
          break;
	}
    }

  // Temporarily set value of operands to what-if values.
  for (unsigned i = 0; i < 4; ++i)
    {
      uint32_t operand = di.ithOperand(i);

      switch (di.ithOperandType(i))
	{
	case OperandType::None:
	case OperandType::Imm:
	  break;
	case OperandType::IntReg:
	  pokeIntReg(operand, di.ithOperandValue(i));
	  break;
	case OperandType::FpReg:
	  pokeFpReg(operand, di.ithOperandValue(i));
	  break;
	case OperandType::CsReg:
	  pokeCsr(CsrNumber(operand), di.ithOperandValue(i));
	  break;
        case OperandType::VecReg:
          assert(0);
          break;
	}
    }

  // Execute instruction.
  pc_ += di.instSize();
  if (di.instEntry()->instId() != InstId::illegal)
    execute(&di);
  bool result = exceptionCount_ == prevExceptionCount;

  // Collect changes. Undo each collected change.
  exceptionCount_ = prevExceptionCount;
  collectAndUndoWhatIfChanges(prevPc, record);

  // Restore temporarily modified registers.
  for (unsigned i = 0; i < 4; ++i)
    {
      uint32_t operand = di.ithOperand(i);

      switch (di.ithOperandType(i))
	{
	case OperandType::None:
	case OperandType::Imm:
	  break;
	case OperandType::IntReg:
	  pokeIntReg(operand, prevRegValues[i]);
	  break;
	case OperandType::FpReg:
	  pokeFpReg(operand, prevRegValues[i]);
	  break;
	case OperandType::CsReg:
	  pokeCsr(CsrNumber(operand), prevRegValues[i]);
	  break;
        case OperandType::VecReg:
          assert(0);
          break;
	}
    }

  setPc(prevPc);
  currPc_ = prevCurrPc;

  return result;
}


template <typename URV>
void
Hart<URV>::collectAndUndoWhatIfChanges(URV prevPc, ChangeRecord& record)
{
  record.clear();

  record.newPc = pc_;
  setPc(prevPc);

  unsigned regIx = 0;
  URV oldValue = 0;
  if (intRegs_.getLastWrittenReg(regIx, oldValue))
    {
      URV newValue = 0;
      peekIntReg(regIx, newValue);
      pokeIntReg(regIx, oldValue);

      record.hasIntReg = true;
      record.intRegIx = regIx;
      record.intRegValue = newValue;
    }

  uint64_t oldFpValue = 0;
  if (fpRegs_.getLastWrittenReg(regIx, oldFpValue))
    {
      uint64_t newFpValue = 0;
      peekFpReg(regIx, newFpValue);
      pokeFpReg(regIx, oldFpValue);

      record.hasFpReg = true;
      record.fpRegIx = regIx;
      record.fpRegValue = newFpValue;
    }

  if (ldStWrite_)
    {
      record.memSize = ldStSize_;
      record.memAddr = ldStPhysAddr1_;
      record.memValue = ldStData_;
      uint64_t addr = ldStPhysAddr1_;
      uint64_t value = ldStPrevData_;
      for (size_t i = 0; i < ldStSize_; ++i, ++addr)
	{
	  memory_.poke(addr, uint8_t(value));
	  value = value >> 8;
	}
    }

  std::vector<CsrNumber> csrNums;
  std::vector<unsigned> triggerNums;
  csRegs_.getLastWrittenRegs(csrNums, triggerNums);

  for (auto csrn : csrNums)
    {
      Csr<URV>* csr = csRegs_.getImplementedCsr(csrn);
      if (not csr)
	continue;

      URV newVal = csr->read();
      URV oldVal = csr->prevValue();
      csr->write(oldVal);

      record.csrIx.push_back(csrn);
      record.csrValue.push_back(newVal);
    }

  clearTraceData();
}


template <typename URV>
inline
void
Hart<URV>::execLui(const DecodedInst* di)
{
  intRegs_.write(di->op0(), SRV(int32_t(di->op1())));
}


template <typename URV>
void
Hart<URV>::execute(const DecodedInst* di)
{
#pragma GCC diagnostic ignored "-Wpedantic"

  // Setup an array of labels to index it by the decoded instruction
  // id to jump to execute function of that instruction. A table of
  // methods would be a lot cleaner (no goto) but it would cost 20%
  // more in execution time.
  static void* labels[] =
    {
     &&illegal,
     &&lui,
     &&auipc,
     &&jal,
     &&jalr,
     &&beq,
     &&bne,
     &&blt,
     &&bge,
     &&bltu,
     &&bgeu,
     &&lb,
     &&lh,
     &&lw,
     &&lbu,
     &&lhu,
     &&sb,
     &&sh,
     &&sw,
     &&addi,
     &&slti,
     &&sltiu,
     &&xori,
     &&ori,
     &&andi,
     &&slli,
     &&srli,
     &&srai,
     &&add,
     &&sub,
     &&sll,
     &&slt,
     &&sltu,
     &&xor_,
     &&srl,
     &&sra,
     &&or_,
     &&and_,
     &&fence,
     &&fence_tso,
     &&fencei,
     &&ecall,
     &&ebreak,
     &&csrrw,
     &&csrrs,
     &&csrrc,
     &&csrrwi,
     &&csrrsi,
     &&csrrci,
     &&lwu,
     &&ld,
     &&sd,
     &&addiw,
     &&slliw,
     &&srliw,
     &&sraiw,
     &&addw,
     &&subw,
     &&sllw,
     &&srlw,
     &&sraw,
     &&mul,
     &&mulh,
     &&mulhsu,
     &&mulhu,
     &&div,
     &&divu,
     &&rem,
     &&remu,
     &&mulw,
     &&divw,
     &&divuw,
     &&remw,
     &&remuw,
     &&lr_w,
     &&sc_w,
     &&amoswap_w,
     &&amoadd_w,
     &&amoxor_w,
     &&amoand_w,
     &&amoor_w,
     &&amomin_w,
     &&amomax_w,
     &&amominu_w,
     &&amomaxu_w,
     &&lr_d,
     &&sc_d,
     &&amoswap_d,
     &&amoadd_d,
     &&amoxor_d,
     &&amoand_d,
     &&amoor_d,
     &&amomin_d,
     &&amomax_d,
     &&amominu_d,
     &&amomaxu_d,
     &&flw,
     &&fsw,
     &&fmadd_s,
     &&fmsub_s,
     &&fnmsub_s,
     &&fnmadd_s,
     &&fadd_s,
     &&fsub_s,
     &&fmul_s,
     &&fdiv_s,
     &&fsqrt_s,
     &&fsgnj_s,
     &&fsgnjn_s,
     &&fsgnjx_s,
     &&fmin_s,
     &&fmax_s,
     &&fcvt_w_s,
     &&fcvt_wu_s,
     &&fmv_x_w,
     &&feq_s,
     &&flt_s,
     &&fle_s,
     &&fclass_s,
     &&fcvt_s_w,
     &&fcvt_s_wu,
     &&fmv_w_x,
     &&fcvt_l_s,
     &&fcvt_lu_s,
     &&fcvt_s_l,
     &&fcvt_s_lu,
     &&fld,
     &&fsd,
     &&fmadd_d,
     &&fmsub_d,
     &&fnmsub_d,
     &&fnmadd_d,
     &&fadd_d,
     &&fsub_d,
     &&fmul_d,
     &&fdiv_d,
     &&fsqrt_d,
     &&fsgnj_d,
     &&fsgnjn_d,
     &&fsgnjx_d,
     &&fmin_d,
     &&fmax_d,
     &&fcvt_s_d,
     &&fcvt_d_s,
     &&feq_d,
     &&flt_d,
     &&fle_d,
     &&fclass_d,
     &&fcvt_w_d,
     &&fcvt_wu_d,
     &&fcvt_d_w,
     &&fcvt_d_wu,
     &&fcvt_l_d,
     &&fcvt_lu_d,
     &&fmv_x_d,
     &&fcvt_d_l,
     &&fcvt_d_lu,
     &&fmv_d_x,
     &&flh,
     &&fsh,
     &&fmadd_h,
     &&fmsub_h,
     &&fnmsub_h,
     &&fnmadd_h,
     &&fadd_h,
     &&fsub_h,
     &&fmul_h,
     &&fdiv_h,
     &&fsqrt_h,
     &&fsgnj_h,
     &&fsgnjn_h,
     &&fsgnjx_h,
     &&fmin_h,
     &&fmax_h,
     &&fcvt_s_h,
     &&fcvt_d_h,
     &&fcvt_h_s,
     &&fcvt_h_d,
     &&fcvt_w_h,
     &&fcvt_wu_h,
     &&fmv_x_h,
     &&feq_h,
     &&flt_h,
     &&fle_h,
     &&fclass_h,
     &&fcvt_h_w,
     &&fcvt_h_wu,
     &&fmv_h_x,
     &&fcvt_l_h,
     &&fcvt_lu_h,
     &&fcvt_h_l,
     &&fcvt_h_lu,
     &&mret,
     &&uret,
     &&sret,
     &&wfi,
     &&dret,
     &&sfence_vma,
     &&c_addi4spn,
     &&c_fld,
     &&c_lq,
     &&c_lw,
     &&c_flw,
     &&c_ld,
     &&c_fsd,
     &&c_sq,
     &&c_sw,
     &&c_fsw,
     &&c_sd,
     &&c_addi,
     &&c_jal,
     &&c_li,
     &&c_addi16sp,
     &&c_lui,
     &&c_srli,
     &&c_srli64,
     &&c_srai,
     &&c_srai64,
     &&c_andi,
     &&c_sub,
     &&c_xor,
     &&c_or,
     &&c_and,
     &&c_subw,
     &&c_addw,
     &&c_j,
     &&c_beqz,
     &&c_bnez,
     &&c_slli,
     &&c_slli64,
     &&c_fldsp,
     &&c_lwsp,
     &&c_flwsp,
     &&c_ldsp,
     &&c_jr,
     &&c_mv,
     &&c_ebreak,
     &&c_jalr,
     &&c_add,
     &&c_fsdsp,
     &&c_swsp,
     &&c_fswsp,
     &&c_addiw,
     &&c_sdsp,

     &&clz,
     &&ctz,
     &&cpop,
     &&clzw,
     &&ctzw,
     &&cpopw,
     &&min,
     &&max,
     &&minu,
     &&maxu,
     &&sext_b,
     &&sext_h,
     &&andn,
     &&orn,
     &&xnor,
     &&rol,
     &&ror,
     &&rori,
     &&rolw,
     &&rorw,
     &&roriw,
     &&rev8,
     &&pack,
     &&packh,
     &&packu,
     &&packw,
     &&packuw,
     &&grev,
     &&grevi,
     &&grevw,
     &&greviw,
     &&gorc,
     &&gorci,
     &&gorcw,
     &&gorciw,
     &&shfl,
     &&shflw,
     &&shfli,
     &&unshfl,
     &&unshfli,
     &&unshflw,
     &&xperm_n,
     &&xperm_b,
     &&xperm_h,
     &&xperm_w,

     // zbs
     &&bset,
     &&bclr,
     &&binv,
     &&bext,
     &&bseti,
     &&bclri,
     &&binvi,
     &&bexti,

     // zbe
     &&bcompress,
     &&bdecompress,
     &&bcompressw,
     &&bdecompressw,

     // zbf
     &&bfp,
     &&bfpw,

     // zbc
     &&clmul,
     &&clmulh,
     &&clmulr,

     // zba
     &&sh1add,
     &&sh2add,
     &&sh3add,
     &&sh1add_uw,
     &&sh2add_uw,
     &&sh3add_uw,
     &&add_uw,
     &&slli_uw,

     // zbr
     &&crc32_b,
     &&crc32_h,
     &&crc32_w,
     &&crc32_d,
     &&crc32c_b,
     &&crc32c_h,
     &&crc32c_w,
     &&crc32c_d,

     // zbm
     &&bmator,
     &&bmatxor,
     &&bmatflip,

     // zbt
     &&cmov,
     &&cmix,
     &&fsl,
     &&fsr,
     &&fsri,
     &&fslw,
     &&fsrw,
     &&fsriw,

     // vevtor
     &&vsetvli,
     &&vsetivli,
     &&vsetvl,
     &&vadd_vv,
     &&vadd_vx,
     &&vadd_vi,
     &&vsub_vv,
     &&vsub_vx,
     &&vrsub_vx,
     &&vrsub_vi,
     &&vwaddu_vv,
     &&vwaddu_vx,
     &&vwsubu_vv,
     &&vwsubu_vx,
     &&vwadd_vv,
     &&vwadd_vx,
     &&vwsub_vv,
     &&vwsub_vx,
     &&vwaddu_wv,
     &&vwaddu_wx,
     &&vwsubu_wv,
     &&vwsubu_wx,
     &&vwadd_wv,
     &&vwadd_wx,
     &&vwsub_wv,
     &&vwsub_wx,

     &&vmseq_vv,
     &&vmseq_vx,
     &&vmseq_vi,
     &&vmsne_vv,
     &&vmsne_vx,
     &&vmsne_vi,
     &&vmsltu_vv,
     &&vmsltu_vx,
     &&vmslt_vv,
     &&vmslt_vx,
     &&vmsleu_vv,
     &&vmsleu_vx,
     &&vmsleu_vi,
     &&vmsle_vv,
     &&vmsle_vx,
     &&vmsle_vi,
     &&vmsgtu_vx,
     &&vmsgtu_vi,
     &&vmsgt_vx,
     &&vmsgt_vi,

     &&vminu_vv,
     &&vminu_vx,
     &&vmin_vv,
     &&vmin_vx,
     &&vmaxu_vv,
     &&vmaxu_vx,
     &&vmax_vv,
     &&vmax_vx,
     &&vand_vv,
     &&vand_vx,
     &&vand_vi,
     &&vor_vv,
     &&vor_vx,
     &&vor_vi,
     &&vxor_vv,
     &&vxor_vx,
     &&vxor_vi,
     &&vsll_vv,
     &&vsll_vx,
     &&vsll_vi,
     &&vsrl_vv,
     &&vsrl_vx,
     &&vsrl_vi,
     &&vsra_vv,
     &&vsra_vx,
     &&vsra_vi,
     &&vnsrl_wv,
     &&vnsrl_wx,
     &&vnsrl_wi,
     &&vnsra_wv,
     &&vnsra_wx,
     &&vnsra_wi,
     &&vrgather_vv,
     &&vrgather_vx,
     &&vrgather_vi,
     &&vrgatherei16_vv,
     &&vcompress_vm,
     &&vredsum_vs,
     &&vredand_vs,
     &&vredor_vs,
     &&vredxor_vs,
     &&vredminu_vs,
     &&vredmin_vs,
     &&vredmaxu_vs,
     &&vredmax_vs,
     &&vwredsumu_vs,
     &&vwredsum_vs,

     &&vmand_mm,
     &&vmnand_mm,
     &&vmandnot_mm,
     &&vmxor_mm,
     &&vmor_mm,
     &&vmnor_mm,
     &&vmornot_mm,
     &&vmxnor_mm,
     &&vpopc_m,
     &&vfirst_m,
     &&vmsbf_m,
     &&vmsif_m,
     &&vmsof_m,
     &&viota_m,
     &&vid_v,
     &&vslideup_vx,
     &&vslideup_vi,
     &&vslide1up_vx,
     &&vslidedown_vx,
     &&vslidedown_vi,
     &&vslide1down_vx,
     &&vfslide1up_vf,
     &&vfslide1down_vf,
     &&vmul_vv,
     &&vmul_vx,
     &&vmulh_vv,
     &&vmulh_vx,
     &&vmulhu_vv,
     &&vmulhu_vx,
     &&vmulhsu_vv,
     &&vmulhsu_vx,
     &&vmadd_vv,
     &&vmadd_vx,
     &&vnmsub_vv,
     &&vnmsub_vx,
     &&vmacc_vv,
     &&vmacc_vx,
     &&vnmsac_vv,
     &&vnmsac_vx,
     &&vwmulu_vv,
     &&vwmulu_vx,
     &&vwmul_vv,
     &&vwmul_vx,
     &&vwmulsu_vv,
     &&vwmulsu_vx,
     &&vwmaccu_vv,
     &&vwmaccu_vx,
     &&vwmacc_vv,
     &&vwmacc_vx,
     &&vwmaccsu_vv,
     &&vwmaccsu_vx,
     &&vwmaccus_vx,
     &&vdivu_vv,
     &&vdivu_vx,
     &&vdiv_vv,
     &&vdiv_vx,
     &&vremu_vv,
     &&vremu_vx,
     &&vrem_vv,
     &&vrem_vx,
     &&vsext_vf2,
     &&vsext_vf4,
     &&vsext_vf8,
     &&vzext_vf2,
     &&vzext_vf4,
     &&vzext_vf8,
     &&vadc_vvm,
     &&vadc_vxm,
     &&vadc_vim,
     &&vsbc_vvm,
     &&vsbc_vxm,
     &&vmadc_vvm,
     &&vmadc_vxm,
     &&vmadc_vim,
     &&vmsbc_vvm,
     &&vmsbc_vxm,
     &&vmerge_vvm,
     &&vmerge_vxm,
     &&vmerge_vim,
     &&vmv_x_s,
     &&vmv_s_x,
     &&vfmv_f_s,
     &&vfmv_s_f,
     &&vmv_v_v,
     &&vmv_v_x,
     &&vmv_v_i,
     &&vmv1r_v,
     &&vmv2r_v,
     &&vmv4r_v,
     &&vmv8r_v,
     &&vsaddu_vv,
     &&vsaddu_vx,
     &&vsaddu_vi,
     &&vsadd_vv,
     &&vsadd_vx,
     &&vsadd_vi,
     &&vssubu_vv,
     &&vssubu_vx,
     &&vssub_vv,
     &&vssub_vx,
     &&vaaddu_vv,
     &&vaaddu_vx,
     &&vaadd_vv,
     &&vaadd_vx,
     &&vasubu_vv,
     &&vasubu_vx,
     &&vasub_vv,
     &&vasub_vx,
     &&vsmul_vv,
     &&vsmul_vx,
     &&vssrl_vv,
     &&vssrl_vx,
     &&vssrl_vi,
     &&vssra_vv,
     &&vssra_vx,
     &&vssra_vi,
     &&vnclipu_wv,
     &&vnclipu_wx,
     &&vnclipu_wi,
     &&vnclip_wv,
     &&vnclip_wx,
     &&vnclip_wi,

     &&vle8_v,
     &&vle16_v,
     &&vle32_v,
     &&vle64_v,
     &&vle128_v,
     &&vle256_v,
     &&vle512_v,
     &&vle1024_v,
     &&vse8_v,
     &&vse16_v,
     &&vse32_v,
     &&vse64_v,
     &&vse128_v,
     &&vse256_v,
     &&vse512_v,
     &&vse1024_v,

     &&vlm_v,
     &&vsm_v,

     &&vlre8_v,
     &&vlre16_v,
     &&vlre32_v,
     &&vlre64_v,
     &&vlre128_v,
     &&vlre256_v,
     &&vlre512_v,
     &&vlre1024_v,
     &&vs1r_v,
     &&vs2r_v,
     &&vs4r_v,
     &&vs8r_v,

     &&vle8ff_v,
     &&vle16ff_v,
     &&vle32ff_v,
     &&vle64ff_v,
     &&vle128ff_v,
     &&vle256ff_v,
     &&vle512ff_v,
     &&vle1024ff_v,

     &&vlse8_v,
     &&vlse16_v,
     &&vlse32_v,
     &&vlse64_v,
     &&vlse128_v,
     &&vlse256_v,
     &&vlse512_v,
     &&vlse1024_v,
     &&vsse8_v,
     &&vsse16_v,
     &&vsse32_v,
     &&vsse64_v,
     &&vsse128_v,
     &&vsse256_v,
     &&vsse512_v,
     &&vsse1024_v,

     &&vloxei8_v,
     &&vloxei16_v,
     &&vloxei32_v,
     &&vloxei64_v,
     &&vluxei8_v,
     &&vluxei16_v,
     &&vluxei32_v,
     &&vluxei64_v,
     &&vsoxei8_v,
     &&vsoxei16_v,
     &&vsoxei32_v,
     &&vsoxei64_v,
     &&vsuxei8_v,
     &&vsuxei16_v,
     &&vsuxei32_v,
     &&vsuxei64_v,

     &&vlsege8_v,
     &&vlsege16_v,
     &&vlsege32_v,
     &&vlsege64_v,
     &&vlsege128_v,
     &&vlsege256_v,
     &&vlsege512_v,
     &&vlsege1024_v,
     &&vssege8_v,
     &&vssege16_v,
     &&vssege32_v,
     &&vssege64_v,
     &&vssege128_v,
     &&vssege256_v,
     &&vssege512_v,
     &&vssege1024_v,

     &&vlssege8_v,
     &&vlssege16_v,
     &&vlssege32_v,
     &&vlssege64_v,
     &&vlssege128_v,
     &&vlssege256_v,
     &&vlssege512_v,
     &&vlssege1024_v,
     &&vsssege8_v,
     &&vsssege16_v,
     &&vsssege32_v,
     &&vsssege64_v,
     &&vsssege128_v,
     &&vsssege256_v,
     &&vsssege512_v,
     &&vsssege1024_v,

     &&vluxsegei8_v,
     &&vluxsegei16_v,
     &&vluxsegei32_v,
     &&vluxsegei64_v,
     &&vluxsegei128_v,
     &&vluxsegei256_v,
     &&vluxsegei512_v,
     &&vluxsegei1024_v,
     &&vsuxsegei8_v,
     &&vsuxsegei16_v,
     &&vsuxsegei32_v,
     &&vsuxsegei64_v,
     &&vsuxsegei128_v,
     &&vsuxsegei256_v,
     &&vsuxsegei512_v,
     &&vsuxsegei1024_v,

     &&vloxsegei8_v,
     &&vloxsegei16_v,
     &&vloxsegei32_v,
     &&vloxsegei64_v,
     &&vloxsegei128_v,
     &&vloxsegei256_v,
     &&vloxsegei512_v,
     &&vloxsegei1024_v,
     &&vsoxsegei8_v,
     &&vsoxsegei16_v,
     &&vsoxsegei32_v,
     &&vsoxsegei64_v,
     &&vsoxsegei128_v,
     &&vsoxsegei256_v,
     &&vsoxsegei512_v,
     &&vsoxsegei1024_v,

     &&vlsege8ff_v,
     &&vlsege16ff_v,
     &&vlsege32ff_v,
     &&vlsege64ff_v,
     &&vlsege128ff_v,
     &&vlsege256ff_v,
     &&vlsege512ff_v,
     &&vlsege1024ff_v,

     &&vfadd_vv,
     &&vfadd_vf,
     &&vfsub_vv,
     &&vfsub_vf,
     &&vfrsub_vf,

     &&vfwadd_vv,
     &&vfwadd_vf,
     &&vfwsub_vv,
     &&vfwsub_vf,
     &&vfwadd_wv,
     &&vfwadd_wf,
     &&vfwsub_wv,
     &&vfwsub_wf,

     &&vfmul_vv,
     &&vfmul_vf,
     &&vfdiv_vv,
     &&vfdiv_vf,
     &&vfrdiv_vf,
     &&vfwmul_vv,
     &&vfwmul_vf,

     &&vfmadd_vv,
     &&vfmadd_vf,
     &&vfnmadd_vv,
     &&vfnmadd_vf,
     &&vfmsub_vv,
     &&vfmsub_vf,
     &&vfnmsub_vv,
     &&vfnmsub_vf,
     &&vfmacc_vv,
     &&vfmacc_vf,
     &&vfnmacc_vv,
     &&vfnmacc_vf,
     &&vfmsac_vv,
     &&vfmsac_vf,
     &&vfnmsac_vv,
     &&vfnmsac_vf,
     &&vfwmacc_vv,
     &&vfwmacc_vf,
     &&vfwnmacc_vv,
     &&vfwnmacc_vf,
     &&vfwmsac_vv,
     &&vfwmsac_vf,
     &&vfwnmsac_vv,
     &&vfwnmsac_vf,
     &&vfsqrt_v,
     &&vfmerge_vfm,
     &&vfmv_v_f,
     &&vmfeq_vv,
     &&vmfeq_vf,
     &&vmfne_vv,
     &&vmfne_vf,
     &&vmflt_vv,
     &&vmflt_vf,
     &&vmfle_vv,
     &&vmfle_vf,
     &&vmfgt_vf,
     &&vmfge_vf,
     &&vfclass_v,
     &&vfcvt_xu_f_v,
     &&vfcvt_x_f_v,
     &&vfcvt_rtz_xu_f_v,
     &&vfcvt_rtz_x_f_v,
     &&vfcvt_f_xu_v,
     &&vfcvt_f_x_v,

     &&vfwcvt_xu_f_v,
     &&vfwcvt_x_f_v,
     &&vfwcvt_rtz_xu_f_v,
     &&vfwcvt_rtz_x_f_v,
     &&vfwcvt_f_xu_v,
     &&vfwcvt_f_x_v,
     &&vfwcvt_f_f_v,

     &&vfncvt_xu_f_w,
     &&vfncvt_x_f_w,
     &&vfncvt_rtz_xu_f_w,
     &&vfncvt_rtz_x_f_w,
     &&vfncvt_f_xu_w,
     &&vfncvt_f_x_w,
     &&vfncvt_f_f_w,
     &&vfncvt_rod_f_f_w,
     &&vfredsum_vs,
     &&vfredosum_vs,
     &&vfredmin_vs,
     &&vfredmax_vs,
     &&vfwredsum_vs,
     &&vfwredosum_vs,
     &&vfrsqrt7_v,
     &&vfrec7_v,
     &&vfmin_vv,
     &&vfmin_vf,
     &&vfmax_vv,
     &&vfmax_vf,
     &&vfsgnj_vv,
     &&vfsgnj_vf,
     &&vfsgnjn_vv,
     &&vfsgnjn_vf,
     &&vfsgnjx_vv,
     &&vfsgnjx_vf,

     // zk
     &&aes32dsi,
     &&aes32dsmi,
     &&aes32esi,
     &&aes32esmi,
     &&aes64ds,
     &&aes64dsm,
     &&aes64es,
     &&aes64esm,
     &&aes64im,
     &&aes64ks1i,
     &&aes64ks2,
     &&sha256sig0,
     &&sha256sig1,
     &&sha256sum0,
     &&sha256sum1,
     &&sha512sig0h,
     &&sha512sig0l,
     &&sha512sig1h,
     &&sha512sig1l,
     &&sha512sum0r,
     &&sha512sum1r,
     &&sha512sig0,
     &&sha512sig1,
     &&sha512sum0,
     &&sha512sum1,
     &&sm3p0,
     &&sm3p1,
     &&sm4ed,
     &&sm4ks,

     &&sinval_vma,
     &&sfence_w_inval,
     &&sfence_inval_ir,

     &&cbo_clean,
     &&cbo_flush,
     &&cbo_inval,
     &&cbo_zero,

    };

  const InstEntry* entry = di->instEntry();
  size_t id = size_t(entry->instId());
  assert(id < sizeof(labels));
  goto *labels[id];

 illegal:
  illegalInst(di);
  return;

 lui:
  execLui(di);
  return;

 auipc:
  execAuipc(di);
  return;

 jal:
  execJal(di);
  return;

 jalr:
  execJalr(di);
  return;

 beq:
  execBeq(di);
  return;

 bne:
  execBne(di);
  return;

 blt:
  execBlt(di);
  return;

 bge:
  execBge(di);
  return;

 bltu:
  execBltu(di);
  return;

 bgeu:
  execBgeu(di);
  return;

 lb:
  execLb(di);
  return;

 lh:
  execLh(di);
  return;

 lw:
  execLw(di);
  return;

 lbu:
  execLbu(di);
  return;

 lhu:
  execLhu(di);
  return;

 sb:
  execSb(di);
  return;

 sh:
  execSh(di);
  return;

 sw:
  execSw(di);
  return;

 addi:
  execAddi(di);
  return;

 slti:
  execSlti(di);
  return;

 sltiu:
  execSltiu(di);
  return;

 xori:
  execXori(di);
  return;

 ori:
  execOri(di);
  return;

 andi:
  execAndi(di);
  return;

 slli:
  execSlli(di);
  return;

 srli:
  execSrli(di);
  return;

 srai:
  execSrai(di);
  return;

 add:
  execAdd(di);
  return;

 sub:
  execSub(di);
  return;

 sll:
  execSll(di);
  return;

 slt:
  execSlt(di);
  return;

 sltu:
  execSltu(di);
  return;

 xor_:
  execXor(di);
  return;

 srl:
  execSrl(di);
  return;

 sra:
  execSra(di);
  return;

 or_:
  execOr(di);
  return;

 and_:
  execAnd(di);
  return;

 fence:
  execFence(di);
  return;

 fence_tso:
  execFence_tso(di);
  return;

 fencei:
  execFencei(di);
  return;

 ecall:
  execEcall(di);
  return;

 ebreak:
  execEbreak(di);
  return;

 csrrw:
  execCsrrw(di);
  return;

 csrrs:
  execCsrrs(di);
  return;

 csrrc:
  execCsrrc(di);
  return;

 csrrwi:
  execCsrrwi(di);
  return;

 csrrsi:
  execCsrrsi(di);
  return;

 csrrci:
  execCsrrci(di);
  return;

 lwu:
  execLwu(di);
  return;

 ld:
  execLd(di);
  return;

 sd:
  execSd(di);
  return;

 addiw:
  execAddiw(di);
  return;

 slliw:
  execSlliw(di);
  return;

 srliw:
  execSrliw(di);
  return;

 sraiw:
  execSraiw(di);
  return;

 addw:
  execAddw(di);
  return;

 subw:
  execSubw(di);
  return;

 sllw:
  execSllw(di);
  return;

 srlw:
  execSrlw(di);
  return;

 sraw:
  execSraw(di);
  return;

 mul:
  execMul(di);
  return;

 mulh:
  execMulh(di);
  return;

 mulhsu:
  execMulhsu(di);
  return;

 mulhu:
  execMulhu(di);
  return;

 div:
  execDiv(di);
  return;

 divu:
  execDivu(di);
  return;

 rem:
  execRem(di);
  return;

 remu:
  execRemu(di);
  return;

 mulw:
  execMulw(di);
  return;

 divw:
  execDivw(di);
  return;

 divuw:
  execDivuw(di);
  return;

 remw:
  execRemw(di);
  return;

 remuw:
  execRemuw(di);
  return;

 lr_w:
  execLr_w(di);
  return;

 sc_w:
  execSc_w(di);
  return;

 amoswap_w:
  execAmoswap_w(di);
  return;

 amoadd_w:
  execAmoadd_w(di);
  return;

 amoxor_w:
  execAmoxor_w(di);
  return;

 amoand_w:
  execAmoand_w(di);
  return;

 amoor_w:
  execAmoor_w(di);
  return;

 amomin_w:
  execAmomin_w(di);
  return;

 amomax_w:
  execAmomax_w(di);
  return;

 amominu_w:
  execAmominu_w(di);
  return;

 amomaxu_w:
  execAmomaxu_w(di);
  return;

 lr_d:
  execLr_d(di);
  return;

 sc_d:
  execSc_d(di);
  return;

 amoswap_d:
  execAmoswap_d(di);
  return;

 amoadd_d:
  execAmoadd_d(di);
  return;

 amoxor_d:
  execAmoxor_d(di);
  return;

 amoand_d:
  execAmoand_d(di);
  return;

 amoor_d:
  execAmoor_d(di);
  return;

 amomin_d:
  execAmomin_d(di);
  return;

 amomax_d:
  execAmomax_d(di);
  return;

 amominu_d:
  execAmominu_d(di);
  return;

 amomaxu_d:
  execAmomaxu_d(di);
  return;

 flw:
  execFlw(di);
  return;

 fsw:
  execFsw(di);
  return;

 fmadd_s:
  execFmadd_s(di);
  return;

 fmsub_s:
  execFmsub_s(di);
  return;

 fnmsub_s:
  execFnmsub_s(di);
  return;

 fnmadd_s:
  execFnmadd_s(di);
  return;

 fadd_s:
  execFadd_s(di);
  return;

 fsub_s:
  execFsub_s(di);
  return;

 fmul_s:
  execFmul_s(di);
  return;

 fdiv_s:
  execFdiv_s(di);
  return;

 fsqrt_s:
  execFsqrt_s(di);
  return;

 fsgnj_s:
  execFsgnj_s(di);
  return;

 fsgnjn_s:
  execFsgnjn_s(di);
  return;

 fsgnjx_s:
  execFsgnjx_s(di);
  return;

 fmin_s:
  execFmin_s(di);
  return;

 fmax_s:
  execFmax_s(di);
  return;

 fcvt_w_s:
  execFcvt_w_s(di);
  return;

 fcvt_wu_s:
  execFcvt_wu_s(di);
  return;

 fmv_x_w:
  execFmv_x_w(di);
  return;

 feq_s:
  execFeq_s(di);
  return;

 flt_s:
  execFlt_s(di);
  return;

 fle_s:
  execFle_s(di);
  return;

 fclass_s:
  execFclass_s(di);
  return;

 fcvt_s_w:
  execFcvt_s_w(di);
  return;

 fcvt_s_wu:
  execFcvt_s_wu(di);
  return;

 fmv_w_x:
  execFmv_w_x(di);
  return;

 fcvt_l_s:
  execFcvt_l_s(di);
  return;

 fcvt_lu_s:
  execFcvt_lu_s(di);
  return;

 fcvt_s_l:
  execFcvt_s_l(di);
  return;

 fcvt_s_lu:
  execFcvt_s_lu(di);
  return;

 fld:
  execFld(di);
  return;

 fsd:
  execFsd(di);
  return;

 fmadd_d:
  execFmadd_d(di);
  return;

 fmsub_d:
  execFmsub_d(di);
  return;

 fnmsub_d:
  execFnmsub_d(di);
  return;

 fnmadd_d:
  execFnmadd_d(di);
  return;

 fadd_d:
  execFadd_d(di);
  return;

 fsub_d:
  execFsub_d(di);
  return;

 fmul_d:
  execFmul_d(di);
  return;

 fdiv_d:
  execFdiv_d(di);
  return;

 fsqrt_d:
  execFsqrt_d(di);
  return;

 fsgnj_d:
  execFsgnj_d(di);
  return;

 fsgnjn_d:
  execFsgnjn_d(di);
  return;

 fsgnjx_d:
  execFsgnjx_d(di);
  return;

 fmin_d:
  execFmin_d(di);
  return;

 fmax_d:
  execFmax_d(di);
  return;

 fcvt_s_d:
  execFcvt_s_d(di);
  return;

 fcvt_d_s:
  execFcvt_d_s(di);
  return;

 feq_d:
  execFeq_d(di);
  return;

 flt_d:
  execFlt_d(di);
  return;

 fle_d:
  execFle_d(di);
  return;

 fclass_d:
  execFclass_d(di);
  return;

 fcvt_w_d:
  execFcvt_w_d(di);
  return;

 fcvt_wu_d:
  execFcvt_wu_d(di);
  return;

 fcvt_d_w:
  execFcvt_d_w(di);
  return;

 fcvt_d_wu:
  execFcvt_d_wu(di);
  return;

 fcvt_l_d:
  execFcvt_l_d(di);
  return;

 fcvt_lu_d:
  execFcvt_lu_d(di);
  return;

 fmv_x_d:
  execFmv_x_d(di);
  return;

 fcvt_d_l:
  execFcvt_d_l(di);
  return;

 fcvt_d_lu:
  execFcvt_d_lu(di);
  return;

 fmv_d_x:
  execFmv_d_x(di);
  return;

 flh:
  execFlh(di);
  return;

 fsh:
  execFsh(di);
  return;

 fmadd_h:
  execFmadd_h(di);
  return;

 fmsub_h:
  execFmsub_h(di);
  return;

 fnmsub_h:
  execFnmsub_h(di);
  return;

 fnmadd_h:
  execFnmadd_h(di);
  return;

 fadd_h:
  execFadd_h(di);
  return;

 fsub_h:
  execFsub_h(di);
  return;

 fmul_h:
  execFmul_h(di);
  return;

 fdiv_h:
  execFdiv_h(di);
  return;

 fsqrt_h:
  execFsqrt_h(di);
  return;

 fsgnj_h:
  execFsgnj_h(di);
  return;

 fsgnjn_h:
  execFsgnjn_h(di);
  return;

 fsgnjx_h:
  execFsgnjx_h(di);
  return;

 fmin_h:
  execFmin_h(di);
  return;

 fmax_h:
  execFmax_h(di);
  return;

 fcvt_s_h:
  execFcvt_s_h(di);
  return;

 fcvt_d_h:
  execFcvt_d_h(di);
  return;

 fcvt_h_s:
  execFcvt_h_s(di);
  return;

 fcvt_h_d:
  execFcvt_h_d(di);
  return;

 fcvt_w_h:
  execFcvt_w_h(di);
  return;

 fcvt_wu_h:
  execFcvt_wu_h(di);
  return;

 fmv_x_h:
  execFmv_x_h(di);
  return;

 feq_h:
  execFeq_h(di);
  return;

 flt_h:
  execFlt_h(di);
  return;

 fle_h:
  execFle_h(di);
  return;

 fclass_h:
  execFclass_h(di);
  return;

 fcvt_h_w:
  execFcvt_h_w(di);
  return;

 fcvt_h_wu:
  execFcvt_h_wu(di);
  return;

 fmv_h_x:
  execFmv_h_x(di);
  return;

 fcvt_l_h:
  execFcvt_l_h(di);
  return;

 fcvt_lu_h:
  execFcvt_lu_h(di);
  return;

 fcvt_h_l:
  execFcvt_h_l(di);
  return;

 fcvt_h_lu:
  execFcvt_h_lu(di);
  return;

 mret:
  execMret(di);
  return;

 uret:
  execUret(di);
  return;

 sret:
  execSret(di);
  return;

 wfi:
  execWfi(di);
  return;

 dret:
  execDret(di);
  return;

 sfence_vma:
  execSfence_vma(di);
  return;

 c_addi4spn:
  if (not isRvc()) illegalInst(di); else execAddi(di);
  return;

 c_fld:
  if (not isRvc()) illegalInst(di); else execFld(di);
  return;

 c_lq:
  if (not isRvc()) illegalInst(di); else illegalInst(di);
  return;

 c_lw:
  if (not isRvc()) illegalInst(di); else execLw(di);
  return;

 c_flw:
  if (not isRvc()) illegalInst(di); else execFlw(di);
  return;

 c_ld:
  if (not isRvc()) illegalInst(di); else execLd(di);
  return;

 c_fsd:
  if (not isRvc()) illegalInst(di); else execFsd(di);
  return;

 c_sq:
  if (not isRvc()) illegalInst(di); else illegalInst(di);
  return;

 c_sw:
  if (not isRvc()) illegalInst(di); else execSw(di);
  return;

 c_fsw:
  if (not isRvc()) illegalInst(di); else execFsw(di);
  return;

 c_sd:
  if (not isRvc()) illegalInst(di); else execSd(di);
  return;

 c_addi:
  if (not isRvc()) illegalInst(di); else execAddi(di);
  return;

 c_jal:
  if (not isRvc()) illegalInst(di); else execJal(di);
  return;

 c_li:
  if (not isRvc()) illegalInst(di); else execAddi(di);
  return;

 c_addi16sp:
  if (not isRvc()) illegalInst(di); else execAddi(di);
  return;

 c_lui:
  if (not isRvc()) illegalInst(di); else execLui(di);
  return;

 c_srli:
  if (not isRvc()) illegalInst(di); else execSrli(di);
  return;

 c_srli64:
  illegalInst(di); // Only valid in rv128 which is not supported.
  return;

 c_srai:
  if (not isRvc()) illegalInst(di); else execSrai(di);
  return;

 c_srai64:
  illegalInst(di); // Only valid in rv128 which is not supported.
  return;

 c_andi:
  if (not isRvc()) illegalInst(di); else execAndi(di);
  return;

 c_sub:
  if (not isRvc()) illegalInst(di); else execSub(di);
  return;

 c_xor:
  if (not isRvc()) illegalInst(di); else execXor(di);
  return;

 c_or:
  if (not isRvc()) illegalInst(di); else execOr(di);
  return;

 c_and:
  if (not isRvc()) illegalInst(di); else execAnd(di);
  return;

 c_subw:
  if (not isRvc()) illegalInst(di); else execSubw(di);
  return;

 c_addw:
  if (not isRvc()) illegalInst(di); else execAddw(di);
  return;

 c_j:
  if (not isRvc()) illegalInst(di); else execJal(di);
  return;

 c_beqz:
  if (not isRvc()) illegalInst(di); else execBeq(di);
  return;

 c_bnez:
  if (not isRvc()) illegalInst(di); else execBne(di);
  return;

 c_slli:
  if (not isRvc()) illegalInst(di); else execSlli(di);
  return;

 c_slli64:
  if (not isRvc()) illegalInst(di); else execSlli(di);
  return;

 c_fldsp:
  if (not isRvc()) illegalInst(di); else execFld(di);
  return;

 c_lwsp:
  if (not isRvc()) illegalInst(di); else execLw(di);
  return;

 c_flwsp:
  if (not isRvc()) illegalInst(di); else execFlw(di);
  return;

 c_ldsp:
  if (not isRvc()) illegalInst(di); else execLd(di);
  return;

 c_jr:
  if (not isRvc()) illegalInst(di); else execJalr(di);
  return;

 c_mv:
  if (not isRvc()) illegalInst(di); else execAdd(di);
  return;

 c_ebreak:
  if (not isRvc()) illegalInst(di); else execEbreak(di);
  return;

 c_jalr:
  if (not isRvc()) illegalInst(di); else execJalr(di);
  return;

 c_add:
  if (not isRvc()) illegalInst(di); else execAdd(di);
  return;

 c_fsdsp:
  if (not isRvc()) illegalInst(di); else execFsd(di);
  return;

 c_swsp:
  if (not isRvc()) illegalInst(di); else execSw(di);
  return;

 c_fswsp:
  if (not isRvc()) illegalInst(di); else execFsw(di);
  return;

 c_addiw:
  if (not isRvc()) illegalInst(di); else execAddiw(di);
  return;

 c_sdsp:
  if (not isRvc()) illegalInst(di); else execSd(di);
  return;

 clz:
  execClz(di);
  return;

 ctz:
  execCtz(di);
  return;

 cpop:
  execCpop(di);
  return;

 clzw:
  execClzw(di);
  return;

 ctzw:
  execCtzw(di);
  return;

 cpopw:
  execCpopw(di);
  return;

 min:
  execMin(di);
  return;

 max:
  execMax(di);
  return;

 minu:
  execMinu(di);
  return;

 maxu:
  execMaxu(di);
  return;

 sext_b:
  execSext_b(di);
  return;

 sext_h:
  execSext_h(di);
  return;

 andn:
  execAndn(di);
  return;

 orn:
  execOrn(di);
  return;

 xnor:
  execXnor(di);
  return;

 rol:
  execRol(di);
  return;

 ror:
  execRor(di);
  return;

 rori:
  execRori(di);
  return;

 rolw:
  execRolw(di);
  return;

 rorw:
  execRorw(di);
  return;

 roriw:
  execRoriw(di);
  return;

 rev8:
  execRev8(di);
  return;

 pack:
  execPack(di);
  return;

 packh:
  execPackh(di);
  return;

 packu:
  execPacku(di);
  return;

 packw:
  execPackw(di);
  return;

 packuw:
  execPackuw(di);
  return;

 grev:
  execGrev(di);
  return;

 grevi:
  execGrevi(di);
  return;

 grevw:
  execGrevw(di);
  return;

 greviw:
  execGreviw(di);
  return;

 gorc:
  execGorc(di);
  return;

 gorci:
  execGorci(di);
  return;

 gorcw:
  execGorcw(di);
  return;

 gorciw:
  execGorciw(di);
  return;

 shfl:
  execShfl(di);
  return;

 shflw:
  execShflw(di);
  return;

 shfli:
  execShfli(di);
  return;

 unshfl:
  execUnshfl(di);
  return;

 unshfli:
  execUnshfli(di);
  return;

 unshflw:
  execUnshflw(di);
  return;

 xperm_n:
  execXperm_n(di);
  return;

 xperm_b:
  execXperm_b(di);
  return;

 xperm_h:
  execXperm_h(di);
  return;

 xperm_w:
  execXperm_w(di);
  return;

 bset:
  execBset(di);
  return;

 bclr:
  execBclr(di);
  return;

 binv:
  execBinv(di);
  return;

 bext:
  execBext(di);
  return;

 bseti:
  execBseti(di);
  return;

 bclri:
  execBclri(di);
  return;

 binvi:
  execBinvi(di);
  return;

 bexti:
  execBexti(di);
  return;

 bcompress:
  execBcompress(di);
  return;

 bdecompress:
  execBdecompress(di);
  return;

 bcompressw:
  execBcompressw(di);
  return;

 bdecompressw:
  execBdecompressw(di);
  return;

 bfp:
  execBfp(di);
  return;

 bfpw:
  execBfpw(di);
  return;

 clmul:
  execClmul(di);
  return;

 clmulh:
  execClmulh(di);
  return;

 clmulr:
  execClmulr(di);
  return;

 sh1add:
  execSh1add(di);
  return;

 sh2add:
  execSh2add(di);
  return;

 sh3add:
  execSh3add(di);
  return;

 sh1add_uw:
  execSh1add_uw(di);
  return;

 sh2add_uw:
  execSh2add_uw(di);
  return;

 sh3add_uw:
  execSh3add_uw(di);
  return;

 add_uw:
  execAdd_uw(di);
  return;

 slli_uw:
  execSlli_uw(di);
  return;

 crc32_b:
  execCrc32_b(di);
  return;

 crc32_h:
  execCrc32_h(di);
  return;

 crc32_w:
  execCrc32_w(di);
  return;

 crc32_d:
  execCrc32_d(di);
  return;

 crc32c_b:
  execCrc32c_b(di);
  return;

 crc32c_h:
  execCrc32c_h(di);
  return;

 crc32c_w:
  execCrc32c_w(di);
  return;

 crc32c_d:
  execCrc32c_d(di);
  return;

 bmator:
  execBmator(di);
  return;

 bmatxor:
  execBmatxor(di);
  return;

 bmatflip:
  execBmatflip(di);
  return;

 cmov:
  execCmov(di);
  return;

 cmix:
  execCmix(di);
  return;

 fsl:
  execFsl(di);
  return;

 fsr:
  execFsr(di);
  return;

 fsri:
  execFsri(di);
  return;

 fslw:
  execFslw(di);
  return;

 fsrw:
  execFsrw(di);
  return;

 fsriw:
  execFsriw(di);
  return;

 vsetvli:
  execVsetvli(di);
  return;

 vsetivli:
  execVsetivli(di);
  return;

 vsetvl:
  execVsetvl(di);
  return;

 vadd_vv:
  execVadd_vv(di);
  return;

 vadd_vx:
  execVadd_vx(di);
  return;

 vadd_vi:
  execVadd_vi(di);
  return;

 vsub_vv:
  execVsub_vv(di);
  return;

 vsub_vx:
  execVsub_vx(di);
  return;

 vrsub_vx:
  execVrsub_vx(di);
  return;

 vrsub_vi:
  execVrsub_vi(di);
  return;

 vwaddu_vv:
  execVwaddu_vv(di);
  return;

 vwaddu_vx:
  execVwaddu_vx(di);
  return;

 vwsubu_vv:
  execVwsubu_vv(di);
  return;

 vwsubu_vx:
  execVwsubu_vx(di);
  return;

 vwadd_vv:
  execVwadd_vv(di);
  return;

 vwadd_vx:
  execVwadd_vx(di);
  return;

 vwsub_vv:
  execVwsub_vv(di);
  return;

 vwsub_vx:
  execVwsub_vx(di);
  return;

 vwaddu_wv:
  execVwaddu_wv(di);
  return;

 vwaddu_wx:
  execVwaddu_wx(di);
  return;

 vwsubu_wv:
  execVwsubu_wv(di);
  return;

 vwsubu_wx:
  execVwsubu_wx(di);
  return;

 vwadd_wv:
  execVwadd_wv(di);
  return;

 vwadd_wx:
  execVwadd_wx(di);
  return;

 vwsub_wv:
  execVwsub_wv(di);
  return;

 vwsub_wx:
  execVwsub_wx(di);
  return;

 vmseq_vv:
  execVmseq_vv(di);
  return;

 vmseq_vx:
  execVmseq_vx(di);
  return;

 vmseq_vi:
  execVmseq_vi(di);
  return;

 vmsne_vv:
  execVmsne_vv(di);
  return;

 vmsne_vx:
  execVmsne_vx(di);
  return;

 vmsne_vi:
  execVmsne_vi(di);
  return;

 vmsltu_vv:
  execVmsltu_vv(di);
  return;

 vmsltu_vx:
  execVmsltu_vx(di);
  return;

 vmslt_vv:
  execVmslt_vv(di);
  return;

 vmslt_vx:
  execVmslt_vx(di);
  return;

 vmsleu_vv:
  execVmsleu_vv(di);
  return;

 vmsleu_vx:
  execVmsleu_vx(di);
  return;

 vmsleu_vi:
  execVmsleu_vi(di);
  return;

 vmsle_vv:
  execVmsle_vv(di);
  return;

 vmsle_vx:
  execVmsle_vx(di);
  return;

 vmsle_vi:
  execVmsle_vi(di);
  return;

 vmsgtu_vx:
  execVmsgtu_vx(di);
  return;

 vmsgtu_vi:
  execVmsgtu_vi(di);
  return;

 vmsgt_vx:
  execVmsgt_vx(di);
  return;

 vmsgt_vi:
  execVmsgt_vi(di);
  return;

 vminu_vv:
  execVminu_vv(di);
  return;

 vminu_vx:
  execVminu_vx(di);
  return;

 vmin_vv:
  execVmin_vv(di);
  return;

 vmin_vx:
  execVmin_vx(di);
  return;

 vmaxu_vv:
  execVmaxu_vv(di);
  return;

 vmaxu_vx:
  execVmaxu_vx(di);
  return;

 vmax_vv:
  execVmax_vv(di);
  return;

 vmax_vx:
  execVmax_vx(di);
  return;

 vand_vv:
  execVand_vv(di);
  return;

 vand_vx:
  execVand_vx(di);
  return;

 vand_vi:
  execVand_vi(di);
  return;

 vor_vv:
  execVor_vv(di);
  return;

 vor_vx:
  execVor_vx(di);
  return;

 vor_vi:
  execVor_vi(di);
  return;

 vxor_vv:
  execVxor_vv(di);
  return;

 vxor_vx:
  execVxor_vx(di);
  return;

 vxor_vi:
  execVxor_vi(di);
  return;

 vsll_vv:
  execVsll_vv(di);
  return;

 vsll_vx:
  execVsll_vx(di);
  return;

 vsll_vi:
  execVsll_vi(di);
  return;

 vsrl_vv:
  execVsrl_vv(di);
  return;

 vsrl_vx:
  execVsrl_vx(di);
  return;

 vsrl_vi:
  execVsrl_vi(di);
  return;

 vsra_vv:
  execVsra_vv(di);
  return;

 vsra_vx:
  execVsra_vx(di);
  return;

 vsra_vi:
  execVsra_vi(di);
  return;

 vnsrl_wv:
  execVnsrl_wv(di);
  return;

 vnsrl_wx:
  execVnsrl_wx(di);
  return;

 vnsrl_wi:
  execVnsrl_wi(di);
  return;

 vnsra_wv:
  execVnsra_wv(di);
  return;

 vnsra_wx:
  execVnsra_wx(di);
  return;

 vnsra_wi:
  execVnsra_wi(di);
  return;

 vrgather_vv:
  execVrgather_vv(di);
  return;

 vrgather_vx:
  execVrgather_vx(di);
  return;

 vrgather_vi:
  execVrgather_vi(di);
  return;

 vrgatherei16_vv:
  execVrgatherei16_vv(di);
  return;

 vcompress_vm:
  execVcompress_vm(di);
  return;

 vredsum_vs:
  execVredsum_vs(di);
  return;

 vredand_vs:
  execVredand_vs(di);
  return;

 vredor_vs:
  execVredor_vs(di);
  return;

 vredxor_vs:
  execVredxor_vs(di);
  return;

 vredminu_vs:
  execVredminu_vs(di);
  return;

 vredmin_vs:
  execVredmin_vs(di);
  return;

 vredmaxu_vs:
  execVredmaxu_vs(di);
  return;

 vredmax_vs:
  execVredmax_vs(di);
  return;

 vwredsumu_vs:
  execVwredsumu_vs(di);
  return;

 vwredsum_vs:
  execVwredsum_vs(di);
  return;

 vmand_mm:
  execVmand_mm(di);
  return;

 vmnand_mm:
  execVmnand_mm(di);
  return;

 vmandnot_mm:
  execVmandnot_mm(di);
  return;

 vmxor_mm:
  execVmxor_mm(di);
  return;

 vmor_mm:
  execVmor_mm(di);
  return;

 vmnor_mm:
  execVmnor_mm(di);
  return;

 vmornot_mm:
  execVmornot_mm(di);
  return;

 vmxnor_mm:
  execVmxnor_mm(di);
  return;

 vpopc_m:
  execVpopc_m(di);
  return;

 vfirst_m:
  execVfirst_m(di);
  return;

 vmsbf_m:
  execVmsbf_m(di);
  return;

 vmsif_m:
  execVmsif_m(di);
  return;

 vmsof_m:
  execVmsof_m(di);
  return;

 viota_m:
  execViota_m(di);
  return;

 vid_v:
  execVid_v(di);
  return;

 vslideup_vx:
  execVslideup_vx(di);
  return;

 vslideup_vi:
  execVslideup_vi(di);
  return;

 vslide1up_vx:
  execVslide1up_vx(di);
  return;

 vslidedown_vx:
  execVslidedown_vx(di);
  return;

 vslidedown_vi:
  execVslidedown_vi(di);
  return;

 vslide1down_vx:
  execVslide1down_vx(di);
  return;

 vfslide1up_vf:
  execVfslide1up_vf(di);
  return;

 vfslide1down_vf:
  execVfslide1down_vf(di);
  return;

 vmul_vv:
  execVmul_vv(di);
  return;

 vmul_vx:
  execVmul_vx(di);
  return;

 vmulh_vv:
  execVmulh_vv(di);
  return;

 vmulh_vx:
  execVmulh_vx(di);
  return;

 vmulhu_vv:
  execVmulhu_vv(di);
  return;

 vmulhu_vx:
  execVmulhu_vx(di);
  return;

 vmulhsu_vv:
  execVmulhsu_vv(di);
  return;

 vmulhsu_vx:
  execVmulhsu_vx(di);
  return;

 vmadd_vv:
  execVmadd_vv(di);
  return;

 vmadd_vx:
  execVmadd_vx(di);
  return;

 vnmsub_vv:
  execVnmsub_vv(di);
  return;

 vnmsub_vx:
  execVnmsub_vx(di);
  return;

 vmacc_vv:
  execVmacc_vv(di);
  return;

 vmacc_vx:
  execVmacc_vx(di);
  return;

 vnmsac_vv:
  execVnmsac_vv(di);
  return;

 vnmsac_vx:
  execVnmsac_vx(di);
  return;

 vwmulu_vv:
  execVwmulu_vv(di);
  return;

 vwmulu_vx:
  execVwmulu_vx(di);
  return;

 vwmul_vv:
  execVwmul_vv(di);
  return;

 vwmul_vx:
  execVwmul_vx(di);
  return;

 vwmulsu_vv:
  execVwmulsu_vv(di);
  return;

 vwmulsu_vx:
  execVwmulsu_vx(di);
  return;

 vwmaccu_vv:
  execVwmaccu_vv(di);
  return;

 vwmaccu_vx:
  execVwmaccu_vx(di);
  return;

 vwmacc_vv:
  execVwmacc_vv(di);
  return;

 vwmacc_vx:
  execVwmacc_vx(di);
  return;

 vwmaccsu_vv:
  execVwmaccsu_vv(di);
  return;

 vwmaccsu_vx:
  execVwmaccsu_vx(di);
  return;

 vwmaccus_vx:
  execVwmaccus_vx(di);
  return;

 vdivu_vv:
  execVdivu_vv(di);
  return;

 vdivu_vx:
  execVdivu_vx(di);
  return;

 vdiv_vv:
  execVdiv_vv(di);
  return;

 vdiv_vx:
  execVdiv_vx(di);
  return;

 vremu_vv:
  execVremu_vv(di);
  return;

 vremu_vx:
  execVremu_vx(di);
  return;

 vrem_vv:
  execVrem_vv(di);
  return;

 vrem_vx:
  execVrem_vx(di);
  return;

 vsext_vf2:
  execVsext_vf2(di);
  return;

 vsext_vf4:
  execVsext_vf4(di);
  return;

 vsext_vf8:
  execVsext_vf8(di);
  return;

 vzext_vf2:
  execVzext_vf2(di);
  return;

 vzext_vf4:
  execVzext_vf4(di);
  return;

 vzext_vf8:
  execVzext_vf8(di);
  return;

 vadc_vvm:
  execVadc_vvm(di);
  return;

 vadc_vxm:
  execVadc_vxm(di);
  return;

 vadc_vim:
  execVadc_vim(di);
  return;

 vsbc_vvm:
  execVsbc_vvm(di);
  return;

 vsbc_vxm:
  execVsbc_vxm(di);
  return;

 vmadc_vvm:
  execVmadc_vvm(di);
  return;

 vmadc_vxm:
  execVmadc_vxm(di);
  return;

 vmadc_vim:
  execVmadc_vim(di);
  return;

 vmsbc_vvm:
  execVmsbc_vvm(di);
  return;

 vmsbc_vxm:
  execVmsbc_vxm(di);
  return;

 vmerge_vvm:
  execVmerge_vvm(di);
  return;

 vmerge_vxm:
  execVmerge_vxm(di);
  return;

 vmerge_vim:
  execVmerge_vim(di);
  return;

 vmv_x_s:
  execVmv_x_s(di);
  return;

 vmv_s_x:
  execVmv_s_x(di);
  return;

 vfmv_f_s:
  execVfmv_f_s(di);
  return;

 vfmv_s_f:
  execVfmv_s_f(di);
  return;

 vmv_v_v:
  execVmv_v_v(di);
  return;

 vmv_v_x:
  execVmv_v_x(di);
  return;

 vmv_v_i:
  execVmv_v_i(di);
  return;

 vmv1r_v:
  execVmv1r_v(di);
  return;

 vmv2r_v:
  execVmv2r_v(di);
  return;

 vmv4r_v:
  execVmv4r_v(di);
  return;

 vmv8r_v:
  execVmv8r_v(di);
  return;

 vsaddu_vv:
  execVsaddu_vv(di);
  return;

 vsaddu_vx:
  execVsaddu_vx(di);
  return;

 vsaddu_vi:
  execVsaddu_vi(di);
  return;

 vsadd_vv:
  execVsadd_vv(di);
  return;

 vsadd_vx:
  execVsadd_vx(di);
  return;

 vsadd_vi:
  execVsadd_vi(di);
  return;

 vssubu_vv:
  execVssubu_vv(di);
  return;

 vssubu_vx:
  execVssubu_vx(di);
  return;

 vssub_vv:
  execVssub_vv(di);
  return;

 vssub_vx:
  execVssub_vx(di);
  return;

 vaaddu_vv:
  execVaaddu_vv(di);
  return;

 vaaddu_vx:
  execVaaddu_vx(di);
  return;

 vaadd_vv:
  execVaadd_vv(di);
  return;

 vaadd_vx:
  execVaadd_vx(di);
  return;

 vasubu_vv:
  execVasubu_vv(di);
  return;

 vasubu_vx:
  execVasubu_vx(di);
  return;

 vasub_vv:
  execVasub_vv(di);
  return;

 vasub_vx:
  execVasub_vx(di);
  return;

 vsmul_vv:
  execVsmul_vv(di);
  return;

 vsmul_vx:
  execVsmul_vx(di);
  return;

 vssrl_vv:
  execVssrl_vv(di);
  return;

 vssrl_vx:
  execVssrl_vx(di);
  return;

 vssrl_vi:
  execVssrl_vi(di);
  return;

 vssra_vv:
  execVssra_vv(di);
  return;

 vssra_vx:
  execVssra_vx(di);
  return;

 vssra_vi:
  execVssra_vi(di);
  return;

 vnclipu_wv:
  execVnclipu_wv(di);
  return;

 vnclipu_wx:
  execVnclipu_wx(di);
  return;

 vnclipu_wi:
  execVnclipu_wi(di);
  return;

 vnclip_wv:
  execVnclip_wv(di);
  return;

 vnclip_wx:
  execVnclip_wx(di);
  return;

 vnclip_wi:
  execVnclip_wi(di);
  return;

 vle8_v:
  execVle8_v(di);
  return;

 vle16_v:
  execVle16_v(di);
  return;

 vle32_v:
  execVle32_v(di);
  return;

 vle64_v:
  execVle64_v(di);
  return;

 vle128_v:
  execVle128_v(di);
  return;

 vle256_v:
  execVle256_v(di);
  return;

 vle512_v:
  execVle512_v(di);
  return;

 vle1024_v:
  execVle1024_v(di);
  return;

 vse8_v:
  execVse8_v(di);
  return;

 vse16_v:
  execVse16_v(di);
  return;

 vse32_v:
  execVse32_v(di);
  return;

 vse64_v:
  execVse64_v(di);
  return;

 vse128_v:
  execVse128_v(di);
  return;

 vse256_v:
  execVse256_v(di);
  return;

 vse512_v:
  execVse512_v(di);
  return;

 vse1024_v:
  execVse1024_v(di);
  return;

 vlm_v:
  execVlm_v(di);
  return;

 vsm_v:
  execVsm_v(di);
  return;

 vlre8_v:
  execVlre8_v(di);
  return;

 vlre16_v:
  execVlre16_v(di);
  return;

 vlre32_v:
  execVlre32_v(di);
  return;

 vlre64_v:
  execVlre64_v(di);
  return;

 vlre128_v:
  execVlre128_v(di);
  return;

 vlre256_v:
  execVlre256_v(di);
  return;

 vlre512_v:
  execVlre512_v(di);
  return;

 vlre1024_v:
  execVlre1024_v(di);
  return;

 vs1r_v:
  execVs1r_v(di);
  return;

 vs2r_v:
  execVs2r_v(di);
  return;

 vs4r_v:
  execVs4r_v(di);
  return;

 vs8r_v:
  execVs8r_v(di);
  return;

 vle8ff_v:
  execVle8ff_v(di);
  return;

 vle16ff_v:
  execVle16ff_v(di);
  return;

 vle32ff_v:
  execVle32ff_v(di);
  return;

 vle64ff_v:
  execVle64ff_v(di);
  return;

 vle128ff_v:
  execVle128ff_v(di);
  return;

 vle256ff_v:
  execVle256ff_v(di);
  return;

 vle512ff_v:
  execVle512ff_v(di);
  return;

 vle1024ff_v:
  execVle1024ff_v(di);
  return;

 vlse8_v:
  execVlse8_v(di);
  return;

 vlse16_v:
  execVlse16_v(di);
  return;

 vlse32_v:
  execVlse32_v(di);
  return;

 vlse64_v:
  execVlse64_v(di);
  return;

 vlse128_v:
  execVlse128_v(di);
  return;

 vlse256_v:
  execVlse256_v(di);
  return;

 vlse512_v:
  execVlse512_v(di);
  return;

 vlse1024_v:
  execVlse1024_v(di);
  return;

 vsse8_v:
  execVsse8_v(di);
  return;

 vsse16_v:
  execVsse16_v(di);
  return;

 vsse32_v:
  execVsse32_v(di);
  return;

 vsse64_v:
  execVsse64_v(di);
  return;

 vsse128_v:
  execVsse128_v(di);
  return;

 vsse256_v:
  execVsse256_v(di);
  return;

 vsse512_v:
  execVsse512_v(di);
  return;

 vsse1024_v:
  execVsse1024_v(di);
  return;

 vloxei8_v:
  execVloxei8_v(di);
  return;

 vloxei16_v:
  execVloxei16_v(di);
  return;

 vloxei32_v:
  execVloxei32_v(di);
  return;

 vloxei64_v:
  execVloxei64_v(di);
  return;

 vluxei8_v:
  execVluxei8_v(di);
  return;

 vluxei16_v:
  execVluxei16_v(di);
  return;

 vluxei32_v:
  execVluxei32_v(di);
  return;

 vluxei64_v:
  execVluxei64_v(di);
  return;

 vsoxei8_v:
  execVsoxei8_v(di);
  return;

 vsoxei16_v:
  execVsoxei16_v(di);
  return;

 vsoxei32_v:
  execVsoxei32_v(di);
  return;

 vsoxei64_v:
  execVsoxei64_v(di);
  return;

 vsuxei8_v:
  execVsuxei8_v(di);
  return;

 vsuxei16_v:
  execVsuxei16_v(di);
  return;

 vsuxei32_v:
  execVsuxei32_v(di);
  return;

 vsuxei64_v:
  execVsuxei64_v(di);
  return;

 vlsege8_v:
  execVlsege8_v(di);
  return;

 vlsege16_v:
  execVlsege16_v(di);
  return;

 vlsege32_v:
  execVlsege32_v(di);
  return;

 vlsege64_v:
  execVlsege64_v(di);
  return;

 vlsege128_v:
  execVlsege128_v(di);
  return;

 vlsege256_v:
  execVlsege256_v(di);
  return;

 vlsege512_v:
  execVlsege512_v(di);
  return;

 vlsege1024_v:
  execVlsege1024_v(di);
  return;

 vssege8_v:
  execVssege8_v(di);
  return;

 vssege16_v:
  execVssege16_v(di);
  return;

 vssege32_v:
  execVssege32_v(di);
  return;

 vssege64_v:
  execVssege64_v(di);
  return;

 vssege128_v:
  execVssege128_v(di);
  return;

 vssege256_v:
  execVssege256_v(di);
  return;

 vssege512_v:
  execVssege512_v(di);
  return;

 vssege1024_v:
  execVssege1024_v(di);
  return;

 vlssege8_v:
  execVlssege8_v(di);
  return;

 vlssege16_v:
  execVlssege16_v(di);
  return;

 vlssege32_v:
  execVlssege32_v(di);
  return;

 vlssege64_v:
  execVlssege64_v(di);
  return;

 vlssege128_v:
  execVlssege128_v(di);
  return;

 vlssege256_v:
  execVlssege256_v(di);
  return;

 vlssege512_v:
  execVlssege512_v(di);
  return;

 vlssege1024_v:
  execVlssege1024_v(di);
  return;

 vsssege8_v:
  execVsssege8_v(di);
  return;

 vsssege16_v:
  execVsssege16_v(di);
  return;

 vsssege32_v:
  execVsssege32_v(di);
  return;

 vsssege64_v:
  execVsssege64_v(di);
  return;

 vsssege128_v:
  execVsssege128_v(di);
  return;

 vsssege256_v:
  execVsssege256_v(di);
  return;

 vsssege512_v:
  execVsssege512_v(di);
  return;

 vsssege1024_v:
  execVsssege1024_v(di);
  return;

 vluxsegei8_v:
  execVluxsegei8_v(di);
  return;

 vluxsegei16_v:
  execVluxsegei16_v(di);
  return;

 vluxsegei32_v:
  execVluxsegei32_v(di);
  return;

 vluxsegei64_v:
  execVluxsegei64_v(di);
  return;

 vluxsegei128_v:
  execVluxsegei128_v(di);
  return;

 vluxsegei256_v:
  execVluxsegei256_v(di);
  return;

 vluxsegei512_v:
  execVluxsegei512_v(di);
  return;

 vluxsegei1024_v:
  execVluxsegei1024_v(di);
  return;

 vsuxsegei8_v:
  execVsuxsegei8_v(di);
  return;

 vsuxsegei16_v:
  execVsuxsegei16_v(di);
  return;

 vsuxsegei32_v:
  execVsuxsegei32_v(di);
  return;

 vsuxsegei64_v:
  execVsuxsegei64_v(di);
  return;

 vsuxsegei128_v:
  execVsuxsegei128_v(di);
  return;

 vsuxsegei256_v:
  execVsuxsegei256_v(di);
  return;

 vsuxsegei512_v:
  execVsuxsegei512_v(di);
  return;

 vsuxsegei1024_v:
  execVsuxsegei1024_v(di);
  return;

 vloxsegei8_v:
  execVloxsegei8_v(di);
  return;

 vloxsegei16_v:
  execVloxsegei16_v(di);
  return;

 vloxsegei32_v:
  execVloxsegei32_v(di);
  return;

 vloxsegei64_v:
  execVloxsegei64_v(di);
  return;

 vloxsegei128_v:
  execVloxsegei128_v(di);
  return;

 vloxsegei256_v:
  execVloxsegei256_v(di);
  return;

 vloxsegei512_v:
  execVloxsegei512_v(di);
  return;

 vloxsegei1024_v:
  execVloxsegei1024_v(di);
  return;

 vsoxsegei8_v:
  execVsoxsegei8_v(di);
  return;

 vsoxsegei16_v:
  execVsoxsegei16_v(di);
  return;

 vsoxsegei32_v:
  execVsoxsegei32_v(di);
  return;

 vsoxsegei64_v:
  execVsoxsegei64_v(di);
  return;

 vsoxsegei128_v:
  execVsoxsegei128_v(di);
  return;

 vsoxsegei256_v:
  execVsoxsegei256_v(di);
  return;

 vsoxsegei512_v:
  execVsoxsegei512_v(di);
  return;

 vsoxsegei1024_v:
  execVsoxsegei1024_v(di);
  return;

 vlsege8ff_v:
  execVlsege8ff_v(di);
  return;

 vlsege16ff_v:
  execVlsege16ff_v(di);
  return;

 vlsege32ff_v:
  execVlsege32ff_v(di);
  return;

 vlsege64ff_v:
  execVlsege64ff_v(di);
  return;

 vlsege128ff_v:
  execVlsege128ff_v(di);
  return;

 vlsege256ff_v:
  execVlsege256ff_v(di);
  return;

 vlsege512ff_v:
  execVlsege512ff_v(di);
  return;

 vlsege1024ff_v:
  execVlsege1024ff_v(di);
  return;

 vfadd_vv:
  execVfadd_vv(di);
  return;

 vfadd_vf:
  execVfadd_vf(di);
  return;

 vfsub_vv:
  execVfsub_vv(di);
  return;

 vfsub_vf:
  execVfsub_vf(di);
  return;

 vfrsub_vf:
  execVfrsub_vf(di);
  return;

 vfwadd_vv:
  execVfwadd_vv(di);
  return;

 vfwadd_vf:
  execVfwadd_vf(di);
  return;

 vfwsub_vv:
  execVfwsub_vv(di);
  return;

 vfwsub_vf:
  execVfwsub_vf(di);
  return;

 vfwadd_wv:
  execVfwadd_wv(di);
  return;

 vfwadd_wf:
  execVfwadd_wf(di);
  return;

 vfwsub_wv:
  execVfwsub_wv(di);
  return;

 vfwsub_wf:
  execVfwsub_wf(di);
  return;

 vfmul_vv:
  execVfmul_vv(di);
  return;

 vfmul_vf:
  execVfmul_vf(di);
  return;

 vfdiv_vv:
  execVfdiv_vv(di);
  return;

 vfdiv_vf:
  execVfdiv_vf(di);
  return;

 vfrdiv_vf:
  execVfrdiv_vf(di);
  return;

 vfwmul_vv:
  execVfwmul_vv(di);
  return;

 vfwmul_vf:
  execVfwmul_vf(di);
  return;

 vfmadd_vv:
  execVfmadd_vv(di);
  return;

 vfmadd_vf:
  execVfmadd_vf(di);
  return;

 vfnmadd_vv:
  execVfnmadd_vv(di);
  return;

 vfnmadd_vf:
  execVfnmadd_vf(di);
  return;

 vfmsub_vv:
  execVfmsub_vv(di);
  return;

 vfmsub_vf:
  execVfmsub_vf(di);
  return;

 vfnmsub_vv:
  execVfnmsub_vv(di);
  return;

 vfnmsub_vf:
  execVfnmsub_vf(di);
  return;

 vfmacc_vv:
  execVfmacc_vv(di);
  return;

 vfmacc_vf:
  execVfmacc_vf(di);
  return;

 vfnmacc_vv:
  execVfnmacc_vv(di);
  return;

 vfnmacc_vf:
  execVfnmacc_vf(di);
  return;

 vfmsac_vv:
  execVfmsac_vv(di);
  return;

 vfmsac_vf:
  execVfmsac_vf(di);
  return;

 vfnmsac_vv:
  execVfnmsac_vv(di);
  return;

 vfnmsac_vf:
  execVfnmsac_vf(di);
  return;

 vfwmacc_vv:
  execVfwmacc_vv(di);
  return;

 vfwmacc_vf:
  execVfwmacc_vf(di);
  return;

 vfwnmacc_vv:
  execVfwnmacc_vv(di);
  return;

 vfwnmacc_vf:
  execVfwnmacc_vf(di);
  return;

 vfwmsac_vv:
  execVfwmsac_vv(di);
  return;

 vfwmsac_vf:
  execVfwmsac_vf(di);
  return;

 vfwnmsac_vv:
  execVfwnmsac_vv(di);
  return;

 vfwnmsac_vf:
  execVfwnmsac_vf(di);
  return;

 vfsqrt_v:
  execVfsqrt_v(di);
  return;

 vfmerge_vfm:
  execVfmerge_vfm(di);
  return;

 vfmv_v_f:
  execVfmv_v_f(di);
  return;

 vmfeq_vv:
  execVmfeq_vv(di);
  return;

 vmfeq_vf:
  execVmfeq_vf(di);
  return;

 vmfne_vv:
  execVmfne_vv(di);
  return;

 vmfne_vf:
  execVmfne_vf(di);
  return;

 vmflt_vv:
  execVmflt_vv(di);
  return;

 vmflt_vf:
  execVmflt_vf(di);
  return;

 vmfle_vv:
  execVmfle_vv(di);
  return;

 vmfle_vf:
  execVmfle_vf(di);
  return;

 vmfgt_vf:
  execVmfgt_vf(di);
  return;

 vmfge_vf:
  execVmfge_vf(di);
  return;

 vfclass_v:
  execVfclass_v(di);
  return;

 vfcvt_xu_f_v:
  execVfcvt_xu_f_v(di);
  return;

 vfcvt_x_f_v:
  execVfcvt_x_f_v(di);
  return;

 vfcvt_rtz_xu_f_v:
  execVfcvt_rtz_xu_f_v(di);
  return;

 vfcvt_rtz_x_f_v:
  execVfcvt_rtz_x_f_v(di);
  return;

 vfcvt_f_xu_v:
  execVfcvt_f_xu_v(di);
  return;

 vfcvt_f_x_v:
  execVfcvt_f_x_v(di);
  return;

 vfwcvt_xu_f_v:
  execVfwcvt_xu_f_v(di);
  return;

 vfwcvt_x_f_v:
  execVfwcvt_x_f_v(di);
  return;

 vfwcvt_rtz_xu_f_v:
  execVfwcvt_rtz_xu_f_v(di);
  return;

 vfwcvt_rtz_x_f_v:
  execVfwcvt_rtz_x_f_v(di);
  return;

 vfwcvt_f_xu_v:
  execVfwcvt_f_xu_v(di);
  return;

 vfwcvt_f_x_v:
  execVfwcvt_f_x_v(di);
  return;

 vfwcvt_f_f_v:
  execVfwcvt_f_f_v(di);
  return;

 vfncvt_xu_f_w:
  execVfncvt_xu_f_w(di);
  return;

 vfncvt_x_f_w:
  execVfncvt_x_f_w(di);
  return;

 vfncvt_rtz_xu_f_w:
  execVfncvt_rtz_xu_f_w(di);
  return;

 vfncvt_rtz_x_f_w:
  execVfncvt_rtz_x_f_w(di);
  return;

 vfncvt_f_xu_w:
  execVfncvt_f_xu_w(di);
  return;

 vfncvt_f_x_w:
  execVfncvt_f_x_w(di);
  return;

 vfncvt_f_f_w:
  execVfncvt_f_f_w(di);
  return;

 vfncvt_rod_f_f_w:
  execVfncvt_rod_f_f_w(di);
  return;

 vfredsum_vs:
  execVfredsum_vs(di);
  return;

 vfredosum_vs:
  execVfredosum_vs(di);
  return;

 vfredmin_vs:
  execVfredmin_vs(di);
  return;

 vfredmax_vs:
  execVfredmax_vs(di);
  return;

 vfwredsum_vs:
  execVfwredsum_vs(di);
  return;

 vfwredosum_vs:
  execVfwredosum_vs(di);
  return;

 vfrsqrt7_v:
  execVfrsqrt7_v(di);
  return;

 vfrec7_v:
  execVfrec7_v(di);
  return;

 vfmin_vv:
  execVfmin_vv(di);
  return;

 vfmin_vf:
  execVfmin_vf(di);
  return;

 vfmax_vv:
  execVfmax_vv(di);
  return;

 vfmax_vf:
  execVfmax_vf(di);
  return;

 vfsgnj_vv:
  execVfsgnj_vv(di);
  return;

 vfsgnj_vf:
  execVfsgnj_vf(di);
  return;

 vfsgnjn_vv:
  execVfsgnjn_vv(di);
  return;

 vfsgnjn_vf:
  execVfsgnjn_vf(di);
  return;

 vfsgnjx_vv:
  execVfsgnjx_vv(di);
  return;

 vfsgnjx_vf:
  execVfsgnjx_vf(di);
  return;

 aes32dsi:
  execAes32dsi(di);
  return;

 aes32dsmi:
  execAes32dsmi(di);
  return;

 aes32esi:
  execAes32esi(di);
  return;

 aes32esmi:
  execAes32esmi(di);
  return;

 aes64ds:
  execAes64ds(di);
  return;

 aes64dsm:
  execAes64dsm(di);
  return;

 aes64es:
  execAes64es(di);
  return;

 aes64esm:
  execAes64esm(di);
  return;

 aes64im:
  execAes64im(di);
  return;

 aes64ks1i:
  execAes64ks1i(di);
  return;

 aes64ks2:
  execAes64ks2(di);
  return;

 sha256sig0:
  execSha256sig0(di);
  return;

 sha256sig1:
  execSha256sig1(di);
  return;

 sha256sum0:
  execSha256sum0(di);
  return;

 sha256sum1:
  execSha256sum1(di);
  return;

 sha512sig0h:
  execSha512sig0h(di);
  return;

 sha512sig0l:
  execSha512sig0l(di);
  return;

 sha512sig1h:
  execSha512sig1h(di);
  return;

 sha512sig1l:
  execSha512sig1l(di);
  return;

 sha512sum0r:
  execSha512sum0r(di);
  return;

 sha512sum1r:
  execSha512sum1r(di);
  return;

 sha512sig0:
  execSha512sig0(di);
  return;

 sha512sig1:
  execSha512sig1(di);
  return;

 sha512sum0:
  execSha512sum0(di);
  return;

 sha512sum1:
  execSha512sum1(di);
  return;

 sm3p0:
  execSm3p0(di);
  return;

 sm3p1:
  execSm3p1(di);
  return;

 sm4ed:
  execSm4ed(di);
  return;

 sm4ks:
  execSm4ks(di);
  return;

 sinval_vma:
  execSinval_vma(di);
  return;

 sfence_w_inval:
  execSfence_w_inval(di);
  return;

 sfence_inval_ir:
  execSfence_inval_ir(di);
  return;

 cbo_clean:
  execCbo_clean(di);
  return;

 cbo_flush:
  execCbo_flush(di);
  return;

 cbo_inval:
  execCbo_inval(di);
  return;

 cbo_zero:
  execCbo_zero(di);
  return;
}


template <typename URV>
void
Hart<URV>::enableInstructionFrequency(bool b)
{
  instFreq_ = b;
  if (b)
    instProfs_.configure();
}


template <typename URV>
void
Hart<URV>::enterDebugMode_(DebugModeCause cause, URV pc)
{
  cancelLr();  // Entering debug modes loses LR reservation.

  if (debugMode_)
    {
      if (debugStepMode_)
	debugStepMode_ = false;
      else
	std::cerr << "Error: Entering debug-halt while in debug-halt\n";
    }
  else
    {
      debugMode_ = true;
      if (debugStepMode_)
	std::cerr << "Error: Entering debug-halt with debug-step true\n";
      debugStepMode_ = false;
    }

  URV value = 0;
  if (peekCsr(CsrNumber::DCSR, value))
    {
      value &= ~(URV(7) << 6);        // Clear cause field (starts at bit 6).
      value |= URV(cause) << 6;       // Set cause field
      value = (value >> 2) << 2;      // Clear privilege mode bits.
      value |= URV(privMode_) & 0x3;  // Set privelge mode bits.

      if (nmiPending_)
	value |= URV(1) << 3;    // Set nmip bit.
      csRegs_.poke(CsrNumber::DCSR, value);

      csRegs_.poke(CsrNumber::DPC, pc);
    }
}


template <typename URV>
void
Hart<URV>::enterDebugMode(URV pc, bool /*force*/)
{
  // This method is used by the test-bench to make the simulator
  // follow it into debug-halt or debug-stop mode. Do nothing if the
  // simulator got into debug mode on its own.
  if (debugMode_)
    return;   // Already in debug mode.

  if (debugStepMode_)
    std::cerr << "Error: Enter-debug command finds hart in debug-step mode.\n";

  debugStepMode_ = false;
  debugMode_ = false;

  enterDebugMode_(DebugModeCause::DEBUGGER, pc);
}


template <typename URV>
void
Hart<URV>::exitDebugMode()
{
  if (not debugMode_)
    {
      std::cerr << "Error: Bench sent exit debug while not in debug mode.\n";
      return;
    }

  cancelLr();  // Exiting debug modes loses LR reservation.

  peekCsr(CsrNumber::DPC, pc_);

  // If in debug-step go to debug-halt. If in debug-halt go to normal
  // or debug-step based on step-bit in DCSR.
  if (debugStepMode_)
    debugStepMode_ = false;
  else
    {
      if (dcsrStep_)
	debugStepMode_ = true;
      else
        {
          debugMode_ = false;
        }
    }

  // If pending nmi bit is set in dcsr, set pending nmi in the hart
  // object.
  URV dcsrVal = 0;
  if (not peekCsr(CsrNumber::DCSR, dcsrVal))
    std::cerr << "Error: Failed to read DCSR in exit debug.\n";

  if ((dcsrVal >> 3) & 1)
    setPendingNmi(nmiCause_);
}


template <typename URV>
void
Hart<URV>::execBlt(const DecodedInst* di)
{
  SRV v1 = intRegs_.read(di->op0()),  v2 = intRegs_.read(di->op1());
  if (v1 < v2)
    {
      URV nextPc = (currPc_ + di->op2As<SRV>()) & ~URV(1);
      if (not isRvc() and (nextPc & 3))
	{
	  // Target must be word aligned if C is off.
	  initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
	}
      else
	{
	  setPc(nextPc);
	  lastBranchTaken_ = true;
	}
    }
}


template <typename URV>
void
Hart<URV>::execBltu(const DecodedInst* di)
{
  URV v1 = intRegs_.read(di->op0()),  v2 = intRegs_.read(di->op1());
  if (v1 < v2)
    {
      URV nextPc = (currPc_ + di->op2As<SRV>()) & ~URV(1);
      if (not isRvc() and (nextPc & 3))
	{
	  // Target must be word aligned if C is off.
	  initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
	}
      else
	{
	  setPc(nextPc);
	  lastBranchTaken_ = true;
	}
    }
}


template <typename URV>
void
Hart<URV>::execBge(const DecodedInst* di)
{
  SRV v1 = intRegs_.read(di->op0()),  v2 = intRegs_.read(di->op1());
  if (v1 >= v2)
    {
      URV nextPc = (currPc_ + di->op2As<SRV>()) & ~URV(1);
      if (not isRvc() and (nextPc & 3))
	{
	  // Target must be word aligned if C is off.
	  initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
	}
      else
	{
	  setPc(nextPc);
	  lastBranchTaken_ = true;
	}
    }
}


template <typename URV>
void
Hart<URV>::execBgeu(const DecodedInst* di)
{
  URV v1 = intRegs_.read(di->op0()),  v2 = intRegs_.read(di->op1());
  if (v1 >= v2)
    {
      URV nextPc = (currPc_ + di->op2As<SRV>()) & ~URV(1);
      if (not isRvc() and (nextPc & 3))
	{
	  // Target must be word aligned if C is off.
	  initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
	}
      else
	{
	  setPc(nextPc);
	  lastBranchTaken_ = true;
	}
    }
}


template <typename URV>
void
Hart<URV>::execJalr(const DecodedInst* di)
{
  URV temp = pc_;  // pc has the address of the instruction after jalr

  URV nextPc = (intRegs_.read(di->op1()) + di->op2As<SRV>()) & ~URV(1);
  if (not isRvc() and (nextPc & 3))
    {
      // Target must be word aligned if C is off.
      initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
    }
  else
    {
      setPc(nextPc);
      intRegs_.write(di->op0(), temp);
      lastBranchTaken_ = true;
    }
}


template <typename URV>
void
Hart<URV>::execJal(const DecodedInst* di)
{
  URV nextPc = (currPc_ + SRV(int32_t(di->op1()))) & ~URV(1);
  if (not isRvc() and (nextPc & 3))
    {
      // Target must be word aligned if C is off.
      initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
    }
  else
    {
      intRegs_.write(di->op0(), pc_);
      setPc(nextPc);
      lastBranchTaken_ = true;
    }
}


template <typename URV>
void
Hart<URV>::execAuipc(const DecodedInst* di)
{
  intRegs_.write(di->op0(), currPc_ + SRV(int32_t(di->op1())));
}


template <typename URV>
inline
bool
Hart<URV>::checkShiftImmediate(const DecodedInst* di, URV imm)
{
  bool bad = isRv64()? imm > 63 : imm > 31;

  if (bad)
    {
      illegalInst(di);
      return false;
    }
  return true;
}


template <typename URV>
void
Hart<URV>::execSlli(const DecodedInst* di)
{
  URV amount = di->op2();
  if (not checkShiftImmediate(di, amount))
    return;

  URV v = intRegs_.read(di->op1()) << amount;
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSlti(const DecodedInst* di)
{
  SRV imm = di->op2As<SRV>();
  URV v = SRV(intRegs_.read(di->op1())) < imm ? 1 : 0;
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSltiu(const DecodedInst* di)
{
  URV imm = di->op2As<SRV>();   // We sign extend then use as unsigned.
  URV v = URV(intRegs_.read(di->op1())) < imm ? 1 : 0;
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execXori(const DecodedInst* di)
{
  URV v = intRegs_.read(di->op1()) ^ di->op2As<SRV>();
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSrli(const DecodedInst* di)
{
  URV amount(di->op2());
  if (not checkShiftImmediate(di, amount))
    return;

  URV v = intRegs_.read(di->op1());
  v >>= amount;
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSrai(const DecodedInst* di)
{
  uint32_t amount(di->op2());
  if (not checkShiftImmediate(di, amount))
    return;

  URV v = SRV(intRegs_.read(di->op1())) >> amount;
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execOri(const DecodedInst* di)
{
  URV v = intRegs_.read(di->op1()) | di->op2As<SRV>();
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSub(const DecodedInst* di)
{
  URV v = intRegs_.read(di->op1()) - intRegs_.read(di->op2());
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSll(const DecodedInst* di)
{
  URV mask = shiftMask();
  URV v = intRegs_.read(di->op1()) << (intRegs_.read(di->op2()) & mask);
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSlt(const DecodedInst* di)
{
  SRV v1 = intRegs_.read(di->op1());
  SRV v2 = intRegs_.read(di->op2());
  URV v = v1 < v2 ? 1 : 0;
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSltu(const DecodedInst* di)
{
  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());
  URV v = v1 < v2 ? 1 : 0;
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execXor(const DecodedInst* di)
{
  URV v = intRegs_.read(di->op1()) ^ intRegs_.read(di->op2());
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSrl(const DecodedInst* di)
{
  URV mask = shiftMask();
  URV v = intRegs_.read(di->op1());
  v >>= (intRegs_.read(di->op2()) & mask);
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSra(const DecodedInst* di)
{
  URV mask = shiftMask();
  URV v = SRV(intRegs_.read(di->op1())) >> (intRegs_.read(di->op2()) & mask);
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execOr(const DecodedInst* di)
{
  URV v = intRegs_.read(di->op1()) | intRegs_.read(di->op2());
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execAnd(const DecodedInst* di)
{
  URV v = intRegs_.read(di->op1()) & intRegs_.read(di->op2());
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execFence(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execFence_tso(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execFencei(const DecodedInst*)
{
  // invalidateDecodeCache();  // No need for this. We invalidate on each write.
}


template <typename URV>
void
Hart<URV>::execEcall(const DecodedInst*)
{
  if (triggerTripped_)
    return;

  if (newlib_ or linux_ or syscallSlam_)
    {
      URV a0 = syscall_.emulate();
      intRegs_.write(RegA0, a0);
      if (not syscallSlam_)
        return;
    }

  if (privMode_ == PrivilegeMode::Machine)
    initiateException(ExceptionCause::M_ENV_CALL, currPc_, 0);
  else if (privMode_ == PrivilegeMode::Supervisor)
    initiateException(ExceptionCause::S_ENV_CALL, currPc_, 0);
  else if (privMode_ == PrivilegeMode::User)
    initiateException(ExceptionCause::U_ENV_CALL, currPc_, 0);
  else
    assert(0 and "Invalid privilege mode in execEcall");
}


template <typename URV>
void
Hart<URV>::execEbreak(const DecodedInst*)
{
  if (triggerTripped_)
    return;

  if (enableGdb_)
    {
      setPc(currPc_);
      handleExceptionForGdb(*this, gdbInputFd_);
      return;
    }

  // If in machine/supervisor/user mode and DCSR bit ebreakm/s/u is
  // set, then enter debug mode.
  URV dcsrVal = 0;
  if (peekCsr(CsrNumber::DCSR, dcsrVal))
    {
      bool ebm = (dcsrVal >> 15) & 1;
      bool ebs = (dcsrVal >> 13) & 1;
      bool ebu = (dcsrVal >> 12) & 1;

      bool debug = ( (ebm and privMode_ == PrivilegeMode::Machine) or
                     (ebs and privMode_ == PrivilegeMode::Supervisor) or
                     (ebu and privMode_ == PrivilegeMode::User) );

      if (debug)
        {
          // The documentation (RISCV external debug support) does
          // not say whether or not we set EPC and MTVAL.
          enterDebugMode_(DebugModeCause::EBREAK, currPc_);
          ebreakInstDebug_ = true;
          recordCsrWrite(CsrNumber::DCSR);
          return;
        }
    }

  URV savedPc = currPc_;  // Goes into MEPC.
  URV trapInfo = currPc_;  // Goes into MTVAL.

  auto cause = ExceptionCause::BREAKP;
  initiateException(cause, savedPc, trapInfo);
}


template <typename URV>
void
Hart<URV>::execSfence_vma(const DecodedInst* di)
{
  if (not isRvs() or privMode_ < PrivilegeMode::Supervisor)
    {
      illegalInst(di);
      return;
    }

  URV status = csRegs_.peekMstatus();
  MstatusFields<URV> fields(status);
  if (fields.bits_.TVM and privMode_ == PrivilegeMode::Supervisor)
    {
      illegalInst(di);
      return;
    }

  // Invalidate whole TLB. This is overkill. TBD FIX: Improve.
  if (di->op1() == 0 and di->op2() == 0)
    virtMem_.tlb_.invalidate();
  else if (di->op1() == 0 and di->op2() != 0)
    {
      URV asid = intRegs_.read(di->op2());
      virtMem_.tlb_.invalidateAsid(asid);
    }
  else if (di->op1() != 0 and di->op2() == 0)
    {
      URV addr = intRegs_.read(di->op1());
      uint64_t vpn = virtMem_.pageNumber(addr);
      virtMem_.tlb_.invalidateVirtualPage(vpn);
    }
  else
    {
      URV addr = intRegs_.read(di->op1());
      uint64_t vpn = virtMem_.pageNumber(addr);
      URV asid = intRegs_.read(di->op2());
      virtMem_.tlb_.invalidateVirtualPage(vpn, asid);
    }

  // std::cerr << "sfence.vma " << di->op1() << ' ' << di->op2() << '\n';
  if (di->op1() == 0)
    invalidateDecodeCache();
  else
    {
      uint64_t va = intRegs_.read(di->op1());
      uint64_t pageStart = virtMem_.pageStartAddress(va);
      uint64_t last = pageStart + virtMem_.pageSize();
      for (uint64_t addr = pageStart; addr < last; addr += 4)
        invalidateDecodeCache(addr, 4);
    }
}


template <typename URV>
void
Hart<URV>::execSinval_vma(const DecodedInst* di)
{
  execSfence_vma(di);
}


template <typename URV>
void
Hart<URV>::execSfence_w_inval(const DecodedInst* di)
{
  if (not isRvs() or not isRvsvinval() or privMode_ < PrivilegeMode::Supervisor)
    {
      illegalInst(di);
      return;
    }
}


template <typename URV>
void
Hart<URV>::execSfence_inval_ir(const DecodedInst* di)
{
  if (not isRvs() or not isRvsvinval() or privMode_ < PrivilegeMode::Supervisor)
    {
      illegalInst(di);
      return;
    }
}


template <typename URV>
void
Hart<URV>::execMret(const DecodedInst* di)
{
  if (privMode_ < PrivilegeMode::Machine)
    {
      illegalInst(di);
      return;
    }

  if (triggerTripped_)
    return;

  cancelLr(); // Clear LR reservation (if any).

  // Restore privilege mode and interrupt enable by getting
  // current value of MSTATUS, ...
  URV value = csRegs_.peekMstatus();

  // ... updating/unpacking its fields,
  MstatusFields<URV> fields(value);
  PrivilegeMode savedMode = PrivilegeMode(fields.bits_.MPP);
  fields.bits_.MIE = fields.bits_.MPIE;
  fields.bits_.MPP = unsigned(PrivilegeMode::User);
  fields.bits_.MPIE = 1;
  if (savedMode != PrivilegeMode::Machine)
    fields.bits_.MPRV = 0;

  // ... and putting it back
  if (not csRegs_.write(CsrNumber::MSTATUS, privMode_, fields.value_))
    assert(0 and "Failed to write MSTATUS register\n");
  updateCachedMstatusFields();

  // Restore program counter from MEPC.
  URV epc;
  if (not csRegs_.read(CsrNumber::MEPC, privMode_, epc))
    illegalInst(di);
  setPc(epc);
      
  // Update privilege mode.
  privMode_ = savedMode;
}


template <typename URV>
void
Hart<URV>::execSret(const DecodedInst* di)
{
  if (not isRvs())
    {
      illegalInst(di);
      return;
    }

  if (privMode_ < PrivilegeMode::Supervisor)
    {
      illegalInst(di);
      return;
    }

  // If MSTATUS.TSR is 1 then sret is illegal in supervisor mode.
  URV mstatus = csRegs_.peekMstatus();
  MstatusFields<URV> mfields(mstatus);
  if (mfields.bits_.TSR and privMode_ == PrivilegeMode::Supervisor)
    {
      illegalInst(di);
      return;
    }

  if (triggerTripped_)
    return;

  cancelLr(); // Clear LR reservation (if any).

  // Restore privilege mode and interrupt enable by getting
  // current value of SSTATUS, ...
  URV value = 0;
  if (not csRegs_.read(CsrNumber::SSTATUS, privMode_, value))
    {
      illegalInst(di);
      return;
    }

  // ... updating/unpacking its fields,
  MstatusFields<URV> fields(value);
  PrivilegeMode savedMode = fields.bits_.SPP? PrivilegeMode::Supervisor :
    PrivilegeMode::User;
  fields.bits_.SIE = fields.bits_.SPIE;
  fields.bits_.SPP = 0;
  fields.bits_.SPIE = 1;
  if (savedMode != PrivilegeMode::Machine)
    fields.bits_.MPRV = 0;

  // ... and putting it back
  if (not csRegs_.write(CsrNumber::SSTATUS, privMode_, fields.value_))
    {
      illegalInst(di);
      return;
    }
  updateCachedMstatusFields();

  // Restore program counter from SEPC.
  URV epc;
  if (not csRegs_.read(CsrNumber::SEPC, privMode_, epc))
    {
      illegalInst(di);
      return;
    }
  setPc(epc);

  // Update privilege mode.
  privMode_ = savedMode;
}


template <typename URV>
void
Hart<URV>::execUret(const DecodedInst* di)
{
  if (not isRvu())
    {
      illegalInst(di);
      return;
    }

  if (privMode_ != PrivilegeMode::User)
    {
      illegalInst(di);
      return;
    }

  if (triggerTripped_)
    return;

  // Restore privilege mode and interrupt enable by getting
  // current value of MSTATUS, ...
  URV value = 0;
  if (not csRegs_.read(CsrNumber::USTATUS, privMode_, value))
    {
      illegalInst(di);
      return;
    }

  // ... updating/unpacking its fields,
  MstatusFields<URV> fields(value);
  fields.bits_.UIE = fields.bits_.UPIE;
  fields.bits_.UPIE = 1;

  // ... and putting it back
  if (not csRegs_.write(CsrNumber::USTATUS, privMode_, fields.value_))
    {
      illegalInst(di);
      return;
    }
  updateCachedMstatusFields();

  // Restore program counter from UEPC.
  URV epc;
  if (not csRegs_.read(CsrNumber::UEPC, privMode_, epc))
    {
      illegalInst(di);
      return;
    }
  setPc(epc);
}


template <typename URV>
void
Hart<URV>::execWfi(const DecodedInst*)
{
  return;   // Currently implemented as a no-op.
}


template <typename URV>
void
Hart<URV>::execDret(const DecodedInst* di)
{
  auto dcsr = csRegs_.getImplementedCsr(CsrNumber::DCSR);
  auto dpc = csRegs_.getImplementedCsr(CsrNumber::DPC);
  if (not dcsr or not dpc)
    {
      illegalInst(di);
      return;
    }

  // The dret instruction is only valid if debug is on. However, if dcsr is
  // not marked debug-only, then allow dret in any mode.
  if (not debugMode_ and dcsr->isDebug())
    {
      illegalInst(di);
      return;
    }

  debugMode_ = false;

  URV value = 0;
  peekCsr(CsrNumber::DPC, value);
  setPc(value);

  value = 0;
  peekCsr(CsrNumber::DCSR, value);
  unsigned mode = value & 3;
  PrivilegeMode pm = PrivilegeMode{mode};
  setPrivilegeMode(pm);
}


template <typename URV>
bool
Hart<URV>::doCsrRead(const DecodedInst* di, CsrNumber csr, URV& value)
{
  if (csr == CsrNumber::SATP and privMode_ == PrivilegeMode::Supervisor)
    {
      URV status = csRegs_.peekMstatus();
      MstatusFields<URV> fields(status);
      if (fields.bits_.TVM)
        {
          illegalInst(di);
          return false;
        }
    }

  if (csr == CsrNumber::FCSR or csr == CsrNumber::FRM or csr == CsrNumber::FFLAGS)
    if (not isFpLegal())
      {
        illegalInst(di);
        return false;
      }

  if (csRegs_.read(csr, privMode_, value))
    return true;

  illegalInst(di);
  return false;
}


template <typename URV>
bool
Hart<URV>::isCsrWriteable(CsrNumber csr) const
{
  if (not csRegs_.isWriteable(csr, privMode_))
    return false;

  if (csr == CsrNumber::SATP and privMode_ == PrivilegeMode::Supervisor)
    {
      URV mstatus = csRegs_.peekMstatus();
      MstatusFields<URV> fields(mstatus);
      if (fields.bits_.TVM)
        return false;
    }

  if (csr == CsrNumber::FCSR or csr == CsrNumber::FRM or csr == CsrNumber::FFLAGS)
    if (not isFpLegal())
      return false;

  if (csr == CsrNumber::VSTART or csr == CsrNumber::VXSAT or csr == CsrNumber::VXRM or
      csr == CsrNumber::VCSR or csr == CsrNumber::VL or csr == CsrNumber::VTYPE or
      csr == CsrNumber::VLENB)
    if (not isVecLegal())
      return false;

  return true;
}



template <typename URV>
void
Hart<URV>::doCsrWrite(const DecodedInst* di, CsrNumber csr, URV val,
                      unsigned intReg, URV intRegVal)
{
  if (not isCsrWriteable(csr))
    {
      illegalInst(di);
      return;
    }

  // Make auto-increment happen before CSR write for minstret and cycle.
  if (csr == CsrNumber::MINSTRET or csr == CsrNumber::MINSTRETH)
    if (minstretEnabled())
      retiredInsts_++;
  if (csr == CsrNumber::MCYCLE or csr == CsrNumber::MCYCLEH)
    cycleCount_++;

  updatePerformanceCountersForCsr(*di);

  // Update CSR.
  csRegs_.write(csr, privMode_, val);

  // Update integer register.
  intRegs_.write(intReg, intRegVal);

  postCsrUpdate(csr, val);

  // Csr was written. If it was minstret, compensate for
  // auto-increment that will be done by run, runUntilAddress or
  // singleStep method.
  if (csr == CsrNumber::MINSTRET or csr == CsrNumber::MINSTRETH)
    if (minstretEnabled())
      retiredInsts_--;

  // Same for mcycle.
  if (csr == CsrNumber::MCYCLE or csr == CsrNumber::MCYCLEH)
    cycleCount_--;
}


// Set control and status register csr (op2) to value of register rs1
// (op1) and save its original value in register rd (op0).
template <typename URV>
void
Hart<URV>::execCsrrw(const DecodedInst* di)
{
  if (triggerTripped_)
    return;

  CsrNumber csr = CsrNumber(di->op2());

  if (preCsrInst_)
    preCsrInst_(hartIx_, csr);

  URV prev = 0;
  if (not doCsrRead(di, csr, prev))
    {
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  URV next = intRegs_.read(di->op1());

  doCsrWrite(di, csr, next, di->op0(), prev);

  if (postCsrInst_)
    postCsrInst_(hartIx_, csr);
}


template <typename URV>
void
Hart<URV>::execCsrrs(const DecodedInst* di)
{
  if (triggerTripped_)
    return;

  CsrNumber csr = CsrNumber(di->op2());

  if (preCsrInst_)
    preCsrInst_(hartIx_, csr);

  URV prev = 0;
  if (not doCsrRead(di, csr, prev))
    {
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  URV next = prev | intRegs_.read(di->op1());
  if (di->op1() == 0)
    {
      updatePerformanceCountersForCsr(*di);
      intRegs_.write(di->op0(), prev);
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  doCsrWrite(di, csr, next, di->op0(), prev);

  if (postCsrInst_)
    postCsrInst_(hartIx_, csr);
}


template <typename URV>
void
Hart<URV>::execCsrrc(const DecodedInst* di)
{
  if (triggerTripped_)
    return;

  CsrNumber csr = CsrNumber(di->op2());

  if (preCsrInst_)
    preCsrInst_(hartIx_, csr);

  URV prev = 0;
  if (not doCsrRead(di, csr, prev))
    {
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  URV next = prev & (~ intRegs_.read(di->op1()));
  if (di->op1() == 0)
    {
      updatePerformanceCountersForCsr(*di);
      intRegs_.write(di->op0(), prev);
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  doCsrWrite(di, csr, next, di->op0(), prev);

  if (postCsrInst_)
    postCsrInst_(hartIx_, csr);
}


template <typename URV>
void
Hart<URV>::execCsrrwi(const DecodedInst* di)
{
  if (triggerTripped_)
    return;

  CsrNumber csr = CsrNumber(di->op2());

  if (preCsrInst_)
    preCsrInst_(hartIx_, csr);

  URV prev = 0;
  if (di->op0() != 0)
    if (not doCsrRead(di, csr, prev))
      {
        if (postCsrInst_)
          postCsrInst_(hartIx_, csr);
        return;
      }

  doCsrWrite(di, csr, di->op1(), di->op0(), prev);

  if (postCsrInst_)
    postCsrInst_(hartIx_, csr);
}


template <typename URV>
void
Hart<URV>::execCsrrsi(const DecodedInst* di)
{
  if (triggerTripped_)
    return;

  CsrNumber csr = CsrNumber(di->op2());

  if (preCsrInst_)
    preCsrInst_(hartIx_, csr);

  URV imm = di->op1();

  URV prev = 0;
  if (not doCsrRead(di, csr, prev))
    {
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  URV next = prev | imm;
  if (imm == 0)
    {
      updatePerformanceCountersForCsr(*di);
      intRegs_.write(di->op0(), prev);
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  doCsrWrite(di, csr, next, di->op0(), prev);

  if (postCsrInst_)
    postCsrInst_(hartIx_, csr);
}


template <typename URV>
void
Hart<URV>::execCsrrci(const DecodedInst* di)
{
  if (triggerTripped_)
    return;

  CsrNumber csr = CsrNumber(di->op2());

  if (preCsrInst_)
    preCsrInst_(hartIx_, csr);

  URV imm = di->op1();

  URV prev = 0;
  if (not doCsrRead(di, csr, prev))
    {
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  URV next = prev & (~ imm);
  if (imm == 0)
    {
      updatePerformanceCountersForCsr(*di);
      intRegs_.write(di->op0(), prev);
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  doCsrWrite(di, csr, next, di->op0(), prev);

  if (postCsrInst_)
    postCsrInst_(hartIx_, csr);
}


template <typename URV>
void
Hart<URV>::execLb(const DecodedInst* di)
{
  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();

  uint64_t data = 0;
  if (load<int8_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
void
Hart<URV>::execLbu(const DecodedInst* di)
{
  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();

  uint64_t data = 0;
  if (load<uint8_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
void
Hart<URV>::execLhu(const DecodedInst* di)
{
  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();

  uint64_t data = 0;
  if (load<uint16_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
ExceptionCause
Hart<URV>::determineStoreException(uint64_t& addr1, uint64_t& addr2,
				   unsigned stSize)
{
  uint64_t va1 = URV(addr1); // Virtual address. Truncate to 32-bits in 32-bit mode.
  uint64_t va2 = va1;        // Used if crossing page boundary
  addr1 = va1;
  addr2 = va2;

  uint64_t alignMask = stSize - 1;
  bool misal = addr1 & alignMask;
  misalignedLdSt_ = misal;

  ExceptionCause cause = ExceptionCause::NONE;
  if (misal)
    {
      cause = determineMisalStoreException(addr1, stSize);
      if (cause == ExceptionCause::NONE)
        return cause;
      va2 = (va1 + stSize) & ~alignMask;
    }

  // Address translation
  if (isRvs())
    {
      PrivilegeMode mode = mstatusMprv_? mstatusMpp_ : privMode_;
      if (mode != PrivilegeMode::Machine)
        {
          cause = virtMem_.translateForStore2(va1, stSize, mode, addr1, addr2);
          if (cause != ExceptionCause::NONE)
            return cause;
        }
    }

  // Physical memory protection. Assuming grain size is >= 8.
  if (pmpEnabled_)
    {
      Pmp pmp = pmpManager_.accessPmp(addr1);
      if (not pmp.isWrite(privMode_, mstatusMpp_, mstatusMprv_))
	{
	  addr1 = va1;
	  return ExceptionCause::STORE_ACC_FAULT;
	}
      if (misal or addr1 != addr2)
	{
	  uint64_t aligned = addr1 & ~alignMask;
	  uint64_t next = addr1 == addr2? aligned + stSize : addr2;
  	  pmp = pmpManager_.accessPmp(next);
	  if (not pmp.isRead(privMode_, mstatusMpp_, mstatusMprv_))
	    {
	      addr1 = va2;
	      return ExceptionCause::LOAD_ACC_FAULT;
	    }
	}
    }

  if (not misal)
    {
      if (not memory_.checkWrite(addr1, stSize))
	{
	  addr1 = va1;
	  return ExceptionCause::STORE_ACC_FAULT;
	}
    }
  else
    {
      uint64_t aligned = addr1 & ~alignMask;
      if (not memory_.checkWrite(aligned, stSize))
	{
	  addr1 = va1;
	  return ExceptionCause::STORE_ACC_FAULT;
	}
      uint64_t next = addr1 == addr2? aligned + stSize : addr2;
      if (not memory_.checkRead(next, stSize))
	{
	  addr1 = va2;
	  return ExceptionCause::STORE_ACC_FAULT;
	}
    }

  return ExceptionCause::NONE;
}


template <typename URV>
void
Hart<URV>::execSb(const DecodedInst* di)
{
  uint32_t rs1 = di->op1();
  URV base = intRegs_.read(rs1);
  URV addr = base + di->op2As<SRV>();
  uint8_t value = uint8_t(intRegs_.read(di->op0()));
  store<uint8_t>(addr, value);
}


template <typename URV>
void
Hart<URV>::execSh(const DecodedInst* di)
{
  uint32_t rs1 = di->op1();
  URV base = intRegs_.read(rs1);
  URV addr = base + di->op2As<SRV>();
  uint16_t value = uint16_t(intRegs_.read(di->op0()));
  store<uint16_t>(addr, value);
}


template<typename URV>
void
Hart<URV>::execMul(const DecodedInst* di)
{
  if (not isRvm())
    {
      illegalInst(di);
      return;
    }

  SRV a = intRegs_.read(di->op1());
  SRV b = intRegs_.read(di->op2());

  SRV c = a * b;
  intRegs_.write(di->op0(), c);
}


namespace WdRiscv
{

  template<>
  void
  Hart<uint32_t>::execMulh(const DecodedInst* di)
  {
    if (not isRvm())
      {
	illegalInst(di);
	return;
      }

    int64_t a = int32_t(intRegs_.read(di->op1()));  // sign extend.
    int64_t b = int32_t(intRegs_.read(di->op2()));
    int64_t c = a * b;
    int32_t high = static_cast<int32_t>(c >> 32);

    intRegs_.write(di->op0(), high);
  }


  template <>
  void
  Hart<uint32_t>::execMulhsu(const DecodedInst* di)
  {
    if (not isRvm())
      {
	illegalInst(di);
	return;
      }

    int64_t a = int32_t(intRegs_.read(di->op1()));
    uint64_t b = uint32_t(intRegs_.read(di->op2()));
    int64_t c = a * b;
    int32_t high = static_cast<int32_t>(c >> 32);

    intRegs_.write(di->op0(), high);
  }


  template <>
  void
  Hart<uint32_t>::execMulhu(const DecodedInst* di)
  {
    if (not isRvm())
      {
	illegalInst(di);
	return;
      }

    uint64_t a = uint32_t(intRegs_.read(di->op1()));
    uint64_t b = uint32_t(intRegs_.read(di->op2()));
    uint64_t c = a * b;
    uint32_t high = static_cast<uint32_t>(c >> 32);

    intRegs_.write(di->op0(), high);
  }


  template<>
  void
  Hart<uint64_t>::execMulh(const DecodedInst* di)
  {
    if (not isRvm())
      {
	illegalInst(di);
	return;
      }

    Int128 a = int64_t(intRegs_.read(di->op1()));  // sign extend.
    Int128 b = int64_t(intRegs_.read(di->op2()));
    Int128 c = a * b;
    int64_t high = static_cast<int64_t>(c >> 64);

    intRegs_.write(di->op0(), high);
  }


  template <>
  void
  Hart<uint64_t>::execMulhsu(const DecodedInst* di)
  {
    if (not isRvm())
      {
	illegalInst(di);
	return;
      }

    Int128 a = int64_t(intRegs_.read(di->op1()));
    Int128 b = intRegs_.read(di->op2());
    Int128 c = a * b;
    int64_t high = static_cast<int64_t>(c >> 64);

    intRegs_.write(di->op0(), high);
  }


  template <>
  void
  Hart<uint64_t>::execMulhu(const DecodedInst* di)
  {
    if (not isRvm())
      {
	illegalInst(di);
	return;
      }

    Uint128 a = intRegs_.read(di->op1());
    Uint128 b = intRegs_.read(di->op2());
    Uint128 c = a * b;
    uint64_t high = static_cast<uint64_t>(c >> 64);

    intRegs_.write(di->op0(), high);
  }

}


template <typename URV>
void
Hart<URV>::execDiv(const DecodedInst* di)
{
  if (not isRvm())
    {
      illegalInst(di);
      return;
    }

  SRV a = intRegs_.read(di->op1());
  SRV b = intRegs_.read(di->op2());
  SRV c = -1;   // Divide by zero result
  if (b != 0)
    {
      SRV minInt = SRV(1) << (mxlen_ - 1);
      if (a == minInt and b == -1)
	c = a;
      else
	c = a / b;  // Per spec: User-Level ISA, Version 2.3, Section 6.2
    }

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  intRegs_.write(di->op0(), c);
}


template <typename URV>
void
Hart<URV>::execDivu(const DecodedInst* di)
{
  if (not isRvm())
    {
      illegalInst(di);
      return;
    }

  URV a = intRegs_.read(di->op1());
  URV b = intRegs_.read(di->op2());
  URV c = ~ URV(0);  // Divide by zero result.
  if (b != 0)
    c = a / b;

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  intRegs_.write(di->op0(), c);
}


// Remainder instruction.
template <typename URV>
void
Hart<URV>::execRem(const DecodedInst* di)
{
  if (not isRvm())
    {
      illegalInst(di);
      return;
    }

  SRV a = intRegs_.read(di->op1());
  SRV b = intRegs_.read(di->op2());
  SRV c = a;  // Divide by zero remainder.
  if (b != 0)
    {
      SRV minInt = SRV(1) << (mxlen_ - 1);
      if (a == minInt and b == -1)
	c = 0;   // Per spec: User-Level ISA, Version 2.3, Section 6.2
      else
	c = a % b;
    }

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  intRegs_.write(di->op0(), c);
}


// Unsigned remainder instruction.
template <typename URV>
void
Hart<URV>::execRemu(const DecodedInst* di)
{
  if (not isRvm())
    {
      illegalInst(di);
      return;
    }

  URV a = intRegs_.read(di->op1());
  URV b = intRegs_.read(di->op2());
  URV c = a;  // Divide by zero remainder.
  if (b != 0)
    c = a % b;

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  intRegs_.write(di->op0(), c);
}


template <typename URV>
void
Hart<URV>::execLwu(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();

  uint64_t data = 0;
  if (load<uint32_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <>
void
Hart<uint32_t>::execLd(const DecodedInst* di)
{
  illegalInst(di);
}


template <>
void
Hart<uint64_t>::execLd(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }
  uint64_t base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();

  uint64_t data = 0;
  if (load<uint64_t>(virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
void
Hart<URV>::execSd(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  unsigned rs1 = di->op1();

  URV base = intRegs_.read(rs1);
  URV addr = base + di->op2As<SRV>();
  URV value = intRegs_.read(di->op0());

  store<uint64_t>(addr, value);
}


template <typename URV>
void
Hart<URV>::execSlliw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t amount(di->op2());

  if (amount > 0x1f)
    {
      illegalInst(di);   // Bits 5 and 6 of immeidate must be zero.
      return;
    }

  int32_t word = int32_t(intRegs_.read(di->op1()));
  word <<= amount;

  SRV value = word; // Sign extend to 64-bit.
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execSrliw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t amount(di->op2());

  if (amount > 0x1f)
    {
      illegalInst(di);   // Bits 5 and 6 of immediate must be zero.
      return;
    }

  uint32_t word = uint32_t(intRegs_.read(di->op1()));
  word >>= amount;

  SRV value = int32_t(word); // Sign extend to 64-bit.
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execSraiw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t amount(di->op2());

  if (amount > 0x1f)
    {
      illegalInst(di);   // Bits 5 and 6 of immeddiate must be zero.
      return;
    }

  int32_t word = int32_t(intRegs_.read(di->op1()));
  word >>= amount;

  SRV value = word; // Sign extend to 64-bit.
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execAddiw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  int32_t word = int32_t(intRegs_.read(di->op1()));
  word += di->op2As<int32_t>();
  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execAddw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  int32_t word = int32_t(intRegs_.read(di->op1()) + intRegs_.read(di->op2()));
  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execSubw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  int32_t word = int32_t(intRegs_.read(di->op1()) - intRegs_.read(di->op2()));
  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execSllw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t shift = intRegs_.read(di->op2()) & 0x1f;
  int32_t word = int32_t(intRegs_.read(di->op1()) << shift);
  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execSrlw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t word = uint32_t(intRegs_.read(di->op1()));
  uint32_t shift = uint32_t(intRegs_.read(di->op2()) & 0x1f);
  word >>= shift;
  SRV value = int32_t(word);  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execSraw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  int32_t word = int32_t(intRegs_.read(di->op1()));
  uint32_t shift = uint32_t(intRegs_.read(di->op2()) & 0x1f);
  word >>= shift;
  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execMulw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  int32_t word1 = int32_t(intRegs_.read(di->op1()));
  int32_t word2 = int32_t(intRegs_.read(di->op2()));
  int32_t word = int32_t(word1 * word2);
  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execDivw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  int32_t word1 = int32_t(intRegs_.read(di->op1()));
  int32_t word2 = int32_t(intRegs_.read(di->op2()));

  int32_t word = -1;  // Divide by zero result
  if (word2 != 0)
    {
      int32_t minInt = int32_t(1) << 31;
      if (word1 == minInt and word2 == -1)
	word = word1;
      else
	word = word1 / word2;
    }

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execDivuw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t word1 = uint32_t(intRegs_.read(di->op1()));
  uint32_t word2 = uint32_t(intRegs_.read(di->op2()));

  uint32_t word = ~uint32_t(0);  // Divide by zero result.
  if (word2 != 0)
    word = word1 / word2;

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  URV value = SRV(int32_t(word));  // Sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execRemw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  int32_t word1 = int32_t(intRegs_.read(di->op1()));
  int32_t word2 = int32_t(intRegs_.read(di->op2()));

  int32_t word = word1;  // Divide by zero remainder
  if (word2 != 0)
    {
      int32_t minInt = int32_t(1) << 31;
      if (word1 == minInt and word2 == -1)
	word = 0;   // Per spec: User-Level ISA, Version 2.3, Section 6.2
      else
	word = word1 % word2;
    }

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execRemuw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t word1 = uint32_t(intRegs_.read(di->op1()));
  uint32_t word2 = uint32_t(intRegs_.read(di->op2()));

  uint32_t word = word1;  // Divide by zero remainder
  if (word2 != 0)
    word = word1 % word2;

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  URV value = SRV(int32_t(word));  // Sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
inline
void
Hart<URV>::markVsDirty()
{
  if (mstatusVs_ == FpFs::Dirty)
    return;

  URV val = csRegs_.peekMstatus();
  MstatusFields<URV> fields(val);
  fields.bits_.VS = unsigned(FpFs::Dirty);
  fields.bits_.SD = 1;

  csRegs_.poke(CsrNumber::MSTATUS, fields.value_);

  URV newVal = csRegs_.peekMstatus();
  if (val != newVal)
    recordCsrWrite(CsrNumber::MSTATUS);

  updateCachedMstatusFields();
}


template
bool
WdRiscv::Hart<uint32_t>::load<uint16_t>(uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint32_t>::load<uint32_t>(uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint32_t>::load<uint64_t>(uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint64_t>::load<uint16_t>(uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint64_t>::load<uint32_t>(uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint64_t>::load<uint64_t>(uint64_t, uint64_t&);


template
bool
WdRiscv::Hart<uint32_t>::store<uint16_t>(uint32_t, uint16_t);

template
bool
WdRiscv::Hart<uint32_t>::store<uint32_t>(uint32_t, uint32_t);

template
bool
WdRiscv::Hart<uint32_t>::store<uint64_t>(uint32_t, uint64_t);

template
bool
WdRiscv::Hart<uint64_t>::store<uint16_t>(uint64_t, uint16_t);

template
bool
WdRiscv::Hart<uint64_t>::store<uint32_t>(uint64_t, uint32_t);

template
bool
WdRiscv::Hart<uint64_t>::store<uint64_t>(uint64_t, uint64_t);


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
