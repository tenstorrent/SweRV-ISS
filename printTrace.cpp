#include <iostream>
#include <sstream>
#include "Hart.hpp"
#include "Trace.hpp"

using namespace WdRiscv;

static std::mutex printInstTraceMutex;

extern void (*tracerExtension)(void*);


static
const char*
privilegeModeToStr(bool vm, PrivilegeMode pm)
{
  if (not vm)
    {
      if (pm == PrivilegeMode::Machine)    return "M";
      if (pm == PrivilegeMode::Supervisor) return "S";
      if (pm == PrivilegeMode::User)       return "U";
    }
  else
    {
      if (pm == PrivilegeMode::Supervisor) return "VS";
      if (pm == PrivilegeMode::User) return "VU";
    }
  return "?";
}


template <typename URV>
void
formatVecInstTrace(FILE* out, uint64_t tag, unsigned hartId,
                   bool vm, PrivilegeMode pm,
		   URV currPc, const char* opcode, unsigned vecReg,
		   const uint8_t* data, unsigned byteCount, const char* assembly);


template <>
void
formatVecInstTrace<uint32_t>(FILE* out, uint64_t tag, unsigned hartId,
			     bool vm, PrivilegeMode pm,
			     uint32_t currPc, const char* opcode,
			     unsigned vecReg, const uint8_t* data,
			     unsigned byteCount, const char* assembly)
{
  const char* pmStr = privilegeModeToStr(vm, pm);
  fprintf(out, "#%jd %d %2s %08x %8s v %02x ",
	  uintmax_t(tag), hartId, pmStr, currPc, opcode, vecReg);
  for (unsigned i = 0; i < byteCount; ++i)
    fprintf(out, "%02x", data[byteCount - 1 - i]);
  fprintf(out, " %s", assembly);
}


template <>
void
formatVecInstTrace<uint64_t>(FILE* out, uint64_t tag, unsigned hartId,
			     bool vm, PrivilegeMode pm,
			     uint64_t currPc, const char* opcode,
			     unsigned vecReg, const uint8_t* data,
			     unsigned byteCount, const char* assembly)
{
  const char* pmStr = privilegeModeToStr(vm, pm);
  fprintf(out, "#%jd %d %2s %016jx %8s v %02x ",
          uintmax_t(tag), hartId, pmStr, uintmax_t(currPc), opcode, vecReg);
  for (unsigned i = 0; i < byteCount; ++i)
    fprintf(out, "%02x", data[byteCount - 1 - i]);
  fprintf(out, " %s", assembly);
}


template <typename URV>
void
formatInstTrace(FILE* out, uint64_t tag, unsigned hartId,
                bool vm, PrivilegeMode pm,
		URV currPc, const char* opcode, char resource, URV addr,
		URV value, const char* assembly);

template <>
void
formatInstTrace<uint32_t>(FILE* out, uint64_t tag, unsigned hartId,
			  bool vm, PrivilegeMode pm,
                          uint32_t currPc, const char* opcode, char resource, uint32_t addr,
                          uint32_t value, const char* assembly)
{
  const char* pmStr = privilegeModeToStr(vm, pm);

  if (resource == 'r')
    {
      fprintf(out, "#%jd %d %2s %08x %8s r %02x         %08x  %s",
              uintmax_t(tag), hartId, pmStr, currPc, opcode, addr, value, assembly);
    }
  else if (resource == 'c')
    {
      if ((addr >> 16) == 0)
        fprintf(out, "#%jd %d %2s %08x %8s c %04x       %08x  %s",
                uintmax_t(tag), hartId, pmStr, currPc, opcode, addr, value, assembly);
      else
        fprintf(out, "#%jd %d %2s %08x %8s c %08x   %08x  %s",
                uintmax_t(tag), hartId, pmStr, currPc, opcode, addr, value, assembly);
    }
  else
    {
      fprintf(out, "#%jd %d %2s %08x %8s %c %08x   %08x  %s", uintmax_t(tag), hartId,
              pmStr, currPc, opcode, resource, addr, value, assembly);
    }
}


template <>
void
formatInstTrace<uint64_t>(FILE* out, uint64_t tag, unsigned hartId,
                          bool vm, PrivilegeMode pm,
                          uint64_t currPc, const char* opcode, char resource, uint64_t addr,
                          uint64_t value, const char* assembly)
{
  const char* pmStr = privilegeModeToStr(vm, pm);
  fprintf(out, "#%jd %d %2s %016jx %8s %c %016jx %016jx %s",
          uintmax_t(tag), hartId, pmStr, uintmax_t(currPc), opcode, resource,
	  uintmax_t(addr), uintmax_t(value), assembly);
}


template <typename URV>
void
formatFpInstTrace(FILE* out, uint64_t tag, unsigned hartId, bool vm, PrivilegeMode pm,
		  URV currPc, const char* opcode, unsigned fpReg,
		  uint64_t fpVal, unsigned width, const char* assembly);

template <>
void
formatFpInstTrace<uint32_t>(FILE* out, uint64_t tag, unsigned hartId, bool vm, PrivilegeMode pm,
			    uint32_t currPc, const char* opcode, unsigned fpReg,
			    uint64_t fpVal, unsigned width,
			    const char* assembly)
{
  const char* pmStr = privilegeModeToStr(vm, pm);
  if (width == 64)
    {
      fprintf(out, "#%jd %d %2s %08x %8s f %02x %016jx  %s", uintmax_t(tag), hartId,
	      pmStr, currPc, opcode, fpReg, uintmax_t(fpVal), assembly);
    }
  else
    {
      uint32_t val32 = fpVal;
      fprintf(out, "#%jd %d %2s %08x %8s f %02x         %08x  %s",
	      uintmax_t(tag), hartId, pmStr, currPc, opcode, fpReg, val32, assembly);
    }
}

template <>
void
formatFpInstTrace<uint64_t>(FILE* out, uint64_t tag, unsigned hartId, bool vm, PrivilegeMode pm,
			    uint64_t currPc, const char* opcode, unsigned fpReg,
			    uint64_t fpVal, unsigned width,
			    const char* assembly)
{
  const char* pmStr = privilegeModeToStr(vm, pm);
  if (width == 64)
    {
      fprintf(out, "#%jd %d %2s %016jx %8s f %016jx %016jx %s",
	      uintmax_t(tag), hartId, pmStr, uintmax_t(currPc), opcode, uintmax_t(fpReg),
	      uintmax_t(fpVal), assembly);
    }
  else
    {
      uint32_t val32 = fpVal;
      fprintf(out, "#%jd %d %2s %016jx %8s f %016jx         %08x %s",
	      uintmax_t(tag), hartId, pmStr, uintmax_t(currPc), opcode, uintmax_t(fpReg),
	      val32, assembly);
    }
}


template <typename URV>
static
void
printPageTableWalk(FILE* out, const Hart<URV>& hart, const char* tag,
		   const std::vector<uint64_t>& addresses)
{
  fputs(tag, out);
  fputs(":", out);
  const char* sep = "";
  for (auto addr : addresses)
    {
      URV pte = 0;
      hart.peekMemory(addr, pte, true);
      uint64_t pte64 = pte;
      fputs(sep, out);
      fprintf(out, "0x%lx=0x%lx", addr, pte64);
      sep = ",";
    }
}


template <typename URV>
void
Hart<URV>::printInstTrace(uint32_t inst, uint64_t tag, std::string& tmp,
			  FILE* out)
{
  if (not out and not tracerExtension)
    return;

  uint32_t ix = (currPc_ >> 1) & decodeCacheMask_;
  DecodedInst* dip = &decodeCache_[ix];
  if (dip->isValid() and dip->address() == currPc_)
    printDecodedInstTrace(*dip, tag, tmp, out);
  else
    {
      DecodedInst di;
      uint64_t physPc = currPc_;
      decode(currPc_, physPc, inst, di);
      printDecodedInstTrace(di, tag, tmp, out);
    }
}


template <typename URV>
void
Hart<URV>::printDecodedInstTrace(const DecodedInst& di, uint64_t tag, std::string& tmp,
                                 FILE* out)
{
  if (tracerExtension)
    {
      TraceRecord<URV> tr(this, di);
      tracerExtension(&tr);
    }

  if (not out)
    return;

  if (csvTrace_)
    {
      printInstCsvTrace(di, out);
      return;
    }

  // Serialize to avoid jumbled output.
  std::lock_guard<std::mutex> guard(printInstTraceMutex);

  disassembleInst(di, tmp);
  if (hasInterrupt_)
    tmp += " (interrupted)";

  if (ldStSize_)
    {
      std::ostringstream oss;
      oss << "0x" << std::hex << ldStAddr_;
      tmp += " [" + oss.str() + "]";
    }
  else if (not vecRegs_.ldStAddr_.empty())
    {
      std::ostringstream oss;
      for (uint64_t i = 0; i < vecRegs_.ldStAddr_.size(); ++i)
	{
	  if (i > 0)
	    oss << ";";
	  oss << "0x" << std::hex << vecRegs_.ldStAddr_.at(i);
	  if (i < vecRegs_.stData_.size())
	    oss << '=' << "0x" << vecRegs_.stData_.at(i);
	}
      tmp += " [" + oss.str() + "]";
    }

  char instBuff[128];
  if (di.instSize() == 4)
    sprintf(instBuff, "%08x", di.inst());
  else
    sprintf(instBuff, "%04x", di.inst() & 0xffff);

  bool pending = false;  // True if a printed line need to be terminated.

  // Order: rfvmc (int regs, fp regs, vec regs, memory, csr)

  // Process integer register diff.
  int reg = intRegs_.getLastWrittenReg();
  URV value = 0;
  if (reg > 0)
    {
      value = intRegs_.read(reg);
      formatInstTrace<URV>(out, tag, hartIx_, lastVirt_, lastPriv_, currPc_, instBuff, 'r',
			   reg, value, tmp.c_str());
      pending = true;
    }

  // Process floating point register diff.
  int fpReg = fpRegs_.getLastWrittenReg();
  if (fpReg >= 0)
    {
      uint64_t val = fpRegs_.readBitsRaw(fpReg);
      if (pending) fprintf(out, "  +\n");
      unsigned width = isRvd() ? 64 : 32;
      formatFpInstTrace<URV>(out, tag, hartIx_, lastVirt_, lastPriv_, currPc_, instBuff, fpReg,
			     val, width, tmp.c_str());
      pending = true;
    }

  // Process vector register diff.
  unsigned groupSize = 0;
  int vecReg = lastVecReg(di, groupSize);
  if (vecReg >= 0)
    {
      for (unsigned i = 0; i < groupSize; ++i, ++vecReg)
	{
	  if (pending)
	    fprintf(out, " +\n");
	  formatVecInstTrace<URV>(out, tag, hartIx_, lastVirt_, lastPriv_, currPc_, instBuff,
				  vecReg, vecRegs_.getVecData(vecReg),
				  vecRegs_.bytesPerRegister(),
				  tmp.c_str());
	  pending = true;
	}
    }

  // Process memory diff.
  if (ldStWrite_)
    {
      if (pending)
	fprintf(out, "  +\n");
      formatInstTrace<URV>(out, tag, hartIx_, lastVirt_, lastPriv_, currPc_, instBuff, 'm',
			   URV(ldStAddr_), URV(ldStData_), tmp.c_str());
      pending = true;
    }

  // Process syscal memory diffs
  if (syscallSlam_ and di.instEntry()->instId() == InstId::ecall)
    {
      std::vector<std::pair<uint64_t, uint64_t>> scVec;
      lastSyscallChanges(scVec);
      for (auto al: scVec)
        {
          uint64_t addr = al.first, len = al.second;
          for (uint64_t ix = 0; ix < len; ix += 8, addr += 8)
            {
              uint64_t val = 0;
              peekMemory(addr, val, true);

              if (pending)
                fprintf(out, "  +\n");
              formatInstTrace<URV>(out, tag, hartIx_, lastVirt_, lastPriv_, currPc_, instBuff, 'm',
                                   addr, val, tmp.c_str());
              pending = true;
            }
        }
    }

  // Process CSR diffs.
  std::vector<CsrNumber> csrs;
  std::vector<unsigned> triggers;
  csRegs_.getLastWrittenRegs(csrs, triggers);

  typedef std::pair<URV, URV> CVP;  // CSR-value pair
  std::vector< CVP > cvps; // CSR-value pairs
  cvps.reserve(csrs.size() + triggers.size());

  // Collect non-trigger CSRs and their values.
  for (CsrNumber csr : csrs)
    {
      if (not csRegs_.peek(csr, value))
	continue;
      if (csr >= CsrNumber::TDATA1 and csr <= CsrNumber::TDATA3)
        continue; // Debug trigger values collected below.
      cvps.push_back(CVP(URV(csr), value));
    }

  // Collect trigger CSRs and their values. A synthetic CSR number
  // is used encoding the trigger number and the trigger component.
  for (unsigned trigger : triggers)
    {
      uint64_t data1(0), data2(0), data3(0);
      if (not peekTrigger(trigger, data1, data2, data3))
	continue;

      // Components of trigger that changed.
      bool t1 = false, t2 = false, t3 = false;
      getTriggerChange(trigger, t1, t2, t3);

      if (t1)
	{
	  URV ecsr = (trigger << 16) | URV(CsrNumber::TDATA1);
          cvps.push_back(CVP(ecsr, data1));
	}

      if (t2)
        {
	  URV ecsr = (trigger << 16) | URV(CsrNumber::TDATA2);
          cvps.push_back(CVP(ecsr, data2));
	}

      if (t3)
	{
	  URV ecsr = (trigger << 16) | URV(CsrNumber::TDATA3);
          cvps.push_back(CVP(ecsr, data3));
	}
    }

  // Sort by CSR number.
  std::sort(cvps.begin(), cvps.end(), [] (const CVP& a, const CVP& b) {
      return a.first < b.first; });

  for (const auto& cvp : cvps)
    {
      if (pending) fprintf(out, "  +\n");
      formatInstTrace<URV>(out, tag, hartIx_, lastVirt_, lastPriv_, currPc_, instBuff, 'c',
			   cvp.first, cvp.second, tmp.c_str());
      pending = true;
    }

  if (not pending)
    formatInstTrace<URV>(out, tag, hartIx_, lastVirt_, lastPriv_, currPc_, instBuff, 'r', 0, 0,
			 tmp.c_str());  // No change: emit X0 as modified reg.

  if (tracePtw_)
    {
      for (const auto& walk : virtMem_.getFetchWalks())
	{
	  fputs("  +\n", out);
	  printPageTableWalk(out, *this, "iptw", walk);
	}

      for (const auto& walk : virtMem_.getDataWalks())
	{
	  fputs("  +\n", out);
	  printPageTableWalk(out, *this, "dptw", walk);
	}
    }
  fputs("\n", out);
}


// We use custom code for speed. This makes a big difference in
// runtime for large traces.
namespace Whisper
{
  class PrintBuffer
  {
  public:

    PrintBuffer()
      : pos_(0)
    { }

    // Append to buffer hexadecimal string representing given number.
    inline
    PrintBuffer& print(uint64_t num)
    {
      if (num == 0)
	buff_.at(pos_++) = '0';
      else
	{
	  size_t beg = pos_;
	  for ( ; num; num = num >> 4 )
	    buff_.at(pos_++) = "0123456789abcdef"[num&0xf];
	  std::reverse(buff_.begin() + beg, buff_.begin() + pos_);
	}
      return *this;
    }

    // Append to buffer copy of given string excluding null char
    inline
    PrintBuffer& print(const char* str)
    {
      if (str)
	while (*str)
	  buff_.at(pos_++) = *str++;
      return *this;
    }

    // Append to buffer copy of given string excluding null char
    inline
    PrintBuffer& print(const std::string& str)
    {
      return print(str.c_str());
    }

    // Append char to buffer.
    inline
    PrintBuffer& printChar(char c)
    {
      buff_.at(pos_++) = c;
      return *this;
    }

    // Write contents of buffer to given file.
    inline
    void write(FILE* out)
    {
      size_t n = fwrite(buff_.data(), pos_, 1, out);
      assert(n == 1);
    }

    // Clear contents of buffer.
    inline
    void clear()
    { pos_ = 0; }

  private:
    typedef std::array<char, 12*1024> Buff;

    Buff buff_;
    size_t pos_ = 0;
  };
}


Whisper::PrintBuffer buffer;


template <typename URV>
void
Hart<URV>::printInstCsvTrace(const DecodedInst& di, FILE* out)
{
  // Serialize to avoid jumbled output.
  std::lock_guard<std::mutex> guard(printInstTraceMutex);

  if (not traceHeaderPrinted_)
    {
      traceHeaderPrinted_ = true;
      fprintf(out, "pc, inst, modified regs, source operands, memory, inst info, privilege, trap, disassembly, hartid");
      if (isRvs())
	fprintf(out, ", iptw, dptw");
      if (pmpEnabled_)
        fprintf(out, ", pmp");
      fprintf(out, "\n");
    }

  buffer.clear();

  // Program counter.
  uint64_t virtPc = di.address(), physPc = di.physAddress();
  buffer.print(virtPc);
  if (physPc != virtPc)
    buffer.printChar(':').print(physPc);

  // Instruction.
  buffer.printChar(',').print(di.inst()).printChar(',');

  // Changed integer register.
  unsigned regCount = 0;
  int reg = lastIntReg();
  uint64_t val64 = 0;
  if (reg > 0)
    {
      val64 = peekIntReg(reg);
      buffer.print(intRegs_.regName(reg)).printChar('=').print(val64);
      regCount++;
    }

  // Changed fp register.
  reg = lastFpReg();
  if (reg >= 0)
    {
      peekFpReg(reg, val64);
      if (not isRvd())
	val64 = uint32_t(val64);  // Clear top 32 bits if only F extension.
      if (regCount) buffer.printChar(';');
      buffer.print(fpRegs_.regName(reg)).printChar('=').print(val64);
      // Print incremental flags since FRM is sticky.
      unsigned fpFlags = lastFpFlags();
      if (fpFlags != 0)
	buffer.print(";ff=").print(fpFlags);
      regCount++;
    }

  // Changed CSR register(s).
  std::vector<CsrNumber> csrns;
  lastCsr(csrns);
  for (auto csrn : csrns)
    {
      URV val = 0;
      peekCsr(csrn, val);
      if (regCount) buffer.printChar(';');
      buffer.printChar('c').print(std::to_string(unsigned(csrn))).printChar('=').print(val);
      regCount++;
    }

  // Changed vector register group.
  unsigned groupSize = 0;
  int vecReg = lastVecReg(di, groupSize);
  if (vecReg >= 0)
    {
      for (unsigned i = 0; i < groupSize; ++i, ++vecReg)
	{
	  if (regCount) buffer.printChar(';');
	  buffer.printChar('v').print(std::to_string(vecReg)).printChar('=');
	  const uint8_t* data = vecRegs_.getVecData(vecReg);
	  unsigned byteCount = vecRegs_.bytesPerRegister();
	  for (unsigned i = 0; i < byteCount; ++i)
	    {
	      unsigned byte = data[byteCount - 1 - i];
	      if (byte < 16) buffer.printChar('0');
	      buffer.print(byte);
	    }
	  regCount++;
	}
    }

  // Non sequential PC change.
  auto instEntry = di.instEntry();
  bool hasTrap = hasInterrupt_ or hasException_;
  if (not hasTrap and instEntry->isBranch() and lastBranchTaken_)
    {
      if (regCount) buffer.printChar(';');
      buffer.print("pc=").print(pc_);
    }

  // Source operands.
  buffer.printChar(',');
  const char* sep = "";
  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      auto mode = instEntry->ithOperandMode(i);
      auto type = instEntry->ithOperandType(i);
      if (mode == OperandMode::Read or mode == OperandMode::ReadWrite or
	  type == OperandType::Imm)
	{
	  unsigned operand = di.ithOperand(i);
	  if (type ==  OperandType::IntReg)
	    buffer.print(sep).print(intRegs_.regName(operand));
	  else if (type ==  OperandType::FpReg)
	    buffer.print(sep).print(fpRegs_.regName(operand));
	  else if (type == OperandType::CsReg)
	    buffer.print(sep).printChar('c').print(std::to_string(operand));
	  else if (type == OperandType::VecReg)
	    {
	      buffer.print(sep).printChar('v').print(std::to_string(operand));
	      unsigned emul = i < vecRegs_.opsEmul_.size() ? vecRegs_.opsEmul_.at(i) : 1;
	      if (emul >= 2 and emul <= 8)
		buffer.printChar('m').print(emul);
	    }
	  else if (type == OperandType::Imm)
	    buffer.print(sep).printChar('i').print(operand);
	  sep = ";";
	}
    }

  // Print rounding mode with source operands.
  if (instEntry->hasRoundingMode())
    {
      RoundingMode rm = effectiveRoundingMode(di.roundingMode());
      buffer.print(sep).print("rm=").print(unsigned(rm));
      sep = ";";
    }

  // Memory
  buffer.printChar(',');
  uint64_t virtDataAddr = 0, physDataAddr = 0;
  bool load = false, store = false;
  if (lastLdStAddress(virtDataAddr, physDataAddr))
    {
      store = ldStWrite_;
      load = not store;
      buffer.print(virtDataAddr);
      if (physDataAddr != virtDataAddr)
	buffer.printChar(':').print(physDataAddr);
      if (store)
	buffer.printChar('=').print(ldStData_);
    }
  else if (not vecRegs_.ldStAddr_.empty())
    {
      for (uint64_t i = 0; i < vecRegs_.ldStAddr_.size(); ++i)
	{
	  if (i > 0)
	    buffer.printChar(';');
	  buffer.print(vecRegs_.ldStAddr_.at(i));
	  if (i < vecRegs_.maskedAddr_.size() and vecRegs_.maskedAddr_.at(i))
	    buffer.printChar('m');
	  if (i < vecRegs_.stData_.size())
	    buffer.printChar('=').print(vecRegs_.stData_.at(i));
	}
    }

  // Instruction information.
  buffer.printChar(',');
  RvExtension type = instEntry->extension();
  if (type == RvExtension::A)
    buffer.printChar('a');
  else if (load)
    buffer.printChar('l');
  else if (store)
    buffer.printChar('s');
  else if (instEntry->isBranch())
    {
      if (instEntry->isConditionalBranch())
	buffer.print(lastBranchTaken_ ? "t" : "nt");
      else
	{
	  if (di.isBranchToRegister() and
	      di.op0() == 0 and di.op1() == IntRegNumber::RegRa and di.op2() == 0)
	    buffer.printChar('r');
	  else if (di.op0() == IntRegNumber::RegRa)
	    buffer.printChar('c');
	  else
	    buffer.printChar('j');
	}
    }
  else if (type == RvExtension::F or type == RvExtension::D or type == RvExtension::Zfh)
    buffer.printChar('f');
  else if (type == RvExtension::V)
    buffer.printChar('v');

  // Privilege mode.
  if (not lastVirt_)
    {
      if      (lastPriv_ == PrivilegeMode::Machine)    buffer.print(",m,");
      else if (lastPriv_ == PrivilegeMode::Supervisor) buffer.print(",s,");
      else if (lastPriv_ == PrivilegeMode::User)       buffer.print(",u,");
      else                                             buffer.print(",,");
    }
  else
    {
      if (lastPriv_ == PrivilegeMode::Supervisor)      buffer.print(",vs,");
      else if (lastPriv_ == PrivilegeMode::User)       buffer.print(",vu,");
      else                                             buffer.print(",,");
    }

  // Interrupt/exception cause.
  if (hasTrap)
    {
      URV cause = 0;
      if (privilegeMode() == PrivilegeMode::Machine)
        peekCsr(CsrNumber::MCAUSE, cause);
      else if (privilegeMode() == PrivilegeMode::Supervisor)
        peekCsr(CsrNumber::SCAUSE, cause);
      buffer.print(uint64_t(cause));
    }
  buffer.printChar(',');

  // Disassembly.
  std::string tmp;
  disassembleInst(di, tmp);
  for (size_t i = 0; i < tmp.size(); ++i)
    if (tmp[i] == ',') tmp[i] = ';';
  buffer.print(tmp);

  // Hart Id.
  buffer.printChar(',').print(sysHartIndex());

  // Page table walk.
  if (isRvs())
    {
      buffer.printChar(',');
      std::vector<uint64_t> addrs, entries;
      getPageTableWalkAddresses(true, 0, addrs);
      getPageTableWalkEntries(true, 0, entries);
      sep = "";
      uint64_t n = std::min(addrs.size(), entries.size());
      for (uint64_t i = 0; i < n; ++i)
	{
	  buffer.print(sep).print(addrs.at(i)).printChar('=').print(entries.at(i));
	  sep = ";";
	}

      buffer.printChar(',');
      getPageTableWalkAddresses(false, 0, addrs);
      getPageTableWalkEntries(false, 0, entries);
      sep = "";
      n = std::min(addrs.size(), entries.size());
      for (uint64_t i = 0; i < n; ++i)
	{
	  buffer.print(sep).print(addrs.at(i)).printChar('=').print(entries.at(i));
	  sep = ";";
	}
    }

  // PMP
  if (pmpEnabled_)
    {
      buffer.printChar(',');
      std::vector<std::pair<uint32_t, Pmp>> pmps;
      getPmpsAccessed(pmps);
      sep = "";
      for (auto& pmp : pmps)
        {
          buffer.print(sep).print(pmp.first).printChar('=').print(pmp.second.val());
          sep = ";";
        }
    }

  buffer.printChar('\n');
  buffer.write(out);
}


/// Report the number of retired instruction count and the simulation
/// rate.
template <typename URV>
void
Hart<URV>::reportInstsPerSec(uint64_t instCount, double elapsed, bool userStop)
{
  std::lock_guard<std::mutex> guard(printInstTraceMutex);

  std::cout.flush();

  char secStr[20];
  sprintf(secStr, "%.2fs", elapsed);

  if (userStop)
    std::cerr << "User stop\n";
  std::cerr << "Retired " << instCount << " instruction"
	    << (instCount > 1? "s" : "") << " in "
	    << secStr;
  if (elapsed > 0)
    std::cerr << "  " << uint64_t(double(instCount)/elapsed) << " inst/s";
  std::cerr << '\n';
}


template <typename URV>
bool
Hart<URV>::logStop(const CoreException& ce, uint64_t counter, FILE* traceFile)
{
  bool success = false;
  bool isRetired = false;

  if (ce.type() == CoreException::Stop)
    {
      isRetired = true;
      success = (ce.value() >> 1) == 0;
      setTargetProgramFinished(true);
    }
  else if (ce.type() == CoreException::Exit)
    {
      isRetired = true;
      success = ce.value() == 0;
      setTargetProgramFinished(true);
    }

  if (isRetired)
    {
      if (minstretEnabled())
        retiredInsts_++;

      uint32_t inst = 0;
      readInst(currPc_, inst);
      std::string instStr;
      printInstTrace(inst, counter, instStr, traceFile);
    }

  using std::cerr;

  {
    std::lock_guard<std::mutex> guard(printInstTraceMutex);

    cerr << std::dec;
    if (ce.type() == CoreException::Stop)
      cerr << (success? "Successful " : "Error: Failed ")
           << "stop: " << ce.what() << ": " << ce.value() << "\n";
    else if (ce.type() == CoreException::Exit)
      cerr << "Target program exited with code " << ce.value() << '\n';
    else
      cerr << "Stopped -- unexpected exception\n";
  }

  return success;
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
