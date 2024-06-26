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

#include <cstdint>
#include <bitset>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <type_traits>
#include <functional>
#include <boost/circular_buffer.hpp>
#include <atomic>
#include "IntRegs.hpp"
#include "CsRegs.hpp"
#include "float-util.hpp"
#include "FpRegs.hpp"
#include "VecRegs.hpp"
#include "Memory.hpp"
#include "InstProfile.hpp"
#include "Syscall.hpp"
#include "PmpManager.hpp"
#include "VirtMem.hpp"
#include "Isa.hpp"
#include "Decoder.hpp"
#include "Disassembler.hpp"
#include "util.hpp"
#include "Imsic.hpp"
#include "FetchCache.hpp"
#include "pci/Pci.hpp"
#include "Stee.hpp"


namespace TT_PERF
{
  class PerfApi;
}


namespace WdRiscv
{

  template <typename URV>
  class Mcm;
  

  class DecodedInst;
  class InstEntry;

  enum class InstId : uint32_t;

  /// Thrown by the simulator when a stop (store to to-host) is seen
  /// or when the target program reaches the exit system call.
  class CoreException : public std::exception
  {
  public:

    enum Type { Stop, Exit, Snapshot };

    CoreException(Type type, const char* message = "", uint64_t address = 0,
		  uint64_t value = 0)
      : type_(type), msg_(message), addr_(address), val_(value)
    { }

    const char* what() const noexcept override
    { return msg_; }

    Type type() const
    { return type_; }

    uint64_t address() const
    { return addr_; }

    uint64_t value() const
    { return val_; }

  private:
    Type type_ = Stop;
    const char* msg_ = "";
    uint64_t addr_ = 0;
    uint64_t val_ = 0;
  };


  /// Changes made by the execution of one instruction. Useful for
  /// test pattern generation.
  struct ChangeRecord
  {
    void clear()
    { *this = ChangeRecord(); }

    uint64_t newPc = 0;        // Value of pc after instruction execution.
    bool hasException = false; // True if instruction causes an exception.

    bool hasIntReg = false;    // True if there is an integer register change.
    unsigned intRegIx = 0;     // Number of changed integer register if any.
    uint64_t intRegValue = 0;  // Value of changed integer register if any.

    bool hasFpReg = false;     // True if there is an FP register change.
    unsigned fpRegIx = 0;      // Number of changed fp register if any.
    uint64_t fpRegValue = 0;   // Value of changed fp register if any.

    unsigned memSize = 0;      // Size of changed memory (0 if none).
    uint64_t memAddr = 0;      // Address of changed memory if any.
    uint64_t memValue = 0;     // Value of changed memory if any.

    // An exception will result in changing multiple CSRs.
    std::vector<CsrNumber> csrIx;   // Numbers of changed CSRs if any.
    std::vector<uint64_t> csrValue; // Values of changed CSRs if any.
  };


  /// Model a RISCV hart with integer registers of type URV (uint32_t
  /// for 32-bit registers and uint64_t for 64-bit registers).
  template <typename URV_>
  class Hart
  {
  public:

    /// Alias the template parameter to allow it to be used outside this
    /// template.
    using URV = URV_;

    /// Signed register type corresponding to URV. For example, if URV
    /// is uint32_t, then SRV will be int32_t.
    using SRV = typename std::make_signed_t<URV>;

    /// Constructor: Define a hart with the given index (unique index
    /// within a system of cores -- see sysHartIndex method) and
    /// associate it with the given memory. The MHARTID is configured as
    /// a read-only CSR with a reset value of hartId.
    Hart(unsigned hartIx, URV hartId, Memory& memory, Syscall<URV>& syscall, uint64_t& time);

    /// Destructor.
    ~Hart();

    /// Return count of integer registers.
    unsigned intRegCount() const
    { return intRegs_.size(); }

    /// Return the name of the given integer register. Return an
    /// abi-name (e.g. sp) if abi names are enabled.
    std::string_view intRegName(unsigned regIx) const
    { return disas_.intRegName(regIx); }

    /// Return the name of the given floating point register. Return an
    /// abi-name (e.g. fa0) if abi names are enabled.
    std::string_view fpRegName(unsigned regIx) const
    { return disas_.fpRegName(regIx); }

    /// Return count of floating point registers. Return zero if
    /// extension f is not enabled.
    unsigned fpRegCount() const
    { return isRvf()? fpRegs_.size() : 0; }

    /// Return count of vector registers. Return zero if extension v
    /// is not enabled.
    unsigned vecRegCount() const
    { return isRvv()? vecRegs_.size() : 0; }

    unsigned vecRegSize() const
    { return isRvv()? vecRegs_.bytesPerRegister() : 0; }

    /// Return size of memory in bytes.
    uint64_t memorySize() const
    { return memory_.size(); }

    /// Return the value of the program counter.
    URV peekPc() const;

    /// Set the program counter to the given address.
    void pokePc(URV address);

    /// Set val to the value of integer register reg returning true on
    /// success. Return false leaving val unmodified if reg is out of
    /// bounds.
    bool peekIntReg(unsigned reg, URV& val) const;

    /// Set val to the value of integer register reg returning true on
    /// success. Return false leaving val unmodified if reg is out of
    /// bounds. If successful, set name to the register name.
    bool peekIntReg(unsigned reg, URV& val, std::string_view& name) const;

    /// Return to the value of integer register reg which must not be
    /// out of bounds (otherwise we trigger an assert).
    URV peekIntReg(unsigned reg) const;

    /// Set the given integer register, reg, to the given value
    /// returning true on success. Return false if reg is out of
    /// bound.
    bool pokeIntReg(unsigned reg, URV val);

    /// Set val to the bit-pattern of the value of the floating point
    /// register returning true on success. Return false leaving val
    /// unmodified if reg is out of bounds of if no floating point
    /// extension is enabled.
    bool peekFpReg(unsigned reg, uint64_t& val) const;

    /// Set val to the bit-pattern of the value of the floating point
    /// register (after unboxing that value if it is nan-boxed)
    /// returning true on success. Return false leaving val unmodified
    /// if reg is out of bounds of if no floating point extension is
    /// enabled.
    bool peekUnboxedFpReg(unsigned reg, uint64_t& val) const;

    /// Set the given FP register, reg, to the given value returning
    /// true on success. Return false if reg is out of bound.
    bool pokeFpReg(unsigned reg, uint64_t val);

    /// Set val to the value of the control and status register csr
    /// returning true on success. Return false leaving val unmodified
    /// if csr is out of bounds.
    [[nodiscard]] bool peekCsr(CsrNumber csr, URV& val) const
    { return csRegs_.peek(csr, val); }

    [[nodiscard]] bool peekCsr(CsrNumber csr, URV& val, bool virtMode) const
    { return csRegs_.peek(csr, val, virtMode); }

    /// Return value of the given csr. Throw exception if csr is out of bounds.
    URV peekCsr(CsrNumber csr) const;

    /// Set val, reset, writeMask, and pokeMask respectively to the
    /// value, reset-value, write-mask, poke-mask, and read-mask of
    /// the control and status register csr returning true on success.
    /// Return false leaving parameters unmodified if csr is out of bounds.
    bool peekCsr(CsrNumber csr, URV& val, URV& reset, URV& writeMask,
		 URV& pokeMask, URV& readMask) const;

    /// Set val/name to the value/name of the control and status
    /// register csr returning true on success. Return false leaving
    /// val/name unmodified if csr is out of bounds.
    bool peekCsr(CsrNumber csr, URV& val, std::string_view& name) const;

    /// Set val to the value of the control and status register csr field
    /// returning true on success. Return false leaving val unmodified
    /// if csr is out of bounds.
    bool peekCsr(CsrNumber csr, std::string_view field, URV& val) const;

    /// Set the given control and status register, csr, to the given
    /// value returning true on success. Return false if csr is out of
    /// bound.
    bool pokeCsr(CsrNumber csr, URV val);

    /// Similar to pokeCsr but meant for server/interactive code: Keep
    /// track of external MIP pokes to avoid clobbering them with internal
    /// ones.
    bool externalPokeCsr(CsrNumber csr, URV val)
    { if (csr == CsrNumber::MIP) mipPoked_ = true; return pokeCsr(csr, val); }

    /// Put in value the bytes of the given vector register (most
    /// significant byte first). Return true on success, return false
    /// if reg is out of bounds.
    bool peekVecReg(unsigned reg, std::vector<uint8_t>& value) const;

    /// Put the bytes of the value in the given vector register.
    /// The first byte in value should be the most significant.
    /// If value is smaller than vector register size, it is padded
    /// with zeros on the most-significant side.
    bool pokeVecReg(unsigned reg, const std::vector<uint8_t>& value);

    /// Find the integer register with the given name (which may
    /// represent an integer or a symbolic name). Set num to the
    /// number of the corresponding register if found. Return true on
    /// success and false if no such register.
    bool findIntReg(std::string_view name, unsigned& num) const;

    /// Find the floating point with the given name.  Set num to the
    /// number of the corresponding register if found. Return true on
    /// success and false if no such register.
    bool findFpReg(std::string_view name, unsigned& num) const;

    /// Find vector register by name. See findFpReg.
    bool findVecReg(std::string_view name, unsigned& num) const;

    /// Find the control and status register with the given name
    /// (which may represent an integer or a symbolic name). Return
    /// pointer to CSR on success and nullptr if no such register.
    Csr<URV>* findCsr(std::string_view name);

    /// Find the control and status register with the given number.
    /// Return pointer to CSR on success and nullptr if no such
    /// register.
    const Csr<URV>* findCsr(CsrNumber number)
    { return csRegs_.findCsr(number); }

    /// Configure given CSR. Return true on success and false if no such CSR.
    bool configCsrByUser(std::string_view name, bool implemented, URV resetValue, URV mask,
			 URV pokeMask, bool shared);

    /// Configure given CSR. Return true on success and false if no such CSR.
    bool configCsr(std::string_view name, bool implemented, URV resetValue, URV mask,
		   URV pokeMask, bool shared);

    /// Define a new CSR (beyond the standard CSRs defined by the
    /// RISCV spec). Return true on success and false if name/number
    /// already in use.
    bool defineCsr(std::string name, CsrNumber number, bool implemented, URV resetValue,
		   URV mask, URV pokeMask);

    /// Configure given trigger with given reset values, write and
    /// poke masks. Return true on success and false on failure.
    bool configTrigger(unsigned trigger,
                       const std::vector<uint64_t>& resets,
		       const std::vector<uint64_t>& masks,
		       const std::vector<uint64_t>& pokeMasks)
    { return csRegs_.configTrigger(trigger, resets, masks, pokeMasks); }

    /// Define the set of supported trigger types.
    bool setSupportedTriggerTypes(const std::vector<std::string>& types)
    { return csRegs_.triggers_.setSupportedTypes(types); }

    /// Enable the extensions defined by the given string. If
    /// updateMisa is true then the MISA CSR reset value is updated to
    /// enable the extensions defined by the given string (this is done
    /// for linux/newlib emulation).
    bool configIsa(std::string_view string, bool updateMisa);

    /// Enable/disable matching all addresses in a load/store access
    /// for debug triggering.
    void configAllLdStAddrTrigger(bool flag)
    { csRegs_.configAllLdStAddrTrigger(flag); }

    /// Enable/disable matching all addresses in a instruction fetch
    /// access for debug triggering.
    void configAllInstAddrTrigger(bool flag)
    { csRegs_.configAllInstAddrTrigger(flag); }

    /// Configure machine mode performance counters returning true on
    /// success and false on failure. N consecutive counters starting
    /// at MHPMCOUNTER3/MHPMCOUNTER3H are made read/write. The
    /// remaining counters are made write-any read-zero. For each
    /// counter that is made read-write the corresponding MHPMEVENT is
    /// made read-write otherwise it is make write-any read-zero.
    /// The cof flag indicates whether or not counter-overflow extension
    /// is enabled.
    bool configMachineModePerfCounters(unsigned n, bool cof);

    /// Configure user mode performance counters returning true on
    /// success and false on failure. N cannot exceed the number of machine
    /// mode performance registers. First N performance counters are configured
    /// as readable, the remaining ones are made read-zero.
    bool configUserModePerfCounters(unsigned n);

    /// Set the maximum event id that can be written to the MHPMEVENT
    /// registers. Larger values are replaced by this max-value before
    /// being written to the MHPMEVENT registers. Return true on
    /// success and false on failure.
    void configMachineModeMaxPerfEvent(URV maxId)
    { csRegs_.setMaxEventId(maxId); }

    /// Configure valid event. If this is used then events outside the
    /// given vector are replaced by zero before being assigned to an
    /// MHPMEVENT register. Otherwise, events greater that
    /// max-event-id are clamped to max-event-id before being assigned
    /// to an MHPMEVENT register.
    void configPerfEvents(std::vector<unsigned>& eventVec)
    { csRegs_.configPerfEvents(eventVec); }

    /// Map the give user event number to the given internal event id.
    /// When the given user number is written to an mphpmevent csr, then
    /// the corresponding event-id is associated with the event counter csr.
    void configEventNumber(URV userNumber, EventNumber eventId)
    { csRegs_.mPerfRegs_.configEventNumber(userNumber, eventId); }

    /// Configure the address translation modes supported by this hart.
    void configAddressTranslationModes(const std::vector<VirtMem::Mode>& modes)
    { virtMem_.setSupportedModes(modes); }

    /// Configure the address translation pointer masking modes supported by this hart.
    void configAddressTranslationPmms(const std::vector<VirtMem::Pmm>& pmms)
    { virtMem_.setSupportedPmms(pmms); }

    /// Enable page based memory types.
    void enableTranslationPbmt(bool flag)
    { enableExtension(RvExtension::Svpbmt, flag); updateTranslationPbmt(); }

    /// Enable Svinval extension.
    void enableSvinval(bool flag)
    { enableExtension(RvExtension::Svinval, flag); }

    /// Called when Svpbmt configuration changes. Enable/disable pbmt in
    /// virtual memory class.
    void updateTranslationPbmt()
    {
      bool flag = extensionIsEnabled(RvExtension::Svpbmt);
      csRegs_.enableSvpbmt(flag);

      auto menv = csRegs_.getImplementedCsr(CsrNumber::MENVCFG);
      if (menv)
	{
	  flag = flag and csRegs_.menvcfgPbmte();
	  bool adu = csRegs_.menvcfgAdue();
	  virtMem_.setFaultOnFirstAccess(not adu);
	  virtMem_.setFaultOnFirstAccessStage2(not adu);
	}
      virtMem_.enablePbmt(flag);
      auto henv = csRegs_.getImplementedCsr(CsrNumber::HENVCFG);
      if (henv)
	{
          flag = flag and csRegs_.henvcfgPbmte();
	  bool adu = csRegs_.henvcfgAdue();
	  virtMem_.setFaultOnFirstAccessStage1(not adu);
	}
      virtMem_.enableVsPbmt(flag);
    }

    /// Called when pointer masking configuration changes.
    void updateTranslationPmm()
    {
      using PM = PrivilegeMode;

      if (isRvSsnpm())
        {
          uint8_t pmm = csRegs_.senvcfgPmm();
          if (isRvu())
            virtMem_.enablePointerMasking(VirtMem::Pmm(pmm), PM::User, false);

          pmm = csRegs_.henvcfgPmm();
          if (isRvh())
            virtMem_.enablePointerMasking(VirtMem::Pmm(pmm), PM::Supervisor, true);
        }

      if (isRvSmnpm())
        {
          uint8_t pmm = csRegs_.menvcfgPmm();
          if (isRvs())
            virtMem_.enablePointerMasking(VirtMem::Pmm(pmm), PM::Supervisor, false);
          else if (isRvu())
            virtMem_.enablePointerMasking(VirtMem::Pmm(pmm), PM::User, false);
        }
    }

    /// Enable page translation naturally aligned power of 2 page sizes.
    void enableTranslationNapot(bool flag)
    { virtMem_.enableNapot(flag); }

    /// Do not consider lr and sc instructions as load/store events for
    /// performance counter when flag is false. Do consider them when
    /// flag is true.
    void perfCountAtomicLoadStore(bool flag)
    { decoder_.perfCountAtomicLoadStore(flag); }

    /// Do not consider flw,fsw,fld,fsd...c instructions as load/store
    /// events for performance counter when flag is false. Do consider
    /// them when flag is true.
    void perfCountFpLoadStore(bool flag)
    { decoder_.perfCountFpLoadStore(flag); }

    /// Configure vector unit of this hart.
    void configVector(unsigned bytesPerVec, unsigned minBytesPerElem,
		      unsigned maxBytesPerElem,
                      std::unordered_map<GroupMultiplier, unsigned>* minSewPerLmul,
		      std::unordered_map<GroupMultiplier, unsigned>* maxSewPerLmul)
    { vecRegs_.config(bytesPerVec, minBytesPerElem, maxBytesPerElem, minSewPerLmul, maxSewPerLmul); }

    /// Configure mask agnostic policy. Allones if flag is true, undisturb if
    /// false.
    void configMaskAgnosticAllOnes(bool flag)
    { vecRegs_.configMaskAgnosticAllOnes(flag); }

    /// Configure tail agnostic policy. Allones if flag is true, undisturb if
    /// false.
    void configTailAgnosticAllOnes(bool flag)
    { vecRegs_.configTailAgnosticAllOnes(flag); }

    /// Return currently configured element width
    ElementWidth elemWidth() const
    { return vecRegs_.elemWidth(); }

    /// Return currently configured group multiplier
    GroupMultiplier groupMultiplier() const
    { return vecRegs_.groupMultiplier(); }

    /// Configure the load-reserve reservation size in bytes.
    /// A size smaller than 4/8 in rv32/rv64 has the effect of 4/8.
    void configReservationSize(unsigned size)
    { lrResSize_ = size; }

    /// Configure SC.W/D to keep/drop (flag=true/false) reservation on
    /// exceptions in SC.W/D.
    void keepReservationOnScException(bool flag)
    { keepReservOnScException_ = flag; }

    /// Get the values of the three components of the given debug
    /// trigger. Return true on success and false if trigger is out of
    /// bounds.
    bool peekTrigger(unsigned trigger, uint64_t& data1, uint64_t& data2,
                     uint64_t& data3) const
    { return csRegs_.peekTrigger(trigger, data1, data2, data3); }

    /// Get the values of the three components of the given debug
    /// trigger as well as the components write and poke masks. Return
    /// true on success and false if trigger is out of bounds.
    bool peekTrigger(unsigned trigger,
                     uint64_t& val1, uint64_t& val2, uint64_t& val3,
		     uint64_t& wm1, uint64_t& wm2, uint64_t& wm3,
		     uint64_t& pm1, uint64_t& pm2, uint64_t& pm3) const
    { return csRegs_.peekTrigger(trigger, val1, val2, val3, wm1, wm2, wm3,
				 pm1, pm2, pm3); }

    /// Set the values of the three components of the given debug
    /// trigger. Return true on success and false if trigger is out of
    /// bounds.
    bool pokeTrigger(URV trigger, URV data1, URV data2, URV data3)
    { return csRegs_.pokeTrigger(trigger, data1, data2, data3); }

    /// Fill given vector (cleared on entry) with the numbers of
    /// implemented CSRs.
    void getImplementedCsrs(std::vector<CsrNumber>& vec) const;

    /// Reset this hart. Reset all CSRs to their initial value. Reset all
    /// integer registers to zero. Reset PC to the reset-pc as
    /// defined by defineResetPc (default is zero).
    void reset(bool resetMemoryMappedRegisters = false);

    /// Run fetch-decode-execute loop. If a stop address (see
    /// setStopAddress) is defined, stop when the program counter
    /// reaches that address. If a tohost address is defined (see
    /// setToHostAdress), stop when a store instruction writes into
    /// that address. If given file is non-null, then print to that
    /// file a record for each executed instruction.
    bool run(FILE* file = nullptr);

    /// Run one instruction at the current program counter. Update
    /// program counter. If file is non-null then print thereon
    /// tracing information related to the executed instruction.
    void singleStep(FILE* file = nullptr);

    /// Same as above but decoded istruction information is placed
    /// in given di object.
    void singleStep(DecodedInst& di, FILE* file = nullptr);

    /// Run until the program counter reaches the given address. Do
    /// execute the instruction at that address. If file is non-null
    /// then print thereon tracing information after each executed
    /// instruction. Similar to method run with respect to tohost.
    bool runUntilAddress(uint64_t address, FILE* file = nullptr);

    /// Helper to runUntiAddress: Same as runUntilAddress but does not
    /// print run-time and instructions per second.
    bool untilAddress(uint64_t address, FILE* file = nullptr);

    /// Helper to single step N times. Returns false if program terminated
    /// with failing condition and true otherwise.
    bool runSteps(uint64_t steps, FILE* file = nullptr);

    /// Define the program counter value at which the run method will
    /// stop.
    void setStopAddress(URV address)
    { stopAddr_ = address; stopAddrValid_ = true; }

    /// Undefine stop address (see setStopAddress).
    void clearStopAddress()
    { stopAddrValid_ = false; }

    /// Define the memory address corresponding to console io. Reading
    /// (lw/lh/lb) or writing (sw/sh/sb) from/to that address
    /// reads/writes a byte to/from the console.
    void setConsoleIo(URV address)
    { conIo_ = address; conIoValid_ = true; }

    /// Do not use console io address for input if flag is false:
    /// Loads from that address simply return last value stored there.
    void enableConsoleInput(bool flag)
    { enableConIn_ = flag; }

    /// Undefine console io address (see setConsoleIo).
    void clearConsoleIo()
    { conIoValid_ = false; }

    /// Console output gets directed to given file.
    void setConsoleOutput(FILE* out)
    { consoleOut_ = out; }

    /// If a console io memory mapped location is defined then put its
    /// address in address and return true; otherwise, return false
    /// leaving address unmodified.
    bool getConsoleIo(URV& address) const
    { if (conIoValid_) address = conIo_; return conIoValid_; }

    /// Start logging at the given instruction rank.
    void setLogStart(uint64_t rank)
    { logStart_ = rank; }

    /// Set whether this hart owns its trace file.
    void setOwnTrace(bool flag)
    { ownTrace_ = flag; }

    /// Define memory mapped locations for CLINT.
    void configAclint(uint64_t base, uint64_t size, uint64_t mswiOffset, bool hasMswi,
                      uint64_t mtimerOffset, uint64_t mtimeOffset, bool hasMtimer,
		      bool softwareInterruptOnReset, bool deliverInterrupts,
                      std::function<Hart<URV>*(unsigned ix)> indexToHart)
    {
      aclintBase_ = base;
      aclintSize_ = size;

      if (hasMswi)
        {
          aclintSwStart_ = mswiOffset;
          aclintSwEnd_ = mswiOffset + 0x4000;
        }

      if (hasMtimer)
        {
          aclintMtimerStart_ = mtimerOffset;
          aclintMtimerEnd_ = mtimerOffset + 0x8000;
          aclintMtimeStart_ = mtimeOffset;
          aclintMtimeEnd_ = mtimeOffset + 0x8;
        }
      aclintSiOnReset_ = softwareInterruptOnReset;
      aclintDeliverInterrupts_ = deliverInterrupts;
      indexToHart_ = indexToHart;
    }

    /// Define a memory mapped locations for interruptor agent.
    void configInterruptor(uint64_t addr,
			   std::function<Hart<URV>*(unsigned ix)> indexToHart)
    {
      interruptor_ = addr;
      hasInterruptor_ = true;
      indexToHart_ = indexToHart;
    }

    /// Set the output file in which to dump the state of accessed
    /// memory lines. Return true on success and false if file cannot
    /// be opened.
    void setInitialStateFile(FILE* file)
    { initStateFile_ = file; }

    /// Disassemble given instruction putting results into the given
    /// string.
    void disassembleInst(uint32_t inst, std::string& str)
    { disas_.disassembleInst(inst, decoder_, str); }

    /// Disassemble given instruction putting results into the given
    /// string.
    void disassembleInst(const DecodedInst& di, std::string& str)
    { disas_.disassembleInst(di, str); }

    /// Decode given instruction returning a pointer to the
    /// instruction information and filling op0, op1 and op2 with the
    /// corresponding operand specifier values. For example, if inst
    /// is the instruction code for "addi r3, r4, 77", then the
    /// returned value would correspond to addi and op0, op1 and op2
    /// will be set to 3, 4, and 77 respectively. If an instruction
    /// has fewer than 3 operands then only a subset of op0, op1 and
    /// op2 will be set. If inst is not a valid instruction , then we
    /// return a reference to the illegal-instruction info.
    const InstEntry& decode(uint32_t inst, uint32_t& op0, uint32_t& op1,
			    uint32_t& op2, uint32_t& op3)
    { return decoder_.decode(inst, op0, op1, op2, op3); }

    /// Similar to the preceding decode method but with decoded data
    /// placed in the given DecodedInst object.
    void decode(URV addr, uint64_t physAddr, uint32_t inst, DecodedInst& decodedInst)
    { decoder_.decode(addr, physAddr, inst, decodedInst); }

    /// Return the 32-bit instruction corresponding to the given 16-bit
    /// compressed instruction. Return an illegal 32-bit opcode if given
    /// 16-bit code is not a valid compressed instruction.
    uint32_t expandCompressedInst(uint16_t inst) const
    { return decoder_.expandCompressedInst(inst); }

    /// Return the instruction table entry associated with the given
    /// instruction id. Return illegal instruction entry id is out of
    /// bounds.
    const InstEntry& getInstructionEntry(InstId id) const
    { return decoder_.getInstructionEntry(id); }

    /// Return the instruction table entry associated with the given
    /// instruction id. Return illegal instruction entry id is out of
    /// bounds.
    const InstEntry& getInstructionEntry(const std::string& name) const
    { return decoder_.getInstructionEntry(name); }

    /// Return the CS registers associated with this hart.
    const CsRegs<URV>& csRegs() const
    { return csRegs_; }

    /// Return the vector registers associated with this hart.
    const VecRegs& vecRegs() const
    { return vecRegs_; }

    /// Return the virtmem associated with this hart.
    const VirtMem& virtMem() const
    { return virtMem_; }

    /// Return the IMSIC associated with this hart.
    const std::shared_ptr<TT_IMSIC::Imsic> imsic() const
    { return imsic_; }

    /// Locate the ELF function containing the give address returning true
    /// on success and false on failure.  If successful set name to the
    /// corresponding function name and symbol to the corresponding symbol
    /// value.
    bool findElfFunction(URV addr, std::string& name, ElfSymbol& value) const
    { return memory_.findElfFunction(addr, name, value); }

    /// Set val to the value of the byte at the given address
    /// returning true on success and false if address is out of
    /// bounds. Memory is little-endian. Bypass physical memory
    /// attribute checking if usePma is false.
    bool peekMemory(uint64_t addr, uint8_t& val, bool usePma) const;

    /// Half-word version of the preceding method.
    bool peekMemory(uint64_t addr, uint16_t& val, bool usePma) const;

    /// Word version of the preceding method.
    bool peekMemory(uint64_t addr, uint32_t& val, bool usePma) const;

    /// Double-word version of the preceding method.
    bool peekMemory(uint64_t addr, uint64_t& val, bool usePma) const;

    /// Set the memory byte at the given address to the given value.
    /// Return true on success and false on failure (address out of
    /// bounds, location not mapped, location not writable etc...)
    /// Bypass physical memory attribute checking if usePma is false.
    bool pokeMemory(uint64_t addr, uint8_t val, bool usePma);

    /// Half-word version of the preceding method.
    bool pokeMemory(uint64_t address, uint16_t val, bool usePma);

    /// Word version of the preceding method.
    bool pokeMemory(uint64_t addr, uint32_t val, bool usePma);

    /// Double-word version of the preceding method.
    bool pokeMemory(uint64_t addr, uint64_t val, bool usePma);

    /// Define value of program counter after a reset.
    void defineResetPc(URV addr)
    { resetPc_ = addr; }

    /// Define value of program counter after a non-maskable interrupt.
    void defineNmiPc(URV addr)
    { nmiPc_ = addr; }

    /// Define value of program counter after an exception in non-maskable
    /// interrupt code.
    void defineNmiExceptionPc(URV addr)
    { nmiExceptionPc_ = addr; }

    /// Clear/set pending non-maskable-interrupt.
    void setPendingNmi(NmiCause cause = NmiCause::UNKNOWN);

    /// Clear pending non-maskable-interrupt.
    void clearPendingNmi();

    /// Set/clear Supervisor external interrupt pin.
    void setSeiPin(bool flag)
    { seiPin_ = flag; csRegs_.setSeiPin(flag); }

    /// Return the current state of the Supervisor external interrupt pin.
    bool getSeiPin() const
    { return seiPin_; }

    /// Define address to which a write will stop the simulator. An
    /// sb, sh, or sw instruction will stop the simulator if the write
    /// address of the instruction is identical to the given address.
    void setToHostAddress(uint64_t address);

    void setFromHostAddress(uint64_t addr, bool enabled)
    { fromHost_ = addr; fromHostValid_ = enabled; }

    /// Undefine address to which a write will stop the simulator
    void clearToHostAddress();

    /// Set address to the special address writing to which stops the simulation. Return
    /// true on success and false on failure (no such address defined).
    bool getToHostAddress(uint64_t& address) const
    { if (toHostValid_) address = toHost_; return toHostValid_; }

    /// Set address to the special address writing to which stops the simulation. Return
    /// true on success and false on failure (no such address defined).
    bool getFromHostAddress(uint64_t& address) const
    { if (fromHostValid_) address = toHost_; return fromHostValid_; }

    /// Return true if given address is an HTIF address.
    bool isHtifAddr(uint64_t a) const
    { return (toHostValid_ and a == toHost_) or (fromHostValid_ and a == fromHost_); }

    /// Program counter.
    URV pc() const
    { return pc_; }

    /// Support for tracing: Return the pc of the last executed
    /// instruction.
    URV lastPc() const
    { return currPc_; }

    /// Support for tracing: Return the privilege mode before the last
    /// executed instruction.
    PrivilegeMode lastPrivMode() const
    { return lastPriv_; }

    /// Support for tracing: Return the index of the integer register
    /// written by the last executed instruction. Return -1 if no
    /// integer register was written.
    int lastIntReg() const
    { return intRegs_.getLastWrittenReg(); }

    /// Similar to lastIntReg() but if successful set val to the
    /// previous value of the integer register written by the last
    /// executed instruction.
    int lastIntReg(uint64_t& val) const
    { return intRegs_.getLastWrittenReg(val); }

    /// Support for tracing: Return the index of the floating point
    /// register written by the last executed instruction. Return -1
    /// it no FP register was written.
    int lastFpReg() const
    { return fpRegs_.getLastWrittenReg(); }

    /// Similar to lastFpReg() but if successful set val to the
    /// previous value of the integer register written by the last
    /// executed instruction.
    int lastFpReg(uint64_t& val) const
    { return fpRegs_.getLastWrittenReg(val); }

    /// Support for tracing: Return the incremental change to the FRM
    /// register by the last floating point instruction. Return zer0
    /// if last instruction was not FP or if it had no impact on FRM.
    unsigned lastFpFlags() const
    { return fpRegs_.getLastFpFlags(); }

    /// Support for tracing: Return the index of the destination
    /// vector register of the last executed instruction. Return -1 if
    /// no vector register was written. Set group to the effective
    /// group multiplier.
    int lastVecReg(const DecodedInst& di, unsigned& group) const;

    /// Support for tracing: Return incremental changes to fp flags and vxsat,
    /// but for vector instructions on per-element basis.
    void lastIncVec(std::vector<uint8_t>& fpFlags, std::vector<uint8_t>& vxsat,
                    std::vector<VecRegs::Step>& steps) const
    { return vecRegs_.lastIncVec(fpFlags, vxsat, steps); }

    /// Return true if the last executed instruction triggered a trap
    /// (had an exception or encoutered an interrupt).
    bool lastInstructionTrapped() const
    { return hasException_ or hasInterrupt_; }

    /// Return true if the last executed instruction was interrupted.
    bool lastInstructionInterrupted() const
    { return hasInterrupt_; }

    /// Support for tracing: Fill the csrs vector with the
    /// register-numbers of the CSRs written by the execution of the
    /// last instruction. CSRs modified as a side effect (e.g. mcycle
    /// and minstret) are not included.
    void lastCsr(std::vector<CsrNumber>& csrs) const
    { csRegs_.getLastWrittenRegs(csrs); }

    /// Return the CSR value produced by the last executed instruction.
    URV lastCsrValue(CsrNumber csr)
    { return csRegs_.lastCsrValue(csr); }

    /// Support for tracing: Fill the csrs vector with the
    /// register-numbers of the CSRs written by the execution of the
    /// last instruction. CSRs modified as a side effect (e.g. mcycle
    /// and minstret) are not included. Fill the triggers vector with
    /// the numbers of the debug-trigger registers written by the
    /// execution of the last instruction.
    void lastCsr(std::vector<CsrNumber>& csrs,
		 std::vector<unsigned>& triggers) const;

    /// Support for tracing: Set address and value to the physical
    /// memory location changed by the last instruction and return the
    /// number of bytes written. Return 0 leaving addr and value
    /// unmodified if last instruction did not write memory (not a
    /// store or store got trapped).
    unsigned lastStore(uint64_t& addr, uint64_t& value) const
    {
      if (not ldStWrite_)
	return 0;
      addr = ldStPhysAddr1_;
      value = ldStData_;
      return ldStSize_;
    }

    /// Similar to the previous lastStore but for page crossing stores, pa2 will be set to
    /// the physical address of the second page. If store did not cross a page boundary
    /// pa2 will be the same as pa1. Va is the virtual address of the store data.
    unsigned lastStore(uint64_t& va, uint64_t& pa1, uint64_t& pa2, uint64_t& value) const
    {
      if (not ldStWrite_)
	return 0;
      va = ldStAddr_;
      pa1 = ldStPhysAddr1_;
      pa2 = ldStPhysAddr2_;
      value = ldStData_;
      return ldStSize_;
    }

    /// If last executed instruction is a CMO (cache maintenance operation), then set
    /// vaddr/paddr to the corresponding data virtual/physical address returning the
    /// cache line size. Return 0 otherwise leaving vadd/paddr unmodified.
    unsigned lastCmo(uint64_t& vaddr, uint64_t& paddr) const
    {
      if (ldStSize_ != cacheLineSize_)
	return 0;
      vaddr = ldStAddr_;
      paddr = ldStPhysAddr1_;
      return ldStSize_;
    }

    bool getLastVectorMemory(std::vector<uint64_t>& addresses,
			     std::vector<uint64_t>& data,
			     unsigned& elementSize) const
    { return vecRegs_.getLastMemory(addresses, data, elementSize); }


    void lastSyscallChanges(std::vector<std::pair<uint64_t, uint64_t>>& v) const
    { syscall_.getMemoryChanges(v); }

    /// Return data size if last instruction is a ld/st instruction (AMO is considered a
    /// store) setting virtAddr and physAddr to the corresponding virtual and physical
    /// data addresses. Return 0 if last instruction was not a ld/st instruction
    /// leaving virtAddr and physAddr unmodified. The value of physAddr will be zero
    /// if virtual to physical translation encountered an exception.
    unsigned lastLdStAddress(uint64_t& virtAddr, uint64_t& physAddr) const
    {
      if (ldStSize_ == 0)
	return 0;
      virtAddr = ldStAddr_;
      physAddr = ldStPhysAddr1_;
      return ldStSize_;
    }

    /// Return the size of the last ld/st instruction or 0 if last
    /// instruction was not a ld/st.
    unsigned lastLdStSize() const
    { return ldStSize_; }

    /// Return true if last branch instruction was taken.
    bool lastBranchTaken() const
    { return lastBranchTaken_; }

    /// Return true if last instruction is a ld/st instruction, setting misal
    /// to true if the memory access was misaligned
    bool misalignedLdSt(bool& misal) const
    {
      if (ldStSize_ == 0)
        return false;
      misal = misalignedLdSt_;
      return true;
    }

    /// Read instruction at given virtual address. Return true on success and false if
    /// address does not translate or if physical address is out of bounds. If successful
    /// set paddr to the translated address.
    bool readInst(uint64_t vaddr, uint64_t& paddr, uint32_t& instr);

    /// Similar to above but does not expose physical address.
    bool readInst(uint64_t vaddr, uint32_t& instr);

    /// Set instruction count limit: When running with tracing the
    /// run and the runUntil methods will stop if the retired instruction
    /// count (true count and not value of minstret) reaches or exceeds
    /// the limit.
    void setInstructionCountLimit(uint64_t limit)
    { instCountLim_ = limit; }

    /// Return current instruction count limit.
    uint64_t getInstructionCountLimit() const
    { return instCountLim_; }

    /// Reset executed instruction count.
    void setInstructionCount(uint64_t count)
    { instCounter_ = count; }

    /// Get executed instruction count.
    uint64_t getInstructionCount() const
    { return instCounter_; }

    /// Get the time CSR value.
    uint64_t getTime() const
    { return time_; }

    /// Return count of traps (exceptions or interrupts) seen by this
    /// hart.
    uint64_t getTrapCount() const
    { return exceptionCount_ + interruptCount_; }

    /// Return count of exceptions seen by this hart.
    uint64_t getExceptionCount() const
    { return exceptionCount_; }

    /// Return count of interrupts seen by this hart.
    uint64_t getInterruptCount() const
    { return interruptCount_; }

    /// Set pre and post to the count of "before"/"after" triggers
    /// that tripped by the last executed instruction.
    void countTrippedTriggers(unsigned& pre, unsigned& post) const
    { csRegs_.countTrippedTriggers(pre, post); }

    /// Set change to the components of the given trigger that were changed by the last
    /// executed instruction. Each entry is a component number (e.g. TDATA1, TINFO, ...)
    /// with the corresponding value.
    void getTriggerChange(URV trigger, std::vector<std::pair<CsrNumber, uint64_t>>& change) const
    { csRegs_.getTriggerChange(trigger, change); }

    /// Enable collection of instruction frequencies.
    void enableInstructionFrequency(bool b);

    /// Enable/disable trapping of arithmetic vector instruction when
    /// vstart is non-zero.
    void enableTrapNonZeroVstart(bool flag)
    { trapNonZeroVstart_ = flag; }

    /// Enable/disable the c (compressed) extension.
    void enableRvc(bool flag)
    { enableExtension(RvExtension::C, flag); csRegs_.enableRvc(flag); }

    /// Enable/disable the f (floating point) extension.
    void enableRvf(bool flag);

    /// Enable/disable the d (double-precision floating point) extension.
    void enableRvd(bool flag)
    { enableExtension(RvExtension::D, flag); }

    /// Enable/disable the supervisor timer compare extension (sstc).
    void enableRvsstc(bool flag)
    { enableExtension(RvExtension::Sstc, flag); csRegs_.enableSstc(flag); }

    /// Enable/disable counter overflow extension (sscofpmf)
    void enableSscofpmf(bool flag)
    { csRegs_.enableSscofpmf(flag); }

    /// Enable/disbale smstateen extension.
    void enableSmstateen(bool flag)
    { csRegs_.enableSmstateen(flag); }

    /// Enable/disable the resumable non maskable interrupt (Smrnmi) extension.
    void enableSmrnmi(bool flag)
    { enableExtension(RvExtension::Smrnmi, flag); csRegs_.enableSmrnmi(flag); }

    /// Enable/disable ssnpm extension.
    void enableSsnpm(bool flag)
    { enableExtension(RvExtension::Ssnpm, flag); csRegs_.enableSsnpm(flag); }

    /// Enable/disable smnpm extension.
    void enableSmnpm(bool flag)
    { enableExtension(RvExtension::Smnpm, flag); csRegs_.enableSmnpm(flag); }

    /// Enable/disable Zicntr extension.
    void enableZicntr(bool flag)
    { enableExtension(RvExtension::Zicntr, flag); csRegs_.enableZicntr(flag); }

    /// Enable/disable Zihpm extension.
    void enableZihpm(bool flag)
    { enableExtension(RvExtension::Zihpm, flag); csRegs_.enableZihpm(flag); }

    /// Enable/disable zkr extension.
    void enableZkr(bool flag)
    { enableExtension(RvExtension::Zkr, flag); csRegs_.enableZkr(flag); }

    /// Put this hart in debug mode setting the DCSR cause field to
    /// the given cause. Set the debug pc (DPC) to the given pc.
    void enterDebugMode_(DebugModeCause cause, URV pc);

    /// Put this hart in debug mode setting the DCSR cause field to
    /// either DEBUGGER or SETP depending on the step bit of DCSR.
    /// Set the debug pc (DPC) to the given pc.
    void enterDebugMode(URV pc);

    /// True if in debug mode.
    bool inDebugMode() const
    { return debugMode_; }

    /// True if DCSR.step is on.
    bool hasDcsrStep() const
    { return dcsrStep_; }

    /// Take this hart out of debug mode.
    void exitDebugMode();

    /// Print collected instruction frequency to the given file.
    void reportInstructionFrequency(FILE* file) const;

    /// Print collected trap stats to the given file.
    void reportTrapStat(FILE* file) const;

    /// Print collected load-reserve/store-conditional stats on the given file.
    void reportLrScStat(FILE* file) const;

    /// Reset trace data (items changed by the execution of an
    /// instruction.)
    void clearTraceData();

    /// Enable debug-triggers. Without this, triggers will not trip and will not cause
    /// exceptions.
    void enableTriggers(bool flag)
    { enableTriggers_ = flag; csRegs_.enableTriggers(flag);  }

    /// Enable/disable firing of triggers in machine mode when interrupts are enabled.
    void enableMmodeTriggersWithIe(bool flag)
    { csRegs_.enableMmodeTriggersWithIe(flag); }

    /// Enable performance counters (count up for some enabled performance counters when
    /// their events do occur).
    void enablePerformanceCounters(bool flag)
    { enableCounters_ = flag;  }

    /// Enable gdb-mode.
    void enableGdb(bool flag)
    { enableGdb_ = flag; }

    /// Open TCP socket for gdb
    bool openTcpForGdb();

    /// Set TCP port for gdb
    void setGdbTcpPort(int port)
    { gdbTcpPort_ = port; }

    /// Enable use of ABI register names (e.g. sp instead of x2) in
    /// instruction disassembly.
    void enableAbiNames(bool flag)
    { disas_.enableAbiNames(flag); }

    /// Return true if ABI register names are enabled.
    bool abiNames() const
    { return disas_.abiNames(); }

    /// Enable emulation of newlib system calls.
    void enableNewlib(bool flag)
    { newlib_ = flag; }

    /// Enable emulation of Linux system calls.
    void enableLinux(bool flag)
    { linux_ = flag; syscall_.enableLinux(flag); }

    /// For Linux emulation: Set initial target program break to the
    /// RISCV page address larger than or equal to the given address.
    void setTargetProgramBreak(URV addr);

    /// For Linux emulation: Put the program arguments and user-specified
    /// environment variables on the stack suitable for calling the
    /// target program main from _start. Return true on success and
    /// false on failure (not all stack area required is writable).
    bool setTargetProgramArgs(const std::vector<std::string>& args,
                              const std::vector<std::string>& envVars);

    /// Return physical memory attribute region of a given address.
    Pma getPma(uint64_t addr) const
    { return memory_.pmaMgr_.getPma(addr); }

    /// Return true if given extension is statically enabled (enabled my
    /// --isa but may be turned off by the MSTATUS/MISA CSRs).
    bool hasIsaExtension(RvExtension ext) const
    { return isa_.isEnabled(ext); }

    /// Return true if rv32f (single precision floating point)
    /// extension is enabled in this hart.
    bool isRvf() const
    { return extensionIsEnabled(RvExtension::F); }

    /// Return true if the half-precision floating point extension is
    /// enabled.
    bool isRvzfh() const
    { return extensionIsEnabled(RvExtension::Zfh); }

    /// Return true if the minimal half-precision floating point
    /// extension is enabled.
    bool isRvzfhmin() const
    { return extensionIsEnabled(RvExtension::Zfhmin); }

    /// Return true if the scalar BF16 converts extension is enabled.
    bool isRvzfbfmin() const
    { return extensionIsEnabled(RvExtension::Zfbfmin); }

    /// Return true if rv64d (double precision floating point)
    /// extension is enabled in this hart.
    bool isRvd() const
    { return extensionIsEnabled(RvExtension::D); }

    /// Return true if the zknd extension (crypto nist decryption) is enabled.
    bool isRvzknd() const
    { return extensionIsEnabled(RvExtension::Zknd); }

    /// Return true if the zkne extension (crypto nist encryption) is enabled.
    bool isRvzkne() const
    { return extensionIsEnabled(RvExtension::Zkne); }

    /// Return true if the zksed extension (crypto sm4 block cipher) is enabled.
    bool isRvzksed() const
    { return extensionIsEnabled(RvExtension::Zksed); }

    /// Return true if the zksh extension (crypto sm3 block hash) is enabled.
    bool isRvzksh() const
    { return extensionIsEnabled(RvExtension::Zksh); }

    /// Return true if the zknh extension (crypto nist hash) is enabled.
    bool isRvzknh() const
    { return extensionIsEnabled(RvExtension::Zknh); }

    /// Return true if the zbkb extension (crypto bit manip) is enabled.
    bool isRvzbkb() const
    { return extensionIsEnabled(RvExtension::Zbkb); }

    /// Return true if the zbkx extension (crypto bit manip) is enabled.
    bool isRvzbkx() const
    { return extensionIsEnabled(RvExtension::Zbkx); }

    /// Return true if the svinval extension (TLB invalidate) is enabled.
    bool isRvsvinval() const
    { return extensionIsEnabled(RvExtension::Svinval); }

    /// Return true if the zicbom extension (cache block management) is enabled.
    bool isRvzicbom() const
    { return extensionIsEnabled(RvExtension::Zicbom); }

    /// Return true if the zicboz extension (cache block zero) is enabled.
    bool isRvzicboz() const
    { return extensionIsEnabled(RvExtension::Zicboz); }

    /// Return true if the zicbop extension (cache block prefetch) is enabled.
    bool isRvzicbop() const
    { return extensionIsEnabled(RvExtension::Zicbop); }

    /// Return true if the zawrs extension (wait reservation store) is enabled.
    bool isRvzawrs() const
    { return extensionIsEnabled(RvExtension::Zawrs); }

    /// Return true if the zmmul extension (multiply) is enabled.
    bool isRvzmmul() const
    { return extensionIsEnabled(RvExtension::Zmmul); }

    /// Return true if rv64e (embedded) extension is enabled in this hart.
    bool isRve() const
    { return extensionIsEnabled(RvExtension::E); }

    /// Return true if rv64 (64-bit option) extension is enabled in
    /// this hart.
    static constexpr bool isRv64()
    { return rv64_; }

    /// Return true if rvm (multiply/divide) extension is enabled in
    /// this hart.
    bool isRvm() const
    { return extensionIsEnabled(RvExtension::M); }

    /// Return true if rvc (compression) extension is enabled in this
    /// hart.
    bool isRvc() const
    { return extensionIsEnabled(RvExtension::C); }

    /// Return true if rva (atomic) extension is enabled in this hart.
    bool isRva() const
    { return extensionIsEnabled(RvExtension::A); }

    /// Return true if rvb (bit-manup) extension is enabled in this hart.
    bool isRvb() const
    { return extensionIsEnabled(RvExtension::B); }

    /// Return true if rvs (supervisor-mode) extension is enabled in this
    /// hart.
    bool isRvs() const
    { return extensionIsEnabled(RvExtension::S); }

    /// Return true if rvh (hypervisor) extension is enabled in this hart.
    bool isRvh() const
    { return extensionIsEnabled(RvExtension::H); }

    /// Return true if rvu (user-mode) extension is enabled in this
    /// hart.
    bool isRvu() const
    { return extensionIsEnabled(RvExtension::U); }

    /// Return true if rvv (vector) extension is enabled in this hart.
    bool isRvv() const
    { return extensionIsEnabled(RvExtension::V); }

    /// Return true if rvn (user-mode-interrupt) extension is enabled
    /// in this hart.
    bool isRvn() const
    { return extensionIsEnabled(RvExtension::N); }

    /// Return true if zba extension is enabled in this hart.
    bool isRvzba() const
    { return extensionIsEnabled(RvExtension::Zba); }

    /// Return true if zbb extension is enabled in this hart.
    bool isRvzbb() const
    { return extensionIsEnabled(RvExtension::Zbb); }

    /// Return true if zbc extension is enabled in this hart.
    bool isRvzbc() const
    { return extensionIsEnabled(RvExtension::Zbc); }

    /// Return true if zbs extension is enabled in this hart.
    bool isRvzbs() const
    { return extensionIsEnabled(RvExtension::Zbs); }

    /// Return true if the half-precision vector floating point
    /// extension is enabled.
    bool isRvzvfh() const
    { return extensionIsEnabled(RvExtension::Zvfh); }

    /// Return true if the minimal half-precision vector floating
    /// point extension is enabled.
    bool isRvzvfhmin() const
    { return extensionIsEnabled(RvExtension::Zvfhmin); }

    /// Return true if the vector bfloat conversions extension is
    /// enabled.
    bool isRvzvfbfmin() const
    { return extensionIsEnabled(RvExtension::Zvfbfmin); }

    /// Return true if the vector BF16 widening mul-add extension
    /// extension is enabled.
    bool isRvzvfbfwma() const
    { return extensionIsEnabled(RvExtension::Zvfbfwma); }

    /// Return true if the supervisor timer compare extension is enabled.
    bool isRvsstc() const
    { return extensionIsEnabled(RvExtension::Sstc); }

    /// Return true if the bit-manip vector extension zvbb is enabled.
    bool isRvzvbb() const
    { return extensionIsEnabled(RvExtension::Zvbb); }

    /// Return true if the bit-manip vector extension zvbc is enabled.
    bool isRvzvbc() const
    { return extensionIsEnabled(RvExtension::Zvbc); }

    /// Return true if the vector zvkg extension is enabled.
    bool isRvzvkg() const
    { return extensionIsEnabled(RvExtension::Zvkg); }

    /// Return true if the vector zvkned extension is enabled.
    bool isRvzvkned() const
    { return extensionIsEnabled(RvExtension::Zvkned); }

    /// Return true if the vector secure hash zvknha extension is enabled.
    bool isRvzvknha() const
    { return extensionIsEnabled(RvExtension::Zvknha); }

    /// Return true if the vector secure hash zvknhb extension is enabled.
    bool isRvzvknhb() const
    { return extensionIsEnabled(RvExtension::Zvknhb); }

    /// Return true if the vector ShangMi extension is enabled.
    bool isRvzvksed() const
    { return extensionIsEnabled(RvExtension::Zvksed); }

    /// Return true if the vector Zvksh extension is enabled.
    bool isRvzvksh() const
    { return extensionIsEnabled(RvExtension::Zvksh); }

    /// Return true if the vector Zvkb extension is enabled.
    bool isRvzvkb() const
    { return extensionIsEnabled(RvExtension::Zvkb); }

    /// Return true if the zicond extension is enabled.
    bool isRvzicond() const
    { return extensionIsEnabled(RvExtension::Zicond); }

    /// Return true if the zcb extension is enabled.
    bool isRvzcb() const
    { return extensionIsEnabled(RvExtension::Zcb); }

    /// Return true if the zcb extension is enabled.
    bool isRvzfa() const
    { return extensionIsEnabled(RvExtension::Zfa); }

    /// Return true if the AIA extension is enabled.
    bool isRvaia() const
    { return extensionIsEnabled(RvExtension::Smaia); }

    /// Return true if the Zacas extension is enabled.
    bool isRvzacas() const
    { return extensionIsEnabled(RvExtension::Zacas); }

    bool isRvzimop() const
    { return extensionIsEnabled(RvExtension::Zimop); }

    bool isRvzcmop() const
    { return extensionIsEnabled(RvExtension::Zcmop); }

    bool isRvSsnpm() const
    { return extensionIsEnabled(RvExtension::Ssnpm); }

    bool isRvSmnpm() const
    { return extensionIsEnabled(RvExtension::Smnpm); }

    /// Return true if current program is considered finished (either
    /// reached stop address or executed exit limit).
    bool hasTargetProgramFinished() const
    { return targetProgFinished_; }

    /// Mark target program as finished/non-finished based on flag.
    void setTargetProgramFinished(bool flag)
    { targetProgFinished_ = flag; }

    /// Make atomic memory operations illegal/legal for non cacheable
    /// memory based on the value of flag (true/false).
    void setAmoInCacheableOnly(bool flag)
    { amoInCacheableOnly_ = flag; }

    /// Make load/store instructions take an exception if the base
    /// address (value in rs1) and the effective address refer to
    /// regions of different types.
    void setEaCompatibleWithBase(bool flag)
    { eaCompatWithBase_ = flag; }

    uint64_t getMemorySize() const
    { return memory_.size(); }

    /// Return the index of this hart within the system. Harts are
    /// assigned indices 0 to m*n - 1 where m is the number of cores
    /// and n is the number of harts per core.
    unsigned sysHartIndex() const
    { return hartIx_; }

    /// Return the value of the MHARTID CSR.
    URV hartId() const
    { return peekCsr(CsrNumber::MHARTID); }

    /// Tie the shared CSRs in this hart to the corresponding CSRs in
    /// the target hart making them share the same location for their
    /// value.
    void tieSharedCsrsTo(Hart<URV>& target)
    { return csRegs_.tieSharedCsrsTo(target.csRegs_); }

    /// Record given CSR number for later reporting of CSRs modified by
    /// an instruction.
    void recordCsrWrite(CsrNumber csr)
    { csRegs_.recordWrite(csr); }

    /// Enable/disable performance counters.
    void setPerformanceCounterControl(uint32_t control)
    {
      prevPerfControl_ = perfControl_;
      perfControl_ = control;
    }

    /// Redirect the given output file descriptor (typically that of
    /// stdout or stderr) to the given file. Return true on success
    /// and false on failure.
    bool redirectOutputDescriptor(int fd, const std::string& file);

    /// Redirect the given input file descriptor (typically that of
    /// stdin) to the given file. Return true on success and false on
    /// failure.
    bool redirectInputDescriptor(int fd, const std::string& file);

    /// Rollback destination register of most recent dev/rem
    /// instruction.  Return true on success and false on failure (no
    /// unrolled div/rem inst). This is for the test-bench.
    bool cancelLastDiv();

    // returns true if there is any valid LR reservation
    bool hasLr() const
    { return memory_.hasLr(hartIx_); }

    /// Cancel load reservation held by this hart (if any). Mark
    /// cause of cancellation which persists until overwritten
    /// by another cancelLr or until a new reservation is made.
    void cancelLr(CancelLrCause cause)
    { memory_.invalidateLr(hartIx_, cause); }

    /// Return the cause of the last LR reservation cancellation.
    CancelLrCause cancelLrCause() const
    { return memory_.cancelLrCause(hartIx_); }

    /// Cancel load reservations in all other harts.
    void cancelOtherHartsLr(uint64_t physAddr)
    {
      uint64_t lineAddr = physAddr - (physAddr % lrResSize_);
      memory_.invalidateOtherHartLr(hartIx_, lineAddr, lrResSize_);
    }

    /// Report the files opened by the target RISCV program during
    /// current run.
    void reportOpenedFiles(std::ostream& out)
    { syscall_.reportOpenedFiles(out); }

    /// Enable forcing a timer interrupt every n instructions.
    /// Nothing is forced if n is zero.
    void setupPeriodicTimerInterrupts(uint64_t n)
    {
      alarmInterval_ = n;
      alarmLimit_ = n? instCounter_ + alarmInterval_ : ~uint64_t(0);
    }

    /// Return the memory page size (e.g. 4096).
    uint64_t pageSize() const
    { return memory_.pageSize(); }

    /// Set the physical memory protection grain size (which must
    /// be a power of 2 greater than or equal to 4).
    bool configMemoryProtectionGrain(uint64_t size);

    /// Set the max number of guest external interrupts.
    bool configGuestInterruptCount(unsigned n);

    /// Set timeout of wfi instruction. A non-zero timeout will make wfi succeed
    /// if it can succeed within a bound timeout.
    void setWfiTimeout(uint64_t t)
    { wfiTimeout_ = t; }

    /// Enable user mode.
    void enableUserMode(bool flag)
    { enableExtension(RvExtension::U, flag); csRegs_.enableUserMode(flag); }

    /// Enable supervisor mode.
    void enableSupervisorMode(bool flag)
    { enableExtension(RvExtension::S, flag); csRegs_.enableSupervisorMode(flag); }

    /// Enable hypervisor mode.
    void enableHypervisorMode(bool flag)
    { enableExtension(RvExtension::H, flag); csRegs_.enableHypervisorMode(flag); }

    /// Enable vector extension.
    void enableVectorExtension(bool flag);

    /// Enable Advance Interrupt Architecture (AIA) extension.
    void enableAiaExtension(bool flag)
    { isa_.enable(RvExtension::Smaia, flag); enableExtension(RvExtension::Smaia, flag); csRegs_.enableAia(flag); }

    /// For privileged spec v1.12, we clear mstatus.MPRV if xRET
    /// causes us to enter a privilege mode not Machine.
    void enableClearMprvOnRet(bool flag)
    { clearMprvOnRet_ = flag; }

    /// Make hfence.gvma ignore guest physical addresses (over-invalidate) when flag is
    /// true.
    void hfenceGvmaIgnoresGpa(bool flag)
    { hfenceGvmaIgnoresGpa_ = flag; }

    /// Clear MTVAL on illegal instruction exception if flag is true.
    /// Otherwise, set MTVAL to the opcode of the illegal instruction.
    void enableClearMtvalOnIllInst(bool flag)
    { clearMtvalOnIllInst_ = flag; }

    /// Clear MTVAL on illegal instruction exception if flag is true.
    /// Otherwise, set MTVAL to the opcode of the illegal instruction.
    void enableClearMtvalOnEbreak(bool flag)
    { clearMtvalOnEbreak_ = flag; }

    /// Disable clearing of reservation set after xRET
    void enableCancelLrOnTrap(bool flag)
    { cancelLrOnTrap_ = flag; }

    /// Enable/disable misaligned access. If disabled then misaligned
    /// ld/st will trigger an exception.
    void enableMisalignedData(bool flag)
    {
      misalDataOk_ = flag;
      memory_.pmaMgr_.enableMisalignedData(flag);
    }

    /// Make misaligned exceptions have priority over page/access fault.
    void misalignedExceptionHasPriority(bool flag)
    { misalHasPriority_ = flag; }

    /// Return current privilege mode.
    PrivilegeMode privilegeMode() const
    { return privMode_; }

    /// Defer interrupts received (to be taken later). This is for testbench
    /// to control when interrupts are handled without affecting architectural state.
    void setDeferredInterrupts(URV val)
    { deferredInterrupts_ = val; }

    /// Return the mask of deferred interrupts.
    URV deferredInterrupts()
    { return deferredInterrupts_; }

    /// This is for performance modeling. Enable a highest level cache
    /// with given size, line size, and set associativity.  Any
    /// previously enabled cache is deleted.  Return true on success
    /// and false if the sizes are not powers of 2 or if any of them
    /// is zero, or if they are too large (more than 64 MB for cache
    /// size, more than 1024 for line size, and more than 64 for
    /// associativity). This has no impact on functionality.  Cache
    /// consists of l lines (l = size/lineSize) organized in s sets (s
    /// = l/setSize) each set contains setSize lines.  An address is
    /// mapped to a memory-line ml (ml = address/lineSize) which is
    /// mapped to a set index (ml % s), then all the lines in that set
    /// are searched for that memory-line.
    bool configureCache(uint64_t size, unsigned lineSize,
                        unsigned setSize);

    /// Delete currently configured cache.
    void deleteCache();

    /// Set number of TLB entries.
    void setTlbSize(unsigned size)
    { virtMem_.setTlbSize(size); }

    /// Debug method: print address translation table.
    void printPageTable(std::ostream& out) const
    { virtMem_.printPageTable(out); }

    /// Trace the last n branches to the given file. No tracing is
    /// done if n is 0.
    void traceBranches(const std::string& file, uint64_t n)
    { branchTraceFile_ = file; branchBuffer_.resize(n); }

    /// Write the collected branch traces to the file at the given path.
    bool saveBranchTrace(const std::string& path);

    /// Set behavior of first access to a virtual memory page: Either
    /// we take a page fault (flag is true) or we update the A/D bits
    /// of the PTE. When V is on, this applies to the first stage (VS)
    /// translation.
    void setFaultOnFirstAccess(bool flag)
    { virtMem_.setFaultOnFirstAccess(flag); }

    /// Similar to setFaultOnFirstAccess but applies to the first stage
    /// of 2-stage translation.
    void setFaultOnFirstAccessStage1(bool flag)
    { virtMem_.setFaultOnFirstAccessStage1(flag); }

    /// Similar to setFaultOnFirstAccess but applies to the second stage
    /// of 2-stage translation.
    void setFaultOnFirstAccessStage2(bool flag)
    { virtMem_.setFaultOnFirstAccessStage2(flag); }

    /// Translate virtual address without updating TLB or updating/checking A/D bits of
    /// PTE. Return ExceptionCause::NONE on success or fault/access exception on
    /// failure. If succesful set pa to the physical address.
    ExceptionCause transAddrNoUpdate(uint64_t va, PrivilegeMode pm,
				     bool twoStage, bool r,
				     bool w, bool x, uint64_t& pa)
    { return virtMem_.transAddrNoUpdate(va, pm, twoStage, r, w, x, pa); }

    /// Return the paging mode before last executed instruction.
    VirtMem::Mode lastPageMode() const
    { return lastPageMode_; }

    /// Return the VS paging mode before last executed instruction.
    VirtMem::Mode lastVsPageMode() const
    { return lastVsPageMode_; }

    /// Return the 2nd stage paging mode before last executed instruction.
    VirtMem::Mode lastPageModeStage2() const
    { return lastPageModeStage2_; }

    /// Return the current paging mode
    VirtMem::Mode pageMode() const
    { return virtMem_.mode(); }

    /// Return the current virtual mode (V bit).
    bool virtMode() const
    { return virtMode_; }

    /// Return the virtual mode before last executed instruction.
    bool lastVirtMode() const
    { return lastVirt_; }

    /// Return the number of page table walks of the last
    /// executed instruction
    unsigned getNumPageTableWalks(bool isInstr) const
    { return isInstr? virtMem_.numFetchWalks() : virtMem_.numDataWalks(); }

    /// Fill the addrs vector (cleared on entry) with the addresses of
    /// instruction/data the page table entries referenced by the
    /// instruction/data page table walk of the last executed
    /// instruction or make it empty if no page table walk took place.
    void getPageTableWalkAddresses(bool isInstr, unsigned ix, std::vector<VirtMem::WalkEntry>& addrs) const
    { addrs = isInstr? virtMem_.getFetchWalks(ix) : virtMem_.getDataWalks(ix); }

    /// Get the page table entries of the page table walk of the last
    /// executed instruction (see getPageTableWAlkAddresses).
    void getPageTableWalkEntries(bool isInstr, unsigned ix, std::vector<uint64_t>& ptes) const
    {
      const auto& addrs = isInstr? virtMem_.getFetchWalks(ix) : virtMem_.getDataWalks(ix);
      ptes.clear();
      for (const auto& addr : addrs)
	{
        if (addr.type_ == VirtMem::WalkEntry::Type::PA)
          {
            URV pte = 0;
            peekMemory(addr.addr_, pte, true);
            ptes.push_back(pte);
          }
	}
    }

    /// Get the page table walk of the last executed instruction.
    void getPageTableWalkEntries(bool isInstr, std::vector<std::vector<VirtMem::WalkEntry>>& walks) const
    {
      walks.clear();
      walks = isInstr? virtMem_.getFetchWalks() : virtMem_.getDataWalks();
    }

    /// Return PMP manager associated with this hart.
    const auto& pmpManager() const
    { return pmpManager_; }

    /// Return PMA manager associated with this hart.
    const auto& pmaManager() const
    { return memory_.pmaMgr_; }

    /// Get the PMP registers accessed by last executed instruction
    void getPmpsAccessed(std::vector<PmpManager::PmpTrace>& pmps) const
    {
      pmps.clear();
      pmps = pmpManager_.getPmpTrace();
    }

    /// Get PMP associated with an address
    Pmp getPmp(uint64_t addr) const
    { return pmpManager_.getPmp(addr); }

    /// Print current PMP map matching a particular address.
    void printPmps(std::ostream& os, uint64_t address) const
    { pmpManager_.printPmps(os, address); }

    /// Print current PMP map.
    void printPmps(std::ostream& os) const
    { pmpManager_.printPmps(os); }

    // Get the PMAs accessed by the last executed instruction
    void getPmasAccessed(std::vector<PmaManager::PmaTrace>& pmas) const
    {
      pmas.clear();
      pmas = memory_.pmaMgr_.getPmaTrace();
    }

    /// Print current PMA map matching a particular address.
    void printPmas(std::ostream& os, uint64_t address) const
    { memory_.pmaMgr_.printPmas(os, address); }

    /// Print current PMA map.
    void printPmas(std::ostream& os) const
    { memory_.pmaMgr_.printPmas(os); }

    /// Invalidate whole cache.
    void invalidateDecodeCache();

    /// Register a callback to be invoked before a CSR instruction
    /// accesses its target CSR. Callback is invoked with the
    /// hart-index (hart index in system) and CSR number. This is for
    /// the SOC (system on chip) model.
    void registerPreCsrInst(std::function<void(unsigned, CsrNumber)> callback)
    { preCsrInst_ = std::move(callback); }

    /// Register a callback to be invoked after a CSR accesses its
    /// target CSR, or in the case of an exception, after the CSR
    /// instruction takes the exception.  Callback is invoked with the
    /// hart-index (hart index in system) and CSR number. This is for
    /// the SOC model.
    void registerPostCsrInst(std::function<void(unsigned, CsrNumber)> callback)
    { postCsrInst_ = std::move(callback); }

    /// Callback to invoke before the execution of an instruction.
    void registerPreInst(std::function<void(Hart<URV>&, bool&, bool&)> callback)
    { preInst_ = std::move(callback); }

    /// Define physical memory attribute region at index ix. Region
    /// addresses are between low and high inclusive. To define a
    /// 1024-byte region at address zero we would set low to zero and
    /// high to 1023. Region are checked in order and the first
    /// matching region applies.
    bool definePmaRegion(unsigned ix, uint64_t low, uint64_t high, Pma pma)
    { return memory_.pmaMgr_.defineRegion(ix, low, high, pma); }

    /// Return true if given address is within a memory mapped register.
    bool isMemMappedReg(size_t addr) const
    { return memory_.pmaMgr_.isMemMappedReg(addr); }

    /// Mark as invalid entry with the given index.
    void invalidatePmaEntry(unsigned ix)
    { memory_.pmaMgr_.invalidateEntry(ix); }

    /// Called after a chance to a PMACFG CSR. Return true on success
    /// and false if num is not that of PMACFG CSR.
    bool processPmaChange(CsrNumber num);

    /// Associate a mask with the word-aligned word at the given
    /// address. Return true on success and flase if given address is
    /// not in a memory mapped region. The size must be 4 or 8. The
    /// address must be word/double-word aligned if size is 4/8.
    bool setMemMappedMask(uint64_t addr, uint64_t mask, unsigned size)
    { return memory_.pmaMgr_.setMemMappedMask(addr, mask, size); }

    /// Unpack the memory protection information defined by the given
    /// physical memory protection entry (entry 0 corresponds to
    /// PMPADDR0, ... 63 to PMPADDR63). Return true on success setting
    /// type, mode, locked, low and high to the corresponding values
    /// associated with the entry. If entry mode is off the low and
    /// high will be set to zero. Return false on failure (entry
    /// index out of bounds or corresponding CSR not implemented).
    bool unpackMemoryProtection(unsigned entryIx, Pmp::Type& type,
                                Pmp::Mode& mode, bool& locked,
                                uint64_t& low, uint64_t& high) const;


    /// an emulated system call. If addr is zero, no slamming is done.
    void defineSyscallSlam(URV addr)
    { syscallSlam_ = addr; }

    /// Return the address set by defineSyscallSlam.
    URV syscallSlam() const
    { return syscallSlam_; }

    /// Force floating point rounding mode to the given mode
    /// regardless of the setting of the FRM CSR. This is useful for
    /// testing/bringup.
    void forceRoundingMode(RoundingMode mode)
    { forcedRounding_ = mode; forceRounding_ = true; }

    /// Enable logging in CSV (comma separated values) format.
    void enableCsvLog(bool flag)
    { csvTrace_ = flag; }

    /// Enable basic block stats if given file is non-null. Print
    /// stats every instCount instructions.
    void enableBasicBlocks(FILE* file, uint64_t instCount)
    { bbFile_ = file; bbLimit_ = instCount; }

    /// Enable memory consistency model.
    void setMcm(std::shared_ptr<Mcm<URV>> mcm);

    typedef TT_PERF::PerfApi PerfApi;

    /// Enable performance model API.
    void setPerfApi(std::shared_ptr<PerfApi> perfApi);

    /// Enable instruction line address tracing.
    void enableInstructionLineTrace(bool flag)
    { instrLineTrace_ = flag; }

    /// Enable instruction line address tracing.
    void enableDataLineTrace(bool flag)
    { dataLineTrace_ = flag; }

    /// Enable/disable page-table-walk info in log.
    void tracePtw(bool flag)
    { tracePtw_ = flag; }

    /// Enable/disable PMP access trace
    void tracePmp(bool flag)
    { pmpManager_.enableTrace(flag); }

    /// Enable/disable PMA access trace
    void tracePma(bool flag)
    { memory_.pmaMgr_.enableTrace(flag); }

    /// Enable/disable top-of-range mode in pmp configurations.
    void enablePmpTor(bool flag)
    { csRegs_.enablePmpTor(flag); }

    /// Enable/disable top-of-range mode in pmp configurations.
    void enablePmpNa4(bool flag)
    { csRegs_.enablePmpNa4(flag); }

    Syscall<URV>& getSyscall()
    { return syscall_; }

    /// Save snapshot of registers (PC, integer, floating point, CSR) into file
    bool saveSnapshotRegs(const std::string& path);

    // Load snapshot of registers (PC, integer, floating point, CSR) into file
    bool loadSnapshotRegs(const std::string& path);

    // Define the additional delay to add to the timecmp register
    // before the timer expires. This is relevant to the clint.
    void setTimeShift(unsigned shift)
    { timeShift_ = shift; }

    // Define time scaling factor such that the time value increment period
    // is scaled down by 2^N.
    void setTimeDownSample(unsigned n)
    { timeDownSample_ = n; }

    /// Return true if external interrupts are enabled and one or more
    /// external interrupt that is pending is also enabled. Set cause
    /// to the type of interrupt if one is possible; otherwise, leave
    /// it unmodified. If more than one interrupt is possible, set
    /// cause to the possible interrupt with the highest priority.
    bool isInterruptPossible(InterruptCause& cause) const;

    /// Return true if this hart would take an interrupt if the MIP
    /// CSR were to have the given value. Do not change MIP, do not
    /// change processor state. If interrupt is possible, set cause
    /// to the interrupt cause; otherwise, leave cause unmodified.
    bool isInterruptPossible(URV mipValue, InterruptCause& cause) const;

    /// Configure this hart to set its program counter to the given
    /// addr on entering debug mode. If addr bits are all set, then
    /// the PC is not changed on entering debug mode.
    void setDebugParkLoop(URV addr)
    { debugParkLoop_ = addr; }

    /// Configure this hart to set its program counter to the given
    /// addr on encountering a trap (except breakpoint) during debug
    /// mode. If addr bits are all set, then the PC is not changed on
    /// a trap.
    void setDebugTrapAddress(URV addr)
    { debugTrapAddr_ = addr; }

    /// Return true a park loop is defined for debug mode.
    bool hasDebugParkLoop() const
    { return debugParkLoop_ != ~URV(0); }

    /// Associate given IMSIC with this hart and define the address
    /// space for all IMSICs in the system.
    void attachImsic(std::shared_ptr<TT_IMSIC::Imsic> imsic,
		     uint64_t mbase, uint64_t mend,
		     uint64_t sbase, uint64_t send,
		     std::function<bool(uint64_t, unsigned, uint64_t&)> readFunc,
		     std::function<bool(uint64_t, unsigned, uint64_t)> writeFunc,
                     bool trace)
    {
      imsic_ = imsic;
      imsicMbase_ = mbase; imsicMend_ = mend;
      imsicSbase_ = sbase; imsicSend_ = send;
      imsicRead_ = readFunc;
      imsicWrite_ = writeFunc;
      imsic_->enableTrace(trace);
      csRegs_.attachImsic(imsic);

      using IC = InterruptCause;
      imsic_->attachMInterrupt([this] (bool flag) {
          URV mipVal = csRegs_.peekMip();
          URV prev = mipVal;

          if (flag)
	    mipVal = mipVal | (URV(1) << URV(IC::M_EXTERNAL));
          else
	    mipVal = mipVal & ~(URV(1) << URV(IC::M_EXTERNAL));

          if (mipVal != prev)
            csRegs_.poke(CsrNumber::MIP, mipVal);
        });

      imsic_->attachSInterrupt([this] (bool flag) {
	  setSeiPin(flag);
        });

      imsic_->attachGInterrupt([this] (bool flag, unsigned guest) {
	  URV gip = csRegs_.peekHgeip();
	  gip = flag ? (gip | (URV(1) << guest)) :  (gip & ~(URV(1) << guest));
	  csRegs_.poke(CsrNumber::HGEIP, gip);

	  URV gie = csRegs_.peekHgeie();
	  URV gmask = URV(1) << URV(IC::G_EXTERNAL);
	  URV mipVal = csRegs_.peekMip();
          mipVal = (gip & gie) ? (mipVal | gmask) : (mipVal & ~gmask);

	  csRegs_.poke(CsrNumber::MIP, mipVal);
        });
    }

    void attachPci(std::shared_ptr<Pci> pci, uint64_t configBase, uint64_t mmioBase, uint64_t mmioSize)
    {
      pci_ = pci;
      pciConfigBase_ = configBase;
      pciConfigEnd_ = pciConfigBase_ + (1ULL << 28);
      pciMmioBase_ = mmioBase;
      pciMmioEnd_ = mmioBase + mmioSize;
    }

    /// Return true if given extension is enabled.
    constexpr bool extensionIsEnabled(RvExtension ext) const
    {
      return ext_enabled_.test(static_cast<std::size_t>(ext));
    }

    /// Post a software interrupt to this hart.
    void setSwInterrupt(uint8_t value)
    { swInterrupt_.value_ = value; }

    /// Fetch an instruction cache line.
    bool mcmIFetch(uint64_t addr)
    {
      auto fetchMem =  [this](uint64_t addr, uint32_t& value) -> bool {
	if (pmpEnabled_)
	  {
	    const Pmp& pmp = pmpManager_.accessPmp(addr, PmpManager::AccessReason::Fetch);
	    if (not pmp.isExec(privMode_))
	      return false;
	  }
	return this->memory_.readInst(addr, value);
      };
      bool ok = fetchCache_.addLine(addr, fetchMem);
      if (not ok)
	fetchCache_.removeLine(addr);
      return ok;
    }

    /// Evict an instruction cache line.
    bool mcmIEvict(uint64_t addr)
    { fetchCache_.removeLine(addr); return true; }

    /// Config vector engine for updating whole mask register for mask-producing
    /// instructions (if flag is false, we only update body and tail elements; otherwise,
    /// we update body, tail, and elements within VLEN beyond tail).
    void configVectorUpdateWholeMask(bool flag)
    { vecRegs_.configUpdateWholeMask(flag); }

    /// When flag is true, trap on invalid/unsupported vtype configurations in vsetvl,
    /// vsetvli, vsetivli. When flag is false, set vtype.vill instead.  .
    void configVectorTrapVtype(bool flag)
    { vecRegs_.configVectorTrapVtype(flag); }

    /// When flag is true, use binary tree reduction for vfredusum and vfwredusum.
    void configVectorFpUnorderedSumRed(bool flag)
    { vecRegs_.configVectorFpUnorderedSumRed(flag); }

    /// When flag is true, when VL > VLMAX reduce AVL to match VLMAX and write
    /// to VL. This only applies to vsetvl/vsetvli instructions.
    void configVectorLegalizeVsetvlAvl(bool flag)
    { vecRegs_.configVectorLegalizeVsetvlAvl(flag); }

    void configVectorLegalizeVsetvliAvl(bool flag)
    { vecRegs_.configVectorLegalizeVsetvliAvl(flag); }

    /// If flag is true, make VL/VSTART value a multiple of EGS in vector-crypto
    /// instructions that have EGS. Otherwise, trigger an exceptio if VL/VSTART is not a
    /// mulitple of EGS for such instrucions.
    void configVectorLegalizeForEgs(bool flag)
    { vecRegs_.configLegalizeForEgs(flag); }

    /// Support memory consistency model (MCM) instruction cache. Read 2 bytes from the
    /// given address (must be even) into inst. Return true on success.  Return false if
    /// the line of the given address is not in the cache.
    bool readInstFromFetchCache(uint64_t addr, uint16_t& inst) const
    { return fetchCache_.read(addr, inst); }

    /// Configure the mask defining which bits of a physical address must be zero for the
    /// address to be considered valid when STEE (static truested execution environment)
    /// is enabled. A bit set in the given mask must correspond to a zero bit in a physical
    /// address; otherwise, the STEE will deem the physical address invalid.
    void configSteeZeroMask(uint64_t mask)
    { stee_.configZeroMask(mask); }

    /// Configure the mask defining the bits of physical address that must be one for the
    /// address to be considered secure when STEE is enabled. For example if bit 55 of
    /// the given mask is 1, then an address with bit 55 set will be considered secure.
    void configSteeSecureMask(uint64_t mask)
    { stee_.configSecureMask(mask); }

    /// Configure the region of memory that is considered secure and that requires secure
    /// access when STEE is enabled. This is purely for testing as the secure region is a
    /// property of the platform.
    void configSteeSecureRegion(uint64_t low, uint64_t high)
    { stee_.configSecureRegion(low, high); }

    /// Enable STEE.
    void enableStee(bool flag)
    { steeEnabled_ = flag; csRegs_.enableStee(flag); }

    /// Return true if ACLINT is configured.
    bool hasAclint() const
    { return aclintSize_ > 0; }

    /// Set the ACLINT alarm to the given value.
    bool hasAclintTimer(uint64_t& addr) const
    {
      if (aclintMtimerStart_ < aclintMtimerEnd_)
	{
	  addr = aclintMtimerStart_;
	  return true;
	}
      return false;
    }

    /// Set the CLINT alarm to the given value.
    void setAclintAlarm(uint64_t value)
    {
      if (hasAclint())
	aclintAlarm_ = value;
    }

    /// Fetch an instruction from the given virtual address. Return ExceptionCause::None
    /// on success. Return exception cause on fail. If successful set pysAddr to the
    /// physical address corresponding to the given virtual address, gPhysAddr to the
    /// guest physical address (0 if not in VS/VU mode), and instr to the fetched
    /// instruction. If a fetch crosses a page boundary then physAddr2 will be the address
    /// of the other paget, otherwise physAddr2 will be the same as physAddr.
    ExceptionCause fetchInstNoTrap(uint64_t& virAddr, uint64_t& physAddr, uint64_t& physAddr2,
				   uint64_t& gPhysAddr, uint32_t& instr);

    bool isAclintAddr(uint64_t addr) const
    { return hasAclint() and addr >= aclintBase_ and addr < aclintBase_ + aclintSize_; }

    bool isAclintMtimeAddr(uint64_t addr) const
    { return addr >= aclintMtimeStart_ and addr < aclintMtimeEnd_; }

    bool isInterruptorAddr(uint64_t addr, unsigned size) const
    { return hasInterruptor_ and addr == interruptor_ and size == 4; }

    bool isImsicAddr(uint64_t addr) const
    {
      return (imsic_ and ((addr >= imsicMbase_ and addr < imsicMend_) or
			  (addr >= imsicSbase_ and addr < imsicSend_)));
    }

    bool isPciAddr(uint64_t addr) const
    {
      return (pci_ and ((addr >= pciConfigBase_ and addr < pciConfigEnd_) or
			(addr >= pciMmioBase_ and addr < pciMmioEnd_)));
    }

    /// Return true if there is one or more active performance counter (a counter that is
    /// assigned a valid event).
    bool hasActivePerfCounter() const
    { return csRegs_.mPerfRegs_.hasActiveCounter(); }

    /// Skip cancel-lr in wrs_sto/wrs_nto if flag is false. This is used in
    /// server/interactive mode where the driver (test-bench) will do an explicit
    /// cancel-lr at the right time.
    void setWrsCancelsLr(bool flag)
    { wrsCancelsLr_ = flag; }

    /// Set hart suspend state. If true, run will have no effect. If suspended,
    /// reset the resume time.
    void setSuspendState(bool flag, uint64_t timeout = 0)
    {
      suspended_ = flag;
      resumeTime_ = flag? time_ + timeout : 0;
    }

    bool isSuspended()
    { return suspended_; }

  protected:

    // Retun cached value of the mpp field of the mstatus CSR.
    PrivilegeMode mstatusMpp() const
    { return PrivilegeMode{mstatus_.bits_.MPP}; }

    // Retun cached value of the mprv field of the mstatus CSR.
    bool mstatusMprv() const
    { return mstatus_.bits_.MPRV; }

    /// Return true if the NMIE bit of NMSTATUS overrides the effect of
    /// MSTATUS.MPRV. See Smrnmi secton in RISCV privileged spec.
    bool nmieOverridesMprv() const
    {
      return (extensionIsEnabled(RvExtension::Smrnmi) and
	      MnstatusFields{csRegs_.peekMnstatus()}.bits_.NMIE == 0);
    }

    /// Return the effective privilege mode: if MSTATUS.MPRV is set then it is the
    /// privilege mode in MSTATUS.MPP
    PrivilegeMode effectivePrivilege() const
    {
      PrivilegeMode pm = privMode_;
      if (mstatusMprv() and not nmieOverridesMprv())
	pm = mstatusMpp();
      return pm;
    }

    /// Return the effective virtual mode: if MSTATUS.MPRV is set then it is the virtual
    /// mode in MSTATUS.MPV
    bool effectiveVirtualMode() const
    {
      bool virt = virtMode_;
      if (mstatusMprv() and not nmieOverridesMprv())
	virt = mstatus_.bits_.MPV;
      return virt;
    }

    /// Read an item that may span 2 physical pages. If pa1 is the
    /// same as pa2 then the item is in one page: do a simple read. If
    /// pa1 is different from pa2, then the item crosses a page
    /// boundary: read the most sig bytes from pa1 and the remaining
    /// bytes from pa2.
    template <typename LOAD_TYPE>
    void memRead(uint64_t pa1, uint64_t pa2, LOAD_TYPE& value)
    {
      if (pa1 == pa2)
	{
	  if (not memory_.read(pa1, value))
	    assert(0);
	  if (bigEnd_)
	    value = util::byteswap(value);
	  return;
	}

      unsigned size = sizeof(value);
      unsigned size1 = size - (pa1 & (size - 1));

      unsigned size2 = size - size1;

      value = 0;
      uint8_t byte = 0;
      unsigned destIx = 0;
      for (unsigned i = 0; i < size1; ++i, ++destIx)
	if (memory_.read(pa1 + i, byte))
	  value |= LOAD_TYPE(byte) << 8*destIx;
	else assert(0);
      for (unsigned i = 0; i < size2; ++i, ++destIx)
	if (memory_.read(pa2 + i, byte))
	  value |= LOAD_TYPE(byte) << 8*destIx;
	else assert(0);

      if (bigEnd_)
	value = util::byteswap(value);
    }

    /// Write an item that may span 2 physical pages. See memRead.
    template <typename STORE_TYPE>
    void memWrite(uint64_t pa1, uint64_t pa2, STORE_TYPE value)
    {
      if (bigEnd_)
	value = util::byteswap(value);

      if (pa1 == pa2)
	{
	  if (not memory_.write(hartIx_, pa1, value))
	    assert(0);
	  return;
	}
      unsigned size = sizeof(value);
      unsigned size1 = size - (pa1 & (size - 1));
      unsigned size2 = size - size1;

      if constexpr (sizeof(STORE_TYPE) > 1)
	{
	  for (unsigned i = 0; i < size1; ++i, value >>= 8)
	    if (not memory_.write(hartIx_, pa1 + i, uint8_t(value & 0xff)))
	      assert(0);
	  for (unsigned i = 0; i < size2; ++i, value >>= 8)
	    if (not memory_.write(hartIx_, pa2 + i, uint8_t(value & 0xff)))
	      assert(0);
	}
    }

    /// Peek an item that may span 2 physical pages. See memRead.
    template <typename LOAD_TYPE>
    void memPeek(uint64_t pa1, uint64_t pa2, LOAD_TYPE& value, bool usePma)
    {
      if (pa1 == pa2)
	{
	  memory_.peek(pa1, value, usePma);
	  return;
	}
      unsigned size = sizeof(value);
      unsigned size1 = size - (pa1 & (size - 1));
      unsigned size2 = size - size1;

      value = 0;
      uint8_t byte = 0;
      unsigned destIx = 0;
      for (unsigned i = 0; i < size1; ++i, ++destIx)
	if (memory_.peek(pa1 + i, byte, usePma))
	  value |= LOAD_TYPE(byte) << 8*destIx;
      for (unsigned i = 0; i < size2; ++i, ++destIx)
	if (memory_.peek(pa2 + i, byte, usePma))
	  value |= LOAD_TYPE(byte) << 8*destIx;
    }

    /// Get the data value for an out of order read (mcm or perfApi).
    bool getOooLoadValue(uint64_t va, uint64_t pa1, uint64_t pa2, unsigned size,
			 uint64_t& value);

    /// Set current privilege mode.
    void setPrivilegeMode(PrivilegeMode m)
    { privMode_ = m; }

    /// Helper to reset: reset floating point related structures.
    /// No-op if no  floating point extension is enabled.
    void resetFloat();

    /// Helper to reset.
    void resetVector();

    // Return true if FS field of mstatus is not off.
    bool isFpEnabled() const
    {
      unsigned fpOff = unsigned(FpStatus::Off);
      if (virtMode_)
	return mstatus_.bits_.FS != fpOff and vsstatus_.bits_.FS != fpOff;
      return mstatus_.bits_.FS != fpOff;
    }

    // Return true if it is legal to execute a zfh instruction: f and zfh
    // extensions must be enabled and FS field of MSTATUS must not be
    // OFF.
    bool isZfhLegal() const
    { return isRvf() and isRvzfh() and isFpEnabled(); }

    // Return true if it is legal to execute a zfhmin instruction: f and zfhmin
    // extensions must be enabled and FS field of MSTATUS must not be
    // OFF.
    bool isZfhminLegal() const
    { return isRvf() and (isRvzfhmin() or isRvzfh()) and isFpEnabled(); }

    // Return true if it is legal to execute a zvfh instruction: f, v,
    // and zvfh extensions must be enabled and FS field of MSTATUS must
    // not be OFF.
    bool isZvfhLegal() const
    { return isRvf() and isRvv() and isRvzvfh() and isFpEnabled(); }

    // Return true if it is legal to execute a zfhmin instruction: f,
    // v, and zvfhmin extensions must be enabled and FS field of
    // MSTATUS must not be OFF.
    bool isZvfhminLegal() const
    { return isRvf() and isRvv() and (isRvzvfhmin() or isRvzvfh()) and isFpEnabled(); }

    // Return true if it is legal to execute a zfhmin instruction: f and zfhmin
    // extensions must be enabled and FS field of MSTATUS must not be
    // OFF.
    bool isZfbfminLegal() const
    { return isRvf() and isRvzfbfmin() and isFpEnabled(); }

    // Return true if it is legal to execute a zvfbfmin instruction: f,
    // v, and zvfbfmin extensions must be enabled and FS field of
    // MSTATUS must not be OFF.
    bool isZvfbfminLegal() const
    { return isRvf() and isRvv() and isRvzvfbfmin() and isFpEnabled(); }

    // Return true if it is legal to execute a zvfbfwma instruction: f,
    // v, and zvfbfwma extensions must be enabled and FS field of
    // MSTATUS must not be OFF.
    bool isZvfbfwmaLegal() const
    { return isRvf() and isRvv() and isRvzvfbfwma() and isFpEnabled(); }

    // Return true if it is legal to execute an FP instruction: F extension must
    // be enabled and FS field of MSTATUS must not be OFF.
    bool isFpLegal() const
    { return isRvf() and isFpEnabled(); }

    // Return true if it is legal to execute a double precision
    // floating point instruction: D extension must be enabled and FS
    // field of MSTATUS must not be OFF.
    bool isDpLegal() const
    { return isRvd() and isFpEnabled(); }

    // Set the FS field of mstatus to the given value.
    void setFpStatus(FpStatus value);

    // Mark FS field of mstatus as dirty.
    void markFsDirty();

    // Return true if VS field of mstatus is not off.
    bool isVecEnabled() const
    {
      unsigned vecOff = unsigned(VecStatus::Off);
      if (virtMode_)
	return mstatus_.bits_.VS != vecOff and vsstatus_.bits_.VS != vecOff;
      return mstatus_.bits_.VS != vecOff;
    }

    // Set the VS field of MSTATUS to the given value.
    void setVecStatus(VecStatus value);

    // Mark VS field of MSTATUS as dirty.
    void markVsDirty()
    {
#ifndef FAST_SLOPPY
      setVecStatus(VecStatus::Dirty);
#endif
    }

    // Enable/disable virtual (V) mode.
    void setVirtualMode(bool mode)
    {
      virtMode_ = mode;
      csRegs_.setVirtualMode(mode);
      if (mode)
	updateCachedVsstatus();
      updateAddressTranslation();
    }

    // Return true if it is legal to execute a vector instruction: V
    // extension must be enabled and VS field of MSTATUS must not be
    // OFF.
    bool isVecLegal() const
    { return isRvv() and isVecEnabled(); }

    // Similar to isVecLegal but also saves copy of vstart ahead of
    // execution to use for logging/tracing after execution.
    bool preVecExec()
    {
      vecRegs_.setLastVstart(csRegs_.peekVstart());
      return isVecLegal();
    }

    // We avoid the cost of locating MSTATUS in the CSRs register file
    // by caching its value in this class. We do this whenever MSTATUS
    // is written/poked.
    void updateCachedMstatus();

    // We avoid the cost of locating VSSTATUS in the CSRs register file
    // by caching its value in this class. We do this whenever VSSTATUS
    // is written/poked.
    void updateCachedVsstatus();

    /// Update cached MSTATUS if non-virtual and VSSTATUS if virtual.
    void updateCachedSstatus()
    {
      if (virtMode_)
	updateCachedVsstatus();
      else
	updateCachedMstatus();
    }

    // We avoid the cost of locating HSTATUS in the CSRs register file
    // by caching its value in this class. We do this whenever HSTATUS
    // is written/poked.
    void updateCachedHstatus();

    /// Write the cached value of MSTATUS (or MSTATUS/MSTATUSH) into the CSR.
    void writeMstatus();

    /// Update big endian mode.
    void updateBigEndian();

    /// Helper to reset: Return count of implemented PMP registers.
    /// If one PMP register is implemented, make sure they are all
    /// implemented.
    unsigned countImplementedPmpRegisters() const;

    /// Helper to reset: Enable/disable extensions based on the bits
    /// of the MISA CSR.
    void processExtensions(bool verbose = true);

    /// Simulate a periodic external timer interrupt: Count-down the
    /// periodic counter. Return true if counter reaches zero (and
    /// keep returning true thereafter until timer interrupt is taken).
    /// If counter reaches zero, it is reset to its initial value.
    bool doAlarmCountdown();

    /// Return the 8-bit content of the pmpconfig register
    /// corresponding to the given pmp entry (0 to 15). Return 0 if
    /// entry is out of bounds or if the corresponding pmpconfig
    /// register is not defined.
    unsigned getPmpConfig(unsigned pmpIx);

    /// Update the physical memory protection manager. This is called
    /// on reset or whenever a pmp address/config register is updated.
    void updateMemoryProtection();

    /// Update the address translation manager. This is called on
    /// reset or whenever the supervisor address translation register
    /// (SATP) is updated.
    void updateAddressTranslation();

    /// Helper to run method: Run until toHost is written or until
    /// exit is called.
    bool simpleRun();

    /// Helper to simpleRun method when an instruction count limit is
    /// present.
    bool simpleRunWithLimit();

    /// Helper to simpleRun method when no instruction count limit is
    /// present.
    bool simpleRunNoLimit();

    /// Helper to decode. Used for compressed instructions.
    const InstEntry& decode16(uint16_t inst, uint32_t& op0, uint32_t& op1,
			      uint32_t& op2);

    /// Return the effective rounding mode for the currently executing
    /// floating point instruction.
    RoundingMode effectiveRoundingMode(unsigned instMode);

    /// Update the accrued floating point bits in the FCSR
    /// register. No-op if a trigger has tripped.
    void updateAccruedFpBits();

    /// Set the flags field in FCSR to the least sig 5 bits of the
    /// given value
    void setFpFlags(unsigned value)
    {
      uint32_t mask = uint32_t(FpFlags::FcsrMask);
      fcsrValue_ = (fcsrValue_ & ~mask) | (value & mask);
    }

    /// Set the rounding-mode field in FCSR to the least sig 3 bits of
    /// the given value
    void setFpRoundingMode(unsigned value)
    {
      uint32_t mask = uint32_t(RoundingMode::FcsrMask);
      uint32_t shift = uint32_t(RoundingMode::FcsrShift);
      fcsrValue_ = (fcsrValue_ & ~mask) | ((value << shift) & mask);
    }

    /// Return the rounding mode in FCSR.
    RoundingMode getFpRoundingMode() const
    {
      uint32_t mask = uint32_t(RoundingMode::FcsrMask);
      uint32_t shift = uint32_t(RoundingMode::FcsrShift);
      return RoundingMode((fcsrValue_ & mask) >> shift);
    }

    /// Return the flags in FCSR.
    uint32_t getFpFlags() const
    {
      uint32_t mask = uint32_t(FpFlags::FcsrMask);
      return fcsrValue_ & mask;
    }

    /// Intended to be called from within the checkRoundingMode<size>
    /// functions; if the instruction rounding mode is not valid,
    /// the take an illegal-instruction exception returning false;
    /// otherwise, return true.
    bool checkRoundingModeCommon(const DecodedInst* di);

    /// Preamble to single precision instruction execution: If F
    /// extension is not enabled or if the instruction rounding mode
    /// is not valid returning, the take an illegal-instruction
    /// exception returning false; otherwise, return true.
    bool checkRoundingModeSp(const DecodedInst* di);

    /// Similar to checkRoundingModeSp but for for half-precision (zfh
    /// extension) instructions.
    bool checkRoundingModeHp(const DecodedInst* di);

    /// Similar to checkRoundingModeSp but for for double-precision (D
    /// extension) instructions.
    bool checkRoundingModeDp(const DecodedInst* di);

    /// Similar to checkRoundingModeSp but for for bfloat16 (zfbfmin
    /// extension) instructions.
    bool checkRoundingModeBf16(const DecodedInst* di);

    /// Record the destination register and corresponding value (prior
    /// to execution) for a div/rem instruction. This is so we
    /// can undo such instruction on behalf of the test-bench.
    void recordDivInst(unsigned rd, URV value);

    /// Undo the effect of the last executed instruction given that
    /// that a trigger has tripped.
    void undoForTrigger();

    /// Return true if the mie bit of the mstatus register is on.
    bool isInterruptEnabled() const
    { return csRegs_.isInterruptEnabled(); }

    /// Based on current trigger configurations, either take an
    /// exception returning false or enter debug mode returning true.
    bool takeTriggerAction(FILE* traceFile, URV epc, URV info,
			   uint64_t& counter, bool beforeTiming);

    /// Helper to load methods: Initiate an exception with the given
    /// cause and data address.
    void initiateLoadException(const DecodedInst* di, ExceptionCause cause, URV addr1, URV addr2 = 0);

    /// Helper to store methods: Initiate an exception with the given
    /// cause and data address.
    void initiateStoreException(const DecodedInst* di, ExceptionCause cause, URV addr1, URV addr2 = 0);

    /// Helper to lb, lh, lw and ld. Load type should be int_8, int16_t
    /// etc... for signed byte, halfword etc... and uint8_t, uint16_t
    /// etc... for lbu, lhu, etc...
    /// Return true if the load is successful. Return false if an exception
    /// or a trigger is encountered. On success, loaded value (sign extended for
    /// signed type) is placed in value. Updating the destination register is
    /// the responsibility of the caller. The hyper flag should be set to true
    /// for hypervisor load/store instruction to select 2-stage address
    /// translation.
    template<typename LOAD_TYPE>
    bool load(const DecodedInst* di, uint64_t virtAddr, bool hyper, uint64_t& value);

    /// For use by performance model.
    template<typename LOAD_TYPE>
    bool fastLoad(const DecodedInst* di, uint64_t virtAddr, uint64_t& value);

    /// Helper to load method: Return possible load exception (without
    /// taking any exception). If supervisor mode is enabled, and
    /// address translation is successful, then addr1 is changed to
    /// the translated physical address and addr2 to the physical
    /// address of the subsequent page in the case of page-crossing
    /// access. If there is an exception, the addr1 is set to the
    /// virtual address causing the trap. If no address translation or
    /// no page crossing, then addr2 will be equal to addr1. The hyper
    /// flags must be true if this is called on behalf of the hypervisor
    /// load/store instructions (e.g. hlv.b).
    ExceptionCause determineLoadException(uint64_t& addr1, uint64_t& addr2,
                                          uint64_t& gaddr1, uint64_t& gaddr2,
					  unsigned ldSize, bool hyper);

    /// Helper to load method. Vaddr is the virtual address. Paddr1 is the physical
    /// address.  Paddr2 is identical to paddr1 for non-page-crossing loads; otherwise, it
    /// is the physical address on the other page.
    template<typename LOAD_TYPE>
    bool readForLoad(const DecodedInst* di, uint64_t vaddr, uint64_t paddr1,
		     uint64_t paddr2, uint64_t& data);

    /// Helper to the cache block operation (cbo) instructions.
    ExceptionCause determineCboException(uint64_t addr, uint64_t& gpa, uint64_t& pa,
					 bool isZero);

    /// Implement part of TIF protocol for writing the "tohost" magical
    /// location.
    template<typename STORE_TYPE>
    void handleStoreToHost(URV physAddr, STORE_TYPE value);

    /// Helper to sb, sh, sw ... Sore type should be uint8_t, uint16_t
    /// etc... for sb, sh, etc...
    /// Return true if store is successful. Return false if an
    /// exception or a trigger is encountered. The hyper flag should
    /// be set to true for hypervisor load/store instruction to select
    /// 2-stage address translation.
    template<typename STORE_TYPE>
    bool store(const DecodedInst* di, URV addr, bool hyper, STORE_TYPE value, bool amoLock = true);

    /// For use by performance model.
    template<typename STORE_TYPE>
    bool fastStore(const DecodedInst* di, URV addr, STORE_TYPE value);

    /// Helper to store method: Return possible exception (without
    /// taking any exception). Update stored value by doing memory
    /// mapped register masking.
    ExceptionCause determineStoreException(uint64_t& addr1, uint64_t& addr2,
                                           uint64_t& gaddr1, uint64_t& gaddr2,
					   unsigned stSize, bool hyper);

    /// Helper to store method. Vaddr is the virtual address. Paddr1 is the physical
    /// address.  Paddr2 is identical to paddr1 for non-page-crossing sores; otherwise, it
    /// is the physical address on the other page.
    template<typename STORE_TYPE>
    bool writeForStore(uint64_t vaddr, uint64_t paddr1, uint64_t paddr2, STORE_TYPE data);

    /// Helper to execLr. Load type must be int32_t, or int64_t.
    /// Return true if instruction is successful. Return false if an
    /// exception occurs or a trigger is tripped. If successful,
    /// physAddr is set to the result of the virtual to physical
    /// translation of the referenced memory address.
    template<typename LOAD_TYPE>
    bool loadReserve(const DecodedInst* di, uint32_t rd, uint32_t rs1);

    /// Helper to execSc. Store type must be uint32_t, or uint64_t.
    /// Return true if store is successful. Return false otherwise
    /// (exception or trigger or condition failed).
    template<typename STORE_TYPE>
    bool storeConditional(const DecodedInst* di, URV addr, STORE_TYPE value);

    /// Helper to the hypervisor load instructions.
    template<typename LOAD_TYPE>
    void hyperLoad(const DecodedInst* di);

    /// Helper to the hypervisor store instructions.
    template<typename LOAD_TYPE>
    void hyperStore(const DecodedInst* di);

    /// Helper for IMSIC csr accesses. Return false if access would
    /// raise virtual or illegal instruction exception and
    /// false otherwise.
    bool imsicAccessible(const DecodedInst* di, CsrNumber csr, PrivilegeMode mode, bool virtMode);

    /// Helper to CSR instructions: return true if given CSR is writebale in the given
    /// privielge level and virtual (V) mode and false otherwise.
    bool isCsrWriteable(CsrNumber csr, PrivilegeMode mode, bool virtMode) const;

    /// Helper to CSR instructions: Write csr and integer register if csr is writeable.
    void doCsrWrite(const DecodedInst* di, CsrNumber csr, URV csrVal,
                    unsigned intReg, URV intRegVal);

    /// Helper to CSR instructions: Read CSR register returning true on success and false
    /// on failure (CSR does not exist or is not accessible). The isWrite flags should be
    /// set to true if doCsrRead is called from a CSR instruction that would write the CSR
    /// register when the read is successful.
    bool doCsrRead(const DecodedInst* di, CsrNumber csr, bool isWrite, URV& csrVal);

    /// Helper to doCsrWrite/doCsrRead.
    bool checkCsrAccess(const DecodedInst* di, CsrNumber csr, bool isWrite);

    /// This is called after a csr is written/poked to update the
    /// procerssor state as a side effect to the csr change.
    void postCsrUpdate(CsrNumber csr, URV val, URV lastVal);

    /// Return true if one or more load-address/store-address trigger has a hit on the
    /// given address and given timing (before/after). Set the hit bit of all the triggers
    /// that trip.
    bool ldStAddrTriggerHit(URV addr, unsigned size, TriggerTiming t, bool isLoad)
    {
      return csRegs_.ldStAddrTriggerHit(addr, size, t, isLoad, privilegeMode(), virtMode(),
					isInterruptEnabled());
    }

    /// Return true if one or more load-address/store-address trigger has a hit on the
    /// given data value and given timing (before/after). Set the hit bit of all the
    /// triggers that trip.
    bool ldStDataTriggerHit(URV value, TriggerTiming t, bool isLoad)
    {
      return csRegs_.ldStDataTriggerHit(value, t, isLoad, privilegeMode(), virtMode(),
					isInterruptEnabled());
    }

    /// Return true if one or more execution trigger has a hit on the given address and
    /// given timing (before/after). Set the hit bit of all the triggers that trip.
    bool instAddrTriggerHit(URV addr, unsigned size, TriggerTiming t)
    {
      return csRegs_.instAddrTriggerHit(addr, size, t, privilegeMode(), virtMode(),
					isInterruptEnabled());
    }

    /// Return true if one or more execution trigger has a hit on the given opcode value
    /// and given timing (before/after). Set the hit bit of all the triggers that trip.
    bool instOpcodeTriggerHit(URV opcode, TriggerTiming t)
    {
      return csRegs_.instOpcodeTriggerHit(opcode, t, privilegeMode(), virtMode(),
					  isInterruptEnabled());
    }

    /// Make all active icount triggers count down, return true if any of them counts down
    /// to zero.
    bool icountTriggerHit()
    {
      return csRegs_.icountTriggerHit(lastPrivMode(), lastVirtMode(), privilegeMode(),
				      virtMode(), isInterruptEnabled());
    }

    /// Return true if this hart has one or more active debug triggers.
    bool hasActiveTrigger() const
    { return (enableTriggers_ and csRegs_.hasActiveTrigger()); }

    /// Return true if this hart has one or more active debug instruction (execute)
    /// triggers.
    bool hasActiveInstTrigger() const
    { return (enableTriggers_ and csRegs_.hasActiveInstTrigger()); }

    /// Collect instruction stats (for instruction profile and/or
    /// performance monitors).
    void accumulateInstructionStats(const DecodedInst&);

    /// Collect exception/interrupt stats.
    void accumulateTrapStats(bool isNmi);

    /// Update performance counters: Enabled counters tick up
    /// according to the events associated with the most recent
    /// retired instruction.
    void updatePerformanceCounters(const DecodedInst& di);

    // For CSR instruction we need to let the counters count before
    // letting CSR instruction write. Consequently we update the
    // counters from within the code executing the CSR instruction
    // using this method.
    void updatePerformanceCountersForCsr(const DecodedInst& di);

    /// Fetch an instruction from the given virtual address. Return
    /// true on success. Return false on fail (in which case an
    /// exception is initiated). If successful set pysAddr to the
    /// physical address corresponding to the given virtual address.
    bool fetchInst(URV virAddr, uint64_t& physAddr, uint32_t& instr);

    /// Helper to the run methods: Fetch an instruction taking debug triggers
    /// into consideration. Return true if successful. Return false if
    /// instruction fetch fails (an exception is signaled in that case).
    bool fetchInstWithTrigger(URV addr, uint64_t& physAddr, uint32_t& inst,
			      FILE* trace);

    /// Helper to fetchInstWithTrigger. Fetch an instruction given
    /// that a trigger has tripped. Return true on success. Return
    /// false on a a fail in which case either a trigger exception is
    /// initiated (as opposed to an instruction-fail exception).
    bool fetchInstPostTrigger(URV virtAddr, uint64_t& physAddr, uint32_t& inst,
			      FILE* trace);

    /// Write trace information about the given instruction to the
    /// given file. This is assumed to be called after instruction
    /// execution. Tag is the record tag (the retired instruction
    /// count after instruction is executed). Tmp is a temporary
    /// string (for performance).
    void printDecodedInstTrace(const DecodedInst& di, uint64_t tag, std::string& tmp,
                               FILE* out);

    /// Variant of the preceding method for cases where the trace is
    /// printed before decode. If the instruction is not available
    /// then a zero (illegal) value is required.
    void printInstTrace(uint32_t instruction, uint64_t tag, std::string& tmp,
			FILE* out);

    void printInstCsvTrace(const DecodedInst& di, FILE* out);

    /// Start a synchronous exceptions.
    void initiateException(ExceptionCause cause, URV pc, URV info, URV info2 = 0,
			   const DecodedInst* di = nullptr);

    /// Start an asynchronous exception (interrupt).
    void initiateInterrupt(InterruptCause cause, URV pc);

    /// Start a non-maskable interrupt. Return true if successful. Return false
    /// if Smrnmi and nmis are disabled.
    bool initiateNmi(URV cause, URV pc);

    /// interrupts without considering the delegation registers.
    void undelegatedInterrupt(URV cause, URV pcToSave, URV nextPc);

    /// If a non-maskable-interrupt is pending take it. If an external
    /// interrupt is pending and interrupts are enabled, then take
    /// it. Return true if an nmi or an interrupt is taken and false
    /// otherwise.
    bool processExternalInterrupt(FILE* traceFile, std::string& insStr);

    /// Helper to FP execution: Or the given flags values to FCSR
    /// recording a write. No-op if a trigger has already tripped.
    void orFcsrFlags(FpFlags value);

    /// Execute decoded instruction. Branch/jump instructions will
    /// modify pc_.
    void execute(const DecodedInst* di);

    /// Helper to disassembleInst32: Disassemble instructions
    /// associated with opcode 1010011.
    void disassembleFp(uint32_t inst, std::ostream& stream);

    /// Change machine state and program counter in reaction to an
    /// exception or an interrupt. Given pc is the program counter to
    /// save (address of instruction causing the asynchronous
    /// exception or the instruction to resume after asynchronous
    /// exception is handled). The info and info2 value holds additional
    /// information about an exception.
    void initiateTrap(const DecodedInst* di, bool interrupt, URV cause, URV pcToSave, URV info,
                      URV info2 = 0);

    /// Create trap instruction information for mtinst/htinst.
    uint32_t createTrapInst(const DecodedInst* di, bool interrupt, unsigned cause, URV info, URV info2) const;

    /// Illegal instruction. Initiate an illegal instruction trap.
    /// This is used for one of the following:
    ///   - Invalid opcode.
    ///   - Extension instruction executed when extension is off in mstatus or misa.
    ///   - Machine mode instruction executed when not in machine mode.
    ///   - Invalid CSR.
    ///   - Write to a read-only CSR.
    void illegalInst(const DecodedInst*);

    /// Initiate a virtual instruction trap.
    void virtualInst(const DecodedInst*);

    /// Place holder for not-yet implemented instructions. Calls
    /// illegal instruction.
    void unimplemented(const DecodedInst*);

    /// Check address associated with an atomic memory operation (AMO) instruction. Return
    /// true if AMO access is allowed. Return false triggering an exception if address is
    /// misaligned.  If successful, the given virtual addr is replaced by the translated
    /// physical address.
    ExceptionCause validateAmoAddr(uint64_t& addr, uint64_t& gaddr, unsigned accessSize);

    /// Do the load value part of a word-sized AMO instruction. Return
    /// true on success putting the loaded value in val. Return false
    /// if a trigger tripped or an exception took place in which case
    /// val is not modified. The loaded word is sign extended to fill
    /// the URV value (this is relevant for rv64). The accessed memory
    /// must have the given attribute (e.g. Pma::Arith, Pma::Swap ...)
    /// or access fault.
    bool amoLoad32(const DecodedInst* di, uint64_t vaddr, Pma::Attrib attrib, URV& val);

    /// Do the load value part of a double-word-sized AMO
    /// instruction. Return true on success putting the loaded value
    /// in val. Return false if a trigger tripped or an exception took
    /// place in which case val is not modified. The accessed memory
    /// must have the given attribute (e.g. Pma::Arith, Pma::Swap ...)
    /// or access fault.
    bool amoLoad64(const DecodedInst* di, uint64_t vaddr, Pma::Attrib attrib, URV& val);

    /// Invalidate cache entries overlapping the bytes written by a
    /// store.
    void invalidateDecodeCache(URV addr, unsigned storeSize);

    /// Helper to shift/bit execute instruction with immediate
    /// operands: Signal an illegal instruction if immediate value is
    /// greater than XLEN-1 returning false; otherwise return true.
    bool checkShiftImmediate(const DecodedInst* di, URV imm);

    /// Report the number of retired instruction count and the simulation
    /// rate.
    void reportInstsPerSec(uint64_t instCount, double elapsed, bool userStop);

    /// Helper to the run methods: Log (on the standard error) the
    /// cause of a stop signaled with an exception. Return true if
    /// program finished successfully, return false otherwise.  If
    /// traceFile is non-null, then trace the instruction that caused
    /// the stop.
    bool logStop(const CoreException& ce, uint64_t instCount, FILE* traceFile);

    /// Return true if mcycle is enabled (not inhibited by mcountinhibit).
    bool mcycleEnabled() const
    { return prevPerfControl_ & 1; }

    /// Return true if minstret is enabled (not inhibited by mcountinhibit).
    bool minstretEnabled() const
    { return prevPerfControl_ & 0x4; }

    /// Called to check if a CLINT memory mapped register is written.
    /// Clear/set software-interrupt bit in the MIP CSR of
    /// corresponding hart if all the conditions are met. Set timer
    /// limit if timer-limit register is written. Update stVal: if location
    /// is outside the range of valid harts, set stVal to zero.  If it is
    /// in the software interrupt range then keep it least sig bit and zero
    /// the rest.
    void processClintWrite(uint64_t addr, unsigned stSize, URV& stVal);

    /// Called if interruptor address is written. Unpack written value
    /// and set MIP bit in target hart.
    void processInterruptorWrite(uint32_t stVal);

    /// Mask to extract shift amount from a integer register value to use
    /// in shift instructions. This returns 0x1f in 32-bit more and 0x3f
    /// in 64-bit mode.
    uint32_t shiftMask() const
    {
      if constexpr (std::is_same<URV, uint32_t>::value)
	return 0x1f;
      if constexpr (std::is_same<URV, uint64_t>::value)
	return 0x3f;
      assert(0 and "Register value type must be uint32_t or uint64_t.");
      return 0x1f;
    }

    /// Return true if maskable integer vector instruction is legal for
    /// current sew and lmul, current vstart, and mask-register
    /// destination register overlap. Check if vector extension is
    /// enabled. Take an illegal instruction exception and return false
    /// otherwise. This is for non-load, non-mask destination,
    /// non-reduction instructions.
    bool checkVecIntInst(const DecodedInst* di);

    /// Same as above but with explicit group multiplier and element width.
    bool checkVecIntInst(const DecodedInst* di, GroupMultiplier gm, ElementWidth eew);

    /// Same as above but for vector load/store. Ignores vstart.
    bool checkVecLdStInst(const DecodedInst* di);

    /// Return true if given arithmetic (non load/store) instruction is
    /// legal. Check if vector extension is enabled. Check if the
    /// current sew/lmul is legal. Return true if legal. Take an
    /// illegal instruction exception and return false otherwise. This
    /// is for integer instructions.
    bool checkSewLmulVstart(const DecodedInst* di);

    /// Similar to above but includes check for floating point operations (is F/D/ZFH
    /// enabled ...).
    bool checkFpSewLmulVstart(const DecodedInst* di, bool wide = false,
			      bool (Hart::*fp16LegalFn)() const = &Hart::isZvfhLegal);

    /// Return true if maskable floating point vector instruction is
    /// legal. Take an illegal instruction exception and return false
    /// otherwise.
    bool checkVecFpInst(const DecodedInst* di, bool wide = false,
			bool (Hart::*fp16LegalFn)() const = &Hart::isZvfhLegal);

    /// Return true if vector operands are multiples of the given group
    /// multiplier (scaled by 8). Initiating an illegal instruction
    /// trap and return false otherwise.
    bool checkVecOpsVsEmul(const DecodedInst* di, unsigned op0, unsigned op1,
			   unsigned op2, unsigned groupX8);

    /// Similar to above but for vector instructions with 2 vector operands.
    bool checkVecOpsVsEmul(const DecodedInst* di, unsigned op0, unsigned op1,
			   unsigned groupX8);

    /// Similar to above but a vector instructions with 1 vector
    /// operand. This also work for checking one vector operand of a
    /// multi-operand instruction.
    bool checkVecOpsVsEmul(const DecodedInst* di, unsigned op, unsigned groupX8);

    /// Return if mask producing instruction (e.g. vmseq) is
    /// legal. Check if vector operands are multiples of the given
    /// group multiplier (scaled by 8). Return true if legal.  Return
    /// false initiating an illegal instruction trap otherwise.
    bool checkVecMaskInst(const DecodedInst* di, unsigned op0, unsigned op1,
			  unsigned groupX8);

    /// Similar to the above but for 3 vector operand instructions.
    bool checkVecMaskInst(const DecodedInst* di, unsigned op0, unsigned op1,
			  unsigned op2, unsigned groupX8);


    /// Similar to the above but for floating point instructions.
    bool checkVecFpMaskInst(const DecodedInst* di, unsigned op0, unsigned op1,
				 unsigned groupX8);

    /// Similar to the above but for floating point instructions with 3
    /// vector operands.
    bool checkVecFpMaskInst(const DecodedInst* di, unsigned op0, unsigned op1,
				 unsigned op2, unsigned groupX8);

    /// Similar to the above but for vector load/store indexed.
    /// Checks overlap for vd/vs3 and index registers.
    bool checkVecLdStIndexedInst(const DecodedInst* di, unsigned vd, unsigned vi,
                                  unsigned offsetWidth, unsigned offsetGroupX8,
                                  unsigned fieldCount);

    /// Check reduction vector operand against the group multiplier. Return true
    /// if operand is a multiple of multiplier and false otherwise. Record group
    /// multiplier for tracing.
    bool checkRedOpVsEmul(const DecodedInst* di, unsigned op1,
			  unsigned groupX8, unsigned vstart);

    /// Similar to above but 3 vector operands and 1st operand is wide.
    bool checkVecOpsVsEmulW0(const DecodedInst* di, unsigned op0, unsigned op1,
			     unsigned op2, unsigned groupX8);

    /// Similar to above but 2 vector operands and 1st operand is wide.
    bool checkVecOpsVsEmulW0(const DecodedInst* di, unsigned op0, unsigned op1,
			     unsigned groupX8);

    /// Similar to above but ternary and 1st operand is wide.
    bool checkVecTernaryOpsVsEmulW0(const DecodedInst* di, unsigned op0, unsigned op1,
			            unsigned op2, unsigned groupX8);

    /// Similar to above but 3 vector operands and 1st 2 operands are wide.
    bool checkVecOpsVsEmulW0W1(const DecodedInst* di, unsigned op0, unsigned op1,
			       unsigned op2, unsigned groupX8);

    /// Similar to above but 2 vector operands and 1st 2 operands are wide.
    bool checkVecOpsVsEmulW0W1(const DecodedInst* di, unsigned op0, unsigned op1,
			       unsigned groupX8);

    /// Similar to above but 3 vector operands with 2nd operand wide.
    bool checkVecOpsVsEmulW1(const DecodedInst* di, unsigned op0, unsigned op1,
			     unsigned op2, unsigned groupX8);

    /// Similar to above but 2 vector operands with 2nd operand wide.
    bool checkVecOpsVsEmulW1(const DecodedInst* di, unsigned op0, unsigned op1,
			     unsigned groupX8);

    /// Performs an in-place group-wise reduction on a series of vector registers.
    /// Does nothing if LMUL <= 1.
    template <typename ELEM_TYPE>
    void doVecFpRedSumGroup(std::vector<ELEM_TYPE>& elems, ElementWidth eew, unsigned groupX8);

    /// Performs an in-place tree sum reduction on adjacent vector elements (starting
    /// at index 0) until numResult is remaining.
    template <typename ELEM_TYPE>
    void doVecFpRedSumAdjacent(std::vector<ELEM_TYPE>& elems, unsigned numElems,
                              unsigned numResult);

    /// Performs an in-place tree sum reduction on every other vector element (starting
    /// at index 0) until numResult is remaining.
    template <typename ELEM_TYPE>
    void doVecFpRedSumStride(std::vector<ELEM_TYPE>& elems, unsigned numElems,
                              unsigned numResult);

    /// Emit a trace record for the given branch instruction or trap in the
    /// branch trace file.
    void traceBranch(const DecodedInst* di);

    /// Called at the end of successful vector instruction to clear the
    /// vstart register and mark VS dirty if a vector register was
    /// updated.
    void postVecSuccess();

    /// Called at the end of a trapping vector instruction to mark VS
    /// dirty if a vector register was updated.
    void postVecFail(const DecodedInst* di);

    /// The program counter is adjusted (size of current instruction
    /// added) before any of the following exec methods are called. To
    /// get the address before adjustment, use currPc_.

    void execBeq(const DecodedInst*);
    void execBne(const DecodedInst*);
    void execBlt(const DecodedInst*);
    void execBltu(const DecodedInst*);
    void execBge(const DecodedInst*);
    void execBgeu(const DecodedInst*);
    void execJalr(const DecodedInst*);
    void execJal(const DecodedInst*);
    void execLui(const DecodedInst*);
    void execAuipc(const DecodedInst*);
    void execAddi(const DecodedInst*);
    void execSlli(const DecodedInst*);
    void execSlti(const DecodedInst*);
    void execSltiu(const DecodedInst*);
    void execXori(const DecodedInst*);
    void execSrli(const DecodedInst*);
    void execSrai(const DecodedInst*);
    void execOri(const DecodedInst*);
    void execAndi(const DecodedInst*);
    void execAdd(const DecodedInst*);
    void execSub(const DecodedInst*);
    void execSll(const DecodedInst*);
    void execSlt(const DecodedInst*);
    void execSltu(const DecodedInst*);
    void execXor(const DecodedInst*);
    void execSrl(const DecodedInst*);
    void execSra(const DecodedInst*);
    void execOr(const DecodedInst*);
    void execAnd(const DecodedInst*);

    void execFence(const DecodedInst*);
    void execFence_tso(const DecodedInst*);
    void execFencei(const DecodedInst*);

    void execEcall(const DecodedInst*);
    void execEbreak(const DecodedInst*);
    void execMret(const DecodedInst*);
    void execSret(const DecodedInst*);
    void execMnret(const DecodedInst*);
    void execWfi(const DecodedInst*);

    void execDret(const DecodedInst*);

    void execSfence_vma(const DecodedInst*);

    void execCsrrw(const DecodedInst*);
    void execCsrrs(const DecodedInst*);
    void execCsrrc(const DecodedInst*);
    void execCsrrwi(const DecodedInst*);
    void execCsrrsi(const DecodedInst*);
    void execCsrrci(const DecodedInst*);

    void execLb(const DecodedInst*);
    void execLh(const DecodedInst*);
    void execLw(const DecodedInst*);
    void execLbu(const DecodedInst*);
    void execLhu(const DecodedInst*);

    void execSb(const DecodedInst*);
    void execSh(const DecodedInst*);
    void execSw(const DecodedInst*);

    void execMul(const DecodedInst*);
    void execMulh(const DecodedInst*);
    void execMulhsu(const DecodedInst*);
    void execMulhu(const DecodedInst*);
    void execDiv(const DecodedInst*);
    void execDivu(const DecodedInst*);
    void execRem(const DecodedInst*);
    void execRemu(const DecodedInst*);

    // rv64i
    void execLwu(const DecodedInst*);
    void execLd(const DecodedInst*);
    void execSd(const DecodedInst*);
    void execSlliw(const DecodedInst*);
    void execSrliw(const DecodedInst*);
    void execSraiw(const DecodedInst*);
    void execAddiw(const DecodedInst*);
    void execAddw(const DecodedInst*);
    void execSubw(const DecodedInst*);
    void execSllw(const DecodedInst*);
    void execSrlw(const DecodedInst*);
    void execSraw(const DecodedInst*);

    // rv128i
    void execLq(const DecodedInst*);
    void execSq(const DecodedInst*);

    // rv64m
    void execMulw(const DecodedInst*);
    void execDivw(const DecodedInst*);
    void execDivuw(const DecodedInst*);
    void execRemw(const DecodedInst*);
    void execRemuw(const DecodedInst*);

    // rv32f
    void execFlw(const DecodedInst*);
    void execFsw(const DecodedInst*);
    void execFmadd_s(const DecodedInst*);
    void execFmsub_s(const DecodedInst*);
    void execFnmsub_s(const DecodedInst*);
    void execFnmadd_s(const DecodedInst*);
    void execFadd_s(const DecodedInst*);
    void execFsub_s(const DecodedInst*);
    void execFmul_s(const DecodedInst*);
    void execFdiv_s(const DecodedInst*);
    void execFsqrt_s(const DecodedInst*);
    void execFsgnj_s(const DecodedInst*);
    void execFsgnjn_s(const DecodedInst*);
    void execFsgnjx_s(const DecodedInst*);
    void execFmin_s(const DecodedInst*);
    void execFmax_s(const DecodedInst*);
    void execFcvt_w_s(const DecodedInst*);
    void execFcvt_wu_s(const DecodedInst*);
    void execFmv_x_w(const DecodedInst*);
    void execFeq_s(const DecodedInst*);
    void execFlt_s(const DecodedInst*);
    void execFle_s(const DecodedInst*);
    void execFclass_s(const DecodedInst*);
    void execFcvt_s_w(const DecodedInst*);
    void execFcvt_s_wu(const DecodedInst*);
    void execFmv_w_x(const DecodedInst*);

    // rv32f + rv64
    void execFcvt_l_s(const DecodedInst*);
    void execFcvt_lu_s(const DecodedInst*);
    void execFcvt_s_l(const DecodedInst*);
    void execFcvt_s_lu(const DecodedInst*);

    // rv32d
    void execFld(const DecodedInst*);
    void execFsd(const DecodedInst*);
    void execFmadd_d(const DecodedInst*);
    void execFmsub_d(const DecodedInst*);
    void execFnmsub_d(const DecodedInst*);
    void execFnmadd_d(const DecodedInst*);
    void execFadd_d(const DecodedInst*);
    void execFsub_d(const DecodedInst*);
    void execFmul_d(const DecodedInst*);
    void execFdiv_d(const DecodedInst*);
    void execFsgnj_d(const DecodedInst*);
    void execFsgnjn_d(const DecodedInst*);
    void execFsgnjx_d(const DecodedInst*);
    void execFmin_d(const DecodedInst*);
    void execFmax_d(const DecodedInst*);
    void execFcvt_d_s(const DecodedInst*);
    void execFcvt_s_d(const DecodedInst*);
    void execFsqrt_d(const DecodedInst*);
    void execFle_d(const DecodedInst*);
    void execFlt_d(const DecodedInst*);
    void execFeq_d(const DecodedInst*);
    void execFcvt_w_d(const DecodedInst*);
    void execFcvt_wu_d(const DecodedInst*);
    void execFcvt_d_w(const DecodedInst*);
    void execFcvt_d_wu(const DecodedInst*);
    void execFclass_d(const DecodedInst*);

    // rv32d + rv64
    void execFcvt_l_d(const DecodedInst*);
    void execFcvt_lu_d(const DecodedInst*);
    void execFcvt_d_l(const DecodedInst*);
    void execFcvt_d_lu(const DecodedInst*);
    void execFmv_d_x(const DecodedInst*);
    void execFmv_x_d(const DecodedInst*);

    // zfh (half precision floating point)
    void execFlh(const DecodedInst*);
    void execFsh(const DecodedInst*);
    void execFmadd_h(const DecodedInst*);
    void execFmsub_h(const DecodedInst*);
    void execFnmsub_h(const DecodedInst*);
    void execFnmadd_h(const DecodedInst*);
    void execFadd_h(const DecodedInst*);
    void execFsub_h(const DecodedInst*);
    void execFmul_h(const DecodedInst*);
    void execFdiv_h(const DecodedInst*);
    void execFsqrt_h(const DecodedInst*);
    void execFsgnj_h(const DecodedInst*);
    void execFsgnjn_h(const DecodedInst*);
    void execFsgnjx_h(const DecodedInst*);
    void execFmin_h(const DecodedInst*);
    void execFmax_h(const DecodedInst*);
    void execFcvt_s_h(const DecodedInst*);
    void execFcvt_d_h(const DecodedInst*);
    void execFcvt_h_s(const DecodedInst*);
    void execFcvt_h_d(const DecodedInst*);
    void execFcvt_w_h(const DecodedInst*);
    void execFcvt_wu_h(const DecodedInst*);
    void execFmv_x_h(const DecodedInst*);
    void execFeq_h(const DecodedInst*);
    void execFlt_h(const DecodedInst*);
    void execFle_h(const DecodedInst*);
    void execFclass_h(const DecodedInst*);
    void execFcvt_h_w(const DecodedInst*);
    void execFcvt_h_wu(const DecodedInst*);
    void execFmv_h_x(const DecodedInst*);

    // zfh + rv64
    void execFcvt_l_h(const DecodedInst*);
    void execFcvt_lu_h(const DecodedInst*);
    void execFcvt_h_l(const DecodedInst*);
    void execFcvt_h_lu(const DecodedInst*);

    // atomic
    template <typename OP>
    void execAmo32Op(const DecodedInst*, Pma::Attrib attrib, OP op);
    void execAmoadd_w(const DecodedInst*);
    void execAmoswap_w(const DecodedInst*);
    void execLr_w(const DecodedInst*);
    void execSc_w(const DecodedInst*);
    void execAmoxor_w(const DecodedInst*);
    void execAmoor_w(const DecodedInst*);
    void execAmoand_w(const DecodedInst*);
    void execAmomin_w(const DecodedInst*);
    void execAmomax_w(const DecodedInst*);
    void execAmominu_w(const DecodedInst*);
    void execAmomaxu_w(const DecodedInst*);

    // atomic + rv64
    template <typename OP>
    void execAmo64Op(const DecodedInst*, Pma::Attrib attrib, OP op);
    void execAmoadd_d(const DecodedInst*);
    void execAmoswap_d(const DecodedInst*);
    void execLr_d(const DecodedInst*);
    void execSc_d(const DecodedInst*);
    void execAmoxor_d(const DecodedInst*);
    void execAmoor_d(const DecodedInst*);
    void execAmoand_d(const DecodedInst*);
    void execAmomin_d(const DecodedInst*);
    void execAmomax_d(const DecodedInst*);
    void execAmominu_d(const DecodedInst*);
    void execAmomaxu_d(const DecodedInst*);

    // Bit manipulation: zbb
    void execClz(const DecodedInst*);
    void execCtz(const DecodedInst*);
    void execCpop(const DecodedInst*);
    void execClzw(const DecodedInst*);
    void execCtzw(const DecodedInst*);
    void execCpopw(const DecodedInst*);
    void execMin(const DecodedInst*);
    void execMax(const DecodedInst*);
    void execMinu(const DecodedInst*);
    void execMaxu(const DecodedInst*);
    void execSext_b(const DecodedInst*);
    void execSext_h(const DecodedInst*);
    void execAndn(const DecodedInst*);
    void execOrc_b(const DecodedInst*);
    void execOrn(const DecodedInst*);
    void execXnor(const DecodedInst*);
    void execRol(const DecodedInst*);
    void execRor(const DecodedInst*);
    void execRori(const DecodedInst*);
    void execRolw(const DecodedInst*);
    void execRorw(const DecodedInst*);
    void execRoriw(const DecodedInst*);
    void execPack(const DecodedInst*);
    void execSlli_uw(const DecodedInst*);
    void execPackh(const DecodedInst*);   // Zbkb
    void execPackw(const DecodedInst*);   // rename zext_h, in Zbb
    void execBrev8(const DecodedInst*);   // In Zbb
    void execBrev8_32(const DecodedInst*);
    void execBrev8_64(const DecodedInst*);
    void execRev8_32(const DecodedInst*);
    void execRev8_64(const DecodedInst*);
    void execUnzip(const DecodedInst*);
    void execZip(const DecodedInst*);

    void execXperm_n(const DecodedInst*); // Zbkx
    void execXperm_b(const DecodedInst*); // Zbkx

    // Bit manipulation: zbs
    void execBset(const DecodedInst*);
    void execBclr(const DecodedInst*);
    void execBinv(const DecodedInst*);
    void execBext(const DecodedInst*);
    void execBseti(const DecodedInst*);
    void execBclri(const DecodedInst*);
    void execBinvi(const DecodedInst*);
    void execBexti(const DecodedInst*);

    void execClmul(const DecodedInst*);
    void execClmulh(const DecodedInst*);
    void execClmulr(const DecodedInst*);

    void execSh1add(const DecodedInst*);
    void execSh2add(const DecodedInst*);
    void execSh3add(const DecodedInst*);
    void execSh1add_uw(const DecodedInst*);
    void execSh2add_uw(const DecodedInst*);
    void execSh3add_uw(const DecodedInst*);
    void execAdd_uw(const DecodedInst*);

    /// Code common to execVsetvli, and execVsetvl. Return true on success and false if an
    /// illegal instruction trap must be taken.
    bool vsetvl(unsigned rd, unsigned rs1, URV vtypeVal, bool isVtypeImm);

    void execVsetvli(const DecodedInst*);
    void execVsetivli(const DecodedInst*);
    void execVsetvl(const DecodedInst*);

    /// Helper to vector vv instructions (eg vadd.vv, vsub.vv). Operation
    /// to be performed (eg. add, sub) is passed in op.
    template<typename ELEM_TYPE>
    void vop_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		unsigned start, unsigned elems, bool masked,
		std::function<ELEM_TYPE(ELEM_TYPE, ELEM_TYPE)> op);

    /// Same as vop_vv, but for floating-point operations. This updates
    /// incremental fp flags for each vector element.
    template<typename ELEM_TYPE>
    void vfop_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		 unsigned start, unsigned elems, bool masked,
		 std::function<ELEM_TYPE(ELEM_TYPE, ELEM_TYPE)> fop);

    /// Helper to vector vv instructions (eg vadd.vx, vsub.vx). Operation
    /// to be performed (eg. add, sub) is passed in op.
    template<typename ELEM_TYPE>
    void vop_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
		unsigned start, unsigned elems, bool masked,
		std::function<ELEM_TYPE(ELEM_TYPE, ELEM_TYPE)> op);

    /// Helper to vector mask vv instructions (eg vmseq.vv). Operation
    /// to be performed (eg. equal_to) passed in op.
    template<typename ELEM_TYPE>
    void vmop_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		 unsigned start, unsigned elems, bool masked,
		 std::function<bool(ELEM_TYPE, ELEM_TYPE)> op);

    /// Helper to vector mask vv instructions (eg vmseq.vx). Operation
    /// to be performed (eg. equal_to) passed in op.
    template<typename ELEM_TYPE>
    void vmop_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
		 unsigned start, unsigned elems, bool masked,
		 std::function<bool(ELEM_TYPE, ELEM_TYPE)> op);

    /// Helper to vector vv integer instructions. Operation to be
    /// performed (e.g. std::plus for vadd.vv) is passed in op.
    template<typename OP>
    void execVop_vv(const DecodedInst*, OP op);

    /// Helper to vector vv instructions and unsigned integer element
    /// types. Operation to be performed (e.g. std::plus for vadd.vv)
    /// is passed in op.
    template<typename OP>
    void execVopu_vv(const DecodedInst*, OP op);

    /// Helper to vector vx integer instructions. Operation to be
    /// performed (e.g. std::plus for vadd.vv) is passed in op.
    template<typename OP>
    void execVop_vx(const DecodedInst*, OP op);

    /// Helper to vector vx instructions and unsigned integer element
    /// types. Operation to be performed (e.g. std::plus for vadd.vv)
    /// is passed in op.
    template<typename OP>
    void execVopu_vx(const DecodedInst*, OP op);

    /// Helper to vector vi integer instructions. Operation to be
    /// performed (e.g. std::plus for vadd.vv) is passed in op.
    template<typename OP>
    void execVop_vi(const DecodedInst*, OP op);

    /// Helper to vector vi instructions and unsigned integer element
    /// types. Operation to be performed (e.g. std::plus for vadd.vv)
    /// is passed in op.
    template<typename OP>
    void execVopu_vi(const DecodedInst*, OP op);

    /// Helper to vector vs integer reduction instructions (eg
    /// vredsum.vs). Operation to be performed (eg. std::plus) is
    /// passed in op.
    template<typename ELEM_TYPE>
    void vredop_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked,
		   std::function<ELEM_TYPE(ELEM_TYPE, ELEM_TYPE)> op);

    /// Helper to vector vs integer reduction instructions (eg vredsum.vs). Operation
    /// to be performed (eg. std::plus) is passed in op.
    template<typename OP>
    void execVredop_vs(const DecodedInst*, OP op);

    /// Helper to vector vs unsigned integer reduction instructions
    /// (eg vredmaxu.vs). Operation to be performed (eg. std::max) is
    /// passed in op.
    template<typename OP>
    void execVredopu_vs(const DecodedInst*, OP op);

    /// Helper to vector mm mask bitwise instructions
    /// (eg vmand.mm). Operation to be performed (eg. std::max) is
    /// passed in op.
    template <typename OP>
    void execVmop_mm(const DecodedInst* di, OP op);

    void execVadd_vv(const DecodedInst*);
    void execVadd_vx(const DecodedInst*);
    void execVadd_vi(const DecodedInst*);

    void execVsub_vv(const DecodedInst*);
    void execVsub_vx(const DecodedInst*);

    void execVrsub_vx(const DecodedInst*);
    void execVrsub_vi(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVwaddu_vv(const DecodedInst*);
    void execVwadd_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwadd_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVwaddu_vx(const DecodedInst*);
    void execVwadd_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwsub_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVwsubu_vx(const DecodedInst*);
    void execVwsub_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVwsubu_vv(const DecodedInst*);
    void execVwsub_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwadd_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVwaddu_wv(const DecodedInst*);
    void execVwadd_wv(const DecodedInst*);

    void execVwaddu_wx(const DecodedInst*);
    void execVwadd_wx(const DecodedInst*);
    void execVwsubu_wx(const DecodedInst*);
    void execVwsub_wx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwsub_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVwsubu_wv(const DecodedInst*);
    void execVwsub_wv(const DecodedInst*);

    void execVmseq_vv(const DecodedInst*);
    void execVmseq_vx(const DecodedInst*);
    void execVmseq_vi(const DecodedInst*);

    void execVmsne_vv(const DecodedInst*);
    void execVmsne_vx(const DecodedInst*);
    void execVmsne_vi(const DecodedInst*);

    void execVmsltu_vv(const DecodedInst*);
    void execVmslt_vv(const DecodedInst*);

    void execVmsltu_vx(const DecodedInst*);
    void execVmslt_vx(const DecodedInst*);

    void execVmsleu_vv(const DecodedInst*);
    void execVmsleu_vx(const DecodedInst*);
    void execVmsleu_vi(const DecodedInst*);

    void execVmsle_vv(const DecodedInst*);
    void execVmsle_vx(const DecodedInst*);
    void execVmsle_vi(const DecodedInst*);

    void execVmsgtu_vx(const DecodedInst*);
    void execVmsgtu_vi(const DecodedInst*);
    void execVmsgt_vx(const DecodedInst*);
    void execVmsgt_vi(const DecodedInst*);

    void execVminu_vv(const DecodedInst*);
    void execVminu_vx(const DecodedInst*);
    void execVmin_vv(const DecodedInst*);
    void execVmin_vx(const DecodedInst*);


    void execVmaxu_vv(const DecodedInst*);
    void execVmaxu_vx(const DecodedInst*);
    void execVmax_vv(const DecodedInst*);
    void execVmax_vx(const DecodedInst*);

    void execVand_vv(const DecodedInst*);
    void execVand_vx(const DecodedInst*);
    void execVand_vi(const DecodedInst*);

    void execVor_vv(const DecodedInst*);
    void execVor_vx(const DecodedInst*);
    void execVor_vi(const DecodedInst*);

    void execVxor_vv(const DecodedInst*);
    void execVxor_vx(const DecodedInst*);
    void execVxor_vi(const DecodedInst*);

    void execVsll_vv(const DecodedInst*);
    void execVsll_vx(const DecodedInst*);
    void execVsll_vi(const DecodedInst*);

    void execVsrl_vv(const DecodedInst*);
    void execVsrl_vx(const DecodedInst*);
    void execVsrl_vi(const DecodedInst*);

    void execVsra_vv(const DecodedInst*);
    void execVsra_vx(const DecodedInst*);
    void execVsra_vi(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vnsr_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		unsigned start, unsigned elems, bool masked);
    void execVnsrl_wv(const DecodedInst*);
    void execVnsra_wv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vnsr_wx(unsigned vd, unsigned vs1, URV e2, unsigned group,
                 unsigned start, unsigned elems, bool masked);
    void execVnsrl_wx(const DecodedInst*);
    void execVnsrl_wi(const DecodedInst*);
    void execVnsra_wx(const DecodedInst*);
    void execVnsra_wi(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vrgather_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked);
    void execVrgather_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vrgather_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                     unsigned start, unsigned elems, bool masked);
    void execVrgather_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vrgather_vi(unsigned vd, unsigned vs1, uint32_t imm, unsigned group,
                     unsigned start, unsigned elems, bool masked);
    void execVrgather_vi(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vrgatherei16_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                         unsigned start, unsigned elems, bool masked);
    void execVrgatherei16_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vcompress_vm(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                      unsigned start, unsigned elems);
    void execVcompress_vm(const DecodedInst*);

    void execVredsum_vs(const DecodedInst*);
    void execVredand_vs(const DecodedInst*);
    void execVredor_vs(const DecodedInst*);
    void execVredxor_vs(const DecodedInst*);
    void execVredminu_vs(const DecodedInst*);
    void execVredmin_vs(const DecodedInst*);
    void execVredmaxu_vs(const DecodedInst*);
    void execVredmax_vs(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwredsum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked);
    void execVwredsumu_vs(const DecodedInst*);
    void execVwredsum_vs(const DecodedInst*);

    void execVmand_mm(const DecodedInst*);
    void execVmnand_mm(const DecodedInst*);
    void execVmandn_mm(const DecodedInst*);
    void execVmxor_mm(const DecodedInst*);
    void execVmor_mm(const DecodedInst*);
    void execVmnor_mm(const DecodedInst*);
    void execVmorn_mm(const DecodedInst*);
    void execVmxnor_mm(const DecodedInst*);
    void execVcpop_m(const DecodedInst*);
    void execVfirst_m(const DecodedInst*);
    void execVmsbf_m(const DecodedInst*);
    void execVmsif_m(const DecodedInst*);
    void execVmsof_m(const DecodedInst*);
    void execViota_m(const DecodedInst*);
    void execVid_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vslideup(unsigned vd, unsigned vs1, URV amount, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVslideup_vx(const DecodedInst*);
    void execVslideup_vi(const DecodedInst*);
    void execVslide1up_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vslidedown(unsigned vd, unsigned vs1, URV amount, unsigned group,
                    unsigned start, unsigned elems, bool masked);
    void execVslidedown_vx(const DecodedInst*);
    void execVslidedown_vi(const DecodedInst*);
    void execVslide1down_vx(const DecodedInst*);

    void execVfslide1up_vf(const DecodedInst*);
    void execVfslide1down_vf(const DecodedInst*);

    void execVmul_vv(const DecodedInst*);
    void execVmul_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmulh_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVmulh_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmulh_vx(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVmulh_vx(const DecodedInst*);

    void execVmulhu_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmulhu_vx(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked);
    void execVmulhu_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmulhsu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked);
    void execVmulhsu_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmulhsu_vx(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked);
    void execVmulhsu_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmadd_vv(unsigned vd, unsigned rs1, unsigned v2, unsigned group,
                    unsigned start, unsigned elems, bool masked);
    void execVmadd_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmadd_vx(unsigned vd, unsigned rs1, unsigned v2, unsigned group,
                    unsigned start, unsigned elems, bool masked);
    void execVmadd_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vnmsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked);
    void execVnmsub_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vnmsub_vx(unsigned vd, unsigned rs1, unsigned v2, unsigned group,
                    unsigned start, unsigned elems, bool masked);
    void execVnmsub_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked);
    void execVmacc_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmacc_vx(unsigned vd, unsigned rs1, unsigned v2, unsigned group,
                    unsigned start, unsigned elems, bool masked);
    void execVmacc_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vnmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked);
    void execVnmsac_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vnmsac_vx(unsigned vd, unsigned rs1, unsigned v2, unsigned group,
                    unsigned start, unsigned elems, bool masked);
    void execVnmsac_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwmulu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked);
    void execVwmulu_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwmulu_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                   unsigned start, unsigned elems, bool masked);
    void execVwmulu_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwmul_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked);
    void execVwmul_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwmul_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                   unsigned start, unsigned elems, bool masked);
    void execVwmul_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwmulsu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked);
    void execVwmulsu_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwmulsu_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                   unsigned start, unsigned elems, bool masked);
    void execVwmulsu_vx(const DecodedInst*);

    // This is used for vwmacc.vv and vwmaccu.vv.
    template<typename ELEM_TYPE>
    void vwmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked);

    void execVwmaccu_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwmaccu_vx(unsigned vd, ELEM_TYPE e1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked);
    void execVwmaccu_vx(const DecodedInst*);

    void execVwmacc_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwmacc_vx(unsigned vd, ELEM_TYPE e1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked);
    void execVwmacc_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwmaccsu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked);
    void execVwmaccsu_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwmaccsu_vx(unsigned vd, ELEM_TYPE e1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked);
    void execVwmaccsu_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwmaccus_vx(unsigned vd, ELEM_TYPE e1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked);
    void execVwmaccus_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vdivu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVdivu_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vdivu_vx(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVdivu_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vdiv_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                 unsigned start, unsigned elems, bool masked);
    void execVdiv_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vdiv_vx(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                 unsigned start, unsigned elems, bool masked);
    void execVdiv_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vremu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVremu_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vremu_vx(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVremu_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vrem_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                 unsigned start, unsigned elems, bool masked);
    void execVrem_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vrem_vx(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                 unsigned start, unsigned elems, bool masked);
    void execVrem_vx(const DecodedInst*);

    template<typename ELEM_TYPE, typename FROM_TYPE>
    void vsext(unsigned vd, unsigned vs1, unsigned group, unsigned fromGroup,
               unsigned start, unsigned elems, bool masked);
    void execVsext_vf2(const DecodedInst*);
    void execVsext_vf4(const DecodedInst*);
    void execVsext_vf8(const DecodedInst*);

    template<typename ELEM_TYPE, typename FROM_TYPE>
    void vzext(unsigned vd, unsigned vs1, unsigned group, unsigned from,
               unsigned start, unsigned elems, bool masked);
    void execVzext_vf2(const DecodedInst*);
    void execVzext_vf4(const DecodedInst*);
    void execVzext_vf8(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vadc_vvm(unsigned vd, unsigned vs1, unsigned vs2, unsigned vcin,
                  unsigned group, unsigned start, unsigned elems);

    template<typename ELEM_TYPE>
    void vadc_vxm(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned vcin,
                  unsigned group, unsigned start, unsigned elems);

    template<typename ELEM_TYPE>
    void vsbc_vvm(unsigned vd, unsigned vs1, unsigned vs2, unsigned vbin,
                  unsigned group, unsigned start, unsigned elems);

    template<typename ELEM_TYPE>
    void vsbc_vxm(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned vbin,
                  unsigned group, unsigned start, unsigned elems);

    template<typename ELEM_TYPE>
    void vmadc_vvm(unsigned vcout, unsigned vs1, unsigned vs2, bool carry,
                   unsigned vcin, unsigned group, unsigned start,
                   unsigned elems);

    template<typename ELEM_TYPE>
    void vmadc_vxm(unsigned vcout, unsigned vs1, ELEM_TYPE e2, bool carry,
                   unsigned vcin, unsigned group, unsigned start,
                   unsigned elems);

    template<typename ELEM_TYPE>
    void vmsbc_vvm(unsigned vbout, unsigned vs1, unsigned vs2, bool borrow,
                   unsigned vbin, unsigned group, unsigned start,
                   unsigned elems);

    template<typename ELEM_TYPE>
    void vmsbc_vxm(unsigned vbout, unsigned vs1, ELEM_TYPE e2, bool borrow,
                   unsigned vbin, unsigned group, unsigned start,
                   unsigned elems);

    void execVadc_vvm(const DecodedInst*);
    void execVadc_vxm(const DecodedInst*);
    void execVadc_vim(const DecodedInst*);
    void execVsbc_vvm(const DecodedInst*);
    void execVsbc_vxm(const DecodedInst*);
    void execVmadc_vvm(const DecodedInst*);
    void execVmadc_vxm(const DecodedInst*);
    void execVmadc_vim(const DecodedInst*);
    void execVmsbc_vvm(const DecodedInst*);
    void execVmsbc_vxm(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmerge_vvm(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems);
    void execVmerge_vvm(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmerge_vxm(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                   unsigned start, unsigned elems);
    void execVmerge_vxm(const DecodedInst*);

    void execVmerge_vim(const DecodedInst*);

    void execVmv_x_s(const DecodedInst*);
    void execVmv_s_x(const DecodedInst*);
    void execVfmv_f_s(const DecodedInst*);
    void execVfmv_s_f(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmv_v_v(unsigned vd, unsigned vs1, unsigned group,
                 unsigned start, unsigned elems);
    void execVmv_v_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmv_v_x(unsigned vd, ELEM_TYPE e1, unsigned group,
                 unsigned start, unsigned elems);
    void execVmv_v_x(const DecodedInst*);
    void execVmv_v_i(const DecodedInst*);

    void execVmv1r_v(const DecodedInst*);
    void execVmv2r_v(const DecodedInst*);
    void execVmv4r_v(const DecodedInst*);
    void execVmv8r_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vsaddu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked);
    void execVsaddu_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vsaddu_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                   unsigned start, unsigned elems, bool masked);
    void execVsaddu_vx(const DecodedInst*);
    void execVsaddu_vi(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vsadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVsadd_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vsadd_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVsadd_vx(const DecodedInst*);
    void execVsadd_vi(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vssubu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked);
    void execVssubu_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vssubu_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                   unsigned start, unsigned elems, bool masked);
    void execVssubu_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vssub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVssub_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vssub_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVssub_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vaadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVaadd_vv(const DecodedInst*);
    void execVaaddu_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vaadd_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVaadd_vx(const DecodedInst*);
    void execVaaddu_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vasub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVasub_vv(const DecodedInst*);
    void execVasubu_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vasub_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVasub_vx(const DecodedInst*);
    void execVasubu_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vsmul_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVsmul_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vsmul_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVsmul_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vssr_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                 unsigned start, unsigned elems, bool masked);
    void execVssrl_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vssr_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVssrl_vx(const DecodedInst*);
    void execVssrl_vi(const DecodedInst*);

    void execVssra_vv(const DecodedInst*);
    void execVssra_vx(const DecodedInst*);
    void execVssra_vi(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vnclip_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked);
    void execVnclipu_wv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vnclip_wx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                   unsigned start, unsigned elems, bool masked);
    void execVnclipu_wx(const DecodedInst*);
    void execVnclipu_wi(const DecodedInst*);

    void execVnclip_wv(const DecodedInst*);
    void execVnclip_wx(const DecodedInst*);
    void execVnclip_wi(const DecodedInst*);

    template <typename ELEM_TYPE>
    [[nodiscard]]
    bool vectorLoad(const DecodedInst*, ElementWidth, bool faultOnFirstOnly);

    void execVle8_v(const DecodedInst*);
    void execVle16_v(const DecodedInst*);
    void execVle32_v(const DecodedInst*);
    void execVle64_v(const DecodedInst*);
    void execVle128_v(const DecodedInst*);
    void execVle256_v(const DecodedInst*);
    void execVle512_v(const DecodedInst*);
    void execVle1024_v(const DecodedInst*);

    template <typename ELEM_TYPE>
    [[nodiscard]]
    bool vectorStore(const DecodedInst*, ElementWidth);

    void execVse8_v(const DecodedInst*);
    void execVse16_v(const DecodedInst*);
    void execVse32_v(const DecodedInst*);
    void execVse64_v(const DecodedInst*);
    void execVse128_v(const DecodedInst*);
    void execVse256_v(const DecodedInst*);
    void execVse512_v(const DecodedInst*);
    void execVse1024_v(const DecodedInst*);

    void execVlm_v(const DecodedInst*);
    void execVsm_v(const DecodedInst*);

    template <typename ELEM_TYPE>
    [[nodiscard]]
    bool vectorLoadWholeReg(const DecodedInst*, ElementWidth);

    void execVlre8_v(const DecodedInst*);
    void execVlre16_v(const DecodedInst*);
    void execVlre32_v(const DecodedInst*);
    void execVlre64_v(const DecodedInst*);
    void execVlre128_v(const DecodedInst*);
    void execVlre256_v(const DecodedInst*);
    void execVlre512_v(const DecodedInst*);
    void execVlre1024_v(const DecodedInst*);

    [[nodiscard]]
    bool vectorStoreWholeReg(const DecodedInst*, GroupMultiplier);

    void execVs1r_v(const DecodedInst*);
    void execVs2r_v(const DecodedInst*);
    void execVs4r_v(const DecodedInst*);
    void execVs8r_v(const DecodedInst*);

    void execVle8ff_v(const DecodedInst*);
    void execVle16ff_v(const DecodedInst*);
    void execVle32ff_v(const DecodedInst*);
    void execVle64ff_v(const DecodedInst*);
    void execVle128ff_v(const DecodedInst*);
    void execVle256ff_v(const DecodedInst*);
    void execVle512ff_v(const DecodedInst*);
    void execVle1024ff_v(const DecodedInst*);

    template <typename ELEM_TYPE>
    [[nodiscard]]
    bool vectorLoadStrided(const DecodedInst*, ElementWidth);

    void execVlse8_v(const DecodedInst*);
    void execVlse16_v(const DecodedInst*);
    void execVlse32_v(const DecodedInst*);
    void execVlse64_v(const DecodedInst*);
    void execVlse128_v(const DecodedInst*);
    void execVlse256_v(const DecodedInst*);
    void execVlse512_v(const DecodedInst*);
    void execVlse1024_v(const DecodedInst*);

    template <typename ELEM_TYPE>
    [[nodiscard]]
    bool vectorStoreStrided(const DecodedInst*, ElementWidth);

    void execVsse8_v(const DecodedInst*);
    void execVsse16_v(const DecodedInst*);
    void execVsse32_v(const DecodedInst*);
    void execVsse64_v(const DecodedInst*);
    void execVsse128_v(const DecodedInst*);
    void execVsse256_v(const DecodedInst*);
    void execVsse512_v(const DecodedInst*);
    void execVsse1024_v(const DecodedInst*);

    template <typename ELEM_TYPE>
    [[nodiscard]]
    bool vectorLoadIndexed(const DecodedInst*, ElementWidth);

    void execVloxei8_v(const DecodedInst*);
    void execVloxei16_v(const DecodedInst*);
    void execVloxei32_v(const DecodedInst*);
    void execVloxei64_v(const DecodedInst*);
    void execVluxei8_v(const DecodedInst*);
    void execVluxei16_v(const DecodedInst*);
    void execVluxei32_v(const DecodedInst*);
    void execVluxei64_v(const DecodedInst*);

    template <typename ELEM_TYPE>
    [[nodiscard]]
    bool vectorStoreIndexed(const DecodedInst*, ElementWidth);

    void execVsoxei8_v(const DecodedInst*);
    void execVsoxei16_v(const DecodedInst*);
    void execVsoxei32_v(const DecodedInst*);
    void execVsoxei64_v(const DecodedInst*);
    void execVsuxei8_v(const DecodedInst*);
    void execVsuxei16_v(const DecodedInst*);
    void execVsuxei32_v(const DecodedInst*);
    void execVsuxei64_v(const DecodedInst*);

    template <typename ELEM_TYPE>
    [[nodiscard]]
    bool vectorLoadSeg(const DecodedInst*, ElementWidth, unsigned fields,
		       uint64_t stride, bool faultOnFirstOnly);

    void execVlsege8_v(const DecodedInst*);
    void execVlsege16_v(const DecodedInst*);
    void execVlsege32_v(const DecodedInst*);
    void execVlsege64_v(const DecodedInst*);
    void execVlsege128_v(const DecodedInst*);
    void execVlsege256_v(const DecodedInst*);
    void execVlsege512_v(const DecodedInst*);
    void execVlsege1024_v(const DecodedInst*);

    template <typename ELEM_TYPE>
    [[nodiscard]]
    bool vectorStoreSeg(const DecodedInst*, ElementWidth, unsigned fields,
			uint64_t stride);

    void execVssege8_v(const DecodedInst*);
    void execVssege16_v(const DecodedInst*);
    void execVssege32_v(const DecodedInst*);
    void execVssege64_v(const DecodedInst*);
    void execVssege128_v(const DecodedInst*);
    void execVssege256_v(const DecodedInst*);
    void execVssege512_v(const DecodedInst*);
    void execVssege1024_v(const DecodedInst*);

    void execVlssege8_v(const DecodedInst*);
    void execVlssege16_v(const DecodedInst*);
    void execVlssege32_v(const DecodedInst*);
    void execVlssege64_v(const DecodedInst*);
    void execVlssege128_v(const DecodedInst*);
    void execVlssege256_v(const DecodedInst*);
    void execVlssege512_v(const DecodedInst*);
    void execVlssege1024_v(const DecodedInst*);

    void execVsssege8_v(const DecodedInst*);
    void execVsssege16_v(const DecodedInst*);
    void execVsssege32_v(const DecodedInst*);
    void execVsssege64_v(const DecodedInst*);
    void execVsssege128_v(const DecodedInst*);
    void execVsssege256_v(const DecodedInst*);
    void execVsssege512_v(const DecodedInst*);
    void execVsssege1024_v(const DecodedInst*);

    template <typename ELEM_TYPE>
    [[nodiscard]]
    bool vectorLoadSegIndexed(const DecodedInst*, ElementWidth, unsigned fields);

    void execVluxsegei8_v(const DecodedInst*);
    void execVluxsegei16_v(const DecodedInst*);
    void execVluxsegei32_v(const DecodedInst*);
    void execVluxsegei64_v(const DecodedInst*);
    void execVluxsegei128_v(const DecodedInst*);
    void execVluxsegei256_v(const DecodedInst*);
    void execVluxsegei512_v(const DecodedInst*);
    void execVluxsegei1024_v(const DecodedInst*);

    template <typename ELEM_TYPE>
    [[nodiscard]]
    bool vectorStoreSegIndexed(const DecodedInst*, ElementWidth, unsigned fields);

    void execVsuxsegei8_v(const DecodedInst*);
    void execVsuxsegei16_v(const DecodedInst*);
    void execVsuxsegei32_v(const DecodedInst*);
    void execVsuxsegei64_v(const DecodedInst*);
    void execVsuxsegei128_v(const DecodedInst*);
    void execVsuxsegei256_v(const DecodedInst*);
    void execVsuxsegei512_v(const DecodedInst*);
    void execVsuxsegei1024_v(const DecodedInst*);

    void execVloxsegei8_v(const DecodedInst*);
    void execVloxsegei16_v(const DecodedInst*);
    void execVloxsegei32_v(const DecodedInst*);
    void execVloxsegei64_v(const DecodedInst*);
    void execVloxsegei128_v(const DecodedInst*);
    void execVloxsegei256_v(const DecodedInst*);
    void execVloxsegei512_v(const DecodedInst*);
    void execVloxsegei1024_v(const DecodedInst*);

    void execVsoxsegei8_v(const DecodedInst*);
    void execVsoxsegei16_v(const DecodedInst*);
    void execVsoxsegei32_v(const DecodedInst*);
    void execVsoxsegei64_v(const DecodedInst*);
    void execVsoxsegei128_v(const DecodedInst*);
    void execVsoxsegei256_v(const DecodedInst*);
    void execVsoxsegei512_v(const DecodedInst*);
    void execVsoxsegei1024_v(const DecodedInst*);

    void execVlsege8ff_v(const DecodedInst*);
    void execVlsege16ff_v(const DecodedInst*);
    void execVlsege32ff_v(const DecodedInst*);
    void execVlsege64ff_v(const DecodedInst*);
    void execVlsege128ff_v(const DecodedInst*);
    void execVlsege256ff_v(const DecodedInst*);
    void execVlsege512ff_v(const DecodedInst*);
    void execVlsege1024ff_v(const DecodedInst*);

    void execVfadd_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfadd_vf(unsigned vd, unsigned vs1, unsigned f2, unsigned group,
		  unsigned start, unsigned elems, bool masked);
    void execVfadd_vf(const DecodedInst*);

    void execVfsub_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfsub_vf(unsigned vd, unsigned vs1, unsigned f2, unsigned group,
		  unsigned start, unsigned elems, bool masked);
    void execVfsub_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfrsub_vf(unsigned vd, unsigned vs1, unsigned f2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfrsub_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfwadd_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwadd_vf(unsigned vd, unsigned vs1, unsigned f2, unsigned group,
		  unsigned start, unsigned elems, bool masked);
    void execVfwadd_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                 unsigned start, unsigned elems, bool masked);
    void execVfwsub_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwsub_vf(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfwsub_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwadd_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfwadd_wv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwadd_wf(unsigned vd, unsigned vs1, unsigned f2, unsigned group,
		  unsigned start, unsigned elems, bool masked);
    void execVfwadd_wf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwsub_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                 unsigned start, unsigned elems, bool masked);
    void execVfwsub_wv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwsub_wf(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfwsub_wf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfmadd_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmadd_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfmadd_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfnmadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfnmadd_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfnmadd_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfnmadd_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfmsub_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmsub_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfmsub_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfnmsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked);
    void execVfnmsub_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfnmsub_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked);
    void execVfnmsub_vf(const DecodedInst*);

    void execVfmul_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmul_vf(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		  unsigned start, unsigned elems, bool masked);
    void execVfmul_vf(const DecodedInst*);

    void execVfdiv_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfdiv_vf(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfdiv_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfrdiv_vf(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfrdiv_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwmul_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfwmul_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwmul_vf(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfwmul_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfmacc_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmacc_vf(unsigned vd, unsigned vf1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfmacc_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfnmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfnmacc_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfnmacc_vf(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfnmacc_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfmsac_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmsac_vf(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfmsac_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfnmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfnmsac_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfnmsac_vf(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfnmsac_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked);
    void execVfwmacc_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwmacc_vf(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked);
    void execVfwmacc_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwnmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked);
    void execVfwnmacc_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwnmacc_vf(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked);
    void execVfwnmacc_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked);
    void execVfwmsac_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwmsac_vf(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked);
    void execVfwmsac_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwnmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked);
    void execVfwnmsac_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwnmsac_vf(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked);
    void execVfwnmsac_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfsqrt_v(unsigned vd, unsigned vs1, unsigned group,
		     unsigned start, unsigned elems, bool masked);
    void execVfsqrt_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmerge(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		 unsigned start, unsigned elems);
    void execVfmerge_vfm(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmv_v_f(unsigned vd, unsigned rs1, unsigned group,
		  unsigned start, unsigned elems);
    void execVfmv_v_f(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmfeq_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVmfeq_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmfeq_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVmfeq_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmfne_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVmfne_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmfne_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVmfne_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmflt_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVmflt_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmflt_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVmflt_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmfle_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVmfle_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmfle_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVmfle_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmfgt_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVmfgt_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vmfge_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVmfge_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfclass_v(unsigned vd, unsigned vs1, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfclass_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfcvt_xu_f_v(unsigned vd, unsigned vs1, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfcvt_xu_f_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfcvt_x_f_v(unsigned vd, unsigned vs1, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfcvt_x_f_v(const DecodedInst*);

    void execVfcvt_rtz_xu_f_v(const DecodedInst*);

    void execVfcvt_rtz_x_f_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfcvt_f_xu_v(unsigned vd, unsigned vs1, unsigned group,
			  unsigned start, unsigned elems, bool masked);
    void execVfcvt_f_xu_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfcvt_f_x_v(unsigned vd, unsigned vs1, unsigned group,
			 unsigned start, unsigned elems, bool masked);
    void execVfcvt_f_x_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwcvt_xu_f_v(unsigned vd, unsigned vs1, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfwcvt_xu_f_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwcvt_x_f_v(unsigned vd, unsigned vs1, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfwcvt_x_f_v(const DecodedInst*);

    void execVfwcvt_rtz_xu_f_v(const DecodedInst*);

    void execVfwcvt_rtz_x_f_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwcvt_f_xu_v(unsigned vd, unsigned vs1, unsigned group,
		       unsigned start, unsigned elems, bool masked);
    void execVfwcvt_f_xu_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwcvt_f_x_v(unsigned vd, unsigned vs1, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfwcvt_f_x_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwcvt_f_f_v(unsigned vd, unsigned vs1, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfwcvt_f_f_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfncvt_xu_f_w(unsigned vd, unsigned vs1, unsigned group,
		       unsigned start, unsigned elems, bool masked);
    void execVfncvt_xu_f_w(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfncvt_x_f_w(unsigned vd, unsigned vs1, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfncvt_x_f_w(const DecodedInst*);

    void execVfncvt_rtz_xu_f_w(const DecodedInst*);

    void execVfncvt_rtz_x_f_w(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfncvt_f_xu_w(unsigned vd, unsigned vs1, unsigned group,
		       unsigned start, unsigned elems, bool masked);
    void execVfncvt_f_xu_w(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfncvt_f_x_w(unsigned vd, unsigned vs1, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfncvt_f_x_w(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfncvt_f_f_w(unsigned vd, unsigned vs1, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfncvt_f_f_w(const DecodedInst*);
    void execVfncvt_rod_f_f_w(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfredusum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfredusum_vs(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfredosum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfredosum_vs(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfredmin_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfredmin_vs(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfredmax_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfredmax_vs(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwredusum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfwredusum_vs(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfwredosum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked);
    void execVfwredosum_vs(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfrsqrt7_v(unsigned vd, unsigned vs1, unsigned group,
		    unsigned start, unsigned elems, bool masked);
    void execVfrsqrt7_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfrec7_v(unsigned vd, unsigned vs1, unsigned group,
		  unsigned start, unsigned elems, bool masked);
    void execVfrec7_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmin_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVfmin_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmin_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		 unsigned start, unsigned elems, bool masked);
    void execVfmin_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmax_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVfmax_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfmax_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		  unsigned start, unsigned elems, bool masked);
    void execVfmax_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfsgnj_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfsgnj_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfsgnj_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfsgnj_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfsgnjn_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfsgnjn_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfsgnjn_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfsgnjn_vf(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfsgnjx_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfsgnjx_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vfsgnjx_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		   unsigned start, unsigned elems, bool masked);
    void execVfsgnjx_vf(const DecodedInst*);

    void execVandn_vv(const DecodedInst*);
    void execVandn_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vbrev_v(unsigned vd, unsigned vs1, unsigned group,
		unsigned start, unsigned elems, bool masked);
    void execVbrev_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vbrev8_v(unsigned vd, unsigned vs1, unsigned group,
		unsigned start, unsigned elems, bool masked);
    void execVbrev8_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vrev8_v(unsigned vd, unsigned vs1, unsigned group,
		unsigned start, unsigned elems, bool masked);
    void execVrev8_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vclz_v(unsigned vd, unsigned vs1, unsigned group,
		unsigned start, unsigned elems, bool masked);
    void execVclz_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vctz_v(unsigned vd, unsigned vs1, unsigned group,
		unsigned start, unsigned elems, bool masked);
    void execVctz_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vcpop_v(unsigned vd, unsigned vs1, unsigned group,
		 unsigned start, unsigned elems, bool masked);
    void execVcpop_v(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vrol_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		  unsigned start, unsigned elems, bool masked);
    void execVrol_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vrol_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVrol_vx(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vror_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		  unsigned start, unsigned elems, bool masked);
    void execVror_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vror_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVror_vx(const DecodedInst*);
    void execVror_vi(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwsll_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		  unsigned start, unsigned elems, bool masked);
    void execVwsll_vv(const DecodedInst*);

    template<typename ELEM_TYPE>
    void vwsll_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                  unsigned start, unsigned elems, bool masked);
    void execVwsll_vx(const DecodedInst*);
    void execVwsll_vi(const DecodedInst*);

    void execVclmul_vv(const DecodedInst*);
    void execVclmul_vx(const DecodedInst*);
    void execVclmulh_vv(const DecodedInst*);
    void execVclmulh_vx(const DecodedInst*);
    void execVghsh_vv(const DecodedInst*);
    void execVgmul_vv(const DecodedInst*);
    void execVaesdf_vv(const DecodedInst*);
    void execVaesdf_vs(const DecodedInst*);
    void execVaesef_vv(const DecodedInst*);
    void execVaesef_vs(const DecodedInst*);
    void execVaesem_vv(const DecodedInst*);
    void execVaesem_vs(const DecodedInst*);
    void execVaesdm_vv(const DecodedInst*);
    void execVaesdm_vs(const DecodedInst*);
    void execVaeskf1_vi(const DecodedInst*);
    void execVaeskf2_vi(const DecodedInst*);
    void execVaesz_vs(const DecodedInst*);
    void execVsha2ms_vv(const DecodedInst*);
    void execVsha2ch_vv(const DecodedInst*);
    void execVsha2cl_vv(const DecodedInst*);
    void execVsm4k_vi(const DecodedInst*);
    void execVsm4r_vv(const DecodedInst*);
    void execVsm4r_vs(const DecodedInst*);
    void execVsm3me_vv(const DecodedInst*);
    void execVsm3c_vi(const DecodedInst*);

    void execAes32dsi(const DecodedInst*);
    void execAes32dsmi(const DecodedInst*);
    void execAes32esi(const DecodedInst*);
    void execAes32esmi(const DecodedInst*);
    void execAes64ds(const DecodedInst*);
    void execAes64dsm(const DecodedInst*);
    void execAes64es(const DecodedInst*);
    void execAes64esm(const DecodedInst*);
    void execAes64im(const DecodedInst*);
    void execAes64ks1i(const DecodedInst*);
    void execAes64ks2(const DecodedInst*);
    void execSha256sig0(const DecodedInst*);
    void execSha256sig1(const DecodedInst*);
    void execSha256sum0(const DecodedInst*);
    void execSha256sum1(const DecodedInst*);
    void execSha512sig0h(const DecodedInst*);
    void execSha512sig0l(const DecodedInst*);
    void execSha512sig1h(const DecodedInst*);
    void execSha512sig1l(const DecodedInst*);
    void execSha512sum0r(const DecodedInst*);
    void execSha512sum1r(const DecodedInst*);
    void execSha512sig0(const DecodedInst*);
    void execSha512sig1(const DecodedInst*);
    void execSha512sum0(const DecodedInst*);
    void execSha512sum1(const DecodedInst*);
    void execSm3p0(const DecodedInst*);
    void execSm3p1(const DecodedInst*);
    void execSm4ed(const DecodedInst*);
    void execSm4ks(const DecodedInst*);

    // Dot product (non standard) exntesion
    void execVqdot_vv(const DecodedInst*);
    void execVqdot_vx(const DecodedInst*);
    void execVqdotu_vv(const DecodedInst*);
    void execVqdotu_vx(const DecodedInst*);
    void execVqdotsu_vv(const DecodedInst*);
    void execVqdotsu_vx(const DecodedInst*);
    void execVqdotus_vx(const DecodedInst*);

    void execSinval_vma(const DecodedInst*);
    void execSfence_w_inval(const DecodedInst*);
    void execSfence_inval_ir(const DecodedInst*);

    void execCbo_clean(const DecodedInst*);
    void execCbo_flush(const DecodedInst*);
    void execCbo_inval(const DecodedInst*);
    void execCbo_zero(const DecodedInst*);
    void execPrefetch_i(const DecodedInst*);
    void execPrefetch_r(const DecodedInst*);
    void execPrefetch_w(const DecodedInst*);

    void execWrs_nto(const DecodedInst*);
    void execWrs_sto(const DecodedInst*);

    void execHfence_vvma(const DecodedInst*);
    void execHfence_gvma(const DecodedInst*);
    void execHlv_b(const DecodedInst*);
    void execHlv_bu(const DecodedInst*);
    void execHlv_h(const DecodedInst*);
    void execHlv_hu(const DecodedInst*);
    void execHlv_w(const DecodedInst*);
    void execHlvx_hu(const DecodedInst*);
    void execHlvx_wu(const DecodedInst*);
    void execHsv_b(const DecodedInst*);
    void execHsv_h(const DecodedInst*);
    void execHsv_w(const DecodedInst*);
    void execHlv_wu(const DecodedInst*);
    void execHlv_d(const DecodedInst*);
    void execHsv_d(const DecodedInst*);
    void execHinval_vvma(const DecodedInst*);
    void execHinval_gvma(const DecodedInst*);

    // zicond
    void execCzero_eqz(const DecodedInst*);
    void execCzero_nez(const DecodedInst*);

    // Zcb
    void execC_zext_h(const DecodedInst*);

    // Zfa
    void execFcvtmod_w_d(const DecodedInst*);
    void execFli_h(const DecodedInst*);
    void execFli_s(const DecodedInst*);
    void execFli_d(const DecodedInst*);
    void execFleq_h(const DecodedInst*);
    void execFleq_s(const DecodedInst*);
    void execFleq_d(const DecodedInst*);
    void execFltq_h(const DecodedInst*);
    void execFltq_s(const DecodedInst*);
    void execFltq_d(const DecodedInst*);
    void execFmaxm_h(const DecodedInst*);
    void execFmaxm_s(const DecodedInst*);
    void execFmaxm_d(const DecodedInst*);
    void execFminm_h(const DecodedInst*);
    void execFminm_s(const DecodedInst*);
    void execFminm_d(const DecodedInst*);
    void execFmvh_x_d(const DecodedInst*);
    void execFmvp_d_x(const DecodedInst*);
    void execFround_h(const DecodedInst*);
    void execFround_s(const DecodedInst*);
    void execFround_d(const DecodedInst*);
    void execFroundnx_h(const DecodedInst*);
    void execFroundnx_s(const DecodedInst*);
    void execFroundnx_d(const DecodedInst*);

    // Zfbfmin
    void execFcvt_bf16_s(const DecodedInst*);
    void execFcvt_s_bf16(const DecodedInst*);

    // Zvfbfmin
    void execVfncvtbf16_f_f_w(const DecodedInst*);
    void execVfwcvtbf16_f_f_v(const DecodedInst*);

    // Zfbfmin
    void execVfwmaccbf16_vv(const DecodedInst*);
    void execVfwmaccbf16_vf(const DecodedInst*);

    // Zacas
    void execAmocas_w(const DecodedInst*);
    void execAmocas_d(const DecodedInst*);
    void execAmocas_q(const DecodedInst*);

    //Zimop
    void execMop_r(const DecodedInst*);
    void execMop_rr(const DecodedInst*);

    //Zcmop
    void execCmop(const DecodedInst*);
  private:

    // We model non-blocking load buffer in order to undo load
    // effects after an imprecise load exception.
    struct LoadInfo
    {
      LoadInfo() = default;

      LoadInfo(unsigned size, uint64_t addr, unsigned regIx, uint64_t prev,
	       bool isWide, unsigned tag, bool fp)
	: size_(size), addr_(addr), regIx_(regIx), tag_(tag), prevData_(prev),
	  valid_(true), wide_(isWide), fp_(fp)
      { }

      bool isValid() const  { return valid_; }
      void makeInvalid() { valid_ = false; fp_ = false; }

      unsigned size_ = 0;
      uint64_t addr_ = 0;
      unsigned regIx_ = 0;
      unsigned tag_ = 0;
      uint64_t prevData_ = 0;
      bool valid_ = false;
      bool wide_ = false;
      bool fp_ = false;
    };

    // Set the program counter to the given value after clearing the
    // least significant bit.
    void setPc(URV value)
    { pc_ = value & pcMask_; }

    // Clear information changed by instruction execution.
    inline
    void resetExecInfo()
    {
      triggerTripped_ = hasInterrupt_ = hasException_ = false;
      ebreakInstDebug_ = false;
      ldStSize_ = 0;
      lastPriv_ = privMode_;
      lastVirt_ = virtMode_;
      ldStWrite_ = false;
      ldStAtomic_ = false;
      lastPageMode_ = virtMem_.mode();
      lastVsPageMode_ = virtMem_.vsMode();
      lastPageModeStage2_ = virtMem_.modeStage2();
      virtMem_.clearExecInfo();
      vecRegs_.clearTraceData();
    }

    void countBasicBlocks(bool isBranch, uint64_t physPc);
    void dumpBasicBlocks();
    void dumpInitState(const char* tag, uint64_t vaddr, uint64_t paddr);

    /// Enable given extension.
    constexpr void enableExtension(RvExtension ext, bool isEnabled)
    {
      ext_enabled_.set(static_cast<std::size_t>(ext), isEnabled);
    }

    unsigned hartIx_ = 0;        // Hart ix in system, see sysHartIndex method.
    Memory& memory_;
    IntRegs<URV> intRegs_;       // Integer register file.
    CsRegs<URV> csRegs_;         // Control and status registers.
    FpRegs fpRegs_;              // Floating point registers.
    VecRegs vecRegs_;            // Vector register file.

    Syscall<URV>& syscall_;
    URV syscallSlam_ = 0;        // Area in which to slam syscall mem changes.

    bool forceRounding_ = false;
    RoundingMode forcedRounding_ = RoundingMode::NearestEven;

    static constexpr bool rv64_ = sizeof(URV)==8; // True if 64-bit base (RV64I).
    std::bitset<static_cast<size_t>(RvExtension::None)> ext_enabled_;

    URV pc_ = 0;                 // Program counter. Incremented by instr fetch.
    URV currPc_ = 0;             // Addr instr being executed (pc_ before fetch).
    URV resetPc_ = 0;            // Pc to use on reset.
    URV stopAddr_ = 0;           // Pc at which to stop the simulator.
    URV pcMask_ = ~URV(1);       // Values are anded with this before being assigned to the program counter.
    bool stopAddrValid_ = false; // True if stopAddr_ is valid.

    URV toHost_ = 0;             // Writing to this stops the simulator.
    bool toHostValid_ = false;   // True if toHost_ is valid.

    URV fromHost_ = 0;
    bool fromHostValid_ = false;
    unsigned pendingHtifGetc_ = 0; // Count of pending HTIF get-character requests.

    URV conIo_ = 0;              // Writing a byte to this writes to console.
    bool conIoValid_ = false;    // True if conIo_ is valid.
    bool enableConIn_ = true;

    uint64_t aclintBase_ = 0;
    uint64_t aclintSize_ = 0;
    uint64_t aclintSwStart_ = 0;
    uint64_t aclintSwEnd_ = 0;
    uint64_t aclintMtimerStart_ = 0;
    uint64_t aclintMtimerEnd_ = 0;
    uint64_t aclintMtimeStart_ = 0;
    uint64_t aclintMtimeEnd_ = 0;
    uint64_t aclintAlarm_ = ~uint64_t(0); // Interrupt when timer >= this
    bool aclintSiOnReset_ = false;
    bool aclintDeliverInterrupts_ = true;
    std::function<Hart<URV>*(unsigned ix)> indexToHart_ = nullptr;

    // True if we want to defer an interrupt for later. By default, take immediately.
    URV deferredInterrupts_ = 0;

    bool  hasInterruptor_ = false;
    uint64_t interruptor_ = 0;

    URV nmiPc_ = 0;             // Non-maskable interrupt handler.
    URV nmiExceptionPc_ = 0;    // Handler for exceptions during non-maskable interrupts.
    bool nmiPending_ = false;
    NmiCause nmiCause_ = NmiCause::UNKNOWN;

    // These must be cleared before each instruction when triggers enabled.
    bool hasException_ = false;      // True if current inst has an exception.
    bool csrException_ = false;      // True if there is a CSR related exception.
    bool hasInterrupt_ = false;      // True if there is an interrupt.
    bool triggerTripped_ = false;    // True if a trigger trips.

    bool lastBranchTaken_ = false; // Useful for performance counters
    bool misalignedLdSt_ = false;  // Useful for performance counters

    bool misalAtomicCauseAccessFault_ = true;

    // True if effective and base addresses must be in regions of the
    // same type.
    bool eaCompatWithBase_ = false;

    bool csvTrace_ = false;      // Print trace in CSV format.

    bool instrLineTrace_ = false;
    bool dataLineTrace_ = false;

    unsigned cacheLineSize_ = 64;

    uint64_t& time_;             // Only hart 0 increments this value.
    uint64_t timeDownSample_ = 0;
    uint64_t timeSample_ = 0;

    uint64_t retiredInsts_ = 0;  // Proxy for minstret CSR.
    uint64_t cycleCount_ = 0;    // Proxy for mcycle CSR.
    URV      fcsrValue_ = 0;     // Proxy for FCSR.
    uint64_t instCounter_ = 0;   // Absolute retired instruction count.
    uint64_t instCountLim_ = ~uint64_t(0);
    uint64_t stimecmp_ = 0;      // Value of STIMECMP CSR.
    uint64_t vstimecmp_ = 0;     // Value of VSTIMECMP CSR.
    uint64_t htimedelta_ = 0;    // Value of HTIMEDELTA CSR.
    uint64_t exceptionCount_ = 0;
    uint64_t interruptCount_ = 0;   // Including non-maskable interrupts.
    uint64_t nmiCount_ = 0;
    uint64_t consecutiveIllegalCount_ = 0;
    uint64_t counterAtLastIllegal_ = 0;
    uint64_t lrCount_ = 0;    // Count of dispatched load-reserve instructions.
    uint64_t lrSuccess_ = 0;  // Count of successful LR (reservation acquired).
    uint64_t scCount_ = 0;    // Count of dispatched store-conditional instructions.
    uint64_t scSuccess_ = 0;  // Count of successful SC (store accomplished).
    unsigned lrResSize_ = sizeof(URV); // LR reservation size.
    bool keepReservOnScException_ = false; // Keep reservation on SC.W/D exception.

    bool instFreq_ = false;         // Collection instruction frequencies.
    bool enableCounters_ = false;   // Enable performance monitors.
    bool enableTriggers_ = false;   // Enable debug triggers.
    bool enableGdb_ = false;        // Enable gdb mode.
    int gdbTcpPort_ = -1;           // Enable gdb mode.
    bool newlib_ = false;           // Enable newlib system calls.
    bool linux_ = false;            // Enable linux system calls.
    bool amoInCacheableOnly_ = false;

    uint32_t perfControl_ = ~0;     // Performance counter control
    uint32_t prevPerfControl_ = ~0; // Value before current instruction.

    URV ldStAddr_ = 0;              // Addr of data of most recent ld/st inst.
    uint64_t ldStPhysAddr1_ = 0;    // Physical address
    uint64_t ldStPhysAddr2_ = 0;    // Physical address of 2nd page across page boundary.
    unsigned ldStSize_ = 0;         // Non-zero if ld/st/atomic.
    uint64_t ldStData_ = 0;         // For tracing
    uint64_t ldStFaultAddr_ = 0;
    bool ldStWrite_ = false;        // True if memory written by last store.
    bool ldStAtomic_ = false;       // True if amo or lr/sc

    PrivilegeMode privMode_ = PrivilegeMode::Machine;   // Privilege mode.
    PrivilegeMode lastPriv_ = PrivilegeMode::Machine;   // Before current inst.

    bool virtMode_ = false;         // True if virtual (V) mode is on.
    bool lastVirt_ = false;         // Before current inst.
    bool lastHyer_ = false;         // Hypervisor extension state before current inst.
    bool hyperLs_ = false;          // True if last instr is hypervisor load/store.

    // These are used to get fast access to the FS and VS bits.
    Emstatus<URV> mstatus_;         // Cached value of mstatus CSR or mstatush/mstatus.
    MstatusFields<URV> vsstatus_;   // Cached value of vsstatus CSR
    HstatusFields<URV> hstatus_;    // Cached value of hstatus CSR
    URV effectiveIe_ = 0;           // Effective interrupt enable.

    bool clearMprvOnRet_ = true;
    bool cancelLrOnTrap_ = true;    // Cancel reservation on traps when true.

    // Make hfence.gvma ignore huest physical addresses when true.
    bool hfenceGvmaIgnoresGpa_ = false;

    VirtMem::Mode lastPageMode_ = VirtMem::Mode::Bare;  // Before current inst
    VirtMem::Mode lastVsPageMode_ = VirtMem::Mode::Bare;
    VirtMem::Mode lastPageModeStage2_ = VirtMem::Mode::Bare;

    bool debugMode_ = false;         // True on debug mode.
    bool dcsrStepIe_ = false;        // True if stepie bit set in dcsr.
    bool dcsrStep_ = false;          // True if step bit set in dcsr.
    bool ebreakInstDebug_ = false;   // True if debug mode entered from ebreak.
    URV debugParkLoop_ = ~URV(0);    // Jump to this address on entering debug mode.
    URV debugTrapAddr_ = ~URV(0);    // Jump to this address on exception in debug mode.

    bool clearMtvalOnIllInst_ = true;
    bool clearMtvalOnEbreak_ = true;

    bool targetProgFinished_ = false;
    bool stepResult_ = false;        // Set by singleStep on caught exception (program success/fail).
    bool tracePtw_ = false;          // Trace paget table walk.
    bool mipPoked_ = false;          // Prevent MIP pokes from being clobbered by CLINT.
    bool seiPin_ = false;            // Supervisor external interrupt pin value.
    unsigned mxlen_ = 8*sizeof(URV);
    FILE* consoleOut_ = nullptr;

    int gdbInputFd_ = -1;  // Input file descriptor when running in gdb mode.

    InstProfiles instProfs_; // Instruction frequency manager

    std::vector<uint64_t> interruptStat_;  // Count of different types of interrupts.

    // Indexed by exception cause.
    std::vector<uint64_t> exceptionStat_;

    // Decoded instruction cache.
    std::vector<DecodedInst> decodeCache_;
    uint32_t decodeCacheSize_ = 0;
    uint32_t decodeCacheMask_ = 0;  // Derived from decodeCacheSize_

    // Following is for test-bench support. It allow us to cancel div/rem
    bool hasLastDiv_ = false;
    URV priorDivRdVal_ = 0;  // Prior value of most recent div/rem dest register.
    URV lastDivRd_ = 0;  // Target register of most recent div/rem instruction.

    uint64_t alarmInterval_ = 0; // Timer interrupt interval.
    uint64_t alarmLimit_ = ~uint64_t(0); // Timer interrupt when inst counter reaches this.
    uint64_t logStart_ = 0; // Start logging at this instruction rank.

    uint64_t wfiTimeout_ = 1;  // If non-zero wfi will succeed.

    bool misalDataOk_ = true;
    bool misalHasPriority_ = true;
    bool trapNonZeroVstart_ = true;  // Trap if vstart > 0 in arith vec instructions
    bool bigEnd_ = false;            // True if big endian
    bool stimecmpActive_ = false;    // True if STIMECMP CSR is implemented.
    bool vstimecmpActive_ = false;   // True if VSTIMECMP CSR is implemented.

    // Physical memory protection.
    bool pmpEnabled_ = false; // True if one or more pmp register defined.
    PmpManager pmpManager_;

    // Static tee (truseted execution environment).
    bool steeEnabled_ = false;
    TT_STEE::Stee stee_;

    VirtMem virtMem_;
    Isa isa_;
    Decoder decoder_;
    Disassembler disas_;
    std::shared_ptr<TT_IMSIC::Imsic> imsic_;
    uint64_t imsicMbase_ = 0;
    uint64_t imsicMend_ = 0;
    uint64_t imsicSbase_ = 0;
    uint64_t imsicSend_ = 0;
    std::function<bool(uint64_t, unsigned, uint64_t&)> imsicRead_ = nullptr;
    std::function<bool(uint64_t, unsigned, uint64_t)> imsicWrite_ = nullptr;
    std::shared_ptr<Pci> pci_;
    uint64_t pciConfigBase_ = 0;
    uint64_t pciConfigEnd_ = 0;
    uint64_t pciMmioBase_ = 0;
    uint64_t pciMmioEnd_ = 0;

    // Callback invoked before a CSR instruction accesses a CSR.
    std::function<void(unsigned, CsrNumber)> preCsrInst_ = nullptr;

    // Callback invoked after a CSR instruction accesses a CSR or, in
    // the case of an exception, after the CSR instruction takes the
    // exception.
    std::function<void(unsigned, CsrNumber)> postCsrInst_ = nullptr;

    // Callback invoked before execution of an instruction. Callback
    // invoked as follows: preInst_(hart, halt, reset), and upon completion
    // the hart will halt if halt is true and will reset if reset is true.
    // If both halt and reset are true, reset takes precedence.
    std::function<void(Hart<URV>&, bool&, bool&)> preInst_ = nullptr;

    // Basic-block stats.
    struct BbStat
    {
      uint64_t count_ = 0;      // Number of times basic block is entered.
      uint64_t access_ = 0;     // Data cache accesses on 1st entry to block.
      uint64_t hit_ = 0;        // Data cache hits on 1st entry to block.
    };
    uint64_t bbInsts_ = 0;              // Count of basic-block instructions.
    uint64_t bbLimit_ = ~uint64_t(0);   // Threshold at which we dump data.
    uint64_t bbPc_ = 0;                 // Entry PC of current basic block.
    uint64_t bbCacheAccess_ = 0;
    uint64_t bbCacheHit_ = 0;
    std::unordered_map<uint64_t, BbStat> basicBlocks_; // Map pc to basic-block frequency.
    FILE* bbFile_ = nullptr;            // Basic block file.

    TT_FETCH_CACHE::FetchCache fetchCache_;

    std::string branchTraceFile_;       // Branch trace file.
    struct BranchRecord
    {
      BranchRecord(char type = 0, uint64_t pc = 0, uint64_t nextPc = 0, uint8_t size = 0)
	: pc_(pc), nextPc_(nextPc), type_(type), size_(size)
      { }

      uint64_t pc_ = 0;
      uint64_t nextPc_ = 0;
      char type_ = 0;
      uint8_t size_ = 0;
    };
    boost::circular_buffer<BranchRecord> branchBuffer_;

    std::shared_ptr<Mcm<URV>> mcm_;    // Memory consistency model.
    std::shared_ptr<PerfApi> perfApi_; // Memory consistency model.
    bool ooo_ = false;                 // Out of order execution (mcm or perfApi).

    bool wrsCancelsLr_ = true;         // wrs_sto/wrs_nto instructions do cancel-lr.

    FILE* initStateFile_ = nullptr;
    std::unordered_set<uint64_t> initInstrLines_;
    std::unordered_set<uint64_t> initDataLines_;

    // Shift executed instruction counter by this amount to produce a
    // fake timer value. For example, if shift amount is 3, we are
    // dividing instruction count by 8 (2 to power 3) to produce a
    // timer value.
    unsigned timeShift_ = 0;

    bool traceHeaderPrinted_ = false;
    bool ownTrace_ = false;

    // For lockless handling of MIP. We assume the software won't
    // trigger multiple interrupts while handling. To be cleared when
    // hart marks relevant bit in MIP.
    union InterruptAlarm
    {
      struct
      {
        bool flag_ : 1;
        bool alarm_ : 1;
      } bits_;

      uint8_t value_ = 0;
    };
    InterruptAlarm swInterrupt_;

    bool suspended_ = false;      // If true, don't execute instructions.
    uint64_t resumeTime_ = 0;     // If non-zero, resume from suspension after time is greater than this value.
  };
}

