#pragma once

#include <vector>
#include <cstdint>
#include <cassert>
#include <memory>
#include <functional>
#include <iostream>
#include <fstream>
#include <sstream>


namespace TT_IMSIC      // TensTorrent Incoming Message Signaled Interrupt Controller.
{

  /// Model an IMSIC file: One enable bit and one pending bit for
  /// each interrupt identity.  For a file of size n, interrupt id 0
  /// is reserved, and valid ids are 1 to n-1. Priorities are
  /// associated with ids with the lowest id having the highest
  /// priority (per spec).
  class File
  {
  public:

    enum ExternalInterruptCsr
    {
      RES0 = 0x00,
      RES1 = 0x2F,

      IPRIO0 = 0x30,
      IPRIO15 = 0x3F,

      RES2 = 0x40,
      RES3 = 0x6F,

      DELIVERY = 0x70,
      SRES0 = 0x71,
      THRESHOLD = 0x72,
      SRES1 = 0x73,
      SRES2 = 0x7F,

      P0 = 0x80,
      P63 = 0xBF,

      E0 = 0xC0,
      E63 = 0xFF,

      RES4 = 0x100
    };

    /// Define an non-configured file. File may be later configured
    /// using the config method. An non-configured file does not cover
    /// any address.
    File ()
    { }

    /// Define a file with n-1 interrupt identities: 1 to
    /// n-1. Identity 0 is reserved. Initial value: All disabled none
    /// pending. The given address must be page aligned and n must be
    /// a multiple of 64.
    File (uint64_t address, unsigned n)
      : addr_(address), pending_(n), enabled_(n), topId_(0), config_(true)
    {
      assert((address & pageSize_) == 0);
      assert((n % 64) == 0);
    }

    /// Return the address of this file.
    uint64_t address() const
    { return addr_; }

    /// Return one plus the largest supported interrupt id.
    unsigned idCount() const
    { return pending_.size(); }

    /// Return true if given interrupt id is pending.
    bool isPending(unsigned id) const
    { return config_ and id < pending_.size() and pending_.at(id); }

    /// Return true if given interrupt id is enabled.
    bool isEnabled(unsigned id) const
    { return config_ and id < enabled_.size() and enabled_.at(id); }

    /// Mark the given interrupt id as pending/not-pending. Marking id 0
    /// or an out of bounds id has no effect.
    /// Update the highest priority enabled top id.
    void setPending(unsigned id, bool flag)
    {
      if (id > 0 and id < pending_.size())
	{
	  pending_.at(id) = flag;
	  if (enabled_.at(id) and ( topId_ == 0 or id <= topId_ ))
	    {
	      if (flag)
		topId_ = id;
	      else
		updateTopId();
	    }

          if (trace_)
            {
              selects_.emplace_back(ExternalInterruptCsr::P0 + (id >> 6), sizeof(uint64_t));
              externalInterrupts_.emplace_back(id);
            }
	}
    }

    /// Mark the given interrupt id as enabled/not-enabled. Marking id
    /// 0 or an out of bounds id has no effect.
    void setEnabled(unsigned id, bool flag)
    {
      if (id > 0 and id < enabled_.size())
	{
	  enabled_.at(id) = flag;
	  if (topId_ == 0 or (id <= topId_ and pending_.at(id)))
	    {
	      if (flag)
		topId_ = id;
	      else
		updateTopId();
	    }

          if (trace_)
            selects_.emplace_back(ExternalInterruptCsr::E0 + (id >> 6), sizeof(uint64_t));
	}
    }

    /// Set the top interrupt id threshold. Ids larger than or equal
    /// to the threshold do not participate in the computation of top
    /// id even if pending and enabled.
    void setThreshold(unsigned t)
    { threshold_ = t & thresholdMask_; }

    /// Return the top interrupt id threshold.  See setThrshold.
    unsigned threshold() const
    { return threshold_; }

    /// Return the id of the highest priority (smallest id) interrupt
    /// id that is pending and enabled and is below the threshold id.
    unsigned topId() const
    { return threshold_ == 0 ? topId_ : (topId_ >= threshold_ ? 0 : topId_); }

    /// Update the id of the highest priority interrupt that is
    /// pending and enabled.
    void updateTopId()
    {
      topId_ = 0;
      if (not config_)
	return;

      auto it = std::mismatch(pending_.begin(), pending_.end(), enabled_.begin(), [] (bool a, bool b) {
            return !(a and b);
      });

      if (it.first != pending_.end())
        topId_ = std::distance(pending_.begin(), it.first);
    }

    /// Return the enable bits of id's i*32 to i*32 + 31 packed in a word.
    /// Return 0 if i is out of bounds.
    uint32_t ithEnableWord(unsigned i) const
    {
      if (not config_)
	return false;
      unsigned word = 0;
      for (unsigned bitIx = 0; bitIx < 32; ++bitIx)
	{
	  unsigned id = i*32 + bitIx;
	  unsigned en = isEnabled(id) ? 1 : 0;
	  word |= en << bitIx;
	}
      return word;
    }

    /// Return true if the given address is valid for this file:
    /// address is word aligned and is within the page associated with
    /// this file.
    bool isValidAddress(uint64_t addr) const
    { return (addr & 3) == 0 and coversAddress(addr); }

    /// Return true if given address is in the rage of addresses
    /// covered by this file. Return false otherwise. Return false
    /// if this file is not configured.
    bool coversAddress(uint64_t addr) const
    { return config_ and addr >= addr_ and addr - addr_ < pageSize_; }

    /// Configure this file and associate it with given address and id
    /// count: interrupts ids are 1 to idCount - 1 inclusive.  All
    /// previous enable/pending state is lost. Given address must be
    /// page aligned and idCount must be a multiple of 64.
    void configure(uint64_t addr, unsigned idCount, unsigned pageSize,
                   unsigned thresholdMask)
    {
      pageSize_ = pageSize;
      assert((addr % pageSize_) == 0);
      assert((idCount % 64) == 0);
      topId_ = 0;
      addr_ = addr;
      delivery_ = 0;
      threshold_ = 0;
      thresholdMask_ = thresholdMask;
      pending_.clear();
      enabled_.clear();
      pending_.resize(idCount);
      enabled_.resize(idCount);
      config_ = true;
    }

    /// Return true if this can delier an interrupt to its hart
    /// (i.e. delivery register set to 1).
    bool canDeliver() const
    { return delivery_ == 1; }

    /// Return true if this file is configured.
    bool isConfigured() const
    { return config_; }

    unsigned pageSize() const
    { return pageSize_; }

    /// Read from selected register into val. Returns false if access
    /// would cause an illegal instruction trap and true otherwise.
    template <typename URV>
    bool iregRead(unsigned select, URV& val) const;

    /// Write to selected register from val. Returns false if access
    /// would cause an illegal instruction trap and true otherwise.
    template <typename URV>
    bool iregWrite(unsigned select, URV val);

    void enableTrace(bool flag)
    { trace_ = flag; }

    void iregModified(std::vector<std::pair<unsigned, unsigned>>& selects) const
    { selects = selects_; }

    void externalInterrupts(std::vector<unsigned>& interrupts) const
    { interrupts = externalInterrupts_; }

    /// Clear trace related information.
    void clearTrace()
    {
      selects_.clear();
      externalInterrupts_.clear();
    }

    bool saveSnapshot(const std::string& filename) const
    {
      std::ofstream ofs(filename, std::ios::trunc);
      if (not ofs)
        {
          std::cerr << "Imsic::saveSnapshot failed - canot open " << filename << " for write\n";
          return false;
        }

      ofs << "p ";
      for (auto p : pending_)
        ofs << p;
      ofs << '\n';

      ofs << "e ";
      for (auto e : enabled_)
        ofs << e;
      ofs << '\n';

      ofs << "d " << delivery_ << '\n';
      ofs << "t " << threshold_ << '\n';

      return true;
    }

    bool loadSnapshot(const std::string& filename)
    {
      std::ifstream ifs(filename);
      std::string line;
      uint64_t errors = 0;
      unsigned lineNum = 0;

      while (std::getline(ifs, line))
        {
          lineNum++;
          std::istringstream iss(line);
          std::string reg;
          std::string val;

          if (not (iss >> reg))
            {
              std::cerr << "Error: Imsic snapshot loader: Line " << lineNum
                        << ": Failed to read register name\n";
              errors++;
              continue;
            }

          if (not (iss >> val))
            {
              std::cerr << "Error: Imsic snapshot loader: Line " << lineNum
                        << ": Failed to read register value\n";
              errors++;
              continue;
            }

          if (reg == "p")
            {
              if (val.size() != pending_.size())
                std::cerr << "Warning: Imsic snapshot loader: Line " << lineNum
                          << ": Mismatched number of interrupt ids specified\n";

              unsigned num = std::min(val.size(), pending_.size());
              for (unsigned i = 0; i < num; ++i)
                pending_.at(i) = val.at(i) - '0';
            }

          if (reg == "e")
            {
              if (val.size() != enabled_.size())
                std::cerr << "Warning: Imsic snapshot loader: Line " << lineNum
                          << ": Mismatched number of interrupt ids specified\n";

              unsigned num = std::min(val.size(), pending_.size());
              for (unsigned i = 0; i < num; ++i)
                enabled_.at(i) = val.at(i) - '0';
            }

          if (reg == "t")
            threshold_ = strtoull(val.c_str(), nullptr, 0);

          if (reg == "d")
            delivery_ = strtoull(val.c_str(), nullptr, 0);
        }

      return errors == 0;
    }

  private:

    uint64_t addr_ = 0;
    std::vector<bool> pending_;
    std::vector<bool> enabled_;
    unsigned topId_ = 0;
    unsigned delivery_ = 0;
    unsigned threshold_ = 0;
    unsigned thresholdMask_ = 0;
    bool config_ = false;
    unsigned pageSize_ = 4096;

    // For coverage information
    bool trace_ = false;
    // IMSIC updates from hart-side
    std::vector<std::pair<unsigned, unsigned>> selects_;
    // IMSIC updates from external-side
    std::vector<unsigned> externalInterrupts_;
  };


  /// Model an IMSIC device.
  class Imsic
  {
  public:

    /// Define a non-configured IMSIC (no machine/supervisor/guest
    /// file, does not cover any address).
    Imsic()
      : mfile_(0, 0), sfile_(0, 0)
    { }

    /// Configure the machine privilege file at the given address and
    /// with the given count of interrupt ids (idCount is the highest
    /// interrupt id plus 1 and must be a multiple of 64). All
    /// previous state enabled/pending state is lost.
    void configureMachine(uint64_t addr, unsigned idCount, unsigned pageSize,
                          unsigned thresholdMask)
    { mfile_.configure(addr, idCount, pageSize, thresholdMask); }

    /// Configure the supervisor privilege file at the given address
    /// and with the given count of interrupt ids (idCount is the
    /// highest interrupt id plus 1 and must be a multiple of 64).
    /// All previous state enabled/pending state is lost.
    void configureSupervisor(uint64_t addr, unsigned idCount, unsigned pageSize,
                             unsigned thresholdMask)
    { sfile_.configure(addr, idCount, pageSize, thresholdMask); }

    /// Configure g guest files of n-1 interrupt ids each. The guest
    /// addresses will be s+p, s+2p, s+3p, ... where s is the
    /// supervisor file address and p is the page size. A supervisor
    /// file must be configured before the guests files are
    /// configured. The parameter g must not exceed 64.
    void configureGuests(unsigned g, unsigned n, unsigned pageSize,
                         unsigned thresholdMask)
    {
      assert(g <= 64);
      assert(sfile_.isConfigured());
      gfiles_.clear();
      gfiles_.resize(g + 1);
      assert(sfile_.pageSize() == pageSize);
      uint64_t addr = sfile_.address() + pageSize;
      for (size_t i = 1; i < gfiles_.size(); ++i)
	{
	  gfiles_.at(i).configure(addr, n, pageSize, thresholdMask);
	  addr += pageSize;
	}
    }

    /// Return the physical address for machine privilege file in this
    /// IMSIC. This is also the address for little-endian access.
    uint64_t machineAddress() const
    { return mfile_.address(); }

    /// Return the physical address of this IMSIC to be used for
    /// writing in big-endian mode for machine privilege file.
    uint64_t bigEndianMachineAddress() const
    { return mfile_.address() + 4; }

    /// Return the physical address for supervisor privilege file in this
    /// IMSIC. This is also the address for little-endian access.
    uint64_t supervisorAddress() const
    { return sfile_.address(); }

    /// Return the physical address of this IMSIC to be used for
    /// writing in big-endian mode for supervisor privilege file.
    uint64_t bigEndianSupervisorAddress() const
    { return sfile_.address() + 4; }

    /// Return true if given address is valid for this IMSIC:
    /// Within the machine/supervisor/guest pages associated with
    /// this IMSIC and word aligned.
    bool isValidAddress(uint64_t addr) const
    {
      if (mfile_.isValidAddress(addr) or sfile_.isValidAddress(addr))
	return true;
      for (const auto& gfile : gfiles_)
	if (gfile.isValidAddress(addr))
	  return true;
      return false;
    }

    /// Write to this IMSIC. Return false doing nothing if address is
    /// not valid. Return true and perform a write if address is
    /// valid.  Write may be a no-op if written data corresponds to a
    /// non-implemented id.
    bool write(uint64_t addr, unsigned size, uint64_t data);

    /// Read from this IMSIC. Return false doing nothing if address is
    /// not valid. Return true and perform a read if address is valid.
    /// Read from an IMSIC always returns a zero.
    bool read(uint64_t addr, unsigned size, uint64_t& data) const
    {
      if (size != 4)
	return false;
      if (not isValidAddress(addr))
	return false;
      data = 0;
      return true;
    }

    /// Poke MIP in the associated hart to clear/assert an interrupt based on IMSIC state
    /// change.
    void checkMInterrupt()
    {
      if (mInterrupt_)
        {
          if (mfile_.canDeliver() and mfile_.topId())
            mInterrupt_(true);
          else
            mInterrupt_(false);
        }
    }

    /// Set/clear the supervisor external interupt pin (SEI pin) in the associated hart to
    /// assert/cleaer a supervisor external interrupt on IMSIC state change.
    void checkSInterrupt()
    {
      if (sInterrupt_)
        {
          if (sfile_.canDeliver() and sfile_.topId())
            sInterrupt_(true);
          else
            sInterrupt_(false);
        }
    }

    /// Poke HGEIP and MIP in the associated hart to to clear/assert a guest external
    /// interrupt based on IMSIC state change.
    void checkGInterrupt(unsigned guest)
    {
      if (guest >= gfiles_.size())
        return;
      auto& gfile = gfiles_.at(guest);

      if (gInterrupt_)
        {
          if (gfile.canDeliver() and gfile.topId())
            gInterrupt_(true, guest);
          else
            gInterrupt_(false, guest);
        }
    }

    /// Called form the associated hart for a CSR write to mireg.
    template <typename URV>
    bool writeMireg(unsigned select, URV value)
    {
      if (not isFileSelAccessible<URV>(select, false) or isFileSelReserved(select))
        return false;
      bool ok = mfile_.iregWrite(select, value);
      checkMInterrupt();
      return ok;
    }

    /// Called form the associated hart for a CSR write to sireg/vsireg.
    /// Guest field is used if virt is true.
    template <typename URV>
    bool writeSireg(bool virt, unsigned guest, unsigned select, URV value)
    {
      if (not isFileSelAccessible<URV>(select, virt) or isFileSelReserved(select))
        return false;

      if (virt)
	{
	  if (not guest or guest >= gfiles_.size())
	    return false;
	  auto& gfile = gfiles_.at(guest);
	  bool ok = gfile.iregWrite(select, value);
          checkGInterrupt(guest);
	  return ok;
	}

      bool ok = sfile_.iregWrite(select, value);
      checkSInterrupt();
      return ok;
    }

    /// Called form the associated hart for a CSR read from mireg.
    template <typename URV>
    bool readMireg(unsigned select, URV& value) const
    {
      if (not isFileSelAccessible<URV>(select, false) or isFileSelReserved(select))
        return false;
      return mfile_.iregRead(select, value);
    }


    /// Called form the associated hart for a CSR read from sireg/vsireg.
    /// Guest field is used if virt is true.
    template <typename URV>
    bool readSireg(bool virt, unsigned guest, unsigned select, URV& value) const
    {
      if (not isFileSelAccessible<URV>(select, false) or isFileSelReserved(select))
        return false;

      if (virt)
	{
	  if (not guest or guest >= gfiles_.size())
	    return false;
	  auto& gfile = gfiles_.at(guest);
	  return gfile.iregRead(select, value);
	}
      return sfile_.iregRead(select, value);
    }

    /// Return the machine file top pending and enabled interrupt id.
    /// Ids filetered out by the threshold mechanism are not
    /// considered.
    unsigned machineTopId() const
    { return mfile_.topId(); }

    /// Return the supervisor file top pending and enabled interrupt id.
    /// Ids filetered out by the threshold mechanism are not
    /// considered.
    unsigned supervisorTopId() const
    { return sfile_.topId(); }

    /// Return the supervisor file top pending and enabled interrupt id.
    /// Ids filetered out by the threshold mechanism are not
    /// considered.
    unsigned guestTopId(unsigned guestIx) const
    { return guestIx < gfiles_.size() ? gfiles_.at(guestIx).topId() : 0; }

    /// Rsturn true if machine file of this IMSIC is enabled to
    /// deliver itnerrupts to the associated hart (i.e. delivery is 1
    /// in the file).
    bool machineEnabled() const
    { return mfile_.canDeliver(); }

    /// Rsturn true if supervisor file of this IMSIC is enabled to
    /// deliver itnerrupts to the associated hart (i.e. delivery is 1
    /// in the file).
    bool supervisorEnabled() const
    { return sfile_.canDeliver(); }

    /// In the machine file, set/clear the pending bit correponding to
    /// the given interrupt id.
    void setMachinePending(unsigned id, bool flag)
    {
      mfile_.setPending(id, flag);
      checkMInterrupt();
    }

    /// In the supervisor file, set/clear the pending bit correponding
    /// to the given interrupt id.
    void setSupervisorPending(unsigned id, bool flag)
    {
      sfile_.setPending(id, flag);
      checkSInterrupt();
    }

    /// In the file of the given guest, set/clear the pending bit
    /// correponding to the given interrupt id.
    void setGuestPending(unsigned guest, unsigned id, bool flag)
    {
      if (guest < gfiles_.size())
	gfiles_.at(guest).setPending(id, flag);
      checkGInterrupt(guest);
    }

    /// Return the number of guests configured into this IMSIC.
    size_t guestCount() const
    { return gfiles_.size(); }

    /// Attach interrupt callbacks
    void attachMInterrupt(const std::function<void(bool)>& cb)
    { mInterrupt_ = cb; }

    void attachSInterrupt(const std::function<void(bool)>& cb)
    { sInterrupt_ = cb; }

    void attachGInterrupt(const std::function<void(bool, unsigned)>& cb)
    { gInterrupt_ = cb; }

    void enableTrace(bool flag)
    {
      trace_ = flag;
      mfile_.enableTrace(flag);
      sfile_.enableTrace(flag);
      for (auto& g : gfiles_)
        g.enableTrace(flag);
    }

    void fileTraces(std::vector<std::pair<unsigned, unsigned>>& mselects,
                    std::vector<std::pair<unsigned, unsigned>>& sselects,
                    std::vector<std::vector<std::pair<unsigned, unsigned>>>& gselects,
                    std::vector<unsigned>& minterrupts,
                    std::vector<unsigned>& sinterrupts,
                    std::vector<std::vector<unsigned>> ginterrupts) const
    {
      if (not trace_)
        return;

      mselects.clear();
      sselects.clear();
      gselects.clear();

      mfile_.iregModified(mselects);
      sfile_.iregModified(sselects);
      for (auto& g : gfiles_)
        {
          std::vector<std::pair<unsigned, unsigned>> tmp;
          g.iregModified(tmp);
          gselects.push_back(std::move(tmp));
        }

      minterrupts.clear();
      sinterrupts.clear();
      ginterrupts.clear();

      mfile_.externalInterrupts(minterrupts);
      sfile_.externalInterrupts(sinterrupts);
      for (auto& g : gfiles_)
        {
          std::vector<unsigned> tmp;
          g.externalInterrupts(tmp);
          ginterrupts.push_back(std::move(tmp));
        }
    }

    void clearTrace()
    {
      if (not trace_)
        return;

      mfile_.clearTrace();
      sfile_.clearTrace();
      for (auto& g : gfiles_)
        g.clearTrace();
    }

    /// Returns false if IMSIC region is marked inaccessible.
    template <typename URV>
    static constexpr bool isFileSelAccessible(unsigned select, bool virt)
    {
      using EIC = File::ExternalInterruptCsr;

      if (select == EIC::SRES0 or (select >= EIC::SRES1 and select <= EIC::SRES2))
        return true;

      if constexpr (sizeof(URV) == 8)
        if (select & 1)
          return false;

      /// The guest file marks the prio array as inaccessible.
      if (virt)
        if (select >= EIC::IPRIO0 and select <= EIC::IPRIO15)
          return false;

      return true;
    }

    /// Returns true if IMSIC region is marked reserved.
    static constexpr bool isFileSelReserved(unsigned select)
    {
      using EIC = File::ExternalInterruptCsr;

      if ((select >= EIC::RES0 and select <= EIC::RES1) or
          (select >= EIC::RES2 and select <= EIC::RES3) or
          (select >= EIC::RES4))
        return true;
      return false;
    }

    bool saveSnapshot(const std::string& filename) const
    {
      bool ok = mfile_.saveSnapshot(filename + "m");
      ok = ok and sfile_.saveSnapshot(filename + "s");
      for (unsigned i = 1; i < gfiles_.size(); ++i)
        ok = ok and gfiles_.at(i).saveSnapshot(filename + "g" + std::to_string(i));
      return ok;
    }

    bool loadSnapshot(const std::string& filename)
    {
      bool ok = mfile_.loadSnapshot(filename + "m");
      ok = ok and sfile_.loadSnapshot(filename + "s");
      for (unsigned i = 1; i < gfiles_.size(); ++i)
        ok = ok and gfiles_.at(i).loadSnapshot(filename + "g" + std::to_string(i));
      return ok;
    }

  private:

    File mfile_;
    File sfile_;
    std::vector<File> gfiles_;
    bool trace_ = false;

    // Callback functions to mark interrupt pending-and-enabled (or not).
    std::function<void(bool flag)> mInterrupt_;
    std::function<void(bool flag)> sInterrupt_;
    std::function<void(bool flag, unsigned guest)> gInterrupt_;
  };


  /// Model a collection of IMSIC devices.
  class ImsicMgr
  {
  public:

    /// Constructor.
    ImsicMgr(unsigned hartCount, unsigned pageSize);

    /// Configure machine privilege IMISC files (one per
    /// hart). Machine files will be at addresses addr, addr + stride,
    /// addr + 2*stride ...  Memory region reserved for machine files
    /// will be [addr, addr + n*stride - 1] where n is the number of
    /// harts in the system. The ids parameter is one plus the largest
    /// interrupt id supported by each file and must be a multiple of
    /// 64. Return true on success and false on failure: We fail if
    /// addr/stride are nog page aligned or if ids is larger than 64.
    bool configureMachine(uint64_t addr, uint64_t stride, unsigned ids,
                          unsigned thresholdMask);

    /// Configure supervisor privilege IMISC files (one per hart) and
    /// guest privilege file (guestCount per hart). Supervisor files
    /// will be at addresses addr, addr + stride, addr + 2*stride.
    /// See configMachine.
    bool configureSupervisor(uint64_t addr, uint64_t stride, unsigned ids,
                             unsigned thresholdMask);

    /// Configure n guests per hart. Guest files at each hart will be
    /// at s + ps, s + 2*ps, s + 3*ps, ... where s is the address of
    /// the supervisor file for that hart and ps is the page size.
    /// Return true on success and false on failure. Fail if
    /// supervisor files are not configured or if n is larger than 64
    /// or if ids is not a multiple of 64.
    bool configureGuests(unsigned n, unsigned ids, unsigned thresholdMask);

    /// Return true if given address is covered by any of the configured
    /// IMSIC files.
    bool coversAddress(uint64_t addr) const
    { return isMachineAddr(addr) or isSupervisorAddr(addr); }

    /// Perform a memory read operation to the memory region
    /// associated with the IMSIC devices. Return true on success
    /// setting value (reads from this region always yield zero).
    /// Return false if address is out of bounds, or address is not
    /// word aligned or if size is not 4.
    bool read(uint64_t addr, unsigned size, uint64_t& data) const
    {
      if (not coversAddress(addr) or size != 4)
	return false;
      data = 0;
      return true;
    }

    /// Perform a memory write operation to the memory region
    /// associated with the IMSIC devices. Return true on success.
    /// Return false if address is out of bounds, if address is not
    /// word aligned, or if size is not 4. If successful, hartIx to the hart
    /// index associated with the targeted IMSIC.
    bool write(uint64_t addr, unsigned size, uint64_t data, unsigned& hartIx)
    {
      if (size != 4)
	return false;
      unsigned ix = 0;
      if (isMachineAddr(addr))
	ix = (addr - mbase_) / mstride_;
      else if (isSupervisorAddr(addr))
	ix = (addr - sbase_) / sstride_;
      else
	return false;

      hartIx = ix;
      auto imsic = imsics_.at(ix);
      imsic->write(addr, size, data);
      return true;
    }

    /// Same as above but without a hart-index parameter.
    bool write(uint64_t addr, unsigned size, uint64_t data)
    {
      if (size != 4)
	return false;
      unsigned ix = 0;
      if (isMachineAddr(addr))
	ix = (addr - mbase_) / mstride_;
      else if (isSupervisorAddr(addr))
	ix = (addr - sbase_) / sstride_;
      else
	return false;

      auto imsic = imsics_.at(ix);
      return imsic->write(addr, size, data);
    }

    /// For the given hart index, return the highest priority pending
    /// and enabled machine interrup id or 0 if no interrupt and
    /// enabled. Interrupt ids filtered out by the threshold mechanism
    /// are not considered.
    unsigned machineTopId(unsigned hartIx) const
    { return hartIx < imsics_.size()? imsics_.at(hartIx)->machineTopId() : 0; }

    /// For the given hart index, return the highest priority pending
    /// and enabled supervisor interrup id or 0 if no interrupt and
    /// enabled. Interrupt ids filtered out by the threshold mechanism
    /// are not considered.
    unsigned supervisorTopId(unsigned hartIx) const
    { return hartIx < imsics_.size()? imsics_.at(hartIx)->supervisorTopId() : 0; }

    /// For the given hart and guest indices (virtual guest associated
    /// with a hart), return the highest priority pending and enabled
    /// supervisor interrup id or 0 if no interrupt and
    /// enabled. Interrupt ids filtered out by the threshold mechanism
    /// are not considered.
    unsigned guestTopId(unsigned hartIx, unsigned guestIx) const
    { return hartIx < imsics_.size()? imsics_.at(hartIx)->guestTopId(guestIx) : 0; }

    /// Called form hart of given hartIx for a CSR write to mireg.
    template <typename URV>
    bool writeMireg(unsigned hartIx, unsigned sel, URV value)
    {
      if (hartIx >= imsics_.size())
	return false;
      return imsics_.at(hartIx)->writeMireg(sel, value);
    }

    /// Called form hart of given hartIx for a CSR write to sireg/vsireg.
    /// Guest field is used if virt is true.
    template<typename URV>
    bool writeSireg(unsigned hartIx, bool virt, unsigned guest, unsigned sel, URV value)
    {
      if (hartIx >= imsics_.size())
	return false;
      return imsics_.at(hartIx)->writeSireg(virt, guest, sel, value);
    }

    /// Return the number of IMSICs in this manager.
    size_t size() const
    { return imsics_.size(); }

    /// Return a pointer to the ith imsic or null if i is out of
    /// bounds.
    std::shared_ptr<Imsic> ithImsic(unsigned i)
    {
      if (i >= imsics_.size())
	return std::shared_ptr<Imsic>();
      return imsics_.at(i);
    }

    bool saveSnapshot(const std::string& filename) const
    {
      bool ok = true;
      for (unsigned i = 0; i < imsics_.size(); ++i)
        ok = ok and imsics_.at(i)->saveSnapshot(filename + std::to_string(i));
      return ok;
    }

    bool loadSnapshot(const std::string& filename) const
    {
      bool ok = true;
      for (unsigned i = 0; i < imsics_.size(); ++i)
        ok = ok and imsics_.at(i)->loadSnapshot(filename + std::to_string(i));
      return ok;
    }

  protected:

    /// Return true if given address is withing the address range of the
    /// machine privilege IMSIC files.
    bool isMachineAddr(uint64_t addr) const
    { return addr >= mbase_ and addr < mbase_ + imsics_.size() * mstride_; }

    /// Return true if given address is withing the address range of the
    /// supervisor privilege IMSIC files (which includes the guest files).
    bool isSupervisorAddr(uint64_t addr) const
    { return addr >= sbase_ and addr < sbase_ + imsics_.size() * sstride_; }

    /// Set ix to the index of the hart covering the

  private:

    unsigned pageSize_ = 0;
    uint64_t mbase_ = 0;     // Base address of machine files.
    uint64_t mstride_ = 0;   // Memory space reserved for each machine file.
    uint64_t sbase_ = 0;     // Base address of supervisor files.
    uint64_t sstride_ = 0;   // Memory space reserved for each supervisor file.

    std::vector< std::shared_ptr<Imsic> > imsics_;
  };
}
