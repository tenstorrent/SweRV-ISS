#pragma once

#include <vector>
#include <cstdint>
#include <cassert>


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
      DELIVERY = 0x70,
      THRESHOLD = 0x72,

      P0 = 0x80,
      P63 = 0xBF,

      E0 = 0xC0,
      E63 = 0xFF
    };

    /// Define an inactive file. File may be later activated it using
    /// the activate method. An inactive file does not cover any
    /// address.
    File ()
    { }

    /// Define a file with n-1 interrupt identities: 1 to
    /// n-1. Identity 0 is reserved. Initial value: All disabled none
    /// pending. The given address must be page aligned and n must be
    /// a multiple of 64.
    File (uint64_t address, unsigned n)
      : addr_(address), pending_(n), enabled_(n), topId_(0), active_(true)
    {
      assert((address & pageSize_) == 0);
      assert((n % 64) == 0);
    }

    /// Return the address of this file.
    uint64_t address() const
    { return addr_; }

    /// Return one plus the largest supported interrupt id.
    unsigned size() const
    { return pending_.size(); }

    /// Return true if given interrupt id is pending.
    bool isPending(unsigned id) const
    {
      if (not active_)
	return false;
      return id < pending_.size() and pending_.at(id);
    }

    /// Return true if given interrupt id is enabled.
    bool isEnabled(unsigned id) const
    {
      if (not active_)
	return false;
      return id < enabled_.size() and enabled_.at(id);
    }

    /// Mark the given interrupt id as pending/not-pending. Marking id 0
    /// or an out of bounds id has no effect. Update the highest priority
    /// enabled top id.
    void setPending(unsigned id, bool flag)
    {
      if (id > 0 and id < pending_.size())
	{
	  pending_.at(id) = flag;
	  if (flag and enabled_.at(id) and id < topId_)
	    topId_ = id;
	  else
	    updateTopId();
	}
    }

    /// Mark the given interrupt id as enabled/not-enabled. Marking id
    /// 0 or an out of bounds id has no effect.
    void setEnabled(unsigned id, bool flag)
    {
      if (id > 0 and id < enabled_.size())
	{
	  enabled_.at(id) = flag;
	  if (flag and pending_.at(id) and id < topId_)
	    topId_ = id;
	  else
	    updateTopId();
	}
    }

    /// Set the top interrupt id threshold. Ids larger than or equal
    /// to the threshold do not participate in the computation of top
    /// id even if pending and enabled.
    void setThreshold(unsigned t)
    { threshold_ = t; }

    /// Return the top interrupt id threshold.  See setThrshold.
    unsigned threshold() const
    { return threshold_; }

    /// Return the id of the highest priority (smallest id) interrupt
    /// id that is pending and enabled and exceeds the threshold id.
    unsigned topId() const
    { return threshold_ == 0 ? 0 : std::min(topId_, threshold_ - 1); }

    /// Update the id of the highest priority interrupt that is
    /// pending and enabled.
    void updateTopId()
    {
      topId_ = 0;
      if (not active_)
	return;
      for (size_t i = 1; i < pending_.size(); ++i)
	if (pending_.at(i) and enabled_.at(i))
	  {
	    topId_ = i;
	    return;
	  }
    }

    /// Return the enable bits of id's i*32 to i*32 + 31 packed in a word.
    /// Return 0 if i is out of bounds.
    uint32_t ithEnableWord(unsigned i) const
    {
      if (not active_)
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
    /// if this file is not active.
    bool coversAddress(uint64_t addr) const
    { return active_ and addr >= addr_ and addr - addr_ < pageSize_; }

    /// Mark this file as active and associate it with given address
    /// and size: interrupts ids are 1 to size - 1 inclusive.
    /// All previous enable/pending state is lost. Given address must
    /// be page aligned and size must be a multiple of 64.
    void activate(uint64_t addr, unsigned size)
    {
      assert((addr % pageSize_) == 0);
      assert((size % 64) == 0);
      topId_ = 0;
      addr_ = addr;
      pending_.clear();
      enabled_.clear();
      pending_.resize(size);
      enabled_.resize(size);
      active_ = true;
    }

    /// Return true if this file is active.
    bool isActive() const
    { return active_; }

    unsigned pageSize() const
    { return pageSize_; }

    /// Select CSR for *ireg if valid.
    void select(uint8_t num)
    { select_ = num; }

    /// Read from selected register by selectRegister. Returns false if
    /// access would cause an illegal instruction trap and true otherwise.
    template <typename URV>
    bool read(URV& val) const;

    /// Write to selected register by selectRegister. Returns false if
    /// access would cause an illegal instruction trap and true otherwise.
    template <typename URV>
    bool write(URV val);

  private:

    uint64_t addr_ = 0;
    std::vector<bool> pending_;
    std::vector<bool> enabled_;
    unsigned topId_ = 0;
    unsigned delivery_ = 0;
    unsigned threshold_ = 0;
    bool active_ = false;
    const unsigned pageSize_ = 1024;
    uint64_t select_ = 0; // cached value of indirect register select
  };


  /// Model an IMSIC device.
  class Imsic
  {
  public:

    /// Define an inactive IMSIC (no machine/supervisor/guest file,
    /// does not cover any address).
    Imsic()
      : mfile_(0, 0), sfile_(0, 0)
    { }

    /// Activate the machine privilege file at the given address and
    /// with the given size (size is the highest interrupt id plus 1
    /// and must be a multiple of 64). All previous state
    /// enabled/pending state is lost.
    void activateMachine(uint64_t addr, unsigned size)
    { mfile_.activate(addr, size); }

    /// Activate the supervisor privilege file at the given address
    /// and with the given size (size is the highest interrupt id plus
    /// 1 and must be a multiple of 64).  All previous state
    /// enabled/pending state is lost.
    void activateSupervisor(uint64_t addr, unsigned size)
    { sfile_.activate(addr, size); }

    /// Activate g guest files of n-1 interrupt ids each. The guest
    /// addresses will be s+p, s+2p, s+3p, ... where s is the
    /// supervisor file address and p is the page size. A supervisor
    /// file must be activated before the guests files are
    /// activated. The parameter g must not exceed 64.
    void activateGuests(unsigned g, unsigned n)
    {
      assert(g <= 64);
      assert(sfile_.isActive());
      gfiles_.clear();
      gfiles_.resize(g);
      unsigned pageSize = sfile_.pageSize();
      uint64_t addr = sfile_.address() + pageSize;
      for (auto& file : gfiles_)
	{
	  file.activate(addr, n);
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
    bool read(uint64_t addr, unsigned size, uint64_t& data)
    {
      if (size != 4)
	return false;
      if (not isValidAddress(addr))
	return false;
      data = 0;
      return true;
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

  private:

    File mfile_;
    File sfile_;
    std::vector<File> gfiles_;
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
    bool configMachine(uint64_t addr, uint64_t stride, unsigned ids);

    /// Configure supervisor privilege IMISC files (one per hart) and
    /// guest privilege file (guestCount per hart). Supervisor files
    /// will be at addresses addr, addr + stride, addr + 2*stride.
    /// See configMachine.
    bool configSupervisor(uint64_t addr, uint64_t stride, unsigned ids);

    /// Configure n guests per hart. Guest files at each hart will be
    /// at s + ps, s + 2*ps, s + 3*ps, ... where s is the address of
    /// the supervisor file for that hart and ps is the page size.
    /// Return true on success and false on failure. Fail if
    /// supervisor files are not configured or if n is larger than 64
    /// or if ids is not a multiple of 64.
    bool configGuests(unsigned n, unsigned ids);

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
      if (not coversAddress(addr) or size != 4 or (size & 3) != 0)
	return false;
      data = 0;
      return true;
    }

    /// Perform a memory write operation to the memory region
    /// associated with the IMSIC devices. Return true on success.
    /// Return false if address is out of bounds, if address is not
    /// word aligned, or if size is not 4. If successful, hartIx to the hart
    /// index associated with the targeted IMSIC.
    bool write(uint64_t addr, unsigned size, uint64_t data,
	       unsigned& hartIx)
    {
      if (size != 4 or (size & 3) != 0)
	return false;
      unsigned ix = 0;
      if (isMachineAddr(addr))
	{
	  ix = (addr - mbase_) / mstride_;
	  assert(ix < harts_);
	}
      else if (isSupervisorAddr(addr))
	{
	  ix = (addr - mbase_) / mstride_;
	  assert(ix < harts_);
	}
      else
	return false;

      hartIx = ix;
      auto& imsic = imsics_.at(ix);
      imsic.write(addr, size, data);
      return true;
    }

    /// For the given hart index, return the highest priority pending
    /// and enabled machine interrup id or 0 if no interrupt and
    /// enabled. Interrupt ids filtered out by the threshold mechanism
    /// are not considered.
    unsigned machineTopId(unsigned hartIx) const
    { return hartIx < imsics_.size()? imsics_.at(hartIx).machineTopId() : 0; }

    /// For the given hart index, return the highest priority pending
    /// and enabled supervisor interrup id or 0 if no interrupt and
    /// enabled. Interrupt ids filtered out by the threshold mechanism
    /// are not considered.
    unsigned supervisorTopId(unsigned hartIx) const
    { return hartIx < imsics_.size()? imsics_.at(hartIx).supervisorTopId() : 0; }

    /// For the given hart and guest indices (virtual guest associated
    /// with a hart), return the highest priority pending and enabled
    /// supervisor interrup id or 0 if no interrupt and
    /// enabled. Interrupt ids filtered out by the threshold mechanism
    /// are not considered.
    unsigned guestTopId(unsigned hartIx, unsigned guestIx) const
    { return hartIx < imsics_.size()? imsics_.at(hartIx).guestTopId(guestIx) : 0; }

  protected:

    /// Return true if given address is withing the address range of the
    /// machine privilege IMSIC files.
    bool isMachineAddr(uint64_t addr) const
    { return addr >= mbase_ and addr < mbase_ + harts_ * mstride_; }

    /// Return true if given address is withing the address range of the
    /// supervisor privilege IMSIC files (which includes the guest files).
    bool isSupervisorAddr(uint64_t addr) const
    { return addr >= sbase_ and addr < sbase_ + harts_ * sstride_; }

    /// Set ix to the index of the hart covering the 

  private:
    unsigned harts_ = 0;
    unsigned pageSize_ = 0;
    uint64_t mbase_ = 0;     // Base address of machine files.
    uint64_t mstride_ = 0;   // Memory space reserved for each machine file.
    uint64_t sbase_ = 0;     // Base address of supervisor files.
    uint64_t sstride_ = 0;   // Memory space reserved for each supervisor file.
    unsigned guests_ = 0;    // Guest count.

    std::vector<Imsic> imsics_;
  };
}
