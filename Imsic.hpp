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
    { return (addr & 3) == 0 and isCoveredAddress(addr); }

    /// Return true if given address is in the rage of addresses
    /// covered by this file. Return false otherwise. Return false
    /// if this file is not active.
    bool isCoveredAddress(uint64_t addr) const
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

  private:

    uint64_t addr_ = 0;
    std::vector<bool> pending_;
    std::vector<bool> enabled_;
    unsigned topId_ = 0;
    unsigned threshold_ = 0;
    bool active_ = false;
    const unsigned pageSize_ = 1024;
  };


  class Imsic
  {
  public:

    /// Define an inactive IMSIC (no machine/supervisor/guest file,
    /// does not cover any address).
    Imsic()
      : mfile_(0, 0), sfile_(0, 0)
    { }

    /// Acivate the machine privilege file at the given address and
    /// with the given size (size is the highest interrupt id plus 1
    /// and must be a multiple of 64).  All pevious state
    /// enabled/pending state is lost.
    void activateMachine(uint64_t addr, unsigned size)
    { mfile_.activate(addr, size); }

    /// Acivate the supervisor privilege file at the given address and
    /// with the given size (size is the highest interrupt id plus 1
    /// and must be a multiple of 64).  All pevious state
    /// enabled/pending state is lost.
    void activateSupervisor(uint64_t addr, unsigned size)
    { sfile_.activate(addr, size); }

    /// Activate g guest files of n-1 interrupt ids each. The guest
    /// addresses will be s+p, s+2p, s+3p, ... where s is the
    /// supervisor file address and p is the page size. A supervisor
    /// file must be activated before the guests files are
    /// activated. The parameter g must not exceede 64.
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
    /// not valid. Return true and perform write if address is valid.
    /// Write may be a no-op if written data corresponds to a
    /// non-implemented id.
    bool write(uint64_t addr, uint64_t data, unsigned size);

  private:

    File mfile_;
    File sfile_;
    std::vector<File> gfiles_;
  };
}
