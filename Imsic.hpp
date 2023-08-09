#pragma once

#include <vector>
#include <cstdint>
#include <cassert>


namespace TT_IMSIC      // TensTorrent Incoming Message Signaled Interrupt Controller.
{

  /// Model an IMISIC file: One enable bit and one pending bit for
  /// each interrupt identity.  For a file of size n, interrupt id 0
  /// is reserved, and valid ids are 1 to n-1. Priorities are
  /// associated with ids with the lowest id having the highest
  /// priority (per spec).
  class File
  {
  public:

    /// Define a file with n-1 interrupt identitities: 1 to
    /// n-1. Identity 0 is reserved. Initial value: All disabled none
    /// pending. The n parameter must be a multiple of 64.
    File (uint64_t address, unsigned n)
      : addr_(address), pending_(n), enabled_(n), topId_(0), active_(true)
    {
      assert((n & 0x3f) == 0);
    }

    /// Reutrn the address of this IMSIC>
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

    /// Mark the given interrupt id as pending/not-pending. Setting id 0
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

    /// Mark the given interrupt id as enabled/not-pending. Setting id 0
    /// or an out of bounds id has no effect.  
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

    /// Set the top interrutp id threshold.
    void setThreshold(unsigned t)
    { threshold_ = t; } 

    /// Return the top interrupt id threshold: Ids larger than or
    /// equal to the threshold do not participate in the computation
    /// of top id even if pending and enabled.
    unsigned threshold() const
    { return threshold_; }

    /// Return the id of the highest priority (smallest id) interrutp
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

    /// Mark this imsic as active/inactive. Harts that do not
    /// implement a supervisor would mark the supervisode file
    /// inactive.
    void setActive(bool flag)
    {
      active_ = flag;
      updateTopId();
    }

    /// Return true iff this imsic is active.
    bool isAtvive() const
    { return active_; }

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

    /// Define an imsic with the given machine privilege address and
    /// size (size the the highest interrupt id plus 1 and must be a
    /// multiple of 64).
    Imsic(uint64_t maddr, unsigned msize)
      : mfile_(maddr, msize), sfile_(0, 0)
    {
      sfile_.setActive(false);
    }

    /// Define an imsic with the given machine/supervisor privilege
    /// address and size (size the the highest interrupt id plus 1 and
    /// must be a multiple of 64).
    Imsic(uint64_t maddr, unsigned msize, uint64_t saddr, unsigned ssize)
      : mfile_(maddr, msize), sfile_(saddr, ssize)
    {
      sfile_.setActive(false);
    }

    /// Return the physical address for machine privilege file in this
    /// IMSIC. This is also the address for little endian access.
    uint64_t machineAddress() const
    { return mfile_.address(); }

    /// Return the physical address of this IMSIC to be used for
    /// writing in big-endian mode for machine privilege file.
    uint64_t bigEndianMachineAddress() const
    { return mfile_.address() + 4; }

    /// Return the physical address for supervisor privilege file in this
    /// IMSIC. This is also the address for little endian access.
    uint64_t supervisorAddress() const
    { return sfile_.address(); }

    /// Return the physical address of this IMSIC to be used for
    /// writing in big-endian mode for supervisor privilege file.
    uint64_t bigEndianSupervisorAddress() const
    { return sfile_.address() + 4; }

    /// Return true if given address is valid for this imsic:
    /// Within the machine/supervisor/guest pages associated with
    /// this imsic and word aligned.
    bool isValidAddress(uint64_t addr) const
    {
      if (mfile_.isValidAddress(addr) or sfile_.isValidAddress(addr))
	return true;
      for (const auto& gfile : gfiles_)
	if (gfile.isValidAddress(addr))
	  return true;
      return false;
    }

    /// Write to this imsic. Return false doing nothing if adress is
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
