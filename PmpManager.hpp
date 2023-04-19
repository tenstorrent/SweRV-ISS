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
#include <vector>
#include <string>
#include <unordered_map>
#include "CsRegs.hpp"

namespace WdRiscv
{

  /// Physical memory protection. An instance of this is usually
  /// associated with a section of the address space. The address
  /// space is evenly divided into contiguous, equally sized sections,
  /// aligned to the section size.
  /// For sub-section protection, an instance is associated with a
  /// word-aligned memory word. To reduce footprint of the PmaMgr
  /// object, we typically use a section size of 8 or more pages.
  class Pmp
  {
  public:

    friend class PmpManager;

    enum Type { Off = 0, Tor = 1, Na4 = 2, Napot = 3, _Count = 4 };

    enum Mode
      {
       None = 0, Read = 1, Write = 2, Exec = 4, ReadWrite = Read | Write,
       Default = Read | Write | Exec
      };

    /// Default constructor: No access allowed.
    Pmp(Mode m = None, unsigned pmpIx = 0, bool locked = false,
        Type type = Type::Off)
      : mode_(m), type_(type), locked_(locked), pmpIx_(pmpIx)
    { }

    /// Return true if read (i.e. load instructions) access allowed
    bool isRead(PrivilegeMode mode, PrivilegeMode prevMode, bool mprv) const
    {
      if (mprv)
        mode = prevMode;
      bool check = (mode != PrivilegeMode::Machine) or locked_;
      return check ? mode_ & Read : true;
    }

    /// Return true if write (i.e. store instructions) access allowed.
    bool isWrite(PrivilegeMode mode, PrivilegeMode prevMode, bool mprv) const
    {
      bool check = (mode != PrivilegeMode::Machine or locked_ or
                    (mprv and prevMode != PrivilegeMode::Machine));
      return check ? mode_ & Write : true;
    }

    /// Return true if instruction fecth is allowed.
    bool isExec(PrivilegeMode mode, PrivilegeMode prevMode, bool mprv) const
    {
      bool check = (mode != PrivilegeMode::Machine or locked_ or
                    (mprv and prevMode != PrivilegeMode::Machine));
      return check ? mode_ & Exec : true;
    }

    /// Return true if this object has the mode attributes as the
    /// given object.
    bool operator== (const Pmp& other) const
    { return mode_ == other.mode_ and pmpIx_ == other.pmpIx_; }

    /// Return true if this object has different attributes from those
    /// of the given object.
    bool operator!= (const Pmp& other) const
    { return mode_ != other.pmpIx_ or pmpIx_ != other.pmpIx_; }

    /// Return string representation of the given PMP type.
    static std::string toString(Type t);

    /// Return string representation of the given PMP mode.
    static std::string toString(Mode m);

    /// Return integer representation of the given PMP configuration.
    uint8_t val() const
    { return (locked_ << 7) | (0 << 5) | ((uint8_t(type_) & 3) << 3) | (mode_ & 7); }

  protected:

    /// Return the index of the PMP entry from which this object was
    /// created.
    unsigned pmpIndex() const
    { return pmpIx_; }

  private:

    uint8_t mode_ = 0;
    Type type_      : 8;
    bool locked_    : 1;
    unsigned pmpIx_ : 5;  // Index of corresponding pmp register.
  } __attribute__((packed));


  /// Physical memory protection manager. One per hart.  rotection
  /// applies to word-aligned regions as small as 1 word but are
  /// expected to be applied to a small number (less than or equal 16)
  /// of large regions.
  class PmpManager
  {
  public:

    friend class Memory;

    /// Constructor. Mark all memory as no access to user/supervisor
    /// (machine mode does access since it is not checked).
    PmpManager(uint64_t memorySize, uint64_t sectionSize = 32*1024);

    /// Destructor.
    ~PmpManager();

    /// Reset: Mark all memory as no access to user/supervisor
    /// (machine mode does have access because it is not checked).
    void reset();

    /// Return the physical memory protection object (pmp) associated
    /// with the word-aligned word designated by the given
    /// address. Return a no-access object if the given address is out
    /// of memory range.
    Pmp getPmp(uint64_t addr) const
    {
      addr = (addr >> 2) << 2;
      for (auto& region : regions_)
	if (addr >= region.firstAddr_ and addr <= region.lastAddr_)
	  return region.pmp_;
      return Pmp();
    }

    /// Return the physical memory protection object (pmp) associated
    /// with a given index. Return a no-access object if the given index
    /// is out of range.
    Pmp peekPmp(size_t ix) const
    {
      for (auto& region : regions_)
        {
          auto pmp = region.pmp_;
          if (pmp.pmpIndex() == ix)
            return pmp;
        }
      return Pmp();
    }

    /// Similar to getPmp but it also updates the access count associated with
    /// each PMP entry.
    inline Pmp accessPmp(uint64_t addr) const
    {
      addr = (addr >> 2) << 2;
      for (auto& region : regions_)
	if (addr >= region.firstAddr_ and addr <= region.lastAddr_)
	  {
	    auto pmp = region.pmp_;
	    auto ix = pmp.pmpIndex();
	    if (trace_)
	      {
		accessCount_.at(ix)++;
		typeCount_.at(ix)++;
		pmpTrace_.push_back(ix);
	      }
	    return pmp;
	  }
      return Pmp();
    }

    /// Enable/disable physical memory protection.
    void enable(bool flag)
    { enabled_ = flag; }

    /// Return true if physical memory protection is enabled.
    bool isEnabled() const
    { return enabled_; }

    /// Set access mode of word-aligned words overlapping given region
    /// for user/supervisor.
    void defineRegion(uint64_t addr0, uint64_t addr1, Pmp::Type type,
		      Pmp::Mode mode, unsigned pmpIx, bool locked);

    /// Print statistics on the given stream.
    bool printStats(std::ostream& out) const;

    /// Print statistics on the given file.
    bool printStats(const std::string& path) const;

    /// Return the access count of PMPs used in most recent instruction.
    const std::vector<uint64_t>& getPmpTrace() const
    { return pmpTrace_; }

    void clearPmpTrace()
    { pmpTrace_.clear(); }

    /// Collect stats if flag is true.
    void enableTrace(bool flag)
    { trace_ = flag; }

  private:

    struct Region
    {
      uint64_t firstAddr_ = 0;
      uint64_t lastAddr_ = 0;
      Pmp pmp_;
    };

    std::vector<Region> regions_;
    bool enabled_ = false;
    bool trace_ = true;   // Collect stats if true.
    mutable std::vector<uint64_t> accessCount_;  // PMP entry access count.
    mutable std::vector<uint64_t> typeCount_;  // PMP type access count.

    // PMPs used in most recent instruction
    mutable std::vector<uint64_t> pmpTrace_;
  };
}
