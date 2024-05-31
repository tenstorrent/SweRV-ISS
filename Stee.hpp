#pragma once

#include <cstdint>


namespace TT_STEE      // TensTorrent Static Trusted Execution Environment.
{

  /// Model a static trusted execution environment controller.
  class Stee
  {
  public:

    /// Constructor.
    Stee() = default;

    /// Return true if given memory access is legal. If the secure world is disabled then
    /// all access is insecure. Otherwise, access with bit 55 clear is insecure.
    bool isValidAccess(uint64_t physAddr, unsigned size) const
    {
      if (not secWorld_  or steecfg_.bits_.sw1dis_)
	return false;   // Not in secure world.

      if ((physAddr & secMask_) != secMask_)
	return false;   // Address must all bits of secMask_ set.

      if ((physAddr & zmask_) != 0)
	return false;  // Zero-mask bits must be zero
      
      uint64_t addr = physAddr;
      addr &= ~secMask_;
      return addr + size - 1 < secLow_ or addr > secHigh_;
    }

    /// Return true if given address falls in the address range of the memory mapped
    /// registers associated with this Stee controller.
    bool hasAddress(uint64_t addr) const
    { return addr >= addr_ and addr < addr_ + size_; }

    /// Write to this Stee controller. Return true on success. Return false if address is
    /// not within range of this Stee or if address is not aligned or if size is not 8. If
    /// this controller is locked the write will have no effect (but we will return true).
    bool write(uint64_t addr, unsigned size, uint64_t value)
    {
      if (not hasAddress(addr) or size != 8 or (addr & 7) != 0)
	return false;
      if (not steecfg_.bits_.locked_)
	{
	  steecfg_.value_ = value;
	  steecfg_.bits_.res_ = 0;
	}
      return true;
    }

    /// Clear the bits corresponding to the secure-mask in the given address.
    uint64_t clearSecureBits(uint64_t addr) const
    { return addr & ~secMask_; }

    /// Read from this Stee controller. Return true on success setting value to the read
    /// value. Return false leaving value unmodified if address is not within range of
    /// this Stee or if address is not aligned or if size is not 8.
    bool read(uint64_t addr, unsigned size, uint64_t& value) const
    {
      if (not hasAddress(addr) or size != 8 or (addr & 7) != 0)
	return false;
      value = steecfg_.value_;
      return true;
    }

    /// Configure the zero mask mask: each one bit in the given mask corresponds to a bit
    /// that must be zero in the physical address for the access to be valid.
    void configZeroMask(uint64_t mask)
    { zmask_ = mask; }

    /// Configure the bit(s) used to indicate secure access: if any of the address bits,
    /// corresponding to the one-bits in mask, is non-zero then the access is considered
    /// secure.  A non-secure access will result in an access fault if it targets the
    /// secure region.
    void configSecureMask(uint64_t mask)
    { secMask_ = mask; }

    /// Return the mask of bits that must be zero in the physical address in order for the
    /// access to be considered valid by isValidAccess.
    uint64_t zeroMask() const
    { return zmask_; }

    /// Return the mask of bits that must be one in the physical address in order for the
    /// access to be considered valid by isValidAccess.
    uint64_t secureMask() const
    { return secMask_; }

    /// Configure the secure access region (only one region is currently supported).  An
    /// insecure access will fail if the target address falls in the secure region. The
    /// region is between addresses low to high inclusive. To define an empty region pass
    /// a high value that is less than low.
    void configSecureRegion(uint64_t low, uint64_t high)
    { secLow_ = low; secHigh_ = high; }

    /// Configure the address of the memory-mapped register associated with this STEE.
    bool configAddress(uint64_t addr)
    {
      if ((addr % 8) != 0)
	return false;
      addr_ = addr;
      return true;
    }

    /// Set the secure world index. An index of zero implies a non-zecure world.
    void setSecureWorld(unsigned world)
    { secWorld_ = world; }

  private:

    struct SteecfgBits
    {
      unsigned sw1dis_   : 1;
      uint64_t res_      : 62;
      unsigned locked_   : 1;
    };

    uint64_t addr_ = 0x0;   // Address of memory mapped region of this stee.
    uint64_t size_ = 8;  // Size of memory mapped region of this stee.
    uint64_t zmask_ = uint64_t(7) << 52;  // Bits 52, 53, and 54.
    uint64_t secMask_ = uint64_t(1) << 55;  // Bit 55.

    uint64_t secLow_ = ~uint64_t(0);
    uint64_t secHigh_ = 0;

    unsigned secWorld_ = 0;    // Non-zero for a secure world.

    union
    {
      uint64_t value_ = 0;
      SteecfgBits bits_;
    } steecfg_;
  };

}
