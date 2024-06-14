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

    /// Return true if given memory access is legal. If the secure world is enabled then
    /// all accesses are legal. Otherwise, access to secure region is invalid.  In both
    /// secure/normal world the zero-bits of the address must be zero.
    bool isValidAccess(uint64_t physAddr, unsigned size) const
    {
      if ((physAddr & zmask_) != 0)
	return false;  // Zero-mask bits must be zero

      if (secWorld_)
	return true;   // In secure world.

      physAddr = clearSecureBits(physAddr);

      // Address must not target secure area.
      return physAddr + size - 1 < secLow_ or physAddr > secHigh_;
    }

    /// Clear the bits corresponding to the secure-mask in the given address.
    uint64_t clearSecureBits(uint64_t addr) const
    { return addr & ~secMask_; }

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

    /// Set the secure world index. An index of zero implies a non-zecure world.
    void setSecureWorld(unsigned world)
    { secWorld_ = world; }

  private:

    uint64_t zmask_ = uint64_t(7) << 52;  // Bits 52, 53, and 54.
    uint64_t secMask_ = uint64_t(1) << 55;  // Bit 55.

    uint64_t secLow_ = ~uint64_t(0);
    uint64_t secHigh_ = 0;

    unsigned secWorld_ = 0;    // Non-zero for a secure world.
  };

}
