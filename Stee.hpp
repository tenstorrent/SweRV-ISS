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

    /// Return true if given memory access is legal: bits of the given address must be
    /// zero if they correspond to the bits set in the zero-mask.
    bool isValidAddress(uint64_t physAddr) const
    { return (physAddr & zmask_) == 0; }

    /// Return true if given memory address is insecure: either not running in
    /// secure world, or running in secure world and the bits of the given
    /// address corresponding to the secure mask are not all set.
    bool isInsecureAddress(uint64_t physAddr) const
    { return secWorld_ == 0 or (physAddr & secMask_) != secMask_; }

    /// Return true if given memory access is insecure: The address is
    /// insecure and it overlaps a secure memory region.
    bool isInsecureAccess(uint64_t physAddr) const
    {
      if (not isInsecureAddress(physAddr))
	return false;  // Secure address.
      uint64_t effAddr = clearSecureBits(physAddr);
      return effAddr >= secLow_ and effAddr < secHigh_;
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
    /// access to be considered valid by isValidAddress.
    uint64_t zeroMask() const
    { return zmask_; }

    /// Return the mask of bits that must be one in the physical address in order for the
    /// address to be considered secure.
    uint64_t secureMask() const
    { return secMask_; }

    /// Configure the secure access region (only one region is currently supported).  An
    /// insecure access will be ignored if the target address falls in the secure
    /// region. The region is between addresses low to high excluding high. To define an
    /// empty region use same value for low and high.
    void configSecureRegion(uint64_t low, uint64_t high)
    {
      secLow_ = low;
      secHigh_ = high;
    }

    /// Set the secure world index. An index of zero implies a non-zecure world.
    void setSecureWorld(unsigned world)
    { secWorld_ = world; }

    /// Return current secure world id.
    unsigned secureWorld() const
    { return secWorld_; }

  private:

    uint64_t zmask_ = uint64_t(7) << 52;  // Bits 52, 53, and 54.
    uint64_t secMask_ = uint64_t(1) << 55;  // Bit 55.

    uint64_t secLow_ = ~uint64_t(0);
    uint64_t secHigh_ = 0;

    unsigned secWorld_ = 0;    // Non-zero for a secure world.
  };

}
