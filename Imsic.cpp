#include "util.hpp"
#include "Imsic.hpp"

using namespace TT_IMSIC;

bool
Imsic::write(uint64_t addr,  unsigned size, uint64_t data)
{
  if (size != 4)
    return false;

  uint32_t word = data;

  File* file = nullptr;

  if (mfile_.isCoveredAddress(addr))
    file = &mfile_;
  else if (sfile_.isCoveredAddress(addr))
    file = &sfile_;
  else
    for (auto& gfile : gfiles_)
      if (gfile.isCoveredAddress(addr))
	file = &gfile;

  if (not file)
    return false;  // Address is not covered by this imsic

  if (not file->isValidAddress(addr))
    return false;

  if (addr == file->address())
    file->setPending(word, true);
  else if (addr == file->address() + 4)
    {
      word = util::byteswap(word);
      file->setPending(word, true);
    }

  return true;
}

