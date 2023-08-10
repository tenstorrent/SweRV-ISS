#include "util.hpp"
#include "Imsic.hpp"

using namespace TT_IMSIC;


template <typename URV>
bool
File::read(URV& val) const
{
  using EIC = ExternalInterruptCsr;

  if (select_ == EIC::DELIVERY)
    val = delivery_;
  else if (select_ == EIC::THRESHOLD)
    val = threshold_;
  else
    {
      val = 0;
      unsigned offset;
      std::vector<bool>* it;
      if (select_ >= EIC::P0 and select_ <= EIC::P63)
        {
          offset = select_ - EIC::P0;
          it = &pending_;
        }
      else if (select_ >= EIC::E0 and select_ <= EIC::E63)
        {
          offset = select_ - EIC::E0;
          it = &enabled_;
        }
      else
        return true;

      if constexpr (sizeof(URV) == 8)
        if (offset & 1)
          return false;

      constexpr size_t bits = 8*sizeof(URV);
      const auto& arr = *it;
      unsigned begin = offset*bits;
      unsigned end = std::min((offset + 1)*bits, arr.size());
      // slow, use bitset?
      for (unsigned i = begin; i < end; i++)
        val |= arr.at(i) << (i - begin);
    }

  return true;
}


template <typename URV>
bool
File::write(URV val)
{
  using EIC = ExternalInterruptCsr;

  if (select_ == EIC::DELIVERY)
    delivery_ = val;
  else if (select_ == EIC::THRESHOLD)
    threshold_ = val;
  else
    {
      val = 0;
      unsigned offset;
      std::vector<bool>* it;
      if (select_ >= EIC::P0 and select_ <= EIC::P63)
        {
          offset = select_ - EIC::P0;
          it = &pending_;
        }
      else if (select_ >= EIC::E0 and select_ <= EIC::E63)
        {
          offset = select_ - EIC::E0;
          it = &enabled_;
        }
      else
        return true;

      if constexpr (sizeof(URV) == 8)
        if (offset & 1)
          return false;

      constexpr size_t bits = 8*sizeof(URV);
      auto& arr = *it;
      unsigned begin = offset*bits;
      unsigned end = std::min((offset + 1)*bits, arr.size());
      // slow, use bitset?
      for (unsigned i = begin; i < end; i++)
        arr[i] = (val >> (i - begin)) & 1;
    }

  return true;
}

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
