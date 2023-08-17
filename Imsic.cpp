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
      const std::vector<bool>* it;
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

  if (mfile_.coversAddress(addr))
    file = &mfile_;
  else if (sfile_.coversAddress(addr))
    file = &sfile_;
  else
    for (auto& gfile : gfiles_)
      if (gfile.coversAddress(addr))
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


bool
ImsicMgr::configMachine(uint64_t addr, uint64_t stride, unsigned ids)
{
  if ((addr % pageSize_) != 0 or (stride % pageSize_) != 0 or stride == 0)
    return false;

  if (ids == 0 or (ids % 64) != 0)
    return false;

  mbase_ = addr;
  mstride_ = stride;
  for (auto& imsic : imsics_)
    {
      imsic.activateMachine(addr, ids);
      addr += stride;
    }
  return true;
}


bool
ImsicMgr::configSupervisor(uint64_t addr, uint64_t stride, unsigned ids)
{
  if ((addr % pageSize_) != 0 or (stride % pageSize_) != 0 or stride == 0)
    return false;

  if (ids == 0 or (ids % 64) != 0)
    return false;

  sbase_ = addr;
  sstride_ = stride;
  for (auto& imsic : imsics_)
    {
      imsic.activateSupervisor(addr, ids);
      addr += stride;
    }
  return true;
}


bool
ImsicMgr::configGuests(unsigned n, unsigned ids)
{
  if (sstride_ < (n+1) * pageSize_)
    return false;  // No enough space.

  for (auto& imsic : imsics_)
    imsic.activateGuests(n, ids);

  return true;
}
