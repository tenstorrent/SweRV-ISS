#include <stdexcept>
#include "util.hpp"
#include "Imsic.hpp"

using namespace TT_IMSIC;


template <typename URV>
bool
File::iregRead(URV& val) const
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
File::iregWrite(URV val)
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
  bool isGuest = false;
  unsigned guestIx = 0;

  if (mfile_.coversAddress(addr))
    file = &mfile_;
  else if (sfile_.coversAddress(addr))
    file = &sfile_;
  else
    for (size_t i = 0; i < gfiles_.size(); ++i)
      if (gfiles_.at(i).coversAddress(addr))
	{
	  file = &gfiles_.at(i);
	  guestIx = i;
	  isGuest = true;
	}

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

  if (isGuest and file->canDeliver() and file->topId())
    guestInterrupts_ |= uint64_t(1) << guestIx;

  return true;
}


ImsicMgr::ImsicMgr(unsigned hartCount, unsigned pageSize)
  : pageSize_(pageSize), imsics_(hartCount)
{
  if (pageSize_ == 0)
    throw std::runtime_error("Zero page size in ImsciMgr constructor.");

  for (unsigned ix = 0; ix < hartCount; ++ix)
    imsics_.at(ix) = std::make_shared<Imsic>();
}


bool
ImsicMgr::configureMachine(uint64_t addr, uint64_t stride, unsigned ids)
{
  if ((addr % pageSize_) != 0 or (stride % pageSize_) != 0 or stride == 0)
    return false;

  if (ids == 0 or (ids % 64) != 0)
    return false;

  mbase_ = addr;
  mstride_ = stride;
  for (auto imsic : imsics_)
    {
      imsic->configureMachine(addr, ids);
      addr += stride;
    }
  return true;
}


bool
ImsicMgr::configureSupervisor(uint64_t addr, uint64_t stride, unsigned ids)
{
  if ((addr % pageSize_) != 0 or (stride % pageSize_) != 0 or stride == 0)
    return false;

  if (ids == 0 or (ids % 64) != 0)
    return false;

  sbase_ = addr;
  sstride_ = stride;
  for (auto imsic : imsics_)
    {
      imsic->configureSupervisor(addr, ids);
      addr += stride;
    }
  return true;
}


bool
ImsicMgr::configureGuests(unsigned n, unsigned ids)
{
  if (sstride_ < (n+1) * pageSize_)
    return false;  // No enough space.

  for (auto imsic : imsics_)
    imsic->configureGuests(n, ids);

  return true;
}
