#include <cmath>
#include <iomanip>
#include <iostream>
#include "VirtMem.hpp"

using namespace WdRiscv;


Tlb::Tlb(unsigned size)
{
  if ((size & (size - 1)) != 0)
    std::cerr << "TLB size must be a power of 2\n";
  else
    entries_.resize(size);
}


bool
Tlb::insertEntry(uint64_t virtPageNum, uint64_t physPageNum, uint32_t asid,
                 bool global, bool isUser, bool read, bool write, bool exec)
{
  auto* entry = getEntry(virtPageNum);

  if (not entry)
    return false;

  if (entry->valid_ and entry->counter_ & 2)
    {
      --entry->counter_;
      return false;
    }
  else
    {
      entry->valid_ = true;
      entry->virtPageNum_ = virtPageNum;
      entry->physPageNum_ = physPageNum;
      entry->counter_ = 0;
      entry->asid_ = asid;
      entry->global_ = global;
      entry->user_ = isUser;
      entry->read_ = read;
      entry->write_ = write;
      entry->exec_ = exec;
      return true;
    }
}


void
Tlb::printTlb(std::ostream& ost) const
{
  for (const auto&te: entries_)
    printEntry(ost, te);
}

void
Tlb::printEntry(std::ostream& ost, const TlbEntry& te)
{
  if (not te.valid_)
    return;
  ost << std::hex << std::setfill('0') << std::setw(10) << te.virtPageNum_ << std::dec << ",";
  if (te.global_)
    ost << "***";
  else
    ost << std::setfill('0') << std::setw(4) << te.asid_;
  ost << " -> " << std::setfill('0') << std::setw(10) << te.physPageNum_;
  ost << " P:" << (te.read_?"r":"-") << (te.write_?"w":"-") << (te.exec_?"x":"-");
  ost << " A:" << (te.accessed_ ? "a" : "-") << (te.dirty_? "d" : "-");
  ost << " S:" << (te.levels_==3 ? "1G" : te.levels_==2 ? "2M" : "4K") << "\n";
}


bool
Tlb::insertEntry(const TlbEntry& te)
{
  auto* entry = getEntry(te.virtPageNum_);

  if (not entry)
    return false;

  if (entry->valid_ and entry->counter_ & 2)
    {
      --entry->counter_;
      return false;
    }
  else
    {
      *entry = te;
      entry->counter_ = 0;
      return true;
    }
}
