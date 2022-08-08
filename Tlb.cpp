#include <cmath>
#include <iomanip>
#include "VirtMem.hpp"

using namespace WdRiscv;


Tlb::Tlb(unsigned size)
  : entries_(size)
{
}


bool
Tlb::insertEntry(uint64_t virtPageNum, uint64_t physPageNum, uint32_t asid,
                 bool global, bool isUser, bool read, bool write, bool exec)
{
  TlbEntry* best = nullptr;
  for (size_t i = 0; i < entries_.size(); ++i)
    {
      auto& entry = entries_[i];
      if (not entry.valid_ or (best and entry.time_ < best->time_))
        best = &entry;
    }
  if (best)
    {
      best->valid_ = true;
      best->virtPageNum_ = virtPageNum;
      best->physPageNum_ = physPageNum;
      best->time_ = time_++;
      best->asid_ = asid;
      best->global_ = global;
      best->user_ = isUser;
      best->read_ = read;
      best->write_ = write;
      best->exec_ = exec;
      return true;
    }
  return false;
}


void
Tlb::printTlb(std::ostream& ost) const
{
  for (auto&te: entries_)
    printEntry(ost, te);
}

void
Tlb::printEntry(std::ostream& ost, const TlbEntry& te) const
{
  if (not te.valid_)
    return;
  ost << std::hex << std::setfill('0') << std::setw(10) << te.virtPageNum_ << ",";
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
  TlbEntry* best = nullptr;
  for (size_t i = 0; i < entries_.size(); ++i)
    {
      auto& entry = entries_[i];
      if (not entry.valid_)
        {
          best = &entry;
          break;
        }
      if (not best or entry.time_ < best->time_)
        best = &entry;
    }

  if (best)
    {
      *best = te;
      best->time_ = time_++;
      return true;
    }
  return false;
}
