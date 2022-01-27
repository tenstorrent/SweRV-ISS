#include <iostream>
#include "Mcm.hpp"
#include "System.hpp"
#include "Hart.hpp"

using namespace TTMcm;
using namespace WdRiscv;

template <typename URV>
Mcm<URV>::Mcm(System<URV>& system)
  : system_(system)
{
  memOps_.reserve(100000);
}


template <typename URV>
Mcm<URV>::~Mcm()
{
}


template <typename URV>
bool
Mcm<URV>::earlyRead(unsigned hartId, uint64_t time, uint64_t instrTag,
		    uint64_t physAddr, unsigned size, uint64_t rtlData,
		    bool internal)
{
  if (time < time_)
    {
      std::cerr << "Error: Mcm::earlyRead: Invalid memory operation time: "
		<< time << "(expecting value >= " << time_ << ")\n";
      return false;
    }

  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    {
      std::cerr << "Error: Mcm::earlyRead: Invalid hart id: " << hartId << '\n';
      return false;
    }

  unsigned hartIx = hartPtr->sysHartIndex();
  assert(hartIx < hartInstrVecs_.size());

  MemoryOp op;
  op.time_ = time;
  op.physAddr_ = physAddr;
  op.rtlData_ = rtlData;
  op.instrTag_ = instrTag;
  op.hartIx_ = hartIx;
  op.size_ = size;
  op.read_ = true;
  op.internal_ = internal;
  if (not internal)
    {
      if (size == 1)
	{
	  uint8_t val = 0;
	  op.failRead_ = not hartPtr->peekMemory(physAddr, size, val);
	  op.data_ = val;
	}
      else if (size == 2)
	{
	  uint16_t val = 0;
	  op.failRead_ = not hartPtr->peekMemory(physAddr, size, val);
	  op.data_ = val;
	}
      else if (size == 4)
	{
	  uint32_t val = 0;
	  op.failRead_ = not hartPtr->peekMemory(physAddr, size, val);
	  op.data_ = val;
	}
      else if (size == 8)
	{
	  uint64_t val = 0;
	  op.failRead_ = not hartPtr->peekMemory(physAddr, size, val);
	  op.data_ = val;
	}
      else
	{
	  op.failRead_ = true;
	  std::cerr << "Error: Mcm::earlyRead: Invalid read size: " << size
		    << '\n';
	  return false;
	}
    }

  memOps_.push_back(op);
  
  time_ = time;
  return true;
}


template class TTMcm::Mcm<uint32_t>;
template class TTMcm::Mcm<uint64_t>;
