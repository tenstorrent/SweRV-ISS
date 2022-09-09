// Copyright 2020 Western Digital Corporation or its affiliates.
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include "Hart.hpp"
#include "Core.hpp"
#include "System.hpp"
#include "Mcm.hpp"
#include "Uart8250.hpp"

using namespace WdRiscv;


inline bool
isPowerOf2(uint64_t x)
{
  return x != 0 and (x & (x-1)) == 0;
}


template <typename URV>
System<URV>::System(unsigned coreCount, unsigned hartsPerCore,
                    unsigned hartIdOffset, size_t memSize,
                    size_t pageSize)
  : hartCount_(coreCount * hartsPerCore), hartsPerCore_(hartsPerCore)
{
  cores_.resize(coreCount);

  memory_ = std::make_shared<Memory>(memSize, pageSize);
  sparseMem_ = nullptr;

  Memory& mem = *(memory_.get());
  mem.setHartCount(hartCount_);

  for (unsigned ix = 0; ix < coreCount; ++ix)
    {
      URV coreHartId = ix * hartIdOffset;
      cores_.at(ix) = std::make_shared<CoreClass>(coreHartId, ix, hartsPerCore, mem);

      // Maintain a vector of all the harts in the system.  Map hart-id to index
      // of hart in system.
      auto core = cores_.at(ix);
      for (unsigned i = 0; i < hartsPerCore; ++i)
        {
          auto hart = core->ithHart(i);
          sysHarts_.push_back(hart);
          URV hartId = coreHartId + i;
          unsigned hartIx = ix*hartsPerCore + i;
          hartIdToIndex_[hartId] = hartIx;
        }
    }

#ifdef MEM_CALLBACKS
  sparseMem_ = new SparseMem();
  auto readf = [this](uint64_t addr, unsigned size, uint64_t& value) -> bool {
                 return sparseMem_->read(addr, size, value); };
  auto writef = [this](uint64_t addr, unsigned size, uint64_t value) -> bool {
                  return sparseMem_->write(addr, size, value); };

  mem.defineReadMemoryCallback(readf);
  mem.defineWriteMemoryCallback(writef);
#endif

  // mem.registerIoDevice(new Uart8250(0x1000000, 0x100));
}


template <typename URV>
System<URV>::~System()
{
  delete sparseMem_;
  sparseMem_ = nullptr;

  delete mcm_;
  mcm_ = nullptr;
}


template <typename URV>
void
System<URV>::checkUnmappedElf(bool flag)
{
  if (memory_)
    memory_->checkUnmappedElf(flag);
}


template <typename URV>
bool
System<URV>::writeAccessedMemory(const std::string& path) const
{
  if (not sparseMem_)
    return false;
  return sparseMem_->writeHexFile(path);
}


template <typename URV>
bool
System<URV>::enableMcm(unsigned mbLineSize, bool mbLineCheckAll)
{
  if (mcm_)
    {
      assert(mcm_->mergeBufferLineSize() == mbLineSize);
      std::cerr << "System::enableMcm: Already enabled\n";
      return true;
    }

  if (mbLineSize != 0)
    if (not isPowerOf2(mbLineSize) or mbLineSize > 512)
      {
	std::cerr << "Error: Invalid merge buffer line size: "
		  << mbLineSize << '\n';
	return false;
      }

  mcm_ = new Mcm<URV>(this->hartCount(), mbLineSize);
  mbSize_ = mbLineSize;
  mcm_->setCheckWholeMbLine(mbLineCheckAll);

  for (auto hart :  sysHarts_)
    hart->setMcm(mcm_);

  return true;
}


template <typename URV>
bool
System<URV>::mcmRead(Hart<URV>& hart, uint64_t time, uint64_t tag,
		     uint64_t addr, unsigned size, uint64_t data,
		     bool internal)
{
  if (not mcm_)
    return false;
  return mcm_->readOp(hart, time, tag, addr, size, data, internal);
}


template <typename URV>
bool
System<URV>::mcmMbWrite(Hart<URV>& hart, uint64_t time, uint64_t addr,
			const std::vector<uint8_t>& data,
			const std::vector<bool>& mask)
{
  if (not mcm_)
    return false;
  return mcm_->mergeBufferWrite(hart, time, addr, data, mask);
}


template <typename URV>
bool
System<URV>::mcmMbInsert(Hart<URV>& hart, uint64_t time, uint64_t tag,
			 uint64_t addr, unsigned size, uint64_t data)
{
  if (not mcm_)
    return false;
  return mcm_->mergeBufferInsert(hart, time, tag, addr, size, data);
}


template <typename URV>
bool
System<URV>::mcmRetire(Hart<URV>& hart, uint64_t time, uint64_t tag,
		       const DecodedInst& di)
{
  if (not mcm_)
    return false;
  return mcm_->retire(hart, time, tag, di);
}

template <typename URV>
bool
System<URV>::mcmSetCurrentInstruction(Hart<URV>& hart, uint64_t tag)
{
  if (not mcm_)
    return false;
  return mcm_->setCurrentInstruction(hart, tag);
}


template class WdRiscv::System<uint32_t>;
template class WdRiscv::System<uint64_t>;
