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

#include <climits>
#include <mutex>
#include <atomic>
#include <cassert>

#include "instforms.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"
#include "Mcm.hpp"

using namespace WdRiscv;


template <typename URV>
ExceptionCause
Hart<URV>::validateAmoAddr(uint64_t& addr, unsigned accessSize)
{
  URV mask = URV(accessSize) - 1;

  uint64_t addr2 = addr;
  auto cause = determineStoreException(addr, addr2, accessSize);

  if (cause == ExceptionCause::STORE_ADDR_MISAL)
    {
      if (misalAtomicCauseAccessFault_)
	return ExceptionCause::STORE_ACC_FAULT;
      return cause;
    }

  // Address must be word aligned for word access and double-word
  // aligned for double-word access.
  if ((addr & mask) != 0)
    return ExceptionCause::STORE_ACC_FAULT;

  if (cause != ExceptionCause::NONE)
    return cause;

  Pma pma = memory_.pmaMgr_.getPma(addr);
  if (not pma.isAmo())
    return ExceptionCause::STORE_ACC_FAULT;

  return cause;
}


template <typename URV>
bool
Hart<URV>::amoLoad32(uint32_t rs1, URV& value)
{
  URV virtAddr = intRegs_.read(rs1);

  ldStAddr_ = virtAddr;   // For reporting load addr in trace-mode.
  ldStPhysAddr1_ = ldStAddr_;
  ldStPhysAddr2_ = ldStAddr_;
  ldStSize_ = 4;
  ldStAtomic_ = true;

  if (hasActiveTrigger())
    {
      if (ldStAddrTriggerHit(virtAddr, TriggerTiming::Before, false /*isLoad*/))
	triggerTripped_ = true;
    }

  uint64_t addr = virtAddr;
  auto cause = validateAmoAddr(addr, ldStSize_);
  ldStPhysAddr1_ = addr;
  ldStPhysAddr2_ = addr;

  if (cause != ExceptionCause::NONE)
    {
      if (not triggerTripped_)
        initiateLoadException(cause, addr);
      return false;
    }

  uint32_t uval = 0;

  bool hasMcmVal = false;
  if (mcm_)
    {
      uint64_t mcmVal = 0;
      if (mcm_->getCurrentLoadValue(*this, addr, ldStSize_, mcmVal))
	{
	  uval = mcmVal;
	  hasMcmVal = true;
	}
    }

  if (not hasMcmVal and not memory_.read(addr, uval))
    {
      assert(0);
      return false;
    }

  value = SRV(int32_t(uval)); // Sign extend.
  return true;  // Success.
}


template <typename URV>
bool
Hart<URV>::amoLoad64(uint32_t rs1, URV& value)
{
  URV virtAddr = intRegs_.read(rs1);

  ldStAddr_ = virtAddr;   // For reporting load addr in trace-mode.
  ldStPhysAddr1_ = ldStAddr_;
  ldStPhysAddr2_ = ldStAddr_;
  ldStSize_ = 8;
  ldStAtomic_ = true;

  if (hasActiveTrigger())
    {
      if (ldStAddrTriggerHit(virtAddr, TriggerTiming::Before, false /*isLoad*/))
	triggerTripped_ = true;
    }

  uint64_t addr = virtAddr;
  auto cause = validateAmoAddr(addr, ldStSize_);
  ldStPhysAddr1_ = addr;
  ldStPhysAddr2_ = addr;

  if (cause != ExceptionCause::NONE)
    {
      if (not triggerTripped_)
        initiateLoadException(cause, addr);
      return false;
    }

  uint64_t uval = 0;
  bool hasMcm = mcm_ and mcm_->getCurrentLoadValue(*this, addr, ldStSize_, uval);

  if (not hasMcm and not memory_.read(addr, uval))
    {
      assert(0);
      return false;
    }

  value = uval;
  return true;  // Success.
}


template <typename URV>
template <typename LOAD_TYPE>
bool
Hart<URV>::loadReserve(uint32_t rd, uint32_t rs1)
{
  URV virtAddr = intRegs_.read(rs1);

  ldStAddr_ = virtAddr;   // For reporting load addr in trace-mode.
  ldStPhysAddr1_ = ldStAddr_;
  ldStPhysAddr2_ = ldStAddr_;
  ldStSize_ = sizeof(LOAD_TYPE);
  ldStAtomic_ = true;

  if (hasActiveTrigger())
    {
      bool isLd = true;
      if (ldStAddrTriggerHit(virtAddr, TriggerTiming::Before, isLd))
	triggerTripped_ = true;
      if (triggerTripped_)
	return false;
    }

  // Unsigned version of LOAD_TYPE
  typedef typename std::make_unsigned<LOAD_TYPE>::type ULT;

  uint64_t addr1 = virtAddr, addr2 = virtAddr;
  auto cause = determineLoadException(addr1, addr2, ldStSize_, false /*hyper*/);
  if (cause == ExceptionCause::LOAD_ADDR_MISAL and misalAtomicCauseAccessFault_)
    cause = ExceptionCause::LOAD_ACC_FAULT;

  ldStPhysAddr1_ = addr1;
  ldStPhysAddr1_ = addr2;

  // Address outside DCCM causes an exception (this is swerv specific).
  bool fail = amoInDccmOnly_ and not isAddrInDccm(addr1);

  // Access must be naturally aligned.
  if ((addr1 & (ldStSize_ - 1)) != 0)
    fail = true;

  if (cause == ExceptionCause::NONE)
    fail = fail or not memory_.pmaMgr_.getPma(addr1).isRsrv();

  if (fail and cause == ExceptionCause::NONE)
    cause = ExceptionCause::LOAD_ACC_FAULT;

  if (cause != ExceptionCause::NONE)
    {
      initiateLoadException(cause, virtAddr);
      return false;
    }

  ULT uval = 0;
  bool hasMcmVal = false;
  if (mcm_)
    {
      uint64_t mcmVal = 0;
      if (mcm_->getCurrentLoadValue(*this, addr1, ldStSize_, mcmVal))
	{
	  uval = mcmVal;
	  hasMcmVal = true;
	}
    }

  if (not hasMcmVal and not memory_.read(addr1, uval))
    {
      assert(0);
      return false;
    }      

  URV value = uval;
  if (not std::is_same<ULT, LOAD_TYPE>::value)
    value = SRV(LOAD_TYPE(uval)); // Sign extend.

  intRegs_.write(rd, value);
  return true;
}


template <typename URV>
void
Hart<URV>::execLr_w(const DecodedInst* di)
{
  if (not isRva())
    {
      illegalInst(di);
      return;
    }

  std::lock_guard<std::mutex> lock(memory_.lrMutex_);

  lrCount_++;
  if (not loadReserve<int32_t>(di->op0(), di->op1()))
    return;

  unsigned size = 4;
  uint64_t resAddr = ldStPhysAddr1_;
  if (lrResSize_ > size)
    {
      // Snap reservation address to the closest smaller muliple of
      // the reservation size (assumed to be a power of 2).
      size = lrResSize_;
      resAddr &= ~uint64_t(size - 1);
    }

  memory_.makeLr(hartIx_, resAddr, size);
  lrSuccess_++;
}


/// STORE_TYPE is either uint32_t or uint64_t.
template <typename URV>
template <typename STORE_TYPE>
bool
Hart<URV>::storeConditional(URV virtAddr, STORE_TYPE storeVal)
{
  ldStAddr_ = virtAddr;   // For reporting ld/st addr in trace-mode.
  ldStPhysAddr1_ = ldStAddr_;
  ldStPhysAddr2_ = ldStAddr_;
  ldStSize_ = sizeof(STORE_TYPE);
  ldStAtomic_ = true;

  // ld/st-address or instruction-address triggers have priority over
  // ld/st access or misaligned exceptions.
  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLoad = false;
  if (hasTrig)
    if (ldStAddrTriggerHit(virtAddr, timing, isLoad))
      triggerTripped_ = true;

  // Misaligned store causes an exception.
  constexpr unsigned alignMask = sizeof(STORE_TYPE) - 1;
  bool misal = virtAddr & alignMask;
  misalignedLdSt_ = misal;

  uint64_t addr1 = virtAddr, addr2 = virtAddr;
  auto cause = determineStoreException(addr1, addr2, sizeof(storeVal));
  ldStPhysAddr1_ = addr1;
  ldStPhysAddr2_ = addr2;
  if (cause == ExceptionCause::STORE_ADDR_MISAL and
      misalAtomicCauseAccessFault_)
    cause = ExceptionCause::STORE_ACC_FAULT;

  bool fail = misal or (amoInDccmOnly_ and not isAddrInDccm(addr1));
  if (cause == ExceptionCause::NONE)
    fail = fail or not memory_.pmaMgr_.getPma(addr1).isRsrv();

  if (fail and cause == ExceptionCause::NONE)
    cause = ExceptionCause::STORE_ACC_FAULT;

  // If no exception: consider store-data  trigger
  if (cause == ExceptionCause::NONE and hasTrig)
    if (ldStDataTriggerHit(storeVal, timing, isLoad))
      triggerTripped_ = true;
  if (triggerTripped_)
    return false;

  if (cause != ExceptionCause::NONE)
    {
      initiateStoreException(cause, virtAddr);
      return false;
    }

  if (not memory_.hasLr(hartIx_, addr1, sizeof(storeVal)))
    return false;

  STORE_TYPE prev = 0;
  memory_.peek(addr1, prev, false /*usePma*/);
  ldStData_ = storeVal;
  ldStPrevData_ = prev;
  ldStWrite_ = true;

  // If we write to special location, end the simulation.
  if (toHostValid_ and addr1 == toHost_ and storeVal != 0)
    {
      if (not memory_.write(hartIx_, addr1, storeVal))
	assert(0);
      throw CoreException(CoreException::Stop, "write to to-host",
			  toHost_, storeVal);
    }

  if (mcm_)
    return true;  // Memory updated when merge buffer is written.

  if (not memory_.write(hartIx_, addr1, storeVal))
    assert(0);

  invalidateDecodeCache(virtAddr, sizeof(STORE_TYPE));
  return true;
}


template <typename URV>
void
Hart<URV>::execSc_w(const DecodedInst* di)
{
  if (not isRva())
    {
      illegalInst(di);
      return;
    }

  std::lock_guard<std::mutex> lock(memory_.lrMutex_);

  uint32_t rd = di->op0(), rs1 = di->op1();
  URV value = intRegs_.read(di->op2());
  URV addr = intRegs_.read(rs1);
  scCount_++;

  uint64_t prevCount = exceptionCount_;

  bool ok = storeConditional(addr, uint32_t(value));
  cancelLr(); // Clear LR reservation (if any).

  if (ok)
    {
      memory_.invalidateOtherHartLr(hartIx_, addr, 4);
      intRegs_.write(rd, 0); // success
      scSuccess_++;

      return;
    }

  // If exception or trigger tripped then rd is not modified.
  if (triggerTripped_ or exceptionCount_ != prevCount)
    return;

  intRegs_.write(di->op0(), 1);  // fail
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execAmo32Op(const DecodedInst* di, OP op)
{
  if (not isRva())
    {
      illegalInst(di);
      return;
    }

  // Lock mutex to serialize AMO instructions. Unlock automatically on
  // exit from this scope.
  std::lock_guard<std::mutex> lock(memory_.amoMutex_);

  URV loadedValue = 0;
  uint32_t rd = di->op0(), rs1 = di->op1(), rs2 = di->op2();
  bool loadOk = amoLoad32(rs1, loadedValue);
  if (loadOk)
    {
      URV addr = intRegs_.read(rs1);

      URV rdVal = loadedValue;
      URV rs2Val = intRegs_.read(rs2);
      URV result = op(rs2Val, rdVal);

      bool storeOk = store<uint32_t>(addr, uint32_t(result));

      if (storeOk and not triggerTripped_)
	intRegs_.write(rd, rdVal);
    }
}


template <typename URV>
void
Hart<URV>::execAmoadd_w(const DecodedInst* di)
{
  execAmo32Op(di, std::plus<URV>{});
}


template <typename URV>
void
Hart<URV>::execAmoswap_w(const DecodedInst* di)
{
  auto getFirst = [] (URV a, URV) -> URV {
    return a;
  };

  execAmo32Op(di, getFirst);
}


template <typename URV>
void
Hart<URV>::execAmoxor_w(const DecodedInst* di)
{
  execAmo32Op(di, std::bit_xor{});
}


template <typename URV>
void
Hart<URV>::execAmoor_w(const DecodedInst* di)
{
  execAmo32Op(di, std::bit_or{});
}


template <typename URV>
void
Hart<URV>::execAmoand_w(const DecodedInst* di)
{
  execAmo32Op(di, std::bit_and{});
}


template <typename URV>
void
Hart<URV>::execAmomin_w(const DecodedInst* di)
{
  auto myMin = [] (URV a, URV b) -> URV {
    auto sa = static_cast<int32_t>(a);
    auto sb = static_cast<int32_t>(b);
    return std::min(sa, sb);
  };
  execAmo32Op(di, myMin);
}


template <typename URV>
void
Hart<URV>::execAmominu_w(const DecodedInst* di)
{
  auto myMin = [] (URV a, URV b) -> URV {
    auto ua = static_cast<uint32_t>(a);
    auto ub = static_cast<uint32_t>(b);
    return std::min(ua, ub);
  };
  execAmo32Op(di, myMin);
}


template <typename URV>
void
Hart<URV>::execAmomax_w(const DecodedInst* di)
{
  auto myMax = [] (URV a, URV b) -> URV {
    auto sa = static_cast<int32_t>(a);
    auto sb = static_cast<int32_t>(b);
    return std::max(sa, sb);
  };
  execAmo32Op(di, myMax);
}


template <typename URV>
void
Hart<URV>::execAmomaxu_w(const DecodedInst* di)
{
  auto myMax = [] (URV a, URV b) -> URV {
    auto ua = static_cast<uint32_t>(a);
    auto ub = static_cast<uint32_t>(b);
    return std::max(ua, ub);
  };
  execAmo32Op(di, myMax);
}


template <typename URV>
void
Hart<URV>::execLr_d(const DecodedInst* di)
{
  if (not isRva() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  std::lock_guard<std::mutex> lock(memory_.lrMutex_);

  lrCount_++;
  if (not loadReserve<int64_t>(di->op0(), di->op1()))
    return;

  unsigned size = 8;
  uint64_t resAddr = ldStPhysAddr1_; 
  if (lrResSize_ > size)
    {
      // Snap reservation address to the closest smaller muliple of
      // the reservation size (assumed to be a power of 2).
      size = lrResSize_;
      resAddr &= ~uint64_t(size - 1);
    }

  memory_.makeLr(hartIx_, resAddr, size);
  lrSuccess_++;
}


template <typename URV>
void
Hart<URV>::execSc_d(const DecodedInst* di)
{
  if (not isRva() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  std::lock_guard<std::mutex> lock(memory_.lrMutex_);

  uint32_t rd = di->op0(), rs1 = di->op1();
  URV value = intRegs_.read(di->op2());
  URV addr = intRegs_.read(rs1);
  scCount_++;

  uint64_t prevCount = exceptionCount_;

  bool ok = storeConditional(addr, uint64_t(value));
  cancelLr(); // Clear LR reservation (if any).

  if (ok)
    {
      memory_.invalidateOtherHartLr(hartIx_, addr, 8);
      intRegs_.write(rd, 0); // success
      scSuccess_++;

      return;
    }

  // If exception or trigger tripped then rd is not modified.
  if (triggerTripped_ or exceptionCount_ != prevCount)
    return;

  intRegs_.write(di->op0(), 1);  // fail
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execAmo64Op(const DecodedInst* di, OP op)
{
  if (not isRva() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  // Lock mutex to serialize AMO instructions. Unlock automatically on
  // exit from this scope.
  std::lock_guard<std::mutex> lock(memory_.amoMutex_);

  URV loadedValue = 0;
  URV rd = di->op0(), rs1 = di->op1(), rs2 = di->op2();
  bool loadOk = amoLoad64(rs1, loadedValue);
  if (loadOk)
    {
      URV addr = intRegs_.read(rs1);

      URV rdVal = loadedValue;
      URV rs2Val = intRegs_.read(rs2);
      URV result = op(rs2Val, rdVal);

      bool storeOk = store<uint64_t>(addr, result);

      if (storeOk and not triggerTripped_)
	intRegs_.write(rd, rdVal);
    }
}


template <typename URV>
void
Hart<URV>::execAmoadd_d(const DecodedInst* di)
{
  execAmo64Op(di, std::plus<URV>{});
}


template <typename URV>
void
Hart<URV>::execAmoswap_d(const DecodedInst* di)
{
  auto getFirst = [] (URV a, URV) -> URV {
    return a;
  };

  execAmo64Op(di, getFirst);
}


template <typename URV>
void
Hart<URV>::execAmoxor_d(const DecodedInst* di)
{
  execAmo64Op(di, std::bit_xor{});
}


template <typename URV>
void
Hart<URV>::execAmoor_d(const DecodedInst* di)
{
  execAmo64Op(di, std::bit_or{});
}


template <typename URV>
void
Hart<URV>::execAmoand_d(const DecodedInst* di)
{
  execAmo64Op(di, std::bit_and{});
}


template <typename URV>
void
Hart<URV>::execAmomin_d(const DecodedInst* di)
{
  auto myMin = [] (URV a, URV b) -> URV {
    auto sa = static_cast<int64_t>(a);
    auto sb = static_cast<int64_t>(b);
    return std::min(sa, sb);
  };
  execAmo64Op(di, myMin);
}


template <typename URV>
void
Hart<URV>::execAmominu_d(const DecodedInst* di)
{
  auto myMin = [] (URV a, URV b) -> URV {
    auto ua = static_cast<uint64_t>(a);
    auto ub = static_cast<uint64_t>(b);
    return std::min(ua, ub);
  };
  execAmo64Op(di, myMin);
}


template <typename URV>
void
Hart<URV>::execAmomax_d(const DecodedInst* di)
{
  auto myMax = [] (URV a, URV b) -> URV {
    auto sa = static_cast<int64_t>(a);
    auto sb = static_cast<int64_t>(b);
    return std::max(sa, sb);
  };
  execAmo64Op(di, myMax);
}


template <typename URV>
void
Hart<URV>::execAmomaxu_d(const DecodedInst* di)
{
  auto myMax = [] (URV a, URV b) -> URV {
    auto ua = static_cast<uint64_t>(a);
    auto ub = static_cast<uint64_t>(b);
    return std::max(ua, ub);
  };
  execAmo64Op(di, myMax);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
