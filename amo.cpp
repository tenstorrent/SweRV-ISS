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
Hart<URV>::validateAmoAddr(uint64_t& addr, uint64_t& gaddr, unsigned accessSize)
{
  using EC = ExceptionCause;

  URV mask = URV(accessSize) - 1;
  bool misal = (addr & mask) != 0;
  if (misal and misalHasPriority_)
    return misalAtomicCauseAccessFault_? EC::STORE_ACC_FAULT : EC::STORE_ADDR_MISAL;

  uint64_t addr2 = addr;
  uint64_t gaddr2 = gaddr;
  auto cause = determineStoreException(addr, addr2, gaddr, gaddr2, accessSize, false /*hyper*/);

  if (cause != ExceptionCause::NONE)
    return cause;

  // Address must be word aligned for word access and double-word
  // aligned for double-word access.
  if (misal)
    return misalAtomicCauseAccessFault_? EC::STORE_ACC_FAULT : EC::STORE_ADDR_MISAL;

  return ExceptionCause::NONE;
}


template <typename URV>
bool
Hart<URV>::amoLoad32([[maybe_unused]] const DecodedInst* di, uint64_t virtAddr,
		     [[maybe_unused]] Pma::Attrib  attrib, URV& value)
{
  ldStAddr_ = virtAddr;   // For reporting load addr in trace-mode.
  ldStPhysAddr1_ = ldStAddr_;
  ldStPhysAddr2_ = ldStAddr_;
  ldStSize_ = 4;
  ldStAtomic_ = true;

  uint64_t addr = virtAddr;

#ifndef FAST_SLOPPY

  if (hasActiveTrigger())
    {
      if (ldStAddrTriggerHit(virtAddr, ldStSize_, TriggerTiming::Before, true /*isLoad*/))
	triggerTripped_ = true;
    }

  uint64_t gaddr = virtAddr;
  auto cause = validateAmoAddr(addr, gaddr, ldStSize_);
  ldStPhysAddr1_ = addr;
  ldStPhysAddr2_ = addr;

  if (cause == ExceptionCause::NONE)
    {
      Pma pma = memory_.pmaMgr_.accessPma(addr, PmaManager::AccessReason::LdSt);
      // Check for non-cacheable pbmt
      pma = virtMem_.overridePmaWithPbmt(pma, virtMem_.lastEffectivePbmt(virtMode_));
      if (not pma.hasAttrib(attrib))
	cause = ExceptionCause::STORE_ACC_FAULT;
    }

  if (cause != ExceptionCause::NONE)
    {
      if (not triggerTripped_)
        initiateLoadException(di, cause, virtAddr, gaddr);
      return false;
    }

#endif

  uint32_t uval = 0;
  uint64_t mcmVal = 0;
  bool hasMcm = mcm_ and mcm_->getCurrentLoadValue(*this, *di, virtAddr, addr, addr, ldStSize_, mcmVal);

  if (hasMcm)
    uval = mcmVal;
  else
    memRead(addr, addr, uval);

  value = SRV(int32_t(uval)); // Sign extend.
  return true;  // Success.
}


template <typename URV>
bool
Hart<URV>::amoLoad64([[maybe_unused]] const DecodedInst* di, uint64_t virtAddr,
		     [[maybe_unused]] Pma::Attrib attrib, URV& value)
{
  ldStAddr_ = virtAddr;   // For reporting load addr in trace-mode.
  ldStPhysAddr1_ = ldStAddr_;
  ldStPhysAddr2_ = ldStAddr_;
  ldStSize_ = 8;
  ldStAtomic_ = true;

  uint64_t addr = virtAddr;

#ifndef FAST_SLOPPY

  if (hasActiveTrigger())
    {
      if (ldStAddrTriggerHit(virtAddr, ldStSize_, TriggerTiming::Before, true /*isLoad*/))
	triggerTripped_ = true;
    }

  uint64_t gaddr = virtAddr;
  auto cause = validateAmoAddr(addr, gaddr, ldStSize_);
  ldStPhysAddr1_ = addr;
  ldStPhysAddr2_ = addr;

  if (cause == ExceptionCause::NONE)
    {
      Pma pma = memory_.pmaMgr_.accessPma(addr, PmaManager::AccessReason::LdSt);
      // Check for non-cacheable pbmt
      pma = virtMem_.overridePmaWithPbmt(pma, virtMem_.lastEffectivePbmt(virtMode_));
      if (not pma.hasAttrib(attrib))
	cause = ExceptionCause::STORE_ACC_FAULT;
    }

  if (cause != ExceptionCause::NONE)
    {
      if (not triggerTripped_)
        initiateLoadException(di, cause, virtAddr, gaddr);
      return false;
    }

#endif

  uint64_t uval = 0;
  bool hasMcm = mcm_ and mcm_->getCurrentLoadValue(*this, *di, virtAddr, addr, addr, ldStSize_, uval);

  if (not hasMcm)
    memRead(addr, addr, uval);

  value = uval;
  return true;  // Success.
}


template <typename URV>
template <typename LOAD_TYPE>
bool
Hart<URV>::loadReserve(const DecodedInst* di, uint32_t rd, uint32_t rs1)
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
      if (ldStAddrTriggerHit(virtAddr, ldStSize_, TriggerTiming::Before, isLd))
	triggerTripped_ = true;
      if (triggerTripped_)
	return false;
    }

  // Unsigned version of LOAD_TYPE
  using ULT = typename std::make_unsigned<LOAD_TYPE>::type;

  uint64_t addr1 = virtAddr, addr2 = virtAddr;
  uint64_t gaddr1 = virtAddr;
  auto cause = ExceptionCause::NONE;
#ifndef FAST_SLOPPY
  uint64_t gaddr2 = virtAddr;
  cause = determineLoadException(addr1, addr2, gaddr1, gaddr2,
				 ldStSize_, false /*hyper*/);
#endif
  if (cause == ExceptionCause::LOAD_ADDR_MISAL and misalAtomicCauseAccessFault_)
    cause = ExceptionCause::LOAD_ACC_FAULT;

  ldStPhysAddr1_ = addr1;
  ldStPhysAddr1_ = addr2;

  bool fail = false;

  // Access must be naturally aligned.
  if ((addr1 & (ldStSize_ - 1)) != 0)
    fail = true;

  if (cause == ExceptionCause::NONE)
    {
      Pma pma = memory_.pmaMgr_.accessPma(addr1, PmaManager::AccessReason::LdSt);
      pma = virtMem_.overridePmaWithPbmt(pma, virtMem_.lastEffectivePbmt(virtMode_));
      fail = fail or not pma.isRsrv();
    }

  if (fail and cause == ExceptionCause::NONE)
    cause = ExceptionCause::LOAD_ACC_FAULT;

  if (cause != ExceptionCause::NONE)
    {
      initiateLoadException(di, cause, virtAddr, gaddr1);
      return false;
    }

  ULT uval = 0;
  bool hasMcmVal = false;
  if (mcm_)
    {
      uint64_t mcmVal = 0;
      if (mcm_->getCurrentLoadValue(*this, *di, virtAddr, addr1, addr1, ldStSize_, mcmVal))
	{
	  uval = mcmVal;
	  hasMcmVal = true;
	}
    }

  if (not hasMcmVal)
    memRead(addr1, addr1, uval);

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
  if (not loadReserve<int32_t>(di, di->op0(), di->op1()))
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
Hart<URV>::storeConditional(const DecodedInst* di, URV virtAddr, STORE_TYPE storeVal)
{
  ldStAtomic_ = true;

  ldStAddr_ = virtAddr;   // For reporting ld/st addr in trace-mode.
  ldStPhysAddr1_ = ldStPhysAddr2_ = ldStAddr_;
  ldStSize_ = sizeof(STORE_TYPE);

  // ld/st-address or instruction-address triggers have priority over
  // ld/st access or misaligned exceptions.
  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = false;  // Not a load.
  if (hasTrig and (ldStAddrTriggerHit(virtAddr, ldStSize_, timing, isLd) or
                   ldStDataTriggerHit(storeVal, timing, isLd)))
    triggerTripped_ = true;

  // Misaligned store causes an exception.
  constexpr unsigned alignMask = sizeof(STORE_TYPE) - 1;
  bool misal = virtAddr & alignMask;
  misalignedLdSt_ = misal;

  using EC = ExceptionCause;
  if (misal and misalHasPriority_)
    {
      auto cause = misalAtomicCauseAccessFault_ ? EC::STORE_ACC_FAULT : EC::STORE_ADDR_MISAL;
      initiateStoreException(di, cause, virtAddr, virtAddr);
      return false;
    }

  uint64_t addr1 = virtAddr, addr2 = virtAddr;
  uint64_t gaddr1 = virtAddr, gaddr2 = virtAddr;
  auto cause = determineStoreException(addr1, addr2, gaddr1, gaddr2, sizeof(storeVal), false /*hyper*/);
  ldStPhysAddr1_ = addr1;
  ldStPhysAddr2_ = addr2;

  if (cause == EC::NONE)
    {
      Pma pma = memory_.pmaMgr_.accessPma(addr1, PmaManager::AccessReason::LdSt);
      pma = virtMem_.overridePmaWithPbmt(pma, virtMem_.lastEffectivePbmt(virtMode_));
      if (not pma.isRsrv())
	cause = EC::STORE_ACC_FAULT;
    }

  if (triggerTripped_)
    return false;

  if (cause == EC::NONE and misal)
    cause = misalAtomicCauseAccessFault_ ? EC::STORE_ACC_FAULT : EC::STORE_ADDR_MISAL;

  if (cause != EC::NONE)
    {
      initiateStoreException(di, cause, virtAddr, gaddr1);
      return false;
    }

  if (not memory_.hasLr(hartIx_, addr1, sizeof(storeVal)))
    return false;

  ldStData_ = storeVal;
  ldStWrite_ = true;

  // If we write to special location, end the simulation.
  if (toHostValid_ and addr1 == toHost_ and storeVal != 0)
    {
      memWrite(addr1, addr1, storeVal);
      throw CoreException(CoreException::Stop, "write to to-host",
			  toHost_, storeVal);
    }

  if (mcm_)
    return true;  // Memory updated when merge buffer is written.

  if (isAclintAddr(addr1))
    {
      assert(addr1 == addr2);
      URV val = storeVal;
      processClintWrite(addr1, ldStSize_, val);
      storeVal = val;
    }
  else if (isInterruptorAddr(addr1, ldStSize_))
    processInterruptorWrite(storeVal);
  else if (isImsicAddr(addr1))
    {
      imsicWrite_(addr1, sizeof(storeVal), storeVal);
      storeVal = 0;  // Reads from IMSIC space will yield zero.
    }

  memWrite(addr1, addr1, storeVal);

  STORE_TYPE temp = 0;
  memPeek(addr1, addr2, temp, false /*usePma*/);
  ldStData_ = temp;

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

  bool ok = storeConditional(di, addr, uint32_t(value));

  // If there is an exception then reservation may/may-not be dropped
  // depending on config.
  if (not keepReservOnScException_ or not hasException_)
    if (not perfApi_)
      cancelLr(CancelLrCause::SC); // Clear LR reservation (if any).

  if (ok)
    {
      memory_.invalidateOtherHartLr(hartIx_, ldStPhysAddr1_, 4);
      intRegs_.write(rd, 0); // success
      scSuccess_++;

      return;
    }

  // If exception or trigger tripped then rd is not modified.
  if (triggerTripped_ or hasException_)
    return;

  intRegs_.write(di->op0(), 1);  // fail
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execAmo32Op(const DecodedInst* di, Pma::Attrib attrib, OP op)
{
  if (not isRva())
    {
      illegalInst(di);
      return;
    }

  // Lock mutex to serialize AMO instructions. Unlock automatically on
  // exit from this scope.
  std::unique_lock lock(memory_.amoMutex_);

  URV loadedValue = 0;
  uint32_t rd = di->op0(), rs1 = di->op1(), rs2 = di->op2();
  URV virtAddr = intRegs_.read(rs1);
  bool loadOk = amoLoad32(di, virtAddr, attrib, loadedValue);
  if (loadOk)
    {
      URV addr = intRegs_.read(rs1);

      URV rdVal = loadedValue;
      URV rs2Val = intRegs_.read(rs2);
      URV result = op(rs2Val, rdVal);

      bool storeOk = store<uint32_t>(di, addr, false /*hyper*/, uint32_t(result), false);

      if (storeOk and not triggerTripped_)
	{
	  intRegs_.write(rd, rdVal);
	  ldStData_ = uint32_t(result);
	  ldStWrite_ = true;
	}
    }
}


template <typename URV>
void
Hart<URV>::execAmoadd_w(const DecodedInst* di)
{
  execAmo32Op(di, Pma::AmoOther, std::plus<URV>{});
}


template <typename URV>
void
Hart<URV>::execAmoswap_w(const DecodedInst* di)
{
  auto getFirst = [] (URV a, URV) -> URV {
    return a;
  };

  execAmo32Op(di, Pma::AmoSwap, getFirst);
}


template <typename URV>
void
Hart<URV>::execAmoxor_w(const DecodedInst* di)
{
  execAmo32Op(di, Pma::AmoLogical, std::bit_xor{});
}


template <typename URV>
void
Hart<URV>::execAmoor_w(const DecodedInst* di)
{
  execAmo32Op(di, Pma::AmoLogical, std::bit_or{});
}


template <typename URV>
void
Hart<URV>::execAmoand_w(const DecodedInst* di)
{
  execAmo32Op(di, Pma::AmoLogical, std::bit_and{});
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
  execAmo32Op(di, Pma::AmoOther, myMin);
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
  execAmo32Op(di, Pma::AmoOther, myMin);
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
  execAmo32Op(di, Pma::AmoOther, myMax);
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
  execAmo32Op(di, Pma::AmoOther, myMax);
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
  if (not loadReserve<int64_t>(di, di->op0(), di->op1()))
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

  bool ok = storeConditional(di, addr, uint64_t(value));

  // If there is an exception then reservation may/may-not be dropped
  // depending on config.
  if (not keepReservOnScException_ or not hasException_)
    if (not perfApi_)
      cancelLr(CancelLrCause::SC); // Clear LR reservation (if any).

  if (ok)
    {
      memory_.invalidateOtherHartLr(hartIx_, ldStPhysAddr1_, 8);
      intRegs_.write(rd, 0); // success
      scSuccess_++;

      return;
    }

  // If exception or trigger tripped then rd is not modified.
  if (triggerTripped_ or hasException_)
    return;

  intRegs_.write(di->op0(), 1);  // fail
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execAmo64Op(const DecodedInst* di, Pma::Attrib attrib, OP op)
{
  if (not isRva() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  // Lock mutex to serialize AMO instructions. Unlock automatically on
  // exit from this scope.
  std::unique_lock lock(memory_.amoMutex_);

  URV loadedValue = 0;
  URV rd = di->op0(), rs1 = di->op1(), rs2 = di->op2();
  URV virtAddr = intRegs_.read(rs1);
  bool loadOk = amoLoad64(di, virtAddr, attrib, loadedValue);
  if (loadOk)
    {
      URV addr = intRegs_.read(rs1);
      URV rdVal = loadedValue;
      URV rs2Val = intRegs_.read(rs2);
      URV result = op(rs2Val, rdVal);

      bool storeOk = store<uint64_t>(di, addr, false /*hyper*/, result, false);

      if (storeOk and not triggerTripped_)
	{
	  intRegs_.write(rd, rdVal);
	  ldStData_ = result;
	  ldStWrite_ = true;
	}
    }
}


template <typename URV>
void
Hart<URV>::execAmoadd_d(const DecodedInst* di)
{
  execAmo64Op(di, Pma::AmoOther, std::plus<URV>{});
}


template <typename URV>
void
Hart<URV>::execAmoswap_d(const DecodedInst* di)
{
  auto getFirst = [] (URV a, URV) -> URV {
    return a;
  };

  execAmo64Op(di, Pma::AmoSwap, getFirst);
}


template <typename URV>
void
Hart<URV>::execAmoxor_d(const DecodedInst* di)
{
  execAmo64Op(di, Pma::AmoLogical, std::bit_xor{});
}


template <typename URV>
void
Hart<URV>::execAmoor_d(const DecodedInst* di)
{
  execAmo64Op(di, Pma::AmoLogical, std::bit_or{});
}


template <typename URV>
void
Hart<URV>::execAmoand_d(const DecodedInst* di)
{
  execAmo64Op(di, Pma::AmoLogical, std::bit_and{});
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
  execAmo64Op(di, Pma::AmoOther, myMin);
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
  execAmo64Op(di, Pma::AmoOther, myMin);
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
  execAmo64Op(di, Pma::AmoOther, myMax);
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
  execAmo64Op(di, Pma::AmoOther, myMax);
}


template <typename URV>
void
Hart<URV>::execAmocas_w(const DecodedInst* di)
{
  if (not isRva() or not isRvzacas())
    {
      illegalInst(di);
      return;
    }

  // Lock mutex to serialize AMO instructions. Unlock automatically on
  // exit from this scope.
  std::unique_lock lock(memory_.amoMutex_);

  URV loadedVal = 0;
  uint32_t rd = di->op0(), rs1 = di->op1(), rs2 = di->op2();
  URV addr = intRegs_.read(rs1);
  bool loadOk = amoLoad32(di, addr, Pma::Attrib::AmoArith, loadedVal);
  uint32_t temp = loadedVal;

  if (loadOk)
    {
      uint32_t rs2Val = intRegs_.read(rs2);
      uint32_t rdVal = intRegs_.read(rd);

      bool storeOk = true;
      if (temp == rdVal)
	storeOk = store<uint32_t>(di, addr, false /*hyper*/, uint32_t(rs2Val), false);

      if (storeOk and not triggerTripped_)
	{
	  SRV result = int32_t(temp);   // Sign extended in RV64
	  intRegs_.write(rd, result);
	}
    }
}


template <>
void
Hart<uint32_t>::execAmocas_d(const DecodedInst* di)
{
  if (not isRva() or not isRvzacas())
    {
      illegalInst(di);
      return;
    }

  // Lock mutex to serialize AMO instructions. Unlock automatically on
  // exit from this scope.
  std::unique_lock lock(memory_.amoMutex_);

  uint32_t rd = di->op0(), rs1 = di->op1(), rs2 = di->op2();
  if ((rd & 1) == 1 or (rs2 & 1) == 1)
    {
      illegalInst(di);    // rd and rs2 must be even
      return;
    }

  Pma::Attrib attrib = Pma::Attrib::AmoArith;

  uint32_t temp0 = 0, temp1 = 0;
  uint32_t addr = intRegs_.read(rs1);
  bool loadOk = (amoLoad32(di, addr, attrib, temp0) and
		 amoLoad32(di, addr + 4, attrib, temp1));
  if (loadOk)
    {
      uint32_t rs2Val0 = intRegs_.read(rs2);
      uint32_t rs2Val1 = intRegs_.read(rs2 + 1);
      uint32_t rdVal0 = intRegs_.read(rd);
      uint32_t rdVal1 = intRegs_.read(rd + 1);
      if (rs2 == 0)
	rs2Val1 = 0;
      if (rd == 0)
	rdVal1 = 0;

      bool storeOk = true;
      if (temp0 == rdVal0 and temp1 == rdVal1)
	{
	  storeOk = store<uint32_t>(di, addr, false /*hyper*/, uint32_t(rs2Val0), false);
	  storeOk = storeOk and store<uint32_t>(di, addr + 4, false, uint32_t(rs2Val1), false);
	}

      if (storeOk and not triggerTripped_ and rd != 0)
	{
	  intRegs_.write(rd, temp0);
	  intRegs_.write(rd+1, temp1);
	}
    }
}


template <>
void
Hart<uint64_t>::execAmocas_d(const DecodedInst* di)
{
  if (not isRva() or not isRvzacas())
    {
      illegalInst(di);
      return;
    }

  // Lock mutex to serialize AMO instructions. Unlock automatically on
  // exit from this scope.
  std::unique_lock lock(memory_.amoMutex_);

  uint32_t rd = di->op0(), rs1 = di->op1(), rs2 = di->op2();

  Pma::Attrib attrib = Pma::Attrib::AmoArith;

  uint64_t temp = 0;
  uint64_t addr = intRegs_.read(rs1);
  bool loadOk = amoLoad64(di, addr, attrib, temp);

  if (loadOk)
    {
      uint64_t rs2Val = intRegs_.read(rs2);
      uint64_t rdVal = intRegs_.read(rd);

      bool storeOk = true;
      if (temp == rdVal)
	storeOk = store<uint64_t>(di, addr, false /*hyper*/, rs2Val, false);

      if (storeOk and not triggerTripped_)
	intRegs_.write(rd, temp);
    }
}


template <>
void
Hart<uint32_t>::execAmocas_q(const DecodedInst* di)
{
  illegalInst(di);
}


template <>
void
Hart<uint64_t>::execAmocas_q(const DecodedInst* di)
{
  if (not isRva() or not isRvzacas())
    {
      illegalInst(di);
      return;
    }

  // Lock mutex to serialize AMO instructions. Unlock automatically on
  // exit from this scope.
  std::unique_lock lock(memory_.amoMutex_);

  uint64_t rd = di->op0(), rs1 = di->op1(), rs2 = di->op2();
  if ((rd & 1) == 1 or (rs2 & 1) == 1)
    {
      illegalInst(di);    // rd and rs2 must be even
      return;
    }

  Pma::Attrib attrib = Pma::AmoArith;

  uint64_t temp0 = 0, temp1 = 0;
  uint64_t addr = intRegs_.read(rs1);

  URV mask = 0xf;
  bool misal = (addr & mask) != 0;
  if (misal and misalHasPriority_)
    {
      if (misalAtomicCauseAccessFault_)
	initiateStoreException(di, ExceptionCause::STORE_ACC_FAULT, addr, addr);
      initiateStoreException(di, ExceptionCause::STORE_ADDR_MISAL, addr, addr);
      return;
    }

  // FIX: This needs to be fixed for correct tracing
  bool loadOk = (amoLoad64(di, addr, attrib, temp0) and
		 amoLoad64(di, addr + 8, attrib, temp1));
  if (loadOk)
    {
      uint64_t rs2Val0 = intRegs_.read(rs2);
      uint64_t rs2Val1 = intRegs_.read(rs2 + 1);
      uint64_t rdVal0 = intRegs_.read(rd);
      uint64_t rdVal1 = intRegs_.read(rd + 1);
      if (rs2 == 0)
	rs2Val1 = 0;
      if (rd == 0)
	rdVal1 = 0;

      bool storeOk = true;
      if (temp0 == rdVal0 and temp1 == rdVal1)
	{
	  storeOk = store<uint64_t>(di, addr, false /*hyper*/, uint64_t(rs2Val0), false);
	  storeOk = storeOk and store<uint64_t>(di, addr + 8, false, uint64_t(rs2Val1), false);
	}

      if (storeOk and not triggerTripped_ and rd != 0)
	{
	  intRegs_.write(rd, temp0);
	  intRegs_.write(rd+1, temp1);
	}
    }
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
