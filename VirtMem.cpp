#include <cmath>
#include <iostream>
#include <ios>
#include "PmpManager.hpp"
#include "VirtMem.hpp"

using namespace WdRiscv;


VirtMem::VirtMem(unsigned hartIx, Memory& memory, unsigned pageSize,
                 PmpManager& pmpMgr, unsigned tlbSize)
  : memory_(memory), pageSize_(pageSize), hartIx_(hartIx),
    pmpMgr_(pmpMgr), tlb_(tlbSize), vsTlb_(tlbSize), stage2Tlb_(tlbSize)
{
  supportedModes_.resize(unsigned(Mode::Limit_));
  setSupportedModes({Mode::Bare, Mode::Sv32, Mode::Sv39, Mode::Sv48, Mode::Sv57, Mode::Sv64});

  supportedPmms_.resize(unsigned(Pmm::Limit_));
  setSupportedPmms({Pmm::Off, Pmm::Pm57, Pmm::Pm48});

  pageBits_ = static_cast<unsigned>(std::log2(pageSize_));
  unsigned p2PageSize =  unsigned(1) << pageBits_;
  (void)p2PageSize;
  assert(p2PageSize == pageSize);
  assert(pageSize >= 64);

  pageMask_ = pageSize_ - 1;
}


/// Page fault type for read/write/exec access (one and only one of
/// which must be true). This is for stage 1 or single-stage translation.
static constexpr
ExceptionCause
stage1PageFaultType(bool read, bool write, bool exec)
{
  if (exec)  return ExceptionCause::INST_PAGE_FAULT;
  if (read)  return ExceptionCause::LOAD_PAGE_FAULT;
  if (write) return ExceptionCause::STORE_PAGE_FAULT;
  assert(0);
  return ExceptionCause::STORE_PAGE_FAULT;
}


/// Page fault type for read/write/exec access (one and only one of
/// which must be true). This is for stage 2 translation only.
static constexpr
ExceptionCause
stage2PageFaultType(bool read, bool write, bool exec)
{
  if (exec)  return ExceptionCause::INST_GUEST_PAGE_FAULT;
  if (read)  return ExceptionCause::LOAD_GUEST_PAGE_FAULT;
  if (write) return ExceptionCause::STORE_GUEST_PAGE_FAULT;
  assert(0);
  return ExceptionCause::STORE_GUEST_PAGE_FAULT;
}


static constexpr
ExceptionCause
accessFaultType(bool read, bool write, bool exec)
{
  if (exec)  return ExceptionCause::INST_ACC_FAULT;
  if (read)  return ExceptionCause::LOAD_ACC_FAULT;
  if (write) return ExceptionCause::STORE_ACC_FAULT;
  assert(0);
  return ExceptionCause::LOAD_ACC_FAULT;
}


/// Change the exception resulting from an implicit access during the
/// VS-stage to the exception type corresponding to the original
/// explicit access (determined by one of read/write/exec). We keep
/// the guest page fault but we may change its flavor. See section
/// 9.5.1. of the privileged spec.
static constexpr
ExceptionCause
stage2ExceptionToStage1(ExceptionCause ec2, bool read, bool write, bool exec)
{
  using EC = ExceptionCause;
  if (ec2 == EC::INST_GUEST_PAGE_FAULT or ec2 == EC::LOAD_GUEST_PAGE_FAULT or
      ec2 == EC::STORE_GUEST_PAGE_FAULT)
    return stage2PageFaultType(read, write, exec);
  if (ec2 == EC::INST_ACC_FAULT or ec2 == EC::LOAD_ACC_FAULT or ec2 == EC::STORE_ACC_FAULT)
    return accessFaultType(read, write, exec);
  return ec2;
}


ExceptionCause
VirtMem::translateForFetch2(uint64_t va, unsigned size, PrivilegeMode priv,
			    bool twoStage, uint64_t& gpa1, uint64_t& pa1,
                            uint64_t& gpa2, uint64_t& pa2)
{
  gpa1 = pa1 = gpa2 = pa2 = va;
  auto cause = translateForFetch(va, priv, twoStage, gpa1, pa1);
  if (cause != ExceptionCause::NONE)
    return cause;

  gpa2 = gpa1;
  pa2 = pa1;
  unsigned excess = va & (size - 1);  // va modulo size

  if (excess == 0)
    return ExceptionCause::NONE;

  // Misaligned acces. Check if crossing page boundary.
  uint64_t n1 = pageNumber(va);
  uint64_t n2 = pageNumber(va + size - 1);
  if (n1 == n2)
    return ExceptionCause::NONE;  // Not page crossing

  fetchPageCross_ = true;
  uint64_t va2 = n2*pageSize_;
  cause = translateForFetch(va2, priv, twoStage, gpa2, pa2);
  if (cause != ExceptionCause::NONE)
    {
      gpa1 = gpa2;
      pa1 = pa2 = va2;
    }

  return cause;
}


ExceptionCause
VirtMem::transAddrNoUpdate(uint64_t va, PrivilegeMode priv, bool twoStage,
			   bool read, bool write, bool exec, uint64_t& pa)
{
  auto prevTrace = trace_; trace_ = false;
  accessDirtyCheck_ = false;

  auto cause = transNoUpdate(va, priv, twoStage, read, write, exec, pa);

  trace_ = prevTrace;
  accessDirtyCheck_ = true;

  return cause;
}


ExceptionCause
VirtMem::transNoUpdate(uint64_t va, PrivilegeMode priv, bool twoStage,
		       bool read, bool write, bool exec, uint64_t& pa)
{
  // Exactly one of read/write/exec must be true.
  assert((static_cast<int>(read) + static_cast<int>(write) + static_cast<int>(exec)) == 1);

  // TBD FIX -- use TLB for two-stage case.
  if (not twoStage)
    {
      if (mode_ == Mode::Bare)
	{
	  pa = va;
	  return ExceptionCause::NONE;
	}

      // Lookup virtual page number in TLB.
      uint64_t virPageNum = va >> pageBits_;
      TlbEntry* entry = tlb_.findEntry(virPageNum, asid_);
      if (entry)
	{
	  // Use TLB entry.
	  if (priv == PrivilegeMode::User and not entry->user_)
	    return stage1PageFaultType(read, write, exec);
	  if (priv == PrivilegeMode::Supervisor)
	    if (entry->user_ and (exec or not sum_))
	      return stage1PageFaultType(read, write, exec);
	  bool ra = entry->read_ or (execReadable_ and entry->exec_);
	  bool wa = entry->write_, xa = entry->exec_;
	  if ((read and not ra) or (write and not wa) or (exec and not xa))
	    return stage1PageFaultType(read, write, exec);
	  // We do not check/update access/dirty bits.
	  pa = (entry->physPageNum_ << pageBits_) | (va & pageMask_);
          pbmt_ = Pbmt(entry->pbmt_);
	  return ExceptionCause::NONE;
	}
    }

  TlbEntry tlbEntry;
  return translateNoTlb(va, priv, twoStage, read, write, exec, pa, tlbEntry);
}


ExceptionCause
VirtMem::translateForLdSt2(uint64_t va, unsigned size, PrivilegeMode priv,
                           bool twoStage, bool load, uint64_t& gpa1, uint64_t& pa1,
                           uint64_t& gpa2, uint64_t& pa2)
{
  gpa1 = pa1 = gpa2 = pa2 = va;

  bool read = load, write = not load, exec = false;
  auto cause = translate(va, priv, twoStage, read, write, exec, gpa1, pa1);
  if (cause != ExceptionCause::NONE)
    return cause;

  gpa2 = gpa1;
  pa2 = pa1;
  unsigned excess = va & (size - 1);  // va modulo size

  if (excess == 0)
    return ExceptionCause::NONE;

  // Misaligned acces. Check if crossing page boundary.
  uint64_t n1 = pageNumber(va);
  uint64_t n2 = pageNumber(va + size - 1);
  if (n1 == n2)
    return ExceptionCause::NONE;  // Not page crossing

  uint64_t va2 = n2*pageSize_;
  cause = translate(va2, priv, twoStage, read, write, exec, gpa2, pa2);
  if (cause != ExceptionCause::NONE) {
    gpa1 = gpa2;
    pa1 = pa2 = va2;
  }

  return cause;
}


ExceptionCause
VirtMem::translate(uint64_t va, PrivilegeMode priv, bool twoStage,
		   bool read, bool write, bool exec, uint64_t& gpa, uint64_t& pa)
{
  if (twoStage)
    return twoStageTranslate(va, priv, read, write, exec, gpa, pa);

  // Exactly one of read/write/exec must be true.
  assert((static_cast<int>(read) + static_cast<int>(write) + static_cast<int>(exec)) == 1);

  pa = va;

  if (mode_ == Mode::Bare)
    return ExceptionCause::NONE;

  // Lookup virtual page number in TLB.
  uint64_t virPageNum = va >> pageBits_;
  TlbEntry* entry = tlb_.findEntryUpdateTime(virPageNum, asid_);
  if (entry)
    {
      // Use TLB entry.
      if (priv == PrivilegeMode::User and not entry->user_)
        return stage1PageFaultType(read, write, exec);
      if (priv == PrivilegeMode::Supervisor)
	if (entry->user_ and (exec or not sum_))
	  return stage1PageFaultType(read, write, exec);
      bool ra = entry->read_ or (execReadable_ and entry->exec_);
      bool wa = entry->write_, xa = entry->exec_;
      if ((read and not ra) or (write and not wa) or (exec and not xa))
        return stage1PageFaultType(read, write, exec);
      if (not entry->accessed_ or (write and not entry->dirty_))
	entry->valid_ = false;
      if (entry->valid_)
	{
	  pa = (entry->physPageNum_ << pageBits_) | (va & pageMask_);
	  pbmt_ = Pbmt(entry->pbmt_);
	  return ExceptionCause::NONE;
	}
    }

  TlbEntry tlbEntry;
  auto cause = translateNoTlb(va, priv, twoStage, read, write, exec, pa, tlbEntry);

  // If successful, put translation results in TLB.
  if (cause == ExceptionCause::NONE)
    tlb_.insertEntry(tlbEntry);

  return cause;
}


ExceptionCause
VirtMem::twoStageTranslateNoTlb(uint64_t va, PrivilegeMode priv, bool read,
				bool write, bool exec, uint64_t& pa, TlbEntry& entry)
{
  uint64_t gpa = va;
  if (vsMode_ != Mode::Bare)
    {
      auto cause = stage1TranslateNoTlb(va, priv, read, write, exec, gpa, entry);
      if (cause != ExceptionCause::NONE)
	return cause;
    }

  if (modeStage2_ == Mode::Bare)
    {
      pa = gpa;
      return ExceptionCause::NONE;
    }

  TlbEntry entry2;
  return stage2TranslateNoTlb(gpa, priv, read, write, exec, /* isPteAddr */ false, pa, entry2);
}


ExceptionCause
VirtMem::translateNoTlb(uint64_t va, PrivilegeMode priv, bool twoStage, bool read,
			bool write, bool exec, uint64_t& pa, TlbEntry& entry)
{
  if (twoStage)
    return twoStageTranslateNoTlb(va, priv, read, write, exec, pa, entry);

  // Perform a page table walk.
  if (mode_ == Mode::Sv32)
    return pageTableWalk<Pte32, Va32>(va, priv, read, write, exec, pa, entry);

  ExceptionCause (VirtMem::*walkFn)(uint64_t, PrivilegeMode, bool, bool, bool, uint64_t&, TlbEntry&);
  unsigned vaMsb = 0;  // Most significant bit of va

  if (mode_ == Mode::Sv39)
    {
      vaMsb = 38; // Bits 63 to 39 of va must equal bit 38
      walkFn = &VirtMem::pageTableWalk<Pte39, Va39>;
    }
  else if (mode_ == Mode::Sv48)
    {
      vaMsb = 47; // Bits 63 to 48 of va must equal bit 47
      walkFn = &VirtMem::pageTableWalk<Pte48, Va48>;
    }
  else if (mode_ == Mode::Sv57)
    {
      vaMsb = 56; // Bits 63 to 57 of va must equal bit 56
      walkFn = &VirtMem::pageTableWalk<Pte57, Va57>;
    }
  else
    {
      assert(0 and "Unsupported virtual memory mode.");
      return ExceptionCause::LOAD_PAGE_FAULT;
    }

  // Bits higher than bit vaMsb must be identical to bit vaMsb.
  uint64_t va1 = va;
  uint64_t va2 = (int64_t(va) << (63-vaMsb)) >> (63-vaMsb); // Expected va.
  if (va1 != va2)
    return stage1PageFaultType(read, write, exec);

  return (this->*walkFn)(va, priv, read, write, exec, pa, entry);
}


ExceptionCause
VirtMem::stage2TranslateNoTlb(uint64_t va, PrivilegeMode priv, bool read,
			      bool write, bool exec, bool isPteAddr, uint64_t& pa, TlbEntry& entry)
{
  // Perform a page table walk.
  ExceptionCause (VirtMem::*stage2PageTableWalk)(uint64_t, PrivilegeMode, bool, bool, bool, bool, uint64_t&, TlbEntry&);
  unsigned lowerMaskBitIndex = 0;

  if (modeStage2_ == Mode::Sv32)
    {
      // Part 2 of address translation: Bits 63-34 must be zero
      lowerMaskBitIndex   = 34;
      stage2PageTableWalk = &VirtMem::stage2PageTableWalk<Pte32, Va32x4>;
    }
  else if (modeStage2_ == Mode::Sv39)
    {
      // Part 2 of address translation: Bits 63-41 must be zero
      lowerMaskBitIndex   = 41;
      stage2PageTableWalk = &VirtMem::stage2PageTableWalk<Pte39, Va39x4>;
    }
  else if (modeStage2_ == Mode::Sv48)
    {
      // Part 2 of address translation: Bits 63-50 must be zero
      lowerMaskBitIndex   = 50;
      stage2PageTableWalk = &VirtMem::stage2PageTableWalk<Pte48, Va48x4>;
    }
  else if (modeStage2_ == Mode::Sv57)
    {
      // Part 2 of address translation: Bits 63-59 must be zero
      lowerMaskBitIndex   = 59;
      stage2PageTableWalk = &VirtMem::stage2PageTableWalk<Pte57, Va57x4>;
    }
  else
    {
      assert(0 and "Unsupported virtual memory mode.");
      return ExceptionCause::LOAD_PAGE_FAULT;
    }

  if ((va >> lowerMaskBitIndex) != 0)
    return stage2PageFaultType(read, write, exec);
  return (this->*stage2PageTableWalk)(va, priv, read, write, exec, isPteAddr, pa, entry);
}


ExceptionCause
VirtMem::stage2Translate(uint64_t va, PrivilegeMode priv, bool read, bool write,
			 bool exec, bool isPteAddr, uint64_t& pa)
{
  // Exactly one of read/write/exec must be true.
  assert((static_cast<int>(read) + static_cast<int>(write) + static_cast<int>(exec)) == 1);

  if (modeStage2_ == Mode::Bare)
    {
      pa = va;
      return ExceptionCause::NONE;
    }

  // Lookup virtual page number in TLB.
  uint64_t virPageNum = va >> pageBits_;
  TlbEntry* entry = stage2Tlb_.findEntryUpdateTime(virPageNum, asid_, vmid_);
  if (entry)
    {
      // Use TLB entry.
      if (not entry->user_)
        return stage2PageFaultType(read, write, exec);
      bool ra = entry->read_ or (execReadable_ and entry->exec_ and not isPteAddr);
      if (not isPteAddr and xForR_)
	ra = entry->exec_;
      bool wa = entry->write_, xa = entry->exec_;
      if ((read and not ra) or (write and not wa) or (exec and not xa))
        return stage2PageFaultType(read, write, exec);
      if (not entry->accessed_ or (write and not entry->dirty_))
	entry->valid_ = false;
      if (entry->valid_)
	{
	  pa = (entry->physPageNum_ << pageBits_) | (va & pageMask_);
	  pbmt_ = Pbmt(entry->pbmt_);
	  return ExceptionCause::NONE;
	}
    }

  TlbEntry tlbEntry;
  auto cause = stage2TranslateNoTlb(va, priv, read, write, exec, isPteAddr, pa, tlbEntry);

  // If successful, put translation results in TLB.
  if (cause == ExceptionCause::NONE)
    stage2Tlb_.insertEntry(tlbEntry);

  return cause;
}


ExceptionCause
VirtMem::twoStageTranslate(uint64_t va, PrivilegeMode priv, bool read, bool write,
			   bool exec, uint64_t& gpa, uint64_t& pa)
{
  // Exactly one of read/write/exec must be true.
  assert((static_cast<int>(read) + static_cast<int>(write) + static_cast<int>(exec)) == 1);

  gpa = pa = va;

  if (vsMode_ != Mode::Bare)
    {
      // Lookup virtual page number in TLB.
      uint64_t virPageNum = va >> pageBits_;
      TlbEntry* entry = vsTlb_.findEntryUpdateTime(virPageNum, vsAsid_);
      if (entry)
	{
	  if (priv == PrivilegeMode::User and not entry->user_)
            return stage1PageFaultType(read, write, exec);
	  if (priv == PrivilegeMode::Supervisor)
	    if (entry->user_ and (exec or not vsSum_))
              return stage1PageFaultType(read, write, exec);
	  bool ra = entry->read_ or ((execReadable_ or s1ExecReadable_) and entry->exec_);
	  if (xForR_)
	    ra = entry->exec_;
	  bool wa = entry->write_, xa = entry->exec_;
	  if ((read and not ra) or (write and not wa) or (exec and not xa))
            return stage1PageFaultType(read, write, exec);
	  if (not entry->accessed_ or (write and not entry->dirty_))
	    entry->valid_ = false;
	  if (entry->valid_)
	    {
	      // Use TLB entry.
	      pbmt_ = Pbmt(entry->pbmt_);
	      gpa = (entry->physPageNum_ << pageBits_) | (va & pageMask_);
	    }
	}

      if (not entry or not entry->valid_)
	{
	  TlbEntry tlbEntry;
	  auto cause = stage1TranslateNoTlb(va, priv, read, write, exec, gpa, tlbEntry);

	  // If successful, put stage1 translation results in TLB.
	  if (cause == ExceptionCause::NONE)
	    vsTlb_.insertEntry(tlbEntry);
	  else
	    return cause;
	}
    }

  return stage2Translate(gpa, priv, read, write, exec, /* isPteAddr */ false, pa);
}


ExceptionCause
VirtMem::stage1TranslateNoTlb(uint64_t va, PrivilegeMode priv, bool read, bool write,
			      bool exec, uint64_t& pa, TlbEntry& entry)
{
  if (vsMode_ == Mode::Sv32)
    {
      auto cause =  stage1PageTableWalk<Pte32, Va32>(va, priv, read, write, exec, pa, entry);
      s1ImplAccTrap_ = cause != ExceptionCause::NONE;
      return cause;
    }

  ExceptionCause (VirtMem::*walkFn)(uint64_t, PrivilegeMode, bool, bool, bool, uint64_t&, TlbEntry&);
  unsigned vaMsb = 0;  // Most significant bit of va

  if (vsMode_ == Mode::Sv39)
    {
      vaMsb = 38; // Bits 63 to 39 of va must equal bit 38
      walkFn = &VirtMem::stage1PageTableWalk<Pte39, Va39>;
    }
  else if (vsMode_ == Mode::Sv48)
    {
      vaMsb = 47; // Bits 63 to 48 of va must equal bit 47
      walkFn = &VirtMem::stage1PageTableWalk<Pte48, Va48>;
    }
  else if (vsMode_ == Mode::Sv57)
    {
      vaMsb = 56; // Bits 63 to 57 of va must equal bit 56
      walkFn = &VirtMem::stage1PageTableWalk<Pte57, Va57>;
    }
  else
    {
      assert(0 and "Unsupported virtual memory mode.");
      return ExceptionCause::LOAD_PAGE_FAULT;
    }

  // Bits higher than bit vaMsb must be identical to bit vaMsb.
  uint64_t va1 = va;
  uint64_t va2 = (int64_t(va) << (63-vaMsb)) >> (63-vaMsb); // Expected va.
  if (va1 != va2)
    return stage1PageFaultType(read, write, exec);

  auto cause = (this->*walkFn)(va, priv, read, write, exec, pa, entry);
  s1ImplAccTrap_ = cause != ExceptionCause::NONE;
  return cause;
}


template<typename PTE, typename VA>
ExceptionCause
VirtMem::pageTableWalk(uint64_t address, PrivilegeMode privMode, bool read, bool write,
		       bool exec, uint64_t& pa, TlbEntry& tlbEntry)
{
  // 1. Root is "a" in section 4.3.2 of the privileged spec, ii is "i" in that section.
  uint64_t root = rootPage_ * pageSize_;

  PTE pte(0);
  const unsigned levels = pte.levels();
  const unsigned pteSize = pte.size();
  int ii = levels - 1;

  VA va(address);

  // Collect PTE addresses used in the translation process.
  auto& walkVec = exec ? fetchWalks_ : dataWalks_;
  if (trace_)
    {
      walkVec.resize(walkVec.size() + 1);
      walkVec.back().emplace_back(address, WalkEntry::Type::GPA);
    }

  bool global = false;

  while (true)
    {
      // 2.
      uint64_t pteAddr = root + va.vpn(ii)*pteSize;

      int walkEntryIx = 0;
      if (trace_)
        {
          walkVec.back().emplace_back(pteAddr);
          walkEntryIx = walkVec.back().size() - 1;
        }

      // Check PMP. The privMode here is the effective one that
      // already accounts for MPRV.
      if (pmpMgr_.isEnabled())
	{
	  const Pmp& pmp = pmpMgr_.accessPmp(pteAddr);
	  if (not pmp.isRead(privMode))
	    return accessFaultType(read, write, exec);
	}


      if (! memRead(pteAddr, bigEnd_, pte.data_))
        return accessFaultType(read, write, exec);
      if (not napotCheck(pte, va))
        return stage1PageFaultType(read, write, exec);

      // 3.
      if (not isValidPte(pte))
        return stage1PageFaultType(read, write, exec);

      // 4.
      global = global or pte.global();
      if (not pte.read() and not pte.exec())
        {  // pte is a pointer to the next level
	  if (pte.accessed() or pte.dirty() or pte.user() or pte.pbmt() != 0)
            return stage1PageFaultType(read, write, exec);  // A/D/U bits must be zero in non-leaf entries.
          ii = ii - 1;
          if (ii < 0)
            return stage1PageFaultType(read, write, exec);
          root = pte.ppn() * pageSize_;
          continue;  // goto 2.
        }

      // 5.  pte.read_ or pte.exec_ : leaf pte
      if (pbmtEnabled_)
	{
          if (trace_)
            walkVec.back().at(walkEntryIx).pbmt_ = static_cast<Pbmt>(pte.pbmt());
	  if (pte.pbmt() == 3)
	    return stage1PageFaultType(read, write, exec);  // pbmt=3 is reserved.
	}
      else if (pte.pbmt() != 0)
        return stage1PageFaultType(read, write, exec);  // Reserved pbmt bits must be 0.
      if (privMode == PrivilegeMode::User and not pte.user())
        return stage1PageFaultType(read, write, exec);
      if (privMode == PrivilegeMode::Supervisor and pte.user() and
	  (not sum_ or exec))
        return stage1PageFaultType(read, write, exec);

      bool pteRead = pte.read() or (execReadable_ and pte.exec());
      if ((read and not pteRead) or (write and not pte.write()) or
	  (exec and not pte.exec()))
        return stage1PageFaultType(read, write, exec);

      // 6.
      for (int j = 0; j < ii; ++j)
	if (pte.ppn(j) != 0)
          return stage1PageFaultType(read, write, exec);

      // 7.
      if (accessDirtyCheck_ and (not pte.accessed() or (write and not pte.dirty())))
	{
	  // We have a choice:
	  // A. Page fault
	  if (faultOnFirstAccess_)
	    return stage1PageFaultType(read, write, exec);  // A

	  // Or B
	  saveUpdatedPte(pteAddr, sizeof(pte.data_), pte.data_);  // For logging

	  // B1. Check PMP. The privMode here is the effective one that
	  // already accounts for MPRV.
	  if (pmpMgr_.isEnabled())
	    {
	      const Pmp& pmp = pmpMgr_.accessPmp(pteAddr);
	      if (not pmp.isWrite(privMode))
		return accessFaultType(read, write, exec);
	    }

	  {
	    // B2. Compare pte to memory.
	    PTE pte2(0);
	    memRead(pteAddr, bigEnd_, pte2.data_);
            if (not napotCheck(pte2, va))
              return stage1PageFaultType(read, write, exec);

	    if (pte.data_ != pte2.data_)
	      continue;  // Comparison fails: return to step 2.
	    pte.bits_.accessed_ = 1;
	    if (write)
	      pte.bits_.dirty_ = 1;

	    if (not memWrite(pteAddr, bigEnd_, pte.data_))
	      return stage1PageFaultType(read, write, exec);
	  }
	}
      break;
    }

  // 8.
  pa = va.offset();

  for (int j = 0; j < ii; ++j)
    pa = pa | (va.vpn(j) << pte.paPpnShift(j)); // Copy from va to pa

  for (unsigned j = ii; j < levels; ++j)
    pa = pa | pte.ppn(j) << pte.paPpnShift(j);

  if (trace_)
    walkVec.back().emplace_back(pa, WalkEntry::Type::RE);

  // Update tlb-entry with data found in page table entry.
  tlbEntry.virtPageNum_ = address >> pageBits_;
  tlbEntry.physPageNum_ = pa >> pageBits_;
  tlbEntry.asid_ = asid_;
  tlbEntry.valid_ = true;
  tlbEntry.global_ = global;
  tlbEntry.user_ = pte.user();
  tlbEntry.read_ = pte.read();
  tlbEntry.write_ = pte.write();
  tlbEntry.exec_ = pte.exec();
  tlbEntry.accessed_ = pte.accessed();
  tlbEntry.dirty_ = pte.dirty();
  tlbEntry.levels_ = 1+ii;
  tlbEntry.pbmt_ = pte.pbmt();

  pbmt_ = Pbmt(pte.pbmt());

  return ExceptionCause::NONE;
}


template<typename PTE, typename VA>
ExceptionCause
VirtMem::stage2PageTableWalk(uint64_t address, PrivilegeMode privMode, bool read, bool write,
			     bool exec, bool isPteAddr, uint64_t& pa, TlbEntry& tlbEntry)
{
  // 1. Root is "a" in section 4.3.2 of the privileged spec, ii is "i" in that section.
  uint64_t root = rootPageStage2_ * pageSize_;

  PTE pte(0);
  const unsigned levels = pte.levels();
  const unsigned pteSize = pte.size();
  int ii = levels - 1;

  VA va(address);

  // Collect PTE addresses used in the translation process.
  auto& walkVec = forFetch_ ? fetchWalks_ : dataWalks_;
  if (trace_)
    {
      walkVec.resize(walkVec.size() + 1);
      walkVec.back().emplace_back(address, WalkEntry::Type::GPA);
    }

  bool global = false;

  while (true)
    {
      // 2.
      uint64_t pteAddr = root + va.vpn(ii)*pteSize;

      int walkEntryIx = 0;
      if (trace_)
        {
          walkVec.back().emplace_back(pteAddr);
          walkEntryIx = walkVec.back().size() - 1;
        }

      // Check PMP. The privMode here is the effective one that
      // already accounts for MPRV.
      if (pmpMgr_.isEnabled())
	{
	  const Pmp& pmp = pmpMgr_.accessPmp(pteAddr);
	  if (not pmp.isRead(privMode))
	    return accessFaultType(read, write, exec);
	}

      if (! memRead(pteAddr, bigEnd_, pte.data_))
        return accessFaultType(read, write, exec);
      if (not napotCheck(pte, va))
        return stage2PageFaultType(read, write, exec);

      // 3.
      if (not isValidPte(pte))
        return stage2PageFaultType(read, write, exec);

      // 4.
      global = global or pte.global();
      if (not pte.read() and not pte.exec())
        {  // pte is a pointer to the next level
	  if (pte.accessed() or pte.dirty() or pte.user() or global or pte.pbmt() != 0)
            return stage2PageFaultType(read, write, exec);  // A/D/U/G bits must be zero in non-leaf entries.
          ii = ii - 1;
          if (ii < 0)
            return stage2PageFaultType(read, write, exec);
          root = pte.ppn() * pageSize_;
          continue;  // goto 2.
        }

      // 5.  pte.read_ or pte.exec_ : leaf pte
      if (pbmtEnabled_)
	{
          if (trace_)
            walkVec.back().at(walkEntryIx).pbmt_ = static_cast<Pbmt>(pte.pbmt());
	  if (pte.pbmt() == 3)
	    return stage2PageFaultType(read, write, exec);  // pbmt=3 is reserved.
	}
      else if (pte.pbmt() != 0)
        return stage2PageFaultType(read, write, exec);  // Reserved pbmt bits must be 0.
      if (not pte.user())
        return stage2PageFaultType(read, write, exec);  // All access as though in User mode.

      bool pteRead = pte.read() or (execReadable_ and pte.exec() and not isPteAddr);
      if (not isPteAddr and xForR_) // xForR_ (hlvx) has no effect when translating for a PTE addr
	pteRead = pte.exec();
      if ((read and not pteRead) or (write and not pte.write()) or
	  (exec and not pte.exec()))
        return stage2PageFaultType(read, write, exec);

      // 6.
      for (int j = 0; j < ii; ++j)
	if (pte.ppn(j) != 0)
          return stage2PageFaultType(read, write, exec);

      // 7.
      if (accessDirtyCheck_ and (not pte.accessed() or (write and not pte.dirty())))
	{
	  // We have a choice:
	  // A. Page fault
	  if (faultOnFirstAccess2_)
	    return stage2PageFaultType(read, write, exec);  // A

	  // Or B
	  saveUpdatedPte(pteAddr, sizeof(pte.data_), pte.data_);  // For logging

	  // B1. Check PMP. The privMode here is the effective one that
	  // already accounts for MPRV.
	  if (pmpMgr_.isEnabled())
	    {
	      const Pmp& pmp = pmpMgr_.accessPmp(pteAddr);
	      if (not pmp.isWrite(privMode))
		return accessFaultType(read, write, exec);
	    }

	  {
	    // B2. Compare pte to memory.
	    PTE pte2(0);
	    memRead(pteAddr, bigEnd_, pte2.data_);
            if (not napotCheck(pte2, va))
              return stage2PageFaultType(read, write, exec);

	    if (pte.data_ != pte2.data_)
	      continue;  // Comparison fails: return to step 2.
	    pte.bits_.accessed_ = 1;
	    if (write)
	      pte.bits_.dirty_ = 1;

	    if (not memWrite(pteAddr, bigEnd_, pte.data_))
	      return stage2PageFaultType(read, write, exec);
	  }
	}
      break;
    }

  // 8.
  pa = va.offset();

  for (int j = 0; j < ii; ++j)
    pa = pa | (va.vpn(j) << pte.paPpnShift(j)); // Copy from va to pa

  for (unsigned j = ii; j < levels; ++j)
    pa = pa | pte.ppn(j) << pte.paPpnShift(j);

  if (trace_)
    walkVec.back().emplace_back(pa, WalkEntry::Type::RE);

  // Update tlb-entry with data found in page table entry.
  tlbEntry.virtPageNum_ = address >> pageBits_;
  tlbEntry.physPageNum_ = pa >> pageBits_;
  tlbEntry.asid_ = asid_;
  tlbEntry.vmid_ = vmid_;
  tlbEntry.valid_ = true;
  tlbEntry.global_ = global;
  tlbEntry.user_ = pte.user();
  tlbEntry.read_ = pte.read();
  tlbEntry.write_ = pte.write();
  tlbEntry.exec_ = pte.exec();
  tlbEntry.accessed_ = pte.accessed();
  tlbEntry.dirty_ = pte.dirty();
  tlbEntry.levels_ = 1+ii;
  tlbEntry.pbmt_ = pte.pbmt();

  pbmt_ = Pbmt(pte.pbmt());

  return ExceptionCause::NONE;
}


template<typename PTE, typename VA>
ExceptionCause
VirtMem::stage1PageTableWalk(uint64_t address, PrivilegeMode privMode, bool read, bool write,
			     bool exec, uint64_t& pa, TlbEntry& tlbEntry)
{
  // 1. Root is "a" in section 4.3.2 of the privileged spec, ii is "i" in that section.
  uint64_t root = vsRootPage_ * pageSize_;

  PTE pte(0);
  const unsigned levels = pte.levels();
  const unsigned pteSize = pte.size();
  int ii = levels - 1;

  VA va(address);

  // Collect PTE addresses used in the translation process.
  auto& walkVec = forFetch_ ? fetchWalks_ : dataWalks_;
  if (trace_)
    {
      walkVec.resize(walkVec.size() + 1);
      walkVec.back().emplace_back(address, WalkEntry::Type::GVA);
    }

  bool global = false;

  while (true)
    {
      // 2.
      uint64_t gpteAddr = root + va.vpn(ii)*pteSize; // Guest pte address.

      if (trace_)
        walkVec.back().emplace_back(gpteAddr, WalkEntry::Type::GPA);

      // Translate guest pteAddr to host physical address.
      uint64_t pteAddr = gpteAddr; pa = gpteAddr;
      auto ec = stage2Translate(gpteAddr, privMode, true, false, false, /* isPteAddr */ true, pteAddr);
      if (ec != ExceptionCause::NONE)
	return stage2ExceptionToStage1(ec, read, write, exec);

      int walkEntryIx = 0;
      if (trace_)
        {
          walkVec.back().emplace_back(pteAddr);
          walkEntryIx = walkVec.back().size() - 1;
        }

      // Check PMP. The privMode here is the effective one that
      // already accounts for MPRV.
      if (pmpMgr_.isEnabled())
	{
	  const Pmp& pmp = pmpMgr_.accessPmp(pteAddr);
	  if (not pmp.isRead(privMode))
	    return accessFaultType(read, write, exec);
	}

      if (! memRead(pteAddr, bigEnd_, pte.data_))
        return accessFaultType(read, write, exec);
      if (not napotCheck(pte, va))
        return stage1PageFaultType(read, write, exec);

      // 3.
      if (not isValidPte(pte))
        return stage1PageFaultType(read, write, exec);

      // 4.
      global = global or pte.global();
      if (not pte.read() and not pte.exec())
        {  // pte is a pointer to the next level
	  if (pte.accessed() or pte.dirty() or pte.user() or pte.pbmt() != 0)
            return stage1PageFaultType(read, write, exec);  // A/D/U bits must be zero in non-leaf entries.
          ii = ii - 1;
          if (ii < 0)
            return stage1PageFaultType(read, write, exec);
          root = pte.ppn() * pageSize_;
          continue;  // goto 2.
        }

      // 5.  pte.read_ or pte.exec_ : leaf pte
      if (vsPbmtEnabled_)
	{
          if (trace_)
            walkVec.back().at(walkEntryIx).pbmt_ = static_cast<Pbmt>(pte.pbmt());
	  if (pte.pbmt() == 3)
	    return stage1PageFaultType(read, write, exec);  // pbmt=3 is reserved.
	}
      else if (pte.pbmt() != 0)
        return stage1PageFaultType(read, write, exec);  // Reserved pbmt bits must be 0.
      if (privMode == PrivilegeMode::User and not pte.user())
        return stage1PageFaultType(read, write, exec);
      if (privMode == PrivilegeMode::Supervisor and pte.user() and
	  (not vsSum_ or exec))
        return stage1PageFaultType(read, write, exec);

      bool pteRead = pte.read() or ((execReadable_ or s1ExecReadable_) and pte.exec());
      if (xForR_)
	pteRead = pte.exec();
      if ((read and not pteRead) or (write and not pte.write()) or
	  (exec and not pte.exec()))
        return stage1PageFaultType(read, write, exec);

      // 6.
      for (int j = 0; j < ii; ++j)
	if (pte.ppn(j) != 0)
          return stage1PageFaultType(read, write, exec);

      // 7.
      if (accessDirtyCheck_ and (not pte.accessed() or (write and not pte.dirty())))
	{
	  // We have a choice:
	  // A. Page fault (if configured or, if page of PTE is non-cachable or is io).
	  //       pbmt_ is that of leaf page of g-stage translation.
	  if (faultOnFirstAccess1_ or pbmt_ != Pbmt::None)
            return stage1PageFaultType(read, write, exec);  // A

	  // Or B
	  saveUpdatedPte(pteAddr, sizeof(pte.data_), pte.data_);  // For logging

          s1ADUpdate_ = true;
	  // B1. Check PMP. The privMode here is the effective one that
	  // already accounts for MPRV.
	  if (pmpMgr_.isEnabled())
	    {
	      const Pmp& pmp = pmpMgr_.accessPmp(pteAddr);
	      if (not pmp.isWrite(privMode))
		return accessFaultType(read, write, exec);
	    }

	  {
	    // B2. Compare pte to memory.
	    PTE pte2(0);
	    memRead(pteAddr, bigEnd_, pte2.data_);
            if (not napotCheck(pte2, va))
              return stage1PageFaultType(read, write, exec);

	    if (pte.data_ != pte2.data_)
	      continue;  // Comparison fails: return to step 2.
	    pte.bits_.accessed_ = 1;
	    if (write)
	      pte.bits_.dirty_ = 1;

	    // Need to make sure we have write access to page.
	    uint64_t pteAddr2 = gpteAddr; pa = gpteAddr;
	    auto ec = stage2Translate(gpteAddr, privMode, false, true, false, /* isPteAddr */ false, pteAddr2);
	    if (ec != ExceptionCause::NONE)
	      return stage2ExceptionToStage1(ec, read, write, exec);
	    assert(pteAddr == pteAddr2);
	    if (not memWrite(pteAddr2, bigEnd_, pte.data_))
	      return stage1PageFaultType(read, write, exec);
	  }
	}
      break;
    }

  // 8.
  pa = va.offset();

  for (int j = 0; j < ii; ++j)
    pa = pa | (va.vpn(j) << pte.paPpnShift(j)); // Copy from va to pa

  for (unsigned j = ii; j < levels; ++j)
    pa = pa | pte.ppn(j) << pte.paPpnShift(j);

  if (trace_)
    walkVec.back().emplace_back(pa, WalkEntry::Type::RE);

  // Update tlb-entry with data found in page table entry.
  tlbEntry.virtPageNum_ = address >> pageBits_;
  tlbEntry.physPageNum_ = pa >> pageBits_;
  tlbEntry.asid_ = vsAsid_;
  tlbEntry.vmid_ = vmid_;
  tlbEntry.valid_ = true;
  tlbEntry.global_ = global;
  tlbEntry.user_ = pte.user();
  tlbEntry.read_ = pte.read();
  tlbEntry.write_ = pte.write();
  tlbEntry.exec_ = pte.exec();
  tlbEntry.accessed_ = pte.accessed();
  tlbEntry.dirty_ = pte.dirty();
  tlbEntry.levels_ = 1+ii;
  tlbEntry.pbmt_ = pte.pbmt();

  vsPbmt_ = Pbmt(pte.pbmt());

  return ExceptionCause::NONE;
}

bool
VirtMem::setPageSize(uint64_t size)
{
  if (size == 0)
    return false;

  unsigned bits = static_cast<unsigned>(std::log2(pageSize_));
  uint64_t p2Size =  uint64_t(1) << bits;

  if (size != p2Size)
    return false;
  
  pageBits_ = bits;
  pageSize_ = size;
  return true;
}


void
VirtMem::printPageTable(std::ostream& os) const
{
  std::ios_base::fmtflags flags(os.flags());

  os << "Page size: " << std::dec << pageSize_ << '\n';
  os << "Mode: ";
  switch(mode_)
    {
    case Mode::Bare: os << "Bare\n"; break;
    case Mode::Sv32: os << "Sv32\n"; break;
    case Mode::Sv39: os << "Sv39\n"; break;
    case Mode::Sv48: os << "Sv48\n"; break;
    case Mode::Sv57: os << "Sv57\n"; break;
    case Mode::Sv64: os << "Sv64\n"; break;
    default:   os << "???\n";  break;
    }

  os << "Root page number: 0x" << std::hex << rootPage_ << std::dec << '\n';
  uint64_t addr = rootPage_ * pageSize_;
  os << "Root page addr: 0x" << std::hex << addr << std::dec << '\n';

  std::string path = "/";

  if (mode_ == Mode::Bare)
    ;  // relax
  else if (mode_ == Mode::Sv32)
    printEntries<Pte32, Va32>(os, addr, path);
  else if (mode_ == Mode::Sv39)
    printEntries<Pte39, Va39>(os, addr, path);
  else if (mode_ == Mode::Sv48)
    printEntries<Pte48, Va48>(os, addr, path);
  else if (mode_ == Mode::Sv57)
    printEntries<Pte57, Va57>(os, addr, path);
  else
    os << "Unsupported virtual memory mode\n";
  os << "TLB:\n";
  tlb_.printTlb(os);
  os.flags(flags);
}


template<typename PTE, typename VA>
void
VirtMem::printEntries(std::ostream& os, uint64_t addr, const std::string& path) const
{
  os << "\n";
  os << "Page table page addr: 0x" << std::hex << addr << std::dec << '\n';
  os << "Path: " << path << '\n';

  unsigned entrySize = sizeof(PTE);
  unsigned entryCount = pageSize() / entrySize;

  uint64_t eaddr = addr;  // Entry address
  for (unsigned ix = 0; ix < entryCount; ++ix, eaddr += entrySize)
    {
      PTE pte(0);
      memRead(eaddr, false /*bigEndian*/, pte.data_);

      if (not pte.valid())
        continue;

      bool leaf = pte.valid() and (pte.read() or pte.exec());
      os << "  ix:" << std::dec << ix << " addr:0x" << std::hex << eaddr
         << " data:0x" << std::hex << pte.data_ 
         << " rwx:" << pte.read() << pte.write() << pte.exec()
         << " leaf:" << leaf << " pa:0x" << (pte.ppn() * pageSize_) << std::dec << '\n';
    }

  eaddr = addr;
  for (unsigned ix = 0; ix < entryCount; ++ix, eaddr += entrySize)
    {
      PTE pte(0);
      memRead(eaddr, false /*bigEnding*/, pte.data_);

      if (not pte.valid())
        continue;

      bool leaf = pte.valid() and (pte.read() or pte.exec());
      if (leaf)
        continue;

      std::string nextPath;
      if (path == "/")
        nextPath = path + std::to_string(ix);
      else
        nextPath = path + "/" + std::to_string(ix);

      uint64_t nextAddr = pte.ppn() * pageSize_;
      printEntries<PTE, VA>(os, nextAddr, nextPath);
    }
}
