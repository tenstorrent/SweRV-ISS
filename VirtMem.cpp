#include <cmath>
#include <iostream>
#include <ios>
#include "PmpManager.hpp"
#include "VirtMem.hpp"
#include "inttypes.h"

using namespace WdRiscv;


VirtMem::VirtMem(unsigned hartIx, Memory& memory, unsigned pageSize,
                 PmpManager& pmpMgr, unsigned tlbSize)
  : memory_(memory), pageSize_(pageSize), hartIx_(hartIx),
    pmpMgr_(pmpMgr), tlb_(tlbSize), vsTlb_(tlbSize), stage2Tlb_(tlbSize)
{
  pageBits_ = static_cast<unsigned>(std::log2(pageSize_));
  unsigned p2PageSize =  unsigned(1) << pageBits_;

  assert(p2PageSize == pageSize);
  assert(pageSize >= 64);

  pageMask_ = pageSize_ - 1;
}


/// Page fault type for read/write/exec access (one and only one of
/// which must be true). This is for non V (S, HS or U) translation.
static
ExceptionCause
nvPageFaultType(bool read, bool write, bool exec)
{
  if (exec)  return ExceptionCause::INST_PAGE_FAULT;
  if (read)  return ExceptionCause::LOAD_PAGE_FAULT;
  if (write) return ExceptionCause::STORE_PAGE_FAULT;
  assert(0);
  return ExceptionCause::STORE_PAGE_FAULT;
}


/// Page fault type for read/write/exec access (one and only one of
/// which must be true). This is for V (VS or VU) translation.
static
ExceptionCause
vPageFaultType(bool read, bool write, bool exec)
{
  if (exec)  return ExceptionCause::INST_GUEST_PAGE_FAULT;
  if (read)  return ExceptionCause::LOAD_GUEST_PAGE_FAULT;
  if (write) return ExceptionCause::STORE_GUEST_PAGE_FAULT;
  assert(0);
  return ExceptionCause::STORE_GUEST_PAGE_FAULT;
}


static
ExceptionCause
stage2ExceptionToStage1(ExceptionCause ec2, bool read, bool write, bool exec)
{
  typedef ExceptionCause EC;
  if (ec2 == EC::INST_GUEST_PAGE_FAULT or ec2 == EC::LOAD_GUEST_PAGE_FAULT or
      ec2 == EC::STORE_GUEST_PAGE_FAULT)
    return nvPageFaultType(read, write, exec);
  return ec2;
}


/// Page fault type for read/write/exec access (one and only one of
/// which must be true).
static
ExceptionCause
pageFaultType(bool twoStage, bool read, bool write, bool exec)
{
  if (twoStage)
    return vPageFaultType(read, write, exec);
  return nvPageFaultType(read, write, exec);
}


inline
ExceptionCause
accessFaultType(bool read, bool write, bool exec)
{
  if (exec)  return ExceptionCause::INST_ACC_FAULT;
  if (read)  return ExceptionCause::LOAD_ACC_FAULT;
  if (write) return ExceptionCause::STORE_ACC_FAULT;
  assert(0);
  return ExceptionCause::LOAD_ACC_FAULT;
}


ExceptionCause
VirtMem::translateForFetch2(uint64_t va, unsigned size, PrivilegeMode priv,
			    bool twoStage, uint64_t& pa1, uint64_t& pa2)
{
  pa1 = pa2 = va;
  auto cause = translateForFetch(va, priv, twoStage, pa1);
  if (cause != ExceptionCause::NONE)
    return cause;

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
  cause = translateForFetch(va2, priv, twoStage, pa2);
  if (cause != ExceptionCause::NONE)
    pa1 = pa2 = va2;

  return cause;
}


ExceptionCause
VirtMem::transAddrNoUpdate(uint64_t va, PrivilegeMode priv, bool twoStage,
			   bool read, bool write, bool exec, uint64_t& pa)
{
  auto prevTrace = trace_; trace_ = false;
  auto prevFile  = attFile_; attFile_ = nullptr;
  accessDirtyCheck_ = false;

  auto cause = transNoUpdate(va, priv, twoStage, read, write, exec, pa);

  trace_ = prevTrace;
  attFile_ = prevFile;
  accessDirtyCheck_ = true;

  return cause;
}


ExceptionCause
VirtMem::transNoUpdate(uint64_t va, PrivilegeMode priv, bool twoStage,
		       bool read, bool write, bool exec, uint64_t& pa)
{
  // Exactly one of read/write/exec must be true.
  unsigned count = 0;
  if (read) count++;
  if (write) count++;
  if (exec) count++;
  assert(count == 1);

  if (mode_ == Bare)
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
        return pageFaultType(twoStage, read, write, exec);
      if (priv == PrivilegeMode::Supervisor)
	if (entry->user_ and (exec or not sum_))
	  return pageFaultType(twoStage, read, write, exec);
      bool ra = entry->read_ or (execReadable_ and entry->exec_);
      bool wa = entry->write_, xa = entry->exec_;
      if ((read and not ra) or (write and not wa) or (exec and not xa))
        return pageFaultType(twoStage, read, write, exec);
      // We do not check/update access/dirty bits.
      pa = (entry->physPageNum_ << pageBits_) | (va & pageMask_);
      return ExceptionCause::NONE;
    }

  TlbEntry tlbEntry;
  return translateNoTlb(va, priv, twoStage, read, write, exec, pa, tlbEntry);
}


ExceptionCause
VirtMem::translateForLdSt2(uint64_t va, unsigned size, PrivilegeMode priv,
                           bool twoStage, bool load, uint64_t& pa1, uint64_t& pa2)
{
  pa1 = pa2 = va;

  bool read = load, write = not load, exec = false;
  auto cause = translate(va, priv, twoStage, read, write, exec, pa1);
  if (cause != ExceptionCause::NONE)
    return cause;

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
  cause = translate(va2, priv, twoStage, read, write, exec, pa2);
  if (cause != ExceptionCause::NONE)
    pa1 = pa2 = va2;

  return cause;
}


ExceptionCause
VirtMem::translate(uint64_t va, PrivilegeMode priv, bool twoStage,
		   bool read, bool write, bool exec, uint64_t& pa)
{
  if (twoStage)
    return twoStageTranslate(va, priv, read, write, exec, pa);

  // Exactly one of read/write/exec must be true.
  unsigned count = 0;
  if (read) count++;
  if (write) count++;
  if (exec) count++;
  assert(count == 1);

  if (mode_ == Bare)
    {
      pa = va;
      return ExceptionCause::NONE;
    }

  // Lookup virtual page number in TLB.
  uint64_t virPageNum = va >> pageBits_;
  TlbEntry* entry = tlb_.findEntryUpdateTime(virPageNum, asid_);
  if (entry)
    {
      // Use TLB entry.
      if (priv == PrivilegeMode::User and not entry->user_)
        return pageFaultType(twoStage, read, write, exec);
      if (priv == PrivilegeMode::Supervisor)
	if (entry->user_ and (exec or not sum_))
	  return pageFaultType(twoStage, read, write, exec);
      bool ra = entry->read_ or (execReadable_ and entry->exec_);
      bool wa = entry->write_, xa = entry->exec_;
      if ((read and not ra) or (write and not wa) or (exec and not xa))
        return pageFaultType(twoStage, read, write, exec);
      if (not entry->accessed_ or (write and not entry->dirty_))
        {
          if (faultOnFirstAccess_)
            return pageFaultType(twoStage, read, write, exec);
          entry->accessed_ = true;
          if (write)
            entry->dirty_ = true;
        }
      pa = (entry->physPageNum_ << pageBits_) | (va & pageMask_);
      return ExceptionCause::NONE;
    }

  TlbEntry tlbEntry;
  auto cause = translateNoTlb(va, priv, twoStage, read, write, exec, pa, tlbEntry);

  // If successful, put translation results in TLB.
  if (cause == ExceptionCause::NONE)
    tlb_.insertEntry(tlbEntry);

  return cause;
}


ExceptionCause
VirtMem::translateNoTlb(uint64_t va, PrivilegeMode priv, bool twoStage, bool read,
			bool write, bool exec, uint64_t& pa, TlbEntry& entry)
{
  // Perform a page table walk.
  ExceptionCause cause = ExceptionCause::LOAD_PAGE_FAULT;

  if (mode_ == Sv32)
    cause = pageTableWalk1p12<Pte32, Va32>(va, priv, read, write, exec, pa, entry);
  else if (mode_ == Sv39)
    {
      // Part 1 of address translation: Bits 63-39 must equal bit 38
      uint64_t mask = (va >> 38) & 1;
      if (mask)
        mask = 0x1ffffff;  // Least sig 25 bits set
      if ((va >> 39) != mask)
        return pageFaultType(twoStage, read, write, exec);
      cause = pageTableWalk1p12<Pte39, Va39>(va, priv, read, write, exec, pa, entry);
    }
  else if (mode_ == Sv48)
    {
      // Part 1 of address translation: Bits 63-47 must equal bit 47
      uint64_t mask = (va >> 47) & 1;
      if (mask)
        mask = 0xffff;  // Least sig 16 bits set
      if ((va >> 48) != mask)
        return pageFaultType(twoStage, read, write, exec);
      cause = pageTableWalk1p12<Pte48, Va48>(va, priv, read, write, exec, pa, entry);
    }
  else if (mode_ == Sv57)
    {
      // Part 1 of address translation: Bits 63-57 muse equal bit 56
      uint64_t mask = (va >> 56) & 1;
      if (mask)
        mask = 0x7f;  // Least sig 7 bits set
      if ((va >> 57) != mask)
        return pageFaultType(twoStage, read, write, exec);
      cause = pageTableWalk1p12<Pte57, Va57>(va, priv, read, write, exec, pa, entry);
    }
  else
    assert(0 and "Unspupported virtual memory mode.");

  return cause;
}


ExceptionCause
VirtMem::stage2TranslateNoTlb(uint64_t va, PrivilegeMode priv, bool read,
			      bool write, bool exec, uint64_t& pa, TlbEntry& entry)
{
  // Perform a page table walk.
  ExceptionCause cause = ExceptionCause::LOAD_PAGE_FAULT;

  if (modeStage2_ == Sv32)
    {
      // Part 1 of address translation: Bits 63-34 must be zero
      if ((va >> 34) != 0)
	return vPageFaultType(read, write, exec);
      cause = stage2PageTableWalk<Pte32, Va32x4>(va, priv, read, write, exec, pa, entry);
    }
  else if (modeStage2_ == Sv39)
    {
      // Part 1 of address translation: Bits 63-41 must be zero
      if ((va >> 41) != 0)
        return vPageFaultType(read, write, exec);
      cause = stage2PageTableWalk<Pte39, Va39x4>(va, priv, read, write, exec, pa, entry);
    }
  else if (modeStage2_ == Sv48)
    {
      // Part 1 of address translation: Bits 63-50 must be zero
      if ((va >> 50) != 0)
        return vPageFaultType(read, write, exec);
      cause = stage2PageTableWalk<Pte48, Va48x4>(va, priv, read, write, exec, pa, entry);
    }
  else if (modeStage2_ == Sv57)
    {
      // Part 1 of address translation: Bits 63-59 must be zero
      if ((va >> 59) != 0)
        return vPageFaultType(read, write, exec);
      cause = stage2PageTableWalk<Pte57, Va57x4>(va, priv, read, write, exec, pa, entry);
    }
  else
    assert(0 and "Unspupported virtual memory mode.");

  return cause;
}


ExceptionCause
VirtMem::stage2Translate(uint64_t va, PrivilegeMode priv, bool read, bool write,
			 bool exec, uint64_t& pa)
{
  // Exactly one of read/write/exec must be true.
  unsigned count = 0;
  if (read) count++;
  if (write) count++;
  if (exec) count++;
  assert(count == 1);

  if (modeStage2_ == Bare)
    {
      pa = va;
      return ExceptionCause::NONE;
    }

  // Lookup virtual page number in TLB.
  uint64_t virPageNum = va >> pageBits_;
  TlbEntry* entry = stage2Tlb_.findEntryUpdateTime(virPageNum, vsAsid_); // FIX: vmid_ insteadm of vsAsid?
  if (entry)
    {
      // Use TLB entry.
      if (not entry->user_)
	return vPageFaultType(read, write, exec);
      bool ra = entry->read_ or (execReadable_ and entry->exec_);
      if (xForR_)
	ra = entry->exec_;
      bool wa = entry->write_, xa = entry->exec_;
      if ((read and not ra) or (write and not wa) or (exec and not xa))
        return vPageFaultType(read, write, exec);
      if (not entry->accessed_ or (write and not entry->dirty_))
        {
          if (faultOnFirstAccess_)
            return vPageFaultType(read, write, exec);
          entry->accessed_ = true;
          if (write)
            entry->dirty_ = true;
        }
      pa = (entry->physPageNum_ << pageBits_) | (va & pageMask_);
      return ExceptionCause::NONE;
    }

  TlbEntry tlbEntry;
  auto cause = stage2TranslateNoTlb(va, priv, read, write, exec, pa, tlbEntry);

  // If successful, put translation results in TLB.
  if (cause == ExceptionCause::NONE)
    stage2Tlb_.insertEntry(tlbEntry);

  return cause;
}


ExceptionCause
VirtMem::twoStageTranslate(uint64_t va, PrivilegeMode priv, bool read, bool write,
			   bool exec, uint64_t& pa)
{
  // Exactly one of read/write/exec must be true.
  unsigned count = 0;
  if (read) count++;
  if (write) count++;
  if (exec) count++;
  assert(count == 1);

  uint64_t gpa = 0;
  if (vsMode_ == Bare)
    gpa = va;
  else
    {
      // Lookup virtual page number in TLB.
      uint64_t virPageNum = va >> pageBits_;
      TlbEntry* entry = vsTlb_.findEntryUpdateTime(virPageNum, vsAsid_);
      if (entry)
	{
	  // Use TLB entry.
	  if (priv == PrivilegeMode::User and not entry->user_)
	    return vPageFaultType(read, write, exec);
	  if (priv == PrivilegeMode::Supervisor)
	    if (entry->user_ and (exec or not vsSum_))
	      return vPageFaultType(read, write, exec);
	  bool ra = entry->read_ or ((execReadable_ or s1ExecReadable_) and entry->exec_);
	  if (xForR_)
	    ra = entry->exec_;
	  bool wa = entry->write_, xa = entry->exec_;
	  if ((read and not ra) or (write and not wa) or (exec and not xa))
	    return vPageFaultType(read, write, exec);
	  if (not entry->accessed_ or (write and not entry->dirty_))
	    {
	      if (faultOnFirstAccess_)
		return vPageFaultType(read, write, exec);
	      entry->accessed_ = true;
	      if (write)
		entry->dirty_ = true;
	    }
	  gpa = (entry->physPageNum_ << pageBits_) | (va & pageMask_);
	}
      else
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

  return stage2Translate(gpa, priv, read, write, exec, pa);
}


ExceptionCause
VirtMem::stage1TranslateNoTlb(uint64_t va, PrivilegeMode priv, bool read, bool write,
			      bool exec, uint64_t& pa, TlbEntry& entry)
{
  if (vsMode_ == Sv32)
    return stage1PageTableWalk<Pte32, Va32>(va, priv, read, write, exec, pa, entry);

  if (vsMode_ == Sv39)
    {
      // Part 1 of address translation: Bits 63-39 must equal bit 38
      uint64_t mask = (va >> 38) & 1;
      if (mask)
        mask = 0x1ffffff;  // Least sig 25 bits set
      if ((va >> 39) != mask)
        return vPageFaultType(read, write, exec);
      return stage1PageTableWalk<Pte39, Va39>(va, priv, read, write, exec, pa, entry);
    }

  if (vsMode_ == Sv48)
    {
      // Part 1 of address translation: Bits 63-47 must equal bit 47
      uint64_t mask = (va >> 47) & 1;
      if (mask)
        mask = 0xffff;  // Least sig 16 bits set
      if ((va >> 48) != mask)
        return vPageFaultType(read, write, exec);
      return stage1PageTableWalk<Pte48, Va48>(va, priv, read, write, exec, pa, entry);
    }

  if (vsMode_ == Sv57)
    {
      // Part 1 of address translation: Bits 63-57 muse equal bit 56
      uint64_t mask = (va >> 56) & 1;
      if (mask)
        mask = 0x7f;  // Least sig 7 bits set
      if ((va >> 57) != mask)
        return vPageFaultType(read, write, exec);
      return stage1PageTableWalk<Pte57, Va57>(va, priv, read, write, exec, pa, entry);
    }

  assert(0 and "Unspupported virtual memory mode.");
  return ExceptionCause::LOAD_GUEST_PAGE_FAULT;
}


template<typename PTE, typename VA>
ExceptionCause
VirtMem::pageTableWalk1p12(uint64_t address, PrivilegeMode privMode, bool read, bool write,
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
    walkVec.resize(walkVec.size() + 1);

  if (attFile_)
    fprintf(attFile_, "VA: 0x%jx\n", uintmax_t(address));

  while (true)
    {
      // 2.
      uint64_t pteAddr = root + va.vpn(ii)*pteSize;
      if (trace_)
	walkVec.back().push_back(pteAddr);

      // Check PMP. The privMode here is the effective one that
      // already accounts for MPRV.
      if (pmpMgr_.isEnabled())
	{
	  Pmp pmp = pmpMgr_.accessPmp(pteAddr);
	  if (not pmp.isRead(privMode, privMode, false))
	    return accessFaultType(read, write, exec);
	}

      if (! memory_.read(pteAddr, pte.data_))
        return nvPageFaultType(read, write, exec);

      if (attFile_)
        {
          bool leaf = pte.valid() and (pte.read() or pte.exec());
          fprintf(attFile_, "addr: 0x%jx\n", uintmax_t(pteAddr));
          fprintf(attFile_, "rwx: %d%d%d, ug: %d%d, ad: %d%d\n", pte.read(),
		  pte.write(), pte.exec(), pte.user(), pte.global(),
		  pte.accessed(), pte.dirty());
          fprintf(attFile_, "leaf: %d, pa:0x%jx", leaf,
		  uintmax_t(pte.ppn()) * pageSize_);
	  if (leaf)
            fprintf(attFile_, " s:%s", pageSize(mode_, ii));
	  fprintf(attFile_, "\n\n");
        }

      // 3.
      if (not pte.valid() or (not pte.read() and pte.write()) or pte.res())
        return nvPageFaultType(read, write, exec);

      // 4.
      if (not pte.read() and not pte.exec())
        {  // pte is a pointer to the next level
	  if (pte.accessed() or pte.dirty() or pte.user() or pte.pbmt() != 0)
	    return nvPageFaultType(read, write, exec);  // A/D/U bits must be zero in non-leaf entries.
          ii = ii - 1;
          if (ii < 0)
            return nvPageFaultType(read, write, exec);
          root = pte.ppn() * pageSize_;
          continue;  // goto 2.
        }

      // 5.  pte.read_ or pte.exec_ : leaf pte
      if (pte.pbmt() != 0)
	return nvPageFaultType(read, write, exec);  // Leaf page must bave pbmt=0.
      if (privMode == PrivilegeMode::User and not pte.user())
	return nvPageFaultType(read, write, exec);
      if (privMode == PrivilegeMode::Supervisor and pte.user() and
	  (not sum_ or exec))
	return nvPageFaultType(read, write, exec);

      bool pteRead = pte.read() or (execReadable_ and pte.exec());
      if ((read and not pteRead) or (write and not pte.write()) or
	  (exec and not pte.exec()))
	return nvPageFaultType(read, write, exec);

      // 6.
      for (int j = 0; j < ii; ++j)
	if (pte.ppn(j) != 0)
	  return nvPageFaultType(read, write, exec);

      // 7.
      if (accessDirtyCheck_ and (not pte.accessed() or (write and not pte.dirty())))
	{
	  // We have a choice:
	  // A. Page fault
	  if (faultOnFirstAccess_)
	    return nvPageFaultType(read, write, exec);  // A

	  // Or B
	  saveUpdatedPte(pteAddr, sizeof(pte.data_), pte.data_);  // For logging

	  // B1. Check PMP. The privMode here is the effective one that
	  // already accounts for MPRV.
	  if (pmpMgr_.isEnabled())
	    {
	      Pmp pmp = pmpMgr_.accessPmp(pteAddr);
	      if (not pmp.isWrite(privMode, privMode, false))
		return accessFaultType(read, write, exec);
	    }

	  {
	    // TODO FIX : this has to be atomic
	    // B2. Compare pte to memory.
	    PTE pte2(0);
	    memory_.read(pteAddr, pte2.data_);
	    if (pte.data_ != pte2.data_)
	      continue;  // Comparison fails: return to step 2.
	    pte.bits_.accessed_ = 1;
	    if (write)
	      pte.bits_.dirty_ = 1;

	    if (not memory_.write(hartIx_, pteAddr, pte.data_))
	      return nvPageFaultType(read, write, exec);
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

  // Update tlb-entry with data found in page table entry.
  tlbEntry.virtPageNum_ = address >> pageBits_;
  tlbEntry.physPageNum_ = pa >> pageBits_;
  tlbEntry.asid_ = asid_;
  tlbEntry.valid_ = true;
  tlbEntry.global_ = pte.global();
  tlbEntry.user_ = pte.user();
  tlbEntry.read_ = pte.read();
  tlbEntry.write_ = pte.write();
  tlbEntry.exec_ = pte.exec();
  tlbEntry.accessed_ = pte.accessed();
  tlbEntry.dirty_ = pte.dirty();
  tlbEntry.levels_ = 1+ii;
  return ExceptionCause::NONE;
}


template<typename PTE, typename VA>
ExceptionCause
VirtMem::stage2PageTableWalk(uint64_t address, PrivilegeMode privMode, bool read, bool write,
			     bool exec, uint64_t& pa, TlbEntry& tlbEntry)
{
  // 1. Root is "a" in section 4.3.2 of the privileged spec, ii is "i" in that section.
  uint64_t root = rootPageStage2_ * pageSize_;

  PTE pte(0);
  const unsigned levels = pte.levels();
  const unsigned pteSize = pte.size();
  int ii = levels - 1;

  VA va(address);

  // Collect PTE addresses used in the translation process.
  auto& walkVec = exec ? fetchWalks_ : dataWalks_;
  if (trace_)
    walkVec.resize(walkVec.size() + 1);

  if (attFile_)
    fprintf(attFile_, "GPA: 0x%jx\n", uintmax_t(address));

  while (true)
    {
      // 2.
      uint64_t pteAddr = root + va.vpn(ii)*pteSize;
      if (trace_)
	walkVec.back().push_back(pteAddr);

      // Check PMP. The privMode here is the effective one that
      // already accounts for MPRV.
      if (pmpMgr_.isEnabled())
	{
	  Pmp pmp = pmpMgr_.accessPmp(pteAddr);
	  if (not pmp.isRead(privMode, privMode, false))
	    return accessFaultType(read, write, exec);
	}

      if (! memory_.read(pteAddr, pte.data_))
        return vPageFaultType(read, write, exec);

      if (attFile_)
        {
          bool leaf = pte.valid() and (pte.read() or pte.exec());
          fprintf(attFile_, "addr: 0x%jx\n", uintmax_t(pteAddr));
          fprintf(attFile_, "rwx: %d%d%d, ug: %d%d, ad: %d%d\n", pte.read(),
		  pte.write(), pte.exec(), pte.user(), pte.global(),
		  pte.accessed(), pte.dirty());
          fprintf(attFile_, "leaf: %d, pa:0x%jx", leaf,
		  uintmax_t(pte.ppn()) * pageSize_);
	  if (leaf)
            fprintf(attFile_, " s:%s", pageSize(modeStage2_, ii));
	  fprintf(attFile_, "\n\n");
        }

      // 3.
      if (not pte.valid() or (not pte.read() and pte.write()) or pte.res())
        return vPageFaultType(read, write, exec);

      // 4.
      if (not pte.read() and not pte.exec())
        {  // pte is a pointer to the next level
	  if (pte.accessed() or pte.dirty() or pte.user() or pte.global() or pte.pbmt() != 0)
	    return vPageFaultType(read, write, exec);  // A/D/U/G bits must be zero in non-leaf entries.
          ii = ii - 1;
          if (ii < 0)
            return vPageFaultType(read, write, exec);
          root = pte.ppn() * pageSize_;
          continue;  // goto 2.
        }

      // 5.  pte.read_ or pte.exec_ : leaf pte
      if (pte.pbmt() != 0)
	return vPageFaultType(read, write, exec);  // Leaf page must bave pbmt=0.
      if (not pte.user())
	return vPageFaultType(read, write, exec);  // All access as though in User mode.

      bool pteRead = pte.read() or (execReadable_ and pte.exec());
      if (xForR_)
	pteRead = pte.exec();
      if ((read and not pteRead) or (write and not pte.write()) or
	  (exec and not pte.exec()))
	return vPageFaultType(read, write, exec);

      // 6.
      for (int j = 0; j < ii; ++j)
	if (pte.ppn(j) != 0)
	  return vPageFaultType(read, write, exec);

      // 7.
      if (accessDirtyCheck_ and (not pte.accessed() or (write and not pte.dirty())))
	{
	  // We have a choice:
	  // A. Page fault
	  if (faultOnFirstAccess_)
	    return vPageFaultType(read, write, exec);  // A

	  // Or B
	  saveUpdatedPte(pteAddr, sizeof(pte.data_), pte.data_);  // For logging

	  // B1. Check PMP. The privMode here is the effective one that
	  // already accounts for MPRV.
	  if (pmpMgr_.isEnabled())
	    {
	      Pmp pmp = pmpMgr_.accessPmp(pteAddr);
	      if (not pmp.isWrite(privMode, privMode, false))
		return accessFaultType(read, write, exec);
	    }

	  {
	    // TODO FIX : this has to be atomic
	    // B2. Compare pte to memory.
	    PTE pte2(0);
	    memory_.read(pteAddr, pte2.data_);
	    if (pte.data_ != pte2.data_)
	      continue;  // Comparison fails: return to step 2.
	    pte.bits_.accessed_ = 1;
	    if (write)
	      pte.bits_.dirty_ = 1;

	    if (not memory_.write(hartIx_, pteAddr, pte.data_))
	      return vPageFaultType(read, write, exec);
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

  // Update tlb-entry with data found in page table entry.
  tlbEntry.virtPageNum_ = address >> pageBits_;
  tlbEntry.physPageNum_ = pa >> pageBits_;
  tlbEntry.asid_ = vsAsid_;
  tlbEntry.vmid_ = vmid_;
  tlbEntry.valid_ = true;
  tlbEntry.global_ = pte.global();
  tlbEntry.user_ = pte.user();
  tlbEntry.read_ = pte.read();
  tlbEntry.write_ = pte.write();
  tlbEntry.exec_ = pte.exec();
  tlbEntry.accessed_ = pte.accessed();
  tlbEntry.dirty_ = pte.dirty();
  tlbEntry.levels_ = 1+ii;
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
  auto& walkVec = exec ? fetchWalks_ : dataWalks_;
  if (trace_)
    walkVec.resize(walkVec.size() + 1);

  if (attFile_)
    fprintf(attFile_, "VVA: 0x%jx\n", uintmax_t(address));

  while (true)
    {
      // 2.
      uint64_t gpteAddr = root + va.vpn(ii)*pteSize; // Guest pte address.
      if (trace_)
	walkVec.back().push_back(gpteAddr);

      // Translate guest pteAddr to host physical address.
      uint64_t pteAddr = gpteAddr;
      auto ec = stage2Translate(gpteAddr, privMode, true, false, false, pteAddr);
      if (ec != ExceptionCause::NONE)
	{
	  stage2ExceptionToStage1(ec, read, write, exec);
	  return ec;
	}

      // Check PMP. The privMode here is the effective one that
      // already accounts for MPRV.
      if (pmpMgr_.isEnabled())
	{
	  Pmp pmp = pmpMgr_.accessPmp(pteAddr);
	  if (not pmp.isRead(privMode, privMode, false))
	    return accessFaultType(read, write, exec);
	}

      if (! memory_.read(pteAddr, pte.data_))
        return nvPageFaultType(read, write, exec);

      if (attFile_)
        {
          bool leaf = pte.valid() and (pte.read() or pte.exec());
          fprintf(attFile_, "addr: 0x%jx\n", uintmax_t(pteAddr));
          fprintf(attFile_, "rwx: %d%d%d, ug: %d%d, ad: %d%d\n", pte.read(),
		  pte.write(), pte.exec(), pte.user(), pte.global(),
		  pte.accessed(), pte.dirty());
          fprintf(attFile_, "leaf: %d, pa:0x%jx", leaf,
		  uintmax_t(pte.ppn()) * pageSize_);
	  if (leaf)
            fprintf(attFile_, " s:%s", pageSize(vsMode_, ii));
	  fprintf(attFile_, "\n\n");
        }

      // 3.
      if (not pte.valid() or (not pte.read() and pte.write()) or pte.res())
        return nvPageFaultType(read, write, exec);

      // 4.
      if (not pte.read() and not pte.exec())
        {  // pte is a pointer to the next level
	  if (pte.accessed() or pte.dirty() or pte.user() or pte.pbmt() != 0)
	    return nvPageFaultType(read, write, exec);  // A/D/U bits must be zero in non-leaf entries.
          ii = ii - 1;
          if (ii < 0)
            return nvPageFaultType(read, write, exec);
          root = pte.ppn() * pageSize_;
          continue;  // goto 2.
        }

      // 5.  pte.read_ or pte.exec_ : leaf pte
      if (pte.pbmt() != 0)
	return nvPageFaultType(read, write, exec);  // Leaf page must bave pbmt=0.
      if (privMode == PrivilegeMode::User and not pte.user())
	return nvPageFaultType(read, write, exec);
      if (privMode == PrivilegeMode::Supervisor and pte.user() and
	  (not vsSum_ or exec))
	return nvPageFaultType(read, write, exec);

      bool pteRead = pte.read() or ((execReadable_ or s1ExecReadable_) and pte.exec());
      if (xForR_)
	pteRead = pte.exec();
      if ((read and not pteRead) or (write and not pte.write()) or
	  (exec and not pte.exec()))
	return nvPageFaultType(read, write, exec);

      // 6.
      for (int j = 0; j < ii; ++j)
	if (pte.ppn(j) != 0)
	  return nvPageFaultType(read, write, exec);

      // 7.
      if (accessDirtyCheck_ and (not pte.accessed() or (write and not pte.dirty())))
	{
	  // We have a choice:
	  // A. Page fault
	  if (faultOnFirstAccess_)
	    return nvPageFaultType(read, write, exec);  // A

	  // Or B
	  saveUpdatedPte(pteAddr, sizeof(pte.data_), pte.data_);  // For logging

	  // B1. Check PMP. The privMode here is the effective one that
	  // already accounts for MPRV.
	  if (pmpMgr_.isEnabled())
	    {
	      Pmp pmp = pmpMgr_.accessPmp(pteAddr);
	      if (not pmp.isWrite(privMode, privMode, false))
		return accessFaultType(read, write, exec);
	    }

	  {
	    // TODO FIX : this has to be atomic
	    // B2. Compare pte to memory.
	    PTE pte2(0);
	    memory_.read(pteAddr, pte2.data_);
	    if (pte.data_ != pte2.data_)
	      continue;  // Comparison fails: return to step 2.
	    pte.bits_.accessed_ = 1;
	    if (write)
	      pte.bits_.dirty_ = 1;

	    // Need to make sure we have write access to page.
	    uint64_t pteAddr2 = gpteAddr;
	    auto ec = stage2Translate(gpteAddr, privMode, false, true, false, pteAddr2);
	    if (ec != ExceptionCause::NONE)
	      {
		stage2ExceptionToStage1(ec, read, write, exec);
		return ec;
	      }
	    assert(pteAddr == pteAddr2);
	    if (not memory_.write(hartIx_, pteAddr2, pte.data_))
	      return nvPageFaultType(read, write, exec);
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

  // Update tlb-entry with data found in page table entry.
  tlbEntry.virtPageNum_ = address >> pageBits_;
  tlbEntry.physPageNum_ = pa >> pageBits_;
  tlbEntry.asid_ = vsAsid_;
  tlbEntry.vmid_ = vmid_;
  tlbEntry.valid_ = true;
  tlbEntry.global_ = pte.global();
  tlbEntry.user_ = pte.user();
  tlbEntry.read_ = pte.read();
  tlbEntry.write_ = pte.write();
  tlbEntry.exec_ = pte.exec();
  tlbEntry.accessed_ = pte.accessed();
  tlbEntry.dirty_ = pte.dirty();
  tlbEntry.levels_ = 1+ii;
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
    case Bare: os << "Bare\n"; break;
    case Sv32: os << "Sv32\n"; break;
    case Sv39: os << "Sv39\n"; break;
    case Sv48: os << "Sv48\n"; break;
    case Sv57: os << "Sv57\n"; break;
    case Sv64: os << "Sv64\n"; break;
    default:   os << "???\n";  break;
    }

  os << "Root page number: 0x" << std::hex << rootPage_ << std::dec << '\n';
  uint64_t addr = rootPage_ * pageSize_;
  os << "Root page addr: 0x" << std::hex << addr << std::dec << '\n';

  std::string path = "/";

  if (mode_ == Bare)
    ;  // relax
  else if (mode_ == Sv32)
    printEntries<Pte32, Va32>(os, addr, path);
  else if (mode_ == Sv39)
    printEntries<Pte39, Va39>(os, addr, path);
  else if (mode_ == Sv48)
    printEntries<Pte48, Va48>(os, addr, path);
  else
    os << "Unsupported virtual memory mode\n";
  os << "TLB:\n";
  tlb_.printTlb(os);
  os.flags(flags);
}


template<typename PTE, typename VA>
void
VirtMem::printEntries(std::ostream& os, uint64_t addr, std::string path) const
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
      memory_.read(eaddr, pte.data_);

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
      memory_.read(eaddr, pte.data_);

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
