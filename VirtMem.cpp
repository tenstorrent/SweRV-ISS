#include <cmath>
#include <iostream>
#include <ios>
#include "PmpManager.hpp"
#include "VirtMem.hpp"
#include "inttypes.h"

using namespace WdRiscv;


VirtMem::VirtMem(unsigned hartIx, Memory& memory, unsigned pageSize,
                 PmpManager& pmpMgr, unsigned tlbSize)
  : memory_(memory), mode_(Sv32), pageSize_(pageSize), hartIx_(hartIx),
    pmpMgr_(pmpMgr), tlb_(tlbSize)
{
  pageBits_ = static_cast<unsigned>(std::log2(pageSize_));
  unsigned p2PageSize =  unsigned(1) << pageBits_;

  assert(p2PageSize == pageSize);
  assert(pageSize >= 64);

  pageMask_ = pageSize_ - 1;
}


inline
ExceptionCause
pageFaultType(bool read, bool write, bool exec)
{
  if (exec)  return ExceptionCause::INST_PAGE_FAULT;
  if (read)  return ExceptionCause::LOAD_PAGE_FAULT;
  if (write) return ExceptionCause::STORE_PAGE_FAULT;
  assert(0);
  return ExceptionCause::STORE_PAGE_FAULT;
}


inline
ExceptionCause
guestPageFaultType(bool read, bool write, bool exec)
{
  if (exec)  return ExceptionCause::INST_GUEST_PAGE_FAULT;
  if (read)  return ExceptionCause::LOAD_GUEST_PAGE_FAULT;
  if (write) return ExceptionCause::STORE_GUEST_PAGE_FAULT;
  assert(0);
  return ExceptionCause::STORE_PAGE_FAULT;
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
VirtMem::translateForFetch(uint64_t va, PrivilegeMode priv, uint64_t& pa)
{
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
        return pageFaultType(false, true, true);
      if (priv == PrivilegeMode::Supervisor and entry->user_)
        return pageFaultType(false, false, true);
      if (not entry->exec_)
        return pageFaultType(false, false, true);
      if (not entry->accessed_)
        {
          if (faultOnFirstAccess_)
            return pageFaultType(false, false, true);
          entry->accessed_ = true;
        }
      pa = (entry->physPageNum_ << pageBits_) | (va & pageMask_);
      return ExceptionCause::NONE;
    }

  TlbEntry tlbEntry;
  auto cause = doTranslate(va, priv, false, false, true, pa, tlbEntry);

  // If successful, put cache translation results in TLB.
  if (cause == ExceptionCause::NONE)
    tlb_.insertEntry(tlbEntry);

  return cause;
}


ExceptionCause
VirtMem::translateForFetch2(uint64_t va, unsigned size, PrivilegeMode priv,
			    uint64_t& pa1, uint64_t& pa2)
{
  pa1 = pa2 = va;
  auto cause = translateForFetch(va, priv, pa1);
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
  cause = translateForFetch(va2, priv, pa2);
  if (cause != ExceptionCause::NONE)
    pa1 = pa2 = va2;

  return cause;
}


ExceptionCause
VirtMem::transAddrNoUpdate(uint64_t va, PrivilegeMode priv, bool read,
			   bool write, bool exec, uint64_t& pa)
{
  auto prevTrace = trace_; trace_ = false;
  auto prevFile  = attFile_; attFile_ = nullptr;
  accessDirtyCheck_ = false;

  auto cause = transNoUpdate(va, priv, read, write, exec, pa);

  trace_ = prevTrace;
  attFile_ = prevFile;
  accessDirtyCheck_ = true;

  return cause;
}


ExceptionCause
VirtMem::transNoUpdate(uint64_t va, PrivilegeMode priv, bool read, bool write,
		       bool exec, uint64_t& pa)
{
  if (mode_ == Bare)
    {
      pa = va;
      return ExceptionCause::NONE;
    }

  assert(read or write or exec);

  // Lookup virtual page number in TLB.
  uint64_t virPageNum = va >> pageBits_;
  TlbEntry* entry = tlb_.findEntryUpdateTime(virPageNum, asid_);
  if (entry)
    {
      // Use TLB entry.
      if (priv == PrivilegeMode::User and not entry->user_)
        return pageFaultType(read, write, exec);
      if (priv == PrivilegeMode::Supervisor and entry->user_)
        return pageFaultType(read, write, exec);
      if (read)
	{
	  bool entryRead = entry->read_ or (execReadable_ and entry->exec_);
	  if (not entryRead)
	    return pageFaultType(true, false, false);
	}
      if (write and not entry->write_)
        return pageFaultType(false, true, false);
      if (exec and not entry->exec_)
        return pageFaultType(false, false, true);
      pa = (entry->physPageNum_ << pageBits_) | (va & pageMask_);
      return ExceptionCause::NONE;
    }


  TlbEntry tlbEntry;
  return doTranslate(va, priv, read, write, exec, pa, tlbEntry);
}


ExceptionCause
VirtMem::translateForLoad(uint64_t va, PrivilegeMode priv, uint64_t& pa)
{
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
        return pageFaultType(true, false, false);
      if (priv == PrivilegeMode::Supervisor and entry->user_ and not supervisorOk_)
        return pageFaultType(true, false, false);
      bool entryRead = entry->read_ or (execReadable_ and entry->exec_);
      if (not entryRead)
        return pageFaultType(true, false, false);
      if (not entry->accessed_)
        {
          if (faultOnFirstAccess_)
            return pageFaultType(true, false, false);
          entry->accessed_ = true;
        }
      pa = (entry->physPageNum_ << pageBits_) | (va & pageMask_);
      return ExceptionCause::NONE;
    }

  TlbEntry tlbEntry;
  auto cause = doTranslate(va, priv, true, false, false, pa, tlbEntry);

  // If successful, put cache translation results in TLB.
  if (cause == ExceptionCause::NONE)
    tlb_.insertEntry(tlbEntry);

  return cause;
}


ExceptionCause
VirtMem::translateForLoad2(uint64_t va, unsigned size, PrivilegeMode priv,
			   uint64_t& pa1, uint64_t& pa2)
{
  pa1 = pa2 = va;
  auto cause = translateForLoad(va, priv, pa1);
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
  cause = translateForLoad(va2, priv, pa2);
  if (cause != ExceptionCause::NONE)
    pa1 = pa2 = va2;

  return cause;
}


ExceptionCause
VirtMem::translateForStore(uint64_t va, PrivilegeMode priv, uint64_t& pa)
{
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
        return pageFaultType(false, true, false);
      if (priv == PrivilegeMode::Supervisor and entry->user_ and not supervisorOk_)
        return pageFaultType(false, true, false);
      if (not entry->write_)
        return pageFaultType(false, true, false);
      if (not entry->accessed_ or not entry->dirty_)
        {
          if (faultOnFirstAccess_)
            return pageFaultType(false, true, false);
          entry->accessed_ = true;
          entry->dirty_ = true;
        }
      pa = (entry->physPageNum_ << pageBits_) | (va & pageMask_);
      return ExceptionCause::NONE;
    }

  TlbEntry tlbEntry;
  auto cause = doTranslate(va, priv, false, true, false, pa, tlbEntry);

  // If successful, put cache translation results in TLB.
  if (cause == ExceptionCause::NONE)
    tlb_.insertEntry(tlbEntry);

  return cause;
}


ExceptionCause
VirtMem::translateForStore2(uint64_t va, unsigned size, PrivilegeMode priv,
			    uint64_t& pa1, uint64_t& pa2)
{
  pa1 = pa2 = va;
  auto cause = translateForStore(va, priv, pa1);
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
  cause = translateForStore(va2, priv, pa2);
  if (cause != ExceptionCause::NONE)
    pa1 = pa2 = va2;

  return cause;
}


ExceptionCause
VirtMem::translate(uint64_t va, PrivilegeMode priv, bool read, bool write,
                   bool exec, uint64_t& pa)
{
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
        return pageFaultType(read, write, exec);
      if (priv == PrivilegeMode::Supervisor and entry->user_ and not supervisorOk_)
        return pageFaultType(read, write, exec);
      bool entryRead = entry->read_ or (execReadable_ and entry->exec_);
      if ((read and not entryRead) or (write and not entry->write_) or
          (exec and not entry->exec_))
        return pageFaultType(read, write, exec);
      if (not entry->accessed_ or (write and not entry->dirty_))
        {
          if (faultOnFirstAccess_)
            return pageFaultType(read, write, exec);
          entry->accessed_ = true;
          if (write)
            entry->dirty_ = true;
        }
      pa = (entry->physPageNum_ << pageBits_) | (va & pageMask_);
      return ExceptionCause::NONE;
    }

  TlbEntry tlbEntry;
  auto cause = doTranslate(va, priv, read, write, exec, pa, tlbEntry);

  // If successful, put cache translation results in TLB.
  if (cause == ExceptionCause::NONE)
    tlb_.insertEntry(tlbEntry);

  return cause;
}


ExceptionCause
VirtMem::doTranslate(uint64_t va, PrivilegeMode priv, bool read,
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
        return pageFaultType(read, write, exec);
      cause = pageTableWalk1p12<Pte39, Va39>(va, priv, read, write, exec, pa, entry);
    }
  else if (mode_ == Sv48)
    {
      // Part 1 of address translation: Bits 63-47 must equal bit 47
      uint64_t mask = (va >> 47) & 1;
      if (mask)
        mask = 0xffff;  // Least sig 16 bits set
      if ((va >> 48) != mask)
        return pageFaultType(read, write, exec);
      cause = pageTableWalk1p12<Pte48, Va48>(va, priv, read, write, exec, pa, entry);
    }
  else if (mode_ == Sv57)
    {
      // Part 1 of address translation: Bits 63-57 muse equal bit 56
      uint64_t mask = (va >> 56) & 1;
      if (mask)
        mask = 0x7f;  // Least sig 7 bits set
      if ((va >> 57) != mask)
        return pageFaultType(read, write, exec);
      cause = pageTableWalk1p12<Pte57, Va57>(va, priv, read, write, exec, pa, entry);
    }
  else
    assert(0 and "Unspupported virtual memory mode.");

  return cause;
}


ExceptionCause
VirtMem::doStage2Translate(uint64_t va, PrivilegeMode priv, bool read,
			   bool write, bool exec, uint64_t& pa, TlbEntry& entry)
{
  // Perform a page table walk.
  ExceptionCause cause = ExceptionCause::LOAD_PAGE_FAULT;

  if (mode_ == Sv32)
    {
      // Part 1 of address translation: Bits 63-34 must be zero
      if ((va >> 34) != 0)
	return guestPageFaultType(read, write, exec);
      cause = pageTableWalkStage2<Pte32, Va32x4>(va, priv, read, write, exec, pa, entry);
    }
  else if (mode_ == Sv39)
    {
      // Part 1 of address translation: Bits 63-41 must be zero
      if ((va >> 41) != 0)
        return guestPageFaultType(read, write, exec);
      cause = pageTableWalkStage2<Pte39, Va39x4>(va, priv, read, write, exec, pa, entry);
    }
  else if (mode_ == Sv48)
    {
      // Part 1 of address translation: Bits 63-50 must be zero
      if ((va >> 50) != 0)
        return guestPageFaultType(read, write, exec);
      cause = pageTableWalkStage2<Pte48, Va48x4>(va, priv, read, write, exec, pa, entry);
    }
  else if (mode_ == Sv57)
    {
      // Part 1 of address translation: Bits 63-59 must be zero
      if ((va >> 59) != 0)
        return guestPageFaultType(read, write, exec);
      cause = pageTableWalkStage2<Pte57, Va57x4>(va, priv, read, write, exec, pa, entry);
    }
  else
    assert(0 and "Unspupported virtual memory mode.");

  return cause;
}


template<typename PTE, typename VA>
ExceptionCause
VirtMem::pageTableWalk(uint64_t address, PrivilegeMode privMode, bool read, bool write,
                       bool exec, uint64_t& pa, TlbEntry& tlbEntry)
{
  // 1. Done in translate method.

  PTE pte(0);

  const unsigned levels = pte.levels();
  const unsigned pteSize = pte.size();

  VA va(address);

  // Collect PTE addresses used in the translation process.
  auto& walkVec = exec ? fetchWalks_ : dataWalks_;
  if (trace_)
    walkVec.resize(walkVec.size() + 1);

  if (attFile_)
    fprintf(attFile_, "VA: 0x%jx\n", uintmax_t(address));

  // 2. Root is "a" in section 4.3.2 of privileged spec.
  uint64_t root = rootPage_ * pageSize_;
  uint64_t pteAddr = 0;
  int ii = levels - 1;

  while (true)
    {
      // 3.
      uint32_t vpn = va.vpn(ii);
      pteAddr = root + vpn*pteSize;
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
        return pageFaultType(read, write, exec);

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
            fprintf(attFile_, " s:%s", VirtMem::pageSize(mode_, ii));
	  fprintf(attFile_, "\n\n");
        }

      // 4.
      if (not pte.valid() or (not pte.read() and pte.write()) or pte.res())
        return pageFaultType(read, write, exec);

      // 5.
      if (not pte.read() and not pte.exec())
        {
          ii = ii - 1;
          if (ii < 0)
            return pageFaultType(read, write, exec);
          root = pte.ppn() * pageSize_;
          // goto 3.
        }
      else
        break;  // goto 6.
    }

  // 6.  pte.read_ or pte.exec_ : leaf pte
  if (pte.pbmt() != 0)
    return pageFaultType(read, write, exec);  // Leaf page must bave pbmt=0.
  if (privMode == PrivilegeMode::User and not pte.user())
    return pageFaultType(read, write, exec);
  if (privMode == PrivilegeMode::Supervisor and pte.user() and
      (not supervisorOk_ or exec))
    return pageFaultType(read, write, exec);

  bool pteRead = pte.read() or (execReadable_ and pte.exec());
  if ((read and not pteRead) or (write and not pte.write()) or
      (exec and not pte.exec()))
    return pageFaultType(read, write, exec);

  // 7.
  for (int j = 0; j < ii; ++j)
    if (pte.ppn(j) != 0)
      return pageFaultType(read, write, exec);

  // 8.
  if (accessDirtyCheck_ and (not pte.accessed() or (write and not pte.dirty())))
    {
      // We have a choice:
      // A. Page fault
      if (faultOnFirstAccess_)
        return pageFaultType(read, write, exec);  // A

      // Or B
      // B1. Set pte->accessed_ to 1 and, if a write, set pte->dirty_ to 1.
      // B2. Access fault if PMP violation.
      saveUpdatedPte(pteAddr, sizeof(pte.data_), pte.data_); // for logging
      pte.bits_.accessed_ = 1;
      if (write)
        pte.bits_.dirty_ = 1;

      // Check PMP. The privMode here is the effective one that
      // already accounts for MPRV.
      if (pmpMgr_.isEnabled())
	{
	  Pmp pmp = pmpMgr_.accessPmp(pteAddr);
	  if (not pmp.isWrite(privMode, privMode, false))
	    return accessFaultType(read, write, exec);
	}

      if (not memory_.write(hartIx_, pteAddr, pte.data_))
        return pageFaultType(read, write, exec);
    }

  // 9.
  pa = va.offset();

  for (int j = 0; j < ii; ++j)
    pa = pa | (va.vpn(j) << pte.paPpnShift(j)); // Copy from va to pa

  for (unsigned j = ii; j < levels; ++j)
    pa = pa | pte.ppn(j) << pte.paPpnShift(j);

  if (attFile_)
    fprintf(attFile_, "PA: 0x%jx\n\n", uintmax_t(pa));

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
        return pageFaultType(read, write, exec);

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
        return pageFaultType(read, write, exec);

      // 4.
      if (not pte.read() and not pte.exec())
        {  // pte is a pointer to the next level
	  if (pte.accessed() or pte.dirty() or pte.user())
	    return pageFaultType(read, write, exec);  // A/D/U bits must be zero in non-leaf entries.
          ii = ii - 1;
          if (ii < 0)
            return pageFaultType(read, write, exec);
          root = pte.ppn() * pageSize_;
          continue;  // goto 2.
        }

      // 5.  pte.read_ or pte.exec_ : leaf pte
      if (pte.pbmt() != 0)
	return pageFaultType(read, write, exec);  // Leaf page must bave pbmt=0.
      if (privMode == PrivilegeMode::User and not pte.user())
	return pageFaultType(read, write, exec);
      if (privMode == PrivilegeMode::Supervisor and pte.user() and
	  (not supervisorOk_ or exec))
	return pageFaultType(read, write, exec);

      bool pteRead = pte.read() or (execReadable_ and pte.exec());
      if ((read and not pteRead) or (write and not pte.write()) or
	  (exec and not pte.exec()))
	return pageFaultType(read, write, exec);

      // 6.
      for (int j = 0; j < ii; ++j)
	if (pte.ppn(j) != 0)
	  return pageFaultType(read, write, exec);

      // 7.
      if (accessDirtyCheck_ and (not pte.accessed() or (write and not pte.dirty())))
	{
	  // We have a choice:
	  // A. Page fault
	  if (faultOnFirstAccess_)
	    return pageFaultType(read, write, exec);  // A

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
	      return pageFaultType(read, write, exec);
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
VirtMem::pageTableWalkStage2(uint64_t address, PrivilegeMode privMode, bool read, bool write,
			     bool exec, uint64_t& pa, TlbEntry& tlbEntry)
{
  // 1. Root is "a" in section 4.3.2 of the privileged spec, ii is "i" in that section.
  uint64_t root = rootPage2_ * pageSize_;

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
        return guestPageFaultType(read, write, exec);

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
        return guestPageFaultType(read, write, exec);

      // 4.
      if (not pte.read() and not pte.exec())
        {  // pte is a pointer to the next level
	  if (pte.accessed() or pte.dirty() or pte.user() or pte.global())
	    return guestPageFaultType(read, write, exec);  // A/D/U/G bits must be zero in non-leaf entries.
          ii = ii - 1;
          if (ii < 0)
            return pageFaultType(read, write, exec);
          root = pte.ppn() * pageSize_;
          continue;  // goto 2.
        }

      // 5.  pte.read_ or pte.exec_ : leaf pte
      if (pte.pbmt() != 0 or pte.global())
	return guestPageFaultType(read, write, exec);  // Leaf page must bave pbmt=0 and G=0.
      if (not pte.user())
	return guestPageFaultType(read, write, exec);  // All access as though in User mode.

      bool pteRead = pte.read() or (execReadable_ and pte.exec());
      if ((read and not pteRead) or (write and not pte.write()) or
	  (exec and not pte.exec()))
	return guestPageFaultType(read, write, exec);

      // 6.
      for (int j = 0; j < ii; ++j)
	if (pte.ppn(j) != 0)
	  return pageFaultType(read, write, exec);

      // 7.
      if (accessDirtyCheck_ and (not pte.accessed() or (write and not pte.dirty())))
	{
	  // We have a choice:
	  // A. Page fault
	  if (faultOnFirstAccess_)
	    return guestPageFaultType(read, write, exec);  // A

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
	      return guestPageFaultType(read, write, exec);
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
  
  if (mode_ == Sv32)
    {
      if (size != 4096)
        return false;
      pageBits_ = bits;
      pageSize_ = size;
      return true;
    }

  if (mode_ == Sv39)
    {
      if (size != 4096 and size != 2*1024*1024 and size != 1024*1024*1024)
        return false;
      pageBits_ = bits;
      pageSize_ = size;
      return true;
    }

  if (mode_ == Sv48)
    {
      if (size != 4096 and size != 2*1024*1024 and size != 1024*1024*1024
          and size != 512L*1024L*1024L*1024L)
        return false;
      pageBits_ = bits;
      pageSize_ = size;
    }

  assert(0 && "Translation modes Sv57 and Sv64 are not currently supported");
  return false;
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

  os << "Root page number: 0x" << std::hex << rootPage_ << '\n';
  uint64_t addr = rootPage_ * pageSize_;
  os << "Root page addr: 0x" << std::hex << addr << '\n';

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
  os << "Page table page addr: 0x" << std::hex << addr << '\n';
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
         << " leaf:" << leaf << " pa:0x" << (pte.ppn() * pageSize_) << '\n';
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
