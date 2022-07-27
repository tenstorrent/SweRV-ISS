#include <iostream>
#include <fstream>
#include <bitset>
#include <limits.h>
#include <boost/bimap.hpp>
#include <boost/assign.hpp>
#include "ArchInfo.hpp"
#include "Hart.hpp"
#include "VecRegs.hpp"
#include "third_party/nlohmann/json.hpp"

using namespace WdRiscv;

typedef boost::bimap<std::string, ArchInfoPoint> StringArchPoint;
StringArchPoint table = boost::assign::list_of< StringArchPoint::relation >
  ( "dest",             ArchInfoPoint::Dest)
  ( "src",              ArchInfoPoint::Src)
  ( "sew",              ArchInfoPoint::Sew)
  ( "lmul",             ArchInfoPoint::Lmul)
  ( "privmode",         ArchInfoPoint::PrivilegeMode)
  ( "pte.V",            ArchInfoPoint::PteV)
  ( "pte.R",            ArchInfoPoint::PteR)
  ( "pte.W",            ArchInfoPoint::PteW)
  ( "pte.X",            ArchInfoPoint::PteX)
  ( "pte.U",            ArchInfoPoint::PteU)
  ( "pte.G",            ArchInfoPoint::PteG)
  ( "pte.A",            ArchInfoPoint::PteA)
  ( "pte.D",            ArchInfoPoint::PteD)
  ( "pagingmode",       ArchInfoPoint::PagingMode)
  ( "paginglevel",      ArchInfoPoint::PagingLevel)
  ( "minterrupt",       ArchInfoPoint::MInterrupt)
  ( "mexception",       ArchInfoPoint::MException)
  ( "mtvecmode",        ArchInfoPoint::MTrapVecMode)
  ( "sinterrupt",       ArchInfoPoint::SInterrupt)
  ( "sexception",       ArchInfoPoint::SException)
  ( "stvecmode",        ArchInfoPoint::STrapVecMode);

template <typename URV>
ArchInfo<URV>::ArchInfo(Hart<URV>& hart, std::string filename)
  : hart_(hart)
{
  std::fstream ifs(filename);
  assert(ifs.good());

  nlohmann::json j;

  try
    {
      ifs >> j;
    }
  catch (std::exception& e)
    {
      std::cerr << e.what() << "\n";
      assert(false);
    }
  catch (...)
    {
      std::cerr << "Caught unknown exception while parsing "
		<< " archinfo file '" << filename << "\n";
      assert(false);
    }

  for (auto& entry : j.items())
    {
      ArchInfoEntry infoEntry;
      infoEntry.name_ = entry.key();
      infoEntry.group_ = getGroup(entry.key());

      if (infoEntry.group_ == ArchInfoGroup::Inst)
        addInstPoints(infoEntry);

      // additional coverpoints
      for (auto& point : entry.value())
        {
          auto it = table.left.find(point);
          if (it == table.left.end())
            std::cerr << "Unsupported info " << point << ", will ignore.\n";
          else
            infoEntry.points_.push_back(table.left.at(point));
        }

      entries_.push_back(infoEntry);
    }
}

template <typename URV>
void
ArchInfo<URV>::addDest(ArchInfoEntry& entry) const
{
  // should only be used within an instruction context
  if (entry.group_ != ArchInfoGroup::Inst)
    return;

  nlohmann::json list;
  InstEntry inst = hart_.getInstructionEntry(entry.name_);

  // four possible operands
  unsigned num = 0;
  for (unsigned i = 0; i < 4; i++)
    {
      if (inst.isIthOperandWrite(i))
        {
          uint32_t limit = 0;
          if (inst.ithOperandType(i) == OperandType::IntReg)
            limit = hart_.intRegCount();
          else if (inst.ithOperandType(i) == OperandType::FpReg)
            limit = hart_.fpRegCount();
          else if (inst.ithOperandType(i) == OperandType::VecReg)
            limit = hart_.vecRegCount();
          else if (inst.ithOperandType(i) == OperandType::CsReg)
            limit = uint32_t(CsrNumber::MAX_CSR_);
          else
            continue;

          for (uint32_t dest = 0; dest < limit; ++dest)
            {
              if (inst.ithOperandType(i) != OperandType::CsReg)
                list.emplace_back(toJsonHex(dest));
              else
                {
                  CsrNumber destEnum = static_cast<CsrNumber>(dest);
                  Csr<URV>* csr; csr = hart_.csRegs().findCsr(destEnum);
                  if (csr and csr->isImplemented())
                    list.emplace_back(toJsonHex(dest));
                }
            }
          entry.addBins(table.right.at(ArchInfoPoint::Dest) + std::to_string(++num), list);
          list.clear();
        }
    }
}

template <typename URV>
void
ArchInfo<URV>::addSrc(ArchInfoEntry& entry) const
{
  if (entry.group_ != ArchInfoGroup::Inst)
    return;

  nlohmann::json list;
  InstEntry inst = hart_.getInstructionEntry(entry.name_);

  // four possible operands
  unsigned num = 0;
  for (unsigned i = 0; i < 4; i++)
    {
      if (inst.isIthOperandRead(i))
        {
          uint32_t limit = 0;
          if (inst.ithOperandType(i) == OperandType::IntReg)
            limit = hart_.intRegCount();
          else if (inst.ithOperandType(i) == OperandType::FpReg)
            limit = hart_.fpRegCount();
          else if (inst.ithOperandType(i) == OperandType::VecReg)
            limit = hart_.vecRegCount();
          else if (inst.ithOperandType(i) == OperandType::CsReg)
            limit = uint32_t(CsrNumber::MAX_CSR_);
          else
            continue;

          for (uint32_t src = 0; src < limit; ++src)
            {
              if (inst.ithOperandType(i) != OperandType::CsReg)
                list.emplace_back(toJsonHex(src));
              else
                {
                  CsrNumber srcEnum = static_cast<CsrNumber>(src);
                  Csr<URV>* csr; csr = hart_.csRegs().findCsr(srcEnum);
                  if (csr and csr->isImplemented())
                    list.emplace_back(toJsonHex(src));
                }
            }
          entry.addBins(table.right.at(ArchInfoPoint::Src) + std::to_string(++num), list);
          list.clear();
        }
    }
}

template <typename URV>
void
ArchInfo<URV>::addSew(ArchInfoEntry& entry) const
{
  nlohmann::json list;
  for (uint32_t sew = 0; sew <= uint32_t(ElementWidth::Word32); ++sew)
    {
      ElementWidth sewEnum = static_cast<ElementWidth>(sew);
      bool enable = hart_.vecRegs().legalConfig(sewEnum, GroupMultiplier::Eight);
      if (enable)
        list.emplace_back(toJsonHex(sew));
    }

  entry.addBins(table.right.at(ArchInfoPoint::Sew), list);
}

template <typename URV>
void
ArchInfo<URV>::addLmul(ArchInfoEntry& entry) const
{
  nlohmann::json list;
  for (uint32_t lmul = 0; lmul <= uint32_t(GroupMultiplier::Half); ++lmul)
    {
      GroupMultiplier lmulEnum = static_cast<GroupMultiplier>(lmul);
      bool enable = hart_.vecRegs().legalConfig(static_cast<ElementWidth>(hart_.vecRegs().minElementSizeInBytes()), lmulEnum);
      enable = enable and (lmulEnum != GroupMultiplier::Reserved);
      if (enable)
        list.emplace_back(toJsonHex(lmul));
    }

  entry.addBins(table.right.at(ArchInfoPoint::Lmul), list);
}

template <typename URV>
void
ArchInfo<URV>::addPrivilegeMode(ArchInfoEntry& entry) const
{
  nlohmann::json list;
  for (uint32_t mode = 0; mode <= uint32_t(PrivilegeMode::Machine); ++mode)
    {
      bool disable = false;
      PrivilegeMode modeEnum = static_cast<PrivilegeMode>(mode);
      disable = (modeEnum == PrivilegeMode::User and not hart_.isRvu()) or disable;
      disable = (modeEnum == PrivilegeMode::Supervisor and not hart_.isRvs()) or disable;
      disable = (modeEnum == PrivilegeMode::Reserved) or disable;

      if (not disable)
        list.emplace_back(toJsonHex(mode));
    }

  entry.addBins(table.right.at(ArchInfoPoint::PrivilegeMode), list);
}

template <typename URV>
void
ArchInfo<URV>::addPteField(ArchInfoEntry& entry, ArchInfoPoint p) const
{
  if (not hart_.isRvs())
    return;

  nlohmann::json list;
  Pte57 pte(~0ULL);
  std::bitset<64> bits;
  switch (p)
    {
      case ArchInfoPoint::PteV:   bits = pte.bits_.valid_; break;
      case ArchInfoPoint::PteR:   bits = pte.bits_.read_; break;
      case ArchInfoPoint::PteW:   bits = pte.bits_.write_; break;
      case ArchInfoPoint::PteX:   bits = pte.bits_.exec_; break;
      case ArchInfoPoint::PteU:   bits = pte.bits_.user_; break;
      case ArchInfoPoint::PteG:   bits = pte.bits_.global_; break;
      case ArchInfoPoint::PteA:   bits = pte.bits_.accessed_; break;
      case ArchInfoPoint::PteD:   bits = pte.bits_.dirty_; break;
      default:                    assert(false); break;
    }

  for (uint32_t i = 0; i <= bits.count(); ++i)
    list.emplace_back(toJsonHex(i));

  entry.addBins(table.right.at(p), list);
}

template <typename URV>
void
ArchInfo<URV>::addPagingMode(ArchInfoEntry& entry) const
{
  nlohmann::json list;
  std::vector<VirtMem::Mode> modes = { VirtMem::Mode::Bare };
  if (not hart_.isRv64())
    modes.insert(modes.end(), { VirtMem::Mode::Sv32 });
  else
    modes.insert(modes.end(), { VirtMem::Mode::Sv39, VirtMem::Mode::Sv48, VirtMem::Mode::Sv57 });

  for (auto mode : modes)
    list.emplace_back(toJsonHex(uint32_t(mode)));

  entry.addBins(table.right.at(ArchInfoPoint::PagingMode), list);
}

template <typename URV>
void
ArchInfo<URV>::addPagingLevel(ArchInfoEntry& entry) const
{
  nlohmann::json list;
  Pte57 pte(0);

  for (uint32_t level = 1; level < pte.levels(); ++level)
    list.emplace_back(toJsonHex(level));

  entry.addBins(table.right.at(ArchInfoPoint::PagingLevel), list);
}

template <typename URV>
void
ArchInfo<URV>::addInterrupt(ArchInfoEntry& entry, ArchInfoPoint p) const
{
  if (p == ArchInfoPoint::SInterrupt and not hart_.isRvs())
    return;

  nlohmann::json list;
  std::vector<InterruptCause> interrupts
                = { InterruptCause::S_SOFTWARE, InterruptCause::S_TIMER,
                    InterruptCause::S_EXTERNAL };

  if (p != ArchInfoPoint::SInterrupt)
    interrupts.insert(interrupts.end(), { InterruptCause::M_SOFTWARE,
                          InterruptCause::M_TIMER, InterruptCause::M_EXTERNAL } );

  for (auto interrupt : interrupts)
    list.emplace_back(toJsonHex(uint32_t(interrupt)));

  entry.addBins(table.right.at(p), list);
}

template <typename URV>
void
ArchInfo<URV>::addException(ArchInfoEntry& entry, ArchInfoPoint p) const
{
  if (p == ArchInfoPoint::SInterrupt and not hart_.isRvs())
    return;

  nlohmann::json list;
  std::vector<ExceptionCause> exceptions
                = { ExceptionCause::INST_ADDR_MISAL, ExceptionCause::INST_ACC_FAULT,
                    ExceptionCause::ILLEGAL_INST, ExceptionCause::BREAKP,
                    ExceptionCause::LOAD_ADDR_MISAL, ExceptionCause::LOAD_ACC_FAULT,
                    ExceptionCause::STORE_ADDR_MISAL, ExceptionCause::STORE_ACC_FAULT,
                    ExceptionCause::U_ENV_CALL, ExceptionCause::S_ENV_CALL,
                    ExceptionCause::INST_PAGE_FAULT, ExceptionCause::LOAD_PAGE_FAULT };

  if (p != ArchInfoPoint::SException)
    exceptions.insert(exceptions.end(), { ExceptionCause::M_ENV_CALL });

  for (auto exception : exceptions)
    list.emplace_back(toJsonHex(uint32_t(exception)));

  entry.addBins(table.right.at(p), list);
}

template <typename URV>
void
ArchInfo<URV>::addTrapVecMode(ArchInfoEntry& entry, ArchInfoPoint p) const
{
  if (p == ArchInfoPoint::STrapVecMode and not hart_.isRvs())
    return;

  nlohmann::json list;
  for (uint32_t mode = 0; mode <= uint32_t(TrapVectorMode::VECTORED); ++mode)
    list.emplace_back(toJsonHex(uint32_t(mode)));

  entry.addBins(table.right.at(p), list);
}


template class WdRiscv::ArchInfo<uint32_t>;
template class WdRiscv::ArchInfo<uint64_t>;
