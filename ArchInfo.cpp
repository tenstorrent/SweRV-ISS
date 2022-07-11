#include <iostream>
#include <fstream>
#include "ArchInfo.hpp"
#include "Hart.hpp"
#include "VecRegs.hpp"
#include "third_party/nlohmann/json.hpp"

using namespace WdRiscv;

const std::unordered_map<std::string, ArchInfoPoint> table =
  { {"dest", ArchInfoPoint::Dest},
    {"src",  ArchInfoPoint::Src},
    {"sew",  ArchInfoPoint::Sew},
    {"lmul", ArchInfoPoint::Lmul},
    {"mode", ArchInfoPoint::Mode} };

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

      // additional options
      if (entry.value().count("append"))
        for (auto& append : entry.value().at("append"))
          infoEntry.crosses_.push_back(table.at(append));


      // additional options
      // TODO: fix this
      // for (auto& options : entry.items())
      //   {
      //     if (options.key() == "cross")
      //       for (auto& cross : options.items())
      //         {
      //           continue;
      //         }
      //   }
      entries_.push_back(infoEntry);
    }
}


template <typename URV>
void
ArchInfo<URV>::addInstPoints(ArchInfoEntry& entry)
{
  InstEntry inst = hart_.instTable_.getEntry(entry.name_);
  bool disable = not hart_.hasIsaExtension(inst.extension());
  if (inst.isCompressed())
    disable = disable and not hart_.isRvc();
  if (disable)
    {
      std::cerr << "Instruction " << entry.name_ << " not enabled by config, will ignore\n";
      return;
    }

  entry.crosses_.push_back(ArchInfoPoint::Dest);
  entry.crosses_.push_back(ArchInfoPoint::Src);

  if (inst.extension() == RvExtension::V)
    {
      entry.crosses_.push_back(ArchInfoPoint::Sew);
      entry.crosses_.push_back(ArchInfoPoint::Lmul);
    }
}

template <typename URV>
bool
ArchInfo<URV>::addDestBins(ArchInfoEntry& entry) const
{
  if (entry.group_ != ArchInfoGroup::Inst)
    return false;

  nlohmann::json list;
  InstEntry inst = hart_.instTable_.getEntry(entry.name_);

  // four possible operands
  for (unsigned i = 0; i < 4; i++)
    {
      if (inst.isIthOperandWrite(i))
        {
          uint32_t limit = 0;
          if (inst.ithOperandType(i) == OperandType::IntReg)
            limit = (hart_.isRve()) ? uint32_t(IntRegNumber::RegX15)
                                                : uint32_t(IntRegNumber::RegX31);
          else if (inst.ithOperandType(i) == OperandType::FpReg)
            limit = uint32_t(FpRegNumber::RegF31);
          else if (inst.ithOperandType(i) == OperandType::VecReg)
            limit = uint32_t(VecRegNumber::RegV31);
          else if (inst.ithOperandType(i) == OperandType::CsReg)
            limit = uint32_t(CsrNumber::MAX_CSR_);
          else
            continue;

          for (uint32_t dest = 0; dest <= limit; ++dest)
            {
              if (inst.ithOperandType(i) != OperandType::CsReg)
                list.emplace_back(toJsonHex(dest));
              else
                {
                  CsrNumber destEnum = static_cast<CsrNumber>(dest);
                  Csr<URV>* csr = hart_.csRegs_.findCsr(destEnum);
                  if (csr and csr->isImplemented())
                    list.emplace_back(toJsonHex(dest));
                }
            }
        }
    }

  entry.j_ += nlohmann::json::object_t::value_type("dest", list);
  return true;
}

template <typename URV>
bool
ArchInfo<URV>::addSrcBins(ArchInfoEntry& entry) const
{
  if (entry.group_ != ArchInfoGroup::Inst)
    return false;

  nlohmann::json list;
  InstEntry inst = hart_.instTable_.getEntry(entry.name_);

  // four possible operands
  for (unsigned i = 0; i < 4; i++)
    {
      if (inst.isIthOperandWrite(i))
        {
          uint32_t limit = 0;
          if (inst.ithOperandType(i) == OperandType::IntReg)
            limit = (hart_.isRve()) ? uint32_t(IntRegNumber::RegX15)
                                        : uint32_t(IntRegNumber::RegX31);
          else if (inst.ithOperandType(i) == OperandType::FpReg)
            limit = uint32_t(FpRegNumber::RegF31);
          else if (inst.ithOperandType(i) == OperandType::VecReg)
            limit = uint32_t(VecRegNumber::RegV31);
          else if (inst.ithOperandType(i) == OperandType::CsReg)
            limit = uint32_t(CsrNumber::MAX_CSR_);
          else
            continue;

          for (uint32_t src = 0; src <= limit; ++src)
            {
              if (inst.ithOperandType(i) != OperandType::CsReg)
                list.emplace_back(toJsonHex(src));
              else
                {
                  CsrNumber srcEnum = static_cast<CsrNumber>(src);
                  Csr<URV>* csr = hart_.csRegs_.findCsr(srcEnum);
                  if (csr and csr->isImplemented())
                    list.emplace_back(toJsonHex(src));
                }
            }
        }
    }

  entry.j_ += nlohmann::json::object_t::value_type("src", list);
  return true;
}

template <typename URV>
bool
ArchInfo<URV>::addSewBins(ArchInfoEntry& entry) const
{
  nlohmann::json list;
  for (uint32_t sew = 0; sew <= uint32_t(ElementWidth::Word32); ++sew)
    {
      ElementWidth sewEnum = static_cast<ElementWidth>(sew);
      bool enable = hart_.vecRegs_.legalConfig(sewEnum, GroupMultiplier::Eight);
      if (enable)
        list.emplace_back(toJsonHex(sew));
    }

  entry.j_ += nlohmann::json::object_t::value_type("sew", list);
  return true;
}

template <typename URV>
bool
ArchInfo<URV>::addLmulBins(ArchInfoEntry& entry) const
{
  nlohmann::json list;
  for (uint32_t lmul = 0; lmul <= uint32_t(GroupMultiplier::Half); ++lmul)
    {
      GroupMultiplier lmulEnum = static_cast<GroupMultiplier>(lmul);
      bool enable = hart_.vecRegs_.legalConfig(static_cast<ElementWidth>(hart_.vecRegs_.minElementSizeInBytes()), lmulEnum);
      enable = enable and (lmulEnum != GroupMultiplier::Reserved);
      if (enable)
        list.emplace_back(toJsonHex(lmul));
    }

  entry.j_ += nlohmann::json::object_t::value_type("lmul", list);
  return true;
}

template <typename URV>
bool
ArchInfo<URV>::addModeBins(ArchInfoEntry& entry) const
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

  entry.j_ += nlohmann::json::object_t::value_type("mode", list);
  return true;
}


template class WdRiscv::ArchInfo<uint32_t>;
template class WdRiscv::ArchInfo<uint64_t>;
