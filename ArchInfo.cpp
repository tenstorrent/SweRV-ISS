#include <iostream>
#include <fstream>
#include "ArchInfo.hpp"
#include "Hart.hpp"
#include "VecRegs.hpp"
#include "third_party/nlohmann/json.hpp"

using namespace WdRiscv;

const std::unordered_map<std::string, ArchEntryName> table =
  { {"opcode", ArchEntryName::Opcode},
    {"mode",   ArchEntryName::Mode},
    {"lmul",   ArchEntryName::Lmul},
    {"sew",    ArchEntryName::Sew} };

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
		<< " archinfo file '" << filename << "'\n";
      assert(false);
    }

  for (auto& entry : j.items())
    {
      struct ArchEntry in;
      in.mask = entry.value()["mask"];

      std::string name = entry.value()["name"];
      try
        {
          entries_[table.at(name)] = in;
        }
      catch (std::exception& e)
        {
          std::cerr << e.what() << name << "\n";
          assert(false);
        }
    }
}


template <typename URV>
bool
ArchInfo<URV>::createInstInfo(nlohmann::json& j)
{
  if (entries_.find(ArchEntryName::Opcode) == entries_.end()) return true;

  for (auto& entry : hart_.instTable_.getInstVec())
    {
      nlohmann::json record;
      bool disable = not hart_.hasIsaExtension(entry.extension());
      if (entry.isCompressed())
        disable = disable and not hart_.hasIsaExtension(RvExtension::C);

      unsigned encoding = entry.code() & entries_.at(ArchEntryName::Opcode).mask;
      if (not symbols_.count(encoding) and not disable)
        {
          std::string ext = extToString(entry.extension());
          if (not typeIdx_.count(ext))
            typeIdx_[ext] = 0;

          symbols_[encoding] = ext + std::to_string(typeIdx_.at(ext));
          ++typeIdx_[ext];
          record += nlohmann::json::object_t::value_type("symbol", symbols_.at(encoding));
          record += nlohmann::json::object_t::value_type("group", "opcode");

          std::ostringstream oss;
          oss << "0x" << std::hex << encoding;
          record += nlohmann::json::object_t::value_type("encoding", oss.str());
          j += record;
        }
    }

  return true;
}


template <typename URV>
bool
ArchInfo<URV>::createModeInfo(nlohmann::json& j)
{
  if (entries_.find(ArchEntryName::Mode) == entries_.end()) return true;

  for (uint32_t mode = 0; mode <= uint32_t(PrivilegeMode::Machine); mode++)
    {
      nlohmann::json record;
      bool disable = false;
      PrivilegeMode modeEnum = static_cast<PrivilegeMode>(mode);
      disable = (modeEnum == PrivilegeMode::User and not hart_.isRvu()) or disable;
      disable = (modeEnum == PrivilegeMode::Supervisor and not hart_.isRvs()) or disable;
      disable = (modeEnum == PrivilegeMode::Reserved) or disable;

      unsigned encoding = mode & entries_.at(ArchEntryName::Mode).mask;
      if (not disable)
        {
          record += nlohmann::json::object_t::value_type("symbol", privToString(modeEnum));
          record += nlohmann::json::object_t::value_type("group", "mode");

          std::ostringstream oss;
          oss << "0x" << std::hex << encoding;
          record += nlohmann::json::object_t::value_type("encoding", oss.str());
          j += record;
        }
    }

  return true;
}


template <typename URV>
bool
ArchInfo<URV>::createLmulInfo(nlohmann::json& j)
{
  if (entries_.find(ArchEntryName::Lmul) == entries_.end()) return true;

  for (uint32_t lmul = 0; lmul <= uint32_t(GroupMultiplier::Half); lmul++)
    {
      nlohmann::json record;
      GroupMultiplier lmulEnum = static_cast<GroupMultiplier>(lmul);
      bool enable = hart_.vecRegs_.legalConfig(static_cast<ElementWidth>(hart_.vecRegs_.minElementSizeInBytes()), lmulEnum);
      enable = enable and (lmulEnum != GroupMultiplier::Reserved);

      unsigned encoding = lmul & entries_.at(ArchEntryName::Lmul).mask;
      if (enable)
        {
          record += nlohmann::json::object_t::value_type("symbol", hart_.vecRegs_.to_string(lmulEnum));
          record += nlohmann::json::object_t::value_type("group", "lmul");

          std::ostringstream oss;
          oss << "0x" << std::hex << encoding;
          record += nlohmann::json::object_t::value_type("encoding", oss.str());
          j += record;
        }
    }

  return true;
}


template <typename URV>
bool
ArchInfo<URV>::createSewInfo(nlohmann::json& j)
{
  if (entries_.find(ArchEntryName::Sew) == entries_.end()) return true;

  for (uint32_t sew = 0; sew <= uint32_t(ElementWidth::Word32); sew++)
    {
      nlohmann::json record;
      ElementWidth sewEnum = static_cast<ElementWidth>(sew);
      bool enable = hart_.vecRegs_.legalConfig(sewEnum, GroupMultiplier::Eight);

      unsigned encoding = sew & entries_.at(ArchEntryName::Sew).mask;
      if (enable)
        {
          record += nlohmann::json::object_t::value_type("symbol", hart_.vecRegs_.to_string(sewEnum));
          record += nlohmann::json::object_t::value_type("group", "sew");

          std::ostringstream oss;
          oss << "0x" << std::hex << encoding;
          record += nlohmann::json::object_t::value_type("encoding", oss.str());
          j += record;
        }
    }

  return true;
}


template class WdRiscv::ArchInfo<uint32_t>;
template class WdRiscv::ArchInfo<uint64_t>;
