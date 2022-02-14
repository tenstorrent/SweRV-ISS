#include <iostream>
#include <fstream>
#include "ArchInfo.hpp"
#include "third_party/nlohmann/json.hpp"

using namespace WdRiscv;


template <typename URV>
ArchInfo<URV>::ArchInfo(std::string filename)
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
      if (name == "opcode")
        entries_[ArchEntryName::Opcode] = in;
      else if (name == "mode")
        entries_[ArchEntryName::Mode] = in;
      else
        entries_[ArchEntryName::Undefined] = in;
    }
}


template <typename URV>
bool
ArchInfo<URV>::createInfoInst(Hart<URV>& hart, nlohmann::json& record, InstEntry entry)
{
  bool disable = false;
  disable = (entry.extension() == RvExtension::F and not hart.isRvf());
  disable = (entry.extension() == RvExtension::Zfh and not hart.isRvzfh()) or disable;
  disable = (entry.extension() == RvExtension::D and not hart.isRvd()) or disable;
  disable = (entry.extension() == RvExtension::E and not hart.isRve()) or disable;
  disable = (entry.extension() == RvExtension::M and not hart.isRvm()) or disable;
  // Compressed extension currently denoted with others exts (I, F, D, etc)
  disable = (entry.name().find("c.") == 0 and not hart.isRvc()) or disable;
  disable = (entry.extension() == RvExtension::A and not hart.isRva()) or disable;
  disable = (entry.extension() == RvExtension::V and not hart.isRvv()) or disable;
  disable = (entry.extension() == RvExtension::Zba and not hart.isRvzba()) or disable;
  disable = (entry.extension() == RvExtension::Zbb and not hart.isRvzbb()) or disable;
  disable = (entry.extension() == RvExtension::Zbc and not hart.isRvzbc()) or disable;
  disable = (entry.extension() == RvExtension::Zbe and not hart.isRvzbe()) or disable;
  disable = (entry.extension() == RvExtension::Zbf and not hart.isRvzbf()) or disable;
  disable = (entry.extension() == RvExtension::Zbm and not hart.isRvzbm()) or disable;
  disable = (entry.extension() == RvExtension::Zbp and not hart.isRvzbp()) or disable;
  disable = (entry.extension() == RvExtension::Zbr and not hart.isRvzbr()) or disable;
  disable = (entry.extension() == RvExtension::Zbs and not hart.isRvzbs()) or disable;
  disable = (entry.extension() == RvExtension::Zbt and not hart.isRvzbt()) or disable;
  disable = (entry.extension() == RvExtension::None or disable);

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
      return true;
    }

  return false;
}


template <typename URV>
bool
ArchInfo<URV>::createInfoMode(Hart<URV>& hart, nlohmann::json& record, PrivilegeMode mode)
{
  bool disable = false;
  disable = (mode == PrivilegeMode::User and not hart.isRvu()) or disable;
  disable = (mode == PrivilegeMode::Supervisor and not hart.isRvs()) or disable;

  unsigned encoding = static_cast<uint32_t>(mode) ^ entries_.at(ArchEntryName::Mode).mask;
  if (not disable)
    {
      record += nlohmann::json::object_t::value_type("symbol", privToString(mode));
      record += nlohmann::json::object_t::value_type("group", "mode");

      std::ostringstream oss;
      oss << "0x" << std::hex << encoding;
      record += nlohmann::json::object_t::value_type("encoding", oss.str());
      return true;
    }

  return false;
}


template class WdRiscv::ArchInfo<uint32_t>;
template class WdRiscv::ArchInfo<uint64_t>;
