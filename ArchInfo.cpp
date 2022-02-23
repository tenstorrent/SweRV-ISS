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
  bool disable = not hart.hasIsaExtension(entry.extension());
  if (entry.isCompressed())
    disable = disable and not hart.hasIsaExtension(RvExtension::C);

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
