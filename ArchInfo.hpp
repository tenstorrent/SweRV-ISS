#pragma once

#include <string>
#include "InstEntry.hpp"
#include "trapEnums.hpp"
#include "Hart.hpp"
#include "third_party/nlohmann/json.hpp"

namespace WdRiscv
{

  // Entry description
  enum class ArchEntryName { Opcode, Mode, Undefined };

  struct ArchEntry
  {
    unsigned mask = 0; // Size of data (# of bits)
  };

  /// Architectural coverage definition. Should be provided as a json file.
  template <typename URV>
  class ArchInfo
  {
  public:

    /// Copy JSON file
    ArchInfo(std::string filename);

    /// Convert string to name enum.
    ArchEntryName stringToName(const std::string str) const
    {
      if (str == "Opcode") return ArchEntryName::Opcode;
      else if (str == "Mode") return ArchEntryName::Mode;
      else return ArchEntryName::Undefined;
    }

    /// Populate json object depending on opcode definition
    bool createInfoInst(Hart<URV>& hart, nlohmann::json& record, InstEntry entry);

    /// Populate json object depending on modes definition
    bool createInfoMode(Hart<URV>& hart, nlohmann::json& record, PrivilegeMode mode);

    /// Privilege mode to string
    std::string privToString(const PrivilegeMode mode) const
    {
      switch(mode)
        {
          case PrivilegeMode::User:         return "UserMode";
          case PrivilegeMode::Supervisor:   return "SupervisorMode";
          case PrivilegeMode::Reserved:     return "ReservedMode";
          case PrivilegeMode::Machine:      return "MachineMode";
          default:                          return "Invalid";
        }
    }

    /// Extension to string
    std::string extToString(const RvExtension ext) const
    {
      switch(ext)
        {
          case RvExtension::M:      return "Rvm";
          case RvExtension::I:      return "Rvi";
          case RvExtension::F:      return "Rvf";
          case RvExtension::D:      return "Rvd";
          case RvExtension::A:      return "Rva";
          case RvExtension::V:      return "Rvv";
          case RvExtension::Zba:    return "Zba";
          case RvExtension::Zbb:    return "Zbb";
          case RvExtension::Zbc:    return "Zbc";
          case RvExtension::Zbe:    return "Zbe";
          case RvExtension::Zbf:    return "Zbf";
          case RvExtension::Zbm:    return "Zbm";
          case RvExtension::Zbp:    return "Zbp";
          case RvExtension::Zbr:    return "Zbr";
          case RvExtension::Zbs:    return "Zbs";
          case RvExtension::Zbt:    return "Zbt";
          case RvExtension::Zfh:    return "Zfh";
          case RvExtension::Zlsseg: return "Zlsseg";
	  case RvExtension::Zk:     return "Zk";
          default:                  return "Invalid";
        }
    }

  private:

    std::unordered_map<ArchEntryName, struct ArchEntry> entries_;

    /// Contains a unique human-readable symbol such that -
    /// The same opcode value will have the same symbol AND
    /// A different opcode value will have a different symbol
    std::unordered_map<unsigned, std::string> symbols_;
    std::unordered_map<std::string, unsigned> typeIdx_;
  };
}
