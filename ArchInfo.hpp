#pragma once

#include <string>
#include "InstEntry.hpp"
#include "trapEnums.hpp"
#include "Hart.hpp"
#include "third_party/nlohmann/json.hpp"

namespace WdRiscv
{

  // Entry description
  enum class ArchEntryName { Opcode, Mode, Lmul, Sew, Undefined };
  enum class OperateMode : uint32_t
  {
    User = 0,
    Hypervisor = 1,
    Reserved = 2,
    Machine = 3,
    VirtUser = 4,
    VirtSupervisor = 5
  };

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
    ArchInfo(Hart<URV>& hart, std::string filename);

    /// Populate json object depending on opcode definition
    bool createInstInfo(nlohmann::json& j);

    /// Populate json object depending on modes definition
    bool createModeInfo(nlohmann::json& j);

    /// Popular json object depending on lmul/sew definition
    bool createLmulInfo(nlohmann::json& j);
    bool createSewInfo(nlohmann::json& j);

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
	  case RvExtension::Zknd:   return "Zknd";
	  case RvExtension::Zkne:   return "Zkne";
	  case RvExtension::Zknh:   return "Zknh";
	  case RvExtension::Zbkb:   return "Zbkb";
	  case RvExtension::Zksed:  return "Zksed";
	  case RvExtension::Zksh:   return "Zksh";
          default:                  return "Invalid";
        }
    }

    /// Operating (virt + priv) mode to string for when hypervisor
    /// is enabled.
    std::string virtPrivToString(const OperateMode mode) const
    {
      switch(mode)
        {
          case OperateMode::User:           return "UserMode";
          case OperateMode::Reserved:       return "HypervisorMode";
          case OperateMode::Hypervisor:     return "Hypervisor-SupervisorMode";
          case OperateMode::Machine:        return "MachineMode";
          case OperateMode::VirtUser:       return "VirtualUserMode";
          case OperateMode::VirtSupervisor: return "VirtualSupervisorMode";
          default:                          return "Invalid";
        }
    }

    std::string privToString(const PrivilegeMode mode) const
    {
      switch(mode)
        {
          case PrivilegeMode::User:          return "UserMode";
          case PrivilegeMode::Supervisor:    return "Supervisor";
          case PrivilegeMode::Reserved:      return "Reserved";
          case PrivilegeMode::Machine:       return "MachineMode";
          default:                           return "Invalid";
        }
    }

  private:

    std::unordered_map<ArchEntryName, struct ArchEntry> entries_;
    Hart<URV>& hart_;

    /// Contains a unique human-readable symbol such that -
    /// The same opcode value will have the same symbol AND
    /// A different opcode value will have a different symbol
    std::unordered_map<unsigned, std::string> symbols_;
    std::unordered_map<std::string, unsigned> typeIdx_;
  };
}
