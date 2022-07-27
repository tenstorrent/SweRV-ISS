#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include "InstEntry.hpp"
#include "trapEnums.hpp"
#include "Hart.hpp"
#include "third_party/nlohmann/json.hpp"

namespace WdRiscv
{

  // Entry description
  enum class ArchInfoPoint { Dest, Src, Sew, Lmul, Frm, Fflags, PrivilegeMode,
                              PteV, PteR, PteW, PteX, PteU, PteG, PteA, PteD,
                              PagingMode, PagingLevel,
                              MInterrupt, MException, MTrapVecMode,
                              SInterrupt, SException, STrapVecMode,
                              Undefined };

  // only instruction info is special
  enum class ArchInfoGroup { Inst, Custom };

  /// Architectural coverage definition. Should be provided as a json file.
  template <typename URV>
  class ArchInfo
  {
  public:

    /// Copy JSON file
    ArchInfo(Hart<URV>& hart, std::string filename);

    /// generate architectural space
    nlohmann::json dumpArchInfo()
    {
      nlohmann::json j;
      for (auto& entry : entries_)
        {
          for (auto point : entry.points_)
            {
              switch (point)
                {
                  case ArchInfoPoint::Dest:             addDest(entry); break;
                  case ArchInfoPoint::Src:              addSrc(entry); break;
                  case ArchInfoPoint::Sew:              addSew(entry); break;
                  case ArchInfoPoint::Lmul:             addLmul(entry); break;
                  case ArchInfoPoint::PrivilegeMode:    addPrivilegeMode(entry); break;
                  case ArchInfoPoint::PteV:
                  case ArchInfoPoint::PteR:
                  case ArchInfoPoint::PteW:
                  case ArchInfoPoint::PteX:
                  case ArchInfoPoint::PteU:
                  case ArchInfoPoint::PteG:
                  case ArchInfoPoint::PteA:
                  case ArchInfoPoint::PteD:             addPteField(entry, point); break;
                  case ArchInfoPoint::PagingMode:       addPagingMode(entry); break;
                  case ArchInfoPoint::PagingLevel:      addPagingLevel(entry); break;
                  case ArchInfoPoint::MInterrupt:
                  case ArchInfoPoint::SInterrupt:       addInterrupt(entry, point);
                                                        break;
                  case ArchInfoPoint::MException:
                  case ArchInfoPoint::SException:       addException(entry, point);
                                                        break;
                  case ArchInfoPoint::MTrapVecMode:
                  case ArchInfoPoint::STrapVecMode:     addTrapVecMode(entry, point);
                                                        break;
                  default: break;
                }
            }
          j += nlohmann::json::object_t::value_type(entry.name_, entry.j_);
        }

      return j;
    }

  private:

    typedef struct
    {
      ArchInfoGroup group_;
      std::string name_;
      nlohmann::json j_;

      std::vector<ArchInfoPoint> points_;

      void addBins(const std::string point, nlohmann::json list)
      { j_ += nlohmann::json::object_t::value_type(point, list); }

    } ArchInfoEntry;

    /// Populate archinfo space depending on instruction attributes.
    void addInstPoints(ArchInfoEntry& entry)
    {
      InstEntry inst = hart_.getInstructionEntry(entry.name_);
      bool disable = not hart_.hasIsaExtension(inst.extension());
      if (inst.isCompressed())
        disable = disable and not hart_.isRvc();

      // ignore illegal EEW vector instructions (MEW=1, reserved)
      int mew = (inst.code() & 0x10000000);
      bool opMatch = not (((inst.code() & 0x27) ^ 0x27) and ((inst.code() & 0x7) ^ 0x7));
      disable = disable or (inst.isVector() and mew and opMatch);

      if (disable)
        {
          std::cerr << "Instruction " << entry.name_
                      << " not enabled by config, will ignore\n";
          return;
        }

      entry.points_.push_back(ArchInfoPoint::Dest);
      entry.points_.push_back(ArchInfoPoint::Src);

      if (inst.extension() == RvExtension::V)
        {
          entry.points_.push_back(ArchInfoPoint::Sew);
          entry.points_.push_back(ArchInfoPoint::Lmul);
        }
    }

    void addDest(ArchInfoEntry& entry) const;

    void addSrc(ArchInfoEntry& entry) const;

    void addSew(ArchInfoEntry& entry) const;

    void addLmul(ArchInfoEntry& entry) const;

    void addPrivilegeMode(ArchInfoEntry& entry) const;

    void addPteField(ArchInfoEntry& entry, ArchInfoPoint p) const;

    void addPagingMode(ArchInfoEntry& entry) const;

    void addPagingLevel(ArchInfoEntry& entry) const;

    void addInterrupt(ArchInfoEntry& entry, ArchInfoPoint p) const;

    void addException(ArchInfoEntry& entry, ArchInfoPoint p) const;

    void addTrapVecMode(ArchInfoEntry& entry, ArchInfoPoint p) const;

    ArchInfoGroup getGroup(std::string name) const
    {
      InstEntry inst = hart_.getInstructionEntry(name);
      return (inst.instId() != InstId::illegal) ? ArchInfoGroup::Inst
                : ArchInfoGroup::Custom;
    }

    static std::string toJsonHex(const unsigned num)
    {
      std::ostringstream oss;
      oss << "0x" << std::hex << num;
      return oss.str();
    }

    std::vector<ArchInfoEntry> entries_;
    Hart<URV>& hart_;
  };
}
