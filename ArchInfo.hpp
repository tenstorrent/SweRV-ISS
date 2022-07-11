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
  enum class ArchInfoPoint { Dest, Src, Sew, Lmul, Frm, Fflags,
                              Mode, Exceptions, Undefined };

  enum class ArchInfoGroup { Inst, Paging, Custom };

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
          for (auto& point : entry.crosses_)
            {
              switch (point)
                {
                  case ArchInfoPoint::Dest: addDestBins(entry); break;
                  case ArchInfoPoint::Src:  addSrcBins(entry); break;
                  case ArchInfoPoint::Sew:  addSewBins(entry); break;
                  case ArchInfoPoint::Lmul: addLmulBins(entry); break;
                  case ArchInfoPoint::Mode: addModeBins(entry); break;
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
      std::vector<ArchInfoPoint> crosses_;
    } ArchInfoEntry;

    /// Populate coverage space depending on instruction definition.
    ///  By default will include:
    ///      all - dest/src operands
    ///      ld/st - alignment
    ///      vector - sew, lmul
    ///      branch - taken/not taken
    ///      fp - frm, fflags
    void addInstPoints(ArchInfoEntry& entry);

    bool addDestBins(ArchInfoEntry& entry) const;

    bool addSrcBins(ArchInfoEntry& entry) const;

    bool addSewBins(ArchInfoEntry& entry) const;

    bool addLmulBins(ArchInfoEntry& entry) const;

    bool addModeBins(ArchInfoEntry& entry) const;

    ArchInfoGroup getGroup(std::string name) const
    {
      InstEntry inst = hart_.instTable_.getEntry(name);
      if (inst.instId() != InstId::illegal)
        return ArchInfoGroup::Inst;
      else
        return ArchInfoGroup::Custom;
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
