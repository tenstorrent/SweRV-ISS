// Copyright 2020 Western Digital Corporation or its affiliates.
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <nlohmann/json.hpp>
#include <charconv>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include "HartConfig.hpp"
#include "System.hpp"
#include "Core.hpp"
#include "Hart.hpp"


using namespace WdRiscv;


constexpr bool
isPowerOf2(uint64_t x)
{
  return x != 0 and (x & (x-1)) == 0;
}


HartConfig::HartConfig()
  : config_(std::make_unique<nlohmann::json>())
{
}


HartConfig::~HartConfig() = default;


bool
HartConfig::loadConfigFile(const std::string& filePath)
{
  std::ifstream ifs(filePath);
  if (not ifs.good())
    {
      std::cerr << "Failed to open config file '" << filePath
		<< "' for input.\n";
      return false;
    }

  try
    {
      // Use json::parse rather than operator>> to allow comments to be ignored
      *config_ = nlohmann::json::parse(ifs, nullptr /* callback */, true /* allow_exceptions */, true /* ignore_comments */);
    }
  catch (std::exception& e)
    {
      std::cerr << e.what() << "\n";
      return false;
    }
  catch (...)
    {
      std::cerr << "Caught unknown exception while parsing "
		<< " config file '" << filePath << "'\n";
      return false;
    }

  return true;
}


namespace WdRiscv
{
  
  /// Convert given json entry to an unsigned integer value honoring
  /// hexadecimal prefix (0x) if any. Return true on succes and false
  /// if given entry does not represent an integer.
  template <typename URV>
  bool
  getJsonUnsigned(std::string_view tag, const nlohmann::json& js, URV& value)
  {
    value = 0;

    if (js.is_number())
      {
        value = js.get<URV>();
        return true;
      }

    if (js.is_string())
      {
        char*            end = nullptr;
        std::string_view str = js.get<std::string_view>();
        uint64_t         u64 = strtoull(str.data(), &end, 0);
        if (end and *end)
          {
            std::cerr << "Invalid config file unsigned value for '" << tag << "': "
                      << str << '\n';
            return false;
          }
        value = static_cast<URV>(u64);
        if (value != u64)
          {
            std::cerr << "Overflow in config file value for '" << tag << "': "
                      << str << '\n';
            return false;
          }

        return true;
      }

    std::cerr << "Config file entry '" << tag << "' must contain a number\n";
    return false;
  }


  /// Convert given json array value to a vector of unsigned integers
  /// honoring any hexadecimal prefix (0x) if any. Return true on
  /// sucess an false on failure.
  template <typename URV>
  bool
  getJsonUnsignedVec(std::string_view tag, const nlohmann::json& js,
                     std::vector<URV>& vec)
  {
    vec.clear();

    if (not js.is_array())
      {
	std::cerr << "Invalid config file value for '" << tag << "'"
		  << " -- expecting array of numbers\n";
	return false;
      }

    unsigned errors = 0;

    for (const auto& item :js)
      {
	if (item.is_number())
	  vec.push_back(item.get<unsigned>());
	else if (item.is_string())
	  {
            char*            end = nullptr;
            std::string_view str = item.get<std::string_view>();
            uint64_t         u64 = strtoull(str.data(), &end, 0);
	    if (end and *end)
	      {
		std::cerr << "Invalid config file value for '" << tag << "': "
			  << str << '\n';
                errors++;
		continue;
	      }

            URV val = static_cast<URV>(u64);
            if (val != u64)
              {
                std::cerr << "Overflow in config file value for '" << tag << "': "
                          << str << '\n';
                errors++;
                continue;
              }

	    vec.push_back(val);
	  }
	else
          {
            std::cerr << "Invalid config file value for '" << tag << "'"
                      << " -- expecting array of number\n";
            errors++;
          }
      }

    return errors == 0;
  }


  /// Convert given json entry to a boolean value. Return ture on
  /// success and false on failure.
  bool
  getJsonBoolean(std::string_view tag, const nlohmann::json& js, bool& value)
  {
    value = false;

    if (js.is_boolean())
      {
        value = js.get<bool>();
        return true;
      }

    if (js.is_number())
      {
        value = js.get<unsigned>() != 0;
        return true;
      }

    if (js.is_string())
      {
        std::string_view str = js.get<std::string_view>();
        if (str == "0" or str == "false" or str == "False")
          value = false;
        else if (str == "1" or str == "true" or str == "True")
          value = true;
        else
          {
            std::cerr << "Invalid config file boolean value for '" << tag << "': "
                      << str << '\n';
            return false;
          }
        return true;
      }

    std::cerr << "Config file entry '" << tag << "' must contain a bool\n";
    return false;
  }

}


template <typename URV>
static
bool
applyCsrConfig(Hart<URV>& hart, std::string_view nm, const nlohmann::json& conf, bool verbose)
{
  unsigned errors = 0;
  URV reset = 0, mask = 0, pokeMask = 0;
  bool isDebug = false, exists = true, shared = false;

  std::string name(nm);
  if (name == "dscratch")
    name +=  "0";

  Csr<URV>* csr = hart.findCsr(name);
  if (csr)
    {
      reset = csr->getResetValue();
      mask = csr->getWriteMask();
      pokeMask = csr->getPokeMask();
      isDebug = csr->isDebug();
    }

  if (conf.contains("reset"))
    getJsonUnsigned(name + ".reset", conf.at("reset"), reset) or errors++;

  if (conf.contains("mask"))
    {
      if (not getJsonUnsigned(name + ".mask", conf.at("mask"), mask))
	errors++;

      // If defining a non-standard CSR (as popposed to configuring an
      // existing CSR) then default the poke-mask to the write-mask.
      if (not csr)
	pokeMask = mask;
    }

  if (conf.contains("poke_mask"))
    getJsonUnsigned(name + ".poke_mask", conf.at("poke_mask"), pokeMask) or errors++;

  if (conf.contains("debug"))
    getJsonBoolean(name + ".debug", conf.at("debug"), isDebug) or errors++;

  if (conf.contains("exists"))
    getJsonBoolean(name + ".exists", conf.at("exists"), exists) or errors++;

  if (conf.contains("shared"))
    getJsonBoolean(name + ".shared", conf.at("shared"), shared) or errors++;

  // If number present and csr is not defined, then define a new
  // CSR; otherwise, configure.
  if (conf.contains("number"))
    {
      unsigned number = 0;
      if (not getJsonUnsigned<unsigned>(name + ".number", conf.at("number"), number))
	errors++;
      else
	{
	  if (csr)
	    {
	      if (csr->getNumber() != CsrNumber(number))
		{
		  std::cerr << "Invalid config file entry for CSR "
			    << name << ": Number (0x" << std::hex << number
			    << ") does not match that of previous definition ("
			    << "0x" << unsigned(csr->getNumber())
			    << ")\n" << std::dec;
		  return false;
		}
	      // If number matches we configure below
	    }
	  else if (hart.defineCsr(name, CsrNumber(number), exists,
				  reset, mask, pokeMask, isDebug))
	    {
	      csr = hart.findCsr(name);
	      assert(csr);
	    }
	  else
	    {
	      std::cerr << "Invalid config file CSR definition with name "
			<< name << " and number 0x" << std::hex << number
			<< ": Number already in use\n" << std::dec;
	      return false;
	    }
	}
    }

  if (not csr)
    {
      std::cerr << "A CSR number must be provided in configuration of non-standard CSR "
		<< name << '\n';
      return false;
    }
  bool exists0 = csr->isImplemented(), isDebug0 = csr->isDebug();
  bool shared0 = csr->isShared();
  URV reset0 = csr->getResetValue(), mask0 = csr->getWriteMask();
  URV pokeMask0 = csr->getPokeMask();

  if (name == "mhartid" or name == "vlenb")
    {
      std::cerr << "CSR " << name << " cannot be configured.\n";
      return true;
    }

  if (name == "sstatus")
    {
      std::cerr << "CSR sstatus is a shadow of mstatus and cannot be configured.\n";
      return true;
    }

  if (errors)
    return false;

  if (not hart.configCsrByUser(name, exists, reset, mask, pokeMask, isDebug, shared))
    {
      std::cerr << "Invalid CSR (" << name << ") in config file.\n";
      return false;
    }
  if ((mask & pokeMask) != mask and hart.sysHartIndex() == 0)
    {
      std::cerr << "Warning: For CSR " << name << " poke mask (0x" << std::hex << pokeMask
		<< ") is not a superset of write\n  mask (0x" << mask << std::dec << ")."
		<< " Only bits set in both masks will be writable by CSR instructions.\n";
    }

  if (name == "misa")
    {
      // If an extension bit is writable, it should reset to 1.
      URV extBits = (URV(1) << 26) - 1;
      URV writeable = extBits & mask, writeableReset = extBits & mask & reset;
      if (writeable != writeableReset and hart.sysHartIndex() == 0)
	std::cerr << "Warning: Reset value of MISA should be 0x"
		  << std::hex << (reset | writeable) << std::dec
		  << " to be compatible with write mask.\n";
      if ((writeable & (URV(1) << ('E' - 'A'))) and hart.sysHartIndex() == 0)
	std::cerr << "Warning: Bit E of MISA cannot be writebale.\n";
      if ((reset & (1 << ('S' - 'A'))) and not (reset & (1 << ('U' - 'A'))))
        {
          std::cerr << "Invalid MISA in config file: cannot have S=1 and U=0.\n";
          return false;
        }
    }

  if (verbose)
    {
      if (exists0 != exists or isDebug0 != isDebug or reset0 != reset or
	  mask0 != mask or pokeMask0 != pokeMask)
	{
	  std::cerr << "Configuration of CSR (" << name <<
	    ") changed in config file:\n";

	  if (exists0 != exists)
	    std::cerr << "  implemented: " << exists0 << " to "
		      << exists << '\n';

	  if (isDebug0 != isDebug)
	    std::cerr << "  debug: " << isDebug0 << " to "
		      << isDebug << '\n';

	  if (shared0 != shared)
	    std::cerr << "  shared: " << shared0 << " to "
		      << shared << '\n';

	  if (reset0 != reset)
	    std::cerr << "  reset: 0x" << std::hex << reset0
		      << " to 0x" << reset << '\n' << std::dec;

	  if (mask0 != mask)
	    std::cerr << "  mask: 0x" << std::hex << mask0
		      << " to 0x" << mask << '\n' << std::dec;

	  if (pokeMask0 != pokeMask)
	    std::cerr << "  poke_mask: " << std::hex << pokeMask0
		      << " to 0x" << pokeMask << '\n' << std::dec;
	}
    }

  return true;
}


template <typename URV>
static
bool
applyCsrConfig(Hart<URV>& hart, const nlohmann::json& config, bool verbose)
{
  if (not config.contains("csr"))
    return true;  // Nothing to apply

  const auto& csrs = config.at("csr");
  if (not csrs.is_object())
    {
      std::cerr << "Invalid csr entry in config file (expecting an object)\n";
      return false;
    }

  unsigned errors = 0;
  for (auto it = csrs.begin(); it != csrs.end(); ++it)
    {
      std::string_view csrName = it.key();
      const auto& conf = it.value();

      std::string_view tag = "range";
      if (not conf.contains(tag))
	{
	  applyCsrConfig(hart, csrName, conf, verbose) or errors++;
	  continue;
	}

      std::vector<unsigned> range;
      if (not getJsonUnsignedVec(util::join("", "csr.", tag, ".range"), conf.at(tag), range)
	  or range.size() != 2 or range.at(0) > range.at(1))
	{
	  std::cerr << "Invalid range in CSR '" << csrName << "': " << conf.at(tag) << '\n';
	  errors++;
	  continue;
	}

      if (range.at(1) - range.at(0) > 256)
	{
	  std::cerr << "Invalid range in CSR '" << csrName << "': " << conf.at(tag)
		    << ": Range size greater than 256\n";
	  errors++;
	  continue;
	}

      for (unsigned n = range.at(0); n <= range.at(1); ++n)
	{
	  std::string strand = util::join("", csrName, std::to_string(n));
	  if (not applyCsrConfig(hart, strand, conf, verbose))
	    {
	      errors++;
	      break;
	    }
	}
    }

  return errors == 0;
}


template <typename URV>
static
bool
applyTriggerConfig(Hart<URV>& hart, const nlohmann::json& config)
{
  if (not config.contains("triggers"))
    return true;  // Nothing to apply

  const auto& triggers = config.at("triggers");
  if (not triggers.is_array())
    {
      std::cerr << "Invalid triggers entry in config file (expecting an array)\n";
      return false;
    }

  unsigned errors = 0;
  unsigned ix = 0;
  for (auto it = triggers.begin(); it != triggers.end(); ++it, ++ix)
    {
      const auto& trig = *it;
      std::string name = std::string("trigger") + std::to_string(ix);
      if (not trig.is_object())
	{
	  std::cerr << "Invalid trigger in config file triggers array "
		    << "(expecting an object at index " << ix << ")\n";
	  ++errors;
	  break;
	}
      bool ok = true;
      for (const auto& tag : {"reset", "mask", "poke_mask"})
        if (not trig.contains(tag))
          {
            std::cerr << "Trigger " << name << " has no '" << tag
                      << "' entry in config file\n";
            ok = false;
          }
      if (not ok)
        {
          errors++;
          continue;
        }

      std::vector<uint64_t> resets, masks, pokeMasks;
      ok = (getJsonUnsignedVec(name + ".reset", trig.at("reset"), resets) and
            getJsonUnsignedVec(name + ".mask", trig.at("mask"), masks) and
            getJsonUnsignedVec(name + ".poke_mask", trig.at("poke_mask"), pokeMasks));
      if (not ok)
        {
          errors++;
          continue;
        }

      // Each trigger has up to 5 components: tdata1, tdata2, tdata3, tinfo, tcontrol
      size_t maxSize = std::max(resets.size(), std::max(masks.size(), pokeMasks.size()));
      if (maxSize > 5)
	std::cerr << "Trigger " << name << ": Unreasonable item count (" << maxSize
		  << ") for 'reset/mask/poke_mask' field in config file. "
		  << " Expecting no more than to 5. Extra fields ignored.\n";

      if (resets.size() != maxSize or masks.size() != maxSize or pokeMasks.size() != maxSize)
	{
	  std::cerr << "Trigger " << name << ": Error: reset/mask/poke_mask fields must have "
		    << " same number of entries.\n";
	  errors++;
	  continue;
	}

      if (not hart.configTrigger(ix, resets, masks, pokeMasks))
	{
	  std::cerr << "Failed to configure trigger " << std::dec << ix << '\n';
	  ++errors;
	}
    }

  return errors == 0;
}


template <typename URV>
static
bool
applyPerfEventMap(Hart<URV>& hart, const nlohmann::json& config)
{
  constexpr std::string_view tag = "mmode_perf_event_map";
  if (not config.contains(tag))
    return true;

  const auto& perfMap = config.at(tag);
  if (not perfMap.is_object())
    {
      std::cerr << "Invalid " << tag << " entry in config file (expecting an object)\n";
      return false;
    }

  std::unordered_set<URV> eventNumbers;

  unsigned errors = 0;
  for (auto it = perfMap.begin(); it != perfMap.end(); ++it)
    {
      std::string_view eventName = it.key();
      const auto& valObj = it.value();
      std::string path = util::join(".", tag, eventName);
      URV value = 0;
      if (not getJsonUnsigned(path, valObj,  value))
	{
	  errors++;
	  continue;
	}

      EventNumber eventId = EventNumber::None;
      if (not PerfRegs::findEvent(eventName, eventId))
	{
	  std::cerr << "No such performance event: " << eventName << '\n';
	  errors++;
	  continue;
	}

      if (eventNumbers.contains(value))
	{
	  std::cerr << "Event number " << value << " associaged with more than one event in mmode_perf_event_map in config file.\n";
	  errors++;
	}
      hart.configEventNumber(value, eventId);
      eventNumbers.insert(value);
    }

  return errors == 0;
}


template <typename URV>
static
bool
applyPerfEvents(Hart<URV>& hart, const nlohmann::json& config,
                bool userMode, bool cof, bool /*verbose*/)
{
  unsigned errors = 0;

  std::string_view tag = "num_mmode_perf_regs";
  if (config.contains(tag))
    {
      unsigned count = 0;
      if (not getJsonUnsigned<unsigned>(tag, config.at(tag), count))
        errors++;
      else
        {
          if (not hart.configMachineModePerfCounters(count, cof))
            errors++;
          if (userMode)
            if (not hart.configUserModePerfCounters(count))
              errors++;
        }
    }

  unsigned maxPerfId = 0;
  tag = "max_mmode_perf_event";
  if (config.contains(tag))
    {
      if (not getJsonUnsigned<unsigned>(tag, config.at(tag), maxPerfId))
        errors++;
      else
        {
          unsigned limit = 16*1024;
          if (maxPerfId > limit)
            {
              std::cerr << "Config file max_mmode_perf_event too large -- Using "
                        << limit << '\n';
              maxPerfId = limit;
            }
          hart.configMachineModeMaxPerfEvent(maxPerfId);
        }
    }

  tag = "mmode_perf_events";
  if (config.contains(tag))
    {
      std::vector<unsigned> eventsVec;

      const auto& events = config.at(tag);
      if (not events.is_array())
        {
          std::cerr << "Invalid mmode_perf_events entry in config file (expecting an array)\n";
          errors++;
        }
      else
        {
          unsigned ix = 0;
          for (auto it = events.begin(); it != events.end(); ++it, ++ix)
            {
              const auto& event = *it;
              std::string elemTag = util::join("", tag, "element ", std::to_string(ix));
              unsigned eventId = 0;
              if (not getJsonUnsigned<unsigned>(elemTag, event, eventId))
                errors++;
              else
                eventsVec.push_back(eventId);
            }
        }
      hart.configPerfEvents(eventsVec);
    }

  if (not applyPerfEventMap(hart, config))
    errors++;

  return errors == 0;
}


// Min SEW per LMUL is allowed by the spec for m1, m2, m4, and m8.
bool
processMinBytesPerLmul(const nlohmann::json& jsonMap, unsigned minBytes, unsigned maxBytes,
		       std::unordered_map<GroupMultiplier, unsigned>& bytesPerLmul)
{
  if (not jsonMap.is_object())
    {
      std::cerr << "Invalid min_bytes_per_lmul entry in config file (expecting an object)\n";
      return false;
    }

  for (auto it = jsonMap.begin(); it != jsonMap.end(); it++)
    {
      GroupMultiplier group;
      std::string_view lmul = it.key();
      const unsigned mewb = it.value();  // min element width in bytes
      if (not VecRegs::to_lmul(lmul, group))
	{
	  std::cerr << "Invalid lmul setting in min_bytes_per_lmul: " << lmul << '\n';
	  return false;
	}
      if (group > GroupMultiplier::Eight)
	{
	  std::cerr << "Invalid lmul setting in min_bytes_per_lmul: " << lmul
		    << " (expecting non-fractional group)\n";
	  return false;
	}

      if (mewb < minBytes or mewb > maxBytes)
	{
	  std::cerr << "Error: Config file min_bytes_per_lmul ("
		    << mewb << ") must be in the range [" << minBytes
		    << "," << maxBytes << "]\n";
	  return false;
	}

      if (not isPowerOf2(mewb))
	{
	  std::cerr << "Error: config file min_bytes_per_lmul ("
		    << mewb << ") is not a power of 2\n";
	  return false;
	}

      bytesPerLmul[group] = mewb;
    }

  return true;
}

// Maximum SEW per LMUL is allowed by the spec for mf8, mf4, and mf2.
bool
processMaxBytesPerLmul(const nlohmann::json& jsonMap, unsigned minBytes, unsigned maxBytes,
		       std::unordered_map<GroupMultiplier, unsigned>& bytesPerLmul)
{
  if (not jsonMap.is_object())
    {
      std::cerr << "Invalid max_bytes_per_lmul tag in config file (expecting an object)\n";
      return false;
    }

  for (auto it = jsonMap.begin(); it != jsonMap.end(); it++)
    {
      GroupMultiplier group;
      std::string_view lmul = it.key();
      const unsigned mewb = it.value();  // max element width in bytes
      if (not VecRegs::to_lmul(lmul, group))
	{
	  std::cerr << "Invalid lmul setting in max_bytes_per_lmul: " << lmul << '\n';
	  return false;
	}
      if (group < GroupMultiplier::Eighth)
	{
	  std::cerr << "Invalid lmul setting in max_bytes_per_lmul: " << lmul
		    << " (expecting fractional group)\n";
	  return false;
	}

      if (mewb < minBytes or mewb > maxBytes)
	{
	  std::cerr << "Error: Config file max_bytes_per_lmul ("
		    << mewb << ") must be in the range [" << minBytes
		    << "," << maxBytes << "]\n";
	  return false;
	}

      if (not isPowerOf2(mewb))
	{
	  std::cerr << "Error: config file max_bytes_per_lmul  ("
		    << mewb << ") is not a power of 2\n";
	  return false;
	}

      bytesPerLmul[group] = mewb;
    }
  return true;
}


template <typename URV>
static
bool
applyVectorConfig(Hart<URV>& hart, const nlohmann::json& config)
{
  using namespace std::string_view_literals;

  if (not config.contains("vector"))
    return true;  // Nothing to apply

  unsigned errors = 0;
  const auto& vconf = config.at("vector");

  unsigned bytesPerVec = 0;
  std::string_view tag = "bytes_per_vec";
  if (not vconf.contains(tag))
    {
      std::cerr << "Error: Missing " << tag << " tag in vector section of config file\n";
      errors++;
    }
  else
    {
      if (not getJsonUnsigned(tag, vconf.at(tag), bytesPerVec))
        errors++;
      else if (bytesPerVec == 0 or bytesPerVec > 4096)
        {
          std::cerr << "Error: Invalid config file bytes_per_vec number: "
                    << bytesPerVec << '\n';
          errors++;
        }
      else if (not isPowerOf2(bytesPerVec))
	{
	  std::cerr << "Error: Config file bytes_per_vec ("
		    << bytesPerVec << ") is not a power of 2\n";
	  errors++;
	}
    }

  std::vector<unsigned> bytesPerElem = { 1, 1 };
  static constexpr auto tags = std::array{ "min_bytes_per_elem"sv, "max_bytes_per_elem"sv };
  for (size_t ix = 0; ix < tags.size(); ++ix)
    {
      unsigned bytes = 0;
      tag = tags.at(ix);
      if (not vconf.contains(tag))
	{
	  if (ix > 0)
	    {
	      std::cerr << "Error: Missing " << tag
			<< " tag in vector section of config file\n";
	      errors++;
	    }
	  continue;
	}

      if (not getJsonUnsigned(tag, vconf.at(tag), bytes))
        errors++;
      else if (bytes == 0 or bytes > bytesPerVec)
        {
          std::cerr << "Error: Invalid config file " << tag << "  number: "
                    << bytes << '\n';
          errors++;
        }
      else
        {
          if (not isPowerOf2(bytes))
            {
              std::cerr << "Error: Config file " << tag << " ("
                        << bytes << ") is not a power of 2\n";
              errors++;
            }
          else
	    bytesPerElem.at(ix) = bytes;
        }
    }

  std::unordered_map<GroupMultiplier, unsigned> minBytesPerLmul;
  tag = "min_sew_per_lmul";
  if (vconf.contains(tag))
    {
      std::cerr << "Tag min_sew_per_lmul is deprecated: Use min_bytes_per_lmul\n";
      if (not processMinBytesPerLmul(vconf.at(tag), bytesPerElem.at(0), bytesPerElem.at(1),
				     minBytesPerLmul))
	errors++;
    }

  tag = "min_bytes_per_lmul";
  if (vconf.contains(tag))
    {
      if (not processMinBytesPerLmul(vconf.at(tag), bytesPerElem.at(0), bytesPerElem.at(1),
				     minBytesPerLmul))
	errors++;
    }

  std::unordered_map<GroupMultiplier, unsigned> maxBytesPerLmul;
  tag = "max_sew_per_lmul";
  if (vconf.contains(tag))
    {
      std::cerr << "Tag max_sew_per_lmul is deprecated: Use max_bytes_per_lmul\n";
      if (not processMaxBytesPerLmul(vconf.at(tag), bytesPerElem.at(0), bytesPerElem.at(1),
				     maxBytesPerLmul))
	errors++;
    }

  tag = "max_bytes_per_lmul";
  if (vconf.contains(tag))
    {
      if (not processMaxBytesPerLmul(vconf.at(tag), bytesPerElem.at(0), bytesPerElem.at(1),
				     maxBytesPerLmul))
	errors++;
    }

  if (errors == 0)
    hart.configVector(bytesPerVec, bytesPerElem.at(0), bytesPerElem.at(1), &minBytesPerLmul,
		      &maxBytesPerLmul);

  tag = "mask_agnostic_policy";
  if (vconf.contains(tag))
    {
      auto& item = vconf.at(tag);
      if (not item.is_string())
	{
	  std::cerr << "Error: Configuration file tag vector.mask_agnostic_policy must have a string value\n";
	errors++;
	}
      else
	{
	  std::string val = item.get<std::string>();
	  if (val == "ones")
	    hart.configMaskAgnosticAllOnes(true);
	  else if (val == "undisturb")
	    hart.configMaskAgnosticAllOnes(false);
	  else
	    {
	      std::cerr << "Error: Configuration file tag vector.mask_agnostic_policy must be 'ones' or 'undisturb'\n";
	      errors++;
	    }
	}
    }

  tag = "tail_agnostic_policy";
  if (vconf.contains(tag))
    {
      auto& item = vconf.at(tag);
      if (not item.is_string())
	{
	  std::cerr << "Error: Configuration file tag vector.tail_agnostic_policy must have a string value\n";
	errors++;
	}
      else
	{
	  std::string val = item.get<std::string>();
	  if (val == "ones")
	    hart.configTailAgnosticAllOnes(true);
	  else if (val == "undisturb")
	    hart.configTailAgnosticAllOnes(false);
	  else
	    {
	      std::cerr << "Error: Configuration file tag vector.tail_agnostic_policy must be 'ones' or 'undisturb'\n";
	      errors++;
	    }
	}
    }

  tag = "trap_non_zero_vstart";
  if (vconf.contains(tag))
    {
      bool flag = false;
      if (not getJsonBoolean(tag, vconf.at(tag), flag))
        errors++;
      else
        hart.enableTrapNonZeroVstart(flag);
    }

  if (errors == 0)
    hart.configVector(bytesPerVec, bytesPerElem.at(0), bytesPerElem.at(1), &minBytesPerLmul,
		      &maxBytesPerLmul);

  tag = "update_whole_mask";
  if (vconf.contains(tag))
    {
      bool flag = false;
      if (not getJsonBoolean(tag, vconf.at(tag), flag))
        errors++;
      else
        hart.configVectorUpdateWholeMask(flag);
    }

  tag = "trap_invalid_vtype";
  if (vconf.contains(tag))
    {
      bool flag = false;
      if (not getJsonBoolean(tag, vconf.at(tag), flag))
        errors++;
      else
        hart.configVectorTrapVtype(flag);
    }

  tag = "tt_fp_usum_tree_reduction";
  if (vconf.contains(tag))
    {
      bool flag = false;
      if (not getJsonBoolean(tag, vconf.at(tag), flag))
        errors++;
      else
        hart.configVectorFpUnorderedSumRed(flag);
    }

  tag = "legalize_vsetvl_avl";
  if (vconf.contains(tag))
    {
      bool flag = false;
      if (not getJsonBoolean(tag, vconf.at(tag), flag))
        errors++;
      else
        hart.configVectorLegalizeVsetvlAvl(flag);
    }

  tag = "legalize_vsetvli_avl";
  if (vconf.contains(tag))
    {
      bool flag = false;
      if (not getJsonBoolean(tag, vconf.at(tag), flag))
        errors++;
      else
        hart.configVectorLegalizeVsetvliAvl(flag);
    }

  return errors == 0;
}


template <typename URV>
static
bool
applySteeConfig(Hart<URV>& hart, const nlohmann::json& config)
{
  if (not config.contains("stee"))
    return true;  // Nothing to apply

  unsigned errors = 0;
  const auto& sconf = config.at("stee");
  
  std::string tag = "zero_mask";
  if (sconf.contains(tag))
    {
      uint64_t mask = 0;
      if (not getJsonUnsigned(tag, sconf.at(tag), mask))
	errors++;
      else
	hart.configSteeZeroMask(mask);
    }

  tag = "secure_mask";
  if (sconf.contains(tag))
    {
      uint64_t mask = 0;
      if (not getJsonUnsigned(tag, sconf.at(tag), mask))
	errors++;
      else
	hart.configSteeSecureMask(mask);
    }

  tag = "secure_region";
  if (sconf.contains(tag))
    {
      std::vector<uint64_t> vec;
      if (not getJsonUnsignedVec(tag, sconf.at(tag), vec))
	errors++;
      else
	{
	  if (vec.size() != 2)
	    {
	      std::cerr << "Invalid config file stee.secure_region: Expecting an array of 2 integers\n";
	      errors++;
	    }
	  else
	    hart.configSteeSecureRegion(vec.at(0), vec.at(1));
	}
    }

  if (not errors)
    hart.enableStee(true);

  return errors == 0;
}


/// Collect the physical memory attributes from the given json object
/// (tagged "attribs") and add them to the given Pma object.  Return
/// true on success and false on failure. Path is the hierarchical
/// name of the condig object in the JSON configuration file.
/// Sample JSON input parse in this functions:
///     "attribs" : [ "read", "write", "exec", "amo", "rsrv" ]
static
bool
getConfigPma(std::string_view path, const nlohmann::json& attribs, Pma& pma)
{
  using std::cerr;

  if (not attribs.is_array())
    {
      cerr << "Error: Invalid \"attribs\" entry in configuraion item " << path
	   << " -- expecting an array\n";
      return false;
    }

  unsigned errors = 0;

  for (const auto& attrib : attribs)
    {
      if (not attrib.is_string())
	{
	  cerr << "Error: Invalid item value in config item " << path << ".attribs"
	       << " -- expecting a string\n";
	  errors++;
	  continue;
	}

      Pma::Attrib attr = Pma::Attrib::None;
      std::string_view valueStr = attrib.get<std::string_view>();
      if (not Pma::stringToAttrib(valueStr, attr))
	{
	  cerr << "Error: Invalid value in config item (" << valueStr << ") "
               << path << ".attribs" << '\n';
	  errors++;
	}
      else
	pma.enable(attr);
    }

  return errors == 0;
}


template <typename URV>
static
bool
processMemMappedMasks(Hart<URV>& hart, std::string_view path, const nlohmann::json& masks,
		      uint64_t low, uint64_t high, unsigned size)
{
  // Parse an array of entries, each entry is an array containing low
  // address, high address, and mask.
  unsigned ix = 0;
  unsigned errors = 0;
  for (auto maskIter = masks.begin(); maskIter != masks.end(); ++maskIter, ++ix)
    {
      const auto& entry = *maskIter;
      std::string entryPath = util::join("", path, ".masks[", std::to_string(ix), "]");
      std::vector<uint64_t> vec;
      if (not getJsonUnsignedVec(entryPath, entry, vec))
	{
	  errors++;
	  continue;
	}

      if (vec.size() != 3)
	{
	  std::cerr << "Error: Expecting 3 values for config item "
		    << entryPath << '\n';
	  errors++;
	  continue;
	}

      bool ok = vec.at(0) >= low and vec.at(0) <= high;
      ok = ok and vec.at(1) >= low and vec.at(1) <= high;
      if (not ok)
	{
	  std::cerr << "Error: Mask address out of PMA region bounds for config item "
		    << entryPath << '\n';
	  errors++;
	  continue;
	}

      uint64_t mask = vec.at(2);
      for (uint64_t addr = vec.at(0); addr <= vec.at(1); addr += size)
	if (not hart.setMemMappedMask(addr, mask, size))
	  {
	    std::cerr << "Error: Failed to configure mask for config item "
		      << entryPath << " at address 0x" << std::hex << addr
		      << std::dec << '\n';
	    errors++;
	  }
    }

  return errors == 0;
}


template <typename URV>
static
bool
applyPmaConfig(Hart<URV>& hart, const nlohmann::json& config)
{
  using std::cerr;

  if (not config.is_array())
    {
      cerr << "Error: Invalid memmap.pma entry in config file memmap (execpting an array)\n";
      return false;
    }

  unsigned errors = 0;
  unsigned ix = 0;
  for (auto it = config.begin(); it != config.end(); ++it, ++ix)
    {
      std::string path = std::string("memmap.pma[") + std::to_string(ix) + "]";

      const auto& item = *it;
      if (not item.is_object())
	{
	  cerr << "Error: Configuration item at" << path << " is not an object\n";
	  errors++;
	  continue;
	}

      unsigned itemErrors = 0;

      std::string_view tag = "low";
      uint64_t low = 0;
      if (not item.contains(tag))
	{
	  cerr << "Error: Missing entry \"low\" in configuration item " << path << "\n";
	  itemErrors++;
	}
      else if (not getJsonUnsigned(util::join(".", path, tag), item.at(tag), low))
	itemErrors++;

      tag = "high";
      uint64_t high = 0;
      if (not item.contains(tag))
	{
	  cerr << "Error: Missing entry \"high\" in configuration item " << path << "\n";
	  itemErrors++;
	}
      else if (not getJsonUnsigned(util::join(".", path, tag), item.at(tag), high))
	itemErrors++;

      tag = "attribs";
      if (not item.contains(tag))
	{
	  cerr << "Error: Missing entry \"attribs\" in configuration item " << path << "\n";
	  itemErrors++;
	}
      else
	{
	  Pma pma;
	  if (not getConfigPma(path, item.at(tag), pma))
	    itemErrors++;
	  if (not itemErrors)
	    {
	      if (not hart.definePmaRegion(ix, low, high, pma))
		itemErrors++;
	      else if (pma.isMemMappedReg())
		{
		  unsigned size = 4;
		  tag = "register_size";
		  if (item.contains(tag))
		    {
		      auto path2 = util::join(".", path, tag);
		      if (not getJsonUnsigned(path2, item.at(tag), size))
			itemErrors++;
		      else if (size != 4 and size != 8)
			{
			  cerr << "Error: Invalid size in config item " << path2 << '\n';
			  itemErrors++;
			}
		    }  

		  if ((low & (size - 1)) != 0 and not itemErrors)
		    {
		      cerr << "Error: Memory mapped region address (0x" << std::hex
			   << low << std::dec << ") must be aligned to its size ("
			   << size << '\n';
		      itemErrors++;
		    }

		  tag = "masks";
		  if (item.contains(tag) and not itemErrors)
		    if (not processMemMappedMasks(hart, path, item.at(tag), low, high, size))
		      itemErrors++;
		}
	    }
	}

      errors += itemErrors;
    }

  return errors == 0;
}


/// Return true if config has a defined pmacfg CSR. This is either a
/// pmacfg with no "exists" attribute or with "exists" attribute set
/// to true.
static bool
hasDefinedPmacfgCsr(const nlohmann::json& config)
{
  if (not config.contains("csr"))
    return false;  // No csr section

  const auto& csrs = config.at("csr");
  if (not csrs.is_object())
    return false;  // No csr section in this config.

  // We could have a pmacfg entry with a range of indices.
  if (csrs.contains("pmacfg"))
    {
      const auto& entry = csrs.at("pmacfg");
      if (entry.is_object())
	{
	  if (not entry.contains("exists"))
	    return true;
	  bool exists = false;
	  if (getJsonBoolean("csr.pmacfg", entry.at("exists"), exists) and exists)
	    return true;
	}
    }

  // We could have a specific pmacfg csr (example pmacfg0).
  for (unsigned i = 0; i < 64; ++i)
    {
      std::string name = "pmacfg";
      name += std::to_string(i);
      if (csrs.contains(name))
	{
	  const auto& entry = csrs.at(name);
	  if (not entry.contains("exists"))
	    return true;
	  bool exists = false;
	  if (getJsonBoolean(name + ".exists", entry.at("exists"), exists) and exists)
	    return true;
	}
    }

  return false;
}


template<typename URV>
bool
HartConfig::applyMemoryConfig(Hart<URV>& hart) const
{
  unsigned errors = 0;

  if (config_ -> contains("memmap"))
    {
      // Apply memory protection windows.
      const auto& memMap = config_ -> at("memmap");
      std::string_view tag = "pma";
      if (memMap.contains(tag))
	{
	  if (hasDefinedPmacfgCsr(*config_) and hart.sysHartIndex() == 0)
	    {
	      std::cerr << "Warning: Configuration file has both memmap pma "
			<< "and a pmacfg CSR. CSRs will override memmap.\n";
	    }
	  if (not applyPmaConfig(hart, memMap.at(tag)))
	    errors++;
	}
    }

  if (config_ -> contains("cache"))
      std::cerr << "Configuration entry 'cache' no longer supported -- ignored\n";

  return errors == 0;
}


template<typename URV>
bool
HartConfig::configAclint(System<URV>& system, Hart<URV>& hart, uint64_t clintStart,
                         uint64_t mswiOffset, bool hasMswi,
                         uint64_t mtimerOffset, uint64_t mtimeOffset, bool hasMtimer,
		         bool siOnReset) const
{
  // Define callback to recover a hart from a hart index. We do
  // this to avoid having the Hart class depend on the System class.
  auto indexToHart = [&system](unsigned ix) -> Hart<URV>* {
    return system.ithHart(ix).get();
  };

  hart.configAclint(clintStart + mswiOffset, hasMswi, clintStart + mtimerOffset,
                   clintStart + mtimeOffset, hasMtimer, siOnReset, indexToHart);
  return true;
}


template<typename URV>
bool
HartConfig::configInterruptor(System<URV>& system, Hart<URV>& hart,
			      uint64_t addr) const
{
  // Define callback to recover a hart from a hart index. We do
  // this to avoid having the Hart class depend on the System class.
  auto indexToHart = [&system](unsigned ix) -> Hart<URV>* {
    return system.ithHart(ix).get();
  };

  hart.configInterruptor(addr, indexToHart);
  return true;
}


template<typename URV>
bool
HartConfig::applyConfig(Hart<URV>& hart, bool userMode, bool verbose) const
{
  using std::cerr;
  unsigned errors = 0;

  // Define PC value after reset.
  std::string_view tag = "reset_vec";
  if (config_ -> contains(tag))
    {
      URV resetPc = 0;
      if (getJsonUnsigned(tag, config_ -> at(tag), resetPc))
        hart.defineResetPc(resetPc);
      else
        errors++;
    }

  // Define non-maskable-interrupt pc
  tag = "nmi_vec";
  if (config_ -> contains(tag))
    {
      URV nmiPc = 0;
      if (getJsonUnsigned(tag, config_ -> at(tag), nmiPc))
        hart.defineNmiPc(nmiPc);
      else
        errors++;
    }

  // Define exception-pc non-maskable-interrupt
  tag = "nmi_exception_vec";
  if (config_ -> contains(tag))
    {
      URV pc = 0;
      if (getJsonUnsigned(tag, config_ -> at(tag), pc))
        hart.defineNmiExceptionPc(pc);
      else
        errors++;
    }

  // Use ABI register names (e.g. sp instead of x2).
  bool flag = false;
  tag = "abi_names";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_->at(tag), flag) or errors++;
      hart.enableAbiNames(flag);
    }

  // Print memory address of load/store instruction in trace log.
  // tag = "print_load_store_address";  // Deprecated -- now always true.

  // Trace page table walk in log.
  tag = "trace_ptw";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.tracePtw(flag);
    }

  // Reservation size in bytes for the load-reserve (LR) instruction.
  // Default is 4 for rv32 and 8 for rv64. A reservation size smaller
  // than default has no effect.
  tag = "reservation_bytes";
  if (config_ -> contains(tag))
    {
      unsigned resBytes = sizeof(URV);
      if (getJsonUnsigned(tag, config_ ->at(tag), resBytes))
	{
	  if (isPowerOf2(resBytes))
	    hart.configReservationSize(resBytes);
	  else
	    {
	      cerr << "Error: Config file reservation_bytes ("
		   << resBytes << ") is not a power of 2\n";
	      errors++;
	    }
	}
      else
	errors++;
    }

  // Keep reservation on exception in SC.W/D.
  tag = "keep_reservation_on_sc_exception";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.keepReservationOnScException(flag);
    }

  // Enable debug triggers.
  tag = "enable_triggers";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.enableTriggers(flag);
    }

  // Enable firing triggers in machine mode even when interrupts are enabled.
  tag = "mmode_triggers_ok_with_ie";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.enableMmodeTriggersWithIe(flag);
    }

  // Enable performance counters.
  tag = "enable_performance_counters";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.enablePerformanceCounters(flag);
    }

  tag = "perf_count_atomic_load_store";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.perfCountAtomicLoadStore(flag);
    }

  tag = "perf_count_fp_load_store";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.perfCountFpLoadStore(flag);
    }

  for (std::string_view ztag : { "zba", "zbb", "zbc", "zbs", "zfh" , "zfhmin", "zknd",
		     "zkne", "zknh", "zbkb", "zbkx", "zksed", "zksh"} )
    {
      std::string etag = util::join("", "enable_", ztag);
      if (config_ -> contains(etag))
	cerr << "Config file tag \"" << etag << "\" deprecated: "
	     << "Add extension string \"" << ztag << "\" to \"isa\" tag instead.\n";
    }

  for (std::string_view ztag : { "zbe", "zbf", "zbm", "zbp", "zbr", "zbt" } )
    {
      std::string etag = util::join("", "enable_", ztag);
      if (config_ -> contains(etag))
	cerr << "Config file tag \"" << etag << "\" is no longer supported.\n";
    }

  // Counter overflow: sscofpmf extension
  std::string isa;
  bool cof = getIsa(isa) and isa.find("sscofpmf") != std::string::npos;

  tag = "enable_counter_overflow";
  if (config_ ->contains(tag))
    {
      cerr << "Config file tag \"enable_counter_overflow\" deprecated: "
	   << " Add extension string \"sscofpmf\" to \"isa\" tag instread.\n";
      getJsonBoolean(tag, config_ ->at(tag), cof) or errors++;
    }

  applyPerfEvents(hart, *config_, userMode, cof, verbose) or errors++;
  applyCsrConfig(hart, *config_, verbose) or errors++;
  applyTriggerConfig(hart, *config_) or errors++;

  // No longer needed here. Remove once enable_counter_overflow is removed.
  hart.enableSscofpmf(cof);

  tag = "trap_non_zero_vstart";
  if (config_ ->contains(tag))
    {
      std::cerr << "Configuration tag trap_non_zero_vstart should be in vector section.\n";
      bool flag = false;
      if (not getJsonBoolean(tag, config_ ->at(tag), flag))
        errors++;
      else
        hart.enableTrapNonZeroVstart(flag);
    }
  applyVectorConfig(hart, *config_) or errors++;

  applySteeConfig(hart, *config_) or errors++;

  tag = "load_data_trigger";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ -> at(tag), flag) or errors++;
      hart.configLoadDataTrigger(flag);
    }

  tag = "exec_opcode_trigger";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ -> at(tag), flag) or errors++;
      hart.configExecOpcodeTrigger(flag);
    }

  tag = "memmap";
  if (config_ -> contains(tag))
    {
      const auto& memmap = config_ -> at(tag);
      tag = "consoleio";
      if (memmap.contains(tag))
	{
          URV io = 0;
          if (getJsonUnsigned("memmap.consoleio", memmap.at(tag), io))
            hart.setConsoleIo(io);
          else
            errors++;
	}
    }

  tag = "syscall_slam_area";
  if (config_ -> contains(tag))
    {
      uint64_t addr = 0;
      if (getJsonUnsigned(tag, config_ -> at(tag), addr))
        {
          hart.defineSyscallSlam(addr);
          hart.enableLinux(true);
        }
      else
        errors++;
    }

  tag = "physical_memory_protection_grain";
  if (config_ -> contains(tag))
    {
      uint64_t size = 0;
      if (getJsonUnsigned<uint64_t>(tag, config_ -> at(tag), size))
        hart.configMemoryProtectionGrain(size);
      else
        errors++;
    }

  tag = "guest_interrupt_count";
  if (config_ -> contains(tag))
    {
      uint64_t size = 0;
      if (getJsonUnsigned<uint64_t>(tag, config_ -> at(tag), size))
        hart.configGuestInterruptCount(size);
      else
        errors++;
    }

  tag = "enable_misaligned_data";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.enableMisalignedData(flag);
    }

  tag = "misaligned_has_priority";
  if (config_ -> contains (tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.misalignedExceptionHasPriority(flag);
    }

  tag = "force_rounding_mode";
  if (config_ -> contains(tag))
    {
      std::string_view str = config_->at(tag).get<std::string_view>();
      if (str == "rne")
	hart.forceRoundingMode(RoundingMode::NearestEven);
      else if (str == "rtz")
	hart.forceRoundingMode(RoundingMode::Zero);
      else if (str == "rdn")
	hart.forceRoundingMode(RoundingMode::Down);
      else if (str == "rup")
	hart.forceRoundingMode(RoundingMode::Up);
      else if (str == "rmm")
	hart.forceRoundingMode(RoundingMode::NearestMax);
      else
	{
	  cerr << "Invalid force_rounding_mode config: " << str << '\n';
	  errors++;
	}
    }

  tag = "enable_csv_log";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ -> at(tag), flag) or errors++;
      hart.enableCsvLog(flag);
    }

  tag = "page_fault_on_first_access";
  if (config_ -> contains(tag))
    {
      if (hart.sysHartIndex() == 0)
	cerr << "Warning: Config tag " << tag << " is deprecated -- "
	     << "feature is now controlled by bit 61 of the MENVCFG/HENVCFG CSR.\n";
      getJsonBoolean(tag, config_ -> at(tag), flag) or errors++;
      // hart.setFaultOnFirstAccess(flag);
    }

  tag = "snapshot_periods";
  if (config_ -> contains(tag))
    {
      std::vector<uint64_t> periods;
      if (not getJsonUnsignedVec(tag, config_ -> at(tag), periods))
        errors++;
      else
        {
          std::sort(periods.begin(), periods.end());
          if (std::find(periods.begin(), periods.end(), 0)
                          != periods.end())
            {
              cerr << "Snapshot periods of 0 are ignored\n";
              periods.erase(std::remove(periods.begin(), periods.end(), 0), periods.end());
            }

          auto it = std::unique(periods.begin(), periods.end());
          if (it != periods.end())
            {
              periods.erase(it, periods.end());
              cerr << "Duplicate snapshot periods not supported, removed duplicates\n";
            }
        }
    }

  tag = "tlb_entries";
  if (config_ -> contains(tag))
    {
      unsigned size = 0;
      if (not getJsonUnsigned(tag, config_ -> at(tag), size))
        errors++;
      else
      {
        if ((size & (size - 1)) != 0)
          {
            cerr << "TLB size must be a power of 2\n";
            errors++;
          }
        else
          hart.setTlbSize(size);
      }
    }

  tag = "clear_mprv_on_ret";
  if (config_ -> contains(tag))
    {
      if (not getJsonBoolean(tag, config_ -> at(tag), flag))
        errors++;
      else
        hart.enableClearMprvOnRet(flag);
    }

  tag = "clear_mtval_on_illegal_instruction";
  if (config_ -> contains(tag))
    {
      if (not getJsonBoolean(tag, config_ -> at(tag), flag))
        errors++;
      else
        hart.enableClearMtvalOnIllInst(flag);
    }

  tag = "clear_mtval_on_ebreak";
  if (config_ -> contains(tag))
    {
      if (not getJsonBoolean(tag, config_ -> at(tag), flag))
        errors++;
      else
        hart.enableClearMtvalOnEbreak(flag);
    }

  // This is used to reduce the frequency of timer interupts. By
  // default the timer runs at the frequence of the instruction
  // counter. By adding a time shift, we put additional delay before
  // the timer expires (reaches timecmp).
  tag = "time_shift";
  if (config_ ->contains(tag))
    {
      if (hart.sysHartIndex() == 0)
	cerr << "Warning: Config tag time_shift may overflow timecmp and have "
	     << "unintended consequences.\n";
      unsigned shift = 0;
      if (not getJsonUnsigned(tag, config_ -> at(tag), shift))
	errors++;
      else
	hart.setTimeShift(shift);
    }

  // Same as above, but this is used to apply a scaling factor
  // instead of a simple shift.
  tag = "time_down_sample";
  if (config_ ->contains(tag))
    {
      unsigned n = 0;
      if (not getJsonUnsigned(tag, config_ -> at(tag), n))
	errors++;
      else
	hart.setTimeDownSample(n);
    }

  tag = "cancel_lr_on_ret";
  if (config_ -> contains(tag))
    {
      cerr << "Config tag cancel_lr_on_ret is deprecated. Use cancel_lr_on_trap.\n";
      if (not getJsonBoolean(tag, config_ -> at(tag), flag))
        errors++;
      else
        hart.enableCancelLrOnTrap(flag);
    }

  tag = "cancel_lr_on_trap";
  if (config_ -> contains(tag))
    {
      if (not getJsonBoolean(tag, config_ -> at(tag), flag))
        errors++;
      else
        hart.enableCancelLrOnTrap(flag);
    }


  tag = "debug_park_loop";
  if (config_ -> contains(tag))
    {
      URV dep = 0;
      if (not getJsonUnsigned(tag, config_ -> at(tag), dep))
        errors++;
      else
        hart.setDebugParkLoop(dep);
    }

  tag = "debug_trap_address";
  if (config_ -> contains(tag))
    {
      URV addr = 0;
      if (not getJsonUnsigned(tag, config_ -> at(tag), addr))
        errors++;
      else
        hart.setDebugTrapAddress(addr);
    }

  tag = "trace_pmp";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.tracePmp(flag);
    }

  tag = "trace_pma";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.tracePma(flag);
    }

  tag = "enable_pmp_tor";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.enablePmpTor(flag);
    }

  tag = "enable_pmp_na4";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.enablePmpNa4(flag);
    }

  tag = "address_translation_modes";
  if (config_ -> contains(tag))
    {
      unsigned atmErrors = 0;
      std::vector<VirtMem::Mode> modes;
      const auto& atm = config_ -> at(tag);
      for (const auto& item : atm)
	{
	  if (not item.is_string())
	    {
	      cerr << "Error: Invalid value in config file item " << tag
		   << " -- expecting string\n";
	      atmErrors++;
	      continue;
	    }
	  std::string_view modeStr = item.get<std::string_view>();
	  VirtMem::Mode mode;
	  if (not VirtMem::to_mode(modeStr, mode))
	    {
	      cerr << "Error no such address translation mode: " << tag << '\n';
	      atmErrors++;
	      continue;
	    }
	  modes.push_back(mode);
	}
      if (std::find(modes.begin(), modes.end(), VirtMem::Mode::Bare) == modes.end())
	{
	  cerr << "Bare mode added to config file address_translation_modes\n";
	  modes.push_back(VirtMem::Mode::Bare);
	}
      if (not atmErrors)
	hart.configAddressTranslationModes(modes);
      errors += atmErrors;
    }

  tag = "address_translation_pmms";
  if (config_ -> contains(tag))
    {
      unsigned atpErrors = 0;
      std::vector<VirtMem::Pmm> pmms;
      const auto& items = config_ -> at(tag);
      for (const auto& item : items)
	{
	  if (not item.is_string())
	    {
	      cerr << "Error: Invalid value in config file item " << tag
		   << " -- expecting string\n";
	      atpErrors++;
	      continue;
	    }
	  std::string_view pmmStr = item.get<std::string_view>();
	  VirtMem::Pmm pmm;
	  if (not VirtMem::to_pmm(pmmStr, pmm))
	    {
	      cerr << "Error no such address translation pmm: " << tag << '\n';
	      atpErrors++;
	      continue;
	    }
	  pmms.push_back(pmm);
	}
      if (not atpErrors)
	hart.configAddressTranslationPmms(pmms);
      errors += atpErrors;
    }

  tag = "enable_translation_pbmt";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.enableTranslationPbmt(flag);
    }

  tag = "enable_pbmt";
  if (config_ -> contains(tag))
    {
      std::cerr << "Config file tag enable_pbmt has been deprecated. Use enable_translation_pbmt.\n";
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.enableTranslationPbmt(flag);
      errors++;
    }      

  tag = "enable_translation_napot";
  if (config_ -> contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.enableTranslationNapot(flag);
    }

  tag = "enable_svinval";
  if (config_ -> contains(tag))
    {
      if (hart.sysHartIndex() == 0)
	cerr << "Warning: Config tag " << tag << " is deprecated. "
	     << "Use svinval with --isa instead.\n";
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.enableSvinval(flag);
    }

  tag = "enable_supervisor_time_compare";
  if (config_ -> contains(tag))
    {
      if (hart.sysHartIndex() == 0)
	cerr << "Warning: Config tag " << tag << " is deprecated. "
	     << "Use sstc with --isa instead.\n";
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.enableRvsstc(flag);
    }

  tag = "enable_aia";
  if (config_ ->contains(tag))
    {
      if (hart.sysHartIndex() == 0)
	cerr << "Warning: Config tag " << tag << " is deprecated. "
	     << "Use smaia with --isa instead.\n";
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.enableAiaExtension(flag);
    }

  tag = "enable_smstateen";
  if (config_ ->contains(tag))
    {
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.enableSmstaten(flag);
    }

  tag = "wfi_timeout";
  if (config_ ->contains(tag))
    {
      uint64_t timeout = 0;
      getJsonUnsigned(tag, config_ ->at(tag), timeout) or errors++;
      hart.setWfiTimeout(timeout);
    }

  tag = "hfence_gvma_ignores_gpa";
  if (config_ ->contains(tag))
    {
      bool flag = false;
      getJsonBoolean(tag, config_ ->at(tag), flag) or errors++;
      hart.hfenceGvmaIgnoresGpa(flag);
    }

  return errors == 0;
}


template<typename URV>
bool
HartConfig::applyClintConfig(System<URV>& system, Hart<URV>& hart) const
{
  std::string_view tag = "clint";
  if (not config_ -> contains(tag))
    return true;

  uint64_t addr = 0;
  if (not getJsonUnsigned(tag, config_ ->at(tag), addr))
    return false;

  if ((addr & 7) != 0)
    {
      std::cerr << "Error: Config file clint address (0x" << std::hex
		<< addr << std::dec << ") is not a multiple of 8\n";
      return false;
    }

  return configAclint(system, hart, addr, 0 /* swOffset */, true /* hasMswi */,
                      0x4000 /* timerOffset */, 0xbff8 /* timeOffset */, true /* hasMtimer */);
}


template<typename URV>
bool
HartConfig::applyAclintConfig(System<URV>& system, Hart<URV>& hart) const
{
  std::string_view tag = "aclint";
  if (not config_ -> contains(tag))
    return true;

  auto& aclint = config_ -> at("aclint");

  uint64_t base = 0, swOffset = 0, timerOffset = 0, timeOffset = 0;

  tag = "base";
  if (aclint.contains(tag))
    if (not getJsonUnsigned("aclint.base", aclint.at(tag), base))
      return false;

  bool hasMswi = false;
  tag = "sw_offset";
  if (aclint.contains(tag))
    {
      if (not getJsonUnsigned("aclint.sw_offset", aclint.at(tag), swOffset))
        return false;
      hasMswi = true;
    }
  uint64_t swEnd = swOffset + 0x4000;

  bool hasMtimer = false;
  tag = "timer_offset";
  if (aclint.contains(tag))
    {
      if (not getJsonUnsigned("aclint.timer_offset", aclint.at(tag), timerOffset))
        return false;
      hasMtimer = true;
    }
  uint64_t timerEnd = timerOffset + 0x8000;

  tag = "time_offset";
  if (aclint.contains(tag))
    {
      if (not hasMtimer)
        {
          std::cerr << "Error: aclint specified time_offset, but no timer_offset\n";
          return false;
        }
      if (not getJsonUnsigned("aclint.time_offset", aclint.at(tag), timeOffset))
        return false;
    }
  else if (hasMtimer)
    {
      std::cerr << "Error: aclint specified timer_offset, but no time_offset\n";
      return false;
    }

  if ((base & 7) != 0 or (swOffset & 7) != 0 or (timerOffset & 7) != 0 or
      (timeOffset & 7) != 0)
    {
      std::cerr << "Error: Config file aclint addresses and offsets\n"
                << "(0x" << std::hex << base << ")\n"
                << "(0x" << swOffset << ")\n"
                << "(0x" << timerOffset << ")\n"
                << "(0x" << timeOffset << std::dec << ")\n"
                << "must be a multiple of 8\n";
      return false;
    }

  // Check overlap.
  if (hasMswi and (timeOffset >= swOffset and timeOffset < swEnd))
    {
      std::cerr << "Error: aclint MTIME cannot sit in MSWI region.\n";
      return false;
    }

  if (hasMswi and hasMtimer and
      ((timerOffset >= swOffset and timerOffset < swEnd) or
       (swOffset >= timerOffset and swOffset < timerEnd)))
    {
      std::cerr << "Error: aclint MTIMER and MSWI regions cannot overlap.\n";
      return false;
    }

  bool siOnReset = false;
  tag = "software_interrupt_on_reset";
  if (aclint.contains(tag))
    {
      if (not getJsonBoolean("aclint.software_interrupt_on_reset", aclint.at(tag), siOnReset))
        return false;
      if (not hasMswi)
        {
          std::cerr << "Error: aclint software_interrupt_on_reset configured"
                    << " without software device enabled.\n";
          return false;
        }
    }

  return configAclint(system, hart, base, swOffset, hasMswi,
                     timerOffset, timeOffset, hasMtimer, siOnReset);
}


template<typename URV>
bool
HartConfig::applyImsicConfig(System<URV>& system) const
{
  if (not config_ -> contains("imsic"))
    return true;

  auto& hart0 = *system.ithHart(0);
  if (not hart0.extensionIsEnabled(RvExtension::Smaia))
    {
      std::cerr << "Cannot configure IMSIC without enabling Smaia\n";
      return false;
    }

  auto& imsic = config_ -> at("imsic");

  uint64_t mbase = 0, mstride = 0, sbase = 0, sstride = 0;

  std::string_view tag = "mbase";
  if (imsic.contains(tag))
    if (not getJsonUnsigned("imsic.mbase",  imsic.at(tag), mbase))
      return false;

  tag = "mstride";
  if (imsic.contains(tag))
    if (not getJsonUnsigned("imsic.mstride",  imsic.at(tag), mstride))
      return false;

  tag = "sbase";
  if (imsic.contains(tag))
    if (not getJsonUnsigned("imsic.sbase",  imsic.at(tag), sbase))
      return false;

  tag = "sstride";
  if (imsic.contains(tag))
    if (not getJsonUnsigned("imsic.stride",  imsic.at(tag), sstride))
      return false;

  unsigned guests = 0;
  tag = "guests";
  if (imsic.contains(tag))
    if (not getJsonUnsigned("imsic.guests", imsic.at(tag), guests))
      return false;

  unsigned ids = 64;
  tag = "ids";
  if (imsic.contains(tag))
    if (not getJsonUnsigned("imsic.ids", imsic.at(tag), ids))
      return false;

  bool trace = false;
  tag = "trace";
  if (imsic.contains(tag))
    if (not getJsonBoolean("imsic.trace", imsic.at(tag), trace))
      return false;

  return system.configImsic(mbase, mstride, sbase, sstride, guests, ids, trace);
}


template<typename URV>
bool
HartConfig::applyPciConfig(System<URV>& system) const
{
  std::string_view tag = "pci";
  if (not config_ -> contains(tag))
    return true;

  auto& pci = config_ -> at(tag);
  if (not pci.contains("config_base") or not pci.contains("mmio_base") or not pci.contains("mmio_size"))
    {
      std::cerr << "Invalid pci entry in config file\n";
      return false;
    }
  uint64_t configBase = 0, mmioBase = 0, mmioSize = 0;
  if (not getJsonUnsigned(util::join("", tag, ".config_base"), pci.at("config_base"), configBase) or
      not getJsonUnsigned(util::join("", tag, ".mmio_base"), pci.at("mmio_base"), mmioBase) or
      not getJsonUnsigned(util::join("", tag, ".mmio_size"), pci.at("mmio_size"), mmioSize))
    return false;
  unsigned buses = 0, slots = 0;
  if (not getJsonUnsigned(util::join("", tag, ".buses"), pci.at("buses"), buses) or
      not getJsonUnsigned(util::join("", tag, ".slots"), pci.at("slots"), slots))
    return false;

  return system.configPci(configBase, mmioBase, mmioSize, buses, slots);
}


template<typename URV>
bool
HartConfig::configHarts(System<URV>& system, bool userMode, bool verbose) const
{
  userMode = userMode or this->userModeEnabled();

  // Apply JSON configuration to each hart.
  for (unsigned i = 0; i < system.hartCount(); ++i)
    {
      Hart<URV>& hart = *system.ithHart(i);
      if (not applyConfig(hart, userMode, verbose))
	return false;

      if (not applyClintConfig(system, hart))
        return false;

      if (not applyAclintConfig(system, hart))
	return false;
    }

  unsigned mbLineSize = 64;
  std::string_view tag = "merge_buffer_line_size";
  if (config_ -> contains(tag))
    if (not getJsonUnsigned(tag, config_ -> at(tag), mbLineSize))
      return false;

  tag = "merge_buffer_check_all";
  bool checkAll = false;
  if (config_ -> contains(tag))
    if (not getJsonBoolean(tag, config_ -> at(tag), checkAll))
      return false;

  tag = "enable_memory_consistency";
  bool enableMcm = false;
  if (config_ -> contains(tag))
    if (not getJsonBoolean(tag, config_ -> at(tag), enableMcm))
      return false;

  if (enableMcm and not system.enableMcm(mbLineSize, checkAll))
    return false;

  tag = "enable_tso";
  bool enableTso = false;
  if (config_ -> contains(tag))
    if (not getJsonBoolean(tag, config_ -> at(tag), enableTso))
      return false;
  system.enableTso(enableTso);

  tag = "uart";
  if (config_ -> contains(tag))
    {
      auto& uart = config_ -> at(tag);
      if (not uart.contains("address") or not uart.contains("size"))
	{
	  std::cerr << "Invalid uart entry in config file: missing address/size entry.\n";
	  return false;
	}
      uint64_t addr = 0, size = 0;
      if (not getJsonUnsigned(util::join("", tag, ".address"), uart.at("address"), addr) or
          not getJsonUnsigned(util::join("", tag, ".size"), uart.at("size"), size))
	return false;

      std::string type = "uart8250";
      if (not uart.contains("type"))
	std::cerr << "Missing uart type. Using uart250. Valid types: uart8250, usartsf.\n";
      else
	{
	  type = uart.at("type").get<std::string>();
	  if (type != "uartsf" and type != "uart8250")
	    {
	      std::cerr << "Invalid uart type: " << type << ". Valid types: uartsf, uart8250.\n";
	      return false;
	    }
	}
		
      if (not system.defineUart(type, addr, size))
	return false;
    }

  if (not applyPciConfig(system))
    return false;

  return finalizeCsrConfig(system);
}


template<typename URV>
bool
HartConfig::configMemory(System<URV>& system, bool unmappedElfOk) const
{
  system.checkUnmappedElf(not unmappedElfOk);

  auto& hart0 = *system.ithHart(0);
  return applyMemoryConfig(hart0);
}


bool
HartConfig::getXlen(unsigned& xlen) const
{
  if (config_ -> contains("xlen"))
    {
      std::cerr << "Config file tag xlen is deprecated: xlen is obtained from the isa tag.\n";
      return getJsonUnsigned("xlen", config_ -> at("xlen"), xlen);
    }
  std::string isa;
  if (not getIsa(isa))
    return false;
  if (isa.size() >= 4 and isa.at(0) == 'r' and isa.at(1) == 'v')
    {
      unsigned len;
      if (auto convResult = std::from_chars(isa.c_str() + 2, isa.c_str() + isa.size(), len);
          convResult.ec == std::errc{})
        {
          if (len == 32 or len == 64)
            {
              xlen = len;
              return true;
            }
        }
      std::cerr << "Invalid register width in isa string (" << isa << ") in config file -- ignored\n";
    }
  return false;
}


bool
HartConfig::getCoreCount(unsigned& count) const
{
  if (config_ -> contains("cores"))
    return getJsonUnsigned("cores", config_ -> at("cores"), count);
  return false;
}


bool
HartConfig::getHartsPerCore(unsigned& count) const
{
  if (config_ -> contains("harts"))
    return getJsonUnsigned("harts", config_ -> at("harts"), count);
  return false;
}


bool
HartConfig::getPageSize(size_t& pageSize) const
{
  if (not config_ -> contains("memmap"))
    return false;

  auto& mem = config_ -> at("memmap");
  if (not mem.contains("page_size"))
    return false;

  return getJsonUnsigned("memmap.page_size", mem.at("page_size"), pageSize);
}


bool
HartConfig::getHartIdOffset(unsigned& offset) const
{
  constexpr std::string_view tag = "core_hart_id_offset";
  if (not config_ -> contains(tag))
    return false;

  return getJsonUnsigned(tag, config_->at(tag), offset);
}


bool
HartConfig::getIsa(std::string& isa) const
{
  constexpr std::string_view tag = "isa";
  if (not config_ -> contains(tag))
    return false;

  auto& item = config_ -> at(tag);
  if (item.is_string())
    {
      isa = item.get<std::string>();
      return true;
    }
  return false;
}


bool
HartConfig::getMemorySize(size_t& memSize) const
{
  if (not config_ -> contains("memmap"))
    return false;

  auto& mem = config_ -> at("memmap");
  if (not mem.contains("size"))
    return false;

  return getJsonUnsigned("memmap.size", mem.at("size"), memSize);
}


bool
HartConfig::getMcmLineSize(unsigned& ls) const
{
  constexpr std::string_view tag = "merge_buffer_line_size";
  if (not config_ -> contains(tag))
    return false;
  return getJsonUnsigned(tag, config_ -> at(tag), ls);
}


bool
HartConfig::getMcmCheckAll(bool& ca) const
{
  constexpr std::string_view tag = "merge_buffer_check_all";
  if (not config_ -> contains(tag))
    return false;
  return getJsonBoolean(tag, config_ -> at(tag), ca);
}


bool
HartConfig::userModeEnabled() const
{
  uint64_t resetVal = 0;
  if (not getMisaReset(resetVal))
    return false;

  // User mode enabled if bit corresponding to U extension is set.
  return ((uint64_t(1) << ('u' - 'a')) & resetVal) != 0;
}


bool
HartConfig::supervisorModeEnabled() const
{
  uint64_t resetVal = 0;
  if (not getMisaReset(resetVal))
    return false;

  // User mode enabled if bit corresponding to S extension is set.
  return ((uint64_t(1) << ('s' - 'a')) & resetVal) != 0;
}


void
HartConfig::clear()
{
  config_ -> clear();
}


/// Associate callback with write/poke of mcounthinibit
template <typename URV>
void
defineMcountinhibitSideEffects(System<URV>& system)
{
  for (unsigned i = 0; i < system.hartCount(); ++i)
    {
      auto hart = system.ithHart(i);
      auto csrPtr = hart->findCsr("mcountinhibit");
      if (not csrPtr)
        continue;

      std::weak_ptr<Hart<URV>> wHart(hart);

      // For poke, the effect takes place immediately (next instruction
      // will see the new control).
      auto postPoke = [wHart] (Csr<URV>&, URV val) -> void {
		        auto hart = wHart.lock();
			if (not hart)
			  return;  // Should not happen.
                        hart->setPerformanceCounterControl(~val);
                        hart->setPerformanceCounterControl(~val);
                      };

      // For write (invoked from current instruction), the effect
      // takes place on the following instruction.
      auto postWrite = [wHart] (Csr<URV>&, URV val) -> void {
	                 auto hart = wHart.lock();
			 if (not hart)
			   return;  // Should not happen.
                         hart->setPerformanceCounterControl(~val);
                       };

      csrPtr->registerPostPoke(postPoke);
      csrPtr->registerPostWrite(postWrite);
    }
}


template <typename URV>
bool
HartConfig::finalizeCsrConfig(System<URV>& system) const
{
  if (system.hartCount() == 0)
    return false;

  // Make shared CSRs in each hart except first one in core point to
  // the corresponding values in the first hart zero.
  for (unsigned ci = 0; ci < system.coreCount(); ++ci)
    {
      auto corePtr = system.ithCore(ci);
      if (not corePtr)
        continue;

      auto hart0 = corePtr->ithHart(0);
      if (not hart0)
        continue;

      for (unsigned hi = 1; hi < corePtr->hartCount(); ++hi)
        {
          auto hartPtr = corePtr->ithHart(hi);
          if (hartPtr)
            hartPtr->tieSharedCsrsTo(*hart0);
        }
    }

  // Define callback to react to write/poke to mcountinhibit CSR.
  defineMcountinhibitSideEffects(system);
  return true;
}


bool
HartConfig::getMisaReset(uint64_t& val) const
{
  val = 0;

  if (not config_ -> contains("csr"))
    return false;  // No csr section

  const auto& csrs = config_ -> at("csr");
  if (not csrs.is_object())
    return false;  // No csr section in this config.

  if (not csrs.contains("misa"))
    return false;  // CSR misa not present in csr section

  const auto& misa = csrs.at("misa");
  if (not misa.is_object())
    return false;

  if (not misa.contains("reset"))
    return false;  // No reset entry under misa

  uint64_t resetVal = 0;
  if (not getJsonUnsigned("csr.misa.reset", misa.at("reset"), resetVal))
    return false;

  val = resetVal;
  return true;
}


bool
HartConfig::hasCsrConfig(std::string_view csrName) const
{
  if (not config_ -> contains("csr"))
    return false;  // No csr section

  const auto& csrs = config_ -> at("csr");
  if (not csrs.is_object())
    return false;  // No csr section in this config.

  if (not csrs.contains(csrName))
    return false;  // Target csr not present in csr section

  return true;
}


// Instantiate tempate member functions

template bool
HartConfig::applyConfig<uint32_t>(Hart<uint32_t>&, bool, bool) const;

template bool
HartConfig::applyConfig<uint64_t>(Hart<uint64_t>&, bool, bool) const;

template bool
HartConfig::configHarts<uint32_t>(System<uint32_t>&, bool, bool) const;

template bool
HartConfig::configHarts<uint64_t>(System<uint64_t>&, bool, bool) const;

template bool
HartConfig::configMemory(System<uint32_t>&, bool) const;

template bool
HartConfig::configMemory(System<uint64_t>&, bool) const;


template bool
HartConfig::applyMemoryConfig<uint32_t>(Hart<uint32_t>&) const;

template bool
HartConfig::applyMemoryConfig<uint64_t>(Hart<uint64_t>&) const;

template bool
HartConfig::finalizeCsrConfig<uint32_t>(System<uint32_t>&) const;

template bool
HartConfig::finalizeCsrConfig<uint64_t>(System<uint64_t>&) const;

template bool
HartConfig::configAclint<uint32_t>(System<uint32_t>&, Hart<uint32_t>&, uint64_t clintStart,
                                   uint64_t mswiOffset, bool hasMswi,
                                   uint64_t mtimerOffset, uint64_t mtimeOffset, bool hasMtimer,
		                   bool siOnReset = false) const;
template bool
HartConfig::configAclint<uint64_t>(System<uint64_t>&, Hart<uint64_t>&, uint64_t clintStart,
                                   uint64_t mswiOffset, bool hasMswi,
                                   uint64_t mtimerOffset, uint64_t mtimeOffset, bool hasMtimer,
		                   bool siOnReset = false) const;

template bool
HartConfig::configInterruptor<uint32_t>(System<uint32_t>& system, Hart<uint32_t>& hart,
					uint64_t addr) const;

template bool
HartConfig::configInterruptor<uint64_t>(System<uint64_t>& system, Hart<uint64_t>& hart,
					uint64_t addr) const;

template bool
HartConfig::applyImsicConfig(System<uint32_t>&) const;

template bool
HartConfig::applyImsicConfig(System<uint64_t>&) const;
