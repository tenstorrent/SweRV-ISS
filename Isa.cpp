#include <algorithm>
#include <cassert>
#include <charconv>
#include <iostream>
#include "Isa.hpp"


using namespace WdRiscv;

// Use this constant list to allow a compile-time check to ensure each extension has a
// value.
static constexpr auto STRING_EXT_PAIRS = std::to_array<std::pair<std::string_view, RvExtension>>({
  { "a", RvExtension::A },
  { "b", RvExtension::B },
  { "c", RvExtension::C },
  { "d", RvExtension::D },
  { "e", RvExtension::E },
  { "f", RvExtension::F },
  { "h", RvExtension::H },
  { "i", RvExtension::I },
  { "m", RvExtension::M },
  { "n", RvExtension::N },
  { "s", RvExtension::S },
  { "u", RvExtension::U },
  { "v", RvExtension::V },
  { "zba", RvExtension::Zba },
  { "zbb", RvExtension::Zbb },
  { "zbc", RvExtension::Zbc },
  { "zbe", RvExtension::Zbe },
  { "zbf", RvExtension::Zbf },
  { "zbm", RvExtension::Zbm },
  { "zbp", RvExtension::Zbp },
  { "zbr", RvExtension::Zbr },
  { "zbs", RvExtension::Zbs },
  { "zbt", RvExtension::Zbt },
  { "zfh", RvExtension::Zfh },
  { "zfhmin", RvExtension::Zfhmin },
  { "zlssegh", RvExtension::Zlsseg },
  { "zknd", RvExtension::Zknd },
  { "zkne", RvExtension::Zkne },
  { "zknh", RvExtension::Zknh },
  { "zbkb", RvExtension::Zbkb },
  { "zbkx", RvExtension::Zbkx },
  { "zksed", RvExtension::Zksed },
  { "zksh", RvExtension::Zksh },
  { "svinval", RvExtension::Svinval },
  { "zicbom", RvExtension::Zicbom },
  { "zicboz", RvExtension::Zicboz },
  { "zawrs", RvExtension::Zawrs },
  { "zmmul", RvExtension::Zmmul },
  { "zvfh", RvExtension::Zvfh },
  { "zvfhmin", RvExtension::Zvfhmin },
  { "zvbb", RvExtension::Zvbb },
  { "zvbc", RvExtension::Zvbc },
  { "zvkg", RvExtension::Zvkg },
  { "zvkned", RvExtension::Zvkned },
  { "zvknha", RvExtension::Zvknha },
  { "zvknhb", RvExtension::Zvknhb },
  { "zvksed", RvExtension::Zvksed },
  { "zvksh", RvExtension::Zvksh },
  { "zicond", RvExtension::Zicond },
  { "zcb", RvExtension::Zcb },
  { "zfa", RvExtension::Zfa },
  { "zfbfmin", RvExtension::Zfbfmin },
  { "zvfbfmin", RvExtension::Zvfbfmin },
  { "zvfbfwma", RvExtension::Zvfbfwma },
  { "sstc", RvExtension::Sstc },
  { "svpbmt", RvExtension::Svpbmt },
});
static_assert(STRING_EXT_PAIRS.size() == static_cast<unsigned>(RvExtension::None));

const std::unordered_map<std::string_view, RvExtension> Isa::stringToExt_(STRING_EXT_PAIRS.cbegin(),
                                                                          STRING_EXT_PAIRS.cend());

// Use this function to do the constant initialization to allow use of indices
template <size_t N, unsigned (*TO_INDEX)(RvExtension)>
static constexpr std::array<std::string_view, N>
buildExtToStr()
{
  std::array<std::string_view, N> extToString;
  for (auto&& [name, id] : STRING_EXT_PAIRS)
    {
      extToString.at(TO_INDEX(id)) = name;
    }
  return extToString;
}

const std::array<std::string_view, Isa::extIx(RvExtension::None)> Isa::extToString_ =
  buildExtToStr<Isa::extIx(RvExtension::None), Isa::extIx>();

Isa::Isa()
{
  infoVec_.at(extIx(RvExtension::A)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(RvExtension::B)) = Info{ {{0,93}}, {0,93} };
  infoVec_.at(extIx(RvExtension::C)) = Info{ {{1,0}, {2,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::D)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(RvExtension::E)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(RvExtension::F)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(RvExtension::H)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::I)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(RvExtension::M)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(RvExtension::S)) = Info{ {{1,2}}, {1,2} };
  infoVec_.at(extIx(RvExtension::U)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::V)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zba)) = Info{ {{0,93}}, {0,93} };
  infoVec_.at(extIx(RvExtension::Zbb)) = Info{ {{0,93}}, {0,93} };
  infoVec_.at(extIx(RvExtension::Zbc)) = Info{ {{0,93}}, {0,93} };
  infoVec_.at(extIx(RvExtension::Zbs)) = Info{ {{0,93}}, {0,93} };
  infoVec_.at(extIx(RvExtension::Zfh)) = Info{ {{0,1}}, {0,1} };
  infoVec_.at(extIx(RvExtension::Zfhmin)) = Info{ {{0,1}}, {0,1} };
  infoVec_.at(extIx(RvExtension::Zlsseg)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zknd)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zkne)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zknh)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zbkb)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zbkx)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zksed)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zksh)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Svinval)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zicbom)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zicboz)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zawrs)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zmmul)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zvfh)) = Info{ {{0,1}}, {0,1} };
  infoVec_.at(extIx(RvExtension::Zvfhmin)) = Info{ {{0,1}}, {0,1} };
  infoVec_.at(extIx(RvExtension::Zfbfmin)) = Info{ {{0,8}}, {0,8} };
  infoVec_.at(extIx(RvExtension::Zvfbfmin)) = Info{ {{0,8}}, {0,8} };
  infoVec_.at(extIx(RvExtension::Zvfbfwma)) = Info{ {{0,8}}, {0,8} };
  infoVec_.at(extIx(RvExtension::Sstc)) = Info{ {{0,5}}, {0,5} };
  infoVec_.at(extIx(RvExtension::Svpbmt)) = Info{ {{1,0}}, {1,0} };

  infoVec_.at(extIx(RvExtension::I)).enabled = true; // I always enabled.
}


bool
Isa::selectVersion(RvExtension ext, unsigned version, unsigned subversion)
{
  unsigned ix = extIx(ext);
  if (ix >= infoVec_.size())
    return false;

  Info& info = infoVec_.at(ix);
  if (not info.supported)
    return false;

  VersionPair target{version, subversion};

  for (auto& vp : info.versions)
    if (vp == target)
      {
	info.selected = target;
	return true;
      }

  return false;
}


bool
Isa::isSupported(RvExtension ext) const
{
  unsigned ix = extIx(ext);
  if (ix >= infoVec_.size())
    return false;

  const Info& info = infoVec_.at(ix);
  return info.supported;
}


bool
Isa::isSupported(RvExtension ext, unsigned version, unsigned subversion) const
{
  unsigned ix = extIx(ext);
  if (ix >= infoVec_.size())
    return false;

  const Info& info = infoVec_.at(ix);
  if (not info.supported)
    return false;

  return std::ranges::any_of(info.versions,
                             [version, subversion](const auto& vp) {
                               return vp.first == version and vp.second == subversion;
                             });
}


bool
Isa::getDefaultVersion(RvExtension ext, unsigned& version, unsigned& subversion) const
{
  unsigned ix = extIx(ext);
  if (ix >= infoVec_.size())
    return false;

  const Info& info = infoVec_.at(ix);
  if (not info.supported)
    return false;

  version = info.dflt.first;
  subversion = info.dflt.second;

  return true;
}


bool
Isa::getVersion(RvExtension ext, unsigned& version) const
{
  unsigned ix = extIx(ext);
  if (ix >= infoVec_.size())
    return false;

  const Info& info = infoVec_.at(ix);
  if (not info.supported)
    return false;

  version = info.selected.first;
  return true;
}


bool
Isa::getVersion(RvExtension ext, unsigned& version, unsigned& subversion) const
{
  unsigned ix = extIx(ext);
  if (ix >= infoVec_.size())
    return false;

  const Info& info = infoVec_.at(ix);
  if (not info.supported)
    return false;

  version = info.selected.first;
  subversion = info.selected.second;
  return true;
}


RvExtension
Isa::stringToExtension(std::string_view str)
{
  const auto iter = stringToExt_.find(str);
  if (iter == stringToExt_.end())
    return RvExtension::None;
  return iter->second;
}


std::string_view
Isa::extensionToString(RvExtension ext)
{
  unsigned ix = extIx(ext);
  return ix < extToString_.size()? extToString_.at(ix) : "";
}


/// Extract a single charecter (not 'z') extension or a multi-character
/// extension starting with a 'z' from the isa string starting at locaton
/// i and update i.  Multi-char extensions are of of the
/// form: z<name><version>p<subversion>
/// where <name> is a sequence of letters, or a sequence of letters
/// followed by a sequence of digits and then a letter other than 'p'.
/// The <version>/<subversion> are sequences of digits. The
/// <version>p<subversion> suffix is optional.
bool
extractExtension(std::string_view isa, size_t& i, std::string_view& extension)
{
  size_t len = isa.size();
  if (i >= len)
    return true;
  if (isa.at(i) == 'z')
    {
      // A sequence of letters following 'z', is part of the extension
      // name.
      size_t init = i;
      for ( ; i < len and isa.at(i) >= 'a' and isa.at(i) <= 'z'; i++)
        ;

      extension = isa.substr(init, i - init);

      // A sequence of digits followed by an letter other than 'p' is also
      // part of the extension name.
      size_t j = i;
      for ( ; j < len and std::isdigit(isa.at(j)); ++j)
        ;

      if (j < len and j > i)
        {
          auto c = isa.at(j);
          if (c >= 'a' and c < 'z' and c != 'p')
            {
              extension = isa.substr(init, j - init);
              i         = j + 1;
            }
        }

      return true;
    }

  if (isa.at(i) >= 'a' and isa.at(i) < 'z')
    {
      extension = isa.substr(i, 1);
      if (i + 1 < len and std::isdigit(isa.at(i + 1)))
        ++i;
      return true;
    }
  return false;
}


// Extract optional version. If next character is a digit, extract
// version and subversion: a sequence of decimal digits followed by a
// 'p' followed by another sequence of decimal digits. Return true on
// success.
bool
extractVersion(std::string_view isa, size_t& i, std::string_view& version,
	       std::string_view& subversion)
{
  size_t len = isa.size();
  if (i >= len)
    return true;

  if (not std::isdigit(isa.at(i)))
    return true;

  size_t j = i;
  for ( ; i < len and  std::isdigit(isa.at(i)); i++)
    ;
  version = isa.substr(j, i - j);

  if (i >= len or isa.at(i) != 'p')
    return false;
  i++;

  j = i;
  for ( ; i < len and  std::isdigit(isa.at(i)); i++)
    ;
  subversion = isa.substr(j, i - j);

  return i > j;
}


bool
Isa::configIsa(std::string_view isa)
{
  if (applyIsaString(isa))
    return true;

  std::cerr << "Invalid ISA: " << isa << '\n';
  return false;
}


bool
Isa::applyIsaString(std::string_view isaStr)
{
  std::string_view isa = isaStr;

  // Check and skip rv prefix.
  if (isa.starts_with("rv32") or isa.starts_with("rv64"))
    isa = isa.substr(4);
  else if (isa.starts_with("rv") and isa.size() > 2 and std::isdigit(isa.at(2)))
    {
      std::cerr << "Unsupported ISA: " << isa << '\n';
      return false;
    }

  bool hasZ = false;

  for (size_t i = 0; i < isa.size(); ++i)
    {
      std::string_view extension, version, subversion;

      char c = isa.at(i);
      if (c == '_')
	{
	  if (i == 0)
	    return false;
	  continue;
	}
      if (c == 'z')
	{
	  // First extension cannot be a z. Z exts must be separated with _.
	  if (i == 0 or ((hasZ and isa.at(i-1) != '_')))
	    return false;  // 1st extension cannot be a z extension.
	  hasZ = true;
	}
      else if (c >= 'a' and c < 'z')
	{
	  if (hasZ)
	    return false; // Cannot have a regular exension after z.
	}
      else
	return false;  // Bad character

      if (not extractExtension(isa, i, extension))
	return false;

      assert(not extension.empty());
      if (not extractVersion(isa, i, version, subversion))
	return false;

      RvExtension ext = stringToExtension(extension);
      if (ext == RvExtension::None)
	{
	  std::cerr << "Unknown extension: " << extension
		    << " -- ignored\n";
	  continue;
	}

      enable(ext, true);

      if (version.empty())
	continue;

      unsigned v = 0;
      std::from_chars(version.begin(), version.end(), v);

      unsigned s = 0;
      if (not subversion.empty())
        std::from_chars(subversion.begin(), subversion.end(), s);
      if (not selectVersion(ext, v, s))
	{
	  getDefaultVersion(ext, v, s);
	  selectVersion(ext, v, s);
	  std::cerr << "Version " << version << "." << subversion
		    << " of extension " << extension << " is not "
		    << "supported -- using default\n";
	}
    }

  return true;
}
