#include <algorithm>
#include <cassert>
#include <charconv>
#include <iostream>
#include <boost/algorithm/string.hpp>
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
  { "zbs", RvExtension::Zbs },
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
  { "zkr", RvExtension::Zkr },
  { "svinval", RvExtension::Svinval },
  { "svnapot", RvExtension::Svnapot },
  { "zicbom", RvExtension::Zicbom },
  { "zicboz", RvExtension::Zicboz },
  { "zicbop", RvExtension::Zicbop },
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
  { "zvkb", RvExtension::Zvkb },
  { "zicond", RvExtension::Zicond },
  { "zca", RvExtension::Zca },
  { "zcb", RvExtension::Zcb },
  { "zcb", RvExtension::Zcd },
  { "zfa", RvExtension::Zfa },
  { "zfbfmin", RvExtension::Zfbfmin },
  { "zvfbfmin", RvExtension::Zvfbfmin },
  { "zvfbfwma", RvExtension::Zvfbfwma },
  { "zvqdot", RvExtension::Zvqdot },
  { "sstc", RvExtension::Sstc },
  { "svpbmt", RvExtension::Svpbmt },
  { "svadu", RvExtension::Svadu },
  { "svade", RvExtension::Svade },
  { "smaia", RvExtension::Smaia },
  { "ssaia", RvExtension::Ssaia },
  { "zacas", RvExtension::Zacas },
  { "zimop", RvExtension::Zimop },
  { "zcmop", RvExtension::Zcmop },
  { "smrnmi", RvExtension::Smrnmi },
  { "zicsr", RvExtension::Zicsr },
  { "zicntr", RvExtension::Zicntr },
  { "zihpm", RvExtension::Zihpm },
  { "zifencei", RvExtension::Zifencei },
  { "zihintpause", RvExtension::Zihintpause },
  { "smmpm", RvExtension::Smmpm },
  { "ssnpm", RvExtension::Ssnpm },
  { "smnpm", RvExtension::Smnpm },
  { "sscofpmf", RvExtension::Sscofpmf },
  { "smstateen", RvExtension::Smstateen },
  { "ssqosid", RvExtension::Ssqosid },
  { "sdtrig", RvExtension::Sdtrig },
  { "zicfilp", RvExtension::Zicfilp },
  { "zic64b", RvExtension::Zic64b },
  { "ziccamoa", RvExtension::Ziccamoa },
  { "ziccif", RvExtension::Ziccif },
  { "zicclsm", RvExtension::Zicclsm },
  { "ziccrse", RvExtension::Ziccrse },
  { "za64rs", RvExtension::Za64rs },
  { "zaamo", RvExtension::Zaamo },
  { "zalrsc", RvExtension::Zalrsc },
  { "zihintntl", RvExtension::Zihintntl },
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
  infoVec_.at(extIx(RvExtension::A)) = Info{ {{2,0}, {2,1}}, {2,1} };
  infoVec_.at(extIx(RvExtension::B)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::C)) = Info{ {{1,0}, {2,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::D)) = Info{ {{2,2}}, {2,2} };
  infoVec_.at(extIx(RvExtension::E)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(RvExtension::F)) = Info{ {{2,2}}, {2,2} };
  infoVec_.at(extIx(RvExtension::H)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::I)) = Info{ {{2,0}, {2,1}}, {2,1} };
  infoVec_.at(extIx(RvExtension::M)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(RvExtension::S)) = Info{ {{1,2}}, {1,2} };
  infoVec_.at(extIx(RvExtension::U)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::V)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zba)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zbb)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zbc)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zbs)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zfh)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zca)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zcb)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zcd)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zfa)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zfhmin)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zlsseg)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zknd)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zkne)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zknh)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zbkb)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zbkx)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zksed)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zksh)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zkr)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Svinval)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Svnapot)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zicbom)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zicboz)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zicbop)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zawrs)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zmmul)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zvfh)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zvkb)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zvkg)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zicond)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zvfhmin)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zfbfmin)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zvfbfmin)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zvfbfwma)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zvbb)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zvbc)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zvqdot)) = Info{ {{0,1}}, {0,1} };
  infoVec_.at(extIx(RvExtension::Sstc)) = Info{ {{0,5}}, {0,5} };
  infoVec_.at(extIx(RvExtension::Svpbmt)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Svadu)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Svade)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Smaia)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Ssaia)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zacas)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zimop)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zcmop)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Smrnmi)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zicsr)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(RvExtension::Zicntr)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(RvExtension::Zihpm)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(RvExtension::Zifencei)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(RvExtension::Zihintpause)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(RvExtension::Smmpm)) = Info { {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Ssnpm)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Smnpm)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Sscofpmf)) = Info{ {{0,5}}, {0,5} };
  infoVec_.at(extIx(RvExtension::Ssqosid)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Sdtrig)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zicfilp)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zic64b)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Ziccamoa)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Ziccif)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zicclsm)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Ziccrse)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Za64rs)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zaamo)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zalrsc)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zihintntl)) = Info{ {{1,0}}, {1,0} };

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


/// Parse a string of the form <ext>[<n>p<m>] into <ext>, <n>, and <m>.
/// Return true on success and false on failure.
/// Example strings: a, m1p0
bool
parseIsa(const std::string_view& token, std::string_view& extension,
	 std::string_view& version, std::string_view& subversion)
{
  if (token.empty() or std::isdigit(token.at(0)))
    {
      std::cerr << "Invalid ISA extension: " << token << '\n';
      return false;
    }

  size_t len = token.size();

  // Look for first non-digit from the end of the token.
  unsigned ix = len;
  while (ix > 0 and std::isdigit(token.at(ix - 1)))
    ix--;

  if (ix == len or ix == 0)
    {
      extension = token;   // No digits at end of token.
      return true;
    }

  if (token.at(ix - 1) != 'p')    // Trailing digits but no 'p'
    {
      std::cerr << "Invalid ISA extension: " << token << '\n';
      return false;
    }

  size_t pIx = ix - 1;

  // Skip all the digits before 'p'
  ix = pIx;
  while (ix > 0 and std::isdigit(token.at(ix - 1)))
    ix--;

  if (ix == 0 or ix == pIx)      // Token is:  <digits>p<digits>  or <string>p<digits>
    {
      std::cerr << "Invalid ISA extension: " << token << '\n';
      return false;
    }

  extension = token.substr(0, ix);
  version = token.substr(ix, pIx - ix);
  subversion = token.substr(pIx + 1);

  return true;
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
  return applyIsaString(isa);
}


static bool
isLongExtension(std::string_view str)
{
  if (str.size() < 2)
    return false;
  if (str.at(0) == 'z')
    return true;
  if (str.at(0) == 's' and not std::isdigit(str.at(1)))
    return true;
  return false;
}


static
void splitFirstIsaToken(const std::string& tok, std::vector<std::string>& parts)
{
  parts.clear();
  if (tok.empty())
    return;

  size_t prev = 0;
  for (size_t i = 0; i < tok.size(); ++i)
    {
      char c = tok.at(i);
      if (c == 'z')
	{
	  if (i > 0)
	    parts.push_back(tok.substr(prev, i-prev));
	  parts.push_back(tok.substr(i));
	  return;
	}
      if (std::isalpha(c) and i > prev and (c != 'p' or not std::isdigit(tok.at(i-1))))
	{
	  parts.push_back(tok.substr(prev, i-prev));
	  prev = i;
	}
    }

  parts.push_back(tok.substr(prev));
}


bool
Isa::applyIsaString(std::string_view isaStr)
{
  using RVE = RvExtension;

  std::string_view isa = isaStr;

  // Check and skip rv prefix.
  if (isa.starts_with("rv32") or isa.starts_with("rv64"))
    isa = isa.substr(4);
  else if (isa.starts_with("rv") and isa.size() > 2 and std::isdigit(isa.at(2)))
    {
      std::cerr << "Unsupported ISA: " << isaStr << '\n';
      return false;
    }

  // Split string around '_'.
  std::vector<std::string> tokens;
  boost::split(tokens, isa, boost::is_any_of("_"));
  if (tokens.empty() or (tokens.size() == 1 and tokens[0].empty()))
    {
      std::cerr << "Invalid ISA string: " << isaStr << '\n';
      return false;
    }

  // First token may have a z-extension without a preceeding underscore. Spilt it:
  // split something like a1p0m1p1zbb into: a1p0, m1p1 and zbb
  if (not tokens.empty() and not tokens.at(0).empty())
    {
      std::vector<std::string> parts;
      splitFirstIsaToken(tokens.at(0), parts);
      if (not parts.empty())
	parts.insert(parts.end(), tokens.begin() + 1, tokens.end());
      std::swap(tokens, parts);
    }

  // Once we see a Z (e.g zbb) or S (e.g. sstc) token, then remaining tokens must
  // be Z or S.
  bool hasLong = false;

  for (const auto& token : tokens)
    {
      if (token.empty())
	continue;

      std::string_view extension, version, subversion;

      if (isLongExtension(token))
	hasLong = true;
      else if (hasLong and (not isLongExtension(token)))
	{
	  std::cerr << "Misordered ISA: Z/S multi-char extensions must "
		    << "be at end: " << isaStr << '\n';
	  return false;
	}

      if (not parseIsa(token, extension, version, subversion))
	return false;

      RVE ext = stringToExtension(extension);
      if (ext == RVE::None)
	{
	  std::cerr << "Unknown extension: " << extension << " -- ignored\n";
	  continue;
	}

      enable(ext, true);

      if (ext == RVE::B)
	for (RVE subExt : { RVE::Zba, RVE::Zbb, RVE::Zbs } )
	  enable(subExt, true);

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

  if (isEnabled(RvExtension::S) and not isEnabled(RvExtension::U))
    {
      std::cerr << "Having supervisor mode without user mode is not a legal architectural state."
                << "Therefore, if 's' is included in the ISA string, 'u' must be as well.\n";
      return false;
    }

  return true;
}
