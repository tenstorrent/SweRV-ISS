#include <boost/algorithm/string.hpp>
#include <iostream>
#include "Isa.hpp"


using namespace WdRiscv;

Isa::Isa()
{
  infoVec_.resize(extIx(RvExtension::None) + 1);

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
  infoVec_.at(extIx(RvExtension::Zksed)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zksh)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Svinval)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zicbom)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zicboz)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(RvExtension::Zawrs)) = Info{ {{1,0}}, {1,0} };

  infoVec_.at(extIx(RvExtension::I)).enabled = true; // I always enabled.

  stringToExt_["a"] = RvExtension::A;
  stringToExt_["b"] = RvExtension::B;
  stringToExt_["c"] = RvExtension::C;
  stringToExt_["d"] = RvExtension::D;
  stringToExt_["e"] = RvExtension::E;
  stringToExt_["f"] = RvExtension::F;
  stringToExt_["h"] = RvExtension::H;
  stringToExt_["i"] = RvExtension::I;
  stringToExt_["m"] = RvExtension::M;
  stringToExt_["s"] = RvExtension::S;
  stringToExt_["u"] = RvExtension::U;
  stringToExt_["v"] = RvExtension::V;
  stringToExt_["zba"] = RvExtension::Zba;
  stringToExt_["zbb"] = RvExtension::Zbb;
  stringToExt_["zbc"] = RvExtension::Zbc;
  stringToExt_["zbs"] = RvExtension::Zbs;
  stringToExt_["zfh"] = RvExtension::Zfh;
  stringToExt_["zfhmin"] = RvExtension::Zfhmin;
  stringToExt_["zlssegh"] = RvExtension::Zlsseg;
  stringToExt_["zknd"] = RvExtension::Zknd;
  stringToExt_["zkne"] = RvExtension::Zkne;
  stringToExt_["zknh"] = RvExtension::Zknh;
  stringToExt_["zbkb"] = RvExtension::Zbkb;
  stringToExt_["zksed"] = RvExtension::Zksed;
  stringToExt_["zksh"] = RvExtension::Zksh;
  stringToExt_["svinval"] = RvExtension::Svinval;
  stringToExt_["zicbom"] = RvExtension::Zicbom;
  stringToExt_["zicboz"] = RvExtension::Zicboz;
  stringToExt_["zawrs"] = RvExtension::Zawrs;

  extToString_.resize(extIx(RvExtension::None));
  extToString_.at(extIx(RvExtension::A)) = "a";
  extToString_.at(extIx(RvExtension::B)) = "b";
  extToString_.at(extIx(RvExtension::C)) = "c";
  extToString_.at(extIx(RvExtension::D)) = "d";
  extToString_.at(extIx(RvExtension::E)) = "e";
  extToString_.at(extIx(RvExtension::F)) = "f";
  extToString_.at(extIx(RvExtension::H)) = "h";
  extToString_.at(extIx(RvExtension::I)) = "i";
  extToString_.at(extIx(RvExtension::M)) = "m";
  extToString_.at(extIx(RvExtension::V)) = "v";
  extToString_.at(extIx(RvExtension::Zba)) = "zba";
  extToString_.at(extIx(RvExtension::Zbb)) = "zbb";
  extToString_.at(extIx(RvExtension::Zbc)) = "zbc";
  extToString_.at(extIx(RvExtension::Zbs)) = "zbs";
  extToString_.at(extIx(RvExtension::Zfh)) = "zfh";
  extToString_.at(extIx(RvExtension::Zfhmin)) = "zfhmin";
  extToString_.at(extIx(RvExtension::Zlsseg)) = "zlssegh";
  extToString_.at(extIx(RvExtension::Zknd)) = "zknd";
  extToString_.at(extIx(RvExtension::Zkne)) = "zkne";
  extToString_.at(extIx(RvExtension::Zknh)) = "zknh";
  extToString_.at(extIx(RvExtension::Zbkb)) = "zbkb";
  extToString_.at(extIx(RvExtension::Zksed)) = "zksed";
  extToString_.at(extIx(RvExtension::Zksh)) = "zksh";
  extToString_.at(extIx(RvExtension::Svinval)) = "svinval";
  extToString_.at(extIx(RvExtension::Zicbom)) = "zicbom";
  extToString_.at(extIx(RvExtension::Zicboz)) = "zicboc";
  extToString_.at(extIx(RvExtension::Zawrs)) = "zawrs";
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

  for (const auto& vp : info.versions)
    if (vp.first == version and vp.second == subversion)
      return true;

  return false;
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
Isa::stringToExtension(const std::string& str) const
{
  const auto iter = stringToExt_.find(str);
  if (iter == stringToExt_.end())
    return RvExtension::None;
  return iter->second;
}


std::string
Isa::extensionToString(RvExtension ext) const
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
extractExtension(const std::string& isa, size_t& i, std::string& extension)
{
  size_t len = isa.size();
  if (i >= len)
    return true;
  if (isa.at(i) == 'z')
    {
      // A sequence of letters following 'z', is part of the extension
      // name.
      while (i < len and isa.at(i) >= 'a' and isa.at(i) <= 'z')
	extension.push_back(isa.at(i++));

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
	      extension += isa.substr(i, j - i + 1);
	      i = j + 1;
	    }
	}

      return true;
    }
  if (isa.at(i) >= 'a' and isa.at(i) < 'z')
    {
      extension.push_back(isa.at(i));
      if (i+1 < len and std::isdigit(isa.at(i+1)))
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
extractVersion(const std::string& isa, size_t& i, std::string& version,
	       std::string& subversion)
{
  size_t len = isa.size();
  if (i >= len)
    return true;

  if (not std::isdigit(isa.at(i)))
    return true;

  while (i < len and  std::isdigit(isa.at(i)))
    version.push_back(isa.at(i++));

  if (i >= len or isa.at(i) != 'p')
    return false;
  i++;
  
  size_t j = i;
  while (i < len and  std::isdigit(isa.at(i)))
    subversion.push_back(isa.at(i++));
  return i > j;
}


bool
Isa::configIsa(const std::string& isa)
{
  if (applyIsaString(isa))
    return true;

  std::cerr << "Invalid ISA: " << isa << '\n';
  return false;
}
  
    
bool
Isa::applyIsaString(const std::string& isaStr)
{
  std::string isa = isaStr;

  // Check and skip rv prefix.
  if (boost::starts_with(isa, "rv32") or boost::starts_with(isa, "rv64"))
    isa = isa.substr(4);
  else if (boost::starts_with(isa, "rv") and isa.size() > 2 and std::isdigit(isa.at(2)))
    {
      std::cerr << "Unsupported ISA: " << isa << '\n';
      return false;
    }

  bool hasZ = false;
  
  for (size_t i = 0; i < isa.size(); ++i)
    {
      std::string extension, version, subversion;

      char c = isa.at(i);
      if (c == '_')
	{
	  if (i == 0)
	    return false;
	  continue;
	}
      else if (c == 'z')
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

      unsigned v = atoi(version.c_str());
      unsigned s = 0;
      if (not subversion.empty())
	s = atoi(subversion.c_str());
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
