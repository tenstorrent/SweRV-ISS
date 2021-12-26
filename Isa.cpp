#include <boost/algorithm/string.hpp>
#include <iostream>
#include "Isa.hpp"


using namespace WdRiscv;

Isa::Isa()
{
  infoVec_.resize(extIx(Extension::None) + 1);

  infoVec_.at(extIx(Extension::A)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(Extension::B)) = Info{ {{0,93}}, {0,93} };
  infoVec_.at(extIx(Extension::C)) = Info{ {{1,0}, {2,0}}, {1,0} };
  infoVec_.at(extIx(Extension::D)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(Extension::E)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(Extension::F)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(Extension::I)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(Extension::M)) = Info{ {{2,0}}, {2,0} };
  infoVec_.at(extIx(Extension::V)) = Info{ {{1,0}}, {1,0} };
  infoVec_.at(extIx(Extension::Zba)) = Info{ {{0,0}}, {0,0} };
  infoVec_.at(extIx(Extension::Zbb)) = Info{ {{0,0}}, {0,0} };
  infoVec_.at(extIx(Extension::Zbc)) = Info{ {{0,0}}, {0,0} };
  infoVec_.at(extIx(Extension::Zbs)) = Info{ {{0,0}}, {0,0} };
  infoVec_.at(extIx(Extension::Zfh)) = Info{ {{0,0}}, {0,0} };
  infoVec_.at(extIx(Extension::Zlsseg)) = Info{ {{1,0}}, {1,0} };

  infoVec_.at(extIx(Extension::I)).enabled = true; // I always enabled.

  stringToExt_["a"] = Extension::A;
  stringToExt_["b"] = Extension::B;
  stringToExt_["c"] = Extension::C;
  stringToExt_["d"] = Extension::D;
  stringToExt_["e"] = Extension::E;
  stringToExt_["f"] = Extension::F;
  stringToExt_["i"] = Extension::I;
  stringToExt_["m"] = Extension::M;
  stringToExt_["v"] = Extension::V;
  stringToExt_["zba"] = Extension::Zba;
  stringToExt_["zbb"] = Extension::Zbb;
  stringToExt_["zbc"] = Extension::Zbc;
  stringToExt_["zbs"] = Extension::Zbs;
  stringToExt_["zfh"] = Extension::Zfh;
  stringToExt_["zlssegh"] = Extension::Zlsseg;

  extToString_.resize(extIx(Extension::None));
  extToString_.at(extIx(Extension::A)) = "a";
  extToString_.at(extIx(Extension::B)) = "b";
  extToString_.at(extIx(Extension::C)) = "c";
  extToString_.at(extIx(Extension::D)) = "d";
  extToString_.at(extIx(Extension::E)) = "e";
  extToString_.at(extIx(Extension::F)) = "f";
  extToString_.at(extIx(Extension::I)) = "i";
  extToString_.at(extIx(Extension::M)) = "m";
  extToString_.at(extIx(Extension::V)) = "v";
  extToString_.at(extIx(Extension::Zba)) = "zba";
  extToString_.at(extIx(Extension::Zbb)) = "zbb";
  extToString_.at(extIx(Extension::Zbc)) = "zbc";
  extToString_.at(extIx(Extension::Zbs)) = "zbs";
  extToString_.at(extIx(Extension::Zfh)) = "zfh";
  extToString_.at(extIx(Extension::Zlsseg)) = "zlssegh";
}


bool
Isa::selectVersion(Extension ext, unsigned version, unsigned subversion)
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
Isa::isSupported(Extension ext) const
{
  unsigned ix = extIx(ext);
  if (ix >= infoVec_.size())
    return false;

  const Info& info = infoVec_.at(ix);
  return info.supported;
}


bool
Isa::isSupported(Extension ext, unsigned version, unsigned subversion) const
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
Isa::getDefaultVersion(Extension ext, unsigned& version, unsigned& subversion) const
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
Isa::getVersion(Extension ext, unsigned& version) const
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
Isa::getVersion(Extension ext, unsigned& version, unsigned& subversion) const
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


Isa::Extension
Isa::stringToExtension(const std::string& str) const
{
  const auto iter = stringToExt_.find(str);
  if (iter == stringToExt_.end())
    return Extension::None;
  return iter->second;
}


std::string
Isa::extensionToString(Extension ext) const
{
  unsigned ix = extIx(ext);
  return ix < extToString_.size()? extToString_.at(ix) : "";
}


bool
extractExtension(const std::string& isa, size_t& i, std::string& extension)
{
  size_t len = isa.size();
  if (i >= len)
    return true;
  if (isa.at(i) == 'z')
    {
      while (i < len and isa.at(i) >= 'a' and isa.at(i) <= 'z')
	extension.push_back(isa.at(i++));
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


// Extract optional versio. If next character i a digit, extract
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

      Extension ext = stringToExtension(extension);
      if (ext == Extension::None)
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
