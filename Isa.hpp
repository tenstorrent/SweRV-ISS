#pragma once

#include <vector>
#include <unordered_map>

namespace WdRiscv
{

  enum class RvExtension : unsigned { A, B, C, D, E, F, H, I, M, S, U, V,
				      Zba, Zbb, Zbc, Zbe, Zbf, Zbm, Zbp,
				      Zbr, Zbs, Zbt, Zfh, Zfhmin, Zlsseg,
				      Zknd, Zkne, Zknh, Zbkb, Zksed, Zksh,
				      Svinval, Zicbom, Zicboz, Zawrs, None };

  /// Model supported extensions with primary/secondary version numbers.
  class Isa
  {
  public:

    Isa();

    ~Isa()
    { }

    /// Select given given version of extension. Return true if
    /// successful. Return false if given extension or associated
    /// version/subversion is not supported. If successful, subsequent
    /// calls to getVersion will return the newly slected version.
    bool selectVersion(RvExtension ext, unsigned version, unsigned subversion);

    /// Return true if given extension is supported.
    bool isSupported(RvExtension ext) const;

    /// Return true if given version of given extension is supported.
    bool isSupported(RvExtension ext, unsigned version, unsigned subversion) const;

    /// Return true if given extension is supported setting verion and
    /// subversion to the corresponding default version. Return false
    /// leaving version/subversion unmodified if given extension is
    /// not supported.
    bool getDefaultVersion(RvExtension ext, unsigned& version,
			   unsigned& subversion) const;

    /// Return true if given extension is supported setting version
    /// to the currently selected primary version number.
    bool getVersion(RvExtension ext, unsigned& version) const;

    /// Return true if given extension is supported setting version/subversino
    /// to the currently selected primary/secondary version numbers.
    bool getVersion(RvExtension ext, unsigned& version, unsigned& subversion) const;

    /// Return true if geven extension is enabled.
    bool isEnabled(RvExtension ext) const
    {
      unsigned ix = extIx(ext);
      return ix < infoVec_.size()? infoVec_.at(ix).enabled : false;
    }

    /// Enable/disable given extension if flag is true/false.
    void enable(RvExtension ext, bool flag)
    {
      unsigned ix = extIx(ext);
      if (ix < infoVec_.size()) infoVec_.at(ix).enabled = flag;
    }

    /// Return extension corresponding to given string. For example,
    /// return RvExtension::A for "a". Return RvExtension::None if no such
    /// extension.
    RvExtension stringToExtension(const std::string& str) const;

    /// Return string correponding to given extension enum. Return empty
    /// string if given extension is out of bounds.
    std::string extensionToString(RvExtension ext) const;

    /// Process extension string enabling etxesions and selecting
    /// versions. Return true on success. Return false if extension
    /// string is not valid or if an extension or extension version is
    /// not supported. Sample ISA string: rv32i2p0_m2p0
    bool configIsa(const std::string& isa);

  protected:

    /// Helper to configIsa.
    bool applyIsaString(const std::string& isa);

    /// Return integer value underlying extension enum.
    unsigned extIx(RvExtension ext) const
    { return static_cast<unsigned>(ext); } // Use std::to_underlying

  private:

    typedef std::pair<unsigned, unsigned> VersionPair;

    struct Info
    {
      Info()
	: supported(false), enabled(false)
      { }

      Info(const std::vector<VersionPair>& versions, VersionPair dflt)
	: supported(not versions.empty()), enabled(false), dflt(dflt),
	  selected(dflt), versions(versions)
      { }

      bool supported = false;
      bool enabled = false;
      VersionPair dflt;  // Default;
      VersionPair selected;  // Currently selected.
      std::vector<VersionPair> versions;
    };

    std::vector<Info> infoVec_;

    std::unordered_map<std::string, RvExtension> stringToExt_;
    std::vector<std::string> extToString_;
  };
}
