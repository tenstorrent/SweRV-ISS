#pragma once

#include <array>
#include <string_view>
#include <vector>
#include <unordered_map>

namespace WdRiscv
{

  enum class RvExtension : unsigned { A, B, C, D, E, F, H, I, M, N, S, U, V,
                                      Zba, Zbb, Zbc, Zbe, Zbf, Zbm, Zbp,
                                      Zbr, Zbs, Zbt, Zfh, Zfhmin, Zlsseg,
                                      Zknd, Zkne, Zknh, Zbkb, Zbkx, Zksed, Zksh,
                                      Svinval, Zicbom, Zicboz, Zicbop, Zawrs, Zmmul,
                                      Zvfh, Zvfhmin, Zvbb, Zvbc, Zvkg,
				      Zvkned, Zvknha, Zvknhb, Zvksed,
                                      Zvksh, Zicond, Zcb, Zfa, Zfbfmin,
                                      Zvfbfmin, Zvfbfwma, Sstc, Svpbmt,
				      Smaia, Ssaia, Zacas, Zimop, Zcmop, Smrnmi, None };

  /// Model supported extensions with primary/secondary version numbers.
  class Isa
  {
  public:

    Isa();

    ~Isa() = default;

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
    static RvExtension stringToExtension(std::string_view str);

    /// Return string correponding to given extension enum. Return empty
    /// string if given extension is out of bounds.
    static std::string_view extensionToString(RvExtension ext);

    /// Process extension string enabling etxesions and selecting
    /// versions. Return true on success. Return false if extension
    /// string is not valid or if an extension or extension version is
    /// not supported. Sample ISA string: rv32i2p0_m2p0
    bool configIsa(std::string_view isa);

  protected:

    /// Helper to configIsa.
    bool applyIsaString(std::string_view isa);

    // Exists to be used below in the std::array size
    // declarations
    template <RvExtension ext>
    using ext_ix = std::integral_constant<unsigned, static_cast<unsigned>(ext)>; // Use std::to_underlying

    /// Return integer value underlying extension enum.
    static constexpr unsigned extIx(RvExtension ext)
    { return static_cast<unsigned>(ext); } // Use std::to_underlying

  private:

    using VersionPair = std::pair<unsigned, unsigned>;

    struct Info
    {
      Info() = default;

      Info(std::vector<VersionPair>&& versions, VersionPair dflt)
	: supported(not versions.empty()), dflt(dflt),
	  selected(dflt), versions(std::move(versions))
      { }

      bool supported = false;
      bool enabled = false;
      VersionPair dflt;  // Default;
      VersionPair selected;  // Currently selected.
      std::vector<VersionPair> versions;
    };

    std::array<Info, ext_ix<RvExtension::None>::value> infoVec_;

    static const std::unordered_map<std::string_view, RvExtension> stringToExt_;
    static const std::array<std::string_view, ext_ix<RvExtension::None>::value> extToString_;
  };
}
