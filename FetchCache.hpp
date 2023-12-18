#include <cassert>
#include <unordered_map>

namespace TT_FETCH_CACHE
{

  /// Model an instruction cache.
  class FetchCache
  {
  public:

    FetchCache(unsigned lineSize = 64)
      : lineSize_(lineSize)
    {
      assert(lineSize > 0 and (lineSize % 8) == 0);
      lineShift_ = std::log2(lineSize);
    }

    /// Add a line to the cache. Data is obtained by calling readMem
    /// for the words of the line. Return true on success. Return false
    /// if any of the memory reads fails.
    bool addLine(uint64_t addr, std::function<bool(uint64_t, uint32_t&)> readMem)
    {
      uint64_t lineNum = addr >> lineShift_;
      auto& vec = data_[lineNum];
      vec.resize(lineSize_);
      bool ok = true;
      unsigned words = lineSize_ / 4;
      addr = lineNum << lineShift_;
      for (unsigned i = 0; i < words; ++i, addr += 4)
	{
	  uint32_t val = 0;
	  ok = readMem(addr, val) and ok;
	  unsigned j = i * 4;
	  vec.at(j) = val;
	  vec.at(j + 1) = val >> 8;
	  vec.at(j + 2) = val >> 16;
	  vec.at(j + 3) = val >> 24;
	}
      return ok;
    }

    /// Remove from this cashe the line contining the given address.
    /// No-op if line is not in cache.
    void removeLine(uint64_t addr)
    {
      uint64_t lineNum = addr >> lineShift_;
      data_.erase(lineNum);
    }

    /// Read into inst the 2-byte instruction at the given
    /// address. Return true on success. Return false if addr is not
    /// aligned (even) or if the corresponding line is not in the
    /// cache.
    bool read(uint64_t addr, uint16_t& inst) const
    {
      if (addr & 1)
	return false; // Addr must be even.
      uint64_t lineNum = addr >> lineShift_;
      auto iter = data_.find(lineNum);
      if (iter == data_.end())
	return false;
      auto& vec = iter->second;
      unsigned byteIx = addr % lineSize_;
      inst = vec.at(byteIx) | (uint16_t(vec.at(byteIx + 1)) << 8);
      return true;
    }

  private:

    unsigned lineSize_ = 64;
    unsigned lineShift_ = 6;
    std::unordered_map< uint64_t, std::vector<uint8_t> > data_;
  };

}
    
    
