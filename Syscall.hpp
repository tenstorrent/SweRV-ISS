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

#pragma once

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <vector>
#include <string>
#include "util.hpp"


namespace WdRiscv
{
  template <typename URV>
  class Hart;


  template <typename URV>
  class Syscall
  {
  public:

    /// Signed register type corresponding to URV. For example, if URV
    /// is uint32_t, then SRV will be int32_t.
    using SRV = typename std::make_signed_t<URV>;

    Syscall(std::vector<std::shared_ptr<Hart<URV>>>& harts, size_t memSize)
      : harts_(harts)
    {
    	auto mem_size = memSize;
    	mmap_blocks_.insert(std::make_pair(mem_size/2L, blk_t(mem_size/2L, true)));
    }

    /// Emulate a system call on the associated hart. Return an integer value
    /// corresponding to the result. The call number is in syscallIx. The call paramters
    /// are in a0, a1, a2, and a3.
    URV emulate(unsigned hartIx, unsigned syscallIx, URV a0, URV a1, URV a2, URV a3);

    URV emulateSemihost(unsigned hartIx, URV a0, URV a1);

    /// Redirect the given output file descriptor (typically that of
    /// stdout or stderr) to the given file. Return true on success
    /// and false on failure.
    bool redirectOutputDescriptor(int fd, const std::string& path);

    /// Redirect the given input file descriptor (typically that of
    /// stdin) to the given file. Return true on success and false on
    /// failure.
    bool redirectInputDescriptor(int fd, const std::string& path);

    void enableLinux(bool flag)
    { linux_ = flag; }

    /// Save the currently open file descriptors to the given file.
    bool saveFileDescriptors(const std::string& path);

    /// Load and open the file descriptors previously saved in given file.
    bool loadFileDescriptors(const std::string& path);

    /// Report the files opened by the target RISCV program during
    /// current run.
    void reportOpenedFiles(std::ostream& out);

    uint64_t mmap_alloc(uint64_t size);

    int mmap_dealloc(Hart<URV>& hart, uint64_t addr, uint64_t size);

    uint64_t mmap_remap(Hart<URV>& hart, uint64_t addr, uint64_t old_size, uint64_t new_size, bool maymove);

    using AddrLen = std::pair<uint64_t, uint64_t>;  // Address/length pair

    void getUsedMemBlocks(uint64_t sp, std::vector<AddrLen>& used_blocks);

    bool saveMmap(const std::string & filename);

    bool loadMmap(const std::string & filename);

  protected:

    friend class Hart<URV>;

    /// For Linux emulation: Set initial target program break to the
    /// RISCV page address larger than or equal to the given address.
    void setTargetProgramBreak(URV addr)
    { progBreak_ = addr; }

    /// Return target program break.
    URV targetProgramBreak() const
    { return progBreak_; }

    /// Map Linux file descriptor to a RISCV file descriptor and install
    /// the result in the riscv-to-linux fd map. Return remapped
    /// descritpor or -1 if remapping is not possible.
    int registerLinuxFd(int linuxFd, const std::string& path, bool isRead);

    /// Return the effective (after redirection) file descriptor
    /// corresponding to the target program file descriptor.
    int effectiveFd(int fd)
    {
      auto it = fdMap_.find(fd);
      if (it != fdMap_.end())
        return it->second;
      return fd;
    }

#if 0
    Hart<URV>* nextAvailHart()
    {
      for (auto hptr : harts_)
        if (hptr->isSuspended())
          return hptr.get();
      return nullptr;
    }
#endif

  private:

    std::vector<std::shared_ptr<Hart<URV>>>& harts_;
    bool linux_ = false;
    URV progBreak_ = 0;          // For brk Linux emulation.

    struct blk_t {
    	blk_t(uint64_t length, bool free): length(length), free(free) {};
    	uint64_t length;
    	bool free;
    };
    using blk_map_t = std::map<uint64_t, blk_t>;
    blk_map_t mmap_blocks_;

    std::unordered_map<int, int> fdMap_;
    std::unordered_map<int, bool> fdIsRead_;
    std::unordered_map<int, std::string> fdPath_;
    std::unordered_set<std::string, util::string_hash, std::equal_to<>> readPaths_;
    std::unordered_set<std::string, util::string_hash, std::equal_to<>> writePaths_;

    std::unordered_map<uint64_t, std::unordered_set<unsigned>> futexMap_;

    std::mutex emulateMutex_;
    std::mutex semihostMutex_;
  };
}
