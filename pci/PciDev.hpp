#pragma once

#include <linux/pci_regs.h>
#include <tuple>
#include <vector>
#include <cstdint>
#include <cassert>
#include <memory>
#include <mutex>
#include <functional>
#include <iostream>
#include <string.h>

// under PCIe, 4096 bytes of configuration space
#define PCI_CFG_SIZE 4096

class PciDev {

  public:

    union config {

      struct fields {
        uint16_t vendor_id;
        uint16_t device_id;
        uint16_t command;
        uint16_t status;
        uint8_t revision_id;
        uint8_t class_code[3];
        uint8_t cache_line_size;
        uint8_t latency_timer;
        uint8_t header_type;
        uint8_t bist;
        uint32_t bar[6];
        uint32_t card_bus;
        uint16_t subsys_vendor_id;
        uint16_t subsys_id;
        uint32_t exp_rom_bar;
        uint8_t cap;
        uint8_t reserved[7];
        uint8_t interrupt_line; // not needed?
        uint8_t interrupt_pin;  // not needed?
        uint8_t min_gnt;
        uint8_t max_lat;
      } __attribute__ ((packed));

      fields bits;
      uint8_t data[PCI_CFG_SIZE] = {0};
    };

    PciDev()
    {
      bars_.resize(6);
      bar_eols_.resize(6, nullptr);
      bar_sizes_.resize(6, 0);

      header_eol_ = header_.data + sizeof(config::fields);
    };

    virtual ~PciDev() {};

    // Setup function after BARs are allocated
    virtual bool setup() = 0;

    /// Helper function to set extra structures to the header memory region (e.g. capability structures). Sets the offset from base address in bytes.
    template <typename T>
    uint8_t* ask_header_blocks(size_t size, uint32_t& offset)
    {
      uintptr_t align = sizeof(T);
      align = (1 << (align - 1));

      uint8_t* tmp = reinterpret_cast<uint8_t*>(uintptr_t(header_eol_ + align - 1) & ~(align - 1));
      if ((tmp + size) > &(header_.data[PCI_CFG_SIZE - 1]))
        return nullptr;

      header_eol_ = tmp + size;
      offset = tmp - header_.data;
      assert(offset <= 0xff);
      return tmp;
    }

    /// Helper function to set extra structures to BARs. Sets the offset from base address in bytes.
    template <typename T>
    uint8_t* ask_bar_blocks(unsigned bar, size_t size, uint32_t& offset)
    {
      assert(bar < 6 and "There are only 6 bars");
      auto bar_eol = uintptr_t(bar_eols_.at(bar));

      uintptr_t align = sizeof(T);
      align = (1 << (align - 1));

      uintptr_t tmp = (bar_eol + align - 1) & ~(align - 1);
      if ((tmp + size) > (bar_eol + bar_sizes_.at(bar) - 1))
        return nullptr;

      auto casted = reinterpret_cast<uint8_t*>(tmp);
      bar_eols_.at(bar) = casted + size;
      offset = casted - bars_.at(bar)->bytes;
      return casted;
    }

    union config& header()
    { return header_; }

    auto& bars()
    { return bars_; }

    auto& bar_eols()
    { return bar_eols_; }

    auto& memmap()
    { return memmap_; }

    auto& msi()
    { return msi_; }

    void set_bar_size(unsigned bar, unsigned size)
    {
      assert(bar < 6 and "There are only 6 bars");
      bar_sizes_.at(bar) = size;
    }

    unsigned bar_size(unsigned bar) const
    {
      assert(bar < 6 and "There are only 6 bars");
      return bar_sizes_.at(bar);
    }

    bool bar_type(unsigned bar, bool& io) const
    {
      assert(bar < 6 and "There are only 6 bars");
      if (not bar_size(bar))
        return false;

      io = (header_.bits.bar[bar] & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_IO;
      return true;
    }

    mutable std::mutex m_;

  protected:

    friend class Pci;

    struct mmio_blocks {

      mmio_blocks(uint32_t base, size_t size,
                  const std::function<uint8_t*(uint64_t, size_t)>& memmap)
        : base(base), size(size)
      {
        auto ptr = memmap(base, size);
        bytes = ptr;
        memset(bytes, 0, size);
        if (not ptr)
          std::cerr << "Failed to map to addr: " << base << " with size: " << size << std::endl;
      };

      uint32_t base;
      size_t size;
      uint8_t* bytes = nullptr;
      // write first, then call callback (if it exists)
      std::function<void(uint32_t offset, size_t len)> write_cb = nullptr;
      // test if read callback, then read if DNE
      std::function<bool(uint32_t offset, uint64_t& data)> read_cb = nullptr;
    };

    template <typename T>
    void write_config(uint8_t offset, T data);

    template <typename T>
    void read_config(uint8_t offset, T& data) const;

    std::tuple<bool, unsigned> active_addr(uint64_t addr) const;

    bool active_bar(unsigned bar) const;

  private:

    union config header_;
    uint8_t* header_eol_;

    std::vector<std::shared_ptr<mmio_blocks>> bars_;
    std::vector<uint8_t*> bar_eols_;
    std::vector<unsigned> bar_sizes_;

    // for r/w to host memory
    std::function<uint8_t*(uint64_t, size_t)> memmap_;
    std::function<void(uint64_t, unsigned, uint64_t)> msi_;
};
