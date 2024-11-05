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
#include <optional>

// under PCIe, 4096 bytes of configuration space
#define PCI_CFG_SIZE 4096

class PciDev {

  public:

    union config
    {
      struct fields
      {
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

    struct mmio_blocks
    {
      mmio_blocks(uint32_t base, size_t size)
        : base_(base), size_(size)
      {
        bytes_.resize(size);
      };

      uint32_t base_;
      size_t size_;
      std::vector<uint8_t> bytes_;

      std::function<void(uint32_t, uint32_t, size_t)> write_dev_ = nullptr;
      std::function<uint64_t(uint32_t, size_t)> read_dev_ = nullptr;
    };

    PciDev()
    {
      bars_.resize(6);
      bar_eols_.resize(6, 0);
      bar_sizes_.resize(6, 0);

      header_eol_ = header_.data + sizeof(config::fields);
    };

    virtual ~PciDev() = default;

    /// Setup function after BARs are allocated
    virtual bool setup() = 0;

    /// Helper function to set extra structures to the header
    /// memory region (e.g. capability structures). Sets the offset from
    /// base address in bytes. Returns a pointer within the header structure.
    template <typename U>
    uint8_t* ask_header_blocks(size_t size, uint32_t& offset)
    {
      uintptr_t align = sizeof(U);
      align = (1 << (align - 1));

      uint8_t* tmp = reinterpret_cast<uint8_t*>(uintptr_t(header_eol_ + align - 1) & ~(align - 1));
      offset = tmp - header_.data;
      if ((tmp + size) > &(header_.data[PCI_CFG_SIZE - 1]))
        return nullptr;
      header_eol_ = tmp + size;
      assert(offset <= 0xff);
      return tmp;
    }

    /// Helper function to set extra structures to BARs. Sets the offset
    /// from base address in bytes. Returns an address within allocated BAR.
    template <typename U>
    uint8_t* ask_bar_blocks(unsigned bar, size_t size, uint32_t& offset)
    {
      assert(bar < 6 and "There are only 6 bars");
      auto bar_eol = uintptr_t(bar_eols_.at(bar));

      uintptr_t align = 1 << (sizeof(U) - 1);
      uintptr_t addr = (bar_eol + align - 1) & ~(align - 1);

      auto casted = reinterpret_cast<uint8_t*>(addr);
      offset = casted - bars_.at(bar)->bytes_.data();
      if ((addr + size) > (bar_eol + bar_sizes_.at(bar) - 1))
        return nullptr;
      bar_eols_.at(bar) = reinterpret_cast<uint8_t*>(addr) + size;
      return reinterpret_cast<uint8_t*>(addr);
    }

    void set_bar_base_address(unsigned bar, uint32_t base)
    {
      assert(bar < 6 and "There are only 6 bars");
      header_.bits.bar[bar] = base | PCI_BASE_ADDRESS_SPACE_MEMORY;
    }

    void set_bar_mmio_blocks(unsigned bar, std::shared_ptr<mmio_blocks> blocks)
    {
      assert(bar < 6 and "There are only 6 bars");
      bars_.at(bar) = blocks;
      bar_eols_.at(bar) = blocks->bytes_.data();
    }

    void set_bar_unused(unsigned bar)
    {
      assert(bar < 6 and "There are only 6 bars");
      header_.bits.bar[bar] = 0;
    }

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

    /// Returns true if BAR size is configured properly. Sets io to true
    /// if BAR is marked as IO space, and false otherwise.
    bool bar_type(unsigned bar, bool& io) const
    {
      assert(bar < 6 and "There are only 6 bars");
      if (not bar_size(bar))
        return false;

      io = (header_.bits.bar[bar] & PCI_BASE_ADDRESS_SPACE) ==
              PCI_BASE_ADDRESS_SPACE_IO;
      return true;
    }

  protected:

    friend class Pci;

    /// Writes data to header based on offset from PCI_BASE_ADDRESS_0.
    template <typename U>
    void write_config(uint8_t offset, U data)
    {
      // We don't allow accesses which cross 4B boundary
      if (((offset & 3) + sizeof(U)) > 4)
        return;

      // We don't guard config writes with mask (other than BARs)
      // probably ok in assuming these will be 4B aligned
      if (offset >= PCI_BASE_ADDRESS_0 and offset <= (PCI_BASE_ADDRESS_5 + 3))
        {
          bool io;
          uint8_t bar = ((offset & ~uint32_t(0x3)) - PCI_BASE_ADDRESS_0) >> 2;

          if (not bar_type(bar, io))
            return;

          uint64_t bar_size_mask = ~uint64_t(bar_sizes_.at(bar) - 1);
          uint64_t bar_mask = io? PCI_BASE_ADDRESS_IO_MASK : PCI_BASE_ADDRESS_MEM_MASK;

          // probably ok in assuming these will be 4B aligned
          uint32_t original = header_.bits.bar[bar];
          uint32_t value = original & ~bar_mask;

          if (data == 0xffffffff)
            value |= bar_size_mask;
          else
            value |= (data & bar_mask);
          header_.bits.bar[bar] = value;
          return;
        }
      else if (offset == PCI_ROM_ADDRESS)
        return;

      void* p = &header_.data[offset];
      memcpy(p, &data, sizeof(data));
    }

    /// Reads data from header based on offset from PCI_BASE_ADDRESS_0.
    template <typename U>
    void read_config(uint8_t offset, U& data) const
    {
      // we don't allow accesses which cross 4B boundary
      if (((offset & 3) + sizeof(U)) > 4)
        return;

      const void* p = &header_.data[offset];
      memcpy(&data, p, sizeof(data));
    }

    /// Similar to active_bar, but also detects which BAR an address
    /// belongs to.
    std::optional<unsigned> active_addr(uint64_t addr) const;

    /// Returns true if the selected BAR is active.
    bool active_bar(unsigned bar) const;

    /// Reads a value from host-side memory.
    template <typename U>
    void read_mem(uint64_t addr, U& data) const
    {
      uint64_t tmp;
      read_mem_(addr, sizeof(U), tmp);
      data = U(tmp);
    }

    /// Writes a value to host-side memory.
    template <typename U>
    void write_mem(uint64_t addr, U data) const
    {
      uint64_t tmp = data;
      write_mem_(addr, sizeof(U), tmp);
    }

    /// Trigger MSI to host.
    void msi(uint64_t addr, size_t size, uint64_t data) const
    { msi_(addr, size, data); }

    union config header_;

    std::vector<std::shared_ptr<mmio_blocks>> bars_;

  private:

    std::function<void(uint64_t, size_t, uint64_t&)> read_mem_;
    std::function<void(uint64_t, size_t, uint64_t)> write_mem_;
    std::function<void(uint64_t, unsigned, uint64_t)> msi_;

    uint8_t* header_eol_;
    std::vector<uint8_t*> bar_eols_;
    std::vector<unsigned> bar_sizes_;
};
