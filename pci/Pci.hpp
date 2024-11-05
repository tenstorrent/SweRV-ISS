#pragma once

#include <linux/pci_regs.h>
#include <functional>
#include <vector>
#include <cstdint>
#include <memory>
#include <variant>

#include "PciDev.hpp"

class Pci {

  public:

    // config_base refers to base address of PCI config regions. mmio_base refers to base address of PCI MMIO region (for example for BARs).
    Pci(uint32_t config_base, uint32_t config_len, uint32_t mmio_base,
        size_t mmio_len, unsigned buses, unsigned slots);

    // Returns true if address relies within the defined PCI memory regions.
    bool contains_addr(uint64_t addr) const
    { return (addr >= config_base_ and addr < (config_base_ + config_len_)) or
             (addr >= mmio_base_ and addr < (mmio_base_ + mmio_len_)); }

    template <typename T>
    void access(uint32_t addr, T& data, bool w)
    {
      if (addr >= config_base_ and addr < (config_base_ + config_len_))
        config_mmio<T>(addr, data, w);
      else if (addr >= mmio_base_ and addr < (mmio_base_ + mmio_len_))
        mmio<T>(addr, data, w);

      return;
    }

    // Finds a device on buses and returns a pointer to it if it's registered.
    std::shared_ptr<PciDev> find_registered_device(unsigned bus, unsigned slot,
                                unsigned function)
    {
      if (bus >= buses_.size())
        return nullptr;

      auto& slots = buses_.at(bus);
      if (slot >= slots.size())
        return nullptr;

      return (function == 0)? slots.at(slot) : nullptr;
    }

    // Register a device to the bus.
    bool register_device(std::shared_ptr<PciDev> dev, unsigned bus,
                          unsigned slot)
    {
      if (bus >= buses_.size())
        {
          std::cerr << "bus location not instantiated" << std::endl;
          return false;
        }

      if (slot >= buses_.at(bus).size())
        {
          std::cerr << "slot location not instantiated" << std::endl;
          return false;
        }

      if (not read_mem_ or not write_mem_ or not msi_)
        return false;

      dev->read_mem_ = read_mem_;
      dev->write_mem_ = write_mem_;
      dev->msi_ = msi_;

      buses_.at(bus).at(slot) = dev;

      for (unsigned bar = 0; bar < 6; ++bar)
        {
          uint32_t size = dev->bar_size(bar);
          if (size)
            {
              if ((mmio_eol_ + size - 1) < (mmio_base_ + mmio_len_))
                {
                  // TODO: check for EOR
                  uint32_t base = (mmio_eol_ + size - 1) & ~(size - 1);
                  auto mmio_blocks = std::make_shared<PciDev::mmio_blocks>(base, size);
                  mmio_.emplace_back(mmio_blocks);

                  // Assign MMIO region to BAR.
                  dev->set_bar_base_address(bar, base);
                  dev->set_bar_mmio_blocks(bar, mmio_blocks);
                  mmio_eol_ = base + size;
                  continue;
                }

              std::cerr << "Ran out of MMIO memory" << std::endl;
              return false;
            }

          // Mark as unused.
          dev->set_bar_unused(bar);
        }

      if (not dev->setup())
        {
          std::cerr << "Failed to setup PCI device" << std::endl;
          return false;
        }

      return true;
    }

    void define_read_mem(const std::function<bool(uint64_t, size_t, uint64_t&)> f)
    { read_mem_ = f; }

    void define_write_mem(const std::function<bool(uint64_t, size_t, uint64_t)> f)
    { write_mem_ = f; }

    void define_msi(const std::function<bool(uint64_t, unsigned, uint64_t)> f)
    { msi_ = f; }

  private:

    // The base address is set on CPU side (config and mmio sit in different regions).
    template <typename T>
    void config_mmio(uint32_t addr, T& data, bool w);

    template <typename T>
    void mmio(uint32_t addr, T& data, bool w);

    // Allocate BARs for device specified. Returns true on success (enough memory) and false on failure.
    // We cheat here by using `linux,pci-probe-only` fdt property (under chosen)
    bool fixup_bars(std::shared_ptr<PciDev> dev);

    union address
    {
      address(uint32_t addr) : data(addr) {};

      struct fields
      {
        uint16_t offset : 2;
        uint16_t reg : 10;
        uint16_t function : 3;
        uint16_t device : 5;
        uint16_t bus : 8;
        uint16_t reserved : 3;
        bool enable : 1;
      } __attribute__ ((packed));

      fields bits;
      uint32_t data;
    };

    // Base represents the MMIO (PCI address).
    // For simplicity this can be the same as the CPU address space.
    uint32_t config_base_; // Base address of the configuration region.
    size_t config_len_;    // Size of the configuration region.
    uint32_t mmio_base_; // Base address of MMIO region.
    size_t mmio_len_;    // Size of the MMIO region.
    uint32_t mmio_eol_;  // Represents the current allocated end of the MMIO region.

    std::vector<std::vector<std::shared_ptr<PciDev>>> buses_;

    std::vector<std::shared_ptr<PciDev::mmio_blocks>> mmio_;

    // Callback functions for read/write/msi.
    std::function<bool(uint64_t, size_t, uint64_t&)> read_mem_ = nullptr;
    std::function<void(uint64_t, size_t, uint64_t)> write_mem_ = nullptr;
    std::function<void(uint64_t, unsigned, uint64_t)> msi_ = nullptr;
};
