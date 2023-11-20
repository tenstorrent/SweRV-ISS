#pragma once

#include <linux/pci_regs.h>
#include <functional>
#include <vector>
#include <cstdint>
#include <memory>

#include "PciDev.hpp"


class Pci {

  public:

    // config_base refers to base address of PCI config regions. mmio_base refers to base address of PCI MMIO region (for example for BARs).
    Pci(uint32_t config_base, uint32_t mmio_base, size_t mmio_len, unsigned buses, unsigned slots);

    // The base address is set on CPU side (config and mmio sit in different regions).
    template <typename T>
    void config_mmio(uint32_t addr, T& data, bool w);

    template <typename T>
    void mmio(uint32_t addr, T& data, bool w);

    // Finds a device on buses and returns a pointer to it if it's registered.
    std::shared_ptr<PciDev> find_registered_device(unsigned bus, unsigned slot, unsigned function);

    // Register a device to the bus.
    bool register_device(std::shared_ptr<PciDev> dev, unsigned bus, unsigned slot,
                          const std::function<uint8_t*(uint64_t, size_t)>& map,
                          const std::function<void(uint64_t, unsigned, uint64_t)>& write);

  private:

    // Allocate BARs for device specified. Returns true on success (enough memory) and false on failure.
    // We cheat here by using `linux,pci-probe-only` fdt property (under chosen)
    bool fixup_bars(std::shared_ptr<PciDev> dev);

    union address {

      address(uint32_t addr) : data(addr) {};

      struct fields {
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

    // Base represents the MMIO (PCI address). For simplicity this can be the same as the CPU address (make sure to mark 32b memory space).
    uint32_t config_base_;
    uint32_t mmio_base_;
    uint32_t mmio_eol_;
    size_t mmio_len_;

    std::vector<std::vector<std::shared_ptr<PciDev>>> buses_;
    std::vector<std::shared_ptr<PciDev::mmio_blocks>> mmio_;
};
