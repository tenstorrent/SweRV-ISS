#include <cassert>
#include <iostream>
#include <cstddef>
#include <cstring>

#include "Pci.hpp"

Pci::Pci(uint32_t config_base, uint32_t config_len, uint32_t mmio_base, size_t mmio_len, unsigned buses, unsigned slots)
  : config_base_(config_base), config_len_(config_len), mmio_base_(mmio_base), mmio_len_(mmio_len), mmio_eol_(mmio_base)
{
  /* config needs 256MB of space minimum */
  buses_.resize(buses);
  for (auto& bus : buses_)
    bus.resize(slots, nullptr);
};


template <typename T>
void
Pci::config_mmio(uint32_t addr, T& data, bool w)
{
  static_assert(sizeof(T) <= 4);

  if (addr < config_base_)
    return;

  addr -= config_base_;
  auto format = address(addr);

  auto slot = find_registered_device(format.bits.bus, format.bits.device, format.bits.function);
  if (slot) {
    if (w) {
      slot->write_config<T>((format.bits.reg << 2) | format.bits.offset, data);
      return;
    }

    data = -1;
    slot->read_config<T>((format.bits.reg << 2) | format.bits.offset, data);
  }
}


template <>
void
Pci::config_mmio<uint64_t>([[maybe_unused]] uint32_t addr, [[maybe_unused]] uint64_t& data, [[maybe_unused]] bool w)
{
  return;
}


template <typename T>
void
Pci::mmio(uint32_t addr, T& data, bool w)
{
  if (addr < mmio_base_)
    return;

  for (auto& mmio : mmio_) {
      if ((mmio->base <= addr) and (addr < (mmio->base + mmio->size))) {
        unsigned offset = addr - mmio->base;
        uint8_t* p = (mmio->bytes + offset);
        if (w) {
          memcpy(p, &data, sizeof(T));
          if (mmio->write_cb)
            mmio->write_cb(offset, sizeof(T));
          return;
        }

        data = -1;
        uint64_t tmp;
        if (mmio->read_cb and mmio->read_cb(offset, tmp)) {
          data = tmp;
          return;
        }
        memcpy(&data, p, sizeof(T));
        return;
      }
  }
}


bool
Pci::register_device(std::shared_ptr<PciDev> dev, unsigned bus, unsigned slot,
                      const std::function<uint8_t*(uint64_t, size_t)>& map,
                      const std::function<void(uint64_t, unsigned, uint64_t)>& write)
{
  if (bus >= buses_.size()) {
    std::cerr << "bus location not instantiated" << std::endl;
    return false;
  }

  if (slot >= buses_.at(bus).size()) {
    std::cerr << "slot location not instantiated" << std::endl;
    return false;
  }

  dev->memmap_ = map;
  dev->msi_ = write;

  buses_.at(bus).at(slot) = dev;
  fixup_bars(dev);
  if (not dev->setup()) {
    std::cerr << "Failed to setup PCI device" << std::endl;
    return false;
  }

  return true;
}


std::shared_ptr<PciDev>
Pci::find_registered_device(unsigned bus, unsigned slot, unsigned function)
{
  if (bus >= buses_.size())
    return nullptr;

  auto& slots = buses_.at(bus);
  if (slot >= slots.size())
    return nullptr;

  return (function == 0)? slots.at(slot) : nullptr;
}


bool
Pci::fixup_bars(std::shared_ptr<PciDev> dev)
{
  if (not dev) {
    std::cerr << "Fixup device does not exist" << std::endl;
    return false;
  }

  for (unsigned bar = 0; bar < 6; bar++) {
    uint32_t size = dev->bar_size(bar);
    if (size) {
      if ((mmio_eol_ + size - 1) < (mmio_base_ + mmio_len_)) {
        // TODO: check for EOR
        uint32_t base = (mmio_eol_ + size - 1) & ~(size - 1);
        auto mmio_blocks = std::make_shared<PciDev::mmio_blocks>(base, size, dev->memmap());
        mmio_.emplace_back(mmio_blocks);

        // Assign MMIO region to BAR.
        dev->header().bits.bar[bar] = base | PCI_BASE_ADDRESS_SPACE_MEMORY;
        dev->bars().at(bar) = mmio_blocks;
        dev->bar_eols().at(bar) = mmio_blocks->bytes;
        mmio_eol_ = base + size;
        continue;
      }

      std::cerr << "Ran out of MMIO memory" << std::endl;
      return false;
    }

    // Mark as unused.
    dev->header().bits.bar[bar] = 0;
  }
  return true;
}


template
void
Pci::config_mmio<uint8_t>(uint32_t addr, uint8_t& data, bool w);

template
void
Pci::config_mmio<uint16_t>(uint32_t addr, uint16_t& data, bool w);

template
void
Pci::config_mmio<uint32_t>(uint32_t addr, uint32_t& data, bool w);

template
void
Pci::mmio<uint8_t>(uint32_t addr, uint8_t& data, bool w);

template
void
Pci::mmio<uint16_t>(uint32_t addr, uint16_t& data, bool w);

template
void
Pci::mmio<uint32_t>(uint32_t addr, uint32_t& data, bool w);

template
void
Pci::mmio<uint64_t>(uint32_t addr, uint64_t& data, bool w);
