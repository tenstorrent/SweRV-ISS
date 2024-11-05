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

  auto dev = find_registered_device(format.bits.bus, format.bits.device,
                                     format.bits.function);
  if (dev)
    {
      if (w)
        dev->write_config<T>((format.bits.reg << 2) | format.bits.offset, data);
      else
        dev->read_config<T>((format.bits.reg << 2) | format.bits.offset, data);
    }
}


template <>
void
Pci::config_mmio<uint64_t>([[maybe_unused]] uint32_t addr,
                           [[maybe_unused]] uint64_t& data,
                           [[maybe_unused]] bool w)
{ return; }


template <typename T>
void
Pci::mmio(uint32_t addr, T& data, bool w)
{
  if (addr < mmio_base_)
    return;

  if (not w)
    data = -1;

  for (auto& mmio : mmio_)
    {
      if ((mmio->base_ <= addr) and (addr < (mmio->base_ + mmio->size_)))
        {
          unsigned offset = addr - mmio->base_;
          uint8_t* p = (mmio->bytes_.data() + offset);
          if (w)
            {
              if (mmio->write_dev_)
                mmio->write_dev_(data, offset, sizeof(T));
              else
                memcpy(p, &data, sizeof(T));
            }
          else
            {
              if (mmio->read_dev_)
                data = mmio->read_dev_(offset, sizeof(T));
              else
                memcpy(&data,  p, sizeof(T));
            }
          return;
        }
    }
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
