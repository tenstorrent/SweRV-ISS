#include "PciDev.hpp"
#include <cstring>

template <typename T>
void
PciDev::write_config(uint8_t offset, T data)
{
  // we don't allow accesses which cross 4B boundary
  if (((offset & 3) + sizeof(T)) > 4)
    return;

  // we don't guard config writes with mask (other than BARs)
  // probably ok in assuming these will be 4B aligned
  if (offset >= PCI_BASE_ADDRESS_0 and offset <= (PCI_BASE_ADDRESS_5 + 3)) {
    bool io;
    uint8_t bar = ((offset & ~uint32_t(0x3)) - PCI_BASE_ADDRESS_0) >> 2; // get corresponding bar
    // probably ok in assuming these will be 4B aligned

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


template <typename T>
void
PciDev::read_config(uint8_t offset, T& data) const
{
  // we don't allow accesses which cross 4B boundary
  if (((offset & 3) + sizeof(T)) > 4)
    return;

  const void* p = &header_.data[offset];
  memcpy(&data, p, sizeof(data));
}


std::tuple<bool, unsigned>
PciDev::active_addr(uint64_t addr) const
{
  bool command_io = (header_.bits.command & PCI_COMMAND_IO) != 0;
  bool command_memory = (header_.bits.command & PCI_COMMAND_MEMORY) != 0;

  for (int i = 0; i < 6; i++) {
    bool io;
    if (not bar_type(i, io))
      continue;

    // ignore 64b BARs for now
    uint64_t lower = header_.bits.bar[i] & (io? PCI_BASE_ADDRESS_IO_MASK : PCI_BASE_ADDRESS_MEM_MASK);
    uint64_t higher = lower + bar_sizes_.at(i);

    if ((command_io and io) || (command_memory and not io))
      if (addr >= lower and addr < higher)
        return std::make_tuple(true, i);
  }

  return std::make_tuple(false, 0);
}


bool
PciDev::active_bar(unsigned bar) const
{
  bool command_io = (header_.bits.command & PCI_COMMAND_IO) != 0;
  bool command_memory = (header_.bits.command & PCI_COMMAND_MEMORY) != 0;

  bool io;
  if (not bar_type(bar, io))
    return false;
  uint64_t address = header_.bits.bar[bar] & (io? PCI_BASE_ADDRESS_IO_MASK : PCI_BASE_ADDRESS_MEM_MASK);

  if ((io and not command_io) || (not io and not command_memory))
    return false;

  return (address == bars_.at(bar)->base) and (address != 0);
}

template
void
PciDev::write_config<uint8_t>(uint8_t offset, uint8_t data);

template
void
PciDev::write_config<uint16_t>(uint8_t offset, uint16_t data);

template
void
PciDev::write_config<uint32_t>(uint8_t offset, uint32_t data);

template
void
PciDev::read_config<uint8_t>(uint8_t offset, uint8_t& data) const;

template
void
PciDev::read_config<uint16_t>(uint8_t offset, uint16_t& data) const;

template
void
PciDev::read_config<uint32_t>(uint8_t offset, uint32_t& data) const;
