#include "PciDev.hpp"
#include <cstring>

std::optional<unsigned>
PciDev::active_addr(uint64_t addr) const
{
  bool command_io = (header_.bits.command & PCI_COMMAND_IO) != 0;
  bool command_memory = (header_.bits.command & PCI_COMMAND_MEMORY) != 0;

  for (int i = 0; i < 6; i++)
    {
      bool io;
      if (not bar_type(i, io))
        continue;

      // ignore 64b BARs for now
      uint64_t lower = header_.bits.bar[i] & (io? PCI_BASE_ADDRESS_IO_MASK : PCI_BASE_ADDRESS_MEM_MASK);
      uint64_t higher = lower + bar_sizes_.at(i);

      if ((command_io and io) || (command_memory and not io))
        if (addr >= lower and addr < higher)
          return i;
    }

  return std::nullopt;
}


bool
PciDev::active_bar(unsigned bar) const
{
  bool command_io = (header_.bits.command & PCI_COMMAND_IO) != 0;
  bool command_memory = (header_.bits.command & PCI_COMMAND_MEMORY) != 0;

  bool io;
  if (not bar_type(bar, io))
    return false;
  if ((io and not command_io) || (not io and not command_memory))
    return false;

  uint64_t address = header_.bits.bar[bar] & (io? PCI_BASE_ADDRESS_IO_MASK : PCI_BASE_ADDRESS_MEM_MASK);
  return (address == bars_.at(bar)->base_) and (address != 0);
}
