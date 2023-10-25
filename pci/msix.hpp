#pragma once

#include <iostream>
#include "PciDev.hpp"

namespace msix {

  struct cap {
    uint8_t cap;
    uint8_t next;
    uint16_t ctrl;
    uint32_t msix_table;
    uint32_t pba_table;
  } __attribute__((packed));

  struct msix_table_entry {
    uint32_t address_lo;
    uint32_t address_hi;
    uint32_t data; // 16 bits
    uint32_t ctrl;
  } __attribute__((packed));

  struct pba_table_entry {
    uint64_t pending;
  } __attribute__((packed));


  // For testing, let's stick to 64 IRQs. User needs to chain the list of CAPs.
  // Allocate cap for MSIX, as well as corresponding msix and pba tables.
  template <typename T>
  bool allocate_cap(T& dev, unsigned num, cap*& cap_entry, uint32_t& cap_offset,
                    msix_table_entry*& msix_table, pba_table_entry*& pba_table) {
    if (num == 0)
      return false;

    cap_entry = reinterpret_cast<cap*>(dev.template ask_header_blocks<uint32_t>(sizeof(cap), cap_offset));
    if (not cap_entry) {
      std::cerr << "No more space for MSIX cap entry" << std::endl;
      return false;
    }

    cap_entry->cap = PCI_CAP_ID_MSIX;
    cap_entry->next = 0;
    cap_entry->ctrl = num - 1; // 1 less than number of MSIX desired

    uint32_t msix_table_offset;
    msix_table = reinterpret_cast<msix_table_entry*>(dev.template ask_bar_blocks<uint64_t>(2, num*sizeof(msix_table_entry), msix_table_offset));
    if (not msix_table) {
      std::cerr << "No more space for MSIX table" << std::endl;
      return false;
    }

    cap_entry->msix_table = msix_table_offset | 2;

    uint32_t pba_table_offset;
    unsigned num_pba_entries = (num - 1)/64 + 1;
    pba_table = reinterpret_cast<pba_table_entry*>(dev.template ask_bar_blocks<uint64_t>(2, num_pba_entries*sizeof(pba_table_entry), pba_table_offset));
    if (not pba_table) {
      std::cerr << "No more space for MSIX PBA table" << std::endl;
      return false;
    }

    cap_entry->pba_table = pba_table_offset | 2;
    return true;
  }


  // TODO: cache this stuff
  void check_interrupts(unsigned num, cap* cap_entry, msix_table_entry* msix_table, pba_table_entry* pba_table, std::vector<std::pair<uint64_t, uint16_t>>& msixs, bool clear) {
    // TODO: need to mutex here?
    constexpr unsigned pba_width = 8*sizeof(pba_table_entry);

    if (not (cap_entry->ctrl & PCI_MSIX_FLAGS_ENABLE) or (cap_entry->ctrl & PCI_MSIX_FLAGS_MASKALL))
      return;

    for (unsigned i = 0; i < num; i += pba_width) {
      auto& pba = pba_table[i/pba_width];

      for (unsigned j = 0; j < pba_width; j++) {
        unsigned irq = (pba.pending >> j) & 1;
        if (irq) {
          auto msix = msix_table[i*pba_width + j];
          // When set to 0, generate message
          if (not (msix.ctrl & PCI_MSIX_ENTRY_CTRL_MASKBIT)) {
            msixs.emplace_back((uint64_t(msix.address_hi) << 32) | msix.address_lo, msix.data & 0xffff);
            // clear out pending bit if it's sent out
            pba.pending &= ~(uint64_t(clear) << j);
          }
        }
      }
    }
  }


  template <typename T>
  void initialize_header(T& dev) {
    if (not dev.bar_size(2))
      dev.set_bar_size(2, 0x1000);
    else
      std::cerr << "Bar 2 size already set" << std::endl;
  }
}
