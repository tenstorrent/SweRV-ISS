#include <cstring>
#include <atomic>
#include <linux/virtio_config.h>
#include "Virtio.hpp"
#include "msix.hpp"

Virtio::Virtio(unsigned subsys_id, unsigned class_code, unsigned num_queues)
  : subsys_id_(subsys_id), class_code_(class_code), num_queues_(num_queues)
{
  initialize_header();
  msix::initialize_header(*this);
  vqs_.resize(num_queues);

  // SR_IOV
  features_ = uint64_t(1) << VIRTIO_F_VERSION_1;
}


bool
Virtio::setup()
{
  uint32_t common_cap_offset;
  uint32_t msix_cap_offset;
  if (not msix::allocate_cap(*this, num_queues_ + 1 /* config? */, msix_cap_, msix_cap_offset,
                              msix_table_, pba_table_) or
      not allocate_caps(common_cap_offset)) {

    std::cerr << "Failed to allocate all caps for virtio" << std::endl;
    return false;
  }

  header().bits.cap = msix_cap_offset;
  msix_cap_->next = common_cap_offset;

  // setup write callback for virtqueue writes
  auto& bar = bars().at(1);

  // TODO: support feature selects...  config vector?
  bar->write_cb = [&](uint32_t offset, size_t len) {
    std::lock_guard<std::mutex> lock(m_);
    uint32_t data = 0;
    uint64_t mask = 0xffffffffULL;
    auto& vq = get_vq(queue_selector_);
    // probably don't need to read back here?
    memcpy(&data, bar->bytes + offset, len);
    switch (offset) {
      case VIRTIO_PCI_COMMON_DFSELECT:
        device_feature_selector_ = data;
        break;
      case VIRTIO_PCI_COMMON_DF:
        assert(false);
        break;
      case VIRTIO_PCI_COMMON_GFSELECT:
        driver_feature_selector_ = data;
        break;
      // ignore GF for now...
      case VIRTIO_PCI_COMMON_MSIX:
        config_msix_vector_ = data;
        break;
      case VIRTIO_PCI_COMMON_STATUS:
        if (data & VIRTIO_CONFIG_S_FAILED) {
          std::cerr << "Driver gave up on device" << std::endl;
          return;
        }

        if (!data)
          reset();
        break;
      case VIRTIO_PCI_COMMON_Q_SELECT:
        queue_selector_ = data;
        break;
      case VIRTIO_PCI_COMMON_Q_SIZE:
        vq.size = data;
        break;
      case VIRTIO_PCI_COMMON_Q_MSIX:
        vq.msix_vector = data;
        break;
      case VIRTIO_PCI_COMMON_Q_ENABLE:
        vq.enable = data;
        if (data) {
          vq.desc = reinterpret_cast<virtqueue::descriptor*>(memmap()(vq.desc_addr, VIRTQ_SIZE*sizeof(virtqueue::descriptor)));
          vq.avail = reinterpret_cast<virtqueue::avail_ring*>(memmap()(vq.avail_addr, sizeof(virtqueue::avail_ring)));
          vq.used = reinterpret_cast<virtqueue::used_ring*>(memmap()(vq.used_addr, sizeof(virtqueue::used_ring)));
          // vq.used->flags = VIRTQ_USED_F_NO_NOTIFY;
        }
        break;
      case VIRTIO_PCI_COMMON_Q_DESCLO:
        vq.desc_addr = (vq.desc_addr & (mask << 32)) | data;
        break;
      case VIRTIO_PCI_COMMON_Q_DESCHI:
        vq.desc_addr = (vq.desc_addr & mask) | (uint64_t(data) << 32);
        break;
      case VIRTIO_PCI_COMMON_Q_AVAILLO:
        vq.avail_addr = (vq.avail_addr & (mask << 32)) | data;
        break;
      case VIRTIO_PCI_COMMON_Q_AVAILHI:
        vq.avail_addr = (vq.avail_addr & mask) | (uint64_t(data) << 32);
        break;
      case VIRTIO_PCI_COMMON_Q_USEDLO:
        vq.used_addr = (vq.used_addr & (mask << 32)) | data;
        break;
      case VIRTIO_PCI_COMMON_Q_USEDHI:
        vq.used_addr = (vq.used_addr & mask) | (uint64_t(data) << 32);
        break;
      case VIRTIO_PCI_COMMON_Q_USEDHI + 4: // notify
        if (vq.enable) {
          notify(true, data);
        }
        break;
    }
  };

  bar->read_cb = [&] (uint32_t offset, uint64_t& data) -> bool {
    std::lock_guard<std::mutex> lock(m_);
    uint64_t mask = 0xffffffffULL << 32;
    auto& vq = get_vq(queue_selector_);
    switch (offset) {
      case VIRTIO_PCI_COMMON_DFSELECT:
        data = device_feature_selector_; return true;
      case VIRTIO_PCI_COMMON_DF:
        data = features_ >> (32*device_feature_selector_); return true;
      case VIRTIO_PCI_COMMON_GFSELECT:
        data = driver_feature_selector_; return true;
      case VIRTIO_PCI_COMMON_MSIX:
        data = config_msix_vector_; return true;
      case VIRTIO_PCI_COMMON_Q_SELECT:
        data = queue_selector_; return true;
      case VIRTIO_PCI_COMMON_Q_SIZE:
        data = vq.size; return true;
      case VIRTIO_PCI_COMMON_Q_MSIX:
        data = vq.msix_vector; return true;
      case VIRTIO_PCI_COMMON_Q_ENABLE:
        data = vq.enable; return true;
      case VIRTIO_PCI_COMMON_Q_DESCLO:
        data = vq.desc_addr & mask; return true;
      case VIRTIO_PCI_COMMON_Q_DESCHI:
        data = vq.desc_addr >> 32; return true;
      case VIRTIO_PCI_COMMON_Q_AVAILLO:
        data = vq.avail_addr & mask; return true;
      case VIRTIO_PCI_COMMON_Q_AVAILHI:
        data = vq.avail_addr >> 32; return true;
      case VIRTIO_PCI_COMMON_Q_USEDLO:
        data = vq.used_addr & mask; return true;
      case VIRTIO_PCI_COMMON_Q_USEDHI:
        data = vq.used_addr >> 32; return true;
    };
    return false;
  };

  // launch task thread
  task_thread_ = std::thread([this] () { (*this)(); });
  return true;
}


void
Virtio::interrupts()
{
  // We don't spawn another thread for issuing MSIs
  std::vector<std::pair<uint64_t, uint16_t>> msis;
  msix::check_interrupts(num_queues_ + 1, msix_cap_, msix_table_, pba_table_, msis, true);

  __sync_synchronize();
  for (auto& [addr, data] : msis)
    msi()(addr, 4, data);
}


void
Virtio::signal_used(unsigned num, const std::vector<virtqueue::used_ring::elem>& elems)
{
  std::lock_guard<std::mutex> lock(m_);
  auto& vq = get_vq(num);
  if (not vq.enable or (elems.size() == 0))
    return;

  for (const auto& elem : elems) {
    vq.used->ring[vq.used->idx % vq.size] = elem;
    vq.used->idx++;
  }

  uint16_t avail_flags = vq.avail->flags;
  uint16_t msix = vq.msix_vector;
  if (msix != VIRTIO_MSI_NO_VECTOR and not (avail_flags & VIRTQ_AVAIL_F_NO_INTERRUPT)) {
    constexpr unsigned pba_width = 8*sizeof(msix::pba_table_entry);
    auto& pba = pba_table_[msix/pba_width];
    auto offset = msix%pba_width;
    pba.pending |= uint64_t(1) << offset;
  }

  interrupts();
}


void
Virtio::signal_config()
{
  std::lock_guard<std::mutex> lock(m_);
  auto msix = config_msix_vector_;
  if (config_msix_vector_ != VIRTIO_MSI_NO_VECTOR) {
    constexpr unsigned pba_width = 8*sizeof(msix::pba_table_entry);
    auto& pba = pba_table_[msix/pba_width];
    auto offset = msix%pba_width;
    pba.pending |= uint64_t(1) << offset;
  }

  interrupts();
}


bool
Virtio::get_descriptors(const unsigned num, std::vector<virtqueue::descriptor>& read, std::vector<virtqueue::descriptor>& write, unsigned& head, bool& finished)
{
  auto& vq = get_vq(num);
  bool last = false;

  if (vq.last_avail_idx == vq.avail->idx)
    return false;

  uint16_t avail_idx = vq.last_avail_idx % vq.size;
  uint16_t desc_idx = head = vq.avail->ring[avail_idx];
  while (not last) {
    assert(desc_idx < vq.size);
    auto descriptor = vq.desc[desc_idx];
    last = !(descriptor.flags & VIRTQ_DESC_F_NEXT);
    desc_idx = descriptor.next;
    if (descriptor.flags & VIRTQ_DESC_F_WRITE)
      write.push_back(descriptor);
    else
      read.push_back(descriptor);
  };

  finished = (++vq.last_avail_idx) == vq.avail->idx;
  return true;
}


void
Virtio::reset()
{
  queue_selector_ = 0;
  config_msix_vector_ = VIRTIO_MSI_NO_VECTOR;
  for (auto& vq : vqs_) {
    vq.msix_vector = VIRTIO_MSI_NO_VECTOR;
    vq.enable = 0;
    vq.desc_addr = 0;
    vq.avail_addr = 0;
    vq.used_addr = 0;
    vq.last_avail_idx = 0;

    if (vq.desc)
      memset(vq.desc, 0, VIRTQ_SIZE*sizeof(virtqueue::descriptor));
    if (vq.avail)
      memset(vq.avail, 0, sizeof(virtqueue::avail_ring));
    if (vq.used)
      memset(vq.used, 0, sizeof(virtqueue::used_ring));
  }
}


// Allocate one of each necessary cap for VIRTIO as well as corresponding datas.
bool
Virtio::allocate_caps(uint32_t& common_cap_offset)
{
  // I read somewhere this needed to be 8B aligned
  common_cap_ = reinterpret_cast<cap*>(ask_header_blocks<uint32_t>(sizeof(cap), common_cap_offset));
  if (not common_cap_) {
    std::cerr << "No more space for VIRTIO common cap entry" << std::endl;
    return false;
  }

  common_cap_->cap = PCI_CAP_ID_VNDR;
  common_cap_->len = sizeof(cap);
  common_cap_->type = VIRTIO_PCI_CAP_COMMON_CFG;
  common_cap_->bar = 1;

  uint32_t common_cfg_offset;
  common_cfg_ = reinterpret_cast<common_cfg*>(ask_bar_blocks<uint32_t>(1, sizeof(common_cfg), common_cfg_offset));
  if (not common_cfg_) {
    std::cerr << "No more space for VIRTIO common cfg structure" << std::endl;
    return false;
  }
  common_cap_->cfg_offset = common_cfg_offset;
  common_cap_->cfg_length = sizeof(common_cfg);

  common_cfg_->msix_config = VIRTIO_MSI_NO_VECTOR;
  common_cfg_->num_queues = num_queues_;

  uint32_t notify_cap_offset;
  notify_cap_ = reinterpret_cast<notify_cap*>(ask_header_blocks<uint32_t>(sizeof(notify_cap), notify_cap_offset));
  if (not notify_cap_) {
    std::cerr << "No more space for VIRTIO notify cap entry" << std::endl;
    return false;
  }
  notify_cap_->cap.cap = PCI_CAP_ID_VNDR;
  notify_cap_->cap.len = sizeof(notify_cap);
  notify_cap_->cap.type = VIRTIO_PCI_CAP_NOTIFY_CFG;
  notify_cap_->cap.bar = 1;
  // we set to 0, so same notify address is used for all queues
  notify_cap_->notify_off_multiplier = 0;

  uint32_t notify_cfg_offset;
  notify_cfg_ = reinterpret_cast<notify_cfg*>(ask_bar_blocks<uint32_t>(1, sizeof(notify_cfg), notify_cfg_offset));
  if (not notify_cfg_) {
    std::cerr << "No more space for VIRTIO notify cfg structure" << std::endl;
    return false;
  }
  notify_cap_->cap.cfg_offset = notify_cfg_offset;
  notify_cap_->cap.cfg_length = sizeof(notify_cfg);

  // Since we use MSI-X, we don't need this either.
  uint32_t isr_cap_offset;
  isr_cap_ = reinterpret_cast<cap*>(ask_header_blocks<uint32_t>(sizeof(cap), isr_cap_offset));
  if (not isr_cap_) {
    std::cerr << "No more space for VIRTIO isr cap entry" << std::endl;
    return false;
  }
  isr_cap_->cap = PCI_CAP_ID_VNDR;
  isr_cap_->len = sizeof(cap);
  isr_cap_->type = VIRTIO_PCI_CAP_ISR_CFG;
  isr_cap_->bar = 1;

  uint32_t isr_cfg_offset;
  isr_cfg_ = reinterpret_cast<uint32_t*>(ask_bar_blocks<uint32_t>(1, sizeof(uint32_t), isr_cfg_offset));
  if (not isr_cfg_) {
    std::cerr << "No more space for VIRTIO isr cfg structure" << std::endl;
    return false;
  }
  isr_cap_->cfg_offset = isr_cfg_offset;
  isr_cap_->cfg_length = sizeof(uint32_t);

  uint32_t device_cap_offset;
  device_cap_ = reinterpret_cast<cap*>(ask_header_blocks<uint32_t>(sizeof(cap), device_cap_offset));
  if (not device_cap_) {
    std::cerr << "No more space for VIRTIO device cap entry" << std::endl;
    return false;
  }
  device_cap_->cap = PCI_CAP_ID_VNDR;
  device_cap_->len = sizeof(cap);
  device_cap_->type = VIRTIO_PCI_CAP_DEVICE_CFG;
  device_cap_->bar = 1;

  uint32_t device_cfg_offset;
  // device dependent allocation, let's just allocate 128B
  device_cfg_ = ask_bar_blocks<uint32_t>(1, 128*sizeof(uint8_t), device_cfg_offset);
  if (not device_cfg_) {
    std::cerr << "No more space for VIRTIO device cfg structure" << std::endl;
    return false;
  }
  device_cap_->cfg_offset = device_cfg_offset;
  device_cap_->cfg_length = 128*sizeof(uint8_t);

  // Apparently nobody uses this for accessing BAR registers so we don't keep it around and we
  // don't allocate its corresponding structure.
  uint32_t pci_cap_offset;
  auto pci_cap_ = reinterpret_cast<pci_cap*>(ask_header_blocks<uint32_t>(sizeof(pci_cap), pci_cap_offset));
  if (not pci_cap_) {
    std::cerr << "No more space for VIRTIO pci cap entry" << std::endl;
    return false;
  }
  pci_cap_->cap.cap = PCI_CAP_ID_VNDR;
  pci_cap_->cap.len = sizeof(pci_cap);
  pci_cap_->cap.type = VIRTIO_PCI_CAP_PCI_CFG;

  common_cap_->next = notify_cap_offset;
  notify_cap_->cap.next = isr_cap_offset;
  isr_cap_->next = device_cap_offset;
  device_cap_->next = pci_cap_offset;
  pci_cap_->cap.next = 0;
  return true;
}


void
Virtio::initialize_header() {
  header().bits.vendor_id = PCI_VENDOR_ID_REDHAT_QUMRANET;
  header().bits.device_id = PCI_DEVICE_ID_VIRTIO_BASE + subsys_id_;
  header().bits.command = PCI_COMMAND_IO | PCI_COMMAND_MEMORY;
  header().bits.status = PCI_STATUS_CAP_LIST;
  header().bits.class_code[0] = class_code_ & 0xff;
  header().bits.class_code[1] = (class_code_ >> 8) & 0xff;
  header().bits.class_code[2] = (class_code_ >> 16) & 0xff;
  header().bits.header_type = PCI_HEADER_TYPE_NORMAL;
  header().bits.subsys_vendor_id = PCI_SUBSYSTEM_VENDOR_ID_REDHAT_QUMRANET;
  header().bits.subsys_id = PCI_SUBSYS_ID_VIRTIO_BASE + subsys_id_;

  if (not bar_size(1))
    set_bar_size(1, 0x1000);
  else
    std::cerr << "Bar 1 size already set" << std::endl;
}
