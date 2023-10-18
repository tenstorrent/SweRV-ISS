#pragma once

#include <iostream>
#include <linux/pci_regs.h>
#include <linux/virtio_pci.h>
#include <condition_variable>
#include "PciDev.hpp"

#define PCI_DEVICE_ID_VIRTIO_BASE		0x1040
#define PCI_SUBSYS_ID_VIRTIO_BASE		0x0040

#define PCI_VENDOR_ID_REDHAT_QUMRANET		0x1af4
#define PCI_SUBSYSTEM_VENDOR_ID_REDHAT_QUMRANET	0x1af4

#define VIRTQ_SIZE                              32
#define VIRTQ_USED_F_NO_NOTIFY                  1
#define VIRTQ_AVAIL_F_NO_INTERRUPT              1
/* This marks a buffer as continuing via the next field. */
#define VIRTQ_DESC_F_NEXT       1
/* This marks a buffer as write-only (otherwise read-only). */
#define VIRTQ_DESC_F_WRITE      2
/* This means the buffer contains a list of buffer descriptors. */
#define VIRTQ_DESC_F_INDIRECT   4

namespace msix {
  struct cap;
  struct msix_table_entry;
  struct pba_table_entry;
}

class Virtio : public PciDev {

  public:

    // can probably just use included header, but let's not
    struct cap {
      uint8_t cap;
      uint8_t next;
      uint8_t len;
      uint8_t type;
      uint8_t bar;
      uint8_t padding[3];
      uint32_t cfg_offset;
      uint32_t cfg_length;
    } __attribute__ ((packed));


    struct common_cfg {
      uint32_t device_feature_select;
      uint32_t device_feature;
      uint32_t driver_feature_select;
      uint32_t driver_feature;
      uint16_t msix_config;
      uint16_t num_queues;
      uint8_t device_status; // TODO: wait for ok
      uint8_t config_generation;

      uint16_t queue_select; // selects specific queue to write to
      uint16_t queue_size;
      uint16_t queue_msix_vector;
      uint16_t queue_enable;
      uint16_t queue_notify_off;
      uint32_t queue_desc_lo;
      uint32_t queue_desc_hi;
      uint32_t queue_avail_lo;
      uint32_t queue_avail_hi;
      uint32_t queue_used_lo;
      uint32_t queue_used_hi;
    } __attribute__ ((packed));


    struct notify_cap {
      struct cap cap;
      uint32_t notify_off_multiplier = 0;
    } __attribute__ ((packed));


    struct notify_cfg {
      uint32_t notify = 0;
    } __attribute__ ((packed));


    struct pci_cap {
      struct cap cap;
      uint8_t pci_cfg_data[4];
    } __attribute__ ((packed));

    struct virtqueue {
      // The virtring lives in host memory
      // there is an array of descriptors of queue size
      struct descriptor {
        uint64_t address;
        uint32_t length;
        // we ignore indirects here (unlikely to be used)
        uint16_t flags;
        uint16_t next;
      } __attribute__((packed));

      struct avail_ring {
        uint16_t flags;
        uint16_t idx;
        uint16_t ring[VIRTQ_SIZE];
      } __attribute__((packed));

      struct used_ring {
        uint16_t flags;
        uint16_t idx;
        struct elem {
          uint32_t idx;
          uint32_t len;
        };
        elem ring[VIRTQ_SIZE];
      } __attribute__((packed));

      uint16_t size = VIRTQ_SIZE;
      uint16_t msix_vector = VIRTIO_MSI_NO_VECTOR;
      uint16_t enable = 0;

      uint64_t desc_addr;
      uint64_t avail_addr;
      uint64_t used_addr;
      uint16_t last_avail_idx = 0;

      // actual pointers to memory
      struct descriptor* desc;
      struct avail_ring* avail;
      struct used_ring* used;
    };

    Virtio(unsigned subsys_id, unsigned class_code, unsigned num_queues);

    virtual bool setup();

    void interrupts();

  protected:

    // Don't need to write to notify because we're using MSI-X!
    void signal_used(unsigned num, const std::vector<virtqueue::used_ring::elem>& elems);

    void signal_config();

    // Returns true if successful.
    bool get_descriptors(const unsigned num, std::vector<virtqueue::descriptor>& read, std::vector<virtqueue::descriptor>& write, unsigned& head, bool& finished);

    auto& get_vq(unsigned num)
    { return vqs_.at(num); }

    // driver should use this to kick off an operation
    void notify(bool flag, unsigned vq)
    {
      {
        std::lock_guard<std::mutex> lock(notify_mutex_);
        notified_ = flag;
        notified_vq_ = vq;
        __sync_synchronize();
      }
      notify_cond_var_.notify_one();
    }

    // device blocking wait until a new notification
    void wait_for_notify(unsigned& vq)
    {
      std::unique_lock<std::mutex> lock(notify_mutex_);
      notify_cond_var_.wait(lock, [this] () -> bool { return notified_; });
      vq = notified_vq_;
      notified_ = false;
      lock.unlock();
    }

    virtual void reset();

    template <typename T>
    T* get_device_config()
    { return reinterpret_cast<T*>(device_cfg_); }

    uint64_t features_;

  private:

    // Allocate one of each necessary cap for VIRTIO as well as corresponding datas.
    bool allocate_caps(uint32_t& common_cap_offset);

    void initialize_header();

    const unsigned subsys_id_;
    const unsigned class_code_;
    const unsigned num_queues_;

    cap* common_cap_;
    common_cfg* common_cfg_;
    notify_cap* notify_cap_;
    notify_cfg* notify_cfg_;
    // reading the ISR clears it, but we don't need to care if we use MSIs
    cap* isr_cap_;
    uint32_t* isr_cfg_;
    cap* device_cap_;
    uint8_t* device_cfg_;

    msix::cap* msix_cap_;
    msix::msix_table_entry* msix_table_;
    msix::pba_table_entry* pba_table_;

    // For driver configuration
    uint32_t device_feature_selector_;
    uint32_t driver_feature_selector_;
    uint16_t config_msix_vector_;
    uint16_t queue_selector_ = 0;
    std::vector<virtqueue> vqs_;

    std::mutex notify_mutex_;
    std::condition_variable notify_cond_var_;
    bool notified_ = false;
    unsigned notified_vq_ = 0;
};
