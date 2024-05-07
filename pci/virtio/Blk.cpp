#include <linux/virtio_ids.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

#include "Blk.hpp"


Blk::Blk(bool readonly) : Virtio(VIRTIO_ID_BLOCK, 0x018000, 1)
{
  if (readonly)
    features_ |= uint64_t(1) << VIRTIO_BLK_F_RO;
}


bool
Blk::open_file(const std::string& filename)
{
  int flags = (features_ & VIRTIO_BLK_F_RO)? O_RDONLY : O_RDWR;
  fd_ = open(filename.c_str(), flags);

  if (fd_ < 0) {
    std::cerr << "Error: Failed to open file " << filename << " as block device " << std::endl;
    return false;
  }

  return true;
}


void
Blk::operator()(unsigned vq)
{
  if (not fd_)
    return;

  std::vector<virtqueue::used_ring::elem> elems;
  bool finished = false;

  while (not finished) {
    std::vector<virtqueue::descriptor> reads, writes;
    unsigned head;

    if (not get_descriptors(vq, reads, writes, head, finished))
      break;

    // order of descriptors is header, buffer, status
    if ((reads.size() + writes.size()) != 3) {
      std::cerr << "unexpected descriptors for virtio-blk (expected 3): " << reads.size() + writes.size() << std::endl;
      break;
    }

    unsigned read_ptr = 0, write_ptr = 0;
    auto desc = reads.at(read_ptr++);
    auto header = reinterpret_cast<virtio_blk_outhdr*>(memmap()(desc.address, desc.length));

    desc = (header->type == VIRTIO_BLK_T_OUT)? reads.at(read_ptr++) : writes.at(write_ptr++);
    auto buffer = reinterpret_cast<uint8_t*>(memmap()(desc.address, desc.length));
    auto length = desc.length;

    desc = writes.at(write_ptr++);
    auto status = reinterpret_cast<uint8_t*>(memmap()(desc.address, desc.length));

    if (header->type != VIRTIO_BLK_T_GET_ID) {
      // TODO: use pread/pwrite instead
      if (lseek(fd_, header->sector * 512, SEEK_SET) < 0) {
        *status = VIRTIO_BLK_S_IOERR;
        elems.push_back({head, 0});
        continue;
      }
    }

    int res = 0;
    switch (header->type) {
      case VIRTIO_BLK_T_IN:
        res = read(fd_, buffer, length);
        break;
      case VIRTIO_BLK_T_OUT:
        res = write(fd_, buffer, length);
        break;
      case VIRTIO_BLK_T_GET_ID:
        buffer[0] = '0';
        buffer[1] = '\0';
        break;
    }

    *status = (res < 0)? VIRTIO_BLK_S_IOERR : VIRTIO_BLK_S_OK;
    elems.push_back({head, (header->type == VIRTIO_BLK_T_IN)? uint32_t(length): 0});
  }

  signal_used(vq, elems);
}
