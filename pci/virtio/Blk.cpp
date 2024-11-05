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

  if (fd_ < 0)
    {
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

  while (not finished)
    {
      std::vector<virtqueue::descriptor> reads, writes;
      unsigned head;

      if (not get_descriptors(vq, reads, writes, head, finished))
        break;

      // order of descriptors is header, buffer, status
      if ((reads.size() + writes.size()) != 3)
        {
          std::cerr << "Unexpected descriptors for virtio-blk (expected 3): " << reads.size() + writes.size() << std::endl;
          break;
        }

      unsigned read_ptr = 0, write_ptr = 0;
      auto desc = reads.at(read_ptr++);
      uint32_t header_type;
      uint64_t header_sector;
      read_mem(desc.address + offsetof(virtio_blk_outhdr, type), header_type);
      read_mem(desc.address + offsetof(virtio_blk_outhdr, sector), header_sector);

      desc = (header_type == VIRTIO_BLK_T_OUT)? reads.at(read_ptr++) : writes.at(write_ptr++);
      uint64_t buffer_addr = desc.address;
      uint32_t buffer_length = desc.length;

      desc = writes.at(write_ptr++);
      uint64_t status_addr = desc.address;

      if (header_type != VIRTIO_BLK_T_GET_ID)
        {
          // TODO: use pread/pwrite instead
          if (lseek(fd_, header_sector * 512, SEEK_SET) < 0)
            {
              write_mem(status_addr, VIRTIO_BLK_S_IOERR);
              elems.push_back({head, 0});
              continue;
            }
        }

      int res = 0;
      switch (header_type)
        {
          case VIRTIO_BLK_T_IN:
            // In reality we could optimize this by reading in chunks (rest of first page + second).
            // Instead, we just operate on bytes.
            for (uint32_t i = 0; i < buffer_length; ++i)
              {
                uint8_t data = 0;
                res = res or (read(fd_, &data, 1) < 0);
                write_mem(buffer_addr + i, data);
              }
            break;
          case VIRTIO_BLK_T_OUT:
            for (uint32_t i = 0; i < buffer_length; ++i)
              {
                uint8_t data = 0;
                read_mem(buffer_addr + i, data);
                res = res or (write(fd_, &data, 1) < 0);
              }
            break;
          case VIRTIO_BLK_T_GET_ID:
            write_mem(buffer_addr, '0');
            write_mem(buffer_addr + 1, '\0');
            break;
        }

      write_mem(status_addr, (res < 0)? VIRTIO_BLK_S_IOERR : VIRTIO_BLK_S_OK);
      elems.push_back({head, (header_type == VIRTIO_BLK_T_IN)? uint32_t(buffer_length) : 0});
    }

  signal_used(vq, elems);
}
