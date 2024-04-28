#pragma once

#include <iostream>
#include <linux/virtio_blk.h>
#include <sys/stat.h>
#include "Virtio.hpp"

class Blk : public Virtio {

  public:

    // input file as disk image
    Blk(bool readonly);

    ~Blk()
    {
      if (fd_)
        close(fd_);
    }

    bool open_file(const std::string& filename);

  private:

    bool setup() final
    {
      if (not fd_)
        return false;

      Virtio::setup();

      config_ = get_device_config<virtio_blk_config>();

      struct stat st;
      fstat(fd_, &st);
      config_->capacity = st.st_size/512;
      return true;
    };

    void operator()(unsigned vq) final;

    void reset() final
    {
      Virtio::reset();

      // TODO: read capacity override
    };

    int fd_ = -1;
    virtio_blk_config* config_;
};
