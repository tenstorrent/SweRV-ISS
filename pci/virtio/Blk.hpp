#pragma once

#include <iostream>
#include <linux/virtio_blk.h>
#include <sys/stat.h>
#include "Virtio.hpp"

class Blk : public Virtio {

  public:

    // input file as disk image
    Blk(const std::string& filename, bool readonly);

    ~Blk()
    {
      close(fd_);
    }

  private:

    bool setup() override
    {
      Virtio::setup();

      config_ = get_device_config<virtio_blk_config>();

      struct stat st;
      fstat(fd_, &st);
      config_->capacity = st.st_size/512;
      return true;
    };

    void operator()() final;

    void reset() override
    {
      Virtio::reset();

      // TODO: read capacity override
    };

    int fd_ = -1;
    virtio_blk_config* config_;
};
