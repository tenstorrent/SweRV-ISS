#pragma once

#include <iostream>
#include <mutex>
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
      terminate_ = true;
      notify(true, 0);
      task_thread_.join();
    }

  private:

    bool setup() override
    {
      Virtio::setup();

      config_ = get_device_config<virtio_blk_config>();

      struct stat st;
      fstat(fd_, &st);
      config_->capacity = st.st_size/512;
      // launch task thread
      task_thread_ = std::thread([this] () { this->task(); });
      return true;
    };

    void reset() override
    {
      Virtio::reset();

      // TODO: read capacity override
    };

    void task();

    int fd_ = -1;
    virtio_blk_config* config_;
    std::thread task_thread_;
    bool terminate_ = false;
};
