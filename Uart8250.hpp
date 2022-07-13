#pragma once

#include <thread>
#include <atomic>
#include <mutex>
#include "IoDevice.hpp"


namespace WdRiscv
{

  class Uart8250 : public IoDevice
  {
  public:

    Uart8250(uint64_t addr, uint64_t size);

    ~Uart8250();

    uint32_t read(uint64_t addr) override;

    void write(uint64_t addr, uint32_t value) override;

  private:

    /// This runs in its own thread. It monitors the standard input and
    /// marks interrupt pending when input is possible placing the input
    /// character in byte_ for the Uart to consme.
    void monitorStdin();

    uint32_t byte_ = 0;  // Pending input byte.
    std::thread stdinThread_;
    std::atomic<bool> terminate_ = false;
    std::mutex mutex_;   // Synchronize access to byte_ with stdinThread_.
  };
}
