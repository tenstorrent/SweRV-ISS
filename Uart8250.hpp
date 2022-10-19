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

    uint8_t ier_ = 0;     // Interrupt enable
    uint8_t iir_ = 1;     // Interrupt id
    uint8_t lcr_ = 0;     // Line control
    uint8_t mcr_ = 0;     // Modem control
    uint8_t lsr_ = 0x60;  // Line satus
    uint8_t msr_ = 0;     // Modem status
    uint8_t scr_ = 0;     // Scratch
    uint8_t fcr_ = 0;     // Fifo control
    uint8_t dll_ = 0x1;   // Divisor latch lsb
    uint8_t dlm_ = 0x1;   // Divisor latch msb

    std::thread stdinThread_;
    std::atomic<bool> terminate_ = false;
    std::mutex mutex_;   // Synchronize access to byte_ with stdinThread_.
  };
}
