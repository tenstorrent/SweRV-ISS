#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <poll.h>
#include "Uart8250.hpp"


using namespace WdRiscv;


Uart8250::Uart8250(uint64_t addr, uint64_t size)
  : IoDevice(addr, size)
{
  auto func = [this]() { this->monitorStdin(); };
  stdinThread_ = std::thread(func);
}


Uart8250::~Uart8250()
{
  terminate_ = true;
  stdinThread_.join();
}



uint32_t
Uart8250::read(uint64_t addr)
{
  if (addr == address())
    {
      std::lock_guard<std::mutex> lock(mutex_);
      uint32_t res = byte_;

      setPendingInterrupt(false);
      byte_ = 0;
      return res;
    }
  return 0;
}


void
Uart8250::write(uint64_t addr, uint32_t value)
{
  if (addr == address())
    {
      putchar(value);
      fflush(stdout);
    }
}


void
Uart8250::monitorStdin()
{
  struct pollfd inPollfd;

  int fd = fileno(stdin);        // stdin file descriptor
  inPollfd.fd = fd;
  inPollfd.events = POLLIN;
  

  while (1)
    {
      if (terminate_)
	return;

      int code = poll(&inPollfd, 1, 20);
      if (code == 0)
	continue;   // Timed out.

      if (code == 1)
	{
	  if ((inPollfd.revents & POLLIN) != 0)
	    {
	      std::lock_guard<std::mutex> lock(mutex_);
	      char c;
	      if (::read(fd, &c, sizeof(c)) != 1)
		std::cerr << "Uart8250::monitorStdin: unexpected fail on read\n";
	      byte_ = c;
	      setPendingInterrupt(true);
	    }
	}

      // TODO: handle error return codes from poll.
    }
}
