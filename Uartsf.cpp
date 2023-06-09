#include <iostream>
#include <unistd.h>
#include <poll.h>
#include <termios.h>
#include "Uartsf.hpp"


using namespace WdRiscv;


Uartsf::Uartsf(uint64_t addr, uint64_t size)
  : IoDevice(addr, size), regs_(RegId::N)
{
  auto func = [this]() { this->monitorStdin(); };
  stdinThread_ = std::thread(func);

  regs_.at(RX_FIFO) = RX_EMPTY;
  regs_.at(TX_CTRL) = TX_EN;
  regs_.at(RX_CTRL) = RX_EN;

  struct termios term;
  tcgetattr(fileno(stdin), &term);
  cfmakeraw(&term);
  tcsetattr(fileno(stdin), 0, &term);
}


Uartsf::~Uartsf()
{
  terminate_ = true;
  stdinThread_.join();
}


uint32_t
Uartsf::read(uint64_t addr)
{
  uint64_t ix = (addr - address()) / 4;
  uint32_t res = ix < regs_.size() ? regs_.at(ix) : 0;

  if (ix == RX_FIFO)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      res = regs_.at(RX_FIFO);
      regs_.at(RX_FIFO) = RX_EMPTY;
    }

  return res;
}


void
Uartsf::write(uint64_t addr, uint32_t value)
{
  uint64_t ix = (addr - address()) / 4;

  if (ix ==  TX_FIFO)
    {
      int c = static_cast<int>(value & 0xff);
      if (c)
	{
	  putchar(c);
	  fflush(stdout);
	}
      regs_.at(TX_FIFO) = value;
    }
  else if (ix < regs_.size())
    regs_.at(ix) = value;
}


void
Uartsf::monitorStdin()
{
  struct pollfd inPollfd;

  int fd = fileno(stdin);        // stdin file descriptor
  inPollfd.fd = fd;
  inPollfd.events = POLLIN;

  while (true)
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
		std::cerr << "Uartsf::monitorStdin: unexpected fail on read\n";
	      regs_.at(RX_FIFO) = c;
	      // setInterruptPending(true);
	    }
	}

      // TODO: handle error return codes from poll.
    }
}
