#include <TraceReader.hpp>
#include "Pte.hpp"


int
main(int argc, char* argv[])
{
  using namespace WhisperUtil;

  TraceReader* reader;

  if (argc > 2)
    {
      reader = new TraceReader(argv[1], argv[2]);
    }
  else if (argc > 1)
    {
      reader = new TraceReader(argv[1]);
    }
  else
    {
      return 0;
    }

  if (not *reader)
    {
      std::cerr << "Failed to open " << argv[1] << " for input.\n";
      return 1;
    }

  TraceRecord record;

  std::vector<uint64_t> walk;

  reader->definePageTableMaker(0x100000000, PageTableMaker::Sv57, 4*1024*1024);

  uint64_t nn = 0;
  while (reader->nextRecord(record))
    {
      record.print(std::cout);
    }

  if (not reader->eof())
    return 1;

  return nn;
}
