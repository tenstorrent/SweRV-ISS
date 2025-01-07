OFLAGS = -O3
CXXFLAGS += -I. -I../ -std=c++20 $(OFLAGS) -fPIC -Wall -Wextra -Werror

SRCS := TraceReader.cpp PageTableMaker.cpp
OBJS := $(SRCS:.cpp=.o)
DEPS := $(OBJS:.o=.d)

default: sample-reader

TraceReader.a: $(OBJS)
	$(AR) cr $@ $^

sample-reader: sample-reader.o TraceReader.a
	$(CXX) -o $@ $^ -lboost_iostreams -lz

-include $(DEPS)

clean:
	rm -f TraceReader.a $(OBJS) $(DEPS) *.o sample-reader
