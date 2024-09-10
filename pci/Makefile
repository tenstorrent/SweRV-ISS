OFLAGS := -O3
CXXFLAGS := $(OFLAGS) -std=c++17 -fPIC -pedantic -Wall -Wextra -Werror -Ivirtio -I.
DEPFLAGS := -MMD -MP

SRCS := Pci.cpp PciDev.cpp virtio/Virtio.cpp virtio/Blk.cpp
OBJS := $(SRCS:.cpp=.o)
DEPS := $(OBJS:.o=.d)

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $(DEPFLAGS) -o $@ $<

libpci.a: $(OBJS)
	ar rcs $@ $^

-include $(DEPS)

clean:
	rm -f libpci.a $(OBJS) $(DEPS)
