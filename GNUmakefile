INSTALL_DIR := .

PROJECT := whisper
PY_PROJECT := $(PROJECT)$(shell python3-config --extension-suffix)

# For Dynamic linking to Boost Library use:
# make STATIC_LINK=0
STATIC_LINK := 1

# We use boost 1.67.
# Set the BOOST_ROOT environment variable to point to the base install
# location of the Boost Libraries
BOOST_ROOT := /wdc/apps/utilities/boost-1.67
BOOST_DIR := $(BOOST_ROOT)
# For Various Installation types of Boost Library
BOOST_INC := $(wildcard $(BOOST_DIR) $(BOOST_DIR)/include)

# These boost libraries must be compiled with: "g++ -std=c++14" or "g++ -std=c++17"
# For Various Installation types of Boost Library
BOOST_LIB_DIR := $(wildcard $(BOOST_DIR)/stage/lib $(BOOST_DIR)/lib)

# Specify only the basename of the Boost libraries
BOOST_LIBS := boost_program_options

# Add extra dependency libraries here
EXTRA_LIBS := -lpthread -lm -lz -ldl -static-libstdc++ -lrt

ifdef SOFT_FLOAT
  override CPPFLAGS += -I$(PWD)/third_party/softfloat/source/include
  override CPPFLAGS += -DSOFT_FLOAT
  soft_float_build := $(wildcard $(PWD)/third_party/softfloat/build/RISCV-GCC)
  soft_float_lib := $(soft_float_build)/softfloat.a
endif

PCI := 1
ifdef PCI
  override CPPFLAGS += -I$(PWD)/pci
  pci_build := $(wildcard $(PWD)/pci/)
  pci_lib := $(PWD)/pci/libpci.a
endif

MEM_CALLBACKS := 1
ifeq ($(MEM_CALLBACKS), 1)
  ifdef FAST_SLOPPY
    $(warning "FAST_SLOPPY not compatible with MEM_CALLBACKS, turning off MEM_CALLBACKS")
    MEM_CALLBACKS := 0
  else
    override CPPFLAGS += -DMEM_CALLBACKS
  endif
endif

ifdef HINT_OPS
  override CPPFLAGS += -DHINT_OPS
endif

ifdef FAST_SLOPPY
  override CPPFLAGS += -DFAST_SLOPPY
endif

ifdef LZ4_COMPRESS
  override CPPFLAGS += -DLZ4_COMPRESS
  EXTRA_LIBS += -llz4
endif


# Add External Library location paths here
LINK_DIRS := $(addprefix -L,$(BOOST_LIB_DIR))

# Generating the Linker options for dependent libraries
ifeq ($(STATIC_LINK), 1)
  LINK_LIBS := $(addprefix -l:lib, $(addsuffix .a, $(BOOST_LIBS))) $(EXTRA_LIBS)
else
  COMMA := ,
  LINK_DIRS += $(addprefix -Wl$(COMMA)-rpath=, $(BOOST_LIB_DIR))
  LINK_LIBS := $(addprefix -l, $(BOOST_LIBS)) $(EXTRA_LIBS)
endif

ifeq (Darwin,$(shell uname))
  EXTRA_LIBS := -lpthread -lm -lz -ldl -lc++
  LINK_LIBS := $(BOOST_LIB_DIR)/lib$(BOOST_LIBS).a $(EXTRA_LIBS)
else
  LINK_LIBS += -Wl,-export-dynamic
endif

ifeq (x86_64,$(shell uname -p))
  ARCH_FLAGS := -mfma
else
  ARCH_FLAGS :=
endif

# For out of source build
BUILD_DIR := build-$(shell uname -s)
MKDIR_P ?= mkdir -p
RM := rm -rf
# Optimization flags.  Use -g for debug.
OFLAGS := -O3

# Include paths.
IFLAGS := $(addprefix -isystem ,$(BOOST_INC)) -isystem third_party -I.

# Command to compile .cpp files.
override CXXFLAGS += -MMD -MP $(ARCH_FLAGS) -std=c++20 $(OFLAGS) $(IFLAGS) -fPIC -pedantic -Wall -Wextra -Wformat -Wwrite-strings

# Rule to make a .o from a .cpp file.
$(BUILD_DIR)/%.cpp.o:  %.cpp
	@if [ ! -d "$(dir $@)" ]; then $(MKDIR_P) $(dir $@); fi
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

# Main target.(only linking)
$(BUILD_DIR)/$(PROJECT): $(BUILD_DIR)/whisper.cpp.o \
                         $(BUILD_DIR)/librvcore.a \
			 $(soft_float_lib) \
			 $(pci_lib)
	$(CXX) -o $@ $(OFLAGS) $^ $(LINK_DIRS) $(LINK_LIBS)

$(BUILD_DIR)/$(PY_PROJECT): $(BUILD_DIR)/py-bindings.cpp.o \
			    $(BUILD_DIR)/librvcore.a \
			    $(soft_float_lib) \
			    $(pci_lib)
	$(CXX) -shared -o $@ $(OFLAGS) $^ $(LINK_DIRS) $(LINK_LIBS)

# Rule to make whisper.cpp.o
$(BUILD_DIR)/whisper.cpp.o:  .FORCE
	@if [ ! -d "$(dir $@)" ]; then $(MKDIR_P) $(dir $@); fi
	sha=`git rev-parse --verify HEAD || echo unknown`; \
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -DGIT_SHA="$$sha" -c -o $@ whisper.cpp

# Rule to make PY_PROJECT .so
$(BUILD_DIR)/py-bindings.cpp.o: .FORCE
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(shell python3 -m pybind11 --includes) -c -o $@ py-bindings.cpp

# List of all CPP sources needed for librvcore.a
RVCORE_SRCS := IntRegs.cpp CsRegs.cpp FpRegs.cpp instforms.cpp \
            Memory.cpp Hart.cpp InstEntry.cpp Triggers.cpp \
            PerfRegs.cpp gdb.cpp HartConfig.cpp \
            Server.cpp Interactive.cpp Disassembler.cpp printTrace.cpp \
	    Syscall.cpp PmaManager.cpp DecodedInst.cpp snapshot.cpp \
	    PmpManager.cpp VirtMem.cpp Core.cpp System.cpp Cache.cpp \
	    Tlb.cpp VecRegs.cpp vector.cpp wideint.cpp float.cpp bitmanip.cpp \
	    amo.cpp SparseMem.cpp InstProfile.cpp Isa.cpp Mcm.cpp \
	    crypto.cpp Decoder.cpp Trace.cpp cbo.cpp Uart8250.cpp \
	    Uartsf.cpp hypervisor.cpp vector-crypto.cpp WhisperMessage.cpp \
	    Imsic.cpp PerfModel.cpp Args.cpp

# List of All CPP Sources for the project
SRCS_CXX += $(RVCORE_SRCS) whisper.cpp

# List of All C Sources for the project
SRCS_C :=

# List of all object files for the project
OBJS_GEN := $(SRCS_CXX:%=$(BUILD_DIR)/%.o) $(SRCS_C:%=$(BUILD_DIR)/%.o)

# List of all auto-genreated dependency files.
DEPS_FILES := $(OBJS_GEN:.o=.d)

# Include Generated Dependency files if available.
-include $(DEPS_FILES)

# Object files needed for librvcore.a
OBJS := $(RVCORE_SRCS:%=$(BUILD_DIR)/%.o) $(SRCS_C:%=$(BUILD_DIR)/%.o)

$(BUILD_DIR)/librvcore.a: $(OBJS)
	$(AR) cr $@ $^

$(soft_float_lib):
	$(MAKE) -C $(soft_float_build)

$(pci_lib):
	$(MAKE) -C $(pci_build)

all: $(BUILD_DIR)/$(PROJECT) $(BUILD_DIR)/$(PY_PROJECT)

install: $(BUILD_DIR)/$(PROJECT)
	@if test "." -ef "$(INSTALL_DIR)" -o "" == "$(INSTALL_DIR)" ; \
         then echo "INSTALL_DIR is not set or is same as current dir" ; \
         else echo cp $^ $(INSTALL_DIR); cp $^ $(INSTALL_DIR); \
         fi

install-py: $(BUILD_DIR)/$(PY_PROJECT)
	@if test "." -ef "$(INSTALL_DIR)" -o "" == "$(INSTALL_DIR)" ; \
         then echo "INSTALL_DIR is not set or is same as current dir" ; \
         else echo cp $^ $(INSTALL_DIR); cp $^ $(INSTALL_DIR); \
         fi

clean:
	$(RM) $(BUILD_DIR)/$(PROJECT) $(BUILD_DIR)/$(PY_PROJECT) $(OBJS_GEN) $(BUILD_DIR)/librvcore.a $(DEPS_FILES) ; \
  $(if $(soft_float_build),$(MAKE) -C $(soft_float_build) clean ;,) \
  $(if $(pci_build),$(MAKE) -C $(pci_build) clean,)

help:
	@echo "Possible targets: $(BUILD_DIR)/$(PROJECT) $(BUILD_DIR)/$(PY_PROJECT) all install install-py clean"
	@echo "To compile for debug: make OFLAGS=-g"
	@echo "To install: make INSTALL_DIR=<target> install"
	@echo "To browse source code: make cscope"

cscope:
	( find . \( -name \*.cpp -or -name \*.hpp -or -name \*.c -or -name \*.h \) -print | xargs cscope -b ) && cscope -d && $(RM) cscope.out

.FORCE:

.PHONY: all install install-py clean help cscope .FORCE

