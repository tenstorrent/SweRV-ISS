Whisper
=================================================================

# Table of Contents
[Introduction](#Introduction)

[Requirements](#Requirements)

[Compiling Whisper](#Compiling)

[Preparing Target Programs](#Preparing)

[Running Whisper](#Running)

[Debugging RISCV Programs Using Gdb and Whisper](#Debugging)

[Configuring Whisper](#Configuring)

[Memory Consistency Checks](#Consistency)

[Enabling Code Coverage](#Coverage)

[Python Support](#Python)

[Limitations](#Limitations)

[Running riscv-arch-test Tests with RISCOF](#RISCOF)


<a name="Introduction"/>

# Introduction

Whisper is a RISCV instruction set simulator (ISS) initially developed for the
verification of the Swerv micro-controller. It allows the user to run RISCV code
without RISCV hardware. It has an interactive mode where the user can single
step the target RISCV code and inspect/modify the RISCV registers or the
simulated system memory. It can also run in lock step with a Verilog simulator
serving as a "golden model" against which an implementation is checked after
each instruction of a test program.

<a name="Requirements"/>

# Requirements

To use Whisper, you would need to download its source code, compile
it, prepare some target test program, compile the test program to
RISCV binary code and then run the RISCV binary within the Whisper
simulator. In particular you would need:

1. A Linux machine to host the RISCV tool-chain and Whisper.

2. The RISCV tool-chain which contains a cross-compiler to compile
   C/C++ code to RISCV binary. This can be installed on most Linux
   distributions using your distros package manager (apt, dnf, pacman
   etc.). Otherwise it can be built from the upstream source code.

   Ubuntu
   ```shell
   $ sudo apt install gcc-riscv64-unknown-elf
   ```

   Arch
   ```shell
   $ sudo pacman -Syu riscv64-elf-gcc
   ```

3. The Whisper source code which can be downloaded from 
   [github.](https://github.com/tenstorrent/SweRV-ISS)

4. The g++ compiler version 11 or higher to compile Whisper. The g++
   compiler can be installed from a Linux distribution. Alternatively,
   the source code can be downloaded from
   [gnu.org/software/gcc.](https://www.gnu.org/software/gcc)

5. The boost library version 1.75 or higher compiled with c++-20.
   Boost source can be downloaded from
   [boost.org.](https://www.boost.org)


<a name="Compiling"/>

# Compiling Whisper

On a Unix system, in the whisper directory, do the following:
```
    make BOOST_DIR=x 
```
where x is the path to your boost library installation.

# Options

There are various Makefile options that can be used.

+ `SOFT_FLOAT=1` to use the soft-float library for RISCV floating point operations.
+ `PCI=1` to build the PCI library.
+ `TRACE_READER=1` to build the trace reader library.
+ `MEM_CALLBACKS=1` to use the sparse memory model.
+ `HINT_OPS=1` to enable HINT ops implemented within whisper.
+ `FAST_SLOPPY=1` to enable faster (but not compliant) execution.
+ `LZ4_COMPRESS=1` to enable loading LZ4 files.

<a name="Preparing"/>

# Preparing Target Programs

Standalone C/assembly programs not requiring operating system support (such programs
cannot do any I/O) should be compiled as follows:
```
    $ riscv32-unknown-elf-gcc -mabi=ilp32 -march=rv32imc -static -O3 -nostdlib -o test1 test1.c
```

The key switch in the above compilation command is "-nostdlib" which
prevents the compiler from linking-in the standard C library.

Note that without the standard C library, there is no "_start"
symbol. The linker will complain that the start symbol is missing and
will use another symbol as the default start address of the
program. The user can always override that start address (program
counter at the beginning of the simulation) by using the --startpc
command line option.

Also note that without an operating system, the simulator does not
know when the program finishes. It will execute instructions
indefinitely. Consider the following test program:
```
    int
    main(int argc, char* argv[])
    {
      int x = 1;
      int y = 2;
      int z = x + y;
      return z;
    }
```

The simulator will start execution at the ELF file entry point
(address corresponding to main) and will return to address 0 (initial
value of return address register) when the instruction corresponding
to "return z" is executed. This will most likely cause an illegal
instruction exception and given that no trap handlers are loaded into
the memory, it will cause an infinite loop of illegal traps. To avoid
this, simple stand-alone no-operating-system programs should define a
global 32-bit integer named "tohost" and should write to that location
at the end of the program. This signals the simulator to terminate the
program.

Here's a modified version of the above program that stops once main is done:
```
    #include <stdint.h>

    volatile uint32_t tohost = 0;
    
    int
    main(int argc, char* argv[])
    {
      int x = 1;
      int y = 2;
      int z = x + y;
      return z;
    }
    
    void _start()
    {
      main(0, 0);
      tohost = 1;
    }
```
And here's how to compile and run the above program
```
    $ riscv32-unknown-elf-gcc -mabi=ilp32 -march=rv32imc -nostdlib -g -o test2 test2.c
    $ whisper test2
```
If no global variable named "tohost" is written by the program, the simulator will stop on
its own if a sequence of 8 consecutive illegal instructions is encountered.

If the above program is compiled for RV64, it will crash with 8 consecutive illegal
instructions. The reason is that the generated code will attempt to push data on the stack
and the default stack pointer value is 0. Pushing on the stack will make the stack pointer
a very large number that exceeds memory size (default is 4GB) which will trigger an access
fault and, without an exception handler, will result in a cascade of illegal instruction
exceptions. To fix that, run the RV64 version of the test2 binary under whisper with
"--setreg sp=0xf0000000" which initializes the stack pointer to an address within the
default memory address range:

```
    $ riscv64-unknown-elf-gcc -mabi=lp64 -march=rv64imc -nostdlib -g -o test2 test2.c
    $ whisper test2   # this will crash
    $ whisper test2  --setreg sp=0xf0000000   # this will run
``` 

For programs requiring minimal operating system support (e.g. brk, open, read and write)
the user can compile with the newlib C library and use the simulator with the "--newlib"
option.

Here's a sample program:
```
    #include <stdio.h>

    int
    main(int argc, char* argv[])
    {
       printf("hello world\n");
       return 0;
    }
```
And here's how to compile and run it (assuming riscv32-unknown-elf-gcc
was compiled with newlib):
```
    $ riscv32-unknown-elf-gcc -mabi=ilp32 -march=rv32imc -static -O3 -o test3 test3.c
    $ whisper --newlib test3
```
Note that in this case the simulator will intercept the exit system
call invoked by the C library code and terminate the program
accordingly. There is no need for the "tohost" mechanism.

<a name="Running"/>

# Running Whisper

Running whisper with -h or --help will print a brief description of all the
command line options. To run a RISCV program, prog, in whisper, one would
issue the Linux command:
```
    whisper prog
```
which will run the program until it writes to the "tohost" location.

A program compiled with the newlib C library need not have a "tohost"
location. Such a program will run until it calls exit. Such a program
would be run as follows:
```
    whisper --newlib prog
```

## Command Line Options

The following is a brief description of the command line options:

    --help      
       Produce help message.

    --log
       Enable tracing to standard output of executed instructions.

    --xlen len
       Specify register width (32 or 64), defaults to 32.

    --isa string
       Select the RISCV extensions to enable. The currently supported options are
       a (atomic), c (compressed instructions), d (double precision fp),
       f (single precision fp), i (base integer), m (multiply divide),
       s (supervisor mode), u (user mode), and v). By default, only i, m and
       c are enabled. Canonical ISA strings with versioned extension are supported.
       Examples: --isa imacf, --isa rv32i2p0_m2p0_f2p0_v1p0_zfh0p1

    --target program
       Specify target program (ELF file) to load into simulated memory. In newlib
       emulations mode, program options may follow program name.

    --hex file
       Hexadecimal file to load into simulator memory.

    --logfile file
       Enable tracing to given file of executed instructions.

    --csv
       Use CSV (comma separated values) format for the trace log file produced
       by the --logfile option. The first output line contains the headers of
       the columns in the rest of the log file. Each executed instruction results
       in a row that includes the PC, opcode, changed registers,
       changed memory locations, ... Fields are separated by commas. Multiple
       values within a field are separated by semicolons.

    --consoleoutfile file
       Redirect console output to given file.

    --commandlog file
       Enable logging of interactive/socket commands to the given file.

    --startpc address
       Set program entry point to the given address (in hex notation with a 0x prefix).
       If not specified, use the ELF file entry point.

    --endpc address
       Set stop program counter to the given address (in hex notation with a 0x 
       prefix). Simulator will stop once instruction at the stop program counter
       is executed. If not specified, use the ELF file _finish symbol.

    --tohost address
       Memory address to which a write stops the simulator (in hex with 0x prefix).

    --consoleio address
       Memory address corresponding to console io (in hex with 0x prefix).
       Reading/writing a byte (using lb/sb instruction) from given address
       reads/writes a byte from the console.

    --maxinst limit
       Limit executed instruction count to given number.

    --interactive
       After loading any target file into memory, the simulator enters interactive
       mode.

    --triggers
       Enable debug triggers (triggers are automatically enabled in interactive and
       server modes).

    --counters
       Enable performance counters.

    --gdb
       Run in gdb mode enabling remote debugging from gdb.

    --profileinst file
       Report executed instruction frequencies to the given file.

    --setreg spec ...
       Initialize registers. Example --setreg x1=4 x2=0xff

    --configfile file
       Configuration file (JSON file defining system features).

    --snapshotdir path
       Directory prefix for saving snapshots: Snapshots (see --sanpshotpreid)
       are placed in sub-directories of the given path. Default: "snapshot".

    --snapshotperiod n
       Snapshot period: Save a snapshot every n instructions putting data in
       directory specified by --snapshotdir.

    --loadfrom path
       Snapshot directory from which to restore a previously saved (snapshot)
       state.

    --newlib
       Emulate limited emulation of newlib system calls. Done automatically 
       if newlib symbols are detected in the target ELF file.

    --linux 
       Emulate limited emulation of Linux system calls. Done automatically 
       if Linux symbols are detected in the target ELF file.

    --raw
       Bare metal mode: Disable emulation of Linux/newlib system call emulation
       even if Linux/newlib symbols detected in the target ELF file.

    --stdout path
       Redirect the standard output of the newlib/Linux target program to the
       file specified by the given path.

    --stderr path
       Redirect the standard error of the newlib/Linux target program to the
       file specified by the given path.

    --alarm period
       External interrupt period in micro-seconds: Convert period to an instruction
       count, n, assuming a 1ghz clock, and set to 1 the timer bit of the MIP
       CSR every n instructions. The timer bit of MIP is automatically cleared if
       the interrupt is actually taken (interrupts enabled in MSTATUS and timer
       bit set in MIE CSR). No-op if n is zero.

    --abinames
       Use ABI register names (e.g. sp instead of x2) in instruction disassembly.

    --verbose
       Produce additional messages.

    --version 
       Print version.


## Interactive Mode

Whisper is started in interactive mode using the "--interactive" command line option.
Here's are some examples:
```
    $ whisper --interactive
    $ whisper --interactive test1
```
In the second example, the program test1 is first loaded into the
simulated memory.  In interactive mode the user can issue commands to
control the execution of the target program and to set/examine the
registers and memory location of the simulated system. The help command
will produce a list of all available interactive commands. The "help x"
command will produce information about command x.

Here's the output of the "help" command:

    The arguments hart=<id> and.or time=<tine> may be used with any command
    to select a hart and specify event time (relevant to memory model)
    They persist until explicitly changed.
    
    help [<command>]
      Print help for given command or for all commands if no command given.
    
    run
      Run till interrupted.
    
    until <address>
      Run until address or interrupted.
    
    step [<n>]
      Execute n instructions (1 if n is missing).
    
    peek <res> <addr>
      Print value of resource res (one of r, f, c, v, m) and address addr.
      For memory (m) up to 2 addresses may be provided to define a range
      of memory locations to be printed; also, an optional file name after
      the two addresses writes the command output to that file.
      examples: peek r x1   peek c mtval   peek m 0x4096
                peek m 0x10 0x40 out
    
    peek pc
      Print value of the program counter.
    
    peek all
      Print value of all non-memory resources
    
    poke res addr value
      Set value of resource res (one of r, c or m) and address addr
      Examples: poke r x1 0xff  poke c 0x4096 0xabcd
    
    disass opcode <code> <code> ...
      Disassemble opcodes. Example: disass opcode 0x3b 0x8082
    
    disass function <name>
      Disassemble function with given name. Example: disas func main
    
    disass <addr1> <addr2>>
      Disassemble memory locations between addr1 and addr2.
    
    elf file
      Load elf file into simulated memory.
    
    hex file
      Load hex file into simulated memory.
    
    replay_file file
      Open command file for replay.
    
    replay n
      Execute the next n commands in the replay file or all the
      remaining commands if n is missing.
    
    replay step n
      Execute consecutive commands from the replay file until n
      step commands are executed or the file is exhausted
    
    reset [<reset_pc>]
      Reset hart.  If reset_pc is given, then change the reset program
      counter to the given reset_pc before resetting the hart.
    
    symbols
      List all the symbols in the loaded ELF file(s).
    
    pagetable
      Print the entries of the address translation table.
    
    nmi [<cause-number>]
      Post a non-maskable interrupt with a given cause number (default 0).
    
    mread tag addr size data i|e
      Perform a memory model (out of order) read for load/amo instruction with
      given tag. Data is the RTL data to be compared with whisper data
      when instruction is later retired. The whisper data is obtained
      forwarding from preceding instructions if 'i' is present; otherwise,
      it is obtained from memory.
    
    mbwrite addr data
      Perform a memory model merge-buffer-write for given address. Given
      data (hexadecimal string) is from a different model (RTL) and is compared
      to whisper data. Addr should be a multiple of cache-line size. If hex
      string is smaller than twice the cache-line size, it will be padded with
      zeros on the most significant side.
    
    mbbypass tag addr size data
      Perform a memory write operation bypassing the merge buffer. Given
      data (hexadecimal string) is from a different model (RTL) and is compared
      to whisper data.
    
    pmp [<address>]
      Print the pmp map (all) or for a matching address
    
    pma [<address>]
      Print the pma map (all) or for a matching address
    
    translate <va> [<permission> [<privilege>]]
      Translate given virtual address <va> to a physical address assuming given
      permission (defaults to read) and privilege mode (defaults to user)
      Allowed permission: r for read, w for write, or x for execute.
      Allowed privilege: u to user or s for supervisor
    
    quit
      Terminate the simulator
        

## Newlib Emulation

Whisper will emulate the newlib open, close, read, write, brk and exit
system calls. This allows simple programs to run and use the newlib C-library
functions such as printf, fopen, fread, fwrite, fclose, malloc,
free and exit. Here an example of running a program with limited
C-library support:
```
    $ whisper --newlib test3
```
And here is an example of passing the command line arguments arg1 and arg2
to the to the target program test3:
```
    $ whisper --newlib "test3 arg1 arg2"
```
And examples of passing command line switches to a target program
that requires them:
```
    $ whisper --newlib "test4 -opt1 val1 -opt2"
    $ whisper --newlib --target "test4 -opt1 val1 -opt2"
```

<a name="Debugging"/>

# Debugging RISCV Programs Using Gdb and Whisper

With the --gdb option, whisper will follow the gdb remote debugging
protocol.  This allows the user to debug a RISCV program using a
cross-compiled gdb and whisper.  For example, to debug a RISCV program
named xyz on a Linux x86 machine, we would start the (cross-compiled)
RISCV gdb as follows: 
```
    $ riscv-unknown-elf-gdb xyz
```
at the gdb prompt, we would connect to whisper by issuing a "target remote"
gdb command as follows:
```
    target remote | whisper --gdb xyz
```

<a name="Configuring"/>

# Configuring Whisper

A JSON configuration file may be specified on the command line using the
--configfile switch. Numeric parameters may be specified as integers or
as strings. For example, a core count of 4 may be specified as:
```
  "cores" : 4
```
or
```
  "cores" : "4"
```
If expressed as a string, a numeric value may be prefixed with 0x to
specify hexadecimal notation (JSON does not support hexadecimal notation
for integers).

The value of a Boolean parameters may be specified as an integer with 0
indicating false and non-zero indicating true. Alternatively it may
be specified using the strings "false", "False", "true", or "True".

Command line options override settings in the configuration file.

C++ style comments are ignored when the file is parsed.

Here is a sample configuration file:
```
    {
        "isa" : "rv32imafd_zfh_zba_zbb_zbc_sbs",
        "abi_names" : "true",
    
        "csr" : {
            "misa" : {
                "reset-comment" : "imabfv",
                "reset" : "0x40201123",
                "mask-comment" : "Misa is not writable by CSR instructions",
                "mask" : "0x0"
             },
             "mstatus" : {
                "mstatus-comment" : "Hardwired to zero except for FS, VS, and SD.",
                "reset" : "0x80006600",
                "mask" : "0x0",
                "poke_mask" : "0x0"
             }
        }
    }
```

A schema for the JSON config file is located in the configuration folder.
It can be used for code completion and validation by adding the following to a config file:
```
"$schema": "<path to this repository>/configuration/config_schema.json",
```

## Configuration parameters

### cores
Number of cores in simulated system.

### xlen
Integer register size in bits.

### isa
Enable instruction set architecture (isa) features.
Example:
```
   "isa" : "rv32imaf"
```

### memmap
Object defining memory organization. Fields of memmap:
* size: Field defining physical memory size
* page_size: Field defining page size
* pma: Array of entries defining physical memory attributes.
Each entry is an object with a "low" and "high" addresses and an
"attribs" array defining the physical memory attributes.

Example:
```
    "memmap" : { "size" : "0x100000000", "page_size" : 4096,
        "pma" : [
            {
                "low" : "0x80000000",  "high" : "0x801fffff",
                "attribs" : [ "read", "write", "exec", "amo", "rsrv", "idempotent" ]
            },
            {
                "low" : "0x0",  "high" : "0xffffffff",
                "attribs" : [ "read", "write", "amo", "rsrv", "idempotent" ]
            }
        ]
    }
```
       
### num_mmode_perf_regs
Number of implemented performance counters. If specified number is n,
then CSRs (counters) mhpmcounter3 to mhpmcounter3+n-1 are implemented
and the remaining counters are hardwired to zero. Same for the
mhpmevent CSRs.

###  enable_performance_counters
Whisper will count events associated with performance counters when
this is set to true. Note that pipeline specific events (such as
mis-predicted branches) are not supported. Synchronous events (such as
count retired load instructions) are supported.

###  abi_names
If set to true then registers are identified by their ABI names in the
log file (e.g. ra instead of x1).

###  trace_ptw
If set to true then page table walk information is emitted to the log file.

###  reservation_bytes
Defines the size of a lr.w/lr.d reservation (default is 4 for RV32 and 8 for RV64).

###  enable_misaligned_data
If set to false then a misaligned data access by a load/store
instructions will trigger an exception.

###  misaligned_has_priority
When true, makes misaligned data exceptions have priority over page and access
fault exceptions. Default is true.

###  page_fault_on_first_access
When true, makes first access to a page table entry trigger a page fault.
Default is true.

###  tlb_entries
Defines the number of translation look-aside buffer entries. Default is 32.

### clear_mprv_on_ret
When true (default), makes the mret/sret instruction clear the mprv bit in
the mstatus/status CSR.

### clear_mtval_on_illegal_instruction
When true, causes the illegal instruction exception to clear the mtval CSR.
Default is false.

### cancel_lr_on_trap
When true (default), causes reservations to be canceled on traps.

### debug_park_loop
Defines the address of the entry point of the debug mode park
look. Whisper will jump to this address upon entering debug mode if
this address is not an all ones bit pattern. Default: all ones bit pattern.

### debug_trap_address
Defines the address of the debug mode exception handler.  Whisper will
jump to this address upon encountering an exception in debug mode if
this address is not an all ones bit pattern. Default: all ones bit
pattern.

###  physical_memory_protection_grain
Defines the G value of the physical memory protection grain. This is
the log base 2 of the grain minus 2.  The default is G=0 (implying a
grain of 4).

###  guest_interrupt_count
Defines the maximum number of guest external interrupt count (GEILEN).
Default is zero.


###  csr
The CSR configuration is a map where each key is a CSR name and the
corresponding value is an object with the following fields: "number",
"reset", "mask", "poke_mask", "exists", and "shared". Set "exists" to
"false" to mark a non implemented CSR (read/write instructions to such a
CSR will trigger illegal instruction exception). Set "mask" to the
write mask of the CSR (zero bits correspond to bits that will be
preserved by write instructions). Set "reset" to reset value of the
CSR. Set "shared" to "true" for CSRs that are shared between harts. The
"number" fields should be used to define the number (address) of a
non-standard CSR. The poke_mask should be used for the rare cases
where poke operation may modify some bits that are not modifiable by
CSR write instructions.

For CSRs with shared base name and different integer suffixes
(e.g. pmpaddr0, pmpaddr1, ...), the configurations may be defined
for each CSR or a common configuration may be defined
with a range attribute. Example:
```
     "csr" : {
         "pmpaddr0" : { "mask" : "0xffffffff" },
         "pmpaddr" : { "exists" : "false",  "range" : [1 , 63] }
      }
```

###  vector
The vector configuration is an object with the following fields:
* bytes_per_vec: vector size in bytes.
* min_bytes_per_elem: narrowest supported element size in bytes (default 1).
* max_bytes_per_elem: widest supported element size in bytes (no default).
* min_bytes_per_lmul: map of lmul to min-element-width-in bytes (default: no min).
* min_bytes_per_lmul: map of lmul to max-element-width-in bytes (default: no max).
* mask_agnostic_policy: "ones" or "undisturb" to set behavior of mask-anostic instructions, default is "ones".
* tail_agnostic_policy: "ones" or "undisturb" to set behavior of tail-anostic instructions, default is "ones".
* trap_non_zero_vstart: causes vector instruction to trap on non-zero vstart, default is true.
* update_whole_mask: when true, compute all the elements of the destination mask register for mask-logical and mask-manipulation instructions regardless of VL.
* trap_invalid_vtype: when true, trap on invalid/unsupported vtype configurations, when false set vtype.vill instead.
* legalize_vsetvl_avl: when true, legalize VL to VLMAX if it would be greater than VLMAX after a vsetvl instruction.
* legalize_vsetvli_avl: when true, legalize VL to VLMAX if it would be greater than VLMAX after a vsetvli instruction.
* tt_fp_usum_tree_reduction: for each EEW, enables Tenstorrent tree reduction-style vfredusum/vfwredusum, default is false.
* fp_usum_nan_canonicalize: for each EEW, enables NaN canonicalization of vfredusum/vfwredusum result, default is false.
* partial_segment_update: partially commit the fields of a load/store segment encountering an exception/trigger-hit at a given index when true and commit no field in the case of an exception when false, default is false.

Example:
```
    "vector" : {
       "bytes_per_vec" : 16,
       "max_bytes_per_elem" : 8,
       "tail_agnositic_policy" : "undisturb",
       "mask_agnositic_policy" : "ones",
       "min_bytes_per_lmul" : { "m2" : 2, "m4" : 2 },
       "max_bytes_per_lmul" : { "mf8" : 4 },
       "tt_fp_usum_tree_reduction" : [ "e16", "e32", "e64" ]
    }
```

###  aclint
The advanced core local interrupt controller (aclint) configuration is an object with the following fields:
* base: base address of the memory area associated with the ACLINT.
* sw_offset: offset to software interrupt region within the ACLINT area).
* timer_offset: offset to timer within the ACLINT area.
* time_offset: offset to time-compare region within the ACLINT area).
* software_interrupt_on_reset: when set to true, write to software interrupt of core 0 on reset.
* deliver_interrupts: when set to true, deliver ACLNT interrupts. This supports the
  test-bench which may decode to deliver ACLINT interrupts by poking the MIP CSR, in which
  case deliver_interrupts should be set to false.


###  reset_vec
Defines the program counter (PC) value after reset. The ELF file
entry point will supersede the reset_vec value unless --raw is
used. The value of the --startpc option will supersede both the
reset_vec and the ELF file entry point. In interactive mode, a reset
command will change the program counter to the value of reset_vec.


###  nmi_vec
Defines the address of the handler of non-maskable interrupts.

###  nmi_exception_vec
Defines the address of the handler of exceptions encountered while
handling non-maskable interrupts.

### indexed_nmi

When false, the PC after an NMI will be base value defined by nmi_vec.
When true, the PC will be the base plus 4 times the NMI cause. Default
value is false.

Similarly, when false, then after an exception while in the NMI interrupt
handler, the PC will be the base value defined by nmi_exception_vec.
When true, the PC will be the base plus 4 times the exception cause.


###  enable_triggers
Enable support for debug triggers when set to true.

### trigger_use_tcontrol
When set to true, the MTE field of the TCONTROL CSR controls the firing of triggers in
machine mode. When set to false, the triggers fire in machine mode only if MSTATUS.MIE
is zero.

###  perf_count_atomic_load_store
When true, the lr/sc instructions will be counted as load/store 
by the performance counters.

### trigger registers
Each trigger register is associated with up to 4 components tdata1, tdata2, tdata3, and tinfo. Here's
an example of how to configure the reset values and masks of these components in a system
with 2 trigger registers (the mask and reset values are made up):
```
     "triggers" : [
         {
	    "reset"    : [0, 0, 0, "0x1008040"],
	    "mask"     : ["0xffffffff", "0xffffffff", "0xffffffff", 0],
	    "poke_mask": ["0xffffffff", "0xffffffff", "0xffffffff", 0]
         },
         {
	    "reset"    : [0, 0, 0, "0x1008040"],
	    "mask"     : ["0xffffffff", "0xffffffff", "0xffffffff", 0],
	    "poke_mask": ["0xffffffff", "0xffffffff", "0xffffffff", 0]
         }
     ],
```

### all_ld_st_addr_trigger
Enable matching on all possible addresses in a load/store access [address, address+size-1].

### all_inst_addr_trigger
Enable matching on all possible addresses in a instruction fetch access [address, address+size-1].

### trigger_types
Define the supported trigger types (type field in tdata1). Example:
```
     "trigger_types" : [ "none", "disabled", "mcontrol6" ]
```
The types "none" and "disabled" must not be excluded from "trigger_types". Possible values that
can be included with in "trigger_types" are:
```
   "none", "mcontrol", "icount", "itrigger", "etriger", "mcontrol6",
   "tmexttriger", and "disabled"
```

### trigger_napot_maskmax
Define the number of maximum bits that the NAPOT mask can support. The maximum
possible value of this number is 63 for an RV64 configuration.

### perf_count_fp_load_store
When true, the floating point load/store instructions will be counted
as load/store by the performance counters.

### stee

The static trusted execution environment (STEE) configuration is an object with the
following fields:
* zero_mask: if bit i is set in the zero_mask value, then bit i must be zero in every
load/store address; otherwise, the address will be invalid and will result in an
access-fault exception.
* secure_mask: if bit i is set in the secure_mask value, then bit i must be one in
a load/sore address in order for that address to be considered secure.
* secure_region: insecure access to this region has no effect, loads will return zero, and stores are ignored. The region bounds should not have the secure bits set. The secure bits of an address are cleared before checking against the secure region.

Example:
```
    "stee" : {
      "zero_mask" :     "0xff70000000000000",
      "secure_mask":    "0x0080000000000000",
      "secure_region": ["0x0001000000000000", "0x0002000000000000"]
    },

```

<a name="Consistency"/>

# Memory Consistency Checks

When run in server or interactive modes, Whisper will check the RISCV
weak memory ordering rules also known as preserved program order (PPO)
rules. This feature is enabled by setting the
"enable_memory_consistency" to "true" in the configuration file or by
using "--mcm" on the command line. Detailed information about the PPO
rules can be found in chapter 17 of the the [RISCV unprivileged specs.](https://github.com/riscv/riscv-isa-manual/releases/download/draft-20221206-b7080e0/riscv-spec.pdf)

By default, we check the ordering rules of the weak memory ordering
model (RVWMO). If the enable_tso configuration tag is set to true, we
check the re-ordering rules of the total-store-order memory model.

Whisper expects to be notified about read and write operations and it
expects such operations to be associated with time stamps. Each memory
instruction (load/store/amo) is associated with one or more memory
read/write operation. A memory operation may occur before/after the
corresponding instruction is retired. Memory operations from two
different instructions may occur in a global memory order that is
different than the program order of those instructions.

A read operation has an instruction tag, an address, a size, a data
value, and an indication of whether or not the data was forwarded from
inside the core (internal) or it came from the memory system
(external). The interactive command for a read operation has the form:
```
   time=<time-stamp> hart=<id>  mread <tag> <addr> <size> <data> <ie>
```
A merge buffer insert has an instruction tag, an address, a size, and
data value. The operation signifies a transfer of the data to the
store buffer. The interactive command for a merge buffer insert has
the form:
```
   time=<time-stamp> hart=<id> mbinsert <tag> <addr> <size> <data>
```
A merge buffer write implies the transfer of data from the merge
buffer to the external memory. This is when the write operations
accumulated in the merge buffer become visible to the global memory
system. The interactive command for a merge buffer write is:
```
   time=<time-stamp> hart=<id> mbwrite <addr> <data>
```
Similarly, we provide server mode commands that allows a client
(typically test bench code running in a Verilog simulator) to provide
whisper information about the time, hart-id, size, instruction tag and
data of read/write operations associated with load/store/amo
instructions. We use such information to check the preserved program
order (ppo) rules of RISCV.

<a name="Coverage" />

# Enabling Code Coverage

C++ code coverage for Whisper can be configured and collected on Linux with minimal effort using [gcov](https://gcc.gnu.org/onlinedocs/gcc/Gcov-Intro.html) and [lcov](https://github.com/linux-test-project/lcov).

To enable and collect C++ code coverage:
1. Pass the `--coverage` flag to the compiler and linker.  Note: coverage should be enabled with optimization disabled to get the most accurate line information.
    - When compiling using the Makefile, invoke `make` with `OFLAGS="-g -O0 --coverage"`.  If done successfully, a .gcno file should exist in the build folder beside the .d and .o file for each source file included in the build.
    - When compile using bazel, invoke `bazel build` with `--compilation_mode dbg --collect_code_coverage --instrumentation_filter=rvcore,whisper`.  If done successfully, a .gcno file should exist in the bazel-bin/_objs folder (or sub-folder for the target) beside the .d and .o file for each source file included in the build.
2. Run a test or set of tests that invokes the whisper executable created in step 1.  Coverage information will be aggregated across invocations into a .gcda file for each source file in the build.  These .gcda files should be created beside the .gcno file but may be put in a different place depending on the working directory.  Running tests from the repository root should prevent this issue, but some testing tools (e.g. riscof) run tests in their own working directory.  The output directory of these .gcda files can be controlled using the `GCOV_PREFIX` and `GCOV_PREFIX_STRIP` environment variables, so set these values if necessary so that the .gcda file is created next to the .gcno file (for example, running riscof with a local bazel build likely requires `GCOV_PREFIX=[Whisper repo root absolute path]` and `GCOV_PREFIX_STRIP=3`).
3. Use `lcov` to aggregate the coverage information from the .gcda files into a single coverage report.  Invoke `lcov` with `--output-file [report name; for example, _coverage_report.dat] --directory [path to directory containing .gcno and .gcda files] --base-directory [Whisper repo root absolute path] --capture --no-external`.  If done correctly, a file with the name passed to `lcov` in the `--output-file` parameter will be created and should not be empty.
4. (optional) Generate an html report using `genhtml`.  `genhtml` should be invoked using `-o [collateral output directory] [coverage report file from step 3]`.  The index.html page in the output directory can then be opened using a web browser.

Note that for any test defined using bazel and run with `bazel test`, steps 2 and 3 will be handled automatically by bazel, and the final coverage report should exist inside bazel-out/_coverage.

<a name="Python" />

# Python Support

There is basic support for python bindings with [pybind11](https://pybind11.readthedocs.io/en/stable/) and its corresponding [Bazel support](https://github.com/pybind/pybind11_bazel). The shared library can be built with `make all`.

Example:
```
    import whisper

    s = whisper.system.System64("config.json")
    h = s.harts()[0]
    h.step()
    print(h.x1)
```

The hart registers are exposed as class attributes and implement step/run functionality.

<a name="Limitations"/>

# Limitations

It is not possible to change XLEN at run time by writing to the MISA
register.

The "round to nearest break tie to max magnitude" rounding mode is not
implemented unless you compile with the softfloat library:
```
   make SOFT_FLOAT=1
```

in which case simulation of floating point instructions slows down
significantly.

Suppprted extensions: A, B, C, D, F, H, I, M, S, U, V, ZFH, ZFHMIN, ZBA, ZBB,
ZBS, ZKND, ZKNE, ZKNH, ZBKB, ZKSED, ZKSH, SVINVAL, SVNAPOT, ZICBOM, ZICBOZ,
ZWARS, ZMMUL, ZVFH, ZVFH, ZVFHMIN, ZVBB, ZVBC, ZVKG, ZVKNED, ZVKNHA, ZVKNHB,
ZVKSED, ZVKSH, ZICOND, ZCB, ZFA, ZFBFMIN, ZVFBFMIN, ZVFBFWMA, SSTC, SVPBMT,
SMAIA, SSAIA, ZACAS.

<a name="RISCOF"/>

# Running riscv-arch-test Tests with RISCOF

[riscv-arch-test](https://github.com/riscv-non-isa/riscv-arch-test) is a repository containing RISC-V compliance tests, and [RISCOF](https://github.com/riscv-software-src/riscof) is a tool that simplifies building and running these tests against a known reference model (Sail and/or Spike).

Whisper includes the functionality necessary to run these tests and a plugin used to run and score the tests with RISCOF.  To run a test or set of tests with RISCOF:
1. Install RISCOF via pip.  For more information, see the [RISCOF docs](https://riscof.readthedocs.io/en/stable/installation.html).
2. Clone the [riscv-arch-test](https://github.com/riscv-non-isa/riscv-arch-test) repository.  Note that this can also be achieved using `riscof arch-test --clone` (riscof provides functionality to specify the clone directory and to update an existing checkout; use `riscof arch-test --help` for more info).
3. Create RISCOF's config.ini file by running `riscof setup --dutname whisper`.  It defaults to using Sail as the reference model; append `--refname spike` to the command to use Spike.
4. Update the `DUTPluginPath` in the `RISCOF` section in the config.ini file to the arch_test_target folder from this repository.  Likewise, set the `pluginpath`, `ispec`, and `pspec` paths to the appropriate locations within the arch_test_target folder.  Note that whisper_isa32.yaml is to be used when running an RV32 architecture; whisper_isa.yaml is for RV64.  RISCOF does not appear to have the ability to configure both architectures in a single file and dynamically switch based on the test.
5. (Optional) set the `jobs` field in the `whisper` and \<Ref> sections to a number larger than 1 to allow running tests in parallel.
6. Update sail_cSim/riscof_sail_cSim.py and/or spike/riscof_spike.py as necessary based on desired usage.  Some modifications may include:
   - Replace the dynamic switching of 32 vs 64 based on ISA when running gcc and objdump to just 64 if your toolchain is compiled for multilib.
   - Disable logging to file and creating dis-assembly files.  Some tests (particularly some floating point tests) are very large, so generating dis-assembly and log files for these tests is very time consuming and can consume large amounts of space.  These files are unused for scoring, so they can safely be disabled if just scoring tests.
   - Ensure extensions for all desired tests are included in the architecture string passed to the compile command and/or executable invocations.
7. Build Whisper and the reference model simulator.  See the Sail or Spike documentation on how to do so.
8. Ensure the paths to the RISC-V toolchain (i.e. gcc and objdump), the reference model executable, and whisper executable are in the `PATH` environment variable.  All need to be able to be invoked without a path.
9. Run the desired test suite using `riscof run`.  The `--suite` parameter should be provided with the riscv-arch-test/riscv-test-suite directory (or a sub-directory) from the clone from step 2 above, and the `--env` folder should be provided with the riscv-arch-test/riscv-test-suite/env folder.
   - By default, the run command will produce an HTML report containing information about which tests passed and failed and will attempt to open this report in the browser once all tests have completed.  If this behavior is undesirable (e.g. running on a headless node or as part of CI), provide the `--no-browser` argument.
