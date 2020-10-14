# minrv32
## Minimal RISC-V Implementation without a register file

This is my take on a small RISC-V implementation....

[orconf 2019 lightning talk introducing this](https://www.youtube.com/watch?v=zy0Y6kQW8Vw&list=PLUg3wIOWD8yodkHgXWGSHQdKACu9MWepT&index=27)

It is currently just a combinatorial RISC-V ISA decode, that makes some pretty unrealistic assumptions on the supporting hardware - such as instruction and data memory (multi-ported, combinatorial read paths) and the register file.  This is very much a work in progress towards having an implementation that does not require a register file and uses a single memory for program, data **and register** contents.

# Goal - some interesting microarchitectures
## and the design methodology to get there

The internet is full of simple RISC-V implementations.  Many CS students that graduate having done a class in computer architecture will have implemented a RISC-V core or subsystem.  This work attempts to do something slightly different and takes inspiration from the processors of yesterday...  This is not necessarily advantageous given current implemenation technologies, where gates and registers are cheap and the design goal is typically to maximize how much can be done in parallel using additional harwdare, rather than minimizing the amount of hardware and tolerating slower multi-cycle excecution.  

In particular, I want to memory-map the RISC-V register file.  This is inspired by the 6502 https://en.wikipedia.org/wiki/MOS_Technology_6502   "In order to make up somewhat for the lack of registers, the 6502 included a zero-page addressing mode that uses one address byte in the instruction instead of the two needed to address the full 64 KB of memory. This provides fast access to the first 256 bytes of RAM by using shorter instructions. Chuck Peddle has said in interviews that the specific intention was to allow these first 256 bytes of RAM to be used like registers". 

See also: https://en.wikipedia.org/wiki/Zero_page

It is not necessary that the "zero-page" be mapped to address RISC-V address `0x0000_0000`.  In fact it will be preferable not to do this. Also, it will be preferable to make normal RISC-V load/stores unable to access the memory region where RISC-V ISA register values are stored (a processor trap or exception would be preferable to the debugging experience of a program over-writing register values through memory writes).

This will require additional memory fetches into temporary registers for the instruction arguments.  A small number of cache registers will be implemented, mininum being two, one for each of the `rs1` and `rs2` arguments.  Additional state will be required to record status such as to which RISC-V ISA registers are currently cached (a five bit field) and whether it is dirty (register has been written, but hasn't yet been written to memory).

## An Alternate View of the RISC-V ISA

Drawing inspiration from the 6502's zero page memory instructions, the RISC-V can be considered to have zero-page  instructions that directly access zero-page memory.  The only ISA state is the program counter!  An instruction such as 'add' becomes a CISC instruction that does two zero-page memory loads, executes the ALU operation for ADD and writes the result back to zero-page memory. Load and store instructions becomes memory contents move between memory and zero-page memory.  Assuming a single ported memory, instructions will necessarily take multiple cycles to execute in order to perform the memory accesses sequentially.

It is possible to optimize some of the memory accesses to remove some redundant operations. If the result of an instruction is preserved in a tempory (unnamed) register and not immediately written to memory, then a subsequent instruction may be able to directly use the stored value, saving a further memory read. It is expected that many instruction sequences have registers that are written in one cycle that are used by the next instruction. If multiple instructions use the same source RISC-V register, then the register value will already available as a side effect of the previous instructions and the load can eliminated.  The unnamed registers effectively form a cache for the RISC-V architectural registers stored in zero-page memory.  It is necessary to maintain knowledge of which RISC-V architectural registers are currently cached, and which contain newer values than the corresponding zero-page memory location.

## Context Switches and Interrupts

By having multiple banks of registers mapped in memory (page-zero, page-one, page-two etc), it would be possible to switch between sets of registers very quickly.  Interrupts will be disportionately fast, since there is very little state to be saved in a context switch.

# Current Implementation and status

Zero-page etc not implemented.

Currently only rv32i.  Compressed instuctions will follow. 

Adapted from Clifford Wolf's picrorc32 :  https://github.com/cliffordwolf/picorv32

ALU optimizations such as replacing the barrel-shifter with a sequential alternative and taking care over resource reuse (to minimize the number of adders etc instantiated) are largely independent of the work towards removing the register-file and are not currently a focus.

The next stage in the implementation process is to add handshaking to each of the memory ports (instruction read, data read, data write, register `rs1` read, register `rs2` read, register `rd` write).  This will then allow an arbiter to be used to make the memory access sequential through a single port.

## Adding source files
Each new .v file added, needs to be added to 

VERILOG_SRCS in [dhrystone/makefile](dhrystone/makefile)

Under [script-sources] in [minrv32-formal/checks.cfg](minrv32-formal/checks.cfg)

Add a read_verilog under [script] and the path under [files] in each of

  [minrv32-formal/complete.sby](minrv32-formal/complete.sby)
  
  [minrv32-formal/cover.sby](minrv32-formal/cover.sby)
  


## minrv32-formal Formal Checking

This requires that riscv-formal (https://github.com/SymbioticEDA/riscv-formal) be installed in a sibling directory (beside the minrv32 directory, at the same heirarchical level).  The stuff in minrv32-formal is similar to the corresponding stuff in picrorv32 directory in riscv-formal: https://github.com/SymbioticEDA/riscv-formal/tree/master/cores/picorv32.  I have added a makefile.  All the instruction checks pass (`make checks`), but some of the other ones don't yet - why TBD.

## firmware

This is pretty much a direct clone of https://github.com/cliffordwolf/picorv32/tree/master/firmware
I expect that I will modify this slightly to set the stack pointer in the startup code, rather than having the hardware do this.

## dhrystone 

This is pretty much a direct clone of https://github.com/cliffordwolf/picorv32/tree/master/dhrystone.
For the current "combinatorial" RISC-V implementation, the benchmark returns a 0.999 (i.e. 1) instructions per cycle - as expected.
Set USE_MYSTDLIB = 1 in Makefile to the startup code in start.S that sets the stack pointer.



