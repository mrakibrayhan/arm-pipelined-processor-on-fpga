# ARMv4-compatible 5-stage pipelined processor on FPGA (ARM-P)

An ARMv4-compatible (ARM-state subset) 5-stage pipelined processor implemented in Verilog and tested on the RZ-EasyFPGA A2.2 board (Intel Cyclone IV E EP4CE6E22C8).

> **Status:** complete and tested on hardware. Compiles with Intel Quartus Prime Lite.

## At a glance

| Item | Value |
|---|---|
| ISA | ARMv4-compatible (ARM-state subset: data processing with barrel shifter, LDR/STR, LDM/STM, B/BL/BX, MUL/MLA, long multiply) |
| Microarchitecture | 5-stage pipelined: IF, ID, EX, MEM, WB with hazard unit, forwarding and stalls. |
| Language | Verilog / SystemVerilog |
| Board | RZ-EasyFPGA A2.2 (Intel Cyclone IV E EP4CE6E22C8, 6,272 LEs, 30 M9K) |
| Tool | Intel Quartus Prime Lite (built with 25.1std) |
| Top-level entity | `top_cpu_arm_p` |
| Board clock | 50 MHz input divided by 4 = 12.5 MHz CPU clock |

## Memory system
Synchronous-read M9K memories: dual-port ROM (1 M9K), byte-enable RAM (1 M9K), register file (3 M9K copies).

## Repository contents
- `alu_arm.v`
- `condlogic_arm_p.v`
- `controller_arm_p.v`
- `cpu_core_arm_p.v`
- `datapath_arm_p.v`
- `debounce.v`
- `decoder_arm.v`
- `ext_arm.v`
- `hazard_arm_p.v`
- `ldmstm_seq_arm_p.v`
- `mem_arm_p.sv`
- `mul_arm_p.v`
- `regfile_arm_p.v`
- `seg7_decoder.v`
- `seg7_display.v`
- `shifter_arm.v`
- `snapshot_arm_p.v`
- `top_cpu_arm_p.v`
- `timing_constraints.sdc`
- `program.hex`
- `data.hex`
- `ARM_Pipelined_Cpu.qpf`
- `ARM_Pipelined_Cpu.qsf`
- `ARM_Pipelined_Cpu.spf`
- `assembler_arm.py`

- `LICENSE`, `.gitignore`, `README.md`

## Build (Quartus)
1. Install Intel Quartus Prime Lite (the Cyclone IV E family is included).
2. Open `ARM_Pipelined_Cpu.qpf`.
3. Processing > Start Compilation.
4. Programmer: load `output_files/ARM_Pipelined_Cpu.sof` (not stored in the repository; see Releases for a ready-made `.sof`).

The `.qsf` already contains the device (`EP4CE6E22C8`), the pin assignments and the file list.

## Run on the board
- **KEY[3]** = reset (the CPU also stays in reset until KEY[2] is pressed once). **KEY[2]** = run.
- LEDG (active low = lit): [0] fault, [1] started, [2] Z flag, [3] halted.
- Stop rule: `SWI`, or `B .` with condition true halts; illegal encoding or BX to an odd address = fault.
- The program is loaded from `program.hex`, the initial RAM from `data.hex`, both when the FPGA is programmed. Reset does not reload the RAM; program the FPGA again to restore it.
- **Snapshot / debug:** when the program halts, or when the ISSP source bit is set, the CPU is frozen and registers and RAM words are copied into small RAMs. Read them in Quartus with *Tools > In-System Memory Content Editor*; read counters and status with *Tools > In-System Sources and Probes Editor*. CPI = cycle counter / instruction counter.

## Writing your own program
`assembler_arm.py` (Python 3, no extra packages) turns an assembly file into the `program.hex` that the CPU loads when the FPGA is programmed:
```
python assembler_arm.py prog.s -o program.hex --depth 128
```
Copy the new `program.hex` (and `data.hex` if the assembler wrote one) into the Quartus project folder and compile again. Run `python assembler_arm.py --help` for all options. The core stops on `SWI`, or on `B .` (branch to itself).

## Limitations
- Subset ISA, no interrupts, no caches, no MMU, no floating point.
{c['lim']}

## Related repositories
This is one of 9 individual processor projects on FPGA: MIPS-compatible, RISC-V and ARMv4-compatible, each as single-cycle, multi-cycle and pipelined. <!-- add links when ready -->

## License
MIT, see `LICENSE`.

## Trademarks
ARM is a trademark of Arm Limited; this project is an independent, educational design that is *compatible with a subset of the ARMv4 ARM-state instruction set*. It is not an Arm-designed core and is not affiliated with or endorsed by Arm Limited.

## Author
Md. Rakib Rayhan, Dept. of ECE, North South University. Research Assistant, Microprocessor, Embedded Systems and IoT Lab (MESIL).
