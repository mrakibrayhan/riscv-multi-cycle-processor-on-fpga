# RISC-V single-cycle processor on FPGA (RISC-V-SC)

A RISC-V (RV32I/RV32IM) single-cycle processor implemented in Verilog and tested on the RZ-EasyFPGA A2.2 board (Intel Cyclone IV E EP4CE6E22C8).

> **Status:** complete and tested on hardware. Compiles with Intel Quartus Prime Lite.

## At a glance

| Item | Value |
|---|---|
| ISA | RV32I (board default); optional Zmmul multiply with `ENABLE_ZMMUL = 1` |
| Microarchitecture | single-cycle: Single-cycle: one instruction per clock, CPI = 1. |
| Language | Verilog / SystemVerilog |
| Board | RZ-EasyFPGA A2.2 (Intel Cyclone IV E EP4CE6E22C8, 6,272 LEs, 30 M9K) |
| Tool | Intel Quartus Prime Lite (built with 25.1std) |
| Top-level entity | `top_cpu_riscv` |
| Board clock | 50 MHz input divided by 4 = 12.5 MHz CPU clock |

## Memory system
Separate asynchronous-read instruction and data memories (single-cycle). Board build: 128-word ROM, 32-word RAM. Optional `MEM_M9K` parameter (new file `mem_m9k_riscv_sc.sv`) moves memories into M9K blocks for larger programs; CPI stays 1. To use it, add `mem_m9k_riscv_sc.sv` to the Quartus project (Project > Add/Remove Files in Project) and set `MEM_M9K = 1`; it is not in the default file list.

## Repository contents
- `alu_riscv.v`
- `control_unit_riscv.v`
- `cpu_top_riscv.v`
- `debounce.v`
- `dmem_riscv.v`
- `imem_riscv.v`
- `imm_gen.v`
- `mem_m9k_riscv_sc.sv`
- `regfile_riscv.v`
- `snapshot_unit.v`
- `top_cpu_riscv.v`
- `timing_constraints.sdc`
- `program.hex`
- `data.hex`
- `assembler_riscv.py (optional, small helper)`
- `RISC_V_Single_Cycle_Cpu.qpf`
- `RISC_V_Single_Cycle_Cpu.qsf`
- `RISC_V_Single_Cycle_Cpu.spf`

- `LICENSE`, `.gitignore`, `README.md`

## Build (Quartus)
1. Install Intel Quartus Prime Lite (the Cyclone IV E family is included).
2. Open `RISC_V_Single_Cycle_Cpu.qpf`.
3. Processing > Start Compilation.
4. Programmer: load `output_files/RISC_V_Single_Cycle_Cpu.sof` (not stored in the repository; see Releases for a ready-made `.sof`).

The `.qsf` already contains the device (`EP4CE6E22C8`), the pin assignments and the file list.

## Run on the board
- **KEY[3]** = reset (the CPU also stays in reset until KEY[2] is pressed once). **KEY[2]** = run.
- LEDG (active low = lit): [0] fault, [1] started, [2] a0 (x10) != 0, [3] halted.
- Stop rule: `ecall`, `ebreak`, or `jal x0,0` halts; illegal encoding = fault.
- The program is loaded from `program.hex`, the initial RAM from `data.hex`, both when the FPGA is programmed. Reset does not reload the RAM; program the FPGA again to restore it.
- **Snapshot / debug:** when the program halts, or when the ISSP source bit is set, the CPU is frozen and registers and RAM words are copied into small RAMs. Read them in Quartus with *Tools > In-System Memory Content Editor*; read counters and status with *Tools > In-System Sources and Probes Editor*. CPI = cycle counter / instruction counter.

## Limitations
- Subset ISA, no interrupts, no caches, no MMU, no floating point.
{c['lim']}

## Related repositories
This is one of 9 individual processor projects on FPGA: MIPS-compatible, RISC-V and ARMv4-compatible, each as single-cycle, multi-cycle and pipelined. <!-- add links when ready -->

## License
MIT, see `LICENSE`.

## Trademarks
RISC-V is a trademark of RISC-V International; this project is independent and not affiliated with or endorsed by them.

## Author
Md. Rakib Rayhan, Dept. of ECE, North South University. Research Assistant, Microprocessor, Embedded Systems and IoT Lab (MESIL).
