# RISC-V multi-cycle processor on FPGA (RISC-V-MC)

A RISC-V (RV32I/RV32IM) multi-cycle processor implemented in Verilog and tested on the RZ-EasyFPGA A2.2 board (Intel Cyclone IV E EP4CE6E22C8).

> **Status:** complete and tested on hardware. Compiles with Intel Quartus Prime Lite.

## At a glance

| Item | Value |
|---|---|
| ISA | RV32IM by default; parameter `ENABLE_M`: 1 = RV32IM, 2 = RV32I + Zmmul (multiply only), 0 = RV32I |
| Microarchitecture | multi-cycle: Multi-cycle FSM: one instruction takes several clock cycles. |
| Language | Verilog / SystemVerilog |
| Board | RZ-EasyFPGA A2.2 (Intel Cyclone IV E EP4CE6E22C8, 6,272 LEs, 30 M9K) |
| Tool | Intel Quartus Prime Lite (built with 25.1std) |
| Top-level entity | `top_cpu_riscv` |
| Board clock | 50 MHz input divided by 4 = 12.5 MHz CPU clock |

## Memory system
Synchronous-read M9K memories: ROM (128 words, 1 M9K), byte-enable RAM (32 words, 1 M9K), register file (2 M9K copies).

## Repository contents
- `alu_riscv.v`
- `alu_control_riscv.v`
- `control_fsm_riscv.v`
- `cpu_top_riscv.v`
- `datapath_regs_riscv.v`
- `debounce.v`
- `dmem_riscv_mc.sv`
- `imem_riscv.v`
- `imm_gen.v`
- `muldiv_riscv.v`
- `regfile_riscv.v`
- `snapshot_unit.v`
- `top_cpu_riscv.v`
- `timing_constraints.sdc`
- `program.hex`
- `data.hex`
- `assembler_riscv.py`
- `RISC_V_Multi_Cycle_Cpu.qpf`
- `RISC_V_Multi_Cycle_Cpu.qsf`
- `RISC_V_Multi_Cycle_Cpu.spf`

- `LICENSE`, `.gitignore`, `README.md`

## Build (Quartus)
1. Install Intel Quartus Prime Lite (the Cyclone IV E family is included).
2. Open `RISC_V_Multi_Cycle_Cpu.qpf`.
3. Processing > Start Compilation.
4. Programmer: load `output_files/RISC_V_Multi_Cycle_Cpu.sof` (not stored in the repository; see Releases for a ready-made `.sof`).

The `.qsf` already contains the device (`EP4CE6E22C8`), the pin assignments and the file list.

## Run on the board
- **KEY[3]** = reset (the CPU also stays in reset until KEY[2] is pressed once). **KEY[2]** = run.
- LEDG (active low = lit): [0] fault, [1] started, [2] last branch taken, [3] halted.
- Stop rule: `ecall`, `ebreak`, or a branch/`jal` to itself halts; illegal encoding = fault.
- The program is loaded from `program.hex`, the initial RAM from `data.hex`, both when the FPGA is programmed. Reset does not reload the RAM; program the FPGA again to restore it.
- **Snapshot / debug:** when the program halts, or when the ISSP source bit is set, the CPU is frozen and registers and RAM words are copied into small RAMs. Read them in Quartus with *Tools > In-System Memory Content Editor*; read counters and status with *Tools > In-System Sources and Probes Editor*. CPI = cycle counter / instruction counter.

## Writing your own program
`assembler_riscv.py` (Python 3, no extra packages) turns an assembly file into the `program.hex` that the CPU loads when the FPGA is programmed:
```
python assembler_riscv.py prog.s --isa rv32im --outdir .
```
Copy the new `program.hex` (and `data.hex` if the assembler wrote one) into the Quartus project folder and compile again. Run `python assembler_riscv.py --help` for all options. The core stops on `ecall`, `ebreak`, or a jump to itself.

## Limitations
- Subset ISA, no interrupts, no caches, no MMU, no floating point.
- Board build memory is small (128 instruction words, 32 data words).

## Related repositories
This is one of 9 individual processor projects on FPGA: MIPS-compatible, RISC-V and ARMv4-compatible, each as single-cycle, multi-cycle and pipelined. <!-- add links when ready -->

## License
MIT, see `LICENSE`.

## Trademarks
RISC-V is a trademark of RISC-V International; this project is independent and not affiliated with or endorsed by them.

## Author
Md. Rakib Rayhan, Dept. of ECE, North South University. Research Assistant, Microprocessor, Embedded Systems and IoT Lab (MESIL).
