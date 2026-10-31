#!/usr/bin/env python3
"""
RV32I base-ISA assembler for the RISC-V single-cycle CPU project (v2).

Syntax is standard RISC-V assembler style:

  Registers : x0-x31, OR the ABI names (zero, ra, sp, gp, tp, t0-t6,
              s0-s11 (fp), a0-a7) - case-insensitive.

  R-type    : add sub and or xor sll srl sra slt sltu   "add rd, rs1, rs2"
  I-type ALU: addi andi ori xori slti sltiu             "addi rd, rs1, imm"
              slli srli srai                            "slli rd, rs1, shamt"
  Loads     : lb lh lw lbu lhu     "lw rd, offset(rs1)"   (offset optional)
  Stores    : sb sh sw             "sw rs2, offset(rs1)"  (offset optional)
  Branches  : beq bne blt bge bltu bgeu   "beq rs1, rs2, label"
  Jumps     : jal rd, label | jal label (rd = ra)
              jalr rd, offset(rs1) | jalr rd, rs1, offset | jalr rs1
  Upper imm : lui rd, imm20 | auipc rd, imm20  (imm20 = bits [31:12])

  Pseudo-ops (each expands to real instructions):
     nop | mv rd,rs | not rd,rs | neg rd,rs | seqz/snez/sltz/sgtz rd,rs
     li rd,imm      (any 32-bit value; 1 instruction if it fits 12 bits,
                     otherwise lui + addi)
     la rd,label    (label from the .data section -> its byte address)
     j label | jr rs | ret | call label | tail label
     beqz/bnez/blez/bgez/bltz/bgtz rs,label
     bgt/ble/bgtu/bleu rs1,rs2,label

  Labels    : "label:" alone on a line, or "label: INSTRUCTION ...".
              Several labels in a row all point at the next instruction.
              A label at the very end points just past the last instruction.
  Comments  : anything after '#'.
  Directives: ".data" starts the data section (one 32-bit value per line,
              decimal or 0x-hex, optional "label:" prefix); ".text" is ignored.

NOT supported (they decode as illegal on the 37-instruction core):
  fence, ecall, ebreak, and the CSR instructions.

Usage:
  python3 assembler_riscv.py program.asm [--imem-depth 128] [--dmem-depth 32]
                                         [--outdir DIR]

Writes program.hex (imem contents, padded to --imem-depth words) and
data.hex (dmem contents, padded to --dmem-depth words). If the program
does not end with a "jal x0, ..." an automatic self-loop (jal x0,0 =
0x0000006F) is appended - this is the "halted" convention the status
LED / ISSP bit uses. The program INCLUDING that appended word must fit in
--imem-depth words, otherwise the assembler stops with an error (the CPU's
PC would wrap around inside a smaller memory and run the wrong code).

All branch/jump offsets are computed in BYTES (4 bytes per instruction).
"""

import argparse
import os
import re
import sys

ABI_NAMES = {
    'zero': 0, 'ra': 1, 'sp': 2, 'gp': 3, 'tp': 4,
    't0': 5, 't1': 6, 't2': 7,
    's0': 8, 'fp': 8, 's1': 9,
    'a0': 10, 'a1': 11, 'a2': 12, 'a3': 13, 'a4': 14, 'a5': 15, 'a6': 16, 'a7': 17,
    's2': 18, 's3': 19, 's4': 20, 's5': 21, 's6': 22, 's7': 23,
    's8': 24, 's9': 25, 's10': 26, 's11': 27,
    't3': 28, 't4': 29, 't5': 30, 't6': 31,
}

DEFAULT_IMEM_DEPTH = 128   # matches imem_riscv.v (reg [31:0] mem [0:127])
DEFAULT_DMEM_DEPTH = 32    # matches dmem_riscv.v


def reg_num(token):
    token = token.strip().strip(',').lower()
    if token.startswith('x') and token[1:].isdigit():
        n = int(token[1:])
    elif token in ABI_NAMES:
        n = ABI_NAMES[token]
    else:
        raise ValueError(f"Expected a register like x5 or 'a0', got '{token}'")
    if not (0 <= n <= 31):
        raise ValueError(f"Register out of range: '{token}'")
    return n


# ---------------------------------------------------------------
# Opcodes / funct3 / funct7 (standard RV32I base encoding)
# ---------------------------------------------------------------
OP_RTYPE, OP_ITYPE, OP_LOAD, OP_STORE = 0b0110011, 0b0010011, 0b0000011, 0b0100011
OP_BRANCH, OP_JAL, OP_JALR = 0b1100011, 0b1101111, 0b1100111
OP_LUI, OP_AUIPC = 0b0110111, 0b0010111

R_FUNCT = {
    'ADD': (0b000, 0b0000000), 'SUB': (0b000, 0b0100000),
    'SLL': (0b001, 0b0000000), 'SLT': (0b010, 0b0000000),
    'SLTU': (0b011, 0b0000000), 'XOR': (0b100, 0b0000000),
    'SRL': (0b101, 0b0000000), 'SRA': (0b101, 0b0100000),
    'OR': (0b110, 0b0000000), 'AND': (0b111, 0b0000000),
}
ITYPE_FUNCT3 = {
    'ADDI': 0b000, 'SLTI': 0b010, 'SLTIU': 0b011,
    'XORI': 0b100, 'ORI': 0b110, 'ANDI': 0b111,
}
SHIFT_IMM = {'SLLI': (0b001, 0b0000000), 'SRLI': (0b101, 0b0000000), 'SRAI': (0b101, 0b0100000)}
LOAD_FUNCT3 = {'LB': 0b000, 'LH': 0b001, 'LW': 0b010, 'LBU': 0b100, 'LHU': 0b101}
STORE_FUNCT3 = {'SB': 0b000, 'SH': 0b001, 'SW': 0b010}
BRANCH_FUNCT3 = {'BEQ': 0b000, 'BNE': 0b001, 'BLT': 0b100, 'BGE': 0b101, 'BLTU': 0b110, 'BGEU': 0b111}

PSEUDO_OPS = {
    'NOP', 'MV', 'NOT', 'NEG', 'SEQZ', 'SNEZ', 'SLTZ', 'SGTZ', 'LI', 'LA',
    'J', 'JR', 'RET', 'CALL', 'TAIL',
    'BEQZ', 'BNEZ', 'BLEZ', 'BGEZ', 'BLTZ', 'BGTZ', 'BGT', 'BLE', 'BGTU', 'BLEU',
}


def strip_comment(line):
    return line.split('#', 1)[0]


def split_sections(text):
    lines = text.splitlines()
    for i, raw in enumerate(lines):
        if strip_comment(raw).strip().lower() == '.data':
            return '\n'.join(lines[:i]), '\n'.join(lines[i + 1:])
    return text, ''


def _ops(operands):
    return [o.strip() for o in operands.split(',') if o.strip()]


def _need(ops, n, name):
    if len(ops) != n:
        raise ValueError(f"'{name.lower()}' needs {n} operand(s), got {len(ops)}")


def expand_pseudo(mnemonic, operands, data_labels):
    """Rewrite a pseudo-op into a LIST of (real_mnemonic, real_operands)."""
    o = _ops(operands)
    if mnemonic == 'NOP':
        return [('ADDI', 'x0, x0, 0')]
    if mnemonic == 'MV':
        _need(o, 2, mnemonic); return [('ADDI', f'{o[0]}, {o[1]}, 0')]
    if mnemonic == 'NOT':
        _need(o, 2, mnemonic); return [('XORI', f'{o[0]}, {o[1]}, -1')]
    if mnemonic == 'NEG':
        _need(o, 2, mnemonic); return [('SUB', f'{o[0]}, x0, {o[1]}')]
    if mnemonic == 'SEQZ':
        _need(o, 2, mnemonic); return [('SLTIU', f'{o[0]}, {o[1]}, 1')]
    if mnemonic == 'SNEZ':
        _need(o, 2, mnemonic); return [('SLTU', f'{o[0]}, x0, {o[1]}')]
    if mnemonic == 'SLTZ':
        _need(o, 2, mnemonic); return [('SLT', f'{o[0]}, {o[1]}, x0')]
    if mnemonic == 'SGTZ':
        _need(o, 2, mnemonic); return [('SLT', f'{o[0]}, x0, {o[1]}')]
    if mnemonic == 'LI':
        _need(o, 2, mnemonic)
        val = int(o[1], 0)
        if not (-2 ** 31 <= val <= 2 ** 32 - 1):
            raise ValueError(f"li value {val} doesn't fit in 32 bits")
        val &= 0xFFFFFFFF
        signed = val - (1 << 32) if val >> 31 else val
        if -2048 <= signed <= 2047:
            return [('ADDI', f'{o[0]}, x0, {signed}')]
        hi = ((val + 0x800) >> 12) & 0xFFFFF
        lo = val & 0xFFF
        lo = lo - 4096 if lo >= 2048 else lo
        out = [('LUI', f'{o[0]}, {hi}')]
        if lo != 0:
            out.append(('ADDI', f'{o[0]}, {o[0]}, {lo}'))
        return out
    if mnemonic == 'LA':
        _need(o, 2, mnemonic)
        if o[1] not in data_labels:
            raise ValueError(f"la: unknown .data label '{o[1]}'")
        return [('ADDI', f'{o[0]}, x0, {data_labels[o[1]] * 4}')]
    if mnemonic == 'J':
        _need(o, 1, mnemonic); return [('JAL', f'x0, {o[0]}')]
    if mnemonic == 'JR':
        _need(o, 1, mnemonic); return [('JALR', f'x0, 0({o[0]})')]
    if mnemonic == 'RET':
        return [('JALR', 'x0, 0(ra)')]
    if mnemonic == 'CALL':
        _need(o, 1, mnemonic); return [('JAL', f'ra, {o[0]}')]
    if mnemonic == 'TAIL':
        _need(o, 1, mnemonic); return [('JAL', f'x0, {o[0]}')]
    # branch pseudo-ops
    if mnemonic in ('BEQZ', 'BNEZ', 'BLEZ', 'BGEZ', 'BLTZ', 'BGTZ'):
        _need(o, 2, mnemonic)
        rs, lab = o
        table = {'BEQZ': ('BEQ', f'{rs}, x0, {lab}'), 'BNEZ': ('BNE', f'{rs}, x0, {lab}'),
                 'BLEZ': ('BGE', f'x0, {rs}, {lab}'), 'BGEZ': ('BGE', f'{rs}, x0, {lab}'),
                 'BLTZ': ('BLT', f'{rs}, x0, {lab}'), 'BGTZ': ('BLT', f'x0, {rs}, {lab}')}
        return [table[mnemonic]]
    if mnemonic in ('BGT', 'BLE', 'BGTU', 'BLEU'):
        _need(o, 3, mnemonic)
        a, b, lab = o
        real = {'BGT': 'BLT', 'BLE': 'BGE', 'BGTU': 'BLTU', 'BLEU': 'BGEU'}[mnemonic]
        return [(real, f'{b}, {a}, {lab}')]
    raise ValueError(f"Unknown pseudo-op '{mnemonic}'")


def parse_lines(text, data_labels):
    """Returns (instructions, end_labels). Each instruction is
    (mnemonic, [labels], operands, raw_line). Several labels in a row all
    attach to the next instruction; labels after the last instruction are
    returned as end_labels."""
    expanded = []
    pending = []
    for raw in text.splitlines():
        line = strip_comment(raw).strip()
        if not line:
            continue
        if line.lower() == '.text':
            continue
        # peel off any number of "label:" prefixes
        while True:
            m = re.match(r'^(\w+):\s*(.*)$', line)
            if not m:
                break
            pending.append(m.group(1))
            line = m.group(2).strip()
        if not line:
            continue
        parts = line.split(None, 1)
        mnemonic = parts[0].upper()
        operands = parts[1] if len(parts) > 1 else ''
        try:
            if mnemonic in PSEUDO_OPS:
                real = expand_pseudo(mnemonic, operands, data_labels)
            else:
                real = [(mnemonic, operands)]
        except ValueError as e:
            raise ValueError(f"{e}  [line: {raw.strip()}]")
        for k, (mn, ops) in enumerate(real):
            expanded.append((mn, pending if k == 0 else [], ops, raw.strip()))
        pending = []
    return expanded, pending


def first_pass(instrs, end_labels):
    labels = {}
    def add(name, addr):
        if name in labels:
            raise ValueError(f"Duplicate label '{name}'")
        labels[name] = addr
    for idx, (_, lbls, _, _) in enumerate(instrs):
        for name in lbls:
            add(name, idx * 4)
    for name in end_labels:
        add(name, len(instrs) * 4)
    return labels


# ---------------------------------------------------------------
# Encoders
# ---------------------------------------------------------------
def _imm_range_check(name, imm):
    if not (-2048 <= imm <= 2047):
        hint = ""
        if 2048 <= imm <= 4095:
            hint = f" (did you mean {imm - 4096}? 12-bit immediates are signed)"
        raise ValueError(f"{name} immediate {imm} out of 12-bit signed range{hint}")


def encode_rtype(mnemonic, operands):
    o = _ops(operands); _need(o, 3, mnemonic)
    rd, rs1, rs2 = (reg_num(x) for x in o)
    funct3, funct7 = R_FUNCT[mnemonic]
    return (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | OP_RTYPE


def encode_itype_alu(mnemonic, operands):
    o = _ops(operands); _need(o, 3, mnemonic)
    rd, rs1 = reg_num(o[0]), reg_num(o[1])
    imm = int(o[2], 0)
    _imm_range_check(mnemonic, imm)
    funct3 = ITYPE_FUNCT3[mnemonic]
    return ((imm & 0xFFF) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | OP_ITYPE


def encode_shift_imm(mnemonic, operands):
    o = _ops(operands); _need(o, 3, mnemonic)
    rd, rs1 = reg_num(o[0]), reg_num(o[1])
    shamt = int(o[2], 0)
    if not (0 <= shamt <= 31):
        raise ValueError(f"{mnemonic} shift amount {shamt} out of range 0-31")
    funct3, funct7 = SHIFT_IMM[mnemonic]
    imm12 = (funct7 << 5) | shamt
    return (imm12 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | OP_ITYPE


def _parse_mem_operand(operands):
    # "rX, offset(rY)" or "rX, (rY)" (offset defaults to 0)
    m = re.match(r'^\s*(\w+)\s*,\s*(-?\w+)?\s*\(\s*(\w+)\s*\)\s*$', operands)
    if not m:
        raise ValueError(f"Bad memory operand syntax: '{operands}' (expected reg, offset(base))")
    reg_tok, off_tok, base_tok = m.groups()
    return reg_num(reg_tok), (int(off_tok, 0) if off_tok else 0), reg_num(base_tok)


def encode_load(mnemonic, operands):
    rd, imm, rs1 = _parse_mem_operand(operands)
    _imm_range_check(mnemonic, imm)
    return ((imm & 0xFFF) << 20) | (rs1 << 15) | (LOAD_FUNCT3[mnemonic] << 12) | (rd << 7) | OP_LOAD


def encode_store(mnemonic, operands):
    rs2, imm, rs1 = _parse_mem_operand(operands)   # first field is the SOURCE register
    _imm_range_check(mnemonic, imm)
    imm_hi, imm_lo = (imm >> 5) & 0x7F, imm & 0x1F
    return (imm_hi << 25) | (rs2 << 20) | (rs1 << 15) | (STORE_FUNCT3[mnemonic] << 12) | (imm_lo << 7) | OP_STORE


def encode_branch(mnemonic, operands, addr, labels):
    o = _ops(operands); _need(o, 3, mnemonic)
    rs1, rs2, target = reg_num(o[0]), reg_num(o[1]), o[2]
    if target not in labels:
        raise ValueError(f"Unknown label '{target}'")
    offset = labels[target] - addr
    if offset % 2 != 0:
        raise ValueError(f"Branch offset {offset} is not 2-byte aligned")
    if not (-4096 <= offset <= 4094):
        raise ValueError(f"Branch offset {offset} out of 13-bit signed range")
    funct3 = BRANCH_FUNCT3[mnemonic]
    imm12, imm11, imm10_5, imm4_1 = (offset >> 12) & 1, (offset >> 11) & 1, (offset >> 5) & 0x3F, (offset >> 1) & 0xF
    return ((imm12 << 31) | (imm10_5 << 25) | (rs2 << 20) | (rs1 << 15) |
            (funct3 << 12) | (imm4_1 << 8) | (imm11 << 7) | OP_BRANCH)


def encode_jal(operands, addr, labels):
    o = _ops(operands)
    if len(o) == 2:
        rd, target = reg_num(o[0]), o[1]
    elif len(o) == 1:           # "jal label" - rd defaults to x1 (ra)
        rd, target = 1, o[0]
    else:
        raise ValueError("'jal' needs 'rd, label' or 'label'")
    if target not in labels:
        raise ValueError(f"Unknown label '{target}'")
    offset = labels[target] - addr
    if offset % 2 != 0:
        raise ValueError(f"JAL offset {offset} is not 2-byte aligned")
    if not (-1048576 <= offset <= 1048574):
        raise ValueError(f"JAL offset {offset} out of 21-bit signed range")
    imm20, imm19_12, imm11, imm10_1 = (offset >> 20) & 1, (offset >> 12) & 0xFF, (offset >> 11) & 1, (offset >> 1) & 0x3FF
    return (imm20 << 31) | (imm19_12 << 12) | (imm11 << 20) | (imm10_1 << 21) | (rd << 7) | OP_JAL


def encode_jalr(operands):
    o = _ops(operands)
    if '(' in operands:                       # jalr rd, offset(rs1)
        rd, imm, rs1 = _parse_mem_operand(operands)
    elif len(o) == 1:                         # jalr rs1
        rd, imm, rs1 = 1, 0, reg_num(o[0])
    elif len(o) == 3:                         # jalr rd, rs1, offset
        rd, rs1, imm = reg_num(o[0]), reg_num(o[1]), int(o[2], 0)
    else:
        raise ValueError("'jalr' needs 'rd, offset(rs1)', 'rd, rs1, offset' or 'rs1'")
    _imm_range_check("JALR", imm)
    return ((imm & 0xFFF) << 20) | (rs1 << 15) | (0b000 << 12) | (rd << 7) | OP_JALR


def encode_utype(mnemonic, operands):
    o = _ops(operands); _need(o, 2, mnemonic)
    rd, imm20 = reg_num(o[0]), int(o[1], 0)
    if not (0 <= imm20 <= 0xFFFFF):
        raise ValueError(f"{mnemonic} immediate {imm20:#x} doesn't fit 20 bits (bits [31:12] directly)")
    return (imm20 << 12) | (rd << 7) | (OP_LUI if mnemonic == 'LUI' else OP_AUIPC)


def assemble(text, data_labels=None):
    data_labels = data_labels or {}
    instrs, end_labels = parse_lines(text, data_labels)
    labels = first_pass(instrs, end_labels)

    results = []
    for idx, (mnemonic, _, operands, raw) in enumerate(instrs):
        addr = idx * 4
        try:
            if mnemonic in R_FUNCT:
                code = encode_rtype(mnemonic, operands)
            elif mnemonic in ITYPE_FUNCT3:
                code = encode_itype_alu(mnemonic, operands)
            elif mnemonic in SHIFT_IMM:
                code = encode_shift_imm(mnemonic, operands)
            elif mnemonic in LOAD_FUNCT3:
                code = encode_load(mnemonic, operands)
            elif mnemonic in STORE_FUNCT3:
                code = encode_store(mnemonic, operands)
            elif mnemonic in BRANCH_FUNCT3:
                code = encode_branch(mnemonic, operands, addr, labels)
            elif mnemonic == 'JAL':
                code = encode_jal(operands, addr, labels)
            elif mnemonic == 'JALR':
                code = encode_jalr(operands)
            elif mnemonic in ('LUI', 'AUIPC'):
                code = encode_utype(mnemonic, operands)
            else:
                raise ValueError(f"Unknown instruction '{mnemonic}'")
        except ValueError as e:
            raise ValueError(f"{e}  [line: {raw}]")
        results.append((addr, mnemonic, operands, code, raw))
    return results, labels


def parse_data(text):
    values, labels = [], {}
    for raw in text.splitlines():
        line = strip_comment(raw).strip()
        if not line:
            continue
        while True:
            m = re.match(r'^(\w+):\s*(.*)$', line)
            if not m:
                break
            labels[m.group(1)] = len(values)
            line = m.group(2).strip()
        if not line:
            continue
        value = int(line, 0)
        if not (-2 ** 31 <= value <= 2 ** 32 - 1):
            raise ValueError(f".data value {value} doesn't fit in 32 bits")
        values.append(value & 0xFFFFFFFF)
    return values, labels


def print_report(results, labels):
    print("Labels found:")
    for name, addr in labels.items():
        print(f"  {name:12s} -> byte address {addr:#06x}")
    print()
    print(f"{'Addr':>6}  {'Source':40s}  Hex")
    print("-" * 64)
    for addr, mnemonic, operands, code, raw in results:
        print(f"{addr:#06x}  {raw:40s}  32'h{code:08X}")
    print()
    print(f"Program length: {len(results)} instructions ({len(results) * 4} bytes)")


def write_hex_file(codes, filename, depth, word_desc):
    if len(codes) > depth:
        raise ValueError(f"{word_desc} use {len(codes)} words but the memory is only {depth} words deep. "
                         f"Shorten the program (or change --imem-depth/--dmem-depth if the hardware changed).")
    with open(filename, 'w') as f:
        for code in codes:
            f.write(f"{code:08X}\n")
        for _ in range(depth - len(codes)):
            f.write("00000000\n")
    print(f"Wrote {filename} ({len(codes)} real {word_desc}, padded to {depth} words with zeros)")


def main():
    ap = argparse.ArgumentParser(description="RV32I assembler for the single-cycle CPU project")
    ap.add_argument('source', help='assembly source file')
    ap.add_argument('--imem-depth', type=int, default=DEFAULT_IMEM_DEPTH,
                    help=f'instruction memory depth in words (default {DEFAULT_IMEM_DEPTH})')
    ap.add_argument('--dmem-depth', type=int, default=DEFAULT_DMEM_DEPTH,
                    help=f'data memory depth in words (default {DEFAULT_DMEM_DEPTH})')
    ap.add_argument('--outdir', default='.', help='where to write program.hex / data.hex')
    args = ap.parse_args()

    with open(args.source) as f:
        full_text = f.read()

    instr_text, data_text = split_sections(full_text)
    data_values, data_labels = parse_data(data_text)       # parsed first so 'la' can resolve labels
    try:
        results, labels = assemble(instr_text, data_labels)
    except ValueError as e:
        print(f"ASSEMBLY ERROR: {e}")
        sys.exit(1)
    print_report(results, labels)

    codes = [code for (_, _, _, code, _) in results]

    # RV32I has no HALT opcode; the convention is a self-loop so a program
    # that runs off the end doesn't execute the zero padding. Auto-append
    # one unless the program already ends with "jal x0, ...".
    if not results or not (results[-1][1] == 'JAL' and results[-1][2].replace(' ', '').startswith('x0,')):
        codes.append(0x0000006F)  # jal x0, 0
        print("(No trailing 'jal x0, ...' found - auto-appended a self-loop 'jal x0, 0' at the end.)")
        print()

    os.makedirs(args.outdir, exist_ok=True)
    try:
        write_hex_file(codes, os.path.join(args.outdir, "program.hex"), args.imem_depth, "instruction words")
        print()
        if data_labels:
            print("Data labels found:")
            for name, addr in data_labels.items():
                print(f"  {name:12s} -> memory[{addr}] (word address, byte address {addr * 4})")
            print()
        write_hex_file(data_values, os.path.join(args.outdir, "data.hex"), args.dmem_depth, "data words")
    except ValueError as e:
        print(f"ERROR: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()