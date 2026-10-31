//====================================================================
// dmem_riscv_mc.sv  (2026-10-24 Step E, gap G18) - same function as the old dmem_riscv.v of this core.
//   CHANGE: the 32-word RAM is now written with the Intel SystemVerilog byte-enable template
//   (logic [3:0][7:0], one write per byte lane), the same style as mem_arm_*.sv / mem_mips_*.sv,
//   so Quartus maps it to ONE M9K. The old 'reg [31:0]' array with part-select writes was built
//   from 1,024 flip-flops. The M9K output register has no power-up value (as in the ARM cores).
//   Module renamed dmem_riscv_mc (was dmem_riscv) so the multi-cycle and pipelined files have unique names.
//====================================================================
//====================================================================
// dmem_riscv.v   (RISC-V RV32I multi-cycle CPU - step 5)
//
//   Synchronous-read data memory with the byte/half/word load-store
//   unit of the single-cycle dmem_riscv.v. See multicycle_spec.md
//   Sections 2.2, 2.4 and 5.
//
//   WHAT CHANGED vs. the single-cycle dmem_riscv.v
//     - The read is now REGISTERED. A word is captured into 'rdata_word'
//       at the clock edge that ends a cycle with mem_read = 1 (the
//       MEMREAD state). It is usable in the NEXT cycle (MEMWB) and then
//       holds until the next mem_read.
//     - mem_read is a new input. mem_write is unchanged.
//     - read_data is still sign/zero-extended and lane-aligned, but it
//       is now built from the REGISTERED word.
//
//   WHAT STAYED THE SAME
//     - 32 words x 32 bit in one M9K block; the read register is the [HH]
//       Data register (MemRead state).
//     - addr[6:2] = word, addr[1:0] = byte lane, funct3 = size/sign.
//     - Byte-enable stores (sb/sh/sw), data.hex loaded at configuration,
//       no reset reload of memory contents.
//     - No misalignment trap: word accesses ignore addr[1:0], halfword
//       accesses ignore addr[0] (the same rule as the ARM cores).
//
//   TIMING CONTRACT with the FSM
//     MEMREAD : addr = ALUOut, funct3 = instr[14:12], mem_read = 1
//     MEMWB   : addr and funct3 must STILL be the same (they select the
//               byte lane and extension), read_data is valid
//     MEMWRITE: addr = ALUOut, write_data = B, funct3, mem_write = 1;
//               the write completes at the end of that cycle
//     mem_read / mem_write should be gated with step_enable in the top.
//
//   funct3   op    size   sign
//   000      lb    byte   signed        100  lbu  byte  unsigned
//   001      lh    half   signed        101  lhu  half  unsigned
//   010      lw    word   -
//   Stores use 000 (sb), 001 (sh), 010 (sw); funct3[2] is ignored.
//====================================================================
module dmem_riscv_mc #(
    parameter ADDR_BITS = 5,                  // 2**5 = 32 words
    parameter DATA_FILE = "data.hex"
) (
    input         clk,
    input  [31:0] addr,          // byte address (ALUOut = rs1 + imm)
    input  [31:0] write_data,    // raw rs2 value (B register)
    input  [2:0]  funct3,
    input         mem_read,      // 1 in MEMREAD: capture word into rdata_word
    input         mem_write,     // 1 in MEMWRITE
    output [31:0] read_data      // aligned + extended, valid in MEMWB
);

    localparam DEPTH = 1 << ADDR_BITS;

    // data memory: one M9K block, initialised from data.hex
    (* ramstyle = "M9K, no_rw_check" *) logic [3:0][7:0] ram [0:DEPTH-1];
    initial $readmemh(DATA_FILE, ram);

    wire [ADDR_BITS-1:0] word_idx = addr[ADDR_BITS+1:2];   // upper address bits are ignored
    wire [1:0] byte_off    = addr[1:0];
    wire [1:0] size        = funct3[1:0];     // 00 byte, 01 half, 10 word
    wire       unsigned_ld = funct3[2];

    //------------------------------------------------------------
    // READ: registered word (this is the "synchronous read")
    //------------------------------------------------------------
    reg [31:0] rdata_word;

    // synthesis translate_off
        // synthesis translate_on

    // (the M9K read is in the write block below: Intel byte-enable RAM template, one always block)

    // lane select + extension, from the REGISTERED word
    wire [7:0]  sel_byte = rdata_word[byte_off*8 +: 8];
    wire [15:0] sel_half = byte_off[1] ? rdata_word[31:16] : rdata_word[15:0];

    reg [31:0] read_data_r;
    assign read_data = read_data_r;

    always @(*) begin
        case (size)
            2'b00:   read_data_r = unsigned_ld ? {24'b0, sel_byte} : {{24{sel_byte[7]}}, sel_byte};
            2'b01:   read_data_r = unsigned_ld ? {16'b0, sel_half} : {{16{sel_half[15]}}, sel_half};
            default: read_data_r = rdata_word;
        endcase
    end

    //------------------------------------------------------------
    // WRITE: byte-enable mask + data lined up in the target lane(s)
    //------------------------------------------------------------
    reg [3:0]  byte_en;
    reg [31:0] write_word;

    always @(*) begin
        case (size)
            2'b00: begin                          // sb
                byte_en    = 4'b0001 << byte_off;
                write_word = {4{write_data[7:0]}};
            end
            2'b01: begin                          // sh
                byte_en    = byte_off[1] ? 4'b1100 : 4'b0011;
                write_word = {2{write_data[15:0]}};
            end
            default: begin                        // sw
                byte_en    = 4'b1111;
                write_word = write_data;
            end
        endcase
    end

    // byte-enable write
    always @(posedge clk) begin
        if (mem_write) begin
            if (byte_en[0]) ram[word_idx][0]   <= write_word[7:0];
            if (byte_en[1]) ram[word_idx][1]  <= write_word[15:8];
            if (byte_en[2]) ram[word_idx][2] <= write_word[23:16];
            if (byte_en[3]) ram[word_idx][3] <= write_word[31:24];
        end
        if (mem_read)
            rdata_word <= ram[word_idx];             // M9K output register = [HH] ReadData / Data register
    end

endmodule
