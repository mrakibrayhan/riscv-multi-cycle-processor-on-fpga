//====================================================================
// control_fsm_riscv.v   (RV32IM multi-cycle CPU - main FSM + instruction legality)
//
//   Reference: [HH] Harris & Harris, "Digital Design and Computer Architecture,
//   RISC-V Edition", Section 7.4 (multicycle processor): Fig. 7.27 datapath, main FSM
//   (Fetch, Decode, MemAdr, MemRead, MemWB, MemWrite, ExecuteR, ExecuteI, ALUWB, BEQ, JAL).
//   ISA: [RV] The RISC-V Instruction Set Manual, Vol. I (Unprivileged), RV32I + M.
//
//   The [HH] states are kept with their [HH] control values. Added states (the [HH] text
//   leaves these instructions as extensions), all built from the same datapath:
//     BRANCH (= [HH] BEQ, all six branches)  JALR_EX/JALR_PC  LUI  AUIPC
//     HALT (ecall / ebreak / fault)          MD_START/MD_WAIT/MD_WB (RV32M unit)
//
//   ALUOp = the [HH] encoding: 00 add, 01 branch compare, 10 "use funct3" (R-type AND
//   I-type, SUB only when op[5] & funct7[5], as in the [HH] ALU decoder).
//
//   Global freeze: en = 0 -> 'state' holds and every output that changes machine state is
//   0. The select outputs only steer muxes and are not gated.
//
//   Instruction legality (exact RV32I / RV32M decode, like decoder_arm.v in the ARM cores):
//   an encoding that is not an RV32I(M) instruction stops the CPU in HALT with fault=1
//   (illegal_flag). There are no traps (no CSRs, M-mode only), so this "stop with fault" is
//   the same rule the ARM cores use for undefined instructions.
//   A taken branch / jal / jalr whose target is not 4-byte aligned is the [RV]
//   "instruction-address-misaligned" case: the PC is NOT written, the CPU stops in HALT with
//   fault=1 (mis_flag) - the same rule as the ARM cores' "BX to an odd address".
//   ecall / ebreak stop the CPU in HALT with fault=0 (normal end of program).
//
//   Cycles: fence 2 | branch 3 | R, I, lui, auipc, jal, store 4 | load, jalr 5 |
//           mul* 38 | div*/rem* 42 | ecall/ebreak/illegal: stop after DECODE.
//====================================================================
module control_fsm_riscv #(
    parameter ENABLE_M = 1          // 1: RV32M multiply/divide, 0: those encodings are illegal, 2: Zmmul (mul, mulh, mulhsu, mulhu only; Step E)
) (
    input         clk,
    input         rst,           // synchronous, active-high
    input         en,            // step enable
    input  [31:0] instr,         // held instruction (valid from DECODE on)
    input         md_done,       // multiply/divide unit finished
    input         taken,         // branch condition (valid in BRANCH)
    input         tgt_bit1,      // bit 1 of the PC target (Result[1]); valid in BRANCH/JAL/JALR_PC

    // ---- datapath selects ----
    output reg [1:0] alu_src_a,  // 00 PC | 01 OldPC | 10 A | 11 zero
    output reg [1:0] alu_src_b,  // 00 B  | 01 ImmExt | 10 four
    output reg [1:0] alu_op,     // 00 add | 01 branch | 10 funct3
    output reg [1:0] result_src, // 00 ALUOut | 01 Data | 10 ALUResult | 11 MulDiv
    output reg [2:0] imm_src,    // 000 I | 001 S | 010 B | 011 U | 100 J

    // ---- state-changing controls (gated by en) ----
    output           pc_write,   // [HH] PCUpdate (already blocked for a misaligned target)
    output           branch,     // [HH] Branch   (already blocked for a misaligned target)
    output           reg_write,
    output           mem_read,   // data-memory read register enable (MemRead)
    output           mem_write,
    output           fetch,      // [HH] IRWrite: instruction register (imem output) + OldPC load
    output           jalr_mask,  // clear PC bit 0 (JALR_PC)
    output           md_start,

    // ---- status ----
    output           stop_event,    // 1 for the enabled cycle that enters HALT
    output reg       illegal_flag,  // sticky: stopped on an illegal instruction
    output reg       mis_flag,      // sticky: stopped on a misaligned jump/branch target
    output reg       sys_flag,      // sticky: stopped by ecall / ebreak
    output [4:0]     state_out
);

    localparam S_FETCH    = 5'd0;
    localparam S_DECODE   = 5'd1;
    localparam S_MEMADR   = 5'd2;
    localparam S_MEMREAD  = 5'd3;
    localparam S_MEMWB    = 5'd4;
    localparam S_MEMWRITE = 5'd5;
    localparam S_EXECUTER = 5'd6;
    localparam S_EXECUTEI = 5'd7;
    localparam S_ALUWB    = 5'd8;
    localparam S_BRANCH   = 5'd9;
    localparam S_JAL      = 5'd10;
    localparam S_JALR_EX  = 5'd11;
    localparam S_JALR_PC  = 5'd12;
    localparam S_LUI      = 5'd13;
    localparam S_AUIPC    = 5'd14;
    localparam S_HALT     = 5'd15;
    localparam S_MD_START = 5'd16;
    localparam S_MD_WAIT  = 5'd17;
    localparam S_MD_WB    = 5'd18;

    localparam OP_RTYPE  = 7'b0110011;
    localparam OP_ITYPE  = 7'b0010011;
    localparam OP_LOAD   = 7'b0000011;
    localparam OP_STORE  = 7'b0100011;
    localparam OP_BRANCH = 7'b1100011;
    localparam OP_JAL    = 7'b1101111;
    localparam OP_JALR   = 7'b1100111;
    localparam OP_LUI    = 7'b0110111;
    localparam OP_AUIPC  = 7'b0010111;
    localparam OP_FENCE  = 7'b0001111;
    localparam OP_SYSTEM = 7'b1110011;

    wire [6:0] opcode = instr[6:0];
    wire [2:0] f3     = instr[14:12];
    wire [6:0] f7     = instr[31:25];

    reg [4:0] state, next_state;
    assign state_out = state;

    //------------------------------------------------------------
    // Exact RV32I / RV32M legality
    //------------------------------------------------------------
    wire md_op  = (opcode == OP_RTYPE) && (f7 == 7'b0000001);
    wire sys_op = (instr == 32'h00000073) || (instr == 32'h00100073);    // ecall, ebreak
    reg  legal;
    always @(*) begin
        case (opcode)
            OP_LUI, OP_AUIPC, OP_JAL: legal = 1'b1;
            OP_JALR:   legal = (f3 == 3'b000);
            OP_BRANCH: legal = (f3 != 3'b010) && (f3 != 3'b011);
            OP_LOAD:   legal = (f3 == 3'b000) || (f3 == 3'b001) || (f3 == 3'b010) ||
                               (f3 == 3'b100) || (f3 == 3'b101);
            OP_STORE:  legal = (f3 == 3'b000) || (f3 == 3'b001) || (f3 == 3'b010);
            OP_ITYPE:  legal = (f3 == 3'b001) ? (f7 == 7'b0000000) :                          // slli
                               (f3 == 3'b101) ? ((f7 == 7'b0000000) || (f7 == 7'b0100000)) :  // srli/srai
                               1'b1;
            OP_RTYPE:  legal = (f7 == 7'b0000000) ||
                               ((f7 == 7'b0100000) && ((f3 == 3'b000) || (f3 == 3'b101))) ||  // sub, sra
                               ((f7 == 7'b0000001) && ((ENABLE_M == 1) || ((ENABLE_M == 2) && !f3[2])));
            OP_FENCE:  legal = (f3 == 3'b000);                                               // FENCE (fence.i = Zifencei, not built)
            OP_SYSTEM: legal = sys_op;                                                        // CSR instructions not built
            default:   legal = 1'b0;
        endcase
    end

    // misaligned PC target: only in the states that write a jump/branch target
    wire tgt_state = (state == S_JAL) || (state == S_JALR_PC) || ((state == S_BRANCH) && taken);
    wire pc_mis    = tgt_state && tgt_bit1;

    //------------------------------------------------------------
    // State register
    //------------------------------------------------------------
    always @(posedge clk) begin
        if (rst)
            state <= S_FETCH;
        else if (en)
            state <= next_state;
    end

    //------------------------------------------------------------
    // Next-state logic
    //------------------------------------------------------------
    always @(*) begin
        case (state)
            S_FETCH:    next_state = S_DECODE;
            S_DECODE: begin
                if (!legal)
                    next_state = S_HALT;                     // illegal -> stop with fault
                else case (opcode)
                    OP_LOAD,
                    OP_STORE:  next_state = S_MEMADR;
                    OP_RTYPE:  next_state = md_op ? S_MD_START : S_EXECUTER;
                    OP_ITYPE:  next_state = S_EXECUTEI;
                    OP_BRANCH: next_state = S_BRANCH;
                    OP_JAL:    next_state = S_JAL;
                    OP_JALR:   next_state = S_JALR_EX;
                    OP_LUI:    next_state = S_LUI;
                    OP_AUIPC:  next_state = S_AUIPC;
                    OP_FENCE:  next_state = S_FETCH;         // no-op (single core, no caches)
                    default:   next_state = S_HALT;          // OP_SYSTEM: ecall / ebreak
                endcase
            end
            S_MEMADR:   next_state = (opcode == OP_STORE) ? S_MEMWRITE : S_MEMREAD;
            S_MEMREAD:  next_state = S_MEMWB;
            S_MEMWB:    next_state = S_FETCH;
            S_MEMWRITE: next_state = S_FETCH;
            S_EXECUTER: next_state = S_ALUWB;
            S_EXECUTEI: next_state = S_ALUWB;
            S_ALUWB:    next_state = S_FETCH;
            S_BRANCH:   next_state = pc_mis ? S_HALT : S_FETCH;
            S_JAL:      next_state = pc_mis ? S_HALT : S_ALUWB;
            S_JALR_EX:  next_state = S_JALR_PC;
            S_JALR_PC:  next_state = pc_mis ? S_HALT : S_ALUWB;
            S_LUI:      next_state = S_ALUWB;
            S_AUIPC:    next_state = S_ALUWB;
            S_HALT:     next_state = S_HALT;
            S_MD_START: next_state = S_MD_WAIT;
            S_MD_WAIT:  next_state = md_done ? S_MD_WB : S_MD_WAIT;
            S_MD_WB:    next_state = S_FETCH;
            default:    next_state = S_FETCH;
        endcase
    end

    //------------------------------------------------------------
    // Instruction decoder: ImmSrc from the opcode ([HH] Instr Decoder, plus U-type)
    //------------------------------------------------------------
    always @(*) begin
        case (opcode)
            OP_STORE:         imm_src = 3'b001;  // S
            OP_BRANCH:        imm_src = 3'b010;  // B
            OP_LUI, OP_AUIPC: imm_src = 3'b011;  // U
            OP_JAL:           imm_src = 3'b100;  // J
            default:          imm_src = 3'b000;  // I
        endcase
    end

    //------------------------------------------------------------
    // Moore outputs
    //------------------------------------------------------------
    reg r_pc_write, r_branch, r_reg_write, r_mem_read, r_mem_write, r_fetch, r_jalr_mask, r_md_start;

    always @(*) begin
        alu_src_a   = 2'b00;
        alu_src_b   = 2'b00;
        alu_op      = 2'b00;
        result_src  = 2'b00;
        r_pc_write  = 1'b0;
        r_branch    = 1'b0;
        r_reg_write = 1'b0;
        r_mem_read  = 1'b0;
        r_mem_write = 1'b0;
        r_fetch     = 1'b0;
        r_jalr_mask = 1'b0;
        r_md_start  = 1'b0;

        case (state)
            S_FETCH: begin      // [HH] S0: IRWrite, PC <= PC + 4
                alu_src_a = 2'b00; alu_src_b = 2'b10; alu_op = 2'b00;
                result_src = 2'b10; r_pc_write = 1'b1; r_fetch = 1'b1;
            end
            S_DECODE: begin     // [HH] S1: ALUOut <= OldPC + ImmExt (branch / jal target)
                alu_src_a = 2'b01; alu_src_b = 2'b01; alu_op = 2'b00;
            end
            S_MEMADR: begin     // [HH] S2: ALUOut <= A + ImmExt
                alu_src_a = 2'b10; alu_src_b = 2'b01; alu_op = 2'b00;
            end
            S_MEMREAD: begin    // [HH] S3: Data <= Mem[ALUOut]
                alu_src_a = 2'b10; alu_src_b = 2'b01; alu_op = 2'b00;
                result_src = 2'b00; r_mem_read = 1'b1;
            end
            S_MEMWB: begin      // [HH] S4: rd <= Data
                result_src = 2'b01; r_reg_write = 1'b1;
            end
            S_MEMWRITE: begin   // [HH] S5: Mem[ALUOut] <= B
                alu_src_a = 2'b10; alu_src_b = 2'b01; alu_op = 2'b00;
                result_src = 2'b00; r_mem_write = 1'b1;
            end
            S_EXECUTER: begin   // [HH] S6
                alu_src_a = 2'b10; alu_src_b = 2'b00; alu_op = 2'b10;
            end
            S_EXECUTEI: begin   // [HH] S7
                alu_src_a = 2'b10; alu_src_b = 2'b01; alu_op = 2'b10;
            end
            S_ALUWB: begin      // [HH] S8: rd <= ALUOut
                result_src = 2'b00; r_reg_write = 1'b1;
            end
            S_BRANCH: begin     // [HH] S10 BEQ, all six branches
                alu_src_a = 2'b10; alu_src_b = 2'b00; alu_op = 2'b01;
                result_src = 2'b00; r_branch = 1'b1;
            end
            S_JAL: begin        // [HH] S9: PC <= ALUOut, ALUOut <= OldPC + 4
                alu_src_a = 2'b01; alu_src_b = 2'b10; alu_op = 2'b00;
                result_src = 2'b00; r_pc_write = 1'b1;
            end
            S_JALR_EX: begin    // ALUOut <= A + ImmExt
                alu_src_a = 2'b10; alu_src_b = 2'b01; alu_op = 2'b00;
            end
            S_JALR_PC: begin    // PC <= ALUOut & ~1, ALUOut <= OldPC + 4
                alu_src_a = 2'b01; alu_src_b = 2'b10; alu_op = 2'b00;
                result_src = 2'b00; r_pc_write = 1'b1; r_jalr_mask = 1'b1;
            end
            S_LUI: begin        // ALUOut <= 0 + U-imm
                alu_src_a = 2'b11; alu_src_b = 2'b01; alu_op = 2'b00;
            end
            S_AUIPC: begin      // ALUOut <= OldPC + U-imm
                alu_src_a = 2'b01; alu_src_b = 2'b01; alu_op = 2'b00;
            end
            S_MD_START: begin
                r_md_start = 1'b1;
            end
            S_MD_WB: begin
                result_src = 2'b11; r_reg_write = 1'b1;
            end
            default: ;
        endcase
    end

    assign pc_write  = r_pc_write  & en & ~pc_mis;
    assign branch    = r_branch    & en & ~pc_mis;
    assign reg_write = r_reg_write & en;
    assign mem_read  = r_mem_read  & en;
    assign mem_write = r_mem_write & en;
    assign fetch     = r_fetch     & en;
    assign md_start  = r_md_start  & en;
    assign jalr_mask = r_jalr_mask;

    //------------------------------------------------------------
    // Stop status
    //------------------------------------------------------------
    wire dec_illegal = (state == S_DECODE) && !legal;
    wire dec_sys     = (state == S_DECODE) && legal && (opcode == OP_SYSTEM);
    assign stop_event = en && (dec_illegal || dec_sys || pc_mis);

    always @(posedge clk) begin
        if (rst) begin
            illegal_flag <= 1'b0;
            mis_flag     <= 1'b0;
            sys_flag     <= 1'b0;
        end else if (en) begin
            if (dec_illegal) illegal_flag <= 1'b1;
            if (pc_mis)      mis_flag     <= 1'b1;
            if (dec_sys)     sys_flag     <= 1'b1;
        end
    end

endmodule
