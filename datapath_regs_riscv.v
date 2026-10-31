//====================================================================
// datapath_regs_riscv.v   (RISC-V RV32I multi-cycle CPU - step 2)
//
//   Reference: Harris & Harris, "Digital Design and Computer
//   Architecture: RISC-V Edition", Ch. 7 (multicycle processor).
//   See multicycle_spec.md Sections 2.2 and 2.3.
//
//   Contains:
//     - the non-architectural registers   OldPC, A, B, ALUOut
//     - the three datapath muxes          SrcA, SrcB, Result  (Result: 11 = RV32M unit result)
//
//   NOT in this module (added in later steps):
//     - PC register and PC-update / JalrMask logic   (step 7, top)
//     - ALU                                           (step 3)
//     - regfile, imm_gen, memories, FSM               (other steps)
//
//   Global freeze: when 'en' = 0 every register holds its value.
//   (en = started, so nothing moves before the run button is pressed.)
//
//   Register behaviour (spec D6):
//     A, B, ALUOut : free-running, loaded every enabled cycle
//     OldPC        : loaded only when 'fetch' = 1 (FETCH state)
//====================================================================
module datapath_regs_riscv #(
    parameter RF_SYNC = 1       // 1: rd1/rd2 are the synchronous M9K register-file outputs, used directly as A and B
                                //    0: A and B are free-running registers (asynchronous register file)
) (
    input         clk,
    input         rst,          // synchronous, active-high; clears registers
    input         en,           // global enable (step_enable / started)

    // ---- data in --------------------------------------------------
    input  [31:0] pc,           // current PC
    input  [31:0] rd1,          // regfile read port 1 (rs1)
    input  [31:0] rd2,          // regfile read port 2 (rs2)
    input  [31:0] imm_ext,      // from imm_gen
    input  [31:0] alu_result,   // from ALU (combinational)
    input  [31:0] load_data,    // aligned/extended load data (MemWB)
    input  [31:0] md_result,    // RV32M multiply/divide result (MD_WB)

    // ---- control in -----------------------------------------------
    input         fetch,        // 1 in FETCH: OldPC <= PC
    input  [1:0]  alu_src_a,    // 00 PC | 01 OldPC | 10 A | 11 zero
    input  [1:0]  alu_src_b,    // 00 B  | 01 ImmExt | 10 four | 11 (unused, 0)
    input  [1:0]  result_src,   // 00 ALUOut | 01 LoadData | 10 ALUResult | 11 MulDivResult

    // ---- register outputs -----------------------------------------
    output reg [31:0] old_pc,
    output [31:0] a,
    output [31:0] b,
    output reg [31:0] alu_out,

    // ---- mux outputs ----------------------------------------------
    output reg [31:0] src_a,    // ALU operand A
    output reg [31:0] src_b,    // ALU operand B
    output reg [31:0] result    // regfile write data / PC next source
);

    // ---------------- registers ----------------
    reg [31:0] a_q, b_q;                          // only used when RF_SYNC = 0
    assign a = RF_SYNC ? rd1 : a_q;
    assign b = RF_SYNC ? rd2 : b_q;

    always @(posedge clk) begin
        if (rst) begin
            old_pc  <= 32'd0;
            a_q     <= 32'd0;
            b_q     <= 32'd0;
            alu_out <= 32'd0;
        end else if (en) begin
            a_q     <= rd1;
            b_q     <= rd2;
            alu_out <= alu_result;
            if (fetch)
                old_pc <= pc;
        end
    end

    // ---------------- muxes ----------------
    always @(*) begin
        case (alu_src_a)
            2'b00:   src_a = pc;
            2'b01:   src_a = old_pc;
            2'b10:   src_a = a;
            default: src_a = 32'd0;
        endcase
    end

    always @(*) begin
        case (alu_src_b)
            2'b00:   src_b = b;
            2'b01:   src_b = imm_ext;
            2'b10:   src_b = 32'd4;
            default: src_b = 32'd0;
        endcase
    end

    always @(*) begin
        case (result_src)
            2'b00:   result = alu_out;
            2'b01:   result = load_data;
            2'b10:   result = alu_result;
            default: result = md_result;     // 2'b11
        endcase
    end

endmodule
