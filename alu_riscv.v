//====================================================================
// alu_riscv.v  (RISC-V ALU, the SAME file in the single-cycle, multi-cycle and pipelined cores)
//
//   Reference: Harris & Harris, Digital Design and Computer Architecture,
//   RISC-V Edition [HH-RV], Section 5.2.4 (ALU) and Section 7.3 (single-cycle
//   processor) - the same textbook family as the ARM project (ARM Edition).
//
//   CHANGE (textbook ALU): the earlier version wrapped ALU_32_bit.v, a
//   gate-level ripple-carry chain of 1-bit slices (Patterson & Hennessy
//   style). [HH-RV] builds the ALU around ONE shared adder written with the
//   "+" operator:
//        Sum = A + (SUB ? ~B : B) + SUB
//   and derives SUB / SLT / SLTU from it. Quartus maps "+" to the dedicated
//   carry chain, exactly like alu_arm.v of the ARM project, so the two ISAs
//   are now compared with the same adder structure (2026-10-24: also used in the
//   multi-cycle and pipelined RISC-V cores instead of ALU_32_bit.v). Result: fewer logic
//   elements and a much shorter critical path. ALU_32_bit.v is no longer used.
//
//   Ports and the alu_ctrl encoding are UNCHANGED (drop-in replacement):
//       alu_ctrl   operation   RV32I instructions
//       4'b0000    ADD         add, addi, loads, stores, jalr, auipc
//       4'b0001    SUB         sub, beq, bne
//       4'b0010    AND         and, andi
//       4'b0011    OR          or, ori
//       4'b0100    XOR         xor, xori
//       4'b0101    SLL         sll, slli
//       4'b0110    SRL         srl, srli
//       4'b0111    SRA         sra, srai
//       4'b1000    SLT         slt, slti, blt, bge   (signed:   Sum[31] ^ V)
//       4'b1001    SLTU        sltu, sltiu, bltu, bgeu (unsigned: borrow = ~Cout)
//   zero_flag = (alu_result == 0)  ([HH-RV] "Zero").
//====================================================================

module alu_riscv (
    input  [31:0] A,
    input  [31:0] B,
    input  [3:0]  alu_ctrl,
    output reg [31:0] alu_result,
    output        zero_flag
);
    // ---------------- shared adder (SUB, SLT and SLTU subtract) ----------------
    wire        sub   = (alu_ctrl == 4'b0001) | (alu_ctrl == 4'b1000) | (alu_ctrl == 4'b1001);
    wire [31:0] condb = sub ? ~B : B;
    wire [32:0] sum33 = {1'b0, A} + {1'b0, condb} + {32'd0, sub};
    wire [31:0] sum   = sum33[31:0];
    wire        cout  = sum33[32];
    // signed overflow ([HH-RV] Fig. 5.17): operands of equal sign (after inversion), result of the other sign
    wire        v     = (A[31] == condb[31]) & (sum[31] != A[31]);
    wire        lt    = sum[31] ^ v;        // signed A < B
    wire        ltu   = ~cout;              // unsigned A < B (borrow)

    // ---------------- shifter (barrel shifter, shamt = B[4:0]) ----------------
    wire [31:0] sll = A << B[4:0];
    wire [31:0] srl = A >> B[4:0];
    wire [31:0] sra = $signed(A) >>> B[4:0];

    always @(*) begin
        case (alu_ctrl)
            4'b0000: alu_result = sum;                 // ADD
            4'b0001: alu_result = sum;                 // SUB
            4'b0010: alu_result = A & B;               // AND
            4'b0011: alu_result = A | B;               // OR
            4'b0100: alu_result = A ^ B;               // XOR
            4'b0101: alu_result = sll;                 // SLL
            4'b0110: alu_result = srl;                 // SRL
            4'b0111: alu_result = sra;                 // SRA
            4'b1000: alu_result = {31'b0, lt};         // SLT
            4'b1001: alu_result = {31'b0, ltu};        // SLTU
            default: alu_result = 32'b0;
        endcase
    end

    assign zero_flag = (alu_result == 32'b0);
endmodule
