//====================================================================
// alu_control_riscv.v   (RISC-V RV32I multi-cycle CPU - step 3)
//
//   Reference: [HH] Harris & Harris, "Digital Design and Computer
//   Architecture, RISC-V Edition", Section 7.4 (ALU decoder), extended to
//   the full RV32I set.
//
//   Two small combinational modules. The ALU itself (alu_riscv.v +
//   ALU_32_bit.v) is reused UNCHANGED from the single-cycle project;
//   its alu_ctrl encoding is kept so both CPUs share one ALU.
//
//   1) alu_decoder_riscv : ALUOp (from the FSM) + funct3 + instr[30]
//                          -> alu_ctrl for alu_riscv
//   2) branch_cond_riscv : funct3 + ALU flags -> branch 'taken'
//
//   ALUOp encoding = [HH] Section 7.4 (same as the [HH] single-cycle ALU decoder):
//       2'b00  add
//       2'b01  branch compare (beq/bne: sub, blt/bge: slt, bltu/bgeu: sltu)
//       2'b10  operation from funct3 (R-type and I-type); SUB only when
//              op[5] & funct7[5] (RtypeSub), SRA when funct7[5] (srai has instr[30]=1)
//====================================================================

module alu_decoder_riscv (
    input      [1:0] alu_op,
    input      [2:0] funct3,
    input            funct7b5,      // instr[30]
    input            op5,           // instr[5]: 1 = R-type, 0 = I-type ([HH] RtypeSub = funct7b5 & op5)
    output reg [3:0] alu_ctrl
);
    // ---- must match alu_riscv.v exactly ----
    localparam ALU_ADD  = 4'b0000;
    localparam ALU_SUB  = 4'b0001;
    localparam ALU_AND  = 4'b0010;
    localparam ALU_OR   = 4'b0011;
    localparam ALU_XOR  = 4'b0100;
    localparam ALU_SLL  = 4'b0101;
    localparam ALU_SRL  = 4'b0110;
    localparam ALU_SRA  = 4'b0111;
    localparam ALU_SLT  = 4'b1000;
    localparam ALU_SLTU = 4'b1001;

    wire rtype_sub = funct7b5 & op5;    // [HH]: sub only for R-type (addi with a negative immediate stays add)

    always @(*) begin
        case (alu_op)
            2'b00: alu_ctrl = ALU_ADD;                    // [HH] 00: lw, sw, PC + 4, targets
            2'b01: begin                                  // [HH] 01: beq (extended to all branches)
                case (funct3)
                    3'b100, 3'b101: alu_ctrl = ALU_SLT;   // blt, bge
                    3'b110, 3'b111: alu_ctrl = ALU_SLTU;  // bltu, bgeu
                    default:        alu_ctrl = ALU_SUB;   // beq, bne
                endcase
            end
            default: begin                                // [HH] 10: R-type and I-type ALU, from funct3
                case (funct3)
                    3'b000:  alu_ctrl = rtype_sub ? ALU_SUB : ALU_ADD;   // add / sub / addi
                    3'b001:  alu_ctrl = ALU_SLL;                         // sll / slli
                    3'b010:  alu_ctrl = ALU_SLT;                         // slt / slti
                    3'b011:  alu_ctrl = ALU_SLTU;                        // sltu / sltiu
                    3'b100:  alu_ctrl = ALU_XOR;                         // xor / xori
                    3'b101:  alu_ctrl = funct7b5 ? ALU_SRA : ALU_SRL;    // sra(i) / srl(i)
                    3'b110:  alu_ctrl = ALU_OR;                          // or / ori
                    default: alu_ctrl = ALU_AND;                         // and / andi
                endcase
            end
        endcase
    end
endmodule


//====================================================================
// branch_cond_riscv: is the branch condition true?
//
//   funct3  inst   ALU op   test
//   000     beq    SUB      zero
//   001     bne    SUB      ~zero
//   100     blt    SLT      result[0]
//   101     bge    SLT      ~result[0]
//   110     bltu   SLTU     result[0]
//   111     bgeu   SLTU     ~result[0]
//
//   funct3[2] picks zero vs. result[0]; funct3[0] inverts.
//   The FSM combines it as:  PCUpdate = PCWrite | (Branch & taken)
//   funct3 = 010 / 011 are not branches (decode marks them illegal).
//====================================================================
module branch_cond_riscv (
    input  [2:0] funct3,
    input        zero_flag,       // from alu_riscv
    input        alu_result_lsb,  // alu_result[0]
    output       taken
);
    assign taken = (funct3[2] ? alu_result_lsb : zero_flag) ^ funct3[0];
endmodule
