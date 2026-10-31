//====================================================================
// imm_gen.v  (RISC-V RV32I single-cycle CPU - immediate generator)
//
//   New module - MIPS only ever had one 16-bit sign-extended
//   immediate field (instr[15:0]), so there was nothing to select.
//   RV32I scatters its immediate bits across five different
//   instruction formats, so this module's whole job is picking the
//   right bits out of the right places and sign-extending the result
//   to 32 bits.
//
//   Field layout per format (see RISC-V Unprivileged ISA Spec,
//   Chapter 2, and Patterson & Hennessy COD RISC-V Ed., Ch.4):
//
//     I-type (addi, lw, jalr, ...):
//       imm[11:0] = instr[31:20]
//
//     S-type (sw, sb, sh):
//       imm[11:5] = instr[31:25]   imm[4:0] = instr[11:7]
//
//     B-type (beq, bne, blt, bge, bltu, bgeu):
//       imm[12]   = instr[31]      imm[11]  = instr[7]
//       imm[10:5] = instr[30:25]   imm[4:1] = instr[11:8]
//       imm[0]    = 0  (branch targets are always 2-byte aligned,
//                        so the encoding never stores this bit)
//
//     U-type (lui, auipc):
//       imm[31:12] = instr[31:12]  imm[11:0] = 0
//       (this one is NOT sign-extended in the usual sense - the
//        upper 20 bits ARE the value, already left-shifted by the
//        hardware's fixed wiring, not by a shifter)
//
//     J-type (jal):
//       imm[20]    = instr[31]     imm[19:12] = instr[19:12]
//       imm[11]    = instr[20]     imm[10:1]  = instr[30:21]
//       imm[0]     = 0  (jump targets are always 2-byte aligned)
//
//   Why B-type and J-type look scrambled: the encoding was chosen so
//   that as many bit positions as possible line up across formats
//   (e.g. imm[31] / the sign bit is always instr[31], funct7 and
//   the top immediate bits share position with R-type), which
//   shrinks the decode hardware everywhere else at the cost of this
//   one module looking odd. Textbook figures usually draw this as a
//   dedicated "immediate generator" for exactly this reason - it is
//   meant to absorb all the odd wiring in one place.
//
//   imm_src selects the format. Encoding chosen to match how the
//   control unit will decode opcode[6:0] (values are the CPU
//   designer's own choice - RISC-V does not standardize this
//   signal, only the bit layouts above):
//
//       imm_src   format   example instructions
//       -------   ------   --------------------
//       3'b000    I-type   addi, andi, ori, xori, slti, sltiu,
//                          slli, srli, srai, lw, jalr
//       3'b001    S-type   sw, sh, sb
//       3'b010    B-type   beq, bne, blt, bge, bltu, bgeu
//       3'b011    U-type   lui, auipc
//       3'b100    J-type   jal
//       (R-type instructions don't use this module at all - reg_write
//        data for those comes straight from the ALU/register file,
//        so imm_src is a don't-care and imm_out is unused downstream)
//====================================================================

module imm_gen (
    input  [31:0] instruction,
    input  [2:0]  imm_src,
    output reg [31:0] imm_out
);

    always @(*) begin
        case (imm_src)
            3'b000: // I-type
                imm_out = {{20{instruction[31]}}, instruction[31:20]};

            3'b001: // S-type
                imm_out = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};

            3'b010: // B-type
                imm_out = {{19{instruction[31]}}, instruction[31], instruction[7],
                           instruction[30:25], instruction[11:8], 1'b0};

            3'b011: // U-type - top 20 bits pass through, bottom 12 are 0.
                    // No sign-extension needed: the field already occupies
                    // the CPU's full upper half.
                imm_out = {instruction[31:12], 12'b0};

            3'b100: // J-type
                imm_out = {{11{instruction[31]}}, instruction[31], instruction[19:12],
                           instruction[20], instruction[30:21], 1'b0};

            default:
                imm_out = 32'b0;
        endcase
    end

endmodule
