//====================================================================
// muldiv_riscv.v   (RV32M extension for the RISC-V multi-cycle CPU)
//
//   Iterative multiply / divide unit: ONE 34-bit adder/subtractor used
//   every cycle, constant execution time, no DSP blocks, no block RAM.
//   Works on the operands held in the CPU's A and B registers (stable for
//   the whole instruction) and returns a 32-bit result.
//
//   funct3 (instr[14:12], funct7 = 0000001):
//     000 mul     low 32 bits of rs1 * rs2
//     001 mulh    high 32 bits, signed   * signed
//     010 mulhsu  high 32 bits, signed   * unsigned
//     011 mulhu   high 32 bits, unsigned * unsigned
//     100 div     signed   quotient   (rounds toward zero)
//     101 divu    unsigned quotient
//     110 rem     signed   remainder  (sign of the dividend)
//     111 remu    unsigned remainder
//   Special cases required by the RISC-V spec (no traps):
//     x / 0   -> quotient = all ones (-1),  remainder = x
//     -2^31 / -1 -> quotient = -2^31,       remainder = 0
//
//   Handshake with the CPU FSM:
//     FSM state MD_START : start = 1 for one cycle
//     FSM state MD_WAIT  : waits until done = 1 (level, stays 1 until the next start)
//     FSM state MD_WB    : result is written to rd
//   'en' freezes the unit (CPU run/pause), exactly like the rest of the CPU.
//
//   Multiply = right-shifting shift-and-add on {acc, lo}:
//       lo = multiplier, acc = 0; 32 times: if lo[0] then acc += M (or -= M on the
//       sign bit of a signed multiplier); arithmetic shift {acc, lo} right by 1.
//       M is a 33-bit value (sign- or zero-extended multiplicand), acc is 34 bits.
//   Divide = restoring division on magnitudes (negated first when signed),
//       then the quotient / remainder signs are fixed (negate = 0 - x on the same adder).
//
//   MUL_FAST = 1: the four multiplies use one 33x33 signed '*' (embedded multipliers) in 3 cycles
//   (operand regs, product reg, result reg); divide is unchanged.
//   Time from start to done (unit cycles): mul 33 (MUL_FAST: 2), div 37. See multicycle_spec.md.
//====================================================================
(* multstyle = "dsp" *)      // (only has an effect when MUL_FAST = 1: the one '*' below goes into the embedded 9x9/18x18 multipliers)
module muldiv_riscv #(
    parameter MUL_FAST = 0,     // 0: serial shift-and-add multiply (34 cycles, no DSP)   1: embedded-multiplier multiply (3 cycles)
    parameter HAS_DIV  = 1      // 0: Zmmul build (Step E, gap G5): no divider hardware (div/rem are illegal in the decoder)
) (
    input         clk,
    input         rst,
    input         en,
    input         start,
    input  [2:0]  funct3,
    input  [31:0] a,            // rs1
    input  [31:0] b,            // rs2
    output reg    done,
    output reg [31:0] result
);

    localparam IDLE  = 4'd0, RUN_M = 4'd1, FIN_M = 4'd2,
               ABS_A = 4'd3, ABS_B = 4'd4, RUN_D = 4'd5,
               FIX_Q = 4'd6, FIX_R = 4'd7, FIN_D = 4'd8,
               MUL1  = 4'd9, MUL2  = 4'd10;           // used only when MUL_FAST = 1

    reg signed [32:0] fa, fb;   // MUL_FAST operands (33-bit signed: sign- or zero-extended per funct3)
    reg signed [65:0] prod;     // MUL_FAST product

    reg [3:0]  state;
    reg [33:0] acc;             // accumulator / partial remainder (signed)
    reg [31:0] lo;              // multiplier -> product low / dividend -> quotient
    reg [32:0] m;               // multiplicand / divisor magnitude
    reg [4:0]  cnt;
    reg        bsgn;            // multiplier is signed (mulh)
    reg        neg_a, neg_b, dz;

    wire a_signed = (funct3 == 3'b001) || (funct3 == 3'b010);   // mulh, mulhsu
    wire b_signed = (funct3 == 3'b001);                         // mulh
    wire d_signed = ~funct3[0];                                 // div, rem

    // ---------------- the single adder / subtractor ----------------
    wire [33:0] acc_sh = {acc[32:0], lo[31]};                   // shifted left, for division

    reg [1:0] psel, qsel;       // P: 0 acc, 1 acc_sh, 2 zero      Q: 0 m, 1 lo, 2 acc
    reg       sub;
    always @(*) begin
        psel = 2'd2; qsel = 2'd0; sub = 1'b1;
        case (state)
            RUN_M: begin psel = 2'd0; qsel = 2'd0; sub = bsgn && (cnt == 5'd31); end
            RUN_D: begin psel = 2'd1; qsel = 2'd0; sub = 1'b1; end
            ABS_A,
            FIX_Q: begin psel = 2'd2; qsel = 2'd1; sub = 1'b1; end
            ABS_B: begin psel = 2'd2; qsel = 2'd0; sub = 1'b1; end
            FIX_R: begin psel = 2'd2; qsel = 2'd2; sub = 1'b1; end
            default: ;
        endcase
    end
    wire [33:0] P = (psel == 2'd0) ? acc : (psel == 2'd1) ? acc_sh : 34'd0;
    wire [33:0] Q = (qsel == 2'd0) ? {m[32], m} : (qsel == 2'd1) ? {2'b00, lo} : acc;
    wire [33:0] S = sub ? (P - Q) : (P + Q);

    wire [33:0] acc_n = lo[0] ? S : acc;                        // multiply step result

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            done  <= 1'b0;
        end else if (en) begin
            case (state)
                IDLE: if (start) begin
                    done <= 1'b0;
                    cnt  <= 5'd0;
                    acc  <= 34'd0;
                    if (!funct3[2] && MUL_FAST) begin           // multiply on the embedded multipliers
                        fa <= {a_signed & a[31], a};
                        fb <= {b_signed & b[31], b};
                        state <= MUL1;
                    end else if (!funct3[2]) begin              // multiply, serial
                        m    <= {a_signed & a[31], a};
                        lo   <= b;
                        bsgn <= b_signed;
                        state <= RUN_M;
                    end else if (!HAS_DIV) begin                // Zmmul: never reached (div/rem are illegal); finish at once
                        result <= 32'd0;
                        done   <= 1'b1;
                    end else begin                              // divide / remainder
                        lo    <= a;
                        m     <= {d_signed & b[31], b};          // sign-extended when signed, so 0 - m is the magnitude
                        neg_a <= d_signed & a[31];
                        neg_b <= d_signed & b[31];
                        dz    <= (b == 32'd0);
                        state <= ABS_A;
                    end
                end

                // ---------------- multiply, embedded multipliers (MUL_FAST) ----------------
                MUL1: begin prod <= fa * fb; state <= MUL2; end
                MUL2: begin
                    result <= (funct3 == 3'b000) ? prod[31:0] : prod[63:32];
                    done   <= 1'b1;
                    state  <= IDLE;
                end

                // ---------------- multiply, serial ----------------
                RUN_M: begin
                    acc  <= {acc_n[33], acc_n[33:1]};
                    lo   <= {acc_n[0], lo[31:1]};
                    cnt  <= cnt + 5'd1;
                    if (cnt == 5'd31) state <= FIN_M;
                end
                FIN_M: begin
                    result <= (funct3 == 3'b000) ? lo : acc[31:0];
                    done   <= 1'b1;
                    state  <= IDLE;
                end

                // ---------------- divide ----------------
                ABS_A: begin
                    if (neg_a) lo <= S[31:0];
                    state <= ABS_B;
                end
                ABS_B: begin
                    if (neg_b) m <= S[32:0];
                    state <= RUN_D;
                end
                RUN_D: begin
                    if (!S[33]) begin                           // trial subtract succeeded
                        acc <= S;
                        lo  <= {lo[30:0], 1'b1};
                    end else begin
                        acc <= acc_sh;
                        lo  <= {lo[30:0], 1'b0};
                    end
                    cnt <= cnt + 5'd1;
                    if (cnt == 5'd31) state <= FIX_Q;
                end
                FIX_Q: begin
                    if ((neg_a ^ neg_b) && !dz) lo <= S[31:0];  // negate quotient
                    state <= FIX_R;
                end
                FIX_R: begin
                    if (neg_a) acc <= S;                        // negate remainder
                    state <= FIN_D;
                end
                FIN_D: begin
                    result <= funct3[1] ? acc[31:0] : lo;
                    done   <= 1'b1;
                    state  <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule