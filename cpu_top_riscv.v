//====================================================================
// cpu_top_riscv.v   (RV32IM multi-cycle CPU core)
//
//   Reference: [HH] Harris & Harris, "Digital Design and Computer Architecture, RISC-V
//   Edition", Section 7.4, Fig. 7.27 (multicycle datapath + controller).
//   ISA: [RV] RISC-V Unprivileged ISA, RV32I + M.
//
//     control_fsm_riscv   main FSM + instruction legality       ([HH] Main FSM / Instr Decoder)
//     alu_decoder_riscv   ALUOp + funct3 + op[5] + funct7[5]     ([HH] ALU Decoder)
//     branch_cond_riscv   branch taken (all six branches)
//     datapath_regs_riscv OldPC, ALUOut + SrcA/SrcB/Result muxes
//     alu_riscv           ALU
//     imem_riscv          instruction memory; its output register = [HH] Instr register (IRWrite = fetch)
//     dmem_riscv          data memory; its read register = [HH] Data register
//     regfile_riscv       register file; its two synchronous outputs = [HH] A and B registers
//     imm_gen             [HH] Extend unit
//     muldiv_riscv        RV32M iterative unit (ENABLE_M)
//
//   Mapping onto M9K (same method as the ARM cores): the [HH] non-architectural registers
//   Instr, Data, A and B are the M9K input/output registers, so no extra flip-flops are used.
//   Deliberate difference from Fig. 7.27: separate instruction ROM and data RAM (Harvard),
//   like the ARM cores, instead of one shared memory with the AdrSrc mux. The FSM states and
//   cycle counts of the [HH] instructions are unchanged.
//
//   PC <= Result when PCUpdate | (Branch & taken), bit 0 cleared for jalr ([RV]).
//   A jump/branch to a target with bit 1 set is not taken: the FSM stops with fault.
//
//   Stop (halted / fault) for the board and the testbench:
//     halted = ecall / ebreak, or a jump/branch to its own address (j . , beq x,x,.) -
//              the "B ." rule of the ARM cores. fault = illegal instruction or misaligned target.
//====================================================================
module cpu_top_riscv #(
    parameter ENABLE_M  = 1,             // 1: RV32M, 0: M encodings are illegal and the unit is removed, 2: Zmmul (multiply only, no divider; Step E)
    parameter MUL_FAST  = 0,             // 0: serial multiply (board default); 1: multiply on the embedded multipliers (3 cycles), see muldiv_riscv.v
    parameter IMEM_AW   = 7,             // instruction ROM: 2**IMEM_AW words (7 = 128 words = 1 M9K)
    parameter DMEM_AW   = 5,             // data RAM: 2**DMEM_AW words (5 = 32 words)
    parameter INIT_FILE = "program.hex",
    parameter DATA_FILE = "data.hex"
) (
    input        clk,
    input        rst,
    input        step_enable,

    // snapshot interface (see snapshot_unit.v): while snap_override is high, register-file read
    // port 1 and the data-memory read address are driven by snap_idx. Data comes back one cycle
    // later on snap_rf_data (register x(idx)) and snap_dm_data (data word idx).
    input         snap_override,
    input  [5:0]  snap_idx,
    output [31:0] snap_rf_data,
    output [31:0] snap_dm_data,

    output [31:0] debug_pc,              // PC register
    output [31:0] debug_oldpc,           // address of the current instruction
    output [31:0] debug_instruction,
    output [4:0]  debug_state,
    output        debug_fetch,           // 1 in an enabled FETCH cycle (instruction counter)
    output        debug_branch_eval,     // 1 in an enabled BRANCH cycle
    output        debug_branch_taken,    // branch condition (valid with debug_branch_eval)
    output        halted,                // sticky: ecall / ebreak / self-loop
    output        fault,                 // sticky: illegal instruction or misaligned target
    output        stop_now,              // 1 for the enabled cycle that sets halted or fault
    output        debug_illegal_flag,
    output        debug_mis_flag,

    // trace (testbench only)
    output        tr_regw,
    output [4:0]  tr_wa,
    output [31:0] tr_wd,
    output        tr_memw,
    output [31:0] tr_adr,
    output [31:0] tr_mdata,
    output [2:0]  tr_f3
);

    //----------------------------------------------------------------
    // Signals
    //----------------------------------------------------------------
    reg  [31:0] pc;

    wire [31:0] instruction;
    wire [4:0]  rd_addr  = instruction[11:7];
    wire [2:0]  funct3   = instruction[14:12];
    wire [4:0]  rs1_addr = instruction[19:15];
    wire [4:0]  rs2_addr = instruction[24:20];
    wire        funct7b5 = instruction[30];
    wire        op5      = instruction[5];

    wire [1:0] alu_src_a, alu_src_b, alu_op, result_src;
    wire [2:0] imm_src;
    wire       pc_write, branch, reg_write, mem_read, mem_write, fetch, jalr_mask;
    wire       stop_event, illegal_flag, mis_flag, sys_flag;
    wire [4:0] state;
    wire       md_start, md_done;
    wire [31:0] md_result;

    wire [31:0] imm_ext, rd1, rd2, src_a, src_b, result, alu_result, load_data;
    wire [31:0] old_pc, a_reg, b_reg, alu_out;
    wire [3:0]  alu_ctrl;
    wire        alu_zero, taken;

    //----------------------------------------------------------------
    // Controller
    //----------------------------------------------------------------
    control_fsm_riscv #(.ENABLE_M(ENABLE_M)) fsm (
        .clk(clk), .rst(rst), .en(step_enable), .instr(instruction), .md_done(md_done),
        .taken(taken), .tgt_bit1(result[1]),
        .alu_src_a(alu_src_a), .alu_src_b(alu_src_b), .alu_op(alu_op),
        .result_src(result_src), .imm_src(imm_src),
        .pc_write(pc_write), .branch(branch), .reg_write(reg_write),
        .mem_read(mem_read), .mem_write(mem_write), .fetch(fetch),
        .jalr_mask(jalr_mask), .md_start(md_start),
        .stop_event(stop_event), .illegal_flag(illegal_flag), .mis_flag(mis_flag), .sys_flag(sys_flag),
        .state_out(state)
    );

    //----------------------------------------------------------------
    // Instruction memory (output register = [HH] Instr, IRWrite = fetch)
    //----------------------------------------------------------------
    imem_riscv #(.ADDR_BITS(IMEM_AW), .INIT_FILE(INIT_FILE)) instr_mem (
        .clk(clk), .fetch_en(fetch), .pc(pc), .instruction(instruction)
    );

    imm_gen immgen (.instruction(instruction), .imm_src(imm_src), .imm_out(imm_ext));

    //----------------------------------------------------------------
    // Register file: outputs = [HH] A / B, Result -> rd
    //----------------------------------------------------------------
    wire [4:0] rf_ra1 = snap_override ? snap_idx[4:0] : rs1_addr;
    regfile_riscv rf (
        .clk(clk), .rst(rst),
        .read_reg1(rf_ra1), .read_reg2(rs2_addr),
        .write_reg(rd_addr), .write_data(result), .write_enable(reg_write),
        .read_data1(rd1), .read_data2(rd2)
    );

    //----------------------------------------------------------------
    // OldPC, ALUOut and the datapath muxes
    //----------------------------------------------------------------
    datapath_regs_riscv #(.RF_SYNC(1)) dpr (
        .clk(clk), .rst(rst), .en(step_enable),
        .pc(pc), .rd1(rd1), .rd2(rd2), .imm_ext(imm_ext),
        .alu_result(alu_result), .load_data(load_data), .md_result(md_result),
        .fetch(fetch), .alu_src_a(alu_src_a), .alu_src_b(alu_src_b), .result_src(result_src),
        .old_pc(old_pc), .a(a_reg), .b(b_reg), .alu_out(alu_out),
        .src_a(src_a), .src_b(src_b), .result(result)
    );

    //----------------------------------------------------------------
    // RV32M unit
    //----------------------------------------------------------------
    generate
        if (ENABLE_M) begin : g_muldiv
            muldiv_riscv #(.MUL_FAST(MUL_FAST), .HAS_DIV(ENABLE_M == 1)) mdu (
                .clk(clk), .rst(rst), .en(step_enable), .start(md_start),
                .funct3(funct3), .a(a_reg), .b(b_reg), .done(md_done), .result(md_result)
            );
        end else begin : g_no_muldiv
            assign md_done   = 1'b0;
            assign md_result = 32'd0;
        end
    endgenerate

    //----------------------------------------------------------------
    // ALU + ALU decoder + branch condition
    //----------------------------------------------------------------
    alu_decoder_riscv aludec (
        .alu_op(alu_op), .funct3(funct3), .funct7b5(funct7b5), .op5(op5), .alu_ctrl(alu_ctrl)
    );

    alu_riscv alu (
        .A(src_a), .B(src_b), .alu_ctrl(alu_ctrl),
        .alu_result(alu_result), .zero_flag(alu_zero)
    );

    branch_cond_riscv bcond (
        .funct3(funct3), .zero_flag(alu_zero), .alu_result_lsb(alu_result[0]), .taken(taken)
    );

    //----------------------------------------------------------------
    // PC register ([HH] PCEn = PCUpdate | Branch & Zero, here Branch & taken)
    //----------------------------------------------------------------
    wire        pc_update = pc_write | (branch & taken);
    wire [31:0] pc_next   = {result[31:1], result[0] & ~jalr_mask};

    always @(posedge clk) begin
        if (rst)
            pc <= 32'd0;
        else if (pc_update)
            pc <= pc_next;
    end

    //----------------------------------------------------------------
    // Data memory (read register = [HH] Data). addr = ALUOut, write data = B
    //----------------------------------------------------------------
    wire [31:0] dm_addr   = snap_override ? {25'd0, snap_idx[4:0], 2'b00} : alu_out;
    wire [2:0]  dm_funct3 = snap_override ? 3'b010 : funct3;
    dmem_riscv_mc #(.ADDR_BITS(DMEM_AW), .DATA_FILE(DATA_FILE)) dmem (
        .clk(clk), .addr(dm_addr), .write_data(b_reg), .funct3(dm_funct3),
        .mem_read(mem_read | snap_override), .mem_write(mem_write), .read_data(load_data)
    );
    assign snap_rf_data = rd1;
    assign snap_dm_data = load_data;

    //----------------------------------------------------------------
    // Stop status: halted (normal end) and fault
    //   self-loop = a jump / branch whose new PC is its own address (not in FETCH)
    //----------------------------------------------------------------
    wire self_loop = pc_update & ~fetch & (pc_next == old_pc);
    reg  loop_flag;
    always @(posedge clk) begin
        if (rst)            loop_flag <= 1'b0;
        else if (self_loop) loop_flag <= 1'b1;
    end
    assign halted   = sys_flag | loop_flag;
    assign fault    = illegal_flag | mis_flag;
    assign stop_now = stop_event | self_loop;      // self_loop is already gated by step_enable

    //----------------------------------------------------------------
    // Debug / trace
    //----------------------------------------------------------------
    assign debug_pc           = pc;
    assign debug_oldpc        = old_pc;
    assign debug_instruction  = instruction;
    assign debug_state        = state;
    assign debug_fetch        = fetch;
    assign debug_branch_eval  = step_enable & (state == 5'd9);
    assign debug_branch_taken = taken;
    assign debug_illegal_flag = illegal_flag;
    assign debug_mis_flag     = mis_flag;

    assign tr_regw  = reg_write & (rd_addr != 5'd0);
    assign tr_wa    = rd_addr;
    assign tr_wd    = result;
    assign tr_memw  = mem_write;
    assign tr_adr   = alu_out;
    assign tr_mdata = b_reg;
    assign tr_f3    = funct3;

endmodule
