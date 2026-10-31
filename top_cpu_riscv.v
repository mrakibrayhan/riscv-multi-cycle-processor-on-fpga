//====================================================================
// top_cpu_riscv.v  (board top-level - RZ-EasyFPGA A2.2 / Cyclone IV EP4CE6)
//
//   RV32IM MULTI-CYCLE CPU (Harris & Harris, RISC-V Edition, Section 7.4) with SYNCHRONOUS-read
//   memories in M9K blocks: instruction ROM (128 words, 1 M9K), byte-enable data RAM (32 words,
//   1 M9K) and register file (2 M9K copies). Same pins, buttons, LEDs, snapshot RAMs and ISSP
//   layout as the ARM boards (top_cpu_arm_p.v), so the same board procedure is used.
//
//   Buttons: KEY[3] = reset (the CPU is also held in reset until the first press of KEY[2]); KEY[2] = run.
//            The reset must last >= 32 CPU cycles (register-file clear): the button filter and the
//            "held until KEY[2]" rule make it milliseconds long.
//   LEDG (active low = lit): [0] fault   [1] started   [2] last branch taken   [3] halted.
//
//   Stop rules (same as the ARM cores):
//     halted = ecall / ebreak, or a jump / branch to its own address ("j ." , "beq x2,x2,done" - the ARM "B .")
//     fault  = illegal instruction, or a taken jump / branch to a target that is not 4-byte aligned
//   PC after a stop = the address of the stopping instruction (RISC-V rule: mepc = that instruction).
//
//   SNAPSHOT (snapshot_unit.v): on halt / fault, or when the ISSP source bit is 1, the CPU stops at the
//   next instruction boundary and x0..x31 and the 32 RAM words are copied into four RAMs read with the
//   In-System Memory Content Editor:  R00 = x0..x15   R16 = x16..x31   D00 = RAM 0..15   D16 = RAM 16..31
//   (RISC-V has 32 registers, so the ARM "R16 = status words" are not available; the status is in the ISSP).
//   The four RAMs are cleared while the CPU is in reset.
//
//   ISSP (altsource_probe, instance "RVMC"): source[0] = manual snapshot request;
//     probe (125 bits, LSB first) - same field positions as the ARM boards:
//         [0] started  [1] halted  [2] fault  [3] snapshot done
//         [7:4]     {0, misaligned target, illegal instruction, last branch taken}   (ARM: NZCV)
//         [23:8]    PC (low 16 bits) = address of the current instruction (after a stop: the stopping one)
//         [55:24]   instruction word
//         [87:56]   CYCLE COUNTER       CPU clock cycles while running, up to and including the stop cycle
//         [119:88]  INSTRUCTION COUNTER instructions fetched (the stopping instruction included)
//         [124:120] FSM state number (15 = HALT)                                          (ARM: pipe status)
//       CPI = cycle counter / instruction counter.
//   RAM starting values come from data.hex (32 lines, line 1 = word 0), loaded when the FPGA is programmed.
//   The reset (KEY3) does not reload or clear the RAM: program the FPGA again to restore the data.hex values.
//   Data addresses: only addr[6:2] selects the word (the RAM is repeated through the address space).
//====================================================================
module top_cpu_riscv #(
    parameter ENABLE_M  = 1,                 // 1: RV32IM, 0: RV32I
    parameter IMEM_AW   = 7,                 // 128 instruction words (1 M9K), same as the ARM boards
    parameter DMEM_AW   = 5,                 // 32 data words (1 M9K with byte enables)
    parameter INIT_FILE = "program.hex",
    parameter DATA_FILE = "data.hex",
    parameter DEBOUNCE  = 250000             // button filter length in clock cycles (the testbench makes it short)
) (
    input        clk,      // PIN_23
    input  [4:0] KEY,      // PIN_88,89,90,91,25 (only KEY[3], KEY[2] used)
    output [7:0] SEG,      // PIN_128,121,125,129,132,126,124,127
    output [3:0] DIG,      // PIN_133,135,136,137
    output [3:0] LEDG      // PIN_87,86,85,84
);
    //----------------------------------------------------------------
    // Clock divider: 50 MHz / 4 = 12.5 MHz for the CPU core
    //----------------------------------------------------------------
    reg [1:0] clkdiv = 2'b00;
    always @(posedge clk) clkdiv <= clkdiv + 2'b01;
    wire cpu_clk = clkdiv[1];

    //----------------------------------------------------------------
    // Buttons: KEY[3] reset (level), KEY[2] run (one pulse)
    //----------------------------------------------------------------
    wire key3_raw = ~KEY[3];
    wire rst_debounced;
    debounce #(.DEBOUNCE_LIMIT(DEBOUNCE)) db_rst (.clk(clk), .btn_in(key3_raw), .btn_out(rst_debounced));

    wire key2_raw = ~KEY[2];
    wire key2_debounced, step_pulse;
    debounce #(.DEBOUNCE_LIMIT(DEBOUNCE)) db_step (.clk(cpu_clk), .btn_in(key2_raw), .btn_out(key2_debounced));
    edge_detector ed_step (.clk(cpu_clk), .signal_in(key2_debounced), .pulse_out(step_pulse));

    reg started = 1'b0;
    always @(posedge cpu_clk) begin
        if (rst_debounced)   started <= 1'b0;
        else if (step_pulse) started <= 1'b1;
    end

    //----------------------------------------------------------------
    // CPU (held in reset until 'started'; frozen while a snapshot runs)
    //----------------------------------------------------------------
    wire        cpu_rst = rst_debounced | ~started;
    wire        snap_freeze, snap_override, snap_src, snap_done;
    wire [5:0]  snap_idx;
    wire [31:0] snap_rf_data, snap_dm_data;
    wire        en_cpu = started & ~snap_freeze;

    wire [31:0] pc_debug, cur_instr;
    wire [4:0]  state_debug;
    wire        fetch_now, br_eval, br_taken, halted, fault, stop_now, ill_flag, mis_flag;

    cpu_top_riscv #(.ENABLE_M(ENABLE_M), .IMEM_AW(IMEM_AW), .DMEM_AW(DMEM_AW),
                    .INIT_FILE(INIT_FILE), .DATA_FILE(DATA_FILE)) mycpu (
        .clk(cpu_clk), .rst(cpu_rst), .step_enable(en_cpu),
        .snap_override(snap_override), .snap_idx(snap_idx),
        .snap_rf_data(snap_rf_data), .snap_dm_data(snap_dm_data),
        .debug_pc(), .debug_oldpc(pc_debug), .debug_instruction(cur_instr), .debug_state(state_debug),
        .debug_fetch(fetch_now), .debug_branch_eval(br_eval), .debug_branch_taken(br_taken),
        .halted(halted), .fault(fault), .stop_now(stop_now),
        .debug_illegal_flag(ill_flag), .debug_mis_flag(mis_flag),
        .tr_regw(), .tr_wa(), .tr_wd(), .tr_memw(), .tr_adr(), .tr_mdata(), .tr_f3()
    );

    //----------------------------------------------------------------
    // Status (cpu_clk, cleared by the CPU reset)
    //----------------------------------------------------------------
    localparam [4:0] S_FETCH = 5'd0, S_HALT = 5'd15;

    reg branch_taken_flag = 1'b0;                        // outcome of the last executed branch
    always @(posedge cpu_clk) begin
        if (cpu_rst)      branch_taken_flag <= 1'b0;
        else if (br_eval) branch_taken_flag <= br_taken;
    end

    // counters: count while the CPU runs (not frozen for a snapshot), stop after the stop cycle
    reg [31:0] cycle_count = 32'd0, instr_count = 32'd0;
    wire running = en_cpu & ~halted & ~fault;
    always @(posedge cpu_clk) begin
        if (cpu_rst) begin
            cycle_count <= 32'd0;
            instr_count <= 32'd0;
        end else if (running) begin
            cycle_count <= cycle_count + 32'd1;
            if (fetch_now) instr_count <= instr_count + 32'd1;
        end
    end

    //----------------------------------------------------------------
    // Snapshot unit + the four In-System Memory Content Editor RAMs
    //----------------------------------------------------------------
    snapshot_unit snap_unit (
        .clk          (cpu_clk),
        .rst          (cpu_rst),
        .started      (started),
        .trigger_auto (halted | fault),
        .trigger_src  (snap_src),
        .at_boundary  (state_debug == S_FETCH || state_debug == S_HALT),
        .rf_data      (snap_rf_data),
        .dm_data      (snap_dm_data),
        .snap_freeze  (snap_freeze),
        .snap_override(snap_override),
        .snap_idx     (snap_idx),
        .snap_done    (snap_done)
    );

    //----------------------------------------------------------------
    // 7-seg display unused
    //----------------------------------------------------------------
    assign SEG = 8'hFF;
    assign DIG = 4'hF;

    // LEDs (active low)
    assign LEDG[0] = ~fault;
    assign LEDG[1] = ~started;
    assign LEDG[2] = ~branch_taken_flag;
    assign LEDG[3] = ~halted;

    //----------------------------------------------------------------
    // ISSP: source = manual snapshot request, probe = status + counters (same layout as the ARM boards)
    //----------------------------------------------------------------
    wire [3:0]   cause = {1'b0, mis_flag, ill_flag, branch_taken_flag};
    wire [124:0] probe_status = {state_debug, instr_count, cycle_count, cur_instr, pc_debug[15:0], cause,
                                 snap_done, fault, halted, started};
    altsource_probe #(
        .lpm_type                ("altsource_probe"),
        .lpm_hint                ("UNUSED"),
        .sld_auto_instance_index ("YES"),
        .sld_instance_index      (0),
        .SLD_NODE_INFO           (4746752),
        .sld_ir_width            (4),
        .instance_id             ("RVMC"),
        .probe_width             (125),
        .source_width            (1),
        .source_initial_value    ("0"),
        .enable_metastability    ("NO")
    ) inst_issp (
        .probe      (probe_status),
        .source     (snap_src),
        .source_clk (cpu_clk),
        .source_ena (1'b1)
    );
endmodule
