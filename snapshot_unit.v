//====================================================================
// snapshot_unit.v  (multi-cycle CPU version of the single-cycle snapshot unit)
//
//   Copies all 32 registers + all 32 data words into FOUR small RAMs (16 words x 32 bit each) that are
//   read over JTAG with Quartus' In-System Memory Content Editor, in groups of 16:
//        R00 = x0..x15       R16 = x16..x31
//        D00 = mem[0..15]    D16 = mem[16..31]
//   (Each In-System-Memory-Editor RAM needs a second, JTAG, port, which makes it a TRUE dual-port M9K
//   and those are only 16 bits wide: a 32-bit RAM therefore takes 2 M9K blocks. Eight views would
//   need 16 blocks; with the 16-block instruction memory that does not fit the 30 blocks of the chip.)
//   This replaces the four 480-bit ISSP probes (about 1,900 logic cells) and, because the
//   views are now read from RAM, the CPU needs no flip-flop copy of its registers or memory.
//
//   WHEN: when the program halts (trigger_auto) or when the ISSP source bit is set
//   (trigger_src), the CPU is frozen at the next instruction boundary (at_boundary = the FSM
//   is in FETCH or HALT). idx then counts 0..63 and the CPU's read ports are driven by idx
//   (snap_override):  idx 0..31 -> register x(idx),  idx 32..63 -> data memory word (idx-32).
//   BOTH CPU read ports are synchronous, so the value for idx arrives ONE CYCLE LATER; the
//   write into the snapshot RAM uses the delayed index.
//
//   RESET CLEAR: while rst = 1 the four RAMs are cleared, so after KEY3 the Memory Content Editor
//   shows zeros, and after KEY2 the fresh snapshot of the new run (same as the ARM boards).
//
//   States: IDLE (CPU runs) -> SNAP (64 cycles) -> FLUSH (last write) -> DONE (CPU stays
//   frozen while the trigger is true; a halted program keeps it true) -> IDLE.
//
//   snap_freeze   = 1 in SNAP, FLUSH, DONE : the top forces the CPU's step_enable low
//   snap_override = 1 in SNAP and FLUSH    : cpu_top uses snap_idx as the read address
//
//   NOTE: freezing in FETCH means the FSM is in DECODE while frozen (the fetch has already
//   completed); the register-file RAM re-captures its read address at the end of the first
//   cycle after the freeze, so the CPU resumes with correct operands.
//====================================================================
module snapshot_unit (
    input         clk,           // cpu_clk
    input         rst,           // debounced reset (level)
    input         started,       // CPU has been started with KEY[2]
    input         trigger_auto,  // program halted (self-loop / ECALL / EBREAK) or fault
    input         trigger_src,   // manual request: ISSP source bit (asynchronous)
    input         at_boundary,   // FSM is in FETCH or HALT (no instruction in flight)
    input  [31:0] rf_data,       // register read data from cpu_top (valid one cycle after idx)
    input  [31:0] dm_data,       // data-memory read data from cpu_top (valid one cycle after idx)
    output        snap_freeze,
    output        snap_override,
    output [5:0]  snap_idx,
    output        snap_done      // 1 while the views are complete and the CPU is held (state DONE)
);

    // synchronize the manual trigger (it comes from the JTAG clock domain)
    reg s1 = 1'b0, s2 = 1'b0;
    always @(posedge clk) begin
        s1 <= trigger_src;
        s2 <= s1;
    end
    wire trigger = trigger_auto | s2;

    localparam IDLE = 2'd0, SNAP = 2'd1, FLUSH = 2'd2, DONE = 2'd3;
    reg [1:0] state = IDLE;
    reg [5:0] idx   = 6'd0;

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            idx   <= 6'd0;
        end else begin
            case (state)
                IDLE:  if (started && trigger && at_boundary) begin
                           state <= SNAP;
                           idx   <= 6'd0;
                       end
                SNAP:  if (idx == 6'd63) state <= FLUSH;
                       else              idx   <= idx + 6'd1;
                FLUSH: state <= DONE;
                DONE:  if (!trigger) state <= IDLE;
            endcase
        end
    end

    assign snap_freeze   = (state != IDLE);
    // override stays on during FLUSH: the value for the LAST index (63) is read in that cycle and the
    // data-memory read alignment (funct3, address bits) must still be the snapshot's 'lw word idx'
    assign snap_override = (state == SNAP) || (state == FLUSH);
    assign snap_idx      = idx;
    assign snap_done     = (state == DONE);

    // the value for idx arrives one cycle later: delay index and valid flag
    reg [5:0] idx_d = 6'd0;
    reg       v_d   = 1'b0;
    always @(posedge clk) begin
        idx_d <= idx;
        v_d   <= (state == SNAP);
    end

    // reset clear (same as the ARM snapshot unit): while rst = 1 all four RAMs are written with 0,
    // word by word (16 cycles; the CPU is held in reset until KEY2, so rst is long enough)
    reg  [3:0]  clr = 4'd0;
    always @(posedge clk) clr <= rst ? clr + 4'd1 : 4'd0;

    wire [31:0] snap_data = rst ? 32'd0 : (idx_d[5] ? dm_data : rf_data);
    wire        grp       = idx_d[4];                   // which group of 16
    wire [3:0]  word_in   = rst ? clr : idx_d[3:0];     // word inside the group

    wire [31:0] q_R00_unused;
    altsyncram #(
        .operation_mode                 ("SINGLE_PORT"),
        .width_a                        (32),
        .widthad_a                      (4),
        .numwords_a                     (16),
        .outdata_reg_a                  ("UNREGISTERED"),
        .clock_enable_input_a           ("BYPASS"),
        .clock_enable_output_a          ("BYPASS"),
        .power_up_uninitialized         ("FALSE"),
        .read_during_write_mode_port_a  ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type                 ("M9K"),
        .intended_device_family         ("Cyclone IV E"),
        .lpm_hint                       ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=R00"),
        .lpm_type                       ("altsyncram")
    ) snap_ram_R00 (
        .address_a (word_in),
        .clock0    (clk),
        .data_a    (snap_data),
        .wren_a    (rst | (v_d && idx_d[5] == 1'b0 && grp == 1'b0)),
        .q_a       (q_R00_unused)
    );

    wire [31:0] q_R16_unused;
    altsyncram #(
        .operation_mode                 ("SINGLE_PORT"),
        .width_a                        (32),
        .widthad_a                      (4),
        .numwords_a                     (16),
        .outdata_reg_a                  ("UNREGISTERED"),
        .clock_enable_input_a           ("BYPASS"),
        .clock_enable_output_a          ("BYPASS"),
        .power_up_uninitialized         ("FALSE"),
        .read_during_write_mode_port_a  ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type                 ("M9K"),
        .intended_device_family         ("Cyclone IV E"),
        .lpm_hint                       ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=R16"),
        .lpm_type                       ("altsyncram")
    ) snap_ram_R16 (
        .address_a (word_in),
        .clock0    (clk),
        .data_a    (snap_data),
        .wren_a    (rst | (v_d && idx_d[5] == 1'b0 && grp == 1'b1)),
        .q_a       (q_R16_unused)
    );

    wire [31:0] q_D00_unused;
    altsyncram #(
        .operation_mode                 ("SINGLE_PORT"),
        .width_a                        (32),
        .widthad_a                      (4),
        .numwords_a                     (16),
        .outdata_reg_a                  ("UNREGISTERED"),
        .clock_enable_input_a           ("BYPASS"),
        .clock_enable_output_a          ("BYPASS"),
        .power_up_uninitialized         ("FALSE"),
        .read_during_write_mode_port_a  ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type                 ("M9K"),
        .intended_device_family         ("Cyclone IV E"),
        .lpm_hint                       ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=D00"),
        .lpm_type                       ("altsyncram")
    ) snap_ram_D00 (
        .address_a (word_in),
        .clock0    (clk),
        .data_a    (snap_data),
        .wren_a    (rst | (v_d && idx_d[5] == 1'b1 && grp == 1'b0)),
        .q_a       (q_D00_unused)
    );

    wire [31:0] q_D16_unused;
    altsyncram #(
        .operation_mode                 ("SINGLE_PORT"),
        .width_a                        (32),
        .widthad_a                      (4),
        .numwords_a                     (16),
        .outdata_reg_a                  ("UNREGISTERED"),
        .clock_enable_input_a           ("BYPASS"),
        .clock_enable_output_a          ("BYPASS"),
        .power_up_uninitialized         ("FALSE"),
        .read_during_write_mode_port_a  ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type                 ("M9K"),
        .intended_device_family         ("Cyclone IV E"),
        .lpm_hint                       ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=D16"),
        .lpm_type                       ("altsyncram")
    ) snap_ram_D16 (
        .address_a (word_in),
        .clock0    (clk),
        .data_a    (snap_data),
        .wren_a    (rst | (v_d && idx_d[5] == 1'b1 && grp == 1'b1)),
        .q_a       (q_D16_unused)
    );

endmodule
