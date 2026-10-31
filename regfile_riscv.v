//====================================================================
// regfile_riscv.v   (multi-cycle CPU: block RAM only; the outputs are the [HH] A / B registers)
//
//   Two identical M9K copies (ram1 for rs1, ram2 for rs2), both written together.
//   Both READ PORTS ARE SYNCHRONOUS: the address (rs1/rs2 field of the held instruction)
//   is captured at the clock edge that ends DECODE and read_data1/read_data2 are valid
//   in the next cycle and stay valid while the instruction is held. The datapath uses
//   these two outputs directly as A and B (no A/B registers, no 32:1 multiplexers).
//
//   There is NO flip-flop copy of the registers any more: the register view comes from
//   the snapshot unit + In-System Memory Content Editor (see snapshot_unit.v).
//
//   x0: never written, both RAM copies are cleared to 0, so reading x0 gives 0.
//
//   Reset: the two RAM copies are cleared by a 5-bit counter that
//   writes zeros to all 32 addresses while rst is high, so rst must stay high for at least
//   32 clock cycles (the board holds the CPU in reset until KEY2, so this is always true).
//====================================================================
module regfile_riscv (
    input         clk,
    input         rst,          // synchronous; hold >= 32 cycles (clears the RAM copies)
    input  [4:0]  read_reg1,    // rs1 (or the snapshot index)
    input  [4:0]  read_reg2,    // rs2
    input  [4:0]  write_reg,    // rd
    input  [31:0] write_data,
    input         write_enable,
    output [31:0] read_data1,   // synchronous: valid one cycle after read_reg1 is captured
    output [31:0] read_data2
);

    reg [4:0] clr_cnt = 5'd0;                       // runs while rst is high
    always @(posedge clk) begin
        if (rst) clr_cnt <= clr_cnt + 5'd1;
        else     clr_cnt <= 5'd0;
    end

    wire        ram_we    = rst | (write_enable & (write_reg != 5'd0));
    wire [4:0]  ram_waddr = rst ? clr_cnt : write_reg;
    wire [31:0] ram_wdata = rst ? 32'd0   : write_data;

    (* ramstyle = "M9K, no_rw_check" *) reg [31:0] ram1 [0:31];
    (* ramstyle = "M9K, no_rw_check" *) reg [31:0] ram2 [0:31];
    reg [31:0] q1, q2;

    integer k;
    initial begin
        for (k = 0; k < 32; k = k + 1) begin ram1[k] = 32'd0; ram2[k] = 32'd0; end
        q1 = 32'd0; q2 = 32'd0;
    end

    always @(posedge clk) begin
        if (ram_we) ram1[ram_waddr] <= ram_wdata;
        q1 <= ram1[read_reg1];
    end
    always @(posedge clk) begin
        if (ram_we) ram2[ram_waddr] <= ram_wdata;
        q2 <= ram2[read_reg2];
    end

    assign read_data1 = q1;
    assign read_data2 = q2;

endmodule
