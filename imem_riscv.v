//====================================================================
// imem_riscv.v   (RISC-V RV32I multi-cycle CPU - step 4, revision B)
//
//   Synchronous-read instruction memory, written in the standard Intel
//   "synchronous ROM" style so that Quartus maps it to M9K block RAM:
//
//       always @(posedge clk) if (fetch_en) instruction <= mem[addr];
//
//   Behaviour is IDENTICAL to revision A (spec Sections 2.2, 2.4, 5):
//     - at a clock edge with fetch_en = 1 the word at pc[ADDR_BITS+1:2]
//       is captured; 'instruction' shows it from the NEXT cycle on
//     - 'instruction' HOLDS while fetch_en = 0
//     - during the FETCH cycle it still shows the previous instruction
//
//   What changed vs. revision A (which synthesised into ~147 logic
//   cells and 0 memory bits):
//     - the read register is now the OUTPUT register (the form Intel's
//       templates use; Quartus moves it to the M9K address port)
//     - memory style attribute is 'ramstyle = "M9K"' (the documented
//       attribute; it applies to ROMs inferred from arrays as well)
//
//   Contents come from program.hex ($readmemh, one 8-digit hex word per
//   line, 2**ADDR_BITS lines). The output register is the [HH] Instr register
//   (fetch_en = IRWrite).
//   ROM only (no write port). A PC beyond the last word wraps.
//====================================================================
module imem_riscv #(
    parameter ADDR_BITS = 7,                  // 2**7 = 128 words = 1 M9K (same as the ARM cores)
    parameter INIT_FILE = "program.hex"
)(
    input             clk,
    input             fetch_en,               // read enable (clock enable)
    input      [31:0] pc,                     // byte address
    output reg [31:0] instruction
);

    (* ramstyle = "M9K" *) reg [31:0] mem [0:(1<<ADDR_BITS)-1];

    initial begin
        $readmemh(INIT_FILE, mem);
        // synthesis translate_off
        instruction = mem[0];                 // simulation only: same power-up value as before
        // synthesis translate_on
    end

    always @(posedge clk) begin
        if (fetch_en)
            instruction <= mem[pc[ADDR_BITS+1:2]];
    end

endmodule
