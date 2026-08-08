// ============================================================================
// FPGA Top-Level Wrapper for Zybo Z7-10
// Wraps rv32i_cache_top with minimal I/O for physical board implementation.
// Internal signals (cycle_count, cache_hits, cache_misses) are kept internal
// and can be probed using ILA (mark_debug).
// ============================================================================
`timescale 1ns/1ps
module fpga_top (
    input  wire       clk,       // 125 MHz board clock (K17)
    input  wire       rst_btn,   // Active-high reset from BTN0 (K18)
    output wire [3:0] led        // 4 on-board LEDs
);

    // --------------------------------------------------------------------
    // Internal wires from the pipeline
    // --------------------------------------------------------------------
    wire        halt;
    wire [31:0] cycle_count;
    wire [31:0] cache_hits;
    wire [31:0] cache_misses;

    // ---- Mark signals for ILA debugging (uncomment as needed) ----------
    // (* mark_debug = "true" *) wire [31:0] dbg_cycle_count = cycle_count;
    // (* mark_debug = "true" *) wire [31:0] dbg_cache_hits  = cache_hits;
    // (* mark_debug = "true" *) wire [31:0] dbg_cache_misses = cache_misses;
    // (* mark_debug = "true" *) wire        dbg_halt        = halt;

    // --------------------------------------------------------------------
    // LED assignments
    //   led[0] = halt flag (lights up when program finishes)
    //   led[1] = cache activity (toggles on cache_hits LSB)
    //   led[2] = cycle_count[20] (~1 Hz blink at 125 MHz = design is alive)
    //   led[3] = rst_btn echo (confirms reset button is working)
    // --------------------------------------------------------------------
    assign led[0] = halt;
    assign led[1] = cache_hits[0];
    assign led[2] = cycle_count[20];
    assign led[3] = rst_btn;

    // --------------------------------------------------------------------
    // Pipeline instantiation
    // --------------------------------------------------------------------
    rv32i_cache_top u_pipeline (
        .clk          (clk),
        .rst          (rst_btn),
        .halt         (halt),
        .cycle_count  (cycle_count),
        .cache_hits   (cache_hits),
        .cache_misses (cache_misses)
    );

endmodule
