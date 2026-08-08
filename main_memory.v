// ============================================================================
// Main Memory with Configurable Latency
// Simulates slow main memory (DRAM-like) with multi-cycle access
// Size: 4KB (1024 words), 10-cycle access latency
// Restructured for Xilinx Block RAM inference
// ============================================================================
`timescale 1ns/1ps
module main_memory #(
    parameter MEM_DEPTH   = 1024,   // number of 32-bit words
    parameter LATENCY     = 10      // access latency in clock cycles
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        req,         // memory request
    input  wire        wr,          // 1=write, 0=read
    input  wire [31:0] addr,        // byte address
    input  wire [31:0] wdata,       // write data
    output reg  [31:0] rdata,       // read data
    output reg         ready        // access complete
);

    // Memory array — attribute forces Block RAM inference
    (* ram_style = "block" *) reg [31:0] mem [0:MEM_DEPTH-1];

    // Initialize data memory for synthesis (Vivado loads into BRAM)
    initial begin
        $readmemh("C:/Users/anike/Desktop/co_project_new/only_cache/data.hex", mem);
    end

    // Latency counter
    reg [3:0] lat_cnt;
    reg       active;

    // Word address (use only the bits needed for indexing)
    wire [9:0] word_idx = addr[11:2];

    // Signals for memory access at completion
    wire access_done = active && (lat_cnt == LATENCY);

    // ---- Block RAM read/write (separate always block, no reset) ----
    always @(posedge clk) begin
        if (access_done && wr) begin
            mem[word_idx] <= wdata;
        end
        if (access_done && !wr) begin
            rdata <= mem[word_idx];
        end
    end

    // ---- Control logic (with reset) ----
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ready   <= 1'b0;
            lat_cnt <= 4'd0;
            active  <= 1'b0;
        end else begin
            ready <= 1'b0;
            if (req && !active && !ready) begin
                active  <= 1'b1;
                lat_cnt <= 4'd1;
            end else if (active) begin
                if (lat_cnt == LATENCY) begin
                    active  <= 1'b0;
                    ready   <= 1'b1;
                    lat_cnt <= 4'd0;
                end else begin
                    lat_cnt <= lat_cnt + 1;
                end
            end
        end
    end

endmodule
