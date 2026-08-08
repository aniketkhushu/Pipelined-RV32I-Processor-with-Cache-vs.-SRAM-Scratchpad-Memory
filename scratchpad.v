// ============================================================================
// SRAM Scratchpad Memory
// Size: 1KB (256 words), single-cycle access
// Address range: 0x00010000 - 0x000103FF (byte addresses)
// ============================================================================
module scratchpad (
    input  wire        clk,
    input  wire        rst,
    // CPU load/store interface
    input  wire        cpu_req,
    input  wire        cpu_wr,
    input  wire [31:0] cpu_addr,
    input  wire [31:0] cpu_wdata,
    output wire [31:0] cpu_rdata,
    // DMA interface (bulk transfer)
    input  wire        dma_req,
    input  wire        dma_wr,
    input  wire [7:0]  dma_spm_addr,  // word address within scratchpad
    input  wire [31:0] dma_wdata,
    output wire [31:0] dma_rdata
);

    localparam SPM_DEPTH = 256; // 1KB / 4 bytes

    // SRAM array
    reg [31:0] spm [0:SPM_DEPTH-1];

    // Word address from CPU byte address (offset from SPM base)
    wire [7:0] cpu_word_addr = cpu_addr[9:2];

    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < SPM_DEPTH; i = i + 1)
                spm[i] <= 32'd0;
        end else begin
            // CPU access (load/store from pipeline MEM stage)
            if (cpu_req && cpu_wr) begin
                spm[cpu_word_addr] <= cpu_wdata;
            end
            // DMA access (from custom instruction controller)
            if (dma_req && dma_wr) begin
                spm[dma_spm_addr] <= dma_wdata;
            end
        end
    end

    // Combinational read
    assign cpu_rdata = spm[cpu_word_addr];
    assign dma_rdata = spm[dma_spm_addr];

endmodule
