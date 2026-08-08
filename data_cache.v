// ============================================================================
// Data Cache - Direct-Mapped, Write-Back, Write-Allocate
// Size: 1KB total, 64 lines, 4 words (16 bytes) per line
// Address breakdown (32-bit byte address):
//   [31:10] tag (22 bits), [9:4] index (6 bits), [3:2] word offset (2 bits), [1:0] byte offset
// ============================================================================
`timescale 1ns/1ps
module data_cache (
    input  wire        clk,
    input  wire        rst,
    // CPU interface
    input  wire        cpu_req,       // CPU requests memory access
    input  wire        cpu_wr,        // 1=write, 0=read
    input  wire [31:0] cpu_addr,      // byte address from CPU
    input  wire [31:0] cpu_wdata,     // write data from CPU
    output wire [31:0] cpu_rdata,     // read data to CPU
    output wire        stall,         // stall pipeline on miss
    // Main memory interface
    output reg         mem_req,       // request to main memory
    output reg         mem_wr,        // 1=write, 0=read
    output reg  [31:0] mem_addr,      // address to main memory
    output reg  [31:0] mem_wdata,     // write data to main memory
    input  wire [31:0] mem_rdata,     // read data from main memory
    input  wire        mem_ready,     // main memory access complete
    // Performance counters
    output reg  [31:0] hit_count,
    output reg  [31:0] miss_count
);

    // Cache parameters
    localparam NUM_LINES    = 64;
    localparam WORDS_PER_LINE = 4;
    localparam TAG_WIDTH    = 22;
    localparam INDEX_WIDTH  = 6;
    localparam OFFSET_WIDTH = 2;

    // Cache storage
    reg [31:0] cache_data [0:NUM_LINES-1][0:WORDS_PER_LINE-1];
    reg [TAG_WIDTH-1:0] cache_tag [0:NUM_LINES-1];
    reg cache_valid [0:NUM_LINES-1];
    reg cache_dirty [0:NUM_LINES-1];

    // Address decomposition
    wire [TAG_WIDTH-1:0]    tag    = cpu_addr[31:10];
    wire [INDEX_WIDTH-1:0]  index  = cpu_addr[9:4];
    wire [OFFSET_WIDTH-1:0] offset = cpu_addr[3:2];

    // Cache hit detection
    wire hit = cache_valid[index] && (cache_tag[index] == tag);

    // FSM states
    localparam IDLE       = 3'd0;
    localparam WB_WRITE   = 3'd1;  // write-back dirty line to memory
    localparam FILL_READ  = 3'd2;  // fill cache line from memory
    localparam DONE       = 3'd3;

    reg [2:0] state, next_state;
    reg [1:0] word_cnt;            // counts words during WB/fill (0..3)

    // Stall when there's a request and we're not in IDLE with a hit
    assign stall = cpu_req && !((state == IDLE) && hit);

    // Combinational read of cache data
    assign cpu_rdata = cache_data[index][offset];

    integer i, j;

    // State register
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state    <= IDLE;
            word_cnt <= 2'd0;
            mem_req  <= 1'b0;
            mem_wr   <= 1'b0;
            mem_addr <= 32'd0;
            mem_wdata <= 32'd0;
            hit_count  <= 32'd0;
            miss_count <= 32'd0;
            for (i = 0; i < NUM_LINES; i = i + 1) begin
                cache_valid[i] <= 1'b0;
                cache_dirty[i] <= 1'b0;
                cache_tag[i]   <= {TAG_WIDTH{1'b0}};
            end
        end else begin
            case (state)
                IDLE: begin
                    mem_req <= 1'b0;
                    if (cpu_req) begin
                        if (hit) begin
                            // Cache hit
                            hit_count <= hit_count + 1;
                            if (cpu_wr) begin
                                cache_data[index][offset] <= cpu_wdata;
                                cache_dirty[index] <= 1'b1;
                            end
                        end else begin
                            // Cache miss
                            miss_count <= miss_count + 1;
                            word_cnt   <= 2'd0;
                            if (cache_valid[index] && cache_dirty[index]) begin
                                // Need to write-back dirty line first
                                state    <= WB_WRITE;
                                mem_req  <= 1'b1;
                                mem_wr   <= 1'b1;
                                mem_addr <= {cache_tag[index], index, 4'b0000};
                                mem_wdata <= cache_data[index][0];
                            end else begin
                                // Clean miss - go straight to fill
                                state    <= FILL_READ;
                                mem_req  <= 1'b1;
                                mem_wr   <= 1'b0;
                                mem_addr <= {tag, index, 4'b0000};
                            end
                        end
                    end
                end

                WB_WRITE: begin
                    if (mem_ready) begin
                        if (word_cnt == 2'd3) begin
                            // Write-back complete, start fill
                            word_cnt <= 2'd0;
                            state    <= FILL_READ;
                            mem_req  <= 1'b1;
                            mem_wr   <= 1'b0;
                            mem_addr <= {tag, index, 4'b0000};
                        end else begin
                            word_cnt  <= word_cnt + 1;
                            mem_req   <= 1'b1;
                            mem_wr    <= 1'b1;
                            mem_addr  <= {cache_tag[index], index, word_cnt + 1'b1, 2'b00};
                            mem_wdata <= cache_data[index][word_cnt + 1'b1];
                        end
                    end
                end

                FILL_READ: begin
                    if (mem_ready) begin
                        cache_data[index][word_cnt] <= mem_rdata;
                        if (word_cnt == 2'd3) begin
                            // Fill complete
                            cache_tag[index]   <= tag;
                            cache_valid[index] <= 1'b1;
                            cache_dirty[index] <= 1'b0;
                            state   <= DONE;
                            mem_req <= 1'b0;
                        end else begin
                            word_cnt <= word_cnt + 1;
                            mem_req  <= 1'b1;
                            mem_wr   <= 1'b0;
                            mem_addr <= {tag, index, word_cnt + 1'b1, 2'b00};
                        end
                    end
                end

                DONE: begin
                    // Re-process the original request (now a hit)
                    if (cpu_wr) begin
                        cache_data[index][offset] <= cpu_wdata;
                        cache_dirty[index] <= 1'b1;
                    end
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
