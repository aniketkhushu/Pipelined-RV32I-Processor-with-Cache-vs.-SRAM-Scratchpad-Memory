// ============================================================================
// Testbench: Cache-based RV32I Pipeline with 4x4 Matrix Multiply
// Matrices: A(4x4) at addr 0x0000, B(4x4) at 0x0040, C(4x4) at 0x0080
// B = 2*I so C = 2*A (easy verification)
// ============================================================================
`timescale 1ns/1ps

module tb_cache_matmul;

    reg         clk, rst;
    wire        halt;
    wire [31:0] cycle_count, cache_hits, cache_misses;

    // Instantiate DUT
    rv32i_cache_top uut (
        .clk         (clk),
        .rst         (rst),
        .halt        (halt),
        .cycle_count (cycle_count),
        .cache_hits  (cache_hits),
        .cache_misses(cache_misses)
    );

    // Clock: 10ns period
    initial clk = 0;
    always #5 clk = ~clk;

    integer i, errors;
    reg [31:0] expected_c [0:15];

    initial begin
        errors=0;
        $dumpfile("cache_matmul.vcd");
        $dumpvars(0, tb_cache_matmul);

        // ================================================================
        // Load matrix-multiply program into instruction memory
        // ================================================================
        // 0: addi x1, x0, 0       # A_base = 0
        uut.imem[0]  = 32'h00000093;
        // 1: addi x2, x0, 64      # B_base = 64
        uut.imem[1]  = 32'h04000113;
        // 2: addi x3, x0, 128     # C_base = 128
        uut.imem[2]  = 32'h08000193;
        // 3: addi x7, x0, 4       # N = 4
        uut.imem[3]  = 32'h00400393;
        // 4: addi x4, x0, 0       # i = 0
        uut.imem[4]  = 32'h00000213;
        // 5: addi x5, x0, 0       # j = 0        [OUTER]
        uut.imem[5]  = 32'h00000293;
        // 6: addi x10, x0, 0      # sum = 0      [MIDDLE]
        uut.imem[6]  = 32'h00000513;
        // 7: addi x6, x0, 0       # k = 0
        uut.imem[7]  = 32'h00000313;
        // 8: slli x11, x4, 2      # i*4          [INNER]
        uut.imem[8]  = 32'h00221593;
        // 9: add  x11, x11, x6    # i*4+k
        uut.imem[9]  = 32'h006585B3;
        // 10: slli x11, x11, 2    # (i*4+k)*4
        uut.imem[10] = 32'h00259593;
        // 11: add  x11, x11, x1   # &A[i][k]
        uut.imem[11] = 32'h001585B3;
        // 12: lw   x8, 0(x11)     # A[i][k]
        uut.imem[12] = 32'h0005A403;
        // 13: slli x12, x6, 2     # k*4
        uut.imem[13] = 32'h00231613;
        // 14: add  x12, x12, x5   # k*4+j
        uut.imem[14] = 32'h00560633;
        // 15: slli x12, x12, 2    # (k*4+j)*4
        uut.imem[15] = 32'h00261613;
        // 16: add  x12, x12, x2   # &B[k][j]
        uut.imem[16] = 32'h00260633;
        // 17: lw   x9, 0(x12)     # B[k][j]
        uut.imem[17] = 32'h00062483;
        // 18: mul  x13, x8, x9    # A[i][k]*B[k][j]
        uut.imem[18] = 32'h029406B3;
        // 19: add  x10, x10, x13  # sum += product
        uut.imem[19] = 32'h00D50533;
        // 20: addi x6, x6, 1      # k++
        uut.imem[20] = 32'h00130313;
        // 21: blt  x6, x7, -52    # if k<4 goto INNER
        uut.imem[21] = 32'hFC7346E3;
        // 22: slli x11, x4, 2     # i*4
        uut.imem[22] = 32'h00221593;
        // 23: add  x11, x11, x5   # i*4+j
        uut.imem[23] = 32'h005585B3;
        // 24: slli x11, x11, 2    # (i*4+j)*4
        uut.imem[24] = 32'h00259593;
        // 25: add  x11, x11, x3   # &C[i][j]
        uut.imem[25] = 32'h003585B3;
        // 26: sw   x10, 0(x11)    # C[i][j] = sum
        uut.imem[26] = 32'h00A5A023;
        // 27: addi x5, x5, 1      # j++
        uut.imem[27] = 32'h00128293;
        // 28: blt  x5, x7, -88    # if j<4 goto MIDDLE
        uut.imem[28] = 32'hFA72C4E3;
        // 29: addi x4, x4, 1      # i++
        uut.imem[29] = 32'h00120213;
        // 30: blt  x4, x7, -100   # if i<4 goto OUTER
        uut.imem[30] = 32'hF8724EE3;
        // 31: jal  x0, 0          # HALT (self-loop)
        uut.imem[31] = 32'h0000006F;

        // Fill rest of imem with NOPs
        for (i = 32; i < 256; i = i + 1)
            uut.imem[i] = 32'h00000013;

        // ================================================================
        // Load matrix data into main memory
        // ================================================================
        // Matrix A at word addresses 0-15:
        // A = [[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,16]]
        uut.dmem.mem[0]  = 32'd1;  uut.dmem.mem[1]  = 32'd2;
        uut.dmem.mem[2]  = 32'd3;  uut.dmem.mem[3]  = 32'd4;
        uut.dmem.mem[4]  = 32'd5;  uut.dmem.mem[5]  = 32'd6;
        uut.dmem.mem[6]  = 32'd7;  uut.dmem.mem[7]  = 32'd8;
        uut.dmem.mem[8]  = 32'd9;  uut.dmem.mem[9]  = 32'd10;
        uut.dmem.mem[10] = 32'd11; uut.dmem.mem[11] = 32'd12;
        uut.dmem.mem[12] = 32'd13; uut.dmem.mem[13] = 32'd14;
        uut.dmem.mem[14] = 32'd15; uut.dmem.mem[15] = 32'd16;

        // Matrix B at word addresses 16-31 (byte addr 0x0040):
        // B = 2*I = [[2,0,0,0],[0,2,0,0],[0,0,2,0],[0,0,0,2]]
        uut.dmem.mem[16] = 32'd2;  uut.dmem.mem[17] = 32'd0;
        uut.dmem.mem[18] = 32'd0;  uut.dmem.mem[19] = 32'd0;
        uut.dmem.mem[20] = 32'd0;  uut.dmem.mem[21] = 32'd2;
        uut.dmem.mem[22] = 32'd0;  uut.dmem.mem[23] = 32'd0;
        uut.dmem.mem[24] = 32'd0;  uut.dmem.mem[25] = 32'd0;
        uut.dmem.mem[26] = 32'd2;  uut.dmem.mem[27] = 32'd0;
        uut.dmem.mem[28] = 32'd0;  uut.dmem.mem[29] = 32'd0;
        uut.dmem.mem[30] = 32'd0;  uut.dmem.mem[31] = 32'd2;

        // Clear C area
        for (i = 32; i < 48; i = i + 1)
            uut.dmem.mem[i] = 32'd0;

        // Expected C = 2*A
        expected_c[0]  = 32'd2;  expected_c[1]  = 32'd4;
        expected_c[2]  = 32'd6;  expected_c[3]  = 32'd8;
        expected_c[4]  = 32'd10; expected_c[5]  = 32'd12;
        expected_c[6]  = 32'd14; expected_c[7]  = 32'd16;
        expected_c[8]  = 32'd18; expected_c[9]  = 32'd20;
        expected_c[10] = 32'd22; expected_c[11] = 32'd24;
        expected_c[12] = 32'd26; expected_c[13] = 32'd28;
        expected_c[14] = 32'd30; expected_c[15] = 32'd32;

        // ================================================================
        // Run simulation
        // ================================================================
        rst = 1;
        #30;
        rst = 0;

        // Wait for halt or timeout
        wait (halt || cycle_count > 50000);
        // Let a few more cycles pass for write-back
        #100;

        // ================================================================
        // Verify results
        // ================================================================
        $display("============================================================");
        $display("  CACHE-BASED PIPELINE: 4x4 Matrix Multiply Results");
        $display("============================================================");
        $display("Total Cycles   : %0d", cycle_count);
        $display("Cache Hits     : %0d", cache_hits);
        $display("Cache Misses   : %0d", cache_misses);
        $display("Hit Rate       : %0d%%",
                 (cache_hits + cache_misses > 0) ?
                 (cache_hits * 100) / (cache_hits + cache_misses) : 0);
        $display("------------------------------------------------------------");

        // Check matrix C from cache (write-back cache keeps data in cache lines)
        // C at byte addr 0x0080-0x00BF: tag=0, index = addr[9:4], offset = addr[3:2]
        // For byte addr = 0x80 + i*4: index = (0x80+i*4)[9:4], offset = (0x80+i*4)[3:2]
        errors = 0;
        $display("Matrix C (Result = 2*A):");
        begin : check_cache_block
            reg [31:0] byte_addr;
            reg [5:0]  c_index;
            reg [1:0]  c_offset;
            reg [31:0] c_val;
            for (i = 0; i < 16; i = i + 1) begin
                byte_addr = 32'h80 + (i * 4);
                c_index  = byte_addr[9:4];
                c_offset = byte_addr[3:2];
                c_val = uut.dcache.cache_data[c_index][c_offset];
                if (c_val !== expected_c[i]) begin
                    $display("  C[%0d][%0d] = %0d (EXPECTED %0d) FAIL",
                             i/4, i%4, c_val, expected_c[i]);
                    errors = errors + 1;
                end else begin
                    $display("  C[%0d][%0d] = %0d  OK", i/4, i%4, c_val);
                end
            end
        end
       
        $display("------------------------------------------------------------");
        $display("DEBUG: Register file at halt:");
        $display("  x1(A_base)=%0d x2(B_base)=%0d x3(C_base)=%0d x4(i)=%0d x5(j)=%0d x6(k)=%0d x7(N)=%0d",
                 uut.regfile[1], uut.regfile[2], uut.regfile[3],
                 uut.regfile[4], uut.regfile[5], uut.regfile[6], uut.regfile[7]);
        $display("  x8=%0d x9=%0d x10(sum)=%0d x11=%0d x12=%0d x13=%0d",
                 uut.regfile[8], uut.regfile[9], uut.regfile[10],
                 uut.regfile[11], uut.regfile[12], uut.regfile[13]);
        $display("DEBUG: Cache valid bits for C lines:");
        $display("  line8(C[0])=%b line9(C[1])=%b line10(C[2])=%b line11(C[3])=%b",
                 uut.dcache.cache_valid[8], uut.dcache.cache_valid[9],
                 uut.dcache.cache_valid[10], uut.dcache.cache_valid[11]);
        $display("DEBUG: Cache data for C[2] (line 10): [%0d, %0d, %0d, %0d]",
                 uut.dcache.cache_data[10][0], uut.dcache.cache_data[10][1],
                 uut.dcache.cache_data[10][2], uut.dcache.cache_data[10][3]);
        $display("DEBUG: Main memory C area (words 32-47):");
        $display("  mem[32..35]=%0d %0d %0d %0d  mem[36..39]=%0d %0d %0d %0d",
                 uut.dmem.mem[32], uut.dmem.mem[33], uut.dmem.mem[34], uut.dmem.mem[35],
                 uut.dmem.mem[36], uut.dmem.mem[37], uut.dmem.mem[38], uut.dmem.mem[39]);
        $display("  mem[40..43]=%0d %0d %0d %0d  mem[44..47]=%0d %0d %0d %0d",
                 uut.dmem.mem[40], uut.dmem.mem[41], uut.dmem.mem[42], uut.dmem.mem[43],
                 uut.dmem.mem[44], uut.dmem.mem[45], uut.dmem.mem[46], uut.dmem.mem[47]);
        $display("DEBUG: Cache valid/data for A lines:");
        $display("  line0 valid=%b data=[%0d,%0d,%0d,%0d]", uut.dcache.cache_valid[0],
                 uut.dcache.cache_data[0][0], uut.dcache.cache_data[0][1],
                 uut.dcache.cache_data[0][2], uut.dcache.cache_data[0][3]);
        $display("  line2 valid=%b data=[%0d,%0d,%0d,%0d]", uut.dcache.cache_valid[2],
                 uut.dcache.cache_data[2][0], uut.dcache.cache_data[2][1],
                 uut.dcache.cache_data[2][2], uut.dcache.cache_data[2][3]);
        $display("  line3 valid=%b data=[%0d,%0d,%0d,%0d]", uut.dcache.cache_valid[3],
                 uut.dcache.cache_data[3][0], uut.dcache.cache_data[3][1],
                 uut.dcache.cache_data[3][2], uut.dcache.cache_data[3][3]);
        $display("------------------------------------------------------------");
        if (errors == 0)
            $display("RESULT: ALL 16 ELEMENTS CORRECT - PASS");
        else
            $display("RESULT: %0d ERRORS - FAIL", errors);
        $display("============================================================");

        #20;
        $finish;
    end
    
    // Timeout safety
    initial begin
        #5000000;
        $display("TIMEOUT: Simulation exceeded maximum time");
        $finish;
    end
    // ================================================================
// LIVE DEBUG TRACE (FINAL — USE THIS)
// ================================================================
always @(posedge clk) begin
    if (!rst) begin
        $display("T=%0t | PC=%h | i(x4)=%0d j(x5)=%0d k(x6)=%0d | sum(x10)=%0d | addr=%h | mem_wr=%b",
            $time,
            uut.pc,                // ✅ program counter
            uut.regfile[4],        // ✅ i
            uut.regfile[5],        // ✅ j
            uut.regfile[6],        // ✅ k
            uut.regfile[10],       // ✅ sum
            uut.cpu_mem_addr,      // ✅ memory address (correct signal)
            uut.cpu_mem_wr         // ✅ write enable (correct signal)
        );
    end
end
endmodule
