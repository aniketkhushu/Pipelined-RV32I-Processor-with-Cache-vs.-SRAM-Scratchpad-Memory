// ============================================================================
// Testbench: Scratchpad-based RV32I Pipeline - 4x4 Matrix Multiply
// Program flow:
//   1. SPM_DMA: copy A (16 words) from main mem -> scratchpad
//   2. SPM_DMA: copy B (16 words) from main mem -> scratchpad
//   3. Compute C = A*B using scratchpad loads/stores (1-cycle each)
//   4. SPM_DMA: copy C (16 words) from scratchpad -> main mem
// ============================================================================
`timescale 1ns/1ps

module tb_spm_matmul;

    reg clk, rst;
    wire halt;
    wire [31:0] cycle_count, dma_cycle_count;

    rv32i_spm_top uut (
        .clk(clk), .rst(rst), .halt(halt),
        .cycle_count(cycle_count), .dma_cycle_count(dma_cycle_count)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    integer i, errors;
    reg [31:0] expected_c [0:15];

    // Custom instruction encoding helper:
    // SPM_DMA: R-type, opcode=0001011
    // funct7[6]=dir (0=mem->spm, 1=spm->mem)
    // rs1=src, rs2=dst, rd=count_reg
    // Encoding: {funct7, rs2, rs1, funct3=000, rd, opcode=0001011}

    initial begin
        errors = 0;
        $dumpfile("spm_matmul.vcd");
        $dumpvars(0, tb_spm_matmul);

        // ================================================================
        // Program: Scratchpad-based matrix multiply
        // ================================================================
        // SPM layout: A at 0x10000 (spm offset 0), B at 0x10040, C at 0x10080
        // Main mem: A at 0x0000, B at 0x0040, C at 0x0080

        // -- Setup phase: load addresses and counts --
        // 0: addi x1, x0, 0          # main mem A base = 0
        uut.imem[0]  = 32'h00000093;
        // 1: addi x2, x0, 64         # main mem B base = 64
        uut.imem[1]  = 32'h04000113;
        // 2: addi x3, x0, 128        # main mem C base = 128
        uut.imem[2]  = 32'h08000193;
        // 3: lui  x14, 0x10           # x14 = 0x10000 (SPM base)
        uut.imem[3]  = 32'h00010737;
        // 4: addi x15, x14, 64       # x15 = 0x10040 (SPM B base)
        uut.imem[4]  = 32'h04070793;
        // 5: addi x16, x14, 128      # x16 = 0x10080 (SPM C base)
        uut.imem[5]  = 32'h08070813;
        // 6: addi x17, x0, 16        # x17 = 16 (word count for DMA)
        uut.imem[6]  = 32'h01000893;
        // 7: addi x7, x0, 4          # N = 4
        uut.imem[7]  = 32'h00400393;

        // -- DMA phase: copy A and B to scratchpad --
        // 8: SPM_DMA x17, x1, x14    # mem->spm: copy 16 words, src=x1(0), dst=x14(0x10000)
        //    funct7=0000000, rs2=x14=01110, rs1=x1=00001, f3=000, rd=x17=10001, op=0001011
        uut.imem[8]  = 32'h00E0888B;
        // 9: SPM_DMA x17, x2, x15    # mem->spm: copy 16 words, src=x2(64), dst=x15(0x10040)
        //    funct7=0000000, rs2=x15=01111, rs1=x2=00010, f3=000, rd=x17=10001, op=0001011
        uut.imem[9]  = 32'h00F1088B;

        // -- Compute phase: triple loop using SPM addresses --
        // 10: addi x4, x0, 0         # i = 0
        uut.imem[10] = 32'h00000213;
        // 11: addi x5, x0, 0         # j = 0        [OUTER]
        uut.imem[11] = 32'h00000293;
        // 12: addi x10, x0, 0        # sum = 0      [MIDDLE]
        uut.imem[12] = 32'h00000513;
        // 13: addi x6, x0, 0         # k = 0
        uut.imem[13] = 32'h00000313;
        // 14: slli x11, x4, 2        # i*4          [INNER]
        uut.imem[14] = 32'h00221593;
        // 15: add  x11, x11, x6      # i*4+k
        uut.imem[15] = 32'h006585B3;
        // 16: slli x11, x11, 2       # (i*4+k)*4 byte offset
        uut.imem[16] = 32'h00259593;
        // 17: add  x11, x11, x14     # SPM addr of A[i][k]
        uut.imem[17] = 32'h00E585B3;
        // 18: lw   x8, 0(x11)        # A[i][k] from SPM (1 cycle!)
        uut.imem[18] = 32'h0005A403;
        // 19: slli x12, x6, 2        # k*4
        uut.imem[19] = 32'h00231613;
        // 20: add  x12, x12, x5      # k*4+j
        uut.imem[20] = 32'h00560633;
        // 21: slli x12, x12, 2       # (k*4+j)*4
        uut.imem[21] = 32'h00261613;
        // 22: add  x12, x12, x15     # SPM addr of B[k][j]
        uut.imem[22] = 32'h00F60633;
        // 23: lw   x9, 0(x12)        # B[k][j] from SPM (1 cycle!)
        uut.imem[23] = 32'h00062483;
        // 24: mul  x13, x8, x9       # product
        uut.imem[24] = 32'h029406B3;
        // 25: add  x10, x10, x13     # sum += product
        uut.imem[25] = 32'h00D50533;
        // 26: addi x6, x6, 1         # k++
        uut.imem[26] = 32'h00130313;
        // 27: blt  x6, x7, -52       # if k<4 goto INNER (instr 14)
        //     offset = (14-27)*4 = -52
        uut.imem[27] = 32'hFC7346E3;
        // 28: slli x11, x4, 2        # i*4
        uut.imem[28] = 32'h00221593;
        // 29: add  x11, x11, x5      # i*4+j
        uut.imem[29] = 32'h005585B3;
        // 30: slli x11, x11, 2       # (i*4+j)*4
        uut.imem[30] = 32'h00259593;
        // 31: add  x11, x11, x16     # SPM addr of C[i][j]
        uut.imem[31] = 32'h010585B3;
        // 32: sw   x10, 0(x11)       # C[i][j] = sum to SPM
        uut.imem[32] = 32'h00A5A023;
        // 33: addi x5, x5, 1         # j++
        uut.imem[33] = 32'h00128293;
        // 34: blt  x5, x7, -88       # if j<4 goto MIDDLE (instr 12)
        //     offset = (12-34)*4 = -88
        uut.imem[34] = 32'hFA72C4E3;
        // 35: addi x4, x4, 1         # i++
        uut.imem[35] = 32'h00120213;
        // 36: blt  x4, x7, -100      # if i<4 goto OUTER (instr 11)
        //     offset = (11-36)*4 = -100
        uut.imem[36] = 32'hF2724EE3;

        // -- Writeback phase: copy C from scratchpad to main memory --
        // 37: SPM_DMA x17, x16, x3   # spm->mem: 16 words, src=x16(0x10080), dst=x3(128)
        //     funct7=1000000, rs2=x3=00011, rs1=x16=10000, f3=000, rd=x17=10001, op=0001011
        uut.imem[37] = 32'h8038088B;

        // 38: jal x0, 0              # HALT
        uut.imem[38] = 32'h0000006F;

        // Fill rest with NOPs
        for (i = 39; i < 256; i = i + 1)
            uut.imem[i] = 32'h00000013;

        // ================================================================
        // Load matrix data into main memory
        // ================================================================
        // A = [[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,16]]
        uut.dmem.mem[0]  = 32'd1;  uut.dmem.mem[1]  = 32'd2;
        uut.dmem.mem[2]  = 32'd3;  uut.dmem.mem[3]  = 32'd4;
        uut.dmem.mem[4]  = 32'd5;  uut.dmem.mem[5]  = 32'd6;
        uut.dmem.mem[6]  = 32'd7;  uut.dmem.mem[7]  = 32'd8;
        uut.dmem.mem[8]  = 32'd9;  uut.dmem.mem[9]  = 32'd10;
        uut.dmem.mem[10] = 32'd11; uut.dmem.mem[11] = 32'd12;
        uut.dmem.mem[12] = 32'd13; uut.dmem.mem[13] = 32'd14;
        uut.dmem.mem[14] = 32'd15; uut.dmem.mem[15] = 32'd16;

        // B = 2*I
        uut.dmem.mem[16] = 32'd2;  uut.dmem.mem[17] = 32'd0;
        uut.dmem.mem[18] = 32'd0;  uut.dmem.mem[19] = 32'd0;
        uut.dmem.mem[20] = 32'd0;  uut.dmem.mem[21] = 32'd2;
        uut.dmem.mem[22] = 32'd0;  uut.dmem.mem[23] = 32'd0;
        uut.dmem.mem[24] = 32'd0;  uut.dmem.mem[25] = 32'd0;
        uut.dmem.mem[26] = 32'd2;  uut.dmem.mem[27] = 32'd0;
        uut.dmem.mem[28] = 32'd0;  uut.dmem.mem[29] = 32'd0;
        uut.dmem.mem[30] = 32'd0;  uut.dmem.mem[31] = 32'd2;

        for (i = 32; i < 64; i = i + 1)
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

        wait (halt || cycle_count > 50000);
        #200;

        // ================================================================
        // Verify results
        // ================================================================
        $display("============================================================");
        $display("  SCRATCHPAD-BASED PIPELINE: 4x4 Matrix Multiply Results");
        $display("============================================================");
        $display("Total Cycles     : %0d", cycle_count);
        $display("DMA Cycles       : %0d", dma_cycle_count);
        $display("Compute Cycles   : %0d", cycle_count - dma_cycle_count);
        $display("------------------------------------------------------------");

        errors = 0;
        $display("Matrix C (Result = 2*A):");
        $display("  Checking scratchpad (C at SPM word offset 32-47):");
        for (i = 0; i < 16; i = i + 1) begin
            // C in scratchpad at word address 32+i (byte addr 0x10080+i*4)
            if (uut.spm_inst.spm[32 + i] !== expected_c[i]) begin
                $display("  C[%0d][%0d] = %0d (EXPECTED %0d) FAIL",
                         i/4, i%4, uut.spm_inst.spm[32 + i], expected_c[i]);
                errors = errors + 1;
            end else begin
                $display("  C[%0d][%0d] = %0d  OK", i/4, i%4, uut.spm_inst.spm[32+i]);
            end
        end
        $display("  Checking main memory (C at word 32-47 after DMA writeback):");
        for (i = 0; i < 16; i = i + 1) begin
            $display("  MainMem C[%0d][%0d] = %0d", i/4, i%4, uut.dmem.mem[32+i]);
        end

        $display("------------------------------------------------------------");
        if (errors == 0)
            $display("RESULT: ALL 16 ELEMENTS CORRECT - PASS");
        else
            $display("RESULT: %0d ERRORS - FAIL", errors);
        $display("============================================================");

        #20;
        $finish;
    end

    initial begin
        #5000000;
        $display("TIMEOUT: Simulation exceeded maximum time");
        $finish;
    end

endmodule
