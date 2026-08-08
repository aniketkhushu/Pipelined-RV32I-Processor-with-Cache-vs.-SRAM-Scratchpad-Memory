// ============================================================================
// rv32i_cache_top.v — force recompile
// RV32I 5-Stage Pipeline with Hardware Data Cache
// Stages: IF -> ID -> EX -> MEM -> WB
// Features: Data forwarding, load-use hazard stall, branch flush
// Supports: RV32I base + MUL (M extension)
// ============================================================================
`timescale 1ns/1ps
module rv32i_cache_top (
    input  wire        clk,
    input  wire        rst,
    output wire        halt,
    output wire [31:0] cycle_count,
    output wire [31:0] cache_hits,
    output wire [31:0] cache_misses
);

    // ========================================================================
    // Performance counter
    // ========================================================================
    reg [31:0] cycles;
    assign cycle_count = cycles;
    always @(posedge clk or posedge rst)
        if (rst) cycles <= 0;
        else     cycles <= cycles + 1;

    // ========================================================================
    // Instruction Memory (ROM, 256 words)
    // ========================================================================
    (* ram_style = "block" *) reg [31:0] imem [0:255];
    wire [31:0] instr_addr;
    wire [31:0] instruction = imem[instr_addr[9:2]];

    // Initialize instruction memory for synthesis (Vivado infers Block RAM)
    // Create a file called "program.hex" with your machine code, one 32-bit
    // hex word per line (e.g., 00500093). Place it in the Vivado project directory.
    initial begin
        $readmemh("C:/Users/anike/Desktop/co_project_new/only_cache/program.hex", imem);
    end

    // ========================================================================
    // Wires
    // ========================================================================
    // Cache <-> Main Memory
    wire        cache_mem_req, cache_mem_wr, mem_ready;
    wire [31:0] cache_mem_addr, cache_mem_wdata, mem_rdata;

    // CPU <-> Cache
    wire        cache_stall;
    wire        cpu_mem_req, cpu_mem_wr;
    wire [31:0] cpu_mem_addr, cpu_mem_wdata, cpu_mem_rdata;

    // Pipeline stall/flush
    wire        stall;          // stall IF, ID, EX stages
    wire        flush_if_id;    // flush on branch taken
    wire        flush_id_ex;
    wire        load_use_stall; // load-use hazard

    assign stall = cache_stall;

    // ========================================================================
    // Register File
    // ========================================================================
    // Pre-declare pipeline registers to fix Vivado warnings
    // IF/ID
    reg [31:0] ifid_instr, ifid_pc;
    reg        ifid_valid;

    // ID/EX
    reg [31:0] idex_pc, idex_rs1_data, idex_rs2_data, idex_imm;
    reg [4:0]  idex_rd, idex_rs1, idex_rs2;
    reg [2:0]  idex_funct3;
    reg [6:0]  idex_funct7;
    reg        idex_reg_write, idex_mem_read, idex_mem_write, idex_mem_to_reg;
    reg        idex_alu_src, idex_is_branch, idex_is_jal, idex_is_jalr;
    reg        idex_is_lui, idex_is_auipc;
    reg [3:0]  idex_alu_op;
    reg        idex_valid;

    // EX/MEM
    reg [31:0] exmem_alu_result, exmem_rs2_data;
    reg [4:0]  exmem_rd;
    reg        exmem_reg_write, exmem_mem_read, exmem_mem_write, exmem_mem_to_reg;
    reg        exmem_valid;

    // MEM/WB
    reg [31:0] memwb_alu_result, memwb_mem_data;
    reg [4:0]  memwb_rd;
    reg        memwb_reg_write, memwb_mem_to_reg;

    wire [4:0]  wb_rd;
    wire        wb_reg_write;
    wire [31:0] wb_write_data;

    (* ram_style = "distributed" *) reg [31:0] regfile [0:31];
    // No reset on regfile — allows LUTRAM inference (saves 1024 FFs)
    // x0 is hardwired to 0 by the read logic below
    always @(posedge clk)
        if (wb_reg_write && wb_rd != 5'd0 && !stall)
            regfile[wb_rd] <= wb_write_data;

    wire [31:0] rs1_data = (id_rs1 == 5'd0) ? 32'd0 : regfile[id_rs1];
    wire [31:0] rs2_data = (id_rs2 == 5'd0) ? 32'd0 : regfile[id_rs2];

    // ========================================================================
    // IF Stage
    // ========================================================================
    reg [31:0] pc;
    assign instr_addr = pc;
    wire [31:0] pc_plus4 = pc + 32'd4;

    // Branch/jump target from EX
    wire        branch_taken;
    wire [31:0] branch_target;

    always @(posedge clk or posedge rst) begin
        if (rst)
            pc <= 32'd0;
        else if (!stall && !load_use_stall) begin
            if (branch_taken)
                pc <= branch_target;
            else
                pc <= pc_plus4;
        end
    end

    // ========================================================================
    // IF/ID Pipeline Register
    // ========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst || (flush_if_id && !stall)) begin
            ifid_instr <= 32'h00000013; // NOP (addi x0, x0, 0)
            ifid_pc    <= 32'd0;
            ifid_valid <= 1'b0;
        end else if (!stall && !load_use_stall) begin
            ifid_instr <= instruction;
            ifid_pc    <= pc;
            ifid_valid <= 1'b1;
        end
    end

    // ========================================================================
    // ID Stage - Decode
    // ========================================================================
    wire [6:0]  id_opcode = ifid_instr[6:0];
    wire [4:0]  id_rd     = ifid_instr[11:7];
    wire [2:0]  id_funct3 = ifid_instr[14:12];
    wire [4:0]  id_rs1;
    wire [4:0]  id_rs2;
    assign id_rs1    = ifid_instr[19:15];
    assign id_rs2    = ifid_instr[24:20];
    wire [6:0]  id_funct7 = ifid_instr[31:25];

    // Immediate generation
    reg [31:0] id_imm;
    always @(*) begin
        case (id_opcode)
            7'b0010011, 7'b0000011, 7'b1100111: // I-type
                id_imm = {{20{ifid_instr[31]}}, ifid_instr[31:20]};
            7'b0100011: // S-type
                id_imm = {{20{ifid_instr[31]}}, ifid_instr[31:25], ifid_instr[11:7]};
            7'b1100011: // B-type
                id_imm = {{19{ifid_instr[31]}}, ifid_instr[31], ifid_instr[7],
                           ifid_instr[30:25], ifid_instr[11:8], 1'b0};
            7'b0110111, 7'b0010111: // U-type (LUI, AUIPC)
                id_imm = {ifid_instr[31:12], 12'd0};
            7'b1101111: // J-type (JAL)
                id_imm = {{11{ifid_instr[31]}}, ifid_instr[31], ifid_instr[19:12],
                           ifid_instr[20], ifid_instr[30:21], 1'b0};
            default: id_imm = 32'd0;
        endcase
    end

    // Control signals
    reg       id_reg_write, id_mem_read, id_mem_write, id_mem_to_reg;
    reg       id_alu_src, id_is_branch, id_is_jal, id_is_jalr, id_is_lui, id_is_auipc;
    reg [3:0] id_alu_op;

    always @(*) begin
        // Defaults
        id_reg_write = 1'b0; id_mem_read = 1'b0; id_mem_write = 1'b0;
        id_mem_to_reg = 1'b0; id_alu_src = 1'b0;
        id_is_branch = 1'b0; id_is_jal = 1'b0; id_is_jalr = 1'b0;
        id_is_lui = 1'b0; id_is_auipc = 1'b0;
        id_alu_op = 4'd0;

        case (id_opcode)
            7'b0110011: begin // R-type (ADD, SUB, MUL, etc.)
                id_reg_write = 1'b1;
                if (id_funct7 == 7'b0000001) // MUL
                    id_alu_op = 4'd10;
                else case (id_funct3)
                    3'b000: id_alu_op = (id_funct7[5]) ? 4'd1 : 4'd0; // SUB/ADD
                    3'b001: id_alu_op = 4'd5; // SLL
                    3'b010: id_alu_op = 4'd2; // SLT
                    3'b011: id_alu_op = 4'd3; // SLTU
                    3'b100: id_alu_op = 4'd4; // XOR
                    3'b101: id_alu_op = (id_funct7[5]) ? 4'd7 : 4'd6; // SRA/SRL
                    3'b110: id_alu_op = 4'd8; // OR
                    3'b111: id_alu_op = 4'd9; // AND
                endcase
            end
            7'b0010011: begin // I-type ALU
                id_reg_write = 1'b1;
                id_alu_src   = 1'b1;
                case (id_funct3)
                    3'b000: id_alu_op = 4'd0; // ADDI
                    3'b001: id_alu_op = 4'd5; // SLLI
                    3'b010: id_alu_op = 4'd2; // SLTI
                    3'b011: id_alu_op = 4'd3; // SLTIU
                    3'b100: id_alu_op = 4'd4; // XORI
                    3'b101: id_alu_op = (id_funct7[5]) ? 4'd7 : 4'd6; // SRAI/SRLI
                    3'b110: id_alu_op = 4'd8; // ORI
                    3'b111: id_alu_op = 4'd9; // ANDI
                endcase
            end
            7'b0000011: begin // LW
                id_reg_write = 1'b1; id_mem_read = 1'b1;
                id_mem_to_reg = 1'b1; id_alu_src = 1'b1;
                id_alu_op = 4'd0;
            end
            7'b0100011: begin // SW
                id_mem_write = 1'b1; id_alu_src = 1'b1;
                id_alu_op = 4'd0;
            end
            7'b1100011: begin // Branch
                id_is_branch = 1'b1;
                id_alu_op = 4'd1; // SUB for comparison
            end
            7'b1101111: begin // JAL
                id_is_jal = 1'b1; id_reg_write = 1'b1;
            end
            7'b1100111: begin // JALR
                id_is_jalr = 1'b1; id_reg_write = 1'b1;
                id_alu_src = 1'b1; id_alu_op = 4'd0;
            end
            7'b0110111: begin // LUI
                id_is_lui = 1'b1; id_reg_write = 1'b1;
            end
            7'b0010111: begin // AUIPC
                id_is_auipc = 1'b1; id_reg_write = 1'b1;
            end
        endcase
    end

    // ========================================================================
    // Load-use hazard detection
    // ========================================================================
    assign load_use_stall = idex_mem_read && idex_valid &&
                            ((idex_rd == id_rs1 && id_rs1 != 5'd0) ||
                             (idex_rd == id_rs2 && id_rs2 != 5'd0 && !id_alu_src));

    assign flush_if_id = branch_taken;
    assign flush_id_ex = branch_taken || load_use_stall;

    // ========================================================================
    // ID/EX Pipeline Register
    // ========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst || (flush_id_ex && !stall)) begin
            idex_pc <= 32'd0; idex_rs1_data <= 32'd0; idex_rs2_data <= 32'd0;
            idex_imm <= 32'd0; idex_rd <= 5'd0; idex_rs1 <= 5'd0; idex_rs2 <= 5'd0;
            idex_funct3 <= 3'd0; idex_funct7 <= 7'd0;
            idex_reg_write <= 1'b0; idex_mem_read <= 1'b0; idex_mem_write <= 1'b0;
            idex_mem_to_reg <= 1'b0; idex_alu_src <= 1'b0;
            idex_is_branch <= 1'b0; idex_is_jal <= 1'b0; idex_is_jalr <= 1'b0;
            idex_is_lui <= 1'b0; idex_is_auipc <= 1'b0;
            idex_alu_op <= 4'd0; idex_valid <= 1'b0;
        end else if (!stall) begin
            idex_pc <= ifid_pc; idex_rs1_data <= rs1_data; idex_rs2_data <= rs2_data;
            idex_imm <= id_imm; idex_rd <= id_rd; idex_rs1 <= id_rs1; idex_rs2 <= id_rs2;
            idex_funct3 <= id_funct3; idex_funct7 <= id_funct7;
            idex_reg_write <= id_reg_write; idex_mem_read <= id_mem_read;
            idex_mem_write <= id_mem_write; idex_mem_to_reg <= id_mem_to_reg;
            idex_alu_src <= id_alu_src; idex_is_branch <= id_is_branch;
            idex_is_jal <= id_is_jal; idex_is_jalr <= id_is_jalr;
            idex_is_lui <= id_is_lui; idex_is_auipc <= id_is_auipc;
            idex_alu_op <= id_alu_op; idex_valid <= ifid_valid;
        end
    end

    // ========================================================================
    // EX Stage
    // ========================================================================
    // Forwarding muxes
    reg [31:0] ex_fwd_a, ex_fwd_b;

    always @(*) begin
        // Forward A (rs1)
        if (exmem_reg_write && exmem_rd != 5'd0 && exmem_rd == idex_rs1)
            ex_fwd_a = exmem_mem_to_reg ? cpu_mem_rdata : exmem_alu_result;
        else if (wb_reg_write && wb_rd != 5'd0 && wb_rd == idex_rs1)
            ex_fwd_a = wb_write_data;
        else
            ex_fwd_a = idex_rs1_data;

        // Forward B (rs2)
        if (exmem_reg_write && exmem_rd != 5'd0 && exmem_rd == idex_rs2)
            ex_fwd_b = exmem_mem_to_reg ? cpu_mem_rdata : exmem_alu_result;
        else if (wb_reg_write && wb_rd != 5'd0 && wb_rd == idex_rs2)
            ex_fwd_b = wb_write_data;
        else
            ex_fwd_b = idex_rs2_data;
    end

    // ALU input B selection
    wire [31:0] alu_in_b = idex_alu_src ? idex_imm : ex_fwd_b;

    // ALU
    reg [31:0] alu_result;
    always @(*) begin
        case (idex_alu_op)
            4'd0:  alu_result = ex_fwd_a + alu_in_b;                          // ADD
            4'd1:  alu_result = ex_fwd_a - alu_in_b;                          // SUB
            4'd2:  alu_result = {31'd0, $signed(ex_fwd_a) < $signed(alu_in_b)}; // SLT
            4'd3:  alu_result = {31'd0, ex_fwd_a < alu_in_b};                 // SLTU
            4'd4:  alu_result = ex_fwd_a ^ alu_in_b;                          // XOR
            4'd5:  alu_result = ex_fwd_a << alu_in_b[4:0];                    // SLL
            4'd6:  alu_result = ex_fwd_a >> alu_in_b[4:0];                    // SRL
            4'd7:  alu_result = $signed(ex_fwd_a) >>> alu_in_b[4:0];          // SRA
            4'd8:  alu_result = ex_fwd_a | alu_in_b;                          // OR
            4'd9:  alu_result = ex_fwd_a & alu_in_b;                          // AND
            4'd10: alu_result = ex_fwd_a * alu_in_b;                          // MUL
            default: alu_result = 32'd0;
        endcase
    end

    // EX result selection (LUI, AUIPC, JAL/JALR override ALU)
    reg [31:0] ex_result;
    always @(*) begin
        if (idex_is_lui)
            ex_result = idex_imm;
        else if (idex_is_auipc)
            ex_result = idex_pc + idex_imm;
        else if (idex_is_jal || idex_is_jalr)
            ex_result = idex_pc + 32'd4; // return address
        else
            ex_result = alu_result;
    end

    // Branch logic
    reg branch_cond;
    always @(*) begin
        case (idex_funct3)
            3'b000: branch_cond = (ex_fwd_a == ex_fwd_b);                          // BEQ
            3'b001: branch_cond = (ex_fwd_a != ex_fwd_b);                          // BNE
            3'b100: branch_cond = ($signed(ex_fwd_a) < $signed(ex_fwd_b));          // BLT
            3'b101: branch_cond = ($signed(ex_fwd_a) >= $signed(ex_fwd_b));         // BGE
            3'b110: branch_cond = (ex_fwd_a < ex_fwd_b);                           // BLTU
            3'b111: branch_cond = (ex_fwd_a >= ex_fwd_b);                          // BGEU
            default: branch_cond = 1'b0;
        endcase
    end

    assign branch_taken = (idex_is_branch && branch_cond) || idex_is_jal || idex_is_jalr;
    assign branch_target = idex_is_jalr ? (ex_fwd_a + idex_imm) & ~32'd1
                                        : idex_pc + idex_imm;

    // ========================================================================
    // EX/MEM Pipeline Register
    // ========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            exmem_alu_result <= 32'd0; exmem_rs2_data <= 32'd0;
            exmem_rd <= 5'd0; exmem_reg_write <= 1'b0;
            exmem_mem_read <= 1'b0; exmem_mem_write <= 1'b0;
            exmem_mem_to_reg <= 1'b0; exmem_valid <= 1'b0;
        end else if (!stall) begin
            exmem_alu_result <= ex_result;
            exmem_rs2_data   <= ex_fwd_b;
            exmem_rd         <= idex_rd;
            exmem_reg_write  <= idex_reg_write && idex_valid;
            exmem_mem_read   <= idex_mem_read && idex_valid;
            exmem_mem_write  <= idex_mem_write && idex_valid;
            exmem_mem_to_reg <= idex_mem_to_reg;
            exmem_valid      <= idex_valid;
        end
    end

    // ========================================================================
    // MEM Stage - Data Cache
    // ========================================================================
    assign cpu_mem_req   = exmem_mem_read || exmem_mem_write;
    assign cpu_mem_wr    = exmem_mem_write;
    assign cpu_mem_addr  = exmem_alu_result;
    assign cpu_mem_wdata = exmem_rs2_data;

    data_cache dcache (
        .clk       (clk),
        .rst       (rst),
        .cpu_req   (cpu_mem_req),
        .cpu_wr    (cpu_mem_wr),
        .cpu_addr  (cpu_mem_addr),
        .cpu_wdata (cpu_mem_wdata),
        .cpu_rdata (cpu_mem_rdata),
        .stall     (cache_stall),
        .mem_req   (cache_mem_req),
        .mem_wr    (cache_mem_wr),
        .mem_addr  (cache_mem_addr),
        .mem_wdata (cache_mem_wdata),
        .mem_rdata (mem_rdata),
        .mem_ready (mem_ready),
        .hit_count (cache_hits),
        .miss_count(cache_misses)
    );

    main_memory #(.MEM_DEPTH(1024), .LATENCY(10)) dmem (
        .clk   (clk),
        .rst   (rst),
        .req   (cache_mem_req),
        .wr    (cache_mem_wr),
        .addr  (cache_mem_addr),
        .wdata (cache_mem_wdata),
        .rdata (mem_rdata),
        .ready (mem_ready)
    );

    // ========================================================================
    // MEM/WB Pipeline Register
    // ========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            memwb_alu_result <= 32'd0; memwb_mem_data <= 32'd0;
            memwb_rd <= 5'd0; memwb_reg_write <= 1'b0; memwb_mem_to_reg <= 1'b0;
        end else if (!stall) begin
            memwb_alu_result <= exmem_alu_result;
            memwb_mem_data   <= cpu_mem_rdata;
            memwb_rd         <= exmem_rd;
            memwb_reg_write  <= exmem_reg_write;
            memwb_mem_to_reg <= exmem_mem_to_reg;
        end
    end

    // ========================================================================
    // WB Stage
    // ========================================================================
    assign wb_rd = memwb_rd;
    assign wb_reg_write = memwb_reg_write;
    assign wb_write_data = memwb_mem_to_reg ? memwb_mem_data : memwb_alu_result;

    // Halt detection: JAL x0, 0 (self-loop) detected in EX stage
    // Using EX stage avoids race conditions with speculative fetches in IF/ID
    assign halt = idex_is_jal && (idex_rd == 5'd0) && (idex_imm == 32'd0) && idex_valid;

endmodule
