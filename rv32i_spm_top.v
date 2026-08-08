// ============================================================================
// RV32I 5-Stage Pipeline with SRAM Scratchpad + Custom DMA Instruction
// Custom instruction SPM_DMA: opcode=0001011 (custom-0), R-type
//   funct7[6]=0: mem->spm, funct7[6]=1: spm->mem
//   rs1=src_addr, rs2=dst_addr, rd=word_count_reg
// Scratchpad address range: 0x00010000-0x000103FF
// ============================================================================
module rv32i_spm_top (
    input  wire        clk,
    input  wire        rst,
    output wire        halt,
    output wire [31:0] cycle_count,
    output wire [31:0] dma_cycle_count
);

    // Performance counters
    reg dma_active;
    reg [31:0] cycles, dma_cycles;
    assign cycle_count = cycles;
    assign dma_cycle_count = dma_cycles;
    always @(posedge clk or posedge rst)
        if (rst) begin cycles <= 0; dma_cycles <= 0; end
        else begin
            cycles <= cycles + 1;
            if (dma_active) dma_cycles <= dma_cycles + 1;
        end

    // Instruction Memory
    reg [31:0] imem [0:255];
    wire [31:0] instr_addr;
    wire [31:0] instruction = imem[instr_addr[9:2]];

    // Stall/flush signals
    wire stall, load_use_stall, flush_if_id, flush_id_ex;
    wire dma_stall;
    wire mem_access_stall;
    assign stall = dma_stall || mem_access_stall;

    // ========================================================================
    // Pipeline Registers
    // ========================================================================
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
    reg        idex_is_lui, idex_is_auipc, idex_is_spm_dma;
    reg [3:0]  idex_alu_op;
    reg        idex_valid;

    // EX/MEM
    reg [31:0] exmem_alu_result, exmem_rs2_data;
    reg [31:0] exmem_rs1_fwd, exmem_rs2_fwd;
    reg [4:0]  exmem_rd;
    reg        exmem_reg_write, exmem_mem_read, exmem_mem_write, exmem_mem_to_reg;
    reg        exmem_is_spm_dma;
    reg [6:0]  exmem_funct7;
    reg        exmem_valid;

    // MEM/WB
    reg [31:0] memwb_alu_result, memwb_mem_data;
    reg [4:0]  memwb_rd;
    reg        memwb_reg_write, memwb_mem_to_reg;

    wire [4:0]  wb_rd;
    wire        wb_reg_write;
    wire [31:0] wb_write_data;

    // Register File
    reg [31:0] regfile [0:31];
    integer ri;
    always @(posedge clk or posedge rst)
        if (rst) for (ri = 0; ri < 32; ri = ri + 1) regfile[ri] <= 32'd0;
        else if (wb_reg_write && wb_rd != 5'd0)
            regfile[wb_rd] <= wb_write_data;

    wire [4:0]  id_rs1 = ifid_instr[19:15];
    wire [4:0]  id_rs2 = ifid_instr[24:20];
    wire [31:0] rs1_data = (id_rs1 == 5'd0) ? 32'd0 : regfile[id_rs1];
    wire [31:0] rs2_data = (id_rs2 == 5'd0) ? 32'd0 : regfile[id_rs2];

    // ========================== IF Stage ==========================
    reg [31:0] pc;
    assign instr_addr = pc;
    wire [31:0] pc_plus4 = pc + 32'd4;
    wire branch_taken;
    wire [31:0] branch_target;

    always @(posedge clk or posedge rst)
        if (rst) pc <= 32'd0;
        else if (!stall && !load_use_stall)
            pc <= branch_taken ? branch_target : pc_plus4;

    // ========================================================================
    // IF/ID Pipeline Register
    // ========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst || (flush_if_id && !stall)) begin
            ifid_instr <= 32'h00000013; ifid_pc <= 0; ifid_valid <= 0;
        end else if (!stall && !load_use_stall) begin
            ifid_instr <= instruction; ifid_pc <= pc; ifid_valid <= 1;
        end
    end

    // ========================== ID Stage ==========================
    wire [6:0] id_opcode = ifid_instr[6:0];
    wire [4:0] id_rd     = ifid_instr[11:7];
    wire [2:0] id_funct3 = ifid_instr[14:12];
    wire [6:0] id_funct7 = ifid_instr[31:25];

    // Immediate generation
    reg [31:0] id_imm;
    always @(*) begin
        case (id_opcode)
            7'b0010011, 7'b0000011, 7'b1100111:
                id_imm = {{20{ifid_instr[31]}}, ifid_instr[31:20]};
            7'b0100011:
                id_imm = {{20{ifid_instr[31]}}, ifid_instr[31:25], ifid_instr[11:7]};
            7'b1100011:
                id_imm = {{19{ifid_instr[31]}}, ifid_instr[31], ifid_instr[7],
                           ifid_instr[30:25], ifid_instr[11:8], 1'b0};
            7'b0110111, 7'b0010111:
                id_imm = {ifid_instr[31:12], 12'd0};
            7'b1101111:
                id_imm = {{11{ifid_instr[31]}}, ifid_instr[31], ifid_instr[19:12],
                           ifid_instr[20], ifid_instr[30:21], 1'b0};
            default: id_imm = 32'd0;
        endcase
    end

    // Control signals
    reg id_reg_write, id_mem_read, id_mem_write, id_mem_to_reg;
    reg id_alu_src, id_is_branch, id_is_jal, id_is_jalr, id_is_lui, id_is_auipc;
    reg id_is_spm_dma;
    reg [3:0] id_alu_op;

    always @(*) begin
        id_reg_write=0; id_mem_read=0; id_mem_write=0; id_mem_to_reg=0;
        id_alu_src=0; id_is_branch=0; id_is_jal=0; id_is_jalr=0;
        id_is_lui=0; id_is_auipc=0; id_is_spm_dma=0; id_alu_op=4'd0;
        case (id_opcode)
            7'b0110011: begin
                id_reg_write = 1;
                if (id_funct7 == 7'b0000001) id_alu_op = 4'd10;
                else case (id_funct3)
                    3'b000: id_alu_op = id_funct7[5] ? 4'd1 : 4'd0;
                    3'b001: id_alu_op = 4'd5;
                    3'b010: id_alu_op = 4'd2;
                    3'b011: id_alu_op = 4'd3;
                    3'b100: id_alu_op = 4'd4;
                    3'b101: id_alu_op = id_funct7[5] ? 4'd7 : 4'd6;
                    3'b110: id_alu_op = 4'd8;
                    3'b111: id_alu_op = 4'd9;
                endcase
            end
            7'b0010011: begin
                id_reg_write = 1; id_alu_src = 1;
                case (id_funct3)
                    3'b000: id_alu_op = 4'd0; 3'b001: id_alu_op = 4'd5;
                    3'b010: id_alu_op = 4'd2; 3'b011: id_alu_op = 4'd3;
                    3'b100: id_alu_op = 4'd4;
                    3'b101: id_alu_op = id_funct7[5] ? 4'd7 : 4'd6;
                    3'b110: id_alu_op = 4'd8; 3'b111: id_alu_op = 4'd9;
                endcase
            end
            7'b0000011: begin id_reg_write=1; id_mem_read=1; id_mem_to_reg=1; id_alu_src=1; end
            7'b0100011: begin id_mem_write=1; id_alu_src=1; end
            7'b1100011: begin id_is_branch=1; id_alu_op=4'd1; end
            7'b1101111: begin id_is_jal=1; id_reg_write=1; end
            7'b1100111: begin id_is_jalr=1; id_reg_write=1; id_alu_src=1; end
            7'b0110111: begin id_is_lui=1; id_reg_write=1; end
            7'b0010111: begin id_is_auipc=1; id_reg_write=1; end
            7'b0001011: begin id_is_spm_dma=1; end // Custom SPM_DMA
        endcase
    end

    // Hazard detection
    assign load_use_stall = idex_mem_read && idex_valid &&
        ((idex_rd == id_rs1 && id_rs1 != 0) ||
         (idex_rd == id_rs2 && id_rs2 != 0 && !id_alu_src));
    assign flush_if_id = branch_taken;
    assign flush_id_ex = branch_taken || load_use_stall;

    // ========================================================================
    // ID/EX Pipeline Register
    // ========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst || (flush_id_ex && !stall)) begin
            idex_pc<=0; idex_rs1_data<=0; idex_rs2_data<=0; idex_imm<=0;
            idex_rd<=0; idex_rs1<=0; idex_rs2<=0; idex_funct3<=0; idex_funct7<=0;
            idex_reg_write<=0; idex_mem_read<=0; idex_mem_write<=0; idex_mem_to_reg<=0;
            idex_alu_src<=0; idex_is_branch<=0; idex_is_jal<=0; idex_is_jalr<=0;
            idex_is_lui<=0; idex_is_auipc<=0; idex_is_spm_dma<=0;
            idex_alu_op<=0; idex_valid<=0;
        end else if (!stall) begin
            idex_pc<=ifid_pc; idex_rs1_data<=rs1_data; idex_rs2_data<=rs2_data;
            idex_imm<=id_imm; idex_rd<=id_rd; idex_rs1<=id_rs1; idex_rs2<=id_rs2;
            idex_funct3<=id_funct3; idex_funct7<=id_funct7;
            idex_reg_write<=id_reg_write; idex_mem_read<=id_mem_read;
            idex_mem_write<=id_mem_write; idex_mem_to_reg<=id_mem_to_reg;
            idex_alu_src<=id_alu_src; idex_is_branch<=id_is_branch;
            idex_is_jal<=id_is_jal; idex_is_jalr<=id_is_jalr;
            idex_is_lui<=id_is_lui; idex_is_auipc<=id_is_auipc;
            idex_is_spm_dma<=id_is_spm_dma;
            idex_alu_op<=id_alu_op; idex_valid<=ifid_valid;
        end
    end

    // ========================== EX Stage ==========================
    wire [31:0] mem_stage_rdata; // Declare here for forwarding
    reg [31:0] ex_fwd_a, ex_fwd_b;
    always @(*) begin
        // Forward A (rs1)
        if (exmem_reg_write && exmem_rd != 5'd0 && exmem_rd == idex_rs1)
            ex_fwd_a = exmem_mem_to_reg ? mem_stage_rdata : exmem_alu_result;
        else if (wb_reg_write && wb_rd != 5'd0 && wb_rd == idex_rs1)
            ex_fwd_a = wb_write_data;
        else
            ex_fwd_a = idex_rs1_data;

        // Forward B (rs2)
        if (exmem_reg_write && exmem_rd != 5'd0 && exmem_rd == idex_rs2)
            ex_fwd_b = exmem_mem_to_reg ? mem_stage_rdata : exmem_alu_result;
        else if (wb_reg_write && wb_rd != 5'd0 && wb_rd == idex_rs2)
            ex_fwd_b = wb_write_data;
        else ex_fwd_b = idex_rs2_data;
    end

    wire [31:0] alu_in_b = idex_alu_src ? idex_imm : ex_fwd_b;

    reg [31:0] alu_result;
    always @(*) begin
        case (idex_alu_op)
            4'd0:  alu_result = ex_fwd_a + alu_in_b;
            4'd1:  alu_result = ex_fwd_a - alu_in_b;
            4'd2:  alu_result = {31'd0, $signed(ex_fwd_a) < $signed(alu_in_b)};
            4'd3:  alu_result = {31'd0, ex_fwd_a < alu_in_b};
            4'd4:  alu_result = ex_fwd_a ^ alu_in_b;
            4'd5:  alu_result = ex_fwd_a << alu_in_b[4:0];
            4'd6:  alu_result = ex_fwd_a >> alu_in_b[4:0];
            4'd7:  alu_result = $signed(ex_fwd_a) >>> alu_in_b[4:0];
            4'd8:  alu_result = ex_fwd_a | alu_in_b;
            4'd9:  alu_result = ex_fwd_a & alu_in_b;
            4'd10: alu_result = ex_fwd_a * alu_in_b;
            default: alu_result = 0;
        endcase
    end

    reg [31:0] ex_result;
    always @(*) begin
        if (idex_is_lui) ex_result = idex_imm;
        else if (idex_is_auipc) ex_result = idex_pc + idex_imm;
        else if (idex_is_jal || idex_is_jalr) ex_result = idex_pc + 32'd4;
        else ex_result = alu_result;
    end

    reg branch_cond;
    always @(*) begin
        case (idex_funct3)
            3'b000: branch_cond = (ex_fwd_a == ex_fwd_b);
            3'b001: branch_cond = (ex_fwd_a != ex_fwd_b);
            3'b100: branch_cond = ($signed(ex_fwd_a) < $signed(ex_fwd_b));
            3'b101: branch_cond = ($signed(ex_fwd_a) >= $signed(ex_fwd_b));
            3'b110: branch_cond = (ex_fwd_a < ex_fwd_b);
            3'b111: branch_cond = (ex_fwd_a >= ex_fwd_b);
            default: branch_cond = 0;
        endcase
    end
    assign branch_taken = (idex_is_branch && branch_cond) || idex_is_jal || idex_is_jalr;
    assign branch_target = idex_is_jalr ? (ex_fwd_a+idex_imm) & ~32'd1 : idex_pc+idex_imm;

    // ========================================================================
    // EX/MEM Pipeline Register
    // ========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            exmem_alu_result<=0; exmem_rs2_data<=0; exmem_rd<=0;
            exmem_reg_write<=0; exmem_mem_read<=0; exmem_mem_write<=0;
            exmem_mem_to_reg<=0; exmem_is_spm_dma<=0; exmem_valid<=0;
            exmem_funct7<=0; exmem_rs1_fwd<=0; exmem_rs2_fwd<=0;
        end else if (!stall) begin
            exmem_alu_result <= ex_result;
            exmem_rs2_data <= ex_fwd_b;
            exmem_rs1_fwd <= ex_fwd_a;
            exmem_rs2_fwd <= ex_fwd_b;
            exmem_rd <= idex_rd;
            exmem_reg_write <= idex_reg_write && idex_valid;
            exmem_mem_read <= idex_mem_read && idex_valid;
            exmem_mem_write <= idex_mem_write && idex_valid;
            exmem_mem_to_reg <= idex_mem_to_reg;
            exmem_is_spm_dma <= idex_is_spm_dma && idex_valid;
            exmem_valid <= idex_valid;
            exmem_funct7 <= idex_funct7;
        end
    end

    // ========================== MEM Stage ==========================
    // Address decode: scratchpad if addr[31:16] == 16'h0001
    wire is_spm_addr = (exmem_alu_result[31:16] == 16'h0001);

    // Scratchpad signals
    wire spm_cpu_req = (exmem_mem_read || exmem_mem_write) && is_spm_addr;
    wire spm_cpu_wr  = exmem_mem_write && is_spm_addr;

    // Main memory signals (for regular non-SPM load/store)
    wire main_mem_req_ls = (exmem_mem_read || exmem_mem_write) && !is_spm_addr;
    wire main_mem_wr_ls  = exmem_mem_write && !is_spm_addr;

    // DMA controller
    // DMA controller FSM state and registers
    reg dma_direction;  // 0=mem->spm, 1=spm->mem
    reg [31:0] dma_src, dma_dst;
    reg [31:0] dma_count, dma_remaining;
    reg [2:0] dma_state;
    localparam DMA_IDLE = 0, DMA_MEM_RD = 1, DMA_SPM_WR = 2,
               DMA_SPM_RD = 3, DMA_MEM_WR = 4;

    // Main memory interface
    reg mem_req, mem_wr;
    reg [31:0] mem_addr, mem_wdata;
    wire [31:0] mem_rdata;
    wire mem_ready;

    // DMA scratchpad interface
    reg dma_spm_req, dma_spm_wr;
    reg [7:0] dma_spm_addr_reg;
    reg [31:0] dma_spm_wdata;
    wire [31:0] dma_spm_rdata;

    // Scratchpad read data for CPU
    wire [31:0] spm_cpu_rdata;

    assign dma_stall = dma_active;
    assign mem_access_stall = main_mem_req_ls && !mem_ready && !dma_active;

    // DMA FSM
    reg [31:0] dma_buf;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            dma_active<=0; dma_state<=DMA_IDLE; dma_remaining<=0;
            dma_spm_req<=0; dma_spm_wr<=0; dma_spm_addr_reg<=0;
            dma_spm_wdata<=0; mem_req<=0; mem_wr<=0; mem_addr<=0; mem_wdata<=0;
            dma_buf<=0; dma_src<=0; dma_dst<=0; dma_direction<=0; dma_count<=0;
        end else begin
            dma_spm_req <= 0;
            case (dma_state)
                DMA_IDLE: begin
                    if (exmem_is_spm_dma && !dma_active) begin
                        dma_active <= 1;
                        dma_direction <= exmem_funct7[6];
                        dma_src <= exmem_rs1_fwd;
                        dma_dst <= exmem_rs2_fwd;
                        dma_count <= regfile[exmem_rd];
                        dma_remaining <= regfile[exmem_rd];
                        if (!exmem_funct7[6]) begin
                            // mem -> spm: read from main memory first
                            dma_state <= DMA_MEM_RD;
                            mem_req <= 1; mem_wr <= 0;
                            mem_addr <= exmem_rs1_fwd;
                        end else begin
                            // spm -> mem: read from scratchpad first
                            dma_state <= DMA_SPM_RD;
                            dma_spm_req <= 1; dma_spm_wr <= 0;
                            dma_spm_addr_reg <= exmem_rs1_fwd[9:2];
                        end
                    end else if (!exmem_is_spm_dma && main_mem_req_ls && !dma_active) begin
                        // Regular memory access
                        mem_req <= 1;
                        mem_wr <= main_mem_wr_ls;
                        mem_addr <= exmem_alu_result;
                        mem_wdata <= exmem_rs2_data;
                    end else if (mem_ready && !dma_active) begin
                        mem_req <= 0;
                    end
                end

                DMA_MEM_RD: begin
                    if (mem_ready) begin
                        dma_buf <= mem_rdata;
                        mem_req <= 0;
                        dma_state <= DMA_SPM_WR;
                        dma_spm_req <= 1; dma_spm_wr <= 1;
                        dma_spm_addr_reg <= dma_dst[9:2];
                        dma_spm_wdata <= mem_rdata;
                    end
                end

                DMA_SPM_WR: begin
                    dma_remaining <= dma_remaining - 1;
                    dma_src <= dma_src + 4;
                    dma_dst <= dma_dst + 4;
                    if (dma_remaining == 1) begin
                        dma_state <= DMA_IDLE;
                        dma_active <= 0;
                        mem_req <= 0;
                    end else begin
                        dma_state <= DMA_MEM_RD;
                        mem_req <= 1; mem_wr <= 0;
                        mem_addr <= dma_src + 4;
                    end
                end

                DMA_SPM_RD: begin
                    // scratchpad read takes 1 cycle
                    dma_buf <= dma_spm_rdata;
                    dma_state <= DMA_MEM_WR;
                    mem_req <= 1; mem_wr <= 1;
                    mem_addr <= dma_dst;
                    mem_wdata <= dma_spm_rdata;
                end

                DMA_MEM_WR: begin
                    if (mem_ready) begin
                        mem_req <= 0;
                        dma_remaining <= dma_remaining - 1;
                        dma_src <= dma_src + 4;
                        dma_dst <= dma_dst + 4;
                        if (dma_remaining == 1) begin
                            dma_state <= DMA_IDLE;
                            dma_active <= 0;
                        end else begin
                            dma_state <= DMA_SPM_RD;
                            dma_spm_req <= 1; dma_spm_wr <= 0;
                            dma_spm_addr_reg <= (dma_src + 4) >> 2;
                        end
                    end
                end
            endcase
        end
    end

    // Scratchpad instance
    scratchpad spm_inst (
        .clk(clk), .rst(rst),
        .cpu_req(spm_cpu_req), .cpu_wr(spm_cpu_wr),
        .cpu_addr(exmem_alu_result), .cpu_wdata(exmem_rs2_data),
        .cpu_rdata(spm_cpu_rdata),
        .dma_req(dma_spm_req), .dma_wr(dma_spm_wr),
        .dma_spm_addr(dma_spm_addr_reg), .dma_wdata(dma_spm_wdata),
        .dma_rdata(dma_spm_rdata)
    );

    // Main memory instance
    main_memory #(.MEM_DEPTH(1024), .LATENCY(10)) dmem (
        .clk(clk), .rst(rst),
        .req(mem_req), .wr(mem_wr),
        .addr(mem_addr), .wdata(mem_wdata),
        .rdata(mem_rdata), .ready(mem_ready)
    );

    // MEM stage read data mux
    assign mem_stage_rdata = is_spm_addr ? spm_cpu_rdata : mem_rdata;

    // ========================================================================
    // MEM/WB Pipeline Register
    // ========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            memwb_alu_result<=0; memwb_mem_data<=0; memwb_rd<=0;
            memwb_reg_write<=0; memwb_mem_to_reg<=0;
        end else if (!stall) begin
            memwb_alu_result <= exmem_alu_result;
            memwb_mem_data <= mem_stage_rdata;
            memwb_rd <= exmem_rd;
            memwb_reg_write <= exmem_reg_write;
            memwb_mem_to_reg <= exmem_mem_to_reg;
        end
    end

    // ========================== WB Stage ==========================
    assign wb_rd = memwb_rd;
    assign wb_reg_write = memwb_reg_write;
    assign wb_write_data = memwb_mem_to_reg ? memwb_mem_data : memwb_alu_result;

    // Halt detection: JAL x0, 0 (self-loop) detected in EX stage
    assign halt = idex_is_jal && (idex_rd == 5'd0) && (idex_imm == 32'd0) && idex_valid;

endmodule
