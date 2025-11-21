module riscv_core (
    input logic clk,
    input logic rst_n,

    // Instruction memory interface
    output logic [31:0] imem_addr,
    input  logic [31:0] imem_data,
    output logic        imem_read,

    // Data memory interface
    output logic [31:0] dmem_addr,
    output logic [31:0] dmem_wdata,
    input  logic [31:0] dmem_rdata,
    output logic        dmem_read,
    output logic        dmem_write,
    output logic [ 3:0] dmem_byte_en,

    // UART interface for register monitoring
    output logic [31:0] reg_data,
    output logic [ 4:0] reg_addr,
    output logic        reg_valid
);

    // Pipeline stage registers
    logic [31:0] if_pc;
    logic [31:0] if_instr;

    logic [31:0] id_pc;
    logic [31:0] id_instr;
    logic [31:0] id_rs1_data;
    logic [31:0] id_rs2_data;
    logic [31:0] id_imm;

    logic [31:0] ex_pc;
    logic [31:0] ex_result;
    logic [31:0] ex_rs2_data;
    logic [ 4:0] ex_rd_addr;
    logic        ex_reg_write;
    logic        ex_mem_read;
    logic        ex_mem_write;

    logic [31:0] mem_result;
    logic [ 4:0] mem_rd_addr;
    logic        mem_reg_write;

    logic [31:0] wb_result;
    logic [ 4:0] wb_rd_addr;
    logic        wb_reg_write;

    // Register file
    logic [31:0] registers     [32];

    // Instantiate pipeline stages
    fetch_stage fetch (
        .clk(clk),
        .rst_n(rst_n),
        .pc(if_pc),
        .instr(if_instr),
        .imem_addr(imem_addr),
        .imem_data(imem_data),
        .imem_read(imem_read)
    );

    decode_stage decode (
        .clk(clk),
        .rst_n(rst_n),
        .instr(if_instr),
        .pc(if_pc),
        .registers(registers),
        .rs1_data(id_rs1_data),
        .rs2_data(id_rs2_data),
        .imm(id_imm)
    );

    execute_stage execute (
        .clk(clk),
        .rst_n(rst_n),
        .pc(id_pc),
        .rs1_data(id_rs1_data),
        .rs2_data(id_rs2_data),
        .imm(id_imm),
        .instr(id_instr),
        .result(ex_result),
        .rd_addr(ex_rd_addr),
        .reg_write(ex_reg_write),
        .mem_read(ex_mem_read),
        .mem_write(ex_mem_write)
    );

    memory_stage memory (
        .clk(clk),
        .rst_n(rst_n),
        .addr(ex_result),
        .wdata(ex_rs2_data),
        .rdata(dmem_rdata),
        .mem_read(ex_mem_read),
        .mem_write(ex_mem_write),
        .result(mem_result),
        .dmem_addr(dmem_addr),
        .dmem_wdata(dmem_wdata),
        .dmem_read(dmem_read),
        .dmem_write(dmem_write),
        .dmem_byte_en(dmem_byte_en)
    );

    writeback_stage writeback (
        .clk(clk),
        .rst_n(rst_n),
        .result(mem_result),
        .rd_addr(mem_rd_addr),
        .reg_write(mem_reg_write),
        .wb_result(wb_result),
        .wb_rd_addr(wb_rd_addr),
        .wb_reg_write(wb_reg_write)
    );

    // Register file write back
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 32; i++) begin
                registers[i] <= 32'h0;
            end
        end else if (wb_reg_write && wb_rd_addr != 5'h0) begin
            registers[wb_rd_addr] <= wb_result;
        end
    end

    // UART register monitoring
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_valid <= 1'b0;
            reg_addr  <= 5'h0;
            reg_data  <= 32'h0;
        end else begin
            reg_valid <= 1'b1;
            reg_addr  <= reg_addr + 1;
            reg_data  <= registers[reg_addr];
        end
    end

endmodule
