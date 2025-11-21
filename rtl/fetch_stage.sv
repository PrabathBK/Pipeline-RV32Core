/*
* RISC-V Fetch Stage
*
* This module implements the instruction fetch stage of the RISC-V pipeline.
* It handles:
* - PC generation and updating
* - Instruction memory interface
* - Branch/jump target handling
* - Pipeline stall and flush handling
* - Instruction prefetching (optional, configurable)
*
* Parameters:
* - RESET_ADDR: Address to jump to on reset (default: 0x00000000)
* - ENABLE_PREFETCH: Enable instruction prefetching (default: 1)
*/

module fetch_stage #(
    parameter RESET_ADDR      = 32'h00000000,
    parameter ENABLE_PREFETCH = 1
) (
    input logic clk,   // System clock
    input logic rst_n, // Active-low reset

    // Pipeline control signals
    input logic        pipeline_stall,  // Stall signal from hazard unit
    input logic        pipeline_flush,  // Flush signal from hazard unit
    input logic        branch_taken,    // Branch taken signal from execute stage
    input logic [31:0] branch_target,   // Branch target address from execute stage

    // Instruction memory interface
    output logic [31:0] imem_addr,   // Instruction memory address
    input  logic [31:0] imem_data,   // Instruction memory data
    output logic        imem_read,   // Instruction memory read enable
    input  logic        imem_valid,  // Instruction memory valid signal
    input  logic        imem_error,  // Instruction memory error signal

    // Output to decode stage
    output logic [31:0] if_pc,     // Current PC
    output logic [31:0] if_instr,  // Current instruction
    output logic        if_valid,  // Instruction valid
    output logic        if_error   // Instruction error (e.g., misaligned fetch)
);

    // Internal signals
    logic [31:0] next_pc;  // Next PC value
    logic [31:0] pc_reg;  // PC register
    logic        fetch_ready;  // Ready to fetch next instruction
    logic        misaligned_pc;  // PC alignment error detection

    // Optional prefetch buffer signals
    generate
        if (ENABLE_PREFETCH) begin : prefetch_buffer
            logic [31:0] prefetch_instr;
            logic        prefetch_valid;
            logic [31:0] prefetch_pc;
        end
    endgenerate

    // PC alignment error detection
    assign misaligned_pc = |pc_reg[1:0];  // RISC-V requires 4-byte aligned instructions

    // Next PC calculation
    always_comb begin
        if (branch_taken) begin
            next_pc = branch_target;
        end else if (!pipeline_stall && !misaligned_pc) begin
            next_pc = pc_reg + 32'h4;
        end else begin
            next_pc = pc_reg;
        end
    end

    // PC update logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc_reg <= RESET_ADDR;
        end else if (!pipeline_stall) begin
            pc_reg <= next_pc;
        end
    end

    // Instruction memory interface logic
    always_comb begin
        imem_addr   = pc_reg;
        imem_read   = !pipeline_stall && !misaligned_pc;
        fetch_ready = imem_valid && !pipeline_stall;
    end

    // Output registers
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            if_pc    <= RESET_ADDR;
            if_instr <= 32'h00000013;  // NOP instruction (addi x0, x0, 0)
            if_valid <= 1'b0;
            if_error <= 1'b0;
        end else if (pipeline_flush) begin
            if_pc    <= next_pc;
            if_instr <= 32'h00000013;  // NOP instruction
            if_valid <= 1'b0;
            if_error <= 1'b0;
        end else if (!pipeline_stall && fetch_ready) begin
            if_pc    <= pc_reg;
            if_instr <= imem_data;
            if_valid <= 1'b1;
            if_error <= imem_error || misaligned_pc;
        end
    end

    // Optional prefetch logic
    generate
        if (ENABLE_PREFETCH) begin : prefetch_logic
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    prefetch_buffer.prefetch_valid <= 1'b0;
                    prefetch_buffer.prefetch_instr <= 32'h0;
                    prefetch_buffer.prefetch_pc    <= RESET_ADDR;
                end else if (pipeline_flush) begin
                    prefetch_buffer.prefetch_valid <= 1'b0;
                end else if (!pipeline_stall && imem_valid) begin
                    prefetch_buffer.prefetch_valid <= 1'b1;
                    prefetch_buffer.prefetch_instr <= imem_data;
                    prefetch_buffer.prefetch_pc    <= pc_reg;
                end
            end
        end
    endgenerate

    // Error checking assertions
    // synthesis translate_off
    always_ff @(posedge clk) begin
        if (rst_n) begin
            // Check for misaligned PC
            assert (!misaligned_pc)
            else $error("Misaligned PC detected: %h", pc_reg);

            // Check for valid instruction memory interface
            assert (!(imem_valid && imem_error))
            else $error("Invalid imem interface state: valid and error both active");

            // Check for valid branch target alignment
            assert (!(branch_taken && |branch_target[1:0]))
            else $error("Misaligned branch target: %h", branch_target);
        end
    end
    // synthesis translate_on

endmodule

// Interface definition for fetch stage
interface fetch_stage_if;
    logic        clk;
    logic        rst_n;
    logic        pipeline_stall;
    logic        pipeline_flush;
    logic        branch_taken;
    logic [31:0] branch_target;
    logic [31:0] imem_addr;
    logic [31:0] imem_data;
    logic        imem_read;
    logic        imem_valid;
    logic        imem_error;
    logic [31:0] if_pc;
    logic [31:0] if_instr;
    logic        if_valid;
    logic        if_error;

    // Modport for fetch stage module
    modport fetch(
        input  clk, rst_n, pipeline_stall, pipeline_flush,
               branch_taken, branch_target, imem_data, imem_valid, imem_error,
        output imem_addr, imem_read, if_pc, if_instr, if_valid, if_error
    );

    // Modport for testbench
    modport test(
        output clk, rst_n, pipeline_stall, pipeline_flush,
               branch_taken, branch_target, imem_data, imem_valid, imem_error,
        input imem_addr, imem_read, if_pc, if_instr, if_valid, if_error
    );
endinterface
