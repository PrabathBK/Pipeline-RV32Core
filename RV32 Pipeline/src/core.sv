`default_nettype none

// 5 stage pipeline
// rv32i base instruction set


module core (
    input  wire         clk,
    input  wire         resetn,
    output logic [31:0] IO_mem_addr,   // IO memory address
    input  wire  [31:0] IO_mem_rdata,  // data read from IO memory
    output logic [31:0] IO_mem_wdata,  // data written to IO memory
    output logic        IO_mem_wr      // IO write flag
);

    reg [31:0] progRom[16384];  // Program ROM (64 KB, 16384 words)
    reg [31:0] dataRam[16384];

`ifdef VERILATOR
    // initial $readmemh("firmware/hex/PROGROM.hex", progRom);
    initial $readmemh("firmware/hex/SIMPLE_P.hex", progRom);
`else
    initial $readmemh("/Users/tony/arc/pro/dev/svdev/rv32core/firmware/hex/PROGROM.hex", progRom);
`endif

`ifdef VERILATOR
    // initial $readmemh("firmware/hex/DATARAM.hex", dataRam);
    initial $readmemh("firmware/hex/SIMPLE_D.hex", dataRam);
`else
    initial $readmemh("/Users/tony/arc/pro/dev/svdev/rv32core/firmware/hex/DATARAM.hex", dataRam);
`endif
    // ─────────────────────────────────────────────────────────────────────────────
    // Instruction functions and decoders
    // ─────────────────────────────────────────────────────────────────────────────

    function automatic isALUreg;
        input [31:0] I;
        isALUreg = (I[6:0] == 7'b0110011);
    endfunction

    function automatic isALUimm;
        input [31:0] I;
        isALUimm = (I[6:0] == 7'b0010011);
    endfunction

    function automatic isBranch;
        input [31:0] I;
        isBranch = (I[6:0] == 7'b1100011);
    endfunction

    function automatic isJALR;
        input [31:0] I;
        isJALR = (I[6:0] == 7'b1100111);
    endfunction

    function automatic isJAL;
        input [31:0] I;
        isJAL = (I[6:0] == 7'b1101111);
    endfunction

    function automatic isAUIPC;
        input [31:0] I;
        isAUIPC = (I[6:0] == 7'b0010111);
    endfunction

    function automatic isLUI;
        input [31:0] I;
        isLUI = (I[6:0] == 7'b0110111);
    endfunction

    function automatic isLoad;
        input [31:0] I;
        isLoad = (I[6:0] == 7'b0000011);
    endfunction

    function automatic isStore;
        input [31:0] I;
        isStore = (I[6:0] == 7'b0100011);
    endfunction

    function automatic isSYSTEM;
        input [31:0] I;
        isSYSTEM = (I[6:0] == 7'b1110011);
    endfunction


    // REGISTER FIELD EXTRACTORS

    function automatic [4:0] rs1Id;
        input [31:0] I;
        rs1Id = I[19:15];
    endfunction

    function automatic [4:0] rs2Id;
        input [31:0] I;
        rs2Id = I[24:20];
    endfunction

    function automatic [4:0] shamt;
        input [31:0] I;
        shamt = I[24:20];
    endfunction

    function automatic [4:0] rdId;
        input [31:0] I;
        rdId = I[11:7];
    endfunction

    function automatic [1:0] csrId;
        input [31:0] I;
        csrId = {I[27], I[21]};
    endfunction


    // RISC-V function fields
    function automatic [2:0] funct3;
        input [31:0] I;
        funct3 = I[14:12];
    endfunction

    function automatic [6:0] funct7;
        input [31:0] I;
        funct7 = I[31:25];
    endfunction


    // Special instructions
    function automatic isEBREAK;
        input [31:0] I;
        isEBREAK = (isSYSTEM(I) && funct3(I) == 3'b000);
    endfunction

    function automatic isCSRRS;
        input [31:0] I;
        isCSRRS = (isSYSTEM(I) && funct3(I) == 3'b010);
    endfunction


    // The 5 immediate formats
    function automatic [31:0] Uimm;
        input [31:0] I;
        Uimm = {I[31:12], {12{1'b0}}};
    endfunction

    function automatic [31:0] Iimm;
        input [31:0] I;
        Iimm = {{21{I[31]}}, I[30:20]};
    endfunction

    function automatic [31:0] Simm;
        input [31:0] I;
        Simm = {{21{I[31]}}, I[30:25], I[11:7]};
    endfunction

    function automatic [31:0] Bimm;
        input [31:0] I;
        Bimm = {{20{I[31]}}, I[7], I[30:25], I[11:8], 1'b0};
    endfunction

    function automatic [31:0] Jimm;
        input [31:0] I;
        Jimm = {{12{I[31]}}, I[19:12], I[20], I[30:21], 1'b0};
    endfunction


    // Helper functions to check read/write usage
    function automatic writesRd;
        input [31:0] I;
        writesRd = !isStore(I) && !isBranch(I);
    endfunction

    function automatic readsRs1;
        input [31:0] I;
        // JAL, AUIPC, LUI do not read rs1
        readsRs1 = !(isJAL(I) || isAUIPC(I) || isLUI(I));
    endfunction

    function automatic readsRs2;
        input [31:0] I;
        // ALUreg, Branch, and Store read rs2
        readsRs2 = isALUreg(I) || isBranch(I) || isStore(I);
    endfunction


    // ─────────────────────────────────────────────────────────────────────────────
    // Internal pipeline signals and registers
    // ─────────────────────────────────────────────────────────────────────────────

    localparam logic NOP = 32'b0000000_00000_00000_000_00000_0110011;

    // Halt execution on EBREAK
    wire halt;

    // Flush signals for decode/execute stages
    wire decode_flush;
    wire exec_flush;

    // Stall signals for fetch/decode stages
    wire fetch_stall;
    wire decode_stall;

    // Cycle and retired instructions counters
    reg [63:0] cycle;
    reg [63:0] instret;  // Instruction retired

    // Update cycle counter
    always_ff @(posedge clk) begin
        if (!resetn) begin
            cycle <= 64'b0;
        end else begin
            cycle <= cycle + 1;
        end
    end

    // ─────────────────────────────────────────────────────────────────────────────
    // F: Instruction Fetch Stage
    // ─────────────────────────────────────────────────────────────────────────────

    reg [31:0] fetch_pc;  // Current PC

    // Pipeline registers between Fetch and Decode (F->D)
    reg [31:0] instr_FD;
    reg [31:0] pc_FD;
    reg nop_FD;  // Marks no-op for flush

    // Next PC selection logic
    // We consider immediate flush from decode stage or a flush from E->M stage
    // to update F_PC if needed.
    wire D_JumpOrBranchNow;
    wire [31:0] D_JumpOrBranchAddr;
    wire EM_JumpOrBranchNow;
    wire [31:0] EM_JumpOrBranchAddr;

    wire [31:0] next_fetch_pc = D_JumpOrBranchNow  ? D_JumpOrBranchAddr  :
                                EM_JumpOrBranchNow ? EM_JumpOrBranchAddr :
                                                     fetch_pc;

    always_ff @(posedge clk) begin
        if (!fetch_stall) begin
            // Read instruction from ROM
            instr_FD <= progRom[next_fetch_pc[15:2]];
            // Store PC in pipeline register
            pc_FD    <= next_fetch_pc;
            // Move PC to next
            fetch_pc <= next_fetch_pc + 4;
        end

        // Mark pipeline flush into FD stage
        nop_FD <= decode_flush | !resetn;

        if (!resetn) begin
            fetch_pc <= 32'b0;
        end
    end

    // ─────────────────────────────────────────────────────────────────────────────
    // D: Instruction Decode Stage
    // ─────────────────────────────────────────────────────────────────────────────

    // FD pipeline registers -> decode stage signals
    wire [31:0] decode_instr = instr_FD;
    wire [31:0] decode_pc = pc_FD;
    wire        decode_nop = nop_FD;

    // Predict branch based on sign bit (BTFNT: Backwards Taken, Forwards Not Taken)
    wire        decode_predictBranch = decode_instr[31];

    // If we see JAL or predicted branch, we redirect next fetch
    assign D_JumpOrBranchNow = !decode_nop && (isJAL(
        decode_instr
    ) || (isBranch(
        decode_instr
    ) && decode_predictBranch));

    assign D_JumpOrBranchAddr = decode_pc + (isJAL(
        decode_instr
    ) ? Jimm(
        decode_instr
    ) : Bimm(
        decode_instr
    ));

    // Register file
    reg  [31:0] regfile  [0:31];

    // Write-back control signals from W stage
    wire        wbEnable;
    wire [31:0] wbData;
    wire [ 4:0] wbRdId;

    // Write back to register file
    always_ff @(posedge clk) begin
        if (wbEnable) begin
            regfile[wbRdId] <= wbData;
        end
    end

    // Pipeline registers between Decode and Execute (D->E)
    reg  [31:0] pc_DE;
    reg  [31:0] instr_DE;
    reg         predBranch_DE;
    wire [31:0] rs1_DE = regfile[rs1Id(instr_DE)];
    wire [31:0] rs2_DE = regfile[rs2Id(instr_DE)];

    // Update decode->execute pipeline
    always_ff @(posedge clk) begin
        if (!decode_stall) begin
            pc_DE         <= decode_pc;
            instr_DE      <= (exec_flush | decode_nop) ? NOP : decode_instr;
            predBranch_DE <= decode_predictBranch;
        end

        if (exec_flush) begin
            instr_DE <= NOP;
        end
    end

    // ─────────────────────────────────────────────────────────────────────────────
    // E: Execute Stage
    // ─────────────────────────────────────────────────────────────────────────────

    // Pipeline inputs
    wire [31:0] exec_pc = pc_DE;
    wire [31:0] exec_instr = instr_DE;

    // Register forwarding from M and W stages
    // Pipeline registers E->M
    reg [31:0] instr_EM;
    reg [31:0] pc_EM;
    reg [31:0] Eresult_EM;
    reg [31:0] rs2_EM;
    reg [31:0] alu_addr_EM;
    reg jumpBranchNow_EM;
    reg [31:0] jumpBranchAddr_EM;

    // Forwarding detection
    wire M_fwd_rs1 = (rdId(
        instr_EM
    ) != 0) && writesRd(
        instr_EM
    ) && (rdId(
        instr_EM
    ) == rs1Id(
        exec_instr
    ));

    wire W_fwd_rs1 = (rdId(
        instr_MW
    ) != 0) && writesRd(
        instr_MW
    ) && (rdId(
        instr_MW
    ) == rs1Id(
        exec_instr
    ));

    wire M_fwd_rs2 = (rdId(
        instr_EM
    ) != 0) && writesRd(
        instr_EM
    ) && (rdId(
        instr_EM
    ) == rs2Id(
        exec_instr
    ));

    wire W_fwd_rs2 = (rdId(
        instr_MW
    ) != 0) && writesRd(
        instr_MW
    ) && (rdId(
        instr_MW
    ) == rs2Id(
        exec_instr
    ));

    // Forwarded operands
    wire [31:0] exec_rs1 = M_fwd_rs1 ? Eresult_EM : (W_fwd_rs1 ? wbData : rs1_DE);
    wire [31:0] exec_rs2 = M_fwd_rs2 ? Eresult_EM : (W_fwd_rs2 ? wbData : rs2_DE);

    // ALU input selection
    wire [31:0] alu_in1 = exec_rs1;
    wire [31:0] alu_in2 = (isALUreg(
        exec_instr
    ) || isBranch(
        exec_instr
    )) ? exec_rs2 : Iimm(
        exec_instr
    );

    wire [4:0] alu_shamt = isALUreg(exec_instr) ? exec_rs2[4:0] : shamt(exec_instr);
    wire alu_sub = exec_instr[30] & isALUreg(exec_instr);
    wire alu_arith = exec_instr[30];  // For shift: arithmetic if set

    // Adder for addition, also used to compute JALR address
    wire [31:0] alu_plus = alu_in1 + alu_in2;

    // Subtractor (33 bits for sub and comparisons)
    wire [32:0] alu_minus = {1'b1, ~alu_in2} + {1'b0, alu_in1} + 33'b1;
    wire alu_lt = (alu_in1[31] ^ alu_in2[31]) ? alu_in1[31] : alu_minus[32];
    wire alu_ltu = alu_minus[32];
    wire alu_eq = (alu_minus[31:0] == 32'b0);

    // "Flip" function for left/right shift (single shifter approach)
    function automatic [31:0] flip32;
        input [31:0] x;
        flip32 = {
            x[0],
            x[1],
            x[2],
            x[3],
            x[4],
            x[5],
            x[6],
            x[7],
            x[8],
            x[9],
            x[10],
            x[11],
            x[12],
            x[13],
            x[14],
            x[15],
            x[16],
            x[17],
            x[18],
            x[19],
            x[20],
            x[21],
            x[22],
            x[23],
            x[24],
            x[25],
            x[26],
            x[27],
            x[28],
            x[29],
            x[30],
            x[31]
        };
    endfunction

    wire [31:0] shifter_in = (funct3(exec_instr) == 3'b001) ? flip32(alu_in1) : alu_in1;
    // Arithmetic shift if alu_arith is set
    // verilator lint_off WIDTH
    wire [31:0] shifter_out = $signed({alu_arith & alu_in1[31], shifter_in}) >>> alu_shamt;
    // verilator lint_on WIDTH

    wire [31:0] left_shift_out = flip32(shifter_out);

    reg  [31:0] alu_res;
    always_comb begin
        unique case (funct3(
            exec_instr
        ))
            3'b000: alu_res = alu_sub ? alu_minus[31:0] : alu_plus;  // ADD / SUB
            3'b001: alu_res = left_shift_out;  // SLL
            3'b010: alu_res = {31'b0, alu_lt};  // SLT
            3'b011: alu_res = {31'b0, alu_ltu};  // SLTU
            3'b100: alu_res = alu_in1 ^ alu_in2;  // XOR
            3'b101: alu_res = shifter_out;  // SRL / SRA
            3'b110: alu_res = alu_in1 | alu_in2;  // OR
            3'b111: alu_res = alu_in1 & alu_in2;  // AND
        endcase
    end

    // Branch decision
    reg takeBranch;
    always_comb begin
        case (funct3(
            exec_instr
        ))
            3'b000:  takeBranch = alu_eq;  // BEQ
            3'b001:  takeBranch = !alu_eq;  // BNE
            3'b100:  takeBranch = alu_lt;  // BLT
            3'b101:  takeBranch = !alu_lt;  // BGE
            3'b110:  takeBranch = alu_ltu;  // BLTU
            3'b111:  takeBranch = !alu_ltu;  // BGEU
            default: takeBranch = 1'b0;
        endcase
    end

    // Actual jump if misprediction or JALR
    // For a branch, we correct if (takeBranch != predBranch)
    wire exec_jumpBranch = isJALR(
        exec_instr
    ) || (isBranch(
        exec_instr
    ) && (takeBranch ^ predBranch_DE));

    wire [31:0] exec_jumpBranchAddr = isBranch(
        exec_instr
    ) ? (exec_pc + (predBranch_DE ? 32'd4 : Bimm(
        exec_instr
    ))) : {alu_plus[31:1], 1'b0};  // JALR address

    // ALU final result: JAL and JALR store PC+4 in rd
    // LUI uses U-imm; AUIPC uses PC+Uimm
    wire [31:0] exec_result = (isJAL(
        exec_instr
    ) || isJALR(
        exec_instr
    )) ? (exec_pc + 32'd4) : isLUI(
        exec_instr
    ) ? Uimm(
        exec_instr
    ) : isAUIPC(
        exec_instr
    ) ? (exec_pc + Uimm(
        exec_instr
    )) : alu_res;

    // Update E->M pipeline registers
    always_ff @(posedge clk) begin
        pc_EM <= exec_pc;
        instr_EM <= exec_instr;
        rs2_EM <= exec_rs2;
        Eresult_EM <= exec_result;
        alu_addr_EM <= isStore(
            exec_instr
        ) ? (exec_rs1 + Simm(
            exec_instr
        )) : (exec_rs1 + Iimm(
            exec_instr
        ));
        jumpBranchNow_EM <= exec_jumpBranch;
        jumpBranchAddr_EM <= exec_jumpBranchAddr;
    end

    // EBREAK halts execution (when not in reset)
    assign halt = resetn & isEBREAK(exec_instr);

    // ─────────────────────────────────────────────────────────────────────────────
    // M: Memory Stage
    // ─────────────────────────────────────────────────────────────────────────────

    // Pipeline registers: E->M
    wire [31:0] mem_pc = pc_EM;
    wire [31:0] mem_instr = instr_EM;
    wire [31:0] mem_rs2 = rs2_EM;
    wire [31:0] mem_aluResult = Eresult_EM;
    wire [31:0] mem_addr = alu_addr_EM;

    assign EM_JumpOrBranchNow  = jumpBranchNow_EM;
    assign EM_JumpOrBranchAddr = jumpBranchAddr_EM;

    // Basic store parameter decoding
    wire [ 2:0] mem_funct3 = funct3(mem_instr);
    wire        mem_isByte = (mem_funct3[1:0] == 2'b00);
    wire        mem_isHalf = (mem_funct3[1:0] == 2'b01);

    // Store data alignment
    // For storing partial bytes/halfs
    wire [31:0] store_data;
    assign store_data[7:0] = mem_rs2[7:0];
    assign store_data[15:8] = mem_addr[0] ? mem_rs2[7:0] : mem_rs2[15:8];
    assign store_data[23:16] = mem_addr[1] ? mem_rs2[7:0] : mem_rs2[23:16];
    assign store_data[31:24] = mem_addr[0] ? mem_rs2[7:0]  :
                               mem_addr[1] ? mem_rs2[15:8] : mem_rs2[31:24];

    // Write mask for store
    //  Word: 1111
    //  Half: 1100 or 0011
    //  Byte: 1000 / 0100 / 0010 / 0001
    wire [3:0] store_wmask = mem_isByte
                                ? (mem_addr[1]
                                     ? (mem_addr[0] ? 4'b1000 : 4'b0100)
                                     : (mem_addr[0] ? 4'b0010 : 4'b0001))
                                : (mem_isHalf
                                     ? (mem_addr[1] ? 4'b1100 : 4'b0011)
                                     : 4'b1111);

    // Distinguish IO vs. RAM region
    wire mem_isIO = mem_addr[22];
    wire mem_isRAM = !mem_isIO;

    // Connect to IO region (when isStore + in IO region)
    assign IO_mem_addr  = mem_addr;
    assign IO_mem_wr    = isStore(mem_instr) && mem_isIO;
    assign IO_mem_wdata = mem_rs2;

    // Actual RAM write mask: only enabled if store + RAM region
    wire [ 3:0] mem_wmask = {4{isStore(mem_instr) & mem_isRAM}} & store_wmask;

    // Data memory (64 KB, 16384 words)
    wire [13:0] mem_word_addr = mem_addr[15:2];

    // Read / Write
    reg  [31:0] M_W_ramData;
    always_ff @(posedge clk) begin
        // Perform read before possible write
        M_W_ramData <= dataRam[mem_word_addr];
        if (mem_wmask[0]) dataRam[mem_word_addr][7:0] <= store_data[7:0];
        if (mem_wmask[1]) dataRam[mem_word_addr][15:8] <= store_data[15:8];
        if (mem_wmask[2]) dataRam[mem_word_addr][23:16] <= store_data[23:16];
        if (mem_wmask[3]) dataRam[mem_word_addr][31:24] <= store_data[31:24];
    end


    // Pipeline registers: M->W
    reg [31:0] pc_MW;
    reg [31:0] instr_MW;
    reg [31:0] result_MW;
    reg [31:0] addr_MW;
    reg [31:0] ramData_MW;
    reg [31:0] ioData_MW;
    reg [31:0] csrData_MW;

    always_ff @(posedge clk) begin
        pc_MW      <= mem_pc;
        instr_MW   <= mem_instr;
        result_MW  <= mem_aluResult;
        ioData_MW  <= IO_mem_rdata;
        addr_MW    <= mem_addr;
        ramData_MW <= M_W_ramData;

        // CSR READ-OUT
        unique case (csrId(
            mem_instr
        ))
            2'b00: csrData_MW <= cycle[31:0];
            2'b10: csrData_MW <= cycle[63:32];
            2'b01: csrData_MW <= instret[31:0];
            2'b11: csrData_MW <= instret[63:32];
        endcase

        if (!resetn) begin
            instret <= 64'b0;
        end else if (mem_instr != NOP) begin
            instret <= instret + 1;
        end
    end

    // ─────────────────────────────────────────────────────────────────────────────
    // W: Write-Back Stage
    // ─────────────────────────────────────────────────────────────────────────────

    wire [31:0] wb_pc = pc_MW;
    wire [31:0] wb_instr = instr_MW;

    // Load decode
    wire [2:0] wb_funct3 = funct3(wb_instr);
    wire wb_isByte = (wb_funct3[1:0] == 2'b00);
    wire wb_isHalf = (wb_funct3[1:0] == 2'b01);
    wire wb_sign_extend = !wb_funct3[2];
    wire wb_isIOAddr = addr_MW[22];

    // Partial-load data selection
    wire [15:0] wb_hword = addr_MW[1] ? ramData_MW[31:16] : ramData_MW[15:0];
    wire [7:0] wb_byte = addr_MW[0] ? wb_hword[15:8] : wb_hword[7:0];
    wire wb_signBit = wb_sign_extend && (wb_isByte ? wb_byte[7] : wb_hword[15]);

    wire [31:0] wb_ramLoaded = wb_isByte
                                  ? {{24{wb_signBit}}, wb_byte}
                                  : (wb_isHalf
                                     ? {{16{wb_signBit}}, wb_hword}
                                     : ramData_MW);

    // Choose final WB data from M/W pipeline
    // Either from load (RAM or IO), from ALU result, or from CSR
    assign wbData = isLoad(
        wb_instr
    ) ? (wb_isIOAddr ? ioData_MW : wb_ramLoaded) : isCSRRS(
        wb_instr
    ) ? csrData_MW : result_MW;

    // Write-back enable if instruction writes to rd and rd != x0
    assign wbEnable = writesRd(wb_instr) && (rdId(wb_instr) != 5'b0);
    assign wbRdId = rdId(wb_instr);

    // ─────────────────────────────────────────────────────────────────────────────
    // Hazard Detection (Load/CSR read) => Stall the pipeline
    // ─────────────────────────────────────────────────────────────────────────────

    wire rs1Hazard = readsRs1(instr_FD) && (rs1Id(instr_FD) == rdId(instr_DE));
    wire rs2Hazard = readsRs2(instr_FD) && (rs2Id(instr_FD) == rdId(instr_DE));

    // If an instruction in decode is a load or CSRRS, and F has a hazard => stall
    wire dataHazard = !nop_FD && (isLoad(
        instr_DE
    ) || isCSRRS(
        instr_DE
    )) && (rs1Hazard || rs2Hazard);

    // Stalls
    assign fetch_stall  = dataHazard | halt;
    assign decode_stall = dataHazard | halt;

    // Flush signals
    assign decode_flush = exec_jumpBranch;  // Adjust PC at decode stage
    assign exec_flush   = exec_jumpBranch | dataHazard;

endmodule  // pipeline
