# Pipeline-RV32Core

A fully functional **5-stage pipelined RISC-V RV32I processor** implementation in SystemVerilog, complete with hazard detection, data forwarding, and branch prediction. The core has been successfully tested with all basic RV32I instructions and includes comprehensive simulation infrastructure.

![RISC-V](https://img.shields.io/badge/RISC--V-RV32I-blue)
![SystemVerilog](https://img.shields.io/badge/SystemVerilog-IEEE%201800-green)
![Status](https://img.shields.io/badge/Status-Tested%20%26%20Working-success)

---

## 📋 Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Features](#features)
- [Directory Structure](#directory-structure)
- [Getting Started](#getting-started)
- [Simulation](#simulation)
- [Synthesis](#synthesis)
- [Testing & Verification](#testing--verification)
- [Implementation Details](#implementation-details)
- [References](#references)

---

## 🎯 Overview

This project implements a **5-stage pipelined RISC-V processor** adhering to the RV32I Base Integer Instruction Set (version 2.1). The processor core features:

- **Classic 5-stage pipeline**: Fetch → Decode → Execute → Memory → Writeback
- **Hazard handling**: Data forwarding and pipeline stalling for load/CSR hazards
- **Branch prediction**: Static backward-taken, forward-not-taken (BTFNT) prediction
- **Memory-mapped I/O**: Support for LEDs and UART communication
- **Tested implementation**: Verified with comprehensive instruction tests including loads, stores, branches, arithmetic, and logical operations

The design is optimized for both **simulation** (Verilator) and **FPGA synthesis** (Xilinx Vivado for Zybo Z7 board).

---

## 🏗️ Architecture

### Pipeline Stages

```
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│  Fetch   │───▶│  Decode  │───▶│ Execute  │───▶│  Memory  │───▶│Writeback │
│   (F)    │    │   (D)    │    │   (E)    │    │   (M)    │    │   (W)    │
└──────────┘    └──────────┘    └──────────┘    └──────────┘    └──────────┘
     │               │                │               │               │
     │               │                │               │               │
 PC Update      Register Read    ALU + Branch    Data Memory    Register Write
                                  Resolution
```

### Key Components

1. **Instruction Fetch (F)**
   - PC management and updating
   - Instruction memory interface (ROM)
   - Branch target handling

2. **Instruction Decode (D)**
   - 32-entry register file (x0-x31, x0 hardwired to 0)
   - Instruction decoding (opcode, funct3, funct7)
   - Immediate generation (I, S, B, U, J formats)
   - Branch prediction (BTFNT)

3. **Execute (E)**
   - ALU operations (ADD, SUB, AND, OR, XOR, SLT, shifts)
   - Branch condition evaluation
   - Data forwarding from M and W stages
   - Jump address calculation (JAL/JALR)

4. **Memory (M)**
   - Data RAM access (load/store)
   - Memory-mapped I/O (UART, LEDs)
   - Byte/halfword/word access support
   - CSR (cycle, instret counters)

5. **Writeback (W)**
   - Register file write-back
   - Load data alignment and sign extension

### Hazard Handling

- **Data Forwarding**: M→E and W→E forwarding paths to resolve RAW hazards
- **Pipeline Stalls**: Automatic stall on load-use and CSR-read hazards
- **Control Hazards**: Static branch prediction with misprediction recovery

---

## ✨ Features

### Supported Instructions

The core implements the complete **RV32I Base Integer Instruction Set**:

| Category | Instructions |
|----------|-------------|
| **Arithmetic** | ADD, ADDI, SUB |
| **Logical** | AND, ANDI, OR, ORI, XOR, XORI |
| **Shift** | SLL, SLLI, SRL, SRLI, SRA, SRAI |
| **Compare** | SLT, SLTI, SLTU, SLTIU |
| **Branch** | BEQ, BNE, BLT, BGE, BLTU, BGEU |
| **Jump** | JAL, JALR |
| **Load** | LB, LH, LW, LBU, LHU |
| **Store** | SB, SH, SW |
| **Upper Immediate** | LUI, AUIPC |
| **System** | ECALL (EBREAK), CSRRS (cycle/instret) |

### Hardware Features

- **32 General-Purpose Registers**: x0 (zero), x1-x31
- **64KB Program ROM**: 16,384 words
- **64KB Data RAM**: 16,384 words with byte-enable write
- **Memory-Mapped I/O**: Configurable UART and LED interfaces
- **Performance Counters**: 64-bit cycle and instruction retired counters
- **UART Output**: 1 Mbaud serial communication for debugging

---

## 📁 Directory Structure

```
Pipeline-RV32Core/
├── src/                    # Main source files
│   ├── core.sv            # 5-stage pipeline CPU core
│   ├── soc.sv             # System-on-Chip top-level
│   ├── clkworks.sv        # Clock management
│   ├── emitter_uart.sv    # UART transmitter
│   └── zybo.xdc           # Zybo Z7 constraints
├── rtl/                    # RTL development files
│   ├── riscv_core.sv      # Alternative core implementation
│   ├── fetch_stage.sv     # Standalone fetch stage module
│   └── monitor.py         # Python monitoring script
├── tb/                     # Testbenches
│   └── soc_tb.sv          # SoC testbench
├── firmware/               # Firmware and test programs
│   ├── asm/               # Assembly source files
│   │   └── sample.s       # Sample assembly program
│   ├── hex/               # Compiled hex files
│   │   ├── PROGROM.hex    # Program ROM image
│   │   ├── DATARAM.hex    # Data RAM image
│   │   ├── SIMPLE_P.hex   # Simple program ROM
│   │   └── SIMPLE_D.hex   # Simple data RAM
│   └── precompiled/       # Benchmark programs
│       ├── COREMARK/      # CoreMark benchmark
│       ├── DHRYSTONES/    # Dhrystone benchmark
│       └── RAYSTONES/     # Custom benchmark
├── boards/                 # FPGA synthesis files
│   ├── project.tcl        # Vivado project script
│   ├── build.tcl          # Build automation
│   ├── synth.ys           # Yosys synthesis
│   └── zybo.tcl           # Zybo-specific config
├── waves/                  # Waveform files
│   ├── soc_tb.vcd         # Simulation waveform (VCD format)
│   └── surfer_state.ron   # Surfer waveform viewer state
├── makefile               # Main build/simulation makefile
├── veridian.yml           # Veridian LSP configuration
└── README.md              # This file
```

---

## 🚀 Getting Started

### Prerequisites

#### For Simulation
- **Verilator** (v4.0+): Open-source SystemVerilog simulator
- **GNU Make**: Build automation
- **Waveform Viewer**: Surfer (recommended) or GTKWave

#### For FPGA Synthesis
- **Xilinx Vivado** (2020.1+): For Zybo Z7 board
- **Yosys** (optional): Open-source synthesis

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/PrabathBK/Pipeline-RV32Core.git
   cd Pipeline-RV32Core
   ```

2. **Install Verilator** (if not already installed)
   ```bash
   # macOS (Homebrew)
   brew install verilator
   
   # Ubuntu/Debian
   sudo apt-get install verilator
   
   # From source
   git clone https://github.com/verilator/verilator
   cd verilator
   autoconf && ./configure && make && sudo make install
   ```

3. **Install waveform viewer**
   ```bash
   # Surfer (recommended - modern web-based viewer)
   cargo install surfer
   
   # GTKWave (alternative)
   brew install gtkwave  # macOS
   sudo apt-get install gtkwave  # Linux
   ```

---

## 🔬 Simulation

### Quick Start

Run simulation with a single command:

```bash
make vl
```

This will:
1. Verilate the design
2. Build the simulation executable
3. Run the simulation
4. Generate waveform file (`waves/soc_tb.vcd`)

### View Waveforms

```bash
make wa
```

Opens the waveform viewer (Surfer by default) with the generated VCD file.

### Detailed Simulation Commands

```bash
# Clean previous build
make clean

# Run complete simulation flow
make all

# Run simulation only
make vl

# View waveforms only
make wa

# Change waveform viewer
WAVEFORM_VIEWER=gtkwave make wa
```

### Simulation Output

The simulation outputs:
- **Console**: UART output from the running program
- **VCD File**: Detailed signal trace for analysis
- **Logs**: Simulation status and debugging information

Example console output:
```
Starting simulation...
UART: Hello from RISC-V!
UART: Testing arithmetic...
UART: Testing branches...
UART: All tests passed!
```

### Customizing Test Programs

1. **Edit assembly code**: Modify `firmware/asm/sample.s`
2. **Compile to hex**: Use RISC-V toolchain (riscv32-unknown-elf-as/ld/objcopy)
3. **Update hex files**: Place in `firmware/hex/`
4. **Update core.sv**: Change the `$readmemh` path if needed
5. **Re-run simulation**: `make clean && make vl`

---

## 🔧 Synthesis

### FPGA Target: Zybo Z7

The design is configured for the **Digilent Zybo Z7-10** board (Xilinx Zynq-7000 SoC).

### Vivado Synthesis

1. **Open Vivado**
   ```bash
   vivado -source boards/project.tcl
   ```

2. **Or use command-line flow**
   ```bash
   vivado -mode batch -source boards/build.tcl
   ```

3. **Configuration**
   - Part: `xc7z010clg400-1`
   - Board: Digilent Zybo Z7-10
   - Top module: `soc`
   - Constraints: `src/zybo.xdc`

### Yosys Open-Source Synthesis

```bash
yosys boards/synth.ys
```

Generates netlist and synthesis reports.

### FPGA Resource Utilization (Estimated)

| Resource | Used | Available | Utilization |
|----------|------|-----------|-------------|
| LUTs | ~2,500 | 17,600 | ~14% |
| Registers | ~1,800 | 35,200 | ~5% |
| BRAM | 32 | 60 | ~53% |
| DSPs | 0 | 80 | 0% |

---

## ✅ Testing & Verification

### Verification Status

✅ **All RV32I instructions tested and verified**

The processor has been thoroughly tested with:

1. **Arithmetic Instructions**: ADD, ADDI, SUB, LUI, AUIPC
2. **Logical Instructions**: AND, OR, XOR, ANDI, ORI, XORI
3. **Shift Instructions**: SLL, SRL, SRA (register and immediate)
4. **Comparison**: SLT, SLTU, SLTI, SLTIU
5. **Branches**: BEQ, BNE, BLT, BGE, BLTU, BGEU with prediction
6. **Jumps**: JAL, JALR with proper link register handling
7. **Memory Operations**: LB, LH, LW, LBU, LHU, SB, SH, SW
8. **Hazards**: Load-use dependencies, register forwarding
9. **CSR**: Cycle counter, instruction retired counter

### Viewing Test Results

The simulation generates a **VCD waveform file** (`waves/soc_tb.vcd`) that contains complete signal traces. Key signals to observe:

#### Pipeline Stages
- `soc_inst.cpu.fetch_pc` - Current PC in fetch
- `soc_inst.cpu.instr_FD` - Instruction in decode
- `soc_inst.cpu.instr_DE` - Instruction in execute
- `soc_inst.cpu.instr_EM` - Instruction in memory
- `soc_inst.cpu.instr_MW` - Instruction in writeback

#### Register File
- `soc_inst.cpu.regfile[0]` through `soc_inst.cpu.regfile[31]` - All 32 registers
- Watch x1 (return address), x2 (stack pointer), x10-x17 (arguments/return values)

#### Hazard Detection
- `soc_inst.cpu.dataHazard` - Data hazard detected
- `soc_inst.cpu.fetch_stall` - Fetch stage stalled
- `soc_inst.cpu.exec_flush` - Execute stage flushed
- `soc_inst.cpu.M_fwd_rs1/rs2` - Memory-to-execute forwarding
- `soc_inst.cpu.W_fwd_rs1/rs2` - Writeback-to-execute forwarding

#### Memory Operations
- `soc_inst.cpu.dataRam` - Data memory contents
- `soc_inst.cpu.progRom` - Program ROM contents
- `soc_inst.IO_mem_addr` - I/O memory address
- `soc_inst.IO_mem_wdata` - I/O write data

#### Performance Counters
- `soc_inst.cpu.cycle` - Total cycle count
- `soc_inst.cpu.instret` - Instructions retired

### Example Test Case Analysis

A typical load-use hazard scenario visible in the VCD:

```
Cycle 100: Fetch  LW x1, 0(x2)
Cycle 101: Decode LW x1, 0(x2)  |  Fetch  ADD x3, x1, x4
Cycle 102: Execute LW           |  Decode ADD (STALLED - dataHazard=1)
Cycle 103: Memory LW            |  Decode ADD (STALLED)
Cycle 104: WB LW               |  Execute ADD (M_fwd_rs1=1, uses forwarded data)
```

### Benchmark Programs

Pre-compiled benchmark programs are available in `firmware/precompiled/`:
- **CoreMark**: Industry-standard CPU benchmark
- **Dhrystone**: Classic integer performance benchmark
- **Raystones**: Custom ray-tracing benchmark

---

## 🔍 Implementation Details

### Register File

- **32 registers**: x0-x31 (x0 is hardwired to zero)
- **Read ports**: 2 (for rs1, rs2)
- **Write port**: 1 (for rd)
- **Write-back**: Synchronous on clock edge

**Standard calling convention**:
- x1: Return address (ra)
- x2: Stack pointer (sp)
- x5: Alternate link register
- x10-x17: Function arguments/return values

### Memory Map

| Address Range | Region | Description |
|---------------|--------|-------------|
| 0x00000000 - 0x0003FFFF | Program ROM | 64KB instruction memory |
| 0x00000000 - 0x0003FFFF | Data RAM | 64KB data memory |
| 0x00400000+ | I/O Region | Memory-mapped peripherals |

**I/O Addresses** (word-aligned):
- 0x400000: LED register (write-only)
- 0x400004: UART data (write-only)
- 0x400008: UART status (read-only, bit 9 = busy)

### Branch Prediction

**Strategy**: Static BTFNT (Backward-Taken, Forward-Not-Taken)
- Predicts **taken** if branch offset is negative (backward branch)
- Predicts **not taken** if branch offset is positive (forward branch)

**Implementation**:
- Prediction decision in Decode stage based on instruction bit 31 (sign of B-immediate)
- Misprediction detected in Execute stage
- Single-cycle penalty on misprediction (flush and redirect)

### Data Forwarding

**Forwarding Paths**:
1. **M→E (Memory to Execute)**: Forward ALU result from Memory stage
2. **W→E (Writeback to Execute)**: Forward writeback data

**Priority**: M→E forwarding takes precedence over W→E

**Logic**:
```systemverilog
M_fwd_rs1 = (rdId(instr_EM) != 0) && writesRd(instr_EM) && (rdId(instr_EM) == rs1Id(exec_instr))
W_fwd_rs1 = (rdId(instr_MW) != 0) && writesRd(instr_MW) && (rdId(instr_MW) == rs1Id(exec_instr))

exec_rs1 = M_fwd_rs1 ? Eresult_EM : (W_fwd_rs1 ? wbData : rs1_DE)
```

### Pipeline Control

**Stall Conditions**:
- Load-use hazard: Load in Decode, dependent instruction in Fetch
- CSR-read hazard: CSRRS in Decode, dependent instruction in Fetch
- EBREAK: Halt execution

**Flush Conditions**:
- Branch misprediction
- Jump instruction (JAL/JALR)

### ALU Design

**Single-Shifter Approach**:
- Uses bit-reversal for left shifts (convert to right shift)
- Supports arithmetic and logical right shifts
- 5-bit shift amount from register or immediate

**Operations**:
- Arithmetic: ADD, SUB (33-bit for comparison)
- Logic: AND, OR, XOR
- Shift: SLL, SRL, SRA
- Compare: SLT, SLTU (generates 0/1 result)

---

## 📄 License

This project is released under the MIT License. See LICENSE file for details.

---


**Status**: ✅ Fully functional and tested with all RV32I instructions  
