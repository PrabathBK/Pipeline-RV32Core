`default_nettype none

module soc (
    input  wire        CLK,    // system clock
    input  wire        RESET,  // reset button
    output logic [3:0] LEDS,   // system LEDs
    // input  wire        RXD,    // UART receive
    output logic       TXD     // UART transmit
);

    //----------------------------------------------------------------------
    // Internal clock and reset signals (would come from a clockworks module)
    //----------------------------------------------------------------------
    wire        clk;
    wire        resetn;

    //----------------------------------------------------------------------
    // Wires for memory-mapped IO from the CPU pipeline
    //----------------------------------------------------------------------
    wire [31:0] IO_mem_addr;  // IO address
    wire [31:0] IO_mem_rdata;  // Data read back to CPU
    wire [31:0] IO_mem_wdata;  // Data written by CPU
    wire        IO_mem_wr;  // Write-enable flag

    //----------------------------------------------------------------------
    // Instantiate the pipeline CPU
    //
    // Replace "Processor" with your pipeline module. Make sure the port
    // names match the interface signals in your pipeline design.
    //----------------------------------------------------------------------

    core cpu (
        .clk         (clk),
        .resetn      (resetn),
        .IO_mem_addr (IO_mem_addr),
        .IO_mem_rdata(IO_mem_rdata),
        .IO_mem_wdata(IO_mem_wdata),
        .IO_mem_wr   (IO_mem_wr)
    );

    //----------------------------------------------------------------------
    // Memory-mapped IO address decode logic
    // We derive a "word address" from IO_mem_addr[15:2].
    //----------------------------------------------------------------------
    wire [13:0] IO_wordaddr = IO_mem_addr[15:2];

    // For this example, we define 3 memory-mapped locations:
    //   0) LEDS register (write-only)
    //   1) UART data register (write-only)
    //   2) UART control/status register (read-only)
    localparam IO_ADDR_LEDS = 0;
    localparam IO_ADDR_UART_DATA = 1;
    localparam IO_ADDR_UART_CTRL = 2;

    //----------------------------------------------------------------------
    // LED Write
    // If the CPU writes to IO_ADDR_LEDS, capture the lower 5 bits to drive the LEDs.
    //----------------------------------------------------------------------
    // always @(posedge clk) begin
    //     if (IO_mem_wr && IO_wordaddr[IO_ADDR_LEDS]) begin
    //         LEDS <= IO_mem_wdata[3:0];
    //     end
    // end

    assign LEDS = 4'b1111;

    //----------------------------------------------------------------------
    // UART Transmitter
    // Here we use a simple UART module that sends bytes out at 1 Mbaud.
    //----------------------------------------------------------------------
    wire uart_write_en = IO_mem_wr && IO_wordaddr[IO_ADDR_UART_DATA];
    wire uart_ready;

    emitter_uart #(
        .clk_freq_hz(10 * 1000000),  // Use an appropriate clock frequency
        .baud_rate  (1000000)        // 1 Mbaud
    ) UART (
        .i_clk    (clk),
        .i_rst    (!resetn),
        .i_data   (IO_mem_wdata[7:0]),  // Only the least significant byte is relevant
        .i_valid  (uart_write_en),      // Write strobe
        .o_ready  (uart_ready),         // '1' if ready to accept another byte
        .o_uart_tx(TXD)                 // UART TX line
    );

    //----------------------------------------------------------------------
    // Read Data (from UART status or default)
    // If we read from IO_ADDR_UART_CTRL, return busy/ready bits in the upper bits.
    //----------------------------------------------------------------------
    assign IO_mem_rdata = IO_wordaddr[IO_ADDR_UART_CTRL] ? {22'b0, !uart_ready, 9'b0} : 32'b0;

    //----------------------------------------------------------------------
    // Simulation Printout for Debug
    // If built with BENCH define, print out each byte that gets written to UART.
    //----------------------------------------------------------------------

`ifdef VERILATOR
    always @(posedge clk) begin
        if (uart_write_en) begin
            // $display("UART: %c", IO_mem_wdata[7:0]);
            $write("%c", IO_mem_wdata[7:0]);
            $fflush(32'h8000_0001);
        end
    end
`endif

    clkworks #(
        .SLOW(5)
    ) cw_inst (
        .clk(CLK),
        .reset(RESET),
        .clk_o(clk),
        .resetn(resetn)
    );
endmodule
