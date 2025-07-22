module soc_tb;
    reg CLK = 0;
    reg RESET;
    wire [3:0] LEDS;
    // wire RXD;
    wire TXD;

    soc soc_inst (
        .CLK  (CLK),
        .RESET(RESET),
        .LEDS (LEDS),
        // .RXD  (RXD),
        .TXD  (TXD)
    );

    initial begin
        forever #10 CLK = ~CLK;
    end
    initial begin
        $display("Starting simulation...");
        RESET = 1;
        #100;
        RESET = 0;
        #10000000;
        $finish;
    end

`ifdef VERILATOR
    initial begin
        $dumpfile("waves/soc_tb.vcd");
        $dumpvars(0, soc_tb);
    end
`endif


    // // Monitor CPU IO activity
    // always @(posedge soc_inst.clk) begin
    //     if (soc_inst.IO_mem_wr) begin
    //         $display("Time: %0t - CPU Write:", $time);
    //         $display("  Address: 0x%h", soc_inst.IO_mem_addr);
    //         $display("  Data: 0x%h", soc_inst.IO_mem_wdata);

    //         // Decode the write based on address
    //         case (soc_inst.IO_wordaddr)
    //             soc_inst.IO_ADDR_LEDS:
    //             $display("  Writing to LEDs: %b", soc_inst.IO_mem_wdata[4:0]);
    //             soc_inst.IO_ADDR_UART_DATA:
    //             $display(
    //                 "  Writing to UART: %c (0x%h)",
    //                 soc_inst.IO_mem_wdata[7:0],
    //                 soc_inst.IO_mem_wdata[7:0]
    //             );
    //             default: $display("  Writing to unknown address");
    //         endcase
    //     end

    //     // Monitor reads
    //     if (|soc_inst.IO_mem_rdata) begin
    //         $display("Time: %0t - CPU Read:", $time);
    //         $display("  Address: 0x%h", soc_inst.IO_mem_addr);
    //         $display("  Data: 0x%h", soc_inst.IO_mem_rdata);
    //     end
    // end


endmodule
