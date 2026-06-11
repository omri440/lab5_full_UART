`timescale 1ns / 1ps

module tx_uart_simple_tb;

    reg [15:0] sw;
    reg        clk;
    reg        rst_n;
    reg        center_btn;
    reg        rx_in;

    wire        uart_tx;
    wire        dp;
    wire [15:0] led;
    wire [7:0]  an;
    wire [6:0]  segment;

    trx_uart_controller dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .sw         (sw),
        .center_btn (center_btn),
        .rx_in      (rx_in),
        .uart_tx    (uart_tx),
        .led        (led),
        .an         (an),
        .segment    (segment),
        .dp         (dp)
    );

    defparam dut.button_latch_inst.DEBOUNCE_CYCLES = 2;
    defparam dut.button_latch_inst.LATCH_CYCLES    = 5;

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    initial begin
        sw         = 16'd0;        // sw[15]=0 -> TX mode (Lab 4 behaviour)
        center_btn = 1'b0;
        rx_in      = 1'b1;         // RX line idle (not exercised here)
        rst_n      = 1'b0;

        #100;
        rst_n = 1'b1;
        #100;

        sw = 16'b0_01_000_00_01000001;   // sw[15]=0, size=01 (32x32), speed=00, data='A'

        #50;

        $display("--- Triggering BTNC for 100 ns ---");
        center_btn = 1'b1;
        #100;
        center_btn = 1'b0;

        $display("--- Monitoring first 60 ms of 32x32 transmission ---");
        #60000000;

        $display("Simulation Finished.");
        $finish;
    end

    initial begin
        $monitor("Time: %t | State: %h | Byte Ctr: %0d | Row Ctr: %0d | LED0: %b | LED15: %b",
                 $time,
                 dut.tx_fsm_inst.state,
                 dut.transmitted_count,
                 dut.row_count,
                 led[0],
                 led[15]);
    end

endmodule
