/**
 * trx_uart_controller_tb.sv - Lab 5 testbench
 *
 * Five scenarios:
 *   1. TX 1x1 mode             (Lab 4 regression, sw[15]=0)
 *   2. TX 32x32 mode           (Lab 4 regression, sw[15]=0)
 *   3. TX gated in RX mode     (sw[15]=1, BTNC press must NOT start a transmission)
 *   4. RX happy path           (sw[15]=1, send "{R032,C016,V255}")
 *   5. RX malformed            (sw[15]=1, send "{R032,X016,V255}" - parser rejects)
 *
 * All scenarios use the scaled-down baud / debounce / latch parameters so the
 * full run fits in a few milliseconds of simulation time.
 */

`timescale 1ns / 1ps

module trx_uart_controller_tb;

    // ----------------------------------------------------------------
    // Sim-scaled parameter values (overridden via defparam below)
    // ----------------------------------------------------------------
    localparam int CLK_PERIOD_NS       = 10;
    localparam int SIM_BAUD_DIVISOR    = 8;
    localparam int SIM_DEBOUNCE_CYCLES = 4;
    localparam int SIM_LATCH_CYCLES    = 16;
    localparam int SIM_DELAY_1_CYCLES  = 6;
    localparam int SIM_DELAY_2_CYCLES  = 10;
    localparam int SIM_DELAY_3_CYCLES  = 14;
    localparam int SIM_DIGIT_PERIOD    = 8;

    // ----------------------------------------------------------------
    // DUT signals
    // ----------------------------------------------------------------
    logic        clk;
    logic        rst_n;
    logic [15:0] sw;
    logic        center_btn;
    logic        rx_in_drv;          // TB drives this -> dut.rx_in
    logic        uart_tx;
    logic [15:0] led;
    logic [7:0]  an;
    logic [6:0]  segment;
    logic        dp;

    byte         rx_byte;
    byte         delimiter_byte;

    trx_uart_controller dut (
        .clk(clk),
        .rst_n(rst_n),
        .sw(sw),
        .center_btn(center_btn),
        .rx_in(rx_in_drv),
        .uart_tx(uart_tx),
        .led(led),
        .an(an),
        .segment(segment),
        .dp(dp)
    );

    // Scale all the slow real-HW thresholds down for fast simulation.
    defparam dut.uart_phy_inst.DIVISOR             = SIM_BAUD_DIVISOR;
    defparam dut.rx_phy_inst.BAUD_DIVISOR          = SIM_BAUD_DIVISOR;
    defparam dut.rx_phy_inst.HALF_BAUD             = SIM_BAUD_DIVISOR / 2;
    defparam dut.button_latch_inst.DEBOUNCE_CYCLES = SIM_DEBOUNCE_CYCLES;
    defparam dut.button_latch_inst.LATCH_CYCLES    = SIM_LATCH_CYCLES;
    defparam dut.tx_fsm_inst.DELAY_1_CYCLES        = SIM_DELAY_1_CYCLES;
    defparam dut.tx_fsm_inst.DELAY_2_CYCLES        = SIM_DELAY_2_CYCLES;
    defparam dut.tx_fsm_inst.DELAY_3_CYCLES        = SIM_DELAY_3_CYCLES;
    defparam dut.seven_seg_inst.DIGIT_PERIOD       = SIM_DIGIT_PERIOD;

    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    // ----------------------------------------------------------------
    // Common tasks
    // ----------------------------------------------------------------
    task automatic reset_dut();
        begin
            clk        = 1'b0;
            rst_n      = 1'b0;
            sw         = '0;
            center_btn = 1'b0;
            rx_in_drv  = 1'b1;            // UART line idles high
            repeat (4) @(posedge clk);
            rst_n = 1'b1;
            repeat (2) @(posedge clk);
        end
    endtask

    // Drive BTNC long enough for the button_latch to fire. Hold until the
    // pulse is observed, then release. Adds a small cooldown so the next
    // call sees a clean rising edge on center_btn.
    task automatic latch_configuration(input logic [15:0] new_sw);
        begin
            sw = new_sw;
            @(posedge clk);
            center_btn = 1'b1;
            wait (dut.latch_triggered === 1'b1);
            @(posedge clk);
            center_btn = 1'b0;
            repeat (SIM_DEBOUNCE_CYCLES + 4) @(posedge clk);
        end
    endtask

    // Sample uart_tx at every TX baud_pulse, picking up one received UART byte.
    task automatic receive_uart_byte(output byte value);
        begin
            value = 8'h00;
            @(negedge uart_tx);
            for (int bit_index = 0; bit_index < 8; bit_index++) begin
                @(posedge dut.baud_pulse);
                value[bit_index] = uart_tx;
            end
            @(posedge dut.baud_pulse);
            if (uart_tx !== 1'b1)
                $error("UART stop bit was not high");
        end
    endtask

    task automatic expect_equal_byte(
        input byte    actual_value,
        input byte    expected_value,
        input string  label
    );
        begin
            if (actual_value !== expected_value)
                $error("%s mismatch. expected=0x%02h actual=0x%02h",
                       label, expected_value, actual_value);
            else
                $display("[TB] %s ok: 0x%02h", label, actual_value);
        end
    endtask

    task automatic expect_equal_byte_quiet(
        input byte    actual_value,
        input byte    expected_value,
        input string  label,
        input int     index
    );
        begin
            if (actual_value !== expected_value)
                $error("%s[%0d] mismatch. expected=0x%02h actual=0x%02h",
                       label, index, expected_value, actual_value);
        end
    endtask

    // Bit-bang an ASCII string onto rx_in_drv.
    //   For each character: 1 start bit (0), 8 data bits LSB first, 1 stop bit (1).
    //   Each bit lasts SIM_BAUD_DIVISOR system-clock cycles.
    task automatic drive_rx_message(input string s);
        byte ch;
        begin
            for (int i = 0; i < s.len(); i++) begin
                ch = s[i];
                // start bit
                rx_in_drv = 1'b0;
                repeat (SIM_BAUD_DIVISOR) @(posedge clk);
                // 8 data bits LSB first
                for (int b = 0; b < 8; b++) begin
                    rx_in_drv = ch[b];
                    repeat (SIM_BAUD_DIVISOR) @(posedge clk);
                end
                // stop bit
                rx_in_drv = 1'b1;
                repeat (SIM_BAUD_DIVISOR) @(posedge clk);
            end
        end
    endtask

    // ----------------------------------------------------------------
    // Main test sequence
    // ----------------------------------------------------------------
    initial begin
        $dumpfile("trx_uart_controller_tb.vcd");
        $dumpvars(0, trx_uart_controller_tb);

        reset_dut();

        // ================================================================
        // Scenario 1: TX 1x1 mode (Lab 4 regression)
        //   sw[15]=0, size=00, speed=00, data=0xA5
        // ================================================================
        $display("\n[TB] === Scenario 1: TX 1x1 mode ===");
        latch_configuration(16'b0_00_000_00_10100101);

        if (dut.num_bytes_to_send !== 17'd1)
            $error("[1x1] expected num_bytes_to_send=1, got %0d", dut.num_bytes_to_send);

        receive_uart_byte(rx_byte);
        expect_equal_byte(rx_byte, 8'hA5, "1x1 data byte");

        receive_uart_byte(delimiter_byte);
        expect_equal_byte(delimiter_byte, 8'h20, "1x1 delimiter byte");

        if (led[0] !== 1'b1)
            $error("[1x1] LED[0] did not toggle after first data byte");
        if (led[15] !== 1'b0)
            $error("[1x1] LED[15] should be 0 in TX mode, got %b", led[15]);

        wait (dut.transmission_done === 1'b1);
        @(posedge clk);

        if (dut.t0_data !== 8'hA5) $error("[1x1] T0 expected 0xA5 got 0x%02h", dut.t0_data);
        if (dut.t1_data !== 8'h00) $error("[1x1] T1 expected 0x00 got 0x%02h", dut.t1_data);
        if (dut.t2_data !== 8'h01) $error("[1x1] T2 expected 0x01 got 0x%02h", dut.t2_data);
        if (dut.t3_data !== 8'h01) $error("[1x1] T3 expected 0x01 got 0x%02h", dut.t3_data);

        // ================================================================
        // Scenario 2: TX 32x32 mode (Lab 4 regression)
        //   sw[15]=0, size=01, speed=00, data=0x5A
        // ================================================================
        $display("\n[TB] === Scenario 2: TX 32x32 mode ===");
        latch_configuration(16'b0_01_000_00_01011010);

        if (dut.num_bytes_to_send !== 17'd1024)
            $error("[32x32] expected num_bytes_to_send=1024, got %0d", dut.num_bytes_to_send);

        for (int row = 0; row < 32; row++) begin
            for (int col = 0; col < 32; col++) begin
                receive_uart_byte(rx_byte);
                expect_equal_byte_quiet(rx_byte, 8'h5A, "32x32 data",  row * 32 + col);
                receive_uart_byte(rx_byte);
                expect_equal_byte_quiet(rx_byte, 8'h20, "32x32 delim", row * 32 + col);
            end
            receive_uart_byte(rx_byte);
            expect_equal_byte_quiet(rx_byte, 8'h0D, "32x32 cr", row);
            receive_uart_byte(rx_byte);
            expect_equal_byte_quiet(rx_byte, 8'h0A, "32x32 lf", row);
        end

        wait (dut.transmission_done === 1'b1);
        @(posedge clk);

        if (dut.row_count !== 8'd32)
            $error("[32x32] expected row_count=32 got %0d", dut.row_count);
        if (dut.t3_data !== 8'h20)
            $error("[32x32] T3 expected 0x20 got 0x%02h", dut.t3_data);

        // ================================================================
        // Scenario 3: TX is gated off in RX mode
        //   Set sw[15]=1, hold BTNC longer than the latch threshold,
        //   verify latch_triggered never fires and uart_tx stays idle.
        // ================================================================
        $display("\n[TB] === Scenario 3: TX gated in RX mode ===");
        sw = 16'b1_00_000_00_00000000;
        @(posedge clk);

        if (led[15] !== 1'b1)
            $error("[RX gate] LED[15] expected 1 (RX mode) got %b", led[15]);

        center_btn = 1'b1;
        repeat (SIM_DEBOUNCE_CYCLES + SIM_LATCH_CYCLES + 50) @(posedge clk);

        if (dut.latch_triggered === 1'b1)
            $error("[RX gate] latch_triggered fired while sw[15]=1");
        if (uart_tx !== 1'b1)
            $error("[RX gate] uart_tx left idle while sw[15]=1");

        center_btn = 1'b0;
        repeat (SIM_DEBOUNCE_CYCLES + 4) @(posedge clk);

        // ================================================================
        // Scenario 4: RX happy path - "{R032,C016,V255}"
        //   sw[15]=1 already from scenario 3.
        // ================================================================
        $display("\n[TB] === Scenario 4: RX happy path ===");
        drive_rx_message("{R032,C016,V255}");

        // Give the MAC + parser a few cycles to settle after the last byte's stop.
        repeat (200) @(posedge clk);

        if (dut.rx_row_val   !== 8'd32)
            $error("[RX happy] row_val expected 32 got %0d",   dut.rx_row_val);
        if (dut.rx_col_val   !== 8'd16)
            $error("[RX happy] col_val expected 16 got %0d",   dut.rx_col_val);
        if (dut.rx_pixel_val !== 8'd255)
            $error("[RX happy] pixel_val expected 255 got %0d", dut.rx_pixel_val);

        // Display mux must now show the RX fields
        if (dut.t0_data !== 8'd255)
            $error("[RX happy] t0_data expected 0xFF got 0x%02h", dut.t0_data);
        if (dut.t2_data !== 8'd16)
            $error("[RX happy] t2_data expected 0x10 got 0x%02h", dut.t2_data);
        if (dut.t3_data !== 8'd32)
            $error("[RX happy] t3_data expected 0x20 got 0x%02h", dut.t3_data);
        if (dut.dash_enable !== 8'b0000_1100)
            $error("[RX happy] dash_enable expected 0x0C got 0x%02h", dut.dash_enable);

        $display("[TB] RX happy path passed");

        // ================================================================
        // Scenario 5: RX malformed - "{R032,X016,V255}"
        //   'X' in the comma slot must cause the parser to reject the
        //   message; row_val/col_val/pixel_val must not change.
        // ================================================================
        $display("\n[TB] === Scenario 5: RX malformed ===");
        drive_rx_message("{R032,X016,V255}");

        repeat (200) @(posedge clk);

        // Outputs must still hold the scenario-4 values.
        if (dut.rx_row_val   !== 8'd32)
            $error("[RX bad] row_val changed (now %0d)",   dut.rx_row_val);
        if (dut.rx_col_val   !== 8'd16)
            $error("[RX bad] col_val changed (now %0d)",   dut.rx_col_val);
        if (dut.rx_pixel_val !== 8'd255)
            $error("[RX bad] pixel_val changed (now %0d)", dut.rx_pixel_val);

        $display("[TB] RX malformed path passed");

        $display("\n[TB] === All 5 scenarios complete ===");
        $finish;
    end

endmodule
