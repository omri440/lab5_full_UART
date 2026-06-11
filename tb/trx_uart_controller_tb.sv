

`timescale 1ns / 1ps

module trx_uart_controller_tb;

    // -------------------------------------------------------------------------
    // Timing constants
    // -------------------------------------------------------------------------
    localparam int CLK_PERIOD_NS     = 10;
    localparam int UART_BIT_CYCLES   = 1736;            // real 57600 baud
    localparam int HALF_BIT_CYCLES   = UART_BIT_CYCLES / 2;
    localparam int RX_RESULT_TIMEOUT = 50_000;
    localparam int TX_START_TIMEOUT  = 100_000;         // covers button hold + first TX byte
    localparam int TX_IDLE_TIMEOUT   = 200_000;
    localparam int BTN_HOLD_CYCLES   = 50;              // > scaled latch threshold

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    logic        clk;
    logic        rst_n;
    logic [15:0] sw;
    logic        center_btn;
    logic        rx_in_drv;
    logic        uart_tx;
    logic [15:0] led;
    logic [7:0]  an;
    logic [6:0]  segment;
    logic        dp;

    trx_uart_controller dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .sw         (sw),
        .center_btn (center_btn),
        .rx_in      (rx_in_drv),
        .uart_tx    (uart_tx),
        .led        (led),
        .an         (an),
        .segment    (segment),
        .dp         (dp)
    );

    // Scale ONLY the button latch (otherwise BTNC presses would burn 1 s of sim).
    // All baud-related parameters stay at their real values so the RX/TX waveforms
    // exactly match the board.
    defparam dut.button_latch_inst.DEBOUNCE_CYCLES = 4;
    defparam dut.button_latch_inst.LATCH_CYCLES    = 16;

    // -------------------------------------------------------------------------
    // Clock
    // -------------------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    // -------------------------------------------------------------------------
    // PASS/FAIL counters + background event monitors
    // -------------------------------------------------------------------------
    int checks_total = 0;
    int checks_pass  = 0;
    int checks_fail  = 0;

    int rx_valid_events;
    int rx_error_events;
    int tx_start_edges;
    int tx_led_rises;

    logic prev_uart_tx;
    logic prev_led0;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_valid_events <= 0;
            rx_error_events <= 0;
            tx_start_edges  <= 0;
            tx_led_rises    <= 0;
            prev_uart_tx    <= 1'b1;
            prev_led0       <= 1'b0;
        end else begin
            if (dut.rx_msg_valid) rx_valid_events <= rx_valid_events + 1;
            if (dut.rx_msg_error) rx_error_events <= rx_error_events + 1;

            if (prev_uart_tx && !uart_tx) tx_start_edges <= tx_start_edges + 1;
            prev_uart_tx <= uart_tx;

            if (!prev_led0 && led[0]) tx_led_rises <= tx_led_rises + 1;
            prev_led0 <= led[0];
        end
    end

    // -------------------------------------------------------------------------
    // Helper tasks
    // -------------------------------------------------------------------------
    task automatic test_start(input string name);
        $display("\n------------------------------------------------------------");
        $display("[TEST] %s", name);
        $display("------------------------------------------------------------");
    endtask

    task automatic info(input string msg);
        $display("INFO  %s", msg);
    endtask

    task automatic check(input string msg, input bit condition);
        checks_total++;
        if (condition) begin
            checks_pass++;
            $display("PASS  %s", msg);
        end else begin
            checks_fail++;
            $display("FAIL  %s", msg);
        end
    endtask

    function automatic string printable_char(input logic [7:0] c);
        if (c >= 8'h20 && c <= 8'h7E) printable_char = $sformatf("%c", c);
        else if (c == 8'h0A)          printable_char = "LF";
        else if (c == 8'h0D)          printable_char = "CR";
        else                          printable_char = ".";
    endfunction

    task automatic tick(input int n);
        repeat (n) @(posedge clk);
    endtask

    task automatic apply_reset;
        begin
            rst_n      = 1'b0;
            center_btn = 1'b0;
            sw         = 16'h0000;
            rx_in_drv  = 1'b1;
            tick(20);
            rst_n = 1'b1;
            tick(20);
        end
    endtask

    task automatic set_tx_mode;
        begin
            sw[15] = 1'b0;
            tick(5);
        end
    endtask

    task automatic set_rx_mode;
        begin
            sw[15] = 1'b1;
            tick(5);
        end
    endtask

    // -------------------------------------------------------------------------
    // RX serial driver - drives the external PC->FPGA UART input.
    // -------------------------------------------------------------------------
    task automatic send_rx_byte(input logic [7:0] data, input bit good_stop = 1'b1);
        begin
            rx_in_drv = 1'b1;
            tick(2);

            rx_in_drv = 1'b0;            // start bit
            tick(UART_BIT_CYCLES);

            for (int i = 0; i < 8; i++) begin
                rx_in_drv = data[i];     // LSB first
                tick(UART_BIT_CYCLES);
            end

            rx_in_drv = good_stop;       // stop bit
            tick(UART_BIT_CYCLES);
            rx_in_drv = 1'b1;
        end
    endtask

    task automatic send_rx_string(input string pkt);
        logic [7:0] ch;
        begin
            $display("INFO  Sending RX string: %s", pkt);
            for (int i = 0; i < pkt.len(); i++) begin
                ch = pkt.getc(i);
                send_rx_byte(ch, 1'b1);
            end
            tick(20);
        end
    endtask

    task automatic wait_parser_result(
        output bit got_valid,
        output bit got_error,
        input  int timeout_cycles = RX_RESULT_TIMEOUT
    );
        begin
            got_valid = 1'b0;
            got_error = 1'b0;
            for (int i = 0; i < timeout_cycles; i++) begin
                @(posedge clk);
                if (dut.rx_msg_valid) begin got_valid = 1'b1; return; end
                if (dut.rx_msg_error) begin got_error = 1'b1; return; end
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // TX helpers
    // -------------------------------------------------------------------------
    task automatic press_button_long_enough;
        begin
            info($sformatf("Pressing BTNC for %0d clock cycles", BTN_HOLD_CYCLES));
            center_btn = 1'b1;
            tick(BTN_HOLD_CYCLES);
            center_btn = 1'b0;
            tick(20);
        end
    endtask

    task automatic receive_tx_byte(
        output bit          ok,
        output logic [7:0]  data,
        input  int          timeout_cycles = TX_START_TIMEOUT
    );
        int wait_count;
        begin
            ok   = 1'b0;
            data = 8'h00;

            // Wait for falling start bit on uart_tx.
            wait_count = 0;
            while (uart_tx !== 1'b0 && wait_count < timeout_cycles) begin
                @(posedge clk);
                wait_count++;
            end

            if (wait_count >= timeout_cycles) return;

            // Step to middle of bit 0.
            tick(UART_BIT_CYCLES + HALF_BIT_CYCLES);

            for (int i = 0; i < 8; i++) begin
                data[i] = uart_tx;
                tick(UART_BIT_CYCLES);
            end

            // Stop bit should be high one bit later.
            ok = (uart_tx === 1'b1);
            tick(HALF_BIT_CYCLES);
        end
    endtask

    task automatic wait_tx_idle(
        output bit idle_ok,
        input  int timeout_cycles = TX_IDLE_TIMEOUT
    );
        begin
            idle_ok = 1'b0;
            for (int i = 0; i < timeout_cycles; i++) begin
                @(posedge clk);
                // IDLE in our FSM enum = 3'd0
                if (uart_tx === 1'b1 && dut.tx_fsm_inst.state == 3'd0) begin
                    idle_ok = 1'b1;
                    return;
                end
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Main test sequence
    // -------------------------------------------------------------------------
    initial begin
        bit          got_valid;
        bit          got_error;
        bit          tx_ok;
        bit          idle_ok;
        logic [7:0]  tx_b0, tx_b1, tx_b2, tx_b3;
        int          v_before, e_before, tx_edges_before, led_rises_before;

        $display("============================================================");
        $display("Lab 5 - trx_uart_controller full-system testbench");
        $display("============================================================");

        apply_reset();

        // ---------------------------------------------------------------------
        test_start("1. Reset behaviour");
        check("UART TX output idles high after reset",       uart_tx === 1'b1);
        check("LED[0] (TX activity) is low after reset",     led[0]  === 1'b0);
        check("LED[15] mirrors sw[15] after reset",          led[15] === sw[15]);
        check("RX row output resets to 0",                   dut.rx_row_val   === 8'd0);
        check("RX column output resets to 0",                dut.rx_col_val   === 8'd0);
        check("RX pixel output resets to 0",                 dut.rx_pixel_val === 8'd0);

        // ---------------------------------------------------------------------
        test_start("2. TX mode blocks the RX path");
        set_tx_mode();
        v_before = rx_valid_events;
        e_before = rx_error_events;
        send_rx_string("{R012,C034,V056}");
        tick(RX_RESULT_TIMEOUT);
        check("No RX valid pulse while sw[15]=0", rx_valid_events == v_before);
        check("No RX error pulse while sw[15]=0", rx_error_events == e_before);
        check("RX row preserved in TX mode",      dut.rx_row_val   === 8'd0);
        check("RX col preserved in TX mode",      dut.rx_col_val   === 8'd0);
        check("RX pixel preserved in TX mode",    dut.rx_pixel_val === 8'd0);

        // ---------------------------------------------------------------------
        test_start("3. Valid RX packet updates outputs");
        set_rx_mode();
        v_before = rx_valid_events;
        e_before = rx_error_events;
        send_rx_string("{R012,C034,V056}");
        tick(100);
        check("Parser asserted msg_valid for valid packet",  rx_valid_events > v_before);
        check("Parser did not assert msg_error",             rx_error_events == e_before);
        check("RX row updated to 12",                         dut.rx_row_val   === 8'd12);
        check("RX col updated to 34",                         dut.rx_col_val   === 8'd34);
        check("RX pixel updated to 56",                       dut.rx_pixel_val === 8'd56);

        // ---------------------------------------------------------------------
        test_start("4. RX range safety rejects value above 255");
        v_before = rx_valid_events;
        e_before = rx_error_events;
        send_rx_string("{R256,C034,V056}");
        tick(100);
        check("Parser asserted msg_error for R256",          rx_error_events > e_before);
        check("Parser did not assert msg_valid",             rx_valid_events == v_before);
        check("RX row preserved after range error",          dut.rx_row_val   === 8'd12);
        check("RX col preserved after range error",          dut.rx_col_val   === 8'd34);
        check("RX pixel preserved after range error",        dut.rx_pixel_val === 8'd56);

        // ---------------------------------------------------------------------
        test_start("5. RX syntax safety rejects wrong header");
        v_before = rx_valid_events;
        e_before = rx_error_events;
        send_rx_string("{X012,C034,V056}");
        tick(100);
        check("Parser asserted msg_error for wrong R header", rx_error_events > e_before);
        check("Parser did not assert msg_valid",              rx_valid_events == v_before);
        check("RX outputs preserved after syntax error",
              dut.rx_row_val   === 8'd12 &&
              dut.rx_col_val   === 8'd34 &&
              dut.rx_pixel_val === 8'd56);

        // ---------------------------------------------------------------------
        test_start("6. RX digit safety rejects non-digit character");
        v_before = rx_valid_events;
        e_before = rx_error_events;
        send_rx_string("{R12A,C034,V056}");
        tick(100);
        check("Parser asserted msg_error for non-digit row",  rx_error_events > e_before);
        check("Parser did not assert msg_valid",              rx_valid_events == v_before);
        check("RX outputs preserved after non-digit error",
              dut.rx_row_val   === 8'd12 &&
              dut.rx_col_val   === 8'd34 &&
              dut.rx_pixel_val === 8'd56);

        // ---------------------------------------------------------------------
        test_start("7. RX MAC framing rejects missing closing brace");
        v_before = rx_valid_events;
        e_before = rx_error_events;
        send_rx_string("{R012,C034,V056!");
        tick(RX_RESULT_TIMEOUT);
        check("No parser valid event (MAC drops frame before parser)",
              rx_valid_events == v_before);
        check("No parser error event (MAC drops frame before parser)",
              rx_error_events == e_before);
        check("RX outputs preserved after MAC drop",
              dut.rx_row_val   === 8'd12 &&
              dut.rx_col_val   === 8'd34 &&
              dut.rx_pixel_val === 8'd56);

        // ---------------------------------------------------------------------
        test_start("8. RX MAC active resynchronization");
        info("Sending partial frame then a fresh valid one");
        v_before = rx_valid_events;
        send_rx_string("{R999,");
        send_rx_string("{R001,C002,V003}");
        tick(100);
        check("Parser asserted msg_valid after resync",       rx_valid_events > v_before);
        check("RX row after resync is 1",                     dut.rx_row_val   === 8'd1);
        check("RX col after resync is 2",                     dut.rx_col_val   === 8'd2);
        check("RX pixel after resync is 3",                   dut.rx_pixel_val === 8'd3);

        // Also check display mux now shows RX fields
        check("Display T0 reflects rx_pixel_val (RX mode)",  dut.t0_data === 8'd3);
        check("Display T2 reflects rx_col_val (RX mode)",    dut.t2_data === 8'd2);
        check("Display T3 reflects rx_row_val (RX mode)",    dut.t3_data === 8'd1);
        check("Display dash_enable lit on T1 in RX mode",    dut.dash_enable === 8'b0000_1100);

        // ---------------------------------------------------------------------
        test_start("9. RX mode blocks the TX button");
        set_rx_mode();
        tx_edges_before = tx_start_edges;
        press_button_long_enough();
        tick(TX_START_TIMEOUT);
        check("No TX UART start edge while sw[15]=1", tx_start_edges == tx_edges_before);
        check("UART output stays idle high in RX mode", uart_tx === 1'b1);
        check("LED[0] (TX activity) stays low in RX mode", led[0]  === 1'b0);

        // ---------------------------------------------------------------------
        test_start("10. TX minimum sequence (1x1 'X', no inter-byte delay)");
        apply_reset();
        set_tx_mode();
        sw[7:0]   = 8'h58;          // 'X'
        sw[9:8]   = 2'b00;          // speed 0
        sw[14:13] = 2'b00;          // size 0 -> 1x1
        tick(10);

        led_rises_before = tx_led_rises;

        fork
            begin
                press_button_long_enough();
            end
            begin
                receive_tx_byte(tx_ok, tx_b0);
                check("TX byte 0 received", tx_ok);
                $display("INFO  TX byte 0 = 0x%02h '%s'", tx_b0, printable_char(tx_b0));

                receive_tx_byte(tx_ok, tx_b1);
                check("TX byte 1 received", tx_ok);
                $display("INFO  TX byte 1 = 0x%02h '%s'", tx_b1, printable_char(tx_b1));

                receive_tx_byte(tx_ok, tx_b2);
                check("TX byte 2 received", tx_ok);
                $display("INFO  TX byte 2 = 0x%02h '%s'", tx_b2, printable_char(tx_b2));

                receive_tx_byte(tx_ok, tx_b3);
                check("TX byte 3 received", tx_ok);
                $display("INFO  TX byte 3 = 0x%02h '%s'", tx_b3, printable_char(tx_b3));
            end
        join

        check("TX byte 0 is data 'X' (0x58)", tx_b0 === 8'h58);
        check("TX byte 1 is space (0x20)",    tx_b1 === 8'h20);
        check("TX byte 2 is CR (0x0D)",       tx_b2 === 8'h0D);
        check("TX byte 3 is LF (0x0A)",       tx_b3 === 8'h0A);

        wait_tx_idle(idle_ok);
        check("TX returned to IDLE state",                idle_ok);
        check("UART output idle high after TX completion", uart_tx === 1'b1);
        check("LED[0] toggled during transmission",       tx_led_rises > led_rises_before);
        check("Display T3 shows row count = 1",            dut.t3_data === 8'h01);

        // ---------------------------------------------------------------------
        test_start("11. Mode indicator LED[15]");
        sw[15] = 1'b0; tick(3);
        check("LED[15] is 0 in TX mode", led[15] === 1'b0);
        sw[15] = 1'b1; tick(3);
        check("LED[15] is 1 in RX mode", led[15] === 1'b1);

        // ---------------------------------------------------------------------
        $display("\n============================================================");
        $display("TEST SUMMARY");
        $display("  Total checks : %0d", checks_total);
        $display("  Passed       : %0d", checks_pass);
        $display("  Failed       : %0d", checks_fail);
        if (checks_fail == 0)
            $display("  RESULT       : PASS");
        else
            $display("  RESULT       : FAIL");
        $display("============================================================");

        #1000;
        $finish;
    end

endmodule
