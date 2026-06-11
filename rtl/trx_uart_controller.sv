/**
 * trx_uart_controller.sv - Top-Level UART TRX Controller (Lab 5)
 *
 * Lab 5 top. Extends the Lab 4 TX-only controller with a UART receive path
 * and a TX/RX mode selector on sw[15]:
 *
 *   sw[15] = 0  ->  TX mode (Lab 4 behaviour). BTNC active, RX line forced
 *                   to idle (1) so the RX path stays in IDLE.
 *   sw[15] = 1  ->  RX mode. BTNC gated to 0 so the TX FSM never gets a
 *                   start pulse and uart_tx stays high. RX line passes
 *                   through and the RX path receives messages.
 *
 * led[15] mirrors sw[15] as the RX/TX indicator. led[0] still toggles per
 * transmitted data byte. led[14:1] are tied off.
 *
 * The RX modules (rx_phy / rx_mac / rx_parser) are added in later steps;
 * this revision only adds the scaffolding (ports, gating, led[15]).
 */

`timescale 1ns / 1ps

module trx_uart_controller (
    input  logic        clk,           // 100 MHz system clock
    input  logic        rst_n,         // CPU_RESETN (active-low)

    input  logic [15:0] sw,            // sw[15]=mode, [14:13]=size, [9:8]=speed, [7:0]=data
    input  logic        center_btn,    // BTNC - configuration latch (1 s hold)
    input  logic        rx_in,         // UART RX serial line (PC -> FPGA, pin C4)

    output logic        uart_tx,       // UART TX serial line
    output logic [15:0] led,           // led[0] toggle-per-byte; led[15] = mode indicator

    output logic [7:0]  an,            // 7-seg anode enables (active low)
    output logic [6:0]  segment,       // 7-seg cathodes a..g (active low)
    output logic        dp             // 7-seg decimal point (active low)
);

    // --------------------------------------------------------------
    // TX/RX mode gating (Lab 5 additions)
    // --------------------------------------------------------------
    // BTNC is only "seen" in TX mode. In RX mode the TX FSM never receives
    // a start pulse, so it stays in IDLE and uart_tx stays at 1 naturally.
    logic gated_btnc;
    assign gated_btnc = center_btn & ~sw[15];

    // RX line is forced to idle (1) in TX mode so the RX FSM stays in IDLE
    // and ignores any noise on the unused line.
    logic gated_rx_in;
    assign gated_rx_in = sw[15] ? rx_in : 1'b1;

    // led[15] is the RX/TX indicator. led[14:1] are unused.
    assign led[15]    = sw[15];
    assign led[14:1]  = '0;

    // --------------------------------------------------------------
    // Internal signals (Lab 4 TX path - unchanged)
    // --------------------------------------------------------------
    logic baud_pulse;
    logic [14:0] latched_config;
    logic        latch_triggered;

    logic        start_transmission;
    logic [7:0]  tx_byte;
    logic [1:0]  speed_mode;
    logic [16:0] num_bytes_to_send;
    logic [8:0]  bytes_per_row_mask;

    logic        data_byte_sent;
    logic        delimiter_sent;
    logic        row_done;
    logic        led_toggle;
    logic        transmission_done;
    logic        transmission_in_progress;

    logic [16:0] transmitted_count;
    logic [7:0]  row_count;
    logic        led_state;

    // --------------------------------------------------------------
    // RX path signals (Lab 5 additions)
    // --------------------------------------------------------------
    logic [7:0]  rx_byte;          // PHY -> MAC: latest received byte
    logic        rx_byte_ready;    // PHY -> MAC: 1-cycle "byte is valid" pulse
    logic [7:0]  rx_msg [15:0];    // MAC -> Parser: 16-byte sliding window
    logic        rx_is_msg;        // MAC -> Parser: 1-cycle "framing match" pulse
    logic [7:0]  rx_row_val;       // Parser -> Display: latched row index
    logic [7:0]  rx_col_val;       // Parser -> Display: latched col index
    logic [7:0]  rx_pixel_val;     // Parser -> Display: latched pixel value
    logic        rx_msg_valid;     // Parser status pulse (currently for debug)
    logic        rx_msg_error;     // Parser status pulse (currently for debug)

    // 7-seg field bytes (high nibble = left digit, low nibble = right digit)
    logic [7:0] t0_data, t1_data, t2_data, t3_data;
    logic [7:0] dp_enable;

    // --------------------------------------------------------------
    // Configuration decoding from latched switches (Lab 4 - unchanged)
    // --------------------------------------------------------------
    assign speed_mode = latched_config[9:8];
    assign tx_byte    = latched_config[7:0];

    assign num_bytes_to_send = (latched_config[14:13] == 2'b00) ? 17'd1     :
                               (latched_config[14:13] == 2'b01) ? 17'd1024  :
                               (latched_config[14:13] == 2'b10) ? 17'd16384 :
                                                                  17'd65536;

    assign bytes_per_row_mask = (latched_config[14:13] == 2'b00) ? 9'd0   :
                                (latched_config[14:13] == 2'b01) ? 9'd31  :
                                (latched_config[14:13] == 2'b10) ? 9'd127 :
                                                                   9'd255;

    // --------------------------------------------------------------
    // 7-segment field byte composition (mode-dependent in Lab 5)
    //
    //                 TX mode (sw[15]=0)         RX mode (sw[15]=1)
    //  T0 (data)      data byte                  pixel_val
    //  T1 (special)   speed code  0.0..2.0       dashes "--"
    //  T2 (count)     row width  01/20/80/00     col_val
    //  T3 (progress)  row count                  row_val
    //
    //  dp_enable      bit3 (DP on T1 left digit) all zero (no DP in RX mode)
    //  dash_enable    all zero                   bits 2 & 3 (T1's two digits)
    // --------------------------------------------------------------

    // TX field values (Lab 4)
    logic [7:0] tx_t1_data, tx_t2_data;

    always_comb begin
        unique case (speed_mode)
            2'b00: tx_t1_data = 8'h00;   // 0.0
            2'b01: tx_t1_data = 8'h05;   // 0.5
            2'b10: tx_t1_data = 8'h10;   // 1.0
            2'b11: tx_t1_data = 8'h20;   // 2.0
        endcase
    end

    always_comb begin
        unique case (latched_config[14:13])
            2'b00: tx_t2_data = 8'h01;
            2'b01: tx_t2_data = 8'h20;
            2'b10: tx_t2_data = 8'h80;
            2'b11: tx_t2_data = 8'h00;
        endcase
    end

    // Mode mux
    logic [7:0] dash_enable;
    assign t0_data     = sw[15] ? rx_pixel_val : tx_byte;
    assign t1_data     = sw[15] ? 8'h00        : tx_t1_data;  // value hidden by dash in RX
    assign t2_data     = sw[15] ? rx_col_val   : tx_t2_data;
    assign t3_data     = sw[15] ? rx_row_val   : row_count;
    assign dp_enable   = sw[15] ? 8'b0         : 8'b0000_1000;
    assign dash_enable = sw[15] ? 8'b0000_1100 : 8'b0;        // T1's two digits in RX

    // --------------------------------------------------------------
    // Submodules
    // --------------------------------------------------------------
    uart_phy #(
        .DIVISOR(1736)
    ) uart_phy_inst (
        .clk(clk),
        .rst_n(rst_n),
        .baud_pulse(baud_pulse)
    );

    // RX bit-level PHY: sees gated_rx_in (forced idle in TX mode), produces
    // one byte + byte_ready pulse per 10-bit UART frame.
    rx_phy #(
        .BAUD_DIVISOR(1736)
    ) rx_phy_inst (
        .clk(clk),
        .rst_n(rst_n),
        .rx_line(gated_rx_in),
        .rx_byte(rx_byte),
        .byte_ready(rx_byte_ready)
    );

    // RX message assembler: shifts incoming bytes into a 16-byte window and
    // pulses is_msg when the framing characters '{' and '}' appear at the
    // ends of the window.
    rx_mac rx_mac_inst (
        .clk(clk),
        .rst_n(rst_n),
        .rx_byte(rx_byte),
        .byte_ready(rx_byte_ready),
        .msg(rx_msg),
        .is_msg(rx_is_msg)
    );

    // RX parser: validates "{R###,C###,V###}" and latches the three fields.
    rx_parser rx_parser_inst (
        .clk(clk),
        .rst_n(rst_n),
        .msg(rx_msg),
        .is_msg(rx_is_msg),
        .row_val(rx_row_val),
        .col_val(rx_col_val),
        .pixel_val(rx_pixel_val),
        .msg_valid(rx_msg_valid),
        .msg_error(rx_msg_error)
    );

    button_latch #(
        .DEBOUNCE_CYCLES(2_000_000),
        .LATCH_CYCLES(100_000_000)
    ) button_latch_inst (
        .clk(clk),
        .rst_n(rst_n),
        .button(gated_btnc),            // gated by sw[15]
        .sw_config(sw[14:0]),           // RX mode bit not part of the latched config
        .latched_config(latched_config),
        .latch_triggered(latch_triggered)
    );

    tx_fsm #(
        .DELAY_0_CYCLES(0),
        .DELAY_1_CYCLES(5_000_000),
        .DELAY_2_CYCLES(10_000_000),
        .DELAY_3_CYCLES(20_000_000)
    ) tx_fsm_inst (
        .clk(clk),
        .rst_n(rst_n),
        .baud_pulse(baud_pulse),
        .start(start_transmission),
        .data_byte(tx_byte),
        .speed_config(speed_mode),
        .num_bytes(num_bytes_to_send),
        .bytes_per_row_mask(bytes_per_row_mask),
        .tx_line(uart_tx),
        .tx_data(),
        .data_byte_sent(data_byte_sent),
        .delimiter_sent(delimiter_sent),
        .row_done(row_done),
        .led_toggle(led_toggle),
        .transmission_done(transmission_done),
        .current_state(),
        .fsm_active(transmission_in_progress)
    );

    byte_counter byte_counter_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(start_transmission),
        .data_byte_sent(data_byte_sent),
        .num_bytes(num_bytes_to_send),
        .count(transmitted_count),
        .done()
    );

    row_counter row_counter_inst (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start(start_transmission),
        .row_done(row_done),
        .count(row_count)
    );

    seven_segment_controller #(
        .DIGIT_PERIOD(25_000)
    ) seven_seg_inst (
        .clk(clk),
        .rst_n(rst_n),
        .t0_data(t0_data),
        .t1_data(t1_data),
        .t2_data(t2_data),
        .t3_data(t3_data),
        .dp_enable(dp_enable),
        .dash_enable(dash_enable),
        .an(an),
        .segment(segment),
        .dp(dp)
    );

    // --------------------------------------------------------------
    // Misc control
    // --------------------------------------------------------------
    assign start_transmission = latch_triggered;

    always_ff @(posedge clk or negedge rst_n) begin
        if (~rst_n)          led_state <= 1'b0;
        else if (led_toggle) led_state <= ~led_state;
    end

    assign led[0] = led_state;

endmodule
