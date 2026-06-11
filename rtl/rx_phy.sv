/**
 * rx_phy.sv - UART RX bit-level receiver
 *
 * Watches rx_line for a falling edge (the start bit) and samples each of
 * 8 data bits in the middle of its bit cell, LSB-first. On a valid stop
 * bit (1) the assembled byte is presented on rx_byte and byte_ready pulses
 * for one cycle. A framing error (stop bit not 1) silently drops the byte.
 *
 * FSM (Moore):
 *   IDLE         wait for rx_line to go low (start bit)
 *   START_CHECK  wait 1/2 baud; if line still low it's a real start bit
 *   SAMPLE_BIT   wait 1 full baud, sample, shift in. Repeat 8 times.
 *   STOP_CHECK   wait 1 full baud, sample. If high, commit byte + pulse.
 *
 * At 100 MHz / 57600 baud: BAUD_DIVISOR = 1736 cycles per bit,
 * HALF_BAUD = 868 cycles to reach the middle of the start bit.
 */

`timescale 1ns / 1ps

module rx_phy #(
    parameter int BAUD_DIVISOR = 1736,
    parameter int HALF_BAUD    = BAUD_DIVISOR / 2
) (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       rx_line,        // raw RX line from the pin (async)
    output logic [7:0] rx_byte,        // last received byte (valid with byte_ready)
    output logic       byte_ready      // 1-cycle pulse when rx_byte is fresh
);

    // ---------------------------------------------------------------
    // 2-stage synchronizer - rx_line is async to clk, this avoids
    // metastability on the FSM inputs.
    // ---------------------------------------------------------------
    logic rx_sync_0, rx_sync_1;
    always_ff @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rx_sync_0 <= 1'b1;
            rx_sync_1 <= 1'b1;
        end else begin
            rx_sync_0 <= rx_line;
            rx_sync_1 <= rx_sync_0;
        end
    end

    wire rx_safe = rx_sync_1;

    // ---------------------------------------------------------------
    // FSM
    // ---------------------------------------------------------------
    typedef enum logic [1:0] {
        IDLE        = 2'd0,
        START_CHECK = 2'd1,
        SAMPLE_BIT  = 2'd2,
        STOP_CHECK  = 2'd3
    } state_t;

    state_t state, state_next;

    logic [$clog2(BAUD_DIVISOR)-1:0] sample_counter;
    logic [3:0]                      bit_count;
    logic [7:0]                      rx_shift;

    wire reached_half = (sample_counter == (HALF_BAUD    - 1));
    wire reached_full = (sample_counter == (BAUD_DIVISOR - 1));

    // state register
    always_ff @(posedge clk or negedge rst_n) begin
        if (~rst_n) state <= IDLE;
        else        state <= state_next;
    end

    // next-state logic
    always_comb begin
        state_next = state;
        unique case (state)
            IDLE: begin
                if (rx_safe == 1'b0) state_next = START_CHECK;
            end

            START_CHECK: begin
                if (reached_half) begin
                    state_next = (rx_safe == 1'b0) ? SAMPLE_BIT : IDLE;
                end
            end

            SAMPLE_BIT: begin
                if (reached_full && bit_count == 4'd7) state_next = STOP_CHECK;
            end

            STOP_CHECK: begin
                if (reached_full) state_next = IDLE;
            end
        endcase
    end

    // datapath / outputs
    always_ff @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            sample_counter <= '0;
            bit_count      <= '0;
            rx_shift       <= '0;
            rx_byte        <= '0;
            byte_ready     <= 1'b0;
        end else begin
            byte_ready <= 1'b0;            // default: drop the pulse next cycle

            unique case (state)
                IDLE: begin
                    sample_counter <= '0;
                    bit_count      <= '0;
                end

                START_CHECK: begin
                    if (reached_half) sample_counter <= '0;
                    else              sample_counter <= sample_counter + 1'b1;
                end

                SAMPLE_BIT: begin
                    if (reached_full) begin
                        // Sample rx_safe at the middle of this bit cell.
                        // Shift right so the first-received bit (LSB) ends
                        // up at rx_shift[0] after the 8th sample.
                        rx_shift       <= {rx_safe, rx_shift[7:1]};
                        bit_count      <= bit_count + 1'b1;
                        sample_counter <= '0;
                    end else begin
                        sample_counter <= sample_counter + 1'b1;
                    end
                end

                STOP_CHECK: begin
                    if (reached_full) begin
                        sample_counter <= '0;
                        if (rx_safe == 1'b1) begin
                            rx_byte    <= rx_shift;   // commit
                            byte_ready <= 1'b1;       // 1-cycle pulse
                        end
                        // else: framing error, drop the byte silently
                    end else begin
                        sample_counter <= sample_counter + 1'b1;
                    end
                end
            endcase
        end
    end

endmodule
