/**
 * rx_mac.sv - UART RX message assembler (16-byte sliding buffer)
 *
 * Sits between rx_phy (which delivers one received byte at a time) and
 * rx_parser (which expects a 16-byte message buffer). Every byte that
 * arrives is shifted "up" the buffer:
 *
 *   msg[0]  <- newest byte (just received)
 *   msg[1]  <- previous byte
 *   ...
 *   msg[15] <- oldest byte still in the window
 *
 * Therefore after receiving the full `{R###,C###,V###}` sequence in order,
 * the opening '{' sits at msg[15] and the closing '}' sits at msg[0]. That
 * matches the indexing the parser uses to find the fixed-position fields.
 *
 * is_msg pulses for one cycle on the clock AFTER a fresh byte completes
 * the framing pattern (msg[15] == '{' AND msg[0] == '}'). If the message
 * fails parser validation, the buffer keeps shifting and the next valid
 * frame will produce another is_msg pulse.
 */

`timescale 1ns / 1ps

module rx_mac (
    input  logic       clk,
    input  logic       rst_n,

    // from rx_phy
    input  logic [7:0] rx_byte,
    input  logic       byte_ready,    // 1-cycle pulse when rx_byte is valid

    // to rx_parser
    output logic [7:0] msg [15:0],    // sliding 16-byte window
    output logic       is_msg         // 1-cycle pulse on complete '{...}' framing
);

    // Delayed copy of byte_ready: needed because on cycle N where byte_ready
    // is high we *issue* the shift (msg[] takes its new values at the next
    // clock edge). The framing check must be performed on cycle N+1 to see
    // the updated buffer.
    logic byte_ready_d;

    // ---------------------------------------------------------------
    // Buffer + shift logic
    // ---------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            byte_ready_d <= 1'b0;
            for (int i = 0; i < 16; i++) msg[i] <= 8'h00;
        end else begin
            byte_ready_d <= byte_ready;

            if (byte_ready) begin
                // shift everything up by one slot
                for (int i = 15; i > 0; i--) msg[i] <= msg[i-1];
                // newest byte enters at slot 0
                msg[0] <= rx_byte;
            end
        end
    end

    // ---------------------------------------------------------------
    // Framing detector
    // ---------------------------------------------------------------
    // ASCII '{' = 0x7B, ASCII '}' = 0x7D.
    // Gated with byte_ready_d so the pulse lasts exactly one cycle even
    // if the framing match remains true while no new bytes are arriving.
    assign is_msg = byte_ready_d && (msg[15] == 8'h7B) && (msg[0] == 8'h7D);

endmodule
