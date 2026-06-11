/**
 * rx_parser.sv - validate and decode an RX message
 *
 * Input: the 16-byte buffer maintained by rx_mac. When is_msg pulses, this
 * module checks the buffer is a well-formed "{R###,C###,V###}" frame and,
 * if so, decodes the three decimal fields and latches them.
 *
 * Buffer layout (from rx_mac):
 *
 *   msg[15] = '{'                msg[10] = ','
 *   msg[14] = 'R'                msg[9]  = 'C'
 *   msg[13] = row hundreds       msg[8]  = col hundreds
 *   msg[12] = row tens           msg[7]  = col tens
 *   msg[11] = row ones           msg[6]  = col ones
 *   msg[5]  = ','                msg[3]  = pixel hundreds
 *   msg[4]  = 'V'                msg[2]  = pixel tens
 *                                msg[1]  = pixel ones
 *                                msg[0]  = '}'
 *
 * Validation rules:
 *   1. The 5 fixed slots above must hold their literal characters.
 *   2. The 9 number slots must hold ASCII '0'..'9' (0x30..0x39).
 *   3. After 100*d2 + 10*d1 + d0, each field must be <= 255 (fits in 8 bits).
 *
 * If all three rules pass, row_val/col_val/pixel_val update and msg_valid
 * pulses for one cycle. If any rule fails the outputs stay at their previous
 * values and msg_error pulses for one cycle.
 *
 * FSM: IDLE -> VALIDATE -> DONE_OK | DONE_ERR -> IDLE
 * Three clock cycles per message - well below the ~17400-cycle gap between
 * consecutive bytes at 57600 baud, so we never miss an is_msg pulse.
 */

`timescale 1ns / 1ps

module rx_parser (
    input  logic       clk,
    input  logic       rst_n,

    input  logic [7:0] msg [15:0],     // sliding buffer from rx_mac
    input  logic       is_msg,         // 1-cycle "buffer looks complete" pulse

    output logic [7:0] row_val,        // latched on a fully-valid message
    output logic [7:0] col_val,
    output logic [7:0] pixel_val,
    output logic       msg_valid,      // 1-cycle pulse on successful latch
    output logic       msg_error       // 1-cycle pulse on rejected message
);

    // ---------------------------------------------------------------
    // ASCII reference values
    // ---------------------------------------------------------------
    localparam logic [7:0] ASCII_R     = 8'h52;
    localparam logic [7:0] ASCII_C     = 8'h43;
    localparam logic [7:0] ASCII_V     = 8'h56;
    localparam logic [7:0] ASCII_COMMA = 8'h2C;
    localparam logic [7:0] ASCII_0     = 8'h30;
    localparam logic [7:0] ASCII_9     = 8'h39;

    // ---------------------------------------------------------------
    // Rule 1: fixed-position characters
    // ---------------------------------------------------------------
    // msg[15] and msg[0] are already checked by rx_mac (= '{' and '}').
    wire syntax_ok = (msg[14] == ASCII_R)     &&
                     (msg[10] == ASCII_COMMA) &&
                     (msg[9]  == ASCII_C)     &&
                     (msg[5]  == ASCII_COMMA) &&
                     (msg[4]  == ASCII_V);

    // ---------------------------------------------------------------
    // Rule 2: digit slots are ASCII '0'..'9'
    // ---------------------------------------------------------------
    function automatic logic is_digit(input logic [7:0] c);
        return (c >= ASCII_0) && (c <= ASCII_9);
    endfunction

    wire row_digits_ok = is_digit(msg[13]) && is_digit(msg[12]) && is_digit(msg[11]);
    wire col_digits_ok = is_digit(msg[8])  && is_digit(msg[7])  && is_digit(msg[6]);
    wire pix_digits_ok = is_digit(msg[3])  && is_digit(msg[2])  && is_digit(msg[1]);

    // ---------------------------------------------------------------
    // Rule 3: decimal-to-binary conversion + range check
    //   value = 100 * hundreds + 10 * tens + ones
    //   max possible 999 -> need 10 bits; we use 11 for headroom
    // ---------------------------------------------------------------
    wire [10:0] row_sum = 11'd100 * msg[13][3:0]
                        + 11'd10  * msg[12][3:0]
                        +           msg[11][3:0];

    wire [10:0] col_sum = 11'd100 * msg[8][3:0]
                        + 11'd10  * msg[7][3:0]
                        +           msg[6][3:0];

    wire [10:0] pix_sum = 11'd100 * msg[3][3:0]
                        + 11'd10  * msg[2][3:0]
                        +           msg[1][3:0];

    wire row_in_range = (row_sum <= 11'd255);
    wire col_in_range = (col_sum <= 11'd255);
    wire pix_in_range = (pix_sum <= 11'd255);

    // ---------------------------------------------------------------
    // Aggregate result - all 7 checks must pass
    // ---------------------------------------------------------------
    wire all_ok = syntax_ok
               && row_digits_ok && col_digits_ok && pix_digits_ok
               && row_in_range  && col_in_range  && pix_in_range;

    // ---------------------------------------------------------------
    // FSM
    // ---------------------------------------------------------------
    typedef enum logic [1:0] {
        IDLE     = 2'd0,
        VALIDATE = 2'd1,
        DONE_OK  = 2'd2,
        DONE_ERR = 2'd3
    } state_t;

    state_t state, state_next;

    always_ff @(posedge clk or negedge rst_n) begin
        if (~rst_n) state <= IDLE;
        else        state <= state_next;
    end

    always_comb begin
        state_next = state;
        unique case (state)
            IDLE:     if (is_msg) state_next = VALIDATE;
            VALIDATE: state_next = all_ok ? DONE_OK : DONE_ERR;
            DONE_OK:  state_next = IDLE;
            DONE_ERR: state_next = IDLE;
        endcase
    end

    // ---------------------------------------------------------------
    // Output registers - only updated when we land in DONE_OK
    // ---------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            row_val   <= 8'h00;
            col_val   <= 8'h00;
            pixel_val <= 8'h00;
            msg_valid <= 1'b0;
            msg_error <= 1'b0;
        end else begin
            // single-cycle pulse defaults
            msg_valid <= 1'b0;
            msg_error <= 1'b0;

            unique case (state_next)
                DONE_OK: begin
                    row_val   <= row_sum[7:0];
                    col_val   <= col_sum[7:0];
                    pixel_val <= pix_sum[7:0];
                    msg_valid <= 1'b1;
                end
                DONE_ERR: begin
                    msg_error <= 1'b1;
                    // row_val/col_val/pixel_val intentionally NOT updated
                end
                default: ;
            endcase
        end
    end

endmodule
