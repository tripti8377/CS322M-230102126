
`timescale 1ns/1ps

module seq_detect_mealy(
    input  wire clk,
    input  wire rst,   // synchronous active-high
    input  wire din,   // serial input bit per clock
    output reg  y      // 1-cycle pulse when pattern ...1101 seen
);

    
    localparam S0   = 2'd0;
    localparam S1   = 2'd1;
    localparam S11  = 2'd2;
    localparam S110 = 2'd3;

    reg [1:0] state, next_state;

    // Next-state and Mealy output
    always @* begin
        next_state = state;
        y = 1'b0;

        case (state)
            S0: begin
                // matched nothing so far
                if (din) next_state = S1;     // saw '1'
                else     next_state = S0;     // stay
            end

            S1: begin
                // matched "1"
                if (din) next_state = S11;    // saw '11'
                else     next_state = S0;     // '10' -> no prefix match
            end

            S11: begin
                // matched "11"
                if (din) next_state = S11;    // '111' -> suffix '11' holds
                else     next_state = S110;   // '110'
            end

            S110: begin
                // matched "110"
                if (din) begin
                    // complete "1101" -> pulse now; fallback to longest prefix "1"
                    y = 1'b1;
                    next_state = S1;
                end else begin
                    // '1100' -> no prefix
                    next_state = S0;
                end
            end

            default: begin
                next_state = S0;
            end
        endcase
    end

    // State register with synchronous, active-high reset
    always @(posedge clk) begin
        if (rst) state <= S0;
        else     state <= next_state;
    end

endmodule