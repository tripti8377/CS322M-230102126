module vending_mealy(
    input  wire       clk,
    input  wire       rst,       // sync active-high reset
    input  wire [1:0] coin,      // 01=5, 10=10, 00=idle, ignore 11
    output reg        dispense,  // 1-cycle pulse
    output reg        chg5       // 1-cycle pulse when returning 5
);

    // State encoding (Verilog-2001 style)
    parameter S0  = 2'b00;  // total = 0
    parameter S5  = 2'b01;  // total = 5
    parameter S10 = 2'b10;  // total = 10
    parameter S15 = 2'b11;  // total = 15

    reg [1:0] state, next_state;

    // Next state logic + outputs (Mealy machine)
    always @(*) begin
        next_state = state;
        dispense   = 0;
        chg5       = 0;

        case (state)
            S0: begin
                if (coin == 2'b01) next_state = S5;       // +5
                else if (coin == 2'b10) next_state = S10; // +10
            end

            S5: begin
                if (coin == 2'b01) next_state = S10;      // +5
                else if (coin == 2'b10) next_state = S15; // +10
            end

            S10: begin
                if (coin == 2'b01) next_state = S15;      // +5
                else if (coin == 2'b10) begin             // total = 20
                    dispense   = 1;
                    next_state = S0;                      // reset after vend
                end
            end

            S15: begin
                if (coin == 2'b01) begin                  // total = 20
                    dispense   = 1;
                    next_state = S0;
                end
                else if (coin == 2'b10) begin             // total = 25
                    dispense   = 1;
                    chg5       = 1;
                    next_state = S0;
                end
            end
        endcase
    end

    // State register (synchronous reset)
    always @(posedge clk) begin
        if (rst)
            state <= S0;
        else
            state <= next_state;
    end

endmodule
