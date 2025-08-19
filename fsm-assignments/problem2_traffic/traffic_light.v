module traffic_light(
    input  wire clk,
    input  wire rst,     // sync active-high reset
    input  wire tick,    // 1Hz tick (1 cycle pulse)
    output reg ns_g, ns_y, ns_r,
    output reg ew_g, ew_y, ew_r
);

    // State encoding
    localparam NS_GREEN  = 2'b00;
    localparam NS_YELLOW = 2'b01;
    localparam EW_GREEN  = 2'b10;
    localparam EW_YELLOW = 2'b11;

    reg [1:0] state, next_state;
    reg [2:0] tick_count;  // enough to count up to 5

    // Duration per state
    localparam NS_G_TIME = 5;
    localparam NS_Y_TIME = 2;
    localparam EW_G_TIME = 5;
    localparam EW_Y_TIME = 2;

    // Sequential block
    always @(posedge clk) begin
        if (rst) begin
            state <= NS_GREEN;
            tick_count <= 0;
        end else begin
            if (tick) begin
                if ( (state == NS_GREEN  && tick_count == NS_G_TIME-1) ||
                     (state == NS_YELLOW && tick_count == NS_Y_TIME-1) ||
                     (state == EW_GREEN  && tick_count == EW_G_TIME-1) ||
                     (state == EW_YELLOW && tick_count == EW_Y_TIME-1) ) begin
                    state <= next_state;
                    tick_count <= 0;
                end else begin
                    tick_count <= tick_count + 1;
                end
            end
        end
    end

    // Next state logic
    always @(*) begin
        case (state)
            NS_GREEN:   next_state = NS_YELLOW;
            NS_YELLOW:  next_state = EW_GREEN;
            EW_GREEN:   next_state = EW_YELLOW;
            EW_YELLOW:  next_state = NS_GREEN;
            default:    next_state = NS_GREEN;
        endcase
    end

    // Output logic (Moore FSM)
    always @(*) begin
        // Default
        ns_g = 0; ns_y = 0; ns_r = 0;
        ew_g = 0; ew_y = 0; ew_r = 0;

        case (state)
            NS_GREEN: begin
                ns_g = 1; ew_r = 1;
            end
            NS_YELLOW: begin
                ns_y = 1; ew_r = 1;
            end
            EW_GREEN: begin
                ew_g = 1; ns_r = 1;
            end
            EW_YELLOW: begin
                ew_y = 1; ns_r = 1;
            end
        endcase
    end

endmodule
