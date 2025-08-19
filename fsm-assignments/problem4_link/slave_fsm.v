module slave_fsm(
    input  wire       clk,
    input  wire       rst,   // synchronous reset
    input  wire       req,
    input  wire [7:0] data_in,
    output reg        ack,
    output reg [7:0]  last_byte
);

    reg [2:0] state;
    reg [1:0] hold_cnt;

    localparam IDLE   = 3'd0,
               LATCH  = 3'd1,
               HOLD   = 3'd2,
               DROP   = 3'd3;

    always @(posedge clk) begin
        if (rst) begin
            state     <= IDLE;
            ack       <= 0;
            last_byte <= 0;
            hold_cnt  <= 0;
        end else begin
            case(state)
                IDLE: begin
                    ack <= 0;
                    if (req) begin
                        last_byte <= data_in; // latch data
                        ack       <= 1;
                        hold_cnt  <= 2;
                        state     <= HOLD;
                    end
                end

                HOLD: begin
                    if (hold_cnt > 1) begin
                        hold_cnt <= hold_cnt - 1;
                    end else begin
                        state <= DROP;
                    end
                end

                DROP: begin
                    ack   <= 0;
                    if (!req) state <= IDLE;
                end
            endcase
        end
    end
endmodule
