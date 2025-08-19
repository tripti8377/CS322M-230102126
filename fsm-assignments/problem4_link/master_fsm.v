module master_fsm(
    input  wire       clk,
    input  wire       rst,   // synchronous reset
    input  wire       ack,
    output reg        req,
    output reg [7:0]  data,
    output reg        done   // 1-cycle when burst completes
);

    // Example data burst: A0..A3
    reg [7:0] burst [0:3];
    initial begin
        burst[0] = 8'hA0;
        burst[1] = 8'hA1;
        burst[2] = 8'hA2;
        burst[3] = 8'hA3;
    end

    reg [2:0] state;
    reg [1:0] index;

    localparam IDLE  = 3'd0,
               SEND  = 3'd1,
               WAIT_ACK = 3'd2,
               DROP_REQ = 3'd3,
               NEXT  = 3'd4,
               DONE  = 3'd5;

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            req   <= 0;
            data  <= 0;
            index <= 0;
            done  <= 0;
        end else begin
            done <= 0; // default
            case(state)
                IDLE: begin
                    index <= 0;
                    req   <= 0;
                    state <= SEND;
                end

                SEND: begin
                    data  <= burst[index];
                    req   <= 1;
                    state <= WAIT_ACK;
                end

                WAIT_ACK: begin
                    if (ack) state <= DROP_REQ;
                end

                DROP_REQ: begin
                    req   <= 0;
                    state <= NEXT;
                end

                NEXT: begin
                    if (index == 2'd3) begin
                        state <= DONE;
                    end else begin
                        index <= index + 1;
                        state <= SEND;
                    end
                end

                DONE: begin
                    done  <= 1;
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule
