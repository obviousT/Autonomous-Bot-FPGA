module uart_rx (
    input  wire       clk,        // 50 MHz
    input  wire       reset_n,    // active low
    input  wire       rx,
    output reg [7:0]  rx_data
   // output reg        rx_valid
);

    parameter BAUD_CNT = 434;

    localparam IDLE  = 2'd0;
    localparam START = 2'd1;
    localparam DATA  = 2'd2;
    localparam STOP  = 2'd3;

    reg [1:0] state;
    reg [8:0] baud_cnt;
    reg [2:0] bit_idx;
    reg [7:0] shift_reg;

    //////////////////////////////////////////////////////
    // RX INPUT SYNCHRONIZER (ONLY ADDITION)
    //////////////////////////////////////////////////////
    reg rx_ff1, rx_ff2,rx_ff3;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rx_ff1 <= 1'b1;
            rx_ff2 <= 1'b1;
				rx_ff3 <= 1'b1;
        end else begin
            rx_ff1 <= rx;
				rx_ff3 <= rx_ff1;
            rx_ff2 <= rx_ff3;
        end
    end


    //////////////////////////////////////////////////////
    // UART RX FSM
    //////////////////////////////////////////////////////
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state     <= IDLE;
            baud_cnt  <= 0;
            bit_idx   <= 0;
            shift_reg <= 0;
            rx_data   <= 0;
           // rx_valid  <= 0;
        end else begin
            //rx_valid <= 1'b0;

            case (state)

            IDLE: begin
                if (rx_ff2 == 1'b0) begin
                    baud_cnt <= BAUD_CNT >> 1;
                    state    <= START;
                end
            end

            START: begin
                if (baud_cnt == 0) begin
                    if (rx_ff2 == 1'b0) begin
                        baud_cnt <= BAUD_CNT - 1'b1;
                        bit_idx  <= 0;
                        state    <= DATA;
                    end else
                        state <= IDLE;
                end else
                    baud_cnt <= baud_cnt - 1'b1;
            end

            DATA: begin
                if (baud_cnt == 0) begin
                    shift_reg[bit_idx] <= rx_ff2;
                    baud_cnt <= BAUD_CNT - 1'b1;

                    if (bit_idx == 3'd7)
                        state <= STOP;
                    else
                        bit_idx <= bit_idx + 1'b1;
                end else
                    baud_cnt <= baud_cnt - 1'b1;
            end

            STOP: begin
                if (baud_cnt == 0) begin
                    if (rx_ff2 == 1'b1) begin
                        rx_data  <= shift_reg;
                        //rx_valid <= 1'b1;
                    end
                    state <= IDLE;
                end else
                    baud_cnt <= baud_cnt - 1'b1;
            end

            endcase
        end
    end

endmodule
