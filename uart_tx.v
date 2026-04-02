module uart_tx(
    input clk_3125,
    input reset ,
    input  tx_start,
    input [7:0] data,
    output reg tx,
    output reg tx_done
);

parameter IDLE = 2'b00 , START = 2'b01 , DATA = 2'b10 , STOP = 2'b11;
parameter CLK_PER_BIT = 27;

reg [1:0] state;
reg [7:0] data_shift;
reg [2:0] bit_idx; 
reg [4:0] per_bit_clk; //for now baud rate 115200

always @(posedge clk_3125 or negedge reset)begin
    if (!reset)begin
        tx <= 1;
        tx_done <= 0;
        data_shift <= 0;
        state <= IDLE;
        bit_idx <= 0;
        per_bit_clk <= 0;
    end
    else begin
        tx_done <= 0;
        case(state)
            IDLE : begin
                tx <= 1;
                bit_idx <= 0; 
                per_bit_clk <= 0;
                if (tx_start)begin
                    data_shift <= data ;
                    state <= START;
                end
            end

            START : begin
                tx <= 0;
                per_bit_clk <= per_bit_clk + 1'b1 ;
                if (per_bit_clk == CLK_PER_BIT - 1)begin
                    per_bit_clk <= 0;
                    state <= DATA;
                    bit_idx <= 0;
                end
            end

            DATA : begin
                per_bit_clk <= per_bit_clk + 1'b1;
                tx <= data_shift[bit_idx];
                if (per_bit_clk == CLK_PER_BIT - 1)begin
                    if(bit_idx == 7)begin
                        state <= STOP;
                        per_bit_clk <= 0; 
                    end
                    else begin
                        bit_idx <= bit_idx + 1'b1;
                        per_bit_clk <= 0;
                    end
                end

            end

            STOP : begin
                tx <= 1'b1;
                per_bit_clk <= per_bit_clk + 1'b1;
                if (per_bit_clk == CLK_PER_BIT - 1)begin
                    state <= IDLE;
                    tx_done <= 1;
                    per_bit_clk <= 0; 
                end
            end
        endcase
    end
end

endmodule