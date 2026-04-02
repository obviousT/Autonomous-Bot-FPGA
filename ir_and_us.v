module ir_and_us(
    input ir_in_left,
    input ir_in_front,
	 input ir_in_right,
    input echo_rx_l,
    input echo_rx_f,
	 input echo_rx_r,
    input clk ,
    input reset,
    output trig_l,
	 output trig_f,
	 output trig_r, 
    output   [15:0] dis_l,
	 output   [15:0] dis_f,
	 output   [15:0] dis_r,
	 output ir_l,
	 output ir_f,
	 output ir_r,
	 output test_mpi
);


parameter STOP=3'b000, FORWARD = 3'b001, RIGHT = 3'b010, LEFT = 3'b011 , UTURN = 3'b100;
wire ob_l, ob_r, ob_f;

wire ob_us_l, ob_us_r, ob_us_f;

ir_sensor irr_l(ir_in_left, clk , reset, ob_l);
ir_sensor irr_r(ir_in_right, clk , reset, ob_r);
ir_sensor irr_f(ir_in_front, clk , reset, ob_f);

us_sensor us_l(clk, reset , echo_rx_l, trig_l,  dis_l);
us_sensor us_r(clk, reset , echo_rx_r, trig_r,  dis_r);
us_sensor us_f(clk, reset , echo_rx_f, trig_f,  dis_f);

assign ir_l = ob_l;
assign ir_r = ob_r;
assign ir_f = ob_f;
assign test_mpi = ob_l && ob_r && ob_f;

endmodule