module uart_cmd_parser(
   input clk,
	input reset,
	input [7:0] rx_data,
	//input rx_valid,
	output bot_start
	);
	
	reg bot_start_reg;
	assign bot_start = bot_start_reg;
	always @(posedge clk or negedge reset) begin
	     if(!reset)begin
				bot_start_reg<= 0;
		  end
		  else if (rx_data == "S" ||rx_data == "T" ||rx_data == "A" ||rx_data == "R")begin
				bot_start_reg<= 1;
		  end
		  else if (rx_data == "E")begin
		      bot_start_reg<= 0;
		  end
   end
endmodule