module t2a_dht(
    input clk_50M,  // 1 clk is 20ns
    input reset,
    inout sensor,
    output reg [7:0] T_integral,
    output reg [7:0] RH_integral,
    output reg [7:0] T_decimal,
    output reg [7:0] RH_decimal,
    output reg [7:0] Checksum,
    output reg data_valid_dht
);

parameter   IDLE      = 3'b000 ,
            LOW_19MS  = 3'b001 , 
            REL_40US  = 3'b010 , 
            WF_LOW80  = 3'b011 ,
            WF_HIGH80 = 3'b100 ,
            WF_AG_LOW = 3'b101 ,
            DATA      = 3'b110 ,
            CHECK_SUM = 3'b111 ;

parameter   MS_19 = 1_000_000 /*20ms */ , REL_TIME = 2500 /* MA 2000, 50us */, THRESHOLD = 2000 /* changed from 2500 50us--->40us */;
parameter WAIT_1S = 50_000_000;
       


reg [2:0] state ;
reg sensor_out ;
//wire sensor_in; 
reg sensor_out_enable;
reg [31:0] counter ;
reg [31:0] timeout_counter ;
reg [39:0] data_shift ;
reg [5:0] bit_count;

assign sensor = sensor_out_enable ? sensor_out : 1'bz ;
//assign sensor_in = sensor ;
//synchronizer
reg s1, s2;
always @(posedge clk_50M) begin     
    s1 <= sensor;
    s2 <= s1;
end
wire sensor_sync = s2;
//Edge Detection
reg sensor_prev;
always @(posedge clk_50M)
    sensor_prev <= sensor_sync;

wire rising_edge  =  sensor_sync && !sensor_prev;
wire falling_edge = !sensor_sync &&  sensor_prev;
//chechsum
wire checksum_ok;
assign checksum_ok =
    (data_shift[39:32] +
     data_shift[31:24] +
     data_shift[23:16] +
     data_shift[15:8]) == data_shift[7:0];


always @(posedge clk_50M or negedge reset)begin
    if (!reset)begin
        sensor_out <= 0;
        state <= IDLE;
        counter <= 0;
        sensor_out_enable <= 0;
        data_valid_dht <= 0;
        bit_count <= 0;
		  data_shift <= 0;
		  timeout_counter <= 0;
    end
    else begin
        data_valid_dht <= 0;
        case(state)
            IDLE : begin
                state <= LOW_19MS;
                counter <= 0;
					 timeout_counter <= 0;
            end
            LOW_19MS : begin
				    sensor_out_enable <= 1'b1;
                sensor_out <= 1'b0;
					 counter  <= counter + 1'b1;
                if (counter == MS_19 )begin
                    state <= REL_40US;
                    counter <= 0;
                    sensor_out_enable <= 1; 
                end
            end

            REL_40US : begin
                sensor_out <= 1'b1;
					 counter  <= counter + 1'b1;
                if (counter == REL_TIME - 1)begin
                    state <= WF_LOW80;
                    sensor_out_enable <= 0; 
                    counter <= 0; 
                end
            end

            //wait for low_80 
            WF_LOW80 : begin
                if(!sensor_sync)begin
                    state <= WF_HIGH80;
                    sensor_out_enable <= 0; 
                    counter <= 0; 
                end
            end

            WF_HIGH80 : begin
                if (sensor_sync) begin
                    state <= WF_AG_LOW;
                    sensor_out_enable <= 0; 
                    counter <= 0; 
                end
            end

            WF_AG_LOW : begin
                if (!sensor_sync) begin
                    state <= DATA;
                    sensor_out_enable <= 0; 
                    counter <= 0; 
                    bit_count <= 0;
						  timeout_counter <= 0;
                end
            end

//            DATA : begin
//                if(sensor_sync)begin
//                    counter  <= counter + 1'b1;
//                end
//                else if(!sensor_sync && counter != 0) begin
//                    data_shift <= {data_shift[38:0], (counter >THRESHOLD)};
//                    bit_count <= bit_count + 1'b1;
//                    counter <= 0;
//                end
//                if(bit_count == 39)begin
//                    state <= CHECK_SUM ;
//                    counter <= 0;
//                    bit_count <= 0;
//                end
//            end
            //Edge based Detection
             DATA: begin
					 timeout_counter <=  timeout_counter + 1'b1;
                if (rising_edge)
                     counter <= 0;
                else if (sensor_sync)
                     counter <= counter + 1;
                else if (falling_edge) begin
                     data_shift <= {data_shift[38:0], (counter > THRESHOLD)};
                     bit_count <= bit_count + 1'b1;
                     counter <= 0;
                end
					 if(bit_count == 39)begin
						 state <= CHECK_SUM ;
						 counter <= 0;
						 timeout_counter <=  0;
					 end
					 else if (timeout_counter >= 230_000)begin
						 state <= CHECK_SUM ;
						 counter <= 0;
						 timeout_counter <=  0;
					 end
					
             end


            CHECK_SUM :begin
				    bit_count <= 0;
				    timeout_counter <= 0; 
				    counter  <= counter + 1'b1;//added counter
				    sensor_out_enable <= 1'b0;
                RH_integral <= data_shift[39:32];
                RH_decimal  <= data_shift [31:24];
                T_integral  <= data_shift [23:16];
                T_decimal   <= data_shift [15:8];
                Checksum    <= data_shift [7:0];
					 
					 if(counter >= WAIT_1S) begin
					 state<= IDLE;
					 counter<=0;
                end
                if ((data_shift[39:32] + data_shift [31:24] + data_shift [23:16]+ data_shift [15:8]) == data_shift [7:0])begin
                   // data_valid_dht <= 1;
						 data_valid_dht <= (checksum_ok && counter ==1);

                end
				
            end
        endcase
    end
end
        
endmodule