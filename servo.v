 module servo (
    input  wire clk_50M,
    input  wire reset,      // ACTIVE HIGH
    output wire SERVO_1,
    output wire SERVO_2,
    input  wire mpi_detected,   // servo_start from motor
    output reg  servo_stop,
	 output reg  send
);

    localparam [19:0] PULSE_90   = 20'd100_000;
    localparam [19:0] PULSE_N_90 = 20'd50_000;
    localparam [19:0] PULSE_0    = 20'd75_000;

    localparam [25:0] WAIT_1S = 26'd20_000_000;
    localparam [25:0] WAIT_4S = 26'd400_000_000;

    localparam S_IDLE    = 3'd0;
    localparam S_MOVE_S2 = 3'd1; 
    localparam S_MOVE_S1 = 3'd2; 
    localparam S_RET_S1  = 3'd3; 
    localparam S_RET_S2  = 3'd4; 

    reg [2:0]  state;
    reg [25:0] timer;
    reg [19:0] pw1, pw2;

    servo_pwm s1 (.clk_50M(clk_50M), .pulse_width(pw1), .servo_out(SERVO_1));
    servo_pwm s2 (.clk_50M(clk_50M), .pulse_width(pw2), .servo_out(SERVO_2));

    /* handshake: DONE pulse */
    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            servo_stop <= 1'b0;
        else if (state == S_RET_S1)
            servo_stop <= 1;
		  else 
		      servo_stop <= 0;
    end
	 //send logic
	 always @(posedge clk_50M or negedge reset) begin
    if (!reset)
        send <= 1'b0;
    else
        send <= (state == S_RET_S1);// && timer == WAIT_1S);
    end


    always @(posedge clk_50M) begin
        if (!reset) begin
            state <= S_IDLE;
            timer <= 0;
            pw1   <= PULSE_0;
            pw2   <= PULSE_N_90;
        end 
        else if (!mpi_detected) begin
            state <= S_IDLE;          // wait for new start
            timer <= 0;
            pw1   <= PULSE_0;
            pw2   <= PULSE_N_90;
        end 
        else begin
            case (state)

                S_IDLE: begin
                    pw1 <= PULSE_0;
                    pw2 <= PULSE_N_90;
                    if (timer >= WAIT_1S) begin
                        timer <= 0;
                        state <= S_MOVE_S2;
                    end else timer <= timer + 1'b1;
                end

                S_MOVE_S2: begin
                    pw1 <= PULSE_0;
                    pw2 <= PULSE_90;
                    if (timer >= WAIT_1S) begin
                        timer <= 0;
                        state <= S_MOVE_S1;
                    end else timer <= timer + 1'b1;
                end

                S_MOVE_S1: begin
                    pw1 <= PULSE_N_90;
                    pw2 <= PULSE_90;
                    if (timer >= WAIT_4S) begin
                        timer <= 0;
                        state <= S_RET_S1;
                    end else timer <= timer + 1'b1;
                end

                S_RET_S1: begin
//                    pw1 <= PULSE_0;
//                    pw2 <= PULSE_90;
//                    if (timer >= WAIT_1S) begin
//                        timer <= 0;
//                        state <= S_IDLE;
//								
//                    end else timer <= timer + 1'b1;
                if (timer >= 500) begin
                        timer <= 0;
                        state <= S_IDLE;
								
                    end else timer <= timer + 1'b1;
                    
						  
                end
//
//                S_RET_S2: begin
//                    pw1 <= PULSE_0;
//                    pw2 <= PULSE_N_90;
//                    if (timer >= WAIT_1S) begin
//                        timer <= 0;
//                        state <= S_IDLE;
//                    end else timer <= timer + 1'b1;
//                end

            endcase
        end
    end
endmodule

module servo_pwm (
    input  wire clk_50M,
    input  wire reset,          // ACTIVE HIGH
    input  wire [19:0] pulse_width,
    output reg  servo_out
);

    localparam [19:0] PERIOD = 20'd1_000_000; // 20 ms
    reg [19:0] counter;

    always @(posedge clk_50M) begin
        
            if (counter == PERIOD - 1)
                counter <= 0;
            else
                counter <= counter + 1;
   
   
        servo_out <= (counter < pulse_width);
    end

endmodule