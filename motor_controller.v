module motor_controller (
    input  wire        clk,
    input  wire        reset,
    input  wire        enable,

    input  wire [15:0] dis_l,
    input  wire [15:0] dis_f,
    input  wire [15:0] dis_r,
    
    input wire [31:0] LEFT_CYCLE,
    input wire [31:0] RIGHT_CYCLE,
    input wire [31:0] UTURN_CYCLE, 
    input wire [31:0] BEFORE_CYCLE,
    input wire [31:0] AFTER_CYCLE, 

    input  wire [31:0] count_A,
    input  wire [31:0] count_B,

    input  wire        ir_l,
    input  wire        ir_f,
    input  wire        ir_r,
    
    
    input wire         servo_stop,

    output wire        ENA,
    output wire        ENB,
    output reg         IN1,
    output reg         IN2,
    output reg         IN3,
    output reg         IN4,
    output reg         End,
    output reg         servo_req,
    output reg  [3:0]  mpi_count,
    output reg  [1:0]  turn_done ,
    
    output reg led1,led2,led3,led4,led5
);

////////////////////////////////////////parameters///////////////////////////////////
parameter  FORWARD = 4'd0;
parameter  LEFT    = 4'd1;
parameter  RIGHT   = 4'd2;
parameter  UTURN   = 4'd3;
parameter  SERVO   = 4'd4;
parameter  STOP    = 4'd5;
parameter  BFD     = 4'd6;
parameter  AFD     = 4'd7;
parameter  SBT     = 4'd8;
parameter  SAT     = 4'd9;

parameter BASE_SPEED_L = 35;
parameter BASE_SPEED_R = 35;
parameter TURN_SPEED_L = 25;
parameter TURN_SPEED_R = 25;
parameter L_MAX_SPEED  = 40;
parameter R_MAX_SPEED  = 40;
parameter L_MIN_SPEED  = 18;
parameter R_MIN_SPEED  = 14;
parameter DEADBAND     = 5;
parameter RAMP_STEP    = 1;

//turn 
parameter S = 3'd0 ,F = 3'd1 , L = 3'd2 , R = 3'd3 , U = 3'd4;


////counts 
//parameter C_BFD_L     = 1400;
//parameter C_BFD_R     = 1400;
//parameter C_AFD_L     = 1400;
//parameter C_AFD_R     = 1400;

// runtime configurable turns
wire [31:0] C_LEFT_L;
wire [31:0] C_LEFT_R;
wire [31:0] C_RIGHT_L;
wire [31:0] C_RIGHT_R;
wire [31:0] C_UTURN_L;
wire [31:0] C_UTURN_R;
wire [31:0] C_BFD_L;
wire [31:0] C_BFD_R;
wire [31:0] C_AFD_L;
wire [31:0] C_AFD_R;

assign C_LEFT_L  = LEFT_CYCLE;
assign C_LEFT_R  = LEFT_CYCLE;

assign C_BFD_L = BEFORE_CYCLE;
assign C_BFD_R = BEFORE_CYCLE;

assign C_AFD_L = AFTER_CYCLE;
assign C_AFD_R = AFTER_CYCLE;


assign C_RIGHT_L = RIGHT_CYCLE;
assign C_RIGHT_R = RIGHT_CYCLE;

assign C_UTURN_L = UTURN_CYCLE;
assign C_UTURN_R = UTURN_CYCLE;


parameter KP=9 ,KD = 6;

//delays 
parameter SBT_DELAY  = 7_000_000; //same delay for bot after stop and before stop

////////////////////////////////////////Asignments/////////////////////////////////

reg [5:0] duty_A,duty_B;
reg [5:0] target_A, target_B;
//wire [5:0] duty_test;
//assign duty_test = 20;
//pwm_generator pwmA(clk,duty_A,ENA);
//pwm_generator pwmB(clk,duty_B,ENB);
pwm_generator pwmA(clk,duty_A,ENA);
pwm_generator pwmB(clk,duty_B,ENB);


////distance assignment 
//reg [15:0] dis_l_f, dis_r_f;
//reg [15:0] dis_l_d1, dis_l_d2;
//reg [15:0] dis_r_d1, dis_r_d2;
//// FILTER
//always @(posedge clk) begin
//    dis_l_d2 <= dis_l_d1;
//    dis_l_d1 <= dis_l;
//    dis_l_f  <= (dis_l + dis_l_d1 + dis_l_d2) / 3;
//
//    dis_r_d2 <= dis_r_d1;
//    dis_r_d1 <= dis_r;
//    dis_r_f  <= (dis_r + dis_r_d1 + dis_r_d2) / 3;
//end


reg [3:0] state;
reg [31:0] delay_counter;
reg [31:0] ir_counter;
reg [2:0] turn ;
reg signed [31:0] count_l , count_r ;

//pid _ controllers 
//reg signed [4:0] error,prev_error;
//wire signed [5:0] derivative;
//wire signed [7:0] pid_out;


//encoders some logic + assignments ................................................
wire signed [31:0] diff_A;
wire signed [31:0] diff_B;
assign diff_A = count_A - count_l;
assign diff_B = count_B - count_r;
//making absolute distance .........................................................
//wire [31:0] dist_A = ((count_A - count_l)>= 0) ? count_A - count_l : count_l - count_A;
//wire [31:0] dist_B = ((count_B - count_r)>= 0) ? count_B - count_r : count_r - count_B;
wire [31:0] dist_B = (diff_B >= 0) ? diff_B : -diff_B;
wire [31:0] dist_A = (diff_A >= 0) ? diff_A : -diff_A;



////////////////////////////////////////signals & handshaking////////////
//-------------------------------------objects detection-----------------
reg ob_ll,ob_ff,ob_rr;
always @(posedge clk) begin
    ob_ll <= (dis_l< 180);
    ob_ff <= (dis_f< 90); 
    ob_rr <= (dis_r< 180);
end
// HYSTERESIS
//always @(posedge clk) begin
//    if(dis_l_f < 180) ob_ll <= 1;
//    else if(dis_l_f > 200) ob_ll <= 0;
//
//    ob_ff <= (dis_f<90);
//
//    if(dis_r_f < 180 ) ob_rr <= 1;
//    else if(dis_r_f > 200) ob_rr <= 0;
//end


//-------------------------------------------servo request---------------
always @(posedge clk or negedge reset) begin
    if (!reset)                 servo_req <= 1'b0;
     else if(!enable)           servo_req <= 1'b0;
    else if (state == SERVO)    servo_req <= 1'b1;      // request servo ONCE
    else                        servo_req <= 1'b0;
end


//-----------------------------------------------END signal--------------
always @(posedge clk or negedge reset) begin
    if (!reset)
        End <= 1'b0;
    else
        End <= (state == STOP);
end
///////////////////////////////////////signals end here ////////////////////

///////////////////////////////////////PID /////////////////////////////////
reg signed [15:0] error, prev_error;
wire signed [15:0] derivative;
wire signed [15:0] pid_out;

assign derivative = error - prev_error;
assign pid_out    = (KP * error) + (KD * derivative);

//wire signed [15:0] diff = dis_r_f - dis_l_f;
//wire signed [15:0] scaled_diff = diff >>> 4;
wire signed [15:0] pid_limited;

assign pid_limited = (pid_out > 25) ? 25 :
                     (pid_out < -25) ? -25 :
                     pid_out;


always @(posedge clk or negedge reset) begin
    if (!reset) begin
        error      <= 0;
        prev_error <= 0;
        ir_counter <= 0;
    end
    else if (!enable) begin
        error      <= 0;
        prev_error <= 0;
        ir_counter <= 0;
    end
    else begin
        prev_error <= error;

        if (state == FORWARD && (dis_l<180)&& (dis_r<180)) begin
            if (ir_l && !ir_r) begin
                ir_counter <= ir_counter + 1;

                if (ir_counter > 30_000_000)
                    error <= 3;
                else if (ir_counter > 20_000_000)
                    error <= 2;
                else
                    error <= 1;

            end 
            else if (!ir_l && ir_r) begin
                ir_counter <= ir_counter + 1;

                if (ir_counter > 50_000_000)
                    error <= -2;
                else if (ir_counter > 10_000_000)
                    error <= -2;
                else
                    error <= -1;

            end 
            else begin
                ir_counter <= 0;
                error <= 0;
            end
        end
        
        else if (state == FORWARD && (dis_l<230)) begin  // [FIX 2: Added 'else' here]
            if(ir_l) begin error <= 0; end
                else error <= -1;
        end
        
        
        else begin
                ir_counter <= 0;
                error  <= 0;
                prev_error <= 0;
        end 
    end
end


///////////////////////////////////////PID ENDS HERE ////////////////////////


//ramp
reg [15:0] ramp_cnt;

always @(posedge clk or negedge reset) begin
    if(!reset) begin
        duty_A   <= 0;
        duty_B   <= 0;
        ramp_cnt <= 0;
    end
    else if(!enable) begin
        duty_A   <= 0;
        duty_B   <= 0;
        ramp_cnt <= 0;
    end
    else begin
        ramp_cnt <= ramp_cnt + 1;
          
          if(state <= FORWARD)begin

              if(ramp_cnt == 20000) begin   // tune this
                    ramp_cnt <= 0;

                    if(duty_A < target_A) duty_A <= duty_A + RAMP_STEP;
                    else if(duty_A > target_A) duty_A <= duty_A - RAMP_STEP;

                    if(duty_B < target_B) duty_B <= duty_B + RAMP_STEP;
                    else if(duty_B > target_B) duty_B <= duty_B - RAMP_STEP;
              end
            end
        else begin
                duty_A   <= target_A;
                duty_B   <= target_B;
                ramp_cnt <= 0;
        end
    end
end



///////////////////////////////////////MAIN FSM//////////////////////////////

always @(posedge clk or negedge reset)begin
    if(!reset)begin
        state <= FORWARD ;
        delay_counter <= 0 ;
        count_l <= 0;
        count_r <= 0;
        target_A <= 0;
        target_B <= 0;
        turn_done <= 0;
        IN1 <= 0; IN2 <= 0;
        IN3 <= 0; IN4 <= 0;
        led1 <= 0; led2 <= 0; led3 <= 0; led4 <= 0 ; led5 <= 0;
    end
    else if (!enable)begin
        state <= FORWARD ;
        delay_counter <= 0 ;
        count_l <= 0;
        count_r <= 0;
        target_A <= 0;
        target_B <= 0;
        turn_done <= 0;
        IN1 <= 0; IN2 <= 0;
        IN3 <= 0; IN4 <= 0;
        led1 <= 0; led2 <= 0; led3 <= 0; led4 <= 0 ; led5 <= 0;
    end
    else begin
        led1 <= 0; led2 <= 0; led3 <= 0; led4 <= 0 ; led5 <= 0;
        case(state)
            FORWARD: begin
                led1 <= 1;
                turn_done <= 0;

                //pid
                if(BASE_SPEED_L + pid_limited > L_MAX_SPEED )begin
                    target_A <= L_MAX_SPEED;
                end
                else if(BASE_SPEED_L + pid_limited< L_MIN_SPEED)begin
                    target_A <= L_MIN_SPEED;
                end 
                else begin
                    target_A <= BASE_SPEED_L + pid_limited;
                end

                if(BASE_SPEED_R - pid_limited > R_MAX_SPEED )begin
                    target_B <= R_MAX_SPEED;
                end
                else if(BASE_SPEED_R - pid_limited< R_MIN_SPEED)begin
                    target_B <= R_MIN_SPEED;
                end 
                else begin
                    target_B <= BASE_SPEED_R - pid_limited;
                end
                //end


                IN1 <= 1; IN2 <= 0;
                IN3 <= 1; IN4 <= 0;
                    

                // DEAD END first
                    if(ir_f && ob_ll && ob_rr) begin
                         turn <= U;
                         count_l <= count_A;
                         count_r <= count_B;
                         state <= SERVO;
                    end

                    // LEFT open
                    else if(!ob_ll && !ir_l) begin
                         turn <= L;
                         count_l <= count_A;
                         count_r <= count_B;
                         state <= BFD;
                    end

                    // RIGHT open
                    else if(ob_ll && ob_ff && !ob_rr) begin
                         turn <= R;
                         count_l <= count_A;
                         count_r <= count_B;
                         state <= BFD;
                    end
                    else if(ob_ll && ob_ff && ob_rr) begin
                         turn <= U;
                         count_l <= count_A;
                         count_r <= count_B;
                         state <= BFD;
                    end

                    // OTHERWISE go forward
                    else begin
                         turn <= F;
                         state <= FORWARD;
                    end

           end

            
            BFD: begin
                    led2 <= 1;
                    turn_done <= 0;

                    IN1 <= 1; IN2 <= 0;
                    IN3 <= 1; IN4 <= 0;

                    if(dist_A >= C_BFD_L)
                        target_A <= 15;
                    else begin
                        target_A <= TURN_SPEED_L;
                    end

                    if(dist_B >= C_BFD_R)
                        target_B <= 15;
                    else begin
                        target_B <= TURN_SPEED_R;
                    end

                    if(((dist_A >= C_BFD_L && dist_B >= C_BFD_R)) || ob_ff )begin// ir_f
                        count_l <= count_A;
                        count_r <= count_B;
                        case(turn)
                            L: state <= LEFT ;
                            R: state <= RIGHT;
                            U: state <= UTURN;
                            default : state <= FORWARD;
                        endcase 
                        delay_counter <= 0;
                    end
                    else begin
                        case(turn)
                            L: if(ir_l) state <= FORWARD ;
                            R: if(ir_r) state <= FORWARD;
                            U: if(ir_f) state <= SERVO;
                        endcase 
                    end
                    
           end
            
            LEFT: begin
                 led3 <= 3;

                 IN1 <= 0; IN2 <= 1;   // left wheel reverse
                 IN3 <= 1; IN4 <= 0;   // right wheel forward

                 if (dist_A >= C_LEFT_L)
                      target_A <= 0;
                 else
                      target_A <= TURN_SPEED_L;

                 if (dist_B >= C_LEFT_R)
                      target_B <= 0;
                 else
                      target_B <= TURN_SPEED_R;

                 // ---- state decision ----
                 if ((dist_A >= C_LEFT_L) && (dist_B >= C_LEFT_R)) begin
                      count_l <= count_A;
                      count_r <= count_B;
                      turn_done <= 1;
                      state <= AFD;
                 end
            end

              
              RIGHT: begin
                    led3 <= 4;

                    IN1 <= 1; IN2 <= 0;   // left wheel forward
                    IN3 <= 0; IN4 <= 1;   // right wheel reversed

                    if(dist_A >= C_RIGHT_L)
                        target_A <= 0;
                    else
                        target_A <= TURN_SPEED_L;

                    if(dist_B >= C_RIGHT_R)
                        target_B <= 0;
                    else
                        target_B <= TURN_SPEED_R;
                        
                 if ((dist_A >= C_RIGHT_L) && (dist_B >= C_RIGHT_R)) begin
                      count_l <= count_A;
                      count_r <= count_B;
                      turn_done <= 2;//1->2
                      state <= AFD;
                 end
              end
              
              SERVO: begin
            delay_counter<= delay_counter + 1'b1;
                    if(!servo_stop) begin 
                         IN1<=0;IN2<=0;
                         IN3<=0;IN4<=0;
                        target_A <= 0;
                        target_B <= 0;
                    end
                    else begin 
                        state   <=UTURN; 
                        delay_counter<=0;
                    end
              end

              
              UTURN : begin
                    led3 <= 5;

                    IN1 <= 1; IN2 <= 0;   // left wheel reverse
                    IN3 <= 0; IN4 <= 1;   // right wheel forward

                    if(dist_A >= C_UTURN_L)
                        target_A <= 0;
                    else
                        target_A <= TURN_SPEED_L;

                    if(dist_B >= C_UTURN_R)
                        target_B <= 0;
                    else
                        target_B <= TURN_SPEED_R;

                    if((dist_A >= C_UTURN_L) && (dist_B >= C_UTURN_R)) begin
                        count_l <= count_A;
                        count_r <= count_B;
                        
                        turn_done <= 3;
                        state <= AFD;
                    end
              end 
              
              AFD: begin
                    led2 <= 1;

                    IN1 <= 1; IN2 <= 0;
                    IN3 <= 1; IN4 <= 0;

                    if(dist_A >= C_AFD_L)
                        target_A <= 14;
                    else begin
                        target_A <= TURN_SPEED_L;
                    end

                    if(dist_B >= C_AFD_R)
                        target_B <= 14;
                    else begin
                        target_B <= TURN_SPEED_R;
                    end

                    if((dist_A >= C_AFD_L) && (dist_B >= C_AFD_R)) begin//ir_f // ir_l
                        count_l <= count_A;
                        count_r <= count_B;
                        state   <= FORWARD;
                        
                    end
           end
            default: begin       //Added default case  
                    state<=FORWARD;
            end
            
            
        endcase
    end
end
// [FIX 1: Removed the extra 'end' that was here!]
endmodule