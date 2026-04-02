module robot_motor_top (
		//universal 
		input  wire clk,
		input  wire reset,     // ACTIVE LOW

		// Encoder inputs
		input  wire ENA_1,
		input  wire ENB_1,
		input  wire ENA_2,
		input  wire ENB_2,

		//ir input
	   input  wire ir_l,
		input  wire ir_r,
		input  wire ir_f,
		
		//rx & tx
		output  tx,
		input rx,
		
		//soilmoisture
	   input  dout,
		output adc_cs_n,din, 
		output adc_sck,

		//ultrasonic
		input wire  ec_l,
		input wire  ec_r,
		input wire  ec_f,
		output wire trig_l,
		output wire trig_f,
		output wire trig_r,
		
		//dht_11 
		inout wire dht_inout,
		
		//servo
		output wire SERVO_1,
		output wire SERVO_2,
	 
		// Motor driver outputs
		output wire ENA,
		output wire ENB,
		output wire IN1,
		output wire IN2,
		output wire IN3,
		output wire IN4,
		
		output wire led1,led2, led3, led4, led5
		

);
	 //soilmoiture
    wire [11:0] d_out_ch0;
	 
	 //tx
	 wire [7:0] tx_data;
	 wire tx_done;
	 wire tx_start;
	 wire End;
	 //rx
    wire [7:0] rx_data;
    wire       rx_valid;

    wire signed [31:0] count_A, count_B;
	 wire [15:0] dis_l, dis_f, dis_r;
	 wire irw_l, irw_f, irw_r;
	 
	 
	 // dht 11 
	 wire [7:0] T_int;
	 wire [7:0] T_dec;
	 wire [7:0] H_int;
	 wire [7:0] H_dec;
	 wire [7:0] C_sum;
	 wire [7:0] Dht_valid;
	 
	 //clk_generator 
	 wire clk_3125k;
	 
	 //servo
	 wire servo_start;
	 wire servo_stop;
	 
	 //start bot
	 wire start_bot;
	 //assign start_bot=1; //bypass BLE
	 
	 //send message
	 wire send_msg;
	 wire [3:0] mpi_count;
	 
	 //motorcontroller 
	 wire [1:0] turn_done;
	 wire [3:0] x,y ;
	 wire [2:0] dir;
	 wire update_done;
	 
	 // ---------------- 3125k clk generator ----------------
	 clk3125 u_clk_div (
        .clk_50M   (clk),
        .reset     (reset),
        .clk_3125k (clk_3125k)
    );
	 
	 

    // ---------------- Encoder blocks ----------------
    encoder_decoder encA (
        .clk   (clk),
        .reset (reset),
        .enc_A (ENA_1),
        .enc_B (ENB_1),
        .count (count_A)
    );

    encoder_decoder encB (
        .clk   (clk),
        .reset (reset),
        .enc_A (ENB_2),
        .enc_B (ENA_2),
        .count (count_B)
    );
	 
	 

   
  

    // ---------------- Motor controller ----------------
    motor_controller motor_ctrl (
				.clk     (clk),
				.reset   (reset),
				.count_A (count_A),
				.count_B (count_B),
				.enable  (1), //added enable
				.dis_l   (dis_l),
				.dis_r   (dis_r),
				.dis_f   (dis_f),
				.ir_l    (irw_l),
				.ir_f    (irw_f),
				.ir_r    (irw_r),
				.ENA     (ENA),
				.ENB     (ENB),
				.IN1     (IN1),
				.IN2     (IN2),
				.IN3     (IN3),
				.IN4     (IN4),
				.led1(led1), .led2(led2), .led3(led3), .led4(led4), .led5(led5),     
				.servo_req(servo_start),
				.servo_stop (servo_stop),
				.End     (End),
				.mpi_count(mpi_count),
				.turn_done (turn_done)
		);
	 
	 
	 // ----------------integrated ir and us ----------------
	 ir_and_us sensors(
				.ir_in_left		(ir_l),
				.ir_in_front	(ir_f),
				.ir_in_right	(ir_r),
				.echo_rx_l		(ec_l),
				.echo_rx_f		(ec_f),
				.echo_rx_r		(ec_r),
				.clk 				(clk),
				.reset			(reset),
				.trig_l			(trig_l),
				.trig_f			(trig_f),
				.trig_r			(trig_r), 
				.dis_l			(dis_l),
				.dis_f			(dis_f),
				.dis_r			(dis_r),
				.ir_l    (irw_l),
				.ir_f    (irw_f),
				.ir_r    (irw_r),
				.test_mpi (mpi_detected)
			
		);
		
		// ---------------- dht11 sensor  ----------------
		 t2a_dht u_dht(
				.clk_50M    (clk),
				.reset      (reset),
				.sensor     (dht_inout),
			   .T_integral (T_int),
				.RH_integral(H_int),
				.T_decimal  (T_dec),
				.RH_decimal (H_dec),
				.Checksum   (C_sum),
				.data_valid_dht (dht_valid)
		);
		
		// ---------------- soil_moisture sensor  ----------------
		moisture_sensor   ms(
            .dout       (dout), 
            .clk50      (clk),
            .adc_cs_n   (adc_cs_n), 
            .din        (din), 
            .adc_sck    (adc_sck),
            .d_out_ch0  (d_out_ch0)  
		);
		
		// ---------------- uart_tx  ----------------
		 uart_tx     uart(
				.clk_3125     (clk_3125k),
				.reset        (reset),
				.tx_start     (tx_start),
				.data         (tx_data),
				.tx	        (tx),
				.tx_done      (tx_done),
				
		  ); 
		  
		  // ---------------- msg_sender  ----------------
		  msg_sender msg_fsm (
				  .clk_3125       (clk_3125k),
				  .reset          (reset),
				  .mpi_detected   (send_msg),
				  .T_integral     (T_int),
				  .T_decimal      (T_dec),
				  .RH_integral    (H_int),
				  .RH_decimal     (H_dec),
				  .d_out_ch0      (d_out_ch0),
				  .tx_data        (tx_data),
				  .tx_start       (tx_start),
				  .tx_done        (tx_done),
				  .End            (End),
				  .mpi_count      (mpi_count),
				  .x_coordinate  (x),
				  .y_coordinate  (y),
				  .dir            (dir),
				  .update_done (update_done)
			 );

			// ----------------servo ----------------
			 servo u_servo (
				 .clk_50M      (clk),
				 .reset        (reset),
				 .SERVO_1      (SERVO_1),
				 .SERVO_2      (SERVO_2),
				 .mpi_detected (servo_start & start_bot),  //added & start_bot
				 .servo_stop   (servo_stop),
				 .send         (send_msg)
			 );
			 
			// ----------------uart_rx ----------------
			 uart_rx u_uart_rx (
				  .clk      (clk),
				  .reset_n  (reset),
				  .rx       (rx),
				  .rx_data  (rx_data)
				  //.rx_valid (rx_valid)
			 );
			 
			 // --------------- uart cmd passer ----------------
			 uart_cmd_parser u_uart_cmd (
              .clk       (clk),
              .reset     (reset),
              .rx_data   (rx_data),
              //.rx_valid  (rx_valid),
              .bot_start (start_bot) //start_bot
          );
			 
			 
			 //----------------position detectoronly two test ----------------
			 bot_position u_bot_position (
				 .clk           (clk),
				 .reset         (reset),

				 .count_L       (count_A),
				 .count_R       (count_B),

				 .turn_done     (turn_done),

				 .x             (x),
				 .y             (y),
				 .dir           (dir),
				 .update_done (update_done)
				 //.reached_exit  (reached_exit)
			);


			 
endmodule
