
module us_sensor(
    input clk_50M,
    input reset,
    input echo_rx,              // async echo from ultrasonic sensor
    output reg trig,             // trigger pulse
    output reg [15:0] distance_out
);

reg [1:0] state;
reg [31:0] counter;             // measures echo pulse width
reg [31:0] echo_counter;        // latched echo width
reg [31:0] delay_counter;       // generic delay counter

// ---------------- Echo synchronizer ----------------
reg echo_ff1, echo_ff2;
wire echo_sync;

assign echo_sync = echo_ff2;

always @(posedge clk_50M or negedge reset) begin
    if (!reset) begin
        echo_ff1 <= 1'b0;
        echo_ff2 <= 1'b0;
    end else begin
        echo_ff1 <= echo_rx;
        echo_ff2 <= echo_ff1;
    end
end

// ---------------- FSM states ----------------
parameter s0 = 2'b00,   // trigger start
          s1 = 2'b01,   // trigger high (10 us)
          s2 = 2'b10,   // wait for echo
          s3 = 2'b11;   // compute distance + delay

// ---------------- Timing parameters ----------------
parameter delay_10_us = 550;
parameter delay_20_ms = 5_000_000;
parameter TIMEOUT     = 25_000_000;
parameter MAX_VALUE   = 25_000_000;

// ---------------- FSM ----------------
always @(posedge clk_50M or negedge reset) begin
    if(!reset) begin // low reset 
        state <= s0;
        trig <= 0;
        counter <= 0;
        echo_counter <= 0;
        delay_counter <= 0;
        distance_out <= 0;
    end 
    else begin
        case(state)

            // Start trigger pulse
            s0: begin
                state <= s1;
                counter <= 0;
                trig <= 1;
                echo_counter <= 0;
                delay_counter <= 0;
            end

            // Hold trigger high for 10 us
            s1: begin
                counter <= counter + 1;
                if(counter >= delay_10_us) begin
                    trig <= 0;
                    counter <= 0;
                    state <= s2;
                    delay_counter <= 0;
                    echo_counter <= 0;
                end
            end

            // Measure echo pulse width
            s2: begin
                delay_counter <= delay_counter + 1;

                if (delay_counter >= TIMEOUT) begin
                    counter <= 0;
                    delay_counter <= 0;
                    state <= s3;
                    echo_counter <= MAX_VALUE;   // timeout case
                end
                else if (echo_sync) begin
                    counter <= counter + 1;
                end
                else if (!echo_sync && counter != 0) begin
                    echo_counter <= counter;
                    counter <= 0;
                    delay_counter <= 0;
                    state <= s3;
                end
            end

            // Compute distance and wait before next cycle
            s3: begin
                delay_counter <= delay_counter + 1;
                if (echo_counter < MAX_VALUE)
                    distance_out <= (echo_counter * 3565) >> 20;

                if (delay_counter >= delay_20_ms) begin
                    state <= s0;
                    counter <= 0;
                    delay_counter <= 0;
                end
            end

        endcase
    end
end

endmodule
