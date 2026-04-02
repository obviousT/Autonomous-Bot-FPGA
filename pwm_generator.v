module pwm_generator(
    input clk,
    input [5:0] duty_cycle,
    output reg pwm_signal   
);

//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////
reg [5:0] pwm_counter = 0;

always @(posedge clk) begin
    if (pwm_counter == 63)
        pwm_counter <= 0;
    else
        pwm_counter <= pwm_counter + 1'b1;

    // --- PWM signal ---
    pwm_signal <= (pwm_counter < duty_cycle);
end
//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////

endmodule