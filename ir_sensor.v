module ir_sensor (
    input ir_in , 
    input clk ,
    input reset ,
    output reg obstacle 
);

reg ir_sync1, ir_sync2;

    // Synchronizer (VERY IMPORTANT)
    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            ir_sync1 <= 1'b0;
            ir_sync2 <= 1'b0;
        end else begin
            ir_sync1 <= ir_in;
            ir_sync2 <= ir_sync1;
        end
    end

    // Obstacle detection logic
    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            obstacle <= 1'b0;
        end else begin
            if (ir_sync2 == 1'b0)
					 obstacle <= 1'b1;   // obstacle detected
            else
                obstacle <= 1'b0;   // no obstacle
        end
    end

endmodule