module clk3125(
    input  wire clk_50M,
    input  wire reset,
    output reg  clk_3125k
);

    reg [3:0] cnt;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            cnt <= 4'd0;
            clk_3125k <= 1'b0;
        end else begin
            if (cnt == 4'd7) begin
                cnt <= 4'd0;
                clk_3125k <= ~clk_3125k;
            end else begin
                cnt <= cnt + 1'b1;
            end
        end
    end

endmodule
