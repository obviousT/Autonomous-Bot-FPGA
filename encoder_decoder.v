module encoder_decoder (
    input  wire clk,
    input  wire reset,
    input  wire enc_A,
    input  wire enc_B,
    output reg signed [31:0] count,
	 output reg [1:0] dir
	   
);

    reg A1, A2, B1, B2;
    reg [1:0] prev;
    wire [1:0] curr;



    // Synchronizers
    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            A1 <= 1'b0; 
            A2 <= 1'b0;
            B1 <= 1'b0; 
            B2 <= 1'b0;
        end else begin
            A1 <= enc_A;
            A2 <= A1;
            B1 <= enc_B;
            B2 <= B1;
        end
    end


    assign curr = {A2,B2};
    // Decode
    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            prev  <= 2'b00;
            count <= 0;
        end else begin
            case ({prev, curr})
                4'b00_01,
					 4'b01_11,
					 4'b11_10,
					 4'b10_00: begin count <= count + 1; dir <= 1;end
					 
                4'b00_10,
					 4'b10_11,
					 4'b11_01,
					 4'b01_00: begin count <= count - 1; dir <= 0;end
					 
                default: count <= count;
            endcase

            prev <= curr;
        end
    end
endmodule