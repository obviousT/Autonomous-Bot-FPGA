module bot_position(
    input clk,
    input reset,

    input [31:0] count_L,
    input [31:0] count_R,

    input [1:0] turn_done,

    output reg [3:0] x,
    output reg [3:0] y,
    output reg [1:0] dir,
    output reg update_done
);

parameter NORTH = 2'b00;
parameter EAST  = 2'b01;
parameter SOUTH = 2'b10;
parameter WEST  = 2'b11;

parameter ENCODER_CPR = 360;
parameter COUNTS_PER_BLOCK = 2400;

reg [31:0] prev_L;
reg [31:0] prev_R;
reg [31:0] accum;

reg [1:0] turn_prev;
wire turn_event;

wire [31:0] dL;
wire [31:0] dR;
wire [31:0] avg_d;

assign dL = (count_L > prev_L) ? count_L - prev_L : prev_L - count_L;
assign dR = (count_R > prev_R) ? count_R - prev_R : prev_R - count_R;
assign avg_d = (dL + dR) >> 1;

assign turn_event = (turn_done != 0) && (turn_prev == 0);

// Add these parameters at the top of your module
parameter MAX_X = 4'd8; // Set this to your maze width - 1
parameter MAX_Y = 4'd8; // Set this to your maze height - 1

// ... (Rest of your module setup stays the same) ...

always @(posedge clk or negedge reset)
begin
    if(!reset)
    begin
        x <= 0;
        y <= 4;
        dir <= NORTH;
        update_done <= 0;

        prev_L <= 0;
        prev_R <= 0;
        accum <= 0;

        turn_prev <= 0;
    end
    else
    begin
        update_done <= 0;

        prev_L <= count_L;
        prev_R <= count_R;

        turn_prev <= turn_done;

        case(turn_done)

        // --- STRAIGHT DRIVING (Translation) ---
        // This is the ONLY place X and Y should ever change.
        2'b00: begin
            accum <= accum + avg_d;

            if(accum >= COUNTS_PER_BLOCK)
            begin
                update_done <= 1;

                // [FIX] Added boundary protections (Saturating Counters)
                case(dir)
                NORTH: y <= (y < MAX_Y) ? y + 1 : MAX_Y;
                SOUTH: y <= (y > 0)     ? y - 1 : 0;
                EAST : x <= (x < MAX_X) ? x + 1 : MAX_X;
                WEST : x <= (x > 0)     ? x - 1 : 0;
                endcase

                accum <= 0;
            end
        end

        // --- LEFT TURN (Rotation Only) ---
        2'b01: if(turn_event) begin
            update_done <= 1;
 
            // [FIX] Removed X/Y changes. Only update direction.
            case(dir)
            NORTH: dir <= WEST;
            WEST : dir <= SOUTH;
            SOUTH: dir <= EAST;
            EAST : dir <= NORTH;
            endcase

            accum <= 0;
        end

        // --- RIGHT TURN (Rotation Only) ---
        2'b10: if(turn_event) begin
            update_done <= 1;

            // [FIX] Removed X/Y changes. Only update direction.
            case(dir)
            NORTH: dir <= EAST;
            EAST : dir <= SOUTH;
            SOUTH: dir <= WEST;
            WEST : dir <= NORTH;
            endcase

            accum <= 0;
        end

        // --- U-TURN (Rotation Only) ---
        2'b11: if(turn_event) begin
            update_done <= 1;

            // [FIX] Removed X/Y changes. Only update direction.
            case(dir)
            NORTH: dir <= SOUTH;
            SOUTH: dir <= NORTH;
            EAST : dir <= WEST;
            WEST : dir <= EAST;
            endcase

            accum <= 0;
        end

        endcase
    end
end
   
endmodule
