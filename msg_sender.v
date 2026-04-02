module msg_sender (
    input  wire clk_3125,
    input  wire reset,
    input  wire End,
    input  wire [3:0] mpi_count,

    input  wire mpi_detected,
    input  wire [7:0] T_integral,
	 input  wire [7:0] T_decimal,
	 input  wire [7:0] RH_decimal,
    input  wire [7:0] RH_integral,

    input  wire [11:0] d_out_ch0,

    input  wire [3:0] x_coordinate,
    input  wire [3:0] y_coordinate,
    input  wire [1:0] dir,
    input  wire update_done,

    output reg  [7:0] tx_data,
    output reg  tx_start,
    input  wire tx_done
);

//---------------- states ----------------
parameter IDLE     = 3'b000;
parameter SEND_MPI = 3'b001;
parameter SEND_MM  = 3'b010;
parameter SEND_TH  = 3'b011;
parameter SEND_END = 3'b100;
parameter SEND_POS = 3'b101;

parameter MOIST_THRESHOLD = 12'd1050;

//---------------- registers ----------------
reg [2:0] state;
reg [4:0] idx;
reg fired;
reg gap;

//---------------- coordinate ascii ----------------
// If coordinate is 0-9, add 48 to get '0'-'9'. 
// If coordinate is 10-15, subtract 10 and add 65 to get 'A'-'F'.
//---------------- coordinate ascii ----------------
// Since the bot_position module now guarantees values from 0 to 8,
// we just add 48 to convert the number directly to an ASCII character.
wire [7:0] x_ascii = x_coordinate + 8'd48;
wire [7:0] y_ascii = y_coordinate + 8'd48;
wire [7:0] dir_ascii= dir + 8'd48;

//---------------- update_done sync ----------------
reg u1,u2,u_prev;

always @(posedge clk_3125 or negedge reset) begin
    if(!reset) begin
        u1<=0; u2<=0; u_prev<=0;
    end else begin
        u1 <= update_done;
        u2 <= u1;
        u_prev <= u2;
    end
end

wire pos_change = u2 & ~u_prev;

//---------------- mpi sync ----------------
reg m1,m2,m_prev;

always @(posedge clk_3125 or negedge reset) begin
    if(!reset) begin
        m1<=0; m2<=0; m_prev<=0;
    end else begin
        m1 <= mpi_detected;
        m2 <= m1;
        m_prev <= m2;
    end
end

wire mpi_rise = m2 & ~m_prev;

//---------------- end sync ----------------
reg e1,e2,e_prev;

always @(posedge clk_3125 or negedge reset) begin
    if(!reset) begin
        e1<=0; e2<=0; e_prev<=0;
    end else begin
        e1 <= End;
        e2 <= e1;
        e_prev <= e2;
    end
end

wire end_rise = e2 & ~e_prev;

//---------------- mpi count sync ----------------
reg [3:0] gm1,gm2;

always @(posedge clk_3125 or negedge reset) begin
    if(!reset) begin
        gm1<=0; gm2<=0;
    end else begin
        gm1 <= mpi_count;
        gm2 <= gm1;
    end
end

//---------------- value limiting ----------------
wire [7:0] T_lim = (T_integral>20)?8'd20:((T_integral<8)?8'd8:T_integral);
wire [7:0] H_lim = (RH_integral>100)?8'd100:((RH_integral<30)?8'd30:RH_integral);

wire [7:0] T_tens = ((T_lim/10)>9?9:(T_lim/10)) + 8'd48;
wire [7:0] T_ones = ((T_lim%10)>9?9:(T_lim%10)) + 8'd48;

wire [7:0] H_tens = ((H_lim/10)>9?9:(H_lim/10)) + 8'd48;
wire [7:0] H_ones = ((H_lim%10)>9?9:(H_lim%10)) + 8'd48;

wire [7:0] MPI_ones = ((gm2%10)>9?9:(gm2%10)) + 8'd48;

wire soil_moist = (d_out_ch0 < MOIST_THRESHOLD);

//---------------- FSM ----------------
always @(posedge clk_3125 or negedge reset) begin
if(!reset) begin
    state<=IDLE;
    idx<=0;
    tx_start<=0;
    tx_data<=0;
    fired<=0;
    gap<=0;
end
else begin

tx_start<=0;

case(state)

//---------------- IDLE ----------------
IDLE: begin
    if(mpi_rise) begin
        state<=SEND_MPI;
        idx<=1;
    end
    else if(pos_change) begin
        state<=SEND_POS;
        idx<=1;
    end
    else if(end_rise) begin
        state<=SEND_END;
        idx<=1;
    end
    fired<=0;
    gap<=0;
end

//---------------- MPI ----------------
SEND_MPI: begin
    case(idx)
        1: tx_data<="M";
        2: tx_data<="P";
        3: tx_data<="I";
        4: tx_data<="M";
        5: tx_data<="-";
        6: tx_data<=MPI_ones;
        7: tx_data<="-";
        8: tx_data<="#";
    endcase

    if(tx_done) begin
        fired<=0; gap<=1;
        if(idx==8) begin
            state<=SEND_MM;
            idx<=1;
        end else idx<=idx+1;
    end
    else if(gap) gap<=0;
    else if(!fired) begin
        tx_start<=1;
        fired<=1;
    end
end

//---------------- SOIL ----------------
SEND_MM: begin
    case(idx)
        1: tx_data<="M";
        2: tx_data<="M";
        3: tx_data<="-";
        4: tx_data<=MPI_ones;
        5: tx_data<="-";
        6: tx_data<=(soil_moist?"M":"D");
        7: tx_data<="-";
        8: tx_data<="#";
    endcase

    if(tx_done) begin
        fired<=0; gap<=1;
        if(idx==8) begin
            state<=SEND_TH;
            idx<=1;
        end else idx<=idx+1;
    end
    else if(gap) gap<=0;
    else if(!fired) begin
        tx_start<=1;
        fired<=1;
    end
end

//---------------- TEMP HUM ----------------
SEND_TH: begin
    case(idx)
        1: tx_data<="T";
        2: tx_data<="H";
        3: tx_data<="-";
        4: tx_data<=MPI_ones;
        5: tx_data<="-";
        6: tx_data<=T_tens;
        7: tx_data<=T_ones;
        8: tx_data<="-";
        9: tx_data<=H_tens;
        10: tx_data<=H_ones;
        11: tx_data<="-";
        12: tx_data<="#";
    endcase

    if(tx_done) begin
        fired<=0; gap<=1;
        if(idx==12) begin
            state<=IDLE;
            idx<=0;
        end else idx<=idx+1;
    end
    else if(gap) gap<=0;
    else if(!fired) begin
        tx_start<=1;
        fired<=1;
    end
end

//---------------- POSITION ----------------
SEND_POS: begin
    case(idx)
        1: tx_data<="(";
        2: tx_data<=x_ascii;
        3: tx_data<=",";
        4: tx_data<=y_ascii;
        5: tx_data<=",";
//        6: tx_data<=dir_ascii;
		  6: begin 
				case(dir)
					2'b00 : tx_data <= "N";
					2'b01 : tx_data <= "E";
					2'b10 : tx_data <= "S"; // 01
					2'b11 : tx_data <= "W"; //01
				endcase
		  end
        7: tx_data<=")";
        8: tx_data<="#";
    endcase

    if(tx_done) begin
        fired<=0; gap<=1;
        if(idx==8) begin
            state<=IDLE;
            idx<=0;
        end else idx<=idx+1;
    end
    else if(gap) gap<=0;
    else if(!fired) begin
        tx_start<=1;
        fired<=1;
    end
end

//---------------- END ----------------
SEND_END: begin
    case(idx)
        1: tx_data<="E";
        2: tx_data<="N";
        3: tx_data<="D";
        4: tx_data<="-";
        5: tx_data<="#";
    endcase

    if(tx_done) begin
        fired<=0; gap<=1;
        if(idx==5) begin
            state<=IDLE;
            idx<=0;
        end else idx<=idx+1;
    end
    else if(gap) gap<=0;
    else if(!fired) begin
        tx_start<=1;
        fired<=1;
    end
end

default: state<=IDLE;

endcase
end
end

endmodule
