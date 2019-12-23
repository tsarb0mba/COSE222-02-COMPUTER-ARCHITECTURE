`timescale 1ns/1ns

module tb_mips();
reg clock;
reg reset;
wire [31:0] PC_val, WD_val;
reg [6:0] HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
mips MM(clock, reset, PC_val, WD_val);

parameter SegF = 7'b011_1000; parameter SegE = 7'b011_0000; parameter SegD = 7'b100_0010; parameter SegC = 7'b011_0001;
parameter SegB = 7'b110_0000; parameter SegA = 7'b000_1000; parameter Seg9 = 7'b000_1100; parameter Seg8 = 7'b000_0000;
parameter Seg7 = 7'b000_1111; parameter Seg6 = 7'b010_0000; parameter Seg5 = 7'b010_0100; parameter Seg4 = 7'b100_1100;
parameter Seg3 = 7'b000_0110; parameter Seg2 = 7'b001_0010; parameter Seg1 = 7'b100_1111; parameter Seg0 = 7'b000_0001;
parameter Segvoid=7'b111_1111;

initial
begin
reset = 1;
clock = 0;
#10 reset = 0;
#5 clock = 1; #5 clock = 0;
#10 reset = 1;
forever #10 clock = !clock;
end
always@(posedge clock)
begin
	case(PC_val[7:4])
		4'b0000: HEX5 = Seg0;
		4'b0001: HEX5 = Seg1;
		4'b0010: HEX5 = Seg2;
		4'b0011: HEX5 = Seg3;
		4'b0100: HEX5 = Seg4;
		4'b0101: HEX5 = Seg5;
		4'b0110: HEX5 = Seg6;
		4'b0111: HEX5 = Seg7;
		4'b1000: HEX5 = Seg8;
		4'b1001: HEX5 = Seg9;
		4'b1010: HEX5 = SegA;
		4'b1011: HEX5 = SegB;
		4'b1100: HEX5 = SegC;
		4'b1101: HEX5 = SegD;
		4'b1110: HEX5 = SegE;
		4'b1111: HEX5 = SegF;
	endcase
	case(PC_val[3:0])
		4'b0000: HEX4 = Seg0;
		4'b0001: HEX4 = Seg1;
		4'b0010: HEX4 = Seg2;
		4'b0011: HEX4 = Seg3;
		4'b0100: HEX4 = Seg4;
		4'b0101: HEX4 = Seg5;
		4'b0110: HEX4 = Seg6;
		4'b0111: HEX4 = Seg7;
		4'b1000: HEX4 = Seg8;
		4'b1001: HEX4 = Seg9;
		4'b1010: HEX4 = SegA;
		4'b1011: HEX4 = SegB;
		4'b1100: HEX4 = SegC;
		4'b1101: HEX4 = SegD;
		4'b1110: HEX4 = SegE;
		4'b1111: HEX4 = SegF;
	endcase
	case(WD_val[15:12])
		4'b0000: HEX3 = Seg0;
		4'b0001: HEX3 = Seg1;
		4'b0010: HEX3 = Seg2;
		4'b0011: HEX3 = Seg3;
		4'b0100: HEX3 = Seg4;
		4'b0101: HEX3 = Seg5;
		4'b0110: HEX3 = Seg6;
		4'b0111: HEX3 = Seg7;
		4'b1000: HEX3 = Seg8;
		4'b1001: HEX3 = Seg9;
		4'b1010: HEX3 = SegA;
		4'b1011: HEX3 = SegB;
		4'b1100: HEX3 = SegC;
		4'b1101: HEX3 = SegD;
		4'b1110: HEX3 = SegE;
		4'b1111: HEX3 = SegF;
	endcase
	case(WD_val[11:8])
		4'b0000: HEX2 = Seg0;
		4'b0001: HEX2 = Seg1;
		4'b0010: HEX2 = Seg2;
		4'b0011: HEX2 = Seg3;
		4'b0100: HEX2 = Seg4;
		4'b0101: HEX2 = Seg5;
		4'b0110: HEX2 = Seg6;
		4'b0111: HEX2 = Seg7;
		4'b1000: HEX2 = Seg8;
		4'b1001: HEX2 = Seg9;
		4'b1010: HEX2 = SegA;
		4'b1011: HEX2 = SegB;
		4'b1100: HEX2 = SegC;
		4'b1101: HEX2 = SegD;
		4'b1110: HEX2 = SegE;
		4'b1111: HEX2 = SegF;
	endcase
	case(WD_val[7:4])
		4'b0000: HEX1 = Seg0;
		4'b0001: HEX1 = Seg1;
		4'b0010: HEX1 = Seg2;
		4'b0011: HEX1 = Seg3;
		4'b0100: HEX1 = Seg4;
		4'b0101: HEX1 = Seg5;
		4'b0110: HEX1 = Seg6;
		4'b0111: HEX1 = Seg7;
		4'b1000: HEX1 = Seg8;
		4'b1001: HEX1 = Seg9;
		4'b1010: HEX1 = SegA;
		4'b1011: HEX1 = SegB;
		4'b1100: HEX1 = SegC;
		4'b1101: HEX1 = SegD;
		4'b1110: HEX1 = SegE;
		4'b1111: HEX1 = SegF;
	endcase
	case(WD_val[3:0])
		4'b0000: HEX0 = Seg0;
		4'b0001: HEX0 = Seg1;
		4'b0010: HEX0 = Seg2;
		4'b0011: HEX0 = Seg3;
		4'b0100: HEX0 = Seg4;
		4'b0101: HEX0 = Seg5;
		4'b0110: HEX0 = Seg6;
		4'b0111: HEX0 = Seg7;
		4'b1000: HEX0 = Seg8;
		4'b1001: HEX0 = Seg9;
		4'b1010: HEX0 = SegA;
		4'b1011: HEX0 = SegB;
		4'b1100: HEX0 = SegC;
		4'b1101: HEX0 = SegD;
		4'b1110: HEX0 = SegE;
		4'b1111: HEX0 = SegF;
	endcase
end
endmodule
