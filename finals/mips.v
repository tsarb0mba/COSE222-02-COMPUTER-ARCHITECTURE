module mips(clk, reset, PC_val, AR_val, RD1, AD2, RD2, ID, controls,AC);
input clk;
input reset;
output [31:0] PC_val;
output [31:0] AR_val, RD1, AD2, RD2, ID;
output [8:0] controls; output[2:0] AC;
mips_control mc(clk, reset, PC_val, WD_val, RD1, AD2, RD2, ID, controls,AC);
endmodule
