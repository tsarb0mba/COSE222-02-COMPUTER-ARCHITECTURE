module mips(clk, reset, PC_val, WD_val);
input clk;
input reset;
output [31:0] PC_val;
output [31:0] WD_val;
mips_control mc(clk, reset, PC_val, WD_val);
endmodule
