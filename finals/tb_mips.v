`timescale 1ns/1ns

module tb_mips();
reg clock;
reg reset;
wire [31:0] PC, AR, RD1, AD2, RD2, ID;
wire [8:0] controls; wire [2:0] AC;
mips MM(clock, reset, PC, AR, RD1, AD2, RD2, ID, controls,AC);

initial
begin
reset = 1;
clock = 0;
#10 reset = 0;
#10 reset = 1;
forever #10 clock = !clock;
end
endmodule
