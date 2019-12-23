//
module mips_control(clk, reset, PC_val, AR_val, RD1, AD2, RD2, ID, controls,AC);
input clk;
input reset;
output [31:0] PC_val;
output [31:0] AR_val, RD1, AD2, RD2, ID;
output [8:0] controls; output [2:0] AC;

wire [31:0] PC, instr, readData1, readData2, alu_data2, alu_result, readData, instr_imm, writeData, newpc;
wire regWrite, zero, regDst, aluSrc, branch, memWrite, memRead, jump;
wire [1:0] aluop;
wire [2:0] alucontrol;
wire [4:0] writeReg;
assign AR_val = alu_result;
assign PC_val = PC;

assign RD1 = readData1;
assign AD2=alu_data2;assign RD2 = readData2; assign ID = instr;
assign controls = {regWrite, regDst, aluSrc, branch, memWrite, memRead, jump, aluop};
assign AC = alucontrol;

instruction_mem	im(PC, instr);
registers		rm(clk, regWrite, instr[25:21], instr[20:16], writeReg, writeData, readData1, readData2);
alu_mips		am(readData1, alu_data2, alucontrol, alu_result, zero);
alu_control		ac(instr[5:0], aluop, alucontrol);
main_control	ct(instr[31:26], regWrite, regDst, aluSrc, branch, memWrite, memRead, jump, aluop);
data_mem		dm(memRead, memWrite, alu_result, readData2, readData);
sign_ex			se(instr[15:0], instr_imm);
pc_adder		pa(PC, instr[25:0], instr_imm, branch, zero, jump, newpc);
pc				pu(reset, clk, newpc, PC);
mux_5b			m1(instr[20:16],instr[15:11],regDst, writeReg);
mux_32b			m2(readData2, instr_imm, aluSrc, alu_data2);
mux_32b			m3(alu_result, readData, memRead, writeData);
endmodule
//
module alu_control(func, aluop, alucontrol);
input [5:0] func;
input [1:0] aluop;
output reg [2:0] alucontrol;

always @(aluop, func)
begin
	case(aluop)
		2'b00: alucontrol <= 3'b100;  // add
		2'b01: alucontrol <= 3'b110;  // sub
		2'b11: alucontrol <= 3'b001;	// or
		2'b10: case(func)          // RTYPE
			6'b100000: alucontrol <= 3'b100; // add
			6'b100010: alucontrol <= 3'b110; // sub
			6'b100100: alucontrol <= 3'b000; // and
			6'b100101: alucontrol <= 3'b001; // or
			6'b101011: alucontrol <= 3'b111; // sltu
			default:   alucontrol <= 3'bxxx; // ???
		endcase
	endcase
end
endmodule
//
module alu_mips(a, b, control, outalu, zero);
input [31:0] a, b;
input [3:0]	control;
output reg [31:0] outalu;
output zero;

assign zero = (outalu==0?1:0);

always@(control, a, b)
begin
	case(control)
		0: outalu=a&b;
		1: outalu= a|b;
		4: outalu=a+b;
		6: outalu=a-b;
		7: outalu=a<b?1:0;
		12: outalu=~(a|b);
		default : outalu = 0;
	endcase
end
endmodule
//
module data_mem (memRead, memWrite,address, write_data, read_data);
	input memRead, memWrite;
	input [31:0] address, write_data;
	output [31:0] read_data;

    parameter        size = 64;	// data mem size
    integer          i;

  //  wire     [31:0]  index;
    reg      [31:0]  register_memory [0:size-1];

   // assign index = address >> 2;  

    initial
    begin
        for (i = 0; i < size; i = i + 1)
          //  register_memory[i] = 32'b0;
	   $readmemb("data_mem.mem", register_memory);
    end

    always @ (memWrite, write_data)
    begin
        if (memWrite == 1'b1) begin
            register_memory[address] = write_data;
        end
    end

 assign read_data = (memRead == 1'b1)?register_memory[address]:32'b0;

endmodule
//
module instruction_mem(pc, inst);
input [31:0] pc;
output reg [31:0] inst;

reg [7:0] mem[99:0];

initial
begin
	mem[0] = 8'b00110100;
	mem[1] = 8'b00001000;
	mem[2] = 8'b00000000;
	mem[3] = 8'b00001011;
	mem[4] = 8'b00110100;
	mem[5] = 8'b00001001;
	mem[6] = 8'b00000000;
	mem[7] = 8'b00001000;
	mem[8] = 8'b00110100;
	mem[9] = 8'b00001010;
	mem[10] = 8'b00000000 ;
	mem[11] = 8'b00110101 ;
	mem[12] = 8'b00000000 ;
	mem[13] = 8'b00010001 ;
	mem[14] = 8'b01000000 ;
	mem[15] = 8'b10000000 ;
	mem[16] = 8'b00000010 ;
	mem[17] = 8'b00101001 ;
	mem[18] = 8'b10010000 ;
	mem[19] = 8'b00100000 ;
	mem[20] = 8'b00010001 ;
	mem[21] = 8'b01001011 ;
	mem[22] = 8'b00000000 ;
	mem[23] = 8'b00000011 ;
	mem[24] = 8'b10001101 ;
	mem[25] = 8'b00110010 ;
	mem[26] = 8'b00000000 ;
	mem[27] = 8'b00000000 ;
	mem[28] = 8'b00100001 ;
	mem[29] = 8'b01101100 ;
	mem[30] = 8'b00000000 ;
	mem[31] = 8'b00000001 ;
	mem[32] = 8'b00001000 ;
	mem[33] = 8'b00000000 ;
	mem[34] = 8'b00000000 ;
	mem[35] = 8'b00000100 ;
	mem[36] = 8'b00000000 ;
	mem[37] = 8'b00000001 ;
	mem[38] = 8'b00011110 ;
	mem[39] = 8'b11110000 ;
	mem[40] = 8'b00110100 ;
	mem[41] = 8'b00001000 ;
	mem[42] = 8'b00000000 ;
	mem[43] = 8'b00000000 ;
	mem[44] = 8'b00110100 ;
	mem[45] = 8'b00001000 ;
	mem[46] = 8'b00000000 ;
	mem[47] = 8'b00000000 ;
	mem[48] = 8'b00110100 ;
	mem[50] = 8'b00001000 ;
	mem[51] = 8'b00000000 ;
	mem[52] = 8'b00110100 ;
	mem[53] = 8'b00001000 ;
	mem[54] = 8'b00000000 ;
	mem[55] = 8'b00000000 ;
	mem[56] = 8'b00110100 ;
	mem[57] = 8'b00001000 ;
	mem[58] = 8'b00000000 ;
	mem[59] = 8'b00000000 ;
	mem[60] = 8'b00110100 ;
	mem[61] = 8'b00001000 ;
	mem[62] = 8'b00000000 ;
	mem[63] = 8'b00000000 ;
	mem[64] = 8'b00110100 ;
	mem[65] = 8'b00001000 ;
	mem[66] = 8'b00000000 ;
	mem[67] = 8'b00000000 ;
	mem[68] = 8'b00110100 ;
	mem[69] = 8'b00001000 ;
	mem[70] = 8'b00000000 ;
	mem[71] = 8'b00000000 ;
	mem[72] = 8'b00110100 ;
	mem[73] = 8'b00001000 ;
	mem[74] = 8'b00000000 ;
	mem[75] = 8'b00000000 ;
	mem[76] = 8'b00110100 ;
	mem[77] = 8'b00001000 ;
	mem[78] = 8'b00000000 ;
	mem[79] = 8'b00000000 ;
	mem[80] = 8'b00110100 ;
	mem[81] = 8'b00001000 ;
	mem[82] = 8'b00000000 ;
	mem[83] = 8'b00000000 ;
	mem[84] = 8'b00110100 ;
	mem[85] = 8'b00001000 ;
	mem[86] = 8'b00000000 ;
	mem[87] = 8'b00000000 ;
	mem[88] = 8'b00110100 ;
	mem[89] = 8'b00001000 ;
	mem[90] = 8'b00000000 ;
	mem[91] = 8'b00000000 ;
	mem[92] = 8'b00110100 ;
	mem[93] = 8'b00001000 ;
	mem[94] = 8'b00000000 ;
	mem[95] = 8'b00000000 ;
	mem[96] = 8'b00110100 ;
	mem[97] = 8'b00001000 ;
	mem[98] = 8'b00000000 ;
	mem[99] = 8'b00000000 ;
end

always@(pc)
begin
	inst={mem[pc], mem[pc+1], mem[pc+2], mem[pc+3]};
end
endmodule
//
module main_control(op, regwrite, regdst, alusrc, branch, memWrite, memRead, jump, aluop);
input [5:0] op;
output regwrite, regdst, alusrc, branch, memWrite, memRead, jump;
output [1:0] aluop;

reg [8:0] controls;
assign {regwrite, regdst, alusrc, branch, memWrite, memRead, jump, aluop} = controls;

always@(*)
begin
	case(op)
		6'b000000 : controls = 9'b110000010;	// R-type
		6'b100011 : controls = 9'b101001000;	// lw
		6'b101011 : controls = 9'b001010000;	// sw
		6'b000100 : controls = 9'b000100001;	// beq
		6'b001000 : controls = 9'b101000000;	// addi
		6'b001101 : controls = 9'b101000011;	// ori
		6'b000010 : controls = 9'b000000001;	// j
		default : controls = 9'bxxxxxxxxx;		// ???
	endcase
end
endmodule
//
module mux_5b(zero, one, S, out);
input [4:0] zero;
input [4:0] one;
input S;
output reg [4:0] out;

always@(*)
begin
	if(S) out = one;
	else out = zero;

end
endmodule
//
module mux_32b(zero, one, S, out);
input [31:0] zero;
input [31:0] one;
input S;
output reg [31:0] out;

always@(*)
begin
	if(S) out <= one;
	else out <= zero;

end
endmodule
//
module pc(rst, clk, newpc, pc);
input rst, clk;
input [31:0] newpc;
output reg [31:0] pc;

always@(negedge rst or posedge clk)
begin
	if(!rst)
		pc <= 0;
	else
		pc <= newpc;
end
endmodule
//
module pc_adder(pc, instr, instr_address, branch, zero, jump, newpc);
input [31:0] pc;
input [25:0] instr;
input [31:0] instr_address;
input branch, zero, jump;
output [31:0] newpc;

wire [31:0] jump_address;
wire [3:0] dump;
wire [31:0] pc4;
wire [31:0] instr_address4;
wire [31:0] a3;
wire [31:0] m1;
wire bnz;
assign pc4 = pc + 4;
assign jump_address[31:28] = pc4[31:28];
assign a3 = pc4 + instr_address4;
assign bnz = branch & zero;
shift_left_two	sl2_1(instr, jump_address[27:0]);
shift_left_two	sl2_2(instr_address, instr_address4);
mux_32b			mx1(pc4,a3,bnz, m1);
mux_32b			mx2(m1, jump_address, jump, newpc);
endmodule
//
module regfile(input             clk, 
               input             we, 
               input      [4:0]  ra1, ra2, wa, 
               input      [31:0] wd, 
               output reg [31:0] rd1, rd2);

	reg [31:0] R1;
	reg [31:0] R2;
	reg [31:0] R3;
	reg [31:0] R4;
	reg [31:0] R5;
	reg [31:0] R6;
	reg [31:0] R7;
	reg [31:0] R8;
	reg [31:0] R9;
	reg [31:0] R10;
	reg [31:0] R11;
	reg [31:0] R12;
	reg [31:0] R13;
	reg [31:0] R14;
	reg [31:0] R15;
	reg [31:0] R16;
	reg [31:0] R17;
	reg [31:0] R18;
	reg [31:0] R19;
	reg [31:0] R20;
	reg [31:0] R21;
	reg [31:0] R22;
	reg [31:0] R23;
	reg [31:0] R24;
	reg [31:0] R25;
	reg [31:0] R26;
	reg [31:0] R27;
	reg [31:0] R28;
	reg [31:0] R29;
	reg [31:0] R30;
	reg [31:0] R31;

	always @(posedge clk)
	begin
		if (we) 
		begin
   		case (wa[4:0])
			5'd0:   ;
			5'd1:   R1  <= wd;
			5'd2:   R2  <= wd;
			5'd3:   R3  <= wd;
			5'd4:   R4  <= wd;
			5'd5:   R5  <= wd;
			5'd6:   R6  <= wd;
			5'd7:   R7  <= wd;
			5'd8:   R8  <= wd;
			5'd9:   R9  <= wd;
			5'd10:  R10 <= wd;
			5'd11:  R11 <= wd;
			5'd12:  R12 <= wd;
			5'd13:  R13 <= wd;
			5'd14:  R14 <= wd;
			5'd15:  R15 <= wd;
			5'd16:  R16 <= wd;
			5'd17:  R17 <= wd;
			5'd18:  R18 <= wd;
			5'd19:  R19 <= wd;
			5'd20:  R20 <= wd;
			5'd21:  R21 <= wd;
			5'd22:  R22 <= wd;
			5'd23:  R23 <= wd;
			5'd24:  R24 <= wd;
			5'd25:  R25 <= wd;
			5'd26:  R26 <= wd;
			5'd27:  R27 <= wd;
			5'd28:  R28 <= wd;
			5'd29:  R29 <= wd;
			5'd30:  R30 <= wd;
			5'd31:  R31 <= wd;
   		endcase
		end
	end

	always @(*)
	begin
		case (ra2[4:0])
		5'd0:   rd2 = 32'b0;
		5'd1:   rd2 = R1;
		5'd2:   rd2 = R2;
		5'd3:   rd2 = R3;
		5'd4:   rd2 = R4;
		5'd5:   rd2 = R5;
		5'd6:   rd2 = R6;
		5'd7:   rd2 = R7;
		5'd8:   rd2 = R8;
		5'd9:   rd2 = R9;
		5'd10:  rd2 = R10;
		5'd11:  rd2 = R11;
		5'd12:  rd2 = R12;
		5'd13:  rd2 = R13;
		5'd14:  rd2 = R14;
		5'd15:  rd2 = R15;
		5'd16:  rd2 = R16;
		5'd17:  rd2 = R17;
		5'd18:  rd2 = R18;
		5'd19:  rd2 = R19;
		5'd20:  rd2 = R20;
		5'd21:  rd2 = R21;
		5'd22:  rd2 = R22;
		5'd23:  rd2 = R23;
		5'd24:  rd2 = R24;
		5'd25:  rd2 = R25;
		5'd26:  rd2 = R26;
		5'd27:  rd2 = R27;
		5'd28:  rd2 = R28;
		5'd29:  rd2 = R29;
		5'd30:  rd2 = R30;
		5'd31:  rd2 = R31;
		endcase
	end

	always @(*)
	begin
		case (ra1[4:0])
		5'd0:   rd1 = 32'b0;
		5'd1:   rd1 = R1;
		5'd2:   rd1 = R2;
		5'd3:   rd1 = R3;
		5'd4:   rd1 = R4;
		5'd5:   rd1 = R5;
		5'd6:   rd1 = R6;
		5'd7:   rd1 = R7;
		5'd8:   rd1 = R8;
		5'd9:   rd1 = R9;
		5'd10:  rd1 = R10;
		5'd11:  rd1 = R11;
		5'd12:  rd1 = R12;
		5'd13:  rd1 = R13;
		5'd14:  rd1 = R14;
		5'd15:  rd1 = R15;
		5'd16:  rd1 = R16;
		5'd17:  rd1 = R17;
		5'd18:  rd1 = R18;
		5'd19:  rd1 = R19;
		5'd20:  rd1 = R20;
		5'd21:  rd1 = R21;
		5'd22:  rd1 = R22;
		5'd23:  rd1 = R23;
		5'd24:  rd1 = R24;
		5'd25:  rd1 = R25;
		5'd26:  rd1 = R26;
		5'd27:  rd1 = R27;
		5'd28:  rd1 = R28;
		5'd29:  rd1 = R29;
		5'd30:  rd1 = R30;
		5'd31:  rd1 = R31;
		endcase
	end
endmodule
//
module registers(input         clk, 
               input         regWrite, 
               input  [4:0]  readReg1, readReg2, writeReg, 
               input  [31:0] writeData, 
               output [31:0] readData1, readData2);

	reg [31:0] reg_mem[31:0];
	assign readData1 = (readReg1 != 0) ? reg_mem[readReg1] : 0;
	assign readData2 = (readReg2 != 0) ? reg_mem[readReg2] : 0;

	initial
	begin
	reg_mem[0] = 0;
	end
	
	always @(posedge clk)
		if (regWrite) reg_mem[writeReg] <= writeData;
endmodule
//
module shift_left_two(a, b);
	input  [31:0] a;
	output [31:0] b;
	
	assign b = {a[29:0], 2'b00};
endmodule
//
module sign_ex(in, out);
input [15:0] in;
output [31:0] out;

assign out = {{16{in[15]}} , in};
endmodule
//