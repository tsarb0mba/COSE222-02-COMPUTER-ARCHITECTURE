//
module mips_control(clk, reset, PC_val, WD_val);
input clk;
input reset;
output [31:0] PC_val;
output [31:0] WD_val;

wire [31:0] PC, instr, readData1, readData2, alu_data2, alu_result, readData, instr_imm, writeData, newpc;
wire regWrite, zero, regDst, aluSrc, branch, memWrite, memRead, jump;
wire [1:0] aluop;
wire [3:0] alucontrol;
wire [4:0] writeReg;
assign PC_val = PC;
assign WD_val = writeData;
instruction_mem	im(PC, instr);
registers		rm(clk, regWrite, instr[25:21], instr[20:16], writeReg, writeData, readData1, readData2);
alu_control		ac(instr[5:0], aluop, alucontrol);
alu_mips		am(readData1, alu_data2, instr[10:6], alucontrol, alu_result, zero);
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
output reg [3:0] alucontrol;

always @(aluop, func)
begin
	case(aluop)
		2'b00: alucontrol <= 4'b0010;  // add
		2'b01: alucontrol <= 4'b0110;  // sub
		2'b11: alucontrol <= 4'b0001;	// or
		2'b10: case(func)          // RTYPE
			6'b100000: alucontrol <= 4'b0010; // add
			6'b100010: alucontrol <= 4'b0110; // sub
			6'b100100: alucontrol <= 4'b0000; // and
			6'b100101: alucontrol <= 4'b0001; // or
			6'b000000: alucontrol <= 4'b1010; // sll
			default:   alucontrol <= 4'bxxxx; // ???
		endcase
	endcase
end
endmodule
//
module alu_mips(a, b, shamt, control, outalu, zero);
input [31:0] a, b;
input [4:0] shamt;
input [3:0]	control;
output reg [31:0] outalu;
output zero;

assign zero = (outalu==0?1:0);

always@(control, a, b)
begin
	case(control)
		4'b0010: outalu = a + b;
		4'b0110: outalu = a - b;
		4'b0000: outalu = a & b;
		4'b0001: outalu = a | b;
		4'b1001: outalu = a ^ b;
		4'b1010: outalu = b << shamt;
		4'b1100: outalu = b >> shamt;
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
	mem[13] = 8'b00001000 ;
	mem[14] = 8'b10001000 ;
	mem[15] = 8'b10000000 ;
	mem[16] = 8'b00000010 ;
	mem[17] = 8'b00101001 ;
	mem[18] = 8'b10010000 ;
	mem[19] = 8'b00100000 ;
	mem[20] = 8'b00010001 ;
	mem[21] = 8'b01010010 ;
	mem[22] = 8'b11111111 ;
	mem[23] = 8'b11111101 ;
	mem[24] = 8'b10001110 ;
	mem[25] = 8'b01001011 ;
	mem[26] = 8'b00000000 ;
	mem[27] = 8'b00000000 ;
	mem[28] = 8'b00100001 ;
	mem[29] = 8'b01101100 ;
	mem[30] = 8'b00000000 ;
	mem[31] = 8'b00000001 ;
	mem[32] = 8'b00001000 ;
	mem[33] = 8'b00000000 ;
	mem[34] = 8'b00000000 ;
	mem[35] = 8'b00001001 ;
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
		6'b101011 : controls = 9'b0x101x000;	// sw
		6'b000100 : controls = 9'b0x010x001;	// beq
		6'b001000 : controls = 9'b101000000;	// addi
		6'b001101 : controls = 9'b101000011;	// ori
		6'b000010 : controls = 9'b0xx0001xx;	// j
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
module registers(clk, regWrite, readReg1, readReg2, writeReg, writeData, readData1, readData2);
	input clk;
	input regWrite;
	input  [4:0]  readReg1, readReg2, writeReg;
	input  [31:0] writeData; 
	output [31:0] readData1, readData2;
	
	reg [31:0] reg_mem[31:0];
	assign readData1 = (readReg1 != 0) ? reg_mem[readReg1] : 0;
	assign readData2 = (readReg2 != 0) ? reg_mem[readReg2] : 0;

	initial
	begin
	reg_mem[31]=0;
	reg_mem[30]=0;
	reg_mem[29]=0;
	reg_mem[28]=0;
	reg_mem[27]=0;
	reg_mem[26]=0;
	reg_mem[25]=0;
	reg_mem[24]=0;
	reg_mem[23]=0;
	reg_mem[22]=0;
	reg_mem[21]=0;
	reg_mem[20]=0;
	reg_mem[19]=0;
	reg_mem[18]=0;
	reg_mem[17]=0;
	reg_mem[16]=0;
	reg_mem[15]=0;
	reg_mem[14]=0;
	reg_mem[13]=0;
	reg_mem[12]=0;
	reg_mem[11]=0;
	reg_mem[10]=0;
	reg_mem[9]=0;
	reg_mem[8]=0;
	reg_mem[7]=0;
	reg_mem[6]=0;
	reg_mem[5]=0;
	reg_mem[4]=0;
	reg_mem[3]=0;
	reg_mem[2]=0;
	reg_mem[1]=0;
	reg_mem[0]=0;
	end
	always @(posedge clk)
		if (regWrite)if(writeReg) reg_mem[writeReg] <= writeData;
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