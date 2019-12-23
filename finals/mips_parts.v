module alu_control(func, aluop, alucontrol);

input   [5:0] func;
input   [1:0] aluop;
output reg [2:0] alucontrol;

always @(aluop, func)
begin
    case(aluop)
      2'b00: alucontrol <= 3'b100;  // add
      2'b01: alucontrol <= 3'b110;  // sub
	  2'b11: alucontrol <= 3'b001;	// or
      default: case(func)          // RTYPE
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

module alu_mips(a, b, control, outalu, zero);

input [31:0] a, b;
input [3:0] control;
output reg [31:0] outalu;
output zero;

always@(control, a, b)
begin
	case(control)
		0 : outalu=a&b;
		1 : outalu= a|b;
		4 : outalu=a+b;
		6 : outalu=a-b;
		7 : outalu=a<b?1:0;
		12 : outalu=~(a|b);
		default : outalu=0;
	endcase
end

assign zero=outalu==0?1:0;

endmodule

module data_mem (memRead, memWrite,address, write_data, read_data);
		    
    input            memRead, memWrite;
    input    [31:0]  address, write_data;

    output   [31:0]  read_data;

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


module instruction_mem(pc, inst);
input [31:0] pc;
output reg [31:0] inst;

reg [7:0] mem[99:0];

initial
begin
	mem[0] = 00110100;
	mem[1] = 00001000;
	mem[2] = 00000000;
	mem[3] = 00001011;
	mem[4] = 00110100;
	mem[5] = 00001001;
	mem[6] = 00000000;
	mem[7] = 00001000;
	mem[8] = 00110100;
	mem[9] = 00001010;
	mem[10] = 00000000 ;
	mem[11] = 00110101 ;
	mem[12] = 00000000 ;
	mem[13] = 00010001 ;
	mem[14] = 01000000 ;
	mem[15] = 10000000 ;
	mem[16] = 00000010 ;
	mem[17] = 00101001 ;
	mem[18] = 10010000 ;
	mem[19] = 00100000 ;
	mem[20] = 00010001 ;
	mem[21] = 01001011 ;
	mem[22] = 00000000 ;
	mem[23] = 00001100 ;
	mem[24] = 10001101 ;
	mem[25] = 00110010 ;
	mem[26] = 00000000 ;
	mem[27] = 00000000 ;
	mem[28] = 00100001 ;
	mem[29] = 01101100 ;
	mem[30] = 00000000 ;
	mem[31] = 00000001 ;
	mem[32] = 00001000 ;
	mem[33] = 00000000 ;
	mem[34] = 00000000 ;
	mem[35] = 00010000 ;
	mem[36] = 00000000 ;
	mem[37] = 00000001 ;
	mem[38] = 00011110 ;
	mem[39] = 11110000 ;
	mem[40] = 00110100 ;
	mem[41] = 00001000 ;
	mem[42] = 00000000 ;
	mem[43] = 00000000 ;
	mem[44] = 00110100 ;
	mem[45] = 00001000 ;
	mem[46] = 00000000 ;
	mem[47] = 00000000 ;
	mem[48] = 00110100 ;
	mem[50] = 00001000 ;
	mem[51] = 00000000 ;
	mem[52] = 00110100 ;
	mem[53] = 00001000 ;
	mem[54] = 00000000 ;
	mem[55] = 00000000 ;
	mem[56] = 00110100 ;
	mem[57] = 00001000 ;
	mem[58] = 00000000 ;
	mem[59] = 00000000 ;
	mem[60] = 00110100 ;
	mem[61] = 00001000 ;
	mem[62] = 00000000 ;
	mem[63] = 00000000 ;
	mem[64] = 00110100 ;
	mem[65] = 00001000 ;
	mem[66] = 00000000 ;
	mem[67] = 00000000 ;
	mem[68] = 00110100 ;
	mem[69] = 00001000 ;
	mem[70] = 00000000 ;
	mem[71] = 00000000 ;
	mem[72] = 00110100 ;
	mem[73] = 00001000 ;
	mem[74] = 00000000 ;
	mem[75] = 00000000 ;
	mem[76] = 00110100 ;
	mem[77] = 00001000 ;
	mem[78] = 00000000 ;
	mem[79] = 00000000 ;
	mem[80] = 00110100 ;
	mem[81] = 00001000 ;
	mem[82] = 00000000 ;
	mem[83] = 00000000 ;
	mem[84] = 00110100 ;
	mem[85] = 00001000 ;
	mem[86] = 00000000 ;
	mem[87] = 00000000 ;
	mem[88] = 00110100 ;
	mem[89] = 00001000 ;
	mem[90] = 00000000 ;
	mem[91] = 00000000 ;
	mem[92] = 00110100 ;
	mem[93] = 00001000 ;
	mem[94] = 00000000 ;
	mem[95] = 00000000 ;
	mem[96] = 00110100 ;
	mem[97] = 00001000 ;
	mem[98] = 00000000 ;
	mem[99] = 00000000 ;
	

end

always@(pc)
begin
	inst={mem[pc], mem[pc+1], mem[pc+2], mem[pc+3]};
end

endmodule
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
		6'b000010 : controls = 9'b0xx00xxx1;	// j
		default : controls = 9'bxxxxxxxxx;		// ???
	endcase
end

endmodule
module mips_control(input clk,
					input reset,
					output [31:0] PC_val,
					output [31:0] WD_val
					);


wire [31:0] PC;
wire [31:0] instr;
wire regWrite;
wire [31:0] readData1, readData2;
wire [31:0] alu_data2;
wire alucontrol;
wire [31:0] alu_result;
wire zero;
wire [1:0] aluop;
wire regDst, aluSrc, branch, memWrite, memRead, jump;
wire [31:0] readData;
wire [31:0] instr_imm;
wire newpc;
wire [4:0] writeReg;
wire [26:0] dump;
wire [31:0] writeData;



instruction_mem im(PC, instr);
registers rm(clk, regWrite, instr[25:21], instr[20:16], writeReg, writeData, readData1, readData2);
alu_mips ma(readData1, alu_data2, alucontrol, alu_result, zero);
alu_control ac(instr[5:0], aluop, alucontrol);
main_control ct(instr[31:26], regWrite, regDst, aluSrc, branch, memWrite, memRead, jump, aluop);

data_mem dm(memRead, memWrite, alu_result, readData2, readData);
sign_ex se(instr[15:0], instr_imm);
pc_adder pa(PC, instr[25:0], instr_imm, branch, zero, jump, newpc);
pc p(reset, clk, newpc, PC);
mux_2to1 m1({27'b0,instr[20:16]},{27'b0,instr[15:11]},regDst, {dump, writeReg});
mux_2to1 m2(readData2, instr_imm, aluSrc, alu_data2);
mux_2to1 m3(alu_result, readData, memRead, writeData);

assign PC_val = PC;
assign WD_val = writeData;



endmodule
module mux_2to1(zero, one, S, out);

input [31:0] zero;
input [31:0] one;
input S;
output reg [31:0] out;

always@(S)
begin
	if(S) out = one;
	else out = zero;

end

endmodule
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
shift_left_two sl2_1({6'b0,instr}, {dump, jump_address[27:0]});
assign jump_address[31:28] = pc4[31:28];
shift_left_two sl2_2(instr_address, instr_address4);
assign a3 = pc4 + instr_address4;
assign bnz = branch & zero;
mux_2to1 mx1(pc4,a3,bnz, m1);
mux_2to1 mx2(m1, jump_address, jump, newpc);


endmodule

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
module registers(input         clk, 
               input         regWrite, 
               input  [4:0]  readReg1, readReg2, writeReg, 
               input  [31:0] writeData, 
               output [31:0] readData1, readData2);

  reg [31:0] reg_mem[31:0];

  always @(posedge clk)
    if (regWrite) reg_mem[writeReg] <= writeData;

  assign readData1 = (readReg1 != 0) ? reg_mem[readReg1] : 0;
  assign readData2 = (readReg2 != 0) ? reg_mem[readReg2] : 0;
endmodule
module shift_left_two(input  [31:0] a,
           output [31:0] b);


	assign b = {a[29:0], 2'b00};

endmodule
module sign_ex(in, out);

input [15:0] in;
output [31:0] out;

assign out = {{16{in[15]}} , in};

endmodule


