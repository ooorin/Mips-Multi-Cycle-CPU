`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/04/02 16:00:03
// Design Name: 
// Module Name: test
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
parameter wid = 32;

parameter FETCH		= 5'b00000;
parameter DECODE	= 5'b00001;
parameter MEMADR 	= 5'b00010;
parameter MEMRD 	= 5'b00011;
parameter MEMWB		= 5'b00100;
parameter MEMWR 	= 5'b00101;
parameter RTYPEEX 	= 5'b00110;
parameter RTYPEWB 	= 5'b00111;
parameter BEQEX 	= 5'b01000;
parameter BNEEX		= 5'b01001;
parameter ADDIEX 	= 5'b01010;
parameter ADDIWB 	= 5'b01011;
parameter ANDIEX	= 5'b01100;
parameter ANDIWB 	= 5'b01101;
parameter ORIEX 	= 5'b01110;
parameter ORIWB 	= 5'b01111;
parameter SLTIEX	= 5'b10000;
parameter SLTIWB	= 5'b10001;
parameter JEX 		= 5'b10010;

parameter LW 		= 6'b100011;
parameter SW 		= 6'b101011;
parameter RTYPE 	= 6'b000000;
parameter BEQ 		= 6'b000100;
parameter BNE 		= 6'b000101;
parameter ADDI 		= 6'b001000; 
parameter ANDI		= 6'b001100; 
parameter ORI		= 6'b001101;
parameter SLTI		= 6'b001010;
parameter J 		= 6'b000010;


module ALU(
	input logic [2:0] aluCtrl,
	input logic [wid - 1:0] a, [wid - 1:0] b,
	
	output logic [wid - 1:0] result,
	output logic zero
    );

	logic [wid:0] r;

	always_comb
	begin
		case(aluCtrl)
			3'b000: r = a & b;
			3'b001: r = a | b;
			3'b010: r = a + b;
			3'b110: r = a - b;
			3'b111: r = a < b ? 32'hFFFFFFFF : 32'b0; // SLT
			default: r = r;
		endcase
	end

	assign zero = (r == 0);
	assign result = r;

endmodule

module PC(
	input logic clk,
	input logic rst,
	input logic en,
	input logic [wid - 1:0] in,

	output logic [wid - 1:0] out
	);

	always_ff @(posedge clk)
	begin
		if (rst) out <= 32 * 4;
		else if (en) out <= in;
	end

endmodule

module FLOPREN( 
	input logic clk,
	input logic en,
	input logic [wid - 1:0] in,

	output logic [wid - 1:0] out
	);

	always_ff @(posedge clk)
	begin
		if (en) out <= in;
	end

endmodule

module FLOPR(
	input logic clk,
	input logic [wid - 1:0] in,

	output logic [wid - 1:0] out
	);

	always_ff @(posedge clk)
	begin
		out <= in;
	end

endmodule

module FLOPR2(
	input logic clk,
	input logic [wid - 1:0] in1, in2,

	output logic [wid - 1:0] out1, out2
	);

	always_ff @(posedge clk)
	begin
		out1 <= in1;
	end

	always_ff @(posedge clk)
	begin
		out2 <= in2;
	end

endmodule

module INSDATMEM(					//instruction and data memory
	input logic clk, wEn,
	input logic [wid - 1:0] addr,
	input logic [wid - 1:0] writeData,

	output logic [wid - 1:0] readData
	);

	logic [wid - 1:0] rom [63:0];
	logic [wid - 1:0] tmp [31:0];

	initial
	begin
		$readmemh("C:/Users/10441/Desktop/project_1/project_1.srcs/sources_1/new/iMemFile2.dat", tmp);
	end

	assign rom[63:32] = tmp[31:0];

	always_ff @(posedge clk)
	begin
		if (wEn) rom[addr[31:2]] <= writeData;
	end

	assign readData = rom[addr[31:2]];

endmodule

module REGFILE(
	input logic clk,
	input logic rst,
	input logic regWriteEn,

	input logic [4:0] regWriteAddr,
	input logic [wid - 1:0] regWriteData,

	input logic [4:0] rsAddr,
	input logic [4:0] rtAddr,

	output logic [wid - 1:0] rsData,
	output logic [wid - 1:0] rtData
	);

	logic [wid - 1:0] rf [wid - 1:0];

	integer i;
	
	always_ff @(posedge clk)
	begin
		if (rst)
		begin
			for (i = 0; i < 32; i = i + 1)
				rf[i] = 32'b0;
		end
		else if (regWriteEn) rf[regWriteAddr] <= regWriteData;
	end

	assign rsData = (rsAddr != 0) ? rf[rsAddr] : 0;
	assign rtData = (rtAddr != 0) ? rf[rtAddr] : 0;

endmodule

module SIGNEXT(
	input logic [wid / 2 - 1:0] in,

	output logic [wid - 1:0] out
	);

	assign out = {{(wid / 2){in[wid / 2 - 1]}}, in};

endmodule

module MUX2 #(parameter w = 32)
	(
	input logic sw,
	input logic [w - 1:0] in1, in2,

	output logic [w - 1:0] out
	);

	logic [w - 1:0] y;

	always_comb
	begin
		if (sw == 1) out = in2;
		else out = in1;
	end

endmodule

module MUX4 #(parameter w = 32)
	(
	input logic [1:0] sw,
	input logic [w - 1:0] in1, in2, in3, in4,

	output logic [w - 1:0] out
	);

	always_comb
	begin
		case (sw)
			2'b00: 		out = in1;
			2'b01: 		out = in2;
			2'b10: 		out = in3;
			2'b11: 		out = in4;
			default: 	out = 2'bxx;
		endcase
	end

endmodule

module SHIFT(
	input logic [wid - 1:0] in,

	output logic [wid - 1:0] out
	);

	assign out = {in[wid - 3:0], 2'b00};

endmodule

module MAINDEC(
	input logic clk, rst,
	input logic [5:0] op,

	output logic pcWrite, memWrite, iRWrite, regWrite,
	output logic aluSrcA, branch0, branch1, IorD, memToReg, regDst,
	output logic [1:0] aluSrcB, pcSrc,
	output logic [2:0] aluOp
	);
	
	logic [3:0] state, nxtState;
	logic [16:0] controls;

	always_ff @(posedge clk)
	begin
		if (rst) state <= FETCH;
		else state <= nxtState;
	end

	always_comb
	begin
		case(state)
			FETCH: 		nxtState = DECODE;
			DECODE: 	case(op)
						LW: 	nxtState = MEMADR;
						SW: 	nxtState = MEMADR;
						RTYPE:	nxtState = RTYPEEX;
						BEQ: 	nxtState = BEQEX;
						BNE:	nxtState = BNEEX;
						ADDI: 	nxtState = ADDIEX;
						ANDI:	nxtState = ANDIEX;
						ORI:	nxtState = ORIEX;
						SLTI:	nxtState = SLTIEX;
						J: 		nxtState = JEX;
						default:
								nxtState = 4'bx;
						endcase
			MEMADR: 	case(op)
						LW:		nxtState = MEMRD;
						SW: 	nxtState = MEMWR;
						default:
								nxtState = 4'bx;
						endcase
			MEMRD: 		nxtState = MEMWB;
			MEMWB: 		nxtState = FETCH;
			MEMWR: 		nxtState = FETCH;
			RTYPEEX: 	nxtState = RTYPEWB;
			RTYPEWB: 	nxtState = FETCH;
			BEQEX: 		nxtState = FETCH;
			BNEEX:		nxtState = FETCH;
			ADDIEX: 	nxtState = ADDIWB;
			ADDIWB: 	nxtState = FETCH;
			ANDIEX:		nxtState = ANDIWB;
			ANDIWB: 	nxtState = FETCH;
			ORIEX:		nxtState = ORIWB;
			ORIWB: 		nxtState = FETCH;
			SLTIEX:		nxtState = SLTIWB;
			SLTIWB: 	nxtState = FETCH;
			JEX: 		nxtState = FETCH;
			default:	nxtState = 4'bx;
		endcase
	end

	assign {branch1, pcWrite, memWrite, iRWrite, regWrite,
			aluSrcA, branch0, IorD, memToReg, regDst,
			aluSrcB, pcSrc,
			aluOp} = controls;

	always_comb
	begin
		case(state)
			FETCH: 		controls = 17'b0101_0000_0001_00000;
			DECODE: 	controls = 17'b0000_0000_0011_00000;
			MEMADR: 	controls = 17'b0000_0100_0010_00000;
			MEMRD: 		controls = 17'b0000_0001_0000_00000;
			MEMWB: 		controls = 17'b0000_1000_1000_00000;
			MEMWR: 		controls = 17'b0010_0001_0000_00000;
			RTYPEEX: 	controls = 17'b0000_0100_0000_00100;
			RTYPEWB: 	controls = 17'b0000_1000_0100_00000;
			BEQEX: 		controls = 17'b0000_0110_0000_01001;
			BNEEX:		controls = 17'b1000_0100_0000_01001;
			ADDIEX: 	controls = 17'b0000_0100_0010_00000;
			ADDIWB: 	controls = 17'b0000_1000_0000_00000;
			ANDIEX:		controls = 17'b0000_0100_0010_00010;
			ANDIWB: 	controls = 17'b0000_1000_0000_00010;
			ORIEX:		controls = 17'b0000_0100_0010_00011;
			ORIWB: 		controls = 17'b0000_1000_0000_00011;
			SLTIEX:		controls = 17'b0000_0100_0010_00101;
			SLTIWB: 	controls = 17'b0000_1000_0000_00101;
			JEX: 		controls = 17'b0100_0000_0000_10000;
			default: 	controls = 17'bx;
		endcase
	end

endmodule

module ALUDEC(
	input logic [5:0] func,
	input logic [2:0] aluOp, // ???

	output logic [2:0] aluCtrl
	);

	always_comb
	begin
		case(aluOp)
			3'b000: aluCtrl = 3'b010;
			3'b001: aluCtrl = 3'b110;
			3'b010: aluCtrl = 3'b000;
			3'b011: aluCtrl = 3'b001;
			3'b101: aluCtrl = 3'b111;
			default: 
				case(func)
					6'b000000: aluCtrl = 3'b010; // NOP
					6'b100000: aluCtrl = 3'b010; // ADD
					6'b100010: aluCtrl = 3'b110; // SUB
					6'b100100: aluCtrl = 3'b000; // AND
					6'b100101: aluCtrl = 3'b001; // OR
					6'b101010: aluCtrl = 3'b111; // SLT
					default:   aluCtrl = 3'bxxx;
				endcase
		endcase
	end

endmodule

module CONTROLLER(
	input logic clk, rst,
	input logic [5:0] op, func,

	output logic pcWrite, memWrite, iRWrite, regWrite,
	output logic aluSrcA, branch0, branch1, IorD, memToReg, regDst,
	output logic [1:0] aluSrcB, pcSrc,
	output logic [2:0] aluCtrl,

	output logic [5:0] opOut
	);

	logic [2:0]aluOp;

	MAINDEC mD(clk, rst, op, pcWrite, memWrite, iRWrite, regWrite, aluSrcA, branch0, branch1, IorD, memToReg, regDst, aluSrcB, pcSrc, aluOp);
	ALUDEC aD(func, aluOp, aluCtrl);

	assign opOut = op;

endmodule

module DATAPATH(
	input logic clk, rst,
	input logic pcWrite, memWrite, iRWrite, regWrite,
	input logic aluSrcA, branch0, branch1, IorD, memToReg, regDst,
	input logic [1:0] aluSrcB, pcSrc,
	input logic [2:0] aluCtrl,

	output logic [wid - 1:0] pcOut, instrOut, readDataOut,
	output logic [wid - 1:0] rsDatOut, rtDatOut,
	output logic [wid - 1:0] srcAOut, srcBOut, aluResOut
	);

	logic pcEn;
	logic zero;
	logic [wid - 1:0] pcNxt, pc;
	logic [wid - 1:0] addr, readData, instr, data, writeRegDat;
	logic [4:0] writeRegAddr;
	logic [wid - 1:0] sigImm, sigImmSh;
	logic [wid - 1:0] rsDat, rtDat, valA, valB, srcA, srcB;
	logic [wid - 1:0] aluRes, aluOut;

	//FLOPREN pcReg(clk, pcEn, pcNxt, pc);
	PC pcReg(clk, rst, pcEn, pcNxt, pc);

	MUX2 addrMux(IorD, pc, aluOut, addr);
	INSDATMEM insDatMem(clk, memWrite, addr, valB, readData);
	FLOPREN insReg(clk, iRWrite, readData, instr);
	FLOPR datReg(clk, readData, data);

	MUX2 #(5) RtorRd(regDst, instr[20:16], instr[15:11], writeRegAddr);
	MUX2 wd3Mux(memToReg, aluOut, data, writeRegDat);
	REGFILE regFile(clk, rst, regWrite, writeRegAddr, writeRegDat, instr[25:21], instr[20:16], rsDat, rtDat);
	SIGNEXT sigExt(instr[15:0], sigImm);
	SHIFT immSh(sigImm, sigImmSh);
	FLOPR2 flopr2(clk, rsDat, rtDat, valA, valB);

	MUX2 srcAMux(aluSrcA, pc, valA, srcA);
	MUX4 srcBMux(aluSrcB, valB, 4, sigImm, sigImmSh, srcB);
	ALU alu(aluCtrl, srcA, srcB, aluRes, zero);
	FLOPR aluReg(clk, aluRes, aluOut);
	MUX4 aluMux(pcSrc, aluRes, aluOut, {instr[31:28], instr[25:0], 2'b00}, 32'bx, pcNxt);

	assign pcEn = (branch0 & zero) | pcWrite | (branch1 & ~zero);

	assign pcOut = pc; 
	assign instrOut = instr; 
	assign readDataOut = readData;
	assign rsDatOut = rsDat; 
	assign rtDatOut = rtDat;
	assign srcAOut = srcA;
	assign srcBOut = srcB;
	assign aluResOut = aluRes;

endmodule	

module MIPS(
	input logic clk, rst,

	output logic [wid - 1:0] pcOut, readDataOut,
	output logic [wid - 1:0] rsDatOut, rtDatOut,
	output logic [wid - 1:0] srcAOut, srcBOut, aluResOut,

	output logic [5:0] opOut
	);

	logic [wid - 1:0] instrOut;

	logic [wid - 1:0] pc;
	logic [wid - 1:0] instr;
	logic pcWrite, memWrite, iRWrite, regWrite;
	logic aluSrcA, branch0, branch1, IorD, memToReg, regDst;
	logic [1:0] aluSrcB, pcSrc;
	logic [2:0] aluCtrl;

	CONTROLLER controller(	clk, rst, instrOut[31:26], instrOut[5:0],
							pcWrite, memWrite, iRWrite, regWrite,
							aluSrcA, branch0, branch1, IorD, memToReg, regDst,
							aluSrcB, pcSrc,
							aluCtrl,
							opOut
							);

	DATAPATH datapath(	clk, rst,
						pcWrite, memWrite, iRWrite, regWrite,
						aluSrcA, branch0, branch1, IorD, memToReg, regDst,
						aluSrcB, pcSrc,
						aluCtrl,

						pcOut, instrOut, readDataOut,
						rsDatOut, rtDatOut,
						srcAOut, srcBOut, aluResOut
						);

endmodule

module TOP(
	input logic clk,
	input logic rst,

	output logic [7:0] pcShow, aluResShow, rsDatShow, rtDatShow,
	output logic [4:0] num
	);

	logic [wid - 1:0] pcOut, readDataOut;
	logic [wid - 1:0] rsDatOut, rtDatOut;
	logic [wid - 1:0] srcAOut, srcBOut, aluResOut;
	logic [5:0] opOut;

	MIPS mips( clk, rst,
				pcOut, readDataOut,
				rsDatOut, rtDatOut,
				srcAOut, srcBOut, aluResOut,
				opOut
			);

	assign pcShow = pcOut[9:2];
	assign aluResShow = aluResOut[7:0];
	assign rsDatShow = rsDatOut[7:0];
	assign rtDatShow = rtDatOut[7:0];

	always_comb
	begin
		case(opOut)
			LW:		num = 5'b11111;
			SW:		num = 5'b11111;
			RTYPE:	num = 5'b01111;
			BEQ:	num = 5'b00111;
			BNE:	num = 5'b00111;
			ADDI:	num = 5'b01111;
			ANDI:	num = 5'b01111;
			ORI:	num = 5'b01111;
			SLTI:	num = 5'b01111;
			J:		num = 5'b00111;
			default:
					num = 5'b00000;
		endcase
	end

endmodule

// test
module clkDiv(
	input logic clk,

	output logic clk190,
	output logic clk48,
	output logic clk1_4
	);

	logic [27:0] q;

	initial
	begin
		q = 0;
	end

	always_ff @(posedge clk)
	begin
		q <= q + 1;
	end

	assign clk190 = q[18];
	assign clk48 = q[20];
	assign clk1_4 = q[26];

endmodule

module forShow(
	input logic clk, rst,
	input logic k,

	output logic [6:0] a2g,
	output logic [7:0] enA2g,
	output logic [4:0] led
	);

	logic clk190, clk48, clk1_4;

	clkDiv div(clk, clk190, clk48, clk1_4);

	logic [31:0] sw; 

	assign clk1_4_k = clk1_4 & k;

	TOP top(clk1_4_k, rst, 
			sw[31:24], sw[23:16], sw[15:8], sw[7:0], led[4:0]);

	logic [2:0] i;
	logic [20:0] fre;

	initial
	begin
		i = 0;
		fre = 0;
	end

	always_ff @(posedge clk)
	begin
		fre <= fre + 1;
	end

	assign i = fre[19:17];

	always_comb
	begin
		if (i == 3'b000)
		begin
			enA2g = 8'b1111_1110;
			case (sw[3:0])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b001)
		begin
			enA2g = 8'b1111_1101;
			case (sw[7:4])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b010)
		begin
			enA2g = 8'b1111_1011;
			case (sw[11:8])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b011)
		begin
			enA2g = 8'b1111_0111;
			case (sw[15:12])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b100)
		begin
			enA2g = 8'b1110_1111;
			case (sw[19:16])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b101)
		begin
			enA2g = 8'b1101_1111;
			case (sw[23:20])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b110)
		begin
			enA2g = 8'b1011_1111;
			case (sw[27:24])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b111)
		begin
			enA2g = 8'b0111_1111;
			case (sw[31:28])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
	end

endmodule

module forShowTop(
	input logic CLK100MHZ, SW[1:0],

	output logic [6:0] A2G,
	output logic [7:0] AN,
	output logic [4:0] LED
	);

	forShow show(CLK100MHZ, SW[0], SW[1], A2G[6:0], AN[7:0], LED[4:0]);

endmodule
