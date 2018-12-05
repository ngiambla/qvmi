module mux4to1 (in, sel, out);
	input [3:0] in;
	input [1:0] sel;
	output out;
	
	assign out = (in[0] & ~sel[1] & ~sel[0]) | (in[1] & ~sel[1] & sel[0]) | (in[2] & sel[1] & ~sel[0]) | (in[3] & sel[1] & sel[0]);
endmodule

module mux8to1(in, sel, out);
	input [7:0] in;
	input [2:0] sel;
	output out;
	
	wire out0, out1;

	mux4to1 m4to1_0 (in[3:0], sel[1:0], out0);
	mux4to1 m4to1_1 (in[7:4], sel[1:0], out1);

	assign out = (out0 & ~sel[2]) | (out1 & sel[2]); 
endmodule

module mux16to1 (in, sel, out);
	input [15:0] in;
	input [3:0] sel;
	output out;

	wire out0, out1;

	mux8to1 m8to1_0 (in[7:0], sel[2:0], out0);
	mux8to1 m8to1_1 (in[15:8], sel[2:0], out1);

	assign out = (out0 & ~sel[3]) | (out1 & sel[3]);	
endmodule

module mux32to1 (in, sel, out);
	input [31:0] in;
	input [4:0] sel;
	output out;

	wire out0, out1;

	mux16to1 m16to1_0 (in[15:0], sel[3:0], out0);
	mux16to1 m16to1_1 (in[31:16], sel[3:0], out1);

	assign out = (out0 & ~sel[4]) | (out1 & sel[4]);
endmodule

module mux64to1 (in, sel, out);
	input [63:0] in;
	input [5:0] sel;
	output out;

	wire out0, out1;
	
	mux32to1 m32to1_0 (in[31:0], sel[4:0], out0);
	mux32to1 m32to1_1 (in[63:32], sel[4:0], out1);

	assign out = (out0 & ~sel[5]) | (out1 & sel[5]);
endmodule

module mux128to1 (in, sel, out);
	input [127:0] in;
	input [6:0] sel;
	output out;

	wire out0, out1;

	mux64to1 m64to1_0 (in[63:0], sel[5:0], out0);
	mux64to1 m64to1_1 (in[127:64], sel[5:0], out1);

	assign out = (out0 & ~sel[6]) | (out1 & sel[6]);
endmodule

module mux256to1 (in, sel, out);
	input [255:0] in;
	input [7:0] sel;
	output out;

	wire out0, out1;

	mux128to1 m128to1_0 (in[127:0], sel[6:0], out0);
	mux128to1 m128to1_1 (in[255:128], sel[6:0], out1);

	assign out = (out0 & ~sel[7]) | (out1 & sel[7]);
endmodule

module mux512to1 (in, sel, out);
	input [511:0] in;
	input [8:0] sel;
	output out;

	wire out0, out1;

	mux256to1 m256to1_0 (in[255:0], sel[7:0], out0);
	mux256to1 m256to1_1 (in[511:256], sel[7:0], out1);

	assign out = (out0 & ~sel[8]) | (out1 & sel[8]);

endmodule

module variableMux #(parameter INWIDTH=512, parameter SELWIDTH=9) (din, sel, qout);
	input [INWIDTH-1:0] din;
	input [SELWIDTH-1:0] sel;
	output qout;
	
	mux512to1 m0 (din[INWIDTH-1:0], sel[SELWIDTH-1:0], qout);
endmodule

module mux512 (din, sel, clk, qout);
	parameter INWIDTH = 512, SELWIDTH = 9;
	
	input [INWIDTH-1:0] din;
	input [SELWIDTH-1:0] sel;
	input clk;
	output reg qout;
	
	reg [INWIDTH-1:0] din_reg;
	wire [INWIDTH-1:0] mux_in;
	wire [SELWIDTH-1:0] sel_in;
	wire mux_out;
	
	// Pipeline registers
	reg qreg0, qreg1, qreg2, qreg3, qreg4, qreg5, qreg6 /* synthesis dont_merge */;
	reg [INWIDTH-1:0] din_pipeline_reg0, din_pipeline_reg1, din_pipeline_reg2, din_pipeline_reg3, din_pipeline_reg4, din_pipeline_reg5, din_pipeline_reg6 /* synthesis preserve_syn_only */;
	reg [SELWIDTH-1:0] sel_reg, sel_pipeline_reg0, sel_pipeline_reg1, sel_pipeline_reg2, sel_pipeline_reg3, sel_pipeline_reg4, sel_pipeline_reg5, sel_pipeline_reg6 /* synthesis preserve_syn_only */;
	
	assign mux_in = din_reg;
	assign sel_in = sel_reg;

	mux512to1 # (
		.INWIDTH(512),
		.SELWIDTH(9)
	) mux512to1_0 (
		.din(mux_in),
		.sel(sel_reg),
		.qout(mux_out)
	);
	
	always @ (posedge clk)
	begin
		qout <= mux_out;
	
		// Add in sel pipeline registers
		sel_pipeline_reg0 <= sel;
		sel_pipeline_reg1 <= sel_pipeline_reg0;
		sel_pipeline_reg2 <= sel_pipeline_reg1;
		sel_pipeline_reg3 <= sel_pipeline_reg2;
		sel_pipeline_reg4 <= sel_pipeline_reg3;
		sel_pipeline_reg5 <= sel_pipeline_reg4;
		sel_pipeline_reg6 <= sel_pipeline_reg5;
		sel_reg <= sel_pipeline_reg6;

		// Add in din pipeline registers
		din_pipeline_reg0 <= din;
		din_pipeline_reg1 <= din_pipeline_reg0;
		din_pipeline_reg2 <= din_pipeline_reg1;
		din_pipeline_reg3 <= din_pipeline_reg2;
		din_pipeline_reg4 <= din_pipeline_reg3;
		din_pipeline_reg5 <= din_pipeline_reg4;
		din_pipeline_reg6 <= din_pipeline_reg5;
		din_reg <= din_pipeline_reg6;
	end
endmodule

module muxtop (clk, en, rst, din, sel, qout);
	parameter IPIPE = 1;
	parameter OPIPE = 1;
	input clk;
	input en;
	input rst;
	input [511:0]din;
	input [8:0]sel;
	output qout;

	wire [511:0] din_mux;
	wire [8:0] sel_mux;
	wire qout_mux;
		
	hyper_pipe #(512, IPIPE) pipe_input (clk, din, din_mux);	
	hyper_pipe #(9, IPIPE) pipe_sel (clk, sel, sel_mux);	
	
	wire [511:0] din_mux_wire;
	wire [8:0] sel_mux_wire;

	wire [520:0] hyper_in;
	wire [520:0] hyper_out;
	
	assign hyper_in = {sel_mux, din_mux};
	assign sel_mux_wire = hyper_out [520:512];
	assign din_mux_wire = hyper_out [511:0];

	hyperpipe_vlat #(521,100) hyperpipe0 (clk, hyper_in, hyper_out);

	mux512to1 mux512to1_0 (din_mux_wire, sel_mux_wire, qout_mux);
	
	hyper_pipe #(1, OPIPE) pipe_output (clk, qout_mux, qout);

endmodule

