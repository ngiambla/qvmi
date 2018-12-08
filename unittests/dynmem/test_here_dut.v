module test_here_ut (
	 clk,
	in,
	out
);

// Inputs and Registers
input   clk;
reg  clk_reg;
input  in;
reg in_reg;

// Outputs, Registers & Wires
output [2:0] out;
reg[2:0] out_reg;
wire[2:0] out_wire;

// Assigning outputs.
assign out = out_reg;
test_here inst1(
	 . clk( clk_reg),
	 .in(in_reg),
	 .out(out_wire)
);

endmodule
