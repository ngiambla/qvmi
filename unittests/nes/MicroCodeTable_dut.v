module MicroCodeTable_ut (
	 clk,
	 ce,
	 reset,
	 IR,
	 State,
	 Mout
);

// Inputs and Registers
input   clk;
reg  clk_reg;
input   ce;
reg  ce_reg;
input   reset;
reg  reset_reg;
input [7:0]  IR;
reg[7:0]  IR_reg;
input [2:0]  State;
reg[2:0]  State_reg;

// Outputs, Registers & Wires
output [37:0]  Mout;
reg[37:0]  Mout_reg;
wire[37:0]  Mout_wire;

// Assigning outputs.
assign  Mout =  Mout_reg;
MicroCodeTable inst1(
	 . clk( clk_reg),
	 . ce( ce_reg),
	 . reset( reset_reg),
	 . IR( IR_reg),
	 . State( State_reg),
	 . Mout( Mout_wire)
);

endmodule
