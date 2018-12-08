module GameLoader_ut (
	 clk,
	 reset,
	 indata,
	 indata_clk,
	 mem_addr,
	 mem_data,
	 mem_write,
	 mapper_flags,
	 done,
	 error
);

// Inputs and Registers
input   clk;
reg  clk_reg;
input   reset;
reg  reset_reg;
input [7:0]  indata;
reg[7:0]  indata_reg;
input   indata_clk;
reg  indata_clk_reg;

// Outputs, Registers & Wires
output [21:0]  mem_addr;
reg[21:0]  mem_addr_reg;
wire[21:0]  mem_addr_wire;
output [7:0]  mem_data;
reg[7:0]  mem_data_reg;
wire[7:0]  mem_data_wire;
output   mem_write;
reg  mem_write_reg;
wire  mem_write_wire;
output [31:0]  mapper_flags;
reg[31:0]  mapper_flags_reg;
wire[31:0]  mapper_flags_wire;
output   done;
reg  done_reg;
wire  done_wire;
output   error;
reg  error_reg;
wire  error_wire;

// Assigning outputs.
assign  mem_addr =  mem_addr_reg;
assign  mem_data =  mem_data_reg;
assign  mem_write =  mem_write_reg;
assign  mapper_flags =  mapper_flags_reg;
assign  done =  done_reg;
assign  error =  error_reg;
GameLoader inst1(
	 . clk( clk_reg),
	 . reset( reset_reg),
	 . indata( indata_reg),
	 . indata_clk( indata_clk_reg),
	 . mem_addr( mem_addr_wire),
	 . mem_data( mem_data_wire),
	 . mem_write( mem_write_wire),
	 . mapper_flags( mapper_flags_wire),
	 . done( done_wire),
	 . error( error_wire)
);

endmodule
