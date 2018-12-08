module MemoryController_ut (
	 clk,
	 read_a,
	 read_b,
	 write,
	 addr,
	 din,
	 dout_a,
	 dout_b,
	 busy,
	 MemOE,
	 MemWR,
	 MemAdv,
	 MemClk,
	 RamCS,
	 RamCRE,
	 RamUB,
	 RamLB,
	 MemAdr,
	 MemDB
);

// Inputs and Registers
input   clk;
reg  clk_reg;
input   read_a;
reg  read_a_reg;
input   read_b;
reg  read_b_reg;
input   write;
reg  write_reg;
input [23:0]  addr;
reg[23:0]  addr_reg;
input [7:0]  din;
reg[7:0]  din_reg;

// Outputs, Registers & Wires
output [7:0]  dout_a;
reg[7:0]  dout_a_reg;
wire[7:0]  dout_a_wire;
output [7:0]  dout_b;
reg[7:0]  dout_b_reg;
wire[7:0]  dout_b_wire;
output   busy;
reg  busy_reg;
wire  busy_wire;
output   MemOE;
reg  MemOE_reg;
wire  MemOE_wire;
output   MemWR;
reg  MemWR_reg;
wire  MemWR_wire;
output   MemAdv;
reg  MemAdv_reg;
wire  MemAdv_wire;
output   MemClk;
reg  MemClk_reg;
wire  MemClk_wire;
output   RamCS;
reg  RamCS_reg;
wire  RamCS_wire;
output   RamCRE;
reg  RamCRE_reg;
wire  RamCRE_wire;
output   RamUB;
reg  RamUB_reg;
wire  RamUB_wire;
output   RamLB;
reg  RamLB_reg;
wire  RamLB_wire;
output [22:0]  MemAdr;
reg[22:0]  MemAdr_reg;
wire[22:0]  MemAdr_wire;

// Assigning outputs.
assign  dout_a =  dout_a_reg;
assign  dout_b =  dout_b_reg;
assign  busy =  busy_reg;
assign  MemOE =  MemOE_reg;
assign  MemWR =  MemWR_reg;
assign  MemAdv =  MemAdv_reg;
assign  MemClk =  MemClk_reg;
assign  RamCS =  RamCS_reg;
assign  RamCRE =  RamCRE_reg;
assign  RamUB =  RamUB_reg;
assign  RamLB =  RamLB_reg;
assign  MemAdr =  MemAdr_reg;
MemoryController inst1(
	 . clk( clk_reg),
	 . read_a( read_a_reg),
	 . read_b( read_b_reg),
	 . write( write_reg),
	 . addr( addr_reg),
	 . din( din_reg),
	 . dout_a( dout_a_wire),
	 . dout_b( dout_b_wire),
	 . busy( busy_wire),
	 . MemOE( MemOE_wire),
	 . MemWR( MemWR_wire),
	 . MemAdv( MemAdv_wire),
	 . MemClk( MemClk_wire),
	 . RamCS( RamCS_wire),
	 . RamCRE( RamCRE_wire),
	 . RamUB( RamUB_wire),
	 . RamLB( RamLB_wire),
	 . MemAdr( MemAdr_wire),
);

endmodule
