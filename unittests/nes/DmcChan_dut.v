module DmcChan_ut (
	 clk,
	 ce,
	 reset,
	 odd_or_even,
	 Addr,
	 DIN,
	 MW,
	 Sample,
	 DmaReq,
	 DmaAck,
	 DmaAddr,
	 DmaData,
	 Irq,
	 IsDmcActive
);

// Inputs and Registers
input   clk;
reg  clk_reg;
input   ce;
reg  ce_reg;
input   reset;
reg  reset_reg;
input   odd_or_even;
reg  odd_or_even_reg;
input [2:0]  Addr;
reg[2:0]  Addr_reg;
input [7:0]  DIN;
reg[7:0]  DIN_reg;
input   MW;
reg  MW_reg;
input   DmaAck;
reg  DmaAck_reg;
input [7:0]  DmaData;
reg[7:0]  DmaData_reg;

// Outputs, Registers & Wires
output [6:0]  Sample;
reg[6:0]  Sample_reg;
wire[6:0]  Sample_wire;
output   DmaReq;
reg  DmaReq_reg;
wire  DmaReq_wire;
output [15:0]  DmaAddr;
reg[15:0]  DmaAddr_reg;
wire[15:0]  DmaAddr_wire;
output   Irq;
reg  Irq_reg;
wire  Irq_wire;
output   IsDmcActive;
reg  IsDmcActive_reg;
wire  IsDmcActive_wire;

// Assigning outputs.
assign  Sample =  Sample_reg;
assign  DmaReq =  DmaReq_reg;
assign  DmaAddr =  DmaAddr_reg;
assign  Irq =  Irq_reg;
assign  IsDmcActive =  IsDmcActive_reg;
DmcChan inst1(
	 . clk( clk_reg),
	 . ce( ce_reg),
	 . reset( reset_reg),
	 . odd_or_even( odd_or_even_reg),
	 . Addr( Addr_reg),
	 . DIN( DIN_reg),
	 . MW( MW_reg),
	 . DmaAck( DmaAck_reg),
	 . DmaData( DmaData_reg),
	 . Sample( Sample_wire),
	 . DmaReq( DmaReq_wire),
	 . DmaAddr( DmaAddr_wire),
	 . Irq( Irq_wire),
	 . IsDmcActive( IsDmcActive_wire)
);

endmodule
