module APU_ut (
	 clk,
	 ce,
	 reset,
	 ADDR,
	 DIN,
	 DOUT,
	 MW,
	 MR,
	 audio_channels,
	 Sample,
	 DmaReq,
	 DmaAck,
	 DmaAddr,
	 DmaData,
	 odd_or_even,
	 IRQ
);

// Inputs and Registers
input   clk;
reg  clk_reg;
input   ce;
reg  ce_reg;
input   reset;
reg  reset_reg;
input [4:0]  ADDR;
reg[4:0]  ADDR_reg;
input [7:0]  DIN;
reg[7:0]  DIN_reg;
input   MW;
reg  MW_reg;
input   MR;
reg  MR_reg;
input [4:0]  audio_channels;
reg[4:0]  audio_channels_reg;
input   DmaAck;
reg  DmaAck_reg;
input [7:0]  DmaData;
reg[7:0]  DmaData_reg;

// Outputs, Registers & Wires
output [7:0]  DOUT;
reg[7:0]  DOUT_reg;
wire[7:0]  DOUT_wire;
output [15:0]  Sample;
reg[15:0]  Sample_reg;
wire[15:0]  Sample_wire;
output   DmaReq;
reg  DmaReq_reg;
wire  DmaReq_wire;
output [15:0]  DmaAddr;
reg[15:0]  DmaAddr_reg;
wire[15:0]  DmaAddr_wire;
output   odd_or_even;
reg  odd_or_even_reg;
wire  odd_or_even_wire;
output   IRQ;
reg  IRQ_reg;
wire  IRQ_wire;

// Assigning outputs.
assign  DOUT =  DOUT_reg;
assign  Sample =  Sample_reg;
assign  DmaReq =  DmaReq_reg;
assign  DmaAddr =  DmaAddr_reg;
assign  odd_or_even =  odd_or_even_reg;
assign  IRQ =  IRQ_reg;
APU inst1(
	 . clk( clk_reg),
	 . ce( ce_reg),
	 . reset( reset_reg),
	 . ADDR( ADDR_reg),
	 . DIN( DIN_reg),
	 . MW( MW_reg),
	 . MR( MR_reg),
	 . audio_channels( audio_channels_reg),
	 . DmaAck( DmaAck_reg),
	 . DmaData( DmaData_reg),
	 . DOUT( DOUT_wire),
	 . Sample( Sample_wire),
	 . DmaReq( DmaReq_wire),
	 . DmaAddr( DmaAddr_wire),
	 . odd_or_even( odd_or_even_wire),
	 . IRQ( IRQ_wire)
);

endmodule
