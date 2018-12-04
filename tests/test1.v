//----------------------------------------------------------------------------
// LegUp High-Level Synthesis Tool Version 5.0 (http://legup.org)
// Compiled: Thu Nov 15 17:38:53 2018
// Copyright (c) 2009-16 University of Toronto. All Rights Reserved.
// For research and academic purposes only. Commercial use is prohibited.
// Please report bugs at: http://legup.org/bugs
// Please email questions to: legup@eecg.toronto.edu
// Date: Fri Nov 30 14:38:38 2018
// ----------------------------------------------------------------------------
`define MEMORY_CONTROLLER_ADDR_SIZE 32
`define MEMORY_CONTROLLER_DATA_SIZE 64
// Number of RAM elements: 14
`define MEMORY_CONTROLLER_TAG_SIZE 9
// @N_FAST_ARENA_0_32 = internal global [256 x i32] zeroinitializer, align 4
`define TAG_g_N_FAST_ARENA_0_32 `MEMORY_CONTROLLER_TAG_SIZE'd2
`define TAG_g_N_FAST_ARENA_0_32_a {`TAG_g_N_FAST_ARENA_0_32, 23'd0}
// @N_FAST_ARENA_1024 = internal global [8192 x i32] zeroinitializer, align 4
`define TAG_g_N_FAST_ARENA_1024 `MEMORY_CONTROLLER_TAG_SIZE'd11
`define TAG_g_N_FAST_ARENA_1024_a {`TAG_g_N_FAST_ARENA_1024, 23'd0}
// @N_FAST_ARENA_1_32 = internal global [256 x i32] zeroinitializer, align 4
`define TAG_g_N_FAST_ARENA_1_32 `MEMORY_CONTROLLER_TAG_SIZE'd3
`define TAG_g_N_FAST_ARENA_1_32_a {`TAG_g_N_FAST_ARENA_1_32, 23'd0}
// @N_FAST_ARENA_2_32 = internal global [256 x i32] zeroinitializer, align 4
`define TAG_g_N_FAST_ARENA_2_32 `MEMORY_CONTROLLER_TAG_SIZE'd4
`define TAG_g_N_FAST_ARENA_2_32_a {`TAG_g_N_FAST_ARENA_2_32, 23'd0}
// @N_FAST_ARENA_3_32 = internal global [256 x i32] zeroinitializer, align 4
`define TAG_g_N_FAST_ARENA_3_32 `MEMORY_CONTROLLER_TAG_SIZE'd5
`define TAG_g_N_FAST_ARENA_3_32_a {`TAG_g_N_FAST_ARENA_3_32, 23'd0}
// @N_FAST_ARENA_4_32 = internal global [256 x i32] zeroinitializer, align 4
`define TAG_g_N_FAST_ARENA_4_32 `MEMORY_CONTROLLER_TAG_SIZE'd6
`define TAG_g_N_FAST_ARENA_4_32_a {`TAG_g_N_FAST_ARENA_4_32, 23'd0}
// @N_FAST_ARENA__128 = internal global [1024 x i32] zeroinitializer, align 4
`define TAG_g_N_FAST_ARENA__128 `MEMORY_CONTROLLER_TAG_SIZE'd8
`define TAG_g_N_FAST_ARENA__128_a {`TAG_g_N_FAST_ARENA__128, 23'd0}
// @N_FAST_ARENA__256 = internal global [2048 x i32] zeroinitializer, align 4
`define TAG_g_N_FAST_ARENA__256 `MEMORY_CONTROLLER_TAG_SIZE'd9
`define TAG_g_N_FAST_ARENA__256_a {`TAG_g_N_FAST_ARENA__256, 23'd0}
// @N_FAST_ARENA__512 = internal global [4096 x i32] zeroinitializer, align 4
`define TAG_g_N_FAST_ARENA__512 `MEMORY_CONTROLLER_TAG_SIZE'd10
`define TAG_g_N_FAST_ARENA__512_a {`TAG_g_N_FAST_ARENA__512, 23'd0}
// @N_FAST_ARENA___64 = internal global [512 x i32] zeroinitializer, align 4
`define TAG_g_N_FAST_ARENA___64 `MEMORY_CONTROLLER_TAG_SIZE'd7
`define TAG_g_N_FAST_ARENA___64_a {`TAG_g_N_FAST_ARENA___64, 23'd0}
`timescale 1 ns / 1 ns

module test_here(input clk, in, output [2:0]out);
	always@(posedge clk)
		begin
			out <= in%2;
		end
endmodule

module nmalloc
(
	clk,
	clk2x,
	clk1x_follower,
	reset,
	memory_controller_waitrequest,
	start,
	finish,
	return_val,
	arg_bytes,
	N_FREE_ADDRESS___LT_write_enable_a,
	N_FREE_ADDRESS___LT_in_a,
	N_FREE_ADDRESS___LT_byteena_a,
	N_FREE_ADDRESS___LT_enable_a,
	N_FREE_ADDRESS___LT_address_a,
	N_FREE_ADDRESS___LT_out_a,
	N_FREE_ADDRESS___LT_write_enable_b,
	N_FREE_ADDRESS___LT_in_b,
	N_FREE_ADDRESS___LT_byteena_b,
	N_FREE_ADDRESS___LT_enable_b,
	N_FREE_ADDRESS___LT_address_b,
	N_FREE_ADDRESS___LT_out_b
);

parameter [5:0] LEGUP_0 = 6'd0;
parameter [5:0] LEGUP_F_nmalloc_BB__0_1 = 6'd1;
parameter [5:0] LEGUP_F_nmalloc_BB__3_2 = 6'd2;
parameter [5:0] LEGUP_F_nmalloc_BB__6_3 = 6'd3;
parameter [5:0] LEGUP_F_nmalloc_BB__6_4 = 6'd4;
parameter [5:0] LEGUP_F_nmalloc_BB__11_5 = 6'd5;
parameter [5:0] LEGUP_F_nmalloc_BB__11_6 = 6'd6;
parameter [5:0] LEGUP_F_nmalloc_BB__16_7 = 6'd7;
parameter [5:0] LEGUP_F_nmalloc_BB__19_8 = 6'd8;
parameter [5:0] LEGUP_F_nmalloc_BB__19_9 = 6'd9;
parameter [5:0] LEGUP_F_nmalloc_BB__24_10 = 6'd10;
parameter [5:0] LEGUP_F_nmalloc_BB__24_11 = 6'd11;
parameter [5:0] LEGUP_F_nmalloc_BB_log2exit_12 = 6'd12;
parameter [5:0] LEGUP_F_nmalloc_BB_log2exit_13 = 6'd13;
parameter [5:0] LEGUP_F_nmalloc_BB_log2exit_14 = 6'd14;
parameter [5:0] LEGUP_F_nmalloc_BB_log2exit_15 = 6'd15;
parameter [5:0] LEGUP_F_nmalloc_BB_log2exit_16 = 6'd16;
parameter [5:0] LEGUP_F_nmalloc_BB_preheaderpreheader_17 = 6'd17;
parameter [5:0] LEGUP_F_nmalloc_BB__36_18 = 6'd18;
parameter [5:0] LEGUP_F_nmalloc_BB__36_19 = 6'd19;
parameter [5:0] LEGUP_F_nmalloc_BB__36_20 = 6'd20;
parameter [5:0] LEGUP_F_nmalloc_BB__42_21 = 6'd21;
parameter [5:0] LEGUP_F_nmalloc_BB__45_22 = 6'd22;
parameter [5:0] LEGUP_F_nmalloc_BB__45_23 = 6'd23;
parameter [5:0] LEGUP_F_nmalloc_BB__50_24 = 6'd24;
parameter [5:0] LEGUP_F_nmalloc_BB__50_25 = 6'd25;
parameter [5:0] LEGUP_F_nmalloc_BB__55_26 = 6'd26;
parameter [5:0] LEGUP_F_nmalloc_BB__58_27 = 6'd27;
parameter [5:0] LEGUP_F_nmalloc_BB__58_28 = 6'd28;
parameter [5:0] LEGUP_F_nmalloc_BB__63_29 = 6'd29;
parameter [5:0] LEGUP_F_nmalloc_BB__63_30 = 6'd30;
parameter [5:0] LEGUP_F_nmalloc_BB_log2exit7_31 = 6'd31;
parameter [5:0] LEGUP_F_nmalloc_BB_log2exit7_32 = 6'd32;
parameter [5:0] LEGUP_F_nmalloc_BB__71_33 = 6'd33;

input  clk;
input  clk2x;
input  clk1x_follower;
input  reset;
input  memory_controller_waitrequest;
input  start;
output reg  finish;
output reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] return_val;
input [31:0] arg_bytes;
output reg  N_FREE_ADDRESS___LT_write_enable_a;
output reg [31:0] N_FREE_ADDRESS___LT_in_a;
output  N_FREE_ADDRESS___LT_byteena_a;
output reg  N_FREE_ADDRESS___LT_enable_a;
output reg [3:0] N_FREE_ADDRESS___LT_address_a;
input [31:0] N_FREE_ADDRESS___LT_out_a;
output  N_FREE_ADDRESS___LT_write_enable_b;
output [31:0] N_FREE_ADDRESS___LT_in_b;
output  N_FREE_ADDRESS___LT_byteena_b;
output  N_FREE_ADDRESS___LT_enable_b;
output [3:0] N_FREE_ADDRESS___LT_address_b;
input [31:0] N_FREE_ADDRESS___LT_out_b;
reg [5:0] cur_state;
reg [5:0] next_state;
reg [31:0] arg_bytes_reg;
reg  fsm_stall;
reg [15:0] nmalloc_0_1;
reg [15:0] nmalloc_0_1_reg;
reg  nmalloc_0_2;
reg [7:0] nmalloc_3_4;
reg [7:0] nmalloc_3_4_reg;
reg  nmalloc_3_5;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_6_7;
reg [7:0] nmalloc_6_8;
reg [7:0] nmalloc_6_9;
reg [7:0] nmalloc_6_10;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_11_12;
reg [7:0] nmalloc_11_13;
reg [7:0] nmalloc_11_14;
reg [7:0] nmalloc_11_15;
reg [23:0] nmalloc_16_17;
reg [23:0] nmalloc_16_17_reg;
reg  nmalloc_16_18;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_19_20;
reg [7:0] nmalloc_19_21;
reg [7:0] nmalloc_19_22;
reg [7:0] nmalloc_19_23;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_24_25;
reg [7:0] nmalloc_24_26;
reg [7:0] nmalloc_24_27;
reg [7:0] nmalloc_log2exit_0i;
reg [7:0] nmalloc_log2exit_0i_reg;
reg [31:0] nmalloc_log2exit_28;
reg [31:0] nmalloc_log2exit_28_reg;
reg  nmalloc_log2exit_29;
reg  nmalloc_log2exit_30;
reg  nmalloc_log2exit_30_reg;
reg [8:0] nmalloc_log2exit_31;
reg [8:0] nmalloc_log2exit_31_reg;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_log2exit_32;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_log2exit_32_reg;
reg [31:0] nmalloc_log2exit_33;
reg [31:0] nmalloc_log2exit_33_reg;
reg  nmalloc_log2exit_34;
reg [31:0] nmalloc_36_37;
reg [31:0] nmalloc_36_37_reg;
reg [31:0] nmalloc_36_38;
reg [31:0] nmalloc_36_38_reg;
reg [31:0] nmalloc_36_39;
reg [31:0] nmalloc_36_39_reg;
reg [15:0] nmalloc_36_40;
reg [15:0] nmalloc_36_40_reg;
reg  nmalloc_36_41;
reg [7:0] nmalloc_42_43;
reg [7:0] nmalloc_42_43_reg;
reg  nmalloc_42_44;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_45_46;
reg [7:0] nmalloc_45_47;
reg [7:0] nmalloc_45_48;
reg [7:0] nmalloc_45_49;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_50_51;
reg [7:0] nmalloc_50_52;
reg [7:0] nmalloc_50_53;
reg [7:0] nmalloc_50_54;
reg [23:0] nmalloc_55_56;
reg [23:0] nmalloc_55_56_reg;
reg  nmalloc_55_57;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_58_59;
reg [7:0] nmalloc_58_60;
reg [7:0] nmalloc_58_61;
reg [7:0] nmalloc_58_62;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_63_64;
reg [7:0] nmalloc_63_65;
reg [7:0] nmalloc_63_66;
reg [7:0] nmalloc_log2exit7_0i6;
reg [7:0] nmalloc_log2exit7_0i6_reg;
reg [31:0] nmalloc_log2exit7_67;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_log2exit7_68;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_log2exit7_69;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_log2exit7_70;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_71_0;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_71_0_reg;
reg [7:0] LogTable25647_address_a;
wire [7:0] LogTable25647_out_a;
wire [7:0] LogTable25647_address_b;
wire [7:0] LogTable25647_out_b;
reg [8:0] N_FAST_ADDRESS___LT_address_a;
wire [31:0] N_FAST_ADDRESS___LT_out_a;
wire [8:0] N_FAST_ADDRESS___LT_address_b;
wire [31:0] N_FAST_ADDRESS___LT_out_b;

// Local Rams


// @LogTable25647 = internal unnamed_addr constant [256 x i8] c"\FF\00\01\01\02\02\02\02\03\03\03\03\03\03\03\03\04\04\04\04\04\04\04\04\04\04\04\04\04\04\04\04\05\05\05\05\05\05\05\05\05\05\05\05\05\05\...
rom_dual_port LogTable25647 (
	.clk( clk ),
	.clken( !memory_controller_waitrequest ),
	.address_a( LogTable25647_address_a ),
	.address_b( LogTable25647_address_b ),
	.q_a( LogTable25647_out_a ),
	.q_b( LogTable25647_out_b)
);
defparam LogTable25647.width_a = 8;
defparam LogTable25647.width_b = 8;
defparam LogTable25647.widthad_a = 8;
defparam LogTable25647.widthad_b = 8;
defparam LogTable25647.numwords_a = 256;
defparam LogTable25647.numwords_b = 256;
defparam LogTable25647.latency = 1;
defparam LogTable25647.init_file = "LogTable25647.mif";


// @N_FAST_ADDRESS___LT = internal unnamed_addr constant [10 x [32 x i32*]] [[32 x i32*] [i32* getelementptr inbounds ([256 x i32]* @N_FAST_ARENA_0_32, i32 0, i32 0), i32* bitcast (i8* getelementptr (i8*...
rom_dual_port N_FAST_ADDRESS___LT (
	.clk( clk ),
	.clken( !memory_controller_waitrequest ),
	.address_a( N_FAST_ADDRESS___LT_address_a ),
	.address_b( N_FAST_ADDRESS___LT_address_b ),
	.q_a( N_FAST_ADDRESS___LT_out_a ),
	.q_b( N_FAST_ADDRESS___LT_out_b)
);
defparam N_FAST_ADDRESS___LT.width_a = 32;
defparam N_FAST_ADDRESS___LT.width_b = 32;
defparam N_FAST_ADDRESS___LT.widthad_a = 9;
defparam N_FAST_ADDRESS___LT.widthad_b = 9;
defparam N_FAST_ADDRESS___LT.numwords_a = 320;
defparam N_FAST_ADDRESS___LT.numwords_b = 320;
defparam N_FAST_ADDRESS___LT.latency = 1;
defparam N_FAST_ADDRESS___LT.init_file = "N_FAST_ADDRESS___LT.mif";

// End Local Rams

/* Unsynthesizable Statements */
/* synthesis translate_off */
always @(posedge clk)
	if (!fsm_stall) begin
	/* nmalloc: %.preheader.preheader*/
	/*   %35 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([17 x i8]* @.str48, i32 0, i32 0)) #2, !MSB !8, !LSB !2, !extendFrom !8*/
	if ((cur_state == LEGUP_F_nmalloc_BB_preheaderpreheader_17)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("returning NULL.\n");
	end
end
/* synthesis translate_on */
always @(posedge clk) begin
if (reset == 1'b1)
	cur_state <= LEGUP_0;
else if (!fsm_stall)
	cur_state <= next_state;
end

always @(*)
begin
next_state = cur_state;
case(cur_state)  // synthesis parallel_case  
LEGUP_0:
	if ((fsm_stall == 1'd0) && (start == 1'd1))
		next_state = LEGUP_F_nmalloc_BB__0_1;
LEGUP_F_nmalloc_BB__0_1:
	if ((fsm_stall == 1'd0) && (nmalloc_0_2 == 1'd1))
		next_state = LEGUP_F_nmalloc_BB__16_7;
	else if ((fsm_stall == 1'd0) && (nmalloc_0_2 == 1'd0))
		next_state = LEGUP_F_nmalloc_BB__3_2;
LEGUP_F_nmalloc_BB__11_5:
		next_state = LEGUP_F_nmalloc_BB__11_6;
LEGUP_F_nmalloc_BB__11_6:
		next_state = LEGUP_F_nmalloc_BB_log2exit_12;
LEGUP_F_nmalloc_BB__16_7:
	if ((fsm_stall == 1'd0) && (nmalloc_16_18 == 1'd1))
		next_state = LEGUP_F_nmalloc_BB__24_10;
	else if ((fsm_stall == 1'd0) && (nmalloc_16_18 == 1'd0))
		next_state = LEGUP_F_nmalloc_BB__19_8;
LEGUP_F_nmalloc_BB__19_8:
		next_state = LEGUP_F_nmalloc_BB__19_9;
LEGUP_F_nmalloc_BB__19_9:
		next_state = LEGUP_F_nmalloc_BB_log2exit_12;
LEGUP_F_nmalloc_BB__24_10:
		next_state = LEGUP_F_nmalloc_BB__24_11;
LEGUP_F_nmalloc_BB__24_11:
		next_state = LEGUP_F_nmalloc_BB_log2exit_12;
LEGUP_F_nmalloc_BB__36_18:
		next_state = LEGUP_F_nmalloc_BB__36_19;
LEGUP_F_nmalloc_BB__36_19:
		next_state = LEGUP_F_nmalloc_BB__36_20;
LEGUP_F_nmalloc_BB__36_20:
	if ((fsm_stall == 1'd0) && (nmalloc_36_41 == 1'd1))
		next_state = LEGUP_F_nmalloc_BB__55_26;
	else if ((fsm_stall == 1'd0) && (nmalloc_36_41 == 1'd0))
		next_state = LEGUP_F_nmalloc_BB__42_21;
LEGUP_F_nmalloc_BB__3_2:
	if ((fsm_stall == 1'd0) && (nmalloc_3_5 == 1'd1))
		next_state = LEGUP_F_nmalloc_BB__11_5;
	else if ((fsm_stall == 1'd0) && (nmalloc_3_5 == 1'd0))
		next_state = LEGUP_F_nmalloc_BB__6_3;
LEGUP_F_nmalloc_BB__42_21:
	if ((fsm_stall == 1'd0) && (nmalloc_42_44 == 1'd1))
		next_state = LEGUP_F_nmalloc_BB__50_24;
	else if ((fsm_stall == 1'd0) && (nmalloc_42_44 == 1'd0))
		next_state = LEGUP_F_nmalloc_BB__45_22;
LEGUP_F_nmalloc_BB__45_22:
		next_state = LEGUP_F_nmalloc_BB__45_23;
LEGUP_F_nmalloc_BB__45_23:
		next_state = LEGUP_F_nmalloc_BB_log2exit7_31;
LEGUP_F_nmalloc_BB__50_24:
		next_state = LEGUP_F_nmalloc_BB__50_25;
LEGUP_F_nmalloc_BB__50_25:
		next_state = LEGUP_F_nmalloc_BB_log2exit7_31;
LEGUP_F_nmalloc_BB__55_26:
	if ((fsm_stall == 1'd0) && (nmalloc_55_57 == 1'd1))
		next_state = LEGUP_F_nmalloc_BB__63_29;
	else if ((fsm_stall == 1'd0) && (nmalloc_55_57 == 1'd0))
		next_state = LEGUP_F_nmalloc_BB__58_27;
LEGUP_F_nmalloc_BB__58_27:
		next_state = LEGUP_F_nmalloc_BB__58_28;
LEGUP_F_nmalloc_BB__58_28:
		next_state = LEGUP_F_nmalloc_BB_log2exit7_31;
LEGUP_F_nmalloc_BB__63_29:
		next_state = LEGUP_F_nmalloc_BB__63_30;
LEGUP_F_nmalloc_BB__63_30:
		next_state = LEGUP_F_nmalloc_BB_log2exit7_31;
LEGUP_F_nmalloc_BB__6_3:
		next_state = LEGUP_F_nmalloc_BB__6_4;
LEGUP_F_nmalloc_BB__6_4:
		next_state = LEGUP_F_nmalloc_BB_log2exit_12;
LEGUP_F_nmalloc_BB__71_33:
		next_state = LEGUP_0;
LEGUP_F_nmalloc_BB_log2exit7_31:
		next_state = LEGUP_F_nmalloc_BB_log2exit7_32;
LEGUP_F_nmalloc_BB_log2exit7_32:
		next_state = LEGUP_F_nmalloc_BB__71_33;
LEGUP_F_nmalloc_BB_log2exit_12:
		next_state = LEGUP_F_nmalloc_BB_log2exit_13;
LEGUP_F_nmalloc_BB_log2exit_13:
		next_state = LEGUP_F_nmalloc_BB_log2exit_14;
LEGUP_F_nmalloc_BB_log2exit_14:
		next_state = LEGUP_F_nmalloc_BB_log2exit_15;
LEGUP_F_nmalloc_BB_log2exit_15:
		next_state = LEGUP_F_nmalloc_BB_log2exit_16;
LEGUP_F_nmalloc_BB_log2exit_16:
	if ((fsm_stall == 1'd0) && (nmalloc_log2exit_34 == 1'd1))
		next_state = LEGUP_F_nmalloc_BB_preheaderpreheader_17;
	else if ((fsm_stall == 1'd0) && (nmalloc_log2exit_34 == 1'd0))
		next_state = LEGUP_F_nmalloc_BB__36_18;
LEGUP_F_nmalloc_BB_preheaderpreheader_17:
		next_state = LEGUP_F_nmalloc_BB__71_33;
default:
	next_state = cur_state;
endcase

end
always @(posedge clk) begin
	if (start) begin
		arg_bytes_reg <= arg_bytes;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(arg_bytes) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to arg_bytes_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	fsm_stall = 1'd0;
	if (reset) begin
		fsm_stall = 1'd0;
	end
	if (memory_controller_waitrequest) begin
		fsm_stall = 1'd1;
	end
end
always @(*) begin
	/* nmalloc: %0*/
	/*   %1 = lshr i32 %bytes, 16, !MSB !1, !LSB !2, !extendFrom !1*/
		nmalloc_0_1 = (arg_bytes_reg >>> (32'd16 % 32));
end
always @(posedge clk) begin
	/* nmalloc: %0*/
	/*   %1 = lshr i32 %bytes, 16, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_nmalloc_BB__0_1)) begin
		nmalloc_0_1_reg <= nmalloc_0_1;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_0_1) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_0_1_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %0*/
	/*   %2 = icmp eq i32 %1, 0, !MSB !2, !LSB !2, !extendFrom !2*/
		nmalloc_0_2 = (nmalloc_0_1 == 32'd0);
end
always @(*) begin
	/* nmalloc: %3*/
	/*   %4 = lshr i32 %bytes, 24, !MSB !4, !LSB !2, !extendFrom !4*/
		nmalloc_3_4 = (arg_bytes_reg >>> (32'd24 % 32));
end
always @(posedge clk) begin
	/* nmalloc: %3*/
	/*   %4 = lshr i32 %bytes, 24, !MSB !4, !LSB !2, !extendFrom !4*/
	if ((cur_state == LEGUP_F_nmalloc_BB__3_2)) begin
		nmalloc_3_4_reg <= nmalloc_3_4;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_3_4) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_3_4_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %3*/
	/*   %5 = icmp eq i32 %4, 0, !MSB !2, !LSB !2, !extendFrom !2*/
		nmalloc_3_5 = (nmalloc_3_4 == 32'd0);
end
always @(*) begin
	/* nmalloc: %6*/
	/*   %7 = getelementptr inbounds [256 x i8]* @LogTable25647, i32 0, i32 %4, !MSB !3, !LSB !2, !extendFrom !3*/
		nmalloc_6_7 = (1'd0 + (1 * {24'd0,nmalloc_3_4_reg}));
end
always @(*) begin
	/* nmalloc: %6*/
	/*   %8 = load i8* %7, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
		nmalloc_6_8 = LogTable25647_out_a;
end
always @(*) begin
	/* nmalloc: %6*/
	/*   %9 = sext i8 %8 to i32, !MSB !8, !LSB !2, !extendFrom !4*/
		nmalloc_6_9 = $signed(nmalloc_6_8);
end
always @(*) begin
	/* nmalloc: %6*/
	/*   %10 = or i32 %9, 24, !MSB !8, !LSB !2, !extendFrom !4*/
		nmalloc_6_10 = ($signed(nmalloc_6_9) | 32'd24);
end
always @(*) begin
	/* nmalloc: %11*/
	/*   %12 = getelementptr inbounds [256 x i8]* @LogTable25647, i32 0, i32 %1, !MSB !3, !LSB !2, !extendFrom !3*/
		nmalloc_11_12 = (1'd0 + (1 * {16'd0,nmalloc_0_1_reg}));
end
always @(*) begin
	/* nmalloc: %11*/
	/*   %13 = load i8* %12, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
		nmalloc_11_13 = LogTable25647_out_a;
end
always @(*) begin
	/* nmalloc: %11*/
	/*   %14 = sext i8 %13 to i32, !MSB !8, !LSB !2, !extendFrom !4*/
		nmalloc_11_14 = $signed(nmalloc_11_13);
end
always @(*) begin
	/* nmalloc: %11*/
	/*   %15 = or i32 %14, 16, !MSB !8, !LSB !2, !extendFrom !4*/
		nmalloc_11_15 = ($signed(nmalloc_11_14) | 32'd16);
end
always @(*) begin
	/* nmalloc: %16*/
	/*   %17 = lshr i32 %bytes, 8, !MSB !9, !LSB !2, !extendFrom !9*/
		nmalloc_16_17 = (arg_bytes_reg >>> (32'd8 % 32));
end
always @(posedge clk) begin
	/* nmalloc: %16*/
	/*   %17 = lshr i32 %bytes, 8, !MSB !9, !LSB !2, !extendFrom !9*/
	if ((cur_state == LEGUP_F_nmalloc_BB__16_7)) begin
		nmalloc_16_17_reg <= nmalloc_16_17;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_16_17) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_16_17_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %16*/
	/*   %18 = icmp eq i32 %17, 0, !MSB !2, !LSB !2, !extendFrom !2*/
		nmalloc_16_18 = (nmalloc_16_17 == 32'd0);
end
always @(*) begin
	/* nmalloc: %19*/
	/*   %20 = getelementptr inbounds [256 x i8]* @LogTable25647, i32 0, i32 %17, !MSB !3, !LSB !2, !extendFrom !3*/
		nmalloc_19_20 = (1'd0 + (1 * {8'd0,nmalloc_16_17_reg}));
end
always @(*) begin
	/* nmalloc: %19*/
	/*   %21 = load i8* %20, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
		nmalloc_19_21 = LogTable25647_out_a;
end
always @(*) begin
	/* nmalloc: %19*/
	/*   %22 = sext i8 %21 to i32, !MSB !8, !LSB !2, !extendFrom !4*/
		nmalloc_19_22 = $signed(nmalloc_19_21);
end
always @(*) begin
	/* nmalloc: %19*/
	/*   %23 = or i32 %22, 8, !MSB !8, !LSB !2, !extendFrom !4*/
		nmalloc_19_23 = ($signed(nmalloc_19_22) | 32'd8);
end
always @(*) begin
	/* nmalloc: %24*/
	/*   %25 = getelementptr inbounds [256 x i8]* @LogTable25647, i32 0, i32 %bytes, !MSB !3, !LSB !2, !extendFrom !3*/
		nmalloc_24_25 = (1'd0 + (1 * arg_bytes_reg));
end
always @(*) begin
	/* nmalloc: %24*/
	/*   %26 = load i8* %25, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
		nmalloc_24_26 = LogTable25647_out_a;
end
always @(*) begin
	/* nmalloc: %24*/
	/*   %27 = sext i8 %26 to i32, !MSB !8, !LSB !2, !extendFrom !4*/
		nmalloc_24_27 = $signed(nmalloc_24_26);
end
always @(*) begin
	/* nmalloc: %log2.exit*/
	/*   %.0.i = phi i32 [ %10, %6 ], [ %15, %11 ], [ %23, %19 ], [ %27, %24 ], !MSB !8, !LSB !2, !extendFrom !4*/
	if (((cur_state == LEGUP_F_nmalloc_BB__6_4) & (fsm_stall == 1'd0))) begin
		nmalloc_log2exit_0i = nmalloc_6_10;
	end
	/* nmalloc: %log2.exit*/
	/*   %.0.i = phi i32 [ %10, %6 ], [ %15, %11 ], [ %23, %19 ], [ %27, %24 ], !MSB !8, !LSB !2, !extendFrom !4*/
	else if (((cur_state == LEGUP_F_nmalloc_BB__11_6) & (fsm_stall == 1'd0))) begin
		nmalloc_log2exit_0i = nmalloc_11_15;
	end
	/* nmalloc: %log2.exit*/
	/*   %.0.i = phi i32 [ %10, %6 ], [ %15, %11 ], [ %23, %19 ], [ %27, %24 ], !MSB !8, !LSB !2, !extendFrom !4*/
	else if (((cur_state == LEGUP_F_nmalloc_BB__19_9) & (fsm_stall == 1'd0))) begin
		nmalloc_log2exit_0i = nmalloc_19_23;
	end
	/* nmalloc: %log2.exit*/
	/*   %.0.i = phi i32 [ %10, %6 ], [ %15, %11 ], [ %23, %19 ], [ %27, %24 ], !MSB !8, !LSB !2, !extendFrom !4*/
	else /* if (((cur_state == LEGUP_F_nmalloc_BB__24_11) & (fsm_stall == 1'd0))) */ begin
		nmalloc_log2exit_0i = nmalloc_24_27;
	end
end
always @(posedge clk) begin
	/* nmalloc: %log2.exit*/
	/*   %.0.i = phi i32 [ %10, %6 ], [ %15, %11 ], [ %23, %19 ], [ %27, %24 ], !MSB !8, !LSB !2, !extendFrom !4*/
	if (((cur_state == LEGUP_F_nmalloc_BB__6_4) & (fsm_stall == 1'd0))) begin
		nmalloc_log2exit_0i_reg <= nmalloc_log2exit_0i;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_log2exit_0i) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_log2exit_0i_reg"); $finish; end
		/* synthesis translate_on */
	end
	/* nmalloc: %log2.exit*/
	/*   %.0.i = phi i32 [ %10, %6 ], [ %15, %11 ], [ %23, %19 ], [ %27, %24 ], !MSB !8, !LSB !2, !extendFrom !4*/
	if (((cur_state == LEGUP_F_nmalloc_BB__11_6) & (fsm_stall == 1'd0))) begin
		nmalloc_log2exit_0i_reg <= nmalloc_log2exit_0i;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_log2exit_0i) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_log2exit_0i_reg"); $finish; end
		/* synthesis translate_on */
	end
	/* nmalloc: %log2.exit*/
	/*   %.0.i = phi i32 [ %10, %6 ], [ %15, %11 ], [ %23, %19 ], [ %27, %24 ], !MSB !8, !LSB !2, !extendFrom !4*/
	if (((cur_state == LEGUP_F_nmalloc_BB__19_9) & (fsm_stall == 1'd0))) begin
		nmalloc_log2exit_0i_reg <= nmalloc_log2exit_0i;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_log2exit_0i) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_log2exit_0i_reg"); $finish; end
		/* synthesis translate_on */
	end
	/* nmalloc: %log2.exit*/
	/*   %.0.i = phi i32 [ %10, %6 ], [ %15, %11 ], [ %23, %19 ], [ %27, %24 ], !MSB !8, !LSB !2, !extendFrom !4*/
	if (((cur_state == LEGUP_F_nmalloc_BB__24_11) & (fsm_stall == 1'd0))) begin
		nmalloc_log2exit_0i_reg <= nmalloc_log2exit_0i;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_log2exit_0i) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_log2exit_0i_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %log2.exit*/
	/*   %28 = shl i32 1, %.0.i, !MSB !8, !LSB !2, !extendFrom !8*/
		nmalloc_log2exit_28 = (32'd1 <<< $signed(($signed({{24{nmalloc_log2exit_0i_reg[7]}},nmalloc_log2exit_0i_reg}) % 32)));
end
always @(posedge clk) begin
	/* nmalloc: %log2.exit*/
	/*   %28 = shl i32 1, %.0.i, !MSB !8, !LSB !2, !extendFrom !8*/
	if ((cur_state == LEGUP_F_nmalloc_BB_log2exit_12)) begin
		nmalloc_log2exit_28_reg <= nmalloc_log2exit_28;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_log2exit_28) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_log2exit_28_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %log2.exit*/
	/*   %29 = icmp ult i32 %28, %bytes, !MSB !2, !LSB !2, !extendFrom !2*/
		nmalloc_log2exit_29 = (nmalloc_log2exit_28_reg < arg_bytes_reg);
end
always @(*) begin
	/* nmalloc: %log2.exit*/
	/*   %30 = zext i1 %29 to i32, !MSB !2, !LSB !2, !extendFrom !2*/
		nmalloc_log2exit_30 = nmalloc_log2exit_29;
end
always @(posedge clk) begin
	/* nmalloc: %log2.exit*/
	/*   %30 = zext i1 %29 to i32, !MSB !2, !LSB !2, !extendFrom !2*/
	if ((cur_state == LEGUP_F_nmalloc_BB_log2exit_13)) begin
		nmalloc_log2exit_30_reg <= nmalloc_log2exit_30;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_log2exit_30) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_log2exit_30_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %log2.exit*/
	/*   %31 = add nsw i32 %30, %.0.i, !MSB !8, !LSB !2, !extendFrom !10*/
		nmalloc_log2exit_31 = (nmalloc_log2exit_30_reg + $signed({{1{nmalloc_log2exit_0i_reg[7]}},nmalloc_log2exit_0i_reg}));
end
always @(posedge clk) begin
	/* nmalloc: %log2.exit*/
	/*   %31 = add nsw i32 %30, %.0.i, !MSB !8, !LSB !2, !extendFrom !10*/
	if ((cur_state == LEGUP_F_nmalloc_BB_log2exit_14)) begin
		nmalloc_log2exit_31_reg <= nmalloc_log2exit_31;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_log2exit_31) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_log2exit_31_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %log2.exit*/
	/*   %32 = getelementptr inbounds [10 x i32]* @N_FREE_ADDRESS___LT, i32 0, i32 %31, !MSB !3, !LSB !2, !extendFrom !3*/
		nmalloc_log2exit_32 = (1'd0 + (4 * $signed({{23{nmalloc_log2exit_31_reg[8]}},nmalloc_log2exit_31_reg})));
end
always @(posedge clk) begin
	/* nmalloc: %log2.exit*/
	/*   %32 = getelementptr inbounds [10 x i32]* @N_FREE_ADDRESS___LT, i32 0, i32 %31, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_nmalloc_BB_log2exit_15)) begin
		nmalloc_log2exit_32_reg <= nmalloc_log2exit_32;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_log2exit_32) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_log2exit_32_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %log2.exit*/
	/*   %33 = load i32* %32, align 4, !tbaa !11, !MSB !8, !LSB !2, !extendFrom !8*/
		nmalloc_log2exit_33 = N_FREE_ADDRESS___LT_out_a;
end
always @(posedge clk) begin
	/* nmalloc: %log2.exit*/
	/*   %33 = load i32* %32, align 4, !tbaa !11, !MSB !8, !LSB !2, !extendFrom !8*/
	if ((cur_state == LEGUP_F_nmalloc_BB_log2exit_16)) begin
		nmalloc_log2exit_33_reg <= nmalloc_log2exit_33;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_log2exit_33) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_log2exit_33_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %log2.exit*/
	/*   %34 = icmp eq i32 %33, -1, !MSB !2, !LSB !2, !extendFrom !2*/
		nmalloc_log2exit_34 = (nmalloc_log2exit_33 == $signed(-32'd1));
end
always @(*) begin
	/* nmalloc: %36*/
	/*   %37 = xor i32 %33, -1, !MSB !8, !LSB !2, !extendFrom !8*/
		nmalloc_36_37 = (nmalloc_log2exit_33_reg ^ $signed(-32'd1));
end
always @(posedge clk) begin
	/* nmalloc: %36*/
	/*   %37 = xor i32 %33, -1, !MSB !8, !LSB !2, !extendFrom !8*/
	if ((cur_state == LEGUP_F_nmalloc_BB__36_18)) begin
		nmalloc_36_37_reg <= nmalloc_36_37;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_36_37) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_36_37_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %36*/
	/*   %38 = add i32 %33, 1, !MSB !8, !LSB !2, !extendFrom !8*/
		nmalloc_36_38 = (nmalloc_log2exit_33_reg + 32'd1);
end
always @(posedge clk) begin
	/* nmalloc: %36*/
	/*   %38 = add i32 %33, 1, !MSB !8, !LSB !2, !extendFrom !8*/
	if ((cur_state == LEGUP_F_nmalloc_BB__36_18)) begin
		nmalloc_36_38_reg <= nmalloc_36_38;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_36_38) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_36_38_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %36*/
	/*   %39 = and i32 %38, %37, !MSB !8, !LSB !2, !extendFrom !8*/
		nmalloc_36_39 = (nmalloc_36_38_reg & nmalloc_36_37_reg);
end
always @(posedge clk) begin
	/* nmalloc: %36*/
	/*   %39 = and i32 %38, %37, !MSB !8, !LSB !2, !extendFrom !8*/
	if ((cur_state == LEGUP_F_nmalloc_BB__36_19)) begin
		nmalloc_36_39_reg <= nmalloc_36_39;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_36_39) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_36_39_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %36*/
	/*   %40 = lshr i32 %39, 16, !MSB !1, !LSB !2, !extendFrom !1*/
		nmalloc_36_40 = (nmalloc_36_39 >>> (32'd16 % 32));
end
always @(posedge clk) begin
	/* nmalloc: %36*/
	/*   %40 = lshr i32 %39, 16, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_nmalloc_BB__36_19)) begin
		nmalloc_36_40_reg <= nmalloc_36_40;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_36_40) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_36_40_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %36*/
	/*   %41 = icmp eq i32 %40, 0, !MSB !2, !LSB !2, !extendFrom !2*/
		nmalloc_36_41 = (nmalloc_36_40_reg == 32'd0);
end
always @(*) begin
	/* nmalloc: %42*/
	/*   %43 = lshr i32 %39, 24, !MSB !4, !LSB !2, !extendFrom !4*/
		nmalloc_42_43 = (nmalloc_36_39_reg >>> (32'd24 % 32));
end
always @(posedge clk) begin
	/* nmalloc: %42*/
	/*   %43 = lshr i32 %39, 24, !MSB !4, !LSB !2, !extendFrom !4*/
	if ((cur_state == LEGUP_F_nmalloc_BB__42_21)) begin
		nmalloc_42_43_reg <= nmalloc_42_43;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_42_43) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_42_43_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %42*/
	/*   %44 = icmp eq i32 %43, 0, !MSB !2, !LSB !2, !extendFrom !2*/
		nmalloc_42_44 = (nmalloc_42_43 == 32'd0);
end
always @(*) begin
	/* nmalloc: %45*/
	/*   %46 = getelementptr inbounds [256 x i8]* @LogTable25647, i32 0, i32 %43, !MSB !3, !LSB !2, !extendFrom !3*/
		nmalloc_45_46 = (1'd0 + (1 * {24'd0,nmalloc_42_43_reg}));
end
always @(*) begin
	/* nmalloc: %45*/
	/*   %47 = load i8* %46, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
		nmalloc_45_47 = LogTable25647_out_a;
end
always @(*) begin
	/* nmalloc: %45*/
	/*   %48 = sext i8 %47 to i32, !MSB !8, !LSB !2, !extendFrom !4*/
		nmalloc_45_48 = $signed(nmalloc_45_47);
end
always @(*) begin
	/* nmalloc: %45*/
	/*   %49 = or i32 %48, 24, !MSB !8, !LSB !2, !extendFrom !4*/
		nmalloc_45_49 = ($signed(nmalloc_45_48) | 32'd24);
end
always @(*) begin
	/* nmalloc: %50*/
	/*   %51 = getelementptr inbounds [256 x i8]* @LogTable25647, i32 0, i32 %40, !MSB !3, !LSB !2, !extendFrom !3*/
		nmalloc_50_51 = (1'd0 + (1 * {16'd0,nmalloc_36_40_reg}));
end
always @(*) begin
	/* nmalloc: %50*/
	/*   %52 = load i8* %51, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
		nmalloc_50_52 = LogTable25647_out_a;
end
always @(*) begin
	/* nmalloc: %50*/
	/*   %53 = sext i8 %52 to i32, !MSB !8, !LSB !2, !extendFrom !4*/
		nmalloc_50_53 = $signed(nmalloc_50_52);
end
always @(*) begin
	/* nmalloc: %50*/
	/*   %54 = or i32 %53, 16, !MSB !8, !LSB !2, !extendFrom !4*/
		nmalloc_50_54 = ($signed(nmalloc_50_53) | 32'd16);
end
always @(*) begin
	/* nmalloc: %55*/
	/*   %56 = lshr i32 %39, 8, !MSB !9, !LSB !2, !extendFrom !9*/
		nmalloc_55_56 = (nmalloc_36_39_reg >>> (32'd8 % 32));
end
always @(posedge clk) begin
	/* nmalloc: %55*/
	/*   %56 = lshr i32 %39, 8, !MSB !9, !LSB !2, !extendFrom !9*/
	if ((cur_state == LEGUP_F_nmalloc_BB__55_26)) begin
		nmalloc_55_56_reg <= nmalloc_55_56;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_55_56) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_55_56_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %55*/
	/*   %57 = icmp eq i32 %56, 0, !MSB !2, !LSB !2, !extendFrom !2*/
		nmalloc_55_57 = (nmalloc_55_56 == 32'd0);
end
always @(*) begin
	/* nmalloc: %58*/
	/*   %59 = getelementptr inbounds [256 x i8]* @LogTable25647, i32 0, i32 %56, !MSB !3, !LSB !2, !extendFrom !3*/
		nmalloc_58_59 = (1'd0 + (1 * {8'd0,nmalloc_55_56_reg}));
end
always @(*) begin
	/* nmalloc: %58*/
	/*   %60 = load i8* %59, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
		nmalloc_58_60 = LogTable25647_out_a;
end
always @(*) begin
	/* nmalloc: %58*/
	/*   %61 = sext i8 %60 to i32, !MSB !8, !LSB !2, !extendFrom !4*/
		nmalloc_58_61 = $signed(nmalloc_58_60);
end
always @(*) begin
	/* nmalloc: %58*/
	/*   %62 = or i32 %61, 8, !MSB !8, !LSB !2, !extendFrom !4*/
		nmalloc_58_62 = ($signed(nmalloc_58_61) | 32'd8);
end
always @(*) begin
	/* nmalloc: %63*/
	/*   %64 = getelementptr inbounds [256 x i8]* @LogTable25647, i32 0, i32 %39, !MSB !3, !LSB !2, !extendFrom !3*/
		nmalloc_63_64 = (1'd0 + (1 * nmalloc_36_39_reg));
end
always @(*) begin
	/* nmalloc: %63*/
	/*   %65 = load i8* %64, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
		nmalloc_63_65 = LogTable25647_out_a;
end
always @(*) begin
	/* nmalloc: %63*/
	/*   %66 = sext i8 %65 to i32, !MSB !8, !LSB !2, !extendFrom !4*/
		nmalloc_63_66 = $signed(nmalloc_63_65);
end
always @(*) begin
	/* nmalloc: %log2.exit7*/
	/*   %.0.i6 = phi i32 [ %49, %45 ], [ %54, %50 ], [ %62, %58 ], [ %66, %63 ], !MSB !8, !LSB !2, !extendFrom !4*/
	if (((cur_state == LEGUP_F_nmalloc_BB__45_23) & (fsm_stall == 1'd0))) begin
		nmalloc_log2exit7_0i6 = nmalloc_45_49;
	end
	/* nmalloc: %log2.exit7*/
	/*   %.0.i6 = phi i32 [ %49, %45 ], [ %54, %50 ], [ %62, %58 ], [ %66, %63 ], !MSB !8, !LSB !2, !extendFrom !4*/
	else if (((cur_state == LEGUP_F_nmalloc_BB__50_25) & (fsm_stall == 1'd0))) begin
		nmalloc_log2exit7_0i6 = nmalloc_50_54;
	end
	/* nmalloc: %log2.exit7*/
	/*   %.0.i6 = phi i32 [ %49, %45 ], [ %54, %50 ], [ %62, %58 ], [ %66, %63 ], !MSB !8, !LSB !2, !extendFrom !4*/
	else if (((cur_state == LEGUP_F_nmalloc_BB__58_28) & (fsm_stall == 1'd0))) begin
		nmalloc_log2exit7_0i6 = nmalloc_58_62;
	end
	/* nmalloc: %log2.exit7*/
	/*   %.0.i6 = phi i32 [ %49, %45 ], [ %54, %50 ], [ %62, %58 ], [ %66, %63 ], !MSB !8, !LSB !2, !extendFrom !4*/
	else /* if (((cur_state == LEGUP_F_nmalloc_BB__63_30) & (fsm_stall == 1'd0))) */ begin
		nmalloc_log2exit7_0i6 = nmalloc_63_66;
	end
end
always @(posedge clk) begin
	/* nmalloc: %log2.exit7*/
	/*   %.0.i6 = phi i32 [ %49, %45 ], [ %54, %50 ], [ %62, %58 ], [ %66, %63 ], !MSB !8, !LSB !2, !extendFrom !4*/
	if (((cur_state == LEGUP_F_nmalloc_BB__45_23) & (fsm_stall == 1'd0))) begin
		nmalloc_log2exit7_0i6_reg <= nmalloc_log2exit7_0i6;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_log2exit7_0i6) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_log2exit7_0i6_reg"); $finish; end
		/* synthesis translate_on */
	end
	/* nmalloc: %log2.exit7*/
	/*   %.0.i6 = phi i32 [ %49, %45 ], [ %54, %50 ], [ %62, %58 ], [ %66, %63 ], !MSB !8, !LSB !2, !extendFrom !4*/
	if (((cur_state == LEGUP_F_nmalloc_BB__50_25) & (fsm_stall == 1'd0))) begin
		nmalloc_log2exit7_0i6_reg <= nmalloc_log2exit7_0i6;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_log2exit7_0i6) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_log2exit7_0i6_reg"); $finish; end
		/* synthesis translate_on */
	end
	/* nmalloc: %log2.exit7*/
	/*   %.0.i6 = phi i32 [ %49, %45 ], [ %54, %50 ], [ %62, %58 ], [ %66, %63 ], !MSB !8, !LSB !2, !extendFrom !4*/
	if (((cur_state == LEGUP_F_nmalloc_BB__58_28) & (fsm_stall == 1'd0))) begin
		nmalloc_log2exit7_0i6_reg <= nmalloc_log2exit7_0i6;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_log2exit7_0i6) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_log2exit7_0i6_reg"); $finish; end
		/* synthesis translate_on */
	end
	/* nmalloc: %log2.exit7*/
	/*   %.0.i6 = phi i32 [ %49, %45 ], [ %54, %50 ], [ %62, %58 ], [ %66, %63 ], !MSB !8, !LSB !2, !extendFrom !4*/
	if (((cur_state == LEGUP_F_nmalloc_BB__63_30) & (fsm_stall == 1'd0))) begin
		nmalloc_log2exit7_0i6_reg <= nmalloc_log2exit7_0i6;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_log2exit7_0i6) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_log2exit7_0i6_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* nmalloc: %log2.exit7*/
	/*   %67 = or i32 %39, %33, !MSB !8, !LSB !2, !extendFrom !8*/
		nmalloc_log2exit7_67 = (nmalloc_36_39_reg | nmalloc_log2exit_33_reg);
end
always @(*) begin
	/* nmalloc: %log2.exit7*/
	/*   %68 = getelementptr inbounds [10 x [32 x i32*]]* @N_FAST_ADDRESS___LT, i32 0, i32 %31, i32 %.0.i6, !MSB !3, !LSB !2, !extendFrom !3*/
		nmalloc_log2exit7_68 = (1'd0 + ((128 * $signed({{23{nmalloc_log2exit_31_reg[8]}},nmalloc_log2exit_31_reg})) + (4 * $signed({{24{nmalloc_log2exit7_0i6_reg[7]}},nmalloc_log2exit7_0i6_reg}))));
end
always @(*) begin
	/* nmalloc: %log2.exit7*/
	/*   %69 = load i32** %68, align 4, !tbaa !13, !MSB !3, !LSB !2, !extendFrom !3*/
		nmalloc_log2exit7_69 = N_FAST_ADDRESS___LT_out_a;
end
always @(*) begin
	/* nmalloc: %log2.exit7*/
	/*   %70 = bitcast i32* %69 to i8*, !MSB !3, !LSB !2, !extendFrom !3*/
		nmalloc_log2exit7_70 = nmalloc_log2exit7_69;
end
always @(*) begin
	/* nmalloc: %71*/
	/*   %.0 = phi i8* [ %70, %log2.exit7 ], [ null, %.preheader.preheader ], !MSB !3, !LSB !2, !extendFrom !3*/
	if (((cur_state == LEGUP_F_nmalloc_BB_preheaderpreheader_17) & (fsm_stall == 1'd0))) begin
		nmalloc_71_0 = 0;
	end
	/* nmalloc: %71*/
	/*   %.0 = phi i8* [ %70, %log2.exit7 ], [ null, %.preheader.preheader ], !MSB !3, !LSB !2, !extendFrom !3*/
	else /* if (((cur_state == LEGUP_F_nmalloc_BB_log2exit7_32) & (fsm_stall == 1'd0))) */ begin
		nmalloc_71_0 = nmalloc_log2exit7_70;
	end
end
always @(posedge clk) begin
	/* nmalloc: %71*/
	/*   %.0 = phi i8* [ %70, %log2.exit7 ], [ null, %.preheader.preheader ], !MSB !3, !LSB !2, !extendFrom !3*/
	if (((cur_state == LEGUP_F_nmalloc_BB_preheaderpreheader_17) & (fsm_stall == 1'd0))) begin
		nmalloc_71_0_reg <= nmalloc_71_0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_71_0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_71_0_reg"); $finish; end
		/* synthesis translate_on */
	end
	/* nmalloc: %71*/
	/*   %.0 = phi i8* [ %70, %log2.exit7 ], [ null, %.preheader.preheader ], !MSB !3, !LSB !2, !extendFrom !3*/
	if (((cur_state == LEGUP_F_nmalloc_BB_log2exit7_32) & (fsm_stall == 1'd0))) begin
		nmalloc_71_0_reg <= nmalloc_71_0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_71_0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_71_0_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	LogTable25647_address_a = 'dx;
	/* nmalloc: %6*/
	/*   %8 = load i8* %7, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
	if ((cur_state == LEGUP_F_nmalloc_BB__6_3)) begin
		LogTable25647_address_a = (nmalloc_6_7 >>> 3'd0);
	end
	/* nmalloc: %11*/
	/*   %13 = load i8* %12, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
	if ((cur_state == LEGUP_F_nmalloc_BB__11_5)) begin
		LogTable25647_address_a = (nmalloc_11_12 >>> 3'd0);
	end
	/* nmalloc: %19*/
	/*   %21 = load i8* %20, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
	if ((cur_state == LEGUP_F_nmalloc_BB__19_8)) begin
		LogTable25647_address_a = (nmalloc_19_20 >>> 3'd0);
	end
	/* nmalloc: %24*/
	/*   %26 = load i8* %25, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
	if ((cur_state == LEGUP_F_nmalloc_BB__24_10)) begin
		LogTable25647_address_a = (nmalloc_24_25 >>> 3'd0);
	end
	/* nmalloc: %45*/
	/*   %47 = load i8* %46, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
	if ((cur_state == LEGUP_F_nmalloc_BB__45_22)) begin
		LogTable25647_address_a = (nmalloc_45_46 >>> 3'd0);
	end
	/* nmalloc: %50*/
	/*   %52 = load i8* %51, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
	if ((cur_state == LEGUP_F_nmalloc_BB__50_24)) begin
		LogTable25647_address_a = (nmalloc_50_51 >>> 3'd0);
	end
	/* nmalloc: %58*/
	/*   %60 = load i8* %59, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
	if ((cur_state == LEGUP_F_nmalloc_BB__58_27)) begin
		LogTable25647_address_a = (nmalloc_58_59 >>> 3'd0);
	end
	/* nmalloc: %63*/
	/*   %65 = load i8* %64, align 1, !tbaa !5, !MSB !4, !LSB !2, !extendFrom !4*/
	if ((cur_state == LEGUP_F_nmalloc_BB__63_29)) begin
		LogTable25647_address_a = (nmalloc_63_64 >>> 3'd0);
	end
end
always @(*) begin
	N_FAST_ADDRESS___LT_address_a = 'dx;
	/* nmalloc: %log2.exit7*/
	/*   %69 = load i32** %68, align 4, !tbaa !13, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_nmalloc_BB_log2exit7_31)) begin
		N_FAST_ADDRESS___LT_address_a = (nmalloc_log2exit7_68 >>> 3'd2);
	end
end
always @(posedge clk) begin
	if ((cur_state == LEGUP_0)) begin
		finish <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to finish"); $finish; end
		/* synthesis translate_on */
	end
	/* nmalloc: %71*/
	/*   ret i8* %.0, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_nmalloc_BB__71_33)) begin
		finish <= (fsm_stall == 1'd0);
		/* synthesis translate_off */
		if (start == 1'b0 && ^((fsm_stall == 1'd0)) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to finish"); $finish; end
		/* synthesis translate_on */
	end
end
always @(posedge clk) begin
	if ((cur_state == LEGUP_0)) begin
		return_val <= 0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to return_val"); $finish; end
		/* synthesis translate_on */
	end
	/* nmalloc: %71*/
	/*   ret i8* %.0, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_nmalloc_BB__71_33)) begin
		return_val <= nmalloc_71_0_reg;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_71_0_reg) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to return_val"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	N_FREE_ADDRESS___LT_write_enable_a = 1'd0;
	/* nmalloc: %log2.exit7*/
	/*   store i32 %67, i32* %32, align 4, !tbaa !11, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_nmalloc_BB_log2exit7_31)) begin
		N_FREE_ADDRESS___LT_write_enable_a = 1'd1;
	end
end
always @(*) begin
	N_FREE_ADDRESS___LT_in_a = 0;
	/* nmalloc: %log2.exit7*/
	/*   store i32 %67, i32* %32, align 4, !tbaa !11, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_nmalloc_BB_log2exit7_31)) begin
		N_FREE_ADDRESS___LT_in_a = nmalloc_log2exit7_67;
	end
end
assign N_FREE_ADDRESS___LT_byteena_a = 1'd1;
always @(*) begin
	N_FREE_ADDRESS___LT_enable_a = 1'd0;
	/* nmalloc: %log2.exit*/
	/*   %33 = load i32* %32, align 4, !tbaa !11, !MSB !8, !LSB !2, !extendFrom !8*/
	if ((cur_state == LEGUP_F_nmalloc_BB_log2exit_15)) begin
		N_FREE_ADDRESS___LT_enable_a = 1'd1;
	end
	/* nmalloc: %log2.exit7*/
	/*   store i32 %67, i32* %32, align 4, !tbaa !11, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_nmalloc_BB_log2exit7_31)) begin
		N_FREE_ADDRESS___LT_enable_a = 1'd1;
	end
end
always @(*) begin
	N_FREE_ADDRESS___LT_address_a = 4'd0;
	/* nmalloc: %log2.exit*/
	/*   %33 = load i32* %32, align 4, !tbaa !11, !MSB !8, !LSB !2, !extendFrom !8*/
	if ((cur_state == LEGUP_F_nmalloc_BB_log2exit_15)) begin
		N_FREE_ADDRESS___LT_address_a = (nmalloc_log2exit_32 >>> 3'd2);
	end
	/* nmalloc: %log2.exit7*/
	/*   store i32 %67, i32* %32, align 4, !tbaa !11, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_nmalloc_BB_log2exit7_31)) begin
		N_FREE_ADDRESS___LT_address_a = (nmalloc_log2exit_32_reg >>> 3'd2);
	end
end
assign N_FREE_ADDRESS___LT_write_enable_b = 1'd0;
assign N_FREE_ADDRESS___LT_in_b = 0;
assign N_FREE_ADDRESS___LT_byteena_b = 1'd1;
assign N_FREE_ADDRESS___LT_enable_b = 1'd0;
assign N_FREE_ADDRESS___LT_address_b = 4'd0;

endmodule
`timescale 1 ns / 1 ns
module t_init_cycle_req
(
	clk,
	clk2x,
	clk1x_follower,
	reset,
	memory_controller_waitrequest,
	start,
	finish,
	nmalloc_start,
	nmalloc_finish,
	nmalloc_return_val,
	nmalloc_arg_bytes,
	N_SHIFT_LEFTS__LT_enable_a,
	N_SHIFT_LEFTS__LT_address_a,
	N_SHIFT_LEFTS__LT_out_a,
	N_SHIFT_LEFTS__LT_enable_b,
	N_SHIFT_LEFTS__LT_address_b,
	N_SHIFT_LEFTS__LT_out_b,
	N_FREE_ADDRESS___LT_write_enable_a,
	N_FREE_ADDRESS___LT_in_a,
	N_FREE_ADDRESS___LT_byteena_a,
	N_FREE_ADDRESS___LT_enable_a,
	N_FREE_ADDRESS___LT_address_a,
	N_FREE_ADDRESS___LT_out_a,
	N_FREE_ADDRESS___LT_write_enable_b,
	N_FREE_ADDRESS___LT_in_b,
	N_FREE_ADDRESS___LT_byteena_b,
	N_FREE_ADDRESS___LT_enable_b,
	N_FREE_ADDRESS___LT_address_b,
	N_FREE_ADDRESS___LT_out_b
);

parameter [3:0] LEGUP_0 = 4'd0;
parameter [3:0] LEGUP_F_t_init_cycle_req_BB__0_1 = 4'd1;
parameter [3:0] LEGUP_F_t_init_cycle_req_BB__0_3 = 4'd3;
parameter [3:0] LEGUP_F_t_init_cycle_req_BB__0_4 = 4'd4;
parameter [3:0] LEGUP_F_t_init_cycle_req_BB__10_5 = 4'd5;
parameter [3:0] LEGUP_F_t_init_cycle_req_BB__10_6 = 4'd6;
parameter [3:0] LEGUP_F_t_init_cycle_req_BB__10_7 = 4'd7;
parameter [3:0] LEGUP_F_t_init_cycle_req_BB__10_8 = 4'd8;
parameter [3:0] LEGUP_F_t_init_cycle_req_BB__10_9 = 4'd9;
parameter [3:0] LEGUP_F_t_init_cycle_req_BB_nfreeexit_10 = 4'd10;
parameter [3:0] LEGUP_function_call_2 = 4'd2;

input  clk;
input  clk2x;
input  clk1x_follower;
input  reset;
input  memory_controller_waitrequest;
input  start;
output reg  finish;
output reg  nmalloc_start;
input  nmalloc_finish;
input [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_return_val;
output reg [31:0] nmalloc_arg_bytes;
output reg  N_SHIFT_LEFTS__LT_enable_a;
output reg [3:0] N_SHIFT_LEFTS__LT_address_a;
input [7:0] N_SHIFT_LEFTS__LT_out_a;
output  N_SHIFT_LEFTS__LT_enable_b;
output [3:0] N_SHIFT_LEFTS__LT_address_b;
input [7:0] N_SHIFT_LEFTS__LT_out_b;
output reg  N_FREE_ADDRESS___LT_write_enable_a;
output reg [31:0] N_FREE_ADDRESS___LT_in_a;
output  N_FREE_ADDRESS___LT_byteena_a;
output reg  N_FREE_ADDRESS___LT_enable_a;
output reg [3:0] N_FREE_ADDRESS___LT_address_a;
input [31:0] N_FREE_ADDRESS___LT_out_a;
output  N_FREE_ADDRESS___LT_write_enable_b;
output [31:0] N_FREE_ADDRESS___LT_in_b;
output  N_FREE_ADDRESS___LT_byteena_b;
output  N_FREE_ADDRESS___LT_enable_b;
output [3:0] N_FREE_ADDRESS___LT_address_b;
input [31:0] N_FREE_ADDRESS___LT_out_b;
reg [3:0] cur_state;
reg [3:0] next_state;
reg  fsm_stall;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_init_cycle_req_0_2;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_init_cycle_req_0_2_reg;
reg [31:0] t_init_cycle_req_0_6;
reg [31:0] t_init_cycle_req_0_6_reg;
reg [8:0] t_init_cycle_req_0_7;
reg [31:0] t_init_cycle_req_0_8;
reg [31:0] t_init_cycle_req_0_8_reg;
reg  t_init_cycle_req_0_9;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_init_cycle_req_10_11;
reg [7:0] t_init_cycle_req_10_12;
reg [7:0] t_init_cycle_req_10_13;
reg [4:0] t_init_cycle_req_10_14;
reg [4:0] t_init_cycle_req_10_15;
reg [4:0] t_init_cycle_req_10_15_reg;
reg [31:0] t_init_cycle_req_10_16;
reg [31:0] t_init_cycle_req_10_16_reg;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_init_cycle_req_10_17;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_init_cycle_req_10_17_reg;
reg [31:0] t_init_cycle_req_10_18;
reg [31:0] t_init_cycle_req_10_18_reg;
reg [31:0] t_init_cycle_req_10_19;

// Local Rams

// End Local Rams

/* Unsynthesizable Statements */
/* synthesis translate_off */
always @(posedge clk)
	if (!fsm_stall) begin
	/* t_init_cycle_req: %0*/
	/*   %1 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([17 x i8]* @.str, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__0_1)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[pre_malloc][0]\n");
	end
	/* t_init_cycle_req: %0*/
	/*   %3 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([18 x i8]* @.str1, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__0_3)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[post_malloc][0]\n");
	end
	/* t_init_cycle_req: %0*/
	/*   %4 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([23 x i8]* @.str2, i32 0, i32 0), i8* %2) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__0_3)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("Address of *p == %08x\n", $signed(t_init_cycle_req_0_2_reg));
		// to fix quartus warning
		if (reset == 1'b0 && ^(t_init_cycle_req_0_2_reg) === 1'bX) finish <= 0;
	end
	/* t_init_cycle_req: %0*/
	/*   %5 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([15 x i8]* @.str3, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__0_3)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[pre_free][0]\n");
	end
	/* t_init_cycle_req: %nfree.exit*/
	/*   %20 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([16 x i8]* @.str4, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB_nfreeexit_10)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[post_free][0]\n");
	end
end
/* synthesis translate_on */
always @(posedge clk) begin
if (reset == 1'b1)
	cur_state <= LEGUP_0;
else if (!fsm_stall)
	cur_state <= next_state;
end

always @(*)
begin
next_state = cur_state;
case(cur_state)  // synthesis parallel_case  
LEGUP_0:
	if ((fsm_stall == 1'd0) && (start == 1'd1))
		next_state = LEGUP_F_t_init_cycle_req_BB__0_1;
LEGUP_F_t_init_cycle_req_BB__0_1:
		next_state = LEGUP_function_call_2;
LEGUP_F_t_init_cycle_req_BB__0_3:
		next_state = LEGUP_F_t_init_cycle_req_BB__0_4;
LEGUP_F_t_init_cycle_req_BB__0_4:
	if ((fsm_stall == 1'd0) && (t_init_cycle_req_0_9 == 1'd1))
		next_state = LEGUP_F_t_init_cycle_req_BB__10_5;
	else if ((fsm_stall == 1'd0) && (t_init_cycle_req_0_9 == 1'd0))
		next_state = LEGUP_F_t_init_cycle_req_BB_nfreeexit_10;
LEGUP_F_t_init_cycle_req_BB__10_5:
		next_state = LEGUP_F_t_init_cycle_req_BB__10_6;
LEGUP_F_t_init_cycle_req_BB__10_6:
		next_state = LEGUP_F_t_init_cycle_req_BB__10_7;
LEGUP_F_t_init_cycle_req_BB__10_7:
		next_state = LEGUP_F_t_init_cycle_req_BB__10_8;
LEGUP_F_t_init_cycle_req_BB__10_8:
		next_state = LEGUP_F_t_init_cycle_req_BB__10_9;
LEGUP_F_t_init_cycle_req_BB__10_9:
		next_state = LEGUP_F_t_init_cycle_req_BB_nfreeexit_10;
LEGUP_F_t_init_cycle_req_BB_nfreeexit_10:
		next_state = LEGUP_0;
LEGUP_function_call_2:
	if ((fsm_stall == 1'd0) && (nmalloc_finish == 1'd1))
		next_state = LEGUP_F_t_init_cycle_req_BB__0_3;
default:
	next_state = cur_state;
endcase

end
always @(*) begin
	fsm_stall = 1'd0;
	if (reset) begin
		fsm_stall = 1'd0;
	end
	if (memory_controller_waitrequest) begin
		fsm_stall = 1'd1;
	end
end
always @(*) begin
	/* t_init_cycle_req: %0*/
	/*   %2 = tail call fastcc noalias i8* @nmalloc(i32 8) #2, !MSB !3, !LSB !2, !extendFrom !3*/
		t_init_cycle_req_0_2 = nmalloc_return_val;
end
always @(posedge clk) begin
	/* t_init_cycle_req: %0*/
	/*   %2 = tail call fastcc noalias i8* @nmalloc(i32 8) #2, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_function_call_2)) begin
		t_init_cycle_req_0_2_reg <= t_init_cycle_req_0_2;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_init_cycle_req_0_2) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_init_cycle_req_0_2_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_init_cycle_req: %0*/
	/*   %6 = ptrtoint i8* %2 to i32, !MSB !1, !LSB !2, !extendFrom !1*/
		t_init_cycle_req_0_6 = t_init_cycle_req_0_2_reg;
end
always @(posedge clk) begin
	/* t_init_cycle_req: %0*/
	/*   %6 = ptrtoint i8* %2 to i32, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__0_3)) begin
		t_init_cycle_req_0_6_reg <= t_init_cycle_req_0_6;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_init_cycle_req_0_6) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_init_cycle_req_0_6_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_init_cycle_req: %0*/
	/*   %7 = lshr i32 %6, 23, !MSB !4, !LSB !2, !extendFrom !4*/
		t_init_cycle_req_0_7 = (t_init_cycle_req_0_6 >>> (32'd23 % 32));
end
always @(*) begin
	/* t_init_cycle_req: %0*/
	/*   %8 = sub i32 %7, lshr (i32 ptrtoint ([256 x i32]* @N_FAST_ARENA_0_32 to i32), i32 23), !MSB !1, !LSB !2, !extendFrom !1*/
		t_init_cycle_req_0_8 = ({23'd0,t_init_cycle_req_0_7} - (`TAG_g_N_FAST_ARENA_0_32_a >>> 32'd23));
end
always @(posedge clk) begin
	/* t_init_cycle_req: %0*/
	/*   %8 = sub i32 %7, lshr (i32 ptrtoint ([256 x i32]* @N_FAST_ARENA_0_32 to i32), i32 23), !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__0_3)) begin
		t_init_cycle_req_0_8_reg <= t_init_cycle_req_0_8;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_init_cycle_req_0_8) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_init_cycle_req_0_8_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_init_cycle_req: %0*/
	/*   %9 = icmp ult i32 %8, 10, !MSB !2, !LSB !2, !extendFrom !2*/
		t_init_cycle_req_0_9 = (t_init_cycle_req_0_8_reg < 32'd10);
end
always @(*) begin
	/* t_init_cycle_req: %10*/
	/*   %11 = getelementptr inbounds [10 x i8]* @N_SHIFT_LEFTS__LT, i32 0, i32 %8, !MSB !3, !LSB !2, !extendFrom !3*/
		t_init_cycle_req_10_11 = (1'd0 + (1 * t_init_cycle_req_0_8_reg));
end
always @(*) begin
	/* t_init_cycle_req: %10*/
	/*   %12 = load i8* %11, align 1, !tbaa !5, !MSB !8, !LSB !2, !extendFrom !8*/
		t_init_cycle_req_10_12 = N_SHIFT_LEFTS__LT_out_a;
end
always @(*) begin
	/* t_init_cycle_req: %10*/
	/*   %13 = zext i8 %12 to i32, !MSB !8, !LSB !2, !extendFrom !8*/
		t_init_cycle_req_10_13 = t_init_cycle_req_10_12;
end
always @(*) begin
	/* t_init_cycle_req: %10*/
	/*   %14 = lshr i32 %6, %13, !MSB !9, !LSB !2, !extendFrom !9*/
		t_init_cycle_req_10_14 = (t_init_cycle_req_0_6_reg >>> (t_init_cycle_req_10_13 % 32));
end
always @(*) begin
	/* t_init_cycle_req: %10*/
	/*   %15 = and i32 %14, 31, !MSB !9, !LSB !2, !extendFrom !9*/
		t_init_cycle_req_10_15 = (t_init_cycle_req_10_14 & 32'd31);
end
always @(posedge clk) begin
	/* t_init_cycle_req: %10*/
	/*   %15 = and i32 %14, 31, !MSB !9, !LSB !2, !extendFrom !9*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__10_6)) begin
		t_init_cycle_req_10_15_reg <= t_init_cycle_req_10_15;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_init_cycle_req_10_15) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_init_cycle_req_10_15_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_init_cycle_req: %10*/
	/*   %16 = shl i32 1, %15, !MSB !1, !LSB !2, !extendFrom !1*/
		t_init_cycle_req_10_16 = (32'd1 <<< ({27'd0,t_init_cycle_req_10_15_reg} % 32));
end
always @(posedge clk) begin
	/* t_init_cycle_req: %10*/
	/*   %16 = shl i32 1, %15, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__10_7)) begin
		t_init_cycle_req_10_16_reg <= t_init_cycle_req_10_16;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_init_cycle_req_10_16) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_init_cycle_req_10_16_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_init_cycle_req: %10*/
	/*   %17 = getelementptr inbounds [10 x i32]* @N_FREE_ADDRESS___LT, i32 0, i32 %8, !MSB !3, !LSB !2, !extendFrom !3*/
		t_init_cycle_req_10_17 = (1'd0 + (4 * t_init_cycle_req_0_8_reg));
end
always @(posedge clk) begin
	/* t_init_cycle_req: %10*/
	/*   %17 = getelementptr inbounds [10 x i32]* @N_FREE_ADDRESS___LT, i32 0, i32 %8, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__10_5)) begin
		t_init_cycle_req_10_17_reg <= t_init_cycle_req_10_17;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_init_cycle_req_10_17) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_init_cycle_req_10_17_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_init_cycle_req: %10*/
	/*   %18 = load i32* %17, align 4, !tbaa !10, !MSB !1, !LSB !2, !extendFrom !1*/
		t_init_cycle_req_10_18 = N_FREE_ADDRESS___LT_out_a;
end
always @(posedge clk) begin
	/* t_init_cycle_req: %10*/
	/*   %18 = load i32* %17, align 4, !tbaa !10, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__10_6)) begin
		t_init_cycle_req_10_18_reg <= t_init_cycle_req_10_18;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_init_cycle_req_10_18) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_init_cycle_req_10_18_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_init_cycle_req: %10*/
	/*   %19 = xor i32 %16, %18, !MSB !1, !LSB !2, !extendFrom !1*/
		t_init_cycle_req_10_19 = (t_init_cycle_req_10_16_reg ^ t_init_cycle_req_10_18_reg);
end
always @(posedge clk) begin
	if ((cur_state == LEGUP_0)) begin
		finish <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to finish"); $finish; end
		/* synthesis translate_on */
	end
	/* t_init_cycle_req: %nfree.exit*/
	/*   ret void, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB_nfreeexit_10)) begin
		finish <= (fsm_stall == 1'd0);
		/* synthesis translate_off */
		if (start == 1'b0 && ^((fsm_stall == 1'd0)) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to finish"); $finish; end
		/* synthesis translate_on */
	end
end
always @(posedge clk) begin
	nmalloc_start <= 1'd0;
	/* t_init_cycle_req: %0*/
	/*   %2 = tail call fastcc noalias i8* @nmalloc(i32 8) #2, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__0_1)) begin
		nmalloc_start <= 1'd1;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd1) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_start"); $finish; end
		/* synthesis translate_on */
	end
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__0_3)) begin
		nmalloc_start <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_start"); $finish; end
		/* synthesis translate_on */
	end
end
always @(posedge clk) begin
	nmalloc_arg_bytes <= 4'd0;
	/* t_init_cycle_req: %0*/
	/*   %2 = tail call fastcc noalias i8* @nmalloc(i32 8) #2, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__0_1)) begin
		nmalloc_arg_bytes <= 32'd8;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(32'd8) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_arg_bytes"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	N_SHIFT_LEFTS__LT_enable_a = 1'd0;
	/* t_init_cycle_req: %10*/
	/*   %12 = load i8* %11, align 1, !tbaa !5, !MSB !8, !LSB !2, !extendFrom !8*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__10_5)) begin
		N_SHIFT_LEFTS__LT_enable_a = 1'd1;
	end
end
always @(*) begin
	N_SHIFT_LEFTS__LT_address_a = 4'd0;
	/* t_init_cycle_req: %10*/
	/*   %12 = load i8* %11, align 1, !tbaa !5, !MSB !8, !LSB !2, !extendFrom !8*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__10_5)) begin
		N_SHIFT_LEFTS__LT_address_a = (t_init_cycle_req_10_11 >>> 3'd0);
	end
end
assign N_SHIFT_LEFTS__LT_enable_b = 1'd0;
assign N_SHIFT_LEFTS__LT_address_b = 4'd0;
always @(*) begin
	N_FREE_ADDRESS___LT_write_enable_a = 1'd0;
	/* t_init_cycle_req: %10*/
	/*   store i32 %19, i32* %17, align 4, !tbaa !10, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__10_8)) begin
		N_FREE_ADDRESS___LT_write_enable_a = 1'd1;
	end
end
always @(*) begin
	N_FREE_ADDRESS___LT_in_a = 0;
	/* t_init_cycle_req: %10*/
	/*   store i32 %19, i32* %17, align 4, !tbaa !10, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__10_8)) begin
		N_FREE_ADDRESS___LT_in_a = t_init_cycle_req_10_19;
	end
end
assign N_FREE_ADDRESS___LT_byteena_a = 1'd1;
always @(*) begin
	N_FREE_ADDRESS___LT_enable_a = 1'd0;
	/* t_init_cycle_req: %10*/
	/*   %18 = load i32* %17, align 4, !tbaa !10, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__10_5)) begin
		N_FREE_ADDRESS___LT_enable_a = 1'd1;
	end
	/* t_init_cycle_req: %10*/
	/*   store i32 %19, i32* %17, align 4, !tbaa !10, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__10_8)) begin
		N_FREE_ADDRESS___LT_enable_a = 1'd1;
	end
end
always @(*) begin
	N_FREE_ADDRESS___LT_address_a = 4'd0;
	/* t_init_cycle_req: %10*/
	/*   %18 = load i32* %17, align 4, !tbaa !10, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__10_5)) begin
		N_FREE_ADDRESS___LT_address_a = (t_init_cycle_req_10_17 >>> 3'd2);
	end
	/* t_init_cycle_req: %10*/
	/*   store i32 %19, i32* %17, align 4, !tbaa !10, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_init_cycle_req_BB__10_8)) begin
		N_FREE_ADDRESS___LT_address_a = (t_init_cycle_req_10_17_reg >>> 3'd2);
	end
end
assign N_FREE_ADDRESS___LT_write_enable_b = 1'd0;
assign N_FREE_ADDRESS___LT_in_b = 0;
assign N_FREE_ADDRESS___LT_byteena_b = 1'd1;
assign N_FREE_ADDRESS___LT_enable_b = 1'd0;
assign N_FREE_ADDRESS___LT_address_b = 4'd0;

endmodule
`timescale 1 ns / 1 ns
module t_overlapping_req
(
	clk,
	clk2x,
	clk1x_follower,
	reset,
	memory_controller_waitrequest,
	start,
	finish,
	nmalloc_start,
	nmalloc_finish,
	nmalloc_return_val,
	nmalloc_arg_bytes,
	N_SHIFT_LEFTS__LT_enable_a,
	N_SHIFT_LEFTS__LT_address_a,
	N_SHIFT_LEFTS__LT_out_a,
	N_SHIFT_LEFTS__LT_enable_b,
	N_SHIFT_LEFTS__LT_address_b,
	N_SHIFT_LEFTS__LT_out_b,
	N_FREE_ADDRESS___LT_write_enable_a,
	N_FREE_ADDRESS___LT_in_a,
	N_FREE_ADDRESS___LT_byteena_a,
	N_FREE_ADDRESS___LT_enable_a,
	N_FREE_ADDRESS___LT_address_a,
	N_FREE_ADDRESS___LT_out_a,
	N_FREE_ADDRESS___LT_write_enable_b,
	N_FREE_ADDRESS___LT_in_b,
	N_FREE_ADDRESS___LT_byteena_b,
	N_FREE_ADDRESS___LT_enable_b,
	N_FREE_ADDRESS___LT_address_b,
	N_FREE_ADDRESS___LT_out_b
);

parameter [4:0] LEGUP_0 = 5'd0;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__0_1 = 5'd1;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__0_3 = 5'd3;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__0_5 = 5'd5;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__8_6 = 5'd6;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__8_7 = 5'd7;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__14_8 = 5'd8;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__14_9 = 5'd9;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__22_10 = 5'd10;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__22_11 = 5'd11;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__22_12 = 5'd12;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__22_13 = 5'd13;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__22_14 = 5'd14;
parameter [4:0] LEGUP_F_t_overlapping_req_BB_nfreeexit_15 = 5'd15;
parameter [4:0] LEGUP_F_t_overlapping_req_BB_nfreeexit_16 = 5'd16;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__38_17 = 5'd17;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__38_18 = 5'd18;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__38_19 = 5'd19;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__38_20 = 5'd20;
parameter [4:0] LEGUP_F_t_overlapping_req_BB__38_21 = 5'd21;
parameter [4:0] LEGUP_F_t_overlapping_req_BB_nfreeexit1_22 = 5'd22;
parameter [4:0] LEGUP_function_call_2 = 5'd2;
parameter [4:0] LEGUP_function_call_4 = 5'd4;

input  clk;
input  clk2x;
input  clk1x_follower;
input  reset;
input  memory_controller_waitrequest;
input  start;
output reg  finish;
output reg  nmalloc_start;
input  nmalloc_finish;
input [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_return_val;
output reg [31:0] nmalloc_arg_bytes;
output reg  N_SHIFT_LEFTS__LT_enable_a;
output reg [3:0] N_SHIFT_LEFTS__LT_address_a;
input [7:0] N_SHIFT_LEFTS__LT_out_a;
output  N_SHIFT_LEFTS__LT_enable_b;
output [3:0] N_SHIFT_LEFTS__LT_address_b;
input [7:0] N_SHIFT_LEFTS__LT_out_b;
output reg  N_FREE_ADDRESS___LT_write_enable_a;
output reg [31:0] N_FREE_ADDRESS___LT_in_a;
output  N_FREE_ADDRESS___LT_byteena_a;
output reg  N_FREE_ADDRESS___LT_enable_a;
output reg [3:0] N_FREE_ADDRESS___LT_address_a;
input [31:0] N_FREE_ADDRESS___LT_out_a;
output  N_FREE_ADDRESS___LT_write_enable_b;
output [31:0] N_FREE_ADDRESS___LT_in_b;
output  N_FREE_ADDRESS___LT_byteena_b;
output  N_FREE_ADDRESS___LT_enable_b;
output [3:0] N_FREE_ADDRESS___LT_address_b;
input [31:0] N_FREE_ADDRESS___LT_out_b;
reg [4:0] cur_state;
reg [4:0] next_state;
reg  fsm_stall;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_overlapping_req_0_2;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_overlapping_req_0_2_reg;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_overlapping_req_0_6;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_overlapping_req_0_6_reg;
reg [4:0] t_overlapping_req_8_9;
reg [4:0] t_overlapping_req_8_9_reg;
reg [6:0] t_overlapping_req_8_10;
reg [6:0] t_overlapping_req_8_10_reg;
reg [5:0] t_overlapping_req_8_13;
reg [5:0] t_overlapping_req_8_13_reg;
reg  t_overlapping_req_8_exitcond;
reg [31:0] t_overlapping_req_14_18;
reg [31:0] t_overlapping_req_14_18_reg;
reg [8:0] t_overlapping_req_14_19;
reg [31:0] t_overlapping_req_14_20;
reg [31:0] t_overlapping_req_14_20_reg;
reg  t_overlapping_req_14_21;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_overlapping_req_22_23;
reg [7:0] t_overlapping_req_22_24;
reg [7:0] t_overlapping_req_22_25;
reg [4:0] t_overlapping_req_22_26;
reg [4:0] t_overlapping_req_22_27;
reg [4:0] t_overlapping_req_22_27_reg;
reg [31:0] t_overlapping_req_22_28;
reg [31:0] t_overlapping_req_22_28_reg;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_overlapping_req_22_29;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_overlapping_req_22_29_reg;
reg [31:0] t_overlapping_req_22_30;
reg [31:0] t_overlapping_req_22_30_reg;
reg [31:0] t_overlapping_req_22_31;
reg [31:0] t_overlapping_req_nfreeexit_34;
reg [31:0] t_overlapping_req_nfreeexit_34_reg;
reg [8:0] t_overlapping_req_nfreeexit_35;
reg [31:0] t_overlapping_req_nfreeexit_36;
reg [31:0] t_overlapping_req_nfreeexit_36_reg;
reg  t_overlapping_req_nfreeexit_37;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_overlapping_req_38_39;
reg [7:0] t_overlapping_req_38_40;
reg [7:0] t_overlapping_req_38_41;
reg [4:0] t_overlapping_req_38_42;
reg [4:0] t_overlapping_req_38_43;
reg [4:0] t_overlapping_req_38_43_reg;
reg [31:0] t_overlapping_req_38_44;
reg [31:0] t_overlapping_req_38_44_reg;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_overlapping_req_38_45;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_overlapping_req_38_45_reg;
reg [31:0] t_overlapping_req_38_46;
reg [31:0] t_overlapping_req_38_46_reg;
reg [31:0] t_overlapping_req_38_47;

// Local Rams

// End Local Rams

/* Unsynthesizable Statements */
/* synthesis translate_off */
always @(posedge clk)
	if (!fsm_stall) begin
	/* t_overlapping_req: %0*/
	/*   %1 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([17 x i8]* @.str5, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__0_1)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[pre_malloc][1]\n");
	end
	/* t_overlapping_req: %0*/
	/*   %3 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([18 x i8]* @.str6, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__0_3)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[post_malloc][1]\n");
	end
	/* t_overlapping_req: %0*/
	/*   %4 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([23 x i8]* @.str7, i32 0, i32 0), i8* %2) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__0_3)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("Address of *q == %08x\n", $signed(t_overlapping_req_0_2_reg));
		// to fix quartus warning
		if (reset == 1'b0 && ^(t_overlapping_req_0_2_reg) === 1'bX) finish <= 0;
	end
	/* t_overlapping_req: %0*/
	/*   %5 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([17 x i8]* @.str8, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__0_3)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[pre_malloc][2]\n");
	end
	/* t_overlapping_req: %0*/
	/*   %7 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([18 x i8]* @.str9, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__0_5)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[post_malloc][2]\n");
	end
	/* t_overlapping_req: %8*/
	/*   %12 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([11 x i8]* @.str10, i32 0, i32 0), i32 %10) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__8_7)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("r[i] = %c\n", t_overlapping_req_8_10_reg);
		// to fix quartus warning
		if (reset == 1'b0 && ^(t_overlapping_req_8_10_reg) === 1'bX) finish <= 0;
	end
	/* t_overlapping_req: %14*/
	/*   %15 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([7 x i8]* @.str11, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__14_8)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("DONE.\n");
	end
	/* t_overlapping_req: %14*/
	/*   %16 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([23 x i8]* @.str12, i32 0, i32 0), i8* %6) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__14_8)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("Address of *r == %08x\n", $signed(t_overlapping_req_0_6_reg));
		// to fix quartus warning
		if (reset == 1'b0 && ^(t_overlapping_req_0_6_reg) === 1'bX) finish <= 0;
	end
	/* t_overlapping_req: %14*/
	/*   %17 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([15 x i8]* @.str13, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__14_8)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[pre_free][2]\n");
	end
	/* t_overlapping_req: %nfree.exit*/
	/*   %32 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([16 x i8]* @.str14, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB_nfreeexit_15)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[post_free][2]\n");
	end
	/* t_overlapping_req: %nfree.exit*/
	/*   %33 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([15 x i8]* @.str15, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB_nfreeexit_15)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[pre_free][1]\n");
	end
	/* t_overlapping_req: %nfree.exit1*/
	/*   %48 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([16 x i8]* @.str16, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB_nfreeexit1_22)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[post_free][1]\n");
	end
end
/* synthesis translate_on */
always @(posedge clk) begin
if (reset == 1'b1)
	cur_state <= LEGUP_0;
else if (!fsm_stall)
	cur_state <= next_state;
end

always @(*)
begin
next_state = cur_state;
case(cur_state)  // synthesis parallel_case  
LEGUP_0:
	if ((fsm_stall == 1'd0) && (start == 1'd1))
		next_state = LEGUP_F_t_overlapping_req_BB__0_1;
LEGUP_F_t_overlapping_req_BB__0_1:
		next_state = LEGUP_function_call_2;
LEGUP_F_t_overlapping_req_BB__0_3:
		next_state = LEGUP_function_call_4;
LEGUP_F_t_overlapping_req_BB__0_5:
		next_state = LEGUP_F_t_overlapping_req_BB__8_6;
LEGUP_F_t_overlapping_req_BB__14_8:
		next_state = LEGUP_F_t_overlapping_req_BB__14_9;
LEGUP_F_t_overlapping_req_BB__14_9:
	if ((fsm_stall == 1'd0) && (t_overlapping_req_14_21 == 1'd1))
		next_state = LEGUP_F_t_overlapping_req_BB__22_10;
	else if ((fsm_stall == 1'd0) && (t_overlapping_req_14_21 == 1'd0))
		next_state = LEGUP_F_t_overlapping_req_BB_nfreeexit_15;
LEGUP_F_t_overlapping_req_BB__22_10:
		next_state = LEGUP_F_t_overlapping_req_BB__22_11;
LEGUP_F_t_overlapping_req_BB__22_11:
		next_state = LEGUP_F_t_overlapping_req_BB__22_12;
LEGUP_F_t_overlapping_req_BB__22_12:
		next_state = LEGUP_F_t_overlapping_req_BB__22_13;
LEGUP_F_t_overlapping_req_BB__22_13:
		next_state = LEGUP_F_t_overlapping_req_BB__22_14;
LEGUP_F_t_overlapping_req_BB__22_14:
		next_state = LEGUP_F_t_overlapping_req_BB_nfreeexit_15;
LEGUP_F_t_overlapping_req_BB__38_17:
		next_state = LEGUP_F_t_overlapping_req_BB__38_18;
LEGUP_F_t_overlapping_req_BB__38_18:
		next_state = LEGUP_F_t_overlapping_req_BB__38_19;
LEGUP_F_t_overlapping_req_BB__38_19:
		next_state = LEGUP_F_t_overlapping_req_BB__38_20;
LEGUP_F_t_overlapping_req_BB__38_20:
		next_state = LEGUP_F_t_overlapping_req_BB__38_21;
LEGUP_F_t_overlapping_req_BB__38_21:
		next_state = LEGUP_F_t_overlapping_req_BB_nfreeexit1_22;
LEGUP_F_t_overlapping_req_BB__8_6:
		next_state = LEGUP_F_t_overlapping_req_BB__8_7;
LEGUP_F_t_overlapping_req_BB__8_7:
	if ((fsm_stall == 1'd0) && (t_overlapping_req_8_exitcond == 1'd1))
		next_state = LEGUP_F_t_overlapping_req_BB__14_8;
	else if ((fsm_stall == 1'd0) && (t_overlapping_req_8_exitcond == 1'd0))
		next_state = LEGUP_F_t_overlapping_req_BB__8_6;
LEGUP_F_t_overlapping_req_BB_nfreeexit1_22:
		next_state = LEGUP_0;
LEGUP_F_t_overlapping_req_BB_nfreeexit_15:
		next_state = LEGUP_F_t_overlapping_req_BB_nfreeexit_16;
LEGUP_F_t_overlapping_req_BB_nfreeexit_16:
	if ((fsm_stall == 1'd0) && (t_overlapping_req_nfreeexit_37 == 1'd1))
		next_state = LEGUP_F_t_overlapping_req_BB__38_17;
	else if ((fsm_stall == 1'd0) && (t_overlapping_req_nfreeexit_37 == 1'd0))
		next_state = LEGUP_F_t_overlapping_req_BB_nfreeexit1_22;
LEGUP_function_call_2:
	if ((fsm_stall == 1'd0) && (nmalloc_finish == 1'd1))
		next_state = LEGUP_F_t_overlapping_req_BB__0_3;
LEGUP_function_call_4:
	if ((fsm_stall == 1'd0) && (nmalloc_finish == 1'd1))
		next_state = LEGUP_F_t_overlapping_req_BB__0_5;
default:
	next_state = cur_state;
endcase

end
always @(*) begin
	fsm_stall = 1'd0;
	if (reset) begin
		fsm_stall = 1'd0;
	end
	if (memory_controller_waitrequest) begin
		fsm_stall = 1'd1;
	end
end
always @(*) begin
	/* t_overlapping_req: %0*/
	/*   %2 = tail call fastcc noalias i8* @nmalloc(i32 34) #2, !MSB !3, !LSB !2, !extendFrom !3*/
		t_overlapping_req_0_2 = nmalloc_return_val;
end
always @(posedge clk) begin
	/* t_overlapping_req: %0*/
	/*   %2 = tail call fastcc noalias i8* @nmalloc(i32 34) #2, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_function_call_2)) begin
		t_overlapping_req_0_2_reg <= t_overlapping_req_0_2;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_0_2) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_0_2_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %0*/
	/*   %6 = tail call fastcc noalias i8* @nmalloc(i32 24) #2, !MSB !3, !LSB !2, !extendFrom !3*/
		t_overlapping_req_0_6 = nmalloc_return_val;
end
always @(posedge clk) begin
	/* t_overlapping_req: %0*/
	/*   %6 = tail call fastcc noalias i8* @nmalloc(i32 24) #2, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_function_call_4)) begin
		t_overlapping_req_0_6_reg <= t_overlapping_req_0_6;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_0_6) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_0_6_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %8*/
	/*   %9 = phi i32 [ 0, %0 ], [ %13, %8 ], !MSB !4, !LSB !2, !extendFrom !4*/
	if (((cur_state == LEGUP_F_t_overlapping_req_BB__0_5) & (fsm_stall == 1'd0))) begin
		t_overlapping_req_8_9 = 32'd0;
	end
	/* t_overlapping_req: %8*/
	/*   %9 = phi i32 [ 0, %0 ], [ %13, %8 ], !MSB !4, !LSB !2, !extendFrom !4*/
	else /* if ((((cur_state == LEGUP_F_t_overlapping_req_BB__8_7) & (fsm_stall == 1'd0)) & (t_overlapping_req_8_exitcond == 1'd0))) */ begin
		t_overlapping_req_8_9 = t_overlapping_req_8_13_reg;
	end
end
always @(posedge clk) begin
	/* t_overlapping_req: %8*/
	/*   %9 = phi i32 [ 0, %0 ], [ %13, %8 ], !MSB !4, !LSB !2, !extendFrom !4*/
	if (((cur_state == LEGUP_F_t_overlapping_req_BB__0_5) & (fsm_stall == 1'd0))) begin
		t_overlapping_req_8_9_reg <= t_overlapping_req_8_9;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_8_9) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_8_9_reg"); $finish; end
		/* synthesis translate_on */
	end
	/* t_overlapping_req: %8*/
	/*   %9 = phi i32 [ 0, %0 ], [ %13, %8 ], !MSB !4, !LSB !2, !extendFrom !4*/
	if ((((cur_state == LEGUP_F_t_overlapping_req_BB__8_7) & (fsm_stall == 1'd0)) & (t_overlapping_req_8_exitcond == 1'd0))) begin
		t_overlapping_req_8_9_reg <= t_overlapping_req_8_9;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_8_9) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_8_9_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %8*/
	/*   %10 = add i32 %9, 65, !MSB !5, !LSB !2, !extendFrom !5*/
		t_overlapping_req_8_10 = ({2'd0,t_overlapping_req_8_9_reg} + 32'd65);
end
always @(posedge clk) begin
	/* t_overlapping_req: %8*/
	/*   %10 = add i32 %9, 65, !MSB !5, !LSB !2, !extendFrom !5*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__8_6)) begin
		t_overlapping_req_8_10_reg <= t_overlapping_req_8_10;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_8_10) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_8_10_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %8*/
	/*   %13 = add nsw i32 %9, 1, !MSB !9, !LSB !2, !extendFrom !9*/
		t_overlapping_req_8_13 = ({1'd0,t_overlapping_req_8_9_reg} + 32'd1);
end
always @(posedge clk) begin
	/* t_overlapping_req: %8*/
	/*   %13 = add nsw i32 %9, 1, !MSB !9, !LSB !2, !extendFrom !9*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__8_6)) begin
		t_overlapping_req_8_13_reg <= t_overlapping_req_8_13;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_8_13) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_8_13_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %8*/
	/*   %exitcond = icmp eq i32 %13, 24, !MSB !2, !LSB !2, !extendFrom !2*/
		t_overlapping_req_8_exitcond = (t_overlapping_req_8_13_reg == 32'd24);
end
always @(*) begin
	/* t_overlapping_req: %14*/
	/*   %18 = ptrtoint i8* %6 to i32, !MSB !1, !LSB !2, !extendFrom !1*/
		t_overlapping_req_14_18 = t_overlapping_req_0_6_reg;
end
always @(posedge clk) begin
	/* t_overlapping_req: %14*/
	/*   %18 = ptrtoint i8* %6 to i32, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__14_8)) begin
		t_overlapping_req_14_18_reg <= t_overlapping_req_14_18;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_14_18) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_14_18_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %14*/
	/*   %19 = lshr i32 %18, 23, !MSB !10, !LSB !2, !extendFrom !10*/
		t_overlapping_req_14_19 = (t_overlapping_req_14_18 >>> (32'd23 % 32));
end
always @(*) begin
	/* t_overlapping_req: %14*/
	/*   %20 = sub i32 %19, lshr (i32 ptrtoint ([256 x i32]* @N_FAST_ARENA_0_32 to i32), i32 23), !MSB !1, !LSB !2, !extendFrom !1*/
		t_overlapping_req_14_20 = ({23'd0,t_overlapping_req_14_19} - (`TAG_g_N_FAST_ARENA_0_32_a >>> 32'd23));
end
always @(posedge clk) begin
	/* t_overlapping_req: %14*/
	/*   %20 = sub i32 %19, lshr (i32 ptrtoint ([256 x i32]* @N_FAST_ARENA_0_32 to i32), i32 23), !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__14_8)) begin
		t_overlapping_req_14_20_reg <= t_overlapping_req_14_20;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_14_20) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_14_20_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %14*/
	/*   %21 = icmp ult i32 %20, 10, !MSB !2, !LSB !2, !extendFrom !2*/
		t_overlapping_req_14_21 = (t_overlapping_req_14_20_reg < 32'd10);
end
always @(*) begin
	/* t_overlapping_req: %22*/
	/*   %23 = getelementptr inbounds [10 x i8]* @N_SHIFT_LEFTS__LT, i32 0, i32 %20, !MSB !3, !LSB !2, !extendFrom !3*/
		t_overlapping_req_22_23 = (1'd0 + (1 * t_overlapping_req_14_20_reg));
end
always @(*) begin
	/* t_overlapping_req: %22*/
	/*   %24 = load i8* %23, align 1, !tbaa !6, !MSB !11, !LSB !2, !extendFrom !11*/
		t_overlapping_req_22_24 = N_SHIFT_LEFTS__LT_out_a;
end
always @(*) begin
	/* t_overlapping_req: %22*/
	/*   %25 = zext i8 %24 to i32, !MSB !11, !LSB !2, !extendFrom !11*/
		t_overlapping_req_22_25 = t_overlapping_req_22_24;
end
always @(*) begin
	/* t_overlapping_req: %22*/
	/*   %26 = lshr i32 %18, %25, !MSB !4, !LSB !2, !extendFrom !4*/
		t_overlapping_req_22_26 = (t_overlapping_req_14_18_reg >>> (t_overlapping_req_22_25 % 32));
end
always @(*) begin
	/* t_overlapping_req: %22*/
	/*   %27 = and i32 %26, 31, !MSB !4, !LSB !2, !extendFrom !4*/
		t_overlapping_req_22_27 = (t_overlapping_req_22_26 & 32'd31);
end
always @(posedge clk) begin
	/* t_overlapping_req: %22*/
	/*   %27 = and i32 %26, 31, !MSB !4, !LSB !2, !extendFrom !4*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__22_11)) begin
		t_overlapping_req_22_27_reg <= t_overlapping_req_22_27;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_22_27) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_22_27_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %22*/
	/*   %28 = shl i32 1, %27, !MSB !1, !LSB !2, !extendFrom !1*/
		t_overlapping_req_22_28 = (32'd1 <<< ({27'd0,t_overlapping_req_22_27_reg} % 32));
end
always @(posedge clk) begin
	/* t_overlapping_req: %22*/
	/*   %28 = shl i32 1, %27, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__22_12)) begin
		t_overlapping_req_22_28_reg <= t_overlapping_req_22_28;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_22_28) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_22_28_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %22*/
	/*   %29 = getelementptr inbounds [10 x i32]* @N_FREE_ADDRESS___LT, i32 0, i32 %20, !MSB !3, !LSB !2, !extendFrom !3*/
		t_overlapping_req_22_29 = (1'd0 + (4 * t_overlapping_req_14_20_reg));
end
always @(posedge clk) begin
	/* t_overlapping_req: %22*/
	/*   %29 = getelementptr inbounds [10 x i32]* @N_FREE_ADDRESS___LT, i32 0, i32 %20, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__22_10)) begin
		t_overlapping_req_22_29_reg <= t_overlapping_req_22_29;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_22_29) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_22_29_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %22*/
	/*   %30 = load i32* %29, align 4, !tbaa !12, !MSB !1, !LSB !2, !extendFrom !1*/
		t_overlapping_req_22_30 = N_FREE_ADDRESS___LT_out_a;
end
always @(posedge clk) begin
	/* t_overlapping_req: %22*/
	/*   %30 = load i32* %29, align 4, !tbaa !12, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__22_11)) begin
		t_overlapping_req_22_30_reg <= t_overlapping_req_22_30;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_22_30) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_22_30_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %22*/
	/*   %31 = xor i32 %28, %30, !MSB !1, !LSB !2, !extendFrom !1*/
		t_overlapping_req_22_31 = (t_overlapping_req_22_28_reg ^ t_overlapping_req_22_30_reg);
end
always @(*) begin
	/* t_overlapping_req: %nfree.exit*/
	/*   %34 = ptrtoint i8* %2 to i32, !MSB !1, !LSB !2, !extendFrom !1*/
		t_overlapping_req_nfreeexit_34 = t_overlapping_req_0_2_reg;
end
always @(posedge clk) begin
	/* t_overlapping_req: %nfree.exit*/
	/*   %34 = ptrtoint i8* %2 to i32, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB_nfreeexit_15)) begin
		t_overlapping_req_nfreeexit_34_reg <= t_overlapping_req_nfreeexit_34;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_nfreeexit_34) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_nfreeexit_34_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %nfree.exit*/
	/*   %35 = lshr i32 %34, 23, !MSB !10, !LSB !2, !extendFrom !10*/
		t_overlapping_req_nfreeexit_35 = (t_overlapping_req_nfreeexit_34 >>> (32'd23 % 32));
end
always @(*) begin
	/* t_overlapping_req: %nfree.exit*/
	/*   %36 = sub i32 %35, lshr (i32 ptrtoint ([256 x i32]* @N_FAST_ARENA_0_32 to i32), i32 23), !MSB !1, !LSB !2, !extendFrom !1*/
		t_overlapping_req_nfreeexit_36 = ({23'd0,t_overlapping_req_nfreeexit_35} - (`TAG_g_N_FAST_ARENA_0_32_a >>> 32'd23));
end
always @(posedge clk) begin
	/* t_overlapping_req: %nfree.exit*/
	/*   %36 = sub i32 %35, lshr (i32 ptrtoint ([256 x i32]* @N_FAST_ARENA_0_32 to i32), i32 23), !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB_nfreeexit_15)) begin
		t_overlapping_req_nfreeexit_36_reg <= t_overlapping_req_nfreeexit_36;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_nfreeexit_36) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_nfreeexit_36_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %nfree.exit*/
	/*   %37 = icmp ult i32 %36, 10, !MSB !2, !LSB !2, !extendFrom !2*/
		t_overlapping_req_nfreeexit_37 = (t_overlapping_req_nfreeexit_36_reg < 32'd10);
end
always @(*) begin
	/* t_overlapping_req: %38*/
	/*   %39 = getelementptr inbounds [10 x i8]* @N_SHIFT_LEFTS__LT, i32 0, i32 %36, !MSB !3, !LSB !2, !extendFrom !3*/
		t_overlapping_req_38_39 = (1'd0 + (1 * t_overlapping_req_nfreeexit_36_reg));
end
always @(*) begin
	/* t_overlapping_req: %38*/
	/*   %40 = load i8* %39, align 1, !tbaa !6, !MSB !11, !LSB !2, !extendFrom !11*/
		t_overlapping_req_38_40 = N_SHIFT_LEFTS__LT_out_a;
end
always @(*) begin
	/* t_overlapping_req: %38*/
	/*   %41 = zext i8 %40 to i32, !MSB !11, !LSB !2, !extendFrom !11*/
		t_overlapping_req_38_41 = t_overlapping_req_38_40;
end
always @(*) begin
	/* t_overlapping_req: %38*/
	/*   %42 = lshr i32 %34, %41, !MSB !4, !LSB !2, !extendFrom !4*/
		t_overlapping_req_38_42 = (t_overlapping_req_nfreeexit_34_reg >>> (t_overlapping_req_38_41 % 32));
end
always @(*) begin
	/* t_overlapping_req: %38*/
	/*   %43 = and i32 %42, 31, !MSB !4, !LSB !2, !extendFrom !4*/
		t_overlapping_req_38_43 = (t_overlapping_req_38_42 & 32'd31);
end
always @(posedge clk) begin
	/* t_overlapping_req: %38*/
	/*   %43 = and i32 %42, 31, !MSB !4, !LSB !2, !extendFrom !4*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__38_18)) begin
		t_overlapping_req_38_43_reg <= t_overlapping_req_38_43;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_38_43) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_38_43_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %38*/
	/*   %44 = shl i32 1, %43, !MSB !1, !LSB !2, !extendFrom !1*/
		t_overlapping_req_38_44 = (32'd1 <<< ({27'd0,t_overlapping_req_38_43_reg} % 32));
end
always @(posedge clk) begin
	/* t_overlapping_req: %38*/
	/*   %44 = shl i32 1, %43, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__38_19)) begin
		t_overlapping_req_38_44_reg <= t_overlapping_req_38_44;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_38_44) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_38_44_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %38*/
	/*   %45 = getelementptr inbounds [10 x i32]* @N_FREE_ADDRESS___LT, i32 0, i32 %36, !MSB !3, !LSB !2, !extendFrom !3*/
		t_overlapping_req_38_45 = (1'd0 + (4 * t_overlapping_req_nfreeexit_36_reg));
end
always @(posedge clk) begin
	/* t_overlapping_req: %38*/
	/*   %45 = getelementptr inbounds [10 x i32]* @N_FREE_ADDRESS___LT, i32 0, i32 %36, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__38_17)) begin
		t_overlapping_req_38_45_reg <= t_overlapping_req_38_45;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_38_45) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_38_45_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %38*/
	/*   %46 = load i32* %45, align 4, !tbaa !12, !MSB !1, !LSB !2, !extendFrom !1*/
		t_overlapping_req_38_46 = N_FREE_ADDRESS___LT_out_a;
end
always @(posedge clk) begin
	/* t_overlapping_req: %38*/
	/*   %46 = load i32* %45, align 4, !tbaa !12, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__38_18)) begin
		t_overlapping_req_38_46_reg <= t_overlapping_req_38_46;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_overlapping_req_38_46) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_38_46_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_overlapping_req: %38*/
	/*   %47 = xor i32 %44, %46, !MSB !1, !LSB !2, !extendFrom !1*/
		t_overlapping_req_38_47 = (t_overlapping_req_38_44_reg ^ t_overlapping_req_38_46_reg);
end
always @(posedge clk) begin
	if ((cur_state == LEGUP_0)) begin
		finish <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to finish"); $finish; end
		/* synthesis translate_on */
	end
	/* t_overlapping_req: %nfree.exit1*/
	/*   ret void, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB_nfreeexit1_22)) begin
		finish <= (fsm_stall == 1'd0);
		/* synthesis translate_off */
		if (start == 1'b0 && ^((fsm_stall == 1'd0)) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to finish"); $finish; end
		/* synthesis translate_on */
	end
end
always @(posedge clk) begin
	nmalloc_start <= 1'd0;
	/* t_overlapping_req: %0*/
	/*   %2 = tail call fastcc noalias i8* @nmalloc(i32 34) #2, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__0_1)) begin
		nmalloc_start <= 1'd1;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd1) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_start"); $finish; end
		/* synthesis translate_on */
	end
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__0_3)) begin
		nmalloc_start <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_start"); $finish; end
		/* synthesis translate_on */
	end
	/* t_overlapping_req: %0*/
	/*   %6 = tail call fastcc noalias i8* @nmalloc(i32 24) #2, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__0_3)) begin
		nmalloc_start <= 1'd1;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd1) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_start"); $finish; end
		/* synthesis translate_on */
	end
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__0_5)) begin
		nmalloc_start <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_start"); $finish; end
		/* synthesis translate_on */
	end
end
always @(posedge clk) begin
	nmalloc_arg_bytes <= 5'd0;
	/* t_overlapping_req: %0*/
	/*   %2 = tail call fastcc noalias i8* @nmalloc(i32 34) #2, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__0_1)) begin
		nmalloc_arg_bytes <= 32'd34;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(32'd34) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_arg_bytes"); $finish; end
		/* synthesis translate_on */
	end
	/* t_overlapping_req: %0*/
	/*   %6 = tail call fastcc noalias i8* @nmalloc(i32 24) #2, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__0_3)) begin
		nmalloc_arg_bytes <= 32'd24;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(32'd24) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_arg_bytes"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	N_SHIFT_LEFTS__LT_enable_a = 1'd0;
	/* t_overlapping_req: %22*/
	/*   %24 = load i8* %23, align 1, !tbaa !6, !MSB !11, !LSB !2, !extendFrom !11*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__22_10)) begin
		N_SHIFT_LEFTS__LT_enable_a = 1'd1;
	end
	/* t_overlapping_req: %38*/
	/*   %40 = load i8* %39, align 1, !tbaa !6, !MSB !11, !LSB !2, !extendFrom !11*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__38_17)) begin
		N_SHIFT_LEFTS__LT_enable_a = 1'd1;
	end
end
always @(*) begin
	N_SHIFT_LEFTS__LT_address_a = 4'd0;
	/* t_overlapping_req: %22*/
	/*   %24 = load i8* %23, align 1, !tbaa !6, !MSB !11, !LSB !2, !extendFrom !11*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__22_10)) begin
		N_SHIFT_LEFTS__LT_address_a = (t_overlapping_req_22_23 >>> 3'd0);
	end
	/* t_overlapping_req: %38*/
	/*   %40 = load i8* %39, align 1, !tbaa !6, !MSB !11, !LSB !2, !extendFrom !11*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__38_17)) begin
		N_SHIFT_LEFTS__LT_address_a = (t_overlapping_req_38_39 >>> 3'd0);
	end
end
assign N_SHIFT_LEFTS__LT_enable_b = 1'd0;
assign N_SHIFT_LEFTS__LT_address_b = 4'd0;
always @(*) begin
	N_FREE_ADDRESS___LT_write_enable_a = 1'd0;
	/* t_overlapping_req: %22*/
	/*   store i32 %31, i32* %29, align 4, !tbaa !12, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__22_13)) begin
		N_FREE_ADDRESS___LT_write_enable_a = 1'd1;
	end
	/* t_overlapping_req: %38*/
	/*   store i32 %47, i32* %45, align 4, !tbaa !12, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__38_20)) begin
		N_FREE_ADDRESS___LT_write_enable_a = 1'd1;
	end
end
always @(*) begin
	N_FREE_ADDRESS___LT_in_a = 0;
	/* t_overlapping_req: %22*/
	/*   store i32 %31, i32* %29, align 4, !tbaa !12, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__22_13)) begin
		N_FREE_ADDRESS___LT_in_a = t_overlapping_req_22_31;
	end
	/* t_overlapping_req: %38*/
	/*   store i32 %47, i32* %45, align 4, !tbaa !12, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__38_20)) begin
		N_FREE_ADDRESS___LT_in_a = t_overlapping_req_38_47;
	end
end
assign N_FREE_ADDRESS___LT_byteena_a = 1'd1;
always @(*) begin
	N_FREE_ADDRESS___LT_enable_a = 1'd0;
	/* t_overlapping_req: %22*/
	/*   %30 = load i32* %29, align 4, !tbaa !12, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__22_10)) begin
		N_FREE_ADDRESS___LT_enable_a = 1'd1;
	end
	/* t_overlapping_req: %22*/
	/*   store i32 %31, i32* %29, align 4, !tbaa !12, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__22_13)) begin
		N_FREE_ADDRESS___LT_enable_a = 1'd1;
	end
	/* t_overlapping_req: %38*/
	/*   %46 = load i32* %45, align 4, !tbaa !12, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__38_17)) begin
		N_FREE_ADDRESS___LT_enable_a = 1'd1;
	end
	/* t_overlapping_req: %38*/
	/*   store i32 %47, i32* %45, align 4, !tbaa !12, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__38_20)) begin
		N_FREE_ADDRESS___LT_enable_a = 1'd1;
	end
end
always @(*) begin
	N_FREE_ADDRESS___LT_address_a = 4'd0;
	/* t_overlapping_req: %22*/
	/*   %30 = load i32* %29, align 4, !tbaa !12, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__22_10)) begin
		N_FREE_ADDRESS___LT_address_a = (t_overlapping_req_22_29 >>> 3'd2);
	end
	/* t_overlapping_req: %22*/
	/*   store i32 %31, i32* %29, align 4, !tbaa !12, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__22_13)) begin
		N_FREE_ADDRESS___LT_address_a = (t_overlapping_req_22_29_reg >>> 3'd2);
	end
	/* t_overlapping_req: %38*/
	/*   %46 = load i32* %45, align 4, !tbaa !12, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__38_17)) begin
		N_FREE_ADDRESS___LT_address_a = (t_overlapping_req_38_45 >>> 3'd2);
	end
	/* t_overlapping_req: %38*/
	/*   store i32 %47, i32* %45, align 4, !tbaa !12, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_overlapping_req_BB__38_20)) begin
		N_FREE_ADDRESS___LT_address_a = (t_overlapping_req_38_45_reg >>> 3'd2);
	end
end
assign N_FREE_ADDRESS___LT_write_enable_b = 1'd0;
assign N_FREE_ADDRESS___LT_in_b = 0;
assign N_FREE_ADDRESS___LT_byteena_b = 1'd1;
assign N_FREE_ADDRESS___LT_enable_b = 1'd0;
assign N_FREE_ADDRESS___LT_address_b = 4'd0;

endmodule
`timescale 1 ns / 1 ns
module t_large_req
(
	clk,
	clk2x,
	clk1x_follower,
	reset,
	memory_controller_waitrequest,
	start,
	finish,
	nmalloc_start,
	nmalloc_finish,
	nmalloc_return_val,
	nmalloc_arg_bytes,
	N_SHIFT_LEFTS__LT_enable_a,
	N_SHIFT_LEFTS__LT_address_a,
	N_SHIFT_LEFTS__LT_out_a,
	N_SHIFT_LEFTS__LT_enable_b,
	N_SHIFT_LEFTS__LT_address_b,
	N_SHIFT_LEFTS__LT_out_b,
	N_FREE_ADDRESS___LT_write_enable_a,
	N_FREE_ADDRESS___LT_in_a,
	N_FREE_ADDRESS___LT_byteena_a,
	N_FREE_ADDRESS___LT_enable_a,
	N_FREE_ADDRESS___LT_address_a,
	N_FREE_ADDRESS___LT_out_a,
	N_FREE_ADDRESS___LT_write_enable_b,
	N_FREE_ADDRESS___LT_in_b,
	N_FREE_ADDRESS___LT_byteena_b,
	N_FREE_ADDRESS___LT_enable_b,
	N_FREE_ADDRESS___LT_address_b,
	N_FREE_ADDRESS___LT_out_b
);

parameter [3:0] LEGUP_0 = 4'd0;
parameter [3:0] LEGUP_F_t_large_req_BB__0_1 = 4'd1;
parameter [3:0] LEGUP_F_t_large_req_BB__0_3 = 4'd3;
parameter [3:0] LEGUP_F_t_large_req_BB__4_4 = 4'd4;
parameter [3:0] LEGUP_F_t_large_req_BB__4_5 = 4'd5;
parameter [3:0] LEGUP_F_t_large_req_BB__4_6 = 4'd6;
parameter [3:0] LEGUP_F_t_large_req_BB__10_7 = 4'd7;
parameter [3:0] LEGUP_F_t_large_req_BB__10_8 = 4'd8;
parameter [3:0] LEGUP_F_t_large_req_BB__16_9 = 4'd9;
parameter [3:0] LEGUP_F_t_large_req_BB__16_10 = 4'd10;
parameter [3:0] LEGUP_F_t_large_req_BB__16_11 = 4'd11;
parameter [3:0] LEGUP_F_t_large_req_BB__16_12 = 4'd12;
parameter [3:0] LEGUP_F_t_large_req_BB__16_13 = 4'd13;
parameter [3:0] LEGUP_F_t_large_req_BB_nfreeexit_14 = 4'd14;
parameter [3:0] LEGUP_function_call_2 = 4'd2;

input  clk;
input  clk2x;
input  clk1x_follower;
input  reset;
input  memory_controller_waitrequest;
input  start;
output reg  finish;
output reg  nmalloc_start;
input  nmalloc_finish;
input [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_return_val;
output reg [31:0] nmalloc_arg_bytes;
output reg  N_SHIFT_LEFTS__LT_enable_a;
output reg [3:0] N_SHIFT_LEFTS__LT_address_a;
input [7:0] N_SHIFT_LEFTS__LT_out_a;
output  N_SHIFT_LEFTS__LT_enable_b;
output [3:0] N_SHIFT_LEFTS__LT_address_b;
input [7:0] N_SHIFT_LEFTS__LT_out_b;
output reg  N_FREE_ADDRESS___LT_write_enable_a;
output reg [31:0] N_FREE_ADDRESS___LT_in_a;
output  N_FREE_ADDRESS___LT_byteena_a;
output reg  N_FREE_ADDRESS___LT_enable_a;
output reg [3:0] N_FREE_ADDRESS___LT_address_a;
input [31:0] N_FREE_ADDRESS___LT_out_a;
output  N_FREE_ADDRESS___LT_write_enable_b;
output [31:0] N_FREE_ADDRESS___LT_in_b;
output  N_FREE_ADDRESS___LT_byteena_b;
output  N_FREE_ADDRESS___LT_enable_b;
output [3:0] N_FREE_ADDRESS___LT_address_b;
input [31:0] N_FREE_ADDRESS___LT_out_b;
reg [3:0] cur_state;
reg [3:0] next_state;
reg  fsm_stall;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_large_req_0_2;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_large_req_0_2_reg;
reg [4:0] t_large_req_4_5;
reg [4:0] t_large_req_4_5_reg;
reg [7:0] t_large_req_4_6;
reg [7:0] t_large_req_4_6_reg;
reg [5:0] t_large_req_4_9;
reg [5:0] t_large_req_4_9_reg;
reg  t_large_req_4_exitcond;
reg  t_large_req_4_exitcond_reg;
reg [31:0] t_large_req_10_12;
reg [31:0] t_large_req_10_12_reg;
reg [8:0] t_large_req_10_13;
reg [31:0] t_large_req_10_14;
reg [31:0] t_large_req_10_14_reg;
reg  t_large_req_10_15;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_large_req_16_17;
reg [7:0] t_large_req_16_18;
reg [7:0] t_large_req_16_19;
reg [4:0] t_large_req_16_20;
reg [4:0] t_large_req_16_21;
reg [4:0] t_large_req_16_21_reg;
reg [31:0] t_large_req_16_22;
reg [31:0] t_large_req_16_22_reg;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_large_req_16_23;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_large_req_16_23_reg;
reg [31:0] t_large_req_16_24;
reg [31:0] t_large_req_16_24_reg;
reg [31:0] t_large_req_16_25;
reg  legup_function_call;
reg  legup_mult_t_large_req_4_6_en;
reg [7:0] t_large_req_4_6_stage0_reg;

// Local Rams

// End Local Rams

/* Unsynthesizable Statements */
/* synthesis translate_off */
always @(posedge clk)
	if (!fsm_stall) begin
	/* t_large_req: %0*/
	/*   %1 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([17 x i8]* @.str17, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_large_req_BB__0_1)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[pre_malloc][3]\n");
	end
	/* t_large_req: %0*/
	/*   %3 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([18 x i8]* @.str18, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_large_req_BB__0_3)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[post_malloc][3]\n");
	end
	/* t_large_req: %4*/
	/*   %8 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([20 x i8]* @.str19, i32 0, i32 0), i32 %6) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_large_req_BB__4_6)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("test_large[i] = %d\n", t_large_req_4_6_reg);
		// to fix quartus warning
		if (reset == 1'b0 && ^(t_large_req_4_6_reg) === 1'bX) finish <= 0;
	end
	/* t_large_req: %10*/
	/*   %11 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([15 x i8]* @.str20, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_large_req_BB__10_7)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[pre_free][3]\n");
	end
	/* t_large_req: %nfree.exit*/
	/*   %26 = tail call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([16 x i8]* @.str21, i32 0, i32 0)) #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_large_req_BB_nfreeexit_14)) begin
		$write("Cycle: %d Time: %d    ", ($time-50)/20, $time);
$write("[post_free][3]\n");
	end
end
/* synthesis translate_on */
always @(posedge clk) begin
if (reset == 1'b1)
	cur_state <= LEGUP_0;
else if (!fsm_stall)
	cur_state <= next_state;
end

always @(*)
begin
next_state = cur_state;
case(cur_state)  // synthesis parallel_case  
LEGUP_0:
	if ((fsm_stall == 1'd0) && (start == 1'd1))
		next_state = LEGUP_F_t_large_req_BB__0_1;
LEGUP_F_t_large_req_BB__0_1:
		next_state = LEGUP_function_call_2;
LEGUP_F_t_large_req_BB__0_3:
		next_state = LEGUP_F_t_large_req_BB__4_4;
LEGUP_F_t_large_req_BB__10_7:
		next_state = LEGUP_F_t_large_req_BB__10_8;
LEGUP_F_t_large_req_BB__10_8:
	if ((fsm_stall == 1'd0) && (t_large_req_10_15 == 1'd1))
		next_state = LEGUP_F_t_large_req_BB__16_9;
	else if ((fsm_stall == 1'd0) && (t_large_req_10_15 == 1'd0))
		next_state = LEGUP_F_t_large_req_BB_nfreeexit_14;
LEGUP_F_t_large_req_BB__16_10:
		next_state = LEGUP_F_t_large_req_BB__16_11;
LEGUP_F_t_large_req_BB__16_11:
		next_state = LEGUP_F_t_large_req_BB__16_12;
LEGUP_F_t_large_req_BB__16_12:
		next_state = LEGUP_F_t_large_req_BB__16_13;
LEGUP_F_t_large_req_BB__16_13:
		next_state = LEGUP_F_t_large_req_BB_nfreeexit_14;
LEGUP_F_t_large_req_BB__16_9:
		next_state = LEGUP_F_t_large_req_BB__16_10;
LEGUP_F_t_large_req_BB__4_4:
		next_state = LEGUP_F_t_large_req_BB__4_5;
LEGUP_F_t_large_req_BB__4_5:
		next_state = LEGUP_F_t_large_req_BB__4_6;
LEGUP_F_t_large_req_BB__4_6:
	if ((fsm_stall == 1'd0) && (t_large_req_4_exitcond_reg == 1'd1))
		next_state = LEGUP_F_t_large_req_BB__10_7;
	else if ((fsm_stall == 1'd0) && (t_large_req_4_exitcond_reg == 1'd0))
		next_state = LEGUP_F_t_large_req_BB__4_4;
LEGUP_F_t_large_req_BB_nfreeexit_14:
		next_state = LEGUP_0;
LEGUP_function_call_2:
	if ((fsm_stall == 1'd0) && (nmalloc_finish == 1'd1))
		next_state = LEGUP_F_t_large_req_BB__0_3;
default:
	next_state = cur_state;
endcase

end
always @(*) begin
	fsm_stall = 1'd0;
	if (reset) begin
		fsm_stall = 1'd0;
	end
	if (memory_controller_waitrequest) begin
		fsm_stall = 1'd1;
	end
end
always @(*) begin
	/* t_large_req: %0*/
	/*   %2 = tail call fastcc noalias i8* @nmalloc(i32 255) #2, !MSB !3, !LSB !2, !extendFrom !3*/
		t_large_req_0_2 = nmalloc_return_val;
end
always @(posedge clk) begin
	/* t_large_req: %0*/
	/*   %2 = tail call fastcc noalias i8* @nmalloc(i32 255) #2, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_function_call_2)) begin
		t_large_req_0_2_reg <= t_large_req_0_2;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_large_req_0_2) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_0_2_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_large_req: %4*/
	/*   %5 = phi i32 [ 0, %0 ], [ %9, %4 ], !MSB !4, !LSB !2, !extendFrom !4*/
	if (((cur_state == LEGUP_F_t_large_req_BB__0_3) & (fsm_stall == 1'd0))) begin
		t_large_req_4_5 = 32'd0;
	end
	/* t_large_req: %4*/
	/*   %5 = phi i32 [ 0, %0 ], [ %9, %4 ], !MSB !4, !LSB !2, !extendFrom !4*/
	else /* if ((((cur_state == LEGUP_F_t_large_req_BB__4_6) & (fsm_stall == 1'd0)) & (t_large_req_4_exitcond_reg == 1'd0))) */ begin
		t_large_req_4_5 = t_large_req_4_9_reg;
	end
end
always @(posedge clk) begin
	/* t_large_req: %4*/
	/*   %5 = phi i32 [ 0, %0 ], [ %9, %4 ], !MSB !4, !LSB !2, !extendFrom !4*/
	if (((cur_state == LEGUP_F_t_large_req_BB__0_3) & (fsm_stall == 1'd0))) begin
		t_large_req_4_5_reg <= t_large_req_4_5;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_large_req_4_5) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_4_5_reg"); $finish; end
		/* synthesis translate_on */
	end
	/* t_large_req: %4*/
	/*   %5 = phi i32 [ 0, %0 ], [ %9, %4 ], !MSB !4, !LSB !2, !extendFrom !4*/
	if ((((cur_state == LEGUP_F_t_large_req_BB__4_6) & (fsm_stall == 1'd0)) & (t_large_req_4_exitcond_reg == 1'd0))) begin
		t_large_req_4_5_reg <= t_large_req_4_5;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_large_req_4_5) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_4_5_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	t_large_req_4_6 = t_large_req_4_6_stage0_reg;
end
always @(posedge clk) begin
	/* t_large_req: %4*/
	/*   %6 = mul i32 %5, 5, !MSB !5, !LSB !2, !extendFrom !5*/
	if ((cur_state == LEGUP_F_t_large_req_BB__4_5)) begin
		t_large_req_4_6_reg <= t_large_req_4_6;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_large_req_4_6) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_4_6_reg"); $finish; end
		/* synthesis translate_on */
	end
	/* t_large_req: %4*/
	/*   %6 = mul i32 %5, 5, !MSB !5, !LSB !2, !extendFrom !5*/
	if ((cur_state == LEGUP_F_t_large_req_BB__4_5)) begin
		t_large_req_4_6_reg <= t_large_req_4_6;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_large_req_4_6) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_4_6_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_large_req: %4*/
	/*   %9 = add nsw i32 %5, 1, !MSB !11, !LSB !2, !extendFrom !11*/
		t_large_req_4_9 = ({1'd0,t_large_req_4_5_reg} + 32'd1);
end
always @(posedge clk) begin
	/* t_large_req: %4*/
	/*   %9 = add nsw i32 %5, 1, !MSB !11, !LSB !2, !extendFrom !11*/
	if ((cur_state == LEGUP_F_t_large_req_BB__4_4)) begin
		t_large_req_4_9_reg <= t_large_req_4_9;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_large_req_4_9) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_4_9_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_large_req: %4*/
	/*   %exitcond = icmp eq i32 %9, 24, !MSB !2, !LSB !2, !extendFrom !2*/
		t_large_req_4_exitcond = (t_large_req_4_9_reg == 32'd24);
end
always @(posedge clk) begin
	/* t_large_req: %4*/
	/*   %exitcond = icmp eq i32 %9, 24, !MSB !2, !LSB !2, !extendFrom !2*/
	if ((cur_state == LEGUP_F_t_large_req_BB__4_5)) begin
		t_large_req_4_exitcond_reg <= t_large_req_4_exitcond;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_large_req_4_exitcond) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_4_exitcond_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_large_req: %10*/
	/*   %12 = ptrtoint i8* %2 to i32, !MSB !1, !LSB !2, !extendFrom !1*/
		t_large_req_10_12 = t_large_req_0_2_reg;
end
always @(posedge clk) begin
	/* t_large_req: %10*/
	/*   %12 = ptrtoint i8* %2 to i32, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_large_req_BB__10_7)) begin
		t_large_req_10_12_reg <= t_large_req_10_12;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_large_req_10_12) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_10_12_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_large_req: %10*/
	/*   %13 = lshr i32 %12, 23, !MSB !12, !LSB !2, !extendFrom !12*/
		t_large_req_10_13 = (t_large_req_10_12 >>> (32'd23 % 32));
end
always @(*) begin
	/* t_large_req: %10*/
	/*   %14 = sub i32 %13, lshr (i32 ptrtoint ([256 x i32]* @N_FAST_ARENA_0_32 to i32), i32 23), !MSB !1, !LSB !2, !extendFrom !1*/
		t_large_req_10_14 = ({23'd0,t_large_req_10_13} - (`TAG_g_N_FAST_ARENA_0_32_a >>> 32'd23));
end
always @(posedge clk) begin
	/* t_large_req: %10*/
	/*   %14 = sub i32 %13, lshr (i32 ptrtoint ([256 x i32]* @N_FAST_ARENA_0_32 to i32), i32 23), !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_large_req_BB__10_7)) begin
		t_large_req_10_14_reg <= t_large_req_10_14;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_large_req_10_14) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_10_14_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_large_req: %10*/
	/*   %15 = icmp ult i32 %14, 10, !MSB !2, !LSB !2, !extendFrom !2*/
		t_large_req_10_15 = (t_large_req_10_14_reg < 32'd10);
end
always @(*) begin
	/* t_large_req: %16*/
	/*   %17 = getelementptr inbounds [10 x i8]* @N_SHIFT_LEFTS__LT, i32 0, i32 %14, !MSB !3, !LSB !2, !extendFrom !3*/
		t_large_req_16_17 = (1'd0 + (1 * t_large_req_10_14_reg));
end
always @(*) begin
	/* t_large_req: %16*/
	/*   %18 = load i8* %17, align 1, !tbaa !13, !MSB !5, !LSB !2, !extendFrom !5*/
		t_large_req_16_18 = N_SHIFT_LEFTS__LT_out_a;
end
always @(*) begin
	/* t_large_req: %16*/
	/*   %19 = zext i8 %18 to i32, !MSB !5, !LSB !2, !extendFrom !5*/
		t_large_req_16_19 = t_large_req_16_18;
end
always @(*) begin
	/* t_large_req: %16*/
	/*   %20 = lshr i32 %12, %19, !MSB !4, !LSB !2, !extendFrom !4*/
		t_large_req_16_20 = (t_large_req_10_12_reg >>> (t_large_req_16_19 % 32));
end
always @(*) begin
	/* t_large_req: %16*/
	/*   %21 = and i32 %20, 31, !MSB !4, !LSB !2, !extendFrom !4*/
		t_large_req_16_21 = (t_large_req_16_20 & 32'd31);
end
always @(posedge clk) begin
	/* t_large_req: %16*/
	/*   %21 = and i32 %20, 31, !MSB !4, !LSB !2, !extendFrom !4*/
	if ((cur_state == LEGUP_F_t_large_req_BB__16_10)) begin
		t_large_req_16_21_reg <= t_large_req_16_21;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_large_req_16_21) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_16_21_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_large_req: %16*/
	/*   %22 = shl i32 1, %21, !MSB !1, !LSB !2, !extendFrom !1*/
		t_large_req_16_22 = (32'd1 <<< ({27'd0,t_large_req_16_21_reg} % 32));
end
always @(posedge clk) begin
	/* t_large_req: %16*/
	/*   %22 = shl i32 1, %21, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_large_req_BB__16_11)) begin
		t_large_req_16_22_reg <= t_large_req_16_22;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_large_req_16_22) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_16_22_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_large_req: %16*/
	/*   %23 = getelementptr inbounds [10 x i32]* @N_FREE_ADDRESS___LT, i32 0, i32 %14, !MSB !3, !LSB !2, !extendFrom !3*/
		t_large_req_16_23 = (1'd0 + (4 * t_large_req_10_14_reg));
end
always @(posedge clk) begin
	/* t_large_req: %16*/
	/*   %23 = getelementptr inbounds [10 x i32]* @N_FREE_ADDRESS___LT, i32 0, i32 %14, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_large_req_BB__16_9)) begin
		t_large_req_16_23_reg <= t_large_req_16_23;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_large_req_16_23) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_16_23_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_large_req: %16*/
	/*   %24 = load i32* %23, align 4, !tbaa !7, !MSB !1, !LSB !2, !extendFrom !1*/
		t_large_req_16_24 = N_FREE_ADDRESS___LT_out_a;
end
always @(posedge clk) begin
	/* t_large_req: %16*/
	/*   %24 = load i32* %23, align 4, !tbaa !7, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_large_req_BB__16_10)) begin
		t_large_req_16_24_reg <= t_large_req_16_24;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(t_large_req_16_24) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_16_24_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	/* t_large_req: %16*/
	/*   %25 = xor i32 %22, %24, !MSB !1, !LSB !2, !extendFrom !1*/
		t_large_req_16_25 = (t_large_req_16_22_reg ^ t_large_req_16_24_reg);
end
always @(*) begin
	legup_function_call = 1'd0;
	/* t_large_req: %0*/
	/*   %2 = tail call fastcc noalias i8* @nmalloc(i32 255) #2, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_function_call_2)) begin
		legup_function_call = 1'd1;
	end
end
always @(*) begin
	legup_mult_t_large_req_4_6_en = ~((fsm_stall | legup_function_call));
end
always @(posedge clk) begin
	/* t_large_req: %4*/
	/*   %6 = mul i32 %5, 5, !MSB !5, !LSB !2, !extendFrom !5*/
	if ((legup_mult_t_large_req_4_6_en == 1'd1)) begin
		t_large_req_4_6_stage0_reg <= ({3'd0,t_large_req_4_5_reg} * 32'd5);
	end
end
always @(posedge clk) begin
	if ((cur_state == LEGUP_0)) begin
		finish <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to finish"); $finish; end
		/* synthesis translate_on */
	end
	/* t_large_req: %nfree.exit*/
	/*   ret void, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_large_req_BB_nfreeexit_14)) begin
		finish <= (fsm_stall == 1'd0);
		/* synthesis translate_off */
		if (start == 1'b0 && ^((fsm_stall == 1'd0)) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to finish"); $finish; end
		/* synthesis translate_on */
	end
end
always @(posedge clk) begin
	nmalloc_start <= 1'd0;
	/* t_large_req: %0*/
	/*   %2 = tail call fastcc noalias i8* @nmalloc(i32 255) #2, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_large_req_BB__0_1)) begin
		nmalloc_start <= 1'd1;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd1) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_start"); $finish; end
		/* synthesis translate_on */
	end
	if ((cur_state == LEGUP_F_t_large_req_BB__0_3)) begin
		nmalloc_start <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_start"); $finish; end
		/* synthesis translate_on */
	end
end
always @(posedge clk) begin
	nmalloc_arg_bytes <= 8'd0;
	/* t_large_req: %0*/
	/*   %2 = tail call fastcc noalias i8* @nmalloc(i32 255) #2, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_large_req_BB__0_1)) begin
		nmalloc_arg_bytes <= 32'd255;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(32'd255) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_arg_bytes"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	N_SHIFT_LEFTS__LT_enable_a = 1'd0;
	/* t_large_req: %16*/
	/*   %18 = load i8* %17, align 1, !tbaa !13, !MSB !5, !LSB !2, !extendFrom !5*/
	if ((cur_state == LEGUP_F_t_large_req_BB__16_9)) begin
		N_SHIFT_LEFTS__LT_enable_a = 1'd1;
	end
end
always @(*) begin
	N_SHIFT_LEFTS__LT_address_a = 4'd0;
	/* t_large_req: %16*/
	/*   %18 = load i8* %17, align 1, !tbaa !13, !MSB !5, !LSB !2, !extendFrom !5*/
	if ((cur_state == LEGUP_F_t_large_req_BB__16_9)) begin
		N_SHIFT_LEFTS__LT_address_a = (t_large_req_16_17 >>> 3'd0);
	end
end
assign N_SHIFT_LEFTS__LT_enable_b = 1'd0;
assign N_SHIFT_LEFTS__LT_address_b = 4'd0;
always @(*) begin
	N_FREE_ADDRESS___LT_write_enable_a = 1'd0;
	/* t_large_req: %16*/
	/*   store i32 %25, i32* %23, align 4, !tbaa !7, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_large_req_BB__16_12)) begin
		N_FREE_ADDRESS___LT_write_enable_a = 1'd1;
	end
end
always @(*) begin
	N_FREE_ADDRESS___LT_in_a = 0;
	/* t_large_req: %16*/
	/*   store i32 %25, i32* %23, align 4, !tbaa !7, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_large_req_BB__16_12)) begin
		N_FREE_ADDRESS___LT_in_a = t_large_req_16_25;
	end
end
assign N_FREE_ADDRESS___LT_byteena_a = 1'd1;
always @(*) begin
	N_FREE_ADDRESS___LT_enable_a = 1'd0;
	/* t_large_req: %16*/
	/*   %24 = load i32* %23, align 4, !tbaa !7, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_large_req_BB__16_9)) begin
		N_FREE_ADDRESS___LT_enable_a = 1'd1;
	end
	/* t_large_req: %16*/
	/*   store i32 %25, i32* %23, align 4, !tbaa !7, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_large_req_BB__16_12)) begin
		N_FREE_ADDRESS___LT_enable_a = 1'd1;
	end
end
always @(*) begin
	N_FREE_ADDRESS___LT_address_a = 4'd0;
	/* t_large_req: %16*/
	/*   %24 = load i32* %23, align 4, !tbaa !7, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_t_large_req_BB__16_9)) begin
		N_FREE_ADDRESS___LT_address_a = (t_large_req_16_23 >>> 3'd2);
	end
	/* t_large_req: %16*/
	/*   store i32 %25, i32* %23, align 4, !tbaa !7, !MSB !3, !LSB !2, !extendFrom !3*/
	if ((cur_state == LEGUP_F_t_large_req_BB__16_12)) begin
		N_FREE_ADDRESS___LT_address_a = (t_large_req_16_23_reg >>> 3'd2);
	end
end
assign N_FREE_ADDRESS___LT_write_enable_b = 1'd0;
assign N_FREE_ADDRESS___LT_in_b = 0;
assign N_FREE_ADDRESS___LT_byteena_b = 1'd1;
assign N_FREE_ADDRESS___LT_enable_b = 1'd0;
assign N_FREE_ADDRESS___LT_address_b = 4'd0;

endmodule
`timescale 1 ns / 1 ns
module main
(
	clk,
	clk2x,
	clk1x_follower,
	reset,
	memory_controller_waitrequest,
	start,
	finish,
	return_val,
	t_init_cycle_req_start,
	t_init_cycle_req_finish,
	t_overlapping_req_start,
	t_overlapping_req_finish,
	t_large_req_start,
	t_large_req_finish
);

parameter [2:0] LEGUP_0 = 3'd0;
parameter [2:0] LEGUP_F_main_BB__0_1 = 3'd1;
parameter [2:0] LEGUP_F_main_BB__0_3 = 3'd3;
parameter [2:0] LEGUP_F_main_BB__0_5 = 3'd5;
parameter [2:0] LEGUP_F_main_BB__0_7 = 3'd7;
parameter [2:0] LEGUP_function_call_2 = 3'd2;
parameter [2:0] LEGUP_function_call_4 = 3'd4;
parameter [2:0] LEGUP_function_call_6 = 3'd6;

input  clk;
input  clk2x;
input  clk1x_follower;
input  reset;
input  memory_controller_waitrequest;
input  start;
output reg  finish;
output reg [31:0] return_val;
output reg  t_init_cycle_req_start;
input  t_init_cycle_req_finish;
output reg  t_overlapping_req_start;
input  t_overlapping_req_finish;
output reg  t_large_req_start;
input  t_large_req_finish;
reg [2:0] cur_state;
reg [2:0] next_state;
reg  fsm_stall;

// Local Rams

// End Local Rams

always @(posedge clk) begin
if (reset == 1'b1)
	cur_state <= LEGUP_0;
else if (!fsm_stall)
	cur_state <= next_state;
end

always @(*)
begin
next_state = cur_state;
case(cur_state)  // synthesis parallel_case  
LEGUP_0:
	if ((fsm_stall == 1'd0) && (start == 1'd1))
		next_state = LEGUP_F_main_BB__0_1;
LEGUP_F_main_BB__0_1:
		next_state = LEGUP_function_call_2;
LEGUP_F_main_BB__0_3:
		next_state = LEGUP_function_call_4;
LEGUP_F_main_BB__0_5:
		next_state = LEGUP_function_call_6;
LEGUP_F_main_BB__0_7:
		next_state = LEGUP_0;
LEGUP_function_call_2:
	if ((fsm_stall == 1'd0) && (t_init_cycle_req_finish == 1'd1))
		next_state = LEGUP_F_main_BB__0_3;
LEGUP_function_call_4:
	if ((fsm_stall == 1'd0) && (t_overlapping_req_finish == 1'd1))
		next_state = LEGUP_F_main_BB__0_5;
LEGUP_function_call_6:
	if ((fsm_stall == 1'd0) && (t_large_req_finish == 1'd1))
		next_state = LEGUP_F_main_BB__0_7;
default:
	next_state = cur_state;
endcase

end
always @(*) begin
	fsm_stall = 1'd0;
	if (reset) begin
		fsm_stall = 1'd0;
	end
	if (memory_controller_waitrequest) begin
		fsm_stall = 1'd1;
	end
end
always @(posedge clk) begin
	if ((cur_state == LEGUP_0)) begin
		finish <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to finish"); $finish; end
		/* synthesis translate_on */
	end
	/* main: %0*/
	/*   ret i32 0, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_main_BB__0_7)) begin
		finish <= (fsm_stall == 1'd0);
		/* synthesis translate_off */
		if (start == 1'b0 && ^((fsm_stall == 1'd0)) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to finish"); $finish; end
		/* synthesis translate_on */
	end
end
always @(posedge clk) begin
	if ((cur_state == LEGUP_0)) begin
		return_val <= 0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to return_val"); $finish; end
		/* synthesis translate_on */
	end
	/* main: %0*/
	/*   ret i32 0, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_main_BB__0_7)) begin
		return_val <= 32'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(32'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to return_val"); $finish; end
		/* synthesis translate_on */
	end
end
always @(posedge clk) begin
	t_init_cycle_req_start <= 1'd0;
	/* main: %0*/
	/*   tail call fastcc void @t_init_cycle_req() #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_main_BB__0_1)) begin
		t_init_cycle_req_start <= 1'd1;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd1) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_init_cycle_req_start"); $finish; end
		/* synthesis translate_on */
	end
	if ((cur_state == LEGUP_F_main_BB__0_3)) begin
		t_init_cycle_req_start <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_init_cycle_req_start"); $finish; end
		/* synthesis translate_on */
	end
end
always @(posedge clk) begin
	t_overlapping_req_start <= 1'd0;
	/* main: %0*/
	/*   tail call fastcc void @t_overlapping_req() #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_main_BB__0_3)) begin
		t_overlapping_req_start <= 1'd1;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd1) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_start"); $finish; end
		/* synthesis translate_on */
	end
	if ((cur_state == LEGUP_F_main_BB__0_5)) begin
		t_overlapping_req_start <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_start"); $finish; end
		/* synthesis translate_on */
	end
end
always @(posedge clk) begin
	t_large_req_start <= 1'd0;
	/* main: %0*/
	/*   tail call fastcc void @t_large_req() #2, !MSB !1, !LSB !2, !extendFrom !1*/
	if ((cur_state == LEGUP_F_main_BB__0_5)) begin
		t_large_req_start <= 1'd1;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd1) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_start"); $finish; end
		/* synthesis translate_on */
	end
	if ((cur_state == LEGUP_F_main_BB__0_7)) begin
		t_large_req_start <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_start"); $finish; end
		/* synthesis translate_on */
	end
end

endmodule
module ram_dual_port
(
	clk,
	clken,
	address_a,
	address_b,
	wren_a,
	wren_b,
	data_a,
	data_b,
	byteena_a,
	byteena_b,
	q_a,
	q_b
);

parameter  width_a = 1'd0;
parameter  width_b = 1'd0;
parameter  widthad_a = 1'd0;
parameter  widthad_b = 1'd0;
parameter  numwords_a = 1'd0;
parameter  numwords_b = 1'd0;
parameter  latency = 1;
parameter  init_file = "UNUSED";
parameter  width_be_a = 1'd0;
parameter  width_be_b = 1'd0;
input  clk;
input  clken;
input [(widthad_a-1):0] address_a;
input [(widthad_b-1):0] address_b;
output wire [(width_a-1):0] q_a;
output wire [(width_b-1):0] q_b;
wire [(width_a-1):0] q_a_wire;
wire [(width_b-1):0] q_b_wire;
input  wren_a;
input  wren_b;
input [(width_a-1):0] data_a;
input [(width_b-1):0] data_b;
input [width_be_a-1:0] byteena_a;
input [width_be_b-1:0] byteena_b;
reg  clk_wire;

altsyncram altsyncram_component (
	.address_a (address_a),
	.address_b (address_b),
    .clock0 (clk_wire),
    .clock1 (1'd1),
    .clocken0 (clken),
    .clocken1 (1'd1),
    .clocken2 (1'd1),
    .clocken3 (1'd1),
    .aclr0 (1'd0),
    .aclr1 (1'd0),
    .addressstall_a (1'd0),
    .addressstall_b (1'd0),
    .eccstatus (),
    .rden_a (clken),
    .rden_b (clken),
    .q_a (q_a_wire),
    .q_b (q_b_wire),
    .wren_a (wren_a),
    .wren_b (wren_b),
    .data_a (data_a),
    .data_b (data_b),
    .byteena_a (byteena_a),
    .byteena_b (byteena_b)
);
defparam
    altsyncram_component.width_byteena_a = width_be_a,
    altsyncram_component.width_byteena_b = width_be_b,
    altsyncram_component.operation_mode = "BIDIR_DUAL_PORT",
    altsyncram_component.read_during_write_mode_mixed_ports = "OLD_DATA",
    altsyncram_component.init_file = init_file,
    altsyncram_component.lpm_hint = "ENABLE_RUNTIME_MOD=NO",
    altsyncram_component.lpm_type = "altsyncram",
    altsyncram_component.power_up_uninitialized = "FALSE",
    altsyncram_component.intended_device_family = "Stratix V",
    altsyncram_component.clock_enable_input_a = "BYPASS",
    altsyncram_component.clock_enable_input_b = "BYPASS",
    altsyncram_component.clock_enable_output_a = "BYPASS",
    altsyncram_component.clock_enable_output_b = "BYPASS",
    altsyncram_component.outdata_aclr_a = "NONE",
    altsyncram_component.outdata_aclr_b = "NONE",
    altsyncram_component.outdata_reg_a = "UNREGISTERED",
    altsyncram_component.outdata_reg_b = "UNREGISTERED",
    altsyncram_component.numwords_a = numwords_a,
    altsyncram_component.numwords_b = numwords_b,
    altsyncram_component.widthad_a = widthad_a,
    altsyncram_component.widthad_b = widthad_b,
    altsyncram_component.width_a = width_a,
    altsyncram_component.width_b = width_b,
    altsyncram_component.address_reg_b = "CLOCK0",
    altsyncram_component.byteena_reg_b = "CLOCK0",
    altsyncram_component.indata_reg_b = "CLOCK0",
    altsyncram_component.wrcontrol_wraddress_reg_b = "CLOCK0";
always @(*) begin
    clk_wire = clk;
end




integer j;
reg [(width_a-1):0] q_a_reg[latency:1], q_b_reg[latency:1];

always @(*)
begin
   q_a_reg[1] <= q_a_wire;
   q_b_reg[1] <= q_b_wire;
end

always @(posedge clk)
if (clken)
begin
   for (j = 1; j < latency; j=j+1)
   begin
       q_a_reg[j+1] <= q_a_reg[j];
       q_b_reg[j+1] <= q_b_reg[j];
   end
end

assign q_a = (clken) ? q_a_reg[latency] : 0;
assign q_b = (clken) ? q_b_reg[latency] : 0;


endmodule
module rom_dual_port
(
	clk,
	clken,
	address_a,
	address_b,
	q_a,
	q_b
);

parameter  width_a = 1'd0;
parameter  width_b = 1'd0;
parameter  widthad_a = 1'd0;
parameter  widthad_b = 1'd0;
parameter  numwords_a = 1'd0;
parameter  numwords_b = 1'd0;
parameter  latency = 1;
parameter  init_file = "UNUSED";
input  clk;
input  clken;
input [(widthad_a-1):0] address_a;
input [(widthad_b-1):0] address_b;
output wire [(width_a-1):0] q_a;
output wire [(width_b-1):0] q_b;
wire [(width_a-1):0] q_a_wire;
wire [(width_b-1):0] q_b_wire;
reg  clk_wire;

altsyncram altsyncram_component (
	.address_a (address_a),
	.address_b (address_b),
    .clock0 (clk_wire),
    .clock1 (1'd1),
    .clocken0 (clken),
    .clocken1 (1'd1),
    .clocken2 (1'd1),
    .clocken3 (1'd1),
    .aclr0 (1'd0),
    .aclr1 (1'd0),
    .addressstall_a (1'd0),
    .addressstall_b (1'd0),
    .eccstatus (),
    .rden_a (clken),
    .rden_b (clken),
    .q_a (q_a_wire),
    .q_b (q_b_wire),
    .wren_a (1'd0),
    .wren_b (1'd0),
    .data_a (),
    .data_b (),
    .byteena_a (),
    .byteena_b ()
);
defparam
    altsyncram_component.operation_mode = "BIDIR_DUAL_PORT",
    altsyncram_component.read_during_write_mode_mixed_ports = "OLD_DATA",
    altsyncram_component.init_file = init_file,
    altsyncram_component.lpm_hint = "ENABLE_RUNTIME_MOD=NO",
    altsyncram_component.lpm_type = "altsyncram",
    altsyncram_component.power_up_uninitialized = "FALSE",
    altsyncram_component.intended_device_family = "Stratix V",
    altsyncram_component.clock_enable_input_a = "BYPASS",
    altsyncram_component.clock_enable_input_b = "BYPASS",
    altsyncram_component.clock_enable_output_a = "BYPASS",
    altsyncram_component.clock_enable_output_b = "BYPASS",
    altsyncram_component.outdata_aclr_a = "NONE",
    altsyncram_component.outdata_aclr_b = "NONE",
    altsyncram_component.outdata_reg_a = "UNREGISTERED",
    altsyncram_component.outdata_reg_b = "UNREGISTERED",
    altsyncram_component.numwords_a = numwords_a,
    altsyncram_component.numwords_b = numwords_b,
    altsyncram_component.widthad_a = widthad_a,
    altsyncram_component.widthad_b = widthad_b,
    altsyncram_component.width_a = width_a,
    altsyncram_component.width_b = width_b,
    altsyncram_component.address_reg_b = "CLOCK0",
    altsyncram_component.byteena_reg_b = "CLOCK0",
    altsyncram_component.indata_reg_b = "CLOCK0",
    altsyncram_component.wrcontrol_wraddress_reg_b = "CLOCK0";
always @(*) begin
    clk_wire = clk;
end




integer j;
reg [(width_a-1):0] q_a_reg[latency:1], q_b_reg[latency:1];

always @(*)
begin
   q_a_reg[1] <= q_a_wire;
   q_b_reg[1] <= q_b_wire;
end

always @(posedge clk)
if (clken)
begin
   for (j = 1; j < latency; j=j+1)
   begin
       q_a_reg[j+1] <= q_a_reg[j];
       q_b_reg[j+1] <= q_b_reg[j];
   end
end

assign q_a = (clken) ? q_a_reg[latency] : 0;
assign q_b = (clken) ? q_b_reg[latency] : 0;


endmodule
module hex_digits(x, hex_LEDs);
    input [3:0] x;
    output [6:0] hex_LEDs;
    
    assign hex_LEDs[0] = (~x[3] & ~x[2] & ~x[1] & x[0]) |
                            (~x[3] & x[2] & ~x[1] & ~x[0]) |
                            (x[3] & x[2] & ~x[1] & x[0]) |
                            (x[3] & ~x[2] & x[1] & x[0]);
    assign hex_LEDs[1] = (~x[3] & x[2] & ~x[1] & x[0]) |
                            (x[3] & x[1] & x[0]) |
                            (x[3] & x[2] & ~x[0]) |
                            (x[2] & x[1] & ~x[0]);
    assign hex_LEDs[2] = (x[3] & x[2] & ~x[0]) |
                            (x[3] & x[2] & x[1]) |
                            (~x[3] & ~x[2] & x[1] & ~x[0]);
    assign hex_LEDs[3] = (~x[3] & ~x[2] & ~x[1] & x[0]) | 
                            (~x[3] & x[2] & ~x[1] & ~x[0]) | 
                            (x[2] & x[1] & x[0]) | 
                            (x[3] & ~x[2] & x[1] & ~x[0]);
    assign hex_LEDs[4] = (~x[3] & x[0]) |
                            (~x[3] & x[2] & ~x[1]) |
                            (~x[2] & ~x[1] & x[0]);
    assign hex_LEDs[5] = (~x[3] & ~x[2] & x[0]) | 
                            (~x[3] & ~x[2] & x[1]) | 
                            (~x[3] & x[1] & x[0]) | 
                            (x[3] & x[2] & ~x[1] & x[0]);
    assign hex_LEDs[6] = (~x[3] & ~x[2] & ~x[1]) | 
                            (x[3] & x[2] & ~x[1] & ~x[0]) | 
                            (~x[3] & x[2] & x[1] & x[0]);
    
endmodule
`timescale 1 ns / 1 ns
module main_tb
(
);

reg  clk;
reg  reset;
reg  start;
reg  waitrequest;
wire [31:0] return_val;
wire  finish;
wire  clk2x;
wire  clk1x_follower;


top top_inst (
	.clk (clk),
	.clk2x (clk2x),
	.clk1x_follower (clk1x_follower),
	.reset (reset),
	.memory_controller_waitrequest (waitrequest),
	.start (start),
	.finish (finish),
	.return_val (return_val)
);


// Local Rams

// End Local Rams


initial 
    clk = 0;
always @(clk)
    clk <= #10 ~clk;

initial begin
//$monitor("At t=%t clk=%b %b %b %b %d", $time, clk, reset, start, finish, return_val);
reset <= 1;
@(negedge clk);
reset <= 0;
start <= 1;
@(negedge clk);
start <= 0;
end

always@(finish) begin
    if (finish == 1) begin
        $display("At t=%t clk=%b finish=%b return_val=%d", $time, clk, finish, return_val);
        $display("Cycles: %d", ($time-50)/20);
        $finish;
    end
end

initial begin
waitrequest <= 1;
@(negedge clk);
waitrequest <= 0;
end


endmodule
`timescale 1 ns / 1 ns
module top
(
	clk,
	clk2x,
	clk1x_follower,
	reset,
	memory_controller_waitrequest,
	start,
	finish,
	return_val
);

input  clk;
input  clk2x;
input  clk1x_follower;
input  reset;
input  memory_controller_waitrequest;
input  start;
output reg  finish;
output reg [31:0] return_val;
reg  main_inst_clk;
reg  main_inst_clk2x;
reg  main_inst_clk1x_follower;
reg  main_inst_reset;
reg  main_inst_memory_controller_waitrequest;
reg  main_inst_start;
wire  main_inst_finish;
wire [31:0] main_inst_return_val;
wire  main_inst_t_init_cycle_req_start;
reg  main_inst_t_init_cycle_req_finish;
wire  main_inst_t_overlapping_req_start;
reg  main_inst_t_overlapping_req_finish;
wire  main_inst_t_large_req_start;
reg  main_inst_t_large_req_finish;
reg  main_inst_finish_reg;
reg [31:0] main_inst_return_val_reg;
reg  t_init_cycle_req_inst_clk;
reg  t_init_cycle_req_inst_clk2x;
reg  t_init_cycle_req_inst_clk1x_follower;
reg  t_init_cycle_req_inst_reset;
reg  t_init_cycle_req_inst_memory_controller_waitrequest;
reg  t_init_cycle_req_inst_start;
wire  t_init_cycle_req_inst_finish;
wire  t_init_cycle_req_inst_nmalloc_start;
reg  t_init_cycle_req_inst_nmalloc_finish;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_init_cycle_req_inst_nmalloc_return_val;
wire [31:0] t_init_cycle_req_inst_nmalloc_arg_bytes;
wire  t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_enable_a;
wire [3:0] t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_address_a;
reg [7:0] t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_out_a;
wire  t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_enable_b;
wire [3:0] t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_address_b;
reg [7:0] t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_out_b;
wire  t_init_cycle_req_inst_N_FREE_ADDRESS___LT_write_enable_a;
wire [31:0] t_init_cycle_req_inst_N_FREE_ADDRESS___LT_in_a;
wire  t_init_cycle_req_inst_N_FREE_ADDRESS___LT_byteena_a;
wire  t_init_cycle_req_inst_N_FREE_ADDRESS___LT_enable_a;
wire [3:0] t_init_cycle_req_inst_N_FREE_ADDRESS___LT_address_a;
reg [31:0] t_init_cycle_req_inst_N_FREE_ADDRESS___LT_out_a;
wire  t_init_cycle_req_inst_N_FREE_ADDRESS___LT_write_enable_b;
wire [31:0] t_init_cycle_req_inst_N_FREE_ADDRESS___LT_in_b;
wire  t_init_cycle_req_inst_N_FREE_ADDRESS___LT_byteena_b;
wire  t_init_cycle_req_inst_N_FREE_ADDRESS___LT_enable_b;
wire [3:0] t_init_cycle_req_inst_N_FREE_ADDRESS___LT_address_b;
reg [31:0] t_init_cycle_req_inst_N_FREE_ADDRESS___LT_out_b;
reg  t_init_cycle_req_inst_finish_reg;
reg  N_SHIFT_LEFTS__LT_inst_clk;
reg  N_SHIFT_LEFTS__LT_inst_clken;
reg [3:0] N_SHIFT_LEFTS__LT_inst_address_a;
wire [7:0] N_SHIFT_LEFTS__LT_inst_q_a;
reg [3:0] N_SHIFT_LEFTS__LT_inst_address_b;
wire [7:0] N_SHIFT_LEFTS__LT_inst_q_b;
reg  N_FREE_ADDRESS___LT_inst_clk;
reg  N_FREE_ADDRESS___LT_inst_clken;
reg [3:0] N_FREE_ADDRESS___LT_inst_address_a;
reg  N_FREE_ADDRESS___LT_inst_wren_a;
reg [31:0] N_FREE_ADDRESS___LT_inst_data_a;
wire  N_FREE_ADDRESS___LT_inst_byteena_a;
wire [31:0] N_FREE_ADDRESS___LT_inst_q_a;
reg [3:0] N_FREE_ADDRESS___LT_inst_address_b;
reg  N_FREE_ADDRESS___LT_inst_wren_b;
reg [31:0] N_FREE_ADDRESS___LT_inst_data_b;
wire  N_FREE_ADDRESS___LT_inst_byteena_b;
wire [31:0] N_FREE_ADDRESS___LT_inst_q_b;
reg  nmalloc_inst_clk;
reg  nmalloc_inst_clk2x;
reg  nmalloc_inst_clk1x_follower;
reg  nmalloc_inst_reset;
reg  nmalloc_inst_memory_controller_waitrequest;
reg  nmalloc_inst_start;
wire  nmalloc_inst_finish;
wire [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_inst_return_val;
reg [31:0] nmalloc_inst_arg_bytes;
wire  nmalloc_inst_N_FREE_ADDRESS___LT_write_enable_a;
wire [31:0] nmalloc_inst_N_FREE_ADDRESS___LT_in_a;
wire  nmalloc_inst_N_FREE_ADDRESS___LT_byteena_a;
wire  nmalloc_inst_N_FREE_ADDRESS___LT_enable_a;
wire [3:0] nmalloc_inst_N_FREE_ADDRESS___LT_address_a;
reg [31:0] nmalloc_inst_N_FREE_ADDRESS___LT_out_a;
wire  nmalloc_inst_N_FREE_ADDRESS___LT_write_enable_b;
wire [31:0] nmalloc_inst_N_FREE_ADDRESS___LT_in_b;
wire  nmalloc_inst_N_FREE_ADDRESS___LT_byteena_b;
wire  nmalloc_inst_N_FREE_ADDRESS___LT_enable_b;
wire [3:0] nmalloc_inst_N_FREE_ADDRESS___LT_address_b;
reg [31:0] nmalloc_inst_N_FREE_ADDRESS___LT_out_b;
reg  nmalloc_inst_finish_reg;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] nmalloc_inst_return_val_reg;
reg  t_overlapping_req_inst_clk;
reg  t_overlapping_req_inst_clk2x;
reg  t_overlapping_req_inst_clk1x_follower;
reg  t_overlapping_req_inst_reset;
reg  t_overlapping_req_inst_memory_controller_waitrequest;
reg  t_overlapping_req_inst_start;
wire  t_overlapping_req_inst_finish;
wire  t_overlapping_req_inst_nmalloc_start;
reg  t_overlapping_req_inst_nmalloc_finish;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_overlapping_req_inst_nmalloc_return_val;
wire [31:0] t_overlapping_req_inst_nmalloc_arg_bytes;
wire  t_overlapping_req_inst_N_SHIFT_LEFTS__LT_enable_a;
wire [3:0] t_overlapping_req_inst_N_SHIFT_LEFTS__LT_address_a;
reg [7:0] t_overlapping_req_inst_N_SHIFT_LEFTS__LT_out_a;
wire  t_overlapping_req_inst_N_SHIFT_LEFTS__LT_enable_b;
wire [3:0] t_overlapping_req_inst_N_SHIFT_LEFTS__LT_address_b;
reg [7:0] t_overlapping_req_inst_N_SHIFT_LEFTS__LT_out_b;
wire  t_overlapping_req_inst_N_FREE_ADDRESS___LT_write_enable_a;
wire [31:0] t_overlapping_req_inst_N_FREE_ADDRESS___LT_in_a;
wire  t_overlapping_req_inst_N_FREE_ADDRESS___LT_byteena_a;
wire  t_overlapping_req_inst_N_FREE_ADDRESS___LT_enable_a;
wire [3:0] t_overlapping_req_inst_N_FREE_ADDRESS___LT_address_a;
reg [31:0] t_overlapping_req_inst_N_FREE_ADDRESS___LT_out_a;
wire  t_overlapping_req_inst_N_FREE_ADDRESS___LT_write_enable_b;
wire [31:0] t_overlapping_req_inst_N_FREE_ADDRESS___LT_in_b;
wire  t_overlapping_req_inst_N_FREE_ADDRESS___LT_byteena_b;
wire  t_overlapping_req_inst_N_FREE_ADDRESS___LT_enable_b;
wire [3:0] t_overlapping_req_inst_N_FREE_ADDRESS___LT_address_b;
reg [31:0] t_overlapping_req_inst_N_FREE_ADDRESS___LT_out_b;
reg  t_overlapping_req_inst_finish_reg;
reg  t_large_req_inst_clk;
reg  t_large_req_inst_clk2x;
reg  t_large_req_inst_clk1x_follower;
reg  t_large_req_inst_reset;
reg  t_large_req_inst_memory_controller_waitrequest;
reg  t_large_req_inst_start;
wire  t_large_req_inst_finish;
wire  t_large_req_inst_nmalloc_start;
reg  t_large_req_inst_nmalloc_finish;
reg [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] t_large_req_inst_nmalloc_return_val;
wire [31:0] t_large_req_inst_nmalloc_arg_bytes;
wire  t_large_req_inst_N_SHIFT_LEFTS__LT_enable_a;
wire [3:0] t_large_req_inst_N_SHIFT_LEFTS__LT_address_a;
reg [7:0] t_large_req_inst_N_SHIFT_LEFTS__LT_out_a;
wire  t_large_req_inst_N_SHIFT_LEFTS__LT_enable_b;
wire [3:0] t_large_req_inst_N_SHIFT_LEFTS__LT_address_b;
reg [7:0] t_large_req_inst_N_SHIFT_LEFTS__LT_out_b;
wire  t_large_req_inst_N_FREE_ADDRESS___LT_write_enable_a;
wire [31:0] t_large_req_inst_N_FREE_ADDRESS___LT_in_a;
wire  t_large_req_inst_N_FREE_ADDRESS___LT_byteena_a;
wire  t_large_req_inst_N_FREE_ADDRESS___LT_enable_a;
wire [3:0] t_large_req_inst_N_FREE_ADDRESS___LT_address_a;
reg [31:0] t_large_req_inst_N_FREE_ADDRESS___LT_out_a;
wire  t_large_req_inst_N_FREE_ADDRESS___LT_write_enable_b;
wire [31:0] t_large_req_inst_N_FREE_ADDRESS___LT_in_b;
wire  t_large_req_inst_N_FREE_ADDRESS___LT_byteena_b;
wire  t_large_req_inst_N_FREE_ADDRESS___LT_enable_b;
wire [3:0] t_large_req_inst_N_FREE_ADDRESS___LT_address_b;
reg [31:0] t_large_req_inst_N_FREE_ADDRESS___LT_out_b;
reg  t_large_req_inst_finish_reg;


main main_inst (
	.clk (main_inst_clk),
	.clk2x (main_inst_clk2x),
	.clk1x_follower (main_inst_clk1x_follower),
	.reset (main_inst_reset),
	.memory_controller_waitrequest (main_inst_memory_controller_waitrequest),
	.start (main_inst_start),
	.finish (main_inst_finish),
	.return_val (main_inst_return_val),
	.t_init_cycle_req_start (main_inst_t_init_cycle_req_start),
	.t_init_cycle_req_finish (main_inst_t_init_cycle_req_finish),
	.t_overlapping_req_start (main_inst_t_overlapping_req_start),
	.t_overlapping_req_finish (main_inst_t_overlapping_req_finish),
	.t_large_req_start (main_inst_t_large_req_start),
	.t_large_req_finish (main_inst_t_large_req_finish)
);



t_init_cycle_req t_init_cycle_req_inst (
	.clk (t_init_cycle_req_inst_clk),
	.clk2x (t_init_cycle_req_inst_clk2x),
	.clk1x_follower (t_init_cycle_req_inst_clk1x_follower),
	.reset (t_init_cycle_req_inst_reset),
	.memory_controller_waitrequest (t_init_cycle_req_inst_memory_controller_waitrequest),
	.start (t_init_cycle_req_inst_start),
	.finish (t_init_cycle_req_inst_finish),
	.nmalloc_start (t_init_cycle_req_inst_nmalloc_start),
	.nmalloc_finish (t_init_cycle_req_inst_nmalloc_finish),
	.nmalloc_return_val (t_init_cycle_req_inst_nmalloc_return_val),
	.nmalloc_arg_bytes (t_init_cycle_req_inst_nmalloc_arg_bytes),
	.N_SHIFT_LEFTS__LT_enable_a (t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_enable_a),
	.N_SHIFT_LEFTS__LT_address_a (t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_address_a),
	.N_SHIFT_LEFTS__LT_out_a (t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_out_a),
	.N_SHIFT_LEFTS__LT_enable_b (t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_enable_b),
	.N_SHIFT_LEFTS__LT_address_b (t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_address_b),
	.N_SHIFT_LEFTS__LT_out_b (t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_out_b),
	.N_FREE_ADDRESS___LT_write_enable_a (t_init_cycle_req_inst_N_FREE_ADDRESS___LT_write_enable_a),
	.N_FREE_ADDRESS___LT_in_a (t_init_cycle_req_inst_N_FREE_ADDRESS___LT_in_a),
	.N_FREE_ADDRESS___LT_byteena_a (t_init_cycle_req_inst_N_FREE_ADDRESS___LT_byteena_a),
	.N_FREE_ADDRESS___LT_enable_a (t_init_cycle_req_inst_N_FREE_ADDRESS___LT_enable_a),
	.N_FREE_ADDRESS___LT_address_a (t_init_cycle_req_inst_N_FREE_ADDRESS___LT_address_a),
	.N_FREE_ADDRESS___LT_out_a (t_init_cycle_req_inst_N_FREE_ADDRESS___LT_out_a),
	.N_FREE_ADDRESS___LT_write_enable_b (t_init_cycle_req_inst_N_FREE_ADDRESS___LT_write_enable_b),
	.N_FREE_ADDRESS___LT_in_b (t_init_cycle_req_inst_N_FREE_ADDRESS___LT_in_b),
	.N_FREE_ADDRESS___LT_byteena_b (t_init_cycle_req_inst_N_FREE_ADDRESS___LT_byteena_b),
	.N_FREE_ADDRESS___LT_enable_b (t_init_cycle_req_inst_N_FREE_ADDRESS___LT_enable_b),
	.N_FREE_ADDRESS___LT_address_b (t_init_cycle_req_inst_N_FREE_ADDRESS___LT_address_b),
	.N_FREE_ADDRESS___LT_out_b (t_init_cycle_req_inst_N_FREE_ADDRESS___LT_out_b)
);



rom_dual_port N_SHIFT_LEFTS__LT_inst (
	.clk (N_SHIFT_LEFTS__LT_inst_clk),
	.clken (N_SHIFT_LEFTS__LT_inst_clken),
	.address_a (N_SHIFT_LEFTS__LT_inst_address_a),
	.q_a (N_SHIFT_LEFTS__LT_inst_q_a),
	.address_b (N_SHIFT_LEFTS__LT_inst_address_b),
	.q_b (N_SHIFT_LEFTS__LT_inst_q_b)
);

defparam
	N_SHIFT_LEFTS__LT_inst.width_a = 8,
	N_SHIFT_LEFTS__LT_inst.width_b = 8,
	N_SHIFT_LEFTS__LT_inst.widthad_a = 4,
	N_SHIFT_LEFTS__LT_inst.widthad_b = 4,
	N_SHIFT_LEFTS__LT_inst.numwords_a = 10,
	N_SHIFT_LEFTS__LT_inst.numwords_b = 10,
	N_SHIFT_LEFTS__LT_inst.latency = 1,
	N_SHIFT_LEFTS__LT_inst.init_file = "N_SHIFT_LEFTS__LT.mif";


ram_dual_port N_FREE_ADDRESS___LT_inst (
	.clk (N_FREE_ADDRESS___LT_inst_clk),
	.clken (N_FREE_ADDRESS___LT_inst_clken),
	.address_a (N_FREE_ADDRESS___LT_inst_address_a),
	.wren_a (N_FREE_ADDRESS___LT_inst_wren_a),
	.data_a (N_FREE_ADDRESS___LT_inst_data_a),
	.byteena_a (N_FREE_ADDRESS___LT_inst_byteena_a),
	.q_a (N_FREE_ADDRESS___LT_inst_q_a),
	.address_b (N_FREE_ADDRESS___LT_inst_address_b),
	.wren_b (N_FREE_ADDRESS___LT_inst_wren_b),
	.data_b (N_FREE_ADDRESS___LT_inst_data_b),
	.byteena_b (N_FREE_ADDRESS___LT_inst_byteena_b),
	.q_b (N_FREE_ADDRESS___LT_inst_q_b)
);

defparam
	N_FREE_ADDRESS___LT_inst.width_a = 32,
	N_FREE_ADDRESS___LT_inst.width_b = 32,
	N_FREE_ADDRESS___LT_inst.widthad_a = 4,
	N_FREE_ADDRESS___LT_inst.widthad_b = 4,
	N_FREE_ADDRESS___LT_inst.width_be_a = 1,
	N_FREE_ADDRESS___LT_inst.width_be_b = 1,
	N_FREE_ADDRESS___LT_inst.numwords_a = 10,
	N_FREE_ADDRESS___LT_inst.numwords_b = 10,
	N_FREE_ADDRESS___LT_inst.latency = 1,
	N_FREE_ADDRESS___LT_inst.init_file = "N_FREE_ADDRESS___LT.mif";


nmalloc nmalloc_inst (
	.clk (nmalloc_inst_clk),
	.clk2x (nmalloc_inst_clk2x),
	.clk1x_follower (nmalloc_inst_clk1x_follower),
	.reset (nmalloc_inst_reset),
	.memory_controller_waitrequest (nmalloc_inst_memory_controller_waitrequest),
	.start (nmalloc_inst_start),
	.finish (nmalloc_inst_finish),
	.return_val (nmalloc_inst_return_val),
	.arg_bytes (nmalloc_inst_arg_bytes),
	.N_FREE_ADDRESS___LT_write_enable_a (nmalloc_inst_N_FREE_ADDRESS___LT_write_enable_a),
	.N_FREE_ADDRESS___LT_in_a (nmalloc_inst_N_FREE_ADDRESS___LT_in_a),
	.N_FREE_ADDRESS___LT_byteena_a (nmalloc_inst_N_FREE_ADDRESS___LT_byteena_a),
	.N_FREE_ADDRESS___LT_enable_a (nmalloc_inst_N_FREE_ADDRESS___LT_enable_a),
	.N_FREE_ADDRESS___LT_address_a (nmalloc_inst_N_FREE_ADDRESS___LT_address_a),
	.N_FREE_ADDRESS___LT_out_a (nmalloc_inst_N_FREE_ADDRESS___LT_out_a),
	.N_FREE_ADDRESS___LT_write_enable_b (nmalloc_inst_N_FREE_ADDRESS___LT_write_enable_b),
	.N_FREE_ADDRESS___LT_in_b (nmalloc_inst_N_FREE_ADDRESS___LT_in_b),
	.N_FREE_ADDRESS___LT_byteena_b (nmalloc_inst_N_FREE_ADDRESS___LT_byteena_b),
	.N_FREE_ADDRESS___LT_enable_b (nmalloc_inst_N_FREE_ADDRESS___LT_enable_b),
	.N_FREE_ADDRESS___LT_address_b (nmalloc_inst_N_FREE_ADDRESS___LT_address_b),
	.N_FREE_ADDRESS___LT_out_b (nmalloc_inst_N_FREE_ADDRESS___LT_out_b)
);



t_overlapping_req t_overlapping_req_inst (
	.clk (t_overlapping_req_inst_clk),
	.clk2x (t_overlapping_req_inst_clk2x),
	.clk1x_follower (t_overlapping_req_inst_clk1x_follower),
	.reset (t_overlapping_req_inst_reset),
	.memory_controller_waitrequest (t_overlapping_req_inst_memory_controller_waitrequest),
	.start (t_overlapping_req_inst_start),
	.finish (t_overlapping_req_inst_finish),
	.nmalloc_start (t_overlapping_req_inst_nmalloc_start),
	.nmalloc_finish (t_overlapping_req_inst_nmalloc_finish),
	.nmalloc_return_val (t_overlapping_req_inst_nmalloc_return_val),
	.nmalloc_arg_bytes (t_overlapping_req_inst_nmalloc_arg_bytes),
	.N_SHIFT_LEFTS__LT_enable_a (t_overlapping_req_inst_N_SHIFT_LEFTS__LT_enable_a),
	.N_SHIFT_LEFTS__LT_address_a (t_overlapping_req_inst_N_SHIFT_LEFTS__LT_address_a),
	.N_SHIFT_LEFTS__LT_out_a (t_overlapping_req_inst_N_SHIFT_LEFTS__LT_out_a),
	.N_SHIFT_LEFTS__LT_enable_b (t_overlapping_req_inst_N_SHIFT_LEFTS__LT_enable_b),
	.N_SHIFT_LEFTS__LT_address_b (t_overlapping_req_inst_N_SHIFT_LEFTS__LT_address_b),
	.N_SHIFT_LEFTS__LT_out_b (t_overlapping_req_inst_N_SHIFT_LEFTS__LT_out_b),
	.N_FREE_ADDRESS___LT_write_enable_a (t_overlapping_req_inst_N_FREE_ADDRESS___LT_write_enable_a),
	.N_FREE_ADDRESS___LT_in_a (t_overlapping_req_inst_N_FREE_ADDRESS___LT_in_a),
	.N_FREE_ADDRESS___LT_byteena_a (t_overlapping_req_inst_N_FREE_ADDRESS___LT_byteena_a),
	.N_FREE_ADDRESS___LT_enable_a (t_overlapping_req_inst_N_FREE_ADDRESS___LT_enable_a),
	.N_FREE_ADDRESS___LT_address_a (t_overlapping_req_inst_N_FREE_ADDRESS___LT_address_a),
	.N_FREE_ADDRESS___LT_out_a (t_overlapping_req_inst_N_FREE_ADDRESS___LT_out_a),
	.N_FREE_ADDRESS___LT_write_enable_b (t_overlapping_req_inst_N_FREE_ADDRESS___LT_write_enable_b),
	.N_FREE_ADDRESS___LT_in_b (t_overlapping_req_inst_N_FREE_ADDRESS___LT_in_b),
	.N_FREE_ADDRESS___LT_byteena_b (t_overlapping_req_inst_N_FREE_ADDRESS___LT_byteena_b),
	.N_FREE_ADDRESS___LT_enable_b (t_overlapping_req_inst_N_FREE_ADDRESS___LT_enable_b),
	.N_FREE_ADDRESS___LT_address_b (t_overlapping_req_inst_N_FREE_ADDRESS___LT_address_b),
	.N_FREE_ADDRESS___LT_out_b (t_overlapping_req_inst_N_FREE_ADDRESS___LT_out_b)
);



t_large_req t_large_req_inst (
	.clk (t_large_req_inst_clk),
	.clk2x (t_large_req_inst_clk2x),
	.clk1x_follower (t_large_req_inst_clk1x_follower),
	.reset (t_large_req_inst_reset),
	.memory_controller_waitrequest (t_large_req_inst_memory_controller_waitrequest),
	.start (t_large_req_inst_start),
	.finish (t_large_req_inst_finish),
	.nmalloc_start (t_large_req_inst_nmalloc_start),
	.nmalloc_finish (t_large_req_inst_nmalloc_finish),
	.nmalloc_return_val (t_large_req_inst_nmalloc_return_val),
	.nmalloc_arg_bytes (t_large_req_inst_nmalloc_arg_bytes),
	.N_SHIFT_LEFTS__LT_enable_a (t_large_req_inst_N_SHIFT_LEFTS__LT_enable_a),
	.N_SHIFT_LEFTS__LT_address_a (t_large_req_inst_N_SHIFT_LEFTS__LT_address_a),
	.N_SHIFT_LEFTS__LT_out_a (t_large_req_inst_N_SHIFT_LEFTS__LT_out_a),
	.N_SHIFT_LEFTS__LT_enable_b (t_large_req_inst_N_SHIFT_LEFTS__LT_enable_b),
	.N_SHIFT_LEFTS__LT_address_b (t_large_req_inst_N_SHIFT_LEFTS__LT_address_b),
	.N_SHIFT_LEFTS__LT_out_b (t_large_req_inst_N_SHIFT_LEFTS__LT_out_b),
	.N_FREE_ADDRESS___LT_write_enable_a (t_large_req_inst_N_FREE_ADDRESS___LT_write_enable_a),
	.N_FREE_ADDRESS___LT_in_a (t_large_req_inst_N_FREE_ADDRESS___LT_in_a),
	.N_FREE_ADDRESS___LT_byteena_a (t_large_req_inst_N_FREE_ADDRESS___LT_byteena_a),
	.N_FREE_ADDRESS___LT_enable_a (t_large_req_inst_N_FREE_ADDRESS___LT_enable_a),
	.N_FREE_ADDRESS___LT_address_a (t_large_req_inst_N_FREE_ADDRESS___LT_address_a),
	.N_FREE_ADDRESS___LT_out_a (t_large_req_inst_N_FREE_ADDRESS___LT_out_a),
	.N_FREE_ADDRESS___LT_write_enable_b (t_large_req_inst_N_FREE_ADDRESS___LT_write_enable_b),
	.N_FREE_ADDRESS___LT_in_b (t_large_req_inst_N_FREE_ADDRESS___LT_in_b),
	.N_FREE_ADDRESS___LT_byteena_b (t_large_req_inst_N_FREE_ADDRESS___LT_byteena_b),
	.N_FREE_ADDRESS___LT_enable_b (t_large_req_inst_N_FREE_ADDRESS___LT_enable_b),
	.N_FREE_ADDRESS___LT_address_b (t_large_req_inst_N_FREE_ADDRESS___LT_address_b),
	.N_FREE_ADDRESS___LT_out_b (t_large_req_inst_N_FREE_ADDRESS___LT_out_b)
);


// Local Rams

// End Local Rams

always @(*) begin
	main_inst_clk = clk;
end
always @(*) begin
	main_inst_clk2x = clk2x;
end
always @(*) begin
	main_inst_clk1x_follower = clk1x_follower;
end
always @(*) begin
	main_inst_reset = reset;
end
always @(*) begin
	main_inst_memory_controller_waitrequest = memory_controller_waitrequest;
end
always @(*) begin
	main_inst_start = start;
end
always @(*) begin
	main_inst_t_init_cycle_req_finish = (~(t_init_cycle_req_inst_start) & t_init_cycle_req_inst_finish_reg);
end
always @(*) begin
	main_inst_t_overlapping_req_finish = (~(t_overlapping_req_inst_start) & t_overlapping_req_inst_finish_reg);
end
always @(*) begin
	main_inst_t_large_req_finish = (~(t_large_req_inst_start) & t_large_req_inst_finish_reg);
end
always @(posedge clk) begin
	if ((reset | main_inst_start)) begin
		main_inst_finish_reg <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to main_inst_finish_reg"); $finish; end
		/* synthesis translate_on */
	end
	if (main_inst_finish) begin
		main_inst_finish_reg <= 1'd1;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd1) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to main_inst_finish_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(posedge clk) begin
	if ((reset | main_inst_start)) begin
		main_inst_return_val_reg <= 0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to main_inst_return_val_reg"); $finish; end
		/* synthesis translate_on */
	end
	if (main_inst_finish) begin
		main_inst_return_val_reg <= main_inst_return_val;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(main_inst_return_val) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to main_inst_return_val_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	t_init_cycle_req_inst_clk = clk;
end
always @(*) begin
	t_init_cycle_req_inst_clk2x = clk2x;
end
always @(*) begin
	t_init_cycle_req_inst_clk1x_follower = clk1x_follower;
end
always @(*) begin
	t_init_cycle_req_inst_reset = reset;
end
always @(*) begin
	t_init_cycle_req_inst_memory_controller_waitrequest = memory_controller_waitrequest;
end
always @(*) begin
	t_init_cycle_req_inst_start = main_inst_t_init_cycle_req_start;
end
always @(*) begin
	t_init_cycle_req_inst_nmalloc_finish = (~(nmalloc_inst_start) & nmalloc_inst_finish_reg);
end
always @(*) begin
	t_init_cycle_req_inst_nmalloc_return_val = nmalloc_inst_return_val_reg;
end
always @(*) begin
	t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_out_a = N_SHIFT_LEFTS__LT_inst_q_a;
end
always @(*) begin
	t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_out_b = N_SHIFT_LEFTS__LT_inst_q_b;
end
always @(*) begin
	t_init_cycle_req_inst_N_FREE_ADDRESS___LT_out_a = N_FREE_ADDRESS___LT_inst_q_a;
end
always @(*) begin
	t_init_cycle_req_inst_N_FREE_ADDRESS___LT_out_b = N_FREE_ADDRESS___LT_inst_q_b;
end
always @(posedge clk) begin
	if ((reset | t_init_cycle_req_inst_start)) begin
		t_init_cycle_req_inst_finish_reg <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_init_cycle_req_inst_finish_reg"); $finish; end
		/* synthesis translate_on */
	end
	if (t_init_cycle_req_inst_finish) begin
		t_init_cycle_req_inst_finish_reg <= 1'd1;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd1) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_init_cycle_req_inst_finish_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	N_SHIFT_LEFTS__LT_inst_clk = clk;
end
always @(*) begin
	N_SHIFT_LEFTS__LT_inst_clken = ~(memory_controller_waitrequest);
end
always @(*) begin
	N_SHIFT_LEFTS__LT_inst_address_a = ((t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_address_a | t_overlapping_req_inst_N_SHIFT_LEFTS__LT_address_a) | t_large_req_inst_N_SHIFT_LEFTS__LT_address_a);
end
always @(*) begin
	N_SHIFT_LEFTS__LT_inst_address_b = ((t_init_cycle_req_inst_N_SHIFT_LEFTS__LT_address_b | t_overlapping_req_inst_N_SHIFT_LEFTS__LT_address_b) | t_large_req_inst_N_SHIFT_LEFTS__LT_address_b);
end
always @(*) begin
	N_FREE_ADDRESS___LT_inst_clk = clk;
end
always @(*) begin
	N_FREE_ADDRESS___LT_inst_clken = ~(memory_controller_waitrequest);
end
always @(*) begin
	N_FREE_ADDRESS___LT_inst_address_a = (((t_init_cycle_req_inst_N_FREE_ADDRESS___LT_address_a | nmalloc_inst_N_FREE_ADDRESS___LT_address_a) | t_overlapping_req_inst_N_FREE_ADDRESS___LT_address_a) | t_large_req_inst_N_FREE_ADDRESS___LT_address_a);
end
always @(*) begin
	N_FREE_ADDRESS___LT_inst_wren_a = (((t_init_cycle_req_inst_N_FREE_ADDRESS___LT_write_enable_a | nmalloc_inst_N_FREE_ADDRESS___LT_write_enable_a) | t_overlapping_req_inst_N_FREE_ADDRESS___LT_write_enable_a) | t_large_req_inst_N_FREE_ADDRESS___LT_write_enable_a);
end
always @(*) begin
	N_FREE_ADDRESS___LT_inst_data_a = (((t_init_cycle_req_inst_N_FREE_ADDRESS___LT_in_a | nmalloc_inst_N_FREE_ADDRESS___LT_in_a) | t_overlapping_req_inst_N_FREE_ADDRESS___LT_in_a) | t_large_req_inst_N_FREE_ADDRESS___LT_in_a);
end
always @(*) begin
	N_FREE_ADDRESS___LT_inst_address_b = (((t_init_cycle_req_inst_N_FREE_ADDRESS___LT_address_b | nmalloc_inst_N_FREE_ADDRESS___LT_address_b) | t_overlapping_req_inst_N_FREE_ADDRESS___LT_address_b) | t_large_req_inst_N_FREE_ADDRESS___LT_address_b);
end
always @(*) begin
	N_FREE_ADDRESS___LT_inst_wren_b = (((t_init_cycle_req_inst_N_FREE_ADDRESS___LT_write_enable_b | nmalloc_inst_N_FREE_ADDRESS___LT_write_enable_b) | t_overlapping_req_inst_N_FREE_ADDRESS___LT_write_enable_b) | t_large_req_inst_N_FREE_ADDRESS___LT_write_enable_b);
end
always @(*) begin
	N_FREE_ADDRESS___LT_inst_data_b = (((t_init_cycle_req_inst_N_FREE_ADDRESS___LT_in_b | nmalloc_inst_N_FREE_ADDRESS___LT_in_b) | t_overlapping_req_inst_N_FREE_ADDRESS___LT_in_b) | t_large_req_inst_N_FREE_ADDRESS___LT_in_b);
end
always @(*) begin
	nmalloc_inst_clk = clk;
end
always @(*) begin
	nmalloc_inst_clk2x = clk2x;
end
always @(*) begin
	nmalloc_inst_clk1x_follower = clk1x_follower;
end
always @(*) begin
	nmalloc_inst_reset = reset;
end
always @(*) begin
	nmalloc_inst_memory_controller_waitrequest = memory_controller_waitrequest;
end
always @(*) begin
	nmalloc_inst_start = ((t_init_cycle_req_inst_nmalloc_start | t_overlapping_req_inst_nmalloc_start) | t_large_req_inst_nmalloc_start);
end
always @(*) begin
	nmalloc_inst_arg_bytes = ((t_init_cycle_req_inst_nmalloc_arg_bytes | t_overlapping_req_inst_nmalloc_arg_bytes) | t_large_req_inst_nmalloc_arg_bytes);
end
always @(*) begin
	nmalloc_inst_N_FREE_ADDRESS___LT_out_a = N_FREE_ADDRESS___LT_inst_q_a;
end
always @(*) begin
	nmalloc_inst_N_FREE_ADDRESS___LT_out_b = N_FREE_ADDRESS___LT_inst_q_b;
end
always @(posedge clk) begin
	if ((reset | nmalloc_inst_start)) begin
		nmalloc_inst_finish_reg <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_inst_finish_reg"); $finish; end
		/* synthesis translate_on */
	end
	if (nmalloc_inst_finish) begin
		nmalloc_inst_finish_reg <= 1'd1;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd1) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_inst_finish_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(posedge clk) begin
	if ((reset | nmalloc_inst_start)) begin
		nmalloc_inst_return_val_reg <= 0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_inst_return_val_reg"); $finish; end
		/* synthesis translate_on */
	end
	if (nmalloc_inst_finish) begin
		nmalloc_inst_return_val_reg <= nmalloc_inst_return_val;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(nmalloc_inst_return_val) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to nmalloc_inst_return_val_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	t_overlapping_req_inst_clk = clk;
end
always @(*) begin
	t_overlapping_req_inst_clk2x = clk2x;
end
always @(*) begin
	t_overlapping_req_inst_clk1x_follower = clk1x_follower;
end
always @(*) begin
	t_overlapping_req_inst_reset = reset;
end
always @(*) begin
	t_overlapping_req_inst_memory_controller_waitrequest = memory_controller_waitrequest;
end
always @(*) begin
	t_overlapping_req_inst_start = main_inst_t_overlapping_req_start;
end
always @(*) begin
	t_overlapping_req_inst_nmalloc_finish = (~(nmalloc_inst_start) & nmalloc_inst_finish_reg);
end
always @(*) begin
	t_overlapping_req_inst_nmalloc_return_val = nmalloc_inst_return_val_reg;
end
always @(*) begin
	t_overlapping_req_inst_N_SHIFT_LEFTS__LT_out_a = N_SHIFT_LEFTS__LT_inst_q_a;
end
always @(*) begin
	t_overlapping_req_inst_N_SHIFT_LEFTS__LT_out_b = N_SHIFT_LEFTS__LT_inst_q_b;
end
always @(*) begin
	t_overlapping_req_inst_N_FREE_ADDRESS___LT_out_a = N_FREE_ADDRESS___LT_inst_q_a;
end
always @(*) begin
	t_overlapping_req_inst_N_FREE_ADDRESS___LT_out_b = N_FREE_ADDRESS___LT_inst_q_b;
end
always @(posedge clk) begin
	if ((reset | t_overlapping_req_inst_start)) begin
		t_overlapping_req_inst_finish_reg <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_inst_finish_reg"); $finish; end
		/* synthesis translate_on */
	end
	if (t_overlapping_req_inst_finish) begin
		t_overlapping_req_inst_finish_reg <= 1'd1;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd1) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_overlapping_req_inst_finish_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(*) begin
	t_large_req_inst_clk = clk;
end
always @(*) begin
	t_large_req_inst_clk2x = clk2x;
end
always @(*) begin
	t_large_req_inst_clk1x_follower = clk1x_follower;
end
always @(*) begin
	t_large_req_inst_reset = reset;
end
always @(*) begin
	t_large_req_inst_memory_controller_waitrequest = memory_controller_waitrequest;
end
always @(*) begin
	t_large_req_inst_start = main_inst_t_large_req_start;
end
always @(*) begin
	t_large_req_inst_nmalloc_finish = (~(nmalloc_inst_start) & nmalloc_inst_finish_reg);
end
always @(*) begin
	t_large_req_inst_nmalloc_return_val = nmalloc_inst_return_val_reg;
end
always @(*) begin
	t_large_req_inst_N_SHIFT_LEFTS__LT_out_a = N_SHIFT_LEFTS__LT_inst_q_a;
end
always @(*) begin
	t_large_req_inst_N_SHIFT_LEFTS__LT_out_b = N_SHIFT_LEFTS__LT_inst_q_b;
end
always @(*) begin
	t_large_req_inst_N_FREE_ADDRESS___LT_out_a = N_FREE_ADDRESS___LT_inst_q_a;
end
always @(*) begin
	t_large_req_inst_N_FREE_ADDRESS___LT_out_b = N_FREE_ADDRESS___LT_inst_q_b;
end
always @(posedge clk) begin
	if ((reset | t_large_req_inst_start)) begin
		t_large_req_inst_finish_reg <= 1'd0;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd0) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_inst_finish_reg"); $finish; end
		/* synthesis translate_on */
	end
	if (t_large_req_inst_finish) begin
		t_large_req_inst_finish_reg <= 1'd1;
		/* synthesis translate_off */
		if (start == 1'b0 && ^(1'd1) === 1'bX) begin $display ("ERROR: Right hand side is 'X'. Assigned to t_large_req_inst_finish_reg"); $finish; end
		/* synthesis translate_on */
	end
end
always @(posedge clk) begin
	finish <= main_inst_finish;
end
always @(posedge clk) begin
	return_val <= main_inst_return_val;
end

endmodule
