module nmalloc_ut (
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

// Inputs and Registers
input [0:0] clk;
input [0:0] clk2x;
reg[0:0] clk2x_reg;
input [0:0] clk1x_follower;
reg[0:0] clk1x_follower_reg;
input [0:0] reset;
reg[0:0] reset_reg;
input [0:0] memory_controller_waitrequest;
reg[0:0] memory_controller_waitrequest_reg;
input [0:0] start;
reg[0:0] start_reg;
input [31:0] arg_bytes;
reg[31:0] arg_bytes_reg;
input [31:0] N_FREE_ADDRESS___LT_out_a;
reg[31:0] N_FREE_ADDRESS___LT_out_a_reg;
input [31:0] N_FREE_ADDRESS___LT_out_b;
reg[31:0] N_FREE_ADDRESS___LT_out_b_reg;

// Outputs, Registers & Wires
output [0:0] finish;
reg[0:0] finish_reg;
wire[0:0] finish_wire;
output [`MEMORY_CONTROLLER_ADDR_SIZE-1:0] return_val;
reg[`MEMORY_CONTROLLER_ADDR_SIZE-1:0] return_val_reg;
wire[`MEMORY_CONTROLLER_ADDR_SIZE-1:0] return_val_wire;
output [0:0] N_FREE_ADDRESS___LT_write_enable_a;
reg[0:0] N_FREE_ADDRESS___LT_write_enable_a_reg;
wire[0:0] N_FREE_ADDRESS___LT_write_enable_a_wire;
output [31:0] N_FREE_ADDRESS___LT_in_a;
reg[31:0] N_FREE_ADDRESS___LT_in_a_reg;
wire[31:0] N_FREE_ADDRESS___LT_in_a_wire;
output [0:0] N_FREE_ADDRESS___LT_byteena_a;
reg[0:0] N_FREE_ADDRESS___LT_byteena_a_reg;
wire[0:0] N_FREE_ADDRESS___LT_byteena_a_wire;
output [0:0] N_FREE_ADDRESS___LT_enable_a;
reg[0:0] N_FREE_ADDRESS___LT_enable_a_reg;
wire[0:0] N_FREE_ADDRESS___LT_enable_a_wire;
output [3:0] N_FREE_ADDRESS___LT_address_a;
reg[3:0] N_FREE_ADDRESS___LT_address_a_reg;
wire[3:0] N_FREE_ADDRESS___LT_address_a_wire;
output [0:0] N_FREE_ADDRESS___LT_write_enable_b;
reg[0:0] N_FREE_ADDRESS___LT_write_enable_b_reg;
wire[0:0] N_FREE_ADDRESS___LT_write_enable_b_wire;
output [31:0] N_FREE_ADDRESS___LT_in_b;
reg[31:0] N_FREE_ADDRESS___LT_in_b_reg;
wire[31:0] N_FREE_ADDRESS___LT_in_b_wire;
output [0:0] N_FREE_ADDRESS___LT_byteena_b;
reg[0:0] N_FREE_ADDRESS___LT_byteena_b_reg;
wire[0:0] N_FREE_ADDRESS___LT_byteena_b_wire;
output [0:0] N_FREE_ADDRESS___LT_enable_b;
reg[0:0] N_FREE_ADDRESS___LT_enable_b_reg;
wire[0:0] N_FREE_ADDRESS___LT_enable_b_wire;
output [3:0] N_FREE_ADDRESS___LT_address_b;
reg[3:0] N_FREE_ADDRESS___LT_address_b_reg;
wire[3:0] N_FREE_ADDRESS___LT_address_b_wire;

// Assigning outputs.
assign finish = finish_reg;
assign return_val = return_val_reg;
assign N_FREE_ADDRESS___LT_write_enable_a = N_FREE_ADDRESS___LT_write_enable_a_reg;
assign N_FREE_ADDRESS___LT_in_a = N_FREE_ADDRESS___LT_in_a_reg;
assign N_FREE_ADDRESS___LT_byteena_a = N_FREE_ADDRESS___LT_byteena_a_reg;
assign N_FREE_ADDRESS___LT_enable_a = N_FREE_ADDRESS___LT_enable_a_reg;
assign N_FREE_ADDRESS___LT_address_a = N_FREE_ADDRESS___LT_address_a_reg;
assign N_FREE_ADDRESS___LT_write_enable_b = N_FREE_ADDRESS___LT_write_enable_b_reg;
assign N_FREE_ADDRESS___LT_in_b = N_FREE_ADDRESS___LT_in_b_reg;
assign N_FREE_ADDRESS___LT_byteena_b = N_FREE_ADDRESS___LT_byteena_b_reg;
assign N_FREE_ADDRESS___LT_enable_b = N_FREE_ADDRESS___LT_enable_b_reg;
assign N_FREE_ADDRESS___LT_address_b = N_FREE_ADDRESS___LT_address_b_reg;

always @(posedge clk)
	begin
		clk2x_reg <= clk2x;
		clk1x_follower_reg <= clk1x_follower;
		reset_reg <= reset;
		memory_controller_waitrequest_reg <= memory_controller_waitrequest;
		start_reg <= start;
		arg_bytes_reg <= arg_bytes;
		N_FREE_ADDRESS___LT_out_a_reg <= N_FREE_ADDRESS___LT_out_a;
		N_FREE_ADDRESS___LT_out_b_reg <= N_FREE_ADDRESS___LT_out_b;
		finish_reg <= finish_wire;
		return_val_reg <= return_val_wire;
		N_FREE_ADDRESS___LT_write_enable_a_reg <= N_FREE_ADDRESS___LT_write_enable_a_wire;
		N_FREE_ADDRESS___LT_in_a_reg <= N_FREE_ADDRESS___LT_in_a_wire;
		N_FREE_ADDRESS___LT_byteena_a_reg <= N_FREE_ADDRESS___LT_byteena_a_wire;
		N_FREE_ADDRESS___LT_enable_a_reg <= N_FREE_ADDRESS___LT_enable_a_wire;
		N_FREE_ADDRESS___LT_address_a_reg <= N_FREE_ADDRESS___LT_address_a_wire;
		N_FREE_ADDRESS___LT_write_enable_b_reg <= N_FREE_ADDRESS___LT_write_enable_b_wire;
		N_FREE_ADDRESS___LT_in_b_reg <= N_FREE_ADDRESS___LT_in_b_wire;
		N_FREE_ADDRESS___LT_byteena_b_reg <= N_FREE_ADDRESS___LT_byteena_b_wire;
		N_FREE_ADDRESS___LT_enable_b_reg <= N_FREE_ADDRESS___LT_enable_b_wire;
		N_FREE_ADDRESS___LT_address_b_reg <= N_FREE_ADDRESS___LT_address_b_wire;
	end

nmalloc inst1(
	 .clk(clk),
	 .clk2x(clk2x_reg),
	 .clk1x_follower(clk1x_follower_reg),
	 .reset(reset_reg),
	 .memory_controller_waitrequest(memory_controller_waitrequest_reg),
	 .start(start_reg),
	 .arg_bytes(arg_bytes_reg),
	 .N_FREE_ADDRESS___LT_out_a(N_FREE_ADDRESS___LT_out_a_reg),
	 .N_FREE_ADDRESS___LT_out_b(N_FREE_ADDRESS___LT_out_b_reg),
	 .finish(finish_wire),
	 .return_val(return_val_wire),
	 .N_FREE_ADDRESS___LT_write_enable_a(N_FREE_ADDRESS___LT_write_enable_a_wire),
	 .N_FREE_ADDRESS___LT_in_a(N_FREE_ADDRESS___LT_in_a_wire),
	 .N_FREE_ADDRESS___LT_byteena_a(N_FREE_ADDRESS___LT_byteena_a_wire),
	 .N_FREE_ADDRESS___LT_enable_a(N_FREE_ADDRESS___LT_enable_a_wire),
	 .N_FREE_ADDRESS___LT_address_a(N_FREE_ADDRESS___LT_address_a_wire),
	 .N_FREE_ADDRESS___LT_write_enable_b(N_FREE_ADDRESS___LT_write_enable_b_wire),
	 .N_FREE_ADDRESS___LT_in_b(N_FREE_ADDRESS___LT_in_b_wire),
	 .N_FREE_ADDRESS___LT_byteena_b(N_FREE_ADDRESS___LT_byteena_b_wire),
	 .N_FREE_ADDRESS___LT_enable_b(N_FREE_ADDRESS___LT_enable_b_wire),
	 .N_FREE_ADDRESS___LT_address_b(N_FREE_ADDRESS___LT_address_b_wire)
);

endmodule
