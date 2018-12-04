module main_ut (
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
input [0:0] finish;
reg[0:0] finish_reg;
input [0:0] t_init_cycle_req_finish;
reg[0:0] t_init_cycle_req_finish_reg;
input [0:0] t_overlapping_req_finish;
reg[0:0] t_overlapping_req_finish_reg;
input [0:0] t_large_req_finish;
reg[0:0] t_large_req_finish_reg;

// Outputs, Registers & Wires
output [31:0] return_val;
reg[31:0] return_val_reg;
wire[31:0] return_val_wire;
output [0:0] t_init_cycle_req_start;
reg[0:0] t_init_cycle_req_start_reg;
wire[0:0] t_init_cycle_req_start_wire;
output [0:0] t_overlapping_req_start;
reg[0:0] t_overlapping_req_start_reg;
wire[0:0] t_overlapping_req_start_wire;
output [0:0] t_large_req_start;
reg[0:0] t_large_req_start_reg;
wire[0:0] t_large_req_start_wire;

// Assigning outputs.
assign return_val = return_val_reg;
assign t_init_cycle_req_start = t_init_cycle_req_start_reg;
assign t_overlapping_req_start = t_overlapping_req_start_reg;
assign t_large_req_start = t_large_req_start_reg;

always @(posedge clk)
	begin
		clk2x_reg <= clk2x;
		clk1x_follower_reg <= clk1x_follower;
		reset_reg <= reset;
		memory_controller_waitrequest_reg <= memory_controller_waitrequest;
		start_reg <= start;
		finish_reg <= finish;
		t_init_cycle_req_finish_reg <= t_init_cycle_req_finish;
		t_overlapping_req_finish_reg <= t_overlapping_req_finish;
		t_large_req_finish_reg <= t_large_req_finish;
		return_val_reg <= return_val_wire;
		t_init_cycle_req_start_reg <= t_init_cycle_req_start_wire;
		t_overlapping_req_start_reg <= t_overlapping_req_start_wire;
		t_large_req_start_reg <= t_large_req_start_wire;
	end

main inst1(
	 .clk(clk),
	 .clk2x(clk2x_reg),
	 .clk1x_follower(clk1x_follower_reg),
	 .reset(reset_reg),
	 .memory_controller_waitrequest(memory_controller_waitrequest_reg),
	 .start(start_reg),
	 .finish(finish_reg),
	 .t_init_cycle_req_finish(t_init_cycle_req_finish_reg),
	 .t_overlapping_req_finish(t_overlapping_req_finish_reg),
	 .t_large_req_finish(t_large_req_finish_reg),
	 .return_val(return_val_wire),
	 .t_init_cycle_req_start(t_init_cycle_req_start_wire),
	 .t_overlapping_req_start(t_overlapping_req_start_wire),
	 .t_large_req_start(t_large_req_start_wire)
);

endmodule
