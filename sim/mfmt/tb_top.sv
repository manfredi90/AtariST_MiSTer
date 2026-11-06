// tb_top.sv - Verilator wrapper: mfmt_track + two mfm_dec (read track / address mark)
module tb_top #(parameter ACC_STEP = 4000, parameter INDEX_CLKS = 16000)
(
	input         clk,
	input         reset,
	input         enable,
	input   [6:0] cyl,
	input         head,
	input         drive,
	input         invalidate,

	output [31:0] sd_lba,
	output        sd_rd,
	output  [5:0] sd_blk_cnt,
	input         sd_ack,
	input  [12:0] sd_buff_addr,
	input  [15:0] sd_dout,
	input         sd_dout_strobe,

	output        cell_stb,
	output        cell_bit,
	output        index,
	output        loaded,
	output [16:0] position,

	input         rt_restart,
	output        rt_stb,
	output  [7:0] rt_val,

	input         am_restart,
	input   [1:0] am_mode,
	output        am_stb,
	output  [7:0] am_val,
	output        am_sync
);

wire busy;
mfmt_track #(.ACC_STEP(ACC_STEP), .INDEX_CLKS(INDEX_CLKS)) drive_i
(
	.clk(clk), .reset(reset), .enable(enable), .cyl(cyl), .head(head), .drive(drive), .invalidate(invalidate),
	.sd_lba(sd_lba), .sd_rd(sd_rd), .sd_blk_cnt(sd_blk_cnt), .sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr), .sd_dout(sd_dout), .sd_dout_strobe(sd_dout_strobe), .busy(busy),
	.cell_stb(cell_stb), .cell_bit(cell_bit), .index(index), .loaded(loaded), .position(position)
);

wire rt_sync, rt_c2;
mfm_dec rt_dec(.clk(clk), .restart(rt_restart), .mode(2'd2), .cell_stb(cell_stb), .cell_bit(cell_bit),
	.byte_stb(rt_stb), .byte_val(rt_val), .byte_sync(rt_sync), .byte_c2(rt_c2));

wire am_c2;
mfm_dec am_dec(.clk(clk), .restart(am_restart), .mode(am_mode), .cell_stb(cell_stb), .cell_bit(cell_bit),
	.byte_stb(am_stb), .byte_val(am_val), .byte_sync(am_sync), .byte_c2(am_c2));

endmodule
