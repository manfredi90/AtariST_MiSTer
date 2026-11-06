//
// mfmt_track.sv - floppy drive playing back MFMT bit-cell tracks
//
// Copyright (c) 2026 Manfredi
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// The track format and the transport are described in Main_MiSTer
// support/st/MFMT.md. In short: when the head lands on a track the module
// reads its META part (4 KiB: header, timing table, weak table, weak mask
// pool) and its CELLS part (16 KiB, 16 cells per word, MSB first) over the
// sd block interface, then plays the cells back in a loop:
//
//  - every clock a phase accumulator gains ACC_STEP units (1/16 ns; 500 at
//    32 MHz); when it reaches the length of the current cell the next cell
//    is output with cell_stb
//  - the cell length comes from the timing table entry covering the word
//  - cells marked in the weak mask read as pseudo random bits
//  - index is high for INDEX_CLKS clocks from cell 0
//
// The cell position keeps running across track changes, as the disk keeps
// turning under a stepping head. While a track is loading no flux is output.
// An unformatted track (or no track at all) reads as noise.

module mfmt_track #(
	parameter ACC_STEP   = 500,     // 1/16 ns per clock (32 MHz)
	parameter INDEX_CLKS = 128000   // ~4 ms at 32 MHz
)(
	input             clk,
	input             reset,

	input             enable,       // MFMT image in the selected drive, drive selected
	input       [6:0] cyl,          // head position
	input             head,
	input             drive,        // selected drive (a change reloads)
	input             invalidate,   // new image mounted

	// sd block interface (shared with the sector path in fdc1772)
	output reg [31:0] sd_lba,
	output reg        sd_rd,
	output reg  [5:0] sd_blk_cnt,
	input             sd_ack,
	input      [12:0] sd_buff_addr, // 16-bit word address
	input      [15:0] sd_dout,
	input             sd_dout_strobe,
	output            busy,         // owns the sd interface

	// cell stream
	output reg        cell_stb,     // one clock per cell
	output reg        cell_bit,         // 1 = flux transition
	output reg        index,
	output            loaded,       // track data valid
	output     [16:0] position      // current cell
);

// ---------------------------------------------------------------------------
// memories (written by the sd interface, read by the sequencer)
// ---------------------------------------------------------------------------

// META word map: 0x000 header, 0x020 timing (256 x 2), 0x220 weak (48 x 2), 0x280 pool
localparam [15:0] ST_CELL_DD = 16'd32000;   // nominal DD cell, 2 us

localparam W_TIM  = 13'h020;
localparam W_WEAK = 13'h220;
localparam W_POOL = 13'h280;

reg [15:0] cells_ram[8192];
reg [15:0] tim_ram[512];
reg [15:0] weak_ram[128];
reg [15:0] pool_ram[2048];

reg [12:0] cells_ra;
reg  [8:0] tim_ra;
reg  [6:0] weak_ra;
reg [10:0] pool_ra;
reg [15:0] cells_q, tim_q, weak_q, pool_q;

reg        part;    // 0 = META, 1 = CELLS being received
wire       wr = sd_dout_strobe & sd_ack & busy;

always @(posedge clk) begin
	if (wr && part)                                         cells_ram[sd_buff_addr] <= sd_dout;
	if (wr && !part && sd_buff_addr >= W_TIM && sd_buff_addr < W_WEAK)  tim_ram[9'(sd_buff_addr - W_TIM)] <= sd_dout;
	if (wr && !part && sd_buff_addr >= W_WEAK && sd_buff_addr < W_POOL) weak_ram[7'(sd_buff_addr - W_WEAK)] <= sd_dout;
	if (wr && !part && sd_buff_addr >= W_POOL)              pool_ram[11'(sd_buff_addr - W_POOL)] <= sd_dout;

	cells_q <= cells_ram[cells_ra];
	tim_q   <= tim_ram[tim_ra];
	weak_q  <= weak_ram[weak_ra];
	pool_q  <= pool_ram[pool_ra];
end

// ---------------------------------------------------------------------------
// loader
// ---------------------------------------------------------------------------

reg [15:0] h_magic0, h_magic1, h_version, h_flags;
reg [31:0] h_cells;
reg  [8:0] h_tim_count;
reg  [5:0] h_weak_count;

localparam L_IDLE = 0, L_META = 1, L_CELLS = 2, L_DONE = 3;
reg  [1:0] lstate;
reg        valid;              // track in memory matches key
reg        key_valid;
reg  [6:0] key_cyl;
reg        key_head, key_drive;
reg        sd_ackD;
reg        load_done;          // one clock: start playing the new track

assign busy = (lstate == L_META) || (lstate == L_CELLS);

wire key_match = key_valid && key_cyl == cyl && key_head == head && key_drive == drive;

wire hdr_ok = h_magic0 == 16'h464D && h_magic1 == 16'h544D && h_version == 16'd1 &&
              h_cells != 0 && h_cells <= 32'd131072 && h_tim_count != 0 && h_tim_count <= 9'd256 &&
              h_weak_count <= 6'd48;

always @(posedge clk) begin
	sd_ackD <= sd_ack;
	load_done <= 0;
	if (sd_ack) sd_rd <= 0;

	if (reset || invalidate) begin
		lstate <= L_IDLE;
		valid <= 0;
		key_valid <= 0;
		sd_rd <= 0;
	end
	else case (lstate)
	L_IDLE:
		if (enable && !key_match) begin
			key_cyl <= cyl;
			key_head <= head;
			key_drive <= drive;
			key_valid <= 1;
			valid <= 0;
			part <= 0;
			sd_lba <= {1'b1, 13'd0, 2'd0, 7'd0, head, 1'b0, cyl};
			sd_blk_cnt <= 6'd7;
			sd_rd <= 1;
			lstate <= L_META;
		end

	L_META: begin
		if (wr) case (sd_buff_addr)
			0: h_magic0 <= sd_dout;
			1: h_magic1 <= sd_dout;
			2: h_version <= sd_dout;
			3: h_flags <= sd_dout;
			4: h_cells[15:0] <= sd_dout;
			5: h_cells[31:16] <= sd_dout;
			6: h_tim_count <= |sd_dout[15:9] ? 9'h1FF : sd_dout[8:0];
			7: h_weak_count <= |sd_dout[15:6] ? 6'h3F : sd_dout[5:0];
			default: ;
		endcase
		if (sd_ackD && !sd_ack) begin
			part <= 1;
			sd_lba <= {1'b1, 13'd0, 2'd1, 7'd0, key_head, 1'b0, key_cyl};
			sd_blk_cnt <= 6'd31;
			sd_rd <= 1;
			lstate <= L_CELLS;
		end
	end

	L_CELLS:
		if (sd_ackD && !sd_ack) lstate <= L_DONE;

	L_DONE: begin
		// the head moved while loading: load again
		if (!key_match) lstate <= L_IDLE;
		else begin
			valid <= hdr_ok;
			load_done <= hdr_ok;
			lstate <= L_IDLE;
		end
	end
	endcase
end

assign loaded = valid && enable;
wire formatted = h_flags[0];

// ---------------------------------------------------------------------------
// prefetch sequencer: prepares word nw_w (bits, weak mask, cell length)
// ---------------------------------------------------------------------------

localparam S_IDLE = 0, S_TREAD = 1, S_TWAIT = 2, S_TGET = 3, S_TCHK = 4,
           S_WREAD = 5, S_WWAIT = 6, S_WGET = 7, S_WCHK = 8, S_FETCH = 9, S_FWAIT = 10, S_FGET = 11;
reg  [3:0] sstate;

// request
reg        pf_go, pf_seek, pf_imm;
reg [12:0] pf_w;
reg        pf_seek_r, pf_imm_r;

// timing state: entry t_i starts at t_ns with length t_nl; t_len applies before it
reg  [8:0] t_i;
reg [15:0] t_ns, t_nl, t_len;
// weak state: segment w_i covers [w_s, w_e), its masks start at w_pb in the pool
reg  [5:0] w_i;
reg [15:0] w_s, w_e;
reg [10:0] w_pb;
reg        w_valid;

// result
reg [15:0] nw_word, nw_mask, nw_len;
reg [12:0] nw_w;
reg        nw_ready;
reg        nw_imm;          // apply at once (start of playback)
reg        sbusy;
reg        half;            // second word of a table entry

always @(posedge clk) begin
	if (reset || !valid) begin
		sstate <= S_IDLE;
		nw_ready <= 0;
		sbusy <= 0;
		nw_imm <= 0;
	end
	else begin
		nw_imm <= 0;
		case (sstate)
		S_IDLE:
			if (pf_go) begin
				sbusy <= 1;
				nw_ready <= 0;
				nw_w <= pf_w;
				if (pf_seek) begin
					t_i <= 0;
					t_len <= ST_CELL_DD;
					w_i <= 0;
					w_pb <= 0;
					sstate <= S_TREAD;
				end
				else sstate <= S_TCHK;
			end

		// ---- timing: read entry t_i into t_ns / t_nl
		S_TREAD: begin
			if (t_i >= h_tim_count) begin
				t_ns <= 16'hFFFF;
				sstate <= S_TCHK;
			end
			else begin
				tim_ra <= {t_i[7:0], 1'b0};
				half <= 0;
				sstate <= S_TWAIT;
			end
		end
		S_TWAIT: begin
			tim_ra <= {t_i[7:0], 1'b1};
			sstate <= S_TGET;
		end
		S_TGET: begin
			if (!half) begin
				t_ns <= tim_q;
				half <= 1;
			end
			else begin
				t_nl <= tim_q;
				sstate <= S_TCHK;
			end
		end
		S_TCHK:
			if ({3'b000, nw_w} >= t_ns && t_ns != 16'hFFFF) begin
				t_len <= t_nl;
				t_i <= t_i + 1'd1;
				sstate <= S_TREAD;
			end
			else sstate <= (pf_seek_r ? S_WREAD : S_WCHK);

		// ---- weak: read segment w_i
		S_WREAD: begin
			if (w_i >= h_weak_count) begin
				w_valid <= 0;
				sstate <= S_WCHK;
			end
			else begin
				weak_ra <= {w_i, 1'b0};
				half <= 0;
				sstate <= S_WWAIT;
			end
		end
		S_WWAIT: begin
			weak_ra <= {w_i, 1'b1};
			sstate <= S_WGET;
		end
		S_WGET: begin
			if (!half) begin
				w_s <= weak_q;
				half <= 1;
			end
			else begin
				w_e <= w_s + weak_q;
				w_valid <= 1;
				sstate <= S_WCHK;
			end
		end
		S_WCHK:
			if (w_valid && {3'b000, nw_w} >= w_e) begin
				w_pb <= w_pb + 11'(w_e - w_s);
				w_i <= w_i + 1'd1;
				sstate <= S_WREAD;
			end
			else sstate <= S_FETCH;

		// ---- fetch the cells and the weak mask of the word
		S_FETCH: begin
			cells_ra <= nw_w;
			pool_ra <= w_pb + 11'(nw_w - w_s[12:0]);
			sstate <= S_FWAIT;
		end
		S_FWAIT: sstate <= S_FGET;
		S_FGET: begin
			nw_word <= cells_q;
			nw_mask <= (w_valid && {3'b000, nw_w} >= w_s) ? pool_q : 16'h0000;
			nw_len <= t_len;
			nw_ready <= 1;
			nw_imm <= pf_imm_r;
			sbusy <= 0;
			sstate <= S_IDLE;
		end
		default: sstate <= S_IDLE;
		endcase
	end
end

always @(posedge clk) if (pf_go && sstate == S_IDLE) begin
	pf_seek_r <= pf_seek;
	pf_imm_r <= pf_imm;
end

// ---------------------------------------------------------------------------
// playback
// ---------------------------------------------------------------------------

reg [16:0] acc;
reg [16:0] c;            // current cell
reg [15:0] cur_word, cur_mask, cur_len;
reg        cur_ok;       // cur_word holds track data
reg [31:0] lfsr = 32'h1;
reg [17:0] index_cnt;

wire [16:0] ncells = (valid && formatted) ? h_cells[16:0] : 17'd100000;
wire [12:0] nwords = 13'((ncells + 17'd15) >> 4);
wire [16:0] c_next = (c + 1'd1 >= ncells) ? 17'd0 : c + 1'd1;
wire [15:0] len    = (valid && formatted && cur_ok) ? cur_len : ST_CELL_DD;
wire [17:0] acc_n  = {1'b0, acc} + ACC_STEP[17:0];

assign position = c;

function [12:0] word_after(input [12:0] w);
	word_after = (w + 1'd1 >= nwords) ? 13'd0 : w + 1'd1;
endfunction

// bits of the word under the head after the next cell step (the word
// changes on the same clock when the next cell starts a word)
wire        next_new  = (c_next[3:0] == 0) && nw_ready && (nw_w == c_next[16:4]);
wire [15:0] next_word = next_new ? nw_word : cur_word;
wire [15:0] next_mask = next_new ? nw_mask : cur_mask;
wire  [3:0] next_bit  = 4'd15 - c_next[3:0];

always @(posedge clk) begin
	lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
	cell_stb <= 0;
	pf_go <= 0;

	if (index_cnt != 0) index_cnt <= index_cnt - 1'd1;
	else index <= 0;

	if (reset) begin
		acc <= 0;
		c <= 0;
		cur_ok <= 0;
		index <= 0;
	end
	else begin
		if (!valid) cur_ok <= 0;

		// a new track is in memory: prepare the word under the head
		if (load_done) begin
			cur_ok <= 0;
			if (c >= ncells) c <= 0;
			pf_w <= (c >= ncells) ? 13'd0 : c[16:4];
			pf_seek <= 1;
			pf_imm <= 1;
			pf_go <= 1;
		end

		if (nw_imm && nw_ready) begin
			cur_word <= nw_word;
			cur_mask <= nw_mask;
			cur_len <= nw_len;
			cur_ok <= 1;
			pf_w <= word_after(nw_w);
			pf_seek <= (word_after(nw_w) == 0);
			pf_imm <= 0;
			pf_go <= 1;
		end

		if (acc_n >= {2'b00, len}) begin
			acc <= 17'(acc_n - {2'b00, len});
			c <= c_next;
			cell_stb <= 1;

			if (c_next == 0) begin
				index <= 1;
				index_cnt <= INDEX_CLKS[17:0];
			end

			// entering a new word: take the prefetched one, prefetch the next
			if (c_next[3:0] == 0 && valid && formatted) begin
				if (nw_ready && nw_w == c_next[16:4] && cur_ok) begin
					cur_word <= nw_word;
					cur_mask <= nw_mask;
					cur_len <= nw_len;
					pf_w <= word_after(c_next[16:4]);
					pf_seek <= (word_after(c_next[16:4]) == 0);
					pf_imm <= 0;
					pf_go <= 1;
				end
				else if (cur_ok) begin
					// prefetch late (cannot happen with >= 64 clocks per word): resync
					cur_ok <= 0;
					pf_w <= c_next[16:4];
					pf_seek <= 1;
					pf_imm <= 1;
					pf_go <= 1;
				end
			end

			// value of the new cell
			if (!valid || !enable) cell_bit <= 0;                         // loading: no flux
			else if (!formatted) cell_bit <= lfsr[0];                     // unformatted: noise
			else if (!cur_ok) cell_bit <= 0;
			else cell_bit <= next_mask[next_bit] ? lfsr[0] : next_word[next_bit];
		end
		else acc <= acc_n[16:0];
	end
end

endmodule
