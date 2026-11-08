//
// scp_track.sv - floppy drive playing back SCP flux images
//
// Copyright (c) 2026 Manfredi
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// The image is mounted like any other: the module reads the file itself
// over the sd block interface, Main does nothing special.
//
//  - on mount it reads the first 1 KiB: the SCP header and the track table
//    (a disk without the "SCP" signature is left to the other paths)
//  - once the head has rested on a track for SETTLE_CLKS (a Greaseweazle
//    behind the image needs a few hundred ms per track, so tracks crossed
//    during a seek are not asked for) it reads that track's header and one
//    revolution of flux into memory
//  - the flux is replayed in real time from the index, and a data separator
//    like the WD1772's (a digital PLL, modelled on MAME's fdc_pll_t) turns
//    it into the cell stream the FDC already reads from MFMT tracks
//
// While a track is loading no flux reaches the head; a track missing from
// the image reads as noise.

module scp_track #(
	parameter ACC         = 20,       // time per clock in 1/16 of 25 ns (32 MHz: 31.25 ns)
	parameter INDEX_CLKS  = 128000,   // ~4 ms at 32 MHz
	parameter SETTLE_CLKS = 320000    // ~10 ms at 32 MHz
)(
	input             clk,
	input             reset,

	input             enable,       // SCP image in the selected drive, drive selected, sd free
	input             sd_free,      // nobody else uses the sd interface
	input       [6:0] cyl,
	input             head,
	input             drive,
	input       [1:0] mount,        // image (re)mounted, one clock per drive
	input       [1:0] mount_ok,     // ... with a file that may be SCP
	input             pll_reset,    // a command starts (the PLL restarts at nominal speed)

	output reg  [1:0] is_scp,       // drive holds an SCP image
	output reg  [1:0] probing,      // its header is being read

	// sd block interface
	output reg [31:0] sd_lba,
	output reg        sd_rd,
	output reg  [5:0] sd_blk_cnt,
	output reg        sd_drive,     // drive the request is for
	input             sd_ack,
	input      [12:0] sd_buff_addr,
	input      [15:0] sd_dout,
	input             sd_dout_strobe,
	output            busy,

	// cell stream (as mfmt_track)
	output reg        cell_stb,
	output reg        cell_bit,
	output reg        index,
	output            loaded,
	output reg [16:0] position
);

// ---------------------------------------------------------------------------
// memories
// ---------------------------------------------------------------------------
// Two banks of one revolution each: one plays while the next revolution of
// the same track loads into the other, and they swap at the index. Weak bits
// then read differently from one turn to the next, as on the disk (a capture
// holds several revolutions; a Greaseweazle behind the image can serve a
// fresh one every time).

reg [15:0] flux_ram[131072];     // {bank, entry}: 25 ns units, 0 = +65536
reg [15:0] tab_ram[1024];        // track table: {drive, track, half}
reg [15:0] hdr_ram[512];         // 1 KiB of the file around a track header

reg [16:0] flux_wa, flux_ra;
reg [15:0] flux_wd, flux_q;
reg        flux_we;
reg  [9:0] tab_ra;
reg [15:0] tab_q;
reg  [8:0] hdr_ra;
reg [15:0] hdr_q;

wire wr = sd_dout_strobe & sd_ack & busy;
reg  [2:0] part;                 // what the transfer is for
localparam P_PROBE = 3'd0, P_HDR = 3'd1, P_FLUX = 3'd2;
reg        pdrive;               // drive being probed

always @(posedge clk) begin
	if (wr && part == P_PROBE && sd_buff_addr >= 8 && sd_buff_addr < 8 + 336)
		tab_ram[{pdrive, 9'(sd_buff_addr - 13'd8)}] <= sd_dout;
	if (wr && part == P_HDR) hdr_ram[sd_buff_addr[8:0]] <= sd_dout;
	if (flux_we) flux_ram[flux_wa] <= flux_wd;

	tab_q  <= tab_ram[tab_ra];
	hdr_q  <= hdr_ram[hdr_ra];
	flux_q <= flux_ram[flux_ra];
end

// ---------------------------------------------------------------------------
// loader
// ---------------------------------------------------------------------------

localparam MAXREV = 8;

reg  [7:0] h_heads[2], h_revs[2], h_res[2], h_first[2], h_last[2];
reg        h_seq[2];             // single sided image numbered by cylinder
reg  [1:0] probe_req;
reg        l_stale;              // the drive being loaded got a new image

reg [15:0] w0, w1, w2, w3, w5;   // header words of the probe

localparam L_IDLE = 4'd0, L_PROBE = 4'd1, L_PROBE_W = 4'd2, L_PROBE_END = 4'd3,
           L_TAB = 4'd4, L_TAB_W = 4'd5, L_TAB_G = 4'd6, L_HDR = 4'd7, L_HDR_W = 4'd8,
           L_HDR_R = 4'd9, L_FLUX = 4'd10, L_FLUX_W = 4'd11, L_DONE = 4'd12;
reg  [3:0] lstate;

// the track wanted, settled
reg        k_drive, k_head;
reg  [6:0] k_cyl;
reg [18:0] settle_cnt;
wire       same_key = (k_drive == drive) && (k_head == head) && (k_cyl == cyl);
wire       settled  = same_key && settle_cnt == SETTLE_CLKS;

// the track in memory
reg        valid, formatted;
reg        v_drive, v_head;
reg  [6:0] v_cyl;
reg [27:0] b_units[2];           // index to index of each bank, 1/16 of 25 ns
reg [16:0] b_nflux[2];           // entries in each bank
reg        pb;                   // bank playing (replay)
reg        sw_force;             // play the other bank now (new track)
reg        other_ready;          // the other bank holds the next revolution
reg        sw_ack, sw_ackD;      // replay took it (toggle)
reg  [2:0] cur_rev, other_rev, l_rev;
reg  [3:0] nrev;                 // revolutions used (at most MAXREV)
reg        bg;                   // the load is for the next revolution
wire       key_match = valid && v_drive == drive && v_head == head && v_cyl == cyl;
assign     loaded = key_match && enable;

reg        l_drive, l_head;
reg  [6:0] l_cyl;
reg [31:0] off;                  // track header, bytes in the file
reg [31:0] rv_index[MAXREV], rv_len[MAXREV], rv_data[MAXREV];
reg  [6:0] hb;                   // header byte being read
reg [16:0] want, nflux;          // flux entries wanted / received
reg [31:0] left;                 // file position of the current request (block aligned)
reg  [9:0] skip;                 // bytes before the data in the first block
reg  [7:0] hi;
reg        have_hi;

assign busy = lstate == L_PROBE_W || lstate == L_TAB_W || lstate == L_HDR_W || lstate == L_FLUX_W;

// track number in the table
wire [7:0] tnum = (h_heads[l_drive] == 0) ? {l_cyl, l_head} :
                  h_seq[l_drive] ? {1'b0, l_cyl} : {l_cyl, l_head};
wire       side_ok = h_heads[l_drive] == 0 || (h_heads[l_drive] == 1 && !l_head) || (h_heads[l_drive] == 2 && l_head);

// header byte hb of the revolution table: (off & 511) + 4 + hb
wire [9:0] hpos  = 10'(off[8:0]) + 10'd4 + 10'(hb);
wire [9:0] hpos1 = hpos + 10'd1;

wire [2:0] next_rev = (4'(cur_rev) + 1'd1 >= nrev) ? 3'd0 : cur_rev + 1'd1;

always @(posedge clk) begin
	flux_we <= 0;
	sw_force <= 0;
	sw_ackD <= sw_ack;
	if (sw_ack != sw_ackD) begin
		other_ready <= 0;
		cur_rev <= other_rev;
	end

	// settle: the head has to stay put before its track is asked for
	if (!same_key) begin
		k_drive <= drive; k_head <= head; k_cyl <= cyl;
		settle_cnt <= 0;
	end
	else if (settle_cnt != SETTLE_CLKS) settle_cnt <= settle_cnt + 1'd1;

	for (int i = 0; i < 2; i++) begin
		if (mount[i]) begin
			is_scp[i] <= 0;
			probe_req[i] <= mount_ok[i];
			probing[i] <= mount_ok[i];
			if (v_drive == i[0]) valid <= 0;
			if (l_drive == i[0] && lstate != L_IDLE) l_stale <= 1;
		end
	end

	if (reset) begin
		lstate <= L_IDLE;
		sd_rd <= 0;
		valid <= 0;
		other_ready <= 0;
	end
	else case (lstate)
	L_IDLE:
		if (sd_free && |probe_req) begin
			pdrive <= !probe_req[0];
			lstate <= L_PROBE;
		end
		else if (enable && !key_match && settled) begin
			l_drive <= drive; l_head <= head; l_cyl <= cyl;
			l_stale <= 0;
			valid <= 0;
			other_ready <= 0;
			bg <= 0;
			lstate <= L_TAB;
		end
		else if (enable && key_match && formatted && nrev > 1 && !other_ready && sw_ack == sw_ackD && sd_free) begin
			// the next revolution into the other bank
			l_rev <= next_rev;
			l_stale <= 0;
			bg <= 1;
			lstate <= L_FLUX;
		end

	// ---- header and track table, first 2 blocks
	L_PROBE: begin
		part <= P_PROBE;
		sd_lba <= 0;
		sd_blk_cnt <= 6'd1;
		sd_drive <= pdrive;
		sd_rd <= 1;
		lstate <= L_PROBE_W;
	end
	L_PROBE_W: begin
		if (sd_ack) sd_rd <= 0;
		if (wr) case (sd_buff_addr)
			0: w0 <= sd_dout;
			1: w1 <= sd_dout;
			2: w2 <= sd_dout;
			3: w3 <= sd_dout;
			5: w5 <= sd_dout;
			default: ;
		endcase
		if (!sd_rd && !sd_ack) lstate <= L_PROBE_END;
	end
	L_PROBE_END: begin
		// "SC", "P", at least one revolution
		is_scp[pdrive] <= w0 == 16'h4353 && w1[7:0] == 8'h50 && w2[15:8] != 0;
		h_revs[pdrive] <= w2[15:8];
		h_heads[pdrive] <= w5[7:0];
		h_res[pdrive] <= w5[15:8];
		h_first[pdrive] <= w3[7:0];
		h_last[pdrive] <= w3[15:8];
		h_seq[pdrive] <= 0;
		probe_req[pdrive] <= 0;
		probing[pdrive] <= 0;
		tab_ra <= {pdrive, 8'd1, 1'b0};
		lstate <= L_TAB_G;
		hb <= 7'd3;
	end
	L_TAB_G: begin
		// a single sided image with track 1 in its table is numbered by cylinder
		hb <= hb - 1'd1;
		if (hb == 1) h_seq[pdrive] <= h_heads[pdrive] == 1 && tab_q != 0;
		if (hb == 0) lstate <= L_IDLE;
	end

	// ---- table entry of the track
	L_TAB: begin
		nrev <= (h_revs[l_drive] > MAXREV) ? 4'(MAXREV) : 4'(h_revs[l_drive]);
		// only the tracks the header says are there (older files have a
		// shorter table, the entries past the last track are data)
		if (!side_ok || tnum < h_first[l_drive] || tnum > h_last[l_drive]) begin
			formatted <= 0;
			lstate <= L_DONE;
		end
		else begin
			tab_ra <= {l_drive, tnum, 1'b0};
			hb <= 0;
			lstate <= L_TAB_W;
		end
	end
	L_TAB_W: begin
		tab_ra <= {l_drive, tnum, 1'b1};
		hb <= hb + 1'd1;
		if (hb == 1) off[15:0] <= tab_q;
		if (hb == 2) begin
			off[31:16] <= tab_q;
			lstate <= L_HDR;
		end
	end

	L_HDR:
		if (off == 0) begin
			formatted <= 0;
			lstate <= L_DONE;
		end
		else begin
			part <= P_HDR;
			sd_lba <= off >> 9;
			sd_blk_cnt <= 6'd1;
			sd_drive <= l_drive;
			sd_rd <= 1;
			lstate <= L_HDR_W;
		end
	L_HDR_W: begin
		if (sd_ack) sd_rd <= 0;
		if (!sd_rd && !sd_ack) begin
			hb <= 0;
			hdr_ra <= 9'((10'(off[8:0]) + 10'd4) >> 1);   // byte 0 (hb is 0 only from the next clock)
			lstate <= L_HDR_R;
		end
	end
	// the revolution table: index time, flux count, data offset (12 bytes each).
	// hdr_ra runs one byte ahead and hdr_q follows two clocks later: at hb = j,
	// hdr_q holds the word of byte j-1
	L_HDR_R: begin
		hdr_ra <= hpos1[9:1];
		if (hb >= 1) begin : getb
			reg [6:0] b;
			reg [9:0] p;
			reg [7:0] v;
			reg [2:0] r;
			reg [3:0] f;
			b = hb - 7'd1;
			p = 10'(off[8:0]) + 10'd4 + 10'(b);
			v = p[0] ? hdr_q[15:8] : hdr_q[7:0];
			r = 3'(b / 12);
			f = 4'(b % 12);
			case (f[3:2])
				0: rv_index[r][f[1:0] * 8 +: 8] <= v;
				1: rv_len[r][f[1:0] * 8 +: 8] <= v;
				default: rv_data[r][f[1:0] * 8 +: 8] <= v;
			endcase
		end
		hb <= hb + 1'd1;
		if (hb == 7'(12 * nrev)) begin
			formatted <= 1;
			l_rev <= 0;
			lstate <= L_FLUX;
		end
	end

	// ---- one revolution into the bank that is not playing
	L_FLUX: begin
		begin : setup
			reg [31:0] st;
			reg [16:0] n;
			st = off + rv_data[l_rev];
			n = (rv_len[l_rev] > 32'd65535) ? 17'd65535 : 17'(rv_len[l_rev]);
			want <= n;
			skip <= 10'(st[8:0]);
			have_hi <= 0;
			nflux <= 0;
			left <= {st[31:9], 9'd0};
			if (n == 0) begin
				if (!bg) formatted <= 0;
				lstate <= L_DONE;
			end
			else begin
				part <= P_FLUX;
				sd_lba <= st >> 9;
				// bytes from the block start: (st & 511) + 2n, at most 32 blocks a request
				sd_blk_cnt <= ((32'(st[8:0]) + 32'(n) * 2 + 511) >> 9) > 32 ? 6'd31 :
				              6'(((32'(st[8:0]) + 32'(n) * 2 + 511) >> 9) - 1);
				sd_drive <= l_drive;
				sd_rd <= 1;
				lstate <= L_FLUX_W;
			end
		end
	end
	L_FLUX_W: begin
		if (sd_ack) sd_rd <= 0;
		if (wr) begin : take
			reg [7:0] b0, b1;
			reg [9:0] sk;
			reg       hh;
			reg [7:0] h;
			reg       we;
			reg [15:0] wd;
			b0 = sd_dout[7:0];
			b1 = sd_dout[15:8];
			sk = skip; hh = have_hi; h = hi; we = 0; wd = 0;
			// samples are big endian; a word holds 2 bytes, at most one sample ends in it
			if (sk != 0) sk = sk - 1'd1;
			else if (hh) begin wd = {h, b0}; we = 1; hh = 0; end
			else begin h = b0; hh = 1; end
			if (sk != 0) sk = sk - 1'd1;
			else if (hh) begin wd = {h, b1}; we = 1; hh = 0; end
			else begin h = b1; hh = 1; end
			skip <= sk; have_hi <= hh; hi <= h;
			if (we && nflux < want) begin
				flux_wa <= {!pb, 16'(nflux)};
				flux_wd <= wd;
				flux_we <= 1;
				nflux <= nflux + 1'd1;
			end
		end
		if (!sd_rd && !sd_ack) begin
			if (nflux >= want) lstate <= L_DONE;
			else begin : next
				// next chunk, right after the blocks received
				reg [31:0] pos, rem;
				pos = left + 32'(sd_blk_cnt + 1'd1) * 512;
				left <= pos;
				rem = 32'(want - nflux) * 2;
				sd_lba <= pos >> 9;
				sd_blk_cnt <= ((rem + 511) >> 9) > 32 ? 6'd31 : 6'(((rem + 511) >> 9) - 1);
				sd_rd <= 1;
			end
		end
	end

	L_DONE: begin
		lstate <= L_IDLE;
		if (l_stale || (enable && (l_drive != drive || l_head != head || l_cyl != cyl))) begin
			// the head moved (or a new image came) while loading: load again
			valid <= 0;
			other_ready <= 0;
		end
		else begin
			b_units[!pb] <= 28'(rv_index[l_rev][23:0]) << 4;
			b_nflux[!pb] <= nflux;
			if (bg) begin
				other_ready <= 1;
				other_rev <= l_rev;
			end
			else begin
				valid <= 1;
				v_drive <= l_drive; v_head <= l_head; v_cyl <= l_cyl;
				cur_rev <= 0;
				if (formatted) sw_force <= 1;
			end
		end
	end
	default: lstate <= L_IDLE;
	endcase
end

// ---------------------------------------------------------------------------
// flux replay: edges in real time from the index
// ---------------------------------------------------------------------------

localparam [27:0] REV_NOMINAL = 28'd8000000 << 4;   // 200 ms

reg [27:0] t;                    // since the index, 1/16 of 25 ns
reg [27:0] t_edge;               // next edge
reg [16:0] k;                    // its entry
reg        edge_ok;              // t_edge belongs to entry k (or noise)
reg        flux_edge;            // one clock per transition
reg        idx_pulse;            // one clock at the index
reg [31:0] lfsr = 32'h1;
reg [17:0] index_cnt;
reg  [1:0] fstate;               // fetch: 0 idle, 1 address set, 2 data
wire [27:0] rev_len = (valid && formatted) ? b_units[pb] : REV_NOMINAL;
wire        play    = valid && formatted && key_match;
// a track is wanted and not there yet (see the index below). Kept while
// TOS selects drive B for a moment, so the held index is not let go then.
reg         want_track;
always @(posedge clk) begin
	if (reset || |mount || key_match) want_track <= 0;
	else if (enable) want_track <= 1;
end
wire        hold_index = want_track && !key_match;
wire        noise   = valid && !formatted && key_match;
reg  [7:0]  res_mul;
reg [27:0] t_last;               // last transition
reg signed [28:0] carry;         // the first entry of the turn counts from the last transition
reg        seeking;              // started in the middle of a turn: skip what is behind the head

always @(posedge clk) begin
	lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
	flux_edge <= 0;
	idx_pulse <= 0;
	if (index_cnt != 0) index_cnt <= index_cnt - 1'd1;
	else index <= 0;
	res_mul <= h_res[v_drive] + 1'd1;

	if (reset) begin
		t <= 0;
		k <= 0;
		edge_ok <= 0;
		fstate <= 0;
		index <= 0;
		pb <= 0;
	end
	else begin
		if (sw_force) begin
			// a new track: its first revolution is in the other bank. Play it
			// from where the disk is now: the fetch below skips the entries
			// already behind the head
			pb <= !pb;
			k <= 0;
			t_edge <= 0;
			edge_ok <= 0;
			fstate <= 0;
			carry <= 0;
			seeking <= 1;
			t <= t + ACC;
		end
		else if (t + ACC >= rev_len && hold_index) begin
			// the track under the head is still being read from the file: no
			// index pulse until it is there, as from a drive whose head is still
			// settling, so the controller does not count turns without data.
			// With a file on the SD card this is a few ms at most; a source that
			// takes longer to answer (Main reading a real floppy) is waited for
			// the same way.
		end
		else if (t + ACC >= rev_len) begin
			// index: the revolution starts again, with the next one if it is there
			t <= 28'(t + ACC - rev_len);
			index <= 1;
			idx_pulse <= 1;
			index_cnt <= INDEX_CLKS[17:0];
			k <= 0;
			edge_ok <= 0;
			t_edge <= 0;
			fstate <= 0;
			// A capture is one stream cut at the index: the first entry of
			// revolution r > 0 is the time from the last transition of r-1,
			// revolution 0 starts at the index. Following them in order keeps
			// a sector that runs across the index whole.
			begin : wrap
				reg [2:0] nr;
				reg [27:0] tl;
				nr = (other_ready && sw_ack == sw_ackD) ? other_rev : cur_rev;
				// an edge of the old turn still pending (its entries add up to a
				// little more than the index time, rounding) goes out now
				tl = t_last;
				if (play && edge_ok) begin
					flux_edge <= 1;
					tl = t_edge;
				end
				carry <= (nr != 0) ? 29'($signed({1'b0, tl}) - $signed({1'b0, rev_len})) : 29'sd0;
			end
			if (other_ready && sw_ack == sw_ackD) begin
				pb <= !pb;
				sw_ack <= !sw_ack;
			end
		end
		else begin
			t <= t + ACC;

			if (play) begin
				// fetch entry k: t_edge += 16 * value (* resolution); entries
				// already behind the head (a track started in the middle of a
				// turn) are skipped without a transition
				case (fstate)
				0: if (!edge_ok && k < b_nflux[pb]) begin flux_ra <= {pb, 16'(k)}; fstate <= 1; end
				1: fstate <= 2;
				2: begin : fetch
					reg [27:0] te;
					te = t_edge + ((flux_q == 0) ? 28'(65536 * 16) : (28'(flux_q) << 4)) * res_mul;
					// the first entry of a turn that continues the previous one
					begin : cont
						reg signed [29:0] x;
						x = $signed({2'b00, te}) + carry;
						te = (x < 0) ? 28'd0 : 28'(x);
					end
					carry <= 0;
					t_edge <= te;
					// an overflow entry has no transition; while seeking, entries
					// behind the head are skipped (otherwise a late edge fires at once)
					if (flux_q != 0 && (!seeking || te > t + 2 * ACC)) begin
						edge_ok <= 1;
						seeking <= 0;
					end
					else k <= k + 1'd1;
					fstate <= 0;
				end
				default: fstate <= 0;
				endcase
				if (edge_ok && t + ACC >= t_edge) begin
					flux_edge <= 1;
					edge_ok <= 0;
					k <= k + 1'd1;
					t_last <= t_edge;
				end
			end
			else if (noise) begin
				// unformatted: a transition every 2..10 us
				if (!edge_ok) begin
					t_edge <= t + 28'd1280 + 28'({lfsr[11:0], 1'b0});
					edge_ok <= 1;
				end
				else if (t + ACC >= t_edge) begin
					flux_edge <= 1;
					edge_ok <= 0;
				end
			end
			else edge_ok <= 0;
		end
	end
end

// ---------------------------------------------------------------------------
// data separator: digital PLL as in the WD1772 (MAME fdc_pll_t)
// ---------------------------------------------------------------------------
// The window ends after one cell period plus a phase correction. A flux edge
// in the window makes a 1 cell; its distance from the window centre moves
// the next window by ~0.65 of it, and after two corrections the same way
// the period by ~5 % of it (within +-25 % of 2 us).

localparam integer CELL     = 80 * 16;     // 2 us in 1/16 of 25 ns
localparam integer CELL_MIN = CELL * 3 / 4;
localparam integer CELL_MAX = CELL * 5 / 4;

reg signed [15:0] rem;           // time left in the window
reg signed [15:0] edge_rem;      // rem when the edge came
reg        got_edge;
reg [15:0] period;
reg signed [15:0] phase;
reg signed [3:0]  hist;          // corrections the same way in a row

always @(posedge clk) begin
	cell_stb <= 0;
	if (reset) begin
		rem <= 16'(CELL);
		period <= 16'(CELL);
		phase <= 0;
		hist <= 0;
		got_edge <= 0;
		position <= 0;
	end
	else begin
		// the first cell after the index is cell 0
		if (idx_pulse) position <= 17'h1FFFF;


		begin : step
			reg signed [15:0] r;
			reg               e;
			reg signed [15:0] er;
			r = rem - 16'(ACC);
			e = got_edge;
			er = edge_rem;
			if (flux_edge && !got_edge) begin
				e = 1;
				er = rem;       // the edge is in this clock: rem before the step
			end
			if (r <= 0) begin
				// window ends: output the cell
				reg signed [15:0] delta, ph;
				reg        [15:0] per;
				reg signed [3:0]  h;
				cell_stb <= 1;
				cell_bit <= e;
				position <= idx_pulse ? 17'd0 : position + 1'd1;
				per = period;
				h = hist;
				if (e) begin
					// delta = edge - window centre; er counts down to the end
					delta = 16'($signed({1'b0, period[15:1]}) - er);
					ph = 16'((32'($signed(delta)) * 21) >>> 5);
					if (delta < 0) h = (hist < 0) ? ((hist == -8) ? hist : hist - 1'd1) : -4'sd1;
					else if (delta > 0) h = (hist > 0) ? ((hist == 7) ? hist : hist + 1'd1) : 4'sd1;
					else h = 0;
					if (h > 1 || h < -1) begin
						per = 16'($signed({1'b0, period}) + 16'((32'($signed(delta)) * 13) >>> 8));
						if (per < CELL_MIN) per = CELL_MIN;
						if (per > CELL_MAX) per = CELL_MAX;
					end
				end
				else ph = 0;
				hist <= h;
				period <= per;
				phase <= ph;
				rem <= r + $signed({1'b0, per}) + ph;
				got_edge <= 0;
			end
			else begin
				rem <= r;
				got_edge <= e;
				edge_rem <= er;
			end
		end

		// Back to the nominal cell at every index, with every command, on a
		// new track and while there is no flux (MAME restarts its PLL with every command). Out of
		// its lock range (after noise: unformatted tracks, the unused side of a
		// single sided disk) the PLL would otherwise stay on a wrong period.
		if (idx_pulse || sw_force || pll_reset || !(play || noise)) begin
			period <= 16'(CELL);
			hist <= 0;
		end
	end
end

endmodule
