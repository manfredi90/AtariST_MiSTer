// tb.cpp - Verilator test of mfmt_track + mfm_dec against the software WD1772 model
//
// The testbench plays Main's part: it converts the STX with the same code
// Main uses (support/st) and answers the core's sd requests with the MFMT
// META / CELLS parts. Then it compares, for one revolution of each track,
// what the RTL produces with the software model (wdsim) used to validate the
// converter:
//   - every cell (except weak ones) and the total revolution time
//   - the ID fields found by an address-mark search (bytes, position, timing)
//   - the read-track byte stream
//
// Usage: tb [-t cyl head] [-s] file.stx...      (-s: also step during playback)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <map>
#include <vector>
#include <string>
#include "Vtb_top.h"
#include "verilated.h"
#include "st_stx.h"
#include "wdsim.h"

#ifndef ACC_STEP
#define ACC_STEP 4000
#endif

static Vtb_top *top;
static uint64_t ticks;

struct Track { std::vector<uint8_t> meta, cells; };
static const StxImage *g_img;
static std::map<int, Track> cache;

static const Track &get_track(int cyl, int head)
{
	int key = cyl * 2 + head;
	auto it = cache.find(key);
	if (it != cache.end()) return it->second;
	Track t;
	t.meta.resize(ST_MFMT_META_BYTES);
	t.cells.resize(ST_MFMT_CELLS_BYTES);
	TrackBuilder tb;
	stx_build_track(*g_img, cyl, head, tb);
	tb.encode(cyl, head, t.meta.data(), t.cells.data());
	return cache[key] = t;
}

// ---- hps_io sd emulation ----
struct Sd
{
	int state = 0;       // 0 idle, 1 ack, 2 data, 3 done
	std::vector<uint16_t> words;
	size_t i = 0;
	int wait = 0;
	int requests = 0;
} sd;

static void sd_step()
{
	top->sd_dout_strobe = 0;
	switch (sd.state)
	{
	case 0:
		if (top->sd_rd)
		{
			uint32_t lba = top->sd_lba;
			int nwords = (top->sd_blk_cnt + 1) * 256;
			sd.words.assign(nwords, 0);
			if (lba & 0x80000000u)
			{
				const Track &t = get_track(lba & 0xFF, (lba >> 8) & 1);
				const std::vector<uint8_t> &src = ((lba >> 16) & 3) ? t.cells : t.meta;
				memcpy(sd.words.data(), src.data(), std::min(src.size(), (size_t)nwords * 2));
			}
			sd.requests++;
			top->sd_ack = 1;
			sd.state = 1;
			sd.wait = 20;
		}
		break;
	case 1:
		if (--sd.wait == 0) { sd.state = 2; sd.i = 0; }
		break;
	case 2:
		// one word every 3 clocks, like a fast SPI transfer
		if (ticks % 3 == 0)
		{
			if (sd.i < sd.words.size())
			{
				top->sd_buff_addr = (uint16_t)sd.i;
				top->sd_dout = sd.words[sd.i];
				top->sd_dout_strobe = 1;
				sd.i++;
			}
			else { sd.state = 3; sd.wait = 5; }
		}
		break;
	case 3:
		if (--sd.wait == 0) { top->sd_ack = 0; sd.state = 0; }
		break;
	}
}

static void tick()
{
	sd_step();
	top->clk = 0; top->eval();
	top->clk = 1; top->eval();
	ticks++;
}

// weak mask per cell from the META part
static std::vector<uint8_t> weak_cells(const Track &t, uint32_t ncells)
{
	std::vector<uint8_t> w(ncells, 0);
	const st_mfmt_header *h = (const st_mfmt_header *)t.meta.data();
	const st_mfmt_weak *seg = (const st_mfmt_weak *)(t.meta.data() + ST_MFMT_WEAK_OFS);
	const uint16_t *pool = (const uint16_t *)(t.meta.data() + ST_MFMT_POOL_OFS);
	size_t p = 0;
	for (int s = 0; s < h->weak_count; s++)
		for (int i = 0; i < seg[s].n_words; i++, p++)
			for (int b = 0; b < 16; b++)
			{
				uint32_t c = (seg[s].start_word + i) * 16 + b;
				if (c < ncells && ((pool[p] >> (15 - b)) & 1)) w[c] = 1;
			}
	return w;
}

struct RtlId { uint8_t id[6]; uint32_t cell; uint64_t tick; };

struct Rev
{
	std::vector<uint8_t> bits;
	std::vector<uint64_t> cell_ticks;
	std::vector<uint8_t> rt;
	std::vector<RtlId> ids;
	uint64_t t_start = 0, t_end = 0;
};

// Wait until the head is on cell ncells-1, then record cells 0..ncells-1
static bool record_rev(uint32_t ncells, Rev &r)
{
	uint64_t limit = ticks + (uint64_t)ncells * (36000 / ACC_STEP + 2) * 3 + 2000000;
	// wait for the last cell of the turn
	while (!(top->cell_stb && top->position == ncells - 1)) { tick(); if (ticks > limit) return false; }
	tick();
	top->rt_restart = 1; top->am_restart = 1; top->am_mode = 0;
	tick();
	top->rt_restart = 0; top->am_restart = 0;

	int am_state = 0, syncs = 0, nfield = 0;
	uint32_t sync_cell[3] = {};
	RtlId cur = {};
	bool started = false;
	while (true)
	{
		tick();
		if (ticks > limit) return false;
		if (top->cell_stb)
		{
			uint32_t pos = top->position;
			if (!started)
			{
				if (pos != 0) return false;
				started = true;
				r.t_start = ticks;
			}
			else if (pos == 0) { r.t_end = ticks; break; }
			r.bits.push_back(top->cell_bit);
			r.cell_ticks.push_back(ticks);
		}
		if (!started) continue;
		if (top->rt_stb) r.rt.push_back(top->rt_val);
		if (top->am_stb)
		{
			uint32_t last = (uint32_t)r.bits.size() - 1;   // cell that completed this byte
			if (am_state == 0)
			{
				if (top->am_sync)
				{
					sync_cell[0] = sync_cell[1]; sync_cell[1] = sync_cell[2]; sync_cell[2] = last - 15;
					syncs++;
				}
				else
				{
					if (syncs >= 3 && top->am_val == 0xFE)
					{
						cur = {};
						cur.cell = sync_cell[0];
						cur.tick = r.cell_ticks[sync_cell[0]];
						am_state = 1; nfield = 0;
						top->am_mode = 1;
					}
					syncs = 0;
				}
			}
			else
			{
				cur.id[nfield++] = top->am_val;
				if (nfield == 6)
				{
					r.ids.push_back(cur);
					am_state = 0; syncs = 0;
					top->am_mode = 0;
				}
			}
		}
	}
	return true;
}

static int check_track(int cyl, int head, bool verbose)
{
	const Track &t = get_track(cyl, head);
	const st_mfmt_header *h = (const st_mfmt_header *)t.meta.data();

	top->reset = 1; tick(); tick(); top->reset = 0;
	top->invalidate = 1; tick(); top->invalidate = 0;
	top->enable = 1; top->cyl = cyl; top->head = head; top->drive = 0;
	uint64_t limit = ticks + 1000000;
	while (!top->loaded) { tick(); if (ticks > limit) { printf("c%02d h%d: never loaded\n", cyl, head); return 1; } }

	bool formatted = h->flags & ST_MFMT_F_FORMATTED;
	uint32_t ncells = formatted ? h->cell_count : 100000;
	Rev r;
	if (!record_rev(ncells, r)) { printf("c%02d h%d: no revolution recorded\n", cyl, head); return 1; }

	int fails = 0;
	char why[512] = "";
	auto fail = [&](const char *m) { fails++; strncat(why, m, sizeof(why) - strlen(why) - 1); };

	if (r.bits.size() != ncells) { char b[80]; snprintf(b, sizeof(b), " cells=%zu/%u", r.bits.size(), ncells); fail(b); }

	WdSim wd;
	std::string err;
	wd.load(t.meta.data(), t.cells.data(), err);
	wd.spin(3, 99);
	const auto &ref = wd.stream();

	int bit_err = 0, weak_n = 0;
	double rt_pct = 0;
	size_t ids_ok = 0, ids_ref = 0;
	double max_id_dt = 0, rev_err_ns = 0;

	if (formatted)
	{
		auto weak = weak_cells(t, ncells);
		for (uint32_t c = 0; c < ncells && c < r.bits.size(); c++)
		{
			if (weak[c]) { weak_n++; continue; }
			if (r.bits[c] != ref[c + ncells].bit) bit_err++;
		}
		if (bit_err) { char b[64]; snprintf(b, sizeof(b), " bit_errors=%d", bit_err); fail(b); }

		// revolution time: clocks * ACC_STEP units vs the sum of the cells
		double rtl_ns = (double)(r.t_end - r.t_start) * ACC_STEP / 16.0;
		rev_err_ns = rtl_ns - wd.rev_ns();
		if (fabs(rev_err_ns) > 2.0 * ACC_STEP / 16.0) { char b[64]; snprintf(b, sizeof(b), " rev_err=%.0fns", rev_err_ns); fail(b); }

		// ID fields
		auto rids = wd.read_addresses(0, ncells);
		ids_ref = rids.size();
		for (auto &x : rids)
		{
			for (auto &y : r.ids)
				if (y.cell == x.cell && !memcmp(y.id, x.id, 6))
				{
					ids_ok++;
					double dt = fabs((double)(y.tick - r.t_start) * ACC_STEP / 16.0 - x.t);
					if (dt > max_id_dt) max_id_dt = dt;
					break;
				}
		}
		if (ids_ok != ids_ref || r.ids.size() != ids_ref) { char b[64]; snprintf(b, sizeof(b), " ids=%zu/%zu(rtl %zu)", ids_ok, ids_ref, r.ids.size()); fail(b); }
		if (max_id_dt > 2.0 * ACC_STEP / 16.0) { char b[64]; snprintf(b, sizeof(b), " id_time=%.0fns", max_id_dt); fail(b); }

		// read track
		auto rt = wd.read_track(ncells);   // 2nd turn: the shift register has the end of the 1st
		size_t n = std::min(rt.size(), r.rt.size()), eq = 0;
		for (size_t i = 0; i < n; i++) if (rt[i] == r.rt[i]) eq++; else if (verbose) printf("  rt[%zu]: model %02x rtl %02x (of %zu/%zu)\n", i, rt[i], r.rt[i], rt.size(), r.rt.size());
		rt_pct = 100.0 * eq / std::max(rt.size(), (size_t)1);
		if (rt.size() != r.rt.size() && !weak_n) { char b[64]; snprintf(b, sizeof(b), " rt_len=%zu/%zu", r.rt.size(), rt.size()); fail(b); }
		if (rt_pct < (weak_n ? 90.0 : 100.0)) { char b[64]; snprintf(b, sizeof(b), " readtrack=%.1f%%", rt_pct); fail(b); }
	}
	else
	{
		// noise: nothing must look like an ID field
		if (!r.ids.empty()) fail(" ids_on_unformatted");
	}

	if (verbose || fails)
		printf("c%02d h%d: %s cells=%u weak=%d ids=%zu/%zu rt=%.1f%% rev_err=%.0fns id_dt=%.0fns%s %s\n",
			cyl, head, formatted ? "fmt" : "noise", ncells, weak_n, ids_ok, ids_ref, rt_pct, rev_err_ns, max_id_dt,
			why, fails ? "FAIL" : "ok");
	return fails ? 1 : 0;
}

// Step while the disk turns: the rotation must continue across the reload
static int check_step(int c1, int c2)
{
	top->reset = 1; tick(); top->reset = 0;
	top->invalidate = 1; tick(); top->invalidate = 0;
	top->enable = 1; top->cyl = c1; top->head = 0;
	while (!top->loaded) tick();
	for (int i = 0; i < 30000; i++) tick();
	uint32_t before = top->position;
	uint64_t t0 = ticks;
	top->cyl = c2;
	int reqs = sd.requests;
	while (top->loaded) tick();
	while (!top->loaded) tick();
	uint32_t after = top->position;
	double cells_expected = (double)(ticks - t0) * ACC_STEP / 32000.0;
	int moved = (int)after - (int)before;
	bool ok = sd.requests == reqs + 2 && fabs(moved - cells_expected) < cells_expected * 0.1 + 32;
	printf("step c%d -> c%d during playback: position %u -> %u (%d cells, expected ~%.0f), %d requests %s\n",
		c1, c2, before, after, moved, cells_expected, sd.requests - reqs, ok ? "ok" : "FAIL");
	// and the new track must play correctly
	return (ok ? 0 : 1) + check_track(c2, 0, true);
}

int main(int argc, char **argv)
{
	Verilated::commandArgs(argc, argv);
	top = new Vtb_top;
	int only_c = -1, only_h = -1, fails = 0, total = 0;
	bool step = false, verbose = false;
	std::vector<const char *> files;
	for (int i = 1; i < argc; i++)
	{
		if (!strcmp(argv[i], "-t") && i + 2 < argc) { only_c = atoi(argv[++i]); only_h = atoi(argv[++i]); }
		else if (!strcmp(argv[i], "-s")) step = true;
		else if (!strcmp(argv[i], "-v")) verbose = true;
		else if (argv[i][0] != '+') files.push_back(argv[i]);
	}
	for (const char *f : files)
	{
		FILE *fp = fopen(f, "rb");
		if (!fp) { perror(f); return 2; }
		std::vector<uint8_t> buf;
		fseek(fp, 0, SEEK_END); buf.resize(ftell(fp)); fseek(fp, 0, SEEK_SET);
		if (fread(buf.data(), 1, buf.size(), fp) != buf.size()) return 2;
		fclose(fp);
		StxImage img;
		if (!img.parse(buf.data(), buf.size())) { printf("%s: %s\n", f, img.error()); return 2; }
		g_img = &img;
		cache.clear();
		int ff = 0, ft = 0;
		for (int c = 0; c < img.cylinders(); c++)
			for (int h = 0; h < img.heads(); h++)
			{
				if (only_c >= 0 && (c != only_c || h != only_h)) continue;
				ff += check_track(c, h, verbose);
				ft++;
			}
		if (step && img.cylinders() > 3) { ff += check_step(1, 2); ft++; }
		printf("%s: %d/%d tracks ok (ACC_STEP %d)\n", f, ft - ff, ft, ACC_STEP);
		fails += ff; total += ft;
	}
	printf("%s\n", fails ? "SOME TRACKS FAILED" : "all tracks ok");
	delete top;
	return fails ? 1 : 0;
}
