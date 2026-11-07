// tb_fdc.cpp - Verilator test of the whole fdc1772 with an MFMT (STX) image
//
// The C++ side is Main (it serves MFMT tracks converted with Main's code)
// and the ST (it drives the WD1772 registers like TOS and the DMA: commands,
// DRQ data reads, status after the interrupt). For every track it checks:
//   - read address: every STX ID is returned, with its CRC status
//   - read sector: data, CRC error, record not found, deleted data, timing
//   - read track: same bytes as the software WD1772 model
//
// Usage: tb_fdc [-t cyl head] [-n every] [-v] file.stx...
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <map>
#include <set>
#include <vector>
#include <utility>
#include <string>
#include "Vfdc1772.h"
#include "Vfdc1772___024root.h"
#include "Vfdc1772_fdc1772.h"
#include "verilated.h"
#include "st_stx.h"
#include "wdsim.h"

// The clock is always 32 MHz (31.25 ns per tick, clk8m_en every 4 ticks).
// TB_CLKCPU_KHZ is the value given to the core's MFMT drive: below 32000 the
// disk turns faster than real by 32000 / TB_CLKCPU_KHZ, to test many tracks.
#ifndef TB_CLKCPU_KHZ
#define TB_CLKCPU_KHZ 32000
#endif
static const double NS_PER_TICK = 31.25;
static const double DISK_SCALE = 32000.0 / TB_CLKCPU_KHZ;

static Vfdc1772 *top;
static uint64_t ticks;
static int verbose = 0;

struct Track { std::vector<uint8_t> meta, cells; };
static const StxImage *g_img;
static std::vector<uint8_t> st_file;   // plain .st image for the sector path regression test
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

// ---- Main side: hps_io sd emulation ----
static struct { int state = 0, wait = 0; std::vector<uint16_t> words; size_t i = 0; } sd;
static uint32_t sd_latency_max = 0;     // ticks; random delay before Main answers (real Main polls)
static uint32_t lat_rng = 1;
static uint32_t rnd() { lat_rng = lat_rng * 1664525u + 1013904223u; return lat_rng >> 8; }

static void sd_step()
{
	top->sd_dout_strobe = 0;
	switch (sd.state)
	{
	case 0:
		if (top->sd_wr && !top->sd_rd)
		{
			// write (sector path only): acknowledge, the data is not kept
			sd.words.clear();
			top->sd_ack = 1;
			sd.state = 3; sd.wait = 300;
		}
		else if (top->sd_rd)
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
			else if (!st_file.empty())
			{
				size_t off = (size_t)lba * 512;
				if (off < st_file.size()) memcpy(sd.words.data(), &st_file[off], std::min(st_file.size() - off, (size_t)nwords * 2));
			}
			else printf("  unexpected sector request lba %u\n", lba);
			sd.state = 4;
			sd.wait = 20 + (sd_latency_max ? rnd() % sd_latency_max : 0);
		}
		break;
	case 4: if (--sd.wait == 0) { top->sd_ack = 1; sd.state = 1; sd.wait = 20; } break;
	case 1: if (--sd.wait == 0) { sd.state = 2; sd.i = 0; } break;
	case 2:
		if (ticks % 3 == 0)
		{
			if (sd.i < sd.words.size())
			{
				top->sd_buff_addr = (uint16_t)sd.i;
				top->sd_dout = sd.words[sd.i++];
				top->sd_dout_strobe = 1;
			}
			else { sd.state = 3; sd.wait = 5; }
		}
		break;
	case 3: if (--sd.wait == 0) { top->sd_ack = 0; sd.state = 0; } break;
	}
}

struct Ev { uint64_t t; char what; int v; };
static std::vector<Ev> evs;
static void tick_events()
{
	static int pv = 0, pi = 0, pe = 0;
	auto *R = top->rootp->fdc1772;
	int v = R->mfmt_drive__DOT__valid, ix = R->mf_index, es = R->mf_estate;
	if (v != pv) evs.push_back({ ticks, 'L', v });
	if (ix && !pi) evs.push_back({ ticks, 'I', (int)R->mf_position });
	if (es != pe) evs.push_back({ ticks, 'E', es });
	if (R->mf_dec_stb && es == 2 && 0) evs.push_back({ ticks, 'b', R->mf_dec_val });
	pv = v; pi = ix; pe = es;
}

static void tick()
{
	tick_events();
	sd_step();
	top->clk8m_en = (ticks % 4) == 0;
	top->clkcpu = 0; top->eval();
	top->clkcpu = 1; top->eval();
	ticks++;
}
static void ticks_n(int n) { while (n--) tick(); }

// ---- ST side: register access ----
enum { R_CMD = 0, R_TRACK = 1, R_SECTOR = 2, R_DATA = 3 };

static void wr(int reg, uint8_t v)
{
	top->cpu_addr = reg; top->cpu_din = v; top->cpu_rw = 0; top->cpu_sel = 1;
	ticks_n(8);
	top->cpu_sel = 0; top->cpu_rw = 1;
	ticks_n(8);
}

// drain the debug event queue (what Main writes to stx.log)
static void dbg_drain()
{
	int n = 0;
	while (top->dbg_valid && n < 200)
	{
		const uint32_t *w = top->dbg_q;   // VlWide<3>: w[0] = bits 31:0
		unsigned type = w[2] >> 24;
		uint32_t us = ((w[2] & 0xFFFFFF) << 8) | (w[1] >> 24);
		uint64_t pl = ((uint64_t)(w[1] & 0xFFFFFF) << 32) | w[0];
		printf("  dbg type %u t %u us payload %014llx\n", type, us, (unsigned long long)pl);
		top->dbg_pop = !top->dbg_pop;
		tick(); tick(); tick();
		n++;
	}
}

static uint8_t rd(int reg)
{
	top->cpu_addr = reg; top->cpu_rw = 1; top->cpu_sel = 1;
	ticks_n(2);
	uint8_t v = top->cpu_dout;
	ticks_n(2);
	top->cpu_sel = 0;
	ticks_n(2);
	return v;
}

struct Cmd
{
	uint8_t status = 0;
	bool timeout = false;
	std::vector<uint8_t> data;
	std::vector<uint64_t> when;
};

// issue a command, read data at every DRQ, stop at the interrupt
static Cmd run(uint8_t cmd, double max_ms)
{
	Cmd c;
	evs.clear();
	wr(R_CMD, cmd);
	uint64_t limit = ticks + (uint64_t)(max_ms * 1e6 / NS_PER_TICK);
	while (!top->irq)
	{
		if (top->drq)
		{
			uint64_t t = ticks;
			c.data.push_back(rd(R_DATA));
			c.when.push_back(t);
		}
		else tick();
		if (ticks > limit) { c.timeout = true; break; }
	}
	c.status = rd(R_CMD);
	return c;
}

static double ms_to_disk(double ms) { return ms / DISK_SCALE; }   // real ms -> simulated ms

// Type I status bit 1 is the index pulse: high only during the pulse, once per turn.
// Polls the status register for a bit more than one turn after a type I command.
static bool check_index_bit(const char *what, double turn_ms)
{
	double span = turn_ms * 1.3;
	uint64_t end = ticks + (uint64_t)(span * 1e6 / NS_PER_TICK);
	int hi = 0, n = 0, rises = 0, prev = -1;
	while (ticks < end)
	{
		int b = (rd(R_CMD) >> 1) & 1;
		if (b && prev != 1) rises++;   // a pulse already in progress at the start counts too
		prev = b; hi += b; n++;
		ticks_n(40);
	}
	double pct = 100.0 * hi / n;
	bool ok = rises == 1 && pct > 0.5 && pct < 6.0;
	printf("%s: index bit high %.1f %% of the time, %d pulse(s) in %.0f ms %s\n", what, pct, rises, span, ok ? "ok" : "FAIL");
	return ok;
}

static int check_track(int cyl, int head, int &cur_cyl)
{
	const StxTrack *t = g_img->find(cyl, head);
	const Track &tr = get_track(cyl, head);
	const st_mfmt_header *h = (const st_mfmt_header *)tr.meta.data();
	bool formatted = h->flags & ST_MFMT_F_FORMATTED;
	int fails = 0;
	std::string why;
	auto fail = [&](const char *fmt, int a = 0, int b = 0, int c2 = 0) {
		char m[160]; snprintf(m, sizeof(m), fmt, a, b, c2); why += " "; why += m; fails++;
	};

	// seek (h=1: no spin-up wait, 6 ms steps, no verify)
	top->floppy_side = head ? 0 : 1;      // ST side select is active low
	if (cyl != cur_cyl)
	{
		wr(R_DATA, cyl);
		Cmd s = run(0x18, 2000);
		if (s.timeout) { printf("c%02d h%d: seek timeout\n", cyl, head); return 1; }
		cur_cyl = cyl;
	}
	// let the drive load the track (a real program waits for the head to settle)
	ticks_n((int)(30e6 / NS_PER_TICK));

	int ns = t ? (int)t->sectors.size() : 0;
	const double rev_ms = 200.0;

	// ---- read address: collect IDs for a bit more than one turn
	std::map<std::vector<uint8_t>, int> seen;   // id -> status bits
	int ra = 0;
	if (formatted && ns)
	{
		for (int i = 0; i < ns + 2; i++)
		{
			Cmd c = run(0xC8, ms_to_disk(rev_ms * 7));
			if (c.timeout) { fail("read address timeout"); break; }
			if (c.status & 0x10) { fail("read address RNF"); break; }
			if (c.data.size() != 6) { fail("read address %d bytes", (int)c.data.size()); break; }
			seen[c.data] = c.status & 0x08;
			if (rd(R_SECTOR) != c.data[0]) fail("read address: sector reg %d, ID track %d", rd(R_SECTOR), c.data[0]);
			ra++;
		}
		for (auto &sec : t->sectors)
		{
			std::vector<uint8_t> id(sec.id, sec.id + 6);
			bool id_err = (sec.status & STX_SECT_RNF) && (sec.status & STX_SECT_CRC_ERROR);
			bool found = false;
			for (auto &kv : seen)
				if (!memcmp(kv.first.data(), sec.id, 4) && (id_err || !memcmp(kv.first.data() + 4, sec.id + 4, 2)))
				{
					found = true;
					if (!!kv.second != id_err) fail("ID %02x: CRC bit %d, STX %d", sec.id[2], !!kv.second, id_err);
				}
			if (!found) fail("ID %02x.%02x not returned by read address", sec.id[0], sec.id[2]);
		}
	}

	// ---- read sector for every distinct track/sector pair of the STX
	int rs = 0, rs_ok = 0;
	double max_terr = 0;
	std::set<int> done;
	for (int s = 0; s < ns; s++)
	{
		const StxSector &sec = t->sectors[s];
		int key = sec.id[0] * 256 + sec.id[2];
		if (done.count(key)) continue;
		done.insert(key);

		// the WD1772 can return any sector with this track/sector pair and a good ID CRC
		std::vector<const StxSector *> cand;
		for (auto &x : t->sectors)
			if (x.id[0] == sec.id[0] && x.id[2] == sec.id[2] && !((x.status & STX_SECT_RNF) && (x.status & STX_SECT_CRC_ERROR)))
				cand.push_back(&x);
		bool any_data = false;
		for (auto *x : cand) if (!(x->status & STX_SECT_RNF)) any_data = true;

		wr(R_TRACK, sec.id[0]);
		wr(R_SECTOR, sec.id[2]);
		Cmd c = run(0x88, ms_to_disk(rev_ms * 7));
		rs++;
		if (c.timeout) { fail("read sector %02x timeout", sec.id[2]); continue; }

		if (!any_data)
		{
			if (!(c.status & 0x10)) fail("sector %02x: expected RNF, status %02x", sec.id[2], c.status);
			else rs_ok++;
			continue;
		}
		if (c.status & 0x10) { fail("sector %02x: RNF (status %02x)", sec.id[2], c.status); continue; }

		const StxSector *m = nullptr;
		for (auto *x : cand)
		{
			if (x->status & STX_SECT_RNF || (int)c.data.size() != x->size) continue;
			bool eq = true;
			for (int i = 0; i < x->size && eq; i++)
			{
				uint8_t mk = x->fuzzy ? x->fuzzy[i] : 0xFF;
				if ((c.data[i] & mk) != (x->data[i] & mk)) eq = false;
			}
			if (eq) { m = x; break; }
		}
		if (!m) { fail("sector %02x: data matches no STX sector (%d bytes)", sec.id[2], (int)c.data.size()); continue; }

		bool exp_crc = (m->status & STX_SECT_CRC_ERROR) || m->fuzzy;
		bool exp_del = m->status & STX_SECT_DELETED;
		if (!!(c.status & 0x08) != exp_crc) fail("sector %02x: CRC bit %d expected %d", sec.id[2], !!(c.status & 0x08), exp_crc);
		if (!!(c.status & 0x20) != exp_del) fail("sector %02x: record type %d expected %d", sec.id[2], !!(c.status & 0x20), exp_del);
		if (c.status & 0x04) fail("sector %02x: lost data", sec.id[2]);

		// timing: first to last data byte, in real time
		double meas = (c.when.back() - c.when.front()) * NS_PER_TICK * DISK_SCALE;
		double exp = 0;
		bool timed = m->timing || m->read_time;
		for (int i = 1; i < m->size; i++)
		{
			double bt = 0;
			if (m->timing)
			{
				int blk = i >> 4;
				uint16_t v = (uint16_t)((m->timing[blk * 2] << 8) | m->timing[blk * 2 + 1]);
				bt = (v * 32.0 + 28.0) * 125.0 / 16.0;
			}
			else if (m->read_time) bt = m->read_time * 1000.0 / m->size;
			else bt = h->rev_time_ns / (h->cell_count / 16.0);
			exp += bt;
		}
		double terr = fabs(meas - exp) / exp * 100.0;
		if (timed && verbose) printf("  sector %02x timed: %.1f us, expected %.1f us\n", sec.id[2], meas / 1000, exp / 1000);
		if (timed && terr > max_terr) max_terr = terr;
		if (timed && terr > 0.3) fail("sector %02x: %d us, expected %d us", sec.id[2], (int)(meas / 1000), (int)(exp / 1000));
		rs_ok++;
	}

	// ---- read track
	double rt_pct = -1;
	if (formatted)
	{
		Cmd c = run(0xE8, ms_to_disk(rev_ms * 3));
		if (c.timeout) fail("read track timeout");
		else
		{
			WdSim wd;
			std::string err;
			wd.load(tr.meta.data(), tr.cells.data(), err);
			wd.spin(3, 7);
			auto ref = wd.read_track(wd.cells_per_rev());
			size_t n = std::min(ref.size(), c.data.size()), eq = 0;
			for (size_t i = 0; i < n; i++) if (ref[i] == c.data[i]) eq++;
			rt_pct = 100.0 * eq / ref.size();
			bool weak = h->weak_count > 0;
			if (c.data.size() != ref.size() && !weak) fail("read track %d bytes, model %d", (int)c.data.size(), (int)ref.size());
			if (rt_pct < (weak ? 90.0 : 100.0)) fail("read track matches %d %%", (int)rt_pct);
			if (c.status & 0x04) fail("read track lost data");
		}
	}

	if (verbose || fails)
		printf("c%02d h%d: %s ids=%zu/%d rsec=%d/%d max_terr=%.2f%% rtrack=%.1f%%%s %s\n", cyl, head,
			formatted ? "fmt" : "noise", seen.size(), ns, rs_ok, rs, max_terr, rt_pct, why.c_str(), fails ? "FAIL" : "ok");
	return fails ? 1 : 0;
}

// Regression test of the unchanged sector path: plain .st image, 9 sectors/track, 2 sides
static int check_st()
{
	const int spt = 9, sides = 2, tracks = 80;
	st_file.resize((size_t)tracks * sides * spt * 512);
	uint32_t x = 12345;
	for (auto &b : st_file) { x = x * 1103515245u + 12345u; b = x >> 16; }

	top->floppy_reset = 0; top->cpu_sel = 0; top->cpu_rw = 1;
	top->floppy_drive = 2; top->floppy_side = 1;
	top->img_wp = 0; top->img_mfmt = 0; top->img_size = 0; top->img_mounted = 0;
	ticks_n(100);
	top->floppy_reset = 1;
	ticks_n(100);
	top->img_size = (uint32_t)st_file.size(); top->img_mfmt = 0; top->img_mounted = 1;
	ticks_n(4);
	top->img_mounted = 0;
	ticks_n(100);
	if (run(0x08, 3000).timeout) { printf(".st: restore timeout\n"); return 1; }

	int fails = 0, checked = 0, cur = 0;
	if (!check_index_bit(".st type I status", 200.0)) fails++;
	for (int trk : { 0, 1, 40, 79 })
		for (int side = 0; side < sides; side++)
		{
			top->floppy_side = side ? 0 : 1;
			if (trk != cur) { wr(R_DATA, trk); run(0x18, 2000); cur = trk; }
			Cmd a = run(0xC8, 2000);
			if (a.data.size() != 6 || a.data[0] != trk || a.data[3] != 2 || (a.status & 0x18)) { printf(".st c%02d h%d: read address bad\n", trk, side); fails++; }
			for (int sec = 1; sec <= spt; sec++)
			{
				wr(R_TRACK, trk);
				wr(R_SECTOR, sec);
				Cmd c = run(0x88, 2000);
				size_t off = (((size_t)trk * sides + side) * spt + sec - 1) * 512;
				bool ok = !c.timeout && c.data.size() == 512 && !(c.status & 0x1C) && !memcmp(c.data.data(), &st_file[off], 512);
				if (!ok) { printf(".st c%02d h%d s%d: FAIL (status %02x, %zu bytes)\n", trk, side, sec, c.status, c.data.size()); fails++; }
				checked++;
			}
			// writes on a .st are still accepted (no write protect)
			wr(R_TRACK, trk);
			wr(R_SECTOR, 1);
			Cmd w = run(0xA8, 2000);
			if (w.status & 0x40) { printf(".st c%02d h%d: write reported write protect\n", trk, side); fails++; }
		}
	printf(".st sector path: %d sectors read, %s\n", checked, fails ? "FAIL" : "ok");
	st_file.clear();
	return fails;
}


// Random reads like a game loader: random track, side and moment, side
// toggles right before the command, commands right after a seek (track still
// loading), idle times long enough for the motor to stop, h/e flags.
static int stress(int iters, int &cur)
{
	struct Ref { int cyl, head; const StxSector *s; };
	std::vector<Ref> refs;
	for (auto &t : g_img->tracks())
	{
		for (auto &s : t.sectors)
		{
			if (!s.data || (s.status & 0x18) || s.fuzzy) continue;
			int dup = 0;
			for (auto &x : t.sectors) if (x.id[0] == s.id[0] && x.id[2] == s.id[2]) dup++;
			if (dup == 1) refs.push_back({ t.cylinder, t.head, &s });
		}
	}
	if (refs.empty()) return 0;
	int fails = 0;
	for (int it = 0; it < iters; it++)
	{
		const Ref &r = refs[rnd() % refs.size()];
		uint32_t a = rnd();
		if ((a & 15) == 0) ticks_n((int)((rnd() % 2600) * 1e6 / NS_PER_TICK / (DISK_SCALE > 1 ? 1 : 1)));  // up to 2.6 s idle: motor off
		else ticks_n(rnd() % 200000);
		top->floppy_side = r.head ? 0 : 1;
		int sought = r.cyl != cur;
		if (r.cyl != cur)
		{
			wr(R_DATA, r.cyl);
			Cmd sk = run(0x18 | (rnd() & 3), 3000);
			if (sk.timeout) { printf("stress %d: seek timeout\n", it); fails++; continue; }
			cur = r.cyl;
		}
		if ((a >> 4 & 3) == 0)
		{
			// toggle the side for a moment: the drive starts loading the other side
			top->floppy_side = !top->floppy_side;
			ticks_n(rnd() % 50000);
			top->floppy_side = r.head ? 0 : 1;
		}
		wr(R_TRACK, r.s->id[0]);
		wr(R_SECTOR, r.s->id[2]);
		uint8_t cmd = 0x80 | ((a >> 6 & 1) ? 0x08 : 0) | ((a >> 7 & 1) ? 0x04 : 0);
		Cmd c = run(cmd, ms_to_disk(200.0 * 8) + 3000.0 * (DISK_SCALE > 1 ? 1.0 / DISK_SCALE : 1) + 100);
		bool ok = !c.timeout && !(c.status & 0x1C) && (int)c.data.size() == r.s->size && !memcmp(c.data.data(), r.s->data, r.s->size);
		if (!ok)
		{
			fails++;
			printf("stress %d: c%02d h%d sector %02x cmd %02x: %s status %02x, %zu bytes [long idle %d, seek %d, side toggle %d]\n", it, r.cyl, r.head, r.s->id[2], cmd,
				c.timeout ? "TIMEOUT" : "FAIL", c.status, c.data.size(), (a & 15) == 0, sought, (a >> 4 & 3) == 0);
			{
				uint64_t t0 = evs.empty() ? ticks : evs.front().t;
				printf("    events (ms of disk time from the command):");
				for (auto &e : evs) printf(" %c%d@%.1f", e.what, e.v, (e.t - t0) * NS_PER_TICK * DISK_SCALE / 1e6);
				printf("\n");
			}
			Cmd dbg = run(0xC8, ms_to_disk(200.0 * 8) + 100);
			{
				auto *R = top->rootp->fdc1772;
				printf("    loader: lstate %d valid %d key_valid %d key_cyl %d key_head %d sd_rd %d sd_ack %d tb_sd_state %d mf_busy %d\n",
					R->mfmt_drive__DOT__lstate, R->mfmt_drive__DOT__valid, R->mfmt_drive__DOT__key_valid,
					R->mfmt_drive__DOT__key_cyl, R->mfmt_drive__DOT__key_head, top->sd_rd, top->sd_ack, sd.state, R->mf_sd_busy);
				printf("    play: cur_ok %d sstate %d nw_ready %d nw_w %d c %d sbusy %d pf_go %d t_i %d\n",
					R->mfmt_drive__DOT__cur_ok, R->mfmt_drive__DOT__sstate, R->mfmt_drive__DOT__nw_ready, R->mfmt_drive__DOT__nw_w,
					R->mfmt_drive__DOT__c, R->mfmt_drive__DOT__sbusy, R->mfmt_drive__DOT__pf_go, R->mfmt_drive__DOT__t_i);
			}
			printf("    then read address: status %02x id %s (physical track wanted %d)\n", dbg.status,
				dbg.data.size() == 6 ? (std::to_string(dbg.data[0]) + "/" + std::to_string(dbg.data[1]) + "/" + std::to_string(dbg.data[2])).c_str() : "-", r.cyl);
		}
		if ((a >> 8 & 7) == 0)
		{
			Cmd ra = run(0xC8, ms_to_disk(200.0 * 8) + 100);
			if (ra.timeout || (ra.status & 0x10) || ra.data.size() != 6) { fails++; printf("stress %d: read address failed (status %02x)\n", it, ra.status); }
		}
		if (it % 100 == 99) printf("stress: %d reads, %d failures\n", it + 1, fails);
	}
	return fails;
}


// Turrican's protection check as Hatari's FDC trace shows it: a read of
// c79/h1 s1, restore, seek 7, sector 0 then sector 0x10 of c7/h0 with DMA
// switched off after 64 resp. 48 bytes, seek 79 and status polling until the
// motor stops. The bytes the loader keeps must be those of the STX sectors.
static Cmd run_partial(uint8_t cmd, size_t keep, double max_ms, uint64_t &first_t)
{
	Cmd c;
	wr(R_CMD, cmd);
	uint64_t t0 = ticks, limit = ticks + (uint64_t)(max_ms * 1e6 / NS_PER_TICK);
	first_t = 0;
	bool prev = false;
	while (!top->irq)
	{
		bool d = top->drq;
		if (d && !prev && c.data.size() < keep)
		{
			if (!first_t) first_t = ticks - t0;
			c.data.push_back(rd(R_DATA));
		}
		else tick();
		prev = d && c.data.size() >= keep;
		if (ticks > limit) { c.timeout = true; break; }
	}
	c.status = rd(R_CMD);
	return c;
}

static int turrican(int iters, int &cur)
{
	const StxTrack *t = g_img->find(7, 0);
	const StxSector *s0 = nullptr, *s16 = nullptr;
	if (t) for (auto &s : t->sectors) { if (s.id[2] == 0x00) s0 = &s; if (s.id[2] == 0x10) s16 = &s; }
	if (!s0 || !s16 || !s0->data || !s16->data) { printf("turrican: sectors 0/0x10 of c7/h0 not found\n"); return 1; }
	double tick_ms = NS_PER_TICK * DISK_SCALE / 1e6;
	int fails = 0;
	for (int it = 0; it < iters; it++)
	{
		ticks_n(rnd() % 3000000);
		uint64_t ft;
		top->floppy_side = 0;
		wr(R_DATA, 79); run(0x13, 4000); cur = 79;
		wr(R_SECTOR, 1);
		Cmd a = run(0x80, 3000);
		top->floppy_side = 1;
		run(0x01, 4000); cur = 0;
		wr(R_DATA, 7); run(0x11, 4000); cur = 7;
		wr(R_SECTOR, 0);
		Cmd r0 = run_partial(0x80, 112, 3000, ft);
		double ft0 = ft * tick_ms;
		wr(R_SECTOR, 0x10);
		Cmd r16 = run_partial(0x80, 48, 3000, ft);
		double ft16 = ft * tick_ms;
		wr(R_DATA, 79); run(0x11, 4000); cur = 79;
		// poll like the loader until the motor bit drops
		uint64_t lim = ticks + (uint64_t)(ms_to_disk(6000) * 1e6 / NS_PER_TICK);
		int pulses = 0, prev = 0;
		uint8_t st = 0;
		while (ticks < lim)
		{
			st = rd(R_CMD);
			int b = (st >> 1) & 1;
			if (b && !prev) pulses++;
			prev = b;
			if (!(st & 0x80)) break;
			ticks_n(200);
		}
		// the loader's test: sector 0 bytes 64-79 FF and more than 204 of the
		// 256 bits of bytes 80-111 at 0; sector 16 bytes 0-15 00 and more than
		// 204 bits of bytes 16-47 at 0
		auto zeros = [](const std::vector<uint8_t> &v, size_t a, size_t b) {
			int z = 0; for (size_t i = a; i < b && i < v.size(); i++) z += 8 - __builtin_popcount(v[i]); return z; };
		bool ok0 = r0.data.size() == 112 && zeros(r0.data, 64, 80) == 0 && zeros(r0.data, 80, 112) > 204;
		bool ok16 = r16.data.size() == 48 && zeros(r16.data, 0, 16) == 128 && zeros(r16.data, 16, 48) > 204;
		bool okm = !(st & 0x80);
		bool ok = ok0 && ok16 && okm && a.data.size() == 512 && !(a.status & 0x1C);
		printf("turrican %d: c79 s1 %02x/%zu  s0 %s st %02x first %.1f ms  s16 %s st %02x first %.1f ms  motor %s after %d pulses  %s\n", it,
			a.status, a.data.size(), ok0 ? "ok" : "BAD", r0.status, ft0, ok16 ? "ok" : "BAD", r16.status, ft16,
			okm ? "off" : "STILL ON", pulses, ok ? "ok" : "FAIL");
		if (!ok0 || !ok16)
		{
			auto dump = [](const char *n, const std::vector<uint8_t> &v, const uint8_t *ref, size_t len) {
				printf("    %s us :", n); for (size_t i = 0; i < len && i < v.size(); i++) printf(" %02x", v[i]); printf("\n");
				printf("    %s STX:", n); for (size_t i = 0; i < len; i++) printf(" %02x", ref[i]); printf("\n");
			};
			if (!ok0) dump("s0 ", r0.data, s0->data, 112);
			if (!ok16) dump("s16", r16.data, s16->data, 48);
		}
		if (!ok) fails++;
	}
	return fails;
}

// ---- replay of a Hatari FDC trace (events.py output) ----------------------
// CPU side actions are replayed in order. Each one waits for the same thing
// the program waited for in Hatari (the last FDC side event before it: a DMA
// address step, the end of a command, the motor stop), then for the same
// delay after it. The ST DMA is modelled: it takes a byte at every DRQ while
// it is on and its sector count is not 0.
struct RDma { uint32_t ctrl = 0x80, addr = 0, fifo = 0, cnt = 0, secb = 0; bool on() const { return !(ctrl & 0x40) && !(ctrl & 0x100) && !(ctrl & 0x08) && cnt; } };
static RDma dma;
static std::vector<uint8_t> cap;     // bytes of the current type II command

static void rstep()
{
	if (top->drq && dma.on())
	{
		cap.push_back(rd(R_DATA));
		if (++dma.fifo == 16) { dma.fifo = 0; dma.addr += 16; }
		if (++dma.secb == 512) { dma.secb = 0; dma.cnt--; }
	}
	else tick();
}

static int replay(const char *evfile)
{
	FILE *f = fopen(evfile, "r");
	if (!f) { perror(evfile); return 1; }
	struct E { uint64_t t; char k[8]; long v, v2, line; };
	std::vector<E> ev;
	E e;
	while (fscanf(f, "%lu %7s %ld %ld %ld", &e.t, e.k, &e.v, &e.v2, &e.line) == 5)
	{
		ev.push_back(e);
	}
	fclose(f);
	// drop the duplicates (same time, same value, data register selected)
	{
		std::vector<E> o; uint32_t ctrl = 0x80;
		for (auto &x : ev)
		{
			if (!strcmp(x.k, "CTRL")) ctrl = x.v;
			if (!strcmp(x.k, "W8604") && !o.empty() && !strcmp(o.back().k, "W8604") && o.back().t == x.t && o.back().v == x.v && !(ctrl & 0x10) && ((ctrl >> 1) & 3) == 3) continue;
			o.push_back(x);
		}
		ev.swap(o);
	}
	auto *R = top->rootp->fdc1772;
	const double tpc = 4.0 / DISK_SCALE;          // sim ticks per Hatari 8 MHz cycle
	const uint64_t tmo = (uint64_t)(ms_to_disk(3000) * 1e6 / NS_PER_TICK);
	uint64_t anchor_T = ticks, anchor_t = 0;
	int diverg = 0, stat_bad = 0, data_bad = 0, cmds = 0;
	int cmd_cyl = 0, cmd_sec = 0, cmd_head = 0; bool in_t2 = false, in_t3 = false; uint8_t cmd_v = 0;
	auto finish_t2 = [&](long line) {
		if (!in_t2) return;
		in_t2 = false;
		const StxTrack *t = g_img->find(R->mfmt_drive__DOT__key_cyl, cmd_head);
		// expected: the sectors from cmd_sec on, as many as were transferred
		std::vector<uint8_t> exp;
		int s = cmd_sec;
		while (t && exp.size() < cap.size())
		{
			const StxSector *m = nullptr;
			for (auto &x : t->sectors) if (x.id[0] == cmd_cyl && x.id[2] == s && x.data) { m = &x; break; }
			if (!m) break;
			exp.insert(exp.end(), m->data, m->data + m->size);
			if (!(cmd_v & 0x10)) break;
			s++;
		}
		size_t n = std::min(exp.size(), cap.size()), bad = n;
		for (size_t i = 0; i < n; i++) if (cap[i] != exp[i]) { bad = i; break; }
		if (bad != n || exp.size() < cap.size())
		{
			data_bad++;
			printf("  line %ld: cmd %02x c%d h%d s%d: %zu bytes, first difference at %zu (expected %zu bytes)\n", line, cmd_v, cmd_cyl, cmd_head, cmd_sec, cap.size(), bad, exp.size());
		}
	};
	for (size_t i = 0; i < ev.size(); i++)
	{
		E &x = ev[i];
		bool anchor = !strcmp(x.k, "HDMA") || !strcmp(x.k, "HDONE") || !strcmp(x.k, "HMOFF");
		if (anchor)
		{
			uint64_t lim = ticks + tmo;
			bool ok = true;
			if (!strcmp(x.k, "HDMA"))
			{
				while (((uint32_t)(dma.addr - x.v) & 0xFFFFFF) > 0x8000 && ticks < lim) rstep();
				ok = ((uint32_t)(dma.addr - x.v) & 0xFFFFFF) <= 0x8000;
				if (!ok) { diverg++; printf("line %ld: Hatari DMA at %06lx, ours stays at %06x (cnt %u)\n", x.line, x.v, dma.addr, dma.cnt); dma.addr = x.v; }
			}
			else if (!strcmp(x.k, "HDONE"))
			{
				while (R->busy && ticks < lim) rstep();
				ok = !R->busy;
				if (!ok) { diverg++; printf("line %ld: Hatari command done, ours still busy\n", x.line); }
			}
			else
			{
				while (R->motor_on && ticks < lim) rstep();
				ok = !R->motor_on;
				if (!ok) { diverg++; printf("line %ld: Hatari motor off, ours still on\n", x.line); }
			}
			anchor_T = ticks; anchor_t = x.t;
			continue;
		}
		uint64_t target = anchor_T + (uint64_t)((x.t - anchor_t) * tpc);
		while (ticks < target) rstep();
		anchor_T = ticks; anchor_t = x.t;
		if (!strcmp(x.k, "SEL"))
		{
			top->floppy_side = x.v ? 0 : 1;
			top->floppy_drive = x.v2 == 0 ? 2 : x.v2 == 1 ? 1 : 3;
		}
		else if (!strcmp(x.k, "CTRL"))
		{
			if ((dma.ctrl ^ x.v) & 0x100) { dma.cnt = 0; dma.fifo = 0; dma.secb = 0; }
			dma.ctrl = x.v;
		}
		else if (!strcmp(x.k, "DMASET")) { dma.addr = x.v; dma.fifo = 0; }
		else if (!strcmp(x.k, "W8604"))
		{
			if (dma.ctrl & 0x10) { dma.cnt = x.v & 0xFF; dma.secb = 0; }
			else if (!(dma.ctrl & 0x08))
			{
				int reg = (dma.ctrl >> 1) & 3;
				uint8_t v = x.v & 0xFF;
				if (reg == 0)
				{
					if (in_t3) { printf("  line %ld: read track c%d: %zu bytes\n", x.line, cmd_cyl, cap.size()); in_t3 = false; }
					finish_t2(x.line);
					cmds++;
					if ((v & 0xF0) == 0xE0) { in_t3 = true; cap.clear(); cmd_cyl = R->track; }
					if ((v & 0xC0) == 0x80)
					{
						in_t2 = true; cmd_v = v; cap.clear();
						cmd_cyl = R->track; cmd_sec = R->sector; cmd_head = !top->floppy_side;
					}
					if (verbose) printf("line %ld: cmd %02x tr %d sec %d side %d\n", x.line, v, R->track, R->sector, !top->floppy_side);
				}
				wr(reg, v);
			}
		}
		else if (!strcmp(x.k, "RSTAT"))
		{
			uint8_t st = rd(R_CMD);
			// the index bit depends on where the disk is: not compared
			if ((st & ~0x02) != (x.v & ~0x02)) { stat_bad++; printf("line %ld: status ours %02x, Hatari %02lx\n", x.line, st, x.v); }
		}
	}
	finish_t2(0);
	printf("replay: %d commands, %d divergences, %d status differences, %d data differences\n", cmds, diverg, stat_bad, data_bad);
	return diverg + stat_bad + data_bad;
}

int main(int argc, char **argv)
{
	setvbuf(stdout, nullptr, _IOLBF, 0);
	Verilated::commandArgs(argc, argv);
	top = new Vfdc1772;
	int only_c = -1, only_h = -1, every = 1, fails = 0, stress_n = 0, turr_n = 0, rt_c = -1, flip_n = 0;
	const char *replay_f = nullptr;
	std::vector<const char *> files;
	for (int i = 1; i < argc; i++)
	{
		if (!strcmp(argv[i], "-t") && i + 2 < argc) { only_c = atoi(argv[++i]); only_h = atoi(argv[++i]); }
		else if (!strcmp(argv[i], "-n") && i + 1 < argc) every = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-stress") && i + 1 < argc) stress_n = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-rt") && i + 1 < argc) rt_c = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-flip") && i + 1 < argc) flip_n = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-replay") && i + 1 < argc) replay_f = argv[++i];
		else if (!strcmp(argv[i], "-turrican") && i + 1 < argc) turr_n = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-lat") && i + 1 < argc) sd_latency_max = (uint32_t)(atof(argv[++i]) * 1e6 / NS_PER_TICK / DISK_SCALE);   // real ms
		else if (!strcmp(argv[i], "-v")) verbose = 1;
		else if (!strcmp(argv[i], "-st")) { fails += check_st(); }
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

		// power on, mount the image on drive A, select it
		top->floppy_reset = 0; top->cpu_sel = 0; top->cpu_rw = 1;
		top->floppy_drive = 2;     // drive A (active low)
		top->floppy_side = 1;
		top->img_wp = 0; top->img_mfmt = 0; top->img_size = 0; top->img_mounted = 0;
		ticks_n(100);
		top->floppy_reset = 1;
		ticks_n(100);
		top->img_size = (uint32_t)buf.size(); top->img_mfmt = 1; top->img_mounted = 1;
		ticks_n(4);
		top->img_mounted = 0;
		ticks_n(100);

		{
			// write protect sensor after the mount: protected (the old disk
			// going out), then the image (STX shows as not protected)
			wr(R_CMD, 0x08);   // restore: type I status, live while it runs
			ticks_n(200);
			std::string seq;
			int prev = -1;
			for (int ms = 0; ms < 1000; ms += 5)
			{
				int wp = (rd(R_CMD) >> 6) & 1;
				if (wp != prev) seq += wp ? '1' : '0';
				prev = wp;
				ticks_n((uint64_t)(ms_to_disk(5) * 1e6 / NS_PER_TICK));
			}
			bool ok = seq == "10";
			printf("%s: write protect after mount %s %s\n", f, seq.c_str(), ok ? "ok" : "FAIL");
			if (!ok) fails++;
		}
		int cur = -1;
		Cmd r = run(0x08, 3000);   // restore, no spin-up wait
		if (r.timeout) { printf("%s: restore timeout\n", f); return 1; }

		cur = 0;
		ticks_n((int)(30e6 / NS_PER_TICK));   // track loaded
		if (!check_index_bit("MFMT type I status", 200.0 / DISK_SCALE)) fails++;

		if (flip_n)
		{
			// commands while drive B is selected for 130 us at random moments,
			// every 0-40 ms (TOS does it every 16 VBLs): no command may hang
			// or end early
			int bad = 0;
			auto cmd_flip = [&](uint8_t c, size_t &bytes) {
				wr(R_CMD, c);
				uint64_t t0 = ticks, lim = (uint64_t)(ms_to_disk(3000) * 1e6 / NS_PER_TICK);
				uint64_t next = ticks + rnd() % (uint64_t)(ms_to_disk(40) * 1e6 / NS_PER_TICK);
				bytes = 0;
				while (!top->irq && ticks - t0 < lim)
				{
					if (ticks >= next && !getenv("NOFLIP"))
					{
						top->floppy_drive = 1;
						uint64_t te = ticks + (uint64_t)(0.13e6 / NS_PER_TICK);
						while (ticks < te) { if (top->drq) { rd(R_DATA); bytes++; } else tick(); }
						top->floppy_drive = 2;
						next = ticks + rnd() % (uint64_t)(ms_to_disk(40) * 1e6 / NS_PER_TICK);
					}
					if (top->drq) { rd(R_DATA); bytes++; } else tick();
				}
				bool hang = !top->irq;
				uint8_t st = rd(R_CMD);
				return std::make_pair(hang, st);
			};
			for (int it = 0; it < flip_n; it++)
			{
				int c = 70 + rnd() % 10;
				size_t b;
				wr(R_DATA, (uint8_t)c);
				auto a = cmd_flip(0x15, b);                       // seek with verify
				auto r = cmd_flip(0xE4, b);                       // read track
				wr(R_SECTOR, 1);
				size_t b2;
				auto q = cmd_flip(0x80, b2);                      // read sector 1
				bool ok = !a.first && !r.first && !q.first && !(a.second & 0x10) && !(r.second & 0x10) && !(q.second & 0x1C) && b > 5000 && b2 == 512;
				if (!ok) { bad++; printf("flip %d: c%d seek %s %02x, read track %s %02x %zu bytes, sector %s %02x %zu\n", it, c,
					a.first ? "HANG" : "ok", a.second, r.first ? "HANG" : "ok", r.second, b, q.first ? "HANG" : "ok", q.second, b2); }
			}
			printf("%s: flip test %d rounds, %d bad\n", f, flip_n, bad);
			fails += bad;
			continue;
		}
		if (rt_c >= 0)
		{
			for (int k = 0; k < 3; k++)
			{
				wr(R_DATA, (uint8_t)(rt_c - k)); Cmd sk = run(0x15, 4000);
				// like TOS's VBL routine: drive B selected for a moment while the read track runs
				wr(R_CMD, 0xE4);
				Cmd t;
				uint64_t t0 = ticks, sel_at = ticks + (uint64_t)(ms_to_disk(80) * 1e6 / NS_PER_TICK);
				bool flipped = false;
				while (!top->irq && ticks - t0 < (uint64_t)(ms_to_disk(4000) * 1e6 / NS_PER_TICK))
				{
					if (!flipped && ticks >= sel_at)
					{
						flipped = true;
						top->floppy_drive = 1; ticks_n((uint64_t)(0.13e6 / NS_PER_TICK)); top->floppy_drive = 2;
					}
					if (top->drq) t.data.push_back(rd(R_DATA)); else tick();
				}
				t.status = rd(R_CMD);
				printf("seek %d status %02x, read track %zu bytes, status %02x (drive B selected for 130 us during it)\n", rt_c - k, sk.status, t.data.size(), t.status);
				if (getenv("DBGQ")) dbg_drain();
			}
			continue;
		}
		if (replay_f)
		{
			fails += replay(replay_f);
			continue;
		}
		if (turr_n)
		{
			int tf = turrican(turr_n, cur);
			printf("%s: turrican check %d runs, %d failures\n", f, turr_n, tf);
			fails += tf;
			continue;
		}
		if (stress_n)
		{
			int sf = stress(stress_n, cur);
			printf("%s: stress %d reads, %d failures\n", f, stress_n, sf);
			fails += sf;
			continue;
		}

		int ff = 0, ft = 0;
		for (int c = 0; c < img.cylinders(); c++)
			for (int h = 0; h < img.heads(); h++)
			{
				if (only_c >= 0 && (c != only_c || h != only_h)) continue;
				if (only_c < 0 && (c % every) && !(c >= 78)) continue;   // always test the last tracks
				ff += check_track(c, h, cur);
				ft++;
			}
		printf("%s: %d/%d tracks ok (clkcpu %d kHz, disk x%.0f)\n", f, ft - ff, ft, TB_CLKCPU_KHZ, DISK_SCALE);
		fails += ff;
	}
	printf("%s\n", fails ? "SOME TRACKS FAILED" : "all tracks ok");
	delete top;
	return fails ? 1 : 0;
}
