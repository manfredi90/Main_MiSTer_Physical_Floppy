// stxtool.cpp - convert STX to MFMT tracks and check them with a WD1772 model
//
// Usage:
//   stxtool [-v] file.stx              check every track
//   stxtool -t CYL HEAD [-v] file.stx  check one track
//   stxtool -o DIR file.stx            also write DIR/cCC_hH.meta / .cells
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <vector>
#include <string>
#include "st_stx.h"
#include "wdsim.h"

static int verbose = 0;

static bool load_file(const char *path, std::vector<uint8_t> &buf)
{
	FILE *f = fopen(path, "rb");
	if (!f) return false;
	fseek(f, 0, SEEK_END);
	long n = ftell(f);
	fseek(f, 0, SEEK_SET);
	buf.resize(n);
	bool ok = fread(buf.data(), 1, n, f) == (size_t)n;
	fclose(f);
	return ok;
}

struct Result
{
	int checks = 0, fails = 0;
	std::vector<std::string> msgs;
	void fail(const char *fmt, ...) __attribute__((format(printf, 2, 3)))
	{
		char b[256];
		va_list ap;
		va_start(ap, fmt);
		vsnprintf(b, sizeof(b), fmt, ap);
		va_end(ap);
		msgs.push_back(b);
		fails++;
	}
};

// expected time of the data field transfer according to the STX
static double expected_data_ns(const StxSector &s)
{
	if (s.timing)
	{
		double t = 0;
		for (int b = 0; b < s.size / 16; b++)
		{
			uint16_t v = (uint16_t)((s.timing[b * 2] << 8) | s.timing[b * 2 + 1]);
			t += (v * 32.0 + 28.0) * 125.0;
		}
		return t;
	}
	// the converter ignores read times far from the standard rate
	if (s.read_time && s.read_time * 1000.0 / s.size > 32000.0 * 0.85 && s.read_time * 1000.0 / s.size < 32000.0 * 1.15) return s.read_time * 1000.0;
	return 0;   // standard: no check
}

static int check_track(const StxImage &img, int cyl, int head, const char *outdir, int &total_fail)
{
	TrackBuilder tb;
	StxBuildReport rep;
	stx_build_track(img, cyl, head, tb, &rep);
	static uint8_t meta[ST_MFMT_META_BYTES], cells[ST_MFMT_CELLS_BYTES];
	tb.encode(cyl, head, meta, cells);

	if (outdir)
	{
		char p[512];
		snprintf(p, sizeof(p), "%s/c%02d_h%d.meta", outdir, cyl, head);
		FILE *f = fopen(p, "wb"); if (f) { fwrite(meta, 1, sizeof(meta), f); fclose(f); }
		snprintf(p, sizeof(p), "%s/c%02d_h%d.cells", outdir, cyl, head);
		f = fopen(p, "wb"); if (f) { fwrite(cells, 1, sizeof(cells), f); fclose(f); }
	}

	WdSim sim;
	std::string err;
	Result r;
	if (!sim.load(meta, cells, err)) { printf("c%02d h%d: MFMT invalid: %s\n", cyl, head, err.c_str()); total_fail++; return 1; }
	sim.spin(3, 1234 + cyl * 2 + head);
	const st_mfmt_header &h = sim.header();
	const StxTrack *t = img.find(cyl, head);
	size_t per = sim.cells_per_rev();

	char line[256];
	snprintf(line, sizeof(line), "c%02d h%d: %5u bytes %6.2f ms timing=%u weak=%u/%uw%s%s",
		cyl, head, h.cell_count / 16, sim.rev_ns() / 1e6, h.timing_count, h.weak_count, h.pool_words,
		(h.flags & ST_MFMT_F_FORMATTED) ? "" : " UNFORMATTED", (h.flags & ST_MFMT_F_TRUNCATED) ? " TRUNCATED" : "");

	if (!(h.flags & ST_MFMT_F_FORMATTED))
	{
		auto ids = sim.read_addresses(0, per * 2);
		r.checks++;
		if (!ids.empty()) r.fail("unformatted track returned %zu IDs", ids.size());
	}
	else
	{
		if (fabs(sim.rev_ns() - 200e6) > 200e6 * 0.002) r.fail("revolution %.3f ms", sim.rev_ns() / 1e6);
		auto ids = sim.read_addresses(0, per);
		auto ids2 = sim.read_addresses(per, per * 2);
		int ns = t ? (int)t->sectors.size() : 0;
		std::vector<int> used(ids.size(), 0);
		double prev_t = -1, prev_bp = -1, max_dev = 0;

		for (int s = 0; s < ns; s++)
		{
			const StxSector &sec = t->sectors[s];
			int expect = tb.wrap((int)floor(stx_bitpos_bytes(*t, sec.bit_position) + 0.5) - 4 + rep.image_offset);
			int best = -1;
			for (size_t i = 0; i < ids.size(); i++)
			{
				if (used[i]) continue;
				int pos = (int)(ids[i].cell / 16);
				int d = abs(pos - expect);
				if (d > tb.size() / 2) d = tb.size() - d;
				if (d <= 48 && !memcmp(ids[i].id, sec.id, 4) && (best < 0 || d < abs((int)(ids[best].cell / 16) - expect))) best = (int)i;
			}
			r.checks++;
			if (best < 0) { r.fail("sector %d [%02x %02x %02x %02x]: ID not found", s, sec.id[0], sec.id[1], sec.id[2], sec.id[3]); continue; }
			used[best] = 1;
			const IdField &id = ids[best];

			bool id_err = (sec.status & STX_SECT_RNF) && (sec.status & STX_SECT_CRC_ERROR);
			r.checks++;
			if (id.crc_ok == id_err) r.fail("sector %d: ID CRC %s, STX says %s", s, id.crc_ok ? "ok" : "bad", id_err ? "bad" : "ok");
			if (!id_err && memcmp(id.id + 4, sec.id + 4, 2))
			{
				// STX may store a CRC that disagrees with a good ID; we always write the good one
				if (verbose) printf("  note: sector %d ID CRC rewritten %02x%02x -> %02x%02x\n", s, sec.id[4], sec.id[5], id.id[4], id.id[5]);
			}

			// inter-sector timing as seen by a program that measures it
			// (bit_position: see stx_bitpos_ns)
			if (prev_t >= 0 && (!t->image || rep.image_ignored))
			{
				double dt = id.t - prev_t;
				double dexp = stx_bitpos_ns(*t, sec.bit_position) - stx_bitpos_ns(*t, prev_bp);
				if (dexp > 0)
				{
					double dev = fabs(dt - dexp) / 1000.0;
					if (dev > max_dev) max_dev = dev;
				}
			}
			prev_t = id.t; prev_bp = sec.bit_position;

			if (sec.status & STX_SECT_RNF)
			{
				if (!id_err)
				{
					DataField d = sim.read_data(id);
					r.checks++;
					if (d.found) r.fail("sector %d: RNF in STX but a data field was found", s);
				}
				continue;
			}

			DataField d = sim.read_data(id);
			r.checks++;
			if (!d.found) { r.fail("sector %d: data field not found", s); continue; }
			r.checks++;
			bool deleted = d.dam == 0xF8;
			if (deleted != !!(sec.status & STX_SECT_DELETED)) r.fail("sector %d: DAM %02x", s, d.dam);
			// random bits cannot keep a valid CRC on a real bit stream (nor on the original disk)
			r.checks++;
			if (sec.fuzzy)
			{
				if (!(sec.status & STX_SECT_CRC_ERROR)) printf("  note: sector %d is fuzzy but the STX says CRC ok\n", s);
			}
			else if (d.crc_ok == !!(sec.status & STX_SECT_CRC_ERROR)) r.fail("sector %d: data CRC %s, STX says %s", s, d.crc_ok ? "ok" : "bad", (sec.status & STX_SECT_CRC_ERROR) ? "bad" : "ok");

			int diff = 0;
			for (int i = 0; i < sec.size; i++)
			{
				uint8_t m = sec.fuzzy ? sec.fuzzy[i] : 0xFF;
				if ((d.data[i] & m) != (sec.data[i] & m)) diff++;
			}
			r.checks++;
			if (diff) r.fail("sector %d: %d data bytes differ", s, diff);
			if (verbose > 2)
			{
				printf("  sector %d (id %02x) reads:", s, sec.id[2]);
				for (int k = 0; k < sec.size; k++) printf("%s%02x", k % 32 ? " " : "\n    ", d.data[k]);
				printf("\n");
			}
			if (diff && verbose > 1)
			{
				int k0 = 0;
				while (k0 < sec.size && d.data[k0] == sec.data[k0]) k0++;
				int from = k0 > 8 ? k0 - 8 : 0;
				printf("  sector %d first difference at byte %d\n    STX: ", s, k0);
				for (int k = from; k < from + 48 && k < sec.size; k++) printf("%02x ", sec.data[k]);
				printf("\n    us : ");
				for (int k = from; k < from + 48 && k < sec.size; k++) printf("%02x ", d.data[k]);
				printf("\n");
			}

			if (sec.fuzzy)
			{
				// same sector on the next revolution must differ in the fuzzy bits
				const IdField *id2 = nullptr;
				for (auto &x : ids2) if (x.cell == id.cell + per) id2 = &x;
				r.checks++;
				if (!id2) r.fail("sector %d: not found on 2nd revolution", s);
				else
				{
					DataField e = sim.read_data(*id2);
					int changed = 0, wrong_stable = 0;
					for (int i = 0; i < sec.size; i++)
					{
						if ((d.data[i] ^ e.data[i]) & ~sec.fuzzy[i]) changed++;
						if ((d.data[i] ^ e.data[i]) & sec.fuzzy[i]) wrong_stable++;
					}
					// a fuzzy mask can be all stable (Xenon 2 track 0 sector 6)
					int nfuzzy = 0;
					for (int i = 0; i < sec.size; i++) if (sec.fuzzy[i] != 0xFF) nfuzzy++;
					if (!changed && nfuzzy) r.fail("sector %d: fuzzy bits did not change between reads", s);
					if (wrong_stable) r.fail("sector %d: %d stable bytes changed between reads", s, wrong_stable);
					if (verbose) printf("  sector %d fuzzy: %d bytes changed between two reads\n", s, changed);
				}
			}

			double exp = expected_data_ns(sec);
			if (exp > 0)
			{
				r.checks++;
				double err_pct = (d.data_ns - exp) / exp * 100.0;
				if (fabs(err_pct) > 0.2) r.fail("sector %d: data read in %.1f us, STX %.1f us (%.2f %%)", s, d.data_ns / 1000, exp / 1000, err_pct);
				if (verbose) printf("  sector %d timing: %.1f us (STX %.1f us)\n", s, d.data_ns / 1000, exp / 1000);
			}
		}

		// extra IDs that are not in the STX
		int extra = 0;
		for (size_t i = 0; i < ids.size(); i++) if (!used[i]) extra++;
		r.checks++;
		if (extra && t && t->sectors.empty()) printf("  note: %d ID fields from the track image, the STX lists no sector\n", extra);
		else if (extra) r.fail("%d ID fields not described by the STX", extra);

		if (max_dev > 64) { r.checks++; r.fail("inter-sector timing deviates by %.0f us", max_dev); }

		if (t && t->image && rep.image_ignored) snprintf(line + strlen(line), sizeof(line) - strlen(line), " image-ignored");
		if (t && t->image && !rep.image_ignored)
		{
			auto rt = sim.read_track(0);
			int n = t->image_size, best = 0, best_off = 0;
			// greedy alignment: the WD1772 output drifts by a byte here and
			// there after a resync, so follow local offsets; the garbled first
			// byte of a sync sequence ("14 A1 A1", "C2 A1 A1"...) counts as equal
			{
				const uint8_t *im = t->image;
				int m = (int)rt.size();
				size_t i = 0, j = 0;
				auto syncpre = [](const uint8_t *p, size_t k, size_t len) { return k + 2 < len && p[k + 1] == 0xA1 && p[k + 2] == 0xA1; };
				while ((int)i < n && (int)j < m)
				{
					if (im[i] == rt[j] || (syncpre(im, i, n) && syncpre(rt.data(), j, m))) { best++; i++; j++; continue; }
					int k, found = 0;
					for (k = 1; k <= 4 && !found; k++)
					{
						if ((int)(i + k + 4) < n && (int)(j + 4) < m && !memcmp(im + i + k, rt.data() + j, 4)) { i += k; found = 1; }
						else if ((int)(j + k + 4) < m && (int)(i + 4) < n && !memcmp(rt.data() + j + k, im + i, 4)) { j += k; found = 1; }
					}
					if (!found) { i++; j++; }
				}
			}
			double pct = 100.0 * best / n;
			if (verbose > 1)
			{
				printf("  image: size %d sync %d, headers found %d/%d, data found %d, offset %d\n", n, t->image_sync,
					rep.headers_found_in_image, rep.sectors, rep.data_found_in_image, rep.image_offset);
				for (int s2 = 0; s2 < ns; s2++) printf("   sec %d bitpos %d -> byte %d status %02x id %02x %02x %02x %02x\n", s2,
					t->sectors[s2].bit_position, (int)stx_bitpos_bytes(*t, t->sectors[s2].bit_position) - 4, t->sectors[s2].status,
					t->sectors[s2].id[0], t->sectors[s2].id[1], t->sectors[s2].id[2], t->sectors[s2].id[3]);
				int run = -1;
				for (int i = 0; i <= n && i <= (int)rt.size(); i++)
				{
					bool eq = i < n && i < (int)rt.size() && rt[i] == t->image[((i + best_off) % n + n) % n];
					if (!eq && run < 0) run = i;
					if (eq && run >= 0) { if (i - run > 2) { printf("   mismatch %d..%d\n     sim:", run, i - 1); for (int k = run - 2; k < i + 1 && k < run + 14; k++) printf(" %02x", rt[k]); printf("\n     img:"); for (int k = run - 2; k < i + 1 && k < run + 14; k++) printf(" %02x", t->image[(k + n) % n]); printf("\n"); } run = -1; }
				}
			}
			snprintf(line + strlen(line), sizeof(line) - strlen(line), " readtrack=%.1f%%", pct);
			// the image of a converted STX comes from the converter's own WD
			// model, so only a gross difference is an error
			r.checks++;
			if (pct < 50.0) r.fail("read track matches the STX image only %.1f %%", pct);
			else if (pct < 75.0) printf("  note: read track matches the STX image %.1f %%\n", pct);
		}
		snprintf(line + strlen(line), sizeof(line) - strlen(line), " sectors=%d/%zu", ns, ids.size());
		if (rep.overlaps) snprintf(line + strlen(line), sizeof(line) - strlen(line), " nested=%d", rep.overlaps);
		if (rep.splice_cells) snprintf(line + strlen(line), sizeof(line) - strlen(line), " splice=+%d", rep.splice_cells);
		if (rep.jumps) snprintf(line + strlen(line), sizeof(line) - strlen(line), " jumps=%d", rep.jumps);
		if (rep.forced_cells) snprintf(line + strlen(line), sizeof(line) - strlen(line), " forced=%d", rep.forced_cells);
		// nested fields inside a fuzzy sector are read one bit off by it (HLS): expected
		bool any_fuzzy = false;
		if (t) for (auto &x : t->sectors) if (x.fuzzy) any_fuzzy = true;
		if (rep.overlap_mismatch && any_fuzzy) printf("  note: %d nested bytes differ inside a fuzzy sector\n", rep.overlap_mismatch);
		else if (rep.overlap_mismatch && r.fails) r.fail("%d nested bytes disagree with the outer sector", rep.overlap_mismatch);
		else if (rep.overlap_mismatch) printf("  note: %d nested bytes differ, every sector still reads as recorded\n", rep.overlap_mismatch);
		if (max_dev > 0 && verbose) printf("  max inter-sector timing deviation %.1f us\n", max_dev);
	}

	printf("%s %s\n", line, r.fails ? "FAIL" : "ok");
	for (auto &m : r.msgs) printf("    %s\n", m.c_str());
	total_fail += r.fails;
	return r.fails;
}

int main(int argc, char **argv)
{
	int only_c = -1, only_h = -1;
	const char *outdir = nullptr, *path = nullptr;
	for (int i = 1; i < argc; i++)
	{
		if (!strcmp(argv[i], "-v")) verbose++;
		else if (!strcmp(argv[i], "-t") && i + 2 < argc) { only_c = atoi(argv[++i]); only_h = atoi(argv[++i]); }
		else if (!strcmp(argv[i], "-o") && i + 1 < argc) outdir = argv[++i];
		else path = argv[i];
	}
	if (!path) { fprintf(stderr, "usage: stxtool [-v] [-t cyl head] [-o dir] file.stx\n"); return 2; }

	std::vector<uint8_t> buf;
	if (!load_file(path, buf)) { perror(path); return 2; }
	StxImage img;
	if (!img.parse(buf.data(), buf.size())) { printf("%s: %s\n", path, img.error()); return 2; }
	printf("%s: revision %d, %zu tracks, %d cylinders, %d heads\n", path, img.revision(), img.tracks().size(), img.cylinders(), img.heads());

	int fails = 0;
	for (int c = 0; c < img.cylinders(); c++)
		for (int h = 0; h < img.heads(); h++)
		{
			if (only_c >= 0 && (c != only_c || h != only_h)) continue;
			check_track(img, c, h, outdir, fails);
		}
	printf("%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
	return fails ? 1 : 0;
}
