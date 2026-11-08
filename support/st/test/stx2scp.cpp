// stx2scp - write an STX image as an SCP flux image (test tool)
//
// Copyright (C) 2026 Manfredi
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Rebuilds every track with the STX converter and writes its cells as flux
// (one transition per 1 cell, the cell lengths of the converter) in several
// revolutions, the weak cells drawn again for each one. Used to test the
// core's flux path against the STX (MFMT) path on the same disks.
//
// usage: stx2scp [-r revs] [-s seed] in.stx out.scp

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <random>
#include "st_stx.h"
#include "st_trackbuild.h"

static const double TICK_NS = 25.0;   // SCP resolution 0: 25 ns

static void put32(std::vector<uint8_t> &v, size_t at, uint32_t x)
{
	v[at] = x; v[at + 1] = x >> 8; v[at + 2] = x >> 16; v[at + 3] = x >> 24;
}

// one revolution of flux in 25 ns ticks; carry = time since the last
// transition, kept across revolutions (a real capture is continuous)
static void rev_flux(const TrackBuilder::Cells &c, bool formatted, std::mt19937 &rng,
                     double &carry, std::vector<uint32_t> &out, double &rev_ns)
{
	out.clear();
	rev_ns = 0;
	if (!formatted)
	{
		// unformatted: random flux, 2..10 us apart, for 200 ms
		std::uniform_real_distribution<double> d(2000.0, 10000.0);
		while (rev_ns < 200e6)
		{
			double t = d(rng);
			rev_ns += t;
			carry += t;
			uint32_t tk = (uint32_t)(carry / TICK_NS + 0.5);
			out.push_back(tk);
			carry -= tk * TICK_NS;   // keep the rounding: the turn stays as long as the index time
		}
		return;
	}
	for (size_t i = 0; i < c.bit.size(); i++)
	{
		carry += c.ns[i];
		rev_ns += c.ns[i];
		int b = c.weak[i] ? (int)(rng() & 1) : c.bit[i];
		if (b)
		{
			uint32_t tk = (uint32_t)(carry / TICK_NS + 0.5);
			out.push_back(tk);
			carry -= tk * TICK_NS;
		}
	}
}

int main(int argc, char **argv)
{
	int revs = 3;
	unsigned seed = 1;
	int a = 1;
	for (; a < argc && argv[a][0] == '-'; a++)
	{
		if (!strcmp(argv[a], "-r") && a + 1 < argc) revs = atoi(argv[++a]);
		else if (!strcmp(argv[a], "-s") && a + 1 < argc) seed = atoi(argv[++a]);
	}
	if (argc - a != 2 || revs < 1 || revs > 5)
	{
		fprintf(stderr, "usage: stx2scp [-r revs 1..5] [-s seed] in.stx out.scp\n");
		return 2;
	}

	FILE *f = fopen(argv[a], "rb");
	if (!f) { perror(argv[a]); return 2; }
	std::vector<uint8_t> buf;
	uint8_t tmp[65536];
	size_t n;
	while ((n = fread(tmp, 1, sizeof(tmp), f)) > 0) buf.insert(buf.end(), tmp, tmp + n);
	fclose(f);

	StxImage img;
	if (!img.parse(buf.data(), buf.size())) { fprintf(stderr, "%s: %s\n", argv[a], img.error()); return 1; }

	int cyls = img.cylinders();
	if (cyls < 80) cyls = 80;
	if (cyls > 84) cyls = 84;
	int heads = img.heads() > 1 ? 2 : 1;

	std::vector<uint8_t> out(0x10 + 168 * 4, 0);
	memcpy(out.data(), "SCP", 3);
	out[3] = 0x19;                       // version 1.9
	out[4] = 0x14;                       // disk type: Atari ST (other)
	out[5] = revs;
	out[6] = 0;                          // first track
	out[7] = (cyls - 1) * 2 + 1;         // last track
	out[8] = 0x01;                       // flags: index aligned
	out[9] = 0;                          // 16 bit flux
	out[10] = heads == 2 ? 0 : 1;        // heads: 0 both, 1 side 0 only
	out[11] = 0;                         // 25 ns

	std::mt19937 rng(seed);
	for (int cyl = 0; cyl < cyls; cyl++)
		for (int head = 0; head < heads; head++)
		{
			TrackBuilder tb;
			bool present = stx_build_track(img, cyl, head, tb);
			TrackBuilder::Cells c;
			bool formatted = present && tb.size() > 0;
			if (formatted) tb.build_cells(c);
			if (formatted && c.bit.empty()) formatted = false;

			int tn = cyl * 2 + head;
			size_t th = out.size();
			put32(out, 0x10 + tn * 4, (uint32_t)th);
			out.resize(th + 4 + revs * 12, 0);
			out[th] = 'T'; out[th + 1] = 'R'; out[th + 2] = 'K'; out[th + 3] = tn;

			double carry = 0;
			for (int r = 0; r < revs; r++)
			{
				std::vector<uint32_t> flux;
				double rev_ns;
				rev_flux(c, formatted, rng, carry, flux, rev_ns);
				std::vector<uint8_t> data;
				for (uint32_t t : flux)
				{
					while (t >= 65536) { data.push_back(0); data.push_back(0); t -= 65536; }
					if (!t) t = 1;
					data.push_back(t >> 8); data.push_back(t & 0xFF);
				}
				size_t e = th + 4 + r * 12;
				put32(out, e, (uint32_t)(rev_ns / TICK_NS + 0.5));
				put32(out, e + 4, (uint32_t)(data.size() / 2));
				put32(out, e + 8, (uint32_t)(out.size() - th));
				out.insert(out.end(), data.begin(), data.end());
			}
		}

	uint32_t sum = 0;
	for (size_t i = 0x10; i < out.size(); i++) sum += out[i];
	put32(out, 12, sum);

	f = fopen(argv[a + 1], "wb");
	if (!f) { perror(argv[a + 1]); return 2; }
	fwrite(out.data(), 1, out.size(), f);
	fclose(f);
	printf("%s: %d cylinders, %d head(s), %d revolutions, %zu bytes\n", argv[a + 1], cyls, heads, revs, out.size());
	return 0;
}
