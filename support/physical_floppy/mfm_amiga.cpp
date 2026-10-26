/*
 * AmigaDOS MFM: flux <-> sectors.
 *
 * Sector: 0x0000 (2 bytes), sync 0x4489 0x4489, then MFM longs, each with its
 * data bits at the 0x55555555 positions and split odd/even:
 *   info (format 0xFF, track, sector, sectors to the gap): odd, even
 *   label (16 bytes): 4 odd, 4 even
 *   header checksum: odd, even (xor of the info and label longs)
 *   data checksum: odd, even (xor of the data longs)
 *   data (512 bytes): 128 odd, 128 even
 */

#include <string.h>
#include "mfm_amiga.h"
#include "mfm_ibm.h"

#define CELL_S 2e-6

/* ------------------------------------------------------------------ decode */

static inline uint32_t get32(const std::vector<uint8_t> &bits, size_t p)
{
	uint32_t v = 0;
	for (int i = 0; i < 32; i++) v = (v << 1) | bits[p + i];
	return v;
}

void mfm_amiga_decode(const std::vector<uint32_t> &flux, double sample_freq, std::vector<amiga_sector> &out)
{
	std::vector<uint8_t> bits;
	mfm_flux_to_bits(flux, sample_freq, CELL_S, bits);
	out.clear();

	const size_t need = 32 * (2 + 8 + 2 + 2 + 256);
	uint32_t sr = 0;
	for (size_t i = 0; i + need < bits.size(); i++)
	{
		sr = (sr << 1) | bits[i];
		if (sr != 0x44894489) continue;

		size_t p = i + 1;
		// a third sync word is possible: skip it
		if ((get32(bits, p) >> 16) == 0x4489) p += 16;
		if (p + need > bits.size()) break;

		const uint32_t M = 0x55555555;
		uint32_t io = get32(bits, p) & M, ie = get32(bits, p + 32) & M;
		uint32_t info = (io << 1) | ie;

		uint32_t hsum = 0;
		for (int k = 0; k < 10; k++) hsum ^= get32(bits, p + 32 * k);
		hsum &= M;
		uint32_t hs = ((get32(bits, p + 320) & M) << 1) | (get32(bits, p + 352) & M);
		uint32_t ds = ((get32(bits, p + 384) & M) << 1) | (get32(bits, p + 416) & M);

		amiga_sector s;
		s.track = (info >> 16) & 0xFF;
		s.sec = (info >> 8) & 0xFF;
		uint32_t dsum = 0;
		for (int k = 0; k < 128; k++)
		{
			uint32_t o = get32(bits, p + 448 + 32 * k), e = get32(bits, p + 448 + 32 * (k + 128));
			dsum ^= o ^ e;
			uint32_t v = ((o & M) << 1) | (e & M);
			s.data[k * 4] = v >> 24;
			s.data[k * 4 + 1] = v >> 16;
			s.data[k * 4 + 2] = v >> 8;
			s.data[k * 4 + 3] = v;
		}
		dsum &= M;
		s.ok = (info >> 24) == 0xFF && hs == hsum && ds == dsum && s.sec < AMIGA_SPT;
		out.push_back(s);

		if (s.ok)
		{
			i = p + need - 1;
			sr = 0;
		}
	}
}

/* ------------------------------------------------------------------ encode */

struct mfm_writer
{
	std::vector<uint8_t> b;
	int prev = 0;
	void raw16(uint16_t w)
	{
		for (int i = 15; i >= 0; i--) b.push_back((w >> i) & 1);
		prev = w & 1;
	}
	// data bits at the 0x55555555 positions, clock bits added
	void mfm_long(uint32_t v)
	{
		v &= 0x55555555;
		for (int i = 31; i >= 0; i--)
		{
			int bit;
			if ((0x55555555u >> i) & 1) bit = (v >> i) & 1;
			else bit = (!prev && !((v >> (i - 1)) & 1)) ? 1 : 0;
			b.push_back(bit);
			prev = bit;
		}
	}
};

static void put_sector(mfm_writer &w, int track, int sec, const uint8_t *data)
{
	const uint32_t M = 0x55555555;
	w.mfm_long(0);
	w.raw16(0x4489);
	w.raw16(0x4489);

	uint32_t info = (0xFFu << 24) | ((uint32_t)track << 16) | ((uint32_t)sec << 8) | (uint32_t)(AMIGA_SPT - sec);
	uint32_t hdr[10] = { (info >> 1) & M, info & M };
	for (int i = 2; i < 10; i++) hdr[i] = 0;
	uint32_t hsum = 0;
	for (int i = 0; i < 10; i++) { w.mfm_long(hdr[i]); hsum ^= hdr[i]; }

	uint32_t dl[256];
	for (int i = 0; i < 128; i++)
	{
		uint32_t v = ((uint32_t)data[i * 4] << 24) | (data[i * 4 + 1] << 16) | (data[i * 4 + 2] << 8) | data[i * 4 + 3];
		dl[i] = (v >> 1) & M;
		dl[i + 128] = v & M;
	}
	uint32_t dsum = 0;
	for (int i = 0; i < 256; i++) dsum ^= dl[i];
	hsum &= M;
	dsum &= M;
	w.mfm_long(hsum >> 1);
	w.mfm_long(hsum);
	w.mfm_long(dsum >> 1);
	w.mfm_long(dsum);
	for (int i = 0; i < 256; i++) w.mfm_long(dl[i]);
}

void mfm_amiga_encode_track(int track, const uint8_t *data, double sample_freq, double rev_ns, std::vector<uint32_t> &flux)
{
	mfm_writer w;
	// a short gap after the index pulse, the sectors, then gap up to the next pulse
	for (int i = 0; i < 4; i++) w.mfm_long(0);
	for (int s = 0; s < AMIGA_SPT; s++) put_sector(w, track, s, data + s * 512);
	size_t total = (size_t)(rev_ns / 2000.0);
	while (w.b.size() + 32 <= total) w.mfm_long(0);

	flux.clear();
	flux.reserve(w.b.size() / 2);
	double t = 0, last = 0;
	const double cell = 2e-6 * sample_freq;
	for (size_t i = 0; i < w.b.size(); i++)
	{
		t += cell;
		if (!w.b[i]) continue;
		double v = (double)(int64_t)(t - last + 0.5);
		flux.push_back((uint32_t)v);
		last += v;
	}
}
