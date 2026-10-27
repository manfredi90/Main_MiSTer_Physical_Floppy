/*
 * Macintosh 400K/800K GCR: flux -> sectors (reading only).
 *
 * Track: self-sync bytes, then per sector
 *   address field: D5 AA 96, cylinder (low 6 bits), sector, side (side << 5 |
 *                  cylinder >> 6), format (0x02 single sided, 0x22 double
 *                  sided), checksum (xor), DE AA
 *   data field:    D5 AA AD, sector, 703 bytes (524 bytes + checksum), DE AA
 * Every byte on the disk is a 6-and-2 GCR code (a "1" bit is a flux transition).
 *
 * decode_sector(): from the Greaseweazle host tools (public domain), from
 * FluxEngine, inspired by MAME (see gcr_mac.h).
 */

#include <string.h>
#include "gcr_mac.h"
#include "mfm_ibm.h"

static const uint8_t gcr6[64] = {
	0x96, 0x97, 0x9a, 0x9b, 0x9d, 0x9e, 0x9f, 0xa6, 0xa7, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb2, 0xb3,
	0xb4, 0xb5, 0xb6, 0xb7, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xcb, 0xcd, 0xce, 0xcf, 0xd3,
	0xd6, 0xd7, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe5, 0xe6, 0xe7, 0xe9, 0xea, 0xeb, 0xec,
	0xed, 0xee, 0xef, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff
};

static int16_t ungcr6[256];

static void init_tables()
{
	static int done = 0;
	if (done) return;
	for (int i = 0; i < 256; i++) ungcr6[i] = -1;
	for (int i = 0; i < 64; i++) ungcr6[gcr6[i]] = i;
	done = 1;
}

int mac_spt(int cyl)
{
	int z = cyl / 16;
	if (z > 4) z = 4;
	return 12 - z;
}

double mac_cell_s(int cyl)
{
	// Sony drives spin faster on the outer zones; read at 300 RPM the cells
	// written at 2 us become longer (Greaseweazle disk definitions)
	static const double cell[5] = { 2.63e-6, 2.89e-6, 3.20e-6, 3.57e-6, 3.98e-6 };
	int z = cyl / 16;
	if (z > 4) z = 4;
	return cell[z];
}

#define LOOKUP_LEN (MAC_SECTOR_BYTES / 3)

// 703 6-bit values -> 524 bytes; returns 0 if the checksum is good
static int decode_sector(const uint8_t *input, uint8_t *output)
{
	const uint8_t *in = input;
	uint8_t *out = output;

	uint8_t b1[LOOKUP_LEN + 1], b2[LOOKUP_LEN + 1], b3[LOOKUP_LEN + 1];

	for (int i = 0; i <= LOOKUP_LEN; i++)
	{
		uint8_t w4 = *in++;
		uint8_t w1 = *in++;
		uint8_t w2 = *in++;
		uint8_t w3 = (i != LOOKUP_LEN) ? *in++ : 0;
		b1[i] = (w1 & 0x3F) | ((w4 << 2) & 0xC0);
		b2[i] = (w2 & 0x3F) | ((w4 << 4) & 0xC0);
		b3[i] = (w3 & 0x3F) | ((w4 << 6) & 0xC0);
	}

	uint32_t c1 = 0, c2 = 0, c3 = 0;
	unsigned count = 0;
	for (;;)
	{
		c1 = (c1 & 0xFF) << 1;
		if (c1 & 0x0100) c1++;

		uint8_t val = b1[count] ^ c1;
		c3 += val;
		if (c1 & 0x0100) { c3++; c1 &= 0xFF; }
		*out++ = val;

		val = b2[count] ^ c3;
		c2 += val;
		if (c3 > 0xFF) { c2++; c3 &= 0xFF; }
		*out++ = val;

		if ((out - output) == MAC_SECTOR_BYTES) break;

		val = b3[count] ^ c2;
		c1 += val;
		if (c2 > 0xFF) { c1++; c2 &= 0xFF; }
		*out++ = val;
		count++;
	}

	uint8_t c4 = ((c1 & 0xC0) >> 6) | ((c2 & 0xC0) >> 4) | ((c3 & 0xC0) >> 2);
	c1 &= 0x3f; c2 &= 0x3f; c3 &= 0x3f; c4 &= 0x3f;
	uint8_t g4 = *in++, g3 = *in++, g2 = *in++, g1 = *in++;
	return !((g4 == c4) && (g3 == c3) && (g2 == c2) && (g1 == c1));
}

static inline int get_byte(const std::vector<uint8_t> &bits, size_t p)
{
	int v = 0;
	for (int i = 0; i < 8; i++) v = (v << 1) | bits[p + i];
	return v;
}

// n disk bytes at bit position p -> 6-bit values (0xFF for an invalid code)
static int get_gcr(const std::vector<uint8_t> &bits, size_t p, uint8_t *out, int n)
{
	if (p + (size_t)n * 8 > bits.size()) return 0;
	for (int i = 0; i < n; i++)
	{
		int v = ungcr6[get_byte(bits, p + i * 8)];
		out[i] = v < 0 ? 0xFF : (uint8_t)v;
	}
	return 1;
}

void mac_gcr_decode(const std::vector<uint32_t> &flux, double sample_freq, int cyl, std::vector<mac_sector> &out)
{
	init_tables();
	out.clear();

	std::vector<uint8_t> bits;
	mfm_flux_to_bits(flux, sample_freq, mac_cell_s(cyl), bits);

	uint32_t sr = 0;
	for (size_t i = 0; i < bits.size(); i++)
	{
		sr = ((sr << 1) | bits[i]) & 0xFFFFFF;
		if (sr != 0xD5AA96) continue;

		// address field
		size_t p = i + 1;
		uint8_t hdr[5];
		if (!get_gcr(bits, p, hdr, 5)) break;
		if (hdr[0] == 0xFF || hdr[1] == 0xFF || hdr[2] == 0xFF || hdr[3] == 0xFF || hdr[4] == 0xFF) continue;
		if ((hdr[0] ^ hdr[1] ^ hdr[2] ^ hdr[3] ^ hdr[4]) != 0) continue;

		mac_sector s;
		s.cyl = hdr[0] | ((hdr[2] & 1) << 6);
		s.side = (hdr[2] >> 5) & 1;
		s.sec = hdr[1];
		s.fmt = hdr[3];
		s.ok = 0;
		memset(s.data, 0, sizeof(s.data));

		// data field within the next 100 bytes
		p += 5 * 8;
		size_t end = p + 100 * 8;
		uint32_t dr = 0;
		size_t q = 0;
		for (size_t k = p; k < end && k < bits.size(); k++)
		{
			dr = ((dr << 1) | bits[k]) & 0xFFFFFF;
			if (dr == 0xD5AAAD) { q = k + 1; break; }
		}
		if (q)
		{
			uint8_t enc[703];
			// sector byte, then the 703 bytes of the sector
			if (get_gcr(bits, q + 8, enc, 703))
			{
				s.ok = !decode_sector(enc, s.data);
				i = q + 8 + 703 * 8 - 1;
				sr = 0;
			}
		}
		out.push_back(s);
	}
}

