/*
 * IBM PC MFM: flux <-> sectors.
 *
 * The PLL follows the one of the Greaseweazle host tools (public domain,
 * Keir Fraser): an aggressive phase/period adjustment that locks quickly on
 * drives with speed variations.
 */

#include <string.h>
#include "mfm_ibm.h"

#define PLL_PERIOD_ADJ 0.05
#define PLL_PHASE_ADJ  0.60
#define PLL_RANGE      0.10

static uint16_t crc16_ccitt(uint16_t crc, const uint8_t *p, int len)
{
	while (len--)
	{
		crc ^= (uint16_t)(*p++) << 8;
		for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}

/* ------------------------------------------------------------------ decode */

static void flux_to_bits(const std::vector<uint32_t> &flux, double freq, double cell_s, std::vector<uint8_t> &bits)
{
	double clock = cell_s;
	double cmin = cell_s * (1.0 - PLL_RANGE);
	double cmax = cell_s * (1.0 + PLL_RANGE);
	double ticks = 0;

	bits.clear();
	bits.reserve(flux.size() * 3);

	for (uint32_t x : flux)
	{
		ticks += x / freq;
		if (ticks < clock / 2) continue;

		int zeros = 0;
		for (;;)
		{
			ticks -= clock;
			if (ticks < clock / 2) break;
			zeros++;
			bits.push_back(0);
		}
		bits.push_back(1);

		double new_ticks = ticks * (1.0 - PLL_PHASE_ADJ);
		if (zeros <= 3) clock += ticks * PLL_PERIOD_ADJ;
		else clock += (cell_s - clock) * PLL_PERIOD_ADJ;
		if (clock < cmin) clock = cmin;
		else if (clock > cmax) clock = cmax;
		ticks = new_ticks;
	}
}

// data byte from 16 cells starting at pos (clock, data, clock, data...)
static inline int mfm_byte(const std::vector<uint8_t> &bits, size_t pos)
{
	if (pos + 16 > bits.size()) return -1;
	int v = 0;
	for (int i = 0; i < 8; i++) v = (v << 1) | bits[pos + i * 2 + 1];
	return v;
}

static int mfm_bytes(const std::vector<uint8_t> &bits, size_t pos, uint8_t *out, int n)
{
	for (int i = 0; i < n; i++)
	{
		int b = mfm_byte(bits, pos + i * 16);
		if (b < 0) return 0;
		out[i] = (uint8_t)b;
	}
	return 1;
}

void mfm_ibm_decode(const std::vector<uint32_t> &flux, double sample_freq, double cell_ns, std::vector<mfm_sector> &out)
{
	std::vector<uint8_t> bits;
	flux_to_bits(flux, sample_freq, cell_ns * 1e-9, bits);

	out.clear();

	static const uint8_t a1[3] = { 0xA1, 0xA1, 0xA1 };
	int id_valid = 0;
	int id_cyl = 0, id_head = 0, id_sec = 0, id_n = 0;
	size_t id_pos = 0;

	uint64_t sr = 0;
	for (size_t i = 0; i < bits.size(); i++)
	{
		sr = (sr << 1) | bits[i];
		if ((sr & 0xFFFFFFFFFFFFULL) != 0x448944894489ULL) continue;

		size_t p = i + 1;
		int mark = mfm_byte(bits, p);
		if (mark < 0) break;

		uint8_t buf[1024 + 8];
		if (mark == 0xFE)
		{
			uint8_t id[7];
			id[0] = 0xFE;
			if (!mfm_bytes(bits, p + 16, id + 1, 6)) break;
			uint16_t crc = crc16_ccitt(crc16_ccitt(0xFFFF, a1, 3), id, 7);
			if (!crc && id[4] <= 3)
			{
				id_valid = 1;
				id_cyl = id[1]; id_head = id[2]; id_sec = id[3]; id_n = id[4];
				id_pos = p;
			}
			else id_valid = 0;
			i = p + 7 * 16 - 1;
			sr = 0;
		}
		else if (mark == 0xFB || mark == 0xF8)
		{
			// the data field follows its ID within a few dozen bytes
			if (id_valid && p - id_pos < 80 * 16)
			{
				int size = 128 << id_n;
				buf[0] = (uint8_t)mark;
				if (!mfm_bytes(bits, p + 16, buf + 1, size + 2)) break;
				uint16_t crc = crc16_ccitt(crc16_ccitt(0xFFFF, a1, 3), buf, size + 3);

				mfm_sector s;
				s.cyl = id_cyl; s.head = id_head; s.sec = id_sec; s.n = id_n;
				s.ok = !crc;
				memcpy(s.data, buf + 1, size);
				out.push_back(s);

				i = p + (size + 3) * 16 - 1;
				sr = 0;
			}
			id_valid = 0;
		}
	}
}

double mfm_guess_cell_ns(const std::vector<uint32_t> &flux, double sample_freq)
{
	if (flux.size() < 2000) return 0;

	size_t hd = 0, dd = 0;
	for (uint32_t x : flux)
	{
		double us = x * 1e6 / sample_freq;
		if (us >= 1.6 && us < 2.8) hd++;
		else if (us >= 3.4 && us < 9.0) dd++;
	}

	if (hd > flux.size() / 5) return 1000;
	if (dd > flux.size() / 2) return 2000;
	return 0;
}

/* ------------------------------------------------------------------ encode */

struct cell_writer
{
	std::vector<uint8_t> cells;
	int last_data = 0;

	void raw16(uint16_t w)
	{
		for (int i = 15; i >= 0; i--) cells.push_back((w >> i) & 1);
		last_data = w & 1;
	}

	void byte(uint8_t b)
	{
		for (int i = 7; i >= 0; i--)
		{
			int d = (b >> i) & 1;
			cells.push_back((!d && !last_data) ? 1 : 0);
			cells.push_back(d);
			last_data = d;
		}
	}

	void bytes(uint8_t b, int n) { while (n--) byte(b); }
};

void mfm_ibm_encode_track(int cyl, int head, int spt, const uint8_t *data, double cell_ns, double sample_freq,
	double rev_ns, std::vector<uint32_t> &flux)
{
	static const uint8_t a1[3] = { 0xA1, 0xA1, 0xA1 };
	int gap3 = spt >= 21 ? 12 : (spt >= 18 ? 84 : (spt >= 15 ? 84 : 80));

	cell_writer w;
	w.bytes(0x4E, 80);
	w.bytes(0x00, 12);
	for (int i = 0; i < 3; i++) w.raw16(0x5224);
	w.byte(0xFC);
	w.bytes(0x4E, 50);

	for (int s = 0; s < spt; s++)
	{
		uint8_t id[7] = { 0xFE, (uint8_t)cyl, (uint8_t)head, (uint8_t)(s + 1), 2, 0, 0 };
		uint16_t crc = crc16_ccitt(crc16_ccitt(0xFFFF, a1, 3), id, 5);
		id[5] = crc >> 8; id[6] = crc & 0xFF;

		w.bytes(0x00, 12);
		for (int i = 0; i < 3; i++) w.raw16(0x4489);
		for (int i = 0; i < 7; i++) w.byte(id[i]);
		w.bytes(0x4E, 22);

		w.bytes(0x00, 12);
		for (int i = 0; i < 3; i++) w.raw16(0x4489);
		uint8_t mark = 0xFB;
		w.byte(mark);
		const uint8_t *d = data + s * 512;
		for (int i = 0; i < 512; i++) w.byte(d[i]);
		crc = crc16_ccitt(crc16_ccitt(crc16_ccitt(0xFFFF, a1, 3), &mark, 1), d, 512);
		w.byte(crc >> 8);
		w.byte(crc & 0xFF);
		w.bytes(0x4E, gap3);
	}

	size_t total = (size_t)(rev_ns / cell_ns);
	while (w.cells.size() < total) w.byte(0x4E);

	flux.clear();
	double tick_per_cell = cell_ns * 1e-9 * sample_freq;
	double acc = 0;
	for (uint8_t c : w.cells)
	{
		acc += tick_per_cell;
		if (c)
		{
			uint32_t t = (uint32_t)(acc + 0.5);
			flux.push_back(t);
			acc -= t;
		}
	}
}
