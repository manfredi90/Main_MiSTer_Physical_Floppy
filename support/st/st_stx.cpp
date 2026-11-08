// st_stx.cpp - Pasti (STX) image reader and STX -> MFMT track conversion
//
// Copyright (C) 2026 Manfredi
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include "st_stx.h"

static inline uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rd32(const uint8_t *p) { return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24)); }

// Standard WD1772 layout used by Pasti for tracks stored without sector blocks
#define STD_GAP1        60
#define STD_GAP2        12
#define STD_RAW_SECTOR  (12 + 3 + 1 + 4 + 2 + 22 + 12 + 3 + 1 + 512 + 2 + 40)   // 614

// Revision 0 Macrodos timing table (from Hatari, Pasti 0.4b behaviour)
static const uint8_t timing_rev0[] = {
	0x00,0x7f,0x00,0x7f,0x00,0x7f,0x00,0x7f,0x00,0x7f,0x00,0x7f,0x00,0x7f,0x00,0x7f,
	0x00,0x85,0x00,0x85,0x00,0x85,0x00,0x85,0x00,0x85,0x00,0x85,0x00,0x85,0x00,0x85,
	0x00,0x79,0x00,0x79,0x00,0x79,0x00,0x79,0x00,0x79,0x00,0x79,0x00,0x79,0x00,0x79,
	0x00,0x7f,0x00,0x7f,0x00,0x7f,0x00,0x7f,0x00,0x7f,0x00,0x7f,0x00,0x7f,0x00,0x7f
};

void StxImage::clear()
{
	m_file.clear();
	m_tracks.clear();
	memset(m_track_index, 0xFF, sizeof(m_track_index));
	m_revision = 0;
	m_heads = 0;
	m_cylinders = 0;
	m_error[0] = 0;
}

bool StxImage::in(const uint8_t *p, size_t n) const
{
	const uint8_t *b = m_file.data();
	return p >= b && n <= m_file.size() && (size_t)(p - b) <= m_file.size() - n;
}

const StxTrack *StxImage::find(int cyl, int head) const
{
	if (cyl < 0 || cyl > 255 || head < 0 || head > 1) return nullptr;
	int i = m_track_index[cyl][head];
	return (i >= 0) ? &m_tracks[i] : nullptr;
}

bool StxImage::parse(const uint8_t *buf, size_t len)
{
	clear();
	if (len < 16 || memcmp(buf, "RSY\0", 4))
	{
		snprintf(m_error, sizeof(m_error), "not an STX file");
		return false;
	}
	m_file.assign(buf, buf + len);
	const uint8_t *p = m_file.data();
	const uint8_t *end = p + len;

	int version = rd16(p + 4);
	int count = p[10];
	m_revision = p[11];
	if (version != 3)
	{
		snprintf(m_error, sizeof(m_error), "unsupported STX version %d", version);
		return false;
	}

	p += 16;
	for (int i = 0; i < count; i++)
	{
		if (!in(p, 16))
		{
			snprintf(m_error, sizeof(m_error), "truncated at track block %d", i);
			return false;
		}
		uint32_t block = rd32(p);
		if (block < 16 || !in(p, block))
		{
			snprintf(m_error, sizeof(m_error), "bad size of track block %d", i);
			return false;
		}
		StxTrack t;
		if (!parse_track(p, p + block, t))
		{
			snprintf(m_error, sizeof(m_error), "bad track block %d (cyl %d head %d)", i, p[14] & 0x7F, p[14] >> 7);
			return false;
		}
		if (m_track_index[t.cylinder][t.head] < 0)
		{
			m_track_index[t.cylinder][t.head] = (int)m_tracks.size();
			m_tracks.push_back(t);
			if (t.head + 1 > m_heads) m_heads = t.head + 1;
			if (t.cylinder + 1 > m_cylinders) m_cylinders = t.cylinder + 1;
		}
		p += block;
	}
	(void)end;
	return true;
}

bool StxImage::parse_track(const uint8_t *p, const uint8_t *end, StxTrack &t)
{
	(void)end;   // all accesses are checked against the whole file with in()
	uint32_t fuzzy_size = rd32(p + 4);
	int nsect = rd16(p + 8);
	t.flags = rd16(p + 10);
	t.mfm_size = rd16(p + 12);
	t.cylinder = p[14] & 0x7F;
	t.head = p[14] >> 7;
	t.simple = !(t.flags & STX_TRK_SECTOR_BLOCK);
	t.image = nullptr;
	t.image_size = 0;
	t.image_sync = -1;

	const uint8_t *q = p + 16;

	if (t.simple)
	{
		// only nsect sectors of 512 bytes, standard positions; mfm_size is in bits
		if (nsect && !in(q, (size_t)nsect * 512)) return false;
		int pos = STD_GAP1 + STD_GAP2 + 4;
		for (int s = 0; s < nsect; s++)
		{
			StxSector sec = {};
			sec.bit_position = (uint16_t)(pos * 8);
			sec.id[0] = t.cylinder;
			sec.id[1] = t.head;
			sec.id[2] = (uint8_t)(s + 1);
			sec.id[3] = 2;
			uint16_t crc = 0xFFFF;
			const uint8_t mark[4] = { 0xA1, 0xA1, 0xA1, 0xFE };
			crc = st_crc16_buf(crc, mark, 4);
			crc = st_crc16_buf(crc, sec.id, 4);
			sec.id[4] = crc >> 8;
			sec.id[5] = crc & 0xFF;
			sec.size = 512;
			sec.data = q + s * 512;
			t.sectors.push_back(sec);
			pos += STD_RAW_SECTOR;
		}
		t.length = t.mfm_size / 8;
		if (t.length < 5000 || t.length > 8000) t.length = 6250;
		t.bitpos_time = false;
		return true;
	}

	const uint8_t *desc = q;
	if (!in(desc, (size_t)nsect * 16)) return false;
	const uint8_t *fuzzy = desc + nsect * 16;
	if (!in(fuzzy, fuzzy_size)) return false;
	const uint8_t *tdata = fuzzy + fuzzy_size;   // sector data offsets are relative to this
	const uint8_t *sectors_data = tdata;

	if (t.flags & STX_TRK_TRACK_IMAGE)
	{
		const uint8_t *ti = tdata;
		if (t.flags & STX_TRK_IMAGE_SYNC)
		{
			if (!in(ti, 2)) return false;
			t.image_sync = rd16(ti);
			ti += 2;
		}
		if (!in(ti, 2)) return false;
		t.image_size = rd16(ti);
		ti += 2;
		if (!in(ti, t.image_size)) return false;
		t.image = ti;
		sectors_data = ti + t.image_size;
	}

	const uint8_t *fz = fuzzy;
	uint32_t max_end = 0;
	bool variable = false;
	for (int s = 0; s < nsect; s++)
	{
		const uint8_t *d = desc + s * 16;
		StxSector sec = {};
		sec.data_offset = rd32(d);
		sec.bit_position = rd16(d + 4);
		sec.read_time = rd16(d + 6);
		memcpy(sec.id, d + 8, 6);
		sec.status = d[14];
		sec.size = 128 << (sec.id[3] & 3);
		if (!(sec.status & STX_SECT_RNF))
		{
			sec.data = tdata + sec.data_offset;
			if (!in(sec.data, sec.size)) return false;
			if (sec.status & STX_SECT_FUZZY)
			{
				if (!in(fz, sec.size) || fz + sec.size > tdata) return false;
				sec.fuzzy = fz;
				fz += sec.size;
			}
			if (sec.data_offset + sec.size > max_end) max_end = sec.data_offset + sec.size;
			if (sec.status & STX_SECT_VARIABLE_TIME) variable = true;
		}
		t.sectors.push_back(sec);
	}

	if (variable)
	{
		const uint8_t *tim = tdata + max_end;
		if (tim < sectors_data) tim = sectors_data;
		const uint8_t *tp = nullptr;
		if (m_revision == 2)
		{
			if (!in(tim, 4)) return false;
			tp = tim + 4;   // u16 flags, u16 size, then 2 bytes per 16 data bytes
		}
		for (StxSector &sec : t.sectors)
		{
			if ((sec.status & STX_SECT_RNF) || !(sec.status & STX_SECT_VARIABLE_TIME)) continue;
			int n = (sec.size / 16) * 2;
			if (m_revision == 2)
			{
				if (!in(tp, n)) return false;
				sec.timing = tp;
				tp += n;
			}
			else if (sec.size <= 512) sec.timing = timing_rev0;   // table covers 512 bytes
		}
	}

	if (t.image) t.length = t.image_size;
	else t.length = t.mfm_size;
	if (t.length < 1000 || t.length > ST_MFMT_MAX_CELLS / 16) t.length = 6250;
	t.bitpos_time = m_revision != 0;
	return true;
}

// ---------------------------------------------------------------------------
// STX -> track
// ---------------------------------------------------------------------------

#define NOMINAL_REV_NS   200000000.0
#define NOMINAL_BYTE_NS  32000.0

// bit_position: Pasti revision 2 (and later tools) store a time from the
// index in 4 us units, 50000 per 200 ms turn whatever the length of the
// track; revision 0 counts the bits of the track itself (its read track
// images line up with bit_position / 8 on tracks of any length). On a track
// written faster or slower than standard the two differ by up to 5 % of a
// turn (Turrican II: 6597 bytes; Ghouls'n Ghosts track 79: 5903).
double stx_bitpos_bytes(const StxTrack &t, int bit_position)
{
	if (t.bitpos_time) return bit_position * (double)t.length / 50000.0;
	return bit_position / 8.0;
}

double stx_bitpos_ns(const StxTrack &t, int bit_position)
{
	if (t.bitpos_time) return bit_position * 4000.0;
	return bit_position * NOMINAL_REV_NS / (8.0 * t.length);
}

static double bitpos_bytes(const StxTrack &t, int bit_position) { return stx_bitpos_bytes(t, bit_position); }

// Time to read data byte i of a sector, 0 if the sector uses standard timing
static double sector_byte_ns(const StxSector &sec, int i)
{
	if (sec.timing)
	{
		int blk = i >> 4;
		uint16_t tv = (uint16_t)((sec.timing[blk * 2] << 8) | sec.timing[blk * 2 + 1]);
		// 1 unit = 32 FDC cycles at 8 MHz, plus 28 cycles per block of 16 bytes
		return (tv * 32.0 + 28.0) * 125.0 / 16.0;
	}
	// a read time far from the standard rate is not a speed the drive could
	// have read at (Dragon's Lair: 1024 byte sectors listed with twice their
	// time): standard timing then
	if (sec.read_time)
	{
		double b = sec.read_time * 1000.0 / sec.size;
		if (b > NOMINAL_BYTE_NS * 0.85 && b < NOMINAL_BYTE_NS * 1.15) return b;
	}
	return 0;
}

static double sector_fixed_ns(const StxSector &sec)
{
	if (!sec.data) return 0;
	double t = 0;
	for (int i = 0; i < sec.size; i++) t += sector_byte_ns(sec, i);
	return t;
}

static bool is_garbled_sync(uint8_t v)
{
	// bytes the WD1772 returns in read track for the first A1 of a sync
	// sequence, before it has locked on the missing clock (see MAME wd_fdc.cpp)
	return v == 0xA1 || v == 0x14 || v == 0xC2 || v == 0x0A || v == 0xE1 || v == 0xCE;
}

// Read track only aligns on sync marks, so the gap bytes it returns before
// the first sync after the index (or after a field) are often the gap read
// out of phase: 4E comes out as one of its bit rotations, 00 as FF. Runs of
// such bytes go back to the real gap values, as MAME does in pasti_dsk.cpp
// (Skyfox track 79 starts with 27 27 27..., a 4E gap read 3 cells late).
static void unshift_image_gaps(TrackBuilder &tb)
{
	static const uint8_t rot4e[] = { 0x4e, 0x21, 0x9c, 0x42, 0x39, 0x84, 0x72, 0x09,
	                                 0xe4, 0x12, 0xc9, 0x24, 0x93, 0x48, 0x27, 0x90 };
	int n = tb.size();
	for (int i = 0; i < n;)
	{
		uint8_t v = tb.at(i).value;
		int j = i + 1;
		while (j < n && tb.at(j).value == v) j++;
		if (j - i >= 4)
		{
			uint8_t r = v;
			if (v == 0xFF) r = 0x00;
			for (uint8_t x : rot4e) if (v == x) r = 0x4E;
			if (r != v) for (int k = i; k < j; k++) tb.put(k, r, 0);
		}
		i = j;
	}
}

// Rebuild the sync marks of a read-track image. "x A1 A1 mark" in read
// track output can only come from real sync marks on the disk, because the
// WD1772 re-aligned its byte boundary there.
static void mark_image_syncs(TrackBuilder &tb)
{
	int n = tb.size();
	for (int i = 0; i < n; i++)
	{
		uint8_t b1 = tb.at(i + 1).value, b2 = tb.at(i + 2).value, m = tb.at(i + 3).value;
		if (b1 == 0xA1 && b2 == 0xA1 && m >= 0xF8)
		{
			if (is_garbled_sync(tb.at(i).value)) tb.put(i, 0xA1, 0, TrackBuilder::MARK_A1);
			tb.put(i + 1, 0xA1, 0, TrackBuilder::MARK_A1);
			tb.put(i + 2, 0xA1, 0, TrackBuilder::MARK_A1);
		}
		else if (b1 == 0xC2 && b2 == 0xC2 && m == 0xFC)
		{
			tb.put(i, 0xC2, 0, TrackBuilder::MARK_C2);
			tb.put(i + 1, 0xC2, 0, TrackBuilder::MARK_C2);
			tb.put(i + 2, 0xC2, 0, TrackBuilder::MARK_C2);
		}
	}
}

static uint16_t id_crc(const uint8_t *id)
{
	static const uint8_t mark[4] = { 0xA1, 0xA1, 0xA1, 0xFE };
	return st_crc16_buf(st_crc16_buf(0xFFFF, mark, 4), id, 4);
}

// Look for "A1 A1 FE/FF tr hd sc sz" in the image around pos (pos = first A1)
static int find_header(const StxTrack &t, const StxSector &s, int pos, int window)
{
	int n = t.image_size;
	for (int d = 0; d <= window; d++)
	{
		for (int sgn = 0; sgn < 2; sgn++)
		{
			if (!d && sgn) continue;
			int p = pos + (sgn ? -d : d);
			p = ((p % n) + n) % n;
			const uint8_t *im = t.image;
			if (im[(p + 1) % n] == 0xA1 && im[(p + 2) % n] == 0xA1 &&
				(im[(p + 3) % n] == 0xFE || im[(p + 3) % n] == 0xFF) &&
				im[(p + 4) % n] == s.id[0] && im[(p + 5) % n] == s.id[1] &&
				im[(p + 6) % n] == s.id[2] && im[(p + 7) % n] == s.id[3])
				return p;
		}
	}
	return -1;
}

static int find_dam(const StxTrack &t, int hpos)
{
	int n = t.image_size;
	const uint8_t *im = t.image;
	for (int j = 14; j < 60; j++)
	{
		int p = (hpos + j) % n;
		uint8_t m = im[(p + 3) % n];
		if (im[(p + 1) % n] == 0xA1 && im[(p + 2) % n] == 0xA1 && m >= 0xF8 && m <= 0xFB) return p;
	}
	return -1;
}

// put a byte unless it belongs to another sector's data (used for gaps/preambles)
static void put_gap(TrackBuilder &tb, int pos, uint8_t v)
{
	if (!tb.at(pos).owner) tb.put(pos, v, 0);
}

static void put_owned(TrackBuilder &tb, int pos, uint8_t v, uint8_t owner, uint8_t mark, StxBuildReport *rep)
{
	TrackBuilder::Byte &b = tb.at(pos);
	if (b.owner && b.owner != owner)
	{
		if (rep)
		{
			rep->overlaps++;
			if (b.value != v) rep->overlap_mismatch++;
		}
		// a sync mark of a field nested in another sector's data stays a sync
		// mark: the outer sector reads the same byte value either way
		if (b.value == v && b.mark && !mark) mark = b.mark;
	}
	tb.put(pos, v, owner, mark);
}


// How a sector's data field reads on the cells: number of bytes equal to the
// STX (stable bits only), and the first differing byte. The WD keeps the byte
// grid it took from the data mark, so after a phase jump it reads shifted.
static int read_score(const TrackBuilder::Cells &cs, int c0, const StxSector &sec, int *first_bad)
{
	int total = (int)cs.bit.size(), eq = 0;
	*first_bad = sec.size;
	for (int i = 0; i < sec.size; i++)
	{
		uint8_t v = 0, wk = 0;
		for (int k = 0; k < 8; k++)
		{
			int c = (c0 + 16 * i + 2 * k + 1) % total;
			v = (uint8_t)((v << 1) | cs.bit[c]);
			wk = (uint8_t)((wk << 1) | cs.weak[c]);
		}
		uint8_t m = (uint8_t)((sec.fuzzy ? sec.fuzzy[i] : 0xFF) & ~wk);
		if (((v ^ sec.data[i]) & m) == 0) eq++;
		else if (*first_bad == sec.size) *first_bad = i;
	}
	return eq;
}

static bool is_rot4e(uint8_t v)
{
	static const uint8_t rot4e[] = { 0x4e, 0x21, 0x9c, 0x42, 0x39, 0x84, 0x72, 0x09,
	                                 0xe4, 0x12, 0xc9, 0x24, 0x93, 0x48, 0x27, 0x90 };
	for (uint8_t x : rot4e) if (v == x) return true;
	return false;
}

// Bytes a protected sector recorded after a phase jump are its out-of-phase
// view of the track, not the track itself. Where they are still its own
// (no other field was written there) they become the gap they show: runs
// of FF become 00, of 4E rotations 4E, and other bytes carry on the gap or
// preamble before them (the byte that straddles a sync mark, e.g. "fe").
static void unshift_owned(TrackBuilder &tb, int from, int to, uint8_t owner)
{
	uint8_t last = 0x4E;
	for (int i = from; i < to; i++)
	{
		TrackBuilder::Byte &b = tb.at(i);
		if (b.owner != owner) { last = b.mark ? last : b.value; continue; }
		uint8_t v = b.value;
		uint8_t r = (v == 0xFF || v == 0x00) ? 0x00 : is_rot4e(v) ? 0x4E : last;
		tb.put(i, r, owner);
		tb.set_weak(i, 0);
		last = r;
	}
}

// Cells fit_cells() may not force: IDs, data marks and the fields of sectors
// without an intentional CRC error.
static void lock_cells(const StxTrack &t, const std::vector<int> &hpos, const std::vector<int> &dpos, const TrackBuilder &tb, const TrackBuilder::Cells &cs, std::vector<uint8_t> &lock)
{
	int n = tb.size(), total = (int)cs.bit.size();
	lock.assign(total, 0);
	auto lock_bytes = [&](int from, int count) {
		for (int b = from; b < from + count; b++)
		{
			int p = tb.wrap(b), c0 = cs.byte_cell[p];
			int c1 = (p + 1 < n) ? cs.byte_cell[p + 1] : total;
			for (int c = c0; c < c1; c++) lock[c] = 1;
		}
	};
	for (int s = 0; s < (int)t.sectors.size(); s++)
	{
		const StxSector &sec = t.sectors[s];
		lock_bytes(hpos[s] - 3, 13);
		if (!sec.data || dpos[s] < 0) continue;
		lock_bytes(dpos[s], 4);
		if (!(sec.status & STX_SECT_CRC_ERROR)) lock_bytes(dpos[s], 6 + sec.size);
	}
}

// read_score() counting as equal the bytes fit_cells() can make equal
static int read_score_fix(const TrackBuilder::Cells &cs, const std::vector<uint8_t> &lock, int c0, const StxSector &sec, int *first_bad)
{
	int total = (int)cs.bit.size(), eq = 0;
	*first_bad = sec.size;
	for (int i = 0; i < sec.size; i++)
	{
		bool ok = true;
		for (int k = 0; k < 8 && ok; k++)
		{
			int c = (c0 + 16 * i + 2 * k + 1) % total;
			if (cs.weak[c]) continue;
			if (sec.fuzzy && !((sec.fuzzy[i] >> (7 - k)) & 1)) continue;
			if (cs.bit[c] != ((sec.data[i] >> (7 - k)) & 1) && lock[c]) ok = false;
		}
		if (ok) eq++;
		else if (*first_bad == sec.size) *first_bad = i;
	}
	return eq;
}

struct Span { int from, to; };   // [from, to) in track bytes, may exceed the size (wrap)

static bool inside(const std::vector<Span> &spans, int pos, int n)
{
	for (const Span &s : spans)
		for (int w = -1; w <= 1; w++)
		{
			int p = pos + w * n;
			if (p > s.from && p < s.to) return true;
		}
	return false;
}

// Protected sectors (bad CRC) whose data does not read as recorded may cross
// splices of a track mastered in several passes: the original WD went on
// with its old byte grid and read what follows out of phase (Turrican tracks
// 7-10: sector 0 reads the data mark and data of sector 16 one cell off, and
// sector 16 reads the ID of sector 1 off phase). Search, just before the
// first differing byte, the jump (position and 1..15 cells) that makes the
// sector read as recorded, without cutting a sound sector's fields.
static void fit_jumps(const StxTrack &t, const std::vector<int> &hpos, const std::vector<int> &dpos, TrackBuilder &tb, StxBuildReport *rep)
{
	int n = tb.size(), ns = (int)t.sectors.size();
	std::vector<Span> keep;
	for (int s = 0; s < ns; s++)
	{
		const StxSector &sec = t.sectors[s];
		keep.push_back(Span{ hpos[s], hpos[s] + 10 });
		bool prot = (sec.status & STX_SECT_CRC_ERROR) || sec.fuzzy;
		if (sec.data && dpos[s] >= 0 && !prot) keep.push_back(Span{ dpos[s], dpos[s] + 6 + sec.size });
	}

	std::vector<int> order;
	for (int s = 0; s < ns; s++)
	{
		const StxSector &sec = t.sectors[s];
		if (sec.data && dpos[s] >= 0 && (sec.status & STX_SECT_CRC_ERROR)) order.push_back(s);
	}
	std::sort(order.begin(), order.end(), [&](int a, int b) { return dpos[a] < dpos[b]; });

	TrackBuilder::Cells cs;
	// fixable bytes of all protected sectors: a jump for one must not cost the others
	auto total_score = [&](const TrackBuilder::Cells &c, const std::vector<uint8_t> &lk) {
		int sum = 0, fb;
		for (int o : order) sum += read_score_fix(c, lk, c.byte_cell[tb.wrap(dpos[o] + 4)], t.sectors[o], &fb);
		return sum;
	};
	for (int s : order)
	{
		const StxSector &sec = t.sectors[s];
		uint8_t owner = (uint8_t)(s < 254 ? s + 1 : 255);
		int start = dpos[s] + 4;
		for (int round = 0; round < 8; round++)
		{
			std::vector<uint8_t> lock;
			tb.build_cells(cs);
			lock_cells(t, hpos, dpos, tb, cs, lock);
			int bad, score = read_score_fix(cs, lock, cs.byte_cell[tb.wrap(start)], sec, &bad);
			if (bad >= sec.size) break;
			int all_score = total_score(cs, lock);

			// a splice is at a boundary (gap / preamble / field), up to 64 bytes
			// before the first byte that reads differently. Every candidate is
			// first scored on the cells shifted in place (cheap: the ARM of the
			// DE10-Nano would need seconds to rebuild the track for each); the
			// best few are then built for real.
			struct Cand { int pos, k, bad, score; };
			std::vector<Cand> cands;
			int total = (int)cs.bit.size(), c0 = cs.byte_cell[tb.wrap(start)];
			for (int b = bad > 64 ? bad - 64 : 1; b <= bad; b++)
			{
				int pos = start + b;
				if (pos >= n || inside(keep, pos, n)) continue;   // the index splice is fit_splice's job
				if (b != bad && tb.at(pos).value == tb.at(pos - 1).value && tb.at(pos).owner == tb.at(pos - 1).owner) continue;
				int J = cs.byte_cell[pos];
				for (int k = -15; k < 16; k++)
				{
					if (!k || J < 16) continue;
					int nt = total + k, thr = k > 0 ? J : J + k;
					auto src = [&](int c) {
						if (c < thr) return c;
						if (k > 0 && c < J + k) return J - 16 + (c - J) % 16;
						return c - k;
					};
					int fb = sec.size, sc = 0;
					for (int i = 0; i < sec.size; i++)
					{
						bool ok = true;
						for (int q = 0; q < 8 && ok; q++)
						{
							int c = src((c0 + 16 * i + 2 * q + 1) % nt);
							if (c < 0 || c >= total || cs.weak[c]) continue;
							if (sec.fuzzy && !((sec.fuzzy[i] >> (7 - q)) & 1)) continue;
							if (cs.bit[c] != ((sec.data[i] >> (7 - q)) & 1) && lock[c]) ok = false;
						}
						if (ok) sc++;
						else if (fb == sec.size) fb = i;
					}
					if (fb > bad) cands.push_back(Cand{ pos, k, fb, sc });
				}
			}
			std::sort(cands.begin(), cands.end(), [](const Cand &a, const Cand &b2) { return a.bad != b2.bad ? a.bad > b2.bad : a.score > b2.score; });
			if (cands.size() > 4) cands.resize(4);

			// a sector read across the index also depends on the splice cells:
			// after a jump they are fitted again
			bool cross = start + sec.size > n;
			int best_score = score, best_pos = -1, best_k = 0, best_bad = bad, best_extra = tb.extra_cells(), best_all = all_score;
			for (const Cand &cd : cands)
			{
				TrackBuilder trial = tb;
				trial.add_jump(cd.pos, cd.k);
				unshift_owned(trial, cd.pos, std::min(start + sec.size, n), owner);
				for (int e = cross ? 0 : tb.extra_cells(); e <= (cross ? 15 : tb.extra_cells()); e++)
				{
					trial.set_extra_cells(e);
					trial.build_cells(cs);
					lock_cells(t, hpos, dpos, trial, cs, lock);
					int fb, sc = read_score_fix(cs, lock, cs.byte_cell[start], sec, &fb);
					int all = total_score(cs, lock);
					if (fb > best_bad || (fb == best_bad && (all > best_all || (all == best_all && sc > best_score))))
					{ best_score = sc; best_pos = cd.pos; best_k = cd.k; best_bad = fb; best_extra = e; best_all = all; }
				}
			}
			// keep a jump only if the sector now reads as recorded much further on
			if (best_pos < 0 || best_bad - bad < 8 || best_score <= score || best_all <= all_score) break;
			tb.add_jump(best_pos, best_k);
			tb.set_extra_cells(best_extra);
			if (rep) rep->splice_cells = best_extra;
			unshift_owned(tb, best_pos, std::min(start + sec.size, n), owner);
			if (rep) rep->jumps++;
		}
	}
}

// A sector whose data runs past the index was read by the original WD
// across the write splice. Find how many extra cells at the end of the turn
// make its recorded data match (Skyfox, track 79: sector 2C reads the ID of
// sector 1 one cell out of phase).
static void fit_splice(const StxTrack &t, const std::vector<int> &dpos, TrackBuilder &tb, StxBuildReport *rep)
{
	int n = tb.size();
	std::vector<int> cross;
	for (int s = 0; s < (int)t.sectors.size(); s++)
	{
		const StxSector &sec = t.sectors[s];
		if (dpos[s] >= 0 && sec.data && dpos[s] + 4 + sec.size > n) cross.push_back(s);
	}
	if (cross.empty()) return;

	TrackBuilder::Cells cs;
	int best_k = 0, best_score = -1;
	for (int k = 0; k < 16; k++)
	{
		tb.set_extra_cells(k);
		tb.build_cells(cs);
		int score = 0, fb;
		for (int s : cross) score += read_score(cs, cs.byte_cell[dpos[s] + 4], t.sectors[s], &fb);
		if (score > best_score) { best_score = score; best_k = k; }
	}
	tb.set_extra_cells(best_k);
	if (rep) rep->splice_cells = best_k;
}

// After the layout is final: a protected sector (bad CRC) that still reads
// differently is read out of phase with the fields around it, so its data
// cells are their clock cells. Set those cells to the recorded data bits
// (a zero next to a zero clock is a stretch without flux, which the WD reads
// as 0 in both phases). The fields of sound sectors and all IDs are left
// alone; if anything that read right before reads worse after, nothing is
// changed.
static void fit_cells(const StxTrack &t, const std::vector<int> &hpos, const std::vector<int> &dpos, TrackBuilder &tb, StxBuildReport *rep)
{
	int ns = (int)t.sectors.size();
	TrackBuilder::Cells cs;
	tb.build_cells(cs);
	int total = (int)cs.bit.size();
	if (!total) return;

	std::vector<uint8_t> lock;
	lock_cells(t, hpos, dpos, tb, cs, lock);
	std::vector<int> prot;
	for (int s = 0; s < ns; s++)
	{
		const StxSector &sec = t.sectors[s];
		if (sec.data && dpos[s] >= 0 && (sec.status & STX_SECT_CRC_ERROR)) prot.push_back(s);
	}
	if (prot.empty()) return;

	auto scores = [&](std::vector<int> &out) {
		tb.build_cells(cs);
		out.clear();
		for (int s = 0; s < ns; s++)
		{
			const StxSector &sec = t.sectors[s];
			int fb;
			out.push_back(sec.data && dpos[s] >= 0 ? read_score(cs, cs.byte_cell[tb.wrap(dpos[s] + 4)], sec, &fb) : 0);
		}
	};
	std::vector<int> before, after;
	scores(before);

	// cells a protected sector reads as stable data: another one's fuzzy bit
	// cannot make them random
	std::vector<uint8_t> stable(total, 0);
	for (int s : prot)
	{
		const StxSector &sec = t.sectors[s];
		int c0 = cs.byte_cell[tb.wrap(dpos[s] + 4)];
		for (int i = 0; i < sec.size; i++)
			for (int k = 0; k < 8; k++)
				if (!sec.fuzzy || ((sec.fuzzy[i] >> (7 - k)) & 1)) stable[(c0 + 16 * i + 2 * k + 1) % total] = 1;
	}

	int forced = 0;
	for (int s : prot)
	{
		const StxSector &sec = t.sectors[s];
		tb.build_cells(cs);
		int c0 = cs.byte_cell[tb.wrap(dpos[s] + 4)];
		for (int i = 0; i < sec.size; i++)
			for (int k = 0; k < 8; k++)
			{
				int c = (c0 + 16 * i + 2 * k + 1) % total;
				int want = (sec.data[i] >> (7 - k)) & 1;
				if (sec.fuzzy && !((sec.fuzzy[i] >> (7 - k)) & 1))
				{
					// a fuzzy bit: its cell must read randomly
					if (!lock[c] && !cs.weak[c] && !stable[c]) { tb.set_weak_cell(c); forced++; }
					continue;
				}
				if (lock[c] || cs.weak[c] || cs.bit[c] == want) continue;
				tb.set_cell(c, want);
				forced++;
			}
	}
	scores(after);
	bool worse = false;
	for (int s = 0; s < ns; s++)
	{
		bool p = std::find(prot.begin(), prot.end(), s) != prot.end();
		if (p ? after[s] < before[s] : after[s] != before[s]) worse = true;
	}
	if (worse) { tb.clear_cells(); forced = 0; }
	if (rep) rep->forced_cells = forced;
}

// Fields that should read right but do not: IDs listed as readable, and the
// data with its CRC of sectors without an intentional CRC error or fuzzy bits.
static int bad_fields(const StxTrack &t, const std::vector<int> &hpos, const std::vector<int> &dpos, const TrackBuilder &tb)
{
	TrackBuilder::Cells cs;
	tb.build_cells(cs);
	int total = (int)cs.bit.size();
	if (!total) return 0;
	auto byte_at = [&](int c0, int i) {
		uint8_t v = 0;
		for (int k = 0; k < 8; k++) v = (uint8_t)((v << 1) | cs.bit[(c0 + 16 * i + 2 * k + 1) % total]);
		return v;
	};
	int bad = 0;
	for (int s = 0; s < (int)t.sectors.size(); s++)
	{
		const StxSector &sec = t.sectors[s];
		if (!(sec.status & STX_SECT_RNF))
		{
			// three A1 sync marks, FE, the ID and a CRC that is right unless
			// the STX says it is not
			int h0 = cs.byte_cell[tb.wrap(hpos[s])];
			bool ok = true;
			for (int i = 0; i < 3 && ok; i++)
			{
				uint16_t w = 0;
				for (int k = 0; k < 16; k++) w = (uint16_t)((w << 1) | cs.bit[(h0 + 16 * i + k) % total]);
				if (w != 0x4489) ok = false;
			}
			if (ok && byte_at(h0, 3) != 0xFE) ok = false;
			uint16_t crc = 0xFFFF;
			for (int i = 0; i < 3; i++) crc = st_crc16(crc, 0xA1);
			crc = st_crc16(crc, 0xFE);
			for (int i = 0; i < 4 && ok; i++)
			{
				uint8_t v = byte_at(h0, 4 + i);
				if (v != sec.id[i]) ok = false;
				crc = st_crc16(crc, v);
			}
			bool crc_ok = ok && ((byte_at(h0, 8) << 8) | byte_at(h0, 9)) == crc;
			if (ok && crc_ok != !(sec.status & STX_SECT_CRC_ERROR && sec.status & STX_SECT_RNF)) ok = false;
			if (!ok) bad++;
		}
		if (!sec.data || dpos[s] < 0 || (sec.status & STX_SECT_CRC_ERROR) || sec.fuzzy) continue;
		int c0 = cs.byte_cell[tb.wrap(dpos[s] + 4)];
		uint16_t crc = 0xFFFF;
		for (int i = 0; i < 3; i++) crc = st_crc16(crc, 0xA1);
		crc = st_crc16(crc, (sec.status & STX_SECT_DELETED) ? 0xF8 : 0xFB);
		bool ok = true;
		for (int i = 0; i < sec.size && ok; i++)
		{
			uint8_t v = byte_at(c0, i);
			if (v != sec.data[i]) ok = false;
			crc = st_crc16(crc, v);
		}
		if (ok && ((byte_at(c0, sec.size) << 8) | byte_at(c0, sec.size + 1)) != crc) ok = false;
		if (!ok) bad++;
	}
	return bad;
}

// Fields nested in the data of another sector (sector in sector protections,
// 70 overlapping sectors on track 79 of Bubble Ghost, Light Corridor, Ishar)
// are shown by that data byte for byte. Where the data of the sector before
// shows the sync marks and mark of a nested ID or data mark one to three
// bytes away (an ID up to 16) from where it was placed, the field moves there: the recorded
// data of the outer sector is more exact than the read track image or the
// timing the position came from.
static void align_nested(const StxTrack &t, std::vector<int> &hpos, std::vector<int> &dpos, int n)
{
	int ns = (int)t.sectors.size();
	auto wrap = [n](int p) { p %= n; return p < 0 ? p + n : p; };
	std::vector<int> order;
	for (int s = 0; s < ns; s++) if (dpos[s] >= 0) order.push_back(s);
	std::sort(order.begin(), order.end(), [&](int a, int b) { return dpos[a] < dpos[b]; });

	// the nearest sector before pos whose data covers it
	auto outer = [&](int pos, int self) {
		int best = -1, best_d = n;
		for (int a : order)
		{
			if (a == self || !t.sectors[a].data) continue;
			int d = wrap(pos - (dpos[a] + 4));
			if (d < t.sectors[a].size && d < best_d) { best = a; best_d = d; }
		}
		return best;
	};
	auto match = [&](const StxSector &a, int off, const uint8_t *pat, int len) {
		if (off < 0 || off + len > a.size) return false;
		for (int i = 0; i < len; i++) if (a.data[off + i] != pat[i]) return false;
		return true;
	};
	for (int b : order)
	{
		const StxSector &sb = t.sectors[b];
		// the ID, then the data mark of b
		for (int field = 0; field < 2; field++)
		{
			int pos = field ? dpos[b] : hpos[b];
			if (field == 0 && (sb.status & STX_SECT_RNF)) continue;
			int a = outer(pos, b);
			if (a < 0) continue;
			uint8_t pat[8] = { 0xA1, 0xA1, 0xA1, 0xFE };
			int len = 4;
			if (field) pat[3] = (sb.status & STX_SECT_DELETED) ? 0xF8 : 0xFB;
			else { memcpy(pat + 4, sb.id, 4); len = 8; }
			int off = wrap(pos - (dpos[a] + 4));
			if (match(t.sectors[a], off, pat, len)) continue;
			// an ID (with its track and sector) is distinctive enough to look
			// further: one missing from the image was placed by time only
			int range = field ? 3 : 16;
			for (int k = 1; k <= range; k++)
			{
				int dk = 0;
				if (match(t.sectors[a], off + k, pat, len)) dk = k;
				else if (match(t.sectors[a], off - k, pat, len)) dk = -k;
				if (!dk) continue;
				if (field) dpos[b] = wrap(dpos[b] + dk);
				else
				{
					// the data mark keeps its distance from the ID, then is checked itself
					hpos[b] = wrap(hpos[b] + dk);
					dpos[b] = wrap(dpos[b] + dk);
				}
				break;
			}
		}
	}
}

static bool build_track(const StxImage &img, int cyl, int head, TrackBuilder &tb, StxBuildReport *rep, bool allow_image)
{
	*rep = StxBuildReport();

	const StxTrack *t = img.find(cyl, head);

	// A track image without sectors and without any sync sequence is the
	// read-track dump of an unformatted track: random bits, different on
	// every read. Play it back as noise rather than as fixed bytes.
	bool noise_image = false;
	if (t && t->image && t->sectors.empty())
	{
		noise_image = true;
		for (int i = 0; i + 2 < t->image_size && noise_image; i++)
			if (t->image[i] == 0xA1 && t->image[i + 1] == 0xA1 && t->image[i + 2] >= 0xF8) noise_image = false;
	}

	if (!t || (t->sectors.empty() && !t->image) || noise_image)
	{
		tb.reset(6250);
		tb.set_unformatted();
		rep->unformatted = true;
		return t != nullptr;
	}

	const int n = t->length;
	const int ns = (int)t->sectors.size();
	rep->sectors = ns;
	tb.reset(n, 0x4E);

	std::vector<int> hpos(ns), dpos(ns, -1);

	// With a track image the ID fields are located inside it, so the gaps
	// and their exact byte counts come from a real read track. An image in
	// which fewer than half of the ID fields can be found is not usable
	// (it was read out of sync, or made by a tool that did): the track is
	// then rebuilt from the sector list alone.
	bool use_image = allow_image && t->image != nullptr;
	std::vector<bool> in_image(ns, false);
	int delta = 0;
	if (use_image)
	{
		bool have_delta = false;
		for (int s = 0; s < ns; s++)
		{
			const StxSector &sec = t->sectors[s];
			int expect = (int)floor(bitpos_bytes(*t, sec.bit_position) + 0.5) - 4;
			int p = have_delta ? find_header(*t, sec, expect + delta, 48) : find_header(*t, sec, expect, n / 2);
			if (p < 0) continue;
			if (!have_delta)
			{
				delta = p - expect;
				have_delta = true;
				rep->image_offset = delta;
			}
			rep->headers_found_in_image++;
			hpos[s] = p;
			in_image[s] = true;
		}
		if (ns && rep->headers_found_in_image * 2 < ns)
		{
			use_image = false;
			rep->image_ignored = true;
			rep->image_offset = 0;
		}
	}

	if (use_image)
	{
		for (int i = 0; i < n; i++) tb.put(i, t->image[i], 0);
		unshift_image_gaps(tb);
		mark_image_syncs(tb);
		for (int s = 0; s < ns; s++)
		{
			if (!in_image[s])
			{
				// placed by time, or on the sync marks of an ID field the image
				// shows nearby with damaged contents (read out of phase): two ID
				// fields a few bytes apart would hide each other
				int e = (int)floor(bitpos_bytes(*t, t->sectors[s].bit_position) + 0.5) - 4 + delta;
				hpos[s] = tb.wrap(e);
				for (int d = 0; d <= 16; d++)
				{
					int found = -1;
					for (int sgn = -1; sgn <= 1 && found < 0; sgn += 2)
					{
						int p = tb.wrap(e + sgn * d);
						const uint8_t *im = t->image;
						if (is_garbled_sync(im[p]) && im[(p + 1) % n] == 0xA1 && im[(p + 2) % n] == 0xA1 && im[(p + 3) % n] == 0xFE)
						{
							bool other = false;
							for (int o = 0; o < ns; o++) if (o != s && in_image[o] && hpos[o] == p) other = true;
							if (!other) found = p;
						}
					}
					if (found >= 0) { hpos[s] = found; break; }
				}
			}
			else if (!(t->sectors[s].status & STX_SECT_RNF))
			{
				// the data mark is only searched behind an ID found in the image
				dpos[s] = find_dam(*t, hpos[s]);
				if (dpos[s] >= 0) rep->data_found_in_image++;
			}
		}
	}
	else
	{
		// bit_position is a time from the index (see bitpos_bytes). Here a
		// turn is 200 ms and the bytes with standard timing last free_byte, so
		// a sector read slower or faster than standard moves the following ID
		// fields by the equivalent number of bytes, to keep them on time.
		double fixed = 0;
		int fixed_bytes = 0;
		for (const StxSector &sec : t->sectors)
		{
			double f = sector_fixed_ns(sec);
			if (f > 0) { fixed += f; fixed_bytes += sec.size; }
		}
		double free_byte = NOMINAL_REV_NS / n;
		if (fixed_bytes && n > fixed_bytes) free_byte = (NOMINAL_REV_NS - fixed) / (n - fixed_bytes);
		if (free_byte < NOMINAL_BYTE_NS * 0.85) free_byte = NOMINAL_BYTE_NS * 0.85;
		if (free_byte > NOMINAL_BYTE_NS * 1.15) free_byte = NOMINAL_BYTE_NS * 1.15;
		double unit = NOMINAL_REV_NS / n;   // average time of a byte of the track

		double shift = 0;   // bytes
		for (int s = 0; s < ns; s++)
		{
			const StxSector &sec = t->sectors[s];
			double when = (bitpos_bytes(*t, sec.bit_position) - 4) * unit;
			hpos[s] = tb.wrap((int)floor(when / free_byte - shift + 0.5));
			double f = sector_fixed_ns(sec);
			if (f > 0) shift += (f - sec.size * free_byte) / free_byte;
		}
	}

	// Data marks not found in an image go at the standard distance from the
	// ID, but a sound sector cannot run into the next ID: on tightly packed
	// tracks (Dragon's Lair, 11 sectors) gap 2 is shorter, so the data mark
	// moves closer to its ID, not before the end of the ID field.
	for (int s = 0; s < ns; s++)
	{
		const StxSector &sec = t->sectors[s];
		if ((sec.status & STX_SECT_RNF) || dpos[s] >= 0) continue;
		int off = 44;
		if (sec.data && !(sec.status & STX_SECT_CRC_ERROR) && !sec.fuzzy)
		{
			int next = n;
			for (int o = 0; o < ns; o++)
			{
				if (o == s) continue;
				int d = tb.wrap(hpos[o] - hpos[s]);
				// an ID the data itself carries is nested on purpose, not in the way
				int r = d - off - 4;
				if (r >= 0 && r + 8 <= sec.size && sec.data[r + 3] == 0xFE && !memcmp(sec.data + r + 4, t->sectors[o].id, 4)) continue;
				if (d > 0 && d < next) next = d;
			}
			int need = off + 4 + sec.size + 2 + 2;
			if (need > next) off = std::max(10 + 2, next - (4 + sec.size + 2 + 2));
		}
		dpos[s] = tb.wrap(hpos[s] + off);
	}

	align_nested(*t, hpos, dpos, n);

	// pass 0: standard preambles (only over gap bytes)
	if (!use_image)
	{
		for (int s = 0; s < ns; s++)
		{
			for (int i = 1; i <= 12; i++) put_gap(tb, hpos[s] - i, 0x00);
			if (dpos[s] >= 0) for (int i = 1; i <= 12; i++) put_gap(tb, dpos[s] - i, 0x00);
		}
	}

	// pass 1: data fields. Sectors with an intentional CRC error or fuzzy
	// bits go first and sound sectors last, so where they overlap the sound
	// ones win: on the original disk those read correctly, while the
	// protected one has a bad CRC anyway (Skyfox: the data of sector 2C on
	// track 79 runs past the index over the ID and data mark of sector 1).
	for (int round = 0; round < 2; round++)
	for (int s = 0; s < ns; s++)
	{
		const StxSector &sec = t->sectors[s];
		if (dpos[s] < 0 || !sec.data) continue;
		bool protected_sector = (sec.status & STX_SECT_CRC_ERROR) || sec.fuzzy;
		if (protected_sector != (round == 0)) continue;
		uint8_t owner = (uint8_t)(s < 254 ? s + 1 : 255);
		int d = dpos[s];
		uint8_t dam = (sec.status & STX_SECT_DELETED) ? 0xF8 : 0xFB;

		uint16_t crc = 0xFFFF;
		for (int i = 0; i < 3; i++)
		{
			put_owned(tb, d + i, 0xA1, owner, TrackBuilder::MARK_A1, rep);
			tb.set_weak(d + i, 0);
			crc = st_crc16(crc, 0xA1);
		}
		put_owned(tb, d + 3, dam, owner, 0, rep);
		tb.set_weak(d + 3, 0);
		crc = st_crc16(crc, dam);

		// A protected sector (bad CRC) whose data runs past the index was read
		// across the write splice: after the index the original WD saw the
		// start of the track out of phase. Its bytes are written up to the
		// index only; the rest is what the track holds there, read through the
		// splice cells chosen by fit_splice().
		bool stop_at_index = (sec.status & STX_SECT_CRC_ERROR) && d + 4 + sec.size > tb.size();

		for (int i = 0; i < sec.size; i++)
		{
			int pos = d + 4 + i;
			uint8_t v = sec.data[i];
			crc = st_crc16(crc, v);
			if (stop_at_index && pos >= tb.size()) continue;
			uint8_t weak = 0;
			if (sec.fuzzy)
			{
				weak = (uint8_t)~sec.fuzzy[i];
				v &= sec.fuzzy[i];
			}
			put_owned(tb, pos, v, owner, 0, rep);
			tb.set_weak(pos, weak);
			double bt = sector_byte_ns(sec, i);
			if (bt > 0) tb.set_time(pos, 1, (float)bt);
		}

		if (sec.status & STX_SECT_CRC_ERROR)
		{
			// keep the CRC bytes of the image if they are already wrong
			bool img_bad = false;
			if (use_image)
			{
				uint8_t c0 = t->image[tb.wrap(d + 4 + sec.size)], c1 = t->image[tb.wrap(d + 5 + sec.size)];
				uint16_t ic = (uint16_t)((c0 << 8) | c1);
				if (ic != crc) { crc = ic; img_bad = true; }
			}
			if (!img_bad) crc ^= 0x5A5A;
		}
		if (stop_at_index) continue;
		put_owned(tb, d + 4 + sec.size, crc >> 8, owner, 0, rep);
		put_owned(tb, d + 5 + sec.size, crc & 0xFF, owner, 0, rep);
		tb.set_weak(d + 4 + sec.size, 0);
		tb.set_weak(d + 5 + sec.size, 0);
	}

	// pass 2: ID fields on top
	for (int s = 0; s < ns; s++)
	{
		const StxSector &sec = t->sectors[s];
		uint8_t owner = (uint8_t)(s < 254 ? s + 1 : 255);
		int h = hpos[s];
		uint8_t id[6];
		memcpy(id, sec.id, 6);
		uint16_t good = id_crc(id);
		uint16_t stored = (uint16_t)((id[4] << 8) | id[5]);
		bool id_error = (sec.status & STX_SECT_RNF) && (sec.status & STX_SECT_CRC_ERROR);
		if (id_error && stored == good) stored ^= 0x5A5A;
		if (!id_error && stored != good) stored = good;
		id[4] = stored >> 8;
		id[5] = stored & 0xFF;

		// an ID nested in another sector's data normally carries the same byte
		// values (the protection was mastered that way); differences are
		// counted in rep->overlap_mismatch
		// an ID listed by the STX was readable, so it is never weak, even when
		// it lies inside a fuzzy sector's data (OutRun, HLS protection: the
		// fuzzy sector 0B reads the gap and the ID of sector 6 one bit off)
		for (int i = 0; i < 3; i++) put_owned(tb, h + i, 0xA1, owner, TrackBuilder::MARK_A1, rep);
		put_owned(tb, h + 3, 0xFE, owner, 0, rep);
		for (int i = 0; i < 6; i++) put_owned(tb, h + 4 + i, id[i], owner, 0, rep);
		for (int i = 0; i < 10; i++) tb.set_weak(h + i, 0);
	}

	fit_splice(*t, dpos, tb, rep);
	fit_jumps(*t, hpos, dpos, tb, rep);
	fit_cells(*t, hpos, dpos, tb, rep);
	rep->used_image = use_image;
	rep->bad_fields = bad_fields(*t, hpos, dpos, tb);
	return true;
}

bool stx_build_track(const StxImage &img, int cyl, int head, TrackBuilder &tb, StxBuildReport *rep)
{
	StxBuildReport dummy;
	if (!rep) rep = &dummy;
	bool ok = build_track(img, cyl, head, tb, rep, true);
	// A track image can mislead the layout (a false ID or data mark found in
	// it, an ID missing from it placed by time next to image positions): if
	// a sound field then reads wrong, rebuild from the sector list alone and
	// keep whichever reads better.
	if (ok && rep->used_image && rep->bad_fields)
	{
		TrackBuilder tb2;
		StxBuildReport r2;
		build_track(img, cyl, head, tb2, &r2, false);
		if (r2.bad_fields < rep->bad_fields)
		{
			tb = tb2;
			*rep = r2;
			rep->image_ignored = true;
		}
	}
	return ok;
}
