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
	return true;
}

// ---------------------------------------------------------------------------
// STX -> track
// ---------------------------------------------------------------------------

#define NOMINAL_REV_NS   200000000.0
#define NOMINAL_BYTE_NS  32000.0

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
	if (sec.read_time) return sec.read_time * 1000.0 / sec.size;
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


// Byte k of a field whose first data byte starts at cell c0, decoded from
// MFMT cells (data bits are the odd cells), wrapping at the end of the turn
static uint8_t cells_byte(const uint8_t *cells, uint32_t ncells, uint32_t c0, int k)
{
	const uint16_t *w = (const uint16_t *)cells;
	uint8_t v = 0;
	for (int i = 0; i < 8; i++)
	{
		uint32_t c = (c0 + 16 * k + 2 * i + 1) % ncells;
		v = (uint8_t)((v << 1) | ((w[c >> 4] >> (15 - (c & 15))) & 1));
	}
	return v;
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

	std::vector<uint8_t> meta(ST_MFMT_META_BYTES), cells(ST_MFMT_CELLS_BYTES);
	int best_k = 0, best_score = -1;
	for (int k = 0; k < 16; k++)
	{
		tb.set_extra_cells(k);
		tb.encode(0, 0, meta.data(), cells.data());
		uint32_t nc = ((const st_mfmt_header *)meta.data())->cell_count;
		int score = 0;
		for (int s : cross)
		{
			const StxSector &sec = t.sectors[s];
			uint32_t c0 = (uint32_t)(dpos[s] + 4) * 16;
			for (int i = 0; i < sec.size; i++)
			{
				uint8_t m = sec.fuzzy ? sec.fuzzy[i] : 0xFF;
				if (((cells_byte(cells.data(), nc, c0, i) ^ sec.data[i]) & m) == 0) score++;
			}
		}
		if (score > best_score) { best_score = score; best_k = k; }
	}
	tb.set_extra_cells(best_k);
	if (rep) rep->splice_cells = best_k;
}

bool stx_build_track(const StxImage &img, int cyl, int head, TrackBuilder &tb, StxBuildReport *rep)
{
	StxBuildReport dummy;
	if (!rep) rep = &dummy;
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
	bool use_image = t->image != nullptr;
	std::vector<bool> in_image(ns, false);
	int delta = 0;
	if (use_image)
	{
		bool have_delta = false;
		for (int s = 0; s < ns; s++)
		{
			const StxSector &sec = t->sectors[s];
			int expect = sec.bit_position / 8 - 4;
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
			if (!in_image[s]) hpos[s] = tb.wrap(t->sectors[s].bit_position / 8 - 4 + delta);
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
		// bit_position is a time from the index: Hatari uses 4 us per data
		// bit with a revolution of length * 32 us, which is the fraction
		// bit_position / (8 * length) of a turn. Here a turn is 200 ms and
		// the bytes with standard timing last free_byte, so a sector read
		// slower or faster than standard moves the following ID fields by
		// the equivalent number of bytes, to keep them on time.
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
		double unit = NOMINAL_REV_NS / n;   // time of one byte of bit_position

		double shift = 0;   // bytes
		for (int s = 0; s < ns; s++)
		{
			const StxSector &sec = t->sectors[s];
			double when = (sec.bit_position / 8.0 - 4) * unit;
			hpos[s] = tb.wrap((int)floor(when / free_byte - shift + 0.5));
			double f = sector_fixed_ns(sec);
			if (f > 0) shift += (f - sec.size * free_byte) / free_byte;
		}
	}

	for (int s = 0; s < ns; s++)
		if (!(t->sectors[s].status & STX_SECT_RNF) && dpos[s] < 0) dpos[s] = tb.wrap(hpos[s] + 44);

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
	return true;
}
