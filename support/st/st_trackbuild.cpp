// st_trackbuild.cpp - build MFMT tracks from a byte level description
//
// Copyright (C) 2026 Manfredi
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "st_trackbuild.h"

uint16_t st_crc16(uint16_t crc, uint8_t value)
{
	crc ^= (uint16_t)value << 8;
	for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
	return crc;
}

uint16_t st_crc16_buf(uint16_t crc, const uint8_t *data, int len)
{
	while (len--) crc = st_crc16(crc, *data++);
	return crc;
}

void TrackBuilder::reset(int len, uint8_t fill_value)
{
	if (len < 1) len = 1;
	m_bytes.assign(len, Byte{ fill_value, MARK_NONE, 0, 0, 0.0f });
	m_formatted = true;
	m_hd = false;
	m_rev_ns = 200000000.0;
	m_extra = 0;
}

void TrackBuilder::put(int pos, uint8_t value, uint8_t owner, uint8_t mark)
{
	if (m_bytes.empty()) return;
	Byte &b = at(pos);
	b.value = value;
	b.mark = mark;
	b.owner = owner;
}

void TrackBuilder::fill(int pos, int count, uint8_t value, uint8_t owner)
{
	for (int i = 0; i < count; i++) put(pos + i, value, owner);
}

void TrackBuilder::set_time(int pos, int count, float time_ns_per_byte)
{
	if (m_bytes.empty()) return;
	for (int i = 0; i < count; i++) at(pos + i).time_ns = time_ns_per_byte;
}

void TrackBuilder::set_weak(int pos, uint8_t weak_mask)
{
	if (m_bytes.empty()) return;
	at(pos).weak = weak_mask;
}

// Standard MFM: clock bit is 1 only between two 0 data bits
static uint16_t mfm_encode(uint8_t value, int &prev_bit)
{
	uint16_t w = 0;
	for (int i = 7; i >= 0; i--)
	{
		int d = (value >> i) & 1;
		int c = (!prev_bit && !d) ? 1 : 0;
		w = (w << 2) | (c << 1) | d;
		prev_bit = d;
	}
	return w;
}

// Expand a data-bit weak mask (bit 7 first) to the data cells of a word
static uint16_t weak_to_cells(uint8_t weak)
{
	uint16_t w = 0;
	for (int k = 0; k < 8; k++) if (weak & (0x80 >> k)) w |= 1 << (14 - 2 * k);
	return w;
}

struct TimingRun
{
	uint32_t start;   // word
	uint32_t words;
	double   cell;    // units (1/16 ns)
};

static void merge_runs(std::vector<TimingRun> &runs, size_t max_runs)
{
	// Merge neighbours whose cell lengths are closest until the table fits.
	// The merged length is the time weighted average, so the total revolution
	// time is preserved.
	double threshold = 1.0;
	while (runs.size() > max_runs)
	{
		std::vector<TimingRun> out;
		for (const TimingRun &r : runs)
		{
			if (!out.empty() && fabs(out.back().cell - r.cell) <= threshold)
			{
				TimingRun &o = out.back();
				double total = o.cell * o.words + r.cell * r.words;
				o.words += r.words;
				o.cell = total / o.words;
			}
			else out.push_back(r);
		}
		runs.swap(out);
		threshold *= 2.0;
	}
}

bool TrackBuilder::encode(uint8_t cyl, uint8_t head, uint8_t *meta, uint8_t *cells) const
{
	memset(meta, 0, ST_MFMT_META_BYTES);
	memset(cells, 0, ST_MFMT_CELLS_BYTES);

	st_mfmt_header *hdr = (st_mfmt_header *)(meta + ST_MFMT_HDR_OFS);
	st_mfmt_timing *tim = (st_mfmt_timing *)(meta + ST_MFMT_TIMING_OFS);
	st_mfmt_weak *weak = (st_mfmt_weak *)(meta + ST_MFMT_WEAK_OFS);
	uint16_t *pool = (uint16_t *)(meta + ST_MFMT_POOL_OFS);
	uint16_t *cw = (uint16_t *)cells;

	hdr->magic = ST_MFMT_MAGIC;
	hdr->version = ST_MFMT_VERSION;
	hdr->cylinder = cyl;
	hdr->head = head;

	if (!m_formatted || m_bytes.empty())
	{
		encode_noise(cyl, head, meta, cells, m_hd);
		return true;
	}

	uint16_t flags = ST_MFMT_F_FORMATTED | (m_hd ? ST_MFMT_F_HD : 0);
	int n = (int)m_bytes.size();
	if (n > ST_MFMT_MAX_CELLS / 16 - 1)
	{
		n = ST_MFMT_MAX_CELLS / 16 - 1;
		flags |= ST_MFMT_F_TRUNCATED;
	}

	// ---- cells ----
	int prev = 0;
	{
		// clock of the first byte depends on the last data bit of the track
		const Byte &last = m_bytes[n - 1];
		prev = (last.mark == MARK_C2) ? 0 : (last.value & 1);
	}
	for (int i = 0; i < n; i++)
	{
		const Byte &b = m_bytes[i];
		uint16_t w;
		if (b.mark == MARK_A1) { w = 0x4489; prev = 1; }
		else if (b.mark == MARK_C2) { w = 0x5224; prev = 0; }
		else w = mfm_encode(b.value, prev);
		cw[i] = w;
	}

	// ---- timing ----
	const double nominal_byte = m_hd ? 16000.0 : 32000.0;   // ns
	double fixed_ns = 0;
	int free_bytes = 0;
	for (int i = 0; i < n; i++)
	{
		if (m_bytes[i].time_ns > 0) fixed_ns += m_bytes[i].time_ns;
		else free_bytes++;
	}
	double free_byte = nominal_byte;
	if (free_bytes) free_byte = (m_rev_ns - fixed_ns) / free_bytes;
	// keep the derived speed physically plausible (+/- 15 %)
	if (free_byte < nominal_byte * 0.85) free_byte = nominal_byte * 0.85;
	if (free_byte > nominal_byte * 1.15) free_byte = nominal_byte * 1.15;

	std::vector<TimingRun> runs;
	double rev = 0;
	for (int i = 0; i < n; i++)
	{
		double t = m_bytes[i].time_ns > 0 ? m_bytes[i].time_ns : free_byte;
		double cell = t;   // byte time in ns == cell length in 1/16 ns units
		rev += t;
		if (!runs.empty() && fabs(runs.back().cell - cell) < 0.5) runs.back().words++;
		else runs.push_back(TimingRun{ (uint32_t)i, 1, cell });
	}
	merge_runs(runs, ST_MFMT_MAX_TIMING);
	for (size_t i = 0; i < runs.size(); i++)
	{
		double c = floor(runs[i].cell + 0.5);
		if (c < 1) c = 1;
		if (c > 65535) c = 65535;
		tim[i].start_word = (uint16_t)runs[i].start;
		tim[i].cell_len = (uint16_t)c;
	}
	hdr->timing_count = (uint16_t)runs.size();
	if (runs.size() > 1) flags |= ST_MFMT_F_VARTIME;
	hdr->rev_time_ns = (uint32_t)(rev + 0.5);

	// ---- weak bits ----
	struct Seg { int start, words; };
	std::vector<Seg> segs;
	for (int i = 0; i < n; i++)
	{
		if (!m_bytes[i].weak) continue;
		if (!segs.empty() && segs.back().start + segs.back().words == i) segs.back().words++;
		else segs.push_back(Seg{ i, 1 });
	}
	// too many segments: join the closest ones (gap words get an empty mask)
	while (segs.size() > ST_MFMT_MAX_WEAK)
	{
		size_t best = 0;
		int best_gap = 1 << 30;
		for (size_t s = 0; s + 1 < segs.size(); s++)
		{
			int gap = segs[s + 1].start - (segs[s].start + segs[s].words);
			if (gap < best_gap) { best_gap = gap; best = s; }
		}
		segs[best].words = segs[best + 1].start + segs[best + 1].words - segs[best].start;
		segs.erase(segs.begin() + best + 1);
	}
	int pool_used = 0;
	int nweak = 0;
	for (const Seg &s : segs)
	{
		int words = s.words;
		if (pool_used + words > ST_MFMT_MAX_POOL_WORDS)
		{
			words = ST_MFMT_MAX_POOL_WORDS - pool_used;
			flags |= ST_MFMT_F_TRUNCATED;
		}
		if (words <= 0) break;
		weak[nweak].start_word = (uint16_t)s.start;
		weak[nweak].n_words = (uint16_t)words;
		for (int i = 0; i < words; i++) pool[pool_used + i] = weak_to_cells(m_bytes[s.start + i].weak);
		pool_used += words;
		nweak++;
	}
	hdr->weak_count = (uint16_t)nweak;
	hdr->pool_words = (uint16_t)pool_used;
	if (nweak) flags |= ST_MFMT_F_WEAK;

	// extra cells at the splice: the last cells of the first word, so the
	// gap at the start of the track simply continues across the splice
	// (only its phase changes, as on a real track)
	if (m_extra)
	{
		cw[n] = (uint16_t)(cw[0] << (16 - m_extra));
		// the first cell of the track now follows the last extra cell: apply
		// the MFM clock rule to it again (1 only between two 0 cells)
		int before = (cw[n] >> (16 - m_extra)) & 1;
		int after = (cw[0] >> 14) & 1;
		if (!(cw[0] == 0x4489 || cw[0] == 0x5224))
			cw[0] = (uint16_t)((cw[0] & 0x7FFF) | ((!before && !after) ? 0x8000 : 0));
		hdr->rev_time_ns += (uint32_t)(m_extra * free_byte / 16.0);
	}
	hdr->cell_count = (uint32_t)n * 16 + m_extra;
	hdr->flags = flags;
	return true;
}

void TrackBuilder::encode_noise(uint8_t cyl, uint8_t head, uint8_t *meta, uint8_t *cells, bool hd)
{
	memset(meta, 0, ST_MFMT_META_BYTES);
	memset(cells, 0, ST_MFMT_CELLS_BYTES);

	st_mfmt_header *hdr = (st_mfmt_header *)(meta + ST_MFMT_HDR_OFS);
	st_mfmt_timing *tim = (st_mfmt_timing *)(meta + ST_MFMT_TIMING_OFS);

	hdr->magic = ST_MFMT_MAGIC;
	hdr->version = ST_MFMT_VERSION;
	hdr->cylinder = cyl;
	hdr->head = head;
	hdr->flags = hd ? ST_MFMT_F_HD : 0;      // not formatted: the core generates noise
	hdr->cell_count = hd ? 200000 : 100000;
	hdr->timing_count = 1;
	hdr->rev_time_ns = 200000000;
	tim[0].start_word = 0;
	tim[0].cell_len = hd ? ST_MFMT_CELL_HD : ST_MFMT_CELL_DD;
}
