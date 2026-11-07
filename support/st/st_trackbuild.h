// st_trackbuild.h - build MFMT tracks from a byte level description
//
// Copyright (C) 2026 Manfredi
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Most observational formats (STX, and the sector view of IPF) describe a
// track as a sequence of decoded bytes. TrackBuilder keeps one entry per data
// byte (16 MFM cells): its value, whether it is written as a sync mark with a
// missing clock, which of its data bits are weak, and how long it takes to
// pass under the head. encode() turns this into the MFMT META and CELLS parts.

#ifndef ST_TRACKBUILD_H
#define ST_TRACKBUILD_H

#include <stdint.h>
#include <vector>
#include <utility>
#include "st_mfmt.h"

class TrackBuilder
{
public:
	enum Mark : uint8_t
	{
		MARK_NONE = 0,
		MARK_A1   = 1,   // 0x4489, A1 with missing clock (ID / data address mark sync)
		MARK_C2   = 2,   // 0x5224, C2 with missing clock (index address mark sync)
	};

	struct Byte
	{
		uint8_t value;
		uint8_t mark;    // Mark
		uint8_t weak;    // data bits (bit 7 first) that read randomly
		uint8_t owner;   // 0 = gap/fill, otherwise who wrote it (diagnostics, overlap checks)
		float   time_ns; // time for the whole byte (16 cells); 0 = derived from the revolution
	};

	TrackBuilder() {}

	// Start a new track of len data bytes filled with 'fill' (normally 0x4E)
	void reset(int len, uint8_t fill = 0x4E);

	int  size() const { return (int)m_bytes.size(); }
	Byte &at(int pos) { return m_bytes[wrap(pos)]; }
	const Byte &at(int pos) const { return m_bytes[wrap(pos)]; }

	// Write helpers, positions wrap around the end of the track
	void put(int pos, uint8_t value, uint8_t owner = 0, uint8_t mark = MARK_NONE);
	void fill(int pos, int count, uint8_t value, uint8_t owner = 0);
	void set_time(int pos, int count, float time_ns_per_byte);
	void set_weak(int pos, uint8_t weak_mask);

	void set_unformatted() { m_formatted = false; }
	void set_hd(bool hd) { m_hd = hd; }

	// Cells added after the last byte (0..15). The write splice near the
	// index of a real track is rarely a whole number of bytes: a sector read
	// across the index then continues one or more cells out of phase.
	void set_extra_cells(int k) { m_extra = (k < 0) ? 0 : (k > 15 ? 15 : k); }
	int extra_cells() const { return m_extra; }

	// Phase jumps inside the track: k cells inserted (1..15) or removed
	// (-1..-15) before byte pos. A protection mastered in several passes has such splices; a
	// field read across one continues out of phase (Turrican tracks 7-10).
	void add_jump(int pos, int k);
	void remove_jump(int pos);
	void clear_jumps() { m_jumps.clear(); }
	int jumps() const { return (int)m_jumps.size(); }

	// Single cells forced to a value, by position in the finished turn (as
	// build_cells() numbers them, so set them after the last change of the
	// layout). Lets a field read one cell out of phase carry its own bits
	// where its data cells are another field's clock cells: no flux where
	// both read 00, as on Turrican tracks 7-10.
	void set_cell(int cell, int bit) { m_cell_set.emplace_back(cell, (uint8_t)(bit & 1)); }
	void clear_cells() { m_cell_set.clear(); }
	int forced_cells() const { return (int)m_cell_set.size(); }

	// The whole turn as cells: cell values, weak cells, cell length (ns), and
	// the first cell of every byte. Used by encode() and by format
	// converters that check how a field reads.
	struct Cells
	{
		std::vector<uint8_t> bit, weak;
		std::vector<float> ns;
		std::vector<int> byte_cell;
	};
	void build_cells(Cells &out) const;

	// Revolution time used to size the bytes without an explicit time.
	// Default 200 ms (300 RPM).
	void set_revolution_ns(double ns) { m_rev_ns = ns; }

	// Encode to MFMT. meta must hold ST_MFMT_META_BYTES, cells ST_MFMT_CELLS_BYTES.
	// Returns false only on invalid state; overflow of tables is reported with
	// ST_MFMT_F_TRUNCATED in the header flags.
	bool encode(uint8_t cyl, uint8_t head, uint8_t *meta, uint8_t *cells) const;

	// Fill meta/cells with an unformatted (noise) track
	static void encode_noise(uint8_t cyl, uint8_t head, uint8_t *meta, uint8_t *cells, bool hd = false);

	int wrap(int pos) const
	{
		int n = (int)m_bytes.size();
		if (!n) return 0;
		pos %= n;
		return pos < 0 ? pos + n : pos;
	}

private:
	std::vector<Byte> m_bytes;
	bool   m_formatted = true;
	bool   m_hd = false;
	double m_rev_ns = 200000000.0;
	int    m_extra = 0;
	std::vector<std::pair<int, int>> m_jumps;   // sorted by position
	std::vector<std::pair<int, uint8_t>> m_cell_set;
	double free_byte_ns() const;
};

// CRC-CCITT as computed by the WD1772 (init 0xFFFF, poly 0x1021)
uint16_t st_crc16(uint16_t crc, uint8_t value);
uint16_t st_crc16_buf(uint16_t crc, const uint8_t *data, int len);

#endif
