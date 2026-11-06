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
};

// CRC-CCITT as computed by the WD1772 (init 0xFFFF, poly 0x1021)
uint16_t st_crc16(uint16_t crc, uint8_t value);
uint16_t st_crc16_buf(uint16_t crc, const uint8_t *data, int len);

#endif
