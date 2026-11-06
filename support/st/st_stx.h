// st_stx.h - Pasti (STX) image reader and STX -> MFMT track conversion
//
// Copyright (C) 2026 Manfredi
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// The meaning of the STX fields follows the reverse engineering documented in
// Hatari (src/floppies/stx.c, GPL v2+, by Nicolas Pomarede and others) and
// the Pasti format notes by Markus Fritze, P. Putnik and Jean Louis Guerin.
// STX is an observational format: it records what a WD1772 returned for
// read address / read sector / read track. The converter rebuilds a physical
// MFM track that gives the same answers.

#ifndef ST_STX_H
#define ST_STX_H

#include <stdint.h>
#include <stddef.h>
#include <vector>
#include "st_trackbuild.h"

// sector FDC_Status flags (bits 3,4,5 have the meaning of the WD1772 status register)
#define STX_SECT_VARIABLE_TIME  0x01
#define STX_SECT_CRC_ERROR      0x08
#define STX_SECT_RNF            0x10
#define STX_SECT_DELETED        0x20
#define STX_SECT_FUZZY          0x80

// track flags
#define STX_TRK_SECTOR_BLOCK    0x0001
#define STX_TRK_TRACK_IMAGE     0x0040
#define STX_TRK_IMAGE_SYNC      0x0080

struct StxSector
{
	uint32_t data_offset;
	uint16_t bit_position;    // data bits from the index to just after the IDAM ($FE)
	uint16_t read_time;       // us for the whole data field, 0 = standard
	uint8_t  id[6];           // track, head, sector, size code, crc hi, crc lo (as on disk)
	uint8_t  status;          // STX_SECT_*
	int      size;            // 128 << (id[3] & 3)
	const uint8_t *data;      // null if RNF
	const uint8_t *fuzzy;     // null if no fuzzy bits; mask bit 1 = stable bit
	const uint8_t *timing;    // null if standard timing; 2 bytes (BE) per 16 data bytes
};

struct StxTrack
{
	uint8_t  cylinder;
	uint8_t  head;
	uint16_t flags;
	uint16_t mfm_size;        // bytes (bits for tracks without sector blocks)
	bool     simple;          // no sector blocks: only 512 byte sectors, standard layout
	std::vector<StxSector> sectors;
	const uint8_t *image;     // optional read-track image
	int      image_size;
	int      image_sync;      // sync position field (-1 if absent)
	int      length;          // track length in bytes used for the rebuild
};

class StxImage
{
public:
	// Parses a complete STX file kept in memory. The buffer is copied.
	bool parse(const uint8_t *buf, size_t len);
	void clear();

	const char *error() const { return m_error; }
	int revision() const { return m_revision; }
	int heads() const { return m_heads; }
	int cylinders() const { return m_cylinders; }
	const std::vector<StxTrack> &tracks() const { return m_tracks; }
	const StxTrack *find(int cyl, int head) const;

private:
	bool parse_track(const uint8_t *p, const uint8_t *end, StxTrack &t);
	bool in(const uint8_t *p, size_t n) const;

	std::vector<uint8_t> m_file;
	std::vector<StxTrack> m_tracks;
	int m_track_index[256][2];
	int m_revision = 0;
	int m_heads = 0;
	int m_cylinders = 0;
	char m_error[128] = {};
};

// Diagnostics collected while rebuilding a track
struct StxBuildReport
{
	int sectors = 0;
	int headers_found_in_image = 0;   // ID fields located inside the track image
	int data_found_in_image = 0;
	int image_offset = 0;             // track image byte offset vs bit_position/8 - 4
	int overlaps = 0;                 // header bytes written over another sector's data
	int overlap_mismatch = 0;         // ... and changing that data
	bool image_ignored = false;       // track image unusable, rebuilt from the sectors
	int splice_cells = 0;             // extra cells at the end of the turn (write splice)
	int jumps = 0;                    // phase jumps found inside the track
	bool unformatted = false;
};

// Rebuild cylinder/head of an STX image into tb. Returns false if the track
// is absent (tb is then set to an unformatted track).
bool stx_build_track(const StxImage &img, int cyl, int head, TrackBuilder &tb, StxBuildReport *rep = nullptr);

#endif
