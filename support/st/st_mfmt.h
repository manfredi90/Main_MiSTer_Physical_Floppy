// st_mfmt.h - "MFMT" track buffer exchanged between Main and the Atari ST core
//
// Copyright (C) 2026 Manfredi
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// One MFMT track describes one revolution of one side of one cylinder as a
// stream of MFM bit cells, plus the time each cell takes to pass under the
// head and the cells whose value is random on every read ("weak" bits).
// Main converts every supported image format (STX, IPF, live Greaseweazle...)
// into this representation; the core only has to play it back like a real
// drive and decode MFM like a WD1772.
//
// Transport: the core requests a track over the normal sd_* block interface
// with bit 31 of sd_lba set (see ST_MFMT_LBA_*). Two parts are requested for
// every track, first the META part, then the CELLS part:
//
//   META  : 8 blocks of 512 bytes (4 KiB)  header + timing table + weak table
//   CELLS : 32 blocks of 512 bytes (16 KiB) up to 131072 bit cells
//
// All multi-byte fields are little endian. Cells and weak masks are stored as
// 16-bit words (one word = 16 cells = one data byte worth of MFM), most
// significant bit first: cell n is bit (15 - n%16) of word n/16. This matches
// the 16-bit word order of the WIDE sd buffer, so the core can shift each word
// out MSB first.

#ifndef ST_MFMT_H
#define ST_MFMT_H

#include <stdint.h>

#define ST_MFMT_MAGIC            0x544D464DUL   // "MFMT" read as LE uint32
#define ST_MFMT_VERSION          1

// ---- sd_lba encoding of a track request (core -> Main) ----
//  bit 31      : 1 = MFMT track request (never a valid .st sector LBA)
//  bits 17..16 : part (0 = META, 1 = CELLS)
//  bit 8       : head (side)
//  bits 7..0   : cylinder
#define ST_MFMT_LBA_FLAG         0x80000000UL
#define ST_MFMT_LBA_PART_SHIFT   16
#define ST_MFMT_PART_META        0
#define ST_MFMT_PART_CELLS       1
#define ST_MFMT_LBA(cyl, head, part) \
	(ST_MFMT_LBA_FLAG | ((uint32_t)(part) << ST_MFMT_LBA_PART_SHIFT) | (((uint32_t)(head) & 1) << 8) | ((uint32_t)(cyl) & 0xFF))

#define ST_MFMT_META_BYTES       4096
#define ST_MFMT_CELLS_BYTES      16384
#define ST_MFMT_MAX_CELLS        (ST_MFMT_CELLS_BYTES * 8)   // 131072

// ---- META layout (byte offsets) ----
#define ST_MFMT_HDR_OFS          0x0000   // st_mfmt_header, 64 bytes
#define ST_MFMT_TIMING_OFS       0x0040   // st_mfmt_timing[ST_MFMT_MAX_TIMING]
#define ST_MFMT_MAX_TIMING       256
#define ST_MFMT_WEAK_OFS         0x0440   // st_mfmt_weak[ST_MFMT_MAX_WEAK]
#define ST_MFMT_MAX_WEAK         48
#define ST_MFMT_POOL_OFS         0x0500   // uint16_t weak masks, one per weak word
#define ST_MFMT_MAX_POOL_WORDS   ((ST_MFMT_META_BYTES - ST_MFMT_POOL_OFS) / 2)  // 1408

// header flags
#define ST_MFMT_F_FORMATTED      0x0001   // 0: no flux at all, the core reads noise
#define ST_MFMT_F_WEAK           0x0002   // weak table is used
#define ST_MFMT_F_VARTIME        0x0004   // more than one timing entry
#define ST_MFMT_F_HD             0x0008   // high density (nominal cell 1 us)
#define ST_MFMT_F_TRUNCATED      0x0100   // source did not fit, track was cut (diagnostics only)

// Cell length unit: 1/16 ns. Nominal DD cell = 2000 ns = 32000 units.
// A 32 MHz core clock period is 31.25 ns = 500 units: the core adds 500 to a
// phase accumulator every clock and moves to the next cell when the
// accumulator reaches the current cell length (subtracting it).
#define ST_MFMT_TIME_UNITS_PER_NS  16
#define ST_MFMT_CELL_DD          32000
#define ST_MFMT_CELL_HD          16000

#pragma pack(push, 1)

typedef struct
{
	uint32_t magic;          // ST_MFMT_MAGIC
	uint16_t version;        // ST_MFMT_VERSION
	uint16_t flags;          // ST_MFMT_F_*
	uint32_t cell_count;     // number of valid cells in one revolution (<= ST_MFMT_MAX_CELLS)
	uint16_t timing_count;   // number of entries in the timing table (>= 1)
	uint16_t weak_count;     // number of entries in the weak table
	uint8_t  cylinder;       // physical position this track was built for
	uint8_t  head;
	uint16_t pool_words;     // number of words used in the weak mask pool
	uint32_t rev_time_ns;    // informative: sum of all cell times
	uint8_t  reserved[40];
} st_mfmt_header;            // 64 bytes

// Every cell from word start_word on lasts cell_len units, until the next
// entry. Entries are sorted, entry 0 starts at word 0.
typedef struct
{
	uint16_t start_word;
	uint16_t cell_len;       // 1/16 ns
} st_mfmt_timing;

// Words [start_word, start_word + n_words) contain weak cells. Their masks
// are stored one word per track word in the pool, segment after segment.
// A mask bit set to 1 means: the cell at this position is random on every
// read. Only data cells (odd cells of an MFM pair) are normally marked.
typedef struct
{
	uint16_t start_word;
	uint16_t n_words;
} st_mfmt_weak;

#pragma pack(pop)

#endif
