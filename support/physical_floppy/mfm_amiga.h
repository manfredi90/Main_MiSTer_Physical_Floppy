#ifndef MFM_AMIGA_H
#define MFM_AMIGA_H

/*
 * AmigaDOS track format (11 sectors of 512 bytes, odd/even MFM longs,
 * sync 0x4489 0x4489): flux -> sectors and sectors -> flux. Used with the
 * Greaseweazle for the AmigaDOS mode of the Minimig.
 */

#include <stdint.h>
#include <vector>

#define AMIGA_SPT 11

struct amiga_sector
{
	int track, sec;     // from the sector header (track = cylinder * 2 + side)
	int ok;             // header and data checksums good
	uint8_t data[512];
};

// flux: intervals between flux transitions in sample ticks (sample_freq Hz), DD (2 us cells)
void mfm_amiga_decode(const std::vector<uint32_t> &flux, double sample_freq, std::vector<amiga_sector> &out);

// a whole AmigaDOS track (sectors 0..10 from data, 11 * 512 bytes), from index
// to index: rev_ns = revolution time. Returns the flux intervals in sample ticks.
void mfm_amiga_encode_track(int track, const uint8_t *data, double sample_freq, double rev_ns, std::vector<uint32_t> &flux);

#endif
