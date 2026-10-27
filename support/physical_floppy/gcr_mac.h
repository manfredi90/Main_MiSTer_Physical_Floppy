#ifndef GCR_MAC_H
#define GCR_MAC_H

/*
 * Macintosh 400K/800K GCR (Sony variable-speed drives): flux -> sectors and
 * sectors -> flux, for a PC drive at a constant 300 RPM (the cell time then
 * depends on the speed zone). Used with the Greaseweazle for the MacPlus.
 *
 * The sector checksum code comes from the Greaseweazle host tools (public
 * domain, Keir Fraser), which took it from FluxEngine (David Given), in turn
 * inspired by the MAME implementation (ap_dsk35.cpp, Nathan Woods, R. Belmont).
 */

#include <stdint.h>
#include <vector>

#define MAC_SECTOR_BYTES 524    // 12 tag bytes + 512 data bytes
#define MAC_MAX_SPT      12

struct mac_sector
{
	int cyl, side, sec, fmt;
	int ok;                     // header and data checksums good
	uint8_t data[MAC_SECTOR_BYTES];
};

// sectors of a track of cylinder cyl: 12 (cylinders 0-15) down to 8 (64-79)
int mac_spt(int cyl);

// cell time (s) of a cylinder read at 300 RPM
double mac_cell_s(int cyl);

void mac_gcr_decode(const std::vector<uint32_t> &flux, double sample_freq, int cyl, std::vector<mac_sector> &out);

// a whole track (one revolution of rev_s seconds, written at 300 RPM) from the
// mac_spt(cyl) sectors in data (524 bytes each, tags first). fmt: 0x22 double
// sided, 0x02 single sided
void mac_gcr_encode_track(int cyl, int side, int fmt, const uint8_t *data, double sample_freq, double rev_s, std::vector<uint32_t> &flux);

#endif
