#ifndef MFM_IBM_H
#define MFM_IBM_H

/*
 * IBM PC MFM track format: flux -> sectors (PLL + decoder) and
 * sectors -> flux (encoder). Used with the Greaseweazle.
 */

#include <stdint.h>
#include <vector>

struct mfm_sector
{
	int cyl, head, sec, n;   // ID field (size = 128 << n)
	int ok;                  // ID and data CRC good
	uint8_t data[1024];
};

// flux: intervals between flux transitions, in sample ticks (sample_freq Hz).
// cell_ns: MFM bitcell (1000 for HD 500kbps, 2000 for DD 250kbps).
// Returns all sectors found (a sector may appear more than once when the
// flux covers more than one revolution; good copies come first per sector).
void mfm_ibm_decode(const std::vector<uint32_t> &flux, double sample_freq, double cell_ns, std::vector<mfm_sector> &out);

// PLL: flux intervals -> MFM cells (1 = transition), cell_s = nominal bitcell in seconds
void mfm_flux_to_bits(const std::vector<uint32_t> &flux, double freq, double cell_s, std::vector<uint8_t> &bits);

// guess the bitcell from the flux timing: 1000 (HD), 2000 (DD), 0 (no MFM-like data)
double mfm_guess_cell_ns(const std::vector<uint32_t> &flux, double sample_freq);

// builds a whole IBM MFM track: sectors 1..spt of 512 bytes, taken from data.
// Gaps as a PC format: returns the flux intervals (sample ticks) of one revolution.
void mfm_ibm_encode_track(int cyl, int head, int spt, const uint8_t *data, double cell_ns, double sample_freq,
	double rev_ns, std::vector<uint32_t> &flux);

#endif
