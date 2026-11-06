// wdsim.h - play back an MFMT track and decode it like a WD1772 (test tool)
#pragma once
#include <stdint.h>
#include <vector>
#include <string>
#include <random>
#include "st_mfmt.h"

struct Cell
{
	uint8_t bit;
	double  t0;   // start time in ns from the first index
	double  len;
};

struct DecByte
{
	uint8_t value;
	bool    sync;     // the 16 cells were 0x4489
	size_t  first;    // first cell
	size_t  last;     // last cell
};

struct IdField
{
	uint8_t id[6];
	bool    crc_ok;
	size_t  cell;     // cell of the first A1
	double  t;        // time of the first A1
	size_t  end_cell; // cell after the CRC
};

struct DataField
{
	bool    found = false;
	uint8_t dam = 0;
	std::vector<uint8_t> data;
	bool    crc_ok = false;
	double  t_first = 0, t_last = 0;  // time of the first / last data byte end
	double  data_ns = 0;              // time to transfer the data bytes
};

class WdSim
{
public:
	bool load(const uint8_t *meta, const uint8_t *cells, std::string &err);

	// expand 'revs' revolutions, weak bits re-randomised each revolution
	void spin(int revs, uint32_t seed);

	const std::vector<Cell> &stream() const { return m_cells; }
	size_t cells_per_rev() const { return m_count; }
	double rev_ns() const { return m_rev_ns; }
	const st_mfmt_header &header() const { return m_hdr; }

	// all ID fields whose first A1 lies in [from, to)
	std::vector<IdField> read_addresses(size_t from, size_t to) const;
	// data field following an ID field (searched within 43 bytes like the WD1772)
	DataField read_data(const IdField &id) const;
	// read track: bytes from the index at 'from' to the next index
	std::vector<uint8_t> read_track(size_t from) const;

private:
	DecByte byte_at(size_t cell) const;   // 16 cells starting at 'cell'

	st_mfmt_header m_hdr = {};
	std::vector<uint16_t> m_words;
	std::vector<st_mfmt_timing> m_tim;
	std::vector<st_mfmt_weak> m_weak;
	std::vector<uint16_t> m_pool;
	size_t m_count = 0;
	double m_rev_ns = 0;
	std::vector<Cell> m_cells;
};
