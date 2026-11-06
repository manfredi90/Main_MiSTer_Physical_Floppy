// wdsim.cpp - play back an MFMT track and decode it like a WD1772 (test tool)
#include <string.h>
#include "wdsim.h"
#include "st_trackbuild.h"

bool WdSim::load(const uint8_t *meta, const uint8_t *cells, std::string &err)
{
	memcpy(&m_hdr, meta + ST_MFMT_HDR_OFS, sizeof(m_hdr));
	if (m_hdr.magic != ST_MFMT_MAGIC) { err = "bad magic"; return false; }
	if (m_hdr.version != ST_MFMT_VERSION) { err = "bad version"; return false; }
	if (m_hdr.cell_count > ST_MFMT_MAX_CELLS) { err = "too many cells"; return false; }
	if (!m_hdr.timing_count || m_hdr.timing_count > ST_MFMT_MAX_TIMING) { err = "bad timing count"; return false; }
	if (m_hdr.weak_count > ST_MFMT_MAX_WEAK) { err = "bad weak count"; return false; }
	if (m_hdr.pool_words > ST_MFMT_MAX_POOL_WORDS) { err = "bad pool size"; return false; }

	m_count = m_hdr.cell_count;
	m_words.assign((const uint16_t *)cells, (const uint16_t *)cells + ST_MFMT_CELLS_BYTES / 2);
	const st_mfmt_timing *t = (const st_mfmt_timing *)(meta + ST_MFMT_TIMING_OFS);
	m_tim.assign(t, t + m_hdr.timing_count);
	if (m_tim[0].start_word != 0) { err = "timing[0] does not start at 0"; return false; }
	for (size_t i = 1; i < m_tim.size(); i++)
		if (m_tim[i].start_word <= m_tim[i - 1].start_word) { err = "timing not sorted"; return false; }
	const st_mfmt_weak *w = (const st_mfmt_weak *)(meta + ST_MFMT_WEAK_OFS);
	m_weak.assign(w, w + m_hdr.weak_count);
	const uint16_t *p = (const uint16_t *)(meta + ST_MFMT_POOL_OFS);
	m_pool.assign(p, p + m_hdr.pool_words);
	size_t used = 0;
	for (auto &s : m_weak) used += s.n_words;
	if (used != m_pool.size()) { err = "weak pool size mismatch"; return false; }
	return true;
}

void WdSim::spin(int revs, uint32_t seed)
{
	std::mt19937 rng(seed);
	m_cells.clear();
	m_cells.reserve(m_count * revs);
	bool formatted = m_hdr.flags & ST_MFMT_F_FORMATTED;

	// per-word weak mask for fast lookup
	std::vector<uint16_t> wmask(m_count / 16 + 1, 0);
	size_t pp = 0;
	for (auto &s : m_weak)
		for (int i = 0; i < s.n_words; i++, pp++)
			if (s.start_word + i < wmask.size()) wmask[s.start_word + i] = m_pool[pp];

	double t = 0;
	for (int r = 0; r < revs; r++)
	{
		size_t ti = 0;
		for (size_t c = 0; c < m_count; c++)
		{
			size_t w = c / 16;
			while (ti + 1 < m_tim.size() && m_tim[ti + 1].start_word <= w) ti++;
			double len = m_tim[ti].cell_len / (double)ST_MFMT_TIME_UNITS_PER_NS;
			uint8_t bit;
			if (!formatted) bit = rng() & 1;
			else
			{
				int sh = 15 - (int)(c % 16);
				bit = (m_words[w] >> sh) & 1;
				if ((wmask[w] >> sh) & 1) bit = rng() & 1;
			}
			m_cells.push_back(Cell{ bit, t, len });
			t += len;
		}
		if (r == 0) m_rev_ns = t;
	}
}

DecByte WdSim::byte_at(size_t cell) const
{
	DecByte b = {};
	uint16_t raw = 0;
	for (int i = 0; i < 16; i++) raw = (raw << 1) | m_cells[cell + i].bit;
	uint8_t v = 0;
	for (int i = 0; i < 8; i++) v = (v << 1) | ((raw >> (14 - 2 * i)) & 1);
	b.value = v;
	b.sync = raw == 0x4489;
	b.first = cell;
	b.last = cell + 15;
	return b;
}

// ID field = three A1 syncs, FE, 4 bytes, CRC
std::vector<IdField> WdSim::read_addresses(size_t from, size_t to) const
{
	std::vector<IdField> out;
	if (to + 16 * 12 > m_cells.size()) to = m_cells.size() - 16 * 12;
	uint16_t shift = 0;
	for (size_t c = from; c < to; c++)
	{
		shift = (shift << 1) | m_cells[c].bit;
		if (shift != 0x4489) continue;
		size_t first = c - 15;
		DecByte b2 = byte_at(first + 16), b3 = byte_at(first + 32);
		if (!b2.sync || !b3.sync) continue;
		DecByte m = byte_at(first + 48);
		if (m.value != 0xFE || m.sync) continue;
		IdField id = {};
		uint16_t crc = 0xFFFF;
		for (int i = 0; i < 3; i++) crc = st_crc16(crc, 0xA1);
		crc = st_crc16(crc, 0xFE);
		for (int i = 0; i < 6; i++)
		{
			id.id[i] = byte_at(first + 64 + 16 * i).value;
			if (i < 4) crc = st_crc16(crc, id.id[i]);
		}
		id.crc_ok = crc == (uint16_t)((id.id[4] << 8) | id.id[5]);
		id.cell = first;
		id.t = m_cells[first].t0;
		id.end_cell = first + 160;
		out.push_back(id);
		c = first + 159;   // the WD1772 is busy with this ID until its end
		shift = 0;
	}
	return out;
}

DataField WdSim::read_data(const IdField &id) const
{
	DataField d;
	int size = 128 << (id.id[3] & 3);
	// WD1772: the data address mark must be found within 43 bytes (MFM) after the ID CRC
	size_t limit = id.end_cell + 43 * 16;
	uint16_t shift = 0;
	for (size_t c = id.end_cell; c < limit && c + 16 * (size + 8) < m_cells.size(); c++)
	{
		shift = (shift << 1) | m_cells[c].bit;
		if (shift != 0x4489) continue;
		size_t first = c - 15;
		DecByte b2 = byte_at(first + 16), b3 = byte_at(first + 32);
		if (!b2.sync || !b3.sync) continue;
		DecByte m = byte_at(first + 48);
		if (m.value < 0xF8 || m.value > 0xFB) return d;
		d.found = true;
		d.dam = m.value;
		uint16_t crc = 0xFFFF;
		for (int i = 0; i < 3; i++) crc = st_crc16(crc, 0xA1);
		crc = st_crc16(crc, m.value);
		size_t p = first + 64;
		for (int i = 0; i < size; i++, p += 16)
		{
			DecByte b = byte_at(p);
			d.data.push_back(b.value);
			crc = st_crc16(crc, b.value);
			if (i == 0) d.t_first = m_cells[b.last].t0 + m_cells[b.last].len;
			if (i == size - 1) d.t_last = m_cells[b.last].t0 + m_cells[b.last].len;
		}
		uint8_t c0 = byte_at(p).value, c1 = byte_at(p + 16).value;
		d.crc_ok = crc == (uint16_t)((c0 << 8) | c1);
		// transfer time of the data bytes: from the end of the DAM to the end of the last byte
		double t_dam_end = m_cells[first + 63].t0 + m_cells[first + 63].len;
		d.data_ns = d.t_last - t_dam_end;
		return d;
	}
	return d;
}

std::vector<uint8_t> WdSim::read_track(size_t from) const
{
	std::vector<uint8_t> out;
	size_t end = from + m_count;
	if (end + 16 > m_cells.size()) end = m_cells.size() - 16;
	uint16_t shift = 0;
	uint16_t raw = 0;
	int count = 0;
	// the shift register already holds the cells before 'from' (a real chip
	// keeps seeing the disk), so a sync mark across the index is detected
	if (from >= 16) for (size_t c = from - 16; c < from; c++) shift = (shift << 1) | m_cells[c].bit;
	raw = shift;
	for (size_t c = from; c < end; c++)
	{
		shift = (shift << 1) | m_cells[c].bit;
		raw = (raw << 1) | m_cells[c].bit;
		count++;
		// during read track the WD1772 re-synchronises on every A1 (0x4489)
		// and C2 (0x5224) pattern, also on false ones inside sector data.
		// The byte in progress is output only if more than 8 cells have been
		// shifted since the last byte boundary (behaviour measured on real
		// chips, see MAME wd_fdc.cpp): this gives the "14 A1 A1" / "C2 A1 A1"
		// sequences found in read-track dumps.
		bool sync = shift == 0x4489 || shift == 0x5224;
		if (sync && count != 16)
		{
			if (count <= 8) { count = 0; continue; }
			count = 16;
		}
		if (count == 16)
		{
			uint8_t v = 0;
			for (int i = 0; i < 8; i++) v = (v << 1) | ((raw >> (14 - 2 * i)) & 1);
			out.push_back(v);
			count = 0;
		}
	}
	return out;
}
