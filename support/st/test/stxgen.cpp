// stxgen.cpp - write synthetic STX files covering the protection cases (test tool)
//
// Usage: stxgen out.stx
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <vector>
#include "st_trackbuild.h"

struct GSector
{
	uint8_t id[6];
	bool auto_crc = true;
	uint8_t status = 0;
	uint16_t bit_position = 0;
	uint16_t read_time = 0;
	std::vector<uint8_t> data;
	std::vector<uint8_t> fuzzy;
	std::vector<uint8_t> timing;   // BE u16 per 16 bytes
};

struct GTrack
{
	int cyl = 0, head = 0;
	bool simple = false;
	int mfm_size = 6250;
	std::vector<GSector> sectors;
	std::vector<uint8_t> image;
	bool with_sync = false;
	uint16_t sync = 0;
};

static void w16(std::vector<uint8_t> &v, uint16_t x) { v.push_back(x & 0xFF); v.push_back(x >> 8); }
static void w32(std::vector<uint8_t> &v, uint32_t x) { w16(v, x & 0xFFFF); w16(v, x >> 16); }

static uint16_t idcrc(const uint8_t *id)
{
	const uint8_t m[4] = { 0xA1, 0xA1, 0xA1, 0xFE };
	return st_crc16_buf(st_crc16_buf(0xFFFF, m, 4), id, 4);
}

static std::vector<uint8_t> track_block(GTrack &t)
{
	std::vector<uint8_t> body;
	uint16_t flags = 0;
	uint32_t fuzzy_size = 0;
	if (t.simple)
	{
		for (auto &s : t.sectors) body.insert(body.end(), s.data.begin(), s.data.end());
	}
	else
	{
		flags |= 0x0001;
		std::vector<uint8_t> fuzzy, tdata, sdata, timing;
		if (!t.image.empty())
		{
			flags |= 0x0040;
			if (t.with_sync) { flags |= 0x0080; w16(tdata, t.sync); }
			w16(tdata, (uint16_t)t.image.size());
			tdata.insert(tdata.end(), t.image.begin(), t.image.end());
		}
		std::vector<uint32_t> offs;
		bool var = false;
		for (auto &s : t.sectors)
		{
			if (s.auto_crc)
			{
				uint16_t c = idcrc(s.id);
				s.id[4] = c >> 8; s.id[5] = c & 0xFF;
			}
			if (!s.fuzzy.empty()) { s.status |= 0x80; fuzzy.insert(fuzzy.end(), s.fuzzy.begin(), s.fuzzy.end()); }
			if (!s.timing.empty()) { s.status |= 0x01; var = true; timing.insert(timing.end(), s.timing.begin(), s.timing.end()); }
			if (s.status & 0x10) { offs.push_back(0); continue; }
			offs.push_back((uint32_t)(tdata.size() + sdata.size()));
			sdata.insert(sdata.end(), s.data.begin(), s.data.end());
		}
		tdata.insert(tdata.end(), sdata.begin(), sdata.end());
		if (var)
		{
			w16(tdata, 5);
			w16(tdata, (uint16_t)(timing.size() + 4));
			tdata.insert(tdata.end(), timing.begin(), timing.end());
		}
		for (size_t i = 0; i < t.sectors.size(); i++)
		{
			auto &s = t.sectors[i];
			w32(body, offs[i]);
			w16(body, s.bit_position);
			w16(body, s.read_time);
			body.insert(body.end(), s.id, s.id + 6);
			body.push_back(s.status);
			body.push_back(0);
		}
		fuzzy_size = (uint32_t)fuzzy.size();
		body.insert(body.end(), fuzzy.begin(), fuzzy.end());
		body.insert(body.end(), tdata.begin(), tdata.end());
	}
	std::vector<uint8_t> blk;
	w32(blk, (uint32_t)(16 + body.size()));
	w32(blk, fuzzy_size);
	w16(blk, (uint16_t)t.sectors.size());
	w16(blk, flags);
	w16(blk, (uint16_t)(t.simple ? t.mfm_size * 8 : t.mfm_size));
	blk.push_back((uint8_t)(t.cyl | (t.head << 7)));
	blk.push_back(0);
	blk.insert(blk.end(), body.begin(), body.end());
	return blk;
}

static std::vector<uint8_t> pattern(int size, int seed)
{
	std::vector<uint8_t> v(size);
	uint32_t x = 0x12345678u ^ (seed * 2654435761u);
	for (int i = 0; i < size; i++) { x = x * 1103515245u + 12345u; v[i] = (uint8_t)(x >> 16); }
	return v;
}

// MFM of a byte sequence contains 0x4489 or 0x5224 off the byte grid?
static bool has_false_sync(const std::vector<uint8_t> &v)
{
	std::vector<uint8_t> cells;
	int prev = 1;
	for (uint8_t b : v)
		for (int i = 7; i >= 0; i--) { int d = (b >> i) & 1; cells.push_back(!prev && !d); cells.push_back(d); prev = d; }
	uint16_t sh = 0;
	for (size_t k = 0; k < cells.size(); k++)
	{
		sh = (sh << 1) | cells[k];
		if (k >= 15 && (sh == 0x4489 || sh == 0x5224)) return true;
	}
	return false;
}

// data without false sync patterns, so that a naive read-track image is exact
static std::vector<uint8_t> clean_pattern(int size, int seed)
{
	std::vector<uint8_t> v = pattern(size, seed);
	for (int i = 0; i < size; i++)
	{
		int lo = i > 2 ? i - 2 : 0;
		std::vector<uint8_t> w(v.begin() + lo, v.begin() + i + 1);
		while (has_false_sync(w)) { v[i] = (uint8_t)(v[i] * 7 + 3); w.back() = v[i]; }
	}
	return v;
}

static GSector sec(int c, int h, int s, int sizecode, int byte_pos, int seed)
{
	GSector g;
	g.id[0] = c; g.id[1] = h; g.id[2] = s; g.id[3] = sizecode;
	g.bit_position = (uint16_t)((byte_pos + 4) * 8);   // bit_position points after the IDAM
	g.data = pattern(128 << sizecode, seed);
	return g;
}

// standard layout: returns the byte position of the next ID field (first A1)
static int next_pos(int pos, int sizecode) { return pos + 10 + 22 + 12 + 4 + (128 << sizecode) + 2 + 40 + 12; }

// build the read-track bytes a WD1772 would return for a list of sectors
static std::vector<uint8_t> make_image(const std::vector<GSector> &ss, int len, uint8_t garble)
{
	std::vector<uint8_t> im(len, 0x4E);
	for (auto &s : ss)
	{
		int h = s.bit_position / 8 - 4;
		for (int i = 1; i <= 12; i++) im[(h - i + len) % len] = 0;
		im[h] = garble; im[h + 1] = 0xA1; im[h + 2] = 0xA1; im[h + 3] = 0xFE;
		memcpy(&im[h + 4], s.id, 6);
		if (s.status & 0x10) continue;
		int d = h + 44;
		for (int i = 1; i <= 12; i++) im[d - i] = 0;
		uint8_t dam = (s.status & 0x20) ? 0xF8 : 0xFB;
		im[d] = garble; im[d + 1] = 0xA1; im[d + 2] = 0xA1; im[d + 3] = dam;
		uint16_t crc = 0xFFFF;
		for (int i = 0; i < 3; i++) crc = st_crc16(crc, 0xA1);
		crc = st_crc16(crc, dam);
		for (size_t i = 0; i < s.data.size(); i++) { im[d + 4 + i] = s.data[i]; crc = st_crc16(crc, s.data[i]); }
		if (s.status & 0x08) crc ^= 0x1234;
		im[d + 4 + s.data.size()] = crc >> 8;
		im[d + 5 + s.data.size()] = crc & 0xFF;
	}
	return im;
}

int main(int argc, char **argv)
{
	if (argc < 2) { fprintf(stderr, "usage: stxgen out.stx\n"); return 1; }
	std::vector<GTrack> tracks;

	// 0: simple track, 9 standard sectors
	{
		GTrack t; t.cyl = 0;
		t.simple = true;
		for (int s = 1; s <= 9; s++) { GSector g; g.data = pattern(512, s); t.sectors.push_back(g); }
		tracks.push_back(t);
	}
	// 1: sector blocks, odd IDs and sizes, errors
	{
		GTrack t; t.cyl = 1;
		int p = 72;
		t.sectors.push_back(sec(1, 0, 1, 0, p, 101)); p = next_pos(p, 0);             // 128 bytes
		t.sectors.push_back(sec(1, 0, 2, 1, p, 102)); p = next_pos(p, 1);             // 256 bytes
		t.sectors.push_back(sec(1, 0, 3, 3, p, 103)); p = next_pos(p, 3);             // 1024 bytes
		GSector dup = sec(1, 0, 3, 1, p, 104); t.sectors.push_back(dup); p = next_pos(p, 1);   // duplicate ID
		GSector odd = sec(0x55, 1, 0xF7, 1, p, 105); t.sectors.push_back(odd); p = next_pos(p, 1);  // strange ID
		GSector idbad = sec(1, 0, 6, 1, p, 106); idbad.status = 0x18; idbad.auto_crc = false;
		idbad.id[4] = 0xDE; idbad.id[5] = 0xAD; t.sectors.push_back(idbad); p = next_pos(p, 1);  // ID CRC error
		GSector nodata = sec(1, 0, 7, 1, p, 107); nodata.status = 0x10; t.sectors.push_back(nodata); p = next_pos(p, 1);
		GSector dcrc = sec(1, 0, 8, 1, p, 108); dcrc.status = 0x08; t.sectors.push_back(dcrc); p = next_pos(p, 1);
		GSector del = sec(1, 0, 9, 1, p, 109); del.status = 0x20; t.sectors.push_back(del); p = next_pos(p, 1);
		tracks.push_back(t);
	}
	// 2: fuzzy bits
	{
		GTrack t; t.cyl = 2;
		int p = 72;
		for (int s = 1; s <= 9; s++)
		{
			GSector g = sec(2, 0, s, 2, p, 200 + s);
			if (s == 5)
			{
				g.fuzzy.assign(512, 0xFF);
				for (int i = 0; i < 64; i++) g.fuzzy[i] = 0x00;
				for (int i = 64; i < 128; i++) g.fuzzy[i] = 0xF0;
				g.status = 0x08;   // a fuzzy sector reads with a CRC error, as on the original disk
			}
			t.sectors.push_back(g);
			p = next_pos(p, 2);
		}
		tracks.push_back(t);
	}
	// 3: variable timing (Macrodos like table) and slow read time
	{
		GTrack t; t.cyl = 3;
		int p = 72;
		for (int s = 1; s <= 9; s++)
		{
			GSector g = sec(3, 0, s, 2, p, 300 + s);
			if (s == 4)
			{
				for (int b = 0; b < 32; b++)
				{
					uint16_t v = b < 8 ? 0x7F : b < 16 ? 0x85 : b < 24 ? 0x79 : 0x7F;
					g.timing.push_back(v >> 8); g.timing.push_back(v & 0xFF);
				}
			}
			if (s == 7) g.read_time = 17000;   // 6 % slower than 16384
			t.sectors.push_back(g);
			p = next_pos(p, 2);
		}
		tracks.push_back(t);
	}
	// 4: track image with garbled first sync bytes + sectors
	{
		GTrack t; t.cyl = 4;
		int p = 72;
		for (int s = 1; s <= 9; s++) { t.sectors.push_back(sec(4, 0, s, 2, p, 400 + s)); t.sectors.back().data = clean_pattern(512, 400 + s); p = next_pos(p, 2); }
		for (auto &s : t.sectors) { uint16_t c = idcrc(s.id); s.id[4] = c >> 8; s.id[5] = c & 0xFF; }
		t.image = make_image(t.sectors, 6256, 0x14);
		t.with_sync = true; t.sync = 0x48;
		tracks.push_back(t);
	}
	// 5: sector inside a sector
	{
		GTrack t; t.cyl = 5;
		GSector a = sec(5, 0, 1, 3, 72, 501);   // 1024 bytes, data starts at 72+48
		GSector b = sec(5, 0, 2, 0, 0, 502);    // 128 bytes, ID inside a's data
		int a_data = 72 + 48;
		int b_hdr = a_data + 200;
		b.bit_position = (uint16_t)((b_hdr + 4) * 8);
		uint16_t c = idcrc(b.id); b.id[4] = c >> 8; b.id[5] = c & 0xFF;
		std::vector<uint8_t> nest = { 0xA1, 0xA1, 0xA1, 0xFE, b.id[0], b.id[1], b.id[2], b.id[3], b.id[4], b.id[5] };
		for (int i = 0; i < 22; i++) nest.push_back(0x4E);
		for (int i = 0; i < 12; i++) nest.push_back(0x00);
		uint16_t dc = 0xFFFF;
		for (uint8_t x : { 0xA1, 0xA1, 0xA1, 0xFB }) { nest.push_back(x); dc = st_crc16(dc, x); }
		for (uint8_t x : b.data) { nest.push_back(x); dc = st_crc16(dc, x); }
		nest.push_back(dc >> 8); nest.push_back(dc & 0xFF);
		memcpy(&a.data[200], nest.data(), nest.size());
		t.sectors.push_back(a);
		t.sectors.push_back(b);
		GSector c3 = sec(5, 0, 3, 2, next_pos(72, 3), 503);
		t.sectors.push_back(c3);
		tracks.push_back(t);
	}
	// 6: empty, not formatted
	{
		GTrack t; t.cyl = 6;
		tracks.push_back(t);
	}
	// 7: track image only (no sectors)
	{
		GTrack t; t.cyl = 7;
		std::vector<GSector> ss;
		ss.push_back(sec(7, 0, 1, 2, 72, 701));
		ss[0].data = clean_pattern(512, 701);
		uint16_t c = idcrc(ss[0].id); ss[0].id[4] = c >> 8; ss[0].id[5] = c & 0xFF;
		t.image = make_image(ss, 6250, 0xC2);
		tracks.push_back(t);
	}
	// 8: long track (6500 bytes), 11 sectors
	{
		GTrack t; t.cyl = 8;
		t.mfm_size = 6500;
		int p = 30;
		for (int s = 1; s <= 11; s++) { t.sectors.push_back(sec(8, 0, s, 2, p, 800 + s)); p += 10 + 22 + 12 + 4 + 512 + 2 + 2 + 12 - 6; }
		tracks.push_back(t);
	}

	std::vector<uint8_t> file;
	const char id[4] = { 'R', 'S', 'Y', 0 };
	file.insert(file.end(), id, id + 4);
	w16(file, 3); w16(file, 1); w16(file, 0);
	file.push_back((uint8_t)tracks.size());
	file.push_back(2);
	w32(file, 0);
	for (auto &t : tracks) { auto b = track_block(t); file.insert(file.end(), b.begin(), b.end()); }

	FILE *f = fopen(argv[1], "wb");
	if (!f) { perror(argv[1]); return 1; }
	fwrite(file.data(), 1, file.size(), f);
	fclose(f);
	printf("wrote %s: %zu tracks, %zu bytes\n", argv[1], tracks.size(), file.size());
	return 0;
}
