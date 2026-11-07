// st_fdd.cpp - Atari ST floppy images served to the core as MFMT tracks
//
// Copyright (C) 2026 Manfredi
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Plain .st images keep using the sector interface of the core. Images that
// carry copy protections (STX for now) are converted here, one track at a
// time, into the MFMT representation of st_mfmt.h. The core asks for a track
// with an sd read whose LBA has bit 31 set; the META part comes first, then
// the CELLS part. Both are built together and cached.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdarg.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/stat.h>
#include <map>
#include <string>
#include <vector>

#include "../../hardware.h"
#include "../../user_io.h"
#include "../../spi.h"
#include "../../file_io.h"
#include "st_fdd.h"
#include "st_mfmt.h"
#include "st_stx.h"

#define ST_FDD_SLOTS     2
#define ST_FDD_MAX_FILE  (4 * 1024 * 1024)

// Built tracks are kept for the whole time the image is mounted: the
// rebuild of a protected track (phase jump search) can take seconds on
// the ARM, and loaders often go back and forth between tracks.
struct StFddTrack
{
	std::vector<uint8_t> meta, cells;
};

struct StFddSlot
{
	StxImage *stx = nullptr;
	std::map<int, StFddTrack> tracks;   // key: cyl * 2 + head
};

static StFddSlot slots[ST_FDD_SLOTS];

// Log on the SD card, /media/fat/logs/stx.log (the previous session is kept
// as stx_prev.log): the lines are buffered and written once a second by a
// thread, so the core never waits for the card and the last lines before
// a game stops are there too.
#define ST_LOG_DIR   "/media/fat/logs"
#define ST_LOG_PATH  ST_LOG_DIR "/stx.log"
#define ST_LOG_PREV  ST_LOG_DIR "/stx_prev.log"
static pthread_mutex_t log_lock = PTHREAD_MUTEX_INITIALIZER;
static std::string log_buf;
static bool log_started = false;

static void *log_thread(void *)
{
	for (;;)
	{
		sleep(1);
		pthread_mutex_lock(&log_lock);
		std::string b;
		b.swap(log_buf);
		pthread_mutex_unlock(&log_lock);
		if (b.empty()) continue;
		FILE *f = fopen(ST_LOG_PATH, "a");
		if (f)
		{
			fwrite(b.data(), 1, b.size(), f);
			fclose(f);
		}
	}
	return nullptr;
}

static void st_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void st_log(const char *fmt, ...)
{
	char line[512];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	printf("STX: %s\n", line);

	pthread_mutex_lock(&log_lock);
	if (!log_started)
	{
		log_started = true;
		mkdir(ST_LOG_DIR, 0755);
		rename(ST_LOG_PATH, ST_LOG_PREV);
		pthread_t th;
		if (!pthread_create(&th, nullptr, log_thread, nullptr)) pthread_detach(th);
	}
	char ts[32];
	snprintf(ts, sizeof(ts), "%10lu ", GetTimer(0));
	log_buf += ts;
	log_buf += line;
	log_buf += '\n';
	pthread_mutex_unlock(&log_lock);
}

static bool has_ext(const char *name, const char *ext)
{
	size_t n = strlen(name), e = strlen(ext);
	return n > e && name[n - e - 1] == '.' && !strcasecmp(name + n - e, ext);
}

void st_fdd_unmount(int index)
{
	if (index < 0 || index >= ST_FDD_SLOTS) return;
	if (slots[index].stx) st_log("drive %c: ejected", 'A' + index);
	delete slots[index].stx;
	slots[index].stx = nullptr;
	slots[index].tracks.clear();
}

int st_fdd_mount_hook(int index, const char *name, fileTYPE *f, int *writable)
{
	if (!is_st() || index < 0 || index >= ST_FDD_SLOTS) return 1;
	st_fdd_unmount(index);
	if (!has_ext(name, "stx")) return 1;

	if (f->size <= 0 || f->size > ST_FDD_MAX_FILE)
	{
		st_log("%s: bad size %lld", name, (long long)f->size);
		return 0;
	}

	size_t n = (size_t)f->size;
	uint8_t *buf = (uint8_t *)malloc(n);
	if (!buf) return 0;
	FileSeek(f, 0, SEEK_SET);
	size_t got = 0;
	while (got < n)
	{
		int r = FileReadAdv(f, buf + got, n - got);
		if (r <= 0) break;
		got += r;
	}
	FileSeek(f, 0, SEEK_SET);

	StxImage *img = new StxImage();
	bool ok = got == n && img->parse(buf, n);
	free(buf);
	if (!ok)
	{
		st_log("%s: %s", name, got == n ? img->error() : "read error");
		delete img;
		return 0;
	}

	slots[index].stx = img;
	*writable = 0;   // writing to STX images is not supported yet
	st_log("drive %c: %s, %d cylinders, %d heads, revision %d (Main built " __DATE__ " " __TIME__ ")", 'A' + index, name, img->cylinders(), img->heads(), img->revision());
	return 1;
}

uint64_t st_fdd_size(int index, uint64_t size)
{
	if (index < 0 || index >= ST_FDD_SLOTS || !slots[index].stx || !is_st()) return size;
	return size | (ST_FDD_TYPE_MFMT << ST_FDD_SIZE_TYPE_SHIFT);
}

static const StFddTrack &build(StFddSlot &s, int cyl, int head)
{
	int key = cyl * 2 + head;
	auto it = s.tracks.find(key);
	if (it != s.tracks.end()) return it->second;
	StFddTrack &t = s.tracks[key];
	t.meta.resize(ST_MFMT_META_BYTES);
	t.cells.resize(ST_MFMT_CELLS_BYTES);
	TrackBuilder tb;
	StxBuildReport rep;
	stx_build_track(*s.stx, cyl, head, tb, &rep);
	tb.encode((uint8_t)cyl, (uint8_t)head, t.meta.data(), t.cells.data());
	if (rep.jumps || rep.splice_cells)
		st_log("track %d.%d rebuilt with %d phase jump(s), splice +%d cells", cyl, head, rep.jumps, rep.splice_cells);
	return t;
}

int st_fdd_sd_service(int disk, int op, uint64_t lba, int sz, int ack)
{
	if (!is_st() || disk < 0 || disk >= ST_FDD_SLOTS || !slots[disk].stx) return 0;
	StFddSlot &s = slots[disk];

	static uint8_t buf[ST_MFMT_CELLS_BYTES];
	if (sz > (int)sizeof(buf)) sz = sizeof(buf);

	if (op == 2)
	{
		st_log("drive %c: write to lba %llx dropped", 'A' + disk, (unsigned long long)lba);
		// writes are accepted and dropped (read-only image)
		EnableIO();
		spi_w(UIO_SECTOR_WR | ack);
		spi_block_read(buf, user_io_get_width(), sz);
		DisableIO();
		return 1;
	}
	if (!(op & 1)) return -1;

	memset(buf, 0, sz);
	if (lba & ST_MFMT_LBA_FLAG)
	{
		int cyl = lba & 0xFF;
		int head = (lba >> 8) & 1;
		int part = (lba >> ST_MFMT_LBA_PART_SHIFT) & 3;
		diskled_on();
		unsigned long t0 = GetTimer(0);
		bool cached = s.tracks.count(cyl * 2 + head) != 0;
		const StFddTrack &t = build(s, cyl, head);
		if (part == ST_MFMT_PART_META)
			st_log("drive %c: track %d.%d %s (%lu ms)", 'A' + disk, cyl, head, cached ? "cached" : "built", GetTimer(0) - t0);
		if (part == ST_MFMT_PART_META) memcpy(buf, t.meta.data(), sz < ST_MFMT_META_BYTES ? sz : ST_MFMT_META_BYTES);
		else if (part == ST_MFMT_PART_CELLS) memcpy(buf, t.cells.data(), sz);
	}
	else st_log("drive %c: sector request lba %llu (not a track)", 'A' + disk, (unsigned long long)lba);
	// a sector request on a track image means an old core: it gets empty sectors

	EnableIO();
	spi_w(UIO_SECTOR_RD | ack);
	spi_block_write(buf, user_io_get_width(), sz);
	DisableIO();
	return 1;
}
