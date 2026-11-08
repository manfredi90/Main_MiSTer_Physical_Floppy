/*
 * Physical floppy for the Atari ST core, flux mode: the Greaseweazle drive in
 * floppy A: or B:, given to the core as an SCP flux image.
 *
 * The core (rtl/fdc1772/scp_track.sv) reads SCP images by itself, from the
 * file, as it plays the flux to its WD1772. Here the file is virtual: a fixed
 * header and track table (the same for every disk), and for each track a
 * slot that is filled from the drive when the core first asks for it. Until
 * then the request is not answered: the core waits, and holds its index
 * pulse meanwhile (a drive whose head is still settling), so the WD1772 does
 * not give up on a read. Any ST disk works this way, copy-protected ones
 * included (timing, weak bits: two revolutions are read, the core plays
 * them in turn).
 *
 * Tracks are read only when the core asks for them (the drive does what the
 * ST asks, nothing ahead), and kept until the disk is changed. A disk
 * inserted, removed or changed is mounted again (tos_insert_disk: the ST
 * notices the change through the write-protect line). Read-only: the disk is
 * shown write-protected (st_tos.cpp), the core cannot write to an SCP image.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <atomic>
#include <memory>
#include <vector>

#include "physical_floppy.h"
#include "pf_drive.h"
#include "../../user_io.h"
#include "../../spi.h"
#include "../../file_io.h"
#include "../st/st_tos.h"

#define SF_TRACKS        168     // 84 cylinders, 2 sides (the SCP table)
#define SF_REVS          2       // revolutions read per track
#define SF_MAX_FLUX      65536   // 16-bit samples per revolution (one bank of the core)
#define SF_TABLE_END     0x400   // header and track table (2 blocks)
#define SF_HDR_SIZE      512     // track header: "TRK", track, revolution table
#define SF_SLOT          (SF_HDR_SIZE + SF_REVS * SF_MAX_FLUX * 2)
#define SF_FILE_SIZE     ((uint64_t)SF_TABLE_END + (uint64_t)SF_TRACKS * SF_SLOT)
#define SF_TICK_HZ       40000000.0   // SCP resolution 0: 25 ns
#define SF_READ_FAILS    3       // a track that fails this many times is given as unformatted
#define SF_ACTIVE_MS     2000    // the core asked for a track within this time: drive in use
#define SF_MOTOR_KEEP_MS 3000    // motor kept on after the last read
#define SF_POLL_MS       1000    // disk presence check when idle
#define SF_ACTIVE_POLL_MS 500    // ... while the ST uses the drive
#define SF_SLOW_MS       1500    // log the tracks the core waited longer than this

/* ------------------------------------------------------------------ state */

// a track as it is in its slot of the virtual file (header + revolutions)
struct sf_track
{
	std::vector<uint8_t> data;   // empty: not read yet
	int fails;
};

static pthread_mutex_t sf_lock = PTHREAD_MUTEX_INITIALIZER;

static struct
{
	pthread_t thread;
	int thread_running;
	std::atomic<int> alive;

	PfFluxDrive *drv;            // I/O thread only
	double hz;

	sf_track tracks[SF_TRACKS];  // under sf_lock
	std::atomic<int> present;
	std::atomic<uint32_t> disk_gen;  // every insertion, removal or change
	std::atomic<int> want;           // track the core waits for, -1 none
	std::atomic<int> last_core;      // last request from the core (ms)

	char last_why[160];
	uint32_t reads;
	double read_ms;
	double last_read;
} sf;

static int sf_slot = -1;
static char sf_path[1024];
static uint32_t sf_seen_gen;
static int sf_placeholder_done = 0;
static int sf_write_logged = 0;
static uint8_t sf_head[SF_TABLE_END];   // header and track table
static double sf_wait_t0;               // the core waits for a track since
static int sf_wait_track = -1;

static int now_ms31() { return (int)((long long)pf_now_ms() & 0x7FFFFFFF); }
static int ms_since(int t) { return (now_ms31() - t) & 0x7FFFFFFF; }

static void put32(uint8_t *p, uint32_t v)
{
	p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

int pfloppy_is_st_flux_placeholder(const char *path)
{
	if (!path || !path[0]) return 0;
	const char *p = strrchr(path, '/');
	p = p ? p + 1 : path;
	return !strcasecmp(p, PFLOPPY_ST_FLUX_PLACEHOLDER);
}

int pfloppy_st_flux_slot(void)
{
	return sf_slot;
}

// the fixed part of the virtual file: SCP header and track table
static void build_head()
{
	memset(sf_head, 0, sizeof(sf_head));
	memcpy(sf_head, "SCP", 3);
	sf_head[3] = 0x19;           // version 1.9
	sf_head[4] = 0x14;           // Atari ST
	sf_head[5] = SF_REVS;
	sf_head[6] = 0;              // first track
	sf_head[7] = SF_TRACKS - 1;  // last track
	sf_head[8] = 0x01;           // index aligned
	sf_head[9] = 0;              // 16-bit samples
	sf_head[10] = 0;             // both sides
	sf_head[11] = 0;             // 25 ns
	for (int t = 0; t < SF_TRACKS; t++) put32(sf_head + 0x10 + t * 4, SF_TABLE_END + t * SF_SLOT);
	// the checksum stays 0: the core does not check it
}

/* ------------------------------------------------------------------ I/O thread */

static void clear_tracks()
{
	pthread_mutex_lock(&sf_lock);
	for (int t = 0; t < SF_TRACKS; t++)
	{
		sf.tracks[t].data.clear();
		sf.tracks[t].data.shrink_to_fit();
		sf.tracks[t].fails = 0;
	}
	pthread_mutex_unlock(&sf_lock);
}

// flux intervals in drive ticks -> 16-bit SCP samples (25 ns), rounding error
// carried on so the revolution keeps its length
static void to_scp(const std::vector<uint32_t> &flux, double hz, std::vector<uint8_t> &out, uint32_t &nwords)
{
	double scale = SF_TICK_HZ / hz, carry = 0;
	nwords = 0;
	for (uint32_t x : flux)
	{
		double exact = x * scale + carry;
		uint32_t v = (uint32_t)lround(exact);
		carry = exact - v;
		if (!v) v = 1;
		while (v >= 65536 && nwords < SF_MAX_FLUX)
		{
			out.push_back(0); out.push_back(0);
			nwords++;
			v -= 65536;
		}
		if (nwords >= SF_MAX_FLUX) break;
		if (!v) v = 1;
		out.push_back(v >> 8); out.push_back(v & 0xFF);
		nwords++;
	}
}

// an unformatted track (unreadable, or a cylinder the drive cannot reach):
// transitions 2..10 us apart, as the noise of an unformatted area
static void noise_rev(std::vector<uint8_t> &out, uint32_t &nwords, uint32_t &len, uint32_t &seed)
{
	nwords = 0;
	len = 0;
	while (len < 200e-3 * SF_TICK_HZ)
	{
		seed = seed * 1103515245u + 12345u;
		uint32_t v = 80 + (seed >> 8) % 320;
		out.push_back(v >> 8); out.push_back(v & 0xFF);
		nwords++;
		len += v;
	}
}

// the slot of track t from its revolutions (noise if revs is empty)
static void store_track(int t, const std::vector<pfd_rev> &revs, double hz)
{
	std::vector<uint8_t> d(SF_HDR_SIZE, 0);
	d[0] = 'T'; d[1] = 'R'; d[2] = 'K'; d[3] = (uint8_t)t;
	uint32_t seed = 0x5EED0000u + t;
	for (int r = 0; r < SF_REVS; r++)
	{
		size_t start = d.size();
		uint32_t nwords = 0, len = 0;
		if (revs.empty()) noise_rev(d, nwords, len, seed);
		else
		{
			// fewer revolutions than asked: the last one again
			const pfd_rev &rv = revs[r < (int)revs.size() ? r : revs.size() - 1];
			to_scp(rv.flux, hz, d, nwords);
			len = (uint32_t)lround(rv.len * SF_TICK_HZ / hz);
		}
		uint8_t *e = d.data() + 4 + r * 12;
		put32(e, len);
		put32(e + 4, nwords);
		put32(e + 8, (uint32_t)start);
	}

	pthread_mutex_lock(&sf_lock);
	sf.tracks[t].data.swap(d);
	pthread_mutex_unlock(&sf_lock);
}

static void sf_close_drive()
{
	if (sf.drv)
	{
		sf.drv->close();
		delete sf.drv;
		sf.drv = NULL;
	}
	if (sf.present)
	{
		pf_log("disk gone (drive closed)");
		sf.present = 0;
		sf.disk_gen++;
	}
	clear_tracks();
}

static int sf_open_drive()
{
	PfFluxDrive *d = pfd_gw_flux_create();
	char desc[128];
	if (!d->open(desc, sizeof(desc)))
	{
		delete d;
		if (strcmp(sf.last_why, "no greaseweazle"))
		{
			snprintf(sf.last_why, sizeof(sf.last_why), "no greaseweazle");
			pf_log("no Greaseweazle found");
		}
		return 0;
	}
	sf.drv = d;
	sf.hz = d->sample_hz();
	sf.last_why[0] = 0;
	pf_log("found %s", desc);
	return 1;
}

static void sf_probe()
{
	char why[160] = "";
	int r = sf.drv->flux_probe(sf.present, why, sizeof(why));
	switch (r)
	{
	case PFD_PROBE_NEW:
		pf_log(sf.present ? "disk changed" : "disk inserted");
		clear_tracks();
		sf.present = 1;
		sf.disk_gen++;
		sf.last_why[0] = 0;
		break;

	case PFD_PROBE_NODISK:
		if (sf.present)
		{
			pf_log("disk removed");
			sf.present = 0;
			clear_tracks();
			sf.disk_gen++;
		}
		if (strcmp(why, sf.last_why))
		{
			snprintf(sf.last_why, sizeof(sf.last_why), "%s", why);
			pf_log("%s", why);
		}
		break;

	case PFD_PROBE_LOST:
		pf_log("Greaseweazle lost: %s", why);
		sf_close_drive();
		break;

	default:
		break;
	}
}

static void sf_read_track(int t)
{
	std::vector<pfd_rev> revs;
	double t0 = pf_now_ms();
	int r = sf.drv->read_revs(t >> 1, t & 1, SF_REVS, revs);
	double ms = pf_now_ms() - t0;
	sf.last_read = pf_now_ms();

	switch (r)
	{
	case PFD_OK:
		store_track(t, revs, sf.hz);
		sf.reads++;
		sf.read_ms += ms;
		break;

	case PFD_CHANGED:
		pf_log("disk removed (track %d)", t);
		sf.present = 0;
		clear_tracks();
		sf.disk_gen++;
		break;

	case PFD_MEDIUM:
		pf_log("track %d: cylinder %d out of reach of the drive, given as unformatted", t, t >> 1);
		store_track(t, std::vector<pfd_rev>(), sf.hz);
		break;

	case PFD_NOT_READY:
	{
		pthread_mutex_lock(&sf_lock);
		int f = ++sf.tracks[t].fails;
		pthread_mutex_unlock(&sf_lock);
		if (f >= SF_READ_FAILS)
		{
			pf_log("track %d: unreadable, given as unformatted", t);
			store_track(t, std::vector<pfd_rev>(), sf.hz);
		}
		break;
	}

	case PFD_LOST:
	default:
		pf_log("Greaseweazle lost while reading track %d", t);
		sf_close_drive();
		break;
	}
}

static int track_ready(int t)
{
	pthread_mutex_lock(&sf_lock);
	int ok = !sf.tracks[t].data.empty();
	pthread_mutex_unlock(&sf_lock);
	return ok;
}

static void *sf_worker(void *)
{
	double last_poll = 0, last_scan = -1e9, last_stats = pf_now_ms();
	uint32_t logged_reads = 0;

	while (sf.alive)
	{
		double now = pf_now_ms();

		if (!sf.drv)
		{
			if (now - last_scan > 2000)
			{
				last_scan = now;
				if (sf_open_drive()) sf_probe();
			}
			usleep(50000);
			continue;
		}

		int active = ms_since(sf.last_core) < SF_ACTIVE_MS;

		// only the track the ST asks for
		int w = sf.want;
		if (sf.present && w >= 0 && w < SF_TRACKS && !track_ready(w))
		{
			sf_read_track(w);
			continue;
		}

		if (sf.drv->flux_motor_is_on() && !active && now - sf.last_read > SF_MOTOR_KEEP_MS)
			sf.drv->flux_motor_off();

		if (now - last_poll >= ((sf.present && active) ? SF_ACTIVE_POLL_MS : SF_POLL_MS))
		{
			last_poll = now;
			sf_probe();
			if (!sf.drv) continue;
		}

		if (now - last_stats >= 10000)
		{
			last_stats = now;
			if (sf.reads != logged_reads)
			{
				logged_reads = sf.reads;
				pf_log("stats: %u track reads, %.0f ms each on average", sf.reads, sf.reads ? sf.read_ms / sf.reads : 0.0);
			}
		}

		usleep(5000);
	}

	if (sf.drv) sf_close_drive();
	return NULL;
}

/* ------------------------------------------------------------------ mount */

static void sf_stop()
{
	if (!sf.thread_running) return;
	sf.alive = 0;
	pthread_join(sf.thread, NULL);
	sf.thread_running = 0;
	if (sf.drv) sf_close_drive();
	pf_log("detached");
}

static int sf_start()
{
	if (sf.thread_running) return 1;
	pf_log_reset();
	pf_log("attached to the Atari ST floppy (flux, read-only)");
	build_head();
	sf.present = 0;
	sf.want = -1;
	sf.last_core = (now_ms31() - SF_ACTIVE_MS - 1) & 0x7FFFFFFF;
	sf.last_why[0] = 0;
	sf.reads = 0;
	sf.read_ms = 0;
	clear_tracks();

	// first look done here, so the core sees the disk at once
	if (!sf.drv && sf_open_drive()) sf_probe();

	sf.alive = 1;
	if (pthread_create(&sf.thread, NULL, sf_worker, NULL))
	{
		pf_log("cannot start the I/O thread");
		sf.alive = 0;
		if (sf.drv) sf_close_drive();
		return 0;
	}
	sf.thread_running = 1;
	return 1;
}

uint64_t pfloppy_st_flux_mount(int index, const char *path)
{
	if (pfloppy_st_slot() >= 0) pfloppy_st_unmount(pfloppy_st_slot());  // the sector mode had the drive
	if (sf_slot >= 0 && sf_slot != index) pfloppy_st_flux_unmount(sf_slot);
	snprintf(sf_path, sizeof(sf_path), "%s", path);

	if (sf_slot != index)
	{
		if (!sf_start()) return 0;
		sf_slot = index;
		sf_write_logged = 0;
		pf_log("Atari ST floppy %c: any ST disk, copy-protected ones too, read-only", 'A' + index);
	}
	sf_seen_gen = sf.disk_gen;
	sf.want = -1;
	sf_wait_track = -1;
	if (!sf.present) return 0;
	pf_log("disk given to the core as a flux image");
	return SF_FILE_SIZE;
}

void pfloppy_st_flux_unmount(int index)
{
	if (index != sf_slot) return;
	sf_stop();
	sf_slot = -1;
}

/* ------------------------------------------------------------------ core requests */

int pfloppy_st_flux_sd_service(int disk, int op, uint32_t lba, int sz, int ack)
{
	if (sf_slot < 0 || disk != sf_slot || !op) return 0;

	static uint8_t buf[64 * 512];
	if (sz > (int)sizeof(buf)) sz = sizeof(buf);

	if (op & 1)
	{
		uint64_t pos = (uint64_t)lba * 512;
		if (pos + sz <= SF_TABLE_END) memcpy(buf, sf_head + pos, sz);
		else if (pos < SF_TABLE_END || !sf.present) memset(buf, 0, sz);
		else
		{
			uint64_t rel = pos - SF_TABLE_END;
			int t = (int)(rel / SF_SLOT);
			if (t >= SF_TRACKS) memset(buf, 0, sz);
			else
			{
				sf.last_core = now_ms31();
				pthread_mutex_lock(&sf_lock);
				const std::vector<uint8_t> &d = sf.tracks[t].data;
				int ready = !d.empty();
				if (ready)
				{
					// a request may run past the track into the next slot: zeros
					size_t o = (size_t)(rel - (uint64_t)t * SF_SLOT);
					for (int i = 0; i < sz; i++) buf[i] = o + i < d.size() ? d[o + i] : 0;
				}
				pthread_mutex_unlock(&sf_lock);
				if (!ready)
				{
					// not read yet: the core waits (and asks again)
					sf.want = t;
					if (sf_wait_track != t)
					{
						sf_wait_track = t;
						sf_wait_t0 = pf_now_ms();
					}
					return -1;
				}
				if (sf_wait_track == t)
				{
					double w = pf_now_ms() - sf_wait_t0;
					if (w > SF_SLOW_MS) pf_log("track %d (cylinder %d side %d): the ST waited %.0f ms", t, t >> 1, t & 1, w);
					sf_wait_track = -1;
				}
			}
		}
		EnableIO();
		spi_w(UIO_SECTOR_RD | ack);
		spi_block_write(buf, user_io_get_width(), sz);
		DisableIO();
		return 1;
	}

	// a write (the core does not write to SCP images): take it, drop it
	EnableIO();
	spi_w(UIO_SECTOR_WR | ack);
	spi_block_read(buf, user_io_get_width(), sz);
	DisableIO();
	if (!sf_write_logged)
	{
		sf_write_logged = 1;
		pf_log("the ST wrote to the disk: not written (read-only)");
	}
	return 1;
}

void pfloppy_st_flux_poll(void)
{
	if (!sf_placeholder_done)
	{
		sf_placeholder_done = 1;
		pfloppy_create_placeholder_named(HomeDir(), PFLOPPY_ST_FLUX_PLACEHOLDER);
	}
	if (sf_slot < 0) return;
	if (sf.disk_gen != sf_seen_gen)
	{
		// mount again: the core sees the new disk (or none), TOS the write-protect change
		char path[1024];
		snprintf(path, sizeof(path), "%s", sf_path);
		tos_insert_disk(sf_slot, path);
	}
}
