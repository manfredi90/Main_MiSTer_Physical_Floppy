/*
 * Physical floppy for the Amiga (Minimig): a PC floppy drive on a Greaseweazle,
 * seen by the core as a flux drive (the same path as .scp/.ipf images,
 * FluxFile in minimig_fdd.h).
 *
 * The drive is read as flux and passed on as MFM cells with their timing (the
 * density mode of the core, as IPF images): the Amiga decodes the disk
 * itself, so any Amiga disk works, copy-protected ones included (long tracks,
 * timing, weak bits: several revolutions are kept and played in turn).
 *
 * A background thread reads the track the core is on (and the other side of
 * the same cylinder), one revolution first, more if the core stays on it.
 * Revolutions are played to the core in a loop, like a spinning disk; until a
 * track is read the core gets a track without flux transitions, as from a
 * drive whose head is still settling. The head follows the steps of the core
 * and the motor follows the motor of the Amiga drive.
 *
 * Read-only: the core's flux drive cannot write.
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
#include "mfm_ibm.h"
#include "../minimig/minimig_fdd.h"
#include "../../menu.h"

#define AM_TRACKS          166     // 83 cylinders, as the core
#define AM_REVS            3       // revolutions kept per track (weak bits differ between them)
#define AM_MORE_REVS_MS    500     // the core stays on a track this long: read more revolutions
#define AM_AHEAD_LIMIT     160     // no read-ahead past cylinder 79 (the end of a normal disk)
#define AM_CACHE_TRACKS    100     // tracks kept in memory (about 80 KB per revolution)
#define AM_ACTIVE_MS       2000    // the core asked for data within this time: motor on
#define AM_MOTOR_KEEP_MS   3000    // motor kept on after a read the core did not ask for (disk just inserted)
#define AM_POLL_MS         1000    // disk presence check when the drive is idle
#define AM_ACTIVE_POLL_MS  500     // ... when the core uses the drive but nothing is read
#define AM_READ_FAILS      3       // a track that fails this many times is played empty
#define AM_SLOW_MS         1000    // log the tracks the core waited longer than this
#define AM_STATS_MS        10000

/* ------------------------------------------------------------------ state */

// one revolution, as the core's flux drive takes it in density mode (as IPF
// images): per word, 8 MFM cells (high byte, MSB first, 1 = flux transition)
// and the length of those cells in core clock ticks minus one (low byte)
struct am_rev
{
	std::vector<uint16_t> w;
};
typedef std::shared_ptr<const am_rev> am_rev_p;

struct am_track
{
	std::vector<am_rev_p> revs;
	int fails;
	int dead;      // unreadable or out of reach: played without flux
	double used;
};

static pthread_mutex_t am_lock = PTHREAD_MUTEX_INITIALIZER;

// shared by the main thread and the I/O thread
static struct
{
	pthread_t thread;
	int thread_running;
	std::atomic<int> alive;

	FluxFile *owner;            // the FluxFile using the drive, NULL if none
	PfFluxDrive *drv;           // open drive (I/O thread only)
	double hz;                  // sample clock of the drive

	am_track tracks[AM_TRACKS]; // under am_lock
	std::atomic<uint32_t> data_gen;  // a track got data
	std::atomic<int> present;        // disk in the drive
	std::atomic<uint32_t> disk_gen;  // incremented on every disk insertion
	std::atomic<int> want;           // track the core is on
	std::atomic<int> seq;            // the core reads the tracks in order
	std::atomic<int> want_t;         // when it got there (ms, low 31 bits)
	std::atomic<int> play_t;         // when the core started getting its data (-1 not yet)
	std::atomic<int> last_core;      // last data request from the core (ms)
	std::atomic<int> prefetch0;      // read track 0 (disk just inserted)

	char last_why[160];
	uint32_t reads, revs_read, cache_hits;
	double read_ms;
	double last_read;           // last read from the drive (ms)
} am;

// main thread: flux played to the core
static struct
{
	int track;
	am_rev_p rev;           // revolution being played, NULL = no data (no transitions)
	int rev_k;
	size_t wi;              // next word of the revolution
	size_t empty_left;      // words left of a revolution without data
	int idx;                // the index word comes next

	std::vector<uint16_t> rewind;
	uint32_t seen_data_gen;
	double wait_t0;         // the core is on a track without data since

	// diagnostics
	uint32_t calls, rewinds, data_bytes, empty_bytes, track_changes;

	// main loop timing while the core streams flux: the core buffers only
	// 9 to 18 ms of flux, a longer pause of the main loop empties it
	double last_poll_ms;
	uint32_t gaps5, gaps9, gaps16;
	double gap_max;

	uint16_t my_status;     // drive status set by us
	uint32_t seen_disk_gen;
	int df_index;
} g;

// AmigaDOS mode (below)
static int dos_df = -1;           // DFx using the drive, -1 none
static uint16_t dos_my_status;    // drive status set by us

static int now_ms31() { return (int)((long long)pf_now_ms() & 0x7FFFFFFF); }
static int ms_since(int t) { return (now_ms31() - t) & 0x7FFFFFFF; }

int pfloppy_is_amiga_placeholder(const char *path)
{
	if (!path || !path[0]) return 0;
	const char *p = strrchr(path, '/');
	p = p ? p + 1 : path;
	return !strcasecmp(p, PFLOPPY_AMIGA_PLACEHOLDER);
}

/* ------------------------------------------------------------------ I/O thread */

static void clear_tracks()
{
	pthread_mutex_lock(&am_lock);
	for (int t = 0; t < AM_TRACKS; t++)
	{
		am.tracks[t].revs.clear();
		am.tracks[t].fails = 0;
		am.tracks[t].dead = 0;
	}
	pthread_mutex_unlock(&am_lock);
	am.data_gen++;
}

/*
 * Flux -> density words: a PLL (as the Greaseweazle tools) gives the MFM
 * cells and their length; the length of each group of 8 cells becomes the
 * density of that word. The lengths are scaled so that the revolution keeps
 * its measured time, and rounded with the error carried on, so the timing of
 * long stretches (long tracks, Copylock sectors written slower or faster) is
 * kept. The core's flux drive takes density + 1 ticks per cell.
 */
static void flux_to_density(const pfd_rev &rv, double hz, std::vector<uint16_t> &w)
{
	const double cell_s = 2e-6;
	const double PERIOD_ADJ = 0.05, PHASE_ADJ = 0.60, RANGE = 0.10;
	double clock = cell_s, cmin = cell_s * (1 - RANGE), cmax = cell_s * (1 + RANGE);
	double ticks = 0;

	std::vector<uint8_t> bits;
	std::vector<float> len;           // length of each cell (s)
	bits.reserve(rv.flux.size() * 3);
	len.reserve(rv.flux.size() * 3);

	for (uint32_t x : rv.flux)
	{
		ticks += x / hz;
		if (ticks < clock / 2) continue;
		int zeros = 0;
		for (;;)
		{
			ticks -= clock;
			if (ticks < clock / 2) break;
			zeros++;
			bits.push_back(0);
			len.push_back((float)clock);
		}
		bits.push_back(1);
		len.push_back((float)clock);
		double nt = ticks * (1.0 - PHASE_ADJ);
		if (zeros <= 3) clock += ticks * PERIOD_ADJ;
		else clock += (cell_s - clock) * PERIOD_ADJ;
		if (clock < cmin) clock = cmin;
		else if (clock > cmax) clock = cmax;
		ticks = nt;
	}

	size_t nbytes = bits.size() / 8;
	w.clear();
	if (!nbytes) return;

	double total = 0;
	for (size_t i = 0; i < nbytes * 8; i++) total += len[i];
	double scale = (rv.len / hz) / total;

	w.reserve(nbytes);
	double carry = 0;
	for (size_t b = 0; b < nbytes; b++)
	{
		uint8_t m = 0;
		double sum = 0;
		for (int k = 0; k < 8; k++)
		{
			m = (m << 1) | bits[b * 8 + k];
			sum += len[b * 8 + k];
		}
		double exact = sum / 8 * scale * FLUX_CLOCK_SPEED_HZ - 1 + carry;
		long d = lround(exact);
		if (d < 3) d = 3;
		if (d > 255) d = 255;
		carry = exact - d;
		w.push_back((uint16_t)((m << 8) | d));
	}
}

static void store_rev(int t, const pfd_rev &rv)
{
	auto r = std::make_shared<am_rev>();
	flux_to_density(rv, am.hz, r->w);
	if (r->w.empty()) return;

	pthread_mutex_lock(&am_lock);
	am_track &tr = am.tracks[t];
	if ((int)tr.revs.size() >= AM_REVS) tr.revs.erase(tr.revs.begin());
	tr.revs.push_back(r);
	tr.used = pf_now_ms();

	// forget the least recently used tracks
	int n = 0;
	for (int i = 0; i < AM_TRACKS; i++) if (am.tracks[i].revs.size()) n++;
	while (n > AM_CACHE_TRACKS)
	{
		int lru = -1;
		for (int i = 0; i < AM_TRACKS; i++)
		{
			if (i == t || i == am.want || !am.tracks[i].revs.size()) continue;
			if (lru < 0 || am.tracks[i].used < am.tracks[lru].used) lru = i;
		}
		if (lru < 0) break;
		am.tracks[lru].revs.clear();
		n--;
	}
	pthread_mutex_unlock(&am_lock);
	am.data_gen++;
}

static void am_close_drive()
{
	if (am.drv)
	{
		am.drv->close();
		delete am.drv;
		am.drv = NULL;
	}
	if (am.present) pf_log("disk gone (drive closed)");
	am.present = 0;
	clear_tracks();
}

static int am_open_drive()
{
	PfFluxDrive *d = pfd_gw_flux_create();
	char desc[128];
	if (!d->open(desc, sizeof(desc)))
	{
		delete d;
		if (strcmp(am.last_why, "no greaseweazle"))
		{
			snprintf(am.last_why, sizeof(am.last_why), "no greaseweazle");
			pf_log("no Greaseweazle found");
		}
		return 0;
	}
	am.drv = d;
	pthread_mutex_lock(&am_lock);
	am.hz = d->sample_hz();
	pthread_mutex_unlock(&am_lock);
	am.last_why[0] = 0;
	pf_log("found %s", desc);
	return 1;
}

static void am_probe()
{
	char why[160] = "";
	int r = am.drv->flux_probe(am.present, why, sizeof(why));
	switch (r)
	{
	case PFD_PROBE_NEW:
		if (am.present) pf_log("disk changed");
		else pf_log("disk inserted");
		clear_tracks();
		am.disk_gen++;
		am.present = 1;
		am.prefetch0 = 1;
		am.last_why[0] = 0;
		break;

	case PFD_PROBE_NODISK:
		if (am.present)
		{
			pf_log("disk removed");
			am.present = 0;
			clear_tracks();
		}
		if (strcmp(why, am.last_why))
		{
			snprintf(am.last_why, sizeof(am.last_why), "%s", why);
			pf_log("%s", why);
		}
		break;

	case PFD_PROBE_LOST:
		pf_log("Greaseweazle lost: %s", why);
		am_close_drive();
		break;

	default:
		break;
	}
}

static void log_density(const pfd_rev &rv)
{
	double cell = mfm_guess_cell_ns(rv.flux, am.hz);
	pf_log("track 0: %s, revolution %.1f ms", cell == 2000 ? "DD disk" :
		cell == 1000 ? "HD disk (the Amiga reads only DD disks)" : "no MFM data (unformatted?)",
		rv.len * 1000.0 / am.hz);
}

static void am_read_track(int t, int first)
{
	std::vector<pfd_rev> revs;
	double t0 = pf_now_ms();
	int r = am.drv->read_revs(t >> 1, t & 1, 1, revs);
	double ms = pf_now_ms() - t0;
	am.last_read = pf_now_ms();

	switch (r)
	{
	case PFD_OK:
		if (first) log_density(revs[0]);
		store_rev(t, revs[0]);
		am.reads++;
		am.revs_read++;
		am.read_ms += ms;
		break;

	case PFD_CHANGED:
		pf_log("disk removed (track %d)", t);
		am.present = 0;
		clear_tracks();
		break;

	case PFD_MEDIUM:
		pthread_mutex_lock(&am_lock);
		am.tracks[t].dead = 1;
		pthread_mutex_unlock(&am_lock);
		pf_log("track %d: cylinder %d out of reach of the drive, played empty", t, t >> 1);
		am.data_gen++;
		break;

	case PFD_NOT_READY:
	{
		pthread_mutex_lock(&am_lock);
		int f = ++am.tracks[t].fails;
		if (f >= AM_READ_FAILS) am.tracks[t].dead = 1;
		pthread_mutex_unlock(&am_lock);
		if (f >= AM_READ_FAILS)
		{
			pf_log("track %d: unreadable, played empty", t);
			am.data_gen++;
		}
		break;
	}

	case PFD_LOST:
	default:
		pf_log("Greaseweazle lost while reading track %d", t);
		am_close_drive();
		break;
	}
}

// the next read: the track the core is on, the other side of the same
// cylinder (the Amiga reads both sides before stepping), the next two tracks
// when the core reads the disk in order, then more revolutions if the core
// keeps reading the same track (a protection check, or a read error retried)
static int next_job()
{
	int w = am.want;
	if (w < 0 || w >= AM_TRACKS) return -1;
	int job = -1;

	pthread_mutex_lock(&am_lock);
	auto need = [](int t) { return t >= 0 && t < AM_TRACKS && !am.tracks[t].dead && am.tracks[t].revs.empty(); };
	if (need(w)) job = w;
	else if (need(w ^ 1)) job = w ^ 1;
	else if (am.seq && w + 1 < AM_AHEAD_LIMIT && need(w + 1)) job = w + 1;
	else if (am.seq && w + 2 < AM_AHEAD_LIMIT && need(w + 2)) job = w + 2;
	else if (!am.tracks[w].dead && (int)am.tracks[w].revs.size() < AM_REVS && am.play_t >= 0 && ms_since(am.play_t) > AM_MORE_REVS_MS) job = w;
	pthread_mutex_unlock(&am_lock);
	return job;
}

static void *am_worker(void *)
{
	double last_poll = 0, last_scan = -1e9, last_stats = pf_now_ms();
	int head_cyl = -1;
	uint32_t logged_reads = 0;

	while (am.alive)
	{
		double now = pf_now_ms();

		if (!am.drv)
		{
			head_cyl = -1;
			if (now - last_scan > 2000)
			{
				last_scan = now;
				if (am_open_drive()) am_probe();
			}
			usleep(50000);
			continue;
		}

		int active = ms_since(am.last_core) < AM_ACTIVE_MS;

		if (am.present && am.prefetch0)
		{
			am.prefetch0 = 0;
			am_read_track(0, 1);
			head_cyl = 0;
			continue;
		}

		if (am.present && active)
		{

			int job = next_job();
			if (job >= 0)
			{
				am_read_track(job, 0);
				head_cyl = job >> 1;
				continue;
			}

			// everything is read: the head still follows the core
			int w = am.want;
			if (w >= 0 && w < AM_TRACKS && (w >> 1) != head_cyl)
			{
				int r = am.drv->flux_seek(w >> 1);
				if (r == PFD_LOST) { am_close_drive(); continue; }
				head_cyl = w >> 1;
			}
		}
		else if (am.drv->flux_motor_is_on() && !active && now - am.last_read > AM_MOTOR_KEEP_MS)
		{
			am.drv->flux_motor_off();
		}

		if (now - last_poll >= ((am.present && active) ? AM_ACTIVE_POLL_MS : AM_POLL_MS))
		{
			last_poll = now;
			am_probe();
			if (!am.drv) continue;
		}

		if (now - last_stats >= AM_STATS_MS)
		{
			last_stats = now;
			if (am.reads != logged_reads)
			{
				logged_reads = am.reads;
				pf_log("stats: %u track reads, %.0f ms each on average", am.reads, am.reads ? am.read_ms / am.reads : 0.0);
			}
			if (g.calls)
			{
				pf_log("core: %u flux requests (%u given back), %u KB with data, %u KB empty, %u track changes, on track %d",
					g.calls, g.rewinds, g.data_bytes >> 10, g.empty_bytes >> 10, g.track_changes, g.track);
				pf_log("main loop pauses while streaming: %u over 5 ms, %u over 9 ms, %u over 16 ms, longest %.1f ms",
					g.gaps5, g.gaps9, g.gaps16, g.gap_max);
			}
		}

		usleep(5000);
	}

	if (am.drv) am_close_drive();
	return NULL;
}

/* ------------------------------------------------------------------ flux to the core */

/*
 * Density mode of the core's flux drive (MiSTerFloppyVirtualFluxDrive.v), the
 * one IPF images use: a word = 8 cells, 0 = index pulse. A word lasts about
 * 16 us, so the core's buffer (1024 to 2048 words) holds 16 to 32 ms: twice
 * the margin of the raw flux mode, where a word holds two transitions.
 */

#define AM_EMPTY_WORD   0x0039   // 8 cells without transitions, 2 us each
#define AM_EMPTY_WORDS  12500    // a revolution without data (200 ms)

// the next revolution of the current track
static void next_rev()
{
	pthread_mutex_lock(&am_lock);
	am_track &tr = am.tracks[g.track];
	if (tr.revs.size())
	{
		g.rev_k = (g.rev_k + 1) % tr.revs.size();
		g.rev = tr.revs[g.rev_k];
		tr.used = pf_now_ms();
	}
	else g.rev = NULL;
	pthread_mutex_unlock(&am_lock);
	g.wi = 0;
	g.empty_left = AM_EMPTY_WORDS;
}

// angle of the disk in the revolution being played (0..1)
static double play_angle()
{
	if (g.rev && g.rev->w.size()) return (double)g.wi / g.rev->w.size();
	return (double)(AM_EMPTY_WORDS - g.empty_left) / AM_EMPTY_WORDS;
}

// switches to the data of the current track at the same angle (the disk keeps spinning)
static void switch_source()
{
	double a = play_angle();

	pthread_mutex_lock(&am_lock);
	am_track &tr = am.tracks[g.track];
	g.rev = tr.revs.size() ? tr.revs[0] : NULL;
	tr.used = pf_now_ms();
	pthread_mutex_unlock(&am_lock);

	g.rev_k = 0;
	if (g.rev) g.wi = (size_t)(a * g.rev->w.size());
	g.empty_left = (size_t)((1 - a) * AM_EMPTY_WORDS);
}

static uint16_t next_word()
{
	if (g.idx)
	{
		g.idx = 0;
		return 0;
	}
	if (g.rev)
	{
		if (g.wi < g.rev->w.size()) return g.rev->w[g.wi++];
	}
	else if (g.empty_left)
	{
		g.empty_left--;
		return AM_EMPTY_WORD;
	}
	// end of the revolution: index pulse, then the next one
	next_rev();
	return 0;
}

static void gen_reset()
{
	g.track = 0;
	g.rev = NULL;
	g.rev_k = 0;
	g.wi = 0;
	g.empty_left = AM_EMPTY_WORDS;
	g.idx = 0;
	g.rewind.clear();
	g.seen_data_gen = am.data_gen;
	g.wait_t0 = 0;
}

/* ------------------------------------------------------------------ attach */

static int am_attach(FluxFile *f)
{
	if (am.owner == f) return 1;
	if (am.owner)
	{
		pf_log("the drive is already inserted in another Amiga floppy drive");
		return 0;
	}

	if (dos_df >= 0)
	{
		int o = dos_df;
		pfloppy_amiga_dos_release(o);
		df[o].status = 0;
	}

	pf_log_reset();
	pf_log("attached to the Amiga floppy (flux, read-only)");

	am.present = 0;
	am.want = 0;
	am.seq = 0;
	am.play_t = -1;
	am.want_t = now_ms31();
	am.last_core = (now_ms31() - AM_ACTIVE_MS - 1) & 0x7FFFFFFF;
	am.prefetch0 = 0;
	am.last_why[0] = 0;
	am.reads = am.revs_read = am.cache_hits = 0;
	am.read_ms = 0;
	clear_tracks();

	// first look done here, so the core sees the disk at once
	if (!am.drv && am_open_drive()) am_probe();

	gen_reset();
	g.my_status = 0;
	g.seen_disk_gen = am.disk_gen;
	g.df_index = -1;
	g.calls = g.rewinds = g.data_bytes = g.empty_bytes = g.track_changes = 0;
	g.last_poll_ms = 0;
	g.gaps5 = g.gaps9 = g.gaps16 = 0;
	g.gap_max = 0;
	am.owner = f;

	am.alive = 1;
	if (pthread_create(&am.thread, NULL, am_worker, NULL))
	{
		pf_log("cannot start the I/O thread");
		am.alive = 0;
		if (am.drv) am_close_drive();
		am.owner = NULL;
		return 0;
	}
	am.thread_running = 1;
	return 1;
}

static void am_detach(FluxFile *f)
{
	if (!f || am.owner != f) return;
	am.alive = 0;
	if (am.thread_running) pthread_join(am.thread, NULL);
	am.thread_running = 0;
	if (am.drv) am_close_drive();
	am.owner = NULL;
	g.rev = NULL;
	g.my_status = 0;
	pf_log("detached");
}

/* ------------------------------------------------------------------ AmigaDOS mode */

// the drive through the sector engine (physical_floppy.cpp), the disk given to
// the core as an ADF: the core's normal floppy emulation, writable


int pfloppy_is_amiga_dos_placeholder(const char *path)
{
	if (!path || !path[0]) return 0;
	const char *p = strrchr(path, '/');
	p = p ? p + 1 : path;
	return !strcasecmp(p, PFLOPPY_AMIGA_DOS_PLACEHOLDER);
}

static uint16_t dos_status()
{
	pfloppy_media_t m;
	pfloppy_get_media(&m);
	if (!m.present) return 0;
	return DSK_INSERTED | (m.write_protected ? 0 : DSK_WRITABLE);
}

void pfloppy_amiga_dos_release(int dfx)
{
	if (dfx < 0 || dfx > 3) return;
	df[dfx].physical = 0;
	if (dos_df != dfx) return;
	dos_df = -1;
	pfloppy_detach();
}

void pfloppy_amiga_dos_insert(int dfx)
{
	if (dfx < 0 || dfx > 3) return;
	// the flux mode or another DFx had the drive
	if (am.owner) am.owner->closeFile();
	if (dos_df >= 0 && dos_df != dfx)
	{
		int o = dos_df;
		pfloppy_amiga_dos_release(o);
		df[o].status = 0;
	}

	pfloppy_set_format(PFLOPPY_FORMAT_AMIGA);
	pfloppy_attach(dfx); // looks at the disk before returning
	pfloppy_media_event();
	pf_log("AmigaDOS mode in DF%d (the disk is given to the core as an ADF, writable)", dfx);

	adfTYPE *d = &df[dfx];
	d->physical = 1;
	d->tracks = 160;
	d->ex_status = 0;
	d->sector_offset = 0;
	d->lastBit = 0xAAAA;
	d->track = 0;
	d->track_prev = -1;
	snprintf(d->name, sizeof(d->name), "%s", PFLOPPY_AMIGA_DOS_PLACEHOLDER);
	d->status = dos_status();
	dos_my_status = d->status;
	dos_df = dfx;
}

static void dos_poll()
{
	if (dos_df < 0) return;
	adfTYPE *d = &df[dos_df];
	if (!d->physical) { dos_df = -1; return; }

	if (d->status != dos_my_status)
	{
		// changed from the menu (eject, configuration reset)
		if (!(d->status & DSK_INSERTED)) pfloppy_amiga_dos_release(dos_df);
		return;
	}

	if (pfloppy_media_event() && (d->status & DSK_INSERTED))
	{
		// disk removed, changed or inserted: the core sees a removal first
		d->status = 0;
		dos_my_status = 0;
		return;
	}

	uint16_t st = dos_status();
	if (st != d->status)
	{
		pf_log("core: DF%d %s", dos_df, st ? (st & DSK_WRITABLE ? "disk in" : "disk in, write-protected") : "no disk");
		d->status = st;
		d->track_prev = -1;
	}
	dos_my_status = d->status;
}

void pfloppy_amiga_poll(void)
{
	dos_poll();
	if (!am.owner) return;

	double t = pf_now_ms();
	if (g.last_poll_ms > 0 && ms_since(am.last_core) < 50)
	{
		double gap = t - g.last_poll_ms;
		if (gap > 5) g.gaps5++;
		if (gap > 9) g.gaps9++;
		if (gap > 16) g.gaps16++;
		if (gap > g.gap_max) g.gap_max = gap;
	}
	g.last_poll_ms = t;

	int i;
	for (i = 0; i < 4; i++) if (df[i].fluxFile == am.owner) break;
	if (i == 4) return;
	adfTYPE *d = &df[i];

	if (g.df_index != i)
	{
		// just inserted by InsertFloppy
		g.df_index = i;
		g.my_status = d->status;
		pf_log("core: in DF%d, status %X, ex %X, tracks %d", i, d->status, d->ex_status, d->tracks);
		// the flux drive came with the real floppy drive support of the core
		// (September 2026), the bit that tells it with the zero drives mode
		if (!floppy_zero_supported)
		{
			pf_log("WARNING: this Minimig core seems too old for flux disks (no virtual flux drive): it needs a core from October 2026 or newer (unstable builds)");
			Info("Minimig core too old for\nthe Greaseweazle: update it", 6000);
		}
	}

	if (d->status != g.my_status)
	{
		// changed from the menu (eject, configuration reset)
		if (!(d->status & DSK_INSERTED)) am.owner->closeFile();
		return;
	}

	int ins = (d->status & DSK_INSERTED) != 0;
	uint32_t gen = am.disk_gen;
	if (gen != g.seen_disk_gen)
	{
		g.seen_disk_gen = gen;
		if (ins)
		{
			// a disk swapped between two checks: show a removal first
			d->status = 0;
			g.my_status = 0;
			return;
		}
	}

	int pres = am.present;
	if (pres && !ins)
	{
		d->status = DSK_INSERTED | DSK_FLUXMODE;
		d->ex_status = DSKEX_FLUXDENSITY;
		pf_log("core: DF%d disk in", i);
	}
	else if (!pres && ins)
	{
		d->status = 0;
		pf_log("core: DF%d no disk", i);
	}
	g.my_status = d->status;
}

/* ------------------------------------------------------------------ FluxFile */

class AmigaLiveFlux : public FluxFile
{
protected:
	bool _openFile(const char *filename) override { (void)filename; return am_attach(this); }
	void _closeFile() override { am_detach(this); }

public:
	~AmigaLiveFlux() override { closeFile(); }

	bool fluxReady() override { return am.owner == this && (g.my_status & DSK_INSERTED); }
	uint8_t fluxMode() override { return FLUXMODE_DENSITY; }

	bool selectTrack(uint32_t track) override
	{
		if (track >= AM_TRACKS) track = AM_TRACKS - 1;
		if ((int)track == g.track) return true;
		if (++g.track_changes <= 60) pf_log("core: track %d -> %d", g.track, track);
		am.seq = (int)track == g.track + 1;
		g.track = track;
		g.rewind.clear();
		am.want = track;
		am.want_t = now_ms31();
		switch_source();
		g.seen_data_gen = am.data_gen;
		g.wait_t0 = g.rev ? 0 : pf_now_ms();
		am.play_t = g.rev ? now_ms31() : -1;
		if (g.rev) am.cache_hits++;
		return true;
	}

	bool fluxDummyRead(uint16_t *out, uint32_t words) override
	{
		// no transitions
		for (uint32_t i = 0; i < words; i++) out[i] = AM_EMPTY_WORD;
		return true;
	}

	bool fluxRead(uint16_t *out, uint32_t words) override
	{
		am.last_core = now_ms31();
		if (am.want != g.track) am.want = g.track;
		g.calls++;

		if (g.rewind.size() == words)
		{
			memcpy(out, g.rewind.data(), words * 2);
			g.rewind.clear();
			return true;
		}
		g.rewind.clear();

		if (!g.rev && am.data_gen != g.seen_data_gen)
		{
			g.seen_data_gen = am.data_gen;
			switch_source();
			if (g.rev) am.play_t = now_ms31();
			if (g.rev && g.wait_t0)
			{
				double w = pf_now_ms() - g.wait_t0;
				if (w > AM_SLOW_MS) pf_log("track %d: the Amiga waited %.0f ms", g.track, w);
				g.wait_t0 = 0;
			}
		}

		if (g.rev) g.data_bytes += words * 2; else g.empty_bytes += words * 2;
		for (uint32_t i = 0; i < words; i++) out[i] = next_word();
		return true;
	}

	void unFluxRead(const uint16_t *in, uint32_t words) override
	{
		g.rewind.assign(in, in + words);
		g.rewinds++;
	}


	uint32_t firstTrack() override { return 0; }
	uint32_t lastTrack() override { return AM_TRACKS; }
	uint32_t numHeads() override { return 2; }
};

FluxFile *pfloppy_amiga_create(void)
{
	return new AmigaLiveFlux();
}
