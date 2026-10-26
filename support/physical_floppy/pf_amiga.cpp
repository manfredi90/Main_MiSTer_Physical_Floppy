/*
 * Physical floppy for the Amiga (Minimig): a PC floppy drive on a Greaseweazle,
 * seen by the core as a flux drive (the same path as .scp/.ipf images,
 * FluxFile in minimig_fdd.h).
 *
 * The drive is read as flux and the flux is passed on as it is: the Amiga
 * decodes the disk itself, so any Amiga disk works, copy-protected ones
 * included (long tracks, timing, weak bits: several revolutions are kept and
 * played in turn).
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
#define AM_EMPTY_REV_S     0.2     // revolution time of a track without data

/* ------------------------------------------------------------------ state */

// one revolution: intervals in sample ticks, 0xFFFF = 65535 ticks without a transition
struct am_rev
{
	std::vector<uint16_t> d;
	uint32_t len;
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
	int dumped0;
} am;

// main thread: flux played to the core
static struct
{
	int track;
	am_rev_p rev;           // revolution being played, NULL = no data (no transitions)
	int rev_k;
	size_t di;              // next interval
	double cum;             // sample ticks from the index pulse to the last event
	double rev_start;       // core clock ticks of the index pulse of this revolution
	double rev_len;         // core clock ticks
	double ratio;           // core clock ticks per sample tick
	int64_t pos;            // core clock ticks scheduled so far

	int pending;            // a transition at pulse_at, its byte not written yet
	int64_t pulse_at;
	int pb;                 // pulse byte to write (-1 none)
	int64_t fill_left;      // ticks of delay to write
	int idx;                // index byte to write
	int has_b2;
	uint8_t b2;

	std::vector<uint16_t> rewind;
	uint32_t seen_data_gen;
	double wait_t0;         // the core is on a track without data since

	// diagnostics
	uint32_t calls, rewinds, data_bytes, empty_bytes, track_changes;
	std::vector<uint8_t> cap;   // first flux sent for track 0 (pfloppy_core0.bin)
	int cap_state;              // 0 waiting, 1 capturing, 2 done
	int last_cap_n;

	uint16_t my_status;     // drive status set by us
	uint32_t seen_disk_gen;
	int df_index;
} g;

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

static void store_rev(int t, const pfd_rev &rv)
{
	auto r = std::make_shared<am_rev>();
	r->d.reserve(rv.flux.size() + 16);
	for (uint32_t v : rv.flux)
	{
		while (v >= 0xFFFF) { r->d.push_back(0xFFFF); v -= 0xFFFF; }
		r->d.push_back((uint16_t)v);
	}
	r->len = rv.len;

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
		if (t == 0 && !am.dumped0)
		{
			am.dumped0 = 1;
			FILE *f = fopen("/media/fat/logs/pfloppy_t0rev.bin", "wb");
			if (f)
			{
				uint32_t hdr[2] = { revs[0].len, (uint32_t)am.hz };
				fwrite(hdr, 4, 2, f);
				fwrite(revs[0].flux.data(), 4, revs[0].flux.size(), f);
				fclose(f);
			}
		}
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
			}
		}

		usleep(5000);
	}

	if (am.drv) am_close_drive();
	return NULL;
}

/* ------------------------------------------------------------------ flux to the core */

/*
 * The core's flux drive (MiSTerFloppyVirtualFluxDrive.v) reads bytes at
 * FLUX_CLOCK_SPEED_HZ: 0 = index pulse, 1 = one tick of delay, 2 = the next
 * byte is a delay without transition, n >= 3 = a transition now, then n ticks.
 * Bytes 0, 1, 2 take one tick, a transition byte n takes n + 1 ticks.
 * Times are kept absolute, so rounding never accumulates.
 */

static double empty_rev_len() { return AM_EMPTY_REV_S * FLUX_CLOCK_SPEED_HZ; }

// called with am_lock held
static void set_ratio_locked()
{
	g.ratio = am.hz > 0 ? FLUX_CLOCK_SPEED_HZ / am.hz : 1;
}

// the next revolution of the current track (or of the same disk position if it has no data)
static void next_rev(double start)
{
	pthread_mutex_lock(&am_lock);
	set_ratio_locked();
	am_track &tr = am.tracks[g.track];
	if (tr.revs.size())
	{
		g.rev_k = (g.rev_k + 1) % tr.revs.size();
		g.rev = tr.revs[g.rev_k];
		tr.used = pf_now_ms();
	}
	else g.rev = NULL;
	pthread_mutex_unlock(&am_lock);

	g.rev_start = start;
	g.di = 0;
	g.cum = 0;
	g.rev_len = g.rev ? g.rev->len * g.ratio : empty_rev_len();
}

// switches to the data of the current track at the same angle (the disk keeps spinning)
static void switch_source()
{
	double now = (double)g.pos;
	double frac = g.rev_len > 0 ? (now - g.rev_start) / g.rev_len : 0;
	frac -= floor(frac);

	pthread_mutex_lock(&am_lock);
	set_ratio_locked();
	am_track &tr = am.tracks[g.track];
	g.rev = tr.revs.size() ? tr.revs[0] : NULL;
	tr.used = pf_now_ms();
	pthread_mutex_unlock(&am_lock);

	g.rev_k = 0;
	g.rev_len = g.rev ? g.rev->len * g.ratio : empty_rev_len();
	g.rev_start = now - frac * g.rev_len;
	g.di = 0;
	g.cum = 0;
	if (g.rev)
	{
		double target = frac * g.rev->len;
		const std::vector<uint16_t> &d = g.rev->d;
		while (g.di < d.size())
		{
			double nc = g.cum + (d[g.di] == 0xFFFF ? 65535 : d[g.di]);
			if (nc >= target) break;
			g.cum = nc;
			g.di++;
		}
	}
	g.pending = 0; // the transition in flight belonged to the other track
}

static void next_event(double *ev, int *is_index)
{
	if (g.rev)
	{
		const std::vector<uint16_t> &d = g.rev->d;
		while (g.di < d.size())
		{
			uint16_t v = d[g.di++];
			if (v == 0xFFFF) { g.cum += 65535; continue; }
			g.cum += v;
			*ev = g.rev_start + g.cum * g.ratio;
			*is_index = 0;
			return;
		}
	}
	*ev = g.rev_start + g.rev_len;
	*is_index = 1;
	next_rev(*ev);
}

// schedules the bytes up to the next event
static void gen_step()
{
	double ev;
	int is_index;
	next_event(&ev, &is_index);
	int64_t e = llround(ev);

	if (g.pending)
	{
		int64_t n = e - g.pulse_at - 1;
		if (n < 3) n = 3;
		if (n > 255) n = 255;
		g.pb = (int)n;
		g.pos = g.pulse_at + n + 1;
		g.pending = 0;
	}
	if (e > g.pos)
	{
		g.fill_left = e - g.pos;
		g.pos = e;
	}
	if (is_index)
	{
		g.idx = 1;
		g.pos += 1;
	}
	else
	{
		g.pending = 1;
		g.pulse_at = g.pos;
	}
}

static uint8_t next_byte()
{
	for (;;)
	{
		if (g.has_b2) { g.has_b2 = 0; return g.b2; }
		if (g.pb >= 0) { uint8_t b = (uint8_t)g.pb; g.pb = -1; return b; }
		if (g.fill_left > 0)
		{
			if (g.fill_left >= 5)
			{
				int64_t d = g.fill_left - 2;
				if (d > 255) d = 255;
				g.fill_left -= d + 2;
				g.b2 = (uint8_t)d;
				g.has_b2 = 1;
				return 2;
			}
			g.fill_left--;
			return 1;
		}
		if (g.idx) { g.idx = 0; return 0; }
		gen_step();
	}
}

static void gen_reset()
{
	g.track = 0;
	g.rev = NULL;
	g.rev_k = 0;
	g.di = 0;
	g.cum = 0;
	g.rev_start = 0;
	g.rev_len = empty_rev_len();
	g.ratio = 1;
	g.pos = 0;
	g.pending = 0;
	g.pb = -1;
	g.fill_left = 0;
	g.idx = 0;
	g.has_b2 = 0;
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
	am.dumped0 = 0;
	am.read_ms = 0;
	clear_tracks();

	// first look done here, so the core sees the disk at once
	if (!am.drv && am_open_drive()) am_probe();

	gen_reset();
	g.my_status = 0;
	g.seen_disk_gen = am.disk_gen;
	g.df_index = -1;
	g.calls = g.rewinds = g.data_bytes = g.empty_bytes = g.track_changes = 0;
	g.cap.clear();
	g.cap_state = 0;
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

void pfloppy_amiga_poll(void)
{
	if (!am.owner) return;

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
		d->ex_status = 0;
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
	uint8_t fluxMode() override { return FLUXMODE_RAWFLUX; }

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
		uint8_t *b = (uint8_t*)out;
		for (uint32_t i = 0; i < words * 2; i++) b[i] = 1;
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
			capture(out, words);
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

		uint8_t *b = (uint8_t*)out;
		if (g.rev) g.data_bytes += words * 2; else g.empty_bytes += words * 2;
		for (uint32_t i = 0; i < words * 2; i++) b[i] = next_byte();
		capture(out, words);
		return true;
	}

	void unFluxRead(const uint16_t *in, uint32_t words) override
	{
		g.rewind.assign(in, in + words);
		g.rewinds++;
		// not sent: drop it from the capture
		if (g.cap_state == 1 && g.last_cap_n && g.cap.size() >= (size_t)g.last_cap_n) g.cap.resize(g.cap.size() - g.last_cap_n);
		g.last_cap_n = 0;
	}

	// diagnostics: the first 1.5 s of flux sent for track 0 with data
	static void capture(const uint16_t *b, uint32_t words)
	{
		g.last_cap_n = 0;
		if (g.cap_state == 0 && g.track == 0 && g.rev) g.cap_state = 1;
		if (g.cap_state != 1) return;
		if (g.track != 0) { g.cap_state = 2; save_capture(); return; }
		const uint8_t *p = (const uint8_t*)b;
		g.cap.insert(g.cap.end(), p, p + words * 2);
		g.last_cap_n = words * 2;
		if (g.cap.size() >= 300000) { g.cap_state = 2; save_capture(); }
	}

	static void save_capture()
	{
		FILE *f = fopen("/media/fat/logs/pfloppy_core0.bin", "wb");
		if (f)
		{
			fwrite(g.cap.data(), 1, g.cap.size(), f);
			fclose(f);
		}
		pf_log("diagnostics: %zu bytes of flux sent for track 0 saved", g.cap.size());
		g.cap.clear();
		g.cap.shrink_to_fit();
	}

	uint32_t firstTrack() override { return 0; }
	uint32_t lastTrack() override { return AM_TRACKS; }
	uint32_t numHeads() override { return 2; }
};

FluxFile *pfloppy_amiga_create(void)
{
	return new AmigaLiveFlux();
}
