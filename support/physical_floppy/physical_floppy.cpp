/*
 * Physical floppy support for MiSTer Main.
 * See physical_floppy.h for an overview.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <dirent.h>
#include <pthread.h>
#include <time.h>
#include <sys/stat.h>

#include <string>

#include "physical_floppy.h"
#include "pf_drive.h"
#include "../../file_io.h"
#include "../../cfg.h"

#define LOG_PATH          "/tmp/pfloppy.log"
#define SD_LOG_DIR        "/media/fat/logs"
#define SD_LOG_PATH       SD_LOG_DIR "/pfloppy.log"
#define SD_PREV_LOG_PATH  SD_LOG_DIR "/pfloppy_prev.log"
#define MAX_SECTORS       5760   // 2.88MB
#define MAX_SPT           36
#define MAX_CHUNK         MAX_SPT // reads are done one track at a time
#define READAHEAD_TRACKS  2      // tracks read in advance after the one in use
#define SLOW_WAIT_MS      800    // log every sector the core waited longer than this

// MiSTer.ini PFLOPPY_PRELOAD=1: after a disk is inserted read the whole disk in
// the background, so every later request is served from the cache. For cores
// whose BIOS gives up before a USB drive can spin up. Default off: the drive
// reads only what the core asks for (plus a little read-ahead), like a real drive.
#define PFLOPPY_BACKGROUND_FILL (cfg.pfloppy_preload)

// keep the motor spinning this long after the last access from the core,
// like the motor-off delay of a PC: a burst of accesses doesn't pay a spin-up each time
#define MOTOR_KEEP_MS     PF_MOTOR_KEEP_MS
#define MOTOR_TICK_MS     700    // a USB drive stops its motor after about a second idle
#define ABANDON_MS        1000   // the core stopped asking for a pending sector
#define RESCAN_MS         2000   // drive search when no drive is open
#define PENDING_GIVEUP_MS 15000  // answer with zeros if a sector never arrives
#define STATS_MS          5000
#define WRITE_RETRIES     3
#define WRITE_SETTLE_MS   300    // a track being written (the Amiga sends its sectors one by one
                                 // over a revolution) is written to the disk once complete

enum { ST_INVALID = 0, ST_VALID = 1, ST_BAD = 2 };

static struct
{
	pthread_mutex_t lock;
	pthread_cond_t cond;
	pthread_t thread;
	int thread_running;
	volatile int alive;

	int unit;
	PfDrive *drv;            // open drive, NULL if none
	PfDrive *usb, *gw;
	char desc[96];
	char last_why[160];

	pfloppy_media_t media;
	pfd_geometry geom;
	uint32_t gen;            // incremented on every disk change
	volatile int event;
	volatile int recheck;    // thread should probe the disk as soon as possible

	uint8_t *data;
	uint8_t state[MAX_SECTORS];
	uint8_t dirty[MAX_SECTORS];
	uint8_t wretry[MAX_SECTORS];
	int dirty_count;
	int last_write_track;    // track of the last sector written by the core
	double last_write_ms;

	int want_lba;
	double want_t0;
	int last_chunk;          // track in use (reads are done one track at a time)
	int ra_next;
	int ra_left;
	int fill_next;           // background fill of the whole disk: next track to check
	double fill_t0;
	int fill_logged;

	uint32_t hits, misses, bad, lost_writes, written;
	double last_call_ms;     // last request from the core
	double last_access_ms;   // last read/write from the core (motor keep-alive)
	double last_io_ms;       // last read/write on the drive
	double last_keep_ms;     // last keep-alive command
	double worst_wait;
	double last_stats;
} pf = {
	PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, 0, 0, 0,
	-1, NULL, NULL, NULL, "", "",
	{}, {}, 0, 0, 0,
	NULL, {}, {}, {}, 0, -1, 0,
	-1, 0, -1, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

double pf_now_ms()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

#define now_ms pf_now_ms
#define plog pf_log

/*
 * The log is written by its own thread: the callers include the main loop of
 * MiSTer (requests from the core), which must never wait for the SD card or
 * the serial console. A Minimig flux drive has only a few milliseconds of
 * flux buffered in the core: a log line written in place could empty it.
 *
 * Always in /tmp (memory). On the SD card (logs/pfloppy.log, readable from the
 * network share) only with MiSTer.ini PFLOPPY_LOG=1, and in blocks: the lines
 * are kept and appended at most every SD_LOG_FLUSH_MS, to spare the card.
 */
#define SD_LOG_FLUSH_MS 5000
static struct
{
	pthread_mutex_t lock;
	pthread_cond_t cond;
	int started;
	int busy;
	int flush_req;       // pf_log_flush: write the SD buffer now
	char *q[256];        // NULL entry = log reset
	int head, tail;      // q[head..tail)
	uint32_t dropped;
} lg = { PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, 0, 0, 0, {}, 0, 0, 0 };

// lines not on the SD card yet (log thread only)
static std::string sd_buf;
static double sd_last_flush = 0;

static void sd_flush()
{
	sd_last_flush = pf_now_ms();
	if (sd_buf.empty()) return;
	FILE *f = fopen(SD_LOG_PATH, "a");
	if (f)
	{
		fwrite(sd_buf.data(), 1, sd_buf.size(), f);
		fclose(f);
	}
	sd_buf.clear();
}

static void log_write(const char *line)
{
	if (!line)
	{
		remove(LOG_PATH);
		if (cfg.pfloppy_log)
		{
			// the log of the previous core session is kept as pfloppy_prev.log
			sd_flush();
			if (!access("/media/fat", F_OK)) mkdir(SD_LOG_DIR, 0755);
			rename(SD_LOG_PATH, SD_PREV_LOG_PATH);
		}
		return;
	}
	printf("PFLOPPY: %s\n", line + 13);
	FILE *f = fopen(LOG_PATH, "a");
	if (f)
	{
		fprintf(f, "%s\n", line);
		fclose(f);
	}
	if (cfg.pfloppy_log)
	{
		sd_buf += line;
		sd_buf += '\n';
		if (sd_buf.size() > 16384 || pf_now_ms() - sd_last_flush >= SD_LOG_FLUSH_MS) sd_flush();
	}
}

static void *log_thread(void *)
{
	pthread_mutex_lock(&lg.lock);
	for (;;)
	{
		while (lg.head == lg.tail)
		{
			if (lg.flush_req || (!sd_buf.empty() && pf_now_ms() - sd_last_flush >= SD_LOG_FLUSH_MS))
			{
				lg.busy = 1;
				pthread_mutex_unlock(&lg.lock);
				sd_flush();
				pthread_mutex_lock(&lg.lock);
				lg.busy = 0;
				lg.flush_req = 0;
				pthread_cond_broadcast(&lg.cond);
				continue;
			}
			if (sd_buf.empty()) pthread_cond_wait(&lg.cond, &lg.lock);
			else
			{
				// the buffered lines go to the SD card when the flush time comes
				double due = sd_last_flush + SD_LOG_FLUSH_MS - pf_now_ms();
				if (due < 1) due = 1;
				struct timespec ts;
				clock_gettime(CLOCK_REALTIME, &ts);
				long long ns = ts.tv_nsec + (long long)(due * 1e6);
				ts.tv_sec += ns / 1000000000;
				ts.tv_nsec = ns % 1000000000;
				pthread_cond_timedwait(&lg.cond, &lg.lock, &ts);
			}
		}
		char *line = lg.q[lg.head % 256];
		lg.head++;
		lg.busy = 1;
		pthread_mutex_unlock(&lg.lock);
		log_write(line);
		free(line);
		pthread_mutex_lock(&lg.lock);
		lg.busy = 0;
		pthread_cond_broadcast(&lg.cond);
	}
	return NULL;
}

static void log_push(char *line)
{
	pthread_mutex_lock(&lg.lock);
	if (!lg.started)
	{
		pthread_t t;
		if (!pthread_create(&t, NULL, log_thread, NULL))
		{
			pthread_detach(t);
			lg.started = 1;
		}
	}
	if (!lg.started)
	{
		pthread_mutex_unlock(&lg.lock);
		log_write(line);
		free(line);
		return;
	}
	if (lg.tail - lg.head >= 256)
	{
		lg.dropped++;
		free(line);
	}
	else
	{
		lg.q[lg.tail % 256] = line;
		lg.tail++;
		pthread_cond_broadcast(&lg.cond);
	}
	pthread_mutex_unlock(&lg.lock);
}

void pf_log(const char *fmt, ...)
{
	char msg[512];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(msg, sizeof(msg), fmt, ap);
	va_end(ap);

	char *line = (char*)malloc(strlen(msg) + 16);
	if (!line) return;
	sprintf(line, "[%10.3f] %s", now_ms() / 1000.0, msg);
	log_push(line);
}

// a new log file the first time the drive is used: one log per core session
// (a Mac disk in another slot, an Amiga disk change: the log goes on)
void pf_log_reset()
{
	static int done = 0;
	if (done) return;
	done = 1;
	log_push(NULL);
}

// waits until everything logged so far is written, on the SD card too (tests)
void pf_log_flush()
{
	pthread_mutex_lock(&lg.lock);
	while (lg.started && (lg.head != lg.tail || lg.busy)) pthread_cond_wait(&lg.cond, &lg.lock);
	if (lg.started)
	{
		lg.flush_req = 1;
		pthread_cond_broadcast(&lg.cond);
		while (lg.flush_req || lg.busy) pthread_cond_wait(&lg.cond, &lg.lock);
	}
	pthread_mutex_unlock(&lg.lock);
}

/* ------------------------------------------------------------------ geometry */

void pf_calc_geometry(int total, int *spt, int *heads, int *cyl)
{
	switch (total)
	{
	case 5760: *spt = 36; *heads = 2; *cyl = 80; break; // 2.88M
	case 3360: *spt = 21; *heads = 2; *cyl = 80; break; // 1.68M DMF
	case 2880: *spt = 18; *heads = 2; *cyl = 80; break; // 1.44M
	case 2400: *spt = 15; *heads = 2; *cyl = 80; break; // 1.2M
	case 1440: *spt = 9;  *heads = 2; *cyl = 80; break; // 720K
	case 720:  *spt = 9;  *heads = 2; *cyl = 40; break; // 360K
	case 640:  *spt = 8;  *heads = 2; *cyl = 40; break; // 320K
	case 360:  *spt = 9;  *heads = 1; *cyl = 40; break; // 180K
	case 320:  *spt = 8;  *heads = 1; *cyl = 40; break; // 160K
	default:
		*spt = 18; *heads = 2;
		*cyl = (total + 35) / 36;
		break;
	}
}

/* ------------------------------------------------------------------ drive */

// MiSTer.ini PFLOPPY_DRIVE: 0 = any (Greaseweazle first), 1 = USB floppy drive only, 2 = Greaseweazle only
static int pf_format = PFLOPPY_FORMAT_PC;

static int open_drive()
{
	if (!pf.usb) pf.usb = pfd_usb_create();
	if (!pf.gw) pf.gw = pf_format == PFLOPPY_FORMAT_AMIGA ? pfd_gw_create_amiga() : pf_format == PFLOPPY_FORMAT_MAC ? pfd_gw_create_mac(0) : pf_format == PFLOPPY_FORMAT_MAC_SD ? pfd_gw_create_mac(1) : pfd_gw_create();

	PfDrive *order[2];
	int n = 0;
	if (cfg.pfloppy_drive != 1 || pf_format != PFLOPPY_FORMAT_PC) order[n++] = pf.gw;
	if (cfg.pfloppy_drive != 2 && pf_format == PFLOPPY_FORMAT_PC) order[n++] = pf.usb; // USB drives can't read Amiga disks

	for (int i = 0; i < n; i++)
	{
		char desc[96] = "";
		if (order[i]->open(desc, sizeof(desc)))
		{
			pthread_mutex_lock(&pf.lock);
			pf.drv = order[i];
			snprintf(pf.desc, sizeof(pf.desc), "%s", desc);
			pf.media.drive = 1;
			pthread_mutex_unlock(&pf.lock);
			plog("drive opened: %s", desc);
			return 1;
		}
	}
	return 0;
}

/* ------------------------------------------------------------------ media state */

// lock must be held
static void reset_cache()
{
	if (pf.dirty_count)
	{
		pf.lost_writes += pf.dirty_count;
		plog("WARNING: %d written sectors were not saved to the disk (disk removed/changed)", pf.dirty_count);
	}
	memset(pf.state, ST_INVALID, sizeof(pf.state));
	memset(pf.dirty, 0, sizeof(pf.dirty));
	memset(pf.wretry, 0, sizeof(pf.wretry));
	pf.dirty_count = 0;
	pf.ra_left = 0;
	pf.last_chunk = -1;
	pf.fill_next = 0;
	pf.fill_t0 = now_ms();
	pf.fill_logged = 0;
	pf.gen++;
}

static void set_no_media(const char *why)
{
	pthread_mutex_lock(&pf.lock);
	int was = pf.media.present;
	pf.media.present = 0;
	pf.media.total_sectors = 0;
	reset_cache();
	if (was) pf.event = 1;
	pthread_mutex_unlock(&pf.lock);
	if (was) plog("disk removed (%s)", why);
}

static void close_drive(const char *why)
{
	set_no_media(why);
	pthread_mutex_lock(&pf.lock);
	PfDrive *d = pf.drv;
	pf.drv = NULL;
	pf.media.drive = 0;
	pthread_mutex_unlock(&pf.lock);
	if (d) d->close();
	plog("drive closed (%s)", why);
}

static void set_media(const pfd_geometry *g)
{
	pthread_mutex_lock(&pf.lock);
	reset_cache();
	pf.geom = *g;
	pf.media.present = 1;
	pf.media.total_sectors = g->total;
	pf.media.write_protected = g->wp;
	pf.media.spt = g->spt;
	pf.media.heads = g->heads;
	pf.media.cylinders = g->cyl;
	pf.event = 1;
	// the drive is spinning anyway right after the insertion: read cylinder 0
	// (boot sector, FAT, root directory) so the first DIR doesn't wait for it
	pf.ra_next = 0;
	pf.ra_left = g->heads;
	pf.last_why[0] = 0;
	pthread_mutex_unlock(&pf.lock);

	if (pf_format != PFLOPPY_FORMAT_PC && g->spt == 12) plog("disk inserted: %d sides%s", g->heads, g->wp ? " write-protected" : "");
	else plog("disk inserted: %d sectors (%dK) C/H/S %d/%d/%d%s", g->total, g->total / 2, g->cyl, g->heads, g->spt, g->wp ? " write-protected" : "");
}

// checks the disk in the drive and updates the media state
// force_changed: a previous command already reported a disk change
static void probe_media(int force_changed = 0)
{
	if (!pf.drv) return;

	pfd_geometry g = {};
	char why[160] = "";
	int r = pf.drv->probe(pf.media.present, force_changed, &g, why, sizeof(why));

	switch (r)
	{
	case PFD_PROBE_LOST:
		close_drive(why[0] ? why : "drive not responding");
		break;

	case PFD_PROBE_NODISK:
		if (pf.media.present) set_no_media(why[0] ? why : "no disk");
		pf.last_why[0] = 0;
		break;

	case PFD_PROBE_UNREADABLE:
		if (pf.media.present) set_no_media("disk not readable");
		if (strcmp(why, pf.last_why))
		{
			snprintf(pf.last_why, sizeof(pf.last_why), "%s", why);
			plog("%s", why);
		}
		break;

	case PFD_PROBE_NEW:
		if (g.total <= 0 || g.total > MAX_SECTORS || g.spt <= 0 || g.spt > MAX_SPT)
		{
			if (pf.media.present) set_no_media("unsupported disk");
			snprintf(why, sizeof(why), "unsupported disk: %d sectors, %d per track", g.total, g.spt);
			if (strcmp(why, pf.last_why))
			{
				snprintf(pf.last_why, sizeof(pf.last_why), "%s", why);
				plog("%s", why);
			}
			break;
		}
		set_media(&g);
		break;

	default: // busy, same
		break;
	}
}

/* ------------------------------------------------------------------ background jobs */

// reacts to a PFD_* result. returns 1 if the operation should not be retried now
static int handle_result(int r, const char *what)
{
	switch (r)
	{
	case PFD_OK:
		return 0;

	case PFD_LOST:
		close_drive(what);
		return 1;

	// disk changed or drive reset: the data may belong to another disk
	case PFD_CHANGED:
		pf.recheck = 2;
		return 1;

	// no disk, or spinning up
	case PFD_NOT_READY:
		if (!pf.recheck) pf.recheck = 1;
		usleep(100000);
		return 1;
	}
	return 0;
}

// reads one track. A track (not a whole cylinder) keeps the time a new
// request from the core may have to wait for the drive short.
static void read_chunk(int chunk)
{
	static uint8_t buf[MAX_CHUNK * 512];
	uint8_t st[MAX_CHUNK];

	pthread_mutex_lock(&pf.lock);
	uint32_t gen = pf.gen;
	pfd_geometry g = pf.geom;
	int first = chunk * g.spt;
	int n = g.total - first;
	pthread_mutex_unlock(&pf.lock);

	if (n <= 0 || !pf.drv) return;
	if (n > g.spt) n = g.spt;

	memset(st, 2, sizeof(st));
	double t0 = now_ms();
	int r = pf.drv->read_track(&g, chunk, buf, st);
	double dt = now_ms() - t0;
	pf.last_io_ms = now_ms();

	if (handle_result(r, "read failed")) return;
	if (r != PFD_OK)
	{
		// whole track unreadable
		memset(st, 2, sizeof(st));
	}

	int bad[MAX_CHUNK], nbad = 0;
	pthread_mutex_lock(&pf.lock);
	if (gen == pf.gen)
	{
		for (int i = 0; i < n; i++)
		{
			if (pf.state[first + i] != ST_INVALID) continue;
			if (st[i] == 1)
			{
				memcpy(pf.data + (first + i) * 512, buf + i * 512, 512);
				pf.state[first + i] = ST_VALID;
			}
			else
			{
				pf.state[first + i] = ST_BAD;
				pf.bad++;
				bad[nbad++] = first + i;
			}
		}
	}
	pthread_mutex_unlock(&pf.lock);

	for (int i = 0; i < nbad; i++)
	{
		int s = bad[i];
		plog("bad sector %d (C/H/S %d/%d/%d), the core gets zeros", s, s / (g.spt * g.heads), (s / g.spt) % g.heads, s % g.spt + 1);
	}
	if (dt > 2500) plog("slow read: track %d took %.0f ms", chunk, dt);
}

// writes one run of contiguous dirty sectors; returns 1 if something was written
// first track with changed sectors that can be written now (lock held): not the
// one the core is still writing, unless it is complete or the core stopped.
// force: any. returns its first sector, -1 none
static int flush_pick(int force)
{
	int spt = pf.media.spt;
	int busy = !force && now_ms() - pf.last_write_ms < WRITE_SETTLE_MS ? pf.last_write_track : -1;
	for (int i = 0; i < pf.media.total_sectors; i++)
	{
		if (!pf.dirty[i]) continue;
		int t = i / spt;
		if (t != busy) return i;
		int t0 = t * spt, all = 1;
		for (int k = t0; k < t0 + spt && k < pf.media.total_sectors; k++) if (!pf.dirty[k]) all = 0;
		if (all) return i;
		i = t0 + spt - 1; // skip the rest of that track
	}
	return -1;
}

static int flush_one(int force)
{
	static uint8_t buf[MAX_SPT * 512];
	uint8_t dirty[MAX_SPT] = {}, valid[MAX_SPT] = {};

	pthread_mutex_lock(&pf.lock);
	if (!pf.dirty_count || !pf.media.present)
	{
		pthread_mutex_unlock(&pf.lock);
		return 0;
	}

	int first = flush_pick(force);
	if (first < 0)
	{
		int any = 0;
		for (int i = 0; i < pf.media.total_sectors && !any; i++) any = pf.dirty[i];
		if (!any) pf.dirty_count = 0;
		pthread_mutex_unlock(&pf.lock);
		return 0;
	}

	// all the changed sectors of that track at once (a Greaseweazle rewrites whole tracks)
	int spt = pf.media.spt;
	int track = first / spt;
	int t0 = track * spt;
	int n = pf.media.total_sectors - t0;
	if (n > spt) n = spt;
	int ndirty = 0;
	for (int i = 0; i < n; i++)
	{
		int s = t0 + i;
		if (pf.state[s] == ST_VALID) memcpy(buf + i * 512, pf.data + s * 512, 512);
		valid[i] = pf.state[s] == ST_VALID;
		if (pf.dirty[s])
		{
			dirty[i] = 1;
			pf.dirty[s] = 0;
			ndirty++;
		}
	}
	pf.dirty_count -= ndirty;
	uint32_t gen = pf.gen;
	pfd_geometry g = pf.geom;
	pthread_mutex_unlock(&pf.lock);

	double tw = now_ms();
	int r = pf.drv ? pf.drv->write_track(&g, track, buf, dirty, valid) : PFD_LOST;
	pf.last_io_ms = now_ms();
	if (r == PFD_OK)
	{
		pthread_mutex_lock(&pf.lock);
		pf.written += ndirty;
		pthread_mutex_unlock(&pf.lock);
		plog("wrote %d sector(s) on track %d in %.0f ms", ndirty, track, pf.last_io_ms - tw);
		return 1;
	}

	if (r == PFD_WP)
	{
		plog("disk is write-protected: %d sectors not written", ndirty);
		pthread_mutex_lock(&pf.lock);
		pf.lost_writes += ndirty;
		if (!pf.media.write_protected) { pf.media.write_protected = 1; pf.event = 1; }
		pthread_mutex_unlock(&pf.lock);
		return 1;
	}

	if (handle_result(r, "write failed"))
	{
		// a lost drive/disk change is final; a busy drive (communication reset) is retried
		if (r == PFD_NOT_READY)
		{
			pthread_mutex_lock(&pf.lock);
			if (gen == pf.gen)
				for (int i = 0; i < n; i++)
					if (dirty[i] && !pf.dirty[t0 + i]) { pf.dirty[t0 + i] = 1; pf.dirty_count++; }
			pthread_mutex_unlock(&pf.lock);
			return 1;
		}
		plog("%d sectors not written (disk removed?)", ndirty);
		pthread_mutex_lock(&pf.lock);
		pf.lost_writes += ndirty;
		pthread_mutex_unlock(&pf.lock);
		return 1;
	}

	// medium error: retry later, give up after a few attempts
	pthread_mutex_lock(&pf.lock);
	int dropped = 0;
	if (gen == pf.gen)
	{
		for (int i = 0; i < n; i++)
		{
			if (!dirty[i]) continue;
			int s = t0 + i;
			if (pf.dirty[s]) continue; // written again meanwhile
			if (++pf.wretry[s] < WRITE_RETRIES) { pf.dirty[s] = 1; pf.dirty_count++; }
			else { pf.wretry[s] = 0; dropped++; }
		}
		pf.lost_writes += dropped;
	}
	pthread_mutex_unlock(&pf.lock);
	plog("write error on track %d%s", track, dropped ? ", sectors dropped" : ", will retry");
	return 1;
}

static void write_stats()
{
	double t = now_ms();
	if (t - pf.last_stats < STATS_MS) return;
	pf.last_stats = t;

	pthread_mutex_lock(&pf.lock);
	uint32_t hits = pf.hits, misses = pf.misses, written = pf.written;
	double worst = pf.worst_wait;
	pf.hits = pf.misses = pf.written = 0;
	pf.worst_wait = 0;
	pthread_mutex_unlock(&pf.lock);

	if (hits || misses || written) plog("stats: %u sectors from cache, %u waited for the drive, worst wait %.0f ms, %u written, bad %u, lost writes %u",
		hits, misses, worst, written, pf.bad, pf.lost_writes);
}

static void *worker(void *)
{
	double last_poll = 0, last_scan = 0;

	while (__atomic_load_n(&pf.alive, __ATOMIC_ACQUIRE))
	{
		if (!pf.drv)
		{
			if (now_ms() - last_scan >= RESCAN_MS)
			{
				last_scan = now_ms();
				if (open_drive()) probe_media();
			}
			usleep(100000);
			continue;
		}

		if (pf.recheck)
		{
			int force = (pf.recheck == 2);
			pf.recheck = 0;
			probe_media(force);
			last_poll = now_ms();
			continue;
		}

		int chunk = -1;
		int flush = 0;
		double fill_done_ms = -1;

		pthread_mutex_lock(&pf.lock);
		if (pf.media.present)
		{
			int size = pf.media.spt;
			int chunks = (pf.media.total_sectors + size - 1) / size;

			if (pf.want_lba >= 0 && pf.want_lba < pf.media.total_sectors && pf.state[pf.want_lba] == ST_INVALID)
			{
				chunk = pf.want_lba / size;
			}
			else if (pf.dirty_count && flush_pick(0) >= 0)
			{
				flush = 1;
			}
			else
			{
				while (pf.ra_left > 0 && chunk < 0)
				{
					int c = pf.ra_next++;
					pf.ra_left--;
					if (c >= chunks) { pf.ra_left = 0; break; }
					for (int i = c * size; i < (c + 1) * size && i < pf.media.total_sectors; i++)
					{
						if (pf.state[i] == ST_INVALID) { chunk = c; break; }
					}
				}

				// nothing else to do: keep filling the cache with the rest of the disk,
				// so later requests from the core don't have to wait for the drive
				while (PFLOPPY_BACKGROUND_FILL && chunk < 0 && pf.fill_next < chunks)
				{
					int c = pf.fill_next;
					for (int i = c * size; i < (c + 1) * size && i < pf.media.total_sectors; i++)
					{
						if (pf.state[i] == ST_INVALID) { chunk = c; break; }
					}
					if (chunk < 0) pf.fill_next++;
				}

				if (PFLOPPY_BACKGROUND_FILL && chunk < 0 && pf.fill_next >= chunks && !pf.fill_logged)
				{
					pf.fill_logged = 1;
					fill_done_ms = now_ms() - pf.fill_t0;
				}
			}
		}

		if (fill_done_ms >= 0)
		{
			pthread_mutex_unlock(&pf.lock);
			plog("whole disk in cache after %.1f s, the drive is used now only for writes and disk change checks", fill_done_ms / 1000);
			pthread_mutex_lock(&pf.lock);
		}

		if (chunk < 0 && !flush)
		{
			// idle
			struct timespec ts;
			clock_gettime(CLOCK_REALTIME, &ts);
			ts.tv_nsec += 50 * 1000 * 1000;
			if (ts.tv_nsec >= 1000000000) { ts.tv_sec++; ts.tv_nsec -= 1000000000; }
			pthread_cond_timedwait(&pf.cond, &pf.lock, &ts);

			double t = now_ms();
			int abandoned = -1;
			double abandoned_ms = 0;
			if (pf.want_lba >= 0 && t - pf.last_call_ms > ABANDON_MS)
			{
				abandoned = pf.want_lba;
				abandoned_ms = pf.last_call_ms > pf.want_t0 ? pf.last_call_ms - pf.want_t0 : 0;
				pf.want_lba = -1;
			}

			// keep-alive commands are not "use": they don't extend the motor-off delay
			double last_cmd = pf.last_io_ms > pf.last_keep_ms ? pf.last_io_ms : pf.last_keep_ms;
			int keep = pf.media.present && pf.last_access_ms > 0 && t - pf.last_access_ms < MOTOR_KEEP_MS && t - last_cmd > MOTOR_TICK_MS;
			int keep_track = pf.last_chunk > 0 ? pf.last_chunk : 0;
			// motor-off delay counts from the last access or drive operation (prefetch included)
			double last_use = pf.last_access_ms > pf.last_io_ms ? pf.last_access_ms : pf.last_io_ms;
			double idle_ms = last_use > 0 ? t - last_use : 1e9;
			pfd_geometry g = pf.geom;
			pthread_mutex_unlock(&pf.lock);

			if (abandoned >= 0) plog("the core stopped asking for sector %d, it waited %.0f ms (BIOS/driver timeout?)", abandoned, abandoned_ms);
			if (keep && pf.drv)
			{
				// this command may be the first to see a disk change: never drop that news
				handle_result(pf.drv->keep_spinning(&g, keep_track), "drive not responding");
				pf.last_keep_ms = now_ms();
			}
			if (pf.drv) pf.drv->idle(idle_ms);

			write_stats();
			if (pf.drv && now_ms() - last_poll >= pf.drv->poll_ms())
			{
				last_poll = now_ms();
				probe_media();
			}
			continue;
		}
		pthread_mutex_unlock(&pf.lock);

		if (chunk >= 0) read_chunk(chunk);
		else flush_one(0);
	}

	// flush pending writes before leaving
	double t0 = now_ms();
	while (pf.drv && pf.dirty_count && now_ms() - t0 < 5000) flush_one(1);

	return NULL;
}

/* ------------------------------------------------------------------ public API */

int pfloppy_is_placeholder(const char *path)
{
	if (!path || !path[0]) return 0;
	const char *p = strrchr(path, '/');
	p = p ? p + 1 : path;
	return !strcasecmp(p, PFLOPPY_PLACEHOLDER);
}

void pfloppy_create_placeholder(const char *dir)
{
	pfloppy_create_placeholder_named(dir, PFLOPPY_PLACEHOLDER);
}

void pfloppy_create_placeholder_named(const char *dir, const char *name)
{
	char path[1024];
	struct stat st;

	snprintf(path, sizeof(path), "%s", getFullPath(dir));
	if (stat(path, &st) || !S_ISDIR(st.st_mode)) return;

	int l = strlen(path);
	snprintf(path + l, sizeof(path) - l, "/%s", name);
	if (!stat(path, &st)) return;

	FILE *f = fopen(path, "wb");
	if (f)
	{
		fclose(f);
		plog("created %s", path);
	}
}

void pfloppy_set_format(int format)
{
	if (format == pf_format) return;
	if (pf.thread_running) pfloppy_detach();
	if (pf.drv) close_drive("format change");
	delete pf.gw;
	pf.gw = NULL;
	pf_format = format;
}

static void attach(int unit, int sync)
{
	pthread_mutex_lock(&pf.lock);
	pf.unit = unit;
	pthread_mutex_unlock(&pf.lock);

	if (pf.thread_running) return;

	if (!pf.data) pf.data = (uint8_t*)malloc(MAX_SECTORS * 512);

	pf_log_reset();
	if (pf_format >= PFLOPPY_FORMAT_MAC) plog("attached to floppy slot %d", unit);
	else plog("attached as floppy %c:", 'A' + unit);

	// first look done here, so the BIOS sees a disk inserted before boot
	// (async: by the I/O thread, the disk is reported with a media event)
	if (sync)
	{
		if (!pf.drv && open_drive()) probe_media();
		pf.event = 0;
	}

	pf.alive = 1;
	if (pthread_create(&pf.thread, NULL, worker, NULL))
	{
		plog("cannot start the I/O thread");
		pf.alive = 0;
		return;
	}
	pf.thread_running = 1;
}

void pfloppy_attach(int unit) { attach(unit, 1); }
void pfloppy_attach_async(int unit) { attach(unit, 0); }

void pfloppy_detach(void)
{
	if (!pf.thread_running) return;

	__atomic_store_n(&pf.alive, 0, __ATOMIC_RELEASE);
	pthread_cond_signal(&pf.cond);
	pthread_join(pf.thread, NULL);
	pf.thread_running = 0;

	if (pf.drv) close_drive("detached");
	pf.unit = -1;
	pf.event = 0;
	plog("detached");
}

int pfloppy_unit(void)
{
	return pf.thread_running ? pf.unit : -1;
}

void pfloppy_get_media(pfloppy_media_t *m)
{
	pthread_mutex_lock(&pf.lock);
	*m = pf.media;
	pthread_mutex_unlock(&pf.lock);
}

int pfloppy_media_event(void)
{
	pthread_mutex_lock(&pf.lock);
	int ev = pf.event;
	pf.event = 0;
	pthread_mutex_unlock(&pf.lock);
	return ev;
}

int pfloppy_read(uint32_t lba, uint8_t *buf)
{
	int res;
	double slow = 0;
	uint32_t slow_track = 0;
	pthread_mutex_lock(&pf.lock);

	if (!pf.media.present || (int)lba >= pf.media.total_sectors)
	{
		pf.want_lba = -1;
		pthread_mutex_unlock(&pf.lock);
		return -1;
	}

	pf.last_call_ms = pf.last_access_ms = now_ms();

	int chunk = lba / pf.media.spt;
	if (chunk != pf.last_chunk)
	{
		pf.last_chunk = chunk;
		pf.ra_next = chunk + 1;
		pf.ra_left = READAHEAD_TRACKS;
	}

	uint8_t st = pf.state[lba];
	if (st == ST_VALID)
	{
		memcpy(buf, pf.data + lba * 512, 512);
		if (pf.want_lba == (int)lba)
		{
			double w = now_ms() - pf.want_t0;
			if (w > pf.worst_wait) pf.worst_wait = w;
			if (w > SLOW_WAIT_MS) { slow = w; slow_track = lba / pf.media.spt; }
			pf.want_lba = -1;
		}
		else pf.hits++;
		res = 1;
	}
	else if (st == ST_BAD)
	{
		if (pf.want_lba == (int)lba) pf.want_lba = -1;
		res = -1;
	}
	else if (pf.want_lba == (int)lba && now_ms() - pf.want_t0 > PENDING_GIVEUP_MS)
	{
		plog("sector %u not delivered in time, the core gets zeros", lba);
		pf.want_lba = -1;
		res = -1;
	}
	else
	{
		if (pf.want_lba != (int)lba)
		{
			pf.want_lba = lba;
			pf.want_t0 = now_ms();
			pf.misses++;
		}
		res = 0;
	}

	pthread_cond_signal(&pf.cond);
	pthread_mutex_unlock(&pf.lock);

	if (slow > 0) plog("sector %u (track %u) waited %.0f ms for the drive", lba, slow_track, slow);
	return res;
}

int pfloppy_write(uint32_t lba, const uint8_t *buf, uint32_t cnt)
{
	pthread_mutex_lock(&pf.lock);

	if (!pf.media.present || pf.media.write_protected || lba + cnt > (uint32_t)pf.media.total_sectors)
	{
		pthread_mutex_unlock(&pf.lock);
		return -1;
	}

	pf.last_access_ms = now_ms();
	pf.last_write_ms = pf.last_access_ms;
	pf.last_write_track = (lba + cnt - 1) / pf.media.spt;
	for (uint32_t i = 0; i < cnt; i++)
	{
		memcpy(pf.data + (lba + i) * 512, buf + i * 512, 512);
		pf.state[lba + i] = ST_VALID;
		pf.wretry[lba + i] = 0;
		if (!pf.dirty[lba + i])
		{
			pf.dirty[lba + i] = 1;
			pf.dirty_count++;
		}
	}

	pthread_cond_signal(&pf.cond);
	pthread_mutex_unlock(&pf.lock);
	return 1;
}
