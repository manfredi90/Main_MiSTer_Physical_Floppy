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
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <scsi/sg.h>

#include "physical_floppy.h"
#include "../../file_io.h"

#define LOG_PATH          "/tmp/pfloppy.log"
#define MAX_SECTORS       5760   // 2.88MB
#define MAX_SPT           36
#define MAX_CHUNK         MAX_SPT // reads are done one track at a time
#define READAHEAD_TRACKS  2      // tracks read in advance after the one in use
#define SLOW_WAIT_MS      800    // log every sector the core waited longer than this

// 1 = after a disk is inserted read the whole disk in the background, so every
// later request is served from the cache. Off: the drive reads only what the
// core asks for (plus a little read-ahead), like a real floppy drive.
#ifndef PFLOPPY_BACKGROUND_FILL
#define PFLOPPY_BACKGROUND_FILL 0
#endif
#define POLL_MS           1000   // disk presence check when idle
#define RESCAN_MS         2000   // drive search when no drive is open
#define IO_TIMEOUT_MS     8000   // spin-up + seek + read
#define PENDING_GIVEUP_MS 15000  // answer with zeros if a sector never arrives
#define STATS_MS          5000
#define WRITE_RETRIES     3

enum { ST_INVALID = 0, ST_VALID = 1, ST_BAD = 2 };

// sense helpers: code = key<<16 | asc<<8 | ascq
#define SK(c)  (((c) >> 16) & 0xF)
#define ASC(c) (((c) >> 8) & 0xFF)

static struct
{
	pthread_mutex_t lock;
	pthread_cond_t cond;
	pthread_t thread;
	int thread_running;
	volatile int alive;

	int unit;
	int fd;
	char dev[64];

	pfloppy_media_t media;
	uint32_t gen;            // incremented on every disk change
	volatile int event;
	volatile int recheck;    // thread should probe the disk as soon as possible

	uint8_t *data;
	uint8_t state[MAX_SECTORS];
	uint8_t dirty[MAX_SECTORS];
	uint8_t wretry[MAX_SECTORS];
	int dirty_count;

	int want_lba;
	double want_t0;
	int last_chunk;          // track in use (reads are done one track at a time)
	int ra_next;
	int ra_left;
	int fill_next;           // background fill of the whole disk: next track to check
	double fill_t0;
	int fill_logged;

	uint32_t hits, misses, bad, lost_writes;
	double worst_wait;
	double last_stats;
	char last_scan[256];
} pf = {
	PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, 0, 0, 0,
	-1, -1, "",
	{}, 0, 0, 0,
	NULL, {}, {}, {}, 0,
	-1, 0, -1, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, ""
};

static double now_ms()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static void plog(const char *fmt, ...)
{
	char msg[512];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(msg, sizeof(msg), fmt, ap);
	va_end(ap);

	printf("PFLOPPY: %s\n", msg);

	FILE *f = fopen(LOG_PATH, "a");
	if (f)
	{
		fprintf(f, "[%10.3f] %s\n", now_ms() / 1000.0, msg);
		fclose(f);
	}
}

/* ------------------------------------------------------------------ geometry */

static void calc_geometry(int total, int *spt, int *heads, int *cyl)
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

/* ------------------------------------------------------------------ SCSI */

// returns 0 = ok, 1 = check condition (see *code), -1 = transport/driver error
static int sg_exec(const uint8_t *cdb, int cdb_len, int dir, void *buf, int len, int timeout_ms, int *code)
{
	uint8_t sense[32] = {};
	sg_io_hdr_t io;

	if (code) *code = 0;
	if (pf.fd < 0) return -1;

	memset(&io, 0, sizeof(io));
	io.interface_id = 'S';
	io.cmd_len = cdb_len;
	io.cmdp = (unsigned char*)cdb;
	io.dxfer_direction = dir;
	io.dxfer_len = len;
	io.dxferp = buf;
	io.sbp = sense;
	io.mx_sb_len = sizeof(sense);
	io.timeout = timeout_ms;

	if (ioctl(pf.fd, SG_IO, &io) < 0) return -1;
	if (io.host_status) return -1;

	int c = 0;
	if (io.sb_len_wr > 0)
	{
		uint8_t rc = sense[0] & 0x7F;
		if (rc == 0x70 || rc == 0x71) c = ((sense[2] & 0xF) << 16) | (sense[12] << 8) | sense[13];
		else if (rc == 0x72 || rc == 0x73) c = ((sense[1] & 0xF) << 16) | (sense[2] << 8) | sense[3];
	}
	if (code) *code = c;

	if (io.status)
	{
		// recovered error is a success
		if (c && SK(c) == 1) return 0;
		return 1;
	}
	if (io.driver_status & ~0x08) return -1; // anything but DRIVER_SENSE (timeout, error)
	return 0;
}

static int scsi_tur(int *code)
{
	uint8_t cdb[6] = { 0x00, 0, 0, 0, 0, 0 };
	return sg_exec(cdb, 6, SG_DXFER_NONE, NULL, 0, 3000, code);
}

static int scsi_capacity(int *total, int *blksz, int *code)
{
	uint8_t cdb[10] = { 0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t buf[8] = {};
	int r = sg_exec(cdb, 10, SG_DXFER_FROM_DEV, buf, sizeof(buf), IO_TIMEOUT_MS, code);
	if (r) return r;
	*total = ((buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3]) + 1;
	*blksz = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];
	return 0;
}

static int scsi_write_protected()
{
	// UFI only supports MODE SENSE(10)
	uint8_t cdb[10] = { 0x5A, 0, 0x3F, 0, 0, 0, 0, 0, 192, 0 };
	uint8_t buf[192] = {};
	int code;
	if (sg_exec(cdb, 10, SG_DXFER_FROM_DEV, buf, sizeof(buf), 3000, &code)) return 0;
	return (buf[3] & 0x80) ? 1 : 0;
}

static int scsi_rw(int write, int lba, int cnt, uint8_t *buf, int *code)
{
	uint8_t cdb[10] = {};
	cdb[0] = write ? 0x2A : 0x28;
	cdb[2] = (lba >> 24) & 0xFF;
	cdb[3] = (lba >> 16) & 0xFF;
	cdb[4] = (lba >> 8) & 0xFF;
	cdb[5] = lba & 0xFF;
	cdb[7] = (cnt >> 8) & 0xFF;
	cdb[8] = cnt & 0xFF;
	return sg_exec(cdb, 10, write ? SG_DXFER_TO_DEV : SG_DXFER_FROM_DEV, buf, cnt * 512, IO_TIMEOUT_MS, code);
}

/* ------------------------------------------------------------------ drive discovery */

static int read_sys(const char *path, char *out, int sz)
{
	out[0] = 0;
	FILE *f = fopen(path, "r");
	if (!f) return 0;
	if (!fgets(out, sz, f)) out[0] = 0;
	fclose(f);
	int l = strlen(out);
	while (l && (out[l - 1] == '\n' || out[l - 1] == ' ')) out[--l] = 0;
	return 1;
}

static int str_has(const char *s, const char *what)
{
	char low[128];
	int i = 0;
	for (; s[i] && i < (int)sizeof(low) - 1; i++) low[i] = tolower((unsigned char)s[i]);
	low[i] = 0;
	return strstr(low, what) != NULL;
}

static int find_drive(char *dev, int sz)
{
	char scan[256] = "";
	int found = 0;

	DIR *d = opendir("/sys/block");
	if (!d) return 0;

	struct dirent *de;
	while ((de = readdir(d)))
	{
		if (strncmp(de->d_name, "sd", 2)) continue;

		char path[PATH_MAX], real[PATH_MAX], val[128], vendor[64], model[64];

		snprintf(path, sizeof(path), "/sys/block/%s/device", de->d_name);
		if (!realpath(path, real)) continue;

		// <usb interface>/hostN/targetN:N:N/N:N:N:N -> usb interface
		for (int i = 0; i < 3; i++)
		{
			char *p = strrchr(real, '/');
			if (p) *p = 0;
		}

		snprintf(path, sizeof(path), "%s/bInterfaceSubClass", real);
		int ufi = read_sys(path, val, sizeof(val)) && !strcmp(val, "04");

		snprintf(path, sizeof(path), "/sys/block/%s/device/vendor", de->d_name);
		read_sys(path, vendor, sizeof(vendor));
		snprintf(path, sizeof(path), "/sys/block/%s/device/model", de->d_name);
		read_sys(path, model, sizeof(model));
		snprintf(path, sizeof(path), "/sys/block/%s/removable", de->d_name);
		int removable = read_sys(path, val, sizeof(val)) && !strcmp(val, "1");

		int named = str_has(model, "floppy") || str_has(model, "fdd") || str_has(model, "ufi") || str_has(vendor, "floppy");

		int l = strlen(scan);
		snprintf(scan + l, sizeof(scan) - l, "%s[%s %s ufi=%d rem=%d] ", de->d_name, vendor, model, ufi, removable);

		if (!found && (ufi || (removable && named)))
		{
			snprintf(dev, sz, "/dev/%s", de->d_name);
			found = 1;
		}
	}
	closedir(d);

	if (strcmp(scan, pf.last_scan))
	{
		snprintf(pf.last_scan, sizeof(pf.last_scan), "%s", scan);
		plog("usb disks: %s-> %s", scan[0] ? scan : "none ", found ? dev : "no floppy drive");
	}
	return found;
}

static void unmount_device(const char *dev)
{
	FILE *f = fopen("/proc/mounts", "r");
	if (!f) return;

	char line[512];
	int len = strlen(dev);
	while (fgets(line, sizeof(line), f))
	{
		if (strncmp(line, dev, len)) continue;
		if (line[len] != ' ' && !isdigit((unsigned char)line[len])) continue;

		char *mp = strchr(line, ' ');
		if (!mp) continue;
		mp++;
		char *end = strchr(mp, ' ');
		if (end) *end = 0;

		if (!umount2(mp, MNT_DETACH)) plog("unmounted %s from %s (the floppy is used by the core)", dev, mp);
		else plog("cannot unmount %s from %s: %s", dev, mp, strerror(errno));
	}
	fclose(f);
}

static int open_drive()
{
	char dev[64];

	// manual override (debug / drives not detected automatically): PFLOPPY_DEV=/dev/sdX
	const char *forced = getenv("PFLOPPY_DEV");
	if (forced && forced[0]) snprintf(dev, sizeof(dev), "%s", forced);
	else if (!find_drive(dev, sizeof(dev))) return 0;

	int fd = open(dev, O_RDWR | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0) fd = open(dev, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0)
	{
		plog("cannot open %s: %s", dev, strerror(errno));
		return 0;
	}

	pthread_mutex_lock(&pf.lock);
	pf.fd = fd;
	snprintf(pf.dev, sizeof(pf.dev), "%s", dev);
	pf.media.drive = 1;
	pthread_mutex_unlock(&pf.lock);

	plog("drive opened: %s", dev);
	unmount_device(dev);
	return 1;
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
	if (pf.fd >= 0) close(pf.fd);
	pf.fd = -1;
	pf.media.drive = 0;
	pf.last_scan[0] = 0;
	pthread_mutex_unlock(&pf.lock);
	plog("drive closed (%s)", why);
}

static void set_media(int total, int wp)
{
	int spt, heads, cyl;
	calc_geometry(total, &spt, &heads, &cyl);

	pthread_mutex_lock(&pf.lock);
	reset_cache();
	pf.media.present = 1;
	pf.media.total_sectors = total;
	pf.media.write_protected = wp;
	pf.media.spt = spt;
	pf.media.heads = heads;
	pf.media.cylinders = cyl;
	pf.event = 1;
	pthread_mutex_unlock(&pf.lock);

	plog("disk inserted: %d sectors (%dK) C/H/S %d/%d/%d%s", total, total / 2, cyl, heads, spt, wp ? " write-protected" : "");
	unmount_device(pf.dev);
}

// checks the disk in the drive and updates the media state
// force_changed: a previous command already reported a disk change
static void probe_media(int force_changed = 0)
{
	int code = 0;
	int changed = force_changed;
	int r = scsi_tur(&code);

	// unit attention (disk changed / reset): ask again
	for (int i = 0; i < 3 && r == 1 && SK(code) == 6; i++)
	{
		if (ASC(code) == 0x28 || ASC(code) == 0x29) changed = 1;
		r = scsi_tur(&code);
	}

	if (r < 0)
	{
		close_drive("drive not responding");
		return;
	}

	if (r == 1)
	{
		// 02/04: becoming ready, decide on the next poll
		if (SK(code) == 2 && ASC(code) == 0x04) return;
		if (pf.media.present || changed) set_no_media(SK(code) == 2 && ASC(code) == 0x3A ? "no disk" : "drive not ready");
		return;
	}

	if (pf.media.present && !changed) return;

	int total = 0, blksz = 0;
	r = scsi_capacity(&total, &blksz, &code);
	if (r)
	{
		if (r < 0) close_drive("drive not responding");
		else
		{
			if (pf.media.present) set_no_media("disk not readable");
			static int last_code = -1;
			if (code != last_code) plog("disk present but not readable (sense %X/%02X/%02X), unformatted?", SK(code), ASC(code), code & 0xFF);
			last_code = code;
		}
		return;
	}

	if (blksz != 512 || total <= 0 || total > MAX_SECTORS)
	{
		plog("unsupported disk: %d sectors of %d bytes", total, blksz);
		if (pf.media.present) set_no_media("unsupported disk");
		return;
	}

	set_media(total, scsi_write_protected());
}

/* ------------------------------------------------------------------ background jobs */

// returns 0 = ok, 1 = media problem handled, -1 = drive lost
static int handle_error(int r, int code, const char *what)
{
	if (r < 0)
	{
		close_drive(what);
		return -1;
	}

	// unit attention: disk changed or drive reset, the data may belong to another disk
	if (SK(code) == 6)
	{
		pf.recheck = 2;
		return 1;
	}

	// not ready: no disk, or spinning up
	if (SK(code) == 2)
	{
		if (!pf.recheck) pf.recheck = 1;
		usleep(100000);
		return 1;
	}
	return 0;
}

// reads one track in a single command. A track (not a whole cylinder) keeps
// the time a new request from the core may have to wait for the drive short.
static void read_chunk(int chunk)
{
	static uint8_t buf[MAX_CHUNK * 512];
	int code;

	pthread_mutex_lock(&pf.lock);
	uint32_t gen = pf.gen;
	int spt = pf.media.spt;
	int heads = pf.media.heads;
	int size = spt;
	int first = chunk * size;
	int n = pf.media.total_sectors - first;
	pthread_mutex_unlock(&pf.lock);

	if (n <= 0) return;
	if (n > size) n = size;

	double t0 = now_ms();
	int r = scsi_rw(0, first, n, buf, &code);
	double dt = now_ms() - t0;

	if (!r)
	{
		pthread_mutex_lock(&pf.lock);
		if (gen == pf.gen)
		{
			for (int i = 0; i < n; i++)
			{
				if (pf.state[first + i] == ST_INVALID)
				{
					memcpy(pf.data + (first + i) * 512, buf + i * 512, 512);
					pf.state[first + i] = ST_VALID;
				}
			}
		}
		pthread_mutex_unlock(&pf.lock);
		if (dt > 2500) plog("slow read: track %d took %.0f ms", chunk, dt);
		return;
	}

	if (handle_error(r, code, "read failed")) return;

	// medium error: find the bad sectors one by one
	plog("read error on track %d (sense %X/%02X/%02X), retrying sector by sector", chunk, SK(code), ASC(code), code & 0xFF);
	for (int i = 0; i < n; i++)
	{
		int s = first + i;

		pthread_mutex_lock(&pf.lock);
		int need = (gen == pf.gen) && pf.state[s] == ST_INVALID;
		pthread_mutex_unlock(&pf.lock);
		if (!need) continue;

		int rr = 1;
		for (int k = 0; k < 2 && rr; k++)
		{
			rr = scsi_rw(0, s, 1, buf, &code);
			// disk changed/removed or drive lost: stop here, never retry on another disk
			if (rr && handle_error(rr, code, "read failed")) return;
		}

		pthread_mutex_lock(&pf.lock);
		if (gen == pf.gen && pf.state[s] == ST_INVALID)
		{
			if (!rr)
			{
				memcpy(pf.data + s * 512, buf, 512);
				pf.state[s] = ST_VALID;
			}
			else
			{
				pf.state[s] = ST_BAD;
				pf.bad++;
			}
		}
		pthread_mutex_unlock(&pf.lock);
		if (rr) plog("bad sector %d (C/H/S %d/%d/%d), the core gets zeros", s, s / (spt * heads), (s / spt) % heads, s % spt + 1);
	}
}

// writes one run of contiguous dirty sectors; returns 1 if something was written
static int flush_one()
{
	uint8_t buf[MAX_SPT * 512];
	int code;

	pthread_mutex_lock(&pf.lock);
	if (!pf.dirty_count || !pf.media.present)
	{
		pthread_mutex_unlock(&pf.lock);
		return 0;
	}

	int first = -1;
	for (int i = 0; i < pf.media.total_sectors; i++) if (pf.dirty[i]) { first = i; break; }
	if (first < 0)
	{
		pf.dirty_count = 0;
		pthread_mutex_unlock(&pf.lock);
		return 0;
	}

	int spt = pf.media.spt;
	int track_end = (first / spt + 1) * spt;
	int n = 0;
	while (first + n < track_end && first + n < pf.media.total_sectors && pf.dirty[first + n])
	{
		memcpy(buf + n * 512, pf.data + (first + n) * 512, 512);
		pf.dirty[first + n] = 0;
		n++;
	}
	pf.dirty_count -= n;
	uint32_t gen = pf.gen;
	pthread_mutex_unlock(&pf.lock);

	int r = scsi_rw(1, first, n, buf, &code);
	if (!r) return 1;

	if (r == 1 && SK(code) == 7)
	{
		plog("disk is write-protected: %d sectors not written", n);
		pthread_mutex_lock(&pf.lock);
		pf.lost_writes += n;
		if (!pf.media.write_protected) { pf.media.write_protected = 1; pf.event = 1; }
		pthread_mutex_unlock(&pf.lock);
		return 1;
	}

	if (handle_error(r, code, "write failed"))
	{
		plog("%d sectors not written (disk removed?)", n);
		pthread_mutex_lock(&pf.lock);
		pf.lost_writes += n;
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
			int s = first + i;
			if (pf.dirty[s]) continue; // written again meanwhile
			if (++pf.wretry[s] < WRITE_RETRIES) { pf.dirty[s] = 1; pf.dirty_count++; }
			else { pf.wretry[s] = 0; dropped++; }
		}
		pf.lost_writes += dropped;
	}
	pthread_mutex_unlock(&pf.lock);
	plog("write error at sector %d (sense %X/%02X/%02X)%s", first, SK(code), ASC(code), code & 0xFF, dropped ? ", sectors dropped" : ", will retry");
	return 1;
}

static void write_stats()
{
	double t = now_ms();
	if (t - pf.last_stats < STATS_MS) return;
	pf.last_stats = t;

	pthread_mutex_lock(&pf.lock);
	uint32_t hits = pf.hits, misses = pf.misses;
	double worst = pf.worst_wait;
	pf.hits = pf.misses = 0;
	pf.worst_wait = 0;
	pthread_mutex_unlock(&pf.lock);

	if (hits || misses) plog("stats: %u sectors from cache, %u waited for the drive, worst wait %.0f ms, bad %u, lost writes %u",
		hits, misses, worst, pf.bad, pf.lost_writes);
}

static void *worker(void *)
{
	double last_poll = 0, last_scan = 0;

	while (__atomic_load_n(&pf.alive, __ATOMIC_ACQUIRE))
	{
		if (pf.fd < 0)
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
			else if (pf.dirty_count)
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
			pthread_mutex_unlock(&pf.lock);

			write_stats();
			if (now_ms() - last_poll >= POLL_MS)
			{
				last_poll = now_ms();
				probe_media();
			}
			continue;
		}
		pthread_mutex_unlock(&pf.lock);

		if (chunk >= 0) read_chunk(chunk);
		else flush_one();
	}

	// flush pending writes before leaving
	double t0 = now_ms();
	while (pf.fd >= 0 && pf.dirty_count && now_ms() - t0 < 5000) flush_one();

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
	char path[1024];
	struct stat st;

	snprintf(path, sizeof(path), "%s", getFullPath(dir));
	if (stat(path, &st) || !S_ISDIR(st.st_mode)) return;

	int l = strlen(path);
	snprintf(path + l, sizeof(path) - l, "/%s", PFLOPPY_PLACEHOLDER);
	if (!stat(path, &st)) return;

	FILE *f = fopen(path, "wb");
	if (f)
	{
		fclose(f);
		plog("created %s", path);
	}
}

void pfloppy_attach(int unit)
{
	pthread_mutex_lock(&pf.lock);
	pf.unit = unit;
	pthread_mutex_unlock(&pf.lock);

	if (pf.thread_running) return;

	if (!pf.data) pf.data = (uint8_t*)malloc(MAX_SECTORS * 512);

	remove(LOG_PATH);
	plog("attached as floppy %c:", 'A' + unit);

	// first look done here, so the BIOS sees a disk inserted before boot
	if (pf.fd < 0 && open_drive()) probe_media();
	pf.event = 0;

	pf.alive = 1;
	if (pthread_create(&pf.thread, NULL, worker, NULL))
	{
		plog("cannot start the I/O thread");
		pf.alive = 0;
		return;
	}
	pf.thread_running = 1;
}

void pfloppy_detach(void)
{
	if (!pf.thread_running) return;

	__atomic_store_n(&pf.alive, 0, __ATOMIC_RELEASE);
	pthread_cond_signal(&pf.cond);
	pthread_join(pf.thread, NULL);
	pf.thread_running = 0;

	if (pf.fd >= 0) close_drive("detached");
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
