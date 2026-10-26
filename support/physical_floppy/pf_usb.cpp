/*
 * Physical floppy backend: USB floppy drive (UFI), SCSI commands through SG_IO.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/mount.h>
#include <scsi/sg.h>

#include "pf_drive.h"

// system calls (replaceable by the host tests)
#ifndef PF_SYS_OPEN
#define PF_SYS_OPEN  ::open
#define PF_SYS_CLOSE ::close
#define PF_SYS_IOCTL ::ioctl
#endif

#define IO_TIMEOUT_MS 8000   // spin-up + seek + read
#define MAX_SECTORS   5760

// sense: code = key<<16 | asc<<8 | ascq
#define SK(c)  (((c) >> 16) & 0xF)
#define ASC(c) (((c) >> 8) & 0xFF)

class UsbDrive : public PfDrive
{
	int fd = -1;
	char dev[64] = "";
	char last_scan[256] = "";

	int sg_exec(const uint8_t *cdb, int cdb_len, int dir, void *buf, int len, int timeout_ms, int *code);
	int tur(int *code);
	int capacity(int *total, int *blksz, int *code);
	int write_protected();
	int rw(int write, int lba, int cnt, uint8_t *buf, int *code);
	int find(char *out, int sz);
	void unmount();
	static int result(int r, int code);

public:
	int open(char *desc, int descsz) override;
	void close() override;
	int probe(int present, int changed, pfd_geometry *g, char *why, int whysz) override;
	int read_track(const pfd_geometry *g, int track, uint8_t *buf, uint8_t *st) override;
	int write_track(const pfd_geometry *g, int track, const uint8_t *buf, const uint8_t *dirty, const uint8_t *valid) override;
	int keep_spinning(const pfd_geometry *g, int track) override;
};

PfDrive *pfd_usb_create() { return new UsbDrive(); }

/* ------------------------------------------------------------------ SCSI */

// 0 = ok, 1 = check condition (see *code), -1 = transport/driver error
int UsbDrive::sg_exec(const uint8_t *cdb, int cdb_len, int dir, void *buf, int len, int timeout_ms, int *code)
{
	uint8_t sense[32] = {};
	sg_io_hdr_t io;

	if (code) *code = 0;
	if (fd < 0) return -1;

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

	if (PF_SYS_IOCTL(fd, SG_IO, &io) < 0) return -1;
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
		if (c && SK(c) == 1) return 0; // recovered error
		return 1;
	}
	if (io.driver_status & ~0x08) return -1; // anything but DRIVER_SENSE
	return 0;
}

int UsbDrive::tur(int *code)
{
	uint8_t cdb[6] = { 0x00, 0, 0, 0, 0, 0 };
	return sg_exec(cdb, 6, SG_DXFER_NONE, NULL, 0, 3000, code);
}

int UsbDrive::capacity(int *total, int *blksz, int *code)
{
	uint8_t cdb[10] = { 0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t buf[8] = {};
	int r = sg_exec(cdb, 10, SG_DXFER_FROM_DEV, buf, sizeof(buf), IO_TIMEOUT_MS, code);
	if (r) return r;
	*total = ((buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3]) + 1;
	*blksz = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];
	return 0;
}

int UsbDrive::write_protected()
{
	// UFI only supports MODE SENSE(10)
	uint8_t cdb[10] = { 0x5A, 0, 0x3F, 0, 0, 0, 0, 0, 192, 0 };
	uint8_t buf[192] = {};
	int code;
	if (sg_exec(cdb, 10, SG_DXFER_FROM_DEV, buf, sizeof(buf), 3000, &code)) return 0;
	return (buf[3] & 0x80) ? 1 : 0;
}

int UsbDrive::rw(int write, int lba, int cnt, uint8_t *buf, int *code)
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

// SCSI result -> PFD_*
int UsbDrive::result(int r, int code)
{
	if (!r) return PFD_OK;
	if (r < 0) return PFD_LOST;
	if (SK(code) == 6) return PFD_CHANGED;     // unit attention: disk changed or drive reset
	if (SK(code) == 2) return PFD_NOT_READY;   // no disk, or spinning up
	if (SK(code) == 7) return PFD_WP;
	return PFD_MEDIUM;
}

/* ------------------------------------------------------------------ discovery */

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

int UsbDrive::find(char *out, int sz)
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
			snprintf(out, sz, "/dev/%s", de->d_name);
			found = 1;
		}
	}
	closedir(d);

	if (strcmp(scan, last_scan))
	{
		snprintf(last_scan, sizeof(last_scan), "%s", scan);
		pf_log("usb disks: %s-> %s", scan[0] ? scan : "none ", found ? out : "no floppy drive");
	}
	return found;
}

// the system automounts USB disks: the floppy is used by the core, not as a folder
void UsbDrive::unmount()
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

		if (!umount2(mp, MNT_DETACH)) pf_log("unmounted %s from %s (the floppy is used by the core)", dev, mp);
		else pf_log("cannot unmount %s from %s: %s", dev, mp, strerror(errno));
	}
	fclose(f);
}

int UsbDrive::open(char *desc, int descsz)
{
	char path[64];

	// manual override (debug / drives not detected automatically): PFLOPPY_DEV=/dev/sdX
	const char *forced = getenv("PFLOPPY_DEV");
	if (forced && forced[0]) snprintf(path, sizeof(path), "%s", forced);
	else if (!find(path, sizeof(path))) return 0;

	int f = PF_SYS_OPEN(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
	if (f < 0) f = PF_SYS_OPEN(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (f < 0)
	{
		pf_log("cannot open %s: %s", path, strerror(errno));
		return 0;
	}

	fd = f;
	snprintf(dev, sizeof(dev), "%s", path);
	snprintf(desc, descsz, "USB floppy drive %s", dev);
	unmount();
	return 1;
}

void UsbDrive::close()
{
	if (fd >= 0) PF_SYS_CLOSE(fd);
	fd = -1;
	last_scan[0] = 0;
}

/* ------------------------------------------------------------------ operations */

int UsbDrive::probe(int present, int changed, pfd_geometry *g, char *why, int whysz)
{
	int code = 0;
	int r = tur(&code);

	// unit attention (disk changed / reset): ask again
	for (int i = 0; i < 3 && r == 1 && SK(code) == 6; i++)
	{
		if (ASC(code) == 0x28 || ASC(code) == 0x29) changed = 1;
		r = tur(&code);
	}

	if (r < 0)
	{
		snprintf(why, whysz, "drive not responding");
		return PFD_PROBE_LOST;
	}

	if (r == 1)
	{
		// 02/04: becoming ready, decide on the next poll
		if (SK(code) == 2 && ASC(code) == 0x04) return PFD_PROBE_BUSY;
		snprintf(why, whysz, "%s", SK(code) == 2 && ASC(code) == 0x3A ? "no disk" : "drive not ready");
		return PFD_PROBE_NODISK;
	}

	if (present && !changed) return PFD_PROBE_SAME;

	int total = 0, blksz = 0;
	r = capacity(&total, &blksz, &code);
	if (r < 0)
	{
		snprintf(why, whysz, "drive not responding");
		return PFD_PROBE_LOST;
	}
	if (r)
	{
		snprintf(why, whysz, "disk present but not readable (sense %X/%02X/%02X), unformatted?", SK(code), ASC(code), code & 0xFF);
		return PFD_PROBE_UNREADABLE;
	}

	if (blksz != 512 || total <= 0 || total > MAX_SECTORS)
	{
		snprintf(why, whysz, "unsupported disk: %d sectors of %d bytes", total, blksz);
		return PFD_PROBE_UNREADABLE;
	}

	g->total = total;
	pf_calc_geometry(total, &g->spt, &g->heads, &g->cyl);
	g->wp = write_protected();
	unmount();
	return PFD_PROBE_NEW;
}

int UsbDrive::read_track(const pfd_geometry *g, int track, uint8_t *buf, uint8_t *st)
{
	int code;
	int first = track * g->spt;
	int n = g->spt;
	if (first + n > g->total) n = g->total - first;
	if (n <= 0) return PFD_MEDIUM;

	int r = rw(0, first, n, buf, &code);
	if (!r)
	{
		memset(st, 1, n);
		return PFD_OK;
	}

	int res = result(r, code);
	if (res != PFD_MEDIUM) return res;

	// medium error: find the bad sectors one by one
	pf_log("read error on track %d (sense %X/%02X/%02X), retrying sector by sector", track, SK(code), ASC(code), code & 0xFF);
	for (int i = 0; i < n; i++)
	{
		int rr = 1;
		for (int k = 0; k < 2 && rr; k++)
		{
			rr = rw(0, first + i, 1, buf + i * 512, &code);
			// disk changed/removed or drive lost: stop here, never retry on another disk
			if (rr && result(rr, code) != PFD_MEDIUM) return result(rr, code);
		}
		st[i] = rr ? 2 : 1;
		if (rr) memset(buf + i * 512, 0, 512);
	}
	return PFD_OK;
}

// writes each run of consecutive changed sectors with one command
int UsbDrive::write_track(const pfd_geometry *g, int track, const uint8_t *buf, const uint8_t *dirty, const uint8_t *valid)
{
	(void)valid;
	int first = track * g->spt;
	int n = g->spt;
	if (first + n > g->total) n = g->total - first;

	for (int i = 0; i < n; )
	{
		if (!dirty[i]) { i++; continue; }
		int run = 1;
		while (i + run < n && dirty[i + run]) run++;

		int code;
		int r = rw(1, first + i, run, (uint8_t*)buf + i * 512, &code);
		int res = result(r, code);
		if (res == PFD_MEDIUM) pf_log("write error at sector %d (sense %X/%02X/%02X)", first + i, SK(code), ASC(code), code & 0xFF);
		if (res != PFD_OK) return res;
		i += run;
	}
	return PFD_OK;
}

// a USB drive stops its motor about a second after the last command: a small read keeps it going
int UsbDrive::keep_spinning(const pfd_geometry *g, int track)
{
	static uint8_t dummy[512];
	int code;
	int lba = track > 0 ? track * g->spt : 0;
	int r = rw(0, lba, 1, dummy, &code);
	int res = result(r, code);
	return res == PFD_MEDIUM ? PFD_OK : res;
}
