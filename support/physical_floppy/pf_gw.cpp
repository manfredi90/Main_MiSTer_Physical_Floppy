/*
 * Physical floppy backend: Greaseweazle (V4/V4.1, CDC ACM serial device).
 *
 * The drive is read as flux; the IBM PC MFM sectors are decoded here
 * (mfm_ibm.cpp). This first version is read-only: the disk is presented to
 * the core as write-protected.
 *
 * Protocol: see the Greaseweazle host tools and firmware (public domain,
 * Keir Fraser), cdc_acm_protocol.h.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <dirent.h>
#include <poll.h>
#include <termios.h>
#include <vector>

#include "pf_drive.h"
#include "mfm_ibm.h"
#include "../../cfg.h"

int gw_set_baud(int fd, int baud);

// read() on the serial port (replaceable by the host tests)
#ifndef PF_GW_READ
#define PF_GW_READ ::read
#endif

// commands
enum
{
	GW_GET_INFO = 0, GW_SEEK = 2, GW_HEAD = 3, GW_MOTOR = 6, GW_READ_FLUX = 7,
	GW_GET_FLUX_STATUS = 9, GW_SELECT = 12, GW_DESELECT = 13, GW_SET_BUS_TYPE = 14,
	GW_RESET = 16, GW_GET_PIN = 20, GW_NOCLICK_STEP = 22
};

// acks
enum { ACK_OKAY = 0, ACK_BAD_COMMAND = 1, ACK_NO_INDEX = 2, ACK_FLUX_OVERFLOW = 4, ACK_BAD_CYLINDER = 11 };

#define BUS_IBMPC 1
#define READ_ATTEMPTS 3

class GwDrive : public PfDrive
{
	int fd = -1;
	char path[64] = "";
	char last_scan[256] = "";
	double freq = 72e6;
	int unit = 0;
	int unit_ok = 0;       // a drive answered on 'unit'
	int unit_logged = 0;
	int selected = 0, motor = 0;
	int double_step = 0;
	double cell_ns = 1000;
	int noclick = 1;
	int ro_logged = 0;
	char err[160] = "";      // last communication error
	int err_logged = 0;
	int comm_fails = 0;      // consecutive communication errors

	int wr(const uint8_t *b, int n);
	int rd(uint8_t *b, int n, int timeout_ms);
	int cmd(const uint8_t *c, int timeout_ms = 3000);
	int get_pin(int pin);
	int select(int on);
	int motor_on();
	void motor_off();
	int seek(int cyl);
	int head(int h);
	int read_flux(int revs, std::vector<uint32_t> &flux);
	int find(char *out, int sz);
	int find_unit();
	int resync();
	int comm_error();
	int detect(pfd_geometry *g, char *why, int whysz);
	int read_good(int cyl, int hd, int heads, int spt, std::vector<mfm_sector> &secs);

public:
	int open(char *desc, int descsz) override;
	void close() override;
	int probe(int present, int changed, pfd_geometry *g, char *why, int whysz) override;
	int read_track(const pfd_geometry *g, int track, uint8_t *buf, uint8_t *st) override;
	int write_sectors(const pfd_geometry *g, int lba, int n, const uint8_t *buf) override;
	int keep_spinning(const pfd_geometry *g, int track) override;
	void idle(double idle_ms) override;
};

PfDrive *pfd_gw_create() { return new GwDrive(); }

/* ------------------------------------------------------------------ serial */

int GwDrive::wr(const uint8_t *b, int n)
{
	while (n > 0)
	{
		int r = write(fd, b, n);
		if (r < 0)
		{
			if (errno == EINTR || errno == EAGAIN) { usleep(1000); continue; }
			return -1;
		}
		b += r;
		n -= r;
	}
	return 0;
}

int GwDrive::rd(uint8_t *b, int n, int timeout_ms)
{
	double deadline = pf_now_ms() + timeout_ms;
	int want = n;
	while (n > 0)
	{
		int left = (int)(deadline - pf_now_ms());
		if (left <= 0)
		{
			snprintf(err, sizeof(err), "timeout after %d ms (%d of %d bytes)", timeout_ms, want - n, want);
			return -1;
		}
		struct pollfd p = { fd, POLLIN, 0 };
		int r = poll(&p, 1, left);
		if (r < 0)
		{
			if (errno == EINTR) continue;
			snprintf(err, sizeof(err), "poll: %s", strerror(errno));
			return -1;
		}
		if (!r) continue;
		if (p.revents & (POLLERR | POLLHUP | POLLNVAL))
		{
			snprintf(err, sizeof(err), "port error (revents %x)", p.revents);
			return -1;
		}
		r = PF_GW_READ(fd, b, n);
		if (r < 0)
		{
			if (errno == EINTR || errno == EAGAIN) continue;
			snprintf(err, sizeof(err), "read: %s", strerror(errno));
			return -1;
		}
		if (r == 0)
		{
			usleep(1000); // nothing yet
			continue;
		}
		b += r;
		n -= r;
	}
	return 0;
}

// sends a command (c[1] = length). returns the ack code, -1 on a communication error
int GwDrive::cmd(const uint8_t *c, int timeout_ms)
{
	if (fd < 0) return -1;
	if (wr(c, c[1]))
	{
		snprintf(err, sizeof(err), "command %d: write: %s", c[0], strerror(errno));
		return -1;
	}
	uint8_t ack[2];
	if (rd(ack, 2, timeout_ms))
	{
		char e[160];
		snprintf(e, sizeof(e), "command %d: no answer, %s", c[0], err);
		snprintf(err, sizeof(err), "%s", e);
		return -1;
	}
	if (ack[0] != c[0])
	{
		snprintf(err, sizeof(err), "command %d: wrong answer %02X %02X", c[0], ack[0], ack[1]);
		return -1;
	}
	comm_fails = 0;
	return ack[1];
}

// resets the communication state of the firmware (10000 baud signal) and checks it answers
int GwDrive::resync()
{
	tcflush(fd, TCIOFLUSH);
	gw_set_baud(fd, 10000);
	gw_set_baud(fd, 9600);
	usleep(20000);
	tcflush(fd, TCIFLUSH);

	uint8_t info_cmd[3] = { GW_GET_INFO, 3, 0 };
	uint8_t info[32];
	if (wr(info_cmd, 3)) return 0;
	uint8_t ack[2];
	if (rd(ack, 2, 1000) || ack[0] != GW_GET_INFO || ack[1] || rd(info, 32, 1000)) return 0;
	return 1;
}

// a command failed: log it, resynchronise. returns 1 if the Greaseweazle answers again
// (the operation can be retried), 0 if it is gone
int GwDrive::comm_error()
{
	comm_fails++;
	if (err_logged < 30)
	{
		err_logged++;
		pf_log("Greaseweazle communication error: %s%s", err, err_logged == 30 ? " (further errors not logged)" : "");
	}
	if (comm_fails > 5) return 0;
	int ok = resync();
	if (!ok) pf_log("Greaseweazle: no answer after a communication reset");
	return ok;
}

int GwDrive::get_pin(int pin)
{
	uint8_t c[3] = { GW_GET_PIN, 3, (uint8_t)pin };
	if (cmd(c) != ACK_OKAY) return -1;
	uint8_t v;
	if (rd(&v, 1, 1000)) return -1;
	return v ? 1 : 0;
}

int GwDrive::select(int on)
{
	if (on == selected) return 0;
	int r;
	if (on)
	{
		uint8_t c[3] = { GW_SELECT, 3, (uint8_t)unit };
		r = cmd(c);
	}
	else
	{
		uint8_t c[2] = { GW_DESELECT, 2 };
		r = cmd(c);
	}
	if (r) return -1;
	selected = on;
	return 0;
}

// the firmware waits for the motor to spin up (750 ms) before acknowledging
int GwDrive::motor_on()
{
	if (select(1)) return -1;
	if (motor) return 0;
	uint8_t c[4] = { GW_MOTOR, 4, (uint8_t)unit, 1 };
	if (cmd(c, 5000)) return -1;
	motor = 1;
	return 0;
}

void GwDrive::motor_off()
{
	if (motor)
	{
		uint8_t c[4] = { GW_MOTOR, 4, (uint8_t)unit, 0 };
		cmd(c);
		motor = 0;
	}
	select(0);
}

int GwDrive::seek(int cyl)
{
	uint8_t c[3] = { GW_SEEK, 3, (uint8_t)(int8_t)cyl };
	return cmd(c, 5000);
}

int GwDrive::head(int h)
{
	uint8_t c[3] = { GW_HEAD, 3, (uint8_t)h };
	return cmd(c);
}

// reads revs revolutions (plus the part before the first index pulse).
// returns the flux status (ACK_*), -1 on a communication error
int GwDrive::read_flux(int revs, std::vector<uint32_t> &flux)
{
	uint16_t max_index = revs + 1;
	uint8_t c[8] = { GW_READ_FLUX, 8, 0, 0, 0, 0, (uint8_t)(max_index & 0xFF), (uint8_t)(max_index >> 8) };
	int r = cmd(c);
	if (r) return r;

	static std::vector<uint8_t> s;
	s.clear();
	uint8_t buf[4096];
	// a zero-length USB packet (sent when a transfer ends on a packet boundary)
	// wakes poll() but read() returns 0: that is not an error
	double deadline = pf_now_ms() + 3000;
	for (;;)
	{
		int left = (int)(deadline - pf_now_ms());
		if (left <= 0)
		{
			snprintf(err, sizeof(err), "flux stream stalled (%zu bytes received)", s.size());
			return -1;
		}
		struct pollfd p = { fd, POLLIN, 0 };
		int pr = poll(&p, 1, left);
		if (pr < 0 && errno != EINTR)
		{
			snprintf(err, sizeof(err), "flux stream: poll: %s", strerror(errno));
			return -1;
		}
		if (pr <= 0) continue;
		int n = PF_GW_READ(fd, buf, sizeof(buf));
		if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
		if (n < 0)
		{
			snprintf(err, sizeof(err), "flux stream: read: %s", strerror(errno));
			return -1;
		}
		if (n == 0)
		{
			usleep(1000);
			continue;
		}
		deadline = pf_now_ms() + 3000; // data is flowing
		s.insert(s.end(), buf, buf + n);
		if (s.back() == 0) break; // end of stream (0 never appears inside it)
	}

	uint8_t st[2] = { GW_GET_FLUX_STATUS, 2 };
	r = cmd(st);
	if (r < 0) return -1;

	// decode the stream into flux intervals
	flux.clear();
	flux.reserve(s.size());
	uint32_t ticks = 0;
	size_t i = 0, n = s.size() - 1;
	while (i < n)
	{
		uint8_t b = s[i++];
		if (b == 255)
		{
			if (i + 5 > n) break;
			uint8_t op = s[i++];
			uint32_t v = ((s[i] & 254) >> 1) | ((s[i + 1] & 254) << 6) | ((s[i + 2] & 254) << 13) | ((uint32_t)(s[i + 3] & 254) << 20);
			i += 4;
			if (op == 2) ticks += v; // space; 1 = index (only the flux matters here)
		}
		else if (b < 250)
		{
			flux.push_back(ticks + b);
			ticks = 0;
		}
		else
		{
			if (i >= n) break;
			flux.push_back(ticks + 250 + (b - 250) * 255 + s[i++] - 1);
			ticks = 0;
		}
	}
	return r;
}

/* ------------------------------------------------------------------ discovery */

static int gw_read_sys(const char *path, char *out, int sz)
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

int GwDrive::find(char *out, int sz)
{
	char scan[256] = "";
	int found = 0;

	DIR *d = opendir("/sys/class/tty");
	if (!d) return 0;

	struct dirent *de;
	while ((de = readdir(d)))
	{
		if (strncmp(de->d_name, "ttyACM", 6)) continue;

		char p[PATH_MAX], real[PATH_MAX], vid[16], pid[16];
		snprintf(p, sizeof(p), "/sys/class/tty/%s/device", de->d_name);
		if (!realpath(p, real)) continue;

		// usb interface -> usb device
		char *slash = strrchr(real, '/');
		if (slash) *slash = 0;
		snprintf(p, sizeof(p), "%s/idVendor", real);
		gw_read_sys(p, vid, sizeof(vid));
		snprintf(p, sizeof(p), "%s/idProduct", real);
		gw_read_sys(p, pid, sizeof(pid));

		int l = strlen(scan);
		snprintf(scan + l, sizeof(scan) - l, "%s[%s:%s] ", de->d_name, vid, pid);

		if (!found && !strcasecmp(vid, "1209") && !strcasecmp(pid, "4d69"))
		{
			snprintf(out, sz, "/dev/%s", de->d_name);
			found = 1;
		}
	}
	closedir(d);

	if (strcmp(scan, last_scan))
	{
		snprintf(last_scan, sizeof(last_scan), "%s", scan);
		if (scan[0]) pf_log("usb serial ports: %s-> %s", scan, found ? out : "no Greaseweazle");
	}
	return found;
}

int GwDrive::open(char *desc, int descsz)
{
	char p[64];

	// manual override: PFLOPPY_GW_DEV=/dev/ttyACMx
	const char *forced = getenv("PFLOPPY_GW_DEV");
	if (forced && forced[0]) snprintf(p, sizeof(p), "%s", forced);
	else if (!find(p, sizeof(p))) return 0;

	fd = ::open(p, O_RDWR | O_NOCTTY | O_CLOEXEC);
	if (fd < 0)
	{
		pf_log("cannot open %s: %s", p, strerror(errno));
		return 0;
	}

	struct termios t;
	if (!tcgetattr(fd, &t))
	{
		cfmakeraw(&t);
		t.c_cflag |= CLOCAL | CREAD;
		t.c_cc[VMIN] = 0;
		t.c_cc[VTIME] = 0;
		tcsetattr(fd, TCSANOW, &t);
	}

	// reset the communication state (in case a previous session was interrupted)
	tcflush(fd, TCIOFLUSH);
	gw_set_baud(fd, 10000);
	gw_set_baud(fd, 9600);
	usleep(20000);
	tcflush(fd, TCIFLUSH);

	uint8_t info_cmd[3] = { GW_GET_INFO, 3, 0 };
	uint8_t info[32];
	if (cmd(info_cmd) != ACK_OKAY || rd(info, 32, 1000))
	{
		pf_log("%s: no answer from the Greaseweazle", p);
		close();
		return 0;
	}

	int major = info[0], minor = info[1], main_fw = info[2];
	if (!main_fw)
	{
		pf_log("%s: the Greaseweazle is in firmware update mode", p);
		close();
		return 0;
	}
	freq = info[4] | (info[5] << 8) | (info[6] << 16) | ((uint32_t)info[7] << 24);
	int model = info[8], submodel = info[9];

	uint8_t reset[2] = { GW_RESET, 2 };
	uint8_t bus[3] = { GW_SET_BUS_TYPE, 3, BUS_IBMPC };
	if (cmd(reset) || cmd(bus))
	{
		pf_log("%s: Greaseweazle setup failed", p);
		close();
		return 0;
	}

	// MiSTer.ini PFLOPPY_GW_DRIVE: 0 = automatic, 1 = drive A (after the cable twist), 2 = drive B
	unit = cfg.pfloppy_gw_drive == 2 ? 1 : 0;
	unit_ok = 0;
	unit_logged = 0;
	selected = motor = 0;

	snprintf(path, sizeof(path), "%s", p);
	snprintf(desc, descsz, "Greaseweazle model %d.%d firmware %d.%d on %s, sample clock %.0f MHz",
		model, submodel, major, minor, path, freq / 1e6);
	return 1;
}

void GwDrive::close()
{
	if (fd >= 0)
	{
		motor_off();
		::close(fd);
	}
	fd = -1;
	selected = motor = 0;
	last_scan[0] = 0;
}

/* ------------------------------------------------------------------ disk */

// reads one track and collects its good sectors (a few attempts for weak sectors)
int GwDrive::read_good(int cyl, int hd, int heads, int spt, std::vector<mfm_sector> &secs)
{
	std::vector<uint32_t> flux;
	std::vector<mfm_sector> out;
	uint8_t got[64] = {};
	int ngot = 0;

	secs.clear();
	if (motor_on()) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	int r = seek(double_step ? cyl * 2 : cyl);
	if (r < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	if (r) return PFD_MEDIUM;
	if (head(hd) < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;

	for (int attempt = 0; attempt < READ_ATTEMPTS; attempt++)
	{
		r = read_flux(1, flux);
		if (r < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
		if (r == ACK_NO_INDEX) return PFD_CHANGED; // no rotation: disk removed
		if (r) continue;                           // overflow etc: read again

		mfm_ibm_decode(flux, freq, cell_ns, out);
		for (auto &s : out)
		{
			if (!s.ok || s.n != 2 || s.sec < 1 || s.sec > 63) continue;
			if (s.cyl != cyl || (heads > 1 && s.head != hd)) continue;
			if (got[s.sec]) continue;
			got[s.sec] = 1;
			ngot++;
			secs.push_back(s);
		}
		if (spt && ngot >= spt) break;
		if (!spt && ngot) break;
	}
	return PFD_OK;
}

static int max_sector(const std::vector<mfm_sector> &secs)
{
	int m = 0;
	for (auto &s : secs) if (s.sec > m) m = s.sec;
	return m;
}

int GwDrive::detect(pfd_geometry *g, char *why, int whysz)
{
	std::vector<uint32_t> flux;
	std::vector<mfm_sector> secs;

	double_step = 0;
	if (motor_on() || seek(0) < 0 || head(0) < 0)
	{
		snprintf(why, whysz, "Greaseweazle not responding");
		return comm_error() ? PFD_PROBE_BUSY : PFD_PROBE_LOST;
	}

	int r = read_flux(1, flux);
	if (r < 0)
	{
		snprintf(why, whysz, "Greaseweazle not responding");
		return comm_error() ? PFD_PROBE_BUSY : PFD_PROBE_LOST;
	}
	if (r == ACK_NO_INDEX) return PFD_PROBE_BUSY; // not spinning yet

	double cell = mfm_guess_cell_ns(flux, freq);
	if (!cell)
	{
		snprintf(why, whysz, "disk present but track 0 has no MFM data (unformatted, or not a PC disk)");
		return PFD_PROBE_UNREADABLE;
	}
	cell_ns = cell;

	r = read_good(0, 0, 1, 0, secs);
	if (r == PFD_LOST) { snprintf(why, whysz, "Greaseweazle not responding"); return PFD_PROBE_LOST; }
	if (r == PFD_NOT_READY || r == PFD_CHANGED) return PFD_PROBE_BUSY;
	int spt = max_sector(secs);
	if (!spt || spt > 36 || (int)secs.size() < spt / 2)
	{
		snprintf(why, whysz, "disk present but no PC sectors on track 0 (%s, Amiga/Mac disk or unformatted?)", cell == 1000 ? "HD" : "DD");
		return PFD_PROBE_UNREADABLE;
	}

	r = read_good(0, 1, 2, 0, secs);
	if (r == PFD_LOST) { snprintf(why, whysz, "Greaseweazle not responding"); return PFD_PROBE_LOST; }
	if (r == PFD_NOT_READY || r == PFD_CHANGED) return PFD_PROBE_BUSY;
	int heads = secs.size() ? 2 : 1;

	int cyl = 80;
	if (cell == 2000)
	{
		// a 40-track disk (360K) in an 80-track drive: physical cylinder 2 holds cylinder 1
		if (seek(2) == ACK_OKAY && head(0) == ACK_OKAY && read_flux(1, flux) == ACK_OKAY)
		{
			std::vector<mfm_sector> out;
			mfm_ibm_decode(flux, freq, cell_ns, out);
			for (auto &s : out)
			{
				if (!s.ok) continue;
				if (s.cyl == 1) { double_step = 1; cyl = 40; }
				break;
			}
		}
	}

	int wp = get_pin(28);
	g->spt = spt;
	g->heads = heads;
	g->cyl = cyl;
	g->total = spt * heads * cyl;
	g->wp = 1; // read-only for now (writing needs the MFM track encoder on the drive)

	if (!ro_logged)
	{
		ro_logged = 1;
		pf_log("Greaseweazle: reading only for now, the disk is shown to the core as write-protected");
	}
	pf_log("format: %s, %d sectors/track, %d head(s), %d cylinders%s%s", cell == 1000 ? "HD 500 kbps" : "DD 250 kbps",
		spt, heads, cyl, double_step ? " (double step)" : "", wp == 0 ? ", write-protect tab set" : "");
	return PFD_PROBE_NEW;
}

// finds the drive: only a connected drive asserts /TRK0, so a recalibration
// (seek to cylinder 0) succeeds only on the right unit. Works without a disk.
// returns 1 found, 0 not found, -1 communication error
int GwDrive::find_unit()
{
	int units[2], n = 0;
	if (cfg.pfloppy_gw_drive == 1) units[n++] = 0;
	else if (cfg.pfloppy_gw_drive == 2) units[n++] = 1;
	else { units[n++] = 0; units[n++] = 1; }

	for (int i = 0; i < n; i++)
	{
		motor_off();
		unit = units[i];
		if (select(1)) return -1;
		int r = seek(0);
		if (r < 0) return -1;
		if (r == ACK_OKAY)
		{
			unit_ok = 1;
			pf_log("Greaseweazle: floppy drive found as drive %c%s", 'A' + unit,
				cfg.pfloppy_gw_drive ? "" : " (automatic, MiSTer.ini PFLOPPY_GW_DRIVE=1/2 forces A/B)");
			return 1;
		}
	}

	select(0);
	if (!unit_logged)
	{
		unit_logged = 1;
		pf_log("Greaseweazle: no floppy drive answers as %s (check cable, power, PFLOPPY_GW_DRIVE)",
			cfg.pfloppy_gw_drive == 1 ? "drive A" : cfg.pfloppy_gw_drive == 2 ? "drive B" : "drive A or B");
	}
	return 0;
}

// PC drives: pin 34 = /DISKCHG, low after the disk was removed, cleared by a
// step pulse when a disk is in. The drive must be selected to read it.
int GwDrive::probe(int present, int changed, pfd_geometry *g, char *why, int whysz)
{
	if (!unit_ok)
	{
		int r = find_unit();
		if (r < 0)
		{
			snprintf(why, whysz, "Greaseweazle not responding");
			return comm_error() ? PFD_PROBE_BUSY : PFD_PROBE_LOST;
		}
		if (!r)
		{
			snprintf(why, whysz, "no floppy drive connected to the Greaseweazle");
			return PFD_PROBE_NODISK;
		}
	}

	int was_selected = selected;
	if (select(1))
	{
		snprintf(why, whysz, "Greaseweazle not responding");
		return comm_error() ? PFD_PROBE_BUSY : PFD_PROBE_LOST;
	}

	int res;
	int chg = get_pin(34);
	if (chg < 0)
	{
		snprintf(why, whysz, "Greaseweazle not responding");
		return comm_error() ? PFD_PROBE_BUSY : PFD_PROBE_LOST;
	}

	if (chg == 1 && present && !changed)
	{
		res = PFD_PROBE_SAME;
	}
	else
	{
		if (chg == 0)
		{
			// step to clear the latch: if it stays low there is no disk
			uint8_t c[2] = { GW_NOCLICK_STEP, 2 };
			int r = noclick ? cmd(c, 5000) : ACK_BAD_COMMAND;
			if (r == ACK_BAD_COMMAND)
			{
				// old firmware: a real step out and back
				noclick = 0;
				seek(1);
				r = seek(0);
			}
			if (r < 0)
			{
				snprintf(why, whysz, "Greaseweazle not responding");
				return comm_error() ? PFD_PROBE_BUSY : PFD_PROBE_LOST;
			}
			chg = get_pin(34);
		}

		if (chg != 1)
		{
			snprintf(why, whysz, "no disk");
			res = PFD_PROBE_NODISK;
		}
		else
		{
			res = detect(g, why, whysz);
		}
	}

	if (!motor && !was_selected) select(0);
	return res;
}

int GwDrive::read_track(const pfd_geometry *g, int track, uint8_t *buf, uint8_t *st)
{
	int cyl = track / g->heads;
	int hd = track % g->heads;

	if (select(1)) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	int chg = get_pin(34);
	if (chg < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	if (chg == 0) return PFD_CHANGED;

	std::vector<mfm_sector> secs;
	int r = read_good(cyl, hd, g->heads, g->spt, secs);
	if (r != PFD_OK) return r;

	memset(st, 2, g->spt);
	memset(buf, 0, g->spt * 512);
	for (auto &s : secs)
	{
		if (s.sec > g->spt) continue;
		memcpy(buf + (s.sec - 1) * 512, s.data, 512);
		st[s.sec - 1] = 1;
	}
	return PFD_OK;
}

int GwDrive::write_sectors(const pfd_geometry *g, int lba, int n, const uint8_t *buf)
{
	(void)g; (void)lba; (void)n; (void)buf;
	return PFD_WP;
}

// the Greaseweazle keeps the motor on until told otherwise
int GwDrive::keep_spinning(const pfd_geometry *g, int track)
{
	(void)g; (void)track;
	return PFD_OK;
}

// motor-off delay, like a PC
void GwDrive::idle(double idle_ms)
{
	if (motor && idle_ms > PF_MOTOR_KEEP_MS) motor_off();
}
