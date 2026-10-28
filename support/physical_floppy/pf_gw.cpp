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
#include <math.h>
#include <vector>
#include <algorithm>

#include "pf_drive.h"
#include "mfm_ibm.h"
#include "mfm_amiga.h"
#include "gcr_mac.h"

#define MAC_READ_S 0.235   // Mac GCR track read: 1.15 revolutions at 300 RPM + 5 ms
#include "../../cfg.h"

int gw_set_baud(int fd, int baud);
int gw_read_sys(const char *path, char *out, int sz);

// read() on the serial port (replaceable by the host tests)
#ifndef PF_GW_READ
#define PF_GW_READ ::read
#endif

// commands
enum
{
	GW_GET_INFO = 0, GW_SET_PARAMS = 4, GW_GET_PARAMS = 5, GW_SEEK = 2, GW_HEAD = 3, GW_MOTOR = 6, GW_READ_FLUX = 7, GW_WRITE_FLUX = 8,
	GW_GET_FLUX_STATUS = 9, GW_SELECT = 12, GW_DESELECT = 13, GW_SET_BUS_TYPE = 14,
	GW_RESET = 16, GW_GET_PIN = 20, GW_NOCLICK_STEP = 22
};

// acks
enum { ACK_OKAY = 0, ACK_BAD_COMMAND = 1, ACK_NO_INDEX = 2, ACK_FLUX_OVERFLOW = 4, ACK_FLUX_UNDERFLOW = 5, ACK_WRPROT = 6, ACK_BAD_CYLINDER = 11 };

#define BUS_IBMPC 1
#define READ_ATTEMPTS 3
#define READ_ATTEMPTS_CRC 5   // sectors still missing with CRC errors: a weak sector, a few more tries

class GwDrive : public PfDrive, public PfFluxDrive
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
	int amiga = 0;           // AmigaDOS disks (11 sectors of 512 bytes per track) instead of PC disks
	int mac = 0;             // Macintosh 400K/800K GCR disks (12 to 8 sectors per track by zone)
	int mac_sd = 0;          // Mac SuperDrive: MFM disks (720K, 1.44 MB) too
	int mac_mfm = 0;         // the Mac disk in the drive is MFM (read as a PC disk)
	int gcr() const { return mac && !mac_mfm; }
	double cell_ns = 1000;
	int noclick = 1;
	double rev_ticks = 0;    // measured revolution time (sample ticks)
	double last_users_check = -1e9;
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
	int read_flux(int revs, std::vector<uint32_t> &flux, std::vector<uint32_t> *idx_pos = NULL, std::vector<uint32_t> *idx_off = NULL);
	int read_flux_cmd(uint32_t ticks, uint16_t max_index, std::vector<uint32_t> &flux, std::vector<uint32_t> *idx_pos, std::vector<uint32_t> *idx_off);
	int splice_rev(const std::vector<uint32_t> &flux, const std::vector<double> &t, double a, pfd_rev &rv);
	int check_disk(int *was_changed);
	int find(char *out, int sz);
	void set_motor_delay();
	int find_unit();
	int resync();
	int comm_error();
	int write_flux(const std::vector<uint32_t> &flux);
	void check_port_users(int always);
	int detect(pfd_geometry *g, char *why, int whysz);
	int read_good(int cyl, int hd, int heads, int spt, std::vector<mfm_sector> &secs);

	void decode(const std::vector<uint32_t> &flux, int cyl, std::vector<mfm_sector> &out);
	int detect_amiga(pfd_geometry *g, char *why, int whysz);
	int detect_mac(pfd_geometry *g, char *why, int whysz);
	int detect_pc(pfd_geometry *g, char *why, int whysz);

public:
	// mac_disks: 1 = Mac GCR disks, 2 = GCR and MFM (SuperDrive)
	GwDrive(int amiga_disks = 0, int mac_disks = 0) : amiga(amiga_disks), mac(mac_disks != 0), mac_sd(mac_disks == 2) {}
	int open(char *desc, int descsz) override;
	void close() override;
	int probe(int present, int changed, pfd_geometry *g, char *why, int whysz) override;
	int read_track(const pfd_geometry *g, int track, uint8_t *buf, uint8_t *st) override;
	int write_track(const pfd_geometry *g, int track, const uint8_t *buf, const uint8_t *dirty, const uint8_t *valid) override;
	int keep_spinning(const pfd_geometry *g, int track) override;
	void idle(double idle_ms) override;

	// PfFluxDrive
	double sample_hz() override { return freq; }
	int flux_probe(int present, char *why, int whysz) override;
	int read_revs(int cyl, int head, int revs, std::vector<pfd_rev> &out) override;
	int flux_seek(int cyl) override;
	void flux_motor_off() override { motor_off(); }
	int flux_motor_is_on() override { return motor; }
};

PfDrive *pfd_gw_create() { return new GwDrive(); }
PfDrive *pfd_gw_create_amiga() { return new GwDrive(1); }
PfDrive *pfd_gw_create_mac(int superdrive) { return new GwDrive(0, superdrive ? 2 : 1); }
PfFluxDrive *pfd_gw_flux_create() { return new GwDrive(); }

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
	if (strstr(err, "wrong answer") || strstr(err, "no answer")) check_port_users(0);
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

// the time the firmware waits for the motor to spin up before acknowledging a
// motor on (750 ms by default). The cores don't know the motor was off: a shorter
// time answers the first sector sooner after a pause. MiSTer.ini
// PFLOPPY_GW_MOTOR_MS sets it (0 = the firmware default)
void GwDrive::set_motor_delay()
{
	int ms = cfg.pfloppy_gw_motor_ms;
	if (!ms) return;
	if (ms < 100) ms = 100;

	// delay parameters: select (us), step (us), seek settle (ms), motor (ms), ...
	uint8_t get[4] = { GW_GET_PARAMS, 4, 0, 8 };
	uint8_t d[8];
	if (cmd(get) || rd(d, 8, 1000))
	{
		pf_log("Greaseweazle: cannot read the drive delays, motor spin-up left as it is");
		return;
	}
	int old = d[6] | (d[7] << 8);
	if (old == ms) return;
	uint8_t set[11] = { GW_SET_PARAMS, 11, 0 };
	memcpy(set + 3, d, 8);
	set[3 + 6] = ms & 0xFF;
	set[3 + 7] = ms >> 8;
	if (cmd(set)) pf_log("Greaseweazle: cannot set the motor spin-up time");
	else pf_log("Greaseweazle: motor spin-up %d ms (was %d)", ms, old);
}

// the firmware waits for the motor to spin up (set_motor_delay) before acknowledging
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
// idx_pos/idx_off (optional): for every index pulse, the number of flux intervals
// before it and the ticks from the last flux transition to the pulse.
// returns the flux status (ACK_*), -1 on a communication error
int GwDrive::read_flux(int revs, std::vector<uint32_t> &flux, std::vector<uint32_t> *idx_pos, std::vector<uint32_t> *idx_off)
{
	return read_flux_cmd(0, revs + 1, flux, idx_pos, idx_off);
}

// ticks: stop after this many sample ticks, max_index: after this many index pulses (0 = no limit)
int GwDrive::read_flux_cmd(uint32_t ticks_max, uint16_t max_index, std::vector<uint32_t> &flux, std::vector<uint32_t> *idx_pos, std::vector<uint32_t> *idx_off)
{
	if (idx_pos) idx_pos->clear();
	if (idx_off) idx_off->clear();
	uint8_t c[8] = { GW_READ_FLUX, 8, (uint8_t)ticks_max, (uint8_t)(ticks_max >> 8), (uint8_t)(ticks_max >> 16), (uint8_t)(ticks_max >> 24),
		(uint8_t)(max_index & 0xFF), (uint8_t)(max_index >> 8) };
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
	// (index markers give the revolution time, used to size written tracks)
	flux.clear();
	flux.reserve(s.size());
	uint32_t ticks = 0;
	int64_t since_index = 0;
	int seen_index = 0;
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
			if (op == 2) ticks += v; // space
			else if (op == 1)        // index
			{
				int64_t rev = since_index + ticks + v;
				if (seen_index && rev > freq * 0.15 && rev < freq * 0.25) rev_ticks = (double)rev;
				if (idx_pos) idx_pos->push_back(flux.size());
				if (idx_off) idx_off->push_back(ticks + v);
				since_index = -(int64_t)(ticks + v);
				seen_index = 1;
			}
		}
		else if (b < 250)
		{
			flux.push_back(ticks + b);
			since_index += ticks + b;
			ticks = 0;
		}
		else
		{
			if (i >= n) break;
			uint32_t v = ticks + 250 + (b - 250) * 255 + s[i++] - 1;
			flux.push_back(v);
			since_index += v;
			ticks = 0;
		}
	}
	return r;
}

/* ------------------------------------------------------------------ discovery */

// other programs that have the Greaseweazle port open (e.g. a service probing
// serial ports for NFC readers) send it data and steal its answers: tell the user
void GwDrive::check_port_users(int always)
{
	if (!always && pf_now_ms() - last_users_check < 30000) return;
	last_users_check = pf_now_ms();

	char port[PATH_MAX];
	if (!realpath(path, port)) snprintf(port, sizeof(port), "%s", path);

	char list[256] = "";
	int zaparoo = 0;
	DIR *d = opendir("/proc");
	if (!d) return;
	struct dirent *de;
	while ((de = readdir(d)))
	{
		int pid = atoi(de->d_name);
		if (pid <= 0 || pid == getpid()) continue;

		char p[64];
		snprintf(p, sizeof(p), "/proc/%d/fd", pid);
		DIR *fdd = opendir(p);
		if (!fdd) continue;

		int found = 0;
		struct dirent *fe;
		while (!found && (fe = readdir(fdd)))
		{
			char lp[96], target[PATH_MAX];
			snprintf(lp, sizeof(lp), "/proc/%d/fd/%s", pid, fe->d_name);
			ssize_t n = readlink(lp, target, sizeof(target) - 1);
			if (n <= 0) continue;
			target[n] = 0;
			if (!strcmp(target, port)) found = 1;
		}
		closedir(fdd);
		if (!found) continue;

		char comm[64] = "?";
		snprintf(p, sizeof(p), "/proc/%d/comm", pid);
		gw_read_sys(p, comm, sizeof(comm));
		if (strcasestr(comm, "zaparoo")) zaparoo = 1;
		int l = strlen(list);
		snprintf(list + l, sizeof(list) - l, "%s%s (pid %d)", l ? ", " : "", comm, pid);
	}
	closedir(d);

	if (list[0])
	{
		pf_log("WARNING: the Greaseweazle port %s is also open by: %s. A program probing serial ports sends data to the Greaseweazle and breaks the communication.%s",
			path, list, zaparoo ? " For Zaparoo, add to zaparoo/config.toml: [readers.drivers.pn532] auto_detect = false, and [readers.drivers.tty2oled] auto_detect = false" : "");
	}
}

int gw_read_sys(const char *path, char *out, int sz)
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
	set_motor_delay();

	// MiSTer.ini PFLOPPY_GW_DRIVE: 0 = automatic, 1 = drive A (after the cable twist), 2 = drive B
	unit = cfg.pfloppy_gw_drive == 2 ? 1 : 0;
	unit_ok = 0;
	unit_logged = 0;
	selected = motor = 0;

	snprintf(path, sizeof(path), "%s", p);
	check_port_users(1);
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

	// what went wrong, for the log when sectors are missing
	int reads = 0, flux_err = 0, last_err = 0, decoded = 0, crc = 0, other_cyl = 0, seen_cyl = -1;
	int recal = 0, extra = 0;

	for (int attempt = 0; attempt < ((crc && spt) ? READ_ATTEMPTS_CRC : READ_ATTEMPTS) + extra; attempt++)
	{
		reads++;
		if (gcr())
		{
			// Mac GCR tracks are not written from the index pulse: read from where
			// the head is, a revolution plus the longest sector (an eighth of a
			// track on cylinders 64-79), instead of waiting for the index
			// (1.5 revolutions on average)
			std::vector<uint32_t> idx;
			r = read_flux_cmd((uint32_t)(freq * MAC_READ_S), 0, flux, &idx, NULL);
			if (r < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
			if (!r && idx.empty()) return PFD_CHANGED; // no index pulse in a revolution: not spinning
		}
		else r = read_flux(1, flux);
		if (r < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
		if (r == ACK_NO_INDEX) return PFD_CHANGED; // no rotation: disk removed
		if (r)
		{
			// flux overflow etc (the stream was not read fast enough): read again,
			// a few more times than for bad sectors
			flux_err++;
			last_err = r;
			if (extra < READ_ATTEMPTS) extra++;
			continue;
		}

		decode(flux, cyl, out);
		for (auto &s : out)
		{
			decoded++;
			if (!s.ok) { crc++; continue; }
			if (s.n != 2 || s.sec < 1 || s.sec > 63) continue;
			if (s.cyl != cyl) { other_cyl++; seen_cyl = s.cyl; continue; }
			if (heads > 1 && s.head != hd) continue;
			if (got[s.sec]) continue;
			got[s.sec] = 1;
			ngot++;
			secs.push_back(s);
		}
		if (spt && ngot >= spt) break;
		if (!spt && ngot) break;

		// only sectors of another cylinder: the head lost its position (missed
		// steps). Recalibrate on track 0 and seek again, once
		if (!ngot && other_cyl && !recal && !double_step)
		{
			recal = 1;
			pf_log("cylinder %d: the head is on cylinder %d (missed steps: check the drive's power supply), recalibrating", cyl, seen_cyl);
			if (seek(0) < 0 || seek(cyl) < 0 || head(hd) < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
			attempt = -1;
		}
	}

	// sectors missing: why (not for an empty track, like the second side of a single sided disk)
	if (spt && ngot < spt && (ngot || decoded || flux_err))
	{
		char why[128] = "";
		if (flux_err) snprintf(why + strlen(why), sizeof(why) - strlen(why), ", %d flux errors (code %d)", flux_err, last_err);
		if (crc) snprintf(why + strlen(why), sizeof(why) - strlen(why), ", %d CRC errors", crc);
		if (other_cyl) snprintf(why + strlen(why), sizeof(why) - strlen(why), ", %d sectors of cylinder %d", other_cyl, seen_cyl);
		pf_log("cylinder %d head %d: %d of %d sectors in %d reads, %d sectors decoded%s", cyl, hd, ngot, spt, reads, decoded, why);
	}
	return PFD_OK;
}

// sectors of a track, as PC sectors: an AmigaDOS sector n (0..10) of track t
// becomes sector n + 1 of cylinder t / 2, head t % 2
void GwDrive::decode(const std::vector<uint32_t> &flux, int cyl, std::vector<mfm_sector> &out)
{
	if (gcr())
	{
		// Mac sector n (0..) of a side: sector n + 1, data without the 12 tag bytes
		std::vector<mac_sector> ms;
		mac_gcr_decode(flux, freq, cyl, ms);
		out.clear();
		for (auto &m : ms)
		{
			mfm_sector s;
			s.cyl = m.cyl;
			s.head = m.side;
			s.sec = m.sec + 1;
			s.n = 2;
			s.ok = m.ok;
			memcpy(s.data, m.data + 12, 512);
			out.push_back(s);
		}
		return;
	}
	if (!amiga)
	{
		mfm_ibm_decode(flux, freq, cell_ns, out);
		return;
	}
	std::vector<amiga_sector> as;
	mfm_amiga_decode(flux, freq, as);
	out.clear();
	for (auto &a : as)
	{
		mfm_sector s;
		s.cyl = a.track >> 1;
		s.head = a.track & 1;
		s.sec = a.sec + 1;
		s.n = 2;
		s.ok = a.ok;
		memcpy(s.data, a.data, 512);
		out.push_back(s);
	}
}

int GwDrive::detect_amiga(pfd_geometry *g, char *why, int whysz)
{
	std::vector<mfm_sector> secs;
	double_step = 0;
	cell_ns = 2000;
	int r = read_good(0, 0, 2, AMIGA_SPT, secs);
	if (r == PFD_LOST) { snprintf(why, whysz, "Greaseweazle not responding"); return PFD_PROBE_LOST; }
	if (r == PFD_NOT_READY || r == PFD_CHANGED) return PFD_PROBE_BUSY;
	if (r != PFD_OK || secs.size() < AMIGA_SPT / 2)
	{
		snprintf(why, whysz, "disk present but no AmigaDOS sectors on track 0 (%zu found): use _GREASEWEAZLE_.adf for other disks", secs.size());
		return PFD_PROBE_UNREADABLE;
	}

	int wp = get_pin(28);
	g->spt = AMIGA_SPT;
	g->heads = 2;
	g->cyl = 80;
	g->total = AMIGA_SPT * 2 * 80;
	g->wp = (wp == 0);
	pf_log("format: AmigaDOS, %d of %d sectors found on track 0%s", (int)secs.size(), AMIGA_SPT, wp == 0 ? ", write-protect tab set" : "");
	return PFD_PROBE_NEW;
}

// Mac disks: 400K single sided, 800K double sided. The geometry given to the
// engine has 12 sectors on every track (the most, cylinders 0-15): the real
// count of each zone is used when reading, the slots after it stay unused
int GwDrive::detect_mac(pfd_geometry *g, char *why, int whysz)
{
	std::vector<mfm_sector> secs;
	double_step = 0;
	int r = read_good(0, 0, 1, mac_spt(0), secs);
	if (r == PFD_LOST) { snprintf(why, whysz, "Greaseweazle not responding"); return PFD_PROBE_LOST; }
	if (r == PFD_NOT_READY || r == PFD_CHANGED) return PFD_PROBE_BUSY;
	if (r != PFD_OK || (int)secs.size() < mac_spt(0) / 2)
	{
		snprintf(why, whysz, "disk present but no Macintosh GCR sectors on track 0 (%zu found)", secs.size());
		return PFD_PROBE_UNREADABLE;
	}

	r = read_good(0, 1, 2, mac_spt(0), secs);
	if (r == PFD_LOST) { snprintf(why, whysz, "Greaseweazle not responding"); return PFD_PROBE_LOST; }
	if (r == PFD_NOT_READY || r == PFD_CHANGED) return PFD_PROBE_BUSY;
	int heads = secs.size() ? 2 : 1;

	int wp = get_pin(28);
	g->spt = MAC_MAX_SPT;
	g->heads = heads;
	g->cyl = 80;
	g->total = MAC_MAX_SPT * heads * 80;
	g->wp = (wp == 0);
	pf_log("format: Macintosh %s GCR%s", heads == 2 ? "800K" : "400K", wp == 0 ? ", write-protect tab set" : "");
	return PFD_PROBE_NEW;
}

static int max_sector(const std::vector<mfm_sector> &secs)
{
	int m = 0;
	for (auto &s : secs) if (s.sec > m) m = s.sec;
	return m;
}

int GwDrive::detect(pfd_geometry *g, char *why, int whysz)
{
	if (amiga) return detect_amiga(g, why, whysz);
	if (!mac) return detect_pc(g, why, whysz);

	mac_mfm = 0;
	int r = detect_mac(g, why, whysz);
	if (r != PFD_PROBE_UNREADABLE || !mac_sd) return r;

	// SuperDrive: a 720K / 1.44 MB MFM disk, the PC format
	mac_mfm = 1;
	char why2[160];
	r = detect_pc(g, why2, sizeof(why2));
	if (r != PFD_PROBE_NEW) mac_mfm = 0;
	if (r == PFD_PROBE_UNREADABLE) snprintf(why, whysz, "disk present but neither Macintosh GCR nor MFM sectors on track 0");
	else if (r != PFD_PROBE_NEW) snprintf(why, whysz, "%s", why2);
	return r;
}

int GwDrive::detect_pc(pfd_geometry *g, char *why, int whysz)
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
	g->wp = (wp == 0);
	pf_log("format: %s, %d sectors/track, %d head(s), %d cylinders%s%s", cell == 1000 ? "HD 500 kbps" : "DD 250 kbps",
		spt, heads, cyl, double_step ? " (double step)" : "", wp == 0 ? ", write-protect tab set" : "");
	return PFD_PROBE_NEW;
}

// finds the drive: only a connected drive asserts /TRK0, so a recalibration
// (seek to cylinder 0) succeeds only on the right unit. Works without a disk.
// returns 1 found, 0 not found, -1 communication error
// the unit the drive was found on, kept in /tmp for the next core (each core
// starts Main again): -1 not known
#define LAST_UNIT_PATH "/tmp/pfloppy_gw_unit"
static int last_unit_get()
{
	int u = -1;
	FILE *f = fopen(LAST_UNIT_PATH, "r");
	if (f)
	{
		if (fscanf(f, "%d", &u) != 1) u = -1;
		fclose(f);
	}
	return (u == 0 || u == 1) ? u : -1;
}

static void last_unit_set(int u)
{
	if (last_unit_get() == u) return;
	FILE *f = fopen(LAST_UNIT_PATH, "w");
	if (f)
	{
		fprintf(f, "%d\n", u);
		fclose(f);
	}
}

int GwDrive::find_unit()
{
	int units[2], n = 0;
	if (cfg.pfloppy_gw_drive == 1) units[n++] = 0;
	else if (cfg.pfloppy_gw_drive == 2) units[n++] = 1;
	else
	{
		// the unit found last time first: looking for a drive on the other one
		// costs about 2.5 s (the recalibration times out)
		int last = last_unit_get();
		units[n++] = last == 1 ? 1 : 0;
		units[n++] = last == 1 ? 0 : 1;
	}

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
			if (!cfg.pfloppy_gw_drive) last_unit_set(unit);
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
// returns 1 disk in (*was_changed = 1 if the latch was set: removed/changed since
// the last check), 0 no disk, -1 communication error, -2 no drive connected
int GwDrive::check_disk(int *was_changed)
{
	*was_changed = 0;
	if (!unit_ok)
	{
		int r = find_unit();
		if (r < 0) return -1;
		if (!r) return -2;
	}

	if (select(1)) return -1;
	int chg = get_pin(34);
	if (chg < 0) return -1;
	if (chg == 0)
	{
		*was_changed = 1;
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
		if (r < 0) return -1;
		chg = get_pin(34);
		if (chg < 0) return -1;
	}
	return chg == 1;
}

int GwDrive::probe(int present, int changed, pfd_geometry *g, char *why, int whysz)
{
	int was_selected = selected;
	int latch = 0;
	int in = check_disk(&latch);
	if (in == -1)
	{
		snprintf(why, whysz, "Greaseweazle not responding");
		return comm_error() ? PFD_PROBE_BUSY : PFD_PROBE_LOST;
	}
	if (in == -2)
	{
		snprintf(why, whysz, "no floppy drive connected to the Greaseweazle");
		return PFD_PROBE_NODISK;
	}

	int res;
	if (in && present && !changed && !latch) res = PFD_PROBE_SAME;
	else if (!in)
	{
		snprintf(why, whysz, "no disk");
		res = PFD_PROBE_NODISK;
	}
	else res = detect(g, why, whysz);

	if (!motor && !was_selected) select(0);
	return res;
}

int GwDrive::flux_probe(int present, char *why, int whysz)
{
	int was_selected = selected;
	int latch = 0;
	int in = check_disk(&latch);
	if (in == -1)
	{
		snprintf(why, whysz, "Greaseweazle not responding");
		return comm_error() ? PFD_PROBE_BUSY : PFD_PROBE_LOST;
	}
	if (in == -2)
	{
		snprintf(why, whysz, "no floppy drive connected to the Greaseweazle");
		return PFD_PROBE_NODISK;
	}

	int res;
	if (!in)
	{
		snprintf(why, whysz, "no disk");
		res = PFD_PROBE_NODISK;
	}
	else res = (present && !latch) ? PFD_PROBE_SAME : PFD_PROBE_NEW;

	if (!motor && !was_selected) select(0);
	return res;
}

int GwDrive::flux_seek(int cyl)
{
	if (select(1)) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	int r = seek(cyl);
	if (r < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	return r ? PFD_MEDIUM : PFD_OK;
}

// time of each transition from the start of a capture
static void flux_times(const std::vector<uint32_t> &flux, std::vector<double> &t)
{
	t.resize(flux.size());
	double acc = 0;
	for (size_t i = 0; i < flux.size(); i++) { acc += flux[i]; t[i] = acc; }
}

// time of an index pulse: pos transitions before it in the stream, off ticks after the last of them
static double index_time(const std::vector<double> &t, uint32_t pos, uint32_t off)
{
	return (pos && pos <= t.size() ? t[pos - 1] : 0) + off;
}

// one revolution from a capture a little longer than a revolution, with one
// index pulse in it: the part after the pulse is the start of the revolution,
// the part before it the end. The overlap (the same piece of track seen twice)
// tells where to join them: the copies are matched on their flux intervals and
// joined on a transition, so the join is as clean as the rest of the track.
// Only a piece of flux that matches in one place is used (not a gap or a
// repeated pattern, where a join a few bits off would lose or add data).
// returns 0 if no clean join was found
int GwDrive::splice_rev(const std::vector<uint32_t> &flux, const std::vector<double> &t, double a, pfd_rev &rv)
{
	const size_t W = 64;      // intervals compared
	const int TRIES = 24;     // places tried in the overlap
	size_t n = flux.size();
	if (n < 2 * W || rev_ticks <= 0) return 0;

	// first transition after the index pulse (the firmware reports the pulse
	// in the stream a little late: a few transitions before it may follow it)
	size_t first = std::upper_bound(t.begin(), t.end(), a) - t.begin();
	if (!first || first >= n) return 0;

	double R = rev_ticks;
	double T = t[n - 1];
	double ov = (T - R < a ? T - R : a) - freq * 0.0002; // overlap before the index pulse
	if (ov < freq * 0.001) return 0;

	for (int tr = 0; tr < TRIES; tr++)
	{
		// a transition in the overlap
		double tb = ov * (tr + 0.5) / TRIES;
		size_t j = std::lower_bound(t.begin(), t.end(), tb) - t.begin();
		if (j + W >= first) break;

		// the same transition one revolution later (within 0.4% of the measured
		// revolution time: the speed of a drive is stable): the best match, and
		// the best one at another position
		double lo = t[j] + R * 0.996, hi = t[j] + R * 1.004;
		size_t k = std::lower_bound(t.begin(), t.end(), lo) - t.begin();
		size_t best = 0;
		double s1 = 1e30, s2 = 1e30;
		for (; k + W < n && t[k] <= hi; k++)
		{
			double sc = 0;
			for (size_t m = 1; m <= W && sc < s2; m++)
			{
				double d = (double)flux[k + m] - (double)flux[j + m];
				sc += d < 0 ? -d : d;
			}
			if (sc < s1)
			{
				// the previous best becomes the second if it is somewhere else
				if (best && fabs(t[k] - t[best]) > freq * 10e-6) s2 = s1;
				s1 = sc;
				best = k;
			}
			else if (sc < s2 && fabs(t[k] - t[best]) > freq * 10e-6) s2 = sc;
		}

		// the same flux (mean difference below 0.3 us) and nowhere else
		if (!best || s1 / W > freq * 0.3e-6 || s2 < s1 * 4 + W * freq * 0.3e-6 || best <= first) continue;

		rv.flux.clear();
		rv.flux.reserve(n);
		rv.flux.push_back((uint32_t)lround(t[first] - a > 1 ? t[first] - a : 1));
		for (size_t i = first + 1; i <= best; i++) rv.flux.push_back(flux[i]);
		for (size_t i = j + 1; i < first; i++) rv.flux.push_back(flux[i]);
		rv.len = (uint32_t)lround(t[best] - t[j]);
		// the revolution time is not updated from here: a measure from index to
		// index (the reads where this fails) is the reference
		return 1;
	}
	return 0;
}

int GwDrive::read_revs(int cyl, int hd, int revs, std::vector<pfd_rev> &out)
{
	out.clear();
	if (select(1)) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	int chg = get_pin(34);
	if (chg < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	if (chg == 0) return PFD_CHANGED;

	if (motor_on()) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	int r = seek(cyl);
	if (r < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	if (r) return PFD_MEDIUM;
	if (head(hd) < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;

	std::vector<uint32_t> flux, pos, off;
	int have = 0;

	// one revolution: read a little more than a revolution from where the head
	// is now, without waiting for the index pulse (about 100 ms saved on average)
	if (revs == 1 && rev_ticks > 0)
	{
		r = read_flux_cmd((uint32_t)(rev_ticks * 1.06 + freq * 0.002), 0, flux, &pos, &off);
		if (r < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
		if (!r)
		{
			pfd_rev rv;
			std::vector<double> t;
			flux_times(flux, t);
			if (pos.size() == 1 && splice_rev(flux, t, index_time(t, pos[0], off[0]), rv))
			{
				out.push_back(std::move(rv));
				return PFD_OK;
			}
			// two pulses: a whole revolution is in the capture. Otherwise (no clean
			// join: unformatted or noisy track) a read from index to index
			have = pos.size() >= 2;
		}
	}

	if (!have)
	{
		r = read_flux(revs, flux, &pos, &off);
		if (r < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
		if (r == ACK_NO_INDEX) return PFD_CHANGED; // no rotation: disk removed
		if (r) return PFD_NOT_READY;               // overflow etc: read again
	}

	// revolutions: the transitions between two index pulses, by time
	// (the firmware reports a pulse in the stream a little late)
	std::vector<double> t;
	flux_times(flux, t);
	for (size_t k = 0; k + 1 < pos.size(); k++)
	{
		double ia = index_time(t, pos[k], off[k]), ib = index_time(t, pos[k + 1], off[k + 1]);
		size_t a = std::upper_bound(t.begin(), t.end(), ia) - t.begin();
		size_t b = std::upper_bound(t.begin(), t.end(), ib) - t.begin();
		if (a >= b) continue;
		pfd_rev rv;
		rv.flux.assign(flux.begin() + a, flux.begin() + b);
		rv.flux[0] = (uint32_t)lround(t[a] - ia > 1 ? t[a] - ia : 1);
		rv.len = (uint32_t)lround(ib - ia);
		if (rv.len > freq * 0.15 && rv.len < freq * 0.25) rev_ticks = rv.len;
		out.push_back(std::move(rv));
	}
	return out.empty() ? PFD_NOT_READY : PFD_OK;
}

int GwDrive::read_track(const pfd_geometry *g, int track, uint8_t *buf, uint8_t *st)
{
	int cyl = track / g->heads;
	int hd = track % g->heads;

	if (select(1)) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	int chg = get_pin(34);
	if (chg < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	if (chg == 0) return PFD_CHANGED;

	// Mac: the sectors of this zone; the other slots of the track are unused
	int spt = gcr() ? mac_spt(cyl) : g->spt;
	std::vector<mfm_sector> secs;
	int r = read_good(cyl, hd, g->heads, spt, secs);
	if (r != PFD_OK) return r;

	memset(st, 1, g->spt);
	memset(st, 2, spt);
	memset(buf, 0, g->spt * 512);
	for (auto &s : secs)
	{
		if (s.sec > spt) continue;
		memcpy(buf + (s.sec - 1) * 512, s.data, 512);
		st[s.sec - 1] = 1;
	}
	return PFD_OK;
}

// writes a flux stream on the current track, from index to index.
// returns ACK_* (ACK_WRPROT write-protected), -1 on a communication error
int GwDrive::write_flux(const std::vector<uint32_t> &flux)
{
	uint8_t c[4] = { GW_WRITE_FLUX, 4, 1, 1 }; // cue at index, terminate at index
	int r = cmd(c);
	if (r) return r;

	// encoding as the Greaseweazle host tools: 1-249 one byte, 250-1524 two bytes,
	// longer: "space" op + 249; a final dummy flux makes sure the last real one is written
	static std::vector<uint8_t> o;
	o.clear();
	o.reserve(flux.size() + 16);
	auto put28 = [](std::vector<uint8_t> &v, uint32_t x)
	{
		v.push_back(1 | ((x << 1) & 255)); v.push_back(1 | ((x >> 6) & 255));
		v.push_back(1 | ((x >> 13) & 255)); v.push_back(1 | ((x >> 20) & 255));
	};
	auto put = [&](uint32_t v)
	{
		if (!v) return;
		if (v < 250) o.push_back(v);
		else if (v <= 1524) { o.push_back(250 + (v - 250) / 255); o.push_back(1 + (v - 250) % 255); }
		else { o.push_back(255); o.push_back(2); put28(o, v - 249); o.push_back(249); }
	};
	for (uint32_t v : flux) put(v);
	put((uint32_t)(100e-6 * freq));
	o.push_back(0);

	if (wr(o.data(), o.size()))
	{
		snprintf(err, sizeof(err), "write flux: %s", strerror(errno));
		return -1;
	}

	uint8_t sync;
	if (rd(&sync, 1, 3000))
	{
		char e[160];
		snprintf(e, sizeof(e), "write flux: no end of write, %s", err);
		snprintf(err, sizeof(err), "%s", e);
		return -1;
	}

	uint8_t st[2] = { GW_GET_FLUX_STATUS, 2 };
	return cmd(st);
}

// rewrites a whole track: the sectors not changed come from the cache or are read
// from the disk first; the track is read back to check the written data
int GwDrive::write_track(const pfd_geometry *g, int track, const uint8_t *buf, const uint8_t *dirty, const uint8_t *valid)
{
	int cyl = track / g->heads;
	int hd = track % g->heads;
	int spt = g->spt;

	if (mac) return PFD_WP; // writing Mac disks: not yet

	if (select(1)) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	int chg = get_pin(34);
	if (chg < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	if (chg == 0) return PFD_CHANGED;
	int wp = get_pin(28);
	if (wp < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
	if (wp == 0) return PFD_WP;

	static uint8_t data[64 * 512];
	uint8_t have[64] = {};
	int missing = 0;
	for (int i = 0; i < spt; i++)
	{
		if (dirty[i] || valid[i])
		{
			memcpy(data + i * 512, buf + i * 512, 512);
			have[i] = 1;
		}
		else missing++;
	}

	std::vector<mfm_sector> secs;
	if (missing)
	{
		int r = read_good(cyl, hd, g->heads, spt, secs);
		if (r != PFD_OK) return r;
		for (auto &s : secs)
		{
			int i = s.sec - 1;
			if (i < 0 || i >= spt || have[i]) continue;
			memcpy(data + i * 512, s.data, 512);
			have[i] = 1;
			missing--;
		}
		if (missing)
		{
			pf_log("track %d: %d sector(s) unreadable, the track is not rewritten (it would lose them)", track, missing);
			return PFD_MEDIUM;
		}
	}

	double rev_ns = rev_ticks > 0 ? rev_ticks * 1e9 / freq : 200e6;
	std::vector<uint32_t> flux;
	if (amiga) mfm_amiga_encode_track(cyl * 2 + hd, data, freq, rev_ns, flux);
	else mfm_ibm_encode_track(cyl, hd, spt, data, cell_ns, freq, rev_ns, flux);

	for (int attempt = 0; attempt < 2; attempt++)
	{
		if (motor_on()) return comm_error() ? PFD_NOT_READY : PFD_LOST;
		int r = seek(double_step ? cyl * 2 : cyl);
		if (r < 0 || head(hd) < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;

		r = write_flux(flux);
		if (r < 0) return comm_error() ? PFD_NOT_READY : PFD_LOST;
		if (r == ACK_WRPROT) return PFD_WP;
		if (r == ACK_NO_INDEX) return PFD_CHANGED;
		if (r) { pf_log("track %d: write status %d, retrying", track, r); continue; }

		// verify
		r = read_good(cyl, hd, g->heads, spt, secs);
		if (r != PFD_OK) return r;
		int ok = 0;
		for (auto &s : secs)
		{
			int i = s.sec - 1;
			if (i >= 0 && i < spt && !memcmp(s.data, data + i * 512, 512)) ok++;
		}
		if (ok == spt) return PFD_OK;
		pf_log("track %d: verify found %d of %d sectors right%s", track, ok, spt, attempt ? "" : ", writing again");
	}
	return PFD_MEDIUM;
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
