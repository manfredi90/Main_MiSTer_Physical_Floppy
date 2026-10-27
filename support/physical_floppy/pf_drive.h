#ifndef PF_DRIVE_H
#define PF_DRIVE_H

/*
 * Physical floppy drive backends: the engine (physical_floppy.cpp) keeps the
 * cache, the thread and the dialogue with the core; a backend talks to one
 * kind of drive (USB UFI drive, Greaseweazle).
 */

#include <stdint.h>
#include <vector>

// results of read/write/keep_spinning
enum
{
	PFD_OK = 0,
	PFD_CHANGED,    // the disk was changed or removed: probe again
	PFD_NOT_READY,  // spinning up / no disk: probe again shortly
	PFD_LOST,       // the drive is gone (unplugged, not responding)
	PFD_WP,         // write-protected
	PFD_MEDIUM      // read/write error on the disk itself
};

// results of probe
enum
{
	PFD_PROBE_LOST = 0,     // drive gone
	PFD_PROBE_NODISK,       // no disk
	PFD_PROBE_BUSY,         // can't tell yet (spinning up), ask again later
	PFD_PROBE_UNREADABLE,   // a disk is there but it can't be used (why says why)
	PFD_PROBE_SAME,         // the disk in use is still there
	PFD_PROBE_NEW           // a (new) disk is ready, geometry in g
};

struct pfd_geometry
{
	int total;   // sectors
	int spt, heads, cyl;
	int wp;
};

class PfDrive
{
public:
	virtual ~PfDrive() {}

	// find and open the drive. desc: short description for the log
	virtual int open(char *desc, int descsz) = 0;
	virtual void close() = 0;

	// check the disk. present: a disk is in use now; changed: a previous command reported a change
	virtual int probe(int present, int changed, pfd_geometry *g, char *why, int whysz) = 0;

	// read one track (track = cyl * heads + head): 512-byte sectors in buf, st[i] = 1 ok, 2 bad
	virtual int read_track(const pfd_geometry *g, int track, uint8_t *buf, uint8_t *st) = 0;

	// write the sectors of one track marked in dirty[] (buf holds the whole track,
	// 512 bytes per sector; valid[] marks the other sectors whose cached copy is in buf too)
	virtual int write_track(const pfd_geometry *g, int track, const uint8_t *buf, const uint8_t *dirty, const uint8_t *valid) = 0;

	// the core is using the floppy: keep the motor spinning
	virtual int keep_spinning(const pfd_geometry *g, int track) = 0;

	// called often by the idle thread: idle_ms = time since the last access from the core
	virtual void idle(double idle_ms) { (void)idle_ms; }

	// how often the disk presence is checked when idle
	virtual int poll_ms() { return 1000; }
};

PfDrive *pfd_usb_create();
PfDrive *pfd_gw_create();
PfDrive *pfd_gw_create_amiga();  // AmigaDOS disks
PfDrive *pfd_gw_create_mac(int superdrive); // Macintosh 400K/800K disks, + 720K/1.44 MB MFM with superdrive

// one revolution of a track, from index pulse to index pulse: flux intervals in
// sample ticks (the first one counted from the index pulse), len = revolution time
struct pfd_rev
{
	std::vector<uint32_t> flux;
	uint32_t len;
};

// flux level access to a drive (Greaseweazle), for the Amiga: the computer
// decodes the disk itself, so no format knowledge is needed here
class PfFluxDrive
{
public:
	virtual ~PfFluxDrive() {}
	virtual int open(char *desc, int descsz) = 0;
	virtual void close() = 0;
	virtual double sample_hz() = 0;

	// present: a disk is in use now. PFD_PROBE_NEW = a (new) disk is in,
	// PFD_PROBE_SAME = still the same disk, PFD_PROBE_NODISK, PFD_PROBE_BUSY, PFD_PROBE_LOST
	virtual int flux_probe(int present, char *why, int whysz) = 0;

	// reads revs revolutions of a track. PFD_OK, PFD_CHANGED (disk removed),
	// PFD_NOT_READY (try again), PFD_MEDIUM (cylinder out of reach), PFD_LOST
	virtual int read_revs(int cyl, int head, int revs, std::vector<pfd_rev> &out) = 0;

	// moves the head without reading (follows the computer)
	virtual int flux_seek(int cyl) = 0;
	virtual void flux_motor_off() = 0;
	virtual int flux_motor_is_on() = 0;
};

PfFluxDrive *pfd_gw_flux_create();

// shared helpers (physical_floppy.cpp)
void pf_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
double pf_now_ms();
void pf_calc_geometry(int total, int *spt, int *heads, int *cyl);
void pf_log_reset();  // new log file (when a drive is attached)
void pf_log_flush();  // waits until the log is written

#define PF_MOTOR_KEEP_MS 3000  // motor-off delay after the last access from the core

#endif
