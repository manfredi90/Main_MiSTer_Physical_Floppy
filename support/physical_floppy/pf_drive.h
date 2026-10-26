#ifndef PF_DRIVE_H
#define PF_DRIVE_H

/*
 * Physical floppy drive backends: the engine (physical_floppy.cpp) keeps the
 * cache, the thread and the dialogue with the core; a backend talks to one
 * kind of drive (USB UFI drive, Greaseweazle).
 */

#include <stdint.h>

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

	// write n sectors from lba (all within one track)
	virtual int write_sectors(const pfd_geometry *g, int lba, int n, const uint8_t *buf) = 0;

	// the core is using the floppy: keep the motor spinning
	virtual int keep_spinning(const pfd_geometry *g, int track) = 0;

	// called often by the idle thread: idle_ms = time since the last access from the core
	virtual void idle(double idle_ms) { (void)idle_ms; }

	// how often the disk presence is checked when idle
	virtual int poll_ms() { return 1000; }
};

PfDrive *pfd_usb_create();
PfDrive *pfd_gw_create();

// shared helpers (physical_floppy.cpp)
void pf_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
double pf_now_ms();
void pf_calc_geometry(int total, int *spt, int *heads, int *cyl);

#define PF_MOTOR_KEEP_MS 3000  // motor-off delay after the last access from the core

#endif
