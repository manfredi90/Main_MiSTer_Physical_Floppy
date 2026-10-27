#ifndef PHYSICAL_FLOPPY_H
#define PHYSICAL_FLOPPY_H

/*
 * Physical floppy support: a real floppy drive used live by the x86 family
 * cores (ao486, PCXT, PCAT286) instead of a floppy image. Two kinds of drive
 * (pf_drive.h): a USB floppy drive (UFI, SCSI commands through SG_IO) or a
 * PC drive on a Greaseweazle (flux, decoded in mfm_ibm.cpp).
 *
 * No copy of the whole disk: a background thread keeps a per-track cache
 * with read-ahead and writes changed sectors back to the disk.
 * Requests from the core never block the main loop: a sector that is
 * not in the cache yet is reported as pending and served on a later poll.
 *
 * The drive is selected by mounting the placeholder file
 * PFLOPPY_PLACEHOLDER (an empty .img created in the core folder).
 */

#include <stdint.h>

#define PFLOPPY_PLACEHOLDER "_USB_FLOPPY_.img"

typedef struct
{
	int drive;          // 1 = USB floppy drive found
	int present;        // 1 = readable disk in the drive
	int total_sectors;  // disk size in 512-byte sectors
	int write_protected;
	int spt, heads, cylinders;
} pfloppy_media_t;

// 1 if the path (any folder) points to the placeholder file
int pfloppy_is_placeholder(const char *path);

// create the empty placeholder file in the given folder (relative to /media/fat) if missing
void pfloppy_create_placeholder(const char *dir);

// same, any file name
void pfloppy_create_placeholder_named(const char *dir, const char *name);

// disk format: PC (x86 cores, default) or AmigaDOS (Minimig, Greaseweazle only).
// Set before pfloppy_attach.
#define PFLOPPY_FORMAT_PC    0
#define PFLOPPY_FORMAT_AMIGA 1
#define PFLOPPY_FORMAT_MAC   2   // Mac 400K/800K GCR
#define PFLOPPY_FORMAT_MAC_SD 3   // Mac SuperDrive: GCR, and 720K/1.44 MB MFM
void pfloppy_set_format(int format);

// use the drive as floppy unit 0 (A:) or 1 (B:). Probes the disk before returning.
void pfloppy_attach(int unit);

// stop using the drive (pending writes are flushed first)
void pfloppy_detach(void);

// unit the drive is attached to, -1 if none
int pfloppy_unit(void);

void pfloppy_get_media(pfloppy_media_t *m);

// returns 1 once after every disk insertion, removal or change
int pfloppy_media_event(void);

// 1 = sector copied to buf, 0 = not ready yet (ask again later), -1 = error/no disk
int pfloppy_read(uint32_t lba, uint8_t *buf);

// 1 = accepted (written to the disk in the background), -1 = error
int pfloppy_write(uint32_t lba, const uint8_t *buf, uint32_t cnt);

/*
 * Amiga (Minimig): the Greaseweazle drive seen by the core as a flux drive
 * (pf_amiga.cpp). The core decodes the disk itself, so any Amiga disk works,
 * copy-protected ones included. Read-only: the core's flux drive cannot write.
 * Selected by inserting PFLOPPY_AMIGA_PLACEHOLDER in a DFx drive.
 */
#define PFLOPPY_AMIGA_PLACEHOLDER "_GREASEWEAZLE_.adf"

class FluxFile;

// AmigaDOS mode: the drive decoded into sectors and given to the core as an
// ADF (the core's normal floppy emulation, writable). Standard AmigaDOS disks only.
#define PFLOPPY_AMIGA_DOS_PLACEHOLDER "_GREASEWEAZLE_DOS_.adf"
int pfloppy_is_amiga_dos_placeholder(const char *path);

// minimig_fdd.cpp InsertFloppy: use the drive in DFx (index in df[])
void pfloppy_amiga_dos_insert(int dfx);
// the DFx drive gets another disk image: stop using the drive
void pfloppy_amiga_dos_release(int dfx);

// 1 if the path (any folder) points to the Amiga placeholder file
int pfloppy_is_amiga_placeholder(const char *path);

// a flux reader for minimig_fdd.cpp (InsertFloppy)
FluxFile *pfloppy_amiga_create(void);

// called from the Minimig poll loop: reports disk insertions and removals to the core
void pfloppy_amiga_poll(void);

/*
 * Macintosh cores (MacPlus, MacLC, MacSE30/MacIIcx): the drive in a floppy
 * slot of the core, which loads the whole disk at insertion: the disk is read
 * from the Greaseweazle then (about a minute for 800K). 400K/800K GCR disks,
 * and 720K/1.44 MB MFM disks on the cores with a SuperDrive. Read-only for now.
 */
#define PFLOPPY_MAC_PLACEHOLDER "_GREASEWEAZLE_.dsk"

int pfloppy_is_mac_placeholder(const char *path);
int pfloppy_mac_core(void);                     // 0 not a Mac core, 1 800K drive, 2 SuperDrive

// user_io_file_mount: the placeholder in slot index. Returns the image size (0 = no disk)
uint64_t pfloppy_mac_mount(int index, const char *path);
void pfloppy_mac_unmount(int index);

// user_io SD requests: 0 = not ours, 1 = served, -1 = not ready (ask again later)
int pfloppy_mac_sd_service(int disk, int op, uint32_t lba, int sz, int ack);

// user_io_poll: a disk inserted, removed or changed is mounted again
void pfloppy_mac_poll(void);

#endif
