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

#endif
