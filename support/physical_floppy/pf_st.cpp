/*
 * Physical floppy for the Atari ST core: the Greaseweazle drive in floppy
 * A: or B: (slot 0 / 1 of the core).
 *
 * The core reads .st images, the sectors of the disk in order (cylinder by
 * cylinder, side 0 then side 1), and asks for them one at a time while it
 * emulates the WD1772 (rtl/fdc1772): the order is the one of the sector
 * engine (physical_floppy.cpp), a sector not read yet is answered later (the
 * core waits). The core takes the geometry from the image size: 9 to 12
 * sectors per track, 79 to 85 cylinders, one or two sides, HD from 1 MB.
 *
 * A disk inserted, removed or changed is mounted again (tos_insert_disk: the
 * ST notices the change through the write-protect line). Writes go to the
 * sector engine, which writes them to the disk in the background (as for the
 * PC cores); a disk with its write-protect tab set is shown write-protected.
 * Formatting from the ST does not work (the core ends the track writes of a
 * sector image at once, without data): format the floppies on a PC.
 */

#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "physical_floppy.h"
#include "pf_drive.h"
#include "../../user_io.h"
#include "../../spi.h"
#include "../st/st_tos.h"

static int st_slot = -1;            // slot with the placeholder, -1 none
static char st_path[1024];
static int st_sectors = 0;          // image sectors, 0 = no disk
static int st_placeholder_done = 0;
static int st_write_logged = 0;
static int st_wp = 1;               // disk write-protected (or none)

int pfloppy_is_st_placeholder(const char *path)
{
	if (!path || !path[0]) return 0;
	const char *p = strrchr(path, '/');
	p = p ? p + 1 : path;
	return !strcasecmp(p, PFLOPPY_ST_PLACEHOLDER);
}

int pfloppy_st_slot(void)
{
	return st_slot;
}

uint64_t pfloppy_st_mount(int index, const char *path)
{
	if (pfloppy_st_flux_slot() >= 0) pfloppy_st_flux_unmount(pfloppy_st_flux_slot());  // the flux mode had the drive
	if (st_slot >= 0 && st_slot != index) pfloppy_st_unmount(st_slot);
	snprintf(st_path, sizeof(st_path), "%s", path);

	if (st_slot != index)
	{
		pfloppy_set_format(PFLOPPY_FORMAT_ST);
		// not waiting for the drive: the slot is empty until the disk is found
		// (pfloppy_st_poll mounts it then)
		pfloppy_attach_async(index);
		st_slot = index;
		st_sectors = 0;
		st_write_logged = 0;
		st_wp = 1;
		pf_log("Atari ST floppy %c: Atari ST and PC disks (9 to 11 sectors per track)", 'A' + index);
		return 0;
	}
	pfloppy_media_event();

	pfloppy_media_t m;
	pfloppy_get_media(&m);
	if (!m.present)
	{
		st_sectors = 0;
		st_wp = 1;
		return 0;
	}
	st_wp = m.write_protected;

	st_sectors = m.total_sectors;
	int sps = m.heads == 2 ? st_sectors / 2 : st_sectors;
	if (m.spt > 12) sps /= 2;
	int ok = (m.heads == 2 ? st_sectors > 85 * 12 : st_sectors <= 85 * 12) && sps >= 79 * 9 && sps <= 85 * 12;
	pf_log("disk given to the core: %dK, %d sectors/track, %d side(s), %d cylinders, %s%s", st_sectors / 2, m.spt, m.heads, m.cylinders,
		st_wp ? "write-protected" : "writable", ok ? "" : " (a geometry the core may not take)");
	return (uint64_t)st_sectors * 512;
}

int pfloppy_st_write_protected(void)
{
	return st_slot < 0 || st_wp;
}

void pfloppy_st_unmount(int index)
{
	if (index != st_slot) return;
	pfloppy_detach();
	st_slot = -1;
	st_sectors = 0;
}

int pfloppy_st_sd_service(int disk, int op, uint32_t lba, int sz, int ack)
{
	if (st_slot < 0 || disk != st_slot || !op) return 0;

	static uint8_t buf[64 * 512];
	if (sz > (int)sizeof(buf)) sz = sizeof(buf);
	int n = sz / 512;

	if (op & 1)
	{
		for (int k = 0; k < n; k++)
		{
			uint32_t s = lba + k;
			if ((int)s >= st_sectors)
			{
				memset(buf + k * 512, 0, 512);
				continue;
			}
			int r = pfloppy_read(s, buf + k * 512);
			if (!r) return -1;                       // not read yet: the core waits
			if (r < 0) memset(buf + k * 512, 0, 512); // unreadable
		}
		EnableIO();
		spi_w(UIO_SECTOR_RD | ack);
		spi_block_write(buf, user_io_get_width(), sz);
		DisableIO();
		return 1;
	}

	// a write: to the sector engine, which writes it to the disk in the background
	EnableIO();
	spi_w(UIO_SECTOR_WR | ack);
	spi_block_read(buf, user_io_get_width(), sz);
	DisableIO();
	if (lba + n > (uint32_t)st_sectors || pfloppy_write(lba, buf, n) < 0)
	{
		// write-protected, no disk or past its end: the data is lost
		if (!st_write_logged)
		{
			st_write_logged = 1;
			pf_log("the ST wrote sector %u: not written (%s)", lba, st_wp ? "write-protected disk" : "no disk or outside it");
		}
	}
	return 1;
}

void pfloppy_st_poll(void)
{
	if (!st_placeholder_done)
	{
		st_placeholder_done = 1;
		pfloppy_create_placeholder_named(HomeDir(), PFLOPPY_ST_PLACEHOLDER);
	}
	if (st_slot < 0) return;
	if (pfloppy_media_event())
	{
		// mount again: the core sees the new size, TOS the write-protect change
		char path[1024];
		snprintf(path, sizeof(path), "%s", st_path);
		tos_insert_disk(st_slot, path);
	}
}
