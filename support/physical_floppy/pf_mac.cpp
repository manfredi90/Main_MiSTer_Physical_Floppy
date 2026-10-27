/*
 * Physical floppy for the Macintosh cores: the Greaseweazle drive in a floppy
 * slot of the core (MacPlus S2/S3, MacLC S6, MacSE30/MacIIcx S0/S1).
 *
 * These cores load the whole disk image into its memory when the image is
 * mounted (floppy_loader.v) and asks for it sector by sector: the requests
 * are served from the sector engine (physical_floppy.cpp, Mac GCR decoded in
 * pf_gw.cpp / gcr_mac.cpp), a sector not read yet is answered later (the core
 * waits). The image is in the order of the .dsk files: cylinder by cylinder,
 * side 0 then side 1, 12 sectors per track on cylinders 0-15 down to 8 on
 * 64-79. The engine has 12 slots on every track: an image sector maps to
 * (cylinder * heads + side) * 12 + sector.
 * The SuperDrive cores (MacLC, MacSE30) also take 720K / 1.44 MB MFM disks:
 * read as PC disks, the image is in the engine's order.
 *
 * A disk inserted, removed or changed is mounted again: the core reloads it.
 * Read-only for now.
 */

#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "physical_floppy.h"
#include "pf_drive.h"
#include "gcr_mac.h"
#include "../../user_io.h"
#include "../../spi.h"

static int mac_slot = -1;           // slot with the placeholder, -1 none
static int mac_gcr = 1;             // the disk is GCR (else MFM)
static char mac_path[1024];
static int mac_heads = 0;
static int mac_sectors = 0;         // image sectors
static uint16_t mac_map[1600];      // image sector -> engine sector
static int mac_placeholder_done = 0;
static double mac_load_t0 = -1;     // the core is loading the disk since (ms)

int pfloppy_is_mac_placeholder(const char *path)
{
	if (!path || !path[0]) return 0;
	const char *p = strrchr(path, '/');
	p = p ? p + 1 : path;
	return !strcasecmp(p, PFLOPPY_MAC_PLACEHOLDER);
}

// the Mac cores with a floppy loaded at insertion (block device slot)
static const struct { const char *name; int superdrive; } mac_cores[] =
{
	{ "MACPLUS", 0 },
	{ "MACLC",   1 },
	{ "MACSE30", 1 },
	{ "MACIICX", 1 },
};

int pfloppy_mac_core(void)
{
	for (auto &c : mac_cores)
		if (!strcasecmp(user_io_get_core_name(0), c.name) || !strcasecmp(user_io_get_core_name(1), c.name))
			return c.superdrive ? 2 : 1;
	return 0;
}

static void build_map(int heads)
{
	int n = 0;
	for (int cyl = 0; cyl < 80; cyl++)
		for (int side = 0; side < heads; side++)
			for (int s = 0; s < mac_spt(cyl); s++)
				if (n < 1600) mac_map[n++] = (uint16_t)((cyl * heads + side) * MAC_MAX_SPT + s);
	mac_sectors = n;
	mac_heads = heads;
}

uint64_t pfloppy_mac_mount(int index, const char *path)
{
	if (mac_slot >= 0 && mac_slot != index) pfloppy_mac_unmount(mac_slot);
	snprintf(mac_path, sizeof(mac_path), "%s", path);
	mac_load_t0 = -1;

	if (mac_slot != index)
	{
		int sd = pfloppy_mac_core() == 2;
		pfloppy_set_format(sd ? PFLOPPY_FORMAT_MAC_SD : PFLOPPY_FORMAT_MAC);
		// not waiting for the drive: the core may be running from its hard disk,
		// whose requests must keep being served. The slot is empty until the
		// disk is found (pfloppy_mac_poll mounts it then)
		pfloppy_attach_async(index);
		mac_slot = index;
		pf_log("%s floppy slot %d: %s disks, the whole disk is read when it is inserted, read-only", user_io_get_core_name(0), index,
			sd ? "400K/800K GCR and 720K/1.44 MB MFM" : "400K/800K GCR");
		mac_sectors = 0;
		return 0;
	}
	pfloppy_media_event();

	pfloppy_media_t m;
	pfloppy_get_media(&m);
	if (!m.present)
	{
		mac_sectors = 0;
		return 0;
	}
	mac_gcr = (m.spt == MAC_MAX_SPT);
	if (mac_gcr)
	{
		build_map(m.heads);
		pf_log("disk given to the core: %s GCR", mac_heads == 2 ? "800K" : "400K");
	}
	else
	{
		mac_sectors = m.total_sectors;
		pf_log("disk given to the core: %dK MFM%s", mac_sectors / 2, (mac_sectors == 1440 || mac_sectors == 2880) ? "" : " (not a Mac size: the core will not take it)");
	}
	return (uint64_t)mac_sectors * 512;
}

void pfloppy_mac_unmount(int index)
{
	if (index != mac_slot) return;
	pfloppy_detach();
	mac_slot = -1;
	mac_sectors = 0;
}

int pfloppy_mac_sd_service(int disk, int op, uint32_t lba, int sz, int ack)
{
	if (mac_slot < 0 || disk != mac_slot || !op) return 0;

	static uint8_t buf[64 * 512];
	if (sz > (int)sizeof(buf)) sz = sizeof(buf);
	int n = sz / 512;

	if (op & 1)
	{
		for (int k = 0; k < n; k++)
		{
			uint32_t s = lba + k;
			if ((int)s >= mac_sectors)
			{
				memset(buf + k * 512, 0, 512);
				continue;
			}
			int r = pfloppy_read(mac_gcr ? mac_map[s] : s, buf + k * 512);
			if (!r) return -1;                       // not read yet: the core waits
			if (!s && mac_load_t0 < 0)
			{
				mac_load_t0 = pf_now_ms();
				pf_log("the core loads the disk");
			}
			if (r < 0) memset(buf + k * 512, 0, 512); // unreadable
		}
		EnableIO();
		spi_w(UIO_SECTOR_RD | ack);
		spi_block_write(buf, user_io_get_width(), sz);
		DisableIO();
		if (mac_load_t0 >= 0 && (int)(lba + n) >= mac_sectors)
		{
			pf_log("disk loaded by the core in %.1f s", (pf_now_ms() - mac_load_t0) / 1000);
			mac_load_t0 = -1;
		}
		return 1;
	}

	// a write (the image is mounted read-only, the core should not write): take it, drop it
	EnableIO();
	spi_w(UIO_SECTOR_WR | ack);
	spi_block_read(buf, user_io_get_width(), sz);
	DisableIO();
	return 1;
}

void pfloppy_mac_poll(void)
{
	if (!mac_placeholder_done)
	{
		mac_placeholder_done = 1;
		pfloppy_create_placeholder_named(HomeDir(), PFLOPPY_MAC_PLACEHOLDER);
	}
	if (mac_slot < 0) return;
	if (pfloppy_media_event())
	{
		// mount again: the core sees the new size and reloads the disk
		char path[1024];
		snprintf(path, sizeof(path), "%s", mac_path);
		user_io_file_mount(path, mac_slot, 0, 0);
	}
}
