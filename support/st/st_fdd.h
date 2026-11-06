// st_fdd.h - Atari ST floppy images served to the core as MFMT tracks
#ifndef ST_FDD_H
#define ST_FDD_H

#include <stdint.h>
#include "../../file_io.h"

// img_size bits 63..56 tell the core how to use the image:
// 0 = sector image (.st), 1 = MFMT track image (STX, ...)
#define ST_FDD_SIZE_TYPE_SHIFT  56
#define ST_FDD_TYPE_MFMT        1ULL

// Called from user_io_file_mount for every mounted image. Returns 1 if the
// mount can go on (also for files that are not ours), 0 to reject the file.
int  st_fdd_mount_hook(int index, const char *name, fileTYPE *f, int *writable);
void st_fdd_unmount(int index);

// Size reported to the core for slot 'index'
uint64_t st_fdd_size(int index, uint64_t size);

// sd request service: 0 = not ours, 1 = handled, -1 = stop
int  st_fdd_sd_service(int disk, int op, uint64_t lba, int sz, int ack);

#endif
