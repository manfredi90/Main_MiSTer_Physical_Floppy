#!/usr/bin/env python3
"""
Mr. BIOS 1.65 (286, 64K) floppy timeout patch, for the MiSTer PCAT-286 core
used with a real floppy drive (Main_MiSTer_Physical_Floppy).

A real floppy drive that has been idle needs to spin up its motor before it
can answer, and that can take a little over a second. The core doesn't know
the motor was off, and the BIOS only waits about one second for the floppy
controller before reporting "drive not ready" / "error on drive A".

This script makes a patched copy of YOUR Mr. BIOS ROM that waits about
3 seconds instead. Only the floppy wait routine is changed (3 bytes), plus
the checksum byte so the BIOS still passes its self test. The original file
is not touched.

Usage:
    python patch_mrbios_floppy.py MR_BIOS_1.65_286_64K.ROM

It writes MR_BIOS_1.65_286_64K_floppy3s.ROM next to it. Select that file as
the BIOS in the PCAT-286 OSD. Mr. BIOS is commercial software: the patched
file is for your own use, please don't share it.
"""

import hashlib
import os
import sys

ROM_SIZE = 0x10000

# offset: (original bytes, patched bytes)
PATCHES = {
    0xDC51: (bytes([0xB8, 0xFF, 0x80]), bytes([0xB8, 0xFF, 0xFF])),  # outer wait loop 0x80 -> 0xFF
    0xDC57: (bytes([0xB9, 0x00, 0x20]), bytes([0xB9, 0x00, 0x40])),  # inner wait loop 0x2000 -> 0x4000
    0xDC6C: (bytes([0x3C, 0xE6]), bytes([0x3C, 0xC8])),              # timeout 25 timer ticks -> 55 (~3 s)
}
CHECKSUM_OFFSET = 0xFFFF

# the ROM this was made for, and the result
KNOWN_ORIGINAL_MD5 = "23a62bc4467ab3b820b17b5f4a7a24f5"
KNOWN_PATCHED_MD5 = "3aaf363d36c02d772b697c63b901cfeb"


def fail(msg):
    print("Error: " + msg)
    sys.exit(1)


def main():
    if len(sys.argv) != 2:
        print(__doc__.strip())
        sys.exit(1)

    src = sys.argv[1]
    try:
        with open(src, "rb") as f:
            rom = bytearray(f.read())
    except OSError as e:
        fail("cannot read %s: %s" % (src, e))

    if len(rom) != ROM_SIZE:
        fail("%s is %d bytes, expected a 64K ROM (%d bytes)" % (src, len(rom), ROM_SIZE))
    if sum(rom) & 0xFF:
        fail("the ROM checksum is wrong: is this a complete, unmodified BIOS dump?")

    already = all(rom[o:o + len(new)] == new for o, (old, new) in PATCHES.items())
    if already:
        fail("this ROM is already patched")
    for o, (old, new) in PATCHES.items():
        if rom[o:o + len(old)] != old:
            fail("unexpected bytes at 0x%04X: this is not the Mr. BIOS 1.65 286 64K this patch is for" % o)

    if hashlib.md5(rom).hexdigest() != KNOWN_ORIGINAL_MD5:
        print("Note: this ROM is not byte for byte the one the patch was tested with,")
        print("but the code to change is exactly where expected, so going on.")

    for o, (old, new) in PATCHES.items():
        rom[o:o + len(new)] = new
    rom[CHECKSUM_OFFSET] = (rom[CHECKSUM_OFFSET] - (sum(rom) & 0xFF)) & 0xFF
    assert sum(rom) & 0xFF == 0

    base, ext = os.path.splitext(src)
    dst = base + "_floppy3s" + (ext or ".ROM")
    if os.path.exists(dst):
        fail("%s already exists, remove it first" % dst)
    with open(dst, "wb") as f:
        f.write(rom)

    md5 = hashlib.md5(rom).hexdigest()
    print("Written %s (md5 %s)" % (dst, md5))
    if md5 == KNOWN_PATCHED_MD5:
        print("Identical to the tested patched ROM.")
    print("Copy it to games/PCAT-286 on your SD card and select it as the BIOS in the PCAT-286 OSD.")


if __name__ == "__main__":
    main()
