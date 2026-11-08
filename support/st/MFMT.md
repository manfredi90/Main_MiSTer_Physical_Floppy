# MFMT: bit-cell tracks for the Atari ST core

MFMT is how Main hands copy-protected floppy images (STX now; IPF, SCP and
live Greaseweazle reads later) to the Atari ST core. Main converts every format
into one revolution of MFM bit cells, with timing and weak bits. The core plays
the cells back like a drive and decodes them like a WD1772. Plain `.st` images
keep using the existing sector interface.

The structures and constants are defined in `st_mfmt.h`. This file explains
what each side has to do with them.

## Why whole tracks and not a live stream

The Minimig flux mode streams words through a FIFO while the disk turns, so
Main has to keep up all the time. The ST core already has the hps_io sd block
interface, which moves up to 16 KiB in one request (`sd_blk_cnt`). That is
enough for one DD revolution (100 000 cells = 12.5 KiB). It is also how the C64
core loads G64 tracks. So the core requests a whole track when the head lands
on it, keeps it in block RAM and loops over it. After that Main has nothing to
do until the next step.

## Detecting an MFMT image

At mount time Main sets bits 63..56 of `img_size` to `1` for MFMT images and to
`0` for sector images. The low 32 bits still hold the file size.

## Requesting a track

There are two reads, both with `sd_lba[31] = 1`:

| part  | sd_lba                          | sd_blk_cnt | bytes  |
|-------|---------------------------------|------------|--------|
| META  | `0x8000_0000 \| head<<8 \| cyl`   | 7          | 4096   |
| CELLS | `0x8001_0000 \| head<<8 \| cyl`   | 31         | 16384  |

The core requests META first, then CELLS, whenever the cylinder, the side or
the selected drive changes. The head settles for 15-30 ms after a step, which
covers the transfer and Main's polling. `sd_buff_addr` has to be widened to 13
bits, since one request covers 8192 words. A write (`sd_wr`) to an MFMT LBA is
accepted and dropped for now, because the images are read-only.

## META part

| offset | content |
|--------|---------|
| 0x0000 | header (64 bytes): magic `MFMT`, version 1, flags, `cell_count`, `timing_count`, `weak_count`, cylinder, head, `pool_words`, `rev_time_ns` |
| 0x0040 | timing table: up to 256 × {`start_word`, `cell_len`} |
| 0x0440 | weak table: up to 48 × {`start_word`, `n_words`} |
| 0x0500 | weak mask pool: up to 1408 words |

Flags: `FORMATTED` (when clear, the core reads noise), `WEAK`, `VARTIME`,
`HD`, and `TRUNCATED` (diagnostics only).

## CELLS part

Cell *n* is bit `15 - n%16` of the little-endian word *n/16*. The core shifts
each word out MSB first. One word holds 16 cells, the MFM of one data byte.

## Playing the track back (drive side)

- **Position.** A cell counter runs from 0 to `cell_count-1` and wraps. The
  index pulse starts at cell 0 and lasts about 4 ms (time, not cells).
- **Timing.** The phase accumulator gains 500 per 32 MHz clock, because one
  clock is 31.25 ns and the unit is 1/16 ns. When it reaches the current
  `cell_len`, subtract `cell_len` and move to the next cell. The current entry
  of the timing table changes when the word index reaches the next entry's
  `start_word`, and resets to entry 0 at the index. The track keeps turning
  while it loads: keep counting time and show "no flux" until both parts have
  arrived.
- **Weak cells.** While the word index lies inside a weak segment, take the
  mask word from the pool (one per word, segments in order). Every cell whose
  mask bit is 1 reads as a pseudo-random bit (LFSR), different on every pass.
  Main only marks data cells. Nothing else is needed for fuzzy-bit
  protections.
- **Unformatted** (`FORMATTED = 0`): every cell is random. No sync mark can
  ever be found.

## Decoding (WD1772 side)

The drive delivers cells with their timing, so the controller does not need a
PLL: data bits are the odd cells after synchronisation.

- **Address marks.** A1 sync is `0x4489`, C2 sync is `0x5224`, looked for in a
  16-cell shift register at every cell. ID field: A1 A1 A1 FE + 4 bytes + CRC.
  Data field: A1 A1 A1 F8..FB + data + CRC, searched within 43 bytes of the end
  of the ID. The CRC is CCITT, initialised to 0xFFFF, covering the A1s.
- **Read address** returns the 6 bytes as read, with a CRC error if they do
  not match. **Read sector** compares track and sector, takes the size from
  the ID (128..1024), and reports a CRC error, "record not found" (5
  revolutions) and the deleted-data flag from what was actually read. All of
  this comes from the cells, so odd IDs, duplicated sectors, intentional CRC
  errors and sectors nested inside other sectors need no special case.
- **Read track.** The byte counter re-aligns on every `0x4489` or `0x5224`,
  including false ones inside data. On a resync the byte being assembled is
  output only if more than 8 cells have gone by since the last byte boundary.
  Otherwise the counter is just reset. This is the behaviour measured on real
  chips (see MAME `wd_fdc.cpp`). It produces the `14 A1 A1` / `C2 A1 A1`
  sequences seen in real read-track dumps, and it changed the match of the
  test images from 85 % to 98 %. Read track starts with the cell under the
  index (cell 0) and ends before the next cell 0. The shift register keeps
  the cells that precede the index, so a sync mark that straddles the index
  is still found.
- **Writes.** For now, write sector and write track on an MFMT image end
  at once with the write-protect bit set, as on a protected disk.

## Reference implementation in the core

AtariST_MiSTer, branch `st-stx`:

- `rtl/fdc1772/mfmt_track.sv`: the drive (loader, playback, weak cells,
  index).
- `rtl/fdc1772/mfm_dec.sv`: the byte assembly (search, field and
  read-track modes).
- `rtl/fdc1772/fdc1772.sv`: the read address / read sector / read track
  engine on the decoded bytes. It is used when `img_size[63:56] == 1`;
  `.st` images keep the sector path.
- `sim/mfmt`, `sim/fdc`: Verilator testbenches. The C++ side serves the
  tracks with this converter and compares against `test/wdsim`.

## STX conversion (Main)

`st_stx.cpp` follows Hatari's reading of the fields:

- The ID field (first A1) is at `bit_position/8 - 4` bytes, read as a
  fraction of a revolution. A sector with an explicit read time or a timing
  table is given that time (data bytes). Free bytes share the rest of the
  200 ms, and later IDs are moved so that they stay on time.
- When the STX has a usable track image, the bytes and gaps come from it and
  the IDs are located inside it. When fewer than half of the IDs can be found,
  the image is ignored.
- Fuzzy mask 0 bits become weak data cells. The STX flags decide CRC errors
  (a CRC is deliberately corrupted when needed), RNF and deleted DAM.
- An image with no sectors and no sync marks is a dump of an unformatted
  track and becomes noise.

## Tests

A host tool, kept outside Main, converts each track, plays it back with a
software WD1772 using the rules above, and compares read address, read sector
(data, CRC, DAM, fuzzy variation, transfer time) and read track with the STX.
Results:

- 492 real tracks (Lotus Esprit Turbo Challenge, The Chaos Engine 1/2), all
  passing.
- Synthetic images for every protection case: odd and duplicated IDs, sizes
  128-1024, ID and data CRC errors, missing data, deleted data, fuzzy bits,
  Macrodos-style timing tables, slow sectors, track images, nested sectors,
  long tracks, unformatted tracks.
