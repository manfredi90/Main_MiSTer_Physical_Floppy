# MiSTer Main – Physical Floppy Support

Use real floppy disks on your MiSTer. Plug in a USB floppy drive, or a
[Greaseweazle](https://github.com/keirf/greaseweazle), pop in a disk and the core reads it live, just like the real machine
would. No need to make disk images first.

> **Heads up: this project was made with the help of AI.** The code was
> written together with Claude, and tested on real hardware
> with real disks. It works for me and I think it's neat, but keep a backup of the disks you
> care about.

## Thanks

This project was inspired by the work of
[Anime0t4ku](https://github.com/Anime0t4ku) and his
[Main_MiSTer_Physical_Disc](https://github.com/Anime0t4ku/Main_MiSTer_Physical_Disc),
which lets the MiSTer play real CDs from a USB drive. Seeing real discs
spinning on a MiSTer made me want the same thing for floppies. Thank you!

## What works

| Core | USB floppy drive | Greaseweazle | Saving to the disk |
|---|---|---|---|
| ao486, z486 | ✔ | ✔ | ✔ |
| PCXT, PCXT-EGA, PCAT-286 | ✔ | ✔ | ✔ |
| Amiga (Minimig), "flux" mode | – | ✔ | – (read only) |
| Amiga (Minimig), "AmigaDOS" mode | – | ✔ | ✔ |
| MacPlus | – | ✔ | – (read only) |
| MacLC, MacSE30, MacIIcx | – | ✔ | – (read only) |

- **PC cores**: 720K and 1.44MB disks (and 360K/1.2MB ones with a 5.25"
  drive on the Greaseweazle, not tested yet). The drive only reads what the PC
  asks for, like a real one, and whatever you save goes straight back to the
  disk.
- **Amiga**, two ways to use it:
  - **Flux mode** hands the raw disk to the core, so it works with any Amiga
    disk, *copy protected games included*. Read only.
  - **AmigaDOS mode** works with normal AmigaDOS disks (Workbench, most games
    and demos) and lets you save to them.
- **Mac**: 400K and 800K Mac disks. The MacLC, MacSE30 and MacIIcx also take
  720K and 1.44MB disks. Mac disks are read only on purpose: they are getting
  rare, and writing them with a PC drive doesn't work with every drive.

Disk images keep working exactly as before: you only get the real drive when
you pick the special file described below.

## What you need

- A MiSTer, of course
- One of these:
  - a **USB floppy drive**, the common kind you find online (tested with a
    Sony USB-FDU). PC cores only.
  - a **Greaseweazle** (tested with V4.1) with a normal 3.5" PC
    floppy drive. Works with every core in the table.
- For the Amiga **flux mode**: a Minimig core from October 10, 2026 or newer
  (for now that's an unstable build). AmigaDOS mode works with any Minimig.
- For the **MacPlus**: an unstable build from September 17, 2026 or newer. The
  MacLC, MacSE30 and MacIIcx releases from late September / October 2026 are
  fine.

## Installation

1. Download `MiSTer_Physical-Floppy` from the [Releases](../../releases) page
   and copy it to the root of your SD card, next to the normal `MiSTer` file.
   Don't replace the normal one!
2. Tell the MiSTer to use it for the cores you want: add these lines at the
   end of your `MiSTer.ini` (and of any other ini you use, like
   `MiSTer_HDMI.ini`). Only add the cores you actually use.

   ```ini
   [ao486]
   main=MiSTer_Physical-Floppy

   [z486]
   main=MiSTer_Physical-Floppy

   [PCXT]
   main=MiSTer_Physical-Floppy

   [PCXT-EGA]
   main=MiSTer_Physical-Floppy

   [PCAT-286]
   main=MiSTer_Physical-Floppy

   [Minimig]
   main=MiSTer_Physical-Floppy

   [MacPlus]
   main=MiSTer_Physical-Floppy

   [MacLC]
   main=MiSTer_Physical-Floppy

   [MacSE30]
   main=MiSTer_Physical-Floppy

   [MacIIcx]
   main=MiSTer_Physical-Floppy
   ```

   When you load one of these cores the MiSTer switches to this Main by
   itself, and goes back to the normal one for every other core.
3. Plug in your drive and start the core.

## How to use it

The first time you start a supported core, a small empty file shows up in its
games folder. Picking that file in the OSD, where you'd normally pick a disk
image, means "use the real drive":

| Core | File to pick | Where in the OSD |
|---|---|---|
| ao486, z486, PCXT, PCXT-EGA, PCAT-286 | `_USB_FLOPPY_.img` | Floppy A: or B: (works with the Greaseweazle too) |
| Amiga, flux mode | `_GREASEWEAZLE_.adf` | DF0: (or another drive) |
| Amiga, AmigaDOS mode | `_GREASEWEAZLE_DOS_.adf` | DF0: (or another drive) |
| Mac | `_GREASEWEAZLE_.dsk` | the floppy drive |

After that, use the drive just like on the real computer: swap disks whenever
you want and the core notices.

A few things worth knowing:

- **Mac disks are loaded all at once.** The Mac cores read the whole disk as
  soon as it goes in, so you wait about 20 seconds for a 400K disk and 40 for
  an 800K one before the Mac sees it (about a minute for 1.44MB). That's just
  how these cores work.
- **Eject Mac disks from the Mac first** (drag them to the trash, or
  Cmd+E), then take the disk out of the drive.
- **Wait for the drive to stop before taking a disk out**, especially right
  after saving something: changes reach the disk a moment after the computer
  saves them.
- **Write protect your precious disks** (slide the little tab). Windows, for
  example, likes to write to a floppy even when you're only copying files
  from it.

## Optional settings

These go in the `[MiSTer]` section of your ini, or in a core's section:

| Setting | What it does |
|---|---|
| `PFLOPPY_DRIVE=0` | Use whatever drive is connected, Greaseweazle first (default) |
| `PFLOPPY_DRIVE=1` | USB floppy drive only |
| `PFLOPPY_DRIVE=2` | Greaseweazle only |
| `PFLOPPY_GW_DRIVE=0` | Find the drive on the Greaseweazle automatically (default) |
| `PFLOPPY_GW_DRIVE=1` or `2` | The drive is on the Greaseweazle's A or B connector |
| `PFLOPPY_PRELOAD=1` | PC cores: read the whole disk in the background as soon as it goes in. Can help when a program gives up because the drive is too slow. Off by default, so the drive only reads what's needed, like a real one. |

## Zaparoo users

If you use [Zaparoo](https://zaparoo.org), its NFC reader detection pokes at
every USB serial port it finds, and that includes the Greaseweazle. The two
end up fighting over it and the floppy stops working.

Until Zaparoo learns to leave the Greaseweazle alone, here's a workaround if
you **don't use a PN532 NFC reader**: open `zaparoo/config.toml` on your SD
card and add this at the end:

```toml
[readers.drivers.pn532]
auto_detect = false
```

Then restart the MiSTer. (If you do use a PN532 reader, configure it by hand
in Zaparoo instead of relying on auto detection.)

## Known issues

- **Windows 95 on ao486/z486** can show floppy errors, or even a blue screen,
  while copying files from a floppy. It happens with disk images too, so it's
  a core thing, not a problem with the real drive.
- **Amiga flux mode**: a few demos with very tricky loaders don't start (for
  example *Desert Dream* by Kefrens). They work in AmigaDOS mode, or from an
  ADF.
- **Mac disks** are read only, and slow to load (see above).

## If something goes wrong

Everything the drive does ends up in `logs/pfloppy.log` on your SD card:
which drive was found, which disk went in, read and write errors and so on.
It's the first place to look, and please attach it if you open an issue.

## Building

Same as the normal Main, see the
[MiSTer compile guide](https://mister-devel.github.io/MkDocs_MiSTer/developer/mistercompile/#general-prerequisites-for-arm-cross-compiling).
All the floppy code lives in `support/physical_floppy`.

## Credits

- [Anime0t4ku](https://github.com/Anime0t4ku), for Main_MiSTer_Physical_Disc
  and the idea behind all this
- [Keir Fraser](https://github.com/keirf), for the Greaseweazle and its host
  tools (public domain): the flux decoding here follows their PLL, and the Mac
  disk code comes from them
- The Mac checksum code goes back to
  [FluxEngine](https://github.com/davidgiven/fluxengine) by David Given and to
  [MAME](https://www.mamedev.org)
- Sorgelig and all the MiSTer developers, for the MiSTer and its Main
- The authors of the cores this works with

Based on [Main_MiSTer](https://github.com/MiSTer-devel/Main_MiSTer) and, like
it, licensed under the GPL v3.
