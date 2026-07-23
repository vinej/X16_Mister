# VERA_2 bitmap-layer demos

Example programs for the SDRAM bitmap layer (`$9F60`–`$9F6F`). See
[`../vera_2.md`](../vera_2.md) for the full register spec. Both demos
feature-detect `$9F61`, so the same `.PRG` runs on the emulator **and** on real
hardware.

| Source | PRG | What it shows |
|---|---|---|
| `vera2fill.s` | `VERA2FILL.PRG` | Switch to 8bpp, fill the whole screen fast with the **blit** (doubling a 16-colour seed), wait for a key, return to BASIC. |
| `vera2incr.s` | `VERA2INCR.PRG` | The **auto-increment stride** (`$9F64[7:4]`): vertical lines drawn with stride **+640**, and a rectangle outline drawn by walking the perimeter with `+1`, `+640`, `-1`, `-640` from a single pointer load. Self-tests the stride first and says so if your build predates it. |
| `vera2blit.s` | `VERA2BLIT.PRG` | 8bpp gradient + 16 random **VERA sprites** + the **mouse** over it (passthru); **left-click** the gradient drops a message box (band saved to scratch via the blit), **click the box** to restore it exactly. |

> ⚠️ **These `.PRG`s need a bitstream/emulator with the auto-increment stride**
> (`$9F64` = `{incr[3:0], ptr[19:16]}`). On an older build `VERA2INCR` prints a
> warning instead of drawing; the other two still work, since they use the
> default `+1` stride. See the breaking-change note in
> [`../vera_2.md`](../vera_2.md).

`vera2demo.cfg` is the cc65 linker config both use (a minimal `$0801` PRG with a
BASIC `SYS` stub).

## Build

Needs [cc65](https://cc65.github.io/):

```
ca65 --cpu 65C02 vera2fill.s -o vera2fill.o
ld65 -C vera2demo.cfg vera2fill.o -o VERA2FILL.PRG

ca65 --cpu 65C02 vera2incr.s -o vera2incr.o
ld65 -C vera2demo.cfg vera2incr.o -o VERA2INCR.PRG

ca65 --cpu 65C02 vera2blit.s -o vera2blit.o
ld65 -C vera2demo.cfg vera2blit.o -o VERA2BLIT.PRG
```

## Run

**Emulator** (must be built with the `-bitmap2` device — see the core repo):

```
x16emu -bitmap2 -prg VERA2FILL.PRG -run
```

**Hardware** (X16-MiSTer): turn on **Bitmap Layer** in the OSD, copy the `.PRG`
to the SD card, then `LOAD"VERA2FILL.PRG"` / `RUN` (or load it however you
normally load programs).
