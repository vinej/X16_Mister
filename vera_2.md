# VERA_2 — SDRAM Bitmap Layer (640×480, 4bpp / 8bpp)

An optional high-resolution **linear framebuffer** layer for the X16‑MiSTer core.

VERA's register model advertises bitmap modes, but its 128 KB VRAM can't hold a
640×480×8 image (that needs 300 KB). This layer stores the framebuffer in the
MiSTer **SDRAM** instead and scans it out **composited over VERA**. VERA itself
is untouched — sprites, tiles, scrolling, PSG/PCM/YM audio, SD, RTC, etc. all
keep working exactly as before.

| | |
|---|---|
| Resolution | **640 × 480**, 1:1 with the output raster (no scaling) |
| Depths | **8bpp** (256 colors) and **4bpp** (16 colors) |
| Palette | independent **256 × RGB444** (4 bits/channel) |
| Registers | **`$9F60`–`$9F6F`** (I/O expansion area) |
| Enable | OSD **“VERA2 Bitmap Layer”** master switch, default **Off** |
| SDRAM | **1 MB** dedicated (not VRAM/HiRAM): 8bpp *displays* 307,200 B, 4bpp 153,600 B — the rest is **save‑under scratch** (a full‑screen save‑under fits) |

> ⚠️ **Not real X16 hardware.** This is a core‑specific extension. Software that
> uses it runs only on this core (or an emulator that copies the register spec).
> Always feature‑detect via the **ID register** and fall back gracefully.

> 🚨 **Breaking change in this revision — `$9F64` (ADDR_H) is now
> `{incr[3:0], ptr[19:16]}`.** The upper nibble used to be ignored; it now
> selects the `DATA` **auto‑increment stride** ([§3.1](#31-auto-increment-stride-addr_h74)).
> Any program that wrote a non‑zero value into `$9F64[7:4]` — for example
> storing a full 24‑bit offset's bank byte without masking it to 4 bits — will
> now walk the pointer with the wrong stride instead of `+1`. **Mask ADDR_H
> writes to `$0F`** (or set the stride you want deliberately). The layer is new
> and experimental, so this revision does **not** keep the old behaviour
> compatible. The bitstream, the `-bitmap2` emulator and the `demo/` `.PRG`s
> must all be from this revision or later — mixing revisions gives wrong
> strides, not a clean failure.

---

## 1. Turning it on

1. **OSD master switch.** In the MiSTer menu set **VERA2 Bitmap Layer → On**. This is
   the master enable; with it **Off** the machine is bit‑identical to stock and
   `$9F60`–`$9F6F` read as open bus.
2. **Software enable.** Write the `CTRL` register to select a mode (below).

Both must be true for anything to show. When the layer is active it **replaces
the whole active display** with the framebuffer (there is no per‑pixel
transparency in this version) — disable it to return to VERA's normal output.
Video timing (HSync/VSync/DE) always comes from VERA; the bitmap rides its
raster.

---

## 2. Register map (`$9F60`–`$9F6F`)

| Addr | Name | R/W | Description |
|------|------|-----|-------------|
| `$9F60` | **CTRL** | R/W | `bit0` = enable, `bit2:1` = mode, `bit3` = passthru (VERA sprites/opaque pixels show **over** the bitmap — keeps the hardware mouse cursor visible). Write `(passthru<<3)\|(mode<<1)\|enable`. Read = `{4'b0, passthru, mode, enable}`. |
| `$9F61` | **ID** | R | Fixed **`$B5`** — feature‑detect signature. |
| `$9F62` | **ADDR_L** | R/W | Framebuffer pointer, bits `[7:0]` (shared by reads and writes). Reads back the *current* pointer. |
| `$9F63` | **ADDR_M** | R/W | Pointer bits `[15:8]`. |
| `$9F64` | **ADDR_H** | R/W | `{incr[3:0], ptr[19:16]}` — pointer bits `[19:16]` (**20‑bit** linear byte pointer → 1 MB) **and** the auto‑increment **stride** select ([§3.1](#31-auto-increment-stride-addr_h74)). One store sets both. |
| `$9F65` | **DATA** | R/W | **Write:** store a byte at the pointer, then advance it by the **stride**. **Read:** return the byte at the pointer, then advance (the read‑back that lets a GUI *save under* a dialog). |
| `$9F66` | **PAL_IDX** | W | Set the palette write cursor (0–255). |
| `$9F67` | **PAL_LO** | W | Low byte of a palette entry: `{G[3:0], B[3:0]}`. |
| `$9F68` | **PAL_HI** | W | High byte `{----, R[3:0]}` → **commits** `{R,G,B}` to `PAL_IDX`, then cursor `++`. |
| `$9F69`–`$9F6B` | **BLIT_DST** | W | Blit destination byte address (20‑bit, L/M/H). |
| `$9F6C`–`$9F6E` | **BLIT_LEN** | W | Blit length in bytes (20‑bit, L/M/H). |
| `$9F6F` | **BLIT_CTRL** | R/W | **Write `1`** = start a blit: copy `BLIT_LEN` bytes from the **`ADDR` pointer** (`$9F62-64`) to `BLIT_DST`, entirely inside SDRAM. **Read bit0** = busy. |

### Fast save-under with the blit

Copying a dialog's pixels to CPU banked RAM is expensive (8bpp eats 4× the bytes, and banked RAM is scarce). Instead, keep a **scratch region in the off‑screen part of the framebuffer**. The framebuffer address space is 1 MB (the 20‑bit pointer) but 8bpp only *displays* the first 307,200 bytes, so bytes `307200…1048575` (**≈724 KB**) are free scratch — enough for a **full‑screen** save‑under (307 KB) with room to spare. Use the **blit** (an SDRAM→SDRAM hardware copy) to move regions there and back:

```
; save under: copy `len` bytes from the region to scratch
set ADDR  = region_start        ; $9F62-64  (blit source)
set BLIT_DST = scratch_offset    ; $9F69-6B
set BLIT_LEN = len               ; $9F6C-6E
POKE $9F6F, 1                    ; start
:  LDA $9F6F : AND #1 : BNE :-   ; wait for busy=0
; ... draw the dialog ...
; restore: copy it back (source = scratch, dest = region)
set ADDR  = scratch_offset
set BLIT_DST = region_start
set BLIT_LEN = len
POKE $9F6F, 1  : wait busy
```

The blit is byte‑wise, so any source/destination alignment works. It runs **below** the scanout in priority (the display never glitches) and doesn't touch CPU RAM. Save‑unders of full‑width scanline strips are contiguous, so one blit covers a whole band. *(Coherency: the blit reads what's already in SDRAM — a pixel the CPU wrote microseconds earlier should be flushed first, which it is for anything drawn a frame or more ago.)*

### Building a GUI (dialogs, menus, a mouse cursor)

- **Read‑back for save‑under.** Before drawing a transient element (dialog, drop‑down), set the pointer to the covered region and **read** it out through `$9F65` into RAM; on close, set the pointer back and **write** it. The `$9F65` read is coherent with prior writes (it drains the write path first). Reads via the debugger do **not** auto‑increment.
- **Keep the mouse visible.** Set `CTRL bit3` (passthru). VERA's hardware sprites — including sprite‑0 (the KERNAL mouse pointer) — then composite **over** the bitmap, while the bitmap fills everywhere VERA is transparent. Disable VERA's layers so only the sprites show through. With passthru = 0 (default) the bitmap fully covers VERA.
- **4bpp partial pixels** need read‑modify‑write (2 px/byte): read the byte, change one nibble, write it back. Set the stride to **`$1` (hold)** first and the read leaves the pointer *on* the byte, so the write‑back needs no pointer reload:
  `lda #$10 : ora hi : sta $9F64` → `lda $9F65` → merge the nibble → `sta $9F65`.
  8bpp needs no RMW (1 px/byte), so it's the simpler target for a toolkit.

### Mode field (`CTRL[2:1]`)

| Mode | Meaning | `CTRL` value to enable |
|------|---------|------------------------|
| 0 | off | `$00` |
| 1 | **640×480 × 8bpp** | `$03` |
| 2 | **640×480 × 4bpp** | `$05` |

---

## 3. Pixel format

You address the framebuffer as a **linear byte array** starting at pointer 0.
The hardware handles the internal SDRAM layout — you never deal with planes or
addresses beyond the 20‑bit pointer (1 MB).

### 8bpp (mode 1)
- 1 byte per pixel = a palette index `0…255`.
- Row stride = **640 bytes**.
- Byte offset of pixel `(x, y)` = **`y*640 + x`**.
- Framebuffer size = `640*480` = **307,200** bytes (`$4B000`).

### 4bpp (mode 2)
- 2 pixels per byte = palette indices `0…15`.
- **High nibble = left pixel, low nibble = right pixel.**
- Row stride = **320 bytes**.
- Byte offset of pixel `(x, y)` = **`y*320 + (x>>1)`**;
  `x` even → high nibble (`bits 7:4`), `x` odd → low nibble (`bits 3:0`).
- Framebuffer size = `320*480` = **153,600** bytes (`$25800`).

Because `DATA` auto‑increments, the fast path is to set the pointer once and
**stream** bytes in row‑major order (top‑left to bottom‑right). Random‑access
plotting just means computing the offset above and loading `ADDR_L/M/H` first.

### 3.1 Auto-increment stride (`ADDR_H[7:4]`)

`DATA` advances the pointer by a **selectable signed stride**, chosen by the
upper nibble of `$9F64` — the same bit position as VERA's `ADDRx_H` increment
field, so the idiom is the one you already know from VRAM. The difference: this
table is **signed**, so both directions live in one field and there is no
separate `DECR` bit to set.

| `incr` | Stride | Typical use |
|---|---|---|
| `$0` | **+1** | linear streaming — **the default after reset** |
| `$1` | **0** | *hold* the pointer: 4bpp read‑modify‑write, re‑read the same byte |
| `$2` | +2 | every other pixel (dither, 2‑px patterns) |
| `$3` | +4 | |
| `$4` | +8 | |
| `$5` | +16 | |
| `$6` | +32 | |
| `$7` | +64 | |
| `$8` | +128 | |
| `$9` | +256 | |
| `$A` | **+320** | 4bpp row stride — one pixel **down** |
| `$B` | **+640** | 8bpp row stride — one pixel **down** |
| `$C` | −1 | reverse streaming, overlapping copies |
| `$D` | −2 | |
| `$E` | **−320** | 4bpp row stride — one pixel **up** |
| `$F` | **−640** | 8bpp row stride — one pixel **up** |

Because the nibble shares a register with `ptr[19:16]`, the stride costs
**nothing** to set — the single `sta $9F64` you already do when loading the
pointer carries it:

```asm
; vertical line at (x,y0) downwards, 8bpp: ONE store per pixel
        lda off+0 : sta $9F62
        lda off+1 : sta $9F63
        lda off+2 : and #$0F : ora #$B0 : sta $9F64   ; ptr[19:16] + stride +640
        ldx #height
        lda #colour
:       sta $9F65                          ; each write steps down one row
        dex
        bne :-
```

Without a stride that inner loop needs a 24‑bit `+640` and three pointer
stores per pixel (~30 cycles); with it, a 4‑cycle `sta`. Note the `and #$0F`
before the `ora`: `ADDR_H` only holds `ptr[19:16]`, so a 24‑bit offset's bank
byte must be masked or it lands in the stride field. Mask it the same way when
you *don't* want a stride — a bare `sta $9F64` of a bank byte is now a bug.

The pointer is **readable** (`$9F62`–`$9F64`), so a loop can hand its end
position to the next routine instead of recomputing it. It is 20 bits and
**wraps modulo 1 MB**, so a negative stride from 0 lands at `$FFFFF`.

Two things the stride does *not* speed up, so you know where not to reach for
it: **row‑major fills** are already optimal at `+1`, and **rectangle** blits
still need a per‑row pointer reload (there is no `640 − width` entry — use the
[blit](#fast-save-under-with-the-blit) for bulk moves instead).

---

## 4. Palette

256 entries, each **RGB444** (12‑bit color). Mode 8bpp uses all 256; mode 4bpp
uses entries 0–15.

To set entry `N` to `(R,G,B)` (each 0–15):

```
POKE $9F66, N              : REM PAL_IDX = N
POKE $9F67, (G*16) + B     : REM PAL_LO  = {G,B}
POKE $9F68, R              : REM PAL_HI  = {R}  -> commits, cursor -> N+1
```

To load a **run** of entries, set `PAL_IDX` once and then write `PAL_LO`/`PAL_HI`
pairs — the cursor advances after every `PAL_HI`.

---

## 5. Feature detection

```asm
    lda $9F61
    cmp #$B5
    bne  no_bitmap      ; not this core (or OSD switch is off / real HW open bus)
```

On real X16 hardware `$9F61` is floating bus, so a stable `$B5` read means the
layer is present. Keep a non‑bitmap fallback path for portability.

---

## 6. Quick start (ca65 / 6502)

```asm
; --- detect + select 8bpp, load a grayscale palette, clear to color 0 ---

        lda $9F61
        cmp #$B5
        bne  no_bitmap

; grayscale ramp: entry i -> gray v = (i>>4) in all channels (tmp = zero page)
        stz $9F66             ; PAL_IDX = 0
        ldx #0
palloop:
        txa
        lsr a : lsr a : lsr a : lsr a   ; A = v = i>>4  (0..15)
        sta tmp
        asl a : asl a : asl a : asl a   ; A = v<<4
        ora tmp               ; A = (v<<4)|v = {G,B}
        sta $9F67             ; PAL_LO
        lda tmp
        sta $9F68             ; PAL_HI = {R} -> commit, cursor++
        inx
        bne palloop

; enable 8bpp
        lda #$03
        sta $9F60

; pointer = 0
        stz $9F62
        stz $9F63
        stz $9F64

; clear the screen to color in A ($4B000 = 307200 bytes = 1200 * 256)
        lda #$00             ; fill color
        ldx #<1200
        ldy #>1200
        stx cnt
        sty cnt+1
outer:  ldx #0
inner:  sta $9F65            ; DATA (auto-increments the pointer)
        inx
        bne inner            ; 256 bytes
        lda cnt              ; 16-bit decrement of cnt
        bne :+
        dec cnt+1
:       dec cnt
        lda cnt
        ora cnt+1
        bne outer
        rts
```

Plot a single 8bpp pixel at `(x,y)` = compute `off = y*640 + x`, then:

```asm
        lda off+0 : sta $9F62
        lda off+1 : sta $9F63
        lda off+2 : and #$0F : sta $9F64   ; ADDR_H: ptr[19:16], stride = +1
        lda color : sta $9F65              ; write the pixel
```

The `and #$0F` matters: `$9F64[7:4]` is the [stride select](#31-auto-increment-stride-addr_h74),
so an unmasked bank byte would set a stride you didn't ask for.

---

## 7. Quick start (BASIC) — switch to the mode, then back

BASIC can't POKE 307,200 pixels one at a time (that would take minutes), so this
fills the whole screen **fast with the blit** — seed 16 pixels, then *double*
them across the framebuffer — shows it, and returns to BASIC on a keypress:

```basic
10 IF PEEK($9F61)<>$B5 THEN PRINT "NO BITMAP LAYER":END
20 REM 16-colour ramp: entry I -> R=I, G=15-I, B=I
30 POKE $9F66,0
40 FOR I=0 TO 15:POKE $9F67,(15-I)*16+I:POKE $9F68,I:NEXT
50 POKE $9F60,3                              : REM enable 8bpp
60 REM seed the first 16 pixels = colours 0..15
70 POKE $9F62,0:POKE $9F63,0:POKE $9F64,0
80 FOR I=0 TO 15:POKE $9F65,I:NEXT
90 REM fill the whole screen by doubling that seed with the BLIT
100 L=16
110 IF L>=307200 THEN 200
120 D=L:IF L+D>307200 THEN D=307200-L
130 GOSUB 500
140 L=L+D:GOTO 110
200 REM picture is up -- wait for a key, then back to BASIC
210 GET A$:IF A$="" THEN 210
220 POKE $9F60,0                             : REM bitmap OFF -> text screen returns
230 END
500 REM blit: copy L bytes from the start (ADDR=0) to offset L
510 POKE $9F62,0:POKE $9F63,0:POKE $9F64,0
520 H=INT(L/256):POKE $9F69,L-256*H:POKE $9F6A,H-256*INT(H/256):POKE $9F6B,INT(L/65536)
530 H=INT(D/256):POKE $9F6C,D-256*H:POKE $9F6D,H-256*INT(H/256):POKE $9F6E,INT(D/65536)
540 POKE $9F6F,1
550 IF PEEK($9F6F)AND 1 THEN 550
560 RETURN
```

You get fine vertical colour stripes; press any key and the BASIC prompt returns
(disabling the bitmap reveals VERA's text screen, untouched). The same doubling
trick clears to a **solid** colour — just seed one pixel (`POKE $9F65,C` with
`L=1`). A tested assembly version of exactly this is `demo/vera2fill.s`.

---

## 8. Notes, performance & limitations

- **Compositing.** Default: the bitmap fully replaces VERA. With `CTRL bit3`
  (passthru), VERA's opaque pixels (sprites — incl. the mouse — and layers)
  show over the bitmap; the bitmap fills where VERA is transparent.
- **`DATA` is readable** (`$9F65` read → byte at the pointer, then the stride is
  applied), which is what makes GUI save‑under and 4bpp read‑modify‑write
  possible. The **pointer** is readable too (`$9F62`–`$9F64`).
- **The stride is free but narrow.** Setting it costs nothing (it rides the
  `ADDR_H` store you already do) and turns column‑major drawing from ~30 cycles
  per pixel into 4. It does **not** make fills or row blits faster — `+1` was
  already optimal there — and there is no `stride − width` entry for
  rectangles. See [§3.1](#31-auto-increment-stride-addr_h74).
- **Frame coherency.** Pixel writes travel through the SDRAM write FIFO and land
  a few microseconds later — invisible for display (the next frame always shows
  the finished data), so no need to sync.
- **SDRAM bandwidth.** 8bpp scanout uses roughly **¾** of the SDRAM read
  bandwidth during active display, so HiRAM‑heavy programs may run slightly
  slower while it's on; 4bpp uses about a third and is negligible. Turning the
  layer off frees all of it.
- **No pixel doubling.** 640×480 is 1:1 with the output; there is no 320×240
  mode in this version (those would pixel‑double). Higher resolutions
  (800×600, 1024×768) are **not** possible — the video path is fixed to VERA's
  25 MHz / 640×480 raster.
- **SDRAM footprint.** The layer reserves a **1 MB** window of SDRAM (the 20‑bit
  byte pointer), entirely separate from VERA VRAM and the CPU's HiRAM/cart banks.
  Of that window, 8bpp *displays* the first 307,200 bytes (4bpp: 153,600);
  everything above is **save‑under scratch** — ≈724 KB, enough for a full‑screen
  save‑under (307 KB image + 307 KB copy = 600 KB) with headroom left for future
  use (double‑buffering, more modes). So a display‑only framebuffer is ~300 KB,
  but with save‑under the working set is larger — hence the 1 MB reservation.
  Internally the bytes are stored **planar** — even and odd bytes in two separate
  SDRAM byte‑planes — which the hardware hides, but it's why the region occupies
  two areas of the chip.

---

## 9. Register cheat‑sheet

```
$9F60 CTRL     W (passthru<<3)|(mode<<1)|enable   R {0,0,0,0,passthru,mode,enable}
$9F61 ID       R $B5
$9F62 ADDR_L  RW ptr[7:0]
$9F63 ADDR_M  RW ptr[15:8]
$9F64 ADDR_H  RW {incr[3:0], ptr[19:16]}   (20-bit ptr = 1 MB; MASK to $0F if
                                            you don't mean to set a stride)
$9F65 DATA     W byte @ptr, ptr+=stride   R byte @ptr, ptr+=stride  (save-under)

incr:  0=+1(default)  1=0(hold)  2=+2  3=+4  4=+8  5=+16  6=+32  7=+64
       8=+128  9=+256  A=+320  B=+640  C=-1  D=-2  E=-320  F=-640
$9F66 PAL_IDX  W palette cursor
$9F67 PAL_LO   W {G[3:0],B[3:0]}
$9F68 PAL_HI   W {----,R[3:0]}  commit + cursor++
$9F69 BDST_L   W blit dst[7:0]       $9F6C BLEN_L  W len[7:0]
$9F6A BDST_M   W blit dst[15:8]      $9F6D BLEN_M  W len[15:8]
$9F6B BDST_H   W blit dst[19:16]     $9F6E BLEN_H  W len[19:16]
$9F6F BCTRL    W bit0=start (copy LEN bytes ADDR->BDST in SDRAM)   R bit0=busy

modes:  0 off   1 = 640x480x8bpp ($9F60=$03)   2 = 640x480x4bpp ($9F60=$05)
8bpp:   1 byte/px, stride 640,  off = y*640 + x
4bpp:   2 px/byte (hi nibble=left), stride 320, off = y*320 + (x>>1)
```

---

## 10. Demos

Ready-to-run examples live in **[`demo/`](demo/)** (source + assembled `.PRG`).
Both feature-detect `$9F61`, so the *same* binary runs on the emulator and on
hardware.

| File | What it shows |
|---|---|
| `vera2fill.s` / `VERA2FILL.PRG` | Switch to 8bpp, fill the whole screen fast with the **blit** (doubling a 16-colour seed), wait for a key, return to BASIC. The assembly form of the §7 BASIC example. |
| `vera2incr.s` / `VERA2INCR.PRG` | The **auto-increment stride** ([§3.1](#31-auto-increment-stride-addr_h74)): 15 full-height vertical lines drawn with stride **+640** (one `sta` per pixel), and a rectangle outline drawn by *walking the perimeter* — the pointer is loaded **once** and each edge just changes the stride (`+1`, `+640`, `-1`, `-640`), reading `$9F64` back to preserve `ptr[19:16]`. Starts with a **self-test** that prints a warning if the machine predates the stride field. |
| `vera2blit.s` / `VERA2BLIT.PRG` | The full picture: an 8bpp gradient, 16 random **VERA sprites** + the **mouse** floating over it (passthru), a green top bar = the **write/read-back** self-test, and **left-click save-under** — click the gradient to drop a message box (the covered band is blitted to scratch), click the box to restore it exactly. |

**Build** (cc65):

```
ca65 --cpu 65C02 vera2fill.s -o vera2fill.o
ld65 -C vera2demo.cfg vera2fill.o -o VERA2FILL.PRG
```

**Run** — emulator: `x16emu -bitmap2 -prg VERA2FILL.PRG -run`.
On hardware: turn on **Bitmap Layer** in the OSD, then `LOAD`/`RUN` the `.PRG`.
