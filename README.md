# Commander X16 for [MiSTer](https://github.com/MiSTer-devel/Main_MiSTer/wiki)

Note: 128M SDRAM module is needed for the X16 core.

## version 1.3 :

**Serial / network card — dual UART + ROMTERM fixes.** The serial/network card
now exposes **two independent UART windows** at `$9FE0` and `$9FE8` (previously
both were aliased to a single UART). A new OSD **MiSTer UART** selector maps
MiSTer's one host UART to a single guest port (`$9FE0` or `$9FE8`) at a time, and
the active-low RTS/DTR handshake is corrected so **ROMTERM** works with its
default RTS/CTS on the first port selection. PS/2 keyboard events are also now
queued, so modifier/key bursts are no longer dropped while earlier bytes are
being serialized. *(Contributed by jnngill.)*

**EXPERIMENTAL — "VERA2" bitmap layer.** An optional SDRAM-backed **640×480
4bpp / 8bpp** linear bitmap layer — the high-resolution mode VERA advertises but
its 128 KB of VRAM can't hold — mapped at `$9F60`–`$9F6F` and composited over
VERA. It is **off by default**; nothing changes for existing software unless you
turn it on.

**How to activate:** in the OSD, set **VERA2 Bitmap Layer → On**. A program then
drives the registers at `$9F60`–`$9F6F` (write pixels, load the palette, use the
SDRAM→SDRAM blit for fast fills / save-under). With the layer **Off** the core
behaves exactly as before.

**New: a selectable auto-increment stride**, like VERA's VRAM port. `$9F65`
(DATA) used to advance the pointer by exactly one byte; it now advances by a
stride chosen with the upper nibble of `$9F64` — `+1` (the default), `0` for
read-modify-write, `±320`/`±640` to step a whole row, and the powers of two in
between. Setting it is free (it rides the `ADDR_H` store you already make), and
it turns column-major drawing from ~30 cycles per pixel into 4. The pointer
registers `$9F62`–`$9F64` are now readable too.

> 🚨 **Breaking change — `$9F64` is now `{incr[3:0], ptr[19:16]}`.** Its upper
> nibble used to be ignored and is now the stride select, so a program that
> stored an unmasked 24-bit bank byte there will walk the pointer with the
> wrong stride. **Mask `ADDR_H` writes to `$0F`.** The layer is new and
> experimental, so this revision deliberately does *not* preserve the old
> behaviour. Update the **bitstream, the `-bitmap2` emulator and the `demo/`
> `.PRG`s together** — mixing revisions gives wrong strides rather than a clean
> failure. `VERA2INCR.PRG` self-tests the field and tells you if a build is
> stale.

* **Documentation / register spec** — [vera_2.md](vera_2.md)
* **Demos** — source (`.s`) + `.PRG` + a ready-to-mount SD image — [demo/](demo/)
* **Emulator that supports v1.3** — a fork of the X16 emulator adding the same
  layer via the `-bitmap2` flag, so you can develop and test VERA2 programs on a
  PC: **[github.com/vinej/x16-emulator](https://github.com/vinej/x16-emulator)**

**Cartridge banks 32–255 now behave per-bank, like real hardware.** On a real
X16 those banks are whatever a physical cartridge provides — ROM, RAM, flash, or
nothing — so a plain `STA` only writes where the cartridge actually has RAM. The
core no longer treats *all* of banks 32–255 as writable RAM unconditionally.
There are **three distinct cartridge operations**, all separate from **Load ROM**
(`F1`, the system ROM in banks 0–15):

* **Load Cart** (`F2`) — stream a raw cart **ROM** image into banks 32+ and run
  it, like inserting a game cartridge (one-shot, read-only).
* **Mount Cart** (`SC2`) — mount a persistent cart image, in one of two modes set
  by the **Cart mount** toggle:
  * **RAM** *(default)* — the whole cart space (banks 32–255) is writable RAM,
    the image restores linearly, and CPU writes are **persisted** back to the
    image (the classic "Mount Cart RAM" — save data or scratch RAM).
  * **Typed** — the image is a typed **`.crt`** built with
    [`makecart`](https://github.com/vinej/x16-emulator); the core parses its
    header for the **per-bank** ROM / RAM / NVRAM layout, so a `STA` sticks on the
    RAM/NVRAM banks and is dropped on the ROM banks, bit-for-bit the same as
    `x16emu -cart game.crt`. RAM/NVRAM banks are writable in SDRAM but currently
    **volatile** (survive a reset, not a power cycle — matching the emulator);
    on-disk persistence is a planned follow-up.

A second toggle, **Cart Banks 32+** (**By Cart** / **All RAM**), is the write
override: *By Cart* (default) follows the mount's per-bank mask (no cart = ROM);
*All RAM* forces every bank 32–255 writable even with no cart mounted — the old
"3.5 MB of scratch RAM" behavior, same as the emulator fork's `-cartram` flag.

**Also in 1.3:**

* **Load ROM cart guard.** Pointing **Load ROM** at a typed `.crt` by mistake now
  writes nothing and leaves the current KERNAL intact instead of overwriting it
  and bricking the system — Load ROM refuses any file that begins with the
  `CX16 CARTRIDGE` magic. (A *raw* cart image has no signature, so it can't be
  told apart from a system ROM by content — use **Load Cart** for cart ROMs.)
* **SD hot-swap.** Mounting a different SD image now cold-reboots the X16 so its
  DOS re-mounts the new card automatically — no manual reset needed. Previously a
  hot-swap kept the old card's directory (a corrupted/empty listing) until you
  relaunched the whole core.

(Compatibility: old raw cart images keep working unchanged — **Mount Cart** with
its default **RAM** mode is exactly the old "Mount Cart RAM". Only the new typed
per-bank layout needs a `.crt` from `makecart` plus **Cart mount → Typed**.)

## version 1.2 :

Add serial card emulation done by jnngill

## version 1.1 :

Fixed a lock-up (frozen boot, no flashing prompt) when relaunching the core with an SD image still mounted. The auto-remounted image could collide with the ROM's boot-time SD probe and wedge the virtual SD card; the guest SD is now kept deselected for ~1.3 s after every reset so the probe always runs cleanly. If you were working around this by remounting the image or toggling the CPU after each launch, that is no longer needed.

## version 1.0 : 
The published `.rbf` contains **no ROM** — the X16 ROM is not ours to
redistribute — so you must supply your own `boot1.rom` at runtime.

Get a 256 KB X16 ROM (all 16 banks) — build it from X16Community/x16-rom, or use the rom.bin / *.BIN from a Commander X16 / emulator release.

This core was coded by Claude Opus 4.8 and Claude Fable 5 My participation was
testing, debugging, thinking, analysing... but not coding.

This port of the Commander X16 to the MiSTer FPGA platform (DE10-Nano,
Cyclone V SoC) is derived from a first version developed for the Cyclone V GX
(Terasic C5G) starter kit — an old board bought a long, long time ago.

Even with AI, it took around 3-4 weeks to achieve the goal because we had many
strange bugs to resolve. It worked only after switching to Claude MAX with Fable 5. 

## Installation

| File                     | Description                   | Where to put it |
|--------------------------|-------------------------------|-----------------|
| `/rom/X16_20260723.rbf`  | version 1.3 no baked ROM      | `_Computer/`    |
| your own boot1.rom       | tested with R49               | `games/X16/`    |

(The published `.rbf` contains **no ROM** — the X16 ROM is not ours to
redistribute — so you must supply your own `boot1.rom` at runtime, see above.
`rom/rom.hex` is **not** included in this repo.)

### Rebuilding `rom.hex` (optional — personal baked-in ROM build)

If you would rather bake the ROM into the bitstream for your **own personal
build** (so the core boots with no `boot1.rom` on the SD), regenerate
`rom/rom.hex` from a 256 KB X16 ROM binary and recompile:

1. Get a **256 KB X16 ROM** (all 16 banks) — build it from
   [X16Community/x16-rom](https://github.com/X16Community/x16-rom), or use the
   `rom.bin` / `*.BIN` from a Commander X16 / emulator release.
2. Convert it to the `$readmemh` format the core expects (262144 lines, one
   byte per line):

   ```
   python scripts/rom_bin2hex.py path/to/rom.bin rom/rom.hex
   ```

   (Run it with no output path to write `rom/rom.hex` automatically.)
3. Enable the baked-in ROM: in `X16.qsf`, uncomment
   `set_global_assignment -name VERILOG_MACRO "BAKE_ROM=1"` — this defines the
   `BAKE_ROM` switch that gates the `$readmemh` in `rtl/rom_banks.sv`.
4. Recompile in Quartus. The resulting `.rbf` now has the ROM baked in.

Keep `BAKE_ROM` commented and `rom/rom.hex` out of any repo you publish — a
bitstream with the ROM baked in redistributes the ROM indirectly.

* Bonus: `scripts/mister_send_input.py <mister-ip>` streams your PC keyboard
  to the MiSTer over the network (nothing to install on the MiSTer;
  PAUSE/BREAK quits, F12 is forwarded to the OSD).

## Peripherals

* **Audio**: YM2151 (FM) and VERA PSG + PCM are both mixed into the MiSTer
  audio output (HDMI and analog).
* **Game controllers**: two MiSTer gamepads appear as SNES controllers on
  the X16 joystick ports 1 and 2 (KERNAL `joystick_get`); map the buttons in
  the OSD ("Define joystick buttons": A, B, X, Y, L, R, Select, Start).
* **Mouse**: a mouse connected to the MiSTer works as the X16 PS/2 mouse
  (`MOUSE ON` in BASIC shows the pointer).
* **Interrupts**: VERA, VIA1, VIA2 and YM2151 IRQs are all wired to the CPU,
  so software using VIA timer or YM timer interrupts runs correctly.
* **SMC**: software reset ($02) and NMI ($03) requests over I2C work, and
  the X16 activity LED drives the MiSTer USER LED.
* **RTC + NVRAM**: the MCP7940N (I2C $6F) is implemented, fed by the
  MiSTer's Linux wall clock — `DATE`/`TIME` are correct at boot, the clock
  ticks on its own, and it can be set with `clock_set_date_time` (a set
  clock is not overwritten by the next MiSTer sync). The 64-byte NVRAM
  (KERNAL settings + checksum) works but is volatile: it survives resets,
  not power cycles.

## Banking

* **RAM banks** ($0000): 256 banks of 8 KB at $A000–$BFFF — 2 MB HiRAM
  (banks 0–1 in on-chip BRAM, the rest in SDRAM).
* **ROM banks** ($0001): the full 8-bit register is implemented, giving 256
  banks of 16 KB at $C000–$FFFF:
  * banks **0–15** — the system ROM (`boot1.rom` / baked-in image);
  * banks **16–31** — unpopulated system-ROM space, reads `$FF` like empty
    flash, writes are ignored;
  * banks **32–255** — the X16 **cartridge space** (SDRAM-backed, 3.5 MB).
    Their ROM/RAM nature follows the mounted cartridge, like real hardware (see
    the version 1.3 notes above). Fill them by running a raw ROM cart with
    **Load Cart** (`F2`) or `boot2.rom`, or by mounting an image with
    **Mount Cart** (`SC2`) in either **RAM** mode (all banks writable, persistent)
    or **Typed** mode (per-bank ROM/RAM/NVRAM from a `.crt`). The **Cart Banks
    32+** toggle overrides writability: **By Cart** (default) follows the mount's
    per-bank mask (no cart = ROM); **All RAM** forces every bank writable.

File conventions (v1.3): the **system ROM is `.bin`/`.rom`** (Load ROM / Load
Cart); a **typed cartridge is a `.crt`** (Mount Cart → Typed). To stop a typed
cartridge being loaded onto the ROM slot by mistake, Load ROM now **refuses any
file that begins with the `CX16 CARTRIDGE` magic** — it writes nothing, so the
current KERNAL is left intact.

The vector page $FFFA–$FFFF always reads from ROM bank 0, so interrupts keep
hitting the KERNAL handlers whatever bank is selected.


## License

MIT — see [LICENSE](LICENSE). Third-party components (MiSTer framework, VERA,
IKAOPM, the R65C02 core, and the X16 ROM) remain under their own licenses, as
listed in the LICENSE file.

Thanks!
