# Commander X16 for [MiSTer](https://github.com/MiSTer-devel/Main_MiSTer/wiki)

Note: 128M SDRAM module is needed for the X16 core.

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
| `/rom/X16_20260714.rbf`  | version 1.0 no baked ROM      | `_Computer/`    |
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
  * banks **32–255** — the X16 **cartridge space**, fully **readable and
    writable** (backed by SDRAM). Software can copy ROM images into these
    banks and run them in place, or use them as 3.5 MB of extra storage —
    matching the real X16, where cartridge banks may be RAM, ROM or flash.
    Fill them from the OSD with **Load Cart** or at core start with
    `boot2.rom` (see Usage above).

The vector page $FFFA–$FFFF always reads from ROM bank 0, so interrupts keep
hitting the KERNAL handlers whatever bank is selected.


## License

MIT — see [LICENSE](LICENSE). Third-party components (MiSTer framework, VERA,
IKAOPM, the R65C02 core, and the X16 ROM) remain under their own licenses, as
listed in the LICENSE file.

Thanks!
