# marisko

**custom firmware for the teenage engineering sp-1 stem player.**  

named after the [marisko flower](https://en.wikipedia.org/wiki/Cypripedium_calceolus)  
originally forked from [softmodded/marisko](https://github.com/softmodded/marisko)

> **status: work in progress.** the firmware has been fixed with ai help but is not
> finished yet. flashing instructions will be added here once it is stable — for
> now i'd hold off flashing production hardware. see [releases](https://github.com/jackiscool123123121/marisko/releases).

## features

`this project is in the very early stages of development, so as of now pretty much anything could pass as a "feature"`

- **bouncing light animation** across the 4 playback leds
- **function button powers off** device (`SYSTEM_OFF`) and returns it to the bootloader
- **watchdog is fed** preventing bootloops
- **audio playback** — IMA-ADPCM stems decoded from eMMC and streamed to the CS42L42 / TAS2505 codecs via I2S
- **stem management over USB CDC-ACM** — songs uploaded, listed, and removed via [rome](#rome) at ~390 KB/s
- **HW stack protection** — stack overflows surface as faults instead of silent corruption

## hardware

- nRF52840 (runs a stripped [Zephyr RTOS](https://www.zephyrproject.org/) app at flash `0x20000`, max `0xdefff`)
- eMMC storage for stems (eMMC protocol via `emmc.c`/`disk.c`)
- CS42L42 (headphone codec) + TAS2505 (speaker amp) over I2C/I2S
- saadc / pwm for the volume & transport ladder and playback leds

## return to bootloader

the sp-1 has no hard reset so to get back:

1. press **function** and wait for the leds go dark, this means device shut off
2. press any button (or plug in usb) — bootloader opens
3. use the [solderless firmware utility](https://solderless.engineering) or `rome flash` to flash new firmware

> you can also trigger this from the host: `rome bootloader` powers the device off
> (SYSTEM_OFF), then pressing **function** wakes it straight into the bootloader.

## rome

**[rome](https://github.com/jackiscool123123121/rome)** is the companion cli — flash firmware, enter the bootloader, and manage stems on the device over USB. **it is required to use marisko.**

install it with a one-liner:

**macOS / Linux (sh):**
```
curl -sSL https://raw.githubusercontent.com/jackiscool123123121/rome/main/install.sh | sh
```

**Windows (PowerShell):**
```
irm https://raw.githubusercontent.com/jackiscool123123121/rome/main/install.ps1 | iex
```

on a fresh device you **must run `rome format` before loading any music** —
it writes the disk header (v3) that stems get stored against:

```
rome format --yes
```

then upload songs:
```
rome song add "my song" drums.wav vocals.wav bass.wav other.wav
```

## building

see **[building.md](BUILDING.md)** for setup, compilation, and flashing instructions.

## contributing

see **[contributing.md](CONTRIBUTING.md)** for rules & guidelines.

## credits

- **[zephyr rtos](https://www.zephyrproject.org/)** + **[nrf connect sdk](https://www.nordicsemi.com/Products/Development-software/nrf-connect-sdk)** — the foundation this firmware runs on
- **[sp-1 developer wiki](https://github.com/timknapen/SP-1-dev)** by tim knapen — hardware documentation, pinouts, and bootloader specs
- **[solderless](https://solderless.engineering)** — the web-based firmware and stem loader that makes all of this possible without opening the device

## project structure

- `app/` — firmware source (`main.c` + modular `audio` / `codec` / `emmc` / `disk` / `usb` / `leds` / `saadc` / `pwm`)
- `boards/arm/sp1/` — custom zephyr board definition for the sp-1
- `patches/` — required zephyr tree patches (i2s ratio + fast cdc-acm upload)

## prebuilt firmware

`marisko.bin` is the built firmware binary (see [releases](https://github.com/jackiscool123123121/marisko/releases) / `build/`). **this is a wip build — do not flash production hardware yet.** once stable, flashing instructions land here.

## license

[MIT](LICENSE)
