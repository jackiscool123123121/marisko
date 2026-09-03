# AGENTS.md — marisko (SP-1 custom firmware)

Shared local handoff doc for Claude Code, Codex, and OpenCode working on this checkout.
Git-ignored — never commit this file.

## Project

Custom Zephyr/NCS firmware for the Teenage Engineering SP-1 stem player (nRF52840). Forked from
`softmodded/marisko`. Companion host tool `rome` (CLI + desktop app) lives in a sibling repo
(`../jack-rome`, upstream `jackiscool123123121/rome`).

Target board: `sp1` (nRF52840), defined under `boards/arm/sp1/`. Build:
```
west build -b sp1 -d build app -- -DBOARD_ROOT=$(pwd)
```
Toolchain: Zephyr SDK 0.17.4 + nRF Connect SDK, via a python venv at `/Volumes/LLMDATA/sp1/venv`
(needed for `pyelftools`, used by the build's `gen_kobject_list.py`).

Release flow: after any firmware change meant to ship, objcopy the `.elf` to `.bin`, `dd skip=131072`
per BUILDING.md, sanity-check the vector table (SP/PC) via a `struct.unpack_from('<II', ...)` on the
first 8 bytes, copy to `~/Desktop/marisko.bin`, commit, tag `vX.Y.Z`, push, and
`gh release create` on `jackiscool123123121/marisko` with the `.bin` attached. **Every firmware
push should be followed by a matching release** — this is a standing user instruction, not optional.

## THE BIG FIVE (never violate these)

1. App must be placed at flash address `0x20000`, max size `0xdefff`.
2. Firmware must feed the watchdog every 5 seconds or less (`feed_wdt()`).
3. lfclk/hfclk and some peripherals (pwm2, pwm3, saadc) are already started by the bootloader —
   re-initializing them can fail silently or hang.
4. The SP-1 has no hard reset button — firmware must provide a `SYSTEM_OFF` path back to the
   bootloader (see `enter_system_off()` in main.c).
5. `RESETREAS` must be cleared before entering `SYSTEM_OFF`.

## Architecture (verified on hardware this session)

- **Threads**: feed thread (`audio.c`, priority 5, ADPCM decode + eMMC + I2S — never starve it),
  UI thread `ui_main()` (main.c, priority 1, ABOVE the feed thread — buttons/faders/LED meters live
  here specifically so the feed thread's ~20-40ms uninterruptible eMMC bursts can't swallow a
  button press), main thread (priority 10, BELOW the feed thread — only USB/settings-flush/power
  logic belongs here, nothing latency-sensitive).
- **eMMC**: bit-banged CMD/CLK/DAT0 + SPIM3-accelerated CMD25 multi-block writes, single shared
  recursive bus lock (`s_bus_lock` in emmc.c) — do NOT add a second, separate lock for
  disk.c/anything else; that caused a real deadlock earlier (disk.c vs. the upload path).
- **Audio mix path**: `decode_block()` in audio.c is the ONE place that decodes ADPCM + mixes 4
  stems + applies per-stem gain — both the normal prefetch path and the in-RAM loop cache call it,
  so any per-sample effect (gate, future effects) belongs there, not duplicated per caller.
- **Loop feature** (v0.6.0, built by Codex/OpenCode, not Claude — see Recent Changes): hold play to
  loop ~1 "bar" (`LOOP_BASE_BLOCKS=375` blocks / divisor from `{1,2,4,8,16}`) at the current
  position; served from an in-RAM cache (`s_audio_buf` union, shared storage with the prefetch
  buffer + baked-level RAM since they're never both live while looping) for a seamless wrap with no
  eMMC seek stall at the seam. Momentary only — releasing play ends it. No latch (explicitly
  rejected by the user in an earlier Claude session; do not re-add one).
- **Gate effect** (v0.6.1-v0.6.4, Claude): hold function + hold one or more track buttons → gates
  those stems (bitmask, `audio_set_gate_mask()`), ~125ms fixed on/off cycle (`GATE_CYCLE_BLOCKS=47`
  in audio.c), applied as a per-stem gain-zero in `decode_block()`. Per TE's own basic-mode guide:
  "Function+Track buttons will activate an audio effect on the corresponding stem. Only gate effect
  is used in basic mode." **Known conflict, fixed**: `main()`'s power-off hold reads the SAME
  function-button GPIO (P0.27) completely independently of `ui_main()` — a naive gate implementation
  made holding function+track also start the shutdown countdown/LED-fill underneath the gate. Fixed
  with two shared flags: `s_gate_gesture` (main() ignores fn_down while a gate is actively engaged)
  and `s_fn_hold_tainted` (once a function hold has been used for ANY gate, that whole physical hold
  stays disqualified from power-off — even after the track button releases — until function is
  fully released and pressed again). Both flags live in main.c near the top (search
  `s_gate_gesture`), NOT in audio.c — they're about arbitrating the two threads' shared GPIO read,
  not audio state.
- **Battery** (v0.5.0, Claude): `battery.c`/`battery.h`, BQ24232 charger IC on P0.21 (/CE)/P0.22
  (/CHG)/P0.24 (/PGOOD), voltage divider on AIN4 (SAADC). Pin mapping and RAW_EMPTY/RAW_FULL
  calibration constants are **ASSUMED**, carried over from an independent third-party SP-1 firmware
  project (`chattock/sp1-tape-looper`) reverse-engineering the same hardware — not independently
  verified against a real full/empty battery on THIS unit. Treat percent/quarters as approximate.
  New USB command `USB_CMD_BATTERY` (0x12).
- **Bluetooth: NOT implemented, and blocked on missing information — do not attempt without more
  data.** The nRF52840 has no antenna; Bluetooth (A2DP to speakers/headphones, per the TE user
  guide's "hold vol+/vol- to scan" gesture) is handled entirely by a SEPARATE Infineon
  CYW20706A2 module (inside the CYBT-353027-02 package) over UART: TX=P1.02, RX=P1.04, RTS=P1.01,
  CTS=P1.03, RESET=P0.10 (active-low), 115200 8N1 baud — **VERIFIED** by a third-party project
  (`bnjreece/feldd-sp1-firmware`, real hardware capture) that reflashed that module for BLE-MIDI.
  The module ships TE's own stock Bluetooth-Classic A2DP-sink app — we do NOT need to reflash it,
  we need to DRIVE it over UART with WICED-HCI-Control commands. Checked Infineon's own public
  reference (`Infineon/mtb-example-btsdk-audio-a2dp-sink`, the template this app is almost
  certainly built from): its stock command handler only implements one trivial command
  (TRACE_ENABLE) — pairing/discoverable-mode is hardcoded into the app's own BT-stack-enable
  callback, not exposed as a toggleable host command in the public template. TE's actual module
  firmware is customized beyond this template (that's how the vol+/vol- gesture works at all) and
  is not public. **UNKNOWN**: the actual opcode(s) TE's customized module firmware accepts for
  triggering scan/pairing, and whether "disable auto-reconnect" is even possible without
  reflashing that module (a separate, much higher-risk operation — see the module's own
  irrecoverable Static-Section warning in feldd's docs). Do not guess UART bytes at this chip
  blind; there is no way to verify without a real UART capture on hardware. If picked back up,
  the next step is either (a) a logic-analyzer capture of the nRF↔module UART traffic during a
  real vol+/vol- gesture on real hardware, or (b) locating TE's actual customized module source
  (not public as of this writing).

## Change Permissions

| Component | Claude | Codex | OpenCode |
|---|---|---|---|
| `app/src/audio.c` / `audio.h` (mix path, loop, gate) | Allowed | Allowed | Allowed |
| `app/src/main.c` (button/thread arbitration, power-off) | Allowed, care around `s_gate_gesture`/`s_fn_hold_tainted`/off-hold timing | Allowed | Allowed |
| `app/src/battery.c` / `battery.h` | Allowed | Allowed | Allowed |
| `app/src/usb.c` / `usb.h` (host protocol) | Allowed — coordinate with `rome` repo's `proto.rs` if adding/changing a command | Allowed | Allowed |
| Board files (`boards/`), linker/partition layout, `Kconfig.defconfig` | Approval required (BIG FIVE risk) | Approval required | Approval required |
| Bluetooth / module UART driver | **Do not implement blind** — see note above; needs real hardware capture or TE source first | same | same |

## Required Verification Before Shipping

1. `west build -b sp1 -d build app -- -DBOARD_ROOT=$(pwd)` — must be clean (warnings OK, errors not).
2. Objcopy + `dd skip=131072`, then sanity-check SP/PC from the vector table are in-range
   (SP `0x20000000-0x20040000`, PC within `0x20000..0x20000+0xdefff`).
3. Renode boot-health check when touching early-boot/init code: `g_fail_step` byte should read
   `0x02` (healthy) after ~18 virtual seconds, no fatal handler triggered. (Skipped for the gate
   effect's later iterations under time/budget pressure — small, localized gain-multiply changes,
   verified by inspection instead. Flag if this becomes a pattern.)
4. Real hardware behavior (buttons, LEDs, audio) can only be verified by the user — Claude has no
   physical access to the device. Say so explicitly rather than claiming a UI/UX behavior "works."

## Recent Changes

### 2026-09-02 — Claude (Sonnet 5, Claude Code)
- **Task:** Add gate effect per TE's basic-mode guide; multiple follow-up fixes from live user testing.
- **Changed:** `app/src/audio.c`, `app/src/audio.h`, `app/src/main.c`.
- **What/Why/How:** See "Gate effect" under Architecture above for the full description and the
  power-off-hold conflict fix.
- **Impact:** New USB-visible-none, purely local button gesture + audio path change. FLASH usage
  +~150 bytes total across the four commits (203484→203676 B, still 19.4% of 1MB).
- **Dependencies:** None outside this repo.
- **Side Effects:** None known beyond the fixed power-off conflict.
- **Risk:** Low — no new peripherals, no eMMC/flash-layout changes, all changes are in already-
  running threads' existing per-tick logic.
- **Agent Permissions:** Any agent may extend the gate effect (e.g. real subdivision-based rate
  instead of the fixed 47-block cycle) — no special restriction beyond the general audio.c/main.c
  rows above.
- **Verification:** Clean `west build` each commit (4 iterations: v0.6.1 add, v0.6.2 power-off
  fix, v0.6.3 taint-hold fix, v0.6.4 multi-stem). Renode-verified for v0.6.1 only (see note above).
  Vector-table sanity-checked every build. **User-tested on real hardware** and confirmed the loop
  feature (built by a prior Codex/OpenCode session, not this one) "works flawlessly"; the gate
  effect's power-off conflict (v0.6.1→v0.6.2) and the mid-hold re-arm issue (v0.6.2→v0.6.3) were
  both found via real hardware testing by the user, not caught statically — a reminder that this
  class of bug (two threads independently reading the same physical button) is easy to miss in
  review.
- **Unresolved:** Bluetooth (see Architecture note — blocked on missing info, do not guess).
  Battery calibration constants are still unverified assumptions (see Architecture note).
- **Handoff:** Next agent picking up Bluetooth should NOT write UART bytes to the module without
  either a real capture or TE's actual source — re-read the Architecture note above first.

— Claude
