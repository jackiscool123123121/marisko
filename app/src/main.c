#include <zephyr/kernel.h>
#include <soc.h>

#include "util.h"
#include "leds.h"
#include "saadc.h"
#include "pwm.h"
#include "emmc.h"
#include "usb.h"
#include "codec.h"
#include "audio.h"
#include "disk.h"
#include "battery.h"

struct btn_db { bool raw_prev, stable; uint8_t cnt; };

/* Current 0..7 master volume level, and whether it's changed since the last
 * disk write. Declared here (not next to s_meter_ticks below, where this used
 * to live) because settings_flush() -- used by enter_system_off(), which sits
 * above the rest of main()'s UI state -- needs both in scope first.
 *
 * g_settings_dirty is volatile: it's now set by the UI thread (on a vol +/-
 * release, once that polling moved there -- see the big comment on ui_main
 * about why) and read/cleared by main(), which is the only thread allowed to
 * actually touch eMMC for it (see settings_flush's own comment). */
static volatile int s_vol_level = 3;
static volatile bool g_settings_dirty;

/* Speaker volume: 8 levels (0 = mute … 7 = loud) → TAS2505 P1/R46 attenuation
 * (0x00 = 0 dB loudest, larger = quieter). File scope: both main() (jack-sense
 * switch) and ui_main() (vol +/- button) apply it. */
static const uint8_t vol_r46[8] = {0x7F, 0x48, 0x3C, 0x30, 0x24, 0x18, 0x0C, 0x00};

/* Output routing: speaker (TAS2505) or headphones (CS42L42) -- exactly one is
 * unmuted at a time. Written by main()'s jack-sense polling, read by
 * ui_main()'s vol +/- handling to know which codec path to adjust. volatile
 * for that cross-thread visibility (same reasoning as g_settings_dirty). */
static volatile bool s_hp_out;

/* On any fatal error (incl. stack-overflow fault with HW_STACK_PROTECTION):
 * light all 8 LEDs solid and feed the WDT forever, so a crash is visible. */
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	(void)reason; (void)esf;
	for (int i = 0; i < NUM_PB_LEDS; i++) set_pb_on(i);
	/* Track LEDs are driven by PWM0, which owns those pins via PSEL — the GPIO
	 * OUTSET in set_trk_on() is overridden and they stayed dark, so a crash
	 * only lit half the panel. Go through PWM0 (its DMA keeps looping without
	 * the CPU, so this still works from a fault context). */
	for (int i = 0; i < NUM_TRK_LEDS; i++) pwm0_set_duty(i, PWM_TOP);
	for (;;) feed_wdt();
}

/* ── Power ─────────────────────────────────────────────────────────────────── */

/* Re-read the header fresh right before writing rather than keeping our own
 * cached copy: main's loop also runs usb_cdc_poll(), and a song add/remove/
 * commit updates the on-disk header via its OWN copy (g_hdr in usb.c) at any
 * point in that same loop. Writing a stale copy here would roll song_count /
 * next_free_block back and corrupt the catalog. Call only outside an upload
 * (usb_upload_active()) so this never lands mid-sequence. On an unformatted
 * disk, disk_read_header fails and this is a no-op every time it's tried --
 * cheap (one failed block read) and harmless. */
static void settings_flush(void)
{
	static disk_header_t s_save_hdr;
	if (!disk_read_header(&s_save_hdr)) return;
	s_save_hdr.settings_magic = DISK_SETTINGS_MAGIC;
	s_save_hdr.vol_level = (uint8_t)s_vol_level;
	if (disk_write_header(&s_save_hdr))
		g_settings_dirty = false;
}

/* Up/down integrator debounce for a ladder-window test. See the comment on
 * the btn_db block in main() for why this exists.
 *
 * The first version of this required 2 CONSECUTIVE IDENTICAL raw reads before
 * accepting any change -- a single flicker anywhere in the run (raw ladder
 * values sitting near a threshold WILL occasionally read one sample on the
 * wrong side even during a genuine, sustained press) reset the counter to
 * zero, and it could then need many samples before two happened to land back
 * to back. On real hardware this made most presses fail to register at all
 * ("only pauses about a quarter of the time") rather than merely allowing an
 * occasional double-fire, which was the ORIGINAL bug this was meant to fix.
 *
 * An up/down counter tolerates that: each read nudges the count by one step
 * toward its own reading rather than resetting on disagreement, so one stray
 * sample mid-press costs a single step of delay, not the whole count. A
 * lone, truly transient glitch (the case this exists to reject) still can't
 * reach the threshold on its own from a stable opposite baseline. */
static bool debounce(bool raw, struct btn_db *b)
{
	if (raw) { if (b->cnt < 2) b->cnt++; }
	else     { if (b->cnt > 0) b->cnt--; }
	if (b->cnt >= 2)      b->stable = true;
	else if (b->cnt == 0) b->stable = false;
	return b->stable;
}

static void enter_system_off(void)
{
	/* "LEDs go dark" is the documented signal that the device powered off and
	 * the bootloader is reachable — so every LED must actually be dark before
	 * SYSTEMOFF. A stopped PWM holds its outputs at the last driven level and
	 * keeps the pins via PSEL, so TASKS_STOP alone froze the track LEDs mid-VU
	 * (and PWM1/pb was never stopped at all). Stop both, hand the pins back to
	 * GPIO, and drive them low. */
	NRF_PWM0->TASKS_STOP = 1;
	NRF_PWM1->TASKS_STOP = 1;
	for (int i = 0; i < 4; i++) {
		NRF_PWM0->PSEL.OUT[i] = 0xFFFFFFFFu;
		NRF_PWM1->PSEL.OUT[i] = 0xFFFFFFFFu;
	}
	for (int i = 0; i < NUM_PB_LEDS; i++) {
		pb_leds[i].port->PIN_CNF[pb_leds[i].pin] = GPIO_OUT_CNF;
		pb_leds[i].port->OUTCLR = (1u << pb_leds[i].pin);
	}
	for (int i = 0; i < NUM_TRK_LEDS; i++) {
		track_leds[i].port->PIN_CNF[track_leds[i].pin] = GPIO_OUT_CNF;
		track_leds[i].port->OUTCLR = (1u << track_leds[i].pin);
	}

	/* Arm wake-on-press so the device can be brought back with no cable.
	 *
	 * Nothing armed a wake source before, so SYSTEM OFF could only be escaped
	 * by USB VBUS — "press any button" in the readme could never have worked.
	 * The function button (P0.27) is the only wake candidate: every other
	 * button sits on an analog resistor ladder into AIN0/AIN1 and cannot drive
	 * the GPIO DETECT signal.
	 *
	 * Order matters. If DETECT is already asserted when SYSTEMOFF is written,
	 * the chip resets instead of powering down — so wait for the button the
	 * user is still holding to come back up, settle the contact bounce, clear
	 * any latched event, and only then enable SENSE.
	 *
	 * Bounded (~3 s): if the contact ever reads stuck-low, fall through and
	 * power down anyway rather than spinning here forever with the LEDs
	 * already dark — worst case DETECT is live and the chip resets into the
	 * bootloader, which is still a usable outcome. */
	/* Power down the external chips FIRST, while the user is still holding the
	 * button — that hold doubles as the settle time for the amp to discharge.
	 * SYSTEM_OFF only stops the nRF; the amp, the headphone codec, the
	 * oscillator and the eMMC I/O rail are separate parts kept alive by the
	 * retained GPIO levels, so they would drain the battery for days, and a
	 * clocked-but-unmuted amp can murmur on its own. (Both helpers mute before
	 * cutting, so the drivers discharge quietly rather than stepping to ground
	 * — that step is the power-off pop.) */
	if (g_settings_dirty) settings_flush();

	codec_power_down();
	emmc_power_down();
	feed_wdt();

	/* k_msleep, not delay_ms: this must not busy-burn ~96k instructions per ms
	 * for up to three seconds. Sleeping yields to the other threads and lets
	 * the kernel idle. */
	for (int i = 0; i < 150 && !(NRF_P0->IN & (1u << 27)); i++) {
		k_msleep(20);
		feed_wdt();
	}
	k_msleep(60);   /* debounce the release */

	NRF_P0->LATCH = 0xFFFFFFFFu;

	NRF_P0->PIN_CNF[27] =
		(GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
		(GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos)  |
		(GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
		(GPIO_PIN_CNF_SENSE_Low     << GPIO_PIN_CNF_SENSE_Pos);
	__DSB();

	feed_wdt();
	NRF_POWER->RESETREAS = 0xFFFFFFFF;
	NRF_POWER->SYSTEMOFF  = 1;
	__DSB();
	for (;;);
}

/* Headphone level via CS42L42 mixer volume (MIXER_CHA/CHB_VOL). 0x00 = 0 dB
 * (full scale — too loud for headphones); attenuation grows with the code.
 * Level 0 = silence (handled by muting HP_CTL, since the mixer can't fully
 * mute); levels 1..7 cap well below full scale. Tune by ear. */
static const uint8_t vol_hp[8] = {0x00, 0x1D, 0x1A, 0x17, 0x14, 0x11, 0x0D, 0x0A};

/* Apply a 0..7 volume level to the headphone output: level 0 mutes via HP_CTL,
 * higher levels unmute and set the mixer attenuation. */
static void hp_apply_level(int lvl)
{
	if (lvl <= 0) {
		codec_headphone_mute(true);
	} else {
		codec_headphone_volume(vol_hp[lvl]);
		codec_headphone_mute(false);
	}
}

/* Decode the AIN0 track-button ladder into a bitmask of held stems (bit s = stem
 * s). Measured on-device: singles + all pairs are distinct, gaps ≥80 counts, so
 * a ±50 nearest-match is unambiguous. Triples/quad are unmeasured → return 0
 * (no solo) rather than a wrong guess. Returns 0 for idle / Play / unknown. */
static uint8_t track_mask(int al)
{
	static const struct { int16_t v; uint8_t m; } tbl[] = {
		{ 207, 0x1 }, { 401, 0x2 }, { 576, 0x3 }, { 725, 0x4 }, { 860, 0x5 },
		{ 981, 0x6 }, { 1211, 0x8 }, { 1307, 0x9 }, { 1390, 0xA }, { 1547, 0xC },
	};
	if (al < 120) return 0;
	for (unsigned i = 0; i < sizeof(tbl) / sizeof(tbl[0]); i++) {
		int d = al - tbl[i].v;
		if (d < 0) d = -d;
		if (d <= 50) return tbl[i].m;
	}
	return 0;
}

/* ── Stem faders ─────────────────────────────────────────────────────────────
 * The faders drive per-stem mix gain and must respond in real time. The audio
 * feed thread is cooperative and spends ~40% of wall-clock in long bit-bang
 * eMMC reads, during which the (lower-priority) main thread can't run — so
 * sampling the faders in main makes them freeze in ~68 ms chunks. Instead a
 * small thread, priority ABOVE the feed thread, samples the faders at ~125 Hz;
 * it preempts the feed even mid-read (a ~150 µs blip the deep TX queue absorbs)
 * so gain tracks the faders with no audible lag. */
#define FADER_MIN 120   /* raw ADC at bottom of travel → gain 0 */
#define FADER_MAX 3100  /* raw ADC at top of travel    → gain 256 (unity) */
#define VU_REF    171   /* baked level 0..255 that fills the 4-bar pb meter */
#define LOOP_HOLD_MS 300

/* Shared UI state: the buttons (main thread) write these, the UI thread reads
 * them to render the pb-LED meter. */
static volatile int s_meter_ticks = 0;   /* >0 = show the volume bar (UI counts down) */

/* Stem on/off (track-button tap) + momentary solo (track-button hold), folded
 * into the fader gains by the UI thread. s_solo_mask: 0 = no solo, else a bit
 * per stem that is currently soloed (only those play). Multiple stems can be
 * muted at once (independent taps). */
static volatile uint8_t s_stem_muted[4] = {0, 0, 0, 0};
static volatile uint8_t s_solo_mask = 0;

/* Set by ui_main() while function+track is held for the gate effect, so
 * main()'s power-off hold (which reads the SAME function button, entirely
 * independently) knows to ignore it instead of starting its own countdown
 * and LED-fill animation underneath the gate gesture. */
static volatile bool s_gate_gesture;

/* Once a function hold has been used for a gate gesture, that whole
 * continuous hold is disqualified from also becoming a power-off hold --
 * even after the track button releases and function is still down. Only
 * clears on a full physical release of function (in main(), below), so
 * letting go of just the track button never suddenly starts the shutdown
 * countdown/animation mid-hold. */
static volatile bool s_fn_hold_tainted;

/* Set by main() while it owns both LED rows for a gesture animation (the
 * power-off hold countdown, currently) so the UI thread's own ~125 Hz VU/meter
 * rendering doesn't fight it for the same PWM channels every 8 ms tick. */
static volatile bool s_gesture_active;

K_THREAD_STACK_DEFINE(s_ui_stack, 1024);
static struct k_thread s_ui_thread;

/* UI thread: samples the 4 faders and renders both LED visualizers at ~125 Hz.
 * Runs ABOVE the audio feed thread so it preempts the feed's long eMMC reads —
 * the faders and the meters stay real-time instead of freezing in ~68 ms chunks
 * (which is what happened when this ran in the read-starved main thread). Its
 * per-tick work is ~0.2 ms, a blip the deep TX queue absorbs.
 *
 * Play/pause, vol +/-, and the prev/next rocker are ALSO handled here now, not
 * in main()'s loop, for the exact same reason track buttons already were (see
 * the comment below on trk_held): main() runs at priority 10, BELOW the audio
 * feed thread (priority 5). During active playback the feed thread does an
 * UNINTERRUPTIBLE ~20-40 ms bit-banged eMMC burst read roughly every ~170 ms,
 * and since it is higher priority, main() cannot run AT ALL during that
 * window -- it is not slowed down, it is completely starved. A short press
 * that lands inside one of those windows is invisible to main() no matter how
 * it is debounced. This is exactly why track buttons were already polled here
 * instead of in main() -- but play/pause sits on the SAME AIN0 ladder as the
 * track buttons and was never moved, which is why "play is instant, pause
 * needs a deliberate hold" was reported: pausing only happens while ALREADY
 * playing, i.e. only while this starvation is actually happening; a plain
 * press from paused (feed thread doing near-nothing, no bursts) never hit it.
 * ui_main runs at priority 1, ABOVE the feed thread, so it is immune. */
static void ui_main(void *a, void *b, void *c)
{
	(void)a; (void)b; (void)c;
	/* saadc pselp per fader (= AIN+1): F1=AIN3=4 (stem0), F2=AIN6=7 (stem1),
	 * F3=AIN2=3 (stem2), F4=AIN7=8 (stem3). */
	static const uint8_t fader_ch[4] = {4u, 7u, 3u, 8u};
	bool    play_prev   = false;
	bool    volup_prev  = false;
	bool    voldn_prev  = false;
	bool    next_prev   = false;
	bool    prev_prev   = false;
	/* Hold-to-repeat timing for vol +/-, in wall-clock ms rather than tick
	 * counts (matching trk_press_ms's own pattern below) so it stays correct
	 * regardless of this thread's actual tick cadence -- tick-counted repeat
	 * rates silently drift if the loop's k_msleep ever changes. */
	int64_t volup_press_ms = 0, voldn_press_ms = 0;
	int64_t volup_repeat_ms = 0, voldn_repeat_ms = 0;
	struct btn_db db_play = {0}, db_volup = {0}, db_voldn = {0}, db_next = {0}, db_prev = {0};
	int64_t play_press_ms = 0;
	bool loop_hold_fired = false;
	bool play_was_playing = false;
	int vu_disp = 0;
	int trk_disp[4] = {0, 0, 0, 0};
	/* Smooth real-time playback position for the meters. The feed thread's
	 * s_cur_block freezes for ~68 ms during each eMMC read then jumps, which
	 * makes the meters stutter. ui_blk advances at the audio rate (375 blk/s →
	 * 3 blocks per 8 ms tick) and is gently eased toward the real position to
	 * stay aligned and to follow song changes / seeks. */
	int32_t ui_blk = 0;

	uint16_t gains[4] = {256, 256, 256, 256};
	int fi = 0;
	/* Track-button state (sensed here at 125 Hz so quick taps aren't dropped).
	 * Tap = toggle the held stem(s); hold >250 ms = solo them until release.
	 * Both work with multiple buttons at once (see track_mask). */
	bool    trk_held    = false;  /* any track button down */
	bool    trk_soloed  = false;  /* this hold has activated solo */
	uint8_t trk_last    = 0;      /* last non-zero held mask (for tap-toggle) */
	int64_t trk_press_ms = 0;
	/* Debounce the DECODED mask, not the raw ADC sample. The ladder read was
	 * fed straight into press/release/live-solo logic every ~8 ms tick with no
	 * filtering: a single noisy sample landing near a different table entry
	 * (finger pressure shifting the ladder resistance, ADC jitter) instantly
	 * changed which stem was soloed for that tick ("another stem cuts in"),
	 * and a one-tick glitch to 0 mid-hold looked like a release+re-press —
	 * toggling mute TWICE for one physical tap ("one click registers as two").
	 * Require 2 consecutive identical decodes (~16 ms) before accepting any
	 * change; a lone glitch never survives two ticks. */
	uint8_t trk_pending  = 0;
	uint8_t trk_pend_cnt = 0;
	uint8_t trk_stable   = 0;

	while (1) {
		/* Faders → per-stem mix gain. Sample ONE fader per tick (round-robin):
		 * each fader updates every 4 ticks ≈ 32 ms, still real-time. Hold the
		 * previous gain on a failed read (no stutter). With oversampling bypassed
		 * a read is ~20 µs, cheap enough to do at this high priority. */
		int raw = saadc_read(fader_ch[fi]);
		if (raw >= 0) {
			int t = (raw - FADER_MIN) * 256 / (FADER_MAX - FADER_MIN);
			if (t < 0)   t = 0;
			if (t > 256) t = 256;
			gains[fi] = (uint16_t)t;
		}
		fi = (fi + 1) & 3;

		/* Track buttons on AIN0 → bitmask of held stems. Tap toggles them; a
		 * hold >250 ms solos them, and the solo set follows the held buttons
		 * live (add/remove a finger to change it). */
		/* Same up/down-integrator fix as debounce() in main() (see its
		 * comment): a disagreeing sample DRAINS the count by one instead of
		 * resetting it to zero and restarting from the new value. Only once
		 * the count fully drains to 0 does the candidate actually switch --
		 * so one stray misread mid-hold costs a step, not the whole count. */
		int ain0 = saadc_read(1u);
		audio_dbg_set_ain0((uint16_t)ain0);   /* expose for threshold tuning via rome audio */
		uint8_t raw_mask = track_mask(ain0);
		if (raw_mask == trk_pending) {
			if (trk_pend_cnt < 3) trk_pend_cnt++;
		} else {
			if (trk_pend_cnt > 0) trk_pend_cnt--;
			if (trk_pend_cnt == 0) trk_pending = raw_mask;
		}
		if (trk_pend_cnt >= 3) trk_stable = trk_pending;
		uint8_t cur = trk_stable;
		/* Basic mode's only effect: hold function + hold one or more track
		 * buttons to gate those stems (see the TE guide's basic-mode page).
		 * Function-held takes over the track buttons entirely -- no solo/mute
		 * toggle for this gesture -- and each gate stops the instant either
		 * function or that stem's button is released. */
		bool fn_held_now = !(NRF_P0->IN & (1u << 27));   /* active-low */
		if (fn_held_now && cur) {
			audio_set_gate_mask(cur);
			s_gate_gesture = true;   /* tell main()'s power-off hold to sit this one out */
			s_fn_hold_tainted = true;   /* ...and keep sitting out until fn fully releases */
			trk_held = false;   /* don't let this also register as a solo/mute hold */
		} else {
			audio_set_gate_mask(0);
			s_gate_gesture = false;
			if (cur) {
				if (!trk_held) { trk_press_ms = k_uptime_get(); trk_soloed = false; trk_held = true; }
				trk_last = cur;
				if (!trk_soloed && k_uptime_get() - trk_press_ms > 250) trk_soloed = true;
				if (trk_soloed) s_solo_mask = cur;          /* live solo of the held set */
			} else if (trk_held) {                              /* all released */
				if (trk_soloed) {
					s_solo_mask = 0;
				} else {
					for (int s = 0; s < 4; s++)
						if (trk_last & (1u << s)) s_stem_muted[s] ^= 1u;
				}
				trk_held = false;
			}
		}

		/* Play/pause button on the SAME AIN0 ladder as the track buttons above
		 * -- reuse that read rather than sampling AIN0 twice. Window per the
		 * original main()-loop code: idle≈0, play≈1808, track1≈210. */
		bool play_now = debounce(ain0 >= 1650 && ain0 <= 1980, &db_play);
		if (play_now && !play_prev) {
			play_press_ms = k_uptime_get();
			play_was_playing = audio_is_playing();
			/* Arm the momentary loop at the press boundary. This avoids losing a
			 * hold while the audio feed is busy; a short press is resolved as the
			 * ordinary play/pause action when it is released below. */
			audio_play();
			audio_loop_start();
			loop_hold_fired = true;
		}
		if (!play_now && play_prev) {
			audio_loop_stop();
			if (k_uptime_get() - play_press_ms < LOOP_HOLD_MS) {
				/* The press temporarily starts playback so a hold can loop. For
				 * a short tap, resolve exactly one normal toggle from the state
				 * before the press—never play-then-toggle. */
				if (play_was_playing) audio_pause();
				else                 audio_play();
			}
		}
		play_prev = play_now;
		bool looping_now = play_now && loop_hold_fired;

		/* Ladder 2 (AIN1): prev≈399, vol-≈729, next≈1207, vol+≈1806. */
		int ain1 = saadc_read(2u);
		audio_dbg_set_ain1((uint16_t)ain1);   /* expose for threshold tuning via rome audio */
		bool volup_now = debounce(ain1 >= 1620 && ain1 <= 1960, &db_volup);
		bool next_now  = debounce(ain1 >= 1080 && ain1 <= 1340, &db_next);
		bool voldn_now = debounce(ain1 >=  620 && ain1 <=  860, &db_voldn);
		bool prev_now  = debounce(ain1 >=  300 && ain1 <=  520, &db_prev);

		/* Vol +/- with hold-to-repeat: step once on the press edge, then after a
		 * ~180 ms hold, auto-repeat every ~70 ms while held. Wall-clock based
		 * (k_uptime_get), not tick-counted, so it stays correct regardless of
		 * this thread's actual loop cadence -- see trk_press_ms above for the
		 * same pattern already used for the track-button solo hold. */
		int vol_step = 0;
		int64_t vol_now_ms = k_uptime_get();
		if (volup_now) {
			if (!volup_prev) {
				volup_press_ms = volup_repeat_ms = vol_now_ms;
				vol_step = 1;
			} else if (vol_now_ms - volup_press_ms > 180 && vol_now_ms - volup_repeat_ms > 70) {
				volup_repeat_ms = vol_now_ms;
				vol_step = 1;
			}
		}
		if (voldn_now) {
			if (!voldn_prev) {
				voldn_press_ms = voldn_repeat_ms = vol_now_ms;
				vol_step = -1;
			} else if (vol_now_ms - voldn_press_ms > 180 && vol_now_ms - voldn_repeat_ms > 70) {
				voldn_repeat_ms = vol_now_ms;
				vol_step = -1;
			}
		}
		if (looping_now) {
			if (vol_step != 0) audio_loop_change_divider(vol_step);
		} else if (vol_step > 0 && s_vol_level < 7) s_vol_level++;
		else if (vol_step < 0 && s_vol_level > 0) s_vol_level--;
		else vol_step = 0;
		if (vol_step != 0 && !looping_now) {
			if (s_hp_out) hp_apply_level(s_vol_level);
			else          codec_speaker_volume(vol_r46[s_vol_level]);
			s_meter_ticks = 120;   /* ~1 s of volume bar at this thread's 8 ms tick */
		}
		/* Persist volume on RELEASE, not per step: a held vol button auto-repeats
		 * every ~70 ms, and writing eMMC that often would wear the card for no
		 * benefit (only the final level after a gesture matters) and contend the
		 * bus with the feed thread during playback. One write per gesture. main()
		 * owns the actual disk write (see its comment); this just raises the flag. */
		if ((volup_prev && !volup_now) || (voldn_prev && !voldn_now))
			g_settings_dirty = true;

		/* Prev/next rocker: skip song + ensure playing. Wraparound to/from the
		 * ends is handled inside audio_skip -> change_song (modulo arithmetic in
		 * audio.c), not here. */
		if (looping_now) {
			if (next_now && !next_prev) audio_loop_move(1);
			if (prev_now && !prev_prev) audio_loop_move(-1);
		} else {
			if (next_now && !next_prev) { audio_skip(1);  audio_play(); }
			if (prev_now && !prev_prev) { audio_skip(-1); audio_play(); }
		}
		volup_prev = volup_now;
		voldn_prev = voldn_now;
		next_prev  = next_now;
		prev_prev  = prev_now;

		/* Fold in stem on/off + solo: while any stem is soloed, only soloed stems
		 * play; otherwise muted stems are silenced. Fader gain is the base. */
		uint16_t eff[4];
		uint8_t solo = s_solo_mask;
		for (int s = 0; s < 4; s++) {
			if (solo)  eff[s] = (solo & (1u << s)) ? gains[s] : 0;
			else       eff[s] = s_stem_muted[s] ? 0 : gains[s];
		}
		audio_set_stem_gains(eff);

		bool playing = audio_is_playing();

		/* Advance the smooth position at the audio rate, then ease toward the
		 * real (bursty) block to correct drift and follow song changes/seeks. */
		if (playing) ui_blk += 3;
		ui_blk += ((int32_t)audio_cur_block() - ui_blk) >> 4;
		if (ui_blk < 0) ui_blk = 0;
		uint32_t blk = (uint32_t)ui_blk;

		/* pb LEDs: volume bar for ~1 s after a vol button, else the live overall
		 * VU (stems × fader gains), else off. One-pole for a smooth envelope. */
		int fill;
		if (s_meter_ticks > 0) {
			s_meter_ticks--;
			fill = s_vol_level * NUM_PB_LEDS * PWM_TOP / 7;
		} else if (playing) {
			int target = (int)audio_vu_level_at(blk) * (NUM_PB_LEDS * PWM_TOP) / VU_REF;
			if (target > NUM_PB_LEDS * PWM_TOP) target = NUM_PB_LEDS * PWM_TOP;
			vu_disp += (target - vu_disp) / 2;
			fill = vu_disp;
		} else {
			vu_disp = 0;
			fill = 0;
		}
		if (!s_gesture_active && usb_upload_active()) {
			/* Upload progress: N of 4 pb LEDs BLINKING (not a solid fill, so it
			 * reads as distinct from the playback VU meter at a glance) where N
			 * is the quarter of the upload reached -- 1 LED in the first
			 * quarter, up to all 4 as it nears completion. Track LEDs go dark;
			 * they'd otherwise still show a stale VU from before the upload
			 * paused playback. */
			static uint32_t upload_blink;
			upload_blink++;
			bool on = ((upload_blink / 38) & 1u) == 0;   /* ~320 ms per half-cycle */
			uint32_t pm = usb_upload_progress_permille();
			int lit = 1 + (int)(pm * NUM_PB_LEDS / 1000u);
			if (lit > NUM_PB_LEDS) lit = NUM_PB_LEDS;
			for (int s = 0; s < NUM_PB_LEDS; s++)
				pwm1_set_duty(s, (s < lit && on) ? PWM_TOP : 0);
			for (int s = 0; s < 4; s++)
				pwm0_set_duty(s, 0);
		} else if (!s_gesture_active) {
			for (int s = 0; s < NUM_PB_LEDS; s++) {
				int b = fill - s * PWM_TOP;
				if (b < 0)       b = 0;
				if (b > PWM_TOP) b = PWM_TOP;
				pwm1_set_duty(NUM_PB_LEDS - 1 - s, (uint16_t)b);
			}

			if (audio_loop_active()) {
				/* While Play is held, the track row shows the selected divider. */
				int lit = 1 + (audio_loop_div_idx() * (NUM_TRK_LEDS - 1)) /
				              (audio_loop_div_count() - 1);
				for (int s = 0; s < NUM_TRK_LEDS; s++)
					pwm0_set_duty(s, s < lit ? PWM_TOP : 0);
			} else {
				/* Track LEDs: per-stem baked level (× fader gain), one-pole smoothed. */
				for (int s = 0; s < 4; s++) {
					int target = playing ? (int)audio_stem_level_at(blk, s) * PWM_TOP / 255 : 0;
					trk_disp[s] += (target - trk_disp[s]) / 2;
					pwm0_set_duty(s, (uint16_t)trk_disp[s]);
				}
			}
		}

		k_msleep(8);
	}
}

/* ── Main ──────────────────────────────────────────────────────────────────── */

int main(void)
{
	/* Drop the main thread BELOW the audio feed thread (preempt 5) so the feed
	 * is never starved by main's button/USB/jack work. Main only needs to run
	 * often enough for input + the watchdog; the feed yields to it whenever its
	 * TX alloc blocks. (Nothing else runs yet, so lowering now is safe.) */
	k_thread_priority_set(k_current_get(), 10);

	/* Captured BEFORE the clear below: true if this boot is a wake from
	 * SYSTEM_OFF via the function button's SENSE (see enter_system_off). A
	 * cold boot (fresh flash, battery just connected) never sets this, so the
	 * hold-to-power-on gate below only applies to the real "device was off,
	 * function was pressed" case -- it can't lock out a first-ever boot. */
	bool off_wake = (NRF_POWER->RESETREAS & POWER_RESETREAS_OFF_Msk) != 0;

	/* Clear RESETREAS on boot as well as before SYSTEMOFF: the bootloader reads
	 * it to decide how it was entered, and stale bits left by a watchdog reset
	 * or a previous power-off can send it down the wrong path. */
	NRF_POWER->RESETREAS = 0xFFFFFFFFUL;

	NRF_PPI->CHENCLR = 0xFFFFFFFFUL;

	/* P1.10 = PIN_BTN_COM: power rail for button ladders and faders */
	NRF_P1->PIN_CNF[10] = GPIO_OUT_CNF;
	NRF_P1->OUTSET      = (1u << 10);

	/* P0.27 = function button: active-low, pull-up */
	NRF_P0->PIN_CNF[27] =
		(GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
		(GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos)  |
		(GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);

	leds_init();
	saadc_init();
	pwm0_init();
	pwm1_init();   /* pb_leds via PWM1 (dimmable) — before any set_pb_on() */
	charger_init(); /* make sure the battery actually charges; also arms
	                  * usb_power_present()/battery_charging() for the
	                  * off_wake gate below. */

	if (off_wake) {
		/* Power-ON gate, symmetric with the power-OFF hold: require the
		 * function button that triggered this wake to still be held for a
		 * full 3 s before actually booting. Rejects a brief accidental touch
		 * on the SENSE-armed pin (a bump, a graze in a bag) without ever
		 * spinning up eMMC/codec/audio for it. All 8 LEDs fill progressively
		 * as feedback -- the power-on animation. codec_init()/emmc_init()
		 * have NOT run yet at this point (that's the whole reason this check
		 * comes first), so there is nothing to power back down if we abort;
		 * unlike enter_system_off(), no codec_power_down()/emmc_power_down()
		 * call belongs here. */
		bool held_full = true;
		int64_t hold_start = k_uptime_get();
		int64_t held_ms = 0;
		while (1) {
			if (NRF_P0->IN & (1u << 27)) { held_full = false; break; }  /* released early */
			held_ms = k_uptime_get() - hold_start;
			if (held_ms >= 3000) break;
			/* Paced by k_uptime_get() (RTC-backed), not delay_ms() (an
			 * imprecise NOP busy-wait -- see util.h) -- this is what keeps
			 * this fill's real duration matching enter_system_off()'s
			 * identical fill, which is timed the same way. Using delay_ms()
			 * here made the two animations drift apart because a NOP count
			 * calibrated for 64 MHz doesn't land on the same wall-clock time
			 * this early in boot as it does deep into runtime. */
			int filled = (int)(held_ms * (NUM_PB_LEDS + NUM_TRK_LEDS) / 3000);
			for (int i = 0; i < NUM_PB_LEDS; i++)
				pwm1_set_duty(i, (i < filled) ? PWM_TOP : 0);
			for (int i = 0; i < NUM_TRK_LEDS; i++)
				pwm0_set_duty(i, (NUM_PB_LEDS + i < filled) ? PWM_TOP : 0);
			k_msleep(20);
			feed_wdt();
		}
		if (!held_full) {
			/* Too short to count: clear the LEDs, debounce the release the
			 * same way enter_system_off() does, then go straight back into
			 * SYSTEM_OFF with SENSE re-armed. No boot happened. */
			for (int i = 0; i < NUM_PB_LEDS; i++)  pwm1_set_duty(i, 0);
			for (int i = 0; i < NUM_TRK_LEDS; i++) pwm0_set_duty(i, 0);

			/* A brief tap (< 600 ms) while plugged into power reads as
			 * "show me the battery", not a failed boot attempt: show the
			 * charge level on the track LEDs (best guess at "right hand
			 * side" -- trivial one-line swap to pb_leds/pwm1 if that's
			 * actually the left side on the real unit) for ~3 s, then fall
			 * through to the same re-arm-and-sleep below. A longer partial
			 * hold (600 ms-3 s) stays a plain aborted boot with no gauge,
			 * so a half-hearted boot attempt isn't misread as a battery
			 * check. */
			if (held_ms < 600 && usb_power_present()) {
				int64_t peek_start = k_uptime_get();
				while (k_uptime_get() - peek_start < 3000) {
					int q = battery_quarters();
					bool chg = battery_charging();
					bool blink = ((k_uptime_get() / 250) & 1) == 0;
					for (int i = 0; i < NUM_TRK_LEDS; i++) {
						bool on;
						if (q <= 0)           on = false;
						else if (i < q - 1)   on = true;
						else if (i == q - 1)  on = chg ? blink : true;
						else                  on = false;
						pwm0_set_duty(i, on ? PWM_TOP : 0);
					}
					k_msleep(40);
					feed_wdt();
				}
				for (int i = 0; i < NUM_TRK_LEDS; i++) pwm0_set_duty(i, 0);
			}

			for (int i = 0; i < 150 && !(NRF_P0->IN & (1u << 27)); i++) {
				k_msleep(20);
				feed_wdt();
			}
			k_msleep(60);
			NRF_P0->LATCH = 0xFFFFFFFFu;
			NRF_P0->PIN_CNF[27] =
				(GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
				(GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos)  |
				(GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
				(GPIO_PIN_CNF_SENSE_Low     << GPIO_PIN_CNF_SENSE_Pos);
			__DSB();
			feed_wdt();
			NRF_POWER->RESETREAS = 0xFFFFFFFFUL;
			NRF_POWER->SYSTEMOFF = 1;
			__DSB();
			for (;;);
		}
	}

	usb_cdc_init();

	/* Boot animation: a light bounces back and forth across the 4 playback
	 * LEDs, fading each neighbor for depth. Confirms the device is alive
	 * before eMMC/codec bring-up even starts — useful on its own, since a
	 * healthy idle device is otherwise dark by design and easy to mistake for
	 * dead. The README has claimed this as a shipped feature; nothing ever
	 * implemented it. 2 round trips at 30 ms/step (~360 ms total) -- long
	 * enough to read as a deliberate animation, short enough not to feel like
	 * a delay before the device is usable. Feeds the watchdog throughout.
	 *
	 * Skipped on an off_wake boot: the power-on hold gate above already ran a
	 * full 3 s "device is alive" fill animation, ending with all 8 LEDs lit.
	 * Playing this ~360 ms extra bounce on TOP of that made a power-on-from-
	 * off take longer overall than a power-off (3.36 s vs 3.0 s) even though
	 * the two hold-fill animations are the same 3000 ms -- the bounce was the
	 * whole difference. A true cold boot (fresh flash, battery just
	 * connected) has no gate before it and still gets this bounce, since
	 * there's nothing else to signal "alive" in that case. */
	if (!off_wake) {
		for (int step = 0; step < 12; step++) {
			int pos = step % 6;
			int lit = (pos <= 3) ? pos : 6 - pos;   /* 0,1,2,3,2,1 -> bounce */
			for (int i = 0; i < NUM_PB_LEDS; i++) {
				int dist = i - lit; if (dist < 0) dist = -dist;
				uint16_t duty = (dist == 0) ? PWM_TOP : (dist == 1) ? PWM_TOP / 6 : 0;
				pwm1_set_duty(i, duty);
			}
			delay_ms(30);
			feed_wdt();
		}
	}
	all_pb_off();
	/* all_trk_off() drives the track pins via raw GPIO, but the power-on hold
	 * gate (and, on a cold boot, nothing here at all) leaves them under PWM0's
	 * control -- a running PWM sequence overrides a plain GPIO write on the
	 * same pin (the same class of bug fixed earlier in the fatal handler).
	 * Only relevant after off_wake's 8-LED fill; harmless (already zero) on a
	 * cold boot. */
	for (int i = 0; i < NUM_TRK_LEDS; i++) pwm0_set_duty(i, 0);

	/* eMMC init. On failure: pb_led[1] blinks N times = which init step failed
	 * (1-9). On SUCCESS nothing lights — an idle, not-playing device is dark by
	 * design, so "no LEDs" means healthy, not dead. (This used to claim
	 * "pb_led[0] solid = success"; no code ever did that.) */
	if (!emmc_init()) {
		uint8_t step = emmc_fail_step();
		for (;;) {
			for (uint8_t i = 0; i < step; i++) {
				set_pb_on(1);
				delay_ms(200);
				all_pb_off();
				delay_ms(200);
			}
			delay_ms(1000);
			feed_wdt();
		}
	}

	feed_wdt();

	/* Audio codec bring-up. Register state available over USB via CODEC_DIAG. */
	bool codec_ok = codec_init();
	feed_wdt();

	/* Scan disk for the first valid song before starting I2S (no concurrent DMA). */
	static disk_header_t s_dh;
	static disk_song_entry_t s_se;
	uint32_t song_block_start = 0, song_block_count = 0;
	uint16_t first_song_idx = 0, total_songs = 0;
	bool song_found = false;
	bool hdr_valid = disk_read_header(&s_dh);
	/* Restore saved master volume. settings_magic guards against reading a
	 * vol_level of 0 from a header written before this field existed (that
	 * padding was always zeroed by disk_format, so an old header reads
	 * magic=0 here, not DISK_SETTINGS_MAGIC) -- fall back to the existing
	 * night-friendly default (s_vol_level's initializer, 3) in that case. */
	if (hdr_valid && s_dh.settings_magic == DISK_SETTINGS_MAGIC && s_dh.vol_level <= 7)
		s_vol_level = s_dh.vol_level;
	if (hdr_valid && s_dh.song_count > 0) {
		total_songs = s_dh.song_count;
		for (uint16_t i = 0; i < s_dh.song_count; i++) {
			/* Skip zero-length entries too (an upload that never committed):
			 * loading one starts the feed on the 440 Hz error tone. */
			if (disk_read_song(i, &s_se) && s_se.name[0] != '\0' && s_se.block_count > 0) {
				song_found       = true;
				first_song_idx   = i;
				song_block_start = s_se.block_start;
				song_block_count = s_se.block_count;
				break;
			}
		}
	}
	feed_wdt();

	/* I2S bring-up + ADPCM playback. Starts paused; play button toggles. */
	if (codec_ok && audio_init()) {
		if (song_found) {
			audio_set_source(AUDIO_SRC_ADPCM);
			audio_set_playlist(total_songs, first_song_idx);
			audio_set_levels_enabled(s_dh.version >= 2);  /* baked VU on v2 discs */
			audio_load_song(song_block_start, song_block_count);
		}
	}
	feed_wdt();

	/* Play/pause, vol +/-, and the prev/next rocker are polled in ui_main(),
	 * not here -- see the big comment there for why. Jack-sense stays in this
	 * loop (I2C is inherently slow and the switch itself is already debounced
	 * over multiple seconds of contact bounce, so starvation from the feed
	 * thread is not a practical problem for it the way it was for buttons). */

	/* Output routing: speaker (TAS2505) or headphones (CS42L42), chosen by the
	 * CS42L42 jack-sense. Exactly one is unmuted. Debounce jack reads so a noisy
	 * insert doesn't flap. Detect the initial state before the loop so booting
	 * with headphones in starts on the right output. */
	s_hp_out = codec_headphones_present();
	int  hp_raw_prev = s_hp_out ? 1 : 0;   /* last raw jack read */
	int  hp_debounce = 0;                /* consecutive stable raw reads */
	if (s_hp_out) {
		codec_speaker_mute(true);
		hp_apply_level(s_vol_level);
	} else {
		codec_speaker_volume(vol_r46[s_vol_level]);
	}

	/* Spawn the UI thread (faders + LED visualizers) at a priority ABOVE the
	 * audio feed thread (K_PRIO_COOP(NUM_COOP-1)) so it preempts the feed's long
	 * eMMC reads and keeps the faders and meters real-time. */
	k_thread_create(&s_ui_thread, s_ui_stack, K_THREAD_STACK_SIZEOF(s_ui_stack),
			ui_main, NULL, NULL, NULL,
			K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
	k_thread_name_set(&s_ui_thread, "ui");

	int64_t off_hold_start = 0;   /* 0 = not currently holding function */
	bool    fn_raw_prev = false;  /* physical P0.27 state, for detecting a full release */

	while (1) {
		bool uploading = usb_upload_active();

		/* Clear the gate-hold taint only on a genuine full release of the
		 * function button -- letting go of just the track button (still
		 * holding function) must NOT re-arm the power-off hold mid-gesture. */
		bool fn_raw_now = !(NRF_P0->IN & (1u << 27));
		if (!fn_raw_now && fn_raw_prev) s_fn_hold_tainted = false;
		fn_raw_prev = fn_raw_now;

		/* Power off: a host command (rome bootloader's POWER_OFF) fires
		 * immediately -- it isn't a physical hold, there's nothing to debounce.
		 * The physical button requires a full 3 s continuous hold, symmetric
		 * with the power-on gate in main()'s startup: a bare tap used to power
		 * the device off instantly, which is exactly the kind of accidental
		 * trigger (bumping it in a pocket, brushing it while reaching for
		 * play) the hold exists to prevent. All 8 LEDs fill progressively over
		 * the hold as feedback -- the power-off animation -- and releasing
		 * early aborts with no effect. Skipped entirely mid-upload either way
		 * (see the uploading guard below). */
		if (!uploading && usb_power_off_requested())
			enter_system_off();

		bool fn_down = fn_raw_now && !s_gate_gesture && !s_fn_hold_tainted;
		if (!uploading && fn_down) {
			if (off_hold_start == 0) {
				off_hold_start = k_uptime_get();
				s_gesture_active = true;   /* claim both LED rows from the UI thread */
			}
			int64_t held_ms = k_uptime_get() - off_hold_start;
			int filled = (int)(held_ms * (NUM_PB_LEDS + NUM_TRK_LEDS) / 3000);
			if (filled > NUM_PB_LEDS + NUM_TRK_LEDS) filled = NUM_PB_LEDS + NUM_TRK_LEDS;
			for (int i = 0; i < NUM_PB_LEDS; i++)
				pwm1_set_duty(i, (i < filled) ? PWM_TOP : 0);
			for (int i = 0; i < NUM_TRK_LEDS; i++)
				pwm0_set_duty(i, (NUM_PB_LEDS + i < filled) ? PWM_TOP : 0);
			if (held_ms >= 3000)
				enter_system_off();   /* does not return */
		} else if (off_hold_start != 0) {
			/* Released before 3 s (or an upload started mid-hold): abort.
			 * Hand the LEDs back to the UI thread, which repaints its own
			 * state on its very next ~8 ms tick. */
			off_hold_start = 0;
			s_gesture_active = false;
		}

		/* Play/pause, vol +/-, and the rocker used to be polled here; moved to
		 * ui_main() (see its comment). settings_flush() stays here -- it does
		 * eMMC I/O and must never run from the UI thread (the highest-priority
		 * thread besides none; a blocking eMMC op there would starve the audio
		 * feed thread itself, a worse version of the exact problem this whole
		 * change fixes). ui_main sets g_settings_dirty on a vol release; this
		 * just watches for it. */
		if (g_settings_dirty && !uploading && !audio_is_playing())
			settings_flush();

		/* (Faders, track buttons, and both LED visualizers are handled by the
		 * UI thread so they stay real-time during the feed thread's long eMMC
		 * reads.) */

		/* Headphone jack sense (every loop ≈ every 18 ms; the I2C read is cheap).
		 * Debounce: 2 stable raw reads before flipping output, so a noisy insert
		 * can't flap but the switch still feels immediate. On a switch, mute the
		 * old output and unmute the new at the current level so loudness stays
		 * roughly consistent across the change. */
		{
			int hp_raw = codec_headphones_present() ? 1 : 0;
			if (hp_raw == hp_raw_prev) {
				if (hp_debounce < 2) hp_debounce++;
			} else {
				hp_debounce = 0;
				hp_raw_prev = hp_raw;
			}
			if (hp_debounce >= 2 && hp_raw != (s_hp_out ? 1 : 0)) {
				s_hp_out = hp_raw;
				if (s_hp_out) {
					codec_speaker_mute(true);
					hp_apply_level(s_vol_level);
				} else {
					codec_headphone_mute(true);
					codec_speaker_volume(vol_r46[s_vol_level]);
					codec_speaker_mute(false);
				}
			}
		}

		/* (The pb + track LED visualizers are rendered by the UI thread.) */
		bool playing = audio_is_playing();

		/* Renode mirror: expose GPIO state for emulator observation */
		*(volatile uint32_t *)0x2000FFF0 = NRF_P0->OUT;
		*(volatile uint32_t *)0x2000FFF4 = NRF_P1->OUT;

		usb_cdc_poll();

		feed_wdt();
		if (!uploading)
			k_msleep(playing ? 18 : 30);   /* faster while playing → smoother VU */
		/* Power-off is handled once, at the TOP of this loop, via the 3 s hold
		 * gate + animation. A second, unconditional "!(NRF_P0->IN & bit27) ->
		 * enter_system_off()" check used to live here too (predating the hold
		 * gate) -- it fired on the very first loop tick the button read low,
		 * bypassing the hold entirely and powering off almost instantly, which
		 * is why the animation never had time to show. Removed. */
	}

	return 0;
}
