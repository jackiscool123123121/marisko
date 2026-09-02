#include "audio.h"
#include "codec.h"
#include "emmc.h"
#include "disk.h"
#include "leds.h"
#include "pwm.h"
#include "util.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>

/* 24-bit SWIDTH, stereo, 48 kHz. Per the nRF52840 PS (I2S → EasyDMA), a memory
 * word at SWIDTH=24Bit holds "one right-aligned 24-bit sample sign extended to
 * 32 bit" — i.e. sample in bits[23:0], NOT bits[31:8]. CONFIG.ALIGN (which
 * Zephyr sets to Left for I2S format) only affects framing on the wire, not the
 * RAM layout. So a 16-bit sample is scaled to full 24-bit scale with << 8.
 * Do not "fix" that shift to << 16 — it would clip everything by 8 bits. */
#define FRAMES_PER_BLOCK 128u
#define CHANNELS         2u
#define BYTES_PER_WORD   4u
#define BLOCK_BYTES      (FRAMES_PER_BLOCK * CHANNELS * BYTES_PER_WORD)
#define NUM_BLOCKS       48u   /* ~128 ms TX runway: absorbs a 23 ms read burst + heavy
                              * main-thread preemption without underrunning the I2S DMA */
#define I2S_TIMEOUT_MS   200u

K_MEM_SLAB_DEFINE_STATIC(s_tx_slab, BLOCK_BYTES, NUM_BLOCKS, 4);

#define FEED_STACK 2048
K_THREAD_STACK_DEFINE(s_feed_stack, FEED_STACK);
static struct k_thread s_feed_thread;

static const struct device *s_i2s;
static volatile audio_source_t s_source = AUDIO_SRC_SILENCE;
static volatile bool s_running;

/* ── 440 Hz sine via a 2nd-order resonator (no libm) ─────────────────────── */
/* y[n] = K·y[n-1] − y[n-2] with y[-1]=0, y[0]=A·sin(w) gives y[n] = A·sin(w(n+1)),
 * so TONE_A is the PEAK, not the seed. The old 4.0e7 was 4.8× past 24-bit full
 * scale (±8388607) and wrapped into a buzz — the "y1_init ≈ 2.3e6 fits" note
 * only checked the first sample. 2.0e6 ≈ −12 dBFS: clearly audible as a fault
 * indicator without being painful. */
#define TONE_A   2.0e6f
#define TONE_K   1.9966822f   /* 2·cos(2π·440/48000) */
#define TONE_SW  0.0575539f   /*   sin(2π·440/48000) */
static float s_y1, s_y2;

static void tone_reset(void)
{
	s_y2 = 0.0f;
	s_y1 = TONE_A * TONE_SW;
}

/* ── IMA-ADPCM decoder ─────────────────────────────────────────────────────
 * Channels 0 (L) and 1 (R) of stem 0. State carries across blocks.
 * Disk layout: ch0 = bytes [0..63], ch1 = bytes [64..127], low nibble first.
 */
static const int16_t s_step_table[89] = {
	7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28,
	31, 34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
	130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408,
	449, 494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282,
	1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660,
	4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442,
	11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623,
	27086, 29794, 32767
};
static const int8_t s_index_table[16] = {
	-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8
};

static int16_t  s_pred[8];
static int8_t   s_sidx[8];
static uint32_t s_song_block_start;
static uint32_t s_song_block_count;
static uint32_t s_cur_block;
static volatile bool s_playing;

/* Basic-mode loop state.  Looping is momentary: the UI owns the hold gesture
 * and clears this flag when Play is released. */
#define LOOP_DIV_COUNT 5u
static const uint8_t s_loop_divisors[LOOP_DIV_COUNT] = { 1u, 2u, 4u, 8u, 16u };
/* 128 frames/block at 48 kHz = 375 blocks/second.  A loop must be short
 * enough to repeat while Play is being held; deriving its length from the
 * entire song made a 1/4 setting take tens of seconds on a long stem. */
#define LOOP_BASE_BLOCKS 375u
static volatile bool s_loop_active;
static volatile bool s_loop_start_req;
static volatile uint8_t s_loop_div_idx = 2u;
static uint32_t s_loop_start, s_loop_end;
static int16_t s_loop_pred[8];
static int8_t s_loop_sidx[8];

/* In-RAM loop cache.  Looping by re-reading the loop region off eMMC on every
 * pass would stall the feed thread at the seam (a blocking CMD18 refill the
 * moment the wrap needs a block no longer in the 64-block prefetch window),
 * which is audible as a ~0.5 s gap.  Instead, load the whole loop region into
 * RAM once at loop start and serve every wrap from it: the seam becomes a pure
 * pointer reset, so it is seamless.  Loops larger than the cache fall back to
 * the prefetch path (still works, just seeks at the seam).
 *
 * Prefetch, the baked level array and the loop cache share one RAM pool.
 * Prefetch + levels are never both live while looping (the track LEDs show the
 * loop divider instead of stem levels), so the loop cache reuses those bytes —
 * net-zero RAM (pf 32 KB + lvl 24 KB = 56 KB) yet covers the default ~93-block
 * loop entirely in RAM. */
#define PREFETCH_BLOCKS 64u        /* see prefetch comment below */
#define LOOP_CAP_BLOCKS 96u        /* ≤ (PREFETCH_BLOCKS*512 + LVL_MAX_BYTES)/512 */
#define LVL_MAX_BYTES   24576u     /* baked-level RAM cap (see levels comment) */
union audio_buf {
	struct {
		uint8_t pf[PREFETCH_BLOCKS * 512u];
		uint8_t lvl[LVL_MAX_BYTES];
	} normal;
	uint8_t loop[LOOP_CAP_BLOCKS * 512u];
};
static union audio_buf s_audio_buf;
#define s_pf_buf  s_audio_buf.normal.pf
#define s_lvl_ram s_audio_buf.normal.lvl
/* Loop cache view is s_audio_buf.loop (loop_preload / fill_block). */

static bool     s_loop_cache_ok;  /* loop_len fits in cache and it is loaded */
static uint32_t s_loop_len;       /* blocks in the loop */
static uint32_t s_loop_pos;       /* next cache index to serve (0..loop_len-1) */
static volatile bool s_loop_pf_dirty;  /* loop stopped: reset prefetch on feed */

static void loop_set_end(void)
{
	uint32_t len = LOOP_BASE_BLOCKS / s_loop_divisors[s_loop_div_idx];
	if (len == 0) len = 1;
	if (len > s_song_block_count) len = s_song_block_count;
	s_loop_end = s_loop_start + len;
	if (s_loop_end > s_song_block_count) s_loop_end = s_song_block_count;
}

/* Load the whole loop region into s_audio_buf.loop so every wrap can be served
 * from RAM with no eMMC access.  Runs on the feed thread once, at loop start. */
static void loop_preload(void)
{
	s_loop_len = s_loop_end - s_loop_start;
	s_loop_cache_ok = false;
	if (s_loop_len == 0 || s_loop_len > LOOP_CAP_BLOCKS) return;
	if (emmc_read_blocks(s_song_block_start + s_loop_start, s_audio_buf.loop, s_loop_len)) {
		s_loop_cache_ok = true;
		s_loop_pos = 0;
	}
}

/* Playlist: feed thread advances to the next catalog song at end-of-song,
 * wrapping after the last. disk_read_song runs in the feed thread (same thread
 * that owns the eMMC during playback) — no cross-thread bus race. */
static uint16_t s_song_count;     /* total catalog entries */
static uint16_t s_cur_song;       /* current catalog index */
static volatile int s_skip_req;   /* +1 = next, -1 = prev (rocker); 0 = none */

static void lvl_setup(uint32_t block_start, uint32_t block_count);

/* Move to the next valid catalog song in direction dir (+1 next, -1 prev),
 * wrapping. Runs in the feed thread (disk_read_song = eMMC). */
static void change_song(int dir)
{
	if (s_song_count == 0) return;
	disk_song_entry_t e;
	int n = (int)s_song_count;
	for (int step = 1; step <= n; step++) {
		int idx = ((int)s_cur_song + dir * step) % n;
		if (idx < 0) idx += n;
		/* block_count == 0 must be skipped like a free slot: adopting it makes
		 * the refill below ask for 0 blocks, emmc_read_blocks fails, and the
		 * feed falls through to the 440 Hz error tone — forever, since the
		 * position never advances past the end. */
		if (disk_read_song((uint16_t)idx, &e) && e.name[0] != '\0' && e.block_count > 0) {
			s_cur_song          = (uint16_t)idx;
			s_song_block_start  = e.block_start;
			s_song_block_count  = e.block_count;
			lvl_setup(e.block_start, e.block_count);
			return;
		}
	}
	/* No other valid song — current one just loops (start/count unchanged). */
}

/* Prefetch: one slow bit-bang CMD17 per 128-frame buffer (~2-3 ms) can't keep
 * up with the 2.67 ms playback budget → underrun buzz. Read PREFETCH_BLOCKS at
 * once via CMD18 streaming (amortizes command overhead) and decode from RAM. */
/* (PREFETCH_BLOCKS is defined above with the loop cache; it shares s_pf_buf.) */
static uint32_t s_pf_pos;    /* next block index within s_pf_buf to decode */
static uint32_t s_pf_count;  /* valid blocks currently in s_pf_buf */
static volatile uint32_t s_last_read_us;  /* diag: µs per block of last refill */
static volatile int32_t  s_peak;          /* peak-hold since last read (v1 LED VU fallback) */

/* ── Feed-thread health counters (read via USB AUDIO_DIAG) ─────────────────── */
static volatile uint32_t s_recoveries;    /* TX underrun recoveries (alloc timeout → DROP/START) */
static volatile uint32_t s_write_fails;   /* i2s_write() failures */
static volatile uint32_t s_max_read_us;   /* worst per-block eMMC read time seen (budget 2670) */
static volatile uint32_t s_blocks_fed;    /* total blocks handed to i2s_write */

/* Per-stem mix gain (0..256, 256 = unity), driven live by the 4 faders from the
 * main thread. Default unity so playback is full until faders are read. */
static volatile uint16_t s_stem_gain[4] = {256u, 256u, 256u, 256u};
void audio_set_stem_gains(const uint16_t g[4])
{
	for (int i = 0; i < 4; i++) s_stem_gain[i] = g[i] > 256u ? 256u : g[i];
}

/* ── Baked per-stem VU levels (disk v3) ─────────────────────────────────────
 * rome bakes 4 level bytes per LVL_DECIM audio blocks — one peak per stem
 * (each >> 7, 0..255) — stored interleaved [s0,s1,s2,s3, …]. The whole array is
 * loaded into RAM once at song start (no recurring eMMC during playback). The
 * track LEDs dim to each stem's level; the overall pb-LED meter sums the four
 * scaled by the live fader gains. Index = (cur_block / LVL_DECIM) * 4 + stem;
 * on a feed stall cur_block freezes so the meter holds rather than dropping. */
#define LVL_DECIM     16u       /* audio blocks per baked level sample — must match rome */
#define LVL_STEMS     4u        /* bytes per sample (one peak per stem) */
/* (LVL_MAX_BYTES is defined above with the loop cache; s_lvl_ram shares it.) */

static bool              s_lvl_enabled;   /* disk reports v2 → levels present */
static bool              s_lvl_present;   /* current song has a level region */
static bool              s_lvl_loaded;    /* level array read into RAM for this song */
static uint32_t          s_lvl_start;     /* first level block of current song */
static uint32_t          s_lvl_blocks;    /* level blocks to read (capped) */
static uint32_t          s_lvl_count;     /* valid level bytes in s_lvl_ram */

/* Recompute the level region for a song from its (audio) block_start/count. */
static void lvl_setup(uint32_t block_start, uint32_t block_count)
{
	s_lvl_present = s_lvl_enabled && block_count > 0;
	s_lvl_start   = block_start + block_count;
	uint32_t bytes = ((block_count + LVL_DECIM - 1u) / LVL_DECIM) * LVL_STEMS;
	if (bytes > LVL_MAX_BYTES) bytes = LVL_MAX_BYTES;
	s_lvl_count   = bytes;
	s_lvl_blocks  = (bytes + 511u) / 512u;
	s_lvl_loaded  = false;
}

static int16_t adpcm_step(uint8_t nibble, int ch)
{
	int step = s_step_table[(uint8_t)s_sidx[ch]];
	int diff = step >> 3;
	if (nibble & 4u) diff += step;
	if (nibble & 2u) diff += step >> 1;
	if (nibble & 1u) diff += step >> 2;
	if (nibble & 8u) diff = -diff;
	int pred = s_pred[ch] + diff;
	if (pred >  32767) pred =  32767;
	if (pred < -32768) pred = -32768;
	s_pred[ch] = (int16_t)pred;
	int idx = s_sidx[ch] + s_index_table[nibble & 0xFu];
	if (idx < 0)  idx = 0;
	if (idx > 88) idx = 88;
	s_sidx[ch] = (int8_t)idx;
	return s_pred[ch];
}

/* ── Buffer fill ────────────────────────────────────────────────────────────── */

/* Mix + ADPCM-decode one 512-byte block (4 stereo stems) into 128 stereo frames.
 * Shared by the prefetch path and the in-RAM loop cache so both stay identical. */
static void decode_block(const uint8_t *blk, int32_t *buf)
{
	int32_t bufpeak = 0;
	for (uint32_t i = 0; i < FRAMES_PER_BLOCK; i++) {
		int32_t l = 0, r = 0;
		/* Mix all 4 stems: each stem = 128 bytes (64 L + 64 R) */
		for (int stem = 0; stem < 4; stem++) {
			uint32_t ol = (uint32_t)stem * 128u + i / 2u;
			uint32_t or_ = (uint32_t)stem * 128u + 64u + i / 2u;
			uint8_t bl  = blk[ol];
			uint8_t br  = blk[or_];
			uint8_t nl  = (i & 1u) ? (bl >> 4) : (bl & 0xFu);
			uint8_t nr  = (i & 1u) ? (br >> 4) : (br & 0xFu);
			/* Decode every stem (advances its ADPCM predictor even when
			 * silenced, so a muted fader never desyncs the decoder), then
			 * scale by the live fader gain (0..256, 256 = unity). */
			int32_t g = s_stem_gain[stem];
			l += (adpcm_step(nl, stem * 2)     * g) >> 8;
			r += (adpcm_step(nr, stem * 2 + 1) * g) >> 8;
		}
		/* Average 4 stems (>>2): sum of four ±32767 values is ±131068,
		 * so >>2 → ±32767 max and NEVER clips. The old >>1 clipped hard
		 * whenever 2+ stems were loud → constant crunch/clicks. Make up
		 * the 6 dB with the speaker volume. Clamp kept as a safety net. */
		l >>= 2;
		r >>= 2;
		if (l >  32767) l =  32767;
		if (l < -32768) l = -32768;
		if (r >  32767) r =  32767;
		if (r < -32768) r = -32768;
		/* Track buffer peak for the LED visualizer. */
		int32_t al = l < 0 ? -l : l;
		int32_t ar = r < 0 ? -r : r;
		if (al > bufpeak) bufpeak = al;
		if (ar > bufpeak) bufpeak = ar;
		/* 16-bit sample → 24-bit right-aligned (bits[23:8]) */
		buf[2 * i]     = (int32_t)(l << 8);
		buf[2 * i + 1] = (int32_t)(r << 8);
	}
	/* Peak-hold for the LED VU. Main reads + resets this every loop and
	 * applies its own time-based decay — so the meter stays smooth even
	 * while the feed thread is busy in a multi-block eMMC read. */
	if (bufpeak > s_peak) s_peak = bufpeak;
}

static void fill_block(int32_t *buf)
{
	if (s_source == AUDIO_SRC_TONE) {
		for (uint32_t i = 0; i < FRAMES_PER_BLOCK; i++) {
			float y = TONE_K * s_y1 - s_y2;
			s_y2 = s_y1;
			s_y1 = y;
			int32_t s = (int32_t)y;
			buf[2 * i]     = s;
			buf[2 * i + 1] = s;
		}
		return;
	}

	if (s_source == AUDIO_SRC_ADPCM && s_playing && s_song_block_count > 0
	    && !emmc_write_multi_active()) {
		/* Consume loop start on the feed thread so the saved ADPCM state and
		 * block position are an atomic, decoder-owned boundary. */
		if (s_loop_start_req) {
			s_loop_start_req = false;
			s_loop_start = s_cur_block < s_song_block_count ? s_cur_block : 0;
			for (int i = 0; i < 8; i++) {
				s_loop_pred[i] = s_pred[i];
				s_loop_sidx[i] = s_sidx[i];
			}
			loop_set_end();
			s_loop_active = s_loop_end > s_loop_start;
			loop_preload();
			s_loop_pf_dirty = false;
		}

		/* Loop stopped (Play released / song changed): drop any stale prefetch
		 * so the normal path re-reads from the current position, and drop the
		 * cache we no longer need. */
		if (s_loop_pf_dirty) {
			s_loop_pf_dirty = false;
			s_loop_cache_ok = false;
			s_pf_pos = s_pf_count = 0;
		}

		/* Seamless in-RAM loop: every pass is served from s_loop_buf, never
		 * touching eMMC, so the wrap is an instant pointer reset (no stall). */
		if (s_loop_active && s_loop_cache_ok) {
			decode_block(s_audio_buf.loop + s_loop_pos * 512u, buf);
			if (++s_loop_pos >= s_loop_len) {
				s_loop_pos = 0;
				s_cur_block = s_loop_start;
				for (int i = 0; i < 8; i++) {
					s_pred[i] = s_loop_pred[i];
					s_sidx[i] = s_loop_sidx[i];
				}
			} else {
				s_cur_block++;
			}
			return;
		}

		if (s_loop_active && s_cur_block >= s_loop_end) {
			s_cur_block = s_loop_start;
			s_pf_pos = s_pf_count = 0;
			for (int i = 0; i < 8; i++) {
				s_pred[i] = s_loop_pred[i];
				s_sidx[i] = s_loop_sidx[i];
			}
		} else if (s_skip_req != 0 || s_cur_block >= s_song_block_count) {
			change_song(s_skip_req < 0 ? -1 : 1);  /* rocker skip, or natural end */
			s_skip_req  = 0;
			s_cur_block = 0;
			s_pf_pos = s_pf_count = 0;   /* force refill from block 0 */
			for (int i = 0; i < 8; i++) { s_pred[i] = 0; s_sidx[i] = 0; }
			s_loop_active = false;
			s_loop_cache_ok = false;
		}

		/* Refill prefetch buffer when exhausted (batched CMD18 streaming read). */
		if (s_pf_pos >= s_pf_count) {
			uint32_t remaining = s_song_block_count - s_cur_block;
			uint32_t n = remaining < PREFETCH_BLOCKS ? remaining : PREFETCH_BLOCKS;
			uint32_t t0 = k_cycle_get_32();
			bool ok = emmc_read_blocks(s_song_block_start + s_cur_block, s_pf_buf, n);
			uint32_t dt = k_cycle_get_32() - t0;
			/* µs per block = cycles / (cyc_per_us) / n */
			s_last_read_us = k_cyc_to_us_floor32(dt) / (n ? n : 1u);
			if (s_last_read_us > s_max_read_us) s_max_read_us = s_last_read_us;
			if (ok) {
				s_pf_count = n;
				s_pf_pos   = 0;
			} else {
				s_pf_count = s_pf_pos = 0;
				goto tone_fallback;
			}

			/* Load the whole (decimated) level array into RAM once, at the
			 * first refill of the song, while we own the bus. One-time only. */
			if (s_lvl_present && !s_lvl_loaded) {
				if (emmc_read_blocks(s_lvl_start, s_lvl_ram, s_lvl_blocks))
					s_lvl_loaded = true;
			}
		}

		{
			const uint8_t *blk = s_pf_buf + s_pf_pos * 512u;
			s_pf_pos++;
			s_cur_block++;
			decode_block(blk, buf);
			return;
		}
tone_fallback:
		/* Seed the resonator on first use. tone_reset() is otherwise only
		 * reached via audio_set_source(AUDIO_SRC_TONE), which main never calls
		 * — so both state vars sat at 0 and this "audible indicator" emitted
		 * pure silence, making a failed eMMC read look like a quiet passage. */
		if (s_y1 == 0.0f && s_y2 == 0.0f) tone_reset();
		/* eMMC read failed: output tone as audible indicator (vs silence = invisible) */
		for (uint32_t i = 0; i < FRAMES_PER_BLOCK; i++) {
			float y = TONE_K * s_y1 - s_y2;
			s_y2 = s_y1; s_y1 = y;
			int32_t s = (int32_t)y;
			buf[2 * i]     = s;
			buf[2 * i + 1] = s;
		}
		return;
	}

	/* Silence (paused or no song loaded) */
	for (uint32_t i = 0; i < CHANNELS * FRAMES_PER_BLOCK; i++)
		buf[i] = 0;
}

/* ── Feed thread ─────────────────────────────────────────────────────────────── */

static void feed_main(void *a, void *b, void *c)
{
	(void)a; (void)b; (void)c;

	/* Prime a deep queue before START so the first (bursty) eMMC refill has
	 * full TX runway. TX queue holds NUM_BLOCKS; fill all but one. Stop early
	 * and graceful if a write can't be queued. */
	int primed = 0;
	for (int i = 0; i < (int)NUM_BLOCKS - 1; i++) {
		void *blk;
		if (k_mem_slab_alloc(&s_tx_slab, &blk, K_FOREVER) != 0) break;
		fill_block((int32_t *)blk);
		if (i2s_write(s_i2s, blk, BLOCK_BYTES) != 0) {
			k_mem_slab_free(&s_tx_slab, blk);
			break;
		}
		primed++;
	}
	if (primed == 0) return;

	if (i2s_trigger(s_i2s, I2S_DIR_TX, I2S_TRIGGER_START) != 0) return;
	s_running = true;

	while (1) {
		void *blk;

		/* Alloc-timeout = TX underrun (eMMC stall drained the queue) or i2s
		 * error state. Recover and keep going — NEVER exit the thread, or audio
		 * dies for good and play can't restart it. */
		if (k_mem_slab_alloc(&s_tx_slab, &blk, K_MSEC(I2S_TIMEOUT_MS)) != 0) {
			s_recoveries++;
			i2s_trigger(s_i2s, I2S_DIR_TX, I2S_TRIGGER_DROP);
			/* DROP returns all queued buffers to the slab. Re-prime the FULL
			 * queue (not just 2) before restarting — a shallow re-prime drains
			 * before the next ~20 ms eMMC prefetch read completes and underruns
			 * again, cascading. Full runway lets a transient underrun self-heal. */
			for (int i = 0; i < (int)NUM_BLOCKS - 1; i++) {
				/* Bounded, NOT K_FOREVER: this assumed DROP always returns every
				 * queued buffer to the slab. If DROP fails (i2s in an error
				 * state) nothing is freed, and a K_FOREVER alloc parks the feed
				 * thread for good — audio dies permanently with no way back,
				 * since only this thread can restart the stream. */
				if (k_mem_slab_alloc(&s_tx_slab, &blk, K_MSEC(I2S_TIMEOUT_MS)) != 0) break;
				fill_block((int32_t *)blk);
				if (i2s_write(s_i2s, blk, BLOCK_BYTES) != 0) {
					k_mem_slab_free(&s_tx_slab, blk);
					break;
				}
			}
			i2s_trigger(s_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
			continue;
		}

		fill_block((int32_t *)blk);
		if (i2s_write(s_i2s, blk, BLOCK_BYTES) != 0) {
			s_write_fails++;
			k_mem_slab_free(&s_tx_slab, blk);
			/* DMA not running (no BCLK yet or error). Drop into the recovery
			 * path on the next iteration via a short yield. */
			i2s_trigger(s_i2s, I2S_DIR_TX, I2S_TRIGGER_DROP);
			k_sleep(K_MSEC(2));
		} else {
			s_blocks_fed++;
		}
	}
}

/* ── Public API ─────────────────────────────────────────────────────────────── */

bool audio_init(void)
{
	codec_note_audio(0, 1);
	s_i2s = DEVICE_DT_GET(DT_NODELABEL(i2s0));
	if (!device_is_ready(s_i2s)) { codec_note_audio(0, 2); return false; }

	struct i2s_config cfg = {0};
	/* SWIDTH=24 (nRF I2S max; 32 is rejected as EINVAL). Samples are
	 * right-aligned in bits[23:0] of each 32-bit word, MSB(bit23) sent first. */
	cfg.word_size      = 24u;
	cfg.channels       = CHANNELS;
	cfg.format         = I2S_FMT_DATA_FORMAT_I2S;
	cfg.options        = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE;
	cfg.frame_clk_freq = 48000u;
	cfg.mem_slab       = &s_tx_slab;
	cfg.block_size     = BLOCK_BYTES;
	cfg.timeout        = I2S_TIMEOUT_MS;

	int cfg_ret = i2s_configure(s_i2s, I2S_DIR_TX, &cfg);
	codec_note_audio(cfg_ret, 3);
	if (cfg_ret != 0) return false;

	/* Preemptible priority 5 — ABOVE the main thread (lowered to 10) so main's
	 * button/USB/LED work can never starve the feed (that starvation was the old
	 * underrun cause), but BELOW the UI thread (priority 1) so the faders + LED
	 * meters can preempt the feed's long bit-bang eMMC reads and stay real-time.
	 * A cooperative feed could not be preempted at all, which froze the UI for
	 * ~68 ms per read. The feed yields to main whenever its alloc/i2s_write
	 * blocks (TX queue full), which is often, so main still gets WDT/buttons. */
	k_thread_create(&s_feed_thread, s_feed_stack, FEED_STACK,
			feed_main, NULL, NULL, NULL,
			K_PRIO_PREEMPT(5), K_FP_REGS, K_NO_WAIT);
	k_thread_name_set(&s_feed_thread, "i2s_feed");

	k_sleep(K_MSEC(20));
	codec_speaker_volume(0x20);
	codec_speaker_mute(false);

	codec_note_audio_running(s_running);
	codec_note_audio(0, 4);
	return true;
}

void audio_set_source(audio_source_t src)
{
	if (src == AUDIO_SRC_TONE) tone_reset();
	s_source = src;
}

bool audio_running(void)
{
	return s_running;
}

void audio_set_playlist(uint16_t song_count, uint16_t current_idx)
{
	s_song_count = song_count;
	s_cur_song   = current_idx;
}

void audio_load_song(uint32_t block_start, uint32_t block_count)
{
	s_playing          = false;
	s_loop_active      = false;
	s_loop_start_req   = false;
	s_loop_cache_ok    = false;
	s_loop_pf_dirty    = false;
	s_song_block_start = block_start;
	s_song_block_count = block_count;
	s_cur_block        = 0;
	s_pf_pos = s_pf_count = 0;
	for (int i = 0; i < 8; i++) { s_pred[i] = 0; s_sidx[i] = 0; }
	lvl_setup(block_start, block_count);
}

void audio_set_levels_enabled(bool enabled) { s_lvl_enabled = enabled; }

void audio_play(void)  { s_playing = true; }
void audio_pause(void) { s_playing = false; }
void audio_skip(int dir) { s_skip_req = (dir < 0) ? -1 : 1; }

void audio_loop_start(void)
{
	if (s_song_block_count == 0) return;
	s_loop_start_req = true;
}

void audio_loop_stop(void)
{
	s_loop_start_req = false;
	s_loop_active    = false;
	s_loop_pf_dirty  = true;
}

void audio_loop_change_divider(int direction)
{
	int next = (int)s_loop_div_idx + (direction < 0 ? -1 : 1);
	if (next < 0) next = 0;
	if (next >= (int)LOOP_DIV_COUNT) next = (int)LOOP_DIV_COUNT - 1;
	s_loop_div_idx = (uint8_t)next;
	/* Re-anchor and reload the cache at the new length so the wrap stays
	 * in-RAM and seamless; resolves like the rocker on the feed thread. */
	if (s_loop_active) s_loop_start_req = true;
}

void audio_loop_move(int direction)
{
	(void)direction;
	/* Re-anchor at a decoded block boundary; this is safe for the ADPCM state
	 * and gives the rocker a responsive momentary loop gesture. */
	if (s_loop_active) audio_loop_start();
}

bool audio_loop_active(void) { return s_loop_active; }
uint8_t audio_loop_div_idx(void) { return s_loop_div_idx; }
uint8_t audio_loop_div_count(void) { return LOOP_DIV_COUNT; }

/* Index of stem `stem` in the baked level array at block `blk`, or UINT32_MAX
 * if there is no valid baked level. The caller passes a smooth real-time block
 * estimate (not s_cur_block, which freezes during eMMC reads → choppy meters). */
static uint32_t lvl_index_at(uint32_t blk, int stem)
{
	if (!s_lvl_present || !s_lvl_loaded || s_lvl_count < LVL_STEMS) return 0xFFFFFFFFu;
	uint32_t idx = (blk / LVL_DECIM) * LVL_STEMS + (uint32_t)stem;
	uint32_t last = s_lvl_count - LVL_STEMS + (uint32_t)stem;
	if (idx > last) idx = last;
	return idx;
}

/* Baked level (0..255) of one stem at block `blk`, scaled by its live fader gain
 * so a muted/faded stem dims. For the per-stem track LEDs. */
uint32_t audio_stem_level_at(uint32_t blk, int stem)
{
	if (stem < 0 || stem > 3) return 0;
	uint32_t idx = lvl_index_at(blk, stem);
	if (idx == 0xFFFFFFFFu) return 0;
	uint32_t v = s_lvl_ram[idx];
	return (v * s_stem_gain[stem]) >> 8;   /* 0..255 */
}

/* Overall VU level (0..255) for the pb-LED meter at block `blk`: the four stems
 * summed, each scaled by its fader gain, so the meter tracks the live mix.
 * Falls back to the on-device peak-hold for discs without baked levels. */
uint32_t audio_vu_level_at(uint32_t blk)
{
	if (s_lvl_present) {
		if (!s_lvl_loaded || s_lvl_count < LVL_STEMS) return 0;
		uint32_t sum = 0;
		for (int s = 0; s < 4; s++) sum += audio_stem_level_at(blk, s);   /* 0..1020 */
		sum >>= 2;                                                        /* 0..255  */
		return sum > 255 ? 255 : sum;
	}
	/* fallback: on-device peak-hold (0..32767), scaled to 0..255. */
	int32_t v = s_peak;
	s_peak = 0;
	v >>= 7;
	return (uint32_t)(v > 255 ? 255 : v);
}

void audio_toggle(void) { s_playing = !s_playing; }

uint32_t audio_cur_block(void) { return s_cur_block; }

uint32_t audio_last_read_us(void) { return s_last_read_us; }

static volatile uint16_t s_dbg_ain0;
static volatile uint16_t s_dbg_ain1;
void audio_dbg_set_ain0(uint16_t v) { s_dbg_ain0 = v; }
void audio_dbg_set_ain1(uint16_t v) { s_dbg_ain1 = v; }

void audio_get_diag(audio_diag_t *d)
{
	if (!d) return;
	d->recoveries  = s_recoveries;
	d->write_fails = s_write_fails;
	d->max_read_us = s_max_read_us;
	d->last_read_us = s_last_read_us;
	d->cur_block   = s_cur_block;
	d->blocks_fed  = s_blocks_fed;
	d->crc_errors  = emmc_crc_errors();
	d->ain0        = s_dbg_ain0;
	d->ain1        = s_dbg_ain1;
}

bool audio_is_playing(void) { return s_playing; }
