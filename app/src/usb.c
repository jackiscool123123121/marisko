#include "usb.h"
#include "disk.h"
#include "emmc.h"
#include "codec.h"
#include "audio.h"
#include "saadc.h"
#include "util.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

static const struct device *cdc_dev;

RING_BUF_DECLARE(g_rx_rb, 65536);

/* ── Low-level I/O ───────────────────────────────────────────────────────────── */

static void uart_cb(const struct device *dev, void *user_data)
{
	(void)user_data;
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			/* Pull straight into the ring's contiguous claim region — avoids the
			 * bounce buffer + a second copy, and drains the whole CDC FIFO in
			 * one go for higher RX throughput. */
			uint8_t *dst;
			uint32_t space = ring_buf_put_claim(&g_rx_rb, &dst, 4096);
			if (space == 0) {
				/* Ring full: drain FIFO to a scratch buf to keep USB flowing. */
				uint8_t tmp[64];
				int n = uart_fifo_read(dev, tmp, sizeof(tmp));
				(void)n;
				ring_buf_put_finish(&g_rx_rb, 0);
				continue;
			}
			int n = uart_fifo_read(dev, dst, space);
			ring_buf_put_finish(&g_rx_rb, n > 0 ? (uint32_t)n : 0);
		}
	}
}

static bool usb_read_bytes(uint8_t *buf, uint32_t len, uint32_t timeout_loops)
{
	uint32_t got = 0;
	while (got < len) {
		uint32_t n = ring_buf_get(&g_rx_rb, buf + got, len - got);
		got += n;
		if (got < len) {
			if (timeout_loops-- == 0) return false;
			/* Sleep (don't spin) so the lower-priority USBD thread gets CPU to
			 * service bulk-OUT and refill the ring — spinning here starves it
			 * and throttles upload throughput. 1 tick ≈ 30 µs. */
			k_sleep(K_TICKS(1));
			feed_wdt();
		}
	}
	return true;
}

static void usb_write_bytes(const uint8_t *buf, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++)
		uart_poll_out(cdc_dev, buf[i]);
}

static void send_ok(const uint8_t *payload, uint32_t plen)
{
	uint8_t hdr[5];
	hdr[0] = USB_STATUS_OK;
	hdr[1] = (uint8_t)(plen & 0xFFu);
	hdr[2] = (uint8_t)((plen >> 8) & 0xFFu);
	hdr[3] = (uint8_t)((plen >> 16) & 0xFFu);
	hdr[4] = (uint8_t)((plen >> 24) & 0xFFu);
	usb_write_bytes(hdr, 5);
	if (payload && plen > 0)
		usb_write_bytes(payload, plen);
}

static void send_ok_stream(uint32_t total_len)
{
	uint8_t hdr[5];
	hdr[0] = USB_STATUS_OK;
	hdr[1] = (uint8_t)(total_len & 0xFFu);
	hdr[2] = (uint8_t)((total_len >> 8) & 0xFFu);
	hdr[3] = (uint8_t)((total_len >> 16) & 0xFFu);
	hdr[4] = (uint8_t)((total_len >> 24) & 0xFFu);
	usb_write_bytes(hdr, 5);
}

static void send_err(void)
{
	uint8_t hdr[5] = { USB_STATUS_ERR, 0, 0, 0, 0 };
	usb_write_bytes(hdr, 5);
}

/* ── Upload state ────────────────────────────────────────────────────────────── */

static uint8_t s_block_buf[512];

typedef struct {
	bool     active;
	bool     multi_begun;  /* CMD25 session is open */
	uint16_t song_idx;
	uint32_t block_start;
	uint32_t blocks_expected;   /* audio + level blocks (total streamed) */
	uint32_t audio_blocks;      /* audio-only count → catalog block_count */
	uint32_t blocks_written;
	uint32_t session_left;   /* blocks remaining in the current CMD25 chunk */
} upload_t;

static upload_t    g_upload;
static disk_header_t g_hdr;
static bool        g_hdr_cached;
static uint8_t     g_ul_fail;       /* 0=none 1=usb-read-timeout 2=resp-reject 3=busy-timeout 4=unknown-write-fail */
static uint32_t    g_ul_fail_block; /* block index where it failed */

static bool ensure_hdr(void)
{
	if (g_hdr_cached) return true;
	if (!disk_read_header(&g_hdr)) return false;
	g_hdr_cached = true;
	return true;
}

/* ── Command handlers ────────────────────────────────────────────────────────── */

static void handle_ping(void) { send_ok(NULL, 0); }

/* 0x10 POWER_OFF: ack first, then let the main loop drop into SYSTEM_OFF. Acking
 * first matters — once SYSTEM_OFF runs, USB is gone and the host would time out. */
static volatile bool g_power_off_req;
bool usb_power_off_requested(void) { return g_power_off_req; }
static void handle_power_off(void)
{
	send_ok(NULL, 0);
	g_power_off_req = true;
}

static void handle_disk_info(void)
{
	if (!emmc_read_block(0, s_block_buf)) { send_err(); return; }
	send_ok(s_block_buf, 512);
}

static void handle_extcsd_dump(void)
{
	send_ok(emmc_ext_csd(), 512);
}

/* 0x0C READ_BLOCK: block_addr[4 LE] → data[512]. Pauses audio to avoid eMMC
 * bus contention with the feed thread. Diagnostic only. */
static void handle_read_block(const uint8_t *payload, uint32_t plen)
{
	if (plen < 4) { send_err(); return; }
	audio_pause();
	k_sleep(K_MSEC(10));   /* let any in-flight feed-thread read finish */
	uint32_t addr = (uint32_t)payload[0]        | ((uint32_t)payload[1] << 8) |
	                ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);
	if (!emmc_read_block(addr, s_block_buf)) { send_err(); return; }
	send_ok(s_block_buf, 512);
}

/* 0x0D WRITE_PROBE: block_addr[4 LE] → writes a test pattern, reads it back,
 * returns 1 byte: 0=write-fail 1=readback-fail 2=mismatch 3=ok.
 * DESTRUCTIVE to that block — diagnostic for mapping the writable region. */
static uint8_t s_probe_rb[512];
static void handle_write_probe(const uint8_t *payload, uint32_t plen)
{
	if (plen < 4) { send_err(); return; }
	audio_pause();
	k_sleep(K_MSEC(10));
	uint32_t addr = (uint32_t)payload[0]        | ((uint32_t)payload[1] << 8) |
	                ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);
	for (int i = 0; i < 512; i++)
		s_block_buf[i] = (uint8_t)(i * 7u + addr + 0x5Au);
	uint8_t result;
	if (!emmc_write_block(addr, s_block_buf)) {
		result = 0;
	} else if (!emmc_read_block(addr, s_probe_rb)) {
		result = 1;
	} else {
		result = 3;
		for (int i = 0; i < 512; i++)
			if (s_probe_rb[i] != s_block_buf[i]) { result = 2; break; }
	}
	send_ok(&result, 1);
}

/* 0x0E WRITE_STRESS: count[4 LE] → writes `count` test blocks starting at
 * block 9 via CMD24 (no USB per block), returns first-fail block index [4 LE]
 * (0xFFFFFFFF = all ok). Maps cumulative-write failures. DESTRUCTIVE. */
static void handle_write_stress(const uint8_t *payload, uint32_t plen)
{
	if (plen < 4) { send_err(); return; }
	audio_pause();
	k_sleep(K_MSEC(10));
	uint32_t count = (uint32_t)payload[0]        | ((uint32_t)payload[1] << 8) |
	                 ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);
	uint32_t first_fail = 0xFFFFFFFFu;
	for (uint32_t i = 0; i < count; i++) {
		for (int b = 0; b < 512; b++)
			s_block_buf[b] = (uint8_t)(b + i);
		feed_wdt();
		if (!emmc_write_block(9u + i, s_block_buf)) { first_fail = i; break; }
	}
	uint8_t r[4] = { (uint8_t)first_fail, (uint8_t)(first_fail >> 8),
	                 (uint8_t)(first_fail >> 16), (uint8_t)(first_fail >> 24) };
	send_ok(r, 4);
}

/* 0x0F AUDIO_DIAG: no payload → audio_diag_t (6 × u32 LE). Feed-thread health. */
static void handle_audio_diag(void)
{
	audio_diag_t d;
	audio_get_diag(&d);
	send_ok((const uint8_t *)&d, sizeof(d));
}

static void handle_codec_diag(void)
{
	/* Returns the snapshot captured at boot — no live I2C here, because a TWIM
	 * read can block K_FOREVER and trip the watchdog from this context. */
	codec_refresh_diag();
	codec_diag_t diag;
	codec_get_diag(&diag);
	/* Named fields now, same wire offsets as before (23..26, 28, 29..30). The
	 * old byte-poking also wrote cur_block/last_read_us into 23..28 and then
	 * immediately overwrote them with this, so that pair never reached the host
	 * — dropped rather than resurrected, since AUDIO_DIAG already reports both. */
	diag.ul_fail_block[0] = (uint8_t)(g_ul_fail_block);
	diag.ul_fail_block[1] = (uint8_t)(g_ul_fail_block >> 8);
	diag.ul_fail_block[2] = (uint8_t)(g_ul_fail_block >> 16);
	diag.ul_fail_block[3] = (uint8_t)(g_ul_fail_block >> 24);
	diag.ul_fail = g_ul_fail;
	/* Live AIN1 (ladder 2: vol up/down + FWD/RWD). */
	int ain1 = saadc_read(2u);  /* PSELP=2 → AIN1 */
	if (ain1 < 0) ain1 = 0;
	diag.ain1[0] = (uint8_t)(ain1);
	diag.ain1[1] = (uint8_t)((uint32_t)ain1 >> 8);
	send_ok((const uint8_t *)&diag, sizeof(diag));
}

static void handle_disk_format(void)
{
	g_hdr_cached = false;
	if (g_upload.multi_begun) {
		emmc_write_multi_end();
		g_upload.multi_begun = false;
	}
	g_upload.active = false;
	if (!disk_format()) { send_err(); return; }
	g_hdr_cached = false;
	send_ok(NULL, 0);
}

/* 0x04 SONG_BEGIN: name[24] + audio_blocks[4] (+ lvl_blocks[4], v2) → song_idx[2 LE].
 * Total stream = audio + level blocks; catalog records audio_blocks, the level
 * region follows at block_start + audio_blocks. 28-byte payload → lvl_blocks=0. */
static void handle_song_begin(const uint8_t *payload, uint32_t plen)
{
	if (plen < 28) { send_err(); return; }
	if (g_upload.active) { send_err(); return; }

	/* Pause audio BEFORE the first disk touch — ensure_hdr() reads block 0, so
	 * pausing after it left that read racing the feed thread. (The eMMC bus
	 * lock makes this correct either way; pausing first just keeps the feed
	 * from queueing up behind rome's disk ops.) */
	audio_pause();

	if (!ensure_hdr()) { send_err(); return; }

	uint32_t audio_blocks = (uint32_t)payload[24]        | ((uint32_t)payload[25] << 8) |
	                        ((uint32_t)payload[26] << 16) | ((uint32_t)payload[27] << 24);
	uint32_t lvl_blocks = 0;
	if (plen >= 32)
		lvl_blocks = (uint32_t)payload[28]        | ((uint32_t)payload[29] << 8) |
		             ((uint32_t)payload[30] << 16) | ((uint32_t)payload[31] << 24);

	disk_song_entry_t entry = {0};
	for (int i = 0; i < 24; i++) entry.name[i] = (char)payload[i];
	entry.name[23]    = '\0';
	entry.block_start = g_hdr.next_free_block;
	entry.block_count = audio_blocks;

	uint16_t idx = disk_add_song(&g_hdr, &entry);
	if (idx == 0xFFFFu) { send_err(); return; }

	if (!disk_write_header(&g_hdr)) { send_err(); return; }
	g_hdr_cached = true;

	g_upload.active          = true;
	g_upload.multi_begun     = false;
	g_upload.song_idx        = idx;
	g_upload.block_start     = g_hdr.next_free_block;
	g_upload.blocks_expected = audio_blocks + lvl_blocks;
	g_upload.audio_blocks    = audio_blocks;
	g_upload.blocks_written  = 0;

	uint8_t resp[2] = { (uint8_t)(idx & 0xFFu), (uint8_t)(idx >> 8) };
	send_ok(resp, 2);
}

/* 0x09 SONG_MULTIBLOCK: payload = count[2 LE]; followed by count×512 raw bytes.
 * CMD25 streaming (SPIM3 16 MHz burst per block) with the write cache on — the
 * fast path. Session opened lazily on the first batch, kept open across batches,
 * closed at SONG_COMMIT. */
static void handle_song_multiblock(const uint8_t *count_payload, uint32_t plen)
{
	/* Without the length check a short/empty packet reads the count out of
	 * whatever the previous command left in s_block_buf. */
	if (plen < 2) { send_err(); return; }
	uint16_t count = (uint16_t)count_payload[0] | ((uint16_t)count_payload[1] << 8);
	if (!g_upload.active || count == 0) { send_err(); return; }
	if ((uint32_t)g_upload.blocks_written + count > g_upload.blocks_expected) {
		send_err(); return;
	}

	if (!g_upload.multi_begun) {
		if (!emmc_write_multi_begin(g_upload.block_start, g_upload.blocks_expected)) {
			g_ul_fail = 2; g_ul_fail_block = g_upload.blocks_written; send_err(); return;
		}
		g_upload.multi_begun = true;
	}

	bool ok = true;
	for (uint16_t i = 0; i < count; i++) {
		if (!usb_read_bytes(s_block_buf, 512, 1000000)) {
			g_ul_fail = 1; g_ul_fail_block = g_upload.blocks_written; ok = false; break;
		}
		feed_wdt();

		if (!emmc_write_multi_block(s_block_buf)) {
			/* Diagnostic only -- NOT retried. The 515-byte SPIM burst for this
			 * block has ALREADY been fully transmitted to the card by the time
			 * a rejection is even detected (spim_xfer happens unconditionally,
			 * before the status-token read). Blindly resending the same block
			 * into an ongoing CMD25 stream is not verified safe: the card may
			 * treat the resend as the NEXT sequential block rather than a
			 * retry of the rejected one, silently shifting every block after
			 * it by one position -- an upload that reports success but wrote
			 * corrupted audio, which is worse than a clean failure. Distinct
			 * fail codes (was a single collapsed "2=write-fail") so the next
			 * failure says exactly which failure mode it was instead of
			 * leaving it a guess: 2=resp-reject, 3=busy-timeout, 4=unknown. */
			uint8_t mbf = emmc_mb_fail();
			g_ul_fail = (mbf == 1) ? 2 : (mbf == 2) ? 3 : 4;
			g_ul_fail_block = g_upload.blocks_written;
			ok = false;
			break;
		}
		g_upload.blocks_written++;
	}

	if (ok) {
		send_ok(NULL, 0);
	} else {
		emmc_write_multi_end();
		g_upload.multi_begun = false;
		g_upload.active = false;
		send_err();
		uint32_t quiet = 0;
		while (quiet < 10) {   /* ~100 ms of continuous silence */
			k_sleep(K_MSEC(10));
			feed_wdt();
			if (ring_buf_size_get(&g_rx_rb) == 0) quiet++;
			else { ring_buf_reset(&g_rx_rb); quiet = 0; }
		}
	}
}

/* 0x05 SONG_BLOCK: block_data[512] */
static void handle_song_block(const uint8_t *payload, uint32_t plen)
{
	if (!g_upload.active || plen < 512) { send_err(); return; }
	if (g_upload.blocks_written >= g_upload.blocks_expected) { send_err(); return; }

	uint32_t block_addr = g_upload.block_start + g_upload.blocks_written;

	feed_wdt();
	if (!emmc_write_block(block_addr, payload)) { send_err(); return; }

	g_upload.blocks_written++;
	send_ok(NULL, 0);
}

/* 0x06 SONG_COMMIT */
static void handle_song_commit(void)
{
	if (!g_upload.active) { send_err(); return; }

	/* Close CMD25 session */
	if (g_upload.multi_begun) {
		emmc_write_multi_end();
		g_upload.multi_begun = false;
	}

	/* Catalog records audio blocks only; the level region follows at
	 * block_start + block_count. next_free jumps past audio + levels. */
	disk_song_entry_t entry;
	if (!disk_read_song(g_upload.song_idx, &entry)) { send_err(); return; }
	entry.block_count = g_upload.audio_blocks;
	if (!disk_write_song(g_upload.song_idx, &entry)) { send_err(); return; }

	/* Advance next_free_block past everything written (audio + levels) */
	g_hdr.next_free_block = g_upload.block_start + g_upload.blocks_written;
	g_upload.active = false;

	if (!disk_write_header(&g_hdr)) { send_err(); return; }
	g_hdr_cached = true;

	/* Arm the song we just wrote so it is playable immediately. The catalog
	 * scan only ran once at boot (main.c), so uploading to a device that had
	 * no songs left the feed thread with block_count == 0 — play did nothing
	 * until a reboot, with no indication why. */
	audio_set_source(AUDIO_SRC_ADPCM);
	audio_set_playlist(g_hdr.song_count, g_upload.song_idx);
	audio_set_levels_enabled(g_hdr.version >= 2);
	audio_load_song(g_upload.block_start, g_upload.audio_blocks);

	send_ok(NULL, 0);
}

/* 0x07 SONG_REMOVE: song_idx[2 LE] */
static void handle_song_remove(const uint8_t *payload, uint32_t plen)
{
	if (plen < 2) { send_err(); return; }
	uint16_t idx = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
	if (!disk_remove_song(idx)) { send_err(); return; }
	send_ok(NULL, 0);
}

/* 0x08 CATALOG_READ: stream all 8 catalog blocks (4096 bytes total) */
static void handle_catalog_read(void)
{
	send_ok_stream(DISK_CATALOG_BLOCKS * 512u);
	for (uint8_t b = 0; b < DISK_CATALOG_BLOCKS; b++) {
		if (!emmc_read_block(DISK_CATALOG_START + b, s_block_buf)) {
			/* Already sent header — fill remainder with zeros on error */
			uint8_t zeros[512] = {0};
			usb_write_bytes(zeros, 512);
		} else {
			usb_write_bytes(s_block_buf, 512);
		}
	}
}

/* ── Command dispatch ────────────────────────────────────────────────────────── */

static void dispatch(uint8_t cmd, const uint8_t *payload, uint32_t plen)
{
	switch (cmd) {
	case USB_CMD_PING:         handle_ping();                          break;
	case USB_CMD_DISK_INFO:    handle_disk_info();                     break;
	case USB_CMD_DISK_FORMAT:  handle_disk_format();                   break;
	case USB_CMD_SONG_BEGIN:   handle_song_begin(payload, plen);       break;
	case USB_CMD_SONG_BLOCK:   handle_song_block(payload, plen);       break;
	case USB_CMD_SONG_COMMIT:  handle_song_commit();                   break;
	case USB_CMD_SONG_REMOVE:  handle_song_remove(payload, plen);      break;
	case USB_CMD_CATALOG_READ:    handle_catalog_read();                break;
	case USB_CMD_SONG_MULTIBLOCK: handle_song_multiblock(payload, plen); break;
	case USB_CMD_EXTCSD_DUMP:     handle_extcsd_dump();                 break;
	case USB_CMD_CODEC_DIAG:      handle_codec_diag();                  break;
	case USB_CMD_READ_BLOCK:      handle_read_block(payload, plen);     break;
	case USB_CMD_WRITE_PROBE:     handle_write_probe(payload, plen);    break;
	case USB_CMD_WRITE_STRESS:    handle_write_stress(payload, plen);   break;
	case USB_CMD_AUDIO_DIAG:      handle_audio_diag();                  break;
	case USB_CMD_POWER_OFF:       handle_power_off();                   break;
	default:                      send_err();                          break;
	}
}

/* ── Public API ──────────────────────────────────────────────────────────────── */

bool usb_cdc_init(void)
{
	cdc_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(cdc_dev)) return false;
	uart_irq_callback_set(cdc_dev, uart_cb);
	uart_irq_rx_enable(cdc_dev);
	return true;
}

bool usb_upload_active(void)
{
	return g_upload.active;
}

uint32_t usb_upload_progress_permille(void)
{
	if (!g_upload.active || g_upload.blocks_expected == 0) return 0;
	uint64_t pm = (uint64_t)g_upload.blocks_written * 1000u / g_upload.blocks_expected;
	return (uint32_t)(pm > 1000u ? 1000u : pm);
}

bool usb_cdc_connected(void)
{
	if (!cdc_dev) return false;
	uint32_t dtr = 0;
	uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &dtr);
	return dtr != 0;
}

void usb_cdc_poll(void)
{
	if (!cdc_dev) return;

	if (ring_buf_size_get(&g_rx_rb) < 5) return;

	uint8_t hdr[5];
	if (!usb_read_bytes(hdr, 5, 0)) return;

	uint8_t  cmd  = hdr[0];
	uint32_t plen = (uint32_t)hdr[1]        | ((uint32_t)hdr[2] << 8) |
	                ((uint32_t)hdr[3] << 16) | ((uint32_t)hdr[4] << 24);

	if (plen > sizeof(s_block_buf)) {
		uint8_t drain[64];
		uint32_t remaining = plen;
		while (remaining > 0) {
			uint32_t chunk = remaining < sizeof(drain) ? remaining : sizeof(drain);
			if (!usb_read_bytes(drain, chunk, 100000)) break;
			remaining -= chunk;
		}
		ring_buf_reset(&g_rx_rb);
		send_err();
		return;
	}

	if (plen > 0 && !usb_read_bytes(s_block_buf, plen, 1000000)) {
		send_err();
		return;
	}

	dispatch(cmd, s_block_buf, plen);
}
