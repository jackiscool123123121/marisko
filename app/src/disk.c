#include "disk.h"
#include "emmc.h"
#include <string.h>

_Static_assert(sizeof(disk_song_entry_t) == 32, "disk_song_entry_t must be 32 bytes");
_Static_assert(sizeof(disk_header_t) == 512, "disk_header_t must be 512 bytes");

/* One scratch block shared by every catalog access, reached from two threads:
 * the audio feed thread (disk_read_song on song change) and main/USB (rome's
 * add/remove/commit). disk_write_song is a read-modify-write of a whole 512 B
 * catalog block through this buffer — a feed-thread read landing between its
 * read and its write made main flush the WRONG block back, wiping up to 16
 * catalog entries. Serialize on the eMMC bus lock rather than a second mutex of
 * our own: a separate lock deadlocked against the upload's CMD25 session (see
 * emmc_bus_lock). It is recursive, so the emmc_* calls below nest for free. */

static uint8_t s_cat_buf[512];

bool disk_read_header(disk_header_t *hdr)
{
    if (!hdr) return false;
    if (!emmc_read_block(DISK_HEADER_BLOCK, (uint8_t *)hdr)) return false;
    return hdr->magic == DISK_MAGIC && hdr->version == DISK_VERSION;
}

bool disk_write_header(const disk_header_t *hdr)
{
    if (!hdr) return false;
    return emmc_write_block(DISK_HEADER_BLOCK, (const uint8_t *)hdr);
}

bool disk_format(void)
{
    static disk_header_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.magic            = DISK_MAGIC;
    hdr.version          = DISK_VERSION;
    hdr.song_count       = 0;
    hdr.next_free_block  = DISK_DATA_START_BLOCK;
    if (!disk_write_header(&hdr)) return false;

    emmc_bus_lock();
    memset(s_cat_buf, 0, sizeof(s_cat_buf));
    bool ok = true;
    for (uint8_t b = 0; b < DISK_CATALOG_BLOCKS; b++) {
        if (!emmc_write_block(DISK_CATALOG_START + b, s_cat_buf)) { ok = false; break; }
    }
    emmc_bus_unlock();
    return ok;
}

static uint32_t cat_block(uint16_t idx)
{
    return DISK_CATALOG_START + idx / DISK_SONGS_PER_BLOCK;
}

static uint32_t cat_offset(uint16_t idx)
{
    return (idx % DISK_SONGS_PER_BLOCK) * (uint32_t)sizeof(disk_song_entry_t);
}

bool disk_read_song(uint16_t idx, disk_song_entry_t *entry)
{
    if (!entry || idx >= DISK_MAX_SONGS) return false;
    emmc_bus_lock();
    bool ok = emmc_read_block(cat_block(idx), s_cat_buf);
    if (ok)
        memcpy(entry, s_cat_buf + cat_offset(idx), sizeof(disk_song_entry_t));
    emmc_bus_unlock();
    return ok;
}

bool disk_write_song(uint16_t idx, const disk_song_entry_t *entry)
{
    if (!entry || idx >= DISK_MAX_SONGS) return false;
    /* Held across the whole read-modify-write, not just each half. */
    emmc_bus_lock();
    bool ok = emmc_read_block(cat_block(idx), s_cat_buf);
    if (ok) {
        memcpy(s_cat_buf + cat_offset(idx), entry, sizeof(disk_song_entry_t));
        ok = emmc_write_block(cat_block(idx), s_cat_buf);
    }
    emmc_bus_unlock();
    return ok;
}

uint16_t disk_add_song(disk_header_t *hdr, const disk_song_entry_t *entry)
{
    if (!hdr || !entry || entry->name[0] == DISK_ENTRY_FREE_MARKER) return 0xFFFFu;

    /* Whole scan-then-claim is one critical section, so two callers can't pick
     * the same free slot. Recursive re-entry from read/write_song is free. */
    emmc_bus_lock();
    uint16_t result = 0xFFFFu;

    /* Scan allocated range for a deleted hole to reuse */
    for (uint16_t i = 0; i < hdr->song_count; i++) {
        disk_song_entry_t e;
        if (!disk_read_song(i, &e)) goto out;
        if (e.name[0] == DISK_ENTRY_FREE_MARKER) {
            if (disk_write_song(i, entry)) result = i;
            goto out;
        }
    }

    /* Append */
    if (hdr->song_count >= DISK_MAX_SONGS) goto out;
    if (!disk_write_song(hdr->song_count, entry)) goto out;
    result = hdr->song_count;
    hdr->song_count++;

out:
    emmc_bus_unlock();
    return result;
}

bool disk_remove_song(uint16_t idx)
{
    disk_song_entry_t empty;
    memset(&empty, 0, sizeof(empty));
    return disk_write_song(idx, &empty);
}
