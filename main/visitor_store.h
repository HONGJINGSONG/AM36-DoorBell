#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Visitor records. An unanswered ring is kept as the raw stream (JPEG frames,
 * the Opus audio that rode with them, arrival times); playback replays the
 * arrivals on their original clock through the live decode / speaker paths.
 *
 *   begin()   the ring started: reserve the pending record
 *   push()    a frame completed (radio task): append it
 *   offer_thumbnail()  first decoded frame (image task) is scaled for the list
 *   commit()  the ring timed out unanswered: keep it, evicting the oldest
 *   discard() answered or dismissed: drop it
 *
 * A ring of APP_VISITOR_RECORDS in PSRAM; records do not survive a reboot and
 * age from uptime. Motion snapshots (PIR stills) live in a separate ring at the
 * end of this file. */

/* Thumbnail geometry, in the screen's (rotated) orientation. */
#define VISITOR_THUMB_W  (APP_IMAGE_OUTPUT_HEIGHT / APP_VISITOR_THUMB_SCALE)
#define VISITOR_THUMB_H  (APP_IMAGE_OUTPUT_WIDTH / APP_VISITOR_THUMB_SCALE)

typedef struct {
    uint32_t start_uptime_ms;   /* when the ring began (esp_timer clock) */
    uint32_t duration_ms;       /* offset of the last frame from the first */
    uint32_t frame_count;
    size_t   bytes;             /* JPEG + audio payload kept */
    /* VISITOR_THUMB_W x VISITOR_THUMB_H pixels in the image canvas format
     * (see ui_gw_make_thumbnail), or NULL if no frame was ever decoded.
     * Valid until the record is evicted - rebuild any view that shows it
     * when the store changes. */
    const uint16_t *thumb;
} visitor_record_info_t;

/* Scale a decoded, unrotated RGB565 frame down into a thumbnail. Provided by
 * the UI, which owns the canvas pixel mapping. */
typedef void (*visitor_thumb_fn_t)(const uint16_t *rgb565, uint32_t w, uint32_t h,
                                   uint16_t *thumb);

/* Playback sinks. All are called from the store's playback task, on the
 * record's original frame clock. `jpeg` points into the record: copy it if
 * it has to outlive the call. `completed` is false when playback was stopped
 * before the last frame. */
typedef void (*visitor_frame_sink_t)(const uint8_t *jpeg, size_t len);
typedef void (*visitor_audio_sink_t)(const uint8_t *opus_blob, size_t len,
                                     uint16_t first_seq);
typedef void (*visitor_play_done_t)(bool completed);

esp_err_t visitor_store_init(void);
void visitor_store_set_thumb_fn(visitor_thumb_fn_t fn);
void visitor_store_set_playback_sinks(visitor_frame_sink_t frame,
                                      visitor_audio_sink_t audio,
                                      visitor_play_done_t done);

/* Recording. begin() returns false when no memory could be reserved; the
 * ring goes on without a record then. push() is a no-op outside a pending
 * record and once the record is full. */
bool visitor_store_begin(void);
bool visitor_store_recording(void);
void visitor_store_push(const uint8_t *jpeg, size_t jpeg_len,
                        const uint8_t *audio, size_t audio_len,
                        uint16_t audio_seq);
void visitor_store_offer_thumbnail(const uint16_t *rgb565, uint32_t w, uint32_t h);
/* commit() returns false when the pending record had no frames (nothing was
 * kept). Both end the pending record. */
bool visitor_store_commit(void);
void visitor_store_discard(void);

/* Reading. index 0 is the newest record. */
int  visitor_store_count(void);
bool visitor_store_get(int index, visitor_record_info_t *out);

/* Playback of one record through the sinks. play() returns false if the
 * index is empty or a playback is already running. stop() returns once the
 * playback task has let go of the record (the done sink fires first). */
bool visitor_store_play(int index);
void visitor_store_stop(void);
bool visitor_store_playing(void);

/* Motion snapshots: a ring of APP_VISITOR_SNAPSHOTS PIR stills, newest first.
 * Thumbnail buffers are allocated once and reused in place on eviction, so a
 * list canvas pointing at one never reads freed memory. */
typedef struct {
    uint32_t uptime_ms;         /* when it arrived (esp_timer clock) */
    size_t   jpeg_len;
    const uint16_t *thumb;      /* VISITOR_THUMB_W x VISITOR_THUMB_H, never NULL */
} visitor_snapshot_info_t;

/* add() copies the JPEG and scales `rgb565` (the decoded frame) into the
 * slot's thumbnail with the registered thumb function. Returns false when
 * nothing could be kept (no memory, no thumb function). Any task. */
bool visitor_store_snapshot_add(const uint8_t *jpeg, size_t jpeg_len,
                                const uint16_t *rgb565, uint32_t w, uint32_t h);
int  visitor_store_snapshot_count(void);
bool visitor_store_snapshot_get(int index, visitor_snapshot_info_t *out);
/* Hand snapshot `index` to the frame sink once, synchronously, for a full
 * screen view. The sink copies it (as it does for playback frames), so the
 * store is free again when this returns. Returns false for an empty index. */
bool visitor_store_snapshot_show(int index);

#ifdef __cplusplus
}
#endif
