#include "visitor_store.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"

static const char *TAG = "visitors";

/* One frame of a record: the JPEG and the audio that rode with it sit back to
 * back in the record's data buffer, exactly as the node's payload had them. */
typedef struct {
    uint32_t t_ms;        /* arrival offset from the record's first frame */
    uint32_t jpeg_off;
    uint32_t jpeg_len;
    uint32_t audio_len;   /* Opus blob right after the JPEG, 0 = none */
    uint16_t audio_seq;   /* sequence of the blob's first Opus frame */
} frame_entry_t;

typedef struct {
    bool           valid;
    uint32_t       start_uptime_ms;
    uint32_t       first_frame_ms;   /* 0 = no frame yet */
    uint32_t       duration_ms;
    uint32_t       frame_count;
    uint32_t       frame_cap;
    frame_entry_t *frames;
    uint8_t       *data;
    size_t         data_len;
    size_t         data_cap;
    uint16_t      *thumb;
    bool           thumb_ready;
    bool           full_logged;
} record_t;

#define STORE_CAPS  (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define THUMB_BYTES (VISITOR_THUMB_W * VISITOR_THUMB_H * sizeof(uint16_t))

/* s_records[0] is the newest. */
static record_t s_records[APP_VISITOR_RECORDS];
static int s_count = 0;
static record_t s_pending;
static bool s_pending_active = false;

/* Motion snapshots, s_snaps[0] newest. The thumbnail slots are allocated once
 * in init and only ever overwritten in place (see the header), so the list
 * canvases that point at them stay valid across evictions. */
typedef struct {
    bool      valid;
    uint32_t  uptime_ms;
    uint8_t  *jpeg;
    size_t    jpeg_len;
    uint16_t *thumb;      /* one of s_snap_thumbs, owned by the slot for life */
} snapshot_t;
static snapshot_t s_snaps[APP_VISITOR_SNAPSHOTS];
static uint16_t *s_snap_thumbs[APP_VISITOR_SNAPSHOTS];
static int s_snap_count = 0;

static SemaphoreHandle_t s_mutex = NULL;
static visitor_thumb_fn_t s_thumb_fn = NULL;
static visitor_frame_sink_t s_frame_sink = NULL;
static visitor_audio_sink_t s_audio_sink = NULL;
static visitor_play_done_t s_done_sink = NULL;

/* Playback: one persistent task, woken per play, reads the record it was
 * given without the mutex - commit() stops it before it evicts anything. */
static TaskHandle_t s_play_task = NULL;
static SemaphoreHandle_t s_play_sem = NULL;
static volatile bool s_playing = false;
static volatile bool s_play_stop = false;
static int s_play_index = -1;

#define PLAY_TASK_STACK_BYTES 8192   /* the done sink rebuilds an LVGL page */
#define PLAY_TASK_PRIORITY    3
#define PLAY_TASK_CORE        0

static void play_task(void *arg);

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static TickType_t ticks_min_1(uint32_t ms)
{
    TickType_t t = pdMS_TO_TICKS(ms);
    return t == 0 ? 1 : t;
}

static void record_free(record_t *r)
{
    if (r->frames) heap_caps_free(r->frames);
    if (r->data) heap_caps_free(r->data);
    if (r->thumb) heap_caps_free(r->thumb);
    memset(r, 0, sizeof(*r));
}

esp_err_t visitor_store_init(void)
{
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) return ESP_ERR_NO_MEM;
    }
    memset(s_records, 0, sizeof(s_records));
    s_count = 0;
    memset(&s_pending, 0, sizeof(s_pending));
    s_pending_active = false;
    /* Playback and its sinks do not write flash or run with cache disabled.
     * Keep this persistent 8 KB stack in PSRAM; internal RAM is scarce. */
    if (s_play_sem == NULL) {
        s_play_sem = xSemaphoreCreateBinary();
        if (s_play_sem == NULL) return ESP_ERR_NO_MEM;
    }
    if (s_play_task == NULL &&
        xTaskCreatePinnedToCoreWithCaps(play_task, "visitor_play", PLAY_TASK_STACK_BYTES,
                                NULL, PLAY_TASK_PRIORITY, &s_play_task,
                                PLAY_TASK_CORE, STORE_CAPS) != pdPASS) {
        s_play_task = NULL;
        ESP_LOGE(TAG, "playback task could not be created: records will not play");
    }
    ESP_LOGI(TAG, "visitor store: %u records, %u KB + %u frames per record, thumb %ux%u",
             (unsigned)APP_VISITOR_RECORDS,
             (unsigned)(APP_VISITOR_RECORD_MAX_BYTES / 1024U),
             (unsigned)APP_VISITOR_RECORD_MAX_FRAMES,
             (unsigned)VISITOR_THUMB_W, (unsigned)VISITOR_THUMB_H);

    /* Snapshot thumbnails: fixed slots for the life of the store. */
    memset(s_snaps, 0, sizeof(s_snaps));
    s_snap_count = 0;
    for (int i = 0; i < (int)APP_VISITOR_SNAPSHOTS; i++) {
        if (s_snap_thumbs[i] == NULL) {
            s_snap_thumbs[i] = (uint16_t *)heap_caps_malloc(THUMB_BYTES, STORE_CAPS);
        }
        if (s_snap_thumbs[i] == NULL) {
            ESP_LOGE(TAG, "no PSRAM for snapshot thumbnail %d: snapshots disabled", i);
            for (int j = 0; j < i; j++) {
                heap_caps_free(s_snap_thumbs[j]);
                s_snap_thumbs[j] = NULL;
            }
            break;
        }
    }
    ESP_LOGI(TAG, "motion snapshots: %u slots%s", (unsigned)APP_VISITOR_SNAPSHOTS,
             s_snap_thumbs[0] ? "" : " (unavailable)");
    return ESP_OK;
}

void visitor_store_set_thumb_fn(visitor_thumb_fn_t fn)
{
    s_thumb_fn = fn;
}

void visitor_store_set_playback_sinks(visitor_frame_sink_t frame,
                                      visitor_audio_sink_t audio,
                                      visitor_play_done_t done)
{
    s_frame_sink = frame;
    s_audio_sink = audio;
    s_done_sink = done;
}

/* ---- recording ---- */

bool visitor_store_begin(void)
{
    if (s_mutex == NULL) return false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_pending_active) {
        /* Should not happen (the session machine is single-flight); drop the
         * stale one rather than leak it. */
        ESP_LOGW(TAG, "begin: a pending record was still open, dropping it");
        record_free(&s_pending);
        s_pending_active = false;
    }

    record_t r;
    memset(&r, 0, sizeof(r));
    r.frames = (frame_entry_t *)heap_caps_malloc(
        APP_VISITOR_RECORD_MAX_FRAMES * sizeof(frame_entry_t), STORE_CAPS);
    r.data = (uint8_t *)heap_caps_malloc(APP_VISITOR_RECORD_MAX_BYTES, STORE_CAPS);
    r.thumb = (uint16_t *)heap_caps_malloc(THUMB_BYTES, STORE_CAPS);
    if (r.frames == NULL || r.data == NULL || r.thumb == NULL) {
        ESP_LOGW(TAG, "begin: no PSRAM for a record (%u KB), ring goes unrecorded",
                 (unsigned)(APP_VISITOR_RECORD_MAX_BYTES / 1024U));
        record_free(&r);
        xSemaphoreGive(s_mutex);
        return false;
    }
    r.frame_cap = APP_VISITOR_RECORD_MAX_FRAMES;
    r.data_cap = APP_VISITOR_RECORD_MAX_BYTES;
    r.start_uptime_ms = now_ms();
    r.valid = true;
    s_pending = r;
    s_pending_active = true;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "recording started");
    return true;
}

bool visitor_store_recording(void)
{
    return s_pending_active;
}

void visitor_store_push(const uint8_t *jpeg, size_t jpeg_len,
                        const uint8_t *audio, size_t audio_len,
                        uint16_t audio_seq)
{
    if (s_mutex == NULL || jpeg == NULL || jpeg_len == 0) return;
    if (audio == NULL) audio_len = 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    record_t *r = &s_pending;
    if (!s_pending_active) {
        xSemaphoreGive(s_mutex);
        return;
    }
    if (r->frame_count >= r->frame_cap ||
        r->data_len + jpeg_len + audio_len > r->data_cap) {
        if (!r->full_logged) {
            ESP_LOGW(TAG, "record full at %lu frames / %u bytes, rest of the ring dropped",
                     (unsigned long)r->frame_count, (unsigned)r->data_len);
            r->full_logged = true;
        }
        xSemaphoreGive(s_mutex);
        return;
    }
    const uint32_t now = now_ms();
    if (r->first_frame_ms == 0) {
        r->first_frame_ms = now ? now : 1;
    }
    frame_entry_t *f = &r->frames[r->frame_count];
    f->t_ms = now - r->first_frame_ms;
    f->jpeg_off = (uint32_t)r->data_len;
    f->jpeg_len = (uint32_t)jpeg_len;
    f->audio_len = (uint32_t)audio_len;
    f->audio_seq = audio_seq;
    memcpy(r->data + r->data_len, jpeg, jpeg_len);
    r->data_len += jpeg_len;
    if (audio_len) {
        memcpy(r->data + r->data_len, audio, audio_len);
        r->data_len += audio_len;
    }
    r->frame_count++;
    r->duration_ms = f->t_ms;
    xSemaphoreGive(s_mutex);
}

void visitor_store_offer_thumbnail(const uint16_t *rgb565, uint32_t w, uint32_t h)
{
    if (s_mutex == NULL || rgb565 == NULL || s_thumb_fn == NULL) return;
    if (!s_pending_active) return;   /* cheap pre-check, re-checked under the lock */
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_pending_active && !s_pending.thumb_ready && s_pending.thumb != NULL) {
        s_thumb_fn(rgb565, w, h, s_pending.thumb);
        s_pending.thumb_ready = true;
    }
    xSemaphoreGive(s_mutex);
}

bool visitor_store_commit(void)
{
    if (s_mutex == NULL) return false;
    /* Nothing may be reading the record about to be evicted. Playback cannot
     * normally be running here (a ring stops it first), so this is a guard. */
    visitor_store_stop();

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (!s_pending_active) {
        xSemaphoreGive(s_mutex);
        return false;
    }
    record_t r = s_pending;
    memset(&s_pending, 0, sizeof(s_pending));
    s_pending_active = false;

    if (r.frame_count == 0) {
        ESP_LOGI(TAG, "commit: no frame arrived, nothing kept");
        record_free(&r);
        xSemaphoreGive(s_mutex);
        return false;
    }

    /* Give back what the reservation did not use. A shrink stays in place
     * with this allocator; if it ever failed the original block is kept. */
    uint8_t *data = (uint8_t *)heap_caps_realloc(r.data, r.data_len, STORE_CAPS);
    if (data != NULL) {
        r.data = data;
        r.data_cap = r.data_len;
    }
    frame_entry_t *frames = (frame_entry_t *)heap_caps_realloc(
        r.frames, r.frame_count * sizeof(frame_entry_t), STORE_CAPS);
    if (frames != NULL) {
        r.frames = frames;
        r.frame_cap = r.frame_count;
    }
    if (!r.thumb_ready) {
        heap_caps_free(r.thumb);
        r.thumb = NULL;
    }

    /* Newest first: the last slot goes, everything else moves down one. */
    if (s_records[APP_VISITOR_RECORDS - 1].valid) {
        ESP_LOGI(TAG, "evicting the oldest record (%lu frames, %u bytes)",
                 (unsigned long)s_records[APP_VISITOR_RECORDS - 1].frame_count,
                 (unsigned)s_records[APP_VISITOR_RECORDS - 1].data_len);
        record_free(&s_records[APP_VISITOR_RECORDS - 1]);
    }
    for (int i = (int)APP_VISITOR_RECORDS - 1; i > 0; i--) {
        s_records[i] = s_records[i - 1];
    }
    s_records[0] = r;
    if (s_count < (int)APP_VISITOR_RECORDS) s_count++;
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "record kept: %lu frames, %u bytes, %lu ms, thumb=%d, %d/%u stored",
             (unsigned long)r.frame_count, (unsigned)r.data_len,
             (unsigned long)r.duration_ms, r.thumb ? 1 : 0, s_count,
             (unsigned)APP_VISITOR_RECORDS);
    return true;
}

void visitor_store_discard(void)
{
    if (s_mutex == NULL) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_pending_active) {
        ESP_LOGI(TAG, "recording discarded (%lu frames)",
                 (unsigned long)s_pending.frame_count);
        record_free(&s_pending);
        s_pending_active = false;
    }
    xSemaphoreGive(s_mutex);
}

/* ---- reading ---- */

int visitor_store_count(void)
{
    return s_count;
}

bool visitor_store_get(int index, visitor_record_info_t *out)
{
    if (s_mutex == NULL || out == NULL || index < 0 ||
        index >= (int)APP_VISITOR_RECORDS) return false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const record_t *r = &s_records[index];
    const bool ok = r->valid;
    if (ok) {
        out->start_uptime_ms = r->start_uptime_ms;
        out->duration_ms = r->duration_ms;
        out->frame_count = r->frame_count;
        out->bytes = r->data_len;
        out->thumb = r->thumb;
    }
    xSemaphoreGive(s_mutex);
    return ok;
}

/* ---- playback ---- */

static void play_task(void *arg)
{
    (void)arg;
    for (;;) {
        xSemaphoreTake(s_play_sem, portMAX_DELAY);

        record_t rec;
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        const bool ok = s_play_index >= 0 && s_play_index < (int)APP_VISITOR_RECORDS &&
                        s_records[s_play_index].valid;
        if (ok) rec = s_records[s_play_index];
        xSemaphoreGive(s_mutex);
        if (!ok) {
            s_playing = false;
            if (s_done_sink) s_done_sink(false);
            continue;
        }

        ESP_LOGI(TAG, "playback %d: %lu frames, %lu ms", s_play_index,
                 (unsigned long)rec.frame_count, (unsigned long)rec.duration_ms);
        const uint32_t t0 = now_ms();
        bool completed = true;
        for (uint32_t i = 0; i < rec.frame_count; i++) {
            const frame_entry_t *f = &rec.frames[i];
            /* Wait for the frame's own arrival time, in short steps so a stop
             * is honoured within ~10 ms. */
            for (;;) {
                if (s_play_stop) break;
                const int32_t left = (int32_t)((t0 + f->t_ms) - now_ms());
                if (left <= 0) break;
                vTaskDelay(ticks_min_1(left > 10 ? 10U : (uint32_t)left));
            }
            if (s_play_stop) {
                completed = false;
                break;
            }
            /* Audio first, as it arrived: it rides ahead of the picture into
             * the jitter buffer just like the live stream. */
            if (f->audio_len && s_audio_sink) {
                s_audio_sink(rec.data + f->jpeg_off + f->jpeg_len, f->audio_len,
                             f->audio_seq);
            }
            if (s_frame_sink) {
                s_frame_sink(rec.data + f->jpeg_off, f->jpeg_len);
            }
        }
        if (completed) {
            /* Let the last frame reach the panel before the page changes. */
            vTaskDelay(ticks_min_1(150));
        }
        ESP_LOGI(TAG, "playback %s", completed ? "complete" : "stopped");
        /* Released BEFORE the done sink: stop() may be waiting on this from
         * under the UI lock the sink is about to take. */
        s_playing = false;
        if (s_done_sink) s_done_sink(completed);
    }
}

bool visitor_store_play(int index)
{
    if (s_mutex == NULL || s_play_sem == NULL || s_play_task == NULL) return false;
    if (s_playing) return false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const bool ok = index >= 0 && index < (int)APP_VISITOR_RECORDS &&
                    s_records[index].valid;
    if (ok) {
        s_play_index = index;
        s_play_stop = false;
        s_playing = true;
    }
    xSemaphoreGive(s_mutex);
    if (!ok) return false;
    xSemaphoreGive(s_play_sem);
    return true;
}

void visitor_store_stop(void)
{
    if (!s_playing) return;
    s_play_stop = true;
    const uint32_t start = now_ms();
    while (s_playing && now_ms() - start < 1000U) {
        vTaskDelay(ticks_min_1(5));
    }
    if (s_playing) {
        ESP_LOGW(TAG, "playback did not stop in time");
    }
}

bool visitor_store_playing(void)
{
    return s_playing;
}

/* ---- motion snapshots ---- */

bool visitor_store_snapshot_add(const uint8_t *jpeg, size_t jpeg_len,
                                const uint16_t *rgb565, uint32_t w, uint32_t h)
{
    if (s_mutex == NULL || jpeg == NULL || jpeg_len == 0 || rgb565 == NULL ||
        s_thumb_fn == NULL || s_snap_thumbs[0] == NULL ||
        w != APP_IMAGE_OUTPUT_WIDTH || h != APP_IMAGE_OUTPUT_HEIGHT) {
        return false;
    }
    /* The copy is taken before the lock: only the caller's buffer is read. */
    uint8_t *copy = (uint8_t *)heap_caps_malloc(jpeg_len, STORE_CAPS);
    if (copy == NULL) {
        ESP_LOGW(TAG, "snapshot: no PSRAM for a %u byte JPEG, dropped",
                 (unsigned)jpeg_len);
        return false;
    }
    memcpy(copy, jpeg, jpeg_len);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    /* Newest first. The last slot's JPEG goes and its thumbnail buffer is
     * handed to the newcomer; the buffers of the survivors move with them. */
    snapshot_t *last = &s_snaps[APP_VISITOR_SNAPSHOTS - 1];
    uint16_t *thumb = last->thumb;
    if (last->valid) {
        ESP_LOGI(TAG, "evicting the oldest snapshot (%u bytes)",
                 (unsigned)last->jpeg_len);
        heap_caps_free(last->jpeg);
    } else {
        /* Unused slot: take the first thumbnail buffer nobody holds yet. */
        thumb = s_snap_thumbs[s_snap_count];
    }
    for (int i = (int)APP_VISITOR_SNAPSHOTS - 1; i > 0; i--) {
        s_snaps[i] = s_snaps[i - 1];
    }
    snapshot_t *s = &s_snaps[0];
    s->valid = true;
    s->uptime_ms = now_ms();
    s->jpeg = copy;
    s->jpeg_len = jpeg_len;
    s->thumb = thumb;
    s_thumb_fn(rgb565, w, h, s->thumb);
    if (s_snap_count < (int)APP_VISITOR_SNAPSHOTS) s_snap_count++;
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "snapshot kept: %u bytes, %d/%u stored", (unsigned)jpeg_len,
             s_snap_count, (unsigned)APP_VISITOR_SNAPSHOTS);
    return true;
}

int visitor_store_snapshot_count(void)
{
    return s_snap_count;
}

bool visitor_store_snapshot_get(int index, visitor_snapshot_info_t *out)
{
    if (s_mutex == NULL || out == NULL || index < 0 ||
        index >= (int)APP_VISITOR_SNAPSHOTS) return false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const snapshot_t *s = &s_snaps[index];
    const bool ok = s->valid;
    if (ok) {
        out->uptime_ms = s->uptime_ms;
        out->jpeg_len = s->jpeg_len;
        out->thumb = s->thumb;
    }
    xSemaphoreGive(s_mutex);
    return ok;
}

bool visitor_store_snapshot_show(int index)
{
    if (s_mutex == NULL || s_frame_sink == NULL || index < 0 ||
        index >= (int)APP_VISITOR_SNAPSHOTS) return false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const snapshot_t *s = &s_snaps[index];
    const bool ok = s->valid;
    if (ok) {
        /* Under the lock: an eviction on another task cannot free the JPEG
         * while the sink is still copying it. */
        s_frame_sink(s->jpeg, s->jpeg_len);
    }
    xSemaphoreGive(s_mutex);
    return ok;
}
