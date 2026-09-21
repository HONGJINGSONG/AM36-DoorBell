#include "audio_diagnostics.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "app_config.h"

namespace {
constexpr const char *TAG = "audio_diag";
constexpr int REC_SECONDS = 3;
constexpr size_t REC_BUF_BYTES = APP_AUDIO_SAMPLE_RATE_HZ * 2 * REC_SECONDS;
// Door chime geometry in frames (one frame = one stereo sample pair).
constexpr int kDoorbellNoteFrames = APP_AUDIO_SAMPLE_RATE_HZ * APP_DOORBELL_TONE_MS / 1000;
constexpr size_t kDoorbellGapFrames = APP_AUDIO_SAMPLE_RATE_HZ * APP_DOORBELL_GAP_MS / 1000;

int32_t abs16(int16_t v)
{
    return v < 0 ? -static_cast<int32_t>(v) : v;
}
} // namespace

esp_err_t AudioDiagnostics::init()
{
    rec_cap_ = REC_BUF_BYTES;
    rec_buf_ = static_cast<uint8_t *>(heap_caps_malloc(rec_cap_, MALLOC_CAP_SPIRAM));
    if (rec_buf_ == nullptr) {
        rec_buf_ = static_cast<uint8_t *>(heap_caps_malloc(rec_cap_, MALLOC_CAP_8BIT));
    }
    if (rec_buf_ == nullptr) {
        ESP_LOGE(TAG, "record buffer alloc %u failed", static_cast<unsigned>(rec_cap_));
        return ESP_ERR_NO_MEM;
    }

    audio_sem_ = xSemaphoreCreateBinary();
    if (audio_sem_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    apply_state();
    BaseType_t ok = xTaskCreate(task_trampoline, "audio", 4096, this, 6, nullptr);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "record buffer %u bytes @%p", static_cast<unsigned>(rec_cap_), rec_buf_);
    return ESP_OK;
}

void AudioDiagnostics::handle_button(bsp_btn_id_t id, bool pressed)
{
    if (intercom_active_) return;
    switch (id) {
    case BSP_BTN_USER1:
        if (pressed) {
            start_record("user1");
        } else {
            stop_record("user1");
        }
        break;

    default:
        break;
    }
}

void AudioDiagnostics::set_intercom_active(bool active)
{
    intercom_active_ = active;
    if (active && state_ != State::idle) {
        state_ = State::idle;
        apply_state();
        xSemaphoreGive(audio_sem_);
    }
}

void AudioDiagnostics::play_startup_chime()
{
#if APP_STARTUP_CHIME_ENABLE
    ESP_LOGI(TAG, "startup chime: %d/%d Hz, %d ms, amp=%d",
             APP_STARTUP_CHIME_FREQ1_HZ, APP_STARTUP_CHIME_FREQ2_HZ,
             APP_STARTUP_CHIME_TONE_MS, APP_STARTUP_CHIME_AMP);
    pa_enable_for_cue();
    play_soft_tone(APP_STARTUP_CHIME_FREQ1_HZ, APP_STARTUP_CHIME_TONE_MS,
                   APP_STARTUP_CHIME_AMP);
    play_silence(APP_STARTUP_CHIME_GAP_MS);
    play_soft_tone(APP_STARTUP_CHIME_FREQ2_HZ, APP_STARTUP_CHIME_TONE_MS,
                   APP_STARTUP_CHIME_AMP);
    play_silence(100);
    bsp_audio_pa_enable(false);
    ESP_LOGI(TAG, "startup chime done");
#else
    ESP_LOGI(TAG, "startup chime disabled");
#endif
}

bool AudioDiagnostics::request_doorbell_ring(uint32_t duration_ms, uint32_t delay_ms)
{
    // A call owns the I2S TX path; there must only ever be one writer.
    if (intercom_active_) {
        ESP_LOGI(TAG, "doorbell ring dropped: call in progress");
        return false;
    }
    // Ring or drop, never queue: a queued bell reads as stuck.
    if (state_ != State::idle) {
        ESP_LOGI(TAG, "doorbell ring dropped: still ringing");
        return false;
    }
    state_ = State::chime;
    // A stop still set belongs to the previous ring; clear it under the idle
    // gate.
    ring_stop_req_ = false;
    ring_duration_ms_ = duration_ms;
    ring_delay_ms_ = delay_ms;
    ring_requested_ = true;
    xSemaphoreGive(audio_sem_);
    return true;
}

void AudioDiagnostics::stop_doorbell_ring()
{
    if (state_ != State::chime) return;   // nothing sounding, nothing to stop
    ring_stop_req_ = true;
}

void AudioDiagnostics::render_doorbell_note(int freq_hz, int16_t *dst)
{
    // Struck-tube model: fundamental + one inharmonic partial at 2.76x (second
    // mode of a free-free bar), which is what reads as a metal tube.
    constexpr float kPartialRatio = 2.76f;
    const float sr = static_cast<float>(APP_AUDIO_SAMPLE_RATE_HZ);
    const int total = kDoorbellNoteFrames;
    const int attack = APP_AUDIO_SAMPLE_RATE_HZ * APP_DOORBELL_ATTACK_MS / 1000;
    // The exponential decay does not reach zero inside the note; ramp the last
    // few ms to true zero to avoid a click.
    const int release = APP_AUDIO_SAMPLE_RATE_HZ * APP_DOORBELL_RELEASE_MS / 1000;
    const float w1 = 2.0f * 3.14159265f * static_cast<float>(freq_hz) / sr;
    const float w2 = w1 * kPartialRatio;
    const float partial = static_cast<float>(APP_DOORBELL_PARTIAL_PERCENT) / 100.0f;

    // Per-sample multiplier for exp(-t / decay): one multiply per sample.
    const float tau_samples = sr * static_cast<float>(APP_DOORBELL_DECAY_MS) / 1000.0f;
    const float decay_step = 1.0f - (1.0f / tau_samples);

    float env = 1.0f;
    for (int t = 0; t < total; t++) {
        const float ph = static_cast<float>(t);
        // The partial dies away faster than the fundamental on a real tube,
        // so square its envelope: bright on the strike, pure on the tail.
        const float body = sinf(w1 * ph) + partial * env * sinf(w2 * ph);
        float amp = env;
        if (t < attack) {
            amp *= static_cast<float>(t) / static_cast<float>(attack);
        }
        const int tail = total - 1 - t;
        if (release > 0 && tail < release) {
            amp *= static_cast<float>(tail) / static_cast<float>(release);
        }
        int32_t s = static_cast<int32_t>(body * amp * APP_DOORBELL_AMP);
        if (s > 32767) s = 32767;
        if (s < -32768) s = -32768;
        dst[2 * t] = static_cast<int16_t>(s);
        dst[2 * t + 1] = static_cast<int16_t>(s);
        env *= decay_step;
    }
}

bool AudioDiagnostics::ensure_ring_pcm()
{
    if (ring_pcm_ != nullptr) return true;

    // One ding-dong as ready-made stereo PCM, rendered once on the first ring
    // so the real-time loop only copies into the DMA ring.
    const size_t frames = kDoorbellNoteFrames + kDoorbellGapFrames + kDoorbellNoteFrames;
    const size_t bytes = frames * 2U * sizeof(int16_t);
    int16_t *buf = static_cast<int16_t *>(
        heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    const char *where = "PSRAM";
    if (buf == nullptr) {
        buf = static_cast<int16_t *>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
        where = "internal RAM";
    }
    if (buf == nullptr) {
        ESP_LOGE(TAG, "doorbell ring: no memory for %u bytes of PCM, cannot ring",
                 static_cast<unsigned>(bytes));
        return false;
    }
    render_doorbell_note(APP_DOORBELL_DING_HZ, buf);
    std::memset(buf + kDoorbellNoteFrames * 2U, 0,
                kDoorbellGapFrames * 2U * sizeof(int16_t));
    render_doorbell_note(APP_DOORBELL_DONG_HZ,
                         buf + (kDoorbellNoteFrames + kDoorbellGapFrames) * 2U);
    ring_pcm_ = buf;
    ring_pcm_frames_ = frames;
    ESP_LOGI(TAG, "doorbell ring rendered: %d/%d Hz, %d ms/note, amp=%d, %u bytes in %s",
             APP_DOORBELL_DING_HZ, APP_DOORBELL_DONG_HZ, APP_DOORBELL_TONE_MS,
             APP_DOORBELL_AMP, static_cast<unsigned>(bytes), where);
    return true;
}

bool AudioDiagnostics::play_ring_pcm()
{
    // One DMA descriptor per write, so a call taking the speaker is honoured
    // within 15 ms. A plain stop is not checked here: the ding-dong ends on
    // its own ramp, which keeps the stop click-free.
    constexpr size_t kChunkFrames = 240;
    size_t frame = 0;
    while (frame < ring_pcm_frames_) {
        if (state_ != State::chime) return false;
        size_t n = ring_pcm_frames_ - frame;
        if (n > kChunkFrames) n = kChunkFrames;
        const size_t bytes = n * 2U * sizeof(int16_t);
        size_t w = 0;
        esp_err_t err = bsp_audio_write(ring_pcm_ + frame * 2U, bytes, &w);
        if (err != ESP_OK || w != bytes) {
            ESP_LOGW(TAG, "doorbell write failed: %s, wrote %u/%u",
                     esp_err_to_name(err), static_cast<unsigned>(w),
                     static_cast<unsigned>(bytes));
            return false;
        }
        frame += n;
    }
    return true;
}

bool AudioDiagnostics::play_ring_silence(int duration_ms, uint32_t deadline_ms,
                                         bool honour_stop)
{
    static const int16_t zeros[128 * 2] = { 0 };
    const int total = APP_AUDIO_SAMPLE_RATE_HZ * duration_ms / 1000;
    int frame = 0;
    while (frame < total) {
        // A call taking the speaker always wins; a stop request and the
        // deadline only where the caller says they may (not in the drain).
        if (state_ != State::chime) return false;
        if (honour_stop && ring_stop_req_) return false;
        if (deadline_ms != 0 &&
            (int32_t)(deadline_ms - static_cast<uint32_t>(esp_timer_get_time() / 1000)) <= 0) {
            return false;
        }
        int batch = total - frame;
        if (batch > 128) batch = 128;
        size_t w = 0;
        esp_err_t err = bsp_audio_write(zeros, batch * 4, &w);
        if (err != ESP_OK || w != batch * 4) {
            ESP_LOGW(TAG, "ring silence write failed: %s, wrote %u/%u",
                     esp_err_to_name(err), static_cast<unsigned>(w),
                     static_cast<unsigned>(batch * 4));
            return false;
        }
        frame += batch;
    }
    return true;
}

void AudioDiagnostics::play_doorbell_ring(uint32_t duration_ms)
{
    if (!ensure_ring_pcm()) return;   // logged there; the task still hands the state back
    ESP_LOGI(TAG, "doorbell ring: up to %lums", static_cast<unsigned long>(duration_ms));
    const uint32_t start_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    const uint32_t deadline_ms = start_ms + duration_ms;

    // The PA stays on across the whole ring, pauses included, to keep its
    // switching transient out of the gaps.
    pa_enable_for_cue();
    unsigned repetitions = 0;
    while (!ring_cancelled() &&
           (int32_t)(deadline_ms - static_cast<uint32_t>(esp_timer_get_time() / 1000)) > 0) {
        // A stop or the deadline waits for the ding-dong to finish; only a
        // call cuts it (play_ring_pcm returns false, and so does a failed write).
        bool cut = false;
        for (int i = 0; i < APP_DOORBELL_REPEAT && !cut; i++) {
            cut = !play_ring_pcm();
            if (!cut && i + 1 < APP_DOORBELL_REPEAT) play_silence(APP_DOORBELL_GAP_MS * 3);
        }
        if (cut) break;
        repetitions++;
        // The pause is where a stop request or the deadline is honoured, so a
        // ring always ends on a finished ding-dong rather than in the middle.
        if (!play_ring_silence(APP_DOORBELL_RING_PAUSE_MS, deadline_ms, true)) break;
    }

    // bsp_audio_write() returns when samples reach the DMA ring, not the
    // speaker. Push a full ring of silence plus a margin so the output sits at
    // zero before the PA switches. The ring depth differs per board.
    uint32_t drain_ms = bsp_audio_tx_ring_ms();
    if (drain_ms == 0U) drain_ms = 100U;
    const bool drained =
        play_ring_silence(static_cast<int>(drain_ms + APP_PA_DRAIN_MARGIN_MS), 0, false);
    if (!drained || state_ != State::chime) {
        // A call took the speaker: leave the PA and the I2S path to it.
        ESP_LOGI(TAG, "doorbell ring cut by a call after %u repetition(s)", repetitions);
        return;
    }
    // What remains is the CST8302A's own turn-off transient (GPIO switched, no
    // ramp).
    bsp_audio_pa_enable(false);
    ESP_LOGI(TAG, "doorbell ring done: %u repetition(s), %s, drained %ums",
             repetitions, ring_stop_req_ ? "stopped" : "timed out",
             static_cast<unsigned>(drain_ms));
}

void AudioDiagnostics::task_trampoline(void *arg)
{
    static_cast<AudioDiagnostics *>(arg)->task();
}

void AudioDiagnostics::task()
{
    while (true) {
        xSemaphoreTake(audio_sem_, portMAX_DELAY);

        if (ring_requested_) {
            ring_requested_ = false;
            if (state_ == State::chime) {
                const uint32_t delay_ms = ring_delay_ms_;
                if (delay_ms > 0) {
                    vTaskDelay(pdMS_TO_TICKS(delay_ms));
                }
                // A call or a stop during the wait cancels the ring.
                if (!ring_cancelled()) {
                    play_doorbell_ring(ring_duration_ms_);
                } else {
                    ESP_LOGI(TAG, "doorbell ring skipped: cancelled before it started");
                }
                // Only hand the state back if a call has not taken it already.
                if (state_ == State::chime) {
                    state_ = State::idle;
                    apply_state();
                }
            }
            continue;
        }

        if (state_ == State::recording) {
            ESP_LOGI(TAG, "record cue: PA on, then beep");
            pa_enable_for_cue();
            beep_beep();
            ESP_LOGI(TAG, "record cue done: PA off, start capture");
            bsp_audio_pa_enable(false);
            drain_rx(APP_AUDIO_SAMPLE_RATE_HZ / 20);

            while (state_ == State::recording && !intercom_active_) {
                size_t room = rec_cap_ - rec_len_;
                if (room == 0) {
                    ESP_LOGW(TAG, "record buffer full");
                    state_ = State::playing;
                    apply_state();
                    break;
                }
                size_t got = record_mono_chunk(room);
                if (got == 0) break;
                rec_len_ += got;
            }
            log_record_stats();
        }

        if (state_ == State::playing) {
            ESP_LOGI(TAG, "playback cue: PA on, then beep");
            pa_enable_for_cue();
            beep_beep();
            play_mono_buffer();
            play_silence(80);
            ESP_LOGI(TAG, "playback done");
            state_ = State::idle;
            apply_state();
        }
    }
}

void AudioDiagnostics::apply_state()
{
    bsp_audio_set_volume(volume_);
    bsp_audio_pa_enable(state_ == State::playing);

    bool rec = state_ == State::recording;
    bool play = state_ == State::playing;
    // Green stays off at idle: an always-lit LED is a constant drain.
    bsp_led_set(false, false, rec || play);
}

void AudioDiagnostics::start_record(const char *source)
{
    if (state_ != State::idle) return;
    rec_len_ = 0;
    reset_record_stats();
    state_ = State::recording;
    ESP_LOGI(TAG, "%s down -> local record diagnostic", source);
    apply_state();
    xSemaphoreGive(audio_sem_);
}

void AudioDiagnostics::stop_record(const char *source)
{
    if (state_ != State::recording) return;
    state_ = State::playing;
    ESP_LOGI(TAG, "%s up -> local playback diagnostic (%u bytes)",
             source, static_cast<unsigned>(rec_len_));
    apply_state();
    xSemaphoreGive(audio_sem_);
}

void AudioDiagnostics::set_volume_delta(int delta)
{
    int next = static_cast<int>(volume_) + delta;
    if (next < 0) next = 0;
    if (next > BSP_AUDIO_VOLUME_MAX) next = BSP_AUDIO_VOLUME_MAX;
    volume_ = static_cast<uint8_t>(next);
    ESP_LOGI(TAG, "volume -> %u/%d", volume_, BSP_AUDIO_VOLUME_MAX);
    apply_state();
}

void AudioDiagnostics::reset_record_stats()
{
    rec_left_min_ = 0;
    rec_left_max_ = 0;
    rec_right_min_ = 0;
    rec_right_max_ = 0;
    rec_left_sum_abs_ = 0;
    rec_right_sum_abs_ = 0;
    rec_frame_count_ = 0;
}

size_t AudioDiagnostics::record_mono_chunk(size_t room)
{
    int16_t stereo[APP_AUDIO_IO_CHUNK_BYTES / sizeof(int16_t)];
    size_t got = 0;
    esp_err_t err = bsp_audio_read(stereo, sizeof(stereo), &got);
    if (err != ESP_OK || got < 4) {
        ESP_LOGW(TAG, "record read failed: %s, got=%u",
                 esp_err_to_name(err), static_cast<unsigned>(got));
        return 0;
    }

    auto *dst = reinterpret_cast<int16_t *>(rec_buf_ + rec_len_);
    size_t frames = got / 4;
    size_t max_frames = room / sizeof(int16_t);
    if (frames > max_frames) frames = max_frames;

    for (size_t i = 0; i < frames; i++) {
        int16_t left = stereo[2 * i];
        int16_t right = stereo[2 * i + 1];
        if (left < rec_left_min_) rec_left_min_ = left;
        if (left > rec_left_max_) rec_left_max_ = left;
        if (right < rec_right_min_) rec_right_min_ = right;
        if (right > rec_right_max_) rec_right_max_ = right;
        rec_left_sum_abs_ += static_cast<uint32_t>(abs16(left));
        rec_right_sum_abs_ += static_cast<uint32_t>(abs16(right));
        dst[i] = (abs16(left) >= abs16(right)) ? left : right;
    }
    rec_frame_count_ += frames;

    return frames * sizeof(int16_t);
}

void AudioDiagnostics::play_mono_buffer()
{
    int16_t stereo[256 * 2];
    const auto *src = reinterpret_cast<const int16_t *>(rec_buf_);
    size_t samples = rec_len_ / sizeof(int16_t);
    size_t pos = 0;
    size_t total_written = 0;

    ESP_LOGI(TAG, "play mono start: samples=%u bytes=%u",
             static_cast<unsigned>(samples), static_cast<unsigned>(rec_len_));

    while (pos < samples && !intercom_active_) {
        size_t batch = samples - pos;
        if (batch > 256) batch = 256;

        for (size_t i = 0; i < batch; i++) {
            int16_t s = src[pos + i];
            stereo[2 * i] = s;
            stereo[2 * i + 1] = s;
        }

        size_t sent = 0;
        esp_err_t err = bsp_audio_write(stereo, batch * 4, &sent);
        if (err != ESP_OK || sent == 0) {
            ESP_LOGW(TAG, "playback write failed: %s, pos=%u",
                     esp_err_to_name(err), static_cast<unsigned>(pos));
            break;
        }
        total_written += sent;
        pos += sent / 4;
    }
    ESP_LOGI(TAG, "play mono done: wrote=%u bytes", static_cast<unsigned>(total_written));
}

void AudioDiagnostics::log_record_stats()
{
    const auto *samples = reinterpret_cast<const int16_t *>(rec_buf_);
    size_t count = rec_len_ / sizeof(int16_t);
    int16_t min = 0;
    int16_t max = 0;
    uint32_t sum_abs = 0;

    for (size_t i = 0; i < count; i++) {
        int16_t s = samples[i];
        if (s < min) min = s;
        if (s > max) max = s;
        sum_abs += static_cast<uint32_t>(abs16(s));
    }

    uint32_t avg_abs = count ? sum_abs / count : 0;
    ESP_LOGI(TAG, "record stats: samples=%u min=%d max=%d avg_abs=%u",
             static_cast<unsigned>(count), min, max, static_cast<unsigned>(avg_abs));
    ESP_LOGI(TAG, "record raw channels: frames=%u L[min=%d max=%d avg_abs=%u] R[min=%d max=%d avg_abs=%u]",
             static_cast<unsigned>(rec_frame_count_),
             rec_left_min_, rec_left_max_,
             static_cast<unsigned>(rec_frame_count_ ? rec_left_sum_abs_ / rec_frame_count_ : 0),
             rec_right_min_, rec_right_max_,
             static_cast<unsigned>(rec_frame_count_ ? rec_right_sum_abs_ / rec_frame_count_ : 0));
}

void AudioDiagnostics::drain_rx(int frames)
{
    uint8_t scratch[APP_AUDIO_IO_CHUNK_BYTES];
    int left = frames * 4;
    while (left > 0) {
        size_t got = 0;
        int want = left < static_cast<int>(sizeof(scratch)) ? left : static_cast<int>(sizeof(scratch));
        if (bsp_audio_read(scratch, want, &got) != ESP_OK || got == 0) break;
        left -= static_cast<int>(got);
    }
}

void AudioDiagnostics::pa_enable_for_cue()
{
    esp_err_t err = bsp_audio_pa_enable(true);
    ESP_LOGI(TAG, "PA enable for cue -> %s", esp_err_to_name(err));
    vTaskDelay(pdMS_TO_TICKS(APP_PA_SETTLE_MS));
}

void AudioDiagnostics::play_tone(int freq_hz, int duration_ms)
{
    const int period = APP_AUDIO_SAMPLE_RATE_HZ / freq_hz;
    const int half = period / 2;
    const int total = APP_AUDIO_SAMPLE_RATE_HZ * duration_ms / 1000;
    int16_t buf[128 * 2];
    int frame = 0;

    while (frame < total) {
        int batch = total - frame;
        if (batch > 128) batch = 128;
        for (int i = 0; i < batch; i++) {
            int pos = (frame + i) % period;
            int16_t s = (pos < half) ? APP_BEEP_AMP : -APP_BEEP_AMP;
            buf[2 * i] = s;
            buf[2 * i + 1] = s;
        }
        size_t w = 0;
        esp_err_t err = bsp_audio_write(buf, batch * 4, &w);
        if (err != ESP_OK || w != batch * 4) {
            ESP_LOGW(TAG, "tone write failed: %s, wrote %u/%u",
                     esp_err_to_name(err), static_cast<unsigned>(w), static_cast<unsigned>(batch * 4));
            break;
        }
        frame += batch;
    }
}

void AudioDiagnostics::play_silence(int duration_ms)
{
    static const int16_t zeros[128 * 2] = { 0 };
    const int total = APP_AUDIO_SAMPLE_RATE_HZ * duration_ms / 1000;
    int frame = 0;
    while (frame < total) {
        int batch = total - frame;
        if (batch > 128) batch = 128;
        size_t w = 0;
        esp_err_t err = bsp_audio_write(zeros, batch * 4, &w);
        if (err != ESP_OK || w != batch * 4) {
            ESP_LOGW(TAG, "silence write failed: %s, wrote %u/%u",
                     esp_err_to_name(err), static_cast<unsigned>(w), static_cast<unsigned>(batch * 4));
            break;
        }
        frame += batch;
    }
}

void AudioDiagnostics::play_soft_tone(int freq_hz, int duration_ms, int amp)
{
    const int period = APP_AUDIO_SAMPLE_RATE_HZ / freq_hz;
    const int total = APP_AUDIO_SAMPLE_RATE_HZ * duration_ms / 1000;
    int16_t buf[128 * 2];
    int frame = 0;

    while (frame < total) {
        int batch = total - frame;
        if (batch > 128) batch = 128;

        for (int i = 0; i < batch; i++) {
            int t = frame + i;
            int pos = t % period;
            int quarter = period / 4;
            if (quarter <= 0) quarter = 1;

            int tri;
            if (pos < quarter) {
                tri = (amp * pos) / quarter;
            } else if (pos < 3 * quarter) {
                tri = amp - (2 * amp * (pos - quarter)) / (2 * quarter);
            } else {
                tri = -amp + (amp * (pos - 3 * quarter)) / quarter;
            }

            int fade = t;
            int tail = total - 1 - t;
            if (tail < fade) fade = tail;
            const int fade_len = APP_AUDIO_SAMPLE_RATE_HZ / 100;
            if (fade > fade_len) fade = fade_len;
            if (fade < 0) fade = 0;

            int16_t s = static_cast<int16_t>((tri * fade) / fade_len);
            buf[2 * i] = s;
            buf[2 * i + 1] = s;
        }

        size_t w = 0;
        esp_err_t err = bsp_audio_write(buf, batch * 4, &w);
        if (err != ESP_OK || w != batch * 4) {
            ESP_LOGW(TAG, "soft tone write failed: %s, wrote %u/%u",
                     esp_err_to_name(err), static_cast<unsigned>(w), static_cast<unsigned>(batch * 4));
            break;
        }
        frame += batch;
    }
}

void AudioDiagnostics::beep_beep()
{
    play_tone(APP_BEEP_FREQ_HZ, APP_BEEP_ON_MS);
    play_silence(APP_BEEP_GAP_MS);
    play_tone(APP_BEEP_FREQ_HZ, APP_BEEP_ON_MS);
    play_silence(APP_BEEP_TAIL_MS);
}
