#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "bsp.h"

#include <stdint.h>

class AudioDiagnostics {
public:
    esp_err_t init();
    void handle_button(bsp_btn_id_t id, bool pressed);
    void play_startup_chime();
    void set_intercom_active(bool active);

    // Ring the two-tone door chime, repeated for up to `duration_ms` or until
    // stop_doorbell_ring(). Safe to call from another task (the radio task
    // calls it straight out of packet handling): it only hands the work to the
    // audio task, which is the single owner of the I2S TX path. A request that
    // lands while a ring is still sounding is DROPPED, not queued, so the bell
    // can never outlast the pressing. Also dropped while a call is up, because
    // the intercom playback owns that path then. Returns whether the request
    // was taken; `delay_ms` holds the ring back that long first (see
    // APP_DOORBELL_PREEMPT_CHIME_DELAY_MS for why a caller would want that).
    bool request_doorbell_ring(uint32_t duration_ms, uint32_t delay_ms = 0);
    // End a ring early (the visitor session is over). Any task. The ring
    // finishes the note it is on, then drains and mutes as it would at the
    // end of its own time. A call taking the speaker does not go through
    // here: set_intercom_active() cuts the ring outright.
    void stop_doorbell_ring();

private:
    enum class State {
        idle,
        recording,
        playing,
        chime,
    };

    static void task_trampoline(void *arg);

    void task();

    volatile bool intercom_active_ = false;
    void apply_state();
    void start_record(const char *source);
    void stop_record(const char *source);
    void set_volume_delta(int delta);

    void reset_record_stats();
    size_t record_mono_chunk(size_t room);
    void play_mono_buffer();
    void log_record_stats();
    void drain_rx(int frames);

    void pa_enable_for_cue();
    void play_tone(int freq_hz, int duration_ms);
    void play_silence(int duration_ms);
    void play_soft_tone(int freq_hz, int duration_ms, int amp);
    void play_doorbell_ring(uint32_t duration_ms);
    // The ring's ding-dong is rendered ONCE (first ring) into ring_pcm_ and
    // played from there, so the real-time path is a copy, not a synthesis.
    bool ensure_ring_pcm();
    bool play_ring_pcm();
    static void render_doorbell_note(int freq_hz, int16_t *dst);
    // Silence that gives up as soon as a call takes the speaker, and - where
    // `honour_stop` / a non-zero `deadline_ms` say so - on a stop request or
    // when the deadline passes; returns whether it ran its full length.
    bool play_ring_silence(int duration_ms, uint32_t deadline_ms, bool honour_stop);
    bool ring_cancelled() const { return state_ != State::chime || ring_stop_req_; }
    void beep_beep();

    SemaphoreHandle_t audio_sem_ = nullptr;
    uint8_t *rec_buf_ = nullptr;
    size_t rec_cap_ = 0;
    size_t rec_len_ = 0;
    volatile uint8_t volume_ = 96;  /* DAC level, +6.5 dB = gateway Volume step 12 */
    volatile State state_ = State::idle;
    // A ring request is a one-shot handed to the task; the state alone cannot
    // carry it, because set_intercom_active() may knock the state back to idle
    // before the task looks. The stop request is likewise a flag the task
    // polls between notes, so a ring can be ended from any task.
    volatile bool ring_requested_ = false;
    volatile bool ring_stop_req_ = false;
    volatile uint32_t ring_duration_ms_ = 0;
    volatile uint32_t ring_delay_ms_ = 0;
    // One rendered ding-dong, stereo int16, PSRAM (see ensure_ring_pcm).
    int16_t *ring_pcm_ = nullptr;
    size_t ring_pcm_frames_ = 0;

    int16_t rec_left_min_ = 0;
    int16_t rec_left_max_ = 0;
    int16_t rec_right_min_ = 0;
    int16_t rec_right_max_ = 0;
    uint32_t rec_left_sum_abs_ = 0;
    uint32_t rec_right_sum_abs_ = 0;
    size_t rec_frame_count_ = 0;
};
