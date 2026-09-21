#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_aec.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class EchoCanceller {
public:
    EchoCanceller() = default;
    ~EchoCanceller();

    EchoCanceller(const EchoCanceller &) = delete;
    EchoCanceller &operator=(const EchoCanceller &) = delete;

    /* Reference FIFO observability: `level` is the lag in samples at the last
     * capture frame, `peak` the largest since the previous read (cleared with
     * `clear_peak`), counters cumulative. The lag decides whether the AEC can
     * cancel at all. */
    struct ReferenceStats {
        uint32_t level = 0;
        uint32_t peak = 0;
        uint32_t trims = 0;
        uint32_t underflows = 0;
        uint32_t overflows = 0;
    };

    bool init();
    void deinit();
    bool ready() const { return ready_.load(std::memory_order_acquire); }
    void reset();
    void push_reference(const int16_t *pcm, size_t samples);
    void process_capture(int16_t *pcm, size_t samples);

    /* Re-arm the aggressive startup NLP without touching the converged filter.
     * Call whenever playback restarts and the alignment settles again. */
    void rearm_nlp();

    ReferenceStats reference_stats(bool clear_peak);

private:
    /* Hard ring capacity. This is only a backstop against a runaway producer;
     * the limit that actually governs alignment is kReferenceMaxLagSamples. */
    static constexpr size_t kReferenceRingSamples = 8192;

    /* High-water mark for the reference FIFO (100 ms at 16 kHz). The FIFO
     * level is the delay between the reference and its echo in the mic; the
     * adaptive filter is causal, so once the reference lags the echo nothing
     * is cancelled and the cross-device loop howls. Nothing regulates the lag
     * on its own: playback restarts push a whole jitter buffer in at once, and
     * the two boards' crystals drift a few hundred ppm apart.
     *
     * 100 ms = two FLRC voice packets (5 frames = 800 samples each), so the
     * healthy 0..800 sample sawtooth plus a packet of jitter clears it. Tune
     * only with log evidence: `ref_peak` pinned here while howling means too
     * loose; `ref_trim` climbing during clean speech means too tight. */
    static constexpr size_t kReferenceMaxLagSamples = 1600;

    void release();
    void append_output(const int16_t *pcm, size_t samples);
    void reset_stream_buffers();
    void report_error(const char *operation, int result);

    aec_handle_t *aec_handle_ = nullptr;
    SemaphoreHandle_t aec_mutex_ = nullptr;
    SemaphoreHandle_t reference_mutex_ = nullptr;

    int16_t *reference_ring_ = nullptr;
    int16_t *mic_pending_ = nullptr;
    int16_t *reference_pending_ = nullptr;
    int16_t *aec_output_ = nullptr;
    int16_t *output_ring_ = nullptr;

    size_t frame_samples_ = 0;
    size_t pending_capacity_ = 0;
    size_t pending_samples_ = 0;
    size_t output_capacity_ = 0;
    size_t output_read_ = 0;
    size_t output_write_ = 0;
    size_t output_samples_ = 0;
    size_t bridge_delay_samples_ = 0;
    size_t reference_read_ = 0;
    size_t reference_write_ = 0;
    size_t reference_samples_ = 0;
    size_t reference_level_ = 0;
    size_t reference_level_peak_ = 0;
    uint32_t error_count_ = 0;
    uint32_t reference_trim_count_ = 0;
    uint32_t reference_overflow_count_ = 0;
    uint32_t reference_underflow_count_ = 0;
    uint32_t reference_wait_frames_ = 0;
    bool reference_started_ = false;

    /* NLP warm-up: hold aggressive residual suppression while the adaptive
     * filter converges after each (re)start, then relax to the steady level. */
    uint32_t nlp_warmup_frames_ = 0;
    uint32_t nlp_processed_frames_ = 0;
    bool nlp_steady_ = false;

    std::atomic<bool> ready_{false};
};
