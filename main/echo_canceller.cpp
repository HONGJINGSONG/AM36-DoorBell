#include "echo_canceller.hpp"

#include <cstring>

#include "app_config.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

namespace {

constexpr const char *TAG = "aec";

int16_t *allocate_psram_samples(size_t samples)
{
    return static_cast<int16_t *>(heap_caps_aligned_calloc(
        16, samples, sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
}

} // namespace

EchoCanceller::~EchoCanceller()
{
    release();
    if (aec_mutex_ != nullptr) vSemaphoreDelete(aec_mutex_);
    if (reference_mutex_ != nullptr) vSemaphoreDelete(reference_mutex_);
    aec_mutex_ = nullptr;
    reference_mutex_ = nullptr;
}

bool EchoCanceller::init()
{
    if (ready()) return true;

    if (aec_mutex_ == nullptr) aec_mutex_ = xSemaphoreCreateMutex();
    if (reference_mutex_ == nullptr) {
        reference_mutex_ = xSemaphoreCreateMutex();
    }
    if (aec_mutex_ == nullptr || reference_mutex_ == nullptr) {
        ESP_LOGE(TAG, "mutex allocation failed");
        release();
        return false;
    }

    aec_handle_ = aec_create(APP_AUDIO_SAMPLE_RATE_HZ,
                             APP_AFE_AEC_FILTER_LENGTH, 1,
                             AEC_MODE_FD_LOW_COST);
    if (aec_handle_ == nullptr) {
        ESP_LOGE(TAG, "official direct AEC create failed");
        release();
        return false;
    }

    /* Start with aggressive NLP: the freshly created adaptive filter has not
     * converged and its residual echo would otherwise loop into howling.
     * process_capture() relaxes NLP to the steady level after the warm-up. */
    aec_set_nlp_level(aec_handle_,
                      static_cast<aec_nlp_level_t>(APP_AFE_AEC_NLP_LEVEL_STARTUP));
    nlp_steady_ = false;
    nlp_processed_frames_ = 0;

    const int frame_samples = aec_get_chunksize(aec_handle_);
    const aec_config_t &config = aec_handle_->config;
    if (config.mode != AEC_MODE_FD_LOW_COST || frame_samples <= 0 ||
        config.sample_rate != static_cast<int>(APP_AUDIO_SAMPLE_RATE_HZ) ||
        config.filter_length != static_cast<int>(APP_AFE_AEC_FILTER_LENGTH) ||
        config.mic_num != 1 || config.ref_num != 1 || config.out_num != 1) {
        ESP_LOGE(TAG,
                 "unexpected direct AEC format: mode=%d rate=%d frame=%d "
                 "filter=%d mic=%d ref=%d out=%d",
                 static_cast<int>(config.mode), config.sample_rate,
                 frame_samples, config.filter_length, config.mic_num,
                 config.ref_num, config.out_num);
        release();
        return false;
    }

    frame_samples_ = static_cast<size_t>(frame_samples);
    if (frame_samples_ > kReferenceRingSamples) {
        ESP_LOGE(TAG, "direct AEC frame too large: frame=%u",
                 static_cast<unsigned>(frame_samples_));
        release();
        return false;
    }
    pending_capacity_ = frame_samples_ + APP_AUDIO_FRAME_SAMPLES;
    bridge_delay_samples_ =
        ((frame_samples_ + APP_AUDIO_FRAME_SAMPLES - 1U) /
         APP_AUDIO_FRAME_SAMPLES) * APP_AUDIO_FRAME_SAMPLES;
    output_capacity_ = bridge_delay_samples_ + frame_samples_;

    /* aec_process() consumes one frame_samples_ chunk per call, so the warm-up
     * duration in ms maps to a chunk count via the sample rate. */
    nlp_warmup_frames_ = static_cast<uint32_t>(
        (static_cast<uint64_t>(APP_AFE_AEC_NLP_WARMUP_MS) *
         APP_AUDIO_SAMPLE_RATE_HZ) / (1000ULL * frame_samples_));

    reference_ring_ = allocate_psram_samples(kReferenceRingSamples);
    mic_pending_ = allocate_psram_samples(pending_capacity_);
    reference_pending_ = allocate_psram_samples(pending_capacity_);
    aec_output_ = allocate_psram_samples(frame_samples_);
    output_ring_ = allocate_psram_samples(output_capacity_);
    if (reference_ring_ == nullptr || mic_pending_ == nullptr ||
        reference_pending_ == nullptr || aec_output_ == nullptr ||
        output_ring_ == nullptr) {
        ESP_LOGE(TAG, "PSRAM buffer allocation failed");
        release();
        return false;
    }

    reset_stream_buffers();
    ready_.store(true, std::memory_order_release);
    ESP_LOGI(TAG,
             "ESP-SR direct AEC ready: api=aec mode=FD_LOW_COST "
             "nlp=%d->%d warmup=%ums(%ufr) "
             "aec_frame=%u io_frame=%u bridge_delay=%lums "
             "reference=fifo max_lag=%ums(%u) filter=%u worker_tasks=0",
             static_cast<int>(APP_AFE_AEC_NLP_LEVEL_STARTUP),
             static_cast<int>(APP_AFE_AEC_NLP_LEVEL_STEADY),
             static_cast<unsigned>(APP_AFE_AEC_NLP_WARMUP_MS),
             static_cast<unsigned>(nlp_warmup_frames_),
             static_cast<unsigned>(frame_samples_),
             static_cast<unsigned>(APP_AUDIO_FRAME_SAMPLES),
             static_cast<unsigned long>(
                 bridge_delay_samples_ * 1000U / APP_AUDIO_SAMPLE_RATE_HZ),
             static_cast<unsigned>(kReferenceMaxLagSamples * 1000U /
                                   APP_AUDIO_SAMPLE_RATE_HZ),
             static_cast<unsigned>(kReferenceMaxLagSamples),
             static_cast<unsigned>(APP_AFE_AEC_FILTER_LENGTH));
    return true;
}

void EchoCanceller::deinit()
{
    release();
}

void EchoCanceller::release()
{
    ready_.store(false, std::memory_order_release);

    const bool have_aec_lock = aec_mutex_ != nullptr &&
        xSemaphoreTake(aec_mutex_, portMAX_DELAY) == pdTRUE;
    const bool have_reference_lock = reference_mutex_ != nullptr &&
        xSemaphoreTake(reference_mutex_, portMAX_DELAY) == pdTRUE;

    if (aec_handle_ != nullptr) aec_destroy(aec_handle_);
    aec_handle_ = nullptr;

    heap_caps_free(reference_ring_);
    heap_caps_free(mic_pending_);
    heap_caps_free(reference_pending_);
    heap_caps_free(aec_output_);
    heap_caps_free(output_ring_);
    reference_ring_ = nullptr;
    mic_pending_ = nullptr;
    reference_pending_ = nullptr;
    aec_output_ = nullptr;
    output_ring_ = nullptr;
    frame_samples_ = 0;
    pending_capacity_ = 0;
    pending_samples_ = 0;
    output_capacity_ = 0;
    output_read_ = 0;
    output_write_ = 0;
    output_samples_ = 0;
    bridge_delay_samples_ = 0;
    reference_read_ = 0;
    reference_write_ = 0;
    reference_samples_ = 0;
    reference_level_ = 0;
    reference_level_peak_ = 0;
    error_count_ = 0;
    reference_trim_count_ = 0;
    reference_overflow_count_ = 0;
    reference_underflow_count_ = 0;
    reference_wait_frames_ = 0;
    reference_started_ = false;

    if (have_reference_lock) xSemaphoreGive(reference_mutex_);
    if (have_aec_lock) xSemaphoreGive(aec_mutex_);
}

void EchoCanceller::reset()
{
    if (aec_mutex_ == nullptr ||
        xSemaphoreTake(aec_mutex_, portMAX_DELAY) != pdTRUE) return;
    if (!ready()) {
        xSemaphoreGive(aec_mutex_);
        return;
    }

    /* Re-arm the NLP warm-up: a re-used AEC handle keeps its converged filter,
     * but a fresh call still benefits from aggressive suppression until the
     * reference/mic alignment settles again. */
    aec_set_nlp_level(
        aec_handle_,
        static_cast<aec_nlp_level_t>(APP_AFE_AEC_NLP_LEVEL_STARTUP));
    nlp_steady_ = false;
    nlp_processed_frames_ = 0;

    if (xSemaphoreTake(reference_mutex_, portMAX_DELAY) == pdTRUE) {
        reset_stream_buffers();
        xSemaphoreGive(reference_mutex_);
    }
    xSemaphoreGive(aec_mutex_);
}

void EchoCanceller::rearm_nlp()
{
    if (aec_mutex_ == nullptr ||
        xSemaphoreTake(aec_mutex_, portMAX_DELAY) != pdTRUE) return;
    if (!ready()) {
        xSemaphoreGive(aec_mutex_);
        return;
    }

    /* Unlike reset(), this keeps the converged filter and the stream buffers:
     * only the residual-echo suppression goes back to the startup level for
     * another warm-up window. The filter is still valid across a playback gap,
     * but the reference/mic alignment is not, and the relaxed steady NLP has no
     * margin for what leaks through while that settles. */
    const bool was_steady = nlp_steady_;
    aec_set_nlp_level(
        aec_handle_,
        static_cast<aec_nlp_level_t>(APP_AFE_AEC_NLP_LEVEL_STARTUP));
    nlp_steady_ = false;
    nlp_processed_frames_ = 0;
    xSemaphoreGive(aec_mutex_);

    if (was_steady) {
        ESP_LOGI(TAG, "AEC NLP re-armed to startup level %d for %ums",
                 static_cast<int>(APP_AFE_AEC_NLP_LEVEL_STARTUP),
                 static_cast<unsigned>(APP_AFE_AEC_NLP_WARMUP_MS));
    }
}

EchoCanceller::ReferenceStats EchoCanceller::reference_stats(bool clear_peak)
{
    ReferenceStats stats;
    if (reference_mutex_ == nullptr ||
        xSemaphoreTake(reference_mutex_, portMAX_DELAY) != pdTRUE) {
        return stats;
    }
    stats.level = static_cast<uint32_t>(reference_level_);
    stats.peak = static_cast<uint32_t>(reference_level_peak_);
    stats.trims = reference_trim_count_;
    stats.underflows = reference_underflow_count_;
    stats.overflows = reference_overflow_count_;
    /* Restart the peak window from the current level rather than from zero, so
     * a standing lag stays visible in the next reporting period instead of
     * being reported as if it had just been cleared. */
    if (clear_peak) reference_level_peak_ = reference_level_;
    xSemaphoreGive(reference_mutex_);
    return stats;
}

void EchoCanceller::reset_stream_buffers()
{
    std::memset(reference_ring_, 0,
                kReferenceRingSamples * sizeof(int16_t));
    std::memset(mic_pending_, 0, pending_capacity_ * sizeof(int16_t));
    std::memset(reference_pending_, 0,
                pending_capacity_ * sizeof(int16_t));
    std::memset(aec_output_, 0, frame_samples_ * sizeof(int16_t));
    std::memset(output_ring_, 0, output_capacity_ * sizeof(int16_t));

    pending_samples_ = 0;
    output_read_ = 0;
    output_write_ = bridge_delay_samples_ % output_capacity_;
    output_samples_ = bridge_delay_samples_;
    reference_read_ = 0;
    reference_write_ = 0;
    reference_samples_ = 0;
    reference_level_ = 0;
    reference_level_peak_ = 0;
    error_count_ = 0;
    reference_trim_count_ = 0;
    reference_overflow_count_ = 0;
    reference_underflow_count_ = 0;
    reference_wait_frames_ = 0;
    reference_started_ = false;
}

void EchoCanceller::push_reference(const int16_t *pcm, size_t samples)
{
    if (pcm == nullptr || samples == 0 || reference_mutex_ == nullptr) return;
    if (xSemaphoreTake(reference_mutex_, portMAX_DELAY) != pdTRUE) return;
    if (!ready()) {
        xSemaphoreGive(reference_mutex_);
        return;
    }

    if (samples > kReferenceRingSamples) {
        pcm += samples - kReferenceRingSamples;
        samples = kReferenceRingSamples;
    }

    const size_t free_samples = kReferenceRingSamples - reference_samples_;
    if (samples > free_samples) {
        const size_t drop = samples - free_samples;
        reference_read_ = (reference_read_ + drop) % kReferenceRingSamples;
        reference_samples_ -= drop;
        ++reference_overflow_count_;
        if (reference_overflow_count_ == 1U ||
            (reference_overflow_count_ % 100U) == 0U) {
            ESP_LOGW(TAG,
                     "reference FIFO overflow: dropped=%u events=%lu",
                     static_cast<unsigned>(drop),
                     static_cast<unsigned long>(reference_overflow_count_));
        }
    }
    for (size_t i = 0; i < samples; ++i) {
        reference_ring_[reference_write_] = pcm[i];
        reference_write_ = (reference_write_ + 1U) % kReferenceRingSamples;
    }
    reference_samples_ += samples;

    /* Bound the reference lag. Dropping the oldest samples steps the reference
     * forward, i.e. back towards the mic timeline; keeping them would leave the
     * AEC permanently fed with stale reference it can no longer match against
     * the echo. See kReferenceMaxLagSamples for why the lag ratchets up in the
     * first place and how to retune this from the ref_* log fields. */
    if (reference_samples_ > kReferenceMaxLagSamples) {
        const size_t drop = reference_samples_ - kReferenceMaxLagSamples;
        reference_read_ = (reference_read_ + drop) % kReferenceRingSamples;
        reference_samples_ -= drop;
        ++reference_trim_count_;
        if (reference_trim_count_ == 1U ||
            (reference_trim_count_ % 100U) == 0U) {
            ESP_LOGW(TAG,
                     "reference lag trimmed: dropped=%u lag=%ums events=%lu",
                     static_cast<unsigned>(drop),
                     static_cast<unsigned>(kReferenceMaxLagSamples * 1000U /
                                           APP_AUDIO_SAMPLE_RATE_HZ),
                     static_cast<unsigned long>(reference_trim_count_));
        }
    }
    xSemaphoreGive(reference_mutex_);
}

void EchoCanceller::process_capture(int16_t *pcm, size_t samples)
{
    if (pcm == nullptr || samples == 0) return;
    if (aec_mutex_ == nullptr ||
        xSemaphoreTake(aec_mutex_, portMAX_DELAY) != pdTRUE) {
        std::memset(pcm, 0, samples * sizeof(int16_t));
        return;
    }
    if (!ready() || samples != APP_AUDIO_FRAME_SAMPLES ||
        pending_samples_ + samples > pending_capacity_) {
        if (ready()) {
            report_error("io_frame", static_cast<int>(samples));
        }
        std::memset(pcm, 0, samples * sizeof(int16_t));
        xSemaphoreGive(aec_mutex_);
        return;
    }

    std::memcpy(mic_pending_ + pending_samples_, pcm,
                samples * sizeof(int16_t));

    size_t reference_copied = 0;
    if (xSemaphoreTake(reference_mutex_, portMAX_DELAY) == pdTRUE) {
        /* Sampled before consuming: this is the lag actually applied to the AEC
         * on this frame, which is the number that decides whether the echo is
         * still inside the filter's reach. */
        reference_level_ = reference_samples_;
        if (reference_level_ > reference_level_peak_) {
            reference_level_peak_ = reference_level_;
        }
        reference_copied =
            reference_samples_ < samples ? reference_samples_ : samples;
        for (size_t i = 0; i < reference_copied; ++i) {
            reference_pending_[pending_samples_ + i] =
                reference_ring_[reference_read_];
            reference_read_ =
                (reference_read_ + 1U) % kReferenceRingSamples;
        }
        reference_samples_ -= reference_copied;
        xSemaphoreGive(reference_mutex_);
    }

    if (reference_copied < samples) {
        ++reference_wait_frames_;
        if (reference_started_) {
            ++reference_underflow_count_;
            if (reference_underflow_count_ == 1U ||
                (reference_underflow_count_ % 100U) == 0U) {
                ESP_LOGW(TAG,
                         "reference FIFO underflow: missing=%u events=%lu",
                         static_cast<unsigned>(samples - reference_copied),
                         static_cast<unsigned long>(
                             reference_underflow_count_));
            }
        }
        std::memset(reference_pending_ + pending_samples_ + reference_copied,
                    0, (samples - reference_copied) * sizeof(int16_t));
    } else if (!reference_started_) {
        reference_started_ = true;
        ESP_LOGI(TAG, "playback reference FIFO active after %lu capture frames",
                 static_cast<unsigned long>(reference_wait_frames_));
    }
    pending_samples_ += samples;

    while (pending_samples_ >= frame_samples_) {
        aec_process(aec_handle_, mic_pending_, reference_pending_,
                    aec_output_);
        append_output(aec_output_, frame_samples_);

        /* Relax NLP from the startup level once the adaptive filter has had the
         * warm-up window to converge, trading residual-echo strength for speech
         * clarity. Only crosses once per call (re-armed in reset()/init()). */
        if (!nlp_steady_ && ++nlp_processed_frames_ >= nlp_warmup_frames_) {
            aec_set_nlp_level(
                aec_handle_,
                static_cast<aec_nlp_level_t>(APP_AFE_AEC_NLP_LEVEL_STEADY));
            nlp_steady_ = true;
            ESP_LOGI(TAG, "AEC NLP relaxed to steady level %d after %lu frames",
                     static_cast<int>(APP_AFE_AEC_NLP_LEVEL_STEADY),
                     static_cast<unsigned long>(nlp_processed_frames_));
        }

        pending_samples_ -= frame_samples_;
        if (pending_samples_ != 0) {
            std::memmove(mic_pending_, mic_pending_ + frame_samples_,
                         pending_samples_ * sizeof(int16_t));
            std::memmove(reference_pending_,
                         reference_pending_ + frame_samples_,
                         pending_samples_ * sizeof(int16_t));
        }
    }

    if (output_samples_ < samples) {
        report_error("output_underflow", static_cast<int>(output_samples_));
        std::memset(pcm, 0, samples * sizeof(int16_t));
    } else {
        for (size_t i = 0; i < samples; ++i) {
            pcm[i] = output_ring_[output_read_];
            output_read_ = (output_read_ + 1U) % output_capacity_;
        }
        output_samples_ -= samples;
    }
    xSemaphoreGive(aec_mutex_);
}

void EchoCanceller::append_output(const int16_t *pcm, size_t samples)
{
    if (samples > output_capacity_) {
        pcm += samples - output_capacity_;
        samples = output_capacity_;
    }
    const size_t free_samples = output_capacity_ - output_samples_;
    if (samples > free_samples) {
        const size_t drop = samples - free_samples;
        output_read_ = (output_read_ + drop) % output_capacity_;
        output_samples_ -= drop;
        report_error("output_overflow", static_cast<int>(drop));
    }
    for (size_t i = 0; i < samples; ++i) {
        output_ring_[output_write_] = pcm[i];
        output_write_ = (output_write_ + 1U) % output_capacity_;
    }
    output_samples_ += samples;
}

void EchoCanceller::report_error(const char *operation, int result)
{
    ++error_count_;
    if (error_count_ == 1U || (error_count_ % 100U) == 0U) {
        ESP_LOGW(TAG, "direct AEC %s failed/result=%d count=%lu", operation,
                 result, static_cast<unsigned long>(error_count_));
    }
}
