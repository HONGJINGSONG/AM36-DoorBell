#pragma once

#include <cstddef>
#include <cstdint>

#include "app_config.h"

/* Feedback killer for the one-way A/V playback on the gateway. See the
 * APP_AV_HOWL_* block in app_config.h for the loop it protects against and the
 * calibration behind the thresholds.
 *
 * Per 10 ms frame it measures two features of the DECODED pcm:
 *   clip_percent - share of samples at or beyond APP_AV_HOWL_CLIP_LEVEL
 *   rms          - root-mean-square amplitude
 * and runs a two-state machine with hysteresis on both axes plus a hard cap on
 * the mute length. While muted the caller zeroes the frame; that opens the
 * loop, the node's microphone stops hearing the speaker, and the features fall
 * back under the OFF limits by themselves one round trip later.
 *
 * No allocation, no floating point, ~2 multiplies per sample: it runs inside
 * the playback task on core 0 next to the Opus decoder. */
class HowlSuppressor {
public:
    struct Features {
        uint32_t clip_percent = 0;
        uint32_t rms = 0;
    };

    /* Feed one decoded frame. Returns true when the frame must be muted. */
    bool process(const int16_t *pcm, size_t samples)
    {
        last_ = measure(pcm, samples);
        const bool over_on = last_.clip_percent >= APP_AV_HOWL_ON_CLIP_PERCENT &&
                             last_.rms >= APP_AV_HOWL_ON_RMS;
        const bool under_off = last_.clip_percent <= APP_AV_HOWL_OFF_CLIP_PERCENT &&
                               last_.rms <= APP_AV_HOWL_OFF_RMS;

        if (!muted_) {
            on_count_ = over_on ? on_count_ + 1U : 0U;
            if (on_count_ >= APP_AV_HOWL_ON_FRAMES) {
                muted_ = true;
                on_count_ = 0;
                off_count_ = 0;
                muted_frames_ = 0;
                mute_events_++;
            }
        } else {
            muted_frames_++;
            off_count_ = under_off ? off_count_ + 1U : 0U;
            // The cap is what keeps a loud but honest visitor audible: without
            // it a voice that never drops under OFF_RMS would stay muted for
            // as long as it talks.
            if (off_count_ >= APP_AV_HOWL_OFF_FRAMES ||
                muted_frames_ >= APP_AV_HOWL_MAX_MUTE_FRAMES) {
                muted_ = false;
                on_count_ = 0;
                off_count_ = 0;
            }
        }
        return muted_;
    }

    bool muted() const { return muted_; }
    const Features &last() const { return last_; }
    uint32_t mute_events() const { return mute_events_; }
    uint32_t muted_frames() const { return muted_frames_; }

    /* Forget the state, keep the event counter: a playback restart is a new
     * timeline, not a new device. */
    void reset()
    {
        muted_ = false;
        on_count_ = 0;
        off_count_ = 0;
        muted_frames_ = 0;
        last_ = {};
    }

    static Features measure(const int16_t *pcm, size_t samples)
    {
        Features f;
        if (pcm == nullptr || samples == 0) return f;
        uint32_t clipped = 0;
        uint64_t sum_sq = 0;
        for (size_t i = 0; i < samples; ++i) {
            const int32_t s = pcm[i];
            const int32_t a = s < 0 ? -s : s;
            if (a >= APP_AV_HOWL_CLIP_LEVEL) clipped++;
            // s*s <= 2^30, fits int32; the sum over a frame does not, hence 64.
            sum_sq += static_cast<uint32_t>(s * s);
        }
        f.clip_percent = static_cast<uint32_t>((clipped * 100U + samples / 2U) / samples);
        f.rms = isqrt32(static_cast<uint32_t>(sum_sq / samples));
        return f;
    }

private:
    static uint32_t isqrt32(uint32_t x)
    {
        uint32_t r = 0;
        uint32_t bit = 1UL << 30;
        while (bit > x) bit >>= 2;
        while (bit != 0) {
            if (x >= r + bit) {
                x -= r + bit;
                r = (r >> 1) + bit;
            } else {
                r >>= 1;
            }
            bit >>= 2;
        }
        return r;
    }

    bool muted_ = false;
    uint32_t on_count_ = 0;
    uint32_t off_count_ = 0;
    uint32_t muted_frames_ = 0;
    uint32_t mute_events_ = 0;
    Features last_;
};
