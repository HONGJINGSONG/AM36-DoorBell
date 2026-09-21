#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include "esp_heap_caps.h"
#include "app_config.h"

struct OpusFrameEntry {
    uint8_t len;
    uint8_t data[APP_OPUS_MAX_PACKET_BYTES];
};

class OpusRingBuf {
public:
    bool init(size_t max_frames)
    {
        buf_ = static_cast<OpusFrameEntry *>(
            heap_caps_malloc(max_frames * sizeof(OpusFrameEntry), MALLOC_CAP_SPIRAM));
        if (!buf_) return false;
        capacity_ = max_frames;
        std::memset(buf_, 0, max_frames * sizeof(OpusFrameEntry));
        return true;
    }

    // Single-producer / single-consumer. write() runs on the tx_task (producer)
    // and touches ONLY write_seq_/the slot it owns; drain()/snapshot() run on the
    // capture task (consumer) and touch ONLY read_seq_. write_seq_ is a monotonic
    // count of frames ever written, so the consumer detects producer overrun by
    // comparing sequence numbers — the producer never writes consumer state, so
    // there is no shared mutable cursor and no lock is needed.
    void write(const uint8_t *data, uint8_t len)
    {
        if (!buf_) return;
        size_t slot = write_seq_ % capacity_;
        buf_[slot].len = len;
        std::memcpy(buf_[slot].data, data, len);
        // Publish the frame only after its data is fully written, so a concurrent
        // consumer that observes the new write_seq_ always sees complete data.
        write_seq_++;
    }

    size_t snapshot(uint8_t *out, size_t max_bytes) const
    {
        uint32_t w = write_seq_;
        uint32_t avail = (w < (uint32_t)capacity_) ? w : (uint32_t)capacity_;
        uint32_t start = w - avail;
        size_t total = 0;
        for (uint32_t i = 0; i < avail; i++) {
            const OpusFrameEntry &e = buf_[(start + i) % capacity_];
            if (e.len == 0) continue;
            if (total + 1 + e.len > max_bytes) break;
            out[total++] = e.len;
            std::memcpy(&out[total], e.data, e.len);
            total += e.len;
        }
        return total;
    }

    // Drain frames written since the previous drain() into `out`, packed as
    // [len][data]... Advances read_seq_ past what it returns, so successive
    // drains yield a continuous, non-overlapping audio timeline. If the unread
    // backlog exceeds either the ring capacity (producer lapped the consumer) or
    // max_bytes, the OLDEST unread frames are dropped first so the output always
    // holds the most recent audio and never overflows.
    size_t drain(uint8_t *out, size_t max_bytes)
    {
        if (!buf_) return 0;

        uint32_t w = write_seq_;            // snapshot the producer's position once
        uint32_t unread = w - read_seq_;

        // Producer lapped us: only the last `capacity_` frames are still valid.
        if (unread > (uint32_t)capacity_) {
            dropped_ += unread - (uint32_t)capacity_;
            read_seq_ = w - (uint32_t)capacity_;
            unread = (uint32_t)capacity_;
        }

        // Drop oldest unread frames until the remaining backlog fits max_bytes.
        for (;;) {
            size_t needed = 0;
            for (uint32_t i = 0; i < unread; i++) {
                const OpusFrameEntry &e = buf_[(read_seq_ + i) % capacity_];
                if (e.len > 0) needed += 1 + e.len;
            }
            if (needed <= max_bytes || unread == 0) break;
            read_seq_++;
            unread--;
            dropped_++;
        }

        // Copy out all remaining unread frames.
        size_t total = 0;
        for (uint32_t i = 0; i < unread; i++) {
            const OpusFrameEntry &e = buf_[(read_seq_ + i) % capacity_];
            if (e.len > 0) {
                if (total + 1 + e.len > max_bytes) break;
                out[total++] = e.len;
                std::memcpy(&out[total], e.data, e.len);
                total += e.len;
            }
        }
        read_seq_ = w;   // everything up to the snapshot is now consumed
        return total;
    }

    uint32_t dropped_count() const { return dropped_; }

private:
    OpusFrameEntry *buf_ = nullptr;
    size_t capacity_ = 0;
    // Producer-owned: monotonic count of frames ever written (never reset).
    volatile uint32_t write_seq_ = 0;
    // Consumer-owned: monotonic count of frames drained so far.
    uint32_t read_seq_ = 0;
    uint32_t dropped_ = 0;
};
