#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_cam_ctlr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class CameraUartStreamer {
public:
    esp_err_t init();
    esp_err_t start();
    esp_err_t capture_frame(uint8_t **out_data,
                            size_t *out_len,
                            uint32_t *out_width,
                            uint32_t *out_height,
                            uint32_t *out_pixelformat);
    // Capture directly into the aligned pixel order required by the JPEG encoder.
    // Caller releases *out_data with heap_caps_free().
    esp_err_t capture_jpeg_input(uint8_t **out_data,
                                 size_t *out_len,
                                 uint32_t *out_width,
                                 uint32_t *out_height,
                                 uint32_t *out_pixelformat);
    esp_err_t power_down();
    // Low power: fully release the DVP + sensor and allow init() to rebuild it
    // on the next capture_frame(). Reversible, unlike power_down() alone.
    esp_err_t low_power_standby();

private:
    esp_err_t configure_camera_pins();
    esp_err_t set_pwdn(bool asserted);
    esp_err_t reset_sensor();
    esp_err_t soft_power_down();
    esp_err_t ensure_dvp_ready();
    esp_err_t capture_frame_impl(uint8_t **out_data,
                                 size_t *out_len,
                                 uint32_t *out_width,
                                 uint32_t *out_height,
                                 uint32_t *out_pixelformat,
                                 bool jpeg_input);

    bool initialized_ = false;
    bool sensor_configured_ = false;
    bool sensor_awake_ = false;
    bool dvp_ready_ = false;

    esp_cam_ctlr_handle_t cam_handle_ = nullptr;
    uint8_t *parking_bufs_[3] = {};
    uint8_t *snapshot_bufs_[2] = {};
    SemaphoreHandle_t capture_sem_ = nullptr;
};
