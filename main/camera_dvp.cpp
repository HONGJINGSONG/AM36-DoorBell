#include "camera_uart.hpp"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "app_config.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_dvp.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "bsp.h"

#include "esp_cache.h"
#include "esp_timer.h"
#include "sp0a39_regs.h"

namespace {

static_assert(APP_IMAGE_OUTPUT_WIDTH > 0U &&
              APP_IMAGE_OUTPUT_WIDTH * 2U <= APP_CAMERA_SENSOR_WIDTH &&
              APP_IMAGE_OUTPUT_WIDTH % 16U == 0U,
              "Invalid centered crop width");
static_assert(APP_IMAGE_OUTPUT_HEIGHT * 2U == APP_CAMERA_SENSOR_HEIGHT &&
              APP_IMAGE_OUTPUT_HEIGHT % 16U == 0U,
              "Output height must match the 2x decimator");

// Byte offset of the centered crop window on a source row, in pixels. The 2x
// decimation itself is unchanged: we simply start it inside the frame and stop
// early, so this costs nothing per pixel (it actually reads fewer bytes than an
// uncropped pass). Rounded DOWN to an even pixel because UYVY packs two pixels
// into one 4-byte macro-pixel group -- an odd offset would start mid-group and
// split a chroma pair, and the output would no longer be valid YUV422.
// With src_w=640 and dst_w=288 this is (640-576)/2 = 32 pixels off each side.
inline uint32_t crop_x0_pixels(uint32_t src_w, uint32_t dst_w)
{
    const uint32_t used = dst_w * 2U;
    if (used >= src_w) return 0U;
    return ((src_w - used) / 2U) & ~1U;
}

// 2x box/decimation downsample of a YUV422 (UYVY/YUYV/VYUY) frame. Source is
// kept as-is byte-for-byte in each retained 4-byte macro-pixel group (2 pixels),
// so the output stays valid YUV422 and the downstream swizzle in
// ImageTransfer::encode_frame needs no change.
//   src: src_w x src_h, stride src_w*2   ->   dst: dst_w x (src_h/2)
// Keeps even rows and every other macro-pixel group (horizontal 2x decimation),
// starting from a centered crop when dst_w*2 < src_w.
void downsample_yuv422_2x(const uint8_t *src, uint8_t *dst,
                          uint32_t src_w, uint32_t src_h, uint32_t dst_w)
{
    const uint32_t src_stride = src_w * 2;          // bytes per source row
    const uint32_t dst_h = src_h / 2;
    const uint32_t x0 = crop_x0_pixels(src_w, dst_w);
    for (uint32_t dy = 0; dy < dst_h; dy++) {
        const uint8_t *srow = src + (size_t)(dy * 2) * src_stride
                                  + (size_t)x0 * 2;  // even rows, cropped start
        uint8_t *drow = dst + (size_t)dy * (dst_w * 2);
        // Take macro-pixel groups 0,2,4,... : copy 4 bytes, skip 4 bytes.
        for (uint32_t dx = 0; dx < dst_w / 2; dx++) {
            const uint8_t *s = srow + (size_t)(dx * 2) * 4;
            uint8_t *d = drow + (size_t)dx * 4;
            d[0] = s[0];
            d[1] = s[1];
            d[2] = s[2];
            d[3] = s[3];
        }
    }
}

// Fuse 2x UYVY decimation with the encoder's YCbYCr byte ordering. This avoids
// materializing an intermediate downsampled UYVY frame and traversing it again.
void downsample_uyvy_to_ycbycr_2x(const uint8_t *src, uint8_t *dst,
                                  uint32_t src_w, uint32_t src_h,
                                  uint32_t dst_w)
{
    const uint32_t src_stride = src_w * 2;
    const uint32_t dst_h = src_h / 2;
    const uint32_t x0 = crop_x0_pixels(src_w, dst_w);
    for (uint32_t dy = 0; dy < dst_h; dy++) {
        const uint8_t *srow = src + (size_t)(dy * 2) * src_stride
                                  + (size_t)x0 * 2;
        uint8_t *drow = dst + (size_t)dy * (dst_w * 2);
        for (uint32_t dx = 0; dx < dst_w / 2; dx++) {
            const uint8_t *s = srow + (size_t)(dx * 2) * 4;
            uint8_t *d = drow + (size_t)dx * 4;
            d[0] = s[1]; // Y0
            d[1] = s[0]; // Cb
            d[2] = s[3]; // Y1
            d[3] = s[2]; // Cr
        }
    }
}

// 2x decimation of a GRAY8 frame (1 byte/pixel): keep even rows, even columns,
// starting from the same centered crop as the YUV422 paths.
void downsample_gray_2x(const uint8_t *src, uint8_t *dst,
                        uint32_t src_w, uint32_t src_h, uint32_t dst_w)
{
    const uint32_t dst_h = src_h / 2;
    const uint32_t x0 = crop_x0_pixels(src_w, dst_w);
    for (uint32_t dy = 0; dy < dst_h; dy++) {
        const uint8_t *srow = src + (size_t)(dy * 2) * src_w + x0;
        uint8_t *drow = dst + (size_t)dy * dst_w;
        for (uint32_t dx = 0; dx < dst_w; dx++) {
            drow[dx] = srow[dx * 2];
        }
    }
}

constexpr const char *TAG = "camera_dvp";
constexpr TickType_t kPwdnSettleTicks = pdMS_TO_TICKS(100);
constexpr TickType_t kResetSettleTicks = pdMS_TO_TICKS(120);
constexpr uint32_t kFourccGrey = 0x59455247; // 'GREY'
constexpr uint32_t kFourccUyvy = 0x59565955; // 'UYVY'
constexpr uint32_t kFourccYuyv = 0x56595559; // 'YUYV'
constexpr uint32_t kFourccVyuy = 0x59555956; // 'VYUY'
constexpr uint32_t kFrameBytes = APP_CAMERA_FRAME_BYTES;
constexpr size_t kParkingBufferCount = 3;
constexpr size_t kSnapshotBufferCount = 2;
constexpr TickType_t kSnapshotMaxAgeTicks = pdMS_TO_TICKS(800);

#if APP_CAMERA_COLOR_ENABLE
constexpr uint32_t kDvpCaptureWidth = APP_CAMERA_SENSOR_WIDTH;
constexpr cam_ctlr_color_t kDvpInputColor = CAM_CTLR_COLOR_YUV422;
constexpr uint32_t kOutputPixelformat = kFourccUyvy;
#else
constexpr uint32_t kDvpCaptureWidth = APP_CAMERA_SENSOR_WIDTH;
constexpr cam_ctlr_color_t kDvpInputColor = CAM_CTLR_COLOR_GRAY8;
constexpr uint32_t kOutputPixelformat = kFourccGrey;
#endif

enum class snapshot_state : uint8_t {
    free,
    dma,
    ready,
    processing,
};

struct snapshot_slot {
    uint8_t *buffer;
    snapshot_state state;
    size_t received;
    TickType_t ready_tick;
};

struct dvp_cb_ctx {
    uint8_t *parking_buffers[kParkingBufferCount];
    snapshot_slot snapshots[kSnapshotBufferCount];
    size_t buflen;
    size_t next_parking;
    size_t next_snapshot;
    int frame_count;
    volatile size_t last_received;
    SemaphoreHandle_t done_sem;
    bool snapshot_requested;
    bool continuous_prefetch;
};

static dvp_cb_ctx s_dvp_ctx;
static portMUX_TYPE s_dvp_lock = portMUX_INITIALIZER_UNLOCKED;

struct snapshot_lease {
    uint8_t *buffer = nullptr;
    size_t received = 0;
    int slot = -1;
    uint32_t age_ms = 0;
};

static bool has_snapshot_dma_locked(const dvp_cb_ctx *ctx)
{
    for (size_t i = 0; i < kSnapshotBufferCount; ++i) {
        if (ctx->snapshots[i].state == snapshot_state::dma) {
            return true;
        }
    }
    return false;
}

static bool has_ready_snapshot_locked(const dvp_cb_ctx *ctx)
{
    for (size_t i = 0; i < kSnapshotBufferCount; ++i) {
        if (ctx->snapshots[i].state == snapshot_state::ready) {
            return true;
        }
    }
    return false;
}

static bool take_ready_snapshot(snapshot_lease *lease, TickType_t now)
{
    int newest = -1;
    TickType_t newest_age = portMAX_DELAY;

    portENTER_CRITICAL(&s_dvp_lock);
    for (size_t i = 0; i < kSnapshotBufferCount; ++i) {
        snapshot_slot &slot = s_dvp_ctx.snapshots[i];
        if (slot.state != snapshot_state::ready) {
            continue;
        }
        const TickType_t age = now - slot.ready_tick;
        if (age > kSnapshotMaxAgeTicks) {
            slot.state = snapshot_state::free;
            slot.received = 0;
            continue;
        }
        if (newest < 0 || age < newest_age) {
            newest = static_cast<int>(i);
            newest_age = age;
        }
    }

    if (newest >= 0) {
        for (size_t i = 0; i < kSnapshotBufferCount; ++i) {
            snapshot_slot &slot = s_dvp_ctx.snapshots[i];
            if (slot.state == snapshot_state::ready &&
                static_cast<int>(i) != newest) {
                slot.state = snapshot_state::free;
                slot.received = 0;
            }
        }
        snapshot_slot &slot = s_dvp_ctx.snapshots[newest];
        slot.state = snapshot_state::processing;
        lease->buffer = slot.buffer;
        lease->received = slot.received;
        lease->slot = newest;
        lease->age_ms = static_cast<uint32_t>(newest_age) * portTICK_PERIOD_MS;
    }
    portEXIT_CRITICAL(&s_dvp_lock);
    return newest >= 0;
}

static void release_snapshot(const snapshot_lease &lease)
{
    if (lease.slot < 0) {
        return;
    }
    portENTER_CRITICAL(&s_dvp_lock);
    snapshot_slot &slot = s_dvp_ctx.snapshots[lease.slot];
    if (slot.state == snapshot_state::processing) {
        slot.state = snapshot_state::free;
        slot.received = 0;
    }
    portEXIT_CRITICAL(&s_dvp_lock);
}

static void reset_snapshot_pipeline()
{
    portENTER_CRITICAL(&s_dvp_lock);
    s_dvp_ctx.snapshot_requested = false;
    s_dvp_ctx.continuous_prefetch = false;
    for (size_t i = 0; i < kSnapshotBufferCount; ++i) {
        s_dvp_ctx.snapshots[i].state = snapshot_state::free;
        s_dvp_ctx.snapshots[i].received = 0;
        s_dvp_ctx.snapshots[i].ready_tick = 0;
    }
    portEXIT_CRITICAL(&s_dvp_lock);
    if (s_dvp_ctx.done_sem) {
        xQueueReset(s_dvp_ctx.done_sem);
    }
}

static void clear_dvp_buffer_refs()
{
    portENTER_CRITICAL(&s_dvp_lock);
    for (size_t i = 0; i < kParkingBufferCount; ++i) {
        s_dvp_ctx.parking_buffers[i] = nullptr;
    }
    for (size_t i = 0; i < kSnapshotBufferCount; ++i) {
        s_dvp_ctx.snapshots[i].buffer = nullptr;
        s_dvp_ctx.snapshots[i].state = snapshot_state::free;
        s_dvp_ctx.snapshots[i].received = 0;
        s_dvp_ctx.snapshots[i].ready_tick = 0;
    }
    s_dvp_ctx.buflen = 0;
    s_dvp_ctx.snapshot_requested = false;
    s_dvp_ctx.continuous_prefetch = false;
    portEXIT_CRITICAL(&s_dvp_lock);
}

static void cancel_snapshot_requests()
{
    portENTER_CRITICAL(&s_dvp_lock);
    s_dvp_ctx.snapshot_requested = false;
    s_dvp_ctx.continuous_prefetch = false;
    portEXIT_CRITICAL(&s_dvp_lock);
}

static bool snapshot_prearm_active()
{
    portENTER_CRITICAL(&s_dvp_lock);
    const bool active = s_dvp_ctx.continuous_prefetch ||
                        s_dvp_ctx.snapshot_requested ||
                        has_snapshot_dma_locked(&s_dvp_ctx) ||
                        has_ready_snapshot_locked(&s_dvp_ctx);
    portEXIT_CRITICAL(&s_dvp_lock);
    return active;
}

static void configure_snapshot_request(bool continuous)
{
    portENTER_CRITICAL(&s_dvp_lock);
    s_dvp_ctx.continuous_prefetch = continuous;
    if (!has_ready_snapshot_locked(&s_dvp_ctx) &&
        !has_snapshot_dma_locked(&s_dvp_ctx)) {
        s_dvp_ctx.snapshot_requested = true;
    }
    portEXIT_CRITICAL(&s_dvp_lock);
}

static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle,
                                       esp_cam_ctlr_trans_t *trans, void *user_data)
{
    dvp_cb_ctx *ctx = static_cast<dvp_cb_ctx *>(user_data);
    uint8_t *buffer = nullptr;

    portENTER_CRITICAL_ISR(&s_dvp_lock);
    if (ctx->snapshot_requested || ctx->continuous_prefetch) {
        for (size_t offset = 0; offset < kSnapshotBufferCount; ++offset) {
            const size_t index = (ctx->next_snapshot + offset) % kSnapshotBufferCount;
            snapshot_slot &slot = ctx->snapshots[index];
            if (slot.state == snapshot_state::free) {
                slot.state = snapshot_state::dma;
                slot.received = 0;
                buffer = slot.buffer;
                ctx->next_snapshot = (index + 1) % kSnapshotBufferCount;
                ctx->snapshot_requested = false;
                break;
            }
        }
    }
    if (!buffer) {
        buffer = ctx->parking_buffers[ctx->next_parking];
        ctx->next_parking = (ctx->next_parking + 1) % kParkingBufferCount;
    }
    portEXIT_CRITICAL_ISR(&s_dvp_lock);

    trans->buffer = buffer;
    trans->buflen = ctx->buflen;
    return trans->buffer != nullptr;
}

static bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t handle,
                                        esp_cam_ctlr_trans_t *trans, void *user_data)
{
    dvp_cb_ctx *ctx = static_cast<dvp_cb_ctx *>(user_data);
    bool snapshot_done = false;

    portENTER_CRITICAL_ISR(&s_dvp_lock);
    ctx->frame_count++;
    ctx->last_received = trans->received_size;
    for (size_t i = 0; i < kSnapshotBufferCount; ++i) {
        snapshot_slot &slot = ctx->snapshots[i];
        if (slot.buffer == trans->buffer && slot.state == snapshot_state::dma) {
            slot.received = trans->received_size;
            slot.ready_tick = xTaskGetTickCountFromISR();
            slot.state = snapshot_state::ready;
            snapshot_done = true;
            break;
        }
    }
    portEXIT_CRITICAL_ISR(&s_dvp_lock);

    if (snapshot_done) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(ctx->done_sem, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }
    return false;
}

i2c_master_dev_handle_t s_sensor_dev = nullptr;

esp_err_t sensor_i2c_attach()
{
    if (s_sensor_dev) return ESP_OK;
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) return ESP_ERR_INVALID_STATE;

    i2c_device_config_t cfg = {};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address = APP_SP0A39_I2C_ADDR;
    cfg.scl_speed_hz = BSP_I2C0_FREQ_HZ;
    return i2c_master_bus_add_device(bus, &cfg, &s_sensor_dev);
}

void sensor_i2c_detach()
{
    if (s_sensor_dev) {
        i2c_master_bus_rm_device(s_sensor_dev);
        s_sensor_dev = nullptr;
    }
}

esp_err_t sensor_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_sensor_dev, buf, 2, 50);
}

esp_err_t sensor_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_sensor_dev, &reg, 1, val, 1, 50);
}

esp_err_t sensor_write_regs()
{
    const size_t count = sizeof(s_sp0a39_regs) / sizeof(s_sp0a39_regs[0]);
    for (size_t i = 0; i < count; i++) {
        esp_err_t ret = sensor_write_reg(s_sp0a39_regs[i][0], s_sp0a39_regs[i][1]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "reg write failed at [%u] 0x%02x=0x%02x: %s",
                     (unsigned)i, s_sp0a39_regs[i][0], s_sp0a39_regs[i][1],
                     esp_err_to_name(ret));
            return ret;
        }
    }
    ESP_LOGI(TAG, "SP0A39 register init done (%u regs)", (unsigned)count);
    return ESP_OK;
}

esp_err_t sensor_read_id()
{
    ESP_RETURN_ON_ERROR(sensor_write_reg(0xfd, 0x00), TAG, "page select");
    uint8_t id_h = 0, id_l = 0;
    ESP_RETURN_ON_ERROR(sensor_read_reg(0x00, &id_h), TAG, "read id_h");
    ESP_RETURN_ON_ERROR(sensor_read_reg(0x01, &id_l), TAG, "read id_l");
    ESP_LOGI(TAG, "SP0A39 chip ID: 0x%02X%02X", id_h, id_l);
    return ESP_OK;
}

void log_sensor_output_regs()
{
    uint8_t p0_1c = 0;
    uint8_t p0_30 = 0;
    uint8_t p0_31 = 0;
    uint8_t p1_32 = 0;
    uint8_t p1_34 = 0;
    uint8_t p1_35 = 0;
    uint8_t p1_36 = 0;
    if (sensor_write_reg(0xfd, 0x00) == ESP_OK) {
        sensor_read_reg(0x1c, &p0_1c);
        sensor_read_reg(0x30, &p0_30);
        sensor_read_reg(0x31, &p0_31);
    }
    if (sensor_write_reg(0xfd, 0x01) == ESP_OK) {
        sensor_read_reg(0x32, &p1_32);
        sensor_read_reg(0x34, &p1_34);
        sensor_read_reg(0x35, &p1_35);
        sensor_read_reg(0x36, &p1_36);
    }
    sensor_write_reg(0xfd, 0x00);
    ESP_LOGI(TAG, "SP0A39 output regs: P0:1c=0x%02x P0:30=0x%02x P0:31=0x%02x P1:32=0x%02x P1:34=0x%02x P1:35=0x%02x P1:36=0x%02x",
             p0_1c, p0_30, p0_31, p1_32, p1_34, p1_35, p1_36);
}

void log_gpio_diagnostics()
{
    int vsync_changes = 0;
    int hsync_changes = 0;
    int pclk_changes = 0;
    int last_vsync = gpio_get_level(BSP_SP0A39_VSYNC_GPIO);
    int last_hsync = gpio_get_level(BSP_SP0A39_HSYNC_GPIO);
    int last_pclk = gpio_get_level(BSP_SP0A39_PCLK_GPIO);
    for (int i = 0; i < 100000; i++) {
        int vsync = gpio_get_level(BSP_SP0A39_VSYNC_GPIO);
        int hsync = gpio_get_level(BSP_SP0A39_HSYNC_GPIO);
        int pclk = gpio_get_level(BSP_SP0A39_PCLK_GPIO);
        if (vsync != last_vsync) {
            vsync_changes++;
            last_vsync = vsync;
        }
        if (hsync != last_hsync) {
            hsync_changes++;
            last_hsync = hsync;
        }
        if (pclk != last_pclk) {
            pclk_changes++;
            last_pclk = pclk;
        }
    }
    ESP_LOGI(TAG, "GPIO diagnostics: VSYNC changes=%d HSYNC changes=%d PCLK changes=%d HSYNC level=%d",
             vsync_changes, hsync_changes, pclk_changes, gpio_get_level(BSP_SP0A39_HSYNC_GPIO));
}

} // namespace

esp_err_t CameraUartStreamer::init()
{
    if (initialized_) return ESP_OK;
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c init");

    // Configure DVP data pins
    configure_camera_pins();

    // Create DVP controller first — it provides XCLK to the sensor
    esp_err_t ret = ensure_dvp_ready();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DVP create failed: %s", esp_err_to_name(ret));
        initialized_ = true;
        return ESP_OK;
    }

    // Now sensor has XCLK, wake it and configure registers
    set_pwdn(false);
    vTaskDelay(pdMS_TO_TICKS(200));
    ret = reset_sensor();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "sensor reset failed: %s", esp_err_to_name(ret));
        initialized_ = true;
        return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    ret = sensor_i2c_attach();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "sensor i2c failed: %s", esp_err_to_name(ret));
        initialized_ = true;
        return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    ret = sensor_read_id();
    if (ret == ESP_OK) {
        ret = sensor_write_regs();
    }
    if (ret == ESP_OK) {
        sensor_configured_ = true;
        sensor_awake_ = true;
        ESP_LOGI(TAG, "sensor initialized, DVP ready");
    } else {
        ESP_LOGW(TAG, "sensor regs failed: %s", esp_err_to_name(ret));
    }

    // Sensor configured — now start DVP permanently
    reset_snapshot_pipeline();
    ret = esp_cam_ctlr_start(cam_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_cam_ctlr_start failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "DVP running permanently");
    }

    initialized_ = true;
    return ESP_OK;
}

esp_err_t CameraUartStreamer::start()
{
    ESP_RETURN_ON_ERROR(init(), TAG, "init");
    return ESP_OK;
}

esp_err_t CameraUartStreamer::set_pwdn(bool asserted)
{
    return bsp_ioexp_set_pin(BSP_SP0A39_PWDN_IOEXP_PIN, asserted);
}

esp_err_t CameraUartStreamer::reset_sensor()
{
    gpio_config_t reset = {};
    reset.pin_bit_mask = 1ULL << BSP_SP0A39_RESET_GPIO;
    reset.mode = GPIO_MODE_OUTPUT;
    reset.pull_up_en = GPIO_PULLUP_DISABLE;
    reset.pull_down_en = GPIO_PULLDOWN_DISABLE;
    reset.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&reset), TAG, "camera reset gpio");
    gpio_set_level(BSP_SP0A39_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(BSP_SP0A39_RESET_GPIO, 1);
    vTaskDelay(kResetSettleTicks);
    return ESP_OK;
}

esp_err_t CameraUartStreamer::configure_camera_pins()
{
    uint64_t mask = 0;
    mask |= 1ULL << BSP_SP0A39_PCLK_GPIO;
    mask |= 1ULL << BSP_SP0A39_VSYNC_GPIO;
    mask |= 1ULL << BSP_SP0A39_HSYNC_GPIO;
    mask |= 1ULL << BSP_SP0A39_D0_GPIO;
    mask |= 1ULL << BSP_SP0A39_D1_GPIO;
    mask |= 1ULL << BSP_SP0A39_D2_GPIO;
    mask |= 1ULL << BSP_SP0A39_D3_GPIO;
    mask |= 1ULL << BSP_SP0A39_D4_GPIO;
    mask |= 1ULL << BSP_SP0A39_D5_GPIO;
    mask |= 1ULL << BSP_SP0A39_D6_GPIO;
    mask |= 1ULL << BSP_SP0A39_D7_GPIO;
    gpio_config_t input_conf = {};
    input_conf.pin_bit_mask = mask;
    input_conf.mode = GPIO_MODE_INPUT;
    input_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    input_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    input_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&input_conf), TAG, "gpio input config");
    ESP_LOGI(TAG, "camera DVP pins configured as inputs");
    return ESP_OK;
}

esp_err_t CameraUartStreamer::power_down()
{
    if (dvp_ready_ && cam_handle_) {
        // init() leaves the controller started, so tear down in the correct
        // order: stop -> disable -> del. Skipping stop() leaves the DVP IO
        // signals claimed and the next esp_cam_new_dvp_ctlr() fails with
        // "failed to claim io signals".
        esp_err_t se = esp_cam_ctlr_stop(cam_handle_);
        if (se != ESP_OK) ESP_LOGW(TAG, "cam stop: %s", esp_err_to_name(se));
        reset_snapshot_pipeline();
        se = esp_cam_ctlr_disable(cam_handle_);
        if (se != ESP_OK) ESP_LOGW(TAG, "cam disable: %s", esp_err_to_name(se));
        se = esp_cam_ctlr_del(cam_handle_);
        if (se != ESP_OK) ESP_LOGW(TAG, "cam del: %s", esp_err_to_name(se));
        cam_handle_ = nullptr;
        for (size_t i = 0; i < kParkingBufferCount; ++i) {
            if (parking_bufs_[i]) {
                heap_caps_free(parking_bufs_[i]);
                parking_bufs_[i] = nullptr;
                s_dvp_ctx.parking_buffers[i] = nullptr;
            }
        }
        for (size_t i = 0; i < kSnapshotBufferCount; ++i) {
            if (snapshot_bufs_[i]) {
                heap_caps_free(snapshot_bufs_[i]);
                snapshot_bufs_[i] = nullptr;
            }
        }
        clear_dvp_buffer_refs();
        dvp_ready_ = false;
    }
    set_pwdn(true);
    gpio_reset_pin(BSP_SP0A39_RESET_GPIO);
    sensor_i2c_detach();
    sensor_configured_ = false;
    sensor_awake_ = false;
    return ESP_OK;
}

esp_err_t CameraUartStreamer::soft_power_down()
{
    set_pwdn(true);
    return ESP_OK;
}

esp_err_t CameraUartStreamer::low_power_standby()
{
    // Release everything (DVP controller, DMA buffers, XCLK, sensor power) and
    // clear initialized_ so the next capture_frame()->init() rebuilds cleanly.
    esp_err_t ret = power_down();
    initialized_ = false;
    ESP_LOGI(TAG, "camera in low power standby (released)");
    return ret;
}

esp_err_t CameraUartStreamer::ensure_dvp_ready()
{
    if (dvp_ready_ && cam_handle_) return ESP_OK;

    esp_cam_ctlr_dvp_pin_config_t pins = {};
    pins.data_width = CAM_CTLR_DATA_WIDTH_8;
    pins.data_io[0] = BSP_SP0A39_D0_GPIO;
    pins.data_io[1] = BSP_SP0A39_D1_GPIO;
    pins.data_io[2] = BSP_SP0A39_D2_GPIO;
    pins.data_io[3] = BSP_SP0A39_D3_GPIO;
    pins.data_io[4] = BSP_SP0A39_D4_GPIO;
    pins.data_io[5] = BSP_SP0A39_D5_GPIO;
    pins.data_io[6] = BSP_SP0A39_D6_GPIO;
    pins.data_io[7] = BSP_SP0A39_D7_GPIO;
    pins.vsync_io = BSP_SP0A39_VSYNC_GPIO;
    pins.de_io = BSP_SP0A39_HSYNC_GPIO;
    pins.pclk_io = BSP_SP0A39_PCLK_GPIO;
    pins.xclk_io = BSP_SP0A39_MCLK_GPIO;

    esp_cam_ctlr_dvp_config_t dvp_cfg = {};
    dvp_cfg.ctlr_id = 0;
    dvp_cfg.clk_src = CAM_CLK_SRC_DEFAULT;
    dvp_cfg.h_res = kDvpCaptureWidth;
    dvp_cfg.v_res = APP_CAMERA_SENSOR_HEIGHT;
    dvp_cfg.input_data_color_type = kDvpInputColor;
    dvp_cfg.pin = &pins;
    dvp_cfg.xclk_freq = APP_SP0A39_MCLK_HZ;
    dvp_cfg.dma_burst_size = 64;
    dvp_cfg.bk_buffer_dis = 1;

    esp_err_t ret = esp_cam_new_dvp_ctlr(&dvp_cfg, &cam_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_cam_new_dvp_ctlr failed: %s", esp_err_to_name(ret));
        return ret;
    }

    for (size_t i = 0; i < kParkingBufferCount; ++i) {
        parking_bufs_[i] = static_cast<uint8_t *>(
            esp_cam_ctlr_alloc_buffer(cam_handle_, kFrameBytes,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA));
        if (!parking_bufs_[i]) {
            ESP_LOGE(TAG, "parking buffer alloc failed");
            for (size_t j = 0; j < kParkingBufferCount; ++j) {
                if (parking_bufs_[j]) {
                    heap_caps_free(parking_bufs_[j]);
                    parking_bufs_[j] = nullptr;
                }
            }
            clear_dvp_buffer_refs();
            esp_cam_ctlr_del(cam_handle_);
            cam_handle_ = nullptr;
            return ESP_ERR_NO_MEM;
        }
    }

    for (size_t i = 0; i < kSnapshotBufferCount; ++i) {
        snapshot_bufs_[i] = static_cast<uint8_t *>(
            esp_cam_ctlr_alloc_buffer(cam_handle_, kFrameBytes,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA));
        if (!snapshot_bufs_[i]) {
            ESP_LOGE(TAG, "snapshot buffer alloc failed");
            for (size_t j = 0; j < kParkingBufferCount; ++j) {
                if (parking_bufs_[j]) {
                    heap_caps_free(parking_bufs_[j]);
                    parking_bufs_[j] = nullptr;
                }
            }
            for (size_t j = 0; j < kSnapshotBufferCount; ++j) {
                if (snapshot_bufs_[j]) {
                    heap_caps_free(snapshot_bufs_[j]);
                    snapshot_bufs_[j] = nullptr;
                }
            }
            clear_dvp_buffer_refs();
            esp_cam_ctlr_del(cam_handle_);
            cam_handle_ = nullptr;
            return ESP_ERR_NO_MEM;
        }
    }
    if (!capture_sem_) {
        capture_sem_ = xSemaphoreCreateBinary();
    }
    if (!capture_sem_) {
        ESP_LOGE(TAG, "capture semaphore alloc failed");
        for (size_t i = 0; i < kParkingBufferCount; ++i) {
            heap_caps_free(parking_bufs_[i]);
            parking_bufs_[i] = nullptr;
        }
        for (size_t i = 0; i < kSnapshotBufferCount; ++i) {
            heap_caps_free(snapshot_bufs_[i]);
            snapshot_bufs_[i] = nullptr;
        }
        clear_dvp_buffer_refs();
        esp_cam_ctlr_del(cam_handle_);
        cam_handle_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&s_dvp_lock);
    for (size_t i = 0; i < kParkingBufferCount; ++i) {
        s_dvp_ctx.parking_buffers[i] = parking_bufs_[i];
    }
    for (size_t i = 0; i < kSnapshotBufferCount; ++i) {
        s_dvp_ctx.snapshots[i].buffer = snapshot_bufs_[i];
        s_dvp_ctx.snapshots[i].state = snapshot_state::free;
        s_dvp_ctx.snapshots[i].received = 0;
        s_dvp_ctx.snapshots[i].ready_tick = 0;
    }
    s_dvp_ctx.buflen = kFrameBytes;
    s_dvp_ctx.next_parking = 0;
    s_dvp_ctx.next_snapshot = 0;
    s_dvp_ctx.frame_count = 0;
    s_dvp_ctx.last_received = 0;
    s_dvp_ctx.snapshot_requested = false;
    s_dvp_ctx.continuous_prefetch = false;
    s_dvp_ctx.done_sem = capture_sem_;
    portEXIT_CRITICAL(&s_dvp_lock);
    xQueueReset(capture_sem_);

    // Register callbacks once (before enable)
    esp_cam_ctlr_evt_cbs_t cbs = {};
    cbs.on_get_new_trans = on_get_new_trans;
    cbs.on_trans_finished = on_trans_finished;
    ret = esp_cam_ctlr_register_event_callbacks(cam_handle_, &cbs, &s_dvp_ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "register cbs failed: %s", esp_err_to_name(ret));
        for (size_t i = 0; i < kParkingBufferCount; ++i) {
            heap_caps_free(parking_bufs_[i]);
            parking_bufs_[i] = nullptr;
        }
        for (size_t i = 0; i < kSnapshotBufferCount; ++i) {
            heap_caps_free(snapshot_bufs_[i]);
            snapshot_bufs_[i] = nullptr;
        }
        clear_dvp_buffer_refs();
        esp_cam_ctlr_del(cam_handle_);
        cam_handle_ = nullptr;
        return ret;
    }

    ret = esp_cam_ctlr_enable(cam_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_cam_ctlr_enable failed: %s", esp_err_to_name(ret));
        for (size_t i = 0; i < kParkingBufferCount; ++i) {
            heap_caps_free(parking_bufs_[i]);
            parking_bufs_[i] = nullptr;
        }
        for (size_t i = 0; i < kSnapshotBufferCount; ++i) {
            heap_caps_free(snapshot_bufs_[i]);
            snapshot_bufs_[i] = nullptr;
        }
        clear_dvp_buffer_refs();
        esp_cam_ctlr_del(cam_handle_);
        cam_handle_ = nullptr;
        return ret;
    }

    dvp_ready_ = true;
    ESP_LOGI(TAG, "DVP controller ready, XCLK on GPIO%d", BSP_SP0A39_MCLK_GPIO);
    return ESP_OK;
}

esp_err_t CameraUartStreamer::capture_frame(uint8_t **out_data,
                                            size_t *out_len,
                                            uint32_t *out_width,
                                            uint32_t *out_height,
                                            uint32_t *out_pixelformat)
{
    return capture_frame_impl(out_data, out_len, out_width, out_height,
                              out_pixelformat, false);
}

esp_err_t CameraUartStreamer::capture_jpeg_input(uint8_t **out_data,
                                                 size_t *out_len,
                                                 uint32_t *out_width,
                                                 uint32_t *out_height,
                                                 uint32_t *out_pixelformat)
{
    return capture_frame_impl(out_data, out_len, out_width, out_height,
                              out_pixelformat, true);
}

esp_err_t CameraUartStreamer::capture_frame_impl(uint8_t **out_data,
                                                 size_t *out_len,
                                                 uint32_t *out_width,
                                                 uint32_t *out_height,
                                                 uint32_t *out_pixelformat,
                                                 bool jpeg_input)
{
    const int64_t t_capture_start = esp_timer_get_time();
    ESP_RETURN_ON_FALSE(out_data && out_len && out_width && out_height && out_pixelformat,
                        ESP_ERR_INVALID_ARG, TAG, "invalid args");
    *out_data = nullptr;
    *out_len = 0;
    *out_width = 0;
    *out_height = 0;
    *out_pixelformat = 0;

    ESP_RETURN_ON_ERROR(init(), TAG, "init");
    if (!dvp_ready_) {
        ESP_LOGE(TAG, "DVP not ready");
        return ESP_ERR_INVALID_STATE;
    }

    snapshot_lease lease;
    bool prefetched = take_ready_snapshot(&lease, xTaskGetTickCount());

    // The semaphore is only a wake hint. Slot state is authoritative because
    // two snapshot completions may coalesce into one binary semaphore token.
    xQueueReset(capture_sem_);
    // Keep continuous prefetch latched: the ISR routes every DVP frame end into
    // a snapshot slot, so a completed frame is essentially always sitting ready
    // and this function returns without waiting for a VSYNC. One-shot arming was
    // tried instead, to keep the kept frame out of the encode's PSRAM window,
    // but the corruption it was working around came from MSPI contention and is
    // fixed at the source by APP_SP0A39_MCLK_HZ. What one-shot arming does cost
    // is throughput: arming lands mid-frame, so the frame is discarded and the
    // next one is awaited, averaging 1.5 sensor frame periods of dead time per
    // capture on top of the downsample and encode. With the sensor at ~10 fps
    // that alone caps the feed at 6-7 fps. Prefetching removes that wait.
    configure_snapshot_request(true);
    if (!prefetched) {
        prefetched = take_ready_snapshot(&lease, xTaskGetTickCount());
    }

    const int64_t t_wait_start = esp_timer_get_time();
    bool got_frame = prefetched;
    if (!got_frame) {
        const TickType_t wait_start_tick = xTaskGetTickCount();
        const TickType_t wait_limit = pdMS_TO_TICKS(5000);
        while (!got_frame) {
            const TickType_t elapsed = xTaskGetTickCount() - wait_start_tick;
            if (elapsed >= wait_limit) {
                break;
            }
            const TickType_t remaining = wait_limit - elapsed;
            if (xSemaphoreTake(capture_sem_, remaining) != pdTRUE) {
                break;
            }
            got_frame = take_ready_snapshot(&lease, xTaskGetTickCount());
        }
    }
    const int64_t t_wait_done = esp_timer_get_time();

    if (got_frame && lease.received >= kFrameBytes && lease.buffer) {
        ESP_LOGD(TAG, "captured frame: %u bytes slot=%d age=%lums",
                 static_cast<unsigned>(lease.received), lease.slot,
                 static_cast<unsigned long>(lease.age_ms));

        // Allocate only the configured cropped/downsampled output.
        constexpr size_t kOutBytes = APP_IMAGE_OUTPUT_BYTES;
        const int64_t t_alloc_start = esp_timer_get_time();
        uint8_t *copy = jpeg_input
            ? static_cast<uint8_t *>(heap_caps_aligned_alloc(
                  16, kOutBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT))
            : static_cast<uint8_t *>(heap_caps_malloc(
                  kOutBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (!copy) {
            copy = jpeg_input
                ? static_cast<uint8_t *>(heap_caps_aligned_alloc(
                      16, kOutBytes, MALLOC_CAP_8BIT))
                : static_cast<uint8_t *>(heap_caps_malloc(
                      kOutBytes, MALLOC_CAP_8BIT));
        }
        if (!copy) {
            ESP_LOGE(TAG, "frame copy alloc failed");
            release_snapshot(lease);
            return ESP_ERR_NO_MEM;
        }
        const int64_t t_alloc_done = esp_timer_get_time();

        const int64_t t_cache_start = esp_timer_get_time();
        esp_cache_msync(lease.buffer, kFrameBytes,
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        const int64_t t_cache_done = esp_timer_get_time();

        // Decimate directly from DMA. A 288-pixel output uses the centered
        // 576 columns of the 640-pixel sensor frame.
        const int64_t t_downsample_start = esp_timer_get_time();
#if APP_CAMERA_COLOR_ENABLE
        if (jpeg_input) {
            downsample_uyvy_to_ycbycr_2x(
                lease.buffer, copy,
                APP_CAMERA_SENSOR_WIDTH, APP_CAMERA_SENSOR_HEIGHT,
                APP_IMAGE_OUTPUT_WIDTH);
        } else {
            downsample_yuv422_2x(lease.buffer, copy,
                                 APP_CAMERA_SENSOR_WIDTH, APP_CAMERA_SENSOR_HEIGHT,
                                 APP_IMAGE_OUTPUT_WIDTH);
        }
#else
        downsample_gray_2x(lease.buffer, copy,
                           APP_CAMERA_SENSOR_WIDTH, APP_CAMERA_SENSOR_HEIGHT,
                           APP_IMAGE_OUTPUT_WIDTH);
#endif
        const int64_t t_downsample_done = esp_timer_get_time();

        release_snapshot(lease);

        *out_data = copy;
        *out_len = kOutBytes;
        *out_width = APP_IMAGE_OUTPUT_WIDTH;
        *out_height = APP_IMAGE_OUTPUT_HEIGHT;
        *out_pixelformat = jpeg_input && APP_CAMERA_COLOR_ENABLE
            ? kFourccYuyv
            : kOutputPixelformat;
        const int64_t t_capture_done = esp_timer_get_time();
        if (jpeg_input) {
            ESP_LOGD(TAG,
                     "[CAPTURE-JPEG] prefetched=%d slot=%d raw_age=%lums "
                     "prearm=%d setup=%lldus wait=%lldus alloc=%lldus "
                     "cache=%lldus fused=%lldus total=%lldus",
                     prefetched ? 1 : 0, lease.slot,
                     static_cast<unsigned long>(lease.age_ms),
                     snapshot_prearm_active() ? 1 : 0,
                     (long long)(t_wait_start - t_capture_start),
                     (long long)(t_wait_done - t_wait_start),
                     (long long)(t_alloc_done - t_alloc_start),
                     (long long)(t_cache_done - t_cache_start),
                     (long long)(t_downsample_done - t_downsample_start),
                     (long long)(t_capture_done - t_capture_start));
        } else {
            ESP_LOGD(TAG,
                     "[CAPTURE] setup=%lldus wait=%lldus alloc=%lldus cache=%lldus "
                     "downsample=%lldus total=%lldus",
                     (long long)(t_wait_start - t_capture_start),
                     (long long)(t_wait_done - t_wait_start),
                     (long long)(t_alloc_done - t_alloc_start),
                     (long long)(t_cache_done - t_cache_start),
                     (long long)(t_downsample_done - t_downsample_start),
                     (long long)(t_capture_done - t_capture_start));
        }
        return ESP_OK;
    }

    cancel_snapshot_requests();
    release_snapshot(lease);
    if (!got_frame && cam_handle_) {
        const esp_err_t stop_err = esp_cam_ctlr_stop(cam_handle_);
        if (stop_err == ESP_OK) {
            reset_snapshot_pipeline();
            const esp_err_t restart_err = esp_cam_ctlr_start(cam_handle_);
            if (restart_err != ESP_OK) {
                ESP_LOGE(TAG, "DVP restart after capture timeout failed: %s",
                         esp_err_to_name(restart_err));
            }
        } else {
            ESP_LOGE(TAG, "DVP stop after capture timeout failed: %s",
                     esp_err_to_name(stop_err));
        }
    }
    int frame_count = 0;
    size_t last_received = 0;
    portENTER_CRITICAL(&s_dvp_lock);
    frame_count = s_dvp_ctx.frame_count;
    last_received = s_dvp_ctx.last_received;
    portEXIT_CRITICAL(&s_dvp_lock);
    ESP_LOGE(TAG, "capture failed: got=%d frames=%d last=%u received=%u expected=%u",
             got_frame ? 1 : 0, frame_count, static_cast<unsigned>(last_received),
             static_cast<unsigned>(lease.received), static_cast<unsigned>(kFrameBytes));
    return got_frame ? ESP_ERR_INVALID_SIZE : ESP_ERR_TIMEOUT;
}
