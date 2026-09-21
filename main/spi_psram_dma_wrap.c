#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_private/spi_common_internal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#include "spi_psram_dma_wrap.h"

#define SPI_POLLING_DMA_BUFFER_BYTES 1024U
#define SPI_POLLING_DMA_BUFFER_ALIGNMENT CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE

/* Octal PSRAM device wrap boundary, not the data cache line. See the note in
 * app_spi_tx_dma_prioritize_psram(). */
#define SPI_TX_PSRAM_BURST_BYTES 32U
/* Above the default 0 that gdma_channel_alloc() assigns to every channel, but
 * one level below the maximum of 5 so audio DMA can still be raised over the
 * display if it ever needs to. */
#define SPI_TX_DMA_PRIORITY 4
/* The camera byte stream cannot be back-pressured: a CAM FIFO overrun is
 * unrecoverable and shifts the UYVY phase for the rest of the frame. Every
 * other GDMA client is transactional and can wait, so the camera takes the
 * ESP32-S3 hardware maximum (levels [0,5]; the HAL constant is private). The
 * LCD TX priority below is on the gateway, which has no camera. */
#define DVP_RX_DMA_PRIORITY 5

static const char *TAG = "spi_dma_wrap";

/* LR20xx polling transactions originate from stack VLAs. Once ESP-SR AFE is
 * resident, allocating two temporary internal DMA buffers for every large
 * packet can fail. Reserve one aligned pair at link time and serialize users. */
static DMA_ATTR uint8_t s_polling_dma_tx[SPI_POLLING_DMA_BUFFER_BYTES]
    __attribute__((aligned(SPI_POLLING_DMA_BUFFER_ALIGNMENT)));
static DMA_ATTR uint8_t s_polling_dma_rx[SPI_POLLING_DMA_BUFFER_BYTES]
    __attribute__((aligned(SPI_POLLING_DMA_BUFFER_ALIGNMENT)));
static StaticSemaphore_t s_polling_dma_mutex_storage;
static SemaphoreHandle_t s_polling_dma_mutex;
static portMUX_TYPE s_polling_dma_init_lock = portMUX_INITIALIZER_UNLOCKED;

static SemaphoreHandle_t polling_dma_mutex(void)
{
    if (s_polling_dma_mutex == NULL) {
        portENTER_CRITICAL(&s_polling_dma_init_lock);
        if (s_polling_dma_mutex == NULL) {
            s_polling_dma_mutex =
                xSemaphoreCreateMutexStatic(&s_polling_dma_mutex_storage);
        }
        portEXIT_CRITICAL(&s_polling_dma_init_lock);
    }
    return s_polling_dma_mutex;
}

esp_err_t app_spi_polling_dma_prepare(spi_host_device_t host_id)
{
#if SOC_GDMA_SUPPORTED
    /* IDF enables a 32-byte AHB data burst for both SPI GDMA directions. On
     * ESP32-S3 the RX burst makes every RX byte count require 4-byte alignment.
     * Disable data burst only on this host's RX channel, then refresh the
     * alignment cached by spi_master. TX burst remains enabled. */
    const spi_dma_ctx_t *const_ctx = spi_bus_get_dma_ctx(host_id);
    if (const_ctx == NULL || const_ctx->rx_dma_chan == NULL) {
        ESP_LOGE(TAG, "SPI%d RX DMA context unavailable", (int)host_id + 1);
        return ESP_ERR_INVALID_STATE;
    }

    gdma_transfer_config_t transfer_cfg = {
        .max_data_burst_size = 0,
        .access_ext_mem = true,
    };
    esp_err_t err = gdma_config_transfer(const_ctx->rx_dma_chan, &transfer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI%d disable RX data burst failed: %s",
                 (int)host_id + 1, esp_err_to_name(err));
        return err;
    }

    size_t int_alignment = 0;
    size_t ext_alignment = 0;
    err = gdma_get_alignment_constraints(const_ctx->rx_dma_chan,
                                         &int_alignment, &ext_alignment);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI%d read RX alignment failed: %s",
                 (int)host_id + 1, esp_err_to_name(err));
        return err;
    }

    /* spi_bus_get_dma_ctx() intentionally exposes a const diagnostic view, but
     * spi_master stores the channel's alignment in this same private context.
     * Keep this IDF-private compatibility adjustment isolated in one helper. */
    spi_dma_ctx_t *ctx = (spi_dma_ctx_t *)const_ctx;
    ctx->dma_align_rx_int = int_alignment;
    ctx->dma_align_rx_ext = ext_alignment;
    ESP_LOGI(TAG, "SPI%d RX data burst off, align internal=%u external=%u",
             (int)host_id + 1, (unsigned)int_alignment,
             (unsigned)ext_alignment);
    return ESP_OK;
#else
    (void)host_id;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t app_spi_tx_dma_prioritize_psram(spi_host_device_t host_id)
{
#if SOC_GDMA_SUPPORTED
    /* With 64-byte cache lines every competing refill holds MSPI twice as
     * long, and the LCD pixel stream (priority 0 after gdma_channel_alloc())
     * runs its TX FIFO dry mid-frame (SPI_TRANS_DMA_TX_FAIL). The fix is
     * arbitration priority, not a wider burst: do NOT raise
     * max_data_burst_size to 64 bytes. The Octal PSRAM's device wrap boundary
     * is 32 bytes ("BurstLen: 32 Byte" at startup); a 64-byte block straddling
     * it fetches misplaced data and shows as colour blocks and tearing with no
     * DMA error flag. */
    const spi_dma_ctx_t *const_ctx = spi_bus_get_dma_ctx(host_id);
    if (const_ctx == NULL || const_ctx->tx_dma_chan == NULL) {
        ESP_LOGE(TAG, "SPI%d TX DMA context unavailable", (int)host_id + 1);
        return ESP_ERR_INVALID_STATE;
    }

    gdma_transfer_config_t transfer_cfg = {
        .max_data_burst_size = SPI_TX_PSRAM_BURST_BYTES,
        .access_ext_mem = true,
    };
    esp_err_t err = gdma_config_transfer(const_ctx->tx_dma_chan, &transfer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI%d set TX burst to %uB failed: %s",
                 (int)host_id + 1, (unsigned)SPI_TX_PSRAM_BURST_BYTES,
                 esp_err_to_name(err));
        return err;
    }

    err = gdma_set_priority(const_ctx->tx_dma_chan, SPI_TX_DMA_PRIORITY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI%d set TX DMA priority failed: %s",
                 (int)host_id + 1, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG,
             "SPI%d TX PSRAM burst=%uB priority=%d (cache line=%uB, device "
             "burst governs)",
             (int)host_id + 1, (unsigned)SPI_TX_PSRAM_BURST_BYTES,
             SPI_TX_DMA_PRIORITY,
             (unsigned)CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE);
    return ESP_OK;
#else
    (void)host_id;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t __real_spi_device_queue_trans(spi_device_handle_t handle,
                                        spi_transaction_t *trans_desc,
                                        TickType_t ticks_to_wait);

esp_err_t __real_spi_device_polling_transmit(spi_device_handle_t handle,
                                             spi_transaction_t *trans_desc);

esp_err_t __real_gdma_connect(gdma_channel_handle_t dma_chan,
                              gdma_trigger_t trig_periph);

esp_err_t __wrap_spi_device_queue_trans(spi_device_handle_t handle,
                                        spi_transaction_t *trans_desc,
                                        TickType_t ticks_to_wait)
{
    if (!(trans_desc->flags & SPI_TRANS_USE_TXDATA) &&
        trans_desc->tx_buffer &&
        esp_ptr_external_ram(trans_desc->tx_buffer) &&
        esp_ptr_dma_ext_capable(trans_desc->tx_buffer)) {
        trans_desc->flags |= SPI_TRANS_DMA_USE_PSRAM;
    }

    return __real_spi_device_queue_trans(handle, trans_desc, ticks_to_wait);
}

esp_err_t __wrap_spi_device_polling_transmit(spi_device_handle_t handle,
                                             spi_transaction_t *trans_desc)
{
    if (trans_desc == NULL ||
        (trans_desc->flags & (SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA)) != 0 ||
        (trans_desc->length & 7U) != 0 ||
        (trans_desc->rxlength & 7U) != 0) {
        return __real_spi_device_polling_transmit(handle, trans_desc);
    }

    const bool has_tx = trans_desc->tx_buffer != NULL;
    const bool has_rx = trans_desc->rx_buffer != NULL;
    const size_t tx_bytes = trans_desc->length / 8U;
    const size_t rx_bytes = has_rx
                                ? ((trans_desc->rxlength != 0
                                        ? trans_desc->rxlength
                                        : trans_desc->length) /
                                   8U)
                                : 0U;

    const bool can_use_reserved_dma =
        (has_tx || has_rx) &&
        tx_bytes <= SPI_POLLING_DMA_BUFFER_BYTES &&
        rx_bytes <= SPI_POLLING_DMA_BUFFER_BYTES;
    if (!can_use_reserved_dma) {
        return __real_spi_device_polling_transmit(handle, trans_desc);
    }

    SemaphoreHandle_t mutex = polling_dma_mutex();
    if (mutex == NULL || xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_NO_MEM;
    }

    /* Preserve the original descriptor for pre/post callbacks that may use
     * extended fields (e.g., ESP-LCD's lcd_spi_trans_descriptor_t). Only swap
     * buffer pointers temporarily while holding the mutex. */
    void *orig_tx_buffer = trans_desc->tx_buffer;
    void *orig_rx_buffer = trans_desc->rx_buffer;
    uint32_t orig_flags = trans_desc->flags;

    if (has_tx) {
        memcpy(s_polling_dma_tx, trans_desc->tx_buffer, tx_bytes);
        trans_desc->tx_buffer = s_polling_dma_tx;
    }
    if (has_rx) {
        trans_desc->rx_buffer = s_polling_dma_rx;
    }
    trans_desc->flags &= ~SPI_TRANS_DMA_USE_PSRAM;

    const esp_err_t err =
        __real_spi_device_polling_transmit(handle, trans_desc);

    /* Restore original pointers regardless of success/failure before releasing
     * the mutex. Copy RX data only if the transfer succeeded. */
    if (err == ESP_OK && has_rx && orig_rx_buffer) {
        memcpy(orig_rx_buffer, s_polling_dma_rx, rx_bytes);
    }
    trans_desc->tx_buffer = orig_tx_buffer;
    trans_desc->rx_buffer = orig_rx_buffer;
    trans_desc->flags = orig_flags;

    xSemaphoreGive(mutex);
    return err;
}

esp_err_t __wrap_gdma_connect(gdma_channel_handle_t dma_chan,
                              gdma_trigger_t trig_periph)
{
    esp_err_t err = __real_gdma_connect(dma_chan, trig_periph);
    if (err != ESP_OK || trig_periph.periph != GDMA_TRIG_PERIPH_CAM) {
        return err;
    }

    err = gdma_set_priority(dma_chan, DVP_RX_DMA_PRIORITY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DVP RX DMA priority setup failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG,
             "DVP RX DMA priority=%d (cache line=%uB, capture pipeline unchanged)",
             DVP_RX_DMA_PRIORITY,
             (unsigned)CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE);
    return ESP_OK;
}
