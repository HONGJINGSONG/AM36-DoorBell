#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Configure one initialized SPI host so RX GDMA accepts exact byte lengths.
 * This is used only by the LR20xx host; it does not change the LCD SPI bus. */
esp_err_t app_spi_polling_dma_prepare(spi_host_device_t host_id);

/* Give one initialized SPI host's TX GDMA priority over the other DMA channels
 * so a long PSRAM-sourced transfer keeps its share of MSPI. This is used only
 * by the LCD host; it does not change the LR20xx SPI bus. */
esp_err_t app_spi_tx_dma_prioritize_psram(spi_host_device_t host_id);

#ifdef __cplusplus
}
#endif
