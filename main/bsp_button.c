/*
 * Buttons on the L-LRMAM36-FANN4-DK01:
 *   BOOT   : independent GPIO0 button, active low (strap/download)
 *   K1     : KEY_ADC (GPIO5)  ~0.82 V, BSP_BTN_PTT
 *   K2     : KEY_ADC          ~1.65 V, BSP_BTN_USER1
 *   K4     : KEY_ADC          ~2.41 V, BSP_BTN_VOL_UP
 *   K5     : KEY_ADC          ~1.11 V, BSP_BTN_VOL_DN
 *   K3     : hardware power key, not connected to this input driver
 *
 * The ADC ladder is a 10 k pull-up on VDD_3V3 and a button-specific series
 * resistor to GND, so when no button is pressed the line sits near 3.3 V.
 *
 * A FreeRTOS task polls GPIO0 and the ADC channel at 50 Hz, debounces two
 * consecutive reads, and fires a callback on each edge.
 */

#include "bsp.h"

#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "bsp_btn";

#define POLL_PERIOD_MS          20
#define DEBOUNCE_COUNT          2

#define ADC_UNIT                BSP_KEY_ADC_UNIT
#define ADC_CHAN                BSP_KEY_ADC_CHANNEL
#define ADC_ATTEN               ADC_ATTEN_DB_12
#define ADC_BITWIDTH            ADC_BITWIDTH_DEFAULT

/* Expected voltages (mV) per button. */
typedef struct {
    bsp_btn_id_t id;
    int          mv;
} adc_btn_t;

static const adc_btn_t s_adc_btns[] = {
    { BSP_BTN_PTT,    820  },   /* K1 */
    { BSP_BTN_VOL_DN, 1110 },   /* K5 */
    { BSP_BTN_USER1,  1650 },   /* K2 */
    { BSP_BTN_VOL_UP, 2410 },   /* K4 */
};

/* A pressed button must be within this window of its target voltage. */
#define ADC_MATCH_WINDOW_MV     250
/* Above this threshold, no button is pressed. */
#define ADC_IDLE_THRESHOLD_MV   2900

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cal;
static bool                      s_cal_valid;

static bsp_btn_cb_t s_cb;
static void        *s_cb_user;

/* Set while the KEY_ADC pad is handed to the digital GPIO peripheral as a
 * light-sleep wake source. The polling task runs at a HIGHER priority than the
 * radio task that arms the wake, so without this it could preempt the arming
 * sequence, call adc_oneshot_read(), put the pad back into analog mode and
 * silently disable the wake source we just armed. */
static volatile bool s_sleep_wake_armed;

/* Debounced state per button id. */
static bool    s_btn_state  [BSP_BTN_COUNT];
static uint8_t s_btn_streak [BSP_BTN_COUNT];
static bool    s_btn_candidate[BSP_BTN_COUNT];

static int read_key_mv(void)
{
    int raw;
    if (adc_oneshot_read(s_adc, ADC_CHAN, &raw) != ESP_OK) return -1;
    if (s_cal_valid) {
        int mv;
        if (adc_cali_raw_to_voltage(s_cal, raw, &mv) == ESP_OK) return mv;
    }
    /* Fallback: raw * 3300 / 4095. Good enough for debouncing. */
    return raw * 3300 / ((1 << 12) - 1);
}

static bsp_btn_id_t classify_adc(int mv)
{
    if (mv < 0 || mv >= ADC_IDLE_THRESHOLD_MV) return BSP_BTN_COUNT;
    for (size_t i = 0; i < sizeof(s_adc_btns) / sizeof(s_adc_btns[0]); i++) {
        if (abs(mv - s_adc_btns[i].mv) <= ADC_MATCH_WINDOW_MV) {
            return s_adc_btns[i].id;
        }
    }
    return BSP_BTN_COUNT;   /* ambiguous / in-between reading */
}

static void fire(bsp_btn_id_t id, bool pressed)
{
    ESP_LOGD(TAG, "btn %d %s", id, pressed ? "down" : "up");
    if (s_cb) s_cb(id, pressed, s_cb_user);
}

static void update_button(bsp_btn_id_t id, bool candidate)
{
    if (candidate == s_btn_candidate[id]) {
        if (s_btn_streak[id] < 0xFF) s_btn_streak[id]++;
    } else {
        s_btn_candidate[id] = candidate;
        s_btn_streak[id]    = 1;
    }
    if (s_btn_streak[id] >= DEBOUNCE_COUNT && candidate != s_btn_state[id]) {
        s_btn_state[id] = candidate;
        fire(id, candidate);
    }
}

static void poll_task(void *arg)
{
    (void)arg;
    TickType_t next = xTaskGetTickCount();

    while (1) {
        /* GPIO boot button */
        bool boot_pressed = gpio_get_level(BSP_BOOT_KEY_GPIO) == 0;
        update_button(BSP_BTN_BOOT, boot_pressed);

        /* ADC ladder: at most one button can be pressed at a time, so we
         * set the active one true and everyone else false. Skipped while the
         * pad is armed as a wake source — reading it would take it back to
         * analog mode. That window is only the few instructions before the
         * caller's light sleep, so at most one poll is lost. */
        if (!s_sleep_wake_armed) {
            int mv = read_key_mv();
            bsp_btn_id_t active = classify_adc(mv);
            for (size_t i = 0; i < sizeof(s_adc_btns) / sizeof(s_adc_btns[0]); i++) {
                bsp_btn_id_t id = s_adc_btns[i].id;
                update_button(id, id == active);
            }
        }

        vTaskDelayUntil(&next, pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}

static esp_err_t adc_setup(void)
{
    adc_oneshot_unit_init_cfg_t u_cfg = { .unit_id = ADC_UNIT };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&u_cfg, &s_adc), TAG, "adc unit");

    adc_oneshot_chan_cfg_t c_cfg = {
        .bitwidth = ADC_BITWIDTH,
        .atten    = ADC_ATTEN,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, ADC_CHAN, &c_cfg),
                        TAG, "adc chan");

    /* Calibration is optional; fall back to linear mapping if unavailable. */
    adc_cali_curve_fitting_config_t cal_cfg = {
        .unit_id  = ADC_UNIT,
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(&cal_cfg, &s_cal) == ESP_OK) {
        s_cal_valid = true;
    } else {
        ESP_LOGW(TAG, "ADC calibration not available, using raw mapping");
    }
    return ESP_OK;
}

bsp_btn_id_t bsp_button_sample_adc(void)
{
    if (s_adc == NULL) return BSP_BTN_COUNT;
    return classify_adc(read_key_mv());
}

esp_err_t bsp_button_arm_sleep_wakeup(void)
{
    /* Without the ADC there is no way to tell which key woke us, so an armed
     * pad would just be a noise-triggered wake source. */
    if (s_adc == NULL) return ESP_ERR_INVALID_STATE;

    /* Claim the pad before touching it, so a poll that lands mid-sequence skips
     * its ADC read instead of undoing the arming. */
    s_sleep_wake_armed = true;

    /* Take the pad back from ADC1. adc_oneshot_read() leaves it in analog mode
     * with the digital input buffer disabled, and a disabled input buffer sees
     * no level at all, so the wake source would be armed on a dead pin. */
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BSP_KEY_ADC_GPIO,
        .mode         = GPIO_MODE_INPUT,
        /* The ladder's own 10 k to VDD_3V3 holds the idle level; an internal
         * pull would fight the button-specific series resistor and push the
         * pressed voltage further away from V_IL. */
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err == ESP_OK) {
        /* A key already held means the pad is already at the wake level. Arming
         * would make esp_light_sleep_start() return instantly and spin the
         * caller's sleep loop for as long as the key is down. The polling task
         * is running anyway, so decline and let the press come through the
         * normal path. */
        if (gpio_get_level(BSP_KEY_ADC_GPIO) == 0) {
            err = ESP_ERR_INVALID_STATE;
        } else {
            err = gpio_wakeup_enable(BSP_KEY_ADC_GPIO, GPIO_INTR_LOW_LEVEL);
            if (err == ESP_OK) err = esp_sleep_enable_gpio_wakeup();
        }
    }

    /* Every failure path gives the pad straight back to the ADC: an armed flag
     * with no armed wake source would blind the polling task for good. */
    if (err != ESP_OK) s_sleep_wake_armed = false;
    return err;
}

void bsp_button_disarm_sleep_wakeup(void)
{
    gpio_wakeup_disable(BSP_KEY_ADC_GPIO);
    /* Put the pad back under ADC1. Re-running the channel config is what undoes
     * the digital-input setup from arming; leaving it digital would let the
     * pull-less input float against the ladder and skew every later reading. */
    if (s_adc != NULL) {
        adc_oneshot_chan_cfg_t c_cfg = {
            .bitwidth = ADC_BITWIDTH,
            .atten    = ADC_ATTEN,
        };
        esp_err_t err = adc_oneshot_config_channel(s_adc, ADC_CHAN, &c_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "key pad back to ADC failed: %s", esp_err_to_name(err));
        }
    }
    s_sleep_wake_armed = false;
}

esp_err_t bsp_button_init(bsp_btn_cb_t cb, void *user)
{
    s_cb      = cb;
    s_cb_user = user;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BSP_BOOT_KEY_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio0");
    ESP_RETURN_ON_ERROR(adc_setup(),     TAG, "adc");

    BaseType_t ok = xTaskCreate(poll_task, "bsp_btn", 3072, NULL, 5, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
