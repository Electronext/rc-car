#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_attr.h"

#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "rc_config.h"
#include "as5048b_controls.h"
#include "status_led.h"
#include "reset_diag.h"

static const char *TAG = "RC";

enum {
    RC_MSG_CONTROL = 1,
    RC_MSG_HEARTBEAT = 2
};

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t sequence;
    uint8_t type;
    int16_t left;       // -1000 ... +1000
    int16_t right;      // -1000 ... +1000
    uint16_t speed;     // 0 ... 1000
    uint8_t flags;
} rc_packet_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t last_control_sequence;
    uint8_t type;
    uint16_t battery_mv;
    int8_t control_rssi;
    uint32_t failsafe_count;
    uint32_t sequence_skips;
    uint32_t max_control_gap_ms;
    int16_t left_pwm_permille;
    int16_t right_pwm_permille;
    uint8_t flags;      // bit 0 charging, bit 1 external power present
} rc_heartbeat_t;

static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};


/* ============================================================
 * Common WiFi / ESP-NOW
 * ============================================================ */

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(
        esp_wifi_set_channel(RC_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE)
    );
}


static void charge_status_init(void)
{
#if CHARGE_STATUS_ENABLED
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << CHARGE_STATUS_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&cfg));
#endif

#if !RC_TRANSMITTER
    gpio_config_t vusb_cfg = {
        .pin_bit_mask = 1ULL << VUSB_PRESENT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&vusb_cfg));
#endif
}


static bool external_power_present(void)
{
#if RC_TRANSMITTER
#if CHARGE_STATUS_ENABLED
    return gpio_get_level(CHARGE_STATUS_GPIO) ==
           CHARGE_STATUS_ACTIVE_LEVEL;
#else
    return false;
#endif
#else
    return gpio_get_level(VUSB_PRESENT_GPIO) ==
           VUSB_PRESENT_ACTIVE_LEVEL;
#endif
}


static bool is_charging(void)
{
#if CHARGE_STATUS_ENABLED
#if RC_TRANSMITTER
    return gpio_get_level(CHARGE_STATUS_GPIO) ==
           CHARGE_STATUS_ACTIVE_LEVEL;
#else
    return external_power_present() &&
           gpio_get_level(CHARGE_STATUS_GPIO) ==
               CHARGE_STATUS_ACTIVE_LEVEL;
#endif
#else
    return false;
#endif
}


static void battery_colour(int battery_mv,
                           uint8_t *red,
                           uint8_t *green,
                           uint8_t *blue)
{
    *blue = 0;

    if (battery_mv <= BATTERY_LED_RED_MV) {
        *red = 255;
        *green = 0;
        return;
    }

    if (battery_mv < BATTERY_LED_YELLOW_MV) {
        int span = BATTERY_LED_YELLOW_MV - BATTERY_LED_RED_MV;
        int pos = battery_mv - BATTERY_LED_RED_MV;

        *red = 255;
        *green = (uint8_t)((255 * pos) / span);
        return;
    }

    if (battery_mv < BATTERY_LED_GREEN_MV) {
        int span = BATTERY_LED_GREEN_MV - BATTERY_LED_YELLOW_MV;
        int pos = battery_mv - BATTERY_LED_YELLOW_MV;

        *red = (uint8_t)(255 - (255 * pos) / span);
        *green = 255;
        return;
    }

    *red = 0;
    *green = 255;
}


static uint8_t charging_breathe_blue(int64_t now_us)
{
    const float two_pi = 6.28318530718f;

    uint32_t phase_ms =
        (uint32_t)((now_us / 1000LL) % CHARGE_BREATHE_PERIOD_MS);

    float phase =
        (float)phase_ms / (float)CHARGE_BREATHE_PERIOD_MS;

    float envelope =
        0.5f - 0.5f * cosf(two_pi * phase);

    return (uint8_t)lroundf(255.0f * envelope);
}


static void render_status_led(bool charging,
                              bool linked,
                              int battery_mv,
                              int64_t now_us)
{
    if (charging) {
        if (linked) {
            status_led_set_rgb(0, 0, 255);
        } else {
            status_led_set_rgb(
                0,
                0,
                charging_breathe_blue(now_us)
            );
        }
        return;
    }

    uint8_t red, green, blue;

    battery_colour(
        battery_mv,
        &red,
        &green,
        &blue
    );

    if (linked) {
        status_led_set_rgb(red, green, blue);
        return;
    }

    uint32_t cycle_ms =
        STATUS_LED_ON_MS + STATUS_LED_OFF_MS;

    uint32_t phase_ms =
        (uint32_t)((now_us / 1000LL) % cycle_ms);

    if (phase_ms < STATUS_LED_ON_MS) {
        status_led_set_rgb(red, green, blue);
    } else {
        status_led_off();
    }
}


/* ============================================================
 * TRANSMITTER
 * ============================================================ */

#if RC_TRANSMITTER

#define TX_RTC_MAGIC 0x52544331UL

enum {
    TX_SLEEP_NONE = 0,
    TX_SLEEP_INACTIVITY = 1,
    TX_SLEEP_LOW_BATTERY = 2,
    TX_SLEEP_DISCONNECTED = 3
};

static const uint8_t tx_mac[ESP_NOW_ETH_ALEN] = RC_TX_MAC_INIT;
static const uint8_t rx_mac[ESP_NOW_ETH_ALEN] = RC_RX_MAC_INIT;

static volatile bool espnow_send_pending = false;
static volatile int64_t last_heartbeat_us = 0;
static volatile uint16_t last_heartbeat_sequence = 0;
static volatile uint16_t last_heartbeat_battery_mv = 0;
static volatile int last_heartbeat_control_rssi = 0;
static volatile uint32_t last_heartbeat_failsafe_count = 0;
static volatile uint32_t last_heartbeat_sequence_skips = 0;
static volatile uint32_t last_heartbeat_max_control_gap_ms = 0;
static volatile int16_t last_heartbeat_left_pwm_permille = 0;
static volatile int16_t last_heartbeat_right_pwm_permille = 0;

/* ESP-NOW link diagnostics (TX perspective). */
static volatile uint32_t tx_mac_send_ok = 0;
static volatile uint32_t tx_mac_send_fail = 0;
static volatile uint32_t tx_send_submit_err = 0;
static volatile uint32_t tx_send_deferred = 0;
static volatile int64_t last_tx_mac_success_us = 0;
static volatile int64_t tx_send_started_us = 0;
static volatile uint32_t tx_send_last_latency_us = 0;
static volatile uint32_t tx_send_max_latency_us = 0;
static volatile uint32_t tx_send_latency_over_40ms = 0;
static volatile uint32_t tx_send_latency_over_100ms = 0;
static volatile uint32_t tx_send_latency_over_250ms = 0;
static volatile uint32_t tx_heartbeat_rx_count = 0;
static volatile int tx_heartbeat_rssi = 0;
static volatile bool tx_heartbeat_rssi_valid = false;

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t battery_cali_handle = NULL;
static adc_channel_t battery_channel;
static adc_channel_t curvature_channel;
static adc_channel_t mode_switch_channel;

RTC_DATA_ATTR static uint32_t rtc_magic = 0;
RTC_DATA_ATTR static uint8_t rtc_sleep_reason = TX_SLEEP_NONE;
RTC_DATA_ATTR static uint16_t rtc_steering_raw = 0;
RTC_DATA_ATTR static uint16_t rtc_throttle_raw = 0;
RTC_DATA_ATTR static uint16_t rtc_curvature_raw = 0;
RTC_DATA_ATTR static int8_t rtc_mode_position = 0;


static int adc_read_channel(adc_channel_t channel)
{
    int raw = 0;
    esp_err_t err = ESP_FAIL;

    for (int attempt = 0;
         attempt < ADC_READ_RETRY_COUNT;
         attempt++) {

        err = adc_oneshot_read(
            adc_handle,
            channel,
            &raw
        );

        if (err == ESP_OK) {
            return raw;
        }

        if (err != ESP_ERR_TIMEOUT) {
            ESP_ERROR_CHECK(err);
        }

        vTaskDelay(1);
    }

    ESP_LOGE(
        TAG,
        "ADC channel %d timed out after %d retries",
        (int)channel,
        ADC_READ_RETRY_COUNT
    );

    ESP_ERROR_CHECK(err);
    return 0;
}


static int adc_average_channel(adc_channel_t channel, int samples)
{
    int64_t total = 0;

    for (int i = 0; i < samples; i++) {
        total += adc_read_channel(channel);
    }

    return (int)(total / samples);
}

static void adc_channel_init(int gpio,
                             adc_channel_t *channel,
                             const adc_oneshot_chan_cfg_t *cfg,
                             const char *name)
{
    adc_unit_t unit;

    ESP_ERROR_CHECK(
        adc_oneshot_io_to_channel(
            gpio,
            &unit,
            channel
        )
    );

    if (unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "%s GPIO%d is not on ADC1", name, gpio);
        abort();
    }

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            *channel,
            cfg
        )
    );
}


static void transmitter_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(&adc_cfg, &adc_handle)
    );

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    adc_channel_init(
        BATTERY_GPIO,
        &battery_channel,
        &chan_cfg,
        "Battery"
    );

    adc_channel_init(
        CURVATURE_POT_GPIO,
        &curvature_channel,
        &chan_cfg,
        "Curvature pot"
    );

    adc_channel_init(
        MODE_SWITCH_GPIO,
        &mode_switch_channel,
        &chan_cfg,
        "Mode switch"
    );

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = battery_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    ESP_ERROR_CHECK(
        adc_cali_create_scheme_curve_fitting(
            &cali_cfg,
            &battery_cali_handle
        )
    );
}


static int read_battery_mv(void)
{
    int raw = adc_average_channel(battery_channel, 32);
    int adc_mv = 0;

    ESP_ERROR_CHECK(
        adc_cali_raw_to_voltage(
            battery_cali_handle,
            raw,
            &adc_mv
        )
    );

    int64_t battery_mv =
        (int64_t)adc_mv *
        (BATTERY_DIVIDER_TOP_OHMS +
         BATTERY_DIVIDER_BOTTOM_OHMS);

    battery_mv =
        (battery_mv + BATTERY_DIVIDER_BOTTOM_OHMS / 2) /
        BATTERY_DIVIDER_BOTTOM_OHMS;

    return (int)battery_mv;
}

static int mode_switch_position_from_raw(int raw)
{
    if (raw <= MODE_SWITCH_ADC_LOW_MAX) {
        return -1;
    }

    if (raw >= MODE_SWITCH_ADC_HIGH_MIN) {
        return 1;
    }

    return 0;
}


static float steering_from_raw(uint16_t raw)
{
    float value;

    if (raw < STEERING_DEADBAND_LOW) {
        value =
            -(float)(STEERING_DEADBAND_LOW - raw) /
            (float)(STEERING_DEADBAND_LOW - STEERING_RAW_MIN);
    } else if (raw > STEERING_DEADBAND_HIGH) {
        value =
            (float)(raw - STEERING_DEADBAND_HIGH) /
            (float)(STEERING_RAW_MAX - STEERING_DEADBAND_HIGH);
    } else {
        value = 0.0f;
    }

    if (value < -1.0f) value = -1.0f;
    if (value >  1.0f) value =  1.0f;

#if STEERING_INVERT
    value = -value;
#endif

    return value;
}


static float throttle_from_raw(uint16_t raw)
{
    float value;

    if (raw < THROTTLE_NEUTRAL_LOW) {
        value =
            (float)(THROTTLE_NEUTRAL_LOW - raw) /
            (float)(THROTTLE_NEUTRAL_LOW - THROTTLE_RAW_FORWARD);
    } else if (raw > THROTTLE_NEUTRAL_HIGH) {
        value =
            -(float)(raw - THROTTLE_NEUTRAL_HIGH) /
            (float)(THROTTLE_RAW_REVERSE - THROTTLE_NEUTRAL_HIGH);
    } else {
        value = 0.0f;
    }

    if (value >  1.0f) value =  1.0f;
    if (value < -1.0f) value = -1.0f;

    return value;
}


static float curvature_exponent_from_raw(int raw)
{
    float p =
        (float)(raw - CURVATURE_POT_ADC_MIN) /
        (float)(CURVATURE_POT_ADC_MAX - CURVATURE_POT_ADC_MIN);

    if (p < 0.0f) p = 0.0f;
    if (p > 1.0f) p = 1.0f;

#if CURVATURE_POT_INVERT
    p = 1.0f - p;
#endif

    return CURVATURE_EXPONENT_MIN +
           p * (CURVATURE_EXPONENT_MAX - CURVATURE_EXPONENT_MIN);
}


static float expo_from_mode(int mode_position)
{
    if (mode_position < 0) {
        return EXPO_LEVEL_LOW;
    }

    if (mode_position > 0) {
        return EXPO_LEVEL_HIGH;
    }

    return EXPO_LEVEL_MEDIUM;
}


static float apply_expo(float value, float expo)
{
    if (expo < 0.0f) expo = 0.0f;
    if (expo > 1.0f) expo = 1.0f;

    return
        (1.0f - expo) * value +
        expo * value * value * value;
}


static float apply_speed_limit(float value, float limit)
{
    if (value == 0.0f || limit <= 0.0f) {
        return 0.0f;
    }

    if (limit >= 1.0f) {
        return value;
    }

    float magnitude = fabsf(value);

    if (magnitude > 1.0f) {
        magnitude = 1.0f;
    }

    /*
     * Soft top-end limiter:
     *
     *     y = L*x / (L + (1-L)*x)
     *
     * where x is command magnitude and L is the selected speed limit.
     *
     * This has unity slope at zero, so LOW/MEDIUM do not further
     * soften initial motor take-up, while x=1 still maps exactly to L.
     * It is monotonic for every 0 < L <= 1.
     */
    float limited =
        (limit * magnitude) /
        (limit + (1.0f - limit) * magnitude);

    return value < 0.0f ? -limited : limited;
}


typedef enum {
    STEERING_MODE_IDLE = 0,
    STEERING_MODE_PIVOT,
    STEERING_MODE_DRIVE
} steering_mode_t;


static const char *steering_mode_name(steering_mode_t mode)
{
    switch (mode) {
        case STEERING_MODE_IDLE:
            return "IDLE";
        case STEERING_MODE_PIVOT:
            return "PIVOT";
        case STEERING_MODE_DRIVE:
        default:
            return "DRIVE";
    }
}


/*
 * Normal drive mixer.
 *
 * Steering is normalized curvature, independent of signed velocity:
 *
 *     q = (1 - |s|) / (1 + |s|)
 *
 * q is the inner/outer wheel-speed ratio. Therefore:
 *   s = 0    -> q = 1, straight line
 *   |s| = 1 -> q = 0, inner wheel stopped
 *
 * Reversing velocity reverses both wheels without changing q, so the
 * same steering command follows the same geometric arc in reverse.
 */
static void arc_drive_mix(float steering,
                          float velocity,
                          float exponent,
                          float *curvature,
                          float *left,
                          float *right)
{
    float steer_mag = fabsf(steering);

    if (steer_mag > 1.0f) {
        steer_mag = 1.0f;
    }

    if (exponent < 1.0f) {
        exponent = 1.0f;
    }

    float k =
        1.0f - powf(1.0f - steer_mag, exponent);

    if (k > 1.0f) {
        k = 1.0f;
    }

    *curvature =
        steering < 0.0f ? -k : k;

    float inner_ratio =
        (1.0f - k) /
        (1.0f + k);

    if (steering > 0.0f) {
        *left = velocity;
        *right = velocity * inner_ratio;
    } else if (steering < 0.0f) {
        *left = velocity * inner_ratio;
        *right = velocity;
    } else {
        *left = velocity;
        *right = velocity;
    }
}


/*
 * Zero-throttle pivot. During this calibration the speed ceiling is
 * fixed at 100%, so pivot authority follows the expo-shaped steering.
 */
static void pivot_mix(float steering,
                      float speed_limit,
                      float *left,
                      float *right)
{
    float pivot =
        apply_speed_limit(steering, speed_limit);

    *left = pivot;
    *right = -pivot;
}


static void enter_timed_sleep(uint8_t reason,
                              uint32_t sleep_ms,
                              uint16_t steering_raw,
                              uint16_t throttle_raw,
                              uint16_t curvature_raw,
                              int mode_position)
{
    status_led_off();

    rtc_magic = TX_RTC_MAGIC;
    rtc_sleep_reason = reason;
    rtc_steering_raw = steering_raw;
    rtc_throttle_raw = throttle_raw;
    rtc_curvature_raw = curvature_raw;
    rtc_mode_position = (int8_t)mode_position;

    ESP_ERROR_CHECK(
        esp_sleep_enable_timer_wakeup(
            (uint64_t)sleep_ms * 1000ULL
        )
    );

#if CHARGE_STATUS_ENABLED
    ESP_ERROR_CHECK(
        esp_deep_sleep_enable_gpio_wakeup(
            1ULL << CHARGE_STATUS_GPIO,
            ESP_GPIO_WAKEUP_GPIO_LOW
        )
    );
#endif

    esp_deep_sleep_start();
}


static void low_battery_warning_and_sleep(int battery_mv,
                                          bool show_warning)
{
    ESP_LOGW(
        TAG,
        "Battery low: %d mV; wireless will not start",
        battery_mv
    );

    if (show_warning) {
        int64_t end_us =
            esp_timer_get_time() +
            (int64_t)LOW_BATTERY_WARNING_MS * 1000LL;

        while (esp_timer_get_time() < end_us) {
            status_led_set_rgb(255, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(STATUS_LED_ON_MS));
            status_led_off();
            vTaskDelay(pdMS_TO_TICKS(STATUS_LED_OFF_MS));
        }
    }

    enter_timed_sleep(
        TX_SLEEP_LOW_BATTERY,
        LOW_BATTERY_RECHECK_MS,
        0,
        0,
        0,
        0
    );
}


static bool controls_moved_since_sleep(uint16_t steering_raw,
                                       uint16_t throttle_raw,
                                       uint16_t curvature_raw,
                                       int mode_position)
{
    if (abs((int)steering_raw - (int)rtc_steering_raw) >=
        TX_WAKE_STEERING_COUNTS) {
        return true;
    }

    if (abs((int)throttle_raw - (int)rtc_throttle_raw) >=
        TX_WAKE_THROTTLE_COUNTS) {
        return true;
    }

    if (abs((int)curvature_raw - (int)rtc_curvature_raw) >=
        TX_WAKE_CURVATURE_COUNTS) {
        return true;
    }

    if (mode_position != rtc_mode_position) {
        return true;
    }

    return false;
}


static void send_cb(const wifi_tx_info_t *tx_info,
                    esp_now_send_status_t status)
{
    (void)tx_info;

    int64_t now = esp_timer_get_time();
    int64_t started_us = tx_send_started_us;

    if (started_us != 0 && now >= started_us) {
        uint32_t latency_us =
            (uint32_t)(now - started_us);

        tx_send_last_latency_us = latency_us;

        if (latency_us > tx_send_max_latency_us) {
            tx_send_max_latency_us = latency_us;
        }

        if (latency_us > 40000U) {
            tx_send_latency_over_40ms++;
        }
        if (latency_us > 100000U) {
            tx_send_latency_over_100ms++;
        }
        if (latency_us > 250000U) {
            tx_send_latency_over_250ms++;
        }
    }

    if (status == ESP_NOW_SEND_SUCCESS) {
        tx_mac_send_ok++;
        last_tx_mac_success_us = now;
    } else {
        tx_mac_send_fail++;
    }

    tx_send_started_us = 0;
    espnow_send_pending = false;
}


static void tx_recv_cb(const esp_now_recv_info_t *info,
                       const uint8_t *data,
                       int len)
{
    if (len != sizeof(rc_heartbeat_t)) {
        return;
    }

    if (memcmp(info->src_addr, rx_mac, ESP_NOW_ETH_ALEN) != 0) {
        return;
    }

    rc_heartbeat_t heartbeat;
    memcpy(&heartbeat, data, sizeof(heartbeat));

    if (heartbeat.magic != RC_MAGIC ||
        heartbeat.type != RC_MSG_HEARTBEAT) {
        return;
    }

    last_heartbeat_sequence = heartbeat.last_control_sequence;
    last_heartbeat_battery_mv = heartbeat.battery_mv;
    last_heartbeat_control_rssi = heartbeat.control_rssi;
    last_heartbeat_failsafe_count = heartbeat.failsafe_count;
    last_heartbeat_sequence_skips = heartbeat.sequence_skips;
    last_heartbeat_max_control_gap_ms = heartbeat.max_control_gap_ms;
    last_heartbeat_left_pwm_permille = heartbeat.left_pwm_permille;
    last_heartbeat_right_pwm_permille = heartbeat.right_pwm_permille;
    last_heartbeat_us = esp_timer_get_time();
    tx_heartbeat_rx_count++;

    if (info->rx_ctrl != NULL) {
        tx_heartbeat_rssi = info->rx_ctrl->rssi;
        tx_heartbeat_rssi_valid = true;
    }
}


static void status_task(void *arg)
{
    (void)arg;

    int battery_mv = read_battery_mv();
    int64_t last_battery_read_us = esp_timer_get_time();

    while (1) {
        int64_t now = esp_timer_get_time();

        if ((now - last_battery_read_us) >= 1000000LL) {
            battery_mv = read_battery_mv();
            last_battery_read_us = now;
        }

        bool charging = is_charging();

        bool linked =
            last_heartbeat_us != 0 &&
            (now - last_heartbeat_us) <=
                ((int64_t)RC_HEARTBEAT_TIMEOUT_MS * 1000LL);

        if (!charging &&
            battery_mv < BATTERY_LOW_CUTOFF_MV) {

            low_battery_warning_and_sleep(
                battery_mv,
                true
            );
        }

        render_status_led(
            charging,
            linked,
            battery_mv,
            now
        );

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


static void transmitter_task(void *arg)
{
    (void)arg;

    rc_packet_t packet = {
        .magic = RC_MAGIC,
        .type = RC_MSG_CONTROL
    };

    int64_t last_activity_us = esp_timer_get_time();
    int64_t disconnected_since_us = 0;

    /*
     * Start in DRIVE rather than IDLE so pivot steering is not armed
     * merely because the transmitter boots with steering already held.
     * Returning both controls to centre arms IDLE.
     */
    steering_mode_t steering_mode = STEERING_MODE_DRIVE;
    bool pivot_to_drive_blending = false;
    int64_t pivot_to_drive_start_us = 0;
    float pivot_blend_start_left = 0.0f;
    float pivot_blend_start_right = 0.0f;

    int activity_curvature_raw = adc_read_channel(curvature_channel);
    int activity_mode_position =
        mode_switch_position_from_raw(
            adc_read_channel(mode_switch_channel)
        );

    while (1) {
        rc_as5048b_sample_t steering_sample = {0};
        rc_as5048b_sample_t throttle_sample = {0};

        esp_err_t steering_err =
            rc_as5048b_read_steering(&steering_sample);

        esp_err_t throttle_err =
            rc_as5048b_read_throttle(&throttle_sample);

        int curvature_raw = adc_read_channel(curvature_channel);
        int mode_raw = adc_read_channel(mode_switch_channel);
        int mode_position =
            mode_switch_position_from_raw(mode_raw);

        bool controls_valid =
            steering_err == ESP_OK &&
            throttle_err == ESP_OK &&
            rc_as5048b_sample_valid(&steering_sample) &&
            rc_as5048b_sample_valid(&throttle_sample);

        float steering = 0.0f;
        float curvature = 0.0f;
        float throttle = 0.0f;
        float left = 0.0f;
        float right = 0.0f;
        float curvature_exponent =
            curvature_exponent_from_raw(curvature_raw);
        float expo = expo_from_mode(mode_position);
        float speed = CALIBRATION_SPEED_LIMIT;

        if (controls_valid) {
            steering = apply_expo(
                steering_from_raw(steering_sample.angle),
                expo
            );

            throttle = apply_expo(
                throttle_from_raw(throttle_sample.angle),
                expo
            );

            bool steering_neutral = steering == 0.0f;
            bool throttle_neutral = throttle == 0.0f;
            int64_t mix_now = esp_timer_get_time();

            /*
             * Pivot is armed only from a fully centred IDLE state.
             * DRIVE remains latched through throttle zero/reversal while
             * steering is held, so crossing zero cannot unexpectedly
             * become differential steering.
             */
            if (steering_mode == STEERING_MODE_IDLE) {
                if (!throttle_neutral) {
                    steering_mode = STEERING_MODE_DRIVE;
                } else if (!steering_neutral) {
                    steering_mode = STEERING_MODE_PIVOT;
                }
            } else if (steering_mode == STEERING_MODE_PIVOT) {
                if (!throttle_neutral) {
                    pivot_mix(
                        steering,
                        speed,
                        &pivot_blend_start_left,
                        &pivot_blend_start_right
                    );

                    steering_mode = STEERING_MODE_DRIVE;
                    pivot_to_drive_blending = true;
                    pivot_to_drive_start_us = mix_now;
                } else if (steering_neutral) {
                    steering_mode = STEERING_MODE_IDLE;
                }
            } else if (throttle_neutral && steering_neutral) {
                steering_mode = STEERING_MODE_IDLE;
                pivot_to_drive_blending = false;
            }

            if (steering_mode == STEERING_MODE_PIVOT) {
                pivot_mix(
                    steering,
                    speed,
                    &left,
                    &right
                );
            } else if (steering_mode == STEERING_MODE_DRIVE) {
                /*
                 * Apply the fixed calibration speed ceiling before mixing.
                 * Wheel ratio depends only on shaped steering curvature,
                 * so commanded radius remains independent of throttle.
                 */
                float velocity =
                    apply_speed_limit(throttle, speed);

                float drive_left = 0.0f;
                float drive_right = 0.0f;

                arc_drive_mix(
                    steering,
                    velocity,
                    curvature_exponent,
                    &curvature,
                    &drive_left,
                    &drive_right
                );

                if (pivot_to_drive_blending) {
                    int64_t elapsed_us =
                        mix_now - pivot_to_drive_start_us;
                    int64_t blend_us =
                        (int64_t)PIVOT_TO_DRIVE_BLEND_MS * 1000LL;

                    float blend =
                        blend_us <= 0
                            ? 1.0f
                            : (float)elapsed_us / (float)blend_us;

                    if (blend >= 1.0f) {
                        blend = 1.0f;
                        pivot_to_drive_blending = false;
                    } else if (blend < 0.0f) {
                        blend = 0.0f;
                    }

                    left =
                        pivot_blend_start_left +
                        blend * (drive_left - pivot_blend_start_left);

                    right =
                        pivot_blend_start_right +
                        blend * (drive_right - pivot_blend_start_right);
                } else {
                    left = drive_left;
                    right = drive_right;
                }
            } else {
                left = 0.0f;
                right = 0.0f;
            }
        } else {
            /*
             * Invalid controls must never leave a stale pivot/drive
             * transition armed.
             */
            pivot_to_drive_blending = false;
        }

        packet.left =
            (int16_t)lroundf(left * 1000.0f);
        packet.right =
            (int16_t)lroundf(right * 1000.0f);
        packet.speed =
            (uint16_t)lroundf(speed * 1000.0f);
        packet.flags = controls_valid ? 0 : 1;

        if (!espnow_send_pending) {
            uint16_t previous_sequence = packet.sequence;
            packet.sequence++;
            tx_send_started_us = esp_timer_get_time();
            espnow_send_pending = true;

            esp_err_t err = esp_now_send(
                broadcast_mac,
                (uint8_t *)&packet,
                sizeof(packet)
            );

            if (err != ESP_OK) {
                packet.sequence = previous_sequence;
                tx_send_started_us = 0;
                espnow_send_pending = false;
                tx_send_submit_err++;

                ESP_LOGW(
                    TAG,
                    "esp_now_send: %s",
                    esp_err_to_name(err)
                );
            }
        } else {
            tx_send_deferred++;
        }

        int64_t now = esp_timer_get_time();

        bool curvature_curve_changed =
            abs(curvature_raw - activity_curvature_raw) >=
                TX_WAKE_CURVATURE_COUNTS;

        bool mode_changed =
            mode_position != activity_mode_position;

        if (steering != 0.0f ||
            throttle != 0.0f ||
            curvature_curve_changed ||
            mode_changed) {

            last_activity_us = now;

            if (curvature_curve_changed) {
                activity_curvature_raw = curvature_raw;
            }

            if (mode_changed) {
                activity_mode_position = mode_position;
            }
        }

        bool charging = is_charging();

        bool linked =
            last_heartbeat_us != 0 &&
            (now - last_heartbeat_us) <=
                ((int64_t)RC_HEARTBEAT_TIMEOUT_MS * 1000LL);

        if (charging || linked) {
            disconnected_since_us = 0;
        } else if (disconnected_since_us == 0) {
            disconnected_since_us = now;
        }

        bool disconnected_timeout =
            !charging &&
            !linked &&
            disconnected_since_us != 0 &&
            (now - disconnected_since_us) >=
                ((int64_t)DISCONNECTED_SLEEP_MS * 1000LL);

        bool inactivity_timeout =
            !charging &&
            (now - last_activity_us) >=
                ((int64_t)TX_INACTIVITY_SLEEP_MS * 1000LL);

        if (disconnected_timeout || inactivity_timeout) {

            ESP_LOGI(
                TAG,
                "%s; entering deep sleep",
                disconnected_timeout
                    ? "TX disconnected timeout"
                    : "Transmitter inactive"
            );

            packet.sequence++;
            packet.left = 0;
            packet.right = 0;
            packet.flags = 1;

            esp_now_send(
                broadcast_mac,
                (uint8_t *)&packet,
                sizeof(packet)
            );

            vTaskDelay(pdMS_TO_TICKS(30));

            enter_timed_sleep(
                disconnected_timeout
                    ? TX_SLEEP_DISCONNECTED
                    : TX_SLEEP_INACTIVITY,
                TX_SLEEP_POLL_MS,
                steering_sample.angle,
                throttle_sample.angle,
                curvature_raw,
                mode_position
            );
        }

        static int log_div = 0;

        if (++log_div >= 20) {
            log_div = 0;

            int64_t heartbeat_age_ms =
                last_heartbeat_us == 0
                    ? -1
                    : (now - last_heartbeat_us) / 1000LL;

            ESP_LOGI(
                TAG,
                "steer=%+.3f k=%+.3f g=%.3f throttle=%+.3f expo=%.2f speed=%.2f | "
                "L=%+.3f R=%+.3f | mode=%d mix=%s%s | "
                "link=%s hbSeq=%u age=%lldms hbRSSI=%s%d ctrlRSSI=%d "
                "rxPWM=%+.3f/%+.3f rxVBAT=%.3fV fs=%lu skips=%lu gapMax=%lums | "
                "tx=%lu/%lu submitErr=%lu deferred=%lu hbRx=%lu | "
                "cb=%lu/%lums >40/100/250=%lu/%lu/%lu",
                steering,
                curvature,
                curvature_exponent,
                throttle,
                expo,
                speed,
                left,
                right,
                mode_position,
                steering_mode_name(steering_mode),
                pivot_to_drive_blending ? "/BLEND" : "",
                linked ? "OK" : "WAIT",
                (unsigned)last_heartbeat_sequence,
                (long long)heartbeat_age_ms,
                tx_heartbeat_rssi_valid ? "" : "?",
                tx_heartbeat_rssi_valid ? tx_heartbeat_rssi : 0,
                last_heartbeat_control_rssi,
                last_heartbeat_left_pwm_permille / 1000.0f,
                last_heartbeat_right_pwm_permille / 1000.0f,
                last_heartbeat_battery_mv / 1000.0f,
                (unsigned long)last_heartbeat_failsafe_count,
                (unsigned long)last_heartbeat_sequence_skips,
                (unsigned long)last_heartbeat_max_control_gap_ms,
                (unsigned long)tx_mac_send_ok,
                (unsigned long)tx_mac_send_fail,
                (unsigned long)tx_send_submit_err,
                (unsigned long)tx_send_deferred,
                (unsigned long)tx_heartbeat_rx_count,
                (unsigned long)(tx_send_last_latency_us / 1000U),
                (unsigned long)(tx_send_max_latency_us / 1000U),
                (unsigned long)tx_send_latency_over_40ms,
                (unsigned long)tx_send_latency_over_100ms,
                (unsigned long)tx_send_latency_over_250ms
            );
        }

        vTaskDelay(pdMS_TO_TICKS(RC_TX_PERIOD_MS));
    }
}


static void add_espnow_peer(const uint8_t *peer_mac)
{
    esp_now_peer_info_t peer = {0};

    memcpy(
        peer.peer_addr,
        peer_mac,
        ESP_NOW_ETH_ALEN
    );

    peer.channel = RC_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    ESP_ERROR_CHECK(
        esp_now_add_peer(&peer)
    );
}


static void log_local_mac(const char *role,
                          const uint8_t *expected)
{
    uint8_t actual[ESP_NOW_ETH_ALEN] = {0};

    ESP_ERROR_CHECK(
        esp_wifi_get_mac(WIFI_IF_STA, actual)
    );

    ESP_LOGI(
        TAG,
        "%s STA MAC %02X:%02X:%02X:%02X:%02X:%02X",
        role,
        actual[0], actual[1], actual[2],
        actual[3], actual[4], actual[5]
    );

    if (memcmp(actual, expected, ESP_NOW_ETH_ALEN) != 0) {
        ESP_LOGW(TAG, "%s MAC does not match configured board", role);
    }
}


static void transmitter_init(void)
{
    ESP_LOGI(TAG, "Starting TRANSMITTER");

    transmitter_adc_init();
    charge_status_init();
    status_led_init();

    int battery_mv = read_battery_mv();
    bool charging = is_charging();

    esp_sleep_wakeup_cause_t wake_cause =
        esp_sleep_get_wakeup_cause();

    bool timer_wake =
        wake_cause == ESP_SLEEP_WAKEUP_TIMER &&
        rtc_magic == TX_RTC_MAGIC;

    if (timer_wake &&
        rtc_sleep_reason == TX_SLEEP_LOW_BATTERY) {

        if (!charging &&
            battery_mv < BATTERY_LOW_RECOVER_MV) {

            low_battery_warning_and_sleep(
                battery_mv,
                false
            );
        }

        rtc_sleep_reason = TX_SLEEP_NONE;
    } else if (!charging &&
               battery_mv < BATTERY_LOW_CUTOFF_MV) {

        low_battery_warning_and_sleep(
            battery_mv,
            true
        );
    }

    /*
     * Prove the WS2812 and RGB order on a real startup without
     * wasting energy on the 1-second inactivity/low-battery timer
     * wake checks.
     */
    if (wake_cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
        status_led_self_test();
    }

    rc_as5048b_init();

    if (timer_wake &&
        (rtc_sleep_reason == TX_SLEEP_INACTIVITY ||
         rtc_sleep_reason == TX_SLEEP_DISCONNECTED) &&
        !charging) {

        rc_as5048b_sample_t steering_sample = {0};
        rc_as5048b_sample_t throttle_sample = {0};

        ESP_ERROR_CHECK(
            rc_as5048b_read_steering(&steering_sample)
        );

        ESP_ERROR_CHECK(
            rc_as5048b_read_throttle(&throttle_sample)
        );

        int curvature_raw = adc_read_channel(curvature_channel);
        int mode_position =
            mode_switch_position_from_raw(
                adc_read_channel(mode_switch_channel)
            );

        if (!controls_moved_since_sleep(
                steering_sample.angle,
                throttle_sample.angle,
                curvature_raw,
                mode_position)) {

            enter_timed_sleep(
                TX_SLEEP_INACTIVITY,
                TX_SLEEP_POLL_MS,
                rtc_steering_raw,
                rtc_throttle_raw,
                rtc_curvature_raw,
                rtc_mode_position
            );
        }

        rtc_sleep_reason = TX_SLEEP_NONE;
    }

    rtc_magic = 0;
    rtc_sleep_reason = TX_SLEEP_NONE;

    wifi_init();
    log_local_mac("TX", tx_mac);

    ESP_ERROR_CHECK(esp_now_init());

    ESP_ERROR_CHECK(
        esp_now_register_send_cb(send_cb)
    );

    ESP_ERROR_CHECK(
        esp_now_register_recv_cb(tx_recv_cb)
    );

    add_espnow_peer(broadcast_mac);

    ESP_LOGI(
        TAG,
        "Broadcast controls on channel %d; expecting heartbeat from "
        "%02X:%02X:%02X:%02X:%02X:%02X",
        RC_WIFI_CHANNEL,
        rx_mac[0], rx_mac[1], rx_mac[2],
        rx_mac[3], rx_mac[4], rx_mac[5]
    );

    xTaskCreate(
        transmitter_task,
        "transmitter",
        4096,
        NULL,
        5,
        NULL
    );

    xTaskCreate(
        status_task,
        "status_led",
        3072,
        NULL,
        4,
        NULL
    );
}

/* ============================================================
 * RECEIVER
 * ============================================================ */

#else

static const uint8_t tx_mac[ESP_NOW_ETH_ALEN] = RC_TX_MAC_INIT;
static const uint8_t rx_mac[ESP_NOW_ETH_ALEN] = RC_RX_MAC_INIT;

static QueueHandle_t packet_queue;

static volatile int64_t last_packet_us = 0;

static uint32_t rx_packet_count = 0;
static int64_t rx_last_log_us = 0;

/* ESP-NOW link diagnostics (RX perspective). */
static volatile int rx_control_rssi = 0;
static volatile bool rx_control_rssi_valid = false;
static volatile int rx_control_rssi_interval_min = 0;
static volatile bool rx_control_rssi_interval_min_valid = false;
static volatile uint32_t rx_max_packet_gap_ms = 0;
static volatile uint32_t rx_max_packet_gap_ever_ms = 0;
static volatile uint32_t rx_failsafe_count = 0;
static uint16_t rx_last_sequence = 0;
static bool rx_sequence_valid = false;
static uint32_t rx_sequence_skips = 0;
static volatile uint32_t rx_heartbeat_send_ok = 0;
static volatile uint32_t rx_heartbeat_send_fail = 0;
static volatile uint32_t rx_heartbeat_submit_err = 0;
static volatile bool rx_heartbeat_send_pending = false;
static volatile int rx_latest_battery_mv = 0;
static TaskHandle_t rx_heartbeat_task_handle = NULL;

#define RX_RTC_MAGIC 0x52585231UL

enum {
    RX_SLEEP_NONE = 0,
    RX_SLEEP_DISCONNECTED = 1,
    RX_SLEEP_LOW_BATTERY = 2
};

RTC_DATA_ATTR static uint32_t rx_rtc_magic = 0;
RTC_DATA_ATTR static uint8_t rx_sleep_reason = RX_SLEEP_NONE;

static bool rx_poll_wake = false;
static volatile bool rx_shutdown_pending = false;
static int64_t rx_radio_start_us = 0;

static adc_oneshot_unit_handle_t rx_adc_handle = NULL;
static adc_cali_handle_t rx_battery_cali_handle = NULL;
static adc_channel_t rx_battery_channel;


static int rx_adc_read_channel(adc_channel_t channel)
{
    int raw = 0;
    esp_err_t err = ESP_FAIL;

    for (int attempt = 0;
         attempt < ADC_READ_RETRY_COUNT;
         attempt++) {

        err = adc_oneshot_read(
            rx_adc_handle,
            channel,
            &raw
        );

        if (err == ESP_OK) {
            return raw;
        }

        if (err != ESP_ERR_TIMEOUT) {
            ESP_ERROR_CHECK(err);
        }

        vTaskDelay(1);
    }

    ESP_LOGE(
        TAG,
        "RX ADC channel %d timed out after %d retries",
        (int)channel,
        ADC_READ_RETRY_COUNT
    );

    ESP_ERROR_CHECK(err);
    return 0;
}


static int rx_read_battery_mv_samples(int samples)
{
    if (samples < 1) {
        samples = 1;
    }

    int64_t total = 0;

    for (int i = 0; i < samples; i++) {
        total += rx_adc_read_channel(rx_battery_channel);
    }

    int raw = (int)(total / samples);
    int adc_mv = 0;

    ESP_ERROR_CHECK(
        adc_cali_raw_to_voltage(
            rx_battery_cali_handle,
            raw,
            &adc_mv
        )
    );

    int64_t battery_mv =
        (int64_t)adc_mv *
        (BATTERY_DIVIDER_TOP_OHMS +
         BATTERY_DIVIDER_BOTTOM_OHMS);

    battery_mv =
        (battery_mv + BATTERY_DIVIDER_BOTTOM_OHMS / 2) /
        BATTERY_DIVIDER_BOTTOM_OHMS;

    return (int)battery_mv;
}


static int rx_read_battery_mv(void)
{
    return rx_read_battery_mv_samples(32);
}


static void receiver_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &adc_cfg,
            &rx_adc_handle
        )
    );

    adc_unit_t unit;

    ESP_ERROR_CHECK(
        adc_oneshot_io_to_channel(
            BATTERY_GPIO,
            &unit,
            &rx_battery_channel
        )
    );

    if (unit != ADC_UNIT_1) {
        ESP_LOGE(
            TAG,
            "RX battery GPIO%d is not on ADC1",
            BATTERY_GPIO
        );
        abort();
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            rx_adc_handle,
            rx_battery_channel,
            &chan_cfg
        )
    );

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = rx_battery_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    ESP_ERROR_CHECK(
        adc_cali_create_scheme_curve_fitting(
            &cali_cfg,
            &rx_battery_cali_handle
        )
    );
}


static void rx_enter_sleep(uint8_t reason,
                           uint32_t sleep_ms)
{
    status_led_off();

    rx_rtc_magic = RX_RTC_MAGIC;
    rx_sleep_reason = reason;

    ESP_ERROR_CHECK(
        esp_sleep_enable_timer_wakeup(
            (uint64_t)sleep_ms * 1000ULL
        )
    );

    ESP_ERROR_CHECK(
        esp_deep_sleep_enable_gpio_wakeup(
            1ULL << VUSB_PRESENT_GPIO,
            ESP_GPIO_WAKEUP_GPIO_HIGH
        )
    );

    esp_deep_sleep_start();
}


static void rx_low_battery_warning_and_sleep(int battery_mv,
                                             bool show_warning)
{
    ESP_LOGW(
        TAG,
        "RX battery low: %d mV; wireless will not start",
        battery_mv
    );

    if (show_warning) {
        int64_t end_us =
            esp_timer_get_time() +
            (int64_t)LOW_BATTERY_WARNING_MS * 1000LL;

        while (esp_timer_get_time() < end_us) {
            status_led_set_rgb(255, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(STATUS_LED_ON_MS));
            status_led_off();
            vTaskDelay(pdMS_TO_TICKS(STATUS_LED_OFF_MS));
        }
    }

    rx_enter_sleep(
        RX_SLEEP_LOW_BATTERY,
        LOW_BATTERY_RECHECK_MS
    );
}

/*
 * Four LEDC channels correspond directly to the four SA8302
 * inputs.
 */
enum {
    PWM_L_FWD = LEDC_CHANNEL_0,
    PWM_L_REV = LEDC_CHANNEL_1,
    PWM_R_FWD = LEDC_CHANNEL_2,
    PWM_R_REV = LEDC_CHANNEL_3
};

typedef struct {
    int8_t direction;
    int64_t boost_until_us;
    int16_t applied_pwm_permille;
} motor_state_t;

static motor_state_t left_motor_state = {0};
static motor_state_t right_motor_state = {0};


static void pwm_set(ledc_channel_t channel, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}


static void motors_stop(void)
{
    pwm_set(PWM_L_FWD, 0);
    pwm_set(PWM_L_REV, 0);
    pwm_set(PWM_R_FWD, 0);
    pwm_set(PWM_R_REV, 0);

    left_motor_state.direction = 0;
    left_motor_state.boost_until_us = 0;
    left_motor_state.applied_pwm_permille = 0;
    right_motor_state.direction = 0;
    right_motor_state.boost_until_us = 0;
    right_motor_state.applied_pwm_permille = 0;
}


static uint32_t motor_duty_from_demand(int16_t demand,
                                       motor_state_t *state)
{
    int direction =
        demand > 0 ? 1 :
        demand < 0 ? -1 : 0;

    if (direction == 0) {
        state->direction = 0;
        state->boost_until_us = 0;
        state->applied_pwm_permille = 0;
        return 0;
    }

    int64_t now = esp_timer_get_time();

    if (state->direction != direction) {
        state->direction = direction;
        state->boost_until_us =
            now + (int64_t)MOTOR_START_BOOST_MS * 1000LL;
    }

    float command =
        (float)abs(demand) / 1000.0f;

    if (command > 1.0f) {
        command = 1.0f;
    }

    /*
     * Compress the logical 0..1 demand into the motor's measured
     * usable running range. Any non-zero logical demand therefore
     * starts at MOTOR_PWM_RUN_MIN rather than wasting stick travel
     * inside the electrical/mechanical dead zone.
     */
    float duty_fraction =
        MOTOR_PWM_RUN_MIN +
        command * (1.0f - MOTOR_PWM_RUN_MIN);

    /*
     * Static friction is higher than running friction. Give a newly
     * started or reversed motor a short minimum-start pulse, then
     * allow it to settle as low as MOTOR_PWM_RUN_MIN.
     */
    if (now < state->boost_until_us &&
        duty_fraction < MOTOR_PWM_START_MIN) {

        duty_fraction = MOTOR_PWM_START_MIN;
    }

    if (duty_fraction > 1.0f) {
        duty_fraction = 1.0f;
    }

    return (uint32_t)lroundf(
        duty_fraction * (float)MOTOR_PWM_MAX
    );
}


static void set_one_motor(int16_t demand,
                          ledc_channel_t forward,
                          ledc_channel_t reverse,
                          bool invert,
                          motor_state_t *state)
{
    if (invert) {
        demand = -demand;
    }

    if (demand > 1000)  demand = 1000;
    if (demand < -1000) demand = -1000;

    uint32_t duty =
        motor_duty_from_demand(demand, state);

    int pwm_permille =
        (int)lroundf(
            ((float)duty * 1000.0f) /
            (float)MOTOR_PWM_MAX
        );

    state->applied_pwm_permille =
        (int16_t)(
            demand < 0
                ? -pwm_permille
                : pwm_permille
        );

    /*
     * Always turn the opposite bridge input OFF before applying PWM
     * to the desired direction.
     */
    if (demand > 0) {
        pwm_set(reverse, 0);
        pwm_set(forward, duty);
    } else if (demand < 0) {
        pwm_set(forward, 0);
        pwm_set(reverse, duty);
    } else {
        pwm_set(forward, 0);
        pwm_set(reverse, 0);
    }
}

static void set_motors(int16_t left, int16_t right)
{
    set_one_motor(
        left,
        PWM_L_FWD,
        PWM_L_REV,
        LEFT_INVERT,
        &left_motor_state
    );

    set_one_motor(
        right,
        PWM_R_FWD,
        PWM_R_REV,
        RIGHT_INVERT,
        &right_motor_state
    );
}





static void rx_heartbeat_send_cb(const wifi_tx_info_t *tx_info,
                                 esp_now_send_status_t status)
{
    (void)tx_info;

    if (status == ESP_NOW_SEND_SUCCESS) {
        rx_heartbeat_send_ok++;
    } else {
        rx_heartbeat_send_fail++;
    }

    rx_heartbeat_send_pending = false;
}


static void recv_cb(const esp_now_recv_info_t *info,
                    const uint8_t *data,
                    int len)
{
    if (len != sizeof(rc_packet_t)) {
        return;
    }

    if (memcmp(info->src_addr, tx_mac, ESP_NOW_ETH_ALEN) != 0) {
        return;
    }

    rc_packet_t packet;
    memcpy(&packet, data, sizeof(packet));

    if (packet.magic != RC_MAGIC ||
        packet.type != RC_MSG_CONTROL) {
        return;
    }

    if (info->rx_ctrl != NULL) {
        int rssi = info->rx_ctrl->rssi;

        rx_control_rssi = rssi;
        rx_control_rssi_valid = true;

        if (!rx_control_rssi_interval_min_valid ||
            rssi < rx_control_rssi_interval_min) {

            rx_control_rssi_interval_min = rssi;
            rx_control_rssi_interval_min_valid = true;
        }
    }

    if (rx_sequence_valid) {
        uint16_t delta =
            (uint16_t)(packet.sequence - rx_last_sequence);

        if (delta > 0x8000U &&
            packet.sequence <= 1024U) {
            /*
             * TX sequence numbers restart near zero after a TX reboot.
             * Treat that large backwards jump as a new sequence epoch
             * rather than ~65k lost control packets.
             */
        } else if (delta > 1U && delta <= 0x8000U) {
            rx_sequence_skips += (uint32_t)(delta - 1U);
        }
    }

    rx_last_sequence = packet.sequence;
    rx_sequence_valid = true;

    int64_t now = esp_timer_get_time();
    int64_t previous_packet_us = last_packet_us;

    if (previous_packet_us != 0) {
        uint32_t gap_ms =
            (uint32_t)((now - previous_packet_us) / 1000LL);

        if (gap_ms > rx_max_packet_gap_ms) {
            rx_max_packet_gap_ms = gap_ms;
        }

        if (gap_ms > rx_max_packet_gap_ever_ms) {
            rx_max_packet_gap_ever_ms = gap_ms;
        }
    }

    last_packet_us = now;

    if (rx_heartbeat_task_handle != NULL) {
        xTaskNotifyGive(rx_heartbeat_task_handle);
    }

    /*
     * Don't do motor peripheral work in the WiFi callback.
     * Just hand latest packet to the motor task.
     */
    xQueueOverwrite(packet_queue, &packet);

    rx_packet_count++;

    if ((now - rx_last_log_us) >= 200000) {   // 200 ms = 5 Hz
        rx_last_log_us = now;

        ESP_LOGI(
            TAG,
            "RX #%lu seq=%u rssi=%s%d skips=%lu | "
            "L=%+.3f R=%+.3f speed=%u | "
            "hbTX=%lu/%lu submitErr=%lu",
            (unsigned long)rx_packet_count,
            packet.sequence,
            rx_control_rssi_valid ? "" : "?",
            rx_control_rssi_valid ? rx_control_rssi : 0,
            (unsigned long)rx_sequence_skips,
            packet.left / 1000.0f,
            packet.right / 1000.0f,
            packet.speed,
            (unsigned long)rx_heartbeat_send_ok,
            (unsigned long)rx_heartbeat_send_fail,
            (unsigned long)rx_heartbeat_submit_err
        );
    }
}


static void rx_heartbeat_task(void *arg)
{
    (void)arg;

    int64_t next_due_us = 0;

    while (1) {
        /*
         * Control packets provide the timing reference. Sending the
         * heartbeat shortly afterward keeps the two broadcast streams
         * separated in time without relying on link-layer retries.
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!rx_sequence_valid || rx_shutdown_pending) {
            continue;
        }

        int64_t packet_us = last_packet_us;

        if (next_due_us == 0) {
            next_due_us = packet_us;
        }

        if (packet_us < next_due_us) {
            continue;
        }

        if (rx_heartbeat_send_pending) {
            continue;
        }

        TickType_t delay_ticks =
            pdMS_TO_TICKS(RC_HEARTBEAT_DELAY_MS);

        if (delay_ticks < 1) {
            delay_ticks = 1;
        }

        vTaskDelay(delay_ticks);

        if (rx_shutdown_pending || rx_heartbeat_send_pending) {
            continue;
        }

        rc_heartbeat_t heartbeat = {
            .magic = RC_MAGIC,
            .last_control_sequence = rx_last_sequence,
            .type = RC_MSG_HEARTBEAT,
            .battery_mv =
                (uint16_t)(rx_latest_battery_mv < 0
                    ? 0
                    : rx_latest_battery_mv),
            .control_rssi =
                (int8_t)(rx_control_rssi_valid
                    ? rx_control_rssi
                    : 0),
            .failsafe_count = rx_failsafe_count,
            .sequence_skips = rx_sequence_skips,
            .max_control_gap_ms = rx_max_packet_gap_ever_ms,
            .left_pwm_permille =
                left_motor_state.applied_pwm_permille,
            .right_pwm_permille =
                right_motor_state.applied_pwm_permille,
            .flags =
                (is_charging() ? 0x01U : 0U) |
                (external_power_present() ? 0x02U : 0U)
        };

        rx_heartbeat_send_pending = true;

        esp_err_t err = esp_now_send(
            broadcast_mac,
            (uint8_t *)&heartbeat,
            sizeof(heartbeat)
        );

        if (err != ESP_OK) {
            rx_heartbeat_send_pending = false;
            rx_heartbeat_submit_err++;

            ESP_LOGW(
                TAG,
                "Heartbeat broadcast failed: %s",
                esp_err_to_name(err)
            );

            continue;
        }

        do {
            next_due_us +=
                (int64_t)RC_HEARTBEAT_PERIOD_MS * 1000LL;
        } while (next_due_us <= packet_us);
    }
}


static void motor_task(void *arg)
{
    rc_packet_t packet;

    bool failsafe_active = true;
    static bool rx_failsafe_active = true;

    while (1) {

        if (xQueueReceive(
                packet_queue,
                &packet,
                pdMS_TO_TICKS(10))) {

            if (rx_shutdown_pending) {
                motors_stop();
                continue;
            }

            set_motors(packet.left, packet.right);

            failsafe_active = false;
            if (rx_failsafe_active) {
                ESP_LOGI(TAG, "RX link active");
                rx_failsafe_active = false;
            }
        }


        int64_t now = esp_timer_get_time();

        if (!failsafe_active &&
            (now - last_packet_us) > ((int64_t)RC_FAILSAFE_MS * 1000)) {

            if (!rx_failsafe_active) {
                rx_failsafe_count++;

                ESP_LOGW(TAG, "RX FAILSAFE - no packet for %d ms",
                        RC_FAILSAFE_MS);
                rx_failsafe_active = true;
            }

            motors_stop();

            failsafe_active = true;

            //ESP_LOGW(TAG, "FAILSAFE - motors stopped");
        }
    }
}


static void rx_status_task(void *arg)
{
    (void)arg;

    int battery_mv = rx_read_battery_mv();
    int64_t now = esp_timer_get_time();
    int64_t last_battery_read_us = now;
    int64_t disconnected_since_us = now;
    int64_t last_link_diag_us = now;

    uint32_t last_sequence_skips = rx_sequence_skips;
    uint32_t last_failsafe_count = rx_failsafe_count;
    uint32_t last_heartbeat_send_fail = rx_heartbeat_send_fail;
    uint32_t last_heartbeat_submit_err = rx_heartbeat_submit_err;

    while (1) {
        now = esp_timer_get_time();

        bool battery_sampled = false;

        if ((now - last_battery_read_us) >=
            ((int64_t)RX_DIAG_BATTERY_SAMPLE_MS * 1000LL)) {

            battery_mv =
                rx_read_battery_mv_samples(
                    RX_DIAG_BATTERY_SAMPLE_COUNT
                );

            last_battery_read_us = now;
            rx_latest_battery_mv = battery_mv;
            battery_sampled = true;
        }

        bool external_power = external_power_present();
        bool charging = is_charging();

        bool linked =
            last_packet_us != 0 &&
            (now - last_packet_us) <=
                ((int64_t)RC_LINK_TIMEOUT_MS * 1000LL);

        if (battery_sampled) {
            reset_diag_runtime_sample(
                battery_mv,
                linked,
                charging,
                rx_packet_count
            );
        }

        if ((now - last_link_diag_us) >=
            ((int64_t)RX_LINK_DIAG_INTERVAL_MS * 1000LL)) {

            uint32_t sequence_skips = rx_sequence_skips;
            uint32_t failsafe_count = rx_failsafe_count;
            uint32_t heartbeat_send_fail = rx_heartbeat_send_fail;
            uint32_t heartbeat_submit_err = rx_heartbeat_submit_err;

            uint32_t sequence_skips_delta =
                sequence_skips - last_sequence_skips;
            uint32_t failsafe_delta =
                failsafe_count - last_failsafe_count;
            uint32_t heartbeat_send_fail_delta =
                heartbeat_send_fail - last_heartbeat_send_fail;
            uint32_t heartbeat_submit_err_delta =
                heartbeat_submit_err - last_heartbeat_submit_err;

            uint32_t max_gap_ms = rx_max_packet_gap_ms;

            if (last_packet_us != 0) {
                uint32_t current_gap_ms =
                    (uint32_t)((now - last_packet_us) / 1000LL);

                if (current_gap_ms > max_gap_ms) {
                    max_gap_ms = current_gap_ms;
                }
            }

            bool anomaly =
                sequence_skips_delta != 0 ||
                failsafe_delta != 0 ||
                heartbeat_send_fail_delta != 0 ||
                heartbeat_submit_err_delta != 0;

            if (anomaly) {
                reset_diag_record_link_summary(
                    battery_mv,
                    rx_control_rssi,
                    rx_control_rssi_interval_min,
                    rx_control_rssi_valid &&
                        rx_control_rssi_interval_min_valid,
                    max_gap_ms,
                    rx_last_sequence,
                    rx_packet_count,
                    sequence_skips,
                    sequence_skips_delta,
                    failsafe_count,
                    failsafe_delta,
                    rx_heartbeat_send_ok,
                    heartbeat_send_fail,
                    heartbeat_send_fail_delta,
                    heartbeat_submit_err,
                    heartbeat_submit_err_delta
                );
            }

            last_sequence_skips = sequence_skips;
            last_failsafe_count = failsafe_count;
            last_heartbeat_send_fail = heartbeat_send_fail;
            last_heartbeat_submit_err = heartbeat_submit_err;

            rx_max_packet_gap_ms = 0;

            if (rx_control_rssi_valid) {
                rx_control_rssi_interval_min = rx_control_rssi;
                rx_control_rssi_interval_min_valid = true;
            } else {
                rx_control_rssi_interval_min = 0;
                rx_control_rssi_interval_min_valid = false;
            }

            last_link_diag_us = now;
        }

        if (linked) {
            rx_poll_wake = false;
            disconnected_since_us = 0;
        } else if (external_power) {
            disconnected_since_us = 0;
        } else if (disconnected_since_us == 0) {
            disconnected_since_us = now;
        }

        if (!external_power &&
            battery_mv < BATTERY_LOW_CUTOFF_MV) {

            rx_shutdown_pending = true;
            motors_stop();

            rx_low_battery_warning_and_sleep(
                battery_mv,
                true
            );
        }

        render_status_led(
            charging,
            linked,
            battery_mv,
            now
        );

        if (!external_power && !linked) {
            bool poll_window_expired =
                rx_poll_wake &&
                (now - rx_radio_start_us) >=
                    ((int64_t)RX_POLL_LISTEN_MS * 1000LL);

            bool disconnected_timeout =
                !rx_poll_wake &&
                disconnected_since_us != 0 &&
                (now - disconnected_since_us) >=
                    ((int64_t)DISCONNECTED_SLEEP_MS * 1000LL);

            if (poll_window_expired ||
                disconnected_timeout) {

                ESP_LOGI(
                    TAG,
                    "%s; RX entering deep sleep",
                    poll_window_expired
                        ? "TX not found during poll"
                        : "RX disconnected timeout"
                );

                rx_shutdown_pending = true;
                motors_stop();

                rx_enter_sleep(
                    RX_SLEEP_DISCONNECTED,
                    RX_SLEEP_POLL_MS
                );
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


static void pwm_channel_init(ledc_channel_t channel,
                             int gpio)
{
    ledc_channel_config_t cfg = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ESP_ERROR_CHECK(
        ledc_channel_config(&cfg)
    );
}


static void receiver_init(void)
{
    ESP_LOGI(TAG, "Starting RECEIVER");

    receiver_adc_init();
    charge_status_init();
    status_led_init();

    int battery_mv = rx_read_battery_mv();
    rx_latest_battery_mv = battery_mv;
    bool external_power = external_power_present();
    bool charging = is_charging();

    esp_sleep_wakeup_cause_t wake_cause =
        esp_sleep_get_wakeup_cause();

    reset_diag_record_boot(
        battery_mv,
        charging,
        rx_sleep_reason
    );

    bool timer_wake =
        wake_cause == ESP_SLEEP_WAKEUP_TIMER &&
        rx_rtc_magic == RX_RTC_MAGIC;

    if (timer_wake &&
        rx_sleep_reason == RX_SLEEP_LOW_BATTERY) {

        if (!external_power &&
            battery_mv < BATTERY_LOW_RECOVER_MV) {

            rx_low_battery_warning_and_sleep(
                battery_mv,
                false
            );
        }
    } else if (!external_power &&
               battery_mv < BATTERY_LOW_CUTOFF_MV) {

        rx_low_battery_warning_and_sleep(
            battery_mv,
            true
        );
    }

    if (wake_cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
        status_led_self_test();
    }

    rx_poll_wake =
        timer_wake &&
        rx_sleep_reason == RX_SLEEP_DISCONNECTED &&
        !external_power;

    rx_rtc_magic = 0;
    rx_sleep_reason = RX_SLEEP_NONE;
    rx_shutdown_pending = false;

    reset_diag_start_console_task();

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(
        ledc_timer_config(&timer)
    );

    pwm_channel_init(
        PWM_L_FWD,
        MOTOR_L_FWD_GPIO
    );

    pwm_channel_init(
        PWM_L_REV,
        MOTOR_L_REV_GPIO
    );

    pwm_channel_init(
        PWM_R_FWD,
        MOTOR_R_FWD_GPIO
    );

    pwm_channel_init(
        PWM_R_REV,
        MOTOR_R_REV_GPIO
    );

    motors_stop();

    packet_queue = xQueueCreate(
        1,
        sizeof(rc_packet_t)
    );

    if (!packet_queue) {
        ESP_LOGE(TAG, "Could not create packet queue");
        abort();
    }


    wifi_init();

    uint8_t actual_mac[ESP_NOW_ETH_ALEN] = {0};
    ESP_ERROR_CHECK(
        esp_wifi_get_mac(WIFI_IF_STA, actual_mac)
    );

    ESP_LOGI(
        TAG,
        "RX STA MAC %02X:%02X:%02X:%02X:%02X:%02X",
        actual_mac[0], actual_mac[1], actual_mac[2],
        actual_mac[3], actual_mac[4], actual_mac[5]
    );

    if (memcmp(actual_mac, rx_mac, ESP_NOW_ETH_ALEN) != 0) {
        ESP_LOGW(TAG, "RX MAC does not match configured board");
    }

    ESP_ERROR_CHECK(esp_now_init());

    ESP_ERROR_CHECK(
        esp_now_register_send_cb(rx_heartbeat_send_cb)
    );

    esp_now_peer_info_t peer = {0};

    memcpy(
        peer.peer_addr,
        broadcast_mac,
        ESP_NOW_ETH_ALEN
    );

    peer.channel = RC_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    ESP_ERROR_CHECK(
        esp_now_add_peer(&peer)
    );

    ESP_ERROR_CHECK(
        esp_now_register_recv_cb(recv_cb)
    );

    ESP_LOGI(
        TAG,
        "Listening for TX %02X:%02X:%02X:%02X:%02X:%02X "
        "and broadcasting %d Hz heartbeat on channel %d",
        tx_mac[0], tx_mac[1], tx_mac[2],
        tx_mac[3], tx_mac[4], tx_mac[5],
        1000 / RC_HEARTBEAT_PERIOD_MS,
        RC_WIFI_CHANNEL
    );

    rx_radio_start_us = esp_timer_get_time();

    if (rx_poll_wake) {
        ESP_LOGI(
            TAG,
            "RX poll wake: listening for TX for %d ms",
            RX_POLL_LISTEN_MS
        );
    }

    xTaskCreate(
        rx_heartbeat_task,
        "rx_heartbeat",
        3072,
        NULL,
        5,
        &rx_heartbeat_task_handle
    );

    xTaskCreate(
        motor_task,
        "motor",
        4096,
        NULL,
        6,
        NULL
    );

    xTaskCreate(
        rx_status_task,
        "rx_status",
        3072,
        NULL,
        4,
        NULL
    );
}

#endif


#if !RC_TRANSMITTER
static void receiver_motor_pins_safe_early(void)
{
    const uint64_t motor_mask =
        (1ULL << MOTOR_L_FWD_GPIO) |
        (1ULL << MOTOR_L_REV_GPIO) |
        (1ULL << MOTOR_R_FWD_GPIO) |
        (1ULL << MOTOR_R_REV_GPIO);

    /*
     * Preload the output latches LOW before enabling output drive.
     * This minimises any software-controlled transient when the pins
     * change from reset/high-Z state to GPIO outputs.
     */
    gpio_set_level(MOTOR_L_FWD_GPIO, 0);
    gpio_set_level(MOTOR_L_REV_GPIO, 0);
    gpio_set_level(MOTOR_R_FWD_GPIO, 0);
    gpio_set_level(MOTOR_R_REV_GPIO, 0);

    gpio_config_t cfg = {
        .pin_bit_mask = motor_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&cfg));

    gpio_set_level(MOTOR_L_FWD_GPIO, 0);
    gpio_set_level(MOTOR_L_REV_GPIO, 0);
    gpio_set_level(MOTOR_R_FWD_GPIO, 0);
    gpio_set_level(MOTOR_R_REV_GPIO, 0);
}
#endif


/* ============================================================
 * app_main
 * ============================================================ */

void app_main(void)
{
#if !RC_TRANSMITTER
    /*
     * Put the H-bridge inputs into their safe state before NVS,
     * Wi-Fi, queues, PWM or any other application initialisation.
     */
    receiver_motor_pins_safe_early();
#endif

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }


#if RC_TRANSMITTER
    transmitter_init();
#else
    receiver_init();
#endif
}