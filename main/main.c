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
#include "as5048b_sanity.h"
#include "as5048b_controls.h"
#include "status_led.h"

static const char *TAG = "RC";

enum {
    RC_MSG_CONTROL = 1,
    RC_MSG_ACK = 2
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
    uint16_t sequence;
    uint8_t type;
} rc_ack_t;


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

    ESP_ERROR_CHECK(
        esp_wifi_set_channel(RC_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE)
    );
}


/* ============================================================
 * TRANSMITTER
 * ============================================================ */

#if RC_TRANSMITTER

#define TX_RTC_MAGIC 0x52544331UL

enum {
    TX_SLEEP_NONE = 0,
    TX_SLEEP_INACTIVITY = 1,
    TX_SLEEP_LOW_BATTERY = 2
};

static const uint8_t tx_mac[ESP_NOW_ETH_ALEN] = RC_TX_MAC_INIT;
static const uint8_t rx_mac[ESP_NOW_ETH_ALEN] = RC_RX_MAC_INIT;

static volatile bool espnow_send_pending = false;
static volatile int64_t last_ack_us = 0;
static volatile uint16_t last_ack_sequence = 0;

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t battery_cali_handle = NULL;
static adc_channel_t battery_channel;
static adc_channel_t speed_channel;
static adc_channel_t mode_switch_channel;

RTC_DATA_ATTR static uint32_t rtc_magic = 0;
RTC_DATA_ATTR static uint8_t rtc_sleep_reason = TX_SLEEP_NONE;
RTC_DATA_ATTR static uint16_t rtc_steering_raw = 0;
RTC_DATA_ATTR static uint16_t rtc_throttle_raw = 0;
RTC_DATA_ATTR static uint16_t rtc_speed_raw = 0;
RTC_DATA_ATTR static int8_t rtc_mode_position = 0;


static esp_err_t adc_read_channel(adc_channel_t channel,
                                  int *raw)
{
    esp_err_t err = ESP_FAIL;

    for (int attempt = 0;
         attempt < ADC_READ_RETRY_COUNT;
         attempt++) {

        err = adc_oneshot_read(
            adc_handle,
            channel,
            raw
        );

        if (err == ESP_OK) {
            return ESP_OK;
        }

        if (err != ESP_ERR_TIMEOUT) {
            return err;
        }

        vTaskDelay(1);
    }

    return err;
}


static esp_err_t adc_average_channel(adc_channel_t channel,
                                     int samples,
                                     int *average)
{
    int64_t total = 0;

    for (int i = 0; i < samples; i++) {
        int raw = 0;

        esp_err_t err =
            adc_read_channel(channel, &raw);

        if (err != ESP_OK) {
            return err;
        }

        total += raw;
    }

    *average = (int)(total / samples);

    return ESP_OK;
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
        SPEED_GPIO,
        &speed_channel,
        &chan_cfg,
        "Speed pot"
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
            (float)THROTTLE_COUNTS_PER_UNIT;
    } else if (raw > THROTTLE_NEUTRAL_HIGH) {
        value =
            -(float)(raw - THROTTLE_NEUTRAL_HIGH) /
            (float)THROTTLE_COUNTS_PER_UNIT;
    } else {
        value = 0.0f;
    }

    if (value >  1.0f) value =  1.0f;
    if (value < -1.0f) value = -1.0f;

    return value;
}


static float speed_scale_from_raw(int raw)
{
    float p =
        (float)(raw - SPEED_POT_ADC_MIN) /
        (float)(SPEED_POT_ADC_MAX - SPEED_POT_ADC_MIN);

    if (p < 0.0f) p = 0.0f;
    if (p > 1.0f) p = 1.0f;

    return SPEED_MIN + p * (1.0f - SPEED_MIN);
}


static void skid_steer_mix(float steering,
                           float throttle,
                           float *left,
                           float *right)
{
    float steer_mag = fabsf(steering);
    float throttle_mag = fabsf(throttle);
    float reduction = TURN_INNER_REDUCTION_FULL_THROTTLE;

    if (steer_mag > 1.0f) steer_mag = 1.0f;
    if (throttle_mag > 1.0f) throttle_mag = 1.0f;

    if (reduction < 0.0f) reduction = 0.0f;
    if (reduction > 1.0f) reduction = 1.0f;

    float mean =
        throttle * (1.0f - 0.5f * reduction * steer_mag);

    float differential_gain =
        (1.0f - throttle_mag) +
        throttle_mag * (0.5f * reduction);

    /*
     * Steering right / clockwise is negative. Negating it here makes
     * right steering produce L > R, i.e. clockwise yaw, regardless
     * of whether the vehicle is moving forward or reverse.
     */
    float differential =
        -steering * differential_gain;

    float l = mean + differential;
    float r = mean - differential;

    if (l >  1.0f) l =  1.0f;
    if (l < -1.0f) l = -1.0f;
    if (r >  1.0f) r =  1.0f;
    if (r < -1.0f) r = -1.0f;

    *left = l;
    *right = r;
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


static void enter_timed_sleep(uint8_t reason,
                              uint32_t sleep_ms,
                              uint16_t steering_raw,
                              uint16_t throttle_raw,
                              uint16_t speed_raw,
                              int mode_position)
{
    status_led_off();

    rtc_magic = TX_RTC_MAGIC;
    rtc_sleep_reason = reason;
    rtc_steering_raw = steering_raw;
    rtc_throttle_raw = throttle_raw;
    rtc_speed_raw = speed_raw;
    rtc_mode_position = (int8_t)mode_position;

    ESP_ERROR_CHECK(
        esp_sleep_enable_timer_wakeup(
            (uint64_t)sleep_ms * 1000ULL
        )
    );

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
                                       uint16_t speed_raw,
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

    if (abs((int)speed_raw - (int)rtc_speed_raw) >=
        TX_WAKE_SPEED_COUNTS) {
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
    (void)status;
    espnow_send_pending = false;
}


static void tx_recv_cb(const esp_now_recv_info_t *info,
                       const uint8_t *data,
                       int len)
{
    if (len != sizeof(rc_ack_t)) {
        return;
    }

    if (memcmp(info->src_addr, rx_mac, ESP_NOW_ETH_ALEN) != 0) {
        return;
    }

    rc_ack_t ack;
    memcpy(&ack, data, sizeof(ack));

    if (ack.magic != RC_MAGIC || ack.type != RC_MSG_ACK) {
        return;
    }

    last_ack_sequence = ack.sequence;
    last_ack_us = esp_timer_get_time();
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

        uint8_t red, green, blue;

        battery_colour(
            battery_mv,
            &red,
            &green,
            &blue
        );

        bool linked =
            last_ack_us != 0 &&
            (now - last_ack_us) <=
                ((int64_t)RC_LINK_TIMEOUT_MS * 1000LL);

        if (linked) {
            status_led_set_rgb(red, green, blue);
        } else {
            uint32_t cycle_ms =
                STATUS_LED_ON_MS + STATUS_LED_OFF_MS;

            uint32_t phase_ms =
                (uint32_t)((now / 1000LL) % cycle_ms);

            if (phase_ms < STATUS_LED_ON_MS) {
                status_led_set_rgb(red, green, blue);
            } else {
                status_led_off();
            }
        }

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
    int activity_speed_raw = adc_read_channel(speed_channel);
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

        int speed_raw = adc_read_channel(speed_channel);
        int mode_raw = adc_read_channel(mode_switch_channel);
        int mode_position =
            mode_switch_position_from_raw(mode_raw);

        bool controls_valid =
            steering_err == ESP_OK &&
            throttle_err == ESP_OK &&
            rc_as5048b_sample_valid(&steering_sample) &&
            rc_as5048b_sample_valid(&throttle_sample);

        float steering = 0.0f;
        float throttle = 0.0f;
        float left = 0.0f;
        float right = 0.0f;

        if (controls_valid) {
            steering = steering_from_raw(steering_sample.angle);
            throttle = throttle_from_raw(throttle_sample.angle);

            skid_steer_mix(
                steering,
                throttle,
                &left,
                &right
            );
        }

        float speed = speed_scale_from_raw(speed_raw);

        left *= speed;
        right *= speed;

        packet.sequence++;
        packet.left =
            (int16_t)lroundf(left * 1000.0f);
        packet.right =
            (int16_t)lroundf(right * 1000.0f);
        packet.speed =
            (uint16_t)lroundf(speed * 1000.0f);
        packet.flags = controls_valid ? 0 : 1;

        if (!espnow_send_pending) {
            espnow_send_pending = true;

            esp_err_t err = esp_now_send(
                rx_mac,
                (uint8_t *)&packet,
                sizeof(packet)
            );

            if (err != ESP_OK) {
                espnow_send_pending = false;

                ESP_LOGW(
                    TAG,
                    "esp_now_send: %s",
                    esp_err_to_name(err)
                );
            }
        }

        int64_t now = esp_timer_get_time();

        bool speed_changed =
            abs(speed_raw - activity_speed_raw) >= TX_WAKE_SPEED_COUNTS;

        bool mode_changed =
            mode_position != activity_mode_position;

        if (steering != 0.0f ||
            throttle != 0.0f ||
            speed_changed ||
            mode_changed) {

            last_activity_us = now;

            if (speed_changed) {
                activity_speed_raw = speed_raw;
            }

            if (mode_changed) {
                activity_mode_position = mode_position;
            }
        }

        if ((now - last_activity_us) >=
            ((int64_t)TX_INACTIVITY_SLEEP_MS * 1000LL)) {

            ESP_LOGI(TAG, "Transmitter inactive; entering deep sleep");

            packet.sequence++;
            packet.left = 0;
            packet.right = 0;
            packet.flags = 1;

            esp_now_send(
                rx_mac,
                (uint8_t *)&packet,
                sizeof(packet)
            );

            vTaskDelay(pdMS_TO_TICKS(30));

            enter_timed_sleep(
                TX_SLEEP_INACTIVITY,
                TX_SLEEP_POLL_MS,
                steering_sample.angle,
                throttle_sample.angle,
                speed_raw,
                mode_position
            );
        }

        static int log_div = 0;

        if (++log_div >= 20) {
            log_div = 0;

            bool linked =
                last_ack_us != 0 &&
                (now - last_ack_us) <=
                    ((int64_t)RC_LINK_TIMEOUT_MS * 1000LL);

            ESP_LOGI(
                TAG,
                "steer=%+.3f throttle=%+.3f speed=%.3f | "
                "L=%+.3f R=%+.3f | mode=%d | link=%s ack=%u",
                steering,
                throttle,
                speed,
                left,
                right,
                mode_position,
                linked ? "OK" : "WAIT",
                (unsigned)last_ack_sequence
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

#if AS5048B_SANITY_TEST
    as5048b_sanity_init();
    return;
#endif

    transmitter_adc_init();
    status_led_init();

    int battery_mv = read_battery_mv();

    esp_sleep_wakeup_cause_t wake_cause =
        esp_sleep_get_wakeup_cause();

    bool timer_wake =
        wake_cause == ESP_SLEEP_WAKEUP_TIMER &&
        rtc_magic == TX_RTC_MAGIC;

    if (timer_wake &&
        rtc_sleep_reason == TX_SLEEP_LOW_BATTERY) {

        if (battery_mv < BATTERY_LOW_RECOVER_MV) {
            low_battery_warning_and_sleep(
                battery_mv,
                false
            );
        }

        rtc_sleep_reason = TX_SLEEP_NONE;
    } else if (battery_mv < BATTERY_LOW_CUTOFF_MV) {
        low_battery_warning_and_sleep(
            battery_mv,
            true
        );
    }

    rc_as5048b_init();

    if (timer_wake &&
        rtc_sleep_reason == TX_SLEEP_INACTIVITY) {

        rc_as5048b_sample_t steering_sample = {0};
        rc_as5048b_sample_t throttle_sample = {0};

        ESP_ERROR_CHECK(
            rc_as5048b_read_steering(&steering_sample)
        );

        ESP_ERROR_CHECK(
            rc_as5048b_read_throttle(&throttle_sample)
        );

        int speed_raw = adc_read_channel(speed_channel);
        int mode_position =
            mode_switch_position_from_raw(
                adc_read_channel(mode_switch_channel)
            );

        if (!controls_moved_since_sleep(
                steering_sample.angle,
                throttle_sample.angle,
                speed_raw,
                mode_position)) {

            enter_timed_sleep(
                TX_SLEEP_INACTIVITY,
                TX_SLEEP_POLL_MS,
                rtc_steering_raw,
                rtc_throttle_raw,
                rtc_speed_raw,
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

    add_espnow_peer(rx_mac);

    ESP_LOGI(
        TAG,
        "Paired RX %02X:%02X:%02X:%02X:%02X:%02X",
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
}


static void set_one_motor(int16_t demand,
                          ledc_channel_t forward,
                          ledc_channel_t reverse,
                          bool invert)
{
    if (invert) {
        demand = -demand;
    }

    if (demand > 1000)  demand = 1000;
    if (demand < -1000) demand = -1000;

    uint32_t duty =
        ((uint32_t)abs(demand) * MOTOR_PWM_MAX) / 1000;


    /*
     * IMPORTANT:
     *
     * Always turn the opposite bridge input OFF before
     * applying PWM to the desired direction.
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
        LEFT_INVERT
    );

    set_one_motor(
        right,
        PWM_R_FWD,
        PWM_R_REV,
        RIGHT_INVERT
    );
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

    last_packet_us = esp_timer_get_time();

    /*
     * Don't do motor peripheral work in the WiFi callback.
     * Just hand latest packet to the motor task.
     */
    xQueueOverwrite(packet_queue, &packet);

    rx_packet_count++;

    int64_t now = esp_timer_get_time();

    if ((now - rx_last_log_us) >= 200000) {   // 200 ms = 5 Hz
        rx_last_log_us = now;

        ESP_LOGI(
            TAG,
            "RX #%lu seq=%u | L=%+.3f R=%+.3f | speed=%u",
            (unsigned long)rx_packet_count,
            packet.sequence,
            packet.left / 1000.0f,
            packet.right / 1000.0f,
            packet.speed
        );
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

            set_motors(packet.left, packet.right);

            rc_ack_t ack = {
                .magic = RC_MAGIC,
                .sequence = packet.sequence,
                .type = RC_MSG_ACK
            };

            esp_err_t ack_err = esp_now_send(
                tx_mac,
                (uint8_t *)&ack,
                sizeof(ack)
            );

            if (ack_err != ESP_OK) {
                ESP_LOGW(
                    TAG,
                    "ACK send failed: %s",
                    esp_err_to_name(ack_err)
                );
            }

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

    esp_now_peer_info_t peer = {0};

    memcpy(
        peer.peer_addr,
        tx_mac,
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
        "Paired TX %02X:%02X:%02X:%02X:%02X:%02X",
        tx_mac[0], tx_mac[1], tx_mac[2],
        tx_mac[3], tx_mac[4], tx_mac[5]
    );


    xTaskCreate(
        motor_task,
        "motor",
        4096,
        NULL,
        6,
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