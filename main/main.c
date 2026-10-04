#include <stdio.h>
#include <string.h>
#include <math.h>

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

#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_adc/adc_oneshot.h"

#include "rc_config.h"
#include "as5048b_sanity.h"

static const char *TAG = "RC";

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t sequence;
    int16_t left;       // -1000 ... +1000
    int16_t right;      // -1000 ... +1000
    uint16_t speed;     // 0 ... 1000
    uint8_t flags;
} rc_packet_t;


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

static volatile bool espnow_send_pending = false;

static adc_oneshot_unit_handle_t adc_handle;

static adc_channel_t joy_x_channel;
static adc_channel_t joy_y_channel;

#if USE_SPEED_POT
    static adc_channel_t speed_channel;
#endif

static int joy_x_center = 2048;
static int joy_y_center = 2048;



// Broadcast keeps initial setup very simple.
// We can switch to receiver-specific MAC addressing later.
static const uint8_t broadcast_addr[ESP_NOW_ETH_ALEN] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

static void send_cb(
    const wifi_tx_info_t *tx_info,
    esp_now_send_status_t status)
{
    espnow_send_pending = false;
}

static int adc_read_channel(adc_channel_t channel)
{
    int raw = 0;

    ESP_ERROR_CHECK(
        adc_oneshot_read(adc_handle, channel, &raw)
    );

    return raw;
}


static int adc_average_channel(adc_channel_t channel, int samples)
{
    int64_t total = 0;

    for (int i = 0; i < samples; i++) {
        total += adc_read_channel(channel);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    return (int)(total / samples);
}


/*
 * Convert ADC reading into approximately -1 ... +1 using
 * the measured centre point.
 *
 * Separate scaling either side of centre compensates for
 * imperfect joystick centring.
 */
static float joystick_axis(int raw, int center)
{
    float value;

    if (raw >= center) {
        value = (float)(raw - center) /
                (float)(4095 - center);
    } else {
        value = (float)(raw - center) /
                (float)center;
    }

    if (value > 1.0f)  value = 1.0f;
    if (value < -1.0f) value = -1.0f;

    /*
     * Deadband around centre, followed by rescaling so that
     * the usable range still runs continuously from 0 to 1.
     */
    float a = fabsf(value);

    if (a <= JOYSTICK_DEADBAND) {
        return 0.0f;
    }

    a = (a - JOYSTICK_DEADBAND) /
        (1.0f - JOYSTICK_DEADBAND);

    return copysignf(a, value);
}


static float read_speed_scale(void)
{
#if USE_SPEED_POT
    int raw = adc_read_channel(speed_channel);

    float p =
        (float)(raw - SPEED_POT_ADC_MIN) /
        (float)(SPEED_POT_ADC_MAX - SPEED_POT_ADC_MIN);

    if (p < 0.0f) p = 0.0f;
    if (p > 1.0f) p = 1.0f;

    return SPEED_MIN + p * (1.0f - SPEED_MIN);
#else
    return 1.0f;
#endif
}


static void joystick_mix(float x, float y,
                         float *left, float *right)
{
    /*
     * Arcade -> tank mixing:
     *
     * forward       L+, R+
     * reverse       L-, R-
     * right turn    L+, R-
     * left turn     L-, R+
     */
    float l = y + x;
    float r = y - x;

    // Preserve ratio but ensure neither exceeds magnitude 1.
    float maximum = fmaxf(fabsf(l), fabsf(r));

    if (maximum > 1.0f) {
        l /= maximum;
        r /= maximum;
    }

    *left = l;
    *right = r;
}


static void enter_deep_sleep(void)
{
    ESP_LOGI(TAG, "Entering deep sleep");

    /*
     * No GPIO wake source is configured here: GPIO4 is now the
     * three-position analogue selector, not the old pushbutton.
     *
     * The final inactivity/low-battery policy will configure the
     * appropriate periodic wake source before calling this helper.
     */
    esp_deep_sleep_start();
}

static void transmitter_task(void *arg)
{
    rc_packet_t packet = {
        .magic = RC_MAGIC
    };

    while (1) {

        int x_raw = adc_read_channel(joy_x_channel);
        int y_raw = adc_read_channel(joy_y_channel);
        
        float x = joystick_axis(x_raw, joy_x_center);
        float y = joystick_axis(y_raw, joy_y_center);

        /*
         * Depending on physical KY-023 orientation, uncomment
         * either/both of these after the first bench test.
         */
         x = -x;
         y = -y;

        float left, right;
        joystick_mix(x, y, &left, &right);

        float speed = read_speed_scale();

        left  *= speed;
        right *= speed;

        packet.sequence++;
        packet.left  = (int16_t)lroundf(left  * 1000.0f);
        packet.right = (int16_t)lroundf(right * 1000.0f);
        packet.speed = (uint16_t)lroundf(speed * 1000.0f);

        if (!espnow_send_pending) {

            espnow_send_pending = true;

            esp_err_t err = esp_now_send(
                broadcast_addr,
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

        static int log_div = 0;

        if (++log_div >= 20) {
            log_div = 0;

            ESP_LOGI(TAG,
                    "ADC X=%4d Y=%4d | axis X=%+.3f Y=%+.3f | "
                    "speed=%.3f | L=%+.3f R=%+.3f",
                    x_raw,
                    y_raw,
                    x,
                    y,
                    speed,
                    left,
                    right);
        }


        vTaskDelay(pdMS_TO_TICKS(RC_TX_PERIOD_MS));
    }
}


static void transmitter_init(void)
{
    ESP_LOGI(TAG, "Starting TRANSMITTER");

#if AS5048B_SANITY_TEST
    /*
     * Bench-test mode only: validate the two magnetic sensors
     * without starting joystick ADC handling, Wi-Fi or ESP-NOW.
     */
    as5048b_sanity_init();
    return;
#endif

    /*
     * Create ADC1 oneshot unit.
     */
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(&adc_cfg, &adc_handle)
    );


    /*
     * Common configuration for all three analogue inputs.
     */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    adc_unit_t unit;


    /*
     * Joystick X.
     */
    ESP_ERROR_CHECK(
        adc_oneshot_io_to_channel(
            JOY_X_GPIO,
            &unit,
            &joy_x_channel
        )
    );

    if (unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "JOY_X_GPIO is not on ADC1");
        abort();
    }

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            joy_x_channel,
            &chan_cfg
        )
    );


    /*
     * Joystick Y.
     */
    ESP_ERROR_CHECK(
        adc_oneshot_io_to_channel(
            JOY_Y_GPIO,
            &unit,
            &joy_y_channel
        )
    );

    if (unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "JOY_Y_GPIO is not on ADC1");
        abort();
    }

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            joy_y_channel,
            &chan_cfg
        )
    );


#if USE_SPEED_POT

    /*
     * Speed-limit potentiometer.
     */
    ESP_ERROR_CHECK(
        adc_oneshot_io_to_channel(
            SPEED_GPIO,
            &unit,
            &speed_channel
        )
    );

    if (unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "SPEED_GPIO is not on ADC1");
        abort();
    }

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            speed_channel,
            &chan_cfg
        )
    );

#endif


    /*
     * Calibrate joystick centre.
     *
     * Stick must be released/centred during startup.
     */
    ESP_LOGI(TAG, "Calibrating joystick centre...");

    vTaskDelay(pdMS_TO_TICKS(300));

    joy_x_center =
        adc_average_channel(joy_x_channel, 64);

    joy_y_center =
        adc_average_channel(joy_y_channel, 64);

    ESP_LOGI(
        TAG,
        "Joystick centre X=%d Y=%d",
        joy_x_center,
        joy_y_center
    );


    /*
     * Start Wi-Fi radio and ESP-NOW.
     */
    wifi_init();

    ESP_ERROR_CHECK(esp_now_init());

    ESP_ERROR_CHECK(
        esp_now_register_send_cb(send_cb)
    );

    /*
     * For initial bring-up we're broadcasting rather than
     * pairing to the receiver's MAC.
     */
    esp_now_peer_info_t peer = {0};

    memcpy(
        peer.peer_addr,
        broadcast_addr,
        ESP_NOW_ETH_ALEN
    );

    peer.channel = RC_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    ESP_ERROR_CHECK(
        esp_now_add_peer(&peer)
    );


    /*
     * Start the 100-Hz controller task.
     */
    xTaskCreate(
        transmitter_task,
        "transmitter",
        4096,
        NULL,
        5,
        NULL
    );
}


/* ============================================================
 * RECEIVER
 * ============================================================ */

#else

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

    rc_packet_t packet;
    memcpy(&packet, data, sizeof(packet));

    if (packet.magic != RC_MAGIC) {
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

    ESP_ERROR_CHECK(esp_now_init());

    ESP_ERROR_CHECK(
        esp_now_register_recv_cb(recv_cb)
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


/* ============================================================
 * app_main
 * ============================================================ */

void app_main(void)
{
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