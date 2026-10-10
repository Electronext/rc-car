#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_now.h"
#include "rc_radio.h"
#include "rc_protocol.h"

// Conventional receiver bring-up: radio and encoder diagnostics only.
// No motor command path is enabled until sensor travel and safety are calibrated.
#define RC_MAGIC 0x52434331UL
#define RC_CHANNEL 6
#define RC_FAILSAFE_MS 240
#define DRIVE_FWD GPIO_NUM_10
#define DRIVE_REV GPIO_NUM_5
#define STEER_LEFT GPIO_NUM_0
#define STEER_RIGHT GPIO_NUM_7
#define SENSOR_SDA GPIO_NUM_2
#define SENSOR_SCL GPIO_NUM_6
#define SENSOR_ADDR 0x40
#define SENSOR_AGC_REG 0xFA

static const char *TAG = "RX_CONVENTIONAL";
static volatile int64_t last_control_us;
static volatile uint16_t last_sequence;
static volatile int16_t last_left, last_right;
static volatile uint32_t valid_frames;
static i2c_master_dev_handle_t sensor;

static void outputs_safe(void)
{
    const gpio_num_t pins[] = { DRIVE_FWD, DRIVE_REV, STEER_LEFT, STEER_RIGHT };
    for (unsigned i = 0; i < sizeof(pins)/sizeof(pins[0]); ++i) {
        gpio_reset_pin(pins[i]);
        gpio_set_level(pins[i], 0);
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);
    }
}

static void receive_cb(const esp_now_recv_info_t *info,
                       const uint8_t *data, int len)
{
    static const uint8_t tx_mac[6] = {0x88,0x56,0xA6,0x58,0x57,0xF8};
    if (!info || !info->src_addr || !data ||
        memcmp(info->src_addr, tx_mac, 6) != 0 ||
        len != sizeof(rc_packet_t)) return;
    rc_packet_t p;
    memcpy(&p, data, sizeof(p));
    if (p.magic != RC_MAGIC || p.type != RC_MSG_CONTROL ||
        (p.flags & RC_CONTROL_FLAG_INVALID)) return;
    last_sequence = p.sequence;
    last_left = p.left;
    last_right = p.right;
    last_control_us = esp_timer_get_time();
    valid_frames++;
}

static void sensor_init(void)
{
    i2c_master_bus_handle_t bus;
    i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SENSOR_SDA,
        .scl_io_num = SENSOR_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus));
    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SENSOR_ADDR,
        .scl_speed_hz = 400000
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev, &sensor));
}

void app_main(void)
{
    outputs_safe(); // First action: both direction inputs of both bridges LOW.
    ESP_LOGW(TAG, "DIAGNOSTIC ONLY: motor outputs disabled");
    sensor_init();
    rc_radio_wifi_init(RC_CHANNEL);
    ESP_ERROR_CHECK(rc_radio_init());
    ESP_ERROR_CHECK(rc_radio_register_recv_cb(receive_cb));

    while (1) {
        uint8_t reg = SENSOR_AGC_REG;
        uint8_t bytes[6] = {0};
        esp_err_t err = i2c_master_transmit_receive(
            sensor, &reg, 1, bytes, sizeof(bytes), 50);
        int64_t now = esp_timer_get_time();
        bool linked = rc_radio_is_recent(now, last_control_us, RC_FAILSAFE_MS);
        if (err == ESP_OK) {
            uint16_t angle = ((uint16_t)bytes[4] << 6) | (bytes[5] & 0x3f);
            uint16_t magnitude = ((uint16_t)bytes[2] << 6) | (bytes[3] & 0x3f);
            uint8_t diag = bytes[1];
            bool valid = (diag & 0x01) && !(diag & 0x0e);
            ESP_LOGI(TAG, "encoder angle=%u mag=%u agc=%u diag=0x%02x valid=%d; radio=%s seq=%u L=%d R=%d frames=%lu; MOTORS OFF",
                     angle, magnitude, bytes[0], diag, valid,
                     linked ? "linked" : "timeout", last_sequence,
                     last_left, last_right, (unsigned long)valid_frames);
        } else {
            ESP_LOGW(TAG, "AS5048B read failed: %s; radio=%s; MOTORS OFF",
                     esp_err_to_name(err), linked ? "linked" : "timeout");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
