#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

#include "rc_config.h"
#include "status_led.h"

#define WS2812_RMT_RESOLUTION_HZ 10000000U

static rmt_channel_handle_t led_channel = NULL;
static rmt_encoder_handle_t led_encoder = NULL;

static uint8_t scale_component(uint8_t value)
{
    return (uint8_t)(((uint16_t)value * STATUS_LED_BRIGHTNESS) / 255U);
}

void status_led_init(void)
{
    if (led_channel != NULL) {
        return;
    }

    rmt_tx_channel_config_t channel_cfg = {
        .gpio_num = STATUS_LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = WS2812_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
        .flags.invert_out = false,
        .flags.with_dma = false
    };

    ESP_ERROR_CHECK(
        rmt_new_tx_channel(&channel_cfg, &led_channel)
    );

    /*
     * 10 MHz RMT clock => 100 ns per tick.
     *
     * WS2812 nominal timing:
     *   0: ~0.4 us high, ~0.85 us low
     *   1: ~0.8 us high, ~0.45 us low
     */
    rmt_bytes_encoder_config_t encoder_cfg = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 4,
            .level1 = 0,
            .duration1 = 9
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 8,
            .level1 = 0,
            .duration1 = 5
        },
        .flags.msb_first = 1
    };

    ESP_ERROR_CHECK(
        rmt_new_bytes_encoder(&encoder_cfg, &led_encoder)
    );

    ESP_ERROR_CHECK(rmt_enable(led_channel));

    status_led_off();
}

void status_led_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (led_channel == NULL || led_encoder == NULL) {
        return;
    }

    /* WS2812 byte order is GRB. */
    uint8_t pixels[3] = {
        scale_component(green),
        scale_component(red),
        scale_component(blue)
    };

    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0
    };

    ESP_ERROR_CHECK(
        rmt_transmit(
            led_channel,
            led_encoder,
            pixels,
            sizeof(pixels),
            &tx_cfg
        )
    );

    ESP_ERROR_CHECK(
        rmt_tx_wait_all_done(led_channel, 20)
    );
}

void status_led_off(void)
{
    status_led_set_rgb(0, 0, 0);
}


void status_led_self_test(void)
{
    status_led_set_rgb(255, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(STATUS_LED_SELF_TEST_MS));

    status_led_set_rgb(0, 255, 0);
    vTaskDelay(pdMS_TO_TICKS(STATUS_LED_SELF_TEST_MS));

    status_led_set_rgb(0, 0, 255);
    vTaskDelay(pdMS_TO_TICKS(STATUS_LED_SELF_TEST_MS));

    status_led_off();
}
