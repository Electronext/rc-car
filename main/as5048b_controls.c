#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

#include "rc_config.h"
#include "as5048b_controls.h"

#define AS5048B_REG_AGC             0xFA
#define AS5048B_READOUT_BYTES       6
#define AS5048B_READ_TIMEOUT_MS     50

#define AS5048B_DIAG_COMP_HIGH      (1U << 3)
#define AS5048B_DIAG_COMP_LOW       (1U << 2)
#define AS5048B_DIAG_COF            (1U << 1)
#define AS5048B_DIAG_OCF            (1U << 0)

static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t steering_sensor = NULL;
static i2c_master_dev_handle_t throttle_sensor = NULL;

static void add_sensor(uint8_t address,
                       i2c_master_dev_handle_t *handle)
{
    ESP_ERROR_CHECK(
        i2c_master_probe(
            bus_handle,
            address,
            AS5048B_READ_TIMEOUT_MS
        )
    );

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = AS5048B_I2C_HZ
    };

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(
            bus_handle,
            &dev_cfg,
            handle
        )
    );
}

void rc_as5048b_init(void)
{
    if (bus_handle != NULL) {
        return;
    }

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = AS5048B_SDA_GPIO,
        .scl_io_num = AS5048B_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false
    };

    ESP_ERROR_CHECK(
        i2c_new_master_bus(
            &bus_cfg,
            &bus_handle
        )
    );

    add_sensor(
        AS5048B_ADDR_1,
        &steering_sensor
    );

    add_sensor(
        AS5048B_ADDR_2,
        &throttle_sensor
    );
}

static esp_err_t read_sensor(i2c_master_dev_handle_t sensor,
                             rc_as5048b_sample_t *sample)
{
    if (sensor == NULL || sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t register_address = AS5048B_REG_AGC;
    uint8_t data[AS5048B_READOUT_BYTES];

    esp_err_t err = i2c_master_transmit_receive(
        sensor,
        &register_address,
        sizeof(register_address),
        data,
        sizeof(data),
        AS5048B_READ_TIMEOUT_MS
    );

    if (err != ESP_OK) {
        return err;
    }

    sample->agc = data[0];
    sample->diagnostics = data[1];

    sample->magnitude =
        ((uint16_t)data[2] << 6) |
        ((uint16_t)data[3] & 0x3FU);

    sample->angle =
        ((uint16_t)data[4] << 6) |
        ((uint16_t)data[5] & 0x3FU);

    return ESP_OK;
}

esp_err_t rc_as5048b_read_steering(rc_as5048b_sample_t *sample)
{
    return read_sensor(steering_sensor, sample);
}

esp_err_t rc_as5048b_read_throttle(rc_as5048b_sample_t *sample)
{
    return read_sensor(throttle_sensor, sample);
}

bool rc_as5048b_sample_valid(const rc_as5048b_sample_t *sample)
{
    if (sample == NULL) {
        return false;
    }

    if ((sample->diagnostics & AS5048B_DIAG_OCF) == 0) {
        return false;
    }

    if (sample->diagnostics & AS5048B_DIAG_COF) {
        return false;
    }

    if (sample->diagnostics &
        (AS5048B_DIAG_COMP_HIGH | AS5048B_DIAG_COMP_LOW)) {
        return false;
    }

    return true;
}
