#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c_master.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "rc_config.h"
#include "as5048b_sanity.h"

#define AS5048B_REG_AGC             0xFA
#define AS5048B_READOUT_BYTES       6

#define AS5048B_DIAG_COMP_HIGH      (1U << 3)
#define AS5048B_DIAG_COMP_LOW       (1U << 2)
#define AS5048B_DIAG_COF            (1U << 1)
#define AS5048B_DIAG_OCF            (1U << 0)

#define AS5048B_COUNTS_PER_REV      16384U
#define AS5048B_READ_TIMEOUT_MS     50
#define AS5048B_TEST_PERIOD_MS      200
#define ADC_SAMPLE_COUNT             16

static const char *TAG = "AS5048B";

typedef struct {
    uint8_t agc;
    uint8_t diagnostics;
    uint16_t magnitude;
    uint16_t angle;
} as5048b_sample_t;

typedef struct {
    bool initialised;
    uint8_t agc_min;
    uint8_t agc_max;
    uint16_t magnitude_min;
    uint16_t magnitude_max;
} as5048b_stats_t;

typedef struct {
    bool initialised;
    int raw_min;
    int raw_max;
    int mv_min;
    int mv_max;
} adc_stats_t;

static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t sensor_1 = NULL;
static i2c_master_dev_handle_t sensor_2 = NULL;

static as5048b_stats_t stats_1 = {0};
static as5048b_stats_t stats_2 = {0};

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;
static adc_channel_t battery_channel;
static adc_channel_t speed_channel;

static adc_stats_t speed_stats = {0};


static esp_err_t add_sensor(uint8_t address,
                            i2c_master_dev_handle_t *handle)
{
    esp_err_t err = i2c_master_probe(
        bus_handle,
        address,
        AS5048B_READ_TIMEOUT_MS
    );

    if (err != ESP_OK) {
        ESP_LOGW(
            TAG,
            "No sensor at 0x%02X: %s",
            address,
            esp_err_to_name(err)
        );

        *handle = NULL;
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = AS5048B_I2C_HZ
    };

    err = i2c_master_bus_add_device(
        bus_handle,
        &dev_cfg,
        handle
    );

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Found sensor at 0x%02X", address);
    } else {
        ESP_LOGE(
            TAG,
            "Could not add sensor 0x%02X: %s",
            address,
            esp_err_to_name(err)
        );

        *handle = NULL;
    }

    return err;
}


static esp_err_t read_sample(i2c_master_dev_handle_t sensor,
                             as5048b_sample_t *sample)
{
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

    /*
     * Consecutive AS5048B readout registers:
     *
     *   0xFA       AGC
     *   0xFB       diagnostic flags
     *   0xFC/0xFD  14-bit magnitude
     *   0xFE/0xFF  14-bit angle
     *
     * The lower register of each 14-bit pair uses bits 5..0.
     */
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


static void update_stats(as5048b_stats_t *stats,
                         const as5048b_sample_t *sample)
{
    if (!stats->initialised) {
        stats->agc_min = sample->agc;
        stats->agc_max = sample->agc;
        stats->magnitude_min = sample->magnitude;
        stats->magnitude_max = sample->magnitude;
        stats->initialised = true;
        return;
    }

    if (sample->agc < stats->agc_min) {
        stats->agc_min = sample->agc;
    }

    if (sample->agc > stats->agc_max) {
        stats->agc_max = sample->agc;
    }

    if (sample->magnitude < stats->magnitude_min) {
        stats->magnitude_min = sample->magnitude;
    }

    if (sample->magnitude > stats->magnitude_max) {
        stats->magnitude_max = sample->magnitude;
    }
}



static void update_adc_stats(adc_stats_t *stats,
                             int raw,
                             int millivolts)
{
    if (!stats->initialised) {
        stats->raw_min = raw;
        stats->raw_max = raw;
        stats->mv_min = millivolts;
        stats->mv_max = millivolts;
        stats->initialised = true;
        return;
    }

    if (raw < stats->raw_min) {
        stats->raw_min = raw;
    }

    if (raw > stats->raw_max) {
        stats->raw_max = raw;
    }

    if (millivolts < stats->mv_min) {
        stats->mv_min = millivolts;
    }

    if (millivolts > stats->mv_max) {
        stats->mv_max = millivolts;
    }
}


static esp_err_t read_adc_average(adc_channel_t channel,
                                  int *raw_average,
                                  int *millivolts)
{
    int64_t total = 0;

    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(
            adc_handle,
            channel,
            &raw
        );

        if (err != ESP_OK) {
            return err;
        }

        total += raw;
    }

    *raw_average = (int)(total / ADC_SAMPLE_COUNT);

    return adc_cali_raw_to_voltage(
        adc_cali_handle,
        *raw_average,
        millivolts
    );
}


static void log_analogue_inputs(void)
{
    int battery_raw = 0;
    int battery_adc_mv = 0;
    int speed_raw = 0;
    int speed_mv = 0;

    esp_err_t battery_err = read_adc_average(
        battery_channel,
        &battery_raw,
        &battery_adc_mv
    );

    esp_err_t speed_err = read_adc_average(
        speed_channel,
        &speed_raw,
        &speed_mv
    );

    if (battery_err == ESP_OK) {
        int64_t battery_mv =
            (int64_t)battery_adc_mv *
            (BATTERY_DIVIDER_TOP_OHMS +
             BATTERY_DIVIDER_BOTTOM_OHMS);

        battery_mv =
            (battery_mv + BATTERY_DIVIDER_BOTTOM_OHMS / 2) /
            BATTERY_DIVIDER_BOTTOM_OHMS;

        ESP_LOGI(
            TAG,
            "BAT  raw=%4d adc=%4d mV -> %lld.%03lld V "
            "(divider %d/%d)",
            battery_raw,
            battery_adc_mv,
            (long long)(battery_mv / 1000),
            (long long)(battery_mv % 1000),
            BATTERY_DIVIDER_TOP_OHMS,
            BATTERY_DIVIDER_BOTTOM_OHMS
        );
    } else {
        ESP_LOGW(
            TAG,
            "Battery ADC read failed: %s",
            esp_err_to_name(battery_err)
        );
    }

    if (speed_err == ESP_OK) {
        update_adc_stats(
            &speed_stats,
            speed_raw,
            speed_mv
        );

        ESP_LOGI(
            TAG,
            "SPEED raw=%4d [%4d..%4d] adc=%4d mV "
            "[%4d..%4d mV]",
            speed_raw,
            speed_stats.raw_min,
            speed_stats.raw_max,
            speed_mv,
            speed_stats.mv_min,
            speed_stats.mv_max
        );
    } else {
        ESP_LOGW(
            TAG,
            "Speed-pot ADC read failed: %s",
            esp_err_to_name(speed_err)
        );
    }
}


static void adc_sanity_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &unit_cfg,
            &adc_handle
        )
    );

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    adc_unit_t unit;

    ESP_ERROR_CHECK(
        adc_oneshot_io_to_channel(
            BATTERY_GPIO,
            &unit,
            &battery_channel
        )
    );

    if (unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "BATTERY_GPIO is not on ADC1");
        abort();
    }

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            battery_channel,
            &chan_cfg
        )
    );

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

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = battery_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    ESP_ERROR_CHECK(
        adc_cali_create_scheme_curve_fitting(
            &cali_cfg,
            &adc_cali_handle
        )
    );

    ESP_LOGI(
        TAG,
        "ADC calibration: GPIO%d battery via %dk/%dk divider; "
        "GPIO%d speed pot via %d ohm + %d ohm pot",
        BATTERY_GPIO,
        BATTERY_DIVIDER_TOP_OHMS / 1000,
        BATTERY_DIVIDER_BOTTOM_OHMS / 1000,
        SPEED_GPIO,
        SPEED_POT_SERIES_OHMS,
        SPEED_POT_OHMS
    );
}


static const char *field_status(uint8_t diagnostics)
{
    if (diagnostics & AS5048B_DIAG_COF) {
        return "INVALID";
    }

    /*
     * The datasheet names these CORDIC comparator flags
     * "Comp High" and "Comp Low". In practice they indicate
     * that the magnetic field is outside the valid range.
     */
    bool comp_high =
        (diagnostics & AS5048B_DIAG_COMP_HIGH) != 0;

    bool comp_low =
        (diagnostics & AS5048B_DIAG_COMP_LOW) != 0;

    if (comp_high && comp_low) {
        return "CHECK";
    }

    if (comp_high) {
        return "WEAK";
    }

    if (comp_low) {
        return "STRONG";
    }

    if (!(diagnostics & AS5048B_DIAG_OCF)) {
        return "STARTUP";
    }

    return "OK";
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

    if (value < -1.0f) {
        value = -1.0f;
    }

    if (value > 1.0f) {
        value = 1.0f;
    }

#if STEERING_INVERT
    value = -value;
#endif

    return value;
}


static void log_sensor(uint8_t address,
                       i2c_master_dev_handle_t sensor,
                       as5048b_stats_t *stats)
{
    if (sensor == NULL) {
        return;
    }

    as5048b_sample_t sample = {0};

    esp_err_t err = read_sample(sensor, &sample);

    if (err != ESP_OK) {
        ESP_LOGW(
            TAG,
            "Read 0x%02X failed: %s",
            address,
            esp_err_to_name(err)
        );
        return;
    }

    update_stats(stats, &sample);

    float degrees =
        (float)sample.angle * 360.0f /
        (float)AS5048B_COUNTS_PER_REV;

    bool ocf =
        (sample.diagnostics & AS5048B_DIAG_OCF) != 0;

    bool cof =
        (sample.diagnostics & AS5048B_DIAG_COF) != 0;

    bool comp_low =
        (sample.diagnostics & AS5048B_DIAG_COMP_LOW) != 0;

    bool comp_high =
        (sample.diagnostics & AS5048B_DIAG_COMP_HIGH) != 0;

    if (address == AS5048B_ADDR_1) {
        float steering = steering_from_raw(sample.angle);

        ESP_LOGI(
            TAG,
            "0x%02X STEER=%+.3f angle=%8.3f deg raw=%5u | "
            "AGC=%3u [%3u..%3u] MAG=%5u [%5u..%5u] | "
            "OCF=%u COF=%u CH=%u CL=%u FIELD=%s",
            address,
            steering,
            degrees,
            sample.angle,
            sample.agc,
            stats->agc_min,
            stats->agc_max,
            sample.magnitude,
            stats->magnitude_min,
            stats->magnitude_max,
            ocf,
            cof,
            comp_high,
            comp_low,
            field_status(sample.diagnostics)
        );
    } else {
        ESP_LOGI(
            TAG,
            "0x%02X angle=%8.3f deg raw=%5u | "
            "AGC=%3u [%3u..%3u] MAG=%5u [%5u..%5u] | "
            "OCF=%u COF=%u CH=%u CL=%u FIELD=%s",
            address,
            degrees,
            sample.angle,
            sample.agc,
            stats->agc_min,
            stats->agc_max,
            sample.magnitude,
            stats->magnitude_min,
            stats->magnitude_max,
            ocf,
            cof,
            comp_high,
            comp_low,
            field_status(sample.diagnostics)
        );
    }
}


static void sanity_task(void *arg)
{
    while (1) {
        log_sensor(
            AS5048B_ADDR_1,
            sensor_1,
            &stats_1
        );

        log_sensor(
            AS5048B_ADDR_2,
            sensor_2,
            &stats_2
        );

        log_analogue_inputs();

        vTaskDelay(pdMS_TO_TICKS(AS5048B_TEST_PERIOD_MS));
    }
}


void as5048b_sanity_init(void)
{
    ESP_LOGI(
        TAG,
        "Sanity test: SDA=GPIO%d SCL=GPIO%d, clock=%u Hz",
        AS5048B_SDA_GPIO,
        AS5048B_SCL_GPIO,
        (unsigned)AS5048B_I2C_HZ
    );

    ESP_LOGI(
        TAG,
        "Expected addresses: 0x%02X (A2=0,A1=0), "
        "0x%02X (A2=1,A1=0)",
        AS5048B_ADDR_1,
        AS5048B_ADDR_2
    );

    adc_sanity_init();

    ESP_LOGI(
        TAG,
        "Steering calibration: raw %d -> deadband %d..%d "
        "(centre %d) -> raw %d",
        STEERING_RAW_MIN,
        STEERING_DEADBAND_LOW,
        STEERING_DEADBAND_HIGH,
        STEERING_CENTER_RAW,
        STEERING_RAW_MAX
    );

    ESP_LOGI(
        TAG,
        "Magnet diagnostic: FIELD should be OK; "
        "AGC 0=stronger field, 255=weaker field. "
        "Rotate each control and watch AGC/MAG min..max span."
    );

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

    esp_err_t err_1 = add_sensor(
        AS5048B_ADDR_1,
        &sensor_1
    );

    esp_err_t err_2 = add_sensor(
        AS5048B_ADDR_2,
        &sensor_2
    );

    if (err_1 != ESP_OK && err_2 != ESP_OK) {
        ESP_LOGE(
            TAG,
            "No AS5048B sensors found. Check power, common ground, "
            "SDA/SCL, 4.7k pull-ups, and A1/A2."
        );
    }

    xTaskCreate(
        sanity_task,
        "as5048b_test",
        3072,
        NULL,
        5,
        NULL
    );
}
