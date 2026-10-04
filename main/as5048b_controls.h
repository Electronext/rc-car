#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint8_t agc;
    uint8_t diagnostics;
    uint16_t magnitude;
    uint16_t angle;
} rc_as5048b_sample_t;

void rc_as5048b_init(void);

esp_err_t rc_as5048b_read_steering(rc_as5048b_sample_t *sample);
esp_err_t rc_as5048b_read_throttle(rc_as5048b_sample_t *sample);

bool rc_as5048b_sample_valid(const rc_as5048b_sample_t *sample);
