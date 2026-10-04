#pragma once

#include <stdint.h>

void status_led_init(void);
void status_led_set_rgb(uint8_t red, uint8_t green, uint8_t blue);
void status_led_off(void);
void status_led_self_test(void);
