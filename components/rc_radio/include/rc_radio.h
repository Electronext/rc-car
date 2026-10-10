#pragma once
#include <stdint.h>
#include "esp_now.h"

extern const uint8_t rc_broadcast_mac[ESP_NOW_ETH_ALEN];
void rc_radio_wifi_init(uint8_t channel);
