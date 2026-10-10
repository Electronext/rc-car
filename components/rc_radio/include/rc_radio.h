#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_now.h"

extern const uint8_t rc_broadcast_mac[ESP_NOW_ETH_ALEN];
void rc_radio_wifi_init(uint8_t channel);

// Existing ESP-NOW behavior: unencrypted broadcast peer, no protocol changes.
esp_err_t rc_radio_init(void);
esp_err_t rc_radio_add_broadcast_peer(uint8_t channel);
esp_err_t rc_radio_send(const uint8_t *destination, const uint8_t *data, size_t size);
