#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_now.h"

extern const uint8_t rc_broadcast_mac[ESP_NOW_ETH_ALEN];
void rc_radio_wifi_init(uint8_t channel);

// Existing ESP-NOW behavior: unencrypted broadcast peer, no protocol changes.
esp_err_t rc_radio_init(void);
esp_err_t rc_radio_add_broadcast_peer(uint8_t channel);
esp_err_t rc_radio_send(const uint8_t *destination, const uint8_t *data, size_t size);

// Preserve the application's existing callback ownership and execution context.
esp_err_t rc_radio_register_send_cb(esp_now_send_cb_t callback);
esp_err_t rc_radio_register_recv_cb(esp_now_recv_cb_t callback);

// Pure timeout predicate; callers own timestamps and failsafe transitions.
bool rc_radio_is_recent(int64_t now_us, int64_t last_seen_us, uint32_t timeout_ms);
