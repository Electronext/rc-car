#include "rc_radio.h"
#include <string.h>
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

const uint8_t rc_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xff,0xff,0xff,0xff,0xff,0xff};

void rc_radio_wifi_init(uint8_t channel)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE)
    );
}



esp_err_t rc_radio_init(void)
{
    return esp_now_init();
}

esp_err_t rc_radio_add_broadcast_peer(uint8_t channel)
{
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, rc_broadcast_mac, ESP_NOW_ETH_ALEN);
    peer.channel = channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    return esp_now_add_peer(&peer);
}

esp_err_t rc_radio_send(const uint8_t *destination, const uint8_t *data, size_t size)
{
    return esp_now_send(destination, data, size);
}

esp_err_t rc_radio_register_send_cb(esp_now_send_cb_t callback)
{
    return esp_now_register_send_cb(callback);
}

esp_err_t rc_radio_register_recv_cb(esp_now_recv_cb_t callback)
{
    return esp_now_register_recv_cb(callback);
}

bool rc_radio_is_recent(int64_t now_us, int64_t last_seen_us, uint32_t timeout_ms)
{
    return last_seen_us != 0 && now_us >= last_seen_us &&
           (uint64_t)(now_us - last_seen_us) <= (uint64_t)timeout_ms * 1000ULL;
}
