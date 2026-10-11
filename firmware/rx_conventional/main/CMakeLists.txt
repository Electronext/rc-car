idf_component_register(SRCS "main.c" INCLUDE_DIRS "." REQUIRES nvs_flash driver esp_driver_i2c esp_timer esp_wifi esp_event esp_netif esp_system rc_radio rc_protocol)
