idf_component_register(SRCS "main.c" "as5048b_sanity.c" "status_led.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi esp_event esp_adc esp_timer driver esp_driver_i2c esp_driver_rmt nvs_flash 
)
