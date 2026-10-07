idf_component_register(SRCS "main.c" "as5048b_controls.c" "status_led.c" "reset_diag.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi esp_event esp_adc esp_timer driver esp_driver_i2c esp_driver_rmt nvs_flash 
)
