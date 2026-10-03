idf_component_register(SRCS "main.c" "as5048b_sanity.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi esp_event esp_adc esp_timer driver esp_driver_i2c nvs_flash 
)
