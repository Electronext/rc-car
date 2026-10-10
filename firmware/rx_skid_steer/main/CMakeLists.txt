# Transitional shared implementation: compile exactly one copy of the
# existing working firmware per target, with role selected by the build.
idf_component_register(
    SRCS "${CMAKE_CURRENT_LIST_DIR}/../../main/main.c"
         "${CMAKE_CURRENT_LIST_DIR}/../../main/as5048b_controls.c"
         "${CMAKE_CURRENT_LIST_DIR}/../../main/status_led.c"
         "${CMAKE_CURRENT_LIST_DIR}/../../main/reset_diag.c"
    INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../main"
    REQUIRES esp_wifi esp_event esp_adc esp_timer driver esp_driver_i2c esp_driver_rmt nvs_flash
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE RC_TRANSMITTER=0)
