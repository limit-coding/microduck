idf_component_register(SRCS "mosaico_power.c" "duck_display.c" "duck_eye_asset.c" "app_main.c" "duck_core.c" "policy.c" "policy_runtime.c" "policy_data.c" "mosaico_imu.c" "mosaico_imu_raw.c" "mosaico_attitude.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_i2c esp_driver_spi esp_lcd esp_lcd_co5300 espressif__bmi270 esp_driver_uart esp_wifi esp_event esp_netif esp_http_server nvs_flash esp_timer espressif__cjson esp_psram heap
    EMBED_TXTFILES "index.html")
target_compile_options(${COMPONENT_LIB} PRIVATE -O3 -ffp-contract=off)
