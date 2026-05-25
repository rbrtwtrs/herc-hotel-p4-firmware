idf_component_register(
    SRCS "main.c" "services.c" "bme68x.c" "camera_video_service.c"
    INCLUDE_DIRS "."
    REQUIRES esp_event esp_netif esp_eth nvs_flash esp_driver_gpio esp_driver_i2c esp_driver_uart app_update esp_https_ota esp_http_server esp_timer esp_video example_video_common espressif__led_strip
    PRIV_REQUIRES espressif__mqtt
)
