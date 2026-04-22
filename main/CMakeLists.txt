idf_component_register(
    SRCS "main.c" "services.c" "bme68x.c"
    INCLUDE_DIRS "."
    REQUIRES esp_event esp_netif esp_eth nvs_flash esp_driver_gpio esp_driver_i2c esp_driver_uart app_update esp_https_ota
    PRIV_REQUIRES espressif__mqtt espressif__ip101
)
