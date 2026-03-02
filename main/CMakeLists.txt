idf_component_register(
    SRCS "main.c" "services.c"
    INCLUDE_DIRS "."
    REQUIRES esp_event esp_netif esp_eth nvs_flash mqtt driver app_update esp_https_ota
)
