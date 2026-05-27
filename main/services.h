#pragma once

void ota_service_init(void);
void ota_service_trigger_url(const char *url);
void i2c_service_init(void);
void uart_service_init(void);
void health_service_init(void);
void status_led_service_init(void);
void neopixel_service_init(void);
void neopixel_handle_cmd(const char *payload);
void neopixel_publish_state(void);
void neopixel_suppress_default_for_snapshot(void);
void mqtt_publish_homeassistant_discovery(void);
void leak_threshold_handle_cmd(const char *payload);
void telemetry_mode_handle_cmd(const char *payload);
void telemetry_mode_publish_state(void);
