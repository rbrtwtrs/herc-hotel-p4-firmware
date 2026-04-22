#pragma once

void ota_service_init(void);
void ota_service_trigger_url(const char *url);
void i2c_service_init(void);
void uart_service_init(void);
void health_service_init(void);
void status_led_service_init(void);
void mqtt_publish_homeassistant_discovery(void);
void leak_threshold_handle_cmd(const char *payload);
