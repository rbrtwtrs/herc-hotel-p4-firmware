#pragma once

#include <stdbool.h>
#include "mqtt_client.h"

typedef struct {
    esp_mqtt_client_handle_t mqtt;
    bool eth_link_up;
    bool ip_ready;
} app_state_t;

extern app_state_t g_app;
