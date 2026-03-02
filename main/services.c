#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "mqtt_client.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"

#include "app_config.h"
#include "app_state.h"

static const char *TAG = "services";

static void health_task(void *arg) {
    while (1) {
        if (g_app.mqtt && g_app.ip_ready) {
            char payload[96];
            snprintf(payload, sizeof(payload), "{\"uptime_s\":%lu,\"heap\":%lu}",
                     (unsigned long)(esp_log_timestamp() / 1000),
                     (unsigned long)esp_get_free_heap_size());
            esp_mqtt_client_publish(g_app.mqtt, MQTT_HEALTH_TOPIC, payload, 0, 0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

static void ota_task(void *arg) {
    char *url = (char *)arg;
    ESP_LOGI(TAG, "Starting OTA from URL: %s", url);

    esp_http_client_config_t http_cfg = {
        .url = url,
        .timeout_ms = 15000,
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    esp_err_t err = esp_https_ota(&ota_cfg);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OTA succeeded, restarting...");
        free(url);
        esp_restart();
        return;
    }

    ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(err));
    free(url);
    vTaskDelete(NULL);
}

void ota_service_trigger_url(const char *url) {
    if (!url || !url[0]) {
        ESP_LOGW(TAG, "OTA trigger ignored: empty URL");
        return;
    }

    char *url_copy = strdup(url);
    if (!url_copy) {
        ESP_LOGE(TAG, "OTA trigger failed: out of memory");
        return;
    }

    BaseType_t ok = xTaskCreate(ota_task, "ota_task", 10240, url_copy, 5, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "OTA task creation failed");
        free(url_copy);
    }
}

void ota_service_init(void) {
    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG, "OTA ready (running %s / %s)", app->project_name, app->version);
    ESP_LOGI(TAG, "Send OTA URL on topic %s", MQTT_CMD_OTA_TOPIC);

    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "Running partition: %s @ 0x%lx", running->label, (unsigned long)running->address);
    esp_ota_mark_app_valid_cancel_rollback();
}

void i2c_service_init(void) {
    i2c_master_bus_handle_t bus = NULL;

    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&cfg, &bus);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "I2C bus ready on SDA=%d SCL=%d", I2C_SDA_GPIO, I2C_SCL_GPIO);
    } else {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
    }
}

void uart_service_init(void) {
    uart_config_t cfg = {
        .baud_rate = BENDER_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(BENDER_UART_PORT, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BENDER_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(BENDER_UART_PORT, BENDER_UART_TX_GPIO, BENDER_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART ready on TX=%d RX=%d @ %d", BENDER_UART_TX_GPIO, BENDER_UART_RX_GPIO, BENDER_UART_BAUD);
}

void health_service_init(void) {
    xTaskCreate(health_task, "health_task", 4096, NULL, 4, NULL);
}
