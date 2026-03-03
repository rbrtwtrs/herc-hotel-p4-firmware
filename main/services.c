#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "mqtt_client.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_rom_sys.h"

#include "bme68x.h"
#include "app_config.h"
#include "app_state.h"

static const char *TAG = "services";
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static struct bme68x_dev s_bme = {0};
static bool s_bme_ready = false;
static uint8_t s_bme_addr = 0x77;
static float s_leak_wet_threshold_v = LEAK_WET_THRESHOLD_V;

static BME68X_INTF_RET_TYPE bme_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t addr = *(uint8_t *)intf_ptr;
    if (!s_i2c_bus) return -1;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &dev) != ESP_OK) return -1;
    esp_err_t err = i2c_master_transmit_receive(dev, &reg_addr, 1, reg_data, len, 50);
    i2c_master_bus_rm_device(dev);
    return err == ESP_OK ? 0 : -1;
}

static BME68X_INTF_RET_TYPE bme_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t addr = *(uint8_t *)intf_ptr;
    if (!s_i2c_bus) return -1;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &dev) != ESP_OK) return -1;

    uint8_t buf[32] = {0};
    if (len + 1 > sizeof(buf)) {
        i2c_master_bus_rm_device(dev);
        return -1;
    }
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, len);
    esp_err_t err = i2c_master_transmit(dev, buf, len + 1, 50);
    i2c_master_bus_rm_device(dev);
    return err == ESP_OK ? 0 : -1;
}

static void bme_delay_us(uint32_t period, void *intf_ptr) {
    (void)intf_ptr;
    if (period >= 1000) {
        vTaskDelay(pdMS_TO_TICKS((period + 999) / 1000));
    } else {
        esp_rom_delay_us(period);
    }
}

static void bme688_task(void *arg) {
    struct bme68x_conf conf = {0};
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_2X;
    conf.os_pres = BME68X_OS_4X;
    conf.os_temp = BME68X_OS_8X;

    struct bme68x_heatr_conf heatr = {0};
    heatr.enable = BME68X_ENABLE;
    heatr.heatr_temp = 300;
    heatr.heatr_dur = 100;

    bme68x_set_conf(&conf, &s_bme);
    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr, &s_bme);

    while (1) {
        if (g_app.mqtt && g_app.ip_ready && s_bme_ready) {
            bme68x_set_op_mode(BME68X_FORCED_MODE, &s_bme);
            uint32_t dur_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &s_bme);
            vTaskDelay(pdMS_TO_TICKS((dur_us / 1000) + heatr.heatr_dur + 20));

            struct bme68x_data data = {0};
            uint8_t n_fields = 0;
            int8_t rs = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &s_bme);
            if (rs == BME68X_OK && n_fields > 0) {
                float t_c = data.temperature / 100.0f;
                float p_hpa = data.pressure / 100.0f;
                float h_pct = data.humidity / 1000.0f;
                float gas_ohm = (float)data.gas_resistance;

                char payload[192];
                snprintf(payload, sizeof(payload), "{\"temperature_c\":%.2f,\"humidity_pct\":%.2f,\"pressure_hpa\":%.2f,\"gas_ohm\":%.0f}",
                         (double)t_c, (double)h_pct, (double)p_hpa, (double)gas_ohm);
                esp_mqtt_client_publish(g_app.mqtt, MQTT_BME_TOPIC_PREFIX, payload, 0, 0, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(15000));
    }
}

void leak_threshold_handle_cmd(const char *payload) {
    if (!payload || !payload[0]) return;

    char *endptr = NULL;
    float v = strtof(payload, &endptr);
    if (endptr == payload) {
        const char *k = strstr(payload, "volts");
        if (k) {
            while (*k && !((*k >= '0' && *k <= '9') || *k == '-' || *k == '.')) k++;
            if (*k) v = strtof(k, &endptr);
        }
    }

    if (endptr == payload || v < 0.1f || v > 5.0f) {
        ESP_LOGW(TAG, "Leak threshold cmd ignored (invalid): %s", payload);
        return;
    }

    s_leak_wet_threshold_v = v;
    ESP_LOGI(TAG, "Leak WET threshold updated to %.3fV", (double)s_leak_wet_threshold_v);

    if (g_app.mqtt && g_app.ip_ready) {
        char msg[64];
        snprintf(msg, sizeof(msg), "{\"wet_below_v\":%.3f}", (double)s_leak_wet_threshold_v);
        esp_mqtt_client_publish(g_app.mqtt, MQTT_LEAK_THRESHOLD_TOPIC, msg, 0, 1, 1);
    }
}

static bool i2c_probe(uint8_t addr) {
    if (!s_i2c_bus) return false;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &dev) != ESP_OK) return false;

    esp_err_t err = i2c_master_probe(s_i2c_bus, addr, 30);
    i2c_master_bus_rm_device(dev);
    return err == ESP_OK;
}

static esp_err_t ads1115_read_channel(uint8_t addr, uint8_t channel, int16_t *raw_out) {
    if (!s_i2c_bus || !raw_out || channel > 3) return ESP_ERR_INVALID_ARG;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &dev);
    if (err != ESP_OK) return err;

    uint16_t mux_bits = (uint16_t)(0x04 + channel); // AINx vs GND
    uint16_t config = 0;
    config |= (1u << 15);          // OS: start single conversion
    config |= (mux_bits << 12);    // MUX
    config |= (0x01u << 9);        // PGA +-4.096V
    config |= (1u << 8);           // MODE: single-shot
    config |= (0x04u << 5);        // DR: 128 SPS
    config |= 0x03u;               // comparator disable

    uint8_t cfg_wr[3] = {0x01, (uint8_t)(config >> 8), (uint8_t)(config & 0xFF)};
    err = i2c_master_transmit(dev, cfg_wr, sizeof(cfg_wr), 30);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(dev);
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t reg = 0x00;
    uint8_t data[2] = {0};
    err = i2c_master_transmit_receive(dev, &reg, 1, data, 2, 30);
    i2c_master_bus_rm_device(dev);
    if (err != ESP_OK) return err;

    *raw_out = (int16_t)((data[0] << 8) | data[1]);
    return ESP_OK;
}

static void i2c_publish_task(void *arg) {
    while (1) {
        if (g_app.mqtt && g_app.ip_ready && s_i2c_bus) {
            char scan_payload[256] = {0};
            size_t used = 0;
            used += snprintf(scan_payload + used, sizeof(scan_payload) - used, "{\"found\":[");

            bool first = true;
            for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
                if (!i2c_probe(addr)) continue;

                used += snprintf(scan_payload + used, sizeof(scan_payload) - used,
                                 "%s\"0x%02X\"", first ? "" : ",", addr);
                first = false;

                if (addr >= 0x48 && addr <= 0x4B) {
                    for (uint8_t ch = 0; ch < 4; ch++) {
                        int16_t raw = 0;
                        if (ads1115_read_channel(addr, ch, &raw) == ESP_OK) {
                            float volts = ((float)raw) * 4.096f / 32768.0f;
                            char topic[96];
                            char payload[96];
                            snprintf(topic, sizeof(topic), "%s/ads1115/0x%02X/ch%u", MQTT_I2C_TOPIC_PREFIX, addr, ch);
                            snprintf(payload, sizeof(payload), "{\"raw\":%d,\"volts\":%.5f}", raw, (double)volts);
                            esp_mqtt_client_publish(g_app.mqtt, topic, payload, 0, 0, 0);

                            if (addr == 0x49) {
                                const char *state = (volts < s_leak_wet_threshold_v) ? "WET" : "DRY";
                                char leak_topic[96];
                                char leak_payload[96];
                                snprintf(leak_topic, sizeof(leak_topic), "%s/zone%u", MQTT_LEAK_TOPIC_PREFIX, (unsigned)(ch + 1));
                                snprintf(leak_payload, sizeof(leak_payload), "{\"state\":\"%s\",\"volts\":%.5f,\"raw\":%d}", state, (double)volts, raw);
                                esp_mqtt_client_publish(g_app.mqtt, leak_topic, leak_payload, 0, 0, 0);
                            }
                        }
                    }
                }
            }

            snprintf(scan_payload + used, sizeof(scan_payload) - used, "]}");
            esp_mqtt_client_publish(g_app.mqtt, MQTT_I2C_SCAN_TOPIC, scan_payload, 0, 0, 0);
            ESP_LOGI(TAG, "I2C scan published: %s", scan_payload);
        }

        vTaskDelay(pdMS_TO_TICKS(15000));
    }
}

static void mqtt_publish_ha_sensor(const char *component, const char *obj_id, const char *name,
                                   const char *state_topic, const char *unit,
                                   const char *device_class, const char *value_template) {
    if (!g_app.mqtt) return;

    char topic[192];
    char payload[768];
    snprintf(topic, sizeof(topic), "homeassistant/%s/%s/config", component, obj_id);
    snprintf(payload, sizeof(payload),
             "{\"name\":\"%s\",\"state_topic\":\"%s\",\"availability_topic\":\"%s\","
             "\"payload_available\":\"online\",\"payload_not_available\":\"offline\","
             "\"unique_id\":\"%s\",\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"Waveshare\",\"model\":\"ESP32-P4-Nano\"}%s%s%s}",
             name, state_topic, MQTT_STATUS_TOPIC,
             obj_id, DEVICE_NAME, DEVICE_NAME,
             unit ? ",\"unit_of_measurement\":\"" : "",
             unit ? unit : "",
             unit ? "\"" : "");

    // Append optional fields safely
    size_t len = strlen(payload);
    if (len > 0 && payload[len - 1] == '}') payload[len - 1] = '\0';
    if (device_class) {
        strncat(payload, ",\"device_class\":\"", sizeof(payload) - strlen(payload) - 1);
        strncat(payload, device_class, sizeof(payload) - strlen(payload) - 1);
        strncat(payload, "\"", sizeof(payload) - strlen(payload) - 1);
    }
    if (value_template) {
        strncat(payload, ",\"value_template\":\"", sizeof(payload) - strlen(payload) - 1);
        strncat(payload, value_template, sizeof(payload) - strlen(payload) - 1);
        strncat(payload, "\"", sizeof(payload) - strlen(payload) - 1);
    }
    strncat(payload, "}", sizeof(payload) - strlen(payload) - 1);

    esp_mqtt_client_publish(g_app.mqtt, topic, payload, 0, 1, 1);
}

static void mqtt_publish_ha_binary(const char *obj_id, const char *name, const char *state_topic,
                                  const char *device_class, const char *payload_on, const char *payload_off) {
    if (!g_app.mqtt) return;

    char topic[192];
    char payload[768];
    snprintf(topic, sizeof(topic), "homeassistant/binary_sensor/%s/config", obj_id);
    snprintf(payload, sizeof(payload),
             "{\"name\":\"%s\",\"state_topic\":\"%s\",\"availability_topic\":\"%s\","
             "\"payload_available\":\"online\",\"payload_not_available\":\"offline\","
             "\"payload_on\":\"%s\",\"payload_off\":\"%s\","
             "\"unique_id\":\"%s\",\"device_class\":\"%s\","
             "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"Waveshare\",\"model\":\"ESP32-P4-Nano\"}}",
             name, state_topic, MQTT_STATUS_TOPIC, payload_on, payload_off,
             obj_id, device_class, DEVICE_NAME, DEVICE_NAME);
    esp_mqtt_client_publish(g_app.mqtt, topic, payload, 0, 1, 1);
}

void mqtt_publish_homeassistant_discovery(void) {
    if (!g_app.mqtt) return;

    // BME688
    mqtt_publish_ha_sensor("sensor", "herc_hotel_p4_bme_temp", "Herc Hotel Temperature", MQTT_BME_TOPIC_PREFIX,
                           "°C", "temperature", "{{ value_json.temperature_c }}");
    mqtt_publish_ha_sensor("sensor", "herc_hotel_p4_bme_humidity", "Herc Hotel Humidity", MQTT_BME_TOPIC_PREFIX,
                           "%", "humidity", "{{ value_json.humidity_pct }}");
    mqtt_publish_ha_sensor("sensor", "herc_hotel_p4_bme_pressure", "Herc Hotel Pressure", MQTT_BME_TOPIC_PREFIX,
                           "hPa", "pressure", "{{ value_json.pressure_hpa }}");
    mqtt_publish_ha_sensor("sensor", "herc_hotel_p4_bme_gas", "Herc Hotel Gas", MQTT_BME_TOPIC_PREFIX,
                           "Ω", NULL, "{{ value_json.gas_ohm }}");

    // ADS1115 channels
    for (int addr = 0x48; addr <= 0x4A; addr++) {
        for (int ch = 0; ch < 4; ch++) {
            char topic[96], obj_id[64], name[96], vt[64];
            snprintf(topic, sizeof(topic), "%s/ads1115/0x%02X/ch%d", MQTT_I2C_TOPIC_PREFIX, addr, ch);
            snprintf(obj_id, sizeof(obj_id), "herc_hotel_p4_ads_0x%02X_ch%d", addr, ch);
            if (addr == 0x49) {
                snprintf(name, sizeof(name), "Leak Zone %d Voltage", ch + 1);
            } else {
                snprintf(name, sizeof(name), "ADS1115 0x%02X CH%d", addr, ch);
            }
            snprintf(vt, sizeof(vt), "{{ value_json.volts }}");
            mqtt_publish_ha_sensor("sensor", obj_id, name, topic, "V", "voltage", vt);
        }
    }

    // Leak zones (0x49 + pull-up bank)
    for (int zone = 1; zone <= 4; zone++) {
        char topic[96], obj_id[64], name[96], vt[64];
        snprintf(topic, sizeof(topic), "%s/zone%d", MQTT_LEAK_TOPIC_PREFIX, zone);
        snprintf(obj_id, sizeof(obj_id), "herc_hotel_p4_leak_zone_%d", zone);
        snprintf(name, sizeof(name), "Leak Zone %d", zone);
        mqtt_publish_ha_binary(obj_id, name, topic, "moisture", "WET", "DRY");

        snprintf(obj_id, sizeof(obj_id), "herc_hotel_p4_leak_zone_%d_state", zone);
        snprintf(name, sizeof(name), "Leak Zone %d State", zone);
        snprintf(vt, sizeof(vt), "{{ value_json.state }}");
        mqtt_publish_ha_sensor("sensor", obj_id, name, topic, NULL, NULL, vt);
    }

    // Bender channels
    for (int id = 0; id < 3; id++) {
        char topic[96], obj_id[64], name[96], vt[96];
        snprintf(topic, sizeof(topic), "%s/%d", MQTT_BENDER_TOPIC_PREFIX, id);
        snprintf(obj_id, sizeof(obj_id), "herc_hotel_p4_bender_%d", id);
        snprintf(name, sizeof(name), "Bender %d Resistance", id);
        snprintf(vt, sizeof(vt), "{{ value_json.resistance_kohm }}");
        mqtt_publish_ha_sensor("sensor", obj_id, name, topic, "kΩ", NULL, vt);
    }

    // Leak threshold status
    {
        char payload[64];
        snprintf(payload, sizeof(payload), "{\"wet_below_v\":%.3f}", (double)s_leak_wet_threshold_v);
        esp_mqtt_client_publish(g_app.mqtt, MQTT_LEAK_THRESHOLD_TOPIC, payload, 0, 1, 1);
    }

    ESP_LOGI(TAG, "Home Assistant MQTT discovery published");
}

static void health_task(void *arg) {
    while (1) {
        if (g_app.mqtt && g_app.ip_ready) {
            char payload[96];
            snprintf(payload, sizeof(payload), "{\"uptime_s\":%lu,\"heap\":%lu}",
                     (unsigned long)(esp_log_timestamp() / 1000),
                     (unsigned long)esp_get_free_heap_size());
            int msg_id = esp_mqtt_client_publish(g_app.mqtt, MQTT_HEALTH_TOPIC, payload, 0, 0, 0);
            ESP_LOGI(TAG, "MQTT publish health msg_id=%d topic=%s payload=%s", msg_id, MQTT_HEALTH_TOPIC, payload);
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
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&cfg, &s_i2c_bus);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "I2C bus ready on SDA=%d SCL=%d", I2C_SDA_GPIO, I2C_SCL_GPIO);
        ESP_LOGI(TAG, "I2C MQTT scan topic: %s", MQTT_I2C_SCAN_TOPIC);
        xTaskCreate(i2c_publish_task, "i2c_pub_task", 6144, NULL, 4, NULL);

        s_bme.intf = BME68X_I2C_INTF;
        s_bme.read = bme_i2c_read;
        s_bme.write = bme_i2c_write;
        s_bme.delay_us = bme_delay_us;
        s_bme.intf_ptr = &s_bme_addr;
        s_bme.amb_temp = 25;
        if (bme68x_init(&s_bme) == BME68X_OK) {
            s_bme_ready = true;
            ESP_LOGI(TAG, "BME688 initialized at 0x%02X", s_bme_addr);
            xTaskCreate(bme688_task, "bme688_task", 6144, NULL, 4, NULL);
        } else {
            ESP_LOGW(TAG, "BME688 init failed at 0x%02X", s_bme_addr);
        }
    } else {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
    }
}

static bool bender_validate_checksum(const char *line) {
    const char *ast = strchr(line, '*');
    if (!ast || ast == line || strlen(ast + 1) < 2) return false;

    unsigned int sum = 0;
    for (const char *p = line; p < ast; p++) sum = (sum + (uint8_t)*p) & 0xFF;

    char hex[3] = {0};
    hex[0] = (char)toupper((unsigned char)ast[1]);
    hex[1] = (char)toupper((unsigned char)ast[2]);

    char expect[3];
    snprintf(expect, sizeof(expect), "%02X", sum);
    return (hex[0] == expect[0] && hex[1] == expect[1]);
}

static void bender_process_line(const char *line) {
    if (!bender_validate_checksum(line)) return;

    char buf[160];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char *ast = strchr(buf, '*');
    if (ast) *ast = '\0';

    char *ctx = NULL;
    char *parts[12] = {0};
    int n = 0;
    for (char *tok = strtok_r(buf, " ", &ctx); tok && n < 12; tok = strtok_r(NULL, " ", &ctx)) {
        parts[n++] = tok;
    }

    if (n >= 7 && strcmp(parts[0], "BGF") == 0) {
        int bender_id = atoi(parts[1]);
        float status = (float)atof(parts[2]);
        float resistance = (float)atof(parts[6]);

        if (g_app.mqtt && g_app.ip_ready) {
            char topic[96];
            char payload[128];
            snprintf(topic, sizeof(topic), "%s/%d", MQTT_BENDER_TOPIC_PREFIX, bender_id);
            snprintf(payload, sizeof(payload), "{\"status\":%.0f,\"resistance_kohm\":%.2f,\"raw\":\"%s\"}",
                     (double)status, (double)resistance, line);
            esp_mqtt_client_publish(g_app.mqtt, topic, payload, 0, 0, 0);
        }
    }
}

static void bender_uart_task(void *arg) {
    uint8_t ch;
    char line[192] = {0};
    int idx = 0;

    while (1) {
        int n = uart_read_bytes(BENDER_UART_PORT, &ch, 1, pdMS_TO_TICKS(100));
        if (n <= 0) continue;

        if (ch == '\n' || ch == '\r') {
            if (idx > 0) {
                line[idx] = '\0';
                bender_process_line(line);
                idx = 0;
            }
        } else if (idx < (int)sizeof(line) - 1) {
            line[idx++] = (char)ch;
        } else {
            idx = 0;
        }
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
    xTaskCreate(bender_uart_task, "bender_uart_task", 4096, NULL, 4, NULL);
}

void health_service_init(void) {
    xTaskCreate(health_task, "health_task", 4096, NULL, 4, NULL);
}
