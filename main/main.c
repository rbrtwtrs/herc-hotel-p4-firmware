#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_eth_phy_ip101.h"

#include "app_config.h"
#include "app_state.h"
#include "services.h"

static const char *TAG = "herc_hotel_p4";
app_state_t g_app = {0};

static void log_memory_status(const char *stage) {
    size_t free_heap = esp_get_free_heap_size();
    size_t min_free_heap = esp_get_minimum_free_heap_size();
    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    ESP_LOGI(TAG,
             "Memory @ %s: free_heap=%u min_free_heap=%u free_internal=%u free_spiram=%u",
             stage,
             (unsigned)free_heap,
             (unsigned)min_free_heap,
             (unsigned)free_internal,
             (unsigned)free_spiram);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED: {
            ESP_LOGI(TAG, "MQTT connected");
            int status_id = esp_mqtt_client_publish(g_app.mqtt, MQTT_STATUS_TOPIC, "online", 0, 1, 1);
            int debug_id = esp_mqtt_client_publish(g_app.mqtt, MQTT_DEBUG_TOPIC, "boot_connected", 0, 0, 0);
            int sub_id = esp_mqtt_client_subscribe(g_app.mqtt, MQTT_CMD_OTA_TOPIC, 1);
            int sub_leak_id = esp_mqtt_client_subscribe(g_app.mqtt, MQTT_CMD_LEAK_THRESHOLD_TOPIC, 1);
            int sub_camera_id = esp_mqtt_client_subscribe(g_app.mqtt, MQTT_CMD_CAMERA_SNAP_TOPIC, 1);
            mqtt_publish_homeassistant_discovery();
            ESP_LOGI(TAG, "MQTT publish status msg_id=%d topic=%s", status_id, MQTT_STATUS_TOPIC);
            ESP_LOGI(TAG, "MQTT publish debug msg_id=%d topic=%s", debug_id, MQTT_DEBUG_TOPIC);
            ESP_LOGI(TAG, "MQTT subscribe msg_id=%d topic=%s", sub_id, MQTT_CMD_OTA_TOPIC);
            ESP_LOGI(TAG, "MQTT subscribe msg_id=%d topic=%s", sub_leak_id, MQTT_CMD_LEAK_THRESHOLD_TOPIC);
            ESP_LOGI(TAG, "MQTT subscribe msg_id=%d topic=%s", sub_camera_id, MQTT_CMD_CAMERA_SNAP_TOPIC);
            break;
        }
        case MQTT_EVENT_DATA: {
            char topic_buf[128] = {0};
            char data_buf[256] = {0};
            int tlen = event->topic_len < (int)sizeof(topic_buf) - 1 ? event->topic_len : (int)sizeof(topic_buf) - 1;
            int dlen = event->data_len < (int)sizeof(data_buf) - 1 ? event->data_len : (int)sizeof(data_buf) - 1;
            memcpy(topic_buf, event->topic, tlen);
            memcpy(data_buf, event->data, dlen);
            topic_buf[tlen] = '\0';
            data_buf[dlen] = '\0';
            ESP_LOGI(TAG, "MQTT data topic=%s payload=%s", topic_buf, data_buf);

            if (event->topic_len == (int)strlen(MQTT_CMD_OTA_TOPIC) &&
                strncmp(event->topic, MQTT_CMD_OTA_TOPIC, event->topic_len) == 0) {
                ESP_LOGI(TAG, "OTA command received: %s", data_buf);
                ota_service_trigger_url(data_buf);
            } else if (event->topic_len == (int)strlen(MQTT_CMD_LEAK_THRESHOLD_TOPIC) &&
                       strncmp(event->topic, MQTT_CMD_LEAK_THRESHOLD_TOPIC, event->topic_len) == 0) {
                ESP_LOGI(TAG, "Leak threshold command received: %s", data_buf);
                leak_threshold_handle_cmd(data_buf);
            } else if (event->topic_len == (int)strlen(MQTT_CMD_CAMERA_SNAP_TOPIC) &&
                       strncmp(event->topic, MQTT_CMD_CAMERA_SNAP_TOPIC, event->topic_len) == 0) {
                ESP_LOGI(TAG, "Camera snapshot command received: %s", data_buf);
                camera_service_trigger_snapshot(data_buf);
            }
            break;
        }
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT event error type=%d esp_tls_last_esp_err=0x%x tls_stack_err=0x%x tls_cert_flags=0x%x", 
                     event->error_handle ? event->error_handle->error_type : -1,
                     event->error_handle ? event->error_handle->esp_tls_last_esp_err : 0,
                     event->error_handle ? event->error_handle->esp_tls_stack_err : 0,
                     event->error_handle ? event->error_handle->esp_tls_cert_verify_flags : 0);
            break;
        default:
            break;
    }
}

static void start_mqtt(void) {
    if (g_app.mqtt) return;

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URI,
        .credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASS,
    };

    g_app.mqtt = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(g_app.mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(g_app.mqtt);
}

static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
        case ETHERNET_EVENT_CONNECTED:
            g_app.eth_link_up = true;
            esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
            ESP_LOGI(TAG, "Ethernet Link Up");
            ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                     mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            g_app.eth_link_up = false;
            g_app.ip_ready = false;
            ESP_LOGW(TAG, "Ethernet Link Down");
            break;
        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "Ethernet Started");
            break;
        case ETHERNET_EVENT_STOP:
            ESP_LOGI(TAG, "Ethernet Stopped");
            break;
        default:
            break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    g_app.ip_ready = true;
    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));

    http_snapshot_service_start();
    start_mqtt();
}

static void init_ethernet(void) {
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ETH_PHY_RESET_GPIO;

    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_gpio.mdc_num = ETH_MDC_GPIO;
    esp32_emac_config.smi_gpio.mdio_num = ETH_MDIO_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);

    esp_eth_handle_t eth_handle = NULL;
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    log_memory_status("boot");

    init_ethernet();
    ota_service_init();
    i2c_service_init();
    uart_service_init();
    health_service_init();
    status_led_service_init();
    camera_service_init();
    http_snapshot_service_init();
    log_memory_status("services_started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
