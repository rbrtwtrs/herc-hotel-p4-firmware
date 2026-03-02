#pragma once

#define DEVICE_NAME "herc-hotel-p4"
#define MQTT_URI "mqtt://10.1.70.244:1883"
#define MQTT_USER "herc"
#define MQTT_PASS "nemo"

#define MQTT_STATUS_TOPIC DEVICE_NAME "/status"
#define MQTT_HEALTH_TOPIC DEVICE_NAME "/health"
#define MQTT_CMD_OTA_TOPIC DEVICE_NAME "/cmd/ota"

#define ETH_PHY_ADDR 1
#define ETH_PHY_RESET_GPIO 51
#define ETH_MDC_GPIO 31
#define ETH_MDIO_GPIO 52

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO 7
#define I2C_SCL_GPIO 8

#define BENDER_UART_PORT UART_NUM_1
#define BENDER_UART_TX_GPIO 22
#define BENDER_UART_RX_GPIO 24
#define BENDER_UART_BAUD 115200
