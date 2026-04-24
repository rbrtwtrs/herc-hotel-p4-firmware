#pragma once

#include "driver/i2c_master.h"
#include "esp_cam_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    esp_sccb_io_handle_t sccb_handle;
    i2c_master_bus_handle_t i2c_bus_handle;
    bool owns_i2c_bus;
} camera_sensor_handle_t;

typedef struct {
    int i2c_port_num;
    int i2c_sda_io_num;
    int i2c_scl_io_num;
    i2c_master_bus_handle_t existing_i2c_bus_handle;
    esp_cam_sensor_port_t port;
    const char *format_name;
} camera_sensor_config_t;

esp_err_t camera_sensor_init(camera_sensor_config_t *sensor_config, camera_sensor_handle_t *out_sensor_handle);
void camera_sensor_deinit(camera_sensor_handle_t sensor_handle);

#ifdef __cplusplus
}
#endif
