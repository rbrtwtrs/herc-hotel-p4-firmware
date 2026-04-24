#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "esp_sccb_intf.h"
#include "esp_sccb_i2c.h"
#include "esp_cam_sensor_detect.h"

#include "camera_sensor_helper.h"

static const char *TAG = "cam_sensor";
#define CAMERA_SCCB_FREQ_HZ (100 * 1000)

esp_err_t camera_sensor_init(camera_sensor_config_t *sensor_config, camera_sensor_handle_t *out_sensor_handle)
{
    if (!sensor_config || !out_sensor_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_FAIL;
    i2c_master_bus_handle_t i2c_bus_handle = sensor_config->existing_i2c_bus_handle;
    bool owns_i2c_bus = false;

    if (!i2c_bus_handle) {
        i2c_master_bus_config_t i2c_bus_conf = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .sda_io_num = sensor_config->i2c_sda_io_num,
            .scl_io_num = sensor_config->i2c_scl_io_num,
            .i2c_port = sensor_config->i2c_port_num,
            .flags.enable_internal_pullup = true,
        };
        ret = i2c_new_master_bus(&i2c_bus_conf, &i2c_bus_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
            return ret;
        }
        owns_i2c_bus = true;
    }

    esp_cam_sensor_config_t cam_config = {
        .reset_pin = -1,
        .pwdn_pin = -1,
        .xclk_pin = -1,
    };

    esp_cam_sensor_device_t *cam = NULL;
    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start; p < &__esp_cam_sensor_detect_fn_array_end; ++p) {
        ESP_LOGI(TAG, "Probing sensor candidate addr=0x%02x port=%d", p->sccb_addr, p->port);
        sccb_i2c_config_t i2c_config = {
            .scl_speed_hz = CAMERA_SCCB_FREQ_HZ,
            .device_address = p->sccb_addr,
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        };
        ret = sccb_new_i2c_io(i2c_bus_handle, &i2c_config, &cam_config.sccb_handle);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to create SCCB I2C IO for addr 0x%02x: %s", p->sccb_addr, esp_err_to_name(ret));
            continue;
        }

        cam_config.sensor_port = p->port;
        cam = (*(p->detect))(&cam_config);
        if (cam) {
            ESP_LOGI(TAG, "Detected sensor candidate at addr=0x%02x port=%d", p->sccb_addr, p->port);
            if (p->port != sensor_config->port) {
                ESP_LOGE(TAG, "Detected camera sensor with mismatched interface");
                esp_sccb_del_i2c_io(cam_config.sccb_handle);
                if (owns_i2c_bus) {
                    i2c_del_master_bus(i2c_bus_handle);
                }
                return ESP_ERR_INVALID_RESPONSE;
            }
            break;
        }
        ESP_LOGI(TAG, "No sensor detected at addr=0x%02x port=%d", p->sccb_addr, p->port);
        esp_sccb_del_i2c_io(cam_config.sccb_handle);
        cam_config.sccb_handle = NULL;
    }

    if (!cam) {
        ESP_LOGE(TAG, "Failed to detect camera sensor");
        if (owns_i2c_bus) {
            i2c_del_master_bus(i2c_bus_handle);
        }
        return ESP_ERR_NOT_FOUND;
    }

    esp_cam_sensor_format_array_t cam_fmt_array = {0};
    esp_cam_sensor_query_format(cam, &cam_fmt_array);
    const esp_cam_sensor_format_t *parray = cam_fmt_array.format_array;
    esp_cam_sensor_format_t *cam_cur_fmt = NULL;
    for (int i = 0; i < cam_fmt_array.count; i++) {
        ESP_LOGI(TAG, "fmt[%d].name:%s", i, parray[i].name);
        if (!strcmp(parray[i].name, sensor_config->format_name)) {
            cam_cur_fmt = (esp_cam_sensor_format_t *)&parray[i];
        }
    }
    if (!cam_cur_fmt) {
        ESP_LOGE(TAG, "Unsupported camera format: %s", sensor_config->format_name ? sensor_config->format_name : "(null)");
        esp_sccb_del_i2c_io(cam_config.sccb_handle);
        if (owns_i2c_bus) {
            i2c_del_master_bus(i2c_bus_handle);
        }
        return ESP_ERR_INVALID_ARG;
    }

    ret = esp_cam_sensor_set_format(cam, (const esp_cam_sensor_format_t *)cam_cur_fmt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Format set failed: %s", esp_err_to_name(ret));
        esp_sccb_del_i2c_io(cam_config.sccb_handle);
        if (owns_i2c_bus) {
            i2c_del_master_bus(i2c_bus_handle);
        }
        return ret;
    }
    ESP_LOGI(TAG, "Format in use:%s", cam_cur_fmt->name);

    int enable_flag = 1;
    ret = esp_cam_sensor_ioctl(cam, ESP_CAM_SENSOR_IOC_S_STREAM, &enable_flag);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Start stream failed: %s", esp_err_to_name(ret));
        esp_sccb_del_i2c_io(cam_config.sccb_handle);
        if (owns_i2c_bus) {
            i2c_del_master_bus(i2c_bus_handle);
        }
        return ret;
    }

    out_sensor_handle->i2c_bus_handle = i2c_bus_handle;
    out_sensor_handle->sccb_handle = cam_config.sccb_handle;
    out_sensor_handle->owns_i2c_bus = owns_i2c_bus;
    return ESP_OK;
}

void camera_sensor_deinit(camera_sensor_handle_t sensor_handle)
{
    if (sensor_handle.sccb_handle) {
        esp_sccb_del_i2c_io(sensor_handle.sccb_handle);
    }
    if (sensor_handle.owns_i2c_bus && sensor_handle.i2c_bus_handle) {
        i2c_del_master_bus(sensor_handle.i2c_bus_handle);
    }
}
