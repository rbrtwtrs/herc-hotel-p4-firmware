#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "esp_ldo_regulator.h"
#include "mqtt_client.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "driver/jpeg_encode.h"
#include "driver/uart.h"
#include "esp_rom_sys.h"
#include "esp_http_server.h"
#include "esp_private/esp_cache_private.h"

#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "camera_sensor_helper.h"

#include "bme68x.h"
#include "app_config.h"
#include "app_state.h"

static const char *TAG = "services";
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static struct bme68x_dev s_bme = {0};
static bool s_bme_ready = false;
static uint8_t s_bme_addr = 0x77;
static float s_leak_wet_threshold_v = LEAK_WET_THRESHOLD_V;
static SemaphoreHandle_t s_camera_mutex = NULL;
static bool s_camera_initialized = false;
static httpd_handle_t s_http_server = NULL;
static esp_err_t s_camera_last_error = ESP_OK;
static char s_camera_last_stage[64] = "idle";
static size_t s_camera_last_http_received_size = 0;
static uint32_t s_camera_last_http_frame_delta = 0;

#define CAMERA_HTTP_JPEG_QUALITY 80
#define CAMERA_HTTP_RGB565_BYTES_PER_PIXEL 2u

#if CAMERA_ENABLED
static esp_cam_ctlr_handle_t s_cam_handle = NULL;
static isp_proc_handle_t s_isp_proc = NULL;
static camera_sensor_handle_t s_sensor_handle = {0};
static void *s_camera_frame_buffer = NULL;
static size_t s_camera_frame_buffer_size = 0;
static size_t s_camera_frame_buffer_alignment = 0;
static esp_cam_ctlr_trans_t s_camera_trans = {0};
static SemaphoreHandle_t s_camera_frame_ready = NULL;
static volatile size_t s_camera_last_received_size = 0;
static volatile uint32_t s_camera_finished_count = 0;
#endif

#define ADS1115_FULL_SCALE_V 4.096f
#define ADS1115_RAW_SCALE 32768.0f
#define MOTOR_TEMP_SENSE_RESISTOR_OHMS 2700.0f
#define PRESSURE_SENSOR_FULL_SCALE_V 5.0f
#define PRESSURE_SENSOR_FULL_SCALE_PSI 6000.0f
#define RES_LEVEL_FULL_SCALE_V 5.0f

typedef struct {
    uint8_t channel;
    const char *key;
    const char *name;
} ads_named_channel_t;

static void publish_camera_status(const char *state, const char *detail) {
    if (!g_app.mqtt || !g_app.ip_ready) return;

    char payload[192];
    snprintf(payload, sizeof(payload), "{\"state\":\"%s\",\"detail\":\"%s\",\"free_internal\":%u,\"free_spiram\":%u,\"camera_enabled\":%s}",
             state ? state : "unknown",
             detail ? detail : "",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             CAMERA_ENABLED ? "true" : "false");
    esp_mqtt_client_publish(g_app.mqtt, MQTT_DEBUG_TOPIC, payload, 0, 0, 0);
}

#if CAMERA_ENABLED
static bool IRAM_ATTR camera_on_get_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
    (void)handle;
    esp_cam_ctlr_trans_t *next_trans = (esp_cam_ctlr_trans_t *)user_data;
    if (!trans || !next_trans) {
        return false;
    }
    trans->buffer = next_trans->buffer;
    trans->buflen = next_trans->buflen;
    return false;
}

static void camera_set_last_error(const char *stage, esp_err_t err) {
    s_camera_last_error = err;
    if (stage) {
        snprintf(s_camera_last_stage, sizeof(s_camera_last_stage), "%s", stage);
    }
}

static bool IRAM_ATTR camera_on_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
    (void)handle;
    (void)user_data;
    BaseType_t high_task_woken = pdFALSE;
    s_camera_last_received_size = trans ? trans->received_size : 0;
    s_camera_finished_count++;
    if (s_camera_frame_ready) {
        xSemaphoreGiveFromISR(s_camera_frame_ready, &high_task_woken);
    }
    return high_task_woken == pdTRUE;
}

static esp_err_t camera_try_init(void) {
    if (s_camera_initialized) {
        return ESP_OK;
    }

    esp_err_t ret;
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = CAMERA_MIPI_LDO_CHANNEL,
        .voltage_mv = CAMERA_MIPI_LDO_VOLTAGE_MV,
    };

    ret = esp_ldo_acquire_channel(&ldo_cfg, &ldo_mipi_phy);
    if (ret != ESP_OK) {
        camera_set_last_error("ldo_acquire", ret);
        ESP_LOGE(TAG, "Camera LDO acquire failed: %s", esp_err_to_name(ret));
        return ret;
    }

    camera_sensor_config_t sensor_cfg = {
        .i2c_port_num = I2C_NUM_0,
        .i2c_sda_io_num = I2C_SDA_GPIO,
        .i2c_scl_io_num = I2C_SCL_GPIO,
        .existing_i2c_bus_handle = s_i2c_bus,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .format_name = CAMERA_SENSOR_FORMAT,
    };
    ret = camera_sensor_init(&sensor_cfg, &s_sensor_handle);
    if (ret != ESP_OK) {
        camera_set_last_error("sensor_init", ret);
        ESP_LOGE(TAG, "Camera sensor init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = CAMERA_MIPI_CSI_HRES,
        .v_res = CAMERA_MIPI_CSI_VRES,
        .lane_bit_rate_mbps = CAMERA_MIPI_CSI_LANE_BITRATE_MBPS,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RAW8,
        .data_lane_num = 2,
        .queue_items = 1,
        .byte_swap_en = false,
    };
    ret = esp_cam_new_csi_ctlr(&csi_config, &s_cam_handle);
    if (ret != ESP_OK) {
        camera_set_last_error("new_csi", ret);
        ESP_LOGE(TAG, "Camera CSI init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = CAMERA_MIPI_CSI_HRES,
        .v_res = CAMERA_MIPI_CSI_VRES,
        .bayer_order = COLOR_RAW_ELEMENT_ORDER_GBRG,
    };
    ret = esp_isp_new_processor(&isp_config, &s_isp_proc);
    if (ret != ESP_OK) {
        camera_set_last_error("isp_new", ret);
        ESP_LOGE(TAG, "Camera ISP init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_isp_demosaic_config_t demosaic_config = {
        .grad_ratio = {
            .integer = 2,
            .decimal = 5,
        },
    };
    ret = esp_isp_demosaic_configure(s_isp_proc, &demosaic_config);
    if (ret != ESP_OK) {
        camera_set_last_error("demosaic_config", ret);
        ESP_LOGE(TAG, "Camera ISP demosaic config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_isp_demosaic_enable(s_isp_proc);
    if (ret != ESP_OK) {
        camera_set_last_error("demosaic_enable", ret);
        ESP_LOGE(TAG, "Camera ISP demosaic enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_isp_enable(s_isp_proc);
    if (ret != ESP_OK) {
        camera_set_last_error("isp_enable", ret);
        ESP_LOGE(TAG, "Camera ISP enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_camera_frame_buffer_size = CAMERA_MIPI_CSI_HRES * CAMERA_MIPI_CSI_VRES * CAMERA_HTTP_RGB565_BYTES_PER_PIXEL;
    ret = esp_cache_get_alignment(0, &s_camera_frame_buffer_alignment);
    if (ret != ESP_OK) {
        camera_set_last_error("cache_alignment", ret);
        ESP_LOGE(TAG, "Camera cache alignment query failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_camera_frame_buffer = heap_caps_aligned_calloc(s_camera_frame_buffer_alignment,
                                                     1,
                                                     s_camera_frame_buffer_size,
                                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_camera_frame_buffer) {
        camera_set_last_error("alloc_fb", ESP_ERR_NO_MEM);
        ESP_LOGE(TAG, "Camera frame buffer allocation failed size=%u", (unsigned)s_camera_frame_buffer_size);
        return ESP_ERR_NO_MEM;
    }

    s_camera_trans.buffer = s_camera_frame_buffer;
    s_camera_trans.buflen = s_camera_frame_buffer_size;

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = camera_on_get_new_trans,
        .on_trans_finished = camera_on_trans_finished,
    };
    ret = esp_cam_ctlr_register_event_callbacks(s_cam_handle, &cbs, &s_camera_trans);
    if (ret != ESP_OK) {
        camera_set_last_error("register_cbs", ret);
        ESP_LOGE(TAG, "Camera callback registration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_cam_ctlr_enable(s_cam_handle);
    if (ret != ESP_OK) {
        camera_set_last_error("ctlr_enable", ret);
        ESP_LOGE(TAG, "Camera controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_cam_ctlr_start(s_cam_handle);
    if (ret != ESP_OK) {
        camera_set_last_error("ctlr_start", ret);
        ESP_LOGE(TAG, "Camera controller start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_cam_ctlr_receive(s_cam_handle, &s_camera_trans, pdMS_TO_TICKS(1500));
    if (ret != ESP_OK) {
        camera_set_last_error("initial_receive", ret);
        ESP_LOGE(TAG, "Camera initial receive queue failed: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    s_camera_initialized = true;
    camera_set_last_error("streaming", ESP_OK);
    ESP_LOGI(TAG, "Camera initialized, frame_buffer=%p size=%u", s_camera_frame_buffer, (unsigned)s_camera_frame_buffer_size);
    return ESP_OK;
}

static esp_err_t camera_capture_locked(size_t *received_size_out, uint32_t *frame_count_delta_out) {
    if (!received_size_out || !frame_count_delta_out) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = camera_try_init();
    if (ret != ESP_OK) {
        return ret;
    }

    uint32_t start_count = s_camera_finished_count;
    while (s_camera_frame_ready && xSemaphoreTake(s_camera_frame_ready, 0) == pdTRUE) {
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    if (!s_camera_frame_ready || xSemaphoreTake(s_camera_frame_ready, pdMS_TO_TICKS(1500)) != pdTRUE) {
        *frame_count_delta_out = s_camera_finished_count - start_count;
        *received_size_out = 0;
        camera_set_last_error("wait_frame", ESP_ERR_TIMEOUT);
        return ESP_ERR_TIMEOUT;
    }

    *received_size_out = s_camera_last_received_size;
    *frame_count_delta_out = s_camera_finished_count - start_count;
    camera_set_last_error("frame_ok", ESP_OK);
    return ESP_OK;
}
#endif

void camera_service_init(void) {
    if (!s_camera_mutex) {
        s_camera_mutex = xSemaphoreCreateMutex();
    }
#if CAMERA_ENABLED
    if (!s_camera_frame_ready) {
        s_camera_frame_ready = xSemaphoreCreateBinary();
    }
#endif

    ESP_LOGI(TAG, "Camera service initialized (enabled=%d, free_internal=%u, free_spiram=%u)",
             CAMERA_ENABLED,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

void camera_service_trigger_snapshot(const char *payload) {
    if (!s_camera_mutex) {
        ESP_LOGW(TAG, "Camera snapshot requested before camera service init");
        publish_camera_status("error", "camera_service_not_initialized");
        return;
    }

    if (xSemaphoreTake(s_camera_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Camera snapshot request dropped, camera service busy");
        publish_camera_status("busy", "camera_snapshot_already_running");
        return;
    }

    ESP_LOGI(TAG, "Camera snapshot requested, payload=%s", payload ? payload : "");

#if CAMERA_ENABLED
    size_t received_size = 0;
    uint32_t frame_count_delta = 0;
    esp_err_t ret = camera_capture_locked(&received_size, &frame_count_delta);
    if (ret != ESP_OK) {
        char detail[96];
        snprintf(detail, sizeof(detail), "camera_capture_failed:%s:frames=%u", esp_err_to_name(ret), (unsigned)frame_count_delta);
        publish_camera_status("error", detail);
        ESP_LOGE(TAG, "Camera snapshot failed: %s frame_count_delta=%u payload=%s",
                 esp_err_to_name(ret),
                 (unsigned)frame_count_delta,
                 payload ? payload : "");
    } else {
        char detail[128];
        snprintf(detail, sizeof(detail), "camera_capture_ok:received=%u:frames=%u",
                 (unsigned)received_size,
                 (unsigned)frame_count_delta);
        publish_camera_status("ok", detail);
        ESP_LOGI(TAG, "Camera snapshot callback received_size=%u frame_count_delta=%u payload=%s",
                 (unsigned)received_size,
                 (unsigned)frame_count_delta,
                 payload ? payload : "");
    }
#else
    ESP_LOGW(TAG, "Camera support scaffold is present but disabled at compile time");
    publish_camera_status("disabled", "set_CAMERA_ENABLED_and_enable_psram_before_bringup");
#endif

    xSemaphoreGive(s_camera_mutex);
}

static esp_err_t http_root_get_handler(httpd_req_t *req) {
    char body[768];
    snprintf(body, sizeof(body),
             "<html><body><h1>herc-hotel-p4 camera</h1>"
             "<p><a href=\"/snapshot.jpg\">Capture snapshot.jpg</a></p>"
             "<p><a href=\"/snapshot_gray.jpg\">Debug snapshot_gray.jpg</a></p>"
             "<p><a href=\"/camera_status\">camera_status</a></p>"
             "<pre>initialized=%s\nlast_stage=%s\nlast_error=%s\nlast_http_received=%u\nlast_http_frames=%u\nlast_callback_received=%u\nfinished_count=%u\nframe_buffer_size=%u</pre>"
             "</body></html>",
             s_camera_initialized ? "true" : "false",
             s_camera_last_stage,
             esp_err_to_name(s_camera_last_error),
             (unsigned)s_camera_last_http_received_size,
             (unsigned)s_camera_last_http_frame_delta,
             (unsigned)s_camera_last_received_size,
             (unsigned)s_camera_finished_count,
             (unsigned)s_camera_frame_buffer_size);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t http_camera_status_get_handler(httpd_req_t *req) {
    char body[512];
    snprintf(body, sizeof(body),
             "initialized=%s\nlast_stage=%s\nlast_error=%s\nlast_http_received=%u\nlast_http_frames=%u\nlast_callback_received=%u\nfinished_count=%u\nframe_buffer_size=%u\n",
             s_camera_initialized ? "true" : "false",
             s_camera_last_stage,
             esp_err_to_name(s_camera_last_error),
             (unsigned)s_camera_last_http_received_size,
             (unsigned)s_camera_last_http_frame_delta,
             (unsigned)s_camera_last_received_size,
             (unsigned)s_camera_finished_count,
             (unsigned)s_camera_frame_buffer_size);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t http_send_jpeg_variant(httpd_req_t *req, bool byte_swap_rgb565, bool grayscale_only, const char *filename) {
#if CAMERA_ENABLED
    if (!s_camera_mutex) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "camera service not initialized");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(s_camera_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        httpd_resp_send(req, "camera busy", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    esp_err_t result = ESP_OK;
    size_t received_size = 0;
    uint32_t frame_count_delta = 0;
    uint8_t *jpeg_buf = NULL;
    void *src_buf = NULL;
    jpeg_encoder_handle_t jpeg_handle = NULL;
    size_t jpeg_buf_capacity = 0;
    uint32_t jpeg_size = 0;
    const uint32_t width = CAMERA_MIPI_CSI_HRES;
    const uint32_t height = CAMERA_MIPI_CSI_VRES;
    const uint32_t rgb565_bytes = width * height * CAMERA_HTTP_RGB565_BYTES_PER_PIXEL;
    const uint32_t gray_bytes = width * height;

    esp_err_t ret = camera_capture_locked(&received_size, &frame_count_delta);
    s_camera_last_http_received_size = received_size;
    s_camera_last_http_frame_delta = frame_count_delta;
    const uint32_t image_size = received_size < rgb565_bytes ? (uint32_t)received_size : rgb565_bytes;

    if (ret != ESP_OK || image_size == 0 || !s_camera_frame_buffer) {
        ESP_LOGE(TAG, "HTTP JPEG snapshot capture failed: %s received=%u frames=%u",
                 esp_err_to_name(ret),
                 (unsigned)received_size,
                 (unsigned)frame_count_delta);
        xSemaphoreGive(s_camera_mutex);
        char body[256];
        snprintf(body, sizeof(body),
                 "camera capture failed\nstage=%s\nerr=%s\nreceived=%u\nframes=%u\ncallback_received=%u\nfinished_count=%u\nframe_buffer_size=%u\n",
                 s_camera_last_stage,
                 esp_err_to_name(ret),
                 (unsigned)received_size,
                 (unsigned)frame_count_delta,
                 (unsigned)s_camera_last_received_size,
                 (unsigned)s_camera_finished_count,
                 (unsigned)s_camera_frame_buffer_size);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    jpeg_encode_engine_cfg_t jpeg_engine_cfg = {
        .intr_priority = 0,
        .timeout_ms = 2000,
    };
    jpeg_encode_memory_alloc_cfg_t jpeg_mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    jpeg_encode_cfg_t jpeg_cfg = {
        .height = height,
        .width = width,
        .src_type = grayscale_only ? JPEG_ENCODE_IN_FORMAT_GRAY : JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = grayscale_only ? JPEG_DOWN_SAMPLING_GRAY : JPEG_DOWN_SAMPLING_YUV422,
        .image_quality = CAMERA_HTTP_JPEG_QUALITY,
    };

    if (byte_swap_rgb565 || grayscale_only) {
        size_t work_size = grayscale_only ? gray_bytes : rgb565_bytes;
        src_buf = malloc(work_size);
        if (!src_buf) {
            xSemaphoreGive(s_camera_mutex);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "work buffer alloc failed");
            return ESP_FAIL;
        }

        if (grayscale_only) {
            uint8_t *gray = (uint8_t *)src_buf;
            const uint16_t *rgb = (const uint16_t *)s_camera_frame_buffer;
            for (uint32_t i = 0; i < width * height; ++i) {
                uint16_t px = rgb[i];
                if (byte_swap_rgb565) {
                    px = (uint16_t)((px << 8) | (px >> 8));
                }
                uint8_t r = (uint8_t)(((px >> 11) & 0x1F) * 255 / 31);
                uint8_t g = (uint8_t)(((px >> 5) & 0x3F) * 255 / 63);
                uint8_t b = (uint8_t)((px & 0x1F) * 255 / 31);
                gray[i] = (uint8_t)((r * 30 + g * 59 + b * 11) / 100);
            }
        } else {
            uint8_t *dst = (uint8_t *)src_buf;
            const uint8_t *src = (const uint8_t *)s_camera_frame_buffer;
            for (uint32_t i = 0; i + 1 < image_size; i += 2) {
                dst[i] = src[i + 1];
                dst[i + 1] = src[i];
            }
        }
    } else {
        src_buf = s_camera_frame_buffer;
    }

    uint32_t src_size = grayscale_only ? gray_bytes : image_size;
    uint32_t jpeg_alloc_size = grayscale_only ? gray_bytes : rgb565_bytes;
    jpeg_buf = (uint8_t *)jpeg_alloc_encoder_mem(jpeg_alloc_size, &jpeg_mem_cfg, &jpeg_buf_capacity);
    if (!jpeg_buf) {
        if (src_buf != s_camera_frame_buffer) {
            free(src_buf);
        }
        xSemaphoreGive(s_camera_mutex);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "jpeg buffer alloc failed");
        return ESP_FAIL;
    }

    ret = jpeg_new_encoder_engine(&jpeg_engine_cfg, &jpeg_handle);
    if (ret == ESP_OK) {
        ret = jpeg_encoder_process(jpeg_handle,
                                   &jpeg_cfg,
                                   (const uint8_t *)src_buf,
                                   src_size,
                                   jpeg_buf,
                                   (uint32_t)jpeg_buf_capacity,
                                   &jpeg_size);
    }

    xSemaphoreGive(s_camera_mutex);

    if (ret != ESP_OK || jpeg_size == 0) {
        if (jpeg_handle) jpeg_del_encoder_engine(jpeg_handle);
        if (jpeg_buf) free(jpeg_buf);
        if (src_buf != s_camera_frame_buffer) free(src_buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "jpeg encode failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    if (filename) {
        char content_disposition[96];
        snprintf(content_disposition, sizeof(content_disposition), "inline; filename=%s", filename);
        httpd_resp_set_hdr(req, "Content-Disposition", content_disposition);
    }
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    result = httpd_resp_send(req, (const char *)jpeg_buf, jpeg_size);

    if (jpeg_handle) jpeg_del_encoder_engine(jpeg_handle);
    if (jpeg_buf) free(jpeg_buf);
    if (src_buf != s_camera_frame_buffer) free(src_buf);
    return result;
#else
    httpd_resp_send_err(req, HTTPD_503_SERVICE_UNAVAILABLE, "camera disabled at compile time");
    return ESP_FAIL;
#endif
}

static esp_err_t http_snapshot_jpg_get_handler(httpd_req_t *req) {
    return http_send_jpeg_variant(req, false, false, "snapshot.jpg");
}

static esp_err_t http_snapshot_gray_jpg_get_handler(httpd_req_t *req) {
    return http_send_jpeg_variant(req, false, true, "snapshot_gray.jpg");
}

static esp_err_t http_snapshot_bmp_get_handler(httpd_req_t *req) {
    httpd_resp_set_status(req, "410 Gone");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_send(req, "snapshot.bmp retired; use /snapshot.jpg", HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
}

void http_snapshot_service_init(void) {
    s_http_server = NULL;
}

void http_snapshot_service_start(void) {
    if (s_http_server || !g_app.ip_ready) {
        return;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.uri_match_fn = httpd_uri_match_wildcard;

    esp_err_t ret = httpd_start(&s_http_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed: %s", esp_err_to_name(ret));
        s_http_server = NULL;
        return;
    }

    static const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = http_root_get_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t snapshot_uri = {
        .uri = "/snapshot.bmp",
        .method = HTTP_GET,
        .handler = http_snapshot_bmp_get_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t snapshot_jpg_uri = {
        .uri = "/snapshot.jpg",
        .method = HTTP_GET,
        .handler = http_snapshot_jpg_get_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t snapshot_gray_jpg_uri = {
        .uri = "/snapshot_gray.jpg",
        .method = HTTP_GET,
        .handler = http_snapshot_gray_jpg_get_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t camera_status_uri = {
        .uri = "/camera_status",
        .method = HTTP_GET,
        .handler = http_camera_status_get_handler,
        .user_ctx = NULL,
    };

    ret = httpd_register_uri_handler(s_http_server, &root_uri);
    if (ret == ESP_OK) {
        ret = httpd_register_uri_handler(s_http_server, &camera_status_uri);
    }
    if (ret == ESP_OK) {
        ret = httpd_register_uri_handler(s_http_server, &snapshot_jpg_uri);
    }
    if (ret == ESP_OK) {
        ret = httpd_register_uri_handler(s_http_server, &snapshot_gray_jpg_uri);
    }
    if (ret == ESP_OK) {
        ret = httpd_register_uri_handler(s_http_server, &snapshot_uri);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HTTP URI registration failed: %s", esp_err_to_name(ret));
        httpd_stop(s_http_server);
        s_http_server = NULL;
        return;
    }

    ESP_LOGI(TAG, "HTTP snapshot server started on port %u (GET /snapshot.jpg)", (unsigned)config.server_port);
}

static const ads_named_channel_t s_leak_channels[] = {
    {0, "stbd_jbox_seawater_det", "STBD JBOX Seawater Det"},
    {1, "port_jbox_seawater_det", "PORT JBOX Seawater Det"},
    {2, "xformer_seawater_det", "XFORMER Seawater Det"},
    {3, "kraft_seawater_det", "KRAFT Seawater Det"},
};

static const ads_named_channel_t s_res_level_channels[] = {
    {0, "res_level_1", "Res Level 1"},
    {1, "res_level_2", "Res Level 2"},
    {2, "res_level_3", "Res Level 3"},
    {3, "res_level_4", "Res Level 4"},
};

static const ads_named_channel_t s_pressure_channels[] = {
    {0, "main_hyd_press", "Main Hyd Press"},
    {1, "aux_press_1", "Aux Press 1"},
    {2, "aux_press_2", "Aux Press 2"},
    {3, "aux_press_3", "Aux Press 3"},
};

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

static void status_led_task(void *arg) {
    (void)arg;

    const int led_on_level = STATUS_LED_ACTIVE_LOW ? 0 : 1;
    const int led_off_level = STATUS_LED_ACTIVE_LOW ? 1 : 0;

    gpio_set_level(STATUS_LED_GPIO, led_off_level);
    gpio_set_level(STATUS_LED_GPIO_2, led_off_level);

    while (1) {
        gpio_set_level(STATUS_LED_GPIO, led_on_level);
        gpio_set_level(STATUS_LED_GPIO_2, led_on_level);
        vTaskDelay(pdMS_TO_TICKS(STATUS_LED_BLINK_MS));
        gpio_set_level(STATUS_LED_GPIO, led_off_level);
        gpio_set_level(STATUS_LED_GPIO_2, led_off_level);
        vTaskDelay(pdMS_TO_TICKS(STATUS_LED_BLINK_MS));
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
                float t_c = (float)data.temperature;
                float p_hpa = (float)data.pressure / 100.0f;
                float h_pct = (float)data.humidity;
                float gas_ohm = (float)data.gas_resistance;

                char payload[192];
                snprintf(payload, sizeof(payload), "{\"temperature_c\":%.2f,\"humidity_pct\":%.2f,\"pressure_hpa\":%.2f,\"gas_ohm\":%.0f}",
                         (double)t_c, (double)h_pct, (double)p_hpa, (double)gas_ohm);
                esp_mqtt_client_publish(g_app.mqtt, MQTT_BME_TOPIC_PREFIX, payload, 0, 0, 0);
                ESP_LOGI(TAG, "BME688 sample T=%.2fC RH=%.2f%% P=%.2fhPa Gas=%.0fΩ", (double)t_c, (double)h_pct, (double)p_hpa, (double)gas_ohm);
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

static float ads1115_raw_to_volts(int16_t raw) {
    return ((float)raw) * ADS1115_FULL_SCALE_V / ADS1115_RAW_SCALE;
}

static int motor_temp_volts_to_centi_c(float volts) {
    float temp_c = ((volts * 1000000.0f) / MOTOR_TEMP_SENSE_RESISTOR_OHMS) - 273.15f;
    return (int)(temp_c * 100.0f + (temp_c >= 0.0f ? 0.5f : -0.5f));
}

static int volts_to_percent_int(float volts) {
    float pct = (volts / RES_LEVEL_FULL_SCALE_V) * 100.0f;
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (int)(pct + 0.5f);
}

static int volts_to_psi_int(float volts) {
    float psi = (volts / PRESSURE_SENSOR_FULL_SCALE_V) * PRESSURE_SENSOR_FULL_SCALE_PSI;
    if (psi < 0.0f) psi = 0.0f;
    if (psi > PRESSURE_SENSOR_FULL_SCALE_PSI) psi = PRESSURE_SENSOR_FULL_SCALE_PSI;
    return (int)(psi + 0.5f);
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

                if (addr == 0x48) {
                    int16_t raw = 0;
                    if (ads1115_read_channel(addr, 0, &raw) == ESP_OK) {
                        float volts = ads1115_raw_to_volts(raw);
                        int temp_centi_c = motor_temp_volts_to_centi_c(volts);
                        char topic[96];
                        char payload[160];
                        snprintf(topic, sizeof(topic), "%s/ads1115/0x48/ch0", MQTT_I2C_TOPIC_PREFIX);
                        snprintf(payload, sizeof(payload), "{\"raw\":%d,\"volts\":%.5f,\"temperature_c\":%.2f}",
                                 raw, (double)volts, (double)temp_centi_c / 100.0);
                        esp_mqtt_client_publish(g_app.mqtt, topic, payload, 0, 0, 0);

                        snprintf(topic, sizeof(topic), "%s/motor_temperature", MQTT_I2C_TOPIC_PREFIX);
                        snprintf(payload, sizeof(payload), "{\"temperature_c\":%.2f,\"volts\":%.5f,\"raw\":%d}",
                                 (double)temp_centi_c / 100.0, (double)volts, raw);
                        esp_mqtt_client_publish(g_app.mqtt, topic, payload, 0, 0, 0);
                    }
                } else if (addr == 0x49) {
                    for (size_t i = 0; i < sizeof(s_leak_channels) / sizeof(s_leak_channels[0]); i++) {
                        int16_t raw = 0;
                        uint8_t ch = s_leak_channels[i].channel;
                        if (ads1115_read_channel(addr, ch, &raw) == ESP_OK) {
                            float volts = ads1115_raw_to_volts(raw);
                            const char *state = (volts <= s_leak_wet_threshold_v) ? "WET" : "DRY";
                            char topic[128];
                            char payload[160];
                            snprintf(topic, sizeof(topic), "%s/ads1115/0x49/ch%u", MQTT_I2C_TOPIC_PREFIX, ch);
                            snprintf(payload, sizeof(payload), "{\"raw\":%d,\"volts\":%.5f,\"state\":\"%s\"}",
                                     raw, (double)volts, state);
                            esp_mqtt_client_publish(g_app.mqtt, topic, payload, 0, 0, 0);

                            snprintf(topic, sizeof(topic), "%s/%s", MQTT_LEAK_TOPIC_PREFIX, s_leak_channels[i].key);
                            snprintf(payload, sizeof(payload), "{\"state\":\"%s\",\"volts\":%.5f,\"raw\":%d}",
                                     state, (double)volts, raw);
                            esp_mqtt_client_publish(g_app.mqtt, topic, payload, 0, 0, 0);
                        }
                    }
                } else if (addr == 0x4A) {
                    for (size_t i = 0; i < sizeof(s_res_level_channels) / sizeof(s_res_level_channels[0]); i++) {
                        int16_t raw = 0;
                        uint8_t ch = s_res_level_channels[i].channel;
                        if (ads1115_read_channel(addr, ch, &raw) == ESP_OK) {
                            float volts = ads1115_raw_to_volts(raw);
                            int pct = volts_to_percent_int(volts);
                            char topic[128];
                            char payload[160];
                            snprintf(topic, sizeof(topic), "%s/ads1115/0x4A/ch%u", MQTT_I2C_TOPIC_PREFIX, ch);
                            snprintf(payload, sizeof(payload), "{\"raw\":%d,\"volts\":%.5f,\"percent\":%d}",
                                     raw, (double)volts, pct);
                            esp_mqtt_client_publish(g_app.mqtt, topic, payload, 0, 0, 0);

                            snprintf(topic, sizeof(topic), "%s/%s", MQTT_I2C_TOPIC_PREFIX, s_res_level_channels[i].key);
                            snprintf(payload, sizeof(payload), "{\"percent\":%d,\"volts\":%.5f,\"raw\":%d}",
                                     pct, (double)volts, raw);
                            esp_mqtt_client_publish(g_app.mqtt, topic, payload, 0, 0, 0);
                        }
                    }
                } else if (addr == 0x4B) {
                    for (size_t i = 0; i < sizeof(s_pressure_channels) / sizeof(s_pressure_channels[0]); i++) {
                        int16_t raw = 0;
                        uint8_t ch = s_pressure_channels[i].channel;
                        if (ads1115_read_channel(addr, ch, &raw) == ESP_OK) {
                            float volts = ads1115_raw_to_volts(raw);
                            int psi = volts_to_psi_int(volts);
                            char topic[128];
                            char payload[160];
                            snprintf(topic, sizeof(topic), "%s/ads1115/0x4B/ch%u", MQTT_I2C_TOPIC_PREFIX, ch);
                            snprintf(payload, sizeof(payload), "{\"raw\":%d,\"volts\":%.5f,\"psi\":%d}",
                                     raw, (double)volts, psi);
                            esp_mqtt_client_publish(g_app.mqtt, topic, payload, 0, 0, 0);

                            snprintf(topic, sizeof(topic), "%s/%s", MQTT_I2C_TOPIC_PREFIX, s_pressure_channels[i].key);
                            snprintf(payload, sizeof(payload), "{\"psi\":%d,\"volts\":%.5f,\"raw\":%d}",
                                     psi, (double)volts, raw);
                            esp_mqtt_client_publish(g_app.mqtt, topic, payload, 0, 0, 0);
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

    // Environmental sensor at 0x77, when present as BME688-compatible
    mqtt_publish_ha_sensor("sensor", "herc_hotel_p4_bme_temp", "Herc Hotel Temperature", MQTT_BME_TOPIC_PREFIX,
                           "°C", "temperature", "{{ value_json.temperature_c }}");
    mqtt_publish_ha_sensor("sensor", "herc_hotel_p4_bme_humidity", "Herc Hotel Humidity", MQTT_BME_TOPIC_PREFIX,
                           "%", "humidity", "{{ value_json.humidity_pct }}");
    mqtt_publish_ha_sensor("sensor", "herc_hotel_p4_bme_pressure", "Herc Hotel Pressure", MQTT_BME_TOPIC_PREFIX,
                           "hPa", "pressure", "{{ value_json.pressure_hpa }}");
    mqtt_publish_ha_sensor("sensor", "herc_hotel_p4_bme_gas", "Herc Hotel Gas", MQTT_BME_TOPIC_PREFIX,
                           "Ω", NULL, "{{ value_json.gas_ohm }}");

    // ADS1115 functional channels
    mqtt_publish_ha_sensor("sensor", "herc_hotel_p4_motor_temperature", "Motor Temperature",
                           MQTT_I2C_TOPIC_PREFIX "/motor_temperature", "°C", "temperature", "{{ value_json.temperature_c }}");
    mqtt_publish_ha_sensor("sensor", "herc_hotel_p4_motor_temperature_voltage", "Motor Temperature Voltage",
                           MQTT_I2C_TOPIC_PREFIX "/motor_temperature", "V", "voltage", "{{ value_json.volts }}");

    for (size_t i = 0; i < sizeof(s_leak_channels) / sizeof(s_leak_channels[0]); i++) {
        char topic[128], obj_id[96], name[128], vt[64];
        snprintf(topic, sizeof(topic), "%s/%s", MQTT_LEAK_TOPIC_PREFIX, s_leak_channels[i].key);
        snprintf(obj_id, sizeof(obj_id), "herc_hotel_p4_%s", s_leak_channels[i].key);
        snprintf(name, sizeof(name), "%s", s_leak_channels[i].name);
        mqtt_publish_ha_binary(obj_id, name, topic, "moisture", "WET", "DRY");

        snprintf(obj_id, sizeof(obj_id), "herc_hotel_p4_%s_state", s_leak_channels[i].key);
        snprintf(name, sizeof(name), "%s State", s_leak_channels[i].name);
        snprintf(vt, sizeof(vt), "{{ value_json.state }}");
        mqtt_publish_ha_sensor("sensor", obj_id, name, topic, NULL, NULL, vt);

        snprintf(obj_id, sizeof(obj_id), "herc_hotel_p4_%s_voltage", s_leak_channels[i].key);
        snprintf(name, sizeof(name), "%s Voltage", s_leak_channels[i].name);
        snprintf(vt, sizeof(vt), "{{ value_json.volts }}");
        mqtt_publish_ha_sensor("sensor", obj_id, name, topic, "V", "voltage", vt);
    }

    for (size_t i = 0; i < sizeof(s_res_level_channels) / sizeof(s_res_level_channels[0]); i++) {
        char topic[128], obj_id[96], name[128], vt[64];
        snprintf(topic, sizeof(topic), "%s/%s", MQTT_I2C_TOPIC_PREFIX, s_res_level_channels[i].key);
        snprintf(obj_id, sizeof(obj_id), "herc_hotel_p4_%s", s_res_level_channels[i].key);
        snprintf(name, sizeof(name), "%s", s_res_level_channels[i].name);
        snprintf(vt, sizeof(vt), "{{ value_json.percent }}");
        mqtt_publish_ha_sensor("sensor", obj_id, name, topic, "%", NULL, vt);

        snprintf(obj_id, sizeof(obj_id), "herc_hotel_p4_%s_voltage", s_res_level_channels[i].key);
        snprintf(name, sizeof(name), "%s Voltage", s_res_level_channels[i].name);
        snprintf(vt, sizeof(vt), "{{ value_json.volts }}");
        mqtt_publish_ha_sensor("sensor", obj_id, name, topic, "V", "voltage", vt);
    }

    for (size_t i = 0; i < sizeof(s_pressure_channels) / sizeof(s_pressure_channels[0]); i++) {
        char topic[128], obj_id[96], name[128], vt[64];
        snprintf(topic, sizeof(topic), "%s/%s", MQTT_I2C_TOPIC_PREFIX, s_pressure_channels[i].key);
        snprintf(obj_id, sizeof(obj_id), "herc_hotel_p4_%s", s_pressure_channels[i].key);
        snprintf(name, sizeof(name), "%s", s_pressure_channels[i].name);
        snprintf(vt, sizeof(vt), "{{ value_json.psi }}");
        mqtt_publish_ha_sensor("sensor", obj_id, name, topic, "psi", "pressure", vt);

        snprintf(obj_id, sizeof(obj_id), "herc_hotel_p4_%s_voltage", s_pressure_channels[i].key);
        snprintf(name, sizeof(name), "%s Voltage", s_pressure_channels[i].name);
        snprintf(vt, sizeof(vt), "{{ value_json.volts }}");
        mqtt_publish_ha_sensor("sensor", obj_id, name, topic, "V", "voltage", vt);
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

        if (i2c_probe(s_bme_addr)) {
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
                ESP_LOGW(TAG, "Device present at 0x%02X but BME688 init failed, leaving env sensor disabled for now", s_bme_addr);
            }
        } else {
            ESP_LOGW(TAG, "No environmental sensor detected at 0x%02X", s_bme_addr);
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

void status_led_service_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STATUS_LED_GPIO) | (1ULL << STATUS_LED_GPIO_2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    xTaskCreate(status_led_task, "status_led_task", 2048, NULL, 2, NULL);
    ESP_LOGI(TAG, "Status LEDs blinking on GPIO%d and GPIO%d at 1 Hz", STATUS_LED_GPIO, STATUS_LED_GPIO_2);
}
