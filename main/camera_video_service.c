#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_cache.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_private/esp_cache_private.h"
#include "example_video_common.h"
#include "linux/videodev2.h"

#include "camera_video_service.h"
#include "services.h"

#define CAMERA_BUFFER_COUNT CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER
#define CAMERA_JPEG_QUALITY CONFIG_EXAMPLE_JPEG_COMPRESSION_QUALITY
#define CAMERA_AE_TARGET 80
#define CAMERA_CAPTURE_LOCK_TIMEOUT_MS 250
#define CAMERA_DQBUF_TIMEOUT_MS 1200
#define CAMERA_DQBUF_RETRY_MS 20

typedef struct {
    int fd;
    example_encoder_handle_t encoder;
    uint8_t *jpeg_out;
    uint32_t jpeg_out_size;
    uint8_t *buffer[CAMERA_BUFFER_COUNT];
    uint32_t buffer_size;
    uint32_t buffer_memory;
    size_t buffer_alignment;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint32_t frame_rate;
    int ae_target;
    SemaphoreHandle_t sem;
    SemaphoreHandle_t capture_sem;
} camera_video_t;

static const char *TAG = "camera_video";
static camera_video_t s_camera = {.fd = -1};
static httpd_handle_t s_httpd;
static bool s_video_init_done;
static bool s_camera_ready;
static char s_last_error[96] = "not_started";
static uint32_t s_capture_ok_count;
static uint32_t s_capture_error_count;

static void set_last_error(const char *msg, esp_err_t err)
{
    if (err == ESP_OK) {
        snprintf(s_last_error, sizeof(s_last_error), "%s", msg);
    } else {
        snprintf(s_last_error, sizeof(s_last_error), "%s:0x%x", msg, (unsigned)err);
    }
}

static esp_err_t sync_buffer(camera_video_t *video, uint32_t index, uint32_t flags)
{
    if (video->buffer_memory != V4L2_MEMORY_USERPTR) {
        return ESP_OK;
    }

    uintptr_t start = (uintptr_t)video->buffer[index];
    size_t align = video->buffer_alignment ? video->buffer_alignment : 64;
    uintptr_t aligned_start = start & ~((uintptr_t)align - 1);
    size_t aligned_size = (size_t)(start + video->buffer_size - aligned_start);
    aligned_size = (aligned_size + align - 1) & ~(align - 1);

    return esp_cache_msync((void *)aligned_start, aligned_size, flags);
}

static esp_err_t capture_jpeg(httpd_req_t *req, camera_video_t *video)
{
    esp_err_t ret = ESP_OK;
    struct v4l2_buffer buf = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = video->buffer_memory,
    };
    uint32_t jpeg_size = 0;
    bool have_capture_lock = false;
    bool have_frame = false;

    ESP_GOTO_ON_FALSE(video->fd >= 0, ESP_ERR_INVALID_STATE, fail, TAG, "camera is not open");
    ESP_GOTO_ON_FALSE(video->capture_sem, ESP_ERR_INVALID_STATE, fail, TAG, "capture semaphore missing");
    ESP_GOTO_ON_FALSE(xSemaphoreTake(video->capture_sem, pdMS_TO_TICKS(CAMERA_CAPTURE_LOCK_TIMEOUT_MS)) == pdPASS,
                      ESP_ERR_TIMEOUT, fail, TAG, "capture already in progress");
    have_capture_lock = true;

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(CAMERA_DQBUF_TIMEOUT_MS);
    do {
        if (ioctl(video->fd, VIDIOC_DQBUF, &buf) == 0) {
            have_frame = true;
            break;
        }
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_GOTO_ON_FALSE(false, ESP_FAIL, fail, TAG, "failed to dequeue frame errno=%d", errno);
        }
        vTaskDelay(pdMS_TO_TICKS(CAMERA_DQBUF_RETRY_MS));
    } while (xTaskGetTickCount() < deadline);

    ESP_GOTO_ON_FALSE(have_frame, ESP_ERR_TIMEOUT, fail, TAG, "timed out waiting for camera frame");
    ESP_GOTO_ON_FALSE(buf.flags & V4L2_BUF_FLAG_DONE, ESP_ERR_INVALID_RESPONSE, requeue, TAG, "frame not done");
    ESP_GOTO_ON_ERROR(sync_buffer(video, buf.index, ESP_CACHE_MSYNC_FLAG_DIR_M2C), requeue, TAG, "failed to sync for cpu");

    httpd_resp_set_type(req, "image/jpeg");
    if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
        ESP_GOTO_ON_ERROR(httpd_resp_send(req, (char *)video->buffer[buf.index], buf.bytesused), requeue, TAG, "failed to send jpeg");
    } else {
        ESP_GOTO_ON_FALSE(xSemaphoreTake(video->sem, portMAX_DELAY) == pdPASS, ESP_FAIL, requeue, TAG, "failed to lock encoder");
        ret = example_encoder_process(video->encoder, video->buffer[buf.index], video->buffer_size,
                                      video->jpeg_out, video->jpeg_out_size, &jpeg_size);
        xSemaphoreGive(video->sem);
        ESP_GOTO_ON_ERROR(ret, requeue, TAG, "failed to encode frame");
        ESP_GOTO_ON_ERROR(httpd_resp_send(req, (char *)video->jpeg_out, jpeg_size), requeue, TAG, "failed to send encoded jpeg");
    }

requeue:
    if (video->buffer_memory == V4L2_MEMORY_USERPTR && buf.index < CAMERA_BUFFER_COUNT) {
        sync_buffer(video, buf.index, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        buf.m.userptr = (unsigned long)video->buffer[buf.index];
        buf.length = video->buffer_size;
    }
    if (buf.index < CAMERA_BUFFER_COUNT) {
        ioctl(video->fd, VIDIOC_QBUF, &buf);
    }
fail:
    if (ret != ESP_OK) {
        set_last_error("capture_failed", ret);
        s_capture_error_count++;
    } else {
        s_capture_ok_count++;
        set_last_error("ok", ESP_OK);
    }
    if (have_capture_lock) {
        xSemaphoreGive(video->capture_sem);
    }
    return ret;
}

static esp_err_t snapshot_handler(httpd_req_t *req)
{
    neopixel_suppress_default_for_snapshot();
    esp_err_t ret = capture_jpeg(req, &s_camera);
    if (ret != ESP_OK) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, s_last_error);
    }
    return ESP_OK;
}

static esp_err_t focus_page_handler(httpd_req_t *req)
{
    static const char html[] =
        "<!doctype html><html><head><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>Hotel Camera Focus</title><style>"
        "body{margin:0;background:#111;color:#eee;font-family:Arial,sans-serif}main{max-width:1100px;margin:auto;padding:14px}"
        "img{width:100%;height:auto;background:#000;image-rendering:auto}.bar{display:flex;gap:16px;align-items:center;flex-wrap:wrap}"
        ".score{font-size:28px;font-weight:700}.hint{color:#bbb}.ok{color:#8f8}"
        "</style></head><body><main><div class=\"bar\"><div class=\"score\">Sharpness: <span id=\"s\">--</span></div>"
        "<div class=\"hint\">Turn lens slowly. Peak score = best focus at current target distance.</div></div>"
        "<img id=\"v\"><canvas id=\"c\" width=\"320\" height=\"320\" style=\"display:none\"></canvas>"
        "<script>const img=document.getElementById('v'),c=document.getElementById('c'),x=c.getContext('2d'),s=document.getElementById('s');"
        "let busy=false;function load(){if(busy)return;busy=true;img.src='/snapshot.jpg?t='+Date.now();}"
        "img.onload=()=>{busy=false;tick();setTimeout(load,400)};img.onerror=()=>{busy=false;setTimeout(load,1000)};"
        "function tick(){try{x.drawImage(img,0,0,c.width,c.height);const d=x.getImageData(0,0,c.width,c.height).data;"
        "let sum=0,n=0,w=c.width;for(let y=1;y<c.height-1;y+=2){for(let q=4;q<(w-1)*4;q+=8){let i=y*w*4+q;"
        "let a=(d[i]+d[i+1]+d[i+2]),b=(d[i+4]+d[i+5]+d[i+6]),e=(d[i+w*4]+d[i+w*4+1]+d[i+w*4+2]);"
        "sum+=Math.abs(a-b)+Math.abs(a-e);n++;}}s.textContent=Math.round(sum/Math.max(1,n));}catch(e){}}"
        "load();</script></main></body></html>";

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char payload[384];
    snprintf(payload, sizeof(payload),
             "{\"ready\":%s,\"last_error\":\"%s\",\"fd\":%d,\"width\":%" PRIu32
             ",\"height\":%" PRIu32 ",\"pixel_format\":%" PRIu32
             ",\"frame_rate\":%" PRIu32 ",\"buffer_size\":%" PRIu32
             ",\"jpeg_quality\":%u,\"ae_target\":%d"
             ",\"capture_ok\":%" PRIu32 ",\"capture_errors\":%" PRIu32
             ",\"free_spiram\":%u}",
             s_camera_ready ? "true" : "false",
             s_last_error,
             s_camera.fd,
             s_camera.width,
             s_camera.height,
             s_camera.pixel_format,
             s_camera.frame_rate,
             s_camera.buffer_size,
             (unsigned)CAMERA_JPEG_QUALITY,
             s_camera.ae_target,
             s_capture_ok_count,
             s_capture_error_count,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, payload);
}

static esp_err_t init_camera_device(void)
{
    esp_err_t ret;
    struct v4l2_format format = {0};
    struct v4l2_requestbuffers req = {0};
    struct v4l2_streamparm streamparm = {0};
    struct v4l2_buffer buf = {0};
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (s_camera_ready) {
        return ESP_OK;
    }

    s_camera.fd = open(EXAMPLE_CAM_DEV_PATH, O_RDWR | O_NONBLOCK);
    ESP_GOTO_ON_FALSE(s_camera.fd >= 0, ESP_FAIL, fail, TAG, "failed to open %s", EXAMPLE_CAM_DEV_PATH);

    s_camera.ae_target = CAMERA_AE_TARGET;
    ESP_LOGI(TAG, "Camera AE target requested in OV5647 sensor init: %d", CAMERA_AE_TARGET);

    format.type = type;
    ESP_GOTO_ON_ERROR(ioctl(s_camera.fd, VIDIOC_G_FMT, &format), fail, TAG, "VIDIOC_G_FMT failed");
    if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB565 &&
        format.fmt.pix.pixelformat != V4L2_PIX_FMT_JPEG) {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
        ESP_GOTO_ON_ERROR(ioctl(s_camera.fd, VIDIOC_S_FMT, &format), fail, TAG, "VIDIOC_S_FMT RGB565 failed");
        ESP_GOTO_ON_ERROR(ioctl(s_camera.fd, VIDIOC_G_FMT, &format), fail, TAG, "VIDIOC_G_FMT after set failed");
    }

    s_camera.width = format.fmt.pix.width;
    s_camera.height = format.fmt.pix.height;
    s_camera.pixel_format = format.fmt.pix.pixelformat;

    streamparm.type = type;
    if (ioctl(s_camera.fd, VIDIOC_G_PARM, &streamparm) == ESP_OK &&
        streamparm.parm.capture.timeperframe.numerator != 0) {
        s_camera.frame_rate = streamparm.parm.capture.timeperframe.denominator /
                              streamparm.parm.capture.timeperframe.numerator;
    }

    req.count = CAMERA_BUFFER_COUNT;
    req.type = type;
    req.memory = V4L2_MEMORY_USERPTR;
    ESP_GOTO_ON_ERROR(ioctl(s_camera.fd, VIDIOC_REQBUFS, &req), fail, TAG, "VIDIOC_REQBUFS failed");
    ESP_GOTO_ON_FALSE(req.count == CAMERA_BUFFER_COUNT, ESP_ERR_NO_MEM, fail, TAG, "driver allocated %u buffers", req.count);
    s_camera.buffer_memory = req.memory;

    for (int i = 0; i < CAMERA_BUFFER_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = type;
        buf.memory = s_camera.buffer_memory;
        buf.index = i;
        ESP_GOTO_ON_ERROR(ioctl(s_camera.fd, VIDIOC_QUERYBUF, &buf), fail, TAG, "VIDIOC_QUERYBUF failed");
        if (i == 0) {
            s_camera.buffer_size = buf.length;
            ESP_GOTO_ON_ERROR(esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &s_camera.buffer_alignment), fail, TAG, "cache alignment failed");
        }
        s_camera.buffer[i] = heap_caps_aligned_calloc(s_camera.buffer_alignment, 1, buf.length, MALLOC_CAP_SPIRAM);
        ESP_GOTO_ON_FALSE(s_camera.buffer[i], ESP_ERR_NO_MEM, fail, TAG, "failed to allocate camera buffer");
        ESP_GOTO_ON_ERROR(sync_buffer(&s_camera, i, ESP_CACHE_MSYNC_FLAG_DIR_C2M), fail, TAG, "failed to sync buffer for dma");
        buf.m.userptr = (unsigned long)s_camera.buffer[i];
        buf.length = s_camera.buffer_size;
        ESP_GOTO_ON_ERROR(ioctl(s_camera.fd, VIDIOC_QBUF, &buf), fail, TAG, "VIDIOC_QBUF failed");
    }

    example_encoder_config_t enc_cfg = {
        .width = s_camera.width,
        .height = s_camera.height,
        .pixel_format = s_camera.pixel_format,
        .quality = CAMERA_JPEG_QUALITY,
    };
    ESP_GOTO_ON_ERROR(example_encoder_init(&enc_cfg, &s_camera.encoder), fail, TAG, "encoder init failed");
    ESP_GOTO_ON_ERROR(example_encoder_alloc_output_buffer(s_camera.encoder, &s_camera.jpeg_out, &s_camera.jpeg_out_size),
                      fail, TAG, "encoder output allocation failed");
    s_camera.sem = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(s_camera.sem, ESP_ERR_NO_MEM, fail, TAG, "encoder semaphore allocation failed");
    s_camera.capture_sem = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(s_camera.capture_sem, ESP_ERR_NO_MEM, fail, TAG, "capture semaphore allocation failed");

    ESP_GOTO_ON_ERROR(ioctl(s_camera.fd, VIDIOC_STREAMON, &type), fail, TAG, "VIDIOC_STREAMON failed");

    s_camera_ready = true;
    set_last_error("ok", ESP_OK);
    ESP_LOGI(TAG, "Camera ready on %s: %" PRIu32 "x%" PRIu32 " fmt=%" PRIu32 " buffer=%" PRIu32,
             EXAMPLE_CAM_DEV_PATH, s_camera.width, s_camera.height, s_camera.pixel_format, s_camera.buffer_size);
    return ESP_OK;

fail:
    set_last_error("init_failed", ret);
    ESP_LOGE(TAG, "Camera init failed: %s", s_last_error);
    return ret;
}

esp_err_t camera_video_service_pre_net_init(void)
{
    if (s_video_init_done) {
        return ESP_OK;
    }

    esp_err_t ret = example_video_init();
    if (ret == ESP_OK) {
        s_video_init_done = true;
        set_last_error("video_init_ok", ESP_OK);
    } else {
        set_last_error("video_init_failed", ret);
    }
    return ret;
}

esp_err_t camera_video_service_start(void)
{
    if (s_httpd) {
        return ESP_OK;
    }

    esp_err_t ret = init_camera_device();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Starting HTTP status endpoint with camera not ready");
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.lru_purge_enable = true;

    ESP_RETURN_ON_ERROR(httpd_start(&s_httpd, &config), TAG, "failed to start camera http server");

    httpd_uri_t status_uri = {
        .uri = "/camera_status",
        .method = HTTP_GET,
        .handler = status_handler,
    };
    httpd_uri_t snapshot_uri = {
        .uri = "/snapshot.jpg",
        .method = HTTP_GET,
        .handler = snapshot_handler,
    };
    httpd_uri_t focus_uri = {
        .uri = "/focus",
        .method = HTTP_GET,
        .handler = focus_page_handler,
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_httpd, &status_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_httpd, &snapshot_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_httpd, &focus_uri));
    ESP_LOGI(TAG, "Camera HTTP endpoints ready: /camera_status /snapshot.jpg /focus");
    return ESP_OK;
}
