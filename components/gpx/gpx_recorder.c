// gpx_recorder.c
// 说明：GPX 轨迹记录实现（简化版本）

#include "gpx_recorder.h"
#include "data_model.h"
#include "macro_def.h"
#include "app_check.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "log_system.h"
#include "rtc_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sys/stat.h>
#include <math.h>

#define GPX_QUEUE_LEN              64
#define GPX_BUFFER_SIZE            4096
#define GPX_FLUSH_INTERVAL_MS      3000
#define GPX_MIN_INTERVAL_MS        500
#define GPX_TIME_TRIGGER_MS        1000
#define GPX_DIST_TRIGGER_M         5.0f
#define GPX_STILL_DIST_M           2.0f
#define GPX_STILL_SPEED_MS         0.5f
#define GPX_MAX_POINTS_PER_HOUR    7200

static const char *TAG = "gpx";

typedef struct {
    gnss_fix_t gnss;
    sensor_sample_t sensor;
    time_t rtc_time;
} gpx_point_t;

static sdmmc_card_t *s_card;
static TaskHandle_t s_gpx_task;
static QueueHandle_t s_point_queue;
static FILE *s_file;
static char s_buf_a[GPX_BUFFER_SIZE];
static char s_buf_b[GPX_BUFFER_SIZE];
static char *s_active_buf = s_buf_a;
static size_t s_active_len = 0;
static TickType_t s_last_flush_tick = 0;
static gnss_fix_t s_last_fix = {0};
static bool s_has_last_fix = false;
static uint32_t s_hour_window_start_ms = 0;
static uint32_t s_hour_points = 0;
static bool s_hour_overflow_logged = false;

static void gpx_task(void *arg);
static esp_err_t mount_sdcard(void);
static esp_err_t open_new_file(void);
static void update_sd_state(esp_err_t err);
static bool buffer_point(const gpx_point_t *pt);
static bool flush_active_buffer(bool force);
static bool buffer_literal(const char *literal);
static float haversine_m(double lat1, double lon1, double lat2, double lon2);
static bool should_record_point(const gnss_fix_t *fix, float *distance_m);
static void reset_hour_window(uint32_t timestamp_ms);
static void write_segment_header(void);
static void write_segment_footer(void);

esp_err_t gpx_recorder_start(void)
{
    CHECK_ESP_RETURN(mount_sdcard());

    if (!s_point_queue) {
        s_point_queue = xQueueCreate(GPX_QUEUE_LEN, sizeof(gpx_point_t));
        if (!s_point_queue) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (!s_gpx_task) {
        BaseType_t ret = xTaskCreatePinnedToCore(gpx_task,
                                                 "gpx_task",
                                                 4096,
                                                 NULL,
                                                 5,
                                                 &s_gpx_task,
                                                 0);
        if (ret != pdPASS) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

static esp_err_t mount_sdcard(void)
{
    if (s_card) {
        update_sd_state(ESP_OK);
        return ESP_OK;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 16,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = GPIO_SD_CLK;
    slot_config.cmd = GPIO_SD_CMD;
    slot_config.d0 = GPIO_SD_D0;
    slot_config.d1 = GPIO_SD_D1;
    slot_config.d2 = GPIO_SD_D2;
    slot_config.d3 = GPIO_SD_D3;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard",
                                            &host,
                                            &slot_config,
                                            &mount_config,
                                            &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sd mount failed %s", esp_err_to_name(ret));
        update_sd_state(ret);
        return ret;
    }

    mkdir("/sdcard/GPX", 0777);
    update_sd_state(ESP_OK);
    return ESP_OK;
}

static void update_sd_state(esp_err_t err)
{
    app_data_model_t *m = data_model_get();
    m->sd.mounted = (err == ESP_OK && s_card != NULL);
    m->sd.last_err = err;
}

static esp_err_t open_new_file(void)
{
    app_data_model_t *m = data_model_get();
    time_t now = rtc_manager_now();
    struct tm tm_now;
    localtime_r(&now, &tm_now);

    snprintf(m->gpx.current_file,
             sizeof(m->gpx.current_file),
             "/sdcard/GPX/%04d%02d%02d_%02d%02d%02d.gpx",
             tm_now.tm_year + 1900,
             tm_now.tm_mon + 1,
             tm_now.tm_mday,
             tm_now.tm_hour,
             tm_now.tm_min,
             tm_now.tm_sec);

    s_file = fopen(m->gpx.current_file, "w");
    if (!s_file) {
        update_sd_state(ESP_FAIL);
        return ESP_FAIL;
    }

        fprintf(s_file,
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            "<gpx version=\"1.1\" creator=\"ESP32S3\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n"
            "<trk><name>ESP32S3 Track</name>\n");
    fflush(s_file);

        write_segment_header();

    m->gpx.state = GPX_RECORDING;
    m->gpx.distance_km = 0;
    m->gpx.point_count = 0;
    m->gpx.file_count++;
    s_has_last_fix = false;
    s_active_buf = s_buf_a;
    s_active_len = 0;
    s_last_flush_tick = xTaskGetTickCount();
    reset_hour_window(0);
    return ESP_OK;
}

esp_err_t gpx_recorder_begin_record(void)
{
    if (!s_card) {
        ESP_LOGW(TAG, "sd not ready");
        return ESP_FAIL;
    }
    if (s_file) {
        return ESP_OK;
    }
    return open_new_file();
}

esp_err_t gpx_recorder_pause(void)
{
    app_data_model_t *m = data_model_get();
    if (m->gpx.state == GPX_RECORDING) {
        write_segment_footer();
        flush_active_buffer(true);
        m->gpx.state = GPX_PAUSED;
    }
    return ESP_OK;
}

esp_err_t gpx_recorder_resume(void)
{
    app_data_model_t *m = data_model_get();
    if (s_file && m->gpx.state == GPX_PAUSED) {
        write_segment_header();
        m->gpx.state = GPX_RECORDING;
    }
    return ESP_OK;
}

esp_err_t gpx_recorder_stop(void)
{
    app_data_model_t *m = data_model_get();
    if (s_file) {
        if (m->gpx.state == GPX_RECORDING) {
            write_segment_footer();
        }
        flush_active_buffer(true);
        fprintf(s_file, "</trk>\n</gpx>\n");
        fclose(s_file);
        s_file = NULL;
    }
    m->gpx.state = GPX_IDLE;
    s_has_last_fix = false;
    reset_hour_window(0);
    return ESP_OK;
}

esp_err_t gpx_recorder_push_point(const gnss_fix_t *gnss,
                                  const sensor_sample_t *sensor)
{
    app_data_model_t *m = data_model_get();
    if (!s_point_queue || !gnss || !sensor) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_file || m->gpx.state != GPX_RECORDING) {
        return ESP_OK;
    }
    if (!gnss->valid) {
        return ESP_OK;
    }

    float distance_m = 0.0f;
    if (!should_record_point(gnss, &distance_m)) {
        return ESP_OK;
    }

    if (s_hour_window_start_ms == 0) {
        reset_hour_window(gnss->timestamp_ms);
    }
    if ((gnss->timestamp_ms - s_hour_window_start_ms) >= 3600000) {
        reset_hour_window(gnss->timestamp_ms);
    }
    if (s_hour_points >= GPX_MAX_POINTS_PER_HOUR) {
        if (!s_hour_overflow_logged) {
            log_system_enqueue("[GPX] hourly point overflow, suppressing new points");
            s_hour_overflow_logged = true;
        }
        return ESP_OK;
    }

    gpx_point_t pt = {.gnss = *gnss, .sensor = *sensor};
    pt.rtc_time = rtc_manager_now();
    if (xQueueSend(s_point_queue, &pt, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_has_last_fix) {
        m->gpx.distance_km += distance_m / 1000.0f;
    }
    s_last_fix = *gnss;
    s_has_last_fix = true;
    s_hour_points++;
    return ESP_OK;
}

static void gpx_task(void *arg)
{
    gpx_point_t point;
    app_data_model_t *m = data_model_get();
    const TickType_t wait_ticks = pdMS_TO_TICKS(200);
    const TickType_t flush_ticks = pdMS_TO_TICKS(GPX_FLUSH_INTERVAL_MS);

    while (1) {
        if (xQueueReceive(s_point_queue, &point, wait_ticks) == pdTRUE) {
            if (buffer_point(&point)) {
                m->gpx.point_count++;
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (s_file && (now - s_last_flush_tick) >= flush_ticks) {
            flush_active_buffer(false);
        }
    }
}

static bool buffer_point(const gpx_point_t *pt)
{
    if (!s_file || !pt) {
        return false;
    }

    char time_str[32];
    struct tm tm_utc;
    gmtime_r(&pt->rtc_time, &tm_utc);
    strftime(time_str, sizeof(time_str), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);

    char line[384];
    int len = snprintf(line,
                       sizeof(line),
                       "  <trkpt lat=\"%.6f\" lon=\"%.6f\">\n"
                       "    <ele>%.2f</ele>\n"
                       "    <time>%s</time>\n"
                       "    <extensions>\n"
                       "      <speed>%.2f</speed>\n"
                       "      <course>%.2f</course>\n"
                       "      <pressure>%.2f</pressure>\n"
                       "      <temperature>%.2f</temperature>\n"
                       "    </extensions>\n"
                       "  </trkpt>\n",
                       pt->gnss.lat,
                       pt->gnss.lon,
                       pt->gnss.altitude,
                       time_str,
                       pt->gnss.speed,
                       pt->gnss.course,
                       pt->sensor.pressure,
                       pt->sensor.baro_temp_c);
    if (len <= 0) {
        return false;
    }

    if ((size_t)len >= GPX_BUFFER_SIZE) {
        flush_active_buffer(true);
        size_t written = fwrite(line, 1, len, s_file);
        return written == (size_t)len;
    }

    if ((size_t)len > (GPX_BUFFER_SIZE - s_active_len)) {
        flush_active_buffer(false);
    }
    memcpy(s_active_buf + s_active_len, line, len);
    s_active_len += (size_t)len;
    return true;
}

static bool buffer_literal(const char *literal)
{
    if (!s_file || !literal) {
        return false;
    }
    size_t len = strlen(literal);
    if (len == 0) {
        return true;
    }
    if (len >= GPX_BUFFER_SIZE) {
        flush_active_buffer(true);
        size_t written = fwrite(literal, 1, len, s_file);
        return written == len;
    }
    if (len > (GPX_BUFFER_SIZE - s_active_len)) {
        flush_active_buffer(false);
    }
    memcpy(s_active_buf + s_active_len, literal, len);
    s_active_len += len;
    return true;
}

static bool flush_active_buffer(bool force)
{
    if (!s_file) {
        s_active_len = 0;
        return false;
    }

    TickType_t now = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(GPX_FLUSH_INTERVAL_MS);
    if (!force && s_active_len == 0 && (now - s_last_flush_tick) < interval) {
        return true;
    }

    if (s_active_len == 0) {
        if (force) {
            fflush(s_file);
        }
        s_last_flush_tick = now;
        return true;
    }

    size_t written = fwrite(s_active_buf, 1, s_active_len, s_file);
    if (written != s_active_len) {
        ESP_LOGE(TAG, "gpx flush error, written=%u need=%u",
                 (unsigned)written,
                 (unsigned)s_active_len);
        return false;
    }
    fflush(s_file);
    s_active_buf = (s_active_buf == s_buf_a) ? s_buf_b : s_buf_a;
    s_active_len = 0;
    s_last_flush_tick = now;
    return true;
}

static void write_segment_header(void)
{
    buffer_literal("  <trkseg>\n");
}

static void write_segment_footer(void)
{
    buffer_literal("  </trkseg>\n");
}

static float haversine_m(double lat1, double lon1, double lat2, double lon2)
{
    double dlat = DEG_TO_RAD(lat2 - lat1);
    double dlon = DEG_TO_RAD(lon2 - lon1);
    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(DEG_TO_RAD(lat1)) * cos(DEG_TO_RAD(lat2)) *
               sin(dlon / 2.0) * sin(dlon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return (float)(6371000.0 * c);
}

static bool should_record_point(const gnss_fix_t *fix, float *distance_m)
{
    if (!s_has_last_fix) {
        if (distance_m) {
            *distance_m = 0.0f;
        }
        return true;
    }

    if (fix->timestamp_ms < s_last_fix.timestamp_ms) {
        s_has_last_fix = false;
        if (distance_m) {
            *distance_m = 0.0f;
        }
        return true;
    }

    uint32_t delta_ms = fix->timestamp_ms - s_last_fix.timestamp_ms;
    if (delta_ms < GPX_MIN_INTERVAL_MS) {
        return false;
    }

    float dist = haversine_m(fix->lat, fix->lon, s_last_fix.lat, s_last_fix.lon);
    bool time_trigger = delta_ms >= GPX_TIME_TRIGGER_MS;
    bool dist_trigger = dist >= GPX_DIST_TRIGGER_M;

    if (fix->speed < GPX_STILL_SPEED_MS && dist < GPX_STILL_DIST_M) {
        return false;
    }

    if (distance_m) {
        *distance_m = dist;
    }
    return time_trigger || dist_trigger;
}

static void reset_hour_window(uint32_t timestamp_ms)
{
    s_hour_window_start_ms = timestamp_ms;
    s_hour_points = 0;
    s_hour_overflow_logged = false;
}
