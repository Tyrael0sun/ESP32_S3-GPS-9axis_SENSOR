// log_system.c
// 说明：日志队列与心跳任务实现

#include "log_system.h"
#include "data_model.h"
#include "app_check.h"
#include "macro_def.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define LOG_QUEUE_LEN 32
#define LOG_LINE_MAX  160

static const char *TAG = "log";

typedef struct {
    char line[LOG_LINE_MAX];
} log_item_t;

static QueueHandle_t s_log_queue = NULL;
static TaskHandle_t s_log_task = NULL;
static TaskHandle_t s_hb_task = NULL;
static uint32_t s_log_drop = 0;
static uint32_t s_hb_miss = 0;
static void emit_heartbeat(void);

static void log_task(void *arg)
{
    log_item_t item;
    while (1) {
        if (xQueueReceive(s_log_queue, &item, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "%s", item.line);
        }
    }
}

void log_system_enqueue(const char *fmt, ...)
{
    if (!s_log_queue) {
        return;
    }

    log_item_t item;
    va_list args;
    va_start(args, fmt);
    vsnprintf(item.line, sizeof(item.line), fmt, args);
    va_end(args);

    if (xQueueSend(s_log_queue, &item, 0) != pdTRUE) {
        s_log_drop++;
    }
}

uint32_t log_system_get_drop_count(void)
{
    return s_log_drop;
}

uint32_t log_system_get_heartbeat_miss(void)
{
    return s_hb_miss;
}

esp_err_t log_system_start(void)
{
    if (!s_log_queue) {
        s_log_queue = xQueueCreate(LOG_QUEUE_LEN, sizeof(log_item_t));
        if (!s_log_queue) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (!s_log_task) {
        BaseType_t ret = xTaskCreatePinnedToCore(log_task,
                                                 "log_task",
                                                 4096,
                                                 NULL,
                                                 5,
                                                 &s_log_task,
                                                 0);
        if (ret != pdPASS) {
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "log system started");
    return ESP_OK;
}

static void heartbeat_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(5000);
    TickType_t last = xTaskGetTickCount();
    while (1) {
        TickType_t start = xTaskGetTickCount();
        emit_heartbeat();
        vTaskDelayUntil(&last, period);
        TickType_t end = xTaskGetTickCount();
        if ((end - start) > period + pdMS_TO_TICKS(50)) {
            s_hb_miss++;
        }
    }
}

esp_err_t heartbeat_task_start(void)
{
    if (s_hb_task) {
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(heartbeat_task,
                                             "heartbeat",
                                             3072,
                                             NULL,
                                             4,
                                             &s_hb_task,
                                             1);
    if (ret != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void emit_heartbeat(void)
{
    app_data_model_t *m = data_model_get();
    log_system_enqueue(
        "[HB] t=%lums mode=%d gnss fix=%d valid=%d sats=%u/%u hdop=%.2f vdop=%.2f pdop=%.2f lat=%.6f lon=%.6f alt=%.1f nmea=%u",
        (unsigned long)m->sensor.timestamp_ms,
        m->mode,
        m->gnss.fix,
        m->gnss.valid,
        m->gnss.sats,
        m->gnss.sats_visible,
        m->gnss.hdop,
        m->gnss.vdop,
        m->gnss.pdop,
        m->gnss.lat,
        m->gnss.lon,
        m->gnss.altitude,
        m->gnss.nmea_ok);
    log_system_enqueue(
        "     sensor ax=%.2f ay=%.2f az=%.2f lin=(%.2f,%.2f,%.2f) gyro=%.1f/%.1f/%.1f yaw=%.1f baro=%.1fkPa alt=%.1f temp=%.1f",
        m->sensor.ax,
        m->sensor.ay,
        m->sensor.az,
        m->sensor.ax_lin,
        m->sensor.ay_lin,
        m->sensor.az_lin,
        m->sensor.gx,
        m->sensor.gy,
        m->sensor.gz,
        m->sensor.yaw,
        m->sensor.pressure / 1000.0f,
        m->sensor.altitude,
        m->sensor.baro_temp_c);
    log_system_enqueue(
        "     sd mounted=%d err=%d gpx_state=%d batt=%.2fV chg=%d drop=%lu hb_miss=%lu",
        m->sd.mounted,
        m->sd.last_err,
        m->gpx.state,
        m->power.vbat,
        m->power.charging,
        (unsigned long)s_log_drop,
        (unsigned long)s_hb_miss);
}
