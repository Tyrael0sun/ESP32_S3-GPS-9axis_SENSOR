// bike_manager.c
// 说明：Bike 模式运行逻辑，负责速度/里程/自动暂停等统计

#include "bike_manager.h"
#include "data_model.h"
#include "macro_def.h"
#include "app_check.h"
#include "log_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "bike";

static TaskHandle_t s_task_handle = NULL;

typedef struct {
    float entries[8];
    size_t head;
    size_t tail;
    size_t count;
} speed_ring_t;

static speed_ring_t s_speed_buf;

typedef enum {
    AUTOPAUSE_ACTIVE,
    AUTOPAUSE_WAITING,
    AUTOPAUSE_PAUSED,
} auto_pause_state_t;

static auto_pause_state_t s_auto_state = AUTOPAUSE_ACTIVE;
static TickType_t s_idle_start_tick = 0;
static float s_lap_last_distance = 0.0f;
static uint32_t s_lap_last_time = 0;

static float ring_average(const speed_ring_t *ring)
{
    if (ring->count == 0) {
        return 0.0f;
    }
    float sum = 0.0f;
    for (size_t i = 0, idx = ring->head; i < ring->count; ++i) {
        sum += ring->entries[idx];
        if (idx == 0) {
            idx = 7;
        } else {
            idx--;
        }
    }
    return sum / (float)ring->count;
}

static void ring_push(speed_ring_t *ring, float val)
{
    ring->head = (ring->head + 1) % 8;
    ring->entries[ring->head] = val;
    if (ring->count < 8) {
        ring->count++;
    } else {
        ring->tail = (ring->tail + 1) % 8;
    }
}

static void update_auto_pause(app_data_model_t *m, float speed_kmh)
{
    auto_ctrl_config_t *cfg = &m->auto_ctrl;
    bool below = speed_kmh < cfg->pause_threshold_kmh;
    bool above = speed_kmh > cfg->resume_threshold_kmh;
    TickType_t now = xTaskGetTickCount();

    switch (s_auto_state) {
    case AUTOPAUSE_ACTIVE:
        if (below) {
            s_auto_state = AUTOPAUSE_WAITING;
            s_idle_start_tick = now;
        }
        break;
    case AUTOPAUSE_WAITING:
        if (above) {
            s_auto_state = AUTOPAUSE_ACTIVE;
        } else if (pdTICKS_TO_MS(now - s_idle_start_tick) >= cfg->pause_delay_s * 1000) {
            s_auto_state = AUTOPAUSE_PAUSED;
            m->bike.auto_paused = true;
            log_system_enqueue("[BIKE] auto pause");
        }
        break;
    case AUTOPAUSE_PAUSED:
        if (above) {
            s_auto_state = AUTOPAUSE_ACTIVE;
            m->bike.auto_paused = false;
            log_system_enqueue("[BIKE] auto resume");
        }
        break;
    }
}

static void record_lap_entry(app_data_model_t *m)
{
    if (m->bike.lap_count >= BIKE_MAX_LAPS) {
        log_system_enqueue("[BIKE] lap overflow");
        return;
    }
    uint16_t idx = m->bike.lap_count;
    bike_lap_entry_t *entry = &m->bike.laps[idx];
    entry->seq = idx + 1;
    entry->distance_km = m->bike.trip_distance_km - s_lap_last_distance;
    entry->moving_time_s = m->bike.ride_time_s - s_lap_last_time;
    if (entry->moving_time_s > 0) {
        entry->avg_speed_kmh = entry->distance_km / ((float)entry->moving_time_s / 3600.0f);
    } else {
        entry->avg_speed_kmh = 0.0f;
    }
    m->bike.lap_count++;
    s_lap_last_distance = m->bike.trip_distance_km;
    s_lap_last_time = m->bike.ride_time_s;
    log_system_enqueue("[BIKE] lap=%u dist=%.2fkm", entry->seq, (double)entry->distance_km);
}

static void bike_task(void *arg)
{
    app_data_model_t *m = data_model_get();
    const TickType_t period = pdMS_TO_TICKS(500);

    while (1) {
        float speed_kmh = m->gnss.speed * 3.6f;
        m->bike.speed_kmh = speed_kmh;
        if (speed_kmh > m->bike.max_speed_kmh) {
            m->bike.max_speed_kmh = speed_kmh;
        }

        ring_push(&s_speed_buf, speed_kmh);
        float filtered_speed = ring_average(&s_speed_buf);
        bool moving = filtered_speed >= 1.0f;
        update_auto_pause(m, filtered_speed);

        bool count_motion = moving && !m->bike.auto_paused;
        if (moving) {
            float delta_km = speed_kmh * (0.5f / 3600.0f);
            m->bike.total_distance_km += delta_km;
            if (count_motion) {
                m->bike.trip_distance_km += delta_km;
                m->bike.ride_time_s += 0.5f;
            }
        }
        m->bike.total_time_s += 0.5f;

        float ratio = (m->bike.ride_time_s > 0) ?
                      (m->bike.trip_distance_km / (m->bike.ride_time_s / 3600.0f)) : 0.0f;
        m->bike.avg_speed_kmh = ratio;

        float pitch = m->sensor.pitch;
        m->bike.slope_percent = tanf(pitch * M_PI / 180.0f) * 100.0f;
        m->bike.vert_speed_mh = m->sensor.ax_lin * 3600.0f / 9.80665f;

        vTaskDelay(period);
    }
}

esp_err_t bike_manager_start(void)
{
    if (s_task_handle) {
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(bike_task,
                                             "bike_task",
                                             4096,
                                             NULL,
                                             5,
                                             &s_task_handle,
                                             0);
    if (ret != pdPASS) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "bike manager started");
    return ESP_OK;
}

void bike_manager_handle_lap(void)
{
    app_data_model_t *m = data_model_get();
    record_lap_entry(m);
}
