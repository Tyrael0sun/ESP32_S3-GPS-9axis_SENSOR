// pgear_manager.c
// 说明：P-GEAR 状态机实现

#include "pgear_manager.h"
#include "data_model.h"
#include "macro_def.h"
#include "app_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <math.h>

static const char *TAG = "pgear";

static QueueHandle_t s_event_queue;
static TaskHandle_t s_task_handle;

typedef void (*pgear_state_handler_t)(app_data_model_t *m,
                                      const pgear_event_t *ev);

static void pgear_handle_idle(app_data_model_t *m, const pgear_event_t *ev);
static void pgear_handle_armed(app_data_model_t *m, const pgear_event_t *ev);
static void pgear_handle_running(app_data_model_t *m, const pgear_event_t *ev);
static void pgear_handle_finished(app_data_model_t *m, const pgear_event_t *ev);

static const pgear_state_handler_t s_state_table[PGEAR_FINISHED + 1] = {
    [PGEAR_IDLE]     = pgear_handle_idle,
    [PGEAR_ARMED]    = pgear_handle_armed,
    [PGEAR_RUNNING]  = pgear_handle_running,
    [PGEAR_FINISHED] = pgear_handle_finished,
};

static void pgear_task(void *arg)
{
    app_data_model_t *m = data_model_get();
    pgear_event_t ev;

    while (1) {
        if (xQueueReceive(s_event_queue, &ev, pdMS_TO_TICKS(100)) == pdTRUE) {
            pgear_state_t state = m->pgear.state;
            if (state <= PGEAR_FINISHED && s_state_table[state]) {
                s_state_table[state](m, &ev);
            }
        } else {
            pgear_event_t auto_ev = {
                .type = PGEAR_EVENT_GNSS_UPDATE,
            };
            pgear_state_t state = m->pgear.state;
            if (s_state_table[state]) {
                s_state_table[state](m, &auto_ev);
            }
        }
    }
}

esp_err_t pgear_manager_start(void)
{
    if (!s_event_queue) {
        s_event_queue = xQueueCreate(16, sizeof(pgear_event_t));
        if (!s_event_queue) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (!s_task_handle) {
        BaseType_t ret = xTaskCreatePinnedToCore(pgear_task,
                                                 "pgear_task",
                                                 4096,
                                                 NULL,
                                                 6,
                                                 &s_task_handle,
                                                 0);
        if (ret != pdPASS) {
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "pgear manager started");
    return ESP_OK;
}

void pgear_manager_post_event(const pgear_event_t *ev)
{
    if (!s_event_queue || !ev) {
        return;
    }
    xQueueSend(s_event_queue, ev, 0);
}

static void pgear_handle_idle(app_data_model_t *m, const pgear_event_t *ev)
{
    if (ev->type != PGEAR_EVENT_GNSS_UPDATE) {
        return;
    }
    float speed = m->gnss.speed * 3.6f;
    m->pgear.current_speed = speed;
    m->pgear.current_accel_g = m->sensor.ax_lin / 9.80665f;

    if (fabsf(speed - m->pgear.v_start) <= 2.0f) {
        m->pgear.state = PGEAR_ARMED;
        ESP_LOGI(TAG, "pgear armed");
    }
}

static void pgear_handle_armed(app_data_model_t *m, const pgear_event_t *ev)
{
    float speed = m->gnss.speed * 3.6f;
    float accel = m->sensor.ax_lin / 9.80665f;
    m->pgear.current_speed = speed;
    m->pgear.current_accel_g = accel;

    if (ev->type == PGEAR_EVENT_BTN_CLEAR) {
        m->pgear.state = PGEAR_IDLE;
        return;
    }

    if (speed >= m->pgear.v_start && accel >= m->pgear.g_trigger) {
        m->pgear.elapsed_time_s = 0;
        m->pgear.state = PGEAR_RUNNING;
        ESP_LOGI(TAG, "pgear running");
    }
}

static void pgear_handle_running(app_data_model_t *m, const pgear_event_t *ev)
{
    float speed = m->gnss.speed * 3.6f;
    m->pgear.current_speed = speed;
    m->pgear.elapsed_time_s += 0.1f;

    if (speed >= m->pgear.v_end) {
        m->pgear.state = PGEAR_FINISHED;
        if (m->pgear.elapsed_time_s < m->pgear.best_time_s) {
            m->pgear.best_time_s = m->pgear.elapsed_time_s;
        }
        ESP_LOGI(TAG, "pgear finished %.2fs", m->pgear.elapsed_time_s);
    }
}

static void pgear_handle_finished(app_data_model_t *m, const pgear_event_t *ev)
{
    if (ev->type == PGEAR_EVENT_BTN_CLEAR) {
        m->pgear.state = PGEAR_IDLE;
        m->pgear.elapsed_time_s = 0;
    } else if (ev->type == PGEAR_EVENT_BTN_RETEST) {
        m->pgear.state = PGEAR_ARMED;
        m->pgear.elapsed_time_s = 0;
    }
}
