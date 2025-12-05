// app_main.c
// 说明：系统入口，初始化各模块并运行 SYS_TASK

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"

#include "rtc_manager.h"
#include "data_model.h"
#include "sensor_manager.h"
#include "gnss_driver.h"
#include "input_driver.h"
#include "power_manager.h"
#include "log_system.h"
#include "gpx_recorder.h"
#include "bike_manager.h"
#include "pgear_manager.h"
#include "ui_manager.h"
#include "display_driver.h"

static const char *TAG = "app";

static TaskHandle_t s_sys_task;
static bool s_pgear_fast_rate = false;
static QueueHandle_t s_input_queue = NULL;
static bool s_ui_ready = false;

#define MAIN_MENU_ITEMS 4
#define SETTINGS_OPTION_COUNT 3

static void switch_mode(system_mode_t next);
static void open_settings(system_mode_t origin);
static void handle_main_menu_input(const input_event_t *evt);
static void handle_bike_input(const input_event_t *evt);
static void handle_gpx_input(const input_event_t *evt);
static void handle_pgear_input(const input_event_t *evt);
static void handle_settings_input(const input_event_t *evt);
static void cycle_auto_pause(auto_ctrl_config_t *cfg);
static uint8_t menu_index_for_mode(system_mode_t mode);
static void update_pgear_rate(bool enable);
static void log_start_result(const char *name, esp_err_t err);

static void log_start_result(const char *name, esp_err_t err)
{
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s failed: %s", name, esp_err_to_name(err));
    }
}

static void update_pgear_rate(bool enable)
{
    if (s_pgear_fast_rate == enable) {
        return;
    }
    esp_err_t err = gnss_driver_request_fast_rate(enable);
    if (err == ESP_OK) {
        s_pgear_fast_rate = enable;
    } else {
        log_system_enqueue("[SYS] fast rate %d failed: %s",
                           enable,
                           esp_err_to_name(err));
    }
}

static void switch_mode(system_mode_t next)
{
    app_data_model_t *m = data_model_get();
    system_mode_t prev = m->mode;
    if (prev == next) {
        return;
    }

    if (next == MODE_PGEAR) {
        update_pgear_rate(true);
    } else if (prev == MODE_PGEAR && next != MODE_PGEAR) {
        update_pgear_rate(false);
    }

    if (next == MODE_MAIN_MENU) {
        system_mode_t highlight = (prev == MODE_SETTINGS) ?
                                  m->ui.settings_origin : prev;
        m->ui.main_menu_index = menu_index_for_mode(highlight);
    }

    m->mode = next;
    log_system_enqueue("[SYS] mode %d -> %d", prev, next);
}

static void open_settings(system_mode_t origin)
{
    app_data_model_t *m = data_model_get();
    m->ui.settings_origin = origin;
    m->ui.settings_index = 0;
    switch_mode(MODE_SETTINGS);
}

static void cycle_auto_pause(auto_ctrl_config_t *cfg)
{
    float next_pause = cfg->pause_threshold_kmh + 1.0f;
    if (next_pause > 8.0f) {
        next_pause = 3.0f;
    }
    cfg->pause_threshold_kmh = next_pause;
    cfg->resume_threshold_kmh = cfg->pause_threshold_kmh + 2.0f;
    log_system_enqueue("[SET] auto pause %.1f/%.1f kmh delay %us",
                       cfg->pause_threshold_kmh,
                       cfg->resume_threshold_kmh,
                       cfg->pause_delay_s);
}

static uint8_t menu_index_for_mode(system_mode_t mode)
{
    switch (mode) {
    case MODE_BIKE: return 0;
    case MODE_GPX: return 1;
    case MODE_PGEAR: return 2;
    case MODE_SETTINGS: return 3;
    default: return 0;
    }
}

static void handle_input_event(const input_event_t *evt)
{
    app_data_model_t *m = data_model_get();
    switch (m->mode) {
    case MODE_MAIN_MENU:
        handle_main_menu_input(evt);
        break;
    case MODE_BIKE:
        handle_bike_input(evt);
        break;
    case MODE_GPX:
        handle_gpx_input(evt);
        break;
    case MODE_PGEAR:
        handle_pgear_input(evt);
        break;
    case MODE_SETTINGS:
        handle_settings_input(evt);
        break;
    default:
        break;
    }
}

static void handle_main_menu_input(const input_event_t *evt)
{
    app_data_model_t *m = data_model_get();
    switch (evt->type) {
    case INPUT_EVENT_ENC_LEFT:
        m->ui.main_menu_index = (m->ui.main_menu_index == 0) ?
                                (MAIN_MENU_ITEMS - 1) :
                                (m->ui.main_menu_index - 1);
        break;
    case INPUT_EVENT_ENC_RIGHT:
        m->ui.main_menu_index = (m->ui.main_menu_index + 1) % MAIN_MENU_ITEMS;
        break;
    case INPUT_EVENT_BTN_CLICK:
    case INPUT_EVENT_BTN_MID:
        switch (m->ui.main_menu_index) {
        case 0: switch_mode(MODE_BIKE); break;
        case 1: switch_mode(MODE_GPX); break;
        case 2: switch_mode(MODE_PGEAR); break;
        case 3: open_settings(MODE_MAIN_MENU); break;
        default: break;
        }
        break;
    default:
        break;
    }
}

static void handle_bike_input(const input_event_t *evt)
{
    switch (evt->type) {
    case INPUT_EVENT_BTN_CLICK:
    case INPUT_EVENT_BTN_MID:
        switch_mode(MODE_MAIN_MENU);
        break;
    case INPUT_EVENT_BTN_DOUBLE:
        bike_manager_handle_lap();
        log_system_enqueue("[BIKE] manual lap=%u", data_model_get()->bike.lap_count);
        break;
    case INPUT_EVENT_BTN_LONG:
        open_settings(MODE_BIKE);
        break;
    case INPUT_EVENT_BTN_ULTRA:
        switch_mode(MODE_MAIN_MENU);
        break;
    default:
        break;
    }
}

static void handle_gpx_input(const input_event_t *evt)
{
    app_data_model_t *m = data_model_get();
    switch (evt->type) {
    case INPUT_EVENT_BTN_CLICK:
    case INPUT_EVENT_BTN_MID:
        if (m->gpx.state == GPX_IDLE || m->gpx.state == GPX_STOPPED) {
            if (gpx_recorder_begin_record() == ESP_OK) {
                log_system_enqueue("[GPX] begin file %s", m->gpx.current_file);
            }
        } else if (m->gpx.state == GPX_RECORDING) {
            gpx_recorder_pause();
            log_system_enqueue("[GPX] paused");
        } else if (m->gpx.state == GPX_PAUSED) {
            gpx_recorder_resume();
            log_system_enqueue("[GPX] resumed");
        }
        break;
    case INPUT_EVENT_BTN_DOUBLE:
        log_system_enqueue("[GPX] waypoint marker");
        break;
    case INPUT_EVENT_BTN_LONG:
    case INPUT_EVENT_BTN_ULTRA:
        gpx_recorder_stop();
        switch_mode(MODE_MAIN_MENU);
        break;
    default:
        break;
    }
}

static void handle_pgear_input(const input_event_t *evt)
{
    pgear_event_t ev = {0};
    switch (evt->type) {
    case INPUT_EVENT_BTN_CLICK:
    case INPUT_EVENT_BTN_MID:
        ev.type = PGEAR_EVENT_BTN_CLEAR;
        pgear_manager_post_event(&ev);
        break;
    case INPUT_EVENT_BTN_DOUBLE:
        ev.type = PGEAR_EVENT_BTN_RETEST;
        pgear_manager_post_event(&ev);
        break;
    case INPUT_EVENT_BTN_LONG:
        open_settings(MODE_PGEAR);
        break;
    case INPUT_EVENT_BTN_ULTRA:
        switch_mode(MODE_MAIN_MENU);
        break;
    default:
        break;
    }
}

static void handle_settings_input(const input_event_t *evt)
{
    app_data_model_t *m = data_model_get();
    switch (evt->type) {
    case INPUT_EVENT_ENC_LEFT:
        m->ui.settings_index = (m->ui.settings_index == 0) ?
                               (SETTINGS_OPTION_COUNT - 1) :
                               (m->ui.settings_index - 1);
        break;
    case INPUT_EVENT_ENC_RIGHT:
        m->ui.settings_index = (m->ui.settings_index + 1) % SETTINGS_OPTION_COUNT;
        break;
    case INPUT_EVENT_BTN_CLICK:
    case INPUT_EVENT_BTN_MID:
        if (m->ui.settings_index == 0) {
            switch_mode(m->ui.settings_origin);
        } else if (m->ui.settings_index == 1) {
            cycle_auto_pause(&m->auto_ctrl);
        } else {
            switch_mode(MODE_MAIN_MENU);
        }
        break;
    case INPUT_EVENT_BTN_LONG:
    case INPUT_EVENT_BTN_ULTRA:
        switch_mode(MODE_MAIN_MENU);
        break;
    default:
        break;
    }
}

static void sys_task(void *arg)
{
    input_event_t evt;

    while (1) {
        if (s_input_queue &&
            xQueueReceive(s_input_queue, &evt, pdMS_TO_TICKS(50)) == pdTRUE) {
            handle_input_event(&evt);
        }

        if (s_ui_ready) {
            ui_manager_update();
        }

        rtc_manager_poll();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    data_model_init();
    rtc_manager_init();

    esp_err_t err;

    err = log_system_start();
    log_start_result("log system", err);

    err = heartbeat_task_start();
    log_start_result("heartbeat task", err);

    err = sensor_manager_start();
    log_start_result("sensor manager", err);

    err = gnss_driver_init();
    log_start_result("gnss driver", err);

    err = input_driver_start();
    log_start_result("input driver", err);
    if (err == ESP_OK) {
        s_input_queue = input_event_queue_get();
        if (s_input_queue == NULL) {
            ESP_LOGE(TAG, "input queue unavailable, events disabled");
        }
    }

    err = power_manager_start();
    log_start_result("power manager", err);

    err = gpx_recorder_start();
    log_start_result("gpx recorder", err);

    err = bike_manager_start();
    log_start_result("bike manager", err);

    err = pgear_manager_start();
    log_start_result("pgear manager", err);

    err = display_driver_init();
    log_start_result("display driver", err);

    err = ui_manager_init();
    log_start_result("ui manager", err);
    s_ui_ready = (err == ESP_OK);

    BaseType_t ret = xTaskCreatePinnedToCore(sys_task,
                                             "sys_task",
                                             4096,
                                             NULL,
                                             6,
                                             &s_sys_task,
                                             0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "create sys task failed");
    }

    ESP_LOGI(TAG, "app started");
}
