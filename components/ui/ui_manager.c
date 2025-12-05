// ui_manager.c
// 说明：UI 框架调度，实现页面 create/update 分离

#include "ui_manager.h"
#include "ui_views.h"
#include "data_model.h"
#include "esp_log.h"
#include "lvgl.h"

static const char *TAG = "ui";

static lv_obj_t *s_page_main_menu;
static lv_obj_t *s_page_bike;
static lv_obj_t *s_page_gpx;
static lv_obj_t *s_page_pgear;
static lv_obj_t *s_page_settings;

static esp_err_t create_pages(void)
{
    lv_obj_t *scr = lv_scr_act();
    if (scr == NULL) {
        ESP_LOGE(TAG, "lvgl active screen unavailable");
        return ESP_ERR_INVALID_STATE;
    }

    s_page_main_menu = lv_obj_create(scr);
    lv_obj_set_size(s_page_main_menu, LV_PCT(100), LV_PCT(100));
    ui_main_menu_create(s_page_main_menu);

    s_page_bike = lv_obj_create(scr);
    lv_obj_set_size(s_page_bike, LV_PCT(100), LV_PCT(100));
    ui_bike_page_create(s_page_bike);

    s_page_gpx = lv_obj_create(scr);
    lv_obj_set_size(s_page_gpx, LV_PCT(100), LV_PCT(100));
    ui_gpx_page_create(s_page_gpx);

    s_page_pgear = lv_obj_create(scr);
    lv_obj_set_size(s_page_pgear, LV_PCT(100), LV_PCT(100));
    ui_pgear_page_create(s_page_pgear);

    s_page_settings = lv_obj_create(scr);
    lv_obj_set_size(s_page_settings, LV_PCT(100), LV_PCT(100));
    ui_settings_page_create(s_page_settings);

    return ESP_OK;
}

esp_err_t ui_manager_init(void)
{
    if (s_page_main_menu != NULL) {
        ESP_LOGW(TAG, "ui already initialized");
        return ESP_OK;
    }

    if (!lv_is_initialized()) {
        ESP_LOGE(TAG, "lvgl not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (lv_display_get_default() == NULL) {
        ESP_LOGE(TAG, "lvgl display not registered");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = create_pages();
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "ui manager init done");
    return ESP_OK;
}

static void show_page(lv_obj_t *target)
{
    lv_obj_add_flag(s_page_main_menu, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_page_bike, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_page_gpx, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_page_pgear, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_page_settings, LV_OBJ_FLAG_HIDDEN);

    lv_obj_clear_flag(target, LV_OBJ_FLAG_HIDDEN);
}

void ui_manager_update(void)
{
    const app_data_model_t *m = data_model_get();
    switch (m->mode) {
    case MODE_MAIN_MENU:
        show_page(s_page_main_menu);
        ui_main_menu_update(m);
        break;
    case MODE_BIKE:
        show_page(s_page_bike);
        ui_bike_page_update(m);
        break;
    case MODE_GPX:
        show_page(s_page_gpx);
        ui_gpx_page_update(m);
        break;
    case MODE_PGEAR:
        show_page(s_page_pgear);
        ui_pgear_page_update(m);
        break;
    case MODE_SETTINGS:
        show_page(s_page_settings);
        ui_settings_page_update(m);
        break;
    default:
        show_page(s_page_main_menu);
        break;
    }

    lv_timer_handler();
}
