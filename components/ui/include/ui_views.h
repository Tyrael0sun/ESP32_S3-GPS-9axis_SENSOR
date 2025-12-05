// ui_views.h
// 说明：UI 页面 Create/Update 接口

#pragma once

#include "lvgl.h"
#include "data_model.h"

void ui_main_menu_create(lv_obj_t *parent);
void ui_main_menu_update(const app_data_model_t *m);

void ui_bike_page_create(lv_obj_t *parent);
void ui_bike_page_update(const app_data_model_t *m);

void ui_gpx_page_create(lv_obj_t *parent);
void ui_gpx_page_update(const app_data_model_t *m);

void ui_pgear_page_create(lv_obj_t *parent);
void ui_pgear_page_update(const app_data_model_t *m);

void ui_settings_page_create(lv_obj_t *parent);
void ui_settings_page_update(const app_data_model_t *m);
