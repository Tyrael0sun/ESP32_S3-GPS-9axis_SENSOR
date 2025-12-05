// ui_bike_page.c
// 说明：Bike 模式主数据页

#include "ui_views.h"
#include <stdio.h>

static lv_obj_t *s_speed_label;
static lv_obj_t *s_dist_label;
static lv_obj_t *s_time_label;
static lv_obj_t *s_alt_label;
static lv_obj_t *s_slope_label;
static lv_obj_t *s_avg_label;
static lv_obj_t *s_max_label;
static lv_obj_t *s_status_label;

void ui_bike_page_create(lv_obj_t *parent)
{
    lv_obj_set_style_bg_color(parent, lv_color_hex(0xFFFFFF), LV_PART_MAIN);

    s_speed_label = lv_label_create(parent);
    lv_obj_align(s_speed_label, LV_ALIGN_TOP_MID, 0, 10);
    lv_label_set_text(s_speed_label, "速度 0.0 km/h");

    s_dist_label = lv_label_create(parent);
    lv_obj_align(s_dist_label, LV_ALIGN_TOP_LEFT, 10, 60);
    lv_label_set_text(s_dist_label, "距离 0.00 km");

    s_time_label = lv_label_create(parent);
    lv_obj_align(s_time_label, LV_ALIGN_TOP_RIGHT, -10, 60);
    lv_label_set_text(s_time_label, "时间 00:00:00");

    s_alt_label = lv_label_create(parent);
    lv_obj_align(s_alt_label, LV_ALIGN_TOP_LEFT, 10, 100);
    lv_label_set_text(s_alt_label, "海拔 0 m");

    s_slope_label = lv_label_create(parent);
    lv_obj_align(s_slope_label, LV_ALIGN_TOP_RIGHT, -10, 100);
    lv_label_set_text(s_slope_label, "坡度 0 %");

    s_avg_label = lv_label_create(parent);
    lv_obj_align(s_avg_label, LV_ALIGN_TOP_LEFT, 10, 140);
    lv_label_set_text(s_avg_label, "平均 0.0 km/h");

    s_max_label = lv_label_create(parent);
    lv_obj_align(s_max_label, LV_ALIGN_TOP_RIGHT, -10, 140);
    lv_label_set_text(s_max_label, "最大 0.0 km/h");

    s_status_label = lv_label_create(parent);
    lv_obj_align(s_status_label, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_label_set_text(s_status_label, "Lap 0 | AutoPause 关");
}

void ui_bike_page_update(const app_data_model_t *m)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "速度 %.1f km/h", m->bike.speed_kmh);
    lv_label_set_text(s_speed_label, buf);

    snprintf(buf, sizeof(buf), "距离 %.2f km", m->bike.trip_distance_km);
    lv_label_set_text(s_dist_label, buf);

    uint32_t t = (uint32_t)m->bike.ride_time_s;
    snprintf(buf, sizeof(buf), "时间 %02lu:%02lu:%02lu",
             (unsigned long)(t / 3600),
             (unsigned long)((t / 60) % 60),
             (unsigned long)(t % 60));
    lv_label_set_text(s_time_label, buf);

    snprintf(buf, sizeof(buf), "海拔 %.0f m", m->sensor.altitude);
    lv_label_set_text(s_alt_label, buf);

    snprintf(buf, sizeof(buf), "坡度 %.1f %%", m->bike.slope_percent);
    lv_label_set_text(s_slope_label, buf);

    snprintf(buf, sizeof(buf), "平均 %.1f km/h", m->bike.avg_speed_kmh);
    lv_label_set_text(s_avg_label, buf);

    snprintf(buf, sizeof(buf), "最大 %.1f km/h", m->bike.max_speed_kmh);
    lv_label_set_text(s_max_label, buf);

    lv_label_set_text_fmt(s_status_label,
                          "Lap %u | AutoPause %s",
                          m->bike.lap_count,
                          m->bike.auto_paused ? "暂停" : "运行");
}
