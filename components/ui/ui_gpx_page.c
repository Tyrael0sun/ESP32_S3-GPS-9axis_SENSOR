// ui_gpx_page.c
// 说明：GPX 轨迹记录页面

#include "ui_views.h"
#include <stdio.h>

static lv_obj_t *s_state_label;
static lv_obj_t *s_file_label;
static lv_obj_t *s_info_label;

void ui_gpx_page_create(lv_obj_t *parent)
{
    lv_obj_set_style_bg_color(parent, lv_color_hex(0xEEF5FF), LV_PART_MAIN);

    s_state_label = lv_label_create(parent);
    lv_obj_align(s_state_label, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_label_set_text(s_state_label, "状态: 空闲");

    s_file_label = lv_label_create(parent);
    lv_obj_align(s_file_label, LV_ALIGN_TOP_LEFT, 10, 40);
    lv_label_set_text(s_file_label, "当前文件: 无");

    s_info_label = lv_label_create(parent);
    lv_obj_align(s_info_label, LV_ALIGN_TOP_LEFT, 10, 80);
    lv_label_set_text(s_info_label, "距离 0.00km 速度 0.0km/h 卫星0");
}

static const char *gpx_state_str(gpx_state_t state)
{
    switch (state) {
    case GPX_IDLE: return "空闲";
    case GPX_RECORDING: return "记录中";
    case GPX_PAUSED: return "暂停";
    case GPX_STOPPED: return "已停止";
    default: return "未知";
    }
}

void ui_gpx_page_update(const app_data_model_t *m)
{
    char buf[96];
    snprintf(buf, sizeof(buf), "状态: %s", gpx_state_str(m->gpx.state));
    lv_label_set_text(s_state_label, buf);

    snprintf(buf, sizeof(buf), "当前文件: %s", m->gpx.current_file);
    lv_label_set_text(s_file_label, buf);

    snprintf(buf, sizeof(buf),
             "距离 %.2fkm 速度 %.1fkm/h 卫星%u",
             m->gpx.distance_km,
             m->gnss.speed * 3.6f,
             m->gnss.sats);
    lv_label_set_text(s_info_label, buf);
}
