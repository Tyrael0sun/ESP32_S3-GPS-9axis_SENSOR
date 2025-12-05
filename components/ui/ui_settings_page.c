// ui_settings_page.c
// 说明：设置页面，展示核心配置并允许用户返回原模式

#include "ui_views.h"
#include <stdio.h>

static const char *kSettingsOptions[] = {
    "返回当前模式",
    "调整自动暂停阈值",
    "退出到主菜单",
};

static lv_obj_t *s_option_labels[3];
static lv_obj_t *s_status_label;

static const char *mode_name(system_mode_t mode)
{
    switch (mode) {
    case MODE_MAIN_MENU: return "主菜单";
    case MODE_BIKE: return "Bike";
    case MODE_GPX: return "GPX";
    case MODE_PGEAR: return "P-GEAR";
    case MODE_SETTINGS: return "设置";
    default: return "未知";
    }
}

void ui_settings_page_create(lv_obj_t *parent)
{
    lv_obj_set_style_bg_color(parent, lv_color_hex(0xF0F0FF), LV_PART_MAIN);

    for (size_t i = 0; i < 3; ++i) {
        s_option_labels[i] = lv_label_create(parent);
        lv_obj_align(s_option_labels[i], LV_ALIGN_TOP_LEFT, 10, 20 + (int)i * 30);
        lv_label_set_text(s_option_labels[i], kSettingsOptions[i]);
    }

    s_status_label = lv_label_create(parent);
    lv_obj_align(s_status_label, LV_ALIGN_BOTTOM_LEFT, 10, -20);
    lv_label_set_text(s_status_label, "Auto Pause: 0/0km/h");
}

void ui_settings_page_update(const app_data_model_t *m)
{
    for (size_t i = 0; i < 3; ++i) {
        bool selected = (m->ui.settings_index == i);
        lv_label_set_text_fmt(s_option_labels[i], "%s %s",
                              selected ? ">" : " ",
                              kSettingsOptions[i]);
    }

    lv_label_set_text_fmt(s_status_label,
                          "AutoPause %.1f/%.1f km/h Delay %us | 返回 %s",
                          m->auto_ctrl.pause_threshold_kmh,
                          m->auto_ctrl.resume_threshold_kmh,
                          m->auto_ctrl.pause_delay_s,
                          mode_name(m->ui.settings_origin));
}
