// ui_pgear_page.c
// 说明：P-GEAR 页面

#include "ui_views.h"
#include <stdio.h>

static lv_obj_t *s_status_label;
static lv_obj_t *s_speed_label;
static lv_obj_t *s_time_label;
static lv_obj_t *s_best_label;

void ui_pgear_page_create(lv_obj_t *parent)
{
    lv_obj_set_style_bg_color(parent, lv_color_hex(0xFFF9F0), LV_PART_MAIN);

    s_status_label = lv_label_create(parent);
    lv_obj_align(s_status_label, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_label_set_text(s_status_label, "状态: 等待中");

    s_speed_label = lv_label_create(parent);
    lv_obj_align(s_speed_label, LV_ALIGN_TOP_LEFT, 10, 50);
    lv_label_set_text(s_speed_label, "速度 0.0 km/h 加速度 0.00 g");

    s_time_label = lv_label_create(parent);
    lv_obj_align(s_time_label, LV_ALIGN_TOP_LEFT, 10, 90);
    lv_label_set_text(s_time_label, "本次 0.00 s 目标 0-100");

    s_best_label = lv_label_create(parent);
    lv_obj_align(s_best_label, LV_ALIGN_TOP_LEFT, 10, 130);
    lv_label_set_text(s_best_label, "最佳 0.00 s");
}

static const char *pgear_state_str(pgear_state_t state)
{
    switch (state) {
    case PGEAR_IDLE: return "等待中";
    case PGEAR_ARMED: return "武装";
    case PGEAR_RUNNING: return "计时中";
    case PGEAR_FINISHED: return "完成";
    default: return "未知";
    }
}

void ui_pgear_page_update(const app_data_model_t *m)
{
    char buf[96];
    snprintf(buf, sizeof(buf), "状态: %s", pgear_state_str(m->pgear.state));
    lv_label_set_text(s_status_label, buf);

    snprintf(buf, sizeof(buf), "速度 %.1f km/h 加速度 %.2f g",
             m->pgear.current_speed,
             m->pgear.current_accel_g);
    lv_label_set_text(s_speed_label, buf);

    snprintf(buf, sizeof(buf), "本次 %.2f s 目标 %.0f-%.0f",
             m->pgear.elapsed_time_s,
             m->pgear.v_start,
             m->pgear.v_end);
    lv_label_set_text(s_time_label, buf);

    snprintf(buf, sizeof(buf), "最佳 %.2f s", m->pgear.best_time_s);
    lv_label_set_text(s_best_label, buf);
}
