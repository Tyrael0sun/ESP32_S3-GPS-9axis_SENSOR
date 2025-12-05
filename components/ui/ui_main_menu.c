// ui_main_menu.c
// 说明：主菜单页面

#include "ui_views.h"

static const char *kMenuItems[] = {
    "自行车码表 (Bike)",
    "GPS 轨迹记录仪 (GPX)",
    "P-GEAR 加速测试",
    "设置 (Settings)",
};

static lv_obj_t *s_label_title;
static lv_obj_t *s_label_items[4];

void ui_main_menu_create(lv_obj_t *parent)
{
    lv_obj_set_style_bg_color(parent, lv_color_hex(0xF5F5F5), LV_PART_MAIN);

    s_label_title = lv_label_create(parent);
    lv_label_set_text(s_label_title, "ESP32S3 BOX");
    lv_obj_align(s_label_title, LV_ALIGN_TOP_MID, 0, 10);

    for (int i = 0; i < 4; ++i) {
        s_label_items[i] = lv_label_create(parent);
        lv_label_set_text(s_label_items[i], kMenuItems[i]);
        lv_obj_align(s_label_items[i], LV_ALIGN_TOP_LEFT, 20, 50 + i * 30);
    }
}

void ui_main_menu_update(const app_data_model_t *m)
{
    for (int i = 0; i < 4; ++i) {
        bool focused = (m->ui.main_menu_index == (uint8_t)i);
        lv_label_set_text_fmt(s_label_items[i], "%s %s",
                              focused ? ">" : " ",
                              kMenuItems[i]);
    }
}
