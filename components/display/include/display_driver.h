// display_driver.h
// 说明：LVGL + LovyanGFX 显示驱动抽象

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 函数说明：初始化 LovyanGFX 与 LVGL 显示对象
esp_err_t display_driver_init(void);

// 函数说明：查询显示是否就绪
bool display_driver_is_ready(void);

// 函数说明：设置背光百分比（0-100）
esp_err_t display_driver_set_backlight(uint8_t percent);

#ifdef __cplusplus
}
#endif
