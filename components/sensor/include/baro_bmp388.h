// baro_bmp388.h
// 说明：BMP388 气压计驱动接口

#pragma once

#include "esp_err.h"

esp_err_t baro_bmp388_init(void);
esp_err_t baro_bmp388_read(float *pressure_pa, float *temp_c);
