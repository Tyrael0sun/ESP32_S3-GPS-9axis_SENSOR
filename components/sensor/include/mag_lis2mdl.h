// mag_lis2mdl.h
// 说明：LIS2MDL 磁力计驱动接口

#pragma once

#include "esp_err.h"
#include "sensor_types.h"

typedef struct {
    mag_calib_t calib;
} mag_lis2mdl_config_t;

esp_err_t mag_lis2mdl_init(const mag_lis2mdl_config_t *cfg);
esp_err_t mag_lis2mdl_read_calibrated(float *mx, float *my, float *mz,
                                      float *temp_c);
