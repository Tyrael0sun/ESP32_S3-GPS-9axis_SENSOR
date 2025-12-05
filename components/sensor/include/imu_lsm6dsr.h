// imu_lsm6dsr.h
// 说明：LSM6DSR IMU 驱动接口

#pragma once

#include "esp_err.h"
#include "sensor_types.h"

esp_err_t imu_lsm6dsr_init(void);
esp_err_t imu_lsm6dsr_read_raw(float *ax, float *ay, float *az,
                               float *gx, float *gy, float *gz,
                               float *temp_c);
