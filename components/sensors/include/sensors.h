#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    bool imu_present;
    bool mag_present;
    bool press_present;
    float imu_temperature_c;
    float mag_temperature_c;
    float press_temperature_c;
    float imu_accel_mps2[3];
    float imu_gyro_dps[3];
    float mag_uT[3];
} sensor_snapshot_t;

esp_err_t sensors_init(void);
esp_err_t sensors_update(void);
void sensors_read_snapshot(sensor_snapshot_t *snapshot);
