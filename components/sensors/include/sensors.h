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
} sensor_snapshot_t;

esp_err_t sensors_init(void);
esp_err_t sensors_update(void);
void sensors_read_snapshot(sensor_snapshot_t *snapshot);
