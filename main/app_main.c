#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "board.h"
#include "sensors.h"
#include "gnss.h"
#include "lcd_spi.h"
#include "encoder.h"

static const char *TAG = "APP";

static void log_snapshot(const char *label, bool verbose)
{
    board_state_t board_state = {0};
    board_read_state(&board_state);

    sensor_snapshot_t sensor_state = {0};
    sensors_read_snapshot(&sensor_state);

    if (!gnss_is_present()) {
        ESP_LOGW(TAG, "%s GNSS not present", label);
    } else {
        gnss_fix_t fix = {0};
        gnss_get_last_fix(&fix);
        ESP_LOGI(TAG, "%s GNSS sats=%d lat=%.6f lon=%.6f alt=%.1f", label,
                 gnss_get_tracked_satellites(), fix.latitude_deg, fix.longitude_deg, fix.altitude_m);
    }

    if (verbose) {
        if (sensor_state.imu_present) {
            if (!isnan(sensor_state.imu_temperature_c)) {
                ESP_LOGI(TAG, "%s LSM6DSR temp=%.1fC", label, sensor_state.imu_temperature_c);
            } else {
                ESP_LOGI(TAG, "%s LSM6DSR present", label);
            }
        } else {
            ESP_LOGW(TAG, "%s LSM6DSR not present", label);
        }

        if (sensor_state.mag_present) {
            if (!isnan(sensor_state.mag_temperature_c)) {
                ESP_LOGI(TAG, "%s LIS2MDL temp=%.1fC", label, sensor_state.mag_temperature_c);
            } else {
                ESP_LOGI(TAG, "%s LIS2MDL present", label);
            }
        } else {
            ESP_LOGW(TAG, "%s LIS2MDL not present", label);
        }

        if (sensor_state.press_present) {
            if (!isnan(sensor_state.press_temperature_c)) {
                ESP_LOGI(TAG, "%s BMP388 temp=%.1fC", label, sensor_state.press_temperature_c);
            } else {
                ESP_LOGI(TAG, "%s BMP388 present", label);
            }
        } else {
            ESP_LOGW(TAG, "%s BMP388 not present", label);
        }
    } else {
        ESP_LOGI(TAG, "%s Sensors imu=%s mag=%s press=%s", label,
                 sensor_state.imu_present ? "ok" : "missing",
                 sensor_state.mag_present ? "ok" : "missing",
                 sensor_state.press_present ? "ok" : "missing");
    }

    ESP_LOGI(TAG, "%s Battery=%.2fV charging=%s encoder_pos=%ld", label, board_state.battery_voltage,
             board_state.charging ? "yes" : "no", (long)encoder_get_count());
}

static void periodic_monitor_task(void *ctx)
{
    TickType_t last_wake = xTaskGetTickCount();
    int iteration = 0;

    while (true) {
        const bool verbose = iteration < 3;
        log_snapshot(verbose ? "Boot" : "Heartbeat", verbose);

        TickType_t interval = (iteration < 2) ? pdMS_TO_TICKS(5000) : pdMS_TO_TICKS(10000);
        iteration++;
        vTaskDelayUntil(&last_wake, interval);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "System start");

    if (board_init() != ESP_OK) {
        ESP_LOGE(TAG, "Board init failed");
    }

    if (sensors_init() != ESP_OK) {
        ESP_LOGW(TAG, "Sensors not present");
    }

    if (gnss_init() != ESP_OK) {
        ESP_LOGW(TAG, "GNSS not present");
    }

    if (lcd_spi_init() != ESP_OK) {
        ESP_LOGW(TAG, "LCD init failed");
    }

    if (encoder_init() != ESP_OK) {
        ESP_LOGW(TAG, "Encoder init failed");
    }

    if (xTaskCreatePinnedToCore(periodic_monitor_task, "monitor", 4096, NULL, 5, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
    }
}
