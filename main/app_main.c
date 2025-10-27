#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "board.h"
#include "sensors.h"
#include "gnss.h"
#include "lcd_spi.h"
#include "encoder.h"

static const char *TAG = "APP";

static void format_temperature(char *dst, size_t dst_len, float value)
{
    if (!dst || dst_len == 0) {
        return;
    }
    if (isnan(value)) {
        snprintf(dst, dst_len, "nan");
    } else {
        snprintf(dst, dst_len, "%.1fC", value);
    }
}

static void format_voltage(char *dst, size_t dst_len, float value)
{
    if (!dst || dst_len == 0) {
        return;
    }
    if (isnan(value)) {
        snprintf(dst, dst_len, "nan");
    } else {
        snprintf(dst, dst_len, "%.2fV", value);
    }
}

static void format_vector(char *dst, size_t dst_len, const float vec[3], const char *unit)
{
    if (!dst || dst_len == 0) {
        return;
    }
    if (!vec || isnan(vec[0]) || isnan(vec[1]) || isnan(vec[2])) {
        snprintf(dst, dst_len, "nan");
        return;
    }
    if (unit) {
        snprintf(dst, dst_len, "[%.2f %.2f %.2f]%s", vec[0], vec[1], vec[2], unit);
    } else {
        snprintf(dst, dst_len, "[%.2f %.2f %.2f]", vec[0], vec[1], vec[2]);
    }
}

static void format_gravity_vector(char *dst, size_t dst_len, const float vec[3])
{
    if (!dst || dst_len == 0) {
        return;
    }
    if (!vec || isnan(vec[0]) || isnan(vec[1]) || isnan(vec[2])) {
        snprintf(dst, dst_len, "nan");
        return;
    }
    const float inv_g = 1.0f / 9.80665f;
    snprintf(dst, dst_len, "[%.2f %.2f %.2f]g",
             vec[0] * inv_g,
             vec[1] * inv_g,
             vec[2] * inv_g);
}

static void log_snapshot(const char *label, bool verbose)
{
    board_state_t board_state = {0};
    board_read_state(&board_state);

    esp_err_t update_err = sensors_update();
    if (update_err != ESP_OK) {
        ESP_LOGW(TAG, "%s sensor update failed (%s)", label, esp_err_to_name(update_err));
    }

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
        if (sensor_state.imu_present) {
            char temp_buf[16];
            char accel_buf[64];
            char accel_g_buf[64];
            char gyro_buf[64];
            format_temperature(temp_buf, sizeof(temp_buf), sensor_state.imu_temperature_c);
            format_vector(accel_buf, sizeof(accel_buf), sensor_state.imu_accel_mps2, "m/s^2");
            format_gravity_vector(accel_g_buf, sizeof(accel_g_buf), sensor_state.imu_accel_mps2);
            format_vector(gyro_buf, sizeof(gyro_buf), sensor_state.imu_gyro_dps, "dps");
            ESP_LOGI(TAG, "%s IMU temp=%s accel=%s grav=%s gyro=%s", label, temp_buf, accel_buf, accel_g_buf, gyro_buf);
        } else {
            ESP_LOGW(TAG, "%s IMU not present", label);
        }

        if (sensor_state.mag_present) {
            char temp_buf[16];
            char mag_buf[64];
            format_temperature(temp_buf, sizeof(temp_buf), sensor_state.mag_temperature_c);
            format_vector(mag_buf, sizeof(mag_buf), sensor_state.mag_uT, "uT");
            ESP_LOGI(TAG, "%s MAG temp=%s field=%s", label, temp_buf, mag_buf);
        } else {
            ESP_LOGW(TAG, "%s MAG not present", label);
        }

        if (sensor_state.press_present) {
            char temp_buf[16];
            format_temperature(temp_buf, sizeof(temp_buf), sensor_state.press_temperature_c);
            ESP_LOGI(TAG, "%s PRESS temp=%s", label, temp_buf);
        } else {
            ESP_LOGW(TAG, "%s PRESS not present", label);
        }
    }

    char batt_buf[16];
    char adc_buf[16];
    format_voltage(batt_buf, sizeof(batt_buf), board_state.battery_voltage);
    format_voltage(adc_buf, sizeof(adc_buf), board_state.battery_adc_voltage);

    ESP_LOGI(TAG, "%s Battery=%s (ADC=%s) charging=%s encoder_pos=%ld", label, batt_buf, adc_buf,
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
    } else if (lcd_spi_install_log_sink() != ESP_OK) {
        ESP_LOGW(TAG, "LCD log sink install failed");
    }

    if (encoder_init() != ESP_OK) {
        ESP_LOGW(TAG, "Encoder init failed");
    }

    if (xTaskCreatePinnedToCore(periodic_monitor_task, "monitor", 4096, NULL, 5, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
    }
}
