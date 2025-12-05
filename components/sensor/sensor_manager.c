// sensor_manager.c
// 说明：传感器采样任务与 Mahony 姿态融合

#include "sensor_manager.h"
#include "data_model.h"
#include "imu_lsm6dsr.h"
#include "mag_lis2mdl.h"
#include "baro_bmp388.h"
#include "sensor_types.h"
#include "macro_def.h"
#include "app_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "sensor_mgr";

static TaskHandle_t s_task_handle = NULL;

static void mahony_update(app_data_model_t *m,
                          float dt,
                          const float acc[3],
                          const float gyro_dps[3],
                          const float mag_uT[3]);
static void try_calibrate_baro_p0(app_data_model_t *m);
static void update_baro_altitude(app_data_model_t *m);
static float compute_baro_altitude(float pressure, float p0);

static void sensor_task(void *arg)
{
    app_data_model_t *model = data_model_get();

    float ax, ay, az;
    float gx, gy, gz;
    float imu_temp;
    float mx, my, mz;
    float mag_temp;

    #define STANDARD_GRAVITY    9.80665f
    #define ACC_ANOMALY_HIGH    (1.2f * STANDARD_GRAVITY)
    #define ACC_ANOMALY_LOW     (0.8f * STANDARD_GRAVITY)
    #define MAG_ALPHA           0.05f

    float pressure;
    float baro_temp;

    const TickType_t period_ticks = pdMS_TO_TICKS(20); // 50Hz
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        if (imu_lsm6dsr_read_raw(&ax, &ay, &az, &gx, &gy, &gz, &imu_temp) == ESP_OK) {
            model->sensor.ax = ax;
            model->sensor.ay = ay;
            model->sensor.az = az;
            model->sensor.gx = gx;
            model->sensor.gy = gy;
            model->sensor.gz = gz;
            model->sensor.imu_temp_c = imu_temp;
        }

        if (mag_lis2mdl_read_calibrated(&mx, &my, &mz, &mag_temp) == ESP_OK) {
            model->sensor.mx = mx;
            model->sensor.my = my;
            model->sensor.mz = mz;
            model->sensor.mag_temp_c = mag_temp;
        }

        if (baro_bmp388_read(&pressure, &baro_temp) == ESP_OK) {
            model->sensor.pressure = pressure;
            model->sensor.baro_temp_c = baro_temp;
        }

        try_calibrate_baro_p0(model);
        update_baro_altitude(model);

        // Mahony 更新
        float dt = ((float)period_ticks * (float)portTICK_PERIOD_MS) / 1000.0f;
        float acc_vec[3] = {ax, ay, az};
        float gyro_vec[3] = {gx, gy, gz};
        float mag_vec[3] = {mx, my, mz};
        mahony_update(model, dt, acc_vec, gyro_vec, mag_vec);

        model->sensor.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        model->last_update_ms = model->sensor.timestamp_ms;

        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

esp_err_t sensor_manager_start(void)
{
    CHECK_ESP_RETURN(imu_lsm6dsr_init());

    mag_lis2mdl_config_t mag_cfg = {0};
    mag_cfg.calib.m_matrix[0] = 1.0f;
    mag_cfg.calib.m_matrix[4] = 1.0f;
    mag_cfg.calib.m_matrix[8] = 1.0f;
    CHECK_ESP_RETURN(mag_lis2mdl_init(&mag_cfg));

    CHECK_ESP_RETURN(baro_bmp388_init());

    if (s_task_handle != NULL) {
        ESP_LOGW(TAG, "sensor task already running");
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(sensor_task,
                                              "sensor_task",
                                              4096,
                                              NULL,
                                              6,
                                              &s_task_handle,
                                              0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "create sensor task failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "sensor manager started");
    return ESP_OK;
}

static void mahony_update(app_data_model_t *m,
                          float dt,
                          const float acc[3],
                          const float gyro_dps[3],
                          const float mag_uT[3])
{
    float q0 = m->sensor.q0;
    float q1 = m->sensor.q1;
    float q2 = m->sensor.q2;
    float q3 = m->sensor.q3;

    static float ix = 0.0f, iy = 0.0f, iz = 0.0f;
    static float mag_strength_ref = 0.0f;

    const float ax_raw = acc[0];
    const float ay_raw = acc[1];
    const float az_raw = acc[2];

    float acc_norm = sqrtf(ax_raw * ax_raw + ay_raw * ay_raw + az_raw * az_raw);
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float acc_weight = 0.0f;
    if (acc_norm > 0.0f) {
        ax = ax_raw / acc_norm;
        ay = ay_raw / acc_norm;
        az = az_raw / acc_norm;
        acc_weight = (acc_norm < ACC_ANOMALY_LOW || acc_norm > ACC_ANOMALY_HIGH) ? 0.2f : 1.0f;
    }

    float mx_raw = mag_uT[0];
    float my_raw = mag_uT[1];
    float mz_raw = mag_uT[2];
    float mag_norm = sqrtf(mx_raw * mx_raw + my_raw * my_raw + mz_raw * mz_raw);
    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;
    float mag_weight = 0.0f;
    if (mag_norm > 0.0f) {
        mx = mx_raw / mag_norm;
        my = my_raw / mag_norm;
        mz = mz_raw / mag_norm;
        if (mag_strength_ref <= 0.0f) {
            mag_strength_ref = mag_norm;
        } else {
            mag_strength_ref = (1.0f - MAG_ALPHA) * mag_strength_ref + MAG_ALPHA * mag_norm;
        }
        mag_weight = 1.0f;
        if (mag_strength_ref > 0.0f) {
            float delta = fabsf(mag_norm - mag_strength_ref);
            if (delta > 0.3f * mag_strength_ref) {
                mag_weight = 0.3f;
            }
        }
    }

    float gx = DEG_TO_RAD(gyro_dps[0]);
    float gy = DEG_TO_RAD(gyro_dps[1]);
    float gz = DEG_TO_RAD(gyro_dps[2]);

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) +
                       my * (q1q2 - q0q3) +
                       mz * (q1q3 + q0q2));
    float hy = 2.0f * (mx * (q1q2 + q0q3) +
                       my * (0.5f - q1q1 - q3q3) +
                       mz * (q2q3 - q0q1));
    float bx = sqrtf(hx * hx + hy * hy);
    float bz = 2.0f * (mx * (q1q3 - q0q2) +
                       my * (q2q3 + q0q1) +
                       mz * (0.5f - q1q1 - q2q2));

    float vx = 2.0f * (q1q3 - q0q2);
    float vy = 2.0f * (q0q1 + q2q3);
    float vz = q0q0 - q1q1 - q2q2 + q3q3;

    float wx = 2.0f * bx * (0.5f - q2q2 - q3q3) +
               2.0f * bz * (q1q3 - q0q2);
    float wy = 2.0f * bx * (q1q2 - q0q3) +
               2.0f * bz * (q0q1 + q2q3);
    float wz = 2.0f * bx * (q0q2 + q1q3) +
               2.0f * bz * (0.5f - q1q1 - q2q2);

    float ex = acc_weight * (ay * vz - az * vy) + mag_weight * (my * wz - mz * wy);
    float ey = acc_weight * (az * vx - ax * vz) + mag_weight * (mz * wx - mx * wz);
    float ez = acc_weight * (ax * vy - ay * vx) + mag_weight * (mx * wy - my * wx);

    float kp = 0.5f;
    float ki = 0.05f;

    ix += ki * ex * dt;
    iy += ki * ey * dt;
    iz += ki * ez * dt;

    gx += kp * ex + ix;
    gy += kp * ey + iy;
    gz += kp * ez + iz;

    q0 += (-q1 * gx - q2 * gy - q3 * gz) * (0.5f * dt);
    q1 += (q0 * gx + q2 * gz - q3 * gy) * (0.5f * dt);
    q2 += (q0 * gy - q1 * gz + q3 * gx) * (0.5f * dt);
    q3 += (q0 * gz + q1 * gy - q2 * gx) * (0.5f * dt);

    float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    m->sensor.q0 = q0;
    m->sensor.q1 = q1;
    m->sensor.q2 = q2;
    m->sensor.q3 = q3;

    m->sensor.roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                            1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    m->sensor.pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
    m->sensor.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                           1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;

    float gx_grav = 2.0f * (q1q3 - q0q2) * STANDARD_GRAVITY;
    float gy_grav = 2.0f * (q0q1 + q2q3) * STANDARD_GRAVITY;
    float gz_grav = (q0q0 - q1q1 - q2q2 + q3q3) * STANDARD_GRAVITY;

    m->sensor.gx_grav = gx_grav;
    m->sensor.gy_grav = gy_grav;
    m->sensor.gz_grav = gz_grav;

    m->sensor.ax_lin = ax_raw - gx_grav;
    m->sensor.ay_lin = ay_raw - gy_grav;
    m->sensor.az_lin = az_raw - gz_grav;
}

static void try_calibrate_baro_p0(app_data_model_t *m)
{
    static uint32_t last_cal_ms = 0;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - last_cal_ms < 5000) {
        return;
    }
    if (!m->gnss.valid || m->gnss.speed >= 2.0f || m->sensor.pressure <= 0.0f) {
        return;
    }

    const float T0 = 288.15f;
    const float L = 0.0065f;
    const float R = 287.05f;
    const float g = STANDARD_GRAVITY;

    float altitude = m->gnss.altitude;
    float term = 1.0f - (L * altitude) / T0;
    if (term <= 0.0f) {
        term = 0.01f;
    }

    float exponent = -g / (R * L);
    float p0 = m->sensor.pressure * powf(term, exponent);
    if (!isfinite(p0) || p0 <= 0.0f) {
        return;
    }

    m->sensor.baro_p0 = 0.9f * m->sensor.baro_p0 + 0.1f * p0;
    last_cal_ms = now;
}

static void update_baro_altitude(app_data_model_t *m)
{
    if (m->sensor.pressure <= 0.0f || m->sensor.baro_p0 <= 0.0f) {
        return;
    }
    m->sensor.altitude = compute_baro_altitude(m->sensor.pressure, m->sensor.baro_p0);
}

static float compute_baro_altitude(float pressure, float p0)
{
    const float T0 = 288.15f;
    const float L = 0.0065f;
    const float R = 287.05f;
    const float g = STANDARD_GRAVITY;

    float ratio = p0 / pressure;
    float exponent = (R * L) / g;
    float pow_term = powf(ratio, exponent) - 1.0f;
    return (T0 / L) * pow_term;
}
