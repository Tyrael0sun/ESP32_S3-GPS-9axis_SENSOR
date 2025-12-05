// mag_lis2mdl.c
// 说明：LIS2MDL 磁力计驱动实现

#include "mag_lis2mdl.h"
#include "i2c_bus.h"
#include "macro_def.h"
#include "app_check.h"
#include "esp_log.h"
#include <string.h>

#define LIS2MDL_REG_WHO_AM_I      0x4F
#define LIS2MDL_REG_CFG_REG_A     0x60
#define LIS2MDL_REG_CFG_REG_B     0x61
#define LIS2MDL_REG_CFG_REG_C     0x62
#define LIS2MDL_REG_OUTX_L        0x68
#define LIS2MDL_REG_TEMP_L        0x6E

static const char *TAG = "mag_lis2mdl";

static i2c_master_dev_handle_t s_mag_dev = NULL;
static mag_calib_t s_calib;

static esp_err_t mag_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    CHECK_ESP_RETURN(i2c_master_transmit_receive(s_mag_dev,
                                                 &reg, 1,
                                                 buf, len,
                                                 1000));
    return ESP_OK;
}

static esp_err_t mag_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tmp[2] = {reg, val};
    CHECK_ESP_RETURN(i2c_master_transmit(s_mag_dev, tmp, sizeof(tmp), 1000));
    return ESP_OK;
}

esp_err_t mag_lis2mdl_init(const mag_lis2mdl_config_t *cfg)
{
    CHECK_ESP_RETURN(i2c_bus_init());

    i2c_master_bus_handle_t bus = i2c_bus_get();
    i2c_device_config_t dev_cfg = {
        .device_address = LIS2MDL_I2C_ADDR,
        .scl_speed_hz   = 400000,
    };

    CHECK_ESP_RETURN(i2c_master_bus_add_device(bus, &dev_cfg, &s_mag_dev));

    uint8_t who = 0;
    CHECK_ESP_RETURN(mag_read_regs(LIS2MDL_REG_WHO_AM_I, &who, 1));
    if (who != 0x40) {
        ESP_LOGE(TAG, "invalid who_am_i = 0x%02X", who);
        return ESP_FAIL;
    }

    // ODR 50Hz, 连续模式
    CHECK_ESP_RETURN(mag_write_reg(LIS2MDL_REG_CFG_REG_A, 0x80 | 0x06));
    // 使能温度、低噪声
    CHECK_ESP_RETURN(mag_write_reg(LIS2MDL_REG_CFG_REG_B, 0x01));
    CHECK_ESP_RETURN(mag_write_reg(LIS2MDL_REG_CFG_REG_C, 0x10));

    if (cfg != NULL) {
        s_calib = cfg->calib;
    } else {
        memset(&s_calib, 0, sizeof(s_calib));
        s_calib.m_matrix[0] = 1.0f;
        s_calib.m_matrix[4] = 1.0f;
        s_calib.m_matrix[8] = 1.0f;
    }

    ESP_LOGI(TAG, "lis2mdl init ok");
    return ESP_OK;
}

static float apply_matrix(const float mat[9], const float vec[3], int idx)
{
    return mat[idx * 3 + 0] * vec[0] +
           mat[idx * 3 + 1] * vec[1] +
           mat[idx * 3 + 2] * vec[2];
}

esp_err_t mag_lis2mdl_read_calibrated(float *mx, float *my, float *mz,
                                      float *temp_c)
{
    CHECK_NULL_RETURN(mx);
    CHECK_NULL_RETURN(my);
    CHECK_NULL_RETURN(mz);
    CHECK_NULL_RETURN(temp_c);

    uint8_t buf[8] = {0};
    CHECK_ESP_RETURN(mag_read_regs(LIS2MDL_REG_OUTX_L, buf, sizeof(buf)));

    int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]);
    int16_t raw_temp = (int16_t)(buf[7] << 8 | buf[6]);

    float vec[3] = {
        (float)raw_x,
        (float)raw_y,
        (float)raw_z,
    };

    vec[0] -= s_calib.m_offset[0];
    vec[1] -= s_calib.m_offset[1];
    vec[2] -= s_calib.m_offset[2];

    float cal[3] = {
        apply_matrix(s_calib.m_matrix, vec, 0),
        apply_matrix(s_calib.m_matrix, vec, 1),
        apply_matrix(s_calib.m_matrix, vec, 2),
    };

    // LIS2MDL LSB 约 1.5mG
    const float scale = 0.15f; // µT

    *mx = cal[0] * scale;
    *my = cal[1] * scale;
    *mz = cal[2] * scale;

    *temp_c = 25.0f + raw_temp / 8.0f;

    return ESP_OK;
}
