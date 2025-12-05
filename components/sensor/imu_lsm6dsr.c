// imu_lsm6dsr.c
// 说明：LSM6DSR IMU 驱动实现

#include "imu_lsm6dsr.h"
#include "i2c_bus.h"
#include "macro_def.h"
#include "app_check.h"
#include "esp_log.h"
#include <string.h>

#define LSM6DSR_REG_WHO_AM_I      0x0F
#define LSM6DSR_REG_CTRL1_XL      0x10
#define LSM6DSR_REG_CTRL2_G       0x11
#define LSM6DSR_REG_CTRL3_C       0x12
#define LSM6DSR_REG_OUT_TEMP_L    0x20
#define LSM6DSR_REG_OUTX_L_G      0x22
#define LSM6DSR_REG_OUTX_L_A      0x28

static const char *TAG = "imu_lsm6dsr";

static i2c_master_dev_handle_t s_imu_dev = NULL;

static esp_err_t imu_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    CHECK_ESP_RETURN(i2c_master_transmit_receive(s_imu_dev,
                                                 &reg, 1,
                                                 buf, len,
                                                 1000));
    return ESP_OK;
}

static esp_err_t imu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tmp[2] = {reg, val};
    CHECK_ESP_RETURN(i2c_master_transmit(s_imu_dev, tmp, sizeof(tmp), 1000));
    return ESP_OK;
}

esp_err_t imu_lsm6dsr_init(void)
{
    CHECK_ESP_RETURN(i2c_bus_init());

    i2c_master_bus_handle_t bus = i2c_bus_get();
    i2c_device_config_t dev_cfg = {
        .device_address = LSM6DSR_I2C_ADDR,
        .scl_speed_hz   = 400000,
    };

    CHECK_ESP_RETURN(i2c_master_bus_add_device(bus, &dev_cfg, &s_imu_dev));

    uint8_t who = 0;
    CHECK_ESP_RETURN(imu_read_regs(LSM6DSR_REG_WHO_AM_I, &who, 1));
    if (who != 0x6C) {
        ESP_LOGE(TAG, "invalid who_am_i = 0x%02X", who);
        return ESP_FAIL;
    }

    // 复位设备
    CHECK_ESP_RETURN(imu_write_reg(LSM6DSR_REG_CTRL3_C, 0x01));
    vTaskDelay(pdMS_TO_TICKS(20));

    // 配置加速度计：104 Hz, ±2g
    CHECK_ESP_RETURN(imu_write_reg(LSM6DSR_REG_CTRL1_XL, 0x50));

    // 配置陀螺仪：104 Hz, ±1000 dps
    CHECK_ESP_RETURN(imu_write_reg(LSM6DSR_REG_CTRL2_G, 0x5C));

    ESP_LOGI(TAG, "lsm6dsr init ok");
    return ESP_OK;
}

static float convert_accel(int16_t raw)
{
    // ±2g, 16-bit, 0.061 mg/LSB
    float mg = raw * 0.061f;
    return mg / 1000.0f * 9.80665f;
}

static float convert_gyro(int16_t raw)
{
    // ±1000 dps, 35 mdps/LSB
    float mdps = raw * 35.0f;
    return mdps / 1000.0f;
}

esp_err_t imu_lsm6dsr_read_raw(float *ax, float *ay, float *az,
                               float *gx, float *gy, float *gz,
                               float *temp_c)
{
    CHECK_NULL_RETURN(ax);
    CHECK_NULL_RETURN(ay);
    CHECK_NULL_RETURN(az);
    CHECK_NULL_RETURN(gx);
    CHECK_NULL_RETURN(gy);
    CHECK_NULL_RETURN(gz);
    CHECK_NULL_RETURN(temp_c);

    uint8_t buf[14] = {0};
    CHECK_ESP_RETURN(imu_read_regs(LSM6DSR_REG_OUT_TEMP_L, buf, sizeof(buf)));

    int16_t raw_temp = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t raw_gx = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t raw_gy = (int16_t)(buf[5] << 8 | buf[4]);
    int16_t raw_gz = (int16_t)(buf[7] << 8 | buf[6]);
    int16_t raw_ax = (int16_t)(buf[9] << 8 | buf[8]);
    int16_t raw_ay = (int16_t)(buf[11] << 8 | buf[10]);
    int16_t raw_az = (int16_t)(buf[13] << 8 | buf[12]);

    *temp_c = 25.0f + raw_temp / 256.0f;

    *gx = -convert_gyro(raw_gx);
    *gy =  convert_gyro(raw_gy);
    *gz = -convert_gyro(raw_gz);

    *ax = -convert_accel(raw_ax);
    *ay =  convert_accel(raw_ay);
    *az = -convert_accel(raw_az);

    return ESP_OK;
}
