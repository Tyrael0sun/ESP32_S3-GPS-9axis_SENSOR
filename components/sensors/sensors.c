#include "sensors.h"

#include <math.h>
#include <string.h>

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_check.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define I2C_PORT                I2C_NUM_0
#define I2C_SCL_GPIO            GPIO_NUM_39
#define I2C_SDA_GPIO            GPIO_NUM_40
#define I2C_FREQ_HZ             400000
#define I2C_TIMEOUT_TICKS       pdMS_TO_TICKS(50)

#define LSM6DSR_WHO_AM_I_REG    0x0F
#define LSM6DSR_ID_PRIMARY      0x6B
#define LSM6DSR_ID_SECONDARY    0x6A
#define LSM6DSR_TEMP_OUT_L      0x20

#define LIS2MDL_WHO_AM_I_REG    0x4F
#define LIS2MDL_ID              0x40
#define LIS2MDL_TEMP_OUT_L      0x2E

#define BMP388_CHIP_ID_REG      0x00
#define BMP388_CHIP_ID          0x50

static const char *TAG = "SENSORS";

static SemaphoreHandle_t s_lock;
static uint8_t s_imu_address;
static uint8_t s_mag_address;
static uint8_t s_press_address;

static esp_err_t sensors_probe_address(uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = i2c_master_start(cmd);
    if (err == ESP_OK) {
        err = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    }
    if (err == ESP_OK) {
        err = i2c_master_stop(cmd);
    }

    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(I2C_PORT, cmd, I2C_TIMEOUT_TICKS);
    }

    i2c_cmd_link_delete(cmd);
    return err;
}

static void sensors_scan_bus(void)
{
    bool found = false;
    for (uint8_t addr = 1; addr < 0x7F; ++addr) {
        if (sensors_probe_address(addr) == ESP_OK) {
            found = true;
            ESP_LOGI(TAG, "I2C device found at 0x%02X", addr);
        }
    }
    if (!found) {
        ESP_LOGW(TAG, "No I2C devices responded");
    }
}
static sensor_snapshot_t s_snapshot = {
    .imu_present = false,
    .mag_present = false,
    .press_present = false,
    .imu_temperature_c = NAN,
    .mag_temperature_c = NAN,
    .press_temperature_c = NAN,
};

static esp_err_t sensors_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, addr, &reg, 1, data, len, I2C_TIMEOUT_TICKS);
}

static esp_err_t sensors_i2c_write(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(I2C_PORT, addr, buf, sizeof(buf), I2C_TIMEOUT_TICKS);
}

static bool sensors_detect_imu(void)
{
    const uint8_t candidates[] = {0x6A, 0x6B};
    uint8_t id = 0;

    for (size_t i = 0; i < sizeof(candidates); ++i) {
        if (sensors_i2c_read(candidates[i], LSM6DSR_WHO_AM_I_REG, &id, 1) == ESP_OK &&
            (id == LSM6DSR_ID_PRIMARY || id == LSM6DSR_ID_SECONDARY)) {
            s_imu_address = candidates[i];
            sensors_i2c_write(s_imu_address, 0x10, 0x60); // 104 Hz accelerometer, 2 g
            sensors_i2c_write(s_imu_address, 0x11, 0x60); // 104 Hz gyro, 2000 dps
            sensors_i2c_write(s_imu_address, 0x12, 0x44); // BDU, auto-increment
            ESP_LOGI(TAG, "LSM6DSR detected at 0x%02X", s_imu_address);
            return true;
        }
    }

    ESP_LOGW(TAG, "LSM6DSR not present");
    return false;
}

static bool sensors_detect_mag(void)
{
    const uint8_t candidates[] = {0x1E};
    uint8_t id = 0;

    for (size_t i = 0; i < sizeof(candidates); ++i) {
        if (sensors_i2c_read(candidates[i], LIS2MDL_WHO_AM_I_REG, &id, 1) == ESP_OK && id == LIS2MDL_ID) {
            s_mag_address = candidates[i];
            sensors_i2c_write(s_mag_address, 0x60, 0x0C); // temp + continuous mode
            ESP_LOGI(TAG, "LIS2MDL detected at 0x%02X", s_mag_address);
            return true;
        }
    }

    ESP_LOGW(TAG, "LIS2MDL not present");
    return false;
}

static bool sensors_detect_press(void)
{
    const uint8_t candidates[] = {0x76, 0x77};
    uint8_t id = 0;

    for (size_t i = 0; i < sizeof(candidates); ++i) {
        if (sensors_i2c_read(candidates[i], BMP388_CHIP_ID_REG, &id, 1) == ESP_OK && id == BMP388_CHIP_ID) {
            s_press_address = candidates[i];
            ESP_LOGI(TAG, "BMP388 detected at 0x%02X", s_press_address);
            return true;
        }
    }

    ESP_LOGW(TAG, "BMP388 not present");
    return false;
}

static bool sensors_read_imu_temp(float *temp_c)
{
    if (!s_snapshot.imu_present) {
        return false;
    }

    uint8_t raw[2] = {0};
    if (sensors_i2c_read(s_imu_address, LSM6DSR_TEMP_OUT_L, raw, sizeof(raw)) != ESP_OK) {
        return false;
    }

    int16_t raw_temp = (int16_t)((raw[1] << 8) | raw[0]);
    *temp_c = ((float)raw_temp / 256.0f) + 25.0f;
    return true;
}

static bool sensors_read_mag_temp(float *temp_c)
{
    if (!s_snapshot.mag_present) {
        return false;
    }

    uint8_t raw[2] = {0};
    if (sensors_i2c_read(s_mag_address, LIS2MDL_TEMP_OUT_L, raw, sizeof(raw)) != ESP_OK) {
        return false;
    }

    int16_t raw_temp = (int16_t)((raw[1] << 8) | raw[0]);
    *temp_c = 25.0f + ((float)raw_temp / 8.0f);
    return true;
}

esp_err_t sensors_init(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        if (!s_lock) {
            return ESP_ERR_NO_MEM;
        }
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    ESP_RETURN_ON_ERROR(i2c_param_config(I2C_PORT, &conf), TAG, "I2C param config failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0), TAG, "I2C driver install failed");

    sensors_scan_bus();

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_snapshot.imu_present = sensors_detect_imu();
    s_snapshot.mag_present = sensors_detect_mag();
    s_snapshot.press_present = sensors_detect_press();
    xSemaphoreGive(s_lock);

    return ESP_OK;
}

void sensors_read_snapshot(sensor_snapshot_t *snapshot)
{
    if (!snapshot) {
        return;
    }

    if (!s_lock) {
        *snapshot = s_snapshot;
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);

    float temp = NAN;
    if (s_snapshot.imu_present && sensors_read_imu_temp(&temp)) {
        s_snapshot.imu_temperature_c = temp;
    }

    temp = NAN;
    if (s_snapshot.mag_present && sensors_read_mag_temp(&temp)) {
        s_snapshot.mag_temperature_c = temp;
    }

    if (!s_snapshot.press_present) {
        s_snapshot.press_temperature_c = NAN;
    }

    *snapshot = s_snapshot;
    xSemaphoreGive(s_lock);
}
