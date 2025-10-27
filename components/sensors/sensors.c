#include "sensors.h"
#include "board.h"

#include <math.h>
#include <string.h>

#include <driver/i2c_master.h>
#include <esp_check.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define I2C_MASTER_TIMEOUT_MS       1000

#define LSM6DSR_ADDR                0x6A
#define LSM6DSR_WHO_AM_I_REG        0x0F
#define LSM6DSR_ID_PRIMARY          0x6B
#define LSM6DSR_CTRL1_XL            0x10
#define LSM6DSR_CTRL2_G             0x11
#define LSM6DSR_CTRL3_C             0x12
#define LSM6DSR_OUT_TEMP_L          0x20

#define LIS2MDL_ADDR                0x1E
#define LIS2MDL_WHO_AM_I_REG        0x4F
#define LIS2MDL_ID                  0x40
#define LIS2MDL_CFG_REG_A           0x60
#define LIS2MDL_CFG_REG_C           0x62
#define LIS2MDL_TEMP_OUT_L          0x6E

#define BMP388_ADDR                 0x76
#define BMP388_CHIP_ID_REG          0x00
#define BMP388_CHIP_ID              0x50
#define BMP388_STATUS_REG           0x03
#define BMP388_STATUS_TEMP_READY    (1 << 6)
#define BMP388_PWR_CTRL_REG         0x1B
#define BMP388_OSR_REG              0x1C
#define BMP388_TEMP_DATA_XLSB       0x22
#define BMP388_CALIB_T1_LSB         0x31
#define BMP388_PWR_TEMP_EN          (1 << 4)
#define BMP388_PWR_PRESS_EN         (1 << 3)
#define BMP388_PWR_MODE_NORMAL      0x03

static const char *TAG = "SENSORS";

typedef struct {
	uint16_t par_t1;
	int16_t par_t2;
	int8_t par_t3;
	bool loaded;
} bmp388_calib_data_t;

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_lsm6dsr_dev;
static i2c_master_dev_handle_t s_lis2mdl_dev;
static i2c_master_dev_handle_t s_bmp388_dev;
static const i2c_device_config_t s_lsm6dsr_cfg = {
	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
	.device_address = LSM6DSR_ADDR,
	.scl_speed_hz = BOARD_I2C_FREQ_HZ,
	.scl_wait_us = 0,
	.flags = {
		.disable_ack_check = false,
	},
};
static const i2c_device_config_t s_lis2mdl_cfg = {
	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
	.device_address = LIS2MDL_ADDR,
	.scl_speed_hz = BOARD_I2C_FREQ_HZ,
	.scl_wait_us = 0,
	.flags = {
		.disable_ack_check = false,
	},
};
static const i2c_device_config_t s_bmp388_cfg = {
	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
	.device_address = BMP388_ADDR,
	.scl_speed_hz = BOARD_I2C_FREQ_HZ,
	.scl_wait_us = 0,
	.flags = {
		.disable_ack_check = false,
	},
};
static SemaphoreHandle_t s_state_lock;
static sensor_snapshot_t s_snapshot;
static bmp388_calib_data_t s_bmp_calib;
static bool s_initialized;

static sensor_snapshot_t snapshot_defaults(void)
{
	return (sensor_snapshot_t){
		.imu_present = false,
		.mag_present = false,
		.press_present = false,
		.imu_temperature_c = NAN,
		.mag_temperature_c = NAN,
		.press_temperature_c = NAN,
	};
}

static esp_err_t i2c_device_read(i2c_master_dev_handle_t dev, uint8_t reg_addr, uint8_t *data, size_t len)
{
	if (!dev || !data || len == 0) {
		return ESP_ERR_INVALID_ARG;
	}
	uint8_t reg = reg_addr;
	return i2c_master_transmit_receive(dev, &reg, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t i2c_device_write(i2c_master_dev_handle_t dev, uint8_t reg_addr, const uint8_t *data, size_t len)
{
	if (!dev || !data || len == 0) {
		return ESP_ERR_INVALID_ARG;
	}
	if (len > 15) {
		return ESP_ERR_INVALID_SIZE;
	}
	uint8_t buffer[16];
	buffer[0] = reg_addr;
	memcpy(&buffer[1], data, len);
	return i2c_master_transmit(dev, buffer, len + 1, I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t i2c_device_write_byte(i2c_master_dev_handle_t dev, uint8_t reg_addr, uint8_t value)
{
	return i2c_device_write(dev, reg_addr, &value, 1);
}

static bool detect_device(i2c_master_dev_handle_t dev, uint8_t who_am_i_reg, uint8_t expected_id)
{
	for (int attempt = 0; attempt < 3; ++attempt) {
		uint8_t id = 0;
		esp_err_t err = i2c_device_read(dev, who_am_i_reg, &id, 1);
		ESP_LOGI(TAG, "WHO_AM_I attempt %d reg=0x%02X err=%s immediate=0x%02X", attempt + 1, who_am_i_reg, esp_err_to_name(err), id);
		if (err == ESP_OK && id == expected_id) {
			return true;
		}
		vTaskDelay(pdMS_TO_TICKS(2));
		ESP_LOGI(TAG, "WHO_AM_I attempt %d reg=0x%02X post-delay=0x%02X", attempt + 1, who_am_i_reg, id);
		if (err == ESP_OK && id == expected_id) {
			ESP_LOGI(TAG, "WHO_AM_I ok (reg=0x%02X id=0x%02X attempt=%d)", who_am_i_reg, id, attempt + 1);
			return true;
		}
		if (err != ESP_OK) {
			ESP_LOGW(TAG, "WHO_AM_I read failed (reg=0x%02X err=%s attempt=%d)", who_am_i_reg, esp_err_to_name(err), attempt + 1);
		} else {
			ESP_LOGW(TAG, "WHO_AM_I mismatch (reg=0x%02X got=0x%02X expected=0x%02X attempt=%d)", who_am_i_reg, id, expected_id, attempt + 1);
		}
		vTaskDelay(pdMS_TO_TICKS(3));
	}
	return false;
}

static esp_err_t lsm6dsr_configure(void)
{
	if (!s_lsm6dsr_dev) {
		return ESP_ERR_INVALID_STATE;
	}
	esp_err_t err = i2c_device_write_byte(s_lsm6dsr_dev, LSM6DSR_CTRL3_C, 0x44);
	err |= i2c_device_write_byte(s_lsm6dsr_dev, LSM6DSR_CTRL1_XL, 0x60);
	err |= i2c_device_write_byte(s_lsm6dsr_dev, LSM6DSR_CTRL2_G, 0x60);
	return err;
}

static esp_err_t lsm6dsr_read_temperature(float *temperature_c)
{
	if (!s_lsm6dsr_dev) {
		return ESP_ERR_INVALID_STATE;
	}
	uint8_t buffer[2] = {0};
	esp_err_t err = i2c_device_read(s_lsm6dsr_dev, LSM6DSR_OUT_TEMP_L | 0x80, buffer, sizeof(buffer));
	if (err != ESP_OK) {
		return err;
	}
	int16_t raw = (int16_t)((buffer[1] << 8) | buffer[0]);
	*temperature_c = 25.0f + ((float)raw / 256.0f);
	return ESP_OK;
}

static esp_err_t lis2mdl_configure(void)
{
	if (!s_lis2mdl_dev) {
		return ESP_ERR_INVALID_STATE;
	}
	esp_err_t err = i2c_device_write_byte(s_lis2mdl_dev, LIS2MDL_CFG_REG_A, 0x8C);
	err |= i2c_device_write_byte(s_lis2mdl_dev, LIS2MDL_CFG_REG_C, 0x10);
	return err;
}

static esp_err_t lis2mdl_read_temperature(float *temperature_c)
{
	if (!s_lis2mdl_dev) {
		return ESP_ERR_INVALID_STATE;
	}
	uint8_t buffer[2] = {0};
	esp_err_t err = i2c_device_read(s_lis2mdl_dev, LIS2MDL_TEMP_OUT_L | 0x80, buffer, sizeof(buffer));
	if (err != ESP_OK) {
		return err;
	}
	int16_t raw = (int16_t)((buffer[1] << 8) | buffer[0]);
	*temperature_c = 25.0f + ((float)raw / 8.0f);
	return ESP_OK;
}

static esp_err_t bmp388_load_calibration(void)
{
	if (s_bmp_calib.loaded) {
		return ESP_OK;
	}
	if (!s_bmp388_dev) {
		return ESP_ERR_INVALID_STATE;
	}
	uint8_t buffer[5] = {0};
	ESP_RETURN_ON_ERROR(i2c_device_read(s_bmp388_dev, BMP388_CALIB_T1_LSB, buffer, sizeof(buffer)), TAG, "BMP388 calib read");
	s_bmp_calib.par_t1 = (uint16_t)((buffer[1] << 8) | buffer[0]);
	s_bmp_calib.par_t2 = (int16_t)((buffer[3] << 8) | buffer[2]);
	s_bmp_calib.par_t3 = (int8_t)buffer[4];
	s_bmp_calib.loaded = true;
	return ESP_OK;
}

static esp_err_t bmp388_configure(void)
{
	ESP_RETURN_ON_ERROR(bmp388_load_calibration(), TAG, "Calibration read failed");
	ESP_RETURN_ON_ERROR(i2c_device_write_byte(s_bmp388_dev, BMP388_OSR_REG, 0x22), TAG, "BMP388 OSR cfg");
	uint8_t pwr = BMP388_PWR_TEMP_EN | BMP388_PWR_PRESS_EN | BMP388_PWR_MODE_NORMAL;
	return i2c_device_write_byte(s_bmp388_dev, BMP388_PWR_CTRL_REG, pwr);
}

static esp_err_t bmp388_read_temperature(float *temperature_c)
{
	ESP_RETURN_ON_ERROR(bmp388_load_calibration(), TAG, "Calibration read failed");

	uint8_t status = 0;
	for (int retries = 0; retries < 5; ++retries) {
		ESP_RETURN_ON_ERROR(i2c_device_read(s_bmp388_dev, BMP388_STATUS_REG, &status, 1), TAG, "BMP388 status read");
		if (status & BMP388_STATUS_TEMP_READY) {
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(5));
	}

	if (!(status & BMP388_STATUS_TEMP_READY)) {
		return ESP_ERR_TIMEOUT;
	}

	uint8_t buffer[3] = {0};
	ESP_RETURN_ON_ERROR(i2c_device_read(s_bmp388_dev, BMP388_TEMP_DATA_XLSB | 0x80, buffer, sizeof(buffer)), TAG, "BMP388 temp read");

	uint32_t raw = ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[1] << 8) | buffer[0];
	double partial_data1 = ((double)raw / 16384.0) - ((double)s_bmp_calib.par_t1 / 1024.0);
	double partial_data2 = partial_data1 * ((double)s_bmp_calib.par_t2);
	double partial_data3 = ((double)raw / 131072.0) - ((double)s_bmp_calib.par_t1 / 8192.0);
	double partial_data4 = (partial_data3 * partial_data3) * ((double)s_bmp_calib.par_t3);
	*temperature_c = (float)(partial_data2 + partial_data4);
	return ESP_OK;
}

static void remove_device(i2c_master_dev_handle_t *handle)
{
	if (handle && *handle) {
		i2c_master_bus_rm_device(*handle);
		*handle = NULL;
	}
}

static esp_err_t add_device(const i2c_device_config_t *cfg, i2c_master_dev_handle_t *handle)
{
	if (!s_bus || !cfg || !handle) {
		return ESP_ERR_INVALID_STATE;
	}
	if (*handle) {
		return ESP_OK;
	}
	return i2c_master_bus_add_device(s_bus, cfg, handle);
}

static esp_err_t sensors_update_temperatures(sensor_snapshot_t *working)
{
	esp_err_t overall = ESP_OK;

	if (working->imu_present && s_lsm6dsr_dev) {
		float temp = NAN;
		esp_err_t err = lsm6dsr_read_temperature(&temp);
		if (err == ESP_OK) {
			working->imu_temperature_c = temp;
		} else {
			overall = err;
			working->imu_temperature_c = NAN;
			ESP_LOGW(TAG, "Failed to read LSM6DSR temperature (%s)", esp_err_to_name(err));
		}
	} else {
		working->imu_temperature_c = NAN;
	}

	if (working->mag_present && s_lis2mdl_dev) {
		float temp = NAN;
		esp_err_t err = lis2mdl_read_temperature(&temp);
		if (err == ESP_OK) {
			working->mag_temperature_c = temp;
		} else {
			overall = (overall == ESP_OK) ? err : overall;
			working->mag_temperature_c = NAN;
			ESP_LOGW(TAG, "Failed to read LIS2MDL temperature (%s)", esp_err_to_name(err));
		}
	} else {
		working->mag_temperature_c = NAN;
	}

	if (working->press_present && s_bmp388_dev) {
		float temp = NAN;
		esp_err_t err = bmp388_read_temperature(&temp);
		if (err == ESP_OK) {
			working->press_temperature_c = temp;
		} else {
			overall = (overall == ESP_OK) ? err : overall;
			working->press_temperature_c = NAN;
			ESP_LOGW(TAG, "Failed to read BMP388 temperature (%s)", esp_err_to_name(err));
		}
	} else {
		working->press_temperature_c = NAN;
	}

	return overall;
}

esp_err_t sensors_init(void)
{
	if (!s_state_lock) {
		s_state_lock = xSemaphoreCreateMutex();
		if (!s_state_lock) {
			return ESP_ERR_NO_MEM;
		}
	}

	if (xSemaphoreTake(s_state_lock, portMAX_DELAY) == pdTRUE) {
		s_snapshot = snapshot_defaults();
		xSemaphoreGive(s_state_lock);
	}

	s_bmp_calib.loaded = false;

	ESP_RETURN_ON_ERROR(board_i2c_init(), TAG, "Board I2C init failed");
	s_bus = board_i2c_get_bus();
	if (!s_bus) {
		return ESP_ERR_INVALID_STATE;
	}
	vTaskDelay(pdMS_TO_TICKS(10));

	bool imu_present = false;
	bool mag_present = false;
	bool press_present = false;

	if (add_device(&s_lsm6dsr_cfg, &s_lsm6dsr_dev) == ESP_OK) {
		if (detect_device(s_lsm6dsr_dev, LSM6DSR_WHO_AM_I_REG, LSM6DSR_ID_PRIMARY) && lsm6dsr_configure() == ESP_OK) {
			imu_present = true;
		} else {
			remove_device(&s_lsm6dsr_dev);
		}
	}

	if (add_device(&s_lis2mdl_cfg, &s_lis2mdl_dev) == ESP_OK) {
		if (detect_device(s_lis2mdl_dev, LIS2MDL_WHO_AM_I_REG, LIS2MDL_ID) && lis2mdl_configure() == ESP_OK) {
			mag_present = true;
		} else {
			remove_device(&s_lis2mdl_dev);
		}
	}

	if (add_device(&s_bmp388_cfg, &s_bmp388_dev) == ESP_OK) {
		if (detect_device(s_bmp388_dev, BMP388_CHIP_ID_REG, BMP388_CHIP_ID) && bmp388_configure() == ESP_OK) {
			press_present = true;
		} else {
			remove_device(&s_bmp388_dev);
			s_bmp_calib.loaded = false;
		}
	}

	sensor_snapshot_t working = snapshot_defaults();
	working.imu_present = imu_present;
	working.mag_present = mag_present;
	working.press_present = press_present;

	esp_err_t initial_read_err = sensors_update_temperatures(&working);
	if (initial_read_err != ESP_OK) {
		ESP_LOGW(TAG, "Initial sensor read had issues (%s)", esp_err_to_name(initial_read_err));
	}

	if (xSemaphoreTake(s_state_lock, portMAX_DELAY) == pdTRUE) {
		s_snapshot = working;
		xSemaphoreGive(s_state_lock);
	}

	ESP_LOGI(TAG, "Sensors initialized: IMU=%s, MAG=%s, PRESS=%s",
			 imu_present ? "OK" : "NOT FOUND",
			 mag_present ? "OK" : "NOT FOUND",
			 press_present ? "OK" : "NOT FOUND");

	s_initialized = true;
	return ESP_OK;
}

esp_err_t sensors_update(void)
{
	if (!s_initialized) {
		return ESP_ERR_INVALID_STATE;
	}

	sensor_snapshot_t working;
	if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
		return ESP_ERR_TIMEOUT;
	}
	working = s_snapshot;
	xSemaphoreGive(s_state_lock);

	esp_err_t err = sensors_update_temperatures(&working);

	if (xSemaphoreTake(s_state_lock, portMAX_DELAY) == pdTRUE) {
		s_snapshot.imu_temperature_c = working.imu_temperature_c;
		s_snapshot.mag_temperature_c = working.mag_temperature_c;
		s_snapshot.press_temperature_c = working.press_temperature_c;
		xSemaphoreGive(s_state_lock);
	}

	return err;
}

void sensors_read_snapshot(sensor_snapshot_t *snapshot)
{
	if (!snapshot) {
		return;
	}

	if (!s_state_lock || xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
		*snapshot = snapshot_defaults();
		return;
	}

	*snapshot = s_snapshot;
	xSemaphoreGive(s_state_lock);
}
