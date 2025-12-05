// i2c_bus.h
// 说明：I2C 总线对外接口（ESP-IDF v6 新驱动）

#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

// 函数说明：初始化 I2C 主机总线
esp_err_t i2c_bus_init(void);

// 函数说明：获取 I2C 主机总线句柄
i2c_master_bus_handle_t i2c_bus_get(void);
