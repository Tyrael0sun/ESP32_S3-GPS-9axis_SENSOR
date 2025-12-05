// bike_manager.h
// 说明：Bike 模式业务逻辑接口

#pragma once

#include "esp_err.h"

esp_err_t bike_manager_start(void);
void bike_manager_handle_lap(void);
