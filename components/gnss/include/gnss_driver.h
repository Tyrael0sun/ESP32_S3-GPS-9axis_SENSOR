// gnss_driver.h
// 说明：NEO-M8N GNSS 驱动接口

#pragma once

#include "esp_err.h"
#include <stdbool.h>

esp_err_t gnss_driver_init(void);
esp_err_t gnss_driver_request_fast_rate(bool enable);
