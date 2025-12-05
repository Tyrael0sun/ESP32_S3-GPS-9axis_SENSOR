// log_system.h
// 说明：日志与心跳任务接口

#pragma once

#include "esp_err.h"

esp_err_t log_system_start(void);
esp_err_t heartbeat_task_start(void);

void log_system_enqueue(const char *fmt, ...);
uint32_t log_system_get_drop_count(void);
uint32_t log_system_get_heartbeat_miss(void);
