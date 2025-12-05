// app_check.h
// 说明：统一错误处理宏定义

#pragma once

#include "esp_err.h"
#include "esp_log.h"

// 宏说明：执行 expr，若失败则打印错误并返回
#define CHECK_ESP_RETURN(expr)                                   \
    do {                                                         \
        esp_err_t _err = (expr);                                 \
        if (_err != ESP_OK) {                                    \
            ESP_LOGE("CHECK", "ESP err 0x%x at %s:%d",           \
                     _err, __FILE__, __LINE__);                  \
            return _err;                                         \
        }                                                        \
    } while (0)

// 宏说明：执行 expr，若失败则打印错误并跳转到 label
#define CHECK_ESP_GOTO(expr, label)                              \
    do {                                                         \
        esp_err_t _err = (expr);                                 \
        if (_err != ESP_OK) {                                    \
            ESP_LOGE("CHECK", "ESP err 0x%x at %s:%d",           \
                     _err, __FILE__, __LINE__);                  \
            goto label;                                          \
        }                                                        \
    } while (0)

// 宏说明：检查指针是否为空，为空则返回 ESP_ERR_INVALID_ARG
#define CHECK_NULL_RETURN(ptr)                                   \
    do {                                                         \
        if ((ptr) == NULL) {                                     \
            ESP_LOGE("CHECK", "NULL ptr at %s:%d",               \
                     __FILE__, __LINE__);                        \
            return ESP_ERR_INVALID_ARG;                          \
        }                                                        \
    } while (0)
