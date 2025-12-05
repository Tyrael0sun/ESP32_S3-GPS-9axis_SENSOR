// input_driver.h
// 说明：按键与旋转编码器输入驱动

#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    INPUT_EVENT_BTN_CLICK = 0,
    INPUT_EVENT_BTN_DOUBLE,
    INPUT_EVENT_BTN_MID,
    INPUT_EVENT_BTN_LONG,
    INPUT_EVENT_BTN_ULTRA,
    INPUT_EVENT_ENC_LEFT,
    INPUT_EVENT_ENC_RIGHT,
} input_event_type_t;

typedef struct {
    input_event_type_t type;
    uint32_t timestamp_ms;
} input_event_t;

esp_err_t input_driver_start(void);
QueueHandle_t input_event_queue_get(void);
