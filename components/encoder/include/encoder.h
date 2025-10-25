#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ENCODER_BUTTON_NONE = 0,
    ENCODER_BUTTON_SHORT,
    ENCODER_BUTTON_MEDIUM,
    ENCODER_BUTTON_LONG,
} encoder_button_event_t;

esp_err_t encoder_init(void);
int32_t encoder_get_count(void);
void encoder_reset_count(void);
encoder_button_event_t encoder_get_last_button_event(void);

#ifdef __cplusplus
}
#endif
