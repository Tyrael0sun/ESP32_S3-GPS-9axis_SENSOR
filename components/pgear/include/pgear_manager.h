// pgear_manager.h
// 说明：P-GEAR 状态机接口

#pragma once

#include "esp_err.h"

typedef enum {
    PGEAR_EVENT_GNSS_UPDATE = 0,
    PGEAR_EVENT_BTN_CLEAR,
    PGEAR_EVENT_BTN_RETEST,
} pgear_event_type_t;

typedef struct {
    pgear_event_type_t type;
} pgear_event_t;

esp_err_t pgear_manager_start(void);
void pgear_manager_post_event(const pgear_event_t *ev);
