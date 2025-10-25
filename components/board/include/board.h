#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    float battery_voltage;
    bool charging;
} board_state_t;

esp_err_t board_init(void);
void board_read_state(board_state_t *state);
