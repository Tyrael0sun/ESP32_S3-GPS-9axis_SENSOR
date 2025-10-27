#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <hal/gpio_types.h>
#include <driver/i2c_master.h>

#define BOARD_I2C_MASTER_NUM     I2C_NUM_0
#define BOARD_I2C_SCL_IO         GPIO_NUM_39
#define BOARD_I2C_SDA_IO         GPIO_NUM_40
#define BOARD_I2C_FREQ_HZ        400000

typedef struct {
    float battery_voltage;
    float battery_adc_voltage;
    bool charging;
} board_state_t;

esp_err_t board_i2c_init(void);
esp_err_t board_init(void);
void board_read_state(board_state_t *state);
i2c_master_bus_handle_t board_i2c_get_bus(void);
