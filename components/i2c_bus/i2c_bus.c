// i2c_bus.c
// 说明：I2C 主机总线初始化与获取接口

#include "i2c_bus.h"
#include "macro_def.h"
#include "app_check.h"
#include "esp_log.h"

static const char *TAG = "i2c_bus";

static i2c_master_bus_handle_t s_i2c_bus = NULL;

// 函数说明：初始化 I2C 主机总线，仅初始化一次
esp_err_t i2c_bus_init(void)
{
    if (s_i2c_bus != NULL) {
        ESP_LOGW(TAG, "i2c bus already initialized");
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = I2C_NUM_0,
        .scl_io_num = GPIO_I2C_SCL,
        .sda_io_num = GPIO_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    CHECK_ESP_RETURN(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));

    ESP_LOGI(TAG, "i2c bus init ok (scl=%d, sda=%d)", 
             GPIO_I2C_SCL, GPIO_I2C_SDA);
    return ESP_OK;
}

// 函数说明：获取 I2C 总线句柄
i2c_master_bus_handle_t i2c_bus_get(void)
{
    return s_i2c_bus;
}
