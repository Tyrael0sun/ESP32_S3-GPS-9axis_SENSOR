// macro_def.h
// 说明：通用宏定义

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// GPIO 定义
#define GPIO_DEBUG_TX       (43)
#define GPIO_DEBUG_RX       (44)
#define GPIO_GNSS_TX        (17)
#define GPIO_GNSS_RX        (18)
#define GPIO_GPS_LDO_EN     (14)

#define GPIO_I2C_SCL        (39)
#define GPIO_I2C_SDA        (40)

#define GPIO_LCD_SCK        (5)
#define GPIO_LCD_MOSI       (8)
#define GPIO_LCD_CS         (7)
#define GPIO_LCD_DC         (6)
#define GPIO_LCD_RST        (4)
#define GPIO_LCD_BL         (9)

#define GPIO_SD_CLK         (36)
#define GPIO_SD_CMD         (35)
#define GPIO_SD_D0          (37)
#define GPIO_SD_D1          (38)
#define GPIO_SD_D2          (34)
#define GPIO_SD_D3          (33)

#define GPIO_ENC_A          (1)
#define GPIO_ENC_B          (3)
#define GPIO_KEY_MAIN       (2)

#define GPIO_BAT_ADC        (12)
#define GPIO_CHRG_STATUS    (21)

// I2C 地址定义
#define LSM6DSR_I2C_ADDR    (0x6A)
#define LIS2MDL_I2C_ADDR    (0x1E)
#define BMP388_I2C_ADDR     (0x76)

// 数学常数
#ifndef M_PI
#define M_PI                (3.14159265358979323846)
#endif

#define DEG_TO_RAD(deg)     ((deg) * M_PI / 180.0f)
#define RAD_TO_DEG(rad)     ((rad) * 180.0f / M_PI)

// 最小/最大值宏
#define MIN(a, b)           (((a) < (b)) ? (a) : (b))
#define MAX(a, b)           (((a) > (b)) ? (a) : (b))
#define CLAMP(val, min, max) (MIN(MAX((val), (min)), (max)))

// 数组元素个数
#define ARRAY_SIZE(arr)     (sizeof(arr) / sizeof((arr)[0]))
