// baro_bmp388.c
// 说明：BMP388 气压计驱动实现

#include "baro_bmp388.h"
#include "i2c_bus.h"
#include "macro_def.h"
#include "app_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define BMP388_REG_CHIP_ID        0x00
#define BMP388_REG_ERR_REG        0x02
#define BMP388_REG_STATUS         0x03
#define BMP388_REG_DATA           0x04
#define BMP388_REG_CMD            0x7E
#define BMP388_REG_PWR_CTRL       0x1B
#define BMP388_REG_OSR            0x1C
#define BMP388_REG_ODR            0x1D
#define BMP388_REG_CONFIG         0x1F
#define BMP388_REG_CALIB          0x31

static const char *TAG = "baro_bmp388";

static i2c_master_dev_handle_t s_baro_dev = NULL;
static struct {
    float t1, t2, t3;
    float p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
} s_calib;
static float s_t_lin = 0.0f;

static esp_err_t baro_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    CHECK_ESP_RETURN(i2c_master_transmit_receive(s_baro_dev,
                                                 &reg, 1,
                                                 buf, len,
                                                 1000));
    return ESP_OK;
}

static esp_err_t baro_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tmp[2] = {reg, val};
    CHECK_ESP_RETURN(i2c_master_transmit(s_baro_dev, tmp, sizeof(tmp), 1000));
    return ESP_OK;
}

static void parse_calib(const uint8_t *buf)
{
    uint16_t T1 = (uint16_t)(buf[1] << 8 | buf[0]);
    uint16_t T2 = (uint16_t)(buf[3] << 8 | buf[2]);
    int8_t   T3 = (int8_t)buf[4];

    int16_t P1 = (int16_t)(buf[6] << 8 | buf[5]);
    int16_t P2 = (int16_t)(buf[8] << 8 | buf[7]);
    int8_t  P3 = (int8_t)buf[9];
    int8_t  P4 = (int8_t)buf[10];
    uint16_t P5 = (uint16_t)(buf[12] << 8 | buf[11]);
    uint16_t P6 = (uint16_t)(buf[14] << 8 | buf[13]);
    int8_t  P7 = (int8_t)buf[15];
    int8_t  P8 = (int8_t)buf[16];
    int16_t P9 = (int16_t)(buf[18] << 8 | buf[17]);
    int8_t  P10 = (int8_t)buf[19];
    int8_t  P11 = (int8_t)buf[20];

    s_calib.t1 = T1 / 0.00390625f;
    s_calib.t2 = T2 / 1073741824.0f;
    s_calib.t3 = T3 / 281474976710656.0f;

    s_calib.p1 = (P1 - 16384) / 1048576.0f;
    s_calib.p2 = (P2 - 16384) / 536870912.0f;
    s_calib.p3 = P3 / 4294967296.0f;
    s_calib.p4 = P4 / 137438953472.0f;
    s_calib.p5 = P5 / 0.125f;
    s_calib.p6 = P6 / 64.0f;
    s_calib.p7 = P7 / 256.0f;
    s_calib.p8 = P8 / 32768.0f;
    s_calib.p9 = P9 / 281474976710656.0f;
    s_calib.p10 = P10 / 281474976710656.0f;
    s_calib.p11 = P11 / 36893488147419103232.0f;
}

esp_err_t baro_bmp388_init(void)
{
    CHECK_ESP_RETURN(i2c_bus_init());

    i2c_master_bus_handle_t bus = i2c_bus_get();
    i2c_device_config_t dev_cfg = {
        .device_address = BMP388_I2C_ADDR,
        .scl_speed_hz   = 400000,
    };

    CHECK_ESP_RETURN(i2c_master_bus_add_device(bus, &dev_cfg, &s_baro_dev));

    uint8_t chip_id = 0;
    CHECK_ESP_RETURN(baro_read_regs(BMP388_REG_CHIP_ID, &chip_id, 1));
    if (chip_id != 0x50) {
        ESP_LOGE(TAG, "invalid chip id 0x%02X", chip_id);
        return ESP_FAIL;
    }

    // 软复位
    CHECK_ESP_RETURN(baro_write_reg(BMP388_REG_CMD, 0xB6));
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t calib_buf[21] = {0};
    CHECK_ESP_RETURN(baro_read_regs(BMP388_REG_CALIB, calib_buf, sizeof(calib_buf)));
    parse_calib(calib_buf);

    // 配置 OSR 和 ODR
    CHECK_ESP_RETURN(baro_write_reg(BMP388_REG_OSR, 0x77));
    CHECK_ESP_RETURN(baro_write_reg(BMP388_REG_ODR, 0x03));
    CHECK_ESP_RETURN(baro_write_reg(BMP388_REG_CONFIG, 0x00));

    // 使能压力和温度，正常模式
    CHECK_ESP_RETURN(baro_write_reg(BMP388_REG_PWR_CTRL, 0x30));

    ESP_LOGI(TAG, "bmp388 init ok");
    return ESP_OK;
}

static float compensate_temp(int32_t adc_t)
{
    float partial = (adc_t / 16384.0f - s_calib.t1);
    s_t_lin = partial * s_calib.t2 + partial * partial * s_calib.t3;
    return s_t_lin;
}

static float compensate_press(int32_t adc_p)
{
    float partial_data1 = s_t_lin * s_t_lin;
    float partial_data2 = partial_data1 * s_t_lin;
    float partial_out1 = s_calib.p6 * s_t_lin;
    float partial_out2 = s_calib.p7 * partial_data1;
    float partial_out3 = s_calib.p8 * partial_data2;

    float partial_data3 = (float)adc_p / 16777216.0f - s_calib.p1;
    float partial_data4 = s_calib.p2 * partial_data3;
    float partial_data5 = partial_data3 * partial_data3;
    float partial_data6 = s_calib.p9 * partial_data5;
    float partial_data7 = partial_data5 * partial_data3;

    float comp_press = s_calib.p5 + partial_out1 + partial_out2 + partial_out3;
    comp_press += partial_data4;
    comp_press += s_calib.p3 * partial_data5 + s_calib.p4 * partial_data7;
    comp_press += partial_data6 * s_t_lin + s_calib.p10 * partial_data5 * s_t_lin;
    comp_press += s_calib.p11 * partial_data7 * s_t_lin;

    return comp_press;
}

esp_err_t baro_bmp388_read(float *pressure_pa, float *temp_c)
{
    CHECK_NULL_RETURN(pressure_pa);
    CHECK_NULL_RETURN(temp_c);

    uint8_t buf[6] = {0};
    CHECK_ESP_RETURN(baro_read_regs(BMP388_REG_DATA, buf, sizeof(buf)));

    int32_t adc_p = (int32_t)(buf[2] << 16 | buf[1] << 8 | buf[0]);
    int32_t adc_t = (int32_t)(buf[5] << 16 | buf[4] << 8 | buf[3]);

    float t = compensate_temp(adc_t);
    float p = compensate_press(adc_p);

    *temp_c = t;
    *pressure_pa = p;

    return ESP_OK;
}
