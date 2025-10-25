#include "lcd_spi.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"

#ifdef CONFIG_LCD_SPI_CREATE_PANEL_IO
#include "esp_lcd_panel_io.h"
#endif

#define LCD_SPI_HOST            SPI3_HOST
#define LCD_PIN_SCLK            5
#define LCD_PIN_MOSI            8
#define LCD_PIN_CS              7
#define LCD_PIN_DC              6
#define LCD_PIN_RST             4
#define LCD_PIN_BACKLIGHT       9

#define LCD_LEDC_TIMER          LEDC_TIMER_0
#define LCD_LEDC_CHANNEL        LEDC_CHANNEL_0
#define LCD_LEDC_SPEED_MODE     LEDC_LOW_SPEED_MODE
#define LCD_LEDC_FREQUENCY_HZ   20000
#define LCD_LEDC_DUTY_RES       LEDC_TIMER_13_BIT

static const char *TAG = "LCD";

static bool s_spi_bus_ready;
static bool s_backlight_ready;

static esp_err_t lcd_spi_configure_bus(void)
{
    if (s_spi_bus_ready) {
        return ESP_OK;
    }

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = LCD_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = LCD_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    esp_err_t err = spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err == ESP_ERR_INVALID_STATE) {
        s_spi_bus_ready = true;
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "SPI bus init failed");

    gpio_config_t dc_cfg = {
        .pin_bit_mask = BIT64(LCD_PIN_DC),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&dc_cfg), TAG, "DC pin config failed");

    if (LCD_PIN_RST >= 0) {
        gpio_config_t rst_cfg = dc_cfg;
        rst_cfg.pin_bit_mask = BIT64(LCD_PIN_RST);
        ESP_RETURN_ON_ERROR(gpio_config(&rst_cfg), TAG, "RST pin config failed");
        gpio_set_level(LCD_PIN_RST, 1);
    }

    if (LCD_PIN_CS >= 0) {
        gpio_config_t cs_cfg = dc_cfg;
        cs_cfg.pin_bit_mask = BIT64(LCD_PIN_CS);
        ESP_RETURN_ON_ERROR(gpio_config(&cs_cfg), TAG, "CS pin config failed");
        gpio_set_level(LCD_PIN_CS, 1);
    }

    s_spi_bus_ready = true;
    return ESP_OK;
}

static esp_err_t lcd_spi_configure_backlight(void)
{
    if (s_backlight_ready) {
        return ESP_OK;
    }

    ledc_timer_config_t timer_cfg = {
        .speed_mode = LCD_LEDC_SPEED_MODE,
        .timer_num = LCD_LEDC_TIMER,
        .freq_hz = LCD_LEDC_FREQUENCY_HZ,
        .duty_resolution = LCD_LEDC_DUTY_RES,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "LEDC timer config failed");

    ledc_channel_config_t ch_cfg = {
        .gpio_num = LCD_PIN_BACKLIGHT,
        .speed_mode = LCD_LEDC_SPEED_MODE,
        .channel = LCD_LEDC_CHANNEL,
        .timer_sel = LCD_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "LEDC channel config failed");

    s_backlight_ready = true;
    return lcd_spi_set_backlight_percent(80);
}

esp_err_t lcd_spi_init(void)
{
    ESP_RETURN_ON_ERROR(lcd_spi_configure_bus(), TAG, "Bus setup failed");
    ESP_RETURN_ON_ERROR(lcd_spi_configure_backlight(), TAG, "Backlight setup failed");
    ESP_LOGI(TAG, "SPI LCD bus ready");
    return ESP_OK;
}

esp_err_t lcd_spi_set_backlight_percent(uint8_t percent)
{
    if (!s_backlight_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    if (percent > 100) {
        percent = 100;
    }

    uint32_t max_duty = (1U << LCD_LEDC_DUTY_RES) - 1U;
    uint32_t duty = (max_duty * percent) / 100U;

    ESP_RETURN_ON_ERROR(ledc_set_duty(LCD_LEDC_SPEED_MODE, LCD_LEDC_CHANNEL, duty), TAG, "set duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LCD_LEDC_SPEED_MODE, LCD_LEDC_CHANNEL), TAG, "update duty failed");
    return ESP_OK;
}

#ifdef CONFIG_LCD_SPI_CREATE_PANEL_IO
esp_err_t lcd_spi_create_panel_io(const esp_lcd_panel_io_spi_config_t *io_config,
                                  esp_lcd_panel_io_handle_t *out_handle)
{
    if (!io_config || !out_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(lcd_spi_configure_bus(), TAG, "Bus setup failed");
    return esp_lcd_new_panel_io_spi(LCD_SPI_HOST, io_config, out_handle);
}
#endif
