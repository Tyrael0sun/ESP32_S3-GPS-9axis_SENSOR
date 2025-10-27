#include "lcd_spi.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "lvgl.h"

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

#define LCD_WIDTH               240
#define LCD_HEIGHT              240

#define LVGL_BUFFER_LINES       40
#define LVGL_TICK_PERIOD_US     5000
#define LVGL_TASK_DELAY_MS      10

#define LOG_BUFFER_SIZE         4096
#define BACKLIGHT_DEFAULT_PERCENT 80

static const char *TAG = "LCD";

static bool s_spi_bus_ready;
static bool s_backlight_ready;
static bool s_panel_ready;
static bool s_lvgl_ready;

static esp_lcd_panel_io_handle_t s_panel_io;
static esp_lcd_panel_handle_t s_panel_handle;

static lv_disp_draw_buf_t s_lvgl_draw_buf;
static lv_disp_drv_t s_lvgl_disp_drv;
static lv_disp_t *s_lvgl_display;
static lv_color_t *s_lvgl_buf1;
static lv_color_t *s_lvgl_buf2;
static TaskHandle_t s_lvgl_task_handle;
static esp_timer_handle_t s_lvgl_tick_timer;

static lv_obj_t *s_log_textarea;

static vprintf_like_t s_prev_logger;
static bool s_log_sink_installed;

static portMUX_TYPE s_log_lock = portMUX_INITIALIZER_UNLOCKED;
static char s_log_buffer[LOG_BUFFER_SIZE];
static size_t s_log_buffer_len;
static bool s_log_dirty;
static char s_log_render_cache[LOG_BUFFER_SIZE];

static void lcd_log_flush_to_lvgl(void);
static void lcd_log_append(const char *text);
static void lcd_lvgl_tick_cb(void *arg);
static void lcd_lvgl_task(void *arg);
static void lcd_lvgl_free_buffers(void);

static void lcd_lvgl_free_buffers(void)
{
    free(s_lvgl_buf1);
    free(s_lvgl_buf2);
    s_lvgl_buf1 = NULL;
    s_lvgl_buf2 = NULL;
}

static void lcd_log_flush_to_lvgl(void)
{
    if (!s_lvgl_ready || !s_log_textarea) {
        return;
    }

    bool dirty = false;
    size_t copy_len = 0;

    portENTER_CRITICAL(&s_log_lock);
    if (s_log_dirty) {
        s_log_dirty = false;
        copy_len = s_log_buffer_len;
        if (copy_len >= sizeof(s_log_render_cache)) {
            copy_len = sizeof(s_log_render_cache) - 1;
        }
        memcpy(s_log_render_cache, s_log_buffer, copy_len);
        s_log_render_cache[copy_len] = '\0';
        dirty = true;
    }
    portEXIT_CRITICAL(&s_log_lock);

    if (dirty) {
        lv_textarea_set_text(s_log_textarea, s_log_render_cache);
        lv_textarea_set_cursor_pos(s_log_textarea, LV_TEXTAREA_CURSOR_LAST);
    }
}

static void lcd_log_append(const char *text)
{
    if (!text) {
        return;
    }

    size_t append_len = 0;
    for (const char *p = text; *p; ++p) {
        if (*p != '\r') {
            append_len++;
        }
    }
    if (append_len == 0) {
        return;
    }

    portENTER_CRITICAL(&s_log_lock);
    if (append_len >= LOG_BUFFER_SIZE) {
        size_t skip = append_len - (LOG_BUFFER_SIZE - 1);
        size_t filtered = 0;
        s_log_buffer_len = 0;
        for (const char *p = text; *p; ++p) {
            if (*p == '\r') {
                continue;
            }
            if (filtered++ < skip) {
                continue;
            }
            if (s_log_buffer_len >= LOG_BUFFER_SIZE - 1) {
                break;
            }
            s_log_buffer[s_log_buffer_len++] = *p;
        }
    } else {
        size_t overflow = 0;
        if (s_log_buffer_len + append_len + 1 > LOG_BUFFER_SIZE) {
            overflow = s_log_buffer_len + append_len + 1 - LOG_BUFFER_SIZE;
            if (overflow > s_log_buffer_len) {
                overflow = s_log_buffer_len;
            }
        }
        if (overflow > 0) {
            memmove(s_log_buffer, s_log_buffer + overflow, s_log_buffer_len - overflow);
            s_log_buffer_len -= overflow;
        }
        for (const char *p = text; *p; ++p) {
            if (*p == '\r') {
                continue;
            }
            s_log_buffer[s_log_buffer_len++] = *p;
        }
    }
    s_log_buffer[s_log_buffer_len] = '\0';
    s_log_dirty = true;
    portEXIT_CRITICAL(&s_log_lock);
}

static void lcd_lvgl_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(LVGL_TICK_PERIOD_US / 1000);
}

static void lcd_lvgl_task(void *arg)
{
    (void)arg;
    while (true) {
        lcd_log_flush_to_lvgl();
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(LVGL_TASK_DELAY_MS));
    }
}

static void lcd_lvgl_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_map)
{
    if (!s_panel_handle || !area || !color_map) {
        lv_disp_flush_ready(disp_drv);
        return;
    }

    int32_t x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t x2 = area->x2 >= LCD_WIDTH ? LCD_WIDTH - 1 : area->x2;
    int32_t y2 = area->y2 >= LCD_HEIGHT ? LCD_HEIGHT - 1 : area->y2;

    esp_lcd_panel_draw_bitmap(s_panel_handle, x1, y1, x2 + 1, y2 + 1, color_map);
    lv_disp_flush_ready(disp_drv);
}

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
        .max_transfer_sz = LCD_WIDTH * LVGL_BUFFER_LINES * sizeof(lv_color_t),
    };

    esp_err_t err = spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err == ESP_ERR_INVALID_STATE) {
        s_spi_bus_ready = true;
    } else {
        ESP_RETURN_ON_ERROR(err, TAG, "SPI bus init failed");
        s_spi_bus_ready = true;
    }

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

    return ESP_OK;
}

static esp_err_t lcd_spi_configure_panel(void)
{
    if (s_panel_ready) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(lcd_spi_configure_bus(), TAG, "Bus setup failed");

    if (!s_panel_io) {
        esp_lcd_panel_io_spi_config_t io_config = {
            .dc_gpio_num = LCD_PIN_DC,
            .cs_gpio_num = LCD_PIN_CS,
            .pclk_hz = 40 * 1000 * 1000,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .spi_mode = 0,
            .trans_queue_depth = 10,
        };
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &io_config, &s_panel_io), TAG,
                            "Panel IO create failed");
    }

    if (!s_panel_handle) {
        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = LCD_PIN_RST,
            .color_space = ESP_LCD_COLOR_SPACE_RGB,
            .bits_per_pixel = 16,
        };
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(s_panel_io, &panel_config, &s_panel_handle), TAG,
                            "ST7789 panel create failed");
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel_handle), TAG, "Panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel_handle), TAG, "Panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel_handle, true), TAG, "Panel display on failed");

    s_panel_ready = true;
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
    return lcd_spi_set_backlight_percent(BACKLIGHT_DEFAULT_PERCENT);
}

static esp_err_t lcd_lvgl_init(void)
{
    if (s_lvgl_ready) {
        return ESP_OK;
    }

    lv_init();

    size_t buf_pixels = LCD_WIDTH * LVGL_BUFFER_LINES;
    size_t buf_size = buf_pixels * sizeof(lv_color_t);

    s_lvgl_buf1 = heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_lvgl_buf2 = heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_lvgl_buf1 || !s_lvgl_buf2) {
        lcd_lvgl_free_buffers();
        return ESP_ERR_NO_MEM;
    }

    lv_disp_draw_buf_init(&s_lvgl_draw_buf, s_lvgl_buf1, s_lvgl_buf2, buf_pixels);

    lv_disp_drv_init(&s_lvgl_disp_drv);
    s_lvgl_disp_drv.hor_res = LCD_WIDTH;
    s_lvgl_disp_drv.ver_res = LCD_HEIGHT;
    s_lvgl_disp_drv.flush_cb = lcd_lvgl_flush;
    s_lvgl_disp_drv.draw_buf = &s_lvgl_draw_buf;

    s_lvgl_display = lv_disp_drv_register(&s_lvgl_disp_drv);
    if (!s_lvgl_display) {
        lcd_lvgl_free_buffers();
        return ESP_FAIL;
    }

    s_log_textarea = lv_textarea_create(lv_scr_act());
    if (!s_log_textarea) {
        return ESP_FAIL;
    }
    lv_obj_set_size(s_log_textarea, LCD_WIDTH, LCD_HEIGHT);
    lv_textarea_set_max_length(s_log_textarea, LOG_BUFFER_SIZE - 1);
    lv_textarea_set_cursor_click_pos(s_log_textarea, false);
    lv_textarea_set_text(s_log_textarea, "");
    lv_obj_set_scrollbar_mode(s_log_textarea, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_pad_all(s_log_textarea, 4, LV_PART_MAIN);
    lv_obj_set_style_text_align(s_log_textarea, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);

    const esp_timer_create_args_t tick_timer_args = {
        .callback = lcd_lvgl_tick_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lvgl_tick",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&tick_timer_args, &s_lvgl_tick_timer), TAG, "LVGL timer create failed");
    esp_err_t err = esp_timer_start_periodic(s_lvgl_tick_timer, LVGL_TICK_PERIOD_US);
    if (err != ESP_OK) {
        esp_timer_delete(s_lvgl_tick_timer);
        s_lvgl_tick_timer = NULL;
        return err;
    }

    BaseType_t created = xTaskCreatePinnedToCore(lcd_lvgl_task, "lvgl", 4096, NULL, 5, &s_lvgl_task_handle, 0);
    if (created != pdPASS) {
        esp_timer_stop(s_lvgl_tick_timer);
        esp_timer_delete(s_lvgl_tick_timer);
        s_lvgl_tick_timer = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_lvgl_ready = true;
    lcd_log_append("LCD log ready\n");
    return ESP_OK;
}

static int lcd_spi_log_vprintf(const char *fmt, va_list args)
{
    va_list copy;
    va_copy(copy, args);
    int needed = vsnprintf(NULL, 0, fmt, copy);
    va_end(copy);

    char stack_buf[256];
    char *buffer = stack_buf;
    size_t capacity = sizeof(stack_buf);

    if (needed >= 0 && (size_t)(needed + 1) > capacity) {
        buffer = malloc((size_t)(needed + 1));
        if (!buffer) {
            return s_prev_logger ? s_prev_logger(fmt, args) : vprintf(fmt, args);
        }
        capacity = (size_t)(needed + 1);
    }

    va_list copy2;
    va_copy(copy2, args);
    vsnprintf(buffer, capacity, fmt, copy2);
    va_end(copy2);

    lcd_log_append(buffer);

    int result = s_prev_logger ? s_prev_logger(fmt, args) : vprintf(fmt, args);

    if (buffer != stack_buf) {
        free(buffer);
    }

    return result;
}

esp_err_t lcd_spi_init(void)
{
    ESP_RETURN_ON_ERROR(lcd_spi_configure_backlight(), TAG, "Backlight setup failed");
    ESP_RETURN_ON_ERROR(lcd_spi_configure_panel(), TAG, "Panel setup failed");
    ESP_RETURN_ON_ERROR(lcd_lvgl_init(), TAG, "LVGL init failed");
    ESP_LOGI(TAG, "SPI LCD ready (ST7789 + LVGL)");
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

esp_err_t lcd_spi_install_log_sink(void)
{
    if (!s_lvgl_ready || !s_log_textarea) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_log_sink_installed) {
        return ESP_OK;
    }

    s_prev_logger = esp_log_set_vprintf(lcd_spi_log_vprintf);
    s_log_sink_installed = true;
    lcd_log_append("[LCD] Log mirror ready\n");
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

    if (!s_panel_io) {
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, io_config, &s_panel_io), TAG,
                            "Panel IO create failed");
    }

    *out_handle = s_panel_io;
    return ESP_OK;
}
#endif

#if 0
/* Legacy bitmap renderer retained for reference while LVGL integration stabilises. */

static esp_err_t lcd_spi_configure_panel(void)
{
    if (s_panel_ready) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(lcd_spi_configure_bus(), TAG, "Bus setup failed");

    if (!s_panel_io) {
        esp_lcd_panel_io_spi_config_t io_config = {
            .dc_gpio_num = LCD_PIN_DC,
            .cs_gpio_num = LCD_PIN_CS,
            .pclk_hz = 40 * 1000 * 1000,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .spi_mode = 0,
            .trans_queue_depth = 10,
        };
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &io_config, &s_panel_io), TAG,
                            "Panel IO create failed");
    }

    if (!s_panel_handle) {
        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = LCD_PIN_RST,
            .color_space = ESP_LCD_COLOR_SPACE_RGB,
            .bits_per_pixel = 16,
        };
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(s_panel_io, &panel_config, &s_panel_handle), TAG,
                            "ST7789 panel create failed");
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel_handle), TAG, "Panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel_handle), TAG, "Panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel_handle, true), TAG, "Panel display on failed");

    s_panel_ready = true;
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

static void lcd_lvgl_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_map)
{
    if (!s_panel_handle || !area || !color_map) {
        lv_disp_flush_ready(disp_drv);
        return;
    }

    int32_t x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t x2 = area->x2 >= LCD_WIDTH ? (LCD_WIDTH - 1) : area->x2;
    int32_t y2 = area->y2 >= LCD_HEIGHT ? (LCD_HEIGHT - 1) : area->y2;

    esp_lcd_panel_draw_bitmap(s_panel_handle, x1, y1, x2 + 1, y2 + 1, color_map);
    lv_disp_flush_ready(disp_drv);
}

static esp_err_t lcd_lvgl_init(void)
{
    if (s_lvgl_ready) {
        return ESP_OK;
    }

    lv_init();

    size_t buf_pixels = LCD_WIDTH * LVGL_BUFFER_LINES;
    size_t buf_size = buf_pixels * sizeof(lv_color_t);

    s_lvgl_buf1 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_lvgl_buf2 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_lvgl_buf1 || !s_lvgl_buf2) {
        free(s_lvgl_buf1);
        free(s_lvgl_buf2);
        return ESP_ERR_NO_MEM;
    }

    lv_disp_draw_buf_init(&s_lvgl_draw_buf, s_lvgl_buf1, s_lvgl_buf2, buf_pixels);

    lv_disp_drv_init(&s_lvgl_disp_drv);
    s_lvgl_disp_drv.hor_res = LCD_WIDTH;
    s_lvgl_disp_drv.ver_res = LCD_HEIGHT;
    s_lvgl_disp_drv.flush_cb = lcd_lvgl_flush;
    s_lvgl_disp_drv.draw_buf = &s_lvgl_draw_buf;
    s_lvgl_display = lv_disp_drv_register(&s_lvgl_disp_drv);
    if (!s_lvgl_display) {
        return ESP_FAIL;
    }

    s_log_label = lv_label_create(lv_scr_act());
    lv_label_set_long_mode(s_log_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_size(s_log_label, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_set_style_text_align(s_log_label, LV_TEXT_ALIGN_LEFT, 0);
    lv_label_set_text(s_log_label, "LCD log ready\n");

    const esp_timer_create_args_t tick_timer_args = {
        .callback = lcd_lvgl_tick_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lvgl_tick"
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&tick_timer_args, &s_lvgl_tick_timer), TAG, "LVGL timer create failed");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_lvgl_tick_timer, 5000), TAG, "LVGL tick start failed");

    BaseType_t created = xTaskCreatePinnedToCore(lcd_lvgl_task, "lvgl", 4096, NULL, 5, &s_lvgl_task_handle, 0);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_lvgl_ready = true;
    return ESP_OK;
}

static int lcd_spi_log_vprintf(const char *fmt, va_list args)
{
    va_list copy;
    va_copy(copy, args);
    int required = vsnprintf(NULL, 0, fmt, copy);
    va_end(copy);

    char stack_buf[256];
    char *buffer = stack_buf;
    size_t buf_capacity = sizeof(stack_buf);

    if (required >= 0 && (size_t)(required + 1) > buf_capacity) {
        buffer = (char *)malloc((size_t)(required + 1));
        if (!buffer) {
            return s_prev_logger ? s_prev_logger(fmt, args) : vprintf(fmt, args);
        }
        buf_capacity = (size_t)(required + 1);
    }

    va_list copy2;
    va_copy(copy2, args);
    vsnprintf(buffer, buf_capacity, fmt, copy2);
    va_end(copy2);

    lcd_log_append(buffer);

    int result = s_prev_logger ? s_prev_logger(fmt, args) : vprintf(fmt, args);

    if (buffer != stack_buf) {
        free(buffer);
    }

    return result;
}

esp_err_t lcd_spi_init(void)
{
    ESP_RETURN_ON_ERROR(lcd_spi_configure_backlight(), TAG, "Backlight setup failed");
    ESP_RETURN_ON_ERROR(lcd_spi_configure_panel(), TAG, "Panel setup failed");
    ESP_RETURN_ON_ERROR(lcd_lvgl_init(), TAG, "LVGL init failed");
    ESP_LOGI(TAG, "SPI LCD ready (ST7789 + LVGL)");
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

esp_err_t lcd_spi_install_log_sink(void)
{
    if (!s_lvgl_ready || !s_log_label) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_log_sink_installed) {
        return ESP_OK;
    }

    s_prev_logger = esp_log_set_vprintf(lcd_spi_log_vprintf);
    s_log_sink_installed = true;
    lcd_log_append("[LCD] Log mirror ready\n");
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

    if (!s_panel_io) {
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, io_config, &s_panel_io), TAG,
                            "Panel IO create failed");
    }

    *out_handle = s_panel_io;
    return ESP_OK;
}
#endif
#include "lcd_spi.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"

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

#define LCD_WIDTH               240
#define LCD_HEIGHT              240
#define FONT_WIDTH              6
#define FONT_HEIGHT             8
#define TEXT_COLUMNS            (LCD_WIDTH / FONT_WIDTH)
#define TEXT_ROWS               (LCD_HEIGHT / FONT_HEIGHT)
#define COLOR_BACKGROUND        0x0000
#define COLOR_FOREGROUND        0xFFFF

static const char *TAG = "LCD";

static bool s_spi_bus_ready;
static bool s_backlight_ready;
static bool s_panel_ready;

static esp_lcd_panel_io_handle_t s_panel_io;
static esp_lcd_panel_handle_t s_panel_handle;

static char s_text_buffer[TEXT_ROWS][TEXT_COLUMNS];
static char s_pending_line[TEXT_COLUMNS];
static size_t s_pending_len;
static bool s_text_dirty;

static vprintf_like_t s_prev_logger;
static bool s_log_sink_installed;

/*
 * Glyph data derived from font8x8 by Daniel Hepper (MIT License).
 * https://github.com/dhepper/font8x8
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
static const uint8_t s_font8x8_basic[96][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // 0x20 ' '
    {0x00, 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 0x00}, // 0x21 '!'
    {0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00}, // 0x22 '"'
    {0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00, 0x00}, // 0x23 '#'
    {0x00, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00, 0x00}, // 0x24 '$'
    {0x00, 0x23, 0x13, 0x08, 0x64, 0x62, 0x00, 0x00}, // 0x25 '%'
    {0x00, 0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x00}, // 0x26 '&'
    {0x00, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00}, // 0x27 '\''
    {0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, 0x00, 0x00}, // 0x28 '('
    {0x00, 0x00, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00}, // 0x29 ')'
    {0x00, 0x14, 0x08, 0x3E, 0x08, 0x14, 0x00, 0x00}, // 0x2A '*'
    {0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00}, // 0x2B '+'
    {0x00, 0x00, 0x50, 0x30, 0x00, 0x00, 0x00, 0x00}, // 0x2C ','
    {0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00}, // 0x2D '-'
    {0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00}, // 0x2E '.'
    {0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00}, // 0x2F '/'
    {0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x00}, // 0x30 '0'
    {0x00, 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, 0x00}, // 0x31 '1'
    {0x00, 0x62, 0x51, 0x49, 0x49, 0x46, 0x00, 0x00}, // 0x32 '2'
    {0x00, 0x22, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00}, // 0x33 '3'
    {0x00, 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, 0x00}, // 0x34 '4'
    {0x00, 0x2F, 0x49, 0x49, 0x49, 0x31, 0x00, 0x00}, // 0x35 '5'
    {0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00, 0x00}, // 0x36 '6'
    {0x00, 0x01, 0x71, 0x09, 0x05, 0x03, 0x00, 0x00}, // 0x37 '7'
    {0x00, 0x36, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00}, // 0x38 '8'
    {0x00, 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x00}, // 0x39 '9'
    {0x00, 0x00, 0x36, 0x36, 0x00, 0x00, 0x00, 0x00}, // 0x3A ':'
    {0x00, 0x00, 0x56, 0x36, 0x00, 0x00, 0x00, 0x00}, // 0x3B ';'
    {0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00, 0x00}, // 0x3C '<'
    {0x00, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x00}, // 0x3D '='
    {0x00, 0x00, 0x41, 0x22, 0x14, 0x08, 0x00, 0x00}, // 0x3E '>'
    {0x00, 0x02, 0x01, 0x59, 0x09, 0x06, 0x00, 0x00}, // 0x3F '?'
    {0x00, 0x3E, 0x41, 0x5D, 0x59, 0x4E, 0x00, 0x00}, // 0x40 '@'
    {0x00, 0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00, 0x00}, // 0x41 'A'
    {0x00, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00}, // 0x42 'B'
    {0x00, 0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, 0x00}, // 0x43 'C'
    {0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00, 0x00}, // 0x44 'D'
    {0x00, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00, 0x00}, // 0x45 'E'
    {0x00, 0x7F, 0x09, 0x09, 0x09, 0x01, 0x00, 0x00}, // 0x46 'F'
    {0x00, 0x3E, 0x41, 0x41, 0x51, 0x32, 0x00, 0x00}, // 0x47 'G'
    {0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x00}, // 0x48 'H'
    {0x00, 0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, 0x00}, // 0x49 'I'
    {0x00, 0x20, 0x40, 0x41, 0x3F, 0x01, 0x00, 0x00}, // 0x4A 'J'
    {0x00, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00}, // 0x4B 'K'
    {0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00}, // 0x4C 'L'
    {0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00, 0x00}, // 0x4D 'M'
    {0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00, 0x00}, // 0x4E 'N'
    {0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00, 0x00}, // 0x4F 'O'
    {0x00, 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00, 0x00}, // 0x50 'P'
    {0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00, 0x00}, // 0x51 'Q'
    {0x00, 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00, 0x00}, // 0x52 'R'
    {0x00, 0x46, 0x49, 0x49, 0x49, 0x31, 0x00, 0x00}, // 0x53 'S'
    {0x00, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x00, 0x00}, // 0x54 'T'
    {0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00, 0x00}, // 0x55 'U'
    {0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00, 0x00}, // 0x56 'V'
    {0x00, 0x7F, 0x20, 0x18, 0x20, 0x7F, 0x00, 0x00}, // 0x57 'W'
    {0x00, 0x63, 0x14, 0x08, 0x14, 0x63, 0x00, 0x00}, // 0x58 'X'
    {0x00, 0x07, 0x08, 0x70, 0x08, 0x07, 0x00, 0x00}, // 0x59 'Y'
    {0x00, 0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x00}, // 0x5A 'Z'
    {0x00, 0x7F, 0x41, 0x41, 0x00, 0x00, 0x00, 0x00}, // 0x5B '['
    {0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00}, // 0x5C '\'
    {0x00, 0x00, 0x41, 0x41, 0x7F, 0x00, 0x00, 0x00}, // 0x5D ']'
    {0x00, 0x04, 0x02, 0x01, 0x02, 0x04, 0x00, 0x00}, // 0x5E '^'
    {0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00}, // 0x5F '_'
    {0x00, 0x00, 0x01, 0x02, 0x04, 0x00, 0x00, 0x00}, // 0x60 '`'
    {0x00, 0x20, 0x54, 0x54, 0x54, 0x78, 0x00, 0x00}, // 0x61 'a'
    {0x00, 0x7F, 0x48, 0x44, 0x44, 0x38, 0x00, 0x00}, // 0x62 'b'
    {0x00, 0x38, 0x44, 0x44, 0x44, 0x20, 0x00, 0x00}, // 0x63 'c'
    {0x00, 0x38, 0x44, 0x44, 0x48, 0x7F, 0x00, 0x00}, // 0x64 'd'
    {0x00, 0x38, 0x54, 0x54, 0x54, 0x18, 0x00, 0x00}, // 0x65 'e'
    {0x00, 0x08, 0x7E, 0x09, 0x01, 0x02, 0x00, 0x00}, // 0x66 'f'
    {0x00, 0x0C, 0x52, 0x52, 0x52, 0x3E, 0x00, 0x00}, // 0x67 'g'
    {0x00, 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x00}, // 0x68 'h'
    {0x00, 0x00, 0x44, 0x7D, 0x40, 0x00, 0x00, 0x00}, // 0x69 'i'
    {0x00, 0x20, 0x40, 0x44, 0x3D, 0x00, 0x00, 0x00}, // 0x6A 'j'
    {0x00, 0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x00}, // 0x6B 'k'
    {0x00, 0x00, 0x41, 0x7F, 0x40, 0x00, 0x00, 0x00}, // 0x6C 'l'
    {0x00, 0x7C, 0x04, 0x18, 0x04, 0x78, 0x00, 0x00}, // 0x6D 'm'
    {0x00, 0x7C, 0x08, 0x04, 0x04, 0x78, 0x00, 0x00}, // 0x6E 'n'
    {0x00, 0x38, 0x44, 0x44, 0x44, 0x38, 0x00, 0x00}, // 0x6F 'o'
    {0x00, 0x7C, 0x14, 0x14, 0x14, 0x08, 0x00, 0x00}, // 0x70 'p'
    {0x00, 0x08, 0x14, 0x14, 0x18, 0x7C, 0x00, 0x00}, // 0x71 'q'
    {0x00, 0x7C, 0x08, 0x04, 0x04, 0x08, 0x00, 0x00}, // 0x72 'r'
    {0x00, 0x48, 0x54, 0x54, 0x54, 0x20, 0x00, 0x00}, // 0x73 's'
    {0x00, 0x04, 0x3F, 0x44, 0x40, 0x20, 0x00, 0x00}, // 0x74 't'
    {0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00, 0x00}, // 0x75 'u'
    {0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00, 0x00}, // 0x76 'v'
    {0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00, 0x00}, // 0x77 'w'
    {0x00, 0x44, 0x28, 0x10, 0x28, 0x44, 0x00, 0x00}, // 0x78 'x'
    {0x00, 0x0C, 0x50, 0x50, 0x50, 0x3C, 0x00, 0x00}, // 0x79 'y'
    {0x00, 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00, 0x00}, // 0x7A 'z'
    {0x00, 0x08, 0x36, 0x41, 0x00, 0x00, 0x00, 0x00}, // 0x7B '{'
    {0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00}, // 0x7C '|'
    {0x00, 0x00, 0x41, 0x36, 0x08, 0x00, 0x00, 0x00}, // 0x7D '}'
    {0x00, 0x08, 0x04, 0x08, 0x10, 0x08, 0x00, 0x00}, // 0x7E '~'
    {0x00, 0x1C, 0x22, 0x41, 0x22, 0x1C, 0x00, 0x00}, // 0x7F DEL (unused)
};

static const uint8_t *lcd_font_lookup(char c)
{
    if ((uint8_t)c < 32 || (uint8_t)c > 127) {
        c = '?';
    }
    return s_font8x8_basic[(uint8_t)c - 32];
}

static void lcd_text_clear(void)
{
    for (size_t row = 0; row < TEXT_ROWS; ++row) {
        memset(s_text_buffer[row], ' ', TEXT_COLUMNS);
    }
    memset(s_pending_line, 0, sizeof(s_pending_line));
    s_pending_len = 0;
    s_text_dirty = true;
}

static void lcd_text_push_line(const char *line, size_t len)
{
    if (TEXT_ROWS == 0) {
        return;
    }

    if (len > TEXT_COLUMNS) {
        len = TEXT_COLUMNS;
    }

    if (TEXT_ROWS > 1) {
        memmove(s_text_buffer, s_text_buffer[1], (TEXT_ROWS - 1) * TEXT_COLUMNS);
    }
    memset(s_text_buffer[TEXT_ROWS - 1], ' ', TEXT_COLUMNS);
    if (len > 0) {
        memcpy(s_text_buffer[TEXT_ROWS - 1], line, len);
    }
    s_text_dirty = true;
}

static void lcd_text_flush_pending(void)
{
    lcd_text_push_line(s_pending_line, s_pending_len);
    s_pending_len = 0;
    memset(s_pending_line, 0, sizeof(s_pending_line));
}

static void lcd_text_process_chunk(const char *chunk)
{
    if (!chunk) {
        return;
    }

    while (*chunk) {
        char ch = *chunk++;
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            lcd_text_flush_pending();
            continue;
        }
        if (s_pending_len >= TEXT_COLUMNS) {
            lcd_text_flush_pending();
        }
        s_pending_line[s_pending_len++] = ch;
    }
}

static void lcd_text_render(void)
{
    if (!s_panel_ready || !s_text_dirty) {
        return;
    }

    static uint16_t line_buffer[LCD_WIDTH * FONT_HEIGHT];

    for (size_t row = 0; row < TEXT_ROWS; ++row) {
        for (size_t i = 0; i < LCD_WIDTH * FONT_HEIGHT; ++i) {
            line_buffer[i] = COLOR_BACKGROUND;
        }

        for (size_t col = 0; col < TEXT_COLUMNS; ++col) {
            const uint8_t *glyph = lcd_font_lookup(s_text_buffer[row][col]);
            for (size_t y = 0; y < FONT_HEIGHT && y < 8; ++y) {
                uint8_t bits = glyph[y];
                for (size_t x = 0; x < FONT_WIDTH; ++x) {
                    size_t dst_x = col * FONT_WIDTH + x;
                    if (dst_x >= LCD_WIDTH) {
                        continue;
                    }
                    bool pixel_on = false;
                    if (x < 6) {
                        pixel_on = (bits & (1U << (7 - x))) != 0;
                    }
                    if (pixel_on) {
                        line_buffer[y * LCD_WIDTH + dst_x] = COLOR_FOREGROUND;
                    }
                }
            }
        }

        esp_lcd_panel_draw_bitmap(s_panel_handle, 0, (int)(row * FONT_HEIGHT), LCD_WIDTH,
                                  (int)((row + 1) * FONT_HEIGHT), line_buffer);
    }

    s_text_dirty = false;
}

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
    } else {
        ESP_RETURN_ON_ERROR(err, TAG, "SPI bus init failed");
        s_spi_bus_ready = true;
    }

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

    return ESP_OK;
}

static esp_err_t lcd_spi_configure_panel(void)
{
    if (s_panel_ready) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(lcd_spi_configure_bus(), TAG, "Bus setup failed");

    if (!s_panel_io) {
        esp_lcd_panel_io_spi_config_t io_config = {
            .dc_gpio_num = LCD_PIN_DC,
            .cs_gpio_num = LCD_PIN_CS,
            .pclk_hz = 40 * 1000 * 1000,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .spi_mode = 0,
            .trans_queue_depth = 10,
        };
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &io_config, &s_panel_io), TAG,
                            "Panel IO create failed");
    }

    if (!s_panel_handle) {
        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = LCD_PIN_RST,
            .color_space = ESP_LCD_COLOR_SPACE_RGB,
            .bits_per_pixel = 16,
        };
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(s_panel_io, &panel_config, &s_panel_handle), TAG,
                            "ST7789 panel create failed");
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel_handle), TAG, "Panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel_handle), TAG, "Panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel_handle, true), TAG, "Panel display on failed");

    s_panel_ready = true;
    lcd_text_clear();
    lcd_text_render();

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

static int lcd_spi_log_vprintf(const char *fmt, va_list args)
{
    va_list copy;
    va_copy(copy, args);
    int required = vsnprintf(NULL, 0, fmt, copy);
    va_end(copy);

    char stack_buf[256];
    char *buffer = stack_buf;
    size_t buf_capacity = sizeof(stack_buf);

    if (required >= 0 && (size_t)(required + 1) > buf_capacity) {
        buffer = (char *)malloc((size_t)(required + 1));
        if (!buffer) {
            return s_prev_logger ? s_prev_logger(fmt, args) : vprintf(fmt, args);
        }
        buf_capacity = (size_t)(required + 1);
    }

    va_list copy2;
    va_copy(copy2, args);
    vsnprintf(buffer, buf_capacity, fmt, copy2);
    va_end(copy2);

    lcd_text_process_chunk(buffer);
    lcd_text_render();

    int result = s_prev_logger ? s_prev_logger(fmt, args) : vprintf(fmt, args);

    if (buffer != stack_buf) {
        free(buffer);
    }

    return result;
}

esp_err_t lcd_spi_init(void)
{
    ESP_RETURN_ON_ERROR(lcd_spi_configure_backlight(), TAG, "Backlight setup failed");
    ESP_RETURN_ON_ERROR(lcd_spi_configure_panel(), TAG, "Panel setup failed");
    ESP_LOGI(TAG, "SPI LCD ready (ST7789 %dx%d)", LCD_WIDTH, LCD_HEIGHT);
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

esp_err_t lcd_spi_install_log_sink(void)
{
    if (!s_panel_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_log_sink_installed) {
        return ESP_OK;
    }

    s_prev_logger = esp_log_set_vprintf(lcd_spi_log_vprintf);
    s_log_sink_installed = true;
    lcd_text_process_chunk("[LCD] Log mirror ready\n");
    lcd_text_render();
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

    if (!s_panel_io) {
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, io_config, &s_panel_io), TAG,
                            "Panel IO create failed");
    }

    *out_handle = s_panel_io;
    return ESP_OK;
}
#endif
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

#endif /* LVGL_LEGACY_IMPLEMENTATION */
