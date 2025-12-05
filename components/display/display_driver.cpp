// display_driver.cpp
// 说明：基于 LovyanGFX 的 ST7789 驱动，并与 LVGL 绑定

#include "display_driver.h"

#include "macro_def.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "hal/spi_types.h"
#include "lvgl.h"

#include <LovyanGFX.hpp>
#include <cstring>

namespace {

static const char *TAG = "disp";

static constexpr uint16_t DISP_H_RES = 240;
static constexpr uint16_t DISP_V_RES = 320;
static constexpr uint32_t DRAW_BUF_LINES = 40;
static constexpr uint32_t LVGL_TICK_MS = 5;

class LGFX_ST7789 : public lgfx::LGFX_Device {
public:
    LGFX_ST7789()
    {
        auto bus_cfg = _bus.config();
        bus_cfg.spi_host = SPI3_HOST;
        bus_cfg.freq_write = 80000000;
        bus_cfg.freq_read = 20000000;
        bus_cfg.spi_mode = 0;
        bus_cfg.use_lock = true;
        bus_cfg.dma_channel = SPI_DMA_CH_AUTO;
        bus_cfg.pin_sclk = GPIO_LCD_SCK;
        bus_cfg.pin_mosi = GPIO_LCD_MOSI;
        bus_cfg.pin_miso = -1;
        bus_cfg.pin_dc = GPIO_LCD_DC;
        _bus.config(bus_cfg);
        _panel.setBus(&_bus);

        auto panel_cfg = _panel.config();
        panel_cfg.pin_cs = GPIO_LCD_CS;
        panel_cfg.pin_rst = GPIO_LCD_RST;
        panel_cfg.pin_busy = -1;
        panel_cfg.panel_width = DISP_H_RES;
        panel_cfg.panel_height = DISP_V_RES;
        panel_cfg.memory_width = DISP_H_RES;
        panel_cfg.memory_height = DISP_V_RES;
        panel_cfg.offset_x = 0;
        panel_cfg.offset_y = 0;
        panel_cfg.offset_rotation = 2;
        panel_cfg.readable = false;
        panel_cfg.invert = true;
        panel_cfg.rgb_order = false;
        _panel.config(panel_cfg);

        setPanel(&_panel);
    }

private:
    lgfx::Panel_ST7789 _panel;
    lgfx::Bus_SPI _bus;
};

static LGFX_ST7789 s_lgfx;
static bool s_display_ready = false;
static bool s_lvgl_initialized = false;
static lv_display_t *s_lv_display = nullptr;
static lv_color_t *s_draw_buf1 = nullptr;
static lv_color_t *s_draw_buf2 = nullptr;
static esp_timer_handle_t s_lv_tick_timer = nullptr;
static uint8_t s_backlight_percent = 0;

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    if (px_map == nullptr || area->x2 < area->x1 || area->y2 < area->y1) {
        lv_display_flush_ready(disp);
        return;
    }

    const uint32_t w = static_cast<uint32_t>(area->x2 - area->x1 + 1);
    const uint32_t h = static_cast<uint32_t>(area->y2 - area->y1 + 1);
    s_lgfx.startWrite();
    s_lgfx.setAddrWindow(area->x1, area->y1, w, h);
    s_lgfx.writePixels(reinterpret_cast<const lgfx::rgb565_t *>(px_map), w * h, true);
    s_lgfx.endWrite();
    lv_display_flush_ready(disp);
}

static void lv_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(LVGL_TICK_MS);
}

static esp_err_t init_backlight_gpio(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = 1ULL << GPIO_LCD_BL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_cfg);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(GPIO_LCD_BL, 0);
    s_backlight_percent = 0;
    return ESP_OK;
}

static void free_draw_buffers(void)
{
    if (s_draw_buf1) {
        heap_caps_free(s_draw_buf1);
        s_draw_buf1 = nullptr;
    }
    if (s_draw_buf2) {
        heap_caps_free(s_draw_buf2);
        s_draw_buf2 = nullptr;
    }
}

static esp_err_t alloc_draw_buffers(void)
{
    const size_t buf_pixels = DISP_H_RES * DRAW_BUF_LINES;
    const size_t buf_size = buf_pixels * sizeof(lv_color_t);
    s_draw_buf1 = static_cast<lv_color_t *>(heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    s_draw_buf2 = static_cast<lv_color_t *>(heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (!s_draw_buf1 || !s_draw_buf2) {
        ESP_LOGE(TAG, "lvgl buffer alloc failed");
        free_draw_buffers();
        return ESP_ERR_NO_MEM;
    }
    memset(s_draw_buf1, 0, buf_size);
    memset(s_draw_buf2, 0, buf_size);
    return ESP_OK;
}

static esp_err_t create_lvgl_display(void)
{
    if (!s_lvgl_initialized) {
        lv_init();
        s_lvgl_initialized = true;
    }

    esp_err_t err = alloc_draw_buffers();
    if (err != ESP_OK) {
        return err;
    }

    s_lv_display = lv_display_create(DISP_H_RES, DISP_V_RES);
    if (s_lv_display == nullptr) {
        ESP_LOGE(TAG, "lv_display_create failed");
        free_draw_buffers();
        return ESP_ERR_NO_MEM;
    }

    lv_display_set_color_format(s_lv_display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(s_lv_display, lvgl_flush_cb);
    const uint32_t buf_pixels = DISP_H_RES * DRAW_BUF_LINES;
    lv_display_set_buffers(s_lv_display,
                           s_draw_buf1,
                           s_draw_buf2,
                           buf_pixels,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_default(s_lv_display);
    return ESP_OK;
}

static esp_err_t start_lvgl_tick_timer(void)
{
    if (s_lv_tick_timer) {
        return ESP_OK;
    }
    const esp_timer_create_args_t args = {
        .callback = &lv_tick_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lv_tick",
        .skip_unhandled_events = false,
    };
    esp_err_t err = esp_timer_create(&args, &s_lv_tick_timer);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_timer_start_periodic(s_lv_tick_timer, LVGL_TICK_MS * 1000ULL);
    if (err != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

static esp_err_t init_lovyangfx(void)
{
    if (!s_lgfx.init()) {
        ESP_LOGE(TAG, "LovyanGFX init failed");
        return ESP_FAIL;
    }
    s_lgfx.setRotation(1);
    s_lgfx.fillScreen(lgfx::color565(0, 0, 0));
    return ESP_OK;
}

} // namespace

extern "C" {

esp_err_t display_driver_init(void)
{
    if (s_display_ready) {
        return ESP_OK;
    }

    esp_err_t err = init_backlight_gpio();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "backlight gpio init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = init_lovyangfx();
    if (err != ESP_OK) {
        return err;
    }

    err = create_lvgl_display();
    if (err != ESP_OK) {
        return err;
    }

    err = start_lvgl_tick_timer();
    if (err != ESP_OK) {
        return err;
    }

    err = display_driver_set_backlight(100);
    if (err != ESP_OK) {
        return err;
    }
    s_display_ready = true;
    ESP_LOGI(TAG, "display driver ready (%ux%u)", DISP_H_RES, DISP_V_RES);
    return ESP_OK;
}

bool display_driver_is_ready(void)
{
    return s_display_ready;
}

esp_err_t display_driver_set_backlight(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    s_backlight_percent = percent;
    // 当前硬件仅支持开/关，后续可扩展 PWM 调光
    return gpio_set_level(GPIO_LCD_BL, percent > 0 ? 1 : 0);
}

} // extern "C"
