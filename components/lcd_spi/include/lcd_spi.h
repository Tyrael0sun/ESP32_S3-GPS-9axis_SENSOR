#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lcd_spi_init(void);
esp_err_t lcd_spi_set_backlight_percent(uint8_t percent);
esp_err_t lcd_spi_install_log_sink(void);

#ifdef CONFIG_LCD_SPI_CREATE_PANEL_IO
#include "esp_lcd_panel_io.h"
esp_err_t lcd_spi_create_panel_io(const esp_lcd_panel_io_spi_config_t *io_config,
                                  esp_lcd_panel_io_handle_t *out_handle);
#endif

#ifdef __cplusplus
}
#endif
