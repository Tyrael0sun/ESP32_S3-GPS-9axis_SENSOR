#include "gnss.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define GNSS_UART_NUM              UART_NUM_1
#define GNSS_UART_BAUD             115200
#define GNSS_UART_RX_BUF           4096
#define GNSS_UART_TX_BUF           0
#define GNSS_PIN_TX                17
#define GNSS_PIN_RX                18
#define GNSS_PRESENCE_TIMEOUT_US   (5 * 1000 * 1000)

static const char *TAG = "GNSS";

static SemaphoreHandle_t s_lock;
static TaskHandle_t s_uart_task;
static gnss_fix_t s_fix = {0};
static bool s_fix_valid;
static int s_tracked_satellites;
static int64_t s_last_sentence_us;

static double gnss_parse_latlon(const char *value, const char *hemi)
{
    if (!value || !value[0] || !hemi || !hemi[0]) {
        return NAN;
    }

    double raw = atof(value);
    double degrees = floor(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);

    if (hemi[0] == 'S' || hemi[0] == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

static void gnss_handle_gga(char **fields, size_t field_count)
{
    if (field_count < 10) {
        return;
    }

    int fix_quality = atoi(fields[6]);
    int satellites = atoi(fields[7]);
    double altitude = atof(fields[9]);
    double latitude = gnss_parse_latlon(fields[2], fields[3]);
    double longitude = gnss_parse_latlon(fields[4], fields[5]);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_tracked_satellites = satellites;
    if (!isnan(latitude) && !isnan(longitude)) {
        s_fix.latitude_deg = latitude;
        s_fix.longitude_deg = longitude;
        s_fix.altitude_m = altitude;
        s_fix_valid = fix_quality > 0;
    }
    xSemaphoreGive(s_lock);
}

static void gnss_handle_rmc(char **fields, size_t field_count)
{
    if (field_count < 7) {
        return;
    }

    if (fields[2][0] != 'A') {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_fix_valid = false;
        xSemaphoreGive(s_lock);
        return;
    }

    double latitude = gnss_parse_latlon(fields[3], fields[4]);
    double longitude = gnss_parse_latlon(fields[5], fields[6]);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!isnan(latitude) && !isnan(longitude)) {
        s_fix.latitude_deg = latitude;
        s_fix.longitude_deg = longitude;
        s_fix_valid = true;
    }
    xSemaphoreGive(s_lock);
}

static void gnss_handle_gsv(char **fields, size_t field_count)
{
    if (field_count < 4) {
        return;
    }

    int satellites = atoi(fields[3]);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (satellites > s_tracked_satellites) {
        s_tracked_satellites = satellites;
    }
    xSemaphoreGive(s_lock);
}

static void gnss_process_sentence(const char *sentence)
{
    if (!sentence || sentence[0] != '$') {
        return;
    }

    char buffer[128];
    size_t len = strnlen(sentence, sizeof(buffer) - 1);
    memcpy(buffer, sentence, len);
    buffer[len] = '\0';

    char *save_ptr = NULL;
    char *fields[16] = {0};
    size_t field_count = 0;

    for (char *token = strtok_r(buffer, ",", &save_ptr);
         token && field_count < (sizeof(fields) / sizeof(fields[0]));
         token = strtok_r(NULL, ",", &save_ptr)) {
        fields[field_count++] = token;
    }

    if (!field_count) {
        return;
    }

    const char *type = fields[0];
    if (strcmp(type, "$GNGGA") == 0 || strcmp(type, "$GPGGA") == 0) {
        gnss_handle_gga(fields, field_count);
    } else if (strcmp(type, "$GNRMC") == 0 || strcmp(type, "$GPRMC") == 0) {
        gnss_handle_rmc(fields, field_count);
    } else if (strcmp(type, "$GNGSV") == 0 || strcmp(type, "$GPGSV") == 0) {
        gnss_handle_gsv(fields, field_count);
    }
}

static void gnss_uart_task(void *arg)
{
    uint8_t byte = 0;
    char line[128] = {0};
    size_t pos = 0;

    while (true) {
        int length = uart_read_bytes(GNSS_UART_NUM, &byte, 1, pdMS_TO_TICKS(500));
        if (length > 0) {
            if (byte == '\r') {
                continue;
            }
            if (byte == '\n') {
                line[pos] = '\0';
                if (pos > 0) {
                    s_last_sentence_us = esp_timer_get_time();
                    gnss_process_sentence(line);
                }
                pos = 0;
                continue;
            }
            if (pos < sizeof(line) - 1) {
                line[pos++] = (char)byte;
            } else {
                pos = 0;
            }
        }
    }
}

esp_err_t gnss_init(void)
{
    if (s_uart_task) {
        return ESP_OK;
    }

    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        if (!s_lock) {
            return ESP_ERR_NO_MEM;
        }
    }

    uart_config_t cfg = {
        .baud_rate = GNSS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(GNSS_UART_NUM, GNSS_UART_RX_BUF, GNSS_UART_TX_BUF, 0, NULL, 0),
                        TAG, "UART install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(GNSS_UART_NUM, &cfg), TAG, "UART config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(GNSS_UART_NUM, GNSS_PIN_TX, GNSS_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "UART pin config failed");

    BaseType_t created = xTaskCreatePinnedToCore(gnss_uart_task, "gnss_uart", 4096, NULL, 4, &s_uart_task, 0);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "GNSS UART ready");
    return ESP_OK;
}

bool gnss_is_present(void)
{
    int64_t now = esp_timer_get_time();
    return (now - s_last_sentence_us) < GNSS_PRESENCE_TIMEOUT_US;
}

int gnss_get_tracked_satellites(void)
{
    if (!gnss_is_present()) {
        return 0;
    }

    int satellites = 0;
    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
        satellites = s_tracked_satellites;
        xSemaphoreGive(s_lock);
    } else {
        satellites = s_tracked_satellites;
    }
    return satellites;
}

void gnss_get_last_fix(gnss_fix_t *fix)
{
    if (!fix) {
        return;
    }

    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        *fix = s_fix;
        xSemaphoreGive(s_lock);
    } else {
        *fix = s_fix;
    }
}
