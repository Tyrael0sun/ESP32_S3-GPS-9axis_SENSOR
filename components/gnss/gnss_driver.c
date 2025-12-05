// gnss_driver.c
// 说明：NEO-M8N GNSS 驱动实现，负责 UART 收发和 DataModel 更新

#include "gnss_driver.h"
#include "data_model.h"
#include "macro_def.h"
#include "app_check.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "rtc_manager.h"

#define GNSS_UART_NUM       UART_NUM_1
#define GNSS_RX_BUF_SIZE    2048
#define GNSS_TX_BUF_SIZE    256
#define GNSS_UBX_SYNC_1     0xB5
#define GNSS_UBX_SYNC_2     0x62
#define GNSS_DEFAULT_BAUD   9600
#define GNSS_HIGH_BAUD      115200
#define GNSS_LDO_STABLE_MS  120
#define GNSS_RECONF_DELAY_MS 2000

#define GNSS_EVENT_QUEUE_LEN 16

static const char *TAG = "gnss";

static TaskHandle_t s_gnss_task = NULL;
static QueueHandle_t s_cfg_queue = NULL;

typedef enum {
    GNSS_CFG_CMD_RATE_1HZ = 0,
    GNSS_CFG_CMD_RATE_5HZ,
} gnss_cfg_cmd_t;

static void parse_nmea_sentence(app_data_model_t *m, const char *line);
static void gnss_task(void *arg);
static esp_err_t send_ubx_cmd(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t payload_len);
static esp_err_t gnss_switch_baud(int baud);
static void parse_gsv(app_data_model_t *m, char **tokens, int count);
static void parse_gsa(app_data_model_t *m, char **tokens, int count);
static void update_rt_clock(app_data_model_t *m, const char *hhmmss, const char *ddmmyy);

esp_err_t gnss_driver_init(void)
{
    gpio_config_t ldo_cfg = {
        .pin_bit_mask = BIT64(GPIO_GPS_LDO_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    CHECK_ESP_RETURN(gpio_config(&ldo_cfg));
    gpio_set_level(GPIO_GPS_LDO_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(GNSS_LDO_STABLE_MS));

    CHECK_ESP_RETURN(uart_driver_install(GNSS_UART_NUM,
                                         GNSS_RX_BUF_SIZE,
                                         GNSS_TX_BUF_SIZE,
                                         0,
                                         NULL,
                                         0));

    uart_config_t uart_cfg = {
        .baud_rate = GNSS_DEFAULT_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    CHECK_ESP_RETURN(uart_param_config(GNSS_UART_NUM, &uart_cfg));
    CHECK_ESP_RETURN(uart_set_pin(GNSS_UART_NUM,
                                  GPIO_GNSS_TX,
                                  GPIO_GNSS_RX,
                                  UART_PIN_NO_CHANGE,
                                  UART_PIN_NO_CHANGE));

    // 配置 GNSS：切换波特率、星座和更新率
    CHECK_ESP_RETURN(send_ubx_cmd(0x06, 0x00, (const uint8_t[]){
        0x01, 0x00,             // UART1
        0x00, 0x00, 0x00, 0x00,
        0xD0, 0x08, 0x00, 0x00,
        0x00, 0xC2, 0x01, 0x00,
        0x07, 0x00,             // 115200
        0x03, 0x00
    }, 20));

    vTaskDelay(pdMS_TO_TICKS(GNSS_RECONF_DELAY_MS));
    CHECK_ESP_RETURN(gnss_switch_baud(GNSS_HIGH_BAUD));
    uart_cfg.baud_rate = GNSS_HIGH_BAUD;
    CHECK_ESP_RETURN(uart_param_config(GNSS_UART_NUM, &uart_cfg));

    CHECK_ESP_RETURN(send_ubx_cmd(0x06, 0x3E, (const uint8_t[]){
        0x00, 0x20, 0x20, 0x07,
        0x08, 0x10, 0x00, 0x01,
        0x00, 0x20, 0x00, 0x01,
        0x00, 0x08, 0x00, 0x01,
        0x00, 0x08, 0x00, 0x01,
        0x00, 0x08, 0x00, 0x01,
        0x00, 0x08, 0x00, 0x01,
        0x00, 0x08, 0x00, 0x01
    }, 32));

    CHECK_ESP_RETURN(send_ubx_cmd(0x06, 0x08, (const uint8_t[]){
        0xE8, 0x03,
        0x01, 0x00,
        0x01, 0x00
    }, 6));

    CHECK_ESP_RETURN(send_ubx_cmd(0x06, 0x09, (const uint8_t[]){
        0x00, 0x00, 0x00, 0x00,
        0xFF, 0xFF, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
    }, 12));

    if (!s_cfg_queue) {
        s_cfg_queue = xQueueCreate(GNSS_EVENT_QUEUE_LEN, sizeof(gnss_cfg_cmd_t));
    }

    if (s_gnss_task == NULL) {
        BaseType_t ret = xTaskCreatePinnedToCore(gnss_task,
                                                 "gnss_task",
                                                 4096,
                                                 NULL,
                                                 7,
                                                 &s_gnss_task,
                                                 0);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "create gnss task failed");
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "gnss driver initialized");
    return ESP_OK;
}

static void gnss_task(void *arg)
{
    app_data_model_t *m = data_model_get();
    char line_buf[256];
    size_t line_pos = 0;
    gnss_cfg_cmd_t cmd;

    while (1) {
        if (s_cfg_queue && xQueueReceive(s_cfg_queue, &cmd, 0) == pdTRUE) {
            if (cmd == GNSS_CFG_CMD_RATE_5HZ) {
                send_ubx_cmd(0x06, 0x08, (const uint8_t[]){
                    0x64, 0x00,
                    0x01, 0x00,
                    0x01, 0x00
                }, 6);
            } else {
                send_ubx_cmd(0x06, 0x08, (const uint8_t[]){
                    0xE8, 0x03,
                    0x01, 0x00,
                    0x01, 0x00
                }, 6);
            }
        }

        uint8_t ch;
        int len = uart_read_bytes(GNSS_UART_NUM, &ch, 1, pdMS_TO_TICKS(100));
        if (len == 1) {
            if (ch == '\n') {
                line_buf[line_pos] = '\0';
                parse_nmea_sentence(m, line_buf);
                line_pos = 0;
            } else if (line_pos < sizeof(line_buf) - 1) {
                line_buf[line_pos++] = (char)ch;
            } else {
                line_pos = 0;
            }
        }
    }
}

static bool checksum_ok(const char *line)
{
    const char *star = strchr(line, '*');
    if (!star) {
        return false;
    }

    uint8_t checksum = 0;
    for (const char *p = line + 1; p < star; ++p) {
        checksum ^= (uint8_t)(*p);
    }

    uint8_t expected = (uint8_t)strtol(star + 1, NULL, 16);
    return checksum == expected;
}

static void parse_gga(app_data_model_t *m, char **tokens, int count)
{
    if (count < 15) {
        return;
    }

    int fix = atoi(tokens[6]);
    if (fix == 0) {
        m->gnss.fix = GNSS_FIX_NONE;
        m->gnss.valid = false;
        return;
    }

    double lat_deg = atof(tokens[2]);
    double lon_deg = atof(tokens[4]);

    int lat_deg_int = (int)(lat_deg / 100);
    double lat_min = lat_deg - lat_deg_int * 100;
    double lat = lat_deg_int + lat_min / 60.0;
    if (tokens[3][0] == 'S') {
        lat = -lat;
    }

    int lon_deg_int = (int)(lon_deg / 100);
    double lon_min = lon_deg - lon_deg_int * 100;
    double lon = lon_deg_int + lon_min / 60.0;
    if (tokens[5][0] == 'W') {
        lon = -lon;
    }

    m->gnss.lat = lat;
    m->gnss.lon = lon;
    m->gnss.sats = (uint8_t)atoi(tokens[7]);
    m->gnss.altitude = (float)atof(tokens[9]);
    m->gnss.hdop = (float)atof(tokens[8]);
    m->gnss.fix = (fix == 1) ? GNSS_FIX_2D : GNSS_FIX_3D;
    m->gnss.valid = true;
    m->gnss.nmea_ok = 1;
    update_rt_clock(m, tokens[1], NULL);
}

static void parse_rmc(app_data_model_t *m, char **tokens, int count)
{
    if (count < 12) {
        return;
    }

    if (tokens[2][0] != 'A') {
        m->gnss.valid = false;
        return;
    }

    double lat_deg = atof(tokens[3]);
    double lon_deg = atof(tokens[5]);

    int lat_deg_int = (int)(lat_deg / 100);
    double lat_min = lat_deg - lat_deg_int * 100;
    double lat = lat_deg_int + lat_min / 60.0;
    if (tokens[4][0] == 'S') lat = -lat;

    int lon_deg_int = (int)(lon_deg / 100);
    double lon_min = lon_deg - lon_deg_int * 100;
    double lon = lon_deg_int + lon_min / 60.0;
    if (tokens[6][0] == 'W') lon = -lon;

    m->gnss.lat = lat;
    m->gnss.lon = lon;
    m->gnss.speed = (float)(atof(tokens[7]) * 0.514444f);
    m->gnss.course = (float)atof(tokens[8]);
    m->gnss.valid = true;
    m->gnss.nmea_ok = 1;
    update_rt_clock(m, tokens[1], tokens[9]);
}

static void parse_vtg(app_data_model_t *m, char **tokens, int count)
{
    if (count < 9) {
        return;
    }

    m->gnss.course = (float)atof(tokens[1]);
    m->gnss.speed = (float)(atof(tokens[7]) / 3.6f);
}

static void parse_gsv(app_data_model_t *m, char **tokens, int count)
{
    if (count < 4) {
        return;
    }
    m->gnss.sats_visible = (uint8_t)atoi(tokens[3]);
}

static void parse_gsa(app_data_model_t *m, char **tokens, int count)
{
    if (count < 18) { // GSA 至少 17 字段
        return;
    }
    m->gnss.pdop = (float)atof(tokens[15]);
    m->gnss.hdop = (float)atof(tokens[16]);
    m->gnss.vdop = (float)atof(tokens[17]);
}

static void parse_nmea_sentence(app_data_model_t *m, const char *line)
{
    if (line[0] != '$') {
        return;
    }

    if (!checksum_ok(line)) {
        ESP_LOGW(TAG, "nmea checksum error: %s", line);
        m->gnss.nmea_ok = 0;
        return;
    }

    char buf[256];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *tokens[24] = {0};
    int count = 0;
    char *saveptr = NULL;
    char *token = strtok_r(buf, ",", &saveptr);
    while (token && count < 24) {
        tokens[count++] = token;
        token = strtok_r(NULL, ",", &saveptr);
    }

    if (count == 0) {
        return;
    }

    if (strcmp(tokens[0], "$GNGGA") == 0 || strcmp(tokens[0], "$GPGGA") == 0) {
        parse_gga(m, tokens, count);
    } else if (strcmp(tokens[0], "$GNRMC") == 0 || strcmp(tokens[0], "$GPRMC") == 0) {
        parse_rmc(m, tokens, count);
    } else if (strcmp(tokens[0], "$GNVTG") == 0 || strcmp(tokens[0], "$GPVTG") == 0) {
        parse_vtg(m, tokens, count);
    } else if (strcmp(tokens[0], "$GNGSV") == 0 || strcmp(tokens[0], "$GPGSV") == 0) {
        parse_gsv(m, tokens, count);
    } else if (strcmp(tokens[0], "$GNGSA") == 0 || strcmp(tokens[0], "$GPGSA") == 0) {
        parse_gsa(m, tokens, count);
    }

    m->gnss.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static esp_err_t send_ubx_cmd(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t payload_len)
{
    uint8_t frame[256];
    uint16_t total_len = payload_len + 8;
    if (total_len > sizeof(frame)) {
        return ESP_ERR_INVALID_SIZE;
    }
    frame[0] = GNSS_UBX_SYNC_1;
    frame[1] = GNSS_UBX_SYNC_2;
    frame[2] = cls;
    frame[3] = id;
    frame[4] = payload_len & 0xFF;
    frame[5] = payload_len >> 8;
    memcpy(&frame[6], payload, payload_len);

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (int i = 2; i < 6 + payload_len; ++i) {
        ck_a += frame[i];
        ck_b += ck_a;
    }
    frame[6 + payload_len] = ck_a;
    frame[7 + payload_len] = ck_b;
    int bytes = uart_write_bytes(GNSS_UART_NUM, (const char *)frame, total_len);
    return (bytes == total_len) ? ESP_OK : ESP_FAIL;
}

static esp_err_t gnss_switch_baud(int baud)
{
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    return uart_param_config(GNSS_UART_NUM, &cfg);
}

    esp_err_t gnss_driver_request_fast_rate(bool enable)
    {
        if (!s_cfg_queue) {
            return ESP_ERR_INVALID_STATE;
        }
        gnss_cfg_cmd_t cmd = enable ? GNSS_CFG_CMD_RATE_5HZ : GNSS_CFG_CMD_RATE_1HZ;
        if (xQueueSend(s_cfg_queue, &cmd, 0) != pdTRUE) {
            xQueueReset(s_cfg_queue);
            if (xQueueSend(s_cfg_queue, &cmd, 0) != pdTRUE) {
                return ESP_ERR_TIMEOUT;
            }
        }
        return ESP_OK;
    }

static void gnss_request_high_rate(bool enable)
{
    if (!s_cfg_queue) {
        return;
    }
    gnss_cfg_cmd_t cmd = enable ? GNSS_CFG_CMD_RATE_5HZ : GNSS_CFG_CMD_RATE_1HZ;
    xQueueSend(s_cfg_queue, &cmd, 0);
}

static void update_rt_clock(app_data_model_t *m, const char *hhmmss, const char *ddmmyy)
{
    if (!hhmmss || strlen(hhmmss) < 6 || !ddmmyy || strlen(ddmmyy) < 6) {
        return;
    }
    struct tm t = {0};
    t.tm_hour = (hhmmss[0] - '0') * 10 + (hhmmss[1] - '0');
    t.tm_min = (hhmmss[2] - '0') * 10 + (hhmmss[3] - '0');
    t.tm_sec = (hhmmss[4] - '0') * 10 + (hhmmss[5] - '0');
    t.tm_mday = (ddmmyy[0] - '0') * 10 + (ddmmyy[1] - '0');
    t.tm_mon = ((ddmmyy[2] - '0') * 10 + (ddmmyy[3] - '0')) - 1;
    t.tm_year = ((ddmmyy[4] - '0') * 10 + (ddmmyy[5] - '0')) + 100;
    time_t utc = mktime(&t);
    rtc_manager_set_utc(utc, true);
    m->gnss.time_synced = true;
}
