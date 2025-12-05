// input_driver.c
// 说明：按键与旋转编码器输入采样任务，实现事件识别并发送到队列

#include "input_driver.h"
#include "macro_def.h"
#include "app_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "log_system.h"
#include "data_model.h"

static const char *TAG = "input";

static QueueHandle_t s_event_queue = NULL;
static TaskHandle_t s_task_handle = NULL;

static void send_event(input_event_type_t type)
{
    if (s_event_queue == NULL) {
        return;
    }
    input_event_t evt = {
        .type = type,
        .timestamp_ms = esp_timer_get_time() / 1000,
    };
    if (xQueueSend(s_event_queue, &evt, 0) == pdTRUE) {
        log_system_enqueue("[EVT] t=%lums type=%d", (unsigned long)evt.timestamp_ms, type);
    }
}

static void input_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last_wake = xTaskGetTickCount();

    TickType_t stable_ticks = 0;
    bool btn_state = true;
    bool btn_prev = true;
    uint32_t press_duration_ms = 0;
    uint32_t last_release_ms = 0;
    uint8_t pending_clicks = 0;
    bool long_sent = false;

    uint8_t enc_state = 0;
    uint8_t enc_last = 0;
    int8_t steps = 0;
    uint32_t last_step_ms = 0;

    while (1) {
        bool btn = gpio_get_level(GPIO_KEY_MAIN);
        if (btn == btn_prev) {
            stable_ticks += period;
        } else {
            stable_ticks = 0;
        }
        if (stable_ticks >= pdMS_TO_TICKS(100) && btn != btn_state) {
            btn_state = btn;
            stable_ticks = 0;
            if (!btn_state) {
                press_duration_ms = 0;
                long_sent = false;
            } else {
                uint32_t now_ms = esp_timer_get_time() / 1000;
                last_release_ms = now_ms;
                if (!long_sent) {
                    if (press_duration_ms < 300) {
                        pending_clicks++;
                    } else if (press_duration_ms < 700) {
                        send_event(INPUT_EVENT_BTN_CLICK);
                    } else if (press_duration_ms < 1300) {
                        send_event(INPUT_EVENT_BTN_MID);
                    } else if (press_duration_ms < 8000) {
                        send_event(INPUT_EVENT_BTN_LONG);
                    } else {
                        send_event(INPUT_EVENT_BTN_ULTRA);
                    }
                }
            }
        }
        btn_prev = btn;

        if (!btn_state) {
            press_duration_ms += period * portTICK_PERIOD_MS;
            if (!long_sent) {
                if (press_duration_ms >= 8000) {
                    send_event(INPUT_EVENT_BTN_ULTRA);
                    long_sent = true;
                    pending_clicks = 0;
                } else if (press_duration_ms >= 1500) {
                    send_event(INPUT_EVENT_BTN_LONG);
                    long_sent = true;
                    pending_clicks = 0;
                }
            }
        } else {
            uint32_t now_ms = esp_timer_get_time() / 1000;
            if (pending_clicks == 2) {
                send_event(INPUT_EVENT_BTN_DOUBLE);
                pending_clicks = 0;
            } else if (pending_clicks == 1 && (now_ms - last_release_ms) > 400) {
                send_event(INPUT_EVENT_BTN_CLICK);
                pending_clicks = 0;
            }
        }

        enc_state = (gpio_get_level(GPIO_ENC_A) << 1) | gpio_get_level(GPIO_ENC_B);
        uint8_t transition = (enc_last << 2) | enc_state;
        switch (transition) {
            case 0b0001:
            case 0b0111:
            case 0b1110:
            case 0b1000:
                steps++;
                break;
            case 0b0010:
            case 0b0100:
            case 0b1101:
            case 0b1011:
                steps--;
                break;
            default:
                break;
        }
        enc_last = enc_state;
        if (steps >= 4 || steps <= -4) {
            uint32_t now_ms = esp_timer_get_time() / 1000;
            if (now_ms - last_step_ms > 30) {
                send_event(steps > 0 ? INPUT_EVENT_ENC_RIGHT : INPUT_EVENT_ENC_LEFT);
                last_step_ms = now_ms;
            }
            steps = 0;
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

esp_err_t input_driver_start(void)
{
    if (s_event_queue == NULL) {
        s_event_queue = xQueueCreate(16, sizeof(input_event_t));
        if (s_event_queue == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    gpio_config_t btn_cfg = {
        .pin_bit_mask = BIT64(GPIO_KEY_MAIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    CHECK_ESP_RETURN(gpio_config(&btn_cfg));

    gpio_config_t enc_cfg = {
        .pin_bit_mask = BIT64(GPIO_ENC_A) | BIT64(GPIO_ENC_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    CHECK_ESP_RETURN(gpio_config(&enc_cfg));

    if (s_task_handle == NULL) {
        BaseType_t ret = xTaskCreatePinnedToCore(input_task,
                                                 "input_task",
                                                 3072,
                                                 NULL,
                                                 4,
                                                 &s_task_handle,
                                                 0);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "create input task failed");
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "input driver started");
    return ESP_OK;
}

QueueHandle_t input_event_queue_get(void)
{
    return s_event_queue;
}
