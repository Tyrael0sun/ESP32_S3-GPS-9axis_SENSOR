#include "encoder.h"

#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define ENCODER_PIN_A               1
#define ENCODER_PIN_B               3
#define ENCODER_PIN_BUTTON          2
#define ENCODER_BUTTON_ACTIVE_LEVEL 0

#define ENCODER_PCNT_UNIT           PCNT_UNIT_0
#define ENCODER_TASK_STACK          4096
#define ENCODER_TASK_PRIORITY       5

#define ENCODER_DEBOUNCE_MS         10
#define ENCODER_COUNTS_PER_STEP     4
#define ENCODER_FILTER_THRESHOLD    250

static const char *TAG = "ENC";

typedef struct {
    encoder_button_event_t event;
} encoder_button_message_t;

static TaskHandle_t s_monitor_task;
static QueueHandle_t s_button_queue;
static portMUX_TYPE s_count_lock = portMUX_INITIALIZER_UNLOCKED;
static int32_t s_position;
static int32_t s_pending_counts;
static encoder_button_event_t s_last_button_event = ENCODER_BUTTON_NONE;
static int64_t s_button_pressed_us;

static void IRAM_ATTR encoder_button_isr(void *arg)
{
    const int level = gpio_get_level(ENCODER_PIN_BUTTON);
    const int64_t now = esp_timer_get_time();

    if (level == ENCODER_BUTTON_ACTIVE_LEVEL) {
        s_button_pressed_us = now;
        return;
    }

    if (!s_button_pressed_us) {
        return;
    }

    int64_t duration_us = now - s_button_pressed_us;
    s_button_pressed_us = 0;

    encoder_button_event_t event = ENCODER_BUTTON_SHORT;
    if (duration_us >= 8 * 1000 * 1000LL) {
        event = ENCODER_BUTTON_LONG;
    } else if (duration_us >= 3 * 1000 * 1000LL) {
        event = ENCODER_BUTTON_MEDIUM;
    }

    encoder_button_message_t msg = {
        .event = event,
    };

    BaseType_t high_task_woken = pdFALSE;
    if (s_button_queue) {
        xQueueSendFromISR(s_button_queue, &msg, &high_task_woken);
    }

    if (high_task_woken) {
        portYIELD_FROM_ISR();
    }
}

static void encoder_monitor_task(void *arg)
{
    int16_t counter_value = 0;

    while (true) {
        if (pcnt_get_counter_value(ENCODER_PCNT_UNIT, &counter_value) == ESP_OK && counter_value != 0) {
            pcnt_counter_clear(ENCODER_PCNT_UNIT);
            portENTER_CRITICAL(&s_count_lock);
            s_pending_counts += counter_value;
            int32_t steps = s_pending_counts / ENCODER_COUNTS_PER_STEP;
            if (steps != 0) {
                s_pending_counts -= steps * ENCODER_COUNTS_PER_STEP;
                s_position -= steps;
            }
            const int32_t position = s_position;
            const int32_t pending = s_pending_counts;
            portEXIT_CRITICAL(&s_count_lock);
            if (steps != 0) {
                ESP_LOGI(TAG, "encoder steps=%ld pending=%ld position=%ld", (long)(-steps), (long)pending, (long)position);
            }
        }

        encoder_button_message_t msg = {0};
        if (s_button_queue && xQueueReceive(s_button_queue, &msg, pdMS_TO_TICKS(5)) == pdTRUE) {
            s_last_button_event = msg.event;
            const char *label = "short";
            if (msg.event == ENCODER_BUTTON_MEDIUM) {
                label = "medium";
            } else if (msg.event == ENCODER_BUTTON_LONG) {
                label = "long";
            }
            ESP_LOGI(TAG, "button %s press", label);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static esp_err_t encoder_configure_pcnt(void)
{
    pcnt_config_t pcnt_cfg = {
        .pulse_gpio_num = ENCODER_PIN_A,
        .ctrl_gpio_num = ENCODER_PIN_B,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = 1000,
        .counter_l_lim = -1000,
        .unit = ENCODER_PCNT_UNIT,
        .channel = PCNT_CHANNEL_0,
    };

    ESP_RETURN_ON_ERROR(pcnt_unit_config(&pcnt_cfg), TAG, "PCNT ch0 config failed");

    pcnt_cfg.pulse_gpio_num = ENCODER_PIN_B;
    pcnt_cfg.ctrl_gpio_num = ENCODER_PIN_A;
    pcnt_cfg.pos_mode = PCNT_COUNT_DEC;
    pcnt_cfg.neg_mode = PCNT_COUNT_INC;
    pcnt_cfg.channel = PCNT_CHANNEL_1;

    ESP_RETURN_ON_ERROR(pcnt_unit_config(&pcnt_cfg), TAG, "PCNT ch1 config failed");

    ESP_RETURN_ON_ERROR(pcnt_set_filter_value(ENCODER_PCNT_UNIT, ENCODER_FILTER_THRESHOLD), TAG, "PCNT filter value failed");
    ESP_RETURN_ON_ERROR(pcnt_filter_enable(ENCODER_PCNT_UNIT), TAG, "PCNT filter enable failed");
    ESP_RETURN_ON_ERROR(pcnt_counter_pause(ENCODER_PCNT_UNIT), TAG, "PCNT pause failed");
    ESP_RETURN_ON_ERROR(pcnt_counter_clear(ENCODER_PCNT_UNIT), TAG, "PCNT clear failed");
    ESP_RETURN_ON_ERROR(pcnt_counter_resume(ENCODER_PCNT_UNIT), TAG, "PCNT resume failed");

    return ESP_OK;
}

static esp_err_t encoder_configure_button(void)
{
    gpio_config_t btn_cfg = {
        .pin_bit_mask = BIT64(ENCODER_PIN_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&btn_cfg), TAG, "Button config failed");

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(ENCODER_PIN_BUTTON, encoder_button_isr, NULL),
                        TAG, "Button ISR add failed");
    return ESP_OK;
}

esp_err_t encoder_init(void)
{
    if (s_monitor_task) {
        return ESP_OK;
    }

    s_button_queue = xQueueCreate(4, sizeof(encoder_button_message_t));
    if (!s_button_queue) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(encoder_configure_pcnt(), TAG, "PCNT setup failed");
    ESP_RETURN_ON_ERROR(encoder_configure_button(), TAG, "Button setup failed");

    BaseType_t created = xTaskCreatePinnedToCore(encoder_monitor_task, "encoder", ENCODER_TASK_STACK, NULL,
                                                ENCODER_TASK_PRIORITY, &s_monitor_task, 0);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Encoder ready");
    return ESP_OK;
}

int32_t encoder_get_count(void)
{
    portENTER_CRITICAL(&s_count_lock);
    int32_t value = s_position;
    portEXIT_CRITICAL(&s_count_lock);
    return value;
}

void encoder_reset_count(void)
{
    portENTER_CRITICAL(&s_count_lock);
    s_position = 0;
    s_pending_counts = 0;
    portEXIT_CRITICAL(&s_count_lock);
    pcnt_counter_clear(ENCODER_PCNT_UNIT);
}

encoder_button_event_t encoder_get_last_button_event(void)
{
    portENTER_CRITICAL(&s_count_lock);
    encoder_button_event_t event = s_last_button_event;
    s_last_button_event = ENCODER_BUTTON_NONE;
    portEXIT_CRITICAL(&s_count_lock);
    return event;
}
