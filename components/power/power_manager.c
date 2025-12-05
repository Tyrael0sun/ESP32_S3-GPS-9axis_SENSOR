// power_manager.c
// 说明：电池电压采样与充电状态监测

#include "power_manager.h"
#include "data_model.h"
#include "macro_def.h"
#include "app_check.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "power";

static TaskHandle_t s_task_handle = NULL;
static adc_oneshot_unit_handle_t s_adc_unit;
static adc_cali_handle_t s_adc_cali;
static adc_channel_t s_adc_channel;

static esp_err_t adc_setup(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    CHECK_ESP_RETURN(adc_oneshot_new_unit(&unit_cfg, &s_adc_unit));

    CHECK_ESP_RETURN(adc_oneshot_io_to_channel(GPIO_BAT_ADC,
                                               &unit_cfg.unit_id,
                                               &s_adc_channel));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    CHECK_ESP_RETURN(adc_oneshot_config_channel(s_adc_unit,
                                                s_adc_channel,
                                                &chan_cfg));

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_2,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali) != ESP_OK) {
        ESP_LOGW(TAG, "adc calibration unavailable");
        s_adc_cali = NULL;
    }

    gpio_config_t chg_cfg = {
        .pin_bit_mask = BIT64(GPIO_CHRG_STATUS),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    CHECK_ESP_RETURN(gpio_config(&chg_cfg));

    return ESP_OK;
}

static float sample_voltage(void)
{
    int raw = 0;
    if (adc_oneshot_read(s_adc_unit, s_adc_channel, &raw) != ESP_OK) {
        return 0.0f;
    }

    int mv = 0;
    if (s_adc_cali) {
        adc_cali_raw_to_voltage(s_adc_cali, raw, &mv);
    } else {
        mv = raw * 3300 / 4095;
    }

    float v = mv / 1000.0f;
    return v * 2.0f;
}

static void power_task(void *arg)
{
    app_data_model_t *m = data_model_get();
    const TickType_t period = pdMS_TO_TICKS(200);

    float v_avg = 0.0f;
    const float alpha = 0.1f;

    while (1) {
        float v = sample_voltage();
        v_avg = (1.0f - alpha) * v_avg + alpha * v;

        m->power.vbat = v_avg;
        float percent = (v_avg - 3.3f) / (4.2f - 3.3f);
        m->power.batt_level = (uint8_t)(CLAMP(percent, 0.0f, 1.0f) * 100.0f);
        m->power.charging = (gpio_get_level(GPIO_CHRG_STATUS) == 0);

        vTaskDelay(period);
    }
}

esp_err_t power_manager_start(void)
{
    CHECK_ESP_RETURN(adc_setup());

    if (s_task_handle == NULL) {
        BaseType_t ret = xTaskCreatePinnedToCore(power_task,
                                                 "power_task",
                                                 2048,
                                                 NULL,
                                                 4,
                                                 &s_task_handle,
                                                 0);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "create power task failed");
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "power manager started");
    return ESP_OK;
}
