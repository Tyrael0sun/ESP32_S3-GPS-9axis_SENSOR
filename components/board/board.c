#include "board.h"

#include <math.h>

#include <driver/gpio.h>
#include <esp_check.h>
#include <esp_idf_version.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#else
#include <driver/adc.h>
#include <esp_adc_cal.h>
#endif

#define GPIO_GNSS_LDO_EN    GPIO_NUM_14
#define GPIO_CHRG_STATUS    GPIO_NUM_21
#define BATTERY_DIVIDER     2.0f
#define BATTERY_SAMPLES     8
#define BATTERY_SAMPLE_US   500

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#define BOARD_ADC_UNIT      ADC_UNIT_2
#define BOARD_ADC_CHANNEL   ADC_CHANNEL_1
#else
#define BOARD_ADC_CHANNEL   ADC2_CHANNEL_1
#endif

static const char *TAG = "BOARD";

static SemaphoreHandle_t s_lock;
static board_state_t s_state = {
    .battery_voltage = 0.0f,
    .charging = false,
};

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static adc_oneshot_unit_handle_t s_adc_unit;
static adc_cali_handle_t s_adc_cali;
static bool s_adc_cali_enabled;
#else
static esp_adc_cal_characteristics_t s_adc_chars;
static bool s_adc_calibrated;
#endif

static esp_err_t board_configure_gpio(void)
{
    gpio_config_t ldo_cfg = {
        .pin_bit_mask = BIT64(GPIO_GNSS_LDO_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&ldo_cfg), TAG, "LDO config failed");
    gpio_set_level(GPIO_GNSS_LDO_EN, 1);

    gpio_config_t charge_cfg = {
        .pin_bit_mask = BIT64(GPIO_CHRG_STATUS),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&charge_cfg), TAG, "Charge status config failed");

    return ESP_OK;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static esp_err_t board_configure_adc(void)
{
    if (s_adc_unit) {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = BOARD_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc_unit), TAG, "ADC unit init failed");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc_unit, BOARD_ADC_CHANNEL, &chan_cfg), TAG, "ADC chan config failed");

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = BOARD_ADC_UNIT,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali) == ESP_OK) {
        s_adc_cali_enabled = true;
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = BOARD_ADC_UNIT,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc_cali) == ESP_OK) {
        s_adc_cali_enabled = true;
    }
#endif

    return ESP_OK;
}

static esp_err_t board_sample_battery(float *voltage_v)
{
    int samples = 0;
    int64_t accum_mv = 0;

    for (int i = 0; i < BATTERY_SAMPLES; ++i) {
        int raw = 0;
        if (adc_oneshot_read(s_adc_unit, BOARD_ADC_CHANNEL, &raw) != ESP_OK) {
            continue;
        }
        int sample_mv = (int)((raw * 3300.0f) / 4095.0f);
        if (s_adc_cali_enabled && s_adc_cali) {
            if (adc_cali_raw_to_voltage(s_adc_cali, raw, &sample_mv) != ESP_OK) {
                sample_mv = (int)((raw * 3300.0f) / 4095.0f);
            }
        }
        accum_mv += sample_mv;
        samples++;
        esp_rom_delay_us(BATTERY_SAMPLE_US);
    }

    if (!samples) {
        return ESP_FAIL;
    }

    float average_mv = (float)accum_mv / (float)samples;
    *voltage_v = (average_mv / 1000.0f) * BATTERY_DIVIDER;
    return ESP_OK;
}

#else

static esp_err_t board_configure_adc(void)
{
    ESP_RETURN_ON_ERROR(adc2_config_channel_atten(BOARD_ADC_CHANNEL, ADC_ATTEN_DB_11), TAG, "ADC atten failed");
    esp_adc_cal_value_t cal_type = esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &s_adc_chars);
    s_adc_calibrated = cal_type != ESP_ADC_CAL_VAL_NOT_SUPPORTED;
    return ESP_OK;
}

static esp_err_t board_sample_battery(float *voltage_v)
{
    int samples = 0;
    int64_t accum_mv = 0;

    for (int i = 0; i < BATTERY_SAMPLES; ++i) {
        int raw = 0;
        if (adc2_get_raw(BOARD_ADC_CHANNEL, ADC_WIDTH_BIT_12, &raw) != ESP_OK) {
            continue;
        }
        uint32_t sample_mv = (uint32_t)((raw * 3300UL) / 4095UL);
        if (s_adc_calibrated) {
            sample_mv = esp_adc_cal_raw_to_voltage(raw, &s_adc_chars);
        }
        accum_mv += sample_mv;
        samples++;
        esp_rom_delay_us(BATTERY_SAMPLE_US);
    }

    if (!samples) {
        return ESP_FAIL;
    }

    float average_mv = (float)accum_mv / (float)samples;
    *voltage_v = (average_mv / 1000.0f) * BATTERY_DIVIDER;
    return ESP_OK;
}

#endif

static void board_refresh_locked(void)
{
    float voltage = s_state.battery_voltage;
    if (board_sample_battery(&voltage) == ESP_OK) {
        s_state.battery_voltage = voltage;
    }
    s_state.charging = gpio_get_level(GPIO_CHRG_STATUS) == 0;
}

esp_err_t board_init(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        if (!s_lock) {
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_RETURN_ON_ERROR(board_configure_gpio(), TAG, "GPIO config failed");
    ESP_RETURN_ON_ERROR(board_configure_adc(), TAG, "ADC init failed");

    xSemaphoreTake(s_lock, portMAX_DELAY);
    board_refresh_locked();
    xSemaphoreGive(s_lock);

    ESP_LOGI(TAG, "Board peripherals ready");
    return ESP_OK;
}

void board_read_state(board_state_t *state)
{
    if (!state) {
        return;
    }

    if (!s_lock) {
        *state = s_state;
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    board_refresh_locked();
    *state = s_state;
    xSemaphoreGive(s_lock);
}
