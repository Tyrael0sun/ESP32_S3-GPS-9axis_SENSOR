// data_model.c
// 说明：全局数据模型实现

#include "data_model.h"
#include <string.h>
#include <math.h>

// 使用静态全局变量存储 DataModel
static app_data_model_t s_data_model;

void data_model_init(void)
{
    memset(&s_data_model, 0, sizeof(s_data_model));
    
    // 初始化默认值
    s_data_model.mode = MODE_MAIN_MENU;
    
    // GNSS 初始值
    s_data_model.gnss.fix = GNSS_FIX_NONE;
    s_data_model.gnss.valid = false;
    
    // 传感器初始值
    s_data_model.sensor.imu_temp_c = NAN;
    s_data_model.sensor.mag_temp_c = NAN;
    s_data_model.sensor.baro_temp_c = NAN;
    s_data_model.sensor.q0 = 1.0f;  // 单位四元数
    s_data_model.sensor.baro_p0 = 101325.0f;
    
    // GPX 状态
    s_data_model.gpx.state = GPX_IDLE;
    
    // P-GEAR 状态
    s_data_model.pgear.state = PGEAR_IDLE;
    s_data_model.pgear.v_start = 0.0f;
    s_data_model.pgear.v_end = 100.0f;
    s_data_model.pgear.g_trigger = 0.3f;
    s_data_model.pgear.best_time_s = 999.9f;
    
    // SD 卡状态
    s_data_model.sd.mounted = false;
    s_data_model.sd.last_err = 0;

    s_data_model.rtc.unix_time = 0;
    s_data_model.rtc.synced = false;

    s_data_model.auto_ctrl.pause_threshold_kmh = 3.0f;
    s_data_model.auto_ctrl.resume_threshold_kmh = 5.0f;
    s_data_model.auto_ctrl.pause_delay_s = 5;

    s_data_model.ui.main_menu_index = 0;
    s_data_model.ui.settings_index = 0;
    s_data_model.ui.settings_origin = MODE_MAIN_MENU;
}

app_data_model_t *data_model_get(void)
{
    return &s_data_model;
}
