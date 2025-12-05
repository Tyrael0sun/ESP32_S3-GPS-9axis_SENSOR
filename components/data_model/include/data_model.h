// data_model.h
// 说明：全局数据模型定义与访问接口

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <time.h>

// ========== 系统模式定义 ==========
typedef enum {
    MODE_MAIN_MENU = 0,
    MODE_BIKE,
    MODE_GPX,
    MODE_PGEAR,
    MODE_SETTINGS,
} system_mode_t;

// ========== GNSS 数据结构 ==========
typedef enum {
    GNSS_FIX_NONE = 0,
    GNSS_FIX_2D,
    GNSS_FIX_3D,
} gnss_fix_type_t;

typedef struct {
    uint32_t timestamp_ms;   // 自上电以来的毫秒计时
    gnss_fix_type_t fix;     // 定位类型
    double   lat;            // 纬度，度
    double   lon;            // 经度，度
    float    altitude;       // 海拔，m
    float    speed;          // 水平速度，m/s
    float    course;         // 航向，度 (0-360)
    float    hdop;           // 水平精度因子
    float    vdop;           // 垂直精度因子
    float    pdop;           // 位置精度因子
    uint8_t  sats;           // 使用的卫星数
    uint8_t  sats_visible;   // 可见卫星数
    bool     valid;          // 定位是否有效
    uint8_t  nmea_ok;        // NMEA 消息接收状态 (0/1)
    bool     time_synced;    // 软件 RTC 是否已被 GNSS 同步
} gnss_fix_t;

typedef struct {
    time_t  unix_time;       // 当前 UTC unix 时间戳
    bool    synced;          // 是否已被 GNSS 同步
} rtc_status_t;

// ========== 传感器数据结构 ==========
typedef struct {
    uint32_t timestamp_ms;

    // IMU 原始数据（车体坐标系）
    float ax, ay, az;        // 加速度，m/s²
    float gx, gy, gz;        // 角速度，dps
    float imu_temp_c;        // IMU 温度，°C

    // Mahony 姿态解算输出
    float q0, q1, q2, q3;    // 姿态四元数
    float roll, pitch, yaw;  // 欧拉角，度

    // 重力向量（车体坐标系）
    float gx_grav, gy_grav, gz_grav;  // m/s²

    // 线性加速度（去重力）
    float ax_lin, ay_lin, az_lin;     // m/s²

    // 磁力计（已校准）
    float mx, my, mz;        // µT
    float mag_temp_c;        // 磁力计温度，°C

    // 气压计
    float pressure;          // Pa
    float altitude;          // m
    float baro_temp_c;       // 气压计温度，°C
    float baro_p0;           // 气压计校准的海平面气压, Pa
} sensor_sample_t;

#define BIKE_MAX_LAPS 999

typedef struct {
    uint16_t seq;             // Lap 序号（从 1 开始）
    float    distance_km;     // 本圈距离
    uint32_t moving_time_s;   // 本圈移动时间
    float    avg_speed_kmh;   // 本圈平均速度
} bike_lap_entry_t;

typedef struct {
    float    pause_threshold_kmh; // 进入自动暂停速度阈值
    float    resume_threshold_kmh; // 离开自动暂停速度阈值
    uint16_t pause_delay_s;       // 自动暂停延迟（秒）
} auto_ctrl_config_t;

// ========== UI 状态 ==========
typedef struct {
    uint8_t main_menu_index;      // 主菜单选中项
    uint8_t settings_index;       // 设置菜单选中项
    system_mode_t settings_origin;// 设置菜单返回目标
} ui_state_t;

// ========== BIKE 运行时数据 ==========
typedef struct {
    float   speed_kmh;          // 当前速度
    float   avg_speed_kmh;      // 平均速度（移动时间）
    float   max_speed_kmh;      // 最大速度
    float   trip_distance_km;   // 本次里程
    float   total_distance_km;  // 累计里程
    uint32_t ride_time_s;       // 本次骑行时间（移动时间）
    uint32_t total_time_s;      // 总时间（含暂停）
    uint16_t lap_count;         // Lap 数量
    float   climb_m;            // 累计爬升
    float   descent_m;          // 累计下降
    float   slope_percent;      // 当前坡度 %
    float   vert_speed_mh;      // 垂直速度 m/h
    bool    auto_paused;        // 是否自动暂停
    bike_lap_entry_t laps[BIKE_MAX_LAPS]; // Lap 列表
} bike_runtime_t;

// ========== GPX 运行时数据 ==========
typedef enum {
    GPX_IDLE = 0,
    GPX_RECORDING,
    GPX_PAUSED,
    GPX_STOPPED,
} gpx_state_t;

typedef struct {
    gpx_state_t state;          // GPX 记录状态
    char     current_file[64];  // 当前 GPX 文件名
    float    distance_km;       // 当前轨迹距离
    uint32_t point_count;       // 记录点数
    uint32_t file_count;        // GPX 文件总数
} gpx_runtime_t;

// ========== P-GEAR 运行时数据 ==========
typedef enum {
    PGEAR_IDLE = 0,
    PGEAR_ARMED,
    PGEAR_RUNNING,
    PGEAR_FINISHED,
} pgear_state_t;

typedef struct {
    pgear_state_t state;
    float    v_start;           // 起始速度 km/h
    float    v_end;             // 结束速度 km/h
    float    g_trigger;         // 触发加速度 g
    float    current_speed;     // 当前速度 km/h
    float    current_accel_g;   // 当前加速度 g
    float    elapsed_time_s;    // 本次用时 s
    float    best_time_s;       // 最佳成绩 s
} pgear_runtime_t;

// ========== 电源状态 ==========
typedef struct {
    float    vbat;              // 电池电压 V
    bool     charging;          // 是否正在充电
    uint8_t  batt_level;        // 电池百分比 0-100
} power_status_t;

// ========== SD 卡状态 ==========
typedef struct {
    bool     mounted;           // 是否已挂载
    int      last_err;          // 最后一次错误码
} sd_status_t;

// ========== 全局数据模型 ==========
typedef struct {
    system_mode_t     mode;
    uint32_t          last_update_ms;

    gnss_fix_t        gnss;
    sensor_sample_t   sensor;
    bike_runtime_t    bike;
    gpx_runtime_t     gpx;
    pgear_runtime_t   pgear;
    power_status_t    power;
    sd_status_t       sd;
    rtc_status_t      rtc;
    auto_ctrl_config_t auto_ctrl;
    ui_state_t        ui;
} app_data_model_t;

// 函数说明：初始化全局数据模型
void data_model_init(void);

// 函数说明：获取全局数据模型指针
app_data_model_t *data_model_get(void);
