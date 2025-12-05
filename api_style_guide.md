````markdown
# api_style_guide.md  
ESP32-S3 三合一设备 —— API 风格与代码模板指南

> 目的：约束本项目的代码风格，并为 AI 生成代码提供统一的“模板”。  
> 要点：**注释全部中文，标识符使用 lower_snake_case，统一使用新 I2C API，统一错误处理宏，统一 DataModel 与 UI 写法。**

---

## 0. 通用规范

### 0.1 命名与编码

- 源码文件统一使用 **UTF-8** 编码。
- 标识符命名：
  - 函数 / 变量：`lower_snake_case`
  - 宏：`UPPER_SNAKE_CASE`
  - 类型：`xxx_t`
- 注释必须使用**简体中文**，简短清晰。

### 0.2 文件组织

- 每个模块提供一对 `.h/.c` 文件，例如：
  - `i2c_bus.h/.c`
  - `imu_lsm6dsr.h/.c`
  - `data_model.h/.c`
  - `ui_bike.h/.c`
- 所有头文件使用：

```c
#pragma once
````

---

## 1. I2C 新驱动模板（ESP-IDF v6+）

> 严禁使用 `driver/i2c.h` 和 `i2c_master_cmd_begin` 等旧 API。
> 必须使用 `driver/i2c_master.h` 提供的新接口。

### 1.1 I2C 总线初始化模板（i2c_bus.c）

```c
// i2c_bus.c
// 说明：I2C 主机总线初始化与获取接口

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "app_check.h"  // 包含 CHECK_ESP_RETURN 宏

static const char *TAG = "i2c_bus";

static i2c_master_bus_handle_t s_i2c_bus = NULL;

// 函数说明：初始化 I2C 主机总线，仅初始化一次
esp_err_t i2c_bus_init(void)
{
    if (s_i2c_bus != NULL) {
        // 已经初始化过，直接返回成功
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT, // 使用默认时钟源
        .i2c_port   = I2C_NUM_0,           // 使用 I2C0
        .scl_io_num = 39,                  // SCL 引脚
        .sda_io_num = 40,                  // SDA 引脚
        .glitch_ignore_cnt = 7,            // 忽略短毛刺
        .flags.enable_internal_pullup = true, // 使能内部上拉
    };

    // 创建 I2C 主机总线
    CHECK_ESP_RETURN(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));

    ESP_LOGI(TAG, "i2c bus init ok");
    return ESP_OK;
}

// 函数说明：获取 I2C 总线句柄
i2c_master_bus_handle_t i2c_bus_get(void)
{
    return s_i2c_bus;
}
```

### 1.2 I2C 总线头文件模板（i2c_bus.h）

```c
// i2c_bus.h
// 说明：I2C 总线对外接口

#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

// 函数说明：初始化 I2C 主机总线
esp_err_t i2c_bus_init(void);

// 函数说明：获取 I2C 主机总线句柄
i2c_master_bus_handle_t i2c_bus_get(void);
```

### 1.3 设备初始化模板（以 LSM6DSR 为例）

```c
// imu_lsm6dsr.c
// 说明：LSM6DSR IMU 驱动示例

#include "imu_lsm6dsr.h"
#include "i2c_bus.h"
#include "app_check.h"
#include "esp_log.h"

#define LSM6DSR_I2C_ADDR  0x6A

static const char *TAG = "imu_lsm6dsr";

static i2c_master_dev_handle_t s_imu_dev = NULL;

// 函数说明：初始化 LSM6DSR 设备
esp_err_t imu_lsm6dsr_init(void)
{
    CHECK_ESP_RETURN(i2c_bus_init());

    i2c_master_bus_handle_t bus = i2c_bus_get();
    i2c_device_config_t dev_cfg = {
        .device_address = LSM6DSR_I2C_ADDR,
        .scl_speed_hz   = 400000, // 400kHz
    };

    // 添加 IMU 设备到 I2C 总线
    CHECK_ESP_RETURN(i2c_master_bus_add_device(bus, &dev_cfg, &s_imu_dev));

    // 在此处完成 LSM6DSR 的寄存器配置，如 ODR、量程等
    // ...

    ESP_LOGI(TAG, "lsm6dsr init ok");
    return ESP_OK;
}
```

### 1.4 设备读写寄存器模板

```c
// 函数说明：向 IMU 写入一个 8 位寄存器
static esp_err_t imu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };

    // 使用新 I2C API 写入数据
    CHECK_ESP_RETURN(i2c_master_transmit(s_imu_dev, buf, sizeof(buf), 1000));
    return ESP_OK;
}

// 函数说明：从 IMU 读取一个 8 位寄存器
static esp_err_t imu_read_reg(uint8_t reg, uint8_t *value)
{
    // 先写寄存器地址，再读回 1 字节数据
    CHECK_ESP_RETURN(i2c_master_transmit_receive(s_imu_dev,
                                                 &reg, 1,
                                                 value, 1,
                                                 1000));
    return ESP_OK;
}

// 函数说明：连续读取多个寄存器
static esp_err_t imu_read_regs(uint8_t start_reg, uint8_t *buf, size_t len)
{
    CHECK_ESP_RETURN(i2c_master_transmit_receive(s_imu_dev,
                                                 &start_reg, 1,
                                                 buf, len,
                                                 1000));
    return ESP_OK;
}
```

---

## 2. 错误处理宏使用示例（app_check.h）

> 所有 ESP-IDF API 调用必须使用统一错误处理宏，禁止裸奔。

```c
// app_check.h

#pragma once

#include "esp_err.h"
#include "esp_log.h"

// 宏说明：执行 expr，若失败则打印错误并返回
#define CHECK_ESP_RETURN(expr)                                   \
    do {                                                         \
        esp_err_t _err = (expr);                                 \
        if (_err != ESP_OK) {                                    \
            ESP_LOGE("CHECK", "ESP err %d at %s:%d",             \
                     _err, __FILE__, __LINE__);                  \
            return _err;                                         \
        }                                                        \
    } while (0)

// 宏说明：执行 expr，若失败则打印错误并跳转到 label
#define CHECK_ESP_GOTO(expr, label)                              \
    do {                                                         \
        esp_err_t _err = (expr);                                 \
        if (_err != ESP_OK) {                                    \
            ESP_LOGE("CHECK", "ESP err %d at %s:%d",             \
                     _err, __FILE__, __LINE__);                  \
            goto label;                                          \
        }                                                        \
    } while (0)
```

使用示例：

```c
esp_err_t some_module_init(void)
{
    CHECK_ESP_RETURN(uart_driver_install(...));
    CHECK_ESP_RETURN(uart_param_config(...));
    // 更多初始化...
    return ESP_OK;
}
```

---

## 3. DataModel 使用模板

> 所有跨模块共享的运行状态统一通过 `app_data_model_t` 传递，禁止在 UI 中直接访问其他模块的全局变量。

### 3.1 DataModel 头文件模板（data_model.h）

```c
// data_model.h
// 说明：全局数据模型定义与访问接口

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "gnss_types.h"
#include "sensor_types.h"

typedef enum {
    MODE_MAIN_MENU = 0,
    MODE_BIKE,
    MODE_GPX,
    MODE_PGEAR,
} system_mode_t;

typedef struct {
    float   speed_kmh;         // 当前速度
    float   avg_speed_kmh;     // 平均速度
    float   max_speed_kmh;     // 最大速度
    float   trip_distance_km;  // 本次里程
    uint32_t ride_time_s;      // 本次骑行时间
    uint16_t lap_count;        // Lap 数量
} bike_runtime_t;

typedef struct {
    bool     recording;        // 是否正在记录
    bool     paused;           // 是否暂停
    char     current_file[32]; // 当前 GPX 文件名
    float    distance_km;      // 当前轨迹距离
} gpx_runtime_t;

typedef struct {
    float    vbat;             // 电池电压
    bool     charging;         // 是否正在充电
    uint8_t  batt_level;       // 电池百分比
} power_status_t;

typedef struct {
    gnss_fix_t        gnss;
    sensor_sample_t   sensor;
    bike_runtime_t    bike;
    gpx_runtime_t     gpx;
    power_status_t    power;
    system_mode_t     mode;
    uint32_t          last_update_ms;
} app_data_model_t;

// 函数说明：初始化全局数据模型
void data_model_init(void);

// 函数说明：获取全局数据模型指针
app_data_model_t *data_model_get(void);
```

### 3.2 DataModel 源文件模板（data_model.c）

```c
// data_model.c

#include "data_model.h"
#include <string.h>

// 使用静态全局变量存储 DataModel
static app_data_model_t s_data_model;

void data_model_init(void)
{
    memset(&s_data_model, 0, sizeof(s_data_model));
    s_data_model.mode = MODE_MAIN_MENU;
}

app_data_model_t *data_model_get(void)
{
    return &s_data_model;
}
```

### 3.3 业务模块写 DataModel 示例

```c
// 例如在 SENSOR_TASK 中更新传感器字段

#include "data_model.h"

static void sensor_task_loop(void)
{
    app_data_model_t *m = data_model_get();

    while (1) {
        sensor_sample_t sample = {0};
        // 读取传感器数据
        sensor_read_all(&sample);

        // 将数据写入 DataModel
        m->sensor = sample;
        m->last_update_ms = sample.timestamp_ms;

        // 延时或阻塞等待下一次采样
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

---

## 4. LVGL UI 模板（Create / Update 分离）

> 规则：
>
> * `ui_xxx_create()`：只负责创建/布局控件，不访问 DataModel；
> * `ui_xxx_update()`：只负责从 DataModel 读取数据，更新界面文本。

### 4.1 头文件模板（ui_bike.h）

```c
// ui_bike.h
// 说明：Bike 模式 UI 接口

#pragma once

#include "lvgl.h"
#include "data_model.h"

// 函数说明：创建 Bike 主页面控件
void ui_bike_main_create(lv_obj_t *parent);

// 函数说明：根据 DataModel 更新 Bike 主页面显示
void ui_bike_main_update(const app_data_model_t *m);
```

### 4.2 源文件模板（ui_bike.c）

```c
// ui_bike.c

#include "ui_bike.h"
#include <stdio.h>

// 静态指针保存控件句柄
static lv_obj_t *s_label_speed = NULL;
static lv_obj_t *s_label_dist  = NULL;

// 函数说明：创建 Bike 主页面控件，仅调用一次
void ui_bike_main_create(lv_obj_t *parent)
{
    // 创建速度标签
    s_label_speed = lv_label_create(parent);
    lv_obj_align(s_label_speed, LV_ALIGN_TOP_MID, 0, 10);
    lv_label_set_text(s_label_speed, "0.0 km/h");

    // 创建距离标签
    s_label_dist = lv_label_create(parent);
    lv_obj_align(s_label_dist, LV_ALIGN_TOP_MID, 0, 40);
    lv_label_set_text(s_label_dist, "0.00 km");
}

// 函数说明：根据 DataModel 更新 Bike 主页面显示
void ui_bike_main_update(const app_data_model_t *m)
{
    char buf[32];

    // 更新速度
    snprintf(buf, sizeof(buf), "%.1f km/h", m->bike.speed_kmh);
    lv_label_set_text(s_label_speed, buf);

    // 更新距离
    snprintf(buf, sizeof(buf), "%.2f km", m->bike.trip_distance_km);
    lv_label_set_text(s_label_dist, buf);
}
```

---

## 5. 状态机函数表模板（避免巨大 switch）

> 对于复杂状态机（如 P-GEAR、GPX 记录等），采用“状态处理函数表 + 函数指针”模式。

### 5.1 状态机头文件模板（pgear_fsm.h）

```c
// pgear_fsm.h
// 说明：P-GEAR 状态机接口

#pragma once

#include "data_model.h"

typedef enum {
    PGEAR_IDLE = 0,
    PGEAR_ARMED,
    PGEAR_RUNNING,
    PGEAR_FINISHED,
    PGEAR_STATE_MAX
} pgear_state_t;

typedef enum {
    PGEAR_EVENT_GNSS_UPDATE = 0,
    PGEAR_EVENT_BTN_CLICK,
    PGEAR_EVENT_BTN_DOUBLE,
    // ...其它事件类型
} pgear_event_type_t;

typedef struct {
    pgear_event_type_t type;
    // 如有需要可添加参数字段
} pgear_event_t;

// 函数说明：处理一次 P-GEAR 事件
void pgear_process_event(app_data_model_t *m,
                         const pgear_event_t *ev);
```

### 5.2 状态机源文件模板（pgear_fsm.c）

```c
// pgear_fsm.c

#include "pgear_fsm.h"

// 定义状态处理函数类型
typedef void (*pgear_state_handler_t)(app_data_model_t *m,
                                      const pgear_event_t *ev);

// 各状态处理函数前向声明
static void pgear_handle_idle(app_data_model_t *m,
                              const pgear_event_t *ev);
static void pgear_handle_armed(app_data_model_t *m,
                               const pgear_event_t *ev);
static void pgear_handle_running(app_data_model_t *m,
                                 const pgear_event_t *ev);
static void pgear_handle_finished(app_data_model_t *m,
                                  const pgear_event_t *ev);

// 状态处理函数表
static const pgear_state_handler_t s_pgear_table[PGEAR_STATE_MAX] = {
    [PGEAR_IDLE]     = pgear_handle_idle,
    [PGEAR_ARMED]    = pgear_handle_armed,
    [PGEAR_RUNNING]  = pgear_handle_running,
    [PGEAR_FINISHED] = pgear_handle_finished,
};

// 函数说明：对外统一事件处理接口
void pgear_process_event(app_data_model_t *m,
                         const pgear_event_t *ev)
{
    pgear_state_t s = m->pgear.state;
    if (s < PGEAR_STATE_MAX && s_pgear_table[s] != NULL) {
        s_pgear_table[s](m, ev);
    }
}

// 以下为各状态具体逻辑（仅示例，实际按 SRS 实现）

static void pgear_handle_idle(app_data_model_t *m,
                              const pgear_event_t *ev)
{
    // 在 IDLE 状态处理各种事件，例如判断是否进入 ARMED
    // ...
}

static void pgear_handle_armed(app_data_model_t *m,
                               const pgear_event_t *ev)
{
    // 在 ARMED 状态处理事件，例如根据加速度/GNSS 进入 RUNNING
    // ...
}

static void pgear_handle_running(app_data_model_t *m,
                                 const pgear_event_t *ev)
{
    // 在 RUNNING 状态处理事件，例如达到目标速度进入 FINISHED
    // ...
}

static void pgear_handle_finished(app_data_model_t *m,
                                  const pgear_event_t *ev)
{
    // 在 FINISHED 状态处理事件，例如按键清除结果回到 IDLE
    // ...
}
```

---

## 6. 日志与心跳模板

### 6.1 事件日志输出示例

```c
// 在输入事件处理后输出日志
#include "esp_log.h"

static const char *TAG = "input";

static void input_report_event(const char *type)
{
    app_data_model_t *m = data_model_get();
    uint32_t t = m->sensor.timestamp_ms; // 或其它时间

    // 说明：此处仅打印文本日志，实际可同时写入 SD 日志缓冲
    ESP_LOGI(TAG, "[EVT] t=%ums mode=%d type=%s",
             (unsigned)t, m->mode, type);
}
```

### 6.2 心跳日志输出示例（简化）

```c
// heartbeat.c

#include "data_model.h"
#include "esp_log.h"

static const char *TAG = "heartbeat";

void heartbeat_task(void *arg)
{
    while (1) {
        app_data_model_t *m = data_model_get();

        ESP_LOGI(TAG,
                 "[HB] t=%ums mode=%d "
                 "gnss_fix=%d sats=%u "
                 "vbat=%.2fV",
                 (unsigned)m->sensor.timestamp_ms,
                 m->mode,
                 m->gnss.fix,
                 m->gnss.sats,
                 m->power.vbat);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
```

---

## 7. 总结

* **I2C：** 统一使用 `i2c_master_*` 新 API + `CHECK_ESP_*` 宏。
* **DataModel：** 业务模块写、UI/日志读，禁止跨模块直接互相访问静态全局。
* **UI：** 严格 `create` / `update` 分离，事件在 SYS 层处理。
* **状态机：** 复杂逻辑使用“函数表 + 状态处理函数”模式，避免巨大 `switch`。

在本项目中，如果 AI 生成的代码与本文件模板不一致，应优先参考本文件进行修改与重构。

```
::contentReference[oaicite:0]{index=0}
```
