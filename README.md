# ESP32S3 三合一设备

## 软件需求规格书（SRS）v1.1

---

## 0. 概述

* **设备功能集：**

  1. 自行车码表（Bike Computer）
  2. GPS 轨迹记录仪（GPX Recorder）
  3. P-GEAR 汽车 0–100 加速测试

* **硬件平台：**

  * SoC：ESP32-S3FH4R2（4MB Flash，2MB PSRAM）
  * GNSS：NEO-M8N（UART1）
  * IMU：LSM6DSR（I2C，6 轴）
  * 磁力计：LIS2MDL（I2C）
  * 氣压计：BMP388（I2C）
  * 显示：ST7789 240×320 竖屏（旋转 180°）
  * 存储：SD 卡，4-bit SDIO
  * 输入：旋转编码器 + 单按键
  * 电源相关：电池 ADC，充电状态引脚

* **软件环境：**

  * ESP-IDF v6.1
  * FreeRTOS
  * LVGL（UI 框架）
  * LovyanGFX（显示驱动）

* **风格要求：**

  * 代码语言：C 为主，可封装少量 C++
  * 代码注释：**必须使用中文**
  * 模块分层清晰，接口规范，便于 AI 生成和维护

---

## 1. 硬件资源与外设

### 1.1 GPIO 分配（固定）

| 模块          | 信号                  | GPIO  | 说明                       |
| ----------- | ------------------- | ----- | ------------------------ |
| 调试串口        | DEBUG_TX / DEBUG_RX | 43/44 | UART0@115200             |
| GNSS        | GNSS_TX / GNSS_RX   | 17/18 | UART1，与 NEO-M8N 相连       |
| GNSS 电源     | GPS_LDO_EN          | 14    | 高电平上电                    |
| I2C0        | SCL / SDA           | 39/40 | 400 kHz，总线挂 IMU/MAG/BARO |
| IMU LSM6DSR | I2C 地址              | 0x6A  |                          |
| MAG LIS2MDL | I2C 地址              | 0x1E  |                          |
| BARO BMP388 | I2C 地址              | 0x76  |                          |
| LCD SPI3    | SCK / MOSI          | 5/8   | LovyanGFX 接口             |
|             | CS / DC             | 7/6   | 片选 / 数据命令                |
|             | RST / BL            | 4/9   | 背光 BL 为 PWM 输出           |
| SDIO        | SD_CLK              | 36    | SD 卡时钟                   |
|             | SD_CMD              | 35    | SD 卡命令线                  |
|             | SD_D0               | 37    | SD 数据 0                  |
|             | SD_D1               | 38    | SD 数据 1                  |
|             | SD_D2               | 34    | SD 数据 2                  |
|             | SD_D3               | 33    | SD 数据 3                  |
| 旋转编码器       | ENC_A / ENC_B       | 1/3   | 上拉输入                     |
| 主按键         | KEY_MAIN            | 2     | 上拉输入                     |
| 电池检测        | BAT_ADC             | 12    | 1:1 分压输入（ADC2 通道）        |
| 充电状态        | CHRG_STATUS         | 21    | 输入                       |

> 说明：
>
> * 当前软件设计中 **不启用 WiFi 功能**，避免 ADC2（BAT_ADC 使用 GPIO12）与 WiFi 资源冲突。
> * 电池电压经 1:1 分压直接接入 ADC 引脚时，要求硬件层面保证 ADC 输入电压不超过芯片推荐最大 ADC 输入（约 3.1V，取决于 ADC 衰减配置）。
> * 软件中需明确配置 ADC 衰减模式，并在注释中说明与硬件分压的对应关系。
> * GPIO3 作为 strapping 管脚，外部编码器电路需保证上电时默认电平满足芯片启动配置要求（上拉到稳定电平），但在正常运行阶段可作为普通输入使用。

### 1.2 显示屏

* 控制器：ST7789
* 分辨率：240×320
* 方向：竖屏，软件配置旋转 180°
* 接口：SPI3 + 背光 PWM（GPIO9，2 kHz，默认 50%）

### 1.3 I2C 软件接口要求

* 所有 I2C 主机访问必须使用 **ESP-IDF v6.0 及以上版本提供的新 I2C 总线/设备驱动接口**（头文件 `driver/i2c_master.h`），**禁止**使用旧版遗留接口 `driver/i2c.h`。
* I2C 总线初始化流程必须遵循“总线-设备”模型：
  * 使用 `i2c_new_master_bus()` 创建 I2C 主机总线句柄 `i2c_master_bus_handle_t`；
  * 使用 `i2c_master_bus_add_device()` 为每个外设（IMU/MAG/BARO 等）创建 `i2c_master_dev_handle_t` 设备句柄。
* I2C 读写操作统一使用以下函数族，不得再使用旧函数名：
  * 写：`i2c_master_transmit()`
  * 读：`i2c_master_receive()`
  * 先写后读：`i2c_master_transmit_receive()`
* 若需要探测设备地址或扫描总线，应使用 `i2c_master_probe()` 等新版 API；不得自行构造旧版 `i2c_cmd_handle_t` 链表。
* 新旧驱动 **不能同时使用**，本项目所有 I2C 相关代码必须只包含 `driver/i2c_master.h`（以及必要的 `i2c_types.h`），不得再 include `driver/i2c.h`。

---

## 2. 输入设备与交互逻辑

### 2.1 主按键（KEY_MAIN）事件定义

按键必须支持 5 种事件类型：

| 事件名        | 条件（按下时长）              | 主要用途                     |
| ---------- | --------------------- | ------------------------ |
| 短按（CLICK）  | < 300 ms              | 菜单选择 / 一般确认 / 模式内小菜单     |
| 双击（DOUBLE） | 400 ms 窗口内两次短按        | BIKE：打圈；GPX：打点；P-GEAR：重测 |
| 中按（MID）    | ≈ 1 s（例如 700–1300 ms） | 预留，用于模式内扩展功能             |
| 长按（LONG）   | ≥ 1.5 s 且 < 8 s       | 模式页面中进入设置                |
| 超长按（ULTRA） | ≥ 8 s                 | 系统级功能预留（如恢复出厂/重启等）       |

要求：

* 实现**100 ms 消抖**。
* 时间窗口重叠时，优先级：ULTRA > LONG > DOUBLE > MID > CLICK。
* 为支持双击识别：

  * 当启用双击识别时，单击 CLICK 事件在双击时间窗口（400 ms）结束后才最终确认触发，避免把双击误识别成两次单击。
* 所有按键事件产生时，必须立即输出一条 UART0 事件日志（见心跳与日志章节）。

### 2.2 旋转编码器

* 每 **4 个脉冲计为 1 步**（防止太灵敏）。
* 若两步之间间隔 ≥ 1000 ms，则清零累计脉冲计数，避免极慢/干扰造成误触。
* 功能（大类约定，详细见 2.3）：

  * 在主菜单 / 设置菜单中：上下移动光标。
  * 在 BIKE 模式：在不同数据页之间切换。
  * 在 GPX 模式：切换信息页/文件页。
  * 在 P-GEAR 模式：切换实时/历史等视图。

### 2.3 模式内按键 / 编码器行为总表

#### 2.3.1 主菜单（MAIN_MENU）

* 旋转编码器：

  * 顺时针 / 逆时针：在菜单项之间上下移动光标。
* 主按键（KEY_MAIN）：

  * CLICK（短按）：进入当前高亮菜单项（Bike / GPX / P-GEAR / 设置）。
  * DOUBLE / MID：保留（当前无功能）。
  * LONG：保留（当前无功能，不进入设置）。
  * ULTRA：保留，用于未来系统级功能（如恢复出厂 / 强制重启）。

> 说明：主菜单中进入“设置”仅通过“光标选中设置 + 短按”实现，长按不进入设置，避免与模式内长按语义混淆。

---

#### 2.3.2 自行车码表模式（MODE_BIKE）

* 旋转编码器：

  * 在 BIKE 数据页内：在“主数据页 / 扩展数据页 / 其他页（如 Lap 列表）”之间循环切换。
* 主按键：

  * CLICK（短按）：在当前模式内弹出 / 关闭 BIKE 子菜单（BIKE_MENU），子菜单中至少包含：

    * 返回数据页
    * 查看 Lap 列表
    * 清除本次骑行数据
    * （可选）返回主菜单
  * DOUBLE：打圈（Lap++），不改变当前显示页面。
  * MID：保留（可用于未来 BIKE 内部的快速切换功能）。
  * LONG：进入设置页面 `SETTINGS_IN_MODE`，并记录来源为 BIKE。
  * ULTRA：保留，用于未来紧急操作。

---

#### 2.3.3 GPX 轨迹记录模式（MODE_GPX）

* 旋转编码器：

  * 在 GPX 主界面内切换信息页（例如“状态页 / 文件列表页”）。
* 主按键：

  * CLICK（短按）：

    * `GPX_IDLE`：开始记录，状态切换到 `GPX_RECORDING`。
    * `GPX_RECORDING`：暂停记录，状态切换到 `GPX_PAUSED`。
    * `GPX_PAUSED`：继续记录，状态切换到 `GPX_RECORDING`。
  * DOUBLE：在当前轨迹上打一个 waypoint（打点），不改变状态。
  * MID：保留（可用于切换显示布局，当前不定义具体行为）。
  * LONG：进入设置页面 `SETTINGS_IN_MODE`（来源为 GPX）。在该设置页面中提供菜单项“停止记录并保存文件”，由用户确认后触发状态从 RECORDING/PAUSED -> STOPPED -> IDLE。
  * ULTRA：保留。

> 说明：长按在 GPX 模式**统一用于进入设置**，停止/保存由设置页面中的显式菜单项完成，不直接由长按触发。

---

#### 2.3.4 P-GEAR 模式（MODE_P_GEAR）

* 旋转编码器：

  * 在 P-GEAR 中切换“实时测试页 / 历史成绩页”等视图。
* 主按键：

  * CLICK（短按）：

    * 在 `PGEAR_IDLE`：清除本次测试的临时数据（当前 run 的起止时间/用时等），保持历史最佳成绩，状态仍为 `PGEAR_IDLE`。
    * 在 `PGEAR_FINISHED`：清除本次测试结果，状态切换回 `PGEAR_IDLE`，保留历史最佳成绩。
    * 在 `PGEAR_ARMED` / `PGEAR_RUNNING`：默认无操作（可在 UI 中提示“测试进行中，短按无效”）。
  * DOUBLE：

    * 从任意非设置状态（IDLE / ARMED / RUNNING / FINISHED）中，将 P-GEAR 状态机**强制复位到 `PGEAR_IDLE` 开始新测试**，保留历史最佳成绩。
  * MID：保留（可作为未来扩展）。
  * LONG：进入设置页面 `SETTINGS_IN_MODE`（来源为 P-GEAR）。
  * ULTRA：保留，用于未来紧急操作。

> 清除历史最佳成绩应作为设置页面中的单独菜单项，而非通过短按/双击触发。

---

## 3. 时间与 RTC 系统

1. 软件维护一个 RTC（实时时钟）。
2. 上电首次启动时，RTC 默认时间为编译时间 `__DATE__` + `__TIME__`。
3. 设置页面提供“时间设置”，允许用户手动设置年月日时分秒。
4. 时间源选项：

   * GNSS 自动同步
   * 仅手动设置
5. GNSS 有效定位且时间有效时，若时间源为 GNSS 自动同步：

   * 自动同步 RTC。
   * **首次成功同步时，在 UI 上显示“时间同步完成”指示 2 秒。**
6. 所有 GPX `<time>` 时间戳必须统一使用 **UTC 时间**，格式为 `YYYY-MM-DDThh:mm:ssZ`，由 RTC 时间转换获得。
7. 定义 `timestamp_ms`：

   * 所有结构体中的 `timestamp_ms` 统一为“自系统上电以来的毫秒数”，来源为系统 Tick（例如 `esp_timer_get_time()/1000`）。
   * `timestamp_ms` 为 `uint32_t`，约每 49.7 天回绕，使用时必须考虑回绕；与 RTC 时间的转换由公共时间模块 `rtc_*` 负责。

---

## 4. GNSS（NEO-M8N）需求

### 4.1 初始化 & 配置流程

1. 拉高 GPS_LDO_EN（GPIO14），延时 ≥ 100 ms。
2. UART1 以 9600 bps 接收 NMEA。
3. 通过 UBX 指令配置波特率为 115200 bps，并保存配置。
4. 重新初始化 UART1 为 115200 bps。
5. 星座模式和更新率由设置界面控制：

   * 星座模式：

     * GPS
     * GPS + BeiDou
     * GPS + GLONASS
   * 更新率：

     * 1 Hz
     * 5 Hz
6. 至少启用 GGA、RMC；建议结合 UBX 消息用于 DOP 信息提取与卫星状态列表。

### 4.2 数据结构

```c
typedef struct {
    uint32_t timestamp_ms;  // 自上电以来的毫秒数

    double   lat;      // 纬度，度
    double   lon;      // 经度，度
    float    alt;      // GNSS 高度，m
    float    speed;    // 水平速度，m/s
    float    course;   // 航向，度 0~360
    float    hdop;     // 水平精度
    float    vdop;     // 垂直精度
    float    pdop;     // 位置精度
    uint8_t  sats;     // 使用卫星数
    uint8_t  fix;      // 0: 无 / 2: 2D / 3: 3D ...
    bool     valid;    // 定位是否有效（按下述规则判定）
} gnss_fix_t;
```

### 4.3 状态判断与 NMEA_OK / 时间有效

* `valid` 判定建议：

  * `fix >= 2` 且 `hdop` 在合理范围（例如 `hdop < 5`）时，认为 `valid = true`。
* 定义 `NMEA_OK`：

  * 在最近 5 秒内接收到并成功解析 ≥ 1 条有效 GGA 或 RMC，则 `NMEA_OK = 1`；否则 0。
* 定义“时间有效”：

  * 最近一次解析成功的 RMC/GGA 中：

    * RMC Status = 'A'（有效）
    * 日期和时间字段不为默认值（比如 1980-01-06 等厂商缺省时间）
  * 满足上述条件即认为 GNSS 时间有效，可用于同步 RTC。
* 心跳日志中必须包含 `NMEA_OK` 状态。
* 当 `valid = false` 时，lat/lon/alt 可以维持最后一次有效值，但需在日志中明确 `valid=0`，以区分当前不在定位状态。

---

## 5. 传感器系统（IMU / MAG / BARO）

### 5.1 坐标系约定

定义统一车体坐标系：

* X：前进方向
* Y：左侧
* Z：向上

所有传感器数据必须转换到该坐标系下。

### 5.2 IMU（LSM6DSR）

* 接口：I2C 地址 0x6A
* 配置：

  * ODR：104 Hz
  * 加速度量程：±4g
  * 陀螺仪量程：±1000 dps
* 轴变换：**X 和 Z 轴反向，Y 轴不变**：

```c
ax = -ax_raw;
ay =  ay_raw;
az = -az_raw;

gx = -gx_raw;
gy =  gy_raw;
gz = -gz_raw;
```

* 输出数据要求：

  * 含重力加速度：ax, ay, az（m/s²，包含重力分量）
  * 线性加速度：ax_lin, ay_lin, az_lin（m/s²，尽量减去重力，可用简单滤波或姿态算法）
  * 角速度：gx, gy, gz（dps 或 rad/s，需在接口中统一说明）
  * 重力向量（如实现姿态算法）：gx_grav, gy_grav, gz_grav（m/s²），未实现时可置 0 或 NAN 并在注释中说明
  * 温度：imu_temp_c（°C）

### 5.3 磁力计（LIS2MDL）

* 接口：I2C 地址 0x1E
* ODR ≥ 20 Hz
* 有轴交换和方向反转（具体矩阵由后续标定给出，软件中必须保留 3×3 变换矩阵 + 偏移）。
* 提供软铁/硬铁校准过程（设置页面入口）。
* 若芯片支持温度，读取 mag_temp_c；否则字段保留，使用 NAN 或无效标记。

### 5.4 气压计（BMP388）

* 接口：I2C 地址 0x76
* 模式：Normal
* Pressure OSR ×4，Temp OSR ×1，IIR ≥ 3，ODR ≥ 25 Hz。
* 输出：

  * pressure（Pa，内部统一单位）
  * baro_temp_c（°C）
  * altitude（m，通过标准公式计算）

> 日志/UI 要求：气压在心跳日志与 UI 中以 kPa 显示（例如 101.5 kPa），由内部 Pa 数值 /1000 转换。

### 5.5 GNSS 辅助气压计校准

* 条件：`fix = 3D` 且 `hdop < 2`，`valid = true`。
* 根据当前 GNSS alt 与气压 altitude 反算海平面气压 P0，用于后续高度计算。
* 校准过程需平滑处理，避免瞬态 GNSS 噪声导致高度突变（可以使用滑动平均 / 指数滤波）。

### 5.6 统一传感器数据结构

```c
typedef struct {
    uint32_t timestamp_ms;   // 自上电以来毫秒

    // IMU
    float ax, ay, az;        // 含重力加速度，m/s²
    float ax_lin, ay_lin, az_lin; // 线性加速度，m/s²
    float gx, gy, gz;        // 角速度
    float gx_grav, gy_grav, gz_grav; // 重力向量估计，未实现可置 0
    float imu_temp_c;        // °C

    // MAG
    float mx, my, mz;        // 磁场原始或校准值
    float mag_temp_c;        // °C（不支持时置 NAN）

    // BARO
    float pressure;          // Pa
    float altitude;          // m
    float baro_temp_c;       // °C
} sensor_sample_t;
```

---

## 6. 轨迹记录（GPX Recorder，MODE_GPX）

### 6.1 文件系统

* 文件系统：FATFS
* 挂载后检查 `/GPX/` 目录，不存在则创建。
* 文件命名：`YYYYMMDD_HHMMSS.gpx`（记录开始时刻，使用 RTC 时间）。

### 6.2 GPX 内容结构

GPX 1.1，包含 `<trk>` / `<trkseg>` / `<trkpt>`，扩展字段包含速度/航向/气压/温度等，例如：

```xml
<trkpt lat=".." lon="..">
  <ele>..</ele>
  <time>2024-03-15T10:30:00Z</time>
  <extensions>
    <speed>..</speed>        <!-- m/s -->
    <course>..</course>      <!-- deg -->
    <pressure>..</pressure>  <!-- Pa -->
    <temperature>..</temperature> <!-- °C (baro_temp_c) -->
  </extensions>
</trkpt>
```

* `<time>` 字段使用 RTC 转换的 **UTC 时间**。
* 气压以 Pa 写入（与内部一致）。

### 6.3 写点策略

在 `GPX_RECORDING` 状态下，在任一时刻满足以下条件之一，即写入一个轨迹点：

1. 距离上一个点时间间隔 ≥ 1 s；
2. 与上一个点的水平距离差 ≥ 5 m。

### 6.4 暂停与分段

* 手动暂停或自动暂停时，关闭当前 `<trkseg>`。
* 恢复记录时，新建一个 `<trkseg>`。
* 停止记录时，关闭所有标签，安全关闭文件。

---

## 7. 自行车码表（Bike Computer，MODE_BIKE）

### 7.1 实时数据（至少包括）

* 当前速度（km/h）
* 平均速度
* 最大速度
* 当次骑行时间 / 总时间
* 当前里程 / 累计里程
* 当前高度 / 高度趋势
* 坡度（%）
* 累计爬升 / 累计下降
* 航向（GNSS + MAG）
* 垂直速度（m/h）：可按最近 10–30 s 内的高度变化做滑动平均，优先使用气压高度。

### 7.2 Lap（圈）

* 使用按键双击触发打圈。
* 每个圈记录：

  * Lap 距离
  * Lap 时间
  * Lap 平均速度
* 在专用页面或数据页中查看当前 Lap 状态。

### 7.3 自动暂停（Auto Pause）

* 当速度 < `V_pause_threshold` 并保持时间 > `T_pause_delay` 时自动暂停。
* 当速度重新 > `V_resume_threshold` 时恢复。
* Auto Pause 参数在设置中配置，并同时用于 BIKE 与 GPX 模式（统一速度门限，便于用户理解）。

### 7.4 自动分圈（Auto Lap）

* 根据距离自动分圈，例如每 5 km / 10 km（用户可配置或关闭）。
* 在设置中配置 Auto Lap 距离阈值。

---

## 8. P-GEAR（0–100 加速测试，MODE_P_GEAR）

### 8.1 功能定位

* 模拟汽车 0–100 km/h 加速测试。
* 支持自定义起始/结束速度范围。

### 8.2 设置项（在设置页面中）

* 起始速度：`V_start`（km/h）
* 结束速度：`V_end`（km/h，必须 > V_start）
* 触发加速度阈值：`G_trigger`，**内部统一使用 m/s²**，UI 可显示成 g：

  * 配置界面中以 g 为单位输入，转换为 m/s² 存储；
  * 实时显示“当前加速度(G)”时以 g 显示。
* 触发速度阈值：`V_trigger`（km/h）

### 8.3 状态机逻辑摘要（P-GEAR）

* IDLE：参数已配置，等待车速接近起始速度。
* ARMED：速度接近 V_start，系统“武装”。
* RUNNING：当速度 ≥ V_start 且 加速度 ≥ G_trigger 时开始计时。
* FINISHED：当速度 ≥ V_end 时停止计时，记录成绩。
* 历史最佳成绩（best）单独保存，短按/重测不清除，需通过设置菜单中的显式选项“清除历史最佳成绩”来清除。

---

## 9. UI 设计（页面草图 + LVGL 映射）

### 9.1 主菜单（第一屏）

```text
+--------------------------------+
|           ESP32S3 BOX          |
|          (logo / 标题)         |
|--------------------------------|
|  > 自行车码表 (Bike)           |
|    GPS 轨迹记录仪 (GPX)        |
|    P-GEAR 加速测试             |
|    设置 (Settings)             |
|--------------------------------|
|  旋钮：上下选择                 |
|  短按：进入                     |
+--------------------------------+
```

* LVGL 建议：

  * 根容器：`lv_obj`
  * 标题：`lv_label`
  * 菜单：`lv_list` 或一列 `lv_btn` + `lv_label`
  * 高亮项使用 `LV_STATE_FOCUSED`/自定义样式

### 9.2 BIKE 主数据页（Page 1）

```text
+--------------------------------+
| 速度(km/h)                     |
| [    28.5    ]                 |
|--------------------------------|
| 距离(km)       骑行时间        |
| [  12.34 ]    [ 00:35:21 ]     |
|--------------------------------|
| 海拔(m)        坡度(%)         |
| [   135  ]    [   6.5   ]      |
|--------------------------------|
| ENC：切换页  短按：菜单        |
| 双击：打圈   长按：设置        |
+--------------------------------+
```

* LVGL：每个数据块用 `lv_obj` + 2 个 `lv_label`（标题+数值）

### 9.3 BIKE 扩展数据页（Page 2）

```text
+--------------------------------+
| 平均速(km/h)   最大速(km/h)    |
| [  23.1  ]     [   46.3 ]      |
|--------------------------------|
| 爬升(m)         下降(m)        |
| [  520   ]     [   430  ]      |
|--------------------------------|
| 航向(°)        垂直速度(m/h)   |
| [  123  ]      [   520  ]      |
|--------------------------------|
| ENC：切页    短按：菜单        |
| 双击：打圈   长按：设          |
+--------------------------------+
```

### 9.4 GPX 轨迹记录页面

```text
+--------------------------------+
| GPS 轨迹记录仪                 |
| 状态: [记录中] 文件数:[ 12 ]   |
|--------------------------------|
| 当前文件:                      |
| [ 20240315_103000.gpx      ]   |
|--------------------------------|
| 距离(km):   [ 12.35 ]          |
| 速度(km/h): [ 25.4  ]          |
| 高度(m):    [  128  ]          |
| 卫星数:     [  10   ] Fix:3D   |
|--------------------------------|
| 短按：开始/暂停 双击：打点      |
| 长按：设置                      |
+--------------------------------+
```

> 说明：长按进入的设置页面中提供“停止记录并保存文件”菜单项，可结束当前记录并返回 IDLE。

### 9.5 P-GEAR 页面

```text
+--------------------------------+
|       P-GEAR 0-100 测试        |
| 状态: [等待中/武装/计时中]     |
|--------------------------------|
| 当前速度(km/h): [  32.4 ]      |
| 当前加速度(G):  [  0.68 ]      |
|--------------------------------|
| 目标: [ 0  -> 100 km/h ]       |
| 本次用时: [  6.32 s ]          |
| 最佳成绩: [  5.98 s ]          |
|--------------------------------|
| ENC：切历史/实时  短按：清本次  |
| 双击：重测        长按：设置    |
+--------------------------------+
```

> “清本次”：清除当前 run 的临时结果，不影响历史最佳成绩。
> “重测”：强制复位状态机到 IDLE，准备下一次测试，同样不清除历史最佳成绩。

### 9.6 设置主界面

```text
+--------------------------------+
|            设置                |
|--------------------------------|
| > 时间设置（RTC）              |
|   GNSS 设置                    |
|   传感器校准                   |
|   自动暂停/自动分圈            |
|   显示与背光                   |
|   P-GEAR 设置                  |
|   外设状态                     |
|   GPS 搜星详细信息             |
|   Debug 调试页面               |
|   返回（回原功能）             |
|   退出到主菜单                 |
+--------------------------------+
```

### 9.7 设置 → 外设状态（来源于心跳日志）

```text
+--------------------------------+
|          外设状态              |
|--------------------------------|
| GNSS: fix=3D sats=10 NMEA_OK=1 |
|       lat=.. lon=.. alt=..     |
|--------------------------------|
| IMU: ax=.. ay=.. az=..         |
|      lin=.. gx=.. gy=.. gz=..  |
|      temp=..°C                 |
|--------------------------------|
| MAG: mx=.. my=.. mz=..         |
|      temp=..°C                 |
|--------------------------------|
| BARO: p=101.5kPa alt=..m       |
|       temp=..°C                |
|--------------------------------|
| SD: mounted=1 err=0            |
| BATT: 3.92V chg=0              |
|--------------------------------|
| 短按：返回  长按：退出到主菜单 |
+--------------------------------+
```

### 9.8 设置 → GPS 搜星详细信息

```text
+--------------------------------+
|         GPS 搜星状态           |
|--------------------------------|
| Fix: 3D  HDOP:0.9  PDOP:1.5    |
| 卫星: 使用10 / 可见14          |
|--------------------------------|
| PRN | SNR | Use | SYS | El |Az |
|  03 | 35  |  *  | G   |45 |120 |
|  08 | 42  |  *  | G   |60 |200 |
|  11 | 18  |     | B   |20 | 80 |
| ...                            |
|--------------------------------|
| (可选底部 SNR 柱状图)          |
| 短按：返回                     |
+--------------------------------+
```

---

## 10. 状态机定义（Mermaid）

### 10.1 顶层模式状态机（主菜单 + 模式 + 设置）

```mermaid
stateDiagram-v2
    [*] --> MAIN_MENU

    MAIN_MENU --> MODE_BIKE:  选择"自行车码表"+短按
    MAIN_MENU --> MODE_GPX:   选择"GPS轨迹记录仪"+短按
    MAIN_MENU --> MODE_PGEAR: 选择"P-GEAR"+短按
    MAIN_MENU --> SETTINGS_ROOT:选择"设置"+短按

    MODE_BIKE --> SETTINGS_IN_MODE: 长按键
    MODE_GPX  --> SETTINGS_IN_MODE: 长按键
    MODE_PGEAR--> SETTINGS_IN_MODE: 长按键

    SETTINGS_ROOT --> MAIN_MENU: 选择"退出到主菜单"

    SETTINGS_IN_MODE --> MAIN_MENU: 选择"退出到主菜单"
    SETTINGS_IN_MODE --> MODE_BIKE: 返回且来源=BIKE
    SETTINGS_IN_MODE --> MODE_GPX:  返回且来源=GPX
    SETTINGS_IN_MODE --> MODE_PGEAR:返回且来源=P-GEAR
```

> LONG（长按）进入 `SETTINGS_IN_MODE` 仅在三大功能模式页面（MODE_BIKE / MODE_GPX / MODE_PGEAR）中有效；在主菜单（MAIN_MENU）中长按无动作（保留）。

### 10.2 GPX 记录状态机

```mermaid
stateDiagram-v2
    [*] --> GPX_IDLE

    GPX_IDLE --> GPX_RECORDING: 短按开始记录
    GPX_RECORDING --> GPX_PAUSED:  短按暂停
    GPX_PAUSED --> GPX_RECORDING:  短按继续

    GPX_RECORDING --> GPX_STOPPED: 设置菜单中选择"停止记录并保存"
    GPX_PAUSED --> GPX_STOPPED:    设置菜单中选择"停止记录并保存"

    GPX_STOPPED --> GPX_IDLE: 文件关闭, 清理状态
```

> 从 GPX 主界面长按进入 `SETTINGS_IN_MODE`，在该设置页中包含“停止记录并保存”菜单项，用户选择该项后触发上述状态转移。

### 10.3 P-GEAR 测试状态机

```mermaid
stateDiagram-v2
    [*] --> PGEAR_IDLE

    PGEAR_IDLE: 等待车速接近V_start
    PGEAR_ARMED: 已武装
    PGEAR_RUNNING: 计时中
    PGEAR_FINISHED: 已完成

    PGEAR_IDLE --> PGEAR_ARMED: 速度接近V_start
    PGEAR_ARMED --> PGEAR_RUNNING: speed>=V_start 且 G>=G_trigger
    PGEAR_RUNNING --> PGEAR_FINISHED: speed>=V_end

    PGEAR_FINISHED --> PGEAR_IDLE: 短按清除本次结果
    PGEAR_IDLE --> PGEAR_IDLE: 短按清除本次临时数据（保持在IDLE）

    PGEAR_ARMED --> PGEAR_IDLE: 双击重测（强制复位）
    PGEAR_RUNNING --> PGEAR_IDLE: 双击重测（强制复位）
    PGEAR_FINISHED --> PGEAR_IDLE: 双击重测（强制复位）
```

> “短按清除本次”不清除历史最佳成绩，双击重测同样不清除，历史最佳成绩通过设置菜单显式清除。

### 10.4 BIKE 自动暂停 / Lap 状态机

```mermaid
stateDiagram-v2
    [*] --> BIKE_RIDING

    BIKE_RIDING --> BIKE_AUTO_PAUSE: 速度<阈值 且保持>T_pause
    BIKE_AUTO_PAUSE --> BIKE_RIDING: 速度>恢复阈值

    BIKE_RIDING --> BIKE_RIDING: 双击按键 -> 打圈(Lap++)
```

### 10.5 设置菜单导航（简化）

```mermaid
stateDiagram-v2
    [*] --> SETTINGS_MAIN

    SETTINGS_MAIN --> SET_TIME:        选"时间设置"
    SETTINGS_MAIN --> SET_GNSS:        选"GNSS 设置"
    SETTINGS_MAIN --> SET_SENSOR_CAL:  选"传感器校准"
    SETTINGS_MAIN --> SET_AUTO_CTRL:   选"自动暂停/自动分圈"
    SETTINGS_MAIN --> SET_DISPLAY:     选"显示与背光"
    SETTINGS_MAIN --> SET_PGEAR:       选"P-GEAR 设置"
    SETTINGS_MAIN --> SET_DEV_STATUS:  选"外设状态"
    SETTINGS_MAIN --> SET_GPS_SKYVIEW: 选"GPS 搜星详细信息"
    SETTINGS_MAIN --> SET_DEBUG:       选"Debug 调试"

    SET_TIME        --> SETTINGS_MAIN: 返回
    SET_GNSS        --> SETTINGS_MAIN: 返回
    SET_SENSOR_CAL  --> SETTINGS_MAIN: 返回
    SET_AUTO_CTRL   --> SETTINGS_MAIN: 返回
    SET_DISPLAY     --> SETTINGS_MAIN: 返回
    SET_PGEAR       --> SETTINGS_MAIN: 返回
    SET_DEV_STATUS  --> SETTINGS_MAIN: 返回
    SET_GPS_SKYVIEW --> SETTINGS_MAIN: 返回
    SET_DEBUG       --> SETTINGS_MAIN: 返回

    SETTINGS_MAIN --> [*]: 选"退出到主菜单"
```

---

## 11. 日志与心跳（UART0）

### 11.1 心跳日志（每 5 秒）

内容必须包含（可多行输出）：

* 时间 `t`（使用 `timestamp_ms`）
* 当前模式 `mode`
* GNSS：fix/sats/hdop/vdop/pdop/lat/lon/alt/NMEA_OK/valid
* IMU：ax/ay/az/ax_lin/ay_lin/az_lin/gx/gy/gz/imu_temp_c
* MAG：mx/my/mz/mag_temp_c
* BARO：pressure（以 kPa 输出）、altitude、baro_temp_c
* SD：mounted / last_err
* 电池：电压、充电状态

示例：

```text
[HB] t=123456ms mode=MODE_BIKE
     gnss: fix=3D valid=1 sats=10 hdop=0.9 vdop=1.2 pdop=1.5
           lat=39.9845 lon=116.3185 alt=201.3m NMEA_OK=1
     imu: ax=-0.02 ay=0.01 az=9.80 lin_ax=0.10 ... temp=32.5
     mag: mx=12.3 my=-5.6 mz=30.1 temp=28.0
     baro: p=101.5kPa alt=123.4m temp=25.6C
     sd: mounted=1 err=0
     batt: volt=3.92V chg=0
```

> pressure 内部单位为 Pa，输出心跳日志时转换为 kPa 显示。

### 11.2 输入事件日志（即时）

每次按键/旋转编码器事件触发时立即输出，例如：

```text
[EVT] t=234567ms mode=MODE_BIKE type=BTN_DOUBLE_CLICK
[EVT] t=234890ms mode=MODE_BIKE type=ENC_RIGHT
```

---

## 12. FreeRTOS 任务与通信

### 12.1 建议任务列表

| 任务名            | 功能              | 优先级（示例） |
| -------------- | --------------- | ------- |
| GNSS_TASK      | GNSS UART 收发解析  | 7       |
| SENSOR_TASK    | IMU/MAG/BARO 采样 | 6       |
| UI_TASK        | LVGL 刷新与渲染      | 5       |
| LOG_TASK       | SD/GPX 写入与管理    | 5       |
| INPUT_TASK     | 按键/旋钮采样与事件识别    | 4       |
| HEARTBEAT_TASK | 心跳与日志输出         | 4       |
| SYS_TASK       | 系统状态机与模式管理      | 6       |
| POWER_TASK     | 电池电量测量与电源策略     | 4       |

### 12.2 通信机制

* 队列：

  * `gnss_event_queue`：GNSS_TASK → SYS/UI/LOG
  * `sensor_event_queue`：SENSOR_TASK → SYS/UI
  * `input_event_queue`：INPUT_TASK → SYS/UI
* 互斥量：

  * `fs_mutex`：所有 FATFS 操作
  * `config_mutex`：NVS 配置读写

---

## 13. 编码规范（对 AI 的硬性要求）

1. 所有公共函数必须在 `.h` 中声明，并只在一个 `.h` 中声明一次。
2. 所有模块内部函数必须使用 `static` 限定。
3. 禁止不同 `.c` 文件存在同名非 static 函数。
4. 所有头文件必须使用 include guard。
5. 注释必须是中文，说明函数用途、参数、返回值及注意事项。
6. 模块命名建议：

   * `gnss_*`, `sensor_*`, `pgear_*`, `gpx_*`, `ui_*`, `input_*`, `log_*`, `rtc_*` 等。
7. 函数应短小，避免深层嵌套，除主循环外尽量非阻塞。

---

