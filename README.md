# ESP32-S3FH4R2 — GNSS + SPI LCD + 9轴 + 电池 项目

简介
- MCU：ESP32-S3FH4R2
- 功能：集成 GNSS (MAX-F10S / ATGM336H)、SPI LCD（背光 PWM 控制）、传感器 (LSM6DSR(0xD5)、LIS2MDL(0x3D)、BMP388(0xED))、旋转编码器（含按键）、电池电压 ADC 与充电状态管理
- 代码按模块拆分，便于开发与调试：components/{board,sensors,gnss,lcd_spi,encoder} + main + scripts

硬件 GPIO 映射
- GNSS UART (UART1)：TX=GPIO17, RX=GPIO18, 115200bps
- Display SPI (SPI3)：SCK=GPIO5, MOSI=GPIO8, CS=GPIO7, DC=GPIO6, RST=GPIO4, BL=GPIO9 (LEDC)
- I2C (I2C_NUM_0)：SCL=GPIO39, SDA=GPIO40
- 传感器中断：ACCGYRO_INT=GPIO41, MAG_INT=GPIO42, PRESS_INT=GPIO13
- 编码器：ENC_A=GPIO1, ENC_B=GPIO3；按键：GPIO2（低电平有效，启用上拉，去抖 100ms）
- GPS 电源控制：GPS_LDO_EN=GPIO14
- 电池 ADC：GPIO12 (ADC2CH1)
- 充电状态：CHRG_STATUS=GPIO21
- DEBUG UART0：Tx=GPIO43, Rx=GPIO44, 115200bps

启动/日志行为
- 调试日志通过 USB/UART0（GPIO43/44）输出，默认 INFO 级别。
- 启动后前 5 秒内（每 10s）输出完整状态：
  - GNSS 状态：工作模式、追踪卫星数、当前位置；若 GNSS 通信超时/模块不可达，打印 "GNSS not present"。
  - 传感器状态：LSM6DSR / LIS2MDL / BMP388 存在性与 IC 温度（若可读）；若 I2C 设备检测失败，打印 "XXX not present"。
  - 电池电压（V）与 CHRG_STATUS 电平。
- 之后为周期性心跳日志。
- 若 GNSS 或传感器通信超时/检测失败，日志会明确输出 "xxxx not present"。

如何构建（快速参考）
- 要求：已安装 ESP-IDF（建议 v4.4+ 或 v5+）并配置好工具链。
- 快速构建和烧录（在项目根目录运行）：
  1. source $HOME/esp/esp-idf/export.sh    # 或者按你安装时的路径
  2. idf.py set-target esp32s3
  3. idf.py build
  4. idf.py -p /dev/ttyUSB0 flash monitor   # 将串口替换为你的设备

项目包含一个 scripts/setup_env.sh 帮助脚本，说明了上述步骤和常见提示。

按钮与编码器行为
- 编码器旋转事件即时打印日志。
- 按键去抖：100 ms。
- 按键三档判定（按用户要求）：
  - 短按：持续 < 1s
  - 中按：持续 >= 3s 且 < 8s
  - 长按：持续 >= 8s

旋转编码器硬件实现建议（PCNT）
- 可使用 ESP32 的 PCNT 外设：把 A 做脉冲输入、B 做控制输入，在脉冲沿时依据 control 电平决定增/减，实现硬件正交解码
- 优点：CPU 负担小、抗抖好；需处理计数溢出与复位等边界情况
- 可提供 PCNT 初始化示例以替换 ISR 实现