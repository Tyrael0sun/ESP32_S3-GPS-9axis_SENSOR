// sensor_types.h
// 说明：传感器相关类型定义

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float m_matrix[9];   // 3x3 软铁矩阵
    float m_offset[3];   // 3x1 硬铁偏移
} mag_calib_t;

typedef struct {
    float kp;            // Mahony 比例增益
    float ki;            // Mahony 积分增益
} mahony_param_t;
