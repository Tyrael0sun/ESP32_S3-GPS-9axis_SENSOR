// gpx_recorder.h
// 说明：GPX 轨迹记录模块接口

#pragma once

#include "esp_err.h"
#include "data_model.h"

esp_err_t gpx_recorder_start(void);
esp_err_t gpx_recorder_begin_record(void);
esp_err_t gpx_recorder_pause(void);
esp_err_t gpx_recorder_resume(void);
esp_err_t gpx_recorder_stop(void);
esp_err_t gpx_recorder_push_point(const gnss_fix_t *gnss,
								  const sensor_sample_t *sensor);
