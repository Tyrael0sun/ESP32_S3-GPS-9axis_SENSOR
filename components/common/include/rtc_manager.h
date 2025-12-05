// rtc_manager.h
// 说明：软件 RTC 管理接口

#pragma once

#include <time.h>

void rtc_manager_init(void);
void rtc_manager_set_utc(time_t utc_ts, bool synced);
time_t rtc_manager_now(void);
void rtc_manager_poll(void);
