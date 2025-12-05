// rtc_manager.c
// 说明：软件 RTC 以 esp_timer 为基础进行时间推算，并同步至 DataModel

#include "rtc_manager.h"
#include "data_model.h"
#include "esp_timer.h"
#include <string.h>
#include <stdlib.h>

static int64_t s_offset_us = 0;
static bool s_synced = false;

static time_t parse_compile_time(void)
{
    struct tm tm_time = {0};
    const char *date = __DATE__;
    const char *time_str = __TIME__;
    static const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";

    char month_str[4] = {0};
    memcpy(month_str, date, 3);
    int month = 0;
    for (int i = 0; i < 12; ++i) {
        if (strncmp(month_str, months + i * 3, 3) == 0) {
            month = i;
            break;
        }
    }
    tm_time.tm_mon = month;
    tm_time.tm_mday = atoi(date + 4);
    tm_time.tm_year = atoi(date + 7) - 1900;
    tm_time.tm_hour = atoi(time_str);
    tm_time.tm_min = atoi(time_str + 3);
    tm_time.tm_sec = atoi(time_str + 6);
    return mktime(&tm_time);
}

static void update_offset(time_t utc)
{
    s_offset_us = (int64_t)utc * 1000000LL - (int64_t)esp_timer_get_time();
}

void rtc_manager_init(void)
{
    time_t utc = parse_compile_time();
    update_offset(utc);
    s_synced = false;
    rtc_manager_poll();
}

void rtc_manager_set_utc(time_t utc_ts, bool synced)
{
    update_offset(utc_ts);
    s_synced = synced;
    rtc_manager_poll();
}

time_t rtc_manager_now(void)
{
    int64_t now_us = (int64_t)esp_timer_get_time() + s_offset_us;
    if (now_us < 0) {
        now_us = 0;
    }
    return (time_t)(now_us / 1000000LL);
}

void rtc_manager_poll(void)
{
    app_data_model_t *m = data_model_get();
    m->rtc.unix_time = rtc_manager_now();
    m->rtc.synced = s_synced;
}
