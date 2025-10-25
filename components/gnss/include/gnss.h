#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
} gnss_fix_t;

esp_err_t gnss_init(void);
bool gnss_is_present(void);
int gnss_get_tracked_satellites(void);
void gnss_get_last_fix(gnss_fix_t *fix);
