idf_component_register(SRCS "gnss.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES driver freertos esp_timer esp_common)
