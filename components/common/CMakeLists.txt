idf_component_register(
    SRCS "rtc_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
    PRIV_REQUIRES data_model
)
