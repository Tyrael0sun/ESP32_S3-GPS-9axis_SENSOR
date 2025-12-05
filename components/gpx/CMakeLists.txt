idf_component_register(
    SRCS "gpx_recorder.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos vfs fatfs sdmmc common data_model logger esp_driver_sdmmc
)
