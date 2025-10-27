idf_component_register(
    SRCS "sensors.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES board esp_driver_i2c freertos esp_common
)
