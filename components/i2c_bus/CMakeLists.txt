idf_component_register(
    SRCS "i2c_bus.c"
    INCLUDE_DIRS "include"
    REQUIRES driver common esp_driver_i2c
)
