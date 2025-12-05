idf_component_register(
    SRCS "imu_lsm6dsr.c" "mag_lis2mdl.c" "baro_bmp388.c" "sensor_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos common i2c_bus data_model
)
