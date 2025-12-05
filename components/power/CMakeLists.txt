idf_component_register(
    SRCS "power_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_adc freertos common data_model esp_driver_gpio
)
