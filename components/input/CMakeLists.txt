idf_component_register(
    SRCS "input_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos common logger esp_timer esp_driver_gpio
)
