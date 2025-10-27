idf_component_register(SRCS "board.c"
                       INCLUDE_DIRS "include"
                       REQUIRES hal esp_driver_i2c
                       PRIV_REQUIRES esp_driver_gpio esp_driver_i2c esp_adc freertos)
