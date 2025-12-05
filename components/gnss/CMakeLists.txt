idf_component_register(
    SRCS "gnss_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos common data_model
)
